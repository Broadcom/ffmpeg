/*
 * Copyright (c) 2018 Broadcom
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * scale video filter
 */

#include <stdio.h>
#include <string.h>

#include "avfilter.h"
#include "bufferqueue.h"
#include "formats.h"
#include "internal.h"
#include "scale_eval.h"
#include "video.h"
#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/hwcontext_vkapi.h"
#include "libavutil/internal.h"
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libavutil/parseutils.h"
#include "libavutil/pixdesc.h"
#include "libswscale/swscale.h"

#define MAX_OUTPUTS 4 //<! max number of scaler output

    // normally, we should get the pool size from HW, but at the time to init the ffmpeg structure
    // with the HW pool size, The HW scaler is not yet initialzed (so the buffer pool not yet available)

    // So we provide to ffmpeg the minimum pool size (4 is also the hardcoded value used by the vaapi implementation)
    // However, ffmpeg seems to ignore this value: that is, it will call the filter, even when no hw frames are
    // available (having the scaler working 32 frames ahead of the encoder is a fairly frequent occurrence).
    // that in practice one need to be prepared to have much more HW frames than specified and still face
    // the case where the conversion will fail due to lack of available HW frames.
#define VK_SCL_INITIAL_POOL_SIZE 4

/* values for the flags, the stuff on the command line is different */

#define VK_CHROMA_OFFSET 16

#define VK_BILINEAR  ((VK_SCL_FLTR_LINEAR << VK_CHROMA_OFFSET)  | VK_SCL_FLTR_LINEAR)
#define VK_BICUBIC   ((VK_SCL_FLTR_CUBIC << VK_CHROMA_OFFSET)   | VK_SCL_FLTR_CUBIC)
#define VK_NEAREST   ((VK_SCL_FLTR_NEAREST << VK_CHROMA_OFFSET) | VK_SCL_FLTR_NEAREST)
#define VK_BICUBLIN  ((VK_SCL_FLTR_LINEAR << VK_CHROMA_OFFSET)  | VK_SCL_FLTR_CUBIC)
#define VK_CATMULL   ((VK_SCL_FLTR_CATMULL << VK_CHROMA_OFFSET) | VK_SCL_FLTR_CATMULL)
#define VK_CUSTOM    ((VK_SCL_FLTR_CUSTOM << VK_CHROMA_OFFSET)  | VK_SCL_FLTR_CUSTOM)

static const enum AVPixelFormat vkapi_pix_fmts[] = {
    AV_PIX_FMT_VKAPI,
    AV_PIX_FMT_NV12,
    AV_PIX_FMT_NV21,
    AV_PIX_FMT_P010,
    AV_PIX_FMT_NONE
};

typedef struct VKAPIVarsHeader {
    FILE * file;
    int32_t size;
    int32_t width;
    int32_t height;
} VKAPIVarsHeader;

typedef struct VKAPIScaleContext {
    const AVClass *class;

    AVDictionary *opts;

    /**
     * New dimensions. Special values are:
     *   0 = original width/height
     *  -1 = keep original aspect
     *  -N = try to keep aspect but make sure it is divisible by N
     */

    char    *w_expr[MAX_OUTPUTS]; //!< width  expression string
    char    *h_expr[MAX_OUTPUTS]; //!< height expression string
    int32_t filter_flags;
    int32_t out_pixel_format;
    char    *varsfile; //!< variance filename prefix
    int32_t vars_size[MAX_OUTPUTS + 1];
    void    *vars_data[MAX_OUTPUTS + 1];
    FILE    *vars[MAX_OUTPUTS + 1];
    char    *customfile;
    FILE    *custom;
    int qpvars;
    int is_inited;
    int noutputs;
    /* HW related field */
    vkil_context *ilctx; //!< the vk context for the encoder
    AVBufferRef  *hw_frames_ctx;
    AVBufferRef  *hwdevice; //!< allow a reference to the hw device
    AVBufferRef *hw_device_ctx;
    AVHWDeviceContext *hwdevice_ctx;
    VKAPIDeviceContext *devctx; //!< handy shortcut used to the hwdevice field
    struct FFBufQueue in_queue;
    struct FFBufQueue wait_queues[MAX_OUTPUTS];
    struct FFBufQueue out_queues[MAX_OUTPUTS];
    int32_t max_lag; //!< max allowed lag in frame between feeding filter, and filter completion
    int flush; // put the scaler in flush mode (finish processing)
    AVFrame *frame; //!< frame to be filtered
    /* for debugging purpose */
    int received_frames;
    int send_frames;
} VKAPIScaleContext;

/**
 * return the pixel sofware format associated with the AVFilterLink
 * @param the inlink
 * @return an AVPixelFormat if positive, error code otherwise
 */
static int get_sw_format(AVFilterLink *inlink)
{
    AVHWFramesContext *hwframes_ctx;

    if (inlink->format == AV_PIX_FMT_VKAPI) {
        if (inlink->hw_frames_ctx) {
            hwframes_ctx = (AVHWFramesContext*)inlink->hw_frames_ctx->data;
            return hwframes_ctx->sw_format;
        } else {
            av_log(inlink, AV_LOG_ERROR, "HW format %d infers a defined hw_frames_ctx\n", inlink->format);
            goto fail;
        }
    } else {
        return inlink->format;
    }

fail:
    return AVERROR(EINVAL);
}

static int vkapi_warnings_retrieve(AVFilterContext *avctx, const char * const caller)
{
    vk_warning warn;
    VKAPIScaleContext *ctx;
    int ret = 0, cnt = 0;

#define VK_FW_MAX_WARNINGS_TOPRINT 12
    // this may be called at early init failure where not much is setup up, so
    // do check individual ptrs progressively before continue.
    if (!avctx || !avctx->priv)
        return ret;
    ctx = avctx->priv;

    if (ctx->ilctx && ctx->ilctx->devctx) {
        while (cnt++ < VK_FW_MAX_WARNINGS_TOPRINT) {
            // get verbose hw error only when the ilctx has effectively been created
            ret = ctx->devctx->ilapi->get_parameter(ctx->ilctx, VK_PARAM_WARNING,
                                                    &warn, VK_CMD_OPT_BLOCKING);
            if ((ret) || (warn.log[0] == '\0'))
                break;

            av_log(avctx, AV_LOG_WARNING, "[FW-WARN]%s:<%s>\n", caller, warn.log);
        }
    }

    return ret;
}

static int vkapi_error_handling(AVFilterContext *avctx, int ret, const char * const caller)
{
    vk_error error;
    VKAPIScaleContext *ctx;

    /* sanity check */
    av_assert0(avctx);
    av_assert0(avctx->priv);

    ctx = avctx->priv;

    // retrieve warnings
    vkapi_warnings_retrieve(avctx, caller);

    if ((ret == -EADV) && ctx->ilctx) {
        // get verbose hw error only when the ilctx has effectively been created
        ret = ctx->devctx->ilapi->get_parameter(ctx->ilctx, VK_PARAM_ERROR,
                                                &error, VK_CMD_OPT_BLOCKING);
        if ((!ret) && (error.log[0] != '\0'))
            av_log(avctx, AV_LOG_ERROR, "[FW-ERR]<%s>\n", error.log);
    }

    if (ret)
        av_log(avctx, AV_LOG_ERROR, "%s: error %s (%d)\n", caller, strerror(abs(ret)), ret);
    else
        ret = AVERROR(EINVAL);

    return ret;
}

static av_cold void uninit(AVFilterContext *avctx)
{
    int i;
    VKAPIScaleContext *ctx = avctx->priv;
    AVFilterLink *outlink;
    AVHWFramesContext *hwframes_ctx;
    VKAPIFramesContext *vk_framectx;

    av_log(ctx, AV_LOG_DEBUG, "close scaler\n");

    av_dict_free(&ctx->opts);
    if (ctx->is_inited) {
        for (i = 0; i < ctx->noutputs; i++) {
            outlink  = avctx->outputs[i];
            if (outlink && outlink->hw_frames_ctx) {
                hwframes_ctx = (AVHWFramesContext *)outlink->hw_frames_ctx->data;
                if (hwframes_ctx) {
                    vk_framectx = hwframes_ctx->hwctx;
                    if (vk_framectx)
                        vk_framectx->ilctx = NULL;
                }
            }
        }

        // retrieve warnings
        vkapi_warnings_retrieve(avctx, __func__);

        if (ctx->ilctx)
            ctx->devctx->ilapi->deinit((void **)&ctx->ilctx);
        ctx->is_inited = 0;
    }

    ff_bufqueue_discard_all(&ctx->in_queue);
    for (i = 0; i < MAX_OUTPUTS; i++) {
        ff_bufqueue_discard_all(&ctx->out_queues[i]);
        ff_bufqueue_discard_all(&ctx->wait_queues[i]);
    }

    if (ctx->hw_frames_ctx)
        av_buffer_unref(&ctx->hw_frames_ctx);
    if (ctx->hwdevice)
        av_buffer_unref(&ctx->hwdevice);
    ctx->is_inited = 0;

    if (ctx->varsfile) {
        for (i = 0; i < MAX_OUTPUTS + 1; i++) {
            if (ctx->vars[i])
                fclose(ctx->vars[i]);
            if (ctx->vars_data[i])
                av_freep(&ctx->vars_data[i]);
        }
    }

    // ctx->hwctx is deinited in hwcontext_vkapi::deinit
}

static int query_formats(AVFilterContext *avctx)
{
    int i, ret;
    VKAPIScaleContext *ctx = avctx->priv;

    static const enum AVPixelFormat hw_pix_fmts[] = {
        AV_PIX_FMT_VKAPI,
        AV_PIX_FMT_NONE
    };

    const enum AVPixelFormat *pix_fmts = vkapi_pix_fmts;

    if (ctx->out_pixel_format == AV_PIX_FMT_VKAPI)
        pix_fmts = hw_pix_fmts;

    ret = ff_formats_ref(ff_make_format_list(vkapi_pix_fmts), &avctx->inputs[0]->out_formats);
    if (ret < 0)
        return ret;

    for (i = 0; i < avctx->nb_outputs; i++) {
        ret = ff_formats_ref(ff_make_format_list(pix_fmts), &avctx->outputs[i]->in_formats);
        if (ret < 0)
            return ret;
    }


    return 0;
}

/**
 * init the hw scaler
 * @param avctx
 * @return error code.
 */
static av_cold int vkapi_scale_init(AVFilterLink *inlink)
{
    int ret = AVERROR(EINVAL);
    AVFilterContext *avctx = inlink->dst;
    VKAPIScaleContext *ctx;
    VKAPIFramesContext *vk_framectx;
    AVHWFramesContext *hwframes_ctx;
    AVBufferRef       *avhwframes_ctx;
    AVHWDeviceContext *hwdevice_ctx;
    AVBufferRef *hw_frames_ref = NULL;

    av_assert0(avctx);
    av_assert0(avctx->priv);

    ctx = avctx->priv;
    avhwframes_ctx = inlink->hw_frames_ctx;

    if (avhwframes_ctx) {
        // this path is taken on hw acceleration
        hwframes_ctx = (AVHWFramesContext *)avhwframes_ctx->data;
        // so the device is provided by the upstream component
        ctx->hwdevice = av_buffer_ref(hwframes_ctx->device_ref);
        if (!(ctx->hwdevice))
            goto fail_noctx;

        ctx->hw_frames_ctx = av_buffer_ref(avhwframes_ctx);
    } else {
        if (avctx->hw_device_ctx) {
            ctx->hwdevice = av_buffer_ref(avctx->hw_device_ctx);
            if (!(ctx->hwdevice))
                goto fail_noctx;
        } else {
            ret = av_hwdevice_ctx_create(&ctx->hwdevice, AV_HWDEVICE_TYPE_VKAPI, NULL, NULL, 0);
            if (ret)
                goto fail_noctx;
        }
        hw_frames_ref = av_hwframe_ctx_alloc(ctx->hwdevice);
        if (!hw_frames_ref) {
            ret = AVERROR(ENOMEM);
            goto fail_noctx;
        }
        ctx->hw_frames_ctx = hw_frames_ref;
        hwframes_ctx = (AVHWFramesContext *)hw_frames_ref->data;
        hwframes_ctx->format    = AV_PIX_FMT_VKAPI;
        hwframes_ctx->sw_format = inlink->format;
        hwframes_ctx->width     = inlink->w;
        hwframes_ctx->height    = inlink->h;
        hwframes_ctx->initial_pool_size = VK_SCL_INITIAL_POOL_SIZE;
        ret = av_hwframe_ctx_init(hw_frames_ref);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "Failed to initialise hwframe ctx %d\n", ret);
            goto fail_noctx;
        }
    }

    hwdevice_ctx = hwframes_ctx->device_ctx;
    ctx->devctx = hwdevice_ctx->hwctx;
    vk_framectx = hwframes_ctx->hwctx;

    ret = ctx->devctx->ilapi->init((void **)(&ctx->ilctx));
    if (ret)
        goto fail_noctx;

    ctx->ilctx->context_essential.component_role = VK_SCALER;
    ctx->ilctx->context_essential.queue_id = vkil_get_processing_pri();
    ret = ctx->devctx->ilapi->init((void **)(&ctx->ilctx));
    if (ret)
        goto fail_noctx;

    if (!vk_framectx->ilctx)
        // if the hwh as no ilcontext, that means it has been created in this function
        // we assign the scaler ilctx one
        vk_framectx->ilctx = ctx->ilctx;

    return 0;

fail_noctx:
    av_log(avctx, AV_LOG_ERROR, "Failed to init scaler ret = %d \n", ret);
    if (hw_frames_ref)
        av_buffer_unref(&hw_frames_ref);
    ret = vkapi_error_handling(avctx, ret, __func__);
    if (ctx->ilctx)
        ctx->devctx->ilapi->deinit((void **)(&ctx->ilctx));
    return ret;
}

static int config_props_x(AVFilterLink *outlink, int i)
{
    AVFilterContext *avctx = outlink->src;
    AVFilterLink *inlink  = outlink->src->inputs[0];
    VKAPIScaleContext *ctx = avctx->priv;
    AVHWFramesContext *hwframes_ctx;
    VKAPIFramesContext *vk_framectx;
    int ret, w, h;

    if (!ctx->is_inited) {
        av_log(avctx, AV_LOG_DEBUG, "hw_frames_ctx=%p\n", inlink->hw_frames_ctx);
        ret = vkapi_scale_init(inlink);
        if (ret)
            goto fail;
        ctx->is_inited = 1;
    }

    if ((!ctx->w_expr[i]) || (!ctx->h_expr[i]))
        return 0;

    ret = ff_scale_eval_dimensions(avctx,
                                   ctx->w_expr[i], ctx->h_expr[i],
                                   inlink, outlink,
                                   &w, &h);
    if (ret < 0)
        goto fail;

    ret = ff_scale_adjust_dimensions(inlink, &w, &h, 0, 0);
    if (ret)
        goto fail;

    outlink->w = w;
    outlink->h = h;

    if (inlink->sample_aspect_ratio.num)
        outlink->sample_aspect_ratio = av_mul_q((AVRational){outlink->h * inlink->w, outlink->w * inlink->h},
                                                inlink->sample_aspect_ratio);
    else
        outlink->sample_aspect_ratio = inlink->sample_aspect_ratio;

    outlink->hw_frames_ctx = av_hwframe_ctx_alloc(ctx->hwdevice);
    if (!outlink->hw_frames_ctx) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    hwframes_ctx = (AVHWFramesContext *)outlink->hw_frames_ctx->data;
    hwframes_ctx->format    = AV_PIX_FMT_VKAPI;

    if (ctx->out_pixel_format == AV_PIX_FMT_NONE)
        hwframes_ctx->sw_format = get_sw_format(inlink);
    else
        hwframes_ctx->sw_format = ctx->out_pixel_format;

    hwframes_ctx->width     = FFALIGN(outlink->w, 32);
    hwframes_ctx->height    = FFALIGN(outlink->h, 32);
    // normally, we should get the below from the HW, but at the time being, the HW didn't have yet
    // initialized the scaler: it needs to know all the output first, then and only then
    // the HW ill allocate the scaler and its output pools. so for the time being, we just allocate
    // what is the minimum size
    hwframes_ctx->initial_pool_size = VK_SCL_INITIAL_POOL_SIZE;
    ret = ff_filter_init_hw_frames(avctx, outlink, VK_SCL_INITIAL_POOL_SIZE);
    if (ret < 0)
        goto fail;
    // In the above, we assume we tell ffmepg the available pool size, however, ffmpeg seems to
    // ignore this value (that is, it will call the filter, even when no hw frame are available)
    // So In practice one need to be prepared to have much more frame than specifiied and still face
    // the case where the hw can't allocate frames.
    //
    // This part of code (regarding the instantiation of the hwframes_ctx is inspired from the vaapi
    // filter implementation (see vaapi_vpp.c:ff_vaapi_vpp_config_output) which also use a hardcoded
    // value for the pool size.


    ret = av_hwframe_ctx_init(outlink->hw_frames_ctx);
    if (ret)
        goto fail;

    vk_framectx = hwframes_ctx->hwctx;
    vk_framectx->port_id = i;

    if (inlink->hw_frames_ctx) {
        hwframes_ctx = (AVHWFramesContext *)inlink->hw_frames_ctx->data;
        memcpy(&vk_framectx->color,
               &((VKAPIFramesContext*)(hwframes_ctx->hwctx))->color, sizeof(vk_framectx->color));
        av_log(avctx, AV_LOG_DEBUG, "copy input color info range=%d, primaries=%d, trc=%d, colorspace=%d/n",
               vk_framectx->color.range, vk_framectx->color.primaries, vk_framectx->color.trc, vk_framectx->color.space);
    }

    if (ctx->qpvars)
        vk_framectx->extra_port_id = MAX_OUTPUTS + i + 1;
    vk_framectx->ilctx = ctx->ilctx;

    av_log(avctx, AV_LOG_DEBUG, "vk_framectx %p, vk_framectx->port_id %d, vk_framectx->ilctx %p\n",
           vk_framectx, vk_framectx->port_id, vk_framectx->ilctx);
    return 0;

fail:
    av_log(avctx, AV_LOG_ERROR, "failed to config outlink %d for reason %d\n", i, ret);
    return ret;
}

static int config_props_0(AVFilterLink *outlink)
{
    return config_props_x(outlink, 0);
}

static int config_props_1(AVFilterLink *outlink)
{
    return config_props_x(outlink, 1);
}

static int config_props_2(AVFilterLink *outlink)
{
    return config_props_x(outlink, 2);
}

static int config_props_3(AVFilterLink *outlink)
{
    return config_props_x(outlink, 3);
}

static int pull_frames(AVFilterLink *outlink)
{
    AVFilterContext *avctx = outlink->src;
    VKAPIScaleContext *ctx = avctx->priv;
    VKAPIDeviceContext *devctx = ctx->devctx;
    vkil_buffer_surface  vk_output_surface[MAX_OUTPUTS];
    vkil_buffer_metadata vk_output_metadata[MAX_OUTPUTS + 1];
    vkil_aggregated_buffers vk_output_buffers = {.prefix.type = VKIL_BUF_AG_BUFFERS};
    AVFrame *tmp_frame = NULL, *out_frame = NULL;
    VKAPIFramesContext *vk_framectx;
    AVHWFramesContext *avhw_framectx;
    int ret, i, valid_format;

    if (!ctx->frame) {
        ret = AVERROR(EINVAL);
        goto fail;
    }

    ctx->received_frames++;
    av_log(avctx, AV_LOG_DEBUG, "pull_frames ctx->received_frames =%d\n", ctx->received_frames);

    // we build an aggregated buffer of ctx->noutputs surface
    for (i = 0; i < ctx->noutputs; i++) {
        vk_output_surface[i].prefix.type = VKIL_BUF_SURFACE;
        vk_output_surface[i].prefix.ref = 0;
        vk_output_buffers.buffer[i] = (vkil_buffer *)&vk_output_surface[i];
    }
    // then we add enough buffer for all possible metadata
    for (i = 0; i < (MAX_OUTPUTS + 1); i++) {
        vk_output_metadata[i].prefix.type = VKIL_BUF_META_DATA;
        vk_output_metadata[i].prefix.ref = 0;
        vk_output_buffers.buffer[MAX_OUTPUTS + i] = (vkil_buffer *)&vk_output_metadata[i];
    }

    ret = devctx->ilapi->process_buffer(ctx->ilctx, &vk_output_buffers,
                                        VK_CMD_OPT_CB | VK_CMD_OPT_BLOCKING);

    if (ret == (-ETIMEDOUT) || ret == (-ENOMSG) || ret == (-EAGAIN))
          return AVERROR(EAGAIN); // no buffer delivered yet, will have to try again
    else if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "transfer buffer result = %d, ctx->received_frames =%d\n", ret, ctx->received_frames);
        goto fail_buf_proc;
    }

    // then retrieve the outputted frames
    for (i = 0; i < ctx->noutputs; i++) {
        AVFilterLink *outlink = avctx->outputs[i];

        out_frame = ff_bufqueue_get(&ctx->wait_queues[i]);
        if (!out_frame)
            goto fail;

        ret = devctx->frame_set_hwprops(out_frame, (vkil_buffer_surface *)vk_output_buffers.buffer[i]);
        if (ret)
            goto fail;

        valid_format = ff_fmt_is_in(outlink->format, vkapi_pix_fmts);
        if (outlink->format == AV_PIX_FMT_VKAPI) {
            if (ctx->varsfile && ctx->qpvars) {
                // if both qpvars and varsfile are enabled, the variance map will be both tunneled
                // to the next component and downloaded, so we need to add a reference on it
                ret = devctx->ilapi->xref_buffer(ctx->ilctx, &vk_output_metadata[i + 1], 1,
                                                 VK_CMD_OPT_BLOCKING);
                if (ret < 0)
                    goto fail_buf_proc;
            }
            if (ctx->qpvars) {
                out_frame->data[VKAPI_METADATA_PLANE] = (uint8_t *)((uintptr_t)vk_output_metadata[i + 1].prefix.handle);
            }
        } else if (valid_format) {
            // not tunneled SW compatible format: we download it
            // the download could be performed well by vf_hwdownload after calling the scaler
            // however we implemnet it here too making the use of the scaler simpler at ffmpeg command
            // line (no need to explictly add an hwdownload filter in non hw accelerated cases)

            tmp_frame = out_frame;
            out_frame = ff_get_video_buffer(outlink, outlink->w, outlink->h);
            if (!out_frame) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }

            av_assert0(tmp_frame->hw_frames_ctx->data);
            avhw_framectx = (AVHWFramesContext *)tmp_frame->hw_frames_ctx->data;
            av_assert0(avhw_framectx->hwctx);
            vk_framectx = avhw_framectx->hwctx;
            vk_framectx->handle = vk_output_buffers.buffer[i]->handle;
            vk_framectx->ilctx  = ctx->ilctx;

            ret = av_hwframe_transfer_data(out_frame, tmp_frame, 0);
            if (ret)
                goto fail;

            if ((!ctx->varsfile) && ctx->qpvars) {
                // if qpvars is selected and we are not in tunneling mode, we need to dereference
                // the variance map, since no one will pick it up
                ret = devctx->ilapi->xref_buffer(ctx->ilctx, &vk_output_metadata[i + 1], -1,
                                                 VK_CMD_OPT_BLOCKING);
                if (ret < 0)
                    goto fail_buf_proc;
            }

            av_frame_free(&tmp_frame);
            tmp_frame = NULL;
        } else {
            av_log(avctx, AV_LOG_ERROR, "unsupported output format %d\n", outlink->format);
            ret = AVERROR(ENODEV);  // we can't process this fromat
            goto fail;
        }

        ret = av_frame_copy_props(out_frame, ctx->frame);
        if (ret < 0)
            goto fail;

        ff_bufqueue_add(avctx, &ctx->out_queues[i], out_frame);
        out_frame = NULL;
    }

    if (!ctx->varsfile  && ctx->qpvars) {
        // if variance map are only used for qpvars, then we don't need the one on input and dereference it
        ret = devctx->ilapi->xref_buffer(ctx->ilctx, &vk_output_metadata[0], -1,
                                         VK_CMD_OPT_BLOCKING);
        if (ret < 0)
            goto fail_buf_proc;
    }

    if (ctx->varsfile && (vk_output_buffers.nbuffers >= (MAX_OUTPUTS + 1))) {
        for (i = 0; i < (ctx->noutputs + 1); i++) {
            vk_output_metadata[i].data = ctx->vars_data[i]; // memory to download vars data
            vk_output_metadata[i].size = ctx->vars_size[i];
            vk_output_metadata[i].prefix.type = VKIL_BUF_META_DATA;
            ret = devctx->ilapi->transfer_buffer(ctx->ilctx, &vk_output_metadata[i],
                                                 VK_CMD_DOWNLOAD | VK_CMD_OPT_BLOCKING);
            if (ret < 0) {
                av_log(avctx, AV_LOG_ERROR, "receive vars buffer failed\n");
                goto fail_buf_proc;
            }
            if (vk_output_metadata[i].size && ctx->vars[i]) {
                ret = fwrite(ctx->vars_data[i], 1, ctx->vars_size[i], ctx->vars[i]);
                if (ret < ctx->vars_size[i])
                    goto fail;
             }
        }
    }

    av_frame_free(&ctx->frame);
    return 0;

fail_buf_proc:
    vkapi_error_handling(avctx, ret, __func__);
    ret = AVERROR(EINVAL);
fail:
    av_frame_free(&ctx->frame);
    av_frame_free(&tmp_frame);
    av_frame_free(&out_frame);
    av_log(avctx, AV_LOG_ERROR, "failure = %d\n", ret);
    return ret;
}

static int int_request_frame_x(AVFilterLink *outlink, int i)
{
    AVFilterContext *avctx = outlink->src;
    VKAPIScaleContext *ctx = avctx->priv;
    AVFrame *frame;
    int ret;


    if (ctx->in_queue.available > ctx->max_lag) {
        if (!ctx->frame)
            ctx->frame = ff_bufqueue_get(&ctx->in_queue);

        ret = pull_frames(outlink);
        if ((ret != AVERROR(EAGAIN)) &&  ret < 0)
            goto fail;

        if (!ret)
            ctx->frame = NULL;

        ret = 0; // the scaler doesn't understand the AVERROR(EAGAIN) (frame not yet ready)
                 // so we just assume success, here, ffmpeg will come back...
    }

    if (ctx->out_queues[i].available) {
        frame = ff_bufqueue_get(&ctx->out_queues[i]);
        return ff_filter_frame(avctx->outputs[i], frame);
    }
    return 0;

fail:
    av_log(NULL, AV_LOG_ERROR, "failure = %d\n", ret);
    return ret;
}

static int request_frame_x(AVFilterLink *outlink, int i)
{
    AVFilterContext *avctx = outlink->src;
    AVFilterLink *inlink  = outlink->src->inputs[0];
    VKAPIScaleContext *ctx = avctx->priv;
    AVFrame *frame;
    int ret = 0;

    if (ctx->in_queue.available <= ctx->max_lag)
    {
        ret = ff_request_frame(inlink);
        if (ret == AVERROR_EOF) {
            ctx->flush = VK_CMD_OPT_BLOCKING;
            ctx->max_lag = 0;
        } else {
            return ret;
        }
    }

    if (ctx->in_queue.available > ctx->max_lag) {
        if (!ctx->frame)
            ctx->frame = ff_bufqueue_get(&ctx->in_queue);

        ret = pull_frames(outlink);
        if ((ret != AVERROR(EAGAIN)) &&  ret < 0)
            goto fail;

        if (!ret)
            ctx->frame = NULL;

        ret = 0;
    }

    if (ctx->out_queues[i].available) {
        frame = ff_bufqueue_get(&ctx->out_queues[i]);
        return ff_filter_frame(avctx->outputs[i], frame);
    }

    if (ctx->flush == VK_CMD_OPT_BLOCKING)
        return AVERROR_EOF;
    else
        return 0;

fail:
    av_log(NULL, AV_LOG_ERROR, "failure = %d\n", ret);
    return ret;
}

static int request_frame_0(AVFilterLink *outlink)
{
    return request_frame_x(outlink, 0);
}

static int request_frame_1(AVFilterLink *outlink)
{
    return request_frame_x(outlink, 1);
}

static int request_frame_2(AVFilterLink *outlink)
{
    return request_frame_x(outlink, 2);
}

static int request_frame_3(AVFilterLink *outlink)
{
    return request_frame_x(outlink, 3);
}

static av_cold int init(AVFilterContext *avctx)
{
    VKAPIScaleContext *ctx;
    int i;
    int ret = -EINVAL;

    static AVFilterPad scale_vkapi_outputs[] = {
        {
            .name         = "default0",
            .type         = AVMEDIA_TYPE_VIDEO,
            .config_props = config_props_0,
            .request_frame = request_frame_0,
        },
        {
            .name         = "default1",
            .type         = AVMEDIA_TYPE_VIDEO,
            .config_props = config_props_1,
            .request_frame = request_frame_1,
        },
        {
            .name         = "default2",
            .type         = AVMEDIA_TYPE_VIDEO,
            .config_props = config_props_2,
            .request_frame = request_frame_2,
        },
        {
            .name         = "default3",
            .type         = AVMEDIA_TYPE_VIDEO,
            .config_props = config_props_3,
            .request_frame = request_frame_3,
        },
    };

    ctx = avctx->priv;
    ctx->is_inited = 0;
    ctx->received_frames = 0;
    ctx->send_frames = 0;

    for (i = 0; i < ctx->noutputs; i++) {
        if (ctx->w_expr[i] == NULL || ctx->h_expr[i] == NULL) {
            av_log(avctx, AV_LOG_ERROR, "scaling data is not available for all stages\n");
            return -EINVAL;
        }
    }

    if (ctx->noutputs > MAX_OUTPUTS) {
        av_log(avctx, AV_LOG_ERROR, "requested number of output %d greater than max %d\n", ctx->noutputs, MAX_OUTPUTS);
        return -EINVAL;
    }

    for (i = 0; i < ctx->noutputs; i++) {
        ret = ff_insert_outpad(avctx, i, &scale_vkapi_outputs[i]);
        if (ret < 0)
            break;
    }

    return ret;
}

static int vkapi_write_header(const VKAPIVarsHeader *header)
{
    int ret = 0;

    ret = fprintf(header->file,
                  "<header>type=vk-variance-map:version=0:size=%d:surface-width=%d:surface-height=%d</header>",
                  header->size, header->width, header->height);
    if (ret < 0)
        goto fail;

    return 0;

fail:
    av_log(NULL, AV_LOG_ERROR, "failed to write file header for %p", header);
    return ret;
}

static int vkapi_internal_init(AVFilterLink *inlink)
{
    int ret, i;
    AVFilterContext *avctx = inlink->dst;
    VKAPIScaleContext *ctx = avctx->priv;
    vk_scl_cfg scl_cfg;
    VKAPIVarsHeader header;
    VKAPIFramesContext *vk_framectx = NULL;
    AVHWFramesContext *avhwframes_ctx = NULL;
    vk_port_id port;

    if (inlink->hw_frames_ctx) {
        // on hardware acceleration, we check that the avhw_frames_ctx is effectively there, and instantiate
        // the VKAPIFramesContext accoridngly
        avhwframes_ctx = (AVHWFramesContext*)inlink->hw_frames_ctx->data;
        if (avhwframes_ctx)
            vk_framectx = avhwframes_ctx->hwctx;
    }

    // ensure we don't have undefined values
    memset(&scl_cfg, 0, sizeof(vk_scl_cfg));
    memset(ctx->vars, 0, sizeof(ctx->vars[0]) * MAX_OUTPUTS);
    // We set the scaler parameters
    scl_cfg.filter_luma = ctx->filter_flags & ((1 << VK_CHROMA_OFFSET) -1);
    scl_cfg.filter_chroma = (ctx->filter_flags >> VK_CHROMA_OFFSET) & ((1 << VK_CHROMA_OFFSET) -1);

    scl_cfg.input_size.width  = inlink->w;
    scl_cfg.input_size.height = inlink->h;

    scl_cfg.in_format  = ctx->devctx->av2vk_fmt(inlink->format);

    if (ctx->out_pixel_format == AV_PIX_FMT_NONE)
        scl_cfg.out_format = scl_cfg.in_format;
    else
        scl_cfg.out_format = ctx->devctx->av2vk_fmt(ctx->out_pixel_format);

    for (i = 0; i < ctx->noutputs; i++) {
        AVFilterLink *outlink  = avctx->outputs[i];
        scl_cfg.output_size[i].width  = outlink->w;
        scl_cfg.output_size[i].height = outlink->h;
    }
    scl_cfg.noutputs = ctx->noutputs;

    if (ctx->varsfile) {
        char filename[80];
        for (i = 0; i < ctx->noutputs + 1; i++) {
            snprintf(filename, sizeof(filename), "%s_%1d", ctx->varsfile, i);
            ctx->vars[i] = fopen(filename,"wb");
            if (!ctx->vars[i]) {
                // will call uninit which will close already open files
                av_log(avctx, AV_LOG_ERROR, "can't open %s\n", filename);
                ret = AVERROR(EINVAL);
                goto fail;
            }
        }
    }

    if (ctx->varsfile || ctx->qpvars)
        scl_cfg.vars_cfg.flags = VK_CFG_FLAG_ENABLE;

    if (ctx->customfile) {
        vk_scl_custom_filter custom_filter;
        vkil_buffer_metadata vk_custom_filter = {.prefix.type = VKIL_BUF_META_DATA, \
                                                 .prefix.port_id = VK_SCL_FILTER_COEFS};
        FILE *custom = fopen(ctx->customfile,"rb");

        if (!custom) {
                av_log(avctx, AV_LOG_ERROR, "can't open %s \n", ctx->customfile);
                ret = AVERROR(EINVAL);
                goto fail;
        }

        // read custom parameters
        vk_custom_filter.size = fread(&custom_filter, 1, sizeof(vk_scl_custom_filter), custom);
        fclose(custom);
        if (vk_custom_filter.size != sizeof(vk_scl_custom_filter)) {
            av_log(avctx, AV_LOG_ERROR,
                   "%s file is of size %d not compatible with the vk_scl_custom_filter structure size ",
                   ctx->customfile, vk_custom_filter.size);
            ret = AVERROR(EINVAL);
        }

        // transfer them to the card
        vk_custom_filter.data = &custom_filter;
        ret = ctx->devctx->ilapi->transfer_buffer(ctx->ilctx,
                         &vk_custom_filter, VK_CMD_UPLOAD | VK_CMD_OPT_BLOCKING);
        if (ret)
            goto fail;

        // then provide the handle in config parameter
        scl_cfg.custom_filter_handle = vk_custom_filter.prefix.handle;
    }

    // if parameters are invalid, the set_parameter will return an EINVAL code
    ret = ctx->devctx->ilapi->set_parameter(ctx->ilctx, VK_PARAM_VIDEO_SCL_CONFIG,
                                            &scl_cfg, VK_CMD_OPT_BLOCKING);
    if (ret)
        goto fail;

    if (vk_framectx && vk_framectx->ilctx) {
        // in tunneled mode we connect the source output pad to the encoder input pad
        vk_port input_port;

        // first we get the source output pad (it is the default pad)
        input_port.port_id.id = VK_SCL_VIDEO_IN;
        input_port.port_id.direction = VK_OUTPUT_PORT;
        ret = ctx->devctx->ilapi->get_parameter(vk_framectx->ilctx, VK_PARAM_PORT,
                                                &input_port, VK_CMD_OPT_BLOCKING);
        if (ret)
            goto fail;

        // then we apply it to the scaler (single) input
        input_port.port_id.direction = VK_INPUT_PORT;
        ret = ctx->devctx->ilapi->set_parameter(ctx->ilctx, VK_PARAM_PORT,
                                                &input_port, VK_CMD_OPT_BLOCKING);
        if (ret)
            goto fail;
    }

    // We get the size of the output variance map if any need to be written
    if (ctx->varsfile) {
        for (i = 0; i < ctx->noutputs + 1; i++) {
            int32_t value = i;
            ret = ctx->devctx->ilapi->get_parameter(ctx->ilctx, VK_PARAM_VARMAP_SIZE,
                                                    &value, VK_CMD_OPT_BLOCKING);
            if (ret)
                goto fail;
            ctx->vars_size[i] = value;
            ctx->vars_data[i] = av_malloc(value);
            if (!ctx->vars_data[i]) {
                av_log(avctx, AV_LOG_ERROR, "Failed to allocate vars buffer\n");
                ret = AVERROR(ENOMEM);
                goto fail;
            }
            // write a variance map header
            header.file = ctx->vars[i];
            header.size = value;
            if (i) {
                header.width = scl_cfg.output_size[i - 1].width;
                header.height = scl_cfg.output_size[i - 1].height;
            } else {
                header.width = scl_cfg.input_size.width;
                header.height = scl_cfg.input_size.height;
            }
            ret = vkapi_write_header(&header);
            if (ret)
                goto fail;
        }
    }

    // Once done, we can complete the scaler initialization
    ret = ctx->devctx->ilapi->init((void **)(&ctx->ilctx));
    if (ret)
        goto fail;

    // get the size of the hw  input pool buffer
    port.direction = VK_INPUT_PORT;
    port.id  = 0;
    // the function will overwrite the port field with the pool size, which
    // will then be available in output_port.map
    ret = ctx->devctx->ilapi->get_parameter(ctx->ilctx, VK_PARAM_POOL_SIZE,
                                            &port, VK_CMD_OPT_BLOCKING);
    if (ret)
        goto fail;

    // a lag of 2 is sufficient to ensure we give time to the HW to do the filtering
    ctx->max_lag = FFMIN(2, port.map);
    av_log(avctx, AV_LOG_DEBUG, "scaler context = %p \n", ctx->ilctx);
    return 0;

fail:
    ret = vkapi_error_handling(avctx, ret, __func__);
    ctx->devctx->ilapi->deinit((void **)&ctx->ilctx);
    return ret;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *avctx = inlink->dst;
    VKAPIScaleContext *ctx = avctx->priv;
    VKAPIDeviceContext *devctx;
    vkil_buffer_surface  vk_buffer_surface;
    AVFrame *tmp_frame = NULL, *out_frame = NULL;
    int i, ret, valid_format;

    if (ctx->is_inited==1) {
        ret = vkapi_internal_init(inlink);
        if (ret) {
            ret = AVERROR(EINVAL);
            goto fail;
        }
        ctx->is_inited = 2;
    }

    if (!frame) {
        ret = AVERROR(EINVAL);
        goto fail;
    }

    devctx = ctx->devctx;

    ctx->send_frames++;

    valid_format = ff_fmt_is_in(inlink->format, vkapi_pix_fmts);

    if (inlink->format == AV_PIX_FMT_VKAPI) {
        // hwframe format as input (tunneled operation)
        // so we expect a minimum of hw configuration
        av_assert0(frame->hw_frames_ctx && frame->format == AV_PIX_FMT_VKAPI);
        devctx->frame_get_hwprops(frame, &vk_buffer_surface);
    } else if (valid_format) {
        // frame uploading is required
        // the upload could be removed of this function, since the operation could be
        // performed as well by vf_hwupload before calling the scaler however keeping it
        // makes the use of the scaler simpler at ffmpeg command line
        tmp_frame = av_frame_alloc();
        if (!tmp_frame) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        ret = av_hwframe_get_buffer(ctx->hw_frames_ctx, tmp_frame, 0);
        if (ret < 0)
            goto fail;

        ret = av_hwframe_transfer_data(tmp_frame, frame, 0);
        if (ret)
            goto fail;

        devctx->frame_get_hwprops(tmp_frame, &vk_buffer_surface);
    } else {
        av_log(avctx, AV_LOG_ERROR, "unsupported input pixel format\n");
        ret = AVERROR(ENODEV);  // we can't encode this
        goto fail;
    }

    // send the input frame to the hw scaler
    devctx= ctx->devctx;
    ret = devctx->ilapi->process_buffer(ctx->ilctx, &vk_buffer_surface, VK_CMD_RUN);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "transfer buffer result = %d\n", ret);
        goto fail_buf_proc;
    }

    if (tmp_frame) {
        devctx->frame_set_hwprops(tmp_frame, &vk_buffer_surface);
        av_frame_free(&tmp_frame);
    } else {
        devctx->frame_set_hwprops(frame, &vk_buffer_surface);
    }

    ff_bufqueue_add(avctx, &ctx->in_queue, frame);

    // from this point, the HW will acquire a HW frame buffer to output the decoded frame
    // to have FFMPEG to be aware of this fact (output buffer occupancy, we reserve an output buffer now

    for (i = 0; i < ctx->noutputs; i++) {
        AVFilterLink *outlink = avctx->outputs[i];
        tmp_frame = av_frame_alloc();
        if (!tmp_frame) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        ret = av_hwframe_get_buffer(outlink->hw_frames_ctx, tmp_frame, 0);
        if (ret)
            goto fail;

        ff_bufqueue_add(avctx, &ctx->wait_queues[i], tmp_frame);
    }

    for (i = 0; i < ctx->noutputs; i++)
        int_request_frame_x(avctx->outputs[i], i);

    return 0;

fail_buf_proc:
    vkapi_error_handling(avctx, ret, __func__);
    ret = AVERROR(EINVAL);
fail:
    av_frame_free(&frame);
    av_frame_free(&tmp_frame);
    av_frame_free(&out_frame);
    av_log(avctx, AV_LOG_ERROR, "failure = %d\n", ret);
    return ret;
}

static const AVClass *child_class_next(const AVClass *prev)
{
    return prev ? NULL : sws_get_class();
}

#define OFFSET(x) offsetof(VKAPIScaleContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption scale_options[] = {
    {"outputs", "set number of outputs",    OFFSET(noutputs),   AV_OPT_TYPE_INT, { .i64 = 1 }, 1, INT_MAX, FLAGS },
    {"w0",     "Output video width",        OFFSET(w_expr[0]),  AV_OPT_TYPE_STRING, .flags = FLAGS},
    {"width0", "Output video width",        OFFSET(w_expr[0]),  AV_OPT_TYPE_STRING, .flags = FLAGS},
    {"h0",     "Output video height",       OFFSET(h_expr[0]),  AV_OPT_TYPE_STRING, .flags = FLAGS},
    {"height0","Output video height",       OFFSET(h_expr[0]),  AV_OPT_TYPE_STRING, .flags = FLAGS},
    {"w1",     "Output video width",        OFFSET(w_expr[1]),  AV_OPT_TYPE_STRING, .flags = FLAGS},
    {"width1", "Output video width",        OFFSET(w_expr[1]),  AV_OPT_TYPE_STRING, .flags = FLAGS},
    {"h1",     "Output video height",       OFFSET(h_expr[1]),  AV_OPT_TYPE_STRING, .flags = FLAGS},
    {"height1","Output video height",       OFFSET(h_expr[1]),  AV_OPT_TYPE_STRING, .flags = FLAGS},
    {"w2",     "Output video width",        OFFSET(w_expr[2]),  AV_OPT_TYPE_STRING, .flags = FLAGS},
    {"width2", "Output video width",        OFFSET(w_expr[2]),  AV_OPT_TYPE_STRING, .flags = FLAGS},
    {"h2",     "Output video height",       OFFSET(h_expr[2]),  AV_OPT_TYPE_STRING, .flags = FLAGS},
    {"height2","Output video height",       OFFSET(h_expr[2]),  AV_OPT_TYPE_STRING, .flags = FLAGS},
    {"w3",     "Output video width",        OFFSET(w_expr[3]),  AV_OPT_TYPE_STRING, .flags = FLAGS},
    {"width3", "Output video width",        OFFSET(w_expr[3]),  AV_OPT_TYPE_STRING, .flags = FLAGS},
    {"h3",     "Output video height",       OFFSET(h_expr[3]),  AV_OPT_TYPE_STRING, .flags = FLAGS},
    {"height3","Output video height",       OFFSET(h_expr[3]),  AV_OPT_TYPE_STRING, .flags = FLAGS},
    {"vars",   "variance map files ",       OFFSET(varsfile),   AV_OPT_TYPE_STRING, .flags = FLAGS},
    {"qpvars", "variance map for qp",       OFFSET(qpvars),     AV_OPT_TYPE_BOOL, {.i64 = 0 }, -1, 1, FLAGS },
    {"custom_params", "custom filter file", OFFSET(customfile), AV_OPT_TYPE_STRING, .flags = FLAGS},
    {"flags", "scaler flags (default bicubic)", OFFSET(filter_flags), AV_OPT_TYPE_FLAGS, {.i64 = VK_BICUBIC}, .flags = FLAGS, .unit = "flags"},
        {"bilinear", "bilinear",                            0, AV_OPT_TYPE_CONST, {.i64 = VK_BILINEAR}, .flags = FLAGS, .unit = "flags"},
        {"bicubic",  "bicubic",                             0, AV_OPT_TYPE_CONST, {.i64 = VK_BICUBIC},  .flags = FLAGS, .unit = "flags"},
        {"neighbor", "nearest neighbor",                    0, AV_OPT_TYPE_CONST, {.i64 = VK_NEAREST},  .flags = FLAGS, .unit = "flags"},
        {"bicublin", "luma cubic, chroma bilinear",         0, AV_OPT_TYPE_CONST, {.i64 = VK_BICUBLIN}, .flags = FLAGS, .unit = "flags"},
        {"catmull",  "catmull type filter",                 0, AV_OPT_TYPE_CONST, {.i64 = VK_CATMULL},  .flags = FLAGS, .unit = "flags"},
        {"custom",   "params read from custom_params file", 0, AV_OPT_TYPE_CONST, {.i64 = VK_CUSTOM},   .flags = FLAGS, .unit = "flags"},
    {"format",  "output pixel format of hardware frames", OFFSET(out_pixel_format), AV_OPT_TYPE_FLAGS, {.i64 = AV_PIX_FMT_NONE}, .flags = FLAGS, .unit = "format"},
        {"same", "same as input",   0, AV_OPT_TYPE_CONST, {.i64 = AV_PIX_FMT_NONE}, .flags = FLAGS, .unit = "format"},
        {"nv12",  "nv12",           0, AV_OPT_TYPE_CONST, {.i64 = AV_PIX_FMT_NV12}, .flags = FLAGS, .unit = "format"},
        {"nv21",  "nv21",           0, AV_OPT_TYPE_CONST, {.i64 = AV_PIX_FMT_NV21}, .flags = FLAGS, .unit = "format"},
        {"p010",  "p010",           0, AV_OPT_TYPE_CONST, {.i64 = AV_PIX_FMT_P010}, .flags = FLAGS, .unit = "format"},
        {"vkapi", "vkapi",          0, AV_OPT_TYPE_CONST, {.i64 = AV_PIX_FMT_VKAPI}, .flags = FLAGS, .unit = "format"},
    { NULL }
};

static const AVClass scale_class = {
    .class_name       = "scale",
    .item_name        = av_default_item_name,
    .option           = scale_options,
    .version          = LIBAVUTIL_VERSION_INT,
    .category         = AV_CLASS_CATEGORY_FILTER,
    .child_class_next = child_class_next,
};

static const AVFilterPad scale_vkapi_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
    },
    { NULL }
};

AVFilter ff_vf_scale_vkapi = {
    .name            = "scale_vkapi",
    .description     = NULL_IF_CONFIG_SMALL("Scale the input video size and/or convert the image format."),
    .init            = init,
    .uninit          = uninit,
    .query_formats   = query_formats,
    .priv_size       = sizeof(VKAPIScaleContext),
    .priv_class      = &scale_class,
    .inputs          = scale_vkapi_inputs,
    .outputs         = NULL,
    .flags_internal  = FF_FILTER_FLAG_HWFRAME_AWARE | AVFILTER_FLAG_DYNAMIC_OUTPUTS,
};
