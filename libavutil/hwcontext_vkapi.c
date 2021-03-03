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

#include "avassert.h"
#include "buffer.h"
#include "buffer_internal.h"
#include "common.h"
#include "hwcontext.h"
#include "hwcontext_internal.h"
#include "hwcontext_vkapi.h"
#include "imgutils.h"
#include "mem.h"
#include "pixdesc.h"
#include "pixfmt.h"

#define VKAPI_FRAME_ALIGNMENT 256

static const enum AVPixelFormat supported_formats[] = {
    AV_PIX_FMT_NV12,
    AV_PIX_FMT_NV21,
    AV_PIX_FMT_P010,
    AV_PIX_FMT_P016,
    AV_PIX_FMT_VKAPI,
    AV_PIX_FMT_NONE,
};

/**
 * check if the format fmt is in an array
 * (this function is a copy and paste from libavfilter/formats.c:ff_fmt_is_in
 * however, due to definition in the libavfilter library prevent the reuse of
 * it as is in libavutil library)
 * @param the format to check
 * @param the array of formats
 * @return 1 if format is presnet in array, 0 otherwise
 */
static int vkapi_fmt_is_in(int fmt, const int *fmts)
{
    const int *p;

    for (p = fmts; *p != -1; p++) {
        if (fmt == *p)
            return 1;
    }
    return 0;
}

/**
 * convert the pixel format from the AVPixelFormat to a vk_format_type
 * equivalent
 * @param an AVPixelFormat
 * @return  a vkil_format_type if positive, error code if negative
 */
static int vkapi_av2vk_fmt(enum AVPixelFormat pixel_format)
{
    switch (pixel_format) {
    case AV_PIX_FMT_NV12:
        return VK_FORMAT_NV12;
    case AV_PIX_FMT_NV21:
        return VK_FORMAT_NV21;
    case AV_PIX_FMT_P010:
        return VK_FORMAT_P010;
    case AV_PIX_FMT_VKAPI:
        return VK_FORMAT_YOL2;
    default:
        return AVERROR(EINVAL);
    }
}

/**
 * Provide the number of buffer currently referenced in the hw pool
 * @param ctx, the hw frames context
 * @return  number of buffer in use in in the internal pool if postive, errorotherwise
 */
static int vkapi_get_pool_occupancy(AVHWFramesContext *ctx)
{
    int val = atomic_load(&ctx->internal->pool_internal->refcount);
    return (val - 1);
}

/**
 * get the reference on the frame hwprops
 * @param the ffmpeg avframe structure
 * @param the vkil buffer surface  associated with the ffmpeg avframe
 * @return  0 if succesfull, error code otherwise
 */
static int32_t vkapi_frame_ref_hwprops(const AVFrame *frame, void *phw_surface_desc)
{
    void **hw_surface_desc = phw_surface_desc;

    if (frame->format != AV_PIX_FMT_VKAPI)
        return AVERROR_BUG;

    *hw_surface_desc = frame->data[3];

    if (!(*hw_surface_desc))
        return AVERROR_BUG;

    return 0;
}

/**
 * set a ffmpeg AVFrame with the vkil buffer surface properties
 * @param the ffmpeg avframe structure
 * @param the vkil buffer surface to be associated with the ffmpeg avframe
 * @return  0 if succesfull, error code otherwise
 */
static int32_t vkapi_frame_set_hwprops(AVFrame *frame, const vkil_buffer_surface *hw_surface_desc)
{
    if (frame->format != AV_PIX_FMT_VKAPI)
        return AVERROR_BUG;

    // vkil buffer surface description is stored in ffmpeg AVframe:data[3]
    av_assert0(frame->data[3]);

    // the "hard copy", concerns just a buffer descriptor, so is a fairly reasonnable tradeoff to
    // not be bothered by life cycle differences between the hw_surface_desc and frame
    memcpy((vkil_buffer_surface *)frame->data[3], hw_surface_desc, sizeof(vkil_buffer_surface));

    return 0;
}

/**
 * retrieve vkil buffer surface properties associated with an ffmpeg AVFrame
 * @param the ffmpeg avframe structure
 * @param a vkil buffer surface, where where to extract the property.
 * @return  0 if succesfull, error code otherwise
 */
static int32_t vkapi_frame_get_hwprops(const AVFrame *frame, vkil_buffer_surface *hw_surface_desc)
{
    if (frame->format != AV_PIX_FMT_VKAPI)
        return AVERROR_BUG;

    // the frame is hw tunneled (hw format)
    // vkil buffer surface description is stored in ffmpeg AVframe:data[3]
    av_assert0(frame->data[3]);

    // the "hard copy", concerns just a buffer descriptor, so is a fairly reasonnable tradeoff to
    // not be bothered by life cycle differences between the hw_surface_desc and frame
    memcpy(hw_surface_desc, (vkil_buffer_surface *)frame->data[3], sizeof(vkil_buffer_surface));
    return 0;
}

static int vkapi_frames_get_constraints(AVHWDeviceContext *ctx,
                                        const void *hwconfig,
                                        AVHWFramesConstraints *constraints)
{
    av_log(ctx, AV_LOG_DEBUG, "vkapi_frames_get_constraints\n");
    return 0;
}

static void vkapi_pool_release(void *opaque, uint8_t *data)
{
    av_free(data);
}

static AVBufferRef *vkapi_pool_alloc(void *opaque, int size)
{
    AVHWFramesContext *ctx = opaque;
    AVBufferRef *ret_ptr = NULL;
    vkil_buffer_surface *hw_surface_desc;

    hw_surface_desc = av_mallocz(sizeof(vkil_buffer_surface));
    if (!hw_surface_desc) {
        av_log(hw_surface_desc, AV_LOG_ERROR, "av_mallocz failed\n");
        goto out;
    }

    ret_ptr = av_buffer_create((uint8_t *)hw_surface_desc, sizeof(*hw_surface_desc),
                               vkapi_pool_release, NULL, AV_BUFFER_FLAG_READONLY);
    if (!ret_ptr) {
        av_log(ctx, AV_LOG_ERROR, "av_buffer_create failed\n");
        av_free(hw_surface_desc);
    }

out:
    return ret_ptr;
}

static int vkapi_frames_init(AVHWFramesContext *ctx)
{
    int aligned_width = FFALIGN(ctx->width, VKAPI_FRAME_ALIGNMENT);
    int ret = AVERROR(EINVAL);
    int valid_format = vkapi_fmt_is_in(ctx->sw_format, supported_formats);
    VKAPIFramesContext *vk_framectx = ctx->hwctx;

    av_log(ctx, AV_LOG_DEBUG, "vkapi_frames_init\n");

    if (!valid_format)
        goto fail;

    // set default VKAPIFramesContext
    vk_framectx->color.range = AVCOL_RANGE_UNSPECIFIED;
    vk_framectx->color.primaries = AVCOL_PRI_UNSPECIFIED;
    vk_framectx->color.trc = AVCOL_TRC_UNSPECIFIED;
    vk_framectx->color.space = AVCOL_SPC_UNSPECIFIED;

    if (!ctx->pool) {
        int size;

        switch (ctx->sw_format) {
        case AV_PIX_FMT_VKAPI:
            size = sizeof(void *);
            break;
        case AV_PIX_FMT_NV12:
        case AV_PIX_FMT_NV21:
            size = aligned_width * ctx->height * 3 / 2;
            break;
        case AV_PIX_FMT_P010:
        case AV_PIX_FMT_P016:
            size = aligned_width * ctx->height * 3;
            break;
        default:
            goto fail;
        }

        ctx->internal->pool_internal = av_buffer_pool_init2(size, ctx, vkapi_pool_alloc, NULL);
        if (!ctx->internal->pool_internal) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
    }
    return 0;

fail:
    av_log(ctx, AV_LOG_ERROR, "vkapi_frames_init failed on error %d\n", ret);
    return ret;

}

static int vkapi_get_buffer(AVHWFramesContext *ctx, AVFrame *frame)
{
    int ret = 0;

    if (!(frame->buf[0] = av_buffer_pool_get(ctx->pool))) {
        av_log(ctx, AV_LOG_ERROR, "av_buffer_pool_get failed\n");
        ret = AVERROR(ENOMEM);
    } else {
        // vkil data are stored in frame->data[3]:
        // we follow the convention established by the vaapi, qsv or vdpau implementations
        frame->data[3] = frame->buf[0]->data;
        frame->format = AV_PIX_FMT_VKAPI;
        frame->width  = ctx->width;
        frame->height = ctx->height;
    }

    return ret;
}

static int vkapi_transfer_get_formats(AVHWFramesContext *ctx,
                                      enum AVHWFrameTransferDirection dir,
                                      enum AVPixelFormat **formats)
{
    enum AVPixelFormat *fmts;
    int ret = 0;

    fmts = av_malloc_array(2, sizeof(*fmts)); // this is freed in hwcontext::transfer_data_alloc
    if (!fmts) {
        av_log(ctx, AV_LOG_ERROR, "vkapi_transfer_get_formats failed\n");
        ret = AVERROR(ENOMEM);
    } else {
        fmts[0] = ctx->sw_format;
        fmts[1] = AV_PIX_FMT_NONE;
        *formats = fmts;
    }

    return ret;
}

static int vkapi_convert_AV2VK_Frame(vkil_buffer_surface *dst, const AVFrame *src)
{
    int i;

    // TODO: for the time being we restraint the thing to progressive material
    av_assert0(src->interlaced_frame == 0);

    dst->max_size.width  = src->width;
    dst->max_size.height = src->height;
    dst->format           = vkapi_av2vk_fmt(src->format);
    dst->quality          = src->quality;
    dst->prefix.type      = VKIL_BUF_SURFACE;
    dst->prefix.port_id   = 0; // set to default (first port)
    dst->prefix.flags     = 0; // other fields set to zero (default value)

    for (i = 0; i < VKIL_BUF_NPLANES; i++) {
        dst->plane_top[i] = src->data[i];
        dst->plane_bot[i] = src->data[i + VKIL_BUF_NPLANES];
        dst->stride[i] = src->linesize[i];
        // sanity check
        // vk structure requires 32 bits alignment
        if (((uintptr_t)dst->plane_top[i] & (VKIL_BUF_ALIGN -1)) ||
            ((uintptr_t)dst->plane_bot[i] & (VKIL_BUF_ALIGN -1)) ||
            (dst->stride[i] &(VKIL_BUF_ALIGN -1)))
                return AVERROR(EINVAL);
    }

    return 0;
}

static void vkapi_release_buffer(AVFrame *frame)
{
    int ret;
    VKAPIFramesContext *hw_framectx;
    AVHWFramesContext *avhw_framectx;
    VKAPIDeviceContext *hw_devicectx;
    vkil_buffer_surface surface;
    vkil_buffer metadata;
    vkil_context *ilctx;

    av_assert0(frame->hw_frames_ctx);
    av_assert0(frame->hw_frames_ctx->data);

    avhw_framectx = (AVHWFramesContext *)frame->hw_frames_ctx->data;
    av_assert0(avhw_framectx);
    hw_framectx = avhw_framectx->hwctx;
    av_assert0(hw_framectx);
    av_assert0(avhw_framectx->device_ctx);
    hw_devicectx = avhw_framectx->device_ctx->hwctx;
    ilctx = hw_framectx->ilctx;

    if (!ilctx) {
        av_log(NULL, AV_LOG_DEBUG, "vkapi_release_buffer ilctx =  NULL \n");
        return;
    }
    // allows to populate the vkil_surface structure with the origin pointer on the host
    vkapi_convert_AV2VK_Frame(&surface, frame);
    ret = vkapi_frame_get_hwprops(frame, &surface);

    if (!ret) {
        av_log(NULL, AV_LOG_DEBUG,
               "frame %p hw_framectx %p hw_devicectx %p ilctx %p frame->data[3] = %p, surface.prefix.ref = %d surface.prefix.handle %x \n",
               frame, hw_framectx, hw_devicectx, ilctx, frame->data[3], surface.prefix.ref, surface.prefix.handle);

        // the command is blocking, since ffmpeg assumes the transfer is complete on function return
        if (surface.prefix.ref) {
            ret = hw_devicectx->ilapi->xref_buffer(ilctx, &surface, -1,
                                                   VK_CMD_RUN | VK_CMD_OPT_BLOCKING);
            if (ret)
                av_log(NULL, AV_LOG_ERROR, "vkapi_release_buffer error for ilctx %p on hw handle 0x%x \n", ilctx, surface.prefix.handle);
            if (frame->data[VKAPI_METADATA_PLANE]) {
                metadata.handle = (uint32_t)frame->data[VKAPI_METADATA_PLANE];
                metadata.type = VKIL_BUF_META_DATA;
                metadata.ref = 1;
                ret = hw_devicectx->ilapi->xref_buffer(ilctx, &metadata, -1,
                                                       VK_CMD_RUN | VK_CMD_OPT_BLOCKING);
                if (ret)
                    av_log(NULL, AV_LOG_ERROR, "vkapi_release_buffer error for ilctx %p on hw handle 0x%x \n", ilctx, metadata.handle);
            }
        }
    }

    vkapi_frame_set_hwprops(frame, &surface);
}

static int vkapi_ref_buffer(AVFrame *frame)
{
    int ret;
    VKAPIFramesContext *hw_framectx;
    AVHWFramesContext *avhw_framectx;
    VKAPIDeviceContext *hw_devicectx;
    vkil_buffer_surface surface;
    vkil_buffer metadata;
    vkil_context *ilctx;

    av_assert0(frame->hw_frames_ctx);
    av_assert0(frame->hw_frames_ctx->data);

    avhw_framectx = (AVHWFramesContext *)frame->hw_frames_ctx->data;
    av_assert0(avhw_framectx);
    hw_framectx = avhw_framectx->hwctx;
    av_assert0(hw_framectx);
    av_assert0(avhw_framectx->device_ctx);
    hw_devicectx = avhw_framectx->device_ctx->hwctx;
    ilctx = hw_framectx->ilctx;

    if (!ilctx) {
        av_log(NULL, AV_LOG_DEBUG, "vkapi_ref_buffer ilctx =  NULL \n");
        return 0;
    }
    // allows to populate the vkil_surface structure with the origin pointer on the host
    vkapi_convert_AV2VK_Frame(&surface, frame);
    ret = vkapi_frame_get_hwprops(frame, &surface);

    if (!ret) {
        av_log(NULL, AV_LOG_DEBUG,
               "frame %p hw_framectx %p hw_devicectx %p ilctx %p frame->data[3] = %p, surface.prefix.ref = %d surface.prefix.handle %x \n",
               frame, hw_framectx, hw_devicectx, ilctx, frame->data[3], surface.prefix.ref, surface.prefix.handle);

        // the command is blocking, since ffmpeg assumes the transfer is complete on function return
        if (surface.prefix.ref) {
            ret = hw_devicectx->ilapi->xref_buffer(ilctx, &surface, 1,
                                                   VK_CMD_RUN | VK_CMD_OPT_BLOCKING);
            if (ret)
                av_log(NULL, AV_LOG_ERROR, "vkapi_ref_buffer error for ilctx %p  on hw handle 0x%x \n", ilctx, surface.prefix.handle);

            if (frame->data[VKAPI_METADATA_PLANE]) {
                metadata.handle = (uint32_t)frame->data[VKAPI_METADATA_PLANE];
                metadata.type = VKIL_BUF_META_DATA;
                metadata.ref = 1;
                ret = hw_devicectx->ilapi->xref_buffer(ilctx, &metadata, 1,
                                                       VK_CMD_RUN | VK_CMD_OPT_BLOCKING);
                if (ret)
                    av_log(NULL, AV_LOG_ERROR, "vkapi_ref_buffer error for ilctx %p  on hw handle 0x%x \n", ilctx, metadata.handle);
            }
        }
    }

    vkapi_frame_set_hwprops(frame, &surface);

    return 0;
}

static int vkapi_transfer_data_from(AVHWFramesContext *ctx, AVFrame *dst,
                                    const AVFrame *src)
{
    int ret;
    int32_t size;
    VKAPIFramesContext *hw_framectx;
    AVHWFramesContext *avhw_framectx;
    VKAPIDeviceContext *hw_devicectx;
    vkil_buffer_surface *surface;
    vkil_buffer metadata;

    hw_devicectx = ctx->device_ctx->hwctx;

    av_assert0(src->hw_frames_ctx->data);
    avhw_framectx = (AVHWFramesContext *)src->hw_frames_ctx->data;

    av_assert0(avhw_framectx->hwctx);
    hw_framectx = avhw_framectx->hwctx;
    vkapi_frame_ref_hwprops(src, &surface);

    // populate the vkil_surface structure with the destination pointer on the host
    vkapi_convert_AV2VK_Frame(surface, dst);

    av_log(ctx, AV_LOG_DEBUG , "vkapi_transfer_data_from surface.prefix.handle = 0x%x , ilctx=%p \n",
           surface->prefix.handle, hw_framectx->ilctx);

    // the command is blocking, since ffmpeg assume the transfer is complete on function return
    ret = hw_devicectx->ilapi->transfer_buffer2(hw_framectx->ilctx, surface,
                                                VK_CMD_DOWNLOAD | VK_CMD_OPT_BLOCKING, &size);
    if (ret < 0)
        goto fail;


    if (src->data[VKAPI_METADATA_PLANE]) {
        // In case of the transferred frame has an associated metadata buffer this one is dereferenced
        // since it make no sense to transfer it back to host
        metadata.handle = (uint32_t)src->data[VKAPI_METADATA_PLANE];
        metadata.type = VKIL_BUF_META_DATA;
        metadata.ref = 1;
        ret = hw_devicectx->ilapi->xref_buffer(hw_framectx->ilctx, &metadata, -1,
                                               VK_CMD_RUN | VK_CMD_OPT_BLOCKING);
        if (ret)
            goto fail;
    }

    return 0;

fail:
    av_log(ctx, AV_LOG_ERROR, "failure %d on vkapi_transfer_data_to\n", ret);
    return AVERROR(EINVAL);
}

static int vkapi_transfer_data_to(AVHWFramesContext *ctx, AVFrame *dst,
                                  const AVFrame *src)
{
    int ret;
    int32_t i, size = 0;
    VKAPIFramesContext *hw_framectx;
    AVHWFramesContext *avhw_framectx;
    VKAPIDeviceContext *hw_devicectx;
    vkil_buffer_surface *surface;
    vkil_context *ilctx;
    uint8_t *tmp_data[4] = {NULL, NULL, NULL, NULL};
    int linesize[4];

    av_assert0(VKIL_BUF_NPLANES * 2 <= 4); //sanity check

    hw_devicectx = ctx->device_ctx->hwctx;

    av_assert0(dst->hw_frames_ctx->data);
    avhw_framectx = (AVHWFramesContext *)dst->hw_frames_ctx->data;

    av_assert0(avhw_framectx->hwctx);
    hw_framectx = avhw_framectx->hwctx;

    ret = vkapi_frame_ref_hwprops(dst, &surface);
    if (ret < 0) {
        ret = AVERROR(EINVAL);
        goto fail;
    }

    // allows to populate the vkil_surface structure with the origin pointer on the host
    ret = vkapi_convert_AV2VK_Frame(surface, src);
    if (ret) { // address or stride is not aligned, we first copy the frame on aligned buffer
        ret = av_image_alloc(tmp_data, linesize, src->width, src->height, src->format, VKIL_BUF_ALIGN);
        if (ret < 0)
            goto fail;

        av_image_copy(tmp_data, linesize, (const uint8_t **)src->data, src->linesize,
                      src->format, src->width, src->height);

        for (i = 0; i < VKIL_BUF_NPLANES; i++) {
                surface->plane_top[i]= tmp_data[i];
                surface->plane_bot[i]= tmp_data[VKIL_BUF_NPLANES + i];
                surface->stride[i] = linesize[i];
        }
    }

    // some ancillary field already set for destination, are copied here to the vkil structure
    surface->quality = dst->quality;

    ilctx = hw_framectx->ilctx;
    if (ilctx == NULL) {
        ret = AVERROR(EINVAL);
        goto fail;
    }

    // the command is blocking, since ffmpeg assumes the transfer is complete on function return
    ret = hw_devicectx->ilapi->transfer_buffer2(ilctx, surface,
                                                VK_CMD_UPLOAD | VK_CMD_OPT_BLOCKING, &size);
    if (ret < 0) {
        ret = AVERROR(EINVAL);
        goto fail;
     }

    hw_framectx->handle = surface->prefix.handle;

    if (tmp_data[0])
        av_free(tmp_data[0]);

    return 0;

fail:
    if (tmp_data[0])
            av_free(tmp_data[0]);

    av_log(ctx, AV_LOG_ERROR, "failure %d on vkapi_transfer_data_to\n", ret);
    return ret;
}

static void vkapi_device_uninit(AVHWDeviceContext *ctx)
{
    VKAPIDeviceContext *hwctx;

    av_assert0(ctx);
    av_assert0(ctx->hwctx);

    hwctx = ctx->hwctx;

    av_log(ctx, AV_LOG_DEBUG, "vkapi_device_uninit\n");

    if (hwctx->ilapi)
        vkil_destroy_api((void **)&hwctx->ilapi);
}

static int vkapi_device_init(AVHWDeviceContext *ctx)
{
    VKAPIDeviceContext *hwctx;
    int ret = AVERROR_EXTERNAL;

    av_assert0(ctx);
    av_assert0(ctx->hwctx);

    av_log(ctx, AV_LOG_DEBUG, "vkapi_device_init\n");

    hwctx = ctx->hwctx;
    hwctx->frame_set_hwprops = vkapi_frame_set_hwprops;
    hwctx->frame_get_hwprops = vkapi_frame_get_hwprops;
    hwctx->frame_ref_hwprops = vkapi_frame_ref_hwprops;
    hwctx->fmt_is_in = vkapi_fmt_is_in;
    hwctx->av2vk_fmt = vkapi_av2vk_fmt;
    hwctx->get_pool_occupancy = vkapi_get_pool_occupancy;

    if (!(hwctx->ilapi = vkil_create_api())) {
        av_log(ctx, AV_LOG_ERROR, "ctx->ilapi failed to be created\n");
        goto out;
    }

    if (!hwctx->ilapi->init) {
        av_log(ctx, AV_LOG_ERROR, "ctx->ilapi not properly initialized\n");
        goto out;
    }

    ret = 0;

out:
    if (ret)
        vkapi_device_uninit(ctx);

    return ret;
}

static int vkapi_device_create(AVHWDeviceContext *ctx, const char *device,
                               AVDictionary *opts, int flags)
{
    av_log(ctx, AV_LOG_DEBUG, "vkapi_device_create\n");
    return 0;
}

const HWContextType ff_hwcontext_type_vkapi = {
    .type                   = AV_HWDEVICE_TYPE_VKAPI,
    .name                   = "VKAPI",
    .device_hwctx_size      = sizeof(VKAPIDeviceContext),
    .frames_hwctx_size      = sizeof(VKAPIFramesContext),
    .device_create          = vkapi_device_create,
    .device_init            = vkapi_device_init,
    .device_uninit          = vkapi_device_uninit,
    .frames_get_constraints = vkapi_frames_get_constraints,
    .frames_init            = vkapi_frames_init,
    .frames_get_buffer      = vkapi_get_buffer,
    .frames_ref_buffer      = vkapi_ref_buffer,
    .frames_release_buffer  = vkapi_release_buffer,
    .transfer_get_formats   = vkapi_transfer_get_formats,
    .transfer_data_to       = vkapi_transfer_data_to,
    .transfer_data_from     = vkapi_transfer_data_from,
    .pix_fmts               = (const enum AVPixelFormat[]){ AV_PIX_FMT_VKAPI, AV_PIX_FMT_NONE },
};
