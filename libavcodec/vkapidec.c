/*
 * Copyright (c) 2018 Broadcom
 *
 * VKAPI Video Decoder
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
#include <time.h>
#include "avcodec.h"
#include "hwconfig.h"
#include "internal.h"
#include "libavcodec/decode.h"
#include "libavutil/avassert.h"
#include "libavutil/buffer.h"
#include "libavutil/common.h"
#include "libavutil/fifo.h"
#include "libavutil/hwcontext_vkapi.h"
#include "libavutil/imgutils.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/log.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "libavutil/timestamp.h"


// this force exit on error on API function only to prevent recursive call otherwise
#define VK_EXIT_ON_ERROR 1
#define VK_EXIT_ERROR 2 // exit code if forced exit in this file

#ifdef VK_EXIT_ON_ERROR
#include "fftools/cmdutils.h"
#endif

/* timeout of seconds for EOS */
#define VKDEC_FLUSH_TIMEOUT_US (5 * 1000000L)
#define VKDEC_FLUSH_YIELD_US   (100 * 1000L)
#define _ELAPSED_US(_end, _st) \
    ((_end.tv_sec - _st.tv_sec) * 1000000L + (_end.tv_nsec - _st.tv_nsec) / 1000L)

#define VKDEC_LAG_IS_ODD 0x1

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_VKAPI, //HW format can't be in last entry to use ff_get_format
    AV_PIX_FMT_NV12,
    AV_PIX_FMT_P010,
    AV_PIX_FMT_NONE,
};

typedef struct VKAPIDecodeContext {
    AVClass *av_class;
    AVBufferRef  *hwdevice; //!< allow a reference to the hw device
    AVBufferRef  *hwframe;
    //! handy shortcut used to the hwdevice field (the defaukt ilctx created by hwctx is currently used)
    VKAPIDeviceContext *hwctx;
    int32_t interlace; //! the stream is interlaced
    int32_t flush; //!< flushing all the the decoder buffers is required (end of stream)
    int32_t lock; //!<  we are forced to wait for frame to be outputted (decoder work in lock step)
    int32_t send_extradata; //!< indicate if extra data need to be or not
    int32_t max_lag;
    int32_t min_lag;
    int64_t received_frames;
    int64_t sent_packets;
    int64_t start_bad_pkts;
    AVFifoBuffer *dts_queue; //!< decoding time stamp.
    int exit_on_error;
} VKAPIDecodeContext;


/*
 * the below function is copied and pasted from libavcodec/decode.c
 * libavcodec/decode.c::decode_receive_frame_internal calls
 * libavcodec/decode.c::guess_correct_pts; which is defined as static; when passing thru
 * libavcodec/decode.c::decode_simple_receive_frame.
 * However when avctx->codec->receive_frame exists the function
 * libavcodec/decode.c::guess_correct_pts is not invoked anymore, we redefine it here to
 * still provide a best effort estimate for the frame's timestamp.
 */

/**
 * Attempt to guess proper monotonic timestamps for decoded video frames
 * which might have incorrect times. Input timestamps may wrap around, in
 * which case the output will as well.
 *
 * @param pts the pts field of the decoded AVPacket, as passed through
 * AVFrame.pts
 * @param dts the dts field of the decoded AVPacket
 * @return one of the input values, may be AV_NOPTS_VALUE
 */
static int64_t guess_correct_pts(AVCodecContext *ctx,
                                 int64_t reordered_pts, int64_t dts)
{
    int64_t pts = AV_NOPTS_VALUE;

    if (dts != AV_NOPTS_VALUE) {
        ctx->pts_correction_num_faulty_dts += dts <= ctx->pts_correction_last_dts;
        ctx->pts_correction_last_dts = dts;
    } else if (reordered_pts != AV_NOPTS_VALUE)
        ctx->pts_correction_last_dts = reordered_pts;

    if (reordered_pts != AV_NOPTS_VALUE) {
        ctx->pts_correction_num_faulty_pts += reordered_pts <= ctx->pts_correction_last_pts;
        ctx->pts_correction_last_pts = reordered_pts;
    } else if(dts != AV_NOPTS_VALUE)
        ctx->pts_correction_last_pts = dts;

    if ((ctx->pts_correction_num_faulty_pts<=ctx->pts_correction_num_faulty_dts || dts == AV_NOPTS_VALUE)
       && reordered_pts != AV_NOPTS_VALUE)
        pts = reordered_pts;
    else
        pts = dts;

    return pts;
}

static int vkapi_warnings_retrieve(AVCodecContext *avctx, const char * const caller)
{
    vk_warning warn;
    VKAPIDecodeContext *ctx;
    AVHWFramesContext *hwframe_ctx;
    VKAPIFramesContext *vk_framectx;
    int ret = 0, cnt = 0;

#define VK_FW_MAX_WARNINGS_TOPRINT 12
    // this may be called at early init failure where not much is setup up, so
    // do check individual ptrs progressively before continue.
    if (!avctx || !avctx->priv_data)
        return ret;
    ctx = avctx->priv_data;
    if (!ctx->hwframe || !ctx->hwframe->data)
        return ret;
    hwframe_ctx = (AVHWFramesContext *)ctx->hwframe->data;
    vk_framectx = hwframe_ctx->hwctx;

    if (vk_framectx && vk_framectx->ilctx && vk_framectx->ilctx->devctx) {
        while (cnt++ < VK_FW_MAX_WARNINGS_TOPRINT) {
            ret = ctx->hwctx->ilapi->get_parameter(vk_framectx->ilctx, VK_PARAM_WARNING,
                                                   &warn, VK_CMD_OPT_BLOCKING);
            if ((ret) || (warn.log[0] == '\0'))
                break;

            av_log(avctx, AV_LOG_WARNING, "[FW-WARN]%s:<%s>\n", caller, warn.log);
        }
    }

    return ret;
}

static int vkapi_error_handling(AVCodecContext *avctx, int ret, const char * const caller)
{
    vk_error error;
    VKAPIDecodeContext *ctx;
    AVHWFramesContext *hwframe_ctx;
    VKAPIFramesContext *vk_framectx;

    /* sanity check */
    av_assert0(avctx);
    av_assert0(avctx->priv_data);

    ctx = avctx->priv_data;
    hwframe_ctx = (AVHWFramesContext *)ctx->hwframe->data;
    vk_framectx = hwframe_ctx->hwctx;

    // retrieve warnings
    vkapi_warnings_retrieve(avctx, caller);

    if ((ret == -EADV) && vk_framectx->ilctx) {
        // get verbose hw error only when the ilctx has effectively been created
        ret = ctx->hwctx->ilapi->get_parameter(vk_framectx->ilctx, VK_PARAM_ERROR,
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

/**
 * @brief Internal flush of the decoder
 *
 * The internal flush is either called from the AVCodec flush api
 * vkapi_decode_close function itself called from the AVCodec close api
 * @param avctx the decoder context AVCodec  decoder context
 * @return  error code.
 */
static int vkapi_decode_flush_internal(AVCodecContext *avctx)
{
    VKAPIDecodeContext *ctx;
    VKAPIDeviceContext *hw_devicectx;
    AVHWFramesContext *hwframe_ctx;
    VKAPIFramesContext *vk_framectx;
    vkil_buffer_surface hw_surface_desc = {.prefix.type = VKIL_BUF_SURFACE};
    vkil_buffer_packet hw_packet_desc = {.prefix.type = VKIL_BUF_PACKET};
    int32_t ret, interlace;
    struct timespec start_time, end_time;
    int delta_us;

    // sanity check
    av_assert0(avctx);
    av_assert0(avctx->priv_data);

    ctx = avctx->priv_data;
    hw_devicectx = ctx->hwctx;

    hwframe_ctx = (AVHWFramesContext *)ctx->hwframe->data;
    vk_framectx = hwframe_ctx->hwctx;

    av_assert0(hw_devicectx && hw_devicectx->ilapi);

    av_log(avctx, AV_LOG_DEBUG, "vkapi_decode_flush\n");
    if ((!ctx->flush) && (ctx->received_frames != ctx->sent_packets)) {
         ret = ctx->hwctx->ilapi->get_parameter(vk_framectx->ilctx, VK_PARAM_IS_STREAM_INTERLACE,
                                                &interlace, VK_CMD_OPT_BLOCKING);
         if (ret)
             goto fail;

         if (!interlace) {
             // if we have send more packet than received frames, and are not in a flushing
             // mode, we need to put the decoder in flushing mode to enable the purging of it
             // (In frame reordering case, the decoder could wait for more frame to release the frame
             // it still hold, putting the decoder in flush mode force it to flush its DPB
             hw_packet_desc.prefix.handle = VK_BUF_EOS;
             ret = hw_devicectx->ilapi->process_buffer(vk_framectx->ilctx, &hw_packet_desc, VK_CMD_RUN);
             if (ret < 0)
                 goto fail;

             ctx->flush = VK_CMD_OPT_BLOCKING;
         }
    }

    /*
     * in all cases, we would try to drain the buffers for a while, and see if we receive
     * the EOS.  If timeout, then, that is an error condition, and we should log.
     */
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    delta_us = 0;
    while ((ctx->flush) && (delta_us < VKDEC_FLUSH_TIMEOUT_US)) {
        clock_gettime(CLOCK_MONOTONIC, &end_time);
        delta_us = _ELAPSED_US(end_time, start_time);

        ret = hw_devicectx->ilapi->process_buffer(vk_framectx->ilctx, &hw_surface_desc,
                                                  VK_CMD_OPT_CB);
        if ((ret == -ENOMSG) || (ret == -EAGAIN)) {
            av_usleep(VKDEC_FLUSH_YIELD_US);
            continue;
        } else if (ret) {
            goto fail;
        }

        if (hw_surface_desc.prefix.handle == VK_BUF_EOS) {
            av_log(avctx, AV_LOG_DEBUG, "end of stream detected in flush\n");
            ctx->flush = 0;
            return 0;
        } else if (hw_surface_desc.prefix.handle == VK_DEC_OUT_INVALID_HDL) {
            av_log(avctx, AV_LOG_ERROR, "invalid handle\n");
            ctx->sent_packets--;
            continue;
        }

        /* update received_frames so that in case the xref below fails, the counter is still valid */
        ctx->received_frames++;

        // otherwise if we retrieve a dangling frame, we just dereference it from the hardware buffer
        // (The hardware could be waiting for a free buffer before processing next packet).
        ret = hw_devicectx->ilapi->xref_buffer(vk_framectx->ilctx, &hw_surface_desc, -1,
                                               VK_CMD_OPT_BLOCKING);
        if (ret)
            goto fail;

        av_log(avctx, AV_LOG_DEBUG, "Flush: sent pkt %ld received_frame %ld delta_us %d\n",
               ctx->sent_packets, ctx->received_frames, delta_us);
        av_usleep(VKDEC_FLUSH_YIELD_US);
    }

    if (delta_us >= VKDEC_FLUSH_TIMEOUT_US) {
        av_log(avctx, AV_LOG_ERROR,
               "No EOS detected, timeout %ld us %ld sent packet %ld received frames\n",
               VKDEC_FLUSH_TIMEOUT_US, ctx->sent_packets,
               ctx->received_frames);
        /* retrieve error if any */
        vkapi_error_handling(avctx, -EADV, __func__);
    }
    return 0;

fail:
    vkapi_error_handling(avctx, ret, __func__);
    av_log(avctx, AV_LOG_ERROR, "Flush failure: %ld sent packet generated %ld decoded frames\n",
           ctx->sent_packets, ctx->received_frames);
    return AVERROR(EINVAL);
}

/**
 * flush the decoder
 * @param avctx the decoder context
 */
static void vkapi_decode_flush(AVCodecContext *avctx)
{
    VKAPIDecodeContext *ctx;
    int ret;

    // sanity check
    av_assert0(avctx);
    av_assert0(avctx->priv_data);

    ctx = avctx->priv_data;
    ret = vkapi_decode_flush_internal(avctx);
    if (ret && ctx->exit_on_error)
        exit_program(VK_EXIT_ERROR);
}

/**
 * dereference the decoder
 * @param avctx
 * @return always zero
 */
static av_cold int vkapi_decode_close(AVCodecContext *avctx)
{
    VKAPIDecodeContext *ctx;
    VKAPIDeviceContext *hw_devicectx;
    AVHWFramesContext *hwframe_ctx;
    VKAPIFramesContext *vk_framectx;

    av_log(avctx, AV_LOG_DEBUG, "vkapi_decode_close\n");

    av_assert0(avctx);
    av_assert0(avctx->priv_data);

    ctx = avctx->priv_data;

    hw_devicectx = ctx->hwctx;

    // notice that this function is also called by exit_program
    // So this function can't call exit program to prevent infinite recursive
    // call

    if (hw_devicectx && hw_devicectx->ilapi)
        vkapi_decode_flush_internal(avctx);

    // retrieve any warnings during trancoding before deinit
    vkapi_warnings_retrieve(avctx, __func__);

    if (ctx->dts_queue)
        av_fifo_freep(&ctx->dts_queue);
    if (ctx->hwframe) {
        hwframe_ctx = (AVHWFramesContext *)ctx->hwframe->data;
        if (hwframe_ctx) {
            vk_framectx = hwframe_ctx->hwctx;
            if (vk_framectx && vk_framectx->ilctx)
                ctx->hwctx->ilapi->deinit((void **)&vk_framectx->ilctx);
        }
        av_buffer_unref(&ctx->hwframe);
    }
    if (ctx->hwdevice)
        av_buffer_unref(&ctx->hwdevice);
    // ctx->hwctx is deinited in hwcontext_vkapi::deinit

    return 0;
}

static int vkapi_get_vkprofilelevel(const int ffprofile, const int fflevel, const enum AVCodecID codec_id)
{
    enum vk_video_profile profile = VK_V_PROFILE_UNKNOWN;
    enum vk_video_level   level   = VK_V_LEVEL_UNKNOWN;

    if (codec_id == AV_CODEC_ID_H264) {
        switch (ffprofile) {
            case FF_PROFILE_H264_BASELINE:             profile = VK_V_PROFILE_H264_BASELINE; break;
            case FF_PROFILE_H264_CONSTRAINED_BASELINE: profile = VK_V_PROFILE_H264_CONSTRAINED_BASELINE; break;
            case FF_PROFILE_H264_MAIN:                 profile = VK_V_PROFILE_H264_MAIN; break;
            case FF_PROFILE_H264_EXTENDED:             profile = VK_V_PROFILE_H264_EXTENDED; break;
            case FF_PROFILE_H264_HIGH:                 profile = VK_V_PROFILE_H264_HIGH; break;
            default: profile = VK_V_PROFILE_UNKNOWN;
        }
        switch (fflevel) {
            case 10: level = VK_V_LEVEL_H264_1;  break;
            case 11: level = VK_V_LEVEL_H264_11; break;
            case 12: level = VK_V_LEVEL_H264_12; break;
            case 13: level = VK_V_LEVEL_H264_13; break;
            case 20: level = VK_V_LEVEL_H264_2;  break;
            case 21: level = VK_V_LEVEL_H264_21; break;
            case 22: level = VK_V_LEVEL_H264_22; break;
            case 30: level = VK_V_LEVEL_H264_3;  break;
            case 31: level = VK_V_LEVEL_H264_31; break;
            case 32: level = VK_V_LEVEL_H264_32; break;
            case 40: level = VK_V_LEVEL_H264_4;  break;
            case 41: level = VK_V_LEVEL_H264_41; break;
            case 42: level = VK_V_LEVEL_H264_42; break;
            case 50: level = VK_V_LEVEL_H264_5;  break;
            case 51: level = VK_V_LEVEL_H264_51; break;
            case 52: level = VK_V_LEVEL_H264_52; break;
            default: level = VK_V_LEVEL_UNKNOWN;
        }
    } else if (codec_id == AV_CODEC_ID_HEVC) {
        switch (ffprofile) {
            case FF_PROFILE_HEVC_MAIN:     profile = VK_V_PROFILE_HEVC_MAIN; break;
            case FF_PROFILE_HEVC_MAIN_10:  profile = VK_V_PROFILE_HEVC_MAIN10; break;
            default: profile = VK_V_PROFILE_UNKNOWN;
        }
        // in HEVC, the provided level (general_level_idc) is 30 times the effective level
        // in accordance with ITU-T Rec H.265 section A.4.1
        switch (fflevel) {
            case  30: level = VK_V_LEVEL_HEVC_1;  break;
            case  60: level = VK_V_LEVEL_HEVC_2;  break;
            case  63: level = VK_V_LEVEL_HEVC_21; break;
            case  90: level = VK_V_LEVEL_HEVC_3;  break;
            case  93: level = VK_V_LEVEL_HEVC_31; break;
            case 120: level = VK_V_LEVEL_HEVC_4;  break;
            case 123: level = VK_V_LEVEL_HEVC_41; break;
            case 150: level = VK_V_LEVEL_HEVC_5;  break;
            case 153: level = VK_V_LEVEL_HEVC_51; break;
            default: level = VK_V_LEVEL_UNKNOWN;
        }
    } else if (codec_id == AV_CODEC_ID_VP9) {
        switch (ffprofile) {
            case FF_PROFILE_VP9_0: profile = VK_V_PROFILE_VP9_0; break;
            case FF_PROFILE_VP9_1: profile = VK_V_PROFILE_VP9_1; break;
            case FF_PROFILE_VP9_2: profile = VK_V_PROFILE_VP9_2; break;
            case FF_PROFILE_VP9_3: profile = VK_V_PROFILE_VP9_3; break;
            default: profile = VK_V_PROFILE_UNKNOWN;
        }
    }
    return (((profile & 0xffff) << 16) | (level & 0xffff));
}

static vk_size vkapi_get_size(int width, int height)
{
    vk_size size;

    size.width = width;
    size.height = height;

    if (size.width & 1)
        size.width++;

    if (size.height & 1)
        size.height++;

    return size;
}

static int vkapi_internal_init(AVCodecContext *avctx)
{
    int ret, profile_level, codec, pix_fmt;
    vk_size size;
    unsigned int fps;
    VKAPIDecodeContext *ctx = avctx->priv_data;
    AVHWFramesContext *hwframe_ctx = (AVHWFramesContext *)ctx->hwframe->data;
    vk_port_id port;
    VKAPIFramesContext *vk_framectx = hwframe_ctx->hwctx;

    ret = ctx->hwctx->ilapi->init((void **)(&vk_framectx->ilctx));
    if (ret)
        goto fail_init;

    av_log(avctx, AV_LOG_DEBUG, "vk_framectx=%p, vk_framectx->ilctx=%p", vk_framectx, vk_framectx->ilctx);
    vk_framectx->ilctx->context_essential.component_role = VK_DECODER;
    vk_framectx->ilctx->context_essential.queue_id = vkil_get_processing_pri();

    // first phase init, allocate a decoder context
    ret = ctx->hwctx->ilapi->init((void **)(&vk_framectx->ilctx));
    if (ret)
        goto fail_init;

    // normally, the decoder can start to decode the stream header allowing to set the below parameter.
    // the first decoding to set the decoder can be done eitehr in HW or SW
    // (currently it is done in SW, so frame size is already set into avctx)
    av_assert0(avctx->width && avctx->height);

    // We set the decoder parameter (allowing to load the proper firmware, and do the proper memory allocation)
    // if parameter are invalid, the set_parameter value will EINVAL code

    switch (avctx->codec_id) {
        case AV_CODEC_ID_H264: codec = VK_V_STANDARD_H264; break;
        case AV_CODEC_ID_HEVC: codec = VK_V_STANDARD_HEVC; break;
        case AV_CODEC_ID_VP9:  codec = VK_V_STANDARD_VP9;  break;
        default: codec = VK_V_STANDARD_UNKNOWN;
    }

    profile_level = vkapi_get_vkprofilelevel(avctx->profile, avctx->level, avctx->codec_id);
    size  = vkapi_get_size(avctx->width, avctx->height);

    av_log(avctx, AV_LOG_DEBUG, "profile_level=%d codec=%d size=(%dx%d) \n",
           profile_level, codec, size.width, size.height);
    ret = ctx->hwctx->ilapi->set_parameter(vk_framectx->ilctx, VK_PARAM_VIDEO_CODEC,
                                           &codec, VK_CMD_OPT_BLOCKING);
    if (ret)
        goto fail;

    ret = ctx->hwctx->ilapi->set_parameter(vk_framectx->ilctx, VK_PARAM_VIDEO_SIZE,
                                           &size, VK_CMD_OPT_BLOCKING);
    if (ret)
        goto fail;

    ret = ctx->hwctx->ilapi->set_parameter(vk_framectx->ilctx, VK_PARAM_VIDEO_PROFILEANDLEVEL,
                                           &profile_level, VK_CMD_OPT_BLOCKING);
    if (ret)
        goto fail;

    ret = pix_fmt = ctx->hwctx->av2vk_fmt(avctx->pix_fmt);
    if (ret < 0)
        goto fail;

    ret = ctx->hwctx->ilapi->set_parameter(vk_framectx->ilctx, VK_PARAM_VIDEO_FORMAT,
                                           &pix_fmt, VK_CMD_OPT_BLOCKING);
    if (ret)
        goto fail;

    fps = (avctx->framerate.num << 16) | (avctx->framerate.den & 0xFFFF);
    ret = ctx->hwctx->ilapi->set_parameter(vk_framectx->ilctx, VK_PARAM_VIDEO_DEC_FPS,
                                           &fps, VK_CMD_OPT_BLOCKING);
    if (ret)
        goto fail;

    // Once done, we can complete the decoder initialization
    ret = ctx->hwctx->ilapi->init((void **)(&vk_framectx->ilctx));
    if (ret)
        goto fail;

    // get the size of the hw decoder output pool buffer
    port.direction = VK_OUTPUT_PORT;
    port.id  = 0;

    // the function will overwrite the port field with the pool size, which
    // will then be available in output_port.map
    ret = ctx->hwctx->ilapi->get_parameter(vk_framectx->ilctx, VK_PARAM_POOL_SIZE,
                                           &port, VK_CMD_OPT_BLOCKING);
    if (ret)
        goto fail;

    vk_framectx->port_id = 0;  // and from which outport it comes

    if (!hwframe_ctx->pool) {
        hwframe_ctx->format = AV_PIX_FMT_VKAPI;
        hwframe_ctx->sw_format = avctx->sw_pix_fmt;
        hwframe_ctx->width = avctx->width;
        hwframe_ctx->height = avctx->height;
        hwframe_ctx->initial_pool_size = port.map;
        ret = av_hwframe_ctx_init(ctx->hwframe);
        if (ret)
            goto fail;
    }

    // get the max lag from the card
    ret = ctx->hwctx->ilapi->get_parameter(vk_framectx->ilctx, VK_PARAM_MAX_LAG,
                                           &ctx->max_lag, VK_CMD_OPT_BLOCKING);
    if (ret)
        goto fail;

    // get the min lag from the card
    ret = ctx->hwctx->ilapi->get_parameter(vk_framectx->ilctx, VK_PARAM_MIN_LAG,
                                           &ctx->min_lag, VK_CMD_OPT_BLOCKING);
    if (ret)
        goto fail;

    ctx->dts_queue = av_fifo_alloc(ctx->max_lag * sizeof(uint64_t));
    if (!ctx->dts_queue) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    // retrieve potential sps and pps Annex B formatted NAL units from extradata
    // see ITU-T h264 or ITIU-T h265 Annex B for definition
    if ((avctx->extradata_size >= 3 && AV_RB24(avctx->extradata) == 1) ||
          (avctx->extradata_size >= 4 && AV_RB32(avctx->extradata) == 1)) {
          av_log(avctx, AV_LOG_VERBOSE,
                 "Extra data looking like to be Annex B (so not processed by hxxx_mp4toannexb), \
                  they are sent to the VK cards \n");
          ctx->send_extradata = 1;
    }

    // add color informations
    vk_framectx->color.range = avctx->color_range;
    vk_framectx->color.primaries = avctx->color_primaries;
    vk_framectx->color.trc = avctx->color_trc;
    vk_framectx->color.space = avctx->colorspace;

    return 0;

fail_init:
    av_log(avctx, AV_LOG_ERROR, "Failed to init decoder ret = %d \n", ret);
fail:
    ret = vkapi_error_handling(avctx, ret, __func__);
    if (vk_framectx->ilctx)
        ctx->hwctx->ilapi->deinit((void **)&vk_framectx->ilctx);
    return ret;
}

/**
 * init the hw decoder
 * @param avctx
 * @return error code.
 */
static av_cold int vkapi_decode_init(AVCodecContext *avctx)
{
    int ret;
    VKAPIDecodeContext *ctx;
    AVHWFramesContext *hwframe_ctx;
    AVHWDeviceContext *hwdevice_ctx;

    av_assert0(avctx);
    av_assert0(avctx->priv_data);

    ctx = avctx->priv_data;
    ctx->flush = 0;
    ctx->received_frames = 0;
    ctx->sent_packets = 0;
    ctx->start_bad_pkts = 0;
    ctx->exit_on_error = 0;

    av_log(avctx, AV_LOG_DEBUG, "vkapi_decode_init avctx->width = %d, avctx->height = %d\n",
           avctx->width, avctx->height);

    // Accelerated transcoding scenarios with 'ffmpeg' require that the
    // pix_fmt be set to AV_PIX_FMT_VKAPI early. The sw_pix_fmt, and the
    // pix_fmt for non-accelerated transcoding, do not need to be correct
    // but need to be set to something.
    ret = ff_get_format(avctx, pix_fmts);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "ff_get_format failed\n");
        goto fail;
    }

    avctx->pix_fmt = ret;

    if (avctx->hw_frames_ctx) {
        ctx->hwframe = av_buffer_ref(avctx->hw_frames_ctx);
        if (!(ctx->hwframe))
            goto fail_alloc;

        hwframe_ctx = (AVHWFramesContext *)ctx->hwframe->data;

        ctx->hwdevice = av_buffer_ref(hwframe_ctx->device_ref);
        if (!(ctx->hwdevice))
            goto fail_alloc;
    } else {
        if (avctx->hw_device_ctx) {
            ctx->hwdevice = av_buffer_ref(avctx->hw_device_ctx);
            if (!(ctx->hwdevice))
                goto fail_alloc;
        } else {
            ret = av_hwdevice_ctx_create(&ctx->hwdevice, AV_HWDEVICE_TYPE_VKAPI, NULL, NULL, 0);
            if (ret)
                goto fail;
        }

        ctx->hwframe = av_hwframe_ctx_alloc(ctx->hwdevice);
        if (!(ctx->hwframe))
            goto fail_alloc;

        hwframe_ctx = (AVHWFramesContext *)ctx->hwframe->data;
    }

    hwdevice_ctx = hwframe_ctx->device_ctx;
    ctx->hwctx = hwdevice_ctx->hwctx;
    ret = vkapi_internal_init(avctx);
    if (ret)
        goto fail;

    return 0;

fail_alloc:
    ret = AVERROR(ENOMEM);
fail:
    av_log(avctx, AV_LOG_ERROR, "failure %d\n", ret);
    if (ctx->exit_on_error)
        exit_program(VK_EXIT_ERROR);
    else
        vkapi_decode_close(avctx);
    return ret;
}


/**
 * send packet to the hw decoder
 * @param avctx
 * @param avpkt
 * @return error code (zero if succesful)
 */
static int vkapi_send_packet(AVCodecContext *avctx, const AVPacket *avpkt)
{
    int written_bytes, ret;
    vkil_buffer_packet hw_packet_desc;
    VKAPIDecodeContext *ctx = avctx->priv_data;
    VKAPIDeviceContext *hwctx = ctx->hwctx;
    AVPacket extended_packet = { 0 };
    AVHWFramesContext *hwframe_ctx = (AVHWFramesContext *)ctx->hwframe->data;
    VKAPIFramesContext *vk_framectx = hwframe_ctx->hwctx;
    // arguments sanity check
    av_assert0(avctx);
    av_assert0(avpkt);

    if (ctx->flush) // we don't send anymore packet in flush mode
        return AVERROR_EOF;

    if ((ctx->send_extradata) && (avctx->extradata_size) > 0 && (avctx->extradata)) {
        int i = 0;

        ret = av_new_packet(&extended_packet, avpkt->size + avctx->extradata_size);
        if (ret < 0)
            goto fail;

        memcpy(extended_packet.data, avctx->extradata, avctx->extradata_size);
        memcpy(extended_packet.data + avctx->extradata_size , avpkt->data, avpkt->size);

        ret = av_packet_copy_props(&extended_packet, avpkt);
        if (ret < 0)
            goto fail;

        avpkt = &extended_packet;
        av_log(avctx, AV_LOG_DEBUG, "send extra data to VK card\n");

        av_log(avctx, AV_LOG_INFO, "Raw SPS/PPS: ");
        for (i = 0; i < avctx->extradata_size; i++)
            av_log(avctx, AV_LOG_INFO, "\\x%x", avctx->extradata[i]);
        av_log(avctx, AV_LOG_INFO, "\n");

        ctx->send_extradata = 0;
    }

    // reference the buffer to a vkil structure
    hw_packet_desc.prefix.type = VKIL_BUF_PACKET;
    hw_packet_desc.prefix.ref = 0;
    hw_packet_desc.prefix.port_id = 0;
    hw_packet_desc.data = avpkt->data;
    // transferred size need always to be a 32 bit multiple
    hw_packet_desc.size = (avpkt->size + 0x3) & (~0x3);
    hw_packet_desc.used_size = avpkt->size;
    hw_packet_desc.prefix.user_data = avpkt->pts;

    if (avpkt->size == 0) {
        av_log(avctx, AV_LOG_DEBUG, "send an EOS packet to decoder \n");
        // end of stream, send an extra void packet (undefined handle)
        hw_packet_desc.prefix.handle = VK_BUF_EOS;
        // and output what is still in the buffers
        ctx->lock = ctx->flush = VK_CMD_OPT_BLOCKING; // to force to wait for flushed buffer
        ret = hwctx->ilapi->process_buffer(vk_framectx->ilctx, &hw_packet_desc, VK_CMD_RUN);
        if (ret < 0)
            goto fail_buf_proc;
        return 0;
    }

    av_assert0(avpkt->buf); // here we don't know what to do if we receive empty packet

    // upload the NAL data to the card
    // because the packet data can be released after this function, currently we require a blocking function
    // (meaning, the function return once the transfer is effectivey completed)
    written_bytes = hwctx->ilapi->transfer_buffer(vk_framectx->ilctx, &hw_packet_desc,
                                                  VK_CMD_UPLOAD | VK_CMD_OPT_BLOCKING);
    if (written_bytes < 0){
        ret = written_bytes == -EAGAIN? AVERROR(EAGAIN) : written_bytes;
        av_log(avctx, AV_LOG_ERROR, "send_buffer failed on error %d\n", written_bytes);
        goto fail_buf_proc;
    }

    // At this time, we assume we are able to transfer the whole buffer to the HW, in one shot, it could be not always
    // the case we have no control on the input bitstream size, and hw work on a best guess. say 4MB buffer size max.
    // so it could be some case we could have to start the decoding of a frame, before the full frame is transferred to
    //  the HW to allow the HW input buffer to drain a bit, before continuing to populate it.
    // av_assert0(avpkt->size==written_bytes); // assume a full transfer has been done.

    // once the packet buffer has been uploaded to the vk hw, we can launch the decoding of it

    ret = hwctx->ilapi->process_buffer(vk_framectx->ilctx, &hw_packet_desc, VK_CMD_RUN);
    if (ret < 0)
        goto fail_buf_proc;
    ctx->sent_packets++;
    av_log(avctx, AV_LOG_DEBUG,
           "sent_packets=%ld, packet %p, size %d, dts=%s, pts=%s, flags=0x%x, hwbuffer=0x%x\n",
           ctx->sent_packets, avpkt, avpkt->size, av_ts2str(avpkt->dts), av_ts2str(avpkt->pts), avpkt->flags, hw_packet_desc.prefix.handle);

    av_packet_unref(&extended_packet);
    return written_bytes; // if AV_CODEC_FLAG_TRUNCATED is not set, assume the decoder always drain the full packet

fail_buf_proc:
    if (ret == -EADV)
        ctx->exit_on_error = 1;
    vkapi_error_handling(avctx, ret, __func__);
    ret = AVERROR(EINVAL);
fail:
    av_log(avctx, AV_LOG_ERROR, "%s packet #%ld, %p failed\n",
           __func__, ctx->sent_packets + 1, avpkt);
    av_packet_unref(&extended_packet);
    if (ctx->exit_on_error)
        exit_program(VK_EXIT_ERROR);
    return ret;
}

/**
 * retrieve decoded frame from the hw decoder
 * @param avctx
 * @param frame
 * @return AVERROR(EAGAIN) if no frame, AVERROR_EOF if end of sequence, zero if successful, other value error code.
 */
static int vkapi_receive_frame(AVCodecContext *avctx, AVFrame *frame)
{
    int ret, written_bytes, is_blocking, valid_format, terminate_on_eos_timeout = 0;
    int32_t lag;
    vk_size coded_size;
    AVFrame *tmp_frame = NULL;
    VKAPIDecodeContext *ctx;
    VKAPIDeviceContext *hw_devicectx;
    vkil_buffer_surface hw_surface_desc = {.prefix.type = VKIL_BUF_SURFACE};
    VKAPIFramesContext *hw_framectx;
    AVHWFramesContext *avhw_framectx;
    AVHWFramesContext *hwframe_ctx;
    VKAPIFramesContext *vk_framectx;

    av_assert0(frame);
    av_assert0(avctx);
    av_assert0(avctx->priv_data);

    ctx = avctx->priv_data;
    av_assert0(ctx->hwctx);

    hw_devicectx = ctx->hwctx;
    is_blocking = ctx->flush;

    hwframe_ctx = (AVHWFramesContext *)ctx->hwframe->data;
    vk_framectx = hwframe_ctx->hwctx;

    // notice this care about counter wrapping
    lag = ctx->sent_packets - ctx->received_frames;

    // if the lag between the # of received frame and # of sent packet is equal or greater than the input buffer size
    // it can means this input buffer can be full (so the HW will not be able to accept the next packet
    //
    // This situation can be due to 2 facts
    // 1 - the host try to feed the input buffer as a rate greater than the HW can process it
    // 2 - the HW can't process the input buffer because its output buffer is full and the host is not flushing it
    //
    // To prevent that we force this app to work in constant lock step, that is we ensure we control exactly the number of intransit
    // buffer.
    // the lag between the input and output is provided by the card, and is designed to ensure that the card will be still able to provide
    // and input for each and every packet provided
    //
    // the constant lockstepping is not required for a good functioning of the decoder only, however it is required for a good functioning
    // of the whole transcoding pipe in tunneled mode (or zero copy mode
    //
    // In a typical transcoding scenario (decoder+scaler+encoder)
    // the encoder consume input in lockstep withe decoder input (when the scaler consume its input in lockstep with the decoder output)
    // so that to enable a tight buffering control between the scaler output and encoder input, we are required to work in lockstep
    // (one input buffer condumed = one outputbuffer produced) on the whole pipe

    if (!ctx->lock) {
        // To ensure decoding in lockstep with a constant delay between fed packets
        // and output frames, we always wait until an output buffer is available.
        // Except during start or on reception of an eos packet (in such case the ctx->lock is set
        if (lag < ctx->min_lag) {
            return AVERROR(EAGAIN);
        } else if ((lag == ctx->min_lag) && (!ctx->interlace)) {
            ret = ctx->hwctx->ilapi->get_parameter(vk_framectx->ilctx, VK_PARAM_IS_STREAM_INTERLACE,
                                                  &ctx->interlace, VK_CMD_OPT_BLOCKING);
            if (ret < 0)
                goto fail;

            if (ctx->interlace)
                ctx->min_lag *= 2;

            av_log(avctx, AV_LOG_DEBUG, "ctx->interlace = %d, ctx->min_lag = %d\n", ctx->interlace, ctx->min_lag);
        }

        if (lag >= ctx->min_lag)
            ctx->lock = VK_CMD_OPT_BLOCKING;
    }

    // once we are lock, we release frame in lock step with input packet, that is we maintain a constant lag
    // until we reach the end of the stream
    if ((lag < ctx->min_lag ) && (!ctx->flush))
        return AVERROR(EAGAIN);

    //for interlace streams ensure we send 2 streams before we move to
    // blocking call. So if lag is odd send an input frame and
    // implement lock-step for even frames only
    if (ctx->interlace && (lag & VKDEC_LAG_IS_ODD) && (!ctx->flush))
        return AVERROR(EAGAIN);

    is_blocking = ctx->lock;

    ret = hw_devicectx->ilapi->process_buffer(vk_framectx->ilctx, &hw_surface_desc,
                                              is_blocking | VK_CMD_OPT_CB);
    if (ret == -EAGAIN)
        return AVERROR(EAGAIN); // no buffer delivered yet, will have to try again
    else if (ret < 0) {
        // if timeout happens after sending EOS packet, then exit gracefully with warnings.
        if ((ret == -ETIMEDOUT) && ctx->flush) {
            av_log(avctx, AV_LOG_WARNING,
                   "Exiting after decoding %ld frames. Possible loss of %d frames \n",
                   ctx->received_frames, ctx->min_lag);
            terminate_on_eos_timeout = 1;
        } else {
            // all other error code are more fatal
            av_log(avctx, AV_LOG_ERROR, "receive_buffer failed %d\n", ret);
            if (ret == -EADV) {
                goto fail;
            } else {
                /* retrieve error if any */
                vkapi_error_handling(avctx, -EADV, __func__);
                ctx->exit_on_error = 1;
                ret = AVERROR(EINVAL);
                goto out;
            }
        }
    }

    if (hw_surface_desc.prefix.handle == VK_DEC_OUT_INVALID_HDL){
        ctx->sent_packets--;
        if (!ctx->received_frames) {
            if (!ctx->start_bad_pkts)
                av_log(avctx, AV_LOG_WARNING, "Invalid or corrupted packet at start of the stream\n");
            ctx->start_bad_pkts++;
        } else if (!ctx->interlace) {
            av_log(avctx, AV_LOG_WARNING, "Corrupted packet in the stream. Appr frame #%ld\n", ctx->received_frames);
        } else {
            av_log(avctx, AV_LOG_DEBUG, "invalid packet or packet not paired with a frame\n");
        }
        return AVERROR(ENODATA);
    } else if ((hw_surface_desc.prefix.handle == VK_BUF_EOS) || terminate_on_eos_timeout) {
        if (!terminate_on_eos_timeout)
            av_log(avctx, AV_LOG_DEBUG, "end of stream detected\n");

        if (!ctx->received_frames)
             av_log(avctx, AV_LOG_WARNING,
                    "Tried to process %ld packets. No frames decoded, its an empty/fully corrupted input file\n",
                    ctx->start_bad_pkts);

        if (ctx->received_frames != ctx->sent_packets) {
             ret = ctx->hwctx->ilapi->get_parameter(vk_framectx->ilctx, VK_PARAM_IS_STREAM_INTERLACE,
                                                    &ctx->interlace, VK_CMD_OPT_BLOCKING);
             if (ret)
                 goto fail;
             if (!ctx->interlace) {
                 av_log(avctx, AV_LOG_ERROR,
                        "Rx EOS. Input/Output mismatch, %ld sent packet generated %ld decoded frames \n",
                        ctx->sent_packets, ctx->received_frames);
                 /*
                  * in case the FW did return an EOS before FFMPEG receives all the frames, the
                  * FW probably will not send any further packet.  In this case, we simulate
                  * an EOF, instead of returning ERR.  If error is returned, since the lag is
                  * positive and flush is still ON, FFMPEG will end in an infinite loop trying
                  * to process a buffer.
                  */
                 ctx->received_frames = ctx->sent_packets;
             }
        }
        ctx->flush = 0;
        return AVERROR_EOF;
    } else if (!ctx->received_frames) {
        if (ctx->start_bad_pkts)
            av_log(avctx, AV_LOG_WARNING, "Total invalid or corrupted packets at start of stream : %ld\n", ctx->start_bad_pkts);

        // if we are here it means we have a valid first buffer as received_frames is not yet updated
        if (!ctx->interlace) {
            ret = ctx->hwctx->ilapi->get_parameter(vk_framectx->ilctx, VK_PARAM_IS_STREAM_INTERLACE,
                                                  &ctx->interlace, VK_CMD_OPT_BLOCKING);
            if (ret < 0)
                goto fail;

            if (ctx->interlace)
                ctx->min_lag *= 2;
        }

        // Get the size in case it has changed after decoding SPS/PPS
        ret = ctx->hwctx->ilapi->get_parameter(vk_framectx->ilctx, VK_PARAM_VIDEO_SIZE,
                                               &coded_size, VK_CMD_OPT_BLOCKING);
        if (ret)
            goto fail;

        if ((coded_size.width != avctx->width) || (coded_size.height != avctx->height)) {
            av_log(avctx, AV_LOG_WARNING, "coded display size (%dx%d) is cropped to (%dx%d) as expected by avctx\n",
                   coded_size.width, coded_size.height, avctx->width, avctx->height);
        }
    }

    written_bytes = ret;
    valid_format = hw_devicectx->fmt_is_in(avctx->pix_fmt, pix_fmts);

    if (avctx->pix_fmt == AV_PIX_FMT_VKAPI) {
        // we just going to return a hw frame
        ret = av_hwframe_get_buffer(ctx->hwframe, frame, 0);
        if (ret)
            goto fail;

        av_log(avctx, AV_LOG_VERBOSE, "frame %p, data tunneled frame->hw_frames_ctx=%p\n", frame, frame->hw_frames_ctx);
        // we populate it with the field we know (witdh / height)
        // notice we don't read it from the hw (the hw will have barked if those fields was
        // not properly set
        ret = ff_decode_frame_props(avctx, frame);
        if (ret)
            goto fail;

        av_assert0(frame->hw_frames_ctx->data);
        avhw_framectx = (AVHWFramesContext *)frame->hw_frames_ctx->data;

        av_assert0(avhw_framectx->hwctx);
        hw_framectx = avhw_framectx->hwctx;

        // handle to the hw surface passed to the hw frame_frame_ctx
        hw_framectx->handle = hw_surface_desc.prefix.handle;
        hw_devicectx->frame_set_hwprops(frame, &hw_surface_desc);
    } else if (valid_format) {
        tmp_frame = av_frame_alloc();
        if (!tmp_frame) {
            av_log(avctx, AV_LOG_ERROR, "av_frame_alloc failed\n");
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        ret = av_hwframe_get_buffer(ctx->hwframe, tmp_frame, 0);
        if (ret)
            goto fail;

        av_assert0(tmp_frame->hw_frames_ctx->data);
        avhw_framectx = (AVHWFramesContext *)tmp_frame->hw_frames_ctx->data;

        av_assert0(avhw_framectx->hwctx);
        hw_framectx = avhw_framectx->hwctx;

        hw_framectx->handle = hw_surface_desc.prefix.handle;
        ret = hw_devicectx->frame_set_hwprops(tmp_frame, &hw_surface_desc);
        if (ret)
            goto fail;

        ret = ff_get_buffer(avctx, frame, 0);
        if (ret)
            goto fail;

        ret = av_hwframe_transfer_data(frame, tmp_frame, 0);
        if (ret)
            goto fail;

        av_frame_free(&tmp_frame);
    } else {
        ret = AVERROR_BUG;
        goto fail;
    }

    frame->pts = hw_surface_desc.prefix.user_data;
#if FF_API_PKT_PTS
FF_DISABLE_DEPRECATION_WARNINGS
    frame->pkt_pts = hw_surface_desc.prefix.user_data;
FF_ENABLE_DEPRECATION_WARNINGS
#endif

    ctx->received_frames++;
    av_log(avctx, AV_LOG_DEBUG, "received frames=%ld, frame->pts=%s, hw_handle=0x%x\n",
           ctx->received_frames, av_ts2str(frame->pts), hw_surface_desc.prefix.handle);

    ret = written_bytes;

out:
    return ret;

fail:
    if (ret == -EADV)
        ctx->exit_on_error = 1;
    ret = vkapi_error_handling(avctx, ret, __func__);
    av_log(avctx, AV_LOG_ERROR, "%s frame # %ld %p failed\n",
           __func__, ctx->received_frames + 1, frame);
    if (tmp_frame)
        av_frame_free(&tmp_frame);
    if (ctx->exit_on_error)
        exit_program(VK_EXIT_ERROR);
    return ret;
}

/**
 * send packet/receive frame
 * @param avctx
 * @param got_frame set to 1 if a frame has been received, zero otherwise
 * @return  error code.
 */
static int vkapi_decode(AVCodecContext *avctx, AVFrame *frame)
{
    int ret, got_frame;
    uint64_t dts = AV_NOPTS_VALUE;
    AVPacket avpkt = {0};
    VKAPIDecodeContext *ctx = avctx->priv_data;

    av_assert0(avctx);
    av_assert0(frame);

    // drain the output buffer first
    got_frame =  vkapi_receive_frame(avctx, frame);

 #define BUG_SOC_11836 1

 #ifdef BUG_SOC_11836
        if (ctx->interlace) {
            ret =  AVERROR(EINVAL);
            av_log(avctx, AV_LOG_ERROR, "interlaced stream not supported %d\n", ret);
            exit_program(VK_EXIT_ERROR);
        }
 #endif

    if (got_frame == AVERROR(EAGAIN)) {
        // we have drained the output, so we can feed more packet to the input
        ret = ff_decode_get_packet(avctx, &avpkt);
        if (ret < 0 && ret != AVERROR_EOF)
            goto out;

        av_fifo_generic_write(ctx->dts_queue, &avpkt.dts, sizeof(uint64_t), NULL);

        if (!ctx->flush) { // we don't send anymore packet in flush mode
            ret = vkapi_send_packet(avctx, &avpkt);
            if (ret < 0 && ret != AVERROR_EOF) {
                // notice that a EAGAIN request is processed as an error by ffmpeg
                // calling function the packet will not be represented, so such
                // should never happen
                av_log(avctx, AV_LOG_ERROR, "vkapi_send_packet failed on error %d \n", ret);
                goto out;
           }
        }
    } else if (got_frame != AVERROR_EOF) {
        av_fifo_generic_read(ctx->dts_queue, &dts, sizeof(uint64_t), NULL);
        // the decoding time stamp needs to increment on each frame outputted, so the reason
        // we keep them in a qeueu (to manage the asynchrnous behavior of the VK decoder)
        if (got_frame != AVERROR(ENODATA))
            frame->best_effort_timestamp = guess_correct_pts(avctx, frame->pts, dts);
        else
            got_frame = AVERROR(EAGAIN);
    }
    ret = got_frame;

out:
    av_packet_unref(&avpkt);
    if ((ret < 0) && (ret != AVERROR_EOF) && (ret != AVERROR(EAGAIN)) && ctx->exit_on_error)
        exit_program(VK_EXIT_ERROR);
    return ret;
}

static const AVCodecHWConfigInternal *vkapi_hw_configs[] = {
    &(const AVCodecHWConfigInternal) {
        .public = {
            .pix_fmt     = AV_PIX_FMT_VKAPI,
            .methods     = AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX |
                           AV_CODEC_HW_CONFIG_METHOD_INTERNAL,
            .device_type = AV_HWDEVICE_TYPE_VKAPI
        },
        .hwaccel = NULL,
    },
    NULL
};

static const AVOption options[]={
    {NULL}
};

#define FF_VKAPI_DEC_CLASS(NAME) \
    static const AVClass ff_vkapi_##NAME##_dec_class = { \
        .class_name = "vkapi_" #NAME "_dec", \
        .item_name  = av_default_item_name, \
        .option     = options, \
        .version    = LIBAVUTIL_VERSION_INT, \
    };

#define FF_VKAPI_DEC(NAME, ID, BSF_NAME) \
    FF_VKAPI_DEC_CLASS(NAME) \
    AVCodec ff_##NAME##_vkapi_decoder = { \
        .name           = #NAME "_vkapi", \
        .long_name      = NULL_IF_CONFIG_SMALL(#NAME " (vkapi)"), \
        .type           = AVMEDIA_TYPE_VIDEO, \
        .id             = ID, \
        .bsfs           = BSF_NAME, \
        .priv_data_size = sizeof(VKAPIDecodeContext), \
        .init           = vkapi_decode_init, \
        .close          = vkapi_decode_close, \
        .receive_frame  = vkapi_decode, \
        .flush          = vkapi_decode_flush, \
        .priv_class     = &ff_vkapi_##NAME##_dec_class, \
        .capabilities   = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_AVOID_PROBING | AV_CODEC_CAP_HARDWARE, \
        .pix_fmts       = (const enum AVPixelFormat[]) { AV_PIX_FMT_VKAPI, \
                                                         AV_PIX_FMT_YUV420P, \
                                                         AV_PIX_FMT_NONE}, \
        .hw_configs     = vkapi_hw_configs, \
        .wrapper_name   = "vkapi", \
    };

#if CONFIG_H264_VKAPI_DECODER
FF_VKAPI_DEC(h264, AV_CODEC_ID_H264, "h264_mp4toannexb")
#endif

#if CONFIG_HEVC_VKAPI_DECODER
FF_VKAPI_DEC(hevc, AV_CODEC_ID_HEVC, "hevc_mp4toannexb")
#endif

#if CONFIG_VP9_VKAPI_DECODER
FF_VKAPI_DEC(vp9, AV_CODEC_ID_VP9, NULL)
#endif
