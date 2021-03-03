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

#ifndef AVUTIL_HWCONTEXT_VKAPI_H
#define AVUTIL_HWCONTEXT_VKAPI_H

#include "vkil_api.h"

#define VKAPI_METADATA_PLANE 4

/**
 * This struct is allocated as AVHWDeviceContext.hwctx
 */
typedef struct _VKAPIDeviceContext {
    vkil_api     *ilapi; //!< connect to the vkil api
    // some internal function to transfer property to/from vkil structure to ffmpeg ones
    int32_t (*frame_ref_hwprops)(const AVFrame *frame, void *phw_surface_desc);
    int32_t (*frame_set_hwprops)(AVFrame *frame, const vkil_buffer_surface *hw_surface_desc);
    int32_t (*frame_get_hwprops)(const AVFrame *frame, vkil_buffer_surface *hw_surface_desc);
    // other utilities function used by various vkapi components
    int (*fmt_is_in)(int fmt, const int *fmts);
    int (*av2vk_fmt)(enum AVPixelFormat pixel_format);
    int (*get_pool_occupancy)(AVHWFramesContext *ctx);
} VKAPIDeviceContext;

/**
 * color information
 */
typedef struct _VKAPIColorContext {
    enum AVColorRange range;
    enum AVColorPrimaries primaries;
    enum AVColorTransferCharacteristic trc;
    enum AVColorSpace space;
    enum AVChromaLocation chroma_location;
} VKAPIColorContext;

/**
 * This struct is allocated as  AVHWFramesContext.hwctx
 */
typedef struct _VKAPIFramesContext {
    uint32_t handle;     //!< handle to a hw frame context
    uint32_t port_id;    //!< HW component port associated to the frame context
    uint32_t extra_port_id;
    VKAPIColorContext color;
    vkil_context *ilctx; //! ilcontext associated to the frame context
} VKAPIFramesContext;

#endif /* AVUTIL_HWCONTEXT_VKAPI_H */
