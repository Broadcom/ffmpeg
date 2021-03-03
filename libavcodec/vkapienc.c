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

#include <strings.h>
#include <time.h>
#include "avcodec.h"
#include "hwconfig.h"
#include "internal.h"
// overide the default value with value large enough for lookahead
#define FF_BUFQUEUE_SIZE 128
#include "libavfilter/bufferqueue.h"
#include "libavutil/avassert.h"
#include "libavutil/buffer.h"
#include "libavutil/common.h"
#include "libavutil/eval.h"
#include "libavutil/fifo.h"
#include "libavutil/hwcontext_vkapi.h"
#include "libavutil/imgutils.h"
#include "libavutil/log.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "libavutil/timestamp.h"

// timeout of seconds for EOS
#define VKENC_FLUSH_TIMEOUT_US (5 * 1000000L)
#define VKENC_FLUSH_EXTENDED_TIMEOUT_US (20 * 1000000L)
#define _ELAPSED_US(_end, _st) \
    ((_end.tv_sec - _st.tv_sec) * 1000000L + (_end.tv_nsec - _st.tv_nsec) / 1000L)
// periodic yield during flush, delay 100 ms if no message
#define VKENC_FLUSH_YIELD_US   (100 * 1000)

//! align val to the closest multiple of align, (align must be a power of 2)
#define VK_ALIGN_UP(val, align) (((val) + (align) - 1) & ~((align) - 1))
#define VK_DIV_RND_UP(val, align) (((val) + (align) - 1) / (align))
#define OPT(STR) else if( !strcmp( key, STR ) )

//! DMA transfer requires the transfer size to be a multiple of 4
#define VK_DMA_SIZE_ALIGN 4

#define VK_BITS_PER_QP  4
#define VK_NUM_IN_CFG_FILES 2 //! number of input configuration files

typedef enum _vk_cfg_inputs {
    VK_CFG_QPMAP = 0,
    VK_CFG_VARSMAP = 1,
    VK_CFG_QUALITY = 2
} vk_cfg_inputs;

#define VK_SIZE_MASK 0xFFFFFF // for BUG_SOC_12095
#define VK_MAX_KEY_LENGTH 255
#define VK_MAX_LINE_LENGTH 2048

/* download data from message for ssim size full */
#define SSIM_FULL_DATA_SIZE_BYTES 24
#define SIZE_OF_HANDLE_BYTES 4
#define SSIM_DATA_IN_HANDLE (SSIM_FULL_DATA_SIZE_BYTES / SIZE_OF_HANDLE_BYTES)
#define MAX_SSIM_SBSIZE 9

/*
 * extra data on end of QPMAP to store offline data. We use VK_DMA_SIZE_ALIGN
 * (=4) to preserve alignment
 */
#define QPMAP_EXTRA_OFFLINE_BYTES VK_DMA_SIZE_ALIGN
#define QPMAP_FRAME_QP_OFFSET 0
#define QPMAP_SEGMENT_QP_OFFSET 1

/*
 * Macro to derive the number of 4x4 blocks in an image of
 * width W and Height H
 */
#define NUM_4x4_BLKS(W, H) ((((W) / 4) - 1) * (((H) / 4) - 1))
#define NUM_4x4_GRID_PT(SURF_SZ) (((SURF_SZ) / 4) - 1)
#define SB_ROUNDUP(LOG2_SB_SZ) ((1 << (LOG2_SB_SZ)) - 1)

#define NUM_SB(SURF_SZ, LOG2_SB_SZ) ((NUM_4x4_GRID_PT(SURF_SZ) + \
                                     SB_ROUNDUP(LOG2_SB_SZ)) >> \
                                     (LOG2_SB_SZ))

typedef enum _coding_mode {
    VK_CM_BINARY  = 0,
    VK_CM_CSV     = 1 //< comma separated value
} coding_mode;

typedef enum _frame_type {
    FRAME_TYPE_I = 0,
    FRAME_TYPE_P = 1,
    FRAME_TYPE_B = 2
} frame_type;

const enum AVPixelFormat vkapi_pix_fmts[] = {
    AV_PIX_FMT_NV12,
    AV_PIX_FMT_P010,
    AV_PIX_FMT_VKAPI,
    AV_PIX_FMT_NONE
};

typedef struct VKAPIFileHeader {
    FILE * file;
    int32_t size;
    int32_t nmemb;
    int32_t memb_size;
    int32_t width;
    int32_t height;
    int32_t version;
    int32_t coding_mode;
    int32_t csvheader;
    char *type;
    void *data;
    void *ancillary_data;
    void *ext_fields; // extra fields storage if csvheader is specified
    char buff[VK_MAX_LINE_LENGTH];
    char *tokens;

} VKAPIFileHeader;

typedef struct VKAPIConfig {
    char *infile[VK_NUM_IN_CFG_FILES];
    char *statsfile;
    char *statsfile_bin;
    char *me_statsfile;
    char *ssimfile;
    char *offline_shotchangefile;
    char *qpmap_outputfile;
    VKAPIFileHeader inmap[VK_NUM_IN_CFG_FILES];
    FILE *stats;
    FILE *stats_bin;
    FILE *me_stats;
    VKAPIFileHeader ssim;
    FILE *offline_shotchange;
    int shotchange_input_present;
    VKAPIFileHeader qpmap_output;
    uint32_t pass;
    vk_enc_cfg cfg;
} VKAPIConfig;

typedef struct VKAPIEncodeContext {
    AVClass *av_class;

    // configuration parameters
    int bframe_delay; //!< decoding delay (in frames) inferred by the stream b-frames structure
    int64_t bframe_delay_time; //!< decoding delay
    int64_t first_pts; // first recorded pts
    int64_t largest_pts; // largets recorded pts
    VKAPIConfig cfg;
    char *vk_opts;

    AVFifoBuffer *pts_queue;
    // vk encoder parameters
    AVBufferRef  *hwdevice; //!< allow a reference to the hw device
    VKAPIDeviceContext *devctx; //!< handy shortcut used to the hwdevice field
    vkil_context *ilctx; //!< the vk context for the encoder
    int32_t flush; //!< indicate we have to flush the encoder buffers (end of stream)
    struct timespec flush_time;
    // stats download memory ptr
    void *stats_data;
    int   stats_size;
    void *me_stats_data;
    int   me_stats_size;
    int32_t max_lag;
    int32_t input_size;
    int32_t input_occ;
    int64_t send_frames;
    int64_t eos_send_idx;
    int64_t received_packets;
    int64_t stream_size;
    struct FFBufQueue bufqueue;
    /* set when end of stream is done */
    int eos_flag;
    // offline status
    uint32_t next_shotchange;
    uint8_t offline_delta_qp;
    // qp data buffer for offline pass 1 output
    void *qp_buffer;
    uint32_t qp_buffer_offset;
    uint32_t qp_buffer_frames;
    vkil_buffer_metadata metadata[VK_NUM_IN_CFG_FILES + 1];
} VKAPIEncodeContext;

/**
 * @brief initialize the multipass algorithms with default values
 * @param[in|out] multipass parameters
 */
static void init_multipass_cfg(vk_lookahead_cfg *cfg)
{
    // Whether to use the static min QP mechanism in the rate_controller.
    // Off for now, but left exposed so that it can be tested and tuned
    // in the future
    cfg->smqp = 0;

    // AQ parameters exposed for now to permit possible tuning. Will either be
    // added to the main interface or pushed down into the AQ code in due course
    cfg->taq_strength = 2;
    cfg->saq_strength = 1;
    cfg->saq_a = 0.8;
    cfg->saq_b = 7.5;

    // default number of GOPs in a segment
    cfg->seg_gops = 4;

    // And defaults for shotchange parameters.
    //
    // Our algorithm is the same as x264's, so we use the same default
    // threshold (their --scenecut parameter)
    // Shotlength_min and max are chosen to be 1 and 10s at 25fps. We
    // adjust the max to be a multiple of the gop length to prevent
    // unnecessary extra I frames. (The x264 equivalent, --keyint is
    // 250).
    cfg->shotchange_threshold = 40;
    cfg->shotlength_min = 25;
}

/**
 * Setting the stats configuration parameters
 * @param encoder context
 * @param ssim configration parameters
 * @return zero if success otherwise error code
 */
static int32_t get_stats_size(AVCodecContext *avctx)
{
    int32_t size_ctu = VK_DIV_RND_UP(avctx->width, 32) * VK_DIV_RND_UP(avctx->height, 32);

    return (sizeof(vk_surface_stats) + size_ctu * sizeof(vk_ctu_stats));
}

/**
 * Get the stats header size
 * @param encoder context
 * @return the size of the encoder stats header
 */
static int32_t get_stats_size_small(AVCodecContext *avctx)
{
    return (2 * sizeof(vk_surface_stats));
}

/**
 * Get the me stats size
 * @param encoder context
 * @return the size of the me_stats buffer needed for the whole frame
 */
static int32_t get_me_stats_size(AVCodecContext *avctx)
{
    int32_t size_ctu = ((avctx->width + 31) / 32) * ((avctx->height + 31) / 32)
                            * 2 * 2;
    return (size_ctu * sizeof(vk_me_stats));
}

// TODO@CRB: get from encoder?
static int32_t get_qpmap_size(AVCodecContext *avctx)
{
    int32_t size_ctu = VK_DIV_RND_UP(avctx->width, 32) * VK_DIV_RND_UP(avctx->height, 32);

    return (sizeof(uint32_t) * size_ctu);
}

static int vkapi_fgets(VKAPIFileHeader *hdr)
{
    char *token = NULL;
    void *ret;

    if (hdr->coding_mode == VK_CM_CSV) {
        if (hdr->tokens)
            token = strtok_r(NULL, ",", &hdr->tokens);
        if (!token) {
            ret = fgets(hdr->buff, VK_MAX_LINE_LENGTH, hdr->file);
            if (ret == NULL)
                goto fail; /* end of file */

            token = strtok_r(hdr->buff, ",", &hdr->tokens);
            if (!token)
                goto fail; /* end of file */
        }
        return atoi(token);
    }

fail:
    av_log(NULL, AV_LOG_ERROR, "failure to read csv value from file %p\n", hdr->file);
    return AVERROR(EINVAL);
}

static int vkapi_fread(VKAPIFileHeader *hdr, int *extrafield)
{
    char *retp, *token = NULL;
    int ret, i = 0;

    if (hdr->coding_mode == VK_CM_CSV) {
        retp = fgets(hdr->buff, VK_MAX_LINE_LENGTH, hdr->file);
        if (retp == NULL)
            goto fail; /* end of file */

        token = strtok_r(hdr->buff, ",", &hdr->tokens);
        if (!token)
            goto fail; /* end of file */

        while (token && (i < hdr->csvheader)) {
            av_assert0(extrafield);
            extrafield[i++] = strtoul(token,NULL,0);
            token = strtok_r(NULL, ",", &hdr->tokens);
        }

        i = 0;
        while (token && (i < hdr->nmemb)) {
            ((int *)hdr->data)[i++] = strtoul(token,NULL,0);
            token = strtok_r(NULL, ",", &hdr->tokens);
        }
    } else {
        ret = fread(hdr->data, 1, hdr->size, hdr->file);
        if (ret != hdr->size) {
            av_log(NULL, AV_LOG_ERROR, "fread failed, read only %d bytes instead of %d bytes\n",
                   ret, hdr->size);
            goto fail;
        }
    }
    return 0;

fail:
    av_log(NULL, AV_LOG_ERROR, "failure to read csv value from file %p\n", hdr->file);
    return AVERROR(EINVAL);
}


static int vkapi_write_ssimheader(VKAPIFileHeader *hdr, const int log_sb_plus1)
{
    int num_y_comp;
    int num_uv_comp;
    int i;
    int ret = fprintf(hdr->file,
                      "<header>type=vk-ssim-map:version=0:size=%d:surface-width=%d:surface-height=%d:log_sb_plus1=%d:coding_format=%s</header>\n",
                      hdr->size, hdr->width, hdr->height,log_sb_plus1, hdr->coding_mode == VK_CM_CSV ? "csv": "bin");
    if (ret < 0)
        goto fail;

    if (hdr->coding_mode == VK_CM_CSV) {
        /* Calculate the number of SSIM values for Y and UV components */
        num_y_comp = NUM_SB(((hdr->width - 2)), log_sb_plus1 - 1) * NUM_SB(((hdr->height - 2)), log_sb_plus1 - 1);
        num_uv_comp= NUM_SB(((hdr->width - 2) >> 1), log_sb_plus1 - 1) * NUM_SB(((hdr->height - 2) >> 1), log_sb_plus1 - 1);

        /* Add the y_ssim, u_ssim, v_ssim to the header file */
        fprintf(hdr->file, "frame_number,y_ssim");
        for (i = 0; i < num_y_comp; ++i)
             fprintf(hdr->file, ",");
        fprintf(hdr->file, "u_ssim");
        for (i = 0; i < num_uv_comp; ++i)
             fprintf(hdr->file, ",");
        fprintf(hdr->file, "v_ssim");
        for (i = 0; i < num_uv_comp; ++i)
             fprintf(hdr->file, ",");
        fprintf(hdr->file, "\n");
    }
    return 0;

fail:
    av_log(NULL, AV_LOG_ERROR, "failed to write file header for %p", hdr);
    return ret;
}

static int vkapi_write_ssimdata( const void * ptr, uint32_t size,
                                 VKAPIFileHeader *hdr, const int log_sb_plus1,
                                 const int64_t frame_number )
{
    int i, ret;
    double tmp;
    double max = 0;
    double min = 1;
    double comp_avg = 0;
    const uint64_t *lptr = ptr;
    int32_t wr_size = 0;
    unsigned int y_num_4x4;
    unsigned int uv_num_4x4;
    unsigned int width, height;
    unsigned int y_sb;
    unsigned int uv_sb;
    unsigned int y_hor_sblks;
    unsigned int y_vert_sblks;
    unsigned int num_y_comp;
    char l_csv_log[VK_MAX_LINE_LENGTH];
    char one_value[20]; /* 64 bit double has max 16 digits */

    width = hdr->width;
    height = hdr->height;

    /* Get the total num of 4x4 blks in luma and chroma plane */
    y_num_4x4  = NUM_4x4_BLKS((width - 2), (height - 2));
    uv_num_4x4 = NUM_4x4_BLKS(((width - 2) / 2), ((height - 2) / 2));

    y_sb = (1 << ((log_sb_plus1 - 1) * 2));
    uv_sb = y_sb;

    /* Calculate the number of SSIM values for Y component */
    y_hor_sblks   = NUM_SB(((width - 2)), log_sb_plus1 - 1);
    y_vert_sblks  = NUM_SB(((height - 2)), log_sb_plus1 - 1);
    num_y_comp    = y_hor_sblks * y_vert_sblks;

    if (hdr->coding_mode == VK_CM_CSV) {
        if (frame_number) {
            ret = sprintf(l_csv_log, "%ld,", frame_number);
            if (ret < 0)
                return -EINVAL;
            wr_size += ret;
        }
        if ((log_sb_plus1 == 0) || (y_sb > y_num_4x4))
            y_sb = y_num_4x4;

        if ((log_sb_plus1 == 0) || (uv_sb > uv_num_4x4))
            uv_sb = uv_num_4x4;

        for (i = 0; i < (size / sizeof(*lptr)); i++) {
            tmp = (double)(*lptr);
            // Thought value are stored on 64 bits
            // a ssim value is a signed 32 + 2*log_sb bits data
            // where the 31 + 2*log_sb lsb are fractional part
            // for output in cvs format, we convert this value into a float
            if(i < (num_y_comp)) {
                tmp /= ((double)y_sb);
            } else {
                tmp /= ((double)uv_sb);
            }
            tmp /= ((double)((uint64_t)1l << 31));
            comp_avg += tmp;
            if (tmp > max)
                max = tmp;
            if (tmp < min)
                min = tmp;
            sprintf(one_value, "%f,", tmp);
            if (wr_size + strlen(one_value) + 1 >= sizeof(l_csv_log))
                break;
            ret = sprintf(l_csv_log + wr_size, "%s", one_value);
            if (ret < 0) {
                ret = fwrite(l_csv_log, 1, wr_size, hdr->file);
                if (ret != wr_size)
                    return AVERROR(EINVAL); // failed to write full dat on file
                wr_size = 0;
            } else {
                wr_size += ret;
            }
            lptr++;
        }
        ret = sprintf(l_csv_log + wr_size, "\n");
        if (ret < 0)
            return -EINVAL;
        wr_size += ret;
        ret = fwrite(l_csv_log, 1, wr_size, hdr->file);
        if (ret != wr_size)
            return AVERROR(EINVAL); // failed to write full dat on file
    } else {
        ret = fwrite(ptr, 1, size, hdr->file);
        if (ret != size)
            return AVERROR(EINVAL); // failed to write full dat on file
    }
    return 0;
}


static void header_deinit(VKAPIFileHeader *hdr)
{
    if (hdr->file)
        fclose(hdr->file);
    if (hdr->data)
        av_freep(&hdr->data);
    if (hdr->ext_fields)
        av_freep(&hdr->ext_fields);
    if (hdr->type)
        free(hdr->type);
}

static int vkapi_search_key(FILE *file, const char *key)
{
    int ret = 0, i = 0;
    char buffer[VK_MAX_KEY_LENGTH + 1];
    int c, record = 0;

    do {
        c = fgetc(file);
        if (c == '<') // start
            record = 1;

        if (record)
            buffer[i++] = c;

        if (i > VK_MAX_KEY_LENGTH)
            break;

        if (c == '>') { // end
            buffer[i] = '\0';
            if (!strcmp(buffer, key))
                break;

            record = 0;
            i = 0;
        }
    } while (c != EOF);

    if ((c == EOF) || (i > VK_MAX_KEY_LENGTH))
        //meet end of file before key or meet a too long key to be instrumented
        return -EINVAL;

    ret = ftell(file) - i;
    return ret;
}

#define check_type_range(key, param, minvalue, maxvalue, value)      \
    do {                                                             \
        long i_;                                                     \
        char *end_ = NULL;                                           \
                                                                     \
        i_ = strtol(value, &end_, 10);                               \
        if (end_ == value) {                                         \
            av_log(NULL, AV_LOG_ERROR, "Could not convert value"     \
                   " %s for parameter %s", value, key);              \
            goto fail;                                               \
        }                                                            \
                                                                     \
        if (i_ > maxvalue || i_ < minvalue) {                        \
            av_log(NULL, AV_LOG_ERROR, "Error: %s = %ld is"          \
                   " out of variable range [%ld, %ld]\n",            \
                   key, i_, (long)minvalue, (long)maxvalue);         \
            goto fail;                                               \
        }                                                            \
        param = i_;                                                  \
    } while (0)

static int32_t header_parse(VKAPIFileHeader *hdr, const char* key, const char* value)
{
    // there is no control on the provided value done here
    // control is done by the HW, which will return an error if a value is not
    // properly set
    if( 0 );
    OPT("size")
        check_type_range("size", hdr->size, INT32_MIN, INT32_MAX, value);
    OPT("surface-width")
        check_type_range("surface-width", hdr->width, INT32_MIN, INT32_MAX, value);
    OPT("surface-height")
        check_type_range("surface-height", hdr->height, INT32_MIN, INT32_MAX, value);
    OPT("version")
        check_type_range("version", hdr->version, INT32_MIN, INT32_MAX, value);
    OPT("nmemb")
        check_type_range("nmemb", hdr->nmemb, INT32_MIN, INT32_MAX, value);
    OPT("csvheader")
        check_type_range("csvheader", hdr->csvheader, INT32_MIN, INT32_MAX, value);
    OPT("type")
       hdr->type = strdup(value);
    OPT("coding-mode")
    {
        if (!strcasecmp(value, "csv"))
            hdr->coding_mode =  VK_CM_CSV;
    }
    else
        goto fail;

    return 0;

fail:
    return AV_LOG_ERROR;
}

static int vkapi_read_header(char **buffer, VKAPIFileHeader *header, const char *xmltag)
{
    int start, end, ret, pos;
    char key[VK_MAX_KEY_LENGTH];

    av_assert0(header->file);

    *buffer = NULL;

    ret = snprintf (key, VK_MAX_KEY_LENGTH, "<%s>", xmltag); // build the open tag
    if ((ret <= 0) || (ret > VK_MAX_KEY_LENGTH))
        goto fail;
    start = vkapi_search_key(header->file, key);
    if (start < 0)
        goto fail;

    start += strlen(key);
    ret = snprintf (key, VK_MAX_KEY_LENGTH, "</%s>", xmltag); // build the closing tag
    if ((ret <= 0) || (ret > VK_MAX_KEY_LENGTH))
        goto fail;

    end = vkapi_search_key(header->file, key);
    if (end < 0)
        goto fail;

    if (start >= end)
        goto fail;

    *buffer = av_mallocz(end - start + 1);

    if (!*buffer)
        goto fail;

    pos = ftell (header->file);
    if (pos < 0)
        goto fail;

    ret = fseek(header->file, start, SEEK_SET);
    if (ret < 0)
        goto fail;

    ret = fread(*buffer, 1, end - start, header->file);
    if (ret != (end - start))
        goto fail;

    // this allow to point to the start of the payload
    end = fseek(header->file, pos, SEEK_SET);
    if (end < 0)
        goto fail;

    return 0;

fail:
    if (*buffer)
        free(*buffer);
    return AV_LOG_ERROR;
}

/**
 * init the file header
 * @param hdr file header
 * @param filename file name
 * @return zero if success, error code otherwise
 */
static int header_init(VKAPIFileHeader *hdr, char *filename)
{
    int ret;
    char *retp, *buffer = NULL;
    AVDictionary *dict    = NULL;
    AVDictionaryEntry *en = NULL;

    hdr->file = fopen(filename,"r");
    if (!hdr->file) {
        av_log(NULL, AV_LOG_ERROR, "input file %s doesn't exist\n", filename);
        ret = AVERROR(EINVAL);
        goto fail;
    }
    ret = vkapi_read_header(&buffer, hdr, "header");
    if (ret)
        goto fail;

    // Now parse the header parameters configuration
    if (!av_dict_parse_string(&dict, buffer, "=", ":", 0)) {
       while ((en = av_dict_get(dict, "", en, AV_DICT_IGNORE_SUFFIX))) {
                ret = header_parse(hdr, en->key, en->value);
                if (ret)
                    av_log(NULL, AV_LOG_WARNING,
                           "unknown header parameter '%s = %s'.\n",
                            en->key, en->value);
        }
        av_dict_free(&dict);
    }

    if ((hdr->coding_mode == VK_CM_CSV) && hdr->nmemb)
        hdr->size *= hdr->nmemb; // that will be the memroy allocation

    if (hdr->csvheader) {
        retp = fgets(hdr->buff, VK_MAX_LINE_LENGTH, hdr->file);
        if (retp == NULL) {
            ret = AVERROR(EINVAL);
            goto fail; // end of file
        }
        // give 1 extra room as csvheader value may be 0
        hdr->ext_fields = av_mallocz((hdr->csvheader + 1) * sizeof(int));
        if (!hdr->ext_fields) {
            av_log(NULL, AV_LOG_ERROR, "Failed to allocate buffer for csv extra field\n");
            ret = AVERROR(ENOMEM);
            goto fail;
        }
    }

    if (hdr->size) {
        hdr->data = av_mallocz(hdr->size);
        if (!hdr->data) {
            av_log(NULL, AV_LOG_ERROR, "Failed to allocate buffer for file's data\n");
            ret = AVERROR(ENOMEM);
            goto fail;
        }
    }

    if (buffer)
        free(buffer);

    memset(hdr->buff, 0, VK_MAX_LINE_LENGTH);
    hdr->tokens = NULL;

    return 0;

fail:
    if (buffer)
        free(buffer);

    return ret;
}

/**
 * init the file header for output (for a qpmap, just requiring a file)
 * @param hdr file header
 * @param filename file name
 * @return zero if success, error code otherwise
 */
static int header_init_out(VKAPIFileHeader *hdr, char *filename, uint32_t size) {
    int ret;
    hdr->file = fopen(filename,"w");
    if (!hdr->file) {
        av_log(NULL, AV_LOG_ERROR, "can't open %s for write\n", filename);
        ret = AVERROR(EINVAL);
        return ret;
    }
    fprintf(hdr->file,"<header>size=%d</header>",size);
    hdr->size = size;
    hdr->data = av_mallocz(hdr->size);
    if (!hdr->data) {
        av_log(NULL, AV_LOG_ERROR, "Failed to allocate buffer for file's data\n");
        ret = AVERROR(ENOMEM);
        return ret;
        }
    return 0;
}

/**
 * Write the stats buffer in a particular format as expected by codec.
 * @param encoder context
 * @param stats buffer data
 * @param size of the data
 * @param file pointer to formatted output(text format)
 * @param file pointer to bin format of stats buffer
 */
static void write_stats_buf(AVCodecContext *avctx, char *data, uint32_t size,
                            FILE* fp, FILE* fp_bin)
{
    vk_ctu_stats *pctu;
    uint32_t row, col;
    char frm_type = 'I';
    vk_surface_stats *stats_buf = (vk_surface_stats *)data;
    vk_ctu_stats *ctu = (vk_ctu_stats *)(data + sizeof(vk_surface_stats));

    if (!fp)
        goto write_bin;

    if (stats_buf->frame_type == FRAME_TYPE_I)
        frm_type = 'I';
    else if (stats_buf->frame_type == FRAME_TYPE_P)
        frm_type = 'P';
    else if (stats_buf->frame_type == FRAME_TYPE_B)
        frm_type = 'B';

    fprintf(fp, "encoder_stats_type:%d\n", stats_buf->encoder_stats_type);
    fprintf(fp, "frame_type:%c\n", frm_type);
    fprintf(fp, "used_as_reference:%d\n", stats_buf->used_as_reference);
    fprintf(fp, "qp:%d\n", stats_buf->qp);
    fprintf(fp, "picture_count:%d\n", stats_buf->picture_count);
    fprintf(fp, "num_cols:%d\n", stats_buf->num_cols);
    fprintf(fp, "num_rows:%d\n", stats_buf->num_rows);
    fprintf(fp, "ref_pic_count:%d,%d\n", stats_buf->ref_pic_count[0],
            stats_buf->ref_pic_count[1]);

    for (row = 0; row < stats_buf->num_rows; row++) {
        pctu = &ctu[row  * (stats_buf->num_cols)];
        for (col = 0; col < stats_buf->num_cols; col++)
            fprintf(fp, "%d,%d intra_count:%d bit_estimate:%d luma_mean:%d luma_cplx:%d rmv:%d,%d\n",
                    col, row, pctu[col].intra_count, pctu[col].bit_estimate,
                    pctu[col].luma_mean, pctu[col].luma_cplx,
                    (int16_t)pctu[col].rmv_x, (int16_t)pctu[col].rmv_y);
    }

write_bin:
    if (fp_bin)
        fwrite(data, 1, size, fp_bin);
}

/**
 * Write the me stats buffer as binary data
 * @param me_stats buffer data
 * @param size of the data
 * @param file pointer to write to
 */
static void write_me_stats_buf(char *data, uint32_t size, FILE* fp)
{
    if (fp)
        fwrite(data, 1, size, fp);
}

static int atobool(const char *str)
{
    if( !strcmp(str, "1") ||
        !strcasecmp(str, "true") ||
        !strcasecmp(str, "yes"))
        return 1;

    if( !strcmp(str, "0") ||
        !strcasecmp(str, "false") ||
        !strcasecmp(str, "no"))
        return 0;


    return -EINVAL;
}

/**
 * Get the matched string's idx in an array
 * @param array of strings to be matched
 * @param limit to check
 * @return idx of the matched string, negative error
 */
static int32_t get_match_str_idx(const char *str_arr[], const uint32_t max_idx,
                                 const char *to_match)
{
    int32_t i;

    for (i = 0; i < max_idx; i++) {
        if (str_arr[i] && !strcasecmp(str_arr[i], to_match))
            return i;
    }
    return -EINVAL;
}

static int32_t param_parse(AVCodecContext *avctx, VKAPIConfig *cfg, const char* key, const char* value)
{
    char *tail;
    double intpart, fracpart;

    // there is no control on the provided value done here
    // control is done by the HW, which will return an error if a value is not
    // properly set
    if( 0 );
    OPT("qp") {
        check_type_range("qp", cfg->cfg.rc_cfg.qpi, 0, UINT8_MAX, value);
        cfg->cfg.rc_cfg.flags |= VK_ENC_CFG_QPI_SET;
    }
    OPT("qpmin") {
        check_type_range("qpmin", cfg->cfg.rc_cfg.min_qp, 0, UINT8_MAX, value);
        cfg->cfg.rc_cfg.flags |= VK_ENC_CFG_MIN_QP_SET;
    }
    OPT("qpmax") {
        check_type_range("qpmax", cfg->cfg.rc_cfg.max_qp, 0, UINT8_MAX, value);
        cfg->cfg.rc_cfg.flags |= VK_ENC_CFG_MAX_QP_SET;
    }
    OPT("dqpp") {
        check_type_range("dqpp", cfg->cfg.rc_cfg.dqpp, INT8_MIN, INT8_MAX, value);
        cfg->cfg.rc_cfg.flags |= VK_ENC_CFG_DQPP_SET;
    }
    OPT("dqpb") {
        check_type_range("dqpb", cfg->cfg.rc_cfg.dqpb, INT8_MIN, INT8_MAX, value);
        cfg->cfg.rc_cfg.flags |= VK_ENC_CFG_DQPB_SET;
    }
    OPT("dqpd") {
        check_type_range("dqpd", cfg->cfg.rc_cfg.dqpd, INT8_MIN, INT8_MAX, value);
        cfg->cfg.rc_cfg.flags |= VK_ENC_CFG_DQPD_SET;
    }
    OPT("write_colr"){
        cfg->cfg.color_cfg.flags = atobool(value) ? VK_CFG_FLAG_ENABLE : 0;
    }
    OPT("varsfile")
    {
        cfg->infile[VK_CFG_VARSMAP] = strdup(value);
        cfg->cfg.varmap_cfg.flags = VK_CFG_FLAG_ENABLE;
    }
    OPT("qpfile")
    {
        cfg->infile[VK_CFG_QPMAP] = strdup(value);
        if (cfg->cfg.qpmap_cfg.flags == VK_CFG_FLAG_ENABLE) {
            av_log(avctx, AV_LOG_ERROR, "qpfile specified but there is already a qpfile or offline data");
            goto fail;
        }
        cfg->cfg.qpmap_cfg.flags = VK_CFG_FLAG_ENABLE;
    }
    OPT("offline-data")
    {
        // Use the same qpfile mechanism
        cfg->infile[VK_CFG_QPMAP] = strdup(value);
        if (cfg->cfg.qpmap_cfg.flags == VK_CFG_FLAG_ENABLE) {
            av_log(avctx, AV_LOG_ERROR, "offline data specified but there is already a qpfile or offline data");
            goto fail;
        }
        cfg->cfg.qpmap_cfg.flags = VK_CFG_FLAG_ENABLE;
    }
    OPT("ssim-log")
    {
        cfg->ssimfile = strdup(value);
        cfg->cfg.ssim_cfg.flags = VK_CFG_FLAG_ENABLE;
    }
    OPT("ssim-sbsize") {
        uint8_t sbsize;

        if (!strcasecmp(value, "full"))
            cfg->cfg.ssim_cfg.log_sb_plus1 = 0;
        else {
            check_type_range("ssim-sbsize", sbsize, 0, UINT8_MAX - 1, value);
            if (sbsize < MAX_SSIM_SBSIZE + 1)
                cfg->cfg.ssim_cfg.log_sb_plus1 = sbsize + 1;
            else
                goto fail;
        }
    }
    OPT("ssim-mode") {
        if (!strcasecmp(value, "csv"))
            cfg->ssim.coding_mode = VK_CM_CSV;
        else if (!strcasecmp(value, "bin"))
            cfg->ssim.coding_mode = VK_CM_BINARY;
        else
            goto fail;
    }
    OPT("stats")
    {
        cfg->statsfile = strdup(value);
        cfg->cfg.stats_cfg.flags = VK_CFG_FLAG_ENABLE;
    }
    OPT("stats-bin")
    {
        cfg->statsfile_bin = strdup(value);
        cfg->cfg.stats_cfg.flags = VK_CFG_FLAG_ENABLE;
    }
    OPT("me-stats")
    {
        cfg->me_statsfile = strdup(value);
        cfg->cfg.me_stats_cfg.flags = VK_CFG_FLAG_ENABLE;
        cfg->cfg.me_stats_cfg.coarse_mvs = 0; /* Use full motion vector list*/
    }
    OPT("pass")
    {
	int32_t value_i;

	check_type_range("pass", value_i, INT32_MIN, INT32_MAX, value);
        if (value_i > 2 || value_i < 0) {
            av_log(avctx, AV_LOG_ERROR, "Pass must be 0, 1 or 2");
            goto fail;
        }
        cfg->cfg.lookahead_cfg.flags |= value_i;
        cfg->cfg.lookahead_cfg.flags |= VK_MULTIPASS_USE_OFFLINE;

    }
    OPT("shotchange-file")
        cfg->offline_shotchangefile = strdup(value);
    OPT("shotchange-threshold")
        check_type_range("shotchange-threshold", cfg->cfg.lookahead_cfg.shotchange_threshold,
                         0, UINT8_MAX, value);
    OPT("shotlength-min")
        check_type_range("shotlength-min", cfg->cfg.lookahead_cfg.shotlength_min,
                         0, UINT32_MAX, value);
    OPT("shotlength-max")
        check_type_range("shotlength-max", cfg->cfg.lookahead_cfg.shotlength_max,
                         0, UINT32_MAX, value);
    OPT("keyint")
        check_type_range("keyint", cfg->cfg.gop_size, 0, UINT16_MAX, value);
    OPT("bframes") {
        uint8_t bframes;
        // use +1 so that 0 is avoided which is the non-defined
        check_type_range("bframes", bframes, 0, UINT8_MAX - 1, value);
        cfg->cfg.nbframes_plus1 = bframes + 1;
    }
    OPT("gop-type") {
        int32_t idx;
        static const char *gop_type_str[VK_GOP_MAX] = {
            [VK_GOP_BIDIRECTIONAL] = "bi-dir",
            [VK_GOP_LOWDELAY]      = "low-delay",
            [VK_GOP_PYRAMID]       = "pyramid",
            [VK_GOP_PYRAMID_EXPLICIT] = "pyramid-explicit",
        };
        idx = get_match_str_idx(gop_type_str, VK_GOP_MAX, value);
        if (idx >= 0)
            cfg->cfg.gop_type = idx;
        else
            goto fail;

    }
    OPT("idr-passthrough")
        check_type_range("idr-passthrough", cfg->cfg.idr_passthrough, 0, UINT8_MAX, value);
    OPT("output-depth")
        check_type_range("output-depth", cfg->cfg.bitdepth, 0, UINT8_MAX, value);
    OPT("bitrate") {
        // bitrate can be suffixed with k unit
        double bitrate = av_strtod(value, &tail);
        if (bitrate)
            cfg->cfg.bitrate = bitrate;
        else
            goto fail;
    }
    OPT("rate-control")
    {
        int32_t idx;
        static const char *rc_mode_str[VK_RC_MAX] = {
            [VK_RC_OFF] = "off",
            [VK_RC_STD] = "std",
            [VK_RC_VBR] = "vbr",
            [VK_RC_CBR] = "cbr",
            [VK_RC_QTY] = "qty",
        };
        idx = get_match_str_idx(rc_mode_str, VK_RC_MAX, value);
        if (idx == VK_RC_QTY) {
           av_log(avctx, AV_LOG_WARNING,
                  "Rate control mode quality not for use in production.\n");
        }
        if (idx >= 0)
            cfg->cfg.rc_cfg.rc_mode = idx;
        else
            goto fail;
    }
    OPT("profile")
    {
        if (cfg->cfg.standard == VK_V_STANDARD_H264) {
            if (!strcasecmp(value, "baseline"))
                cfg->cfg.profile =  VK_V_PROFILE_H264_BASELINE;
            else if (!strcasecmp(value, "main"))
                cfg->cfg.profile =  VK_V_PROFILE_H264_MAIN;
            else if (!strcasecmp(value, "high"))
                cfg->cfg.profile =  VK_V_PROFILE_H264_HIGH;
            else
                goto fail;
        } else if (cfg->cfg.standard == VK_V_STANDARD_HEVC) {
            if (!strcasecmp(value, "main"))
                cfg->cfg.profile =  VK_V_PROFILE_HEVC_MAIN;
            else if (!strcasecmp(value, "main10"))
                cfg->cfg.profile =  VK_V_PROFILE_HEVC_MAIN10;
            else if (!strcasecmp(value, "msp"))
                cfg->cfg.profile = VK_V_PROFILE_HEVC_MAIN_STILL;
            else if (!strcasecmp(value, "mainstillpicture"))
                cfg->cfg.profile = VK_V_PROFILE_HEVC_MAIN_STILL;
            else if (!strcasecmp(value, "main-intra"))
                cfg->cfg.profile = VK_V_PROFILE_HEVC_MAIN_INTRA;
            else
                goto fail;
        } else if (cfg->cfg.standard == VK_V_STANDARD_VP9) {
            if (!strcmp(value, "0"))
                cfg->cfg.profile = VK_V_PROFILE_VP9_0;
            else if (!strcmp(value, "1"))
                cfg->cfg.profile = VK_V_PROFILE_VP9_1;
            else if (!strcmp(value, "2"))
                cfg->cfg.profile = VK_V_PROFILE_VP9_2;
            else if (!strcmp(value, "3"))
                cfg->cfg.profile = VK_V_PROFILE_VP9_3;
            else
                av_log(avctx, AV_LOG_WARNING, "Ignoring invalid setting of profile for vp9: %s\n", value);
        }
    }
    OPT("repeat-headers")
    {
        int repeatheaders = atobool(value);

        if (repeatheaders < 0)
            goto fail;
        cfg->cfg.no_repeatheaders = !repeatheaders;
    }
    OPT("level-idc")
    {
        if (cfg->cfg.standard == VK_V_STANDARD_H264) {
            if (!strcasecmp(value, "1.0"))
                cfg->cfg.level =  VK_V_LEVEL_H264_1;
            else if (!strcasecmp(value, "1.b"))
                cfg->cfg.level =  VK_V_LEVEL_H264_1b;
            else if (!strcasecmp(value, "1.1"))
                cfg->cfg.level =  VK_V_LEVEL_H264_11;
            else if (!strcasecmp(value, "1.2"))
                cfg->cfg.level =  VK_V_LEVEL_H264_12;
            else if (!strcasecmp(value, "1.3"))
                cfg->cfg.level =  VK_V_LEVEL_H264_13;
            else if (!strcasecmp(value, "2.0"))
                cfg->cfg.level =  VK_V_LEVEL_H264_2;
            else if (!strcasecmp(value, "2.1"))
                cfg->cfg.level =  VK_V_LEVEL_H264_21;
            else if (!strcasecmp(value, "2.2"))
                cfg->cfg.level =  VK_V_LEVEL_H264_22;
            else if (!strcasecmp(value, "3.0"))
                cfg->cfg.level =  VK_V_LEVEL_H264_3;
            else if (!strcasecmp(value, "3.1"))
                cfg->cfg.level =  VK_V_LEVEL_H264_31;
            else if (!strcasecmp(value, "3.2"))
                cfg->cfg.level =  VK_V_LEVEL_H264_32;
            else if (!strcasecmp(value, "4.0"))
                cfg->cfg.level =  VK_V_LEVEL_H264_4;
            else if (!strcasecmp(value, "4.1"))
                cfg->cfg.level =  VK_V_LEVEL_H264_41;
            else if (!strcasecmp(value, "4.2"))
                cfg->cfg.level =  VK_V_LEVEL_H264_42;
            else if (!strcasecmp(value, "5.0"))
                cfg->cfg.level =  VK_V_LEVEL_H264_5;
            else if (!strcasecmp(value, "5.1"))
                cfg->cfg.level =  VK_V_LEVEL_H264_51;
            else if (!strcasecmp(value, "5.2"))
                cfg->cfg.level =  VK_V_LEVEL_H264_52;
            else if (!strcasecmp(value, "6.0"))
                cfg->cfg.level =  VK_V_LEVEL_H264_6;
            else if (!strcasecmp(value, "6.1"))
                cfg->cfg.level =  VK_V_LEVEL_H264_61;
            else if (!strcasecmp(value, "6.2"))
                cfg->cfg.level =  VK_V_LEVEL_H264_62;
            else
                goto fail;
        } else if (cfg->cfg.standard == VK_V_STANDARD_HEVC) {
            if (!strcasecmp(value, "1.0"))
                cfg->cfg.level =  VK_V_LEVEL_HEVC_1;
            else if (!strcasecmp(value, "2.0"))
                cfg->cfg.level =  VK_V_LEVEL_HEVC_2;
            else if (!strcasecmp(value, "2.1"))
                cfg->cfg.level =  VK_V_LEVEL_HEVC_21;
            else if (!strcasecmp(value, "3.0"))
                cfg->cfg.level =  VK_V_LEVEL_HEVC_3;
            else if (!strcasecmp(value, "3.1"))
                cfg->cfg.level =  VK_V_LEVEL_HEVC_31;
            else if (!strcasecmp(value, "4.0"))
                cfg->cfg.level =  VK_V_LEVEL_HEVC_4;
            else if (!strcasecmp(value, "4.1"))
                cfg->cfg.level =  VK_V_LEVEL_HEVC_41;
            else if (!strcasecmp(value, "5.0"))
                cfg->cfg.level =  VK_V_LEVEL_HEVC_5;
            else if (!strcasecmp(value, "5.1"))
                cfg->cfg.level =  VK_V_LEVEL_HEVC_51;
            else if (!strcasecmp(value, "5.2"))
                cfg->cfg.level =  VK_V_LEVEL_HEVC_52;
            else if (!strcasecmp(value, "6.0"))
                cfg->cfg.level =  VK_V_LEVEL_HEVC_6;
            else if (!strcasecmp(value, "6.1"))
                cfg->cfg.level =  VK_V_LEVEL_HEVC_61;
            else if (!strcasecmp(value, "6.2"))
                cfg->cfg.level =  VK_V_LEVEL_HEVC_62;
            else
                goto fail;
        }
    }
    OPT("n-hrd")
        check_type_range("n-hrd", cfg->cfg.n_hrd, 1, UINT8_MAX, value);
    OPT("ext_adapt_quant_a")
    {
        check_type_range("ext_adapt_quant_a", cfg->cfg.adaptqp_cfg.a, INT32_MIN, INT32_MAX, value);
        cfg->cfg.adaptqp_cfg.flags |= VK_ADAPT_QP_A_SET;
    }
    OPT("ext_adapt_quant_b")
    {
        check_type_range("ext_adapt_quant_b", cfg->cfg.adaptqp_cfg.b, INT32_MIN, INT32_MAX, value);
        cfg->cfg.adaptqp_cfg.flags |= VK_ADAPT_QP_B_SET;
    }
    OPT("ext_adapt_quant_split_thresh")
    {
        check_type_range("ext_adapt_quant_split_thresh", cfg->cfg.adaptqp_cfg.split_thresh, 0, UINT8_MAX, value);
        // split_thresh is only supported with HEVC (but 0 is equivalent to
        // split_thresh disabled).
        if (cfg->cfg.adaptqp_cfg.split_thresh != 0 && cfg->cfg.standard != VK_V_STANDARD_HEVC)
            goto fail;
        cfg->cfg.adaptqp_cfg.flags |= VK_ADAPT_QP_SPLIT_THRESH_SET;
    }
    OPT("ext_adapt_quant_bpr")
    {
        check_type_range("ext_adapt_quant_bpr", cfg->cfg.adaptqp_cfg.bpr_force, INT8_MIN, INT8_MAX, value);
        cfg->cfg.adaptqp_cfg.flags |= VK_ADAPT_QP_BPR_FORCE_SET;
    }
    OPT("ext_adapt_quant_last_qpd")
    {
        check_type_range("ext_adapt_quant_last_qpd", cfg->cfg.adaptqp_cfg.last_qpd_mode, INT8_MIN, INT8_MAX, value);
        cfg->cfg.adaptqp_cfg.flags |= VK_ADAPT_QP_LAST_QPD_MODE_SET;
    }
    OPT("ext_adapt_quant_qtd_bpp")
    {
        fracpart = modf(atof(value), &intpart);
        if ((intpart < 0) || (intpart > 0xFFFF))
            goto fail;

        fracpart *= (1<<16);
        cfg->cfg.adaptqp_cfg.qpd_sum_disable_threshold_bpp = (((int)intpart) << 16) | (((int)fracpart) & 0xFFFF);
        cfg->cfg.adaptqp_cfg.flags |= VK_ADAPT_QP_QPD_SUM_DISABLE_THRESHOLD_BPP_SET;
    }
    OPT("ext_adapt_quant_sct_bpp")
    {
        fracpart = modf(atof(value), &intpart);
        if ((intpart < 0) || (intpart > 0xFFFF))
            goto fail;

        fracpart *= (1<<16);
        cfg->cfg.adaptqp_cfg.sig_cost_threshold_bpp = (((int)intpart) << 16) | (((int)fracpart) & 0xFFFF);
        cfg->cfg.adaptqp_cfg.flags |= VK_ADAPT_QP_SIG_COST_THRESHOLD_BPP_SET;
    }
    OPT("ext_adapt_quant_sct_qp")
    {
        check_type_range("ext_adapt_quant_sct_qp", cfg->cfg.adaptqp_cfg.sig_cost_threshold_qp, INT8_MIN, INT8_MAX, value);
        cfg->cfg.adaptqp_cfg.flags |= VK_ADAPT_QP_SIG_COST_THRESHOLD_QP_SET;
    }
    OPT("ext_adapt_quant_qtd_qp")
    {
        check_type_range("ext_adapt_quant_qtd_qp", cfg->cfg.adaptqp_cfg.qpd_sum_disable_threshold_qp, INT8_MIN, INT8_MAX, value);
        cfg->cfg.adaptqp_cfg.flags |= VK_ADAPT_QP_QPD_SUM_DISABLE_THRESHOLD_QP_SET;
    }
    OPT("ext_adapt_quant_qst")
    {
        check_type_range("ext_adapt_quant_qst", cfg->cfg.adaptqp_cfg.qpd_sum_threshold, INT32_MIN, INT32_MAX, value);
        cfg->cfg.adaptqp_cfg.flags |= VK_ADAPT_QP_QPD_SUM_THRESHOLD_SET;
    }
    OPT("ext_adapt_quant_mode")
    {
        check_type_range("ext_adapt_quant_mode", cfg->cfg.adaptqp_cfg.aq_mode, 0, 9, value);
        if (value != 0 && cfg->cfg.standard == VK_V_STANDARD_VP9)
            av_log(avctx, AV_LOG_WARNING, "Adaptive Quantisation is NOT recommended with VP9");

        cfg->cfg.adaptqp_cfg.flags |= VK_ADAPT_QP_AQ_MODE_SET;
    }
    OPT("lookahead-frames")
        check_type_range("lookahead-frames", cfg->cfg.lookahead_cfg.frames, 0, UINT8_MAX, value);
    OPT("seg-gops")
        check_type_range("seg-gops", cfg->cfg.lookahead_cfg.seg_gops, 0, UINT8_MAX, value);
    else
        goto fail;

    return 0;

fail:
    return AV_LOG_ERROR;
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
            default: level = VK_V_LEVEL_H264_12;
        }
    }
    return (((profile & 0xffff) << 16) | (level & 0xffff));
}

static int vkapi_check_hyperpyramid(AVCodecContext *avctx, VKAPIEncodeContext *ctx)
{
    int ret;
    uint32_t hyperpyramid_supported;

    // Find out whether hyperpyramid is supported.
    // We have to explicitly set the codec first since this affects the mini-GOP length.
    ret = ctx->devctx->ilapi->set_parameter(ctx->ilctx, VK_PARAM_VIDEO_CODEC,
                                            &ctx->cfg.cfg.standard, VK_CMD_OPT_BLOCKING);
    if (ret)
        return ret;

    ret = ctx->devctx->ilapi->get_parameter(ctx->ilctx, VK_PARAM_VIDEO_ENC_HYPERPYRAMID_SUPPORTED,
                                            &hyperpyramid_supported, VK_CMD_OPT_BLOCKING);
    if (ret)
        return ret;

    if (ctx->cfg.cfg.gop_type == VK_GOP_PYRAMID_EXPLICIT) {
        // Only support pyramid-explicit if hyperpyramid is supported.
        if (!hyperpyramid_supported) {
            av_log(avctx, AV_LOG_ERROR, "GOP type pyramid-explicit is not supported.\n");
            return AVERROR(EINVAL);
        }
        if (ctx->cfg.cfg.nbframes_plus1 == 0) {
            ctx->cfg.cfg.nbframes_plus1 = 7 + 1;
        } else if (ctx->cfg.cfg.nbframes_plus1 != 3 + 1 &&
                   (ctx->cfg.cfg.nbframes_plus1 < 7 + 1 ||
                   ctx->cfg.cfg.nbframes_plus1 > 15 + 1)) {
            av_log(avctx, AV_LOG_ERROR, "GOP type pyramid-explicit only supports B frame values 3,7-15.\n");
            return AVERROR(EINVAL);
        } else if ((ctx->cfg.cfg.lookahead_cfg.flags & VK_MULTIPASS_USE_LOOKAHEAD) &&
                   ctx->cfg.cfg.nbframes_plus1 != 3 + 1 &&
                   ctx->cfg.cfg.nbframes_plus1 != 7 + 1) {
            av_log(avctx, AV_LOG_ERROR, "Lookahead mode only supports GOP type pyramid-explicit "
                "with B frames values 3 or 7.\n");
            return AVERROR(EINVAL);
        }
    }

    // Align default GOP with vksim and vkservices
    if (ctx->cfg.cfg.gop_type == VK_GOP_UNDEF) {
        if (ctx->cfg.cfg.profile == VK_V_PROFILE_H264_BASELINE)
            // If baseline profile is selected and no gop type, we
            // enforce a gop type compatible with baseline profile
            ctx->cfg.cfg.gop_type = VK_GOP_LOWDELAY;
        else
            ctx->cfg.cfg.gop_type = VK_GOP_PYRAMID;
    }

    // If GOP type pyramid, check that bframes setting matches the default value.
    if (ctx->cfg.cfg.gop_type == VK_GOP_PYRAMID) {
        uint32_t default_bframes = hyperpyramid_supported ? 7 : 3;

        if (ctx->cfg.cfg.nbframes_plus1 != 0 &&
            ctx->cfg.cfg.nbframes_plus1 != default_bframes + 1)
            av_log(avctx, AV_LOG_WARNING, "bframes value will be overriden for gop-type=pyramid. "
                "Set bframes=%d to remove this warning or use gop-type=pyramid-explicit\n",
                default_bframes);

        ctx->cfg.cfg.nbframes_plus1 = default_bframes + 1;
    }

    return 0;
}

static int vkapi_close_config(AVCodecContext *avctx)
{
    VKAPIEncodeContext *ctx = avctx->priv_data;
    int i;

    for (i = 0; i < VK_NUM_IN_CFG_FILES ; i++) {
        if (ctx->cfg.infile[i])
            free(ctx->cfg.infile[i]);
        if (&ctx->cfg.inmap[i])
            header_deinit(&ctx->cfg.inmap[i]);
    }

    if (ctx->cfg.ssimfile) {
        header_deinit(&ctx->cfg.ssim);
        free(ctx->cfg.ssimfile);
    }

    if (ctx->cfg.statsfile)
        free(ctx->cfg.statsfile);

    if (ctx->cfg.statsfile_bin)
        free(ctx->cfg.statsfile_bin);

    if (ctx->cfg.me_statsfile)
        free(ctx->cfg.me_statsfile);

    if (ctx->cfg.offline_shotchangefile)
        free(ctx->cfg.offline_shotchangefile);

    if (ctx->cfg.stats || ctx->cfg.stats_bin) {
        av_freep(&ctx->stats_data);
        if (ctx->cfg.stats)
            fclose(ctx->cfg.stats);

        if (ctx->cfg.stats_bin)
            fclose(ctx->cfg.stats_bin);
    }

    if (ctx->cfg.me_stats) {
        av_freep(&ctx->me_stats_data);
        fclose(ctx->cfg.me_stats);
    }

    return 0;
}

static int vkapi_init_config(AVCodecContext *avctx)
{
    VKAPIEncodeContext *ctx = avctx->priv_data;
    int32_t ret, profile_level, codec;
    int32_t fps_num = avctx->framerate.num;
    int32_t fps_den = avctx->framerate.den;

    // sanity check
    av_assert0(ctx);
    // we at least expect a valid video size
    av_assert0(avctx->width && avctx->height);

    switch (avctx->codec_id) {
        case AV_CODEC_ID_H264: codec = VK_V_STANDARD_H264; break;
        case AV_CODEC_ID_HEVC: codec = VK_V_STANDARD_HEVC; break;
        case AV_CODEC_ID_VP9:  codec = VK_V_STANDARD_VP9;  break;
        default: codec = VK_V_STANDARD_UNKNOWN;
    }

    // if undefined, the hw card will infer it
    profile_level = vkapi_get_vkprofilelevel(avctx->profile, avctx->level, avctx->codec_id);

    // ensure that the fps_num/fps_den ratio can be encoded on 16 bits
    while ((fps_num > USHRT_MAX) || (fps_den > USHRT_MAX)) {
        fps_num >>= 1;
        fps_den >>= 1;
    }
    if (!fps_den)
        fps_den = 1;

    switch (avctx->pix_fmt) {
        case AV_PIX_FMT_NV12:
            ctx->cfg.cfg.format = VK_FORMAT_NV12;
            break;
        case AV_PIX_FMT_P010:
            ctx->cfg.cfg.format = VK_FORMAT_P010;
            break;
        case AV_PIX_FMT_VKAPI:
            ctx->cfg.cfg.format = VK_FORMAT_YOL2;
            break;
        default:
            ret = AVERROR(EINVAL);
            av_log(avctx, AV_LOG_ERROR, "avctx->pix_fmt = %d unsupported by HW\n", avctx->pix_fmt);
            goto fail;
    }

    // config parameters inferred from avctx,
    // however they can be  overriden by the command line vk-params parameters

    ctx->cfg.cfg.standard    = codec;
    ctx->cfg.cfg.size.width  = avctx->width;
    ctx->cfg.cfg.size.height = avctx->height;
    ctx->cfg.cfg.profile     = (profile_level >> 16) & 0xFFFF;
    ctx->cfg.cfg.level       = profile_level & 0xFFFF;
    ctx->cfg.cfg.fps = (fps_num << 16) | (fps_den & 0xFFFF);
    ctx->cfg.cfg.gop_size = avctx->gop_size;
    ctx->cfg.cfg.bitrate  = avctx->bit_rate;
    ctx->cfg.cfg.rc_cfg.flags = 0;
    ctx->cfg.cfg.rc_cfg.rc_mode  = VK_RC_UNSET;

    // color_cfg is enabled per default since to align with other ffmpeg encoder
    // it is disabled by the use of the write_colr option
    ctx->cfg.cfg.color_cfg.flags = VK_CFG_FLAG_ENABLE;

    /* convention for adapt QP is to enable, to say "read the config" */
    ctx->cfg.cfg.adaptqp_cfg.flags = VK_CFG_FLAG_ENABLE;

    init_multipass_cfg(&ctx->cfg.cfg.lookahead_cfg);

    // Now parse the vk-params configuration options
    if (ctx->vk_opts) {
        AVDictionary *dict    = NULL;
        AVDictionaryEntry *en = NULL;
        int32_t err = 0;
        if (!av_dict_parse_string(&dict, ctx->vk_opts, "=", ":", 0)) {
            while ((en = av_dict_get(dict, "", en, AV_DICT_IGNORE_SUFFIX))) {
                ret = param_parse(avctx, &ctx->cfg, en->key, en->value);
                if (ret) {
                    // we parse all the parameters first and record errors
                    av_log(avctx, AV_LOG_WARNING,
                           "Error parsing option '%s = %s'.\n",
                            en->key, en->value);
                    err = AVERROR(EINVAL);
                }
            }
            av_dict_free(&dict);
        }
        if (err) {
            ret = AVERROR(EINVAL);
            goto fail;
        }
    }

    if (ctx->cfg.cfg.color_cfg.flags & VK_CFG_FLAG_ENABLE) {

        // direct copy of color range since vk definition align with ffmpeg one
	ctx->cfg.cfg.color_cfg.range = avctx->color_range;
        // direct copy of color info possible since both ffmpeg and vk definition align on ITU-T H273 / ISO23000-8
        ctx->cfg.cfg.color_cfg.primaries =  avctx->color_primaries;
        ctx->cfg.cfg.color_cfg.transfer =  avctx->color_trc;
        ctx->cfg.cfg.color_cfg.matrix =  avctx->colorspace;
        av_log(avctx, AV_LOG_DEBUG, "Set color range=%d, primaries=%d, transfer=%d, matrix=%d/n",
               avctx->color_range, avctx->color_primaries, avctx->color_trc, avctx->colorspace);
    }

    if (ctx->cfg.cfg.lookahead_cfg.frames && !(ctx->cfg.cfg.lookahead_cfg.flags & VK_MULTIPASS_USE_OFFLINE)) {
        // If we set lookahead-frames _and_ pass, we are in offline
        ctx->cfg.cfg.lookahead_cfg.flags |= VK_MULTIPASS_USE_LOOKAHEAD; // use the lookahead mechanism
    }

    if (ctx->cfg.cfg.lookahead_cfg.flags & VK_MULTIPASS_USE_OFFLINE) {
        if (!ctx->cfg.cfg.lookahead_cfg.frames)
            ctx->cfg.cfg.lookahead_cfg.frames = 30;

        if (!ctx->cfg.cfg.lookahead_cfg.shotlength_max) {
            // Default is the nearest multiple of the gop-length to 250
            int num_gops = (int)floor(250.0 / ctx->cfg.cfg.gop_size + 0.5);
            ctx->cfg.cfg.lookahead_cfg.shotlength_max = num_gops * ctx->cfg.cfg.gop_size;
        }
    }

    /* if rc_mode wasn't set, default to off if an attempt to use fixed_qp has been made */
    if (ctx->cfg.cfg.rc_cfg.rc_mode == VK_RC_UNSET) {
        if ((ctx->cfg.cfg.rc_cfg.flags & VK_ENC_CFG_QPI_SET) ||
            (ctx->cfg.cfg.rc_cfg.flags & VK_ENC_CFG_DQPP_SET) ||
            (ctx->cfg.cfg.rc_cfg.flags & VK_ENC_CFG_DQPB_SET) ||
            (ctx->cfg.cfg.rc_cfg.flags & VK_ENC_CFG_DQPD_SET)) {
            ctx->cfg.cfg.rc_cfg.rc_mode = VK_RC_OFF;
        } else {
            ctx->cfg.cfg.rc_cfg.rc_mode = VK_RC_DEF;
        }
    }
    if (!ctx->cfg.cfg.lookahead_cfg.flags &&
        ctx->cfg.cfg.rc_cfg.rc_mode != VK_RC_OFF &&
        ((ctx->cfg.cfg.rc_cfg.flags & VK_ENC_CFG_QPI_SET) ||
         (ctx->cfg.cfg.rc_cfg.flags & VK_ENC_CFG_DQPP_SET) ||
         (ctx->cfg.cfg.rc_cfg.flags & VK_ENC_CFG_DQPB_SET) ||
         (ctx->cfg.cfg.rc_cfg.flags & VK_ENC_CFG_DQPD_SET))) {
        ret = AVERROR(EINVAL);
        av_log(avctx, AV_LOG_ERROR,
               "For single-pass, it is not valid to set qp, dqpp, dqpb or dqpd unless rate-control is set to off\n");
        goto fail;
    }
    if (((ctx->cfg.cfg.rc_cfg.flags & VK_ENC_CFG_MIN_QP_SET) &&
         !(ctx->cfg.cfg.rc_cfg.flags & VK_ENC_CFG_MAX_QP_SET)) ||
        (!(ctx->cfg.cfg.rc_cfg.flags & VK_ENC_CFG_MIN_QP_SET) &&
         (ctx->cfg.cfg.rc_cfg.flags & VK_ENC_CFG_MAX_QP_SET))) {
        ret = AVERROR(EINVAL);
        av_log(avctx, AV_LOG_ERROR,
               "Either set qpmin and qpmax or neither\n");
        goto fail;
    }

    av_log(avctx, AV_LOG_DEBUG, "varsfile in = %s\n", ctx->cfg.infile[VK_CFG_VARSMAP]);
    av_log(avctx, AV_LOG_DEBUG, "qpfile in = %s\n", ctx->cfg.infile[VK_CFG_QPMAP]);
    av_log(avctx, AV_LOG_DEBUG, "stats = %s\n",  ctx->cfg.statsfile);
    av_log(avctx, AV_LOG_DEBUG, "stats = %s\n",  ctx->cfg.statsfile_bin);
    av_log(avctx, AV_LOG_DEBUG, "me_stats = %s\n",  ctx->cfg.me_statsfile);
    return 0;

fail:
    return ret;
}

static int vkapi_warnings_retrieve(AVCodecContext *avctx, const char * const caller)
{
    vk_warning warn;
    VKAPIEncodeContext *ctx;
    int ret = 0, cnt = 0;

#define VK_FW_MAX_WARNINGS_TOPRINT 12
    // this may be called at early init failure where not much is setup up, so
    // do check individual ptrs progressively before continue.
    if (!avctx || !avctx->priv_data)
        return ret;
    ctx = avctx->priv_data;

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

static int vkapi_error_handling(AVCodecContext *avctx, int ret, const char * const caller)
{
    vk_error error;
    VKAPIEncodeContext *ctx;

    /* sanity check */
    av_assert0(avctx);
    av_assert0(avctx->priv_data);

    ctx = avctx->priv_data;

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

static int vkapi_encode_close(AVCodecContext *avctx)
{
    VKAPIEncodeContext *ctx;
    float bit_rate = 0.0;
    float frame_rate = 0.0;

    av_log(avctx, AV_LOG_DEBUG, "vkapi_encode_close\n");

    av_assert0(avctx);
    av_assert0(avctx->priv_data);

    ctx = avctx->priv_data;
    vkapi_close_config(avctx);

    ff_bufqueue_discard_all(&ctx->bufqueue);

    if (ctx->stream_size != UINT64_MAX) {
        if (avctx->framerate.num && avctx->framerate.den)
            frame_rate = (float)avctx->framerate.num / (float)avctx->framerate.den;
        bit_rate = ((float)ctx->stream_size / (float)ctx->received_packets) / 1000;
        bit_rate *= frame_rate * 8;
    }
    // else we have maxed out our counter, we consider the data not valid

    if (bit_rate)
        av_log(avctx, AV_LOG_INFO, "kb/s:%.2f\n", bit_rate);
    else
        av_log(avctx, AV_LOG_INFO, "kb/s:NA\n");

    // retrieve warnings before closing
    vkapi_warnings_retrieve(avctx, __func__);

    if (ctx->ilctx)
        ctx->devctx->ilapi->deinit((void **)&ctx->ilctx);

    av_fifo_freep(&ctx->pts_queue);

    if (ctx->hwdevice)
        av_buffer_unref(&ctx->hwdevice);

    // ctx->hwctx is deinited in hwcontext_vkapi::deinit
    return 0;
}

static int vkapi_internal_init(AVCodecContext *avctx)
{
    int i, ret, output_size, gop_type, tunneling = 0;
    VKAPIEncodeContext *ctx;
    AVHWFramesContext *hwframes_ctx;
    VKAPIFramesContext *vk_framectx;
    vk_port port;
    vkil_buffer_packet vk_packet = {.prefix.type = VKIL_BUF_PACKET};
    int32_t used_size = 0;

    // some assignment with sanity check
    av_assert0(avctx && avctx->hw_frames_ctx);
    ctx = avctx->priv_data;
    hwframes_ctx = (AVHWFramesContext *)avctx->hw_frames_ctx->data;
    av_assert0(hwframes_ctx);
    vk_framectx = hwframes_ctx->hwctx;

    if (vk_framectx->ilctx) {
        // set color from hw whenever not already specifed by avctx
        if (avctx->color_range == AVCOL_RANGE_UNSPECIFIED)
            avctx->color_range = vk_framectx->color.range;
        if (avctx->color_primaries == AVCOL_PRI_UNSPECIFIED)
            avctx->color_primaries = vk_framectx->color.primaries;
        if (avctx->color_trc == AVCOL_TRC_UNSPECIFIED)
            avctx->color_trc = vk_framectx->color.trc;
        if (avctx->colorspace == AVCOL_SPC_UNSPECIFIED)
            avctx->colorspace = vk_framectx->color.space;
        tunneling = 1;
    }

    // We set the encoder configuration
    ret = vkapi_init_config(avctx);
    if (ret)
        goto fail_init;

    ret = ctx->devctx->ilapi->init((void **)(&ctx->ilctx));
    if (ret)
        goto fail_init;

    av_assert0(!(ctx->cfg.cfg.lookahead_cfg.flags & VK_MULTIPASS_USE_OFFLINE &&
        ctx->cfg.cfg.lookahead_cfg.flags & VK_MULTIPASS_USE_LOOKAHEAD));

    if (ctx->cfg.cfg.lookahead_cfg.flags &&
        ctx->cfg.cfg.standard != VK_V_STANDARD_H264) {
        av_log(avctx, AV_LOG_ERROR, "Lookahead and offline only supported for h264\n");
        ret = AVERROR(EINVAL);
        goto fail;
    }

    if (ctx->cfg.cfg.lookahead_cfg.flags & (VK_MULTIPASS_USE_OFFLINE | VK_MULTIPASS_USE_LOOKAHEAD)) {
        ctx->ilctx->context_essential.component_role = VK_MULTIPASS_ENCODER;
    } else {
        ctx->ilctx->context_essential.component_role = VK_ENCODER;
    }

    if ((ctx->cfg.cfg.lookahead_cfg.flags & VK_MULTIPASS_USE_OFFLINE) &&
       (ctx->cfg.cfg.lookahead_cfg.flags & VK_MULTIPASS_PASS_MASK) == 1 &&
        ctx->cfg.infile[VK_CFG_QPMAP]) {
        // For offline pass 1, QPMAP is an output, so move the name across
        ctx->cfg.qpmap_outputfile = ctx->cfg.infile[VK_CFG_QPMAP];
        ctx->cfg.infile[VK_CFG_QPMAP] = NULL;
        ctx->cfg.cfg.qpmap_cfg.flags = 0;

        // Add an extra word for frame and segment qp data
        ret = header_init_out(&ctx->cfg.qpmap_output,
            ctx->cfg.qpmap_outputfile,
            VK_ALIGN_UP(get_qpmap_size(avctx),
                VK_DMA_SIZE_ALIGN) + QPMAP_EXTRA_OFFLINE_BYTES);
        if (ret)
            goto fail;
    }

    ctx->ilctx->context_essential.queue_id = vkil_get_processing_pri();

    ret = ctx->devctx->ilapi->init((void **)(&ctx->ilctx));
    if (ret)
        goto fail_init;

    ret = vkapi_check_hyperpyramid(avctx, ctx);
    if (ret)
        goto fail;

    for (i = 0; i < VK_NUM_IN_CFG_FILES; i++) {
        if (ctx->cfg.infile[i]) {
            ret = header_init(&ctx->cfg.inmap[i], ctx->cfg.infile[i]);
            if (ret)
                goto fail;

            if (i == VK_CFG_VARSMAP) {
                if (ctx->cfg.cfg.lookahead_cfg.flags) {
                    av_log(avctx, AV_LOG_ERROR,
                           "Multipass must not be run with varmap from a file");
                    ret = AVERROR(EINVAL);
                    goto fail;
                }
                if (ctx->cfg.cfg.standard == VK_V_STANDARD_VP9) {
                    av_log(avctx, AV_LOG_WARNING,
                           "Adaptive Quantisation is NOT recommended with VP9 (enabled by providing a varsmap file)\n");
                }

                ctx->cfg.cfg.varmap_cfg.flags = VK_CFG_FLAG_ENABLE;
                ctx->cfg.cfg.varmap_cfg.size  = ctx->cfg.inmap[i].size;
            } else if (i == VK_CFG_QPMAP && ctx->cfg.inmap[i].size) {
                if (ctx->cfg.cfg.standard == VK_V_STANDARD_VP9) {
                    av_log(avctx, AV_LOG_WARNING,
                           "Adaptive Quantisation is NOT recommended with VP9 (enabled by providing a qpmap file)\n");
                }

                ctx->cfg.cfg.qpmap_cfg.flags = VK_CFG_FLAG_ENABLE;
                ctx->cfg.cfg.qpmap_cfg.size  = ctx->cfg.inmap[i].size;
            }
        }
    }

    if (tunneling) {
        // in tunneled mode we connect the source output pad to the encoder input pad
        hwframes_ctx = (AVHWFramesContext *)avctx->hw_frames_ctx->data;
        vk_framectx = hwframes_ctx->hwctx;

        // first we get the source output pad (it is the default pad)
        port.port_id.id = vk_framectx->port_id;
        port.port_id.direction = VK_OUTPUT_PORT;
        ret = ctx->devctx->ilapi->get_parameter(vk_framectx->ilctx, VK_PARAM_PORT,
                                                &port, VK_CMD_OPT_BLOCKING);
        if (ret)
            goto fail;

        // then we apply it to the encoder first (default) input
        port.port_id.id = 0;
        port.port_id.direction = VK_INPUT_PORT;
        ret = ctx->devctx->ilapi->set_parameter(ctx->ilctx, VK_PARAM_PORT,
                                                &port, VK_CMD_OPT_BLOCKING);
        if (ret)
            goto fail;

        // do we have associated extra data to send
        if (vk_framectx->extra_port_id) {
            if (ctx->cfg.infile[VK_CFG_VARSMAP]) {
                av_log(avctx, AV_LOG_ERROR, "can't use variance file %s and tunneled variance map from source at the same time \n",
                       ctx->cfg.infile[VK_CFG_VARSMAP]);
                ret = AVERROR(EINVAL);
                goto fail;
            }
            if (ctx->cfg.cfg.standard == VK_V_STANDARD_VP9) {
                av_log(avctx, AV_LOG_WARNING,
                       "Adaptive Quantisation is NOT recommended with VP9 (enabled by tunnelling a varmap)\n");
            }
            ctx->cfg.cfg.varmap_cfg.flags = VK_CFG_FLAG_ENABLE;

            // first we get the source extra output pad
            port.port_id.id = vk_framectx->extra_port_id;
            port.port_id.direction = VK_OUTPUT_PORT;
            ret = ctx->devctx->ilapi->get_parameter(vk_framectx->ilctx, VK_PARAM_PORT,
                                                    &port, VK_CMD_OPT_BLOCKING);
            if (ret)
                goto fail;

            // then we apply it to the relevant encoder input
            port.port_id.id = VK_CFG_VARSMAP + 1;
            port.port_id.direction = VK_INPUT_PORT;
            ret = ctx->devctx->ilapi->set_parameter(ctx->ilctx, VK_PARAM_PORT,
                                                    &port, VK_CMD_OPT_BLOCKING);
            if (ret)
                goto fail;
        }
    }

    if (ctx->cfg.cfg.lookahead_cfg.flags) {
#define USING_LOOKAHEAD(ctx) (ctx->cfg.cfg.lookahead_cfg.flags &          \
                              VK_MULTIPASS_USE_LOOKAHEAD)
#define USING_OFFLINE_PASS(ctx, x) ((ctx->cfg.cfg.lookahead_cfg.flags &   \
                                     VK_MULTIPASS_USE_OFFLINE) &&         \
                                    (ctx->cfg.cfg.lookahead_cfg.flags &   \
                                     VK_MULTIPASS_PASS_MASK) == x)
        if (ctx->cfg.cfg.stats_cfg.flags) {
            av_log(avctx, AV_LOG_ERROR,
                   "Multipass must not be run with stats or stats-bin\n");
            ret = AVERROR(EINVAL);
            goto fail;
        }
        if (USING_OFFLINE_PASS(ctx, 1)) {
            if (!ctx->cfg.qpmap_outputfile)  {
                av_log(avctx, AV_LOG_ERROR,
                       "Multipass must be run with offline-data for offline pass 1\n");
               ret = AVERROR(EINVAL);
               goto fail;
           }
        } else if (USING_OFFLINE_PASS(ctx, 2)) {
            if (!ctx->cfg.cfg.qpmap_cfg.flags) {
                av_log(avctx, AV_LOG_ERROR,
                       "Multipass must be run with offline-data for offline pass 2\n");
               ret = AVERROR(EINVAL);
               goto fail;
           }
        } else {
            if (ctx->cfg.cfg.qpmap_cfg.flags) {
                av_log(avctx, AV_LOG_ERROR,
                      "Multipass must not be run with qpfile or offline-data except required for offline pass 1 & 2\n");
               ret = AVERROR(EINVAL);
               goto fail;
           }
        }
        /* Varmap needed for LE and Offline Pass 0 & 1, not otherwise */
        if (USING_LOOKAHEAD(ctx) ||
            USING_OFFLINE_PASS(ctx, 0) ||
            USING_OFFLINE_PASS(ctx, 1) ||
            USING_OFFLINE_PASS(ctx, 2) /* TODO remove varmap in pass 2 */) {
            if (!ctx->cfg.cfg.varmap_cfg.flags) {
                av_log(avctx, AV_LOG_ERROR,
                       "Multipass must be run with varmap enabled (via a tunnelled scaler component) for LE and Offline Pass 0 & 1\n");
                ret = AVERROR(EINVAL);
                goto fail;
            }
        } else {
            if (ctx->cfg.cfg.varmap_cfg.flags) {
                av_log(avctx, AV_LOG_ERROR,
                       "Multipass must not be run with varmap enabled (via a tunnelled scaler component) except Lookahead and Offline Pass 0 & 1\n");
                ret = AVERROR(EINVAL);
                goto fail;
            }
        }
        /* SSIM only allowed for LE and Offline Pass 2 */
        if (ctx->cfg.cfg.ssim_cfg.flags &&
            !(USING_LOOKAHEAD(ctx) ||
              USING_OFFLINE_PASS(ctx, 2))) {
            av_log(avctx, AV_LOG_ERROR,
                   "Multipass must not be run with ssim enabled for except for Lookahead and Offline Pass 2\n");
            ret = AVERROR(EINVAL);
            goto fail;
        }
#undef USING_LOOKAHEAD
#undef USING_OFFLINE_PASS
    }

    // force no repeat headers when GLOBAL headr is enabled.
    if ((avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER) &&
        (ctx->cfg.cfg.standard != VK_V_STANDARD_VP9))
        ctx->cfg.cfg.no_repeatheaders = VK_GLOBAL_HEADER;

    ret = ctx->devctx->ilapi->set_parameter(ctx->ilctx, VK_PARAM_VIDEO_ENC_CONFIG,
                                            &ctx->cfg.cfg, VK_CMD_OPT_BLOCKING);
    if (ret)
        goto fail;

    if (ctx->cfg.statsfile || ctx->cfg.statsfile_bin) {
        if (ctx->cfg.statsfile) {
            ctx->cfg.stats = fopen(ctx->cfg.statsfile,"w");
            if (!ctx->cfg.stats) {
                av_log(avctx, AV_LOG_ERROR, "can't open %s\n", ctx->cfg.statsfile);
                ret = AVERROR(ENOENT);
                goto fail;
            }
            fprintf(ctx->cfg.stats, "Encoder statistics\n");
        }
        if (ctx->cfg.statsfile_bin) {
            ctx->cfg.stats_bin = fopen(ctx->cfg.statsfile_bin ,"w");
            if (!ctx->cfg.stats_bin) {
                av_log(avctx, AV_LOG_ERROR, "can't open %s\n", ctx->cfg.statsfile_bin);
                ret = AVERROR(ENOENT);
                goto fail;
            }
        }
        if (ctx->cfg.cfg.me_stats_cfg.flags == VK_CFG_FLAG_ENABLE)
            ctx->stats_size = VK_ALIGN_UP(get_stats_size_small(avctx), VK_DMA_SIZE_ALIGN);
        else
            ctx->stats_size = VK_ALIGN_UP(get_stats_size(avctx), VK_DMA_SIZE_ALIGN);
        ctx->stats_data = av_mallocz(ctx->stats_size);
        if (!ctx->stats_data) {
            av_log(avctx, AV_LOG_ERROR, "Failed to allocate stats buffer\n");
            ret = AVERROR(ENOMEM);
            goto fail;
        }
    }

    if (ctx->cfg.me_statsfile) {
        if (ctx->cfg.me_statsfile) {
            ctx->cfg.me_stats = fopen(ctx->cfg.me_statsfile,"w");
            if (!ctx->cfg.me_stats) {
                av_log(avctx, AV_LOG_ERROR, "can't open %s\n", ctx->cfg.me_statsfile);
                ret = AVERROR(ENOENT);
                goto fail;
            }
        }

        ctx->me_stats_size = VK_ALIGN_UP(get_me_stats_size(avctx), VK_DMA_SIZE_ALIGN);
        ctx->me_stats_data = av_mallocz(ctx->me_stats_size);
        if (!ctx->me_stats_data) {
            av_log(avctx, AV_LOG_ERROR, "Failed to allocate me_stats buffer\n");
            ret = AVERROR(ENOMEM);
            goto fail;
        }
    }

    // Shot change file is written in pass 0, read in passes 1 and 2
    if (ctx->cfg.cfg.lookahead_cfg.flags & VK_MULTIPASS_USE_OFFLINE &&
        (ctx->cfg.cfg.lookahead_cfg.flags & VK_MULTIPASS_PASS_MASK) == 0) {
          if (ctx->cfg.offline_shotchangefile) {
            ctx->cfg.offline_shotchange = fopen(ctx->cfg.offline_shotchangefile, "w");
            if (!ctx->cfg.offline_shotchange) {
                av_log(avctx, AV_LOG_ERROR, "can't open %s for write\n", ctx->cfg.offline_shotchangefile);
                ret = AVERROR(ENOENT);
                goto fail;
            }
            ctx->cfg.shotchange_input_present = 0;
        }
    } else {
        if (ctx->cfg.offline_shotchangefile) {
            ctx->cfg.offline_shotchange = fopen(ctx->cfg.offline_shotchangefile, "r");
            if (!ctx->cfg.offline_shotchange) {
                av_log(avctx, AV_LOG_ERROR, "can't open %s for read\n", ctx->cfg.offline_shotchangefile);
                ret = AVERROR(ENOENT);
                goto fail;
            }
            ctx->cfg.shotchange_input_present = 1;
        }
    }

    // Once done, we can complete the encoder initialization
    ret = ctx->devctx->ilapi->init((void **)(&ctx->ilctx));
    if (ret)
        goto fail;

    // Get the SSIM size after the encoder is initialised
    if (ctx->cfg.ssimfile) {
        int32_t value;
        ctx->cfg.ssim.file = fopen(ctx->cfg.ssimfile,"w");
        if (!ctx->cfg.ssim.file) {
            av_log(avctx, AV_LOG_ERROR, "can't open %s\n", ctx->cfg.ssimfile);
            ret = AVERROR(EINVAL);
            goto fail;
        }
        ret = ctx->devctx->ilapi->get_parameter(ctx->ilctx, VK_PARAM_SSIMMAP_SIZE,
                                           &value, VK_CMD_OPT_BLOCKING);
        if (ret)
            goto fail;

        ctx->cfg.ssim.size = VK_ALIGN_UP(value, VK_DMA_SIZE_ALIGN);
        ctx->cfg.ssim.data = av_mallocz(ctx->cfg.ssim.size);
        ctx->cfg.ssim.width = avctx->width;
        ctx->cfg.ssim.height = avctx->height;
        if (!ctx->cfg.ssim.data) {
            av_log(avctx, AV_LOG_ERROR, "Failed to allocate ssim buffer\n");
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        vkapi_write_ssimheader(&ctx->cfg.ssim,ctx->cfg.cfg.ssim_cfg.log_sb_plus1);
    }

    // The packet pts is provided by the vkil api, however, the packet dts is inferred at ffmpeg
    // level, (monitically increase at same pace as the input frame rate).
    // To infer the packet dts, we use the input frame pts, rather than the frame rate, to take
    // account of possible frame rate variation:
    // frame pts are stored in a FIFO, which i sized as big as the possible number of intransit
    // frame into the vk card.

    // first we get the number of frames possibly stored in the card
    port.port_id.id = 0;
    port.port_id.direction = VK_INPUT_PORT;
    ret = ctx->devctx->ilapi->get_parameter(ctx->ilctx, VK_PARAM_POOL_SIZE,
                                            &port.port_id, VK_CMD_OPT_BLOCKING);
    if (ret)
        goto fail;

    ctx->input_size = port.port_id.map;

    // second we get the number of coded frame possibly stored in the card
    port.port_id.id = 0;
    port.port_id.direction = VK_OUTPUT_PORT;
    ret = ctx->devctx->ilapi->get_parameter(ctx->ilctx, VK_PARAM_POOL_SIZE,
                                            &port.port_id, VK_CMD_OPT_BLOCKING);
    if (ret)
        goto fail;


    // the output ppol may use one buffer for the header information, so we reduce the available
    // output_size by 1 buffer to take account of that.
    output_size = FFMAX(0, (int)port.port_id.map -1);

    ctx->pts_queue = av_fifo_alloc(sizeof(int64_t) * (ctx->input_size + output_size));
    // get the max lag from the card
    ret = ctx->devctx->ilapi->get_parameter(ctx->ilctx, VK_PARAM_MAX_LAG,
                                            &ctx->max_lag, VK_CMD_OPT_BLOCKING);
    if (ret)
        goto fail;

    if (!ctx->max_lag) {
        // allowed lag needs to be at least 1.
        ret = AVERROR(EINVAL);
        goto fail;
    }

    if (ctx->cfg.cfg.standard == VK_V_STANDARD_VP9) {
        ctx->bframe_delay = 0;
    } else {
        // the presentation time can be delayed over the decoding time
        // this delay is function of the stream GOP type (whether it contains B frames, and in the
        // later case, if we have hierarchical coding  of B frames or not)
        ret = ctx->devctx->ilapi->get_parameter(ctx->ilctx, VK_PARAM_VIDEO_ENC_GOP_TYPE,
                                                &gop_type, VK_CMD_OPT_BLOCKING);
        if (ret)
            goto fail;

        if (gop_type == VK_GOP_BIDIRECTIONAL)
            ctx->bframe_delay = 1; // 1 frame delay
        else if (gop_type == VK_GOP_PYRAMID || gop_type == VK_GOP_PYRAMID_EXPLICIT)
            /* 2 or 3 frames delay, depending on pyramid depth */
            ctx->bframe_delay = ctx->cfg.cfg.nbframes_plus1 == 4 ? 2 : 3;
        else
            ctx->bframe_delay = 0;
    }

    if (ctx->cfg.cfg.no_repeatheaders == VK_GLOBAL_HEADER) {
        ret = ctx->devctx->ilapi->get_parameter(ctx->ilctx, VK_PARAM_CODEC_CONFIG,
                                                &vk_packet.prefix.handle, VK_CMD_OPT_BLOCKING);
        if (!ret && !vk_packet.prefix.handle) {
            av_log(avctx, AV_LOG_ERROR, "no handle provided \n");
            ret = AVERROR(ENOMEM);
        }
        if (ret) {
            avctx->extradata_size = 0;
            goto fail;
        }

        // add a reference to the codec cfg buffer
        ret = ctx->devctx->ilapi->xref_buffer(ctx->ilctx, &vk_packet, 1,
                                              VK_CMD_OPT_BLOCKING);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "reference on receive_extradata failed\n");
            goto fail;
        }

        vk_packet.size = 4096; // eyeball configuration packet size estimation
        do {
            vk_packet.size -= used_size;
            vk_packet.size = VK_ALIGN_UP(vk_packet.size, VK_DMA_SIZE_ALIGN);
            // increase configuration packet size as required
            ret = av_reallocp(&avctx->extradata, vk_packet.size);
            if (ret < 0) {
                av_log(avctx, AV_LOG_ERROR, "extradata realloc failure\n");
                ret = AVERROR(ENOMEM);
                goto fail;
            }

            vk_packet.data = avctx->extradata;
            ret = ctx->devctx->ilapi->transfer_buffer2(ctx->ilctx, &vk_packet,
                                                       VK_CMD_DOWNLOAD | VK_CMD_OPT_BLOCKING,
                                                       &used_size);
            if (ret < 0) {
                av_log(avctx, AV_LOG_ERROR, "receive_extradata failed\n");
                goto fail;
            }
         } while (used_size < 0);
         avctx->extradata_size = used_size;
    }

    if (!tunneling) {
        // in non tunneling mode, we asscoiate the ilctx to the input hwframes_ctx
        hwframes_ctx = (AVHWFramesContext *)avctx->hw_frames_ctx->data;
        av_assert0(hwframes_ctx->hwctx);
        vk_framectx = hwframes_ctx->hwctx;
        vk_framectx->ilctx = ctx->ilctx;
    }

    return 0;

fail_init:
    av_log(avctx, AV_LOG_ERROR, "Failed to init encoder ret = %d \n", ret);
    /* fall through */
fail:
    ret = vkapi_error_handling(avctx, ret, __func__);
    if (ctx->ilctx)
        ctx->devctx->ilapi->deinit((void **)(&ctx->ilctx));
    return ret;
}

/**
 * init the hw decoder
 * @param avctx
 * @return error code.
 */
static av_cold int vkapi_encode_init(AVCodecContext *avctx)
{
    int ret;
    VKAPIEncodeContext *ctx;
    AVHWFramesContext *hwframes_ctx;
    AVHWDeviceContext *hwdevice_ctx;
    AVBufferRef *hw_frames_ref = NULL;

    av_assert0(avctx);
    av_assert0(avctx->priv_data);

    ctx = avctx->priv_data;
    // set uninited parameters to zero
    ctx->bframe_delay = 0;
    ctx->flush = 0;
    ctx->ilctx = NULL;
    ctx->hwdevice = NULL;
    memset(&ctx->cfg, 0, sizeof(VKAPIConfig)); // just ensure we don't have undefined value

    // track number of frames sent and received to track lag and counting frames for shot
    // changes
    ctx->send_frames = 0;
    ctx->received_packets = 0;
    ctx->eos_flag = 0;
    if (avctx->hw_frames_ctx) {
        // this path is taken on hw acceleration
        hwframes_ctx = (AVHWFramesContext *)avctx->hw_frames_ctx->data;
        // so the device is provided by the upstream component
        ctx->hwdevice = av_buffer_ref(hwframes_ctx->device_ref);
        if (!(ctx->hwdevice))
            goto fail;
    } else {
        if (avctx->hw_device_ctx) {
            ctx->hwdevice = av_buffer_ref(avctx->hw_device_ctx);
            if (!(ctx->hwdevice))
                goto fail;
        } else {
            ret = av_hwdevice_ctx_create(&ctx->hwdevice, AV_HWDEVICE_TYPE_VKAPI, NULL, NULL, 0);
            if (ret)
                goto fail;
        }
        hw_frames_ref = av_hwframe_ctx_alloc(ctx->hwdevice);
        if (!hw_frames_ref) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        hwframes_ctx = (AVHWFramesContext *)(hw_frames_ref->data);
        hwframes_ctx->format    = AV_PIX_FMT_VKAPI;
        hwframes_ctx->sw_format = AV_PIX_FMT_NV12;
        hwframes_ctx->width     = avctx->width;
        hwframes_ctx->height    = avctx->height;
        hwframes_ctx->initial_pool_size = 0;

        hwdevice_ctx = hwframes_ctx->device_ctx;

        ret = av_hwframe_ctx_init(hw_frames_ref);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "av_hwframe_ctx_init failure %d\n", ret);
            goto fail;
        }
        avctx->hw_frames_ctx = hw_frames_ref;
        hw_frames_ref = NULL; // this prevent spurious dereferencing on subsequent errors
                              // this will now be freed by ffmpeg call to hwframe_ctx_free
    }

    hwdevice_ctx = hwframes_ctx->device_ctx;
    ctx->devctx = hwdevice_ctx->hwctx;

    // then we effectively init the context
    ret = vkapi_internal_init(avctx);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "failure %d\n", ret);
        goto fail;
    }

    return ret;

fail:
    if (hw_frames_ref)
        av_buffer_unref(&hw_frames_ref);
    // vkapi_encode_close will be the called by ffmpeg
    return ret;
}

static int set_metadata(AVCodecContext *avctx, const AVFrame *frame, vkil_aggregated_buffers *input_buffers)
{
    VKAPIEncodeContext *ctx = avctx->priv_data;
    VKAPIDeviceContext *devctx = ctx->devctx;
    int i, ret;
    int shotchange = 0;
    uint8_t frame_qp = 0;
    uint8_t segment_qp = 0;
    int quality = 0;
    int use_offline = ctx->cfg.cfg.lookahead_cfg.flags & VK_MULTIPASS_USE_OFFLINE;
    int pass = ctx->cfg.cfg.lookahead_cfg.flags & VK_MULTIPASS_PASS_MASK;
    vkil_buffer_metadata *metadata;

    for (i = 0; i < VK_NUM_IN_CFG_FILES ; i++) {
        if (!ctx->cfg.inmap[i].file) {
            input_buffers->buffer[i + 1] = NULL;
            continue;
        }
        if (ctx->cfg.inmap[i].size) {
            VKAPIFileHeader *hdr = &ctx->cfg.inmap[i];

            /* retrieve and store data in ext_fields if needed */
            ret = vkapi_fread(hdr, hdr->ext_fields);
            if (ret) {
                if (use_offline)
                    av_log(avctx, AV_LOG_ERROR, "Offline data file not long enough: pass 1 was run on fewer frames or didn't complete.\n");
                goto fail;
            }
            if ((i == VK_CFG_QPMAP) && ctx->cfg.inmap[i].ancillary_data) {
                quality = ((int32_t *)ctx->cfg.inmap[i].ancillary_data)[0];
                av_log(avctx, AV_LOG_DEBUG, "quality = %d \n", quality);
            }

            metadata = &ctx->metadata[i];

            metadata->prefix.type = VKIL_BUF_META_DATA;
            metadata->prefix.handle = 0;
            metadata->data = ctx->cfg.inmap[i].data;
            metadata->size = VK_ALIGN_UP(ctx->cfg.inmap[i].size, VK_DMA_SIZE_ALIGN);
            // actual inmap size populated in used_size here
            metadata->used_size = ctx->cfg.inmap[i].size;

            if (i == VK_CFG_QPMAP) {
                // Last four bytes are frame_qp, segment_qp, unused, unused
                frame_qp = ((char *)metadata->data)[metadata->size - QPMAP_EXTRA_OFFLINE_BYTES + QPMAP_FRAME_QP_OFFSET];
                segment_qp = ((char *)metadata->data)[metadata->size - QPMAP_EXTRA_OFFLINE_BYTES + QPMAP_SEGMENT_QP_OFFSET];
                metadata->size -= QPMAP_EXTRA_OFFLINE_BYTES;
                metadata->used_size -= QPMAP_EXTRA_OFFLINE_BYTES;
            }

            metadata->prefix.port_id = i + 1;

            ret = devctx->ilapi->transfer_buffer(ctx->ilctx,
                                                 metadata, VK_CMD_UPLOAD | VK_CMD_OPT_BLOCKING);
            if (ret < 0) {
                av_log(avctx, AV_LOG_ERROR, "send inmap buffer failed case %d\n",i);
                goto fail;
            }

            input_buffers->buffer[i + 1] = (vkil_buffer *)metadata;
        } else {
            quality = vkapi_fgets(&ctx->cfg.inmap[i]);
            if (quality < 0)
                quality = 0;
        }
    }
    /* Read input shotchange data if it exists */
    if (ctx->cfg.shotchange_input_present) {
         if (ctx->send_frames == 0) {
            // Shot change file starts with a zero which we need to ignore
            if (fscanf(ctx->cfg.offline_shotchange, "%d\n", &(ctx->next_shotchange)) != 1 || ctx->next_shotchange != 0) {
                av_log(avctx, AV_LOG_ERROR, "Failed to read initial 0 in shot change file\n");
                goto fail;
            }
        }
        if (ctx->next_shotchange == ctx->send_frames) {
            shotchange = 1;
            ret = fscanf(ctx->cfg.offline_shotchange, "%d\n", &(ctx->next_shotchange));
            if (ret == EOF) {
                ctx->next_shotchange = (uint32_t)-1;
            } else if (ret != 1) {
                av_log(avctx, AV_LOG_ERROR, "Failed in parsing shot change file\n");
                goto fail;
            }
        }
    }

   if (ctx->cfg.inmap[VK_CFG_QPMAP].file || ctx->cfg.inmap[VK_CFG_VARSMAP].file ||
      ((frame->format == AV_PIX_FMT_VKAPI) && frame->data[VKAPI_METADATA_PLANE])) {
        if ((frame->format == AV_PIX_FMT_VKAPI) && frame->data[VKAPI_METADATA_PLANE]) {
            ctx->metadata[VK_CFG_VARSMAP].prefix.handle = (uint32_t)frame->data[VKAPI_METADATA_PLANE];
            ctx->metadata[VK_CFG_VARSMAP].prefix.type = VKIL_BUF_META_DATA;
            ctx->metadata[VK_CFG_VARSMAP].prefix.ref = 1;
            input_buffers->buffer[VK_CFG_VARSMAP + 1] = (vkil_buffer *)&ctx->metadata[VK_CFG_VARSMAP];
        }
        input_buffers->nbuffers = 1 + VK_NUM_IN_CFG_FILES;
    }

    // Pass extra data in handle. We can't support both simultaneously.
    av_assert0(!(quality && use_offline));

    if (quality) {
        ctx->metadata[VK_CFG_QUALITY].prefix.type = VKIL_BUF_EXTRA_FIELD;
        ctx->metadata[VK_CFG_QUALITY].prefix.handle = quality;
    } else if (use_offline && pass > 0) {
        uint16_t frames_until_next_shotchange;

        if (shotchange) {
            frames_until_next_shotchange = 0;
        } else if ((!ctx->cfg.shotchange_input_present) ||
                   (ctx->next_shotchange == -1) ||
                   (ctx->next_shotchange - ctx->send_frames >
                    VKIL_OFFLINE_NO_FUTURE_SHOTCHANGE)) {
            frames_until_next_shotchange = VKIL_OFFLINE_NO_FUTURE_SHOTCHANGE;
        } else {
            frames_until_next_shotchange = ctx->next_shotchange - ctx->send_frames;
        }
        ctx->metadata[VK_CFG_QUALITY].prefix.type = VKIL_BUF_EXTRA_FIELD;
        ctx->metadata[VK_CFG_QUALITY].prefix.handle = (frames_until_next_shotchange << VKIL_OFFLINE_SHOTCHANGE_POS) +
                                                      (frame_qp << VKIL_OFFLINE_FRAMEQP_POS) +
                                                      (segment_qp << VKIL_OFFLINE_DELTAQP_POS);
    }
    if (quality || (use_offline && pass > 0)) {
        input_buffers->buffer[VK_NUM_IN_CFG_FILES + 1] = (vkil_buffer *)&ctx->metadata[VK_CFG_QUALITY];
        input_buffers->nbuffers = 1 + VK_NUM_IN_CFG_FILES + 1;
    }

    return 0;

fail:
    av_log(avctx, AV_LOG_ERROR, "error %d in %s", ret, __func__);
    return ret;
}

static int vkapi_send_frame(AVCodecContext *avctx, const AVFrame *frame)
{
    VKAPIEncodeContext *ctx = avctx->priv_data;
    int ret = 0;
    AVFrame *tmp_frame =  NULL;
    VKAPIDeviceContext *devctx = ctx->devctx;
    vkil_buffer *vk_buffer;
    vkil_aggregated_buffers vk_input_buffers = {.prefix.type = VKIL_BUF_AG_BUFFERS, .prefix.handle = 0xdeadbeef};
    vkil_buffer_surface *vk_buffer_surface;

    if (!frame) {
        vkil_buffer_surface buffer_eos =   {.prefix.type = VKIL_BUF_SURFACE, .prefix.handle = VK_BUF_EOS};
        // mark this time for timeout handling
        clock_gettime(CLOCK_MONOTONIC, &ctx->flush_time);

        // end of stream, output what is still in the buffers
        ctx->flush = VK_CMD_OPT_BLOCKING; // to force to wait for flushed buffer
        // indicate to the HW it is the last buffer sent
        av_log(avctx, AV_LOG_DEBUG, "send EOS\n");
        ctx->eos_send_idx = ctx->send_frames;
        ret = devctx->ilapi->process_buffer(ctx->ilctx, &buffer_eos, VK_CMD_RUN);
        if (ret < 0)
            goto fail;
        return 0;
    }
    // else we have a valid frame to process

    if (!ctx->send_frames)
        ctx->first_pts = frame->pts;
    else if (frame->pts > ctx->largest_pts)
        av_log(avctx, AV_LOG_WARNING, "non-strictly-montonic PTS\n");

    if (ctx->send_frames == ctx->bframe_delay)
        ctx->bframe_delay_time = frame->pts - ctx->first_pts;

    ctx->largest_pts = frame->pts + 1;

    tmp_frame = av_frame_alloc();
    if (!tmp_frame) {
        av_log(avctx, AV_LOG_ERROR, "av_frame_alloc failed\n");
        ret = ENOMEM;
        goto fail;
    }

    if (avctx->pix_fmt == AV_PIX_FMT_NV12 || avctx->pix_fmt == AV_PIX_FMT_P010) {
        // frame uploading is required
        // the upload could be removed of this function, since the operation is expected to
        // be performed as well by vf_hwupload before calling the encoder

        ret = av_hwframe_get_buffer(avctx->hw_frames_ctx, tmp_frame, 0);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "av_hwframe_get_buffer failed\n");
            goto fail;
        }

        // What happens if we can't transfer the buffer to the HW (e.g. because this one has no memory left)?
        //
        // At this time, if it is due to a memory issue (lack of input buffer), the HW returns a -EAGAIN
        // This situation can be due to 2 facts
        // 1 - the host try to feed the input buffer as a rate greater than the HW can process it
        // 2 - the HW can't process the input buffer because its output buffer is full and the host is not flushing it
        //
        // However, as long as we are in this function (send_frame), ffmpeg will not call receive_packet
        // (to flush the output), and ffmpeg doesn't recognize a EAGAIN request from send_frame, and process it as
        // an error. so the later is not an option.
        //
        // from there, the HW has little choice,
        // 1- wait for and alloc. In case of the lack of buffer is due to a feeding rate greather than the
        // processing rate: it will force ffmpeg send_frame call rate to not be greater than the HW capabilities
        // 2 - However, if ffmpeg is not picking up data on the output, we will have no other choice than to drop the
        // frame (probably that will translate to a "repeat" frame at hw level), since a failure to transfer a frame
        // here result in the conversion abortion.
        //
        // To minimize this later occurrence, the HW need to be configured with an input buffer larger
        // (in term of frame) than the output buffer and we monitor the input occunacy buffer to ensure we force
        // the card to return packet if the buffer is full
        ret = av_hwframe_transfer_data(tmp_frame, frame, 0);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "av_hwframe_transfer_data failed ret=%d\n", ret);
            goto fail;
        }

        devctx->frame_ref_hwprops(tmp_frame, &vk_buffer_surface);
    } else if (frame->hw_frames_ctx && frame->format == AV_PIX_FMT_VKAPI) {
        devctx->frame_ref_hwprops(frame, &vk_buffer_surface);
        // we transfer relevant frame data to the enqueued tmp_frame
        tmp_frame->hw_frames_ctx = av_buffer_ref(frame->hw_frames_ctx);
        if (!vk_buffer_surface->prefix.ref) {
            // when there is no reference on a frame, we repeat the last frame
            av_log(avctx, AV_LOG_DEBUG, "repeat frame\n");
            vk_buffer_surface->prefix.type = VKIL_BUF_EXTRA_FIELD;
            vk_buffer_surface->prefix.handle = VK_BUF_RPT;
        }
    } else {
        av_log(avctx, AV_LOG_ERROR, "unsupported encoding format %d\n", frame->format);
        ret = AVERROR(ENODEV);  // we can't encode this
        goto fail;
    }

    av_assert0((AVHWFramesContext *)tmp_frame->hw_frames_ctx);
    av_assert0((AVHWFramesContext *)tmp_frame->hw_frames_ctx->data);

    vk_buffer_surface->prefix.user_data = frame->pts;
    if (vk_buffer_surface->prefix.handle != VK_BUF_RPT) {
        // because the repeat frame is somewhat due to ffmpeg behavior
        // 1/ the metata file could not match anymore
        // 2/ typically a repeat frame can be skipped (all Mb encoded as skipped)
        //    by encoder, so that metadata is really superfluous here
        // 3/ needless to say  it makes little  sense to encode a repeat frame as Intra
        ret = set_metadata(avctx, frame, &vk_input_buffers);
        if (ret < 0)
            goto fail;
    }

    if (vk_input_buffers.nbuffers > 1) {
        // we have some metadata, so use the aggregated buffer
        vk_input_buffers.buffer[0] = (vkil_buffer *)vk_buffer_surface;
        vk_buffer = (vkil_buffer *)&vk_input_buffers;
        vk_buffer->user_data = vk_buffer_surface->prefix.user_data;
    } else {
       // otherwise use directly the surface buffer
        vk_buffer = (vkil_buffer *)vk_buffer_surface;
    }

    // send the frame to the hw encoder
    ret = devctx->ilapi->process_buffer(ctx->ilctx, vk_buffer, VK_CMD_RUN);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "process buffer result = %d\n", ret);
        goto fail;
    }

    ctx->input_occ = devctx->get_pool_occupancy((AVHWFramesContext *)tmp_frame->hw_frames_ctx->data);
    ff_bufqueue_add(avctx, &ctx->bufqueue, tmp_frame);
    av_fifo_generic_write(ctx->pts_queue, (void *)&frame->pts, sizeof(int64_t), NULL);
    ctx->send_frames++;

    av_log(avctx, AV_LOG_DEBUG, "sent frames=%ld, frame %p, pkt_dts=%s, pts=%s, frame->flags=0x%x, hwbuffer=0x%x\n",
           ctx->send_frames, frame, av_ts2str(frame->pkt_dts), av_ts2str(frame->pts), frame->flags, vk_buffer_surface->prefix.handle);

    return 0;

fail:
    if (tmp_frame)
        av_frame_free(&tmp_frame);

    return vkapi_error_handling(avctx, ret, __func__);
}

static int vkapi_receive_packet(AVCodecContext *avctx, AVPacket *avpkt)
{
    VKAPIEncodeContext *ctx = avctx->priv_data;
    VKAPIDeviceContext *devctx = ctx->devctx;
    int ssim_id, lag, is_blocking, ret = 0, i, j = 0;
    vkil_buffer *vk_buffer;
    vkil_aggregated_buffers vk_output_buffers = {.prefix.type = VKIL_BUF_AG_BUFFERS, .prefix.handle = 0xdeadbeef};
    vkil_buffer_metadata vk_stats =  {.prefix.type = VKIL_BUF_META_DATA};
    vkil_buffer_metadata vk_me_stats = {.prefix.type = VKIL_BUF_META_DATA};
    vkil_buffer_metadata vk_ssim =   {.prefix.type = VKIL_BUF_META_DATA};
    vkil_buffer_packet   vk_packet = {.prefix.type = VKIL_BUF_PACKET, .data = NULL};
    vkil_buffer_metadata vk_offline_data = {.prefix.type = VKIL_BUF_META_DATA};
    vkil_buffer_metadata vk_qpmap = {.prefix.type = VKIL_BUF_META_DATA};
    // data is copied to handle of each buffer descriptor.
    // hence we send 6 descriptors to get 24 bytes of ssim data when size is full.
    vkil_buffer_metadata vk_ssim_data[SSIM_DATA_IN_HANDLE] = {0};
    int receive_ssim_data = 0;
    uint32_t ssim_value[SSIM_DATA_IN_HANDLE];
    AVFrame *tmp_frame;
    int use_offline, use_lookahead, pass;
    int32_t used_size = 0;

    // if the received packets match the # of frames that have been sent
    // before shipping the EOS.  This means that enough packets have
    // been collected.
    if ((ctx->eos_flag) ||
        (ctx->eos_send_idx && (ctx->received_packets > ctx->eos_send_idx))){
        av_log(avctx, AV_LOG_DEBUG, "end of stream\n");
        return AVERROR_EOF;
    }

    use_offline = ctx->cfg.cfg.lookahead_cfg.flags & VK_MULTIPASS_USE_OFFLINE;
    use_lookahead = ctx->cfg.cfg.lookahead_cfg.flags & VK_MULTIPASS_USE_LOOKAHEAD;
    pass = ctx->cfg.cfg.lookahead_cfg.flags & VK_MULTIPASS_PASS_MASK;

    if (ctx->flush) {
        int delta_us;
        int limit_us;
        struct timespec end_time;

        limit_us = (use_offline || use_lookahead) ?
                    VKENC_FLUSH_EXTENDED_TIMEOUT_US : VKENC_FLUSH_TIMEOUT_US;
        clock_gettime(CLOCK_MONOTONIC, &end_time);
        delta_us = _ELAPSED_US(end_time, ctx->flush_time);
        if (delta_us > limit_us) {
            av_log(avctx, AV_LOG_ERROR, "Flush: timeout %d us reached, no EOS, exiting...\n",
                   delta_us);
            vkapi_error_handling(avctx, -EADV, __func__);
            return AVERROR_EOF;
        }
    } else if (ctx->send_frames <= ctx->bframe_delay) {
        // in order to properly compute the dts, ctx->bframe_delay_time
        // need to be set if a bframe_delay is required
        // that requires to have received ctx->bframe_delay frames
        return AVERROR(EAGAIN);
    }

    if (!(use_offline && pass == 2) &&
         (ctx->cfg.stats || ctx->cfg.stats_bin || ctx->cfg.ssim.file ||
         (use_offline && pass < 2) || ctx->cfg.me_stats)) {
        vk_output_buffers.buffer[0] = (vkil_buffer *)&vk_packet;
        // buffers are organized in this way
        // slot 0: bitsream
        // slot 1: medata data stats, or ssim (when no stats buffer exists) or qpmap data (offline pass 1)
        // slot 2: ssim (used only when stats buffer is prsent in slot 1)
        // slot 3: (offline pass 0 or 1) offline_data
        // slot 4: me_stats if B0 Mali
        ssim_id = 1; // index for the ssim buffer when no stats buffer is there
        if (ctx->cfg.stats || ctx->cfg.stats_bin) {
            vk_output_buffers.buffer[1] = (vkil_buffer *)&vk_stats;
            vk_stats.size = ctx->stats_size;
            vk_stats.data = ctx->stats_data; //memory to download stats
            ssim_id = 2; //we bump-out the ssim index to next slot
        }
        if (ctx->cfg.ssim.file) {
            if (!ctx->cfg.cfg.ssim_cfg.log_sb_plus1) {
                for (i = ssim_id; i < (ssim_id + SSIM_DATA_IN_HANDLE); i++)
                    vk_output_buffers.buffer[i] = (vkil_buffer *)&vk_ssim_data[j++];
                receive_ssim_data = 1;
            } else {
                vk_output_buffers.buffer[ssim_id] = (vkil_buffer *)&vk_ssim;
                vk_ssim.size = ctx->cfg.ssim.size;
                vk_ssim.data = ctx->cfg.ssim.data; //memory to download ssim data
            }
        }
        if (use_offline && pass == 1 && ctx->cfg.qpmap_output.size) {
            vk_output_buffers.buffer[1] = (vkil_buffer *)&vk_qpmap;
            // qpmap size is the actual qpmap size, output_size has 4 more bytes
            // for the QPs
            vk_qpmap.size = ctx->cfg.qpmap_output.size - 4;
            vk_qpmap.data = ctx->cfg.qpmap_output.data;
        }

        if (use_offline && pass < 2) {
            vk_output_buffers.buffer[3] = (vkil_buffer *)&vk_offline_data;
        }
        // me_stats is always in slot 4
        if (ctx->cfg.me_stats) {
            vk_output_buffers.buffer[4] = (vkil_buffer *)&vk_me_stats;
            vk_me_stats.size = ctx->me_stats_size;
            vk_me_stats.data = ctx->me_stats_data; //memory to download me stats
        }
        vk_buffer = (vkil_buffer *)&vk_output_buffers;
    } else {
        vk_buffer = (vkil_buffer *)&vk_packet;
    }

    is_blocking = 0;

    // notice this care about counter wrapping
    lag = ctx->send_frames - ctx->received_packets;

    // if the lag between the # of sent frames and # of received packets is equal or greater than
    // the input or output buffer size
    // it can means this buffer can be full (so the HW will not be able to accept the next frame
    //
    // This situation can be due to 2 facts
    // 1 - the host tries to feed input buffer at a rate greater than the HW can process it
    // 2 - the HW can't process the input buffer because its output buffer pool is full and the host
    // is not flushing it
    //
    // To prevent that we force this app to wait for a received packet once there is a a chance of
    // pool overflow
    if (lag >= ctx->max_lag)
        is_blocking |= VK_CMD_OPT_BLOCKING;

    if (ctx->input_occ >= (ctx->input_size - 1))
        is_blocking |= VK_CMD_OPT_BLOCKING;

    ret = devctx->ilapi->process_buffer(ctx->ilctx, vk_buffer,
                                        is_blocking | VK_CMD_OPT_CB);
    if (ret == -EAGAIN) {
          av_log(avctx, AV_LOG_VERBOSE, "no buffer received\n");
          // Sleep for some time if it is in flush mode.  When in flush, FFMPEG will keep calling this
          // function to drain until the EOS is detected (in a while loop).  Just in case FW does not return
          // EOS (and it does happen!), we want to exit based on the check at beginning of this func.
          if (ctx->flush)
              av_usleep(VKENC_FLUSH_YIELD_US);
          return AVERROR(EAGAIN);
    } else if (ret < 0) {
         av_log(avctx, AV_LOG_ERROR, "error %d on buffer reception\n", ret);
         goto fail;
    }

    if (vk_packet.prefix.handle == VK_BUF_EOS) {
        av_log(avctx, AV_LOG_DEBUG, "end of stream\n");
        ctx->flush = 0;
        ctx->eos_flag = 1;
        if (ctx->cfg.ssim.file && (vk_ssim.prefix.handle || (vk_ssim_data[0].prefix.handle && receive_ssim_data))) {
            if (receive_ssim_data) {
                for (i = 0; i < (SSIM_DATA_IN_HANDLE ) ; i++) {
                    ssim_value[i] = vk_ssim_data[i].prefix.handle;
                }
                ret = vkapi_write_ssimdata( ssim_value,
                                            sizeof(ssim_value),
                                            &ctx->cfg.ssim,
                                            ctx->cfg.cfg.ssim_cfg.log_sb_plus1,
                                            ctx->received_packets );
                if (ret)
                    goto fail;

            } else {
                av_log(avctx, AV_LOG_DEBUG, "vk_ssim.prefix.handle = %x\n", vk_ssim.prefix.handle);
                ret = devctx->ilapi->transfer_buffer2(ctx->ilctx,
                                                      &vk_ssim, VK_CMD_DOWNLOAD | VK_CMD_OPT_BLOCKING,
                                                      &used_size);
                if (ret < 0) {
                    av_log(avctx, AV_LOG_ERROR, "receive ssim buffer failed\n");
                    goto fail;
                }
                if (vk_ssim.size && vk_ssim.data) {
                    ret = vkapi_write_ssimdata( vk_ssim.data,
                                                vk_ssim.size,
                                                &ctx->cfg.ssim,
                                                ctx->cfg.cfg.ssim_cfg.log_sb_plus1,
                                                ctx->received_packets );
                    if (ret)
                        goto fail;
                }
            }
        }
        // Get the delta QP for the last segment
        if (use_offline && pass == 1) {
            uint8_t delta_qp = (vk_offline_data.prefix.handle >>
                                VKIL_OFFLINE_DELTAQP_POS) & VKIL_OFFLINE_DELTAQP_MASK;
            // Output qp file. This duplicates the end-of-segment code, so we ought to refactor
            for (i = 0; i < ctx->qp_buffer_frames; i++)
                ((char *)ctx->qp_buffer)[(i + 1) * (vk_qpmap.size + 4) - 3] = delta_qp;
            ret = fwrite(ctx->qp_buffer, 1, ctx->qp_buffer_offset, ctx->cfg.qpmap_output.file);
            if (ret < vk_qpmap.size) {
                ret = AVERROR(EINVAL);
                goto fail;
            }
        }
        return AVERROR_EOF;
    }

    tmp_frame = ff_bufqueue_get(&ctx->bufqueue);
    av_frame_free(&tmp_frame);
    ctx->input_occ--;

    if (ctx->cfg.ssim.file)
        av_log(avctx, AV_LOG_DEBUG, "vk_ssim.prefix.handle = %x\n", vk_ssim.prefix.handle);
    // download the NAL from the card. DMA is assumed to be faster than a memory mapping here
    // (thought the later is not ruled out yet).
    // so we need to instantiate a packet, we could poll the card for the packet size, but we
    // prefer to allocate a large enough memory allowing successful transfer in most case to reduce
    // data traffic thru PCIE bridge
    vk_packet.size = (avctx->width * avctx->height) >> 2;

    // In the odd case there is not enough memory, the download will fails, and will tell us how much
    // is required: the buffer will be then reallocated (we don't use the avpacket structure directly;
    // e.g. av_grow_packet; since such reallocation infer a memcpy of the orginal packet data, we don't
    // need.
    av_packet_unref(avpkt);


#define BUG_SOC_12095 1

#ifdef BUG_SOC_12095
    if ((!ctx->cfg.cfg.lookahead_cfg.flags) || (vk_packet.prefix.handle & VK_SIZE_MASK)) {
        // TODO due to a bug in multipass codec, we have to skip the loop below
        // in case of the handle is 24 bits align
#endif
    do {
        vk_packet.size -= used_size;
        vk_packet.size = VK_ALIGN_UP(vk_packet.size, VK_DMA_SIZE_ALIGN);
        ret = av_reallocp(&vk_packet.data, vk_packet.size);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "realloc failure\n");
            goto fail;
        }
        ret = devctx->ilapi->transfer_buffer2(ctx->ilctx, &vk_packet,
                                              VK_CMD_DOWNLOAD | VK_CMD_OPT_BLOCKING,
                                              &used_size);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "receive_buffer failed\n");
            av_packet_unref(avpkt);
            goto fail;
        }
    } while (used_size < 0);

#ifdef BUG_SOC_12095
    }
#endif

    // the vk_packet buffer data is directly assigned to av packet
    av_packet_from_data(avpkt, vk_packet.data, used_size);

    // we add to the stream size if it doesn't overflow
    if (used_size > UINT64_MAX - ctx->stream_size)
        ctx->stream_size = UINT64_MAX;
    else
        ctx->stream_size += used_size;

    if (ctx->received_packets == UINT64_MAX)
        ctx->stream_size = UINT64_MAX; // we gonna wrap up, so all bitrate computation will be meaningless


    if ((ctx->cfg.stats || ctx->cfg.stats_bin) && vk_stats.prefix.handle) {
        ret = devctx->ilapi->transfer_buffer2(ctx->ilctx, &vk_stats,
                                              VK_CMD_DOWNLOAD | VK_CMD_OPT_BLOCKING,
                                              &used_size);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "receive stats buffer failed\n");
            goto fail;
        }
        if (vk_stats.size && vk_stats.data) {
             av_assert0(vk_stats.size == used_size);
             write_stats_buf(avctx, vk_stats.data, vk_stats.size,
                             ctx->cfg.stats, ctx->cfg.stats_bin);
        }
    }
    if (ctx->cfg.me_stats && vk_me_stats.prefix.handle) {
        ret = devctx->ilapi->transfer_buffer(ctx->ilctx, &vk_me_stats,
                                             VK_CMD_DOWNLOAD | VK_CMD_OPT_BLOCKING);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "receive me_stats buffer failed\n");
            ret = AVERROR(EINVAL);
            goto fail;
        }
        if (vk_me_stats.size && vk_me_stats.data) {
             av_assert0(vk_me_stats.size == vk_me_stats.prefix.handle & VK_SIZE_MASK);
             write_me_stats_buf(vk_me_stats.data, vk_me_stats.size,
                             ctx->cfg.me_stats);
        }
    }
    if (ctx->cfg.ssim.file && (vk_ssim.prefix.handle || (vk_ssim_data[0].prefix.handle && receive_ssim_data))) {
        if (receive_ssim_data) {
            for (i = 0; i < SSIM_DATA_IN_HANDLE; i++) {
                 ssim_value[i] = vk_ssim_data[i].prefix.handle;
            }
            ret = vkapi_write_ssimdata( ssim_value,
                                        sizeof(ssim_value),
                                        &ctx->cfg.ssim,
                                        ctx->cfg.cfg.ssim_cfg.log_sb_plus1,
                                        ctx->received_packets );
            if (ret)
                goto fail;

        } else {
            ret = devctx->ilapi->transfer_buffer2(ctx->ilctx, &vk_ssim,
                                                  VK_CMD_DOWNLOAD | VK_CMD_OPT_BLOCKING,
                                                  &used_size);
            if (ret < 0) {
                av_log(avctx, AV_LOG_ERROR, "receive ssim buffer failed\n");
                goto fail;
            }
            if (vk_ssim.size && vk_ssim.data) {
                av_assert0(vk_ssim.size == used_size);
                ret = vkapi_write_ssimdata( vk_ssim.data,
                                            vk_ssim.size,
                                            &ctx->cfg.ssim,
                                            ctx->cfg.cfg.ssim_cfg.log_sb_plus1,
                                            ctx->received_packets );
                if (ret)
                    goto fail;
            }
        }
    }

    if (ctx->cfg.qpmap_output.size) {
        ret = devctx->ilapi->transfer_buffer2(ctx->ilctx, &vk_qpmap,
                                              VK_CMD_DOWNLOAD | VK_CMD_OPT_BLOCKING,
                                              &used_size);
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "receive qpmap buffer failed\n");
            goto fail;
        }
        if (vk_qpmap.size && vk_qpmap.data) {
            if (!(use_offline && pass==1)) {
                ret = fwrite(vk_qpmap.data, 1, vk_qpmap.size, ctx->cfg.qpmap_output.file);
                if (ret < vk_qpmap.size) {
                    ret = AVERROR(EINVAL);
                    goto fail;
                }
            } else {
                // We combine it with frame qp and segment delta qp, but the latter
                // isn't available until the end of the segment, so cache the data
                // for now
                if (!ctx->qp_buffer) {
                    // Allocate enough space for the maximum length of a segment we have specified,
                    // plus one frame (we need to read the first frame of the next segment in to
                    // detect the segment change)
                    // TODO: In B0 frames don't always come back to vkapi_receive_packet in order,
                    // so we need to extend qp_buffer further. Do this by nbframes + 1 for now, but
                    // it's not clear why this happens and what extra space is actually needed.
                    ctx->qp_buffer = av_mallocz((vk_qpmap.size + QPMAP_EXTRA_OFFLINE_BYTES) *
                        (ctx->cfg.cfg.lookahead_cfg.shotlength_max + 1 + ctx->cfg.cfg.nbframes_plus1));
                }
                memcpy((char *)ctx->qp_buffer + ctx->qp_buffer_offset, vk_qpmap.data, vk_qpmap.size);
                ctx->qp_buffer_offset += vk_qpmap.size;
            }

        }
    }

    if (use_offline && pass < 2)  {
        uint32_t offline_data = vk_offline_data.prefix.handle;
        uint8_t shotchange = (offline_data >> VKIL_OFFLINE_SHOTCHANGE_POS) & VKIL_OFFLINE_SHOTCHANGE_MASK;
        uint8_t frame_qp = (offline_data >> VKIL_OFFLINE_FRAMEQP_POS) & VKIL_OFFLINE_FRAMEQP_MASK;
        uint8_t delta_qp = (offline_data >> VKIL_OFFLINE_DELTAQP_POS) & VKIL_OFFLINE_DELTAQP_MASK;

        if (pass == 0 && shotchange) {
            av_assert0(ctx->cfg.offline_shotchange);
            fprintf(ctx->cfg.offline_shotchange,"%ld\n",ctx->received_packets);
        }
        if (pass == 1) {
            // Write into qp buffer
            ((char *)ctx->qp_buffer)[ctx->qp_buffer_offset + QPMAP_FRAME_QP_OFFSET] = frame_qp;
            ctx->qp_buffer_offset += QPMAP_EXTRA_OFFLINE_BYTES;
            ctx->qp_buffer_frames += 1;
            av_assert0(ctx->qp_buffer_offset == ctx->qp_buffer_frames * (vk_qpmap.size + 4));
            av_assert0(ctx->qp_buffer_frames <= (ctx->cfg.cfg.lookahead_cfg.shotlength_max + 1 + ctx->cfg.cfg.nbframes_plus1));

            if (shotchange && ctx->received_packets > 0) {
                // We have reached end of segment, so can now output qp file for everything preceding
                // this frame
                uint32_t write_size;

                for (int i = 0; i < ctx->qp_buffer_frames - 1; i++) {
                    ((char *)ctx->qp_buffer)[(i + 1) * (vk_qpmap.size + QPMAP_EXTRA_OFFLINE_BYTES) - QPMAP_EXTRA_OFFLINE_BYTES + QPMAP_SEGMENT_QP_OFFSET] = delta_qp;
                }
                write_size = (vk_qpmap.size + QPMAP_EXTRA_OFFLINE_BYTES) * (ctx->qp_buffer_frames - 1);
                ret = fwrite(ctx->qp_buffer, 1, write_size, ctx->cfg.qpmap_output.file);
                memcpy(ctx->qp_buffer,(char *)ctx->qp_buffer + write_size, vk_qpmap.size + QPMAP_EXTRA_OFFLINE_BYTES);
                if (ret < vk_qpmap.size) {
                    ret = AVERROR(EINVAL);
                    goto fail;
                }
                ctx->qp_buffer_offset = vk_qpmap.size + QPMAP_EXTRA_OFFLINE_BYTES;
                ctx->qp_buffer_frames = 1;
            }
        }
    }

    avpkt->pts =  vk_packet.prefix.user_data;
    av_fifo_generic_read(ctx->pts_queue, &avpkt->dts, sizeof(int64_t), NULL);
    avpkt->dts -= ctx->bframe_delay_time;
    ctx->received_packets++;

    /* eos flag set to 1. next iteration we return EOF */
    if (vk_packet.prefix.flags & VKIL_BUFFER_PACKET_FLAG_EOS) {
        av_log(avctx, AV_LOG_DEBUG, "Flag set to EOS\n");
        ctx->eos_flag = 1;
    }

    av_log(avctx, AV_LOG_DEBUG, "received packet %ld, packet %p, size=%d, dts=%s, pts=%s, flags=0x%x, hwbuffer=0x%x\n",
           ctx->received_packets, avpkt, avpkt->size, av_ts2str(avpkt->dts), av_ts2str(avpkt->pts), avpkt->flags, vk_packet.prefix.handle);

    return ret;

fail:
    return vkapi_error_handling(avctx, ret, __func__);
}

#define OFFSET(x) offsetof(VKAPIEncodeContext, x)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    {"vk-params", "set the vk configuration using a :-separated list of key=value parameters", OFFSET(vk_opts), AV_OPT_TYPE_STRING, { 0 }, 0, 0, VE },
    { NULL }
};

static const AVCodecHWConfigInternal *vkapi_hw_configs[] = {
    &(const AVCodecHWConfigInternal) {
        .public = {
            .pix_fmt     = AV_PIX_FMT_VKAPI,
            .methods     = AV_CODEC_HW_CONFIG_METHOD_HW_FRAMES_CTX |
                           AV_CODEC_HW_CONFIG_METHOD_INTERNAL,
            .device_type = AV_HWDEVICE_TYPE_VKAPI
        },
        .hwaccel = NULL,
    },
    NULL
};

static const AVCodecDefault defaults[] = {
    { NULL },
};

#define FF_VKAPI_ENC_CLASS(NAME) \
    static const AVClass ff_vkapi_##NAME##_enc_class = { \
        .class_name = "vkapi_" #NAME "_enc", \
        .item_name  = av_default_item_name, \
        .option     = options, \
        .version    = LIBAVUTIL_VERSION_INT, \
    };

#define FF_VKAPI_ENC(NAME, ID) \
    FF_VKAPI_ENC_CLASS(NAME) \
    AVCodec ff_##NAME##_vkapi_encoder = { \
        .name           = #NAME "_vkapi", \
        .long_name      = NULL_IF_CONFIG_SMALL(#NAME " (vkapi)"), \
        .type           = AVMEDIA_TYPE_VIDEO, \
        .id             = ID, \
        .init           = vkapi_encode_init, \
        .send_frame     = vkapi_send_frame, \
        .receive_packet = vkapi_receive_packet, \
        .close          = vkapi_encode_close, \
        .priv_data_size = sizeof(VKAPIEncodeContext), \
        .priv_class     = &ff_vkapi_##NAME##_enc_class, \
        .defaults       = defaults, \
        .capabilities   = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_HARDWARE, \
        .caps_internal  = FF_CODEC_CAP_INIT_THREADSAFE | FF_CODEC_CAP_INIT_CLEANUP, \
        .pix_fmts       = vkapi_pix_fmts, \
        .hw_configs     = vkapi_hw_configs, \
        .wrapper_name   = "vkapi", \
    };

#if CONFIG_H264_VKAPI_ENCODER
FF_VKAPI_ENC(h264, AV_CODEC_ID_H264)
#endif

#if CONFIG_HEVC_VKAPI_ENCODER
FF_VKAPI_ENC(hevc, AV_CODEC_ID_HEVC)
#endif

#if CONFIG_VP9_VKAPI_ENCODER
FF_VKAPI_ENC(vp9, AV_CODEC_ID_VP9)
#endif
