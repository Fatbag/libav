/*
 * Opus decoder
 * Copyright (c) 2012 Andrew D'Addesio
 *
 * This file is part of Libav.
 *
 * Libav is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * Libav is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with Libav; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * Opus decoder by Fatbag; this is a WIP
 *
 * Codec homepage: http://opus-codec.org/
 * Specification: http://tools.ietf.org/html/draft-ietf-codec-opus
 * OggOpus specification: https://wiki.xiph.org/OggOpus
 *
 * This decoder will only work with Ogg-contained .opus files;
 * these can be produced with opus-tools:
 * http://git.xiph.org/?p=opus-tools.git
 */

#include "avcodec.h"
#include "get_bits.h"
#include "bytestream.h"
#include "unary.h"
#include "opus.h"

#define MAX_FRAME_SIZE 1275
#define MAX_FRAMES     48
#define MAX_FRAME_DUR  5760 /* in samples @ 48kHz */

enum OpusMode {
    OPUS_MODE_SILK,
    OPUS_MODE_HYBRID,
    OPUS_MODE_CELT
};

#ifdef DEBUG
static const char *const opus_mode_str[3] = {
    "silk", "hybrid", "celt"
};
#endif

enum OpusBandwidth {
    OPUS_BANDWIDTH_NARROWBAND,
    OPUS_BANDWIDTH_MEDIUMBAND,
    OPUS_BANDWIDTH_WIDEBAND,
    OPUS_BANDWIDTH_SUPERWIDEBAND,
    OPUS_BANDWIDTH_FULLBAND
};

#ifdef DEBUG
static const char *const opus_bandwidth_str[5] = {
    "narrowband", "medium-band", "wideband", "super-wideband", "fullband"
};
#endif

typedef struct {
    int size;                       /** packet size */
    int code;                       /** packet code: specifies the frame layout */
    int stereo;                     /** whether this packet is mono or stereo */
    int vbr;                        /** vbr flag */
    int config;                     /** configuration: tells the audio mode,
                                     **                bandwidth, and frame duration */
    int frame_count;                /** frame count */
    int frame_offset[MAX_FRAMES];   /** frame offsets */
    int frame_size[MAX_FRAMES];     /** frame sizes */
    int padding;                    /** padding size */
    int frame_duration;             /** frame duration, in samples @ 48kHz */
    enum OpusMode mode;             /** mode */
    enum OpusBandwidth bandwidth;   /** bandwidth */
} OpusPacket;

typedef struct {
    GetBitContext *gb;
    unsigned int range;
    unsigned int value;
} OpusRangeCoder;

typedef struct {
    int stereo_weights[2];
    int previous_gain[2];
    int midonly;
    int subframes;
} SilkContext;

typedef struct {
    AVCodecContext *avctx;
    AVFrame frame;
    GetBitContext gb;
    OpusRangeCoder rc;
    SilkContext silk;
    // CeltContext celt;

    OpusPacket packet;
    int currentframe;
    uint8_t *buf;
    int buf_size;

    float output[2*MAX_FRAME_DUR]; /* stereo samples before resampling */
} OpusContext;

static void opus_dprint_packet(AVCodecContext *avctx, OpusPacket *pkt)
{
#ifdef DEBUG
    int i;
    av_dlog(avctx, "[OPUS_PACKET]\n");
    av_dlog(avctx, "size=%d\n",            pkt->size);
    av_dlog(avctx, "code=%d\n",            pkt->code);
    av_dlog(avctx, "stereo=%d\n",          pkt->stereo);
    av_dlog(avctx, "vbr=%d\n",             pkt->vbr);
    av_dlog(avctx, "config=%d\n",          pkt->config);
    av_dlog(avctx, "mode=%s\n",            opus_mode_str[pkt->mode]);
    av_dlog(avctx, "bandwidth=%s\n",       opus_bandwidth_str[pkt->bandwidth]);
    av_dlog(avctx, "frame duration=%d\n",  pkt->frame_duration);
    av_dlog(avctx, "frame count=%d\n",     pkt->frame_count);
    av_dlog(avctx, "packet duration=%d\n", pkt->frame_duration * pkt->frame_count);
    for (i = 0; i < pkt->frame_count; i++)
        av_dlog(avctx, "frame %d : size=%d offset=%d\n", i, pkt->frame_size[i],
                pkt->frame_offset[i]);
    av_dlog(avctx, "[/OPUS_PACKET]\n");
#endif
}

/**
 * Read a 1- or 2-byte frame length
 */
static inline int read_2byte_value(const uint8_t **ptr, const uint8_t *end)
{
    int val;

    if (*ptr >= end)
        return AVERROR_INVALIDDATA;
    val = *(*ptr)++;
    if (val >= 252) {
        if (*ptr >= end)
            return AVERROR_INVALIDDATA;
        val += 4 * *(*ptr)++;
    }
    return val;
}

/**
 * Read a multi-byte length (used for code 3 packet padding size)
 */
static inline int read_multibyte_value(const uint8_t **ptr, const uint8_t *end)
{
    int val = 0;
    int next;

    while (1) {
        if (*ptr >= end || val > INT_MAX - 254)
            return AVERROR_INVALIDDATA;
        next = *(*ptr)++;
        val += next;
        if (next < 255)
            break;
        else
            val--;
    }
    return val;
}

/**
 * Parse Opus packet info from raw packet data
 *
 * TODO: add option for self-delimited packets
 */
static int opus_parse_packet(OpusContext *s)
{
    int frame_bytes, i;
    OpusPacket *pkt = &s->packet;
    int len = s->buf_size;
    const uint8_t *ptr = s->buf;
    const uint8_t *end = s->buf + len;

    if (len < 1)
        return AVERROR_INVALIDDATA;

    pkt->size    = len;
    pkt->padding = 0;

    /* TOC byte */
    i = *ptr++;
    pkt->code   = (i     ) & 0x3;
    pkt->stereo = (i >> 2) & 0x1;
    pkt->config = (i >> 3) & 0x1F;

    /* code 2 and code 3 packets have at least 1 byte after the TOC */
    if (pkt->code >= 2 && len < 1)
        return AVERROR_INVALIDDATA;

    switch (pkt->code) {
    case 0:
        /* 1 frame */
        pkt->frame_count = 1;
        pkt->vbr   = 0;
        frame_bytes = end - ptr;
        if (frame_bytes > MAX_FRAME_SIZE)
            return AVERROR_INVALIDDATA;
        pkt->frame_offset[0] = ptr - s->buf;
        pkt->frame_size[0]   = frame_bytes;
        break;
    case 1:
        /* 2 frames, equal size */
        pkt->frame_count = 2;
        pkt->vbr   = 0;
        frame_bytes = end - ptr;
        if (frame_bytes & 1 || frame_bytes / 2 > MAX_FRAME_SIZE)
            return AVERROR_INVALIDDATA;
        pkt->frame_offset[0] = ptr - s->buf;
        pkt->frame_size[0]   = frame_bytes / 2;
        pkt->frame_offset[1] = pkt->frame_offset[0] + pkt->frame_size[0];
        pkt->frame_size[1]   = frame_bytes / 2;
        break;
    case 2:
        /* 2 frames, different sizes */
        pkt->frame_count = 2;
        pkt->vbr   = 1;

        /* read 1st frame size */
        frame_bytes = read_2byte_value(&ptr, end);
        if (frame_bytes < 0)
            return AVERROR_INVALIDDATA;
        pkt->frame_offset[0] = ptr - s->buf;
        pkt->frame_size[0]   = frame_bytes;

        /* calculate 2nd frame size */
        frame_bytes = len - pkt->frame_size[0] - pkt->frame_offset[0];
        if (frame_bytes < 0 || frame_bytes > MAX_FRAME_SIZE)
            return AVERROR_INVALIDDATA;
        pkt->frame_offset[1] = pkt->frame_offset[0] + pkt->frame_size[0];
        pkt->frame_size[1]   = frame_bytes;
        break;
    case 3:
        /* 1 to 48 frames, can be different sizes */
        i = *ptr++;
        pkt->frame_count   = (i     ) & 0x3F;
        pkt->padding = (i >> 6) & 0x01;
        pkt->vbr     = (i >> 7) & 0x01;

        if (pkt->frame_count == 0)
            return AVERROR_INVALIDDATA;

        /* read padding size */
        if (pkt->padding) {
            pkt->padding = read_multibyte_value(&ptr, end);
            if (pkt->padding < 0)
                return AVERROR_INVALIDDATA;
        }
        if (end - ptr < pkt->padding)
            return AVERROR_INVALIDDATA;
        end -= pkt->padding;

        /* read frame sizes */
        if (pkt->vbr) {
            /* for VBR, all frames except the final one have their size coded
               in the bitstream. the last frame size is implicit. */
            int total_bytes = 0;
            for (i = 0; i < pkt->frame_count - 1; i++) {
                frame_bytes = read_2byte_value(&ptr, end);
                if (frame_bytes < 0)
                    return AVERROR_INVALIDDATA;
                pkt->frame_size[i] = frame_bytes;
                total_bytes += frame_bytes;
            }
            frame_bytes = end - ptr;
            if (total_bytes > frame_bytes)
                return AVERROR_INVALIDDATA;
            pkt->frame_offset[0] = ptr - s->buf;
            for (i = 1; i < pkt->frame_count; i++)
                pkt->frame_offset[i] = pkt->frame_offset[i-1] + pkt->frame_size[i-1];
            pkt->frame_size[pkt->frame_count-1] = frame_bytes - total_bytes;
        } else {
            /* for CBR, the remaining packet bytes are divided evenly between
               the frames */
            frame_bytes = end - ptr;
            if (frame_bytes % pkt->frame_count ||
                frame_bytes / pkt->frame_count > MAX_FRAME_SIZE)
                return AVERROR_INVALIDDATA;
            frame_bytes /= pkt->frame_count;
            pkt->frame_offset[0] = ptr - s->buf;
            pkt->frame_size[0]   = frame_bytes;
            for (i = 1; i < pkt->frame_count; i++) {
                pkt->frame_offset[i] = pkt->frame_offset[i-1] + pkt->frame_size[i-1];
                pkt->frame_size[i]   = frame_bytes;
            }
        }
    }

    /* total packet duration cannot be larger than 120ms */
    pkt->frame_duration = opus_frame_duration[pkt->config];
    if (pkt->frame_duration * pkt->frame_count > MAX_FRAME_DUR)
        return AVERROR_INVALIDDATA;

    /* set mode and bandwidth */
    if (pkt->config < 12) {
        pkt->mode = OPUS_MODE_SILK;
        pkt->bandwidth = pkt->config / 4;
    } else if (pkt->config < 16) {
        pkt->mode = OPUS_MODE_HYBRID;
        pkt->bandwidth = OPUS_BANDWIDTH_SUPERWIDEBAND + (pkt->config >= 14);
    } else {
        pkt->mode = OPUS_MODE_CELT;
        pkt->bandwidth = (pkt->config - 16) / 4;
        /* skip mediumband */
        if (pkt->bandwidth)
            pkt->bandwidth++;
    }

    opus_dprint_packet(s->avctx, pkt);
    return ptr - s->buf;
}

/**
 * Opus RangeCoder models and functions
 *
 * Models are described by CDFs rather than PDFs.
 */

static const uint16_t rc_model_bit[] = {2, 1, 2};

static const uint16_t silk_model_stereo_s1[] = {
    256,   7,   9,  10,  11,  12,  22,  46,  54,  55,  56,  59,  82, 174, 197, 200,
    201, 202, 210, 234, 244, 245, 246, 247, 249, 256
};
static const uint16_t silk_model_stereo_s2[] = {256, 85, 171, 256};
static const uint16_t silk_model_stereo_s3[] = {256, 51, 102, 154, 205, 256};
static const uint16_t silk_model_mid_only[] = {256, 192, 256};
static const uint16_t silk_model_frame_type_unvoiced[] = {256, 26, 256};
static const uint16_t silk_model_frame_type_voiced[] = {256, 24, 98, 246, 256};
static const uint16_t silk_model_gain[3][9] = {
    {256,  32, 144, 212, 241, 253, 254, 255, 256},
    {256,   2,  19,  64, 124, 186, 233, 252, 256},
    {256,   1,   4,  30, 101, 195, 245, 254, 256}
};
static const uint16_t silk_model_gain_lsbits[] = {256, 32, 64, 96, 128, 160, 192, 224, 256};
static const uint16_t silk_model_gain_delta[] = {
    256,   6,  11,  22,  53, 185, 206, 214, 218, 221, 223, 225, 227, 228, 229, 230,
    231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246,
    247, 248, 249, 250, 251, 252, 253, 254, 255, 256
};
static const uint16_t silk_model_lsf_s1[2][2][33] = {
    {
        {    // NB or MB, unvoiced
            256,  44,  78, 108, 127, 148, 160, 171, 174, 177, 179, 195, 197, 199, 200, 205,
            207, 208, 211, 214, 215, 216, 218, 220, 222, 225, 226, 235, 244, 246, 253, 255, 256
        }, { // NB or MB, voiced
            256,   1,  11,  12,  20,  23,  31,  39,  53,  66,  80,  81,  95, 107, 120, 131,
            142, 154, 165, 175, 185, 196, 204, 213, 221, 228, 236, 237, 238, 244, 245, 251, 256
        }
    }, {
        {    // WB, unvoiced
            256,  31,  52,  55,  72,  73,  81,  98, 102, 103, 121, 137, 141, 143, 146, 147,
            157, 158, 161, 177, 188, 204, 206, 208, 211, 213, 224, 225, 229, 238, 246, 253, 256
        }, { // WB, voiced
            256,   1,   5,  21,  26,  44,  55,  60,  74,  89,  90,  93, 105, 118, 132, 146,
            152, 166, 178, 180, 186, 187, 199, 211, 222, 232, 235, 245, 250, 251, 252, 253, 256
        }
    }
};
static const uint16_t silk_model_lsf_vq2[32][10] = {
    // NB, MB
    {256,   1,   2,   3,  18, 242, 253, 254, 255, 256},
    {256,   1,   2,   4,  38, 221, 253, 254, 255, 256},
    {256,   1,   2,   6,  48, 197, 252, 254, 255, 256},
    {256,   1,   2,  10,  62, 185, 246, 254, 255, 256},
    {256,   1,   4,  20,  73, 174, 248, 254, 255, 256},
    {256,   1,   4,  21,  76, 166, 239, 254, 255, 256},
    {256,   1,   8,  32,  85, 159, 226, 252, 255, 256},
    {256,   1,   2,  20,  83, 161, 219, 249, 255, 256},

    // WB
    {256,   1,   2,   3,  12, 244, 253, 254, 255, 256},
    {256,   1,   2,   4,  32, 218, 253, 254, 255, 256},
    {256,   1,   2,   5,  47, 199, 252, 254, 255, 256},
    {256,   1,   2,  12,  61, 187, 252, 254, 255, 256},
    {256,   1,   5,  24,  72, 172, 249, 254, 255, 256},
    {256,   1,   2,  16,  70, 170, 242, 254, 255, 256},
    {256,   1,   2,  17,  78, 165, 226, 251, 255, 256},
    {256,   1,   8,  29,  79, 156, 237, 254, 255, 256}
};

static inline void opus_rc_normalize(OpusRangeCoder *rc)
{
    while (rc->range <= 1<<23) {
        av_dlog(NULL, "--start-- value: %u\n          range: %u\n", rc->value, rc->range);
        rc->value = ((rc->value << 8) | (255 - get_bits(rc->gb, 8))) & ((1U<<31)-1);
        rc->range <<= 8;
        av_dlog(NULL, "--end--   value: %u\n          range: %u\n", rc->value, rc->range);
    }
}

static inline void opus_rc_init(OpusRangeCoder *rc)
{
    rc->range = 128;
    rc->value = 127 - get_bits(rc->gb, 7);
    av_dlog(NULL, "[rc init] range: %u, value: %u\n", rc->range, rc->value);
    opus_rc_normalize(rc);
}

static unsigned int opus_rc_getsymbol(OpusRangeCoder *rc, const uint16_t *cdf)
{
    unsigned int k, scale, ptotal, psymbol, plow, phigh;

    ptotal = *cdf++;

    scale   = rc->range / ptotal;
    psymbol = rc->value / scale + 1;
    psymbol = ptotal - (psymbol < ptotal ? psymbol : ptotal);

    for (k = 0; (phigh = cdf[k]) <= psymbol; k++);
    plow = k ? cdf[k-1] : 0;

    rc->value -= scale * (ptotal - phigh);
    rc->range  = plow ? scale * (phigh - plow)
                      : rc->range - scale * (ptotal - phigh);

    opus_rc_normalize(rc);

    av_dlog(NULL, "%d <= %d < %d\n", plow, psymbol, phigh);
    return k;
}

/**
 * SILK decoder
 */

static const int16_t silk_stereo_weights[] = {
    -13732, -10050,  -8266,  -7526,  -6500,  -5000,  -2950,   -820,
       820,   2950,   5000,   6500,   7526,   8266,  10050,  13732
};
static const uint8_t silk_coarse_codebook_nbmb[32][10] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {1, 3, 1, 2, 2, 1, 2, 1, 1, 1},
    {2, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 2, 2, 2, 2, 1, 2, 1, 1, 1},
    {2, 3, 3, 3, 3, 2, 2, 2, 2, 2},
    {0, 5, 3, 3, 2, 2, 2, 2, 1, 1},
    {0, 2, 2, 2, 2, 2, 2, 2, 2, 1},
    {2, 3, 6, 4, 4, 4, 5, 4, 5, 5},
    {2, 4, 5, 5, 4, 5, 4, 6, 4, 4},
    {2, 4, 4, 7, 4, 5, 4, 5, 5, 4},
    {4, 3, 3, 3, 2, 3, 2, 2, 2, 2},
    {1, 5, 5, 6, 4, 5, 4, 5, 5, 5},
    {2, 7, 4, 6, 5, 5, 5, 5, 5, 5},
    {2, 7, 5, 5, 5, 5, 5, 6, 5, 4},
    {3, 3, 5, 4, 4, 5, 4, 5, 4, 4},
    {2, 3, 3, 5, 5, 4, 4, 4, 4, 4},
    {2, 4, 4, 6, 4, 5, 4, 5, 5, 5},
    {2, 5, 4, 6, 5, 5, 5, 4, 5, 4},
    {2, 7, 4, 5, 4, 5, 4, 5, 5, 5},
    {2, 5, 4, 6, 7, 6, 5, 6, 5, 4},
    {3, 6, 7, 4, 6, 5, 5, 6, 4, 5},
    {2, 7, 6, 4, 4, 4, 5, 4, 5, 5},
    {4, 5, 5, 4, 6, 6, 5, 6, 5, 4},
    {2, 5, 5, 6, 5, 6, 4, 6, 4, 4},
    {4, 5, 5, 5, 3, 7, 4, 5, 5, 4},
    {2, 3, 4, 5, 5, 6, 4, 5, 5, 4},
    {2, 3, 2, 3, 3, 4, 2, 3, 3, 3},
    {1, 1, 2, 2, 2, 2, 2, 3, 2, 2},
    {4, 5, 5, 6, 6, 6, 5, 6, 4, 5},
    {3, 5, 5, 4, 4, 4, 4, 3, 3, 2},
    {2, 5, 3, 7, 5, 5, 4, 4, 5, 4},
    {4, 4, 5, 4, 5, 6, 5, 6, 5, 4}
};
static const uint8_t silk_coarse_codebook_wb[32][16] = {
    { 8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8},
    {10, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10,  9,  9,  9,  8, 11},
    {10, 13, 13, 11, 15, 12, 12, 13, 10, 13, 12, 13, 13, 12, 11, 11},
    { 8, 10,  9, 10, 10,  9,  9,  9,  9,  9,  8,  8,  8,  8,  8,  9},
    { 8, 14, 13, 12, 14, 12, 15, 13, 12, 12, 12, 13, 13, 12, 12, 11},
    { 8, 11, 13, 13, 12, 11, 11, 13, 11, 11, 11, 11, 11, 11, 10, 12},
    { 8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8},
    { 8, 10, 14, 11, 15, 10, 13, 11, 12, 13, 13, 12, 11, 11, 10, 11},
    { 8, 14, 10, 14, 14, 12, 13, 12, 14, 13, 12, 12, 13, 11, 11, 11},
    {10,  9,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8},
    { 8,  9,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  9},
    {10, 10, 11, 12, 13, 11, 11, 11, 11, 11, 11, 11, 10, 10,  9, 11},
    {10, 10, 11, 11, 12, 11, 11, 11, 11, 11, 11, 11, 11, 10,  9, 11},
    {11, 12, 12, 12, 14, 12, 12, 13, 11, 13, 12, 12, 13, 12, 11, 12},
    { 8, 14, 12, 13, 12, 15, 13, 10, 14, 13, 15, 12, 12, 11, 13, 11},
    { 8,  9,  8,  9,  9,  9,  9,  9,  9,  9,  8,  8,  8,  8,  9,  8},
    { 9, 14, 13, 15, 13, 12, 13, 11, 12, 13, 12, 12, 12, 11, 11, 12},
    { 9, 11, 11, 12, 12, 11, 11, 13, 10, 11, 11, 13, 13, 13, 11, 12},
    {10, 11, 11, 10, 10, 10, 11, 10,  9, 10,  9, 10,  9,  9,  9, 12},
    { 8, 10, 11, 13, 11, 11, 10, 10, 10,  9,  9,  8,  8,  8,  8,  8},
    {11, 12, 11, 13, 11, 11, 10, 10,  9,  9,  9,  9,  9, 10, 10, 12},
    {10, 14, 11, 15, 15, 12, 13, 12, 13, 11, 13, 11, 11, 10, 11, 11},
    {10, 11, 13, 14, 14, 11, 13, 11, 12, 12, 11, 11, 11, 11, 10, 12},
    { 9, 11, 11, 12, 12, 12, 12, 11, 13, 13, 13, 11,  9,  9,  9,  9},
    {10, 13, 11, 14, 14, 12, 15, 12, 12, 13, 11, 12, 12, 11, 11, 11},
    { 8, 14,  9,  9,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8},
    { 8, 14, 14, 11, 13, 10, 13, 13, 11, 12, 12, 15, 15, 12, 12, 12},
    {11, 11, 15, 11, 13, 12, 11, 11, 11, 10, 10, 11, 11, 11, 10, 11},
    { 8,  8,  9,  8,  8,  8, 10,  9, 10,  9,  9, 10, 10, 10,  9,  9},
    { 8, 11, 10, 13, 11, 11, 10, 11, 10,  9,  8,  8,  9,  8,  8,  9},
    {11, 13, 13, 12, 15, 13, 11, 11, 10, 11, 10, 10,  9,  8,  9,  8},
    {10, 11, 13, 11, 12, 11, 11, 11, 10,  9, 10, 14, 12,  8,  8,  8}
};

static int silk_decode_frame(OpusContext *s, int frame, int channel, int active)
{
    int i, j, subframe, order;
    int voiced, qoffset_high, gain_indices[4], vq_index;
    int lsf_coeff[16];

    /* obtain stereo weights */
    if (s->packet.stereo && channel == 0) {
        int n, c[2], wi[2];
        n     = opus_rc_getsymbol(&s->rc, silk_model_stereo_s1);
        wi[0] = opus_rc_getsymbol(&s->rc, silk_model_stereo_s2) + 3*(n/5);
         c[0] = opus_rc_getsymbol(&s->rc, silk_model_stereo_s3);
        wi[1] = opus_rc_getsymbol(&s->rc, silk_model_stereo_s2) + 3*(n%5);
         c[1] = opus_rc_getsymbol(&s->rc, silk_model_stereo_s3);

        for (i=0; i<2; i++)
            s->silk.stereo_weights[i] = silk_stereo_weights[wi[i]]
                + (((silk_stereo_weights[wi[i]+1] - silk_stereo_weights[wi[i]]) * 6554) >> 16)
                    * (2*c[i] + 1);

        s->silk.stereo_weights[0] -= s->silk.stereo_weights[1];

        /* and read the mid-only flag */
        s->silk.midonly = active ? 0 : opus_rc_getsymbol(&s->rc, silk_model_mid_only);
    }

    /* obtain frame type */
    if (!active) {
        qoffset_high = opus_rc_getsymbol(&s->rc, silk_model_frame_type_unvoiced);
        voiced = 0;
    } else {
        int type = opus_rc_getsymbol(&s->rc, silk_model_frame_type_voiced);
        qoffset_high = type & 1;
        voiced = (type & 2) >> 1;
    }

    /* obtain subframe quantization gain indices */
    for (subframe = 0; subframe < s->silk.subframes; subframe++) {
        if (subframe == 0 && frame == 0) {
            /* gain index is coded absolute */
            int x = opus_rc_getsymbol(&s->rc, silk_model_gain[active + voiced]);
            gain_indices[0] = (x<<3) | opus_rc_getsymbol(&s->rc, silk_model_gain_lsbits);
        } else {
            /* gain index is coded relative */
            gain_indices[0] = opus_rc_getsymbol(&s->rc, silk_model_gain_delta);
        }
    }

    /* obtain normalized LSF coefficients */
    vq_index = opus_rc_getsymbol(&s->rc, silk_model_lsf_s1[s->packet.bandwidth ==
                                         OPUS_BANDWIDTH_WIDEBAND][voiced]);
    order = (s->packet.bandwidth != OPUS_BANDWIDTH_WIDEBAND) ? 10 : 16;
    for (i = 0; i < order; i++) {
        int index = (s->packet.bandwidth != OPUS_BANDWIDTH_WIDEBAND)
                    ? silk_coarse_codebook_nbmb[order][i] : silk_coarse_codebook_wb[order][i];
        lsf_coeff[i] = opus_rc_getsymbol(&s->rc, silk_model_lsf_vq2[index]);
    }
    return 0;
}

static int celt_decode_frame(OpusContext *s)
{
    return 0;
}

static av_cold int opus_decode_init(AVCodecContext *avctx)
{
    OpusContext *s = avctx->priv_data;

    av_dlog(avctx, "--> opus_decode_init <--\n");

    s->avctx = avctx;
    s->rc.gb = &s->gb;

    avctx->sample_fmt = AV_SAMPLE_FMT_S16;

    avcodec_get_frame_defaults(&s->frame);
    avctx->coded_frame = &s->frame;

    return 0;
}

static av_cold int opus_decode_close(AVCodecContext *avctx)
{
    av_dlog(avctx, "--> opus_decode_close <--\n");
    return 0;
}

static int opus_decode_frame(AVCodecContext *avctx, void *data,
                             int *got_frame_ptr, AVPacket *avpkt)
{
    int header = 0;
    int ret;
    OpusContext *s = avctx->priv_data;
    s->buf = avpkt->data;

    av_dlog(avctx, "\n\n--> opus_decode_frame <--\npts: %"PRId64"\n\n", avpkt->pts);
    *got_frame_ptr = 0;

    /* if this is a new packet, parse its header */
    if (s->currentframe == s->packet.frame_count) {
        s->buf_size = avpkt->size;
        if ((header = opus_parse_packet(s)) < 0) {
            av_log(avctx, AV_LOG_ERROR, "Error parsing packet\n");
            return header;
        }
        if (s->packet.frame_count == 0)
            return avpkt->size;

        s->buf += header;
        s->currentframe = 0;
        av_dlog(avctx, "TEST 1: %.4X %.4X %.4X %.4X %.4X %.4X %.4X %.4X %.4X %.4X\n",
            s->buf[0], s->buf[1], s->buf[2], s->buf[3], s->buf[4],
            s->buf[5], s->buf[6], s->buf[7], s->buf[8], s->buf[9]);
    }

    s->buf_size = s->packet.frame_size[s->currentframe];

    s->frame.nb_samples = s->packet.frame_duration * avctx->sample_rate / 48000;
    if ((ret = avctx->get_buffer(avctx, &s->frame)) < 0) {
        av_log(avctx, AV_LOG_ERROR, "get_buffer() failed\n");
        return ret;
    }

    init_get_bits(&s->gb, s->buf, s->buf_size<<3);
    opus_rc_init(&s->rc); /* reset the range decoder on each new Opus frame */

    if (s->packet.mode == OPUS_MODE_SILK || s->packet.mode == OPUS_MODE_HYBRID) {
        /* Decode 1-3 SILK frames */
        int i, j, ret, silkframes;
        int active[2][6], redundancy[2];

        silkframes        = 1 + (s->packet.frame_duration >= 1920)
                              + (s->packet.frame_duration == 2880);
        s->silk.subframes = (s->packet.frame_duration == 960) ? 2 : 4; // per whole SILK frame

        if (!s->packet.stereo)
            s->silk.previous_gain[1] = 0;

        /* read the LP-layer header bits */
        for (i = 0; i <= s->packet.stereo; i++) {
            for (j = 0; j < silkframes; j++)
                active[i][j] = opus_rc_getsymbol(&s->rc, rc_model_bit);

            redundancy[i] = opus_rc_getsymbol(&s->rc, rc_model_bit);
            if (redundancy[i]) {
                av_log(avctx, AV_LOG_ERROR, "LBRR frames present; this is unsupported\n");
                return AVERROR_PATCHWELCOME;
            }
        }

        for (i = 0; i < silkframes; i++) {
            for (j = 0; j <= s->packet.stereo; j++) {
                if ((ret = silk_decode_frame(s, i, j, active[j][i])) < 0) {
                    av_log(avctx, AV_LOG_ERROR, "Error reading SILK frame\n");
                    return ret;
                }
            }
        }
    } else {
        s->silk.previous_gain[0] = 0;
        s->silk.previous_gain[1] = 0;
    }

    if (s->packet.mode == OPUS_MODE_CELT || s->packet.mode == OPUS_MODE_HYBRID) {
        /* Decode a CELT frame */
        celt_decode_frame(s);
    }

    *got_frame_ptr   = 1;
    *(AVFrame *)data = s->frame;

    avpkt->duration -= s->packet.frame_duration;
    avpkt->pts = avpkt->dts += s->packet.frame_duration;

    /* more frames in the packet */
    if (++s->currentframe == s->packet.frame_count)
        return header + s->buf_size;

    /* skip padding at the end of the packet */
    return avpkt->size;
}

AVCodec ff_opus_decoder = {
    .name            = "opus",
    .type            = AVMEDIA_TYPE_AUDIO,
    .id              = CODEC_ID_OPUS,
    .priv_data_size  = sizeof(OpusContext),
    .init            = opus_decode_init,
    .close           = opus_decode_close,
    .decode          = opus_decode_frame,
    .capabilities    = CODEC_CAP_DR1 | CODEC_CAP_SUBFRAMES,
    .long_name       = NULL_IF_CONFIG_SMALL("Opus"),
};