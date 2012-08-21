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
 * Opus decoder
 * @author Andrew D'Addesio
 *
 * Codec homepage: http://opus-codec.org/
 * Specification: http://tools.ietf.org/html/draft-ietf-codec-opus
 * Ogg Opus specification: http://tools.ietf.org/html/draft-terriberry-oggopus
 *
 * Ogg-contained .opus files can be produced with opus-tools:
 * http://git.xiph.org/?p=opus-tools.git
 */

#include "avcodec.h"
#include "get_bits.h"
#include "bytestream.h"
#include "unary.h"
#include "mathops.h"
#include "celp_filters.h"
#include "opus.h"
#include "opusdata.h"

#define MAX_FRAME_SIZE 1275
#define MAX_FRAMES     48
#define MAX_FRAME_DUR  5760 /* in samples @ 48kHz */

#define ROUND_MULL(a,b,s) (((MUL64(a, b) >> (s - 1)) + 1) >> 1)
#define ilog(i) av_log2((i)<<1)

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
    const uint8_t *position;
    uint8_t byte;
    unsigned int unreadbits;
    unsigned int cache;
} RawBitsContext;

typedef struct {
    GetBitContext *gb;
    RawBitsContext rb;
    unsigned int range;
    unsigned int value;
} OpusRangeCoder;

typedef struct {
    int coded;
    int voiced;
    int log_gain;
    int16_t nlsf[16];
    int16_t lpc[16];
    float output[320];
    int primarylag;
} SilkFrame;

typedef struct {
    int midonly;
    int subframes;
    int sflength;
    int flength;

    SilkFrame prevframe[2];
    int stereo_weights[2];
} SilkContext;

typedef struct {
    int coded;
} CeltFrame;

typedef struct {
    CeltFrame prevframe[2];
} CeltContext;

typedef struct {
    AVCodecContext *avctx;
    AVFrame frame;
    GetBitContext gb;
    OpusRangeCoder rc;
    SilkContext silk;
    CeltContext celt;

    OpusPacket packet;
    int currentframe;
    uint8_t *buf;
    int buf_size;

    float output[2*MAX_FRAME_DUR];  /** stereo samples @ 48kHz */
} OpusContext;

static inline void opus_dprint_packet(AVCodecContext *avctx, OpusPacket *pkt)
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
 */
static inline int opus_parse_packet(OpusContext *s, int selfdelimited)
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
        if (frame_bytes & 1 || frame_bytes >> 1 > MAX_FRAME_SIZE)
            return AVERROR_INVALIDDATA;
        pkt->frame_offset[0] = ptr - s->buf;
        pkt->frame_size[0]   = frame_bytes >> 1;
        pkt->frame_offset[1] = pkt->frame_offset[0] + pkt->frame_size[0];
        pkt->frame_size[1]   = frame_bytes >> 1;
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
        pkt->bandwidth = pkt->config >> 2;
    } else if (pkt->config < 16) {
        pkt->mode = OPUS_MODE_HYBRID;
        pkt->bandwidth = OPUS_BANDWIDTH_SUPERWIDEBAND + (pkt->config >= 14);
    } else {
        pkt->mode = OPUS_MODE_CELT;
        pkt->bandwidth = (pkt->config - 16) >> 2;
        /* skip mediumband */
        if (pkt->bandwidth)
            pkt->bandwidth++;
    }

    /* update the raw bits location in the entropy decoder */
    s->rc.rb.position = end + pkt->padding;
    s->rc.rb.cache = 8;
    s->rc.rb.byte  = s->rc.rb.position[0];

    opus_dprint_packet(s->avctx, pkt);
    return ptr - s->buf;
}

/**
 * Range decoder
 */

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
    psymbol = ptotal - FFMIN(psymbol, ptotal);

    for (k = 0; (phigh = cdf[k]) <= psymbol; k++);
    plow = k ? cdf[k-1] : 0;

    rc->value -= scale * (ptotal - phigh);
    rc->range  = plow ? scale * (phigh - plow)
                      : rc->range - scale * (ptotal - phigh);

    opus_rc_normalize(rc);

    return k;
}

static unsigned int opus_rc_extract(OpusRangeCoder *rc, unsigned int ptotal)
{
    unsigned int k, scale, psymbol, plow, phigh;
    scale = rc->range / ptotal;
    psymbol = rc->value / scale;
    k = ptotal - FFMIN(psymbol+1, ptotal);

    plow = k;
    phigh = k+1;

    rc->value -= scale * (ptotal - phigh);
    rc->range  = plow ? scale * (phigh - plow)
                      : rc->range - scale * (ptotal - phigh);

    opus_rc_normalize(rc);

    return k;
}

/**
 * CELT: read raw bits at the end of the Opus frame, backwards byte-wise
 *
 * The spec says that it is impossible to read more raw bits
 * than there are actual bits in the frame, so we do not do bounds checking.
 */
static unsigned int opus_getrawbits(OpusRangeCoder *rc, unsigned int count)
{
    unsigned int value = 0, bit = 0;

    while (count) {
        if (count <= rc->rb.cache) {
            value |= (rc->rb.byte & ((1<<count)-1)) << bit;
            bit += count;
            rc->rb.cache -= count;
            rc->rb.byte >>= count;
        } else {
            value |= rc->rb.byte << bit;
            bit += rc->rb.cache;
            rc->rb.cache = 0;
        }

        if (rc->rb.cache == 0) {
            rc->rb.position--;
            rc->rb.cache = 8;
            rc->rb.byte = rc->rb.position[0];
        }
    }

    return value;
}

/**
 * CELT: read a uniform distribution
 */
static unsigned int opus_rc_getsymbol_uni(OpusRangeCoder *rc, unsigned int size){
    unsigned int bits, k, ptotal;

    bits = ilog(size-1);
    if (bits > 8) {
        /* combine with raw bits */
        size--;
        bits -= 8;
        ptotal = (size>>bits) + 1;
        k = opus_rc_extract(rc, ptotal);
        k = k << bits | opus_getrawbits(rc, bits);
        return FFMIN(k, size);
    } else
        return opus_rc_extract(rc, size);
}

/**
 * SILK decoder
 */

static inline void silk_stabilize_lsf(int16_t nlsf[16], int order, const uint16_t min_delta[17])
{
    int pass;
    for (pass = 0; 1; pass++) {
        int i, k, min_diff = 0;
        for (i = 0; i < order+1; i++) {
            int low  = i != 0     ? nlsf[i-1] : 0;
            int high = i != order ? nlsf[i]   : 32768;
            int diff = (high - low) - (min_delta[i]);

            if (diff < min_diff) {
                min_diff = diff;
                k = i;

                if (pass == 20)
                    break;
            }
        }
        if (min_diff == 0) /* no issues; stabilized */
            return;

        if (pass != 20) {
            /* wiggle one or two LSFs */
            if (k == 0) {
                /* repel away from lower bound */
                nlsf[0] = min_delta[0];
            } else if (k == order) {
                /* repel away from higher bound */
                nlsf[order-1] = 32768 - min_delta[order];
            } else {
                /* repel away from current position */
                int min_center = 0, max_center = 32768, center_val;

                /* lower extent */
                for (i = 0; i < k; i++)
                    min_center += min_delta[i];
                min_center += min_delta[k] >> 1;

                /* upper extent */
                for (i = order; i > k; i--)
                    max_center -= min_delta[k];
                max_center -= min_delta[k] >> 1;

                /* move apart */
                center_val = nlsf[k-1] + nlsf[k];
                center_val = (center_val>>1) + (center_val&1); // rounded divide by 2
                if (center_val < min_center)      center_val = min_center;
                else if (center_val > max_center) center_val = max_center;
                nlsf[k-1] = center_val - (min_delta[k]>>1);
                nlsf[k] = nlsf[k-1] + min_delta[k];
            }
        } else {
            /* resort to the fall-back method, the standard method for LSF stabilization */

            /* sort; as the LSFs should be nearly sorted, use insertion sort */
            for (i = 1; i < order; i++) {
                int j, value = nlsf[i];
                for (j = i-1; j >= 0 && nlsf[j] > value; j--)
                    nlsf[j+1] = nlsf[j];
                nlsf[j+1] = value;
            }

            /* push forwards to increase distance */
            if (nlsf[0] < min_delta[0])
                nlsf[0] = min_delta[0];
            for (i = 1; i < order; i++)
                if (nlsf[i] < nlsf[i-1] + min_delta[i])
                    nlsf[i] = nlsf[i-1] + min_delta[i];

            /* push backwards to increase distance */
            if (nlsf[order-1] > 32768 - min_delta[order])
                nlsf[order-1] = 32768 - min_delta[order];
            for (i = order-2; i >= 0; i--)
                if (nlsf[i] > nlsf[i+1] - min_delta[i+1])
                    nlsf[i] = nlsf[i+1] - min_delta[i+1];

            return;
        }
    }
}

static inline int silk_is_lpc_stable(const int16_t lpc[16], int order)
{
    int k, j, DC_resp = 0;
    int32_t lpc32[2][16];       // Q24
    int totalinvgain = 1 << 30; // 1.0 in Q30
    int32_t *row = lpc32[0], *prevrow;

    /* initialize the first row for the Levinson recursion */
    for (k = 0; k < order; k++) {
        DC_resp += lpc[k];
        row[k] = lpc[k] << 12;
    }

    if (DC_resp >= 4096)
        return 0;

    /* check if prediction gain pushes any coefficients too far */
    for (k = order - 1; 1; k--) {
        int rc;      // Q31; reflection coefficient
        int gaindiv; // Q30; inverse of the gain (the divisor)
        int gain;    // gain for this reflection coefficient
        int fbits;   // fractional bits used for the gain
        int error;   // Q29; estimate of the error of our partial estimate of 1/gaindiv

        if (FFABS(row[k]) > 16773022)
            return 0;

        rc      = -(row[k] << 7);
        gaindiv = (1<<30) - MULH(rc, rc);

        totalinvgain = MULH(totalinvgain, gaindiv) << 2;
        if (k == 0)
            return (totalinvgain >= 107374);

        /* approximate 1.0/gaindiv */
        fbits = ilog(gaindiv);
        gain  = ((1<<29) - 1) / (gaindiv >> (fbits+1-16)); // Q<fbits-16>
        error = (1<<29) - MULL(gaindiv << (15+16-fbits), gain, 16);
        gain  = ((gain << 16) + (error*gain >> 13));

        /* switch to the next row of the LPC coefficients */
        prevrow = row;
        row = lpc32[k & 1];

        for (j = 0; j < k; j++) {
            int x = prevrow[j] - ROUND_MULL(prevrow[k-j-1], rc, 31);
            row[j] = ROUND_MULL(x, gain, fbits);
        }
    }
}

static void silk_lsp2poly(const int32_t lsp[16], int32_t pol[16], int half_order)
{
    int i, j;

    pol[0] = 65536; // 1.0 in Q16
    pol[1] = -lsp[0];

    for (i = 1; i < half_order; i++) {
        pol[i+1] = (pol[i-1] << 1) - ROUND_MULL(lsp[2*i], pol[i], 16);
        for (j = i; j > 1; j--)
            pol[j] += pol[j-2] - ROUND_MULL(lsp[2*i], pol[j-1], 16);

        pol[1] -= lsp[2*i];
    }
}

static void silk_lsf2lpc(const int16_t nlsf[16], int16_t lpc[16], int order)
{
    int i, k;
    int32_t lsp[16];    // Q17; 2*cos(LSF)
    int32_t p[9], q[9]; // Q16
    int32_t lpc32[16];  // Q17

    /* convert the LSFs to LSPs, i.e. 2*cos(LSF) */
    for (k = 0; k < order; k++) {
        int index = nlsf[k] >> 8;
        int offset = nlsf[k] & 255;
        int k2 = (order == 10) ? silk_lsf_ordering_nbmb[k] : silk_lsf_ordering_wb[k];

        /* interpolate and round */
        lsp[k2]  = silk_cosine[index] << 8;
        lsp[k2] += (silk_cosine[index+1] - silk_cosine[index])*offset;
        lsp[k2]  = (lsp[k2] + 4) >> 3;
    }

    silk_lsp2poly(lsp  , p, order>>1);
    silk_lsp2poly(lsp+1, q, order>>1);

    /* reconstruct A(z) */
    for (k = 0; k < order>>1; k++) {
        lpc32[k]         = -p[k+1] - p[k] - q[k+1] + q[k];
        lpc32[order-k-1] = -p[k+1] - p[k] + q[k+1] - q[k];
    }

    /* limit the range of the LPC coefficients to each fit within an int16_t */
    for (i = 0; i < 10; i++) {
        int j;
        unsigned int maxabs = 0;
        for (j = 0, k = 0; j < order; j++) {
            unsigned int x = FFABS(lpc32[k]);
            if (x > maxabs) {
                maxabs = x; // Q17
                k      = j;
            }
        }

        maxabs = (maxabs + 16) >> 5; // convert to Q12

        if (maxabs > 32767) {
            /* perform bandwidth expansion */
            unsigned int chirp, chirp_base; // Q16
            maxabs = FFMIN(maxabs, 163838); // anything above this overflows chirp's numerator
            chirp_base = chirp = 65470 - ((maxabs - 32767) << 14) / ((maxabs * (k+1)) >> 2);

            for (k = 0; k < order; k++) {
                lpc32[k] = ROUND_MULL(lpc32[k], chirp, 16);
                chirp    = (chirp_base * chirp + 32768) >> 16;
            }
        } else break;
    }

    if (i == 10) {
        /* time's up: just clamp */
        for (k = 0; k < order; k++) {
            int x = (lpc32[k] + 16) >> 5;
            lpc[k] = av_clip_int16(x);
            lpc32[k] = lpc[k] << 5; // shortcut mandated by the spec; drops lower 5 bits
        }
    } else {
        for (k = 0; k < order; k++)
            lpc[k] = (lpc32[k] + 16) >> 5;
    }

    /* if the prediction gain causes the LPC filter to become unstable,
       apply further bandwidth expansion on the Q17 coefficients */
    for (i = 1; i <= 16 && !silk_is_lpc_stable(lpc, order); i++) {
        unsigned int chirp, chirp_base;
        chirp_base = chirp = 65536 - (1 << i);

        for (k = 0; k < order; k++) {
            lpc32[k] = ROUND_MULL(lpc32[k], chirp, 16);
            lpc[k]   = (lpc32[k] + 16) >> 5;
            chirp    = (chirp_base * chirp + 32768) >> 16;
        }
    }
}

static inline void silk_decode_lpc(OpusContext *s, int16_t lpc_leadin[16], int16_t lpc[16],
                                   int *lpc_order, int *has_lpc_leadin, int voiced, int channel)
{
    int i;
    int order;                   // order of the LP polynomial; 10 for NB/MB and 16 for WB
    int8_t  lsf_i1, lsf_i2[16];  // stage-1 and stage-2 codebook indices
    int16_t lsf_res[16];         // residual as a Q10 value
    int16_t nlsf[16];            // Q15

    *lpc_order = order = (s->packet.bandwidth != OPUS_BANDWIDTH_WIDEBAND) ? 10 : 16;

    /* obtain LSF stage-1 and stage-2 indices */
    lsf_i1 = opus_rc_getsymbol(&s->rc, silk_model_lsf_s1[s->packet.bandwidth ==
                                       OPUS_BANDWIDTH_WIDEBAND][voiced]);
    for (i = 0; i < order; i++) {
        int index = (s->packet.bandwidth != OPUS_BANDWIDTH_WIDEBAND)
                    ? silk_lsf_s2_model_sel_nbmb[lsf_i1][i]
                    : silk_lsf_s2_model_sel_wb[lsf_i1][i];
        lsf_i2[i] = opus_rc_getsymbol(&s->rc, silk_model_lsf_s2[index]) - 4;
        if (lsf_i2[i] == -4)     lsf_i2[i] -= opus_rc_getsymbol(&s->rc, silk_model_lsf_s2_ext);
        else if (lsf_i2[i] == 4) lsf_i2[i] += opus_rc_getsymbol(&s->rc, silk_model_lsf_s2_ext);
    }

    /* reverse the backwards-prediction step */
    for (i = order - 1; i >= 0; i--) {
        int qstep = (s->packet.bandwidth != OPUS_BANDWIDTH_WIDEBAND) ? 11796 : 9830;

        lsf_res[i] = lsf_i2[i] << 10;
        if (lsf_i2[i] < 0)      lsf_res[i] += 102;
        else if (lsf_i2[i] > 0) lsf_res[i] -= 102;
        lsf_res[i] = (lsf_res[i] * qstep) >> 16;

        if (i+1 < order) {
            int weight = (s->packet.bandwidth != OPUS_BANDWIDTH_WIDEBAND)
                         ? silk_lsf_pred_weights_nbmb[silk_lsf_weight_sel_nbmb[lsf_i1][i]][i]
                         : silk_lsf_pred_weights_wb[silk_lsf_weight_sel_wb[lsf_i1][i]][i];
            lsf_res[i] += (lsf_res[i+1] * weight) >> 8;
        }
    }

    /* reconstruct the NLSF coefficients from the supplied indices */
    for (i = 0; i < order; i++) {
        const uint8_t * codebook = (s->packet.bandwidth != OPUS_BANDWIDTH_WIDEBAND)
                                   ? silk_lsf_codebook_nbmb[lsf_i1]
                                   : silk_lsf_codebook_wb[lsf_i1];
        int cur, prev, next, weight_sq, weight, ipart, fpart, y, value;

        /* find the weight of the residual */
        /* TODO: precompute */
        cur = codebook[i];
        prev = i ? codebook[i-1] : 0;
        next = i+1 < order ? codebook[i+1] : 256;
        weight_sq = (1024/(cur - prev) + 1024/(next - cur)) << 16;

        /* approximate square-root with mandated fixed-point arithmetic */
        ipart = ilog(weight_sq);
        fpart = (weight_sq >> (ipart-8)) & 127;
        y = ((ipart&1) ? 32768 : 46214) >> ((32-ipart)>>1);
        weight = y + ((213*fpart*y) >> 16);

        value = (cur << 7) + (lsf_res[i] << 14) / weight;
        nlsf[i] = av_clip(value, 0, 32767);
    }

    /* stabilize the NLSF coefficients */
    silk_stabilize_lsf(nlsf, order, (s->packet.bandwidth != OPUS_BANDWIDTH_WIDEBAND)
                                    ? silk_lsf_min_spacing_nbmb : silk_lsf_min_spacing_wb);

    /* produce an interpolation for the first 2 subframes, */
    /* and then convert both sets of NLSFs to LPC coefficients */
    *has_lpc_leadin = 0;
    if (s->silk.subframes == 4) {
        int offset = opus_rc_getsymbol(&s->rc, silk_model_lsf_interpolation_offset);
        if (offset != 4 && s->silk.prevframe[channel].coded) {
            *has_lpc_leadin = 1;
            if (offset != 0) {
                int16_t nlsf_leadin[16];
                for (i = 0; i < order; i++)
                    nlsf_leadin[i] = s->silk.prevframe[channel].nlsf[i] +
                        ((nlsf[i] - s->silk.prevframe[channel].nlsf[i]) * offset >> 2);
                silk_lsf2lpc(nlsf_leadin, lpc_leadin, order);
            } else  /* avoid re-computation for a (roughly) 1-in-4 occurrence */
                memcpy(lpc_leadin, s->silk.prevframe[channel].lpc, sizeof(lpc_leadin));
        }

        silk_lsf2lpc(nlsf, lpc, order);
        memcpy(s->silk.prevframe[channel].nlsf, nlsf, sizeof(nlsf));
        memcpy(s->silk.prevframe[channel].lpc, lpc, sizeof(lpc));
    } else
        silk_lsf2lpc(nlsf, lpc, order);
}

static inline void silk_count_children(OpusRangeCoder *rc, int model, int32_t total,
                                       int32_t child[2])
{
    if (total != 0) {
        child[0] = opus_rc_getsymbol(rc,
                       silk_model_pulse_location[model] + (((total-1 + 5) * (total-1)) >> 1));
        child[1] = total - child[0];
    } else {
        child[0] = 0;
        child[1] = 0;
    }
}

static inline void silk_decode_excitation(OpusContext *s, float excitationf[320],
                                          int qoffset_high, int active, int voiced)
{
    int i;
    int32_t seed;
    int shellblocks;
    int ratelevel;
    uint8_t pulsecount[20];     // total pulses in each shell block
    uint8_t lsbcount[20] = {0}; // raw lsbits defined for each pulse in each shell block
    int32_t excitation[320];    // Q23

    /* excitation parameters */
    seed = opus_rc_getsymbol(&s->rc, silk_model_lcg_seed);
    shellblocks = silk_shell_blocks[s->packet.bandwidth][s->silk.subframes >> 2];
    ratelevel = opus_rc_getsymbol(&s->rc, silk_model_exc_rate[voiced]);

    for (i = 0; i < shellblocks; i++) {
        pulsecount[i] = opus_rc_getsymbol(&s->rc, silk_model_pulse_count[ratelevel]);
        if (pulsecount[i] == 17) {
            while (pulsecount[i] == 17 && ++lsbcount[i] != 10)
                pulsecount[i] = opus_rc_getsymbol(&s->rc, silk_model_pulse_count[9]);
            if (lsbcount[i] == 10)
                pulsecount[i] = opus_rc_getsymbol(&s->rc, silk_model_pulse_count[10]);
        }
    }

    /* decode pulse locations using PVQ */
    for (i = 0; i < shellblocks; i++) {
        if (pulsecount[i] != 0) {
            int a, b, c, d;
            int32_t * location = excitation + 16*i;
            int32_t branch[4][2];
            branch[0][0] = pulsecount[i];

            /* unrolled tail recursion */
            for (a = 0; a < 1; a++) {
                silk_count_children(&s->rc, 0, branch[0][a], branch[1]);
                for (b = 0; b < 2; b++) {
                    silk_count_children(&s->rc, 1, branch[1][b], branch[2]);
                    for (c = 0; c < 2; c++) {
                        silk_count_children(&s->rc, 2, branch[2][c], branch[3]);
                        for (d = 0; d < 2; d++) {
                            silk_count_children(&s->rc, 3, branch[3][d], location);
                            location += 2;
                        }
                    }
                }
            }
        } else
            memset(excitation + 16*i, 0, 16*sizeof(int32_t));
    }

    /* decode least significant bits */
    for (i = 0; i < shellblocks << 4; i++) {
        int bit;
        for (bit = 0; bit < lsbcount[i >> 4]; bit++)
            excitation[i] = (excitation[i] << 1) |
                            opus_rc_getsymbol(&s->rc, silk_model_excitation_lsb);
    }

    /* decode signs */
    for (i = 0; i < shellblocks << 4; i++) {
        if (excitation[i] != 0) {
            int sign = opus_rc_getsymbol(&s->rc, silk_model_excitation_sign[active+
                                         voiced][qoffset_high][FFMIN(pulsecount[i >> 4], 6)]);
            if (sign == 0)
                excitation[i] *= -1;
        }
    }

    /* assemble the excitation */
    for (i = 0; i < shellblocks << 4; i++) {
        int value = excitation[i];
        excitation[i] = (value << 8) | silk_quant_offset[voiced][qoffset_high];
        if (value < 0)      excitation[i] += 20;
        else if (value > 0) excitation[i] -= 20;

        /* invert samples pseudorandomly */
        seed = 196314165*seed + 907633515;
        if (seed < 0)
            excitation[i] *= -1;
        seed += value;

        excitationf[i] = (float)excitation[i] / 8388608.0f;
    }
}

static inline int silk_decode_frame(OpusContext *s, int frame, int channel, int active)
{
    int i, idx;

    /* per frame */
    int voiced;       // combines with active to indicate inactive, active, or active+voiced
    int qoffset_high;
    int order;
    int16_t lpc_leadin[16], lpc_body[16]; // Q12
    int has_lpc_leadin;
    float lpc[16];
    float ltpscale;
    float excitation[320];
    float sLTP[640] = {0};

    /* per subframe */
    struct {
        float gain;
        int pitchlag;
        float ltptaps[5];
    } sf[4];

    SilkFrame * const prevframe = s->silk.prevframe + channel;

    /* obtain stereo weights */
    if (s->packet.stereo && channel == 0) {
        int n, wi[2], ws[2];
        n     = opus_rc_getsymbol(&s->rc, silk_model_stereo_s1);
        wi[0] = opus_rc_getsymbol(&s->rc, silk_model_stereo_s2) + 3*(n/5);
        ws[0] = opus_rc_getsymbol(&s->rc, silk_model_stereo_s3);
        wi[1] = opus_rc_getsymbol(&s->rc, silk_model_stereo_s2) + 3*(n%5);
        ws[1] = opus_rc_getsymbol(&s->rc, silk_model_stereo_s3);

        for (i=0; i<2; i++)
            s->silk.stereo_weights[i] = silk_stereo_weights[wi[i]]
                + (((silk_stereo_weights[wi[i]+1] - silk_stereo_weights[wi[i]]) * 6554) >> 16)
                    * (ws[i]*2 + 1);

        s->silk.stereo_weights[0] -= s->silk.stereo_weights[1];

        /* and read the mid-only flag */
        s->silk.midonly = active ? 0 : opus_rc_getsymbol(&s->rc, silk_model_mid_only);
    }

    /* obtain frame type */
    if (!active) {
        qoffset_high = opus_rc_getsymbol(&s->rc, silk_model_frame_type_inactive);
        voiced = 0;
    } else {
        int type = opus_rc_getsymbol(&s->rc, silk_model_frame_type_active);
        qoffset_high = type & 1;
        voiced = type >> 1;
    }

    /* obtain subframe quantization gains */
    for (i = 0; i < s->silk.subframes; i++) {
        int log_gain;     //Q7
        int ipart, fpart, lingain;

        if (i == 0 && (frame == 0 || !prevframe->coded)) {
            /* gain is coded absolute */
            int x = opus_rc_getsymbol(&s->rc, silk_model_gain_highbits[active + voiced]);
            log_gain = (x<<3) | opus_rc_getsymbol(&s->rc, silk_model_gain_lowbits);

            if (prevframe->coded)
                log_gain = FFMAX(log_gain, prevframe->log_gain - 16);
        } else {
            /* gain is coded relative */
            int delta_gain = opus_rc_getsymbol(&s->rc, silk_model_gain_delta);
            log_gain = av_clip(FFMAX((delta_gain<<1) - 16,
                                     prevframe->log_gain + delta_gain - 4), 0, 63);
        }

        prevframe->log_gain = log_gain;

        /* approximate 2**(x/128) with a Q7 (i.e. non-integer) input */
        log_gain = (log_gain * 0x1D1C71 >> 16) + 2090;
        ipart = log_gain >> 7;
        fpart = log_gain & 127;
        lingain = (1<<ipart) + ((-174 * fpart * (128-fpart) >>16) + fpart) * ((1<<ipart) >> 7);
        sf[i].gain = (float)lingain / 65536.0f;
    }

    /* obtain LPC filter coefficients */
    silk_decode_lpc(s, lpc_leadin, lpc_body, &order, &has_lpc_leadin, voiced, channel);

    /* obtain pitch lags, if this is a voiced frame */
    if (voiced) {
        int primarylag;         // primary pitch lag for the entire SILK frame
        int ltpfilter;
        const int8_t * offsets;

        if (frame == 0 || !prevframe->coded || !prevframe->voiced) {
            /* primary lag is coded absolute */
            int highbits, lowbits;
            const uint16_t *model[] = {
                silk_model_pitch_lowbits_nb, silk_model_pitch_lowbits_mb,
                silk_model_pitch_lowbits_wb
            };
            highbits = opus_rc_getsymbol(&s->rc, silk_model_pitch_highbits);
            lowbits  = opus_rc_getsymbol(&s->rc, model[s->packet.bandwidth]);

            primarylag = silk_pitch_min_lag[s->packet.bandwidth] +
                         highbits*silk_pitch_scale[s->packet.bandwidth] + lowbits;
        } else {
            /* primary lag is coded relative */
            primarylag = prevframe->primarylag +
                         opus_rc_getsymbol(&s->rc, silk_model_pitch_delta) - 9;
        }
        prevframe->primarylag = primarylag;

        if (s->silk.subframes == 2)
            offsets = (s->packet.bandwidth == OPUS_BANDWIDTH_NARROWBAND)
                     ? silk_pitch_offset_nb10ms[opus_rc_getsymbol(&s->rc,
                                                silk_model_pitch_contour_nb10ms)]
                     : silk_pitch_offset_mbwb10ms[opus_rc_getsymbol(&s->rc,
                                                silk_model_pitch_contour_mbwb10ms)];
        else
            offsets = (s->packet.bandwidth == OPUS_BANDWIDTH_NARROWBAND)
                     ? silk_pitch_offset_nb20ms[opus_rc_getsymbol(&s->rc,
                                                silk_model_pitch_contour_nb20ms)]
                     : silk_pitch_offset_mbwb20ms[opus_rc_getsymbol(&s->rc,
                                                silk_model_pitch_contour_mbwb20ms)];

        for (i = 0; i < s->silk.subframes; i++)
            sf[i].pitchlag = av_clip(primarylag + offsets[i],
                                     silk_pitch_min_lag[s->packet.bandwidth],
                                     silk_pitch_max_lag[s->packet.bandwidth]);

        /* obtain LTP filter coefficients */
        ltpfilter = opus_rc_getsymbol(&s->rc, silk_model_ltp_filter);
        for (i = 0; i < s->silk.subframes; i++) {
            int index, j;
            const uint16_t *filter_sel[] = {
                silk_model_ltp_filter0_sel, silk_model_ltp_filter1_sel,
                silk_model_ltp_filter2_sel
            };
            const int8_t (*filter_taps[])[5] = {
                silk_ltp_filter0_taps, silk_ltp_filter1_taps, silk_ltp_filter2_taps
            };
            index = opus_rc_getsymbol(&s->rc, filter_sel[ltpfilter]);
            for (j = 0; j < 5; j++)
                sf[i].ltptaps[j] = (float)filter_taps[ltpfilter][index][j] / 128.0f;
        }
    }

    /* obtain LTP scale factor */
    if (voiced && frame == 0)
        ltpscale = silk_ltp_scale_factor[opus_rc_getsymbol(&s->rc,
                                         silk_model_ltp_scale_index)] / 16384.0f;
    else ltpscale = (float)15565/16384.0f;

    /* generate the excitation signal for the entire frame */
    silk_decode_excitation(s, excitation, qoffset_high, active, voiced);

    for (i = 0; i < s->silk.flength; i++)
        av_dlog(NULL, "excitation[%d] = %f\n", i, excitation[i]);

    /* generate the output signal */
    for (i = 0, idx = s->silk.flength; i < s->silk.subframes; i++, idx += s->silk.sflength) {
        int j, k;

        if (i == 0 || (i == 2 && has_lpc_leadin)) {
            const int16_t * lpcin = (i == 0) ? lpc_leadin : lpc_body;

            /* convert the LPC coefficients to floating point */
            for (k = 0; k < order; k++)
                lpc[k] = (float)lpcin[k] / 4096.0f;

            if (voiced) {
                /* since the LPC coefficients likely changed, */
                /* run the output history through a rewhiten filter */
                int buffer_length = sf[i].pitchlag + order + 2;

                for (j = idx - buffer_length; j < s->silk.flength; j++) {
                    sLTP[j] = prevframe->output[j];
                    for (k = 1; k <= order; k++)
                        sLTP[j] -= lpc[k-1] * prevframe->output[j-k];
                }
                for (j = idx - buffer_length; j < s->silk.flength; j++)
                    sLTP[j] = av_clipf(sLTP[j], -1.0, 1.0) * ltpscale / sf[i].gain;
            }
        }

        if (voiced) {
            /* LTP synthesis */
            for (j = 0; j < 5; j++)
                av_dlog(NULL, "ltptaps[%d] = %f\n", i, sf[i].ltptaps[j]);
            //getchar();

            for (j = idx; j < idx + s->silk.sflength; j++) {
                sLTP[j] = excitation[j - s->silk.flength];
                for (k = 0; k < 5; k++)
                    sLTP[j] += sf[i].ltptaps[k] * sLTP[j - sf[i].pitchlag + 2 - k];
            }
        } else
            memcpy(sLTP + idx, excitation + idx - s->silk.flength,
                   s->silk.sflength * sizeof(float));

        for (j = idx; j < idx + s->silk.sflength; j++)
            sLTP[j] *= sf[i].gain;

        for (j = idx; j < idx + s->silk.sflength; j++)
            av_dlog(NULL, "sLTP[%d] = %f\n", j, sLTP[j]);
        //getchar();

        /* LPC synthesis */
        for (j = idx; j < idx + s->silk.sflength; j++) {
            for (k = 1; k <= order; k++)
                sLTP[j] += lpc[k-1] * sLTP[j-k];
        }

        for (j = idx; j < idx + s->silk.sflength; j++)
            av_dlog(NULL, "sLTP[%d] = %f\n", j, sLTP[j]);
        //getchar();
    }

    memcpy(prevframe->output, sLTP + s->silk.flength, s->silk.flength * sizeof(float));

    return 0;
}

/**
 * CELT decoder
 */

static int celt_decode_frame(OpusContext *s, int hybrid)
{
    int silence;
    int has_postfilter;
    int pf_octave;
    int pf_period;
    int pf_gain;
    int pf_tapset;
    int transient;
    int intra;
    int coarseenergy;
    int tfchange;
    int tfselect;
    int spread;
    int dynalloc;
    int alloctrim;
    int skip;
    int intensity;
    int dual;
    int fineenergy;
    int residual;
    int anticollapse;
    int finalize;

    silence = opus_rc_getsymbol(&s->rc, celt_model_silence);
    has_postfilter = opus_rc_getsymbol(&s->rc, rc_model_bit);

    if (has_postfilter) {
        pf_octave = opus_rc_getsymbol_uni(&s->rc, 6);
        pf_period = opus_getrawbits(&s->rc, 4+pf_octave);
        pf_gain = opus_getrawbits(&s->rc, 3);
        pf_tapset = opus_rc_getsymbol(&s->rc, celt_model_tapset);
    }

    transient = opus_rc_getsymbol(&s->rc, celt_model_transient_intra);
    intra = opus_rc_getsymbol(&s->rc, celt_model_transient_intra);

    /* ... */

    spread = opus_rc_getsymbol(&s->rc, celt_model_spread);

    /* ... */

    alloctrim = opus_rc_getsymbol(&s->rc, celt_model_alloc_trim);
    return 0;
}

/**
 * Opus stream decoder
 */

static av_cold int opus_decode_init(AVCodecContext *avctx)
{
    OpusContext *s = avctx->priv_data;

    av_dlog(avctx, "--> opus_decode_init <--\n");

    s->avctx = avctx;
    s->rc.gb = &s->gb;

    avctx->sample_fmt = AV_SAMPLE_FMT_FLT;
    avctx->channels = 1;

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
    int i, j, ret;
    OpusContext *s = avctx->priv_data;
    s->buf = avpkt->data;

    av_dlog(avctx, "\n\n--> opus_decode_frame <--\npts: %"PRId64"\n\n", avpkt->pts);
    *got_frame_ptr = 0;

    /* if this is a new packet, parse its header */
    if (s->currentframe == s->packet.frame_count) {
        s->buf_size = avpkt->size;
        if ((header = opus_parse_packet(s, 0)) < 0) {
            av_log(avctx, AV_LOG_ERROR, "Error parsing packet\n");
            return header;
        }
        if (s->packet.frame_count == 0)
            return avpkt->size;

        s->buf += header;
        s->currentframe = 0;
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
        int silkframes;
        int active[2][6], redundancy[2];

        silkframes        = 1 + (s->packet.frame_duration >= 1920)
                              + (s->packet.frame_duration == 2880);
        s->silk.subframes = (s->packet.frame_duration == 480) ? 2 : 4; // per whole SILK frame
        s->silk.sflength  = 20 * (s->packet.bandwidth + 2);
        s->silk.flength   = s->silk.sflength * ((s->packet.frame_duration == 480) ? 2 : 4);

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
            float * data = (float*) s->frame.data[0];
            for (j = 0; j <= s->packet.stereo; j++) {
                if ((ret = silk_decode_frame(s, i, j, active[j][i])) < 0) {
                    av_log(avctx, AV_LOG_ERROR, "Error reading SILK frame\n");
                    return ret;
                }
            }

            for (j = i * s->silk.flength; j < (i + 1) * s->silk.flength; j++)
                data[j] = av_clipf(s->silk.prevframe[0].output[j], -1.0, 1.0);

            s->silk.prevframe[0].coded = 1;
            s->silk.prevframe[1].coded = s->packet.stereo;
        }
    } else {
        s->silk.prevframe[0].coded = 0;
        s->silk.prevframe[0].coded = 0;
    }

    if (s->packet.mode == OPUS_MODE_CELT || s->packet.mode == OPUS_MODE_HYBRID) {
        /* Decode a CELT frame */
        celt_decode_frame(s, s->packet.mode == OPUS_MODE_HYBRID);

        s->celt.prevframe[0].coded = 1;
    } else {
        s->celt.prevframe[0].coded = 0;
    }

    *got_frame_ptr   = 1;
    *(AVFrame *)data = s->frame;

    avpkt->duration -= s->packet.frame_duration;
    avpkt->pts = avpkt->dts += s->packet.frame_duration;

    /* more frames in the packet */
    if (++s->currentframe != s->packet.frame_count)
        return header + s->buf_size;

    /* skip padding at the end of the packet */
    return avpkt->size;
}

static av_cold void opus_decode_flush(AVCodecContext *ctx)
{
    OpusContext *s = ctx->priv_data;
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
    .flush           = opus_decode_flush,
    .long_name       = NULL_IF_CONFIG_SMALL("Opus"),
};