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
 * Specification: http://tools.ietf.org/html/rfc6716
 * Ogg Opus specification: http://tools.ietf.org/html/draft-terriberry-oggopus
 *
 * Ogg-contained .opus files can be produced with opus-tools:
 * http://git.xiph.org/?p=opus-tools.git
 */

#include "avcodec.h"
#include "get_bits.h"
#include "mathops.h"
#include "celp_filters.h"
#include "dsputil.h"
#include "fft.h"
#include "opus.h"
#include "opusdata.h"

#define MAX_FRAME_SIZE               1275
#define MAX_FRAMES                   48
#define MAX_FRAME_DUR                5760

#define SILK_HISTORY                 306
#define SILK_MAX_LPC                 16

#define CELT_HISTORY                 960
#define CELT_MAX_BANDS               21
#define CELT_VECTORS                 11
#define CELT_ALLOC_STEPS             6
#define CELT_FINE_OFFSET             21
#define CELT_MAX_FINE_BITS           8
#define CELT_NORM_SCALE              16384
#define CELT_FINE_OFFSET             21
#define CELT_QTHETA_OFFSET           4
#define CELT_QTHETA_OFFSET_TWOPHASE  16

#define ROUND_MULL(a,b,s) (((MUL64(a, b) >> (s - 1)) + 1) >> 1)
#define ROUND_MUL16(a,b)  ((MUL16(a, b) + 16384) >> 15)
#define ilog(i) ((i) ? av_log2(i)+1 : 0)

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

enum CeltSpread {
    CELT_SPREAD_NONE,
    CELT_SPREAD_LIGHT,
    CELT_SPREAD_NORMAL,
    CELT_SPREAD_AGGRESSIVE
};

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
    unsigned int bytes;
    unsigned int cachelen;
    unsigned int cacheval;
} RawBitsContext;

typedef struct {
    GetBitContext *gb;
    RawBitsContext rb;
    unsigned int range;
    unsigned int value;
    unsigned int total_read_bits;
} OpusRangeCoder;

typedef struct {
    int coded;
    int voiced;
    int log_gain;
    int16_t nlsf[16];
    float lpc[16];
    float output[SILK_HISTORY];
    float lpc_history[16];
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
    float band_energy[CELT_MAX_BANDS];
    float log_band_energy[2][CELT_MAX_BANDS];
    float output[CELT_HISTORY];
} CeltFrame;

typedef struct {
    int framebytes;
    int framebits;
    int duration;

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
        rc->value = ((rc->value << 8) | (255 - get_bits(rc->gb, 8))) & ((1u<<31)-1);
        rc->range <<= 8;
        av_dlog(NULL, "--end--   value: %u\n          range: %u\n", rc->value, rc->range);
        rc->total_read_bits += 8;
    }
}

static inline void opus_rc_init(OpusRangeCoder *rc)
{
    rc->range = 128;
    rc->value = 127 - get_bits(rc->gb, 7);
    rc->total_read_bits = 9;
    av_dlog(NULL, "[rc init] range: %u, value: %u\n", rc->range, rc->value);
    opus_rc_normalize(rc);
}

static inline void opus_rc_seek(OpusRangeCoder *rc, unsigned int scale,
                                unsigned int plow, unsigned int phigh, unsigned int ptotal)
{
    rc->value -= scale * (ptotal - phigh);
    rc->range  = plow ? scale * (phigh - plow)
                      : rc->range - scale * (ptotal - phigh);
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

    opus_rc_seek(rc, scale, plow, phigh, ptotal);

    return k;
}

static unsigned int opus_rc_p2model(OpusRangeCoder *rc, unsigned int bits)
{
    unsigned int k, scale;
    scale = rc->range >> bits; // in this case, scale = psymbol
    k = rc->value < scale;     // in this case, k = plow

    if(k == 0)
        rc->value -= scale;
    rc->range = k ? scale : rc->range - scale;
    opus_rc_normalize(rc);

    return k;
}

static inline void opus_raw_init(OpusRangeCoder *rc, const uint8_t *rightend,
                                 unsigned int bytes)
{
    rc->rb.position = rightend;
    rc->rb.bytes    = bytes;
    rc->rb.cachelen = 0;
    rc->rb.cacheval = 0;
}

/**
 * CELT: read 1-25 raw bits at the end of the frame, backwards byte-wise
 */
static unsigned int opus_getrawbits(OpusRangeCoder *rc, unsigned int count)
{
    unsigned int value = 0;

    while (rc->rb.bytes && rc->rb.cachelen < count) {
        rc->rb.cacheval |= *--rc->rb.position << rc->rb.cachelen;
        rc->rb.cachelen += 8;
        rc->rb.bytes--;
    }

    value = rc->rb.cacheval & ((1<<count)-1);
    rc->rb.cacheval >>= count;
    rc->rb.cachelen -= count;
    rc->total_read_bits += count;

    return value;
}

/**
 * CELT: read a uniform distribution
 */
static unsigned int opus_rc_unimodel(OpusRangeCoder *rc, unsigned int size)
{
    unsigned int bits, k, scale, ptotal;

    bits = ilog(size-1);
    ptotal = (bits > 8) ? ((size-1) >> (bits-8)) + 1 : size;

    scale  = rc->range / ptotal;
    k      = rc->value / scale + 1;
    k      = ptotal - FFMIN(k, ptotal);
    opus_rc_seek(rc, scale, k, k + 1, ptotal);

    if (bits > 8) {
        k = k << (bits-8) | opus_getrawbits(rc, bits-8);
        return FFMIN(k, size-1);
    } else
        return k;
}

static inline int opus_rc_laplace(OpusRangeCoder *rc, unsigned int psymbol, int decay)
{
    /* extends the range coder to model a Laplace distribution */
    int value = 0;
    unsigned int scale, plow = 0, pcenter;

    scale = rc->range >> 15;
    pcenter = rc->value / scale + 1;
    pcenter = (1<<15) - FFMIN(pcenter, 1<<15);

    if (pcenter >= psymbol) {
        value++;
        plow = psymbol;
        psymbol = 1 + ((32768 - 32 - psymbol) * (16384-decay) >> 15);

        while(psymbol > 1 && pcenter >= plow + 2*psymbol) {
            value++;
            psymbol *= 2;
            plow += psymbol;
            psymbol = (((psymbol-2) * decay) >> 15) + 1;
        }

        if (psymbol <= 1) {
            int distance = (pcenter - plow) >> 1;
            value += distance;
            plow += 2*distance;
        }

        if (pcenter < plow + psymbol)
            value *= -1;
        else
            plow += psymbol;
    }

    opus_rc_seek(rc, scale, plow, FFMIN(plow+psymbol, 32768), 32768);

    return value;
}

static inline unsigned int opus_rc_stepmodel(OpusRangeCoder *rc, int k0)
{
    /* Use a probability of 3 up to itheta=8192 and then use 1 after */
    unsigned int k, scale, psymbol, ptotal = (k0+1)*3 + k0;
    scale = rc->range / ptotal;
    psymbol = rc->value / scale + 1;
    psymbol = ptotal - FFMIN(psymbol, ptotal);

    k = (psymbol < (k0+1)*3) ? psymbol/3 : psymbol - (k0+1)*2;

    opus_rc_seek(rc, scale, (k <= k0) ? 3*(k+0) : (k-1-k0) + 3*(k0+1),
                            (k <= k0) ? 3*(k+1) : (k-0-k0) + 3*(k0+1), ptotal);
    return k;
}

static inline unsigned int opus_rc_trimodel(OpusRangeCoder *rc, int qn)
{
    unsigned int k, scale, psymbol, ptotal, plow, pcenter;

    ptotal = ((qn>>1) + 1) * ((qn>>1) + 1);
    scale   = rc->range / ptotal;
    pcenter = rc->value / scale + 1;
    pcenter = ptotal - FFMIN(pcenter, ptotal);

    if (pcenter < ptotal >> 1) {
        k       = (ff_sqrt(8*pcenter + 1) - 1) >> 1;
        plow    = k * (k + 1) >> 1;
        psymbol = k + 1;
    } else {
        k       = (2*(qn + 1) - ff_sqrt(8*(ptotal - pcenter - 1) + 1)) >> 1;
        plow    = ptotal - ((qn + 1 - k) * (qn + 2 - k) >> 1);
        psymbol = qn + 1 - k;
    }

    opus_rc_seek(rc, scale, plow, plow+psymbol, ptotal);

    return k;
}

/**
 * CELT: estimate bits of entropy that have thus far been consumed for the
 *       current CELT frame, to integer and fractional (1/8th bit) precision
 */
static inline unsigned int opus_rc_tell(const OpusRangeCoder *rc)
{
    return rc->total_read_bits - ilog(rc->range);
}

static unsigned int opus_rc_tell_frac(const OpusRangeCoder *rc)
{
    unsigned int i, total_bits, rcbuffer, range;

    total_bits = rc->total_read_bits << 3;
    rcbuffer   = ilog(rc->range);
    range      = rc->range >> (rcbuffer-16);

    for (i = 0; i < 3; i++) {
        int bit;
        range = range * range >> 15;
        bit = range >> 16;
        rcbuffer = rcbuffer << 1 | bit;
        range >>= bit;
    }

    return total_bits - rcbuffer;
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

static void silk_lsf2lpc(const int16_t nlsf[16], float lpcf[16], int order)
{
    int i, k;
    int32_t lsp[16];     // Q17; 2*cos(LSF)
    int32_t p[9], q[9];  // Q16
    int32_t lpc32[16];   // Q17
    int16_t lpc[16];     // Q12

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

    for (i = 0; i < order; i++)
        lpcf[i] = (float)lpc[i] / 4096.0f;
}

static inline void silk_decode_lpc(OpusContext *s, float lpc_leadin[16], float lpc[16],
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
                memcpy(lpc_leadin, s->silk.prevframe[channel].lpc, 16*sizeof(float));
        }

        silk_lsf2lpc(nlsf, lpc, order);
        memcpy(s->silk.prevframe[channel].nlsf, nlsf, 16*sizeof(int16_t));
        memcpy(s->silk.prevframe[channel].lpc, lpc, 16*sizeof(float));
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
    uint32_t seed;
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
        if (seed & 0x80000000)
            excitation[i] *= -1;
        seed += value;

        excitationf[i] = (float)excitation[i] / 8388608.0f;
    }
}

static inline void silk_decode_frame(OpusContext *s, int frame, int channel, int active)
{
    int i, idx;

    /* per frame */
    int voiced;       // combines with active to indicate inactive, active, or active+voiced
    int qoffset_high;
    int order;                             // order of the LPC coefficients
    float lpc_leadin[16], lpc_body[16];
    int has_lpc_leadin;
    float ltpscale;
    float excitation[320];
    float output[SILK_MAX_LPC+320];

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
    else ltpscale = 15565.0f/16384.0f;

    /* generate the excitation signal for the entire frame */
    silk_decode_excitation(s, excitation, qoffset_high, active, voiced);

    memcpy(output, prevframe->lpc_history, SILK_MAX_LPC * sizeof(float));

    /* generate the output signal */
    for (i = 0, idx = 0; i < s->silk.subframes; i++, idx += s->silk.sflength) {
        int j, k;
        float residual[SILK_HISTORY+320];
        const float * lpc = (i < 2 && has_lpc_leadin) ? lpc_leadin : lpc_body;
        float * resptr = (voiced) ? residual + SILK_HISTORY : excitation;
        float * outptr = output + SILK_MAX_LPC;

        if (voiced) {
            int hist_end = (i>=2 && has_lpc_leadin) ? 2*s->silk.sflength : 0;

            /* when the LPC coefficients change, a re-whitening filter is used */
            /* to produce a residual that accounts for the change */

            for (j = idx - sf[i].pitchlag - order - 2; j < hist_end; j++) {
                resptr[j] = prevframe->output[s->silk.flength+j];
                for (k = 1; k <= order; k++)
                    resptr[j] -= lpc[k-1] * prevframe->output[s->silk.flength+j-k];
                resptr[j] = av_clipf(resptr[j], -1.0, 1.0) * ((i<2)?ltpscale:1.0f) / sf[i].gain;
            }

            for (j = hist_end; j < idx; j++) {
                resptr[j] = outptr[j];
                for (k = 1; k <= order; k++)
                    resptr[j] -= lpc[k-1] * outptr[j-k];
                resptr[j] = resptr[j] / sf[i].gain;
            }

            /* LTP synthesis */
            for (j = idx; j < idx + s->silk.sflength; j++) {
                resptr[j] = excitation[j];
                for (k = 0; k < 5; k++)
                    resptr[j] += sf[i].ltptaps[k] * resptr[j - sf[i].pitchlag + 2 - k];
            }
        }

        /* LPC synthesis */
        for (j = idx; j < idx + s->silk.sflength; j++) {
            outptr[j] = resptr[j] * sf[i].gain;
            for (k = 1; k <= order; k++)
                outptr[j] += lpc[k-1] * outptr[j-k];
        }
    }

    memcpy(prevframe->lpc_history, output + s->silk.flength, SILK_MAX_LPC * sizeof(float));
    for (i = 0; i < s->silk.flength; i++)
        prevframe->output[i] = av_clipf(output[i + SILK_MAX_LPC], -1.0, 1.0);
}

/**
 * CELT decoder
 */

static inline int16_t celt_cos(int16_t x)
{
    x = (MUL16(x, x) + 4096) >> 13;
    x = (32767-x) + ROUND_MUL16(x, (-7651 + ROUND_MUL16(x, (8277 + ROUND_MUL16(-626, x)))));
    return 1+x;
}

static inline int celt_log2tan(int isin, int icos)
{
    int lc, ls;
    lc = ilog(icos);
    ls = ilog(isin);
    icos <<= 15 - lc;
    isin <<= 15 - ls;
    return ((ls-lc) << 11)
           + ROUND_MUL16(isin, ROUND_MUL16(isin, -2597) + 7932)
           - ROUND_MUL16(icos, ROUND_MUL16(icos, -2597) + 7932);
}

static inline float celt_exp2(float x)
{
    int ipart = x;
    float fpart;
    if (ipart > 14)
        return 0x7f000000;
    if (ipart < -15)
        return 0;
    fpart = x - ipart;
    fpart = 16383 + fpart * (22804 + fpart * (14819 + fpart * 10204));
    return fpart;
}

static inline void celt_decode_coarse_energy(OpusContext *s, int startband, int endband)
{
    int i, j;
    float prev[2] = {0};
    float alpha, beta;
    const uint8_t *model;

    /* use the 2D z-transform to apply prediction in both */
    /* the time domain (alpha) and the frequency domain (beta) */

    if (opus_rc_tell(&s->rc)+3 <= s->celt.framebits && opus_rc_p2model(&s->rc, 3)) {
        /* intra frame */
        alpha = 0;
        beta  = 1.0f - 4915.0f/32768.0f;
        model = celt_coarse_energy_dist[s->celt.duration][1];
        av_dlog(NULL, "intra: %d\n", 1);
    } else {
        alpha = celt_alpha_coef[s->celt.duration];
        beta  = 1.0f - celt_beta_coef[s->celt.duration];
        model = celt_coarse_energy_dist[s->celt.duration][0];
        av_dlog(NULL, "intra: %d\n", 0);
    }

    av_dlog(NULL, "alpha: %f, beta: %f, LM: %d\n", alpha, 1.0f - beta, s->celt.duration);
    getchar();

    for (i = startband; i < endband; i++) {
        for (j = 0; j <= s->packet.stereo; j++) {
            float value;
            int available = s->celt.framebits - opus_rc_tell(&s->rc);

            if (available >= 15) {
                /* decode using a Laplace distribution */
                int k = FFMIN(i, 20) << 1;
                value = opus_rc_laplace(&s->rc, model[k] << 7, model[k+1] << 6);
            } else if (available >= 2) {
                int x = opus_rc_getsymbol(&s->rc, celt_model_energy_small);
                value = (x>>1) ^ -(x&1);
            } else if (available >= 1) {
                value = -opus_rc_p2model(&s->rc, 1);
            } else value = -1;

            s->celt.prevframe[j].band_energy[i] =
                FFMAX(-9.0f, s->celt.prevframe[j].band_energy[i]) * alpha + prev[j] + value;
            prev[j] += beta * value;
            av_dlog(NULL, "coarse: s->celt.prevframe[%d].band_energy[%d] = %f\n", j, i, s->celt.prevframe[j].band_energy[i]);
        }
    }
    getchar();
}

static inline void celt_decode_fine_energy(OpusContext *s, const int fine_bits[CELT_MAX_BANDS],
                                           int startband, int endband)
{
    int i;
    for (i = startband; i < endband; i++) {
        int j;
        if (fine_bits[i] <= 0) // TODO: Can this really be less than 0?
            continue;

        for (j = 0; j <= s->packet.stereo; j++) {
            int q2;
            float offset;
            q2 = opus_getrawbits(&s->rc, fine_bits[i]);
            offset = ((float)q2 + 0.5f) * (1<<(14-fine_bits[i])) / 16384.0f - 0.5f;
            s->celt.prevframe[j].band_energy[i] += offset;
            av_dlog(NULL, "fine: bandenergy[%d][%d] = %f\n", j, i, s->celt.prevframe[j].band_energy[i]);
        }
    }
    getchar();
}

static inline void celt_decode_final_energy(OpusContext *s, int startband, int endband,
                                            const int fine_bits[CELT_MAX_BANDS],
                                            const int fine_priority[CELT_MAX_BANDS],
                                            int bits_left)
{
    int priority, i, j;

    av_dlog(NULL, "unquant_energy_finalise bits_left: %d\n", bits_left);
    for (priority = 0; priority < 2; priority++) {
        for (i = startband; i < endband && bits_left > s->packet.stereo; i++) {
            if (fine_priority[i] != priority || fine_bits[i] >= CELT_MAX_FINE_BITS)
                continue;

            for (j = 0; j <= s->packet.stereo; j++) {
                int q2;
                float offset;
                q2 = opus_getrawbits(&s->rc, 1);
                offset = ((float)q2 - 0.5f) * (1<<(14-fine_bits[i]-1)) / 16384.0f;
                s->celt.prevframe[j].band_energy[i] += offset;
                av_dlog(NULL, "final: bandenergy[%d][%d] = %f\n", j, i, s->celt.prevframe[j].band_energy[i]);
                bits_left--;
            }
        }
    }
    getchar();
}

static inline void celt_decode_tf_changes(OpusContext *s, int transient,
                                          int tf_change[CELT_MAX_BANDS],
                                          int startband, int endband)
{
    int i, diff = 0, tf_select = 0, tf_changed = 0, tf_select_bit;
    int consumed, bits = transient ? 2 : 4;

    consumed = opus_rc_tell(&s->rc);
    tf_select_bit = (s->celt.duration != 0 && consumed+bits+1 <= s->celt.framebits);

    for (i = startband; i < endband; i++) {
        if (consumed+bits+tf_select_bit <= s->celt.framebits) {
            diff ^= opus_rc_p2model(&s->rc, bits);
            consumed = opus_rc_tell(&s->rc);
            tf_changed |= diff;
        }
        tf_change[i] = diff;
        bits = transient ? 4 : 5;
    }

    if (tf_select_bit && celt_tf_select[s->celt.duration][transient][0][tf_changed] !=
                         celt_tf_select[s->celt.duration][transient][1][tf_changed])
        tf_select = opus_rc_p2model(&s->rc, 1);

    for (i = startband; i < endband; i++) {
        tf_change[i] = celt_tf_select[s->celt.duration][transient][tf_select][tf_change[i]];
        av_dlog(NULL, "tf_change[%d] = %d\n", i, tf_change[i]);
    }
    getchar();
}

static inline int celt_bits2pulses(const OpusContext *s, const uint8_t * cache, int bits)
{
    // TODO: Find the size of cache and make it into an array in the parameters list
    int i, plow = 0, phigh;

    phigh = cache[0];
    bits--;

    for (i = 0; i < 6; i++) {
        int pcenter = (plow+phigh+1)>>1;
        if (cache[pcenter] >= bits)
            phigh = pcenter;
        else
            plow = pcenter;
    }

    return (bits - (plow == 0 ? -1 : cache[plow]) <= cache[phigh] - bits) ? plow : phigh;
}

static inline int celt_pulses2bits(const OpusContext *s, const uint8_t * cache, int pulses)
{
    // TODO: Find the size of cache and make it into an array in the parameters list
   return (pulses == 0) ? 0 : cache[pulses]+1;
}

static inline void celt_normalize_residual(const int * restrict iy, float * restrict X,
                                           int N, float Ryy, float gain)
{
    int i;
    float g = gain / sqrt(Ryy);
    for (i = 0; i < N; i++)
        X[i] = g * iy[i];
    for (i = 0; i < N; i++)
        av_dlog(NULL, "X[%d] = %f\n", i, X[i]);
}

static void celt_exp_rotation1(float *X, int len, int stride, float c, float s)
{
    int i;
    float *Xptr;

    Xptr = X;
    for (i = 0; i < len-stride; i++) {
        float x1, x2;
        x1 = Xptr[0];
        x2 = Xptr[stride];
        Xptr[stride] = c * x2 + s * x1;
        *Xptr++      = c * x1 - s * x2;
    }

    Xptr = &X[len-2*stride-1];
    for (i = len-2*stride-1; i >= 0; i--) {
        float x1, x2;
        x1 = Xptr[0];
        x2 = Xptr[stride];
        Xptr[stride] = c * x2 + s * x1;
        *Xptr--      = c * x1 - s * x2;
    }
}

static inline void celt_exp_rotation(float *X, int len, int stride, int K, int spread)
{
    int i;
    float c, s;
    float gain, theta;
    int stride2 = 0;

    av_dlog(NULL, "len = %d, stride = %d, K = %d, spread = %d\n", len, stride, K, spread);

    if (2*K >= len || spread == CELT_SPREAD_NONE)
        return;

    gain = (float)len / (len + (20 - 5*spread) * K);
    theta = gain * gain / 2;

    c = cos(.5f*M_PI*theta);
    s = cos(.5f*M_PI*(1.0f - theta)); /* sin(theta) */

    if (len >= stride<<3) {
        stride2 = 1;
        /* This is just a simple (equivalent) way of computing sqrt(len/stride) with rounding.
        It's basically incrementing long as (stride2+0.5)^2 < len/stride. */
        while ((stride2*stride2+stride2)*stride + (stride>>2) < len)
            stride2++;
    }

    /*NOTE: As a minor optimization, we could be passing around log2(B), not B, for both this and for
    extract_collapse_mask().*/
    len /= stride;
    av_dlog(NULL, "len/stride = %d, gain = %f, theta = %f, c = %f, s = %f, stride2 = %d\n", len, gain, theta, c, s, stride2);
    for (i = 0; i < stride; i++) {
        if (stride2)
            celt_exp_rotation1(X + i*len, len, stride2, s, c);
        celt_exp_rotation1(X + i*len, len, 1, c, s);
    }
}

static inline unsigned int celt_extract_collapse_mask(const int *iy, int N, int B)
{
    unsigned int collapse_mask;
    int N0;
    int i, j;
    av_dlog(NULL, "celt_extract_collapse_mask: N = %d, B = %d\n", N, B);
    if (B <= 1) {
        av_dlog(NULL, "collapse_mask = %d\n", 1);
        return 1;
    }
    /*NOTE: As a minor optimization, we could be passing around log2(B), not B, for both this and for
    exp_rotation().*/
    N0 = N/B;
    collapse_mask = 0;
    for (i = 0; i < B; i++)
        for (j = 0; j < N0; j++)
            collapse_mask |= (iy[i*N0+j]!=0)<<i;
    av_dlog(NULL, "collapse_mask = %d\n", collapse_mask);
    return collapse_mask;
}

static inline void celt_renormalize_vector(float *X, int N, float gain)
{
    int i;
    float g = 1e-15f;
    for (i = 0; i < N; i++)
        g += X[i] * X[i];
    g = gain / sqrt(g);

    for (i = 0; i < N; i++)
        X[i] *= g;
}

static inline void celt_stereo_merge(float *X, float *Y, float mid, int N)
{
    int i;
    float xp = 0, side = 0;
    float E[2];
    float mid2;
    float t, gain[2];

    /* Compute the norm of X+Y and X-Y as |X|^2 + |Y|^2 +/- sum(xy) */
    for (i = 0; i < N; i++) {
        xp += X[i] * Y[i];
        side += Y[i] * Y[i];
    }

    /* Compensating for the mid normalization */
    xp *= mid;
    mid2 = mid;
    E[0] = mid2*mid2 + side - 2*xp;
    E[1] = mid2*mid2 + side + 2*xp;
    if (E[0] < 6e-4f || E[1] < 6e-4f) {
        for (i = 0; i < N; i++)
            Y[i] = X[i];
        return;
    }

    t = E[0];
    gain[0] = 1.0f/sqrt(t);
    t = E[1];
    gain[1] = 1.0f/sqrt(t);

    for (i = 0; i < N; i++) {
        float value[2];
        /* Apply mid scaling (side is already scaled) */
        value[0] = mid * X[i];
        value[1] = Y[i];
        X[i] = gain[0] * (value[0] - value[1]);
        Y[i] = gain[1] * (value[0] + value[1]);
    }
}

static void celt_interleave_hadamard(float *X, int N0, int stride, int hadamard, int interleave)
{
    // TODO: Study N and B to find out the maximum size to allocate, and do so statically
    // TODO: Also, combine deinterleave and interleave together into the same function
    int i, j;
    int N = N0*stride;
    float *tmp = av_malloc(N * sizeof(float));

    av_dlog(NULL, "celt_interleave_hadamard: N0 = %d, stride = %d, hadamard = %d, interleave = %d\n", N0, stride, hadamard, interleave);

    if (hadamard) {
        const uint8_t *ordery = celt_hadamard_ordery + stride - 2;
        for (i = 0; i < stride; i++)
            for (j = 0; j < N0; j++)
                if (interleave)
                    tmp[j*stride+i] = X[ordery[i]*N0+j];
                else
                    tmp[ordery[i]*N0+j] = X[j*stride+i];
    } else {
        for (i = 0; i < stride; i++)
            for (j = 0; j < N0; j++)
                if (interleave)
                    tmp[j*stride+i] = X[i*N0+j];
                else
                    tmp[i*N0+j] = X[j*stride+i];
    }

    for (i = 0; i < N; i++)
        X[i] = tmp[i];
    for (i = 0; i < N; i++)
        av_dlog(NULL, "X[%d] = %f\n", i, X[i]);
    av_free(tmp);
}

static void celt_haar1(float *X, int N0, int stride)
{
    int i, j;
    N0 >>= 1;
    av_dlog(NULL, "celt_haar1: N0 = %d, stride = %d\n", N0, stride);
    for (i = 0; i < stride; i++) {
        for (j = 0; j < N0; j++) {
            X[stride*(2*j+0)+i] = (X[stride*(2*j+0)+i] + X[stride*(2*j+1)+i]) * M_SQRT1_2;
            X[stride*(2*j+1)+i] = (X[stride*(2*j+0)+i] - X[stride*(2*j+1)+i]) * M_SQRT1_2;
            av_dlog(NULL, "X[%d] = %f, X[%d] = %f\n", stride*(2*j+0)+i, X[stride*(2*j+0)+i], stride*(2*j+1)+i, X[stride*(2*j+1)+i]);
        }
    }
}

static inline int celt_compute_qn(int N, int b, int offset, int pulse_cap, int dualstereo)
{
    int qn, qb;
    int N2 = 2*N-1;
    if (dualstereo && N == 2)
        N2--;

    /* The upper limit ensures that in a stereo split with itheta==16384, we'll
    always have enough bits left over to code at least one pulse in the
    side; otherwise it would collapse, since it doesn't get folded. */
    qb = FFMIN3(b-pulse_cap-(4<<3), (b+N2*offset)/N2, 8<<3);
    qn = (qb<(1<<3>>1)) ? 1 : ((celt_qn_exp2[qb&0x7]>>(14-(qb>>3)))+1)>>1<<1;
    return qn;
}

static void celt_useek(unsigned int *ui, unsigned int len, unsigned int ui0, int next)
{
    unsigned int j, ui1;
    av_dlog(NULL, "celt_useek: len = %d, ui0 = %d, next = %d\n", len, ui0, next);
    for (j = 0; next && j < len; j++)
        av_dlog(NULL, "u[%d] = %d\n", j, ui[j]);
    for (j = 1; j < len; j++) {
        ui1 = (next) ? (ui[j] + ui[j-1] + ui0) : (ui[j] - ui[j-1] - ui0);
        ui[j-1] = ui0;
        ui0 = ui1;
    }
    ui[j-1] = ui0;
    for (j = 0; next && j < len; j++)
        av_dlog(NULL, "u[%d] = %d\n", j, ui[j]);
}

static inline unsigned int celt_ncwrs_urow(unsigned int n, unsigned int len, unsigned int *u)
{
    // TODO: Switch to large footprint logic?
    unsigned int k, um2;
    u[0] = 0;
    u[1] = um2 = 1;
    /*If _n==0, _u[0] should be 1 and the rest should be 0.*/
    /*If _n==1, _u[i] should be 1 for i>1.*/
    /*If _k==0, the following do-while loop will overflow the buffer.*/
    for (k = 2; k < len+2; k++)
        u[k] = (k<<1)-1;
    for (k = 2; k < n; k++)
        celt_useek(u+1, len+1, 1, 1);
    av_dlog(NULL, "celt_ncwrs_urow with n = %d, len = %d returned %d\n", n, len, u[len] + u[len+1]);
    return u[len] + u[len+1];
}

static inline void celt_cwrsi(int n, int k, unsigned int i, int *y, unsigned int *u)
{
    int j;
    av_dlog(NULL, "celt_cwrsi: n = %d, k = %d, i = %d\n", n, k, i);
    for (j = 0; j < n; j++) {
        unsigned int p = u[k+1];
        int s, yj;
        s = -(i >= p);
        i -= p & s;
        yj = k;
        p = u[k];
        while(p > i)
            p = u[--k];
        i -= p;
        yj -= k;
        y[j] = (yj+s) ^ s;
        celt_useek(u, k+2, 0, 0);
    }
    av_dlog(NULL, "celt_cwrsi end\n");
}

static inline void celt_decode_pulses(int *y, int n, int k, OpusContext *s)
{
    // TODO: provide n=2,3,4 special cases?
    // TODO: determine the maximum value of _n and preallocate statically
    unsigned int * u = av_malloc((k+2) * sizeof(unsigned int));
    av_dlog(NULL, "celt_decode_pulses: n = %d, k = %d\n", n, k);
    celt_cwrsi(n, k, opus_rc_unimodel(&s->rc, celt_ncwrs_urow(n, k, u)), y, u);
    av_dlog(NULL, "celt_decode_pulses end\n");
    av_free(u);
}

/** Decode pulse vector and combine the result with the pitch vector to produce
    the final normalised signal in the current band. */
static inline unsigned int celt_alg_unquant(OpusContext *s, float *X, int N, int K,
                                            int spread, int B, float gain)
{
    // TODO: Determine the maximum value of N and preallocate statically
    int i;
    float Ryy = 0.0f;
    unsigned int collapse_mask;
    int * y = av_malloc(N * sizeof(int));
    av_dlog(NULL, "celt_alg_unquant: N = %d, K = %d\n", N, K);
    celt_decode_pulses(y, N, K, s);
    av_dlog(NULL, "reached 1\n");
    for (i = 0; i < N; i++)
        Ryy += y[i] * y[i];
    av_dlog(NULL, "Ryy = %f\n", Ryy);
    celt_normalize_residual(y, X, N, Ryy, gain);
    celt_exp_rotation(X, N, B, K, spread);
    collapse_mask = celt_extract_collapse_mask(y, N, B);
    av_free(y);
    av_dlog(NULL, "celt_alg_unquant end\n");
    return collapse_mask;
}

static unsigned int celt_decode_band(OpusContext *s, int band, float *X, float *Y,
    int N, int b, int spread, int B, int intensity, int tf_change, float *lowband,
    int *remaining, int duration, float *lowband_out, int level,
    uint32_t *seed, float gain, float *lowband_scratch, int fill)
{
    const uint8_t *cache;
    int q;
    int curr_bits;
    int dualstereo, split;
    int imid = 0, iside = 0;
    int N0 = N;
    int N_B;
    int N_B0;
    int B0 = B;
    int time_divide = 0;
    int recombine = 0;
    int inv = 0;
    float mid = 0, side = 0;
    int longblocks = (B0 == 1);
    unsigned int cm = 0;

    N_B0 = N_B = N / B;
    split = dualstereo = (Y != NULL);

    av_dlog(NULL, "celt_decode_band: band=%d, N=%d, b=%d, spread=%d, B=%d, intensity=%d,\n"
                  "dualstereo=%d, tf_change=%d, duration=%d, level=%d, seed=%u, gain=%f, fill=%d\n",
                  band, N, b, spread, B, intensity, dualstereo, tf_change, duration, level, *seed, gain, fill);
    /*getchar();*/

    if (N == 1) {
        /* special case for one sample */
        int i;
        float *x = X;
        av_dlog(NULL, "one sample case:\n");
        for (i = 0; i <= dualstereo; i++) {
            int sign = 0;
            if (*remaining >= 1<<3) {
                sign = opus_getrawbits(&s->rc, 1);
                *remaining -= 1<<3;
                b -= 1<<3;
            }
            x[0] = sign ? -CELT_NORM_SCALE : CELT_NORM_SCALE;
            av_dlog(NULL, "%s[0] = %f\n", i ? "Y" : "X", x[0]);
            x = Y;
        }
        if (lowband_out)
            lowband_out[0] = X[0];
        av_dlog(NULL, "cm: %d\n", 1);
        return 1;
    }

    if (!dualstereo && level == 0) {
        int k;
        if (tf_change > 0)
            recombine = tf_change;
        /* Band recombining to increase frequency resolution */

        if (lowband && (recombine || ((N_B&1) == 0 && tf_change<0) || B0>1)) {
            int j;
            for (j = 0; j < N; j++)
                lowband_scratch[j] = lowband[j];
            lowband = lowband_scratch;
        }

        av_dlog(NULL, "recombine = %d\n", recombine);
        for (k = 0; k < recombine; k++) {
            if (lowband)
                celt_haar1(lowband, N>>k, 1<<k);
            fill = celt_bit_interleave[fill&0xF] | celt_bit_interleave[fill>>4] << 2;
        }
        B >>= recombine;
        N_B <<= recombine;

        /* Increasing the time resolution */
        av_dlog(NULL, "N_B = %d, tf_change = %d\n", N_B, tf_change);
        while ((N_B&1) == 0 && tf_change < 0) {
            av_dlog(NULL, "reached 1\n");
            if (lowband)
                celt_haar1(lowband, N_B, B);
            av_dlog(NULL, "reached 2\n");
            fill |= fill<<B;
            B <<= 1;
            N_B >>= 1;
            time_divide++;
            tf_change++;
        }
        av_dlog(NULL, "reached 3\n");
        B0 = B;
        N_B0 = N_B;

        /* Reorganize the samples in time order instead of frequency order */
        av_dlog(NULL, "B0: %d, lowband: %s\n", B0, lowband ? "nonzero" : "zero");
        if (B0 > 1 && lowband)
            celt_interleave_hadamard(lowband, N_B>>recombine, B0<<recombine, longblocks, 0);
        av_dlog(NULL, "reached 4\n");
    }

    /* If we need 1.5 more bit than we can produce, split the band in two. */
    cache = celt_cache_bits + celt_cache_index[(duration+1)*CELT_MAX_BANDS+band];
    av_dlog(NULL, "reached 5\n");
    if (!dualstereo && duration >= 0 && b > cache[cache[0]]+12 && N > 2) {
        N >>= 1;
        Y = X+N;
        split = 1;
        duration -= 1;
        if (B == 1)
            fill = (fill&1) | (fill<<1);
        B = (B+1) >> 1;
    }

    av_dlog(NULL, "split: %d, fill: %d, N = %d, B = %d\n", split, fill, N, B);
    if (split) {
        int qn;
        int itheta;
        int mbits, sbits, delta;
        int qalloc;
        int pulse_cap;
        int offset;
        int orig_fill;
        int tell;

        /* Decide on the resolution to give to the split parameter theta */
        pulse_cap = celt_log_freq_range[band] + (duration<<3);
        offset = (pulse_cap>>1) - (dualstereo && N == 2 ? CELT_QTHETA_OFFSET_TWOPHASE :
                                                          CELT_QTHETA_OFFSET);
        qn = (dualstereo && band >= intensity) ? 1 :
             celt_compute_qn(N, b, offset, pulse_cap, dualstereo);
        tell = opus_rc_tell_frac(&s->rc);
        if (qn != 1) {
            /* Entropy coding of the angle. We use a uniform pdf for the
            time split, a step for stereo, and a triangular one for the rest. */
            if (dualstereo && N > 2)
                itheta = opus_rc_stepmodel(&s->rc, qn/2);
            else if (dualstereo || B0 > 1)
                itheta = opus_rc_unimodel(&s->rc, qn+1);
            else
                itheta = opus_rc_trimodel(&s->rc, qn);
            itheta = itheta * 16384 / qn;
            /* NOTE: Renormalising X and Y *may* help fixed-point a bit at very high rate.
            Let's do that at higher complexity */
        } else if (dualstereo) {
            inv = (b > 2<<3 && *remaining > 2<<3) ? opus_rc_p2model(&s->rc, 2) : 0;
            itheta = 0;
        }
        qalloc = opus_rc_tell_frac(&s->rc) - tell;
        b -= qalloc;

        orig_fill = fill;
        if (itheta == 0) {
            imid = 32767;
            iside = 0;
            fill &= (1<<B)-1;
            delta = -16384;
        } else if (itheta == 16384) {
            imid = 0;
            iside = 32767;
            fill &= ((1<<B)-1)<<B;
            delta = 16384;
        } else {
            imid = celt_cos(itheta);
            iside = celt_cos(16384-itheta);
            /* This is the mid vs side allocation that minimizes squared error
            in that band. */
            delta = ROUND_MUL16((N-1)<<7, celt_log2tan(iside, imid));
        }

        mid = (float)imid/32768.0f;
        side = (float)iside/32768.0f;

        /* This is a special case for N=2 that only works for stereo and takes
        advantage of the fact that mid and side are orthogonal to encode
        the side with just one bit. */
        if (N == 2 && dualstereo) {
            int c;
            int sign = 0;
            float tmp;
            float *x2, *y2;
            mbits = b;
            /* Only need one bit for the side */
            sbits = (itheta != 0 && itheta != 16384) ? 1<<3 : 0;
            mbits -= sbits;
            c = (itheta > 8192);
            *remaining -= qalloc+sbits;

            x2 = c ? Y : X;
            y2 = c ? X : Y;
            if (sbits)
                sign = opus_getrawbits(&s->rc, 1);
            sign = 1 - 2*sign;
            /* We use orig_fill here because we want to fold the side, but if
            itheta==16384, we'll have cleared the low bits of fill. */
            cm = celt_decode_band(s, band, x2, NULL, N, mbits, spread, B, intensity, tf_change, lowband, remaining, duration, lowband_out, level, seed, gain, lowband_scratch, orig_fill);
            /* We don't split N=2 bands, so cm is either 1 or 0 (for a fold-collapse),
            and there's no need to worry about mixing with the other channel. */
            y2[0] = -sign*x2[1];
            y2[1] = sign*x2[0];
            X[0] *= mid;
            X[1] *= mid;
            Y[0] *= side;
            Y[1] *= side;
            tmp = X[0];
            X[0] = tmp - Y[0];
            Y[0] = tmp + Y[0];
            tmp = X[1];
            X[1] = tmp - Y[1];
            Y[1] = tmp + Y[1];
        } else {
            /* "Normal" split code */
            float *next_lowband2 = NULL;
            float *next_lowband_out1 = NULL;
            int next_level = 0;
            int rebalance;

            /* Give more bits to low-energy MDCTs than they would otherwise deserve */
            if (B0 > 1 && !dualstereo && (itheta & 0x3fff)) {
                if (itheta > 8192)
                    /* Rough approximation for pre-echo masking */
                    delta -= delta >> (4 - duration);
                else
                    /* Corresponds to a forward-masking slope of 1.5 dB per 10 ms */
                    delta = FFMIN(0, delta + (N << 3 >> (5 - duration)));
            }
            mbits = av_clip((b - delta)/2, 0, b);
            sbits = b - mbits;
            *remaining -= qalloc;

            if (lowband && !dualstereo)
                next_lowband2 = lowband+N; /* >32-bit split case */

            /* Only stereo needs to pass on lowband_out. Otherwise, it's
            handled at the end */
            if (dualstereo)
                next_lowband_out1 = lowband_out;
            else
                next_level = level+1;

            rebalance = *remaining;
            av_dlog(NULL, "mbits: %d, sbits: %d, delta: %d\n", mbits, sbits, delta);
            if (mbits >= sbits) {
                /* In stereo mode, we do not apply a scaling to the mid because we need the normalized
                mid for folding later */
                cm = celt_decode_band(s, band, X, NULL, N, mbits, spread, B, intensity, tf_change,
                    lowband, remaining, duration, next_lowband_out1,
                    next_level, seed, dualstereo ? 1.0f : (gain * mid), lowband_scratch, fill);

                rebalance = mbits - (rebalance - *remaining);
                if (rebalance > 3<<3 && itheta != 0)
                    sbits += rebalance - (3<<3);

                /* For a stereo split, the high bits of fill are always zero, so no
                folding will be done to the side. */
                cm |= celt_decode_band(s, band, Y, NULL, N, sbits, spread, B, intensity, tf_change,
                    next_lowband2, remaining, duration, NULL,
                    next_level, seed, gain * side, NULL, fill>>B)
                    << ((B0>>1) & (dualstereo-1));
            } else {
                /* For a stereo split, the high bits of fill are always zero, so no
                folding will be done to the side. */
                cm = celt_decode_band(s, band, Y, NULL, N, sbits, spread, B, intensity, tf_change,
                    next_lowband2, remaining, duration, NULL,
                    next_level, seed, gain * side, NULL, fill>>B)
                    << ((B0>>1) & (dualstereo-1));

                rebalance = sbits - (rebalance-*remaining);
                if (rebalance > 3<<3 && itheta != 16384)
                    mbits += rebalance - (3<<3);

                /* In stereo mode, we do not apply a scaling to the mid because we need the normalized
                mid for folding later */
                cm |= celt_decode_band(s, band, X, NULL, N, mbits, spread, B, intensity, tf_change,
                    lowband, remaining, duration, next_lowband_out1,
                    next_level, seed, dualstereo ? 1.0f : (gain * mid), lowband_scratch, fill);
            }
        }
    } else {
        /* This is the basic no-split case */
        av_dlog(NULL, "reached 7\n");
        q = celt_bits2pulses(s, cache, b);
        curr_bits = celt_pulses2bits(s, cache, q);
        *remaining -= curr_bits;
        av_dlog(NULL, "q: %d\n", q);

        /* Ensures we can never bust the budget */
        while (*remaining < 0 && q > 0) {
            *remaining += curr_bits;
            curr_bits = celt_pulses2bits(s, cache, --q);
            *remaining -= curr_bits;
        }

        if (q != 0) {
            /* Finally do the actual quantization */
            av_dlog(NULL, "reached 8\n");
            cm = celt_alg_unquant(s, X, N, (q < 8) ? q : (8 + (q&7)) << ((q>>3)-1),
                                  spread, B, gain);
            av_dlog(NULL, "reached 9\n");
        } else {
            /* If there's no pulse, fill the band anyway */
            int j;
            unsigned int cm_mask = (1<<B) - 1;
            fill &= cm_mask;
            if (!fill) {
                for (j = 0; j < N; j++)
                    X[j] = 0.0f;
            } else {
                av_dlog(NULL, "Loading noise with N = %d\n", N);
                if (lowband == NULL) {
                    /* Noise */
                    for (j = 0; j < N; j++) {
                        *seed = 1664525 * (*seed) + 1013904223;
                        X[j] = (float) (*seed >> 20);
                    }
                    cm = cm_mask;
                } else {
                    /* Folded spectrum */
                    for (j = 0; j < N; j++) {
                        *seed = 1664525 * (*seed) + 1013904223;
                        /* About 48 dB below the "normal" folding level */
                        X[j] = lowband[j] + (((*seed)&0x8000) ? 1.0f/256 : -1.0f/256);
                    }
                    cm = fill;
                }
                celt_renormalize_vector(X, N, gain);
            }
        }
    }

    av_dlog(NULL, "level: %d, B0: %d, time_divide: %d, recombine: %d, seed: %u\n", level, B0, time_divide, recombine, *seed);

    /* This code is used by the decoder and by the resynthesis-enabled encoder */
    if (dualstereo) {
        int j;
        if (N != 2)
            celt_stereo_merge(X, Y, mid, N);
        if (inv) {
            for (j = 0; j < N; j++)
                Y[j] *= -1;
        }
        for (j = 0; j < N; j++)
            av_dlog(NULL, "X[%d] = %f\n", j, X[j]);
        for (j = 0; j < N; j++)
            av_dlog(NULL, "Y[%d] = %f\n", j, Y[j]);
    } else if (level == 0) {
        int k;

        /* Undo the sample reorganization going from time order to frequency order */
        if (B0 > 1)
            celt_interleave_hadamard(X, N_B>>recombine, B0<<recombine, longblocks, 1);

        /* Undo time-freq changes that we did earlier */
        N_B = N_B0;
        B = B0;
        for (k = 0; k < time_divide; k++) {
            B >>= 1;
            N_B <<= 1;
            cm |= cm>>B;
            celt_haar1(X, N_B, B);
        }

        for (k = 0; k < recombine; k++) {
            cm = celt_bit_deinterleave[cm];
            celt_haar1(X, N0>>k, 1<<k);
        }
        B <<= recombine;

        /* Scale output for later folding */
        if (lowband_out) {
            int j;
            float n = sqrt(N0);
            for (j = 0; j < N0; j++)
                lowband_out[j] = n * X[j];
        }
        cm &= (1<<B)-1;
    }
    av_dlog(NULL, "returned from level %d with cm: %d\n", level, cm);
    return cm;
}

static inline int celt_decode_frame(OpusContext *s, int hybrid, int startband, int endband)
{
    int i, j;

    int consumed;            // bits of entropy consumed thus far for this frame
    int silence = 0;
    int has_postfilter = 0;
    int pf_octave;
    int pf_period;
    int pf_gain;
    int pf_tapset;
    int transient = 0;
    int shortblocks;
    int tf_change[CELT_MAX_BANDS];
    int spread = CELT_SPREAD_NORMAL;
    int cap[CELT_MAX_BANDS]; // approx. maximum bit allocation for each band before boost/trim
    int dynalloc = 6;
    int totalbits;
    int boost[CELT_MAX_BANDS];
    int alloctrim = 5;
    int anticollapse_bit = 0;
    int skip_bit = 0;
    // int skip = 0;
    int intensitystereo_bit = 0;
    int intensitystereo = 0;
    int dualstereo_bit = 0;
    int dualstereo = 0;
    int threshold[CELT_MAX_BANDS];
    int trim_offset[CELT_MAX_BANDS];
    int plow, phigh, ptotal, done;
    int skip_startband = startband;
    int codedbands, remaining, bandbits;
    int bits1[CELT_MAX_BANDS];
    int bits2[CELT_MAX_BANDS];
    int pulses[CELT_MAX_BANDS];
    int extrabits = 0;
    int fine_bits[CELT_MAX_BANDS];
    int fine_priority[CELT_MAX_BANDS];
    uint32_t seed = 0;
    float X[2][CELT_HISTORY];
    float norm[2*8*100];
    int B;
    int update_lowband = 1;
    int lowband_offset = 0;
    uint8_t collapse_masks[2][CELT_MAX_BANDS] = {{0}};
    float lowband_scratch[8*22];
    int fineenergy;
    int residual;
    int anticollapse = 0;
    int finalize;

    consumed = opus_rc_tell(&s->rc);
    av_dlog(NULL, "startband = %d, endband = %d\n", startband, endband);
    av_dlog(NULL, "initial consumption: %d, total bits: %d\n", consumed, s->celt.framebits);

    /* obtain silence flag */
    if (consumed >= s->celt.framebits)
        silence = 1;
    else if (consumed == 1)
        silence = opus_rc_p2model(&s->rc, 15);

    if (silence) {
        /* ignore the rest of the bits in this frame */
        consumed = s->celt.framebits;
        s->rc.total_read_bits += s->celt.framebits - opus_rc_tell(&s->rc);
    }

    av_dlog(NULL, "silence: %d\n", silence);

    /* obtain post-filter options */
    if (startband == 0 && consumed+16 <= s->celt.framebits) {
        has_postfilter = opus_rc_p2model(&s->rc, 1);
        av_dlog(NULL, "has postfilter: %d\n", has_postfilter);
        if (has_postfilter) {
            pf_octave = opus_rc_unimodel(&s->rc, 6);
            pf_period = (16<<pf_octave) + opus_getrawbits(&s->rc, 4+pf_octave) - 1;
            pf_gain   = 0.09375f * (opus_getrawbits(&s->rc, 3) + 1);
            pf_tapset = (opus_rc_tell(&s->rc)+2 <= s->celt.framebits)
                        ? opus_rc_getsymbol(&s->rc, celt_model_tapset) : 0;
        }
        consumed = opus_rc_tell(&s->rc);
    }

    /* obtain transient flag */
    if (s->celt.duration != 0 && consumed+3 <= s->celt.framebits)
        transient = opus_rc_p2model(&s->rc, 3);
    shortblocks = transient ? 1 << s->celt.duration : 0;

    av_dlog(NULL, "transient: %d, shortblocks: %d\n", transient, shortblocks);

    celt_decode_coarse_energy(s, startband, endband);
    celt_decode_tf_changes(s, transient, tf_change, startband, endband);
    consumed = opus_rc_tell(&s->rc);

    /* obtain spread flag */
    if (consumed+4 <= s->celt.framebits)
        spread = opus_rc_getsymbol(&s->rc, celt_model_spread);
    av_dlog(NULL, "spread: %d\n", spread);

    /* generate static allocation caps */
    for (i = 0; i < CELT_MAX_BANDS; i++) {
        cap[i] = (celt_static_caps[s->celt.duration][s->packet.stereo][i] + 64)
                 * celt_freq_range[i] << s->packet.stereo << s->celt.duration >> 2;
        av_dlog(NULL, "cap[%d] = %d\n", i, cap[i]);
    }
    getchar();

    /* obtain band boost */
    totalbits = s->celt.framebits << 3; // convert to 1/8 bits
    consumed = opus_rc_tell_frac(&s->rc);
    for (i = startband; i < endband; i++) {
        int quanta, band_dynalloc;
        boost[i] = 0;
        quanta = celt_freq_range[i] << s->packet.stereo << s->celt.duration;
        quanta = FFMIN(quanta<<3, FFMAX(6<<3, quanta));
        band_dynalloc = dynalloc;
        while (consumed + (band_dynalloc<<3) < totalbits && boost[i] < cap[i]) {
            int add = opus_rc_p2model(&s->rc, band_dynalloc);
            consumed = opus_rc_tell_frac(&s->rc);
            if (!add)
                break;
            boost[i] += quanta;
            totalbits -= quanta;
            band_dynalloc = 1;
        }
        av_dlog(NULL, "boost[%d] = %d\n", i, boost[i]);
        /* dynalloc is more likely to occur if it's already been used for earlier bands */
        if (boost[i])
            dynalloc = FFMAX(2, dynalloc-1);
    }
    getchar();

    /* obtain allocation trim */
    if (consumed + (6<<3) <= totalbits)
        alloctrim = opus_rc_getsymbol(&s->rc, celt_model_alloc_trim);

    av_dlog(NULL, "alloctrim: %d\n", alloctrim);
    getchar();

    /* anti-collapse bit reservation */
    totalbits = (s->celt.framebits << 3) - opus_rc_tell_frac(&s->rc) - 1;
    if (transient && s->celt.duration >= 2 && totalbits >= ((s->celt.duration+2) << 3))
        anticollapse_bit = 1<<3;
    totalbits -= anticollapse_bit;

    av_dlog(NULL, "anticollapse_bit: %d\n", anticollapse_bit);

    /* band skip bit reservation */
    if (totalbits >= 1<<3)
        skip_bit = 1<<3;
    totalbits -= skip_bit;

    av_dlog(NULL, "skip_bit: %d\n", skip_bit);

    /* intensity/dual stereo bit reservation */
    if (s->packet.stereo) {
        intensitystereo_bit = celt_log2_frac[endband - startband];
        if (intensitystereo_bit <= totalbits) {
            totalbits -= intensitystereo_bit;
            if (totalbits >= 1<<3) {
                dualstereo_bit = 1<<3;
                totalbits -= 1<<3;
            }
        } else intensitystereo_bit = 0;
    }

    av_dlog(NULL, "intensitystereo_bit: %d, dualstereo_bit: %d\n", intensitystereo_bit, dualstereo_bit);

    for (i = startband; i < endband; i++) {
        /* PVQ minimum allocation threshold; below this value, the band is skipped */
        threshold[i] = FFMAX(3*celt_freq_range[i] << s->celt.duration << 3 >> 4,
                             (s->packet.stereo+1)<<3);

        /* trim offset */
        trim_offset[i] = celt_freq_range[i]
                         * (alloctrim-5-s->celt.duration) * (endband-i-1)
                         << (s->celt.duration+3) << s->packet.stereo >> 6;
        if (celt_freq_range[i] << s->celt.duration == 1)
            trim_offset[i] -= (s->packet.stereo+1)<<3;

        av_dlog(NULL, "threshold[%d] = %d, trim_offset[%d] = %d\n", i, threshold[i], i, trim_offset[i]);
    }

    /* bisection */
    plow = 1;
    phigh = CELT_VECTORS - 1;
    while (plow <= phigh) {
        int pcenter = (plow + phigh) >> 1;
        done = ptotal = 0;

        for (i = endband-1; i >= startband; i--) {
            int bandbits = celt_freq_range[i] * celt_static_alloc[pcenter][i]
                           << s->packet.stereo << s->celt.duration >> 2;

            if (bandbits)
                bandbits = FFMAX(0, bandbits + trim_offset[i]);
            bandbits += boost[i];

            if (bandbits >= threshold[i] || done) {
                done = 1;
                ptotal += FFMIN(bandbits, cap[i]);
            } else if (bandbits >= (s->packet.stereo+1)<<3)
                ptotal += (s->packet.stereo+1)<<3;
        }

        if (ptotal > totalbits)
            phigh = pcenter - 1;
        else
            plow = pcenter + 1;
    }

    phigh = plow--;

    for (i = startband; i < endband; i++) {
        bits1[i] = celt_freq_range[i] * celt_static_alloc[plow][i]
                   << s->packet.stereo << s->celt.duration >> 2;
        bits2[i] = phigh >= CELT_VECTORS ? cap[i] :
                   celt_freq_range[i] * celt_static_alloc[phigh][i]
                   << s->packet.stereo << s->celt.duration >> 2;

        if (bits1[i])
            bits1[i] = FFMAX(0, bits1[i] + trim_offset[i]);
        if (bits2[i])
            bits2[i] = FFMAX(0, bits2[i] + trim_offset[i]);
        if (plow)
            bits1[i] += boost[i];
        bits2[i] += boost[i];

        if (boost[i])
            skip_startband = i;
        bits2[i] = FFMAX(0, bits2[i] - bits1[i]);
        av_dlog(NULL, "bits1[%d] = %d, bits2[%d] = %d\n", i, bits1[i], i, bits2[i]);
    }
    getchar();

    /* bisection */
    plow = 0;
    phigh = 1 << CELT_ALLOC_STEPS;
    for (i = 0; i < CELT_ALLOC_STEPS; i++) {
        int pcenter = (plow + phigh) >> 1;
        done = ptotal = 0;

        for (j = endband-1; j >= startband; j--) {
            int bandbits = bits1[j] + (pcenter * bits2[j] >> CELT_ALLOC_STEPS);

            if (bandbits >= threshold[j] || done) {
                done = 1;
                ptotal += FFMIN(bandbits, cap[j]);
            } else if (bandbits >= (s->packet.stereo+1)<<3)
                ptotal += (s->packet.stereo+1)<<3;
        }
        if (ptotal > totalbits)
            phigh = pcenter;
        else
            plow = pcenter;
    }
    av_dlog(NULL, "plow = %d, phigh = %d\n", plow, phigh);

    done = ptotal = 0;
    for (i = endband-1; i >= startband; i--) {
        int bandbits = bits1[i] + (plow * bits2[i] >> CELT_ALLOC_STEPS);

        if (bandbits >= threshold[i] || done)
            done = 1;
        else
            bandbits = (bandbits >= (s->packet.stereo+1)<<3) ? (s->packet.stereo+1)<<3 : 0;
        bandbits = FFMIN(bandbits, cap[i]);
        pulses[i] = bandbits;
        ptotal += bandbits;
    }
    av_dlog(NULL, "ptotal: %d\n", ptotal);

    /* band skipping */
    for (codedbands = endband; ; codedbands--) {
        int allocation;
        j = codedbands - 1;

        if (j == skip_startband) {
            /* all remaining bands are not skipped */
            totalbits += skip_bit;
            break;
        }

        /* determine the number of bits available for coding "do not skip" markers */
        remaining   = totalbits - ptotal;
        bandbits    = remaining / (celt_freq_bands[j+1] - celt_freq_bands[startband]);
        remaining  -= bandbits  * (celt_freq_bands[j+1] - celt_freq_bands[startband]);
        allocation  = pulses[j] + bandbits * celt_freq_range[j]
                      + FFMAX(0, remaining - (celt_freq_bands[j] - celt_freq_bands[startband]));

        /* a "do not skip" marker is only coded if the allocation is
           above the chosen threshold */
        if (allocation >= FFMAX(threshold[j], (s->packet.stereo+1 + 1)<<3)) {
            if (opus_rc_p2model(&s->rc, 1))
                break;

            ptotal     += 1<<3;
            allocation -= 1<<3;
        }

        /* the band is skipped, so reclaim its bits */
        ptotal -= pulses[j];
        if (intensitystereo_bit) {
            ptotal -= intensitystereo_bit;
            intensitystereo_bit = celt_log2_frac[j-startband];
            ptotal += intensitystereo_bit;
        }
        ptotal += pulses[j] = (allocation >= (s->packet.stereo+1)<<3)
                              ? (s->packet.stereo+1)<<3 : 0;
    }
    av_dlog(NULL, "codedbands: %d\n", codedbands);

    /* obtain stereo flags */
    if (intensitystereo_bit)
        intensitystereo = startband + opus_rc_unimodel(&s->rc, codedbands+1-startband);
    if (intensitystereo <= startband)
        totalbits += dualstereo_bit; /* no intensity stereo means no dual stereo */
    else if (dualstereo_bit)
        dualstereo = opus_rc_p2model(&s->rc, 1);
    av_dlog(NULL, "intensitystereo: %d\n", intensitystereo);
    av_dlog(NULL, "dualstereo: %d\n", dualstereo);
    getchar();

    /* supply the remaining bits in this frame to lower bands */
    remaining = totalbits - ptotal;
    bandbits = remaining / (celt_freq_bands[codedbands] - celt_freq_bands[startband]);
    remaining -= bandbits * (celt_freq_bands[codedbands] - celt_freq_bands[startband]);
    for (i = startband; i < codedbands; i++) {
        int bits = FFMIN(remaining, celt_freq_range[i]);
        pulses[i] += bits + bandbits * celt_freq_range[i];
        remaining -= bits;
        av_dlog(NULL, "pulses[%d] = %d\n", i, pulses[i]);
    }
    getchar();

    for (i = startband; i < codedbands; i++) {
        int N = celt_freq_range[i] << s->celt.duration;
        int prev_extra = extrabits;
        pulses[i] += extrabits;

        if (N > 1) {
            int dof;        // degrees of freedom
            int temp;       // dof * channels * log(dof)
            int offset;     // fine energy quantization offset, i.e. extra bits assigned
                            // over the standard totalbits/dof

            extrabits = FFMAX(0, pulses[i] - cap[i]);
            pulses[i] -= extrabits;

            /* intensity stereo makes use of an extra degree of freedom */
            dof = (N<<s->packet.stereo)
                  + (s->packet.stereo && N > 2 && !dualstereo && i < intensitystereo);
            temp = dof * (celt_log_freq_range[i] + (s->celt.duration<<3));
            offset = (temp >> 1) - dof * CELT_FINE_OFFSET;
            if (N == 2) /* dof=2 is the only case that doesn't fit the model */
                offset += dof<<1;

            /* grant an additional bias for the first and second pulses */
            if (pulses[i] + offset < 2 * (dof<<3))
                offset += temp >> 2;
            else if (pulses[i] + offset < 3 * (dof<<3))
                offset += temp >> 3;

            fine_bits[i] = av_clip((pulses[i] + offset + (dof<<2)) / (dof<<3),
                                   0, FFMIN((pulses[i]>>3) >> s->packet.stereo,
                                            CELT_MAX_FINE_BITS));

            /* if fine_bits was rounded down or capped,
               give priority for the final fine energy pass */
            fine_priority[i] = (fine_bits[i] * (dof<<3) >= pulses[i] + offset);

            /* the remaining bits are assigned to PVQ */
            pulses[i] -= fine_bits[i] << s->packet.stereo << 3;
        } else {
            /* all bits go to fine energy except for the sign bit */
            extrabits = FFMAX(0, pulses[i] - (1<<s->packet.stereo<<3));
            pulses[i] -= extrabits;
            fine_bits[i] = 0;
            fine_priority[i] = 1;
        }

        /* hand back a limited number of extra fine energy bits to this band */
        if(extrabits > 0) {
            int fineextra = FFMIN(extrabits >> (s->packet.stereo+3),
                                  CELT_MAX_FINE_BITS - fine_bits[i]);
            fine_bits[i] += fineextra;

            fineextra <<= s->packet.stereo + 3;
            fine_priority[i] = (fineextra >= extrabits - prev_extra);
            extrabits -= fineextra;
        }

        av_dlog(NULL, "pulses[%d] = %d, fine_bits[%d] = %d, fine_priority[%d] = %d\n", i, pulses[i], i, fine_bits[i], i, fine_priority[i]);
    }

    /* skipped bands dedicate all of their bits for fine energy */
    for (; i < endband; i++) {
        fine_bits[i] = pulses[i] >> s->packet.stereo >> 3;
        pulses[i] = 0;
        fine_priority[i] = (fine_bits[i] < 1); // TODO: can ebits[i] really be negative?
        av_dlog(NULL, "pulses[%d] = %d, fine_bits[%d] = %d, fine_priority[%d] = %d\n", i, pulses[i], i, fine_bits[i], i, fine_priority[i]);
    }

    av_dlog(NULL, "bits: %d, remaining: %d\n", totalbits, remaining);
    getchar();

    celt_decode_fine_energy(s, fine_bits, startband, endband);

    B = shortblocks ? 1 << s->celt.duration : 1;

    av_dlog(NULL, "codedbands-1: %d\n", codedbands-1);
    totalbits = (s->celt.framebits << 3) - anticollapse_bit;

    for (i = startband; i < endband; i++) {
        int b;
        int remaining2;
        int effective_lowband = -1;
        unsigned int cm[2];
        float * norm2 = norm + 8*100;

        consumed = opus_rc_tell_frac(&s->rc);

        /* Compute how many bits we want to allocate to this band */
        av_dlog(NULL, "totalbits: %d, remaining: %d\n", totalbits, remaining);
        if (i != startband)
            remaining -= consumed;
        remaining2 = totalbits - consumed - 1;
        av_dlog(NULL, "remaining: %d, totalbits: %d, consumed: %d, remaining2: %d\n", remaining, totalbits, consumed, remaining2);
        if (i <= codedbands-1) {
            int curr_balance = remaining / FFMIN(3, codedbands-i);
            av_dlog(NULL, "curr_balance: %d, pulses[%d]: %d\n", curr_balance, i, pulses[i]);
            b = av_clip(FFMIN(remaining2+1, pulses[i]+curr_balance), 0, 16383);
        } else
            b = 0;

        if (celt_freq_bands[i] - celt_freq_range[i] >= celt_freq_bands[startband] && (update_lowband || lowband_offset == 0))
            lowband_offset = i;

        /* Get a conservative estimate of the collapse_mask's for the bands we're
        going to be folding from. */
        av_dlog(NULL, "lowband_offset: %d\n", lowband_offset);
        if (lowband_offset != 0 && (spread != CELT_SPREAD_AGGRESSIVE || B > 1 || tf_change[i] < 0)) {
            int foldstart, foldend;

            /* This ensures we never repeat spectral content within one band */
            effective_lowband = FFMAX(celt_freq_bands[startband], celt_freq_bands[lowband_offset] - celt_freq_range[i]);
            foldstart = lowband_offset;
            while(celt_freq_bands[--foldstart] > effective_lowband);
            foldend = lowband_offset-1;
            while(celt_freq_bands[++foldend] < effective_lowband + celt_freq_range[i]);
            cm[0] = cm[1] = 0;
            for (j = foldstart; j < foldend; j++) {
                cm[0] |= collapse_masks[               0][j];
                cm[1] |= collapse_masks[s->packet.stereo][j];
            }
        } else
            /* Otherwise, we'll be using the LCG to fold, so all blocks will (almost
            always) be non-zero.*/
            cm[0] = cm[1] = (1<<B)-1;

        if (dualstereo && i == intensitystereo) {
            /* Switch off dual stereo to do intensity */
            dualstereo = 0;
            for (j = celt_freq_bands[startband] << s->celt.duration; j < celt_freq_bands[i] << s->celt.duration; j++)
                norm[j] = (norm[j] + norm2[j])/2;
        }

        av_dlog(NULL, "Decoding band %d with cm[0] = %d, cm[1] = %d\n", i, cm[0], cm[1]);
        if (dualstereo) {
            cm[0] = celt_decode_band(s, i, X[0] + (celt_freq_bands[i] << s->celt.duration), NULL, celt_freq_range[i] << s->celt.duration, b/2, spread, B, intensitystereo, tf_change[i],
            effective_lowband != -1 ? norm + (effective_lowband << s->celt.duration) : NULL, &remaining2, s->celt.duration,
            norm + (celt_freq_bands[i] << s->celt.duration), 0, &seed, 1.0f, lowband_scratch, cm[0]);

            cm[1] = celt_decode_band(s, i, X[1] + (celt_freq_bands[i] << s->celt.duration), NULL, celt_freq_range[i] << s->celt.duration, b/2, spread, B, intensitystereo, tf_change[i],
            effective_lowband != -1 ? norm2 + (effective_lowband << s->celt.duration) : NULL, &remaining2, s->celt.duration,
            norm2 + (celt_freq_bands[i] << s->celt.duration), 0, &seed, 1.0f, lowband_scratch, cm[1]);
        } else {
            cm[0] = celt_decode_band(s, i, X[0] + (celt_freq_bands[i] << s->celt.duration), s->packet.stereo ? X[1] + (celt_freq_bands[i] << s->celt.duration) : NULL, celt_freq_range[i] << s->celt.duration, b, spread, B, intensitystereo, tf_change[i],
            effective_lowband != -1 ? norm + (effective_lowband << s->celt.duration) : NULL, &remaining2, s->celt.duration,
            norm + (celt_freq_bands[i] << s->celt.duration), 0, &seed, 1.0f, lowband_scratch, cm[0]|cm[1]);

            cm[1] = cm[0];
        }

        collapse_masks[               0][i] = (uint8_t) cm[0];
        collapse_masks[s->packet.stereo][i] = (uint8_t) cm[1];
        remaining += pulses[i] + consumed;

        /* Update the folding position only as long as we have 1 bit/sample depth */
        update_lowband = (b > celt_freq_range[i] << s->celt.duration << 3);
    }

    if (anticollapse_bit)
        anticollapse = opus_getrawbits(&s->rc, 1);
    av_dlog(NULL, "anticollapse: %d\n", anticollapse);

    celt_decode_final_energy(s, startband, endband, fine_bits, fine_priority,
                             s->celt.framebits - opus_rc_tell(&s->rc));

    /* anti-collapse */
    if (anticollapse) {
        int c, k;
        for (i = startband; i < endband; i++) {
            float thresh, sqrt_1;
            int depth;

            /* depth in 1/8 bits */
            depth = (1 + pulses[i]) / (celt_freq_range[i] << s->celt.duration);
            thresh = celt_exp2(-.125f * depth) / 2;
            sqrt_1 = 1.0f / sqrt(celt_freq_range[i] << s->celt.duration);

            for (c = 0; c <= s->packet.stereo; c++) {
                float *xptr;
                float prev[2];
                float Ediff, r;
                int renormalize = 0;
                xptr = X[c] + (celt_freq_bands[i] << s->celt.duration);
                av_dlog(NULL, "xptr: %I64d\n", xptr - X[0]);
                prev[0] = s->celt.prevframe[0].log_band_energy[c][i];
                prev[1] = s->celt.prevframe[1].log_band_energy[c][i];
                av_dlog(NULL, "prev1: %f, prev2: %f\n", prev[0], prev[1]);
                if (!s->packet.stereo) {
                    prev[0] = FFMAX(prev[0], s->celt.prevframe[0].log_band_energy[1][i]);
                    prev[1] = FFMAX(prev[1], s->celt.prevframe[1].log_band_energy[1][i]);
                    av_dlog(NULL, "-> prev1: %f, prev2: %f\n", prev[0], prev[1]);
                }
                Ediff = s->celt.prevframe[c].band_energy[i] - FFMIN(prev[0], prev[1]);
                getchar();
                Ediff = FFMAX(0, Ediff);

                /* r needs to be multiplied by 2 or 2*sqrt(2) depending on LM because
                short blocks don't have the same energy as long */
                r = celt_exp2(-Ediff) * 2;
                if (s->celt.duration == 3)
                    r *= M_SQRT2;
                r = FFMIN(thresh, r) * sqrt_1;
                for (k = 0; k < 1 << s->celt.duration; k++) {
                    /* Detect collapse */
                    if (!(collapse_masks[c][i] & 1 << k)) {
                        /* Fill with noise */
                        for (j = 0; j < celt_freq_range[i]; j++) {
                            seed = 1664525 * seed + 1013904223;
                            xptr[(j << s->celt.duration) + k] = (seed&0x8000) ? r : -r;
                        }
                        renormalize = 1;
                    }
                }

                /* We just added some energy, so we need to renormalize */
                if (renormalize)
                    celt_renormalize_vector(xptr, celt_freq_range[i] << s->celt.duration, 1.0f);
            }
        }
    }
    return 0;
}

/**
 * Opus stream decoder
 */

static av_cold void opus_decode_flush(AVCodecContext *ctx)
{
    int i, j;
    OpusContext *s = ctx->priv_data;

    memset(&s->silk, 0, sizeof(s->silk));
    memset(&s->celt, 0, sizeof(s->celt));

    for (i = 0; i < 2; i++)
        for (j = 0; j < 2 * CELT_MAX_BANDS; j++)
            s->celt.prevframe[i].log_band_energy[0][j] = -28.0f;
}

static av_cold int opus_decode_init(AVCodecContext *avctx)
{
    OpusContext *s = avctx->priv_data;

    av_dlog(avctx, "--> opus_decode_init <--\n");

    s->avctx = avctx;
    s->rc.gb = &s->gb;

    avctx->sample_fmt = AV_SAMPLE_FMT_FLTP;

    avcodec_get_frame_defaults(&s->frame);
    avctx->coded_frame = &s->frame;

    opus_decode_flush(avctx);

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
                active[i][j] = opus_rc_p2model(&s->rc, 1);

            redundancy[i] = opus_rc_p2model(&s->rc, 1);
            if (redundancy[i]) {
                av_log(avctx, AV_LOG_ERROR, "LBRR frames present; this is unsupported\n");
                return AVERROR_PATCHWELCOME;
            }
        }

        // TODO: Resample, memset to zero if SILK layer isn't coded
        for (i = 0; i < silkframes; i++) {
            for (j = 0; j <= s->packet.stereo; j++) {
                silk_decode_frame(s, i, j, active[j][i]);
                memcpy(s->frame.extended_data[j] + i * s->silk.flength,
                       s->silk.prevframe[j].output + i * s->silk.flength,
                       s->silk.flength * sizeof(float));
            }

            s->silk.prevframe[0].coded = 1;
            s->silk.prevframe[1].coded = s->packet.stereo;
        }
    } else {
        s->silk.prevframe[0].coded = 0;
        s->silk.prevframe[1].coded = 0;
    }

    if (s->packet.mode == OPUS_MODE_CELT || s->packet.mode == OPUS_MODE_HYBRID) {
        /* Decode a CELT frame */
        const uint8_t band_end[] = {13, 0, 17, 19, 21};

        opus_raw_init(&s->rc, s->buf + s->buf_size, s->buf_size);
        s->celt.framebytes = s->buf_size;
        s->celt.framebits  = s->buf_size << 3;
        s->celt.duration = (s->packet.frame_duration >= 240)
                           + (s->packet.frame_duration >= 480)
                           + (s->packet.frame_duration == 960);

        celt_decode_frame(s, s->packet.mode == OPUS_MODE_HYBRID,
                          (s->packet.mode == OPUS_MODE_HYBRID) ? 17 : 0,
                          band_end[s->packet.bandwidth]);

        // TODO: Resample, add instead of copy
        for (i = 0; i <= s->packet.stereo; i++)
            memcpy(s->frame.extended_data[i], s->celt.prevframe[i].output,
                   s->packet.frame_duration * sizeof(float));

        s->celt.prevframe[0].coded = 1;
        s->celt.prevframe[1].coded = s->packet.stereo;
    } else {
        s->celt.prevframe[0].coded = 0;
        s->celt.prevframe[1].coded = 0;
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