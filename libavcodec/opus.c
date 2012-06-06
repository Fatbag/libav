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

enum OpusMode {
    OPUS_MODE_SILK,
    OPUS_MODE_HYBRID,
    OPUS_MODE_CELT
};

#ifdef DEBUG
static const char *opus_mode_str[3] = {
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
static const char *opus_bandwidth_str[5] = {
    "narrowband", "medium-band", "wideband", "super-wideband", "fullband"
};
#endif

#define MAX_FRAME_SIZE 1275
#define MAX_FRAMES     48

typedef struct {
    int size;                       /** packet size */
    int code;                       /** packet code: specifies the frame layout */
    int stereo;                     /** stereo flag */
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
    OpusPacket packet;
    AVFrame frame;
    int currentframe;
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

static int opus_parse_packet(AVCodecContext *avctx, OpusPacket *pkt,
                             const uint8_t *data, int len)
{
    int frame_bytes, i;
    const uint8_t *ptr = data;
    const uint8_t *end = data + len;

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
        pkt->frame_offset[0] = ptr - data;
        pkt->frame_size[0]   = frame_bytes;
        break;
    case 1:
        /* 2 frames, equal size */
        pkt->frame_count = 2;
        pkt->vbr   = 0;
        frame_bytes = end - ptr;
        if (frame_bytes & 1 || frame_bytes / 2 > MAX_FRAME_SIZE)
            return AVERROR_INVALIDDATA;
        pkt->frame_offset[0] = ptr - data;
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
        pkt->frame_offset[0] = ptr - data;
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
            pkt->frame_offset[0] = ptr - data;
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
            pkt->frame_offset[0] = ptr - data;
            pkt->frame_size[0]   = frame_bytes;
            for (i = 1; i < pkt->frame_count; i++) {
                pkt->frame_offset[i] = pkt->frame_offset[i-1] + pkt->frame_size[i-1];
                pkt->frame_size[i]   = frame_bytes;
            }
        }
    }

    /* total packet duration cannot be larger than 120ms */
    pkt->frame_duration = opus_frame_duration[pkt->config];
    if (pkt->frame_duration * pkt->frame_count > 5760)
        return AVERROR_INVALIDDATA;

    /* set mode and bandwidth */
    if (pkt->config < 12) {
        pkt->mode = OPUS_MODE_SILK;
        pkt->bandwidth = pkt->config / 4;
    } else if (pkt->config < 16) {
        pkt->mode = OPUS_MODE_HYBRID;
        pkt->bandwidth = OPUS_BANDWIDTH_SUPERWIDEBAND + (pkt->config >= 14);
    }else {
        pkt->mode = OPUS_MODE_CELT;
        pkt->bandwidth = (pkt->config - 16) / 4;
        /* skip mediumband */
        if (pkt->bandwidth)
            pkt->bandwidth++;
    }

    opus_dprint_packet(avctx, pkt);
    return ptr - data;
}

static av_cold int opus_decode_init(AVCodecContext *avctx)
{
    OpusContext *s = avctx->priv_data;

    av_dlog(avctx, "--> opus_decode_init <--\n");

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
    int ret, duration;
    OpusContext *s = avctx->priv_data;
    const int buf_size = avpkt->size;

    av_dlog(avctx, "\n\n--> opus_decode_frame <--\npts: %"PRId64"\n\n", avpkt->pts);
    *got_frame_ptr = 0;

    /* if this is a new packet, parse its header */
    if (!s->packet.frame_count) {
        if ((header = opus_parse_packet(avctx, &s->packet, avpkt->data, avpkt->size)) < 0) {
            av_log(avctx, AV_LOG_ERROR, "Error parsing packet\n");
            return buf_size;
        }
        s->currentframe = 0;
    }

    duration = s->packet.frame_duration < avpkt->duration ?
               s->packet.frame_duration : avpkt->duration;
    if (duration) {
        s->frame.nb_samples = duration * avctx->sample_rate / 48000;
        if ((ret = avctx->get_buffer(avctx, &s->frame)) < 0) {
            av_log(avctx, AV_LOG_ERROR, "get_buffer() failed\n");
            return buf_size;
        }
        avpkt->duration -= duration;
    }

    *got_frame_ptr   = 1;
    *(AVFrame *)data = s->frame;
    
    if (--s->packet.frame_count && avpkt->duration) {
        /* more frames in the packet */
        avpkt->pts = avpkt->dts += s->packet.frame_duration;
        return header + s->packet.frame_size[s->currentframe++];
    }
    
    /* skip unused frames or padding at the end of the packet */
    return buf_size;
}

AVCodec ff_opus_decoder = {
    .name            = "opus",
    .type            = AVMEDIA_TYPE_AUDIO,
    .id              = CODEC_ID_OPUS,
    .priv_data_size  = sizeof(OpusContext),
    .init            = opus_decode_init,
    .close           = opus_decode_close,
    .decode          = opus_decode_frame,
    .capabilities    = CODEC_CAP_DR1,
    .long_name       = NULL_IF_CONFIG_SMALL("Opus"),
};