/*
 * Opus parser for Ogg
 * Copyright (c) 2011 Justin Ruggles
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

#include "libavutil/audioconvert.h"
#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "internal.h"
#include "oggdec.h"

struct opus_params {
    int header_count;
    int pre_skip;
};

static const uint64_t opus_channel_layouts[9] = {
    AV_CH_LAYOUT_MONO,
    AV_CH_LAYOUT_STEREO,
    AV_CH_LAYOUT_SURROUND,
    AV_CH_LAYOUT_QUAD,
    AV_CH_LAYOUT_5POINT0_BACK,
    AV_CH_LAYOUT_5POINT1_BACK,
    AV_CH_LAYOUT_5POINT1|AV_CH_BACK_CENTER,
    AV_CH_LAYOUT_7POINT1,
    0
};

static int opus_header(AVFormatContext *s, int idx) {
    struct ogg *ogg = s->priv_data;
    struct ogg_stream *os = ogg->streams + idx;
    struct opus_params *op = os->private;
    AVStream *st = s->streams[idx];
    uint8_t *p = os->buf + os->pstart;

    if (!op) {
        op = av_mallocz(sizeof(*op));
        if (!op)
            return AVERROR(ENOMEM);
        os->private = op;
    } else if (op->header_count > 1)
        return 0;

    if (op->header_count == 0) {
        // ID header
        if (os->psize < 19) {
            av_log(s, AV_LOG_ERROR, "OpusHead header packet is too small\n");
            return AVERROR_INVALIDDATA;
        }
        if (memcmp(p, "OpusHead", 8)) {
            av_log(s, AV_LOG_ERROR, "first packet must be an id header\n");
            return AVERROR_INVALIDDATA;
        }
        if (AV_RL8(p + 8) != 1) {
            av_log(s, AV_LOG_ERROR, "unrecognized OggOpus version\n");
            return AVERROR_INVALIDDATA;
        }

        st->codec->channels = AV_RL8(p + 9);
        if (st->codec->channels == 0) {
            av_log(s, AV_LOG_ERROR, "invalid number of source channels\n");
            return AVERROR_INVALIDDATA;
        }

        op->pre_skip = AV_RL16(p + 10);

        st->codec->sample_rate = AV_RL32(p + 12);
        if (st->codec->sample_rate == 0) {
            av_log(s, AV_LOG_ERROR, "invalid sample rate\n");
            return AVERROR_INVALIDDATA;
        }

        st->codec->codec_type     = AVMEDIA_TYPE_AUDIO;
        st->codec->codec_id       = CODEC_ID_OPUS;
        st->codec->channel_layout = (st->codec->channels > 8) ? 0 :
                                    opus_channel_layouts[st->codec->channels - 1];

        if (AV_RL8(p + 18) == 0 && st->codec->channels > 2)
            av_log(s, AV_LOG_WARNING,
                "RTP channel mapping is undefined for more than one stream\n");
        if (AV_RL8(p + 18) > 1)
            av_log(s, AV_LOG_WARNING, "channel mapping unrecognized\n");

        avpriv_set_pts_info(st, 64, 1, st->codec->sample_rate);
    } else  {
        // comment header
        if (os->psize < 8) {
            av_log(s, AV_LOG_ERROR, "OpusTags header packet is too small\n");
            return AVERROR_INVALIDDATA;
        }
        if (memcmp(p, "OpusTags", 8)) {
            av_log(s, AV_LOG_ERROR, "second packet must be a comment header\n");
            return AVERROR_INVALIDDATA;
        }
        if (ff_vorbis_comment(s, &st->metadata, p + 8, os->psize - 8)) {
            av_log(s, AV_LOG_ERROR, "invalid OpusTags header packet\n");
            return AVERROR_INVALIDDATA;
        }
    }

    op->header_count++;
    return 1;
}

static uint64_t opus_gptopts(AVFormatContext *s, int idx, uint64_t granule,
                             int64_t *dts_out)
{
    struct ogg *ogg = s->priv_data;
    struct ogg_stream *os = ogg->streams + idx;
    struct opus_params *op = os->private;
    AVStream *st = s->streams[idx];
    return (granule - op->pre_skip)*st->codec->sample_rate/48000;
}

const struct ogg_codec ff_opus_codec = {
    .magic     = "OpusHead",
    .magicsize = 8,
    .header    = opus_header,
    .gptopts   = opus_gptopts,
};