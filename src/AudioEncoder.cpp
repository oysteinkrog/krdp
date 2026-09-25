// SPDX-FileCopyrightText: 2026 krdp contributors
//
// SPDX-License-Identifier: LGPL-2.1-only OR LGPL-3.0-only OR LicenseRef-KDE-Accepted-LGPL

#include "AudioEncoder.h"

#include "krdp_logging.h"

#include <opus/opus.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/audio_fifo.h>
#include <libavutil/channel_layout.h>
#include <libavutil/samplefmt.h>
#include <libswresample/swresample.h>
}

namespace KRdp
{

namespace
{

constexpr size_t s_opusMaxPacketBytes = 4000;

struct AacState {
    AVCodecContext *context = nullptr;
    SwrContext *resampler = nullptr;
    AVAudioFifo *fifo = nullptr;
    AVFrame *frame = nullptr;
    AVPacket *packet = nullptr;
    int channels = 0;
    int64_t pts = 0;

    uint8_t **convertBuffer = nullptr;
    int convertCapacity = 0;

    void close()
    {
        if (packet) {
            av_packet_free(&packet);
        }
        if (frame) {
            av_frame_free(&frame);
        }
        if (fifo) {
            av_audio_fifo_free(fifo);
            fifo = nullptr;
        }
        if (resampler) {
            swr_free(&resampler);
        }
        if (context) {
            avcodec_free_context(&context);
        }
        if (convertBuffer) {
            av_freep(&convertBuffer[0]);
            av_freep(&convertBuffer);
        }
        convertCapacity = 0;
        pts = 0;
    }
};

struct OpusState {
    ::OpusEncoder *encoder = nullptr;
    int channels = 0;
    int frameSamples = 0;
    std::vector<uint8_t> leftover;

    void close()
    {
        if (encoder) {
            opus_encoder_destroy(encoder);
            encoder = nullptr;
        }
        frameSamples = 0;
        leftover.clear();
    }
};

}

class AudioEncoder::Private
{
public:
    AacState aac;
    OpusState opus;
};

AudioEncoder::AudioEncoder()
    : d(std::make_unique<Private>())
{
}

AudioEncoder::~AudioEncoder()
{
    d->aac.close();
    d->opus.close();
}

static bool openAac(AacState &state, uint32_t sampleRate, int channels, int bitrate)
{
    state.close();
    state.channels = channels;

    const AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_AAC);
    if (!codec) {
        qCWarning(KRDP) << "AAC: no AAC encoder available in this FFmpeg build";
        return false;
    }

    state.context = avcodec_alloc_context3(codec);
    if (!state.context) {
        return false;
    }
    state.context->sample_rate = int(sampleRate);
    state.context->sample_fmt = AV_SAMPLE_FMT_FLTP;
    state.context->bit_rate = bitrate;
    state.context->profile = AV_PROFILE_AAC_LOW;
    av_channel_layout_default(&state.context->ch_layout, channels);
    state.context->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

    if (avcodec_open2(state.context, codec, nullptr) < 0) {
        qCWarning(KRDP) << "AAC: failed to open the encoder";
        state.close();
        return false;
    }

    AVChannelLayout inLayout;
    av_channel_layout_default(&inLayout, channels);
    if (swr_alloc_set_opts2(&state.resampler,
                            &state.context->ch_layout,
                            AV_SAMPLE_FMT_FLTP,
                            int(sampleRate),
                            &inLayout,
                            AV_SAMPLE_FMT_S16,
                            int(sampleRate),
                            0,
                            nullptr)
            < 0
        || swr_init(state.resampler) < 0) {
        qCWarning(KRDP) << "AAC: failed to set up the resampler";
        av_channel_layout_uninit(&inLayout);
        state.close();
        return false;
    }
    av_channel_layout_uninit(&inLayout);

    state.fifo = av_audio_fifo_alloc(AV_SAMPLE_FMT_FLTP, channels, state.context->frame_size);
    state.frame = av_frame_alloc();
    state.packet = av_packet_alloc();
    if (!state.fifo || !state.frame || !state.packet) {
        state.close();
        return false;
    }
    state.frame->format = AV_SAMPLE_FMT_FLTP;
    state.frame->sample_rate = int(sampleRate);
    state.frame->nb_samples = state.context->frame_size;
    av_channel_layout_copy(&state.frame->ch_layout, &state.context->ch_layout);
    if (av_frame_get_buffer(state.frame, 0) < 0) {
        qCWarning(KRDP) << "AAC: failed to allocate the encode frame";
        state.close();
        return false;
    }

    qCDebug(KRDP) << "AAC: encoder ready" << sampleRate << "Hz" << channels << "ch" << bitrate / 1000 << "kbit/s, frame" << state.context->frame_size;
    return true;
}

static int drainAacPackets(AacState &state, std::vector<QByteArray> &units)
{
    int count = 0;
    while (avcodec_receive_packet(state.context, state.packet) == 0) {
        units.emplace_back(reinterpret_cast<const char *>(state.packet->data), state.packet->size);
        av_packet_unref(state.packet);
        ++count;
    }
    return count;
}

static std::vector<QByteArray> encodeAac(AacState &state, const uint8_t *pcm, size_t bytes)
{
    std::vector<QByteArray> units;
    if (!state.context || !pcm || bytes == 0) {
        return units;
    }

    const int inSamples = int(bytes / (size_t(state.channels) * sizeof(int16_t)));
    if (inSamples <= 0) {
        return units;
    }

    const int outCapacity = int(swr_get_out_samples(state.resampler, inSamples));
    if (outCapacity <= 0) {
        return units;
    }
    if (outCapacity > state.convertCapacity) {
        if (state.convertBuffer) {
            av_freep(&state.convertBuffer[0]);
            av_freep(&state.convertBuffer);
        }
        if (av_samples_alloc_array_and_samples(&state.convertBuffer, nullptr, state.channels, outCapacity, AV_SAMPLE_FMT_FLTP, 0) < 0) {
            state.convertCapacity = 0;
            return units;
        }
        state.convertCapacity = outCapacity;
    }
    const int outSamples = swr_convert(state.resampler, state.convertBuffer, outCapacity, &pcm, inSamples);
    if (outSamples > 0) {
        av_audio_fifo_write(state.fifo, reinterpret_cast<void **>(state.convertBuffer), outSamples);
    }

    const int frameSize = state.context->frame_size;
    while (av_audio_fifo_size(state.fifo) >= frameSize) {
        if (av_frame_make_writable(state.frame) < 0) {
            break;
        }
        if (av_audio_fifo_read(state.fifo, reinterpret_cast<void **>(state.frame->data), frameSize) < frameSize) {
            break;
        }
        state.frame->pts = state.pts;
        state.pts += frameSize;

        int sent = -1;
        for (;;) {
            sent = avcodec_send_frame(state.context, state.frame);
            if (sent != AVERROR(EAGAIN) || drainAacPackets(state, units) == 0) {
                break;
            }
        }
        if (sent < 0) {
            continue;
        }
        drainAacPackets(state, units);
    }

    return units;
}

static bool openOpus(OpusState &state, uint32_t sampleRate, int channels, int bitrate)
{
    state.close();
    state.channels = channels;

    int error = OPUS_OK;
    state.encoder = opus_encoder_create(opus_int32(sampleRate), channels, OPUS_APPLICATION_AUDIO, &error);
    if (error != OPUS_OK || !state.encoder) {
        qCWarning(KRDP) << "Opus: failed to create the encoder:" << opus_strerror(error);
        state.encoder = nullptr;
        return false;
    }
    opus_encoder_ctl(state.encoder, OPUS_SET_BITRATE(bitrate));
    state.frameSamples = int(sampleRate / 50);

    qCDebug(KRDP) << "Opus: encoder ready" << sampleRate << "Hz" << channels << "ch" << bitrate / 1000 << "kbit/s, frame" << state.frameSamples;
    return true;
}

static std::vector<QByteArray> encodeOpus(OpusState &state, const uint8_t *pcm, size_t bytes)
{
    std::vector<QByteArray> units;
    if (!state.encoder || !pcm || bytes == 0) {
        return units;
    }

    state.leftover.insert(state.leftover.end(), pcm, pcm + bytes);

    const size_t frameBytes = size_t(state.frameSamples) * size_t(state.channels) * sizeof(int16_t);
    std::vector<unsigned char> packet(s_opusMaxPacketBytes);
    size_t offset = 0;
    while (state.leftover.size() - offset >= frameBytes) {
        const auto *samples = reinterpret_cast<const opus_int16 *>(state.leftover.data() + offset);
        const opus_int32 length = opus_encode(state.encoder, samples, state.frameSamples, packet.data(), opus_int32(packet.size()));
        if (length > 0) {
            units.emplace_back(reinterpret_cast<const char *>(packet.data()), length);
        } else if (length < 0) {
            qCWarning(KRDP) << "Opus: encode failed:" << opus_strerror(length);
        }
        offset += frameBytes;
    }
    if (offset > 0) {
        state.leftover.erase(state.leftover.begin(), state.leftover.begin() + offset);
    }

    return units;
}

bool AudioEncoder::open(Codec codec, uint32_t sampleRate, int channels, int bitrate)
{
    return codec == Codec::Aac ? openAac(d->aac, sampleRate, channels, bitrate) : openOpus(d->opus, sampleRate, channels, bitrate);
}

bool AudioEncoder::isOpen(Codec codec) const
{
    return codec == Codec::Aac ? d->aac.context != nullptr : d->opus.encoder != nullptr;
}

int AudioEncoder::frameSamples(Codec codec) const
{
    return codec == Codec::Aac ? (d->aac.context ? d->aac.context->frame_size : 0) : d->opus.frameSamples;
}

std::vector<QByteArray> AudioEncoder::encode(Codec codec, const uint8_t *pcm, size_t bytes)
{
    return codec == Codec::Aac ? encodeAac(d->aac, pcm, bytes) : encodeOpus(d->opus, pcm, bytes);
}

}
