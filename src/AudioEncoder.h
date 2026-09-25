// SPDX-FileCopyrightText: 2026 krdp contributors
//
// SPDX-License-Identifier: LGPL-2.1-only OR LGPL-3.0-only OR LicenseRef-KDE-Accepted-LGPL

#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <QByteArray>

namespace KRdp
{

/**
 * Encodes interleaved S16 PCM into AAC-LC (FFmpeg) or Opus (libopus) access units for
 * rdpsnd's compressed formats (WAVE_FORMAT_AAC_MS, WAVE_FORMAT_OPUS).
 */
class AudioEncoder
{
public:
    enum class Codec {
        Aac,
        Opus
    };

    AudioEncoder();
    ~AudioEncoder();
    AudioEncoder(const AudioEncoder &) = delete;
    AudioEncoder &operator=(const AudioEncoder &) = delete;

    bool open(Codec codec, uint32_t sampleRate, int channels, int bitrate);
    bool isOpen(Codec codec) const;

    int frameSamples(Codec codec) const;

    std::vector<QByteArray> encode(Codec codec, const uint8_t *pcm, size_t bytes);

private:
    class Private;
    const std::unique_ptr<Private> d;
};

}
