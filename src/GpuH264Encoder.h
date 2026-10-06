// SPDX-FileCopyrightText: 2026 Øystein Krog <oystein.krog@gmail.com>
//
// SPDX-License-Identifier: LGPL-2.1-only OR LGPL-3.0-only OR LicenseRef-KDE-Accepted-LGPL

#pragma once

#include <QByteArray>
#include <QSize>

struct AVCodecContext;
struct AVFrame;
struct AVPacket;

namespace KRdp
{

/**
 * H.264 with FFmpeg's h264_nvenc, fed with CUDA frames from GpuAvc444Converter, so the
 * pictures never leave the GPU. Uses the same NVENC settings as KRDP's FreeRDP build
 * (codec/h264_ffmpeg.c): constant QP, ultra low latency, no B-frames, no delay.
 *
 * AVC444 sends both pictures through one encoder: the client decodes them with one
 * decoder, as one stream.
 */
class GpuH264Encoder
{
public:
    enum class Speed {
        Default, ///< NVENC preset p4
        Fast, ///< p3
        Fastest, ///< p1
    };

    GpuH264Encoder();
    ~GpuH264Encoder();

    /// Opens a new encoder if none is open or any parameter changed. The frames must come
    /// from one CUDA frames context, which the encoder takes from the first frame.
    bool ensure(const AVFrame *frame, quint32 qp, int frameRate, Speed speed);

    /// Counts the encoders opened; each one starts with a key frame.
    quint64 generation() const;

    bool encode(AVFrame *frame, QByteArray &out);

    void close();

private:
    AVCodecContext *m_context = nullptr;
    AVPacket *m_packet = nullptr;
    QSize m_size;
    quint32 m_qp = 0;
    int m_frameRate = 0;
    Speed m_speed = Speed::Fast;
    quint64 m_generation = 0;
    qint64 m_pts = 0;
};

}
