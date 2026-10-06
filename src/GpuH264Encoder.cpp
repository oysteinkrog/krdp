// SPDX-FileCopyrightText: 2026 Øystein Krog <oystein.krog@gmail.com>
//
// SPDX-License-Identifier: LGPL-2.1-only OR LGPL-3.0-only OR LicenseRef-KDE-Accepted-LGPL

#include "GpuH264Encoder.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
#include <libavutil/opt.h>
}

#include "krdp_logging.h"

namespace KRdp
{

GpuH264Encoder::GpuH264Encoder() = default;

GpuH264Encoder::~GpuH264Encoder()
{
    close();
}

void GpuH264Encoder::close()
{
    avcodec_free_context(&m_context);
    av_packet_free(&m_packet);
    m_size = QSize();
}

void GpuH264Encoder::requestKeyFrame()
{
    if (m_context) {
        m_keyFrameRequested = true;
        ++m_generation;
    }
}

quint64 GpuH264Encoder::generation() const
{
    return m_generation;
}

bool GpuH264Encoder::ensure(const AVFrame *frame, quint32 qp, int frameRate, Speed speed)
{
    if (!frame || !frame->hw_frames_ctx) {
        return false;
    }
    const QSize size(frame->width, frame->height);
    if (m_context && m_size == size && m_qp == qp && m_frameRate == frameRate && m_speed == speed) {
        return true;
    }
    close();

    const AVCodec *codec = avcodec_find_encoder_by_name("h264_nvenc");
    if (!codec) {
        qCWarning(KRDP) << "GPU encode: FFmpeg has no h264_nvenc";
        return false;
    }
    m_context = avcodec_alloc_context3(codec);
    m_packet = av_packet_alloc();
    if (!m_context || !m_packet) {
        close();
        return false;
    }
    m_context->width = size.width();
    m_context->height = size.height();
    m_context->time_base = AVRational{1, frameRate};
    m_context->framerate = AVRational{frameRate, 1};
    m_context->pix_fmt = AV_PIX_FMT_CUDA;
    m_context->hw_frames_ctx = av_buffer_ref(frame->hw_frames_ctx);
    m_context->max_b_frames = 0;
    // FFmpeg leaves the GOP to the preset (g = -1), and with tune ull NVENC never inserts a
    // periodic key frame. That is what we want: every reset of the client's graphics opens a
    // new encoder, which starts with one. tools/gpuavc444test checks it.
    m_context->delay = 0;
    m_context->flags |= AV_CODEC_FLAG_LOOP_FILTER;
    const char *preset = speed == Speed::Fastest ? "p1" : speed == Speed::Fast ? "p3" : "p4";
    av_opt_set(m_context->priv_data, "preset", preset, 0);
    av_opt_set(m_context->priv_data, "tune", "ull", 0);
    av_opt_set_int(m_context->priv_data, "zerolatency", 1, 0);
    av_opt_set_int(m_context->priv_data, "delay", 0, 0);
    av_opt_set_int(m_context->priv_data, "qp", qp, 0);
    // A requested key frame is an IDR picture, which also repeats SPS and PPS.
    av_opt_set_int(m_context->priv_data, "forced-idr", 1, 0);

    if (const int rc = avcodec_open2(m_context, codec, nullptr); rc < 0) {
        char error[AV_ERROR_MAX_STRING_SIZE] = {};
        av_strerror(rc, error, sizeof(error));
        qCWarning(KRDP) << "GPU encode: cannot open h264_nvenc:" << error;
        close();
        return false;
    }
    m_size = size;
    m_qp = qp;
    m_frameRate = frameRate;
    m_speed = speed;
    m_pts = 0;
    m_keyFrameRequested = false;
    ++m_generation;
    qCDebug(KRDP) << "GPU encode: h264_nvenc" << size << "QP" << qp << "preset" << preset;
    return true;
}

bool GpuH264Encoder::encode(AVFrame *frame, QByteArray &out)
{
    if (!m_context) {
        return false;
    }
    frame->pts = m_pts++;
    const bool keyFrame = m_keyFrameRequested;
    frame->pict_type = keyFrame ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_NONE;
    const int sent = avcodec_send_frame(m_context, frame);
    frame->pict_type = AV_PICTURE_TYPE_NONE;
    if (sent < 0) {
        qCWarning(KRDP) << "GPU encode: avcodec_send_frame failed";
        return false;
    }
    m_keyFrameRequested = false;
    out.clear();
    // With zero latency and no B-frames, NVENC returns the packet for this frame right away.
    while (true) {
        const int rc = avcodec_receive_packet(m_context, m_packet);
        if (rc == AVERROR(EAGAIN) || rc == AVERROR_EOF) {
            break;
        }
        if (rc < 0) {
            qCWarning(KRDP) << "GPU encode: avcodec_receive_packet failed";
            return false;
        }
        if ((m_packet->flags & AV_PKT_FLAG_KEY) && m_pts > 1) {
            qCDebug(KRDP) << "GPU encode:" << (keyFrame ? "requested" : "unexpected") << "key frame at picture" << m_pts - 1 << "bytes" << m_packet->size;
        }
        out.append(reinterpret_cast<const char *>(m_packet->data), m_packet->size);
        av_packet_unref(m_packet);
    }
    if (out.isEmpty()) {
        qCWarning(KRDP) << "GPU encode: NVENC returned no packet";
        return false;
    }
    return true;
}

}
