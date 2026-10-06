// SPDX-FileCopyrightText: 2026 David Edmundson <kde@davidedmundson.co.uk>
// SPDX-FileCopyrightText: 2023 Arjen Hiemstra <ahiemstra@heimr.nl>
//
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <atomic>
#include <memory>
#include <deque>
#include <functional>
#include <mutex>
#include <vector>

#include <DmaBufHandler>
#include <PipeWireEncodedStream>
#include <PipeWireSourceStream>
#include <freerdp/codec/h264.h>
#include <freerdp/codec/progressive.h>
#include <freerdp/server/rdpgfx.h>

#include "GpuAvc444Converter.h"
#include "VideoStream.h"

namespace KRdp
{

struct Surface {
    uint16_t id;
    uint32_t codecContextId;
    QSize size;
};

class VideoStreamSurface : public QObject
{
public:
    explicit VideoStreamSurface(VideoStream *stream);

    void setActiveEncodingMode(VideoStream::EncodingMode mode, quint8 quality, int requestedFrameRate);
    void setStreamingEnabled(bool enabled);
    void setVideoQuality(quint8 quality);
    void setRequestedSize(const QSize &size);
    void setPipeWireSource(quint32 nodeId, quint64 objectSerial, int fd);

    void queueFrame(const VideoFrame &frame);
    void onPacketReceived(const PipeWireEncodedStream::Packet &data);
    void onFrameReceived(const PipeWireFrame &data);
    bool sendFrameH264(RdpgfxServerContext *gfxContext, uint32_t frameId, const VideoFrame &frame, quint64 resetGeneration);
    bool sendFrameProgressive(RdpgfxServerContext *gfxContext, PROGRESSIVE_CONTEXT *progressive, uint32_t frameId, const VideoFrame &frame, quint64 resetGeneration);

    enum class AvcResult {
        Sent,
        Unchanged, ///< nothing changed since the last frame, nothing sent
        Failed, ///< the encoder failed
        Stale, ///< the client reset its GFX state while the frame was encoded, nothing sent
    };
    AvcResult sendFrameAvc(RdpgfxServerContext *gfxContext, H264_CONTEXT *h264, bool avc444, uint32_t frameId, const VideoFrame &frame, quint64 resetGeneration);
    /// AVC444 from a frame the GPU already converted. encode(chroma, out) encodes the luma
    /// or chroma picture of the frame. encoderGeneration changes whenever the H.264 encoder
    /// was opened again, which means both pictures must be sent in full.
    using AvcEncode = std::function<bool(bool chroma, QByteArray &out)>;
    AvcResult sendFrameAvcGpu(RdpgfxServerContext *gfxContext,
                              const AvcEncode &encode,
                              quint64 encoderGeneration,
                              quint32 qp,
                              uint32_t frameId,
                              const VideoFrame &frame,
                              quint64 resetGeneration);

    std::unique_ptr<PipeWireEncodedStream> encodedStream;
    std::unique_ptr<PipeWireSourceStream> sourceStream;
    DmaBufHandler dmaBufHandler;
    quint32 nodeId = 0;
    int pipeWireFd = -1;
    quint64 objectSerial = quint64(-1);
    Surface surface;
    QSize size;
    QSize requestedSize;
    bool pendingReset = true;
    VideoStream::EncodingMode rawMode = VideoStream::EncodingMode::H264;

    // GPU conversion for AVC444, on the main thread in onFrameReceived().
    std::unique_ptr<GpuAvc444Converter> gpuConverter;
    std::atomic_bool gpuFailed = false; // set by the frame submission thread if encoding fails
    // Changed tiles of every converted frame, by sequence number. Each frame's tiles say what
    // changed since the frame before it, so a frame that is dropped before encoding must pass
    // its tiles on: the encoder takes the union of all frames since the last one it encoded.
    std::mutex gpuChangesMutex;
    std::deque<std::pair<quint64, std::vector<uint8_t>>> gpuChanges;
    quint64 gpuSequence = 0;
    bool gpuChangesLost = false;
    // Encoder side, only touched by the frame submission thread.
    quint64 gpuEncoderGeneration = 0;
    bool gpuLumaSent = false;
    bool gpuChromaSent = false;
    /// The union of changed tiles since the last encoded frame, up to and including sequence.
    /// Returns false if some changes were lost, in which case everything must be sent.
    bool takeGpuChanges(quint64 sequence, std::vector<uint8_t> &tiles);

    // Held while a frame is encoded and sent, and while a client reset drops the surface.
    std::mutex frameMutex;
    // Counts client resets (caps re-advertisements). It goes up before the reset waits for
    // frameMutex, so a frame still being encoded sees the change and is not sent to a
    // surface the client has already dropped. That gives protocol error 0xd06 in mstsc.
    std::atomic<quint64> resetGeneration = 0;

private:
    VideoStream *const m_stream;
};
}
