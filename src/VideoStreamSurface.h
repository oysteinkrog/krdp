// SPDX-FileCopyrightText: 2026 David Edmundson <kde@davidedmundson.co.uk>
// SPDX-FileCopyrightText: 2023 Arjen Hiemstra <ahiemstra@heimr.nl>
//
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <atomic>
#include <memory>
#include <mutex>

#include <DmaBufHandler>
#include <PipeWireEncodedStream>
#include <PipeWireSourceStream>
#include <freerdp/codec/h264.h>
#include <freerdp/codec/progressive.h>
#include <freerdp/server/rdpgfx.h>

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
