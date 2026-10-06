// SPDX-FileCopyrightText: 2026 David Edmundson <kde@davidedmundson.co.uk>
// SPDX-FileCopyrightText: 2023 Arjen Hiemstra <ahiemstra@heimr.nl>
//
// SPDX-License-Identifier: GPL-2.0-or-later

#include "VideoStreamSurface.h"

#include <chrono>
#include <optional>
#include <algorithm>
#include <utility>

#include <unistd.h>

#include <QDateTime>

#include "krdp_logging.h"

namespace KRdp
{

namespace clk = std::chrono;

static RECTANGLE_16 toRectangle16(const QRect &rect)
{
    RECTANGLE_16 result = {};
    result.left = rect.left();
    result.top = rect.top();
    result.right = rect.right() + 1;
    result.bottom = rect.bottom() + 1;
    return result;
}

static std::optional<REGION16> toRegion16(const QRegion &region, const QRect &frameRect)
{
    REGION16 invalidRegion = {};
    region16_init(&invalidRegion);

    const QRegion clipped = region.isEmpty() ? QRegion(frameRect) : region.intersected(frameRect);
    for (const QRect &rect : clipped) {
        if (!rect.isValid()) {
            continue;
        }

        const RECTANGLE_16 rectangle = toRectangle16(rect);
        if (!region16_union_rect(&invalidRegion, &invalidRegion, &rectangle)) {
            region16_uninit(&invalidRegion);
            return std::nullopt;
        }
    }

    if (region16_is_empty(&invalidRegion)) {
        const RECTANGLE_16 fullFrame = toRectangle16(frameRect);
        if (!region16_union_rect(&invalidRegion, &invalidRegion, &fullFrame)) {
            region16_uninit(&invalidRegion);
            return std::nullopt;
        }
    }

    return invalidRegion;
}

VideoStreamSurface::VideoStreamSurface(VideoStream *stream)
    : QObject(stream)
    , m_stream(stream)
{
}

void VideoStreamSurface::setActiveEncodingMode(VideoStream::EncodingMode mode, quint8 quality, int requestedFrameRate)
{
    if (encodedStream) {
        encodedStream->stop();
        encodedStream.reset();
    }
    if (sourceStream && mode != VideoStream::EncodingMode::H264) {
        // Every raw frame mode uses the same source stream, so keep it. Destroying it
        // drops the last reference to KPipeWire's PipeWire core, which closes the shared
        // fd, and a new stream on that fd then fails to connect. mstsc re-advertises its
        // caps on every connection, which used to do exactly that.
        sourceStream->setActive(m_stream->streamingEnabled() && nodeId != 0);
        return;
    }
    if (sourceStream) {
        sourceStream->stopStreaming();
        sourceStream.reset();
    }

    if (mode == VideoStream::EncodingMode::H264) {
        encodedStream = std::make_unique<PipeWireEncodedStream>();
        encodedStream->setEncodingPreference(PipeWireBaseEncodedStream::EncodingPreference::Speed);
        encodedStream->setColorRange(PipeWireBaseEncodedStream::ColorRange::Full);
        encodedStream->setEncoder(PipeWireEncodedStream::H264Baseline);
        encodedStream->setQuality(quality);
        encodedStream->setMaxFramerate(requestedFrameRate, 1);
        encodedStream->setMaxPendingFrames(requestedFrameRate);
        if (requestedSize.isValid()) {
            encodedStream->setRequestedSize(requestedSize);
        }

        QObject::connect(encodedStream.get(), &PipeWireEncodedStream::newPacket, this, [this](const auto &packet) {
            onPacketReceived(packet);
        });
        QObject::connect(encodedStream.get(), &PipeWireEncodedStream::sizeChanged, this, [this](const QSize &newSize) {
            m_stream->setSize(newSize);
        });
        QObject::connect(encodedStream.get(), &PipeWireEncodedStream::cursorChanged, m_stream, &VideoStream::cursorChanged);
        if (nodeId != 0) {
            encodedStream->setObjectSerial(objectSerial);
            encodedStream->setNodeId(nodeId);
            if (pipeWireFd > 0) {
                // KPipeWire closes the fd when its encoder stops; give each encoder its
                // own copy so a restarted encoder can still reach the stream.
                encodedStream->setFd(dup(pipeWireFd));
            }
        }
        if (m_stream->streamingEnabled() && nodeId != 0) {
            encodedStream->start();
        }
    } else {
        sourceStream = std::make_unique<PipeWireSourceStream>();
        sourceStream->setAllowDmaBuf(true);
        sourceStream->setDamageEnabled(true);
        sourceStream->setMaxFramerate({static_cast<quint32>(requestedFrameRate), 1});
        if (requestedSize.isValid()) {
            sourceStream->setRequestedSize(requestedSize);
        }
        QObject::connect(
            sourceStream.get(),
            &PipeWireSourceStream::frameReceived,
            this,
            [this](const auto &frame) {
                onFrameReceived(frame);
            },
            // Direct: KPipeWire hands the buffer back to KWin as soon as frameReceived
            // returns, so a queued call would read a buffer KWin may be drawing into again.
            Qt::DirectConnection);
        QObject::connect(sourceStream.get(), &PipeWireSourceStream::streamParametersChanged, this, [this]() {
            m_stream->setSize(sourceStream->size());
        });
        QObject::connect(
            sourceStream.get(),
            &PipeWireSourceStream::frameReceived,
            this,
            [this](const PipeWireFrame &frame) {
                if (frame.cursor) {
                    Q_EMIT m_stream->cursorChanged(*frame.cursor);
                }
            },
            Qt::QueuedConnection);

        if (nodeId != 0 && pipeWireFd) {
            bool created = false;
            if (objectSerial != quint64(-1)) {
                created = sourceStream->createStream(objectSerial, pipeWireFd);
            } else {
                created = sourceStream->createStream(nodeId, pipeWireFd);
            }
            if (!created) {
                qCWarning(KRDP) << "Could not create PipeWire source stream" << sourceStream->error();
                m_stream->failVideoInitialization();
                return;
            }
            size = sourceStream->size();
        }
        sourceStream->setActive(m_stream->streamingEnabled() && nodeId != 0);
    }
}

void VideoStreamSurface::queueFrame(const VideoFrame &frame)
{
    m_stream->queueFrame(frame);
}

void VideoStreamSurface::setStreamingEnabled(bool enabled)
{
    if (encodedStream) {
        if (enabled && nodeId != 0) {
            if (encodedStream->state() == PipeWireBaseEncodedStream::Paused) {
                encodedStream->resume();
            } else {
                encodedStream->start();
            }
        } else {
            encodedStream->pause();
        }
    }
    if (sourceStream) {
        sourceStream->setActive(enabled && nodeId != 0);
    }
}

void VideoStreamSurface::setVideoQuality(quint8 quality)
{
    if (encodedStream) {
        encodedStream->setQuality(quality);
    }
}

void VideoStreamSurface::setRequestedSize(const QSize &newSize)
{
    requestedSize = newSize;
    if (encodedStream) {
        encodedStream->setRequestedSize(newSize);
    }
    if (sourceStream) {
        sourceStream->setRequestedSize(newSize);
    }
}

void VideoStreamSurface::setPipeWireSource(quint32 newNodeId, quint64 newObjectSerial, int fd)
{
    nodeId = newNodeId;
    objectSerial = newObjectSerial;
    pipeWireFd = fd;

    if (encodedStream) {
        encodedStream->setObjectSerial(objectSerial);
        encodedStream->setNodeId(nodeId);
        if (pipeWireFd > 0) {
            encodedStream->setFd(dup(pipeWireFd));
        }
        if (m_stream->streamingEnabled()) {
            encodedStream->start();
        }
    }

    if (!sourceStream) {
        return;
    }

    bool created = false;
    if (objectSerial != quint64(-1)) {
        created = sourceStream->createStream(objectSerial, fd);
    } else {
        created = sourceStream->createStream(nodeId, fd);
    }

    if (!created) {
        qCWarning(KRDP) << "Could not create PipeWire source stream" << sourceStream->error();
        m_stream->failVideoInitialization();
        return;
    }

    m_stream->setSize(sourceStream->size());
    sourceStream->setActive(m_stream->streamingEnabled());
}

void VideoStreamSurface::onPacketReceived(const PipeWireEncodedStream::Packet &data)
{
    VideoFrame frameData;
    frameData.size = size;
    frameData.data = data.data();
    frameData.isKeyFrame = data.isKeyFrame();
    queueFrame(frameData);
}

void VideoStreamSurface::onFrameReceived(const PipeWireFrame &data)
{
    VideoFrame frameData;
    frameData.size = data.dataFrame ? data.dataFrame->size : QSize(data.dmabuf ? data.dmabuf->width : 0, data.dmabuf ? data.dmabuf->height : 0);
    frameData.damage = data.damage.value_or(QRegion(QRect(QPoint(0, 0), frameData.size)));
    if (data.presentationTimestamp) {
        frameData.presentationTimeStamp = clk::system_clock::time_point(clk::duration_cast<clk::microseconds>(*data.presentationTimestamp));
    }

    if (data.dataFrame) {
        frameData.image = data.dataFrame->toImage().convertToFormat(QImage::Format_RGB32);
    } else if (data.dmabuf) {
        // The encoders want BGRX (Format_RGB32), but DmaBufHandler can only read back RGBA.
        // Swapping R and B in place and relabelling is a fast SIMD pass; converting from
        // RGBA8888_Premultiplied with convertToFormat() took Qt's slow generic path.
        QImage image(frameData.size, QImage::Format_RGBX8888);
        if (!dmaBufHandler.downloadFrame(image, data)) {
            qCWarning(KRDP) << "Failed to download DMA-BUF frame";
            return;
        }
        image.rgbSwap();
        image.reinterpretAsFormat(QImage::Format_RGB32);
        frameData.image = std::move(image);
    } else {
        // KWin sends buffers without image data when only the cursor moved; the cursor
        // itself is handled by the other frameReceived connection.
        return;
    }

    queueFrame(frameData);
}

bool VideoStreamSurface::sendFrameH264(RdpgfxServerContext *gfxContext, uint32_t frameId, const VideoFrame &frame, quint64 resetGeneration)
{
    if (frame.data.isEmpty()) {
        return false;
    }

    if (surface.id == 0) {
        qCWarning(KRDP) << "No graphics surface available for H264 frame submission";
        return false;
    }

    RDPGFX_START_FRAME_PDU startFramePdu = {};
    RDPGFX_END_FRAME_PDU endFramePdu = {};

    const auto now = QDateTime::currentDateTimeUtc().time();
    startFramePdu.timestamp = now.hour() << 22 | now.minute() << 16 | now.second() << 10 | now.msec();
    startFramePdu.frameId = frameId;
    endFramePdu.frameId = frameId;

    RDPGFX_SURFACE_COMMAND surfaceCommand = {};
    surfaceCommand.surfaceId = surface.id;
    surfaceCommand.codecId = RDPGFX_CODECID_AVC420;
    surfaceCommand.contextId = 0;
    surfaceCommand.format = PIXEL_FORMAT_BGRX32;
    surfaceCommand.right = frame.size.width();
    surfaceCommand.bottom = frame.size.height();
    surfaceCommand.width = frame.size.width();
    surfaceCommand.height = frame.size.height();

    RDPGFX_AVC420_BITMAP_STREAM avcStream = {};
    surfaceCommand.extra = &avcStream;
    avcStream.data = reinterpret_cast<BYTE *>(const_cast<char *>(frame.data.data()));
    avcStream.length = frame.data.length();

    avcStream.meta.numRegionRects = 1;
    RECTANGLE_16 rect = {0, 0, static_cast<UINT16>(frame.size.width()), static_cast<UINT16>(frame.size.height())};
    avcStream.meta.regionRects = &rect;
    RDPGFX_H264_QUANT_QUALITY quality = {22, 0, 100};
    avcStream.meta.quantQualityVals = &quality;

    if (this->resetGeneration != resetGeneration) {
        qCDebug(KRDP) << "Client reset its graphics; dropping frame" << frameId;
        return false;
    }

    const UINT startStatus = gfxContext->StartFrame(gfxContext, &startFramePdu);
    if (startStatus != CHANNEL_RC_OK) {
        qCWarning(KRDP) << "StartFrame failed" << startStatus << "frameId" << frameId;
        return true;
    }

    const UINT commandStatus = gfxContext->SurfaceCommand(gfxContext, &surfaceCommand);
    if (commandStatus != CHANNEL_RC_OK) {
        qCWarning(KRDP) << "SurfaceCommand failed" << commandStatus << "frameId" << frameId << "surface" << surface.id << "encodedBytes" << frame.data.size();
    }

    const UINT endStatus = gfxContext->EndFrame(gfxContext, &endFramePdu);
    if (endStatus != CHANNEL_RC_OK) {
        qCWarning(KRDP) << "EndFrame failed" << endStatus << "frameId" << frameId;
    }

    return true;
}

VideoStreamSurface::AvcResult
VideoStreamSurface::sendFrameAvc(RdpgfxServerContext *gfxContext, H264_CONTEXT *h264, bool avc444, uint32_t frameId, const VideoFrame &frame, quint64 resetGeneration)
{
    if (frame.image.isNull()) {
        return AvcResult::Unchanged;
    }

    if (surface.id == 0) {
        qCWarning(KRDP) << "No graphics surface available for AVC frame submission";
        return AvcResult::Unchanged;
    }

    const QImage image = frame.image.convertToFormat(QImage::Format_RGB32);
    const auto width = static_cast<UINT16>(image.width());
    const auto height = static_cast<UINT16>(image.height());
    // Always convert the whole frame: FreeRDP alternates between two YUV buffers, so a
    // partial update would leave the rest of the picture two frames old. FreeRDP compares
    // the new picture with the previous one and only lists the changed areas.
    const RECTANGLE_16 regionRect = {0, 0, width, height};

    RDPGFX_START_FRAME_PDU startFramePdu = {};
    RDPGFX_END_FRAME_PDU endFramePdu = {};
    const auto now = QDateTime::currentDateTimeUtc().time();
    startFramePdu.timestamp = now.hour() << 22 | now.minute() << 16 | now.second() << 10 | now.msec();
    startFramePdu.frameId = frameId;
    endFramePdu.frameId = frameId;

    RDPGFX_SURFACE_COMMAND surfaceCommand = {};
    surfaceCommand.surfaceId = surface.id;
    surfaceCommand.contextId = 0;
    surfaceCommand.format = PIXEL_FORMAT_BGRX32;
    surfaceCommand.right = width;
    surfaceCommand.bottom = height;
    surfaceCommand.width = width;
    surfaceCommand.height = height;

    INT32 rc = 0;
    RDPGFX_AVC420_BITMAP_STREAM avc420 = {};
    RDPGFX_AVC444_BITMAP_STREAM avc444Stream = {};
    if (avc444) {
        rc = avc444_compress(h264,
                             image.constBits(),
                             PIXEL_FORMAT_BGRX32,
                             image.bytesPerLine(),
                             width,
                             height,
                             2, // AVC444v2
                             &regionRect,
                             &avc444Stream.LC,
                             &avc444Stream.bitstream[0].data,
                             &avc444Stream.bitstream[0].length,
                             &avc444Stream.bitstream[1].data,
                             &avc444Stream.bitstream[1].length,
                             &avc444Stream.bitstream[0].meta,
                             &avc444Stream.bitstream[1].meta);
        if (rc > 0 && avc444Stream.LC == 2) {
            // Chroma only: [MS-RDPEGFX] 2.2.4.5 puts the chroma frame in the first
            // bitstream, but avc444_compress returns it as the second one.
            std::swap(avc444Stream.bitstream[0], avc444Stream.bitstream[1]);
        }
        // Size of the first stream as written on the wire: numRegionRects, then 8 bytes of
        // rectangle and 2 of quant/quality per region, then the H.264 data.
        avc444Stream.cbAvc420EncodedBitstream1 = 4 + 10 * avc444Stream.bitstream[0].meta.numRegionRects + avc444Stream.bitstream[0].length;
        surfaceCommand.codecId = RDPGFX_CODECID_AVC444v2;
        surfaceCommand.extra = &avc444Stream;
    } else {
        rc = avc420_compress(h264, image.constBits(), PIXEL_FORMAT_BGRX32, image.bytesPerLine(), width, height, &regionRect, &avc420.data, &avc420.length, &avc420.meta);
        surfaceCommand.codecId = RDPGFX_CODECID_AVC420;
        surfaceCommand.extra = &avc420;
    }

    const auto freeMetablocks = [&]() {
        free_h264_metablock(&avc420.meta);
        free_h264_metablock(&avc444Stream.bitstream[0].meta);
        free_h264_metablock(&avc444Stream.bitstream[1].meta);
    };

    if (rc < 0) {
        qCWarning(KRDP) << (avc444 ? "avc444_compress" : "avc420_compress") << "failed" << rc << "size" << frame.size;
        freeMetablocks();
        return AvcResult::Failed;
    }
    if (rc == 0) {
        freeMetablocks();
        return AvcResult::Unchanged;
    }

    if (KRDP().isDebugEnabled() && frameId < 3) {
        const auto describe = [](const RDPGFX_AVC420_BITMAP_STREAM &bs) {
            QString rects;
            for (UINT32 i = 0; i < std::min<UINT32>(bs.meta.numRegionRects, 3); ++i) {
                const auto &r = bs.meta.regionRects[i];
                const auto &q = bs.meta.quantQualityVals[i];
                rects += QStringLiteral("[%1,%2,%3,%4 qp%5 r%6 p%7 q%8]").arg(r.left).arg(r.top).arg(r.right).arg(r.bottom).arg(q.qp).arg(q.r).arg(q.p).arg(q.qualityVal);
            }
            const QByteArray data = QByteArray::fromRawData(reinterpret_cast<const char *>(bs.data), bs.length);
            return QStringLiteral("len %1 rects %2 %3 head %4 crc %5")
                .arg(bs.length)
                .arg(bs.meta.numRegionRects)
                .arg(rects, QString::fromLatin1(data.left(48).toHex()))
                .arg(qChecksum(data));
        };
        if (avc444) {
            qCDebug(KRDP) << "AVC frame" << frameId << "surface" << surfaceCommand.surfaceId << "AVC444v2 LC" << avc444Stream.LC << "cb1" << avc444Stream.cbAvc420EncodedBitstream1
                          << "bs1:" << describe(avc444Stream.bitstream[0]) << "bs2:" << describe(avc444Stream.bitstream[1]);
        } else {
            qCDebug(KRDP) << "AVC frame" << frameId << "surface" << surfaceCommand.surfaceId << "AVC420" << describe(avc420);
        }
    }

    if (this->resetGeneration != resetGeneration) {
        qCDebug(KRDP) << "Client reset its graphics while frame" << frameId << "was encoded; dropping it";
        freeMetablocks();
        return AvcResult::Stale;
    }

    const UINT status = gfxContext->SurfaceFrameCommand(gfxContext, &surfaceCommand, &startFramePdu, &endFramePdu);
    if (status != CHANNEL_RC_OK) {
        qCWarning(KRDP) << "SurfaceFrameCommand failed" << status << "frameId" << frameId << "surface" << surfaceCommand.surfaceId << (avc444 ? "AVC444" : "AVC420");
    }

    freeMetablocks();
    return AvcResult::Sent;
}

bool VideoStreamSurface::sendFrameProgressive(RdpgfxServerContext *gfxContext, PROGRESSIVE_CONTEXT *progressive, uint32_t frameId, const VideoFrame &frame, quint64 resetGeneration)
{
    if (frame.image.isNull()) {
        return false;
    }

    if (surface.id == 0) {
        qCWarning(KRDP) << "No graphics surface available for progressive frame submission";
        return false;
    }

    QImage image = frame.image.convertToFormat(QImage::Format_RGB32);
    const QRect frameRect(QPoint(0, 0), image.size());
    auto invalidRegion = toRegion16(frame.damage, frameRect);
    if (!invalidRegion) {
        qCWarning(KRDP) << "Failed to build invalid region for progressive frame";
        return false;
    }

    BYTE *encodedData = nullptr;
    UINT32 encodedSize = 0;
    const UINT32 rectCount = region16_n_rects(&*invalidRegion);
    const int compressionStatus = progressive_compress(progressive,
                                                       image.constBits(),
                                                       image.sizeInBytes(),
                                                       PIXEL_FORMAT_BGRX32,
                                                       image.width(),
                                                       image.height(),
                                                       image.bytesPerLine(),
                                                       &*invalidRegion,
                                                       &encodedData,
                                                       &encodedSize);
    if (compressionStatus < 0 || !encodedData || encodedSize == 0) {
        region16_uninit(&*invalidRegion);
        qCWarning(KRDP) << "Failed to compress progressive frame"
                        << "status" << compressionStatus << "rects" << rectCount << "size" << frame.size;
        return false;
    }

    RDPGFX_START_FRAME_PDU startFramePdu = {};
    RDPGFX_END_FRAME_PDU endFramePdu = {};

    const auto now = QDateTime::currentDateTimeUtc().time();
    startFramePdu.timestamp = now.hour() << 22 | now.minute() << 16 | now.second() << 10 | now.msec();
    startFramePdu.frameId = frameId;
    endFramePdu.frameId = frameId;

    const RECTANGLE_16 *extents = region16_extents(&*invalidRegion);
    RDPGFX_SURFACE_COMMAND surfaceCommand = {};
    surfaceCommand.surfaceId = surface.id;
    surfaceCommand.codecId = RDPGFX_CODECID_CAPROGRESSIVE;
    surfaceCommand.contextId = surface.codecContextId;
    surfaceCommand.format = PIXEL_FORMAT_BGRX32;
    surfaceCommand.left = extents->left;
    surfaceCommand.top = extents->top;
    surfaceCommand.right = extents->right;
    surfaceCommand.bottom = extents->bottom;
    surfaceCommand.width = frame.size.width();
    surfaceCommand.height = frame.size.height();
    surfaceCommand.length = encodedSize;
    surfaceCommand.data = encodedData;

    if (this->resetGeneration != resetGeneration) {
        qCDebug(KRDP) << "Client reset its graphics while frame" << frameId << "was encoded; dropping it";
        region16_uninit(&*invalidRegion);
        return false;
    }

    const UINT status = gfxContext->SurfaceFrameCommand(gfxContext, &surfaceCommand, &startFramePdu, &endFramePdu);
    if (status != CHANNEL_RC_OK) {
        qCWarning(KRDP) << "SurfaceFrameCommand failed" << status << "frameId" << frameId << "surface" << surface.id << "encodedBytes" << encodedSize
                        << "damageRects" << rectCount;
    }

    region16_uninit(&*invalidRegion);
    return true;
}
}
