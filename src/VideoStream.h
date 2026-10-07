// SPDX-FileCopyrightText: 2023 Arjen Hiemstra <ahiemstra@heimr.nl>
//
// SPDX-License-Identifier: LGPL-2.1-only OR LGPL-3.0-only OR LicenseRef-KDE-Accepted-LGPL

#pragma once

#include <chrono>
#include <memory>

#include <QObject>
#include <QPoint>
#include <QRegion>
#include <QSize>

#include <PipeWireEncodedStream>
#include <PipeWireSourceStream>
#include <freerdp/server/rdpgfx.h>

#include "VideoFrame.h"
#include "krdp_export.h"

namespace KRdp
{

class RdpConnection;
class VideoStreamSurface;

/**
 * How the server encodes video, read from the settings file at startup.
 */
struct KRDP_EXPORT VideoEncoderSettings {
    enum class Codec {
        KPipeWire, ///< H.264 4:2:0 from the KPipeWire encoder (the original path)
        Auto, ///< AVC444 if the client supports it, else AVC420, else RemoteFX
        AVC444, ///< H.264 4:4:4 (AVC444v2) from FreeRDP, falling back like Auto
        AVC420, ///< H.264 4:2:0 from FreeRDP, else RemoteFX
        RemoteFX, ///< RemoteFX Progressive, no H.264
    };
    enum class Encoder {
        Auto, ///< NVENC, and libx264 if NVENC cannot encode
        NVENC,
        Libx264,
    };
    enum class Speed {
        Default,
        Fast,
        Fastest,
    };

    Codec codec = Codec::KPipeWire;
    Encoder encoder = Encoder::Auto;
    Speed speed = Speed::Fast;
    /// RemoteFX quality, 0 to 100. 100 keeps every detail, 50 is the MS default.
    int remoteFxQuality = 100;
    /// AVC444: convert and encode on the GPU (GL compute shader and NVENC from CUDA memory)
    /// when CUDA is available, instead of FreeRDP's CPU conversion. Ignored with libx264.
    bool gpuEncode = true;
    /// GPU AVC444: seconds between full-screen key frames, so a client decoder that got out
    /// of step recovers. 0 turns them off; then only a reconnect brings a key frame.
    int keyFrameInterval = 10;

    /// Build settings from the values in krdpserverrc. Unknown values keep the default.
    static VideoEncoderSettings
    fromStrings(const QString &codec, const QString &encoder, const QString &speed, int remoteFxQuality, bool gpuEncode, int keyFrameInterval);
};

/**
 * A class that encapsulates an RdpGfx video stream.
 *
 * Video streaming is done using the "RDP Graphics Pipeline" protocol
 * extension which allows using h264 as the codec for the video stream.
 * However, this protocol extension is fairly complex to setup and use.
 *
 * VideoStream makes sure to handle most of the complexity of the RdpGfx
 * protocol like ensuring the client knows the right resolution and a
 * surface at the right size. It also takes care of sending the frames,
 * using a separate thread for a submission queue.
 *
 * VideoStream is managed by Session. Each session will have one instance
 * of this class.
 */
class KRDP_EXPORT VideoStream : public QObject
{
    Q_OBJECT

public:
    enum class EncodingMode {
        H264, ///< AVC420 packets from KPipeWire
        Progressive,
        AVC420, ///< raw frames encoded by FreeRDP
        AVC444,
    };

    explicit VideoStream(RdpConnection *session);
    ~VideoStream() override;

    static bool h264Disabled();
    static void setEncoderSettings(const VideoEncoderSettings &settings);
    static const VideoEncoderSettings &encoderSettings();
    /// Whether the settings allow AVC444, so the server can advertise it.
    static bool avc444Allowed();

    bool initialize();
    void close();
    Q_SIGNAL void closed();
    Q_SIGNAL void sizeChanged(const QSize &size);
    Q_SIGNAL void cursorChanged(const PipeWireCursor &cursor);

    /**
     * Queue a frame to be sent to the client.
     *
     * This will add the provided frame to the queue of frames that should
     * be sent to the client.
     *
     * \param frame The frame to send.
     */
    void queueFrame(const VideoFrame &frame);

    /**
     * Indicate that the video state should be reset.
     *
     * This means the screen resolution and other information of the client
     * will be updated based on the current state of the VideoStream.
     */
    void reset();

    /**
     */
    bool enabled() const;
    void setEnabled(bool enabled);
    Q_SIGNAL void enabledChanged();
    void setStreamingEnabled(bool enabled);
    void setVideoQuality(quint8 quality);
    void setAdaptiveQuality(bool enabled);
    void seedQuality(quint8 quality);
    void setRequestedSize(const QSize &size);
    /**
     * Hold back frames of any other size until one of this size arrives or the
     * timeout passes. Used while a resize hook changes the monitor to the client's
     * size: mstsc drops the connection if a new session starts at the wrong size.
     */
    void waitForFrameSize(const QSize &size, std::chrono::milliseconds timeout);
    void setPipeWireSource(quint32 nodeId, quint64 objectSerial, int fd = -1);

    bool openChannel();

private:
    friend class VideoStreamSurface;
    friend BOOL gfxChannelIdAssigned(RdpgfxServerContext *, uint32_t);
    friend uint32_t gfxCapsAdvertise(RdpgfxServerContext *, const RDPGFX_CAPS_ADVERTISE_PDU *);
    friend uint32_t gfxFrameAcknowledge(RdpgfxServerContext *, const RDPGFX_FRAME_ACKNOWLEDGE_PDU *);

    void updateBackpressure();

    bool onChannelIdAssigned(uint32_t channelId);
    uint32_t onCapsAdvertise(const RDPGFX_CAPS_ADVERTISE_PDU *capsAdvertise);
    uint32_t onFrameAcknowledge(const RDPGFX_FRAME_ACKNOWLEDGE_PDU *frameAcknowledge);

    void onPacketReceived(const PipeWireEncodedStream::Packet &data);
    void onFrameReceived(const PipeWireFrame &frame);
    void setActiveEncodingMode(EncodingMode mode);
    void setSize(const QSize &size);
    bool streamingEnabled() const;
    void failVideoInitialization();
    void destroySurface();
    void forgetSurface();
    void performReset(QSize size);
    bool hasInFlightCapacity() const;
    void sendFrame(const VideoFrame &frame);

    void updateInFlightWindow();
    double effectiveProducerFps();
    void updateAdaptiveQuality();
    void applyQuality();

    class Private;
    const std::unique_ptr<Private> d;
};

}
