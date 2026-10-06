// SPDX-FileCopyrightText: 2023 Arjen Hiemstra <ahiemstra@heimr.nl>
//
// This file is roughly based on grd-rdp-graphics-pipeline.c from Gnome Remote
// Desktop which is:
//
// SPDX-FileCopyrightText: 2021 Pascal Nowack
//
// SPDX-License-Identifier: GPL-2.0-or-later

#include "VideoStream.h"
#include "VideoStreamSurface.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>
#include <thread>

#include <QQueue>
#include <QHash>

#include <freerdp/codec/h264.h>
#include <freerdp/freerdp.h>
#include <freerdp/peer.h>
#include <freerdp/update.h>
#include <qassert.h>

#include "GpuH264Encoder.h"
#include "NetworkDetection.h"
#include "PeerContext_p.h"
#include "RdpConnection.h"

#include "krdp_logging.h"

namespace KRdp
{

namespace clk = std::chrono;

constexpr qsizetype MaximumInFlightFrames = 2; // in-flight window floor
constexpr double InFlightGain = 1.0; // window spans this many round trips of frames
constexpr double LatencyBudgetSec = 1.0; // never buffer more than this many seconds of video
constexpr double MinimumWindowFrameRate = 5.0; // floor for producer-rate window sizing (avoids stop-and-wait)
constexpr double ProducerFpsEwmaAlpha = 0.25; // smoothing for the producer-rate estimate
constexpr double RttEwmaAlpha = 0.125; // second-stage smoothing of the (already windowed) averageRTT
constexpr double MinimumValidRttMs = 5.0; // ignore implausibly-low RTT samples
constexpr double MaximumValidRttMs = 60000.0; // ignore garbage RTT samples

constexpr qsizetype HighQueueCount = 3; // Level of frames in the pending-send queue that pauses frames pre-encoder
constexpr qsizetype LowQueueCount = 1; // Level of frames in the pending-send queue that resumes the encoder

constexpr uint32_t ProgressiveCodecContextId = 1;

constexpr clk::milliseconds FirstFrameDelay(500); // after the first CapsConfirm of a connection

// RemoteFX quantization, in the order LL3, LH3, HL3, HH3, LH2, HL2, HH2, LH1, HL1, HH1.
// 6 keeps a band at full precision; each step above halves it. The finest bands (the
// last three) carry text edges.
static std::array<UINT32, 10> remoteFxQuantization(int quality)
{
    if (quality >= 90) {
        return {6, 6, 6, 6, 6, 6, 6, 6, 6, 6};
    }
    if (quality >= 70) {
        return {6, 6, 6, 6, 6, 6, 7, 7, 7, 8};
    }
    if (quality >= 50) {
        return {6, 6, 6, 6, 7, 7, 8, 8, 8, 9}; // the MS default
    }
    if (quality >= 30) {
        return {7, 7, 7, 7, 8, 8, 9, 9, 9, 10};
    }
    return {8, 8, 8, 8, 9, 9, 10, 10, 10, 11};
}

// H.264 constant QP for the FreeRDP encoders: quality 100 gives QP 12, 50 gives QP 26.
static quint32 h264QpForQuality(int quality)
{
    return quint32(std::clamp(int(std::lround(40.0 - quality * 0.28)), 10, 45));
}

constexpr clk::system_clock::duration QualityUpdateInterval = clk::milliseconds(1500);
constexpr int MinAdaptiveQuality = 10;
constexpr int QualityStepUp = 5;
constexpr int QualityStepDown = 10;

struct BitrateAnchor {
    double pixels;
    double kbit;
};

// "Quality 100" targets by resolution, no fps term - matches RustDesk's base_bitrate().
constexpr std::array<BitrateAnchor, 4> FullQualityBitrateAnchors = {{
    {921'600.0, 1500.0}, // 1280x720
    {2'073'600.0, 3110.0}, // 1920x1080
    {3'686'400.0, 4500.0}, // 2560x1440
    {8'294'400.0, 7500.0}, // 3840x2160
}};

static double fullQualityKbit(double pixels)
{
    const auto *nearest = std::min_element(FullQualityBitrateAnchors.begin(), FullQualityBitrateAnchors.end(), [pixels](const auto &a, const auto &b) {
        return std::abs(a.pixels - pixels) < std::abs(b.pixels - pixels);
    });
    return nearest->kbit * (pixels / nearest->pixels);
}

struct RdpCapsInformation {
    uint32_t version;
    RDPGFX_CAPSET capSet;
    bool avcSupported : 1 = false;
    bool yuv420Supported : 1 = false;
    bool avc444Supported : 1 = false;
};

const char *capVersionToString(uint32_t version)
{
    switch (version) {
    case RDPGFX_CAPVERSION_107:
        return "RDPGFX_CAPVERSION_107";
    case RDPGFX_CAPVERSION_106:
        return "RDPGFX_CAPVERSION_106";
    case RDPGFX_CAPVERSION_105:
        return "RDPGFX_CAPVERSION_105";
    case RDPGFX_CAPVERSION_104:
        return "RDPGFX_CAPVERSION_104";
    case RDPGFX_CAPVERSION_103:
        return "RDPGFX_CAPVERSION_103";
    case RDPGFX_CAPVERSION_102:
        return "RDPGFX_CAPVERSION_102";
    case RDPGFX_CAPVERSION_101:
        return "RDPGFX_CAPVERSION_101";
    case RDPGFX_CAPVERSION_10:
        return "RDPGFX_CAPVERSION_10";
    case RDPGFX_CAPVERSION_81:
        return "RDPGFX_CAPVERSION_81";
    case RDPGFX_CAPVERSION_8:
        return "RDPGFX_CAPVERSION_8";
    default:
        return "UNKNOWN_VERSION";
    }
}

BOOL gfxChannelIdAssigned(RdpgfxServerContext *context, uint32_t channelId)
{
    auto stream = reinterpret_cast<VideoStream *>(context->custom);
    if (stream->onChannelIdAssigned(channelId)) {
        return TRUE;
    }
    return FALSE;
}

uint32_t gfxCapsAdvertise(RdpgfxServerContext *context, const RDPGFX_CAPS_ADVERTISE_PDU *capsAdvertise)
{
    auto stream = reinterpret_cast<VideoStream *>(context->custom);
    return stream->onCapsAdvertise(capsAdvertise);
}

uint32_t gfxFrameAcknowledge(RdpgfxServerContext *context, const RDPGFX_FRAME_ACKNOWLEDGE_PDU *frameAcknowledge)
{
    auto stream = reinterpret_cast<VideoStream *>(context->custom);
    return stream->onFrameAcknowledge(frameAcknowledge);
}

uint32_t gfxQoEFrameAcknowledge(RdpgfxServerContext *, const RDPGFX_QOE_FRAME_ACKNOWLEDGE_PDU *)
{
    return CHANNEL_RC_OK;
}

class KRDP_NO_EXPORT VideoStream::Private
{
public:
    using RdpGfxContextPtr = std::unique_ptr<RdpgfxServerContext, decltype(&rdpgfx_server_context_free)>;
    using ProgressiveContextPtr = std::unique_ptr<PROGRESSIVE_CONTEXT, decltype(&progressive_context_free)>;
    using H264ContextPtr = std::unique_ptr<H264_CONTEXT, decltype(&h264_context_free)>;

    bool ensureH264(const QSize &size);

    RdpConnection *session;
    std::optional<EncodingMode> activeEncodingMode;
    std::unique_ptr<VideoStreamSurface> surface;

    RdpGfxContextPtr gfxContext = RdpGfxContextPtr(nullptr, rdpgfx_server_context_free);
    ProgressiveContextPtr progressive = ProgressiveContextPtr(nullptr, progressive_context_free);

    // FreeRDP H.264 encoder for AVC420/AVC444. Only the frame submission thread touches it;
    // other threads ask for changes through the atomics below.
    H264ContextPtr h264 = H264ContextPtr(nullptr, h264_context_free);
    QSize h264Size;
    quint32 h264Qp = 0;
    quint64 h264Generation = 0; ///< counts encoder (re)opens; each starts with a key frame
    // NVENC fed straight from the GPU converter's CUDA frames; see GpuH264Encoder.
    GpuH264Encoder gpuEncoder;
    // The client needs a key frame on the GPU path: a forced IDR picture, not a new encoder.
    std::atomic_bool gpuKeyFrameNeeded = false;
    GpuH264Encoder::Speed gpuSpeed() const;
    /// Opens NVENC for a frame that is waiting, before frames may be sent; see sendFrame().
    void prepareGpuEncoder();
    bool nvencFailed = false;
    std::atomic_bool h264Recreate = false;
    std::atomic<quint32> h264TargetQp = h264QpForQuality(100);

    uint32_t frameId = 0;
    uint32_t channelId = 0;
    uint16_t nextSurfaceId = 1;
    bool enabled = false;
    bool streamingEnabled = false;
    bool capsConfirmed = false;

    std::mutex expectedSizeMutex;
    QSize expectedFrameSize; // see waitForFrameSize()
    clk::steady_clock::time_point expectedSizeDeadline;
    // No frames before this time; see onCapsAdvertise().
    std::atomic<clk::steady_clock::time_point> firstFrameNotBefore{};
    bool channelOpen = false;

    std::jthread frameSubmissionThread;
    std::mutex frameQueueMutex;
    QQueue<VideoFrame> frameQueue;
    struct PendingFrame {
        clk::steady_clock::time_point sent; // encode start, then the time it was sent
        quint32 bytes = 0; // encoded size; 0 if the path does not report it
        qsizetype inFlight = 0; // frames in flight when it was sent, itself included
    };
    QHash<uint32_t, PendingFrame> pendingFrames; // by frame id
    std::mutex pendingFramesMutex;

    // Latency statistics for the debug log, guarded by pendingFramesMutex. See logLatencyStats().
    struct LatencyStats {
        clk::steady_clock::time_point windowStart = clk::steady_clock::now();
        int frames = 0;
        clk::microseconds encodeTotal{0};
        clk::microseconds encodeMax{0};
        int acks = 0;
        clk::microseconds ackTotal{0};
        clk::microseconds ackMax{0};
        uint32_t queueDepth = 0;
        quint64 bytesTotal = 0;
        quint32 bytesMax = 0;
        // Acknowledgement time by frame size: below 64 kB, below 256 kB, and larger.
        std::array<int, 3> sizeAcks{};
        std::array<clk::microseconds, 3> sizeAckTotal{};
    } latency;
    void logLatencyStats(clk::steady_clock::time_point now);

    std::atomic_int requestedFrameRate = 60;
    std::atomic<qsizetype> maxInFlight{MaximumInFlightFrames}; // recomputed from RTT on rttChanged
    // Producer-rate estimate for window sizing (see VideoStream::effectiveProducerFps()).
    std::atomic<uint64_t> producedFrames = 0; // frames entering krdp; written from frame callbacks
    uint64_t lastProducedFrames = 0; // touched only by updateInFlightWindow()
    clk::steady_clock::time_point lastProducerRateUpdate{}; // touched only by updateInFlightWindow()
    double smoothedProducerFps = MinimumWindowFrameRate; // touched only by updateInFlightWindow()
    // RTT smoothing for window sizing (see updateInFlightWindow()); touched only there.
    double smoothedRttMs = 0.0; // EWMA of valid averageRTT samples
    bool hasSmoothedRtt = false;
    double baseRttMs = 0.0; // session-minimum valid RTT (path BDP floor)
    bool hasBaseRtt = false;

    bool initialized = false;
    quint8 quality = 100;
    quint8 qualityCap = 100;
    bool adaptiveQuality = true;
    clk::system_clock::time_point lastQualityUpdate;
};

static QString encodingModeName(VideoStream::EncodingMode mode)
{
    switch (mode) {
    case VideoStream::EncodingMode::H264:
        return QStringLiteral("h264");
    case VideoStream::EncodingMode::Progressive:
        return QStringLiteral("progressive");
    case VideoStream::EncodingMode::AVC420:
        return QStringLiteral("avc420");
    case VideoStream::EncodingMode::AVC444:
        return QStringLiteral("avc444");
    }
    Q_UNREACHABLE();
}

static bool usesFreeRdpH264(std::optional<VideoStream::EncodingMode> mode)
{
    return mode == VideoStream::EncodingMode::AVC420 || mode == VideoStream::EncodingMode::AVC444;
}

static VideoEncoderSettings s_encoderSettings;

VideoEncoderSettings VideoEncoderSettings::fromStrings(const QString &codec, const QString &encoder, const QString &speed, int remoteFxQuality, bool gpuEncode)
{
    VideoEncoderSettings settings;
    const auto is = [](const QString &value, QLatin1StringView name) {
        return value.compare(name, Qt::CaseInsensitive) == 0;
    };

    if (is(codec, QLatin1StringView("Auto"))) {
        settings.codec = Codec::Auto;
    } else if (is(codec, QLatin1StringView("AVC444"))) {
        settings.codec = Codec::AVC444;
    } else if (is(codec, QLatin1StringView("AVC420"))) {
        settings.codec = Codec::AVC420;
    } else if (is(codec, QLatin1StringView("RemoteFX"))) {
        settings.codec = Codec::RemoteFX;
    } else if (!codec.isEmpty() && !is(codec, QLatin1StringView("KPipeWire"))) {
        qCWarning(KRDP) << "Unknown VideoCodec" << codec << "- using KPipeWire";
    }

    if (is(encoder, QLatin1StringView("NVENC"))) {
        settings.encoder = Encoder::NVENC;
    } else if (is(encoder, QLatin1StringView("libx264"))) {
        settings.encoder = Encoder::Libx264;
    } else if (!encoder.isEmpty() && !is(encoder, QLatin1StringView("Auto"))) {
        qCWarning(KRDP) << "Unknown VideoEncoder" << encoder << "- using Auto";
    }

    if (is(speed, QLatin1StringView("Default"))) {
        settings.speed = Speed::Default;
    } else if (is(speed, QLatin1StringView("Fastest"))) {
        settings.speed = Speed::Fastest;
    } else if (!speed.isEmpty() && !is(speed, QLatin1StringView("Fast"))) {
        qCWarning(KRDP) << "Unknown EncoderSpeed" << speed << "- using Fast";
    }

    settings.remoteFxQuality = std::clamp(remoteFxQuality, 0, 100);
    settings.gpuEncode = gpuEncode;
    return settings;
}

void VideoStream::setEncoderSettings(const VideoEncoderSettings &settings)
{
    s_encoderSettings = settings;
}

const VideoEncoderSettings &VideoStream::encoderSettings()
{
    return s_encoderSettings;
}

bool VideoStream::h264Disabled()
{
    static const bool h264Disabled = qEnvironmentVariableIntValue("KRDP_DISABLE_H264") != 0;
    return h264Disabled || s_encoderSettings.codec == VideoEncoderSettings::Codec::RemoteFX;
}

bool VideoStream::avc444Allowed()
{
    return !h264Disabled() && (s_encoderSettings.codec == VideoEncoderSettings::Codec::Auto || s_encoderSettings.codec == VideoEncoderSettings::Codec::AVC444);
}

GpuH264Encoder::Speed VideoStream::Private::gpuSpeed() const
{
    switch (encoderSettings().speed) {
    case VideoEncoderSettings::Speed::Fastest:
        return GpuH264Encoder::Speed::Fastest;
    case VideoEncoderSettings::Speed::Default:
        return GpuH264Encoder::Speed::Default;
    case VideoEncoderSettings::Speed::Fast:
        break;
    }
    return GpuH264Encoder::Speed::Fast;
}

void VideoStream::Private::prepareGpuEncoder()
{
    // Opening NVENC takes about 100 ms. The first frame of a connection waits 500 ms anyway
    // (see onCapsAdvertise()), so open it in that time.
    if (activeEncodingMode != EncodingMode::AVC444) {
        return;
    }
    std::shared_ptr<AVFrame> lumaFrame;
    {
        std::lock_guard lock(frameQueueMutex);
        if (frameQueue.isEmpty() || !frameQueue.first().avc444) {
            return;
        }
        lumaFrame = frameQueue.first().avc444->lumaFrame;
    }
    if (lumaFrame) {
        gpuEncoder.ensure(lumaFrame.get(), h264TargetQp.load(), requestedFrameRate.load(), gpuSpeed());
    }
}

bool VideoStream::Private::ensureH264(const QSize &size)
{
    if (h264Recreate.exchange(false)) {
        h264.reset();
    }

    const auto &settings = s_encoderSettings;
    if (!h264) {
        h264.reset(h264_context_new(TRUE));
        if (!h264) {
            qCWarning(KRDP) << "Failed to create H.264 encoder context";
            return false;
        }

        const bool nvenc = settings.encoder == VideoEncoderSettings::Encoder::NVENC
            || (settings.encoder == VideoEncoderSettings::Encoder::Auto && !nvencFailed);
        H264_ENCODER_SPEED speed = H264_ENCODER_SPEED_FAST;
        if (settings.speed == VideoEncoderSettings::Speed::Default) {
            speed = H264_ENCODER_SPEED_DEFAULT;
        } else if (settings.speed == VideoEncoderSettings::Speed::Fastest) {
            speed = H264_ENCODER_SPEED_FASTEST;
        }
        if (!h264_context_set_option(h264.get(), H264_CONTEXT_OPTION_ENCODER, nvenc ? H264_ENCODER_NVENC : H264_ENCODER_LIBX264)
            || !h264_context_set_option(h264.get(), H264_CONTEXT_OPTION_ENCODER_SPEED, speed)) {
            qCWarning(KRDP) << "This FreeRDP cannot choose the H.264 encoder; build KRDP against the krdp-local FreeRDP";
        }
        qCDebug(KRDP) << "H.264 encoder:" << (nvenc ? "NVENC" : "libx264");
        h264Size = QSize();
    }

    const quint32 qp = h264TargetQp.load();
    if (h264Size == size && h264Qp == qp) {
        return true;
    }

    // A reset opens a new encoder, so it also applies the new QP; the next frame is a key frame.
    if (!h264_context_reset(h264.get(), size.width(), size.height())) {
        qCWarning(KRDP) << "Failed to reset H.264 encoder for size" << size;
        h264.reset();
        return false;
    }
    ++h264Generation;
    if (!h264_context_set_option(h264.get(), H264_CONTEXT_OPTION_RATECONTROL, H264_RATECONTROL_CQP)
        || !h264_context_set_option(h264.get(), H264_CONTEXT_OPTION_QP, qp)
        || !h264_context_set_option(h264.get(), H264_CONTEXT_OPTION_FRAMERATE, quint32(requestedFrameRate.load()))
        || !h264_context_set_option(h264.get(), H264_CONTEXT_OPTION_USAGETYPE, H264_SCREEN_CONTENT_REAL_TIME)) {
        qCWarning(KRDP) << "Failed to configure H.264 encoder";
    }
    h264Size = size;
    h264Qp = qp;
    return true;
}

VideoStream::VideoStream(RdpConnection *session)
    : QObject(nullptr)
    , d(std::make_unique<Private>())
{
    d->session = session;
    d->surface = std::make_unique<VideoStreamSurface>(this);
}

void VideoStream::setActiveEncodingMode(EncodingMode mode)
{
    if (d->activeEncodingMode == mode) {
        return;
    }

    {
        std::lock_guard lock(d->frameQueueMutex);
        d->frameQueue.clear();
    }
    d->surface->setActiveEncodingMode(mode, d->quality, d->requestedFrameRate.load());
    d->activeEncodingMode = mode;
    if (usesFreeRdpH264(mode)) {
        // A new stream needs a new encoder, so the client gets a key frame first.
        d->h264Recreate = true;
        d->gpuKeyFrameNeeded = true;
    }
}

void VideoStream::setSize(const QSize &newSize)
{
    if (d->surface->size == newSize) {
        return;
    }

    d->surface->size = newSize;
    Q_EMIT sizeChanged(newSize);
}

bool VideoStream::streamingEnabled() const
{
    return d->streamingEnabled;
}

void VideoStream::failVideoInitialization()
{
    d->session->close(RdpConnection::CloseReason::VideoInitFailed);
}

VideoStream::~VideoStream()
{
    close();
}

bool VideoStream::initialize()
{
    if (d->initialized) {
        return true;
    }

    d->gfxContext.reset(rdpgfx_server_context_new(contextForPeer(d->session->rdpPeer())->virtualChannelManager));
    if (!d->gfxContext) {
        qCWarning(KRDP) << "Failed to create graphics pipeline context";
        return false;
    }

    d->gfxContext->custom = this;
    d->gfxContext->ChannelIdAssigned = gfxChannelIdAssigned;
    d->gfxContext->CapsAdvertise = gfxCapsAdvertise;
    d->gfxContext->FrameAcknowledge = gfxFrameAcknowledge;
    d->gfxContext->QoeFrameAcknowledge = gfxQoEFrameAcknowledge;
    d->gfxContext->rdpcontext = d->session->rdpPeerContext();

    if (!d->gfxContext->Initialize(d->gfxContext.get(), FALSE)) {
        qCWarning(KRDP) << "Failed to initialize graphics pipeline context";
        d->gfxContext.reset();
        return false;
    }

    d->progressive.reset(progressive_context_new(TRUE));
    if (!d->progressive) {
        qCWarning(KRDP) << "Failed to create progressive codec context";
        d->gfxContext.reset();
        return false;
    }
    const auto quantization = remoteFxQuantization(encoderSettings().remoteFxQuality);
    if (!progressive_context_set_quantization_values(d->progressive.get(), quantization.data(), quantization.size())) {
        qCWarning(KRDP) << "Failed to set RemoteFX quality" << encoderSettings().remoteFxQuality;
    }

    d->initialized = true;

    connect(d->session->networkDetection(), &NetworkDetection::rttChanged, this, &VideoStream::updateInFlightWindow);
    connect(d->session->networkDetection(), &NetworkDetection::bandwidthChanged, this, &VideoStream::updateAdaptiveQuality);

    d->frameSubmissionThread = std::jthread([this](std::stop_token token) {
        while (!token.stop_requested()) {
            if (!hasInFlightCapacity() || !d->gfxContext || !d->capsConfirmed || clk::steady_clock::now() < d->firstFrameNotBefore.load()) {
                d->prepareGpuEncoder();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            VideoFrame nextFrame;
            {
                std::unique_lock lock(d->frameQueueMutex);
                if (!d->frameQueue.isEmpty()) {
                    nextFrame = d->frameQueue.takeFirst();
                }
            }
            if (nextFrame.size.isEmpty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000) / d->requestedFrameRate.load());
                continue;
            }
            // sendFrame() can drop the frame without sending it (write blocked, or a GFX reset
            // in progress). Re-check backpressure for every frame taken off the queue, otherwise
            // a paused encoder is never resumed once the queue drains that way.
            QMetaObject::invokeMethod(this, &VideoStream::updateBackpressure, Qt::QueuedConnection);
            sendFrame(nextFrame);
        }
    });

    qCDebug(KRDP) << "Video stream initialized with H.264" << (h264Disabled() ? "disabled" : "enabled") << "codec setting"
                  << int(encoderSettings().codec) << "RemoteFX quality" << encoderSettings().remoteFxQuality;

    return true;
}

void VideoStream::close()
{
    if (d->surface->encodedStream) {
        d->surface->encodedStream->stop();
    }
    if (d->surface->sourceStream) {
        d->surface->sourceStream->stopStreaming();
    }
    if (d->frameSubmissionThread.joinable()) {
        d->frameSubmissionThread.request_stop();
        d->frameSubmissionThread.join();
    }

    {
        std::lock_guard lock(d->pendingFramesMutex);
        d->pendingFrames.clear();
    }
    {
        std::lock_guard lock(d->frameQueueMutex);
        d->frameQueue.clear();
    }

    destroySurface();

    if (d->gfxContext) {
        if (d->channelOpen) {
            d->gfxContext->Close(d->gfxContext.get());
            d->channelOpen = false;
        }
        d->gfxContext.reset();
    }
    d->activeEncodingMode.reset();
    d->initialized = false;

    Q_EMIT closed();
}

void VideoStream::queueFrame(const KRdp::VideoFrame &frame)
{
    if (d->session->state() != RdpConnection::State::Streaming || !d->enabled) {
        return;
    }
    d->producedFrames.fetch_add(1, std::memory_order_relaxed);

    if (d->activeEncodingMode == EncodingMode::H264) {
        std::lock_guard lock(d->frameQueueMutex);
        if (frame.isKeyFrame) {
            d->frameQueue.clear();
        }
        d->frameQueue.append(frame);
    } else if (d->activeEncodingMode) {
        std::lock_guard lock(d->frameQueueMutex);
        // for the raw frame paths we only need to keep the latest frame, but accumulate damage
        QRegion lastDamage;
        if (!d->frameQueue.isEmpty()) {
            lastDamage = d->frameQueue.last().damage;
            d->frameQueue.clear();
        }
        VideoFrame nextFrame = frame;
        nextFrame.damage += lastDamage;
        d->frameQueue.append(std::move(nextFrame));
    }
    updateBackpressure();
}

void VideoStream::reset()
{
    d->surface->pendingReset = true;
}

bool VideoStream::enabled() const
{
    return d->enabled;
}

void VideoStream::setEnabled(bool enabled)
{
    if (d->enabled == enabled) {
        return;
    }

    d->enabled = enabled;
    Q_EMIT enabledChanged();
}

void VideoStream::setStreamingEnabled(bool enabled)
{
    if (d->streamingEnabled == enabled) {
        return;
    }

    d->streamingEnabled = enabled;
    d->surface->setStreamingEnabled(enabled);
}

void VideoStream::applyQuality()
{
    d->surface->setVideoQuality(d->quality);
    // The FreeRDP encoders use the configured quality only; see updateAdaptiveQuality().
    d->h264TargetQp = h264QpForQuality(d->qualityCap);
}

void VideoStream::setVideoQuality(quint8 quality)
{
    d->qualityCap = quality;
    d->quality = d->adaptiveQuality ? std::min(d->quality, d->qualityCap) : d->qualityCap;
    applyQuality();
}

void VideoStream::setAdaptiveQuality(bool enabled)
{
    if (d->adaptiveQuality == enabled) {
        return;
    }
    d->adaptiveQuality = enabled;
    if (!enabled) {
        d->quality = d->qualityCap;
        applyQuality();
    }
}

void VideoStream::seedQuality(quint8 quality)
{
    if (!d->adaptiveQuality || d->activeEncodingMode != EncodingMode::H264) {
        return;
    }
    const quint8 hi = std::max<quint8>(d->qualityCap, quint8(MinAdaptiveQuality));
    d->quality = std::clamp<quint8>(quality, quint8(MinAdaptiveQuality), hi);
    applyQuality();
}

void VideoStream::waitForFrameSize(const QSize &size, std::chrono::milliseconds timeout)
{
    std::lock_guard lock(d->expectedSizeMutex);
    d->expectedFrameSize = size;
    d->expectedSizeDeadline = clk::steady_clock::now() + timeout;
}

void VideoStream::setRequestedSize(const QSize &size)
{
    d->surface->setRequestedSize(size);
}

void VideoStream::setPipeWireSource(quint32 nodeId, quint64 objectSerial, int fd)
{
    d->surface->setPipeWireSource(nodeId, objectSerial, fd);
}

bool VideoStream::onChannelIdAssigned(uint32_t channelId)
{
    d->channelId = channelId;

    return true;
}

uint32_t VideoStream::onCapsAdvertise(const RDPGFX_CAPS_ADVERTISE_PDU *capsAdvertise)
{
    // Windows clients (mstsc) send CapsAdvertise twice: once during
    // initial setup and again after confirming. If we already confirmed
    // caps, this is a GFX channel reset — clear surface state so
    // surfaces get re-created on the next frame.
    const bool readvertised = d->capsConfirmed;
    std::unique_lock frameLock(d->surface->frameMutex, std::defer_lock);
    if (d->capsConfirmed) {
        qCDebug(KRDP) << "GFX channel reset (re-advertisement), resetting surface state";
        // Tell a frame that is being encoded not to send, then wait for it to finish.
        ++d->surface->resetGeneration;
        frameLock.lock();
        d->capsConfirmed = false;
        d->surface->pendingReset = true;
        // A client that re-advertises has already dropped its GFX state. Sending it
        // DeleteSurface or DeleteEncodingContext for surfaces it no longer knows makes
        // mstsc stop acknowledging frames (black screen), so only forget them locally.
        forgetSurface();
        {
            std::lock_guard lock(d->pendingFramesMutex);
            d->pendingFrames.clear();
        }
        // Not held across the blocking call to the main thread below.
        frameLock.unlock();
    }

    auto capsSets = capsAdvertise->capsSets;
    auto count = capsAdvertise->capsSetCount;

    std::vector<RdpCapsInformation> capsInformation;
    capsInformation.reserve(count);

    qCDebug(KRDP) << "Received caps:";
    for (int i = 0; i < count; ++i) {
        auto set = capsSets[i];

        RdpCapsInformation caps;
        caps.version = set.version;
        caps.capSet = set;

        switch (caps.version) {
        case RDPGFX_CAPVERSION_107:
        case RDPGFX_CAPVERSION_106:
        case RDPGFX_CAPVERSION_105:
        case RDPGFX_CAPVERSION_104:
            caps.yuv420Supported = true;
            Q_FALLTHROUGH();
        case RDPGFX_CAPVERSION_103:
        case RDPGFX_CAPVERSION_102:
        case RDPGFX_CAPVERSION_101:
        case RDPGFX_CAPVERSION_10:
            if (!(set.flags & RDPGFX_CAPS_FLAG_AVC_DISABLED)) {
                caps.avcSupported = true;
                // A thin client asks for AVC420 only.
                caps.avc444Supported = !(set.flags & RDPGFX_CAPS_FLAG_AVC_THINCLIENT);
            }
            break;
        case RDPGFX_CAPVERSION_81:
            if (set.flags & RDPGFX_CAPS_FLAG_AVC420_ENABLED) {
                caps.avcSupported = true;
                caps.yuv420Supported = true;
            }
            break;
        case RDPGFX_CAPVERSION_8:
            break;
        }

        qCDebug(KRDP) << " " << capVersionToString(caps.version) << "flags:" << Qt::hex << set.flags << Qt::dec << "AVC:" << caps.avcSupported
                      << "YUV420:" << caps.yuv420Supported << "AVC444:" << caps.avc444Supported;

        capsInformation.push_back(caps);
    }

    const bool supportsProgresive = !capsInformation.empty();

    const bool supportsH264 = std::any_of(capsInformation.begin(), capsInformation.end(), [](const RdpCapsInformation &caps) {
        return caps.avcSupported && caps.yuv420Supported;
    });

    auto maxVersion = std::max_element(capsInformation.begin(), capsInformation.end(), [](const auto &first, const auto &second) {
        return first.version < second.version;
    });
    // AVC444 is only possible with the caps set we confirm, which is the highest one.
    const bool supportsAvc444 = maxVersion != capsInformation.end() && maxVersion->avcSupported && maxVersion->avc444Supported;

    EncodingMode negotiatedMode = EncodingMode::Progressive;
    if (!h264Disabled() && supportsH264) {
        switch (encoderSettings().codec) {
        case VideoEncoderSettings::Codec::KPipeWire:
            negotiatedMode = EncodingMode::H264;
            break;
        case VideoEncoderSettings::Codec::Auto:
        case VideoEncoderSettings::Codec::AVC444:
            negotiatedMode = supportsAvc444 ? EncodingMode::AVC444 : EncodingMode::AVC420;
            break;
        case VideoEncoderSettings::Codec::AVC420:
            negotiatedMode = EncodingMode::AVC420;
            break;
        case VideoEncoderSettings::Codec::RemoteFX:
            break;
        }
    } else if (!supportsProgresive) {
        qCWarning(KRDP) << "Client advertised no usable graphics capability sets";
        d->session->close(RdpConnection::CloseReason::VideoInitFailed);
        return CHANNEL_RC_INITIALIZATION_ERROR;
    }

    QMetaObject::invokeMethod(
        this,
        [this, negotiatedMode, readvertised]() {
            if (readvertised) {
                // The client dropped every frame it had, including the H.264 key frame.
                // setActiveEncodingMode() skips an unchanged mode, so force a new encoder:
                // a fresh encoder starts with a key frame, without one the client stays black.
                d->activeEncodingMode.reset();
            }
            setActiveEncodingMode(negotiatedMode);
        },
        Qt::BlockingQueuedConnection); // RDP callbacks are on the connection thread, VideoStream operates on the main thread
    qCDebug(KRDP) << "Selected encoding mode:" << encodingModeName(negotiatedMode);

    qCDebug(KRDP) << "Selected caps:" << capVersionToString(maxVersion->version);

    RDPGFX_CAPS_CONFIRM_PDU capsConfirmPdu;
    capsConfirmPdu.capsSet = &(maxVersion->capSet);
    const UINT status = d->gfxContext->CapsConfirm(d->gfxContext.get(), &capsConfirmPdu);
    if (status != CHANNEL_RC_OK) {
        qCWarning(KRDP) << "CapsConfirm failed" << status;
        return status;
    }

    if (!readvertised) {
        // On a reconnect mstsc resets its GFX channel and advertises its caps again 50 to
        // 100 ms after our first CapsConfirm. Anything we send in between (ResetGraphics,
        // CreateSurface, a frame) makes it drop the connection with a protocol error. The
        // KPipeWire encoder never had a frame ready that soon; FreeRDP with NVENC does.
        static const clk::milliseconds delay = qEnvironmentVariableIsSet("KRDP_FIRST_FRAME_DELAY_MS")
            ? clk::milliseconds(qEnvironmentVariableIntValue("KRDP_FIRST_FRAME_DELAY_MS"))
            : FirstFrameDelay;
        d->firstFrameNotBefore = clk::steady_clock::now() + delay;
    }
    d->capsConfirmed = true;

    return CHANNEL_RC_OK;
}

void VideoStream::Private::logLatencyStats(clk::steady_clock::time_point now)
{
    const auto window = now - latency.windowStart;
    if (window < clk::seconds(5)) {
        return;
    }
    const auto ms = [](clk::microseconds us) {
        return QString::number(us.count() / 1000.0, 'f', 1);
    };
    const double seconds = clk::duration<double>(window).count();
    qCDebug(KRDP).noquote() << "Video latency:" << QString::number(latency.frames / seconds, 'f', 1) << "fps, encode avg"
                            << ms(latency.frames ? latency.encodeTotal / latency.frames : clk::microseconds(0)) << "ms max" << ms(latency.encodeMax)
                            << "ms, send to ack avg" << ms(latency.acks ? latency.ackTotal / latency.acks : clk::microseconds(0)) << "ms max"
                            << ms(latency.ackMax) << "ms, in flight" << pendingFrames.size() << "client queue" << latency.queueDepth;
    if (latency.bytesTotal > 0) {
        QString bySize;
        static constexpr const char *sizeNames[] = {"<64k", "<256k", ">=256k"};
        for (size_t i = 0; i < latency.sizeAcks.size(); ++i) {
            if (latency.sizeAcks[i] > 0) {
                bySize += QStringLiteral(" %1: %2x %3ms").arg(QLatin1StringView(sizeNames[i])).arg(latency.sizeAcks[i]).arg(ms(latency.sizeAckTotal[i] / latency.sizeAcks[i]));
            }
        }
        qCDebug(KRDP).noquote() << "Video bytes: avg" << (latency.frames ? latency.bytesTotal / latency.frames / 1024 : 0) << "kB max" << latency.bytesMax / 1024
                                << "kB," << QString::number(latency.bytesTotal * 8.0 / seconds / 1e6, 'f', 1) << "Mbit/s; ack by size" << bySize;
    }
    latency = LatencyStats{};
    latency.windowStart = now;
}

uint32_t VideoStream::onFrameAcknowledge(const RDPGFX_FRAME_ACKNOWLEDGE_PDU *frameAcknowledge)
{
    auto id = frameAcknowledge->frameId;

    std::lock_guard lock(d->pendingFramesMutex);

    auto itr = d->pendingFrames.constFind(id);
    if (itr == d->pendingFrames.cend()) {
        qCWarning(KRDP) << "Got frame acknowledge for an unknown frame";
        return CHANNEL_RC_OK;
    }

    if (KRDP().isDebugEnabled()) {
        const auto now = clk::steady_clock::now();
        const auto ackTime = clk::duration_cast<clk::microseconds>(now - itr->sent);
        d->latency.acks++;
        d->latency.ackTotal += ackTime;
        d->latency.ackMax = std::max(d->latency.ackMax, ackTime);
        d->latency.queueDepth = frameAcknowledge->queueDepth;
        if (itr->bytes > 0) {
            const size_t bucket = itr->bytes < 64 * 1024 ? 0 : itr->bytes < 256 * 1024 ? 1 : 2;
            d->latency.sizeAcks[bucket]++;
            d->latency.sizeAckTotal[bucket] += ackTime;
        }
        static const bool trace = qEnvironmentVariableIntValue("KRDP_FRAME_TRACE") != 0;
        if (trace) {
            qCDebug(KRDP).noquote() << "Frame trace:" << id << "bytes" << itr->bytes << "ack us" << ackTime.count() << "in flight" << itr->inFlight;
        }
        d->logLatencyStats(now);
    }

    d->pendingFrames.erase(itr);

    return CHANNEL_RC_OK;
}

void VideoStream::onPacketReceived(const PipeWireEncodedStream::Packet &data)
{
    d->surface->onPacketReceived(data);
}

void VideoStream::onFrameReceived(const PipeWireFrame &data)
{
    d->surface->onFrameReceived(data);
}

bool VideoStream::openChannel()
{
    if (!d->gfxContext) {
        return false;
    }
    if (d->channelOpen) {
        return true;
    }

    if (!d->gfxContext->Open(d->gfxContext.get())) {
        qCWarning(KRDP) << "Failed to open RDPGFX dynamic channel";
        return false;
    }

    d->channelOpen = true;
    return true;
}

void VideoStream::destroySurface()
{
    auto &surface = d->surface->surface;
    if (surface.id == 0) {
        return;
    }

    if (d->gfxContext && surface.codecContextId != 0) {
        RDPGFX_DELETE_ENCODING_CONTEXT_PDU deleteEncodingContextPdu = {};
        deleteEncodingContextPdu.surfaceId = surface.id;
        deleteEncodingContextPdu.codecContextId = surface.codecContextId;
        const UINT status = d->gfxContext->DeleteEncodingContext(d->gfxContext.get(), &deleteEncodingContextPdu);
        if (status != CHANNEL_RC_OK && status != CHANNEL_RC_NOT_INITIALIZED) {
            qCWarning(KRDP) << "DeleteEncodingContext failed" << status;
        }
    }

    if (d->gfxContext) {
        RDPGFX_DELETE_SURFACE_PDU deleteSurfacePdu = {};
        deleteSurfacePdu.surfaceId = surface.id;
        const UINT status = d->gfxContext->DeleteSurface(d->gfxContext.get(), &deleteSurfacePdu);
        if (status != CHANNEL_RC_OK && status != CHANNEL_RC_NOT_INITIALIZED) {
            qCWarning(KRDP) << "DeleteSurface failed" << status;
        }
    }

    if (d->progressive) {
        progressive_delete_surface_context(d->progressive.get(), surface.id);
    }

    surface = Surface{};
}

void VideoStream::forgetSurface()
{
    auto &surface = d->surface->surface;
    if (surface.id != 0 && d->progressive) {
        progressive_delete_surface_context(d->progressive.get(), surface.id);
    }
    surface = Surface{};
}

void VideoStream::performReset(QSize newSize)
{
    auto &surface = d->surface->surface;
    if (!d->gfxContext) {
        auto settings = d->session->rdpPeerContext()->settings;
        freerdp_settings_set_uint32(settings, FreeRDP_DesktopWidth, newSize.width());
        freerdp_settings_set_uint32(settings, FreeRDP_DesktopHeight, newSize.height());
        d->session->rdpPeerContext()->update->DesktopResize(d->session->rdpPeerContext());
        surface.size = newSize;
        return;
    }

    destroySurface();

    RDPGFX_RESET_GRAPHICS_PDU resetGraphicsPdu;
    resetGraphicsPdu.width = newSize.width();
    resetGraphicsPdu.height = newSize.height();
    resetGraphicsPdu.monitorCount = 1;

    MONITOR_DEF monitor = {};
    monitor.left = 0;
    monitor.right = newSize.width();
    monitor.top = 0;
    monitor.bottom = newSize.height();
    monitor.flags = MONITOR_PRIMARY;
    resetGraphicsPdu.monitorDefArray = &monitor;
    qCDebug(KRDP) << "ResetGraphics" << newSize << "next surface" << d->nextSurfaceId << "frameId" << d->frameId;
    UINT status = d->gfxContext->ResetGraphics(d->gfxContext.get(), &resetGraphicsPdu);
    if (status != CHANNEL_RC_OK) {
        qCWarning(KRDP) << "ResetGraphics failed" << status << "for size" << newSize;
        return;
    }

    RDPGFX_CREATE_SURFACE_PDU createSurfacePdu;
    createSurfacePdu.width = newSize.width();
    createSurfacePdu.height = newSize.height();
    const uint16_t surfaceId = d->nextSurfaceId++;
    createSurfacePdu.surfaceId = surfaceId;
    createSurfacePdu.pixelFormat = GFX_PIXEL_FORMAT_XRGB_8888;
    status = d->gfxContext->CreateSurface(d->gfxContext.get(), &createSurfacePdu);
    if (status != CHANNEL_RC_OK) {
        qCWarning(KRDP) << "CreateSurface failed" << status << "surface" << surfaceId << "size" << newSize;
        return;
    }

    if (usesFreeRdpH264(d->activeEncodingMode)) {
        // A new surface starts black on the client. FreeRDP only lists the blocks that
        // changed since its previous frame, so a reused encoder would leave every
        // unchanged block black. A fresh encoder lists the whole frame first.
        d->h264Recreate = true;
        d->gpuKeyFrameNeeded = true;
    }

    surface = Surface{
        .id = surfaceId,
        .codecContextId = d->activeEncodingMode == EncodingMode::Progressive ? ProgressiveCodecContextId : 0,
        .size = newSize,
    };

    if (d->activeEncodingMode == EncodingMode::Progressive) {
        if (progressive_create_surface_context(d->progressive.get(), surfaceId, newSize.width(), newSize.height()) < 0) {
            qCWarning(KRDP) << "Failed to create progressive surface context";
            destroySurface();
            return;
        }
    }

    RDPGFX_MAP_SURFACE_TO_OUTPUT_PDU mapSurfaceToOutputPdu;
    mapSurfaceToOutputPdu.outputOriginX = 0;
    mapSurfaceToOutputPdu.outputOriginY = 0;
    mapSurfaceToOutputPdu.surfaceId = surfaceId;
    status = d->gfxContext->MapSurfaceToOutput(d->gfxContext.get(), &mapSurfaceToOutputPdu);
    if (status != CHANNEL_RC_OK) {
        qCWarning(KRDP) << "MapSurfaceToOutput failed" << status << "surface" << surfaceId;
        destroySurface();
    }
}

double VideoStream::effectiveProducerFps()
{
    // Producer rate = frames entering krdp from the source/encoder callbacks. It is measured
    // upstream of the send window, so sizing the window from it cannot feed back into itself
    // (unlike client-decoded FPS). Invariant: producedFrames is written from frame callbacks;
    // the smoothing state is touched only here (updateInFlightWindow() is the sole caller, one
    // rttChanged connection, so no lock is needed). All returns are clamped to a bootstrap/
    // stop-and-wait floor (MinimumWindowFrameRate), and to the requested rate when it is above
    // that floor (so a very low requested rate yields the floor, not less).
    const double requested = d->requestedFrameRate.load();
    const double ceiling = std::max(MinimumWindowFrameRate, requested);
    const auto clampFps = [&](double f) {
        return std::clamp(f, MinimumWindowFrameRate, ceiling);
    };

    const auto now = clk::steady_clock::now();
    const uint64_t total = d->producedFrames.load(std::memory_order_relaxed);
    if (d->lastProducerRateUpdate == clk::steady_clock::time_point{}) {
        d->lastProducerRateUpdate = now;
        d->lastProducedFrames = total;
        // Optimistic bootstrap: seed from the requested rate so the initial full-screen
        // burst gets a usable window immediately; the EWMA converges down to the measured
        // producer rate as frames arrive.
        d->smoothedProducerFps = clampFps(requested);
        return d->smoothedProducerFps;
    }
    const double elapsedSec = clk::duration<double>(now - d->lastProducerRateUpdate).count();
    const uint64_t delta = total - d->lastProducedFrames;
    d->lastProducerRateUpdate = now;
    d->lastProducedFrames = total;
    if (elapsedSec <= 0.0 || delta == 0) {
        return clampFps(d->smoothedProducerFps); // idle: hold last active rate, do not shrink
    }
    const double instantFps = double(delta) / elapsedSec;
    d->smoothedProducerFps = d->smoothedProducerFps * (1.0 - ProducerFpsEwmaAlpha) + instantFps * ProducerFpsEwmaAlpha;
    return clampFps(d->smoothedProducerFps);
}

void VideoStream::updateInFlightWindow()
{
    // Size the in-flight window from the bandwidth-delay product: how many produced frames fit
    // within the round-trip time. averageRTT() is already windowed; we smooth it a second time
    // (EWMA) so transient RTT spikes do not immediately inflate the submission window, and floor
    // the smoothed value at the session-minimum RTT so the window never drops below the path's
    // BDP (which would collapse high-RTT throughput). Producer FPS is measured from frames
    // entering krdp; averageRTT() is published via atomics; both are safe to read here. The RTT
    // smoothing state is updated only from here, which the rttChanged connection invokes serially,
    // so no lock is needed. Falls back to the fixed floor until a valid RTT is known.
    const double fps = effectiveProducerFps();
    const double rttMs = clk::duration<double, std::milli>(d->session->networkDetection()->averageRTT()).count();
    qsizetype window = MaximumInFlightFrames;
    if (rttMs >= MinimumValidRttMs && rttMs < MaximumValidRttMs) {
        if (!d->hasBaseRtt || rttMs < d->baseRttMs) {
            d->baseRttMs = rttMs;
            d->hasBaseRtt = true;
        }
        if (!d->hasSmoothedRtt) {
            d->smoothedRttMs = rttMs;
            d->hasSmoothedRtt = true;
        } else {
            d->smoothedRttMs = (1.0 - RttEwmaAlpha) * d->smoothedRttMs + RttEwmaAlpha * rttMs;
        }
        const double effectiveRttMs = std::max(d->baseRttMs, d->smoothedRttMs);
        const double rttSec = effectiveRttMs / 1000.0;
        const qsizetype bdp = qsizetype(std::ceil(fps * rttSec * InFlightGain));
        const qsizetype cap = std::max<qsizetype>(MaximumInFlightFrames, qsizetype(std::ceil(fps * LatencyBudgetSec)));
        window = std::clamp(bdp, qsizetype(MaximumInFlightFrames), cap);
    }
    d->maxInFlight.store(window);
}

void VideoStream::updateAdaptiveQuality()
{
    // Not for the FreeRDP encoders: a new QP needs a new encoder and a key frame, and
    // adaptive quality changes it every few seconds.
    if (!d->adaptiveQuality || d->activeEncodingMode != EncodingMode::H264) {
        return;
    }

    const quint32 goodputKbit = d->session->networkDetection()->bandwidth();
    if (goodputKbit == 0) {
        return;
    }

    const auto now = clk::system_clock::now();
    if (now - d->lastQualityUpdate < QualityUpdateInterval) {
        return;
    }

    const int fps = std::max(1, d->requestedFrameRate.load());
    const double pixels = double(d->surface->size.width()) * double(d->surface->size.height());
    if (pixels <= 0.0) {
        return;
    }
    const double fullKbit = fullQualityKbit(pixels);

    const int hi = std::max(int(d->qualityCap), MinAdaptiveQuality);
    int target = std::clamp(int(std::lround(goodputKbit / fullKbit * 100.0)), MinAdaptiveQuality, hi);

    const auto avg = clk::duration_cast<clk::milliseconds>(d->session->networkDetection()->averageRTT());
    const auto min = clk::duration_cast<clk::milliseconds>(d->session->networkDetection()->minimumRTT());
    const bool congested = min.count() > 0 && avg.count() > min.count() * 3 / 2;
    if (congested) {
        target = std::clamp(int(d->quality) - QualityStepDown, MinAdaptiveQuality, target);
    }

    int next = d->quality;
    if (target < next) {
        next = std::max(target, next - QualityStepDown);
    } else if (target > next && !congested) {
        next = std::min(target, next + QualityStepUp);
    }

    if (next == int(d->quality)) {
        return;
    }

    d->lastQualityUpdate = now;
    d->quality = quint8(next);
    applyQuality();
    qCDebug(KRDP) << "Adaptive quality ->" << d->quality << "(target" << target << "cap" << d->qualityCap << "goodput" << goodputKbit << "kbit/s, fps" << fps
                  << (congested ? ", congested" : "") << ")";
}

bool VideoStream::hasInFlightCapacity() const
{
    std::lock_guard lock(d->pendingFramesMutex);
    return d->pendingFrames.size() < d->maxInFlight.load();
}

void VideoStream::sendFrame(const VideoFrame &frame)
{
    auto peer = d->session->rdpPeer();
    if (peer->IsWriteBlocked && peer->IsWriteBlocked(peer)) {
        return;
    }

    if (!d->activeEncodingMode) {
        return;
    }

    std::lock_guard frameLock(d->surface->frameMutex);
    const quint64 resetGeneration = d->surface->resetGeneration;
    if (!d->gfxContext || !d->capsConfirmed) {
        return;
    }

    {
        std::lock_guard lock(d->expectedSizeMutex);
        if (d->expectedFrameSize.isValid()) {
            if (frame.size == d->expectedFrameSize) {
                d->expectedFrameSize = QSize();
            } else if (clk::steady_clock::now() < d->expectedSizeDeadline) {
                return; // the monitor is still being resized
            } else {
                qCDebug(KRDP) << "Monitor did not reach" << d->expectedFrameSize << "in time; streaming" << frame.size;
                d->expectedFrameSize = QSize();
            }
        }
    }

    if (d->surface->pendingReset) {
        d->surface->pendingReset = false;
        performReset(frame.size);
    }
    if (d->surface->surface.size != frame.size) {
        performReset(frame.size);
    }

    const auto frameId = d->frameId++;
    {
        std::lock_guard lock(d->pendingFramesMutex);
        d->pendingFrames.insert(frameId, Private::PendingFrame{.sent = clk::steady_clock::now()});
    }

    bool submitted = false;
    if (d->activeEncodingMode == EncodingMode::H264) {
        submitted = d->surface->sendFrameH264(d->gfxContext.get(), frameId, frame, resetGeneration);
    } else if (usesFreeRdpH264(d->activeEncodingMode)) {
        if (d->ensureH264(frame.size)) {
            const bool avc444 = d->activeEncodingMode == EncodingMode::AVC444;
            VideoStreamSurface::AvcResult result = VideoStreamSurface::AvcResult::Failed;
            if (avc444 && frame.avc444 && frame.avc444->lumaFrame) {
                const quint32 qp = d->h264TargetQp.load();
                if (d->gpuEncoder.ensure(frame.avc444->lumaFrame.get(), qp, d->requestedFrameRate.load(), d->gpuSpeed())) {
                    if (d->gpuKeyFrameNeeded.exchange(false)) {
                        d->gpuEncoder.requestKeyFrame();
                    }
                    const auto &picture = *frame.avc444;
                    result = d->surface->sendFrameAvcGpu(
                        d->gfxContext.get(),
                        [this, &picture](bool chroma, QByteArray &out) {
                            return d->gpuEncoder.encode(chroma ? picture.chromaFrame.get() : picture.lumaFrame.get(), out);
                        },
                        d->gpuEncoder.generation(),
                        qp,
                        frameId,
                        frame,
                        resetGeneration);
                }
                if (result == VideoStreamSurface::AvcResult::Failed) {
                    // Go back to the CPU path for the next frames; this frame is lost.
                    qCWarning(KRDP) << "GPU encode failed; using the CPU path from now on";
                    d->surface->gpuFailed = true;
                    result = VideoStreamSurface::AvcResult::Unchanged;
                }
            } else {
                result = d->surface->sendFrameAvc(d->gfxContext.get(), d->h264.get(), avc444, frameId, frame, resetGeneration);
            }
            submitted = result == VideoStreamSurface::AvcResult::Sent;
            if (result == VideoStreamSurface::AvcResult::Failed && encoderSettings().encoder == VideoEncoderSettings::Encoder::Auto && !d->nvencFailed) {
                qCWarning(KRDP) << "H.264 encoding failed with NVENC; switching to libx264";
                d->nvencFailed = true;
                d->h264Recreate = true;
            }
        }
    } else {
        submitted = d->surface->sendFrameProgressive(d->gfxContext.get(), d->progressive.get(), frameId, frame, resetGeneration);
    }

    if (!submitted) {
        std::lock_guard lock(d->pendingFramesMutex);
        d->pendingFrames.remove(frameId);
    } else if (KRDP().isDebugEnabled()) {
        std::lock_guard lock(d->pendingFramesMutex);
        const auto now = clk::steady_clock::now();
        if (auto itr = d->pendingFrames.find(frameId); itr != d->pendingFrames.end()) {
            const auto encodeTime = clk::duration_cast<clk::microseconds>(now - itr->sent);
            d->latency.frames++;
            d->latency.encodeTotal += encodeTime;
            d->latency.encodeMax = std::max(d->latency.encodeMax, encodeTime);
            // Measure the acknowledgement from when the frame went out, not from the encode start.
            itr->sent = now;
            if (usesFreeRdpH264(d->activeEncodingMode)) {
                itr->bytes = d->surface->lastFrameBytes;
                d->latency.bytesTotal += itr->bytes;
                d->latency.bytesMax = std::max(d->latency.bytesMax, itr->bytes);
            }
            itr->inFlight = d->pendingFrames.size();
        }
    }

    QMetaObject::invokeMethod(this, &VideoStream::updateBackpressure, Qt::QueuedConnection);
}

void VideoStream::updateBackpressure()
{
    if (!d->surface->encodedStream) {
        return;
    }

    qsizetype pendingSendQueueCount = 0;
    {
        std::lock_guard lock(d->frameQueueMutex);
        pendingSendQueueCount = d->frameQueue.count();
    }

    const bool encodingPaused = d->surface->encodedStream->encoderPaused();

    if (!encodingPaused && pendingSendQueueCount > HighQueueCount) {
        qCDebug(KRDP) << "Encoder backpressure activated" << pendingSendQueueCount;
        d->surface->encodedStream->setEncoderPaused(true);
    } else if (encodingPaused && pendingSendQueueCount < LowQueueCount) {
        qCDebug(KRDP) << "Encoder backpressure released" << pendingSendQueueCount;
        d->surface->encodedStream->setEncoderPaused(false);
    }
}
}

#include "moc_VideoStream.cpp"
