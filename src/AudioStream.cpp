// SPDX-FileCopyrightText: 2026 Nick Haghiri <nick@haghiri.net>
//
// This file is roughly based on grd-rdp-audio-output-stream.c and
// grd-rdp-dvc-audio-playback.c from Gnome Remote Desktop, which are:
//
// SPDX-FileCopyrightText: 2021-2022 Pascal Nowack
//
// SPDX-License-Identifier: GPL-2.0-or-later

#include "AudioStream.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <vector>

#include <QScopeGuard>

#include "PeerContext_p.h"
#include "PipeWireStreamUtils_p.h"
#include "RdpConnection.h"
#include "krdp_logging.h"

#include <freerdp/codec/audio.h>
#include <freerdp/codec/dsp.h>
#include <freerdp/freerdp.h>
#include <freerdp/server/rdpsnd.h>
#include <freerdp/settings.h>
#include <winpr/synch.h>

#include <pipewire/extensions/metadata.h>
#include <pipewire/pipewire.h>
#include <spa/param/audio/raw.h>
#include <spa/param/format.h>
#include <spa/pod/builder.h>
#include <spa/utils/dict.h>

namespace KRdp
{

static constexpr size_t s_maxBufferedBytes = 72000 * s_blockAlign;
static constexpr int s_aacFrameSamples = 1024;
// mstsc decodes AAC at 44100 Hz whatever rate is negotiated, so that's the only AAC rate offered
static constexpr uint32_t s_aacSampleRate = 44100;
static constexpr uint32_t s_maxRenderLatencyMs = 300;
// Opus needs a longer minimum: see drainCaptureRingIntoPending().
static constexpr auto s_silenceTimeoutOpus = std::chrono::seconds(10);
static constexpr auto s_latencyLogInterval = std::chrono::seconds(5);

static AudioSettings s_settings;

AudioSettings AudioSettings::fromStrings(const QString &codec, int bitrateKbit, int idleTimeoutSeconds)
{
    AudioSettings settings;
    const QString c = codec.trimmed().toLower();
    if (c == QLatin1String("pcm")) {
        settings.codec = Codec::Pcm;
    } else if (c == QLatin1String("aac")) {
        settings.codec = Codec::Aac;
    } else if (c == QLatin1String("opus")) {
        settings.codec = Codec::Opus;
    } else {
        settings.codec = Codec::Auto;
    }
    settings.bitrateKbit = std::clamp(bitrateKbit, 32, 320);
    settings.idleTimeout = std::chrono::seconds(std::max(0, idleTimeoutSeconds));
    return settings;
}

void AudioStream::setSettings(const AudioSettings &settings)
{
    s_settings = settings;
}

AudioSettings AudioStream::settings()
{
    return s_settings;
}

class AudioStream::Private
{
public:
    RdpConnection *connection = nullptr;

    RdpsndServerContext *rdpsnd = nullptr;
    bool initializeTried = false;
    uint32_t sampleRate = 48000;

    enum class Codec {
        Pcm,
        Aac,
        Opus
    } codec = Codec::Pcm;
    std::unique_ptr<FREERDP_DSP_CONTEXT, decltype(&freerdp_dsp_context_free)> encoder{nullptr, freerdp_dsp_context_free};
    struct BlockInfo {
        uint16_t renderLatencyMs = 0;
        std::chrono::steady_clock::time_point setAt{};
    };
    std::mutex blockInfosMutex;
    std::array<BlockInfo, 256> blockInfos{};
    std::atomic_bool activationPending = false;
    std::mutex clientFormatsMutex;
    std::vector<AUDIO_FORMAT> clientFormats;
    UINT16 clientVersion = 0;

    std::unique_ptr<PipeWireLoopConnection> pipewire;
    pw_stream *stream = nullptr;
    spa_hook streamListener{};
    bool holdsRemoteSink = false;

    std::atomic_bool running = false;
    AudioByteQueue captureRing{s_maxBufferedBytes};
    bool captureDropWarned = false;
    bool sendFailedWarned = false;
    std::vector<uint8_t> pending;
    uint64_t framesSent = 0;
    std::chrono::steady_clock::time_point lastSound{};
    bool idle = false;
    std::chrono::steady_clock::time_point lastLatencyLog{};
    UINT16 clientFormatIndex = 0;
    HANDLE wakeEvent = nullptr;

    void onActivated();
    void startCapture();
    bool connectCapture();
    void stopCapture();
    void onCaptureData(const uint8_t *data, size_t bytes);
    void drainCaptureRingIntoPending();
    int frameSamples() const;
    QByteArray encodeFrame(const uint8_t *pcm);
    void sendPackets();
    uint32_t currentRenderLatencyMs();
    void clearBlockInfos();

    static void rdpsndActivated(RdpsndServerContext *context);
    static UINT rdpsndConfirmBlock(RdpsndServerContext *context, BYTE confirmBlockNum, UINT16 wTimestamp);

private:
    static void streamParamChanged(void *data, uint32_t id, const spa_pod *param);
    static void streamProcess(void *data);

    static constexpr pw_stream_events s_streamEvents = {
        .version = PW_VERSION_STREAM_EVENTS,
        .param_changed = streamParamChanged,
        .process = streamProcess,
    };
};

namespace
{

constexpr char s_remoteSinkName[] = "krdp_remote_sink";
constexpr char s_remoteSinkDefaultJson[] = R"({"name":"krdp_remote_sink"})";
void makeRemoteSinkDefault(PipeWireLoopConnection &pipewire)
{
    struct Binder {
        pw_registry *registry = nullptr;
        spa_hook registryListener{};
        pw_metadata *metadata = nullptr;
    } binder;
    static constexpr pw_registry_events s_registryEvents = {
        .version = PW_VERSION_REGISTRY_EVENTS,
        .global =
            [](void *data, uint32_t id, uint32_t, const char *type, uint32_t, const spa_dict *props) {
                auto *binder = static_cast<Binder *>(data);
                const char *name = props ? spa_dict_lookup(props, PW_KEY_METADATA_NAME) : nullptr;
                if (!binder->metadata && strcmp(type, PW_TYPE_INTERFACE_Metadata) == 0 && name && strcmp(name, "default") == 0) {
                    binder->metadata = static_cast<pw_metadata *>(pw_registry_bind(binder->registry, id, PW_TYPE_INTERFACE_Metadata, PW_VERSION_METADATA, 0));
                }
            },
    };

    binder.registry = pw_core_get_registry(pipewire.core(), PW_VERSION_REGISTRY, 0);
    if (binder.registry) {
        pw_registry_add_listener(binder.registry, &binder.registryListener, &s_registryEvents, &binder);
        pipewire.roundtrip();
    }
    if (binder.metadata) {
        pw_metadata_set_property(binder.metadata, PW_ID_CORE, "default.configured.audio.sink", "Spa:String:JSON", s_remoteSinkDefaultJson);
        pipewire.roundtrip();
        pw_proxy_destroy(reinterpret_cast<pw_proxy *>(binder.metadata));
        qCDebug(KRDP) << "Audio: made the Remote Desktop sink the default output";
    } else {
        qCWarning(KRDP) << "Audio: could not make the Remote Desktop sink the default output";
    }
    if (binder.registry) {
        spa_hook_remove(&binder.registryListener);
        pw_proxy_destroy(reinterpret_cast<pw_proxy *>(binder.registry));
    }
}

class RemoteSink
{
public:
    RemoteSink()
    {
        if (!m_pipewire.isValid()) {
            qCWarning(KRDP) << "Audio: could not connect to PipeWire for the Remote Desktop sink";
            return;
        }

        pw_thread_loop_lock(m_pipewire.loop());
        pw_properties *props = pw_properties_new(PW_KEY_FACTORY_NAME,
                                                 "support.null-audio-sink",
                                                 PW_KEY_MEDIA_CLASS,
                                                 "Audio/Sink",
                                                 PW_KEY_NODE_NAME,
                                                 s_remoteSinkName,
                                                 PW_KEY_NODE_DESCRIPTION,
                                                 "Remote Desktop",
                                                 "monitor.channel-volumes",
                                                 "true",
                                                 SPA_KEY_AUDIO_RATE,
                                                 "48000",
                                                 SPA_KEY_AUDIO_POSITION,
                                                 "FL,FR",
                                                 nullptr);
        m_node = props ? static_cast<pw_proxy *>(pw_core_create_object(m_pipewire.core(), "adapter", PW_TYPE_INTERFACE_Node, PW_VERSION_NODE, &props->dict, 0))
                       : nullptr;
        pw_properties_free(props);
        if (m_node && m_pipewire.roundtrip()) {
            makeRemoteSinkDefault(m_pipewire);
        } else {
            qCWarning(KRDP) << "Audio: could not create the Remote Desktop sink";
        }
        pw_thread_loop_unlock(m_pipewire.loop());
    }

    ~RemoteSink()
    {
        if (!m_node) {
            return;
        }
        pw_thread_loop_lock(m_pipewire.loop());
        pw_proxy_destroy(m_node);
        pw_thread_loop_unlock(m_pipewire.loop());
    }

    RemoteSink(const RemoteSink &) = delete;
    RemoteSink &operator=(const RemoteSink &) = delete;

private:
    PipeWireLoopConnection m_pipewire{"krdp-remote-sink"};
    pw_proxy *m_node = nullptr;
};

std::mutex s_remoteSinkMutex;
std::unique_ptr<RemoteSink> s_remoteSink;
int s_remoteSinkUsers = 0;

void acquireRemoteSink()
{
    std::lock_guard lock(s_remoteSinkMutex);
    if (s_remoteSinkUsers++ == 0) {
        s_remoteSink = std::make_unique<RemoteSink>();
    }
}

void releaseRemoteSink()
{
    std::lock_guard lock(s_remoteSinkMutex);
    if (--s_remoteSinkUsers == 0) {
        s_remoteSink.reset();
    }
}

}

void AudioStream::Private::streamParamChanged(void *data, uint32_t id, const spa_pod *param)
{
    respondParamBuffers(static_cast<Private *>(data)->stream, id, param);
}

void AudioStream::Private::streamProcess(void *data)
{
    auto *d = static_cast<Private *>(data);
    pw_buffer *buffer = pw_stream_dequeue_buffer(d->stream);
    if (!buffer) {
        return;
    }
    for (pw_buffer *next = pw_stream_dequeue_buffer(d->stream); next; next = pw_stream_dequeue_buffer(d->stream)) {
        pw_stream_queue_buffer(d->stream, buffer);
        buffer = next;
    }
    const spa_data &block = buffer->buffer->datas[0];
    if (block.data && block.chunk->size > 0) {
        d->onCaptureData(static_cast<const uint8_t *>(block.data) + block.chunk->offset, block.chunk->size);
    }
    pw_stream_queue_buffer(d->stream, buffer);
}

void AudioStream::Private::rdpsndActivated(RdpsndServerContext *context)
{
    auto *d = static_cast<Private *>(context->data);
    {
        // the rdpsnd thread frees and reallocates client_formats if the client sends another Formats PDU
        std::lock_guard lock(d->clientFormatsMutex);
        d->clientFormats.assign(context->client_formats, context->client_formats + context->num_client_formats);
        for (AUDIO_FORMAT &format : d->clientFormats) {
            format.data = nullptr;
            format.cbSize = 0;
        }
        d->clientVersion = context->clientVersion;
    }
    d->activationPending = true;
    SetEvent(d->wakeEvent);
}

UINT AudioStream::Private::rdpsndConfirmBlock(RdpsndServerContext *context, BYTE confirmBlockNum, UINT16 wTimestamp)
{
    auto *d = static_cast<Private *>(context->data);
    if (wTimestamp > 0) {
        std::lock_guard lock(d->blockInfosMutex);
        d->blockInfos[confirmBlockNum] = {wTimestamp, std::chrono::steady_clock::now()};
    }
    return CHANNEL_RC_OK;
}

AudioStream::AudioStream(RdpConnection *connection)
    : d(std::make_unique<Private>())
{
    d->connection = connection;
    d->wakeEvent = CreateEvent(nullptr, TRUE, FALSE, nullptr);
}

AudioStream::~AudioStream()
{
    close();
    if (d->wakeEvent) {
        CloseHandle(d->wakeEvent);
        d->wakeEvent = nullptr;
    }
}

void *AudioStream::wakeHandle() const
{
    return d->wakeEvent;
}

bool AudioStream::initialize()
{
    if (d->rdpsnd) {
        return true;
    }
    if (d->initializeTried) {
        return false;
    }
    d->initializeTried = true;

    auto peerContext = reinterpret_cast<PeerContext *>(d->connection->rdpPeerContext());
    if (!peerContext || !peerContext->virtualChannelManager) {
        return false;
    }

    d->rdpsnd = rdpsnd_server_context_new(peerContext->virtualChannelManager);
    if (!d->rdpsnd) {
        qCWarning(KRDP) << "Failed to create rdpsnd server context";
        return false;
    }

    d->rdpsnd->data = d.get();
    d->rdpsnd->rdpcontext = d->connection->rdpPeerContext();
    d->rdpsnd->Activated = Private::rdpsndActivated;
    d->rdpsnd->ConfirmBlock = Private::rdpsndConfirmBlock;

    auto cleanup = qScopeGuard([this]() {
        rdpsnd_server_context_free(d->rdpsnd);
        d->rdpsnd = nullptr;
    });

    const AUDIO_FORMAT aac = audioFormat(WAVE_FORMAT_AAC_MS, s_aacSampleRate, s_settings.bitrateKbit * 1000 / 8);
    const AUDIO_FORMAT opus = audioFormat(WAVE_FORMAT_OPUS, 48000, s_settings.bitrateKbit * 1000 / 8);
    const bool haveAac = freerdp_dsp_supports_format(&aac, TRUE);
    // FreeRDP's non-FFmpeg Opus encoder over-reports each packet's length by 4x (dsp.c, freerdp_dsp_encode_opus)
    const bool haveOpus = strstr(freerdp_get_build_config(), "WITH_DSP_FFMPEG=ON") && freerdp_dsp_supports_format(&opus, TRUE);
    const uint16_t numFormats = uint16_t(2 + (haveAac ? 1 : 0) + (haveOpus ? 1 : 0));

    auto *formats = static_cast<AUDIO_FORMAT *>(calloc(numFormats, sizeof(AUDIO_FORMAT)));
    if (!formats) {
        return false;
    }
    // The client takes the first format in this list that it supports.
    uint16_t idx = 0;
    const auto codec = s_settings.codec;
    if (codec == AudioSettings::Codec::Pcm) {
        formats[idx++] = audioFormat(WAVE_FORMAT_PCM, 48000);
    }
    if (haveOpus && codec == AudioSettings::Codec::Opus) {
        formats[idx++] = opus;
    }
    if (haveAac) {
        formats[idx++] = aac;
    }
    if (haveOpus && codec != AudioSettings::Codec::Opus) {
        formats[idx++] = opus;
    }
    if (codec != AudioSettings::Codec::Pcm) {
        formats[idx++] = audioFormat(WAVE_FORMAT_PCM, 48000);
    }
    formats[idx++] = audioFormat(WAVE_FORMAT_PCM, 44100);
    d->rdpsnd->server_formats = formats;
    d->rdpsnd->num_server_formats = numFormats;

    if (d->rdpsnd->Initialize(d->rdpsnd, TRUE) != CHANNEL_RC_OK) {
        qCWarning(KRDP) << "Failed to initialize rdpsnd channel";
        return false;
    }
    cleanup.dismiss();

    qCDebug(KRDP) << "Audio output (rdpsnd) channel initialized";
    return true;
}

void AudioStream::handleMessages()
{
    if (!d->rdpsnd) {
        return;
    }
    if (d->activationPending.exchange(false)) {
        d->onActivated();
    }
    if (!d->running.load()) {
        return;
    }
    if (KRDP().isDebugEnabled() && !d->idle) {
        const auto now = std::chrono::steady_clock::now();
        if (now - d->lastLatencyLog >= s_latencyLogInterval) {
            d->lastLatencyLog = now;
            qCDebug(KRDP) << "Audio latency: client reports" << d->currentRenderLatencyMs() << "ms, server buffer"
                          << d->pending.size() * 1000 / (s_blockAlign * d->sampleRate) << "ms";
        }
    }
    if (const auto latencyMs = d->currentRenderLatencyMs(); latencyMs > s_maxRenderLatencyMs) {
        qCDebug(KRDP) << "Audio: client latency" << latencyMs << "ms is over" << s_maxRenderLatencyMs << "ms; dropping" << d->pending.size() * 1000 / (s_blockAlign * d->sampleRate) << "ms of buffered audio";
        d->pending.clear();
        d->clearBlockInfos();
    }
    d->drainCaptureRingIntoPending();
    if (!d->idle) {
        d->sendPackets();
    }
}

void AudioStream::Private::onActivated()
{
    if (running.load()) {
        qCWarning(KRDP) << "Audio: ignoring re-activation while already streaming";
        return;
    }

    std::vector<AUDIO_FORMAT> formats;
    UINT16 version;
    {
        std::lock_guard lock(clientFormatsMutex);
        formats = clientFormats;
        version = clientVersion;
    }
    if (formats.empty()) {
        qCWarning(KRDP) << "Audio: client advertised no formats";
        return;
    }
    for (const AUDIO_FORMAT &format : formats) {
        qCDebug(KRDP) << "Audio: client format" << audio_format_get_tag_string(format.wFormatTag) << format.nSamplesPerSec << "Hz" << format.nChannels << "ch"
                      << format.wBitsPerSample << "bit";
    }

    static constexpr UINT16 s_minClientVersionForWave2 = 0x08;
    if (version < s_minClientVersionForWave2) {
        qCWarning(KRDP) << "Audio: client's rdpsnd version" << version << "predates Wave2 PDU support; audio redirection not available";
        return;
    }

    const rdpSettings *settings = connection->rdpPeerContext()->settings;
    if (freerdp_settings_get_bool(settings, FreeRDP_RemoteConsoleAudio)) {
        qCDebug(KRDP) << "Audio: client plays audio on the server; not redirecting";
        return;
    }

    if (!freerdp_settings_get_bool(settings, FreeRDP_AudioPlayback)) {
        qCDebug(KRDP) << "Audio: client asked for no audio playback; not redirecting";
        return;
    }

    for (uint16_t j = 0; j < rdpsnd->num_server_formats; ++j) {
        const UINT16 formatTag = rdpsnd->server_formats[j].wFormatTag;
        for (UINT16 i = 0; i < formats.size(); ++i) {
            if (!audio_format_compatible(&rdpsnd->server_formats[j], &formats[i])) {
                continue;
            }
            codec = formatTag == WAVE_FORMAT_AAC_MS ? Codec::Aac : formatTag == WAVE_FORMAT_OPUS ? Codec::Opus : Codec::Pcm;
            const char *codecName = codec == Codec::Aac ? "AAC" : codec == Codec::Opus ? "Opus" : "PCM";
            if (codec != Codec::Pcm) {
                encoder.reset(freerdp_dsp_context_new(TRUE));
                if (!encoder || !freerdp_dsp_context_reset(encoder.get(), &rdpsnd->server_formats[j], 0)) {
                    qCWarning(KRDP) << "Audio: FreeRDP could not set up the" << codecName << "encoder";
                    continue;
                }
            }
            clientFormatIndex = i;
            sampleRate = rdpsnd->server_formats[j].nSamplesPerSec;
            qCDebug(KRDP) << "Audio: negotiated" << codecName << sampleRate << "Hz stereo; starting capture";
            startCapture();
            return;
        }
    }
    qCWarning(KRDP) << "Audio: no format compatible with the client; no audio redirected";
}

void AudioStream::Private::startCapture()
{
    if (running.load()) {
        return;
    }

    acquireRemoteSink();
    holdsRemoteSink = true;

    pipewire = std::make_unique<PipeWireLoopConnection>("krdp-audio-output");
    if (!pipewire->isValid() || !connectCapture()) {
        qCWarning(KRDP) << "Audio: could not capture the Remote Desktop sink";
        stopCapture();
        return;
    }

    framesSent = 0;
    lastSound = std::chrono::steady_clock::now();
    idle = false;
    pending.clear();
    clearBlockInfos();
    running = true;
}

bool AudioStream::Private::connectCapture()
{
    uint8_t buffer[512];
    spa_pod_builder builder = SPA_POD_BUILDER_INIT(buffer, sizeof(buffer));
    const spa_pod *params[1] = {buildAudioFormatPod(builder, sampleRate)};

    pw_properties *props = pw_properties_new(PW_KEY_MEDIA_TYPE,
                                             "Audio",
                                             PW_KEY_MEDIA_CATEGORY,
                                             "Capture",
                                             PW_KEY_NODE_NAME,
                                             "krdp-audio-output",
                                             PW_KEY_NODE_FORCE_QUANTUM,
                                             "256",
                                             PW_KEY_STREAM_CAPTURE_SINK,
                                             "true",
                                             PW_KEY_TARGET_OBJECT,
                                             s_remoteSinkName,
                                             nullptr);

    pw_thread_loop_lock(pipewire->loop());
    stream = pw_stream_new(pipewire->core(), "krdp-audio-output", props);
    if (stream) {
        pw_stream_add_listener(stream, &streamListener, &s_streamEvents, this);
        const auto flags = static_cast<pw_stream_flags>(PW_STREAM_FLAG_AUTOCONNECT | PW_STREAM_FLAG_RT_PROCESS | PW_STREAM_FLAG_MAP_BUFFERS);
        if (pw_stream_connect(stream, PW_DIRECTION_INPUT, PW_ID_ANY, flags, params, 1) < 0) {
            destroyPwStream(stream, streamListener);
        }
    }
    pw_thread_loop_unlock(pipewire->loop());
    return stream;
}

void AudioStream::Private::onCaptureData(const uint8_t *data, size_t bytes)
{
    logDropOnce(captureRing.write(data, bytes), captureDropWarned, "Audio: capture backlog exceeded buffer; dropping oldest audio");
    if (wakeEvent) {
        SetEvent(wakeEvent);
    }
}

void AudioStream::Private::drainCaptureRingIntoPending()
{
    const std::vector<uint8_t> captured = captureRing.readAll();
    const auto now = std::chrono::steady_clock::now();
    if (std::any_of(captured.begin(), captured.end(), [](uint8_t byte) {
            return byte != 0;
        })) {
        lastSound = now;
        if (idle) {
            idle = false;
            qCDebug(KRDP) << "Audio: sound resumed; streaming to the client again";
        }
    } else if (const auto silenceTimeout = codec == Codec::Opus ? std::max(s_settings.idleTimeout, s_silenceTimeoutOpus) : s_settings.idleTimeout;
               s_settings.idleTimeout.count() > 0 && !idle && now - lastSound >= silenceTimeout) {
        idle = true;
        pending.clear();
        rdpsnd->Close(rdpsnd);
        qCDebug(KRDP) << "Audio: silent for" << silenceTimeout.count() << "s; closed the client's stream until sound returns";
    }
    if (idle) {
        return;
    }
    pending.insert(pending.end(), captured.begin(), captured.end());
    if (pending.size() > s_maxBufferedBytes) {
        pending.erase(pending.begin(), pending.begin() + (pending.size() - s_maxBufferedBytes));
    }
}

uint32_t AudioStream::Private::currentRenderLatencyMs()
{
    std::lock_guard lock(blockInfosMutex);
    uint32_t total = 0;
    uint32_t count = 0;
    const auto now = std::chrono::steady_clock::now();
    for (const auto &info : blockInfos) {
        if (info.renderLatencyMs > 0 && now - info.setAt < std::chrono::seconds(1)) {
            total += info.renderLatencyMs;
            ++count;
        }
    }
    return count ? total / count : 0;
}

void AudioStream::Private::clearBlockInfos()
{
    std::lock_guard lock(blockInfosMutex);
    blockInfos.fill({});
}

int AudioStream::Private::frameSamples() const
{
    return codec == Codec::Aac ? s_aacFrameSamples : int(sampleRate / 50);
}

QByteArray AudioStream::Private::encodeFrame(const uint8_t *pcm)
{
    const size_t frameBytes = size_t(frameSamples()) * s_blockAlign;
    if (codec == Codec::Pcm) {
        return QByteArray(reinterpret_cast<const char *>(pcm), qsizetype(frameBytes));
    }
    const AUDIO_FORMAT source = audioFormat(WAVE_FORMAT_PCM, sampleRate);
    wStream *out = Stream_New(nullptr, frameBytes);
    QByteArray unit;
    if (out && freerdp_dsp_encode(encoder.get(), &source, pcm, frameBytes, out)) {
        unit = QByteArray(reinterpret_cast<const char *>(Stream_Buffer(out)), qsizetype(Stream_GetPosition(out)));
    }
    Stream_Free(out, TRUE);
    return unit;
}

void AudioStream::Private::sendPackets()
{
    const size_t frameBytes = size_t(frameSamples()) * s_blockAlign;
    size_t offset = 0;
    for (; offset + frameBytes <= pending.size(); offset += frameBytes) {
        const QByteArray unit = encodeFrame(pending.data() + offset);
        if (unit.isEmpty()) {
            continue;
        }
        const UINT rc = rdpsnd->SendSamples2(rdpsnd,
                                             clientFormatIndex,
                                             reinterpret_cast<const BYTE *>(unit.constData()),
                                             UINT32(unit.size()),
                                             0,
                                             UINT32(framesSent * 1000ULL / sampleRate));
        if (rc != CHANNEL_RC_OK) {
            if (!sendFailedWarned) {
                qCWarning(KRDP) << "Audio: SendSamples2 failed (" << rc << ")";
                sendFailedWarned = true;
            }
            break;
        }
        framesSent += frameSamples();
    }
    pending.erase(pending.begin(), pending.begin() + offset);
}

void AudioStream::Private::stopCapture()
{
    running = false;
    if (pipewire && pipewire->loop()) {
        pw_thread_loop_lock(pipewire->loop());
        destroyPwStream(stream, streamListener);
        pw_thread_loop_unlock(pipewire->loop());
    }
    pipewire.reset();
    if (holdsRemoteSink) {
        holdsRemoteSink = false;
        releaseRemoteSink();
    }
    captureRing.reset();
    captureDropWarned = false;
    sendFailedWarned = false;
}

void AudioStream::close()
{
    d->stopCapture();
    if (d->rdpsnd) {
        rdpsnd_server_context_free(d->rdpsnd);
        d->rdpsnd = nullptr;
    }
}
}
