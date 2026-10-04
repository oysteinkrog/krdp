// SPDX-FileCopyrightText: 2026 Nick Haghiri <nick@haghiri.net>
//
// This file is roughly based on grd-rdp-dvc-audio-input.c from Gnome Remote
// Desktop, which is:
//
// SPDX-FileCopyrightText: 2022 Pascal Nowack
//
// SPDX-License-Identifier: GPL-2.0-or-later

#include "AudinStream.h"

#include <atomic>
#include <cstring>
#include <vector>

#include <QElapsedTimer>
#include <QProcess>
#include <QScopeGuard>

#include "PeerContext_p.h"
#include "PipeWireStreamUtils_p.h"
#include "RdpConnection.h"
#include "krdp_logging.h"

#include <freerdp/codec/audio.h>
#include <freerdp/server/audin.h>
#include <winpr/stream.h>

#include <pipewire/pipewire.h>
#include <spa/param/audio/raw.h>
#include <spa/param/format.h>
#include <spa/pod/builder.h>

namespace KRdp
{

static constexpr uint32_t s_sampleRate = 44100;
static constexpr uint32_t s_maxLatencyMs = 200;
// mstsc rejects the AUDIO_INPUT channel when it is opened very early in the session
// (within about 100 ms of activation), so retry when the client hasn't answered.
static constexpr qint64 s_openTimeoutMs = 2000;
static constexpr int s_maxOpenAttempts = 3;

class AudinStream::Private
{
public:
    RdpConnection *connection = nullptr;

    audin_server_context *audin = nullptr;
    bool openTried = false;
    int openAttempts = 0;
    QElapsedTimer openedAt;
    // Set from FreeRDP's audin thread when the client's Version PDU arrives.
    std::atomic<bool> clientAnswered = false;
    psAudinServerVersion defaultReceiveVersion = nullptr;

    std::unique_ptr<PipeWireLoopConnection> pipewire;
    pw_stream *stream = nullptr;
    spa_hook streamListener{};
    bool micSetupTried = false;
    // The configured default source before we pointed it at the virtual microphone,
    // as the JSON value pw-metadata prints; empty when none was set.
    QString savedDefaultSource;
    bool defaultSourceChanged = false;
    void makeMicrophoneDefault();
    void restoreDefaultSource();

    AudioByteQueue captureRing{size_t(s_sampleRate) * s_maxLatencyMs / 1000 * s_blockAlign};
    bool captureDropWarned = false;
    std::vector<uint8_t> partialFrame;

    void onData(const SNDIN_DATA *data);
    void ensureMicrophone();
    void teardownMicrophone();

    static UINT audinData(audin_server_context *context, const SNDIN_DATA *data);
    static UINT audinVersion(audin_server_context *context, const SNDIN_VERSION *version);

private:
    static void streamParamChanged(void *data, uint32_t id, const spa_pod *param);
    static void streamProcess(void *data);

    static constexpr pw_stream_events s_streamEvents = {
        .version = PW_VERSION_STREAM_EVENTS,
        .param_changed = streamParamChanged,
        .process = streamProcess,
    };
};

UINT AudinStream::Private::audinVersion(audin_server_context *context, const SNDIN_VERSION *version)
{
    auto *d = static_cast<Private *>(context->userdata);
    d->clientAnswered = true;
    return d->defaultReceiveVersion ? d->defaultReceiveVersion(context, version) : CHANNEL_RC_OK;
}

UINT AudinStream::Private::audinData(audin_server_context *context, const SNDIN_DATA *data)
{
    static_cast<Private *>(context->userdata)->onData(data);
    return CHANNEL_RC_OK;
}

void AudinStream::Private::streamParamChanged(void *data, uint32_t id, const spa_pod *param)
{
    respondParamBuffers(static_cast<Private *>(data)->stream, id, param);
}

void AudinStream::Private::streamProcess(void *data)
{
    auto *d = static_cast<Private *>(data);
    pw_buffer *buffer = pw_stream_dequeue_buffer(d->stream);
    if (!buffer) {
        return;
    }
    spa_data &block = buffer->buffer->datas[0];
    uint64_t frames = block.maxsize / s_blockAlign;
    if (buffer->requested > 0 && buffer->requested < frames) {
        frames = buffer->requested;
    }
    const size_t written = block.data ? d->captureRing.read(static_cast<uint8_t *>(block.data), frames * s_blockAlign) : 0;
    block.chunk->offset = 0;
    block.chunk->stride = s_blockAlign;
    block.chunk->size = static_cast<uint32_t>(written);
    pw_stream_queue_buffer(d->stream, buffer);
}

AudinStream::AudinStream(RdpConnection *connection)
    : d(std::make_unique<Private>())
{
    d->connection = connection;
}

AudinStream::~AudinStream()
{
    close();
}

bool AudinStream::open()
{
    if (d->audin) {
        if (d->clientAnswered || d->openedAt.elapsed() < s_openTimeoutMs || d->openAttempts >= s_maxOpenAttempts) {
            return true;
        }
        qCWarning(KRDP) << "Audio input: client did not answer the audin channel; reopening, attempt" << d->openAttempts + 1;
        d->audin->Close(d->audin);
        audin_server_context_free(d->audin);
        d->audin = nullptr;
        d->openTried = false;
    }
    if (d->openTried) {
        return false;
    }
    d->openTried = true;

    auto peerContext = reinterpret_cast<PeerContext *>(d->connection->rdpPeerContext());
    if (!peerContext || !peerContext->virtualChannelManager) {
        return false;
    }
    if (!freerdp_settings_get_bool(d->connection->rdpPeerContext()->settings, FreeRDP_AudioCapture)) {
        qCDebug(KRDP) << "Audio input: client didn't ask for microphone redirection";
        return false;
    }

    d->audin = audin_server_context_new(peerContext->virtualChannelManager);
    if (!d->audin) {
        qCWarning(KRDP) << "Failed to create audin server context";
        return false;
    }

    d->audin->userdata = d.get();
    d->audin->rdpcontext = d->connection->rdpPeerContext();
    d->audin->Data = Private::audinData;
    d->defaultReceiveVersion = d->audin->ReceiveVersion;
    d->audin->ReceiveVersion = Private::audinVersion;
    d->clientAnswered = false;

    auto cleanup = qScopeGuard([this]() {
        audin_server_context_free(d->audin);
        d->audin = nullptr;
    });

    const AUDIO_FORMAT serverFormat = audioFormat(WAVE_FORMAT_PCM, s_sampleRate);
    if (!audin_server_set_formats(d->audin, 1, &serverFormat)) {
        qCWarning(KRDP) << "Failed to set audin server formats";
        return false;
    }

    if (!d->audin->Open(d->audin)) {
        qCWarning(KRDP) << "Failed to open audin (AUDIO_INPUT) channel";
        return false;
    }
    cleanup.dismiss();
    d->openedAt.start();
    ++d->openAttempts;

    qCDebug(KRDP) << "Audio input (audin) channel opened";
    return true;
}

namespace
{
constexpr auto s_defaultSourceKey = "default.configured.audio.source";

QString readDefaultSource()
{
    QProcess p;
    p.start(QStringLiteral("pw-metadata"), {QStringLiteral("0"), QString::fromLatin1(s_defaultSourceKey)});
    if (!p.waitForFinished(2000)) {
        return {};
    }
    const QString out = QString::fromUtf8(p.readAllStandardOutput());
    const QString start = QStringLiteral("value:'");
    const qsizetype from = out.indexOf(start);
    if (from < 0) {
        return {};
    }
    const qsizetype to = out.indexOf(QStringLiteral("' type:"), from);
    return to < 0 ? QString() : out.mid(from + start.size(), to - from - start.size());
}

bool writeDefaultSource(const QString &json)
{
    QStringList args;
    if (json.isEmpty()) {
        args = {QStringLiteral("-d"), QStringLiteral("0"), QString::fromLatin1(s_defaultSourceKey)};
    } else {
        args = {QStringLiteral("0"), QString::fromLatin1(s_defaultSourceKey), json, QStringLiteral("Spa:String:JSON")};
    }
    return QProcess::execute(QStringLiteral("pw-metadata"), args) == 0;
}
}

void AudinStream::Private::makeMicrophoneDefault()
{
    // Apps record from the default source, so point it at the client's microphone
    // while connected and put the previous choice back on disconnect.
    savedDefaultSource = readDefaultSource();
    if (writeDefaultSource(QStringLiteral(R"({"name":"krdp_microphone"})"))) {
        defaultSourceChanged = true;
        qCDebug(KRDP) << "Audio input: made the Remote Desktop microphone the default source; previous" << savedDefaultSource;
    } else {
        qCWarning(KRDP) << "Audio input: could not make the Remote Desktop microphone the default source";
    }
}

void AudinStream::Private::restoreDefaultSource()
{
    if (!defaultSourceChanged) {
        return;
    }
    defaultSourceChanged = false;
    writeDefaultSource(savedDefaultSource);
    qCDebug(KRDP) << "Audio input: restored the default source" << savedDefaultSource;
}

void AudinStream::Private::ensureMicrophone()
{
    if (micSetupTried) {
        return;
    }
    micSetupTried = true;

    pipewire = std::make_unique<PipeWireLoopConnection>("krdp-audio-input");
    if (!pipewire->isValid()) {
        qCWarning(KRDP) << "Audio input: could not connect to PipeWire";
        teardownMicrophone();
        return;
    }

    uint8_t buffer[512];
    spa_pod_builder builder = SPA_POD_BUILDER_INIT(buffer, sizeof(buffer));
    const spa_pod *params[1] = {buildAudioFormatPod(builder, s_sampleRate)};

    pw_properties *props = pw_properties_new(PW_KEY_MEDIA_TYPE,
                                             "Audio",
                                             PW_KEY_MEDIA_CATEGORY,
                                             "Playback",
                                             PW_KEY_MEDIA_CLASS,
                                             "Audio/Source",
                                             PW_KEY_NODE_SUSPEND_ON_IDLE,
                                             "true",
                                             PW_KEY_NODE_NAME,
                                             "krdp_microphone",
                                             PW_KEY_NODE_DESCRIPTION,
                                             "Remote Desktop Microphone",
                                             nullptr);

    pw_thread_loop_lock(pipewire->loop());
    stream = pw_stream_new(pipewire->core(), "krdp-audio-input", props);
    if (!stream) {
        pw_thread_loop_unlock(pipewire->loop());
        qCWarning(KRDP) << "Audio input: could not create virtual microphone stream";
        teardownMicrophone();
        return;
    }
    pw_stream_add_listener(stream, &streamListener, &s_streamEvents, this);

    const auto flags = static_cast<pw_stream_flags>(PW_STREAM_FLAG_AUTOCONNECT | PW_STREAM_FLAG_RT_PROCESS | PW_STREAM_FLAG_MAP_BUFFERS);
    const bool connected = pw_stream_connect(stream, PW_DIRECTION_OUTPUT, PW_ID_ANY, flags, params, 1) >= 0;
    pw_thread_loop_unlock(pipewire->loop());
    if (!connected) {
        qCWarning(KRDP) << "Audio input: could not connect virtual microphone stream";
        teardownMicrophone();
        return;
    }

    qCDebug(KRDP) << "Audio input: virtual microphone ready";
    makeMicrophoneDefault();
}

void AudinStream::Private::onData(const SNDIN_DATA *data)
{
    if (!data || !data->Data) {
        return;
    }

    ensureMicrophone();
    if (!stream) {
        return;
    }

    const size_t incoming = Stream_GetRemainingLength(data->Data);
    const auto *samples = reinterpret_cast<const uint8_t *>(Stream_Pointer(data->Data));
    partialFrame.insert(partialFrame.end(), samples, samples + incoming);

    const size_t whole = (partialFrame.size() / s_blockAlign) * s_blockAlign;
    if (whole > 0) {
        logDropOnce(captureRing.write(partialFrame.data(), whole),
                    captureDropWarned,
                    "Audio input: capture backlog exceeded buffer; dropping oldest microphone audio");
        partialFrame.erase(partialFrame.begin(), partialFrame.begin() + whole);
    }
}

void AudinStream::Private::teardownMicrophone()
{
    restoreDefaultSource();
    if (pipewire && pipewire->loop()) {
        pw_thread_loop_lock(pipewire->loop());
        destroyPwStream(stream, streamListener);
        pw_thread_loop_unlock(pipewire->loop());
    }
    pipewire.reset();
    captureRing.reset();
    captureDropWarned = false;
    partialFrame.clear();
}

void AudinStream::close()
{
    if (d->audin) {
        d->audin->Close(d->audin);
        audin_server_context_free(d->audin);
        d->audin = nullptr;
    }
    d->teardownMicrophone();
}

}
