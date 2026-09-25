// SPDX-FileCopyrightText: 2026 Nick Haghiri <nick@haghiri.net>
//
// SPDX-License-Identifier: LGPL-2.1-only OR LGPL-3.0-only OR LicenseRef-KDE-Accepted-LGPL

#pragma once

#include <algorithm>
#include <cstdint>
#include <mutex>
#include <utility>
#include <vector>

#include <freerdp/codec/audio.h>
#include <pipewire/pipewire.h>
#include <spa/param/audio/raw.h>
#include <spa/param/buffers.h>
#include <spa/param/format.h>
#include <spa/pod/builder.h>

#include "krdp_logging.h"

namespace KRdp
{

inline constexpr uint16_t s_channels = 2;
inline constexpr uint16_t s_bitsPerSample = 16;
inline constexpr uint16_t s_blockAlign = s_channels * s_bitsPerSample / 8;
inline constexpr uint32_t s_position[s_channels] = {SPA_AUDIO_CHANNEL_FL, SPA_AUDIO_CHANNEL_FR};

inline AUDIO_FORMAT audioFormat(UINT16 formatTag, uint32_t rate, uint32_t avgBytesPerSec = 0)
{
    AUDIO_FORMAT format{};
    format.wFormatTag = formatTag;
    format.nChannels = s_channels;
    format.nSamplesPerSec = rate;
    format.wBitsPerSample = s_bitsPerSample;
    format.nBlockAlign = s_blockAlign;
    format.nAvgBytesPerSec = avgBytesPerSec ? avgBytesPerSec : rate * s_blockAlign;
    return format;
}

inline void logDropOnce(bool dropped, bool &warned, const char *message)
{
    if (dropped && !warned) {
        qCWarning(KRDP) << message;
        warned = true;
    }
}

inline void destroyPwStream(pw_stream *&stream, spa_hook &listener)
{
    if (!stream) {
        return;
    }
    spa_hook_remove(&listener);
    pw_stream_destroy(stream);
    stream = nullptr;
}

inline const spa_pod *buildAudioFormatPod(spa_pod_builder &builder, uint32_t rate)
{
    return static_cast<spa_pod *>(spa_pod_builder_add_object(&builder,
                                                             SPA_TYPE_OBJECT_Format,
                                                             SPA_PARAM_EnumFormat,
                                                             SPA_FORMAT_mediaType,
                                                             SPA_POD_Id(SPA_MEDIA_TYPE_audio),
                                                             SPA_FORMAT_mediaSubtype,
                                                             SPA_POD_Id(SPA_MEDIA_SUBTYPE_raw),
                                                             SPA_FORMAT_AUDIO_format,
                                                             SPA_POD_Id(SPA_AUDIO_FORMAT_S16),
                                                             SPA_FORMAT_AUDIO_rate,
                                                             SPA_POD_Int(rate),
                                                             SPA_FORMAT_AUDIO_channels,
                                                             SPA_POD_Int(s_channels),
                                                             SPA_FORMAT_AUDIO_position,
                                                             SPA_POD_Array(sizeof(uint32_t), SPA_TYPE_Id, s_channels, s_position)));
}

inline void respondParamBuffers(pw_stream *stream, uint32_t id, const spa_pod *param)
{
    if (id != SPA_PARAM_Format || !param) {
        return;
    }
    uint8_t buffer[512];
    spa_pod_builder builder = SPA_POD_BUILDER_INIT(buffer, sizeof(buffer));
    const spa_pod *params[1];
    params[0] = static_cast<spa_pod *>(spa_pod_builder_add_object(&builder,
                                                                  SPA_TYPE_OBJECT_ParamBuffers,
                                                                  SPA_PARAM_Buffers,
                                                                  SPA_PARAM_BUFFERS_buffers,
                                                                  SPA_POD_CHOICE_RANGE_Int(4, 2, 8),
                                                                  SPA_PARAM_BUFFERS_blocks,
                                                                  SPA_POD_Int(1),
                                                                  SPA_PARAM_BUFFERS_stride,
                                                                  SPA_POD_Int(s_blockAlign),
                                                                  SPA_PARAM_BUFFERS_dataType,
                                                                  SPA_POD_CHOICE_FLAGS_Int(1u << SPA_DATA_MemPtr)));
    pw_stream_update_params(stream, params, 1);
}

class PipeWireLoopConnection
{
public:
    explicit PipeWireLoopConnection(const char *loopName)
    {
        pw_init(nullptr, nullptr);
        m_loop = pw_thread_loop_new(loopName, nullptr);
        if (!m_loop || pw_thread_loop_start(m_loop) < 0) {
            return;
        }
        pw_thread_loop_lock(m_loop);
        m_context = pw_context_new(pw_thread_loop_get_loop(m_loop), nullptr, 0);
        m_core = m_context ? pw_context_connect(m_context, nullptr, 0) : nullptr;
        pw_thread_loop_unlock(m_loop);
    }

    ~PipeWireLoopConnection()
    {
        if (!m_loop) {
            return;
        }
        pw_thread_loop_lock(m_loop);
        if (m_core) {
            roundtrip();
            pw_core_disconnect(m_core);
        }
        if (m_context) {
            pw_context_destroy(m_context);
        }
        pw_thread_loop_unlock(m_loop);
        pw_thread_loop_stop(m_loop);
        pw_thread_loop_destroy(m_loop);
    }

    PipeWireLoopConnection(const PipeWireLoopConnection &) = delete;
    PipeWireLoopConnection &operator=(const PipeWireLoopConnection &) = delete;

    bool isValid() const
    {
        return m_loop && m_core;
    }

    bool roundtrip()
    {
        struct SyncState {
            pw_thread_loop *loop;
            int pendingSeq;
            bool done = false;
        } state{m_loop, 0};
        spa_hook hook{};
        pw_core_events events{};
        events.version = PW_VERSION_CORE_EVENTS;
        events.done = [](void *data, uint32_t, int seq) {
            auto *syncState = static_cast<SyncState *>(data);
            if (seq == syncState->pendingSeq) {
                syncState->done = true;
                pw_thread_loop_signal(syncState->loop, false);
            }
        };
        pw_core_add_listener(m_core, &hook, &events, &state);
        state.pendingSeq = pw_core_sync(m_core, PW_ID_CORE, 0);
        for (int waited = 0; waited < 2 && !state.done; ++waited) {
            pw_thread_loop_timed_wait(m_loop, 1);
        }
        spa_hook_remove(&hook);
        return state.done;
    }
    pw_thread_loop *loop() const
    {
        return m_loop;
    }
    pw_core *core() const
    {
        return m_core;
    }

private:
    pw_thread_loop *m_loop = nullptr;
    pw_context *m_context = nullptr;
    pw_core *m_core = nullptr;
};

class AudioByteQueue
{
public:
    explicit AudioByteQueue(size_t capacity)
        : m_capacity(capacity)
    {
    }

    bool write(const uint8_t *data, size_t bytes)
    {
        std::lock_guard lock(m_mutex);
        bool dropped = bytes > m_capacity;
        if (dropped) {
            data += bytes - m_capacity;
            bytes = m_capacity;
        }
        m_data.insert(m_data.end(), data, data + bytes);
        if (m_data.size() > m_capacity) {
            m_data.erase(m_data.begin(), m_data.begin() + (m_data.size() - m_capacity));
            dropped = true;
        }
        return dropped;
    }

    size_t read(uint8_t *dst, size_t maxBytes)
    {
        std::lock_guard lock(m_mutex);
        const size_t bytes = std::min(maxBytes, m_data.size());
        std::copy_n(m_data.begin(), bytes, dst);
        m_data.erase(m_data.begin(), m_data.begin() + bytes);
        return bytes;
    }

    std::vector<uint8_t> readAll()
    {
        std::lock_guard lock(m_mutex);
        return std::exchange(m_data, {});
    }

    void reset()
    {
        std::lock_guard lock(m_mutex);
        m_data.clear();
    }

private:
    size_t m_capacity;
    std::mutex m_mutex;
    std::vector<uint8_t> m_data;
};

}
