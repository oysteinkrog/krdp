// SPDX-FileCopyrightText: 2026 Nick Haghiri <nick@haghiri.net>
//
// SPDX-License-Identifier: LGPL-2.1-only OR LGPL-3.0-only OR LicenseRef-KDE-Accepted-LGPL

#pragma once

#include <chrono>
#include <memory>

#include <QString>

#include "krdp_export.h"

namespace KRdp
{

class RdpConnection;

/** Audio output settings from krdpserverrc. */
struct KRDP_EXPORT AudioSettings {
    enum class Codec {
        Auto, ///< AAC, else Opus, else PCM
        Pcm, ///< uncompressed 48 kHz; mstsc buffers it about twice as long as AAC
        Aac,
        Opus,
    };
    Codec codec = Codec::Auto;
    /// Bit rate for AAC and Opus.
    int bitrateKbit = 192;
    /// Close the client's audio stream after this much silence; 0 keeps it open.
    std::chrono::seconds idleTimeout{60};

    static AudioSettings fromStrings(const QString &codec, int bitrateKbit, int idleTimeoutSeconds);
};

/** Server-to-client audio output (rdpsnd / MS-RDPEA) over a native PipeWire stream. */
class KRDP_EXPORT AudioStream
{
public:
    explicit AudioStream(RdpConnection *connection);
    ~AudioStream();

    bool initialize();
    void handleMessages();
    void close();
    void *wakeHandle() const;

    /// Applies to connections made after the call.
    static void setSettings(const AudioSettings &settings);
    static AudioSettings settings();

private:
    class Private;
    const std::unique_ptr<Private> d;
};

}
