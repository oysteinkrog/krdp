// SPDX-FileCopyrightText: 2026 Nick Haghiri <nick@haghiri.net>
//
// SPDX-License-Identifier: LGPL-2.1-only OR LGPL-3.0-only OR LicenseRef-KDE-Accepted-LGPL

#pragma once

#include <memory>

#include "krdp_export.h"

namespace KRdp
{

class RdpConnection;

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

private:
    class Private;
    const std::unique_ptr<Private> d;
};

}
