// SPDX-FileCopyrightText: 2026 Nick Haghiri <nick@haghiri.net>
//
// SPDX-License-Identifier: LGPL-2.1-only OR LGPL-3.0-only OR LicenseRef-KDE-Accepted-LGPL

#pragma once

#include <memory>

#include "krdp_export.h"

namespace KRdp
{

class RdpConnection;

/** Client-to-server audio input (audin / MS-RDPEAI) as a native PipeWire Audio/Source node. */
class KRDP_EXPORT AudinStream
{
public:
    explicit AudinStream(RdpConnection *connection);
    ~AudinStream();

    bool open();
    void close();

private:
    class Private;
    const std::unique_ptr<Private> d;
};

}
