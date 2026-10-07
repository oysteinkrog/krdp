// SPDX-FileCopyrightText: 2026 Øystein Krog <oystein.krog@gmail.com>
//
// SPDX-License-Identifier: LGPL-2.1-only OR LGPL-3.0-only OR LicenseRef-KDE-Accepted-LGPL

// Decodes a capture of KRDP's GPU AVC444 stream the way a client does: one FreeRDP H.264
// decoder, and a surface that only takes the region rects of each frame. It writes the
// surface as PNG files, so a picture that is broken on the client can be checked against
// what KRDP actually sent. If the PNGs look right, the client broke it.
//
// Start a capture with `touch $XDG_RUNTIME_DIR/krdp-dump-now`; KRDP writes the next 10 s to
// /var/tmp/krdp-dump/<time>/. Then:
//   avc444dumpdecode /var/tmp/krdp-dump/<time> [every N frames, default 30]
//
// Build:
//   export PKG_CONFIG_PATH=/opt/krdp-local/lib/pkgconfig
//   g++ -std=c++20 -O2 -o avc444dumpdecode tools/avc444dumpdecode.cpp \
//       $(pkg-config --cflags --libs Qt6Core Qt6Gui freerdp3 winpr3) -Wl,-rpath,/opt/krdp-local/lib

#include <cstdio>
#include <cstring>
#include <vector>

#include <QDir>
#include <QFile>
#include <QImage>

#include <freerdp/channels/rdpgfx.h>
#include <freerdp/codec/color.h>
#include <freerdp/codec/h264.h>

namespace
{
struct Picture {
    std::vector<RECTANGLE_16> rects;
    QByteArray data;
};

bool readPicture(QFile &file, Picture &picture)
{
    quint32 count = 0;
    if (file.read(reinterpret_cast<char *>(&count), 4) != 4 || count > 100000) {
        return false;
    }
    picture.rects.resize(count);
    const qint64 rectBytes = qint64(count) * qint64(sizeof(RECTANGLE_16));
    if (count && file.read(reinterpret_cast<char *>(picture.rects.data()), rectBytes) != rectBytes) {
        return false;
    }
    quint32 length = 0;
    if (file.read(reinterpret_cast<char *>(&length), 4) != 4) {
        return false;
    }
    picture.data = file.read(length);
    return picture.data.size() == qsizetype(length);
}
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <capture dir> [every N frames]\n", argv[0]);
        return 2;
    }
    const QDir dir(QString::fromLocal8Bit(argv[1]));
    const int every = argc > 2 ? std::max(1, atoi(argv[2])) : 30;
    const QStringList files = dir.entryList({QStringLiteral("*.bin")}, QDir::Files, QDir::Name);
    if (files.isEmpty()) {
        fprintf(stderr, "no frames in %s\n", argv[1]);
        return 1;
    }

    H264_CONTEXT *decoder = nullptr;
    QImage surface;
    int failures = 0;
    for (int index = 0; index < files.size(); ++index) {
        QFile file(dir.filePath(files[index]));
        char magic[4] = {};
        quint32 header[3] = {};
        Picture luma;
        Picture chroma;
        if (!file.open(QIODevice::ReadOnly) || file.read(magic, 4) != 4 || memcmp(magic, "KAV4", 4) != 0
            || file.read(reinterpret_cast<char *>(header), sizeof(header)) != sizeof(header) || !readPicture(file, luma) || !readPicture(file, chroma)) {
            fprintf(stderr, "%s: cannot read\n", qPrintable(files[index]));
            return 1;
        }
        const quint32 width = header[0];
        const quint32 height = header[1];
        const quint32 lc = header[2];
        if (!decoder) {
            decoder = h264_context_new(FALSE);
            if (!decoder || !h264_context_reset(decoder, width, height)) {
                fprintf(stderr, "cannot create the H.264 decoder\n");
                return 1;
            }
            surface = QImage(int(width), int(height), QImage::Format_RGB32);
            surface.fill(Qt::black);
        }
        // [MS-RDPEGFX] 2.2.4.5: the only picture goes in the first bitstream.
        const Picture &first = lc == 2 ? chroma : luma;
        const Picture *second = lc == 0 ? &chroma : nullptr;
        const INT32 rc = avc444_decompress(decoder,
                                           BYTE(lc),
                                           first.rects.data(),
                                           UINT32(first.rects.size()),
                                           reinterpret_cast<const BYTE *>(first.data.constData()),
                                           UINT32(first.data.size()),
                                           second ? second->rects.data() : nullptr,
                                           second ? UINT32(second->rects.size()) : 0,
                                           second ? reinterpret_cast<const BYTE *>(second->data.constData()) : nullptr,
                                           second ? UINT32(second->data.size()) : 0,
                                           surface.bits(),
                                           PIXEL_FORMAT_BGRX32,
                                           UINT32(surface.bytesPerLine()),
                                           width,
                                           height,
                                           RDPGFX_CODECID_AVC444v2);
        printf("%s: LC %u, luma %zu rects %lld bytes, chroma %zu rects %lld bytes%s\n",
               qPrintable(files[index]),
               lc,
               luma.rects.size(),
               (long long)luma.data.size(),
               chroma.rects.size(),
               (long long)chroma.data.size(),
               rc < 0 ? ", DECODE FAILED" : "");
        failures += rc < 0 ? 1 : 0;
        if (index % every == 0 || index == files.size() - 1) {
            const QString png = dir.filePath(QStringLiteral("decoded-%1.png").arg(index, 6, 10, QLatin1Char('0')));
            surface.save(png);
            printf("  wrote %s\n", qPrintable(png));
        }
    }
    h264_context_free(decoder);
    printf("%lld frames, %d failed to decode\n", (long long)files.size(), failures);
    return failures ? 1 : 0;
}
