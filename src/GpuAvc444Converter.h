// SPDX-FileCopyrightText: 2026 Øystein Krog <oystein.krog@gmail.com>
//
// SPDX-License-Identifier: LGPL-2.1-only OR LGPL-3.0-only OR LicenseRef-KDE-Accepted-LGPL

#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <QSize>

struct PipeWireFrame;
struct AVFrame;
struct AVBufferRef;

namespace KRdp
{

/**
 * One frame converted to the two YUV 4:2:0 pictures of AVC444v2 ([MS-RDPEGFX] 3.3.8.3.3),
 * the same output as FreeRDP's RGBToAVC444YUVv2. Each picture is a Y plane of width x
 * height bytes followed by U and V planes of width / 2 x height / 2 bytes, without padding.
 */
struct GpuAvc444Picture {
    QSize size;
    std::vector<uint8_t> luma; ///< the main picture: full Y, averaged U and V
    std::vector<uint8_t> chroma; ///< the auxiliary picture with the rest of U and V
    /// One byte per 64x64 screen tile, row by row: bit 0 set if the luma picture changed
    /// there since the previous conversion, bit 1 if the chroma picture did.
    std::vector<uint8_t> tiles;
    int tilesPerRow = 0;
    /// With Output::Cuda: the two pictures as FFmpeg CUDA frames (YUV420P), ready for NVENC.
    std::shared_ptr<AVFrame> lumaFrame;
    std::shared_ptr<AVFrame> chromaFrame;

    static constexpr uint8_t LumaChanged = 1;
    static constexpr uint8_t ChromaChanged = 2;
};

/**
 * Converts DMA-BUF frames to AVC444v2 pictures with a GL compute shader, in its own EGL
 * context. It also compares each picture with the one from the previous call and marks the
 * screen tiles that changed, so the comparison FreeRDP does on the CPU is not needed.
 *
 * Not thread safe; call it from one thread.
 */
class GpuAvc444Converter
{
public:
    GpuAvc444Converter();
    ~GpuAvc444Converter();

    /// Whether frames of this size can be converted: the width must be a multiple of 32
    /// (the decoder aligns the chroma halves to that) and the height even.
    static bool sizeSupported(const QSize &size);

    enum class Output {
        Cpu, ///< read both pictures back into GpuAvc444Picture::luma and chroma
        Cuda, ///< copy them into CUDA frames on the GPU, see GpuAvc444Picture::lumaFrame
        TilesOnly, ///< only the changed tiles, for timing tests
    };

    /// Converts the frame. Returns false if the GPU path is not available or failed, in
    /// which case the caller should use the CPU path.
    bool convert(const PipeWireFrame &frame, GpuAvc444Picture &picture, Output output = Output::Cpu);

    /// Whether Output::Cuda works: libcuda is there and can share buffers with the GL context.
    bool cudaAvailable();

    /// Forgets the previous picture, so the next conversion marks every tile as changed.
    void reset();

private:
    class Private;
    const std::unique_ptr<Private> d;
};

}
