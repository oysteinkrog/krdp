// SPDX-FileCopyrightText: 2026 Øystein Krog <oystein.krog@gmail.com>
//
// SPDX-License-Identifier: LGPL-2.1-only OR LGPL-3.0-only OR LicenseRef-KDE-Accepted-LGPL

// Offline test for GpuAvc444Converter: fills a GBM buffer with known pixels, converts it on
// the GPU and compares both pictures byte for byte with FreeRDP's C conversion. Then it
// changes a few pixels and checks that exactly the right tiles are marked as changed.
//
// Build it against a configured KRDP build tree (for the generated krdp_logging files):
//   export PKG_CONFIG_PATH=/opt/krdp-local/lib/pkgconfig
//   g++ -std=c++20 -O2 -fPIC -o gpuavc444test tools/gpuavc444test.cpp src/GpuAvc444Converter.cpp \
//       BUILD/src/krdp_logging.cpp -IBUILD/src -I/opt/krdp-local/include/KPipeWire \
//       $(pkg-config --cflags --libs Qt6Core Qt6Gui epoxy gbm libdrm freerdp3 winpr3 libpipewire-0.3) \
//       -Wl,-rpath,/opt/krdp-local/lib

#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <random>
#include <unistd.h>

#include <drm_fourcc.h>
#include <epoxy/egl.h>
#include <epoxy/gl.h>
#include <freerdp/primitives.h>
#include <gbm.h>

#include <QDir>
#include <QElapsedTimer>

#include <PipeWireSourceStream>

#include <cmath>

#include <freerdp/channels/rdpgfx.h>
#include <freerdp/codec/h264.h>

#include "../src/GpuAvc444Converter.h"
#include "../src/GpuH264Encoder.h"

using namespace KRdp;

namespace
{
constexpr int W = 2560;
constexpr int H = 1440;

struct Buffer {
    gbm_bo *bo = nullptr;
    int fd = -1;
    uint32_t stride = 0;
    uint64_t modifier = 0;
};

// NVIDIA cannot map its tiled buffers for the CPU, so the test writes the pixels through
// its own GL context: the buffer imported as a texture, then glTexSubImage2D.
struct Writer {
    EGLDisplay display = EGL_NO_DISPLAY;
    EGLContext context = EGL_NO_CONTEXT;
    GLuint texture = 0;
    EGLImage image = EGL_NO_IMAGE_KHR;
};

bool fill(Buffer &buffer, const std::vector<uint32_t> &pixels, gbm_device *gbm)
{
    static Writer writer;
    if (writer.context == EGL_NO_CONTEXT) {
        writer.display = eglGetPlatformDisplayEXT(EGL_PLATFORM_GBM_KHR, gbm, nullptr);
        eglInitialize(writer.display, nullptr, nullptr);
        eglBindAPI(EGL_OPENGL_API);
        const EGLint attribs[] = {EGL_CONTEXT_MAJOR_VERSION, 4, EGL_CONTEXT_MINOR_VERSION, 5, EGL_NONE};
        writer.context = eglCreateContext(writer.display, EGL_NO_CONFIG_KHR, EGL_NO_CONTEXT, attribs);
        eglMakeCurrent(writer.display, EGL_NO_SURFACE, EGL_NO_SURFACE, writer.context);
        const EGLint imageAttribs[] = {EGL_WIDTH,
                                       W,
                                       EGL_HEIGHT,
                                       H,
                                       EGL_LINUX_DRM_FOURCC_EXT,
                                       DRM_FORMAT_XRGB8888,
                                       EGL_DMA_BUF_PLANE0_FD_EXT,
                                       buffer.fd,
                                       EGL_DMA_BUF_PLANE0_OFFSET_EXT,
                                       0,
                                       EGL_DMA_BUF_PLANE0_PITCH_EXT,
                                       EGLint(buffer.stride),
                                       EGL_DMA_BUF_PLANE0_MODIFIER_LO_EXT,
                                       EGLint(buffer.modifier & 0xffffffff),
                                       EGL_DMA_BUF_PLANE0_MODIFIER_HI_EXT,
                                       EGLint(buffer.modifier >> 32),
                                       EGL_NONE};
        writer.image = eglCreateImageKHR(writer.display, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, nullptr, imageAttribs);
        if (writer.image == EGL_NO_IMAGE_KHR) {
            printf("writer: cannot import the buffer (0x%x)\n", eglGetError());
            return false;
        }
        glGenTextures(1, &writer.texture);
        glBindTexture(GL_TEXTURE_2D, writer.texture);
        glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, writer.image);
    }
    eglMakeCurrent(writer.display, EGL_NO_SURFACE, EGL_NO_SURFACE, writer.context);
    glBindTexture(GL_TEXTURE_2D, writer.texture);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, W, H, GL_BGRA, GL_UNSIGNED_BYTE, pixels.data());
    glFinish();
    if (const GLenum error = glGetError(); error != GL_NO_ERROR) {
        printf("writer: GL error 0x%x\n", error);
        return false;
    }
    return true;
}

PipeWireFrame frameFor(const Buffer &buffer)
{
    PipeWireFrame frame;
    DmaBufAttributes attribs;
    attribs.width = W;
    attribs.height = H;
    attribs.format = DRM_FORMAT_XRGB8888;
    attribs.modifier = buffer.modifier;
    attribs.planes.append({buffer.fd, 0, buffer.stride});
    frame.dmabuf = attribs;
    return frame;
}

int compare(const GpuAvc444Picture &picture, const std::vector<uint32_t> &pixels)
{
    std::vector<uint8_t> luma(size_t(W) * H * 3 / 2);
    std::vector<uint8_t> chroma(luma.size());
    BYTE *lumaPlanes[3] = {luma.data(), luma.data() + W * H, luma.data() + W * H + W * H / 4};
    BYTE *chromaPlanes[3] = {chroma.data(), chroma.data() + W * H, chroma.data() + W * H + W * H / 4};
    const UINT32 strides[3] = {W, W / 2, W / 2};
    const prim_size_t roi = {W, H};
    primitives_get_generic()->RGBToAVC444YUVv2(reinterpret_cast<const BYTE *>(pixels.data()), PIXEL_FORMAT_BGRX32, W * 4, lumaPlanes, strides, chromaPlanes, strides, &roi);

    int failures = 0;
    const char *names[2] = {"luma", "chroma"};
    const std::vector<uint8_t> *gpu[2] = {&picture.luma, &picture.chroma};
    const std::vector<uint8_t> *cpu[2] = {&luma, &chroma};
    for (int p = 0; p < 2; ++p) {
        size_t differing = 0;
        for (size_t i = 0; i < cpu[p]->size(); ++i) {
            if ((*gpu[p])[i] != (*cpu[p])[i]) {
                if (differing < 5) {
                    const size_t ySize = size_t(W) * H;
                    const char *plane = i < ySize ? "Y" : (i < ySize * 5 / 4 ? "U" : "V");
                    printf("  %s %s byte %zu: GPU %d CPU %d\n", names[p], plane, i, (*gpu[p])[i], (*cpu[p])[i]);
                }
                ++differing;
            }
        }
        printf("%s picture: %zu of %zu bytes differ\n", names[p], differing, cpu[p]->size());
        failures += differing ? 1 : 0;
    }
    return failures;
}
}

int main()
{
    const QStringList nodes = QDir(QStringLiteral("/dev/dri")).entryList({QStringLiteral("renderD*")}, QDir::System, QDir::Name);
    const int drmFd = open(("/dev/dri/" + nodes.first()).toLatin1().constData(), O_RDWR | O_CLOEXEC);
    gbm_device *gbm = gbm_create_device(drmFd);
    Buffer buffer;
    buffer.bo = gbm_bo_create(gbm, W, H, GBM_FORMAT_XRGB8888, GBM_BO_USE_RENDERING | GBM_BO_USE_LINEAR);
    if (!buffer.bo) {
        buffer.bo = gbm_bo_create(gbm, W, H, GBM_FORMAT_XRGB8888, GBM_BO_USE_RENDERING);
    }
    if (!buffer.bo) {
        printf("cannot create a GBM buffer\n");
        return 2;
    }
    buffer.fd = gbm_bo_get_fd(buffer.bo);
    buffer.stride = gbm_bo_get_stride(buffer.bo);
    buffer.modifier = gbm_bo_get_modifier(buffer.bo);
    printf("buffer: stride %u modifier 0x%llx\n", buffer.stride, (unsigned long long)buffer.modifier);

    std::mt19937 random(1234);
    std::vector<uint32_t> pixels(size_t(W) * H);
    for (auto &pixel : pixels) {
        pixel = 0xff000000u | (random() & 0xffffffu);
    }
    // A few flat and extreme areas too.
    for (int y = 0; y < 128; ++y) {
        for (int x = 0; x < 256; ++x) {
            pixels[size_t(y) * W + x] = 0xffffffffu;
            pixels[size_t(y + 200) * W + x] = 0xff000000u;
            pixels[size_t(y + 400) * W + x] = 0xffff0000u;
        }
    }

    GpuAvc444Converter converter;
    GpuAvc444Picture picture;
    int failures = 0;

    if (!fill(buffer, pixels, gbm)) {
        printf("cannot map the GBM buffer for writing\n");
        return 2;
    }
    if (!converter.convert(frameFor(buffer), picture)) {
        printf("first conversion failed\n");
        return 2;
    }
    printf("first frame:\n");
    failures += compare(picture, pixels);
    int marked = 0;
    for (uint8_t t : picture.tiles) {
        marked += t == 3;
    }
    printf("tiles marked as changed in both pictures: %d of %zu (expected all)\n", marked, picture.tiles.size());
    failures += marked == int(picture.tiles.size()) ? 0 : 1;

    // Second frame: one pixel brighter in tile (3, 2), and one pixel with the same luma but
    // a different colour in tile (30, 20).
    pixels[size_t(2 * 64 + 5) * W + 3 * 64 + 7] ^= 0x00ffffffu;
    const size_t colourOnly = size_t(20 * 64 + 9) * W + 30 * 64 + 11;
    pixels[colourOnly] = 0xff808080u;
    if (!fill(buffer, pixels, gbm) || !converter.convert(frameFor(buffer), picture)) {
        printf("second conversion failed\n");
        return 2;
    }
    pixels[colourOnly] = 0xff8a7c86u; // Y stays 128 within rounding, U and V change
    if (!fill(buffer, pixels, gbm) || !converter.convert(frameFor(buffer), picture)) {
        printf("third conversion failed\n");
        return 2;
    }
    printf("third frame:\n");
    failures += compare(picture, pixels);
    const int tilesPerRow = picture.tilesPerRow;
    for (size_t i = 0; i < picture.tiles.size(); ++i) {
        if (picture.tiles[i]) {
            printf("  tile (%zu, %zu) changed: luma %d chroma %d\n", i % tilesPerRow, i / tilesPerRow, picture.tiles[i] & 1, (picture.tiles[i] >> 1) & 1);
        }
    }
    const uint8_t expected = picture.tiles[20 * tilesPerRow + 30];
    int others = 0;
    for (size_t i = 0; i < picture.tiles.size(); ++i) {
        others += (i != size_t(20 * tilesPerRow + 30) && picture.tiles[i]) ? 1 : 0;
    }
    printf("tile (30, 20) bits %d, other tiles marked %d (expected chroma bit set, 0 others)\n", expected, others);
    failures += ((expected & 2) && others == 0) ? 0 : 1;

    // Timing.
    QElapsedTimer timer;
    timer.start();
    for (int i = 0; i < 100; ++i) {
        converter.convert(frameFor(buffer), picture);
    }
    printf("convert + read back: %.2f ms per frame\n", timer.nsecsElapsed() / 1e6 / 100);
    timer.restart();
    for (int i = 0; i < 100; ++i) {
        converter.convert(frameFor(buffer), picture, GpuAvc444Converter::Output::TilesOnly);
    }
    printf("convert + tiles only: %.2f ms per frame\n", timer.nsecsElapsed() / 1e6 / 100);

    // End to end on a desktop-like picture: a gradient with one-pixel coloured lines, which
    // is where 4:4:4 matters. Convert into CUDA frames, encode with NVENC, decode with
    // FreeRDP's AVC444 decoder, and compare with the source.
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            uint32_t r = (x * 255) / W;
            uint32_t g = (y * 255) / H;
            uint32_t b = 128;
            if (x % 7 == 0) {
                r = 255, g = 0, b = 0;
            }
            if (y % 5 == 0) {
                r = 0, g = 0, b = 255;
            }
            pixels[size_t(y) * W + x] = 0xff000000u | (r << 16) | (g << 8) | b;
        }
    }
    GpuAvc444Converter cudaConverter;
    if (!cudaConverter.cudaAvailable()) {
        printf("CUDA output not available\n");
        return 1;
    }
    GpuAvc444Picture cudaPicture;
    if (!fill(buffer, pixels, gbm) || !cudaConverter.convert(frameFor(buffer), cudaPicture, GpuAvc444Converter::Output::Cuda) || !cudaPicture.lumaFrame
        || !cudaPicture.chromaFrame) {
        printf("CUDA conversion failed\n");
        return 1;
    }
    GpuH264Encoder encoder;
    QByteArray lumaStream;
    QByteArray chromaStream;
    if (!encoder.ensure(cudaPicture.lumaFrame.get(), 12, 60, GpuH264Encoder::Speed::Fast) || !encoder.encode(cudaPicture.lumaFrame.get(), lumaStream)
        || !encoder.encode(cudaPicture.chromaFrame.get(), chromaStream)) {
        printf("NVENC encoding failed\n");
        return 1;
    }
    printf("NVENC: luma %lld bytes, chroma %lld bytes\n", (long long)lumaStream.size(), (long long)chromaStream.size());

    const auto decode = [&](bool withChroma, double &psnr, int &maxDiff) {
        H264_CONTEXT *decoder = h264_context_new(FALSE);
        h264_context_reset(decoder, W, H);
        std::vector<uint8_t> out(size_t(W) * H * 4);
        const RECTANGLE_16 full = {0, 0, W, H};
        const INT32 rc = avc444_decompress(decoder,
                                           withChroma ? 0 : 1,
                                           &full,
                                           1,
                                           reinterpret_cast<const BYTE *>(lumaStream.constData()),
                                           UINT32(lumaStream.size()),
                                           withChroma ? &full : nullptr,
                                           withChroma ? 1 : 0,
                                           withChroma ? reinterpret_cast<const BYTE *>(chromaStream.constData()) : nullptr,
                                           withChroma ? UINT32(chromaStream.size()) : 0,
                                           out.data(),
                                           PIXEL_FORMAT_BGRX32,
                                           W * 4,
                                           W,
                                           H,
                                           RDPGFX_CODECID_AVC444v2);
        h264_context_free(decoder);
        if (rc < 0) {
            return false;
        }
        double sum = 0;
        maxDiff = 0;
        for (size_t i = 0; i < size_t(W) * H; ++i) {
            for (int c = 0; c < 3; ++c) {
                const int a = (pixels[i] >> (8 * c)) & 0xff;
                const int b = out[i * 4 + c];
                sum += double(a - b) * (a - b);
                maxDiff = std::max(maxDiff, std::abs(a - b));
            }
        }
        psnr = 10 * std::log10(255.0 * 255.0 / (sum / (double(W) * H * 3)));
        return true;
    };
    double psnr444 = 0;
    double psnr420 = 0;
    int max444 = 0;
    int max420 = 0;
    if (!decode(true, psnr444, max444) || !decode(false, psnr420, max420)) {
        printf("FreeRDP could not decode the stream\n");
        return 1;
    }
    printf("decoded 4:4:4: PSNR %.1f dB, max error %d; luma picture only (4:2:0): PSNR %.1f dB, max error %d\n", psnr444, max444, psnr420, max420);
    failures += (psnr444 > psnr420 + 3) ? 0 : 1;

    // The same picture through FreeRDP's own path (CPU conversion, FreeRDP's NVENC setup),
    // as the reference for what the stream should look like.
    {
        H264_CONTEXT *cpuEncoder = h264_context_new(TRUE);
        h264_context_set_option(cpuEncoder, H264_CONTEXT_OPTION_ENCODER, H264_ENCODER_NVENC);
        h264_context_set_option(cpuEncoder, H264_CONTEXT_OPTION_ENCODER_SPEED, H264_ENCODER_SPEED_FAST);
        h264_context_reset(cpuEncoder, W, H);
        h264_context_set_option(cpuEncoder, H264_CONTEXT_OPTION_RATECONTROL, H264_RATECONTROL_CQP);
        h264_context_set_option(cpuEncoder, H264_CONTEXT_OPTION_QP, 12);
        h264_context_set_option(cpuEncoder, H264_CONTEXT_OPTION_FRAMERATE, 60);
        const RECTANGLE_16 full = {0, 0, W, H};
        BYTE op = 0;
        BYTE *a = nullptr;
        BYTE *b = nullptr;
        UINT32 aSize = 0;
        UINT32 bSize = 0;
        RDPGFX_H264_METABLOCK m1 = {};
        RDPGFX_H264_METABLOCK m2 = {};
        const INT32 rc = avc444_compress(cpuEncoder, reinterpret_cast<const BYTE *>(pixels.data()), PIXEL_FORMAT_BGRX32, W * 4, W, H, 2, &full, &op, &a, &aSize, &b, &bSize, &m1, &m2);
        if (rc > 0) {
            lumaStream = QByteArray(reinterpret_cast<const char *>(a), aSize);
            chromaStream = QByteArray(reinterpret_cast<const char *>(b), bSize);
            double p444 = 0;
            double p420 = 0;
            int m444 = 0;
            int m420 = 0;
            decode(true, p444, m444);
            decode(false, p420, m420);
            printf("FreeRDP path: luma %u bytes, chroma %u bytes; decoded 4:4:4 PSNR %.1f dB max %d; 4:2:0 PSNR %.1f dB max %d\n", aSize, bSize, p444, m444, p420, m420);
        } else {
            printf("FreeRDP avc444_compress failed (%d)\n", rc);
        }
        free_h264_metablock(&m1);
        free_h264_metablock(&m2);
        h264_context_free(cpuEncoder);
    }

    timer.restart();
    for (int i = 0; i < 100; ++i) {
        cudaConverter.convert(frameFor(buffer), cudaPicture, GpuAvc444Converter::Output::Cuda);
    }
    printf("convert into CUDA frames: %.2f ms per frame\n", timer.nsecsElapsed() / 1e6 / 100);
    timer.restart();
    for (int i = 0; i < 100; ++i) {
        encoder.encode(cudaPicture.lumaFrame.get(), lumaStream);
        encoder.encode(cudaPicture.chromaFrame.get(), chromaStream);
    }
    printf("NVENC luma + chroma: %.2f ms per frame\n", timer.nsecsElapsed() / 1e6 / 100);
    {
        // No periodic key frames: 300 more frames (600 pictures) must not contain an IDR
        // slice (NAL type 5). A key frame is a burst of data that delays the frames after it.
        const auto hasNal = [](const QByteArray &data, int type) {
            for (qsizetype i = 0; i + 3 < data.size(); ++i) {
                if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1 && (data[i + 3] & 0x1f) == type) {
                    return true;
                }
            }
            return false;
        };
        const auto hasIdr = [&](const QByteArray &data) {
            return hasNal(data, 5);
        };
        int idrPictures = 0;
        for (int i = 0; i < 300; ++i) {
            encoder.encode(cudaPicture.lumaFrame.get(), lumaStream);
            idrPictures += hasIdr(lumaStream);
            encoder.encode(cudaPicture.chromaFrame.get(), chromaStream);
            idrPictures += hasIdr(chromaStream);
        }
        printf("IDR pictures in 600 after the start: %d\n", idrPictures);
        if (idrPictures != 0) {
            ++failures;
        }
        // A requested key frame: the next picture is IDR with SPS (NAL type 7), the one after
        // it is not, and the generation moves on.
        const quint64 generation = encoder.generation();
        encoder.requestKeyFrame();
        encoder.encode(cudaPicture.lumaFrame.get(), lumaStream);
        encoder.encode(cudaPicture.chromaFrame.get(), chromaStream);
        const bool keyOk = hasIdr(lumaStream) && hasNal(lumaStream, 7) && !hasIdr(chromaStream) && encoder.generation() == generation + 1;
        printf("requested key frame: luma IDR %d SPS %d, chroma IDR %d: %s\n", hasIdr(lumaStream), hasNal(lumaStream, 7), hasIdr(chromaStream), keyOk ? "ok" : "WRONG");
        // A new decoder, as mstsc has after a reset, starts from the requested key frame.
        double keyPsnr = 0;
        int keyMax = 0;
        const bool keyDecoded = decode(true, keyPsnr, keyMax);
        printf("fresh decoder from the requested key frame: %s, PSNR %.1f dB\n", keyDecoded ? "decoded" : "FAILED", keyPsnr);
        if (!keyOk || !keyDecoded || keyPsnr < psnr444 - 1) {
            ++failures;
        }
    }
    for (auto speed : {GpuH264Encoder::Speed::Default, GpuH264Encoder::Speed::Fast, GpuH264Encoder::Speed::Fastest}) {
        GpuH264Encoder presetEncoder;
        presetEncoder.ensure(cudaPicture.lumaFrame.get(), 22, 60, speed);
        // Alternate the detailed frame with a slightly changed copy, as a desktop would.
        timer.restart();
        for (int i = 0; i < 60; ++i) {
            presetEncoder.encode(cudaPicture.lumaFrame.get(), lumaStream);
            presetEncoder.encode(cudaPicture.chromaFrame.get(), chromaStream);
        }
        printf("preset %d (QP 22): %.2f ms per frame, last luma %lld bytes\n", int(speed), timer.nsecsElapsed() / 1e6 / 60, (long long)lumaStream.size());
    }

    printf(failures ? "FAILED\n" : "PASSED\n");
    return failures ? 1 : 0;
}
