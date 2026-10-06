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

#include "../src/GpuAvc444Converter.h"

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
        converter.convert(frameFor(buffer), picture, false);
    }
    printf("convert + tiles only: %.2f ms per frame\n", timer.nsecsElapsed() / 1e6 / 100);

    printf(failures ? "FAILED\n" : "PASSED\n");
    return failures ? 1 : 0;
}
