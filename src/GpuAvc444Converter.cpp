// SPDX-FileCopyrightText: 2026 Øystein Krog <oystein.krog@gmail.com>
//
// SPDX-License-Identifier: LGPL-2.1-only OR LGPL-3.0-only OR LicenseRef-KDE-Accepted-LGPL

#include "GpuAvc444Converter.h"

#include <fcntl.h>
#include <unistd.h>

#include <drm_fourcc.h>
#include <epoxy/egl.h>
#include <epoxy/gl.h>
#include <gbm.h>

#include <QDir>
#include <QList>

#include <PipeWireSourceStream>

#include "krdp_logging.h"

namespace KRdp
{

namespace
{
constexpr int TileSize = 64;
// Each shader invocation converts 16 x 2 pixels, so it writes whole 32-bit words to every
// plane of both pictures (see sizeSupported() for the alignment this needs).
constexpr int BlockWidth = 16;
constexpr int BlockHeight = 2;
constexpr int GroupWidth = 8;
constexpr int GroupHeight = 8;

// The conversion follows FreeRDP's general_RGBToAVC444YUVv2 (prim_YUV.c) exactly: the
// RGB2Y/RGB2U/RGB2V integer formulas from prim_internal.h, luma U and V as the truncated
// average of each 2x2 block, and the chroma picture's B4 to B9 areas.
constexpr const char *ShaderSource = R"(#version 430
layout(local_size_x = 8, local_size_y = 8) in;

layout(binding = 0) uniform sampler2D source;
layout(std430, binding = 1) writeonly buffer LumaOut { uint lumaOut[]; };
layout(std430, binding = 2) writeonly buffer ChromaOut { uint chromaOut[]; };
layout(std430, binding = 3) readonly buffer LumaPrev { uint lumaPrev[]; };
layout(std430, binding = 4) readonly buffer ChromaPrev { uint chromaPrev[]; };
layout(std430, binding = 5) buffer Tiles { uint tiles[]; };

uniform ivec2 size;
uniform int tilesPerRow;

bool lumaChanged = false;
bool chromaChanged = false;

uint pack4(int a, int b, int c, int d)
{
    return uint(a) | (uint(b) << 8) | (uint(c) << 16) | (uint(d) << 24);
}

void putLuma(int byteOffset, uint word)
{
    const int i = byteOffset / 4;
    lumaOut[i] = word;
    lumaChanged = lumaChanged || lumaPrev[i] != word;
}

void putChroma(int byteOffset, uint word)
{
    const int i = byteOffset / 4;
    chromaOut[i] = word;
    chromaChanged = chromaChanged || chromaPrev[i] != word;
}

void main()
{
    const int x0 = int(gl_GlobalInvocationID.x) * 16;
    const int y = int(gl_GlobalInvocationID.y) * 2;
    if (x0 >= size.x || y >= size.y) {
        return;
    }
    const int width = size.x;
    const int ySize = width * size.y;
    const int uvSize = ySize / 4;
    const int uvStride = width / 2;

    int Y[2][16];
    int U[2][16];
    int V[2][16];
    for (int r = 0; r < 2; ++r) {
        for (int c = 0; c < 16; ++c) {
            const ivec3 rgb = ivec3(round(texelFetch(source, ivec2(x0 + c, y + r), 0).rgb * 255.0));
            Y[r][c] = (54 * rgb.r + 183 * rgb.g + 18 * rgb.b) >> 8;
            U[r][c] = ((-29 * rgb.r - 99 * rgb.g + 128 * rgb.b) >> 8) + 128;
            V[r][c] = ((128 * rgb.r - 116 * rgb.g - 12 * rgb.b) >> 8) + 128;
        }
    }

    // Luma picture (B1 to B3).
    for (int r = 0; r < 2; ++r) {
        for (int w = 0; w < 4; ++w) {
            putLuma((y + r) * width + x0 + 4 * w, pack4(Y[r][4 * w], Y[r][4 * w + 1], Y[r][4 * w + 2], Y[r][4 * w + 3]));
        }
    }
    int avgU[8];
    int avgV[8];
    for (int i = 0; i < 8; ++i) {
        avgU[i] = (U[0][2 * i] + U[0][2 * i + 1] + U[1][2 * i] + U[1][2 * i + 1]) / 4;
        avgV[i] = (V[0][2 * i] + V[0][2 * i + 1] + V[1][2 * i] + V[1][2 * i + 1]) / 4;
    }
    const int uvRow = (y / 2) * uvStride + x0 / 2;
    putLuma(ySize + uvRow, pack4(avgU[0], avgU[1], avgU[2], avgU[3]));
    putLuma(ySize + uvRow + 4, pack4(avgU[4], avgU[5], avgU[6], avgU[7]));
    putLuma(ySize + uvSize + uvRow, pack4(avgV[0], avgV[1], avgV[2], avgV[3]));
    putLuma(ySize + uvSize + uvRow + 4, pack4(avgV[4], avgV[5], avgV[6], avgV[7]));

    // Chroma picture. B4 and B5: U and V of the odd columns, every row, in the left and
    // right half of the Y plane.
    for (int r = 0; r < 2; ++r) {
        const int row = (y + r) * width + x0 / 2;
        putChroma(row, pack4(U[r][1], U[r][3], U[r][5], U[r][7]));
        putChroma(row + 4, pack4(U[r][9], U[r][11], U[r][13], U[r][15]));
        putChroma(row + width / 2, pack4(V[r][1], V[r][3], V[r][5], V[r][7]));
        putChroma(row + width / 2 + 4, pack4(V[r][9], V[r][11], V[r][13], V[r][15]));
    }
    // B6 to B9: U and V of the odd rows, columns 4k (U plane) and 4k + 2 (V plane), with
    // U in the left quarter and V in the right quarter of each row.
    const int quarterRow = (y / 2) * uvStride + x0 / 4;
    putChroma(ySize + quarterRow, pack4(U[1][0], U[1][4], U[1][8], U[1][12]));
    putChroma(ySize + quarterRow + width / 4, pack4(V[1][0], V[1][4], V[1][8], V[1][12]));
    putChroma(ySize + uvSize + quarterRow, pack4(U[1][2], U[1][6], U[1][10], U[1][14]));
    putChroma(ySize + uvSize + quarterRow + width / 4, pack4(V[1][2], V[1][6], V[1][10], V[1][14]));

    const uint bits = (lumaChanged ? 1u : 0u) | (chromaChanged ? 2u : 0u);
    if (bits != 0u) {
        atomicOr(tiles[(y / 64) * tilesPerRow + x0 / 64], bits);
    }
}
)";

QByteArray eglErrorString()
{
    return QByteArray::number(eglGetError(), 16);
}
}

class GpuAvc444Converter::Private
{
public:
    ~Private();
    bool initialize();
    bool ensureBuffers(const QSize &size);
    GLuint importTexture(const DmaBufAttributes &dmabuf, EGLImage *image);

    bool initialized = false;
    bool failed = false;

    int drmFd = -1;
    gbm_device *gbm = nullptr;
    EGLDisplay display = EGL_NO_DISPLAY;
    EGLContext context = EGL_NO_CONTEXT;
    GLuint program = 0;
    GLint sizeUniform = -1;
    GLint tilesPerRowUniform = -1;

    QSize bufferSize;
    GLuint lumaBuffers[2] = {};
    GLuint chromaBuffers[2] = {};
    GLuint tileBuffer = 0;
    int current = 0; ///< index of the buffers the next conversion writes
    bool havePrevious = false;
};

GpuAvc444Converter::Private::~Private()
{
    if (context != EGL_NO_CONTEXT) {
        eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, context);
        glDeleteBuffers(2, lumaBuffers);
        glDeleteBuffers(2, chromaBuffers);
        glDeleteBuffers(1, &tileBuffer);
        glDeleteProgram(program);
        eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        eglDestroyContext(display, context);
    }
    if (display != EGL_NO_DISPLAY) {
        eglTerminate(display);
    }
    if (gbm) {
        gbm_device_destroy(gbm);
    }
    if (drmFd >= 0) {
        close(drmFd);
    }
}

bool GpuAvc444Converter::Private::initialize()
{
    if (initialized || failed) {
        return initialized;
    }
    failed = true; // until everything below worked

    // The same route as KPipeWire's DmaBufHandler: a GBM display on the render node.
    const QStringList nodes = QDir(QStringLiteral("/dev/dri")).entryList({QStringLiteral("renderD*")}, QDir::System, QDir::Name);
    if (nodes.isEmpty()) {
        qCWarning(KRDP) << "GPU AVC444: no render node";
        return false;
    }
    const QByteArray node = QByteArrayLiteral("/dev/dri/") + nodes.first().toLatin1();
    drmFd = open(node.constData(), O_RDWR | O_CLOEXEC);
    if (drmFd < 0) {
        qCWarning(KRDP) << "GPU AVC444: cannot open" << node;
        return false;
    }
    gbm = gbm_create_device(drmFd);
    if (!gbm) {
        qCWarning(KRDP) << "GPU AVC444: cannot create a GBM device";
        return false;
    }
    display = eglGetPlatformDisplayEXT(EGL_PLATFORM_GBM_KHR, gbm, nullptr);
    if (display == EGL_NO_DISPLAY || !eglInitialize(display, nullptr, nullptr)) {
        qCWarning(KRDP) << "GPU AVC444: cannot initialize EGL" << eglErrorString();
        return false;
    }
    if (!epoxy_has_egl_extension(display, "EGL_EXT_image_dma_buf_import_modifiers") || !epoxy_has_egl_extension(display, "EGL_KHR_surfaceless_context")
        || !epoxy_has_egl_extension(display, "EGL_KHR_no_config_context")) {
        qCWarning(KRDP) << "GPU AVC444: EGL lacks DMA-BUF import, surfaceless or no-config contexts";
        return false;
    }
    if (!eglBindAPI(EGL_OPENGL_API)) {
        qCWarning(KRDP) << "GPU AVC444: cannot bind OpenGL";
        return false;
    }
    const EGLint contextAttribs[] = {
        EGL_CONTEXT_MAJOR_VERSION,
        4,
        EGL_CONTEXT_MINOR_VERSION,
        5,
        EGL_CONTEXT_OPENGL_PROFILE_MASK,
        EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
        EGL_NONE,
    };
    context = eglCreateContext(display, EGL_NO_CONFIG_KHR, EGL_NO_CONTEXT, contextAttribs);
    if (context == EGL_NO_CONTEXT || !eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, context)) {
        qCWarning(KRDP) << "GPU AVC444: cannot create an OpenGL 4.5 context" << eglErrorString();
        return false;
    }
    if (!epoxy_has_gl_extension("GL_OES_EGL_image")) {
        qCWarning(KRDP) << "GPU AVC444: GL_OES_EGL_image is missing";
        return false;
    }

    const GLuint shader = glCreateShader(GL_COMPUTE_SHADER);
    glShaderSource(shader, 1, &ShaderSource, nullptr);
    glCompileShader(shader);
    GLint ok = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[4096] = {};
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        qCWarning(KRDP) << "GPU AVC444: shader does not compile:" << log;
        glDeleteShader(shader);
        return false;
    }
    program = glCreateProgram();
    glAttachShader(program, shader);
    glLinkProgram(program);
    glDeleteShader(shader);
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[4096] = {};
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        qCWarning(KRDP) << "GPU AVC444: shader does not link:" << log;
        return false;
    }
    sizeUniform = glGetUniformLocation(program, "size");
    tilesPerRowUniform = glGetUniformLocation(program, "tilesPerRow");

    qCDebug(KRDP) << "GPU AVC444: ready on" << node << reinterpret_cast<const char *>(glGetString(GL_RENDERER));
    failed = false;
    initialized = true;
    return true;
}

bool GpuAvc444Converter::Private::ensureBuffers(const QSize &size)
{
    if (size == bufferSize) {
        return true;
    }
    glDeleteBuffers(2, lumaBuffers);
    glDeleteBuffers(2, chromaBuffers);
    glDeleteBuffers(1, &tileBuffer);

    const GLsizeiptr pictureSize = GLsizeiptr(size.width()) * size.height() * 3 / 2;
    const GLsizeiptr tileCount = GLsizeiptr((size.width() + TileSize - 1) / TileSize) * ((size.height() + TileSize - 1) / TileSize);
    glCreateBuffers(2, lumaBuffers);
    glCreateBuffers(2, chromaBuffers);
    glCreateBuffers(1, &tileBuffer);
    for (int i = 0; i < 2; ++i) {
        glNamedBufferStorage(lumaBuffers[i], pictureSize, nullptr, GL_DYNAMIC_STORAGE_BIT);
        glNamedBufferStorage(chromaBuffers[i], pictureSize, nullptr, GL_DYNAMIC_STORAGE_BIT);
    }
    glNamedBufferStorage(tileBuffer, tileCount * sizeof(GLuint), nullptr, GL_DYNAMIC_STORAGE_BIT);
    if (glGetError() != GL_NO_ERROR) {
        qCWarning(KRDP) << "GPU AVC444: cannot allocate buffers for" << size;
        bufferSize = QSize();
        return false;
    }
    bufferSize = size;
    havePrevious = false;
    return true;
}

GLuint GpuAvc444Converter::Private::importTexture(const DmaBufAttributes &dmabuf, EGLImage *image)
{
    static constexpr EGLint planeAttribs[4][5] = {
        {EGL_DMA_BUF_PLANE0_FD_EXT, EGL_DMA_BUF_PLANE0_OFFSET_EXT, EGL_DMA_BUF_PLANE0_PITCH_EXT, EGL_DMA_BUF_PLANE0_MODIFIER_LO_EXT, EGL_DMA_BUF_PLANE0_MODIFIER_HI_EXT},
        {EGL_DMA_BUF_PLANE1_FD_EXT, EGL_DMA_BUF_PLANE1_OFFSET_EXT, EGL_DMA_BUF_PLANE1_PITCH_EXT, EGL_DMA_BUF_PLANE1_MODIFIER_LO_EXT, EGL_DMA_BUF_PLANE1_MODIFIER_HI_EXT},
        {EGL_DMA_BUF_PLANE2_FD_EXT, EGL_DMA_BUF_PLANE2_OFFSET_EXT, EGL_DMA_BUF_PLANE2_PITCH_EXT, EGL_DMA_BUF_PLANE2_MODIFIER_LO_EXT, EGL_DMA_BUF_PLANE2_MODIFIER_HI_EXT},
        {EGL_DMA_BUF_PLANE3_FD_EXT, EGL_DMA_BUF_PLANE3_OFFSET_EXT, EGL_DMA_BUF_PLANE3_PITCH_EXT, EGL_DMA_BUF_PLANE3_MODIFIER_LO_EXT, EGL_DMA_BUF_PLANE3_MODIFIER_HI_EXT},
    };
    QList<EGLint> attribs{EGL_WIDTH, dmabuf.width, EGL_HEIGHT, dmabuf.height, EGL_LINUX_DRM_FOURCC_EXT, EGLint(dmabuf.format)};
    for (int i = 0; i < std::min<qsizetype>(dmabuf.planes.size(), 4); ++i) {
        const auto &plane = dmabuf.planes[i];
        attribs << planeAttribs[i][0] << plane.fd << planeAttribs[i][1] << EGLint(plane.offset) << planeAttribs[i][2] << EGLint(plane.stride);
        if (dmabuf.modifier != DRM_FORMAT_MOD_INVALID) {
            attribs << planeAttribs[i][3] << EGLint(dmabuf.modifier & 0xffffffff) << planeAttribs[i][4] << EGLint(dmabuf.modifier >> 32);
        }
    }
    attribs << EGL_NONE;

    *image = eglCreateImageKHR(display, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, nullptr, attribs.constData());
    if (*image == EGL_NO_IMAGE_KHR) {
        qCWarning(KRDP) << "GPU AVC444: cannot import the DMA-BUF" << eglErrorString();
        return 0;
    }
    GLuint texture = 0;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, *image);
    return texture;
}

GpuAvc444Converter::GpuAvc444Converter()
    : d(std::make_unique<Private>())
{
}

GpuAvc444Converter::~GpuAvc444Converter() = default;

bool GpuAvc444Converter::sizeSupported(const QSize &size)
{
    return size.width() > 0 && size.height() > 0 && size.width() % 32 == 0 && size.height() % 2 == 0;
}

void GpuAvc444Converter::reset()
{
    d->havePrevious = false;
}

bool GpuAvc444Converter::convert(const PipeWireFrame &frame, GpuAvc444Picture &picture)
{
    if (!frame.dmabuf || frame.dmabuf->planes.isEmpty()) {
        return false;
    }
    const QSize size(frame.dmabuf->width, frame.dmabuf->height);
    if (!sizeSupported(size) || !d->initialize()) {
        return false;
    }
    if (!eglMakeCurrent(d->display, EGL_NO_SURFACE, EGL_NO_SURFACE, d->context)) {
        qCWarning(KRDP) << "GPU AVC444: cannot make the context current" << eglErrorString();
        return false;
    }
    if (!d->ensureBuffers(size)) {
        return false;
    }

    EGLImage image = EGL_NO_IMAGE_KHR;
    const GLuint texture = d->importTexture(*frame.dmabuf, &image);
    if (!texture) {
        return false;
    }

    const int tilesPerRow = (size.width() + TileSize - 1) / TileSize;
    const int tileRows = (size.height() + TileSize - 1) / TileSize;
    const GLuint zero = 0;
    glClearNamedBufferData(d->tileBuffer, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, &zero);

    const int cur = d->current;
    const int prev = 1 - cur;
    glUseProgram(d->program);
    glUniform2i(d->sizeUniform, size.width(), size.height());
    glUniform1i(d->tilesPerRowUniform, tilesPerRow);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, d->lumaBuffers[cur]);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, d->chromaBuffers[cur]);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, d->lumaBuffers[prev]);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, d->chromaBuffers[prev]);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, d->tileBuffer);
    const int blocksX = size.width() / BlockWidth;
    const int blocksY = size.height() / BlockHeight;
    glDispatchCompute((blocksX + GroupWidth - 1) / GroupWidth, (blocksY + GroupHeight - 1) / GroupHeight, 1);
    glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);

    // Reading back waits for the shader, so the DMA-BUF is no longer used when this returns
    // and KPipeWire can hand it back to KWin.
    const qsizetype pictureSize = qsizetype(size.width()) * size.height() * 3 / 2;
    std::vector<GLuint> tileWords(size_t(tilesPerRow) * tileRows);
    glGetNamedBufferSubData(d->tileBuffer, 0, GLsizeiptr(tileWords.size() * sizeof(GLuint)), tileWords.data());
    picture.luma.resize(pictureSize);
    picture.chroma.resize(pictureSize);
    glGetNamedBufferSubData(d->lumaBuffers[cur], 0, pictureSize, picture.luma.data());
    glGetNamedBufferSubData(d->chromaBuffers[cur], 0, pictureSize, picture.chroma.data());

    glBindTexture(GL_TEXTURE_2D, 0);
    glDeleteTextures(1, &texture);
    eglDestroyImageKHR(d->display, image);

    if (const GLenum error = glGetError(); error != GL_NO_ERROR) {
        qCWarning(KRDP) << "GPU AVC444: GL error" << Qt::hex << error;
        d->havePrevious = false;
        return false;
    }

    picture.size = size;
    picture.tilesPerRow = tilesPerRow;
    picture.tiles.resize(tileWords.size());
    for (size_t i = 0; i < tileWords.size(); ++i) {
        picture.tiles[i] = d->havePrevious ? uint8_t(tileWords[i]) : uint8_t(GpuAvc444Picture::LumaChanged | GpuAvc444Picture::ChromaChanged);
    }
    d->havePrevious = true;
    d->current = prev;
    return true;
}

}
