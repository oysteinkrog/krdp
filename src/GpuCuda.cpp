// SPDX-FileCopyrightText: 2026 Øystein Krog <oystein.krog@gmail.com>
//
// SPDX-License-Identifier: LGPL-2.1-only OR LGPL-3.0-only OR LicenseRef-KDE-Accepted-LGPL

#include "GpuCuda.h"

#include <dlfcn.h>

#include "krdp_logging.h"

namespace KRdp
{

namespace
{
// cuda.h maps many calls to versioned names with macros (cuMemcpy2DAsync is
// cuMemcpy2DAsync_v2), so expand the macro before turning it into the symbol name.
#define KRDP_CUDA_NAME2(x) #x
#define KRDP_CUDA_NAME(x) KRDP_CUDA_NAME2(x)

GpuCuda *load()
{
    void *library = dlopen("libcuda.so.1", RTLD_NOW | RTLD_LOCAL);
    if (!library) {
        qCDebug(KRDP) << "GPU encode: no libcuda.so.1";
        return nullptr;
    }
    static GpuCuda cuda;
    bool ok = true;
    const auto resolve = [&](auto &function, const char *name) {
        function = reinterpret_cast<std::remove_reference_t<decltype(function)>>(dlsym(library, name));
        if (!function) {
            qCWarning(KRDP) << "GPU encode: libcuda lacks" << name;
            ok = false;
        }
    };
    resolve(cuda.init, KRDP_CUDA_NAME(cuInit));
    resolve(cuda.glGetDevices, KRDP_CUDA_NAME(cuGLGetDevices));
    resolve(cuda.devicePrimaryCtxRetain, KRDP_CUDA_NAME(cuDevicePrimaryCtxRetain));
    resolve(cuda.devicePrimaryCtxRelease, KRDP_CUDA_NAME(cuDevicePrimaryCtxRelease));
    resolve(cuda.ctxPushCurrent, KRDP_CUDA_NAME(cuCtxPushCurrent));
    resolve(cuda.ctxPopCurrent, KRDP_CUDA_NAME(cuCtxPopCurrent));
    resolve(cuda.graphicsGLRegisterBuffer, KRDP_CUDA_NAME(cuGraphicsGLRegisterBuffer));
    resolve(cuda.graphicsUnregisterResource, KRDP_CUDA_NAME(cuGraphicsUnregisterResource));
    resolve(cuda.graphicsMapResources, KRDP_CUDA_NAME(cuGraphicsMapResources));
    resolve(cuda.graphicsUnmapResources, KRDP_CUDA_NAME(cuGraphicsUnmapResources));
    resolve(cuda.graphicsResourceGetMappedPointer, KRDP_CUDA_NAME(cuGraphicsResourceGetMappedPointer));
    resolve(cuda.memcpy2DAsync, KRDP_CUDA_NAME(cuMemcpy2DAsync));
    resolve(cuda.streamSynchronize, KRDP_CUDA_NAME(cuStreamSynchronize));
    resolve(cuda.getErrorName, KRDP_CUDA_NAME(cuGetErrorName));
    if (!ok) {
        return nullptr;
    }
    if (const CUresult result = cuda.init(0); result != CUDA_SUCCESS) {
        qCWarning(KRDP) << "GPU encode: cuInit failed" << int(result);
        return nullptr;
    }
    return &cuda;
}
}

const GpuCuda *GpuCuda::get()
{
    static const GpuCuda *cuda = load();
    return cuda;
}

const char *GpuCuda::errorName(CUresult result) const
{
    const char *name = nullptr;
    if (getErrorName(result, &name) != CUDA_SUCCESS || !name) {
        return "unknown CUDA error";
    }
    return name;
}

}
