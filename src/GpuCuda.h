// SPDX-FileCopyrightText: 2026 Øystein Krog <oystein.krog@gmail.com>
//
// SPDX-License-Identifier: LGPL-2.1-only OR LGPL-3.0-only OR LicenseRef-KDE-Accepted-LGPL

#pragma once

#include <cuda.h>
#include <cudaGL.h>

namespace KRdp
{

/**
 * The few CUDA driver calls the GPU encode path needs, loaded from libcuda.so.1 at run time
 * so that KRDP still starts on machines without the NVIDIA driver.
 */
struct GpuCuda {
    /// The loaded functions, or nullptr if libcuda or one of the calls is missing.
    static const GpuCuda *get();

    decltype(&::cuInit) init;
    decltype(&::cuGLGetDevices) glGetDevices;
    decltype(&::cuDevicePrimaryCtxRetain) devicePrimaryCtxRetain;
    decltype(&::cuDevicePrimaryCtxRelease) devicePrimaryCtxRelease;
    decltype(&::cuCtxPushCurrent) ctxPushCurrent;
    decltype(&::cuCtxPopCurrent) ctxPopCurrent;
    decltype(&::cuGraphicsGLRegisterBuffer) graphicsGLRegisterBuffer;
    decltype(&::cuGraphicsUnregisterResource) graphicsUnregisterResource;
    decltype(&::cuGraphicsMapResources) graphicsMapResources;
    decltype(&::cuGraphicsUnmapResources) graphicsUnmapResources;
    decltype(&::cuGraphicsResourceGetMappedPointer) graphicsResourceGetMappedPointer;
    decltype(&::cuMemcpy2DAsync) memcpy2DAsync;
    decltype(&::cuStreamSynchronize) streamSynchronize;
    decltype(&::cuGetErrorName) getErrorName;

    const char *errorName(CUresult result) const;
};

}
