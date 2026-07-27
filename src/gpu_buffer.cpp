#include "gpu_buffer.h"

#include <cstddef>
#include <print>
#include <utility>

GpuVertexBuffer::GpuVertexBuffer(size_t size) : m_handle(1001), m_size(size) {
    std::println("GPU Buffer {} allocated ({} bytes)", m_handle, m_size);
}

GpuVertexBuffer::~GpuVertexBuffer() noexcept {
    if (m_handle != 0) {
        std::println("GPU Buffer {} freed on driver", m_handle);
    }
}

GpuVertexBuffer::GpuVertexBuffer(GpuVertexBuffer&& other) noexcept
    : m_handle(std::exchange(other.m_handle, 0)),
      m_size(std::exchange(other.m_size, 0)) {}

GpuVertexBuffer& GpuVertexBuffer::operator=(GpuVertexBuffer&& other) noexcept {
    if (this != &other) {
        if (m_handle != 0) {
            std::println("Overwritten GPU Buffer {} freed", m_handle);
        }

        m_handle = std::exchange(other.m_handle, 0);
        m_size = std::exchange(other.m_size, 0);
    }

    return *this;
}