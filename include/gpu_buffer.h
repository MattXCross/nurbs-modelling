#pragma once

#include <cstddef>
#include <cstdint>

class GpuVertexBuffer {
private:
    uint32_t m_handle{0};
    uint32_t m_size{0};

public:
    explicit GpuVertexBuffer(size_t size);
    ~GpuVertexBuffer() noexcept;

    GpuVertexBuffer(const GpuVertexBuffer&) = delete;
    GpuVertexBuffer& operator=(const GpuVertexBuffer&) = delete;

    GpuVertexBuffer(GpuVertexBuffer&& other) noexcept;
    GpuVertexBuffer& operator=(GpuVertexBuffer&& other) noexcept;

    [[nodiscard]] uint32_t id() const { return m_handle; }
    [[nodiscard]] size_t size() const { return m_size; }
};