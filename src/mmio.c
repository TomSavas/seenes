#include <assert.h>
#include <stdlib.h>

#include "mmio.h"

bool mmio_in_managed_range(struct mmio *mmio, uint16_t addr)
{
    return addr >= mmio->managed_memory_start && addr <= mmio->managed_memory_end;
}

uint8_t mmio_default_read(struct mmio *mmio, uint16_t addr)
{
    assert(mmio_in_managed_range(mmio, addr));

    addr -= mmio->managed_memory_start;
    return mmio->data[addr];
}

void mmio_default_write(struct mmio *mmio, uint16_t addr, uint8_t val)
{
    mmio->data[addr - mmio->managed_memory_start] = val;
}

struct mmio make_mmio(uint16_t memory_start, uint16_t memory_end, uint8_t *data)
{
    return (struct mmio)
    {
        .type = MMIO,

        .managed_memory_start = memory_start,
        .managed_memory_end = memory_end,
        .data = data,

        .mmio_read = mmio_default_read,
        .mmio_write = mmio_default_write
    };
}

uint8_t mmio_read(struct mmio *mmio, uint16_t addr)
{
    return mmio->mmio_read(mmio, addr);
}

void mmio_write(struct mmio *mmio, uint16_t addr, uint8_t val)
{
    return mmio->mmio_write(mmio, addr, val);
}

uint16_t mirrored_addr(struct mirrored_mmio *m_mmio, uint16_t addr)
{
    // Remaps any address in the range [mmio.managed_memory_start; mmio.managed_memory_end]
    // to [m_mmio.actual_memory_start; m_mmio.actual_memory_end]
    return ((addr % m_mmio->mirror_size) + m_mmio->actual_memory_start) % (m_mmio->actual_memory_end + 1);
}

uint8_t mirrored_mmio_read(struct mmio *mmio, uint16_t addr)
{
    assert(mmio->type == MIRRORED_MMIO);

    struct mirrored_mmio *m_mmio = (struct mirrored_mmio*)mmio;
    return mmio_default_read(mmio, mirrored_addr(m_mmio, addr));
}

void mirrored_mmio_write(struct mmio *mmio, uint16_t addr, uint8_t val)
{
    assert(mmio->type == MIRRORED_MMIO);

    struct mirrored_mmio *m_mmio = (struct mirrored_mmio*)mmio;
    return mmio_default_write(mmio, mirrored_addr(m_mmio, addr), val);
}

struct mirrored_mmio make_mirrored_mmio(struct mmio mmio, uint16_t actual_memory_start, uint16_t actual_memory_end)
{
    uint16_t size = mmio.managed_memory_end - mmio.managed_memory_start + 1;
    uint16_t mirror_size = actual_memory_end - actual_memory_start + 1;
    // General case scenario we want to have an integer amount of mirrors,
    // if not, we need a separate solution on how we are mirroring different regions
    assert(size % mirror_size == 0);

    mmio.type = MIRRORED_MMIO;
    mmio.mmio_read = mirrored_mmio_read;
    mmio.mmio_write = mirrored_mmio_write;

    return (struct mirrored_mmio)
    {
        .mmio = mmio,
        .mirror_size = mirror_size,
        .actual_memory_start = actual_memory_start,
        .actual_memory_end = actual_memory_end
    };
}
