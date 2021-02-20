#ifndef MMIO_H
#define MMIO_H

#include <stdint.h>
#include <stdbool.h>

// TODO: really stupid solution for type checking. Should be a better way to do this, right?
enum mmio_type
{
    MMIO = 0,
    MIRRORED_MMIO
};

struct mmio
{
    enum mmio_type type;

    uint16_t managed_memory_start;
    uint16_t managed_memory_end;

    uint8_t *data;

    uint8_t (*mmio_read)(struct mmio *mmio, uint16_t addr);
    void (*mmio_write)(struct mmio *mmio, uint16_t addr, uint8_t val);
};

struct mirrored_mmio
{
    struct mmio mmio;

    uint16_t mirror_size;
    uint16_t actual_memory_start;
    uint16_t actual_memory_end;
};

struct mirrored_mmio make_mirrored_mmio(struct mmio mmio, uint16_t actual_memory_start, uint16_t actual_memory_end);
//uint8_t mirrored_mmio_read(struct mmio *mmio, uint16_t addr);
//void mirrored_mmio_write(struct mmio *mmio, uint16_t addr, uint8_t val);

struct mmio make_mmio(uint16_t memory_start, uint16_t memory_end, uint8_t *data);

bool mmio_in_managed_range(struct mmio *mmio, uint16_t addr);

//uint8_t mmio_default_read(struct mmio *mmio, uint16_t addr);
//void mmio_default_write(struct mmio *mmio, uint16_t addr, uint8_t val);
uint8_t mmio_read(struct mmio *mmio, uint16_t addr);
void mmio_write(struct mmio *mmio, uint16_t addr, uint8_t val);

#endif
