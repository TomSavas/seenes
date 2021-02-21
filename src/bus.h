#ifndef BUS_H
#define BUS_H

#include <stdint.h>

struct mmio;
struct bus
{
    struct mmio **devices;

    int devices_size;
    int devices_count;
};

struct bus init_bus(int device_count_hint);

void attach(struct bus *bus, struct mmio *mmio);
void detach(struct bus *bus, struct mmio *mmio);

uint8_t bus_read(struct bus *bus, uint16_t addr);
void bus_write(struct bus *bus, uint16_t addr, uint8_t val);

#endif
