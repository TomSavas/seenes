#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>

#include "bus.h"
#include "mmio.h"

struct bus init_bus(int device_count_hint)
{
    return (struct bus)
    {
        .devices = malloc(sizeof(struct mmio*) * device_count_hint),
        .devices_size = device_count_hint,
        .devices_count = 0
    };
}

void attach(struct bus *bus, struct mmio *mmio)
{
    assert(mmio->managed_memory_start < mmio->managed_memory_end);

    for (int i = 0; i < bus->devices_count; i++)
    {
        bool surrounds_existing_lower_bound = 
            mmio->managed_memory_start <= bus->devices[i]->managed_memory_start && 
            mmio->managed_memory_end >= bus->devices[i]->managed_memory_start;
        bool surrounds_existing_upper_bound =
            mmio->managed_memory_start <= bus->devices[i]->managed_memory_end && 
            mmio->managed_memory_end >= bus->devices[i]->managed_memory_end;
        bool inside_existing_bounds =
            mmio->managed_memory_start >= bus->devices[i]->managed_memory_start && mmio->managed_memory_start <= bus->devices[i]->managed_memory_end ||
            mmio->managed_memory_end >= bus->devices[i]->managed_memory_start && mmio->managed_memory_end <= bus->devices[i]->managed_memory_end;

        assert(!surrounds_existing_lower_bound && !surrounds_existing_upper_bound && !inside_existing_bounds);
    }

    if (bus->devices_count + 1 > bus->devices_size)
    {
        bus->devices_size *= 2;
        bus->devices = (struct mmio**)realloc(bus->devices, sizeof(struct mmio*) * bus->devices_size);
    }

    bus->devices[bus->devices_count++] = mmio;
}

void detach(struct bus *bus, struct mmio *mmio)
{
    for (int i = 0; i < bus->devices_count; i++)
    {
        if (bus->devices[i] == mmio)
        {
            bus->devices[i] = bus->devices[bus->devices_count--];
            break;
        }
    }
}

uint8_t bus_read(struct bus *bus, uint16_t addr)
{
    for (int i = 0; i < bus->devices_count; i++)
    {
        if (mmio_in_managed_range(bus->devices[i], addr))
            return mmio_read(bus->devices[i], addr);
    }

    assert(false);
}

void bus_write(struct bus *bus, uint16_t addr, uint8_t val)
{
    for (int i = 0; i < bus->devices_count; i++)
    {
        if (mmio_in_managed_range(bus->devices[i], addr))
        {
            mmio_write(bus->devices[i], addr, val);
            return;
        }
    }

    assert(false);
}
