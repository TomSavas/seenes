#include <stdio.h>
#include <stdlib.h>

#include "cartridge.h"
#include "mmio.h"
#include "bus.h"

struct cartridge cartridge_from_file(const char *path)
{
    struct cartridge c;

    FILE *rom = fopen(path, "r");
    size_t a = sizeof(struct ines_header);
    fread(&c.ines, a, 1, rom);

    if (c.ines.has_trainer != 0)
    {
        // Let's skip the trainer for now, we are unlikely to be using it
        fread(NULL, 512, 1, rom);
    }

    const size_t prg_rom_bank_size = 1 << 14;
    const size_t prg_rom_size = c.ines.prg_rom_16k_bank_count * prg_rom_bank_size;
    c.prg_rom = malloc(prg_rom_size);
    fread(c.prg_rom, prg_rom_size, 1, rom);

    const size_t chr_rom_bank_size = 1 << 13;
    const size_t chr_rom_size = c.ines.chr_rom_8k_bank_count * chr_rom_bank_size;
    c.chr_rom = malloc(chr_rom_size);
    fread(c.chr_rom, chr_rom_size, 1, rom);

    // Ignore playchoice stuff for now

    c.mapper.prg_rom = malloc(sizeof(struct mirrored_mmio));
    c.mapper.prg_rom->mmio.name = "prg_rom";
    *c.mapper.prg_rom = make_mirrored_mmio(make_mmio(0x4020, 0xFFFF, c.prg_rom), 0x0000, prg_rom_size - 1);

    c.mapper.chr_rom = malloc(sizeof(struct mirrored_mmio));
    c.mapper.chr_rom->mmio.name = "chr_rom";
    *c.mapper.chr_rom = make_mirrored_mmio(make_mmio(0x0000, 0x1FFF, c.chr_rom), 0x0000, chr_rom_size - 1);

    fclose(rom);

    return c;
}

void attach_cartridge(struct bus *cpu_bus, struct bus *ppu_bus, struct cartridge *cartridge)
{
    attach(cpu_bus, &cartridge->mapper.prg_rom->mmio);
    attach(ppu_bus, &cartridge->mapper.chr_rom->mmio);
}
