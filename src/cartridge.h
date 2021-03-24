#ifndef CARTRIDGE_H
#define CARTRIDGE_H

#include <stdint.h>
#include <stdbool.h>

enum mirroring
{
    HORIZONTAL = 0x0,
    VERTICAL   = 0x1,
};

enum tv_sys
{
    NTSC = 0x0,
    PAL  = 0x1
};

struct ines_header
{
    uint8_t nes_header[4];
    uint8_t prg_rom_16k_bank_count;
    uint8_t chr_rom_8k_bank_count;
    union
    {
        struct
        {
            uint8_t mirroring        : 1;
            bool has_prg_ram         : 1;
            bool has_trainer         : 1;
            bool ignore_mirroring    : 1;
            uint8_t mapper_low       : 4;
        };
        uint8_t flags_6;
    };
    union
    {
        struct 
        {
            bool is_vs_unisystem  : 1;
            bool is_playchoice_10 : 1;
            uint8_t is_nes_2_fmt  : 2;
            uint8_t mapper_high   : 4;
        };
        uint8_t flags_7;
    };
    uint8_t prg_ram_8k_block_count;
    union
    {
        struct
        {
            uint8_t tv_sys : 1;
        };
        uint8_t flags_9;
    };
    union
    {
        uint8_t flags_10;
    };
    uint8_t unused[5];
};
uint8_t mapper_index(struct ines_header ines);

struct mapper
{
    struct mirrored_mmio *prg_rom;
    struct mirrored_mmio *chr_rom;
};

struct mmio;
struct cartridge
{
    struct ines_header ines;

    struct mapper mapper;

    uint8_t *prg_rom;
    uint8_t *prg_ram;
    uint8_t *chr_rom;
};

struct cartridge cartridge_from_file(const char *path);
struct bus;
void attach_cartridge(struct bus *cpu_bus, struct bus *ppu_bus, struct cartridge *cartridge);

#endif
