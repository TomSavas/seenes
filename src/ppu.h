#ifndef PPU_H
#define PPU_H

#include <stdint.h>
#include <stdbool.h>

#include "raylib.h"
#include "seenes.h"

enum ppu_status_flag
{
    PPU_STATUS_O = 0x20,
    PPU_STATUS_S = 0x40,
    PPU_STATUS_V = 0x80
};

enum ctrl_increment_mode
{
    VERTICAL_INC_MODE = 0,
    HORIZONTAL_INC_MODE
};

#pragma pack(1)
union vram_addr_reg
{
    uint16_t reg;
    struct
    {
        uint8_t coarse_x           : 5;
        uint8_t coarse_y           : 5;
        uint8_t nametable_select_x : 1;
        uint8_t nametable_select_y : 1;
        uint8_t fine_y             : 3;
        uint8_t unused             : 1;
    };
};
SEENES_STATIC_ASSERT(sizeof(union vram_addr_reg) == 2, vram_adrr_reg_is_2_bytes);

union ppu_ctrl_reg
{
    uint8_t reg;
    struct 
    {
        uint8_t nametable_select                 : 2;
        uint8_t increment_mode                   : 1;
        uint8_t sprite_pattern_table_addr_select : 1;
        uint8_t bg_pattern_table_addr_select     : 1;
        uint8_t h                                : 1; // sprite height
        uint8_t p                                : 1; // ppu master/slave
        uint8_t nmi_enable                       : 1;
    };
};
SEENES_STATIC_ASSERT(sizeof(union ppu_ctrl_reg) == 1, ppu_ctrl_reg_is_1_byte);

union ppu_mask_reg
{
    uint8_t reg;
    struct
    {
        uint8_t greyscale         : 1;
        uint8_t bm                : 1;
        uint8_t sm                : 1;
        uint8_t render_background : 1;
        uint8_t render_sprites    : 1;
        uint8_t bgr               : 3;
    };
};
SEENES_STATIC_ASSERT(sizeof(union ppu_mask_reg) == 1, ppu_mask_reg_is_1_byte);

union ppu_status_reg
{
    uint8_t reg;
    struct
    {
        uint8_t unused          : 5;
        uint8_t sprite_overflow : 1;
        uint8_t sprite_0_hit    : 1;
        uint8_t vertical_blank  : 1;
    };
};
SEENES_STATIC_ASSERT(sizeof(union ppu_status_reg) == 1, ppu_status_reg_is_1_byte);
#pragma pack(0)

struct ppu_registers
{
    union ppu_ctrl_reg ctrl;
    union ppu_mask_reg mask;
    union ppu_status_reg status;

    uint8_t oma_dma;
    uint8_t oam_addr;
    uint8_t data_buffer;

    union vram_addr_reg vram_addr;
    union vram_addr_reg temp_vram_addr;

    uint8_t fine_x_scroll;
    bool is_first_write;

    uint16_t bg_shift_pattern_low_reg;
    uint16_t bg_shift_pattern_high_reg;
    // In reality these are 1B and are fed by separate 1B latches, but I employ the same
    // process as with pattern registers
    uint16_t bg_shift_attrib_low_reg;
    uint16_t bg_shift_attrib_high_reg;

    uint8_t bg_next_nametable_tile_index;
    uint8_t bg_next_tile_attrib;
    uint8_t bg_next_tile_pattern_low;
    uint8_t bg_next_tile_pattern_high;
};

union oam
{
    struct 
    {
        uint8_t y_coord;
        uint8_t tile_num;
        uint8_t attrib;
        uint8_t x_coord;
    } entry[64];
    uint8_t data[256];
};

struct bus;
struct ppu
{
    struct ppu_registers reg;
    struct bus *bus;
    union oam oam;

    uint16_t scanline;
    uint16_t cycle;

    uint64_t cycles;
};

enum frame_result
{
    FRAME_RUNNING = 0,
    FRAME_FINISHED
};

void ppu_reset(struct ppu *ppu);
enum frame_result ppu_clock(struct ppu *ppu);

struct mmio;
uint8_t ppu_reg_read(struct mmio *mmio, uint16_t addr);
void ppu_reg_write(struct mmio *mmio, uint16_t addr, uint8_t val);

struct cartridge;
uint16_t ppu_nametable_addr(struct cartridge *c, uint16_t addr);
uint8_t ppu_nametable_read(struct mmio *mmio, uint16_t addr);
void ppu_nametable_write(struct mmio *mmio, uint16_t addr, uint8_t val);

uint16_t ppu_palette_addr(uint16_t addr);
uint8_t ppu_palette_read(struct mmio *mmio, uint16_t addr);
void ppu_palette_write(struct mmio *mmio, uint16_t addr, uint8_t val);

Color color_from_index(uint8_t color_index);

#endif
