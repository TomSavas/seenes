#include <assert.h>
#include <signal.h>

#include "raylib.h"

#include "ppu.h"
#include "mmio.h"
#include "bus.h"
#include "cartridge.h"

void ppu_reset(struct ppu *ppu)
{
    ppu->reg = (struct ppu_registers) { 0 };
    ppu->reg.status.reg = PPU_STATUS_V | PPU_STATUS_O;
    ppu->scanline = 0;
    ppu->cycle = 0;
    ppu->cycles = 0;
}

enum frame_result ppu_clock(struct ppu *ppu)
{
    const uint16_t prerender_scanline = 261; 
    const uint16_t postrender_scanline = 240; 

    enum frame_result frame_result = FRAME_RUNNING;

    if (ppu->scanline < postrender_scanline || ppu->scanline == prerender_scanline)
    {
        if (ppu->scanline == 0 && ppu->cycle == 0)
        {
        }

        if (ppu->scanline == prerender_scanline && ppu->cycle == 1)
        {
            ppu->reg.status.reg = 0;
        }

        if ((2 <= ppu->cycle && ppu->cycle <= 256) || (321 <= ppu->cycle && ppu->cycle <= 338))
        {
            if (ppu->reg.mask.render_background)
            {
                ppu->reg.bg_shift_pattern_low_reg <<= 1;
                ppu->reg.bg_shift_pattern_high_reg <<= 1;

                ppu->reg.bg_shift_attrib_low_reg <<= 1;
                ppu->reg.bg_shift_attrib_high_reg <<= 1;
            }

            switch ((ppu->cycle-1) % 8)
            {
            case 0:
                // nt byte
                ppu->reg.bg_shift_pattern_low_reg = (ppu->reg.bg_shift_pattern_low_reg & 0xFF00) | ppu->reg.bg_next_tile_pattern_low;
                ppu->reg.bg_shift_pattern_high_reg = (ppu->reg.bg_shift_pattern_high_reg & 0xFF00) | ppu->reg.bg_next_tile_pattern_high;
                
                ppu->reg.bg_shift_attrib_low_reg = (ppu->reg.bg_shift_attrib_low_reg & 0xFF00) | ((ppu->reg.bg_next_tile_attrib & 0x1) ? 0xFF : 0x00);
                ppu->reg.bg_shift_attrib_high_reg = (ppu->reg.bg_shift_attrib_high_reg & 0xFF00) | ((ppu->reg.bg_next_tile_attrib & 0x2) ? 0xFF : 0x00);

                ppu->reg.bg_next_nametable_tile_index = bus_read(ppu->bus, 0x2000 | (ppu->reg.vram_addr.reg & 0x0FFF));
                break;
            case 2:
                // at byte
                ppu->reg.bg_next_tile_attrib = bus_read(ppu->bus, 
                        0x23C0 | ((uint16_t)ppu->reg.vram_addr.nametable_select_y << 11) | 
                        (uint16_t)ppu->reg.vram_addr.nametable_select_x << 10 | 
                        (((uint16_t)ppu->reg.vram_addr.coarse_y >> 2) << 3) |
                        ((uint16_t)ppu->reg.vram_addr.coarse_x >> 2));

                if (ppu->reg.vram_addr.coarse_y & 0x02)
                    ppu->reg.bg_next_tile_attrib >>= 4;
                if (ppu->reg.vram_addr.coarse_x & 0x02)
                    ppu->reg.bg_next_tile_attrib >>= 2;
                ppu->reg.bg_next_tile_attrib &= 0x03;

                break;
            case 4:
                // low bg byte
                ppu->reg.bg_next_tile_pattern_low = bus_read(ppu->bus, 
                        ((uint16_t)ppu->reg.ctrl.bg_pattern_table_addr_select << 12) |
                        ((uint16_t)ppu->reg.bg_next_nametable_tile_index << 4) |
                        (uint16_t)ppu->reg.vram_addr.fine_y);
                break;
            case 6:
                // high bg byte
                ppu->reg.bg_next_tile_pattern_high = bus_read(ppu->bus, 
                        ((uint16_t)ppu->reg.ctrl.bg_pattern_table_addr_select << 12) |
                        ((uint16_t)ppu->reg.bg_next_nametable_tile_index << 4) |
                        (uint16_t)ppu->reg.vram_addr.fine_y |
                        0x0008);
                break;
            case 7:
                // Move one tile to the left, switch to next nametable horizontally once
                // we're finished with 32 tiles in the current nametable
                if (ppu->reg.mask.render_background || ppu->reg.mask.render_sprites)
                {
                    if (ppu->reg.vram_addr.coarse_x == 31)
                    {
                        ppu->reg.vram_addr.coarse_x = 0;
                        ppu->reg.vram_addr.nametable_select_x = ~ppu->reg.vram_addr.nametable_select_x;
                    }
                    else
                    {
                        ppu->reg.vram_addr.coarse_x++;
                    }
                }
                break;
            }

            if (ppu->cycle == 256)
            {
                if (ppu->reg.mask.render_background || ppu->reg.mask.render_sprites)
                {
                    if (ppu->reg.vram_addr.fine_y < 7)
                    {
                        ppu->reg.vram_addr.fine_y++;
                    }
                    else
                    {
                        ppu->reg.vram_addr.fine_y = 0;
                        if (ppu->reg.vram_addr.coarse_y == 29)
                        {
                            ppu->reg.vram_addr.coarse_y = 0;
                            ppu->reg.vram_addr.nametable_select_y = ~ppu->reg.vram_addr.nametable_select_y;
                        }
                        else if (ppu->reg.vram_addr.coarse_y == 31)
                        {
                            ppu->reg.vram_addr.coarse_y = 0;
                        }
                        else
                        {
                            ppu->reg.vram_addr.coarse_y++;
                        }
                    }
                }
            }

        }

        if (ppu->cycle == 257)
        {
            if (ppu->reg.mask.render_background || ppu->reg.mask.render_sprites)
            {
                ppu->reg.vram_addr.coarse_x = ppu->reg.temp_vram_addr.coarse_x;
                ppu->reg.vram_addr.nametable_select_x = ppu->reg.temp_vram_addr.nametable_select_x;
            }
        }

        if (ppu->cycle == 339)
        {
            // unused NT read
            ppu->reg.bg_shift_pattern_low_reg = (ppu->reg.bg_shift_pattern_low_reg & 0xFF00) | ppu->reg.bg_next_tile_pattern_low;
            ppu->reg.bg_shift_pattern_high_reg = (ppu->reg.bg_shift_pattern_high_reg & 0xFF00) | ppu->reg.bg_next_tile_pattern_high;
            
            ppu->reg.bg_shift_attrib_low_reg = (ppu->reg.bg_shift_attrib_low_reg & 0xFF00) | ((ppu->reg.bg_next_tile_attrib & 0x1) ? 0xFF : 0x00);
            ppu->reg.bg_shift_attrib_high_reg = (ppu->reg.bg_shift_attrib_high_reg & 0xFF00) | ((ppu->reg.bg_next_tile_attrib & 0x2) ? 0xFF : 0x00);

            ppu->reg.bg_next_nametable_tile_index = bus_read(ppu->bus, 0x2000 | (ppu->reg.vram_addr.reg & 0x0FFF));
        }

        if (ppu->scanline == prerender_scanline && 280 <= ppu->cycle && ppu->cycle <= 304)
        {
            if (ppu->reg.mask.render_background || ppu->reg.mask.render_sprites)
            {
                ppu->reg.vram_addr.coarse_y = ppu->reg.temp_vram_addr.coarse_y;
                ppu->reg.vram_addr.nametable_select_y = ppu->reg.temp_vram_addr.nametable_select_y;
                ppu->reg.vram_addr.fine_y = ppu->reg.temp_vram_addr.fine_y;
            }
        }

        if (ppu->cycle < 256 && ppu->scanline < postrender_scanline && ppu->reg.mask.render_background)
        {
            // TODO: cleanup
            uint16_t bit_mux = 0x8000 >> ppu->reg.fine_x_scroll;

            uint8_t p0_pixel = (ppu->reg.bg_shift_pattern_low_reg & bit_mux) > 0;
            uint8_t p1_pixel = (ppu->reg.bg_shift_pattern_high_reg & bit_mux) > 0;
            uint8_t pixel = (p1_pixel << 1) | p0_pixel;

            uint8_t bg_p0_palette = (ppu->reg.bg_shift_attrib_low_reg & bit_mux) > 0;
            uint8_t bg_p1_palette = (ppu->reg.bg_shift_attrib_high_reg & bit_mux) > 0;
            uint8_t bg_palette = (bg_p1_palette << 1) | bg_p0_palette;
            
            uint8_t color_index = bus_read(ppu->bus, 0x3F00 | (bg_palette << 2) + pixel);
            Color c = color_from_index(color_index & 0x3F);

            const int upscale_factor = 4;
            DrawRectangle((ppu->cycle)*upscale_factor, ppu->scanline*upscale_factor, upscale_factor, upscale_factor, c);
        }
    }

    // POSTRENDER
    if (postrender_scanline <= ppu->scanline && ppu->scanline < prerender_scanline)
    {
        if (ppu->scanline == 241 && ppu->cycle == 1)
        {
            ppu->reg.status.vertical_blank = 1;
            frame_result = FRAME_FINISHED;
        }
    }

    ppu->cycles++;
    ppu->cycle++;
    if (ppu->cycle >= 341)
    {
        ppu->cycle = 0;
        ppu->scanline++;

        if (ppu->scanline > 261)
            ppu->scanline = 0;
    }

    return frame_result;
}

uint8_t ppu_reg_read(struct mmio *mmio, uint16_t addr)
{
    // Mask off any unneeded bits
    addr &= 0x0007;   
    struct ppu *ppu = mmio->additional_data;

    // ppu changes state upon reads from some registers
    uint8_t data = 0x00; //= ppu->reg.cpu_exposed_ppu_registers[addr];
    switch(addr)
    {
    case 0:
        assert(false);
        break;
    case 1:
        assert(false);
        break;
    case 2:
        // Turns out cpu is using the unused bits as "random data" (-_-)
        data = ppu->reg.status.reg & 0xE0 | ppu->reg.data_buffer & 0x1F;

        ppu->reg.status.vertical_blank = 0;
        ppu->reg.is_first_write = true;
        break;
    case 3:
        break;
    case 4:
        break;
    case 5:
        break;
    case 6:
        break;
    case 7:
        data = ppu->reg.data_buffer;

        ppu->reg.data_buffer = bus_read(ppu->bus, ppu->reg.vram_addr.reg);

        // There's no delay if we're reading from palette ram
        if (0x3F00 <= ppu->reg.vram_addr.reg && ppu->reg.vram_addr.reg <= 0x3FFF)
            data = ppu->reg.data_buffer;

        ppu->reg.vram_addr.reg += (ppu->reg.ctrl.increment_mode == VERTICAL_INC_MODE ? 1 : 32);
        break;
    default:
        assert(false);
    }

    return data;
}

void ppu_reg_write(struct mmio *mmio, uint16_t addr, uint8_t val)
{
    // Mask off any unneeded bits
    addr &= 0x0007;   
    struct ppu *ppu = mmio->additional_data;

    switch(addr)
    {
    case 0:
            ppu->reg.ctrl.reg = val;
            ppu->reg.temp_vram_addr.nametable_select_x = ppu->reg.ctrl.nametable_select & 0x01;
            ppu->reg.temp_vram_addr.nametable_select_y = (ppu->reg.ctrl.nametable_select & 0x02) >> 1;
        break;
    case 1:
            ppu->reg.mask.reg = val;
        break;
    case 2:
            assert(false);
        break;
    case 3:
        break;
    case 4:
        break;
    case 5:
        if (ppu->reg.is_first_write)
        {
            ppu->reg.fine_x_scroll = val & 0x07;
            ppu->reg.temp_vram_addr.coarse_x = val >> 3;
            ppu->reg.is_first_write = false;
        }
        else
        {
            ppu->reg.temp_vram_addr.fine_y = val & 0x07;
            ppu->reg.temp_vram_addr.coarse_y = val >> 3;
            ppu->reg.is_first_write = true;
        }
        break;
    case 6:
        if (ppu->reg.is_first_write)
        {
            // msb (15th bit, the register is only 15b) is always cleared on first write
            // ah, let's clear the unused bit as well, who gives a shit
            ppu->reg.temp_vram_addr.reg &= 0x00FF;
            ppu->reg.temp_vram_addr.reg |= ((uint16_t)val & 0x3F) << 8;

            ppu->reg.is_first_write = false;
        }
        else
        {
            ppu->reg.temp_vram_addr.reg &= 0xFF00;
            ppu->reg.temp_vram_addr.reg |= val;

            ppu->reg.vram_addr = ppu->reg.temp_vram_addr;
            ppu->reg.is_first_write = true;
        }
        break;
    case 7:
        bus_write(ppu->bus, ppu->reg.vram_addr.reg & 0x3FFF, val);
        ppu->reg.vram_addr.reg += (ppu->reg.ctrl.increment_mode == VERTICAL_INC_MODE ? 1 : 32);
        break;
    default:
        assert(false);
    }
}

uint16_t ppu_nametable_addr(struct cartridge *c, uint16_t addr)
{
    if (addr > 0x2FFF)
        addr -= 0x1000;
    addr &= 0x2FFF;

    if (c->ines.mirroring == VERTICAL)
    {
        if (0x2800 <= addr)
            addr -= 0x0800;
    }
    else
    {
        if ((0x2400 <= addr && addr < 0x2800) ||
            (0x2C00 <= addr && addr <= 0x2FFF))
            addr -= 0x0400;
    }

    return addr - 0x2000;
}

uint8_t ppu_nametable_read(struct mmio *mmio, uint16_t addr)
{
    struct cartridge *c = mmio->additional_data;
    return mmio->data[ppu_nametable_addr(c, addr)];
}

void ppu_nametable_write(struct mmio *mmio, uint16_t addr, uint8_t val)
{
    struct cartridge *c = mmio->additional_data;
    mmio->data[ppu_nametable_addr(c, addr)] = val;
}

uint16_t ppu_palette_addr(uint16_t addr)
{
    // We only care about the lower 5 bits
    addr &= 0x001F;

    switch(addr)
    {
    case 0x0010:
    case 0x0014:
    case 0x0018:
    case 0x001C:
        addr &= 0x000F;
        break;
    }

    return addr;
}

uint8_t ppu_palette_read(struct mmio *mmio, uint16_t addr)
{
    struct ppu *ppu = mmio->additional_data;
    return mmio->data[ppu_palette_addr(addr)] & (ppu->reg.mask.greyscale ? 0x30 : 0x3F);
}

void ppu_palette_write(struct mmio *mmio, uint16_t addr, uint8_t val)
{
    mmio->data[ppu_palette_addr(addr)] = val;
}

Color color_from_index(uint8_t color_index)
{
    static Color color_index_to_color[] =
    {
        {  84,  84,  84, 255},        
        {   0,  30, 116, 255},        
        {   8,  16, 144, 255},        
        {  48,   0, 136, 255},        
        {  68,   0, 100, 255},        
        {  92,   0,  48, 255},        
        {  84,   4,   0, 255},        
        {  60,  24,   0, 255},        
        {  32,  42,   0, 255},        
        {   8,  58,   0, 255},        
        {   0,  64,   0, 255},        
        {   0,  60,   0, 255},        
        {   0,  50,  60, 255},        
        {   0,   0,   0, 255},        
        {   0,   0,   0, 255},        
        {   0,   0,   0, 255},        

        { 152, 150, 152, 255},
        {   8,  76, 196, 255},
        {  48,  50, 236, 255},
        {  92,  30, 228, 255},
        { 136,  20, 176, 255},
        { 160,  20, 100, 255},
        { 152,  34,  32, 255},
        { 120,  60,   0, 255},
        {  84,  90,   0, 255},
        {  40, 114,   0, 255},
        {   8, 124,   0, 255},
        {   0, 118,  40, 255},
        {   0, 102, 120, 255},
        {   0,   0,   0, 255},
        {   0,   0,   0, 255},
        {   0,   0,   0, 255},

        { 236, 238, 236, 255},
        {  76, 154, 236, 255},
        { 120, 124, 236, 255},
        { 176,  98, 236, 255},
        { 228,  84, 236, 255},
        { 236,  88, 180, 255},
        { 236, 106, 100, 255},
        { 212, 136,  32, 255},
        { 160, 170,   0, 255},
        { 116, 196,   0, 255},
        {  76, 208,  32, 255},
        {  56, 204, 108, 255},
        {  56, 180, 204, 255},
        {  60,  60,  60, 255},
        {   0,   0,   0, 255},
        {   0,   0,   0, 255},

        { 236, 238, 236, 255},
        { 168, 204, 236, 255},
        { 188, 188, 236, 255},
        { 212, 178, 236, 255},
        { 236, 174, 236, 255},
        { 236, 174, 212, 255},
        { 236, 180, 176, 255},
        { 228, 196, 144, 255},
        { 204, 210, 120, 255},
        { 180, 222, 120, 255},
        { 168, 226, 144, 255},
        { 152, 226, 180, 255},
        { 160, 214, 228, 255},
        { 160, 162, 160, 255},
        {   0,   0,   0, 255},
        {   0,   0,   0, 255}
    };

    return color_index_to_color[color_index];
}
