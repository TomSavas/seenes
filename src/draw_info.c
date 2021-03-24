struct font_cursor
{
    Font font;
    int initial_x;   
    int x;
    int y;
    int font_size;
    int spacing;

    int glyph_width;
    int glyph_height;
};

void draw_text(struct font_cursor *cursor, const char *str, Color color)
{
    DrawTextEx(cursor->font, str, (Vector2){cursor->x, cursor->y}, cursor->font_size, cursor->spacing, color);

    Vector2 size = MeasureTextEx(cursor->font, str, cursor->font_size, cursor->spacing);
    
    if (strstr(str, "\n") != NULL)
    {
        cursor->y += cursor->glyph_height;
        cursor->x = cursor->initial_x;
    }
    else
    {
        cursor->x += size.x;
    }
}

void draw_game(struct emu *emu) 
{
    Color c = color_from_index(bus_read(&emu->ppu_bus, 0x3F00));
    DrawRectangle(0, 0, 256*4, 240*4, c);
}

void draw_working_mem(struct emu *emu) 
{
    int x = 256*4;
    int y = 0;
    int w = 1400-x;
    int h = w;

    static struct decompiled_loc *src = NULL;
    if (src == NULL)
    {
        src = malloc((1 << 16) * sizeof(struct decompiled_loc));
        decompile(&emu->cpu_bus, src);
    }

    Vector2 glyph_size = MeasureTextEx(emu->font, "0", 16, 0);
    struct font_cursor cursor = {emu->font, x + 2, x + 2, y + 5, 16, 0, glyph_size.x, glyph_size.y};

    uint16_t addr = emu->cpu.reg.pc;
    int valid_ops = 0;
    uint16_t last_valid_op_addr = 0xffff;
    bool last_valid_op_set = false;
    while(valid_ops < 10)
    {
        if (strlen(src[addr].code) == 0)
        {
            addr--;
            continue;
        }

        if (!last_valid_op_set && addr <= emu->cpu.reg.pc)
        {
            last_valid_op_set = true;
            last_valid_op_addr = addr;
        }

        valid_ops++;
        addr--;
    }

    int added_lines = 0;
    while (added_lines < 23)
    {
        if (strlen(src[addr].code) == 0)
        {
            addr++;
            continue;
        }

        char txt[128];
        sprintf(txt, "%04x: ", addr);
        strcat(txt, src[addr].code);
        strcat(txt, "\n");
        
        Color c = GREEN;
        for (int i = 2000; i < 2008; i++)
        {
            char ppu_addr[5];
            sprintf(ppu_addr, "%d", i);
            
            if (strstr(src[addr].code, ppu_addr) != NULL)
            {
                c = YELLOW;
                break;
            }
        }

        draw_text(&cursor, txt, addr == emu->cpu.reg.pc ? RED : c);

        addr++;
        added_lines++;
    }
}

void draw_cpu_state(struct emu *emu) 
{
    int x = 256*4;
    int y = 1400-x + 10;
    int w = 1400-x;
    int h = w/4;

    Vector2 glyph_size = MeasureTextEx(emu->font, "0", 16, 0);
    struct font_cursor cursor = {emu->font, x+2, x+2, y+5, 16, 0, glyph_size.x, glyph_size.y};

    char cpu_info[512] = "\0";

    sprintf(cpu_info, "CPU Cycles: %lu\n", emu->cpu.cycles);
    draw_text(&cursor, cpu_info, GREEN);
    sprintf(cpu_info, "A: %02X (%4d)   SP: %02X\n", emu->cpu.reg.a, emu->cpu.reg.a, emu->cpu.reg.sp);
    draw_text(&cursor, cpu_info, GREEN);
    sprintf(cpu_info, "X: %02X (%4d)   PC: %04X\n", emu->cpu.reg.x, emu->cpu.reg.x, emu->cpu.reg.pc);
    draw_text(&cursor, cpu_info, GREEN);
    sprintf(cpu_info, "Y: %02X (%4d)   S : %02X\n", emu->cpu.reg.y, emu->cpu.reg.y, emu->cpu.reg.s.reg);
    draw_text(&cursor, cpu_info, GREEN);

    draw_text(&cursor, "N ", get_flag(&emu->cpu, CPU_N) ? GREEN : RED);
    draw_text(&cursor, "V ", get_flag(&emu->cpu, CPU_V) ? GREEN : RED);
    draw_text(&cursor, "- ", get_flag(&emu->cpu, CPU_U) ? GREEN : RED);
    draw_text(&cursor, "B ", get_flag(&emu->cpu, CPU_B) ? GREEN : RED);
    draw_text(&cursor, "D ", get_flag(&emu->cpu, CPU_D) ? GREEN : RED);
    draw_text(&cursor, "I ", get_flag(&emu->cpu, CPU_I) ? GREEN : RED);
    draw_text(&cursor, "Z ", get_flag(&emu->cpu, CPU_Z) ? GREEN : RED);
    draw_text(&cursor, "C ", get_flag(&emu->cpu, CPU_C) ? GREEN : RED);
}

void draw_ppu_state(struct emu *emu)
{
    int x = 1402;
    int y = 1400-256*4 + 10;
    int w = 1400-256*4;
    int h = w/4;

    Vector2 glyph_size = MeasureTextEx(emu->font, "0", 16, 0);
    struct font_cursor cursor = {emu->font, x+2, x+2, y+5, 16, 0, glyph_size.x, glyph_size.y};

    char ppu_info[512] = "\0";

    sprintf(ppu_info, "PPU Scanline: %*hu   PPU Cycle: %*hu\n", 3, emu->ppu.scanline, 3, emu->ppu.cycle);
    draw_text(&cursor, ppu_info, GREEN);
    sprintf(ppu_info, "vram: %04X  temp_vram: %04X\n", emu->ppu.reg.vram_addr.reg, emu->ppu.reg.temp_vram_addr.reg);
    draw_text(&cursor, ppu_info, GREEN);

    sprintf(ppu_info, "CTRL: ");
    draw_text(&cursor, ppu_info, GREEN);
    sprintf(ppu_info, "NMI ");
    draw_text(&cursor, ppu_info, emu->ppu.reg.ctrl.nmi_enable ? GREEN : RED);
    sprintf(ppu_info, "M_S ");
    draw_text(&cursor, ppu_info, emu->ppu.reg.ctrl.p ? GREEN : RED);
    sprintf(ppu_info, "SP_H ");
    draw_text(&cursor, ppu_info, emu->ppu.reg.ctrl.h ? GREEN : RED);
    sprintf(ppu_info, "BG_SEL ");
    draw_text(&cursor, ppu_info, emu->ppu.reg.ctrl.bg_pattern_table_addr_select ? GREEN : RED);
    sprintf(ppu_info, "SP_SEL ");
    draw_text(&cursor, ppu_info, emu->ppu.reg.ctrl.sprite_pattern_table_addr_select ? GREEN : RED);
    sprintf(ppu_info, "INC_M ");
    draw_text(&cursor, ppu_info, emu->ppu.reg.ctrl.increment_mode ? GREEN : RED);
    sprintf(ppu_info, "%d \n", emu->ppu.reg.ctrl.nametable_select);
    draw_text(&cursor, ppu_info, GREEN);

    sprintf(ppu_info, "MASK: ");
    draw_text(&cursor, ppu_info, GREEN);
    sprintf(ppu_info, "BGR: %01x ", emu->ppu.reg.mask.bgr);
    draw_text(&cursor, ppu_info, GREEN);
    sprintf(ppu_info, "R_SP ");
    draw_text(&cursor, ppu_info, emu->ppu.reg.mask.render_sprites ? GREEN : RED);
    sprintf(ppu_info, "R_BG ");
    draw_text(&cursor, ppu_info, emu->ppu.reg.mask.render_background ? GREEN : RED);
    sprintf(ppu_info, "SM ");
    draw_text(&cursor, ppu_info, emu->ppu.reg.mask.sm ? GREEN : RED);
    sprintf(ppu_info, "BM ");
    draw_text(&cursor, ppu_info, emu->ppu.reg.mask.bm ? GREEN : RED);
    sprintf(ppu_info, "G \n");
    draw_text(&cursor, ppu_info, emu->ppu.reg.mask.greyscale ? GREEN : RED);

    sprintf(ppu_info, "STATUS: ");
    draw_text(&cursor, ppu_info, GREEN);
    sprintf(ppu_info, "VB ");
    draw_text(&cursor, ppu_info, emu->ppu.reg.status.vertical_blank ? GREEN : RED);
    sprintf(ppu_info, "SP_0_H ");
    draw_text(&cursor, ppu_info, emu->ppu.reg.status.sprite_0_hit ? GREEN : RED);
    sprintf(ppu_info, "SP_V ");
    draw_text(&cursor, ppu_info, emu->ppu.reg.status.sprite_overflow ? GREEN : RED);
}

void draw_ram(struct emu *emu) 
{
    int x = 256*4;
    int w = 1400-256*4;
    int y = 1400-(256*4) + 10 + w/4 + 10;
    int h = w;

    Vector2 glyph_size = MeasureTextEx(emu->font, "0", 14, 0);
    struct font_cursor cursor = {emu->font, x+2, x+2, y+5, 16, 0, glyph_size.x, glyph_size.y};

    uint16_t top_lines[] = {0x0000, 0x0730};

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 15 + i; j++)
        {
            char txt[128] = "\0";
            uint16_t line_addr = top_lines[i] + j*8;

            char buf[16];
            sprintf(buf, "%04x: ", line_addr);
            strcat(txt, buf);

            draw_text(&cursor, txt, GREEN);

            for (int k = 0; k < 8; k++)
            {
                char buf[16];
                sprintf(buf, "%02x ", bus_read(&emu->cpu_bus, line_addr + k));
                if (0x0770 <= line_addr+k && line_addr+k <= 0x0771)
                    draw_text(&cursor, buf, RED);
                else
                    draw_text(&cursor, buf, GREEN);
            }

            buf[0] = '\n';
            buf[1] = '\0';
            draw_text(&cursor, buf, GREEN);
        }

        if (i == 0)
        {
            char buf[] = "-----------------------------\n";
            draw_text(&cursor, buf, GREEN);
        }
    }
}

void draw_stack(struct emu *emu) 
{
    int x = 1402;
    int w = 1400-256*4;
    int y = 1400-(256*4) + 10 + w/4 + 10;
    int h = w;

    Vector2 glyph_size = MeasureTextEx(emu->font, "0", 14, 0);
    struct font_cursor cursor = {emu->font, x+2, x+2, y+5, 16, 0, glyph_size.x, glyph_size.y};

    uint16_t top_line = 0x0100;
    for(int i = 0; i < 32; i++)
    {
        char txt[128] = "\0";
        uint16_t line_addr = top_line + i*8;

        char buf[16];
        sprintf(buf, "%04x: ", line_addr);
        strcat(txt, buf);

        draw_text(&cursor, txt, GREEN);

        for (int j = 0; j < 8; j++)
        {
            char buf[16];
            sprintf(buf, "%02x ", bus_read(&emu->cpu_bus, line_addr + j));
            draw_text(&cursor, buf, (0x0100 | (uint16_t)emu->cpu.reg.sp) == (line_addr+j) ? RED : GREEN);
        }

        buf[0] = '\n';
        buf[1] = '\0';
        draw_text(&cursor, buf, GREEN);
    }
}

void draw_pattern_table(struct emu *emu)
{
    int X = 1400 + 2;
    int Y = 0;
    int w = 256;
    int h = w;

    Color lookup[] = { BLACK, RED, GREEN, BLUE };
    for (uint16_t lr = 0; lr < 2; lr++)
    {
        for (uint16_t y = 0; y < 16; y++)
        {
            for (uint16_t x = 0; x < 16; x++)
            {
                uint16_t lower_plane_addr = (lr << 12) | (y << 8) | (x << 4) | 0x0000;
                uint16_t upper_plane_addr = (lr << 12) | (y << 8) | (x << 4) | 0x0008;

                for (uint16_t row = 0; row < 8; row++)
                {
                    uint8_t lower = bus_read(&emu->ppu_bus, lower_plane_addr | row);
                    uint8_t upper = bus_read(&emu->ppu_bus, upper_plane_addr | row);

                    for (uint16_t col = 0; col < 8; col++)
                    {
                        bool lower_set = lower & (1 << col);
                        bool upper_set = upper & (1 << col);
                        uint8_t index = (lower_set ? 1 : 0) | ((upper_set ? 1 : 0) << 1);

                        DrawPixel(X + x*16 + (7-col)*2     + (lr == 0 ? 0 : 256 + 2), Y + y*16 + row*2    , lookup[index]);
                        DrawPixel(X + x*16 + (7-col)*2 + 1 + (lr == 0 ? 0 : 256 + 2), Y + y*16 + row*2    , lookup[index]);
                        DrawPixel(X + x*16 + (7-col)*2     + (lr == 0 ? 0 : 256 + 2), Y + y*16 + row*2 + 1, lookup[index]);
                        DrawPixel(X + x*16 + (7-col)*2 + 1 + (lr == 0 ? 0 : 256 + 2), Y + y*16 + row*2 + 1, lookup[index]);
                    }
                }
            }
        }
    }
}

void draw_palettes(struct emu *emu)
{
    int x = 1400 + 2;
    int y = 256 + 2;
    int w = 256;

    Vector2 glyph_size = MeasureTextEx(emu->font, "0", 16, 0);
    struct font_cursor cursor = {emu->font, x+2, x+2, y+5, 16, 0, glyph_size.x, glyph_size.y};

    draw_text(&cursor, "Universal bg color ", GREEN);
    Color c = color_from_index(bus_read(&emu->ppu_bus, 0x3F00));
    DrawRectangle(cursor.x, cursor.y, cursor.glyph_width, cursor.glyph_width, c);

    draw_text(&cursor, "\n", GREEN);

    draw_text(&cursor, "Bg palettes:\n", GREEN);
    for (int i = 0; i < 4; i++)
    {
        char txt[5];
        sprintf(txt, "#%d: ", i);
        draw_text(&cursor, txt, GREEN);

        for (int j = 0; j < 4; j++)
        {
            uint16_t addr = 0x3F00 | (i << 2) | j;
            uint8_t color_index = bus_read(&emu->ppu_bus, addr);
            Color c = color_from_index(color_index);
            DrawRectangle(cursor.x + (cursor.glyph_width + 4) * j, cursor.y, cursor.glyph_width, cursor.glyph_width, c);
            //char buf[16];
            //sprintf(buf, "%02x ", color_index);
            //draw_text(&cursor, buf, GREEN);
        }
        cursor.y += cursor.glyph_height;
        cursor.x = cursor.initial_x;
    }
    
    cursor.initial_x = x+2 + 256;
    cursor.x = cursor.initial_x;
    cursor.y -= 5 * cursor.glyph_height;
    draw_text(&cursor, "Fg palettes:\n", GREEN);
    for (int i = 4; i < 8; i++)
    {
        char txt[5];
        sprintf(txt, "#%d: ", i);
        draw_text(&cursor, txt, GREEN);

        for (int j = 0; j < 4; j++)
        {
            uint16_t addr = 0x3F00 | (1 << 4) | (i << 2) | j;
            uint8_t color_index = bus_read(&emu->ppu_bus, addr);
            Color c = color_from_index(color_index);
            DrawRectangle(cursor.x + (cursor.glyph_width + 4) * j, cursor.y, cursor.glyph_width, cursor.glyph_width, c);
            //char buf[16];
            //sprintf(buf, "%02x ", color_index);
            //draw_text(&cursor, buf, GREEN);
        }
        cursor.y += cursor.glyph_height;
        cursor.x = cursor.initial_x;
    }
}
