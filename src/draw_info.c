struct font_cursor
{
    Font font;
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
        cursor->y += cursor->glyph_height;
    else
        cursor->x += size.x;
}

void draw_game(struct emu *emu) 
{
    DrawRectangle(0, 0, 256*4, 240*4, BLACK);
    /*
    Image game_img = GenImageColor(256 * 4, 240 * 4, RED);
    Texture2D game_tex = LoadTextureFromImage(game_img);
    
    DrawTexture(game_tex, 0, 0, BLACK);
    */
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
        decompile(&emu->cpu_bus, 0x0000, 0xFFFF, src);
    }

    Vector2 glyph_size = MeasureTextEx(emu->font, "0", 16, 0);
    struct font_cursor cursor = {emu->font, x + 2, y + 5, 16, 0, glyph_size.x, glyph_size.y};

    uint16_t addr = emu->cpu.reg.pc;
    int valid_ops = 0;
    uint16_t last_valid_op_addr = 0xffff;
    bool last_valid_op_set = false;
    while(valid_ops < 10)
    {
        if (!src[addr].valid_loc)
        {
            addr--;
            continue;
        }

        if (!last_valid_op_set && addr < emu->cpu.reg.pc)
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
        if (!src[addr].valid_loc)
        {
            addr++;
            continue;
        }

        char txt[128];
        sprintf(txt, "%04x: ", addr);
        strcat(txt, src[addr].code);
        strcat(txt, "\n");

        draw_text(&cursor, txt, addr == (uint16_t)last_valid_op_addr ? RED : GREEN);

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
    struct font_cursor cursor = {emu->font, x+2, y+5, 16, 0, glyph_size.x, glyph_size.y};

    char cpu_info[512] = "\0";

    sprintf(cpu_info, "Cycles: %lu\n", emu->cpu.cycles);
    draw_text(&cursor, cpu_info, GREEN);
    sprintf(cpu_info, "A: %02X (%4d)   SP: %02X\n", emu->cpu.reg.a, emu->cpu.reg.a, emu->cpu.reg.sp);
    draw_text(&cursor, cpu_info, GREEN);
    sprintf(cpu_info, "X: %02X (%4d)   PC: %04X\n", emu->cpu.reg.x, emu->cpu.reg.x, emu->cpu.reg.pc);
    draw_text(&cursor, cpu_info, GREEN);
    sprintf(cpu_info, "Y: %02X (%4d)   S : %02X\n", emu->cpu.reg.y, emu->cpu.reg.y, emu->cpu.reg.s);
    draw_text(&cursor, cpu_info, GREEN);

    draw_text(&cursor, "N ", get_flag(&emu->cpu, N) ? GREEN : RED);
    draw_text(&cursor, "V ", get_flag(&emu->cpu, V) ? GREEN : RED);
    draw_text(&cursor, "- ", get_flag(&emu->cpu, U) ? GREEN : RED);
    draw_text(&cursor, "B ", get_flag(&emu->cpu, B) ? GREEN : RED);
    draw_text(&cursor, "D ", get_flag(&emu->cpu, D) ? GREEN : RED);
    draw_text(&cursor, "I ", get_flag(&emu->cpu, I) ? GREEN : RED);
    draw_text(&cursor, "Z ", get_flag(&emu->cpu, Z) ? GREEN : RED);
    draw_text(&cursor, "C ", get_flag(&emu->cpu, C) ? GREEN : RED);
}

void draw_ram(struct emu *emu) 
{
    int x = 256*4;
    int w = 1400-x;
    int y = 1400-x + 10 + w/4 + 10;
    int h = w;

    Vector2 glyph_size = MeasureTextEx(emu->font, "0", 16, 0);
    struct font_cursor cursor = {emu->font, x+2, y+5, 16, 0, glyph_size.x, glyph_size.y};

    static uint16_t top_line = 0x0000;
    for(int i = 0; i < 16; i++)
    {
        char txt[128] = "\0";
        uint16_t line_addr = top_line + i*8;

        char buf[16];
        sprintf(buf, "%04x: ", line_addr);
        strcat(txt, buf);

        for (int j = 0; j < 8; j++)
        {
            char buf[16];
            sprintf(buf, "%02x ", emu->mem[line_addr + j]);
            strcat(txt, buf);
        }
        strcat(txt, "\n");

        draw_text(&cursor, txt, GREEN);
    }
}
