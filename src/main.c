#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>

#include "raylib.h"

#include "cpu_6502.h"
#include "ppu.h"
#include "bus.h"
#include "mmio.h"
#include "cartridge.h"

#include "decompiler.c"

struct emu
{
    struct cpu_6502 cpu;
    struct ppu ppu;

    uint8_t onboard_ram[0x0800];
    struct mirrored_mmio ram;

    uint8_t onboard_vram[0x1FFF];
    struct mmio pattern_table;
    struct mmio nametable;

    uint8_t onboard_palette_vram[0x0020];
    struct mmio palette_vram;

    struct cartridge cartridge;

    struct bus cpu_bus;
    struct bus ppu_bus;

    bool paused;

    Font font;
    uint64_t cycles;
};

#include "draw_info.c"

void emu_reset(struct emu *emu)
{
    emu->cpu = (struct cpu_6502){ 0 };
    emu->cpu.bus = &emu->cpu_bus;
    cpu_reset(&emu->cpu);
    memset(emu->onboard_ram, 0, 0x0800);

    emu->ppu = (struct ppu){ 0 };
    emu->ppu.bus = &emu->ppu_bus;
    ppu_reset(&emu->ppu);
    memset(emu->onboard_vram, 0, 0x1FFF);
}

struct emu init_emu(const char *cartridge_path)
{
    struct emu emu =
    {
        .cpu_bus = init_bus(10),
        .ppu_bus = init_bus(10),
        .cartridge = cartridge_from_file(cartridge_path),
        .paused = true,
        .font = LoadFont("../font.ttf")
    };

    // cpu mappings
    {
        emu.ram = make_mirrored_mmio(make_mmio(0x0000, 0x1FFF, emu.onboard_ram), 0x0000, 0x07FF);
        emu.ram.mmio.name = "ram";
        attach(&emu.cpu_bus, (struct mmio*)&emu.ram);

        struct mmio *mm_ppu_reg = malloc(sizeof(struct mmio));
        *mm_ppu_reg = make_mmio(0x2000, 0x3FFF, (uint8_t*) NULL);
        mm_ppu_reg->name = "mm_ppu_reg";
        mm_ppu_reg->mmio_read = ppu_reg_read;
        mm_ppu_reg->mmio_write = ppu_reg_write;
        mm_ppu_reg->additional_data = &emu.ppu;
        attach(&emu.cpu_bus, mm_ppu_reg);

        // add apu/io mappings  
        // add usually disabled apu/io mappings, just in case
        uint8_t *unused_data = malloc(0x1F);
        struct mmio *unused = malloc(sizeof(struct mmio));
        // memleaks which I don't give a shit about atm
        *unused = make_mmio(0x4000, 0x401F, unused_data);
        unused->name = "unused apu/io";
        attach(&emu.cpu_bus, unused);
    }

    attach_cartridge(&emu.cpu_bus, &emu.ppu_bus, &emu.cartridge);

    // ppu mappings
    {
        emu.nametable = make_mmio(0x2000, 0x3EFF, emu.onboard_vram);
        emu.nametable.name = "nametable";
        emu.nametable.mmio_read = ppu_nametable_read;
        emu.nametable.mmio_write = ppu_nametable_write;
        emu.nametable.additional_data = &emu.cartridge;
        attach(&emu.ppu_bus, (struct mmio*)&emu.nametable);

        emu.palette_vram = make_mmio(0x3F00, 0x3FFF, emu.onboard_palette_vram);
        emu.palette_vram.name = "palette_vram";
        emu.palette_vram.mmio_read = ppu_palette_read;
        emu.palette_vram.mmio_write = ppu_palette_write;
        emu.palette_vram.additional_data = &emu.ppu;
        attach(&emu.ppu_bus, &emu.palette_vram);
    }

    emu_reset(&emu);

    return emu;
}

enum frame_result clock(struct emu *emu)
{
    enum frame_result result = ppu_clock(&emu->ppu);
    if (result == FRAME_FINISHED)
    {
        if (emu->ppu.reg.ctrl.nmi_enable)
        {
            nmi(&emu->cpu);
        }
    }

    if (emu->cycles++ % 3 == 0)
        cpu_clock(&emu->cpu);

    return result;
}

int main(int argc, const char **argv)
{
    InitWindow(1400 + 256 * 2 + 8, 960, "seenes");   
    SetTargetFPS(-1);
    //SetTargetFPS(60);

    struct emu emu = init_emu(argv[1]);

    while(!WindowShouldClose())
    {
        BeginDrawing();
        ClearBackground((Color){ 0, 0, 164, 255 });

        draw_working_mem(&emu);
        draw_cpu_state(&emu);
        draw_ppu_state(&emu);
        draw_ram(&emu);
        draw_stack(&emu);

        draw_pattern_table(&emu);
        draw_palettes(&emu);

        draw_game(&emu);

        if (!emu.paused)
            while (clock(&emu) == FRAME_RUNNING) {}

        DrawFPS(0, 0);
        EndDrawing();

        if (IsKeyPressed(KEY_R))
        {
            emu_reset(&emu);
        }

        if (IsKeyPressed(KEY_P))
        {
            emu.paused = !emu.paused;
        }

        if (emu.paused)
        {
            static int cooldown;

            if ((!IsKeyDown(KEY_O) && !IsKeyDown(KEY_I) && !IsKeyDown(KEY_U) && !IsKeyDown(KEY_Y) && !IsKeyDown(KEY_T)) || cooldown-- > 0)
                continue;

            clock(&emu);
            if (IsKeyDown(KEY_O))
                cooldown = 5;
            else if (IsKeyDown(KEY_I))
                cooldown = 1;
            else if (IsKeyDown(KEY_U))
            {
                for (int i = 0; i < 99; i++)
                    clock(&emu);
                cooldown = 5;
            }
            else if (IsKeyDown(KEY_Y))
            {
                for (int i = 0; i < 999; i++)
                    clock(&emu);
                cooldown = 5;
            }
            else if (IsKeyDown(KEY_T))
            {
                for (int i = 0; i < 9999; i++)
                    clock(&emu);
                cooldown = 5;
            }
        }
    }

    CloseWindow();

    return 0;
}
