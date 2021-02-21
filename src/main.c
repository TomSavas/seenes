#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "raylib.h"

#include "cpu_6502.h"
#include "bus.h"
#include "mmio.h"

#include "decompiler.c"

struct emu
{
    struct cpu_6502 cpu;

    uint8_t *mem;

    // mmio devices, indexes memory in mem
    struct mirrored_mmio ram;
    struct mirrored_mmio ppu_registers;
    struct mmio apu_registers;
    struct mmio cartridge;

    struct bus cpu_bus;
    struct bus ppu_bus;

    // Controls
    bool paused;

    // Visualization
    Font font;
};

#include "draw_info.c"

void write_test(uint8_t *mem)
{
    // reset vector
    mem[0xFFFD] = 0x80;
    mem[0xFFFC] = 0x00;

    // program (calculate 3*10 in "a" register)
    //      LDX #10                      load "10" into "x" register
    //      STX $0000                    store register "x" into ram at address 0x0000
    //      LDX #3                       load "3" into "x" register
    //      STX $0001                    store register "x" into ram at address 0x0001
    //      LDY $0000                    load from ram address 0x0000 into "y" register
    //      LDA #0                       load "0" into "a" register
    //      CLC                          unset "carry" flag
    //      loop                         just a label
    //      ADC $0001                    add from ram address 0x0001 (and carry, that's why we reset it above) to "a" register 
    //      DEY                          decrement "y" register by one
    //      BNE loop                     branch if "zero" flag not set, i.e. if register "y" is not zero
    //      STA $0002                    store register "a" into ram at address 0x0002
    //      NOP
    //      NOP
    //      NOP
    // decompiled: A2 0A 8E 00 00 A2 03 8E 01 00 AC 00 00 A9 00 18 6D 01 00 88 D0 FA 8D 02 00 EA EA EA
    mem[0x8000 + 0] = 0xA2;
    mem[0x8000 + 1] = 0x0A;
    mem[0x8000 + 2] = 0x8E;
    mem[0x8000 + 3] = 0x00;
    mem[0x8000 + 4] = 0x00;
    mem[0x8000 + 5] = 0xA2;
    mem[0x8000 + 6] = 0x03;
    mem[0x8000 + 7] = 0x8E;
    mem[0x8000 + 8] = 0x01;
    mem[0x8000 + 9] = 0x00;
    mem[0x8000 + 10] = 0xAC;
    mem[0x8000 + 11] = 0x00;
    mem[0x8000 + 12] = 0x00;
    mem[0x8000 + 13] = 0xA9;
    mem[0x8000 + 14] = 0x00;
    mem[0x8000 + 15] = 0x18;
    mem[0x8000 + 16] = 0x6D;
    mem[0x8000 + 17] = 0x01;
    mem[0x8000 + 18] = 0x00;
    mem[0x8000 + 19] = 0x88;
    mem[0x8000 + 20] = 0xD0;
    mem[0x8000 + 21] = 0xFA;
    mem[0x8000 + 22] = 0x8D;
    mem[0x8000 + 23] = 0x02;
    mem[0x8000 + 24] = 0x00;
    mem[0x8000 + 25] = 0xEA;
    mem[0x8000 + 26] = 0xEA;
    mem[0x8000 + 27] = 0xEA;
}

void emu_reset(struct emu *emu)
{
    emu->cpu = (struct cpu_6502){ 0 };
    emu->cpu.bus = &emu->cpu_bus;
    memset(emu->mem, 0, 0x1FFF);

    write_test(emu->mem);

    cpu_reset(&emu->cpu);
}

struct emu init_emu()
{
    struct emu emu =
    {
        .mem = (uint8_t*)malloc(1 << 16),
        .cpu_bus = init_bus(10),

        .paused = true,
        
        .font = LoadFont("../font.ttf")
    };

    // Set up memory map
    emu.ram = make_mirrored_mmio(make_mmio(0x0000, 0x1FFF, &emu.mem[0x0000]), 0x0000, 0x07FF);
    emu.ppu_registers = make_mirrored_mmio(make_mmio(0x2000, 0x3FFF, &emu.mem[0x2000]), 0x2000, 0x2007);
    emu.apu_registers = make_mmio(0x4000, 0x401F, &emu.mem[0x4000]);
    emu.cartridge = make_mmio(0x4020, 0xFFFF, &emu.mem[0x4020]);

    attach(&emu.cpu_bus, (struct mmio*)&emu.ram);
    attach(&emu.cpu_bus, (struct mmio*)&emu.ppu_registers);
    attach(&emu.cpu_bus, &emu.apu_registers);
    attach(&emu.cpu_bus, &emu.cartridge);

    emu_reset(&emu);

    return emu;
}

void clock(struct emu *emu)
{
    cpu_clock(&emu->cpu);
}

int main()
{
    InitWindow(1400, 960, "seenes");   
    SetTargetFPS(-1);

    struct emu emu = init_emu();

    while(!WindowShouldClose())
    {
        BeginDrawing();
        ClearBackground((Color) { 0, 0, 164 } );

        draw_working_mem(&emu);
        draw_cpu_state(&emu);
        draw_ram(&emu);

        draw_game(&emu);

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
            if (!IsKeyPressed(KEY_O))
                continue;
        }

        clock(&emu);
    }

    CloseWindow();

    return 0;
}
