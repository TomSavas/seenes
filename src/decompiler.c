#include <stdio.h>

#include "cpu_6502.h"

struct decompiled_loc
{
    char code[32];
    bool valid_loc;
};
void decompile(struct bus *bus, uint16_t addr_start, uint16_t addr_end, struct decompiled_loc *decompiled_src)
{
    struct cpu_6502 fake_cpu = { .bus = bus };

    uint16_t last_addr = addr_start;
    for(uint16_t addr = addr_start; addr <= addr_end;)
    {
        if (last_addr > addr)
            break;

        fake_cpu.reg = (struct cpu_registers){ 0 };
        fake_cpu.cycles_till_instruction_completion = 0;
        fake_cpu.instruction = (struct instruction) { "NOP", &nop, "IMP", &am_imp, 2 };
        fake_cpu.reg.pc = addr;
        cpu_clock(&fake_cpu);

        if (strcmp(fake_cpu.instruction.addr_mode_name, "ACC") == 0)
            sprintf(decompiled_src[addr].code, "%                 {ACC)", fake_cpu.instruction.op_code_name);
        else if (strcmp(fake_cpu.instruction.addr_mode_name, "ABS") == 0)
            sprintf(decompiled_src[addr].code, "%s %02x%02x           {ABS)", fake_cpu.instruction.op_code_name, fake_cpu.instruction.operand1, fake_cpu.instruction.operand0);
        else if (strcmp(fake_cpu.instruction.addr_mode_name, "ABX") == 0)
            sprintf(decompiled_src[addr].code, "%s %02x%02x [%04x]{ABX)", fake_cpu.instruction.op_code_name, fake_cpu.instruction.operand1, fake_cpu.instruction.operand0, fake_cpu.instruction.addr);
        else if (strcmp(fake_cpu.instruction.addr_mode_name, "ABY") == 0)
            sprintf(decompiled_src[addr].code, "%s %02x%02x [%04x]{ABY)", fake_cpu.instruction.op_code_name, fake_cpu.instruction.operand1, fake_cpu.instruction.operand0, fake_cpu.instruction.addr);
        else if (strcmp(fake_cpu.instruction.addr_mode_name, "IMM") == 0)
            sprintf(decompiled_src[addr].code, "%s #%02x            {IMM)", fake_cpu.instruction.op_code_name, fake_cpu.instruction.operand0);
        else if (strcmp(fake_cpu.instruction.addr_mode_name, "IMP") == 0)
            sprintf(decompiled_src[addr].code, "%s                {IMP)", fake_cpu.instruction.op_code_name);
        else if (strcmp(fake_cpu.instruction.addr_mode_name, "IND") == 0)
            sprintf(decompiled_src[addr].code, "%s %02x%02x [%04x]{IND)", fake_cpu.instruction.op_code_name, fake_cpu.instruction.operand1, fake_cpu.instruction.operand0, fake_cpu.instruction.addr);
        else if (strcmp(fake_cpu.instruction.addr_mode_name, "INX") == 0)
            sprintf(decompiled_src[addr].code, "%s %02x       [%04x]{INX)", fake_cpu.instruction.op_code_name, fake_cpu.instruction.operand0, fake_cpu.instruction.addr);
        else if (strcmp(fake_cpu.instruction.addr_mode_name, "INY") == 0)
            sprintf(decompiled_src[addr].code, "%s %02x       [%04x]{INY)", fake_cpu.instruction.op_code_name, fake_cpu.instruction.operand0, fake_cpu.instruction.addr);
        else if (strcmp(fake_cpu.instruction.addr_mode_name, "REL") == 0)
            sprintf(decompiled_src[addr].code, "%s %02x       [%04x]{REL)", fake_cpu.instruction.op_code_name, fake_cpu.instruction.operand0, fake_cpu.instruction.addr);
        else if (strcmp(fake_cpu.instruction.addr_mode_name, "ZP0") == 0)
            sprintf(decompiled_src[addr].code, "%s %02x           {ZP0)", fake_cpu.instruction.op_code_name, fake_cpu.instruction.operand0);
        else if (strcmp(fake_cpu.instruction.addr_mode_name, "ZPX") == 0)
            sprintf(decompiled_src[addr].code, "%s %02x       [%04x]{ZPX)", fake_cpu.instruction.op_code_name, fake_cpu.instruction.operand0, fake_cpu.instruction.addr);
        else if (strcmp(fake_cpu.instruction.addr_mode_name, "ZPY") == 0)
            sprintf(decompiled_src[addr].code, "%s %02x       [%04x]{ZPY)", fake_cpu.instruction.op_code_name, fake_cpu.instruction.operand0, fake_cpu.instruction.addr);
        else
            sprintf(decompiled_src[addr].code, "FAILED DECOMPILATION");

        decompiled_src[addr].valid_loc = true;
        for (uint16_t a = addr + 1; a < fake_cpu.reg.pc; a++)
            decompiled_src[a].valid_loc = false;

        last_addr = addr;
        addr = fake_cpu.reg.pc;
    }
}

