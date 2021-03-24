#include <stdbool.h>
#include <stdio.h>
#include <signal.h>

#include "cpu_6502.h"
#include "bus.h"

static const struct instruction instructions[] = 
{
/*         0x00                                0x01                                0x02                                0x03                                0x04                                0x05                                0x06                                0x07                                0x08                                0x09                                0x0A                                0x0B                                0x0C                                0x0D                                0x0E                                0x0F                             */
/* 0x00 */ { "BRK", &brk, "IMP", &am_imp, 7 }, { "ORA", &ora, "INX", &am_inx, 6 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 8 }, { "???", &nop, "IMP", &am_imp, 3 }, { "ORA", &ora, "ZP0", &am_zp0, 3 }, { "ASL", &asl, "ZP0", &am_zp0, 5 }, { "???", &nop, "IMP", &am_imp, 5 }, { "PHP", &php, "IMP", &am_imp, 3 }, { "ORA", &ora, "IMM", &am_imm, 2 }, { "ASL", &asl, "ACC", &am_acc, 2 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 4 }, { "ORA", &ora, "ABS", &am_abs, 4 }, { "ASL", &asl, "ABS", &am_abs, 6 }, { "???", &nop, "IMP", &am_imp, 6 },
/* 0x10 */ { "BPL", &bpl, "REL", &am_rel, 2 }, { "ORA", &ora, "INY", &am_iny, 5 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 8 }, { "???", &nop, "IMP", &am_imp, 4 }, { "ORA", &ora, "ZPX", &am_zpx, 4 }, { "ASL", &asl, "ZPX", &am_zpx, 6 }, { "???", &nop, "IMP", &am_imp, 6 }, { "CLC", &clc, "IMP", &am_imp, 2 }, { "ORA", &ora, "ABY", &am_aby, 4 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 7 }, { "???", &nop, "IMP", &am_imp, 4 }, { "ORA", &ora, "ABX", &am_abx, 4 }, { "ASL", &asl, "ABX", &am_abx, 7 }, { "???", &nop, "IMP", &am_imp, 7 },
/* 0x20 */ { "JSR", &jsr, "ABS", &am_abs, 6 }, { "AND", &and, "INX", &am_inx, 6 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 8 }, { "BIT", &bit, "ZP0", &am_zp0, 3 }, { "AND", &and, "ZP0", &am_zp0, 3 }, { "ROL", &rol, "ZP0", &am_zp0, 5 }, { "???", &nop, "IMP", &am_imp, 5 }, { "PLP", &plp, "IMP", &am_imp, 4 }, { "AND", &and, "IMM", &am_imm, 2 }, { "ROL", &rol, "ACC", &am_acc, 2 }, { "???", &nop, "IMP", &am_imp, 2 }, { "BIT", &bit, "ABS", &am_abs, 4 }, { "AND", &and, "ABS", &am_abs, 4 }, { "ROL", &rol, "ABS", &am_abs, 6 }, { "???", &nop, "IMP", &am_imp, 6 },
/* 0x30 */ { "BMI", &bmi, "REL", &am_rel, 2 }, { "AND", &and, "INY", &am_iny, 5 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 8 }, { "???", &nop, "IMP", &am_imp, 4 }, { "AND", &and, "ZPX", &am_zpx, 4 }, { "ROL", &rol, "ZPX", &am_zpx, 6 }, { "???", &nop, "IMP", &am_imp, 6 }, { "SEC", &sec, "IMP", &am_imp, 2 }, { "AND", &and, "ABY", &am_aby, 4 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 7 }, { "???", &nop, "IMP", &am_imp, 4 }, { "AND", &and, "ABX", &am_abx, 4 }, { "ROL", &rol, "ABX", &am_abx, 7 }, { "???", &nop, "IMP", &am_imp, 7 },
/* 0x40 */ { "RTI", &rti, "IMP", &am_imp, 6 }, { "EOR", &eor, "INX", &am_inx, 6 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 8 }, { "???", &nop, "IMP", &am_imp, 3 }, { "EOR", &eor, "ZP0", &am_zp0, 3 }, { "LSR", &lsr, "ZP0", &am_zp0, 5 }, { "???", &nop, "IMP", &am_imp, 5 }, { "PHA", &pha, "IMP", &am_imp, 3 }, { "EOR", &eor, "IMM", &am_imm, 2 }, { "LSR", &lsr, "ACC", &am_acc, 2 }, { "???", &nop, "IMP", &am_imp, 2 }, { "JMP", &jmp, "ABS", &am_abs, 3 }, { "EOR", &eor, "ABS", &am_abs, 4 }, { "LSR", &lsr, "ABS", &am_abs, 6 }, { "???", &nop, "IMP", &am_imp, 6 },
/* 0x50 */ { "BVC", &bvc, "REL", &am_rel, 2 }, { "EOR", &eor, "INY", &am_iny, 5 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 8 }, { "???", &nop, "IMP", &am_imp, 4 }, { "EOR", &eor, "ZPX", &am_zpx, 4 }, { "LSR", &lsr, "ZPX", &am_zpx, 6 }, { "???", &nop, "IMP", &am_imp, 6 }, { "CLI", &cli, "IMP", &am_imp, 2 }, { "EOR", &eor, "ABY", &am_aby, 4 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 7 }, { "???", &nop, "IMP", &am_imp, 4 }, { "EOR", &eor, "ABX", &am_abx, 4 }, { "LSR", &lsr, "ABX", &am_abx, 7 }, { "???", &nop, "IMP", &am_imp, 7 },
/* 0x60 */ { "RTS", &rts, "IMP", &am_imp, 6 }, { "ADC", &adc, "INX", &am_inx, 6 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 8 }, { "???", &nop, "IMP", &am_imp, 3 }, { "ADC", &adc, "ZP0", &am_zp0, 3 }, { "ROR", &ror, "ZP0", &am_zp0, 5 }, { "???", &nop, "IMP", &am_imp, 5 }, { "PLA", &pla, "IMP", &am_imp, 4 }, { "ADC", &adc, "IMM", &am_imm, 2 }, { "ROR", &ror, "ACC", &am_acc, 2 }, { "???", &nop, "IMP", &am_imp, 2 }, { "JMP", &jmp, "IND", &am_ind, 5 }, { "ADC", &adc, "ABS", &am_abs, 4 }, { "ROR", &ror, "ABS", &am_abs, 6 }, { "???", &nop, "IMP", &am_imp, 6 },
/* 0x70 */ { "BVS", &bvs, "REL", &am_rel, 2 }, { "ADC", &adc, "INY", &am_iny, 5 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 8 }, { "???", &nop, "IMP", &am_imp, 4 }, { "ADC", &adc, "ZPX", &am_zpx, 4 }, { "ROR", &ror, "ZPX", &am_zpx, 6 }, { "???", &nop, "IMP", &am_imp, 6 }, { "SEI", &sei, "IMP", &am_imp, 2 }, { "ADC", &adc, "ABY", &am_aby, 4 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 7 }, { "???", &nop, "IMP", &am_imp, 4 }, { "ADC", &adc, "ABX", &am_abx, 4 }, { "ROR", &ror, "ABX", &am_abx, 7 }, { "???", &nop, "IMP", &am_imp, 7 },
/* 0x80 */ { "???", &nop, "IMP", &am_imp, 2 }, { "STA", &sta, "INX", &am_inx, 6 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 6 }, { "STY", &sty, "ZP0", &am_zp0, 3 }, { "STA", &sta, "ZP0", &am_zp0, 3 }, { "STX", &stx, "ZP0", &am_zp0, 3 }, { "???", &nop, "IMP", &am_imp, 3 }, { "DEY", &dey, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 2 }, { "TXA", &txa, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 2 }, { "STY", &sty, "ABS", &am_abs, 4 }, { "STA", &sta, "ABS", &am_abs, 4 }, { "STX", &stx, "ABS", &am_abs, 4 }, { "???", &nop, "IMP", &am_imp, 4 },
/* 0x90 */ { "BCC", &bcc, "REL", &am_rel, 2 }, { "STA", &sta, "INY", &am_iny, 6 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 6 }, { "STY", &sty, "ZPX", &am_zpx, 4 }, { "STA", &sta, "ZPX", &am_zpx, 4 }, { "STX", &stx, "ZPY", &am_zpy, 4 }, { "???", &nop, "IMP", &am_imp, 4 }, { "TYA", &tya, "IMP", &am_imp, 2 }, { "STA", &sta, "ABY", &am_aby, 5 }, { "TXS", &txs, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 5 }, { "???", &nop, "IMP", &am_imp, 5 }, { "STA", &sta, "ABX", &am_abx, 5 }, { "???", &nop, "IMP", &am_imp, 5 }, { "???", &nop, "IMP", &am_imp, 5 },
/* 0xA0 */ { "LDY", &ldy, "IMM", &am_imm, 2 }, { "LDA", &lda, "INX", &am_inx, 6 }, { "LDX", &ldx, "IMM", &am_imm, 2 }, { "???", &nop, "IMP", &am_imp, 6 }, { "LDY", &ldy, "ZP0", &am_zp0, 3 }, { "LDA", &lda, "ZP0", &am_zp0, 3 }, { "LDX", &ldx, "ZP0", &am_zp0, 3 }, { "???", &nop, "IMP", &am_imp, 3 }, { "TAY", &tay, "IMP", &am_imp, 2 }, { "LDA", &lda, "IMM", &am_imm, 2 }, { "TAX", &tax, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 2 }, { "LDY", &ldy, "ABS", &am_abs, 4 }, { "LDA", &lda, "ABS", &am_abs, 4 }, { "LDX", &ldx, "ABS", &am_abs, 4 }, { "???", &nop, "IMP", &am_imp, 4 },
/* 0xB0 */ { "BCS", &bcs, "REL", &am_rel, 2 }, { "LDA", &lda, "INY", &am_iny, 5 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 5 }, { "LDY", &ldy, "ZPX", &am_zpx, 4 }, { "LDA", &lda, "ZPX", &am_zpx, 4 }, { "LDX", &ldx, "ZPY", &am_zpy, 4 }, { "???", &nop, "IMP", &am_imp, 4 }, { "CLV", &clv, "IMP", &am_imp, 2 }, { "LDA", &lda, "ABY", &am_aby, 4 }, { "TSX", &tsx, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 4 }, { "LDY", &ldy, "ABX", &am_abx, 4 }, { "LDA", &lda, "ABX", &am_abx, 4 }, { "LDX", &ldx, "ABY", &am_aby, 4 }, { "???", &nop, "IMP", &am_imp, 4 },
/* 0xC0 */ { "CPY", &cpy, "IMM", &am_imm, 2 }, { "CMP", &cmp, "INX", &am_inx, 6 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 8 }, { "CPY", &cpy, "ZP0", &am_zp0, 3 }, { "CMP", &cmp, "ZP0", &am_zp0, 3 }, { "DEC", &dec, "ZP0", &am_zp0, 5 }, { "???", &nop, "IMP", &am_imp, 5 }, { "INY", &iny, "IMP", &am_imp, 2 }, { "CMP", &cmp, "IMM", &am_imm, 2 }, { "DEX", &dex, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 2 }, { "CPY", &cpy, "ABS", &am_abs, 4 }, { "CMP", &cmp, "ABS", &am_abs, 4 }, { "DEC", &dec, "ABS", &am_abs, 6 }, { "???", &nop, "IMP", &am_imp, 6 },
/* 0xD0 */ { "BNE", &bne, "REL", &am_rel, 2 }, { "CMP", &cmp, "INY", &am_iny, 5 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 8 }, { "???", &nop, "IMP", &am_imp, 4 }, { "CMP", &cmp, "ZPX", &am_zpx, 4 }, { "DEC", &dec, "ZPX", &am_zpx, 6 }, { "???", &nop, "IMP", &am_imp, 6 }, { "CLD", &cld, "IMP", &am_imp, 2 }, { "CMP", &cmp, "ABY", &am_aby, 4 }, { "NOP", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 7 }, { "???", &nop, "IMP", &am_imp, 4 }, { "CMP", &cmp, "ABX", &am_abx, 4 }, { "DEC", &dec, "ABX", &am_abx, 7 }, { "???", &nop, "IMP", &am_imp, 7 },
/* 0xE0 */ { "CPX", &cpx, "IMM", &am_imm, 2 }, { "SBC", &sbc, "INX", &am_inx, 6 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 8 }, { "CPX", &cpx, "ZP0", &am_zp0, 3 }, { "SBC", &sbc, "ZP0", &am_zp0, 3 }, { "INC", &inc, "ZP0", &am_zp0, 5 }, { "???", &nop, "IMP", &am_imp, 5 }, { "INX", &inx, "IMP", &am_imp, 2 }, { "SBC", &sbc, "IMM", &am_imm, 2 }, { "NOP", &nop, "IMP", &am_imp, 2 }, { "???", &sbc, "IMP", &am_imp, 2 }, { "CPX", &cpx, "ABS", &am_abs, 4 }, { "SBC", &sbc, "ABS", &am_abs, 4 }, { "INC", &inc, "ABS", &am_abs, 6 }, { "???", &nop, "IMP", &am_imp, 6 },
/* 0xF0 */ { "BEQ", &beq, "REL", &am_rel, 2 }, { "SBC", &sbc, "INY", &am_iny, 5 }, { "???", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 8 }, { "???", &nop, "IMP", &am_imp, 4 }, { "SBC", &sbc, "ZPX", &am_zpx, 4 }, { "INC", &inc, "ZPX", &am_zpx, 6 }, { "???", &nop, "IMP", &am_imp, 6 }, { "SED", &sed, "IMP", &am_imp, 2 }, { "SBC", &sbc, "ABY", &am_aby, 4 }, { "NOP", &nop, "IMP", &am_imp, 2 }, { "???", &nop, "IMP", &am_imp, 7 }, { "???", &nop, "IMP", &am_imp, 4 }, { "SBC", &sbc, "ABX", &am_abx, 4 }, { "INC", &inc, "ABX", &am_abx, 7 }, { "???", &nop, "IMP", &am_imp, 7 }
};

uint8_t instruction_data(const struct cpu_6502 *cpu)
{
    return cpu->instruction.addressing_mode_fn == &am_imm || cpu->instruction.addressing_mode_fn == &am_acc
        ? cpu->instruction.operand0
        : bus_read(cpu->bus, cpu->instruction.addr);
}

uint8_t get_flag(struct cpu_6502 *cpu, enum cpu_status_flag flag)
{
    return (cpu->reg.s.reg & flag) != 0 ? 1 : 0;
}

void set_flag(struct cpu_6502 *cpu, enum cpu_status_flag flag, bool set)
{
    if (set)
        cpu->reg.s.reg |= flag;
    else
        cpu->reg.s.reg &= ~flag;
}

void branch(struct cpu_6502 *cpu)
{
    // Branch taken, +1 cycle penalty
    cpu->cycles_till_instruction_completion++;
    // Crossed page boundary, +1 cycle penalty
    if ((cpu->reg.pc & 0xFF00) != (cpu->instruction.addr & 0xFF00))
        cpu->cycles_till_instruction_completion++;

    cpu->reg.pc = cpu->instruction.addr;
}

void stack_push(struct cpu_6502 *cpu, uint8_t val)
{
    bus_write(cpu->bus, 0x0100 | (uint16_t)(cpu->reg.sp--), val);
}

void stack_addr_push(struct cpu_6502 *cpu, uint16_t ptr)
{
    stack_push(cpu, (uint8_t)(ptr >> 8));
    stack_push(cpu, (uint8_t)(ptr & 0x00FF));
}

uint8_t stack_pull(struct cpu_6502 *cpu)
{
    return bus_read(cpu->bus, 0x0100 | (uint16_t)(++cpu->reg.sp));
}

uint16_t stack_addr_pull(struct cpu_6502 *cpu)
{
    uint16_t low = stack_pull(cpu);
    uint16_t high = stack_pull(cpu);

    return (high << 8) | low;
}

uint8_t fetch(struct cpu_6502 *cpu)
{
    return bus_read(cpu->bus, cpu->reg.pc++);
}

bool cpu_clock(struct cpu_6502 *cpu)
{
    if (cpu->cycles_till_instruction_completion == 0)
    {
        cpu->raw_instruction = fetch(cpu);
        cpu->instruction = instructions[cpu->raw_instruction];
        cpu->cycles_till_instruction_completion = cpu->instruction.base_cycle_count;

        cpu->instruction.addressing_mode_fn(cpu);
        if (!cpu->disable_instruction_execution)
            cpu->instruction.instruction_fn(cpu);
    }

    cpu->cycles_till_instruction_completion--;
    cpu->cycles++;

    return true;
}

void cpu_reset(struct cpu_6502 *cpu) 
{
    cpu->reg = (struct cpu_registers){ 0 };
    cpu->instruction = instructions[0xEA]; // NOP
    uint16_t low = (uint16_t)bus_read(cpu->bus, 0xFFFC);
    uint16_t high = (uint16_t)bus_read(cpu->bus, 0xFFFD) << 8;
    uint16_t addr = high | low;
    cpu->reg.pc = addr;
    cpu->cycles_till_instruction_completion = 8;
}

void irq(struct cpu_6502 *cpu)
{
    if (!get_flag(cpu, CPU_I))
        return;

    stack_addr_push(cpu, cpu->reg.pc);

    set_flag(cpu, CPU_B, false);
    set_flag(cpu, CPU_U, true);
    set_flag(cpu, CPU_I, true);
    stack_push(cpu, cpu->reg.s.reg);

    cpu->reg.pc = ((uint16_t)bus_read(cpu->bus, 0xFFFF) << 8) | (uint16_t)bus_read(cpu->bus, 0xFFFE);
    cpu->cycles_till_instruction_completion = 7;
}

void nmi(struct cpu_6502 *cpu)
{
    stack_addr_push(cpu, cpu->reg.pc);

    set_flag(cpu, CPU_B, false);
    set_flag(cpu, CPU_U, true);
    set_flag(cpu, CPU_I, true);
    stack_push(cpu, cpu->reg.s.reg);

    cpu->reg.pc = ((uint16_t)bus_read(cpu->bus, 0xFFFB) << 8) | (uint16_t)bus_read(cpu->bus, 0xFFFA);
    cpu->cycles_till_instruction_completion = 8;
}

void adc(struct cpu_6502 *cpu)
{
    uint16_t data = (uint16_t)instruction_data(cpu);
    uint16_t res = (uint16_t)cpu->reg.a + data + (uint16_t)get_flag(cpu, CPU_C);

    set_flag(cpu, CPU_C, res > 0x00FF);
    set_flag(cpu, CPU_Z, (res & 0x00FF) == 0x00);
    set_flag(cpu, CPU_N, (res & 0x0080) != 0x00);

    bool overflow = ~(cpu->reg.a & 0x80) & ~(data & 0x80) & (res & 0x0080);
    bool underflow = (cpu->reg.a & 0x80) & (data & 0x80) & ~(res & 0x0080);
    set_flag(cpu, CPU_V, overflow || underflow);

    cpu->reg.a = res & 0x00FF;
}

void and(struct cpu_6502 *cpu)
{
    cpu->reg.a = cpu->reg.a & instruction_data(cpu);

    set_flag(cpu, CPU_Z, (cpu->reg.a & 0x00FF) == 0x00);
    set_flag(cpu, CPU_N, cpu->reg.a & 0x0080);
}

void asl(struct cpu_6502 *cpu)
{
    uint16_t res = (uint16_t)(instruction_data(cpu)) << 1;

    set_flag(cpu, CPU_C, res > 0x00FF);
    set_flag(cpu, CPU_Z, (res & 0x00FF) == 0x00);
    set_flag(cpu, CPU_N, res & 0x0080);

    if (cpu->instruction.addressing_mode_fn == &am_acc)
        cpu->reg.a = (uint8_t)(res & 0x00FF);
    else
        bus_write(cpu->bus, cpu->instruction.addr, (uint8_t)(res & 0x00FF));
}

void bcc(struct cpu_6502 *cpu)
{
    if (!get_flag(cpu, CPU_C))
        branch(cpu);
}

void bcs(struct cpu_6502 *cpu)
{
    if (get_flag(cpu, CPU_C))
        branch(cpu);
}

void beq(struct cpu_6502 *cpu)
{
    if (get_flag(cpu, CPU_Z))
        branch(cpu);
}

void bit(struct cpu_6502 *cpu)
{
    uint8_t byte = instruction_data(cpu);

    set_flag(cpu, CPU_N, byte & 0x80);
    set_flag(cpu, CPU_Z, (byte & cpu->reg.a) == 0x00);
    set_flag(cpu, CPU_V, byte & 0x40);
}

void bmi(struct cpu_6502 *cpu)
{
    if (get_flag(cpu, CPU_N))
        branch(cpu);
}

void bne(struct cpu_6502 *cpu)
{
    if (!get_flag(cpu, CPU_Z))
        branch(cpu);
}

void bpl(struct cpu_6502 *cpu)
{
    if (!get_flag(cpu, CPU_N))
        branch(cpu);
}

void brk(struct cpu_6502 *cpu)
{
    set_flag(cpu, CPU_I, true);
    stack_addr_push(cpu, cpu->reg.pc);

    set_flag(cpu, CPU_B, true);
    stack_push(cpu, cpu->reg.sp);
    set_flag(cpu, CPU_B, false);

    cpu->reg.pc = ((uint16_t)bus_read(cpu->bus, 0xFFFF) << 8) | (uint16_t)bus_read(cpu->bus, 0xFFFE);
}

void bvc(struct cpu_6502 *cpu)
{
    if (!get_flag(cpu, CPU_V))
        branch(cpu);
}

void bvs(struct cpu_6502 *cpu)
{
    if (get_flag(cpu, CPU_V))
        branch(cpu);
}

void clc(struct cpu_6502 *cpu)
{
    set_flag(cpu, CPU_C, false);
}

void cld(struct cpu_6502 *cpu)
{
    set_flag(cpu, CPU_D, false);
}

void cli(struct cpu_6502 *cpu)
{
    set_flag(cpu, CPU_I, false);
}

void clv(struct cpu_6502 *cpu)
{
    set_flag(cpu, CPU_V, false);
}

void cmp(struct cpu_6502 *cpu)
{
    uint16_t data = (uint16_t)instruction_data(cpu);
    uint16_t byte = (uint16_t)cpu->reg.a - data;

    set_flag(cpu, CPU_N, byte & 0x0080);
    set_flag(cpu, CPU_Z, (byte & 0x00FF) == 0x0000);
    set_flag(cpu, CPU_C, cpu->reg.a >= data);
}

void cpx(struct cpu_6502 *cpu)
{
    uint16_t data = (uint16_t)instruction_data(cpu);
    uint16_t byte = (uint16_t)cpu->reg.x - data;

    set_flag(cpu, CPU_N, byte & 0x0080);
    set_flag(cpu, CPU_Z, (byte & 0x00FF) == 0x0000);
    set_flag(cpu, CPU_C, cpu->reg.x >= data);
}

void cpy(struct cpu_6502 *cpu)
{
    uint16_t data = (uint16_t)instruction_data(cpu);
    uint16_t byte = (uint16_t)cpu->reg.y - data;

    set_flag(cpu, CPU_N, byte & 0x0080);
    set_flag(cpu, CPU_Z, (byte & 0x00FF) == 0x0000);
    set_flag(cpu, CPU_C, cpu->reg.y >= data);
}

void dec(struct cpu_6502 *cpu)
{
    uint8_t res = instruction_data(cpu) - 1;

    set_flag(cpu, CPU_N, res & 0x80);
    set_flag(cpu, CPU_Z, res == 0);

    bus_write(cpu->bus, cpu->instruction.addr, res);
}

void dex(struct cpu_6502 *cpu)
{
    cpu->reg.x--;

    set_flag(cpu, CPU_N, cpu->reg.x & 0x80);
    set_flag(cpu, CPU_Z, cpu->reg.x == 0);
}

void dey(struct cpu_6502 *cpu)
{
    cpu->reg.y--;

    set_flag(cpu, CPU_N, cpu->reg.y & 0x80);
    set_flag(cpu, CPU_Z, cpu->reg.y == 0);
}

void eor(struct cpu_6502 *cpu)
{
    cpu->reg.a ^= instruction_data(cpu);

    set_flag(cpu, CPU_N, cpu->reg.a & 0x80);
    set_flag(cpu, CPU_Z, cpu->reg.a == 0);
}

void inc(struct cpu_6502 *cpu)
{
    uint8_t res = instruction_data(cpu) + 1;

    set_flag(cpu, CPU_N, res & 0x80);
    set_flag(cpu, CPU_Z, res == 0);

    bus_write(cpu->bus, cpu->instruction.addr, res);
}

void inx(struct cpu_6502 *cpu)
{
    cpu->reg.x++;

    set_flag(cpu, CPU_N, cpu->reg.x & 0x80);
    set_flag(cpu, CPU_Z, cpu->reg.x == 0);
}

void iny(struct cpu_6502 *cpu)
{
    cpu->reg.y++;

    set_flag(cpu, CPU_N, cpu->reg.y & 0x80);
    set_flag(cpu, CPU_Z, cpu->reg.y == 0);
}

void jmp(struct cpu_6502 *cpu)
{
    cpu->reg.pc = cpu->instruction.addr;
}

void jsr(struct cpu_6502 *cpu)
{
    stack_addr_push(cpu, cpu->reg.pc);
    cpu->reg.pc = cpu->instruction.addr;
}

void lda(struct cpu_6502 *cpu)
{
    cpu->reg.a = instruction_data(cpu);

    set_flag(cpu, CPU_N, cpu->reg.a & 0x80);
    set_flag(cpu, CPU_Z, cpu->reg.a == 0);
}

void ldx(struct cpu_6502 *cpu)
{
    cpu->reg.x = instruction_data(cpu);

    set_flag(cpu, CPU_N, cpu->reg.x & 0x80);
    set_flag(cpu, CPU_Z, cpu->reg.x == 0);
}

void ldy(struct cpu_6502 *cpu)
{
    cpu->reg.y = instruction_data(cpu);

    set_flag(cpu, CPU_N, cpu->reg.y & 0x80);
    set_flag(cpu, CPU_Z, cpu->reg.y == 0);
}

void lsr(struct cpu_6502 *cpu)
{
    uint8_t res = instruction_data(cpu);

    set_flag(cpu, CPU_C, res & 0x01);

    res >>= 1;

    set_flag(cpu, CPU_Z, res == 0x00);
    set_flag(cpu, CPU_N, res & 0x80);

    if (cpu->instruction.addressing_mode_fn == &am_acc)
        cpu->reg.a = res;
    else
        bus_write(cpu->bus, cpu->instruction.addr, res);
}

void nop(struct cpu_6502 *cpu)
{
}

void ora(struct cpu_6502 *cpu)
{
    cpu->reg.a |= instruction_data(cpu);

    set_flag(cpu, CPU_N, cpu->reg.a & 0x80);
    set_flag(cpu, CPU_Z, cpu->reg.a == 0x00);
}

void pha(struct cpu_6502 *cpu)
{
    stack_push(cpu, cpu->reg.a);
}

void php(struct cpu_6502 *cpu)
{
    set_flag(cpu, CPU_B, true);
    set_flag(cpu, CPU_U, true);
    stack_push(cpu, cpu->reg.s.reg);
    set_flag(cpu, CPU_B, false);
    set_flag(cpu, CPU_U, false);
}

void pla(struct cpu_6502 *cpu)
{
    cpu->reg.a = stack_pull(cpu);
    set_flag(cpu, CPU_N, cpu->reg.a & 0x80);
    set_flag(cpu, CPU_Z, cpu->reg.a == 0x00);
}

void plp(struct cpu_6502 *cpu)
{
    cpu->reg.s.reg = stack_pull(cpu);
    set_flag(cpu, CPU_U, true);
}

void rol(struct cpu_6502 *cpu)
{
    uint16_t res = (uint16_t)(instruction_data(cpu)) << 1;
    res |= (uint16_t)get_flag(cpu, CPU_C);

    set_flag(cpu, CPU_C, res > 0x00FF);

    set_flag(cpu, CPU_Z, (res & 0x00FF) == 0x00);
    set_flag(cpu, CPU_N, res & 0x0080);

    if (cpu->instruction.addressing_mode_fn == &am_acc)
        cpu->reg.a = res & 0x00FF;
    else
        bus_write(cpu->bus, cpu->instruction.addr, res & 0x00FF);
}

void ror(struct cpu_6502 *cpu)
{
    uint8_t res = instruction_data(cpu);

    set_flag(cpu, CPU_C, res & 0x01);

    res >>= 1;
    res |= get_flag(cpu, CPU_C) << 7;

    set_flag(cpu, CPU_Z, res == 0x00);
    set_flag(cpu, CPU_N, res & 0x80);

    if (cpu->instruction.addressing_mode_fn == &am_acc)
        cpu->reg.a = res;
    else
        bus_write(cpu->bus, cpu->instruction.addr, res);
}

void rti(struct cpu_6502 *cpu)
{
    cpu->reg.s.reg = stack_pull(cpu);
    set_flag(cpu, CPU_B, false);
    set_flag(cpu, CPU_U, false);

    cpu->reg.pc = stack_addr_pull(cpu);
}

void rts(struct cpu_6502 *cpu)
{
    cpu->reg.pc = stack_addr_pull(cpu);
}

void sbc(struct cpu_6502 *cpu)
{
    uint16_t data = (uint16_t)instruction_data(cpu);

    // res = A - data - ~C
    // res = A + (~data + 1) - ~C
    // res = A + (~data + 1) - (1 - C)
    // res = A + ~data + C
    
    uint16_t res = (uint16_t)cpu->reg.a + ~data + (uint16_t)get_flag(cpu, CPU_C);

    set_flag(cpu, CPU_C, res > 0x00FF);
    set_flag(cpu, CPU_Z, (res & 0x00FF) == 0x00);
    set_flag(cpu, CPU_N, (res & 0x0080) != 0x00);

    bool overflow = ~(cpu->reg.a & 0x80) & (data & 0x80) & (res & 0x0080);
    bool underflow = (cpu->reg.a & 0x80) & ~(data & 0x80) & ~(res & 0x0080);
    set_flag(cpu, CPU_V, overflow || underflow);

    cpu->reg.a = res & 0x00FF;
}

void sec(struct cpu_6502 *cpu)
{
    set_flag(cpu, CPU_C, true);
}

void sed(struct cpu_6502 *cpu)
{
    set_flag(cpu, CPU_D, true);
}

void sei(struct cpu_6502 *cpu)
{
    set_flag(cpu, CPU_I, true);
}

void sta(struct cpu_6502 *cpu)
{
    bus_write(cpu->bus, cpu->instruction.addr, cpu->reg.a);
}

void stx(struct cpu_6502 *cpu)
{
    bus_write(cpu->bus, cpu->instruction.addr, cpu->reg.x);
}

void sty(struct cpu_6502 *cpu)
{
    bus_write(cpu->bus, cpu->instruction.addr, cpu->reg.y);
}

void tax(struct cpu_6502 *cpu)
{
    cpu->reg.x = cpu->reg.a;

    set_flag(cpu, CPU_Z, cpu->reg.x == 0x00);
    set_flag(cpu, CPU_N, cpu->reg.x & 0x80);
}

void tay(struct cpu_6502 *cpu)
{
    cpu->reg.y = cpu->reg.a;

    set_flag(cpu, CPU_Z, cpu->reg.y == 0x00);
    set_flag(cpu, CPU_N, cpu->reg.y & 0x80);
}

void tsx(struct cpu_6502 *cpu)
{
    cpu->reg.x = cpu->reg.sp;

    set_flag(cpu, CPU_Z, cpu->reg.x == 0x00);
    set_flag(cpu, CPU_N, cpu->reg.x & 0x80);
}

void txa(struct cpu_6502 *cpu)
{
    cpu->reg.a = cpu->reg.x;

    set_flag(cpu, CPU_Z, cpu->reg.a == 0x00);
    set_flag(cpu, CPU_N, cpu->reg.a & 0x80);
}

void txs(struct cpu_6502 *cpu)
{
    cpu->reg.sp = cpu->reg.x;
}

void tya(struct cpu_6502 *cpu)
{
    cpu->reg.a = cpu->reg.y;

    set_flag(cpu, CPU_Z, cpu->reg.a == 0x00);
    set_flag(cpu, CPU_N, cpu->reg.a & 0x80);
}

// Addressing modes
void am_acc(struct cpu_6502 *cpu)
{
    cpu->instruction.operand0 = cpu->reg.a;   
}

void am_abs(struct cpu_6502 *cpu)
{
    uint8_t low = fetch(cpu);
    uint8_t high = fetch(cpu);

    cpu->instruction.addr = ((uint16_t)high << 8) | (uint16_t)low;
    cpu->instruction.operand0 = low;
    cpu->instruction.operand1 = high;
}

void am_abx(struct cpu_6502 *cpu)
{
    uint8_t low = fetch(cpu);
    uint8_t high = fetch(cpu);

    cpu->instruction.addr = (((uint16_t)high << 8) | (uint16_t)low) + (uint16_t)cpu->reg.x;
    cpu->instruction.operand0 = low;
    cpu->instruction.operand1 = high;

    // Crossed page boundary, +1 cycle penalty
    if ((cpu->instruction.addr & 0xFF00) != ((uint16_t)high << 8))
        cpu->cycles_till_instruction_completion++;
}

void am_aby(struct cpu_6502 *cpu)
{
    uint8_t low = fetch(cpu);
    uint8_t high = fetch(cpu);

    cpu->instruction.addr = (((uint16_t)high << 8) | (uint16_t)low) + (uint16_t)cpu->reg.y;
    cpu->instruction.operand0 = low;
    cpu->instruction.operand1 = high;

    // Crossed page boundary, +1 cycle penalty
    if ((cpu->instruction.addr & 0xFF00) != ((uint16_t)high << 8))
        cpu->cycles_till_instruction_completion++;
}

void am_imm(struct cpu_6502 *cpu)
{
    cpu->instruction.operand0 = fetch(cpu);
}

void am_imp(struct cpu_6502 *cpu)
{
    // operand is implied -- specified by the instruction
}

void am_ind(struct cpu_6502 *cpu)
{
    uint8_t low = fetch(cpu);
    uint8_t high = fetch(cpu);

    uint16_t ptr = ((uint16_t)high << 8) | (uint16_t)low;

    if (low == 0x00FF)
        cpu->instruction.addr = ((uint16_t)bus_read(cpu->bus, ptr & 0xFF00) << 8) | (uint16_t)bus_read(cpu->bus, ptr);
    else
        cpu->instruction.addr = ((uint16_t)bus_read(cpu->bus, ptr + 1) << 8) | (uint16_t)bus_read(cpu->bus, ptr);
    cpu->instruction.operand0 = low;
    cpu->instruction.operand1 = high;

    // According to javidx9 there's a bug:
    /*
    if (low == 0xFF)
        cpu->instruction.addr = ((uint16_t)bus_read(cpu->bus, ptr & 0xFF00) << 8) | (uint16_t)bus_read(cpu->bus, ptr);
    */
}

// Aka indexed indirect
void am_inx(struct cpu_6502 *cpu)
{
    cpu->instruction.operand0 = fetch(cpu);
    uint16_t ptr = (uint16_t)cpu->instruction.operand0 + (uint16_t)cpu->reg.x;

    uint8_t low = bus_read(cpu->bus, ptr);
    uint8_t high = bus_read(cpu->bus, ptr + 1);

    cpu->instruction.addr = ((uint16_t)high << 8) | (uint16_t)low;
}

// Aka indirect indexed
void am_iny(struct cpu_6502 *cpu)
{
    cpu->instruction.operand0 = fetch(cpu);
    uint16_t ptr = (uint16_t)cpu->instruction.operand0;

    uint8_t low = bus_read(cpu->bus, ptr);
    uint8_t high = bus_read(cpu->bus, ptr + 1);

    cpu->instruction.addr = (((uint16_t)high << 8) | (uint16_t)low) + (uint16_t)cpu->reg.y;

    // Crossed page boundary, +1 cycle penalty
    if ((cpu->instruction.addr & 0xFF00) != ((uint16_t)high << 8))
        cpu->cycles_till_instruction_completion++;
}

void am_rel(struct cpu_6502 *cpu)
{
    cpu->instruction.operand0 = fetch(cpu);
    int16_t offset = (int8_t)cpu->instruction.operand0;
    cpu->instruction.addr = (int16_t)cpu->reg.pc + offset;

    // Crossed page boundary, +1 cycle penalty
    if ((cpu->instruction.addr & 0xFF00) != (cpu->reg.pc << 8))
        cpu->cycles_till_instruction_completion++;
}

void am_zp0(struct cpu_6502 *cpu)
{
    cpu->instruction.operand0 = fetch(cpu);
    cpu->instruction.addr = (uint16_t)cpu->instruction.operand0 & 0x00FF;
}

void am_zpx(struct cpu_6502 *cpu)
{
    cpu->instruction.operand0 = fetch(cpu);
    cpu->instruction.addr = (uint16_t)cpu->instruction.operand0 + cpu->reg.x;
    // To remove carry
    cpu->instruction.addr &= 0x00FF;
}

void am_zpy(struct cpu_6502 *cpu)
{
    cpu->instruction.operand0 = fetch(cpu);
    cpu->instruction.addr = (uint16_t)cpu->instruction.operand0 + cpu->reg.y;
    // To remove carry
    cpu->instruction.addr &= 0x00FF;
}
