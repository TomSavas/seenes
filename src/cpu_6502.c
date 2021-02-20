#include <stdbool.h>
#include <stdio.h>

#include "cpu_6502.h"
#include "bus.h"

static const struct instruction instructions[] = 
{
/*         0x00                         0x01                         0x02                         0x03                         0x04                         0x05                         0x06                         0x07                         0x08                         0x09                         0x0A                         0x0B                         0x0C                         0x0D                         0x0E                         0x0F                   */
/* 0x00 */ { "BRK", &brk, &am_imp, 7 }, { "ORA", &ora, &am_inx, 6 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 8 }, { "???", &nop, &am_imp, 3 }, { "ORA", &ora, &am_zp0, 3 }, { "ASL", &asl, &am_zp0, 5 }, { "???", &nop, &am_imp, 5 }, { "PHP", &php, &am_imp, 3 }, { "ORA", &ora, &am_imm, 2 }, { "ASL", &asl, &am_acc, 2 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 4 }, { "ORA", &ora, &am_abs, 4 }, { "ASL", &asl, &am_abs, 6 }, { "???", &nop, &am_imp, 6 },
/* 0x10 */ { "BPL", &bpl, &am_rel, 2 }, { "ORA", &ora, &am_iny, 5 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 8 }, { "???", &nop, &am_imp, 4 }, { "ORA", &ora, &am_zpx, 4 }, { "ASL", &asl, &am_zpx, 6 }, { "???", &nop, &am_imp, 6 }, { "CLC", &clc, &am_imp, 2 }, { "ORA", &ora, &am_aby, 4 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 7 }, { "???", &nop, &am_imp, 4 }, { "ORA", &ora, &am_abx, 4 }, { "ASL", &asl, &am_abx, 7 }, { "???", &nop, &am_imp, 7 },
/* 0x20 */ { "JSR", &jsr, &am_abs, 6 }, { "AND", &and, &am_inx, 6 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 8 }, { "BIT", &bit, &am_zp0, 3 }, { "AND", &and, &am_zp0, 3 }, { "ROL", &rol, &am_zp0, 5 }, { "???", &nop, &am_imp, 5 }, { "PLP", &plp, &am_imp, 4 }, { "AND", &and, &am_imm, 2 }, { "ROL", &rol, &am_acc, 2 }, { "???", &nop, &am_imp, 2 }, { "BIT", &bit, &am_abs, 4 }, { "AND", &and, &am_abs, 4 }, { "ROL", &rol, &am_abs, 6 }, { "???", &nop, &am_imp, 6 },
/* 0x30 */ { "BMI", &bmi, &am_rel, 2 }, { "AND", &and, &am_iny, 5 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 8 }, { "???", &nop, &am_imp, 4 }, { "AND", &and, &am_zpx, 4 }, { "ROL", &rol, &am_zpx, 6 }, { "???", &nop, &am_imp, 6 }, { "SEC", &sec, &am_imp, 2 }, { "AND", &and, &am_aby, 4 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 7 }, { "???", &nop, &am_imp, 4 }, { "AND", &and, &am_abx, 4 }, { "ROL", &rol, &am_abx, 7 }, { "???", &nop, &am_imp, 7 },
/* 0x40 */ { "RTI", &rti, &am_imp, 6 }, { "EOR", &eor, &am_inx, 6 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 8 }, { "???", &nop, &am_imp, 3 }, { "EOR", &eor, &am_zp0, 3 }, { "LSR", &lsr, &am_zp0, 5 }, { "???", &nop, &am_imp, 5 }, { "PHA", &pha, &am_imp, 3 }, { "EOR", &eor, &am_imm, 2 }, { "LSR", &lsr, &am_acc, 2 }, { "???", &nop, &am_imp, 2 }, { "JMP", &jmp, &am_abs, 3 }, { "EOR", &eor, &am_abs, 4 }, { "LSR", &lsr, &am_abs, 6 }, { "???", &nop, &am_imp, 6 },
/* 0x50 */ { "BVC", &bvc, &am_rel, 2 }, { "EOR", &eor, &am_iny, 5 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 8 }, { "???", &nop, &am_imp, 4 }, { "EOR", &eor, &am_zpx, 4 }, { "LSR", &lsr, &am_zpx, 6 }, { "???", &nop, &am_imp, 6 }, { "CLI", &cli, &am_imp, 2 }, { "EOR", &eor, &am_aby, 4 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 7 }, { "???", &nop, &am_imp, 4 }, { "EOR", &eor, &am_abx, 4 }, { "LSR", &lsr, &am_abx, 7 }, { "???", &nop, &am_imp, 7 },
/* 0x60 */ { "RTS", &rts, &am_imp, 6 }, { "ADC", &adc, &am_inx, 6 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 8 }, { "???", &nop, &am_imp, 3 }, { "ADC", &adc, &am_zp0, 3 }, { "ROR", &ror, &am_zp0, 5 }, { "???", &nop, &am_imp, 5 }, { "PLA", &pla, &am_imp, 4 }, { "ADC", &adc, &am_imm, 2 }, { "ROR", &ror, &am_acc, 2 }, { "???", &nop, &am_imp, 2 }, { "JMP", &jmp, &am_ind, 5 }, { "ADC", &adc, &am_abs, 4 }, { "ROR", &ror, &am_abs, 6 }, { "???", &nop, &am_imp, 6 },
/* 0x70 */ { "BVS", &bvs, &am_rel, 2 }, { "ADC", &adc, &am_iny, 5 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 8 }, { "???", &nop, &am_imp, 4 }, { "ADC", &adc, &am_zpx, 4 }, { "ROR", &ror, &am_zpx, 6 }, { "???", &nop, &am_imp, 6 }, { "SEI", &sei, &am_imp, 2 }, { "ADC", &adc, &am_aby, 4 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 7 }, { "???", &nop, &am_imp, 4 }, { "ADC", &adc, &am_abx, 4 }, { "ROR", &ror, &am_abx, 7 }, { "???", &nop, &am_imp, 7 },
/* 0x80 */ { "???", &nop, &am_imp, 2 }, { "STA", &sta, &am_inx, 6 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 6 }, { "STY", &sty, &am_zp0, 3 }, { "STA", &sta, &am_zp0, 3 }, { "STX", &stx, &am_zp0, 3 }, { "???", &nop, &am_imp, 3 }, { "DEY", &dey, &am_imp, 2 }, { "???", &nop, &am_imp, 2 }, { "TXA", &txa, &am_imp, 2 }, { "???", &nop, &am_imp, 2 }, { "STY", &sty, &am_abs, 4 }, { "STA", &sta, &am_abs, 4 }, { "STX", &stx, &am_abs, 4 }, { "???", &nop, &am_imp, 4 },
/* 0x90 */ { "BCC", &bcc, &am_rel, 2 }, { "STA", &sta, &am_iny, 6 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 6 }, { "STY", &sty, &am_zpx, 4 }, { "STA", &sta, &am_zpx, 4 }, { "STX", &stx, &am_zpy, 4 }, { "???", &nop, &am_imp, 4 }, { "TYA", &tya, &am_imp, 2 }, { "STA", &sta, &am_aby, 5 }, { "TXS", &txs, &am_imp, 2 }, { "???", &nop, &am_imp, 5 }, { "???", &nop, &am_imp, 5 }, { "STA", &sta, &am_abx, 5 }, { "???", &nop, &am_imp, 5 }, { "???", &nop, &am_imp, 5 },
/* 0xA0 */ { "LDY", &ldy, &am_imm, 2 }, { "LDA", &lda, &am_inx, 6 }, { "LDX", &ldx, &am_imm, 2 }, { "???", &nop, &am_imp, 6 }, { "LDY", &ldy, &am_zp0, 3 }, { "LDA", &lda, &am_zp0, 3 }, { "LDX", &ldx, &am_zp0, 3 }, { "???", &nop, &am_imp, 3 }, { "TAY", &tay, &am_imp, 2 }, { "LDA", &lda, &am_imm, 2 }, { "TAX", &tax, &am_imp, 2 }, { "???", &nop, &am_imp, 2 }, { "LDY", &ldy, &am_abs, 4 }, { "LDA", &lda, &am_abs, 4 }, { "LDX", &ldx, &am_abs, 4 }, { "???", &nop, &am_imp, 4 },
/* 0xB0 */ { "BCS", &bcs, &am_rel, 2 }, { "LDA", &lda, &am_iny, 5 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 5 }, { "LDY", &ldy, &am_zpx, 4 }, { "LDA", &lda, &am_zpx, 4 }, { "LDX", &ldx, &am_zpy, 4 }, { "???", &nop, &am_imp, 4 }, { "CLV", &clv, &am_imp, 2 }, { "LDA", &lda, &am_aby, 4 }, { "TSX", &tsx, &am_imp, 2 }, { "???", &nop, &am_imp, 4 }, { "LDY", &ldy, &am_abx, 4 }, { "LDA", &lda, &am_abx, 4 }, { "LDX", &ldx, &am_aby, 4 }, { "???", &nop, &am_imp, 4 },
/* 0xC0 */ { "CPY", &cpy, &am_imm, 2 }, { "CMP", &cmp, &am_inx, 6 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 8 }, { "CPY", &cpy, &am_zp0, 3 }, { "CMP", &cmp, &am_zp0, 3 }, { "DEC", &dec, &am_zp0, 5 }, { "???", &nop, &am_imp, 5 }, { "INY", &iny, &am_imp, 2 }, { "CMP", &cmp, &am_imm, 2 }, { "DEX", &dex, &am_imp, 2 }, { "???", &nop, &am_imp, 2 }, { "CPY", &cpy, &am_abs, 4 }, { "CMP", &cmp, &am_abs, 4 }, { "DEC", &dec, &am_abs, 6 }, { "???", &nop, &am_imp, 6 },
/* 0xD0 */ { "BNE", &bne, &am_rel, 2 }, { "CMP", &cmp, &am_iny, 5 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 8 }, { "???", &nop, &am_imp, 4 }, { "CMP", &cmp, &am_zpx, 4 }, { "DEC", &dec, &am_zpx, 6 }, { "???", &nop, &am_imp, 6 }, { "CLD", &cld, &am_imp, 2 }, { "CMP", &cmp, &am_aby, 4 }, { "NOP", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 7 }, { "???", &nop, &am_imp, 4 }, { "CMP", &cmp, &am_abx, 4 }, { "DEC", &dec, &am_abx, 7 }, { "???", &nop, &am_imp, 7 },
/* 0xE0 */ { "CPX", &cpx, &am_imm, 2 }, { "SBC", &sbc, &am_inx, 6 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 8 }, { "CPX", &cpx, &am_zp0, 3 }, { "SBC", &sbc, &am_zp0, 3 }, { "INC", &inc, &am_zp0, 5 }, { "???", &nop, &am_imp, 5 }, { "INX", &inx, &am_imp, 2 }, { "SBC", &sbc, &am_imm, 2 }, { "NOP", &nop, &am_imp, 2 }, { "???", &sbc, &am_imp, 2 }, { "CPX", &cpx, &am_abs, 4 }, { "SBC", &sbc, &am_abs, 4 }, { "INC", &inc, &am_abs, 6 }, { "???", &nop, &am_imp, 6 },
/* 0xF0 */ { "BEQ", &beq, &am_rel, 2 }, { "SBC", &sbc, &am_iny, 5 }, { "???", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 8 }, { "???", &nop, &am_imp, 4 }, { "SBC", &sbc, &am_zpx, 4 }, { "INC", &inc, &am_zpx, 6 }, { "???", &nop, &am_imp, 6 }, { "SED", &sed, &am_imp, 2 }, { "SBC", &sbc, &am_aby, 4 }, { "NOP", &nop, &am_imp, 2 }, { "???", &nop, &am_imp, 7 }, { "???", &nop, &am_imp, 4 }, { "SBC", &sbc, &am_abx, 4 }, { "INC", &inc, &am_abx, 7 }, { "???", &nop, &am_imp, 7 }
};

uint8_t instruction_data(struct cpu_6502 *cpu)
{
    return cpu->instruction.addressing_mode_fn == &am_imm || cpu->instruction.addressing_mode_fn == &am_acc
        ? cpu->instruction.operand
        : read(cpu->bus, cpu->instruction.addr);
}

uint8_t get_flag(struct cpu_6502 *cpu, enum status_flag flag)
{
    return (cpu->reg.s & flag) != 0 ? 1 : 0;
}

void set_flag(struct cpu_6502 *cpu, enum status_flag flag, bool set)
{
    if (set)
        cpu->reg.s |= flag;
    else
        cpu->reg.s &= ~flag;
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
    write(cpu->bus, cpu->reg.sp++, val);
}

void stack_addr_push(struct cpu_6502 *cpu, uint16_t ptr)
{
    stack_push(cpu, (uint8_t)(ptr & 0x00FF));
    stack_push(cpu, (uint8_t)(ptr << 8));
}

uint8_t stack_pull(struct cpu_6502 *cpu)
{
    return read(cpu->bus, cpu->reg.sp--);
}

uint16_t stack_addr_pull(struct cpu_6502 *cpu)
{
    uint16_t low = stack_pull(cpu);
    uint16_t high = stack_pull(cpu);

    return (high << 8) | low;
}

uint8_t fetch(struct cpu_6502 *cpu)
{
    return read(cpu->bus, cpu->reg.pc++);
}

bool cpu_clock(struct cpu_6502 *cpu)
{
    if (cpu->cycles_till_instruction_completion-- > 0)
        return false;

    cpu->instruction.instruction_fn(cpu);

    // Load up a new instruction
    cpu->instruction = instructions[fetch(cpu)];
    cpu->cycles_till_instruction_completion = cpu->instruction.base_cycle_count;
    cpu->instruction.addressing_mode_fn(cpu);

    //printf("a = %04x, x = %04x, y = %04x, pc = %04x, sp = %04x, s = %04x {c=%01x, z=%01x, i=%01x, d=%01x, b=%01x, u=%01x, v=%01x, n=%01x} current instruction: %s\n",
    printf("a = %04x, x = %04x, y = %04x, pc = %04x, sp = %04x, s = %04x {n=%01x, v=%01x, u=%01x, b=%01x, d=%01x, i=%01x, z=%01x, c=%01x} -- %s\n",
            cpu->reg.a, cpu->reg.x, cpu->reg.y, cpu->reg.pc, cpu->reg.sp, cpu->reg.s, cpu->reg.status_flags.n, cpu->reg.status_flags.v, cpu->reg.status_flags.u, cpu->reg.status_flags.b,
            cpu->reg.status_flags.d, cpu->reg.status_flags.i, cpu->reg.status_flags.z, cpu->reg.status_flags.c, cpu->instruction.op_code_name);
    return true;
}

void cpu_reset(struct cpu_6502 *cpu) 
{
    cpu->reg = (struct cpu_registers){ 0 };
    cpu->instruction = instructions[0xEA]; // NOP
    uint16_t low = (uint16_t)read(cpu->bus, 0xFFFC);
    uint16_t high = (uint16_t)read(cpu->bus, 0xFFFD) << 8;
    uint16_t addr = high | low;
    cpu->reg.pc = addr;
}

void irq(struct cpu_6502 *cpu)
{
    // TODO: probably stash some stuff
    cpu->reg.pc = ((uint16_t)read(cpu->bus, 0xFFFF) << 8) | (uint16_t)read(cpu->bus, 0xFFFE);
}

void nmi(struct cpu_6502 *cpu)
{
    // TODO: probably stash some stuff
    cpu->reg.pc = ((uint16_t)read(cpu->bus, 0xFFFB) << 8) | (uint16_t)read(cpu->bus, 0xFFFA);
}

void adc(struct cpu_6502 *cpu)
{
    uint16_t data = (uint16_t)instruction_data(cpu);
    uint16_t res = (uint16_t)cpu->reg.a + data + (uint16_t)get_flag(cpu, C);

    set_flag(cpu, C, res > 0x00FF);
    set_flag(cpu, Z, (res & 0x00FF) == 0x00);
    set_flag(cpu, N, (res & 0x0080) != 0x00);

    bool positive_overflow = ~(cpu->reg.a & 0x80) & ~(data & 0x80) & (res & 0x0080);
    bool negative_underflow = (cpu->reg.a & 0x80) & (data & 0x80) & ~(res & 0x0080);
    set_flag(cpu, V, positive_overflow || negative_underflow);

    cpu->reg.a = res & 0x00FF;
}

void and(struct cpu_6502 *cpu)
{
    cpu->reg.a = cpu->reg.a & instruction_data(cpu);

    set_flag(cpu, Z, (cpu->reg.a & 0x00FF) == 0x00);
    set_flag(cpu, N, (cpu->reg.a & 0x0080) != 0x00);
}

void asl(struct cpu_6502 *cpu)
{
    uint16_t res = (uint16_t)(instruction_data(cpu)) << 1;

    set_flag(cpu, C, res > 0x00FF);
    set_flag(cpu, Z, (res & 0x00FF) == 0x00);
    set_flag(cpu, N, (res & 0x0080) != 0x00);

    if (cpu->instruction.addressing_mode_fn == &am_acc)
        cpu->reg.a = (uint8_t)(res & 0x00FF);
    else
        write(cpu->bus, cpu->instruction.addr, (uint8_t)(res & 0x00FF));
}

void bcc(struct cpu_6502 *cpu)
{
    if (!get_flag(cpu, C))
        branch(cpu);
}

void bcs(struct cpu_6502 *cpu)
{
    if (get_flag(cpu, C))
        branch(cpu);
}

void beq(struct cpu_6502 *cpu)
{
    if (get_flag(cpu, Z))
        branch(cpu);
}

void bit(struct cpu_6502 *cpu)
{
    uint8_t byte = instruction_data(cpu);

    set_flag(cpu, N, byte & 0x80);
    // todo: dafuq, javidx9 did this with a !
    set_flag(cpu, Z, byte & cpu->reg.a);
    set_flag(cpu, V, byte & 0x40);
}

void bmi(struct cpu_6502 *cpu)
{
    if (get_flag(cpu, N))
        branch(cpu);
}

void bne(struct cpu_6502 *cpu)
{
    if (!get_flag(cpu, Z))
        branch(cpu);
}

void bpl(struct cpu_6502 *cpu)
{
    if (!get_flag(cpu, N))
        branch(cpu);
}

void brk(struct cpu_6502 *cpu)
{
    set_flag(cpu, I, true);
    stack_addr_push(cpu, cpu->reg.pc - 1);
    stack_push(cpu, cpu->reg.sp);
}

void bvc(struct cpu_6502 *cpu)
{
    if (!get_flag(cpu, V))
        branch(cpu);
}

void bvs(struct cpu_6502 *cpu)
{
    if (get_flag(cpu, V))
        branch(cpu);
}

void clc(struct cpu_6502 *cpu)
{
    set_flag(cpu, C, 0);
}

void cld(struct cpu_6502 *cpu)
{
    set_flag(cpu, D, 0);
}

void cli(struct cpu_6502 *cpu)
{
    set_flag(cpu, I, 0);
}

void clv(struct cpu_6502 *cpu)
{
    set_flag(cpu, V, 0);
}

void cmp(struct cpu_6502 *cpu)
{
    uint16_t byte = (uint16_t)cpu->reg.a - (uint16_t)instruction_data(cpu);

    set_flag(cpu, N, (byte & 0x80) != 0);
    set_flag(cpu, Z, byte == 0);
    set_flag(cpu, C, (byte & 0xFF00) != 0);
}

void cpx(struct cpu_6502 *cpu)
{
    uint16_t byte = (uint16_t)cpu->reg.x - (uint16_t)instruction_data(cpu);

    set_flag(cpu, N, (byte & 0x80) != 0);
    set_flag(cpu, Z, byte == 0);
    set_flag(cpu, C, (byte & 0xFF00) != 0);
}

void cpy(struct cpu_6502 *cpu)
{
    uint16_t byte = (uint16_t)cpu->reg.y - (uint16_t)instruction_data(cpu);

    set_flag(cpu, N, (byte & 0x80) != 0);
    set_flag(cpu, Z, byte == 0);
    set_flag(cpu, C, (byte & 0xFF00) != 0);
}

void dec(struct cpu_6502 *cpu)
{
    uint8_t res = instruction_data(cpu) - 1;

    set_flag(cpu, N, (res & 0x80) != 0);
    set_flag(cpu, Z, res == 0);

    write(cpu->bus, cpu->instruction.addr, res);
}

void dex(struct cpu_6502 *cpu)
{
    cpu->reg.x--;

    set_flag(cpu, N, (cpu->reg.x & 0x80) != 0);
    set_flag(cpu, Z, cpu->reg.x == 0);
}

void dey(struct cpu_6502 *cpu)
{
    cpu->reg.y--;

    set_flag(cpu, N, (cpu->reg.y & 0x80) != 0);
    set_flag(cpu, Z, cpu->reg.y == 0);
}

void eor(struct cpu_6502 *cpu)
{
    cpu->reg.a ^= instruction_data(cpu);

    set_flag(cpu, N, (cpu->reg.a & 0x80) != 0);
    set_flag(cpu, Z, cpu->reg.a == 0);
}

void inc(struct cpu_6502 *cpu)
{
    uint8_t res = instruction_data(cpu) + 1;

    set_flag(cpu, N, (res & 0x80) != 0);
    set_flag(cpu, Z, res == 0);

    write(cpu->bus, cpu->instruction.addr, res);
}

void inx(struct cpu_6502 *cpu)
{
    cpu->reg.x++;

    set_flag(cpu, N, (cpu->reg.x & 0x80) != 0);
    set_flag(cpu, Z, cpu->reg.x == 0);
}

void iny(struct cpu_6502 *cpu)
{
    cpu->reg.y++;

    set_flag(cpu, N, (cpu->reg.y & 0x80) != 0);
    set_flag(cpu, Z, cpu->reg.y == 0);
}

void jmp(struct cpu_6502 *cpu)
{
    cpu->reg.pc = cpu->instruction.addr;
}

void jsr(struct cpu_6502 *cpu)
{
    stack_push(cpu, cpu->reg.pc - 1);
    cpu->reg.pc = cpu->instruction.addr;
}

void lda(struct cpu_6502 *cpu)
{
    cpu->reg.a = instruction_data(cpu);

    set_flag(cpu, N, (cpu->reg.a & 0x80) != 0);
    set_flag(cpu, Z, cpu->reg.a == 0);
}

void ldx(struct cpu_6502 *cpu)
{
    cpu->reg.x = instruction_data(cpu);

    set_flag(cpu, N, (cpu->reg.x & 0x80) != 0);
    set_flag(cpu, Z, cpu->reg.x == 0);
}

void ldy(struct cpu_6502 *cpu)
{
    cpu->reg.y = instruction_data(cpu);

    set_flag(cpu, N, (cpu->reg.y & 0x80) != 0);
    set_flag(cpu, Z, cpu->reg.y == 0);
}

void lsr(struct cpu_6502 *cpu)
{
    uint8_t res = instruction_data(cpu);

    set_flag(cpu, C, (res & 0x01) != 0);

    res >>= 1;

    set_flag(cpu, Z, res == 0x00);
    set_flag(cpu, N, false);

    if (cpu->instruction.addressing_mode_fn == &am_acc)
        cpu->reg.a = res;
    else
        write(cpu->bus, cpu->instruction.addr, res);
}

void nop(struct cpu_6502 *cpu)
{
}

void ora(struct cpu_6502 *cpu)
{
    cpu->reg.a |= instruction_data(cpu);

    set_flag(cpu, N, (cpu->reg.a & 0x80) != 0);
    set_flag(cpu, Z, cpu->reg.a == 0);
}

void pha(struct cpu_6502 *cpu)
{
    stack_push(cpu, cpu->reg.a);
}

void php(struct cpu_6502 *cpu)
{
    stack_push(cpu, cpu->reg.s);
}

void pla(struct cpu_6502 *cpu)
{
    cpu->reg.a = stack_pull(cpu);
}

void plp(struct cpu_6502 *cpu)
{
    cpu->reg.s = stack_pull(cpu);
}

void rol(struct cpu_6502 *cpu)
{
    uint16_t res = (uint16_t)(instruction_data(cpu)) << 1;
    res |= (uint16_t)get_flag(cpu, C);

    set_flag(cpu, C, res > 0x00FF);

    set_flag(cpu, Z, (res & 0x00FF) == 0x00);
    set_flag(cpu, N, (res & 0x0080) != 0);

    if (cpu->instruction.addressing_mode_fn == &am_acc)
        cpu->reg.a = res & 0x00FF;
    else
        write(cpu->bus, cpu->instruction.addr, res & 0x00FF);
}

void ror(struct cpu_6502 *cpu)
{
    uint8_t res = instruction_data(cpu);

    set_flag(cpu, C, (res & 0x01) != 0);

    res >>= 1;
    res |= get_flag(cpu, C) << 7;

    set_flag(cpu, Z, res == 0x00);
    set_flag(cpu, N, (res & 0x80) != 0);

    if (cpu->instruction.addressing_mode_fn == &am_acc)
        cpu->reg.a = res;
    else
        write(cpu->bus, cpu->instruction.addr, res);

}

void rti(struct cpu_6502 *cpu)
{
    cpu->reg.s = stack_pull(cpu);
    cpu->reg.pc = stack_addr_pull(cpu);
}

void rts(struct cpu_6502 *cpu)
{
    cpu->reg.pc = stack_addr_pull(cpu);
}

void sbc(struct cpu_6502 *cpu)
{

}

void sec(struct cpu_6502 *cpu)
{
    set_flag(cpu, C, true);
}

void sed(struct cpu_6502 *cpu)
{
    set_flag(cpu, D, true);
}

void sei(struct cpu_6502 *cpu)
{
    set_flag(cpu, I, true);
}

void sta(struct cpu_6502 *cpu)
{
    write(cpu->bus, cpu->instruction.addr, cpu->reg.a);
}

void stx(struct cpu_6502 *cpu)
{
    write(cpu->bus, cpu->instruction.addr, cpu->reg.x);
}

void sty(struct cpu_6502 *cpu)
{
    write(cpu->bus, cpu->instruction.addr, cpu->reg.y);
}

void tax(struct cpu_6502 *cpu)
{
    cpu->reg.x = cpu->reg.a;

    set_flag(cpu, Z, cpu->reg.x == 0x00);
    set_flag(cpu, N, (cpu->reg.x & 0x80) != 0);
}

void tay(struct cpu_6502 *cpu)
{
    cpu->reg.y = cpu->reg.a;

    set_flag(cpu, Z, cpu->reg.y == 0x00);
    set_flag(cpu, N, (cpu->reg.y & 0x80) != 0);
}

void tsx(struct cpu_6502 *cpu)
{
    cpu->reg.x = cpu->reg.sp;

    set_flag(cpu, Z, cpu->reg.x == 0x00);
    set_flag(cpu, N, (cpu->reg.x & 0x80) != 0);
}

void txa(struct cpu_6502 *cpu)
{
    cpu->reg.a = cpu->reg.x;

    set_flag(cpu, Z, cpu->reg.a == 0x00);
    set_flag(cpu, N, (cpu->reg.a & 0x80) != 0);
}

void txs(struct cpu_6502 *cpu)
{
    cpu->reg.sp = cpu->reg.x;
}

void tya(struct cpu_6502 *cpu)
{
    cpu->reg.a = cpu->reg.y;

    set_flag(cpu, Z, cpu->reg.a == 0x00);
    set_flag(cpu, N, (cpu->reg.a & 0x80) != 0);
}

// Addressing modes
void am_acc(struct cpu_6502 *cpu)
{
    cpu->instruction.operand = cpu->reg.a;   
}

void am_abs(struct cpu_6502 *cpu)
{
    uint8_t low = fetch(cpu);
    uint8_t high = fetch(cpu);

    cpu->instruction.addr = ((uint16_t)high << 8) | (uint16_t)low;
}

void am_abx(struct cpu_6502 *cpu)
{
    uint8_t low = fetch(cpu);
    uint8_t high = fetch(cpu);

    cpu->instruction.addr = ((uint16_t)high << 8) | (uint16_t)low + (uint16_t)cpu->reg.x;

    // Crossed page boundary, +1 cycle penalty
    if ((cpu->instruction.addr & 0xFF00) != ((uint16_t)high << 8))
        cpu->cycles_till_instruction_completion++;
}

void am_aby(struct cpu_6502 *cpu)
{
    uint8_t low = fetch(cpu);
    uint8_t high = fetch(cpu);

    cpu->instruction.addr = ((uint16_t)high << 8) | (uint16_t)low + (uint16_t)cpu->reg.y;

    // Crossed page boundary, +1 cycle penalty
    if ((cpu->instruction.addr & 0xFF00) != ((uint16_t)high << 8))
        cpu->cycles_till_instruction_completion++;
}

void am_imm(struct cpu_6502 *cpu)
{
    cpu->instruction.operand = fetch(cpu);
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
    cpu->instruction.addr = ((uint16_t)read(cpu->bus, ptr + 1) << 8) | (uint16_t)read(cpu->bus, ptr);

    // According to javidx9 there's a bug:
    /*
    if (low == 0xFF)
        cpu->instruction.addr = ((uint16_t)read(cpu->bus, ptr & 0xFF00) << 8) | (uint16_t)read(cpu->bus, ptr);
    */
    
}

// Aka indexed indirect
void am_inx(struct cpu_6502 *cpu)
{
    uint16_t ptr = (uint16_t)fetch(cpu) + (uint16_t)cpu->reg.x;

    uint8_t low = read(cpu->bus, ptr);
    uint8_t high = read(cpu->bus, ptr + 1);
    cpu->instruction.addr = ((uint16_t)high << 8) | (uint16_t)low;
}

// aka indirect indexed
void am_iny(struct cpu_6502 *cpu)
{
    uint16_t ptr = (uint16_t)fetch(cpu);

    uint8_t low = read(cpu->bus, ptr);
    uint8_t high = read(cpu->bus, ptr + 1);

    cpu->instruction.addr = (((uint16_t)high << 8) | (uint16_t)low) + (uint16_t)cpu->reg.y;

    // Crossed page boundary, +1 cycle penalty
    if ((cpu->instruction.addr & 0xFF00) != ((uint16_t)high << 8))
        cpu->cycles_till_instruction_completion++;
}

void am_rel(struct cpu_6502 *cpu)
{
    uint16_t initial_pc = cpu->reg.pc;
    cpu->instruction.operand = fetch(cpu);
    cpu->instruction.addr = (int16_t)initial_pc + (int16_t)(int8_t)cpu->instruction.operand;

    // Crossed page boundary, +1 cycle penalty
    if ((cpu->instruction.addr & 0xFF00) != (initial_pc << 8))
        cpu->cycles_till_instruction_completion++;
}

void am_zp0(struct cpu_6502 *cpu)
{
    cpu->instruction.addr = (uint16_t)fetch(cpu) & 0x00FF;
}

void am_zpx(struct cpu_6502 *cpu)
{
    cpu->instruction.addr = (uint16_t)fetch(cpu) + cpu->reg.x;
    // To remove carry
    cpu->instruction.addr &= 0x00FF;
}

void am_zpy(struct cpu_6502 *cpu)
{
    cpu->instruction.addr = (uint16_t)fetch(cpu) + cpu->reg.y;
    // To remove carry
    cpu->instruction.addr &= 0x00FF;
}
