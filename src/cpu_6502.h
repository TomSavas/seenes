#ifndef CPU_6502_H
#define CPU_6502_H

#include <stdint.h>
#include <stdbool.h>

#include "seenes.h"

struct cpu_6502;
struct instruction
{
    const char *op_code_name;
    void (*instruction_fn)(struct cpu_6502 *);

    const char *addr_mode_name;
    void (*addressing_mode_fn)(struct cpu_6502 *);

    uint8_t base_cycle_count;

    uint8_t operand0;
    uint8_t operand1;
    uint16_t addr;
};

enum cpu_status_flag
{
    CPU_C = 0x01,
    CPU_Z = 0x02,
    CPU_I = 0x04,
    CPU_D = 0x08,
    CPU_B = 0x10, // unused in nes
    CPU_U = 0x20, // unused in 6502
    CPU_V = 0x40,
    CPU_N = 0x80
};

#pragma pack(1)
union cpu_status_reg
{
    uint8_t reg;
    struct
    {
        uint8_t c: 1;
        uint8_t z: 1;
        uint8_t i: 1;
        uint8_t d: 1;
        uint8_t b: 1; // unused in nes
        uint8_t u: 1; // unused in 6502
        uint8_t v: 1;
        uint8_t n: 1;
    };
};
SEENES_STATIC_ASSERT(sizeof(union cpu_status_reg) == 1, cpu_status_reg_is_1_byte);
#pragma pack(0)

struct cpu_registers
{
    uint8_t a;
    uint8_t x;
    uint8_t y;
    uint16_t pc;
    uint8_t sp;
    union cpu_status_reg s;
};

struct bus;
struct cpu_6502
{
    struct cpu_registers reg;
    struct bus *bus;
    
    uint8_t raw_instruction;
    struct instruction instruction;
    uint8_t cycles_till_instruction_completion;

    uint64_t cycles;

    bool disable_instruction_execution;
};

uint8_t get_flag(struct cpu_6502 *cpu, enum cpu_status_flag flag);
void set_flag(struct cpu_6502 *cpu, enum cpu_status_flag flag, bool set);

bool cpu_clock(struct cpu_6502 *cpu);
void cpu_reset(struct cpu_6502 *cpu);

void nmi(struct cpu_6502 *cpu);

// instructions
void adc(struct cpu_6502 *cpu);
void and(struct cpu_6502 *cpu);
void asl(struct cpu_6502 *cpu);
void bcc(struct cpu_6502 *cpu);
void bcs(struct cpu_6502 *cpu);
void beq(struct cpu_6502 *cpu);
void bit(struct cpu_6502 *cpu);
void bmi(struct cpu_6502 *cpu);
void bne(struct cpu_6502 *cpu);
void bpl(struct cpu_6502 *cpu);
void brk(struct cpu_6502 *cpu);
void bvc(struct cpu_6502 *cpu);
void bvs(struct cpu_6502 *cpu);
void clc(struct cpu_6502 *cpu);
void cld(struct cpu_6502 *cpu);
void cli(struct cpu_6502 *cpu);
void clv(struct cpu_6502 *cpu);
void cmp(struct cpu_6502 *cpu);
void cpx(struct cpu_6502 *cpu);
void cpy(struct cpu_6502 *cpu);
void dec(struct cpu_6502 *cpu);
void dex(struct cpu_6502 *cpu);
void dey(struct cpu_6502 *cpu);
void eor(struct cpu_6502 *cpu);
void inc(struct cpu_6502 *cpu);
void inx(struct cpu_6502 *cpu);
void iny(struct cpu_6502 *cpu);
void jmp(struct cpu_6502 *cpu);
void jsr(struct cpu_6502 *cpu);
void lda(struct cpu_6502 *cpu);
void ldx(struct cpu_6502 *cpu);
void ldy(struct cpu_6502 *cpu);
void lsr(struct cpu_6502 *cpu);
void nop(struct cpu_6502 *cpu);
void ora(struct cpu_6502 *cpu);
void pha(struct cpu_6502 *cpu);
void php(struct cpu_6502 *cpu);
void pla(struct cpu_6502 *cpu);
void plp(struct cpu_6502 *cpu);
void rol(struct cpu_6502 *cpu);
void ror(struct cpu_6502 *cpu);
void rti(struct cpu_6502 *cpu);
void rts(struct cpu_6502 *cpu);
void sbc(struct cpu_6502 *cpu);
void sec(struct cpu_6502 *cpu);
void sed(struct cpu_6502 *cpu);
void sei(struct cpu_6502 *cpu);
void sta(struct cpu_6502 *cpu);
void stx(struct cpu_6502 *cpu);
void sty(struct cpu_6502 *cpu);
void tax(struct cpu_6502 *cpu);
void tay(struct cpu_6502 *cpu);
void tsx(struct cpu_6502 *cpu);
void txa(struct cpu_6502 *cpu);
void txs(struct cpu_6502 *cpu);
void tya(struct cpu_6502 *cpu);

// addressing modes
void am_acc(struct cpu_6502 *cpu);
void am_abs(struct cpu_6502 *cpu);
void am_abx(struct cpu_6502 *cpu);
void am_aby(struct cpu_6502 *cpu);
void am_imm(struct cpu_6502 *cpu);
void am_imp(struct cpu_6502 *cpu);
void am_ind(struct cpu_6502 *cpu);
void am_inx(struct cpu_6502 *cpu);
void am_iny(struct cpu_6502 *cpu);
void am_rel(struct cpu_6502 *cpu);
void am_zp0(struct cpu_6502 *cpu);
void am_zpx(struct cpu_6502 *cpu);
void am_zpy(struct cpu_6502 *cpu);

#endif
