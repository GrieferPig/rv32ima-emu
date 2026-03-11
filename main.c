#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <unistd.h>

#define CSR_MTVEC 0x305
#define CSR_MEPC 0x341
#define CSR_MCAUSE 0x342
#define CSR_MSTATUS 0x300
#define CSR_MIE 0x304 // 咩
#define CSR_MIP 0x344

#define CSR_MISA 0x301
#define CSR_MHARTID 0xF14

#define CSR_MSTATUS_MIE (1 << 3)
#define CSR_MSTATUS_MPIE (1 << 7)
#define CSR_MSTATUS_MPP_MASK (3 << 11)
#define CSR_MSTATUS_MPP_M (3 << 11)
#define CSR_MIE_MEIE (1 << 11)
#define CSR_MIE_MTIE (1 << 7)
#define CSR_MIE_MSIE (1 << 3)
#define CSR_MIP_MEIP (1 << 11)
#define CSR_MIP_MTIP (1 << 7)
#define CSR_MIP_MSIP (1 << 3)

// misa: RV32 (MXL=1) + I + M + A
#define MISA_VALUE ((1u << 30) | (1 << 12) | (1 << 8) | (1 << 0))

#define RAM_BASE 0x80000000u
#define RAM_SIZE (64u * 1024u * 1024u)
#define FDT_OFFSET (8u * 1024u * 1024u)
#define FDT_ADDR (RAM_BASE + FDT_OFFSET)
#define UART_BASE 0x10000000u
#define UART_SIZE 0x100u

#define PLIC_BASE 0x0c000000u
#define PLIC_SIZE 0x4000000u
#define PLIC_PENDING_OFFSET 0x1000u
#define PLIC_ENABLE_OFFSET 0x2000u
#define PLIC_CONTEXT_OFFSET 0x200000u
#define PLIC_CONTEXT_STRIDE 0x1000u
#define PLIC_THRESHOLD_OFFSET 0x0u
#define PLIC_CLAIM_OFFSET 0x4u
#define PLIC_UART_SOURCE 1u

#define CLINT_BASE 0x02000000u
#define CLINT_SIZE 0x10000u
#define CLINT_MTIMECMP (CLINT_BASE + 0x4000)
#define CLINT_MTIME (CLINT_BASE + 0xbff8)

#define UART_REG_RBR 0x0
#define UART_REG_THR 0x0
#define UART_REG_DLL 0x0
#define UART_REG_IER 0x1
#define UART_REG_DLM 0x1
#define UART_REG_IIR 0x2
#define UART_REG_FCR 0x2
#define UART_REG_LCR 0x3
#define UART_REG_MCR 0x4
#define UART_REG_LSR 0x5
#define UART_REG_MSR 0x6
#define UART_REG_SCR 0x7

#define UART_IER_RDI (1 << 0)
#define UART_LCR_DLAB (1 << 7)
#define UART_LSR_DR (1 << 0)
#define UART_LSR_THRE (1 << 5)
#define UART_LSR_TEMT (1 << 6)
#define UART_FCR_CLEAR_RCVR (1 << 1)
#define UART_IIR_NO_INT 0x01
#define UART_IIR_RDI 0x04

typedef enum
{
    OPCODE_RTYPE,
    OPCODE_ITYPE,
    OPCODE_BTYPE,
    OPCODE_STYPE,
    OPCODE_JTYPE,
    OPCODE_UTYPE,
    OPCODE_UNKNOWN
} OpcodeType;

typedef struct
{
    uint32_t regs[32];
    uint32_t pc;
    uint32_t csr[4096];
    uint64_t atomic_resv_addr; // all 1s when not used
} CpuState;

typedef struct
{
    uint64_t mtime;
    uint64_t mtimecmp;
    uint32_t msip;
} CLINTState;

typedef struct
{
    uint8_t ier;
    uint8_t fcr;
    uint8_t lcr;
    uint8_t mcr;
    uint8_t lsr;
    uint8_t msr;
    uint8_t scr;
    uint8_t dll;
    uint8_t dlm;
    uint8_t rx_data;
    uint8_t rx_ready;
} UARTState;

typedef struct
{
    uint32_t priority[2];
    uint32_t pending_bits;
    uint32_t source_level_bits;
    uint32_t enable_bits;
    uint32_t threshold;
    uint32_t claimed_source;
    uint32_t claim_read_latch;
    uint32_t claim_write_latch;
    uint8_t claim_read_active;
    uint8_t claim_write_mask;
} PLICState;

typedef uint8_t (*MemoryRead8Fn)(uint32_t addr);
typedef void (*MemoryWrite8Fn)(uint32_t addr, uint8_t value);

typedef struct
{
    MemoryRead8Fn read8;
    MemoryWrite8Fn write8;
} MemoryDispatch;

CpuState cpu = {0};
CLINTState clint = {0};
UARTState uart = {.lsr = UART_LSR_THRE | UART_LSR_TEMT};
PLICState plic = {.priority = {0, 1}};

static struct termios stdin_termios;
static int stdin_is_tty = 0;
static int stdin_flags = -1;

static void uart_restore_stdio(void);
static void uart_update_irq(void);

static void uart_signal_handler(int signo)
{
    uart_restore_stdio();
    signal(signo, SIG_DFL);
    raise(signo);
}

static void plic_update_mip(void)
{
    uint32_t deliverable = plic.pending_bits & plic.enable_bits;

    if ((deliverable & (1u << PLIC_UART_SOURCE)) && plic.priority[PLIC_UART_SOURCE] > plic.threshold)
        cpu.csr[CSR_MIP] |= CSR_MIP_MEIP;
    else
        cpu.csr[CSR_MIP] &= ~CSR_MIP_MEIP;
}

static void plic_update_source_level(uint32_t source, int asserted)
{
    uint32_t bit = 1u << source;

    if (asserted)
    {
        plic.source_level_bits |= bit;
        if (plic.claimed_source != source)
            plic.pending_bits |= bit;
    }
    else
    {
        plic.source_level_bits &= ~bit;
    }

    plic_update_mip();
}

static uint32_t plic_claim(void)
{
    uint32_t source = 0;

    if ((plic.pending_bits & plic.enable_bits & (1u << PLIC_UART_SOURCE)) &&
        plic.priority[PLIC_UART_SOURCE] > plic.threshold)
    {
        source = PLIC_UART_SOURCE;
        plic.pending_bits &= ~(1u << source);
        plic.claimed_source = source;
    }

    plic_update_mip();
    return source;
}

static void plic_complete(uint32_t source)
{
    if (plic.claimed_source == source)
    {
        plic.claimed_source = 0;
        if (plic.source_level_bits & (1u << source))
            plic.pending_bits |= 1u << source;
    }

    plic_update_mip();
}

static uint32_t plic_read32(uint32_t addr)
{
    uint32_t offset = addr - PLIC_BASE;

    if (offset == 4u * PLIC_UART_SOURCE)
        return plic.priority[PLIC_UART_SOURCE];

    if (offset == PLIC_PENDING_OFFSET)
        return plic.pending_bits;

    if (offset == PLIC_ENABLE_OFFSET)
        return plic.enable_bits;

    if (offset == PLIC_CONTEXT_OFFSET + PLIC_THRESHOLD_OFFSET)
        return plic.threshold;

    if (offset == PLIC_CONTEXT_OFFSET + PLIC_CLAIM_OFFSET)
        return plic_claim();

    return 0;
}

static void plic_write32(uint32_t addr, uint32_t value)
{
    uint32_t offset = addr - PLIC_BASE;

    if (offset == 4u * PLIC_UART_SOURCE)
    {
        plic.priority[PLIC_UART_SOURCE] = value & 0x7;
    }
    else if (offset == PLIC_ENABLE_OFFSET)
    {
        plic.enable_bits = value;
    }
    else if (offset == PLIC_CONTEXT_OFFSET + PLIC_THRESHOLD_OFFSET)
    {
        plic.threshold = value;
    }
    else if (offset == PLIC_CONTEXT_OFFSET + PLIC_CLAIM_OFFSET)
    {
        plic_complete(value);
        return;
    }

    plic_update_mip();
}

static void uart_update_irq(void)
{
    int asserted = uart.rx_ready && (uart.ier & UART_IER_RDI);

    plic_update_source_level(PLIC_UART_SOURCE, asserted);
}

static void uart_restore_stdio(void)
{
    if (stdin_flags != -1)
    {
        fcntl(STDIN_FILENO, F_SETFL, stdin_flags);
        stdin_flags = -1;
    }

    if (stdin_is_tty)
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &stdin_termios);
        stdin_is_tty = 0;
    }
}

static int uart_init_stdio(void)
{
    stdin_is_tty = isatty(STDIN_FILENO);

    if (stdin_is_tty)
    {
        struct termios raw_termios;

        if (tcgetattr(STDIN_FILENO, &stdin_termios) != 0)
        {
            perror("failed to read terminal settings");
            return -1;
        }

        raw_termios = stdin_termios;
        raw_termios.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
        raw_termios.c_oflag &= ~(OPOST);
        raw_termios.c_cflag |= CS8;
        raw_termios.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
        raw_termios.c_cc[VMIN] = 0;
        raw_termios.c_cc[VTIME] = 0;

        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw_termios) != 0)
        {
            perror("failed to configure terminal");
            return -1;
        }
    }

    stdin_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (stdin_flags == -1)
    {
        perror("failed to get stdin flags");
        uart_restore_stdio();
        return -1;
    }

    if (fcntl(STDIN_FILENO, F_SETFL, stdin_flags | O_NONBLOCK) != 0)
    {
        perror("failed to set stdin nonblocking");
        uart_restore_stdio();
        return -1;
    }

    if (atexit(uart_restore_stdio) != 0)
    {
        fprintf(stderr, "failed to register stdio cleanup\n");
        uart_restore_stdio();
        return -1;
    }

    if (signal(SIGINT, uart_signal_handler) == SIG_ERR ||
        signal(SIGTERM, uart_signal_handler) == SIG_ERR ||
        signal(SIGHUP, uart_signal_handler) == SIG_ERR)
    {
        perror("failed to register signal handlers");
        uart_restore_stdio();
        return -1;
    }

    return 0;
}

static void uart_poll_input(void)
{
    if (uart.rx_ready)
    {
        uart_update_irq();
        return;
    }

    for (;;)
    {
        unsigned char byte;
        ssize_t nread = read(STDIN_FILENO, &byte, 1);

        if (nread == 1)
        {
            uart.rx_data = byte;
            uart.rx_ready = 1;
            uart.lsr |= UART_LSR_DR;
            uart_update_irq();
            return;
        }

        if (nread == 0)
        {
            uart_update_irq();
            return;
        }

        if (errno == EINTR)
        {
            continue;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            uart_update_irq();
            return;
        }

        perror("failed to read stdin");
        exit(1);
    }
}

OpcodeType opcode_get_format(uint32_t opcode)
{
    switch (opcode)
    {
    case 0x33:
    case 0x2f: // atomic
        return OPCODE_RTYPE;
    case 0x13:
    case 0x03:
    case 0x67:
    case 0x73:
    case 0x0f: // fence.i
        return OPCODE_ITYPE;
    case 0x63:
        return OPCODE_BTYPE;
    case 0x23:
        return OPCODE_STYPE;
    case 0x6f:
        return OPCODE_JTYPE;
    case 0x37:
    case 0x17:
        return OPCODE_UTYPE;
    }
    return OPCODE_UNKNOWN;
}

uint8_t *memory;

static inline uint32_t memory_offset(uint32_t addr)
{
    if (addr < RAM_BASE || addr >= RAM_BASE + RAM_SIZE)
    {
        fprintf(stderr, "invalid memory access at 0x%08x (PC=0x%08x)\n", addr, cpu.pc);
        exit(1);
    }

    return addr - RAM_BASE;
}

static inline uint8_t ram_read8(uint32_t addr)
{
    return memory[memory_offset(addr)];
}

static inline void ram_write8(uint32_t addr, uint8_t value)
{
    memory[memory_offset(addr)] = value;
}

static inline uint8_t uart_read8(uint32_t addr)
{
    uint32_t offset = (addr - UART_BASE) & 0x7;

    switch (offset)
    {
    case UART_REG_RBR:
        if (uart.lcr & UART_LCR_DLAB)
        {
            return uart.dll;
        }
        uart_poll_input();
        if (!uart.rx_ready)
        {
            return 0;
        }

        uart.rx_ready = 0;
        uart.lsr &= (uint8_t)~UART_LSR_DR;
        uart_update_irq();
        return uart.rx_data;
    case UART_REG_IER:
        if (uart.lcr & UART_LCR_DLAB)
        {
            return uart.dlm;
        }
        return uart.ier;
    case UART_REG_IIR:
    {
        uint8_t fifo_bits = (uart.fcr & 0x1) ? 0xc0 : 0x00;

        if (uart.rx_ready && (uart.ier & UART_IER_RDI))
            return fifo_bits | UART_IIR_RDI;

        return fifo_bits | UART_IIR_NO_INT;
    }
    case UART_REG_LCR:
        return uart.lcr;
    case UART_REG_MCR:
        return uart.mcr;
    case UART_REG_LSR:
        uart_poll_input();
        return uart.lsr;
    case UART_REG_MSR:
        return uart.msr;
    case UART_REG_SCR:
        return uart.scr;
    default:
        return 0;
    }
}

static inline void uart_write8(uint32_t addr, uint8_t value)
{
    uint32_t offset = (addr - UART_BASE) & 0x7;

    switch (offset)
    {
    case UART_REG_THR:
        if (uart.lcr & UART_LCR_DLAB)
        {
            uart.dll = value;
            return;
        }
        putchar(value);
        fflush(stdout);
        uart.lsr |= UART_LSR_THRE | UART_LSR_TEMT;
        return;
    case UART_REG_IER:
        if (uart.lcr & UART_LCR_DLAB)
        {
            uart.dlm = value;
            return;
        }
        uart.ier = value;
        uart_update_irq();
        return;
    case UART_REG_FCR:
        uart.fcr = value;
        if (value & UART_FCR_CLEAR_RCVR)
        {
            uart.rx_ready = 0;
            uart.lsr &= (uint8_t)~UART_LSR_DR;
            uart_update_irq();
        }
        return;
    case UART_REG_LCR:
        uart.lcr = value;
        return;
    case UART_REG_MCR:
        uart.mcr = value;
        return;
    case UART_REG_SCR:
        uart.scr = value;
        return;
    default:
        return;
    }
}

static inline uint8_t clint_read8(uint32_t addr)
{
    switch (addr)
    {
    case CLINT_BASE:
    case CLINT_BASE + 1:
    case CLINT_BASE + 2:
    case CLINT_BASE + 3:
    {
        uint32_t offset = (addr - CLINT_BASE) * 8;
        return (clint.msip >> offset) & 0xff;
    }
    case CLINT_MTIME:
    case CLINT_MTIME + 1:
    case CLINT_MTIME + 2:
    case CLINT_MTIME + 3:
    case CLINT_MTIME + 4:
    case CLINT_MTIME + 5:
    case CLINT_MTIME + 6:
    case CLINT_MTIME + 7:
    {
        uint32_t offset = (addr - CLINT_MTIME) * 8;
        uint64_t mask = (0xffULL << offset);
        return (clint.mtime & mask) >> offset;
    }
    case CLINT_MTIMECMP:
    case CLINT_MTIMECMP + 1:
    case CLINT_MTIMECMP + 2:
    case CLINT_MTIMECMP + 3:
    case CLINT_MTIMECMP + 4:
    case CLINT_MTIMECMP + 5:
    case CLINT_MTIMECMP + 6:
    case CLINT_MTIMECMP + 7:
    {
        uint32_t offset = (addr - CLINT_MTIMECMP) * 8;
        uint64_t mask = (0xffULL << offset);
        return (clint.mtimecmp & mask) >> offset;
    }
    default:
        return 0;
    }
}

static inline uint8_t plic_read8(uint32_t addr)
{
    uint32_t word_addr = addr & ~0x3u;
    uint32_t shift = (addr & 0x3u) * 8u;
    uint32_t value;

    if (word_addr == PLIC_BASE + PLIC_CONTEXT_OFFSET + PLIC_CLAIM_OFFSET)
    {
        if ((addr & 0x3u) == 0 || !plic.claim_read_active)
        {
            plic.claim_read_latch = plic_read32(word_addr);
            plic.claim_read_active = 1;
        }

        value = plic.claim_read_latch;

        if ((addr & 0x3u) == 3)
            plic.claim_read_active = 0;
    }
    else
    {
        value = plic_read32(word_addr);
    }

    return (value >> shift) & 0xffu;
}

static inline void plic_write8(uint32_t addr, uint8_t value)
{
    uint32_t word_addr = addr & ~0x3u;
    uint32_t shift = (addr & 0x3u) * 8u;

    if (word_addr == PLIC_BASE + PLIC_CONTEXT_OFFSET + PLIC_CLAIM_OFFSET)
    {
        plic.claim_write_latch &= ~(0xffu << shift);
        plic.claim_write_latch |= (uint32_t)value << shift;
        plic.claim_write_mask |= 1u << (addr & 0x3u);

        if (plic.claim_write_mask == 0x0fu)
        {
            plic_write32(word_addr, plic.claim_write_latch);
            plic.claim_write_latch = 0;
            plic.claim_write_mask = 0;
        }

        return;
    }

    uint32_t current = plic_read32(word_addr);
    current &= ~(0xffu << shift);
    current |= (uint32_t)value << shift;
    plic_write32(word_addr, current);
}

static inline void clint_write8(uint32_t addr, uint8_t value)
{
    switch (addr)
    {
    case CLINT_MTIME:
    case CLINT_MTIME + 1:
    case CLINT_MTIME + 2:
    case CLINT_MTIME + 3:
    case CLINT_MTIME + 4:
    case CLINT_MTIME + 5:
    case CLINT_MTIME + 6:
    case CLINT_MTIME + 7:
    {
        uint32_t offset = (addr - CLINT_MTIME) * 8;
        uint64_t mask = (0xffULL << offset);
        clint.mtime &= ~mask;
        clint.mtime |= (uint64_t)value << offset;
        break;
    }
    case CLINT_MTIMECMP:
    case CLINT_MTIMECMP + 1:
    case CLINT_MTIMECMP + 2:
    case CLINT_MTIMECMP + 3:
    case CLINT_MTIMECMP + 4:
    case CLINT_MTIMECMP + 5:
    case CLINT_MTIMECMP + 6:
    case CLINT_MTIMECMP + 7:
    {
        uint32_t offset = (addr - CLINT_MTIMECMP) * 8;
        uint64_t mask = (0xffULL << offset);
        clint.mtimecmp &= ~mask;
        clint.mtimecmp |= (uint64_t)value << offset;
        break;
    }
    case CLINT_BASE:
    case CLINT_BASE + 1:
    case CLINT_BASE + 2:
    case CLINT_BASE + 3:
    {
        uint32_t offset = (addr - CLINT_BASE) * 8;
        uint32_t mask = 0xffu << offset;
        clint.msip = (clint.msip & ~mask) | ((uint32_t)value << offset);
        break;
    }
    }
}

static inline MemoryDispatch memory_dispatch(uint32_t addr)
{
    if (addr >= UART_BASE && addr < UART_BASE + UART_SIZE)
    {
        return (MemoryDispatch){.read8 = uart_read8, .write8 = uart_write8};
    }

    if (addr >= PLIC_BASE && addr < PLIC_BASE + PLIC_SIZE)
    {
        return (MemoryDispatch){.read8 = plic_read8, .write8 = plic_write8};
    }

    if (addr >= CLINT_BASE && addr < CLINT_BASE + CLINT_SIZE)
    {
        return (MemoryDispatch){.read8 = clint_read8, .write8 = clint_write8};
    }

    if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE)
    {
        return (MemoryDispatch){.read8 = ram_read8, .write8 = ram_write8};
    }

    fprintf(stderr, "invalid memory access at 0x%08x (PC=0x%08x)\n", addr, cpu.pc);
    exit(1);
}

static inline uint8_t memory_read8(uint32_t addr)
{
    return memory_dispatch(addr).read8(addr);
}

static inline uint16_t memory_read16(uint32_t addr)
{
    return (uint16_t)memory_read8(addr) | ((uint16_t)memory_read8(addr + 1) << 8);
}

static inline uint32_t memory_read32(uint32_t addr)
{
    return (uint32_t)memory_read8(addr) |
           ((uint32_t)memory_read8(addr + 1) << 8) |
           ((uint32_t)memory_read8(addr + 2) << 16) |
           ((uint32_t)memory_read8(addr + 3) << 24);
}

static inline void memory_write8(uint32_t addr, uint8_t value)
{
    memory_dispatch(addr).write8(addr, value);
}

static inline void memory_write16(uint32_t addr, uint16_t value)
{
    memory_write8(addr, value & 0xff);
    memory_write8(addr + 1, (value >> 8) & 0xff);
}

static inline void memory_write32(uint32_t addr, uint32_t value)
{
    memory_write8(addr, value & 0xff);
    memory_write8(addr + 1, (value >> 8) & 0xff);
    memory_write8(addr + 2, (value >> 16) & 0xff);
    memory_write8(addr + 3, (value >> 24) & 0xff);
}

static int load_program(const char *path)
{
    FILE *f = fopen(path, "rb");
    size_t nread;

    if (f == NULL)
    {
        perror("failed to open program");
        return -1;
    }

    nread = fread(memory, 1, RAM_SIZE, f);
    if (ferror(f))
    {
        perror("failed to read program");
        fclose(f);
        return -1;
    }

    if (!feof(f))
    {
        fprintf(stderr, "program is larger than emulator memory (%u bytes)\n", RAM_SIZE);
        fclose(f);
        return -1;
    }

    fclose(f);
    printf("loaded %zu bytes from %s\n", nread, path);
    return 0;
}

void trap(CpuState *cpu, uint32_t cause)
{
    if (cpu->csr[CSR_MSTATUS] & CSR_MSTATUS_MIE)
    {
        cpu->csr[CSR_MSTATUS] |= CSR_MSTATUS_MPIE;
    }
    else
    {
        cpu->csr[CSR_MSTATUS] &= ~CSR_MSTATUS_MPIE;
    }
    cpu->csr[CSR_MSTATUS] &= ~CSR_MSTATUS_MIE;
    // Set MPP to M-mode (3)
    cpu->csr[CSR_MSTATUS] = (cpu->csr[CSR_MSTATUS] & ~CSR_MSTATUS_MPP_MASK) | CSR_MSTATUS_MPP_M;
    cpu->csr[CSR_MEPC] = cpu->pc;
    cpu->csr[CSR_MCAUSE] = cause;
    cpu->pc = cpu->csr[CSR_MTVEC] - 4;
}

void handleRTypeInst(CpuState *cpu, uint32_t instr, uint32_t opcode)
{
    uint32_t rd = (instr & 0xf80) >> 7;
    uint32_t func3 = (instr & 0x7000) >> 12;
    uint32_t rs1 = (instr & 0xf8000) >> 15;
    uint32_t rs2 = (instr & 0x1f00000) >> 20;
    uint32_t func7 = (instr & 0xfe000000) >> 25;
    switch (opcode)
    {
    case 0x33:
    {
        switch (func7)
        {
        case 0x0:
        {
            switch (func3)
            {
            case 0x0: // add
                cpu->regs[rd] = cpu->regs[rs1] + cpu->regs[rs2];
                break;
            case 0x1: // sll
                cpu->regs[rd] = cpu->regs[rs1] << (cpu->regs[rs2] & 0x1f);
                break;
            case 0x2: // slt
                cpu->regs[rd] = ((int32_t)cpu->regs[rs1] < (int32_t)cpu->regs[rs2]) ? 1 : 0;
                break;
            case 0x3: // sltu
                cpu->regs[rd] = (cpu->regs[rs1] < cpu->regs[rs2]) ? 1 : 0;
                break;
            case 0x4: // xor
                cpu->regs[rd] = cpu->regs[rs1] ^ cpu->regs[rs2];
                break;
            case 0x5:
                cpu->regs[rd] = cpu->regs[rs1] >> (cpu->regs[rs2] & 0x1f); // srl
                break;
            case 0x6: // or
                cpu->regs[rd] = cpu->regs[rs1] | cpu->regs[rs2];
                break;
            case 0x7: // and
                cpu->regs[rd] = cpu->regs[rs1] & cpu->regs[rs2];
                break;
            }
            break;
        }
        case 0x20: // sub
        {
            if (func3 == 0x0)
            {
                cpu->regs[rd] = cpu->regs[rs1] - cpu->regs[rs2];
                break;
            }
            else if (func3 == 0x5) // sra
            {
                cpu->regs[rd] = (uint32_t)((int32_t)cpu->regs[rs1] >> (cpu->regs[rs2] & 0x1f)); // sra
                break;
            }
            break;
        }
        case 0x01: // mul
        {
            switch (func3)
            {
            case 0x0: // mul
                cpu->regs[rd] = (uint64_t)cpu->regs[rs1] * (uint64_t)cpu->regs[rs2] & 0xffffffff;
                break;
            case 0x1: // mulh
                cpu->regs[rd] = ((int64_t)(int32_t)cpu->regs[rs1] * (int64_t)(int32_t)cpu->regs[rs2] & 0xffffffff00000000ULL) >> 32;
                break;
            case 0x2: // mulhsu
                cpu->regs[rd] = ((int64_t)(int32_t)cpu->regs[rs1] * (int64_t)cpu->regs[rs2] & 0xffffffff00000000ULL) >> 32;
                break;
            case 0x3: // mulhu
                cpu->regs[rd] = ((uint64_t)cpu->regs[rs1] * (uint64_t)cpu->regs[rs2] & 0xffffffff00000000ULL) >> 32;
                break;
            case 0x4: // div
                if (cpu->regs[rs1] == 0x80000000 && cpu->regs[rs2] == 0xffffffff)
                {
                    cpu->regs[rd] = 0x80000000; // special case: overflow
                }
                else
                {
                    cpu->regs[rd] = (cpu->regs[rs2] == 0) ? ~0U : (int32_t)cpu->regs[rs1] / (int32_t)cpu->regs[rs2];
                }
                break;
            case 0x5: // divu
                cpu->regs[rd] = (cpu->regs[rs2] == 0) ? ~0U : cpu->regs[rs1] / cpu->regs[rs2];
                break;
            case 0x6: // rem
                if (cpu->regs[rs1] == 0x80000000 && cpu->regs[rs2] == 0xffffffff)
                {
                    cpu->regs[rd] = 0; // special case: overflow
                }
                else
                {
                    cpu->regs[rd] = (cpu->regs[rs2] == 0) ? cpu->regs[rs1] : (int32_t)cpu->regs[rs1] % (int32_t)cpu->regs[rs2];
                }
                break;
            case 0x7: // remu
                cpu->regs[rd] = (cpu->regs[rs2] == 0) ? cpu->regs[rs1] : cpu->regs[rs1] % cpu->regs[rs2];
                break;
            }
            break;
        }
        }
        break;
    }
    case 0x2f: // atomic
    {
        uint32_t func5 = (func7 >> 2);
        switch (func5)
        {
        case 0x02: // lr.w
        {
            uint32_t addr = cpu->regs[rs1];
            cpu->regs[rd] = memory_read32(addr);
            cpu->atomic_resv_addr = addr;
            break;
        }
        case 0x03: // sc.w
        {
            uint32_t addr = cpu->regs[rs1];
            if (cpu->atomic_resv_addr == addr)
            {
                memory_write32(addr, cpu->regs[rs2]);
                cpu->regs[rd] = 0;
            }
            else
            {
                cpu->regs[rd] = 1;
            }
            cpu->atomic_resv_addr = ~0ULL;
            break;
        }
        case 0x01: // amoswap.w
        {
            uint32_t addr = cpu->regs[rs1];
            uint32_t old = memory_read32(addr);
            memory_write32(addr, cpu->regs[rs2]);
            cpu->regs[rd] = old;
            break;
        }
        case 0x00: // amoadd.w
        {
            uint32_t addr = cpu->regs[rs1];
            uint32_t old = memory_read32(addr);
            memory_write32(addr, old + cpu->regs[rs2]);
            cpu->regs[rd] = old;
            break;
        }
        case 0x04: // amoxor.w
        {
            uint32_t addr = cpu->regs[rs1];
            uint32_t old = memory_read32(addr);
            memory_write32(addr, old ^ cpu->regs[rs2]);
            cpu->regs[rd] = old;
            break;
        }
        case 0x0c: // amoand.w
        {
            uint32_t addr = cpu->regs[rs1];
            uint32_t old = memory_read32(addr);
            memory_write32(addr, old & cpu->regs[rs2]);
            cpu->regs[rd] = old;
            break;
        }
        case 0x08: // amoor.w
        {
            uint32_t addr = cpu->regs[rs1];
            uint32_t old = memory_read32(addr);
            memory_write32(addr, old | cpu->regs[rs2]);
            cpu->regs[rd] = old;
            break;
        }
        case 0x10: // amomin.w
        {
            uint32_t addr = cpu->regs[rs1];
            uint32_t old = memory_read32(addr);
            int32_t s_old = (int32_t)old;
            int32_t s_rs2 = (int32_t)cpu->regs[rs2];
            memory_write32(addr, (uint32_t)(s_old < s_rs2 ? s_old : s_rs2));
            cpu->regs[rd] = old;
            break;
        }
        case 0x14: // amomax.w
        {
            uint32_t addr = cpu->regs[rs1];
            uint32_t old = memory_read32(addr);
            int32_t s_old = (int32_t)old;
            int32_t s_rs2 = (int32_t)cpu->regs[rs2];
            memory_write32(addr, (uint32_t)(s_old > s_rs2 ? s_old : s_rs2));
            cpu->regs[rd] = old;
            break;
        }
        case 0x18: // amominu.w
        {
            uint32_t addr = cpu->regs[rs1];
            uint32_t old = memory_read32(addr);
            memory_write32(addr, old < cpu->regs[rs2] ? old : cpu->regs[rs2]);
            cpu->regs[rd] = old;
            break;
        }
        case 0x1c: // amomaxu.w
        {
            uint32_t addr = cpu->regs[rs1];
            uint32_t old = memory_read32(addr);
            memory_write32(addr, old > cpu->regs[rs2] ? old : cpu->regs[rs2]);
            cpu->regs[rd] = old;
            break;
        }
        }
        break;
    }
    }
}

void handleITypeInst(CpuState *cpu, uint32_t instr, uint32_t opcode)
{
    uint32_t rd = (instr & 0xf80) >> 7;
    uint32_t func3 = (instr & 0x7000) >> 12;
    uint32_t rs1 = (instr & 0xf8000) >> 15;
    int32_t imm = ((uint32_t)instr & 0x7ff00000) >> 20;
    if (instr & 0x80000000)
    {
        imm |= 0xfffff800; // sign-extend
    }
    uint32_t csr_addr = ((uint32_t)instr & 0xfff00000) >> 20;

    switch (opcode)
    {
    case 0x13:
    {
        switch (func3)
        {
        case 0x0: // addi
            cpu->regs[rd] = cpu->regs[rs1] + imm;
            break;
        case 0x1: // slli
            cpu->regs[rd] = cpu->regs[rs1] << (imm & 0x1f);
            break;
        case 0x2: // slti (Set Less Than Immediate, Signed)
            cpu->regs[rd] = ((int32_t)cpu->regs[rs1] < imm) ? 1 : 0;
            break;
        case 0x3: // sltiu (Set Less Than Immediate, Unsigned)
            // Must cast the sign-extended imm back to unsigned for the comparison!
            cpu->regs[rd] = (cpu->regs[rs1] < (uint32_t)imm) ? 1 : 0;
            break;
        case 0x4: // xori
            cpu->regs[rd] = cpu->regs[rs1] ^ imm;
            break;
        case 0x5:                                                                    // srli / srai
            if (instr & 0x40000000)                                                  // check bit 30 for arithmetic shift
                cpu->regs[rd] = (uint32_t)((int32_t)cpu->regs[rs1] >> (imm & 0x1f)); // srai
            else
                cpu->regs[rd] = cpu->regs[rs1] >> (imm & 0x1f); // srli
            break;
        case 0x6: // ori
            cpu->regs[rd] = cpu->regs[rs1] | imm;
            break;
        case 0x7: // andi
            cpu->regs[rd] = cpu->regs[rs1] & imm;
            break;
        }
        break;
    }
    case 0x03:
    {
        uint32_t addr = cpu->regs[rs1] + imm;
        switch (func3)
        {
        case 0x0: // lb
            cpu->regs[rd] = (int32_t)(int8_t)memory_read8(addr);
            break;
        case 0x1: // lh
        {
            int16_t val = (int16_t)memory_read16(addr);
            cpu->regs[rd] = (int32_t)val;
            break;
        }
        case 0x2: // lw
            cpu->regs[rd] = memory_read32(addr);
            break;
        case 0x4: // lbu
            cpu->regs[rd] = memory_read8(addr);
            break;
        case 0x5: // lhu
            cpu->regs[rd] = memory_read16(addr);
            break;
        }
        break;
    }
    case 0x67:
    {
        // jalr
        uint32_t target = (cpu->regs[rs1] + imm) & ~1;
        cpu->regs[rd] = cpu->pc + 4;
        cpu->pc = target - 4; // update PC
        break;
    }
    case 0x73:
    {
        switch (func3)
        {
        case 0x0:
            if (imm == 0x0)
            {
                trap(cpu, 11); // ecall
                break;
            }
            else if (imm == 0x302)
            {
                // mret
                cpu->pc = cpu->csr[CSR_MEPC] - 4;
                if (cpu->csr[CSR_MSTATUS] & CSR_MSTATUS_MPIE)
                {
                    cpu->csr[CSR_MSTATUS] |= CSR_MSTATUS_MIE;
                }
                else
                {
                    cpu->csr[CSR_MSTATUS] &= ~CSR_MSTATUS_MIE;
                }
                cpu->csr[CSR_MSTATUS] |= CSR_MSTATUS_MPIE;
                break;
            }
            break;
        case 0x1: // csrrw
        {
            uint32_t old_csr = cpu->csr[csr_addr];
            cpu->csr[csr_addr] = cpu->regs[rs1];
            cpu->regs[rd] = old_csr;
            break;
        }
        case 0x2: // csrrs
        {
            uint32_t old_csr = cpu->csr[csr_addr];
            cpu->csr[csr_addr] = old_csr | cpu->regs[rs1];
            cpu->regs[rd] = old_csr;
            break;
        }
        case 0x3: // csrrc
        {
            uint32_t old_csr = cpu->csr[csr_addr];
            cpu->csr[csr_addr] = old_csr & ~cpu->regs[rs1];
            cpu->regs[rd] = old_csr;
            break;
        }
        case 0x5: // csrrwi
        {
            uint32_t old_csr = cpu->csr[csr_addr];
            cpu->csr[csr_addr] = rs1;
            cpu->regs[rd] = old_csr;
            break;
        }
        case 0x6: // csrrsi
        {
            uint32_t old_csr = cpu->csr[csr_addr];
            cpu->csr[csr_addr] = old_csr | rs1;
            cpu->regs[rd] = old_csr;
            break;
        }
        case 0x7: // csrrci
        {
            uint32_t old_csr = cpu->csr[csr_addr];
            cpu->csr[csr_addr] = old_csr & ~rs1;
            cpu->regs[rd] = old_csr;
            break;
        }
        }
    }
    case 0x0f: // fence.i
    {
        // do nothing as there is no cache
        break;
    }
    }
}

void handleBTypeInst(CpuState *cpu, uint32_t instr)
{
    uint32_t func3 = (instr & 0x7000) >> 12;
    uint32_t rs1 = (instr & 0xf8000) >> 15;
    uint32_t rs2 = (instr & 0x1f00000) >> 20;
    int32_t imm = 0;
    if (instr & 0x80000000)
    {
        imm = 0xfffff000; // sign-extend
    }
    imm |= (instr & 0x80) << 4;        // imm[11]
    imm |= (instr & 0x7e000000) >> 20; // imm[5:10]
    imm |= (instr & 0xf00) >> 7;       // imm[1:4]

    uint32_t take_branch = 0;
    switch (func3)
    {
    case 0x0: // beq
        take_branch = (cpu->regs[rs1] == cpu->regs[rs2]);
        break;
    case 0x1: // bne
        take_branch = (cpu->regs[rs1] != cpu->regs[rs2]);
        break;
    case 0x4: // blt
        take_branch = ((int32_t)cpu->regs[rs1] < (int32_t)cpu->regs[rs2]);
        break;
    case 0x5: // bge
        take_branch = ((int32_t)cpu->regs[rs1] >= (int32_t)cpu->regs[rs2]);
        break;
    case 0x6: // bltu
        take_branch = (cpu->regs[rs1] < cpu->regs[rs2]);
        break;
    case 0x7: // bgeu
        take_branch = (cpu->regs[rs1] >= cpu->regs[rs2]);
        break;
    }

    if (take_branch)
    {
        cpu->pc += imm - 4; // count for the pc increment in main loop
    }
}

void handleSTypeInst(CpuState *cpu, uint32_t instr)
{
    uint32_t func3 = (instr & 0x7000) >> 12;
    uint32_t rs1 = (instr & 0xf8000) >> 15;
    uint32_t rs2 = (instr & 0x1f00000) >> 20;
    int32_t imm = 0;
    if (instr & 0x80000000)
    {
        imm = 0xfffff800; // sign-extend
    }
    imm |= (instr & 0x7e000000) >> 20; // imm[5:10]
    imm |= (instr & 0xf80) >> 7;       // imm[0:4]

    uint32_t addr = cpu->regs[rs1] + imm;
    switch (func3)
    {
    case 0x0: // sb
        memory_write8(addr, cpu->regs[rs2] & 0xff);
        break;

    case 0x1: // sh
        memory_write16(addr, cpu->regs[rs2] & 0xffff);
        break;
    case 0x2: // sw
        memory_write32(addr, cpu->regs[rs2]);
        break;
    }
}

void handleJTypeInst(CpuState *cpu, uint32_t instr)
{
    uint32_t rd = (instr & 0xf80) >> 7;
    int32_t imm = 0;
    if (instr & 0x80000000)
    {
        imm = 0xfff00000; // sign-extend
    }
    imm |= (instr & 0x7FE00000) >> 20; // imm[10:1]
    imm |= (instr & 0x100000) >> 9;    // imm[11]
    imm |= (instr & 0xFF000);          // imm[19:12]

    // jal
    if (rd != 0)
    {
        cpu->regs[rd] = cpu->pc + 4; // return address
    }
    cpu->pc += imm - 4; // update PC
}

void handleUTypeInst(CpuState *cpu, uint32_t instr, uint32_t opcode)
{
    uint32_t rd = (instr & 0xf80) >> 7;

    uint32_t imm = instr & 0xfffff000;

    switch (opcode)
    {
    case 0x37: // lui
        cpu->regs[rd] = imm;
        break;
    case 0x17: // auipc
        cpu->regs[rd] = cpu->pc + imm;
        break;
    }
}

#define MTIME_DIVISOR 10
static uint64_t mtime_accumulator = 0;

static void machine_update_irqs(void)
{
    mtime_accumulator++;
    if (mtime_accumulator >= MTIME_DIVISOR)
    {
        mtime_accumulator = 0;
        clint.mtime++;
    }

    if (clint.mtime >= clint.mtimecmp)
        cpu.csr[CSR_MIP] |= CSR_MIP_MTIP;
    else
        cpu.csr[CSR_MIP] &= ~CSR_MIP_MTIP;

    if (clint.msip & 1)
        cpu.csr[CSR_MIP] |= CSR_MIP_MSIP;
    else
        cpu.csr[CSR_MIP] &= ~CSR_MIP_MSIP;

    if (!(cpu.csr[CSR_MSTATUS] & CSR_MSTATUS_MIE))
        return;

    uint32_t cause = 0;
    if ((cpu.csr[CSR_MIP] & CSR_MIP_MEIP) && (cpu.csr[CSR_MIE] & CSR_MIE_MEIE))
        cause = 11;
    else if ((cpu.csr[CSR_MIP] & CSR_MIP_MSIP) && (cpu.csr[CSR_MIE] & CSR_MIE_MSIE))
        cause = 3;
    else if ((cpu.csr[CSR_MIP] & CSR_MIP_MTIP) && (cpu.csr[CSR_MIE] & CSR_MIE_MTIE))
        cause = 7;
    else
        return;

    // Save and clear MIE
    if (cpu.csr[CSR_MSTATUS] & CSR_MSTATUS_MIE)
        cpu.csr[CSR_MSTATUS] |= CSR_MSTATUS_MPIE;
    else
        cpu.csr[CSR_MSTATUS] &= ~CSR_MSTATUS_MPIE;
    cpu.csr[CSR_MSTATUS] &= ~CSR_MSTATUS_MIE;
    // Set MPP to M-mode (3)
    cpu.csr[CSR_MSTATUS] = (cpu.csr[CSR_MSTATUS] & ~CSR_MSTATUS_MPP_MASK) | CSR_MSTATUS_MPP_M;

    cpu.csr[CSR_MEPC] = cpu.pc;
    cpu.csr[CSR_MCAUSE] = 0x80000000u | cause;

    uint32_t base = cpu.csr[CSR_MTVEC] & ~3u;
    uint32_t mode = cpu.csr[CSR_MTVEC] & 3u;
    if (mode == 1)
        cpu.pc = base + 4 * cause;
    else
        cpu.pc = base;
}

int main(int argc, char **argv)
{
    // initialize resv addr to all 1s
    cpu.atomic_resv_addr = ~0ULL;

    memory = calloc(1, RAM_SIZE);
    if (memory == NULL)
    {
        perror("failed to allocate emulator RAM");
        return 1;
    }

    if (uart_init_stdio() != 0)
    {
        free(memory);
        return 1;
    }

    uint32_t instr = 0;
    uint32_t tohost_addr = 0;
    const char *bin_path = NULL;

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--tohost") == 0 && i + 1 < argc)
        {
            tohost_addr = (uint32_t)strtoul(argv[++i], NULL, 0);
        }
        else if (!bin_path)
        {
            bin_path = argv[i];
        }
        else
        {
            fprintf(stderr, "usage: %s [--tohost ADDR] <program.bin>\n", argv[0]);
            free(memory);
            return 1;
        }
    }

    if (!bin_path)
    {
        fprintf(stderr, "usage: %s [--tohost ADDR] <program.bin>\n", argv[0]);
        free(memory);
        return 1;
    }

    if (load_program(bin_path) != 0)
    {
        free(memory);
        return 1;
    }

    cpu.pc = RAM_BASE;
    cpu.regs[10] = 0;
    cpu.regs[11] = FDT_ADDR;
    cpu.csr[CSR_MISA] = MISA_VALUE;
    cpu.csr[CSR_MHARTID] = 0;

    while (1)
    {
        if (cpu.pc < RAM_BASE || cpu.pc >= RAM_BASE + RAM_SIZE)
        {
            fprintf(stderr, "PC out of range: 0x%08x\n", cpu.pc);
            free(memory);
            return 1;
        }

        // fetch
        instr = memory_read32(cpu.pc);

        // decode
        uint32_t opcode = instr & 0b1111111;

        OpcodeType format = opcode_get_format(opcode);

        // execute
        switch (format)
        {
        case OPCODE_RTYPE:
            handleRTypeInst(&cpu, instr, opcode);
            break;
        case OPCODE_ITYPE:
            handleITypeInst(&cpu, instr, opcode);
            break;
        case OPCODE_BTYPE:
            handleBTypeInst(&cpu, instr);
            break;
        case OPCODE_STYPE:
            handleSTypeInst(&cpu, instr);
            break;
        case OPCODE_JTYPE:
            handleJTypeInst(&cpu, instr);
            break;
        case OPCODE_UTYPE:
            handleUTypeInst(&cpu, instr, opcode);
            break;

        default:
            fprintf(stderr, "Illegal instr at PC 0x%08x: 0x%08x\n", cpu.pc, instr);
            free(memory);
            return 1;
        }

        cpu.pc += 4;
        cpu.regs[0] = 0; // x0 is always 0

        if (tohost_addr)
        {
            uint32_t val = memory_read32(tohost_addr);
            if (val != 0)
            {
                free(memory);
                return (val == 1) ? 0 : 1;
            }
        }

        uart_poll_input();
        machine_update_irqs();
    }
}