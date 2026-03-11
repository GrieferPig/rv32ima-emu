CC := cc
CFLAGS := -std=c11 -O2 -Wall -Wextra

EMULATOR := emulator
EMULATOR_SRC := main.c

RISCV_PREFIX := riscv64-unknown-elf-
AS := $(RISCV_PREFIX)as
LD := $(RISCV_PREFIX)ld
OBJCOPY := $(RISCV_PREFIX)objcopy

ASM_SRC := test.asm
LINKER_SCRIPT := linkerscript.ld
ASM_OBJ := test.o
ASM_ELF := test.elf
ASM_BIN := test.bin

.PHONY: all

all: $(EMULATOR)

$(EMULATOR): $(EMULATOR_SRC)
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f $(EMULATOR)
