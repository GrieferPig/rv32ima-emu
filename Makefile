CC := cc
CFLAGS := -std=c11 -O2 -Wall -Wextra

EMULATOR := emulator
EMULATOR_SRC := main.c
MINIFB_DIR := third_party/minifb
MINIFB_SRC := \
	$(MINIFB_DIR)/src/MiniFB_common.c \
	$(MINIFB_DIR)/src/MiniFB_internal.c \
	$(MINIFB_DIR)/src/MiniFB_timer.c \
	$(MINIFB_DIR)/src/MiniFB_linux.c \
	$(MINIFB_DIR)/src/x11/X11MiniFB.c
X11_LIB := $(firstword $(wildcard /usr/lib/x86_64-linux-gnu/libX11.so.6 /usr/lib/arm-linux-gnueabihf/libX11.so.6 /usr/lib64/libX11.so.6 /usr/lib/libX11.so.6))
XKBCOMMON_LIB := $(firstword $(wildcard /usr/lib/x86_64-linux-gnu/libxkbcommon.so.0 /usr/lib/arm-linux-gnueabihf/libxkbcommon.so.0 /usr/lib64/libxkbcommon.so.0 /usr/lib/libxkbcommon.so.0))
EMULATOR_CPPFLAGS := -I$(MINIFB_DIR)/include -I$(MINIFB_DIR)/src
EMULATOR_LDLIBS := $(X11_LIB) $(XKBCOMMON_LIB)

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

$(EMULATOR): $(EMULATOR_SRC) $(MINIFB_SRC)
	$(CC) $(CFLAGS) $(EMULATOR_CPPFLAGS) -o $@ $(EMULATOR_SRC) $(MINIFB_SRC) $(EMULATOR_LDLIBS)

clean:
	rm -f $(EMULATOR)
