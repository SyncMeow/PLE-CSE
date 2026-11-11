# toolchain.mk -- picks the RISC-V GCC prefix for the bare-metal software Makefiles.
# Tries riscv64-unknown-elf first (Homebrew / macOS), then riscv32-unknown-elf, and takes the
# first one whose gcc accepts $(ARCH_FLAGS) and can link the matching rv32/ilp32 libgcc, so an
# old GCC (no Zicbom) or a toolchain missing that multilib is skipped instead of failing later.
# Define ARCH_FLAGS before including. Override with TARGET_PREFIX=<prefix> to skip detection.

RISCV_PREFIX_CANDIDATES ?= riscv64-unknown-elf riscv32-unknown-elf \
                           /opt/riscv/bin/riscv32-unknown-elf

# 64-bit division forces a libgcc call (__divdi3), so a wrong-ABI libgcc fails the link.
rv_toolchain_ok = $(shell printf 'long long f(long long a, long long b) { return a / b; }\nvoid _start(void) {}\n' | \
    $(1)-gcc $(ARCH_FLAGS) -x c - -nostdlib -nostartfiles -lgcc -o /dev/null >/dev/null 2>&1 && echo $(1))

ifeq ($(origin TARGET_PREFIX),undefined)
TARGET_PREFIX := $(firstword $(foreach p,$(RISCV_PREFIX_CANDIDATES),$(call rv_toolchain_ok,$(p))))
ifeq ($(TARGET_PREFIX),)
ifneq ($(filter-out clean distclean,$(or $(MAKECMDGOALS),all)),)
$(error No usable RISC-V GCC: tried $(RISCV_PREFIX_CANDIDATES) with "$(ARCH_FLAGS)". Install one or pass TARGET_PREFIX=)
endif
endif
endif
# Recursive makes reuse the result instead of probing again.
export TARGET_PREFIX
