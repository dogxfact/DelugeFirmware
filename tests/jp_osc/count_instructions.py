#!/usr/bin/env python3
# Count ARM instructions executed per 44.1 kHz output sample by the JP Super Saw render (one voice).
# Usage: count_instructions.py bench.elf   (needs: pip install unicorn pyelftools)
import io
import sys

from elftools.elf.elffile import ELFFile
from unicorn import UC_ARCH_ARM, UC_HOOK_CODE, UC_MODE_ARM, Uc
from unicorn.arm_const import (
    UC_ARM_REG_C1_C0_2,
    UC_ARM_REG_FPEXC,
    UC_ARM_REG_LR,
    UC_ARM_REG_R0,
    UC_ARM_REG_SP,
)

with open(sys.argv[1], "rb") as f:
    elf = ELFFile(io.BytesIO(f.read()))
mu = Uc(UC_ARCH_ARM, UC_MODE_ARM)
for seg in elf.iter_segments():
    if seg["p_type"] != "PT_LOAD":
        continue
    base = seg["p_vaddr"] & ~0xFFF
    size = ((seg["p_vaddr"] + seg["p_memsz"] + 0xFFF) & ~0xFFF) - base
    mu.mem_map(base, size)
    mu.mem_write(seg["p_vaddr"], seg.data())
syms = {
    s.name: s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()
}
STACK, RET = 0x7F000000, 0x7E000000
mu.mem_map(STACK - 0x100000, 0x100000)
mu.mem_map(RET, 0x1000)
mu.reg_write(
    UC_ARM_REG_C1_C0_2, mu.reg_read(UC_ARM_REG_C1_C0_2) | (0xF << 20)
)  # enable VFP/NEON
mu.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
count = [0]
mu.hook_add(UC_HOOK_CODE, lambda uc, addr, size, ud: count.__setitem__(0, count[0] + 1))


def call(name, arg=0):
    count[0] = 0
    mu.reg_write(UC_ARM_REG_SP, STACK - 16)
    mu.reg_write(UC_ARM_REG_LR, RET)
    mu.reg_write(UC_ARM_REG_R0, arg)
    addr = syms[name]
    mu.emu_start(addr | (1 if addr & 1 else 0), RET)
    return count[0]


call("benchSetup")
blocks = 64
for name in ("benchSettled", "benchMoving"):
    n = call(name, blocks)
    print(
        f"{name[5:].lower():8s}: {n / (blocks * 128):7.1f} instructions per 44.1 kHz sample per voice"
    )
