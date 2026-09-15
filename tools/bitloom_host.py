"""cocotb-side host driver for BitLoom.

Owns the testbench's input pins (``ui_in`` / ``uio_in``), bit-bangs the SPI
host port on uio[7:4], resolves the bidirectional uio[3:0] pins against
external models, and provides the register-level API used by the tests.
"""
from __future__ import annotations

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge

from bitloom_regs import *  # noqa: F401,F403
from bitloom_regs import (CMD_IMEM, CMD_WRITE, R_ENABLE, R_RESTART, R_STEP, S_DIV_FRAC,
                          S_DIV_INT_H, S_DIV_INT_L, S_EXEC_H, S_EXEC_L, S_IN_BASE, S_JMP_PIN,
                          S_LEVELS, S_OUT_BASE, S_OUT_COUNT, S_PC, S_PULL_THRESH, S_PUSH_THRESH,
                          S_RXF_H, S_RXF_L, S_SET_BASE, S_SET_COUNT, S_SHIFTCTRL, S_SIDESET,
                          S_STATUS, S_TXF_H, S_TXF_L, S_WRAP_BOT, S_WRAP_TOP, S_X_H, S_X_L,
                          S_Y_H, S_Y_L, sm_reg)
from bitloom_asm import Program


def sig_int(sig) -> int:
    """Integer value of a signal, treating X/Z bits (before reset) as 0."""
    v = sig.value
    try:
        return int(v)
    except ValueError:
        return int("".join(c if c in "01" else "0" for c in str(v)), 2)


class Pins:
    """Single owner of the DUT input buses.

    ``ui`` holds the 8 dedicated inputs.  ``ext_uio`` holds the external drive
    for uio[3:0]: ``None`` means released (pulled up), otherwise 0/1.  When the
    chip drives a uio pin (oe=1) the pad value follows the chip; an external
    low drive wins over a chip high (open-drain bus model).
    ``loopback`` maps ui bit index -> gpio number whose driven value feeds it.
    """

    def __init__(self, dut):
        self.dut = dut
        self.ui = 0
        self.ext_uio = [None] * 4
        self.loopback: dict[int, int] = {}
        self.spi_cs = 1
        self.spi_sck = 0
        self.spi_mosi = 0
        self._task = cocotb.start_soon(self._drive())

    def uio_line(self, i: int) -> int:
        """Resolved logic level of uio[i] (i in 0..3)."""
        oe = sig_int(self.dut.uio_oe) >> i & 1
        out = sig_int(self.dut.uio_out) >> i & 1
        ext = self.ext_uio[i]
        level = 1
        if oe:
            level = out
        if ext == 0:
            level = 0
        return level

    def gpio_out(self, n: int) -> int:
        """Value the chip currently drives on gpio n (8..19)."""
        if 8 <= n < 16:
            return sig_int(self.dut.uo_out) >> (n - 8) & 1
        return self.uio_line(n - 16)

    async def _drive(self):
        while True:
            ui = self.ui
            for bit, g in self.loopback.items():
                ui = (ui & ~(1 << bit)) | (self.gpio_out(g) << bit)
            self.dut.ui_in.value = ui
            low = 0
            for i in range(4):
                low |= self.uio_line(i) << i
            self.dut.uio_in.value = (
                low | (self.spi_cs << 4) | (self.spi_sck << 5) | (self.spi_mosi << 6)
            )
            await RisingEdge(self.dut.clk)


class BitLoom:
    HALF = 5  # SCK half period in clk cycles

    def __init__(self, dut, pins: Pins):
        self.dut = dut
        self.pins = pins

    # ---- SPI transport ---------------------------------------------------
    async def xfer(self, tx: list[int]) -> list[int]:
        p = self.pins
        dut = self.dut
        rx = []
        p.spi_cs = 0
        p.spi_sck = 0
        await ClockCycles(dut.clk, self.HALF)
        for byte in tx:
            r = 0
            for i in range(7, -1, -1):
                p.spi_mosi = (byte >> i) & 1
                await ClockCycles(dut.clk, self.HALF)
                p.spi_sck = 1
                await ClockCycles(dut.clk, 1)
                r = (r << 1) | (sig_int(dut.uio_out) >> 7 & 1)
                await ClockCycles(dut.clk, self.HALF - 1)
                p.spi_sck = 0
            rx.append(r)
        await ClockCycles(dut.clk, self.HALF)
        p.spi_cs = 1
        p.spi_mosi = 0
        await ClockCycles(dut.clk, self.HALF)
        return rx

    async def write(self, addr: int, data, imem: bool = False):
        if isinstance(data, int):
            data = [data]
        cmd = CMD_WRITE | (CMD_IMEM if imem else 0)
        await self.xfer([cmd, addr & 0xFF, *[d & 0xFF for d in data]])

    async def read(self, addr: int, n: int = 1, imem: bool = False) -> list[int]:
        cmd = CMD_IMEM if imem else 0
        r = await self.xfer([cmd, addr & 0xFF] + [0] * n)
        return r[2:]

    async def read1(self, addr: int) -> int:
        return (await self.read(addr))[0]

    # ---- programs ----------------------------------------------------------
    async def load_program(self, prog: Program, offset: int | None = None) -> int:
        """Write a program to instruction memory. Returns its load address."""
        if offset is None:
            offset = prog.origin or 0
        assert offset + len(prog.words) <= 64, "program does not fit"
        data = []
        for w in prog.words_at(offset):
            data += [w & 0xFF, w >> 8]
        await self.write(offset * 2, data, imem=True)
        return offset

    async def read_imem(self, offset: int, n: int) -> list[int]:
        b = await self.read(offset * 2, 2 * n, imem=True)
        return [b[2 * i] | (b[2 * i + 1] << 8) for i in range(n)]

    # ---- machine configuration --------------------------------------------
    async def configure(self, sm: int, prog: Program | None = None, *, offset: int = 0,
                        div: float = 1.0, out_base=0, out_count=0, set_base=0, set_count=0,
                        in_base=0, sideset_base=0, jmp_pin=0, out_shr=False, in_shr=False,
                        autopull=False, autopush=False, pull_thresh=0, push_thresh=0,
                        wrap_top=None, wrap_bot=None, sideset_count=None,
                        sideset_pindirs=None):
        div_int = int(div)
        div_frac = int(round((div - div_int) * 256))
        if div_frac == 256:
            div_int += 1
            div_frac = 0
        if prog is not None:
            wrap_top = offset + prog.wrap if wrap_top is None else wrap_top
            wrap_bot = offset + prog.wrap_target if wrap_bot is None else wrap_bot
            sideset_count = prog.side_set if sideset_count is None else sideset_count
            sideset_pindirs = prog.side_pindirs if sideset_pindirs is None else sideset_pindirs
        wrap_top = 63 if wrap_top is None else wrap_top
        wrap_bot = 0 if wrap_bot is None else wrap_bot
        sideset_count = sideset_count or 0
        regs = [
            div_int & 0xFF, div_int >> 8, div_frac,
            out_base, out_count, set_base, set_count, in_base,
            (sideset_base & 0x1F) | (sideset_count << 5) | ((1 << 7) if sideset_pindirs else 0),
            jmp_pin, wrap_top, wrap_bot,
            (out_shr << 0) | (in_shr << 1) | (autopull << 2) | (autopush << 3),
            pull_thresh, push_thresh, offset,
        ]
        await self.write(sm_reg(sm, S_DIV_INT_L), regs)

    async def enable(self, mask: int):
        await self.write(R_ENABLE, mask)

    async def restart(self, mask: int):
        await self.write(R_RESTART, mask)

    async def step(self, mask: int):
        await self.write(R_STEP, mask)

    async def set_pc(self, sm: int, pc: int):
        await self.write(sm_reg(sm, S_PC), pc)

    async def exec(self, sm: int, instr: int):
        await self.write(sm_reg(sm, S_EXEC_L), [instr & 0xFF, instr >> 8])

    async def status(self, sm: int) -> int:
        return await self.read1(sm_reg(sm, S_STATUS))

    async def pc(self, sm: int) -> int:
        return await self.read1(sm_reg(sm, S_PC))

    async def x(self, sm: int) -> int:
        lo, hi = await self.read(sm_reg(sm, S_X_L), 2)
        return lo | hi << 8

    async def y(self, sm: int) -> int:
        lo, hi = await self.read(sm_reg(sm, S_Y_L), 2)
        return lo | hi << 8

    # ---- FIFOs ---------------------------------------------------------------
    async def levels(self, sm: int) -> tuple[int, int]:
        v = await self.read1(sm_reg(sm, S_LEVELS))
        return v & 7, (v >> 4) & 7

    async def push(self, sm: int, word: int):
        await self.write(sm_reg(sm, S_TXF_L), [word & 0xFF, (word >> 8) & 0xFF])

    async def push_blocking(self, sm: int, word: int):
        while (await self.levels(sm))[0] >= 4:
            await ClockCycles(self.dut.clk, 20)
        await self.push(sm, word)

    async def pop(self, sm: int) -> int:
        lo, hi = await self.read(sm_reg(sm, S_RXF_L), 2)
        return lo | hi << 8

    async def pop_blocking(self, sm: int, timeout_cycles: int = 200_000) -> int:
        waited = 0
        while (await self.levels(sm))[1] == 0:
            await ClockCycles(self.dut.clk, 50)
            waited += 50
            assert waited < timeout_cycles, "timed out waiting for RX data"
        return await self.pop(sm)
