"""MicroPython driver for BitLoom on the Tiny Tapeout demo board.

Untested on hardware until the chip returns from fabrication; it mirrors
tools/bitloom_host.py, which is exercised by the cocotb suite.

Usage on the demo board (RP2040 MicroPython with the tt library):

    from ttboard.demoboard import DemoBoard
    tt = DemoBoard.get()
    tt.shuttle.tt_um_sheehanmunim_bitloom.enable()
    bl = BitLoom(tt)
    assert bl.read(0x00) == 0xB1
    bl.load_program(UART_TX_WORDS)
    bl.configure(0, div=54.25, out_base=8, out_count=1, set_base=8, set_count=1,
                 out_shr=True, pull_thresh=8, wrap_top=5, wrap_bot=0, pc=0)
    bl.exec(0, 0xE001)          # set pins, 1
    bl.enable(1)
    for b in b"hello\\n": bl.push(0, b)

Words come from `python3 tools/bitloom_asm.py programs/uart_tx.pio --json`.
"""
import time

CMD_WRITE = 0x80
CMD_IMEM = 0x01
SM_BASE = 0x20
SM_STRIDE = 0x20

# uio bit positions of the host port
CS, SCK, MOSI, MISO = 4, 5, 6, 7


class BitLoom:
    def __init__(self, tt):
        self.tt = tt
        self.tt.uio_oe_pico.value = (1 << CS) | (1 << SCK) | (1 << MOSI)  # RP2040 drives these
        self._uio = 1 << CS
        self._out(self._uio)

    def _out(self, v):
        self._uio = v
        self.tt.uio_in.value = v

    def xfer(self, tx):
        rx = []
        self._out(self._uio & ~(1 << CS))
        for byte in tx:
            r = 0
            for i in range(7, -1, -1):
                v = (self._uio & ~((1 << MOSI) | (1 << SCK))) | (((byte >> i) & 1) << MOSI)
                self._out(v)
                self._out(v | (1 << SCK))
                r = (r << 1) | ((int(self.tt.uio_out.value) >> MISO) & 1)
            self._out(v)
            rx.append(r)
        self._out(self._uio | (1 << CS))
        return rx

    def write(self, addr, data, imem=False):
        if isinstance(data, int):
            data = [data]
        self.xfer([CMD_WRITE | (CMD_IMEM if imem else 0), addr & 0xFF] + [d & 0xFF for d in data])

    def read(self, addr, n=1, imem=False):
        r = self.xfer([CMD_IMEM if imem else 0, addr & 0xFF] + [0] * n)
        return r[2] if n == 1 else r[2:]

    def load_program(self, words, offset=0):
        data = []
        for w in words:
            data += [w & 0xFF, w >> 8]
        self.write(offset * 2, data, imem=True)
        return offset

    def configure(self, sm, div=1.0, out_base=0, out_count=0, set_base=0, set_count=0,
                  in_base=0, sideset_base=0, sideset_count=0, sideset_pindirs=False,
                  jmp_pin=0, wrap_top=63, wrap_bot=0, out_shr=False, in_shr=False,
                  autopull=False, autopush=False, pull_thresh=0, push_thresh=0, pc=0):
        di = int(div)
        df = int((div - di) * 256 + 0.5)
        regs = [di & 0xFF, di >> 8, df, out_base, out_count, set_base, set_count, in_base,
                (sideset_base & 0x1F) | (sideset_count << 5) | (0x80 if sideset_pindirs else 0),
                jmp_pin, wrap_top, wrap_bot,
                (out_shr << 0) | (in_shr << 1) | (autopull << 2) | (autopush << 3),
                pull_thresh, push_thresh, pc]
        self.write(SM_BASE + SM_STRIDE * sm, regs)

    def enable(self, mask):
        self.write(0x04, mask)

    def restart(self, mask):
        self.write(0x05, mask)

    def step(self, mask):
        self.write(0x06, mask)

    def exec(self, sm, instr):
        self.write(SM_BASE + SM_STRIDE * sm + 0x1C, [instr & 0xFF, instr >> 8])

    def status(self, sm):
        return self.read(SM_BASE + SM_STRIDE * sm + 0x10)

    def levels(self, sm):
        v = self.read(SM_BASE + SM_STRIDE * sm + 0x17)
        return v & 7, (v >> 4) & 7

    def push(self, sm, word, timeout_ms=100):
        t0 = time.ticks_ms()
        while self.levels(sm)[0] >= 4:
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                raise OSError("TX FIFO full")
        self.write(SM_BASE + SM_STRIDE * sm + 0x18, [word & 0xFF, (word >> 8) & 0xFF])

    def pop(self, sm, timeout_ms=100):
        t0 = time.ticks_ms()
        while self.levels(sm)[1] == 0:
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                raise OSError("RX FIFO empty")
        lo, hi = self.read(SM_BASE + SM_STRIDE * sm + 0x1A, 2)
        return lo | (hi << 8)
