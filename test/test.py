# SPDX-License-Identifier: Apache-2.0
"""BitLoom verification suite (cocotb).

Every protocol test loads a program from ../programs, configures a state
machine over the SPI host port, and checks the pins against an independent
Python reference model (UART / SPI / I2C / WS2812 / Manchester decoders or
slaves).  Data is constrained-random with a fixed seed so runs are repeatable.
"""
import random
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

from bitloom_asm import assemble, assemble_file
from bitloom_host import BitLoom, Pins
from bitloom_regs import *  # noqa: F401,F403

PROG_DIR = Path(__file__).resolve().parent.parent / "programs"
SEED = 20260915


def prog(name):
    return assemble_file(str(PROG_DIR / f"{name}.pio"))


async def setup(dut):
    clock = Clock(dut.clk, 20, unit="ns")   # 50 MHz
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0x10  # CS_n high
    dut.rst_n.value = 0
    pins = Pins(dut)
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    host = BitLoom(dut, pins)
    assert await host.read1(R_ID) == ID_VALUE, "chip ID mismatch"
    return host, pins


# ---------------------------------------------------------------------------
# Reference models
# ---------------------------------------------------------------------------
async def uart_decode(dut, pins, gpio, bit_clks, out, nbytes):
    """Sample a UART frame on gpio at nominal mid-bit times."""
    while len(out) < nbytes:
        while pins.gpio_out(gpio) == 1:
            await RisingEdge(dut.clk)
        elapsed = 0
        v = 0
        for i in range(9):
            target = round(bit_clks * (1.5 + i))
            await ClockCycles(dut.clk, target - elapsed)
            elapsed = target
            bit = pins.gpio_out(gpio)
            if i < 8:
                v |= bit << i
            else:
                assert bit == 1, f"missing stop bit after byte 0x{v:02x}"
        out.append(v)


async def uart_encode(dut, pins, ui_bit, bit_clks, data, stop_bits=1, gap=0):
    """Drive UART frames on ui[ui_bit]."""
    for byte in data:
        bits = [0] + [(byte >> i) & 1 for i in range(8)] + [1] * stop_bits
        elapsed = 0
        for i, b in enumerate(bits):
            pins.ui = (pins.ui & ~(1 << ui_bit)) | (b << ui_bit)
            target = round(bit_clks * (i + 1))
            await ClockCycles(dut.clk, target - elapsed)
            elapsed = target
        if gap:
            await ClockCycles(dut.clk, gap)


async def spi_slave(dut, pins, sck_g, mosi_g, miso_bit, resp, got):
    """Mode-0 SPI slave: sample MOSI on SCK rise, present MISO on SCK fall."""
    bits = [(b >> i) & 1 for b in resp for i in range(7, -1, -1)]
    idx = 0

    def drive(v):
        pins.ui = (pins.ui & ~(1 << miso_bit)) | (v << miso_bit)

    drive(bits[0] if bits else 0)
    prev = 0
    cur = 0
    n = 0
    while True:
        await RisingEdge(dut.clk)
        s = pins.gpio_out(sck_g)
        if s and not prev:
            cur = (cur << 1) | pins.gpio_out(mosi_g)
            n += 1
            if n == 8:
                got.append(cur)
                cur, n = 0, 0
        elif prev and not s:
            idx += 1
            drive(bits[idx] if idx < len(bits) else 0)
        prev = s


class I2CSlave:
    """Register-style I2C slave on uio[0]=SDA, uio[1]=SCL.

    The first byte written after the address sets a pointer; further writes
    store bytes at pointer++; reads return bytes from pointer++.
    """

    def __init__(self, dut, pins, addr):
        self.dut, self.pins, self.addr = dut, pins, addr
        self.mem = bytearray(range(256))
        self.ptr = 0
        self.writes = []
        self.starts = 0
        self.stops = 0
        self.reads = 0
        cocotb.start_soon(self.run())

    def drive(self, bit):
        self.pins.ext_uio[0] = None if bit else 0

    async def run(self):
        p = self.pins
        prev_sda, prev_scl = 1, 1
        state = "idle"
        bitcnt = 0
        shreg = 0
        cur = 0
        first = False
        while True:
            await RisingEdge(self.dut.clk)
            sda, scl = p.uio_line(0), p.uio_line(1)
            if scl and prev_scl:
                if prev_sda and not sda:
                    state, bitcnt, shreg = "addr", 0, 0
                    self.starts += 1
                    self.drive(1)
                elif not prev_sda and sda:
                    state = "idle"
                    self.stops += 1
                    self.drive(1)
            if scl and not prev_scl:  # rising edge: sample
                if state in ("addr", "data"):
                    shreg = (shreg << 1) | sda
                    bitcnt += 1
                elif state == "mack":
                    self.master_ack = sda == 0
            if not scl and prev_scl:  # falling edge: drive
                if state == "addr" and bitcnt == 8:
                    if shreg >> 1 == self.addr:
                        self.drive(0)
                        state = "ack_addr"
                        rw = shreg & 1
                    else:
                        state = "idle"
                elif state == "ack_addr":
                    if rw:
                        cur = self.mem[self.ptr]
                        self.drive(cur >> 7 & 1)
                        state, bitcnt = "tx", 1
                    else:
                        self.drive(1)
                        state, bitcnt, shreg, first = "data", 0, 0, True
                elif state == "data" and bitcnt == 8:
                    if first:
                        self.ptr = shreg
                        first = False
                    else:
                        self.mem[self.ptr] = shreg
                        self.writes.append((self.ptr, shreg))
                        self.ptr = (self.ptr + 1) & 0xFF
                    self.drive(0)
                    state = "ack_data"
                elif state == "ack_data":
                    self.drive(1)
                    state, bitcnt, shreg = "data", 0, 0
                elif state == "tx":
                    if bitcnt < 8:
                        self.drive(cur >> (7 - bitcnt) & 1)
                        bitcnt += 1
                    else:
                        self.drive(1)
                        state = "mack"
                elif state == "mack":
                    self.reads += 1
                    self.ptr = (self.ptr + 1) & 0xFF
                    if self.master_ack:
                        cur = self.mem[self.ptr]
                        self.drive(cur >> 7 & 1)
                        state, bitcnt = "tx", 1
                    else:
                        state = "idle"
            prev_sda, prev_scl = sda, scl


def i2c_word(data=0, start=False, stop=False, read=False, nack=False):
    return (start << 15) | (nack << 14) | (read << 13) | ((data & 0xFF) << 1) | int(stop)


async def edge_log(dut, pins, gpio, log, stop):
    """Record (cycle, level) for every transition of gpio."""
    prev = pins.gpio_out(gpio)
    cyc = 0
    while not stop[0]:
        await RisingEdge(dut.clk)
        cyc += 1
        v = pins.gpio_out(gpio)
        if v != prev:
            log.append((cyc, v))
            prev = v


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
@cocotb.test(timeout_time=30, timeout_unit="ms")
async def test_host_port(dut):
    """SPI host port: ID, register write/readback, instruction memory."""
    host, pins = await setup(dut)
    assert await host.read1(R_VERSION) == 1
    assert await host.read1(R_NUM_SM) == 4
    assert await host.read1(R_IMEM_SIZE) == 64

    await host.write(sm_reg(2, S_DIV_INT_L), [0x34, 0x12, 0x80, 0x1F])
    assert await host.read(sm_reg(2, S_DIV_INT_L), 4) == [0x34, 0x12, 0x80, 0x1F]
    assert await host.read(sm_reg(1, S_DIV_INT_L), 2) == [1, 0], "other machine untouched"

    rng = random.Random(SEED)
    words = [rng.randrange(1 << 16) for _ in range(64)]
    data = []
    for w in words:
        data += [w & 0xFF, w >> 8]
    await host.write(0, data, imem=True)
    assert await host.read_imem(0, 64) == words
    assert await host.read_imem(17, 3) == words[17:20]

    # GPIO input readback (raw pins through the synchroniser)
    pins.ui = 0xA5
    await ClockCycles(dut.clk, 5)
    assert await host.read1(R_GPIO_IN0) == 0xA5


@cocotb.test(timeout_time=30, timeout_unit="ms")
async def test_uart_tx(dut):
    """UART transmitter at integer and fractional dividers, random payload."""
    host, pins = await setup(dut)
    rng = random.Random(SEED + 1)
    p = prog("uart_tx")
    off = await host.load_program(p, 0)
    for div in (1.0, 3.0, 2.5, 4.75):
        data = [rng.randrange(256) for _ in range(6)]
        await host.enable(0)
        await host.configure(0, p, offset=off, div=div, out_base=8, out_count=1,
                             set_base=8, set_count=1, out_shr=True, pull_thresh=8)
        await host.restart(1)
        await host.exec(0, assemble("set pins, 1").words[0])   # idle high
        await host.enable(1)
        await ClockCycles(dut.clk, 40)
        got = []
        cocotb.start_soon(uart_decode(dut, pins, 8, 8 * div, got, len(data)))
        for b in data:
            await host.push_blocking(0, b)
        for _ in range(2000):
            if len(got) == len(data):
                break
            await ClockCycles(dut.clk, 50)
        assert got == data, f"div={div}: got {got} expected {data}"


@cocotb.test(timeout_time=30, timeout_unit="ms")
async def test_uart_rx(dut):
    """Both receiver variants (cycle-counted and deadline-scheduled) decode random frames."""
    host, pins = await setup(dut)
    rng = random.Random(SEED + 2)
    for name in ("uart_rx", "uart_rx_time"):
        p = prog(name)
        off = await host.load_program(p, 0)
        for div in (2.0, 3.0, 2.5):
            data = [rng.randrange(256) for _ in range(5)]
            pins.ui |= 1  # idle high
            await host.enable(0)
            await host.configure(1, p, offset=off, div=div, in_base=0, jmp_pin=0, in_shr=True)
            await host.restart(2)
            await host.enable(2)
            await ClockCycles(dut.clk, 40)
            gen = cocotb.start_soon(uart_encode(dut, pins, 0, 8 * div, data, gap=30))
            got = [(await host.pop_blocking(1)) >> 8 for _ in data]
            await gen
            assert got == data, f"{name} div={div}: got {got} expected {data}"


@cocotb.test(timeout_time=30, timeout_unit="ms")
async def test_uart_rx_framing_error(dut):
    """A frame with a bad stop bit is dropped and reception resynchronises."""
    host, pins = await setup(dut)
    p = prog("uart_rx")
    await host.load_program(p, 0)
    pins.ui |= 1
    await host.configure(1, p, offset=0, div=2, in_base=0, jmp_pin=0, in_shr=True)
    await host.restart(2)
    await host.enable(2)
    await ClockCycles(dut.clk, 40)
    bit = 16
    # Break: start bit followed by all zeros including the stop position.
    for i in range(10):
        pins.ui &= ~1
        await ClockCycles(dut.clk, bit)
    pins.ui |= 1
    await ClockCycles(dut.clk, 3 * bit)
    await uart_encode(dut, pins, 0, bit, [0x5A])
    assert (await host.pop_blocking(1)) >> 8 == 0x5A
    assert (await host.levels(1))[1] == 0, "framing error must not push a byte"


@cocotb.test(timeout_time=30, timeout_unit="ms")
async def test_uart_loopback(dut):
    """Machine 0 transmits, machine 1 receives through an external loopback wire."""
    host, pins = await setup(dut)
    rng = random.Random(SEED + 3)
    tx = prog("uart_tx")
    rx = prog("uart_rx_time")
    tx_off = await host.load_program(tx, 0)
    rx_off = await host.load_program(rx, 8)
    pins.loopback = {0: 8}   # ui[0] <- gpio 8 (uo[0])
    div = 2.0
    await host.configure(0, tx, offset=tx_off, div=div, out_base=8, out_count=1,
                         set_base=8, set_count=1, out_shr=True, pull_thresh=8)
    await host.configure(1, rx, offset=rx_off, div=div, in_base=0, jmp_pin=0, in_shr=True)
    await host.restart(3)
    await host.exec(0, assemble("set pins, 1").words[0])
    await ClockCycles(dut.clk, 20)
    await host.enable(3)
    data = [rng.randrange(256) for _ in range(12)]
    got = []
    for b in data:
        await host.push_blocking(0, b)
        if (await host.levels(1))[1]:
            got.append((await host.pop(1)) >> 8)
    while len(got) < len(data):
        got.append((await host.pop_blocking(1)) >> 8)
    assert got == data


@cocotb.test(timeout_time=30, timeout_unit="ms")
async def test_spi_master(dut):
    """Full-duplex SPI master against a Python slave, chip select via exec."""
    host, pins = await setup(dut)
    rng = random.Random(SEED + 4)
    p = prog("spi_master")
    off = await host.load_program(p, 0)
    # MOSI = gpio 9 (uo[1]), SCK = gpio 10 (uo[2]), CS = gpio 11 (uo[3]), MISO = gpio 1 (ui[1])
    await host.configure(0, p, offset=off, div=4, out_base=9, out_count=1, in_base=1,
                         sideset_base=10, set_base=11, set_count=1,
                         autopull=True, autopush=True, pull_thresh=8, push_thresh=8)
    await host.exec(0, assemble("set pins, 1").words[0])   # CS idle high
    await host.restart(1)
    await host.enable(1)
    tx_bytes = [rng.randrange(256) for _ in range(8)]
    rx_bytes = [rng.randrange(256) for _ in range(8)]
    got = []
    cocotb.start_soon(spi_slave(dut, pins, 10, 9, 1, rx_bytes, got))
    await host.exec(0, assemble("set pins, 0").words[0])   # assert CS
    received = []
    for b in tx_bytes:
        await host.push_blocking(0, b << 8)
        if (await host.levels(0))[1]:
            received.append(await host.pop(0))
    while len(received) < len(tx_bytes):
        received.append(await host.pop_blocking(0))
    await ClockCycles(dut.clk, 50)
    await host.exec(0, assemble("set pins, 1").words[0])   # release CS
    assert got == tx_bytes, f"slave saw {got}, expected {tx_bytes}"
    assert received == rx_bytes, f"master read {received}, expected {rx_bytes}"
    assert pins.gpio_out(11) == 1


@cocotb.test(timeout_time=30, timeout_unit="ms")
async def test_i2c_master(dut):
    """I2C write and repeated-start read against a Python register slave."""
    host, pins = await setup(dut)
    rng = random.Random(SEED + 5)
    p = prog("i2c_master")
    off = await host.load_program(p, 0)
    addr = 0x3C
    slave = I2CSlave(dut, pins, addr)
    await host.configure(0, p, offset=off, div=4, out_base=16, out_count=1, set_base=16,
                         set_count=1, in_base=16, sideset_base=17)
    await host.restart(1)
    await host.enable(1)

    ptr = 0x10
    payload = [rng.randrange(256) for _ in range(3)]
    seq = [i2c_word(addr << 1, start=True), i2c_word(ptr)]
    seq += [i2c_word(b) for b in payload[:-1]] + [i2c_word(payload[-1], stop=True)]
    acks = []
    for w in seq:
        await host.push_blocking(0, w)
        acks.append(await host.pop_blocking(0))
    assert all(a & 1 == 0 for a in acks), f"expected ACKs, got {acks}"
    assert slave.writes == [(ptr + i, b) for i, b in enumerate(payload)]
    assert slave.stops == 1

    # Read back with a repeated start.
    seq = [i2c_word(addr << 1, start=True), i2c_word(ptr),
           i2c_word((addr << 1) | 1, start=True),
           i2c_word(read=True), i2c_word(read=True), i2c_word(read=True, nack=True, stop=True)]
    res = []
    for w in seq:
        await host.push_blocking(0, w)
        res.append(await host.pop_blocking(0))
    assert res[:3] == [0, 0, 0], f"address/pointer not acked: {res}"
    assert [r & 0xFF for r in res[3:]] == payload, f"read {res[3:]} expected {payload}"
    assert slave.stops == 2 and slave.starts == 3

    # A non-existent address must NACK.
    await host.push(0, i2c_word(0x51 << 1, start=True, stop=True))
    assert (await host.pop_blocking(0)) & 1 == 1
    await ClockCycles(dut.clk, 100)
    assert pins.uio_line(0) == 1 and pins.uio_line(1) == 1, "bus not released"


@cocotb.test(timeout_time=30, timeout_unit="ms")
async def test_edge_capture(dut):
    """Timestamps of both edges are captured exactly, in divided-clock ticks."""
    host, pins = await setup(dut)
    rng = random.Random(SEED + 6)
    p = prog("edge_capture")
    off = await host.load_program(p, 0)
    div = 4
    await host.configure(2, p, offset=off, div=div, in_base=2)
    await host.restart(4)
    await host.enable(4)
    await ClockCycles(dut.clk, 40)
    widths = [rng.randrange(5, 60) for _ in range(6)]
    gap = 300  # ticks; long enough for the host to drain two words

    async def gen():
        for w in widths:
            pins.ui |= 1 << 2
            await ClockCycles(dut.clk, w * div)
            pins.ui &= ~(1 << 2)
            await ClockCycles(dut.clk, gap * div)

    cocotb.start_soon(gen())
    stamps = [await host.pop_blocking(2) for _ in range(2 * len(widths))]
    highs = [(stamps[2 * i + 1] - stamps[2 * i]) & 0xFFFF for i in range(len(widths))]
    lows = [(stamps[2 * i + 2] - stamps[2 * i + 1]) & 0xFFFF for i in range(len(widths) - 1)]
    assert highs == widths, f"high widths {highs} != {widths}"
    assert all(l == gap for l in lows), f"low widths {lows} != {gap}"


@cocotb.test(timeout_time=30, timeout_unit="ms")
async def test_ws2812(dut):
    """WS2812 bit timing: 2/7 ticks high for 0/1, 10-tick period."""
    host, pins = await setup(dut)
    rng = random.Random(SEED + 7)
    p = prog("ws2812")
    off = await host.load_program(p, 0)
    await host.configure(3, p, offset=off, div=1, sideset_base=12, autopull=True, pull_thresh=8)
    await host.restart(8)
    log, stop = [], [False]
    cocotb.start_soon(edge_log(dut, pins, 12, log, stop))
    data = [rng.randrange(256) for _ in range(3)]
    for b in data:            # prefill: the 80-cycle bytes outrun the SPI host
        await host.push(3, b << 8)
    await host.enable(8)
    await ClockCycles(dut.clk, 400)
    stop[0] = True
    rises = [t for t, v in log if v == 1]
    falls = [t for t, v in log if v == 0]
    assert len(rises) == 24 and len(falls) == 24, f"{len(rises)} rises, {len(falls)} falls"
    bits = []
    for r, f in zip(rises, falls):
        assert f - r in (2, 7), f"bad high width {f - r}"
        bits.append(1 if f - r == 7 else 0)
    periods = {b - a for a, b in zip(rises, rises[1:])}
    assert periods == {10}, f"unexpected bit periods {periods}"
    expected = [(b >> i) & 1 for b in data for i in range(7, -1, -1)]
    assert bits == expected


@cocotb.test(timeout_time=30, timeout_unit="ms")
async def test_manchester_tx(dut):
    """Deadline-scheduled Manchester stream keeps every edge on the half-bit grid."""
    host, pins = await setup(dut)
    rng = random.Random(SEED + 8)
    p = prog("manchester_tx")
    off = await host.load_program(p, 0)
    half = 64     # ticks; a byte takes 1024 cycles, slower than a SPI push
    div = 1
    await host.configure(2, p, offset=off, div=div, out_base=11, out_count=1,
                         out_shr=True, autopull=True, pull_thresh=8)
    await host.restart(4)
    log, stop = [], [False]
    cocotb.start_soon(edge_log(dut, pins, 11, log, stop))
    data = [0x00] + [rng.randrange(256) for _ in range(4)]   # 0x00 gives a clean anchor edge
    await host.push(2, half)
    for b in data[:3]:
        await host.push(2, b)          # prefill the 4-deep FIFO
    await host.enable(4)
    for b in data[3:]:
        await host.push_blocking(2, b)  # keep it fed while it streams
    await ClockCycles(dut.clk, half * div * 2 * 8 * len(data) + 200)
    stop[0] = True
    anchor = log[0][0]
    assert log[0][1] == 1
    H = half * div
    for t, _ in log:
        assert (t - anchor) % H == 0, f"edge at {t} is off the half-bit grid ({anchor}, {H})"
    nbits = 8 * len(data)
    # Reconstruct: the level during the second half of each bit is the bit value.
    levels = []
    cur = 0
    li = 0
    for b in range(nbits):
        sample_t = anchor + (2 * b + 1) * H + H // 2
        while li < len(log) and log[li][0] <= sample_t:
            cur = log[li][1]
            li += 1
        levels.append(cur)
    expected = [(b >> i) & 1 for b in data for i in range(8)]
    assert levels == expected, f"decoded {levels} expected {expected}"
    last_edge = log[-1][0]
    assert last_edge - anchor == (2 * nbits - 1) * H, "stream length wrong"


@cocotb.test(timeout_time=30, timeout_unit="ms")
async def test_debug_exec_step(dut):
    """Host-side debugging: exec, single-step with register readback, wrap."""
    host, pins = await setup(dut)
    p = assemble("""
        set x, 5
        set y, 9
        set x, 1
        .wrap_target
        loop: jmp x-- loop
        set y, 0
        .wrap
    """)
    off = await host.load_program(p, 20)
    await host.configure(3, p, offset=off, set_base=13, set_count=1)
    await host.restart(8)
    assert await host.pc(3) == 20
    await host.exec(3, assemble("set pins, 1").words[0])
    await ClockCycles(dut.clk, 5)
    assert pins.gpio_out(13) == 1
    assert await host.pc(3) == 20, "exec must not move PC"
    await host.step(8)
    assert await host.pc(3) == 21 and await host.x(3) == 5
    await host.step(8)
    assert await host.pc(3) == 22 and await host.y(3) == 9
    await host.step(8)
    await host.step(8)          # jmp x-- with x=1: taken, x -> 0
    assert await host.pc(3) == 23 and await host.x(3) == 0
    await host.step(8)          # x == 0: fall through
    assert await host.pc(3) == 24
    await host.step(8)          # set y,0 then wrap to 23
    assert await host.pc(3) == 23 and await host.y(3) == 0
    st = await host.status(3)
    assert st & ST_ENABLED == 0 and st & ST_TX_EMPTY and st & ST_RX_EMPTY


@cocotb.test(timeout_time=30, timeout_unit="ms")
async def test_flags_and_filter(dut):
    """Flag handshake between two machines, and the input glitch filter."""
    host, pins = await setup(dut)
    a = assemble("""
        set flag 3
        wait 0 flag 3
        set pins, 1
        halt: jmp halt
    """)
    b = assemble("""
        wait 1 gpio 3
        wait 1 flag 3
        set pins, 1
        halt: jmp halt
    """)
    oa = await host.load_program(a, 0)
    ob = await host.load_program(b, 10)
    await host.configure(0, a, offset=oa, set_base=14, set_count=1)
    await host.configure(1, b, offset=ob, set_base=15, set_count=1)
    await host.write(R_INFILT, 1)
    await host.restart(3)
    await host.enable(3)
    await ClockCycles(dut.clk, 30)
    assert await host.read1(R_FLAGS) == 0x08, "flag 3 should be pending"
    assert pins.gpio_out(15) == 0
    # A one-cycle glitch on gpio 3 must be filtered out.
    pins.ui |= 1 << 3
    await ClockCycles(dut.clk, 1)
    pins.ui &= ~(1 << 3)
    await ClockCycles(dut.clk, 30)
    assert pins.gpio_out(15) == 0 and pins.gpio_out(14) == 0
    pins.ui |= 1 << 3
    await ClockCycles(dut.clk, 30)
    assert pins.gpio_out(15) == 1 and pins.gpio_out(14) == 1
    assert await host.read1(R_FLAGS) == 0, "wait 1 flag must clear the flag"
