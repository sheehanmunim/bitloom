# BitLoom — a programmable protocol emulator ASIC

![test](../../workflows/test/badge.svg) ![gds](../../workflows/gds/badge.svg) ![docs](../../workflows/docs/badge.svg)

BitLoom is an open-source, general-purpose protocol emulator chip built for the
[Jane Street protocol emulator ASIC competition](https://blog.janestreet.com/protocol-emulator-asic-competition/)
on IHP's 130 nm CMOS5L process via [Tiny Tapeout](https://tinytapeout.com)
(6x4 tiles). Four tiny programmable I/O state machines bit-bang protocols from
firmware with cycle-exact timing, so new protocols can be added after
fabrication.

Think RP2040 PIO, with these differences:

| | PIO | BitLoom |
|---|---|---|
| Timing model | count cycles on every path | per-machine timebase + deadline register (`time` / `wait time`) |
| Reverse engineering | none | `in t, 16` streams edge timestamps to the host |
| Debug | none on silicon | single-step, `exec`, PC/X/Y/T readback, status |
| Host interface | AHB in the MCU | 4-wire SPI slave, any MCU can drive it |
| Input conditioning | 2-flop sync | 2-flop sync + optional 3-sample majority filter |
| Instruction memory | 32 × 16 shared | 64 × 16 shared |
| Machines | 4 | 3 (4 fit in area; routing closure pending) |

Shipped firmware (`programs/`): UART TX, UART RX (cycle-counted and
deadline-scheduled), SPI master, I2C master with repeated start and NACK
handling, WS2812 LED driver, Manchester transmitter, edge-timestamp capture.

## Repository layout

```
src/            RTL: project.v (top + host register file), bitloom_sm.v (state machine),
                bitloom_fifo.v, bitloom_spi.v, bitloom_defs.vh
programs/       protocol firmware in BitLoom assembly (.pio)
tools/          bitloom_asm.py (assembler), bitloom_host.py (cocotb host driver),
                bitloom_regs.py (register map)
firmware/       bitloom_upy.py: MicroPython driver for the Tiny Tapeout demo board
test/           cocotb test suite with Python UART / SPI / I2C / WS2812 / Manchester models
formal/         bounded formal checks (Yosys SAT)
synth/          quick technology-independent area estimate
docs/           info.md (datasheet), isa.md (instruction set), registers.md
```

## Architecture in one page

* **Pins.** 20 GPIOs as seen by programs: 0–7 inputs (`ui`), 8–15 outputs
  (`uo`), 16–19 bidirectional (`uio[3:0]`) with per-pin direction under
  program control. `uio[7:4]` is the host SPI port.
* **Machines.** Four identical state machines, each with PC, X, Y, ISR, OSR,
  timebase T, deadline DL, a 16.8 fractional clock divider, and 4-deep TX/RX
  FIFOs. One instruction per tick, deterministic stalls on `wait`, blocking
  `push`/`pull`, and autopull/autopush.
* **ISA.** Eight opcodes in 16 bits: `jmp wait in out push/pull mov set time`.
  Four delay/side-set bits per instruction. See [docs/isa.md](docs/isa.md).
* **Host.** SPI mode 0, MSB first, `cmd addr data...` frames with auto
  increment; register space and instruction-memory space. See
  [docs/registers.md](docs/registers.md).

### Why deadlines

PIO programs hit timing by making every path through the program the same
length. That is fine for UART and painful for anything with branches. BitLoom
keeps PIO's delay field but adds an absolute-time primitive:

```
    wait 0 pin 0        ; start bit edge seen at tick e
    time t+10           ; DL = e + 11
    set x, 7
bitloop:
    wait time           ; block until T >= DL
    in pins, 1          ; executes at exactly DL + 1
    time dl+8           ; next sample one bit period later
    jmp x-- bitloop
```

The sample points are `e + 12 + 8n` no matter what runs between them. The
Manchester transmitter uses the same trick to keep every edge on the half-bit
grid even while the OSR is being refilled from the FIFO.

## Building and testing

Requirements: Icarus Verilog, Python 3.11+, `pip install -r test/requirements.txt`
(cocotb, pytest). Yosys for the area estimate and formal check.

```sh
cd test && make                         # full cocotb suite, writes results.xml and tb.fst
COCOTB_TEST_FILTER=test_i2c_master make # a single test
yosys -q -s synth/area.ys               # generic cell count
yosys -q -s formal/check.ys             # bounded proof of the FIFO invariants
python3 tools/bitloom_asm.py programs/i2c_master.pio   # assemble a program
```

The GitHub Actions workflows run the test suite and the full Tiny Tapeout
LibreLane flow (synthesis, place and route, precheck, gate-level simulation)
on every push.

## Verification approach

* **Reference models, not golden waveforms.** Every protocol test drives the
  chip through the real SPI host port and checks the pins against an
  independent Python implementation: a UART decoder and encoder, a mode-0 SPI
  slave, a register-style I2C slave (address match, ACK/NACK, repeated start,
  STOP detection), WS2812 pulse-width decoding, and Manchester grid checking.
* **Constrained random.** Payloads and, for UART, dividers (integer and
  fractional) are drawn from a seeded RNG so failures reproduce.
* **Negative tests.** Framing errors, NACK from an absent I2C address, bus
  release after STOP, glitches shorter than the majority filter window.
* **Formal.** The FIFO carries `assert`s under `` `ifdef FORMAL `` proved with
  Yosys' SAT engine for 24 cycles from reset (`formal/check.ys`).
* **Host-observable state.** Single-step plus PC/X/Y/T readback is tested and is
  the same path a user would take to debug firmware on the fabricated chip.

## AI-assisted design

The RTL, firmware, assembler, models and this documentation were written with
Claude (Anthropic) as the pair programmer, directed and reviewed by Sheehan
Munim. The verification strategy leans on independent Python reference models
precisely because generated RTL needs an oracle that was not generated from
the same description.

## Status and limits

* Instruction memory is flip-flop based (64 words). SRAM macros would shrink it
  and are the first thing to try if the 8x4 allocation opens up.
* Host SPI clock must be at most `clk / 8` (6.25 MHz at 50 MHz).
* Not yet implemented: low-speed USB and 10 Mbit Ethernet firmware. USB needs a
  12 MHz-multiple tick and NRZI/bit-stuffing in firmware; 10BASE-T Manchester
  needs a 20 MHz tick, i.e. a 40 MHz or 60 MHz system clock. Both fit the ISA.
* Not taped out yet. The CI GDS job is the source of truth for area and timing.
* The first hardening attempt with four machines synthesised to 28.1K cells, 0.389 mm² (49% utilisation) and detailed routing was still converging (24 violations left) when it hit the six-hour CI limit, on the three routing layers the Tiny Tapeout flow allows. The current build uses three machines (0.316 mm², 21.9K cells) and a lower placement density. `N_SM` in `src/project.v` is the knob.

## License

Apache-2.0. See [LICENSE](LICENSE).
