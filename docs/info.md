<!---
This file is used to generate your project datasheet.
-->

## How it works

BitLoom is a general-purpose protocol emulator: four small programmable I/O
state machines that bit-bang protocols from firmware with cycle-exact timing,
in the spirit of the RP2040 PIO and TI PRU, plus a few things those lack.

Each machine executes one 16-bit instruction per tick of its own fractional
clock divider. The instruction set is built around pins and time: `jmp`,
`wait`, `in`, `out`, `push`/`pull`, `mov`, `set`, and `time`. Every
instruction carries a delay field and optional side-set bits so a program can
drive a pin *and* pad to an exact bit period in a single word. All four
machines share a 64-word instruction memory, 20 GPIOs, eight inter-machine
flags, and each has a 4-deep 16-bit TX and RX FIFO towards the host.

Two features are the main departure from PIO:

* **Deadline scheduling.** Each machine has a free-running 16-bit timebase `T`
  and a deadline register `DL`. `time t+N` / `time dl+N` set the next deadline
  and `wait time` blocks until it arrives. A receiver can sample every bit at
  `start_edge + 12 + 8n` ticks regardless of how many instructions run in
  between, which removes the "count every cycle on every branch" discipline that
  PIO programs need.
* **Edge timestamping.** `in t, 16` shifts the timebase into the ISR, so a
  six-instruction program turns a machine into a logic-analyser front end that
  streams edge timestamps to the host. That is exactly what you want when
  reverse-engineering an unknown protocol before writing an emulator for it.

Other conveniences: single-step and register readback for debugging programs
on real silicon, `exec` of a host-supplied instruction, an optional
three-sample majority glitch filter on the inputs, and open-drain support via
side-set on pin directions (used by the I2C program).

The host talks to the chip over a SPI slave port (mode 0, up to clk/8) on
`uio[7:4]`: command byte (bit 7 write, bit 0 instruction memory), address byte
with auto-increment, then data. All configuration, FIFO access, debug state
and program loading go through that port, so any microcontroller can drive it.

Firmware in the repository implements UART TX and RX (both cycle-counted and
deadline-scheduled variants), SPI master, I2C master with repeated start,
WS2812 LED driving, a Manchester transmitter, and edge-timestamp capture.

## How to test

Connect a host microcontroller (the RP2040 on the Tiny Tapeout demo board
works) to the SPI port: CS_n on `uio[4]`, SCK on `uio[5]`, MOSI on `uio[6]`,
MISO on `uio[7]`. Reading address 0 returns the ID byte `0xB1`.

To send UART at 115200 baud on GPIO 8 (`uo[0]`) with a 50 MHz clock:

1. Assemble `programs/uart_tx.pio` with `tools/bitloom_asm.py` and write the
   words to instruction memory (command `0x81`, address 0).
2. Configure machine 0: divider 54.25 (`DIV_INT=54`, `DIV_FRAC=64`),
   `OUT_BASE=8`, `OUT_COUNT=1`, `SET_BASE=8`, `SET_COUNT=1`, `SHIFTCTRL=1`
   (shift right), `PULL_THRESH=8`, `WRAP_TOP=5`, `WRAP_BOT=0`, `PC=0`.
3. Exec `set pins, 1` (write `0xE001` to the EXEC registers) so TX idles high.
4. Enable machine 0 (write `0x01` to `ENABLE`).
5. Write bytes to the TX FIFO (`TXF_L` then `TXF_H`) and watch `uo[0]`.

`firmware/bitloom_upy.py` is a MicroPython driver that does all of the above
from the demo board; `tools/bitloom_host.py` does the same inside the cocotb
testbench. The `test/` suite exercises every shipped program against Python
protocol models.

## External hardware

Nothing is required beyond a SPI-capable host. For the I2C program, pull-up
resistors on the SDA/SCL pins. A logic analyser or an oscilloscope helps to
watch the generated waveforms.
