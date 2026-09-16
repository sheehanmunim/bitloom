# Host register map

SPI mode 0, MSB first, CS_n active low, SCK at most clk/8. Every frame is:

```
byte 0  command   bit 7: 1 = write, 0 = read     bit 0: 1 = instruction memory, 0 = registers
byte 1  address   auto-increments after each data byte
byte 2+ data
```

Instruction memory is byte addressed: word `w` occupies bytes `2w` (low) and
`2w + 1` (high). A word is committed when its high byte is written.

## Global registers

| Addr | Name | R/W | Bits |
|---|---|---|---|
| 0x00 | ID | R | 0xB1 |
| 0x01 | VERSION | R | 0x01 |
| 0x02 | NUM_SM | R | 4 |
| 0x03 | IMEM_SIZE | R | 64 |
| 0x04 | ENABLE | RW | bit n enables machine n |
| 0x05 | RESTART | W | bit n restarts machine n: clears X, Y, ISR, OSR, T, DL, delay, FIFOs, pending step/exec. PC is kept. |
| 0x06 | STEP | W | bit n executes one instruction on machine n (ignores pending delay) |
| 0x07 | FLAGS | RW | read: flag bits; write: clear mask |
| 0x08 | GPIO_IN0 | R | GPIO 7..0 |
| 0x09 | GPIO_IN1 | R | GPIO 15..8 (driven values) |
| 0x0A | GPIO_IN2 | R | GPIO 19..16 |
| 0x0C | GPIO_OUT1 | R | output latch, GPIO 15..8 |
| 0x0D | GPIO_OUT2 | R | output latch, GPIO 19..16 |
| 0x0E | GPIO_OE2 | R | direction latch, GPIO 19..16 |
| 0x0F | INFILT | RW | bit 0: enable 3-sample majority filter on inputs |

## Per-machine registers

Base address `0x20 + 0x20 * n` for machine `n`.

| Off | Name | R/W | Bits |
|---|---|---|---|
| 0x00 | DIV_INT_L | RW | divider integer part, low byte (0 means 65536, 1 behaves as 2) |
| 0x01 | DIV_INT_H | RW | divider integer part, high byte |
| 0x02 | DIV_FRAC | RW | divider fraction / 256 |
| 0x03 | OUT_BASE | RW | first GPIO for `out pins` / `out pindirs` / `mov pins` |
| 0x04 | OUT_COUNT | RW | number of pins written by those (0..16) |
| 0x05 | SET_BASE | RW | first GPIO for `set pins` / `set pindirs` |
| 0x06 | SET_COUNT | RW | 0..6 |
| 0x07 | IN_BASE | RW | GPIO that becomes bit 0 of `in pins` / `wait pin` |
| 0x08 | SIDESET | RW | [4:0] base GPIO, [6:5] bit count, [7] side-set drives pin directions |
| 0x09 | JMP_PIN | RW | GPIO tested by `jmp pin` |
| 0x0A | WRAP_TOP | RW | after executing this address, continue at WRAP_BOT |
| 0x0B | WRAP_BOT | RW | |
| 0x0C | SHIFTCTRL | RW | [0] OUT shifts right, [1] IN shifts right, [2] autopull, [3] autopush |
| 0x0D | PULL_THRESH | RW | OSR count at which it is empty (0 = 16) |
| 0x0E | PUSH_THRESH | RW | ISR count at which it autopushes (0 = 16) |
| 0x0F | PC | RW | program counter |
| 0x10 | STATUS | R | [0] enabled [1] stalled [2] delaying [3] tx_empty [4] tx_full [5] rx_empty [6] rx_full [7] step/exec pending |
| 0x11 | X_L | R | |
| 0x12 | X_H | R | |
| 0x13 | Y_L | R | |
| 0x14 | Y_H | R | |
| 0x15 | T_L | R | timebase |
| 0x16 | T_H | R | |
| 0x17 | LEVELS | R | [2:0] TX FIFO level, [6:4] RX FIFO level |
| 0x18 | TXF_L | W | low byte of the next TX word |
| 0x19 | TXF_H | W | high byte; writing pushes the word |
| 0x1A | RXF_L | R | reading pops the RX FIFO and latches the high byte |
| 0x1B | RXF_H | R | latched high byte |
| 0x1C | EXEC_L | W | low byte of an instruction |
| 0x1D | EXEC_H | W | high byte; writing executes the instruction once on the next tick |

Configuration registers are meant to be written while the machine is disabled.
