# BitLoom instruction set

Every instruction is one 16-bit word and executes in exactly one tick unless
it stalls. A tick is one period of the machine's clock divider
(`clk / (DIV_INT + DIV_FRAC/256)`). Instruction fetch is pipelined, so a
tick is at least two clocks: `DIV_INT = 1` behaves as 2.

```
 15 14 13 | 12 11 10 9 | 8 7 6 5 4 3 2 1 0
  opcode  | delay/side |      operand
```

The 4-bit delay/side-set field is split per machine by `SIDESET.count`
(0 to 3). The top `count` bits are side-set values written to the side-set
pins (or their directions when `SIDESET.pindirs` is set) on every execution
attempt, including stalled ones. The remaining low bits are a delay of extra
ticks inserted after the instruction completes.

## Registers per machine

| Register | Width | Purpose |
|---|---|---|
| PC | 6 | Instruction pointer into the shared 64-word memory |
| X, Y | 16 | Scratch / loop counters |
| ISR | 16 | Input shift register with 0..16 shift count |
| OSR | 16 | Output shift register with 0..16 shift count |
| T | 16 | Timebase: increments once per tick while the machine is enabled |
| DL | 16 | Deadline compared against T by `wait time` |

## Opcodes

### 0 — JMP `jmp [cond] addr`
`operand[8:6]` condition, `operand[5:0]` target.

| cond | mnemonic | jump when |
|---|---|---|
| 0 | | always |
| 1 | `!x` | X == 0 |
| 2 | `x--` | X != 0, then X -= 1 |
| 3 | `!y` | Y == 0 |
| 4 | `y--` | Y != 0, then Y -= 1 |
| 5 | `x!=y` | X != Y |
| 6 | `pin` | the `JMP_PIN` GPIO is high |
| 7 | `!osre` | OSR is not empty (count < pull threshold) |

### 1 — WAIT `wait pol src idx` / `wait time`
`operand[8]` polarity, `operand[7:6]` source, `operand[5:0]` index. Stalls until
the condition holds.

| src | mnemonic | condition |
|---|---|---|
| 0 | `gpio n` | absolute GPIO n == pol |
| 1 | `pin n` | GPIO (IN_BASE + n) mod 20 == pol |
| 2 | `flag n` | flag n == pol; with pol=1 the flag is cleared when observed |
| 3 | `time` | T >= DL (wrap-safe within 32767 ticks) |

### 2 — IN `in src, n`
Shift `n` (1..16) bits from a source into ISR, direction per `SHIFTCTRL.in_shr`.
Sources: `pins` (16 GPIOs starting at IN_BASE, rotating), `x`, `y`, `null`,
`t` (timebase snapshot), `status`, `isr`, `osr`. With autopush enabled the ISR
is pushed to the RX FIFO once its count reaches the push threshold; the IN stalls
if the FIFO is full.

`status` is `{rx_full, rx_empty, tx_full, tx_empty}` in bits 3..0.

### 3 — OUT `out dst, n`
Shift `n` bits out of OSR, direction per `SHIFTCTRL.out_shr`. Destinations:
`pins` (OUT_COUNT pins from OUT_BASE), `x`, `y`, `null`, `pindirs`, `pc`,
`isr` (also sets ISR count to n), `dl`. With autopull enabled an empty OSR is
refilled from the TX FIFO in the same tick; the OUT stalls while the FIFO is empty.

### 4 — PUSH / PULL
`operand[8]` 0 = push, 1 = pull; `operand[7]` if-full / if-empty; `operand[6]` block.

* `push [iffull] [block|noblock]`: write ISR to the RX FIFO and clear it.
  Blocking stalls on a full FIFO; non-blocking drops the word.
* `pull [ifempty] [block|noblock]`: load OSR from the TX FIFO. Blocking stalls
  on an empty FIFO; non-blocking copies X into OSR instead.

### 5 — MOV `mov dst, [op]src`
`operand[8:6]` destination, `operand[5:4]` operation (0 none, 1 `!`/`~` invert,
2 `::` bit-reverse), `operand[2:0]` source (same encoding as IN).
Destinations: `pins`, `x`, `y`, `dl`, `pindirs`, `pc`, `isr`, `osr`.
`nop` assembles to `mov y, y`.

### 6 — SET `set dst, imm`
`operand[8:6]` destination, `operand[5:0]` immediate 0..63.
Destinations: `pins` (SET_COUNT pins from SET_BASE), `x`, `y`, `pindirs`,
`flag n` (set flag imm[2:0]), `clrflag n`, `t` (reset the timebase).

### 7 — TIME `time base+add`
`operand[8:7]` mode, `operand[6:0]` immediate.

| mode | mnemonic | effect |
|---|---|---|
| 0 | `time t+N` | DL = T + N |
| 1 | `time dl+N` | DL = DL + N |
| 2 | `time t+x` | DL = T + X |
| 3 | `time dl+x` | DL = DL + X |

Pattern for jitter-free periodic sampling: `time t+N` once, then in the loop
`wait time` / do the work / `time dl+PERIOD`. The instruction after `wait time`
executes exactly at T = DL + 1, no matter how many instructions ran since the
previous deadline (as long as they fit in the period).

## Pin semantics

* 20 GPIOs: 0..7 are inputs (`ui`), 8..15 outputs (`uo`), 16..19 bidirectional
  (`uio`). Reading an output GPIO returns its driven value.
* All pin bases index a 20-bit vector that wraps, so a field may straddle 19 -> 0.
* When several machines write the same pin in one cycle the highest-numbered
  machine wins. Within one instruction side-set beats OUT / SET / MOV.
* Inputs pass through a two-flop synchroniser (two clk cycles of latency); the
  global `INFILT` bit adds a three-sample majority filter.
