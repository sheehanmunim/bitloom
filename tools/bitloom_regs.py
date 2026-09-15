"""BitLoom host-visible register map (shared by the drivers and the tests)."""

ID_VALUE = 0xB1

# Global registers
R_ID = 0x00
R_VERSION = 0x01
R_NUM_SM = 0x02
R_IMEM_SIZE = 0x03
R_ENABLE = 0x04
R_RESTART = 0x05       # write-only, bit per machine
R_STEP = 0x06          # write-only, bit per machine
R_FLAGS = 0x07         # read: flags, write: clear mask
R_GPIO_IN0 = 0x08
R_GPIO_IN1 = 0x09
R_GPIO_IN2 = 0x0A
R_GPIO_OUT1 = 0x0C
R_GPIO_OUT2 = 0x0D
R_GPIO_OE2 = 0x0E
R_INFILT = 0x0F

# Per-machine registers: base = SM_BASE + SM_STRIDE * sm
SM_BASE = 0x20
SM_STRIDE = 0x20
S_DIV_INT_L = 0x00
S_DIV_INT_H = 0x01
S_DIV_FRAC = 0x02
S_OUT_BASE = 0x03
S_OUT_COUNT = 0x04
S_SET_BASE = 0x05
S_SET_COUNT = 0x06
S_IN_BASE = 0x07
S_SIDESET = 0x08       # [4:0] base, [6:5] count, [7] pindirs
S_JMP_PIN = 0x09
S_WRAP_TOP = 0x0A
S_WRAP_BOT = 0x0B
S_SHIFTCTRL = 0x0C     # [0] out_shr, [1] in_shr, [2] autopull, [3] autopush
S_PULL_THRESH = 0x0D
S_PUSH_THRESH = 0x0E
S_PC = 0x0F
S_STATUS = 0x10        # [0] en [1] stalled [2] delaying [3] tx_empty [4] tx_full [5] rx_empty [6] rx_full [7] pending
S_X_L = 0x11
S_X_H = 0x12
S_Y_L = 0x13
S_Y_H = 0x14
S_T_L = 0x15
S_T_H = 0x16
S_LEVELS = 0x17        # [2:0] tx level, [6:4] rx level
S_TXF_L = 0x18
S_TXF_H = 0x19         # writing pushes {H, L}
S_RXF_L = 0x1A         # reading pops and latches the high byte
S_RXF_H = 0x1B
S_EXEC_L = 0x1C
S_EXEC_H = 0x1D        # writing executes {H, L} once

# Status bits
ST_ENABLED = 1 << 0
ST_STALLED = 1 << 1
ST_DELAYING = 1 << 2
ST_TX_EMPTY = 1 << 3
ST_TX_FULL = 1 << 4
ST_RX_EMPTY = 1 << 5
ST_RX_FULL = 1 << 6
ST_PENDING = 1 << 7

# Command byte bits
CMD_WRITE = 0x80
CMD_IMEM = 0x01

# GPIO numbering
GPIO_UI = 0      # gpio[7:0]   = ui_in
GPIO_UO = 8      # gpio[15:8]  = uo_out
GPIO_UIO = 16    # gpio[19:16] = uio[3:0]


def sm_reg(sm: int, off: int) -> int:
    return SM_BASE + SM_STRIDE * sm + off
