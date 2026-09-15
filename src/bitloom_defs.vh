// BitLoom instruction-set constants.
// Instruction word: [15:13] opcode, [12:9] delay/side-set, [8:0] operand.
`ifndef BITLOOM_DEFS_VH
`define BITLOOM_DEFS_VH

`define BL_OP_JMP   3'd0
`define BL_OP_WAIT  3'd1
`define BL_OP_IN    3'd2
`define BL_OP_OUT   3'd3
`define BL_OP_PUSH  3'd4   // PUSH / PULL
`define BL_OP_MOV   3'd5
`define BL_OP_SET   3'd6
`define BL_OP_TIME  3'd7

// JMP conditions (operand[8:6])
`define BL_JC_ALWAYS 3'd0
`define BL_JC_NOTX   3'd1
`define BL_JC_XDEC   3'd2
`define BL_JC_NOTY   3'd3
`define BL_JC_YDEC   3'd4
`define BL_JC_XNEY   3'd5
`define BL_JC_PIN    3'd6
`define BL_JC_NOTOSRE 3'd7

// WAIT sources (operand[7:6])
`define BL_WS_GPIO 2'd0
`define BL_WS_PIN  2'd1
`define BL_WS_FLAG 2'd2
`define BL_WS_TIME 2'd3

// IN / MOV sources (operand[8:6] / operand[2:0])
`define BL_SRC_PINS   3'd0
`define BL_SRC_X      3'd1
`define BL_SRC_Y      3'd2
`define BL_SRC_NULL   3'd3
`define BL_SRC_T      3'd4
`define BL_SRC_STATUS 3'd5
`define BL_SRC_ISR    3'd6
`define BL_SRC_OSR    3'd7

// OUT destinations (operand[8:6])
`define BL_OD_PINS    3'd0
`define BL_OD_X       3'd1
`define BL_OD_Y       3'd2
`define BL_OD_NULL    3'd3
`define BL_OD_PINDIRS 3'd4
`define BL_OD_PC      3'd5
`define BL_OD_ISR     3'd6
`define BL_OD_DL      3'd7

// MOV destinations (operand[8:6])
`define BL_MD_PINS    3'd0
`define BL_MD_X       3'd1
`define BL_MD_Y       3'd2
`define BL_MD_DL      3'd3
`define BL_MD_PINDIRS 3'd4
`define BL_MD_PC      3'd5
`define BL_MD_ISR     3'd6
`define BL_MD_OSR     3'd7

// SET destinations (operand[8:6])
`define BL_SD_PINS    3'd0
`define BL_SD_X       3'd1
`define BL_SD_Y       3'd2
`define BL_SD_PINDIRS 3'd3
`define BL_SD_FLAGSET 3'd4
`define BL_SD_FLAGCLR 3'd5
`define BL_SD_T       3'd6

// TIME modes (operand[8:7])
`define BL_TM_T_IMM   2'd0   // DL = T  + imm
`define BL_TM_DL_IMM  2'd1   // DL = DL + imm
`define BL_TM_T_X     2'd2   // DL = T  + X
`define BL_TM_DL_X    2'd3   // DL = DL + X

`endif
