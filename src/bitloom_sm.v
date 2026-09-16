/*
 * BitLoom state machine.
 *
 * One programmable I/O state machine. It executes one 16-bit instruction per
 * "tick" (a clock-divided cycle) with deterministic timing: an instruction
 * either completes in exactly one tick or stalls (WAIT / blocking PUSH / PULL)
 * until its condition holds. Every instruction also carries a delay field so
 * the program can pad to exact bit periods.
 *
 * Instruction fetch is registered: the word at PC is latched into IR every
 * clock and decoded on the next tick. The 64:1 read mux of the shared
 * instruction memory is the longest path in the chip and this keeps it out
 * of the decode/execute cycle. The price is that a tick is at least two
 * clocks long, so DIV_INT = 1 behaves as 2 (25 M instructions/s at 50 MHz).
 *
 * Registers (per state machine):
 *   PC        instruction pointer (shared instruction memory)
 *   X, Y      16-bit scratch registers
 *   ISR, OSR  16-bit input / output shift registers with shift counters
 *   T         16-bit free-running timebase, increments once per tick
 *   DL        16-bit deadline; "wait time" blocks until T reaches DL
 *
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none
`include "bitloom_defs.vh"

module bitloom_sm #(
    parameter IMEM_AW = 6
) (
    input  wire        clk,
    input  wire        rst_n,

    // ---- control -----------------------------------------------------
    input  wire        cfg_enable,
    input  wire        restart,        // pulse: clear datapath state
    input  wire        step,           // pulse: execute exactly one instruction
    input  wire        pc_wr,
    input  wire [IMEM_AW-1:0] pc_wdata,
    input  wire        exec_wr,        // pulse: execute exec_instr once
    input  wire [15:0] exec_instr,

    // ---- static configuration ---------------------------------------
    input  wire [15:0] cfg_div_int,
    input  wire [7:0]  cfg_div_frac,
    input  wire [4:0]  cfg_out_base,
    input  wire [4:0]  cfg_out_count,
    input  wire [4:0]  cfg_set_base,
    input  wire [2:0]  cfg_set_count,
    input  wire [4:0]  cfg_in_base,
    input  wire [4:0]  cfg_ss_base,
    input  wire [1:0]  cfg_ss_count,
    input  wire        cfg_ss_pindirs,
    input  wire [4:0]  cfg_jmp_pin,
    input  wire [IMEM_AW-1:0] cfg_wrap_top,
    input  wire [IMEM_AW-1:0] cfg_wrap_bot,
    input  wire        cfg_out_shr,    // 1: OUT shifts right (LSB first)
    input  wire        cfg_in_shr,     // 1: IN shifts right
    input  wire        cfg_autopull,
    input  wire        cfg_autopush,
    input  wire [4:0]  cfg_pull_thresh, // 0 means 16
    input  wire [4:0]  cfg_push_thresh, // 0 means 16

    // ---- instruction memory -----------------------------------------
    output wire [IMEM_AW-1:0] imem_addr,
    input  wire [15:0] imem_rdata,

    // ---- pins --------------------------------------------------------
    input  wire [19:0] gpio_in,
    output reg  [19:0] pin_out_mask,   // combinational, valid this cycle
    output reg  [19:0] pin_out_val,
    output reg  [19:0] pin_oe_mask,
    output reg  [19:0] pin_oe_val,

    // ---- inter-machine flags ----------------------------------------
    input  wire [7:0]  flags,
    output reg  [7:0]  flag_set,
    output reg  [7:0]  flag_clr,

    // ---- host FIFO ports --------------------------------------------
    input  wire        tx_push,
    input  wire [15:0] tx_wdata,
    output wire        tx_full,
    output wire        tx_empty,
    output wire [2:0]  tx_level,
    input  wire        rx_pop,
    output wire [15:0] rx_rdata,
    output wire        rx_full,
    output wire        rx_empty,
    output wire [2:0]  rx_level,

    // ---- debug view --------------------------------------------------
    output wire [IMEM_AW-1:0] dbg_pc,
    output wire [15:0] dbg_x,
    output wire [15:0] dbg_y,
    output wire [15:0] dbg_t,
    output wire [7:0]  dbg_status
);

  // ------------------------------------------------------------------
  // Helpers
  // ------------------------------------------------------------------
  function [19:0] rotl20;
    input [19:0] v;
    input [4:0]  n;
    reg   [39:0] t;
    begin
      t = {v, v} << n;
      rotl20 = t[39:20];
    end
  endfunction

  function [19:0] rotr20;
    input [19:0] v;
    input [4:0]  n;
    reg   [39:0] t;
    begin
      t = {v, v} >> n;
      rotr20 = t[19:0];
    end
  endfunction

  function [15:0] bitrev16;
    input [15:0] v;
    integer i;
    begin
      for (i = 0; i < 16; i = i + 1) bitrev16[i] = v[15 - i];
    end
  endfunction

  function [15:0] lowmask;      // (1 << n) - 1 for n in 1..16
    input [4:0] n;
    reg [16:0] t;
    begin
      t = 17'h1 << n;
      lowmask = t[15:0] - 16'h1;
      if (n == 5'd16) lowmask = 16'hFFFF;
    end
  endfunction

  function [4:0] sat16;         // saturating add of two shift counts
    input [4:0] a;
    input [4:0] b;
    reg [5:0] s;
    begin
      s = {1'b0, a} + {1'b0, b};
      sat16 = (s > 6'd16) ? 5'd16 : s[4:0];
    end
  endfunction

  // ------------------------------------------------------------------
  // State
  // ------------------------------------------------------------------
  reg [IMEM_AW-1:0] pc;
  reg [15:0] x, y, isr, osr, t, dl;
  reg [4:0]  isr_cnt, osr_cnt;
  reg [3:0]  delay_cnt;
  reg [15:0] div_cnt;
  reg [7:0]  div_frac;
  reg        exec_pend;
  reg [15:0] exec_ir;
  reg        step_pend;
  reg        stalled_r;
  reg [15:0] ir_r;                 // instruction at PC, fetched last clock

  assign imem_addr = pc;
  assign dbg_pc = pc;
  assign dbg_x = x;
  assign dbg_y = y;
  assign dbg_t = t;

  // ------------------------------------------------------------------
  // FIFOs
  // ------------------------------------------------------------------
  reg         txf_pop;
  wire [15:0] txf_rdata;
  reg         rxf_push;
  reg  [15:0] rxf_wdata;

  bitloom_fifo #(.W(16), .AW(2)) u_txf (
      .clk(clk), .rst_n(rst_n), .clr(restart),
      .push(tx_push), .wdata(tx_wdata),
      .pop(txf_pop), .rdata(txf_rdata),
      .full(tx_full), .empty(tx_empty), .level(tx_level)
  );

  bitloom_fifo #(.W(16), .AW(2)) u_rxf (
      .clk(clk), .rst_n(rst_n), .clr(restart),
      .push(rxf_push), .wdata(rxf_wdata),
      .pop(rx_pop), .rdata(rx_rdata),
      .full(rx_full), .empty(rx_empty), .level(rx_level)
  );

  // ------------------------------------------------------------------
  // Clock divider: tick when the down-counter hits zero. Reload with
  // int-1 (+1 when the fractional accumulator carries). int = 0 means 65536.
  // ------------------------------------------------------------------
  wire       tick = (div_cnt == 16'd0);
  wire [8:0] frac_sum = {1'b0, div_frac} + {1'b0, cfg_div_frac};
  // The fetch pipeline needs one clock between ticks: a divider of 1 runs as 2.
  wire [15:0] div_int_eff = (cfg_div_int == 16'd1) ? 16'd2 : cfg_div_int;

  always @(posedge clk) begin
    if (!rst_n) begin
      div_cnt  <= 16'd0;
      div_frac <= 8'd0;
    end else if (tick) begin
      div_cnt  <= div_int_eff - 16'd1 + {15'd0, frac_sum[8]};
      div_frac <= frac_sum[7:0];
    end else begin
      div_cnt  <= div_cnt - 16'd1;
    end
  end

  // ------------------------------------------------------------------
  // Decode
  // ------------------------------------------------------------------
  wire        run     = tick && (cfg_enable || step_pend || exec_pend) && !restart;
  wire        do_exec = run && (exec_pend || step_pend || delay_cnt == 4'd0);

  // ------------------------------------------------------------------
  // Fetch: PC is only ever changed at a commit or by the host, and the next
  // tick is at least two clocks away, so IR is always current when decoded.
  // ------------------------------------------------------------------
  always @(posedge clk) begin
    if (!rst_n) ir_r <= 16'd0;
    else        ir_r <= imem_rdata;
  end

  wire [15:0] ir  = exec_pend ? exec_ir : ir_r;
  wire [2:0]  op  = ir[15:13];
  wire [3:0]  ds  = ir[12:9];
  wire [8:0]  arg = ir[8:0];

  wire [3:0]  delay_field = ds & ~(4'hF << (3'd4 - {1'b0, cfg_ss_count}));
  wire [2:0]  ss_val      = ds[3:1] >> (2'd3 - cfg_ss_count);
  wire [19:0] ss_mask     = rotl20({17'd0, 3'b111 >> (2'd3 - cfg_ss_count)}, cfg_ss_base);
  wire [19:0] ss_bits     = rotl20({17'd0, ss_val}, cfg_ss_base);

  wire [4:0]  cnt_field   = (arg[4:0] == 5'd0) ? 5'd16 : arg[4:0];
  wire [4:0]  pull_thresh = (cfg_pull_thresh == 5'd0) ? 5'd16 : cfg_pull_thresh;
  wire [4:0]  push_thresh = (cfg_push_thresh == 5'd0) ? 5'd16 : cfg_push_thresh;
  wire        osr_empty   = (osr_cnt >= pull_thresh);

  wire [19:0] pins_rot    = rotr20(gpio_in, cfg_in_base);
  wire [15:0] pins_in     = pins_rot[15:0];
  wire [15:0] status      = {12'd0, rx_full, rx_empty, tx_full, tx_empty};

  wire [19:0] out_mask    = rotl20({4'd0, lowmask(cfg_out_count)}, cfg_out_base);
  wire [19:0] set_mask    = rotl20({14'd0, 6'h3F >> (3'd6 - cfg_set_count)}, cfg_set_base);

  // Source mux shared by IN and MOV
  reg [15:0] src_val;
  always @(*) begin
    case (op == `BL_OP_MOV ? arg[2:0] : arg[8:6])
      `BL_SRC_PINS:   src_val = pins_in;
      `BL_SRC_X:      src_val = x;
      `BL_SRC_Y:      src_val = y;
      `BL_SRC_NULL:   src_val = 16'd0;
      `BL_SRC_T:      src_val = t;
      `BL_SRC_STATUS: src_val = status;
      `BL_SRC_ISR:    src_val = isr;
      default:        src_val = osr;
    endcase
  end

  reg [15:0] mov_val;
  always @(*) begin
    case (arg[5:4])
      2'd1:    mov_val = ~src_val;
      2'd2:    mov_val = bitrev16(src_val);
      default: mov_val = src_val;
    endcase
  end

  // OUT datapath with autopull-on-demand
  wire        out_pull   = cfg_autopull && osr_empty;
  wire [15:0] osr_src    = out_pull ? txf_rdata : osr;
  wire [4:0]  osr_cnt_src = out_pull ? 5'd0 : osr_cnt;
  wire [15:0] out_data   = cfg_out_shr ? (osr_src & lowmask(cnt_field))
                                       : (osr_src >> (5'd16 - cnt_field));
  wire [15:0] osr_shift  = cfg_out_shr ? (osr_src >> cnt_field) : (osr_src << cnt_field);
  wire [4:0]  osr_cnt_nx = sat16(osr_cnt_src, cnt_field);

  // IN datapath
  wire [15:0] in_data    = src_val & lowmask(cnt_field);
  wire [15:0] isr_shift  = cfg_in_shr ? ((isr >> cnt_field) | (in_data << (5'd16 - cnt_field)))
                                      : ((isr << cnt_field) | in_data);
  wire [4:0]  isr_cnt_nx = sat16(isr_cnt, cnt_field);
  wire        in_autopush = cfg_autopush && (isr_cnt_nx >= push_thresh);

  // WAIT condition
  reg wait_ok;
  always @(*) begin
    case (arg[7:6])
      `BL_WS_GPIO: wait_ok = (gpio_in[arg[4:0]] == arg[8]);
      `BL_WS_PIN:  wait_ok = (pins_rot[arg[4:0]] == arg[8]);
      `BL_WS_FLAG: wait_ok = (flags[arg[2:0]] == arg[8]);
      default:     wait_ok = !((t - dl) >> 15);  // T >= DL (wrap-safe within 32767 ticks)
    endcase
  end

  // JMP condition
  reg jmp_take;
  always @(*) begin
    case (arg[8:6])
      `BL_JC_ALWAYS:  jmp_take = 1'b1;
      `BL_JC_NOTX:    jmp_take = (x == 16'd0);
      `BL_JC_XDEC:    jmp_take = (x != 16'd0);
      `BL_JC_NOTY:    jmp_take = (y == 16'd0);
      `BL_JC_YDEC:    jmp_take = (y != 16'd0);
      `BL_JC_XNEY:    jmp_take = (x != y);
      `BL_JC_PIN:     jmp_take = gpio_in[cfg_jmp_pin];
      default:        jmp_take = !osr_empty;
    endcase
  end

  // TIME datapath
  wire [15:0] time_base = arg[7] ? dl : t;
  wire [15:0] time_add  = arg[8] ? x : {9'd0, arg[6:0]};
  wire [15:0] dl_nx     = time_base + time_add;

  // Stall determination
  reg stall;
  always @(*) begin
    stall = 1'b0;
    case (op)
      `BL_OP_WAIT: stall = !wait_ok;
      `BL_OP_IN:   stall = in_autopush && rx_full;
      `BL_OP_OUT:  stall = out_pull && tx_empty;
      `BL_OP_PUSH: begin
        if (!arg[8]) begin                           // PUSH
          if (!(arg[7] && isr_cnt < push_thresh)) stall = rx_full && arg[6];
        end else begin                               // PULL
          if (!(arg[7] && !osr_empty)) stall = tx_empty && arg[6];
        end
      end
      default: ;
    endcase
  end

  wire commit = do_exec && !stall;
  wire [IMEM_AW-1:0] pc_seq = (pc == cfg_wrap_top) ? cfg_wrap_bot : pc + 1'b1;

  // ------------------------------------------------------------------
  // Pin / flag effects (combinational, consumed by the top level)
  // ------------------------------------------------------------------
  always @(*) begin
    pin_out_mask = 20'd0; pin_out_val = 20'd0;
    pin_oe_mask  = 20'd0; pin_oe_val  = 20'd0;
    flag_set = 8'd0; flag_clr = 8'd0;

    if (commit) begin
      case (op)
        `BL_OP_OUT: begin
          if (arg[8:6] == `BL_OD_PINS) begin
            pin_out_mask = out_mask;
            pin_out_val  = rotl20({4'd0, out_data}, cfg_out_base);
          end else if (arg[8:6] == `BL_OD_PINDIRS) begin
            pin_oe_mask = out_mask;
            pin_oe_val  = rotl20({4'd0, out_data}, cfg_out_base);
          end
        end
        `BL_OP_MOV: begin
          if (arg[8:6] == `BL_MD_PINS) begin
            pin_out_mask = out_mask;
            pin_out_val  = rotl20({4'd0, mov_val}, cfg_out_base);
          end else if (arg[8:6] == `BL_MD_PINDIRS) begin
            pin_oe_mask = out_mask;
            pin_oe_val  = rotl20({4'd0, mov_val}, cfg_out_base);
          end
        end
        `BL_OP_SET: begin
          if (arg[8:6] == `BL_SD_PINS) begin
            pin_out_mask = set_mask;
            pin_out_val  = rotl20({14'd0, arg[5:0]}, cfg_set_base);
          end else if (arg[8:6] == `BL_SD_PINDIRS) begin
            pin_oe_mask = set_mask;
            pin_oe_val  = rotl20({14'd0, arg[5:0]}, cfg_set_base);
          end else if (arg[8:6] == `BL_SD_FLAGSET) begin
            flag_set = 8'h1 << arg[2:0];
          end else if (arg[8:6] == `BL_SD_FLAGCLR) begin
            flag_clr = 8'h1 << arg[2:0];
          end
        end
        `BL_OP_WAIT: begin
          // WAIT 1 FLAG n clears the flag once it is observed set.
          if (arg[7:6] == `BL_WS_FLAG && arg[8]) flag_clr = 8'h1 << arg[2:0];
        end
        default: ;
      endcase
    end

    // Side-set is applied on every execution attempt, even when stalled,
    // and takes priority over OUT/SET/MOV in the same instruction.
    if (do_exec && cfg_ss_count != 2'd0) begin
      if (cfg_ss_pindirs) begin
        pin_oe_mask = pin_oe_mask | ss_mask;
        pin_oe_val  = (pin_oe_val & ~ss_mask) | ss_bits;
      end else begin
        pin_out_mask = pin_out_mask | ss_mask;
        pin_out_val  = (pin_out_val & ~ss_mask) | ss_bits;
      end
    end
  end

  // ------------------------------------------------------------------
  // FIFO strobes
  // ------------------------------------------------------------------
  always @(*) begin
    txf_pop   = 1'b0;
    rxf_push  = 1'b0;
    rxf_wdata = isr_shift;
    if (commit) begin
      case (op)
        `BL_OP_OUT:  txf_pop = out_pull;
        `BL_OP_IN:   rxf_push = in_autopush;
        `BL_OP_PUSH: begin
          if (!arg[8]) begin
            if (!(arg[7] && isr_cnt < push_thresh) && !rx_full) begin
              rxf_push  = 1'b1;
              rxf_wdata = isr;
            end
          end else begin
            if (!(arg[7] && !osr_empty) && !tx_empty) txf_pop = 1'b1;
          end
        end
        default: ;
      endcase
    end
  end

  // ------------------------------------------------------------------
  // Sequential state
  // ------------------------------------------------------------------
  always @(posedge clk) begin
    if (!rst_n) begin
      pc <= 0; x <= 0; y <= 0; isr <= 0; osr <= 0; t <= 0; dl <= 0;
      isr_cnt <= 0; osr_cnt <= 5'd16; delay_cnt <= 0;
      exec_pend <= 0; exec_ir <= 0; step_pend <= 0; stalled_r <= 0;
    end else begin
      // Host side
      if (pc_wr)   pc <= pc_wdata;
      if (exec_wr) begin exec_pend <= 1'b1; exec_ir <= exec_instr; end
      if (step)    step_pend <= 1'b1;

      if (restart) begin
        x <= 0; y <= 0; isr <= 0; osr <= 0; t <= 0; dl <= 0;
        isr_cnt <= 0; osr_cnt <= 5'd16; delay_cnt <= 0;
        exec_pend <= 0; step_pend <= 0; stalled_r <= 0;
      end else begin
        if (run && cfg_enable) t <= t + 16'd1;
        if (run && !do_exec) delay_cnt <= delay_cnt - 4'd1;

        if (do_exec) stalled_r <= stall;

        if (commit) begin
          if (exec_pend) exec_pend <= 1'b0;
          else begin
            pc <= pc_seq;
            delay_cnt <= delay_field;
          end
          step_pend <= 1'b0;

          case (op)
            `BL_OP_JMP: if (jmp_take) begin
              pc <= arg[IMEM_AW-1:0];
              if (arg[8:6] == `BL_JC_XDEC) x <= x - 16'd1;
              if (arg[8:6] == `BL_JC_YDEC) y <= y - 16'd1;
            end

            `BL_OP_IN: begin
              if (in_autopush) begin
                isr <= 16'd0; isr_cnt <= 5'd0;
              end else begin
                isr <= isr_shift; isr_cnt <= isr_cnt_nx;
              end
            end

            `BL_OP_OUT: begin
              osr <= osr_shift; osr_cnt <= osr_cnt_nx;
              case (arg[8:6])
                `BL_OD_X:   x <= out_data;
                `BL_OD_Y:   y <= out_data;
                `BL_OD_PC:  pc <= out_data[IMEM_AW-1:0];
                `BL_OD_ISR: begin isr <= out_data; isr_cnt <= cnt_field; end
                `BL_OD_DL:  dl <= out_data;
                default: ;
              endcase
            end

            `BL_OP_PUSH: begin
              if (!arg[8]) begin
                if (!(arg[7] && isr_cnt < push_thresh)) begin
                  if (!rx_full) begin isr <= 16'd0; isr_cnt <= 5'd0; end
                end
              end else begin
                if (!(arg[7] && !osr_empty)) begin
                  if (tx_empty) begin
                    osr <= x; osr_cnt <= 5'd0;   // non-blocking PULL on empty copies X
                  end else begin
                    osr <= txf_rdata; osr_cnt <= 5'd0;
                  end
                end
              end
            end

            `BL_OP_MOV: begin
              case (arg[8:6])
                `BL_MD_X:   x <= mov_val;
                `BL_MD_Y:   y <= mov_val;
                `BL_MD_DL:  dl <= mov_val;
                `BL_MD_PC:  pc <= mov_val[IMEM_AW-1:0];
                `BL_MD_ISR: begin isr <= mov_val; isr_cnt <= 5'd0; end
                `BL_MD_OSR: begin osr <= mov_val; osr_cnt <= 5'd0; end
                default: ;
              endcase
            end

            `BL_OP_SET: begin
              case (arg[8:6])
                `BL_SD_X: x <= {10'd0, arg[5:0]};
                `BL_SD_Y: y <= {10'd0, arg[5:0]};
                `BL_SD_T: t <= {10'd0, arg[5:0]};
                default: ;
              endcase
            end

            `BL_OP_TIME: dl <= dl_nx;

            default: ;
          endcase
        end
      end
    end
  end

  assign dbg_status = {exec_pend | step_pend, rx_full, rx_empty, tx_full, tx_empty,
                       delay_cnt != 4'd0, stalled_r, cfg_enable};

endmodule
