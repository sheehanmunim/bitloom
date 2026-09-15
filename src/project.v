/*
 * BitLoom: a programmable protocol-emulator ASIC for Tiny Tapeout (IHP CMOS5L).
 *
 * Top level. Instantiates N_SM state machines that share a 64-word
 * instruction memory, exposes 20 GPIOs to them, and offers a SPI slave host
 * port on uio[7:4] for loading programs, configuring machines and moving data
 * through the FIFOs.
 *
 * GPIO numbering as seen by programs:
 *   gpio[7:0]   = ui_in[7:0]   (input only)
 *   gpio[15:8]  = uo_out[7:0]  (output only, read back as driven value)
 *   gpio[19:16] = uio[3:0]     (bidirectional; direction under program control)
 *
 * Host port:  uio[4] = CS_n, uio[5] = SCK, uio[6] = MOSI, uio[7] = MISO
 *
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

module tt_um_sheehanmunim_bitloom (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);
  localparam N_SM    = 3;
  localparam IMEM_AW = 6;
  localparam IMEM_N  = 1 << IMEM_AW;
  localparam VERSION = 8'h01;

  wire _unused = &{ena, 1'b0};

  // ------------------------------------------------------------------
  // Host SPI port
  // ------------------------------------------------------------------
  wire       h_wr, h_rd, h_space;
  wire [7:0] h_addr, h_raddr, h_wdata;
  reg  [7:0] h_rdata;
  wire       h_miso;

  bitloom_spi u_spi (
      .clk(clk), .rst_n(rst_n),
      .sck(uio_in[5]), .csn(uio_in[4]), .mosi(uio_in[6]), .miso(h_miso),
      .wr_en(h_wr), .rd_en(h_rd), .space(h_space), .addr(h_addr), .raddr(h_raddr),
      .wdata(h_wdata), .rdata(h_rdata)
  );

  // ------------------------------------------------------------------
  // Input synchronisers + optional 3-sample majority filter
  // ------------------------------------------------------------------
  reg [11:0] in_s1, in_s2, in_s3, in_s4;
  reg        cfg_infilt;
  wire [11:0] in_raw = {uio_in[3:0], ui_in};
  always @(posedge clk) begin
    if (!rst_n) begin
      in_s1 <= 0; in_s2 <= 0; in_s3 <= 0; in_s4 <= 0;
    end else begin
      in_s1 <= in_raw; in_s2 <= in_s1; in_s3 <= in_s2; in_s4 <= in_s3;
    end
  end
  wire [11:0] in_maj = (in_s2 & in_s3) | (in_s3 & in_s4) | (in_s2 & in_s4);
  wire [11:0] in_f   = cfg_infilt ? in_maj : in_s2;

  reg  [19:8] gpio_out;
  reg  [19:16] gpio_oe;
  wire [19:0] gpio_in = {in_f[11:8], gpio_out[15:8], in_f[7:0]};

  assign uo_out       = gpio_out[15:8];
  assign uio_out[3:0] = gpio_out[19:16];
  assign uio_oe[3:0]  = gpio_oe[19:16];
  assign uio_out[7:4] = {h_miso, 3'b000};
  assign uio_oe[7:4]  = 4'b1000;

  // ------------------------------------------------------------------
  // Instruction memory (flip-flop based, shared by all machines)
  // ------------------------------------------------------------------
  reg [15:0] imem [0:IMEM_N-1];
  reg [7:0]  imem_lo;
  wire [IMEM_AW-1:0] h_imem_word = h_addr[IMEM_AW:1];
  wire [15:0] h_imem_rd = imem[h_raddr[IMEM_AW:1]];

  always @(posedge clk) begin
    if (h_wr && h_space) begin
      if (!h_addr[0]) imem_lo <= h_wdata;
      else            imem[h_imem_word] <= {h_wdata, imem_lo};
    end
  end

  // ------------------------------------------------------------------
  // Per-machine configuration registers
  // ------------------------------------------------------------------
  reg [N_SM-1:0] cfg_enable;
  reg [7:0]      flags;

  reg [15:0] cfg_div_int   [0:N_SM-1];
  reg [7:0]  cfg_div_frac  [0:N_SM-1];
  reg [4:0]  cfg_out_base  [0:N_SM-1];
  reg [4:0]  cfg_out_count [0:N_SM-1];
  reg [4:0]  cfg_set_base  [0:N_SM-1];
  reg [2:0]  cfg_set_count [0:N_SM-1];
  reg [4:0]  cfg_in_base   [0:N_SM-1];
  reg [7:0]  cfg_sideset   [0:N_SM-1];   // [4:0] base, [6:5] count, [7] pindirs
  reg [4:0]  cfg_jmp_pin   [0:N_SM-1];
  reg [IMEM_AW-1:0] cfg_wrap_top [0:N_SM-1];
  reg [IMEM_AW-1:0] cfg_wrap_bot [0:N_SM-1];
  reg [3:0]  cfg_shift     [0:N_SM-1];   // [0] out_shr [1] in_shr [2] autopull [3] autopush
  reg [4:0]  cfg_pull_th   [0:N_SM-1];
  reg [4:0]  cfg_push_th   [0:N_SM-1];
  reg [7:0]  txf_lo, exec_lo, rxf_hi;

  // Address decode: 0x00-0x1F global, then 0x20 per machine.
  wire        h_is_sm = (h_addr[7:5] != 3'd0) && (h_addr[7:5] <= N_SM);
  wire [2:0]  h_sm    = h_addr[7:5] - 3'd1;
  wire [4:0]  h_off   = h_addr[4:0];
  // Read-data mux is addressed by the running address so the byte is ready
  // when the SPI engine loads it; strobes use the latched address.
  wire        r_is_sm = (h_raddr[7:5] != 3'd0) && (h_raddr[7:5] <= N_SM);
  wire [2:0]  r_sm    = h_raddr[7:5] - 3'd1;
  wire [4:0]  r_off   = h_raddr[4:0];
  wire        h_wr_reg = h_wr && !h_space;
  wire        h_rd_reg = h_rd && !h_space;

  // ------------------------------------------------------------------
  // State machines
  // ------------------------------------------------------------------
  wire [IMEM_AW-1:0] sm_imem_addr [0:N_SM-1];
  wire [19:0] sm_out_mask [0:N_SM-1];
  wire [19:0] sm_out_val  [0:N_SM-1];
  wire [19:0] sm_oe_mask  [0:N_SM-1];
  wire [19:0] sm_oe_val   [0:N_SM-1];
  wire [7:0]  sm_flag_set [0:N_SM-1];
  wire [7:0]  sm_flag_clr [0:N_SM-1];
  wire        sm_tx_full  [0:N_SM-1];
  wire        sm_tx_empty [0:N_SM-1];
  wire [2:0]  sm_tx_level [0:N_SM-1];
  wire [15:0] sm_rx_rdata [0:N_SM-1];
  wire        sm_rx_full  [0:N_SM-1];
  wire        sm_rx_empty [0:N_SM-1];
  wire [2:0]  sm_rx_level [0:N_SM-1];
  wire [IMEM_AW-1:0] sm_pc [0:N_SM-1];
  wire [15:0] sm_x [0:N_SM-1];
  wire [15:0] sm_y [0:N_SM-1];
  wire [15:0] sm_t [0:N_SM-1];
  wire [7:0]  sm_status [0:N_SM-1];

  genvar g;
  generate
    for (g = 0; g < N_SM; g = g + 1) begin : sm
      wire sel      = h_is_sm && (h_sm == g);
      wire restart  = h_wr_reg && (h_addr == 8'h05) && h_wdata[g];
      wire step     = h_wr_reg && (h_addr == 8'h06) && h_wdata[g];
      wire pc_wr    = h_wr_reg && sel && (h_off == 5'h0F);
      wire exec_wr  = h_wr_reg && sel && (h_off == 5'h1D);
      wire tx_push  = h_wr_reg && sel && (h_off == 5'h19);
      wire rx_pop   = h_rd_reg && sel && (h_off == 5'h1A);

      bitloom_sm #(.IMEM_AW(IMEM_AW)) u_sm (
          .clk(clk), .rst_n(rst_n),
          .cfg_enable(cfg_enable[g]), .restart(restart), .step(step),
          .pc_wr(pc_wr), .pc_wdata(h_wdata[IMEM_AW-1:0]),
          .exec_wr(exec_wr), .exec_instr({h_wdata, exec_lo}),
          .cfg_div_int(cfg_div_int[g]), .cfg_div_frac(cfg_div_frac[g]),
          .cfg_out_base(cfg_out_base[g]), .cfg_out_count(cfg_out_count[g]),
          .cfg_set_base(cfg_set_base[g]), .cfg_set_count(cfg_set_count[g]),
          .cfg_in_base(cfg_in_base[g]),
          .cfg_ss_base(cfg_sideset[g][4:0]), .cfg_ss_count(cfg_sideset[g][6:5]),
          .cfg_ss_pindirs(cfg_sideset[g][7]),
          .cfg_jmp_pin(cfg_jmp_pin[g]),
          .cfg_wrap_top(cfg_wrap_top[g]), .cfg_wrap_bot(cfg_wrap_bot[g]),
          .cfg_out_shr(cfg_shift[g][0]), .cfg_in_shr(cfg_shift[g][1]),
          .cfg_autopull(cfg_shift[g][2]), .cfg_autopush(cfg_shift[g][3]),
          .cfg_pull_thresh(cfg_pull_th[g]), .cfg_push_thresh(cfg_push_th[g]),
          .imem_addr(sm_imem_addr[g]), .imem_rdata(imem[sm_imem_addr[g]]),
          .gpio_in(gpio_in),
          .pin_out_mask(sm_out_mask[g]), .pin_out_val(sm_out_val[g]),
          .pin_oe_mask(sm_oe_mask[g]), .pin_oe_val(sm_oe_val[g]),
          .flags(flags), .flag_set(sm_flag_set[g]), .flag_clr(sm_flag_clr[g]),
          .tx_push(tx_push), .tx_wdata({h_wdata, txf_lo}),
          .tx_full(sm_tx_full[g]), .tx_empty(sm_tx_empty[g]), .tx_level(sm_tx_level[g]),
          .rx_pop(rx_pop), .rx_rdata(sm_rx_rdata[g]),
          .rx_full(sm_rx_full[g]), .rx_empty(sm_rx_empty[g]), .rx_level(sm_rx_level[g]),
          .dbg_pc(sm_pc[g]), .dbg_x(sm_x[g]), .dbg_y(sm_y[g]), .dbg_t(sm_t[g]),
          .dbg_status(sm_status[g])
      );
    end
  endgenerate

  // ------------------------------------------------------------------
  // GPIO output registers: later machines take priority on conflicts.
  // ------------------------------------------------------------------
  reg [19:0] out_nx, oe_nx;
  integer i;
  always @(*) begin
    out_nx = {gpio_out, 8'd0};
    oe_nx  = {gpio_oe, 16'd0};
    for (i = 0; i < N_SM; i = i + 1) begin
      out_nx = (out_nx & ~sm_out_mask[i]) | (sm_out_val[i] & sm_out_mask[i]);
      oe_nx  = (oe_nx  & ~sm_oe_mask[i])  | (sm_oe_val[i]  & sm_oe_mask[i]);
    end
  end

  reg [7:0] flag_set_any, flag_clr_any;
  always @(*) begin
    flag_set_any = 8'd0;
    flag_clr_any = 8'd0;
    for (i = 0; i < N_SM; i = i + 1) begin
      flag_set_any = flag_set_any | sm_flag_set[i];
      flag_clr_any = flag_clr_any | sm_flag_clr[i];
    end
    if (h_wr_reg && h_addr == 8'h07) flag_clr_any = flag_clr_any | h_wdata;
  end

  always @(posedge clk) begin
    if (!rst_n) begin
      gpio_out <= 12'd0;
      gpio_oe  <= 4'd0;
      flags    <= 8'd0;
    end else begin
      gpio_out <= out_nx[19:8];
      gpio_oe  <= oe_nx[19:16];
      flags    <= (flags & ~flag_clr_any) | flag_set_any;
    end
  end

  // ------------------------------------------------------------------
  // Register writes
  // ------------------------------------------------------------------
  integer k;
  always @(posedge clk) begin
    if (!rst_n) begin
      cfg_enable <= 0;
      cfg_infilt <= 1'b0;
      txf_lo <= 8'd0; exec_lo <= 8'd0;
      for (k = 0; k < N_SM; k = k + 1) begin
        cfg_div_int[k] <= 16'd1; cfg_div_frac[k] <= 8'd0;
        cfg_out_base[k] <= 5'd0; cfg_out_count[k] <= 5'd0;
        cfg_set_base[k] <= 5'd0; cfg_set_count[k] <= 3'd0;
        cfg_in_base[k] <= 5'd0; cfg_sideset[k] <= 8'd0; cfg_jmp_pin[k] <= 5'd0;
        cfg_wrap_top[k] <= {IMEM_AW{1'b1}}; cfg_wrap_bot[k] <= {IMEM_AW{1'b0}};
        cfg_shift[k] <= 4'd0; cfg_pull_th[k] <= 5'd0; cfg_push_th[k] <= 5'd0;
      end
    end else if (h_wr_reg) begin
      if (!h_is_sm) begin
        case (h_addr)
          8'h04: cfg_enable <= h_wdata[N_SM-1:0];
          8'h0F: cfg_infilt <= h_wdata[0];
          default: ;
        endcase
      end else begin
        for (k = 0; k < N_SM; k = k + 1) begin
          if (h_sm == k) begin
            case (h_off)
              5'h00: cfg_div_int[k][7:0]  <= h_wdata;
              5'h01: cfg_div_int[k][15:8] <= h_wdata;
              5'h02: cfg_div_frac[k]      <= h_wdata;
              5'h03: cfg_out_base[k]      <= h_wdata[4:0];
              5'h04: cfg_out_count[k]     <= h_wdata[4:0];
              5'h05: cfg_set_base[k]      <= h_wdata[4:0];
              5'h06: cfg_set_count[k]     <= h_wdata[2:0];
              5'h07: cfg_in_base[k]       <= h_wdata[4:0];
              5'h08: cfg_sideset[k]       <= h_wdata;
              5'h09: cfg_jmp_pin[k]       <= h_wdata[4:0];
              5'h0A: cfg_wrap_top[k]      <= h_wdata[IMEM_AW-1:0];
              5'h0B: cfg_wrap_bot[k]      <= h_wdata[IMEM_AW-1:0];
              5'h0C: cfg_shift[k]         <= h_wdata[3:0];
              5'h0D: cfg_pull_th[k]       <= h_wdata[4:0];
              5'h0E: cfg_push_th[k]       <= h_wdata[4:0];
              5'h18: txf_lo               <= h_wdata;
              5'h1C: exec_lo              <= h_wdata;
              default: ;
            endcase
          end
        end
      end
    end
  end

  // RX FIFO high byte is latched when the low byte is read (and popped).
  always @(posedge clk) begin
    if (!rst_n) rxf_hi <= 8'd0;
    else if (h_rd_reg && h_is_sm && h_off == 5'h1A) rxf_hi <= sm_rx_rdata[h_sm][15:8];
  end

  // ------------------------------------------------------------------
  // Register reads
  // ------------------------------------------------------------------
  always @(*) begin
    h_rdata = 8'h00;
    if (h_space) begin
      h_rdata = h_raddr[0] ? h_imem_rd[15:8] : h_imem_rd[7:0];
    end else if (!r_is_sm) begin
      case (h_raddr)
        8'h00: h_rdata = 8'hB1;                 // ID
        8'h01: h_rdata = VERSION;
        8'h02: h_rdata = N_SM;
        8'h03: h_rdata = IMEM_N;
        8'h04: h_rdata = {{(8-N_SM){1'b0}}, cfg_enable};
        8'h07: h_rdata = flags;
        8'h08: h_rdata = gpio_in[7:0];
        8'h09: h_rdata = gpio_in[15:8];
        8'h0A: h_rdata = {4'd0, gpio_in[19:16]};
        8'h0C: h_rdata = gpio_out[15:8];
        8'h0D: h_rdata = {4'd0, gpio_out[19:16]};
        8'h0E: h_rdata = {4'd0, gpio_oe[19:16]};
        8'h0F: h_rdata = {7'd0, cfg_infilt};
        default: ;
      endcase
    end else begin
      case (r_off)
        5'h00: h_rdata = cfg_div_int[r_sm][7:0];
        5'h01: h_rdata = cfg_div_int[r_sm][15:8];
        5'h02: h_rdata = cfg_div_frac[r_sm];
        5'h03: h_rdata = {3'd0, cfg_out_base[r_sm]};
        5'h04: h_rdata = {3'd0, cfg_out_count[r_sm]};
        5'h05: h_rdata = {3'd0, cfg_set_base[r_sm]};
        5'h06: h_rdata = {5'd0, cfg_set_count[r_sm]};
        5'h07: h_rdata = {3'd0, cfg_in_base[r_sm]};
        5'h08: h_rdata = cfg_sideset[r_sm];
        5'h09: h_rdata = {3'd0, cfg_jmp_pin[r_sm]};
        5'h0A: h_rdata = {{(8-IMEM_AW){1'b0}}, cfg_wrap_top[r_sm]};
        5'h0B: h_rdata = {{(8-IMEM_AW){1'b0}}, cfg_wrap_bot[r_sm]};
        5'h0C: h_rdata = {4'd0, cfg_shift[r_sm]};
        5'h0D: h_rdata = {3'd0, cfg_pull_th[r_sm]};
        5'h0E: h_rdata = {3'd0, cfg_push_th[r_sm]};
        5'h0F: h_rdata = {{(8-IMEM_AW){1'b0}}, sm_pc[r_sm]};
        5'h10: h_rdata = sm_status[r_sm];
        5'h11: h_rdata = sm_x[r_sm][7:0];
        5'h12: h_rdata = sm_x[r_sm][15:8];
        5'h13: h_rdata = sm_y[r_sm][7:0];
        5'h14: h_rdata = sm_y[r_sm][15:8];
        5'h15: h_rdata = sm_t[r_sm][7:0];
        5'h16: h_rdata = sm_t[r_sm][15:8];
        5'h17: h_rdata = {1'b0, sm_rx_level[r_sm], 1'b0, sm_tx_level[r_sm]};
        5'h1A: h_rdata = sm_rx_rdata[r_sm][7:0];
        5'h1B: h_rdata = rxf_hi;
        default: ;
      endcase
    end
  end

endmodule
