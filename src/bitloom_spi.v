/*
 * BitLoom host port: SPI slave (mode 0, MSB first), synchronised into clk.
 *
 * Transaction (CS low for the whole frame):
 *   byte 0  command   bit 7 = 1 write / 0 read, bit 0 = 1 instruction memory / 0 registers
 *   byte 1  address   auto-increments after every data byte
 *   byte 2+ data      written to, or read from, the selected space
 *
 * SCK must be no faster than clk/4 (two clk periods per SCK half period).
 *
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

module bitloom_spi (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       sck,
    input  wire       csn,
    input  wire       mosi,
    output reg        miso,

    output reg        wr_en,     // one-cycle pulse, wdata/addr/space valid
    output reg        rd_en,     // one-cycle pulse: rdata for addr/space is being consumed
    output reg        space,     // 0 = registers, 1 = instruction memory
    output reg  [7:0] addr,      // address that wr_en / rd_en refer to
    output reg  [7:0] raddr,     // running address: drives the read-data mux
    output reg  [7:0] wdata,
    input  wire [7:0] rdata
);
  // Two-stage synchronisers
  reg [1:0] sck_s, csn_s, mosi_s;
  always @(posedge clk) begin
    if (!rst_n) begin
      sck_s <= 2'b00; csn_s <= 2'b11; mosi_s <= 2'b00;
    end else begin
      sck_s  <= {sck_s[0], sck};
      csn_s  <= {csn_s[0], csn};
      mosi_s <= {mosi_s[0], mosi};
    end
  end

  reg  sck_d;
  wire sck_rise = sck_s[1] & ~sck_d;
  wire sck_fall = ~sck_s[1] & sck_d;
  wire cs_act   = ~csn_s[1];

  reg [2:0] bitcnt;
  reg [1:0] phase;      // 0 = command, 1 = address, 2 = data
  reg [7:0] shift_in;
  reg [7:0] shift_out;
  reg       is_write;

  wire [7:0] byte_in = {shift_in[6:0], mosi_s[1]};

  always @(posedge clk) begin
    if (!rst_n) begin
      sck_d <= 1'b0; bitcnt <= 3'd0; phase <= 2'd0; shift_in <= 8'd0;
      shift_out <= 8'd0; is_write <= 1'b0; space <= 1'b0; addr <= 8'd0; raddr <= 8'd0;
      wdata <= 8'd0; wr_en <= 1'b0; rd_en <= 1'b0; miso <= 1'b0;
    end else begin
      sck_d <= sck_s[1];
      wr_en <= 1'b0;
      rd_en <= 1'b0;

      if (!cs_act) begin
        bitcnt <= 3'd0;
        phase  <= 2'd0;
        miso   <= 1'b0;
      end else begin
        if (sck_rise) begin
          shift_in <= byte_in;
          bitcnt   <= bitcnt + 3'd1;
          if (bitcnt == 3'd7) begin
            case (phase)
              2'd0: begin
                is_write <= byte_in[7];
                space    <= byte_in[0];
                phase    <= 2'd1;
              end
              2'd1: begin
                raddr <= byte_in;
                phase <= 2'd2;
              end
              default: begin
                if (is_write) begin
                  wr_en <= 1'b1;
                  wdata <= byte_in;
                  addr  <= raddr;
                  raddr <= raddr + 8'd1;
                end
              end
            endcase
          end
        end

        if (sck_fall) begin
          if (phase == 2'd2 && !is_write && bitcnt == 3'd0) begin
            // First falling edge of a read data byte: fetch and present bit 7.
            shift_out <= {rdata[6:0], 1'b0};
            miso      <= rdata[7];
            rd_en     <= 1'b1;
            addr      <= raddr;
            raddr     <= raddr + 8'd1;
          end else begin
            miso      <= shift_out[7];
            shift_out <= {shift_out[6:0], 1'b0};
          end
        end
      end
    end
  end

endmodule
