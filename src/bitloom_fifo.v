/*
 * BitLoom: 4-deep synchronous FIFO used for the TX and RX paths of each
 * state machine. Both ports live in the same clock domain.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

module bitloom_fifo #(
    parameter W = 16,
    parameter AW = 2               // depth = 2**AW
) (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          clr,
    input  wire          push,
    input  wire [W-1:0]  wdata,
    input  wire          pop,
    output wire [W-1:0]  rdata,
    output wire          full,
    output wire          empty,
    output wire [AW:0]   level
);
  localparam D = 1 << AW;

  reg [W-1:0]  mem [0:D-1];
  reg [AW-1:0] wp, rp;
  reg [AW:0]   cnt;

  assign full  = cnt[AW];
  assign empty = (cnt == 0);
  assign level = cnt;
  assign rdata = mem[rp];

  wire do_push = push && !full;
  wire do_pop  = pop  && !empty;

  always @(posedge clk) begin
    if (!rst_n || clr) begin
      wp  <= 0;
      rp  <= 0;
      cnt <= 0;
    end else begin
      if (do_push) begin
        mem[wp] <= wdata;
        wp <= wp + 1'b1;
      end
      if (do_pop) rp <= rp + 1'b1;
      case ({do_push, do_pop})
        2'b10: cnt <= cnt + 1'b1;
        2'b01: cnt <= cnt - 1'b1;
        default: ;
      endcase
    end
  end

`ifdef FORMAL
  reg f_past_valid;
  initial f_past_valid = 0;
  always @(posedge clk) f_past_valid <= 1;

  always @(posedge clk) begin
    // Occupancy is always within bounds.
    assert(cnt <= D);
    // Pointers and count agree.
    assert(wp - rp == cnt[AW-1:0]);
    if (f_past_valid && $past(rst_n) && !$past(clr)) begin
      // Popping an empty FIFO or pushing a full one never moves a pointer.
      if ($past(empty) && !$past(push)) assert(rp == $past(rp));
      if ($past(full)  && !$past(pop))  assert(wp == $past(wp));
      // Count moves by at most one per cycle.
      assert(cnt == $past(cnt) || cnt == $past(cnt) + 1 || cnt == $past(cnt) - 1);
    end
  end
`endif

endmodule
