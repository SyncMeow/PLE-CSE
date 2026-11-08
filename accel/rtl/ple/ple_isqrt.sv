// ISQRT: floor(sqrt(x)) of a 64-bit unsigned value, one result bit per cycle (golden fx.h fx_isqrt).
// Used for edge lengths: len_h = isqrt(d2 << 24). done rises at the 32nd clock edge after the start edge.
module ple_isqrt (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,                           // ignored while busy
    input  logic [63:0] x,
    output logic        busy,
    output logic        done,                            // one-cycle pulse, root valid from then on
    output logic [31:0] root
);

  logic [63:0] rem;                                      // remaining radicand
  logic [63:0] res;                                      // root being formed (shifted form)
  logic [63:0] bitv;                                     // current bit, starts at 1 << 62
  logic [5:0]  step;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      busy <= 1'b0;
      done <= 1'b0;
    end else begin
      done <= 1'b0;
      if (start && !busy) begin
        busy <= 1'b1;
        step <= '0;
      end else if (busy) begin
        step <= step + 6'd1;
        if (step == 6'd31) begin
          busy <= 1'b0;
          done <= 1'b1;
        end
      end
    end
  end

  // classic digit-by-digit square root: same iteration as the golden model, without its early skip
  always_ff @(posedge clk) begin
    if (start && !busy) begin
      rem  <= x;
      res  <= '0;
      bitv <= 64'h4000_0000_0000_0000;
    end else if (busy) begin
      if (rem >= res + bitv) begin
        rem <= rem - (res + bitv);
        res <= (res >> 1) + bitv;
      end else begin
        res <= res >> 1;
      end
      bitv <= bitv >> 2;
    end
  end

  assign root = res[31:0];

  logic unused;
  assign unused = ^res[63:32];

endmodule
