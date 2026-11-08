// DIV7: seven signed floor divisions by one shared unsigned divisor (golden fx.h fx_fdiv).
// q_j = floor(a_j / b), r_j = a_j - q_j * b (0 <= r_j < b). Restoring division of |a_j|, one quotient bit
// per cycle, then the floor sign fix. Used for EXTEND scaling (a = d * range_h, b = nd_h), CONNECT step
// divmod (a = ovec, b = n_ext) and the scalar n_ext / inc_h divisions (lane 0).
// done rises at the (AW + 1)th clock edge after the start edge. b must be non-zero.
module ple_div7
  import ple_pkg::*;
#(
    parameter int AW = 50,                               // dividend width (signed)
    parameter int BW = 32                                // divisor width (unsigned)
) (
    input  logic                       clk,
    input  logic                       rst_n,
    input  logic                       start,            // ignored while busy
    input  logic [DIM-1:0][AW-1:0]     a,                // signed dividends
    input  logic [BW-1:0]              b,
    output logic                       busy,
    output logic                       done,             // one-cycle pulse, q / r valid from then on
    output logic [DIM-1:0][AW-1:0]     q,                // signed floor quotients
    output logic [DIM-1:0][BW-1:0]     r                 // remainders, 0 <= r < b
);

  logic [DIM-1:0]          neg;
  logic [DIM-1:0][AW-1:0]  mag;                          // |a| shifted out MSB first
  logic [DIM-1:0][AW-1:0]  qm;                           // magnitude quotient
  logic [DIM-1:0][BW-1:0]  pr;                           // partial remainder
  logic [BW-1:0]           bh;
  logic [6:0]              step;
  logic                    fix;                          // sign-fix cycle

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      busy <= 1'b0;
      done <= 1'b0;
      fix  <= 1'b0;
    end else begin
      done <= 1'b0;
      fix  <= 1'b0;
      if (start && !busy) begin
        busy <= 1'b1;
        step <= '0;
      end else if (busy && !fix) begin
        step <= step + 7'd1;
        if (step == 7'(AW - 1)) fix <= 1'b1;
      end else if (fix) begin
        busy <= 1'b0;
        done <= 1'b1;
      end
    end
  end

  always_ff @(posedge clk) begin
    if (start && !busy) begin
      bh <= b;
      for (int j = 0; j < DIM; ++j) begin
        logic signed [AW-1:0] sa;
        sa = signed'(a[j]);
        neg[j] <= sa[AW-1];
        mag[j] <= sa[AW-1] ? AW'(-sa) : AW'(sa);
        qm[j]  <= '0;
        pr[j]  <= '0;
      end
    end else if (busy && !fix) begin
      for (int j = 0; j < DIM; ++j) begin
        logic [BW:0] t;
        t = {pr[j], mag[j][AW-1]};
        if (t >= {1'b0, bh}) begin
          pr[j] <= BW'(t - {1'b0, bh});
          qm[j] <= {qm[j][AW-2:0], 1'b1};
        end else begin
          pr[j] <= BW'(t);
          qm[j] <= {qm[j][AW-2:0], 1'b0};
        end
        mag[j] <= {mag[j][AW-2:0], 1'b0};
      end
    end else if (fix) begin
      // floor: a negative dividend with a remainder rounds one further down, remainder b - pr
      for (int j = 0; j < DIM; ++j) begin
        logic nz;
        nz   = (pr[j] != 0);
        q[j] <= neg[j] ? (nz ? ~qm[j] : ~qm[j] + 1'b1) : qm[j];   // -m-1 = ~m, -m = ~m+1
        r[j] <= (neg[j] && nz) ? bh - pr[j] : pr[j];
      end
    end
  end

endmodule
