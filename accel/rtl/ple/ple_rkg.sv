// RKG: rake generator (golden fx_plan.c edge_ok). Poses start + floor(vec * m / (8n)), m = (i+1)n - k,
// k = 0..n-1 (outer), i = 0..7 (inner); n = max(1, ceil(len_h * 4 / 2^24)).
// Exact DDA per joint: within a row m grows by n (add floor(vec/8), remainder n*(vec&7)); a new row starts
// at m-1 of the previous row start (subtract divmod(vec, 8n), one divider per joint at edge start).
// One pose per cycle on a valid/ready stream; `kill` drops the rest of the edge. A pose whose handshake
// falls in the same cycle as `kill` counts as not delivered. start is ignored while busy; kill wins over
// start in the same cycle. len_h must be < 1023 * 2^22 (n <= 1023); Panda edges are < 14 rad = 2.4e8.
module ple_rkg
  import ple_pkg::*;
#(
    parameter int VW = 18                                // vec width (signed); |vec| < 2^17
) (
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    start,               // ignored while busy
    input  conf_t                   s_q,                 // edge start configuration
    input  logic [DIM-1:0][VW-1:0]  vec,                 // signed joint vector
    input  logic [31:0]             len_h,               // edge length, 2^-24 rad
    input  logic                    kill,                // drop the rest of the edge
    output logic                    busy,
    output logic [15:0]             n_out,               // rake rows (8 * n poses), valid while busy
    // pose stream
    output logic                    p_valid,
    input  logic                    p_ready,
    output conf_t                   p_q,
    output logic                    p_last
);

  localparam int DW = 19;                                // den = 8n, n < 2^16 -> fits in 19 bits for n < 2^16
  localparam int RW = DW + 1;                            // signed remainder workspace

  typedef enum logic [1:0] {S_IDLE, S_DIV, S_ROW, S_DONE} state_e;
  state_e state;

  conf_t                       sq;
  logic [15:0]                 n;
  logic [DW-1:0]               den;
  logic [DIM-1:0][VW-1:0]      v;
  logic [DIM-1:0][VW-1:0]      qn;                       // floor(vec / 8)
  logic [DIM-1:0][DW-1:0]      rn;                       // n * (vec & 7) < den
  logic [DIM-1:0][VW-1:0]      q1;                       // floor(vec / den)
  logic [DIM-1:0][DW-1:0]      r1;                       // vec mod den, 0 <= r1 < den
  logic [DIM-1:0][VW-1:0]      qs, qc;                   // row start / current quotient
  logic [DIM-1:0][RW-1:0]      rs, rc;                   // row start / current remainder (signed)
  logic [15:0]                 k;
  logic [2:0]                  i;

  // n = max(1, ceil(len_h * 4 / 2^24))
  logic [33:0] len4;
  logic [15:0] n_calc;
  always_comb begin
    len4   = {len_h, 2'b00} + 34'h0FF_FFFF;
    n_calc = (len4[33:24] == 0) ? 16'd1 : 16'(len4[33:24]);
  end
  logic unused_len;
  assign unused_len = ^len4[23:0];

  // ---------------------------------------------------------------- dividers: |vec| / den, restoring
  logic [DIM-1:0][VW-1:0] dq;                            // quotient bits being formed
  logic [DIM-1:0][DW-1:0] dr;                            // partial remainder
  logic [DIM-1:0][VW-1:0] dn;                            // |vec| shifted out MSB first
  logic [4:0]             dstep;

  assign n_out  = n;
  assign busy   = (state != S_IDLE);
  assign p_last = (k == n - 16'd1) && (i == 3'd7);

  // pose output: start + current quotient
  always_comb for (int j = 0; j < DIM; ++j) p_q[j] = JW'(signed'(sq[j]) + signed'(qc[j]));
  assign p_valid = (state == S_ROW);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      state <= S_IDLE;
    end else if (kill) begin
      state <= S_IDLE;
    end else begin
      unique case (state)
        S_IDLE: if (start) state <= S_DIV;
        S_DIV:  if (dstep == 5'(VW)) state <= S_ROW;
        S_ROW:  if (p_ready && p_last) state <= S_IDLE;
        default: state <= S_IDLE;
      endcase
    end
  end

  always_ff @(posedge clk) begin
    unique case (state)
      S_IDLE: if (start) begin
        logic [15:0] nn;
        nn = n_calc;
        sq    <= s_q;
        v     <= vec;
        n     <= nn;
        den   <= DW'({nn, 3'b000});
        dstep <= '0;
        k     <= '0;
        i     <= '0;
        for (int j = 0; j < DIM; ++j) begin
          logic signed [VW-1:0] sv;
          sv = signed'(vec[j]);
          qn[j] <= VW'(sv >>> 3);
          rn[j] <= DW'(32'(nn) * 32'(vec[j][2:0]));
          dn[j] <= sv[VW-1] ? VW'(-sv) : VW'(sv);
          dq[j] <= '0;
          dr[j] <= '0;
        end
      end
      S_DIV: begin
        if (dstep != 5'(VW)) begin
          for (int j = 0; j < DIM; ++j) begin
            logic [DW:0] t;
            t = {dr[j], dn[j][VW-1]};
            if (t >= {1'b0, den}) begin
              dr[j] <= DW'(t - {1'b0, den});
              dq[j] <= {dq[j][VW-2:0], 1'b1};
            end else begin
              dr[j] <= DW'(t);
              dq[j] <= {dq[j][VW-2:0], 1'b0};
            end
            dn[j] <= {dn[j][VW-2:0], 1'b0};
          end
          dstep <= dstep + 5'd1;
        end else begin
          // floor division with sign fix, then the first row start (m = n): q = floor(vec/8), r = n*(vec&7)
          for (int j = 0; j < DIM; ++j) begin
            logic neg, nz;
            neg = v[j][VW-1];
            nz  = (dr[j] != 0);
            q1[j] <= neg ? (nz ? ~dq[j] : ~dq[j] + 1'b1) : dq[j];   // -q-1 = ~q, -q = ~q+1
            r1[j] <= (neg && nz) ? den - dr[j] : dr[j];
            qs[j] <= qn[j];
            rs[j] <= RW'(rn[j]);
            qc[j] <= qn[j];
            rc[j] <= RW'(rn[j]);
          end
        end
      end
      S_ROW: if (p_ready) begin
        if (i == 3'd7) begin
          // next row: start one step lower (m - 1): subtract divmod(vec, den)
          for (int j = 0; j < DIM; ++j) begin
            logic signed [RW-1:0] r;
            logic signed [VW-1:0] qq;
            r  = signed'(rs[j]) - signed'(RW'(r1[j]));
            qq = signed'(qs[j]) - signed'(q1[j]);
            if (r < 0) begin
              r  = r + signed'(RW'(den));
              qq = qq - 1;
            end
            qs[j] <= VW'(qq);
            rs[j] <= RW'(r);
            qc[j] <= VW'(qq);
            rc[j] <= RW'(r);
          end
          i <= '0;
          k <= k + 16'd1;
        end else begin
          // same row: m + n
          for (int j = 0; j < DIM; ++j) begin
            logic signed [RW-1:0] r;
            logic signed [VW-1:0] qq;
            r  = signed'(rc[j]) + signed'(RW'(rn[j]));
            qq = signed'(qc[j]) + signed'(qn[j]);
            if (r >= signed'(RW'(den))) begin
              r  = r - signed'(RW'(den));
              qq = qq + 1;
            end
            qc[j] <= VW'(qq);
            rc[j] <= RW'(r);
          end
          i <= i + 3'd1;
        end
      end
      default: ;
    endcase
  end

endmodule
