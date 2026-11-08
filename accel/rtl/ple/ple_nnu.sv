// NNU: streaming brute-force nearest neighbour (golden fx_plan.c nnu).
// The query is held for K PEs; one JOINT row (K nodes) streams in per cycle. Ties keep the lowest node
// index: the pairwise reduction takes the right (higher-index) side only when strictly smaller, and the
// running minimum updates only on strictly less.
// Pipeline: issue row -> tree read -> squares -> 7-term adder tree -> K-way min tree -> running min.
// done pulses ceil(n/K) + 5 cycles after the start cycle. The controller guarantees n >= 1 (the root
// exists); n = 0 returns idx 0, d2 all ones (golden returns INT64_MAX, never used).
module ple_nnu
  import ple_pkg::*;
#(
    parameter int K = 4                                  // PEs, power of 2, >= 2
) (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,                    // ignored while busy
    input  logic               tree,
    input  conf_t              q,
    input  logic [CNT_W-1:0]   n,                        // nodes in the tree (1..NODE_CAP)
    output logic               busy,
    output logic               done,                     // one-cycle pulse, idx / d2 valid from then on
    output logic [IDX_W-1:0]   idx,
    output logic [D2_W-1:0]    d2,
    // TREE port B
    output logic               nn_en,
    output logic               nn_tree,
    output logic [IDX_W-1:0]   nn_row,
    input  conf_t [K-1:0]      nn_q
);

  localparam int LW   = $clog2(K);
  localparam int SQ_W = 2 * (JW + 1);                    // (17-bit diff)^2

  if (K < 2 || (1 << LW) != K) begin : g_bad_k
    $error("ple_nnu: K must be a power of 2 and >= 2");
  end

  conf_t            qh;
  logic [CNT_W-1:0] nh;
  logic [IDX_W-1:0] row, rows;                           // rows = ceil(n / K)
  logic             issuing, tree_h;

  // control flags (reset) and data (no reset)
  logic             s1_v, s2_v, s3_v, s4_v, s1_last, s2_last, s3_last, s4_last;
  logic [IDX_W-1:0] s1_row, s2_row, s3_row;

  // ---------------------------------------------------------------- issue
  assign nn_en   = issuing;
  assign nn_tree = tree_h;
  assign nn_row  = row;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      issuing <= 1'b0;
      busy    <= 1'b0;
    end else if (start && !busy) begin
      busy    <= 1'b1;
      issuing <= (n != 0);
    end else begin
      if (issuing && row + 1'b1 == rows) issuing <= 1'b0;
      if (done) busy <= 1'b0;
    end
  end

  always_ff @(posedge clk) begin
    if (start && !busy) begin
      tree_h <= tree;
      qh     <= q;
      nh     <= n;
      rows   <= IDX_W'((32'(n) + K - 1) / K);
      row    <= '0;
    end else if (issuing) row <= row + 1'b1;
  end

  // ---------------------------------------------------------------- stage 1: tree data arrives
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      s1_v <= 1'b0; s1_last <= 1'b0;
    end else begin
      s1_v    <= issuing;
      s1_last <= issuing && (row + 1'b1 == rows);
    end
    s1_row <= row;
  end

  // ---------------------------------------------------------------- stage 2: squares
  logic [K-1:0][DIM-1:0][SQ_W-1:0] sq;
  logic [K-1:0]                    s2_lv;                 // lane holds a real node
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      s2_v <= 1'b0; s2_last <= 1'b0;
    end else begin
      s2_v <= s1_v; s2_last <= s1_last;
    end
    s2_row <= s1_row;
    for (int l = 0; l < K; ++l) begin
      s2_lv[l] <= (32'(s1_row) * K + l) < 32'(nh);
      for (int j = 0; j < DIM; ++j) begin
        logic signed [JW:0] dd;
        dd = (JW+1)'(signed'(qh[j])) - (JW+1)'(signed'(nn_q[l][j]));
        sq[l][j] <= SQ_W'(dd * dd);
      end
    end
  end

  // ---------------------------------------------------------------- stage 3: 7-term adder tree
  logic [K-1:0][D2_W-1:0] sum;
  logic [K-1:0]           s3_lv;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      s3_v <= 1'b0; s3_last <= 1'b0;
    end else begin
      s3_v <= s2_v; s3_last <= s2_last;
    end
    s3_row <= s2_row;
    s3_lv  <= s2_lv;
    for (int l = 0; l < K; ++l) begin
      logic [D2_W-1:0] a01, a23, a45, a0123, a456;
      a01   = D2_W'(sq[l][0]) + D2_W'(sq[l][1]);
      a23   = D2_W'(sq[l][2]) + D2_W'(sq[l][3]);
      a45   = D2_W'(sq[l][4]) + D2_W'(sq[l][5]);
      a0123 = a01 + a23;
      a456  = a45 + D2_W'(sq[l][6]);
      sum[l] <= a0123 + a456;
    end
  end

  // ---------------------------------------------------------------- stage 4: K-way minimum tree
  // level 0 holds the lanes; each level halves the candidates, keeping the left one unless the right is
  // valid and strictly smaller (left always has the lower node index)
  logic [D2_W-1:0]  rb_d2;
  logic [IDX_W-1:0] rb_idx;
  logic             rb_v;
  always_ff @(posedge clk) begin
    logic [K-1:0][D2_W-1:0] cd;
    logic [K-1:0][LW-1:0]   cl;
    logic [K-1:0]           cv;
    if (!rst_n) begin
      s4_v <= 1'b0; s4_last <= 1'b0; rb_v <= 1'b0;
    end else begin
      s4_v <= s3_v; s4_last <= s3_last;
      rb_v <= s3_v && (|s3_lv);
    end
    for (int l = 0; l < K; ++l) begin cd[l] = sum[l]; cl[l] = LW'(l); cv[l] = s3_lv[l]; end
    for (int w = K / 2; w >= 1; w = w / 2)
      for (int i = 0; i < w; ++i) begin
        logic take_r;
        take_r = cv[2*i+1] && (!cv[2*i] || cd[2*i+1] < cd[2*i]);
        cd[i] = take_r ? cd[2*i+1] : cd[2*i];
        cl[i] = take_r ? cl[2*i+1] : cl[2*i];
        cv[i] = cv[2*i] || cv[2*i+1];
      end
    rb_d2  <= cd[0];
    rb_idx <= IDX_W'({s3_row[IDX_W-LW-1:0], cl[0]});
  end

  // ---------------------------------------------------------------- stage 5: running minimum
  logic best_v;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      done <= 1'b0; idx <= '0; d2 <= '1; best_v <= 1'b0;
    end else begin
      done <= 1'b0;
      if (start && !busy) begin
        idx <= '0; d2 <= '1; best_v <= 1'b0;
        if (n == 0) done <= 1'b1;
      end else if (s4_v) begin
        if (rb_v && (!best_v || rb_d2 < d2)) begin
          d2 <= rb_d2; idx <= rb_idx; best_v <= 1'b1;
        end
        if (s4_last) done <= 1'b1;
      end
    end
  end

  logic unused;
  assign unused = ^{s3_row[IDX_W-1:IDX_W-LW]};

endmodule
