// PLE-internal definitions (golden model: Platform/sw_rrtc/fx, records/20261108-fx-golden.md).
/* verilator lint_off UNUSEDPARAM */  // each module uses only part of the package
package ple_pkg;

  localparam int DIM       = 7;                   // Panda joints
  localparam int JW        = 16;                  // joint angle s16, LSB = 2^-12 rad
  localparam int NODE_CAP  = 1024;                // nodes per tree
  localparam int IDX_W     = 10;                  // node index
  localparam int CNT_W     = 11;                  // node count 0..1024
  localparam int D2_W      = 37;                  // squared joint distance: 7 x (17-bit diff)^2
  localparam int RAD_W     = 32;                  // DD radius, LSB = 2^-24 rad
  localparam logic [RAD_W-1:0] RAD_INF = '1;

  typedef logic signed [JW-1:0] joint_t;
  typedef joint_t [DIM-1:0] conf_t;               // one configuration, packed (112 bits)

  // ---- SMP: integer Halton (golden smp_reset / smp_next)
  localparam int H_DIGITS = 24;
  localparam int unsigned H_BASE [DIM] = '{3, 5, 7, 11, 13, 17, 19};
  // floor(lim_a * 2^12) and floor(lim_m * 2^12) of the Panda joint limits (golden smp_a / smp_m)
  localparam logic signed [JW-1:0] SMP_A [DIM] = '{-16'sd12154, -16'sd7507, -16'sd12154, -16'sd12868,
                                                   -16'sd12154, -16'sd358, -16'sd12154};
  localparam logic [15:0] SMP_M [DIM] = '{16'd24306, 16'd15012, 16'd24306, 16'd13225, 16'd24306, 16'd16013,
                                          16'd24306};

  // inv_k = floor(2^32 / b^(k+1)), 0 once b^(k+1) > 2^32
  function automatic logic [31:0] h_inv(input int unsigned b, input int k);
    logic [63:0] p = 64'(b);
    for (int i = 0; i < k; ++i) if (p <= 64'h1_0000_0000) p = p * 64'(b);
    return (p > 64'h1_0000_0000) ? 32'd0 : 32'((64'h1_0000_0000) / p);
  endfunction

  // Step when digit k is the lowest digit below b-1: u += inv_k - (b-1) * sum_{i<k} inv_i (mod 2^32)
  function automatic logic [31:0] h_delta(input int unsigned b, input int k);
    logic [31:0] d = h_inv(b, k);
    for (int i = 0; i < k; ++i) d = d - 32'(b - 1) * h_inv(b, i);
    return d;
  endfunction

endpackage
/* verilator lint_on UNUSEDPARAM */
