// SMP: integer Halton sampler (golden fx_plan.c smp_reset / smp_next).
// The sequence does not depend on the tree, so the next sample is generated in the background and held
// ready (valid) until the controller takes it. STEP (1 cycle, all joints) + MAP (1 joint per cycle,
// one 16x16 multiplier, 3-stage pipeline): a new sample is ready 10 cycles after init / take.
// `shift` is read combinationally in every MAP cycle: it must stay stable for the whole problem
// (from init until the last sample is taken).
module ple_smp
  import ple_pkg::*;
(
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    init,                // restart the sequence (start of a problem)
    input  logic [DIM-1:0][31:0]    shift,               // Cranley-Patterson rotation per joint (0 = none)
    input  logic                    take,                // controller consumed q (only while valid)
    output logic                    valid,
    output conf_t                   q
);

  typedef enum logic [1:0] {S_IDLE, S_STEP, S_MAP, S_HOLD} state_e;
  state_e state;

  logic [DIM-1:0][H_DIGITS-1:0][4:0] dig;               // base-b digits (b <= 19)
  logic [DIM-1:0][31:0]              u;                  // sum_k d_k * inv_k, Q0.32
  logic [2:0]                        mj;                 // joint being mapped

  // u step per lowest free digit, constant table built at elaboration (ple_pkg::h_delta)
  logic [DIM-1:0][H_DIGITS-1:0][31:0] delta;
  for (genvar j = 0; j < DIM; ++j) begin : g_dj
    for (genvar k = 0; k < H_DIGITS; ++k) begin : g_dk
      localparam logic [31:0] D = h_delta(H_BASE[j], k);
      assign delta[j][k] = D;
    end
  end

  // lowest digit below b-1 per joint (H_DIGITS if none; never reached within 2^32 samples)
  function automatic int first_free(input logic [H_DIGITS-1:0][4:0] d, input int unsigned b);
    for (int k = 0; k < H_DIGITS; ++k) if (32'(d[k]) != b - 1) return k;
    return H_DIGITS;
  endfunction

  // MAP: q_j = a_j + (((u_j + shift_j) >> 16) * m_j) >> 16, wrapped to 16 bits as in the model.
  // Pipeline: stage A adds the rotation, stage B multiplies, stage C adds a_j and writes q_j.
  logic             a_v, b_v;
  logic [2:0]       a_j, b_j;
  logic [15:0]      a_uu;                                // (u + shift) >> 16
  logic [31:0]      b_prod;
  always_ff @(posedge clk) begin
    a_uu   <= 16'((u[mj] + shift[mj]) >> 16);
    a_j    <= mj;
    b_prod <= 32'(a_uu) * 32'(SMP_M[a_j]);
    b_j    <= a_j;
  end
  logic unused_bits;
  assign unused_bits = ^b_prod[15:0];

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      state <= S_IDLE;
      valid <= 1'b0;
      mj    <= '0;
      dig   <= '0;
      u     <= '0;
      q     <= '0;
      a_v   <= 1'b0;
      b_v   <= 1'b0;
    end else if (init) begin
      state <= S_STEP;
      valid <= 1'b0;
      dig   <= '0;
      u     <= '0;
      a_v   <= 1'b0;
      b_v   <= 1'b0;
    end else begin
      // MAP pipeline valid flags and the final stage (q_j = a_j + prod >> 16)
      a_v <= (state == S_MAP);
      b_v <= a_v;
      if (b_v) begin
        q[b_j] <= SMP_A[b_j] + b_prod[31:16];
        if (b_j == 3'(DIM - 1)) valid <= 1'b1;
      end
      unique case (state)
        S_IDLE: ;
        S_STEP: begin
          for (int j = 0; j < DIM; ++j) begin
            int k;
            k = first_free(dig[j], H_BASE[j]);
            for (int i = 0; i < H_DIGITS; ++i)
              if (i < k) dig[j][i] <= '0;
              else if (i == k) dig[j][i] <= dig[j][i] + 5'd1;
            if (k < H_DIGITS) u[j] <= u[j] + delta[j][k];
          end
          mj    <= '0;
          state <= S_MAP;
        end
        S_MAP: begin                                     // issue joint mj into the pipeline
          if (mj == 3'(DIM - 1)) begin
            state <= S_HOLD;
            mj    <= '0;                              // keep the index in range outside MAP
          end else mj <= mj + 3'd1;
        end
        S_HOLD: if (take && valid) begin               // wait until the pipeline has written q_6
          valid <= 1'b0;
          state <= S_STEP;
        end
      endcase
    end
  end

endmodule
