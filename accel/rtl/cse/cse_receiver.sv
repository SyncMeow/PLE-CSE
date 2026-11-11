// Locked round-robin receiver for CU responses and the front-end cancel source.
module cse_receiver
  import ple_cse_pkg::*;
#(
    parameter int unsigned N_SRC = 5
) (
    input  logic                 clk,
    input  logic                 rst_n,
    input  logic [N_SRC-1:0]     source_valid,
    output logic [N_SRC-1:0]     source_ready,
    input  resp_t                source_payload[N_SRC],
    output logic                 out_valid,
    input  logic                 out_ready,
    output resp_t                out_payload
);

  localparam int SRC_W = (N_SRC <= 1) ? 1 : $clog2(N_SRC);

  logic lock_valid, select_found;
  logic [SRC_W-1:0] lock_idx, rr_ptr, select_idx;
  logic out_fire;

  if (N_SRC < 1) begin : g_bad_nsrc
    $error("cse_receiver: N_SRC must be at least one");
  end

  always_comb begin : p_scan
    logic [SRC_W:0] candidate;                       // narrow index math: a 32-bit int crashes Vivado 2024.1 synth
    select_found = 1'b0;
    select_idx = '0;
    for (int offset = 0; offset < N_SRC; ++offset) begin
      candidate = {1'b0, rr_ptr} + (SRC_W + 1)'(offset);
      if (candidate >= (SRC_W + 1)'(N_SRC)) candidate = candidate - (SRC_W + 1)'(N_SRC);
      if (!select_found && source_valid[candidate[SRC_W-1:0]]) begin
        select_found = 1'b1;
        select_idx = candidate[SRC_W-1:0];
      end
    end
  end

  assign out_valid = lock_valid;
  assign out_payload = source_payload[lock_idx];
  assign out_fire = out_valid && out_ready;
  always_comb begin
    source_ready = '0;
    if (lock_valid) source_ready[lock_idx] = out_ready;
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      lock_valid <= 1'b0;
      lock_idx   <= '0;
      rr_ptr     <= '0;
    end else begin
      if (out_fire) begin
        lock_valid <= 1'b0;
        rr_ptr <= (lock_idx == SRC_W'(N_SRC-1)) ? '0 : lock_idx + 1'b1;
      end else if (!lock_valid && select_found) begin
        lock_valid <= 1'b1;
        lock_idx   <= select_idx;
      end
    end
  end

endmodule
