// Round-robin whole-query dispatcher with a pre-CU killed-response bypass.
module cse_dispatcher
  import ple_cse_pkg::*;
  import cse_cu_pkg::*;
#(
    parameter int unsigned N_CU = 4,
    parameter bit KILL_ENABLE = 1'b1
) (
    input  logic                                      clk,
    input  logic                                      rst_n,
    input  logic                                      in_valid,
    output logic                                      in_ready,
    input  cse_query_meta_t                           in_payload,
    input  logic                                      in_killed,
    input  logic [(1<<TAG_W)-1:0][KILL_GEN_W-1:0]    kill_gen,
    output logic [N_CU-1:0]                          cu_valid,
    input  logic [N_CU-1:0]                          cu_ready,
    output cse_query_meta_t                           cu_payload[N_CU],
    output logic                                      cancel_valid,
    input  logic                                      cancel_ready,
    output resp_t                                     cancel_payload,
    output logic                                      waiting_for_cu
);

  localparam int CU_W = (N_CU <= 1) ? 1 : $clog2(N_CU);

  logic hold_valid, hold_killed, target_valid;
  cse_query_meta_t hold_payload;
  logic [CU_W-1:0] target_idx, rr_ptr, select_idx;
  logic select_found, held_cancelled, cu_fire, cancel_fire, release_hold;

  if (N_CU < 1) begin : g_bad_ncu
    $error("cse_dispatcher: N_CU must be at least one");
  end

  assign held_cancelled = hold_killed ||
      (KILL_ENABLE && hold_valid &&
       (hold_payload.birth_gen != kill_gen[hold_payload.query.tag]));

  always_comb begin : p_scan
    int candidate;
    select_found = 1'b0;
    select_idx = '0;
    for (int offset = 0; offset < N_CU; ++offset) begin
      candidate = int'(rr_ptr) + offset;
      if (candidate >= N_CU) candidate = candidate - N_CU;
      if (!select_found && cu_ready[candidate]) begin
        select_found = 1'b1;
        select_idx = CU_W'(candidate);
      end
    end
  end

  always_comb begin
    cu_valid = '0;
    for (int i = 0; i < N_CU; ++i) cu_payload[i] = hold_payload;
    if (hold_valid && target_valid) cu_valid[target_idx] = 1'b1;
  end

  assign cancel_valid = hold_valid && !target_valid && held_cancelled;
  assign cancel_payload = '{tag:hold_payload.query.tag, user:hold_payload.query.user,
                            status:ST_KILLED};
  assign cu_fire = hold_valid && target_valid && cu_ready[target_idx];
  assign cancel_fire = cancel_valid && cancel_ready;
  assign release_hold = cu_fire || cancel_fire;
  assign in_ready = !hold_valid || release_hold;
  assign waiting_for_cu = hold_valid && !held_cancelled &&
      ((!target_valid && !select_found) || (target_valid && !cu_ready[target_idx]));

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      hold_valid   <= 1'b0;
      hold_killed  <= 1'b0;
      hold_payload <= '0;
      target_valid <= 1'b0;
      target_idx   <= '0;
      rr_ptr       <= '0;
    end else begin
      if (cu_fire) rr_ptr <= (target_idx == CU_W'(N_CU-1)) ? '0 : target_idx + 1'b1;

      if (release_hold) begin
        hold_valid   <= 1'b0;
        hold_killed  <= 1'b0;
        target_valid <= 1'b0;
      end else if (hold_valid && !target_valid && held_cancelled) begin
        hold_killed <= 1'b1;
      end else if (hold_valid && !target_valid && select_found) begin
        target_valid <= 1'b1;
        target_idx   <= select_idx;
      end

      if (in_valid && in_ready) begin
        hold_valid   <= 1'b1;
        hold_payload <= in_payload;
        hold_killed  <= in_killed;
        target_valid <= 1'b0;
      end
    end
  end

endmodule
