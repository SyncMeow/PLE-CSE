// Complete CSE: kill-aware input queue, round-robin CU array, receiver, and response queue.
module cse_top
  import ple_cse_pkg::*;
  import cse_cu_pkg::*;
#(
    parameter int unsigned N_CU = 4,
    parameter int unsigned INPUT_FIFO_DEPTH = 8,
    parameter int unsigned RESPONSE_FIFO_DEPTH = 8,
    parameter bit KILL_ENABLE = 1'b1,
    parameter int unsigned CU_VOXEL_FIFO_DEPTH = 2,
    parameter int unsigned CU_CELL_FIFO_DEPTH = 2,
    parameter int unsigned CU_MAX_PTS_OUTSTANDING = 8
) (
    input  logic                      clk,
    input  logic                      rst_n,
    input  logic                      q_valid,
    output logic                      q_ready,
    input  logic [TAG_W-1:0]          q_tag,
    input  logic [USER_W-1:0]         q_user,
    input  logic                      q_kill_on_hit,
    input  logic                      q_env_buf,
    input  logic signed [COORD_W-1:0] q_cx,
    input  logic signed [COORD_W-1:0] q_cy,
    input  logic signed [COORD_W-1:0] q_cz,
    input  logic [RQ_W-1:0]           q_r,
    output logic                      r_valid,
    input  logic                      r_ready,
    output logic [TAG_W-1:0]          r_tag,
    output logic [USER_W-1:0]         r_user,
    output logic [1:0]                r_status,
    input  logic                      k_valid,
    output logic                      k_ready,
    input  logic                      k_all,
    input  logic [TAG_W-1:0]          k_tag,
    output logic                      cse_idle,
    output logic                      cse_err,
    output logic [N_CU-1:0]                    rd_buf,
    output logic [N_CU-1:0][DIR_AW-1:0]        dir_addr,
    input  logic [N_CU-1:0][15:0]              dir_data,
    output logic [N_CU-1:0][SLOT_W-1:0]        cell_addr,
    input  logic [N_CU-1:0][127:0]             cell_data,
    output logic [N_CU-1:0][PT_AW-1:0]         pt_addr,
    input  logic [N_CU-1:0][PT_DATA_W-1:0]     pt_data,
    input  logic [1:0]                          hdr_valid,
    input  logic [1:0]                          hdr_err,
    input  logic [1:0][PT_AW-1:0]               hdr_n_pts,
    input  logic [1:0][2:0][PT_W-1:0]          hdr_glo,
    input  logic [1:0][2:0][PT_W-1:0]          hdr_ghi
);

  localparam int NTAG = 1 << TAG_W;
  localparam int N_SRC = N_CU + 1;

  query_t flat_query;
  kill_t kill_payload;
  cse_query_meta_t top_query;
  logic [NTAG-1:0][KILL_GEN_W-1:0] kill_gen, kill_gen_eff;
  logic q_fire, k_fire, r_fire;

  logic qfifo_in_ready, qfifo_out_valid, qfifo_out_ready, qfifo_out_killed;
  cse_query_meta_t qfifo_out_payload;

  logic [N_CU-1:0] cu_q_valid, cu_q_ready;
  cse_query_meta_t cu_q_payload[N_CU];
  logic cancel_in_valid, cancel_in_ready;
  resp_t cancel_in_payload;
  logic dispatcher_waiting;
  logic cancel_out_valid, cancel_out_ready;
  resp_t cancel_out_payload;

  logic [N_CU-1:0] cu_r_valid, cu_r_ready, cu_idle, cu_err;
  resp_t cu_r_payload[N_CU];

  logic [N_SRC-1:0] source_valid, source_ready;
  resp_t source_payload[N_SRC];
  logic receiver_valid, receiver_ready;
  resp_t receiver_payload;
  logic response_out_valid, response_out_ready;
  resp_t response_out_payload;

  logic [31:0] outstanding;
  logic unused_status;

  if (N_CU < 1 || INPUT_FIFO_DEPTH < 1 || RESPONSE_FIFO_DEPTH < 1) begin : g_bad_config
    $error("cse_top: N_CU and FIFO depths must be at least one");
  end

  assign flat_query = '{tag:q_tag, user:q_user, kill_on_hit:q_kill_on_hit,
                        env_buf:q_env_buf, cx:q_cx, cy:q_cy, cz:q_cz, r:q_r};
  assign kill_payload = '{all:k_all, tag:k_tag};
  assign k_ready = rst_n;
  assign k_fire = k_valid && k_ready;

  always_comb begin
    for (int t = 0; t < NTAG; ++t) begin
      kill_gen_eff[t] = kill_gen[t];
      if (KILL_ENABLE && k_fire && (k_all || k_tag == t[TAG_W-1:0]))
        kill_gen_eff[t] = kill_gen[t] + 1'b1;
    end
  end

  assign top_query = '{query:flat_query, birth_gen:kill_gen_eff[q_tag]};
  assign q_ready = rst_n && qfifo_in_ready;
  assign q_fire = q_valid && q_ready;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      kill_gen <= '0;
    end else if (KILL_ENABLE && k_fire) begin
      for (int t = 0; t < NTAG; ++t)
        if (k_all || k_tag == t[TAG_W-1:0]) kill_gen[t] <= kill_gen[t] + 1'b1;
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk) begin
    if (rst_n && KILL_ENABLE && k_fire && outstanding != 0)
      for (int t = 0; t < NTAG; ++t)
        if ((k_all || k_tag == t[TAG_W-1:0]) && (&kill_gen[t]))
          $error("cse_top: kill generation wrapped with work outstanding");
  end
`endif

  cse_query_fifo #(.DEPTH(INPUT_FIFO_DEPTH), .KILL_ENABLE(KILL_ENABLE)) u_query_fifo (
      .clk, .rst_n,
      .in_valid(q_valid && rst_n), .in_ready(qfifo_in_ready), .in_payload(top_query),
      .kill_fire(k_fire), .kill_payload,
      .out_valid(qfifo_out_valid), .out_ready(qfifo_out_ready),
      .out_payload(qfifo_out_payload), .out_killed(qfifo_out_killed)
  );

  cse_dispatcher #(.N_CU(N_CU), .KILL_ENABLE(KILL_ENABLE)) u_dispatcher (
      .clk, .rst_n,
      .in_valid(qfifo_out_valid), .in_ready(qfifo_out_ready),
      .in_payload(qfifo_out_payload), .in_killed(qfifo_out_killed),
      .kill_gen(kill_gen_eff),
      .cu_valid(cu_q_valid), .cu_ready(cu_q_ready), .cu_payload(cu_q_payload),
      .cancel_valid(cancel_in_valid), .cancel_ready(cancel_in_ready),
      .cancel_payload(cancel_in_payload), .waiting_for_cu(dispatcher_waiting)
  );

  cse_fifo #(.PAYLOAD_T(resp_t), .DEPTH(1)) u_cancel_holder (
      .clk, .rst_n,
      .in_valid(cancel_in_valid), .in_ready(cancel_in_ready), .in_payload(cancel_in_payload),
      .out_valid(cancel_out_valid), .out_ready(cancel_out_ready), .out_payload(cancel_out_payload)
  );

  for (genvar i = 0; i < N_CU; ++i) begin : g_cu
    cse_cu #(
        .VOXEL_FIFO_DEPTH(CU_VOXEL_FIFO_DEPTH),
        .CELL_FIFO_DEPTH(CU_CELL_FIFO_DEPTH),
        .MAX_PTS_OUTSTANDING(CU_MAX_PTS_OUTSTANDING),
        .KILL_ENABLE(KILL_ENABLE)
    ) u_cu (
        .clk, .rst_n,
        .q_valid(cu_q_valid[i]), .q_ready(cu_q_ready[i]), .q_payload(cu_q_payload[i]),
        .r_valid(cu_r_valid[i]), .r_ready(cu_r_ready[i]), .r_payload(cu_r_payload[i]),
        .kill_gen(kill_gen_eff),
        .cse_idle(cu_idle[i]), .cse_err(cu_err[i]),
        .mem_rd_buf(rd_buf[i]),
        .mem_dir_addr(dir_addr[i]), .mem_dir_data(dir_data[i]),
        .mem_cell_addr(cell_addr[i]), .mem_cell_data(cell_data[i]),
        .mem_pt_addr(pt_addr[i]), .mem_pt_data(pt_data[i]),
        .hdr_valid, .hdr_err, .hdr_n_pts, .hdr_glo, .hdr_ghi
    );

    assign source_valid[i] = cu_r_valid[i];
    assign source_payload[i] = cu_r_payload[i];
    assign cu_r_ready[i] = source_ready[i];
  end

  assign source_valid[N_CU] = cancel_out_valid;
  assign source_payload[N_CU] = cancel_out_payload;
  assign cancel_out_ready = source_ready[N_CU];

  cse_receiver #(.N_SRC(N_SRC)) u_receiver (
      .clk, .rst_n,
      .source_valid, .source_ready, .source_payload,
      .out_valid(receiver_valid), .out_ready(receiver_ready), .out_payload(receiver_payload)
  );

  cse_fifo #(.PAYLOAD_T(resp_t), .DEPTH(RESPONSE_FIFO_DEPTH)) u_response_fifo (
      .clk, .rst_n,
      .in_valid(receiver_valid), .in_ready(receiver_ready), .in_payload(receiver_payload),
      .out_valid(response_out_valid), .out_ready(response_out_ready),
      .out_payload(response_out_payload)
  );

  assign r_valid = response_out_valid;
  assign response_out_ready = r_ready;
  assign r_tag = response_out_payload.tag;
  assign r_user = response_out_payload.user;
  assign r_status = response_out_payload.status;
  assign r_fire = r_valid && r_ready;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      outstanding <= '0;
      cse_err <= 1'b0;
    end else begin
      unique case ({q_fire, r_fire})
        2'b10: outstanding <= outstanding + 1'b1;
        2'b01: outstanding <= outstanding - 1'b1;
        default: outstanding <= outstanding;
      endcase
      if (|cu_err) cse_err <= 1'b1;
`ifndef SYNTHESIS
      if (r_fire && !q_fire && outstanding == 0) $error("cse_top: outstanding underflow");
`endif
    end
  end

  assign cse_idle = (outstanding == 0);
  assign unused_status = ^cu_idle;

`ifndef SYNTHESIS
  longint unsigned stat_cycles, stat_top_queries, stat_top_responses;
  longint unsigned stat_input_full_cycles, stat_dispatch_wait_cycles;
  longint unsigned stat_response_full_cycles, stat_receiver_wait_cycles;
  longint unsigned stat_front_killed, stat_cu_killed, stat_gap, stat_max_gap;
  longint unsigned stat_cu_accept[N_CU], stat_cu_complete[N_CU];
  int unsigned stat_input_occupancy, stat_input_peak;
  int unsigned stat_response_occupancy, stat_response_peak;

  always_ff @(posedge clk) begin : p_stats
    logic input_pop, response_push;
    input_pop = qfifo_out_valid && qfifo_out_ready;
    response_push = receiver_valid && receiver_ready;
    if (!rst_n) begin
      stat_cycles <= 0;
      stat_top_queries <= 0;
      stat_top_responses <= 0;
      stat_input_full_cycles <= 0;
      stat_dispatch_wait_cycles <= 0;
      stat_response_full_cycles <= 0;
      stat_receiver_wait_cycles <= 0;
      stat_front_killed <= 0;
      stat_cu_killed <= 0;
      stat_gap <= 0;
      stat_max_gap <= 0;
      stat_input_occupancy <= 0;
      stat_input_peak <= 0;
      stat_response_occupancy <= 0;
      stat_response_peak <= 0;
      for (int i = 0; i < N_CU; ++i) begin
        stat_cu_accept[i] <= 0;
        stat_cu_complete[i] <= 0;
      end
    end else begin
      stat_cycles <= stat_cycles + 1;
      if (q_fire) stat_top_queries <= stat_top_queries + 1;
      if (r_fire) stat_top_responses <= stat_top_responses + 1;
      if (stat_input_occupancy == INPUT_FIFO_DEPTH) stat_input_full_cycles <= stat_input_full_cycles + 1;
      if (dispatcher_waiting) stat_dispatch_wait_cycles <= stat_dispatch_wait_cycles + 1;
      if (stat_response_occupancy == RESPONSE_FIFO_DEPTH)
        stat_response_full_cycles <= stat_response_full_cycles + 1;
      if (receiver_valid && !receiver_ready) stat_receiver_wait_cycles <= stat_receiver_wait_cycles + 1;
      if (cancel_in_valid && cancel_in_ready) stat_front_killed <= stat_front_killed + 1;

      unique case ({q_fire, input_pop})
        2'b10: stat_input_occupancy <= stat_input_occupancy + 1;
        2'b01: stat_input_occupancy <= stat_input_occupancy - 1;
        default: stat_input_occupancy <= stat_input_occupancy;
      endcase
      if (q_fire && !input_pop && stat_input_occupancy + 1 > stat_input_peak)
        stat_input_peak <= stat_input_occupancy + 1;

      unique case ({response_push, r_fire})
        2'b10: stat_response_occupancy <= stat_response_occupancy + 1;
        2'b01: stat_response_occupancy <= stat_response_occupancy - 1;
        default: stat_response_occupancy <= stat_response_occupancy;
      endcase
      if (response_push && !r_fire && stat_response_occupancy + 1 > stat_response_peak)
        stat_response_peak <= stat_response_occupancy + 1;

      if (outstanding == 0 || r_fire) begin
        stat_gap <= 0;
      end else if (r_ready) begin
        stat_gap <= stat_gap + 1;
        if (stat_gap + 1 > stat_max_gap) stat_max_gap <= stat_gap + 1;
      end

      for (int i = 0; i < N_CU; ++i) begin
        if (cu_q_valid[i] && cu_q_ready[i]) stat_cu_accept[i] <= stat_cu_accept[i] + 1;
        if (cu_r_valid[i] && cu_r_ready[i]) begin
          stat_cu_complete[i] <= stat_cu_complete[i] + 1;
          if (cu_r_payload[i].status == ST_KILLED) stat_cu_killed <= stat_cu_killed + 1;
        end
      end
    end
  end

  final begin
    $display("CSE_STATS cycles=%0d q=%0d r=%0d in_peak=%0d in_full=%0d dispatch_wait=%0d out_peak=%0d out_full=%0d receiver_wait=%0d killed_front=%0d killed_cu=%0d max_gap=%0d",
             stat_cycles, stat_top_queries, stat_top_responses, stat_input_peak,
             stat_input_full_cycles, stat_dispatch_wait_cycles, stat_response_peak,
             stat_response_full_cycles, stat_receiver_wait_cycles, stat_front_killed,
             stat_cu_killed, stat_max_gap);
    for (int i = 0; i < N_CU; ++i)
      $display("CSE_STATS_CU index=%0d accepted=%0d completed=%0d", i,
               stat_cu_accept[i], stat_cu_complete[i]);
  end
`endif

endmodule
