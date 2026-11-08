// EVU: validates one edge (golden fx_plan.c edge_ok with the pose check fx_pose_ok = environment + self).
// RKG emits the poses (VAMP order) -> FKU computes the 70 sphere centres -> SCU (self collision) and
// QD (environment queries to the CSE) both take the centres; FKU starts the next pose once both took them.
// The edge is free when all 8n poses pass both checks; the first collision (or a CSE ERROR) kills RKG and
// the QD tag, waits for FKU / SCU to finish their current pose, then reports. start waits for a free tag.
module ple_evu
  import ple_pkg::*;
  import ple_cse_pkg::*;
#(
    parameter int VW    = 18,                            // edge vector width (signed)
    parameter int NSLOT = 8                              // QD pose slots
) (
    input  logic                      clk,
    input  logic                      rst_n,
    input  logic                      cfg_flat,
    input  logic                      cfg_env_buf,
    // edge
    input  logic                      start,             // ignored while busy
    input  conf_t                     s_q,
    input  logic [DIM-1:0][VW-1:0]    vec,
    input  logic [31:0]               len_h,
    output logic                      busy,
    output logic                      done,              // one-cycle pulse
    output logic                      edge_ok,           // 1 = every pose free (valid with done)
    output logic                      env_err,           // a CSE ERROR ended the edge
    output logic [15:0]               n_poses,           // poses computed by FKU for this edge
    // CSE interface
    output logic                      q_valid,
    input  logic                      q_ready,
    output logic [TAG_W-1:0]          q_tag,
    output logic [USER_W-1:0]         q_user,
    output logic                      q_kill_on_hit,
    output logic                      q_env_buf,
    output logic signed [COORD_W-1:0] q_cx,
    output logic signed [COORD_W-1:0] q_cy,
    output logic signed [COORD_W-1:0] q_cz,
    output logic [RQ_W-1:0]           q_r,
    input  logic                      r_valid,
    output logic                      r_ready,
    input  logic [TAG_W-1:0]          r_tag,
    input  logic [USER_W-1:0]         r_user,
    input  logic [1:0]                r_status,
    output logic                      k_valid,
    input  logic                      k_ready,
    output logic                      k_all,
    output logic [TAG_W-1:0]          k_tag
);

  typedef enum logic [1:0] {S_IDLE, S_TAG, S_RUN, S_DRAIN} state_e;
  state_e state;

  // ---------------------------------------------------------------- units
  logic rkg_start, rkg_kill, rkg_busy, rkg_pv, rkg_pr, rkg_last;
  logic [15:0] rkg_n;
  conf_t rkg_q;
  conf_t eq_s;
  logic [DIM-1:0][VW-1:0] e_vec;
  logic [31:0] e_len;
  ple_rkg #(.VW(VW)) u_rkg (.clk, .rst_n, .start(rkg_start), .s_q(eq_s), .vec(e_vec), .len_h(e_len), .kill(rkg_kill),
                           .busy(rkg_busy), .n_out(rkg_n), .p_valid(rkg_pv), .p_ready(rkg_pr), .p_q(rkg_q),
                           .p_last(rkg_last));

  logic fku_start, fku_busy, fku_done;
  conf_t q_hold;
  logic [69:0][2:0][17:0] fku_c;
  ple_fku u_fku (.clk, .rst_n, .start(fku_start), .q(q_hold), .busy(fku_busy), .done(fku_done), .c(fku_c));

  logic scu_start, scu_busy, scu_done, scu_coll;
  ple_scu u_scu (.clk, .rst_n, .start(scu_start), .c(fku_c), .busy(scu_busy), .done(scu_done), .collide(scu_coll));

  logic qd_ready_edge, qd_start, qd_kill, qd_idle, qd_err, qd_pv, qd_pr, pd_valid, pd_hit;
  logic [15:0] pd_id, pose_id;
  ple_qd #(.NSLOT(NSLOT), .PIDW(16)) u_qd (
      .clk, .rst_n, .cfg_flat, .cfg_env_buf, .edge_ready(qd_ready_edge), .edge_start(qd_start), .edge_kill(qd_kill),
      .idle(qd_idle), .env_err(qd_err), .p_valid(qd_pv), .p_ready(qd_pr), .p_id(pose_id), .p_c(fku_c),
      .pd_valid, .pd_id, .pd_hit, .q_valid, .q_ready, .q_tag, .q_user, .q_kill_on_hit, .q_env_buf, .q_cx, .q_cy,
      .q_cz, .q_r, .r_valid, .r_ready, .r_tag, .r_user, .r_status, .k_valid, .k_ready, .k_all, .k_tag);

  // ---------------------------------------------------------------- edge bookkeeping
  logic [18:0] total;                                    // 8n poses
  logic [18:0] scu_ok, qd_ok;
  logic        fku_go, c_scu, c_qd, bad, err_seen;
  wire  c_pend = c_scu || c_qd;                          // FKU output not yet taken by both

  // n = max(1, ceil(len_h * 4 / 2^24)) as in ple_rkg
  logic [33:0] len4;
  logic [15:0] n_calc;
  always_comb begin
    len4   = {len_h, 2'b00} + 34'h0FF_FFFF;
    n_calc = (len4[33:24] == 0) ? 16'd1 : 16'(len4[33:24]);
  end

  assign rkg_pr    = (state == S_RUN) && !bad && !fku_busy && !fku_go && !fku_start && !c_pend && !fku_done;
  assign scu_start = c_scu && !scu_busy && !bad;
  assign qd_pv     = c_qd && !bad;
  assign busy      = (state != S_IDLE);

  wire collide_now = (scu_done && scu_coll) || (pd_valid && pd_hit) || qd_err;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      state <= S_IDLE; done <= 1'b0; rkg_start <= 1'b0; rkg_kill <= 1'b0; qd_start <= 1'b0; qd_kill <= 1'b0;
      fku_start <= 1'b0; fku_go <= 1'b0; c_scu <= 1'b0; c_qd <= 1'b0; bad <= 1'b0; err_seen <= 1'b0;
      edge_ok <= 1'b0; env_err <= 1'b0; pose_id <= '0; n_poses <= '0;
    end else begin
      done <= 1'b0; rkg_start <= 1'b0; rkg_kill <= 1'b0; qd_start <= 1'b0; qd_kill <= 1'b0; fku_start <= 1'b0;
      unique case (state)
        S_IDLE: if (start) begin
          eq_s <= s_q; e_vec <= vec; e_len <= len_h;
          total <= {n_calc[15:0], 3'b000};
          scu_ok <= '0; qd_ok <= '0; bad <= 1'b0; err_seen <= 1'b0; n_poses <= '0;
          c_scu <= 1'b0; c_qd <= 1'b0; fku_go <= 1'b0;
          state <= S_TAG;
        end
        S_TAG: if (qd_ready_edge) begin                  // a free tag: start QD and RKG together
          qd_start <= 1'b1; rkg_start <= 1'b1; state <= S_RUN;
        end
        S_RUN, S_DRAIN: begin
          // RKG -> FKU (q held stable for the whole FKU run)
          if (rkg_pv && rkg_pr) begin q_hold <= rkg_q; fku_go <= 1'b1; end
          if (fku_go) begin fku_start <= 1'b1; fku_go <= 1'b0; n_poses <= n_poses + 16'd1; end
          // FKU -> SCU and QD
          if (fku_done && !bad) begin c_scu <= 1'b1; c_qd <= 1'b1; pose_id <= pose_id + 16'd1; end
          if (scu_start) c_scu <= 1'b0;
          if (qd_pv && qd_pr) c_qd <= 1'b0;
          // results
          if (scu_done && !scu_coll) scu_ok <= scu_ok + 19'd1;
          if (pd_valid && !pd_hit) qd_ok <= qd_ok + 19'd1;
          if (collide_now && !bad) begin
            bad <= 1'b1; rkg_kill <= 1'b1; qd_kill <= 1'b1; c_scu <= 1'b0; c_qd <= 1'b0; fku_go <= 1'b0;
            state <= S_DRAIN;
          end
          if (qd_err) err_seen <= 1'b1;
          // decisions
          if (state == S_RUN && !bad && !collide_now && scu_ok == total && qd_ok == total) begin
            edge_ok <= 1'b1; env_err <= 1'b0; done <= 1'b1; state <= S_IDLE;
          end
          if (state == S_DRAIN && !fku_busy && !fku_go && !fku_start && !scu_busy && !rkg_busy) begin
            edge_ok <= 1'b0; env_err <= err_seen || qd_err; done <= 1'b1; state <= S_IDLE;
          end
        end
        default: state <= S_IDLE;
      endcase
    end
  end

  logic unused;
  assign unused = ^{qd_idle, rkg_last, rkg_n, pd_id, len4[23:0]};

endmodule
