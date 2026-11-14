// ple_evu (RKG + FKU + SCU + QD) with the CSE behavioural model, against the golden edge_ok with the full
// pose check (environment + self collision). Edges: long random edges in the joint limits (DIRECT-like)
// and short ones (EXTEND / CONNECT-like, |vec| <= 1 rad), in several MotionBenchMaker environments.
module tb_evu #(parameter bit FLAT = 1'b0, parameter bit AK = 1'b0);
  import ple_pkg::*;
  import ple_cse_pkg::*;

  import "DPI-C" function int fx_ref_env_load(input int i, input int flat);
  import "DPI-C" function int fx_ref_edge_ok(input shortint s[7], input int v[7], input longint len_h, output int poses);
  import "DPI-C" function longint unsigned fx_ref_isqrt(input longint unsigned v);

  localparam int VW = 18;
  logic clk = 1'b0, rst_n = 1'b0, eot = 1'b0;
  always #5 clk = ~clk;

  logic start = 0, busy, done, edge_ok, env_err;
  conf_t s_q = '0;
  logic [DIM-1:0][VW-1:0] vec = '0;
  logic [31:0] len_h = '0;
  logic [15:0] n_poses;
  logic q_valid, q_ready, q_kill_on_hit, q_env_buf, r_valid, r_ready, k_valid, k_ready, k_all, cse_idle, cse_err;
  logic [TAG_W-1:0] q_tag, r_tag, k_tag;
  logic [USER_W-1:0] q_user, r_user;
  logic signed [COORD_W-1:0] q_cx, q_cy, q_cz;
  logic [RQ_W-1:0] q_r;
  logic [1:0] r_status;

  ple_evu #(.VW(VW)) u_evu (.clk, .rst_n, .cfg_flat(FLAT), .cfg_env_buf(1'b0), .start, .s_q, .vec, .len_h, .busy,
                            .done, .edge_ok, .env_err, .n_poses, .q_valid, .q_ready, .q_tag, .q_user, .q_kill_on_hit,
                            .q_env_buf, .q_cx, .q_cy, .q_cz, .q_r, .r_valid, .r_ready, .r_tag, .r_user, .r_status,
                            .k_valid, .k_ready, .k_all, .k_tag);
  cse_bfm #(.AK(AK)) u_cse (.*);
  ple_cse_if_checker #(.AUTO_KILL(AK)) u_chk (.*);

  int errs = 0, edges = 0, free_edges = 0, coll_edges = 0, pose_sum = 0, gold_pose_sum = 0;
  longint unsigned cyc = 0, busy_cycles = 0;
  always @(posedge clk) begin
    cyc <= cyc + 1;
    if (busy) busy_cycles <= busy_cycles + 1;
    if (cyc > 64'd400000000) $fatal(1, "TB_EVU watchdog: no progress");
  end

  task automatic run_edge(input shortint s[7], input int v[7]);
    longint d2 = 0;
    longint len;
    int gold, gposes, w = 0;
    for (int j = 0; j < DIM; ++j) d2 += longint'(v[j]) * longint'(v[j]);
    len = longint'(fx_ref_isqrt(64'(d2) << 24));                 // len_h as the model computes it
    gold = fx_ref_edge_ok(s, v, len, gposes);
    @(negedge clk) begin
      for (int j = 0; j < DIM; ++j) begin s_q[j] = s[j]; vec[j] = VW'(v[j]); end
      len_h = 32'(len);
      start = 1;
    end
    @(negedge clk) start = 0;
    while (!done) begin
      @(negedge clk); w++;
      if (w > 2000000) begin
        $display("STUCK state=%0d total=%0d scu_ok=%0d qd_ok=%0d bad=%0d rkg_busy=%0d rkg_pv=%0d rkg_pr=%0d fku_busy=%0d fku_go=%0d c_scu=%0d c_qd=%0d scu_busy=%0d qd_pr=%0d qd_live=%b n_poses=%0d",
                 u_evu.state, u_evu.total, u_evu.scu_ok, u_evu.qd_ok, u_evu.bad, u_evu.rkg_busy, u_evu.rkg_pv, u_evu.rkg_pr,
                 u_evu.fku_busy, u_evu.fku_go, u_evu.c_scu, u_evu.c_qd, u_evu.scu_busy, u_evu.qd_pr, u_evu.u_qd.live, n_poses);
        $fatal(1, "TB_EVU edge %0d stuck", edges);
      end
    end
    edges++;
    // contract: a free edge is reported only after every pose passed both the self and the env check
    if (edge_ok && (u_evu.scu_ok != u_evu.total || u_evu.qd_ok != u_evu.total || u_evu.scu_busy)) begin
      if (errs < 10) $display("ERROR edge %0d reported free with scu_ok=%0d qd_ok=%0d of %0d (scu busy %0d)", edges,
                              u_evu.scu_ok, u_evu.qd_ok, u_evu.total, u_evu.scu_busy);
      errs++;
    end
    pose_sum += int'(n_poses);
    gold_pose_sum += gposes;
    if (edge_ok) free_edges++; else coll_edges++;
    if (edge_ok != (gold != 0) || env_err) begin
      if (errs < 10) $display("ERROR edge %0d rtl ok=%0d err=%0d golden ok=%0d (golden poses %0d, rtl %0d)", edges, edge_ok,
                              env_err, gold, gposes, n_poses);
      errs++;
    end
  endtask

  int problems_list[4] = '{1, 260, 410, 600};

  int qd_wait = 0, qd_wait_max = 0;                      // longest wait of FKU centres for a QD slot
  always @(posedge clk) begin
    if (u_evu.c_qd) qd_wait <= qd_wait + 1; else qd_wait <= 0;
    if (qd_wait > qd_wait_max) qd_wait_max <= qd_wait;
  end

  initial begin
    shortint s[7];
    int v[7];
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    foreach (problems_list[p]) begin
      if (fx_ref_env_load(problems_list[p], int'(FLAT)) < 0) $fatal(1, "environment build failed");
      // second half: a fast CSE (QD can finish before SCU, the self check must still be waited for)
      // problem 1: a slow CSE (QD stays full longer than one FKU run: the FKU must wait for QD to take the centres)
      u_cse.maxlat = (p >= 2) ? 2 : 40;
      u_cse.rdy_pct = (p >= 2) ? 100 : (p == 1) ? 6 : 75;
      for (int e = 0; e < 120; ++e) begin
        for (int j = 0; j < DIM; ++j) begin
          int a = int'(SMP_A[j]), m = int'(SMP_M[j]);
          s[j] = shortint'(a + int'($urandom % m));
          if (e % 3 == 0) v[j] = a + int'($urandom % m) - int'(s[j]);              // long edge
          else begin                                                              // short edge, inside the limits
            int t = int'(s[j]) + int'($urandom % 1201) - 600;
            if (t < a) t = a;
            if (t > a + m - 1) t = a + m - 1;
            v[j] = t - int'(s[j]);
          end
        end
        run_edge(s, v);
      end
    end
    repeat (300) @(negedge clk);
    eot = 1'b1;
    repeat (2) @(negedge clk);
    if (u_chk.errors != 0) begin $display("ERROR interface checker reported %0d errors", u_chk.errors); errs++; end
    if (qd_wait_max < 80) begin $display("ERROR QD back-pressure longer than an FKU run not exercised (max wait %0d)", qd_wait_max); errs++; end
    $display("TB_EVU flat=%0d auto_kill=%0d edges=%0d (free %0d, collide %0d) fku_poses=%0d golden_poses_checked=%0d queries=%0d cycles_per_edge=%0d qd_wait_max=%0d errors=%0d",
             FLAT, AK, edges, free_edges, coll_edges, pose_sum, gold_pose_sum, u_cse.nq, int'(busy_cycles / 64'(edges)), qd_wait_max, errs);
    if (errs != 0) $fatal(1, "TB_EVU FAIL");
    $finish;
  end

endmodule
