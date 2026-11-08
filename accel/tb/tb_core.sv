// ple_core (whole PLE planner) with the CSE behavioural model against complete golden planning runs
// (fx_solve, DPI-C): status, iterations, tree sizes, DD rejects, edges, NN queries and the path, bit exact.
// After the main runs: short runs that end in FAIL_ITER, and an ENV_ERR run (one injected CSE ERROR) followed
// by a normal run. CAP / PCAP must match the golden library (FX_NODE_CAP / FX_PATH_CAP).
//   +probs=<n>  number of problems (default 14)   +seeds=<n>  seeds per problem (default 2)   +iters=<max>
//   +extra=0    skip the FAIL_ITER / ENV_ERR runs
module tb_core #(parameter bit FLAT = 1'b0, parameter bit AK = 1'b0, parameter int RANGE = 1 << 24,
                 parameter int CAP = 1024, parameter int PCAP = 64);
  import ple_pkg::*;
  import ple_cse_pkg::*;

  import "DPI-C" function int fx_ref_env_load(input int i, input int flat);
  import "DPI-C" function int fx_ref_solve(input int i, input int seed, input int max_iter, input int range_h, output int stats[8],
                                           output shortint sg[14], output int shift[7], output shortint path[448]);

  logic clk = 1'b0, rst_n = 1'b0, eot = 1'b0;
  always #5 clk = ~clk;

  conf_t cfg_start = '0, cfg_goal = '0;
  logic [DIM-1:0][31:0] cfg_shift = '0;
  logic [31:0] cfg_max_iter = 32'd3000;
  logic go = 0, busy, done;
  logic [2:0] status;
  logic [31:0] iterations, dd_rejects, edges, nn_queries;
  logic [15:0] size_start, size_goal;
  logic [7:0] path_len;
  logic [$clog2(PCAP)-1:0] pr_idx = '0;
  conf_t pr_q;
  logic q_valid, q_ready, q_kill_on_hit, q_env_buf, r_valid, r_ready, k_valid, k_ready, k_all, cse_idle, cse_err;
  logic [TAG_W-1:0] q_tag, r_tag, k_tag;
  logic [USER_W-1:0] q_user, r_user;
  logic signed [COORD_W-1:0] q_cx, q_cy, q_cz;
  logic [RQ_W-1:0] q_r;
  logic [1:0] r_status;

  ple_core #(.CAP(CAP), .PATH_CAP(PCAP)) u_core (
      .clk, .rst_n, .cfg_start, .cfg_goal, .cfg_range_h(32'(RANGE)), .cfg_radius(32'd4 << 24),
      .cfg_alpha(32'd429496), .cfg_min_radius(32'd1 << 24), .cfg_tree_ratio(32'd1 << 16), .cfg_dd(1'b1),
      .cfg_balance(1'b1), .cfg_flat(FLAT), .cfg_env_buf(1'b0), .cfg_max_iter, .cfg_shift, .go, .abort_req(1'b0), .busy, .done, .status,
      .iterations, .dd_rejects, .edges, .nn_queries, .poses(), .size_start, .size_goal, .path_len, .pr_idx, .pr_q,
      .q_valid, .q_ready, .q_tag, .q_user, .q_kill_on_hit, .q_env_buf, .q_cx, .q_cy, .q_cz, .q_r,
      .r_valid, .r_ready, .r_tag, .r_user, .r_status, .k_valid, .k_ready, .k_all, .k_tag);
  cse_bfm #(.AK(AK)) u_cse (.*);
  ple_cse_if_checker #(.AUTO_KILL(AK)) u_chk (.*);

  int errs = 0, runs = 0, solved = 0, failed_iter = 0, failed_cap = 0, env_errs = 0, total_iter = 0;
  int cov_reach = 0, cov_scaled = 0, cov_steps = 0, cov_conn0 = 0;   // branch coverage
  always @(posedge clk) if (rst_n) begin
    if (u_core.state.name() == "S_SQA" && u_core.sq_done) begin
      if (u_core.sq_root < 32'(RANGE)) cov_reach++; else cov_scaled++;
    end
    if (u_core.state.name() == "S_STEP") cov_steps++;
    if (u_core.state.name() == "S_NEXT" && u_core.dv_done && u_core.dv_q[0] == '0) cov_conn0++;
  end
  longint unsigned cyc = 0, run_cycles = 0;
  always @(posedge clk) begin
    cyc <= cyc + 1;
    if (busy) run_cycles <= run_cycles + 1;
  end

  task automatic check(input string what, input longint rtl, input longint gold);
    if (rtl != gold) begin
      if (errs < 20) $display("ERROR run %0d %s rtl=%0d golden=%0d", runs, what, rtl, gold);
      errs++;
    end
  endtask

  // one planning run of problem prob (environment already loaded); expect_err: an ERROR is injected
  task automatic do_run(input int prob, input int sd, input int iters, input bit expect_err);
    int st[8], sh[7], w;
    shortint sg[14], path[448];
    void'(fx_ref_solve(prob, sd, iters, RANGE, st, sg, sh, path));
    if (expect_err) u_cse.err_at = u_cse.nq + 100;           // the 100th query from now answers ERROR
    @(negedge clk) begin
      for (int j = 0; j < DIM; ++j) begin cfg_start[j] = sg[j]; cfg_goal[j] = sg[DIM + j]; cfg_shift[j] = 32'(sh[j]); end
      cfg_max_iter = 32'(iters);
      go = 1;
    end
    @(negedge clk) go = 0;
    w = 0;
    while (!done) begin
      @(negedge clk); w++;
      if (w > 50000000) $fatal(1, "TB_CORE run %0d (problem %0d seed %0d) stuck in state %0d", runs, prob, sd,
                               u_core.state);
    end
    if (expect_err) begin
      check("status after an injected ERROR", status, 3);
      if (u_cse.nq < u_cse.err_at) begin $display("ERROR run %0d ended before the ERROR was injected", runs); errs++; end
      u_cse.err_at = -1;
      env_errs += int'(status == 3);
    end else begin
      check("status", status, st[0]);
      check("iterations", iterations, st[1]);
      check("size_start", size_start, st[2]);
      check("size_goal", size_goal, st[3]);
      check("dd_rejects", dd_rejects, st[4]);
      check("edges", edges, st[5]);
      check("nn_queries", nn_queries, st[6]);
      if (st[0] == 0) begin                                    // path length is defined for SOLVED only
        check("path_len", path_len, st[7]);
        for (int i = 0; i < int'(path_len) && i < st[7]; ++i) begin
          pr_idx = $clog2(PCAP)'(i);
          #1;
          for (int j = 0; j < DIM; ++j) check($sformatf("path[%0d][%0d]", i, j), pr_q[j], path[i * DIM + j]);
        end
      end else check("path_len of a failed run", path_len, 0);
      solved += int'(st[0] == 0);
      failed_iter += int'(st[0] == 1);
      failed_cap += int'(st[0] == 2);
      total_iter += st[1];
    end
    runs++;
  endtask

  initial begin
    int nprob = 14, nseed = 2, iters = 3000, extra = 1;
    int plist[14] = '{1, 40, 100, 160, 210, 260, 320, 380, 410, 470, 530, 600, 650, 690};
    void'($value$plusargs("probs=%d", nprob));
    void'($value$plusargs("seeds=%d", nseed));
    void'($value$plusargs("iters=%d", iters));
    void'($value$plusargs("extra=%d", extra));
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    for (int p = 0; p < nprob && p < 14; ++p) begin
      if (fx_ref_env_load(plist[p], int'(FLAT)) < 0) $fatal(1, "environment build failed");
      for (int sd = 0; sd < nseed; ++sd) do_run(plist[p], sd, iters, 1'b0);
    end
    if (extra != 0) begin
      for (int p = 0; p < 4; ++p) begin                        // FAIL_ITER: max iterations reached
        if (fx_ref_env_load(plist[p], int'(FLAT)) < 0) $fatal(1, "environment build failed");
        do_run(plist[p], 5, 3 + 4 * p, 1'b0);
      end
      if (fx_ref_env_load(plist[1], int'(FLAT)) < 0) $fatal(1, "environment build failed");
      do_run(plist[1], 7, iters, 1'b1);                        // ENV_ERR, then a normal run must still match
      do_run(plist[1], 7, iters, 1'b0);
      if (failed_iter == 0 || env_errs == 0) begin $display("ERROR FAIL_ITER or ENV_ERR not exercised"); errs++; end
    end
    repeat (300) @(negedge clk);
    eot = 1'b1;
    repeat (2) @(negedge clk);
    if (u_chk.errors != 0) begin $display("ERROR interface checker reported %0d errors", u_chk.errors); errs++; end
    $display("TB_CORE flat=%0d auto_kill=%0d range=%0d cap=%0d path_cap=%0d runs=%0d solved=%0d fail_iter=%0d fail_cap=%0d env_err=%0d iterations=%0d cycles=%0d queries=%0d extend_reach=%0d extend_scaled=%0d connect_steps=%0d connect_zero=%0d errors=%0d",
             FLAT, AK, RANGE, CAP, PCAP, runs, solved, failed_iter, failed_cap, env_errs, total_iter, run_cycles, u_cse.nq,
             cov_reach, cov_scaled, cov_steps, cov_conn0, errs);
    if (errs != 0) $fatal(1, "TB_CORE FAIL");
    $finish;
  end

endmodule
