// ple_qd with a CSE behavioural model that answers from the golden cse_ref (DPI-C), checked against the
// golden pose environment check fx_env_ok() and by the interface checker. Edges of 1..24 poses from the
// golden FK of random joint vectors in several MotionBenchMaker environments; random CSE latency and
// reordering, random q_ready stalls, KILLED answers after a kill, optional auto-kill, and edges killed
// from outside (as for a self collision). Hierarchical and flat mode.
//   +auto_kill=1    CSE model auto-kills a tag on a kill_on_hit HIT
module tb_qd #(parameter bit FLAT = 1'b0, parameter bit AK = 1'b0);
  import ple_pkg::*;
  import ple_cse_pkg::*;

  import "DPI-C" function int fx_ref_env_load(input int i, input int flat);
  import "DPI-C" function void fx_ref_fk_ws(input shortint q[7], output int c[210]);
  import "DPI-C" function int fx_ref_env_ok(input int c[210]);
  import "DPI-C" function int fx_ref_cse(input int cx, input int cy, input int cz, input int rq);

  localparam int PIDW = 16;
  logic clk = 1'b0, rst_n = 1'b0, eot = 1'b0;
  always #5 clk = ~clk;

  logic edge_ready, edge_start = 0, edge_kill = 0, idle, env_err;
  logic p_valid = 0, p_ready;
  logic [PIDW-1:0] p_id = '0;
  logic [69:0][2:0][17:0] p_c = '0;
  logic pd_valid, pd_hit;
  logic [PIDW-1:0] pd_id;
  logic q_valid, q_ready, q_kill_on_hit, q_env_buf, r_valid, r_ready, k_valid, k_ready, k_all, cse_idle, cse_err;
  logic [TAG_W-1:0] q_tag, r_tag, k_tag;
  logic [USER_W-1:0] q_user, r_user;
  logic signed [COORD_W-1:0] q_cx, q_cy, q_cz;
  logic [RQ_W-1:0] q_r;
  logic [1:0] r_status;

  ple_qd #(.NSLOT(8), .PIDW(PIDW)) u_qd (
      .clk, .rst_n, .cfg_flat(FLAT), .cfg_env_buf(1'b0), .edge_ready, .edge_start, .edge_kill, .idle, .env_err,
      .p_valid, .p_ready, .p_id, .p_c, .pd_valid, .pd_id, .pd_hit,
      .q_valid, .q_ready, .q_tag, .q_user, .q_kill_on_hit, .q_env_buf, .q_cx, .q_cy, .q_cz, .q_r,
      .r_valid, .r_ready, .r_tag, .r_user, .r_status, .k_valid, .k_ready, .k_all, .k_tag);

  ple_cse_if_checker #(.AUTO_KILL(AK)) u_chk (.*);

  int errs = 0;
  longint unsigned cyc = 0;
  always @(posedge clk) begin
    cyc <= cyc + 1;
    if (cyc > 64'd400000000) $fatal(1, "TB_QD watchdog: no progress");
  end

  // ---------------------------------------------------------------- CSE behavioural model
  typedef struct {
    logic [TAG_W-1:0] tag;
    logic [USER_W-1:0] user;
    logic koh;
    int cx, cy, cz, rq;
    longint unsigned due;
    bit killed;
  } ent_t;
  ent_t cq[$];
  int nq = 0, nkilled = 0;
  assign cse_err = 1'b0;
  always_comb cse_idle = (cq.size() == 0) && !r_valid;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      q_ready <= 1'b0; k_ready <= 1'b0; r_valid <= 1'b0; r_tag <= '0; r_user <= '0; r_status <= '0;
    end else begin
      q_ready <= ($urandom % 100) < 75;
      k_ready <= ($urandom % 100) < 70;
      if (k_valid && k_ready) foreach (cq[i]) if (k_all || cq[i].tag == k_tag) cq[i].killed = 1;
      if (q_valid && q_ready) begin
        cq.push_back('{q_tag, q_user, q_kill_on_hit, int'(q_cx), int'(q_cy), int'(q_cz), int'(q_r),
                       cyc + 1 + 64'($urandom % 40), 1'b0});
        nq++;
      end
      if (r_valid && r_ready) r_valid <= 1'b0;
      if (!r_valid || r_ready) begin
        int pick = -1;
        for (int i = 0; i < cq.size() && i < 24; ++i) if (cq[i].due <= cyc && ($urandom % 3) != 0) begin pick = i; break; end
        if (pick >= 0) begin
          ent_t e = cq[pick];
          logic [1:0] st;
          cq.delete(pick);
          if (e.killed && ($urandom % 2)) begin st = ST_KILLED; nkilled++; end
          else begin
            st = fx_ref_cse(e.cx, e.cy, e.cz, e.rq) ? ST_HIT : ST_MISS;
            if (AK && e.koh && st == ST_HIT) foreach (cq[i]) if (cq[i].tag == e.tag) cq[i].killed = 1;
          end
          r_valid <= 1'b1; r_tag <= e.tag; r_user <= e.user; r_status <= st;
        end
      end
    end
  end

  // ---------------------------------------------------------------- pose results
  int exp_ok[int];                                     // pose id -> golden environment result
  int reported[int];
  int pd_hits = 0, pd_oks = 0, edge_hit = 0, edge_oks = 0, after_kill = 0;
  bit killed_ext = 0;
  always @(posedge clk) if (rst_n && pd_valid) begin
    if (!exp_ok.exists(int'(pd_id))) begin $display("ERROR result for unknown pose %0d", pd_id); errs++; end
    else begin
      if (reported.exists(int'(pd_id))) begin $display("ERROR pose %0d reported twice", pd_id); errs++; end
      reported[int'(pd_id)] = 1;
      if (pd_hit == (exp_ok[int'(pd_id)] != 0)) begin
        if (errs < 10) $display("ERROR pose %0d rtl hit=%0d golden ok=%0d", pd_id, pd_hit, exp_ok[int'(pd_id)]);
        errs++;
      end
    end
    if (killed_ext) after_kill++;
    if (pd_hit) begin pd_hits++; edge_hit = 1; end else begin pd_oks++; edge_oks++; end
  end

  // ---------------------------------------------------------------- edges
  int edges = 0, poses_fed = 0, ext_kills = 0, envs = 0, free_edges = 0, hit_edges = 0;
  int next_id = 0;
  int problems_list[5] = '{1, 120, 260, 410, 600};

  task automatic stuck(input string where);
    $display("STUCK edge=%0d at %s: edge_ready=%0d idle=%0d tag_out=%0d/%0d cur_tag=%0d live=%b dead=%0d live_edge=%0d q_valid=%0d outst0=%0d pend0=%b xact0=%0d optr0=%0d cse_q=%0d edge_hit=%0d edge_oks=%0d",
             edges, where, edge_ready, idle, u_qd.tag_out[0], u_qd.tag_out[1], u_qd.cur_tag, u_qd.live, u_qd.dead,
             u_qd.live_edge, q_valid, u_qd.outst[0], u_qd.pend[0], u_qd.xact[0], u_qd.optr[0], cq.size(), edge_hit, edge_oks);
    $fatal(1, "TB_QD stuck");
  endtask

  task automatic run_edge(input int np, input int kill_at);
    shortint qq[7];
    int cc[210];
    int fed = 0, any_coll = 0, w = 0;
    bit taken;
    w = 0; while (!edge_ready) begin @(negedge clk); w++; if (w > 100000) stuck("edge_ready"); end
    @(negedge clk) edge_start = 1;
    @(negedge clk) edge_start = 0;
    edge_hit = 0; edge_oks = 0; killed_ext = 0;
    exp_ok.delete(); reported.delete();
    while (fed < np && !edge_hit) begin
      for (int j = 0; j < DIM; ++j) qq[j] = shortint'(int'(SMP_A[j]) + int'($urandom % SMP_M[j]));
      fx_ref_fk_ws(qq, cc);
      exp_ok[next_id] = fx_ref_env_ok(cc);
      any_coll |= (exp_ok[next_id] == 0);
      for (int s = 0; s < 70; ++s) for (int k = 0; k < 3; ++k) p_c[s][k] = 18'(cc[3 * s + k]);
      p_id = PIDW'(next_id);
      p_valid = 1;
      taken = 0;
      forever begin                                     // sample p_ready before the rising edge
        #1;
        if (p_ready) begin @(posedge clk); taken = 1; break; end
        if (edge_hit) break;
        @(negedge clk);
      end
      @(negedge clk) p_valid = 0;
      if (taken) begin fed++; poses_fed++; end
      else exp_ok.delete(next_id);                     // not taken: the edge was already decided
      next_id++;
      if (kill_at == fed) begin                         // edge invalid elsewhere (self collision)
        @(negedge clk) edge_kill = 1;
        @(negedge clk) edge_kill = 0;
        @(negedge clk) killed_ext = 1;                  // results of the kill cycle itself are allowed
        ext_kills++;
        break;
      end
      repeat ($urandom % 3) @(negedge clk);
    end
    if (!killed_ext) begin
      // wait for the decision: a hit, or every fed pose reported free
      w = 0; while (!edge_hit && edge_oks < fed) begin @(negedge clk); w++; if (w > 100000) stuck("decision"); end
      if (edge_hit) begin
        @(negedge clk) edge_kill = 1;                   // the controller also kills on an env hit
        @(negedge clk) edge_kill = 0;
        hit_edges++;
      end else begin
        free_edges++;
        if (any_coll) begin $display("ERROR edge %0d: all poses reported free but one collides", edges); errs++; end
      end
    end
    w = 0; while (!idle) begin @(negedge clk); w++; if (w > 100000) stuck("idle"); end
    edges++;
  endtask

  initial begin
    int r;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    foreach (problems_list[i]) begin
      r = fx_ref_env_load(problems_list[i], int'(FLAT));
      if (r < 0) $fatal(1, "environment build failed");
      envs++;
      for (int e = 0; e < 150; ++e)
        run_edge(1 + int'($urandom % 24), (e % 9 == 4) ? 1 + int'($urandom % 6) : -1);
    end
    repeat (200) @(negedge clk);
    eot = 1'b1;
    repeat (2) @(negedge clk);
    if (u_chk.errors != 0) begin $display("ERROR interface checker reported %0d errors", u_chk.errors); errs++; end
    if (after_kill != 0) begin $display("ERROR %0d results after an external kill", after_kill); errs++; end
    $display("TB_QD flat=%0d auto_kill=%0d envs=%0d edges=%0d (free %0d, env hit %0d, killed %0d) poses=%0d queries=%0d killed_answers=%0d hits_reported=%0d free_reported=%0d errors=%0d",
             FLAT, AK, envs, edges, free_edges, hit_edges, ext_kills, poses_fed, nq, nkilled, pd_hits, pd_oks, errs);
    if (errs != 0) $fatal(1, "TB_QD FAIL");
    $finish;
  end

endmodule
