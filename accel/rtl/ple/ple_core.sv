// PLE core: RRT-Connect controller (golden fx_plan.c fx_solve) with SMP, TREE, NNU, ISQRT, DIV7, EVU and
// the path backtrace (PBT) into a path buffer. Every arithmetic step follows the golden integer definition:
// DD reject rad^2 < d2 << 24, len_h = isqrt(d2 << 24), EXTEND ext = floor(d * range_h / nd_h), CONNECT
// n_ext = ceil(od_h / range_h), inc_h = od_h / n_ext, step node = new + floor(ovec * k / n_ext).
// Tree 0 holds the start tree, tree 1 the goal tree; "a" is the tree being extended.
// Status: 0 SOLVED, 1 FAIL_ITER, 2 FAIL_CAPACITY (tree or path full), 3 ENV_ERR, 4 ABORTED.
// abort_req is taken at the next iteration boundary (EVU idle, no query in flight).
module ple_core
  import ple_pkg::*;
  import ple_cse_pkg::*;
#(
    parameter int CAP      = NODE_CAP,                  // nodes per tree
    parameter int PATH_CAP = 64,
    parameter int NSLOT    = 8
) (
    input  logic                      clk,
    input  logic                      rst_n,
    // configuration (stable while busy)
    input  conf_t                     cfg_start,
    input  conf_t                     cfg_goal,
    input  logic [31:0]               cfg_range_h,       // 2^-24 rad
    input  logic [31:0]               cfg_radius,        // DD radius on first failure, 2^-24 rad
    input  logic [31:0]               cfg_alpha,         // Q0.32
    input  logic [31:0]               cfg_min_radius,
    input  logic [31:0]               cfg_tree_ratio,    // Q16.16
    input  logic                      cfg_dd,
    input  logic                      cfg_balance,
    input  logic                      cfg_flat,
    input  logic                      cfg_env_buf,
    input  logic [31:0]               cfg_max_iter,
    input  logic [DIM-1:0][31:0]      cfg_shift,         // Halton rotation (splitmix64 of the seed, by the CPU)
    input  logic                      go,
    input  logic                      abort_req,         // pulse; held until the next iteration boundary
    // status
    output logic                      busy,
    output logic                      done,
    output logic [2:0]                status,
    output logic [31:0]               iterations,
    output logic [31:0]               dd_rejects,
    output logic [31:0]               edges,
    output logic [31:0]               nn_queries,
    output logic [31:0]               poses,             // poses computed by FKU (perf only)
    output logic [31:0]               self_pairs,        // SCU pair tests incl. row filter tests (perf only)
    output logic [15:0]               size_start,
    output logic [15:0]               size_goal,
    output logic [7:0]                path_len,
    // path read (combinational)
    input  logic [$clog2(PATH_CAP)-1:0] pr_idx,
    output conf_t                     pr_q,
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

  localparam int IW = $clog2(CAP), NW = $clog2(CAP + 1), VW = 18, PW = $clog2(PATH_CAP);
  localparam logic [2:0] ST_SOLVED = 3'd0, ST_FAIL_ITER = 3'd1, ST_FAIL_CAP = 3'd2, ST_ENV_ERR = 3'd3,
                         ST_ABORTED = 3'd4;

  // ---------------------------------------------------------------- units
  logic smp_init, smp_take, smp_valid;
  conf_t smp_q;
  ple_smp u_smp (.clk, .rst_n, .init(smp_init), .shift(cfg_shift), .take(smp_take), .valid(smp_valid), .q(smp_q));

  logic wr_en, wr_tree, rad_we, rad_tree, rd_en, rd_tree, nn_en, nn_tree;
  logic [IW-1:0] wr_idx, wr_parent, rad_idx, rd_idx, rd_parent, nn_row;
  conf_t wr_q, rd_q;
  conf_t [3:0] nn_q;
  logic [RAD_W-1:0] rad_wdata, rd_rad;
  ple_tree #(.K(4), .CAP(CAP)) u_tree (.clk, .wr_en, .wr_tree, .wr_idx, .wr_q, .wr_parent, .rad_we, .rad_tree, .rad_idx,
                                      .rad_wdata, .rd_en, .rd_tree, .rd_idx, .rd_q, .rd_parent, .rd_rad, .nn_en,
                                      .nn_tree, .nn_row, .nn_q);

  logic nnu_start, nnu_tree, nnu_busy, nnu_done;
  conf_t nnu_qin;
  logic [NW-1:0] nnu_n;
  logic [IW-1:0] nnu_idx;
  logic [D2_W-1:0] nnu_d2;
  ple_nnu #(.K(4), .CAP(CAP)) u_nnu (.clk, .rst_n, .start(nnu_start), .tree(nnu_tree), .q(nnu_qin), .n(nnu_n),
                                    .busy(nnu_busy), .done(nnu_done), .idx(nnu_idx), .d2(nnu_d2), .nn_en, .nn_tree,
                                    .nn_row, .nn_q);

  logic sq_start, sq_busy, sq_done;
  logic [63:0] sq_x;
  logic [31:0] sq_root;
  ple_isqrt u_sqrt (.clk, .rst_n, .start(sq_start), .x(sq_x), .busy(sq_busy), .done(sq_done), .root(sq_root));

  logic dv_start, dv_busy, dv_done;
  logic [DIM-1:0][49:0] dv_a, dv_q;
  logic [31:0] dv_b;
  logic [DIM-1:0][31:0] dv_r;
  ple_div7 #(.AW(50), .BW(32)) u_div (.clk, .rst_n, .start(dv_start), .a(dv_a), .b(dv_b), .busy(dv_busy),
                                      .done(dv_done), .q(dv_q), .r(dv_r));

  logic ev_start, ev_busy, ev_done, ev_ok, ev_err;
  conf_t ev_s;
  logic [DIM-1:0][VW-1:0] ev_vec;
  logic [31:0] ev_len;
  logic [15:0] ev_np;
  logic ev_spv;
  logic [9:0] ev_spn;
  ple_evu #(.VW(VW), .NSLOT(NSLOT)) u_evu (
      .clk, .rst_n, .cfg_flat, .cfg_env_buf, .start(ev_start), .s_q(ev_s), .vec(ev_vec), .len_h(ev_len),
      .busy(ev_busy), .done(ev_done), .edge_ok(ev_ok), .env_err(ev_err), .n_poses(ev_np), .sp_valid(ev_spv), .sp_pairs(ev_spn), .q_valid, .q_ready, .q_tag,
      .q_user, .q_kill_on_hit, .q_env_buf, .q_cx, .q_cy, .q_cz, .q_r, .r_valid, .r_ready, .r_tag, .r_user, .r_status,
      .k_valid, .k_ready, .k_all, .k_tag);

  // ---------------------------------------------------------------- controller state
  typedef enum logic [5:0] {
    S_IDLE, S_D2, S_D2B, S_DSQ, S_DSQW, S_DEVW, S_INS0, S_INS1, S_LOOP, S_BAL, S_SAMPLE, S_NNA, S_RDA, S_RDA2, S_DDM, S_DD,
    S_SQA, S_DIVE, S_EVE, S_EVEW, S_SHRINK, S_INSN, S_GROW, S_CON, S_NNB, S_RDB, S_RDB2, S_SQB, S_NEXT, S_INCH,
    S_STEP, S_STEPW, S_EVCW, S_MUL1, S_MUL2, S_BT_A, S_BT_A2, S_BT_B, S_BT_B2, S_BT_W, S_BT_W2, S_DONE
  } state_e;
  state_e state, mul_ret;

  logic [1:0][NW-1:0] n_t;                               // nodes per tree
  logic        a_sel;                                    // tree being extended (0 start, 1 goal)
  conf_t       q_s, qn, qnew, prior, oq;
  logic [D2_W-1:0] nd2, od2;
  logic [31:0] nrad, nd_h, od_h, n_ext, inc_h;
  logic [63:0] prod;                                   // mul_x * mul_y, 2 cycles after they are set
  logic [31:0] mul_x, mul_y;
  logic [63:0] mul_m;
  logic [DIM-1:0][49:0] ext_m, ext_p;                 // ext * range_h, pipelined (ext is stable during ISQRT)
  logic [DIM-1:0][D2_W-1:0] d2j;                       // DIRECT edge: per-joint squared differences
  logic [IW-1:0] near_i, ni, other_i, prior_i;
  logic [DIM-1:0][VW-1:0] ext, ovec;
  logic [15:0] k;
  // backtrace buffers
  logic [IW-1:0] bt_a [PATH_CAP], bt_b [PATH_CAP];
  logic [7:0]  na, nb, bt_s, bt_tot;
  logic [IW-1:0] bt_c;
  logic        bt_over;
  logic [1:0]  bt_ph;
  conf_t       path_mem [PATH_CAP];
  assign pr_q = path_mem[pr_idx];

  assign busy = (state != S_IDLE);

  // pipelined multipliers (DSP MREG / PREG)
  always_ff @(posedge clk) begin
    mul_m <= 64'(mul_x) * 64'(mul_y);
    prod  <= mul_m;
    for (int j = 0; j < DIM; ++j) begin
      ext_m[j] <= 50'(signed'(ext[j])) * 50'({1'b0, cfg_range_h});
      ext_p[j] <= ext_m[j];
    end
  end

  logic abort_q;
  always_ff @(posedge clk) begin
    if (!rst_n) begin abort_q <= 1'b0; poses <= '0; self_pairs <= '0; end
    else begin
      if (state == S_IDLE && go) abort_q <= 1'b0;
      else if (abort_req && busy) abort_q <= 1'b1;
      if (state == S_IDLE && go) poses <= '0;
      else if (ev_done) poses <= poses + 32'(ev_np);
      if (state == S_IDLE && go) self_pairs <= '0;
      else if (ev_spv) self_pairs <= self_pairs + 32'(ev_spn);
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      state <= S_IDLE; done <= 1'b0; status <= ST_FAIL_ITER;
      smp_init <= 1'b0; smp_take <= 1'b0; wr_en <= 1'b0; rad_we <= 1'b0; rd_en <= 1'b0; nnu_start <= 1'b0;
      sq_start <= 1'b0; dv_start <= 1'b0; ev_start <= 1'b0;
    end else begin
      done <= 1'b0; smp_init <= 1'b0; smp_take <= 1'b0; wr_en <= 1'b0; rad_we <= 1'b0; rd_en <= 1'b0;
      nnu_start <= 1'b0; sq_start <= 1'b0; dv_start <= 1'b0; ev_start <= 1'b0;
      unique case (state)
        // ---------------------------------------------------------------- start, DIRECT
        S_IDLE: if (go) begin
          n_t <= '0; iterations <= '0; dd_rejects <= '0; edges <= '0; nn_queries <= '0; path_len <= '0;
          status <= ST_FAIL_ITER; a_sel <= 1'b1; smp_init <= 1'b1;
          state <= S_D2;
        end
        S_D2: begin                                      // per-joint squares, summed next cycle (timing)
          for (int j = 0; j < DIM; ++j) begin
            logic signed [JW:0] d;
            d = (JW+1)'(signed'(cfg_goal[j])) - (JW+1)'(signed'(cfg_start[j]));
            d2j[j] <= D2_W'(d * d);
          end
          state <= S_D2B;
        end
        S_D2B: begin
          logic [D2_W-1:0] t;
          t = '0;
          for (int j = 0; j < DIM; ++j) t = t + d2j[j];
          sq_x <= 64'(t) << 24; sq_start <= 1'b1; state <= S_DSQ;
        end
        S_DSQ: state <= S_DSQW;
        S_DSQW: if (sq_done) begin
          ev_s <= cfg_start;
          for (int j = 0; j < DIM; ++j) ev_vec[j] <= VW'(signed'(cfg_goal[j]) - signed'(cfg_start[j]));
          ev_len <= sq_root; ev_start <= 1'b1; edges <= edges + 32'd1; state <= S_DEVW;
        end
        S_DEVW: if (ev_done) begin
          if (ev_err) begin status <= ST_ENV_ERR; state <= S_DONE; end
          else if (ev_ok) begin
            path_mem[0] <= cfg_start; path_mem[1] <= cfg_goal; path_len <= 8'd2; status <= ST_SOLVED;
            n_t[0] <= NW'(1); n_t[1] <= NW'(1); state <= S_DONE;
          end else state <= S_INS0;
        end
        S_INS0: begin
          wr_en <= 1'b1; wr_tree <= 1'b0; wr_idx <= '0; wr_q <= cfg_start; wr_parent <= '0; n_t[0] <= NW'(1);
          state <= S_INS1;
        end
        S_INS1: begin
          wr_en <= 1'b1; wr_tree <= 1'b1; wr_idx <= '0; wr_q <= cfg_goal; wr_parent <= '0; n_t[1] <= NW'(1);
          state <= S_LOOP;
        end
        // ---------------------------------------------------------------- iteration
        S_LOOP: if (abort_q) begin
          status <= ST_ABORTED; state <= S_DONE;
        end else if (iterations >= cfg_max_iter) begin
          status <= ST_FAIL_ITER; state <= S_DONE;
        end else begin
          iterations <= iterations + 32'd1;
          mul_x <= cfg_tree_ratio; mul_y <= 32'(n_t[a_sel]);     // ratio * na
          mul_ret <= S_BAL; state <= S_MUL1;
        end
        S_BAL: begin
          logic [NW-1:0] diff;
          diff = (n_t[a_sel] > n_t[~a_sel]) ? n_t[a_sel] - n_t[~a_sel] : n_t[~a_sel] - n_t[a_sel];
          if (!cfg_balance || (64'(diff) << 16) < prod) a_sel <= ~a_sel;
          state <= S_SAMPLE;
        end
        S_SAMPLE: if (smp_valid && !nnu_busy) begin
          q_s <= smp_q; smp_take <= 1'b1;
          nnu_tree <= a_sel; nnu_qin <= smp_q; nnu_n <= n_t[a_sel]; nnu_start <= 1'b1;
          nn_queries <= nn_queries + 32'd1;
          state <= S_NNA;
        end
        S_NNA: if (nnu_done) begin
          near_i <= nnu_idx; nd2 <= nnu_d2; rd_en <= 1'b1; rd_tree <= a_sel; rd_idx <= nnu_idx; state <= S_RDA;
        end
        S_RDA: state <= S_RDA2;                                // tree read latency
        S_RDA2: begin
          qn <= rd_q; nrad <= rd_rad; mul_x <= rd_rad; mul_y <= rd_rad; state <= S_DDM;
        end
        S_DDM: begin                                     // first wait for rad^2
          mul_ret <= S_DD; state <= S_MUL2;
          for (int j = 0; j < DIM; ++j) ext[j] <= VW'(signed'(q_s[j]) - signed'(qn[j]));   // d = q_rand - q_near
        end
        S_DD: if (cfg_dd && nrad != RAD_INF && prod < (64'(nd2) << 24)) begin
          dd_rejects <= dd_rejects + 32'd1; state <= S_LOOP;
        end else begin
          sq_x <= 64'(nd2) << 24; sq_start <= 1'b1; state <= S_SQA;
        end
        S_SQA: if (sq_done) begin
          nd_h <= sq_root;
          if (sq_root < cfg_range_h) begin                     // reach: the whole vector (ext = d)
            ev_len <= sq_root; state <= S_EVE;
          end else begin                                       // scale to range: floor(d * range_h / nd_h)
            for (int j = 0; j < DIM; ++j)
              dv_a[j] <= ext_p[j];
            dv_b <= sq_root; dv_start <= 1'b1; ev_len <= cfg_range_h; state <= S_DIVE;
          end
        end
        S_DIVE: if (dv_done) begin
          for (int j = 0; j < DIM; ++j) ext[j] <= VW'(dv_q[j]);
          state <= S_EVE;
        end
        S_EVE: if (!ev_busy) begin
          ev_s <= qn; ev_vec <= ext; ev_start <= 1'b1; edges <= edges + 32'd1; state <= S_EVEW;
        end
        S_EVEW: if (ev_done) begin
          if (ev_err) begin status <= ST_ENV_ERR; state <= S_DONE; end
          else if (!ev_ok) begin
            if (cfg_dd) begin mul_x <= nrad; mul_y <= cfg_alpha; mul_ret <= S_SHRINK; state <= S_MUL1; end
            else state <= S_LOOP;
          end else begin
            for (int j = 0; j < DIM; ++j) qnew[j] <= JW'(signed'(qn[j]) + signed'(ext[j]));
            state <= S_INSN;
          end
        end
        S_SHRINK: begin                                          // rad = inf ? radius : max(rad - rad*a, min)
          logic [31:0] v;
          v = nrad - prod[63:32];
          rad_we <= 1'b1; rad_tree <= a_sel; rad_idx <= near_i;
          rad_wdata <= (nrad == RAD_INF) ? cfg_radius : ((v > cfg_min_radius) ? v : cfg_min_radius);
          state <= S_LOOP;
        end
        S_INSN: if (n_t[a_sel] == NW'(CAP)) begin
          status <= ST_FAIL_CAP; state <= S_DONE;
        end else begin
          wr_en <= 1'b1; wr_tree <= a_sel; wr_idx <= IW'(n_t[a_sel]); wr_q <= qnew; wr_parent <= near_i;
          ni <= IW'(n_t[a_sel]); n_t[a_sel] <= n_t[a_sel] + 1'b1;
          mul_x <= nrad; mul_y <= cfg_alpha; mul_ret <= S_GROW; state <= S_MUL1;
        end
        S_GROW: begin                                            // a separate cycle from the insert (TREE contract)
          if (cfg_dd && nrad != RAD_INF) begin
            rad_we <= 1'b1; rad_tree <= a_sel; rad_idx <= near_i; rad_wdata <= nrad + prod[63:32];
          end
          state <= S_CON;
        end
        // ---------------------------------------------------------------- CONNECT
        S_CON: if (!nnu_busy) begin
          nnu_tree <= ~a_sel; nnu_qin <= qnew; nnu_n <= n_t[~a_sel]; nnu_start <= 1'b1;
          nn_queries <= nn_queries + 32'd1; state <= S_NNB;
        end
        S_NNB: if (nnu_done) begin
          other_i <= nnu_idx; od2 <= nnu_d2; rd_en <= 1'b1; rd_tree <= ~a_sel; rd_idx <= nnu_idx; state <= S_RDB;
        end
        S_RDB: state <= S_RDB2;
        S_RDB2: begin
          oq <= rd_q;
          for (int j = 0; j < DIM; ++j) ovec[j] <= VW'(signed'(rd_q[j]) - signed'(qnew[j]));
          sq_x <= 64'(od2) << 24; sq_start <= 1'b1; state <= S_SQB;
        end
        S_SQB: if (sq_done) begin                                // n_ext = ceil(od_h / range_h)
          od_h <= sq_root;
          dv_a <= '0; dv_a[0] <= 50'(sq_root) + 50'(cfg_range_h) - 50'd1; dv_b <= cfg_range_h; dv_start <= 1'b1;
          state <= S_NEXT;
        end
        S_NEXT: if (dv_done) begin
          n_ext <= dv_q[0][31:0];
          prior <= qnew; prior_i <= ni; k <= 16'd1;
          if (dv_q[0] == '0) state <= S_BT_A;                    // already connected
          else begin
            dv_a <= '0; dv_a[0] <= 50'(od_h); dv_b <= dv_q[0][31:0]; dv_start <= 1'b1; state <= S_INCH;
          end
        end
        S_INCH: if (dv_done) begin
          inc_h <= dv_q[0][31:0]; state <= S_STEP;
        end
        S_STEP: begin                                            // node k = new + floor(ovec * k / n_ext)
          for (int j = 0; j < DIM; ++j) dv_a[j] <= 50'(signed'(ovec[j]) * signed'({1'b0, k}));
          dv_b <= n_ext; dv_start <= 1'b1; state <= S_STEPW;
        end
        S_STEPW: if (dv_done && !ev_busy) begin
          conf_t nx;
          for (int j = 0; j < DIM; ++j) begin
            nx[j] = JW'(signed'(qnew[j]) + signed'(dv_q[j]));
            ev_vec[j] <= VW'(signed'(nx[j]) - signed'(prior[j]));
          end
          oq <= nx;                                              // the candidate node (oq reused)
          ev_s <= prior; ev_len <= inc_h; ev_start <= 1'b1; edges <= edges + 32'd1;
          state <= S_EVCW;
        end
        S_EVCW: if (ev_done) begin
          if (ev_err) begin status <= ST_ENV_ERR; state <= S_DONE; end
          else if (!ev_ok) state <= S_LOOP;
          else if (n_t[a_sel] == NW'(CAP)) begin status <= ST_FAIL_CAP; state <= S_DONE; end
          else begin
            wr_en <= 1'b1; wr_tree <= a_sel; wr_idx <= IW'(n_t[a_sel]); wr_q <= oq; wr_parent <= prior_i;
            prior_i <= IW'(n_t[a_sel]); prior <= oq; n_t[a_sel] <= n_t[a_sel] + 1'b1;
            if (32'(k) == n_ext) state <= S_BT_A;
            else begin k <= k + 16'd1; state <= S_STEP; end
          end
        end
        // ---------------------------------------------------------------- backtrace (golden backtrace)
        S_BT_A: begin                                            // a-chain from prior_i (tree a)
          na <= '0; nb <= '0; bt_over <= 1'b0; bt_c <= prior_i;
          rd_en <= 1'b1; rd_tree <= a_sel; rd_idx <= prior_i; state <= S_BT_A2;
        end
        S_BT_A2: state <= S_BT_B;                                // wait for parent of bt_c
        S_BT_B: begin
          if (na == 8'(PATH_CAP)) bt_over <= 1'b1; else bt_a[na[PW-1:0]] <= bt_c;
          na <= na + 8'd1;
          if (rd_parent == bt_c || na == 8'(PATH_CAP)) begin      // root reached (or path too long)
            bt_c <= other_i; rd_en <= 1'b1; rd_tree <= ~a_sel; rd_idx <= other_i; state <= S_BT_B2;
          end else begin
            bt_c <= rd_parent; rd_en <= 1'b1; rd_tree <= a_sel; rd_idx <= rd_parent; state <= S_BT_A2;
          end
        end
        S_BT_B2: state <= S_BT_W;
        S_BT_W: begin                                            // b-chain: parents of other_i up to the root
          if (rd_parent == bt_c || 9'(na) + 9'(nb) > 9'(PATH_CAP)) begin
            bt_tot <= na + nb; bt_s <= '0;
            if (bt_over || 9'(na) + 9'(nb) > 9'(PATH_CAP)) begin status <= ST_FAIL_CAP; state <= S_DONE; end
            else begin bt_ph <= 2'd0; state <= S_BT_W2; end
          end else begin
            if (nb != 8'(PATH_CAP)) bt_b[nb[PW-1:0]] <= rd_parent;
            nb <= nb + 8'd1;
            bt_c <= rd_parent; rd_en <= 1'b1; rd_tree <= ~a_sel; rd_idx <= rd_parent; state <= S_BT_B2;
          end
        end
        S_BT_W2: begin                                           // path element s: issue read, wait, store
          if (bt_s == bt_tot) begin
            path_len <= bt_tot; status <= ST_SOLVED; state <= S_DONE;
          end else if (bt_ph == 2'd0) begin
            logic [7:0] kk;
            kk = a_sel == 1'b0 ? bt_s : bt_tot - 8'd1 - bt_s;      // a_is_start = (a is tree 0)
            rd_en <= 1'b1;
            if (kk < na) begin rd_tree <= a_sel; rd_idx <= bt_a[PW'(na - 8'd1 - kk)]; end
            else begin rd_tree <= ~a_sel; rd_idx <= bt_b[PW'(kk - na)]; end
            bt_ph <= 2'd1;
          end else if (bt_ph == 2'd1) bt_ph <= 2'd2;             // tree read latency
          else begin
            path_mem[bt_s[PW-1:0]] <= rd_q; bt_s <= bt_s + 8'd1; bt_ph <= 2'd0;
          end
        end
        S_MUL1: state <= S_MUL2;                                 // multiplier pipeline
        S_MUL2: state <= mul_ret;
        S_DONE: begin done <= 1'b1; state <= S_IDLE; end
        default: state <= S_IDLE;
      endcase
    end
  end

  assign size_start = 16'(n_t[0]);
  assign size_goal  = 16'(n_t[1]);

  logic unused;
  assign unused = ^{nd_h, dv_r, dv_busy, sq_busy, od_h[0]};

endmodule
