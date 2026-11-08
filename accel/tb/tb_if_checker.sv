// Self-test of ple_cse_if_checker: random legal PLE/CSE traffic must give 0 errors; +bug=N injects one
// protocol violation that the checker must report. Also the seed of the CSE / PLE behavioural models.
//   1 drop a response      2 duplicate a response   3 change R payload while stalled
//   4 KILLED without kill  5 query after kill       6 q_r > R_MAX
//   7 cse_idle stuck high  8 response stall > LAT_LIMIT
module tb_if_checker #(parameter bit AK = 1'b1);   // AK: CSE model auto-kills
  import ple_cse_pkg::*;

  localparam int NQ = 4000, NT = 4, LAT = 200;
  logic clk = 1'b0, rst_n = 1'b0, eot = 1'b0;
  always #5 clk = ~clk;
  int bug = 0;
  longint unsigned cyc = 0;

  // interface signals
  logic q_valid, q_ready, q_kill_on_hit, q_env_buf, r_valid, r_ready, k_valid, k_ready, k_all, cse_idle, cse_err;
  logic [TAG_W-1:0] q_tag, r_tag, k_tag;
  logic [USER_W-1:0] q_user, r_user;
  logic signed [COORD_W-1:0] q_cx, q_cy, q_cz;
  logic [RQ_W-1:0] q_r;
  logic [1:0] r_status;

  ple_cse_if_checker #(.AUTO_KILL(AK), .LAT_LIMIT(LAT)) u_chk (.*);

  // ---------------------------------------------------------------- PLE-side driver
  int sent = 0, out_d[1 << TAG_W];
  bit blocked[1 << TAG_W];
  bit bug5_done = 0, bug6_done = 0;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      q_valid <= 0; k_valid <= 0; k_all <= 0; k_tag <= 0;
      q_tag <= 0; q_user <= 0; q_kill_on_hit <= 0; q_env_buf <= 0; q_cx <= 0; q_cy <= 0; q_cz <= 0; q_r <= 0;
      for (int t = 0; t < (1 << TAG_W); ++t) begin out_d[t] = 0; blocked[t] = 0; end
    end else begin
      if (r_valid && r_ready) begin
        out_d[r_tag]--;
        if (out_d[r_tag] == 0) blocked[r_tag] = 0;
      end
      if (k_valid && k_ready) begin
        blocked[k_tag] = 1;
        k_valid <= 0;
      end else if (!k_valid && ($urandom % 100) < 2) begin
        int t = $urandom % NT;
        if (out_d[t] != 0 && !blocked[t]) begin k_valid <= 1; k_tag <= t[TAG_W-1:0]; end
      end
      if (q_valid && q_ready) begin
        out_d[q_tag]++;
        q_valid <= 0;
      end
      if ((!q_valid || q_ready) && sent < NQ && ($urandom % 100) < 60) begin
        int t = $urandom % NT;
        bit force_blk = (bug == 5) && !bug5_done && blocked[t];
        if (!blocked[t] || force_blk) begin
          if (force_blk) bug5_done = 1;
          q_valid <= 1;
          q_tag <= t[TAG_W-1:0];
          q_user <= sent[USER_W-1:0];
          q_kill_on_hit <= 1'($urandom % 2);
          q_cx <= COORD_W'($urandom); q_cy <= COORD_W'($urandom); q_cz <= COORD_W'($urandom);
          q_r <= RQ_W'($urandom % (32'(R_MAX) + 1));
          if (bug == 6 && !bug6_done && sent == NQ / 2) begin q_r <= R_MAX + 1; bug6_done = 1; end
          sent++;
        end
      end
    end
  end

  // ---------------------------------------------------------------- CSE-side responder
  typedef struct {
    logic [TAG_W-1:0] tag;
    logic [USER_W-1:0] user;
    logic koh;
    longint unsigned due;
    bit killed;
  } ent_t;
  ent_t q[$];
  int out_c[1 << TAG_W];
  bit b1 = 0, b2 = 0, b3 = 0, b4 = 0, b8 = 0;
  ent_t dup;
  bit have_dup = 0;

  assign cse_err = 1'b0;
  always_comb cse_idle = (bug == 7) ? 1'b1 : (q.size() == 0 && !r_valid);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      r_valid <= 0; r_tag <= 0; r_user <= 0; r_status <= 0; q_ready <= 0; k_ready <= 0; r_ready <= 0;
      for (int t = 0; t < (1 << TAG_W); ++t) out_c[t] = 0;
    end else begin
      q_ready <= ($urandom % 100) < 70;
      k_ready <= ($urandom % 100) < 80;
      r_ready <= (bug == 8 && cyc > 3000 && !b8) ? 1'b1 : (($urandom % 100) < 85);   // bug 8: ready held high so C5 applies
      if (k_valid && k_ready)
        foreach (q[i]) if (k_all || q[i].tag == k_tag) q[i].killed = 1;
      if (q_valid && q_ready) begin
        q.push_back('{q_tag, q_user, q_kill_on_hit, cyc + 1 + 64'($urandom % 60), 1'b0});
        out_c[q_tag]++;
      end
      if (r_valid && r_ready) begin
        out_c[r_tag]--;
        r_valid <= 0;
      end else if (r_valid && bug == 3 && !b3) begin
        r_user <= r_user ^ 1;                          // bug 3: payload change while stalled
        b3 = 1;
      end
      if (bug == 8 && !b8 && cyc > 3000 && cyc < 64'(3000) + 64'(LAT) + 64'(50)) begin
        if (cyc == 64'(3000) + 64'(LAT) + 64'(49)) b8 = 1;            // bug 8: stop answering
      end else if (!r_valid || r_ready) begin
        if (have_dup) begin
          r_valid <= 1; r_tag <= dup.tag; r_user <= dup.user; r_status <= ST_MISS;
          out_c[dup.tag]++;
          have_dup = 0;
        end else begin
          int pick = -1;
          for (int i = 0; i < q.size() && i < 16; ++i)
            if (q[i].due <= cyc && ($urandom % 3) != 0) begin pick = i; break; end
          if (pick >= 0) begin
            ent_t e = q[pick];
            logic [1:0] st;
            q.delete(pick);
            if (bug == 1 && !b1 && cyc > 2000) begin   // bug 1: drop it
              b1 = 1;
              out_c[e.tag]--;
            end else begin
              if (e.killed && ($urandom % 2) != 0) st = ST_KILLED;
              else if (($urandom % 100) < 10) begin
                st = ST_HIT;
                if (AK && e.koh) foreach (q[i]) if (q[i].tag == e.tag) q[i].killed = 1;   // auto-kill
              end else st = ST_MISS;
              if (bug == 4 && !b4 && cyc > 2000 && !e.killed) begin st = ST_KILLED; b4 = 1; end
              r_valid <= 1; r_tag <= e.tag; r_user <= e.user; r_status <= st;
              if (bug == 2 && !b2 && cyc > 2000) begin dup = e; have_dup = 1; b2 = 1; end
            end
          end
        end
      end
    end
  end

  initial begin
    if (!$value$plusargs("bug=%d", bug)) bug = 0;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    while (cyc < 200000 && !(sent == NQ && !q_valid && q.size() == 0 && !r_valid && !have_dup)) @(posedge clk);
    repeat (5) @(posedge clk);
    eot = 1'b1;
    repeat (2) @(posedge clk);
    $display("TB auto_kill=%0d bug=%0d cycles=%0d checker_errors=%0d", AK, bug, cyc, u_chk.errors);
    $finish;
  end
  always @(posedge clk) cyc <= cyc + 1;

endmodule
