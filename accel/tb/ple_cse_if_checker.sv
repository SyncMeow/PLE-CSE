// Passive monitor of the PLE <-> CSE interface: handshake rules (4.3) and invariants (4.5).
// Checks H1-H3 (valid/payload stability), C1/C2 (exactly once, echoed tag/user), C4 (KILLED only after a
// kill), C5 (response gap <= MAX_LAT), C6 (cse_idle => nothing outstanding), P2 (no query after kill until
// the tag retires), P5 (q_r <= R_MAX), and drain at end of test. Errors are counted in `errors` (first 20 printed).
module ple_cse_if_checker
  import ple_cse_pkg::*;
#(
    parameter bit AUTO_KILL = 1'b1,           // CSE may auto-kill on a kill_on_hit HIT
    parameter int unsigned LAT_LIMIT = MAX_LAT
) (
    input logic                      clk,
    input logic                      rst_n,
    input logic                      q_valid, q_ready,
    input logic [TAG_W-1:0]          q_tag,
    input logic [USER_W-1:0]         q_user,
    input logic                      q_kill_on_hit, q_env_buf,
    input logic signed [COORD_W-1:0] q_cx, q_cy, q_cz,
    input logic [RQ_W-1:0]           q_r,
    input logic                      r_valid, r_ready,
    input logic [TAG_W-1:0]          r_tag,
    input logic [USER_W-1:0]         r_user,
    input logic [1:0]                r_status,
    input logic                      k_valid, k_ready, k_all,
    input logic [TAG_W-1:0]          k_tag,
    input logic                      cse_idle, cse_err,
    input logic                      eot             // end of test: everything must be drained
);

  localparam int NTAG = 1 << TAG_W;
  typedef logic [TAG_W+USER_W-1:0] key_t;

  int unsigned errors = 0;
  int unsigned n_query = 0, n_resp = 0, n_killed = 0, n_kill = 0;

  int unsigned pend[key_t];                   // outstanding queries per {tag, user}
  int unsigned pend_koh[key_t];               // of which kill_on_hit = 1
  longint unsigned acc_t[key_t][$];           // accept cycle of each outstanding query (FIFO per key)
  longint unsigned last_k[NTAG];              // last cycle a K covered the tag (0 = never)
  longint unsigned last_kh[NTAG];             // last cycle a kill_on_hit HIT of the tag was answered
  longint unsigned un_lo[NTAG][$], un_hi[NTAG][$];   // KILLED lifetimes not yet justified
  longint unsigned now = 1;
  int unsigned out_tag[NTAG];                 // outstanding per tag
  bit k_block[NTAG];                          // P2: K(T) sent, T not retired yet
  bit k_allow[NTAG];                          // P2: the query already on Q at the K handshake may still go
  int unsigned total_out = 0, gap = 0;

  // previous-cycle copies for the stability rules
  logic pq_v, pq_r, pr_v, pr_r, pk_v, pk_r, eot_q;
  query_t pq;
  resp_t pr;
  kill_t pk;
  query_t cq;
  resp_t cr;
  kill_t ck;
  assign cq = '{q_tag, q_user, q_kill_on_hit, q_env_buf, q_cx, q_cy, q_cz, q_r};
  assign cr = '{r_tag, r_user, status_e'(r_status)};
  assign ck = '{k_all, k_tag};

  task automatic fail(input string what);
    if (errors < 20) $display("[if_checker] %0t ERROR %s", $time, what);
    errors++;
  endtask

  function automatic key_t mk(input logic [TAG_W-1:0] t, input logic [USER_W-1:0] u);
    return {t, u};
  endfunction

  always @(posedge clk) begin   // testbench scoreboard: blocking updates are intended
    if (!rst_n) begin
      pq_v <= 1'b0; pr_v <= 1'b0; pk_v <= 1'b0;
      pq_r <= 1'b0; pr_r <= 1'b0; pk_r <= 1'b0; eot_q <= 1'b0;
      pend.delete(); pend_koh.delete(); acc_t.delete();
      for (int t = 0; t < NTAG; ++t) begin
        out_tag[t] = 0; k_block[t] = 1'b0;
        k_allow[t] = 1'b0; last_k[t] = 0; last_kh[t] = 0; un_lo[t].delete(); un_hi[t].delete();
      end
      total_out = 0; gap = 0; now = 1;
    end else begin
      int unsigned out_before;
      out_before = total_out;
      // H1-H3: once valid, hold valid and payload until the handshake
      if (pq_v && !pq_r && (!q_valid || cq != pq)) fail("Q valid dropped or payload changed before q_ready");
      if (pr_v && !pr_r && (!r_valid || cr != pr)) fail("R valid dropped or payload changed before r_ready");
      if (pk_v && !pk_r && (!k_valid || ck != pk)) fail("K valid dropped or payload changed before k_ready");

      // K handshake: applies to queries outstanding at this cycle (before this cycle's Q/R)
      if (k_valid && k_ready) begin
        n_kill++;
        for (int t = 0; t < NTAG; ++t)
          if (k_all || k_tag == t[TAG_W-1:0]) begin
            last_k[t] = now;
            // nothing in flight and nothing waiting on Q: the tag is retired at once
            if (out_tag[t] != 0 || (q_valid && q_tag == t[TAG_W-1:0])) k_block[t] = 1'b1;
            if (q_valid && q_tag == t[TAG_W-1:0]) k_allow[t] = 1'b1;
          end
      end

      // R handshake
      if (r_valid && r_ready) begin
        key_t k;
        n_resp++;
        k = mk(r_tag, r_user);
        if ((pend.exists(k) == 0) || pend[k] == 0) fail($sformatf("C1/C2 response without a matching query tag=%0d user=%h", r_tag, r_user));
        else begin
          longint unsigned a;
          a = acc_t[k].pop_front();
          pend[k]--;
          out_tag[r_tag]--;
          total_out--;
          if (r_status == ST_HIT && (pend_koh.exists(k) != 0) && pend_koh[k] != 0) begin
            pend_koh[k]--;
            last_kh[r_tag] = now;
            // auto-kill by this HIT can only reach queries whose lifetime overlaps [a, now]
            for (int i = un_lo[r_tag].size() - 1; i >= 0; --i)
              if (un_hi[r_tag][i] >= a) begin un_lo[r_tag].delete(i); un_hi[r_tag].delete(i); end
          end else if ((pend_koh.exists(k) != 0) && pend_koh[k] > pend[k]) pend_koh[k]--;
          if (r_status == ST_KILLED) begin
            n_killed++;
            // justified now by a K during its lifetime; otherwise wait for an overlapping auto-kill HIT
            if (!((last_k[r_tag] != 0 && last_k[r_tag] >= a) || (AUTO_KILL && last_kh[r_tag] != 0 && last_kh[r_tag] >= a))) begin
              if (!AUTO_KILL) fail($sformatf("C4 KILLED on tag %0d user=%h without a kill", r_tag, r_user));
              else begin un_lo[r_tag].push_back(a); un_hi[r_tag].push_back(now); end
            end
          end
          // tag retired: C4 is decided here (an auto-kill HIT may arrive after the KILLED responses)
          if (out_tag[r_tag] == 0) begin
            if (un_lo[r_tag].size() != 0)
              fail($sformatf("C4 %0d KILLED on tag %0d without a kill", un_lo[r_tag].size(), r_tag));
            un_lo[r_tag].delete(); un_hi[r_tag].delete();
            if (!k_allow[r_tag]) k_block[r_tag] = 1'b0;
          end
        end
      end

      // Q handshake
      if (q_valid && q_ready) begin
        key_t k;
        n_query++;
        if (k_block[q_tag]) begin
          if (k_allow[q_tag]) k_allow[q_tag] = 1'b0;
          else fail($sformatf("P2 new query on tag %0d after K before the tag retired", q_tag));
        end
        if (q_r > R_MAX) fail($sformatf("P5 q_r=%0d > R_MAX", q_r));
        k = mk(q_tag, q_user);
        if ((pend.exists(k) == 0)) pend[k] = 0;
        if ((pend_koh.exists(k) == 0)) pend_koh[k] = 0;
        pend[k]++;
        acc_t[k].push_back(now);
        if (q_kill_on_hit) pend_koh[k]++;
        out_tag[q_tag]++;
        total_out++;
      end

      // C5: with r_ready high and work outstanding, some response must arrive within LAT_LIMIT cycles
      if ((r_valid && r_ready) || total_out == 0) gap = 0;
      else if (r_ready) gap++;                  // paused (not reset) while r_ready is low
      if (gap == LAT_LIMIT) fail("C5 no response for LAT_LIMIT cycles");
      // C6: idle means nothing outstanding (checked against the state before this cycle's handshakes)
      if (cse_idle && out_before != 0) fail("C6 cse_idle with outstanding queries");
      if (eot && !eot_q && total_out != 0) fail($sformatf("drain: %0d queries never answered", total_out));
      if (eot && !eot_q)
        for (int t = 0; t < NTAG; ++t)
          if (un_lo[t].size() != 0) fail($sformatf("C4 %0d KILLED on tag %0d without a kill", un_lo[t].size(), t));
      now++;
      eot_q <= eot;

      pq_v <= q_valid; pq_r <= q_ready; pq <= cq;
      pr_v <= r_valid; pr_r <= r_ready; pr <= cr;
      pk_v <= k_valid; pk_r <= k_ready; pk <= ck;
    end
  end

  logic unused_err;
  assign unused_err = cse_err;   // cse_err semantics are checked by the testbench, not here

  final $display("[if_checker] queries=%0d responses=%0d killed=%0d kills=%0d outstanding=%0d errors=%0d",
                 n_query, n_resp, n_killed, n_kill, total_out, errors);

endmodule
