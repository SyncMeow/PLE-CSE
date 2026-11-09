// CSE behavioural model for PLE testbenches: answers every query from the golden cse_ref (DPI-C) on the
// environment loaded with fx_ref_env_load(). Random q_ready / k_ready stalls, latency 1..MAXLAT cycles,
// out-of-order answers, KILLED (or the real result) for queries a kill covered, optional auto-kill.
// Follows the interface contract C1-C6 (checked by ple_cse_if_checker in the testbenches).
// Performance mode (mode != 0, set by the testbench before reset ends; mode 0 above is unchanged):
//   deterministic timing. Input FIFO of fd entries (q_ready = not full); nu units take queries in order;
//   a killed query at the FIFO head is answered KILLED without a unit. mode 1: a unit is busy ii cycles,
//   the result comes lat cycles after it starts. mode 2: a unit is busy cost cycles (fx_ref_cse_cost:
//   design doc 4.1 per-cell cost with `lanes` point lanes), the result comes cost + lat cycles after it
//   starts. One response per cycle, earliest first. k_ready = 1.
module cse_bfm
  import ple_cse_pkg::*;
#(
    parameter bit AK     = 1'b0,                         // auto-kill on a kill_on_hit HIT
    parameter int MAXLAT = 40
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
    output logic                      cse_err
);

  import "DPI-C" function int fx_ref_cse(input int cx, input int cy, input int cz, input int rq);
  import "DPI-C" function int fx_ref_cse_cost(input int cx, input int cy, input int cz, input int rq, input int lanes);

  typedef struct {
    logic [TAG_W-1:0] tag;
    logic [USER_W-1:0] user;
    logic koh;
    int cx, cy, cz, rq;
    longint unsigned due;
    bit killed;
    int idx;                                         // query number (for ERROR injection)
    logic [1:0] res;                                 // performance mode: the answer
  } ent_t;
  ent_t cq[$];
  longint unsigned cyc = 0;
  int nq = 0, nkilled = 0, nhit = 0;
  int maxlat = MAXLAT;                                 // the testbench may change these at run time
  int rdy_pct = 75;
  int err_at = -1;                                     // query number answered ERROR (-1: none)

  // performance mode
  int mode = 0, lat = 4, ii = 1, nu = 1, fd = 8, lanes = 8;
  longint unsigned work_cycles = 0, busy_unit_cycles = 0, fifo_full_cycles = 0;
  ent_t fifo[$], outq[$];
  longint unsigned ufree[16];

  assign cse_err = 1'b0;
  always_comb cse_idle = (cq.size() == 0) && (fifo.size() == 0) && (outq.size() == 0) && !r_valid;

  task automatic perf_cycle();
    k_ready <= 1'b1;
    if (k_valid && k_ready) foreach (fifo[i]) if (k_all || fifo[i].tag == k_tag) fifo[i].killed = 1;
    if (q_valid && q_ready) begin
      fifo.push_back('{q_tag, q_user, q_kill_on_hit, int'(q_cx), int'(q_cy), int'(q_cz), int'(q_r), 0, 1'b0, nq, ST_MISS});
      nq++;
    end
    for (int u = 0; u < nu; ++u) begin
      while (fifo.size() > 0 && fifo[0].killed) begin            // killed before it started: no unit used
        ent_t e;
        e = fifo.pop_front(); e.due = cyc + 1; e.res = ST_KILLED; outq.push_back(e);
      end
      if (fifo.size() > 0 && ufree[u] <= cyc) begin
        ent_t e;
        int c;
        e = fifo.pop_front();
        c = (mode == 1) ? ii : fx_ref_cse_cost(e.cx, e.cy, e.cz, e.rq, lanes);
        ufree[u] = cyc + 64'(c);
        work_cycles += 64'(c);
        e.due = cyc + 64'(lat) + ((mode == 1) ? 64'd0 : 64'(c));
        if (e.idx == err_at) e.res = ST_ERROR;
        else e.res = fx_ref_cse(e.cx, e.cy, e.cz, e.rq) ? ST_HIT : ST_MISS;
        outq.push_back(e);
      end
    end
    for (int u = 0; u < nu; ++u) busy_unit_cycles += 64'(ufree[u] > cyc);
    fifo_full_cycles += 64'(fifo.size() >= fd);
    q_ready <= fifo.size() < fd;
    if (r_valid && r_ready) r_valid <= 1'b0;
    if (!r_valid || r_ready) begin
      int pick;
      pick = -1;
      foreach (outq[i]) if (outq[i].due <= cyc && (pick < 0 || outq[i].due < outq[pick].due)) pick = i;
      if (pick >= 0) begin
        ent_t e;
        e = outq[pick];
        outq.delete(pick);
        nkilled += int'(e.res == ST_KILLED);
        nhit += int'(e.res == ST_HIT);
        if (AK && e.koh && e.res == ST_HIT) foreach (fifo[i]) if (fifo[i].tag == e.tag) fifo[i].killed = 1;
        r_valid <= 1'b1; r_tag <= e.tag; r_user <= e.user; r_status <= e.res;
      end
    end
  endtask

  always_ff @(posedge clk) begin
    cyc <= cyc + 1;
    if (!rst_n) begin
      q_ready <= 1'b0; k_ready <= 1'b0; r_valid <= 1'b0; r_tag <= '0; r_user <= '0; r_status <= '0;
      foreach (ufree[i]) ufree[i] = 0;
    end else if (mode != 0) perf_cycle();
    else begin
      q_ready <= ($urandom % 100) < rdy_pct;
      k_ready <= ($urandom % 100) < 70;
      if (k_valid && k_ready) foreach (cq[i]) if (k_all || cq[i].tag == k_tag) cq[i].killed = 1;
      if (q_valid && q_ready) begin
        cq.push_back('{q_tag, q_user, q_kill_on_hit, int'(q_cx), int'(q_cy), int'(q_cz), int'(q_r),
                       cyc + 1 + 64'($urandom % maxlat), 1'b0, nq, ST_MISS});
        nq++;
      end
      if (r_valid && r_ready) r_valid <= 1'b0;
      if (!r_valid || r_ready) begin
        int pick;
        pick = -1;
        for (int i = 0; i < cq.size() && i < 24; ++i) if (cq[i].due <= cyc && (rdy_pct == 100 || ($urandom % 3) != 0)) begin pick = i; break; end
        if (pick >= 0) begin
          ent_t e;
          logic [1:0] st;
          e = cq[pick];
          cq.delete(pick);
          if (e.idx == err_at) st = ST_ERROR;
          else if (e.killed && ($urandom % 2)) begin st = ST_KILLED; nkilled++; end
          else begin
            st = fx_ref_cse(e.cx, e.cy, e.cz, e.rq) ? ST_HIT : ST_MISS;
            nhit += int'(st == ST_HIT);
            if (AK && e.koh && st == ST_HIT) foreach (cq[i]) if (cq[i].tag == e.tag) cq[i].killed = 1;
          end
          r_valid <= 1'b1; r_tag <= e.tag; r_user <= e.user; r_status <= st;
        end
      end
    end
  end

  logic unused;
  assign unused = q_env_buf;

endmodule
