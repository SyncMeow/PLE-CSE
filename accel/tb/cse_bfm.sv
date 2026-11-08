// CSE behavioural model for PLE testbenches: answers every query from the golden cse_ref (DPI-C) on the
// environment loaded with fx_ref_env_load(). Random q_ready / k_ready stalls, latency 1..MAXLAT cycles,
// out-of-order answers, KILLED (or the real result) for queries a kill covered, optional auto-kill.
// Follows the interface contract C1-C6 (checked by ple_cse_if_checker in the testbenches).
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

  typedef struct {
    logic [TAG_W-1:0] tag;
    logic [USER_W-1:0] user;
    logic koh;
    int cx, cy, cz, rq;
    longint unsigned due;
    bit killed;
  } ent_t;
  ent_t cq[$];
  longint unsigned cyc = 0;
  int nq = 0, nkilled = 0, nhit = 0;
  int maxlat = MAXLAT;                                 // the testbench may change these at run time
  int rdy_pct = 75;

  assign cse_err = 1'b0;
  always_comb cse_idle = (cq.size() == 0) && !r_valid;

  always_ff @(posedge clk) begin
    cyc <= cyc + 1;
    if (!rst_n) begin
      q_ready <= 1'b0; k_ready <= 1'b0; r_valid <= 1'b0; r_tag <= '0; r_user <= '0; r_status <= '0;
    end else begin
      q_ready <= ($urandom % 100) < rdy_pct;
      k_ready <= ($urandom % 100) < 70;
      if (k_valid && k_ready) foreach (cq[i]) if (k_all || cq[i].tag == k_tag) cq[i].killed = 1;
      if (q_valid && q_ready) begin
        cq.push_back('{q_tag, q_user, q_kill_on_hit, int'(q_cx), int'(q_cy), int'(q_cz), int'(q_r),
                       cyc + 1 + 64'($urandom % maxlat), 1'b0});
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
          if (e.killed && ($urandom % 2)) begin st = ST_KILLED; nkilled++; end
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
