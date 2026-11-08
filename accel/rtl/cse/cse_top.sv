// CSE top-level port list (design doc section 4.2). Skeleton: outputs idle, to be implemented.
module cse_top
  import ple_cse_pkg::*;
(
    input  logic                      clk,
    input  logic                      rst_n,
    // Q: query (PLE -> CSE)
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
    // R: response (CSE -> PLE)
    output logic                      r_valid,
    input  logic                      r_ready,
    output logic [TAG_W-1:0]          r_tag,
    output logic [USER_W-1:0]         r_user,
    output logic [1:0]                r_status,
    // K: kill (PLE -> CSE)
    input  logic                      k_valid,
    output logic                      k_ready,
    input  logic                      k_all,
    input  logic [TAG_W-1:0]          k_tag,
    // status
    output logic                      cse_idle,
    output logic                      cse_err
    // environment memory ports: defined by the CSE owner (INTERFACES.md I3)
);

  assign q_ready  = 1'b0;
  assign r_valid  = 1'b0;
  assign r_tag    = '0;
  assign r_user   = '0;
  assign r_status = ST_MISS;
  assign k_ready  = 1'b1;     // v1: kill ignored (decision A2)
  assign cse_idle = 1'b1;
  assign cse_err  = 1'b0;

  logic unused;
  assign unused = ^{clk, rst_n, q_valid, q_tag, q_user, q_kill_on_hit, q_env_buf, q_cx, q_cy, q_cz, q_r,
                    r_ready, k_valid, k_all, k_tag};

endmodule
