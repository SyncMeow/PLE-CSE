// PLE top-level port list (design doc sections 4.2 and 6). Skeleton: outputs idle, to be implemented.
module ple_top
  import ple_cse_pkg::*;
(
    input  logic                      clk,
    input  logic                      rst_n,
    // host: AXI-Lite slave (register map draft in section 6) + interrupt
    input  logic [11:0]               s_axil_awaddr,
    input  logic                      s_axil_awvalid,
    output logic                      s_axil_awready,
    input  logic [31:0]               s_axil_wdata,
    input  logic [3:0]                s_axil_wstrb,
    input  logic                      s_axil_wvalid,
    output logic                      s_axil_wready,
    output logic [1:0]                s_axil_bresp,
    output logic                      s_axil_bvalid,
    input  logic                      s_axil_bready,
    input  logic [11:0]               s_axil_araddr,
    input  logic                      s_axil_arvalid,
    output logic                      s_axil_arready,
    output logic [31:0]               s_axil_rdata,
    output logic [1:0]                s_axil_rresp,
    output logic                      s_axil_rvalid,
    input  logic                      s_axil_rready,
    output logic                      irq,
    // Q: query (PLE -> CSE)
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
    // R: response (CSE -> PLE)
    input  logic                      r_valid,
    output logic                      r_ready,
    input  logic [TAG_W-1:0]          r_tag,
    input  logic [USER_W-1:0]         r_user,
    input  logic [1:0]                r_status,
    // K: kill (PLE -> CSE)
    output logic                      k_valid,
    input  logic                      k_ready,
    output logic                      k_all,
    output logic [TAG_W-1:0]          k_tag,
    // status
    input  logic                      cse_idle,
    input  logic                      cse_err
);

  assign s_axil_awready = 1'b0;
  assign s_axil_wready  = 1'b0;
  assign s_axil_bresp   = 2'b00;
  assign s_axil_bvalid  = 1'b0;
  assign s_axil_arready = 1'b0;
  assign s_axil_rdata   = '0;
  assign s_axil_rresp   = 2'b00;
  assign s_axil_rvalid  = 1'b0;
  assign irq            = 1'b0;
  assign q_valid        = 1'b0;
  assign q_tag          = '0;
  assign q_user         = '0;
  assign q_kill_on_hit  = 1'b0;
  assign q_env_buf      = 1'b0;
  assign q_cx           = '0;
  assign q_cy           = '0;
  assign q_cz           = '0;
  assign q_r            = '0;
  assign r_ready        = 1'b1;     // P1: v1 ties r_ready high
  assign k_valid        = 1'b0;
  assign k_all          = 1'b0;
  assign k_tag          = '0;

  logic unused;
  assign unused = ^{clk, rst_n, s_axil_awaddr, s_axil_awvalid, s_axil_wdata, s_axil_wstrb, s_axil_wvalid,
                    s_axil_bready, s_axil_araddr, s_axil_arvalid, s_axil_rready, q_ready, r_valid, r_tag,
                    r_user, r_status, k_ready, cse_idle, cse_err};

endmodule
