// PLE + CSE accelerator: ple_top (AXI-Lite slave) driving cse_top over Q/R/K, environment memory outside the CSE core.
// The environment is written through env_wr_* (a loader or DMA in the SoC; port layout of cse_env_mem).
module ple_cse_top
  import ple_cse_pkg::*;
#(
    parameter int unsigned N_CU  = 2,
    parameter int          N_BUF = 1       // environment buffers (1 or 2); MODE.env_buf must be 0 when 1
) (
    input  logic              clk,
    input  logic              rst_n,
    input  logic [11:0]       s_axil_awaddr,
    input  logic              s_axil_awvalid,
    output logic              s_axil_awready,
    input  logic [31:0]       s_axil_wdata,
    input  logic [3:0]        s_axil_wstrb,
    input  logic              s_axil_wvalid,
    output logic              s_axil_wready,
    output logic [1:0]        s_axil_bresp,
    output logic              s_axil_bvalid,
    input  logic              s_axil_bready,
    input  logic [11:0]       s_axil_araddr,
    input  logic              s_axil_arvalid,
    output logic              s_axil_arready,
    output logic [31:0]       s_axil_rdata,
    output logic [1:0]        s_axil_rresp,
    output logic              s_axil_rvalid,
    input  logic              s_axil_rready,
    output logic              irq,
    input  logic              env_wr_en,
    input  logic              env_wr_buf,
    input  env_region_e       env_wr_region,
    input  logic [PT_AW-1:0]  env_wr_addr,
    input  logic [127:0]      env_wr_data
);

  logic                      q_valid, q_ready, q_kill_on_hit, q_env_buf;
  logic [TAG_W-1:0]          q_tag, r_tag, k_tag;
  logic [USER_W-1:0]         q_user, r_user;
  logic signed [COORD_W-1:0] q_cx, q_cy, q_cz;
  logic [RQ_W-1:0]           q_r;
  logic                      r_valid, r_ready, k_valid, k_ready, k_all, cse_idle, cse_err;
  logic [1:0]                r_status;

  logic [N_CU-1:0]                rd_buf;
  logic [N_CU-1:0][DIR_AW-1:0]    dir_addr;
  logic [N_CU-1:0][15:0]          dir_data;
  logic [N_CU-1:0][SLOT_W-1:0]    cell_addr;
  logic [N_CU-1:0][127:0]         cell_data;
  logic [N_CU-1:0][PT_AW-1:0]     pt_addr;
  logic [N_CU-1:0][PT_DATA_W-1:0] pt_data;
  logic [1:0]                     hdr_valid, hdr_err;
  logic [1:0][PT_AW-1:0]          hdr_n_pts;
  logic [1:0][SLOT_W:0]           hdr_n_cells;
  logic [1:0][2:0][PT_W-1:0]      hdr_glo, hdr_ghi;

  ple_top u_ple (
      .clk, .rst_n,
      .s_axil_awaddr, .s_axil_awvalid, .s_axil_awready, .s_axil_wdata, .s_axil_wstrb, .s_axil_wvalid, .s_axil_wready,
      .s_axil_bresp, .s_axil_bvalid, .s_axil_bready, .s_axil_araddr, .s_axil_arvalid, .s_axil_arready,
      .s_axil_rdata, .s_axil_rresp, .s_axil_rvalid, .s_axil_rready, .irq,
      .q_valid, .q_ready, .q_tag, .q_user, .q_kill_on_hit, .q_env_buf, .q_cx, .q_cy, .q_cz, .q_r,
      .r_valid, .r_ready, .r_tag, .r_user, .r_status, .k_valid, .k_ready, .k_all, .k_tag, .cse_idle, .cse_err);

  cse_top #(.N_CU(N_CU)) u_cse (
      .clk, .rst_n,
      .q_valid, .q_ready, .q_tag, .q_user, .q_kill_on_hit, .q_env_buf, .q_cx, .q_cy, .q_cz, .q_r,
      .r_valid, .r_ready, .r_tag, .r_user, .r_status, .k_valid, .k_ready, .k_all, .k_tag, .cse_idle, .cse_err,
      .rd_buf, .dir_addr, .dir_data, .cell_addr, .cell_data, .pt_addr, .pt_data,
      .hdr_valid, .hdr_err, .hdr_n_pts, .hdr_glo, .hdr_ghi);

  cse_env_mem #(.NRD(N_CU), .N_BUF(N_BUF)) u_env (
      .clk, .rd_buf, .dir_addr, .dir_data, .cell_addr, .cell_data, .pt_addr, .pt_data,
      .hdr_valid, .hdr_err, .hdr_n_pts, .hdr_n_cells, .hdr_glo, .hdr_ghi,
      .wr_en(env_wr_en), .wr_buf(env_wr_buf), .wr_region(env_wr_region), .wr_addr(env_wr_addr), .wr_data(env_wr_data));

  logic unused_cells;
  assign unused_cells = ^hdr_n_cells;

endmodule
