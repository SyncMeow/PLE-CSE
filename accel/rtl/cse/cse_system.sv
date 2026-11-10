// Simulation/integration wrapper that places cse_env_mem outside the CSE core top.
module cse_system
  import ple_cse_pkg::*;
#(
    parameter int unsigned N_CU = 4,
    parameter int unsigned INPUT_FIFO_DEPTH = 8,
    parameter int unsigned RESPONSE_FIFO_DEPTH = 8,
    parameter bit KILL_ENABLE = 1'b1,
    parameter int unsigned CU_VOXEL_FIFO_DEPTH = 2,
    parameter int unsigned CU_CELL_FIFO_DEPTH = 2,
    parameter int unsigned CU_MAX_PTS_OUTSTANDING = 8
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

  logic [N_CU-1:0]                    rd_buf;
  logic [N_CU-1:0][DIR_AW-1:0]        dir_addr;
  logic [N_CU-1:0][15:0]              dir_data;
  logic [N_CU-1:0][SLOT_W-1:0]        cell_addr;
  logic [N_CU-1:0][127:0]             cell_data;
  logic [N_CU-1:0][PT_AW-1:0]         pt_addr;
  logic [N_CU-1:0][PT_DATA_W-1:0]     pt_data;
  logic [1:0]                          hdr_valid, hdr_err;
  logic [1:0][PT_AW-1:0]               hdr_n_pts;
  logic [1:0][SLOT_W:0]                hdr_n_cells;
  logic [1:0][2:0][PT_W-1:0]          hdr_glo, hdr_ghi;
  logic                                unused_hdr_cells;

  cse_top #(
      .N_CU(N_CU),
      .INPUT_FIFO_DEPTH(INPUT_FIFO_DEPTH),
      .RESPONSE_FIFO_DEPTH(RESPONSE_FIFO_DEPTH),
      .KILL_ENABLE(KILL_ENABLE),
      .CU_VOXEL_FIFO_DEPTH(CU_VOXEL_FIFO_DEPTH),
      .CU_CELL_FIFO_DEPTH(CU_CELL_FIFO_DEPTH),
      .CU_MAX_PTS_OUTSTANDING(CU_MAX_PTS_OUTSTANDING)
  ) u_cse (
      .clk, .rst_n,
      .q_valid, .q_ready, .q_tag, .q_user, .q_kill_on_hit, .q_env_buf,
      .q_cx, .q_cy, .q_cz, .q_r,
      .r_valid, .r_ready, .r_tag, .r_user, .r_status,
      .k_valid, .k_ready, .k_all, .k_tag,
      .cse_idle, .cse_err,
      .rd_buf, .dir_addr, .dir_data, .cell_addr, .cell_data,
      .pt_addr, .pt_data, .hdr_valid, .hdr_err, .hdr_n_pts, .hdr_glo, .hdr_ghi
  );

  cse_env_mem #(.NRD(N_CU)) u_env_mem (
      .clk,
      .rd_buf,
      .dir_addr, .dir_data,
      .cell_addr, .cell_data,
      .pt_addr, .pt_data,
      .hdr_valid, .hdr_err, .hdr_n_pts, .hdr_n_cells, .hdr_glo, .hdr_ghi,
      .wr_en(1'b0), .wr_buf(1'b0), .wr_region(ENV_HDR), .wr_addr('0), .wr_data('0)
  );

  assign unused_hdr_cells = ^hdr_n_cells;

endmodule
