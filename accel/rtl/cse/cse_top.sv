// Flat PLE-facing wrapper around one CSE CU and one double-buffered environment memory.
module cse_top
  import ple_cse_pkg::*;
#(
    parameter int unsigned VOXEL_FIFO_DEPTH = 2,
    parameter int unsigned CELL_FIFO_DEPTH = 2,
    parameter int unsigned MAX_PTS_OUTSTANDING = 8,
    parameter bit KILL_ENABLE = 1'b1
)
(
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

  query_t q_payload;
  resp_t  r_payload;
  kill_t  k_payload;

  logic [0:0]                    rd_buf;
  logic [0:0][DIR_AW-1:0]        dir_addr;
  logic [0:0][15:0]              dir_data;
  logic [0:0][SLOT_W-1:0]        cell_addr;
  logic [0:0][127:0]             cell_data;
  logic [0:0][PT_AW-1:0]         pt_addr;
  logic [0:0][PT_DATA_W-1:0]     pt_data;
  logic [1:0]                    hdr_valid, hdr_err;
  logic [1:0][PT_AW-1:0]         hdr_n_pts;
  logic [1:0][SLOT_W:0]          hdr_n_cells;
  logic [1:0][2:0][PT_W-1:0]    hdr_glo, hdr_ghi;
  logic                          unused_hdr_cells;

  assign q_payload = '{tag:q_tag, user:q_user, kill_on_hit:q_kill_on_hit,
                       env_buf:q_env_buf, cx:q_cx, cy:q_cy, cz:q_cz, r:q_r};
  assign k_payload = '{all:k_all, tag:k_tag};
  assign r_tag      = r_payload.tag;
  assign r_user     = r_payload.user;
  assign r_status   = r_payload.status;

  cse_cu #(
      .VOXEL_FIFO_DEPTH(VOXEL_FIFO_DEPTH),
      .CELL_FIFO_DEPTH(CELL_FIFO_DEPTH),
      .MAX_PTS_OUTSTANDING(MAX_PTS_OUTSTANDING),
      .KILL_ENABLE(KILL_ENABLE)
  ) u_cu (
      .clk, .rst_n,
      .q_valid, .q_ready, .q_payload,
      .r_valid, .r_ready, .r_payload,
      .k_valid, .k_ready, .k_payload,
      .cse_idle, .cse_err,
      .mem_rd_buf(rd_buf[0]),
      .mem_dir_addr(dir_addr[0]), .mem_dir_data(dir_data[0]),
      .mem_cell_addr(cell_addr[0]), .mem_cell_data(cell_data[0]),
      .mem_pt_addr(pt_addr[0]), .mem_pt_data(pt_data[0]),
      .hdr_valid, .hdr_err, .hdr_n_pts, .hdr_glo, .hdr_ghi
  );

  cse_env_mem #(.NRD(1)) u_env_mem (
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
