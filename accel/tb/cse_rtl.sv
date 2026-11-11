// Real CSE (cse_top + cse_env_mem) behind the cse_bfm port list, for the PLE testbenches (+define+RTL_CSE).
// load_env() copies the golden environment (fx_ref_env_load) into env buffer 0 through the memory's write port.
// err_at >= 0: once that many queries were accepted, HDR word 0 gets err = 1 (later queries answer ERROR);
// err_at back to -1 restores it. The cse_bfm performance knobs exist only so the testbenches still compile.
module cse_rtl
  import ple_cse_pkg::*;
#(
    parameter bit AK     = 1'b0,                         // must be 0: the RTL CSE has no auto-kill
`ifdef CSE_N_CU
    parameter int N_CU   = `CSE_N_CU
`else
    parameter int N_CU   = 2
`endif
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

  import "DPI-C" function int fx_ref_env_word(input int region, input int addr, input int w);

  if (AK) begin : g_bad_ak
    $error("cse_rtl: the RTL CSE has no auto-kill, build with AK = 0");
  end

  // same names as cse_bfm (statistics, error injection, unused performance knobs)
  int nq = 0, nkilled = 0, nhit = 0, err_at = -1;
  int mode = 0, lat = 0, ii = 0, nu = 0, fd = 0, lanes = 0;
  longint unsigned work_cycles = 0, busy_unit_cycles = 0, fifo_full_cycles = 0;

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

  logic         wr_en = 1'b0;
  env_region_e  wr_region = ENV_HDR;
  logic [PT_AW-1:0] wr_addr = '0;
  logic [127:0] wr_data = '0;

  cse_top #(.N_CU(N_CU)) u_top (
      .clk, .rst_n,
      .q_valid, .q_ready, .q_tag, .q_user, .q_kill_on_hit, .q_env_buf, .q_cx, .q_cy, .q_cz, .q_r,
      .r_valid, .r_ready, .r_tag, .r_user, .r_status,
      .k_valid, .k_ready, .k_all, .k_tag, .cse_idle, .cse_err,
      .rd_buf, .dir_addr, .dir_data, .cell_addr, .cell_data, .pt_addr, .pt_data,
      .hdr_valid, .hdr_err, .hdr_n_pts, .hdr_glo, .hdr_ghi);

  cse_env_mem #(.NRD(N_CU)) u_mem (
      .clk, .rd_buf, .dir_addr, .dir_data, .cell_addr, .cell_data, .pt_addr, .pt_data,
      .hdr_valid, .hdr_err, .hdr_n_pts, .hdr_n_cells, .hdr_glo, .hdr_ghi,
      .wr_en, .wr_buf(1'b0), .wr_region, .wr_addr, .wr_data);

  always @(posedge clk) if (rst_n) begin
    if (q_valid && q_ready) nq <= nq + 1;
    if (r_valid && r_ready && r_status == 2'd2) nkilled <= nkilled + 1;
    if (r_valid && r_ready && r_status == 2'd1) nhit <= nhit + 1;
    if (!cse_idle) work_cycles <= work_cycles + 1;
    if (q_valid && !q_ready) fifo_full_cycles <= fifo_full_cycles + 1;
  end

  // all writes come from this one process: environment loads and the ERROR injection
  bit load_req = 0, err_on = 0;
  int hdr0 = 0;
  task automatic wr(input env_region_e rg, input int a, input logic [127:0] d);
    @(negedge clk);
    wr_en = 1'b1; wr_region = rg; wr_addr = PT_AW'(a); wr_data = d;
  endtask
  initial forever begin
    @(negedge clk);
    wr_en = 1'b0;
    if (load_req) begin
      int np, nc;
      np = fx_ref_env_word(0, 1, 0);
      nc = fx_ref_env_word(0, 2, 0);
      if (np > PT_DEPTH || nc > CELL_DEPTH) $fatal(1, "cse_rtl: environment %0d points / %0d cells exceeds the memory", np, nc);
      wr(ENV_HDR, 0, '0);                                  // invalid while the tables change
      for (int i = 0; i < ENV_NCELL; ++i) wr(ENV_DIR, i, 128'(fx_ref_env_word(1, i, 0)));
      for (int i = 0; i < nc; ++i)
        wr(ENV_CELL, i, {fx_ref_env_word(2, i, 3), fx_ref_env_word(2, i, 2), fx_ref_env_word(2, i, 1), fx_ref_env_word(2, i, 0)});
      for (int i = 0; i < np; ++i) wr(ENV_PTS, i, {80'd0, 16'(fx_ref_env_word(3, i, 1)), 32'(fx_ref_env_word(3, i, 0))});
      for (int i = 1; i < HDR_WORDS; ++i) wr(ENV_HDR, i, 128'(fx_ref_env_word(0, i, 0)));
      hdr0 = fx_ref_env_word(0, 0, 0);
      wr(ENV_HDR, 0, 128'(hdr0));
      @(negedge clk) wr_en = 1'b0;
      load_req = 0;
    end else if (!err_on && err_at >= 0 && nq >= err_at) begin
      wr(ENV_HDR, 0, 128'(hdr0 | 2));
      err_on = 1;
    end else if (err_on && err_at < 0) begin
      wr(ENV_HDR, 0, 128'(hdr0));
      err_on = 0;
    end
  end

  // called by the testbench after fx_ref_env_load(), while the PLE is idle
  task automatic load_env();
    load_req = 1;
    wait (load_req == 0);
  endtask

endmodule
