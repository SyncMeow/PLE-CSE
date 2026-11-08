// cse_env_mem check: load two traces (+env0, +env1), read every word back through different ports and
// dump it; the Makefile diffs the dumps against the original hex files.
module tb_env_mem;
  import ple_cse_pkg::*;

  localparam int NRD = 4;
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic [NRD-1:0] rd_buf;
  logic [NRD-1:0][DIR_AW-1:0] dir_addr;
  logic [NRD-1:0][15:0] dir_data;
  logic [NRD-1:0][SLOT_W-1:0] cell_addr;
  logic [NRD-1:0][127:0] cell_data;
  logic [NRD-1:0][PT_AW-1:0] pt_addr;
  logic [NRD-1:0][PT_DATA_W-1:0] pt_data;
  logic [1:0] hdr_valid, hdr_err;
  logic [1:0][PT_AW-1:0] hdr_n_pts;
  logic [1:0][SLOT_W:0] hdr_n_cells;
  logic [1:0][2:0][PT_W-1:0] hdr_glo, hdr_ghi;

  cse_env_mem #(.NRD(NRD)) u_mem (
      .clk, .rd_buf, .dir_addr, .dir_data, .cell_addr, .cell_data, .pt_addr, .pt_data,
      .hdr_valid, .hdr_err, .hdr_n_pts, .hdr_n_cells, .hdr_glo, .hdr_ghi,
      .wr_en(1'b0), .wr_buf(1'b0), .wr_region(ENV_HDR), .wr_addr('0), .wr_data('0));

  string out;
  int fd, fc, fx, fy, fz;

  // buffer b is read through port p = 2b (DIR, CELL) and 2b+1 (PTS); one address per cycle, latency 1
  task automatic dump(input int b);
    int p = 2 * b, pp = 2 * b + 1;
    fd = $fopen($sformatf("%s/buf%0d_dir.hex", out, b), "w");
    fc = $fopen($sformatf("%s/buf%0d_cell.hex", out, b), "w");
    fx = $fopen($sformatf("%s/buf%0d_ptx.hex", out, b), "w");
    fy = $fopen($sformatf("%s/buf%0d_pty.hex", out, b), "w");
    fz = $fopen($sformatf("%s/buf%0d_ptz.hex", out, b), "w");
    rd_buf[p] = b[0];
    rd_buf[pp] = b[0];
    for (int i = 0; i < ENV_NCELL; ++i) begin
      dir_addr[p] = DIR_AW'(i);
      @(posedge clk); #1;
      $fwrite(fd, "%04x\n", dir_data[p]);
    end
    for (int i = 0; i < int'(hdr_n_cells[b]); ++i) begin
      cell_addr[p] = SLOT_W'(i);
      @(posedge clk); #1;
      $fwrite(fc, "%032x\n", cell_data[p]);
    end
    for (int i = 0; i < int'(hdr_n_pts[b]); ++i) begin
      pt_addr[pp] = PT_AW'(i);
      @(posedge clk); #1;
      $fwrite(fx, "%04x\n", pt_data[pp][15:0]);
      $fwrite(fy, "%04x\n", pt_data[pp][31:16]);
      $fwrite(fz, "%04x\n", pt_data[pp][47:32]);
    end
    $fclose(fd); $fclose(fc); $fclose(fx); $fclose(fy); $fclose(fz);
    $display("TB buf%0d valid=%0d err=%0d n_pts=%0d n_cells=%0d glo=%0d,%0d,%0d ghi=%0d,%0d,%0d", b, hdr_valid[b],
             hdr_err[b], hdr_n_pts[b], hdr_n_cells[b], hdr_glo[b][0], hdr_glo[b][1], hdr_glo[b][2], hdr_ghi[b][0],
             hdr_ghi[b][1], hdr_ghi[b][2]);
  endtask

  initial begin
    if (!$value$plusargs("out=%s", out)) out = ".";
    rd_buf = '0; dir_addr = '0; cell_addr = '0; pt_addr = '0;
    @(posedge clk);
    dump(0);
    dump(1);
    $finish;
  end

endmodule
