// cse_env_mem check: load two traces (+env0, +env1), read every word back through different ports and
// dump it; the Makefile diffs the dumps against the original hex files.
module tb_env_mem #(parameter int N_CELL = ple_cse_pkg::CELL_DEPTH, parameter int N_PT = ple_cse_pkg::PT_DEPTH,
                    parameter int N_BUF = 2);
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

  cse_env_mem #(.NRD(NRD), .N_BUF(N_BUF), .N_CELL(N_CELL), .N_PT(N_PT)) u_mem (
      .clk, .rd_buf, .dir_addr, .dir_data, .cell_addr, .cell_data, .pt_addr, .pt_data,
      .hdr_valid, .hdr_err, .hdr_n_pts, .hdr_n_cells, .hdr_glo, .hdr_ghi,
      .wr_en, .wr_buf, .wr_region, .wr_addr, .wr_data);

  logic              wr_en = 1'b0, wr_buf = 1'b0;
  env_region_e       wr_region = ENV_HDR;
  logic [PT_AW-1:0]  wr_addr = '0;
  logic [127:0]      wr_data = '0;

  // writes to a buffer that does not exist must be dropped: all-ones into every region of buffer 1
  task automatic write_absent();
    static int addrs[4] = '{0, 1, 5, 100};
    for (int r = 0; r < 4; ++r)
      for (int a = 0; a < 4; ++a) begin
        @(negedge clk);
        wr_en = 1'b1; wr_buf = 1'b1; wr_region = env_region_e'(r); wr_addr = PT_AW'(addrs[a]); wr_data = '1;
      end
    @(negedge clk);
    wr_en = 1'b0; wr_buf = 1'b0;
  endtask

  string out;
  int bw;
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

  // a buffer that does not exist must read invalid and 0 at every address (probe pattern: all, low, high bits)
  task automatic absent(input int b);
    int bad = 0;
    for (int p = 0; p < NRD; ++p) rd_buf[p] = b[0];
    if (hdr_valid[b] || hdr_err[b] || hdr_n_pts[b] != 0 || hdr_n_cells[b] != 0) bad++;
    for (int i = 0; i < ENV_NCELL; ++i) begin
      for (int p = 0; p < NRD; ++p) begin
        dir_addr[p] = DIR_AW'(i); cell_addr[p] = SLOT_W'(i); pt_addr[p] = PT_AW'(i);
      end
      @(posedge clk); #1;
      for (int p = 0; p < NRD; ++p) if (dir_data[p] != 0 || cell_data[p] != 0 || pt_data[p] != 0) bad++;
    end
    $display("TB buf%0d absent valid=%0d nonzero_reads=%0d", b, hdr_valid[b], bad);
    if (bad != 0) $fatal(1, "TB absent buffer is visible");
  endtask

  // reference copy of buffer b, loaded from the same files as the memory
  logic [31:0]          r_hdr [2][HDR_WORDS];
  logic [15:0]          r_dir [2][ENV_NCELL];
  logic [127:0]         r_cell[2][N_CELL];
  logic [PT_DATA_W-1:0] r_pt  [2][N_PT];
  task automatic load_ref(input int b);
    string pre;
    logic [31:0] h[HDR_WORDS];
    logic [15:0] d[ENV_NCELL];
    logic [127:0] c[N_CELL];
    logic [PT_W-1:0] tx[N_PT], ty[N_PT], tz[N_PT];
    if (!$value$plusargs(b == 0 ? "env0=%s" : "env1=%s", pre)) $fatal(1, "TB missing +env%0d", b);
    $readmemh({pre, "_hdr.hex"}, h);
    $readmemh({pre, "_dir.hex"}, d);
    $readmemh({pre, "_cell.hex"}, c, 0, int'(h[2]) - 1);
    $readmemh({pre, "_ptx.hex"}, tx, 0, int'(h[1]) - 1);
    $readmemh({pre, "_pty.hex"}, ty, 0, int'(h[1]) - 1);
    $readmemh({pre, "_ptz.hex"}, tz, 0, int'(h[1]) - 1);
    for (int i = 0; i < HDR_WORDS; ++i) r_hdr[b][i] = h[i];
    for (int i = 0; i < ENV_NCELL; ++i) r_dir[b][i] = d[i];
    for (int i = 0; i < int'(h[2]); ++i) r_cell[b][i] = c[i];
    for (int i = 0; i < int'(h[1]); ++i) r_pt[b][i] = {tz[i], ty[i], tx[i]};
  endtask

  // every port reads all three tables in the same cycle, each port at a different address
  task automatic ports(input int b, input string tag);
    int bad = 0, n = 0, nc = int'(r_hdr[b][2]), np = int'(r_hdr[b][1]);
    int da[NRD], ca[NRD], pa[NRD];
    for (int p = 0; p < NRD; ++p) rd_buf[p] = b[0];
    for (int i = 0; i < ENV_NCELL; ++i) begin
      for (int p = 0; p < NRD; ++p) begin
        da[p] = (i + p * 8191) % ENV_NCELL; ca[p] = (i + p * 37) % nc; pa[p] = (i * 3 + p * 1021) % np;
        dir_addr[p] = DIR_AW'(da[p]); cell_addr[p] = SLOT_W'(ca[p]); pt_addr[p] = PT_AW'(pa[p]);
      end
      @(posedge clk); #1;
      for (int p = 0; p < NRD; ++p) begin
        n += 3;
        if (dir_data[p] !== r_dir[b][da[p]]) bad++;
        if (cell_data[p] !== r_cell[b][ca[p]]) bad++;
        if (pt_data[p] !== r_pt[b][pa[p]]) bad++;
      end
    end
    $display("TB buf%0d ports %s checked=%0d port_errors=%0d", b, tag, n, bad);
    if (bad != 0) $fatal(1, "TB port read mismatch");
  endtask

  task automatic wr(input int b, input env_region_e rg, input int a, input logic [127:0] d);
    @(negedge clk);
    wr_en = 1'b1; wr_buf = b[0]; wr_region = rg; wr_addr = PT_AW'(a); wr_data = d;
  endtask

  // while a write is on the port: every port reads the other tables, odd ports also read the written table
  int side_n = 0, side_bad = 0;
  task automatic side(input int b, input env_region_e rg, input int i);
    int nc = int'(r_hdr[b][2]), np = int'(r_hdr[b][1]);
    int da[NRD], ca[NRD], pa[NRD];
    for (int p = 0; p < NRD; ++p) begin                      // never the address written in this cycle
      da[p] = ((i * 163) % ENV_NCELL + 1 + p * 11) % ENV_NCELL; ca[p] = ((i * 5) % nc + 1 + p) % nc;
      pa[p] = ((i * 61) % np + 1 + p) % np;
      rd_buf[p] = b[0]; dir_addr[p] = DIR_AW'(da[p]); cell_addr[p] = SLOT_W'(ca[p]); pt_addr[p] = PT_AW'(pa[p]);
    end
    @(posedge clk); #1;
    for (int p = 0; p < NRD; ++p) begin
      if (rg != ENV_DIR || p % 2 == 1) begin side_n++; if (dir_data[p] !== r_dir[b][da[p]]) side_bad++; end
      if (rg != ENV_CELL || p % 2 == 1) begin side_n++; if (cell_data[p] !== r_cell[b][ca[p]]) side_bad++; end
      if (rg != ENV_PTS || p % 2 == 1) begin side_n++; if (pt_data[p] !== r_pt[b][pa[p]]) side_bad++; end
    end
  endtask

  // write new words through the write port (header invalid meanwhile); every port must see them
  task automatic rewrite(input int b);
    int nc = int'(r_hdr[b][2]), np = int'(r_hdr[b][1]);
    side_n = 0; side_bad = 0;
    wr(b, ENV_HDR, 0, '0);
    for (int i = 0; i < 200; ++i) begin
      int da = (i * 163) % ENV_NCELL, ca = (i * 5) % nc, pa = (i * 61) % np;
      wr(b, ENV_DIR, da, 128'(16'(i * 257 + 1)));
      side(b, ENV_DIR, i);
      r_dir[b][da] = 16'(i * 257 + 1);
      wr(b, ENV_CELL, ca, {4{32'(i) ^ 32'hA5C3_0F96}});
      side(b, ENV_CELL, i);
      r_cell[b][ca] = {4{32'(i) ^ 32'hA5C3_0F96}};
      wr(b, ENV_PTS, pa, 128'({3{16'(i + 7)}}));
      side(b, ENV_PTS, i);
      r_pt[b][pa] = PT_DATA_W'({3{16'(i + 7)}});
    end
    wr(b, ENV_HDR, 0, 128'(r_hdr[b][0]));
    @(negedge clk);
    wr_en = 1'b0; wr_buf = 1'b0;
    $display("TB buf%0d reads during writes checked=%0d port_errors=%0d", b, side_n, side_bad);
    if (side_bad != 0) $fatal(1, "TB read during write mismatch");
  endtask

  initial begin
    if (!$value$plusargs("out=%s", out)) out = ".";
    rd_buf = '0; dir_addr = '0; cell_addr = '0; pt_addr = '0;
    @(posedge clk);
    if ($value$plusargs("bad_write=%d", bw)) begin            // table write while valid (bw = region + 4 * buffer)
      wr(bw / 4, env_region_e'(bw % 4), 3, 128'd1);
      @(negedge clk); wr_en = 1'b0;
      @(negedge clk);
      $display("TB bad_write not caught");
    end else begin
      if (N_BUF == 1) write_absent();
      dump(0);
      if (N_BUF == 2) dump(1);
      else absent(1);
      for (int b = 0; b < N_BUF; ++b) begin
        load_ref(b);
        ports(b, "loaded");
        rewrite(b);
        ports(b, "rewritten");
      end
    end
    $finish;
  end

endmodule
