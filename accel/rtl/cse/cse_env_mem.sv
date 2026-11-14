// Environment memory, format v0, N_BUF buffers (INTERFACES.md I3). The ports always carry two buffers;
// with N_BUF = 1 buffer 1 does not exist: its header reads invalid, its data reads 0, writes to it are dropped.
// Each table is stored in ceil(NRD/2) true dual-port copies: two read-port sets share one copy.
// A table write uses port A of every copy for that cycle, so read ports 0, 2, .. of that table return
// undefined data then: clear the header's valid bit before the tables of a buffer are written.
// Simulation load: +env0=<trace prefix> / +env1=<prefix> reads <prefix>_{hdr,dir,cell,ptx,pty,ptz}.hex.
module cse_env_mem
  import ple_cse_pkg::*;
#(
    parameter int NRD        = 4,                             // read-port sets (one per CU)
    parameter int N_BUF      = 1,                             // buffers: 1 (single) or 2 (double buffered)
    parameter int N_CELL     = ple_cse_pkg::CELL_DEPTH,       // CELL entries per buffer (format allows 4096)
    parameter int N_PT       = ple_cse_pkg::PT_DEPTH          // points per buffer (format allows 65536)
) (
    input  logic                          clk,
    // read ports: address at edge t, data after edge t+1 (ENV_RD_LAT = 1); out of range reads 0
    input  logic [NRD-1:0]                rd_buf,             // which buffer each port reads
    input  logic [NRD-1:0][DIR_AW-1:0]    dir_addr,
    output logic [NRD-1:0][15:0]          dir_data,
    input  logic [NRD-1:0][SLOT_W-1:0]    cell_addr,
    output logic [NRD-1:0][127:0]         cell_data,
    input  logic [NRD-1:0][PT_AW-1:0]     pt_addr,
    output logic [NRD-1:0][PT_DATA_W-1:0] pt_data,            // {z, y, x}
    // header of both buffers (registers, no read latency)
    output logic [1:0]                    hdr_valid,
    output logic [1:0]                    hdr_err,
    output logic [1:0][PT_AW-1:0]         hdr_n_pts,
    output logic [1:0][SLOT_W:0]          hdr_n_cells,
    output logic [1:0][2:0][PT_W-1:0]     hdr_glo,            // [buf][axis]
    output logic [1:0][2:0][PT_W-1:0]     hdr_ghi,
    // build-engine write port (unused in v1 simulation: tie wr_en low)
    input  logic                          wr_en,
    input  logic                          wr_buf,
    input  env_region_e                   wr_region,
    input  logic [PT_AW-1:0]              wr_addr,            // HDR: word index; PTS: point index
    input  logic [127:0]                  wr_data             // PTS: {z, y, x} in [47:0]
);

  localparam int CA_W = $clog2(N_CELL);
  localparam int PA_W = $clog2(N_PT);
  localparam int HA_W = $clog2(HDR_WORDS);

  if (N_BUF < 1 || N_BUF > 2) begin : g_bad_buf
    $error("cse_env_mem: N_BUF must be 1 or 2");
  end
  if (N_CELL < 1 || N_CELL > (1 << SLOT_W) || N_PT < 1 || N_PT > (1 << PT_AW)) begin : g_bad_depth
    $error("cse_env_mem: N_CELL must be 1..4096 and N_PT 1..65536 (format v0 address widths)");
  end

  localparam int NCP = (NRD + 1) / 2;                     // memory copies: one per two read ports

  logic [N_BUF-1:0][NRD-1:0][15:0]          dir_q;
  logic [N_BUF-1:0][NRD-1:0][127:0]         cell_q;
  logic [N_BUF-1:0][NRD-1:0][PT_DATA_W-1:0] pt_q;

  for (genvar b = 0; b < N_BUF; ++b) begin : g_buf
    logic [31:0] hdr[HDR_WORDS];

    assign hdr_valid[b]   = hdr[0][0];
    assign hdr_err[b]     = hdr[0][1];
    assign hdr_n_pts[b]   = hdr[1][PT_AW-1:0];
    assign hdr_n_cells[b] = hdr[2][SLOT_W:0];
    for (genvar k = 0; k < 3; ++k) begin : g_axis
      assign hdr_glo[b][k] = hdr[3 + k][PT_W-1:0];
      assign hdr_ghi[b][k] = hdr[6 + k][PT_W-1:0];
    end

    // build-engine writes: the header is registers, a table write goes to every copy
    logic wr_sel, we_dir, we_cell, we_pt;
    assign wr_sel  = wr_en && wr_buf == b[0];
    assign we_dir  = wr_sel && wr_region == ENV_DIR && int'(wr_addr) < ENV_NCELL;
    assign we_cell = wr_sel && wr_region == ENV_CELL && int'(wr_addr) < N_CELL;
    assign we_pt   = wr_sel && wr_region == ENV_PTS && int'(wr_addr) < N_PT;
    always_ff @(posedge clk)
      if (wr_sel && wr_region == ENV_HDR && int'(wr_addr) < HDR_WORDS) hdr[wr_addr[HA_W-1:0]] <= wr_data[31:0];

`ifndef SYNTHESIS
    // a table write takes port A of that table: the buffer must be invalid so that no CU reads it
    always_ff @(posedge clk)
      if (wr_sel && wr_region != ENV_HDR && hdr[0][0]) $error("cse_env_mem: buf%0d table written while valid", b);
`endif

    // each copy is a true dual-port RAM: port A = write or read port 2c, port B = read port 2c+1
    for (genvar c = 0; c < NCP; ++c) begin : g_cp
      localparam int PA = 2 * c;
      localparam int PB = 2 * c + 1;
      logic [15:0]          dirm [ENV_NCELL];
      logic [127:0]         cellm[N_CELL];
      logic [PT_DATA_W-1:0] ptm  [N_PT];

      logic [DIR_AW-1:0]    dir_aa;
      logic [CA_W-1:0]      cell_aa;
      logic [PA_W-1:0]      pt_aa;
      logic [15:0]          dir_ra;
      logic [127:0]         cell_ra;
      logic [PT_DATA_W-1:0] pt_ra;
      logic                 dir_oka, cell_oka, pt_oka;        // read address was inside the depth
      assign dir_aa  = we_dir ? wr_addr[DIR_AW-1:0] : dir_addr[PA];
      assign cell_aa = we_cell ? wr_addr[CA_W-1:0] : cell_addr[PA][CA_W-1:0];
      assign pt_aa   = we_pt ? wr_addr[PA_W-1:0] : pt_addr[PA][PA_W-1:0];
      always_ff @(posedge clk) begin
        if (we_dir) dirm[dir_aa] <= wr_data[15:0];
        dir_ra <= dirm[dir_aa];
      end
      always_ff @(posedge clk) begin
        if (we_cell) cellm[cell_aa] <= wr_data;
        cell_ra <= cellm[cell_aa];
      end
      always_ff @(posedge clk) begin
        if (we_pt) ptm[pt_aa] <= wr_data[PT_DATA_W-1:0];
        pt_ra <= ptm[pt_aa];
      end
      always_ff @(posedge clk) begin
        dir_oka  <= int'(dir_addr[PA]) < ENV_NCELL;
        cell_oka <= int'(cell_addr[PA]) < N_CELL;
        pt_oka   <= int'(pt_addr[PA]) < N_PT;
      end
      assign dir_q[b][PA]  = dir_oka ? dir_ra : '0;
      assign cell_q[b][PA] = cell_oka ? cell_ra : '0;
      assign pt_q[b][PA]   = pt_oka ? pt_ra : '0;

      if (PB < NRD) begin : g_pb
        logic [15:0]          dir_rb;
        logic [127:0]         cell_rb;
        logic [PT_DATA_W-1:0] pt_rb;
        logic                 dir_okb, cell_okb, pt_okb;
        always_ff @(posedge clk) dir_rb <= dirm[dir_addr[PB]];
        always_ff @(posedge clk) cell_rb <= cellm[cell_addr[PB][CA_W-1:0]];
        always_ff @(posedge clk) pt_rb <= ptm[pt_addr[PB][PA_W-1:0]];
        always_ff @(posedge clk) begin
          dir_okb  <= int'(dir_addr[PB]) < ENV_NCELL;
          cell_okb <= int'(cell_addr[PB]) < N_CELL;
          pt_okb   <= int'(pt_addr[PB]) < N_PT;
        end
        assign dir_q[b][PB]  = dir_okb ? dir_rb : '0;
        assign cell_q[b][PB] = cell_okb ? cell_rb : '0;
        assign pt_q[b][PB]   = pt_okb ? pt_rb : '0;
      end

`ifndef SYNTHESIS
      logic [31:0]     th[HDR_WORDS];
      logic [PT_W-1:0] tx[N_PT], ty[N_PT], tz[N_PT];
      initial begin
        string pre;
        for (int i = 0; i < HDR_WORDS; ++i) th[i] = '0;
        for (int i = 0; i < ENV_NCELL; ++i) dirm[i] = '0;
        if ($value$plusargs(b == 0 ? "env0=%s" : "env1=%s", pre)) begin
          $readmemh({pre, "_hdr.hex"}, th);
          $readmemh({pre, "_dir.hex"}, dirm);
          if (th[2] != 0) $readmemh({pre, "_cell.hex"}, cellm, 0, int'(th[2]) - 1);
          if (th[1] != 0) begin
            $readmemh({pre, "_ptx.hex"}, tx, 0, int'(th[1]) - 1);
            $readmemh({pre, "_pty.hex"}, ty, 0, int'(th[1]) - 1);
            $readmemh({pre, "_ptz.hex"}, tz, 0, int'(th[1]) - 1);
            for (int i = 0; i < int'(th[1]); ++i) ptm[i] = {tz[i], ty[i], tx[i]};
          end
        end
      end
`endif
    end

`ifndef SYNTHESIS
    initial begin
      string pre;
      for (int i = 0; i < HDR_WORDS; ++i) hdr[i] = '0;
      if ($value$plusargs(b == 0 ? "env0=%s" : "env1=%s", pre)) begin
        $readmemh({pre, "_hdr.hex"}, hdr);
        $display("[cse_env_mem] buf%0d <- %s: %0d points, %0d cells", b, pre, hdr[1], hdr[2]);
      end
    end
`endif
  end

  // a buffer that does not exist: header invalid
  for (genvar b = N_BUF; b < 2; ++b) begin : g_nobuf
    assign hdr_valid[b]   = 1'b0;
    assign hdr_err[b]     = 1'b0;
    assign hdr_n_pts[b]   = '0;
    assign hdr_n_cells[b] = '0;
    assign hdr_glo[b]     = '0;
    assign hdr_ghi[b]     = '0;
  end

  // output select: rd_buf registered so the mux matches the data's cycle
  logic [NRD-1:0] buf_q;
  always_ff @(posedge clk) buf_q <= rd_buf;
  for (genvar p = 0; p < NRD; ++p) begin : g_out
    if (N_BUF == 2) begin : g_two
      assign dir_data[p]  = dir_q[buf_q[p]][p];
      assign cell_data[p] = cell_q[buf_q[p]][p];
      assign pt_data[p]   = pt_q[buf_q[p]][p];
    end else begin : g_one                               // buffer 1 does not exist: reads 0
      assign dir_data[p]  = buf_q[p] ? '0 : dir_q[0][p];
      assign cell_data[p] = buf_q[p] ? '0 : cell_q[0][p];
      assign pt_data[p]   = buf_q[p] ? '0 : pt_q[0][p];
    end
  end

endmodule
