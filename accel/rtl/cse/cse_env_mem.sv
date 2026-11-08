// Environment memory, format v0, double buffered (INTERFACES.md I3).
// Reference model: plain arrays read by NRD port sets; the CSE owner may replace the inside with banked
// or replicated BRAM as long as the ports and ENV_RD_LAT stay the same.
// Simulation load: +env0=<trace prefix> / +env1=<prefix> reads <prefix>_{hdr,dir,cell,ptx,pty,ptz}.hex.
module cse_env_mem
  import ple_cse_pkg::*;
#(
    parameter int NRD = 4                                     // read-port sets (one per CU)
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

  localparam int CA_W = $clog2(CELL_DEPTH);
  localparam int PA_W = $clog2(PT_DEPTH);
  localparam int HA_W = $clog2(HDR_WORDS);

  logic [1:0][NRD-1:0][15:0]          dir_q;
  logic [1:0][NRD-1:0][127:0]         cell_q;
  logic [1:0][NRD-1:0][PT_DATA_W-1:0] pt_q;

  for (genvar b = 0; b < 2; ++b) begin : g_buf
    logic [31:0]          hdr  [HDR_WORDS];
    logic [15:0]          dirm [ENV_NCELL];
    logic [127:0]         cellm[CELL_DEPTH];
    logic [PT_DATA_W-1:0] ptm  [PT_DEPTH];

    // reads (both buffers are read; the port's rd_buf picks one, registered with the data)
    for (genvar p = 0; p < NRD; ++p) begin : g_rd
      always_ff @(posedge clk) begin
        dir_q[b][p]  <= (int'(dir_addr[p]) < ENV_NCELL) ? dirm[dir_addr[p]] : '0;
        cell_q[b][p] <= (int'(cell_addr[p]) < CELL_DEPTH) ? cellm[cell_addr[p][CA_W-1:0]] : '0;
        pt_q[b][p]   <= (int'(pt_addr[p]) < PT_DEPTH) ? ptm[pt_addr[p][PA_W-1:0]] : '0;
      end
    end

    assign hdr_valid[b]   = hdr[0][0];
    assign hdr_err[b]     = hdr[0][1];
    assign hdr_n_pts[b]   = hdr[1][PT_AW-1:0];
    assign hdr_n_cells[b] = hdr[2][SLOT_W:0];
    for (genvar k = 0; k < 3; ++k) begin : g_axis
      assign hdr_glo[b][k] = hdr[3 + k][PT_W-1:0];
      assign hdr_ghi[b][k] = hdr[6 + k][PT_W-1:0];
    end

    // build-engine writes
    always_ff @(posedge clk) begin
      if (wr_en && wr_buf == b[0]) begin
        unique case (wr_region)
          ENV_HDR:  if (int'(wr_addr) < HDR_WORDS) hdr[wr_addr[HA_W-1:0]] <= wr_data[31:0];
          ENV_DIR:  if (int'(wr_addr) < ENV_NCELL) dirm[wr_addr[DIR_AW-1:0]] <= wr_data[15:0];
          ENV_CELL: if (int'(wr_addr) < CELL_DEPTH) cellm[wr_addr[CA_W-1:0]] <= wr_data;
          ENV_PTS:  if (int'(wr_addr) < PT_DEPTH) ptm[wr_addr[PA_W-1:0]] <= wr_data[PT_DATA_W-1:0];
        endcase
      end
    end

`ifndef SYNTHESIS
    logic [PT_W-1:0] tx[PT_DEPTH], ty[PT_DEPTH], tz[PT_DEPTH];
    initial begin
      string pre;
      for (int i = 0; i < HDR_WORDS; ++i) hdr[i] = '0;
      for (int i = 0; i < ENV_NCELL; ++i) dirm[i] = '0;
      if ($value$plusargs(b == 0 ? "env0=%s" : "env1=%s", pre)) begin
        $readmemh({pre, "_hdr.hex"}, hdr);
        $readmemh({pre, "_dir.hex"}, dirm);
        if (hdr[2] != 0) $readmemh({pre, "_cell.hex"}, cellm, 0, int'(hdr[2]) - 1);
        if (hdr[1] != 0) begin
          $readmemh({pre, "_ptx.hex"}, tx, 0, int'(hdr[1]) - 1);
          $readmemh({pre, "_pty.hex"}, ty, 0, int'(hdr[1]) - 1);
          $readmemh({pre, "_ptz.hex"}, tz, 0, int'(hdr[1]) - 1);
          for (int i = 0; i < int'(hdr[1]); ++i) ptm[i] = {tz[i], ty[i], tx[i]};
        end
        $display("[cse_env_mem] buf%0d <- %s: %0d points, %0d cells", b, pre, hdr[1], hdr[2]);
      end
    end
`endif
  end

  // output select: rd_buf registered so the mux matches the data's cycle
  logic [NRD-1:0] buf_q;
  always_ff @(posedge clk) buf_q <= rd_buf;
  for (genvar p = 0; p < NRD; ++p) begin : g_out
    assign dir_data[p]  = dir_q[buf_q[p]][p];
    assign cell_data[p] = cell_q[buf_q[p]][p];
    assign pt_data[p]   = pt_q[buf_q[p]][p];
  end

endmodule
