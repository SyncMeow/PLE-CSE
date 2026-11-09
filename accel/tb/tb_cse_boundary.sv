// Exact d^2 == r^2 test; also serves as the comparator-mutant detector.
module tb_cse_boundary;
  import ple_cse_pkg::*;
  import cse_cu_pkg::*;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  logic q_valid, q_ready;
  query_t q_payload;
  logic r_valid, r_ready;
  resp_t r_payload;
  logic k_valid, k_ready;
  kill_t k_payload;
  logic cse_idle, cse_err;
  logic mem_rd_buf;
  logic [DIR_AW-1:0] mem_dir_addr;
  logic [15:0] mem_dir_data;
  logic [SLOT_W-1:0] mem_cell_addr;
  logic [127:0] mem_cell_data;
  logic [PT_AW-1:0] mem_pt_addr;
  logic [PT_DATA_W-1:0] mem_pt_data;
  logic [1:0] hdr_valid, hdr_err;
  logic [1:0][PT_AW-1:0] hdr_n_pts;
  logic [1:0][2:0][PT_W-1:0] hdr_glo, hdr_ghi;
  cell_t one_cell;

  cse_cu dut (.*);

  always_comb begin
    one_cell = '0;
    one_cell.count = COUNT_W'(1);
    one_cell.lo_x = AOFF_W'(1000);
    one_cell.lo_y = AOFF_W'(1000);
    one_cell.lo_z = AOFF_W'(1000);
    one_cell.hi_x = AOFF_W'(1000);
    one_cell.hi_y = AOFF_W'(1000);
    one_cell.hi_z = AOFF_W'(1000);
  end

  always_ff @(posedge clk) begin
    mem_dir_data  <= 16'h8000;
    mem_cell_data <= one_cell;
    mem_pt_data   <= {16'd1000, 16'd1000, 16'd1000};
  end

  initial begin
    q_valid = 1'b0;
    q_payload = '0;
    r_ready = 1'b1;
    k_valid = 1'b0;
    k_payload = '0;
    hdr_valid = 2'b01;
    hdr_err = '0;
    hdr_n_pts = '0;
    hdr_n_pts[0] = 1;
    hdr_glo = '0;
    hdr_ghi = '0;
    for (int axis = 0; axis < 3; ++axis) begin
      hdr_glo[0][axis] = 16'd1000;
      hdr_ghi[0][axis] = 16'd1000;
    end

    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    @(negedge clk);
    q_payload.tag = 4'd9;
    q_payload.user = 16'h3456;
    q_payload.cx = 18'sd1003;
    q_payload.cy = 18'sd1004;
    q_payload.cz = 18'sd1000;
    q_payload.r = 14'd5;
    q_valid = 1'b1;
    do @(posedge clk); while (!q_ready);
    @(negedge clk);
    q_valid = 1'b0;
    while (!r_valid) @(posedge clk);
    if (r_payload.tag != 9 || r_payload.user != 16'h3456 || r_payload.status != ST_HIT)
      $fatal(1, "d^2 == r^2 must be HIT, got status=%0d", r_payload.status);
    $display("TB_CSE_BOUNDARY d2=25 r2=25 status=HIT");
    $finish;
  end

  logic unused;
  assign unused = ^{k_ready, cse_idle, cse_err, mem_rd_buf, mem_dir_addr, mem_cell_addr, mem_pt_addr};

endmodule
