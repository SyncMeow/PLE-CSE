// Legal dense 9x9x9 R_MAX workload used to measure the single-CU response bound.
module tb_cse_worst;
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
  cell_t dense_cell;
  longint unsigned cycles, accepted_at, latency;

  cse_cu dut (.*);

  always_comb begin
    dense_cell = '0;
    dense_cell.base = '0;
    dense_cell.count = COUNT_W'(64);
    dense_cell.lo_x = '0;
    dense_cell.lo_y = '0;
    dense_cell.lo_z = '0;
    dense_cell.hi_x = {AOFF_W{1'b1}};
    dense_cell.hi_y = {AOFF_W{1'b1}};
    dense_cell.hi_z = {AOFF_W{1'b1}};
  end

  // Same one-cycle read timing as cse_env_mem; every requested voxel is a 64-point cell.
  always_ff @(posedge clk) begin
    mem_dir_data  <= 16'h8000;
    mem_cell_data <= dense_cell;
    mem_pt_data   <= {16'hffff, 16'hffff, 16'hffff};
  end

  always_ff @(posedge clk) begin
    if (!rst_n) cycles <= 0;
    else cycles <= cycles + 1;
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
    hdr_n_pts[0] = PT_AW'(46656);
    hdr_glo = '0;
    hdr_ghi = '0;
    hdr_ghi[0][0] = 16'd40959;
    hdr_ghi[0][1] = 16'd40959;
    hdr_ghi[0][2] = 16'd40959;

    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    @(negedge clk);
    q_payload.tag = 4'd1;
    q_payload.user = 16'hbeef;
    q_payload.env_buf = 1'b0;
    q_payload.cx = 18'sd20480;
    q_payload.cy = 18'sd20480;
    q_payload.cz = 18'sd20480;
    q_payload.r = R_MAX;
    q_valid = 1'b1;
    do @(posedge clk); while (!q_ready);
    accepted_at = cycles;
    @(negedge clk);
    q_valid = 1'b0;

    while (!r_valid) begin
      @(posedge clk);
      if (cycles - accepted_at > 64'(MAX_LAT)) $fatal(1, "CSE exceeded MAX_LAT before response");
    end
    latency = cycles - accepted_at;
    if (r_payload.tag != 1 || r_payload.user != 16'hbeef || r_payload.status != ST_MISS)
      $fatal(1, "wrong dense-workload response");
    @(posedge clk);
    if (!cse_idle) @(posedge clk);
    $display("TB_CSE_WORST voxels=729 points=46656 latency=%0d limit=%0d", latency, MAX_LAT);
    if (latency >= 64'(MAX_LAT) || cse_err) $fatal(1, "dense-workload latency test failed");
    $finish;
  end

  logic unused_memory_controls;
  assign unused_memory_controls = ^{k_ready, mem_rd_buf, mem_dir_addr, mem_cell_addr, mem_pt_addr};

endmodule
