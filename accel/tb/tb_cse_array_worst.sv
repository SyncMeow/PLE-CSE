// Full-array legal PT_DEPTH stress: R_MAX, 729 voxels, 256 outside-sphere cells, 16384 points.
module tb_cse_array_worst;
  import ple_cse_pkg::*;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  logic q_valid, q_ready;
  logic [TAG_W-1:0] q_tag;
  logic [USER_W-1:0] q_user;
  logic q_kill_on_hit, q_env_buf;
  logic signed [COORD_W-1:0] q_cx, q_cy, q_cz;
  logic [RQ_W-1:0] q_r;
  logic r_valid, r_ready;
  logic [TAG_W-1:0] r_tag;
  logic [USER_W-1:0] r_user;
  logic [1:0] r_status;
  logic k_valid, k_ready, k_all;
  logic [TAG_W-1:0] k_tag;
  logic cse_idle, cse_err, eot;
  longint unsigned cycles, accepted_at, latency;

  cse_system #(.N_CU(4), .INPUT_FIFO_DEPTH(8), .RESPONSE_FIFO_DEPTH(8)) dut (.*);
  ple_cse_if_checker #(.AUTO_KILL(1'b0)) u_checker (.*);

  always_ff @(posedge clk) begin
    if (!rst_n) cycles <= 0;
    else cycles <= cycles + 1;
  end

  initial begin : p_test
    cell_t cell_word;
    int slot, base, index;
    int gx, gy, gz, org_x, org_y, org_z, px, py, pz;
    int off_x, off_y, off_z, dx, dy, dz;
    longint signed distance2;
    int glo_x, glo_y, glo_z, ghi_x, ghi_y, ghi_z;

    q_valid = 1'b0;
    q_tag = '0;
    q_user = '0;
    q_kill_on_hit = 1'b0;
    q_env_buf = 1'b0;
    q_cx = '0;
    q_cy = '0;
    q_cz = '0;
    q_r = '0;
    r_ready = 1'b1;
    k_valid = 1'b0;
    k_all = 1'b0;
    k_tag = '0;
    eot = 1'b0;

    // Let cse_env_mem's simulation initializer finish, then build a valid dense image in buffer zero.
    #1;
    for (int d = 0; d < ENV_NCELL; ++d) dut.u_env_mem.g_buf[0].dirm[d] = '0;
    slot = 0;
    base = 0;
    glo_x = 65535; glo_y = 65535; glo_z = 65535;
    ghi_x = 0; ghi_y = 0; ghi_z = 0;
    for (gz = 6; gz <= 14; ++gz)
      for (gy = 6; gy <= 14; ++gy)
        for (gx = 6; gx <= 14; ++gx) begin
          org_x = gx << ENV_QSHIFT;
          org_y = gy << ENV_QSHIFT;
          org_z = gz << ENV_QSHIFT;
          px = ((20480 - org_x) >= ((org_x + 2047) - 20480)) ? org_x : org_x + 2047;
          py = ((20480 - org_y) >= ((org_y + 2047) - 20480)) ? org_y : org_y + 2047;
          pz = ((20480 - org_z) >= ((org_z + 2047) - 20480)) ? org_z : org_z + 2047;
          if (px < 12288) px = 12288; if (px > 28672) px = 28672;
          if (py < 12288) py = 12288; if (py > 28672) py = 28672;
          if (pz < 12288) pz = 12288; if (pz > 28672) pz = 28672;
          dx = px - 20480; dy = py - 20480; dz = pz - 20480;
          distance2 = longint'(dx) * dx + longint'(dy) * dy + longint'(dz) * dz;
          if (distance2 > 64'd67108864 && slot < 256) begin
            off_x = px - org_x; off_y = py - org_y; off_z = pz - org_z;
            index = (gz * ENV_G + gy) * ENV_G + gx;
            dut.u_env_mem.g_buf[0].dirm[index] = 16'h8000 | 16'(slot);
            cell_word = '0;
            cell_word.base = PT_AW'(base);
            cell_word.count = COUNT_W'(64);
            cell_word.lo_x = AOFF_W'(off_x); cell_word.hi_x = AOFF_W'(off_x);
            cell_word.lo_y = AOFF_W'(off_y); cell_word.hi_y = AOFF_W'(off_y);
            cell_word.lo_z = AOFF_W'(off_z); cell_word.hi_z = AOFF_W'(off_z);
            dut.u_env_mem.g_buf[0].cellm[slot] = cell_word;
            for (int p = 0; p < 64; ++p)
              dut.u_env_mem.g_buf[0].ptm[base+p] = {16'(pz), 16'(py), 16'(px)};
            base += 64;
            slot++;
            if (px < glo_x) glo_x = px; if (px > ghi_x) ghi_x = px;
            if (py < glo_y) glo_y = py; if (py > ghi_y) ghi_y = py;
            if (pz < glo_z) glo_z = pz; if (pz > ghi_z) ghi_z = pz;
          end
        end
    if (slot != 256 || base != PT_DEPTH) $fatal(1, "failed to construct dense legal image");
    dut.u_env_mem.g_buf[0].hdr[0] = 32'd1;
    dut.u_env_mem.g_buf[0].hdr[1] = 32'(base);
    dut.u_env_mem.g_buf[0].hdr[2] = 32'(slot);
    dut.u_env_mem.g_buf[0].hdr[3] = 32'(glo_x);
    dut.u_env_mem.g_buf[0].hdr[4] = 32'(glo_y);
    dut.u_env_mem.g_buf[0].hdr[5] = 32'(glo_z);
    dut.u_env_mem.g_buf[0].hdr[6] = 32'(ghi_x);
    dut.u_env_mem.g_buf[0].hdr[7] = 32'(ghi_y);
    dut.u_env_mem.g_buf[0].hdr[8] = 32'(ghi_z);

    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    @(negedge clk);
    q_tag = 4'd1;
    q_user = 16'hbeef;
    q_cx = 18'sd20480;
    q_cy = 18'sd20480;
    q_cz = 18'sd20480;
    q_r = R_MAX;
    q_valid = 1'b1;
    do @(posedge clk); while (!q_ready);
    accepted_at = cycles;
    @(negedge clk);
    q_valid = 1'b0;

    while (!r_valid) begin
      @(posedge clk);
      if (cycles - accepted_at > 64'(MAX_LAT)) $fatal(1, "array exceeded MAX_LAT");
    end
    latency = cycles - accepted_at;
    if (r_tag != 1 || r_user != 16'hbeef || r_status != ST_MISS)
      $fatal(1, "wrong array stress response %0d", r_status);
    @(posedge clk);
    while (!cse_idle) @(posedge clk);
    @(negedge clk);
    eot = 1'b1;
    repeat (2) @(posedge clk);
    $display("TB_CSE_ARRAY_WORST ncu=4 voxels=729 cells=%0d points=%0d latency=%0d limit=%0d checker_errors=%0d",
             slot, base, latency, MAX_LAT, u_checker.errors);
    if (latency >= 64'(MAX_LAT) || cse_err || u_checker.errors != 0)
      $fatal(1, "array stress failed");
    $finish;
  end

  logic unused;
  assign unused = k_ready;

endmodule
