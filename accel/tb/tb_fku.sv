// ple_fku vs golden fx_fk_ws (DPI-C): all 70 x 3 workspace coordinates per pose. Poses: the 128 joint-limit
// corners, random poses within the limits, and full-range int16 joints (register-width check).
module tb_fku;
  import ple_pkg::*;

  import "DPI-C" function void fx_ref_fk_ws(input shortint q[7], output int c[210]);

  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  logic start = 0, busy, done;
  conf_t q = '0;
  logic [69:0][2:0][17:0] c;
  ple_fku u_fku (.clk, .rst_n, .start, .q, .busy, .done, .c);

  int errs = 0, poses = 0, lat0 = -1, nlim = 0, busy_starts = 0, done_starts = 0;
  longint unsigned cyc = 0;
  always @(posedge clk) begin
    cyc <= cyc + 1;
    if (cyc > 64'd50000000) $fatal(1, "TB_FKU watchdog: no progress");
  end

  // joint limits: floor(lim_a * 2^12), floor(lim_m * 2^12) (ple_pkg SMP_A / SMP_M)
  task automatic run(input shortint qq[7]);
    int ref_c[210];
    int lat = 0;
    fx_ref_fk_ws(qq, ref_c);
    @(negedge clk) begin
      for (int j = 0; j < DIM; ++j) q[j] = qq[j];
      start = 1;
    end
    @(negedge clk) start = 0;
    while (!done) begin
      start = busy && ($urandom % 7 == 0);              // ignored while busy
      busy_starts += int'(start);
      @(negedge clk);
      lat++;
    end
    start = 0;
    if ($urandom % 5 == 0) begin                        // a start in the done cycle is dropped
      start = 1;
      @(negedge clk) start = 0;
      done_starts++;
      if (busy) begin $display("ERROR start in the done cycle was accepted"); errs++; end
    end
    if (lat0 < 0) lat0 = lat;
    else if (lat != lat0) begin $display("ERROR latency %0d vs %0d", lat, lat0); errs++; end
    poses++;
    for (int s = 0; s < 70; ++s)
      for (int k = 0; k < 3; ++k)
        if (int'(signed'(c[s][k])) != ref_c[3 * s + k]) begin
          if (errs < 10) $display("ERROR pose %0d sphere %0d axis %0d rtl=%0d ref=%0d", poses, s, k,
                                  int'(signed'(c[s][k])), ref_c[3 * s + k]);
          errs++;
        end
  endtask

  initial begin
    shortint qq[7];
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    for (int m = 0; m < 128; ++m) begin                             // every joint at its lower / upper limit
      for (int j = 0; j < DIM; ++j) qq[j] = ((m >> j) & 1) ? shortint'(SMP_A[j] + signed'({1'b0, SMP_M[j]}) - 1) : SMP_A[j];
      run(qq);
    end
    nlim = poses;
    for (int i = 0; i < 20000; ++i) begin
      for (int j = 0; j < DIM; ++j)
        qq[j] = (i % 10 == 9) ? shortint'($urandom) : shortint'(int'(SMP_A[j]) + int'($urandom % SMP_M[j]));
      run(qq);
    end
    $display("TB_FKU poses=%0d (limit corners %0d) latency=%0d coords_checked=%0d busy_starts=%0d done_cycle_starts=%0d errors=%0d",
             poses, nlim, lat0, poses * 210, busy_starts, done_starts, errs);
    if (errs != 0) $fatal(1, "TB_FKU FAIL");
    $finish;
  end

endmodule
