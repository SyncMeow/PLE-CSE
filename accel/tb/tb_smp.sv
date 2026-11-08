// ple_smp vs golden smp_next (DPI-C): every sample of several seeds, random take delays.
//   +n=<samples per seed>   +refoff=1 makes the reference use seed+1 (control: must mismatch)
module tb_smp;
  import ple_pkg::*;

  import "DPI-C" function void fx_ref_smp_reset(input int seed);
  import "DPI-C" function int fx_ref_smp_shift(input int j);
  import "DPI-C" function void fx_ref_smp_next(output shortint q[7]);
  import "DPI-C" function int unsigned fx_ref_smp_delta(input int j, input int k);

  logic clk = 1'b0, rst_n = 1'b0, init = 1'b0, take = 1'b0, valid;
  logic [DIM-1:0][31:0] shift;
  conf_t q;
  always #5 clk = ~clk;

  ple_smp u_smp (.clk, .rst_n, .init, .shift, .take, .valid, .q);

  int n = 50000, refoff = 0, mism = 0, total = 0, dmism = 0, lat_min = 1 << 30, lat_max = 0, inits = 0, verr = 0;
  longint unsigned cyc = 0;
  always @(posedge clk) begin
    cyc <= cyc + 1;
    if (cyc > 64'(n) * 64 + 100000) $fatal(1, "TB_SMP watchdog: no progress");
  end
  int seeds[4] = '{0, 1, 7, 123};
  shortint r[7];

  initial begin
    void'($value$plusargs("n=%d", n));
    void'($value$plusargs("refoff=%d", refoff));
    shift = '0;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    // the whole 7 x 24 step table, including digits a 50k-sample run never reaches
    fx_ref_smp_reset(0);
    for (int j = 0; j < DIM; ++j)
      for (int k = 0; k < H_DIGITS; ++k)
        if (u_smp.delta[j][k] != fx_ref_smp_delta(j, k)) begin
          if (dmism < 5) $display("DELTA MISMATCH j=%0d k=%0d rtl=%h ref=%h", j, k, u_smp.delta[j][k], fx_ref_smp_delta(j, k));
          dmism++;
        end
    foreach (seeds[s]) begin
      fx_ref_smp_reset(seeds[s]);
      for (int j = 0; j < DIM; ++j) shift[j] = 32'(fx_ref_smp_shift(j));
      fx_ref_smp_reset(seeds[s] + refoff);
      @(negedge clk) init = 1'b1;
      @(negedge clk) init = 1'b0;
      for (int i = 0; i < n; ++i) begin
        int lat = 0;
        while (!valid) begin @(negedge clk); lat++; end
        if (i > 0) begin
          if (lat < lat_min) lat_min = lat;
          if (lat > lat_max) lat_max = lat;
        end
        repeat ($urandom % 3) @(negedge clk);       // the sample must stay stable while held
        fx_ref_smp_next(r);
        total++;
        for (int j = 0; j < DIM; ++j)
          if (q[j] != r[j]) begin
            if (mism < 5) $display("MISMATCH seed=%0d sample=%0d joint=%0d rtl=%0d ref=%0d", seeds[s], i + 1, j, q[j], r[j]);
            mism++;
          end
        take = 1'b1;
        @(negedge clk) take = 1'b0;
        if (valid) begin verr++; if (verr < 5) $display("ERROR valid still high after take"); end
        if (s == 3 && ($urandom % 500) == 0) begin       // last seed only: long runs keep slow drift visible
          repeat ($urandom % 10) @(negedge clk);
          init = 1'b1;
          @(negedge clk) init = 1'b0;
          fx_ref_smp_reset(seeds[s] + refoff);
          inits++;
        end
      end
    end
    $display("TB_SMP samples=%0d mismatches=%0d delta_table_mismatches=%0d take_to_valid=%0d..%0d mid_inits=%0d valid_errors=%0d",
             total, mism, dmism, lat_min, lat_max, inits, verr);
    if (mism != 0 || dmism != 0 || verr != 0 || lat_min != lat_max) $fatal(1, "TB_SMP FAIL");
    $finish;
  end

endmodule
