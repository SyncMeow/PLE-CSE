// ple_scu vs golden fx_self_ok (DPI-C). Centres come from the golden FK of random poses (within the limits
// and full-range int16), plus boundary cases: an inner pair placed exactly at / one past its threshold
// with its outer pair forced to hit. Also: start while busy and in the done cycle is ignored.
module tb_scu;
  import ple_pkg::*;

  import "DPI-C" function void fx_ref_fk_ws(input shortint q[7], output int c[210]);
  import "DPI-C" function int fx_ref_self_ok(input int c[210]);
  import "DPI-C" function longint fx_ref_self_info(input int i, output int a, output int b, output int oa,
                                                   output int ob, output longint othr);
  import "DPI-C" function longint unsigned fx_ref_isqrt(input longint unsigned v);

  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  logic start = 0, busy, done, collide;
  logic [69:0][2:0][17:0] c = '0;
  ple_scu u_scu (.clk, .rst_n, .start, .c, .busy, .done, .collide);

  // integer offset with dx^2 + dy^2 + dz^2 == target (searched; 0 if none found)
  function automatic bit find3(input longint target, output int dx, output int dy, output int dz);
    for (longint x = longint'(fx_ref_isqrt(64'(target))); x >= 0 && x > longint'(fx_ref_isqrt(64'(target))) - 64; --x) begin
      longint r = target - x * x;
      for (longint y = longint'(fx_ref_isqrt(64'(r))); y >= 0; --y) begin
        longint z2 = r - y * y;
        longint z = longint'(fx_ref_isqrt(64'(z2)));
        if (z * z == z2) begin dx = int'(x); dy = int'(y); dz = int'(z); return 1; end
        if (y * y < r / 2) break;
      end
    end
    return 0;
  endfunction

  int exact = 0;
  int errs = 0, poses = 0, hits = 0, bnd = 0, bnd_hits = 0, lat_min = 1 << 30, lat_max = 0, busy_starts = 0;
  longint unsigned cyc = 0;
  always @(posedge clk) begin
    cyc <= cyc + 1;
    if (cyc > 64'd100000000) $fatal(1, "TB_SCU watchdog: no progress");
  end

  task automatic run(input int cc[210], input bit boundary);
    int exp_ok, lat = 0;
    exp_ok = fx_ref_self_ok(cc);
    @(negedge clk) begin
      for (int s = 0; s < 70; ++s) for (int k = 0; k < 3; ++k) c[s][k] = 18'(cc[3 * s + k]);
      start = 1;
    end
    @(negedge clk) begin
      start = 0;
      c = '0;                                           // c is latched at start
    end
    while (!done) begin
      start = busy && ($urandom % 9 == 0);
      busy_starts += int'(start);
      @(negedge clk);
      lat++;
    end
    start = 0;
    if ($urandom % 5 == 0) begin                        // a start in the done cycle is dropped
      start = 1;
      @(negedge clk) start = 0;
      if (busy) begin $display("ERROR start in the done cycle was accepted"); errs++; end
    end
    poses++;
    if (lat < lat_min) lat_min = lat;
    if (lat > lat_max) lat_max = lat;
    hits += int'(collide);
    if (boundary) begin bnd++; bnd_hits += int'(collide); end
    if (collide != (exp_ok == 0)) begin
      if (errs < 10) $display("ERROR pose %0d boundary=%0d rtl collide=%0d golden ok=%0d", poses, boundary, collide, exp_ok);
      errs++;
    end
  endtask

  initial begin
    shortint qq[7];
    int cc[210];
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    for (int i = 0; i < 20000; ++i) begin
      for (int j = 0; j < DIM; ++j)
        qq[j] = (i % 10 == 9) ? shortint'($urandom) : shortint'(int'(SMP_A[j]) + int'($urandom % SMP_M[j]));
      fx_ref_fk_ws(qq, cc);
      run(cc, 0);
      if (i % 4 == 0) begin                             // boundary: one inner pair at its threshold
        int a, b, oa, ob, p;
        longint thr, othr;
        longint unsigned r;
        int dx, dy, dz;
        p = $urandom % 690;
        thr = fx_ref_self_info(p, a, b, oa, ob, othr);
        for (int k = 0; k < 3; ++k) cc[3 * ob + k] = cc[3 * oa + k];            // outer pair hits
        if (find3(thr + longint'($urandom % 2), dx, dy, dz)) begin            // d^2 == thr or thr + 1
          exact++;
        end else begin
          r = fx_ref_isqrt(64'(thr)) + 64'($urandom % 2);                     // nearest along x
          dx = int'(r); dy = 0; dz = 0;
        end
        cc[3 * b] = cc[3 * a] + dx;
        cc[3 * b + 1] = cc[3 * a + 1] + dy;
        cc[3 * b + 2] = cc[3 * a + 2] + dz;
        run(cc, 1);
      end
    end
    $display("TB_SCU poses=%0d self_collisions=%0d boundary=%0d (exact d2 %0d, hits %0d) latency=%0d..%0d busy_starts=%0d errors=%0d",
             poses, hits, bnd, exact, bnd_hits, lat_min, lat_max, busy_starts, errs);
    if (errs != 0) $fatal(1, "TB_SCU FAIL");
    $finish;
  end

endmodule
