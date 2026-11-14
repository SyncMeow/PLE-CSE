// ple_scu vs golden fx_self_ok_filt (DPI-C, the self check with the row filter) on every input, and vs the plain
// fx_self_ok on FK poses (the filter must not change the result there). Centres come from the golden FK of random
// poses (within the limits and full-range int16), plus boundary cases with the outer pair forced to hit: an inner
// pair exactly at / one past its threshold, and a row exactly at / one past its filter threshold with an inner hit
// behind it. Also: start while busy and in the done cycle is ignored.
module tb_scu;
  import ple_pkg::*;

  import "DPI-C" function void fx_ref_fk_ws(input shortint q[7], output int c[210]);
  import "DPI-C" function int fx_ref_self_ok(input int c[210]);
  import "DPI-C" function int fx_ref_self_ok_filt(input int c[210]);
  import "DPI-C" function longint fx_ref_self_fthr(input int i);
  import "DPI-C" function longint fx_ref_self_info(input int i, output int a, output int b, output int oa,
                                                   output int ob, output longint othr);
  import "DPI-C" function longint unsigned fx_ref_isqrt(input longint unsigned v);

  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  logic start = 0, busy, done, collide;
  logic [9:0] pairs;
  int pair_checks = 0;
  logic [69:0][2:0][17:0] c = '0;
  ple_scu u_scu (.clk, .rst_n, .start, .c, .busy, .done, .collide, .pairs);

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

  // offset with the largest d^2 <= target (up = 0) or the smallest d^2 > target (up = 1) that integers can reach
  function automatic longint near3(input longint target, input bit up, output int dx, output int dy, output int dz);
    for (longint t = up ? target + 1 : target; t >= 0 && t <= target + 64 && t >= target - 64; t += up ? 1 : -1)
      if (find3(t, dx, dy, dz)) return t;
    $fatal(1, "TB_SCU near3: no reachable d2 near %0d", target);
    return 0;
  endfunction
  int errs = 0, poses = 0, hits = 0, bnd = 0, bnd_hits = 0, lat_min = 1 << 30, lat_max = 0, busy_starts = 0;
  int fk_poses = 0, fk_lat = 0, filt_diff = 0, fbnd = 0, fbnd_hits = 0;
  longint unsigned cyc = 0;
  always @(posedge clk) begin
    cyc <= cyc + 1;
    if (cyc > 64'd100000000) $fatal(1, "TB_SCU watchdog: no progress");
  end

  // kind: 0 = FK pose, 1 = inner threshold boundary, 2 = filter threshold boundary, 3 = arbitrary centres
  task automatic run(input int cc[210], input int kind);
    int exp_ok, lat = 0, r;
    longint p0;
    bit boundary = (kind == 1);
    r = fx_ref_self_ok_filt(cc);
    exp_ok = r & 1;
    p0 = longint'(r >> 1);
    if (kind == 0 && fx_ref_self_ok(cc) != exp_ok) begin
      if (errs < 10) $display("ERROR pose %0d: the row filter changes the golden result (filter ok=%0d)", poses, exp_ok);
      errs++;
    end
    if (kind != 0 && fx_ref_self_ok(cc) != exp_ok) filt_diff++;
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
    if (exp_ok != 0) begin                                // collision-free: hardware tests exactly the golden pairs
      pair_checks++;
      if (longint'(pairs) != p0) begin
        if (errs < 10) $display("ERROR pose %0d pair tests rtl=%0d golden=%0d hitm=%b", poses, pairs, p0, u_scu.hitm);
        errs++;
      end
    end else if (pairs < 10'd22) begin $display("ERROR pose %0d: %0d pair tests on a collision", poses, pairs); errs++; end
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
    if (kind == 2) begin fbnd++; fbnd_hits += int'(collide); end
    if (kind == 0) begin fk_poses++; fk_lat += lat; end
    if (collide != (exp_ok == 0)) begin
      if (errs < 10) $display("ERROR pose %0d boundary=%0d rtl collide=%0d golden ok=%0d", poses, boundary, collide, exp_ok);
      errs++;
    end
  endtask

  // every inner pair alone (all other spheres far apart): the pair at the last reachable d^2 inside / first outside its
  // threshold (v = 0), and the pair hitting with its row at the last / first reachable d^2 of the filter threshold (v = 1)
  int iso = 0, iso_hits = 0, iso_tight = 0;
  task automatic isolated();
    int cc[210], a, b, oa, ob, dx, dy, dz, h0;
    longint thr, othr, fthr, t;
    for (int p = 0; p < 690; ++p)
      for (int v = 0; v < 2; ++v)
        for (int up = 0; up < 2; ++up) begin
          for (int sp = 0; sp < 70; ++sp) begin
            cc[3 * sp] = (sp % 5) * 40000 - 80000; cc[3 * sp + 1] = ((sp / 5) % 5) * 40000 - 80000; cc[3 * sp + 2] = (sp / 25) * 40000 - 40000;
          end
          thr = fx_ref_self_info(p, a, b, oa, ob, othr);
          fthr = fx_ref_self_fthr(p);
          t = near3(v == 0 ? thr : fthr, up != 0, dx, dy, dz);
          iso_tight += int'(t == (v == 0 ? thr : fthr) + longint'(up));
          if (v == 0) begin                               // b at the boundary, the row passes (outer sphere on a)
            cc[3 * b] = cc[3 * a] + dx; cc[3 * b + 1] = cc[3 * a + 1] + dy; cc[3 * b + 2] = cc[3 * a + 2] + dz;
            for (int k = 0; k < 3; ++k) cc[3 * ob + k] = cc[3 * a + k];
          end else begin                                  // b on a (hit), the outer sphere at the filter boundary
            for (int k = 0; k < 3; ++k) cc[3 * b + k] = cc[3 * a + k];
            cc[3 * ob] = cc[3 * a] + dx; cc[3 * ob + 1] = cc[3 * a + 1] + dy; cc[3 * ob + 2] = cc[3 * a + 2] + dz;
          end
          for (int k = 0; k < 3; ++k) cc[3 * oa + k] = cc[3 * ob + k];           // outer pair hits
          h0 = hits;
          run(cc, 3);
          iso++;
          iso_hits += hits - h0;
          if ((hits - h0) != int'(up == 0)) begin
            if (errs < 10) $display("ERROR isolated pair %0d v=%0d up=%0d: collide=%0d", p, v, up, hits - h0);
            errs++;
          end
        end
  endtask

  initial begin
    shortint qq[7];
    int full_range = 0;
    int cc[210];
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    isolated();
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
        r = 64'($urandom % 2);
        exact += int'(near3(thr, r[0], dx, dy, dz) == thr + longint'(r));     // last d^2 inside / first outside
        cc[3 * b] = cc[3 * a] + dx;
        cc[3 * b + 1] = cc[3 * a + 1] + dy;
        cc[3 * b + 2] = cc[3 * a + 2] + dz;
        run(cc, 1);
      end
      if (i % 4 == 2) begin                             // boundary: a row at its filter threshold, an inner hit behind it
        int a, b, oa, ob, p;
        longint thr, othr, fthr;
        int dx, dy, dz;
        p = $urandom % 690;
        thr = fx_ref_self_info(p, a, b, oa, ob, othr);
        fthr = fx_ref_self_fthr(p);
        for (int k = 0; k < 3; ++k) cc[3 * b + k] = cc[3 * a + k];             // the inner pair hits (d = 0)
        void'(near3(fthr, 1'($urandom % 2), dx, dy, dz));                     // last d^2 inside / first outside
        cc[3 * ob] = cc[3 * a] + dx;
        cc[3 * ob + 1] = cc[3 * a + 1] + dy;
        cc[3 * ob + 2] = cc[3 * a + 2] + dz;
        for (int k = 0; k < 3; ++k) cc[3 * oa + k] = cc[3 * ob + k];           // outer pair hits
        run(cc, 2);
      end
    end
    // full s18 range: uniform, clusters around a random base, and the extreme values (signed widths)
    for (int i = 0; i < 6000; ++i) begin
      int base[3], m;
      m = i % 3;
      for (int k = 0; k < 3; ++k) base[k] = int'($urandom % 262144) - 131072;
      for (int sp = 0; sp < 70; ++sp)
        for (int k = 0; k < 3; ++k) begin
          int v;
          if (m == 0) v = int'($urandom % 262144) - 131072;
          else if (m == 1) begin
            v = base[k] + int'($urandom % 4001) - 2000;
            if (v > 131071) v = 131071;
            if (v < -131072) v = -131072;
          end else v = ($urandom % 2) ? 131071 : -131072;
          cc[3 * sp + k] = v;
        end
      run(cc, 3);
      full_range++;
    end
    if (fbnd_hits == 0 || fbnd_hits == fbnd) begin $display("ERROR filter boundary: %0d of %0d collide (both sides needed)", fbnd_hits, fbnd); errs++; end
    if (filt_diff == 0) begin $display("ERROR no input where the filter hides an inner hit (reject path not exercised)"); errs++; end
    if (fk_lat > 40 * fk_poses) begin $display("ERROR mean latency on FK poses %0d cycles (limit 40)", fk_lat / fk_poses); errs++; end
    $display("TB_SCU pair_checks=%0d poses=%0d self_collisions=%0d boundary=%0d (exact d2 %0d, hits %0d) filter_boundary=%0d (hits %0d) filter_hides=%0d isolated=%0d (hits %0d, at thr %0d) full_range=%0d latency=%0d..%0d fk_mean_latency_x10=%0d busy_starts=%0d errors=%0d", pair_checks,
             poses, hits, bnd, exact, bnd_hits, fbnd, fbnd_hits, filt_diff, iso, iso_hits, iso_tight, full_range, lat_min, lat_max, 10 * fk_lat / fk_poses, busy_starts, errs);
    if (errs != 0) $fatal(1, "TB_SCU FAIL");
    $finish;
  end

endmodule
