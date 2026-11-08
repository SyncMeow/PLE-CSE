// ple_isqrt and ple_div7 vs golden fx_isqrt / fx_fdiv (DPI-C): corner values, perfect squares +-1,
// realistic edge lengths and EXTEND / CONNECT operands, full-range random; fixed latency checked.
module tb_alu;
  import ple_pkg::*;

  import "DPI-C" function longint unsigned fx_ref_isqrt(input longint unsigned v);
  import "DPI-C" function longint fx_ref_fdiv(input longint a, input longint b);

  localparam int AW = 50, BW = 32;
  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  logic s_start = 0, s_busy, s_done;
  logic [63:0] s_x = '0;
  logic [31:0] s_root;
  ple_isqrt u_sqrt (.clk, .rst_n, .start(s_start), .x(s_x), .busy(s_busy), .done(s_done), .root(s_root));

  logic d_start = 0, d_busy, d_done;
  logic [DIM-1:0][AW-1:0] d_a = '0, d_q;
  logic [BW-1:0] d_b = 32'd1;
  logic [DIM-1:0][BW-1:0] d_r;
  ple_div7 #(.AW(AW), .BW(BW)) u_div (.clk, .rst_n, .start(d_start), .a(d_a), .b(d_b), .busy(d_busy),
                                       .done(d_done), .q(d_q), .r(d_r));

  int errs = 0, nsq = 0, ndiv = 0, lat_s = -1, lat_d = -1;
  longint unsigned cyc = 0;
  always @(posedge clk) begin
    cyc <= cyc + 1;
    if (cyc > 64'd50000000) $fatal(1, "TB_ALU watchdog: no progress");
  end

  function automatic longint unsigned r64();
    return {$urandom, $urandom};
  endfunction

  task automatic sq(input longint unsigned x);
    int lat = 0;
    longint unsigned ref_r = fx_ref_isqrt(x);
    @(negedge clk) begin s_x = x; s_start = 1; end
    @(negedge clk) s_start = 0;
    while (!s_done) begin @(negedge clk); lat++; end
    if (lat_s < 0) lat_s = lat;
    else if (lat != lat_s) begin $display("ERROR isqrt latency %0d vs %0d", lat, lat_s); errs++; end
    nsq++;
    if (64'(s_root) != ref_r) begin
      if (errs < 10) $display("ERROR isqrt(%0d) rtl=%0d ref=%0d", x, s_root, ref_r);
      errs++;
    end
  endtask

  task automatic dv(input longint av[7], input longint unsigned bv);
    int lat = 0;
    @(negedge clk) begin
      for (int j = 0; j < DIM; ++j) d_a[j] = AW'(av[j]);
      d_b = BW'(bv);
      d_start = 1;
    end
    @(negedge clk) d_start = 0;
    while (!d_done) begin @(negedge clk); lat++; end
    if (lat_d < 0) lat_d = lat;
    else if (lat != lat_d) begin $display("ERROR div7 latency %0d vs %0d", lat, lat_d); errs++; end
    for (int j = 0; j < DIM; ++j) begin
      longint rq = fx_ref_fdiv(av[j], longint'(bv));
      longint rr = av[j] - rq * longint'(bv);
      ndiv++;
      if (longint'(signed'(d_q[j])) != rq || longint'(d_r[j]) != rr) begin
        if (errs < 10) $display("ERROR fdiv(%0d, %0d) rtl q=%0d r=%0d ref q=%0d r=%0d", av[j], bv,
                                longint'(signed'(d_q[j])), d_r[j], rq, rr);
        errs++;
      end
    end
  endtask

  initial begin
    longint av[7];
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    // ---- isqrt
    sq(0); sq(1); sq(2); sq(3); sq(4); sq(64'hFFFF_FFFF_FFFF_FFFF); sq(64'hFFFF_FFFE_0000_0001);
    for (int i = 0; i < 3000; ++i) begin
      longint unsigned t = {32'd0, $urandom};
      sq(t * t); sq(t * t - 1); sq(t * t + 1);                     // perfect squares and neighbours
    end
    for (int i = 0; i < 6000; ++i) begin
      longint unsigned d2 = r64() % (64'd7 << 34);                  // squared joint distance
      sq(d2 << 24);                                                 // len_h as in the model
      sq(r64());                                                    // full range
    end
    // ---- div7
    for (int i = 0; i < 4000; ++i) begin
      int mode = i % 5;
      longint unsigned b;
      for (int j = 0; j < DIM; ++j)
        case (mode)
          0: av[j] = longint'(int'($urandom % 131071) - 65535) * longint'(1 << 24);    // EXTEND: d * range_h
          1: av[j] = longint'(int'($urandom % 131071) - 65535);                       // CONNECT: ovec
          2: av[j] = (j % 2) ? -(longint'(1) <<< (AW - 1)) : ((longint'(1) <<< (AW - 1)) - 1);  // extremes
          3: av[j] = longint'($urandom % 7) - 3;                                       // tiny, near zero
          default: av[j] = longint'(r64() % (64'd1 << (AW - 1))) * ((j % 2) ? -1 : 1);
        endcase
      case (mode)
        0: b = 64'($urandom % (32'd14 << 24)) + 1;                  // nd_h
        1: b = 64'($urandom % 16) + 1;                              // n_ext
        2: b = (i % 3 == 0) ? 64'd1 : 64'hFFFF_FFFF;
        3: b = 64'($urandom % 4) + 1;
        default: b = 64'($urandom) | 64'd1;
      endcase
      dv(av, b);
    end
    $display("TB_ALU isqrt=%0d (latency %0d) div=%0d (latency %0d) errors=%0d", nsq, lat_s, ndiv, lat_d, errs);
    if (errs != 0) $fatal(1, "TB_ALU FAIL");
    $finish;
  end

endmodule
