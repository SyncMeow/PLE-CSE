// ple_rkg vs golden edge_ok pose sequence (DPI-C): random edges (negative / zero vectors, n = 1 up to
// hundreds of rows), random backpressure, aborts in the middle of an edge.
module tb_rkg;
  import ple_pkg::*;

  import "DPI-C" function int fx_ref_rake(input shortint s[7], input int v[7], input longint len_h);
  import "DPI-C" function void fx_ref_rake_pose(input int i, output shortint q[7]);

  localparam int VW = 18;
  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  logic start = 0, kill = 0, busy, p_valid, p_ready = 0, p_last;
  conf_t s_q = '0, p_q;
  logic [DIM-1:0][VW-1:0] vec = '0;
  logic [31:0] len_h = '0;
  logic [15:0] n_out;

  ple_rkg #(.VW(VW)) u_rkg (.clk, .rst_n, .start, .s_q, .vec, .len_h, .kill, .busy, .n_out, .p_valid, .p_ready,
                            .p_q, .p_last);

  int errs = 0, edges = 0, poses = 0, aborts = 0, max_n = 0, busy_starts = 0, start_kills = 0;
  longint unsigned cyc = 0;
  always @(posedge clk) begin
    cyc <= cyc + 1;
    if (cyc > 64'd20000000) $fatal(1, "TB_RKG watchdog: no progress");
  end

  function automatic int rj();
    return int'($urandom % 25737) - 12868;              // within the Panda joint limits
  endfunction

  task automatic run_edge(input shortint s[7], input int v[7], input longint len, input int abort_at);
    int np, got;
    shortint r[7];
    np = fx_ref_rake(s, v, len);
    @(negedge clk);
    for (int j = 0; j < DIM; ++j) begin s_q[j] = s[j]; vec[j] = VW'(v[j]); end
    len_h = 32'(len);
    start = 1;
    @(negedge clk) start = 0;
    got = 0;
    while (1) begin
      logic hs, last;
      p_ready = ($urandom % 4) != 0;
      if (abort_at >= 0 && got == abort_at) begin
        kill = 1;
        if ($urandom % 2) begin start = 1; start_kills++; end   // kill wins: this start is dropped
        @(negedge clk) begin kill = 0; start = 0; end
        aborts++;
        if (busy) begin $display("ERROR edge %0d busy after kill", edges); errs++; end
        break;
      end
      if (busy && ($urandom % 9) == 0) begin              // start while busy must be ignored
        start = 1;
        busy_starts++;
      end
      #1;                                               // sample before the rising edge
      hs = p_valid && p_ready;
      last = p_last;
      if (hs) begin
        fx_ref_rake_pose(got, r);
        for (int j = 0; j < DIM; ++j)
          if (p_q[j] != r[j]) begin
            if (errs < 8) $display("ERROR edge %0d pose %0d joint %0d rtl=%0d ref=%0d (n=%0d)", edges, got, j, p_q[j], r[j], n_out);
            errs++;
          end
        if (last != (got == np - 1)) begin
          if (errs < 8) $display("ERROR edge %0d pose %0d p_last=%0d (count %0d)", edges, got, last, np);
          errs++;
        end
        got++;
        poses++;
      end
      @(negedge clk) start = 0;
      if (hs && last) break;
    end
    if (abort_at < 0 && got != np) begin
      $display("ERROR edge %0d emitted %0d poses, golden %0d", edges, got, np);
      errs++;
    end
    if (int'(n_out) > max_n) max_n = int'(n_out);
    edges++;
    p_ready = 0;
  endtask

  initial begin
    shortint s[7];
    int v[7];
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    // directed extremes: |vec| up to 2^17 - 1, n = 1, 2 and the maximum 1023, joint extremes
    begin
      longint lens[3] = '{longint'(1), longint'((1 << 22) + 1), longint'(1023) << 22};
      int pats[4][7] = '{'{131071, -131071, 131071, -131071, 131071, -131071, 131071},
                         '{-131071, -131071, -131071, -131071, -131071, -131071, -131071},
                         '{131071, 0, -1, 1, -131072 + 1, 65536, -65537},
                         '{7, -7, 8, -8, 9, -9, 0}};
      foreach (lens[a])
        foreach (pats[b]) begin
          for (int j = 0; j < DIM; ++j) begin
            s[j] = (b % 2) ? 16'sh7fff : -16'sh8000;
            v[j] = pats[b][j];
          end
          run_edge(s, v, lens[a], -1);
        end
    end
    for (int e = 0; e < 3000; ++e) begin
      longint len;
      int mode = e % 6;
      for (int j = 0; j < DIM; ++j) begin
        s[j] = shortint'(rj());
        v[j] = (mode == 5) ? 0 : rj() - int'(s[j]);    // vec = end - start (zero vector in mode 5)
      end
      // len_h independent of vec (as in the model): tiny (n = 1), up to range (n <= 4), DIRECT-like, huge
      case (mode)
        0: len = longint'($urandom % (1 << 22));                       // n = 1
        1, 5: len = longint'($urandom % (1 << 24));                    // n <= 4
        2: len = longint'(($urandom % 14) + 1) << 24;                  // exact multiples of 1 rad
        3: len = longint'($urandom % (14 << 24));                      // DIRECT-like, n <= 56
        default: len = longint'($urandom % (100 << 24));               // up to n = 400
      endcase
      run_edge(s, v, len, (e % 13 == 7) ? int'($urandom % 20) : -1);
    end
    $display("TB_RKG edges=%0d poses=%0d aborts=%0d busy_starts=%0d start_kills=%0d max_n=%0d errors=%0d", edges, poses,
             aborts, busy_starts, start_kills, max_n, errs);
    if (errs != 0) $fatal(1, "TB_RKG FAIL");
    $finish;
  end

endmodule
