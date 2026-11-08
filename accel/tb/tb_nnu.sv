// ple_tree + ple_nnu vs golden tree_insert / nnu (DPI-C).
// Tree sizes around every K boundary up to NODE_CAP, duplicates (ties), extreme joint values, both trees;
// also checks the controller read port (q, parent, RAD = inf after insert, RAD write) and NNU latency.
module tb_nnu;
  import ple_pkg::*;

  import "DPI-C" function void fx_ref_tree_clear(input int tree);
  import "DPI-C" function int fx_ref_tree_insert(input int tree, input shortint q[7], input int parent);
  import "DPI-C" function int fx_ref_nnu(input int tree, input shortint q[7], output longint d2);

  localparam int K = 4;
  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  logic wr_en = 0, wr_tree = 0, rad_we = 0, rad_tree = 0, rd_en = 0, rd_tree = 0, start = 0, tree = 0;
  logic [IDX_W-1:0] wr_idx = '0, wr_parent = '0, rad_idx = '0, rd_idx = '0, rd_parent, idx, nn_row;
  logic [RAD_W-1:0] rad_wdata = '0, rd_rad;
  conf_t wr_q = '0, rd_q, q = '0;
  conf_t [K-1:0] nn_q;
  logic nn_en, nn_tree, busy, done;
  logic [CNT_W-1:0] n = '0;
  logic [D2_W-1:0] d2;

  ple_tree #(.K(K)) u_tree (.clk, .wr_en, .wr_tree, .wr_idx, .wr_q, .wr_parent, .rad_we, .rad_tree, .rad_idx,
                            .rad_wdata, .rd_en, .rd_tree, .rd_idx, .rd_q, .rd_parent, .rd_rad, .nn_en, .nn_tree,
                            .nn_row, .nn_q);
  ple_nnu #(.K(K)) u_nnu (.clk, .rst_n, .start, .tree, .q, .n, .busy, .done, .idx, .d2, .nn_en, .nn_tree, .nn_row,
                          .nn_q);

  int errs = 0, queries = 0, ties = 0, reads = 0, rad_checks = 0;
  longint unsigned cyc = 0;
  always @(posedge clk) begin
    cyc <= cyc + 1;
    if (cyc > 64'd5000000) $fatal(1, "TB_NNU watchdog: no progress");
  end
  int cnt[2];
  conf_t mirror[2][NODE_CAP];
  logic [IDX_W-1:0] par[2][NODE_CAP];

  function automatic conf_t rand_conf(input int mode);
    conf_t c;
    for (int j = 0; j < DIM; ++j)
      c[j] = (mode == 1) ? (($urandom % 2) ? 16'sh7fff : -16'sh8000)       // extreme values
                         : JW'(int'($urandom % 25737) - 12868);            // within the joint limits
    return c;
  endfunction

  function automatic void to_arr(input conf_t c, output shortint a[7]);
    for (int j = 0; j < DIM; ++j) a[j] = c[j];
  endfunction

  task automatic insert(input int t, input conf_t c);
    shortint a[7];
    int ri, p;
    p = cnt[t] ? int'($urandom % cnt[t]) : 0;
    to_arr(c, a);
    ri = fx_ref_tree_insert(t, a, p);
    if (ri != cnt[t]) begin $display("ERROR ref insert index %0d != %0d", ri, cnt[t]); errs++; end
    @(negedge clk);
    wr_en = 1; wr_tree = t[0]; wr_idx = IDX_W'(cnt[t]); wr_q = c; wr_parent = IDX_W'(p);
    @(negedge clk) wr_en = 0;
    mirror[t][cnt[t]] = c;
    par[t][cnt[t]] = IDX_W'(p);
    cnt[t]++;
  endtask

  task automatic query(input int t, input conf_t c);
    shortint a[7];
    longint rd2;
    int ri, cyc;
    to_arr(c, a);
    ri = fx_ref_nnu(t, a, rd2);
    @(negedge clk);
    start = 1; tree = t[0]; q = c; n = CNT_W'(cnt[t]);
    @(negedge clk) start = 0;
    cyc = 1;
    while (!done) begin @(negedge clk); cyc++; end
    queries++;
    if (int'(idx) != ri || d2 != D2_W'(rd2)) begin
      if (errs < 10) $display("ERROR tree=%0d n=%0d rtl idx=%0d d2=%0d ref idx=%0d d2=%0d", t, cnt[t], idx, d2, ri, rd2);
      errs++;
    end
    if (cyc != (cnt[t] + K - 1) / K + 5) begin       // rows issued + 4 pipeline stages + done register
      if (errs < 10) $display("ERROR latency n=%0d: %0d cycles", cnt[t], cyc);
      errs++;
    end
  endtask

  task automatic check_read(input int t, input int i);
    @(negedge clk);
    rd_en = 1; rd_tree = t[0]; rd_idx = IDX_W'(i);
    @(negedge clk) rd_en = 0;
    reads++;
    if (rd_q != mirror[t][i] || rd_parent != par[t][i] || rd_rad != RAD_INF) begin
      $display("ERROR read tree=%0d idx=%0d", t, i);
      errs++;
    end
  endtask

  int sizes[] = '{1, 2, 3, 4, 5, 7, 8, 9, 15, 16, 17, 63, 64, 65, 255, 256, 257, 511, 1023, 1024};

  initial begin
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    foreach (sizes[s]) begin
      for (int t = 0; t < 2; ++t) begin
        int other = 1 - t;
        fx_ref_tree_clear(0); fx_ref_tree_clear(1); cnt[0] = 0; cnt[1] = 0;
        for (int i = 0; i < 3; ++i) insert(other, rand_conf(0));        // the other tree must be ignored
        for (int i = 0; i < sizes[s]; ++i) begin
          int r = $urandom % 10;
          if (r < 2 && cnt[t] > 0) insert(t, mirror[t][$urandom % cnt[t]]);   // duplicate -> ties
          else insert(t, rand_conf(r == 9 ? 1 : 0));
        end
        for (int k = 0; k < 24; ++k) begin
          conf_t c;
          if (k % 3 == 0) begin c = mirror[t][$urandom % cnt[t]]; ties++; end   // exact node: d2 = 0
          else c = rand_conf(k % 7 == 0 ? 1 : 0);
          query(t, c);
        end
        for (int k = 0; k < 8; ++k) check_read(t, $urandom % cnt[t]);
      end
    end
    // RAD: random writes over both trees, then read everything back
    begin
      logic [RAD_W-1:0] rmir[2][NODE_CAP];
      for (int t = 0; t < 2; ++t) for (int i = 0; i < NODE_CAP; ++i) rmir[t][i] = RAD_INF;
      for (int t = 0; t < 2; ++t) for (int i = 0; i < cnt[t]; ++i) rmir[t][i] = RAD_INF;
      for (int k = 0; k < 3000; ++k) begin
        int t = $urandom % 2, i = $urandom % NODE_CAP;
        logic [RAD_W-1:0] v = $urandom;
        @(negedge clk);
        rad_we = 1; rad_tree = t[0]; rad_idx = IDX_W'(i); rad_wdata = v;
        rmir[t][i] = v;
        if (k % 50 == 0) begin                            // insert in the same cycle wins: RAD = inf
          wr_en = 1; wr_tree = t[0]; wr_idx = IDX_W'(i); wr_q = rand_conf(0); wr_parent = '0;
          rmir[t][i] = RAD_INF;
        end
        @(negedge clk) begin rad_we = 0; wr_en = 0; end
      end
      for (int t = 0; t < 2; ++t)
        for (int i = 0; i < NODE_CAP; i += 3) begin
          @(negedge clk) begin rd_en = 1; rd_tree = t[0]; rd_idx = IDX_W'(i); end
          @(negedge clk) rd_en = 0;
          rad_checks++;
          if (rd_rad != rmir[t][i]) begin
            if (errs < 10) $display("ERROR rad tree=%0d idx=%0d rtl=%h exp=%h", t, i, rd_rad, rmir[t][i]);
            errs++;
          end
        end
    end
    if ($test$plusargs("rad_conflict")) begin          // control: must trip the ple_tree assertion
      @(negedge clk);
      wr_en = 1; wr_tree = 0; wr_idx = 10'd5; rad_we = 1; rad_tree = 0; rad_idx = 10'd6;
      @(negedge clk) begin wr_en = 0; rad_we = 0; end
    end
    $display("TB_NNU queries=%0d exact_node_queries=%0d reads=%0d rad_checks=%0d errors=%0d", queries, ties, reads,
             rad_checks, errs);
    if (errs != 0) $fatal(1, "TB_NNU FAIL");
    $finish;
  end

endmodule
