// ple_top through its AXI-Lite host interface, with the CSE behavioural model: the host programs the registers,
// starts a run, waits for the interrupt and reads results, counters and the path over AXI-Lite; compared with
// golden fx_solve. Also: register read-back, byte strobes, writes ignored while busy, IRQ clear, abort, restart.
//   +probs=<n> (default 4)  +iters=<max> (default 3000)
module tb_top;
  import ple_pkg::*;
  import ple_cse_pkg::*;

  import "DPI-C" function int fx_ref_env_load(input int i, input int flat);
  import "DPI-C" function int fx_ref_solve(input int i, input int seed, input int max_iter, input int range_h, output int stats[8],
                                           output shortint sg[14], output int shift[7], output shortint path[448]);

  logic clk = 1'b0, rst_n = 1'b0, eot = 1'b0;
  always #5 clk = ~clk;

  logic [11:0] s_axil_awaddr = '0, s_axil_araddr = '0;
  logic s_axil_awvalid = 0, s_axil_awready, s_axil_wvalid = 0, s_axil_wready, s_axil_bvalid, s_axil_bready = 0;
  logic [31:0] s_axil_wdata = '0, s_axil_rdata;
  logic [3:0] s_axil_wstrb = '0;
  logic [1:0] s_axil_bresp, s_axil_rresp;
  logic s_axil_arvalid = 0, s_axil_arready, s_axil_rvalid, s_axil_rready = 0, irq;
  logic q_valid, q_ready, q_kill_on_hit, q_env_buf, r_valid, r_ready, k_valid, k_ready, k_all, cse_idle, cse_err;
  logic [TAG_W-1:0] q_tag, r_tag, k_tag;
  logic [USER_W-1:0] q_user, r_user;
  logic signed [COORD_W-1:0] q_cx, q_cy, q_cz;
  logic [RQ_W-1:0] q_r;
  logic [1:0] r_status;

  ple_top u_top (.*);
  cse_bfm u_cse (.*);
  ple_cse_if_checker u_chk (.*);

  int errs = 0;
  bit started = 1'b0;                                      // STATUS must show busy or done once a run started
  always @(posedge clk) if (rst_n) begin
    if (u_top.go) started <= 1'b1;
    if (started && !u_top.st_busy && !u_top.st_done) begin
      if (errs < 20) $display("ERROR STATUS shows neither busy nor done at %0t", $time);
      errs++;
    end
  end
  task automatic check(input string what, input longint rtl, input longint gold);
    if (rtl != gold) begin
      if (errs < 20) $display("ERROR %s rtl=%0d expected=%0d", what, rtl, gold);
      errs++;
    end
  endtask

  // ---------------------------------------------------------------- AXI-Lite host tasks (random gaps)
  task automatic axw(input logic [11:0] a, input logic [31:0] d, input logic [3:0] be = 4'hF);
    @(negedge clk);
    s_axil_awaddr = a; s_axil_wdata = d; s_axil_wstrb = be;
    s_axil_awvalid = 1; s_axil_wvalid = 1;
    do @(posedge clk); while (!(s_axil_awready && s_axil_wready));
    @(negedge clk);
    s_axil_awvalid = 0; s_axil_wvalid = 0;
    repeat ($urandom % 3) @(negedge clk);
    s_axil_bready = 1;
    do @(posedge clk); while (!s_axil_bvalid);
    @(negedge clk);
    s_axil_bready = 0;
  endtask

  task automatic axr(input logic [11:0] a, output logic [31:0] d);
    @(negedge clk);
    s_axil_araddr = a; s_axil_arvalid = 1;
    do @(posedge clk); while (!s_axil_arready);
    @(negedge clk);
    s_axil_arvalid = 0;
    repeat ($urandom % 3) @(negedge clk);
    s_axil_rready = 1;
    do @(posedge clk); while (!s_axil_rvalid);
    d = s_axil_rdata;
    @(negedge clk);
    s_axil_rready = 0;
  endtask

  function automatic logic [31:0] pair(input shortint a, input shortint b);
    return {b, a};
  endfunction

  task automatic program_run(input shortint sg[14], input int sh[7], input int iters);
    for (int k = 0; k < 4; ++k) begin
      axw(12'h040 + 12'(4 * k), k < 3 ? pair(sg[2*k], sg[2*k+1]) : {16'd0, sg[6]});
      axw(12'h050 + 12'(4 * k), k < 3 ? pair(sg[7+2*k], sg[7+2*k+1]) : {16'd0, sg[13]});
    end
    for (int j = 0; j < 7; ++j) axw(12'h060 + 12'(4 * j), sh[j]);
    axw(12'h028, iters);
  endtask

  task automatic wait_irq(input string what);
    int w = 0;
    while (!irq) begin
      @(negedge clk); w++;
      if (w > 50000000) $fatal(1, "TB_TOP %s: no interrupt (core state %0d)", what, u_top.u_core.state);
    end
  endtask

  initial begin
    int nprob = 4, iters = 3000, runs = 0, solved = 0;
    int plist[4] = '{1, 160, 410, 650};
    int st[8], sh[7];
    shortint sg[14], path[448];
    logic [31:0] v;
    void'($value$plusargs("probs=%d", nprob));
    void'($value$plusargs("iters=%d", iters));
    repeat (3) @(posedge clk);
    rst_n = 1'b1;

    // reset values and read-back, byte strobes
    axr(12'h010, v); check("MODE reset", v, 32'hC);
    axr(12'h014, v); check("range reset", v, 32'd1 << 24);
    axr(12'h028, v); check("max_iter reset", v, 1000000);
    axr(12'h030, v); check("W_MIN x", v, W_MIN_LSB);
    axr(12'h038, v); check("W_MIN z", v, W_MIN_LSB);
    axw(12'h01C, 32'hA5A5_A5A5); axw(12'h01C, 32'h1234_5678, 4'b0101);
    axr(12'h01C, v); check("alpha strobes", v, 32'hA534_A578);
    axw(12'h01C, 32'd429496);
    axw(12'h074, 32'hDEAD_BEEF); axr(12'h074, v); check("SMP_SHIFT[5]", v, 32'hDEAD_BEEF);
    axw(12'h04C, 32'hFFFF_8001); axr(12'h04C, v); check("START word 3 (joint 6 only)", v, 32'h0000_8001);
    axr(12'h0FC, v); check("unmapped", v, 0);

    for (int p = 0; p < nprob && p < 4; ++p) begin
      if (fx_ref_env_load(plist[p], 0) < 0) $fatal(1, "environment build failed");
      void'(fx_ref_solve(plist[p], p, iters, 1 << 24, st, sg, sh, path));
      program_run(sg, sh, iters);
      axw(12'h000, 32'h5);                                 // start + irq enable
      axr(12'h004, v); check("busy after start", v[0], 1);
      axw(12'h014, 32'd7);                                 // ignored while busy
      axw(12'h000, 32'h5);                                 // start while busy: ignored
      wait_irq("run");
      axr(12'h014, v); check("range unchanged by write while busy", v, 32'd1 << 24);
      axr(12'h004, v);
      check("STATUS busy", v[0], 0); check("STATUS done", v[1], 1); check("STATUS result", v[7:4], st[0]);
      axr(12'h080, v); check("iterations", v, st[1]);
      axr(12'h084, v); check("size_start", v[15:0], st[2]); check("size_goal", v[31:16], st[3]);
      axr(12'h088, v); check("path_len", v, st[7]);
      axr(12'h0C4, v); check("edges", v, st[5]);
      axr(12'h0D8, v); check("dd_rejects", v, st[4]);
      axr(12'h0DC, v); check("nn_queries", v, st[6]);
      axr(12'h0C0, v); if (v == 0) begin $display("ERROR cycles counter is 0"); errs++; end
      axr(12'h0C8, v); if (v == 0) begin $display("ERROR poses counter is 0"); errs++; end
      axr(12'h0CC, v); if (v == 0) begin $display("ERROR queries counter is 0"); errs++; end
      for (int i = 0; i < st[7]; ++i)
        for (int k = 0; k < 4; ++k) begin
          axr(12'h400 + 12'(16 * i + 4 * k), v);
          check($sformatf("path[%0d] word %0d", i, k), v,
                k < 3 ? pair(path[i*7+2*k], path[i*7+2*k+1]) : {16'd0, path[i*7+6]});
        end
      axw(12'h008, 32'h1);                                 // clear IRQ
      @(negedge clk); check("irq cleared", irq, 0);
      axr(12'h004, v); check("done stays set after IRQ clear", v[1], 1);
      runs++;
      solved += int'(st[0] == 0);
    end

    // abort mid-run, then a normal run must still match
    if (fx_ref_env_load(plist[0], 0) < 0) $fatal(1, "environment build failed");
    void'(fx_ref_solve(plist[0], 9, iters, 1 << 24, st, sg, sh, path));
    program_run(sg, sh, 1000000);
    axw(12'h000, 32'h5);
    repeat (3000) @(negedge clk);
    if (u_top.busy) begin
      axw(12'h000, 32'h6);                                 // abort, keep irq enabled
      wait_irq("abort");
      axr(12'h004, v); check("ABORTED result", v[7:4], 4); check("busy after abort", v[0], 0);
      axw(12'h008, 32'h1);
    end else begin
      $display("ERROR abort test: run ended before abort"); errs++;
    end
    void'(fx_ref_solve(plist[0], 9, iters, 1 << 24, st, sg, sh, path));
    program_run(sg, sh, iters);
    axw(12'h000, 32'h5);
    axr(12'h004, v); check("done cleared by start", v[1], 0);
    wait_irq("after abort");
    axr(12'h004, v); check("result after abort", v[7:4], st[0]);
    axr(12'h080, v); check("iterations after abort", v, st[1]);
    axr(12'h088, v); check("path_len after abort", v, st[7]);

    // IRQ clear written in the very cycle the next run ends: the new done must stay pending
    axw(12'h008, 32'h1);
    program_run(sg, sh, iters);
    axw(12'h000, 32'h5);
    while (!u_top.done) @(negedge clk);
    s_axil_awaddr = 12'h008; s_axil_wdata = 32'h1; s_axil_wstrb = 4'hF; s_axil_awvalid = 1; s_axil_wvalid = 1;
    #1;
    if (!(u_top.wr_fire && u_top.done)) begin $display("ERROR IRQ test: clear not in the done cycle"); errs++; end
    @(negedge clk);
    s_axil_awvalid = 0; s_axil_wvalid = 0; s_axil_bready = 1;
    do @(posedge clk); while (!s_axil_bvalid);
    @(negedge clk);
    s_axil_bready = 0;
    check("IRQ kept by a done in the clear cycle", irq, 1);
    axw(12'h008, 32'h1);

    // abort while the DIRECT edge runs: taken at the first iteration boundary, no iteration counted
    program_run(sg, sh, 1000000);
    axw(12'h000, 32'h5);
    axw(12'h000, 32'h6);
    wait_irq("abort in DIRECT");
    axr(12'h004, v); check("ABORTED in DIRECT", v[7:4], 4);
    axr(12'h080, v); check("iterations after abort in DIRECT", v, 0);
    axr(12'h088, v); check("path_len after abort in DIRECT", v, 0);
    axw(12'h008, 32'h1);

    repeat (300) @(negedge clk);
    eot = 1'b1;
    repeat (2) @(negedge clk);
    if (u_chk.errors != 0) begin $display("ERROR interface checker reported %0d errors", u_chk.errors); errs++; end
    $display("TB_TOP runs=%0d solved=%0d queries=%0d errors=%0d", runs, solved, u_cse.nq, errs);
    if (errs != 0) $fatal(1, "TB_TOP FAIL");
    $finish;
  end

endmodule
