// Full-array response-backpressure saturation and mid-flight reset recovery.
module tb_cse_control;
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

  int phase, accepted, received, errors;
  bit seen[256];

  cse_system #(.N_CU(4), .INPUT_FIFO_DEPTH(4), .RESPONSE_FIFO_DEPTH(2)) dut (.*);
  ple_cse_if_checker #(.AUTO_KILL(1'b0)) u_checker (.*);

  always @(posedge clk) begin
    if (rst_n && r_valid && r_ready) begin
      if (phase == 1) begin
        int index;
        index = int'(r_user);
        if (index >= accepted || seen[index] || r_status != ST_MISS) begin
          $display("TB_CSE_CONTROL ERROR backpressure response user=%0d status=%0d", r_user, r_status);
          errors++;
        end else begin
          seen[index] = 1'b1;
        end
        received++;
      end else if (phase == 2) begin
        if (r_tag != 4'd9 || r_user != 16'hbeef || r_status != ST_MISS) begin
          $display("TB_CSE_CONTROL ERROR stale/wrong post-reset response %0d/%0h/%0d",
                   r_tag, r_user, r_status);
          errors++;
        end
        received++;
      end else begin
        $display("TB_CSE_CONTROL ERROR response in unexpected phase");
        errors++;
      end
    end
  end

  task automatic send_one(
      input int tag_i, input int user_i, input int cx_i, input int cy_i,
      input int cz_i, input int radius_i
  );
    @(negedge clk);
    q_tag = TAG_W'(tag_i);
    q_user = USER_W'(user_i);
    q_cx = COORD_W'(cx_i);
    q_cy = COORD_W'(cy_i);
    q_cz = COORD_W'(cz_i);
    q_r = RQ_W'(radius_i);
    q_valid = 1'b1;
    do @(posedge clk); while (!q_ready);
    @(negedge clk);
    q_valid = 1'b0;
  endtask

  initial begin : p_test
    int stalled_cycles, watchdog;
    bit accepted_now;

    q_valid = 1'b0;
    q_tag = 4'd1;
    q_user = '0;
    q_kill_on_hit = 1'b0;
    q_env_buf = 1'b0;
    q_cx = -18'sd10000;
    q_cy = -18'sd10000;
    q_cz = -18'sd10000;
    q_r = '0;
    r_ready = 1'b0;
    k_valid = 1'b0;
    k_all = 1'b0;
    k_tag = '0;
    eot = 1'b0;
    phase = 0;
    accepted = 0;
    received = 0;
    errors = 0;
    for (int i = 0; i < 256; ++i) seen[i] = 1'b0;

    repeat (4) @(posedge clk);
    rst_n = 1'b1;

    // Hold PLE R blocked and offer queries until all internal capacity propagates back to Q.
    phase = 1;
    stalled_cycles = 0;
    @(negedge clk);
    q_valid = 1'b1;
    while (stalled_cycles < 5) begin
      @(posedge clk);
      accepted_now = q_ready;
      if (accepted_now) begin
        accepted++;
        stalled_cycles = 0;
      end else begin
        stalled_cycles++;
      end
      @(negedge clk);
      if (accepted_now) q_user = USER_W'(accepted);
      if (accepted >= 255) $fatal(1, "unexpectedly failed to backpressure Q");
    end
    if (accepted < 6) $fatal(1, "too little work accepted before saturation");
    r_ready = 1'b1;
    do @(posedge clk); while (!q_ready);
    accepted++;
    @(negedge clk);
    q_valid = 1'b0;

    watchdog = 0;
    while (received < accepted || !cse_idle) begin
      @(posedge clk);
      watchdog++;
      if (watchdog > 10000) $fatal(1, "backpressure drain timeout");
    end
    if (dut.u_cse.stat_response_peak != 2 || dut.u_cse.stat_response_full_cycles == 0) begin
      $display("TB_CSE_CONTROL ERROR response FIFO was not observed full peak=%0d full_cycles=%0d",
               dut.u_cse.stat_response_peak, dut.u_cse.stat_response_full_cycles);
      errors++;
    end

    // Start a nontrivial query, reset it mid-flight, then prove no stale response escapes.
    phase = 0;
    send_one(8, 'h800, 20000, 25000, 20000, 8192);
    repeat (8) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b0;
    repeat (3) @(posedge clk);
    if (r_valid) begin
      $display("TB_CSE_CONTROL ERROR r_valid during reset");
      errors++;
    end
    @(negedge clk);
    rst_n = 1'b1;
    if (!cse_idle) begin
      $display("TB_CSE_CONTROL ERROR not idle after reset");
      errors++;
    end

    phase = 2;
    received = 0;
    send_one(9, 'hbeef, -10000, -10000, -10000, 0);
    watchdog = 0;
    while (received < 1 || !cse_idle) begin
      @(posedge clk);
      watchdog++;
      if (watchdog > 1000) $fatal(1, "post-reset query timeout");
    end
    @(negedge clk);
    eot = 1'b1;
    repeat (2) @(posedge clk);
    $display("TB_CSE_CONTROL saturated_accepts=%0d errors=%0d checker_errors=%0d",
             accepted, errors, u_checker.errors);
    if (errors != 0 || u_checker.errors != 0 || cse_err) $fatal(1, "control test failed");
    $finish;
  end

  logic unused;
  assign unused = k_ready;

endmodule
