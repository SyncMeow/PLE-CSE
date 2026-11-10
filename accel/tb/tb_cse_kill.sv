// Directed kill-generation, same-cycle Q/K, K(all), error, and held-response tests.
module tb_cse_kill #(
    parameter bit KE = 1'b1
);
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

  typedef struct packed {
    logic [TAG_W-1:0] tag;
    logic [USER_W-1:0] user;
    status_e status;
  } expected_t;
  expected_t expected[16];
  bit expected_seen[16];
  int unsigned n_expected, n_recv, errors;

  cse_system #(.KILL_ENABLE(KE)) dut (.*);
  ple_cse_if_checker #(.AUTO_KILL(1'b0)) u_checker (.*);

  task automatic expect_response(input int tag_i, input int user_i, input status_e status_i);
    expected[n_expected] = '{TAG_W'(tag_i), USER_W'(user_i), status_i};
    expected_seen[n_expected] = 1'b0;
    n_expected++;
  endtask

  task automatic send_query(
      input int tag_i, input int user_i, input int buf_i,
      input int cx_i, input int cy_i, input int cz_i, input int radius_i
  );
    @(negedge clk);
    q_tag = TAG_W'(tag_i);
    q_user = USER_W'(user_i);
    q_kill_on_hit = 1'b0;
    q_env_buf = buf_i[0];
    q_cx = COORD_W'(cx_i);
    q_cy = COORD_W'(cy_i);
    q_cz = COORD_W'(cz_i);
    q_r = RQ_W'(radius_i);
    q_valid = 1'b1;
    do @(posedge clk); while (!q_ready);
    @(negedge clk);
    q_valid = 1'b0;
  endtask

  task automatic send_kill(input int tag_i, input bit all_i);
    @(negedge clk);
    k_tag = TAG_W'(tag_i);
    k_all = all_i;
    k_valid = 1'b1;
    do @(posedge clk); while (!k_ready);
    @(negedge clk);
    k_valid = 1'b0;
  endtask

  task automatic wait_received(input int target);
    int unsigned watchdog;
    watchdog = 0;
    while (n_recv < target) begin
      @(posedge clk);
      watchdog++;
      if (watchdog > 70000) $fatal(1, "response timeout target=%0d recv=%0d", target, n_recv);
    end
  endtask

  always @(posedge clk) begin
    if (rst_n && r_valid && r_ready) begin
      int match;
      match = -1;
      for (int i = 0; i < 16; ++i)
        if (i < n_expected && !expected_seen[i] && r_tag == expected[i].tag &&
            r_user == expected[i].user && r_status == expected[i].status && match < 0)
          match = i;
      if (match < 0) begin
        $display("TB_CSE_KILL ERROR unexpected response %0d/%0h/%0d", r_tag, r_user, r_status);
        errors++;
      end else begin
        expected_seen[match] = 1'b1;
      end
      n_recv++;
    end
  end

  initial begin : p_test
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
    n_expected = 0;
    n_recv = 0;
    errors = 0;

    repeat (4) @(posedge clk);
    rst_n = 1'b1;

    // An established query is canceled while still near the front of the pipeline.
    expect_response(1, 'h101, KE ? ST_KILLED : ST_MISS);
    send_query(1, 'h101, 0, 25192, 20743, 28806, 435);
    send_kill(1, 1'b0);
    wait_received(1);

    // A matching K and Q handshake together: the new query is born after this generation bump.
    expect_response(2, 'h202, ST_MISS);
    @(negedge clk);
    q_tag = 2; q_user = 'h202; q_env_buf = 1'b0;
    q_cx = -18'sd10000; q_cy = -18'sd10000; q_cz = -18'sd10000; q_r = '0;
    q_valid = 1'b1;
    k_tag = 2; k_all = 1'b0; k_valid = 1'b1;
    do @(posedge clk); while (!(q_ready && k_ready));
    @(negedge clk);
    q_valid = 1'b0;
    k_valid = 1'b0;
    wait_received(2);

    // A kill with no work cannot poison a later query of the same tag.
    send_kill(3, 1'b0);
    expect_response(3, 'h303, ST_MISS);
    send_query(3, 'h303, 0, -10000, -10000, -10000, 0);
    wait_received(3);

    // Once a true response is displayed, a later kill cannot rewrite it under backpressure.
    r_ready = 1'b0;
    expect_response(4, 'h404, ST_MISS);
    send_query(4, 'h404, 0, -10000, -10000, -10000, 0);
    while (!r_valid) @(posedge clk);
    if (r_status != ST_MISS) begin
      $display("TB_CSE_KILL ERROR held response was not MISS before kill");
      errors++;
    end
    send_kill(4, 1'b0);
    repeat (3) begin
      @(posedge clk);
      if (!r_valid || r_status != ST_MISS || r_tag != 4 || r_user != 'h404) begin
        $display("TB_CSE_KILL ERROR held response changed after kill");
        errors++;
      end
    end
    @(negedge clk);
    r_ready = 1'b1;
    wait_received(4);

    // K(all) covers both queries already resident in different pipeline positions.
    expect_response(5, 'h505, KE ? ST_KILLED : ST_MISS);
    expect_response(6, 'h606, KE ? ST_KILLED : ST_MISS);
    send_query(5, 'h505, 0, 25192, 20743, 28806, 435);
    send_query(6, 'h606, 0, 25108, 18594, 30094, 435);
    send_kill(0, 1'b1);
    wait_received(6);

    // Buffer 1 has no loaded header and must report ERROR and set the sticky flag.
    expect_response(7, 'h707, ST_ERROR);
    send_query(7, 'h707, 1, 0, 0, 0, 0);
    wait_received(7);
    if (!cse_err) begin
      $display("TB_CSE_KILL ERROR cse_err did not become sticky");
      errors++;
    end

    while (!cse_idle) @(posedge clk);
    @(negedge clk);
    eot = 1'b1;
    repeat (2) @(posedge clk);
    $display("TB_CSE_KILL responses=%0d errors=%0d checker_errors=%0d", n_recv, errors,
             u_checker.errors);
    if (n_recv != n_expected || errors != 0 || u_checker.errors != 0)
      $fatal(1, "kill test failed");
    $finish;
  end

endmodule
