// Replay one golden CSE trace through the single-CU RTL with response backpressure.
module tb_cse_trace;
  import ple_cse_pkg::*;

  localparam int MAX_QUERY = 200000;

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
  expected_t expected[MAX_QUERY];
  int unsigned n_sent, n_recv, errors;
  logic [15:0] lfsr;

  cse_top dut (.*);
  ple_cse_if_checker #(.AUTO_KILL(1'b0)) u_checker (.*);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      lfsr   <= 16'h1;
      r_ready <= 1'b0;
    end else begin
      lfsr   <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
      r_ready <= lfsr[0] | lfsr[3];
    end
  end

  always @(posedge clk) begin
    if (rst_n && r_valid && r_ready) begin
      if (n_recv >= n_sent) begin
        $display("TB_CSE_TRACE ERROR response without expected entry");
        errors++;
      end else begin
        if (r_tag != expected[n_recv].tag || r_user != expected[n_recv].user ||
            r_status != expected[n_recv].status) begin
          if (errors < 20)
            $display("TB_CSE_TRACE ERROR n=%0d got tag=%0d user=%0h status=%0d expected tag=%0d user=%0h status=%0d",
                     n_recv, r_tag, r_user, r_status, expected[n_recv].tag,
                     expected[n_recv].user, expected[n_recv].status);
          errors++;
        end
        n_recv++;
      end
    end
  end

  task automatic send_query(
      input int tag_i, input int user_i, input int koh_i, input int buf_i,
      input int cx_i, input int cy_i, input int cz_i, input int radius_i,
      input int hit_i
  );
    if (n_sent >= MAX_QUERY) $fatal(1, "too many trace queries");
    expected[n_sent].tag = TAG_W'(tag_i);
    expected[n_sent].user = USER_W'(user_i);
    expected[n_sent].status = (hit_i != 0) ? ST_HIT : ST_MISS;
    @(negedge clk);
    q_tag = TAG_W'(tag_i);
    q_user = USER_W'(user_i);
    q_kill_on_hit = koh_i[0];
    q_env_buf = buf_i[0];
    q_cx = COORD_W'(cx_i);
    q_cy = COORD_W'(cy_i);
    q_cz = COORD_W'(cz_i);
    q_r = RQ_W'(radius_i);
    q_valid = 1'b1;
    do @(posedge clk); while (!q_ready);
    n_sent++;
    @(negedge clk);
    q_valid = 1'b0;
  endtask

  initial begin : p_test
    string query_file, line;
    int fd, got;
    int tag_i, user_i, koh_i, buf_i, cx_i, cy_i, cz_i, radius_i, hit_i;
    int unsigned watchdog;

    q_valid = 1'b0;
    q_tag = '0;
    q_user = '0;
    q_kill_on_hit = 1'b0;
    q_env_buf = 1'b0;
    q_cx = '0;
    q_cy = '0;
    q_cz = '0;
    q_r = '0;
    k_valid = 1'b0;
    k_all = 1'b0;
    k_tag = '0;
    eot = 1'b0;
    n_sent = 0;
    n_recv = 0;
    errors = 0;

    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    if (!$value$plusargs("queries=%s", query_file)) $fatal(1, "+queries is required");
    fd = $fopen(query_file, "r");
    if (fd == 0) $fatal(1, "cannot open %s", query_file);
    got = $fgets(line, fd);
    while (!$feof(fd)) begin
      got = $fscanf(fd, "%d %d %d %d %d %d %d %d %d\n",
                    tag_i, user_i, koh_i, buf_i, cx_i, cy_i, cz_i, radius_i, hit_i);
      if (got == 9)
        send_query(tag_i, user_i, koh_i, buf_i, cx_i, cy_i, cz_i, radius_i, hit_i);
      else if (!$feof(fd))
        $fatal(1, "malformed query trace near entry %0d", n_sent);
    end
    $fclose(fd);

    watchdog = 0;
    while ((n_recv != n_sent) || !cse_idle) begin
      @(posedge clk);
      watchdog++;
      if (watchdog > 200000) $fatal(1, "drain timeout sent=%0d recv=%0d", n_sent, n_recv);
    end
    @(negedge clk);
    eot = 1'b1;
    repeat (2) @(posedge clk);
    $display("TB_CSE_TRACE queries=%0d responses=%0d errors=%0d checker_errors=%0d cse_err=%0d",
             n_sent, n_recv, errors, u_checker.errors, cse_err);
    if (errors != 0 || u_checker.errors != 0 || cse_err) $fatal(1, "trace test failed");
    $finish;
  end

  logic unused;
  assign unused = ^{k_ready, got_unused};
  logic got_unused;
  assign got_unused = 1'b0;

endmodule
