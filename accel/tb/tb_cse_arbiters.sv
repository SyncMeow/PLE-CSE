// Dispatcher target locking/cancel routing and receiver round-robin/backpressure tests.
module tb_cse_arbiters;
  import ple_cse_pkg::*;
  import cse_cu_pkg::*;

  localparam int N_CU = 3;
  localparam int N_SRC = 4;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  logic d_in_valid, d_in_ready, d_in_killed;
  cse_query_meta_t d_in_payload;
  logic [(1<<TAG_W)-1:0][KILL_GEN_W-1:0] kill_gen;
  logic [N_CU-1:0] cu_valid, cu_ready;
  cse_query_meta_t cu_payload[N_CU];
  logic cancel_valid, cancel_ready;
  resp_t cancel_payload;
  logic dispatcher_waiting;

  logic [N_SRC-1:0] source_valid, source_ready;
  resp_t source_payload[N_SRC];
  logic receiver_valid, receiver_ready;
  resp_t receiver_payload;

  int dispatch_count, cancel_count, receive_count, errors;
  int dispatch_index[8], receive_user[8];

  cse_dispatcher #(.N_CU(N_CU)) u_dispatcher (
      .clk, .rst_n,
      .in_valid(d_in_valid), .in_ready(d_in_ready), .in_payload(d_in_payload),
      .in_killed(d_in_killed), .kill_gen,
      .cu_valid, .cu_ready, .cu_payload,
      .cancel_valid, .cancel_ready, .cancel_payload, .waiting_for_cu(dispatcher_waiting)
  );

  cse_receiver #(.N_SRC(N_SRC)) u_receiver (
      .clk, .rst_n, .source_valid, .source_ready, .source_payload,
      .out_valid(receiver_valid), .out_ready(receiver_ready), .out_payload(receiver_payload)
  );

  function automatic cse_query_meta_t make_query(input int tag_i, input int user_i);
    cse_query_meta_t value;
    value = '0;
    value.query.tag = TAG_W'(tag_i);
    value.query.user = USER_W'(user_i);
    return value;
  endfunction

  task automatic send_dispatch(input int tag_i, input int user_i, input bit killed_i);
    @(negedge clk);
    d_in_payload = make_query(tag_i, user_i);
    d_in_killed = killed_i;
    d_in_valid = 1'b1;
    do @(posedge clk); while (!d_in_ready);
    @(negedge clk);
    d_in_valid = 1'b0;
    d_in_killed = 1'b0;
  endtask

  task automatic wait_dispatch_count(input int target);
    int watchdog;
    watchdog = 0;
    while (dispatch_count < target) begin
      @(posedge clk);
      watchdog++;
      if (watchdog > 100) $fatal(1, "dispatcher timeout");
    end
  endtask

  always @(posedge clk) begin
    for (int i = 0; i < N_CU; ++i)
      if (cu_valid[i] && cu_ready[i]) begin
        dispatch_index[dispatch_count] = i;
        dispatch_count++;
      end
    if (cancel_valid && cancel_ready) cancel_count++;

    if (receiver_valid && receiver_ready) begin
      receive_user[receive_count] = int'(receiver_payload.user);
      receive_count++;
    end
    for (int i = 0; i < N_SRC; ++i)
      if (source_valid[i] && source_ready[i]) source_valid[i] <= 1'b0;
  end

  initial begin
    d_in_valid = 1'b0;
    d_in_payload = '0;
    d_in_killed = 1'b0;
    kill_gen = '0;
    cu_ready = '0;
    cancel_ready = 1'b1;
    source_valid = '0;
    for (int i = 0; i < N_SRC; ++i)
      source_payload[i] = '{tag:TAG_W'(i), user:USER_W'(i), status:ST_MISS};
    receiver_ready = 1'b0;
    dispatch_count = 0;
    cancel_count = 0;
    receive_count = 0;
    errors = 0;

    repeat (3) @(posedge clk);
    rst_n = 1'b1;

    // Select CU0, then remove ready before the first valid handshake: target must stay locked.
    send_dispatch(1, 'h101, 1'b0);
    @(negedge clk);
    cu_ready = 3'b001;
    @(posedge clk);
    @(negedge clk);
    cu_ready = '0;
    repeat (2) begin
      @(posedge clk);
      if (cu_valid != 3'b001 || cu_payload[0].query.user != 16'h101) begin
        $display("TB_CSE_ARBITERS ERROR dispatcher target/payload did not hold");
        errors++;
      end
    end
    @(negedge clk);
    cu_ready = 3'b001;
    wait_dispatch_count(1);

    // With all CUs ready, round-robin must continue at CU1 then CU2.
    cu_ready = 3'b111;
    send_dispatch(2, 'h202, 1'b0);
    wait_dispatch_count(2);
    send_dispatch(3, 'h303, 1'b0);
    wait_dispatch_count(3);
    if (dispatch_index[0] != 0 || dispatch_index[1] != 1 || dispatch_index[2] != 2) begin
      $display("TB_CSE_ARBITERS ERROR dispatcher RR order %0d,%0d,%0d",
               dispatch_index[0], dispatch_index[1], dispatch_index[2]);
      errors++;
    end

    // FIFO-killed and generation-killed unselected work must use the bypass.
    cu_ready = '0;
    send_dispatch(4, 'h404, 1'b1);
    while (cancel_count < 1) @(posedge clk);
    send_dispatch(5, 'h505, 1'b0);
    @(negedge clk);
    kill_gen[5] = 1;
    while (cancel_count < 2) @(posedge clk);

    // Once CU valid is shown, a later kill cannot withdraw or reroute it.
    send_dispatch(6, 'h606, 1'b0);
    @(negedge clk);
    cu_ready = 3'b001;
    @(posedge clk);
    @(negedge clk);
    cu_ready = '0;
    kill_gen[6] = 1;
    @(posedge clk);
    if (cu_valid != 3'b001 || cancel_valid) begin
      $display("TB_CSE_ARBITERS ERROR selected query was rerouted after kill");
      errors++;
    end
    @(negedge clk);
    cu_ready = 3'b001;
    wait_dispatch_count(4);

    // Receiver locks source zero under backpressure, then serves all four sources fairly.
    @(negedge clk);
    source_valid = '1;
    repeat (2) @(posedge clk);
    if (!receiver_valid || receiver_payload.user != 0) begin
      $display("TB_CSE_ARBITERS ERROR receiver did not lock source zero");
      errors++;
    end
    repeat (2) begin
      @(posedge clk);
      if (!receiver_valid || receiver_payload.user != 0) begin
        $display("TB_CSE_ARBITERS ERROR receiver payload changed under backpressure");
        errors++;
      end
    end
    @(negedge clk);
    receiver_ready = 1'b1;
    while (receive_count < 4) @(posedge clk);
    for (int i = 0; i < 4; ++i)
      if (receive_user[i] != i) begin
        $display("TB_CSE_ARBITERS ERROR receiver order[%0d]=%0d", i, receive_user[i]);
        errors++;
      end

    $display("TB_CSE_ARBITERS dispatches=%0d cancels=%0d receives=%0d errors=%0d",
             dispatch_count, cancel_count, receive_count, errors);
    if (errors != 0) $fatal(1, "arbiter test failed");
    $finish;
  end

  logic unused_cancel;
  assign unused_cancel = ^cancel_payload ^ dispatcher_waiting;

endmodule
