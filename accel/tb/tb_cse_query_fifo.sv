// Kill-aware query FIFO tests for depth one and non-power-of-two depth three.
module tb_cse_query_fifo #(
    parameter int unsigned D = 3
);
  import ple_cse_pkg::*;
  import cse_cu_pkg::*;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  logic in_valid, in_ready, kill_fire, out_valid, out_ready, out_killed;
  cse_query_meta_t in_payload, out_payload;
  kill_t kill_payload;
  int unsigned errors;

  cse_query_fifo #(.DEPTH(D)) dut (.*);

  function automatic cse_query_meta_t make_query(input int tag_i, input int user_i);
    cse_query_meta_t value;
    value = '0;
    value.query.tag = TAG_W'(tag_i);
    value.query.user = USER_W'(user_i);
    value.birth_gen = KILL_GEN_W'(tag_i);
    return value;
  endfunction

  task automatic push(input int tag_i, input int user_i);
    @(negedge clk);
    in_payload = make_query(tag_i, user_i);
    in_valid = 1'b1;
    do @(posedge clk); while (!in_ready);
    @(negedge clk);
    in_valid = 1'b0;
  endtask

  task automatic send_kill(input int tag_i, input bit all_i);
    @(negedge clk);
    kill_payload = '{all:all_i, tag:TAG_W'(tag_i)};
    kill_fire = 1'b1;
    @(posedge clk);
    @(negedge clk);
    kill_fire = 1'b0;
  endtask

  task automatic pop_check(input int tag_i, input int user_i, input bit killed_i);
    while (!out_valid) @(posedge clk);
    @(negedge clk);
    out_ready = 1'b1;
    @(posedge clk);
    if (out_payload.query.tag != TAG_W'(tag_i) ||
        out_payload.query.user != USER_W'(user_i) || out_killed != killed_i) begin
      $display("TB_CSE_QFIFO ERROR got tag=%0d user=%0h killed=%0d expected %0d/%0h/%0d",
               out_payload.query.tag, out_payload.query.user, out_killed,
               tag_i, user_i, killed_i);
      errors++;
    end
    @(negedge clk);
    out_ready = 1'b0;
  endtask

  task automatic simultaneous_full_reuse(
      input int old_tag, input int old_user, input int new_tag, input int new_user
  );
    @(negedge clk);
    in_payload = make_query(new_tag, new_user);
    in_valid = 1'b1;
    out_ready = 1'b1;
    kill_payload = '{all:1'b1, tag:'0};
    kill_fire = 1'b1;
    @(posedge clk);
    if (out_payload.query.tag != TAG_W'(old_tag) ||
        out_payload.query.user != USER_W'(old_user) || !out_killed) begin
      $display("TB_CSE_QFIFO ERROR full reuse did not return killed old entry");
      errors++;
    end
    @(negedge clk);
    in_valid = 1'b0;
    out_ready = 1'b0;
    kill_fire = 1'b0;
  endtask

  initial begin
    in_valid = 1'b0;
    in_payload = '0;
    kill_fire = 1'b0;
    kill_payload = '0;
    out_ready = 1'b0;
    errors = 0;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;

    if (D == 1) begin
      push(1, 'h11);
      simultaneous_full_reuse(1, 'h11, 2, 'h22);
      pop_check(2, 'h22, 1'b0);
    end else begin
      push(1, 'h11);
      push(2, 'h22);
      push(3, 'h33);
      send_kill(2, 1'b0);
      pop_check(1, 'h11, 1'b0);
      pop_check(2, 'h22, 1'b1);
      pop_check(3, 'h33, 1'b0);

      push(4, 'h44);
      push(5, 'h55);
      push(6, 'h66);
      simultaneous_full_reuse(4, 'h44, 7, 'h77);
      pop_check(5, 'h55, 1'b1);
      pop_check(6, 'h66, 1'b1);
      pop_check(7, 'h77, 1'b0);
    end

    // A query accepted in the same cycle as its matching K is initialized clean.
    @(negedge clk);
    in_payload = make_query(8, 'h88);
    in_valid = 1'b1;
    kill_payload = '{all:1'b0, tag:TAG_W'(8)};
    kill_fire = 1'b1;
    @(posedge clk);
    @(negedge clk);
    in_valid = 1'b0;
    kill_fire = 1'b0;
    pop_check(8, 'h88, 1'b0);

    $display("TB_CSE_QFIFO depth=%0d errors=%0d", D, errors);
    if (errors != 0) $fatal(1, "query FIFO test failed");
    $finish;
  end

endmodule
