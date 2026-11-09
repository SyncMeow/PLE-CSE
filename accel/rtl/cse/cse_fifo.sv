// Small synthesizable ready/valid FIFO used between CSE CU stages.
module cse_fifo #(
    parameter type PAYLOAD_T = logic,
    parameter int unsigned DEPTH = 2
) (
    input  logic     clk,
    input  logic     rst_n,
    input  logic     in_valid,
    output logic     in_ready,
    input  PAYLOAD_T in_payload,
    output logic     out_valid,
    input  logic     out_ready,
    output PAYLOAD_T out_payload
);

  localparam int PTR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH);
  localparam int CNT_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH + 1);

  PAYLOAD_T mem[DEPTH];
  logic [PTR_W-1:0] rd_ptr, wr_ptr;
  logic [CNT_W-1:0] count;
  logic push, pop;

  if (DEPTH < 1) begin : g_bad_depth
    $error("cse_fifo: DEPTH must be at least one");
  end

  assign out_valid   = (count != 0);
  assign out_payload = mem[rd_ptr];
  assign in_ready    = (count < CNT_W'(DEPTH)) || (out_valid && out_ready);
  assign push        = in_valid && in_ready;
  assign pop         = out_valid && out_ready;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      rd_ptr <= '0;
      wr_ptr <= '0;
      count  <= '0;
    end else begin
      if (push) begin
        mem[wr_ptr] <= in_payload;
        wr_ptr <= (wr_ptr == PTR_W'(DEPTH-1)) ? '0 : wr_ptr + 1'b1;
      end
      if (pop) rd_ptr <= (rd_ptr == PTR_W'(DEPTH-1)) ? '0 : rd_ptr + 1'b1;
      unique case ({push, pop})
        2'b10: count <= count + 1'b1;
        2'b01: count <= count - 1'b1;
        default: count <= count;
      endcase
    end
  end

endmodule
