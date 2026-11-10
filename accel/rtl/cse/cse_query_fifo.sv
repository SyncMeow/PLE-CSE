// Query FIFO with per-entry kill metadata and top-level birth generation.
module cse_query_fifo
  import ple_cse_pkg::*;
  import cse_cu_pkg::*;
#(
    parameter int unsigned DEPTH = 8,
    parameter bit KILL_ENABLE = 1'b1
) (
    input  logic             clk,
    input  logic             rst_n,
    input  logic             in_valid,
    output logic             in_ready,
    input  cse_query_meta_t  in_payload,
    input  logic             kill_fire,
    input  kill_t            kill_payload,
    output logic             out_valid,
    input  logic             out_ready,
    output cse_query_meta_t  out_payload,
    output logic             out_killed
);

  localparam int PTR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH);
  localparam int CNT_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH + 1);

  cse_query_meta_t payload_mem[DEPTH];
  logic [DEPTH-1:0] occupied, killed;
  logic [PTR_W-1:0] rd_ptr, wr_ptr;
  logic [CNT_W-1:0] count;
  logic push, pop, head_kill_now;

  if (DEPTH < 1) begin : g_bad_depth
    $error("cse_query_fifo: DEPTH must be at least one");
  end

  assign out_valid = (count != 0);
  assign out_payload = payload_mem[rd_ptr];
  assign head_kill_now = KILL_ENABLE && kill_fire && out_valid &&
                         (kill_payload.all || kill_payload.tag == out_payload.query.tag);
  assign out_killed = killed[rd_ptr] || head_kill_now;
  assign in_ready = (count < CNT_W'(DEPTH)) || (out_valid && out_ready);
  assign push = in_valid && in_ready;
  assign pop  = out_valid && out_ready;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      occupied <= '0;
      killed   <= '0;
      rd_ptr   <= '0;
      wr_ptr   <= '0;
      count    <= '0;
    end else begin
      if (KILL_ENABLE && kill_fire)
        for (int i = 0; i < DEPTH; ++i)
          if (occupied[i] && (kill_payload.all || kill_payload.tag == payload_mem[i].query.tag))
            killed[i] <= 1'b1;

      if (pop) begin
        occupied[rd_ptr] <= 1'b0;
        killed[rd_ptr]   <= 1'b0;
        rd_ptr <= (rd_ptr == PTR_W'(DEPTH-1)) ? '0 : rd_ptr + 1'b1;
      end
      if (push) begin
        payload_mem[wr_ptr] <= in_payload;
        occupied[wr_ptr]    <= 1'b1;
        killed[wr_ptr]      <= 1'b0;
        wr_ptr <= (wr_ptr == PTR_W'(DEPTH-1)) ? '0 : wr_ptr + 1'b1;
      end

      unique case ({push, pop})
        2'b10: count <= count + 1'b1;
        2'b01: count <= count - 1'b1;
        default: count <= count;
      endcase
    end
  end

endmodule
