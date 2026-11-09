// The CPU driver (sw/ple_drv.c) against ple_top: tb/drv_harness.c runs a test program on the real driver
// code; each register access it makes is performed here over AXI-Lite (random gaps). The harness compares
// results and paths with golden fx_solve and reports its error count; the interface checker watches the CSE side.
module tb_drv;
  import ple_pkg::*;
  import ple_cse_pkg::*;

  import "DPI-C" context function int drv_step(input int rdata, output int op, output int addr, output int data);

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

  task automatic axw(input logic [11:0] a, input logic [31:0] d);
    @(negedge clk);
    s_axil_awaddr = a; s_axil_wdata = d; s_axil_wstrb = 4'hF;
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

  initial begin
    int op, addr, data, rdata = 0, n_rd = 0, n_wr = 0, n_irq = 0;
    logic [31:0] v;
    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    forever begin
      void'(drv_step(rdata, op, addr, data));
      case (op)
        0: begin axr(12'(addr), v); rdata = int'(v); n_rd++; end
        1: begin axw(12'(addr), 32'(data)); n_wr++; end
        2: begin
          int w = 0;
          while (!irq) begin
            @(negedge clk);
            w = w + 1;
            if (w > 50000000) $fatal(1, "TB_DRV no interrupt");
          end
          n_irq++;
        end
        3: repeat (data) @(negedge clk);
        default: begin
          repeat (300) @(negedge clk);
          eot = 1'b1;
          repeat (2) @(negedge clk);
          if (u_chk.errors != 0) begin $display("ERROR interface checker reported %0d errors", u_chk.errors); data++; end
          $display("TB_DRV reads=%0d writes=%0d irq_waits=%0d errors=%0d", n_rd, n_wr, n_irq, data);
          if (data != 0) $fatal(1, "TB_DRV FAIL");
          $finish;
        end
      endcase
    end
  end

endmodule
