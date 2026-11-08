// PLE top: host interface (HIF, AXI-Lite slave + interrupt, register map in design doc section 6) around ple_core.
// Register map (byte addresses, 32-bit words; unlisted addresses read 0, writes ignored):
//   0x000 CTRL   W [0] start (1 = go), [1] abort; R/W [2] irq enable
//   0x004 STATUS R [0] busy, [1] done (cleared by start), [7:4] result, [8] cse_idle, [9] cse_err
//   0x008 IRQ    [0] done pending, write 1 to clear
//   0x010 MODE   [0] env_buf, [1] flat (fine spheres only), [2] dynamic domain, [3] balance
//   0x014 range_h  0x018 DD radius  0x01C alpha  0x020 min radius  0x024 tree ratio  0x028 max iterations
//   0x030-0x038  W_MIN x, y, z (read only, fixed by the FKU build)
//   0x040-0x04C  START, 0x050-0x05C GOAL: word k = {joint 2k+1, joint 2k} (s16 each)
//   0x060-0x078  SMP_SHIFT[0..6]
//   0x080 iterations  0x084 {size_goal, size_start}  0x088 path length
//   0x0C0 cycles  0x0C4 edges  0x0C8 poses  0x0CC env queries  0x0D0 KILLED responses  0x0D4 self pairs (reserved, 0)
//   0x0D8 DD rejects  0x0DC NN queries
//   0x400-0x7FF  PATH: node n at 0x400 + 16 n, word k = {joint 2k+1, joint 2k}
// Writes to configuration registers while busy are ignored (the core needs them stable). Results and counters
// stay valid until the next start. STATUS busy is set from the start write on, done from the cycle the run ends.
// Rules: range_h must be >= 1 (0 divides by zero). start + abort in one write: the abort is ignored.
// Path length is valid only for SOLVED (0 for every other result). The IRQ stays pending across a start until
// cleared; a done in the same cycle as a clear keeps it pending.
module ple_top
  import ple_pkg::*;
  import ple_cse_pkg::*;
#(
    parameter int CAP      = NODE_CAP,
    parameter int PATH_CAP = 64,
    parameter int NSLOT    = 8
) (
    input  logic                      clk,
    input  logic                      rst_n,
    // host: AXI-Lite slave + interrupt
    input  logic [11:0]               s_axil_awaddr,
    input  logic                      s_axil_awvalid,
    output logic                      s_axil_awready,
    input  logic [31:0]               s_axil_wdata,
    input  logic [3:0]                s_axil_wstrb,
    input  logic                      s_axil_wvalid,
    output logic                      s_axil_wready,
    output logic [1:0]                s_axil_bresp,
    output logic                      s_axil_bvalid,
    input  logic                      s_axil_bready,
    input  logic [11:0]               s_axil_araddr,
    input  logic                      s_axil_arvalid,
    output logic                      s_axil_arready,
    output logic [31:0]               s_axil_rdata,
    output logic [1:0]                s_axil_rresp,
    output logic                      s_axil_rvalid,
    input  logic                      s_axil_rready,
    output logic                      irq,
    // Q: query (PLE -> CSE)
    output logic                      q_valid,
    input  logic                      q_ready,
    output logic [TAG_W-1:0]          q_tag,
    output logic [USER_W-1:0]         q_user,
    output logic                      q_kill_on_hit,
    output logic                      q_env_buf,
    output logic signed [COORD_W-1:0] q_cx,
    output logic signed [COORD_W-1:0] q_cy,
    output logic signed [COORD_W-1:0] q_cz,
    output logic [RQ_W-1:0]           q_r,
    // R: response (CSE -> PLE)
    input  logic                      r_valid,
    output logic                      r_ready,
    input  logic [TAG_W-1:0]          r_tag,
    input  logic [USER_W-1:0]         r_user,
    input  logic [1:0]                r_status,
    // K: kill (PLE -> CSE)
    output logic                      k_valid,
    input  logic                      k_ready,
    output logic                      k_all,
    output logic [TAG_W-1:0]          k_tag,
    // status
    input  logic                      cse_idle,
    input  logic                      cse_err
);

  localparam int PW = $clog2(PATH_CAP);

  // ---------------------------------------------------------------- registers
  logic        irq_en, done_flag, irq_pend;
  logic [3:0]  mode;                                     // env_buf, flat, dd, balance
  logic [31:0] r_range, r_radius, r_alpha, r_min_rad, r_ratio, r_max_iter;
  conf_t       r_start, r_goal;
  logic [DIM-1:0][31:0] r_shift;
  logic        go, abort_req;

  logic        busy, done;
  logic [2:0]  status;
  logic [31:0] iterations, dd_rejects, edges, nn_queries, poses;
  logic [15:0] size_start, size_goal;
  logic [7:0]  path_len;
  logic [PW-1:0] pr_idx;
  conf_t       pr_q;
  logic [31:0] c_cycles, c_queries, c_killed;

  // ---------------------------------------------------------------- write channel
  wire  wr_fire = s_axil_awvalid && s_axil_wvalid && !s_axil_bvalid;
  wire  [9:0] wa = s_axil_awaddr[11:2];
  assign s_axil_awready = wr_fire;
  assign s_axil_wready  = wr_fire;
  assign s_axil_bresp   = 2'b00;

  function automatic logic [31:0] merge(input logic [31:0] old, input logic [31:0] d, input logic [3:0] be);
    for (int b = 0; b < 4; ++b) if (be[b]) old[8*b +: 8] = d[8*b +: 8];
    return old;
  endfunction

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      s_axil_bvalid <= 1'b0; go <= 1'b0; abort_req <= 1'b0; irq_en <= 1'b0; irq_pend <= 1'b0; done_flag <= 1'b0;
      mode <= 4'b1100;
      r_range <= 32'd1 << 24; r_radius <= 32'd4 << 24; r_alpha <= 32'd429496; r_min_rad <= 32'd1 << 24;
      r_ratio <= 32'd1 << 16; r_max_iter <= 32'd1000000;
      r_start <= '0; r_goal <= '0; r_shift <= '0;
    end else begin
      go <= 1'b0; abort_req <= 1'b0;
      if (s_axil_bvalid && s_axil_bready) s_axil_bvalid <= 1'b0;
      if (done) done_flag <= 1'b1;
      if (wr_fire) begin
        logic [31:0] d;
        logic [3:0]  be;
        d = s_axil_wdata;
        be = s_axil_wstrb;
        s_axil_bvalid <= 1'b1;
        if (wa == 10'h000 && be[0]) begin
          irq_en <= d[2];
          if (d[0] && !busy) begin go <= 1'b1; done_flag <= 1'b0; end
          if (d[1]) abort_req <= 1'b1;
        end
        if (wa == 10'h002 && be[0] && d[0]) irq_pend <= 1'b0;
        if (!busy) begin
          unique case (wa)
            10'h004: if (be[0]) mode <= d[3:0];
            10'h005: r_range    <= merge(r_range, d, be);
            10'h006: r_radius   <= merge(r_radius, d, be);
            10'h007: r_alpha    <= merge(r_alpha, d, be);
            10'h008: r_min_rad  <= merge(r_min_rad, d, be);
            10'h009: r_ratio    <= merge(r_ratio, d, be);
            10'h00A: r_max_iter <= merge(r_max_iter, d, be);
            default: ;
          endcase
          for (int k = 0; k < 4; ++k) begin
            if (wa == 10'h010 + 10'(k)) begin
              if (be[0]) r_start[2*k][7:0] <= d[7:0];
              if (be[1]) r_start[2*k][15:8] <= d[15:8];
              if (k < 3 && be[2]) r_start[2*k+1][7:0] <= d[23:16];
              if (k < 3 && be[3]) r_start[2*k+1][15:8] <= d[31:24];
            end
            if (wa == 10'h014 + 10'(k)) begin
              if (be[0]) r_goal[2*k][7:0] <= d[7:0];
              if (be[1]) r_goal[2*k][15:8] <= d[15:8];
              if (k < 3 && be[2]) r_goal[2*k+1][7:0] <= d[23:16];
              if (k < 3 && be[3]) r_goal[2*k+1][15:8] <= d[31:24];
            end
          end
          for (int j = 0; j < DIM; ++j) if (wa == 10'h018 + 10'(j)) r_shift[j] <= merge(r_shift[j], d, be);
        end
      end
      if (done) irq_pend <= 1'b1;                       // after the clear: a done in the clear cycle wins
    end
  end
  assign irq = irq_en && irq_pend;
  wire st_busy = busy || go;                            // no gap between the start write and the core busy
  wire st_done = (done_flag || done) && !go;            // no gap between the core idle and done_flag

  // ---------------------------------------------------------------- performance counters (cleared by start)
  always_ff @(posedge clk) begin
    if (!rst_n) begin c_cycles <= '0; c_queries <= '0; c_killed <= '0; end
    else if (go) begin c_cycles <= '0; c_queries <= '0; c_killed <= '0; end
    else begin
      if (busy) c_cycles <= c_cycles + 32'd1;
      if (q_valid && q_ready) c_queries <= c_queries + 32'd1;
      if (r_valid && r_ready && r_status == ST_KILLED) c_killed <= c_killed + 32'd1;
    end
  end

  // ---------------------------------------------------------------- read channel (one-cycle registered)
  wire [9:0] ra = s_axil_araddr[11:2];
  assign s_axil_arready = !s_axil_rvalid;
  assign s_axil_rresp   = 2'b00;
  assign pr_idx = PW'(s_axil_araddr[PW+3:4]);

  function automatic logic [31:0] pair(input conf_t q, input int k);
    return (k < 3) ? {q[2*k+1], q[2*k]} : {16'd0, q[2*k]};
  endfunction

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      s_axil_rvalid <= 1'b0; s_axil_rdata <= '0;
    end else begin
      if (s_axil_rvalid && s_axil_rready) s_axil_rvalid <= 1'b0;
      if (s_axil_arvalid && s_axil_arready) begin
        logic [31:0] v;
        v = '0;
        if (ra[9:8] == 2'b01) begin                       // 0x400-0x7FF path window
          if (32'(ra[7:2]) < PATH_CAP) v = pair(pr_q, int'(ra[1:0]));
        end else begin
          unique case (ra)
            10'h000: v = {29'd0, irq_en, 2'd0};
            10'h001: v = {22'd0, cse_err, cse_idle, 1'b0, status, 2'd0, st_done, st_busy};
            10'h002: v = {31'd0, irq_pend};
            10'h004: v = {28'd0, mode};
            10'h005: v = r_range;
            10'h006: v = r_radius;
            10'h007: v = r_alpha;
            10'h008: v = r_min_rad;
            10'h009: v = r_ratio;
            10'h00A: v = r_max_iter;
            10'h00C, 10'h00D, 10'h00E: v = W_MIN_LSB;
            10'h020: v = iterations;
            10'h021: v = {size_goal, size_start};
            10'h022: v = {24'd0, path_len};
            10'h030: v = c_cycles;
            10'h031: v = edges;
            10'h032: v = poses;
            10'h033: v = c_queries;
            10'h034: v = c_killed;
            10'h036: v = dd_rejects;
            10'h037: v = nn_queries;
            default: begin
              for (int k = 0; k < 4; ++k) begin
                if (ra == 10'h010 + 10'(k)) v = pair(r_start, k);
                if (ra == 10'h014 + 10'(k)) v = pair(r_goal, k);
              end
              for (int j = 0; j < DIM; ++j) if (ra == 10'h018 + 10'(j)) v = r_shift[j];
            end
          endcase
        end
        s_axil_rdata <= v; s_axil_rvalid <= 1'b1;
      end
    end
  end

  logic unused;
  assign unused = ^{s_axil_awaddr[1:0], s_axil_araddr[1:0]};

  // ---------------------------------------------------------------- planner
  ple_core #(.CAP(CAP), .PATH_CAP(PATH_CAP), .NSLOT(NSLOT)) u_core (
      .clk, .rst_n, .cfg_start(r_start), .cfg_goal(r_goal), .cfg_range_h(r_range), .cfg_radius(r_radius),
      .cfg_alpha(r_alpha), .cfg_min_radius(r_min_rad), .cfg_tree_ratio(r_ratio), .cfg_dd(mode[2]),
      .cfg_balance(mode[3]), .cfg_flat(mode[1]), .cfg_env_buf(mode[0]), .cfg_max_iter(r_max_iter),
      .cfg_shift(r_shift), .go, .abort_req, .busy, .done, .status, .iterations, .dd_rejects, .edges, .nn_queries,
      .poses, .size_start, .size_goal, .path_len, .pr_idx, .pr_q, .q_valid, .q_ready, .q_tag, .q_user,
      .q_kill_on_hit, .q_env_buf, .q_cx, .q_cy, .q_cz, .q_r, .r_valid, .r_ready, .r_tag, .r_user, .r_status,
      .k_valid, .k_ready, .k_all, .k_tag);

endmodule
