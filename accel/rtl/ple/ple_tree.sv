// TREE: node memory of both RRT-Connect trees (golden fx_plan.c tree_t / tree_insert).
// Node i of tree t lives in lane i%K, row t*ROWS + i/K. Each lane is stored twice (port A copy for the
// controller read, port B copy for the NNU row read), so every array is one write + one read port and
// maps to simple dual-port BRAM. PARENT and RAD: one entry per node. All reads: 1-cycle latency,
// read-first (a read in the cycle of a write to the same entry returns the old value).
// Controller contract (checked by simulation assertions):
//  - an insert and a RAD update to a different node must not share a cycle (single RAD write port);
//  - no read of an entry in the cycle it is written (BRAM collision behaviour is not modelled);
//  - no insert into a tree while the NNU scans it. The controller keeps the node counts.
module ple_tree
  import ple_pkg::*;
#(
    parameter int K   = 4,                               // nodes per JOINT row (NNU PEs), power of 2, >= 2
    parameter int CAP = NODE_CAP,                        // nodes per tree (SRAM size), power of 2, >= K
    parameter int IW  = $clog2(CAP)                      // node index width (derived, do not override)
) (
    input  logic                         clk,
    // insert (node index chosen by the controller = current count); also sets RAD to infinity
    input  logic                         wr_en,
    input  logic                         wr_tree,
    input  logic [IW-1:0]             wr_idx,
    input  conf_t                        wr_q,
    input  logic [IW-1:0]             wr_parent,
    // RAD update (dynamic domain); an insert in the same cycle wins
    input  logic                         rad_we,
    input  logic                         rad_tree,
    input  logic [IW-1:0]             rad_idx,
    input  logic [RAD_W-1:0]             rad_wdata,
    // controller read of one node
    input  logic                         rd_en,
    input  logic                         rd_tree,
    input  logic [IW-1:0]             rd_idx,
    output conf_t                        rd_q,
    output logic [IW-1:0]             rd_parent,
    output logic [RAD_W-1:0]             rd_rad,
    // NNU row read
    input  logic                         nn_en,
    input  logic                         nn_tree,
    input  logic [IW-1:0]             nn_row,          // row within the tree (< CAP / K)
    output conf_t [K-1:0]                nn_q
);

  localparam int ROWS = CAP / K;
  localparam int LW   = $clog2(K);
  localparam int RW   = $clog2(ROWS);

  if (K < 2 || (1 << LW) != K) begin : g_bad_k
    $error("ple_tree: K must be a power of 2 and >= 2");
  end
  if (CAP < K || (1 << IW) != CAP) begin : g_bad_cap
    $error("ple_tree: CAP must be a power of 2 and >= K");
  end

  logic [RW:0]   wr_row, rd_row, nn_row_a;
  logic [LW-1:0] wr_lane, rd_lane, rd_lane_q;
  assign wr_row   = {wr_tree, wr_idx[IW-1:LW]};
  assign wr_lane  = wr_idx[LW-1:0];
  assign rd_row   = {rd_tree, rd_idx[IW-1:LW]};
  assign rd_lane  = rd_idx[LW-1:0];
  assign nn_row_a = {nn_tree, nn_row[RW-1:0]};

  // memory words are flat vectors: Vivado will not infer BRAM from arrays of packed-array elements
  localparam int QW = DIM * JW;
  conf_t [K-1:0] rd_rowq;
  for (genvar l = 0; l < K; ++l) begin : g_lane
    logic [QW-1:0] ja[2 * ROWS];                         // port A copy
    logic [QW-1:0] jb[2 * ROWS];                         // port B copy
    logic [QW-1:0] ra, rb;
    always_ff @(posedge clk) begin
      if (wr_en && wr_lane == LW'(l)) ja[wr_row] <= QW'(wr_q);
      if (rd_en) ra <= ja[rd_row];
    end
    always_ff @(posedge clk) begin
      if (wr_en && wr_lane == LW'(l)) jb[wr_row] <= QW'(wr_q);
      if (nn_en) rb <= jb[nn_row_a];
    end
    assign rd_rowq[l] = conf_t'(ra);
    assign nn_q[l]    = conf_t'(rb);
  end

  always_ff @(posedge clk) if (rd_en) rd_lane_q <= rd_lane;
  assign rd_q = rd_rowq[rd_lane_q];

  // PARENT, RAD
  logic [IW-1:0] pm[2 * CAP];
  logic [RAD_W-1:0] rm[2 * CAP];
  // RAD has one write port: an insert (RAD = inf) wins over a dynamic-domain update
  logic                   rm_we;
  logic [IW:0]         rm_wa;
  logic [RAD_W-1:0]       rm_wd;
  assign rm_we = wr_en | rad_we;
  assign rm_wa = wr_en ? {wr_tree, wr_idx} : {rad_tree, rad_idx};
  assign rm_wd = wr_en ? RAD_INF : rad_wdata;
  always_ff @(posedge clk) begin
    if (wr_en) pm[{wr_tree, wr_idx}] <= wr_parent;
    if (rd_en) rd_parent <= pm[{rd_tree, rd_idx}];
  end
  always_ff @(posedge clk) begin
    if (rm_we) rm[rm_wa] <= rm_wd;
    if (rd_en) rd_rad <= rm[{rd_tree, rd_idx}];
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk) begin
    if (wr_en && rad_we && {rad_tree, rad_idx} != {wr_tree, wr_idx})
      $fatal(1, "ple_tree: insert and RAD update of different nodes in one cycle (RAD update lost)");
    if (wr_en && rd_en && {rd_tree, rd_idx} == {wr_tree, wr_idx})
      $fatal(1, "ple_tree: read of a node in the cycle it is inserted");
    if (wr_en && nn_en && nn_row_a == wr_row)
      $fatal(1, "ple_tree: insert into a JOINT row the NNU is reading");
    if (rad_we && !wr_en && rd_en && {rd_tree, rd_idx} == {rad_tree, rad_idx})
      $fatal(1, "ple_tree: read of RAD in the cycle it is written");
  end
`endif

  logic unused;
  assign unused = ^nn_row[IW-1:RW];

endmodule
