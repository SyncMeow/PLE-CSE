// One complete CSE compute unit: ball-to-voxel-to-cell-to-point streaming search.
module cse_cu
  import ple_cse_pkg::*;
  import cse_cu_pkg::*;
#(
    parameter int unsigned VOXEL_FIFO_DEPTH = 2,
    parameter int unsigned CELL_FIFO_DEPTH = 2,
    parameter int unsigned MAX_PTS_OUTSTANDING = 8,
    parameter bit KILL_ENABLE = 1'b1
) (
    input  logic                       clk,
    input  logic                       rst_n,
    input  logic                       q_valid,
    output logic                       q_ready,
    input  query_t                     q_payload,
    output logic                       r_valid,
    input  logic                       r_ready,
    output resp_t                      r_payload,
    input  logic                       k_valid,
    output logic                       k_ready,
    input  kill_t                      k_payload,
    output logic                       cse_idle,
    output logic                       cse_err,
    output logic                       mem_rd_buf,
    output logic [DIR_AW-1:0]          mem_dir_addr,
    input  logic [15:0]                mem_dir_data,
    output logic [SLOT_W-1:0]          mem_cell_addr,
    input  logic [127:0]               mem_cell_data,
    output logic [PT_AW-1:0]           mem_pt_addr,
    input  logic [PT_DATA_W-1:0]       mem_pt_data,
    input  logic [1:0]                 hdr_valid,
    input  logic [1:0]                 hdr_err,
    input  logic [1:0][PT_AW-1:0]      hdr_n_pts,
    input  logic [1:0][2:0][PT_W-1:0] hdr_glo,
    input  logic [1:0][2:0][PT_W-1:0] hdr_ghi
);

  localparam int NTAG = 1 << TAG_W;
  localparam int PEND_W = (MAX_PTS_OUTSTANDING <= 1) ? 1 : $clog2(MAX_PTS_OUTSTANDING + 1);
  localparam logic signed [18:0] GRID_LAST = 19'(ENV_G-1);

  typedef enum logic [1:0] {S2_IDLE, S2_DIR_WAIT, S2_CELL_WAIT, S2_OUT} s2_state_e;

  if (MAX_PTS_OUTSTANDING < 1) begin : g_bad_max_points
    $error("cse_cu: MAX_PTS_OUTSTANDING must be at least one");
  end

  logic [KILL_GEN_W-1:0] kill_gen[NTAG];
  logic [KILL_GEN_W-1:0] kill_gen_eff[NTAG];
  logic k_fire, q_fire, r_fire;

  cse_ball_t s0_payload, s0_new_payload;
  logic s0_valid, s0_ready;

  cse_ball_t s1_ball;
  logic s1_active, s1_end;
  logic [4:0] s1_gx, s1_gy, s1_gz;
  logic s1_out_valid, s1_out_ready, s1_out_fire;
  cse_voxel_t s1_out_payload;

  logic voxel_valid, voxel_ready;
  cse_voxel_t voxel_payload;

  s2_state_e s2_state;
  cse_voxel_t s2_voxel;
  cse_cell_token_t s2_out_payload;
  logic s2_out_valid, s2_out_ready;
  dir_t dir_word;
  cell_t cell_word;

  logic cell_valid, cell_ready;
  cse_cell_token_t cell_payload;

  logic s3_ball_active, s3_cell_active, s3_end_seen, s3_hit_seen;
  cse_ctx_t s3_ctx;
  logic [PT_AW-1:0] s3_issue_addr;
  logic [COUNT_W-1:0] s3_issue_left;
  logic [PEND_W-1:0] s3_pending;
  logic pt_issue, pt_return_valid;
  logic d0_valid, d1_valid, d2_valid;
  cse_dist_d0_t d0_payload;
  cse_dist_d1_t d1_payload;
  cse_dist_d2_t d2_payload;
  logic [37:0] d0_sq_x, d0_sq_y, d0_sq_z;
  logic signed [37:0] d0_dx_ext, d0_dy_ext, d0_dz_ext;
  logic [39:0] d1_sum;
  logic d2_hit, s3_drain_done, s3_hit_after;

  logic s4_valid;
  resp_t s4_payload;
  logic s4_can_accept, s3_complete;

  logic [7:0] outstanding;
  logic unused_internal;

  function automatic logic ctx_cancelled(input cse_ctx_t ctx);
    return (KILL_ENABLE && (ctx.birth_gen != kill_gen_eff[ctx.tag])) | (1'b0 & ^ctx);
  endfunction

  function automatic logic [4:0] clamp_grid(input logic signed [18:0] value);
    if (value < 0) return 5'd0;
    if (value > GRID_LAST) return GRID_LAST[4:0];
    return value[4:0];
  endfunction

  assign k_ready = rst_n;
  assign k_fire  = k_valid && k_ready;
  assign q_fire  = q_valid && q_ready;
  assign r_fire  = r_valid && r_ready;

  always_comb begin
    for (int t = 0; t < NTAG; ++t) begin
      kill_gen_eff[t] = kill_gen[t];
      if (KILL_ENABLE && k_fire && (k_payload.all || k_payload.tag == t[TAG_W-1:0]))
        kill_gen_eff[t] = kill_gen[t] + 1'b1;
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      for (int t = 0; t < NTAG; ++t) kill_gen[t] <= '0;
    end else if (KILL_ENABLE && k_fire) begin
      for (int t = 0; t < NTAG; ++t)
        if (k_payload.all || k_payload.tag == t[TAG_W-1:0]) kill_gen[t] <= kill_gen[t] + 1'b1;
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk) begin
    if (rst_n && KILL_ENABLE && k_fire && outstanding != 0)
      for (int t = 0; t < NTAG; ++t)
        if ((k_payload.all || k_payload.tag == t[TAG_W-1:0]) && (&kill_gen[t]))
          $error("cse_cu: kill generation wrapped with work outstanding");
  end
`endif

  // S0: capture a query and compute its exact integer bounding box and voxel range.
  always_comb begin : p_s0_preprocess
    logic signed [18:0] cx, cy, cz, radius;
    logic signed [18:0] sx_lo, sy_lo, sz_lo, sx_hi, sy_hi, sz_hi;
    logic signed [18:0] gx_lo, gy_lo, gz_lo, gx_hi, gy_hi, gz_hi;
    logic signed [18:0] hglo_x, hglo_y, hglo_z, hghi_x, hghi_y, hghi_z;
    logic empty_axis, global_cull;
    logic [27:0] radius_u;

    cx = $signed({q_payload.cx[COORD_W-1], q_payload.cx});
    cy = $signed({q_payload.cy[COORD_W-1], q_payload.cy});
    cz = $signed({q_payload.cz[COORD_W-1], q_payload.cz});
    radius = $signed({5'b0, q_payload.r});
    radius_u = {14'b0, q_payload.r};
    sx_lo = cx - radius;
    sy_lo = cy - radius;
    sz_lo = cz - radius;
    sx_hi = cx + radius;
    sy_hi = cy + radius;
    sz_hi = cz + radius;
    gx_lo = sx_lo >>> ENV_QSHIFT;
    gy_lo = sy_lo >>> ENV_QSHIFT;
    gz_lo = sz_lo >>> ENV_QSHIFT;
    gx_hi = sx_hi >>> ENV_QSHIFT;
    gy_hi = sy_hi >>> ENV_QSHIFT;
    gz_hi = sz_hi >>> ENV_QSHIFT;
    hglo_x = $signed({3'b000, hdr_glo[q_payload.env_buf][0]});
    hglo_y = $signed({3'b000, hdr_glo[q_payload.env_buf][1]});
    hglo_z = $signed({3'b000, hdr_glo[q_payload.env_buf][2]});
    hghi_x = $signed({3'b000, hdr_ghi[q_payload.env_buf][0]});
    hghi_y = $signed({3'b000, hdr_ghi[q_payload.env_buf][1]});
    hghi_z = $signed({3'b000, hdr_ghi[q_payload.env_buf][2]});
    global_cull = (sx_hi < hglo_x) || (sx_lo > hghi_x) ||
                  (sy_hi < hglo_y) || (sy_lo > hghi_y) ||
                  (sz_hi < hglo_z) || (sz_lo > hghi_z);
    empty_axis = (sx_hi < 0) || (sy_hi < 0) || (sz_hi < 0) ||
                 (gx_lo > GRID_LAST) || (gy_lo > GRID_LAST) || (gz_lo > GRID_LAST);

    s0_new_payload = '0;
    s0_new_payload.ctx.tag         = q_payload.tag;
    s0_new_payload.ctx.user        = q_payload.user;
    s0_new_payload.ctx.kill_on_hit = q_payload.kill_on_hit;
    s0_new_payload.ctx.env_buf     = q_payload.env_buf;
    s0_new_payload.ctx.cx          = q_payload.cx;
    s0_new_payload.ctx.cy          = q_payload.cy;
    s0_new_payload.ctx.cz          = q_payload.cz;
    s0_new_payload.ctx.r2          = radius_u * radius_u;
    s0_new_payload.ctx.bmin_x      = sx_lo;
    s0_new_payload.ctx.bmin_y      = sy_lo;
    s0_new_payload.ctx.bmin_z      = sz_lo;
    s0_new_payload.ctx.bmax_x      = sx_hi;
    s0_new_payload.ctx.bmax_y      = sy_hi;
    s0_new_payload.ctx.bmax_z      = sz_hi;
    s0_new_payload.ctx.birth_gen   = kill_gen_eff[q_payload.tag];
    s0_new_payload.ctx.error       = !hdr_valid[q_payload.env_buf] || hdr_err[q_payload.env_buf];
    s0_new_payload.lo[0]           = clamp_grid(gx_lo);
    s0_new_payload.lo[1]           = clamp_grid(gy_lo);
    s0_new_payload.lo[2]           = clamp_grid(gz_lo);
    s0_new_payload.hi[0]           = clamp_grid(gx_hi);
    s0_new_payload.hi[1]           = clamp_grid(gy_hi);
    s0_new_payload.hi[2]           = clamp_grid(gz_hi);
    s0_new_payload.skip_search     = s0_new_payload.ctx.error ||
                                     (hdr_n_pts[q_payload.env_buf] == 0) || global_cull || empty_axis;
  end

  assign q_ready = rst_n && (!s0_valid || s0_ready);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      s0_valid   <= 1'b0;
      s0_payload <= '0;
    end else begin
      if (q_fire) begin
        s0_valid   <= 1'b1;
        s0_payload <= s0_new_payload;
      end else if (s0_ready) begin
        s0_valid <= 1'b0;
      end
    end
  end

  // S1: enumerate x-fastest voxels, then emit exactly one END token.
  assign s0_ready = !s1_active;
  assign s1_out_valid = s1_active;
  assign s1_out_fire  = s1_out_valid && s1_out_ready;
  always_comb begin
    s1_out_payload = '0;
    s1_out_payload.kind = s1_end ? CSE_VOXEL_END : CSE_VOXEL;
    s1_out_payload.ctx  = s1_ball.ctx;
    s1_out_payload.gx   = s1_gx;
    s1_out_payload.gy   = s1_gy;
    s1_out_payload.gz   = s1_gz;
    s1_out_payload.dir_addr = ((DIR_AW'(s1_gz) * DIR_AW'(ENV_G)) + DIR_AW'(s1_gy)) *
                              DIR_AW'(ENV_G) + DIR_AW'(s1_gx);
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      s1_active <= 1'b0;
      s1_end    <= 1'b0;
      s1_ball   <= '0;
      s1_gx     <= '0;
      s1_gy     <= '0;
      s1_gz     <= '0;
    end else begin
      if (!s1_active && s0_valid) begin
        s1_active <= 1'b1;
        s1_ball   <= s0_payload;
        s1_gx     <= s0_payload.lo[0];
        s1_gy     <= s0_payload.lo[1];
        s1_gz     <= s0_payload.lo[2];
        s1_end    <= s0_payload.skip_search || ctx_cancelled(s0_payload.ctx);
      end else if (s1_out_fire) begin
        if (s1_end) begin
          s1_active <= 1'b0;
          s1_end    <= 1'b0;
        end else if (ctx_cancelled(s1_ball.ctx) ||
                     ((s1_gx == s1_ball.hi[0]) && (s1_gy == s1_ball.hi[1]) &&
                      (s1_gz == s1_ball.hi[2]))) begin
          s1_end <= 1'b1;
        end else if (s1_gx != s1_ball.hi[0]) begin
          s1_gx <= s1_gx + 1'b1;
        end else if (s1_gy != s1_ball.hi[1]) begin
          s1_gx <= s1_ball.lo[0];
          s1_gy <= s1_gy + 1'b1;
        end else begin
          s1_gx <= s1_ball.lo[0];
          s1_gy <= s1_ball.lo[1];
          s1_gz <= s1_gz + 1'b1;
        end
      end
    end
  end

  cse_fifo #(.PAYLOAD_T(cse_voxel_t), .DEPTH(VOXEL_FIFO_DEPTH)) u_voxel_fifo (
      .clk, .rst_n, .in_valid(s1_out_valid), .in_ready(s1_out_ready),
      .in_payload(s1_out_payload), .out_valid(voxel_valid), .out_ready(voxel_ready),
      .out_payload(voxel_payload)
  );

  // S2: one-at-a-time DIR/CELL lookup and conservative sphere-box rejection.
  assign dir_word  = dir_t'(mem_dir_data);
  assign cell_word = cell_t'(mem_cell_data);
  assign voxel_ready = (s2_state == S2_IDLE);
  assign s2_out_valid = (s2_state == S2_OUT);

  always_comb begin : p_s2_memory
    mem_dir_addr  = '0;
    mem_cell_addr = '0;
    if ((s2_state == S2_IDLE) && voxel_valid && (voxel_payload.kind == CSE_VOXEL) &&
        !ctx_cancelled(voxel_payload.ctx))
      mem_dir_addr = voxel_payload.dir_addr;
    if ((s2_state == S2_DIR_WAIT) && !ctx_cancelled(s2_voxel.ctx) && dir_word.occ)
      mem_cell_addr = dir_word.slot;
  end

  always_ff @(posedge clk) begin : p_s2_fsm
    logic signed [18:0] org_x, org_y, org_z;
    logic signed [18:0] alo_x, alo_y, alo_z, ahi_x, ahi_y, ahi_z;
    logic reject;
    if (!rst_n) begin
      s2_state       <= S2_IDLE;
      s2_voxel       <= '0;
      s2_out_payload <= '0;
    end else begin
      unique case (s2_state)
        S2_IDLE: begin
          if (voxel_valid) begin
            if (voxel_payload.kind == CSE_VOXEL_END) begin
              s2_out_payload.kind <= CSE_CELL_END;
              s2_out_payload.ctx  <= voxel_payload.ctx;
              s2_out_payload.base <= '0;
              s2_out_payload.count <= '0;
              s2_state <= S2_OUT;
            end else if (!ctx_cancelled(voxel_payload.ctx)) begin
              s2_voxel <= voxel_payload;
              s2_state <= S2_DIR_WAIT;
            end
          end
        end
        S2_DIR_WAIT: begin
          if (ctx_cancelled(s2_voxel.ctx) || !dir_word.occ) begin
            s2_state <= S2_IDLE;
          end else begin
            s2_state <= S2_CELL_WAIT;
          end
        end
        S2_CELL_WAIT: begin
          org_x = $signed({3'b000, s2_voxel.gx, {ENV_QSHIFT{1'b0}}});
          org_y = $signed({3'b000, s2_voxel.gy, {ENV_QSHIFT{1'b0}}});
          org_z = $signed({3'b000, s2_voxel.gz, {ENV_QSHIFT{1'b0}}});
          alo_x = org_x + $signed({8'b0, cell_word.lo_x});
          alo_y = org_y + $signed({8'b0, cell_word.lo_y});
          alo_z = org_z + $signed({8'b0, cell_word.lo_z});
          ahi_x = org_x + $signed({8'b0, cell_word.hi_x});
          ahi_y = org_y + $signed({8'b0, cell_word.hi_y});
          ahi_z = org_z + $signed({8'b0, cell_word.hi_z});
          reject = (s2_voxel.ctx.bmax_x < alo_x) || (s2_voxel.ctx.bmin_x > ahi_x) ||
                   (s2_voxel.ctx.bmax_y < alo_y) || (s2_voxel.ctx.bmin_y > ahi_y) ||
                   (s2_voxel.ctx.bmax_z < alo_z) || (s2_voxel.ctx.bmin_z > ahi_z);
          if (ctx_cancelled(s2_voxel.ctx) || (cell_word.count == 0) || reject) begin
            s2_state <= S2_IDLE;
          end else begin
            s2_out_payload.kind  <= CSE_CELL;
            s2_out_payload.ctx   <= s2_voxel.ctx;
            s2_out_payload.base  <= cell_word.base;
            s2_out_payload.count <= cell_word.count;
            s2_state <= S2_OUT;
          end
        end
        default: begin
          if (s2_out_ready) s2_state <= S2_IDLE;
        end
      endcase
    end
  end

  cse_fifo #(.PAYLOAD_T(cse_cell_token_t), .DEPTH(CELL_FIFO_DEPTH)) u_cell_fifo (
      .clk, .rst_n, .in_valid(s2_out_valid), .in_ready(s2_out_ready),
      .in_payload(s2_out_payload), .out_valid(cell_valid), .out_ready(cell_ready),
      .out_payload(cell_payload)
  );

  // The memory port set has one shared buffer select. The interface contract keeps it fixed while busy.
  always_comb begin
    mem_rd_buf = 1'b0;
    if (s3_ball_active) mem_rd_buf = s3_ctx.env_buf;
    else if (s2_state != S2_IDLE) mem_rd_buf = s2_voxel.ctx.env_buf;
    else if (voxel_valid) mem_rd_buf = voxel_payload.ctx.env_buf;
    else if (s1_active) mem_rd_buf = s1_ball.ctx.env_buf;
  end

  // S3: issue at most one point per cycle and drain a fixed D0-D3 distance pipeline.
  assign cell_ready = !s3_ball_active || (!s3_cell_active && !s3_end_seen);
  assign pt_issue = s3_cell_active && !ctx_cancelled(s3_ctx) && !s3_hit_seen &&
                    ((s3_pending < PEND_W'(MAX_PTS_OUTSTANDING)) || d2_valid);
  assign mem_pt_addr = s3_issue_addr;
  assign d0_dx_ext = {{19{d0_payload.dx[18]}}, d0_payload.dx};
  assign d0_dy_ext = {{19{d0_payload.dy[18]}}, d0_payload.dy};
  assign d0_dz_ext = {{19{d0_payload.dz[18]}}, d0_payload.dz};
  assign d0_sq_x = $unsigned(d0_dx_ext * d0_dx_ext);
  assign d0_sq_y = $unsigned(d0_dy_ext * d0_dy_ext);
  assign d0_sq_z = $unsigned(d0_dz_ext * d0_dz_ext);
  assign d1_sum  = {2'b0, d1_payload.sx} + {2'b0, d1_payload.sy} + {2'b0, d1_payload.sz};
  assign d2_hit  = d2_payload.d2 <= {12'b0, s3_ctx.r2};
  assign s3_drain_done = (s3_pending == PEND_W'(d2_valid)) && !pt_issue;
  assign s3_hit_after  = s3_hit_seen || (d2_valid && d2_hit);
  assign s4_can_accept = !s4_valid || r_ready;
  assign s3_complete = s3_ball_active && s3_end_seen && !s3_cell_active &&
                       s3_drain_done && s4_can_accept;

  always_ff @(posedge clk) begin : p_s3
    logic signed [18:0] px, py, pz;
    if (!rst_n) begin
      s3_ball_active <= 1'b0;
      s3_cell_active <= 1'b0;
      s3_end_seen    <= 1'b0;
      s3_hit_seen    <= 1'b0;
      s3_ctx         <= '0;
      s3_issue_addr  <= '0;
      s3_issue_left  <= '0;
      s3_pending     <= '0;
      pt_return_valid <= 1'b0;
      d0_valid       <= 1'b0;
      d1_valid       <= 1'b0;
      d2_valid       <= 1'b0;
      d0_payload     <= '0;
      d1_payload     <= '0;
      d2_payload     <= '0;
    end else begin
      pt_return_valid <= pt_issue;
      d0_valid <= pt_return_valid;
      d1_valid <= d0_valid;
      d2_valid <= d1_valid;

      if (pt_return_valid) begin
        px = $signed({3'b000, mem_pt_data[15:0]});
        py = $signed({3'b000, mem_pt_data[31:16]});
        pz = $signed({3'b000, mem_pt_data[47:32]});
        d0_payload.dx <= $signed({s3_ctx.cx[COORD_W-1], s3_ctx.cx}) - px;
        d0_payload.dy <= $signed({s3_ctx.cy[COORD_W-1], s3_ctx.cy}) - py;
        d0_payload.dz <= $signed({s3_ctx.cz[COORD_W-1], s3_ctx.cz}) - pz;
      end
      if (d0_valid) begin
        d1_payload.sx <= d0_sq_x;
        d1_payload.sy <= d0_sq_y;
        d1_payload.sz <= d0_sq_z;
      end
      if (d1_valid) d2_payload.d2 <= d1_sum;

      unique case ({pt_issue, d2_valid})
        2'b10: s3_pending <= s3_pending + 1'b1;
        2'b01: s3_pending <= s3_pending - 1'b1;
        default: s3_pending <= s3_pending;
      endcase

      if (pt_issue) begin
        s3_issue_addr <= s3_issue_addr + 1'b1;
        s3_issue_left <= s3_issue_left - 1'b1;
        if (s3_issue_left == 1) s3_cell_active <= 1'b0;
      end

      if (d2_valid && d2_hit) begin
        s3_hit_seen    <= 1'b1;
        s3_cell_active <= 1'b0;
      end
      if (s3_ball_active && ctx_cancelled(s3_ctx)) s3_cell_active <= 1'b0;

      if (cell_valid && cell_ready) begin
        if (!s3_ball_active) begin
          s3_ball_active <= 1'b1;
          s3_ctx         <= cell_payload.ctx;
          s3_hit_seen    <= 1'b0;
          s3_end_seen    <= (cell_payload.kind == CSE_CELL_END);
          if ((cell_payload.kind == CSE_CELL) && !ctx_cancelled(cell_payload.ctx)) begin
            s3_cell_active <= 1'b1;
            s3_issue_addr  <= cell_payload.base;
            s3_issue_left  <= cell_payload.count;
          end else begin
            s3_cell_active <= 1'b0;
          end
        end else if (cell_payload.kind == CSE_CELL_END) begin
          s3_end_seen <= 1'b1;
        end else if (!ctx_cancelled(s3_ctx) && !s3_hit_after) begin
          s3_cell_active <= 1'b1;
          s3_issue_addr  <= cell_payload.base;
          s3_issue_left  <= cell_payload.count;
        end
      end

      if (s3_complete) begin
        s3_ball_active <= 1'b0;
        s3_cell_active <= 1'b0;
        s3_end_seen    <= 1'b0;
        s3_hit_seen    <= 1'b0;
      end
    end
  end

  // S4: hold a completed response stable until the receiver accepts it.
  assign r_valid   = s4_valid;
  assign r_payload = s4_payload;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      s4_valid   <= 1'b0;
      s4_payload <= '0;
      cse_err    <= 1'b0;
    end else begin
      if (r_fire) s4_valid <= 1'b0;
      if (s3_complete) begin
        s4_valid        <= 1'b1;
        s4_payload.tag  <= s3_ctx.tag;
        s4_payload.user <= s3_ctx.user;
        if (s3_ctx.error) begin
          s4_payload.status <= ST_ERROR;
          cse_err <= 1'b1;
        end else if (s3_hit_after) begin
          s4_payload.status <= ST_HIT;
        end else if (ctx_cancelled(s3_ctx)) begin
          s4_payload.status <= ST_KILLED;
        end else begin
          s4_payload.status <= ST_MISS;
        end
      end
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      outstanding <= '0;
    end else begin
      unique case ({q_fire, r_fire})
        2'b10: outstanding <= outstanding + 1'b1;
        2'b01: outstanding <= outstanding - 1'b1;
        default: outstanding <= outstanding;
      endcase
    end
  end

  assign cse_idle = (outstanding == 0);
  assign unused_internal = ^{s1_ball.skip_search, s2_voxel.kind, s2_voxel.dir_addr,
                             dir_word.rsvd, cell_word.rsvd};

endmodule
