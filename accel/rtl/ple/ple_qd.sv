// QD: environment queries for the poses of one edge (PSM + QD + RSP + TAG of the design doc, 7.4).
// A pose's 70 sphere centres go into a slot (PSM: one small memory per sphere, written in one cycle).
// Hierarchical mode: the 11 outer spheres are queried first; an outer HIT queues its fine spheres
// (kill_on_hit = 1). Flat mode: the 59 fine spheres directly. Same pose result as golden fx_env_ok().
// Issue order: fine spheres first, then outer spheres; oldest pose first. One query per cycle.
// A fine HIT decides the pose (pd_hit) and the edge: QD stops accepting / issuing and sends K(tag); the
// controller also raises edge_kill for self collisions. Two tags alternate: a new edge needs the other
// tag's responses all back (edge_ready). r_ready is tied high (interface rule P1).
module ple_qd
  import ple_cse_pkg::*;
  import ple_robot_pkg::*;
#(
    parameter int NSLOT = 8,                             // pose slots (<= 16: user field has 4 bits)
    parameter int PIDW  = 16                             // pose id width
) (
    input  logic                      clk,
    input  logic                      rst_n,
    input  logic                      cfg_flat,          // 1 = query fine spheres only
    input  logic                      cfg_env_buf,
    // edge control
    output logic                      edge_ready,        // the next tag has no response outstanding
    input  logic                      edge_start,        // begin an edge (only while edge_ready)
    input  logic                      edge_kill,         // edge invalid elsewhere: stop, kill its queries
    output logic                      idle,              // no pose in flight, no query held
    output logic                      env_err,           // an ERROR response arrived (sticky per edge)
    // poses (from FKU)
    input  logic                      p_valid,
    output logic                      p_ready,
    input  logic [PIDW-1:0]           p_id,
    input  logic [69:0][2:0][17:0]    p_c,
    // pose results: hit = 1 environment collision (reported at the fine HIT), 0 = pose free
    output logic                      pd_valid,
    output logic [PIDW-1:0]           pd_id,
    output logic                      pd_hit,
    // CSE interface (ple_cse_pkg)
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
    input  logic                      r_valid,
    output logic                      r_ready,
    input  logic [TAG_W-1:0]          r_tag,
    input  logic [USER_W-1:0]         r_user,
    input  logic [1:0]                r_status,
    output logic                      k_valid,
    input  logic                      k_ready,
    output logic                      k_all,
    output logic [TAG_W-1:0]          k_tag
);

  localparam int SW = $clog2(NSLOT);

  if (NSLOT > 16 || NSLOT < 2 || (1 << SW) != NSLOT) begin : g_bad_nslot
    $error("ple_qd: NSLOT must be a power of 2 in 2..16");
  end

  // outer sphere -> index in OUTER (or -1)
  function automatic int outer_idx(input int s);
    for (int o = 0; o < N_OUTER; ++o) if (int'(OUTER[o]) == s) return o;
    return -1;
  endfunction

  // ---------------------------------------------------------------- PSM
  logic [53:0] psm [N_SPH][NSLOT];

  // ---------------------------------------------------------------- slot state
  logic [NSLOT-1:0]        live, shit;               // shit: a fine HIT decided this pose
  logic [NSLOT-1:0][PIDW-1:0] sid;
  logic [NSLOT-1:0][7:0]   sage;                     // allocation order (wraps; compared as differences)
  logic [NSLOT-1:0][3:0]   optr;                     // next outer to query (N_OUTER = all queried)
  logic [NSLOT-1:0][N_OUTER-1:0] pend;               // outer HITs whose fine spheres are not started
  logic [NSLOT-1:0]        xact;                     // expanding an outer's fine spheres
  logic [NSLOT-1:0][3:0]   xo;                       // outer being expanded
  logic [NSLOT-1:0][7:0]   xk;                       // next index into KIDS
  logic [NSLOT-1:0][7:0]   outst;                    // responses outstanding
  logic [7:0]              age_ctr;

  logic       cur_tag, live_edge, dead;
  logic [1:0][15:0] tag_out;                         // responses outstanding per tag (tags 0 / 1)

  assign r_ready    = 1'b1;
  assign q_env_buf  = cfg_env_buf;
  assign k_all      = 1'b0;
  assign edge_ready = (tag_out[~cur_tag] == 0) && !k_valid;
  assign idle       = (live == '0) && !q_valid;

  // ---------------------------------------------------------------- pose accept
  logic [SW-1:0] fs;                                 // lowest free slot
  logic          has_free;
  always_comb begin
    fs = '0; has_free = 1'b0;
    for (int s = NSLOT - 1; s >= 0; --s) if (!live[s]) begin fs = SW'(s); has_free = 1'b1; end
  end
  assign p_ready = live_edge && !dead && has_free;
  wire accept = p_valid && p_ready;

  // ---------------------------------------------------------------- issue selection
  logic          can_q, sel_v, sel_child;
  logic [SW-1:0] sel;
  always_comb begin
    logic [7:0] best;
    logic       bv;
    can_q = live_edge && !dead && (!q_valid || q_ready);
    sel = '0; sel_v = 1'b0; sel_child = 1'b0;
    // oldest slot with fine spheres to send
    bv = 1'b0; best = '0;
    for (int s = 0; s < NSLOT; ++s)
      if (live[s] && (xact[s] || pend[s] != 0) && (!bv || $signed(sage[s] - best) < 0)) begin
        bv = 1'b1; best = sage[s]; sel = SW'(s);
      end
    if (bv) begin sel_v = 1'b1; sel_child = 1'b1; end
    else
      for (int s = 0; s < NSLOT; ++s)
        if (live[s] && optr[s] != 4'(N_OUTER) && (!bv || $signed(sage[s] - best) < 0)) begin
          bv = 1'b1; best = sage[s]; sel = SW'(s);
        end
    if (bv && !sel_child) sel_v = 1'b1;
  end
  wire issue = can_q && sel_v;

  // sphere for the selected slot
  logic [7:0] sph;
  logic [3:0] c_o;                                   // outer being expanded
  logic [7:0] c_k;                                   // KIDS index
  always_comb begin
    c_o = xo[sel];
    c_k = xk[sel];
    if (sel_child && !xact[sel])
      for (int o = N_OUTER - 1; o >= 0; --o) if (pend[sel][o]) begin c_o = 4'(o); c_k = KID_OFF[o]; end
    sph = sel_child ? KIDS[c_k[$clog2(N_FINE)-1:0]] : OUTER[optr[sel]];
  end

  // ---------------------------------------------------------------- response decode
  wire [SW-1:0] r_slot = r_user[USER_SLOT_LSB +: SW];
  wire [7:0]    r_sph  = r_user[USER_SPHERE_LSB +: 8];
  wire          r_cur  = r_valid && (r_tag[0] == cur_tag) && live_edge && !dead && live[r_slot];

  // ---------------------------------------------------------------- state update
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      live <= '0; shit <= '0; age_ctr <= '0; cur_tag <= 1'b1; live_edge <= 1'b0; dead <= 1'b0;
      tag_out <= '0; q_valid <= 1'b0; k_valid <= 1'b0; k_tag <= '0; pd_valid <= 1'b0; env_err <= 1'b0;
    end else begin
      logic [NSLOT-1:0]           n_live, n_xact;
      logic [NSLOT-1:0][N_OUTER-1:0] n_pend;
      logic [NSLOT-1:0][7:0]      n_outst;
      logic [1:0][15:0]           n_tag;
      logic                       fine_hit, kill_now, done_v;
      logic [SW-1:0]              done_s;
      n_live = live; n_pend = pend; n_xact = xact; n_outst = outst; n_tag = tag_out;
      fine_hit = 1'b0;
      pd_valid <= 1'b0;

      // responses
      if (r_valid) n_tag[r_tag[0]] = n_tag[r_tag[0]] - 16'd1;
      if (r_cur) begin
        n_outst[r_slot] = n_outst[r_slot] - 8'd1;
        if (r_status == ST_HIT) begin
          if (outer_idx(int'(r_sph)) >= 0) n_pend[r_slot][outer_idx(int'(r_sph))] = 1'b1;
          else fine_hit = 1'b1;
        end
      end
      if (r_valid && r_status == ST_ERROR) env_err <= 1'b1;

      // issue
      if (issue) begin
        q_valid       <= 1'b1;
        q_tag         <= TAG_W'(cur_tag);
        q_user        <= USER_W'({4'(sel), sph, 4'b0000});
        q_kill_on_hit <= sel_child;
        {q_cz, q_cy, q_cx} <= psm[sph[$clog2(N_SPH)-1:0]][sel];
        q_r           <= RQ[sph[$clog2(N_SPH)-1:0]];
        n_outst[sel]  = n_outst[sel] + 8'd1;
        n_tag[cur_tag] = n_tag[cur_tag] + 16'd1;
        if (sel_child) begin
          if (!xact[sel]) n_pend[sel][c_o] = 1'b0;
          if (c_k + 8'd1 == KID_OFF[c_o + 4'd1]) n_xact[sel] = 1'b0;
          else begin n_xact[sel] = 1'b1; xo[sel] <= c_o; xk[sel] <= c_k + 8'd1; end
        end else optr[sel] <= optr[sel] + 4'd1;
      end else if (q_ready) q_valid <= 1'b0;

      // a fine HIT decides the pose and the edge
      kill_now = (fine_hit || edge_kill) && live_edge && !dead;
      if (fine_hit) begin
        pd_valid <= 1'b1; pd_id <= sid[r_slot]; pd_hit <= 1'b1; shit[r_slot] <= 1'b1;
      end
      if (kill_now) begin
        dead <= 1'b1; n_live = '0;
        k_valid <= 1'b1; k_tag <= TAG_W'(cur_tag);
      end else if (k_valid && k_ready) k_valid <= 1'b0;

      // pose completion (one report per cycle; a fine HIT report wins, completion waits a cycle)
      done_v = 1'b0; done_s = '0;
      for (int s = NSLOT - 1; s >= 0; --s)
        if (n_live[s] && optr[s] == 4'(N_OUTER) && !(issue && !sel_child && sel == SW'(s)) && n_pend[s] == 0 &&
            !n_xact[s] && n_outst[s] == 0 && !shit[s]) begin
          done_v = 1'b1; done_s = SW'(s);
        end
      if (done_v && !fine_hit && !kill_now) begin
        pd_valid <= 1'b1; pd_id <= sid[done_s]; pd_hit <= 1'b0; n_live[done_s] = 1'b0;
      end

      // accept a pose into the lowest free slot (a pose taken in the kill cycle is dropped: the edge is decided)
      if (accept && !kill_now) begin
        for (int sp = 0; sp < N_SPH; ++sp) psm[sp][fs] <= {p_c[sp][2], p_c[sp][1], p_c[sp][0]};
        n_live[fs] = 1'b1;
        shit[fs]   <= 1'b0;
        sid[fs]    <= p_id;
        sage[fs]   <= age_ctr;
        age_ctr    <= age_ctr + 8'd1;
        optr[fs]   <= cfg_flat ? 4'(N_OUTER) : 4'd0;
        n_pend[fs] = cfg_flat ? '1 : '0;
        n_xact[fs] = 1'b0;
        n_outst[fs] = '0;
      end

      // edge start: switch tag, clear everything
      if (edge_start) begin
        cur_tag <= ~cur_tag; live_edge <= 1'b1; dead <= 1'b0; env_err <= 1'b0;
        n_live = '0; shit <= '0;
      end

      live <= n_live; pend <= n_pend; xact <= n_xact; outst <= n_outst; tag_out <= n_tag;
    end
  end

  logic unused;
  assign unused = ^{r_user[3:0], r_user[USER_W-1:USER_SLOT_LSB + SW], r_tag[TAG_W-1:1], sph[7:$clog2(N_SPH)],
                    c_k[7:$clog2(N_FINE)]};

endmodule
