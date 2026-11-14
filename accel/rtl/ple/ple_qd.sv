// QD: environment queries for the poses of one edge (PSM + QD + RSP + TAG of the design doc, 7.4).
// A pose's 70 sphere centres go into a slot (PSM: 4 LUTRAM banks, written 4 spheres per cycle from a staging
// register, so a pose is accepted in one cycle).
// Hierarchical mode: the 11 outer spheres are queried first; an outer HIT queues its fine spheres
// (kill_on_hit = 1). Flat mode: the 59 fine spheres directly. Same pose result as golden fx_env_ok().
// Issue order: fine spheres first, then outer spheres; oldest pose first. One query per cycle, two stages
// (select, then PSM read into the Q register).
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
    output logic                      env_err,           // an ERROR of the current tag arrived (sticky per edge)
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

  // ---------------------------------------------------------------- PSM: 4 banks (sphere mod 4), LUTRAM
  // A pose is latched into a staging register when accepted and written 4 spheres per cycle (WR_N cycles);
  // the slot issues only after its write finished (wip = 0).
  localparam int NB = 4, WR_N = (N_SPH + NB - 1) / NB, IXW = $clog2(WR_N), AW = SW + IXW;
  logic [WR_N*NB-1:0][53:0] stg;
  logic          wr_busy;
  logic [SW-1:0] wr_slot;
  logic [IXW-1:0] wr_cnt;
  logic [NSLOT-1:0] wip;                             // slot accepted, PSM write not finished
  logic [NB-1:0][53:0] psm_rd;

  // ---------------------------------------------------------------- slot state
  logic [NSLOT-1:0]        live, shit;               // shit: a fine HIT decided this pose
  logic [NSLOT-1:0]        unk;                      // a KILLED answer: the pose can no longer be reported free
  logic [NSLOT-1:0][PIDW-1:0] sid;
  logic [NSLOT-1:0][NSLOT-1:0] older;                // older[i][j]: slot i was accepted before slot j
  logic [NSLOT-1:0][3:0]   optr;                     // next outer to query (N_OUTER = all queried)
  logic [NSLOT-1:0][N_OUTER-1:0] pend;               // outer HITs whose fine spheres are not started
  logic [NSLOT-1:0]        xact;                     // expanding an outer's fine spheres
  logic [NSLOT-1:0][3:0]   xo;                       // outer being expanded
  logic [NSLOT-1:0][7:0]   xk;                       // next index into KIDS
  logic [NSLOT-1:0][7:0]   outst;                    // responses outstanding (stage A included)

  logic       cur_tag, live_edge, dead;
  logic [1:0][15:0] tag_out;                         // responses outstanding per tag (tags 0 / 1)

  // issue stage A (selected slot / sphere); stage B is the Q register
  logic          a_v, a_child;
  logic [SW-1:0] a_slot;
  logic [6:0]    a_sph;

  assign r_ready    = 1'b1;
  assign q_env_buf  = cfg_env_buf;
  assign k_all      = 1'b0;
  assign edge_ready = (tag_out[~cur_tag] == 0) && !k_valid;
  assign idle       = (live == '0) && !q_valid && !a_v && !wr_busy;

  // ---------------------------------------------------------------- pose accept
  logic [SW-1:0] fs;                                 // lowest free slot
  logic          has_free;
  always_comb begin
    fs = '0; has_free = 1'b0;
    for (int s = NSLOT - 1; s >= 0; --s) if (!live[s]) begin fs = SW'(s); has_free = 1'b1; end
  end
  assign p_ready = live_edge && !dead && has_free && !wr_busy;
  wire accept = p_valid && p_ready;

  // ---------------------------------------------------------------- issue selection (stage A)
  wire a_adv  = a_v && (!q_valid || q_ready);       // stage A moves into the Q register
  wire a_free = !a_v || a_adv;
  logic          sel_v, sel_child;
  logic [SW-1:0] sel;
  logic [NSLOT-1:0] oh;                              // selected slot, one-hot
  always_comb begin
    logic [NSLOT-1:0] ec, eo, e;
    for (int s = 0; s < NSLOT; ++s) begin
      ec[s] = live[s] && !wip[s] && (xact[s] || pend[s] != 0);
      eo[s] = live[s] && !wip[s] && optr[s] != 4'(N_OUTER);
    end
    sel_child = (ec != 0);                           // fine spheres first
    e = sel_child ? ec : eo;
    for (int i = 0; i < NSLOT; ++i) begin           // oldest eligible slot (one-hot)
      oh[i] = e[i];
      for (int j = 0; j < NSLOT; ++j) if (j != i && e[j] && !older[i][j]) oh[i] = 1'b0;
    end
    sel_v = (e != 0);
    sel = '0;
    for (int s = 0; s < NSLOT; ++s) if (oh[s]) sel = sel | SW'(s);
  end
  wire issue = live_edge && !dead && sel_v && a_free;

  // next sphere of every slot, in parallel with the selection; the one-hot select picks one
  logic [NSLOT-1:0][3:0] s_co;                       // outer being expanded
  logic [NSLOT-1:0][7:0] s_ck;                       // KIDS index
  logic [NSLOT-1:0]      s_last;                     // s_ck is the last fine sphere of s_co
  logic [NSLOT-1:0][6:0] s_kid, s_out;
  logic [6:0]            sph;
  always_comb begin
    for (int t = 0; t < NSLOT; ++t) begin
      s_co[t] = xo[t];
      s_ck[t] = xk[t];
      if (!xact[t])
        for (int o = N_OUTER - 1; o >= 0; --o) if (pend[t][o]) begin s_co[t] = 4'(o); s_ck[t] = KID_OFF[o]; end
      s_last[t] = (s_ck[t] + 8'd1 == KID_OFF[s_co[t] + 4'd1]);
      s_kid[t]  = 7'(KIDS[s_ck[t][$clog2(N_FINE)-1:0]]);
      s_out[t]  = (optr[t] < 4'(N_OUTER)) ? 7'(OUTER[optr[t]]) : 7'd0;
    end
    sph = '0;
    for (int t = 0; t < NSLOT; ++t) if (oh[t]) sph = sph | (sel_child ? s_kid[t] : s_out[t]);
  end

  // ---------------------------------------------------------------- PSM banks
  for (genvar b = 0; b < NB; ++b) begin : g_psm
    logic [53:0] mem [1 << AW];
    always_ff @(posedge clk) if (wr_busy) mem[{wr_slot, wr_cnt}] <= stg[b];
    assign psm_rd[b] = mem[{a_slot, IXW'(a_sph[6:2])}];
  end

  // ---------------------------------------------------------------- response decode
  wire [SW-1:0] r_slot = r_user[USER_SLOT_LSB +: SW];
  wire [7:0]    r_sph  = r_user[USER_SPHERE_LSB +: 8];
  wire          r_cur  = r_valid && (r_tag[0] == cur_tag) && live_edge && !dead && live[r_slot];

  // ---------------------------------------------------------------- state update
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      live <= '0; shit <= '0; unk <= '0; wip <= '0; wr_busy <= 1'b0; cur_tag <= 1'b1; live_edge <= 1'b0; dead <= 1'b0;
      tag_out <= '0; a_v <= 1'b0; q_valid <= 1'b0; k_valid <= 1'b0; k_tag <= '0; pd_valid <= 1'b0;
      env_err <= 1'b0;
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
        if (r_status == ST_KILLED) unk[r_slot] <= 1'b1;   // auto-kill: the HIT that killed it may come later
        if (r_status == ST_HIT) begin
          if (outer_idx(int'(r_sph)) >= 0) n_pend[r_slot][outer_idx(int'(r_sph))] = 1'b1;
          else fine_hit = 1'b1;
        end
      end
      if (r_valid && r_status == ST_ERROR && r_tag[0] == cur_tag) env_err <= 1'b1;   // an old tag's ERROR is stale

      // stage B: Q register
      if (a_adv) begin
        q_valid       <= 1'b1;
        q_tag         <= TAG_W'(cur_tag);
        q_user        <= USER_W'({4'(a_slot), 1'b0, a_sph, 4'b0000});
        q_kill_on_hit <= a_child;
        {q_cz, q_cy, q_cx} <= psm_rd[a_sph[1:0]];
        q_r           <= RQ[a_sph];
      end else if (q_ready) q_valid <= 1'b0;

      // stage A: select
      if (issue) begin
        a_v <= 1'b1; a_slot <= sel; a_sph <= sph; a_child <= sel_child;
        n_tag[cur_tag] = n_tag[cur_tag] + 16'd1;
        for (int t = 0; t < NSLOT; ++t) if (oh[t]) begin
          n_outst[t] = n_outst[t] + 8'd1;
          if (sel_child) begin
            if (!xact[t]) n_pend[t][s_co[t]] = 1'b0;
            if (s_last[t]) n_xact[t] = 1'b0;
            else begin n_xact[t] = 1'b1; xo[t] <= s_co[t]; xk[t] <= s_ck[t] + 8'd1; end
          end else optr[t] <= optr[t] + 4'd1;
        end
      end else if (a_adv) a_v <= 1'b0;

      // a fine HIT decides the pose and the edge
      kill_now = (fine_hit || edge_kill) && live_edge && !dead;
      if (fine_hit) begin
        pd_valid <= 1'b1; pd_id <= sid[r_slot]; pd_hit <= 1'b1; shit[r_slot] <= 1'b1;
      end
      if (kill_now) begin
        dead <= 1'b1; n_live = '0;
        k_valid <= 1'b1; k_tag <= TAG_W'(cur_tag);
        if (issue || (a_v && !a_adv)) n_tag[cur_tag] = n_tag[cur_tag] - 16'd1;   // stage A query dropped
        a_v <= 1'b0;
      end else if (k_valid && k_ready) k_valid <= 1'b0;

      // pose completion from registered state (one report per cycle; a fine HIT report wins, completion waits a
      // cycle). Safe: a slot with nothing left to send and no response outstanding gets no issue or response now.
      done_v = 1'b0; done_s = '0;
      for (int s = NSLOT - 1; s >= 0; --s)
        if (live[s] && !wip[s] && optr[s] == 4'(N_OUTER) && pend[s] == 0 && !xact[s] && outst[s] == 0 &&
            !shit[s] && !unk[s]) begin
          done_v = 1'b1; done_s = SW'(s);
        end
      if (done_v && !fine_hit && !kill_now && !edge_start) begin
        pd_valid <= 1'b1; pd_id <= sid[done_s]; pd_hit <= 1'b0; n_live[done_s] = 1'b0;
      end

      // PSM write of the accepted pose
      if (wr_busy) begin
        stg <= stg >> (NB * 54);
        wr_cnt <= wr_cnt + IXW'(1);
        if (wr_cnt == IXW'(WR_N - 1)) begin wr_busy <= 1'b0; wip[wr_slot] <= 1'b0; end
      end

      // accept a pose into the lowest free slot (a pose taken in the kill cycle is dropped: the edge is decided)
      if (accept && !kill_now) begin
        stg <= (WR_N*NB*54)'(p_c);
        wr_busy <= 1'b1; wr_slot <= fs; wr_cnt <= '0; wip[fs] <= 1'b1;
        n_live[fs] = 1'b1;
        shit[fs]   <= 1'b0;
        unk[fs]    <= 1'b0;
        sid[fs]    <= p_id;
        for (int j = 0; j < NSLOT; ++j) begin older[fs][j] <= 1'b0; if (j != int'(fs)) older[j][fs] <= 1'b1; end
        optr[fs]   <= cfg_flat ? 4'(N_OUTER) : 4'd0;
        n_pend[fs] = cfg_flat ? '1 : '0;
        n_xact[fs] = 1'b0;
        n_outst[fs] = '0;
      end

      // kill or edge start: stop the PSM write
      if (kill_now || edge_start) begin wr_busy <= 1'b0; wip <= '0; end

      // edge start: switch tag, clear everything
      if (edge_start) begin
        cur_tag <= ~cur_tag; live_edge <= 1'b1; dead <= 1'b0; env_err <= 1'b0;
        n_live = '0; shit <= '0; unk <= '0;
      end

      live <= n_live; pend <= n_pend; xact <= n_xact; outst <= n_outst; tag_out <= n_tag;
    end
  end

`ifndef SYNTHESIS
  always @(posedge clk) if (rst_n && edge_start && (a_v || wr_busy))
    $fatal(1, "ple_qd: edge_start with a query in stage A or a PSM write in progress");
`endif

  logic unused;
  logic [NSLOT-1:0][7-$clog2(N_FINE):0] ck_hi;
  always_comb for (int t = 0; t < NSLOT; ++t) ck_hi[t] = s_ck[t][7:$clog2(N_FINE)];
  assign unused = ^{r_user[3:0], r_user[USER_W-1:USER_SLOT_LSB + SW], r_tag[TAG_W-1:1], ck_hi};

endmodule
