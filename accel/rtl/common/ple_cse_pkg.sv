// Shared definitions of the PLE <-> CSE interface and environment format v0.
// Source of truth: records/20261107-ple-cse-design.md sections 4 and 5.1 (draft v0.1, not frozen).
/* verilator lint_off UNUSEDPARAM */  // each module uses only part of the package
package ple_cse_pkg;

  // ---------------------------------------------------------------- PLE <-> CSE (section 4)
  localparam int TAG_W   = 4;                     // edge tag, up to 16 edges in flight
  localparam int USER_W  = 16;                    // opaque to CSE, echoed in the response
  localparam int COORD_W = 18;                    // signed, LSB = 2^-14 m, origin = w_min
  localparam int RQ_W    = 14;                    // unsigned, LSB = 2^-14 m
  localparam logic [RQ_W-1:0] R_MAX = 14'd8192;   // PLE guarantees q_r <= R_MAX (0.5 m)
  localparam int unsigned MAX_LAT = 65536;        // C5: max gap between responses (r_ready high, work outstanding)

  typedef enum logic [1:0] {
    ST_MISS   = 2'd0,
    ST_HIT    = 2'd1,
    ST_KILLED = 2'd2,                             // not evaluated; never counts as a miss
    ST_ERROR  = 2'd3                              // environment unusable; also sets cse_err
  } status_e;

  // Optional bundles; the port contract is the flat signal list in section 4.2
  typedef struct packed {
    logic [TAG_W-1:0]          tag;
    logic [USER_W-1:0]         user;
    logic                      kill_on_hit;
    logic                      env_buf;
    logic signed [COORD_W-1:0] cx;
    logic signed [COORD_W-1:0] cy;
    logic signed [COORD_W-1:0] cz;
    logic [RQ_W-1:0]           r;
  } query_t;

  typedef struct packed {
    logic [TAG_W-1:0]  tag;
    logic [USER_W-1:0] user;
    status_e           status;
  } resp_t;

  typedef struct packed {
    logic             all;
    logic [TAG_W-1:0] tag;
  } kill_t;

  // q_user as used by the PLE (internal convention, CSE must not depend on it)
  localparam int USER_SLOT_LSB   = 12;            // [15:12] pose slot
  localparam int USER_SPHERE_LSB = 4;             // [11:4]  sphere id (8 bits, Panda uses 0..69)

  // ---------------------------------------------------------------- environment format v0 (section 5.1)
  localparam int PT_W        = 16;                // point coordinate, unsigned, LSB = 2^-14 m
  localparam int ENV_G       = 20;                // query cells per axis (12.5 cm)
  localparam int ENV_QSHIFT  = 11;                // query cell = q >> 11
  localparam int ENV_FSHIFT  = 9;                 // fine cell  = q >> 9 (3.125 cm)
  localparam int ENV_NCELL   = ENV_G * ENV_G * ENV_G;
  localparam int DIR_AW      = 13;                // DIR index (z*G + y)*G + x < 8000
  localparam int SLOT_W      = 12;                // CELL index, up to 4096 non-empty cells
  localparam int PT_AW       = 16;                // PTS index, up to 65535 points
  localparam int COUNT_W     = 10;                // points per query cell (<= 64 for Panda)
  localparam int AOFF_W      = 11;                // AABB offset from the cell origin
  localparam logic [31:0] W_MIN_LSB = -32'sd19497;  // floor(-1.19 m * 2^14)

  // environment memory ports (INTERFACES.md I3): one read-port set per CU, sync read
  localparam int ENV_RD_LAT  = 1;                 // address at edge t, data valid after edge t+1
  localparam int PT_DATA_W   = 3 * PT_W;          // PTS read data {z, y, x}
  localparam int CELL_DEPTH  = 1024;              // memory depth (format allows 4096); max seen 651
  localparam int PT_DEPTH    = 16384;             // memory depth (format allows 65535); max seen 11670
  localparam int HDR_WORDS   = 9;                 // 0 {err, valid}, 1 n_pts, 2 n_cells, 3-5 glo, 6-8 ghi

  typedef enum logic [1:0] {                      // build-engine write port region
    ENV_HDR  = 2'd0,
    ENV_DIR  = 2'd1,
    ENV_CELL = 2'd2,
    ENV_PTS  = 2'd3
  } env_region_e;

  typedef struct packed {                         // DIR word
    logic              occ;                       // [15]
    logic [2:0]        rsvd;                      // [14:12]
    logic [SLOT_W-1:0] slot;                      // [11:0]
  } dir_t;

  typedef struct packed {                         // CELL word, 128 bits (MSB first)
    logic [35:0]        rsvd;                     // [127:92]
    logic [AOFF_W-1:0]  hi_z;                     // [91:81]
    logic [AOFF_W-1:0]  hi_y;                     // [80:70]
    logic [AOFF_W-1:0]  hi_x;                     // [69:59]
    logic [AOFF_W-1:0]  lo_z;                     // [58:48]
    logic [AOFF_W-1:0]  lo_y;                     // [47:37]
    logic [AOFF_W-1:0]  lo_x;                     // [36:26]
    logic [COUNT_W-1:0] count;                    // [25:16]
    logic [PT_AW-1:0]   base;                     // [15:0]
  } cell_t;

endpackage
/* verilator lint_on UNUSEDPARAM */
