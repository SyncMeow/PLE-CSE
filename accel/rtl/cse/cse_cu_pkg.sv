// Private payloads for the single-CU CSE streaming pipeline.
package cse_cu_pkg;
  import ple_cse_pkg::*;

  localparam int KILL_GEN_W = 32;

  typedef struct packed {
    query_t                  query;
    logic [KILL_GEN_W-1:0]  birth_gen;
  } cse_query_meta_t;

  typedef struct packed {
    logic [TAG_W-1:0]          tag;
    logic [USER_W-1:0]         user;
    logic                      kill_on_hit;
    logic                      env_buf;
    logic signed [COORD_W-1:0] cx;
    logic signed [COORD_W-1:0] cy;
    logic signed [COORD_W-1:0] cz;
    logic [27:0]               r2;
    logic signed [18:0]        bmin_x;
    logic signed [18:0]        bmin_y;
    logic signed [18:0]        bmin_z;
    logic signed [18:0]        bmax_x;
    logic signed [18:0]        bmax_y;
    logic signed [18:0]        bmax_z;
    logic [KILL_GEN_W-1:0]     birth_gen;
    logic                      error;
  } cse_ctx_t;

  typedef struct packed {
    cse_ctx_t       ctx;
    logic [2:0][4:0] lo;
    logic [2:0][4:0] hi;
    logic            skip_search;
  } cse_ball_t;

  typedef enum logic { CSE_VOXEL, CSE_VOXEL_END } cse_voxel_kind_e;
  typedef struct packed {
    cse_voxel_kind_e kind;
    cse_ctx_t        ctx;
    logic [4:0]      gx;
    logic [4:0]      gy;
    logic [4:0]      gz;
    logic [DIR_AW-1:0] dir_addr;
  } cse_voxel_t;

  typedef enum logic { CSE_CELL, CSE_CELL_END } cse_cell_kind_e;
  typedef struct packed {
    cse_cell_kind_e  kind;
    cse_ctx_t        ctx;
    logic [PT_AW-1:0] base;
    logic [COUNT_W-1:0] count;
  } cse_cell_token_t;

  typedef struct packed {
    logic signed [18:0] dx;
    logic signed [18:0] dy;
    logic signed [18:0] dz;
  } cse_dist_d0_t;

  typedef struct packed {
    logic [37:0] sx;
    logic [37:0] sy;
    logic [37:0] sz;
  } cse_dist_d1_t;

  typedef struct packed {
    logic [39:0] d2;
  } cse_dist_d2_t;

endpackage
