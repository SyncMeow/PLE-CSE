# CSE Compute Unit Pipeline Report

> This report describes the implemented single-CU microarchitecture in `rtl/cse/cse_cu.sv` as of November 10, 2026. It focuses on what each pipeline stage does, how stages communicate, and how cancellation and backpressure are handled.

## 1. Purpose of a Compute Unit

A CSE compute unit receives one sphere query at its input and determines whether any point in the selected environment lies inside or on that sphere:

```text
(cx - px)^2 + (cy - py)^2 + (cz - pz)^2 <= r^2
```

All calculations use fixed-width integer arithmetic. Equality is included, so a point exactly on the sphere boundary produces `HIT`.

The CU does not store a private environment. It accesses external DIR, CELL, and PTS memories through synchronous read ports. A CU can contain several sphere queries at the same time, provided they occupy different pipeline stages or inter-stage FIFOs. However, each major stage operates on only one active sphere context at a time.

## 2. Pipeline Overview

```text
Query input
    │
    ▼
┌──────────────────────────────────────────────────────────────┐
│ S0: Query preprocessing                                     │
│     Radius square, sphere AABB, grid range, global culling  │
└───────────────────────────────┬──────────────────────────────┘
                                ▼
┌──────────────────────────────────────────────────────────────┐
│ S1: Voxel enumeration                                       │
│     X-fastest traversal, DIR address generation, END token  │
└───────────────────────────────┬──────────────────────────────┘
                                ▼
                         Voxel FIFO
                                │
                                ▼
┌──────────────────────────────────────────────────────────────┐
│ S2: DIR/CELL lookup and local culling                       │
│     Empty-voxel skip, cell AABB reconstruction and reject   │
└───────────────────────────────┬──────────────────────────────┘
                                ▼
                          Cell FIFO
                                │
                                ▼
┌──────────────────────────────────────────────────────────────┐
│ S3: Point traversal and distance pipeline                   │
│     PTS reads, subtraction, square, sum, compare and drain  │
└───────────────────────────────┬──────────────────────────────┘
                                ▼
┌──────────────────────────────────────────────────────────────┐
│ S4: Response generation and holding                         │
│     ERROR/HIT/KILLED/MISS selection and valid/ready output  │
└───────────────────────────────┬──────────────────────────────┘
                                ▼
                         Response output
```

The two internal FIFOs decouple variable-rate stages:

- The Voxel FIFO separates voxel enumeration from synchronous DIR/CELL lookup.
- The Cell FIFO separates memory lookup and culling from point traversal.

Both FIFOs preserve token order. This is important because an `END` token marks the end of each sphere's token stream.

## 3. Query Context and Cancellation Identity

Before a query enters the CU, `cse_top` attaches a 32-bit `birth_gen` to it. The CU copies the query fields and this generation into `cse_ctx_t`. The context contains:

- `tag` and `user` for response matching.
- `kill_on_hit` as query metadata.
- `env_buf` for environment selection.
- Sphere center `cx/cy/cz`.
- Squared radius `r2`.
- Integer sphere bounding box `bmin` and `bmax`.
- `birth_gen` for cancellation detection.
- An environment error flag.

Cancellation is tested with:

```systemverilog
birth_gen != kill_gen[tag]
```

The comparison is wrapped by `ctx_cancelled()`. Every stage carries the same context, so a kill can be detected without editing tokens already stored in a FIFO.

## 4. S0 — Query Preprocessing

### 4.1 Input and storage

S0 accepts `cse_query_meta_t` through `q_valid/q_ready`. It contains one elastic register, represented by `s0_valid` and `s0_payload`.

The input is ready when:

```text
reset is inactive, and
the S0 register is empty or S1 can take the stored query
```

This permits a new query to replace the previous S0 entry in the same cycle that S1 consumes it.

### 4.2 Arithmetic

For each query, S0 calculates:

```text
r2 = radius × radius

bmin = center - radius
bmax = center + radius

grid_lo = (center - radius) >>> ENV_QSHIFT
grid_hi = (center + radius) >>> ENV_QSHIFT
```

`ENV_QSHIFT` is 11, corresponding to a 12.5 cm query voxel. The shift is arithmetic, so negative coordinates are rounded toward negative infinity.

The calculated grid indices are clamped to `[0, 19]`, matching the 20×20×20 DIR grid.

### 4.3 Width handling

The sphere center is signed 18-bit and the radius is unsigned 14-bit. S0 extends both operands to signed 19-bit values before addition or subtraction. This is required for spheres with negative centers or centers outside the workspace.

The radius square is stored in 28 bits:

```text
maximum r2 = 8192^2 = 67,108,864
```

### 4.4 Global culling and error detection

S0 compares the sphere AABB against the environment-wide `glo/ghi` AABB from the selected header. It also detects axes that lie completely outside the 20-cell grid.

`skip_search` is asserted when any of the following is true:

- The selected environment header is not valid.
- The selected environment header reports an error.
- The environment contains zero points.
- The sphere AABB does not overlap the global environment AABB.
- At least one sphere axis lies completely outside the query grid.

An invalid or erroneous header also sets `ctx.error`. A skipped query is not discarded; it is converted into an END-only token stream so that S4 still produces exactly one response.

### 4.5 Kill behavior

S0 does not modify its stored payload when a kill arrives. The query retains its original `birth_gen`. When S1 takes the query, S1 compares that generation against the current global generation. Therefore, a query waiting in S0 is still cancellable, although cancellation is acted upon at the S0-to-S1 boundary.

### 4.6 Output

S0 outputs a `cse_ball_t` containing:

- The complete query context.
- Clamped low and high grid coordinates for all three axes.
- The `skip_search` flag.

## 5. S1 — Voxel Enumeration

### 5.1 Function

S1 converts one sphere's grid range into an ordered stream of voxel tokens. It holds one active sphere in `s1_ball` and three counters: `s1_gx`, `s1_gy`, and `s1_gz`.

Enumeration uses X as the fastest-changing coordinate:

```text
for gz = lo_z to hi_z
  for gy = lo_y to hi_y
    for gx = lo_x to hi_x
      emit voxel(gx, gy, gz)
```

The linear DIR address is:

```text
dir_addr = (gz × 20 + gy) × 20 + gx
```

At the maximum legal radius, each axis may cover 9 cells, producing at most 729 voxel tokens.

### 5.2 END token

After the final voxel, S1 emits exactly one `CSE_VOXEL_END` token. The END token carries the same context as all other tokens for that sphere.

If S0 set `skip_search`, or if the query is already cancelled when S1 starts, S1 emits only the END token and does not enumerate any normal voxel.

END tokens make completion independent of how many normal tokens later stages discard. S2 and S3 can skip empty cells, rejected cells, cancelled work, or work after a HIT while still knowing where the sphere ends.

### 5.3 Backpressure

S1 holds its current coordinates and output token whenever the Voxel FIFO is full. Counters advance only on `s1_out_valid && s1_out_ready`.

### 5.4 Kill behavior

S1 checks cancellation:

- When it first loads an S0 query.
- After each accepted voxel token.

If a kill occurs while a voxel token is stalled, S1 cannot change that already-presented token because valid/ready payload stability must be preserved. After the token is accepted, S1 switches to the END state. S2 rechecks the token context before issuing a memory read, so the extra accepted token does not cause invalid search work.

## 6. Voxel FIFO

The Voxel FIFO stores `cse_voxel_t` tokens between S1 and S2. Its default depth is 2 and is controlled by `VOXEL_FIFO_DEPTH`.

Each normal token contains:

- Token kind.
- Full sphere context.
- `gx/gy/gz`.
- Precomputed DIR address.

The FIFO does not contain a separately updated kill bit. Cancellation is recognized when S2 reads the context and calls `ctx_cancelled()`. This avoids scanning and rewriting every internal token on each kill.

Because S1 may begin the next sphere after inserting the previous sphere's END, the FIFO can contain tokens belonging to adjacent spheres while preserving their order.

## 7. S2 — DIR/CELL Lookup and Local Culling

### 7.1 State machine

S2 processes one voxel at a time using four states:

```text
S2_IDLE → S2_DIR_WAIT → S2_CELL_WAIT → S2_OUT
```

The external memory has one-cycle synchronous read latency.

### 7.2 S2_IDLE

When a normal voxel is available and is not cancelled, S2:

1. Drives `mem_dir_addr` with the token's precomputed DIR address.
2. Captures the voxel context in `s2_voxel`.
3. Moves to `S2_DIR_WAIT`.

If the token is already cancelled, S2 consumes and drops it without issuing a DIR read.

If the input is `CSE_VOXEL_END`, S2 converts it into `CSE_CELL_END` and moves directly to `S2_OUT`.

### 7.3 S2_DIR_WAIT

The DIR result is interpreted as `dir_t`:

- `occ=0`: the voxel is empty, so S2 returns to `S2_IDLE`.
- `occ=1`: `slot` identifies the CELL entry, so S2 drives `mem_cell_addr` and moves to `S2_CELL_WAIT`.

Cancellation is checked again before the CELL read. A cancelled voxel returns to idle without additional work.

### 7.4 S2_CELL_WAIT

The CELL word provides:

- `base`: first point address.
- `count`: number of points in the voxel.
- `lo_x/lo_y/lo_z` and `hi_x/hi_y/hi_z`: local point-cloud AABB offsets.

S2 reconstructs the absolute cell AABB:

```text
voxel_origin = grid_coordinate << ENV_QSHIFT
cell_lo      = voxel_origin + local_lo
cell_hi      = voxel_origin + local_hi
```

It then performs three interval-overlap tests between the sphere AABB and cell AABB. The cell is rejected if any axis is disjoint.

This is a conservative sphere-to-cell filter. It may keep a cell whose AABB overlaps the sphere AABB even when the actual sphere does not reach the cell, but it cannot remove a cell containing a real hit.

The cell is discarded when:

- The sphere has been cancelled.
- `count` is zero.
- The sphere AABB and cell AABB are disjoint.

Otherwise S2 builds a `CSE_CELL` token containing the context, point base, and point count.

### 7.5 S2_OUT

S2 holds the CELL or CELL_END token until the Cell FIFO accepts it. Payload and valid remain stable during backpressure.

If a kill arrives after a CELL token has already been formed, S2 does not rewrite the stalled token. S3 checks the context again before starting its point reads.

## 8. Cell FIFO

The Cell FIFO stores `cse_cell_token_t` entries between S2 and S3. Its default depth is 2 and is controlled by `CELL_FIFO_DEPTH`.

A normal CELL token contains the complete sphere context and a contiguous PTS range. A CELL_END token contains the context and marks the end of the current sphere.

The FIFO preserves the ordering:

```text
all CELL tokens for sphere A
CELL_END for sphere A
all CELL tokens for sphere B
CELL_END for sphere B
```

Like the Voxel FIFO, it carries the original generation and relies on the consumer to perform a current cancellation check.

## 9. S3 — Point Traversal and Distance Pipeline

### 9.1 Sphere and cell control

S3 owns one active sphere context in `s3_ctx`. It tracks:

- Whether a sphere is active.
- Whether a cell is currently issuing points.
- Whether the sphere's END token has arrived.
- Whether a hit has already been detected.
- The current PTS address and remaining point count.
- The number of issued points that have not completed comparison.

S3 accepts CELL tokens in order. A new sphere begins when S3 is inactive and accepts either its first CELL token or its CELL_END token. Subsequent CELL tokens are accepted only when the previous cell has finished issuing points.

### 9.2 Point issue

For an active cell, S3 can issue at most one PTS read per cycle. A read is issued only when:

- The current sphere is not cancelled.
- No HIT has been recorded.
- The number of pending points is below `MAX_PTS_OUTSTANDING`, or one comparison is completing in the same cycle.

The default outstanding limit is 8.

PTS addresses increment sequentially from `base` for `count` points. The PTS memory returns `{z, y, x}` one cycle after the address is presented.

### 9.3 Distance pipeline

The distance datapath is divided into registered steps:

| Step | Operation | Main width |
|---|---|---:|
| Memory return | Unpack unsigned `px/py/pz` | 16 bits per axis |
| D0 | `dx=cx-px`, `dy=cy-py`, `dz=cz-pz` | signed 19 bits |
| D1 | `sx=dx²`, `sy=dy²`, `sz=dz²` | 38 bits each |
| D2 | `d2=sx+sy+sz` | 40 bits |
| Compare | `d2 <= r2` | inclusive boundary |

The widened intermediate values prevent overflow and avoid accidental signed multiplication behavior.

`s3_pending` increments when a point is issued and decrements when its D2 result completes. Simultaneous issue and completion leave the count unchanged.

### 9.4 HIT handling

When any completed point satisfies `d2 <= r2`:

- `s3_hit_seen` is set.
- No new point is issued.
- The active cell is stopped.
- Later CELL tokens for the same sphere are consumed without starting new point reads.
- Already-issued points continue through the pipeline until drained.

Stopping early reduces point-memory traffic while preserving a clean boundary between sphere contexts.

### 9.5 Kill handling

When cancellation is detected:

- `pt_issue` is immediately disabled.
- The current cell is marked inactive.
- Later CELL tokens are consumed without issuing points.
- Already-issued points remain in the distance pipeline and are drained.

An in-flight point may still produce a real HIT after the kill. The interface contract permits a killed query to return a true result, so a detected HIT is retained and has priority over KILLED.

### 9.6 Completion condition

A sphere completes only when all of the following are true:

- A sphere context is active.
- Its CELL_END token has been received.
- No cell is issuing additional points.
- All issued points have drained from the pipeline.
- S4 can accept a new result.

This condition prevents point data from one sphere from being interpreted using the next sphere's context.

## 10. S4 — Response Generation and Holding

S4 contains one response register. When S3 completes a sphere, S4 stores its tag, user, and final status.

Status priority is:

```text
1. Environment error  → ST_ERROR
2. Hit detected       → ST_HIT
3. Query cancelled    → ST_KILLED
4. Otherwise          → ST_MISS
```

The priority has two important consequences:

- An unusable environment never becomes MISS or KILLED.
- A confirmed HIT from an already-issued point remains HIT even if the query was also killed.

Once `s4_valid` is asserted, S4 does not reconsider later kill events. It holds the complete response unchanged until `r_valid && r_ready`. This is required by the output valid/ready contract.

S4 can consume the previous response and accept a new S3 completion in the same cycle.

## 11. Multiple Spheres Inside One CU

The CU supports inter-stage overlap rather than multiple sphere contexts inside the same execution stage. A possible steady-state arrangement is:

| Location | Possible sphere |
|---|---|
| S4 | Sphere A response waiting for acceptance |
| S3 | Sphere B point traversal |
| Cell FIFO / S2 | Sphere C cells or voxel lookup |
| Voxel FIFO / S1 | Sphere D voxel enumeration |
| S0 | Sphere E waiting to enter S1 |

The exact number of simultaneously resident sphere contexts depends on FIFO occupancy and token boundaries. It is not a fixed five-sphere capacity, because several locations may contain tokens from the same sphere.

Important throughput properties are:

- S0 can hold one query ahead of S1.
- S1 enumerates only one sphere at a time.
- S2 performs only one DIR/CELL transaction sequence at a time.
- S3 performs point reads for only one sphere at a time.
- The distance pipeline may contain multiple points, but all belong to the same S3 sphere.
- Multiple CUs are required for true parallel point traversal of different spheres.

## 12. Shared Environment-Buffer Constraint

Each CU has separate DIR, CELL, and PTS address/data signals, but they share one `mem_rd_buf` selector. The selector priority is:

```text
S3 active sphere
else S2 active voxel
else Voxel FIFO head
else S1 active sphere
```

This is safe under the system contract that all outstanding queries use the same environment buffer and the active environment changes only when the complete CSE is idle.

If different overlapping spheres were allowed to use different `env_buf` values, an S3 sphere could force its buffer selection while S2 is reading DIR or CELL for another sphere. Supporting that mode would require independent buffer selection for each memory request, or additional serialization. The current CU does not implement that mode.

## 13. Backpressure Behavior

Backpressure is propagated without dropping work:

- A full Voxel FIFO stops S1 coordinate advancement.
- A busy S2 stops Voxel FIFO removal.
- A full Cell FIFO holds S2 in `S2_OUT`.
- An active S3 cell stops acceptance of the next CELL token.
- A full S4 stops S3 completion.
- A receiver stall keeps the S4 response stable.

The CU's `q_ready` eventually deasserts when the pipeline fills from the output backward.

## 14. CU Idle and Error Signals

The CU increments an internal outstanding counter when it accepts a query and decrements it when its response is accepted. Its local `cse_idle` is true when this count is zero.

At the array level, `cse_top` uses a separate global counter for the external `cse_idle`. The global counter includes queries in the input FIFO, dispatcher, all CUs, receiver, and Response FIFO.

`cse_err` is sticky. S4 sets it when a query completes with an invalid or erroneous environment header, and reset clears it.

## 15. Synthesis Boundary

The functional S0–S4 datapath, FIFOs, cancellation checks, memory interfaces, response logic, idle counter, and error flag are synthesizable.

Code under `` `ifndef SYNTHESIS `` contains simulation-only statistics:

- Number of queries and responses.
- DIR, CELL, and PTS read counts.
- A final `CSE_CU_STATS` report.

These counters do not participate in the functional datapath and are removed for synthesis.

## 16. Current Verification Status

The implemented CU has been exercised by:

- Golden trace replay with `N_CU=1`, completing 1,874 queries with zero mismatch and zero checker errors.
- Complete array trace replay with `N_CU=1/2/3/4/8`.
- Exact-boundary testing where `d² == r²` must produce HIT.
- A comparator mutant changing `<=` to `<`, which is correctly detected.
- A dense single-CU workload of 729 voxels and 46,656 point evaluations, completing in 47,396 cycles.
- Kill tests covering front-end cancellation, Q/K in the same cycle, K(all), disabled-kill mode, and response stability after completion.

Static inspection confirms cancellation checks at the S0/S1 boundary, throughout S1, throughout the S2 memory FSM, at S3 point issue and CELL acceptance, and during S4 status formation.

The current test suite does not yet force a kill at every named internal state of one CU. A stage-directed `N_CU=1` test should be added to explicitly inject kills while a query is in:

- S0 waiting behind S1.
- S1 enumeration.
- `S2_DIR_WAIT`.
- `S2_CELL_WAIT`.
- S3 with point reads pending.
- S4 under output backpressure.

Such a test would strengthen dynamic coverage of behavior that is currently established by RTL inspection and broader end-to-end tests.

## 17. Summary

The CU is a five-stage, token-based search pipeline. S0 establishes an exact fixed-point search context, S1 enumerates candidate voxels, S2 removes empty or conservatively rejected cells, S3 performs exact point-to-sphere distance tests, and S4 produces a stable response.

The design supports multiple sphere queries resident in different stages, but each major stage has one active sphere context. Generation-based cancellation remains effective across stored tokens because each token carries the original context and downstream stages recheck it before performing work. END tokens and pipeline draining guarantee that every accepted query reaches exactly one clean completion even after culling, HIT, kill, or backpressure.
