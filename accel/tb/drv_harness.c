// Runs the CPU driver (Platform/accel/sw/ple_drv.c, built with PLE_DRV_HOSTSIM) against the ple_top RTL:
// the test program below runs as a coroutine (ucontext); every register access switches back to tb_drv.sv,
// which performs it over AXI-Lite and resumes the coroutine with the read data. Results are compared with
// golden fx_solve (DPI library). Single-threaded and deterministic.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ucontext.h>
#include "ple_drv.h"
#include "problems.h"

#ifdef __cplusplus
extern "C" {
#endif
int fx_ref_env_load(int i, int flat);
int fx_ref_solve(int i, int seed, int max_iter, int range_h, int *stats, short *sg, int *shift, short *path);
int drv_step(int rdata, int *op, int *addr, int *data);
#ifdef __cplusplus
}
#endif

enum { OP_RD = 0, OP_WR = 1, OP_WAIT_IRQ = 2, OP_IDLE = 3, OP_END = 4 };
static ucontext_t tb_ctx, drv_ctx;
static int started, finished, req_op, req_addr, req_data, resp, errors;

static void yield_req(int op, int addr, int data)
{
    req_op = op; req_addr = addr; req_data = data;
    swapcontext(&drv_ctx, &tb_ctx);
}
uint32_t ple_io_rd(uint32_t off) { yield_req(OP_RD, (int)off, 0); return (uint32_t)resp; }
void ple_io_wr(uint32_t off, uint32_t v) { yield_req(OP_WR, (int)off, (int)v); }
static void wait_irq(void) { yield_req(OP_WAIT_IRQ, 0, 0); }
static void idle(int n) { yield_req(OP_IDLE, 0, n); }

static void check(const char *what, long long got, long long exp)
{
    if (got != exp) {
        if (errors < 20) printf("ERROR drv %s got=%lld expected=%lld\n", what, got, exp);
        errors++;
    }
}

// one run through the driver, compared with golden; use_irq: wait for the interrupt instead of polling
static void run_problem(ple_dev_t *d, int prob, int seed, int iters, int use_irq)
{
    int st[8], gsh[7];
    short sg[14], path[448];
    if (fx_ref_env_load(prob, 0) < 0) { printf("ERROR env %d\n", prob); errors++; return; }
    fx_ref_solve(prob, seed, iters, 1 << 24, st, sg, gsh, path);
    int16_t qs[PLE_DIM], qg[PLE_DIM];
    uint32_t sh[PLE_DIM];
    for (int j = 0; j < PLE_DIM; ++j) {
        qs[j] = ple_q_from_float(problems[prob].start[j]);
        qg[j] = ple_q_from_float(problems[prob].goal[j]);
        check("start conversion", qs[j], sg[j]);
        check("goal conversion", qg[j], sg[PLE_DIM + j]);
    }
    ple_seed_shift((uint32_t)seed, sh);
    for (int j = 0; j < PLE_DIM; ++j) check("seed shift", (int32_t)sh[j], gsh[j]);
    ple_cfg_t c;
    ple_default_cfg(&c);
    c.max_iterations = (uint32_t)iters;
    check("configure", ple_configure(d, &c), 0);
    check("set_problem", ple_set_problem(d, qs, qg, sh), 0);
    ple_start(d, use_irq);
    int status;
    if (use_irq) {
        wait_irq();
        check("irq pending", ple_irq_pending(d), 1);
        status = ple_wait(d, 1);                   // done is already set
        ple_irq_clear(d);
        check("irq cleared", ple_irq_pending(d), 0);
    } else status = ple_wait(d, 0);
    ple_result_t r;
    ple_read_result(d, &r);
    check("status", status, st[0]);
    check("result status", r.status, st[0]);
    check("iterations", r.iterations, st[1]);
    check("size_start", r.size_start, st[2]);
    check("size_goal", r.size_goal, st[3]);
    check("dd_rejects", r.dd_rejects, st[4]);
    check("edges", r.edges, st[5]);
    check("nn_queries", r.nn_queries, st[6]);
    if (r.cycles == 0 || r.poses == 0 || r.queries == 0 || r.self_pairs < 21) { printf("ERROR drv zero counter\n"); errors++; }
    if (st[0] == PLE_SOLVED) {
        static int16_t p[64][PLE_DIM];
        uint32_t n = ple_read_path(d, p, 64);
        check("path_len", n, st[7]);
        for (uint32_t i = 0; i < n && i < 64; ++i)
            for (int j = 0; j < PLE_DIM; ++j) check("path", p[i][j], path[i * PLE_DIM + j]);
    }
    printf("DRV run problem=%d seed=%d irq=%d status=%d iterations=%u cycles=%u path_len=%u\n", prob, seed, use_irq,
           r.status, r.iterations, r.cycles, r.path_len);
}

static void drv_main(void)
{
    static ple_dev_t dev;                          // base unused in host simulation
    const int plist[4] = {1, 160, 410, 650};
    for (int p = 0; p < 4; ++p) run_problem(&dev, plist[p], p, 3000, p & 1);
    // abort: long run, abort after a while, then a normal run must still match
    {
        int st[8], gsh[7];
        short sg[14], path[448];
        uint32_t sh[PLE_DIM];
        fx_ref_env_load(plist[0], 0);
        fx_ref_solve(plist[0], 9, 3000, 1 << 24, st, sg, gsh, path);
        ple_cfg_t c;
        ple_default_cfg(&c);
        int16_t qs[PLE_DIM], qg[PLE_DIM];
        for (int j = 0; j < PLE_DIM; ++j) { qs[j] = sg[j]; qg[j] = sg[PLE_DIM + j]; }
        ple_seed_shift(9, sh);
        ple_configure(&dev, &c);
        ple_set_problem(&dev, qs, qg, sh);
        ple_start(&dev, 0);
        idle(3000);
        check("busy before abort", ple_busy(&dev), 1);
        check("configure while busy", ple_configure(&dev, &c), -1);
        ple_abort(&dev);
        check("aborted", ple_wait(&dev, 0), PLE_ABORTED);
        run_problem(&dev, plist[0], 9, 3000, 0);
    }
    printf("DRV done errors=%d\n", errors);
    finished = 1;
    swapcontext(&drv_ctx, &tb_ctx);
}

// called by tb_drv.sv: resumes the driver with the last read data; returns the next request
int drv_step(int rdata, int *op, int *addr, int *data)
{
    static char *stack;
    if (!started) {
        stack = (char *)malloc(16u << 20);
        getcontext(&drv_ctx);
        drv_ctx.uc_stack.ss_sp = stack;
        drv_ctx.uc_stack.ss_size = 16u << 20;
        drv_ctx.uc_link = 0;
        makecontext(&drv_ctx, drv_main, 0);
        started = 1;
    }
    if (finished) { *op = OP_END; *data = errors; return 0; }
    resp = rdata;
    swapcontext(&tb_ctx, &drv_ctx);
    if (finished) { *op = OP_END; *addr = 0; *data = errors; return 0; }
    *op = req_op; *addr = req_addr; *data = req_data;
    return 1;
}
