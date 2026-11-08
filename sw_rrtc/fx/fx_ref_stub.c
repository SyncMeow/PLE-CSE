// Symbols the golden library needs outside the planner driver (DPI build of libfxref.a).
#include <stdio.h>
#include <stdlib.h>
#include "stats.h"

stats_t st;                                     // counters touched by src/env.c

void fatal(const char *msg)
{
    fprintf(stderr, "fatal: %s\n", msg);
    exit(2);
}
