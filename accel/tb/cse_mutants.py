#!/usr/bin/env python3
"""Create the strict-comparator CSE mutant used by test-cse-mutant."""

from pathlib import Path
import sys


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit("usage: cse_mutants.py INPUT OUTPUT")
    source = Path(sys.argv[1]).read_text()
    original = "assign d2_hit  = d2_payload.d2 <= {12'b0, s3_ctx.r2};"
    mutant = "assign d2_hit  = d2_payload.d2 <  {12'b0, s3_ctx.r2};"
    if source.count(original) != 1:
        raise SystemExit("expected one distance comparator in cse_cu.sv")
    Path(sys.argv[2]).write_text(source.replace(original, mutant))


if __name__ == "__main__":
    main()
