# Platform/accel -- PLE / CSE RTL (records/20261107-ple-cse-design.md section 13)
BUILD   ?= $(abspath ../build/accel)
VERI    ?= verilator
PKG     := rtl/common/ple_cse_pkg.sv
CSE_PKG := rtl/cse/cse_cu_pkg.sv
CSE_RTL := $(CSE_PKG) rtl/cse/cse_fifo.sv rtl/cse/cse_query_fifo.sv rtl/cse/cse_dispatcher.sv \
           rtl/cse/cse_receiver.sv rtl/cse/cse_cu.sv rtl/cse/cse_env_mem.sv rtl/cse/cse_top.sv \
           rtl/cse/cse_system.sv
VFLAGS  := -Wall --assert -Irtl/common
PLE_PKG := rtl/ple/ple_pkg.sv
FXDIR   := $(abspath ../sw_rrtc/fx)
FKUGEN  := $(abspath ../sw_rrtc/tools/gen_fx_fku_rtl.py)
SCUGEN  := $(abspath ../sw_rrtc/tools/gen_fx_scu_rtl.py)
SWDIR   := $(abspath ../sw_rrtc)
# testbenches call the golden model through DPI-C (fx_ref.c includes fx_plan.c)
# golden model as a C library (gcc): DPI needs C linkage, and the golden C is not valid C++
FXLIB   := $(BUILD)/libfxref.a
SRCDIR  := $(abspath ../sw_rrtc/src)
GENDIR  := $(abspath ../sw_rrtc/gen)
FXLIB_SRC := $(FXDIR)/fx_ref.c $(FXDIR)/fx_env.c $(FXDIR)/fx_ref_stub.c $(SRCDIR)/env.c $(GENDIR)/problems_panda.c
DPI     := -LDFLAGS $(FXLIB) -LDFLAGS -lm
TBFLAGS := --binary --timing --assert -Wno-fatal -Irtl/common

.PHONY: test-ple-cse test-cse test-cse-unit test-cse-mutant test-ple-drv lint test-checker test-env-mem test-ple-unit test-ple-core test-ple-top test-ple-mutants clean

# every RTL file must lint clean
lint:
	$(VERI) --lint-only $(VFLAGS) $(PKG) $(CSE_RTL) --top-module cse_top
	$(VERI) --lint-only $(VFLAGS) $(PKG) $(CSE_RTL) --top-module cse_system
	$(VERI) --lint-only $(VFLAGS) $(PKG) $(CSE_PKG) rtl/cse/cse_fifo.sv rtl/cse/cse_cu.sv --top-module cse_cu
	$(VERI) --lint-only $(VFLAGS) $(PKG) $(CSE_PKG) rtl/cse/cse_query_fifo.sv --top-module cse_query_fifo
	$(VERI) --lint-only $(VFLAGS) $(PKG) $(CSE_PKG) rtl/cse/cse_dispatcher.sv --top-module cse_dispatcher
	$(VERI) --lint-only $(VFLAGS) $(PKG) rtl/cse/cse_receiver.sv --top-module cse_receiver
	$(VERI) --lint-only $(VFLAGS) $(PKG) rtl/cse/cse_env_mem.sv --top-module cse_env_mem
	$(VERI) --lint-only $(VFLAGS) $(PKG) rtl/ple/ple_robot_pkg.sv rtl/ple/ple_qd.sv --top-module ple_qd
	$(VERI) --lint-only $(VFLAGS) $(PKG) $(EVU_RTL) rtl/ple/ple_evu.sv --top-module ple_evu
	$(VERI) --lint-only $(VFLAGS) $(PKG) $(CORE_RTL) --top-module ple_core
	$(VERI) --lint-only $(VFLAGS) $(PKG) $(CORE_RTL) rtl/ple/ple_top.sv --top-module ple_top
	@for m in ple_smp ple_tree ple_nnu ple_rkg ple_isqrt ple_div7 ple_fku ple_scu; do $(VERI) --lint-only $(VFLAGS) $(PLE_PKG) rtl/ple/$$m.sv --top-module $$m || exit 1; done
	$(VERI) --lint-only $(VFLAGS) -Wno-BLKSEQ $(PKG) tb/ple_cse_if_checker.sv --top-module ple_cse_if_checker

# interface checker self-test: clean traffic passes, each injected violation (bug 1..8) is caught
$(BUILD)/tb_if_checker/Vtb_if_checker: $(PKG) tb/ple_cse_if_checker.sv tb/tb_if_checker.sv
	@mkdir -p $(BUILD)/tb_if_checker
	$(VERI) --binary --timing --assert -Wno-fatal -Irtl/common -Mdir $(BUILD)/tb_if_checker \
	    --top-module tb_if_checker $(PKG) tb/ple_cse_if_checker.sv tb/tb_if_checker.sv -o Vtb_if_checker >/dev/null

$(BUILD)/tb_if_checker_ak0/Vtb_if_checker: $(PKG) tb/ple_cse_if_checker.sv tb/tb_if_checker.sv
	@mkdir -p $(BUILD)/tb_if_checker_ak0
	$(VERI) --binary --timing --assert -Wno-fatal -Irtl/common -Mdir $(BUILD)/tb_if_checker_ak0 -GAK=0 \
	    --top-module tb_if_checker $(PKG) tb/ple_cse_if_checker.sv tb/tb_if_checker.sv -o Vtb_if_checker >/dev/null

# with auto-kill an injected KILLED can be indistinguishable from a legal one, so bug 4 runs without it
test-checker: $(BUILD)/tb_if_checker/Vtb_if_checker $(BUILD)/tb_if_checker_ak0/Vtb_if_checker
	@for b in 0 1 2 3 5 6 7 8; do $< +bug=$$b > $(BUILD)/tb_if_checker/bug$$b.log; grep '^TB' $(BUILD)/tb_if_checker/bug$$b.log; grep -m1 ERROR $(BUILD)/tb_if_checker/bug$$b.log; done
	@for b in 0 4; do $(BUILD)/tb_if_checker_ak0/Vtb_if_checker +bug=$$b > $(BUILD)/tb_if_checker_ak0/bug$$b.log; grep '^TB' $(BUILD)/tb_if_checker_ak0/bug$$b.log; grep -m1 ERROR $(BUILD)/tb_if_checker_ak0/bug$$b.log; done

# cse_env_mem: load two golden-model traces, read everything back, diff against the source files
TRACE ?= $(abspath ../build/sw_rrtc_host/cse_trace)
$(BUILD)/tb_env_mem/Vtb_env_mem: $(PKG) rtl/cse/cse_env_mem.sv tb/tb_env_mem.sv
	@mkdir -p $(BUILD)/tb_env_mem
	$(VERI) --binary --timing -Wno-fatal -Irtl/common -Mdir $(BUILD)/tb_env_mem \
	    --top-module tb_env_mem $(PKG) rtl/cse/cse_env_mem.sv tb/tb_env_mem.sv -o Vtb_env_mem >/dev/null

# single buffer (the default of cse_env_mem): buffer 0 as above, buffer 1 must read invalid and 0
$(BUILD)/tb_env_mem_1buf/Vtb_env_mem: $(PKG) rtl/cse/cse_env_mem.sv tb/tb_env_mem.sv
	@mkdir -p $(BUILD)/tb_env_mem_1buf
	$(VERI) --binary --timing -Wno-fatal -Irtl/common -Mdir $(BUILD)/tb_env_mem_1buf -GN_BUF=1 \
	    --top-module tb_env_mem $(PKG) rtl/cse/cse_env_mem.sv tb/tb_env_mem.sv -o Vtb_env_mem >/dev/null

test-env-mem: $(BUILD)/tb_env_mem/Vtb_env_mem $(BUILD)/tb_env_mem_1buf/Vtb_env_mem
	$< +env0=$(TRACE)/s0_p1 +env1=$(TRACE)/s0_p2 +out=$(BUILD)/tb_env_mem | grep -E 'TB|cse_env_mem'
	@for f in dir cell ptx pty ptz; do \
	  cmp -s $(BUILD)/tb_env_mem/buf0_$$f.hex $(TRACE)/s0_p1_$$f.hex && echo "buf0 $$f OK" || echo "buf0 $$f DIFF"; \
	  cmp -s $(BUILD)/tb_env_mem/buf1_$$f.hex $(TRACE)/s0_p2_$$f.hex && echo "buf1 $$f OK" || echo "buf1 $$f DIFF"; done
	$(BUILD)/tb_env_mem_1buf/Vtb_env_mem +env0=$(TRACE)/s0_p1 +env1=$(TRACE)/s0_p2 +out=$(BUILD)/tb_env_mem_1buf | grep -E 'TB|cse_env_mem'
	@for f in dir cell ptx pty ptz; do \
	  cmp -s $(BUILD)/tb_env_mem_1buf/buf0_$$f.hex $(TRACE)/s0_p1_$$f.hex && echo "1buf buf0 $$f OK" || echo "1buf buf0 $$f DIFF"; done
	@for w in 1 2 3 5 6 7; do $(BUILD)/tb_env_mem/Vtb_env_mem +bad_write=$$w +env0=$(TRACE)/s0_p1 +env1=$(TRACE)/s0_p2 2>&1 \
	  | grep -q 'table written while valid' && echo "table write while valid ($$w): reported" || echo "table write while valid ($$w): DIFF"; done
	@mkdir -p $(BUILD)/tb_env_mem_small
	@$(VERI) --binary --timing -Wno-fatal -Irtl/common -Mdir $(BUILD)/tb_env_mem_small -GN_CELL=512 -GN_PT=8192 -GN_BUF=2 \
	    --top-module tb_env_mem $(PKG) rtl/cse/cse_env_mem.sv tb/tb_env_mem.sv -o V >/dev/null
	@$(BUILD)/tb_env_mem_small/V +env0=$(TRACE)/s0_p1 +env1=$(TRACE)/s0_p2 +out=$(BUILD)/tb_env_mem_small > /dev/null
	@for f in dir cell ptx pty ptz; do \
	  cmp -s $(BUILD)/tb_env_mem_small/buf0_$$f.hex $(TRACE)/s0_p1_$$f.hex && echo "depth 512/8192 buf0 $$f OK" || echo "depth 512/8192 buf0 $$f DIFF"; done

# Full-array golden trace replay with randomized response backpressure.
$(BUILD)/tb_cse_trace/Vtb_cse_trace: $(PKG) $(CSE_RTL) tb/ple_cse_if_checker.sv tb/tb_cse_trace.sv
	@mkdir -p $(BUILD)/tb_cse_trace
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_cse_trace --top-module tb_cse_trace \
	    $(PKG) $(CSE_RTL) tb/ple_cse_if_checker.sv tb/tb_cse_trace.sv -o Vtb_cse_trace >/dev/null

$(BUILD)/tb_cse_qfifo3/Vtb_cse_query_fifo: $(PKG) $(CSE_PKG) rtl/cse/cse_query_fifo.sv tb/tb_cse_query_fifo.sv
	@mkdir -p $(BUILD)/tb_cse_qfifo3
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_cse_qfifo3 --top-module tb_cse_query_fifo \
	    $(PKG) $(CSE_PKG) rtl/cse/cse_query_fifo.sv tb/tb_cse_query_fifo.sv -o Vtb_cse_query_fifo >/dev/null

$(BUILD)/tb_cse_qfifo1/Vtb_cse_query_fifo: $(PKG) $(CSE_PKG) rtl/cse/cse_query_fifo.sv tb/tb_cse_query_fifo.sv
	@mkdir -p $(BUILD)/tb_cse_qfifo1
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_cse_qfifo1 -GD=1 --top-module tb_cse_query_fifo \
	    $(PKG) $(CSE_PKG) rtl/cse/cse_query_fifo.sv tb/tb_cse_query_fifo.sv -o Vtb_cse_query_fifo >/dev/null

$(BUILD)/tb_cse_arbiters/Vtb_cse_arbiters: $(PKG) $(CSE_PKG) rtl/cse/cse_dispatcher.sv rtl/cse/cse_receiver.sv tb/tb_cse_arbiters.sv
	@mkdir -p $(BUILD)/tb_cse_arbiters
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_cse_arbiters --top-module tb_cse_arbiters \
	    $(PKG) $(CSE_PKG) rtl/cse/cse_dispatcher.sv rtl/cse/cse_receiver.sv \
	    tb/tb_cse_arbiters.sv -o Vtb_cse_arbiters >/dev/null

test-cse-unit: $(BUILD)/tb_cse_qfifo1/Vtb_cse_query_fifo $(BUILD)/tb_cse_qfifo3/Vtb_cse_query_fifo \
               $(BUILD)/tb_cse_arbiters/Vtb_cse_arbiters
	@$< | grep TB_CSE_QFIFO
	@$(BUILD)/tb_cse_qfifo3/Vtb_cse_query_fifo | grep TB_CSE_QFIFO
	@$(BUILD)/tb_cse_arbiters/Vtb_cse_arbiters | grep TB_CSE_ARBITERS

$(BUILD)/tb_cse_trace_1/Vtb_cse_trace: $(PKG) $(CSE_RTL) tb/ple_cse_if_checker.sv tb/tb_cse_trace.sv
	@mkdir -p $(BUILD)/tb_cse_trace_1
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_cse_trace_1 -GNC=1 -GID=1 -GRD=1 \
	    --top-module tb_cse_trace $(PKG) $(CSE_RTL) tb/ple_cse_if_checker.sv tb/tb_cse_trace.sv -o Vtb_cse_trace >/dev/null

$(BUILD)/tb_cse_trace_3/Vtb_cse_trace: $(PKG) $(CSE_RTL) tb/ple_cse_if_checker.sv tb/tb_cse_trace.sv
	@mkdir -p $(BUILD)/tb_cse_trace_3
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_cse_trace_3 -GNC=3 -GID=3 -GRD=3 \
	    --top-module tb_cse_trace $(PKG) $(CSE_RTL) tb/ple_cse_if_checker.sv tb/tb_cse_trace.sv -o Vtb_cse_trace >/dev/null

$(BUILD)/tb_cse_trace_2/Vtb_cse_trace: $(PKG) $(CSE_RTL) tb/ple_cse_if_checker.sv tb/tb_cse_trace.sv
	@mkdir -p $(BUILD)/tb_cse_trace_2
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_cse_trace_2 -GNC=2 -GID=4 -GRD=2 \
	    --top-module tb_cse_trace $(PKG) $(CSE_RTL) tb/ple_cse_if_checker.sv tb/tb_cse_trace.sv -o Vtb_cse_trace >/dev/null

$(BUILD)/tb_cse_trace_8/Vtb_cse_trace: $(PKG) $(CSE_RTL) tb/ple_cse_if_checker.sv tb/tb_cse_trace.sv
	@mkdir -p $(BUILD)/tb_cse_trace_8
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_cse_trace_8 -GNC=8 -GID=16 -GRD=16 \
	    --top-module tb_cse_trace $(PKG) $(CSE_RTL) tb/ple_cse_if_checker.sv tb/tb_cse_trace.sv -o Vtb_cse_trace >/dev/null

$(BUILD)/tb_cse_kill/Vtb_cse_kill: $(PKG) $(CSE_RTL) tb/ple_cse_if_checker.sv tb/tb_cse_kill.sv
	@mkdir -p $(BUILD)/tb_cse_kill
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_cse_kill --top-module tb_cse_kill \
	    $(PKG) $(CSE_RTL) tb/ple_cse_if_checker.sv tb/tb_cse_kill.sv -o Vtb_cse_kill >/dev/null

$(BUILD)/tb_cse_kill0/Vtb_cse_kill: $(PKG) $(CSE_RTL) tb/ple_cse_if_checker.sv tb/tb_cse_kill.sv
	@mkdir -p $(BUILD)/tb_cse_kill0
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_cse_kill0 -GKE=0 --top-module tb_cse_kill \
	    $(PKG) $(CSE_RTL) tb/ple_cse_if_checker.sv tb/tb_cse_kill.sv -o Vtb_cse_kill >/dev/null

$(BUILD)/tb_cse_control/Vtb_cse_control: $(PKG) $(CSE_RTL) tb/ple_cse_if_checker.sv tb/tb_cse_control.sv
	@mkdir -p $(BUILD)/tb_cse_control
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_cse_control --top-module tb_cse_control \
	    $(PKG) $(CSE_RTL) tb/ple_cse_if_checker.sv tb/tb_cse_control.sv -o Vtb_cse_control >/dev/null

$(BUILD)/tb_cse_worst/Vtb_cse_worst: $(PKG) $(CSE_PKG) rtl/cse/cse_fifo.sv rtl/cse/cse_cu.sv tb/tb_cse_worst.sv
	@mkdir -p $(BUILD)/tb_cse_worst
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_cse_worst --top-module tb_cse_worst \
	    $(PKG) $(CSE_PKG) rtl/cse/cse_fifo.sv rtl/cse/cse_cu.sv tb/tb_cse_worst.sv -o Vtb_cse_worst >/dev/null

$(BUILD)/tb_cse_array_worst/Vtb_cse_array_worst: $(PKG) $(CSE_RTL) tb/ple_cse_if_checker.sv tb/tb_cse_array_worst.sv
	@mkdir -p $(BUILD)/tb_cse_array_worst
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_cse_array_worst --top-module tb_cse_array_worst \
	    $(PKG) $(CSE_RTL) tb/ple_cse_if_checker.sv tb/tb_cse_array_worst.sv -o Vtb_cse_array_worst >/dev/null

$(BUILD)/tb_cse_boundary/Vtb_cse_boundary: $(PKG) $(CSE_PKG) rtl/cse/cse_fifo.sv rtl/cse/cse_cu.sv tb/tb_cse_boundary.sv
	@mkdir -p $(BUILD)/tb_cse_boundary
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_cse_boundary --top-module tb_cse_boundary \
	    $(PKG) $(CSE_PKG) rtl/cse/cse_fifo.sv rtl/cse/cse_cu.sv tb/tb_cse_boundary.sv -o Vtb_cse_boundary >/dev/null

$(BUILD)/cse_mutants/cse_cu_lt.sv: rtl/cse/cse_cu.sv tb/cse_mutants.py
	@mkdir -p $(BUILD)/cse_mutants
	python3 -I tb/cse_mutants.py $< $@

$(BUILD)/cse_mutants/Vtb_cse_boundary: $(PKG) $(CSE_PKG) rtl/cse/cse_fifo.sv $(BUILD)/cse_mutants/cse_cu_lt.sv tb/tb_cse_boundary.sv
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/cse_mutants --top-module tb_cse_boundary \
	    $(PKG) $(CSE_PKG) rtl/cse/cse_fifo.sv $(BUILD)/cse_mutants/cse_cu_lt.sv \
	    tb/tb_cse_boundary.sv -o Vtb_cse_boundary >/dev/null

test-cse-mutant: $(BUILD)/cse_mutants/Vtb_cse_boundary
	@if $< > $(BUILD)/cse_mutants/run.log 2>&1; then \
	  echo "cse comparator mutant SURVIVED"; exit 1; \
	else echo "cse comparator mutant killed: $$(grep -m1 'd^2 == r^2' $(BUILD)/cse_mutants/run.log)"; fi

test-cse: $(BUILD)/tb_cse_trace/Vtb_cse_trace $(BUILD)/tb_cse_trace_1/Vtb_cse_trace \
          $(BUILD)/tb_cse_trace_2/Vtb_cse_trace $(BUILD)/tb_cse_trace_3/Vtb_cse_trace \
          $(BUILD)/tb_cse_trace_8/Vtb_cse_trace $(BUILD)/tb_cse_kill/Vtb_cse_kill \
          $(BUILD)/tb_cse_kill0/Vtb_cse_kill $(BUILD)/tb_cse_control/Vtb_cse_control \
          $(BUILD)/tb_cse_worst/Vtb_cse_worst \
          $(BUILD)/tb_cse_array_worst/Vtb_cse_array_worst \
          $(BUILD)/tb_cse_boundary/Vtb_cse_boundary test-cse-unit test-cse-mutant
	@for p in 1 2 3; do \
	  $< +env0=$(TRACE)/s0_p$$p +queries=$(TRACE)/s0_p$${p}_seed0_queries.txt \
	    > $(BUILD)/tb_cse_trace/p$$p.log || exit $$?; \
	  grep -E 'TB_CSE_TRACE|if_checker' $(BUILD)/tb_cse_trace/p$$p.log; done
	@$(BUILD)/tb_cse_trace_1/Vtb_cse_trace +env0=$(TRACE)/s0_p1 \
	  +queries=$(TRACE)/s0_p1_seed0_queries.txt > $(BUILD)/tb_cse_trace_1/p1.log
	@grep -E 'TB_CSE_TRACE|if_checker' $(BUILD)/tb_cse_trace_1/p1.log
	@$(BUILD)/tb_cse_trace_2/Vtb_cse_trace +env0=$(TRACE)/s0_p1 \
	  +queries=$(TRACE)/s0_p1_seed0_queries.txt > $(BUILD)/tb_cse_trace_2/p1.log
	@grep -E 'TB_CSE_TRACE|if_checker' $(BUILD)/tb_cse_trace_2/p1.log
	@$(BUILD)/tb_cse_trace_3/Vtb_cse_trace +env0=$(TRACE)/s0_p1 \
	  +queries=$(TRACE)/s0_p1_seed0_queries.txt > $(BUILD)/tb_cse_trace_3/p1.log
	@grep -E 'TB_CSE_TRACE|if_checker' $(BUILD)/tb_cse_trace_3/p1.log
	@$(BUILD)/tb_cse_trace_8/Vtb_cse_trace +env0=$(TRACE)/s0_p1 \
	  +queries=$(TRACE)/s0_p1_seed0_queries.txt > $(BUILD)/tb_cse_trace_8/p1.log
	@grep -E 'TB_CSE_TRACE|if_checker' $(BUILD)/tb_cse_trace_8/p1.log
	@$(BUILD)/tb_cse_kill/Vtb_cse_kill +env0=$(TRACE)/s0_p1 > $(BUILD)/tb_cse_kill/run.log || \
	  { grep -E 'TB_CSE_KILL|if_checker|ERROR' $(BUILD)/tb_cse_kill/run.log; exit 1; }
	@grep -E 'TB_CSE_KILL|if_checker' $(BUILD)/tb_cse_kill/run.log
	@$(BUILD)/tb_cse_kill0/Vtb_cse_kill +env0=$(TRACE)/s0_p1 > $(BUILD)/tb_cse_kill0/run.log || \
	  { grep -E 'TB_CSE_KILL|if_checker|ERROR' $(BUILD)/tb_cse_kill0/run.log; exit 1; }
	@grep -E 'TB_CSE_KILL|if_checker' $(BUILD)/tb_cse_kill0/run.log
	@$(BUILD)/tb_cse_control/Vtb_cse_control +env0=$(TRACE)/s0_p1 > $(BUILD)/tb_cse_control/run.log || \
	  { grep -E 'TB_CSE_CONTROL|if_checker|ERROR' $(BUILD)/tb_cse_control/run.log; exit 1; }
	@grep -E 'TB_CSE_CONTROL|if_checker' $(BUILD)/tb_cse_control/run.log
	@$(BUILD)/tb_cse_worst/Vtb_cse_worst | grep TB_CSE_WORST
	@$(BUILD)/tb_cse_array_worst/Vtb_cse_array_worst | grep TB_CSE_ARRAY_WORST
	@$(BUILD)/tb_cse_boundary/Vtb_cse_boundary | grep TB_CSE_BOUNDARY

SMALL_DEF := -DFX_NODE_CAP=32 -DFX_PATH_CAP=8
FXLIB_SMALL := $(BUILD)/libfxref_small.a
DPI_SMALL := -LDFLAGS $(FXLIB_SMALL) -LDFLAGS -lm
$(FXLIB_SMALL): $(FXLIB_SRC) $(wildcard $(FXDIR)/*.c) $(wildcard $(FXDIR)/*.h) $(wildcard $(SRCDIR)/*.h)
	@mkdir -p $(BUILD)/fxlib_small
	cd $(BUILD)/fxlib_small && for f in $(FXLIB_SRC); do gcc -O2 -std=gnu11 -ffp-contract=off -fPIC $(SMALL_DEF) -I$(FXDIR) -I$(SRCDIR) -c $$f || exit 1; done
	rm -f $@ && ar rcs $@ $(BUILD)/fxlib_small/*.o

$(FXLIB): $(FXLIB_SRC) $(wildcard $(FXDIR)/*.c) $(wildcard $(FXDIR)/*.h) $(wildcard $(SRCDIR)/*.h)
	@mkdir -p $(BUILD)/fxlib
	cd $(BUILD)/fxlib && for f in $(FXLIB_SRC); do gcc -O2 -std=gnu11 -ffp-contract=off -fPIC -I$(FXDIR) -I$(SRCDIR) -c $$f || exit 1; done
	rm -f $@ && ar rcs $@ $(BUILD)/fxlib/*.o

# ---- PLE unit tests against the golden model
$(BUILD)/tb_smp/Vtb_smp: $(PLE_PKG) rtl/ple/ple_smp.sv tb/tb_smp.sv $(FXLIB)
	@mkdir -p $(BUILD)/tb_smp
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_smp --top-module tb_smp $(PLE_PKG) rtl/ple/ple_smp.sv tb/tb_smp.sv $(DPI) -o Vtb_smp >/dev/null

$(BUILD)/tb_nnu/Vtb_nnu: $(PLE_PKG) rtl/ple/ple_tree.sv rtl/ple/ple_nnu.sv tb/tb_nnu.sv $(FXLIB)
	@mkdir -p $(BUILD)/tb_nnu
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_nnu --top-module tb_nnu $(PLE_PKG) rtl/ple/ple_tree.sv rtl/ple/ple_nnu.sv tb/tb_nnu.sv $(DPI) -o Vtb_nnu >/dev/null

$(BUILD)/tb_rkg/Vtb_rkg: $(PLE_PKG) rtl/ple/ple_rkg.sv tb/tb_rkg.sv $(FXLIB)
	@mkdir -p $(BUILD)/tb_rkg
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_rkg --top-module tb_rkg $(PLE_PKG) rtl/ple/ple_rkg.sv tb/tb_rkg.sv $(DPI) -o Vtb_rkg >/dev/null

$(BUILD)/tb_alu/Vtb_alu: $(PLE_PKG) rtl/ple/ple_isqrt.sv rtl/ple/ple_div7.sv tb/tb_alu.sv $(FXLIB)
	@mkdir -p $(BUILD)/tb_alu
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_alu --top-module tb_alu $(PLE_PKG) rtl/ple/ple_isqrt.sv rtl/ple/ple_div7.sv tb/tb_alu.sv $(DPI) -o Vtb_alu >/dev/null

# FKU is generated from the golden fixed-point FK (2 link units, 4 sphere lanes)
rtl/ple/ple_fku.sv: $(FKUGEN) $(FXDIR)/fx_fk_gen.c
	python3 -I $(FKUGEN) $(FXDIR)/fx_fk_gen.c $@ 2 4

$(BUILD)/tb_fku/Vtb_fku: $(PLE_PKG) rtl/ple/ple_fku.sv tb/tb_fku.sv $(FXLIB) $(FXDIR)/fx_fk_gen.c
	@mkdir -p $(BUILD)/tb_fku
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_fku --top-module tb_fku $(PLE_PKG) rtl/ple/ple_fku.sv tb/tb_fku.sv $(DPI) -o Vtb_fku >/dev/null

# SCU is generated from the golden pair tables and thresholds (8 lanes)
$(BUILD)/scu_dump.txt: $(SWDIR)/host/fx_dump_scu.c $(FXDIR)/fx_pose.c $(FXDIR)/fx_pose.h $(FXDIR)/fx_fk_gen.c
	@mkdir -p $(BUILD)
	gcc -O2 -std=gnu11 -I$(FXDIR) -o $(BUILD)/fx_dump_scu $(SWDIR)/host/fx_dump_scu.c $(FXDIR)/fx_pose.c $(FXDIR)/fx_fk_gen.c -lm
	$(BUILD)/fx_dump_scu > $@

rtl/ple/ple_scu.sv: $(SCUGEN) $(BUILD)/scu_dump.txt
	python3 -I $(SCUGEN) $(BUILD)/scu_dump.txt $@ 8

$(BUILD)/tb_scu/Vtb_scu: $(PLE_PKG) rtl/ple/ple_scu.sv tb/tb_scu.sv $(FXLIB) $(FXDIR)/fx_pose.c
	@mkdir -p $(BUILD)/tb_scu
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_scu --top-module tb_scu $(PLE_PKG) rtl/ple/ple_scu.sv tb/tb_scu.sv $(DPI) -o Vtb_scu >/dev/null

# robot constants (outer order, fine spheres, r_q) generated from the golden model
rtl/ple/ple_robot_pkg.sv: $(SWDIR)/host/fx_gen_robot_pkg.c $(FXDIR)/fx_pose.c $(FXDIR)/fx_pose.h $(FXDIR)/fx_fk_gen.c
	@mkdir -p $(BUILD)
	gcc -O2 -std=gnu11 -I$(FXDIR) -o $(BUILD)/fx_gen_robot_pkg $(SWDIR)/host/fx_gen_robot_pkg.c $(FXDIR)/fx_pose.c $(FXDIR)/fx_fk_gen.c -lm
	$(BUILD)/fx_gen_robot_pkg > $@

QD_SRC := $(PKG) $(PLE_PKG) rtl/ple/ple_robot_pkg.sv
QD_TB  := tb/ple_cse_if_checker.sv tb/tb_qd.sv
$(BUILD)/tb_qd/Vtb_qd: $(QD_SRC) rtl/ple/ple_qd.sv $(QD_TB) $(FXLIB)
	@mkdir -p $(BUILD)/tb_qd
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_qd --top-module tb_qd $(QD_SRC) rtl/ple/ple_qd.sv $(QD_TB) $(DPI) -o Vtb_qd >/dev/null
$(BUILD)/tb_qd_flat/Vtb_qd: $(QD_SRC) rtl/ple/ple_qd.sv $(QD_TB) $(FXLIB)
	@mkdir -p $(BUILD)/tb_qd_flat
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_qd_flat -GFLAT=1 --top-module tb_qd $(QD_SRC) rtl/ple/ple_qd.sv $(QD_TB) $(DPI) -o Vtb_qd >/dev/null
$(BUILD)/tb_qd_ak/Vtb_qd: $(QD_SRC) rtl/ple/ple_qd.sv $(QD_TB) $(FXLIB)
	@mkdir -p $(BUILD)/tb_qd_ak
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_qd_ak -GAK=1 --top-module tb_qd $(QD_SRC) rtl/ple/ple_qd.sv $(QD_TB) $(DPI) -o Vtb_qd >/dev/null

EVU_RTL := $(PLE_PKG) rtl/ple/ple_robot_pkg.sv rtl/ple/ple_rkg.sv rtl/ple/ple_fku.sv rtl/ple/ple_scu.sv rtl/ple/ple_qd.sv
EVU_TB  := tb/cse_bfm.sv tb/ple_cse_if_checker.sv tb/tb_evu.sv
$(BUILD)/tb_evu%/Vtb_evu: $(PKG) $(EVU_RTL) rtl/ple/ple_evu.sv $(EVU_TB) $(FXLIB)
	@mkdir -p $(dir $@)
	$(VERI) $(TBFLAGS) -Mdir $(dir $@) $(if $(findstring flat,$*),-GFLAT=1) $(if $(findstring ak,$*),-GAK=1) \
	    --top-module tb_evu $(PKG) $(EVU_RTL) rtl/ple/ple_evu.sv $(EVU_TB) $(DPI) -o Vtb_evu >/dev/null

CORE_RTL := $(EVU_RTL) rtl/ple/ple_smp.sv rtl/ple/ple_tree.sv rtl/ple/ple_nnu.sv rtl/ple/ple_isqrt.sv rtl/ple/ple_div7.sv \
            rtl/ple/ple_evu.sv rtl/ple/ple_core.sv
CORE_TB  := tb/cse_bfm.sv tb/ple_cse_if_checker.sv tb/tb_core.sv
$(BUILD)/tb_core%/Vtb_core: $(PKG) $(CORE_RTL) $(CORE_TB) $(FXLIB)
	@mkdir -p $(dir $@)
	$(VERI) $(TBFLAGS) -Mdir $(dir $@) $(if $(findstring flat,$*),-GFLAT=1) $(if $(findstring ak,$*),-GAK=1) $(if $(findstring big,$*),-GRANGE=67108864) \
	    --top-module tb_core $(PKG) $(CORE_RTL) $(CORE_TB) $(DPI) -o Vtb_core >/dev/null

# tree of 32 nodes, path of 8 (golden paths of exactly 8 and of 9-20 nodes occur): FAIL_CAPACITY (tree full, path too long) against a golden library built the same way
$(BUILD)/tb_core_small/Vtb_core: $(PKG) $(CORE_RTL) $(CORE_TB) $(FXLIB_SMALL)
	@mkdir -p $(dir $@)
	$(VERI) $(TBFLAGS) -Mdir $(dir $@) -GCAP=32 -GPCAP=8 --top-module tb_core $(PKG) $(CORE_RTL) $(CORE_TB) $(DPI_SMALL) -o Vtb_core >/dev/null

# whole planner against complete golden runs (14 problems x 2 seeds; flat / auto-kill / 4x range: 6 problems x 1 seed;
# small: 14 x 2 with 32 nodes / 8 path nodes); each also runs FAIL_ITER and ENV_ERR cases
test-ple-core: $(BUILD)/tb_core_h/Vtb_core $(BUILD)/tb_core_flat/Vtb_core $(BUILD)/tb_core_ak/Vtb_core $(BUILD)/tb_core_big/Vtb_core $(BUILD)/tb_core_small/Vtb_core
	$(BUILD)/tb_core_h/Vtb_core > $(BUILD)/tb_core_h/run.log; rc=$$?; grep -E 'TB_CORE|ERROR' $(BUILD)/tb_core_h/run.log; exit $$rc
	$(BUILD)/tb_core_flat/Vtb_core +probs=6 +seeds=1 > $(BUILD)/tb_core_flat/run.log; rc=$$?; grep -E 'TB_CORE|ERROR' $(BUILD)/tb_core_flat/run.log; exit $$rc
	$(BUILD)/tb_core_ak/Vtb_core +probs=6 +seeds=1 > $(BUILD)/tb_core_ak/run.log; rc=$$?; grep -E 'TB_CORE|ERROR' $(BUILD)/tb_core_ak/run.log; exit $$rc
	$(BUILD)/tb_core_big/Vtb_core +probs=6 +seeds=1 > $(BUILD)/tb_core_big/run.log; rc=$$?; grep -E 'TB_CORE|ERROR' $(BUILD)/tb_core_big/run.log; exit $$rc
	$(BUILD)/tb_core_small/Vtb_core > $(BUILD)/tb_core_small/run.log; rc=$$?; grep -E 'TB_CORE|ERROR' $(BUILD)/tb_core_small/run.log; exit $$rc

# PLE core + real CSE RTL (cse_top + cse_env_mem, environment loaded through the write port) vs golden runs.
# NCU=<n> CSE units (default 2). No auto-kill variant: the RTL CSE has no auto-kill.
NCU ?= 2
RTLCSE_TB := tb/cse_rtl.sv tb/ple_cse_if_checker.sv tb/tb_core.sv
$(BUILD)/tb_core_rtlcse$(NCU)/Vtb_core $(BUILD)/tb_core_rtlcse$(NCU)_flat/Vtb_core: $(PKG) $(CORE_RTL) $(CSE_RTL) $(RTLCSE_TB) $(FXLIB)
	@mkdir -p $(dir $@)
	$(VERI) $(TBFLAGS) -Mdir $(dir $@) +define+RTL_CSE +define+CSE_N_CU=$(NCU) $(if $(findstring flat,$@),-GFLAT=1) \
	    --top-module tb_core $(PKG) $(CORE_RTL) $(CSE_RTL) $(RTLCSE_TB) $(DPI) -o Vtb_core >/dev/null

test-ple-cse: $(BUILD)/tb_core_rtlcse$(NCU)/Vtb_core $(BUILD)/tb_core_rtlcse$(NCU)_flat/Vtb_core
	$(BUILD)/tb_core_rtlcse$(NCU)/Vtb_core $(ARGS) > $(BUILD)/tb_core_rtlcse$(NCU)/run.log; rc=$$?; grep -E 'TB_CORE|ERROR|if_checker' $(BUILD)/tb_core_rtlcse$(NCU)/run.log; exit $$rc
	$(BUILD)/tb_core_rtlcse$(NCU)_flat/Vtb_core +probs=6 +seeds=1 $(ARGS) > $(BUILD)/tb_core_rtlcse$(NCU)_flat/run.log; rc=$$?; grep -E 'TB_CORE|ERROR|if_checker' $(BUILD)/tb_core_rtlcse$(NCU)_flat/run.log; exit $$rc

TOP_TB := tb/cse_bfm.sv tb/ple_cse_if_checker.sv tb/tb_top.sv
$(BUILD)/tb_top/Vtb_top: $(PKG) $(CORE_RTL) rtl/ple/ple_top.sv $(TOP_TB) $(FXLIB)
	@mkdir -p $(dir $@)
	$(VERI) $(TBFLAGS) -Mdir $(dir $@) --top-module tb_top $(PKG) $(CORE_RTL) rtl/ple/ple_top.sv $(TOP_TB) $(DPI) -o Vtb_top >/dev/null

# PLE top through AXI-Lite: register map, interrupt, abort, results and path vs golden
test-ple-top: $(BUILD)/tb_top/Vtb_top
	$(BUILD)/tb_top/Vtb_top > $(BUILD)/tb_top/run.log; rc=$$?; grep -E 'TB_TOP|ERROR' $(BUILD)/tb_top/run.log; exit $$rc

# CPU driver (sw/ple_drv.c) run against ple_top through AXI-Lite (tb/drv_harness.c: coroutine test program)
DRV_CFLAGS := -CFLAGS -DPLE_DRV_HOSTSIM -CFLAGS -I$(abspath sw) -CFLAGS -I$(SRCDIR)
$(BUILD)/tb_drv/Vtb_drv: $(PKG) $(CORE_RTL) rtl/ple/ple_top.sv tb/cse_bfm.sv tb/ple_cse_if_checker.sv tb/tb_drv.sv tb/drv_harness.c sw/ple_drv.c sw/ple_drv.h $(FXLIB)
	@mkdir -p $(dir $@)
	$(VERI) $(TBFLAGS) -Mdir $(dir $@) --top-module tb_drv $(PKG) $(CORE_RTL) rtl/ple/ple_top.sv tb/cse_bfm.sv tb/ple_cse_if_checker.sv \
	    tb/tb_drv.sv $(abspath tb/drv_harness.c) $(abspath sw/ple_drv.c) $(DRV_CFLAGS) $(DPI) -o Vtb_drv >/dev/null

test-ple-drv: $(BUILD)/tb_drv/Vtb_drv
	$(BUILD)/tb_drv/Vtb_drv > $(BUILD)/tb_drv/run.log; rc=$$?; grep -E 'TB_DRV|DRV|ERROR' $(BUILD)/tb_drv/run.log; exit $$rc

test-ple-unit: $(BUILD)/tb_evu_h/Vtb_evu $(BUILD)/tb_evu_flat/Vtb_evu $(BUILD)/tb_evu_ak/Vtb_evu $(BUILD)/tb_qd/Vtb_qd $(BUILD)/tb_qd_flat/Vtb_qd $(BUILD)/tb_qd_ak/Vtb_qd $(BUILD)/tb_scu/Vtb_scu $(BUILD)/tb_smp/Vtb_smp $(BUILD)/tb_nnu/Vtb_nnu $(BUILD)/tb_rkg/Vtb_rkg $(BUILD)/tb_alu/Vtb_alu $(BUILD)/tb_fku/Vtb_fku
	$(BUILD)/tb_smp/Vtb_smp +n=50000 > $(BUILD)/tb_smp/run.log; rc=$$?; grep -E 'TB_SMP|MISMATCH' $(BUILD)/tb_smp/run.log; exit $$rc
	$(BUILD)/tb_nnu/Vtb_nnu > $(BUILD)/tb_nnu/run.log; rc=$$?; grep -E 'TB_NNU|ERROR' $(BUILD)/tb_nnu/run.log; exit $$rc
	@for cap in 512 4096; do \
	  $(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_nnu_cap$$cap -GCAP=$$cap --top-module tb_nnu $(PLE_PKG) rtl/ple/ple_tree.sv rtl/ple/ple_nnu.sv tb/tb_nnu.sv $(DPI) -o V >/dev/null || exit 1; \
	  $(BUILD)/tb_nnu_cap$$cap/V > $(BUILD)/tb_nnu_cap$$cap/run.log; rc=$$?; grep -E 'TB_NNU|ERROR' $(BUILD)/tb_nnu_cap$$cap/run.log; [ $$rc = 0 ] || exit $$rc; done
	$(BUILD)/tb_rkg/Vtb_rkg > $(BUILD)/tb_rkg/run.log; rc=$$?; grep -E 'TB_RKG|ERROR' $(BUILD)/tb_rkg/run.log; exit $$rc
	$(BUILD)/tb_alu/Vtb_alu > $(BUILD)/tb_alu/run.log; rc=$$?; grep -E 'TB_ALU|ERROR' $(BUILD)/tb_alu/run.log; exit $$rc
	$(BUILD)/tb_fku/Vtb_fku > $(BUILD)/tb_fku/run.log; rc=$$?; grep -E 'TB_FKU|ERROR' $(BUILD)/tb_fku/run.log; exit $$rc
	$(BUILD)/tb_scu/Vtb_scu > $(BUILD)/tb_scu/run.log; rc=$$?; grep -E 'TB_SCU|ERROR' $(BUILD)/tb_scu/run.log; exit $$rc
	@for v in tb_qd tb_qd_flat tb_qd_ak; do $(BUILD)/$$v/Vtb_qd > $(BUILD)/$$v/run.log; rc=$$?; grep -E 'TB_QD|STUCK|ERROR' $(BUILD)/$$v/run.log; [ $$rc = 0 ] || exit $$rc; done
	@for v in tb_evu_h tb_evu_flat tb_evu_ak; do $(BUILD)/$$v/Vtb_evu > $(BUILD)/$$v/run.log; rc=$$?; grep -E 'TB_EVU|STUCK|ERROR' $(BUILD)/$$v/run.log; [ $$rc = 0 ] || exit $$rc; done
	@if $(BUILD)/tb_smp/Vtb_smp +n=2000 +refoff=1 > $(BUILD)/tb_smp/control.log 2>&1; then echo "control (ref seed+1) PASSED: comparison is broken"; exit 1; else echo "control (ref seed+1): failed as expected"; fi
	@if $(BUILD)/tb_nnu/Vtb_nnu +rad_conflict > $(BUILD)/tb_nnu/control.log 2>&1; then echo "control (RAD conflict) PASSED: assertion is broken"; exit 1; else echo "control (RAD conflict): $$(grep -m1 -o 'ple_tree: [^)]*)' $(BUILD)/tb_nnu/control.log)"; fi

# planted RTL bugs (tb/mutants.py): each must make its unit test fail
MUT := $(BUILD)/mutants
test-ple-mutants:
	@mkdir -p $(MUT)
	python3 -I tb/mutants.py rtl/ple $(MUT)
	$(MAKE) $(FXLIB)
	@$(VERI) $(TBFLAGS) -Mdir $(MUT)/smp_m1 --top-module tb_smp $(PLE_PKG) $(MUT)/smp_m1.sv tb/tb_smp.sv $(DPI) -o V >/dev/null
	@if $(MUT)/smp_m1/V +n=20000 > $(MUT)/smp_m1.log 2>&1; then echo "smp_m1 SURVIVED"; exit 1; else echo "smp_m1 killed: $$(grep TB_SMP $(MUT)/smp_m1.log)"; fi
	@$(VERI) $(TBFLAGS) -Mdir $(MUT)/smp_m2 --top-module tb_smp $(PLE_PKG) $(MUT)/smp_m2.sv tb/tb_smp.sv $(DPI) -o V >/dev/null
	@if $(MUT)/smp_m2/V +n=2000 > $(MUT)/smp_m2.log 2>&1; then echo "smp_m2 SURVIVED"; exit 1; else echo "smp_m2 killed: $$(grep -m1 DELTA $(MUT)/smp_m2.log)"; fi
	@for m in nnu_m1 nnu_m2 nnu_m3 nnu_m4; do \
	  $(VERI) $(TBFLAGS) -Mdir $(MUT)/$$m --top-module tb_nnu $(PLE_PKG) rtl/ple/ple_tree.sv $(MUT)/$$m.sv tb/tb_nnu.sv $(DPI) -o V >/dev/null || exit 1; \
	  if $(MUT)/$$m/V > $(MUT)/$$m.log 2>&1; then echo "$$m SURVIVED"; exit 1; else echo "$$m killed: $$(grep -m1 -E 'TB_NNU|watchdog' $(MUT)/$$m.log)"; fi; done
	@for m in rkg_m1 rkg_m2 rkg_m3 rkg_m4 rkg_m5; do \
	  $(VERI) $(TBFLAGS) -Mdir $(MUT)/$$m --top-module tb_rkg $(PLE_PKG) $(MUT)/$$m.sv tb/tb_rkg.sv $(DPI) -o V >/dev/null || exit 1; \
	  if $(MUT)/$$m/V > $(MUT)/$$m.log 2>&1; then echo "$$m SURVIVED"; exit 1; else echo "$$m killed: $$(grep -m1 TB_RKG $(MUT)/$$m.log)"; fi; done
	@for m in alu_m1 alu_m2 alu_m3; do \
	  f=$$( [ $$m = alu_m1 ] && echo ple_isqrt || echo ple_div7 ); \
	  other=$$( [ $$m = alu_m1 ] && echo rtl/ple/ple_div7.sv || echo rtl/ple/ple_isqrt.sv ); \
	  $(VERI) $(TBFLAGS) -Mdir $(MUT)/$$m --top-module tb_alu $(PLE_PKG) $(MUT)/$$m.sv $$other tb/tb_alu.sv $(DPI) -o V >/dev/null || exit 1; \
	  if $(MUT)/$$m/V > $(MUT)/$$m.log 2>&1; then echo "$$m SURVIVED"; exit 1; else echo "$$m killed: $$(grep -m1 TB_ALU $(MUT)/$$m.log)"; fi; done
	python3 -I $(FKUGEN) $(FXDIR)/fx_fk_gen.c $(MUT)/fku_m4.sv 2 4 2 > /dev/null
	python3 -I $(FKUGEN) $(FXDIR)/fx_fk_gen.c $(MUT)/fku_m5.sv 2 4 3 2 > /dev/null
	python3 -I $(FKUGEN) $(FXDIR)/fx_fk_gen.c $(MUT)/fku_m10.sv 2 4 3 3 19 > /dev/null
	@for m in fku_m1 fku_m2 fku_m3 fku_m4 fku_m5 fku_m6 fku_m7 fku_m8 fku_m9 fku_m10; do \
	  $(VERI) $(TBFLAGS) -Mdir $(MUT)/$$m --top-module tb_fku $(PLE_PKG) $(MUT)/$$m.sv tb/tb_fku.sv $(DPI) -o V >/dev/null || exit 1; \
	  if $(MUT)/$$m/V > $(MUT)/$$m.log 2>&1; then echo "$$m SURVIVED"; exit 1; else echo "$$m killed: $$(grep -m1 -E 'TB_FKU|overflows' $(MUT)/$$m.log)"; fi; done
	@for m in scu_m1 scu_m2 scu_m3 scu_m4 scu_m5 scu_m6 scu_m7; do \
	  $(VERI) $(TBFLAGS) -Mdir $(MUT)/$$m --top-module tb_scu $(PLE_PKG) $(MUT)/$$m.sv tb/tb_scu.sv $(DPI) -o V >/dev/null || exit 1; \
	  if $(MUT)/$$m/V > $(MUT)/$$m.log 2>&1; then echo "$$m SURVIVED"; exit 1; else echo "$$m killed: $$(grep -m1 TB_SCU $(MUT)/$$m.log)"; fi; done
	@for m in qd_m1 qd_m2 qd_m3 qd_m4 qd_m5 qd_m6 qd_m7 qd_m8 qd_m9 qd_m10 qd_m11 qd_m13 qd_m14; do \
	  $(VERI) $(TBFLAGS) -Mdir $(MUT)/$$m --top-module tb_qd $(QD_SRC) $(MUT)/$$m.sv $(QD_TB) $(DPI) -o V >/dev/null || exit 1; \
	  if timeout 600 $(MUT)/$$m/V > $(MUT)/$$m.log 2>&1; then echo "$$m SURVIVED"; exit 1; else echo "$$m killed: $$(grep -m1 -E 'TB_QD|STUCK|ERROR' $(MUT)/$$m.log)"; fi; done
	@for m in qd_m12; do \
	  $(VERI) $(TBFLAGS) -Mdir $(MUT)/$$m -GAK=1 --top-module tb_qd $(QD_SRC) $(MUT)/$$m.sv $(QD_TB) $(DPI) -o V >/dev/null || exit 1; \
	  if timeout 600 $(MUT)/$$m/V > $(MUT)/$$m.log 2>&1; then echo "$$m SURVIVED"; exit 1; else echo "$$m killed: $$(grep -m1 -E 'TB_QD|STUCK|ERROR' $(MUT)/$$m.log)"; fi; done
	@for m in evu_m1 evu_m2 evu_m3 evu_m4; do \
	  $(VERI) $(TBFLAGS) -Mdir $(MUT)/$$m --top-module tb_evu $(PKG) $(EVU_RTL) $(MUT)/$$m.sv $(EVU_TB) $(DPI) -o V >/dev/null || exit 1; \
	  if timeout 900 $(MUT)/$$m/V > $(MUT)/$$m.log 2>&1; then echo "$$m SURVIVED"; exit 1; else echo "$$m killed: $$(grep -m1 -E 'TB_EVU|STUCK|ERROR' $(MUT)/$$m.log)"; fi; done
	$(MAKE) $(FXLIB_SMALL)
	@for m in core_m1 core_m2 core_m3 core_m4 core_m5 core_m6 core_m7 core_m8 core_m9 core_m11 core_m12 core_m13; do \
	  $(VERI) $(TBFLAGS) -Mdir $(MUT)/$$m --top-module tb_core $(PKG) $(filter-out rtl/ple/ple_core.sv,$(CORE_RTL)) $(MUT)/$$m.sv $(CORE_TB) $(DPI) -o V >/dev/null || exit 1; \
	  $(VERI) $(TBFLAGS) -Mdir $(MUT)/$${m}_big -GRANGE=67108864 --top-module tb_core $(PKG) $(filter-out rtl/ple/ple_core.sv,$(CORE_RTL)) $(MUT)/$$m.sv $(CORE_TB) $(DPI) -o V >/dev/null || exit 1; \
	  $(VERI) $(TBFLAGS) -Mdir $(MUT)/$${m}_small -GCAP=32 -GPCAP=8 --top-module tb_core $(PKG) $(filter-out rtl/ple/ple_core.sv,$(CORE_RTL)) $(MUT)/$$m.sv $(CORE_TB) $(DPI_SMALL) -o V >/dev/null || exit 1; \
	  if timeout 900 $(MUT)/$${m}_big/V +probs=6 +seeds=1 > $(MUT)/$$m.log 2>&1 && timeout 900 $(MUT)/$${m}_small/V >> $(MUT)/$$m.log 2>&1 && \
	     timeout 900 $(MUT)/$$m/V >> $(MUT)/$$m.log 2>&1; then echo "$$m SURVIVED"; exit 1; \
	  else echo "$$m killed: $$(grep -m1 -E 'TB_CORE|ERROR|stuck' $(MUT)/$$m.log)"; fi; done
	@for m in top_m1 top_m2 top_m3 top_m4 top_m5 top_m6 top_m7 top_m8 top_m9 top_m10; do \
	  $(VERI) $(TBFLAGS) -Mdir $(MUT)/$$m --top-module tb_top $(PKG) $(CORE_RTL) $(MUT)/$$m.sv $(TOP_TB) $(DPI) -o V >/dev/null || exit 1; \
	  if timeout 900 $(MUT)/$$m/V +probs=2 > $(MUT)/$$m.log 2>&1; then echo "$$m SURVIVED"; exit 1; else echo "$$m killed: $$(grep -m1 -E 'TB_TOP|ERROR' $(MUT)/$$m.log)"; fi; done
	@for m in corev_m1; do \
	  $(VERI) $(TBFLAGS) -Mdir $(MUT)/$$m --top-module tb_core $(PKG) $(filter-out rtl/ple/ple_evu.sv,$(CORE_RTL)) $(MUT)/$$m.sv $(CORE_TB) $(DPI) -o V >/dev/null || exit 1; \
	  if timeout 900 $(MUT)/$$m/V +probs=1 +seeds=1 > $(MUT)/$$m.log 2>&1; then echo "$$m SURVIVED"; exit 1; else echo "$$m killed: $$(grep -m1 -E 'TB_CORE|ERROR' $(MUT)/$$m.log)"; fi; done
	@for m in core_m14 core_m15; do \
	  $(VERI) $(TBFLAGS) -Mdir $(MUT)/$$m --top-module tb_top $(PKG) $(filter-out rtl/ple/ple_core.sv,$(CORE_RTL)) $(MUT)/$$m.sv rtl/ple/ple_top.sv $(TOP_TB) $(DPI) -o V >/dev/null || exit 1; \
	  if timeout 900 $(MUT)/$$m/V +probs=2 > $(MUT)/$$m.log 2>&1; then echo "$$m SURVIVED"; exit 1; else echo "$$m killed: $$(grep -m1 -E 'TB_TOP|ERROR' $(MUT)/$$m.log)"; fi; done
	@for m in topc_m1; do \
	  $(VERI) $(TBFLAGS) -Mdir $(MUT)/$$m --top-module tb_top $(PKG) $(filter-out rtl/ple/ple_core.sv,$(CORE_RTL)) $(MUT)/$$m.sv rtl/ple/ple_top.sv $(TOP_TB) $(DPI) -o V >/dev/null || exit 1; \
	  if timeout 900 $(MUT)/$$m/V +probs=2 > $(MUT)/$$m.log 2>&1; then echo "$$m SURVIVED"; exit 1; else echo "$$m killed: $$(grep -m1 -E 'TB_TOP|ERROR' $(MUT)/$$m.log)"; fi; done

clean:
	rm -rf $(BUILD)
