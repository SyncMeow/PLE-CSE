# Platform/accel -- PLE / CSE RTL (records/20261107-ple-cse-design.md section 13)
BUILD   ?= $(abspath ../build/accel)
VERI    ?= verilator
PKG     := rtl/common/ple_cse_pkg.sv
VFLAGS  := -Wall --assert -Irtl/common
PLE_PKG := rtl/ple/ple_pkg.sv
FXDIR   := $(abspath ../sw_rrtc/fx)
FKUGEN  := $(abspath ../sw_rrtc/tools/gen_fx_fku_rtl.py)
# testbenches call the golden model through DPI-C (fx_ref.c includes fx_plan.c)
DPI     := $(FXDIR)/fx_ref.c -CFLAGS -I$(FXDIR) -LDFLAGS -lm
TBFLAGS := --binary --timing --assert -Wno-fatal -Irtl/common

.PHONY: lint test-checker test-env-mem test-ple-unit test-ple-mutants clean

# every RTL file must lint clean
lint:
	$(VERI) --lint-only $(VFLAGS) $(PKG) rtl/cse/cse_top.sv --top-module cse_top
	$(VERI) --lint-only $(VFLAGS) $(PKG) rtl/ple/ple_top.sv --top-module ple_top
	$(VERI) --lint-only $(VFLAGS) $(PKG) rtl/cse/cse_env_mem.sv --top-module cse_env_mem
	@for m in ple_smp ple_tree ple_nnu ple_rkg ple_isqrt ple_div7 ple_fku; do $(VERI) --lint-only $(VFLAGS) $(PLE_PKG) rtl/ple/$$m.sv --top-module $$m || exit 1; done
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

test-env-mem: $(BUILD)/tb_env_mem/Vtb_env_mem
	$< +env0=$(TRACE)/s0_p1 +env1=$(TRACE)/s0_p2 +out=$(BUILD)/tb_env_mem | grep -E 'TB|cse_env_mem'
	@for f in dir cell ptx pty ptz; do \
	  cmp -s $(BUILD)/tb_env_mem/buf0_$$f.hex $(TRACE)/s0_p1_$$f.hex && echo "buf0 $$f OK" || echo "buf0 $$f DIFF"; \
	  cmp -s $(BUILD)/tb_env_mem/buf1_$$f.hex $(TRACE)/s0_p2_$$f.hex && echo "buf1 $$f OK" || echo "buf1 $$f DIFF"; done

# ---- PLE unit tests against the golden model
$(BUILD)/tb_smp/Vtb_smp: $(PLE_PKG) rtl/ple/ple_smp.sv tb/tb_smp.sv $(FXDIR)/fx_ref.c $(FXDIR)/fx_plan.c
	@mkdir -p $(BUILD)/tb_smp
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_smp --top-module tb_smp $(PLE_PKG) rtl/ple/ple_smp.sv tb/tb_smp.sv $(DPI) -o Vtb_smp >/dev/null

$(BUILD)/tb_nnu/Vtb_nnu: $(PLE_PKG) rtl/ple/ple_tree.sv rtl/ple/ple_nnu.sv tb/tb_nnu.sv $(FXDIR)/fx_ref.c $(FXDIR)/fx_plan.c
	@mkdir -p $(BUILD)/tb_nnu
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_nnu --top-module tb_nnu $(PLE_PKG) rtl/ple/ple_tree.sv rtl/ple/ple_nnu.sv tb/tb_nnu.sv $(DPI) -o Vtb_nnu >/dev/null

$(BUILD)/tb_rkg/Vtb_rkg: $(PLE_PKG) rtl/ple/ple_rkg.sv tb/tb_rkg.sv $(FXDIR)/fx_ref.c $(FXDIR)/fx_plan.c
	@mkdir -p $(BUILD)/tb_rkg
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_rkg --top-module tb_rkg $(PLE_PKG) rtl/ple/ple_rkg.sv tb/tb_rkg.sv $(DPI) -o Vtb_rkg >/dev/null

$(BUILD)/tb_alu/Vtb_alu: $(PLE_PKG) rtl/ple/ple_isqrt.sv rtl/ple/ple_div7.sv tb/tb_alu.sv $(FXDIR)/fx_ref.c $(FXDIR)/fx_plan.c
	@mkdir -p $(BUILD)/tb_alu
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_alu --top-module tb_alu $(PLE_PKG) rtl/ple/ple_isqrt.sv rtl/ple/ple_div7.sv tb/tb_alu.sv $(DPI) -o Vtb_alu >/dev/null

# FKU is generated from the golden fixed-point FK (2 link units, 4 sphere lanes)
rtl/ple/ple_fku.sv: $(FKUGEN) $(FXDIR)/fx_fk_gen.c
	python3 -I $(FKUGEN) $(FXDIR)/fx_fk_gen.c $@ 2 4

$(BUILD)/tb_fku/Vtb_fku: $(PLE_PKG) rtl/ple/ple_fku.sv tb/tb_fku.sv $(FXDIR)/fx_ref.c $(FXDIR)/fx_plan.c $(FXDIR)/fx_fk_gen.c
	@mkdir -p $(BUILD)/tb_fku
	$(VERI) $(TBFLAGS) -Mdir $(BUILD)/tb_fku --top-module tb_fku $(PLE_PKG) rtl/ple/ple_fku.sv tb/tb_fku.sv $(DPI) -o Vtb_fku >/dev/null

test-ple-unit: $(BUILD)/tb_smp/Vtb_smp $(BUILD)/tb_nnu/Vtb_nnu $(BUILD)/tb_rkg/Vtb_rkg $(BUILD)/tb_alu/Vtb_alu $(BUILD)/tb_fku/Vtb_fku
	$(BUILD)/tb_smp/Vtb_smp +n=50000 > $(BUILD)/tb_smp/run.log; rc=$$?; grep -E 'TB_SMP|MISMATCH' $(BUILD)/tb_smp/run.log; exit $$rc
	$(BUILD)/tb_nnu/Vtb_nnu > $(BUILD)/tb_nnu/run.log; rc=$$?; grep -E 'TB_NNU|ERROR' $(BUILD)/tb_nnu/run.log; exit $$rc
	$(BUILD)/tb_rkg/Vtb_rkg > $(BUILD)/tb_rkg/run.log; rc=$$?; grep -E 'TB_RKG|ERROR' $(BUILD)/tb_rkg/run.log; exit $$rc
	$(BUILD)/tb_alu/Vtb_alu > $(BUILD)/tb_alu/run.log; rc=$$?; grep -E 'TB_ALU|ERROR' $(BUILD)/tb_alu/run.log; exit $$rc
	$(BUILD)/tb_fku/Vtb_fku > $(BUILD)/tb_fku/run.log; rc=$$?; grep -E 'TB_FKU|ERROR' $(BUILD)/tb_fku/run.log; exit $$rc
	@if $(BUILD)/tb_smp/Vtb_smp +n=2000 +refoff=1 > $(BUILD)/tb_smp/control.log 2>&1; then echo "control (ref seed+1) PASSED: comparison is broken"; exit 1; else echo "control (ref seed+1): failed as expected"; fi
	@if $(BUILD)/tb_nnu/Vtb_nnu +rad_conflict > $(BUILD)/tb_nnu/control.log 2>&1; then echo "control (RAD conflict) PASSED: assertion is broken"; exit 1; else echo "control (RAD conflict): $$(grep -m1 -o 'ple_tree: [^)]*)' $(BUILD)/tb_nnu/control.log)"; fi

# planted RTL bugs (tb/mutants.py): each must make its unit test fail
MUT := $(BUILD)/mutants
test-ple-mutants:
	@mkdir -p $(MUT)
	python3 -I tb/mutants.py rtl/ple $(MUT)
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

clean:
	rm -rf $(BUILD)
