# sw_rrtc/Makefile -- VCC collision checking + dynamic-domain balanced RRT-Connect on Panda
# (MotionBenchMaker), brute-force vs Morton nearest neighbour. Same C sources on the host
# (statistics, shadow mode, validation) and on the SoC (cycle counts).
SHELL := /bin/bash
ARCH_FLAGS := -march=rv32im_zicsr_zfinx -mabi=ilp32 -mcmodel=medany
include ../toolchain.mk
CC      = $(TARGET_PREFIX)-gcc
OBJCOPY = $(TARGET_PREFIX)-objcopy
HOSTCC ?= gcc
MKDIR   = mkdir -p

BUILD_DIR ?= $(abspath ../build/sw_rrtc)
HOST_DIR  ?= $(abspath ../build/sw_rrtc_host)

# -ffp-contract=off on both sides: a fused multiply-add rounds differently and the host/SoC runs
# would no longer be bit-identical (checked by `make cmp-soc`).
COMMON := -O2 -std=gnu11 -ffp-contract=off -fno-math-errno -Wall -Wextra -Wdouble-promotion -Isrc
CORE   := src/env.c src/cc.c src/nn.c src/rrtc.c src/bench.c gen/panda_fk.c gen/problems_panda.c

# ---- SoC knobs (compile-time; the image runs PROB_FIRST..PROB_LAST x SEEDS and stops) ----
PROB_FIRST ?= 0
PROB_LAST  ?= 699
SCENES     ?= 0123456
SEEDS      ?= 1
SEED0      ?= 0
NN         ?= brute
NN_M       ?= 1
NN_W       ?= 8
NN_B       ?= 32
NN_BITS    ?= 9
MAX_ITER   ?= 1000000
MAX_SAMPLES ?= 1000000
TIMING     ?= 1
SOC_DEFS := -DPROB_FIRST=$(PROB_FIRST) -DPROB_LAST=$(PROB_LAST) -DSCENES=\"$(SCENES)\" -DSEEDS=$(SEEDS) \
            -DSEED0=$(SEED0) -DNN_MODE=$(if $(filter morton,$(NN)),1,0) -DNN_M=$(NN_M) -DNN_W=$(NN_W) \
            -DNN_B=$(NN_B) -DNN_BITS=$(NN_BITS) -DMAX_ITER=$(MAX_ITER) -DMAX_SAMPLES=$(MAX_SAMPLES) \
            $(if $(filter 0,$(TIMING)),-DNO_TIMING)

CFLAGS  := $(ARCH_FLAGS) $(COMMON) -g -ffunction-sections -fdata-sections -fno-builtin-printf $(SOC_DEFS) $(CFLAGS_EXTRA)
LDFLAGS := $(ARCH_FLAGS) -T soc/linker.ld -nostartfiles -nostdlib -Wl,--gc-sections -Wl,--no-relax
LDLIBS  := -Wl,--start-group -lm -lc -lgcc -Wl,--end-group

SOC_SRCS := $(CORE) soc/main_soc.c soc/syscalls.c
SOC_OBJS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(SOC_SRCS)) $(BUILD_DIR)/crt0.o

all: $(BUILD_DIR)/main.bin

$(BUILD_DIR)/main.bin: $(BUILD_DIR)/main.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/main.elf: $(SOC_OBJS) soc/linker.ld
	$(CC) $(LDFLAGS) -o $@ $(SOC_OBJS) $(LDLIBS)
	@$(TARGET_PREFIX)-size $@

$(BUILD_DIR)/cflags.stamp: FORCE
	@$(MKDIR) $(BUILD_DIR)
	@echo '$(CFLAGS)' > $@.tmp
	@cmp -s $@.tmp $@ && rm $@.tmp || mv $@.tmp $@

$(BUILD_DIR)/%.o: %.c $(wildcard src/*.h) $(BUILD_DIR)/cflags.stamp
	@$(MKDIR) $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# copied newlib stubs: unused parameters by design
$(BUILD_DIR)/soc/syscalls.o: CFLAGS += -Wno-unused-parameter -Wno-builtin-declaration-mismatch

$(BUILD_DIR)/crt0.o: soc/crt0.s
	@$(MKDIR) $(BUILD_DIR)
	$(CC) $(ARCH_FLAGS) -c $< -o $@

# ---- host ----
host: $(HOST_DIR)/rrtc_host $(HOST_DIR)/rrtc_host_timed $(HOST_DIR)/libvcc.so $(HOST_DIR)/rrtc_host_es $(HOST_DIR)/rrtc_host_flat $(HOST_DIR)/nn_scale

# design-independent workload of a flat robot-sphere stream (host/flatstat.c)
$(HOST_DIR)/rrtc_host_flat: host/main_host.c host/flatstat.c host/flatstat.h $(CORE) $(wildcard src/*.h)
	@$(MKDIR) $(HOST_DIR)
	$(HOSTCC) $(COMMON) -DENV_STATS -DFLAT_STATS -Ihost -o $@ host/main_host.c host/flatstat.c $(CORE) -lm

# Morton vs brute-force NN on synthetic trees of growing size (host/nn_scale.c)
$(HOST_DIR)/nn_scale: host/nn_scale.c $(CORE) $(wildcard src/*.h)
	@$(MKDIR) $(HOST_DIR)
	$(HOSTCC) $(COMMON) -Ihost -o $@ host/nn_scale.c $(CORE) -lm

# experiments A/B (host/envstat.c): same planner, observes every environment query
$(HOST_DIR)/rrtc_host_es: host/main_host.c host/envstat.c host/envstat.h $(CORE) $(wildcard src/*.h)
	@$(MKDIR) $(HOST_DIR)
	$(HOSTCC) $(COMMON) -DENV_STATS -Ihost -o $@ host/main_host.c host/envstat.c $(CORE) -lm

# same planner with rdtsc timing of each category (NN query/insert, FK, env, ...); x86 host only
$(HOST_DIR)/rrtc_host_timed: host/main_host.c $(CORE) $(wildcard src/*.h)
	@mkdir -p $(@D)
	$(HOSTCC) $(COMMON) -DHOST_TIMING -o $@ host/main_host.c $(CORE) -lm

$(HOST_DIR)/rrtc_host: host/main_host.c $(CORE) $(wildcard src/*.h)
	@$(MKDIR) $(HOST_DIR)
	$(HOSTCC) $(COMMON) -o $@ host/main_host.c $(CORE) -lm

# fixed-point golden model of the PLE/CSE accelerator (fx/, records/20261107-ple-cse-design.md)
FX_SRCS := fx/fx_plan.c fx/fx_pose.c fx/fx_fk_gen.c fx/fx_env.c
$(HOST_DIR)/rrtc_fx: host/main_fx.c $(FX_SRCS) $(CORE) $(wildcard src/*.h) $(wildcard fx/*.h)
	@$(MKDIR) $(HOST_DIR)
	$(HOSTCC) $(COMMON) -Ifx -o $@ host/main_fx.c $(FX_SRCS) $(CORE) -lm

FX_CHK_SRCS := src/env.c src/cc.c src/bench.c src/rrtc.c src/nn.c gen/panda_fk.c gen/problems_panda.c
$(HOST_DIR)/fx_fkcheck: host/fx_fkcheck.c fx/fx_fk_gen.c $(FX_CHK_SRCS) $(wildcard src/*.h) $(wildcard fx/*.h)
	@$(MKDIR) $(HOST_DIR)
	$(HOSTCC) $(COMMON) -Ifx -o $@ host/fx_fkcheck.c fx/fx_fk_gen.c $(FX_CHK_SRCS) -lm

$(HOST_DIR)/fx_envcheck: host/fx_envcheck.c fx/fx_env.c $(FX_CHK_SRCS) $(wildcard src/*.h) $(wildcard fx/*.h)
	@$(MKDIR) $(HOST_DIR)
	$(HOSTCC) $(COMMON) -Ifx -o $@ host/fx_envcheck.c fx/fx_env.c $(FX_CHK_SRCS) -lm

# regenerate the fixed-point FK (F = fraction bits, TS = sin/cos table step 2^TS joint LSBs)
FX_F  ?= 18
FX_TS ?= 6
fx-regen:
	python3 -I tools/gen_fx_fk.py gen/panda_fk.c fx/fx_fk_gen.c $(FX_F) $(FX_TS)

# golden-model self checks: delta_FK over 10^7 poses, cse_ref vs definition
fx-check: $(HOST_DIR)/fx_fkcheck $(HOST_DIR)/fx_envcheck
	$(HOST_DIR)/fx_fkcheck 10000000
	$(HOST_DIR)/fx_envcheck 0:60 20000

fx: $(HOST_DIR)/rrtc_fx $(HOST_DIR)/fx_fkcheck $(HOST_DIR)/fx_envcheck
.PHONY: fx fx-regen fx-check

$(HOST_DIR)/libvcc.so: host/api.c $(CORE) $(wildcard src/*.h)
	@$(MKDIR) $(HOST_DIR)
	$(HOSTCC) $(COMMON) -fPIC -shared -o $@ host/api.c $(CORE) -lm

# ---- experiments A (voxel lookup, MVT vs Morton bitmap) and B (CHECK batch order), host only ----
EXP_DIR ?= $(HOST_DIR)/expAB
exp-ab: host
	@mkdir -p $(EXP_DIR)
	for sc in 0 1 2 3 4 5 6; do $(HOST_DIR)/rrtc_host_es -S $$sc -n 10 -R > $(EXP_DIR)/es_s$$sc.csv & done; wait
	@grep -h ESCHK $(EXP_DIR)/es_s?.csv | sort | uniq -c
	python3 tools/analyze_ab.py $(EXP_DIR)/es_s?.csv

# ---- regenerate the vendored sources from a VAMP checkout (needs a Python with vamp installed) ----
VAMP_DIR ?=
VAMP_PY  ?= python3
regen:
	@test -n "$(VAMP_DIR)" || { echo "set VAMP_DIR=<vamp checkout> (and VAMP_PY=<python with vamp>)"; exit 1; }
	python3 tools/gen_panda_fk.py $(VAMP_DIR)/src/impl/vamp/robots/panda.hh gen/panda_fk.c
	$(VAMP_PY) tools/gen_problems.py $(VAMP_DIR) gen/problems_panda.c

# ---- validation against VAMP + internal references (host) ----
validate: host
	$(VAMP_PY) -I host/validate.py $(HOST_DIR)/libvcc.so

# ---- board ----
BOARD_LOG ?= $(BUILD_DIR)/board.log
run: all
	@sudo chmod 666 /dev/ttyUSB1 2>/dev/null || true
	python3 run_board.py $(BUILD_DIR)/main.bin --log $(BOARD_LOG)
	@$(MAKE) --no-print-directory host
	python3 tools/cmp_runs.py --soc $(BOARD_LOG) --host-bin $(HOST_DIR)/rrtc_host

# ---- RTL simulation (model shared via ../hw/tb/sim.mk) ----
# Own model (64 MB of simulated DDR) in its own SIM_DIR, so the default 2 MB model is untouched.
SIM_DIR       := $(abspath ../build/sim_rtl_rrtc)
SIM_DDR_BYTES := 67108864
SIM_BUILD_DIR := $(abspath ../build/sw_rrtc_sim)
SIM_LOG       ?= $(SIM_BUILD_DIR)/sim.log
SIM_TIMEOUT   ?= 4000000000
SIM_PROBS     ?= 200:200
sim:
	@$(MAKE) --no-print-directory all BUILD_DIR=$(SIM_BUILD_DIR) PROB_FIRST=$(word 1,$(subst :, ,$(SIM_PROBS))) \
	    PROB_LAST=$(word 2,$(subst :, ,$(SIM_PROBS))) SEEDS=1 MAX_SAMPLES=262144 CFLAGS_EXTRA=-DARENA_END=0x64000000u
	@$(MAKE) --no-print-directory sim-model
	-$(RUN_SIM) --image $(SIM_BUILD_DIR)/main.bin --log $(SIM_LOG) --timeout $(SIM_TIMEOUT) --idle-timeout 0
	@$(MAKE) --no-print-directory host
	python3 tools/cmp_runs.py --soc $(SIM_LOG) --host-bin $(HOST_DIR)/rrtc_host

# ---- fixed-point planner on the CPU (PLE baseline): tb_core problems x FX_SEEDS, mcycle per fx_solve ----
# FX_PROFILE=1 adds per-pose / per-query cycle split; FX_PROBS=first:last indexes the 14-problem list
FX_SEEDS   ?= 2
FX_PROFILE ?= 0
FX_PROBS   ?= 0:13
FX_SOC_DIR := $(abspath ../build/sw_rrtc_fx_soc)/p$(subst :,_,$(FX_PROBS))_prof$(FX_PROFILE)
FX_SOC_SRCS := soc/main_fx_soc.c $(FX_SRCS) $(CORE) soc/syscalls.c
# FX_BLOB=1: one problem per image, its environment built on the host (host/fx_dump_env.c) and linked in,
# so the RTL simulation runs fx_solve only
FX_BLOB    ?= 0
FX_PLIST   := 1 40 100 160 210 260 320 380 410 470 530 600 650 690
FX_GPROB    = $(word $(shell echo $$(($(word 1,$(subst :, ,$(FX_PROBS))) + 1))),$(FX_PLIST))
ifeq ($(FX_BLOB),1)
FX_SOC_DIR := $(FX_SOC_DIR)_blob
FX_BLOB_FLAGS = -DFX_ENV_BLOB -DFX_ENV_FILE=\"$(FX_SOC_DIR)/env.bin\" soc/env_blob.S
endif
$(HOST_DIR)/fx_dump_env: host/fx_dump_env.c $(FX_SRCS) $(CORE) $(wildcard src/*.h) $(wildcard fx/*.h)
	@$(MKDIR) $(HOST_DIR)
	$(HOSTCC) $(COMMON) -Ifx -o $@ host/fx_dump_env.c $(FX_SRCS) $(CORE) -lm
fx-soc: $(if $(filter 1,$(FX_BLOB)),$(HOST_DIR)/fx_dump_env)
	@$(MKDIR) $(FX_SOC_DIR)
	$(if $(filter 1,$(FX_BLOB)),$(HOST_DIR)/fx_dump_env $(FX_GPROB) $(FX_SOC_DIR)/env.bin)
	$(CC) $(ARCH_FLAGS) $(COMMON) -Ifx $(FX_BLOB_FLAGS) -g -ffunction-sections -fdata-sections -fno-builtin-printf -DFX_SEEDS=$(FX_SEEDS) \
	    -DFX_PROFILE=$(FX_PROFILE) -DFX_PFIRST=$(word 1,$(subst :, ,$(FX_PROBS))) -DFX_PLAST=$(word 2,$(subst :, ,$(FX_PROBS))) \
	    -DMAX_SAMPLES=262144 -DPROB_FIRST=0 -DPROB_LAST=0 -DSCENES=\"0\" -DSEEDS=1 -DSEED0=0 -DNN_MODE=0 -DNN_M=1 -DNN_W=8 -DNN_B=32 \
	    -DNN_BITS=9 -DMAX_ITER=1000000 -Wno-unused-parameter -Wno-builtin-declaration-mismatch \
	    $(FX_SOC_SRCS) soc/crt0.s $(LDFLAGS) $(LDLIBS) -o $(FX_SOC_DIR)/main.elf
	$(OBJCOPY) -O binary $(FX_SOC_DIR)/main.elf $(FX_SOC_DIR)/main.bin
	@$(TARGET_PREFIX)-size $(FX_SOC_DIR)/main.elf
fx-sim: fx-soc
	@$(MAKE) --no-print-directory sim-model
	$(subst --sim-dir $(SIM_DIR),--sim-dir $(FX_SOC_DIR)/run,$(RUN_SIM)) --image $(FX_SOC_DIR)/main.bin --log $(FX_SOC_DIR)/sim.log \
	    --timeout 4000000000 --idle-timeout 0
.PHONY: fx-soc fx-sim

clean:
	rm -rf $(BUILD_DIR) $(HOST_DIR) $(SIM_BUILD_DIR)

FORCE:
.PHONY: all host exp-ab regen validate run sim clean FORCE

include ../hw/tb/sim.mk
