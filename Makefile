# PLE/CSE accelerator: entry point (make help). accel/ and sw_rrtc/ keep their own Makefiles.
HOST  := $(abspath build/sw_rrtc_host)
TRACE := $(HOST)/cse_trace
NCU   ?= 2

.PHONY: help fx trace lint test test-ple test-cse clean

help: ## List the targets
	@grep -E '^[a-z-]+:.*## ' $(MAKEFILE_LIST) | sed 's/:.*## /\t/'

fx: ## Build the golden model and host tools (sw_rrtc fx)
	$(MAKE) -C sw_rrtc fx

trace: fx ## Environment images + query traces of problems 0..9 (input of the CSE tests)
	@mkdir -p $(TRACE)
	$(HOST)/rrtc_fx -p 0:9 -n 1 -T $(TRACE) > $(TRACE)/rrtc_fx.log

lint: ## Lint every RTL file
	$(MAKE) -C accel lint

test-ple: ## PLE unit + core tests, and the PLE with the real CSE (NCU=n, default 2)
	$(MAKE) -C accel test-ple-unit
	$(MAKE) -C accel test-ple-core
	$(MAKE) -C accel test-ple-cse NCU=$(NCU)

test-cse: trace ## CSE tests, interface checker, environment memory
	$(MAKE) -C accel test-checker
	$(MAKE) -C accel test-env-mem
	$(MAKE) -C accel test-cse

test: lint test-ple test-cse ## Everything above

clean: ## Remove build/
	rm -rf build
