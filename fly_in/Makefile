# ─────────────────────────────────────────────────────────────────────────────
# Drone Routing — Makefile
#
# VARIABLES (override on the command line):
#   MAP    path to the map file           default: maps/easy1.txt
#   ALGO   dijkstra | bellman_ford | both  default: dijkstra
#   TURNS  max simulation turns (int)     default: auto
#
# EXAMPLES:
#   make run
#   make run    MAP=maps/hard1.txt
#   make run    MAP=maps/medium1.txt ALGO=bellman_ford
#   make run    MAP=maps/easy1.txt   TURNS=50
#   make metrics MAP=maps/hard1.txt
#   make compare MAP=maps/hard1.txt
#   make bf      MAP=maps/easy1.txt
#   make both    MAP=maps/medium1.txt
# ─────────────────────────────────────────────────────────────────────────────

PYTHON  := python3
MAIN    := main.py

# Defaults — override any of these on the command line
MAP   ?= 
ALGO  ?= dijkstra
TURNS ?=

# Build the optional --turns flag only when TURNS is set
TURNS_FLAG := $(if $(TURNS),--turns $(TURNS),)

# ─────────────────────────────────────────────────────────────────────────────
# MANDATORY TARGETS  (required by spec)
# ─────────────────────────────────────────────────────────────────────────────

.PHONY: install
install:                          ## Install project dependencies
	pip install -r requirements.txt --break-system-packages

.PHONY: run
run:                              ## Run simulation  [MAP=] [ALGO=] [TURNS=]
	$(PYTHON) $(MAIN) $(MAP) --algo $(ALGO) $(TURNS_FLAG)

.PHONY: debug
debug:                            ## Run with pdb debugger  [MAP=] [ALGO=]
	$(PYTHON) -m pdb $(MAIN) $(MAP) --algo $(ALGO) $(TURNS_FLAG)

.PHONY: clean
clean:                            ## Remove caches and temp files
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name .mypy_cache -exec rm -rf {} + 2>/dev/null || true
	find . -name "*.pyc" -delete 2>/dev/null || true

.PHONY: lint
lint:                             ## flake8 + mypy (spec mandatory flags)
	flake8 .
	mypy . --warn-return-any --warn-unused-ignores \
	       --ignore-missing-imports \
	       --disallow-untyped-defs --check-untyped-defs

.PHONY: lint-strict
lint-strict:                      ## flake8 + mypy --strict
	flake8 .
	mypy . --strict

# ─────────────────────────────────────────────────────────────────────────────
# CONVENIENCE TARGETS
# ─────────────────────────────────────────────────────────────────────────────

.PHONY: metrics
metrics:                          ## Run + print metrics panel  [MAP=] [ALGO=]
	$(PYTHON) $(MAIN) $(MAP) --algo $(ALGO) $(TURNS_FLAG) --metrics

.PHONY: compare
compare:                          ## Run both algos + comparison  [MAP=]
	$(PYTHON) $(MAIN) $(MAP) --algo both $(TURNS_FLAG) --compare

.PHONY: bf
bf:                               ## Run with Bellman-Ford  [MAP=] [TURNS=]
	$(PYTHON) $(MAIN) $(MAP) --algo bellman_ford $(TURNS_FLAG)

.PHONY: both
both:                             ## Run both algos (no comparison panel)  [MAP=]
	$(PYTHON) $(MAIN) $(MAP) --algo both $(TURNS_FLAG)

.PHONY: help
help:                             ## Show this help
	@echo ""
	@echo "  Drone Routing — available targets"
	@echo ""
	@grep -E '^[a-zA-Z_-]+:.*##' $(MAKEFILE_LIST) \
	    | awk 'BEGIN {FS = ":.*##"}; {printf "  \033[36m%-14s\033[0m %s\n", $$1, $$2}'
	@echo ""
	@echo "  Variables:  MAP=$(MAP)  ALGO=$(ALGO)  TURNS=$(TURNS)"
	@echo ""

# Default target
.DEFAULT_GOAL := help