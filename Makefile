# Rob_Box — top-level test shortcuts.
#
# These targets wrap the pytest invocations documented in each package's
# README so contributors don't have to memorise the per-package
# ``PYTHONPATH=.`` / ``cd src/...`` incantations.
#
# Targets:
#   make test-tts         — run the MiniMax TTS provider + conformance suite
#                           with the 85% coverage gate (mirrors CI).
#   make test-tts-fast    — same suite, no coverage gate (faster local loop).
#   make test-tts-verbose — same suite with ``-vv`` and stdout-captured logs.
#   make lint-cc          — run the ADR-0021 CC-budget guard locally (CI mirror).
#   make stt-tars-stats   — empirical STT-distortion summary for helm
#                           wake-word «ТАРС» (ADR-0114 §2.3).
#
# Сборка и анализ — ТЕМ ЖЕ кодом, что в CI (run 36351952819):
#   make build-list                  — что можно собрать (из docker/build-manifest.yaml)
#   make build-base [BASE=depthai]   — базовые образы (по умолчанию все, pcl после ros2-zenoh)
#   make build SERVICE=oak-d         — один сервис; PI=vision|main — весь Pi
#   make build-all                   — базы + оба Pi
#     BUILD_FLAGS='--dry-run|--no-push|--no-cache|--platform linux/amd64|--docker-tag dev'
#   make ci-list                     — какие workflow/job'ы гоняются локально
#   make lint                        — G-Lint Code целиком (те же run:-шаги из YAML)
#   make audit                       — G-Architecture Audit (статическая часть)
#   make ci WF='G-Run Tests' [JOB=…] — любой workflow; CI_FLAGS='--with-install --step …'

.PHONY: test-tts test-tts-fast test-tts-verbose lint-cc stt-tars-stats help \
        build-list build-base build build-all build-help ci-list ci lint audit

# Include the cross-provider conformance module explicitly: ``-k minimax``
# selects only the MiniMax parametrisations and silently drops the
# FakeTTSProvider half of the matrix.
TTS_TEST_FILTER := minimax or tts_conformance

# Common pytest flags — kept short so a typing dev can paste them.
TTS_COV_ARGS := --cov=rob_box_llm.providers.minimax_tts \
                --cov-report=term-missing \
                --cov-fail-under=85

help:
	@echo "Available targets:"
	@echo "  make test-tts           Run MiniMax TTS conformance + unit tests (85% coverage gate, mirrors CI)"
	@echo "  make test-tts-fast      Same suite, no coverage gate (faster local feedback loop)"
	@echo "  make test-tts-verbose   Same suite with -vv and captured stdout"
	@echo "  make lint-cc            Run ADR-0021 CC-budget guard (dialogue_node.py + new voice nodes)"
	@echo "  make stt-tars-stats     Empirical STT-distortion summary for helm wake-word «ТАРС» (ADR-0114)."
	@echo "                            Pass JSONL=<path> and/or YAML=<path>. Use --diff for candidates."
	@echo ""
	@echo "Build (same engine as CI: scripts/build/build.py → buildx_build.sh):"
	@echo "  make build-list | build-base [BASE=x] | build SERVICE=x | build PI=vision | build-all"
	@echo "  BUILD_FLAGS='--dry-run --no-push --no-cache --platform linux/amd64 --docker-tag dev'"
	@echo "CI checks locally (scripts/ci/run_workflow_job.py — runs the workflow's own run: steps):"
	@echo "  make ci-list | lint | audit | ci WF='G-Run Tests' [JOB=id] [CI_FLAGS='--with-install']"

# ADR-0021 R1 (issue #1984): CC<=15 for methods, CC<=20 for __init__.
# Baseline exemptions live in scripts/lint/cc_budget_baseline.json; run
# ``python scripts/lint/cc_budget.py --update-baseline`` after a refactor
# that shrinks a grandfathered method.
lint-cc:
	python scripts/lint/cc_budget.py

# Run from the package directory so the local pytest.ini (testpaths = test,
# asyncio_mode = auto, coverage config) is picked up. PYTHONPATH=. is the
# legacy way to make the in-tree rob_box_llm package importable; ``pip
# install -e .[dev]`` is the cleaner alternative if the dev has done that.
test-tts:
	cd src/rob_box_llm && PYTHONPATH=. python3 -m pytest -k '$(TTS_TEST_FILTER)' $(TTS_COV_ARGS)

test-tts-fast:
	cd src/rob_box_llm && PYTHONPATH=. python3 -m pytest -k '$(TTS_TEST_FILTER)'

test-tts-verbose:
	cd src/rob_box_llm && PYTHONPATH=. python3 -m pytest -k '$(TTS_TEST_FILTER)' $(TTS_COV_ARGS) -vv -s

# ADR-0114 §2.3 — сухая сводка STT-семплов «ТАРС» с шлема (без правок YAML).
# Требует:  ROBBOX_STT_COLLECT=1 при работе stt_node + наличие JSONL.
# Пример:   JSONL=data/stt_tars_samples.jsonl make stt-tars-stats
#           JSONL=data/stt_tars_samples.jsonl make stt-tars-stats ARGS='--diff --top 30'
JSONL ?= data/stt_tars_samples.jsonl
ARGS  ?=
stt-tars-stats:
	python3 scripts/stt/tars_stats.py $(JSONL) $(ARGS)

# ---- Сборка: тот же манифест и движок, что в L-Build workflow --------------
BUILD_PY    := python3 scripts/build/build.py
BUILD_FLAGS ?=
BASE        ?= all

build-help:
	$(BUILD_PY) --help

build-list:
	$(BUILD_PY) list

build-base:
	$(BUILD_PY) base $(BASE) $(BUILD_FLAGS)

build:
ifdef SERVICE
	$(BUILD_PY) service $(SERVICE) $(BUILD_FLAGS)
else ifdef PI
	$(BUILD_PY) pi $(PI) $(BUILD_FLAGS)
else
	@echo "usage: make build SERVICE=<name> | make build PI=vision|main (см. make build-list)"; exit 2
endif

build-all:
	$(BUILD_PY) all $(BUILD_FLAGS)

# ---- Анализ: run:-шаги прямо из .github/workflows (ни одной копии команд) ---
CI_PY    := python3 scripts/ci/run_workflow_job.py
CI_FLAGS ?=

ci-list:
	$(CI_PY) --list

ci:
	@test -n "$(WF)" || { echo "usage: make ci WF='<workflow>' [JOB=<id>] (см. make ci-list)"; exit 2; }
	$(CI_PY) "$(WF)" $(if $(JOB),-j $(JOB)) $(CI_FLAGS)

lint:
	$(CI_PY) "G-Lint Code" $(CI_FLAGS)

audit:
	$(CI_PY) "G-Architecture Audit" $(CI_FLAGS)
