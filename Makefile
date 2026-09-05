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

.PHONY: test-tts test-tts-fast test-tts-verbose lint-cc help

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
