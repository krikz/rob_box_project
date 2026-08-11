---
phase: 06-harness-p0-finalization
nyquist_version: 1.0
created: 2026-07-27
source: 06-RESEARCH.md §Validation Architecture
---

# Validation Strategy — Phase 6: Harness P0 Finalization

## Test Framework

| Property | Value |
|----------|-------|
| Framework | pytest 6.2.5 + pytest-asyncio >=0.21 + pytest-cov >=4.0 |
| Config file | `pytest.ini` (repo root) |
| Quick run command | `python3 -m pytest src/rob_box_harness/test -x -q` |
| Full suite command | `python3 -m pytest src/rob_box_harness/test tests/unit/harness --cov=rob_box_harness --cov-fail-under=85` |

## Phase Requirements → Test Map

| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| DOC-MERGE-01 | adr/0003 merged from architecture/ copy | static | `diff <(wc -l docs/adr/0003*) <(wc -l docs/architecture/minimax-tts-architecture.md)` | ❌ Wave 0 |
| DOC-MERGE-02 | adr/0004 merged from architecture/ copy | static | `diff <(wc -l docs/adr/0004*) <(wc -l docs/architecture/minimax-tts-integration-design.md)` | ❌ Wave 0 |
| DOC-MERGE-03 | 0007a/b/c fragments merged into 0007-final | static | `test -f docs/adr/0007-minimax-tts-integration-final.md && test ! -f docs/adr/0007a*.md` | ❌ Wave 0 |
| DOC-DEDUP-04 | architecture/ has cross-refs to adr/ only | static | `! grep -rL "см. adr/" docs/architecture/minimax-*.md` | ❌ Wave 0 |
| DOC-SPEC-05 | SPEC_CURRENT.md: P0→Done, no Hermes refs | static | `grep "P0.*Done\|P0.*✅" SPEC_CURRENT.md && ! grep -i "hermes\|kanban create" SPEC_CURRENT.md` | ❌ Wave 0 |
| DOCKER-06 | rob_box_harness in voice_assistant Dockerfile | static | `grep "rob_box_harness" docker/vision/voice_assistant/Dockerfile && ! grep -E "COPY.*(config|scripts)/" docker/vision/voice_assistant/Dockerfile` | ❌ Wave 0 |
| DOCKER-07 | Docker build succeeds | manual | `docker build -f docker/vision/voice_assistant/Dockerfile -t rob_box_voice:harness-test docker/` | ❌ Wave 0 |
| HARN-DIALOG-08 | DialogHarness wraps LLM+tools+memory | unit | `pytest src/rob_box_harness/test/test_dialog_harness.py -x` | ❌ Wave 0 |
| HARN-DSM-09 | DialogueStateMachine state transitions | unit | `pytest src/rob_box_harness/test/test_dsm.py -x` | ❌ Wave 0 |
| HARN-PERSIST-10 | PersistentHarness lifecycle unification | unit | `pytest src/rob_box_harness/test/test_persistent_harness.py -x` | ❌ Wave 0 |
| HARN-TG-11 | TelegramHarness command routing | unit | `pytest src/rob_box_harness/test/test_telegram_harness.py -x` | ❌ Wave 0 |
| PORT-ROS2-12 | ROS2Transport subscribe/publish | unit | `pytest src/rob_box_harness/test/test_ros2_transport.py -x` | ❌ Wave 0 |
| PORT-SQLITE-13 | SQLiteVoiceMemory CRUD | unit | `pytest src/rob_box_harness/test/test_sqlite_voice_memory.py -x` | ❌ Wave 0 |
| TEST-DIALOG-14 | dialogue_node 9%→80% | unit | `pytest src/rob_box_voice/test/ -x --cov=rob_box_voice.dialogue_node` | ❌ Wave 0 |
| TEST-TG-15 | telegram_node 0%→50% | unit | `pytest src/rob_box_telegram/test/ -x --cov=rob_box_telegram` | ❌ Wave 0 |
| TEST-MCP-16 | MCP tools →70% | unit | `pytest src/rob_box_mcp_tools/test/ -x --cov=rob_box_mcp_tools` | ❌ Wave 0 |
| TEST-E2E-17 | Integration E2E harness+real nodes | integration | `pytest tests/integration/ -x -m integration` | ❌ Wave 0 |
| MYPY-21 | mypy strict-clean | static | `mypy src/rob_box_harness/rob_box_harness/` | ✅ existing |
| LINT-22 | black/isort/flake8 | static | `black --check ... && isort --check ... && flake8 ...` | ✅ existing |

## Sampling Rate

- **Per task commit:** `python3 -m pytest src/rob_box_harness/test -x -q` (fast unit tests)
- **Per wave merge:** `python3 -m pytest src/rob_box_harness/test tests/unit/harness --cov=rob_box_harness` (full unit suite)
- **Phase gate:** Full suite green + coverage ≥85% + mypy strict-clean + linters pass

## Wave 0 Gaps (pre-execution checklist)

- [ ] `test/test_dialog_harness.py` — covers HARN-DIALOG-08
- [ ] `test/test_dsm.py` — covers HARN-DSM-09
- [ ] `test/test_persistent_harness.py` — covers HARN-PERSIST-10
- [ ] `test/test_telegram_harness.py` — covers HARN-TG-11
- [ ] `test/test_ros2_transport.py` — covers PORT-ROS2-12
- [ ] `test/test_sqlite_voice_memory.py` — covers PORT-SQLITE-13
- [ ] `src/rob_box_voice/test/` — covers TEST-DIALOG-14 (may need directory creation)
- [ ] `src/rob_box_telegram/test/` — covers TEST-TG-15 (may need directory creation)
- [ ] `src/rob_box_mcp_tools/test/` — covers TEST-MCP-16 (may need directory creation)
- [ ] `tests/integration/` — covers TEST-E2E-17 (may need directory creation)
- [ ] `pip install mypy flake8` — tools for W21-W22
- [ ] `conftest.py` shared fixtures — for new test files (FakeLLMProvider, RecordingBus, etc.)

## Feedback Latency

| Verification Type | Target Latency | Tool/Command |
|-------------------|---------------|--------------|
| Dockerfile structure check | <1s | `grep` |
| Unit tests (single file) | <5s | `pytest -x -q` |
| Unit tests (full suite) | <30s | `pytest --cov` |
| Docker build (full) | 5-15min | `docker build` — MANUAL only |
| Linters (per file) | <3s | `black --check`, `isort --check`, `flake8` |
| mypy (full package) | <30s | `mypy src/rob_box_harness/` |

---

*Validation strategy derived from 06-RESEARCH.md. Covers all 22 phase requirements with automated and manual verification commands.*
