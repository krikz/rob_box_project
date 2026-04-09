# FAQ Event Mode Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add a reusable event FAQ mode so ROBBOX can identify itself as an event assistant, answer event-specific questions from a provided FAQ file, and stay in normal chat mode for everything else.

**Architecture:** Reuse the existing SQLite + FTS5 + optional sqlite-vec/Ollama stack from voice memory for a dedicated FAQ knowledge store. Add an event-aware prompt path plus a focused FAQ skill so the compositor can delegate event questions to retrieval instead of relying on prompt-only recall.

**Tech Stack:** Python, ROS 2 node config, OpenAI Agents SDK, SQLite FTS5, sqlite-vec, Ollama embeddings, YAML, openpyxl, pytest.

---

### Task 1: Inspect FAQ Input And Lock The Schema

**Files:**
- Read: `/home/builder/Downloads/FAQ 2026 Общие вопросы + бак (1).xlsx`
- Modify: `docs/plans/2026-04-08-faq-event-mode.md`

**Step 1: Inspect workbook sheets and columns**

Run: `python - <<'PY'`
Expected: print workbook sheet names, header row per sheet, and first 3 non-empty rows.

**Step 2: Update this plan if the file shape differs from assumptions**

Expected: parser target columns are explicit before implementation starts.

**Step 3: Commit**

```bash
git add docs/plans/2026-04-08-faq-event-mode.md
git commit -m "docs(voice): lock faq event mode plan"
```

### Task 2: Add Failing Tests For FAQ Storage And Loading

**Files:**
- Create: `src/rob_box_voice/test/unit/core/test_faq_store.py`
- Create: `src/rob_box_voice/test/unit/core/test_faq_loader.py`
- Read: `src/rob_box_voice/test/unit/core/test_conversation_history.py`
- Read: `src/rob_box_voice/test/unit/core/test_dialogue_manager.py`

**Step 1: Write failing tests for FAQ store search behavior**

```python
def test_search_faq_returns_keyword_match(tmp_path):
    store = FAQStore(db_path=str(tmp_path / "faq.db"), migrations_dir=str(MIGRATIONS_DIR))
    store.replace_items(
        items=[
            {
                "question": "Какие направления бакалавриата доступны?",
                "answer": "Доступны AI и робототехника.",
                "category": "admission",
            }
        ]
    )

    results = store.search("направления бакалавриата", limit=3)

    assert len(results) == 1
    assert results[0]["category"] == "admission"
```

**Step 2: Write failing tests for event FAQ parsing**

```python
def test_load_xlsx_extracts_question_answer_rows(tmp_path):
    workbook_path = tmp_path / "faq.xlsx"
    create_sample_workbook(workbook_path)

    items = load_faq_items(workbook_path)

    assert items == [
        {
            "question": "Где проходит день открытых дверей?",
            "answer": "В главном корпусе.",
            "category": "general",
        }
    ]
```

**Step 3: Run tests to verify RED**

Run: `pytest src/rob_box_voice/test/unit/core/test_faq_store.py src/rob_box_voice/test/unit/core/test_faq_loader.py -v`
Expected: FAIL because FAQ store and loader do not exist yet.

**Step 4: Commit**

```bash
git add src/rob_box_voice/test/unit/core/test_faq_store.py src/rob_box_voice/test/unit/core/test_faq_loader.py
git commit -m "test(voice): add red tests for faq event mode"
```

### Task 3: Implement FAQ Data Store And Loader

**Files:**
- Create: `src/rob_box_voice/rob_box_voice/core/faq_store.py`
- Create: `src/rob_box_voice/scripts/load_faq_event.py`
- Modify: `migrations/005_faq.sql`
- Modify: `src/rob_box_voice/requirements.txt`
- Read: `src/rob_box_voice/rob_box_voice/core/voice_memory.py`

**Step 1: Implement minimal FAQ store API to satisfy tests**

```python
class FAQStore:
    def replace_items(self, items: list[dict]) -> int:
        ...

    def search(self, query: str, limit: int = 3) -> list[dict]:
        ...
```

Requirements:
- Separate FAQ tables from `voice_turns` / `voice_facts`
- Reuse FTS5 and optional vector search pattern from `VoiceMemory`
- Keep graceful degradation when sqlite-vec or Ollama is unavailable
- Support `question`, `answer`, `category`, `source`, `event_id`

**Step 2: Implement xlsx/csv/json loader**

Requirements:
- `load_faq_items(path)` accepts `.xlsx`, `.csv`, `.json`
- Normalizes rows into `{"question", "answer", "category"}`
- Skips blank rows
- Raises clear error if required columns are missing

**Step 3: Implement CLI entry flow**

Run target: `python src/rob_box_voice/scripts/load_faq_event.py --faq-file <path> --event-name <name>`

Requirements:
- Read FAQ file
- Replace indexed items for the selected event
- Print count of loaded rows

**Step 4: Run focused tests to verify GREEN**

Run: `pytest src/rob_box_voice/test/unit/core/test_faq_store.py src/rob_box_voice/test/unit/core/test_faq_loader.py -v`
Expected: PASS

**Step 5: Commit**

```bash
git add src/rob_box_voice/rob_box_voice/core/faq_store.py src/rob_box_voice/scripts/load_faq_event.py migrations/005_faq.sql src/rob_box_voice/requirements.txt
git commit -m "feat(voice): add faq knowledge store"
```

### Task 4: Add FAQ Skill And Event Mode Prompting

**Files:**
- Create: `src/rob_box_voice/rob_box_voice/skills/faq_skill.py`
- Modify: `src/rob_box_voice/rob_box_voice/skills/__init__.py`
- Modify: `src/rob_box_voice/rob_box_voice/dialogue_node.py`
- Create: `src/rob_box_voice/prompts/event_prompt.txt`
- Create: `src/rob_box_voice/prompts/skills/faq_skill_prompt.txt`
- Modify: `src/rob_box_voice/prompts/compositor_prompt.txt`
- Modify: `docker/vision/config/voice_assistant/voice_assistant.yaml`

**Step 1: Write failing test for FAQ skill wiring**

Create test in `src/rob_box_voice/test/unit/node/test_faq_event_mode.py` that asserts:
- event mode disabled -> `handle_faq` tool absent
- event mode enabled with config -> `handle_faq` tool present
- rendered system prompt includes event metadata and robot role

**Step 2: Run test to verify RED**

Run: `pytest src/rob_box_voice/test/unit/node/test_faq_event_mode.py -v`
Expected: FAIL because event mode hooks do not exist yet.

**Step 3: Implement FAQ skill and event config loading**

Requirements:
- FAQ skill subclasses `BaseSkill`
- Event mode controlled by config, not hardcoded prompt edits
- Event config includes `name`, `organization`, `location`, `date`, `description`, `robot_role`
- Event prompt tells the agent to use FAQ retrieval for event-specific questions and summarize long answers into spoken form
- Non-event questions continue to work through existing tools and default conversation flow

**Step 4: Run test to verify GREEN**

Run: `pytest src/rob_box_voice/test/unit/node/test_faq_event_mode.py -v`
Expected: PASS

**Step 5: Commit**

```bash
git add src/rob_box_voice/rob_box_voice/skills/faq_skill.py src/rob_box_voice/rob_box_voice/skills/__init__.py src/rob_box_voice/rob_box_voice/dialogue_node.py src/rob_box_voice/prompts/event_prompt.txt src/rob_box_voice/prompts/skills/faq_skill_prompt.txt src/rob_box_voice/prompts/compositor_prompt.txt docker/vision/config/voice_assistant/voice_assistant.yaml src/rob_box_voice/test/unit/node/test_faq_event_mode.py
git commit -m "feat(voice): add faq event mode"
```

### Task 5: Add Deployment Inputs For Real Events

**Files:**
- Create: `docker/vision/config/voice_assistant/event_mode.yaml.example`
- Modify: `docker/vision/docker-compose.yaml`
- Read: `docker/vision/scripts/voice_assistant/start_voice_assistant.sh`

**Step 1: Add event config example**

Config must support:
- event metadata
- robot role string
- faq file path
- optional short identity summary for spoken self-introduction

**Step 2: Wire environment/config into the voice service**

Requirements:
- mount config and FAQ file via volumes
- do not `COPY config/` or `COPY scripts/` into Dockerfiles
- event mode must be opt-in and safe when disabled

**Step 3: Run config sanity checks**

Run: `rg -n "event_mode|faq" docker/vision`
Expected: config paths and env vars are discoverable in compose and config examples.

**Step 4: Commit**

```bash
git add docker/vision/config/voice_assistant/event_mode.yaml.example docker/vision/docker-compose.yaml
git commit -m "feat(docker): add faq event mode config"
```

### Task 6: Verify End-To-End Behavior

**Files:**
- Modify: `progress.md`

**Step 1: Run targeted unit tests**

Run: `pytest src/rob_box_voice/test/unit/core/test_faq_store.py src/rob_box_voice/test/unit/core/test_faq_loader.py src/rob_box_voice/test/unit/node/test_faq_event_mode.py -v`
Expected: PASS

**Step 2: Run style checks on touched Python files**

Run: `black --check src/rob_box_voice/rob_box_voice/core/faq_store.py src/rob_box_voice/rob_box_voice/skills/faq_skill.py src/rob_box_voice/scripts/load_faq_event.py src/rob_box_voice/test/unit/core/test_faq_store.py src/rob_box_voice/test/unit/core/test_faq_loader.py src/rob_box_voice/test/unit/node/test_faq_event_mode.py`
Expected: PASS

Run: `isort --check-only src/rob_box_voice/rob_box_voice/core/faq_store.py src/rob_box_voice/rob_box_voice/skills/faq_skill.py src/rob_box_voice/scripts/load_faq_event.py src/rob_box_voice/test/unit/core/test_faq_store.py src/rob_box_voice/test/unit/core/test_faq_loader.py src/rob_box_voice/test/unit/node/test_faq_event_mode.py`
Expected: PASS

Run: `flake8 src/rob_box_voice/rob_box_voice/core/faq_store.py src/rob_box_voice/rob_box_voice/skills/faq_skill.py src/rob_box_voice/scripts/load_faq_event.py src/rob_box_voice/test/unit/core/test_faq_store.py src/rob_box_voice/test/unit/core/test_faq_loader.py src/rob_box_voice/test/unit/node/test_faq_event_mode.py`
Expected: PASS

**Step 3: Smoke test loader against the real workbook**

Run: `python src/rob_box_voice/scripts/load_faq_event.py --faq-file "/home/builder/Downloads/FAQ 2026 Общие вопросы + бак (1).xlsx" --event-name "open-day-2026" --dry-run`
Expected: prints normalized FAQ rows count and sample categories without modifying the main DB.

**Step 4: Update progress log**

Add a short note to `progress.md` with the implemented files and validation steps.

**Step 5: Commit**

```bash
git add progress.md
git commit -m "docs(progress): record faq event mode verification"
```