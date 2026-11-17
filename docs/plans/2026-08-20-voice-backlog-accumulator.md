# Voice Backlog Accumulator Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Речь без wake-слова не дропается, а копится в аккумулятор со скользящим окном и при следующем wake-word уходит в LLM блоком `<speech_backlog>` внутри `<system_context>`.

**Architecture:** Новый чистый класс `SpeechAccumulator` (core, без I/O) + два параметра конфига + правки `_on_stt` (накопление + флаг слива) и `_build_dynamic_system_context` (вставка XML-блока). Существующие тесты не ломаем через `getattr`-дефолты (паттерн уже используется для `_command_intent_gate_enabled`).

**Tech Stack:** Python 3.10+, ROS 2 Humble (rclpy), pytest, YAML-конфиги.

**Design:** `docs/plans/2026-08-20-voice-backlog-accumulator-design.md`

---

### Task 1: SpeechAccumulator — чистый класс + unit-тесты

**Files:**
- Create: `src/rob_box_voice/rob_box_voice/core/speech_accumulator.py`
- Test: `src/rob_box_voice/test/unit/core/test_speech_accumulator.py`

**Step 1: Write the failing test**

```python
"""Unit-тесты SpeechAccumulator — аккумулятор фоновой речи без wake-слова."""

import time

import pytest

from rob_box_voice.core.speech_accumulator import (
    SpeechAccumulator,
    format_ago_s,
)


class TestFormatAgoS:
    def test_seconds(self):
        assert format_ago_s(0) == "0с"
        assert format_ago_s(12) == "12с"
        assert format_ago_s(59) == "59с"

    def test_minutes(self):
        assert format_ago_s(60) == "1м0с"
        assert format_ago_s(185) == "3м5с"

    def test_hours(self):
        assert format_ago_s(3700) == "1ч1м"


class TestSpeechAccumulator:
    def test_empty_block_returns_none(self):
        acc = SpeechAccumulator()
        assert acc.format_block() is None

    def test_add_and_format(self):
        acc = SpeechAccumulator()
        acc.add("расскажи про погоду", speaker_tag="0", speaker_name="Антон")
        block = acc.format_block()
        assert block is not None
        assert "<speech_backlog>" in block
        assert 'speaker="Антон"' in block
        assert "расскажи про погоду" in block

    def test_unknown_speaker_defaults_to_neznakomets(self):
        acc = SpeechAccumulator()
        acc.add("привет", speaker_tag=None, speaker_name=None)
        block = acc.format_block()
        assert 'speaker="незнакомец"' in block

    def test_empty_text_ignored(self):
        acc = SpeechAccumulator()
        acc.add("   ", speaker_tag="0", speaker_name="Антон")
        assert acc.is_empty()

    def test_prune_expired_entries(self, monkeypatch):
        acc = SpeechAccumulator(window_sec=60.0)
        now = time.time()
        acc.add("старая фраза", speaker_tag="0", speaker_name="Антон")
        # подменяем timestamp записи в прошлое
        acc._entries[0]["ts"] = now - 120.0
        acc.prune(now=now)
        assert acc.is_empty()

    def test_max_entries_cap(self):
        acc = SpeechAccumulator(max_entries=2)
        for i in range(5):
            acc.add(f"фраза {i}", speaker_tag="0", speaker_name="Антон")
        assert len(acc._entries) == 2
        assert acc._entries[0]["text"] == "фраза 3"

    def test_clear(self):
        acc = SpeechAccumulator()
        acc.add("привет", speaker_tag="0", speaker_name="Антон")
        acc.clear()
        assert acc.is_empty()

    def test_xml_escape(self):
        acc = SpeechAccumulator()
        acc.add('а < b & c > "d"', speaker_tag="0", speaker_name="Антон")
        block = acc.format_block()
        assert "&lt;" in block and "&gt;" in block and "&amp;" in block
```

**Step 2: Run test to verify it fails**

Run: `.\.venv\Scripts\python.exe -m pytest src/rob_box_voice/test/unit/core/test_speech_accumulator.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'rob_box_voice.core.speech_accumulator'`

**Step 3: Write minimal implementation**

```python
#!/usr/bin/env python3
"""SpeechAccumulator — аккумулятор фоновой речи без wake-слова.

Чистый модуль: без I/O, без ROS2. DialogueNode держит один экземпляр,
кладет сюда распознанные фразы без wake-word и сливает их в
``<speech_backlog>`` при следующем wake-word (см. docs/plans/
2026-08-20-voice-backlog-accumulator-design.md).
"""

from __future__ import annotations

import time
from typing import List, Optional

DEFAULT_MAX_ENTRIES = 30

_INSTRUCTION = (
    "Ниже — фразы, услышанные без обращённого wake-слова до текущего "
    "обращения. Пользователь мог иметь в виду одну из них как запрос. "
    "Определи, о чём просили, и ответь по существу, не пересказывая "
    "эти фразы дословно."
)


def _xml_escape(value: str) -> str:
    return (
        value.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def format_ago_s(seconds: float) -> str:
    """Человекочитаемое «сколько назад» для записи бэклога."""
    seconds = max(0, int(seconds))
    if seconds < 60:
        return f"{seconds}с"
    minutes = seconds // 60
    if minutes < 60:
        return f"{minutes}м{seconds % 60}с"
    hours = minutes // 60
    return f"{hours}ч{minutes % 60}м"


class SpeechAccumulator:
    """Скользящее окно распознанной речи без wake-слова."""

    def __init__(
        self,
        window_sec: float = 180.0,
        max_entries: int = DEFAULT_MAX_ENTRIES,
    ) -> None:
        self.window_sec = window_sec
        self.max_entries = max_entries
        self._entries: List[dict] = []

    def add(
        self,
        text: str,
        speaker_tag: Optional[str] = None,
        speaker_name: Optional[str] = None,
    ) -> None:
        text = (text or "").strip()
        if not text:
            return
        self._entries.append(
            {
                "ts": time.time(),
                "text": text,
                "speaker_tag": speaker_tag,
                "speaker_name": speaker_name or "незнакомец",
            }
        )
        self._trim()

    def prune(self, now: Optional[float] = None) -> None:
        now = time.time() if now is None else now
        cutoff = now - self.window_sec
        self._entries = [e for e in self._entries if e["ts"] >= cutoff]
        self._trim()

    def _trim(self) -> None:
        if len(self._entries) > self.max_entries:
            self._entries = self._entries[-self.max_entries:]

    def is_empty(self) -> bool:
        return not self._entries

    def clear(self) -> None:
        self._entries.clear()

    def format_block(self, now: Optional[float] = None) -> Optional[str]:
        """XML-блок ``<speech_backlog>`` или ``None``, если пусто."""
        self.prune(now)
        if self.is_empty():
            return None
        now = time.time() if now is None else now
        lines = [
            "<speech_backlog>",
            f"  <instruction>{_xml_escape(_INSTRUCTION)}</instruction>",
        ]
        for entry in self._entries:
            ago_s = format_ago_s(now - entry["ts"])
            tag = entry["speaker_tag"] or "?"
            speaker = _xml_escape(entry["speaker_name"] or "незнакомец")
            text = _xml_escape(entry["text"])
            lines.append(
                f'  <entry speaker_tag="{tag}" speaker="{speaker}" '
                f'ago_s="{ago_s}">{text}</entry>'
            )
        lines.append("</speech_backlog>")
        return "\n".join(lines)
```

**Step 4: Run test to verify it passes**

Run: `.\.venv\Scripts\python.exe -m pytest src/rob_box_voice/test/unit/core/test_speech_accumulator.py -q`
Expected: PASS

**Step 5: Commit**

```bash
git add src/rob_box_voice/rob_box_voice/core/speech_accumulator.py src/rob_box_voice/test/unit/core/test_speech_accumulator.py
git commit -m "feat(voice): add SpeechAccumulator for no-wake speech backlog"
```

---

### Task 2: Параметры конфига (код + YAML)

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/dialogue_node.py` (метод `_declare_params`)
- Modify: `src/rob_box_voice/config/dialogue_node.yaml`
- Modify: `docker/vision/config/voice_assistant/dialogue_node.yaml`

**Step 1: Добавить declare_parameter**

В `_declare_params` после блока `self.declare_parameter("speaker_min_phrases", 2)` (или рядом с прочими voice-параметрами) добавить:

```python
        # Бэклог-аккумулятор фоновой речи без wake-слова (docs/plans/
        # 2026-08-20-voice-backlog-accumulator-design.md).
        self.declare_parameter("accumulate_no_wake_enabled", True)
        self.declare_parameter("accumulate_window_sec", 180.0)
```

**Step 2: Добавить ключи в src YAML**

В `src/rob_box_voice/config/dialogue_node.yaml` в секции `ros__parameters` после `speaker_min_phrases`-аналога (там его нет — добавить рядом с `speaker_id_enabled`/`speaker_db_path`) :

```yaml
    # Бэклог-аккумулятор фоновой речи без wake-слова.
    accumulate_no_wake_enabled: true
    accumulate_window_sec: 180.0
```

**Step 3: Добавить ключи в docker YAML (живой конфиг робота)**

В `docker/vision/config/voice_assistant/dialogue_node.yaml` в `ros__parameters` добавить те же два ключа:

```yaml
    # Бэклог-аккумулятор фоновой речи без wake-слова.
    accumulate_no_wake_enabled: true
    accumulate_window_sec: 180.0
```

**Step 4: Проверить консистентность YAML↔declare_parameter**

Run: `.\.venv\Scripts\python.exe -m pytest src/rob_box_voice/test/test_yaml_param_consistency.py -q`
Expected: PASS (новые ключи объявлены — иначе тест упадёт с «YAML-ключи ... не объявлены»)

**Step 5: Commit**

```bash
git add src/rob_box_voice/rob_box_voice/dialogue_node.py src/rob_box_voice/config/dialogue_node.yaml docker/vision/config/voice_assistant/dialogue_node.yaml
git commit -m "feat(voice): add accumulator config params"
```

---

### Task 3: Инициализация аккумулятора в `DialogueNode.__init__`

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/dialogue_node.py`

**Step 1: Добавить импорт**

В блок импортов из `rob_box_voice.core.*` (рядом с `from rob_box_voice.core.music_guard import ...`) добавить:

```python
from rob_box_voice.core.speech_accumulator import SpeechAccumulator
```

**Step 2: Инициализировать атрибуты**

После `self._speaker_lock = threading.Lock()` (в `__init__`, рядом с speaker-блоками) добавить:

```python
        # Бэклог-аккумулятор фоновой речи без wake-слова (docs/plans/
        # 2026-08-20-voice-backlog-accumulator-design.md).
        self._speech_accumulator = SpeechAccumulator(
            window_sec=float(self.get_parameter("accumulate_window_sec").value),
        )
        self._accumulate_no_wake_enabled = bool(
            self.get_parameter("accumulate_no_wake_enabled").value
        )
        self._pending_backlog_flush = False
```

**Step 3: Запустить smoke-проверку импорта**

Run: `.\.venv\Scripts\python.exe -m pytest src/rob_box_voice/test/unit/node/test_dialogue_node_imports.py -q`
Expected: PASS

**Step 4: Commit**

```bash
git add src/rob_box_voice/rob_box_voice/dialogue_node.py
git commit -m "feat(voice): init speech accumulator in dialogue_node"
```

---

### Task 4: Накопление в `_on_stt` (ветка no_wake_word)

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/dialogue_node.py` (метод `_on_stt`)

**Step 1: Заменить ветку no_wake_word**

Найти блок:

```python
        if tg_chat_id is None and not has_wake_word(text_lower, self._wake_words):
            self._llm_skipped_counter["no_wake_word"] += 1
            self.get_logger().info(
                f"🔇 [diagnostics] ignored: no_wake_word text={text[:60]!r} "
                f"state={state.name}"
            )
            self._maybe_log_skip_summary()
            return
```

Заменить на:

```python
        if tg_chat_id is None and not has_wake_word(text_lower, self._wake_words):
            accumulator = getattr(self, "_speech_accumulator", None)
            if getattr(self, "_accumulate_no_wake_enabled", False) and accumulator is not None:
                # Бэклог-аккумулятор: не дропаем, а копим фоновую речь
                # (текст + спикер + время) до следующего wake-слова.
                with self._speaker_lock:
                    sp = dict(getattr(self, "_current_speaker", {}) or {})
                sp_name = sanitize_speaker_name(sp.get("name")) if sp.get("is_known") else ""
                accumulator.add(
                    text,
                    speaker_tag=speaker_tag,
                    speaker_name=sp_name or None,
                )
                self.get_logger().info(
                    f"🗒️ [backlog] accumulated (no_wake_word) "
                    f"tag={speaker_tag!r} speaker={sp_name or 'незнакомец'!r} "
                    f"text={text[:60]!r}"
                )
            else:
                self._llm_skipped_counter["no_wake_word"] += 1
                self.get_logger().info(
                    f"🔇 [diagnostics] ignored: no_wake_word text={text[:60]!r} "
                    f"state={state.name}"
                )
                self._maybe_log_skip_summary()
            return
```

Примечание: `sanitize_speaker_name` уже импортирован в `dialogue_node.py`.

**Step 2: Запустить существующие тесты `_on_stt` (фичи off по умолчанию)**

Run: `.\.venv\Scripts\python.exe -m pytest src/rob_box_voice/test/unit/node/test_issue_1195_tg_source.py src/rob_box_voice/test/unit/node/test_command_intent_gate.py -q`
Expected: PASS (без новых атрибутов `getattr` даёт `False` → прежний дроп)

**Step 3: Commit**

```bash
git add src/rob_box_voice/rob_box_voice/dialogue_node.py
git commit -m "feat(voice): accumulate no-wake speech instead of dropping"
```

---

### Task 5: Слив бэклога в `<system_context>` + голое wake-слово

**Files:**
- Modify: `src/rob_box_voice/rob_box_voice/dialogue_node.py` (методы `_on_stt` и `_build_dynamic_system_context`)

**Step 1: В `_on_stt` — вычислить `backlog_pending` и не дропать голое wake**

Найти блок:

```python
        clean = strip_wake_word(text, self._wake_words)
        if not clean:
            self._llm_skipped_counter["empty_after_strip"] += 1
            self.get_logger().info(
                f"🔇 [diagnostics] ignored: empty_after_strip_wake text={text[:60]!r}"
            )
            return
```

Заменить на:

```python
        accumulator = getattr(self, "_speech_accumulator", None)
        backlog_pending = bool(
            getattr(self, "_accumulate_no_wake_enabled", False)
            and accumulator is not None
            and not accumulator.is_empty()
        )
        clean = strip_wake_word(text, self._wake_words)
        if not clean:
            if backlog_pending:
                # Голое wake-слово («робот»): user_input не должен быть
                # пустым — оставляем исходную фразу как сигнал, бэклог
                # уйдёт в <system_context>.
                clean = text
            else:
                self._llm_skipped_counter["empty_after_strip"] += 1
                self.get_logger().info(
                    f"🔇 [diagnostics] ignored: empty_after_strip_wake "
                    f"text={text[:60]!r}"
                )
                return
```

**Step 2: В `_on_stt` — поднять флаг слива перед dispatch**

Найти (сразу после command-intent gate, перед `self._cancel_run("new STT input")`):

```python
        self._cancel_run("new STT input")
```

Вставить перед этой строкой:

```python
        if backlog_pending:
            self._pending_backlog_flush = True
```

(Строка `self._cancel_run("new STT input")` остаётся на месте после вставки.)

**Step 3: В `_build_dynamic_system_context` — вставить блок перед закрытием тега**

Найти:

```python
        else:
            lines.append("  <generated_music>idle</generated_music>")
        lines.append("</system_context>")
```

Заменить на:

```python
        else:
            lines.append("  <generated_music>idle</generated_music>")
        # Бэклог-аккумулятор фоновой речи без wake-слова: при сливе добавляем
        # <speech_backlog> внутрь <system_context>. raw_user_command при этом
        # не трогаем — гарды смотрят только на текущую фразу.
        if getattr(self, "_pending_backlog_flush", False):
            self._pending_backlog_flush = False
            acc = getattr(self, "_speech_accumulator", None)
            if acc is not None:
                block = acc.format_block()
                if block:
                    lines.append(block)
                acc.clear()
        lines.append("</system_context>")
```

**Step 4: Запустить тесты диалога (smoke)**

Run: `.\.venv\Scripts\python.exe -m pytest src/rob_box_voice/test/unit/node/test_dialogue_node.py src/rob_box_voice/test/test_dialogue_shell.py -q`
Expected: PASS

**Step 5: Commit**

```bash
git add src/rob_box_voice/rob_box_voice/dialogue_node.py
git commit -m "feat(voice): flush speech backlog into system_context on wake"
```

---

### Task 6: Node-тесты полного потока

**Files:**
- Test: `src/rob_box_voice/test/unit/node/test_speech_backlog_accumulator.py`

**Step 1: Написать тесты (fixture в стиле test_issue_1195_tg_source.py)**

```python
"""Node-тесты бэклог-аккумулятора фоновой речи без wake-слова."""

from unittest.mock import MagicMock

import pytest

from rob_box_voice.core.speech_accumulator import SpeechAccumulator
from rob_box_voice.dialogue_node import DialogueNode


@pytest.fixture
def node():
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger

    n._wake_words = ["робок", "робот", "роббокс", "робокс", "robbox", "rob box"]
    n._dsm = MagicMock()
    n._dsm.current_state = MagicMock()
    from rob_box_harness.core.dialogue_state_machine import (
        DialogueStateKind,
    )
    n._dsm.current_state.name = DialogueStateKind.IDLE.name
    n._dj = MagicMock()
    n._dj.state.enabled = False
    n._cancel_run = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._publish_state = MagicMock()
    n._dispatch_turn = MagicMock()
    n._verbose_llm = False
    n._speaker_by_text = {}
    n._speaker_lock = MagicMock()
    n._speaker_lock.__enter__ = MagicMock(return_value=None)
    n._speaker_lock.__exit__ = MagicMock(return_value=False)
    n._current_speaker = {"is_known": False}
    n._llm_skipped_counter = {
        "no_wake_word": 0,
        "silenced": 0,
        "silence_command": 0,
        "empty_after_strip": 0,
        "stt_rejected": 0,
        "music_stop": 0,
    }
    n._maybe_log_skip_summary = MagicMock()
    n._active_tg_chat_id = None
    n._command_parser = MagicMock()
    n._command_parser.parse.return_value = MagicMock(
        intent=MagicMock(value="UNKNOWN"), confidence=0.0
    )
    n._command_intent_gate_enabled = False

    n._speech_accumulator = SpeechAccumulator(window_sec=180.0)
    n._accumulate_no_wake_enabled = True
    n._pending_backlog_flush = False
    return n


def _stt(data):
    msg = MagicMock()
    msg.data = data
    return msg


class TestNoWakeAccumulates:
    def test_no_wake_text_accumulates_not_dropped(self, node):
        node._on_stt(_stt("расскажи про погоду"))
        assert not node._speech_accumulator.is_empty()
        node._dispatch_turn.assert_not_called()
        assert node._llm_skipped_counter["no_wake_word"] == 0

    def test_feature_off_drops(self, node):
        node._accumulate_no_wake_enabled = False
        node._on_stt(_stt("расскажи про погоду"))
        assert node._speech_accumulator.is_empty()
        assert node._llm_skipped_counter["no_wake_word"] == 1


class TestWakeFlushes:
    def test_wake_with_backlog_sets_flush_flag_and_dispatches(self, node):
        node._on_stt(_stt("расскажи про погоду"))
        node._on_stt(_stt("робот включи свет"))
        assert node._pending_backlog_flush is True
        node._dispatch_turn.assert_called_once()

    def test_bare_wake_word_flushes_backlog(self, node):
        node._on_stt(_stt("расскажи про погоду"))
        node._on_stt(_stt("робот"))
        assert node._pending_backlog_flush is True
        # user_input не пустой: исходная фраза «робот»
        dispatched = node._dispatch_turn.call_args.args[0]
        assert dispatched == "робот"
        node._dispatch_turn.assert_called_once()

    def test_empty_backlog_no_flag(self, node):
        node._on_stt(_stt("робот включи свет"))
        assert node._pending_backlog_flush is False


class TestBuildDynamicContextFlushes:
    def test_flush_inserts_block_and_clears(self, node):
        node._on_stt(_stt("расскажи про погоду"))
        node._on_stt(_stt("робот"))
        ctx = node._build_dynamic_system_context()
        assert "<speech_backlog>" in ctx
        assert "расскажи про погоду" in ctx
        assert node._pending_backlog_flush is False
        assert node._speech_accumulator.is_empty()
```

Примечание: `_build_dynamic_system_context` обращается к `_current_speaker`, параметру `tts_provider` (через `get_parameter`) и т.д. Если в тесте это падает — дополнить fixture: `n.get_parameter = MagicMock(return_value=MagicMock(value="minimax"))`, `n._actual_tts_provider = None`, `n._actual_tts_voice = None`, `n._current_tts_voice = None`, `n._generated_music_state = {}`, `n._scheduler_executor = None`.

**Step 2: Run test to verify it fails**

Run: `.\.venv\Scripts\python.exe -m pytest src/rob_box_voice/test/unit/node/test_speech_backlog_accumulator.py -q`
Expected: FAIL (реализация ещё не добавлена — Task 4/5 ещё не сделаны, если идти строго по TDD; если Tasks 4–5 уже готовы, то PASS — тогда просто проверяем зелёный)

**Step 3: Commit**

```bash
git add src/rob_box_voice/test/unit/node/test_speech_backlog_accumulator.py
git commit -m "test(voice): backlog accumulator node flow"
```

---

### Task 7: Финальная проверка

**Step 1: Полный прогон релевантных тестов**

Run: `.\.venv\Scripts\python.exe -m pytest src/rob_box_voice/test/unit/core/test_speech_accumulator.py src/rob_box_voice/test/unit/node/test_speech_backlog_accumulator.py src/rob_box_voice/test/test_yaml_param_consistency.py src/rob_box_voice/test/unit/node/test_issue_1195_tg_source.py src/rob_box_voice/test/unit/node/test_command_intent_gate.py -q`
Expected: PASS

**Step 2: Линт (flake8/black) на изменённые файлы**

Run: `.\.venv\Scripts\python.exe -m flake8 src/rob_box_voice/rob_box_voice/core/speech_accumulator.py src/rob_box_voice/rob_box_voice/dialogue_node.py src/rob_box_voice/test/unit/core/test_speech_accumulator.py src/rob_box_voice/test/unit/node/test_speech_backlog_accumulator.py`
Expected: чисто (0 ошибок)

**Step 3: Итоговый diff-ревью**

Run: `git status --short; git diff --stat HEAD`
Expected: только целевые файлы, без мусора.

---

## Handoff

После выполнения плана — `requesting-code-review` (перед PR), мерж только товарищем Шифу. Деплой — только через GitHub Actions.
