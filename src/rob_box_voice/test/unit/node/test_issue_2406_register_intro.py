"""
test_issue_2406_register_intro.py — Regression guards for issue #2406
(n201_sasha_intro_long, n204_boris_intro_long): LLM должен звать
``register_speaker`` на intro-сценарии («давай знакомиться, меня зовут Саша»),
а не отвечать verbal-only «Приятно познакомиться!» на основе
``<name>unknown</name>`` тега.

Покрывает:
- В ``master_prompt_compact.txt`` есть RULE #REGISTER с триггер-словами
  («давай знакомиться», «меня зовут …», и т.п.).
- В ``_build_dynamic_system_context()`` есть четвёртый ``<reminder>`` блок
  про ``register_speaker``, МЕЖДУ stop_music и get_music_state reminders.
- Позиция: новый reminder идёт ПОСЛЕ stop_music, НО ПЕРЕД get_music_state —
  это сохраняет оба существующих контракта:
    * ``reminders[-1] == time-reminder`` (test_issue_1777_time_format)
    * ``reminders[-2] == get_music_state-reminder`` (test_issue_2347)

Не требует ROS2 — rclpy замокан в conftest.py; dataclass mutable-default
bug в rob_box_core.tool_catalog даёт skip на Python 3.11+ (CI на 3.10).
"""

import re
import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_IS_PY_311_PLUS = sys.version_info >= (3, 11)
_SKIP_REASON = (
    "Pre-existing dataclass mutable-default bug in rob_box_core.tool_catalog; "
    "CI runs Python 3.10 where this is a soft warning, not a raise. "
    "Skip locally; the CI will validate these tests."
)

try:
    from rob_box_voice.dialogue_node import DialogueNode

    _DIALOGUE_NODE_IMPORT_OK = True
except ImportError:
    DialogueNode = None  # type: ignore[assignment]
    _DIALOGUE_NODE_IMPORT_OK = False
except Exception:  # noqa: BLE001
    DialogueNode = None  # type: ignore[assignment]
    _DIALOGUE_NODE_IMPORT_OK = False


# ────────────────────────────────────────────────────────────────────────
#  Fixtures (зеркало test_issue_2347_n313_music_state.py)
# ────────────────────────────────────────────────────────────────────────


def _make_node(parameters: dict | None = None):  # type: ignore[no-untyped-def]
    """Минимальный DialogueNode для ``_build_dynamic_system_context``."""
    values = parameters or {}
    assert DialogueNode is not None
    n = object.__new__(DialogueNode)  # type: ignore[arg-type]
    logger = MagicMock()
    n._logger = logger
    n.get_logger = lambda: logger

    def _gp(name):
        return type("P", (), {"value": values.get(name)})()

    n.get_parameter = _gp

    n._current_speaker = {"is_known": False}
    n._speaker_lock = MagicMock()
    n._speaker_lock.__enter__ = MagicMock(return_value=n._speaker_lock)
    n._speaker_lock.__exit__ = MagicMock(return_value=False)

    n._actual_tts_provider = None
    n._actual_tts_voice = None
    n._current_tts_voice = None

    n._pose_snapshot = None
    n._scheduler_executor = None
    n._pending_backlog_flush = False
    n._speech_accumulator = None

    n._build_music_state_snapshot = lambda: "  <music_state>stub</music_state>"
    return n


def _prompt_path() -> Path:
    repo_root = Path(__file__).resolve().parents[5]
    return (
        repo_root
        / "src"
        / "rob_box_voice"
        / "prompts"
        / "master_prompt_compact.txt"
    )


# ────────────────────────────────────────────────────────────────────────
#  Prompt: RULE #REGISTER (issue #2406)
# ────────────────────────────────────────────────────────────────────────


class TestPromptRegisterRule:
    """System prompt содержит явное правило для LLM про intro-сценарий."""

    @pytest.fixture(scope="class")
    def prompt_text(self) -> str:
        path = _prompt_path()
        assert path.exists(), f"prompt not found: {path}"
        return path.read_text(encoding="utf-8")

    def test_rule_register_block_present(self, prompt_text: str):
        """В prompt есть RULE #REGISTER — заголовок виден LLM при старте."""
        assert "RULE #REGISTER" in prompt_text, (
            "RULE #REGISTER отсутствует в master_prompt_compact.txt — "
            "LLM не знает, что register_speaker обязателен на intro"
        )

    def test_rule_mentions_intro_scenario_steps(self, prompt_text: str):
        """Tool указан явно + n201/n204 явно упомянуты как e2e-гейты."""
        m = re.search(
            r"RULE #REGISTER.*?(?=\n🚨 \*\*RULE #)",
            prompt_text,
            flags=re.DOTALL,
        )
        assert m, "RULE #REGISTER block not followed by another RULE"
        block = m.group(0)
        assert "register_speaker" in block
        # n201/n204 — конкретные e2e-гейты, ради которых fix делается.
        assert "n201" in block and "n204" in block, (
            "RULE #REGISTER должен явно ссылаться на n201/n204 "
            "(иначе при ревью будет непонятно, зачем этот RULE)"
        )
        # Связь с issue #2406 — для traceability. Раньше здесь искался сам
        # номер карточки, но из промпта он убран (#2765): модели он ничего
        # не говорит, а утечь в реплику может. Якорем стал ЗАПРЕТ, ради
        # которого правило писалось, — не отвечать приветствием по одному
        # лишь тегу, который может быть stale.
        assert "stale" in block, (
            "RULE #REGISTER must keep the explicit warning that the "
            "<name> tag may be stale — this is the anchor that ties it "
            "to issue #2406"
        )

    def test_rule_lists_intro_trigger_phrases(self, prompt_text: str):
        """Ключевые триггер-слова из acceptance перечислены в rule."""
        # Из n201: «давай знакомиться», «меня зовут». Из n204: «я Борис»,
        # «запомни мой голос». Покрываем основные формулировки intro.
        triggers = (
            "давай знакомиться",
            "я ",
            "зовут меня",
            "меня зовут",
            "привет, я",
            "запомни меня как",
        )
        m = re.search(
            r"RULE #REGISTER.*?(?=\n🚨 \*\*RULE #)",
            prompt_text,
            flags=re.DOTALL,
        )
        assert m
        block = m.group(0)
        missing = [t for t in triggers if t not in block]
        assert not missing, f"triggers missing from RULE #REGISTER: {missing}"

    def test_rule_has_do_and_dont_examples(self, prompt_text: str):
        """В блоке есть ✅ и ❌ — модель учится на примерах."""
        m = re.search(
            r"RULE #REGISTER.*?(?=\n🚨 \*\*RULE #)",
            prompt_text,
            flags=re.DOTALL,
        )
        assert m
        block = m.group(0)
        assert "✅" in block
        assert "❌" in block

    def test_rule_explains_stale_name_rationale(self, prompt_text: str):
        """Rationale: <name>unknown</name> тег stale для нового юзера."""
        m = re.search(
            r"RULE #REGISTER.*?(?=\n🚨 \*\*RULE #)",
            prompt_text,
            flags=re.DOTALL,
        )
        assert m
        block = m.group(0)
        assert "stale" in block, (
            "RULE #REGISTER должен объяснить, почему verbal-only «опасен» "
            "(stale <name>unknown</name> → нельзя опираться)"
        )


# ────────────────────────────────────────────────────────────────────────
#  Dynamic system context: <reminder> для register_speaker (issue #2406)
# ────────────────────────────────────────────────────────────────────────


@pytest.mark.skipif(
    _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
    reason=_SKIP_REASON,
)
class TestDynamicContextRegisterReminder:
    """``_build_dynamic_system_context()`` содержит напоминание про tool."""

    def test_reminder_block_present(self):
        n = _make_node(parameters={"tts_provider": "minimax"})
        ctx = n._build_dynamic_system_context()

        assert "<reminder>" in ctx
        # Должен быть reminder про register_speaker.
        assert "register_speaker" in ctx, (
            "register_speaker reminder отсутствует в dynamic system context"
        )

    def test_reminder_position_between_stop_music_and_get_music_state(self):
        """КРИТИЧНО: новый reminder идёт МЕЖДУ stop_music и get_music_state.

        Контракты существующих тестов:
          * ``reminders[-1] == time-reminder`` (test_issue_1777_time_format)
          * ``reminders[-2] == get_music_state-reminder`` (test_issue_2347)

        Новый reminder для register_speaker занимает ``reminders[1]``
        (сразу после stop_music, ПЕРЕД get_music_state). При таком
        расположении оба контракта выше остаются зелёными.

        Если Шифу однажды переставит reminder — упадёт либо
        test_issue_1777_time_format, либо test_issue_2347, плюс наш
        новый тест ниже.
        """
        n = _make_node()
        ctx = n._build_dynamic_system_context()

        reminders = re.findall(r"<reminder>(.*?)</reminder>", ctx, flags=re.DOTALL)
        # Текущее количество reminder'ов после PR #2406: 4
        # (stop_music, register_speaker, get_music_state, time).
        assert len(reminders) >= 4, (
            f"expected ≥4 reminder blocks (stop_music, register_speaker, "
            f"get_music_state, time), got {len(reminders)}"
        )
        # Последний — по-прежнему time.
        time_reminder = reminders[-1]
        assert "get_current_time" in time_reminder, (
            "reminders[-1] должен быть time-reminder; "
            "новый reminder не должен сдвигать его"
        )
        # reminders[-2] — по-прежнему get_music_state.
        music_state_reminder = reminders[-2]
        assert "get_music_state" in music_state_reminder, (
            f"reminders[-2] должен содержать get_music_state; "
            f"got: {music_state_reminder[:200]!r}"
        )
        # reminders[0] — по-прежнему stop_music.
        stop_music_reminder = reminders[0]
        assert "stop_music" in stop_music_reminder, (
            "reminders[0] должен быть stop_music reminder"
        )
        # Новый reminders[1] — register_speaker.
        register_reminder = reminders[1]
        assert "register_speaker" in register_reminder, (
            f"reminders[1] должен содержать register_speaker; "
            f"got: {register_reminder[:200]!r}"
        )

    def test_reminder_lists_intro_trigger_phrases(self):
        """Ключевые intro-фразы из acceptance перечислены в reminder'е."""
        n = _make_node()
        ctx = n._build_dynamic_system_context()

        reminders = re.findall(r"<reminder>(.*?)</reminder>", ctx, flags=re.DOTALL)
        register_reminder = reminders[1]
        # Минимум: «давай знакомиться» + «меня зовут» — самые частотные
        # формулировки из acceptance (n201, n204).
        for trigger in ("давай знакомиться", "меня зовут"):
            assert trigger in register_reminder, (
                f"intro trigger {trigger!r} missing from register_speaker reminder"
            )

    def test_reminder_mentions_stale_name_rationale(self):
        """Rationale: <name>unknown</name> тег stale, бери имя из user_input."""
        n = _make_node()
        ctx = n._build_dynamic_system_context()

        reminders = re.findall(r"<reminder>(.*?)</reminder>", ctx, flags=re.DOTALL)
        register_reminder = reminders[1]
        assert "stale" in register_reminder, (
            "register_speaker reminder должен упоминать stale <name> тег"
        )


# ────────────────────────────────────────────────────────────────────────
#  Phrase-trigger contract (intro-фразы)
# ────────────────────────────────────────────────────────────────────────


class TestIntroPhraseContract:
    """Контракт: intro-фразы должны покрываться reminder'ом и RULE."""

    # Полный список 6 триггеров из спеки. RULE содержит ВСЕ 6 (как
    # статическая память LLM на старте сессии). Напоминание в dynamic
    # context содержит 6 representative-фраз (как и в RULE — короткий
    # список, умещается в один reminder).
    RULE_TRIGGERS = (
        "давай знакомиться",
        "я ",
        "зовут меня",
        "меня зовут",
        "привет, я",
        "запомни меня как",
    )

    # То, что фактически есть в reminder'е (см. спека issue #2406).
    # Полный список — 6 фраз, ни одной не пропускаем (RULE+reminder
    # должны быть консистентны: список маленький, добавлять всё).
    REMINDER_TRIGGERS = (
        "давай знакомиться",
        "я ",
        "зовут меня",
        "меня зовут",
        "привет, я",
        "запомни меня как",
    )

    def test_all_reminder_triggers_listed_in_reminder(self):
        if _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK:
            pytest.skip(_SKIP_REASON)
        n = _make_node()
        ctx = n._build_dynamic_system_context()

        reminders = re.findall(r"<reminder>(.*?)</reminder>", ctx, flags=re.DOTALL)
        assert len(reminders) >= 4
        register_reminder = reminders[1]
        missing = [t for t in self.REMINDER_TRIGGERS if t not in register_reminder]
        assert not missing, (
            f"triggers missing from register_speaker reminder: {missing}"
        )

    def test_all_rule_triggers_listed_in_rule(self):
        path = _prompt_path()
        prompt = path.read_text(encoding="utf-8")

        m = re.search(
            r"RULE #REGISTER.*?(?=\n🚨 \*\*RULE #)",
            prompt,
            flags=re.DOTALL,
        )
        assert m, "RULE #REGISTER block not followed by another RULE"
        rule = m.group(0)
        missing = [t for t in self.RULE_TRIGGERS if t not in rule]
        assert not missing, (
            f"triggers missing from RULE #REGISTER: {missing}"
        )


# ────────────────────────────────────────────────────────────────────────
#  Backward-compat: existing reminders сохранили свою позицию
# ────────────────────────────────────────────────────────────────────────


@pytest.mark.skipif(
    _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
    reason=_SKIP_REASON,
)
class TestExistingRemindersUnaffected:
    """PR #2406 НЕ должен сломать позиционные контракты PR #1777 / #2347."""

    def test_time_reminder_still_last(self):
        """time-reminder остаётся reminders[-1] (контракт PR #1777)."""
        n = _make_node()
        ctx = n._build_dynamic_system_context()

        reminders = re.findall(r"<reminder>(.*?)</reminder>", ctx, flags=re.DOTALL)
        time_reminder = reminders[-1]
        assert "get_current_time" in time_reminder
        assert "formatted_time" in time_reminder

    def test_get_music_state_reminder_still_second_to_last(self):
        """get_music_state-reminder остаётся reminders[-2] (контракт PR #2347)."""
        n = _make_node()
        ctx = n._build_dynamic_system_context()

        reminders = re.findall(r"<reminder>(.*?)</reminder>", ctx, flags=re.DOTALL)
        music_state_reminder = reminders[-2]
        assert "get_music_state" in music_state_reminder
        assert "stale" in music_state_reminder

    def test_stop_music_reminder_still_first(self):
        """stop_music-reminder остаётся reminders[0] (контракт PR #1544)."""
        n = _make_node()
        ctx = n._build_dynamic_system_context()

        reminders = re.findall(r"<reminder>(.*?)</reminder>", ctx, flags=re.DOTALL)
        stop_music_reminder = reminders[0]
        assert "stop_music" in stop_music_reminder