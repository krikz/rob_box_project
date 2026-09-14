"""
test_issue_2347_n313_music_state.py — Regression guards for issue #2347
(n313 silence_restored): LLM должен звать `get_music_state` на state-вопросе,
а не отвечать verbal-only на основе stale <music_state> тега.

Покрывает:
- В master_prompt_compact.txt есть RULE #MUSIC-STATE с триггер-словами.
- В ``_build_dynamic_system_context()`` есть третий ``<reminder>`` блок
  про ``get_music_state`` (между stop_music и time).
- Позиция: новый reminder идёт МЕЖДУ stop_music и time, не сдвигая
  ``reminders[-1] == time-reminder`` контракт
  ``test_issue_1777_time_format.py``.

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
#  Fixtures
# ────────────────────────────────────────────────────────────────────────


def _make_node(parameters: dict | None = None):  # type: ignore[no-untyped-def]
    """Минимальный DialogueNode для ``_build_dynamic_system_context``.

    Зеркало test_issue_1777_time_format.py::_make_node — чтобы оба теста
    жили одним контрактом.
    """
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


def _composer_path() -> Path:
    repo_root = Path(__file__).resolve().parents[5]
    return repo_root / "src" / "rob_box_voice" / "prompts" / "skills" / "composer.txt"


# ────────────────────────────────────────────────────────────────────────
#  Prompt: RULE #MUSIC-STATE (issue #2347)
# ────────────────────────────────────────────────────────────────────────


class TestPromptMusicStateRule:
    """System prompt содержит явное правило для LLM про state-запросы."""

    @pytest.fixture(scope="class")
    def prompt_text(self) -> str:
        path = _prompt_path()
        assert path.exists(), f"prompt not found: {path}"
        return path.read_text(encoding="utf-8")

    def test_rule_music_state_block_present(self, prompt_text: str):
        """В prompt есть RULE #MUSIC-STATE — заголовок виден LLM при старте."""
        assert "RULE #MUSIC-STATE" in prompt_text
        assert "n313_silence_restored" in prompt_text

    def test_rule_lists_state_trigger_phrases(self, prompt_text: str):
        """Ключевые триггер-слова из acceptance перечислены в rule."""
        # Достаточно проверить несколько representative — полный список
        # в спеке: «тихо?», «тишина?», «тише?», «играет ли музыка?», «что
        # играет?», «что сейчас играет?», «музыка включена?», «any music?».
        triggers = (
            "тихо?",
            "тишина?",
            "играет ли музыка?",
            "что играет?",
            "any music?",
        )
        # Извлекаем именно блок RULE #MUSIC-STATE, не весь файл.
        m = re.search(
            r"RULE #MUSIC-STATE.*?(?=\n🚨 \*\*RULE #)",
            prompt_text,
            flags=re.DOTALL,
        )
        assert m, "RULE #MUSIC-STATE block not followed by another RULE"
        block = m.group(0)
        missing = [t for t in triggers if t not in block]
        assert not missing, f"triggers missing from RULE #MUSIC-STATE: {missing}"

    def test_rule_mentions_get_music_state(self, prompt_text: str):
        """Tool указан явно — без него LLM не знает, что вызывать."""
        m = re.search(
            r"RULE #MUSIC-STATE.*?(?=\n🚨 \*\*RULE #)",
            prompt_text,
            flags=re.DOTALL,
        )
        assert m
        block = m.group(0)
        assert "get_music_state" in block
        assert "<music_state>" in block
        # Rationale про stale тег — ключ к поведению LLM.
        assert "stale" in block

    def test_rule_has_do_and_dont_examples(self, prompt_text: str):
        """В блоке есть ✅ и ❌ — модель учится на примерах."""
        m = re.search(
            r"RULE #MUSIC-STATE.*?(?=\n🚨 \*\*RULE #)",
            prompt_text,
            flags=re.DOTALL,
        )
        assert m
        block = m.group(0)
        assert "✅" in block
        assert "❌" in block


# ────────────────────────────────────────────────────────────────────────
#  Dynamic system context: <reminder> для get_music_state (issue #2347)
# ────────────────────────────────────────────────────────────────────────


@pytest.mark.skipif(
    _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK,
    reason=_SKIP_REASON,
)
class TestDynamicContextMusicStateReminder:
    """``_build_dynamic_system_context()`` содержит напоминание про tool."""

    def test_reminder_block_present(self):
        n = _make_node(parameters={"tts_provider": "minimax"})
        ctx = n._build_dynamic_system_context()

        assert "<reminder>" in ctx
        # Должен быть reminder про get_music_state.
        assert "get_music_state" in ctx

    def test_reminder_position_between_stop_music_and_time(self):
        """КРИТИЧНО: новый reminder идёт МЕЖДУ stop_music и time.

        ``reminders[-1] == time-reminder`` — контракт
        ``test_issue_1777_time_format.py``. Если новый reminder
        попадёт ПОСЛЕ time — те сломаются.
        """
        n = _make_node()
        ctx = n._build_dynamic_system_context()

        reminders = re.findall(r"<reminder>(.*?)</reminder>", ctx, flags=re.DOTALL)
        # Минимум 3 reminder блока: stop_music, get_music_state, time.
        assert len(reminders) >= 3, (
            f"expected ≥3 reminder blocks (stop_music, get_music_state, "
            f"time), got {len(reminders)}"
        )
        # Последний — по-прежнему time.
        time_reminder = reminders[-1]
        assert "get_current_time" in time_reminder, (
            "reminders[-1] должен быть time-reminder; "
            "новый reminder не должен сдвигать его"
        )
        # Средний (или один из средних) — про get_music_state.
        music_state_reminder = reminders[-2]
        assert "get_music_state" in music_state_reminder, (
            f"reminders[-2] должен содержать get_music_state; "
            f"got: {music_state_reminder[:200]!r}"
        )
        # Первый — по-прежнему stop_music.
        stop_music_reminder = reminders[0]
        assert "stop_music" in stop_music_reminder, (
            "reminders[0] должен быть stop_music reminder"
        )

    def test_reminder_lists_state_trigger_phrases(self):
        """Ключевые триггер-слова из acceptance перечислены в reminder'е."""
        n = _make_node()
        ctx = n._build_dynamic_system_context()

        reminders = re.findall(r"<reminder>(.*?)</reminder>", ctx, flags=re.DOTALL)
        # Ищем именно reminder про get_music_state — он по позиции
        # между stop_music и time, т.е. reminders[-2].
        music_state_reminder = reminders[-2]
        for trigger in ("тихо?", "тишина?", "играет ли музыка?", "что играет?"):
            assert trigger in music_state_reminder, (
                f"trigger {trigger!r} missing from get_music_state reminder"
            )

    def test_reminder_mentions_stale_rationale(self):
        """Rationale про stale snapshot — ключ к поведению LLM."""
        n = _make_node()
        ctx = n._build_dynamic_system_context()

        reminders = re.findall(r"<reminder>(.*?)</reminder>", ctx, flags=re.DOTALL)
        music_state_reminder = reminders[-2]
        assert "stale" in music_state_reminder


# ────────────────────────────────────────────────────────────────────────
#  Composer skill: state-вопрос (issue #2347, опц. hunk 3 из спеки)
# ────────────────────────────────────────────────────────────────────────


class TestComposerSkillMusicState:
    """``composer.txt`` упоминает обязательность get_music_state на state-вопросе."""

    def test_state_question_phrase_listed(self):
        path = _composer_path()
        assert path.exists(), f"composer skill not found: {path}"
        text = path.read_text(encoding="utf-8")

        # get_music_state абзац должен упоминать state-вопрос юзера.
        # Извлекаем блок описания tool (внутри "- `get_music_state` ... \n- `stop_music`").
        m = re.search(
            r"- `get_music_state`.*?(?=\n- `stop_music`)",
            text,
            flags=re.DOTALL,
        )
        assert m, "`get_music_state` block not found before `stop_music` block"
        block = m.group(0)
        # Минимум один representative триггер должен быть в этом блоке.
        assert "тихо?" in block or "играет ли музыка?" in block, (
            "state-вопрос не упомянут в описании get_music_state"
        )
        # Прямое указание обязательности — ключевое слово из спеки.
        assert "ОБЯЗАТЕЛЬНО" in block


# ────────────────────────────────────────────────────────────────────────
#  Phrase-trigger contract (state-вопросы)
# ────────────────────────────────────────────────────────────────────────


class TestStatePhraseContract:
    """Контракт: state-фразы должны покрываться reminder'ом и RULE."""

    # Полный список 10 триггеров из спеки. RULE содержит ВСЕ 10 (как
    # статическая память LLM на старте сессии). Напоминание в dynamic
    # context содержит 8 representative-фраз (без «есть звук?» и «any
    # music?» — это периферийные формулировки, для краткости текста).
    # См. спека §2 Hunk 2.
    RULE_TRIGGERS = (
        "тихо?",
        "тишина?",
        "тише?",
        "играет ли музыка?",
        "что сейчас играет?",
        "что играет?",
        "музыка включена?",
        "есть звук?",
        "слышно что-нибудь?",
        "any music?",
    )

    # То, что фактически есть в reminder'е (спек §2 Hunk 2, ровно эти 8).
    REMINDER_TRIGGERS = (
        "тихо?",
        "тишина?",
        "тише?",
        "играет ли музыка?",
        "что играет?",
        "что сейчас играет?",
        "музыка включена?",
        "слышно что-нибудь?",
    )

    def test_all_reminder_triggers_listed_in_reminder(self):
        if _IS_PY_311_PLUS or not _DIALOGUE_NODE_IMPORT_OK:
            pytest.skip(_SKIP_REASON)
        n = _make_node()
        ctx = n._build_dynamic_system_context()

        reminders = re.findall(r"<reminder>(.*?)</reminder>", ctx, flags=re.DOTALL)
        assert len(reminders) >= 3
        music_state_reminder = reminders[-2]
        missing = [t for t in self.REMINDER_TRIGGERS if t not in music_state_reminder]
        assert not missing, (
            f"triggers missing from get_music_state reminder: {missing}"
        )

    def test_all_rule_triggers_listed_in_rule(self):
        path = _prompt_path()
        prompt = path.read_text(encoding="utf-8")

        m = re.search(
            r"RULE #MUSIC-STATE.*?(?=\n🚨 \*\*RULE #)",
            prompt,
            flags=re.DOTALL,
        )
        assert m, "RULE #MUSIC-STATE block not followed by another RULE"
        rule = m.group(0)
        missing = [t for t in self.RULE_TRIGGERS if t not in rule]
        assert not missing, f"triggers missing from RULE #MUSIC-STATE: {missing}"