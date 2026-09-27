"""
test_e2e_act_session_reset.py — issue #2890: сброс сессии dialogue_node перед актом E2E.

Акты E2E были изолированы только по БД дикторов: окно ходов LLM переживало
переход между актами. Прогон акта 2b 35903232434 сразу после акта 2
35900007906 — в контексте LLM стоял ход акта 2 «[Spkr:Саша] поищи … про
чай», и новой Саше робот сказал факт старой.

Харнесс теперь в начале каждого акта делает ``ros2 param set /dialogue_node
e2e_session_reset_token <токен>`` и читает токен обратно. Контракт узла:
  * новое значение = тот же ``_reset_dialogue_session``, что у «новой
    сессии», но МОЛЧА (без фразы и IMMUNE-окна TTS — иначе фраза попала бы
    в запись первого шага акта);
  * окно ходов действительно очищено (``AgentCore.clear_history`` в
    asyncio-цикле) ДО ответа колбэка;
  * не удалось очистить — ``successful=False``, харнесс увидит несовпадение
    токена и зафейлит акт;
  * «новая сессия» голосом по-прежнему говорит подтверждение.

Приём — ``object.__new__(DialogueNode)`` + ручные атрибуты, как в
``test_new_session_reset.py``; asyncio-цикл — настоящий, в отдельном
потоке (как в ноде).
"""

from __future__ import annotations

import asyncio
import threading
from unittest.mock import MagicMock

import pytest

from rob_box_voice.dialogue_node import DialogueNode


class _Param:
    def __init__(self, name, value):
        self.name = name
        self.value = value


class _Core:
    """Окно ходов AgentCore в миниатюре: clear_history очищает список."""

    def __init__(self, turns, fail: bool = False):
        self.turns = list(turns)
        self.fail = fail

    def clear_history(self):
        if self.fail:
            raise RuntimeError("window locked")
        self.turns.clear()


@pytest.fixture
def loop():
    lp = asyncio.new_event_loop()
    th = threading.Thread(target=lp.run_forever, daemon=True)
    th.start()
    yield lp
    lp.call_soon_threadsafe(lp.stop)
    th.join(timeout=5)
    lp.close()


def _make_node(loop, core):
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._loop = loop
    n._core = core
    n._cancel_run = MagicMock()
    n._reset_session_music_and_dj = MagicMock()
    n._tts_control_pub = MagicMock()
    n._pending_backlog_flush = False
    n._dsm = MagicMock()
    n._speaker_lock = threading.Lock()
    n._current_speaker = {"is_known": True, "name": "Саша"}
    n._speaker_by_text = {"поищи про чай": {"name": "Саша"}}
    n._speaker_tracker = MagicMock()
    n._maybe_record_session_end = MagicMock()
    n._publish_state = MagicMock()
    n._publish_response = MagicMock()
    return n, logger


ACT2_TURNS = [
    ("user", "[Spkr:Саша] поищи у себя в памяти что я говорил про чай"),
    ("assistant", "Зелёный чай без сахара и лук ни в каком виде."),
]


def test_token_clears_llm_window_of_previous_act(loop):
    core = _Core(ACT2_TURNS)
    n, logger = _make_node(loop, core)

    result = n.parameters_callback([_Param("e2e_session_reset_token", "e2e-20260923T100000Z-1")])

    assert result.successful is True
    assert core.turns == [], "ходы акта 2 остались в окне LLM — акт 2b их увидит (issue #2890)"
    assert n._current_speaker == {"is_known": False}
    assert n._speaker_by_text == {}
    n._dsm.reset.assert_called()
    n._speaker_tracker.reset.assert_called()


def test_e2e_reset_is_silent(loop):
    """Подтверждение «Начинаю новую сессию…» попало бы в запись первого шага."""
    core = _Core(ACT2_TURNS)
    n, _ = _make_node(loop, core)

    n.parameters_callback([_Param("e2e_session_reset_token", "e2e-x")])

    assert core.turns == [], "сброса не было — проверять тишину бессмысленно"
    n._publish_response.assert_not_called()
    published = [
        c.args[0].data for c in n._tts_control_pub.publish.call_args_list if c.args
    ]
    assert not any("IGNORE_STOP_MS" in str(p) for p in published)


def test_explicit_reset_line_in_robot_log(loop):
    n, logger = _make_node(loop, _Core(ACT2_TURNS))

    n.parameters_callback([_Param("e2e_session_reset_token", "e2e-tok-7")])

    lines = [str(c.args[0]) for c in logger.warning.call_args_list if c.args]
    assert any(
        "e2e-сброс сессии перед актом" in line and "e2e-tok-7" in line for line in lines
    ), lines


def test_failed_clear_rejects_token(loop):
    """Окно не очистилось — значение отклонено, харнесс увидит несовпадение."""
    core = _Core(ACT2_TURNS, fail=True)
    n, logger = _make_node(loop, core)

    result = n.parameters_callback([_Param("e2e_session_reset_token", "e2e-y")])

    assert result.successful is False
    assert core.turns, "фикстура: окно должно остаться грязным"
    assert logger.error.called


def test_no_asyncio_loop_rejects_token():
    """Без цикла окно очистить нечем — это отказ, а не молчаливый успех."""
    n, _ = _make_node(None, _Core(ACT2_TURNS))

    result = n.parameters_callback([_Param("e2e_session_reset_token", "e2e-z")])

    assert result.successful is False


def test_empty_token_is_noop(loop):
    """Дефолт параметра при старте ноды — не сброс."""
    core = _Core(ACT2_TURNS)
    n, _ = _make_node(loop, core)

    result = n.parameters_callback([_Param("e2e_session_reset_token", "")])

    assert result.successful is True
    assert core.turns == [t for t in ACT2_TURNS]
    n._cancel_run.assert_not_called()


def test_voice_new_session_still_announces(loop):
    """Регресс: голосовая «новая сессия» по-прежнему говорит подтверждение."""
    n, _ = _make_node(loop, _Core(ACT2_TURNS))

    n._reset_dialogue_session()

    n._publish_response.assert_called_once()
    published = [
        c.args[0].data for c in n._tts_control_pub.publish.call_args_list if c.args
    ]
    assert any("IGNORE_STOP_MS" in str(p) for p in published)
