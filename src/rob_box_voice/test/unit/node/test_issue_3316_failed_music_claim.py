"""Issue #3316 — упал музыкальный тул, а реплика говорит «Трек запустился».

Живой прогон 01.10.2026 14:18Z::

    compose_music {'form': 'arc', ..., 'seed': 1}
    ❌ compose_music: Ручки seed действуют только с name= ...
    spoken='Трек запустился. Вот славянская вечеринка — поехали!'
    tools=['load_skill', 'compose_music', 'set_dj_mode']
    [issue 2966] ... NOT treating as success
    [issue 3266 Bug C] set_dj_mode в ходе прошёл ...
    TTS: 'Трек запустился. Во…'   → 2 минуты тишины

Решение — по структуре хода, без разбора фраз (ADR-0148).
"""

from __future__ import annotations

import logging

import pytest

from rob_box_voice.core.music_guard import (
    DJ_SET_TAKES_OVER,
    FAILED_MUSIC_HONEST_PHRASE,
    MusicGuard,
    MusicGuardVerdict,
    MusicGuardVerdictKind,
    music_launch_failed,
    withhold_text_of_failed_music_launch,
)
from rob_box_voice.core.speak_helpers import ensure_dj_music_response
from rob_box_voice.core.turn_speech import TurnSpeechHold

LIVE_INPUT = "Робот включи диджей сет на тему славянская вечеринка"
LIVE_SPOKEN = "Трек запустился. Вот славянская вечеринка — поехали!"
LIVE_TOOLS = ("load_skill", "compose_music", "set_dj_mode")
LIVE_OK = ("load_skill", "set_dj_mode")
TAKES_OVER = MusicGuardVerdict(
    kind=MusicGuardVerdictKind.SKIP_NOT_APPLICABLE, reason=DJ_SET_TAKES_OVER
)


def _hold(text=LIVE_SPOKEN) -> TurnSpeechHold:
    hold = TurnSpeechHold()
    if text is not None:
        hold.hold(text, LIVE_INPUT)
    return hold


def _withhold(hold, verdict=TAKES_OVER, **over) -> bool:
    kw = dict(
        tools_called=LIVE_TOOLS, succeeded_tools=LIVE_OK, tool_error_occurred=True,
    )
    kw.update(over)
    return withhold_text_of_failed_music_launch(hold, verdict, **kw)


def test_live_turn_text_is_replaced_with_honest_phrase() -> None:
    hold = _hold()
    assert _withhold(hold) is True
    assert hold.text == FAILED_MUSIC_HONEST_PHRASE
    assert hold.user_input == LIVE_INPUT


def test_through_real_guard_live_turn_takes_over_and_is_withheld() -> None:
    guard = MusicGuard(logger=logging.getLogger("t3316"))
    verdict = guard.evaluate(
        was_dj_auto=False, user_input=LIVE_INPUT, tools_called=LIVE_TOOLS,
        dj_enabled=True, spoken=LIVE_SPOKEN, tool_error_occurred=True,
        succeeded_tools=LIVE_OK, music_playing=False,
    )
    assert verdict.reason == DJ_SET_TAKES_OVER
    hold = _hold()
    assert _withhold(hold, verdict) is True
    assert hold.text == FAILED_MUSIC_HONEST_PHRASE


@pytest.mark.parametrize(
    "over",
    [
        {"tool_error_occurred": False},
        {"succeeded_tools": None},
        {"succeeded_tools": ("compose_music", "set_dj_mode")},
        {"tools_called": ("set_dj_mode",)},
    ],
    ids=["no_error", "unknown_success", "music_tool_ok", "no_music_tool"],
)
def test_text_kept_when_launch_did_not_fail(over) -> None:
    hold = _hold()
    assert _withhold(hold, **over) is False
    assert hold.text == LIVE_SPOKEN


def test_other_verdicts_and_empty_hold_untouched() -> None:
    retry = MusicGuardVerdict(kind=MusicGuardVerdictKind.USER_RETRY, reason="bug_c")
    hold = _hold()
    assert _withhold(hold, retry) is False and hold.text == LIVE_SPOKEN
    assert _withhold(_hold(None)) is False
    assert _withhold(None) is False


def test_music_launch_failed_helper() -> None:
    assert music_launch_failed(LIVE_TOOLS, LIVE_OK, True)
    assert not music_launch_failed(LIVE_TOOLS, ("compose_music",), True)
    assert not music_launch_failed(LIVE_TOOLS, None, True)
    assert not music_launch_failed(LIVE_TOOLS, LIVE_OK, False)


# --- Путь C: DJ-fallback не выдумывает подтверждение упавшего тула ---------


def test_dj_fallback_silent_when_music_tool_failed() -> None:
    tools = ["compose_music", "set_dj_mode"]
    # Контроль: без отказа фраза прежняя.
    assert ensure_dj_music_response("", tools) == "Готово, играю."
    assert ensure_dj_music_response(
        "", tools, is_dj_auto=True, track_name="Калинка"
    ) == "Дальше — Калинка!"
    # Тул упал: подтверждение не выдумываем (имя трека берётся из аргументов).
    assert ensure_dj_music_response("", tools, music_failed=True) == ""
    assert ensure_dj_music_response(
        "", tools, is_dj_auto=True, track_name="Калинка", music_failed=True
    ) == ""
