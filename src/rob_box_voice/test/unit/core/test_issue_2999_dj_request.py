"""Issue #2999 / ADR-0140 — «Ты диджей X…» должен включать/менять DJ-сет.

Два живых сценария (Vision Pi, LLM minimax):

* **A (24.09.2026 11:40–11:44 UTC)** — трек играет с прошлого хода, DJ
  выключен. «Ты диджей Снупдог и у нас сегодня вечеринка ганкста…» →
  ``tools=[]``; Bug C ретраил музыкальным промптом ×3 на каждый из 5
  запросов, сет не стартовал до «забудь все».
* **B (28.09.2026, 1790611527)** — DJ включён («диджей Дайв»). «Ты диджей
  Анакен скайвокер и у нас сегодня имперский слет в клубе» → обёртка
  «Не вызывай set_dj_mode», ``tools=[]`` → ``['lookup_melody']`` →
  разовый ``execute_music_code``; персона/тема сета не сменились.

Issue #3134: «ты диджей X» исполняет роутер медиакоманд кодом ДО LLM
(``set_dj_mode``), поэтому DJ-ретрай ``MusicGuard.evaluate_turn``, его
промпт/фолбэк и особая обёртка хода удалены. Здесь остались: детектор
(перенесён в ``core/media_command_grammar.py``), утечка CRITICAL в TTS и
смена персоны на идущем сете в ``DJModeController``. Роутер — в
``test_issue_3134_media_router.py``.
"""

from __future__ import annotations

import json
import logging
from unittest.mock import MagicMock

import pytest

from rob_box_voice.core.dialogue_guards import MUSIC_STARTING_TOOLS
from rob_box_voice.core.dj_mode import DJHook, DJModeController
from rob_box_voice.core.media_command_grammar import (
    extract_dj_request_hint,
    is_dj_request,
)
from rob_box_voice.core.turn_speech import decide_turn_speech, is_retry_prompt_leak

LIVE_A = "[TG] Ты диджей Снупдог и у нас сегодня вечеринка ганкста в чорном квартале"
LIVE_B = "[TG] Ты диджей Анакен скайвокер и у нас сегодня имперский слет в клубе"
LIVE_DIVE = "[TG] Ты диджей Дайв и у нас сегодня вечеринка в клубе"
DJ_AUTO = (
    '[DJ_AUTO переход #2] Ты диджей Дайв. Тема вечеринки: "клубная вечеринка". '
    "Сейчас по плану — Трек 2"
)


# ── 1. Детектор: таблица ────────────────────────────────────────────────


@pytest.mark.parametrize(
    "text",
    [
        LIVE_A,
        LIVE_B,
        LIVE_DIVE,
        "Ты диджей PAUL OAKENFOLD и у нас сегодня вечеринка",
        "стань диджеем",
        "будь диджеем Псом",
        "теперь ты диджей Кот",
        "ты сегодня диджей",
        "ты dj snoop",
        "Ты ДИДЖЕЙ Снуп",
        "запусти диджей-сет",
        "включи режим диджея",
        "давай dj сет",
        "врубай dj mode",
    ],
)
def test_is_dj_request_positive(text: str) -> None:
    assert is_dj_request(text) is True


@pytest.mark.parametrize(
    "text",
    [
        "",
        None,
        "диджей, сделай громче",
        "диджей Дайв, сделай погромче",
        "ты диджей, сделай громче",
        "ты диджей?",
        "стоп диджей",
        "хватит диджеить",
        "выключи диджея",
        "выключи режим диджея",
        "кто такой диджей?",
        "диджей играет в наушниках",
        "ты диджеишь круто",
        "сыграй трек",
        "включи музыку",
        "сделай громче",
        DJ_AUTO,
    ],
)
def test_is_dj_request_negative(text) -> None:
    assert is_dj_request(text) is False


@pytest.mark.parametrize(
    ("text", "persona", "theme"),
    [
        (LIVE_A, "диджей Снупдог", "вечеринка ганкста в чорном квартале"),
        (LIVE_B, "диджей Анакен скайвокер", "имперский слет в клубе"),
        (LIVE_DIVE, "диджей Дайв", "вечеринка в клубе"),
        ("давай dj сет", "", ""),
        ("стань диджеем", "", ""),
    ],
)
def test_extract_dj_request_hint(text: str, persona: str, theme: str) -> None:
    assert extract_dj_request_hint(text) == (persona, theme)


# ── 4. CRITICAL не доходит до TTS ──────────────────────────────────────


@pytest.mark.parametrize(
    "spoken",
    [
        "[CRITICAL] В прошлом цикле ты НЕ вызвал set_dj_mode, хотя юзер назначил тебя диджеем",
        "[Speaker:unknown] [CRITICAL] В прошлом цикле ты НЕ вызвал set_dj_mode",
        "Йо! [critical] вызови set_dj_mode",
    ],
)
def test_critical_leak_is_never_spoken(spoken: str) -> None:
    assert is_retry_prompt_leak(spoken)
    # Даже когда ретрая нет и тул вызван (гейт #1882 такой ответ пропускает).
    assert decide_turn_speech(spoken, n_chunks=1) is None


def test_normal_dj_reply_is_spoken() -> None:
    reply = "Йо, Анакен на связи, имперский слет начинается!"
    assert not is_retry_prompt_leak(reply)
    assert decide_turn_speech(reply, n_chunks=1) == reply


# ── 5. Обёртка хода и смена персоны на идущем сете ─────────────────────


def _dj(on_stop=None) -> DJModeController:
    clock = MagicMock(return_value=1_000_000.0)
    return DJModeController(
        hook=DJHook(
            dispatch=MagicMock(),
            is_active=lambda: False,
            is_dialogue_active=lambda: False,
            on_stop=on_stop,
        ),
        logger=logging.getLogger("test_2999"),
        clock=clock,
    )


def _dive_set(dj: DJModeController) -> None:
    dj.handle_message(json.dumps({
        "enabled": True, "persona": "диджей Дайв", "theme": "клубная вечеринка",
        "bpm": 128, "plan": "Трек 1: клуб\nТрек 2: клуб", "next_transition_sec": 45,
    }))


def test_preamble_is_single_neutral_wrapper() -> None:
    dj = _dj()
    _dive_set(dj)
    assert "Не вызывай set_dj_mode" in dj.preamble()


def test_set_dj_mode_on_active_set_switches_persona_theme_without_reset() -> None:
    on_stop = MagicMock()
    dj = _dj(on_stop)
    _dive_set(dj)
    dj.state.transition_count = 3
    dj.state.tracks_started = 2
    started = dj.state.started_at

    # Ровно то, что вызывает роутер медиакоманд (#3134) — без bpm.
    dj.handle_message(json.dumps({
        "enabled": True, "persona": "диджей Анакен скайвокер",
        "theme": "имперский слет в клубе",
    }))

    s = dj.state
    assert s.enabled is True
    assert s.persona == "диджей Анакен скайвокер"
    assert s.theme == "имперский слет в клубе"
    assert s.set_bpm == 128  # #3113 — темп сета сохранён
    assert s.started_at == started
    assert s.transition_count == 3 and s.tracks_started == 2
    assert s.set_plan == ""  # план старой темы сброшен
    on_stop.assert_not_called()  # не «конец вечеринки», музыка не глушится
    prompt = dj.build_auto_prompt(4)
    assert "Анакен" in prompt and "имперский слет" in prompt
    assert "Дайв" not in prompt and "клубная вечеринка" not in prompt


def test_persona_change_turn_counts_as_set_track() -> None:
    dj = _dj()
    _dive_set(dj)
    before = dj.state.tracks_started
    counted = dj.note_turn_tools(
        ("set_dj_mode", "execute_music_code"),
        sorted(MUSIC_STARTING_TOOLS),
        is_dj_auto=False,
        turn_text=LIVE_B,
    )
    assert counted is True
    assert dj.state.tracks_started == before + 1
