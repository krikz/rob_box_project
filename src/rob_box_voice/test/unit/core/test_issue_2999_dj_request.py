"""Issue #2999 / ADR-0140 — «Ты диджей X…» должен включать/менять DJ-сет.

Два живых сценария (Vision Pi, LLM minimax):

* **A (24.09.2026 11:40–11:44 UTC)** — трек играет с прошлого хода, DJ
  выключен. «Ты диджей Снупдог и у нас сегодня вечеринка ганкста…» →
  ``tools=[]``; Bug C ретраил музыкальным промптом ×3 на каждый из 5
  запросов, сет не стартовал до «забудь все».
* **B (28.09.2026, 1790611527)** — DJ включён («диджей Дайв»). «Ты диджей
  Анакен скайвокер и у нас сегодня имперский слет в клубе» → обёртка
  «Не вызывай set_dj_mode», ``tools=[]`` → ``['lookup_melody']`` →
  разовый ``compose_music``; персона/тема сета не сменились.

Здесь — чистые части: детектор (таблица), промпт/фолбэк, вердикты
``MusicGuard.evaluate_turn``, утечка CRITICAL в TTS (``decide_turn_speech``),
обёртка ``DJModeController.preamble(dj_request=...)`` и смена персоны на
идущем сете. Адаптер ``DialogueNode`` — в
``test/unit/node/test_issue_2999_dj_request_node.py``.
"""

from __future__ import annotations

import json
import logging
from unittest.mock import MagicMock

import pytest

from rob_box_voice.core.dialogue_guards import MUSIC_STARTING_TOOLS
from rob_box_voice.core.dj_mode import DJHook, DJModeController
from rob_box_voice.core.dj_request import (
    DJ_REQUEST_MAX_RETRIES,
    build_dj_request_exhausted_fallback,
    build_dj_request_retry_prompt,
    extract_dj_request_hint,
    is_dj_request,
)
from rob_box_voice.core.music_guard import MusicGuard, MusicGuardVerdictKind
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


# ── 2. Промпт ретрая и фолбэк ──────────────────────────────────────────


def test_retry_prompt_names_dj_tools_with_hints() -> None:
    prompt = build_dj_request_retry_prompt(LIVE_A, dj_active=False)
    assert prompt.startswith("[CRITICAL]")
    assert "load_skill(skill='dj')" in prompt
    assert "set_dj_mode(enabled=true" in prompt
    assert "persona='диджей Снупдог'" in prompt
    assert "theme='вечеринка ганкста в чорном квартале'" in prompt
    # Не музыкальный промпт Bug C.
    assert "ни один музыкальный тул" not in prompt
    assert "НЕ DJ-сет" in prompt


def test_retry_prompt_active_set_keeps_bpm_and_music() -> None:
    prompt = build_dj_request_retry_prompt(LIVE_B, dj_active=True)
    assert "УЖЕ ИДЁТ" in prompt
    assert "bpm НЕ передавай" in prompt
    assert "stop_music НЕ вызывай" in prompt
    assert "persona='диджей Анакен скайвокер'" in prompt


def test_exhausted_fallback_is_honest() -> None:
    text = build_dj_request_exhausted_fallback(LIVE_A)
    assert "не запустился" in text
    low = text.lower()
    for bad in ("извин", "прости", "попробую", "[critical]"):
        assert bad not in low


# ── 3. MusicGuard.evaluate_turn ────────────────────────────────────────


def _turn(guard: MusicGuard, user_input: str, tools=(), **kw):
    return guard.evaluate_turn(
        was_dj_auto=kw.pop("was_dj_auto", False),
        user_input=user_input,
        tools_called=tuple(tools),
        dj_enabled=kw.pop("dj_enabled", False),
        build_music_retry_prompt=lambda u: "[CRITICAL] В прошлом цикле ты НЕ вызвал ни один музыкальный тул",
        build_dj_retry_prompt=lambda: "[CRITICAL] DJ",
        **kw,
    )


class TestScenarioA:
    """DJ выключен, трек играет, модель нарративит с ``tools=[]``."""

    def test_first_miss_is_dj_retry_not_bug_c(self) -> None:
        v = _turn(MusicGuard(), LIVE_A, ())
        assert v.kind is MusicGuardVerdictKind.USER_RETRY
        assert v.reason == "dj_request"
        assert "set_dj_mode" in v.prompt
        assert "ни один музыкальный тул" not in v.prompt

    def test_budget_then_dj_fallback_not_bug_c_loop(self) -> None:
        g = MusicGuard()
        kinds = [_turn(g, LIVE_A, ()).reason for _ in range(DJ_REQUEST_MAX_RETRIES)]
        assert kinds == ["dj_request"] * DJ_REQUEST_MAX_RETRIES
        last = _turn(g, LIVE_A, ())
        assert last.kind is MusicGuardVerdictKind.FALLBACK
        assert last.reason == "dj_request_retry_exhausted"
        assert "Диджей-сет" in last.prompt
        # Бюджет сброшен: следующий запрос юзера — снова DJ-ретрай.
        assert g.user_retry_count == 0

    def test_exhausted_with_music_started_is_skip(self) -> None:
        g = MusicGuard()
        for _ in range(DJ_REQUEST_MAX_RETRIES):
            _turn(g, LIVE_A, ())
        v = _turn(g, LIVE_A, ("compose_music",))
        assert v.kind is MusicGuardVerdictKind.SKIP
        assert v.reason == "dj_request_exhausted_music_playing"

    @pytest.mark.parametrize(
        "tools",
        [
            ("set_dj_mode", "compose_music"),
            ("load_skill", "set_dj_mode", "compose_music"),
        ],
    )
    def test_set_dj_mode_with_track_is_skip(self, tools) -> None:
        g = MusicGuard()
        _turn(g, LIVE_A, ())
        v = _turn(g, LIVE_A, tools)
        assert v.kind is MusicGuardVerdictKind.SKIP
        assert v.reason == "executed"
        assert g.user_retry_count == 0

    def test_set_dj_mode_without_track_is_bug_c_as_before(self) -> None:
        # DJ-часть закрыта; трек в том же ходе требует скилл dj, и
        # это по-прежнему работа Bug C (не DJ-ретрай).
        v = _turn(MusicGuard(), LIVE_A, ("load_skill", "set_dj_mode"))
        assert v.kind is MusicGuardVerdictKind.USER_RETRY
        assert v.reason == "bug_c"

    def test_retry_after_set_dj_mode_does_not_demand_it_again(self) -> None:
        g = MusicGuard()
        _turn(g, LIVE_A, ("set_dj_mode",))  # Bug C: трека нет
        v = _turn(g, LIVE_A, ("compose_music",))  # ретрай дозапустил трек
        assert v.kind is MusicGuardVerdictKind.SKIP
        assert v.reason == "executed"

    def test_closed_flag_resets_on_new_user_request(self) -> None:
        g = MusicGuard()
        _turn(g, LIVE_A, ("set_dj_mode", "compose_music"))
        g.reset_for_new_user_request()
        assert _turn(g, LIVE_A, ()).reason == "dj_request"
        g._dj_request_closed = True
        g.reset_for_new_session()
        assert _turn(g, LIVE_A, ()).reason == "dj_request"

    def test_load_skill_alone_does_not_satisfy(self) -> None:
        v = _turn(MusicGuard(), LIVE_A, ("load_skill",))
        assert v.kind is MusicGuardVerdictKind.USER_RETRY
        assert v.reason == "dj_request"

    def test_reset_for_new_user_request_gives_fresh_budget(self) -> None:
        g = MusicGuard()
        for _ in range(DJ_REQUEST_MAX_RETRIES):
            _turn(g, LIVE_A, ())
        g.reset_for_new_user_request()
        assert _turn(g, LIVE_A, ()).reason == "dj_request"


class TestScenarioB:
    """DJ включён, юзер назначает новую персону посреди сета."""

    @pytest.mark.parametrize("tools", [(), ("lookup_melody",), ("lookup_melody", "compose_music")])
    def test_one_off_track_is_not_enough(self, tools) -> None:
        v = _turn(MusicGuard(), LIVE_B, tools, dj_enabled=True)
        assert v.kind is MusicGuardVerdictKind.USER_RETRY
        assert v.reason == "dj_request"
        assert "УЖЕ ИДЁТ" in v.prompt

    def test_set_dj_mode_mid_set_satisfies(self) -> None:
        v = _turn(
            MusicGuard(), LIVE_B, ("set_dj_mode", "compose_music"),
            dj_enabled=True,
        )
        assert v.kind is MusicGuardVerdictKind.SKIP


class TestNotDjRequestFallsThrough:
    def test_dj_auto_transition_goes_to_bug_b(self) -> None:
        v = _turn(MusicGuard(), DJ_AUTO, (), was_dj_auto=True, dj_enabled=True)
        assert v.kind is MusicGuardVerdictKind.DJ_RETRY
        assert v.reason == "bug_b"

    def test_plain_music_request_still_bug_c(self) -> None:
        v = _turn(MusicGuard(), "включи музыку", ())
        assert v.kind is MusicGuardVerdictKind.USER_RETRY
        assert v.reason == "bug_c"

    def test_volume_request_is_not_dj_path(self) -> None:
        v = _turn(MusicGuard(), "диджей, сделай громче", (), dj_enabled=True)
        assert v.reason != "dj_request"

    def test_evaluate_itself_unchanged(self) -> None:
        # Прямой ``evaluate`` (без DJ-входа) по-прежнему даёт Bug C —
        # DJ-ветка живёт только в ``evaluate_turn``.
        v = MusicGuard().evaluate(was_dj_auto=False, user_input=LIVE_A, tools_called=())
        assert v.reason == "bug_c"


# ── 4. CRITICAL не доходит до TTS ──────────────────────────────────────


@pytest.mark.parametrize(
    "spoken",
    [
        build_dj_request_retry_prompt(LIVE_A),
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


def test_default_preamble_unchanged() -> None:
    dj = _dj()
    _dive_set(dj)
    text = dj.preamble()
    assert "Не вызывай set_dj_mode" in text
    assert text == dj.preamble(dj_request=False)


def test_persona_change_preamble_asks_for_set_dj_mode_keeping_bpm() -> None:
    dj = _dj()
    _dive_set(dj)
    text = dj.preamble(dj_request=is_dj_request(LIVE_B))
    assert "Не вызывай set_dj_mode" not in text
    assert "set_dj_mode(enabled=true" in text
    assert "bpm НЕ передавай" in text
    assert "128 BPM" in text
    assert "диджей Дайв" in text  # что сейчас играет — модель видит контекст


def test_set_dj_mode_on_active_set_switches_persona_theme_without_reset() -> None:
    on_stop = MagicMock()
    dj = _dj(on_stop)
    _dive_set(dj)
    dj.state.transition_count = 3
    dj.state.tracks_started = 2
    started = dj.state.started_at

    # Ровно то, что модель должна вызвать по новой обёртке (без bpm).
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
        ("set_dj_mode", "compose_music"),
        sorted(MUSIC_STARTING_TOOLS),
        is_dj_auto=False,
        turn_text=LIVE_B,
    )
    assert counted is True
    assert dj.state.tracks_started == before + 1
