"""test_issue_2875_dj_plan_start.py — сет по плану юзера стартует с его трека.

Живой прогон 23.09.2026 17:30: «Ты диджей Снупдог … играй по очереди:
Still Dre, Next Episode, Drop It Like It's Hot». Модель сохранила план
через ``set_dj_mode`` и не запустила трек; переход #1 по промпту «СТАРТ
ВЕЧЕРИНКИ» гнал её в ресёрч и новый план (вышла болтовня без музыки);
первый звук — на переходе #2, через 4 минуты. Номер трека плана брался из
номера перехода, поэтому провал #1 сдвинул план: финал по плану на #3,
Drop It не прозвучал бы.

Контракт после фикса:

* план есть → переход #1 играет «Трек 1» плана через
  ``compose_music(name=…)``, без ресёрча и без нового плана;
* номер трека плана = число РЕАЛЬНО запущенных треков + 1
  (``note_turn_tools``), провал перехода трек не съедает;
* финал (по плану и по лимиту ``max_tracks`` из #2856) — по тому же счётчику;
* ``dj.txt`` требует ``set_dj_mode(plan)`` + ``compose_music(name=Трек 1)``
  в одном ходе и запрещает «трека нет» без ``lookup_melody``.
"""
from __future__ import annotations

import json
import logging
from pathlib import Path

from rob_box_voice.core.dj_mode import DJModeController, plan_entry

T0 = 1_800_000_000.0
FORM_S = 180.0
FINAL_MARK = "ФИНАЛЬНЫЙ ТРЕК"
MUSIC = frozenset({"compose_music", "execute_music_code"})
PLAN = "Трек 1: Still Dre\nТрек 2: Next Episode\nТрек 3: Drop It Like It's Hot"

DJ_SKILL = (
    Path(__file__).resolve().parents[3] / "prompts" / "skills" / "dj.txt"
)


class _Clock:
    def __init__(self, now: float) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


class _Hook:
    def __init__(self, clock: _Clock) -> None:
        self.persona_default = "Роббокс"
        self.dispatches: list = []
        self.stops: list = []
        self._clock = clock
        self.on_stop = lambda persona: self.stops.append(clock.now)

    def dispatch(self, prompt: str, from_tick: bool = False) -> None:  # noqa: ARG002
        self.dispatches.append(prompt)

    def is_active(self) -> bool:
        return False

    def is_dialogue_active(self) -> bool:
        return False


def _controller(payload: dict | None = None):
    clock = _Clock(T0)
    hook = _Hook(clock)
    ctrl = DJModeController(hook=hook, logger=logging.getLogger("t"), clock=clock)
    if payload is not None:
        ctrl.handle_message(json.dumps({"enabled": True, **payload}))
    return ctrl, hook, clock


def _run(ctrl, hook, clock, *, failed=frozenset(), horizon_s=3 * 3600.0):
    """Тики до остановки DJ; переходы из ``failed`` музыку не запускают."""
    seen = 0
    while ctrl.state.enabled and clock.now < T0 + horizon_s:
        clock.now += DJModeController.DJ_TICK_INTERVAL_S
        ctrl.tick()
        if len(hook.dispatches) > seen:
            seen = len(hook.dispatches)
            if seen in failed:
                continue
            ctrl.note_turn_tools(
                ["speak_text", "compose_music"], MUSIC, is_dj_auto=True
            )
            ctrl.state.form_ends_at = clock.now + FORM_S


# ── Переход #1 при готовом плане ──────────────────────────────────────


def test_first_transition_with_plan_plays_track_one_without_research() -> None:
    ctrl, _, _ = _controller({"plan": PLAN, "persona": "диджей Снупдог"})

    prompt = ctrl.build_auto_prompt(1)

    assert 'compose_music(name="Still Dre")' in prompt
    assert "Трек 1" in prompt
    assert "search_web(" not in prompt
    assert "gen_search_library(" not in prompt
    assert "СОСТАВЬ ПЛАН" not in prompt
    assert "НЕ составляй новый план" in prompt


def test_first_transition_without_plan_keeps_research_and_planning() -> None:
    ctrl, _, _ = _controller({"theme": "панк"})

    prompt = ctrl.build_auto_prompt(1)

    assert "СНАЧАЛА ИССЛЕДУЙ МАТЕРИАЛ" in prompt
    assert "СОСТАВЬ ПЛАН СЕТА" in prompt
    assert "Сейчас по плану" not in prompt


def test_track_played_in_the_users_turn_moves_transition_one_to_track_two() -> None:
    """Ход юзера сам запустил Трек 1 → переход #1 играет Трек 2."""
    ctrl, _, _ = _controller({"plan": PLAN})
    assert ctrl.note_turn_tools(["set_dj_mode", "compose_music"], MUSIC)

    prompt = ctrl.build_auto_prompt(1)

    track_line = prompt.split("▶", 1)[1]
    assert 'compose_music(name="Next Episode")' in track_line
    assert "Still Dre" not in track_line
    assert FINAL_MARK not in prompt


def test_live_request_ten_minutes_three_tracks_plays_all_three() -> None:
    """Сценарий 23.09 после фикса: ход юзера играет Трек 1, переходы — 2 и 3."""
    ctrl, hook, clock = _controller({
        "plan": PLAN, "max_minutes": 10, "next_transition_sec": 50,
        "persona": "диджей Снупдог",
    })
    ctrl.note_turn_tools(["set_dj_mode", "compose_music"], MUSIC)
    ctrl.state.form_ends_at = clock.now + FORM_S

    _run(ctrl, hook, clock)

    assert len(hook.dispatches) == 2
    assert 'name="Next Episode"' in hook.dispatches[0]
    assert "Drop It Like It's Hot" in hook.dispatches[1]
    assert FINAL_MARK in hook.dispatches[1]
    assert hook.stops and hook.stops[0] - T0 <= 10 * 60.0


# ── Счётчик треков ────────────────────────────────────────────────────


def test_note_turn_tools_counts_only_music_starts_while_enabled() -> None:
    ctrl, _, _ = _controller()
    assert not ctrl.note_turn_tools(["compose_music"], MUSIC), "DJ выключен"

    ctrl.handle_message(json.dumps({"enabled": True, "plan": PLAN}))
    assert not ctrl.note_turn_tools(["set_dj_mode", "speak_text"], MUSIC)
    assert not ctrl.note_turn_tools(None, MUSIC)
    assert not ctrl.note_turn_tools([], MUSIC)
    assert ctrl.state.tracks_started == 0

    assert ctrl.note_turn_tools(
        ["lookup_melody", "compose_music"], MUSIC, is_dj_auto=True
    )
    assert ctrl.state.tracks_started == 1


def test_fresh_start_and_stop_reset_the_track_counter() -> None:
    ctrl, _, _ = _controller({"plan": PLAN})
    ctrl.note_turn_tools(["compose_music"], MUSIC, is_dj_auto=True)
    ctrl.handle_message(json.dumps({"enabled": False}))
    assert ctrl.state.tracks_started == 0

    ctrl.handle_message(json.dumps({"enabled": True, "plan": PLAN}))
    assert ctrl.state.tracks_started == 0


def test_failed_transition_does_not_eat_a_plan_track() -> None:
    """Живой сценарий 23.09: переход #1 провален — все три трека звучат."""
    ctrl, hook, clock = _controller({"plan": PLAN, "next_transition_sec": 50})

    _run(ctrl, hook, clock, failed=frozenset({1}))

    assert not ctrl.state.enabled
    # #1 провал (Трек 1 снова на #2), #2 Трек 1, #3 Трек 2, #4 Трек 3 — финал.
    assert len(hook.dispatches) == 4
    assert 'name="Still Dre"' in hook.dispatches[0]
    assert 'name="Still Dre"' in hook.dispatches[1]
    assert 'name="Next Episode"' in hook.dispatches[2]
    assert "Drop It Like It's Hot" in hook.dispatches[3]
    assert FINAL_MARK in hook.dispatches[3]
    assert all(FINAL_MARK not in p for p in hook.dispatches[:3])
    assert len(hook.stops) == 1


def test_plan_set_stops_after_its_last_track_really_started() -> None:
    ctrl, hook, clock = _controller({"plan": PLAN, "next_transition_sec": 50})

    _run(ctrl, hook, clock)

    assert not ctrl.state.enabled
    assert len(hook.dispatches) == 3
    assert FINAL_MARK in hook.dispatches[2]
    assert len(hook.stops) == 1


def test_max_tracks_limit_counts_started_tracks_not_transitions() -> None:
    """#2856 лимит ``max_tracks`` по тому же счётчику: провал не считается."""
    ctrl, hook, clock = _controller({"max_tracks": 3, "next_transition_sec": 50})

    _run(ctrl, hook, clock, failed=frozenset({2}))

    assert not ctrl.state.enabled
    assert ctrl.state.tracks_started == 0  # сброшен остановкой
    assert len(hook.dispatches) == 4
    assert FINAL_MARK in hook.dispatches[-1]
    assert all(FINAL_MARK not in p for p in hook.dispatches[:-1])


def test_failed_final_transition_gets_another_attempt() -> None:
    """Финал по лимиту, где трек не запустился, не закрывает сет молча."""
    ctrl, hook, clock = _controller({"max_tracks": 2, "next_transition_sec": 50})

    _run(ctrl, hook, clock, failed=frozenset({2}))

    finals = [p for p in hook.dispatches if FINAL_MARK in p]
    assert len(finals) == 2, "провалившийся финал повторён ровно один раз"
    assert len(hook.dispatches) == 3
    assert len(hook.stops) == 1


# ── Разбор плана ──────────────────────────────────────────────────────


def test_plan_entry_parses_track_lines() -> None:
    assert plan_entry(PLAN, 1) == "Still Dre"
    assert plan_entry(PLAN, 3) == "Drop It Like It's Hot"
    assert plan_entry(PLAN, 4) == ""
    assert plan_entry("Трек 2 — Next Episode", 2) == "Next Episode"


def test_unparsable_plan_line_still_names_the_track_number() -> None:
    ctrl, _, _ = _controller({"plan": "Трек 1\nТрек 2\nТрек 3"})

    prompt = ctrl.build_auto_prompt(1)

    assert "Сейчас по плану — Трек 1" in prompt
    assert "compose_music" in prompt


# ── dj.txt ───────────────────────────────────────────────────────────


def test_dj_skill_demands_plan_and_first_track_in_the_same_turn() -> None:
    text = DJ_SKILL.read_text(encoding="utf-8")

    assert "в ОДНОМ ходе ДВА шага" in text
    assert "set_dj_mode(enabled=true, plan=" in text
    # Трек играет скилл composer: dj.txt не называет его инструменты
    # (test_skill_prompt_contract), а маршрутизирует через имя скилла.
    assert "СРАЗУ сыграй Трек 1" in text
    assert 'load_skill(skill="composer")' in text


def test_dj_skill_forbids_claiming_a_song_is_missing_without_lookup() -> None:
    text = " ".join(DJ_SKILL.read_text(encoding="utf-8").split())

    assert "библиотеки известных мелодий (RTTTL) скилла composer" in text
    assert "не проверив библиотеку мелодий composer" in text
