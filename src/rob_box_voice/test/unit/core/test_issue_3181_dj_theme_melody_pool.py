"""Issue #3181 — тема сета не теряется и подбирает пул мелодий RTTTL-архива.

Живой лог 29.09 (StartedAt 2026-09-29T07:57:48Z): «Ты диджей 8 битный
монстр и у нас сегодня клубная вечеринка любителей денди» →
``set_dj_mode(enabled=true, next_transition_sec=45)`` БЕЗ ``theme`` →
каждый DJ-переход исполнял готовый ``_club_call`` без мелодии — сет
звучал однотипно, хотя в архиве лежат узнаваемые NES-темы (тег ``game``).

Два сценария:

1. ``set_dj_mode`` пришёл без ``theme`` на генуинном старте — тема не
   должна теряться: контроллер сохраняет реплику юзера как пришла в STT
   (``raw_utterance``), а не выдумывает её заново.
2. Тема сопоставляется тегу RTTTL-архива по общему правилу (не хаку под
   «денди») — переходы сета подставляют ``name=<id>`` следующей ещё не
   сыгранной мелодии пула; без темы/пула — вызов побайтно как раньше.
"""

from __future__ import annotations

import json
import logging
import re

import pytest

from rob_box_voice.core.dj_mode import DJHook, DJModeController
from rob_box_voice.core.dj_theme_melodies import (
    MELODY_POOLS,
    pick_melody,
    theme_to_tag,
)

START = 1_759_000_000.0
LIVE_TEXT = (
    "Ты диджей 8 битный монстр и у нас сегодня клубная вечеринка "
    "любителей денди"
)


def _controller(now=START):
    clock = {"t": now}
    dispatched = []
    hook = DJHook(
        dispatch=lambda prompt, from_tick=False: dispatched.append(prompt),
        is_active=lambda: False,
        is_dialogue_active=lambda: False,
    )
    ctrl = DJModeController(
        hook=hook, logger=logging.getLogger("test"), clock=lambda: clock["t"]
    )
    return ctrl, clock, dispatched


_CLUB_CALL_RE = re.compile(
    r'compose_music\(style="club", (?:name="(?P<name>[^"]+)", )?'
    r'bpm=(?P<bpm>\d+), root="(?P<root>[A-G]#?)", scale="minor", '
    r'seed=(?P<seed>\d+), repeat=(?P<repeat>true|false), '
    r'transition="fade"\)'
)


# ── 1. Тема не теряется без явного theme= ───────────────────────────────


def test_theme_fallback_from_raw_stt_on_fresh_start():
    """``set_dj_mode`` без ``theme`` на старте — тема берётся из STT."""
    ctrl, _, _ = _controller()
    ctrl.handle_message(
        json.dumps({"enabled": True, "next_transition_sec": 45}),
        raw_utterance=LIVE_TEXT,
    )
    assert ctrl.state.theme == LIVE_TEXT


def test_explicit_theme_wins_over_stt_fallback():
    """``theme=`` в payload — источник правды, фолбэк не перезаписывает его."""
    ctrl, _, _ = _controller()
    ctrl.handle_message(
        json.dumps({"enabled": True, "theme": "панк-вечеринка"}),
        raw_utterance=LIVE_TEXT,
    )
    assert ctrl.state.theme == "панк-вечеринка"


def test_stt_fallback_does_not_fire_mid_set():
    """Фолбэк — только на генуинном старте, не на каждом переходе без theme."""
    ctrl, _, _ = _controller()
    ctrl.handle_message(
        json.dumps({"enabled": True, "theme": "панк-вечеринка"}),
        raw_utterance=LIVE_TEXT,
    )
    # Переход посреди сета — модель повторяет set_dj_mode без theme (как в
    # build_auto_prompt), а STT для этого хода — авто-промпт, не юзер.
    ctrl.handle_message(
        json.dumps({"enabled": True, "next_transition_sec": 45}),
        raw_utterance="[DJ_AUTO переход #2] ...",
    )
    assert ctrl.state.theme == "панк-вечеринка"


def test_no_fallback_without_raw_utterance():
    """Нет ``raw_utterance`` (нода не передала) — тема просто пустая, как раньше."""
    ctrl, _, _ = _controller()
    ctrl.handle_message(json.dumps({"enabled": True, "next_transition_sec": 45}))
    assert ctrl.state.theme == ""


# ── 2. Тема → тег архива (общее правило, без хака под «денди») ─────────


@pytest.mark.parametrize(
    "theme,expected_tag",
    [
        ("клубная вечеринка любителей денди", "game"),
        ("8 битный монстр", "game"),
        ("dendy party", "game"),
        ("вечеринка в стиле nes", "game"),
        ("вечеринка нинтендо", "game"),
        ("сегодня сега", "game"),
        ("достали старую приставку", "game"),
        ("игровая тусовка", "game"),
        ("вечеринка в стиле кино", "movie"),
        ("тема — фильмы Marvel", "movie"),
        ("movie night", "movie"),
        ("новогодняя вечеринка", "christmas"),
        ("рождественский вечер", "christmas"),
        ("christmas party", "christmas"),
        ("классическая музыка", "classical"),
        ("classical evening", "classical"),
        ("просто клубная вечеринка", ""),
        ("панк-вечеринка", ""),
        ("", ""),
    ],
)
def test_theme_to_tag_rules(theme, expected_tag):
    assert theme_to_tag(theme) == expected_tag


def test_melody_pools_nonempty_for_supported_tags():
    for tag in ("game", "movie", "christmas", "classical"):
        assert MELODY_POOLS[tag], f"пул тега {tag} пуст"


def test_pick_melody_deterministic_and_skips_played():
    pool = ("a", "b", "c")
    assert pick_melody(pool, 1, []) == "a"
    assert pick_melody(pool, 2, []) == "b"
    assert pick_melody(pool, 4, []) == "a"  # цикл: (4-1) % 3 == 0
    # "a" уже сыграна (без учёта регистра) — обход пропускает её.
    assert pick_melody(pool, 1, ["A"]) == "b"
    assert pick_melody(pool, 1, ["a", "b", "c"]) == "a"  # пул исчерпан — повтор


def test_pick_melody_empty_pool():
    assert pick_melody((), 1, []) == ""


# ── 3. Переходы сета подставляют мелодию пула ───────────────────────────


def _club_calls(prompt):
    return [m.groupdict() for m in _CLUB_CALL_RE.finditer(prompt)]


def test_dendy_set_transitions_use_game_pool_melodies():
    """Живой сценарий issue #3181: денди-тема → 3 разных game-мелодии подряд."""
    ctrl, clock, dispatched = _controller()
    ctrl.handle_message(
        json.dumps({"enabled": True, "next_transition_sec": 45}),
        raw_utterance=LIVE_TEXT,
    )
    assert ctrl.state.theme == LIVE_TEXT
    assert ctrl.state.melody_tag == "game"
    assert ctrl.state.melody_pool == MELODY_POOLS["game"]

    names = []
    for track in range(1, 4):
        clock["t"] += 60
        ctrl.tick()
        ctrl.state.tracks_started = track
        calls = _club_calls(dispatched[-1])
        assert len(calls) == 1
        name = calls[0]["name"]
        assert name, f"переход #{track}: name= не подставлен"
        assert name in MELODY_POOLS["game"]
        names.append(name)
        # Модель повторяет set_dj_mode на каждом переходе, без theme
        # (issue #3181 сценарий) — фолбэк не должен затирать уже
        # сохранённую тему выдумкой из авто-промпта.
        ctrl.handle_message(
            json.dumps({"enabled": True, "next_transition_sec": 45}),
            raw_utterance=dispatched[-1],
        )
    # Требование issue: ≥3 перехода с разными мелодиями.
    assert len(set(names)) == len(names), f"мелодии повторились: {names}"
    assert names == list(MELODY_POOLS["game"][:3])
    assert ctrl.state.theme == LIVE_TEXT


def test_no_theme_match_leaves_club_call_byte_identical():
    """Тема без совпадения тега — вызов побайтно как до #3181 (нет name=)."""
    ctrl, clock, dispatched = _controller()
    ctrl.handle_message(json.dumps({"enabled": True, "theme": "панк-вечеринка"}))
    clock["t"] += 60
    ctrl.tick()
    prompt = dispatched[-1]
    assert 'compose_music(style="club", bpm=' in prompt
    assert 'name=' not in prompt


def test_no_theme_at_all_leaves_club_call_byte_identical():
    ctrl, clock, dispatched = _controller()
    ctrl.handle_message(json.dumps({"enabled": True}))
    clock["t"] += 60
    ctrl.tick()
    prompt = dispatched[-1]
    assert 'compose_music(style="club", bpm=' in prompt
    assert 'name=' not in prompt


def test_pr_evidence_dendy_transitions_2_3_4():
    """Для PR-описания: реальный вывод build_auto_prompt для переходов #2-#4."""
    ctrl, _, _ = _controller()
    ctrl.handle_message(
        json.dumps({"enabled": True, "next_transition_sec": 45}),
        raw_utterance=LIVE_TEXT,
    )
    ctrl.state.tracks_started = 1
    for n in (2, 3, 4):
        prompt = ctrl.build_auto_prompt(n)
        calls = _club_calls(prompt)
        assert calls and calls[0]["name"]
        ctrl.state.tracks_started = n
        print(f"--- переход #{n} ---\n{prompt}\n")
