"""Issue #3249 — характер темы (``base_bpm``/``scale`` от LLM) доходит до club-треков.

Ночной прогон 30.09: 37 club-треков в 8 темах — все 120–128 BPM, 70 % минора.
Тема влияла только на мелодии, а темп/лад DJ-сета от неё не зависели. Словаря
«тема → BPM» нет: характер выводит LLM по правилам ``skills/dj.txt`` и отдаёт в
``set_dj_mode(base_bpm=..., scale=...)``; код его уважает, сохраняя дрейф и
родственные лады (#3226).
"""

from __future__ import annotations

import json
import logging
import re
from collections import Counter
from pathlib import Path

from rob_box_voice.core.dj_mode import DJ_SET_DEFAULT_BPM, DJHook, DJModeController
from rob_box_voice.core.dj_set_walk import (
    SET_BASE_BPM_RANGE,
    SET_SCALE_SHARE,
    SET_SCALES,
    track_key,
)

START = 1_759_000_000.0


def _controller(now=START):
    clock = {"t": now}
    dispatched = []
    hook = DJHook(
        dispatch=lambda prompt, from_tick=False: dispatched.append(prompt),
        is_active=lambda: False,
        is_dialogue_active=lambda: False,
    )
    ctrl = DJModeController(hook=hook, logger=logging.getLogger("test"), clock=lambda: clock["t"])
    return ctrl, clock, dispatched


def _send(ctrl, **payload):
    ctrl.handle_message(json.dumps({"enabled": True, **payload}))


def _club_calls(prompt):
    return re.findall(r'compose_music\(style="club", (?:theme="[^"]*", )?bpm=(\d+), root="([A-G]#?)", '
                      r'scale="(\w+)", seed=(\d+), repeat=(true|false), transition="fade"\)', prompt)


def _set_calls(ctrl, clock, dispatched, tracks):
    calls = []
    for track in range(1, tracks + 1):
        clock["t"] += 20
        ctrl.tick()
        ctrl.state.tracks_started = track
        calls.extend(_club_calls(dispatched[-1]))
    return calls


def test_base_bpm_moves_drift_center_without_locking():
    ctrl, clock, dispatched = _controller()
    _send(ctrl, theme="техно", base_bpm=100, next_transition_sec=15)
    assert ctrl.state.set_bpm == 100
    assert ctrl.state.bpm_locked is False
    bpms = [int(c[0]) for c in _set_calls(ctrl, clock, dispatched, 5)]
    assert bpms, dispatched[-1]
    assert all(96 <= b <= 104 for b in bpms), bpms
    assert "плавно дрейфует около 100 BPM" in dispatched[-1]


def test_scale_preference_starts_set_and_dominates():
    ctrl, clock, dispatched = _controller()
    _send(ctrl, theme="техно", scale="major", next_transition_sec=15)
    assert ctrl.state.set_scale == "major"
    scales = [c[2] for c in _set_calls(ctrl, clock, dispatched, 4)]
    assert scales and scales[0] == "major", scales


def test_echo_on_transition_does_not_move_character():
    ctrl, _, _ = _controller()
    _send(ctrl, theme="техно", base_bpm=132, scale="dorian")
    # Переход: модель повторяет set_dj_mode с той же темой и другим характером.
    _send(ctrl, theme="техно", base_bpm=100, scale="major")
    assert (ctrl.state.set_bpm, ctrl.state.set_scale) == (132, "dorian")


def test_new_theme_mid_set_takes_new_character():
    ctrl, _, _ = _controller()
    _send(ctrl, theme="техно", base_bpm=132, scale="phrygian")
    _send(ctrl, theme="весенний пикник", base_bpm=110, scale="major")
    assert (ctrl.state.set_bpm, ctrl.state.set_scale) == (110, "major")


def test_user_bpm_beats_character():
    ctrl, _, _ = _controller()
    _send(ctrl, theme="техно", base_bpm=100, bpm=140)
    assert ctrl.state.set_bpm == 140 and ctrl.state.bpm_locked


def test_garbage_is_rejected_and_logged(caplog):
    ctrl, _, _ = _controller()
    with caplog.at_level(logging.INFO):
        _send(ctrl, theme="техно", base_bpm="быстро", scale="lydian")
    assert ctrl.state.set_bpm == DJ_SET_DEFAULT_BPM
    assert ctrl.state.set_scale == ""
    assert "отклонён" in caplog.text


def test_base_bpm_is_clamped():
    ctrl, _, _ = _controller()
    _send(ctrl, theme="техно", base_bpm=40)
    assert ctrl.state.set_bpm == SET_BASE_BPM_RANGE[0]


def test_character_is_reset_with_set():
    ctrl, _, _ = _controller()
    _send(ctrl, theme="техно", base_bpm=100, scale="major")
    ctrl.handle_message(json.dumps({"enabled": False}))
    assert (ctrl.state.set_bpm, ctrl.state.set_scale) == (DJ_SET_DEFAULT_BPM, "")
    _send(ctrl, theme="техно")
    assert (ctrl.state.set_bpm, ctrl.state.set_scale) == (DJ_SET_DEFAULT_BPM, "")


def test_without_character_track_key_is_unchanged():
    for seed in range(40):
        for n in range(1, 21):
            assert track_key("E", n, seed) == track_key("E", n, seed, "")


def test_preferred_scale_share_and_variety_over_many_sets():
    for prefer in SET_SCALES:
        modes = Counter(track_key("E", n, seed, prefer)[1] for seed in range(40) for n in range(3, 21))
        share = modes[prefer] / sum(modes.values())
        assert abs(share - SET_SCALE_SHARE) < 0.08, (prefer, modes)
        assert len(modes) >= 3, (prefer, modes)  # родственные лады не пропали
        assert all(track_key("E", n, 7, prefer)[1] == prefer for n in (1, 2))


def test_skill_teaches_rules_not_theme_dictionary():
    skill = Path(__file__).resolve().parents[3] / "prompts" / "skills" / "dj.txt"
    text = skill.read_text(encoding="utf-8")
    assert "base_bpm" in text and 'scale="major"' in text
    for theme in ("космос", "пират", "киберпанк", "чайк"):
        assert theme not in text.lower()
