"""#3780 B: вайб — шар. Событие сдвигает, без событий за время затухания шар у нейтрали."""

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.vibe import NEUTRAL, VibeBall


class Clock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


def test_decay_is_the_middle_of_shifu_window_and_lives_in_one_place():
    """05.11: без событий 5–15 минут, потом обычное лицо. Середина — 10 минут, одно число."""
    assert kn.VIBE_DECAY_S == 600.0
    assert 5 * 60 <= kn.VIBE_DECAY_S <= 15 * 60
    assert kn.VIBE_INERTIA_PER_S > 0 and kn.VIBE_DRIFT_PER_S > 0


def test_push_moves_the_ball_off_neutral():
    ball = VibeBall(Clock())
    landed = ball.push("shades")
    assert landed.style == "shades"
    assert landed.pull == 0.0
    assert landed.speed == pytest.approx(kn.VIBE_DRIFT_PER_S)
    assert not landed.neutral()


def test_without_new_events_the_ball_is_neutral_after_decay():
    clock = Clock()
    ball = VibeBall(clock)
    ball.push("shades")
    clock.t = kn.VIBE_DECAY_S
    landed = ball.at()
    assert landed.neutral()
    assert landed.style == NEUTRAL
    assert landed.pull == 0.0
    assert landed.speed == 0.0


def test_before_decay_the_ball_still_holds_the_style():
    clock = Clock()
    ball = VibeBall(clock)
    ball.push("shades", strength=1.0)
    clock.t = kn.VIBE_DECAY_S / 2
    held = ball.at()
    assert held.style == "shades"
    assert held.pull > 0.0
    assert not held.neutral()


def test_a_new_event_before_decay_keeps_the_ball_off_neutral():
    clock = Clock()
    ball = VibeBall(clock)
    ball.push("fire")
    clock.t = kn.VIBE_DECAY_S / 2
    ball.push("fire")
    clock.t += kn.VIBE_DECAY_S - 1.0
    assert not ball.at().neutral()
    clock.t += kn.VIBE_DECAY_S
    assert ball.at().neutral()


def test_neutral_push_sends_the_ball_back():
    clock = Clock()
    ball = VibeBall(clock)
    ball.push("shades")
    clock.t = 1.0
    ball.at()
    back = ball.push("neutral")
    assert back.style == "shades" and back.speed < 0.0
    clock.t = 1.0 + kn.VIBE_DECAY_S
    assert ball.at().neutral()


def test_zero_strength_does_not_move_a_resting_ball():
    ball = VibeBall(Clock())
    assert ball.push("shades", strength=0.0).neutral()
    assert ball.push("", strength=1.0).neutral()


def test_a_weak_push_still_moves_before_it_fades():
    clock = Clock()
    ball = VibeBall(clock)
    landed = ball.push("matrix", strength=kn.VIBE_HINT_STRENGTH)
    assert landed.style == "matrix" and landed.speed > 0.0
    clock.t = 1.0
    held = ball.at()
    assert held.style == "matrix" and held.pull > 0.0 and not held.neutral()
    clock.t = kn.VIBE_DECAY_S
    assert ball.at().neutral()


def test_hint_is_rare_and_does_not_push_without_a_book():
    clock = Clock()
    ball = VibeBall(clock)
    assert ball.hint("matrix") is None
    assert ball.at().neutral()
    book: dict = {}
    first = ball.hint("matrix", accepted=book)
    assert first is not None and first.style == "matrix"
    assert ball.hint("matrix", accepted=book) is None
    clock.t = kn.VIBE_HINT_MIN_S
    again = ball.hint("matrix", accepted=book)
    assert again is not None and again.style == "matrix"
