# Copyright 2024-2026 Rob Box Project
#
# Licensed under the MIT License. See LICENSE file for details.
# SPDX-License-Identifier: MIT
"""Мигание лица между чанками TTS: idle на 25–79 мс и обратно в talking.

Из живого лога робота (2026-08-29, рэп про металлолом, 7 куплетов).
``tts_node`` шлёт ``ready`` после каждого чанка и ``synthesizing`` перед
следующим, а между ними — десятки миллисекунд:

    idle в 545.887, обратно talking через 79 мс
    idle в 551.970, обратно talking через 55 мс
    idle в 558.964, обратно talking через 25 мс
    idle в 565.911, обратно talking через 50 мс

Каждое такое переключение — полная перезагрузка манифеста
(``Loaded animation: idle (5 panels)`` → ``Loaded animation: talking``),
то есть видимый глазом дёрг лица на каждой границе куплетов.

Речь при этом НЕ прерывалась: это один непрерывный монолог, разбитый на
чанки. Значит короткий провал в «замолчал» — артефакт нарезки, а не
событие, которое стоит показывать.

Лечится дебаунсом: уход в idle откладывается, и приход ``synthesizing``
успевает его отменить. Тесты дёргают методы ноды на стабе — сама нода
тянет rclpy, поднимать её незачем.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest


_NODE_SRC = (
    Path(__file__).resolve().parents[1] / "scripts" / "animation_player_node.py"
)


def _load_callback_ns():
    """Скомпилировать методы TTS-переключения без импорта модуля целиком."""
    tree = ast.parse(_NODE_SRC.read_text(encoding="utf-8"))
    cls = next(
        n
        for n in tree.body
        if isinstance(n, ast.ClassDef) and n.name == "AnimationPlayerNode"
    )
    wanted = {
        "tts_state_callback",
        "_cancel_idle_debounce",
        "_schedule_idle_debounce",
        "_idle_debounce_fired",
    }
    picked = [
        m
        for m in cls.body
        if isinstance(m, ast.FunctionDef) and m.name in wanted
    ]
    missing = wanted - {m.name for m in picked}
    assert not missing, f"в ноде нет методов дебаунса: {sorted(missing)}"
    mod = ast.Module(body=picked, type_ignores=[])
    ast.fix_missing_locations(mod)
    ns: dict = {}
    exec(compile(mod, "<animation_player_node>", "exec"), ns)
    return ns


class _Msg:
    def __init__(self, data: str) -> None:
        self.data = data


class _Player:
    def __init__(self) -> None:
        self.played: list[str] = []

    def play_animation(self, name: str) -> bool:
        self.played.append(name)
        return True


class _Timer:
    def __init__(self) -> None:
        self.cancelled = False

    def cancel(self) -> None:
        self.cancelled = True


class _Stub:
    """Минимальная нода: только то, чего касаются проверяемые методы."""

    def __init__(self, ns: dict) -> None:
        self._ns = ns
        self.player = _Player()
        self.idle_animation = "idle"
        self.talking_animation = "talking"
        self.is_robot_speaking = False
        self.manual_animation_active = False
        self.idle_debounce_s = 0.6
        self._idle_debounce_timer = None
        self.timers: list[_Timer] = []

    # ── то, что нода зовёт у самой себя ────────────────────────────
    def get_logger(self):
        class _L:
            def __getattr__(self, _):
                return lambda *a, **k: None

        return _L()

    def create_timer(self, period, callback):
        timer = _Timer()
        timer.period = period
        timer.callback = callback
        self.timers.append(timer)
        return timer

    # ── проверяемые методы, привязанные к стабу ────────────────────
    def __getattr__(self, name):
        fn = self.__dict__["_ns"].get(name)
        if fn is None:
            raise AttributeError(name)
        return fn.__get__(self, type(self))


@pytest.fixture
def stub():
    return _Stub(_load_callback_ns())


def test_short_gap_between_chunks_does_not_flap_to_idle(stub) -> None:
    """``ready`` → 50 мс → ``synthesizing`` не должен показывать idle."""
    stub.tts_state_callback(_Msg("synthesizing"))
    assert stub.player.played == ["talking.yaml"]

    # Конец чанка: уход в idle только запланирован, не выполнен.
    stub.tts_state_callback(_Msg("ready"))
    assert stub.player.played == ["talking.yaml"], (
        "лицо ушло в idle сразу — это и есть мигание между куплетами"
    )
    assert stub.timers, "дебаунс-таймер не заведён"

    # Следующий чанк пришёл раньше срабатывания таймера.
    stub.tts_state_callback(_Msg("synthesizing"))
    assert stub.timers[-1].cancelled, "таймер не отменён приходом речи"
    assert stub.player.played == ["talking.yaml"], (
        "перезагрузили talking повторно — тот же дёрг"
    )


def test_real_end_of_speech_still_reaches_idle(stub) -> None:
    """Если речь действительно кончилась — таймер доводит до idle."""
    stub.tts_state_callback(_Msg("playing"))
    stub.tts_state_callback(_Msg("ready"))
    assert stub.player.played == ["talking.yaml"]

    stub._idle_debounce_fired()
    assert stub.player.played == ["talking.yaml", "idle.yaml"]


def test_debounce_does_not_fire_if_speech_resumed(stub) -> None:
    """Сработавший таймер не гасит лицо, если робот снова говорит."""
    stub.tts_state_callback(_Msg("playing"))
    stub.tts_state_callback(_Msg("ready"))
    stub.tts_state_callback(_Msg("playing"))

    stub._idle_debounce_fired()
    assert "idle.yaml" not in stub.player.played
