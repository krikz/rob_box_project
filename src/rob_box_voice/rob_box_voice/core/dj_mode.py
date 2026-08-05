"""
dj_mode.py — Autonomous DJ mode state machine and prompt builder.

Extracted from the legacy ``dialogue_node.py`` so the ROS2 shell stays
≤350 LOC. Owns:

* Shell-local DJ state (``enabled``, ``theme``, ``persona``, ``set_plan``).
* The 5-second tick hook that fires an autonomous transition when
  ``next_transition_at`` has elapsed.
* Prompt builders for the first transition (``start of party``) and
  every subsequent track.

The actual LLM turn runs through :class:`DialogCore.process_input` —
DJ-mode just produces the prompt the shell hands to it. A
:class:`DJHook` carries the three shell-side callbacks the controller
needs (``dispatch``, ``is_active``, ``is_dialogue_active``).
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass
from typing import Any, Callable


# States where DJ-mode should defer its transition by 15 seconds.
_NON_IDLE_STATES = frozenset({"DIALOGUE", "SILENCED"})


@dataclass
class DJState:
    """Plain-Python state bag for the autonomous DJ state machine."""

    enabled: bool = False
    next_transition_at: float = 0.0
    transition_count: int = 0
    theme: str = ""
    set_plan: str = ""
    persona: str = ""


@dataclass
class DJHook:
    """Shell-side dependencies the controller needs to fire a transition.

    Attributes:
        dispatch: Coroutine launcher — wraps ``asyncio.run_coroutine_threadsafe``
            or equivalent for the shell's loop.
        is_active: Returns True when a turn is in-flight (skip transition).
        is_dialogue_active: Returns True when DSM is in DIALOGUE / SILENCED.
        persona_default: DJ persona fallback when none is set yet.
    """

    dispatch: Callable[..., Any]  # Issue #992: signature is (prompt, from_tick=False)
    is_active: Callable[[], bool]
    is_dialogue_active: Callable[[], bool]
    persona_default: str = "ДиДжей РОббокс"


class DJModeController:
    """High-level façade around :class:`DJState` for the shell's timer hooks."""

    FALLBACK_INTERVAL_S: float = 120.0
    POSTPONE_INTERVAL_S: float = 15.0
    DJ_TICK_INTERVAL_S: float = 5.0
    DJ_AUTO_STOP_THRESHOLD: int = 3
    # 🔴 FIX (live 11:19 DJ): save_dj_set_plan тула НЕТ → set_plan пуст →
    # авто-стоп по плану не срабатывал → DJ крутился бесконечно (#24+).
    # Жёсткий лимит переходов без плана (DJ_AUTO_MAX_TRANSITIONS):
    # после N переходов DJ сам выключается (юзер может включить снова).
    DJ_AUTO_MAX_TRANSITIONS: int = 8

    def __init__(self, *, hook: DJHook, logger: logging.Logger) -> None:
        self._hook = hook
        self._logger = logger
        self.state = DJState()
        self._persona_default = hook.persona_default

    # ── Message handlers ────────────────────────────────────────────

    def handle_message(self, payload: str) -> None:
        """Parse a JSON ``/voice/dj_mode`` message and update the state."""
        try:
            data = json.loads(payload)
            enabled = bool(data.get("enabled", False))
        except (json.JSONDecodeError, KeyError, TypeError):
            self._logger.warning(f"⚠️ DJ mode: bad message {payload!r}")
            return

        self.state.enabled = enabled
        if enabled:
            self._apply_enable_payload(data)
        else:
            self._reset_state()

    def _apply_enable_payload(self, data: dict) -> None:
        theme = data.get("theme")
        if theme and isinstance(theme, str) and theme.strip():
            if self.state.transition_count == 0 or not self.state.theme:
                self.state.theme = theme.strip()
                self._logger.info(f"🎧 DJ theme: {self.state.theme!r}")
        # 🔴 FIX (live 10:13 DJ): персона юзера — «ты диджей Пёс» →
        # сохраняем, чтобы автопромпты использовали её вместо дефолта.
        persona = data.get("persona")
        if persona and isinstance(persona, str) and persona.strip():
            self.state.persona = persona.strip()
            self._logger.info(f"🎧 DJ persona: {self.state.persona!r}")
        next_sec = data.get("next_transition_sec")
        delay = float(max(15, min(300, int(next_sec)))) if next_sec else 60.0
        self.state.next_transition_at = time.time() + delay
        self._logger.info(f"🎧 DJ Mode ON — next transition in {delay:.0f}s")

    def _reset_state(self) -> None:
        self.state.next_transition_at = 0.0
        self.state.transition_count = 0
        self.state.theme = ""
        self.state.set_plan = ""
        self.state.persona = ""
        # 🔴 FIX (live 11:46): без этого enabled оставался True после
        # авто-стопа → следующий tick (5с) видел next_transition_at=0.0 и
        # запускал НОВЫЙ DJ-цикл #1 — DJ «оживал» через 5 секунд после
        # остановки (бесконечность). enabled=False — единственный
        # надёжный выключатель: tick() сразу возвращается.
        self.state.enabled = False
        self._logger.info("🎧 DJ Mode OFF")

    # ── Tick ────────────────────────────────────────────────────────

    def tick(self) -> None:
        """Called from the shell's 5-second timer."""
        if not self.state.enabled:
            return
        now = time.time()
        if now < self.state.next_transition_at:
            return
        # Don't interrupt an active dialogue or sound playback.
        if self._hook.is_dialogue_active() or self._hook.is_active():
            self.state.next_transition_at = now + self.POSTPONE_INTERVAL_S
            return

        # Hard-stop when the plan is exhausted.
        plan_tracks = self.state.set_plan.count("Трек ")
        next_n = self.state.transition_count + 1
        # 🔴 FIX (live 11:19 DJ): save_dj_set_plan тула НЕТ — set_plan всегда
        # пуст → plan_tracks=0 → авто-стоп никогда не срабатывал → DJ
        # крутился бесконечно (#22+, час музыки, юзер не может выйти).
        # Фолбэк: если плана нет — жёсткий лимит переходов
        # (DJ_AUTO_MAX_TRANSITIONS), после которого DJ сам выключается.
        if plan_tracks == 0 and next_n > self.DJ_AUTO_MAX_TRANSITIONS:
            self._logger.warning(
                f"🛑 DJ auto-stop: переход #{next_n} превысил лимит "
                f"без плана ({self.DJ_AUTO_MAX_TRANSITIONS}) — "
                "save_dj_set_plan не вызывался, останавливаю DJ"
            )
            self._reset_state()
            return
        if plan_tracks > 0 and next_n > plan_tracks + self.DJ_AUTO_STOP_THRESHOLD:
            self._logger.warning(
                f"🛑 DJ auto-stop: transition #{next_n} beyond plan ({plan_tracks})"
            )
            self._reset_state()
            return

        self.state.transition_count = next_n
        self.state.next_transition_at = now + self.FALLBACK_INTERVAL_S
        self._logger.info(f"🎧 DJ auto-transition #{next_n}")
        # Issue #992 Bug B — ``from_tick=True`` lets the dispatcher
        # reset its synchronous-retry budget for this fresh transition.
        self._hook.dispatch(self.build_auto_prompt(next_n), True)

    # ── Prompt builders ─────────────────────────────────────────────

    def preamble(self) -> str:
        """Prefix injected into user STT turns when DJ-mode is active."""
        persona = self.state.persona or self._persona_default
        plan_line = (
            f"Текущий план сета:\n{self.state.set_plan}\n"
            if self.state.set_plan
            else ""
        )
        return (
            f"[🎧 DJ-РЕЖИМ АКТИВЕН, переход #{self.state.transition_count}. "
            f'Тема: "{self.state.theme}". '
            f'Твой DJ-образ: "{persona}". '
            f"{plan_line}"
            "Если DJ-режим должен что-то обновить — вызови "
            "set_dj_mode(enabled=true, next_transition_sec=45, theme='...') "
            "или просто ответь голосом.] "
        )

    def build_auto_prompt(self, n: int) -> str:
        persona = self.state.persona or self._persona_default
        theme_line = (
            f'Тема вечеринки: "{self.state.theme}". '
            if self.state.theme
            else ""
        )
        if n == 1:
            return (
                "[DJ_AUTO — СТАРТ ВЕЧЕРИНКИ] "
                f"Ты {persona} — первый в мире робот-диджей. {theme_line}"
                "Запусти музыку через execute_music_code (бит в духе темы, "
                "segments 64-128 — сет непрерывный). Затем представься как "
                f"{persona} через speak_text. Переходы делай через "
                "set_dj_mode(enabled=true, next_transition_sec=45)."
            )
        plan_block = (
            f"План сета:\n{self.state.set_plan}\n" if self.state.set_plan else ""
        )
        return (
            f"[DJ_AUTO переход #{n}] "
            f"Ты {persona}. {theme_line}{plan_block}"
            "Сыграй следующий трек через execute_music_code (segments 64-128, "
            "другой бит/темп в духе темы). После этого вызови "
            "set_dj_mode(enabled=true, next_transition_sec=45) для следующего "
            "перехода. Изредка произноси тематическую фразу через speak_text()."
        )


__all__ = ["DJModeController", "DJState", "DJHook"]
