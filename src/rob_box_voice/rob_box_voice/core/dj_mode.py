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
        on_stop: Optional farewell hook — shell can speak a goodbye
            phrase via ``speak_text`` when DJ-mode goes off (issue #1101).
    """

    dispatch: Callable[..., Any]  # Issue #992: signature is (prompt, from_tick=False)
    is_active: Callable[[], bool]
    is_dialogue_active: Callable[[], bool]
    persona_default: str = "ДиДжей РОббокс"
    on_stop: Optional[Callable[[str], None]] = None  # (persona) -> None


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
    # 🔴 FIX (live 15:20 06.08): 8 переходов ≈ 6 минут сета — юзер слышал
    # «однотипное потом замолчал». Поднято до 24 (~18 мин при 45с).
    DJ_AUTO_MAX_TRANSITIONS: int = 24

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
        # 🔴 FIX (live 15:30 06.08): план сета из set_dj_mode(plan=...) —
        # DJ идёт по плану и завершается финальным объявлением, а не
        # молча по лимиту DJ_AUTO_MAX_TRANSITIONS.
        plan = data.get("plan")
        if plan and isinstance(plan, str) and plan.strip():
            new_plan = plan.strip()
            if new_plan != self.state.set_plan:
                is_rewrite = bool(self.state.set_plan)
                self.state.set_plan = new_plan
                if is_rewrite:
                    # Переписываем сет заново — счётчик с нуля.
                    self.state.transition_count = 0
                self._logger.info(
                    f"🎧 DJ plan: {len(new_plan.splitlines())} треков"
                    f"{' (rewrite)' if is_rewrite else ''}"
                )
        self._logger.info(f"🎧 DJ Mode ON — next transition in {delay:.0f}s")

    def _reset_state(self) -> None:
        # Capture persona before clearing state so the farewell hook can
        # address the user with the correct DJ name (issue #1101).
        farewell_persona = self.state.persona or self._persona_default
        farewell_theme = self.state.theme or ""
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
        if self._hook.on_stop is not None:
            try:
                self._hook.on_stop(farewell_persona)
            except Exception as exc:  # noqa: BLE001
                self._logger.warning(
                    f"⚠️ DJ on_stop hook failed: {type(exc).__name__}: {exc}"
                )

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
        """Prefix injected into user STT turns when DJ-mode is active.

        🔴 FIX (live 11:48): раньше preamble подмешивал ПОЛНЫЕ DJ-инструкции
        («вызови set_dj_mode(enabled=true...)») к КАЖДОЙ user-команде —
        LLM видела «[DJ-РЕЖИМ АКТИВЕН, переход #3...]» перед «расскажи
        анекдот» и продолжала диджеить вместо ответа юзеру. Теперь это
        НЕЙТРАЛЬНАЯ подсказка: DJ играет в фоне, юзер говорит обычную
        команду — ответь на неё; не трогай DJ, если юзер не просит.
        Полные DJ-инструкции живут только в build_auto_prompt (DJ_AUTO).
        """
        persona = self.state.persona
        persona_line = (
            f", диджей: {persona}" if persona else ""
        )
        theme_line = (
            f', тема: "{self.state.theme}"' if self.state.theme else ""
        )
        return (
            f"[🎧 Музыкальный режим активен — фоновая музыка играет{theme_line}"
            f"{persona_line}. Это ОБЫЧНАЯ команда юзера, не DJ-переход. "
            "Ответь на неё нормально. Не вызывай set_dj_mode и не меняй "
            "музыку, если юзер об этом не просит.] "
        )

    def build_auto_prompt(self, n: int) -> str:
        persona = self.state.persona or self._persona_default
        theme_line = (
            f'Тема вечеринки: "{self.state.theme}". '
            if self.state.theme
            else ""
        )
        # Тех-ограничения (live 20.08): какофония из-за chop/драйва/
        # выдуманных сэмплов/лишних слоёв на 16kHz DAC без лимитера.
        tech_line = (
            "⚙️ ТЕХНИКА МУЗЫКИ (жёстко, без исключений): "
            "1) ПЕРВАЯ строка кода — Clock.clear() (иначе старые паттерны "
            "копятся поверх новых → каша из 50+ голосов). "
            "2) Максимум 6 паттернов: d1-d3 + p1-p3. НИКОГДА d4/d5/p4/p5. "
            "3) Барабаны amp≤0.2, синты amp≤0.5, СУММА amp всех слоёв ≤0.8. "
            "4) dur≥0.5, BPM≥60. "
            "5) НЕ используй chop= — на 16kHz даёт щелчки. "
            "6) НЕ выдумывай буквы сэмплов — сначала search_samples(<слово>) "
            "и бери ТОЧНО возвращённую букву. "
            "7) НЕ повторяй синт/гамму предыдущего трека."
        )
        stage_line = (
            f"Стадия сета: переход #{n}. "
            "Трек должен РАЗВИВАТЬСЯ внутри перехода "
            "(не повторять один и тот же рисунок)."
        )
        library_line = (
            "🎵 РЕФЕРЕНСЫ: 1) list_tracks(tag=<жанр>, min_rating=4) — найди "
            "готовый трек; 2) load_track(name=...) ВЕРНЁТ его код, но НЕ "
            "запускай его как есть — в библиотеке есть chop и лишние плееры "
            "(s1/m1/h1/c1/fx) — ПЕРЕПИШИ под тех-ограничения выше через "
            "execute_music_code; 3) НЕ загружай тот же трек, что в прошлом "
            "переходе — бери другой; 4) search_samples(<стиль>) — найди "
            "реальные сэмплы. НЕ упрощай до статичного лупа."
        )
        if n == 1:
            return (
                "[DJ_AUTO — СТАРТ ВЕЧЕРИНКИ] "
                f"Ты {persona} — первый в мире робот-диджей. {theme_line}"
                "🔎 СНАЧАЛА ИССЛЕДУЙ МАТЕРИАЛ: "
                "1) search_web(<персона> — стиль, темп, характерные приёмы) — "
                "изучи персону и её музыку; 2) search_samples(<стиль>) — найди "
                "реальные сэмплы (макс. 2 вызова); 3) list_tracks / "
                "gen_search_library(<персона>) — найди готовые композиции. "
                "📋 ЗАТЕМ СОСТАВЬ ПЛАН СЕТА из 5-8 треков (дуга: вход → "
                "нарастание → пик → спуск) и сохрани через "
                "set_dj_mode(enabled=true, plan=<список треков, каждый с новой "
                "строки 'Трек N: ...'>, next_transition_sec=45). Потом сыграй "
                f"трек #1 через execute_music_code. {library_line} {stage_line} "
                f"Затем представься как {persona} через speak_text."
            )
        plan_block = (
            f"План сета:\n{self.state.set_plan}\n" if self.state.set_plan else ""
        )
        plan_tracks = self.state.set_plan.count("Трек ") if self.state.set_plan else 0
        if plan_tracks and n >= plan_tracks:
            return (
                f"[DJ_AUTO переход #{n} — ФИНАЛЬНЫЙ ТРЕК] "
                f"Ты {persona}. {theme_line}{plan_block}"
                f"{library_line} {tech_line} "
                "Это ПОСЛЕДНИЙ трек сета. Сыграй завершающий трек через "
                "execute_music_code (спокойный финал, затухание). Затем "
                "ОБЯЗАТЕЛЬНО: 1) speak_text: «Вот и всё, вечеринка "
                "заканчивается! Спасибо, что были со мной!» (или в тему "
                "сета); 2) set_dj_mode(enabled=false) — DJ-режим завершается."
            )
        return (
            f"[DJ_AUTO переход #{n}] "
            f"Ты {persona}. {theme_line}{plan_block}"
            f"{library_line} {tech_line} {stage_line} "
            "Сыграй следующий трек через execute_music_code (segments 64-128, "
            "другой бит/темп в духе темы, С РАЗВИТИЕМ внутри трека — "
            "минимум один из: .every(), Pvar, linvar, Clock.future). "
            "После этого вызови set_dj_mode(enabled=true, next_transition_sec=45) "
            "для следующего перехода. Изредка произноси тематическую фразу "
            "через speak_text()."
        )


__all__ = ["DJModeController", "DJState", "DJHook"]
