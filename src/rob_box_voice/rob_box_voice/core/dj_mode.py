"""
dj_mode.py — Autonomous DJ mode state machine and prompt builder.

Extracted from the legacy ``dialogue_node.py`` so the ROS2 shell stays
≤350 LOC. Owns:

* Shell-local DJ state (``enabled``, ``theme``, ``persona``, ``set_plan``).
* The 5-second tick hook that fires an autonomous transition when
  ``next_transition_at`` has elapsed.
* Prompt builders for the first transition (``start of party``) and
  every subsequent track.

The actual LLM turn runs through :class:`AgentCore.process_input` —
DJ-mode just produces the prompt the shell hands to it. A
:class:`DJHook` carries the three shell-side callbacks the controller
needs (``dispatch``, ``is_active``, ``is_dialogue_active``).
"""

from __future__ import annotations

import json
import logging
import re
import time
from dataclasses import dataclass
from typing import Any, Callable, Iterable, Optional


# States where DJ-mode should defer its transition by 15 seconds.
_NON_IDLE_STATES = frozenset({"DIALOGUE", "SILENCED"})

# Issue #2875 — строка плана «Трек N: <что играть>» (как её пишет
# set_dj_mode(plan=...) по dj.txt).
_PLAN_ENTRY_RE = re.compile(r"^\s*Трек\s*(\d+)\s*[:.)\-—–]\s*(.+?)\s*$")


def plan_entry(plan: str, track_no: int) -> str:
    """Текст строки ``Трек <track_no>: ...`` плана или ``""``."""
    for line in plan.splitlines():
        match = _PLAN_ENTRY_RE.match(line)
        if match and int(match.group(1)) == track_no:
            return match.group(2)
    return ""


@dataclass
class DJState:
    """Plain-Python state bag for the autonomous DJ state machine."""

    enabled: bool = False
    next_transition_at: float = 0.0
    transition_count: int = 0
    theme: str = ""
    set_plan: str = ""
    persona: str = ""
    # Issue #2461 — конец текущего прохода формы, АБСОЛЮТНОЕ стенное
    # время (``time.time()``-эпоха), приходит из ``/voice/music/form``
    # (mcp_server → dialogue_node → сюда через ``_on_music_form``).
    # mcp_server сам переводит свой ``form_cycle_remaining_s`` (посчитанный
    # через ``time.monotonic()`` внутри ``MusicManager``) в epoch ПЕРЕД
    # публикацией — ``time.monotonic()`` не сопоставим между процессами
    # (mcp_server и dialogue_node — РАЗНЫЕ ОС-процессы, см.
    # voice_assistant.launch.py), а ``time.time()`` для обоих процессов
    # общий (одна машина). Здесь поле хранится как получено, без
    # пересчёта. ``None`` — данных нет (топик ещё не пришёл, форма не
    # играет) — тогда ``tick()`` этим полем не гейтится вообще.
    form_ends_at: Optional[float] = None
    # Issue #2856 — лимит сета по ВРЕМЕНИ. ``started_at`` — стенное время
    # генуинного старта сета (``0.0`` — неизвестно, ``tick()`` взведёт при
    # первом вызове). ``max_seconds`` / ``max_tracks`` — явный лимит от
    # юзера через ``set_dj_mode(max_minutes=..., max_tracks=...)``; ``None``
    # / ``0`` — лимита нет (без плана действует ``DJ_SET_DEFAULT_MAX_S``).
    # ``final_dispatched`` — финальный трек ПО ЛИМИТУ уже отдан модели:
    # следующий переход — остановка, а не ещё один трек.
    started_at: float = 0.0
    last_transition_at: float = 0.0
    max_seconds: Optional[float] = None
    max_tracks: int = 0
    final_dispatched: bool = False
    # Issue #2875 — сколько треков РЕАЛЬНО запущено в этом сете (ход, где
    # был вызван тул из ``MUSIC_STARTING_TOOLS``; считает нода через
    # :meth:`DJModeController.note_turn_tools`). Номер трека плана и финал
    # (по плану и по лимиту ``max_tracks``) берутся отсюда, а не из
    # ``transition_count``: провалившийся переход (модель не вызвала
    # compose_music) больше не «съедает» трек плана. ``final_track_no`` —
    # номер трека, который объявлен финальным по лимиту (#2856).
    tracks_started: int = 0
    final_track_no: int = 0
    # Issue #2875 (дополнение) — отложенное прощание: стенное время, когда
    # его сказать (``None`` — не ждём), и персона, от чьего имени.
    farewell_at: Optional[float] = None
    farewell_persona: str = ""


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
    #
    # 🔴 FIX (issue #2856): с #2461 переход ждёт конца формы
    # (``form_ends_at``), реальный интервал 150-200 с → 24 перехода ≈
    # 60-80 минут (живой сет 23.09 14:51 → #17 в 15:39, конца не видно).
    # Счётчик переходов остаётся только страховкой; основной лимит сета
    # без плана — по ВРЕМЕНИ (``DJ_SET_DEFAULT_MAX_S``).
    DJ_AUTO_MAX_TRANSITIONS: int = 24
    # Issue #2856 — длительность сета без плана по умолчанию. Юзер может
    # задать свою через ``set_dj_mode(max_minutes=N)``.
    DJ_SET_DEFAULT_MAX_S: float = 20 * 60.0
    # Границы явных лимитов из ``set_dj_mode`` (защита от мусора модели).
    DJ_SET_MAX_MINUTES_RANGE: tuple = (1, 180)
    # Минимум 2: переход #1 («СТАРТ ВЕЧЕРИНКИ») финальным не бывает.
    DJ_SET_MAX_TRACKS_RANGE: tuple = (2, 50)
    # Issue #2875 — прощание ждёт конец финальной формы, но не дольше этого
    # (страховка от протухшего/ошибочного ``form_ends_at``).
    FAREWELL_MAX_DEFER_S: float = 300.0

    def __init__(
        self,
        *,
        hook: DJHook,
        logger: logging.Logger,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self._hook = hook
        self._logger = logger
        # Issue #2856 — стенные часы инжектируются, чтобы тест мог прогнать
        # часовой сет за миллисекунды. Должны быть той же эпохи, что и
        # ``form_ends_at`` (``time.time()``, см. DJState).
        self._clock = clock
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

        # Issue #992 — capture BEFORE overwriting: this is the only
        # reliable "is this a genuine fresh start" signal. See
        # ``_apply_enable_payload`` for why ``transition_count == 0`` and
        # plan-string equality both turned out to be wrong signals for it.
        was_enabled = self.state.enabled
        self.state.enabled = enabled
        if enabled:
            self._apply_enable_payload(data, is_fresh_start=not was_enabled)
        else:
            # Issue #2835 — прощание только если DJ реально играл: «выключи»
            # по уже выключенному DJ (эхо собственной публикации ноды после
            # «новой сессии») не должно говорить «Вечеринка подошла к концу».
            self._reset_state(farewell=was_enabled)

    def reset_silently(self) -> None:
        """Issue #2835 — выключить DJ без прощальной фразы.

        Для «новой сессии»: нода сама говорит «Начинаю новую сессию…»,
        прощание DJ поверх неё — второй голос в тот же момент. Отложенное
        прощание (#2875) тоже отменяется — по той же причине.
        """
        self.state.farewell_at = None
        self._reset_state(farewell=False)

    def _apply_enable_payload(self, data: dict, *, is_fresh_start: bool) -> None:
        # 🔴 FIX (live 03.09 07:58): тема обновлялась ТОЛЬКО на генуинном
        # старте (``is_fresh_start or not self.state.theme``) — внутри
        # идущего сета «теперь тема Изнанка» меняло персону (у неё такого
        # гейта нет) и НЕ меняло тему. ``build_auto_prompt`` продолжал
        # подставлять `Тема вечеринки: "<старая>"` в каждый переход, и сет
        # уезжал обратно к прошлой теме. Асимметрия persona/theme ничем не
        # оправдана — обновляем так же безусловно.
        theme = data.get("theme")
        if theme and isinstance(theme, str) and theme.strip():
            new_theme = theme.strip()
            if new_theme != self.state.theme:
                theme_changed_midset = bool(
                    self.state.theme and not is_fresh_start
                )
                self.state.theme = new_theme
                self._logger.info(f"🎧 DJ theme: {self.state.theme!r}")
                if theme_changed_midset and self.state.set_plan:
                    # План прошлой темы («Трек 1: костры рябин...») в промпте
                    # новой темы — тот же откат, только через plan_block.
                    # Чистим ЗДЕСЬ, до разбора ``plan`` ниже: payload, где
                    # тема и новый план пришли вместе, отработает штатно.
                    #
                    # transition_count НЕ трогаем сознательно: сброс счётчика
                    # по содержимому payload — ровно та регрессия #992, из-за
                    # которой каждый переход снова становился «СТАРТ
                    # ВЕЧЕРИНКИ» (см. длинный комментарий ниже).
                    self._logger.info(
                        "🎧 DJ тема сменилась внутри сета — сбрасываю план "
                        f"прошлой темы (прогресс сохранён на "
                        f"#{self.state.transition_count})"
                    )
                    self.state.set_plan = ""
        # 🔴 FIX (live 10:13 DJ): персона юзера — «ты диджей Пёс» →
        # сохраняем, чтобы автопромпты использовали её вместо дефолта.
        persona = data.get("persona")
        if persona and isinstance(persona, str) and persona.strip():
            self.state.persona = persona.strip()
            self._logger.info(f"🎧 DJ persona: {self.state.persona!r}")
        next_sec = data.get("next_transition_sec")
        delay = float(max(15, min(300, int(next_sec)))) if next_sec else 60.0
        self.state.next_transition_at = self._clock() + delay
        # 🔴 FIX (live 15:30 06.08): план сета из set_dj_mode(plan=...) —
        # DJ идёт по плану и завершается финальным объявлением, а не
        # молча по лимиту DJ_AUTO_MAX_TRANSITIONS.
        #

        # 🔴 FIX (issue #992, ef525468e; РЕГРЕССИЯ вернулась в 102a6dea и
        # снова снята здесь). Текст плана — НЕГОДНЫЙ сигнал «новый сет».
        # Промпт перехода #1 сам просит модель сочинить план и отдать его
        # через set_dj_mode(plan=...), а стартовый вызов юзера почти всегда
        # уже несёт какой-нибудь план («Трек 1: ...»). Значит set_plan к
        # моменту перехода #1 непустой, приходящий план от него отличается —
        # и сброс счётчика по «переписыванию» превращал КАЖДЫЙ переход в
        # переход #1: build_auto_prompt(1) снова выдавал «СТАРТ ВЕЧЕРИНКИ»,
        # модель снова представлялась и снова писала план.
        #
        # Живой лог робота 01.09 (до ef525468e), «панк-вечеринка»:
        #   08:20:04 DJ auto-transition #1 → 08:20:27 plan 7 треков (rewrite)
        #   08:21:14 DJ auto-transition #1 → 08:21:33 plan 8 треков (rewrite)
        #   08:22:19 DJ auto-transition #1 → 08:22:40 plan 7 треков (rewrite)
        #   08:23:29 DJ auto-transition #1 → ... шесть раз подряд, ни одного
        # перехода дальше #1. И финальный трек, и auto-stop гейтятся на
        # transition_count, поэтому сет не мог ни развиться, ни закончиться.
        #
        # Единственный надёжный сигнал генуинного старта — DJ был ВЫКЛЮЧЕН
        # (``is_fresh_start = not was_enabled``), он обрабатывается ниже.
        #
        # Контракт (``test_dramaturgy_fix_1016``):
        #   * генуинный старт (DJ был выключен) — счётчик с нуля, иначе
        #     наследие прошлой сессии заставит ``build_auto_prompt(1)``
        #     притвориться «СТАРТ ВЕЧЕРИНКИ» уже не для новой партии;
        #   * первый план внутри идущей сессии — счётчик не трогаем;
        #   * переписывание плана в идущей сессии — тоже не трогаем:
        #     переписанный текст ≠ новый сет, прогресс должен сохраниться.
        plan = data.get("plan")
        if plan and isinstance(plan, str) and plan.strip():
            new_plan = plan.strip()
            if new_plan != self.state.set_plan:
                self.state.set_plan = new_plan

                self._logger.info(
                    f"🎧 DJ plan: {len(new_plan.splitlines())} треков "
                    f"(progress kept at #{self.state.transition_count})"
                )
        if is_fresh_start and self.state.transition_count:
            # Генуинный старт (enabled False→True) — прогресс прошлого сета
            # не должен утекать в новый.
            self._logger.info(
                f"🎧 DJ fresh start — сбрасываю счётчик переходов "
                f"(был #{self.state.transition_count})"
            )
            self.state.transition_count = 0
        self._apply_set_limits(data, is_fresh_start=is_fresh_start)
        self._logger.info(f"🎧 DJ Mode ON — next transition in {delay:.0f}s")

    @staticmethod
    def _clamped_int(value: Any, bounds: tuple) -> Optional[int]:
        """``value`` → int в ``bounds``; мусор/``<=0`` → ``None`` (лимита нет)."""
        if value is None or isinstance(value, bool):
            return None
        try:
            number = int(float(value))
        except (TypeError, ValueError):
            return None
        if number <= 0:
            return None
        low, high = bounds
        return max(low, min(high, number))

    def _apply_set_limits(self, data: dict, *, is_fresh_start: bool) -> None:
        """Issue #2856 — старт отсчёта сета и явные лимиты юзера.

        ``max_minutes`` — общая длительность сета ОТ СТАРТА (не «ещё N минут
        от этого вызова»): модель повторяет аргументы на каждом переходе, и
        отсчёт «от вызова» двигал бы дедлайн бесконечно — ровно тот баг,
        который здесь чинится.
        """
        if is_fresh_start or not self.state.started_at:
            self.state.started_at = self._clock()
            self.state.last_transition_at = 0.0
            self.state.final_dispatched = False
            # Issue #2875 — счёт треков принадлежит одному сету; прощание
            # прошлого сета, не успевшее прозвучать, новому не нужно.
            self.state.tracks_started = 0
            self.state.final_track_no = 0
            self.state.farewell_at = None
        minutes = self._clamped_int(
            data.get("max_minutes"), self.DJ_SET_MAX_MINUTES_RANGE
        )
        if minutes is not None:
            self.state.max_seconds = minutes * 60.0
            self._logger.info(f"🎧 DJ лимит сета: {minutes} мин от старта")
        tracks = self._clamped_int(
            data.get("max_tracks"), self.DJ_SET_MAX_TRACKS_RANGE
        )
        if tracks is not None:
            self.state.max_tracks = tracks
            self._logger.info(f"🎧 DJ лимит сета: {tracks} треков")

    def _reset_state(self, *, farewell: bool = True) -> None:
        # Capture persona before clearing state so the farewell hook can
        # address the user with the correct DJ name (issue #1101).
        farewell_persona = self.state.persona or self._persona_default
        farewell_theme = self.state.theme or ""
        # Issue #2875 — конец формы нужен прощанию (см. _farewell_after_form).
        form_ends_at = self.state.form_ends_at
        self.state.next_transition_at = 0.0
        self.state.transition_count = 0
        self.state.theme = ""
        self.state.set_plan = ""
        self.state.persona = ""
        # Issue #2461 — не тащить дедлайн формы прошлого сета в следующий.
        self.state.form_ends_at = None
        # Issue #2856 — лимиты и отсчёт времени принадлежат одному сету.
        self.state.started_at = 0.0
        self.state.last_transition_at = 0.0
        self.state.max_seconds = None
        self.state.max_tracks = 0
        self.state.final_dispatched = False
        self.state.tracks_started = 0
        self.state.final_track_no = 0
        # 🔴 FIX (live 11:46): без этого enabled оставался True после
        # авто-стопа → следующий tick (5с) видел next_transition_at=0.0 и
        # запускал НОВЫЙ DJ-цикл #1 — DJ «оживал» через 5 секунд после
        # остановки (бесконечность). enabled=False — единственный
        # надёжный выключатель: tick() сразу возвращается.
        self.state.enabled = False
        self._logger.info("🎧 DJ Mode OFF")
        if farewell:
            self._farewell_after_form(farewell_persona, form_ends_at)

    # ── Farewell (issue #2875 addendum) ─────────────────────────────

    def _farewell_after_form(
        self, persona: str, form_ends_at: Optional[float]
    ) -> None:
        """Прощание — после конца играющей формы, а не поверх её начала.

        Живой прогон 23.09 17:54: финальный переход → compose_music
        (repeat=false, форма ~250 с) → через 2 с set_dj_mode(enabled=false)
        → «Вечеринка подошла к концу…» прозвучало В НАЧАЛЕ финального
        трека. Если форма ещё играет — прощание откладывается до её конца
        (не дольше ``FAREWELL_MAX_DEFER_S``); ``tick()`` его произнесёт.
        ``form_ends_at`` сохраняется, чтобы ``tick()`` видел остановку
        музыки (mcp_server присылает ``null``) и прощался сразу.
        """
        now = self._clock()
        if form_ends_at is None or form_ends_at <= now:
            self._say_farewell(persona)
            return
        self.state.form_ends_at = form_ends_at
        self.state.farewell_at = min(form_ends_at, now + self.FAREWELL_MAX_DEFER_S)
        self.state.farewell_persona = persona
        self._logger.info(
            f"🎧 DJ прощание отложено до конца формы "
            f"(через {self.state.farewell_at - now:.0f}с)"
        )

    def _say_farewell(self, persona: str) -> None:
        if self._hook.on_stop is None:
            return
        try:
            self._hook.on_stop(persona)
        except Exception as exc:  # noqa: BLE001
            self._logger.warning(
                f"⚠️ DJ on_stop hook failed: {type(exc).__name__}: {exc}"
            )

    def _fire_due_farewell(self, now: float) -> None:
        """Сказать отложенное прощание: форма доиграла или музыка стоп."""
        due_at = self.state.farewell_at
        if due_at is None:
            return
        music_stopped = self.state.form_ends_at is None
        if now < due_at and not music_stopped:
            return
        self.state.farewell_at = None
        self._say_farewell(self.state.farewell_persona)

    # ── Tick ────────────────────────────────────────────────────────

    def tick(self) -> None:
        """Called from the shell's 5-second timer."""
        now = self._clock()
        # Issue #2875 — отложенное прощание живёт после выключения DJ.
        self._fire_due_farewell(now)
        if not self.state.enabled:
            return
        if not self.state.started_at:
            # Включили в обход handle_message (тесты, рестарт) — отсчёт
            # сета с первого тика, а не с эпохи 0 (иначе сразу «финал»).
            self.state.started_at = now
        gate_at = self.state.next_transition_at
        # Issue #2461 — форма как НИЖНЯЯ граница перехода, не единственный
        # источник. ``next_transition_sec`` (через ``next_transition_at``)
        # остаётся ручным перекрытием модели — она может ЗАТЯНУТЬ переход,
        # но структурно больше не может его УКОРОТИТЬ ниже реального конца
        # формы: живой баг (#2461) был именно в этом — next_transition_sec=45
        # при форме на 96-190с срезал дроп на каждом сете.
        #
        # ``form_ends_at`` учитывается ТОЛЬКО если оно ещё не в прошлом.
        # ``None`` (топик не пришёл/форма не играет) и протухшее значение
        # (форма из прошлого трека, уже отыгравшая) одинаково НЕ блокируют
        # переход — работает прежнее поведение по ``next_transition_at``.
        # Иначе стухший сигнал (пропущенное сообщение, трек сменился без
        # обновления) держал бы DJ замороженным навсегда.
        form_ends_at = self.state.form_ends_at
        if form_ends_at is not None and form_ends_at > now:
            gate_at = max(gate_at, form_ends_at)
        if now < gate_at:
            return
        # Don't interrupt an active dialogue or sound playback.
        if self._hook.is_dialogue_active() or self._hook.is_active():
            self.state.next_transition_at = now + self.POSTPONE_INTERVAL_S
            return

        # Hard-stop when the plan / set limit is exhausted.
        plan_tracks = self.state.set_plan.count("Трек ")
        next_n = self.state.transition_count + 1
        if self._should_stop(next_n, plan_tracks):
            self._reset_state()
            return

        if self._set_limit_reached(next_n, now, plan_tracks):
            # Issue #2856 — этот переход последний: объявленный финальный
            # трек + прощание вместо молчаливого обрыва на следующем тике.
            self.state.final_dispatched = True
            # Issue #2875 — финал «закрыт», только когда этот трек реально
            # запустился; провал перехода даёт финалу ещё одну попытку.
            self.state.final_track_no = self.state.tracks_started + 1
            self._logger.info(
                f"🎧 DJ лимит сета достигнут — переход #{next_n} финальный "
                f"(сет идёт {now - self.state.started_at:.0f}с)"
            )
        self.state.transition_count = next_n
        self.state.last_transition_at = now
        self.state.next_transition_at = now + self.FALLBACK_INTERVAL_S
        self._logger.info(f"🎧 DJ auto-transition #{next_n}")
        # Issue #992 Bug B — ``from_tick=True`` lets the dispatcher
        # reset its synchronous-retry budget for this fresh transition.
        self._hook.dispatch(self.build_auto_prompt(next_n), True)

    def _should_stop(self, next_n: int, plan_tracks: int) -> bool:
        """True — сет исчерпан, переход ``next_n`` не играть, а выключить DJ."""
        started = self.state.tracks_started
        if self.state.final_dispatched and started >= self.state.final_track_no:
            # Issue #2856 — финальный трек по лимиту уже отыгран, а модель
            # не выключила DJ сама. Прощание скажет хук ``on_stop``.
            # Issue #2875 — «отыгран» = реально запущен (счётчик треков).
            self._logger.info(
                f"🛑 DJ auto-stop: финальный трек сета отыгран "
                f"(переход #{next_n} не нужен)"
            )
            return True
        if plan_tracks > 0 and started >= plan_tracks:
            # Issue #2875 — все треки плана реально сыграны, модель не
            # выключила DJ сама после финального.
            self._logger.info(
                f"🛑 DJ auto-stop: план сыгран ({started}/{plan_tracks} треков)"
            )
            return True
        # 🔴 FIX (live 11:19 DJ): save_dj_set_plan тула НЕТ — set_plan всегда
        # пуст → plan_tracks=0 → авто-стоп никогда не срабатывал → DJ
        # крутился бесконечно (#22+, час музыки, юзер не может выйти).
        # Фолбэк: если плана нет — жёсткий лимит переходов
        # (DJ_AUTO_MAX_TRANSITIONS), после которого DJ сам выключается.
        # С #2856 это только страховка: основной лимит — по времени.
        if plan_tracks == 0 and next_n > self.DJ_AUTO_MAX_TRANSITIONS:
            self._logger.warning(
                f"🛑 DJ auto-stop: переход #{next_n} превысил лимит "
                f"без плана ({self.DJ_AUTO_MAX_TRANSITIONS}) — "
                "save_dj_set_plan не вызывался, останавливаю DJ"
            )
            return True
        if plan_tracks > 0 and next_n > plan_tracks + self.DJ_AUTO_STOP_THRESHOLD:
            self._logger.warning(
                f"🛑 DJ auto-stop: transition #{next_n} beyond plan ({plan_tracks})"
            )
            return True
        return False

    def _set_limit_s(self, plan_tracks: int) -> Optional[float]:
        """Лимит сета по времени: явный от юзера, иначе дефолт — только без плана."""
        if self.state.max_seconds:
            return self.state.max_seconds
        if plan_tracks == 0:
            return self.DJ_SET_DEFAULT_MAX_S
        return None

    def _set_limit_reached(self, n: int, now: float, plan_tracks: int) -> bool:
        """Issue #2856 — переход ``n`` должен стать финальным треком сета.

        Трек перехода ``n`` играет примерно до ``now + интервал``. Финальным
        он становится, когда ЕЩЁ ОДИН трек после него уже не влезет в лимит:
        ``now + 2·интервал > started_at + лимит``. Тогда финальный трек
        доигрывает не позже лимита, и сет заканчивается объявленным финалом
        + прощанием, а не обрывом. Если оценка промахнулась и мы уже за
        лимитом — условие тем более истинно: всё равно объявленный финал,
        не обрыв. Интервал — длина последнего перехода (с #2461 это реальная
        длина формы, 150-200 с); до второго перехода — ``FALLBACK_INTERVAL_S``.

        Переход #1 («СТАРТ ВЕЧЕРИНКИ») финальным не бывает, пока в сете не
        сыграно ни одного трека. Финал по плану здесь не решается — его, как
        и раньше, решает ``build_auto_prompt``.

        Issue #2875 — лимит ``max_tracks`` сравнивается с номером
        СЛЕДУЮЩЕГО ТРЕКА (``tracks_started + 1``), а не перехода: переход,
        где модель не запустила музыку, трек не расходует.
        """
        track_no = self.state.tracks_started + 1
        if n <= 1 and track_no <= 1:
            return False
        track_limit = self.state.max_tracks or (
            self.DJ_AUTO_MAX_TRANSITIONS if plan_tracks == 0 else 0
        )
        if track_limit and track_no >= track_limit:
            return True
        if n <= 1:
            return False
        limit_s = self._set_limit_s(plan_tracks)
        if limit_s is None:
            return False
        last = self.state.last_transition_at
        interval = (now - last) if last else self.FALLBACK_INTERVAL_S
        return now + 2 * interval > self.state.started_at + limit_s

    # ── Track accounting (issue #2875) ──────────────────────────────

    def note_turn_tools(
        self,
        tools_called: Optional[Iterable[str]],
        music_tools: Iterable[str],
        *,
        is_dj_auto: bool = False,
        turn_text: str = "",
    ) -> bool:
        """Учесть завершённый ход: запустил ли он трек в идущем сете.

        Нода зовёт это на КАЖДЫЙ ход (DJ_AUTO, ретрай, реплика юзера) с
        ``result.tools_called`` и ``MUSIC_STARTING_TOOLS`` — тем же
        сигналом «музыка стартовала», по которому работают её гарды.
        Считается ход, а не вызов: два compose_music за ход — один трек
        (#2859 и так не даёт больше одного запуска за ход).

        Трек СЕТА — ход DJ_AUTO (``is_dj_auto``) или ход юзера, который сам
        управлял сетом (``set_dj_mode`` в том же ходе): «ты диджей … играй
        X, Y, Z» с ``set_dj_mode`` + ``compose_music``. ``/voice/dj_mode``
        доходит до ноды раньше результата хода (живой лог 23.09 17:30:21),
        так что DJ к этому моменту уже включён, и переход #1 сыграет Трек 2.

        Ретраи DJ-перехода (Bug D / music-гард) идут с ``is_dj_auto=False``
        (живой лог 23.09 17:32:33), но несут текст перехода — маркер
        ``[DJ_AUTO`` в ``turn_text`` тоже делает ход ходом сета.

        Музыкальная просьба юзера посреди сета без ``set_dj_mode``
        («сыграй тему марио») — заказ гостя, см. :meth:`_hold_for_user_track`.

        Returns:
            True — засчитан трек сета.
        """
        if not self.state.enabled or not tools_called:
            return False
        tools = set(tools_called)
        if not tools & set(music_tools):
            return False
        set_turn = is_dj_auto or "[DJ_AUTO" in (turn_text or "")
        if not set_turn and "set_dj_mode" not in tools:
            self._hold_for_user_track()
            return False
        self.state.tracks_started += 1
        self._logger.info(
            f"🎧 DJ трек #{self.state.tracks_started} запущен "
            f"(переход #{self.state.transition_count})"
        )
        return True

    def _hold_for_user_track(self) -> None:
        """Заказ юзера посреди сета доигрывает форму, потом сет продолжается.

        Живой прогон 23.09 17:52: «сыграй тему марио…» посреди сета →
        играет Марио → переход #6 в 17:54:52 его заменил. Выбран вариант
        «сет ждёт заказ», а не «заказ выключает DJ»:

        * юзер сам задал сету рамки (план, ``max_minutes``) — одна просьба
          посреди вечеринки это заказ гостя диджею, а не отмена вечеринки;
          выключение молча выбросило бы план и потребовало бы новой команды;
        * «хватит/выключи диджея» по-прежнему выключает DJ явно (stop-гарды);
        * механизм уже есть: переход гейтится концом формы (#2461), здесь
          только гарантируем, что до ``/voice/music/form`` нового трека
          переход не выстрелит по старому ``next_transition_at``.

        Заказ не расходует трек плана и лимит ``max_tracks``: трек не из
        сета (лимит по времени #2856 при этом идёт как шёл).
        """
        now = self._clock()
        hold_until = now + self.FALLBACK_INTERVAL_S
        if self.state.next_transition_at < hold_until:
            self.state.next_transition_at = hold_until
        self._logger.info(
            "🎧 DJ: заказ юзера посреди сета — следующий переход не раньше "
            f"{hold_until - now:.0f}с и конца его формы (трек сета не засчитан)"
        )

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

    def suppresses_free_text(self, n: int) -> bool:
        """True — свободный текст ответа LLM на переходе ``n`` не озвучивать.

        Речь на переходе идёт ТОЛЬКО через инструменты/хуки, а не через
        свободный текст:
        * #1 («СТАРТ ВЕЧЕРИНКИ») — представление диджея через ``speak_text``,
          свободная форма разрешена (возвращает False);
        * середина сета — короткая тематическая фраза через ``speak_text``;
        * финальный трек — прощание произносит хук ``_on_dj_stop_farewell``.

        Свободный текст на середине/финале — это мета-болтовня
        («Переход номер два отыгран — нарастание с дропом в ре миноре
        фригийском, сто сорок ударов!»), которую модель писала мимо
        ``speak_text``, и она уходила в TTS поверх бита каждые 45 секунд.
        Поэтому свободный текст там глушится: фраза идёт только через
        ``speak_text``, прощание — через хук.
        """
        if n <= 1:
            return False
        return True

    def _plan_track_line(self, track_no: int) -> str:
        """Issue #2875 — какой трек плана играть сейчас и как.

        Пусто без плана. Названные песни играются из RTTTL-библиотеки
        через ``compose_music(name=...)``: живой прогон 23.09 — модель
        заявила, что Still Dre / Next Episode «не лежат» в архиве, хотя
        stilldre_2 / nextepis там есть; lookup_melody она не вызывала.

        Issue #2966 (повтор живьём 24.09.2026, тот же класс) — формулировка
        ниже ОБЩАЯ, без конкретных названий: composer.txt уже держит
        RULE #KNOWN-MELODY с тем же контрактом («названо по имени →
        сначала lookup») для обычных запросов юзера; «трек плана» — просто
        ещё один источник имени, к которому применяется то же правило, а
        не отдельный список песен под спецобработку.
        """
        if not self.state.set_plan:
            return ""
        entry = plan_entry(self.state.set_plan, track_no)
        if not entry:
            return (
                f"▶ Сейчас по плану — Трек {track_no}: сыграй его через "
                "compose_music. "
            )
        return (
            f"▶ Сейчас по плану — Трек {track_no}: «{entry}». Если это "
            "название конкретной песни/композиции (не жанр и не "
            "описание вайба) — действует RULE #KNOWN-MELODY (см. "
            "composer.txt): НЕ импровизируй по памяти, СНАЧАЛА "
            f'compose_music(name="{entry}") — тул сам ищет точные ноты в '
            f'RTTTL-базе; при сомнении в написании названия — lookup_melody('
            f'name="{entry}") первым отдельным вызовом. Если в строке плана '
            "не песня, а описание — compose_music в этом духе, без name=. "
            "НЕ говори, что трека нет, не вызвав lookup_melody. "
        )

    def build_auto_prompt(self, n: int) -> str:
        persona = self.state.persona or self._persona_default
        theme_line = (
            f'Тема вечеринки: "{self.state.theme}". '
            if self.state.theme
            else ""
        )
        # 🔴 FIX (issue #2441): аранжировка больше не дублируется в
        # build_auto_prompt. Раньше переходы несли свою копию «ремесла
        # композитора» (плотность/свинг/индексы сэмплов; form/progression/
        # слои) — она отстала от composer.txt, и диджей на старте сета не
        # знал про lookup_melody / name= / RTTTL-темы. Теперь prompt
        # описывает только СОСТОЯНИЕ сета и DJ-запреты (load_track /
        # list_tracks); ремесло аранжировки живёт в одном месте —
        # composer.txt, и DJ_AUTO-ход форсирует его через
        # _activate_skill_for(force_skill="composer").
        # 🔴 FIX (live 02.09): диджей переключал трек каждые 40-45 с, а форма
        # compose_music играет 96-190 с (arranger.FORMS). На buildup при
        # 128 BPM это 142 с: intro 15 с + build 30 с — и переключение ровно
        # на 45-й секунде, ДО gap/drop/drop2. За 30 часов лога робота drop не
        # прозвучал ни разу ни в одном сете: слушатель получал шесть подряд
        # «вступление + разгон». Отсюда жалоба «музыка однотипная» — при том
        # что материал у треков был вполне разный.
        length_line = (
            "⏱ Дай треку доиграть форму: следующий переход назначай не раньше, "
            "чем длительность, которую вернул compose_music (next_transition_sec)."
        )
        stage_marker = f"Стадия сета: переход #{n}. "
        library_line = (
            "❌ НЕ вызывай load_track / list_tracks в переходах — load_track "
            "СРАЗУ запускает сохранённый трек из базы и даёт резкую вставку "
            "между треками. ✅ search_samples(<стиль>) — для "
            "реальных сэмплов и разнообразия тембров между треками."
        )
        plan_block = (
            f"План сета:\n{self.state.set_plan}\n" if self.state.set_plan else ""
        )
        plan_tracks = self.state.set_plan.count("Трек ") if self.state.set_plan else 0
        # Issue #2875 — номер трека плана = треки, РЕАЛЬНО запущенные в сете
        # + 1, а не номер перехода: провал перехода трек не съедает.
        track_no = self.state.tracks_started + 1
        track_line = self._plan_track_line(track_no)
        if n == 1 and plan_tracks and track_no < plan_tracks:
            # Issue #2875 — план уже задан юзером: без ресёрча и нового
            # плана, сразу играем его трек. Живой прогон 23.09: промпт
            # ниже гнал модель в search_web + новый план, трек не звучал
            # 4 минуты.
            return (
                "[DJ_AUTO — СТАРТ ВЕЧЕРИНКИ] "
                f"Ты {persona} — первый в мире робот-диджей. {theme_line}"
                f"{plan_block}"
                "План сета УЖЕ ЕСТЬ — НЕ исследуй материал (search_web / "
                "gen_search_library не нужны) и НЕ составляй новый план. "
                f"{track_line}{library_line} {stage_marker}{length_line} "
                f"Затем представься как {persona} через speak_text."
            )
        if n == 1 and not plan_tracks:
            return (
                "[DJ_AUTO — СТАРТ ВЕЧЕРИНКИ] "
                f"Ты {persona} — первый в мире робот-диджей. {theme_line}"
                "🔎 СНАЧАЛА ИССЛЕДУЙ МАТЕРИАЛ: "
                "1) search_web(<персона> — стиль, темп, характерные приёмы) — "
                "изучи персону и её музыку; 2) search_samples(<стиль>) — найди "
                "реальные сэмплы (макс. 2 вызова); 3) gen_search_library(<персона>) "
                "— посмотри, что есть в AI-библиотеке для вдохновения. "
                "📋 ЗАТЕМ СОСТАВЬ ПЛАН СЕТА из 5-8 треков (дуга: вход → "
                "нарастание → пик → спуск) и сохрани через "
                "set_dj_mode(enabled=true, plan=<список треков, каждый с новой "
                "строки 'Трек N: ...'>, next_transition_sec=<длительность формы "
                "из ответа compose_music>). Потом сыграй "
                f"трек #1 через compose_music. {library_line} {stage_marker}"
                f"{length_line} "
                f"Затем представься как {persona} через speak_text."
            )
        # Issue #2856 — финал по лимиту времени/треков (``final_dispatched``
        # взводит ``tick()``) звучит так же, как финал по плану.
        limit_final = (
            self.state.final_dispatched and n == self.state.transition_count
        )
        if (plan_tracks and track_no >= plan_tracks) or limit_final:
            return (
                f"[DJ_AUTO переход #{n} — ФИНАЛЬНЫЙ ТРЕК] "
                f"Ты {persona}. {theme_line}{plan_block}"
                f"{track_line}{library_line} "
                "Это ПОСЛЕДНИЙ трек сета. Сыграй завершающий трек через "
                "compose_music с repeat=false (форма сама доводит его до "
                "спокойного финала и затухания — не проси зацикленный трек). "
                "Затем ОБЯЗАТЕЛЬНО вызови set_dj_mode(enabled=false) — "
                "DJ-режим завершается. Прощание НЕ говори и НЕ пиши текст, "
                "и НЕ вызывай speak_text в этом ходе: система сама скажет "
                "«вечеринка подошла к концу», когда трек доиграет."
            )
        return (
            f"[DJ_AUTO переход #{n}] "
            f"Ты {persona}. {theme_line}{plan_block}"
            f"{track_line}{library_line} {stage_marker} "
            "Сыграй следующий трек через compose_music (repeat=true, другой "
            "bpm/root/scale/synth в духе темы, чем предыдущий трек). "
            f"{length_line} "
            "🔥 РАЗОГРЕЙ ТОЛПУ: перед стартом трека вызови speak_text с ОДНОЙ "
            "короткой тематической фразой-выкриком в стиле персоны и в тему "
            "сета (до 30 символов, напр. «Разгоняемся!» или «А теперь — "
            "пожар!»). НЕ пиши свободный текст ответа и НЕ комментируй, что "
            "ты делаешь — свободный текст не озвучивается, работает только "
            "speak_text. После этого вызови set_dj_mode(enabled=true, "
            "next_transition_sec=<столько же секунд>) для следующего перехода."
        )


__all__ = ["DJModeController", "DJState", "DJHook", "plan_entry"]
