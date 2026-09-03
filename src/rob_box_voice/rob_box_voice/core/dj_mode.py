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

        # Issue #992 — capture BEFORE overwriting: this is the only
        # reliable "is this a genuine fresh start" signal. See
        # ``_apply_enable_payload`` for why ``transition_count == 0`` and
        # plan-string equality both turned out to be wrong signals for it.
        was_enabled = self.state.enabled
        self.state.enabled = enabled
        if enabled:
            self._apply_enable_payload(data, is_fresh_start=not was_enabled)
        else:
            self._reset_state()

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
        self.state.next_transition_at = time.time() + delay
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

    def is_music_only_transition(self, n: int) -> bool:
        """True, когда промпт перехода ``n`` запрещает модели говорить.

        Зеркалит ветвление :meth:`build_auto_prompt`: переход #1 —
        «СТАРТ ВЕЧЕРИНКИ» с представлением диджея, последний трек плана —
        прощание; всё между ними обязано быть только музыкой.

        Shell использует это, чтобы не озвучивать свободный текст ответа
        LLM на таких переходах. Запрет «НЕ вызывай speak_text» в промпте
        закрывал только тул: модель вместо него писала обычную реплику
        («Переход номер два отыгран — нарастание с дропом в ре миноре
        фригийском, сто сорок ударов!»), и та уходила в TTS поверх бита
        каждые 45 секунд.
        """
        if n <= 1:
            return False
        plan_tracks = self.state.set_plan.count("Трек ") if self.state.set_plan else 0
        if plan_tracks and n >= plan_tracks:
            return False  # финальный трек — прощание разрешено
        return True

    def build_auto_prompt(self, n: int) -> str:
        persona = self.state.persona or self._persona_default
        theme_line = (
            f'Тема вечеринки: "{self.state.theme}". '
            if self.state.theme
            else ""
        )
        # 🔴 FIX (issue #1811): переходы раньше писались рукописным
        # Renardo-кодом мимо аранжировщика — отсюда слои на p4 (#1804),
        # рисунки не по такту (#1803), октавы за пределом (bell(oct=7)).
        # #1805/#1806 научили compose_music развитию по секциям
        # (транспозиция/инверсия/ракоход/реприза), плотности и свингу —
        # старые ограничения на паттерны/сумму amp теперь держит аранжировщик
        # и мастер-фильтр, модели про них думать не нужно.
        tech_line = (
            "⚙️ compose_music: используй hats_sample/perc/perc_sample, "
            "чтобы хэты и перкуссия отличались от предыдущего трека — "
            "иначе сет звучит одним и тем же битом. swing=0.1-0.2 для "
            "джаза/фанка/шафла — на ровных восьмых они не звучат как жанр. "
            "Разные drums_sample/hats_sample/perc_sample индексы между "
            "треками — не переиспользуй одни и те же. НЕ повторяй "
            "synth/root/scale предыдущего трека — меняй хотя бы один."
        )
        # 🔴 FIX (live 02.09): диджей переключал трек каждые 40-45 с, а форма
        # compose_music играет 96-190 с (arranger.FORMS). На buildup при
        # 128 BPM это 142 с: intro 15 с + build 30 с — и переключение ровно
        # на 45-й секунде, ДО gap/drop/drop2. За 30 часов лога робота drop не
        # прозвучал ни разу ни в одном сете: слушатель получал шесть подряд
        # «вступление + разгон». Отсюда жалоба «музыка однотипная» — при том
        # что материал у треков был вполне разный.
        length_line = (
            "⏱ compose_music возвращает длительность формы. Дай треку "
            "доиграть: следующий переход назначай НЕ РАНЬШЕ этого времени "
            "(next_transition_sec). Переключение на 45-й секунде срезает "
            "дроп и кульминацию — весь сет звучит как несколько одинаковых "
            "вступлений."
        )
        stage_line = (
            f"Стадия сета: переход #{n}. "
            "Трек должен РАЗВИВАТЬСЯ внутри перехода — это делает форма "
            "compose_music (form=buildup для нарастания с дропом, form=arc "
            "для универсальной дуги), передавай progression и 3+ слоя, "
            "не проси статичный четырёхтактовый луп."
        )
        library_line = (
            "🎵 compose_music — ТВОЙ ИНСТРУМЕНТ ДЛЯ ЛЮБОГО ПЕРЕХОДА: он даёт "
            "форму и развитие бесплатно, тебе нужно только описать материал "
            "(bpm/root/scale/синты/ступени). "
            "❌ НЕ вызывай load_track / list_tracks в переходах — load_track "
            "СРАЗУ запускает сохранённый трек из базы и даёт резкую вставку "
            "между треками. ✅ search_samples(<стиль>) — для "
            "реальных сэмплов и разнообразия тембров между треками."
        )
        if n == 1:
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
                f"трек #1 через compose_music. {library_line} {stage_line} "
                f"{length_line} "
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
                "compose_music с repeat=false (форма сама доводит его до "
                "спокойного финала и затухания — не проси зацикленный трек). "
                "Затем ОБЯЗАТЕЛЬНО: 1) speak_text: «Вот и всё, вечеринка "
                "заканчивается! Спасибо, что были со мной!» (или в тему "
                "сета); 2) set_dj_mode(enabled=false) — DJ-режим завершается."
            )
        return (
            f"[DJ_AUTO переход #{n}] "
            f"Ты {persona}. {theme_line}{plan_block}"
            f"{library_line} {tech_line} {stage_line} "
            "Сыграй следующий трек через compose_music (repeat=true, другой "
            "bpm/root/scale/synth в духе темы, чем предыдущий трек). "
            f"{length_line} "
            "После этого вызови set_dj_mode(enabled=true, "
            "next_transition_sec=<столько же секунд>) для следующего перехода. "
            "НЕ вызывай speak_text И НЕ ПИШИ текст ответа — переход только про "
            "музыку. Объявление поверх играющего бита юзер не просил."
        )


__all__ = ["DJModeController", "DJState", "DJHook"]
