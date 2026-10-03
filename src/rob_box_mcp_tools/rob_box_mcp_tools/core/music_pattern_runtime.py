"""music_pattern_runtime.py — runtime паттернов Renardo (ADR-0134 Phase 4).

Сюда вынесены 13 методов из ``tools/music.py::MusicManager``, которые
отвечают за валидацию, маршрутизацию и жизненный цикл паттернов
(``execute_code`` / ``stop_pattern`` / ``stop_all`` / watchdog / deadlines).
``MusicManager`` остаётся composition root: держит состояние и сервисы,
а ``MusicPatternRuntime`` — только методы, которые это состояние
потребляют (по ADR-0134 §3.2, «composition root, не DI-фреймворк»).

Почему отдельный модуль
=======================

В ``MusicManager`` исторически скопились:

* ``execute_code`` CC=22 — фильтр кода, health-check, segments safety-net,
  prewarm, exec, teardown старых нод, master-gain, сессионный учёт.
* ``stop_all`` CC=16 — per-player stop, Clock.clear, ramp-down, freeAll,
  сброс сессии, warning-логика.

Без выноса CC-budget ADR-0021 R1 (METHOD_LIMIT=15, жёстче — ≤12 по issue
#3014) ловит их как regression, а любой будущий рефакторинг внутри
``MusicManager`` трогает всех потребителей. Вынос в ``core/`` следует
прецеденту ``core/arrangement_presets`` (PR-7) и ``core/renardo_sanitizer``
(PR-1) — без ``rclpy``, без I/O, тестируется ``Mock``-ом ``MusicManager``.

Декомпозиция
============

* ``execute_code`` (CC=22) → 1 orchestrator + 6 private helpers, каждый
  с одной ответственностью. Orchestrator линеен, без вложенных
  if-else; CC оценка ~6.
* ``stop_all`` (CC=16) → 1 orchestrator + 3 private helpers. CC ~5.
* Остальные 11 методов — CC ≤ 12 без декомпозиции, по оценке дизайна
  ``docs/design/music_pattern_runtime.md``.

Границы
=======

* **Composition:** ``MusicPatternRuntime`` хранит ``self._mgr`` —
  ссылку на ``MusicManager`` (один composition root). Никаких полей
  состояния в runtime; никакого DI-контейнера; никакого кеша.
* **Сервисы:** ``is_music_stack_healthy`` / ``music_stack_unavailable_error``
  / ``_check_supercollider`` / ``_ensure_renardo_available`` /
  ``_send_osc_raw`` / ``set_master_gain`` остаются в ``MusicManager``
  до Phase 2/3/5 (см. ``docs/design/music_pattern_runtime.md`` §2.1).
  Здесь они вызываются через ``self._mgr.<method>()`` (shim-вызовы).
* **Public API ``MusicManager``:** 13 сигнатур сохранены байт-в-байт —
  ``MusicManager.execute_code`` / ``stop_pattern`` / ``stop_all`` /
  ``auto_stop_idle_music`` / ``stop_music_on_session_end`` /
  ``set_form_deadline`` / ``set_form_cycle_end`` / ``clear_form_deadline``
  остаются публичными методами с теми же сигнатурами. Тело заменено на
  1-строчный ``return self._runtime.<method>(...)``. ``ExecuteMusicCodeTool.execute``
  и другие потребители продолжают работать без изменений.
* **Сторона имён:** runtime-методы, которые в ``MusicManager`` назывались
  ``_call_player_stop`` / ``_prewarm_sample_buffers`` / ``_resolve_pattern_name``
  / ``_renardo_bpm`` / ``_schedule_stop`` (с подчёркиванием — internal),
  здесь публичны **без** подчёркивания (``call_player_stop`` и т.д.) —
  ``core/`` модуль не имеет понятия о private-API ``MusicManager``.
  Wrappers в ``MusicManager`` сохраняют свои ``_``-имена для
  совместимости с любыми внутренними вызовами.
"""

from __future__ import annotations

import re
import sys
import time
from typing import Any, Dict, Optional, Tuple

# Санитизация Renardo-кода вынесена в ``core/renardo_sanitizer`` —
# единый seam, чтобы все запреты (security / quality / slot / amp-cap)
# были в одном месте, не размазаны по ``MusicManager``.
from . import renardo_sanitizer, sample_loops

# Адаптер Renardo (ADR-0142, владелец плеера v2) — общая утилита
# предзагрузки sample-буферов (``load_sample_buffers`` фильтрует
# ``.``-паузы и пробелы внутри, см. renardo_adapter.py:148).
from ..engine import renardo_adapter

# Module-level: символы ``play("x-o-")`` для предзагрузки буферов (issue
# #1815, live 13.08). ``-`` — звучащий символ, ``.`` — настоящая пауза.
_PLAY_SYMBOLS_RE = re.compile(r'play\(\s*"([^"]*)"')

#: Renardo player namespace: d1-d9, p1-p9, s1-s9, l1-l9. Тот же
#: набор, что в ``tools/music.py`` (RCE-защита, issue G-MUSIC).
_RENARDO_PLAYER_NAMES: frozenset = frozenset(
    f"{prefix}{i}" for prefix in ("d", "p", "s", "l") for i in range(1, 10)
)

#: Имя паттерна — bare Python identifier. Никаких точек, вызовов, кавычек,
#: комментариев, пробелов — иначе считаем имя LLM-инъекцией.
_PATTERN_NAME_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]{0,31}$")


# ---------------------------------------------------------------------------
# Public class
# ---------------------------------------------------------------------------


class MusicPatternRuntime:
    """Runtime паттернов Renardo: валидация, маршрутизация, жизненный цикл.

    Не владеет состоянием — держит ссылку на ``MusicManager`` (composition
    root, ADR-0134 §3.2). Декомпозирует 13 методов из ``tools/music.py``
    так, чтобы каждый public-метод и каждый helper ≤ 12 CC (issue #3014,
    жёстче, чем ``METHOD_LIMIT=15`` ADR-0021 R1).

    Использование:

    .. code-block:: python

        runtime = MusicPatternRuntime(manager)   # manager: MusicManager
        result = runtime.execute_code(
            "d1 >> play('x')",
            pattern_name="drums",
            segments=8,
        )
    """

    def __init__(self, manager: Any) -> None:
        """``manager`` — :class:`MusicManager` (composition root)."""
        self._mgr = manager

    # ------------------------------------------------------------------
    # execute_code — оркестратор + 6 helpers
    # ------------------------------------------------------------------

    def execute_code(
        self,
        code: str,
        pattern_name: Optional[str] = None,
        *,
        segments: Optional[int] = None,
        duration_sec: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Безопасно выполнить Renardo-код. (контракт см. ``tools/music.py``).

        Шаги (linear fall-through, ни одного вложенного if-else):
        1. ``_execute_sanitize`` — единый seam очистки; ранний return error.
        2. ``_execute_emit_debug_log`` — debug-stderr «FINAL CODE» (live 15:44).
        3. ``_execute_check_stack_health`` — health + SC + Renardo; ранний return.
        4. ``_execute_apply_segments_safety_net`` — переменные ``__total_*``
           + ``schedule_stop`` (issue #990).
        5. ``prewarm_sample_buffers`` — анти-xrun для play("..."), live 13.08.
        6. ``_execute_run`` — exec + post-exec teardown; ранний return error.

        Завершение (без ранних return):
        7. lazy apply master gain (мастер-лимитер в scsynth ещё не
           существует в первые ~5с после старта sclang — отложенная
           отправка).
        8. ``_execute_record_session_activity`` — финальный dict (success +
           warnings + session state).
        """
        sanitized, err = self._execute_sanitize(code)
        if err is not None:
            return err
        code = sanitized.code
        quality_warnings: list = list(sanitized.warnings)

        self._execute_emit_debug_log(code)

        health_err = self._execute_check_stack_health()
        if health_err is not None:
            return health_err

        self._execute_apply_segments_safety_net(
            code, segments=segments, duration_sec=duration_sec,
        )

        self.prewarm_sample_buffers(code)
        self._mgr._prepare_renardo_namespace()

        has_clock_clear = "Clock.clear()" in code
        # ``is_fade_wrapped`` приходит из ``core.club_transition`` (ADR-0149
        # PR-6) — у fade-обёртки ``Clock.clear()`` teardown старых нод
        # происходит ВНУТРИ ``_rbx_next_track`` на момент реального старта
        # нового трека, а не здесь (issue #3166). Импортим лениво, чтобы
        # core/ не зависел от порядка инициализации пакетов.
        from .club_transition import is_fade_wrapped
        deferred_start = is_fade_wrapped(code)

        exec_err = self._execute_run(code, has_clock_clear, deferred_start)
        if exec_err is not None:
            return exec_err

        # Lazy master-gain (issue #986): ``masterlimiter`` появляется в
        # scsynth только через ~5с после старта sclang, а ``__init__``
        # отрабатывает раньше — отправляем /n_set на первом успешном exec.
        if not self._mgr._master_gain_applied:
            self._mgr._master_gain_applied = True
            self._mgr.set_master_gain(self._mgr._master_gain)

        if pattern_name:
            self._mgr._pattern_history[pattern_name] = code
            self._mgr._active_patterns.add(pattern_name)

        return self._execute_record_session_activity(code, quality_warnings)

    def _execute_sanitize(
        self, code: str,
    ) -> Tuple[Any, Optional[Dict[str, Any]]]:
        """Sanitize + ранний возврат ``(sanitized, None)`` или ``(None, error_dict)``.

        Renardo sanitizer (core/renardo_sanitizer) — единый seam:
        security → quality → slot-rewriter → pianovel→rhpiano → pattern
        length → amp-cap. Возвращаем ``error_dict`` для security/quality/
        slot, оркестратор делает ``return error_dict`` без вложенной
        логики. CC: 4-5 (3 short-circuit ветки + успех).
        """
        sanitized = renardo_sanitizer.sanitize_renando(
            code,
            self._mgr._max_amp,
            known_synths=self._mgr.known_synth_names(),
            pack1_loops_enabled=sample_loops.pack1_loops_enabled(),
        )
        if sanitized.security_error:
            return sanitized, {"success": False, "error": sanitized.security_error}
        if sanitized.quality_errors:
            return sanitized, {
                "success": False,
                "error": "⛔ Код отклонён музыкальным валидатором: "
                + " ".join(sanitized.quality_errors),
                "code": sanitized.code,
            }
        if sanitized.slot_error:
            return sanitized, {
                "success": False,
                "error": sanitized.slot_error,
                "code": sanitized.code,
            }
        return sanitized, None

    def _execute_emit_debug_log(self, code: str) -> None:
        """Debug-stderr dump ``FINAL CODE`` (live 15:44 «Error in Player: 'amp'»).

        Side-effect, без return. ``sys.stderr.write`` + flush, чтобы
        при падении exec'а полный код был в логе и можно было воспроизвести
        баг (KeyError('amp') в Players.py = event без ключа amp). CC: 2.
        """
        sys.stderr.write(
            f"🎵 [execute_music_code] FINAL CODE:\n{code}\n"
            f"🎵 [execute_music_code] FINAL CODE END (len={len(code)})\n"
        )
        sys.stderr.flush()

    def _execute_check_stack_health(self) -> Optional[Dict[str, Any]]:
        """Health + SC + Renardo check; ``error_dict`` или ``None``.

        Три штуки в одной: music-stack health (issue G-MUSIC, fail-fast
        на degraded sclang), SuperCollider-доступность, Renardo-доступность.
        Если хоть одна недоступна — ранний return с понятным сообщением.
        CC: 4 (3 проверки).
        """
        mgr = self._mgr
        if mgr._require_healthy and not mgr.is_music_stack_healthy():
            return mgr.music_stack_unavailable_error()

        if not mgr._check_supercollider():
            return {
                "success": False,
                "error": (
                    "SuperCollider не запущен. Запустите SuperCollider "
                    "перед воспроизведением музыки."
                ),
            }

        if not mgr._ensure_renardo_available():
            error = "Renardo недоступен."
            renardo_last_error = getattr(mgr, "_renardo_last_error", None)
            if renardo_last_error:
                error = f"{error} Последняя ошибка инициализации: {renardo_last_error}"
            return {"success": False, "error": error}

        return None

    def _execute_apply_segments_safety_net(
        self,
        code: str,
        *,
        segments: Optional[int],
        duration_sec: Optional[float],
    ) -> None:
        """Issue #990 — класть ``__total_*`` в контекст и взводить ``schedule_stop``.

        ``segments`` (bars) — основной путь, новые вызовы LLM. ``duration_sec``
        — backward-compat clamp (issue #949 → #990): ``__total_beats`` живёт
        для legacy-кода, но ``schedule_stop`` НЕ взводится (старый путь
        резал музыку на 6с, потому что LLM не знала реальный TTS-длину).
        CC: 6 (2 ветки + 5 присваиваний + 1 schedule).
        """
        mgr = self._mgr
        if segments is not None and int(segments) > 0:
            segments_i = max(1, min(int(segments), mgr.MAX_SEGMENTS))
            current_bpm = self.renardo_bpm()
            beats_per_bar = mgr.BEATS_PER_BAR
            total_beats = segments_i * beats_per_bar
            bar_duration_s = beats_per_bar * 60.0 / current_bpm
            mgr._renardo_context["__total_segments"] = segments_i
            mgr._renardo_context["__total_beats"] = total_beats
            mgr._renardo_context["__bpm"] = current_bpm
            mgr._renardo_context["__bar_duration"] = bar_duration_s
            self.schedule_stop(segments=segments_i, bpm=current_bpm)
        elif duration_sec is not None and duration_sec > 0:
            clamped = max(float(duration_sec), mgr.DEPRECATED_DURATION_SEC_CLAMP)
            current_bpm = self.renardo_bpm()
            total_beats = (clamped * current_bpm) / 60.0
            mgr._renardo_context["__total_beats"] = total_beats
            mgr._renardo_context["__duration_sec"] = clamped
            mgr._renardo_context["__bpm"] = current_bpm

    def _execute_run(
        self,
        code: str,
        has_clock_clear: bool,
        deferred_start: bool,
    ) -> Optional[Dict[str, Any]]:
        """``exec(code, ctx)`` + post-exec teardown старых нод.

        Если ``Clock.clear()`` есть в коде и НЕ внутри fade-обёртки (issue
        #3166), планируем teardown через ``_schedule_transition_cleanup``
        (issue #3137 anti-silence — раньше freeAll шёл СРАЗУ и давал окно
        тишины между треками; теперь откладывается почти до старта нового).
        CC: 5-7 (try/except + ветка deferred).
        """
        mgr = self._mgr
        try:
            exec(code, mgr._renardo_context)  # noqa: S102 — намеренный exec
        except Exception as exc:
            return {"success": False, "error": f"Ошибка выполнения: {exc}"}

        if has_clock_clear and not deferred_start:
            mgr._schedule_transition_cleanup(1)

        return None

    def _execute_record_session_activity(
        self,
        code: str,
        quality_warnings: list,
    ) -> Dict[str, Any]:
        """Session stamp (issue #935) + финальный success dict.

        ``_stamp_new_track`` (ADR-0141) делает под локом: ставит
        ``_last_music_activity_at``/``_music_session_active_since``,
        сбрасывает form-deadlines и бьёт ``_start_new_track_id``.
        CC: 4-5.
        """
        self._mgr._stamp_new_track()

        if quality_warnings:
            return {
                "success": True,
                "message": "Код выполнен успешно. ⚠️ " + " ".join(quality_warnings),
                "code": code,
                # ADR-0132: compose_music переписывает message своим текстом
                # и раньше эти предупреждения терял — отдаём их отдельно.
                "warnings": quality_warnings,
            }
        return {"success": True, "message": "Код выполнен успешно", "code": code}

    # ------------------------------------------------------------------
    # stop_pattern — валидация имени + call_player_stop
    # ------------------------------------------------------------------

    def stop_pattern(self, pattern_name: str) -> Dict[str, Any]:
        """Остановить именованный паттерн (RCE-safe, issue G-MUSIC).

        ``resolve_pattern_name`` валидирует имя по whitelist (builtin
        players d/p/s/l + ``_active_patterns``/``_pattern_history``), затем
        ``call_player_stop`` достаёт плеер из ``_renardo_context`` и зовёт
        ``.stop()`` — строка от LLM **никогда** не становится кодом.
        Issue G-MUSIC safety-net: даже при degraded Renardo/SC имя всё
        равно дропается из ``_active_patterns``, чтобы watchdog видел
        закрытие сессии.
        """
        mgr = self._mgr
        name_ok, name_error = self.resolve_pattern_name(pattern_name)
        if not name_ok:
            return {"success": False, "error": name_error}

        stop_error: Optional[str] = None
        degraded = mgr._require_healthy and not mgr.is_music_stack_healthy()

        if (
            not degraded
            and mgr._renardo_available
            and mgr._check_supercollider()
        ):
            try:
                self.call_player_stop(pattern_name)
            except Exception as exc:  # noqa: BLE001
                # Renardo may not know this player (e.g. we never started
                # it), or SC is degraded. Log and continue: we still want
                # to drop the pattern from our internal active set so the
                # watchdog sees that the session is over (issue #935).
                stop_error = f"Ошибка остановки паттерна: {exc}"

        mgr._active_patterns.discard(pattern_name)
        # Auto-close the music session if there are no patterns left
        # (issue #935).
        if not mgr._active_patterns:
            mgr._last_stop_at = time.monotonic()

        if stop_error:
            return {
                "success": False,
                "error": stop_error,
                "warning": (
                    "Паттерн исключён из active_patterns (issue #935 safety-net) "
                    f"несмотря на ошибку Renardo: {pattern_name}."
                ),
            }
        if degraded:
            return {
                "success": False,
                "error": (
                    "Музыка недоступна — Renardo в degraded-режиме. "
                    f"Локальное состояние для '{pattern_name}' всё равно "
                    "очищено чтобы не блокировать watchdog."
                ),
            }
        return {
            "success": True,
            "message": f"Паттерн '{pattern_name}' остановлен",
        }

    # ------------------------------------------------------------------
    # stop_all — оркестратор + 3 helpers
    # ------------------------------------------------------------------

    def stop_all(self) -> Dict[str, Any]:
        """Остановить всю музыку: ramp-down → freeAll + сброс сессии (issue #1000).

        Этапы (issue #1000 anti-click, общий путь с execute_code через
        ``_ramp_down_group``):
        1. ``.stop()`` на d/p/s/l 1-9.
        2. ``Clock.clear()``.
        3-4. ``_ramp_down_group(1)`` — gate=0 → release ADSR →
           ``/g_freeAll``. СИНХРОННО, в отличие от execute_code (там
           отложенный — там важна секунда тишины между треками).
        5. ``_end_music_session(now)`` (issue #3133) — сброс сессии.
        """
        mgr = self._mgr
        clock_error: Optional[str] = None
        degraded = mgr._require_healthy and not mgr.is_music_stack_healthy()

        if (
            not degraded
            and mgr._renardo_available
            and mgr._check_supercollider()
        ):
            teardown_error = self._stop_all_teardown()
            clock_error = teardown_error

        # Явный стоп — не «доиграл сам» (issue #3133): finished_track_id=None.
        mgr._end_music_session(time.monotonic())
        return self._stop_all_build_response(clock_error, degraded)

    def _stop_all_teardown(self) -> Optional[str]:
        """3-step teardown Renardo/SC. Возвращает ``clock_error`` или ``None``.

        Шаги 1-2 синхронные (per-player stop, Clock.clear), шаги 3-4
        делегированы ``_ramp_down_group`` (issue #3137, общий хелпер для
        execute_code и stop_all). CC: 4 (3 try/except + 1 method call).
        """
        mgr = self._mgr
        player_names = (
            [f"d{i}" for i in range(1, 10)]
            + [f"p{i}" for i in range(1, 10)]
            + [f"s{i}" for i in range(1, 10)]
            + [f"l{i}" for i in range(1, 10)]
        )

        # Шаг 1: остановить все плееры (best-effort)
        stop_code = "\n".join(
            f"try:\n  {name}.stop()\nexcept Exception:\n  pass"
            for name in player_names
        )
        try:
            exec(stop_code, mgr._renardo_context)  # noqa: S102
        except Exception:
            pass

        # Шаг 2: Clock.clear() — failures non-fatal for our state
        clock_error: Optional[str] = None
        try:
            exec("Clock.clear()", mgr._renardo_context)  # noqa: S102
        except Exception as exc:  # noqa: BLE001
            clock_error = f"Clock.clear() failed: {exc}"

        # Шаги 3-4: gate=0 ramp-down → freeAll (issue #3137, #1000).
        mgr._ramp_down_group(1)
        return clock_error

    def _stop_all_build_response(
        self,
        clock_error: Optional[str],
        degraded: bool,
    ) -> Dict[str, Any]:
        """Собрать dict-ответ для ``stop_all`` (CC: 3)."""
        if clock_error:
            return {
                "success": False,
                "error": clock_error,
                "warning": (
                    "Внутреннее состояние всё равно сброшено (issue #935 "
                    "safety-net): active_patterns=[], session_active=None."
                ),
            }
        if degraded:
            return {
                "success": False,
                "error": (
                    "Музыка недоступна — Renardo в degraded-режиме. "
                    "Локальное состояние (active_patterns, session_active) "
                    "всё равно сброшено (issue #935 safety-net)."
                ),
            }
        return {"success": True, "message": "Вся музыка остановлена"}

    # ------------------------------------------------------------------
    # Internal helpers — называния без подчёркивания в core/, чтобы не
    # маскировать public API; wrappers в ``MusicManager`` сохраняют
    # свои ``_``-имена для совместимости с тестами и внутренними
    # вызовами.
    # ------------------------------------------------------------------

    def call_player_stop(self, pattern_name: str) -> None:
        """Достать плеер из ``_renardo_context`` по имени и зовёт ``.stop()``.

        ``pattern_name`` уже прошёл :meth:`resolve_pattern_name` —
        имя гарантированно безопасное. Достаём объект через ``dict.get``
        (без ``exec``!) и зовём ``.stop()`` напрямую. Не-Renardo плеер
        или отсутствующий метод ``stop`` — no-op, без исключения наружу
        (это best-effort остановка, а не валидация).
        """
        player = self._mgr._renardo_context.get(pattern_name)
        if player is None:
            return
        stop = getattr(player, "stop", None)
        if callable(stop):
            stop()

    def prewarm_sample_buffers(self, code: str) -> None:
        """Предзагрузить sample-буферы для ``play("x-o-")`` ДО ``exec`` (issue #1815).

        Live 13.08: ``play("x-o-")`` стартовал в тот же тик, что и
        ``/b_allocRead`` — scsynth логировал ``Buffer UGen: no buffer data``
        и на старте музыки был резкий свист/xrun-бурст. Renardo кэширует
        буферы в ``Samples`` (BufferManager), поэтому предзагрузка до
        exec — cache-hit.

        Контракт issue #1815: ``-`` — ЗВУЧАЩИЙ символ (hyphen-каталог в
        0_foxdot_default/_/ и в 1_pitchglitch_samples/_/), ``.`` — настоящая
        пауза (каталога нет ни в одном сэмпл-паке), пробел — разделитель.
        Символы фильтруются внутри ``renardo_adapter.load_sample_buffers``
        (владелец плеера v2, ADR-0142). CC: 5 (try + 1 for + nested call).
        """
        try:
            samples = self._mgr._renardo_context.get("Samples")
            if samples is None:
                return
            for match in _PLAY_SYMBOLS_RE.finditer(code):
                renardo_adapter.load_sample_buffers(samples, match.group(1))
        except Exception:  # noqa: BLE001
            # Предзагрузка не должна ломать exec ни при каком раскладе.
            return

    def resolve_pattern_name(self, pattern_name: str) -> Tuple[bool, str]:
        """Валидация имени паттерна по whitelist (RCE-защита, issue G-MUSIC).

        Разрешены ТОЛЬКО:
        (а) builtin плееры Renardo (d1-d9, p1-p9, s1-s9, l1-l9),
        (б) имена, зарегистрированные через ``execute_code``
            (``_active_patterns`` ∪ ``_pattern_history``).

        Всё остальное (``__import__('os').system('id') #``, неидентификатор,
        неизвестное имя) — отказ. Раньше имя шло в
        ``f"{pattern_name}.stop()"`` и уходило в ``exec()`` (прямой RCE);
        теперь имя проверяется regex'ом, а плеер достаётся поиском по
        namespace-у Renardo, без сборки и выполнения кода.

        CC: 6 (1 regex + 1 set union + 2 ветки + 1 успех).
        """
        if not isinstance(pattern_name, str) or not _PATTERN_NAME_RE.match(
            pattern_name
        ):
            return False, (
                "Недопустимое имя паттерна — ожидается идентификатор "
                "вида 'p1' или 'bass'."
            )
        mgr = self._mgr
        known = (
            _RENARDO_PLAYER_NAMES
            | set(mgr._active_patterns)
            | set(mgr._pattern_history)
        )
        if pattern_name not in known:
            if mgr._active_patterns:
                available = ", ".join(sorted(mgr._active_patterns))
                return False, (
                    f"Неизвестный паттерн '{pattern_name}'. "
                    f"Активны: {available}."
                )
            return False, (
                f"Неизвестный паттерн '{pattern_name}' — "
                "активных паттернов нет."
            )
        return True, ""

    def renardo_bpm(self) -> float:
        """Текущий Renardo BPM с дефолтом 120 (issue #990 anchor).

        CC: 3 (try/except + 1 fallback).
        """
        try:
            clock = self._mgr._renardo_context.get("Clock", None)
            bpm = float(getattr(clock, "bpm", 120) or 120)
        except Exception:
            bpm = 120.0
        return bpm if bpm > 0 else 120.0

    def schedule_stop(self, *, segments: int, bpm: float) -> None:
        """Segments safety-net deadline (issue #990).

        Дедлайн — wall-clock backstop: обычно музыка останавливается на
        ``tts_batch_complete`` (dialogue_node → /mcp/music_cleanup →
        ``stop_music_on_session_end``). Если TTS-батч завис, watchdog
        ``auto_stop_idle_music`` бьёт ``stop_all`` при истечении дедлайна.

        Floor ``MIN_SEGMENTS_DEADLINE_SECONDS`` (60с) гарантирует, что
        LLM-эвристика «8 тактов @ 90bpm = 21.3s» не убьёт трек раньше
        конца песни (live 30.08 vision-pi 12:30: «сыграй короткий бит»
        → segments=8 при 90bpm → watchdog убил через 20с, а TTS ещё
        шёл 11с).
        """
        mgr = self._mgr
        bar_duration_s = mgr.BEATS_PER_BAR * 60.0 / max(1.0, float(bpm))
        timeout_s = max(
            segments * bar_duration_s * mgr.SEGMENTS_DEADLINE_SAFETY_FACTOR,
            mgr.MIN_SEGMENTS_DEADLINE_SECONDS,
        )
        mgr._music_deadline_at = time.monotonic() + timeout_s
        mgr._music_deadline_segments = int(segments)

    # ------------------------------------------------------------------
    # Form deadlines — issue #1812, #2461
    # ------------------------------------------------------------------

    def set_form_deadline(self, duration_seconds: float) -> None:
        """Записать момент конца одной формы ``repeat=False`` (issue #1812).

        Взводится из ``ComposeMusicTool`` сразу после успешного
        ``execute_code`` для трека без зацикливания. До дедлайна watchdog
        ``auto_stop_idle_music`` НЕ считает молчание диалога простоем
        (слушаем форму в тишине — ожидаемое использование).
        """
        self._mgr._music_form_deadline_at = time.monotonic() + max(
            0.0, float(duration_seconds)
        )

    def set_form_cycle_end(self, duration_seconds: float) -> None:
        """Момент конца ОДНОГО прохода формы, НЕЗАВИСИМО от ``repeat`` (issue #2461).

        В отличие от :meth:`set_form_deadline` (только ``repeat=False``,
        только watchdog-защита), это — общий «форма отыграла один раз»-
        канал, в т.ч. для DJ-сетов. Взводится безусловно на каждый
        успешный ``compose_music()``.
        """
        self._mgr._music_form_cycle_ends_at = time.monotonic() + max(
            0.0, float(duration_seconds)
        )

    def clear_form_deadline(self) -> None:
        """Снять защиту «форма ещё не доиграла» (issue #1812 + #2461).

        Зовётся из ``execute_code`` (новый код заменил старый) и из
        ``stop_all`` (явный стоп). Сбрасывает ОБА поля:
        ``_music_form_deadline_at`` (#1812) и ``_music_form_cycle_ends_at``
        (#2461) — оба описывают одну форму, оба теряют смысл на тех же
        двух точках (новый код / явный стоп).
        """
        self._mgr._music_form_deadline_at = None
        self._mgr._music_form_cycle_ends_at = None

    # ------------------------------------------------------------------
    # Watchdog — issue #935, #990, #1000, #1812, #2461
    # ------------------------------------------------------------------

    def auto_stop_idle_music(
        self,
        ttl_seconds: Optional[float] = None,
        now: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Watchdog: остановить музыку при простое диалога дольше TTL.

        Приоритеты (более специфичный контракт выигрывает):

        1. ``segments_deadline`` (issue #990) — если истёк, останавливаем
           немедленно (TTS-батч завис). DJ-режим игнорирует (DJ-сет
           непрерывен, segments=8 при 90bpm каждые 30-120с убивал бы
           сет посреди перехода).
        2. ``form_deadline`` (issue #1812) — если repeat=False форма ещё
           не доиграла, hold (молчание = слушаем форму, а не
           заброшенный диалог).
        3. ``idle_ttl`` (issue #935) — стоп через ``stop_all()`` и
           инкремент ``_auto_stop_count``.

        CC: ~10 (3 ветки + helper-calls). Декомпозиция не нужна, ≤12.
        """
        mgr = self._mgr
        ttl = mgr._auto_stop_ttl_seconds if ttl_seconds is None else float(ttl_seconds)
        now_m = time.monotonic() if now is None else float(now)
        result: Dict[str, Any] = {
            "stopped": False,
            "idle_seconds": None,
            "ttl_seconds": ttl,
            "active_patterns": list(mgr._active_patterns),
            "auto_stop_count": mgr._auto_stop_count,
        }
        # Fast path: no music activity recorded → nothing to auto-stop.
        # NOTE: deliberately *not* gating on _active_patterns — LLM may
        # have executed music code without a pattern_name (issue #935
        # regression), so _active_patterns can be empty while music IS
        # playing. We rely on _last_music_activity_at alone.
        if mgr._last_music_activity_at is None:
            return result
        idle = now_m - mgr._last_music_activity_at
        result["idle_seconds"] = idle

        # 1) segments_deadline — самый специфичный контракт, приоритет.
        deadline = mgr._music_deadline_at
        if deadline is not None and now_m >= deadline:
            if mgr.dj_mode_enabled:
                # DJ живёт по idle-TTL; сбросим дедлайн — следующий
                # переход продлит сессию.
                mgr._music_deadline_at = None
                mgr._music_deadline_segments = None
                return result
            segments_for_log = mgr._music_deadline_segments
            stop_result = self.stop_all()
            result["stopped"] = True
            result["stop_reason"] = "segments_deadline"
            result["deadline_segments"] = segments_for_log
            result["stop_result"] = stop_result
            mgr._auto_stop_count += 1
            result["auto_stop_count"] = mgr._auto_stop_count
            return result

        if idle < ttl:
            return result

        # 2) form_deadline — repeat=False форма ещё не доиграла.
        form_deadline = mgr._music_form_deadline_at
        if form_deadline is not None and now_m < form_deadline:
            result["held_reason"] = "form_not_finished"
            result["form_deadline_remaining_s"] = form_deadline - now_m
            return result

        # 3) idle_ttl — обычный watchdog.
        stop_result = self.stop_all()
        result["stopped"] = True
        result["stop_reason"] = "idle_ttl"
        result["stop_result"] = stop_result
        mgr._auto_stop_count += 1
        result["auto_stop_count"] = mgr._auto_stop_count
        return result

    def stop_music_on_session_end(self) -> Dict[str, Any]:
        """Hook DIALOGUE_END (issue #935).

        Idempotent: вызывается безусловно, в т.ч. когда музыки нет.
        Спасает от unnamed-паттернов (issue #935 regression:
        ``_active_patterns`` пуст, но музыка играет — старый safety net
        был blind к безымянным паттернам).
        """
        mgr = self._mgr
        was_active = mgr._music_session_active_since is not None
        stopped = list(mgr._active_patterns)  # may be empty (unnamed patterns)
        result = self.stop_all()
        return {
            "was_active": was_active,
            "stopped_patterns": stopped,
            "stop_result": result,
            "message": (
                f"Диалог завершился с активной музыкой "
                f"({len(stopped)} именованных, + безымянные паттерны). "
                "Автоматический stop_music сработал (issue #935)."
            )
            if was_active
            else (
                "Активной музыки не обнаружено — stop_all вызван "
                "профилактически (issue #935)."
            ),
        }
