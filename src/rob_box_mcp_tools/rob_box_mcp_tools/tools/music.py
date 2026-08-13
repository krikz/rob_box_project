#!/usr/bin/env python3
"""
music.py - Инструменты для управления музыкой в реальном времени через Renardo

Модуль предоставляет:
- MusicManager: Базовый класс управления Renardo (история паттернов, SC-проверка, пресеты, фильтрация кода)
- TrackLibrary: Персистентная медиатека треков (JSON на диске)
- ExecuteMusicCodeTool: Выполнить Renardo-код в безопасном контексте
- StopMusicTool: Остановить паттерны или всю музыку
- SetVibePresetTool: Применить вайб-пресет (скейл, темп, тоника)
- GetMusicStateTool: Получить текущее состояние музыки и историю паттернов
- SaveTrackTool: Сохранить трек в медиатеку
- ListTracksTool: Просмотреть треки медиатеки
- LoadTrackTool: Загрузить и воспроизвести сохранённый трек
- DeleteTrackTool: Удалить трек из медиатеки
"""

import json
import os
import re
import socket
import sqlite3
import struct
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from rob_box_voice.core.music_stack_validation import (
    MusicStackStatus,
    load_sclang_health,
)
from rob_box_voice.core.sc_only_custom_synthdefs import register_sc_only_custom_synthdefs

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType

# ---------------------------------------------------------------------------
# Safety filter — compiled once at import time
# ---------------------------------------------------------------------------

_BLOCKED_TOKENS = re.compile(
    r"\b("
    r"import|os|sys|subprocess|shutil|socket|requests|urllib|http|ftplib|"
    r"importlib|builtins|__import__|__builtins__|__class__|__subclasses__|"
    r"open|exec|eval|compile|globals|locals|vars|delattr"
    r")\b"
)


# ---------------------------------------------------------------------------
# MusicManager
# ---------------------------------------------------------------------------


class MusicManager:
    """Управляет интеграцией с Renardo для LLM-контроля музыки в реальном времени.

    Возможности:
    - Безопасное выполнение Renardo-кода (execute_code)
    - История паттернов с возможностью мутации и остановки по имени
    - Проверка доступности SuperCollider перед воспроизведением
    - Вайб-пресеты для быстрой настройки скейла / BPM / тоники
    - Фильтрация опасных системных команд в пользовательском коде
    """

    #: Доступные вайб-пресеты: имя -> {scale, bpm, root}
    #: root — целое число полутонов от C (C=0, D=2, E=4, F=5, G=7, A=9, B=11)
    VIBE_PRESETS: Dict[str, Dict[str, Any]] = {
        "chill":      {"scale": "major",       "bpm": 85,  "root": 0},   # C
        "energetic":  {"scale": "minor",       "bpm": 140, "root": 9},   # A
        "ambient":    {"scale": "dorian",      "bpm": 70,  "root": 2},   # D
        "jazz":       {"scale": "lydian",      "bpm": 120, "root": 5},   # F
        "dark":       {"scale": "phrygian",    "bpm": 100, "root": 4},   # E
        "rock":       {"scale": "minor",       "bpm": 120, "root": 4},   # E
        "latin":      {"scale": "dorian",      "bpm": 105, "root": 2},   # D
        "electronic": {"scale": "minor",       "bpm": 128, "root": 9},   # A
        "cinematic":  {"scale": "minor",       "bpm": 90,  "root": 0},   # C
        "funk":       {"scale": "mixolydian",  "bpm": 110, "root": 7},   # G
        "reggae":     {"scale": "major",       "bpm": 75,  "root": 7},   # G
        "classical":  {"scale": "major",       "bpm": 100, "root": 0},   # C
    }

    SC_HOST: str = "127.0.0.1"
    SC_PORT: int = 57110

    # ------------------------------------------------------------------
    # Issue #990 — segments safety-net contract
    # ------------------------------------------------------------------
    # The LLM must NOT pass duration_sec anymore (it cannot know the real
    # TTS duration — that was the root cause of music cutting off mid-song).
    # Instead it passes ``segments`` (number of bars) which is ONLY a
    # backstop: the system stops the music at tts_batch_complete; segments
    # caps playback only if the TTS batch hangs.
    #: 1 bar = 4 beats in Renardo's default meter.
    BEATS_PER_BAR: int = 4
    #: Floor for the segments deadline (seconds). A tiny LLM guess (e.g.
    #: segments=2) must not cut a real song off after 2 seconds — the
    #: deadline is a TTS-hang backstop, not a song-length contract.
    MIN_SEGMENTS_DEADLINE_SECONDS: float = 15.0
    #: Upper bound for accepted segments (guard against absurd values).
    MAX_SEGMENTS: int = 512
    #: Backward-compat clamp for the deprecated ``duration_sec`` param
    #: (#949 → #990). If an old LLM still passes duration_sec we only use it
    #: to define ``__total_beats`` so legacy generated code does not
    #: NameError, but we clamp it to at least this many seconds so it can
    #: never stop the music before the song ends. No stop is scheduled from
    #: duration_sec.
    DEPRECATED_DURATION_SEC_CLAMP: float = 60.0

    #: ``ROB_BOX_MUSIC_REQUIRE_HEALTHY=1`` → degraded sclang runtime blocks
    #: ``execute_music_code`` / ``set_vibe_preset`` instead of letting the LLM
    #: fight a broken Renardo stack. Defaults to ``False`` — music tools work
    #: in degraded mode (non-critical SynthDef parse errors don't block).
    #: Set to ``1`` to fail-fast on any sclang startup error.
    REQUIRE_HEALTHY_DEFAULT = False
    #: Critical SynthDefs the music subsystem depends on. Mirrors the list
    #: used by ``start_voice_assistant.sh`` — keep them in sync.
    DEFAULT_CRITICAL_SYNTHS: Tuple[str, ...] = (
        "strings",
        "wobblebass",
        "pianovel",
        "warmpad",
        "retrobass",
        "supersawlead",
        "imperialbrass",
        "marchstrings",
        "strangerpulsepad",
        "strangerarp",
        "strangerbrass",
    )

    def __init__(
        self,
        max_amp: float = 0.7,
        *,
        critical_synths: Optional[List[str]] = None,
        require_healthy: Optional[bool] = None,
        sclang_log_path: Optional[str] = None,
    ) -> None:
        #: максимальная амплитуда для любого паттерна (0.0-1.0)
        self._max_amp: float = max(0.0, min(1.0, max_amp))
        #: pattern_name -> последний выполненный код
        self._pattern_history: Dict[str, str] = {}
        #: множество имён активных паттернов
        self._active_patterns: set = set()
        #: имя текущего пресета
        self._current_preset: Optional[str] = None
        #: контекст выполнения для renardo
        self._renardo_context: Dict[str, Any] = {}
        #: True если renardo доступен, False/None иначе
        self._renardo_available: Optional[bool] = None
        #: Последняя ошибка инициализации renardo для диагностики
        self._renardo_last_error: Optional[str] = None
        #: Music-stack health snapshot (from ``load_sclang_health``). When
        #: ``is_healthy is False``, ``execute_music_code`` / ``set_vibe_preset``
        #: short-circuit with a clear "music unavailable" error so the LLM
        #: doesn't keep retrying against a broken Renardo/FoxDot upstream.
        self._music_stack_status: MusicStackStatus = MusicStackStatus(
            is_healthy=True,
            oscdef_registered=True,
            missing_synths=(),
            fatal_errors=(),
        )
        #: When True, ``execute_code`` / ``set_vibe_preset`` reject calls when
        #: ``_music_stack_status.is_healthy`` is False. Set False only for
        #: tests / dev environments where we explicitly want degraded mode.
        if require_healthy is None:
            env_flag = os.environ.get("ROB_BOX_MUSIC_REQUIRE_HEALTHY")
            if env_flag is None:
                self._require_healthy: bool = self.REQUIRE_HEALTHY_DEFAULT
            else:
                self._require_healthy: bool = env_flag.strip().lower() not in {"0", "false", "no", "off"}
        else:
            self._require_healthy = bool(require_healthy)
        #: Critical SynthDef set used by the boot-time health check.
        if critical_synths is None:
            env_synths = os.environ.get("ROB_BOX_MUSIC_CRITICAL_SYNTHS")
            if env_synths:
                self._critical_synths: Tuple[str, ...] = tuple(
                    name.strip() for name in env_synths.split(",") if name.strip()
                )
            else:
                self._critical_synths = self.DEFAULT_CRITICAL_SYNTHS
        else:
            self._critical_synths = tuple(critical_synths)
        # ------------------------------------------------------------------
        # Music session lifecycle tracking — issue #935
        # Tracks wall-clock timestamps for "music session" so a safety-net
        # watchdog can auto-stop music when the LLM forgets to call
        # ``stop_music`` after a rap/poem/spoken-word sequence, e.g. when
        # ``_MAX_TOOL_ITERATIONS=5`` is hit and the loop returns the last
        # spoken text without flushing stop_music.
        # ------------------------------------------------------------------
        # default 5 min — overridable via MUSIC_AUTO_STOP_TTL_SECONDS env
        default_ttl = 300
        try:
            env_ttl = int(os.environ.get("MUSIC_AUTO_STOP_TTL_SECONDS", str(default_ttl)))
            self._auto_stop_ttl_seconds: int = max(1, env_ttl)
        except (TypeError, ValueError):
            self._auto_stop_ttl_seconds = default_ttl
        # wall-clock timestamps — None until the first music activity in
        # the session. Stored as float seconds since epoch.
        self._music_session_active_since: Optional[float] = None
        self._last_music_activity_at: Optional[float] = None
        self._last_stop_at: Optional[float] = None
        # Issue #990 — segments safety-net deadline. Wall-clock monotonic
        # timestamp (from ``_schedule_stop``) after which the watchdog stops
        # music if the TTS batch never completed (no tts_batch_complete).
        # None = no deadline (music plays until tts_batch_complete / idle TTL).
        self._music_deadline_at: Optional[float] = None
        #: segments value that produced the deadline (diagnostics only).
        self._music_deadline_segments: Optional[int] = None
        # stats — surfaced via get_state() for the DialogCore safety-net
        self._auto_stop_count: int = 0
        # ------------------------------------------------------------------
        # Issue #1000 — DJ mode flag. When True, ``execute_code`` strips
        # ``Clock.future(outro/Clock.clear())`` patterns because the LLM
        # keeps planning its own stop (banned by contract #992) — only the
        # system clock + watchdog should stop DJ transitions.
        # ------------------------------------------------------------------
        self._dj_mode_enabled: bool = False
        # ------------------------------------------------------------------
        # Music stack health (issue G-MUSIC, architect review v3)
        # ------------------------------------------------------------------
        # If sclang already wrote a startup log and it's degraded, refuse to
        # initialize Renardo and surface a clear "music unavailable" error.
        # We do this BEFORE calling _initialize_renardo() so a broken
        # upstream .scd file cannot manifest as silent exec errors later.
        self._evaluate_music_stack_health(sclang_log_path=sclang_log_path)
        self._initialize_renardo()

    # ------------------------------------------------------------------
    # DJ Mode — issue #1000
    # ------------------------------------------------------------------

    @property
    def dj_mode_enabled(self) -> bool:
        """True when DJ mode is active — ``Clock.future`` stop patterns are stripped."""
        return self._dj_mode_enabled

    def set_dj_mode(self, enabled: bool) -> None:
        """Set DJ mode flag. Called by :class:`SetDjModeTool`."""
        self._dj_mode_enabled = bool(enabled)

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def _initialize_renardo(self) -> None:
        """Попытка инициализировать Renardo-контекст и загрузить SynthDef-ы в SC.

        Pipeline:
        1. Создаём директории семплов (иначе renardo_lib.runtime падает при импорте).
        2. Импортируем renardo_lib.runtime.
        3. Подключаемся к scsynth через Server.init_connection().
        4. Создаём Group 1 в scsynth через raw OSC (иначе /s_new падает).
        5. Загружаем все SynthDef-ы: sdef.add() → write(.scd) + load() →
           OSC /foxdot → sclang компилирует .scd → /d_recv → scsynth.
        6. Ждём 5 секунд пока sclang скомпилирует все 188 SynthDef-ов.

        NOTE: SynthDefs — это plain dict, НЕ объект с методом .reload()!
        Правильный способ: for sdef in SynthDefs.values(): sdef.add()
        """
        try:
            # renardo_lib.runtime при импорте пытается листить директории сэмплов.
            # Если 0_foxdot_default не установлен — падает FileNotFoundError.
            # Создаём пустую структуру директорий заранее, чтобы импорт проходил.
            import pathlib
            import shutil

            samples_base = pathlib.Path.home() / ".config" / "renardo" / "samples" / "0_foxdot_default"
            _SAMPLE_SUBDIRS = ["_", "_loop_"] + list("abcdefghijklmnopqrstuvwxyz")
            for subdir in _SAMPLE_SUBDIRS:
                (samples_base / subdir).mkdir(parents=True, exist_ok=True)

            # Renardo всегда ищет сэмплы ТОЛЬКО в 0_foxdot_default/ (sample_path_from_symbol
            # захардкожена на DEFAULT_SAMPLES_PACK_NAME). Буква 'c' (vokals) отсутствует
            # в foxdot_default, но есть в 1_pitchglitch_samples/c/.
            # Копируем отсутствующие файлы чтобы play("c   ") находило вокальные сэмплы.
            pitchglitch = pathlib.Path.home() / ".config" / "renardo" / "samples" / "1_pitchglitch_samples"
            if pitchglitch.exists():
                for letter in list("abcdefghijklmnopqrstuvwxyz"):
                    for case_dir in ("lower", "upper"):
                        src_dir = pitchglitch / letter / case_dir
                        dst_dir = samples_base / letter / case_dir
                        if not src_dir.exists():
                            continue
                        dst_dir.mkdir(parents=True, exist_ok=True)
                        dst_wavs = set(f.name for f in dst_dir.glob("*.wav"))
                        for wav in src_dir.glob("*.wav"):
                            if wav.name not in dst_wavs:
                                shutil.copy2(wav, dst_dir / wav.name)

            # renardo_lib само по себе пустое; нужен renardo_lib.runtime
            import renardo_lib.runtime as _rt

            # Подключаемся к scsynth (Server.booted = True после этого)
            if not _rt.Server.booted:
                _rt.Server.init_connection()

            # Создаём Group 1 в scsynth — renardo отправляет все ноты в эту группу.
            # Без неё scsynth возвращает "Group 1 not found" на каждый /s_new.
            self._send_osc_raw("/g_new", 1, 0, 0)

            # Загружаем все SynthDef-ы через sclang.
            # SynthDefs — это plain Python dict, НЕ объект с .reload()!
            # sdef.add() = write(.scd файл на диск) + load() (отправляет путь
            # через OSC /foxdot → sclang → компилирует → /d_recv → scsynth)
            # 🔴 FIX (live 12.08): 188 sdef.add() залпом роняют UDP-буфер sclang
            # (drops >500 в /proc/net/udp) — часть SynthDef-ов (pads, bass, karp,
            # bell...) не доезжает до scsynth → "SynthDef not found" → ТИШИНА.
            # Пейсинг 0.1с между отправками + верификация с досылкой пропавших.
            for idx, sdef in enumerate(_rt.SynthDefs.values()):
                sdef.add()
                if idx % 5 == 4:
                    time.sleep(0.1)

            # Загружаем эффекты (reverb/volume) — иначе scsynth отвечает
            # "SynthDef reverb not found" / "SynthDef volume not found" на каждый
            # Player с room=/amp-fx и музыка молчит (live 05.08: все e2e-прогоны
            # после деплоя тихие, TTS работает, музыка нет).
            # EffectManager.reload() = effect.load() для каждого эффекта +
            # In() + Out() (служебные bus-ноды).
            try:
                _rt.effect_manager.reload()
            except Exception as exc:  # noqa: BLE001
                self._renardo_last_error = f"effect_manager.reload failed: {exc}"

            # Ждём компиляции всех 188 SynthDef-ов через sclang.
            # Без паузы renardo сразу пытается играть, scsynth отвечает "not found".
            time.sleep(5)

            # 🔴 FIX (live 12.08): верификация — пробуем /s_new на критичные
            # синты и досылаем пропавшие через sdef.add() (до 3 раундов).
            # Без этого музыка тихо молчит при "SynthDef not found".
            self._verify_and_retry_synthdefs(_rt, self._send_osc_raw)

            self._renardo_context = vars(_rt).copy()
            register_sc_only_custom_synthdefs(_rt, self._renardo_context)
            self._renardo_available = True
            self._renardo_last_error = None
        except (ImportError, Exception) as exc:
            self._renardo_available = False
            self._renardo_context = {}
            self._renardo_last_error = str(exc)

    def _verify_and_retry_synthdefs(
        self,
        _rt: Any,
        _send_osc_raw: Any,
        max_rounds: int = 3,
    ) -> None:
        """Verify critical SynthDefs exist in scsynth; re-send missing ones.

        live 12.08: после бурста sdef.add() часть SynthDef-ов пропадает
        (UDP drops на 57120). Пробуем /s_new для каждого критичного синта,
        пропавшие досылаем через sdef.add() → /foxdot → sclang → /d_recv.

        Args:
            _rt: renardo_lib.runtime module.
            _send_osc_raw: callable для отправки OSC на scsynth.
            max_rounds: сколько раундов досылки пробовать.
        """
        import struct as _struct
        import time as _time

        _CRITICAL_SYNTHS = (
            "pads", "bass", "bell", "blip", "fuzz", "gong", "karp",
            "dub", "pluck", "space", "epiano", "saw", "varsaw", "square",
            "ambi", "faim", "marimba", "sitar", "viola", "noise",
            "scatter", "orient", "creep", "shaker",
            "strings", "wobblebass", "brass", "organ", "tb303",
            "play1", "play2",
        )

        def _probe_missing(names):
            """Return subset of names whose SynthDef is absent in scsynth."""
            missing: list[str] = []
            probe_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            probe_sock.settimeout(0.4)
            for i, name in enumerate(names):
                node_id = 9000 + i
                msg = bytearray(b"/s_new\x00")
                types = b",siii\x00\x00"
                name_b = name.encode() + b"\x00"
                while len(name_b) % 4:
                    name_b += b"\x00"
                msg.extend(types)
                msg.extend(name_b)
                msg.extend(_struct.pack(">iii", node_id, 0, 1))
                try:
                    probe_sock.sendto(bytes(msg), (self.SC_HOST, self.SC_PORT))
                    data, _ = probe_sock.recvfrom(512)
                    if b"not found" in data:
                        missing.append(name)
                except socket.timeout:
                    pass  # синт создан — def на месте
            probe_sock.close()
            return missing

        missing = list(_CRITICAL_SYNTHS)
        for round_no in range(max_rounds):
            missing = _probe_missing(missing)
            if not missing:
                return
            self._log_warning(
                f"[music] round {round_no + 1}: missing SynthDefs: {missing} — re-sending"
            )
            for name in missing:
                try:
                    sdef = _rt.SynthDefs.get(name)
                except Exception:  # noqa: BLE001
                    continue
                if sdef is None:
                    continue
                try:
                    sdef.add()
                except Exception:  # noqa: BLE001
                    continue
                _time.sleep(0.3)
            _time.sleep(5)  # время на компиляцию
        self._log_warning(
            f"[music] SynthDefs still missing after {max_rounds} rounds: {missing}"
        )

    def _log_warning(self, message: str) -> None:
        """Log via the manager's logger when available (fallback to print)."""
        logger = getattr(self, "_logger", None)
        if logger is not None:
            try:
                logger.warning(message)
                return
            except Exception:  # noqa: BLE001
                pass
        import sys as _sys
        _sys.stderr.write(f"{message}\n")
        _sys.stderr.flush()

    def _ensure_renardo_available(self) -> bool:
        """Retry Renardo initialization when a previous startup attempt failed.

        This avoids a permanent degraded state when container startup races cause
        the first one-shot initialization to fail before scsynth/sclang are fully ready.
        """

        if self._renardo_available:
            return True

        self._initialize_renardo()
        return bool(self._renardo_available)

    # ------------------------------------------------------------------
    # Music-stack health (issue G-MUSIC, architect review v3)
    # ------------------------------------------------------------------

    def _evaluate_music_stack_health(
        self,
        sclang_log_path: Optional[str] = None,
    ) -> MusicStackStatus:
        """Snapshot sclang health from the startup log and mark the manager.

        When ``is_healthy is False`` AND ``_require_healthy`` is True, this
        will also clear ``_renardo_available`` (without touching
        ``_renardo_last_error``) so downstream tools see consistent state.

        Args:
            sclang_log_path: Override log location. Falls back to
                ``SCLANG_LOG_PATH`` env var, then ``/tmp/sclang.log``.

        Returns:
            The :class:`MusicStackStatus` that was applied.
        """

        status = load_sclang_health(
            sclang_log_path,
            critical_synths=list(self._critical_synths),
        )
        self._music_stack_status = status

        if not status.is_healthy and self._require_healthy:
            # Mark Renardo as unavailable WITHOUT clearing the existing
            # last_error (which might be informative for diagnostics). The
            # operator should see both "music stack degraded" AND any
            # subsequent renardo init failure that follows.
            self._renardo_available = False

        return status

    def is_music_stack_healthy(self) -> bool:
        """True if the sclang startup log was healthy at the last check."""

        return bool(self._music_stack_status.is_healthy)

    def music_stack_unavailable_error(self) -> Dict[str, str]:
        """Build a stable error payload for ``music unavailable`` replies.

        Used by ``execute_code`` / ``set_vibe_preset`` / ``stop_music`` so
        the LLM gets a single, recognizable error message rather than
        a different string for each entry-point.
        """

        status = self._music_stack_status
        details: List[str] = []
        if status.fatal_errors:
            details.append("; ".join(status.fatal_errors[:3]))
        if status.missing_synths:
            details.append(f"missing SynthDefs: {', '.join(status.missing_synths)}")
        detail_str = (" — " + "; ".join(details)) if details else ""
        log_path = os.environ.get("SCLANG_LOG_PATH", "/tmp/sclang.log")
        return {
            "success": False,
            "error": (
                "Музыка недоступна: sclang стартовал в degraded-режиме "
                "(syntax error в startup-логе Renardo/FoxDot)"
                f"{detail_str}. См. {log_path}."
            ),
        }

    # ------------------------------------------------------------------
    # SuperCollider check
    # ------------------------------------------------------------------

    def _check_supercollider(self) -> bool:
        """Проверить, запущен ли SuperCollider, отправив OSC /status по UDP.

        scsynth слушает на UDP-порту SC_PORT. Отправляем минимальный OSC
        /status запрос и ждём ответа. TCP-проверка не подходит — scsynth
        по умолчанию принимает только UDP.

        Returns:
            True если scsynth отвечает на SC_PORT.
        """
        # Минимальный OSC /status: "/status\0" (8 байт) + ",\0\0\0" (4 байта)
        osc_status = b"/status\x00,\x00\x00\x00"
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.settimeout(1.0)
                sock.sendto(osc_status, (self.SC_HOST, self.SC_PORT))
                data, _ = sock.recvfrom(512)
                return len(data) > 0
        except OSError:
            return False

    def _send_osc_raw(self, address: str, *args: Any) -> None:
        """Отправить raw OSC сообщение на scsynth (UDP 57110).

        OSC-packet собирается вручную: 4-byte aligned address + type-tag +
        big-endian args. Поддерживает int (``i``) и float (``f``) аргументы.

        Выделено как self-метод вместо замыкания, чтобы можно было
        переиспользовать из ``execute_music_code`` / ``stop_all`` без
        дублирования byte-packing логики. Раньше ``execute_music_code``
        собирал ``/g_new`` руками bytearray-ом, что (а) дублировало код
        и (б) легко ломалось при изменении формата.

        Issue #778 (deployment critical_log ``FAILURE IN SERVER /g_new
        negative node IDs are reserved``): между ``/g_freeAll`` и
        ``/g_new`` нужна пауза ≥50ms — UDP fire-and-forget, scsynth не
        успевает освободить ID Group 1, и Renardo Player-ы присылают
        ``/s_new`` с target_id=1, которого ещё нет. Пауза лечит race
        condition без изменения семантики (свободные ноды умирают
        сами, мы просто даём scsynth обработать free до пересоздания
        Group).
        """
        msg = bytearray()
        addr_bytes = address.encode() + b"\x00"
        while len(addr_bytes) % 4:
            addr_bytes += b"\x00"
        types = b"," + b"".join(b"i" if isinstance(a, int) else b"f" for a in args) + b"\x00"
        while len(types) % 4:
            types += b"\x00"
        msg.extend(addr_bytes)
        msg.extend(types)
        for a in args:
            if isinstance(a, int):
                msg.extend(struct.pack(">i", a))
            elif isinstance(a, float):
                msg.extend(struct.pack(">f", a))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(bytes(msg), (self.SC_HOST, self.SC_PORT))

    # ------------------------------------------------------------------
    # Code safety filter
    # ------------------------------------------------------------------

    def _filter_code(self, code: str) -> Tuple[bool, str]:
        """Проверить код на наличие опасных конструкций.

        Args:
            code: Строка кода для проверки.

        Returns:
            (is_safe, error_message) — (True, "") если код безопасен.
        """
        match = _BLOCKED_TOKENS.search(code)
        if match:
            return False, f"Запрещённый токен в коде: '{match.group()}'"
        return True, ""

    def _cap_amp(self, code: str) -> str:
        """Ограничить громкость/октаву в коде до безопасных пределов.

        Issue #1000 — phase-3.2 anti-click caps:
        - ``amp=0.9``               → ``amp=0.7`` (если max_amp=0.7)
        - ``amp=P[0.5, 1.0]``       → ``amp=P[0.5, 0.7]``
        - ``amp=1``                 → ``amp=0.7``
        - ``amplify=var([1,0.3])``  → ``amplify=var([0.7,0.3])``
        - ``amplify=0.8``           → ``amplify=0.7``
        - ``oct=5``                 → ``oct=4`` (макс 4 — oct=5 очень резкое/громкое)
        """
        max_amp = self._max_amp
        max_oct = 4  # oct=5 и выше слишком резкое/громкое (issue #1000)

        # 1. Сначала P[...] паттерны (более специфичный случай)
        def _cap_p(m: re.Match) -> str:
            def _cap_num(n: re.Match) -> str:
                return f"{min(float(n.group()), max_amp):.3g}"
            return "amp=P[" + re.sub(r"\b\d+(?:\.\d*)?\b", _cap_num, m.group(1)) + "]"

        code = re.sub(r"amp\s*=\s*P\[([^\]]+)\]", _cap_p, code)

        # 2. amp= простые числа
        def _cap_n(m: re.Match) -> str:
            return f"amp={min(float(m.group(1)), max_amp):.3g}"

        code = re.sub(r"amp\s*=\s*(\d+(?:\.\d*)?)", _cap_n, code)

        # 3. amplify=var([...]) — ограничиваем числа внутри var() (issue #1000)
        def _cap_amplify_var(m: re.Match) -> str:
            inner = m.group(1)
            def _cap_num(n: re.Match) -> str:
                return f"{min(float(n.group()), max_amp):.3g}"
            inner = re.sub(r"\b\d+(?:\.\d*)?\b", _cap_num, inner)
            return f"amplify=var({inner})"

        code = re.sub(r"amplify\s*=\s*var\(([^)]+)\)", _cap_amplify_var, code)

        # 4. amplify= простые числа
        def _cap_amplify_n(m: re.Match) -> str:
            return f"amplify={min(float(m.group(1)), max_amp):.3g}"

        code = re.sub(r"amplify\s*=\s*(\d+(?:\.\d*)?)", _cap_amplify_n, code)

        # 5. oct= — ограничиваем до max_oct (issue #1000)
        def _cap_oct(m: re.Match) -> str:
            return f"oct={min(int(m.group(1)), max_oct)}"

        code = re.sub(r"oct\s*=\s*(\d+)", _cap_oct, code)
        return code

    # ------------------------------------------------------------------
    # Issue #990 — segments safety-net
    # ------------------------------------------------------------------

    def _renardo_bpm(self) -> float:
        """Current Renardo BPM (default 120 when Clock is unavailable)."""
        try:
            clock = self._renardo_context.get("Clock", None)
            bpm = float(getattr(clock, "bpm", 120) or 120)
        except Exception:
            bpm = 120.0
        return bpm if bpm > 0 else 120.0

    def _schedule_stop(self, *, segments: int, bpm: float) -> None:
        """Set the segments safety-net deadline (issue #990).

        The deadline is a wall-clock backstop only: the system normally
        stops the music at ``tts_batch_complete`` (dialogue_node →
        ``/mcp/music_cleanup`` → ``stop_music_on_session_end``). If the TTS
        batch hangs (or the batch_complete event is lost), the mcp_server
        watchdog calls ``auto_stop_idle_music`` and stops music once the
        deadline passes, so it cannot play forever.

        A floor (``MIN_SEGMENTS_DEADLINE_SECONDS``) guarantees a tiny LLM
        guess cannot cut a real song off prematurely.
        """
        bar_duration_s = self.BEATS_PER_BAR * 60.0 / max(1.0, float(bpm))
        timeout_s = max(segments * bar_duration_s, self.MIN_SEGMENTS_DEADLINE_SECONDS)
        self._music_deadline_at = time.monotonic() + timeout_s
        self._music_deadline_segments = int(segments)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def execute_code(
        self,
        code: str,
        pattern_name: Optional[str] = None,
        *,
        segments: Optional[int] = None,
        duration_sec: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Безопасно выполнить Renardo-код.

        Перед выполнением проверяется:
        1. Фильтр опасных конструкций.
        2. Доступность SuperCollider.
        3. Доступность библиотеки Renardo.

        Args:
            code: Строка Python/Renardo-кода.
            pattern_name: Имя паттерна для хранения в истории (опционально).
            segments: Количество тактов (баров, 1 бар = 4 бита) — ТОЛЬКО
                предохранитель (issue #990). Если задан, в контекст Renardo
                добавляются переменные ``__total_beats`` (segments * 4),
                ``__total_segments``, ``__bpm``, ``__bar_duration``, и
                устанавливается дедлайн ``_schedule_stop`` — watchdog
                остановит музыку, если TTS-батч завис. Музыка ВСЕГДА живёт
                до ``tts_batch_complete``; segments лишь ограничивает время
                игры при зависшем TTS.
            duration_sec: DEPRECATED (#949 → #990). Игнорируется для
                остановки музыки. Оставлен только для обратной
                совместимости: ``__total_beats`` определяется из него со
                сдвигом вверх (clamp ≥ 60s), чтобы старый код не падал с
                NameError, но и не мог оборвать музыку раньше конца песни.

        Returns:
            dict с ключами ``success``, ``message`` (или ``error``), ``code``.
        """
        is_safe, filter_error = self._filter_code(code)
        if not is_safe:
            return {"success": False, "error": filter_error}

        # 🔴 FIX (live 11:41 «цоканье»): автозамена pianovel/piano → rhpiano
        # (обе используют MdaPiano физмодель — цокает/щёлкает; rhpiano —
        # компилируемый и чистый). LLM продолжает писать pianovel несмотря
        # на запрет в промпте → защита на уровне кода. Взято из ветки
        # phase-3.2-music-testing (проверено в live-экспериментах юзера).
        if "pianovel" in code:
            code = code.replace("pianovel", "rhpiano")
        # piano заменяем только если это отдельное слово (не rhpiano, pianovel и т.д.)
        code = re.sub(r"(?<![a-zA-Z])piano(?![a-zA-Z])", "rhpiano", code)

        # Ограничиваем amp до максимально допустимого значения
        code = self._cap_amp(code)

        # 🔴 DEBUG (live 15:44 «Error in Player: 'amp'»): полный код ПОСЛЕ
        # всех трансформаций (pianovel→rhpiano, amp-caps) — чтобы видеть,
        # что реально уходит в renardo. Ошибка KeyError('amp') в Players.py
        # означает, что в event плеера нет ключа amp — нужен полный код
        # для воспроизведения.
        import sys as _sys
        _sys.stderr.write(
            f"🎵 [execute_music_code] FINAL CODE:\n{code}\n"
            f"🎵 [execute_music_code] FINAL CODE END (len={len(code)})\n"
        )
        _sys.stderr.flush()

        # Issue G-MUSIC: short-circuit before sending anything to Renardo if
        # the sclang startup log shows the music stack is degraded. This
        # prevents the LLM from retrying code that will keep failing because
        # of an upstream-renardo syntax error in a .scd file.
        if self._require_healthy and not self.is_music_stack_healthy():
            return self.music_stack_unavailable_error()

        if not self._check_supercollider():
            return {
                "success": False,
                "error": "SuperCollider не запущен. Запустите SuperCollider перед воспроизведением музыки.",
            }

        if not self._ensure_renardo_available():
            error = "Renardo недоступен."
            renardo_last_error = getattr(self, "_renardo_last_error", None)
            if renardo_last_error:
                error = f"{error} Последняя ошибка инициализации: {renardo_last_error}"
            return {
                "success": False,
                "error": error,
            }

        # Если код содержит Clock.clear() — СНАЧАЛА выполняем код (регистрируем новые
        # паттерны), ПОТОМ посылаем /g_freeAll чтобы убить старые SC-ноды.
        #
        # Порядок важен для бесшовного перехода:
        # 1. exec(code): Clock.clear() + новые паттерны зарегистрированы (мгновенно)
        # 2. /g_freeAll: убиваем старые SC-синтезаторы (они играли пока LLM думал)
        # 3. /g_new: пересоздаём Group 1 для новых нот
        # При таком порядке нет тишины — новые паттерны уже ждут следующего beat,
        # а /g_freeAll только убивает СТАРЫЕ ноды, которые overlap не нужен.
        #
        # Почему нужен /g_freeAll: Clock.clear() останавливает планировщик Renardo,
        # но НЕ посылает /g_freeAll в scsynth. Уже запущенные синтезаторы живут
        # до конца sus-конверта. После многих переходов 1024-нодовая таблица SC
        # забивается → "too many nodes" / "negative node IDs" → тишина.
        has_clock_clear = "Clock.clear()" in code

        # Issue #990: the music lifecycle is owned by the system
        # (tts_batch_complete → music_cleanup → stop_music_on_session_end).
        # The LLM must pass ``segments`` (bars) as a *safety net* only: the
        # deadline below stops music if the TTS batch hangs. The old
        # ``duration_sec`` contract (#949) is deprecated — it is clamped and
        # never used to schedule an early stop (the LLM cannot know the real
        # TTS duration; that was the root cause of music cutting off at 6s
        # while the song was 51s).
        if segments is not None and int(segments) > 0:
            segments_i = max(1, min(int(segments), self.MAX_SEGMENTS))
            current_bpm = self._renardo_bpm()
            beats_per_bar = self.BEATS_PER_BAR
            total_beats = segments_i * beats_per_bar
            bar_duration_s = beats_per_bar * 60.0 / current_bpm
            self._renardo_context["__total_segments"] = segments_i
            self._renardo_context["__total_beats"] = total_beats
            self._renardo_context["__bpm"] = current_bpm
            self._renardo_context["__bar_duration"] = bar_duration_s
            self._schedule_stop(segments=segments_i, bpm=current_bpm)
        elif duration_sec is not None and duration_sec > 0:
            # Backward compat (#949 → #990): keep the context variables
            # alive so legacy generated code referencing __total_beats does
            # not NameError — but clamp the value so an LLM guess (e.g. 6.0s)
            # can never stop the music before the song ends. No stop is
            # scheduled from duration_sec.
            clamped = max(float(duration_sec), self.DEPRECATED_DURATION_SEC_CLAMP)
            current_bpm = self._renardo_bpm()
            total_beats = (clamped * current_bpm) / 60.0
            self._renardo_context["__total_beats"] = total_beats
            self._renardo_context["__duration_sec"] = clamped
            self._renardo_context["__bpm"] = current_bpm

        try:
            exec(code, self._renardo_context)  # noqa: S102
        except Exception as exc:
            return {"success": False, "error": f"Ошибка выполнения: {exc}"}

        if has_clock_clear:
            # Убиваем старые SC-ноды ПОСЛЕ того как новые паттерны зарегистрированы
            try:
                self._send_osc_raw("/g_freeAll", 1)
            except Exception:
                pass  # если SC недоступен — не критично, старые ноды умрут сами
            # Пауза между /g_freeAll и /g_new обязательна (issue #778):
            # UDP — fire-and-forget, scsynth обрабатывает /g_freeAll
            # асинхронно и не освобождает ID Group 1 мгновенно. Без паузы
            # наш /g_new (или любой /s_new от Renardo Player-а, который
            # попытается вставить ноту в Group 1) приходит в scsynth, когда
            # ID ещё занят → "FAILURE IN SERVER /g_new negative node IDs are
            # reserved" в логах supercollider (старый баг, был
            # замаскирован тем, что renardo при инициализации сначала
            # пересоздаёт Group, и в среднем прокатывало). 50ms достаточно
            # для scsynth обработать free и освободить ID.
            try:
                time.sleep(0.05)
            except Exception:
                pass
            # Пересоздаём Group 1 — renardo всегда отправляет ноты в эту группу
            try:
                self._send_osc_raw("/g_new", 1, 0, 0)
            except Exception:
                pass

        if pattern_name:
            self._pattern_history[pattern_name] = code
            self._active_patterns.add(pattern_name)

        # Stamp music-session lifecycle (issue #935). Always mark activity
        # when code executes successfully — even without pattern_name — so
        # the safety nets (dialogue-end hook + watchdog) can stop music that
        # the LLM started but didn't name.
        now = time.monotonic()
        self._last_music_activity_at = now
        if self._music_session_active_since is None:
            self._music_session_active_since = now

        return {"success": True, "message": "Код выполнен успешно", "code": code}

    def stop_pattern(self, pattern_name: str) -> Dict[str, Any]:
        """Остановить именованный паттерн.

        Не требует наличия паттерна в истории — LLM может вызвать stop для
        любого player (d1, p1, ...) даже если execute_code не сохранял по имени.

        Issue G-MUSIC: even when the sclang startup is degraded we still drop
        ``pattern_name`` from ``_active_patterns`` (no live SC nodes to worry
        about), but we tell the caller that music is unavailable so the LLM
        can short-circuit further tool calls.

        Args:
            pattern_name: Имя паттерна/плеера (d1, p1, bass и т.д.).

        Returns:
            dict с ключами ``success`` и ``message`` (или ``error``).
        """
        stop_code = f"{pattern_name}.stop()"
        stop_error: Optional[str] = None
        degraded = self._require_healthy and not self.is_music_stack_healthy()

        if not degraded and self._renardo_available and self._check_supercollider():
            try:
                exec(stop_code, self._renardo_context)  # noqa: S102
            except Exception as exc:  # noqa: BLE001
                # Renardo may not know this player (e.g. we never started it),
                # or SC is degraded. Log and continue: we still want to drop
                # the pattern from our internal active set so the watchdog
                # sees that the session is over (issue #935 safety-net).
                stop_error = f"Ошибка остановки паттерна: {exc}"

        self._active_patterns.discard(pattern_name)
        # Auto-close the music session if there are no patterns left (issue #935).
        if not self._active_patterns:
            self._last_stop_at = time.monotonic()
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
                    f"Локальное состояние для '{pattern_name}' всё равно очищено "
                    "чтобы не блокировать watchdog."
                ),
            }
        return {"success": True, "message": f"Паттерн '{pattern_name}' остановлен"}

    def stop_all(self) -> Dict[str, Any]:
        """Остановить всю музыку: плавный gate=0 ramp-down → freeAll.

        Issue #1000 (phase-3.2 anti-click):
            Hard ``/g_freeAll`` без ramp-down даёт щелчки на MdaPiano/rhpiano
            (физ-модели — release-фаза ADSR не успевает затухнуть). Делаем
            ``amp=0`` на всех плеерах → release → ~50ms → ``/g_freeAll``.

        Этапы:
        1. ``amp=0`` на всех живых плеерах (d1-d9, p1-p9, s1-s9, l1-l9) —
           запускает release-фазу ADSR, синты затухают естественно.
        2. ``Clock.clear()`` — убрать все запланированные события.
        3. ~50ms sleep — дать ADSR release затухнуть.
        4. OSC ``/g_freeAll`` — убить все живые синтезаторы в scsynth.

        Returns:
            dict с ключами ``success`` и ``message`` (или ``error``).
        """
        # Track Clock.clear() failures so we can warn the operator while
        # still tearing down our internal session state (issue #935).
        clock_error: Optional[str] = None
        degraded = self._require_healthy and not self.is_music_stack_healthy()

        if not degraded and self._renardo_available and self._check_supercollider():
            # 🔴 FIX (live 15:44 «Error in Player: 'amp'»): ramp-down через
            # ``{name}.amp = 0`` УБРАН. Renardo Player.__setattr__ оборачивает
            # любое присваивание в asStream() → attr["amp"] становится PGroup,
            # а не скаляром → get_event() строит event с PGroup-amp →
            # send_osc_message не находит скаляр → KeyError('amp') на каждом
            # кадре → музыка мертва (рэп/Бах/DJ — всё) с деплоя 15:26, когда
            # влился 3cc04a0c. Останавливаем плееры только через .stop()
            # (как работало в 12:27), без трюка с amp.
            player_names = (
                [f"d{i}" for i in range(1, 10)]
                + [f"p{i}" for i in range(1, 10)]
                + [f"s{i}" for i in range(1, 10)]
                + [f"l{i}" for i in range(1, 10)]
            )

            # Шаг 1: остановить все плееры
            stop_code = "\n".join(
                f"try:\n  {name}.stop()\nexcept Exception:\n  pass"
                for name in player_names
            )
            try:
                exec(stop_code, self._renardo_context)  # noqa: S102
            except Exception:
                pass  # best-effort, продолжаем

            # Шаг 2: очистить Clock
            try:
                exec("Clock.clear()", self._renardo_context)  # noqa: S102
            except Exception as exc:  # noqa: BLE001
                # Clock.clear() failure is non-fatal for our internal state —
                # the patterns are still held in Renardo's namespace, but
                # ``/g_freeAll`` below terminates the live synths and we
                # still need to reset our own lifecycle fields. Issue #935.
                clock_error = f"Clock.clear() failed: {exc}"

            # Шаг 3: ~50ms на release ADSR (issue #1000 anti-click)
            try:
                time.sleep(0.05)
            except Exception:
                pass

            # Шаг 4: убить все синтезаторы в SuperCollider (/g_freeAll на Group 1)
            try:
                self._send_osc_raw("/g_freeAll", 1)
            except Exception:
                pass  # если SC недоступен — не страшно, Clock уже очищен

        self._active_patterns.clear()
        self._last_stop_at = time.monotonic()
        # Issue #990 — a stop cancels the segments safety-net deadline:
        # music is no longer playing, so there is nothing to backstop.
        self._music_deadline_at = None
        self._music_deadline_segments = None
        # Reset session only when the *whole* session is over so a partial
        # ``stop_pattern``-then-restart sequence doesn't lose the timer
        # (issue #935 — keeps audit trail of when music was active).
        self._music_session_active_since = None
        self._last_music_activity_at = None
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

    def set_vibe_preset(self, preset_name: str) -> Dict[str, Any]:
        """Применить вайб-пресет (скейл, BPM, тоника).

        Если Renardo/SC недоступны, пресет запоминается для отложенного применения.

        Args:
            preset_name: Ключ из VIBE_PRESETS.

        Returns:
            dict с ключами ``success``, ``message``, ``preset`` (или ``error``).
        """
        if preset_name not in self.VIBE_PRESETS:
            available = ", ".join(self.VIBE_PRESETS)
            return {
                "success": False,
                "error": f"Пресет '{preset_name}' не найден. Доступные: {available}",
            }

        preset = self.VIBE_PRESETS[preset_name]
        self._current_preset = preset_name

        # Issue G-MUSIC: in degraded mode there's no point sending
        # ``Clock.bpm = …`` / ``Scale.default = …`` to Renardo — sclang
        # may not even have compiled the required SynthDefs. Refuse and
        # tell the operator / LLM to fall back to speech-only mode.
        if self._require_healthy and not self.is_music_stack_healthy():
            err = self.music_stack_unavailable_error()
            err["warning"] = (
                "Пресет не применён — Renardo в degraded-режиме. "
                "Проверьте /tmp/sclang.log и сообщите оператору."
            )
            return err

        if self._renardo_available and self._check_supercollider():
            # Root.default принимает целое число (полутонов от C) или строку "C"
            # Root.C и подобные атрибуты НЕ существуют в Renardo!
            preset_code = (
                f"Clock.bpm = {preset['bpm']}\n"
                f"Scale.default = Scale.{preset['scale']}\n"
                f"Root.default = {preset['root']}"
            )
            try:
                exec(preset_code, self._renardo_context)  # noqa: S102
            except Exception as exc:
                return {"success": False, "error": f"Ошибка применения пресета: {exc}"}

        return {
            "success": True,
            "message": f"Пресет '{preset_name}' применён: scale={preset['scale']}, bpm={preset['bpm']}, root={preset['root']}",
            "preset": preset,
        }

    def get_state(self) -> Dict[str, Any]:
        """Вернуть текущее состояние музыкального менеджера.

        Returns:
            dict с полями renardo_available, supercollider_running,
            current_preset, pattern_history, active_patterns.
        """
        supercollider_running = self._check_supercollider()
        if not self._renardo_available and supercollider_running:
            self._ensure_renardo_available()

        return {
            "renardo_available": self._renardo_available,
            "supercollider_running": supercollider_running,
            "current_preset": self._current_preset,
            "pattern_history": dict(self._pattern_history),
            "active_patterns": list(self._active_patterns),
            "renardo_last_error": getattr(self, "_renardo_last_error", None),
            # Music-stack health snapshot — surfaced so the LLM can see
            # ``music_stack_healthy: false`` and avoid retrying calls that
            # will be rejected by ``execute_music_code`` / ``set_vibe_preset``.
            "music_stack_healthy": self._music_stack_status.is_healthy,
            "music_stack_oscdef_registered": self._music_stack_status.oscdef_registered,
            "music_stack_missing_synths": list(self._music_stack_status.missing_synths),
            "music_stack_fatal_errors": list(self._music_stack_status.fatal_errors),
            "music_stack_require_healthy": self._require_healthy,
            # ---- music session lifecycle (issue #935) ----
            # ``music_session_active_since`` is monotonic seconds since first
            # ``execute_code`` after the most recent ``stop_all``. ``None``
            # when no music session is currently open.
            # ``last_music_activity_at`` is the most recent timestamp that a
            # pattern was started/restarted. ``auto_stop_ttl_seconds`` is the
            # configured idle threshold used by ``auto_stop_idle_music``.
            "music_session_active_since": self._music_session_active_since,
            "last_music_activity_at": self._last_music_activity_at,
            "last_stop_at": self._last_stop_at,
            "auto_stop_ttl_seconds": self._auto_stop_ttl_seconds,
            "auto_stop_count": self._auto_stop_count,
            # Issue #990 — segments safety-net deadline (None = no deadline).
            "music_deadline_at": self._music_deadline_at,
            "music_deadline_segments": self._music_deadline_segments,
            "idle_seconds": (
                time.monotonic() - self._last_music_activity_at
                if self._last_music_activity_at is not None
                else None
            ),
        }

    # ------------------------------------------------------------------
    # Music session cleanup — safety-net for issue #935
    # ------------------------------------------------------------------
    def auto_stop_idle_music(
        self,
        ttl_seconds: Optional[float] = None,
        now: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Auto-stop music if no activity for ``ttl_seconds``.

        The DialogCore / watchdog should call this periodically (e.g. once
        per second, or once per turn boundary). If music is currently
        active AND the time since the last ``execute_code`` exceeds the
        configured TTL, this method calls ``stop_all()`` and increments
        ``auto_stop_count`` for diagnostics. It is **safe to call
        arbitrarily often**: when there is no music session, or the TTL
        has not been exceeded, it is a no-op.

        Args:
            ttl_seconds: Idle threshold (default: ``self._auto_stop_ttl_seconds``).
            now: Override for ``time.monotonic()`` (used in tests).

        Returns:
            dict with keys ``stopped`` (bool), ``idle_seconds`` (float | None),
            ``ttl_seconds`` (float), ``active_patterns`` (list[str]),
            ``auto_stop_count`` (int).
        """
        ttl = self._auto_stop_ttl_seconds if ttl_seconds is None else float(ttl_seconds)
        now_m = time.monotonic() if now is None else float(now)
        result: Dict[str, Any] = {
            "stopped": False,
            "idle_seconds": None,
            "ttl_seconds": ttl,
            "active_patterns": list(self._active_patterns),
            "auto_stop_count": self._auto_stop_count,
        }
        # Fast path: no music activity recorded → nothing to auto-stop.
        # NOTE: deliberately *not* gating on _active_patterns — the LLM
        # may have executed music code without a pattern_name (issue #935
        # regression), so _active_patterns can be empty while music IS
        # playing.  We rely on _last_music_activity_at alone.
        if self._last_music_activity_at is None:
            return result
        idle = now_m - self._last_music_activity_at
        result["idle_seconds"] = idle
        # Issue #990 — segments safety-net: if the LLM passed ``segments``
        # and the deadline has passed, the TTS batch likely hung (no
        # tts_batch_complete → no music_cleanup). Stop music so it cannot
        # play forever. This takes priority over the idle TTL because the
        # deadline is the more precise contract the LLM asked for.
        # 🔴 FIX (live 10:13 DJ): при активном DJ-режиме дедлайн
        # ИГНОРИРУЕТСЯ — DJ-сет непрерывен (переходы каждые 30-120с),
        # segments-дедлайн #990 (~30с) убивал музыку посреди сета.
        # DJ-флаг приходит из mcp_server (подписка на /voice/dj_mode).
        deadline = self._music_deadline_at
        if deadline is not None and now_m >= deadline:
            if getattr(self, "_dj_active", False):
                # DJ живёт по idle-TTL; сбросим дедлайн — следующий
                # переход продлит сессию.
                self._music_deadline_at = None
                self._music_deadline_segments = None
                return result
            stop_result = self.stop_all()
            result["stopped"] = True
            result["stop_reason"] = "segments_deadline"
            result["stop_result"] = stop_result
            self._auto_stop_count += 1
            result["auto_stop_count"] = self._auto_stop_count
            return result
        if idle < ttl:
            return result
        # Auto-stop — call the existing stop_all() so the closure logic
        # (3-stage clean: per-player stop + Clock.clear() + /g_freeAll)
        # is reused as-is.
        stop_result = self.stop_all()
        result["stopped"] = True
        result["stop_result"] = stop_result
        self._auto_stop_count += 1
        result["auto_stop_count"] = self._auto_stop_count
        return result

    def stop_music_on_session_end(self) -> Dict[str, Any]:
        """Force-stop all music when the dialogue ends.

        Convenience hook for DialogCore / dialogue_node to call on
        DIALOGUE_END. Always calls ``stop_all()`` unconditionally — the
        LLM may have started music without a ``pattern_name``, in which
        case ``_active_patterns`` is empty but music IS playing (issue #935
        regression: safety net was blind to unnamed patterns).

        ``stop_all()`` is idempotent and safe to call even when nothing is
        playing.

        Returns:
            dict with keys ``was_active`` (bool), ``stopped_patterns``
            (list[str]), ``message`` (str).
        """
        was_active = self._music_session_active_since is not None
        stopped = list(self._active_patterns)  # may be empty (unnamed patterns)
        result = self.stop_all()
        return {
            "was_active": was_active,
            "stopped_patterns": stopped,
            "stop_result": result,
            "message": (
                f"Диалог завершился с активной музыкой ({len(stopped)} именованных, "
                f"+ безымянные паттерны). Автоматический stop_music сработал (issue #935)."
            ) if was_active else (
                "Активной музыки не обнаружено — stop_all вызван профилактически (issue #935)."
            ),
        }


# ---------------------------------------------------------------------------
# MCPTool wrappers
# ---------------------------------------------------------------------------


class ExecuteMusicCodeTool(MCPTool):
    """Выполнить Renardo/FoxDot-код для создания или изменения музыкального паттерна."""

    def __init__(self, node, manager: MusicManager) -> None:
        super().__init__(node)
        self._manager = manager

    @property
    def name(self) -> str:
        return "execute_music_code"

    @property
    def description(self) -> str:
        return (
            "Выполнить Renardo-код для создания или изменения музыкального паттерна в реальном времени. "
            "Код выполняется в контексте Renardo (FoxDot-совместимый синтаксис). "
            "Пример: 'p1 >> pluck([0, 2, 4], dur=0.5, amp=0.8)'. "
            "Перед выполнением проверяется доступность SuperCollider. "
            "Опасные системные команды автоматически блокируются. "
            "Укажи pattern_name чтобы паттерн можно было остановить или изменить позже."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="code",
                type="string",
                description="Строка Python/Renardo-кода для выполнения. Например: 'p1 >> pluck([0, 2, 4])'",
                required=True,
            ),
            MCPToolParameter(
                name="pattern_name",
                type="string",
                description=(
                    "Имя паттерна для хранения в истории (например: 'p1', 'bass', 'drums'). "
                    "Используется для последующей мутации или остановки паттерна."
                ),
                required=False,
            ),
            MCPToolParameter(
                name="segments",
                type="integer",
                description=(
                    "Сколько тактов (баров) должна играть фоновая музыка "
                    "(1 бар = 4 бита). Это ТОЛЬКО предохранитель: система "
                    "сама останавливает музыку после tts_batch_complete, "
                    "segments лишь ограничивает время игры, если TTS завис. "
                    "Для песни обычно 8-16 тактов. Если не знаешь — НЕ "
                    "указывай (дефолт: музыка играет до конца озвучки). #990"
                ),
                required=False,
            ),
            MCPToolParameter(
                name="duration_sec",
                type="number",
                description=(
                    "DEPRECATED (#990) — игнорируется для остановки музыки, "
                    "оставлен для обратной совместимости. НЕ используй. "
                    "Вместо него передавай segments."
                ),
                required=False,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def execute(
        self,
        code: str,
        pattern_name: Optional[str] = None,
        segments: Optional[int] = None,
        duration_sec: Optional[float] = None,
    ) -> MCPToolResult:
        """Выполнить Renardo-код (#990: segments как предохранитель)."""
        self.log_info(f"Выполнение музыкального кода: {code[:80]}...")
        result = self._manager.execute_code(
            code, pattern_name, segments=segments, duration_sec=duration_sec
        )
        if result["success"]:
            # Issue 989 Fix C: немедленно сообщаем audio_node, что музыка
            # активна — VAD threshold поднимается без ожидания watchdog.
            self._notify_music_state()
            return MCPToolResult(success=True, data=result, message=result["message"])
        return MCPToolResult(success=False, error=result["error"])

    def _notify_music_state(self) -> None:
        """Опубликовать /voice/music/state на сервере (issue 989 Fix C)."""
        if self.node is None:
            return
        publisher = getattr(self.node, "publish_music_state", None)
        if publisher is None:
            return
        try:
            publisher()
        except Exception as exc:  # noqa: BLE001
            self.log_warning(f"Не удалось опубликовать music_state: {exc}")


class StopMusicTool(MCPTool):
    """Остановить музыкальный паттерн по имени или всю музыку сразу."""

    def __init__(self, node, manager: MusicManager) -> None:
        super().__init__(node)
        self._manager = manager

    @property
    def name(self) -> str:
        return "stop_music"

    @property
    def description(self) -> str:
        return (
            "Остановить музыкальный паттерн по имени или всю музыку. "
            "Если указан pattern_name — остановится только этот паттерн. "
            "Если pattern_name не указан или равен 'all' — остановится вся музыка (Clock.clear())."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="pattern_name",
                type="string",
                description=(
                    "Имя паттерна для остановки (например: 'p1', 'bass'). "
                    "Передай 'all' или оставь пустым для остановки всей музыки."
                ),
                required=False,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def execute(self, pattern_name: Optional[str] = None) -> MCPToolResult:
        """Остановить паттерн или всю музыку."""
        if not pattern_name or pattern_name.strip().lower() == "all":
            self.log_info("Остановка всей музыки")
            result = self._manager.stop_all()
        else:
            self.log_info(f"Остановка паттерна: {pattern_name}")
            result = self._manager.stop_pattern(pattern_name)

        if result["success"]:
            # Issue 989 Fix C: немедленно сообщаем audio_node, что музыка
            # остановлена — VAD threshold возвращается к обычному.
            self._notify_music_state()
            return MCPToolResult(success=True, data=result, message=result["message"])
        return MCPToolResult(success=False, error=result["error"])

    def _notify_music_state(self) -> None:
        """Опубликовать /voice/music/state на сервере (issue 989 Fix C)."""
        if self.node is None:
            return
        publisher = getattr(self.node, "publish_music_state", None)
        if publisher is None:
            return
        try:
            publisher()
        except Exception as exc:  # noqa: BLE001
            self.log_warning(f"Не удалось опубликовать music_state: {exc}")


class SetVibePresetTool(MCPTool):
    """Применить вайб-пресет для быстрой настройки скейла, BPM и тоники."""

    PRESET_NAMES: List[str] = list(MusicManager.VIBE_PRESETS.keys())

    def __init__(self, node, manager: MusicManager) -> None:
        super().__init__(node)
        self._manager = manager

    @property
    def name(self) -> str:
        return "set_vibe_preset"

    @property
    def description(self) -> str:
        presets_desc = ", ".join(
            f"{name} (scale={p['scale']}, bpm={p['bpm']})" for name, p in MusicManager.VIBE_PRESETS.items()
        )
        return (
            "Применить вайб-пресет для быстрой настройки музыкального контекста. "
            "Устанавливает скейл, BPM и тонику в Renardo одной командой. "
            f"Доступные пресеты: {presets_desc}. "
            "Устанавливает: Clock.bpm, Scale.default, Root.default (целое число полутонов от C)."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="preset_name",
                type="string",
                description="Имя пресета для применения",
                required=True,
                enum=self.PRESET_NAMES,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def execute(self, preset_name: str) -> MCPToolResult:
        """Применить вайб-пресет."""
        self.log_info(f"Применение пресета: {preset_name}")
        result = self._manager.set_vibe_preset(preset_name)
        if result["success"]:
            return MCPToolResult(success=True, data=result, message=result["message"])
        return MCPToolResult(success=False, error=result["error"])


class GetMusicStateTool(MCPTool):
    """Получить текущее состояние музыки: доступность Renardo/SC, активные паттерны, история."""

    def __init__(self, node, manager: MusicManager) -> None:
        super().__init__(node)
        self._manager = manager

    @property
    def name(self) -> str:
        return "get_music_state"

    @property
    def description(self) -> str:
        return (
            "Получить текущее состояние музыкального менеджера: "
            "доступность Renardo и SuperCollider, список активных паттернов, "
            "историю кода паттернов и применённый пресет."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def read_only(self) -> bool:
        return True

    @property
    def destructive(self) -> bool:
        return False

    def execute(self) -> MCPToolResult:
        """Вернуть текущее состояние музыки."""
        state = self._manager.get_state()
        parts = [
            f"Renardo: {'доступен' if state['renardo_available'] else 'недоступен'}",
            f"SuperCollider: {'запущен' if state['supercollider_running'] else 'не запущен'}",
            f"Пресет: {state['current_preset'] or 'не задан'}",
            f"Активные паттерны: {', '.join(state['active_patterns']) or 'нет'}",
            f"История паттернов: {', '.join(state['pattern_history'].keys()) or 'нет'}",
        ]
        return MCPToolResult(
            success=True,
            data=state,
            message="\n".join(parts),
        )


# ---------------------------------------------------------------------------
# TrackLibrary — персистентная медиатека треков (SQLite)
# ---------------------------------------------------------------------------

from pathlib import Path as _Path

# Migration file lookup: Docker mounts repo migrations/ at /migrations,
# dev/build environments find it relative to the package root.
_MIGRATION_FILE = _Path("/migrations/004_music_library.sql")
if not _MIGRATION_FILE.exists():
    _MIGRATION_FILE = _Path(__file__).resolve().parents[4] / "migrations" / "004_music_library.sql"


class TrackLibrary:
    """Персистентная SQLite медиатека треков.

    Использует ту же БД что и VoiceMemory/WaypointStore (VOICE_MEMORY_DB_PATH).
    Миграция ``004_music_library.sql`` применяется идемпотентно при инициализации
    (CREATE TABLE IF NOT EXISTS + INSERT OR IGNORE для bootstrap-трека).

    Thread-safe: все публичные методы используют ``self._lock``.

    Args:
        db_path: Путь к SQLite-файлу. По умолчанию — из VOICE_MEMORY_DB_PATH
                 или ``/data/voice_memory.db``.
    """

    def __init__(self, db_path: Optional[str] = None) -> None:
        self._db_path = db_path or os.getenv("VOICE_MEMORY_DB_PATH", "/data/voice_memory.db")
        self._lock = threading.Lock()

        os.makedirs(os.path.dirname(self._db_path) or ".", exist_ok=True)
        self._conn = sqlite3.connect(self._db_path, check_same_thread=False)
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.execute("PRAGMA foreign_keys=ON")
        self._conn.row_factory = sqlite3.Row
        self._apply_migration()

    # ------------------------------------------------------------------
    # Migration
    # ------------------------------------------------------------------

    def _apply_migration(self) -> None:
        """Применить 004_music_library.sql идемпотентно (IF NOT EXISTS + INSERT OR IGNORE)."""
        if not _MIGRATION_FILE.exists():
            raise FileNotFoundError(
                f"Миграция не найдена: {_MIGRATION_FILE}. "
                "Проверьте volume монтирование migrations/ в docker-compose."
            )
        ddl = _MIGRATION_FILE.read_text(encoding="utf-8")
        with self._lock:
            self._conn.executescript(ddl)
            self._conn.commit()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _slug(name: str) -> str:
        return re.sub(r"[^a-z0-9_]", "_", name.lower().strip())

    @staticmethod
    def _row_to_dict(row: sqlite3.Row, include_code: bool = True) -> Dict[str, Any]:
        d = dict(row)
        d["tags"] = json.loads(d.get("tags") or "[]")
        if not include_code:
            d.pop("code", None)
        return d

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def save_track(
        self,
        name: str,
        code: str,
        title: str = "",
        description: str = "",
        tags: Optional[List[str]] = None,
        rating: int = 0,
        notes: str = "",
    ) -> Dict[str, Any]:
        """Сохранить или обновить трек в медиатеке (INSERT OR REPLACE).

        Args:
            name: Уникальный идентификатор (slug, без пробелов).
            code: Renardo-код трека.
            title: Читаемое название.
            description: Описание трека.
            tags: Список тегов.
            rating: Оценка 0-5.
            notes: Личные заметки о треке.

        Returns:
            dict ``success``, ``message``, ``name``.
        """
        slug = self._slug(name)
        if not slug:
            return {"success": False, "error": "Некорректное имя трека"}

        ts = datetime.now(timezone.utc).isoformat()
        tags_json = json.dumps(tags or [], ensure_ascii=False)
        rating = max(0, min(5, rating))

        with self._lock:
            # Сохраняем created_at существующей записи если она есть
            row = self._conn.execute(
                "SELECT created_at FROM music_tracks WHERE name = ?", (slug,)
            ).fetchone()
            created_at = row["created_at"] if row else ts
            action = "обновлён" if row else "сохранён"

            self._conn.execute(
                """
                INSERT INTO music_tracks
                    (name, title, code, description, tags, rating, notes,
                     play_count, created_at, updated_at)
                VALUES (?, ?, ?, ?, ?, ?, ?, COALESCE(
                    (SELECT play_count FROM music_tracks WHERE name = ?), 0
                ), ?, ?)
                ON CONFLICT(name) DO UPDATE SET
                    title       = excluded.title,
                    code        = excluded.code,
                    description = excluded.description,
                    tags        = excluded.tags,
                    rating      = excluded.rating,
                    notes       = excluded.notes,
                    updated_at  = excluded.updated_at
                """,
                (slug, title or name, code, description, tags_json,
                 rating, notes, slug, created_at, ts),
            )
            self._conn.commit()

        return {"success": True, "message": f"Трек '{slug}' {action}", "name": slug}

    def list_tracks(self, tag: Optional[str] = None, min_rating: int = 0) -> Dict[str, Any]:
        """Вернуть список треков с фильтрацией (без поля code).

        Args:
            tag: Фильтр по тегу (опционально).
            min_rating: Минимальный рейтинг (0-5).

        Returns:
            dict ``success``, ``tracks`` (list of dicts), ``total``.
        """
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT name, title, description, tags, rating, notes,
                       play_count, created_at, updated_at
                FROM music_tracks
                WHERE rating >= ?
                ORDER BY rating DESC, name ASC
                """,
                (min_rating,),
            ).fetchall()

        tracks = [self._row_to_dict(r, include_code=False) for r in rows]
        if tag:
            tracks = [t for t in tracks if tag in t.get("tags", [])]
        return {"success": True, "tracks": tracks, "total": len(tracks)}

    def load_track(self, name: str) -> Dict[str, Any]:
        """Получить код трека и инкрементировать play_count.

        Args:
            name: Имя трека.

        Returns:
            dict ``success``, ``code``, ``track`` (метаданные без code) или ``error``.
        """
        slug = self._slug(name)
        with self._lock:
            row = self._conn.execute(
                "SELECT * FROM music_tracks WHERE name = ?", (slug,)
            ).fetchone()

            if not row:
                names = [r[0] for r in self._conn.execute(
                    "SELECT name FROM music_tracks ORDER BY name"
                ).fetchall()]
                available = ", ".join(names) or "библиотека пуста"
                return {"success": False, "error": f"Трек '{slug}' не найден. Доступны: {available}"}

            self._conn.execute(
                "UPDATE music_tracks SET play_count = play_count + 1 WHERE name = ?",
                (slug,),
            )
            self._conn.commit()

        full = self._row_to_dict(row, include_code=True)
        code = full.pop("code")
        full["play_count"] += 1  # reflect incremented value
        return {"success": True, "code": code, "track": full}

    def delete_track(self, name: str) -> Dict[str, Any]:
        """Удалить трек из медиатеки.

        Args:
            name: Имя трека.

        Returns:
            dict ``success``, ``message`` или ``error``.
        """
        slug = self._slug(name)
        with self._lock:
            cur = self._conn.execute(
                "DELETE FROM music_tracks WHERE name = ? RETURNING name", (slug,)
            )
            deleted = cur.fetchone()
            self._conn.commit()

        if not deleted:
            return {"success": False, "error": f"Трек '{slug}' не найден"}
        return {"success": True, "message": f"Трек '{slug}' удалён из медиатеки"}


# ---------------------------------------------------------------------------
# MCP Tool wrappers для TrackLibrary
# ---------------------------------------------------------------------------


class SaveTrackTool(MCPTool):
    """Сохранить Renardo-трек в персистентную медиатеку робота."""

    def __init__(self, node, library: TrackLibrary, manager: MusicManager) -> None:
        super().__init__(node)
        self._library = library
        self._manager = manager

    @property
    def name(self) -> str:
        return "save_track"

    @property
    def description(self) -> str:
        return (
            "Сохранить Renardo-трек в медиатеку робота для повторного воспроизведения. "
            "Если code не передан — сохраняется код последнего выполненного паттерна из истории. "
            "Медиатека хранится в /config/music_library.json (персистентно между перезапусками). "
            "Используй для сохранения понравившихся треков, заготовок или процедурных шедевров."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="name",
                type="string",
                description="Уникальное имя-идентификатор трека (slug, например: 'csm_chill_v2')",
                required=True,
            ),
            MCPToolParameter(
                name="code",
                type="string",
                description=(
                    "Renardo-код трека. Если не передан — берётся из истории паттернов "
                    "по ключу pattern_name или последний выполненный код."
                ),
                required=False,
            ),
            MCPToolParameter(
                name="title",
                type="string",
                description="Читаемое название трека (например: 'Night Drive в C minor')",
                required=False,
            ),
            MCPToolParameter(
                name="description",
                type="string",
                description="Описание трека: настроение, структура, особенности",
                required=False,
            ),
            MCPToolParameter(
                name="tags",
                type="array",
                description="Список тегов (например: ['chill', 'minor', '90bpm', 'full_track'])",
                required=False,
            ),
            MCPToolParameter(
                name="rating",
                type="integer",
                description="Оценка трека от 0 до 5",
                required=False,
            ),
            MCPToolParameter(
                name="notes",
                type="string",
                description="Личные заметки о треке",
                required=False,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def execute(
        self,
        name: str,
        code: Optional[str] = None,
        title: str = "",
        description: str = "",
        tags: Optional[List[str]] = None,
        rating: int = 0,
        notes: str = "",
    ) -> MCPToolResult:
        """Сохранить трек."""
        # Если код не передан — берём из истории MusicManager
        if not code:
            history = self._manager.get_state().get("pattern_history", {})
            if history:
                # Берём первый ключ (обычно 'full_track' или последнее сохранённое)
                code = next(iter(history.values()))
            else:
                return MCPToolResult(success=False, error="Код не передан и история паттернов пуста")

        result = self._library.save_track(
            name=name,
            code=code,
            title=title,
            description=description,
            tags=tags,
            rating=rating,
            notes=notes,
        )
        if result["success"]:
            return MCPToolResult(success=True, data=result, message=result["message"])
        return MCPToolResult(success=False, error=result["error"])


class ListTracksTool(MCPTool):
    """Просмотреть медиатеку треков с опциональной фильтрацией по тегам и рейтингу."""

    def __init__(self, node, library: TrackLibrary) -> None:
        super().__init__(node)
        self._library = library

    @property
    def name(self) -> str:
        return "list_tracks"

    @property
    def description(self) -> str:
        return (
            "Просмотреть все треки в медиатеке робота. "
            "Можно фильтровать по тегу или минимальному рейтингу. "
            "Показывает: название, теги, рейтинг, количество воспроизведений, заметки."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="tag",
                type="string",
                description="Фильтр по тегу (например: 'full_track', 'chill', 'robot_authored')",
                required=False,
            ),
            MCPToolParameter(
                name="min_rating",
                type="integer",
                description="Показать только треки с рейтингом не ниже указанного (0-5)",
                required=False,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def read_only(self) -> bool:
        return True

    @property
    def destructive(self) -> bool:
        return False

    def execute(self, tag: Optional[str] = None, min_rating: int = 0) -> MCPToolResult:
        """Вернуть список треков."""
        result = self._library.list_tracks(tag=tag, min_rating=min_rating)
        tracks = result["tracks"]
        if not tracks:
            return MCPToolResult(success=True, data=result, message="Медиатека пуста (нет подходящих треков)")

        lines = [f"Медиатека: {result['total']} трек(ов)\n"]
        for t in tracks:
            stars = "★" * t["rating"] + "☆" * (5 - t["rating"]) if t["rating"] else "☆☆☆☆☆"
            tags_str = " ".join(f"[{tag}]" for tag in t["tags"]) if t["tags"] else ""
            lines.append(f"• {t['name']} — {t['title']} {stars}")
            if t["description"]:
                lines.append(f"  {t['description']}")
            if tags_str:
                lines.append(f"  {tags_str}  ▶ сыграно: {t['play_count']}x")
            if t["notes"]:
                lines.append(f"  📝 {t['notes']}")
        return MCPToolResult(success=True, data=result, message="\n".join(lines))


class LoadTrackTool(MCPTool):
    """Загрузить трек из медиатеки и воспроизвести его."""

    def __init__(self, node, library: TrackLibrary, manager: MusicManager) -> None:
        super().__init__(node)
        self._library = library
        self._manager = manager

    @property
    def name(self) -> str:
        return "load_track"

    @property
    def description(self) -> str:
        return (
            "Загрузить трек из медиатеки и воспроизвести его через Renardo. "
            "Трек идентифицируется по имени (slug). "
            "Используй list_tracks чтобы узнать доступные имена. "
            "Счётчик воспроизведений обновляется автоматически."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="name",
                type="string",
                description="Имя трека для загрузки (slug, например: 'csm_132_full_track')",
                required=True,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def execute(self, name: str) -> MCPToolResult:
        """Загрузить и воспроизвести трек."""
        load_result = self._library.load_track(name)
        if not load_result["success"]:
            return MCPToolResult(success=False, error=load_result["error"])

        code = load_result["code"]
        track_meta = load_result["track"]
        self.log_info(f"Загрузка трека '{name}' из медиатеки")

        play_result = self._manager.execute_code(code, pattern_name=name)
        if not play_result["success"]:
            return MCPToolResult(success=False, error=f"Трек загружен но не запущен: {play_result['error']}")

        return MCPToolResult(
            success=True,
            data={"track": track_meta, "play_count": track_meta.get("play_count", 1)},
            message=(
                f"▶ Воспроизводится: {track_meta.get('title', name)} "
                f"(сыграно {track_meta.get('play_count', 1)}x)"
            ),
        )


class DeleteTrackTool(MCPTool):
    """Удалить трек из медиатеки."""

    def __init__(self, node, library: TrackLibrary) -> None:
        super().__init__(node)
        self._library = library

    @property
    def name(self) -> str:
        return "delete_track"

    @property
    def description(self) -> str:
        return (
            "Удалить трек из медиатеки робота. "
            "Действие необратимо. "
            "Используй list_tracks чтобы уточнить имя перед удалением."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="name",
                type="string",
                description="Имя трека для удаления (slug)",
                required=True,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return True

    def execute(self, name: str) -> MCPToolResult:
        """Удалить трек."""
        result = self._library.delete_track(name)
        if result["success"]:
            return MCPToolResult(success=True, data=result, message=result["message"])
        return MCPToolResult(success=False, error=result["error"])


# ---------------------------------------------------------------------------
# SearchSamplesTool — filesystem search over Renardo sample packs
# ---------------------------------------------------------------------------

# Default samples path (same as MusicSkill._DEFAULT_SAMPLES_PATH)
_SEARCH_SAMPLES_DEFAULT_PATH = os.environ.get(
    "RENARDO_SAMPLES_PATH",
    "/root/.config/renardo/samples",
)


class SearchSamplesTool(MCPTool):
    """Search Renardo sample packs by keyword in filename.

    Returns letter, sample_index, and ready-to-use play_code for ``d1 >> play(...)``.
    Ported from ``MusicSkill.search_samples`` (filesystem-based, no Renardo runtime needed).
    """

    def __init__(self, node, samples_path: Optional[str] = None) -> None:
        super().__init__(node)
        self._samples_path = Path(samples_path or _SEARCH_SAMPLES_DEFAULT_PATH)

    @property
    def name(self) -> str:
        return "search_samples"

    @property
    def description(self) -> str:
        return (
            "Поиск Renardo-сэмплов по ключевому слову в имени файла. "
            "Возвращает букву, sample_index и готовый play_code. "
            "Используй когда нужно найти неизвестную букву/индекс сэмпла. "
            "query='*' — обзор всех доступных букв и количества сэмплов в паке."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="query",
                type="string",
                description=(
                    "Ключевое слово: kick, snare, hat, bass, synth, vocal, glitch, dist, loop. "
                    "'*' — компактный обзор всех букв."
                ),
                required=True,
            ),
            MCPToolParameter(
                name="pack",
                type="string",
                description=(
                    "Имя пакета: '0_foxdot_default' (стандартный) или "
                    "'1_pitchglitch_samples' (расширенный, включает вокал/FX)."
                ),
                required=False,
            ),
            MCPToolParameter(
                name="case",
                type="string",
                description="Регистр буквы: 'lower' или 'upper'.",
                required=False,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def read_only(self) -> bool:
        return True

    @property
    def destructive(self) -> bool:
        return False

    def execute(
        self,
        query: str,
        pack: str = "0_foxdot_default",
        case: str = "lower",
    ) -> MCPToolResult:
        """Search samples by keyword."""
        from rob_box_voice.core.sample_search import search_renardo_samples

        result = search_renardo_samples(self._samples_path, query, pack, case)

        if "error" in result:
            hint = result.get("hint", "")
            available = result.get("available_packs")
            detail = f" Доступны: {available}" if available else ""
            return MCPToolResult(
                success=False,
                error=f"{result['error']}.{detail} {hint}".strip(),
            )

        if "letters" in result:
            letters = result["letters"]
            return MCPToolResult(
                success=True,
                data=result,
                message=(
                    f"Пак '{pack}': {len(letters)} букв, "
                    f"всего сэмплов: {result.get('total_samples', sum(letters.values()))}"
                ),
            )

        found = result.get("found", 0)
        results_list = result.get("results", [])

        if found == 0:
            return MCPToolResult(
                success=True,
                data=result,
                message=(
                    f"По запросу '{query}' в паке '{pack}' ничего не найдено. "
                    "Попробуй: kick, snare, hat, bass, synth, '*'."
                ),
            )

        self.log_info(
            f"[search_samples] query={query!r} pack={pack} → {found} results"
        )
        play_codes = [r["play_code"] for r in results_list[:5]]
        suffix = f" ... и ещё {found - 5}" if found > 5 else ""
        return MCPToolResult(
            success=True,
            data=result,
            message=(
                f"Найдено {found} сэмплов по запросу '{query}': "
                + ", ".join(play_codes)
                + suffix
            ),
        )


class SetDjModeTool(MCPTool):
    """Включить или выключить режим DJ — автономные плавные переходы между треками."""

    def __init__(self, node) -> None:
        super().__init__(node)
        from std_msgs.msg import String as _String
        self._dj_mode_pub = node.create_publisher(_String, "/voice/dj_mode", 10)

    @property
    def name(self) -> str:
        return "set_dj_mode"

    @property
    def description(self) -> str:
        return (
            "Включить или выключить режим DJ. "
            "В режиме DJ робот автономно делает плавные переходы между музыкальными треками "
            "каждые 30–60 секунд, создавая атмосферу живой вечеринки. "
            "Используй enabled=true чтобы включить, enabled=false чтобы выключить. "
            "Перед включением убедись что музыка уже играет (запусти трек через execute_music_code)."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="enabled",
                type="boolean",
                description="true — включить DJ-режим (автопереходы), false — выключить",
                required=True,
            ),
            MCPToolParameter(
                name="next_transition_sec",
                type="integer",
                description=(
                    "Через сколько секунд сделать следующий автоматический переход (30–120). "
                    "ЛЛМ выбирает сам исходя из темпа сета: "
                    "быстрый энергичный сет → 30–40 сек, "
                    "медленный амбиент → 60–90 сек. "
                    "Обязательно передавай при enabled=true, в том числе при каждом DJ-переходе."
                ),
                required=False,
            ),
            MCPToolParameter(
                name="theme",
                type="string",
                description=(
                    "Тема вечеринки / контекст для DJ (например: '8 марта, женский день', "
                    "'день рождения Антона', 'хэллоуин', 'корпоратив в стиле 90-х'). "
                    "Передавай при первом включении DJ-режима — робот будет подстраивать музыку "
                    "и иногда тематически обращаться к публике. При повторных вызовах set_dj_mode "
                    "внутри DJ-переходов тему передавать не нужно — она запомнена."
                ),
                required=False,
            ),
            MCPToolParameter(
                name="persona",
                type="string",
                description=(
                    "DJ-образ / персона, которую юзер задал словами "
                    "(например 'диджей Пёс', 'диджей Кот'). Передавай когда "
                    "юзер назначил роль — робот будет представляться этим "
                    "образом. По умолчанию 'ДиДжей РОббокс'."
                ),
                required=False,
            ),
            MCPToolParameter(
                name="plan",
                type="string",
                description=(
                    "План DJ-сета: список треков/блоков через новую строку, "
                    "каждый начинается с 'Трек N:', например: "
                    "'Трек 1: энергичный старт 128bpm\\nТрек 2: диско-хит 90-х\\nТрек 3: финальный вальс'. "
                    "Передавай при ПЕРВОМ включении DJ — робот пройдёт по плану "
                    "и на последнем треке объявит 'вечеринка заканчивается' и сам выключит DJ."
                ),
                required=False,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.INSTANT

    @property
    def destructive(self) -> bool:
        return False

    def execute(self, enabled: bool, next_transition_sec: Optional[int] = None, theme: Optional[str] = None, transition_seconds: Optional[int] = None, persona: Optional[str] = None, plan: Optional[str] = None) -> MCPToolResult:
        """Опубликовать команду включения/выключения DJ-режима."""
        from std_msgs.msg import String as _String
        # LLM иногда шлёт transition_seconds вместо next_transition_sec
        if transition_seconds is not None and next_transition_sec is None:
            next_transition_sec = transition_seconds
        payload: dict = {"enabled": enabled}
        if next_transition_sec is not None:
            payload["next_transition_sec"] = max(15, min(300, int(next_transition_sec)))
        if theme and isinstance(theme, str) and theme.strip():
            payload["theme"] = theme.strip()
        # 🔴 FIX (live 10:13 DJ): персона юзера («ты диджей Пёс») —
        # пробрасываем в DJState, чтобы автопромпты не перезаписывали
        # её дефолтом «ДиДжей РОббокс».
        if persona and isinstance(persona, str) and persona.strip():
            payload["persona"] = persona.strip()
        # 🔴 FIX (live 15:30 06.08): план сета — DJ проходит по плану и
        # корректно завершается с финальным объявлением, а не молча по лимиту.
        if plan and isinstance(plan, str) and plan.strip():
            payload["plan"] = plan.strip()
        msg = _String()
        msg.data = json.dumps(payload)
        self._dj_mode_pub.publish(msg)
        action = "включён" if enabled else "выключен"
        interval_info = f" (следующий через {next_transition_sec}с)" if next_transition_sec and enabled else ""
        persona_info = f", персона: {persona}" if persona else ""
        plan_info = f", план: {len(plan.splitlines())} треков" if plan else ""
        self.log_info(f"🎧 DJ-режим {action}{interval_info}{persona_info}{plan_info}")
        return MCPToolResult(success=True, message=f"DJ-режим {action}{interval_info}{persona_info}{plan_info}")
