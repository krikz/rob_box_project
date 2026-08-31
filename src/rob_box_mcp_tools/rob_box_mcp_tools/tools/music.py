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

import ast
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
from ..core.arranger import (
    FORMS,
    VALID_ROOTS,
    ArrangementError,
    form_summary,
    render,
    spec_from_flat,
)

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

# Live 13.08 — символы сэмплов в play("x-o-") для предзагрузки буферов.
_PLAY_SYMBOLS_RE = re.compile(r'play\(\s*"([^"]*)"')

# ---------------------------------------------------------------------------
# Pattern-name whitelist (security) — see stop_pattern()
# ---------------------------------------------------------------------------
# ``stop_pattern`` used to build ``f"{pattern_name}.stop()"`` and hand it to
# exec(), so an LLM-supplied (or prompt-injected) name like
# ``__import__('os').system('id') #`` was arbitrary code execution with the
# MCP server's privileges. The name is now (a) shape-checked against a plain
# identifier, (b) checked against the whitelist of patterns that actually
# exist, and (c) resolved via attribute lookup instead of exec.

#: Renardo's built-in player namespace: d1-d9, p1-p9, s1-s9, l1-l9.
_RENARDO_PLAYER_NAMES: frozenset = frozenset(
    f"{prefix}{i}" for prefix in ("d", "p", "s", "l") for i in range(1, 10)
)

#: A pattern name must be a bare Python identifier — no dots, calls, quotes,
#: comments or whitespace can survive this.
_PATTERN_NAME_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]{0,31}$")

#: Reflection builtins that stay allowed (legitimate Renardo use — e.g.
#: ``Clock.future(8, lambda: setattr(Clock, "bpm", 170))``) but only when the
#: attribute name is a plain string literal, never a computed one.
_LITERAL_ATTR_BUILTINS: frozenset = frozenset({"getattr", "setattr", "hasattr"})

# ---------------------------------------------------------------------------
# Issue #1016 — music-quality guardrail (dramaturgy validator)
# ---------------------------------------------------------------------------
# The safety filter above blocks *dangerous system tokens*. This separate
# guardrail validates *musical quality* before the code reaches Renardo so
# the LLM cannot regenerate a static 4-8 note loop:
#
#   1. Absolute frequencies (freq=440 / hz=220 / midinote=69) are rejected
#      — Renardo wants scale *degrees* (p1 >> pluck([0,4,7])), not Hz.
#   2. Every non-play player must carry an explicit ``dur=`` — otherwise
#      the pattern defaults to a staccato click-train.
#   3. (soft) A multi-part track without any developing pattern
#      (``.every`` / ``Pvar`` / ``linvar`` / ``Clock.future``) is a static
#      loop — warn so the LLM can fix it before the user hears it.
#
# Errors block execution; warnings are appended to the result message.

_ABSOLUTE_FREQ_RE = re.compile(
    r"\b(?:freq|frequency|hz|midinote|note)\s*=\s*(\d+(?:\.\d+)?)"
)
# Player creation lines: `p1 >> pluck([0,2,4], dur=0.5)` / `d1 >> play("x-o-")`
_PLAYER_LINE_RE = re.compile(r"^\s*(\w+)\s*>>\s*(\w+)\s*\(([^)]*)\)", re.MULTILINE)
# Developing patterns that break a static loop (issue #1016).
_DEV_PATTERN_RE = re.compile(
    r"\.every\(|Pvar\(|pvar\(|linvar\(|var\(|Clock\.future|chop=|stutter|shuffle|reverse"
)
# Hard-blocked hardware constraints (live 20.08): deepseek игнорирует
# промпт-запреты, поэтому ловим на уровне кода. chop= (не 0) → щелчки на
# 16 kHz DAC; spack= (не 0) → сырые глитчевые сэмплы pitchglitch-пака.
_CHOP_RE = re.compile(r"\bchop\s*=\s*(?!0\b)")
_SPACK_NONZERO_RE = re.compile(r"\bspack\s*=\s*[1-9]")

# Issue #1804 — на роботе физически смонтированы только d1-d3/p1-p3.
# Токен слева от ``>>`` в форме [dpsl]+цифра — это renardo-плеер; если он
# вне допустимой шестёрки, код обязан переставить слой в свободный слот
# (см. ``_remap_illegal_slots``), а не молча дать модели написать в d4/p5.
_ALLOWED_PLAYER_SLOTS: Tuple[str, ...] = ("d1", "d2", "d3", "p1", "p2", "p3")
_ALLOWED_PLAYER_SLOTS_SET: frozenset = frozenset(_ALLOWED_PLAYER_SLOTS)
_PLAYER_ASSIGN_RE = re.compile(
    r"^(?P<indent>[ \t]*)(?P<name>[dpsl]\d+)(?P<arrow>\s*>>\s*)(?P<synth>\w+)\s*\(",
    re.MULTILINE,
)

# Issue #1803 — длина рисунка play("...") задаёт его период; если она не
# делит такт, рисунок плывёт относительно соседних слоёв на каждом
# повторе (см. ``_fix_pattern_length``).
_PLAY_PATTERN_LEN_RE = re.compile(r"play\((\s*)(['\"])([^'\"]*)\2")


# ---------------------------------------------------------------------------
# MusicManager
# ---------------------------------------------------------------------------


def _is_dunder(name: str) -> bool:
    """``True`` для ``__name__``-подобных имён (см. :meth:`MusicManager._filter_code_ast`)."""
    return name.startswith("__") and name.endswith("__")


# 🔴 FIX (live 30.08): этот список ОБЯЗАН покрывать всю палитру,
# которую промпт предлагает модели. Раньше он был отдельной копией и
# разъехался: живой опрос scsynth показал, что 15 предлагаемых синтов
# на сервере отсутствуют, и девять из них — arpy, pianovel, cs80lead,
# supersawlead, dirt, moogbass, strangerpulsepad, rave, donk — не
# покрывались ни прелоадом, ни досылкой отсюда. Модель выбирает такой
# синт для мелодии, /s_new отбивается, и трек играет без темы: в
# прогоне 30.08 это был supersawlead. Синты из списка досылались и
# работали, так что механизм исправен — дырой был именно охват.
#
# Порядок: сначала палитра из master_prompt_compact.txt, затем то,
# что палитра не рекламирует, но чем пользуется execute_music_code.
CRITICAL_SYNTHS: tuple = (
    # melody
    "blip", "arpy", "pianovel", "epiano", "rhpiano", "karp", "sitar",
    "marimba", "bell", "cs80lead", "supersawlead", "imperialbrass",
    "strangerarp",
    # bass
    "dub", "wobblebass", "fuzz", "dirt", "subbass", "moogbass",
    "retrobass",
    # pads
    "strings", "pads", "ambi", "space", "sinepad", "warmpad",
    "strangerpulsepad",
    # brass
    "brass", "flute", "soprano", "eoboe", "organ", "strangerbrass",
    # glitch
    "rave", "donk", "varsaw", "pulse", "tb303",
    # не в палитре, но используются напрямую
    "bass", "gong", "pluck", "saw", "square", "faim", "viola",
    "noise", "scatter", "orient", "creep", "play1", "play2",
)


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
    # Issue #1808 — слушатель ответов scsynth (/fail, /done)
    # ------------------------------------------------------------------
    # Renardo шлёт ноты в scsynth fire-and-forget и НИКОГДА не читает ответы
    # (см. ``_attach_renardo_reply_listener`` ниже) — все отказы звукового
    # тракта («SynthDef not found», «too many nodes», «Group N not found»)
    # были видны только в логе контейнера ``supercollider`` (сам scsynth их
    # печатает), куда никто не смотрит при разборе инцидентов.
    #
    # Таймаут ниже используется ТОЛЬКО в ``_send_osc_raw`` (наши собственные
    # админ-сообщения — /g_new, /g_freeAll, /n_set мастер-фейдера): после
    # sendto() кратко слушаем тот же сокет на предмет /fail. На УСПЕШНЫЙ
    # /g_new или /n_set scsynth вообще ничего не шлёт в ответ — значит этот
    # таймаут оплачивается ПОЛНОСТЬЮ на каждом успешном вызове. Держим его
    # маленьким (заметно меньше уже существующей паузы 50ms между
    # /g_freeAll и /g_new, issue #778) — на loopback ответ, если он будет,
    # приходит за микросекунды, а лишние 30ms на нечастых admin-вызовах
    # (пересоздание группы, смена мастер-гейна) незаметны на фоне музыки.
    OSC_REPLY_TIMEOUT_SECONDS: float = 0.03

    # ------------------------------------------------------------------
    # Master limiter (docs/analysis/2026-08-30-music-quality-audit.md)
    # ------------------------------------------------------------------
    #: Node ID синта ``masterlimiter``, который ``foxdot_init.sc`` ставит в
    #: хвост RootNode. Держится НИЖЕ 1000: renardo раздаёт ID начиная с 1001
    #: и только вверх (``ServerManager.nextnodeID``), поэтому коллизии быть
    #: не может, а ``/g_freeAll 1`` (Clock.clear / stop_all) чистит только
    #: группу 1 и лимитер не трогает.
    MASTER_LIMITER_NODE: int = 999
    #: Уровень мастер-фейдера ПОСЛЕ лимитера. Именно он задаёт громкость
    #: музыки относительно речи (issue #986), а не покомпонентные капы amp.
    DEFAULT_MASTER_GAIN: float = 0.5
    #: Class-level fallback-ы: ``__init__`` их перекрывает, но менеджер
    #: конструируют и через ``MusicManager.__new__`` (тесты, восстановление
    #: после частичной деградации). Без них ``execute_code`` падал бы с
    #: AttributeError — тот же defensive-SSoT приём, что в #1395.
    _master_gain: float = DEFAULT_MASTER_GAIN
    _master_gain_applied: bool = False
    #: Issue #1808 — сокет Renardo (``_rt.Server.client.socket``), к которому
    #: подключён фоновый слушатель ответов scsynth. ``None`` пока слушатель
    #: не подключён (или подключить не удалось — best-effort). Тот же
    #: defensive-SSoT приём: тесты создают ``MusicManager`` через
    #: ``__new__`` в обход ``__init__``.
    _renardo_reply_sock: Optional[Any] = None

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
    #:
    #: 🔴 FIX (live 30.08, vision-pi 12:30): «сыграй короткий бит» →
    #: ``segments=8`` при ``Clock.bpm=90`` = 21.3 s. Watchdog убил бит через
    #: 20 s — то есть дедлайн, объявленный «предохранителем», на практике и
    #: был длиной трека: TTS закончился на 11-й секунде, а музыка играла
    #: одна ещё 7 секунд и оборвалась. Юзер в следующем ходе просил
    #: «продолжай развивать бит», когда играть было уже нечему.
    #:
    #: Держим дедлайн предохранителем: пол поднят с 15 s до 60 s, а
    #: посчитанная по ``segments`` длительность умножается на
    #: ``SEGMENTS_DEADLINE_SAFETY_FACTOR``. Верхняя граница остаётся —
    #: музыка по-прежнему не может играть вечно.
    MIN_SEGMENTS_DEADLINE_SECONDS: float = 60.0
    #: Во сколько раз дедлайн длиннее музыкальной длины, посчитанной по
    #: ``segments``. Оценка LLM — ориентир, а не контракт.
    SEGMENTS_DEADLINE_SAFETY_FACTOR: float = 2.0
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
        max_amp: float = 0.85,
        *,
        master_gain: Optional[float] = None,
        critical_synths: Optional[List[str]] = None,
        require_healthy: Optional[bool] = None,
        sclang_log_path: Optional[str] = None,
    ) -> None:
        #: Санитарный потолок амплитуды ОДНОГО слоя (0.0-1.0). Это НЕ
        #: регулятор громкости: сумму держит ``masterlimiter`` в scsynth,
        #: а уровень относительно речи — ``_master_gain``. Поэтому потолок
        #: высокий: слоям снова можно быть разной громкости, иначе микс
        #: получается плоским (RC1 в аудите).
        self._max_amp: float = max(0.0, min(1.0, max_amp))
        #: Уровень мастер-фейдера лимитера.
        self._master_gain: float = max(
            0.0,
            min(1.0, self.DEFAULT_MASTER_GAIN if master_gain is None else master_gain),
        )
        #: Отправлен ли ``/n_set`` с мастер-фейдером хотя бы раз.
        self._master_gain_applied: bool = False
        #: pattern_name -> последний выполненный код
        self._pattern_history: Dict[str, str] = {}
        #: множество имён активных паттернов
        self._active_patterns: set = set()
        #: SynthDef-ы, уже загруженные через sdef.add(). Повторный add()
        #: мутирует UGen-граф (osc*env) → компаундинг ("too big for
        #: sending") → scsynth не тянет → "late" и троттл (live 20.08).
        self._synthdefs_added: set = set()
        #: имя текущего пресета
        self._current_preset: Optional[str] = None
        #: контекст выполнения для renardo
        self._renardo_context: Dict[str, Any] = {}
        #: True если renardo доступен, False/None иначе
        self._renardo_available: Optional[bool] = None
        #: Последняя ошибка инициализации renardo для диагностики
        self._renardo_last_error: Optional[str] = None
        #: Issue #1808 — сокет Renardo, к которому подключён фоновый
        #: слушатель ответов scsynth (см. ``_attach_renardo_reply_listener``).
        self._renardo_reply_sock: Optional[Any] = None
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

            # 🔴 FIX (issue #1808): Renardo шлёт ноты в scsynth
            # fire-and-forget и никогда не читает ответы — все отказы
            # звукового тракта («SynthDef X not found», «too many nodes»,
            # «Group N not found») уходили только в лог контейнера
            # supercollider, куда никто не смотрит при разборе (см.
            # docstring ``_attach_renardo_reply_listener``). Best-effort,
            # ничего не ломает при неудаче.
            self._attach_renardo_reply_listener(_rt)

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
            for idx, (name, sdef) in enumerate(_rt.SynthDefs.items()):
                if name in self._synthdefs_added:
                    continue
                sdef.add()
                self._synthdefs_added.add(name)
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

        _CRITICAL_SYNTHS = CRITICAL_SYNTHS

        def _probe_missing(names):
            """Return subset of names whose SynthDef is absent in scsynth."""
            missing: list[str] = []
            probe_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            probe_sock.settimeout(0.4)

            def _free_probe_node(node_id: int) -> None:
                """Free a probe node via /n_free (best-effort, fire-and-forget)."""
                free_msg = bytearray(b"/n_free\x00")
                free_msg.extend(b",i\x00\x00")
                free_msg.extend(_struct.pack(">i", node_id))
                try:
                    probe_sock.sendto(bytes(free_msg), (self.SC_HOST, self.SC_PORT))
                except OSError:
                    pass

            for i, name in enumerate(names):
                node_id = 9000 + i
                # 🔴 FIX (live 13.08): OSC-адрес обязан быть выровнен до
                # кратного 4. "/s_new" = 6 байт + 2 нуля = 8 (было 7 —
                # scsynth отвечал "FAILURE IN SERVER: /s_new Command not
                # found", а строка "not found" матчилась как «синт
                # отсутствует» → ложные 3 раунда досылки всех 31 синтов).
                msg = bytearray(b"/s_new\x00\x00")
                # 🔴 FIX (19.08, свист #1444): type tag был ",siiiif" —
                # control-имя "amp" типизировано как int (i), а не string (s).
                # scsynth читал "amp\x00" как int32 → amp=0 НЕ применялся,
                # пробные ноды играли с дефолтным amp=1 и sus=1 (sustained)
                # и никогда не освобождались → постоянный свист из динамика.
                # Правильный tag ",siiisf": name(s) + node/addAction/target
                # (iii) + "amp"(s) + 0.0(f).
                types = b",siiisf\x00"
                name_b = name.encode() + b"\x00"
                while len(name_b) % 4:
                    name_b += b"\x00"
                msg.extend(types)
                msg.extend(name_b)
                msg.extend(_struct.pack(">iii", node_id, 0, 1))
                msg.extend(b"amp\x00")
                msg.extend(_struct.pack(">f", 0.0))
                try:
                    probe_sock.sendto(bytes(msg), (self.SC_HOST, self.SC_PORT))
                    data, _ = probe_sock.recvfrom(512)
                    # Only a REAL "SynthDef X not found" counts as missing.
                    # "Command not found" (malformed) and "Group N not found"
                    # must not trigger re-sending.
                    if b"SynthDef" in data and b"not found" in data:
                        missing.append(name)
                    else:
                        # 🔴 FIX (19.08, свист #1444): освобождаем созданную
                        # пробную ноду сразу (иначе она живёт вечно с sus=1).
                        _free_probe_node(node_id)
                except socket.timeout:
                    # Нет ответа = def на месте = нода создана — освобождаем.
                    _free_probe_node(node_id)
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
                    if name in self._synthdefs_added:
                        # Уже добавляли — повторный add() мутирует UGen-граф
                        # (компаундинг). Досылаем без мутации через load()
                        # (отправка готового .scd), если метод доступен.
                        load = getattr(sdef, "load", None)
                        if load is not None:
                            load()
                    else:
                        sdef.add()
                        self._synthdefs_added.add(name)
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

    # ------------------------------------------------------------------
    # Issue #1808 — слушатель ответов scsynth (/fail, /done)
    # ------------------------------------------------------------------
    #
    # РЕШЕНИЕ (обоснование выбора «только логировать», см. issue #1808):
    #
    # У scsynth-трафика два независимых источника:
    #   (а) наши собственные админ-команды — ``_send_osc_raw`` (создание
    #       Group 1, /g_freeAll+/g_new при Clock.clear(), /n_set мастер-
    #       фейдера) — синхронные, отправляются и завершаются внутри
    #       одного вызова Python;
    #   (б) реальные ноты, которые Renardo шлёт из своего Clock-потока —
    #       АСИНХРОННО, зачастую на следующий бит ПОСЛЕ того, как
    #       ``execute_code``/``compose_music`` уже вернул «успешно».
    #
    # Для (б) нет способа синхронно привязать ответ scsynth к конкретному
    # вызову тула — только эвристика по времени, а именно её юзер попросил
    # не городить («привязывать по времени осторожно, ложные срабатывания
    # хуже молчания»). Один /fail может относиться к вызову N, а прийти
    # уже во время обработки вызова N+1 — риск обвинить не тот tool-call.
    #
    # Поэтому оба источника (а) и (б) только ЛОГИРУЮТСЯ в лог ноды
    # mcp_server (тот же ``_log_warning``, что и остальные диагностические
    # сообщения в этом файле — попадает в ``docker logs voice-assistant``).
    # Возврат в результат ``execute_music_code``/``compose_music`` — заявлен
    # в issue как ценное развитие, оставлен как отдельный follow-up, когда
    # появится безопасный способ привязки без ложных срабатываний.

    def _attach_renardo_reply_listener(self, _rt: Any) -> None:
        """Повесить фоновый слушатель на СОБСТВЕННЫЙ сокет Renardo (best-effort).

        Renardo (``renardo.sc_backend.server_manager.ServerManager``) держит
        ОДИН долгоживущий UDP-сокет для всех сообщений к scsynth
        (``Server.client.socket``, законнекченный на 127.0.0.1:57110) и
        никогда его не читает — ``OSCClient.send()`` только ``sendall()``,
        ни одного ``recv`` во всём классе. Значит ответы scsynth на РЕАЛЬНЫЕ
        ноты («SynthDef blip not found», «/s_new too many nodes», «Group N
        not found» — именно те три бага, что стоили нам двух дней отладки)
        сейчас просто лежат непрочитанными в приёмном буфере этого сокета.

        Мы ничего не меняем в отправке (Renardo продолжает слать как
        раньше) и только ЧИТАЕМ из ТОГО ЖЕ сокета в отдельном потоке —
        recv() и send() на законнекченном UDP-сокете независимы друг от
        друга, гонки с Renardo нет (он этот сокет не читает вовсе).

        Полностью best-effort и не бросает исключений наружу: если версия
        Renardo другая и объектный граф не совпадает (``Server``/``client``/
        ``socket`` переименованы или отсутствуют), просто не получаем этот
        источник и остаёмся с логированием только ``_send_osc_raw`` —
        никогда не роняем инициализацию музыки и никогда не выдумываем
        логи (нечего слушать = тишина, а не ложное срабатывание).
        """
        try:
            server = getattr(_rt, "Server", None)
            client = getattr(server, "client", None)
            sock = getattr(client, "socket", None)
            if not isinstance(sock, socket.socket):
                return
            if sock is self._renardo_reply_sock:
                return  # уже слушаем этот же сокет (повторный _ensure_renardo_available)
            self._renardo_reply_sock = sock
            thread = threading.Thread(
                target=self._renardo_reply_listener_loop,
                args=(sock,),
                name="scsynth-reply-listener",
                daemon=True,
            )
            thread.start()
        except Exception:  # noqa: BLE001 — best-effort, не мешаем инициализации
            pass

    def _renardo_reply_listener_loop(self, sock: "socket.socket") -> None:
        """Фоновый цикл: блокирующий recv на сокете Renardo, лог каждого /fail.

        Сокет Renardo обычно блокирующий (без ``settimeout``) — поток тихо
        спит между ответами, CPU не тратит. Если сокет закроют (например,
        Renardo пересоздаст ``Server.client`` при повторной инициализации),
        ``recvfrom`` бросит ``OSError`` — поток завершается сам, без шума.
        """
        while True:
            try:
                data, _addr = sock.recvfrom(4096)
            except OSError:
                return
            except Exception:  # noqa: BLE001 — единичный кривой пакет не должен убивать поток
                continue
            try:
                self._log_osc_reply(data)
            except Exception:  # noqa: BLE001
                continue

    def _log_scsynth_reply_if_any(self, sock: "socket.socket") -> None:
        """После собственного ``sendto`` кратко послушать тот же сокет на /fail.

        Таймаут короткий (``OSC_REPLY_TIMEOUT_SECONDS``) — см. обоснование
        у объявления константы. Полностью best-effort: таймаут/любая ошибка
        чтения — это НОРМА (большинство успешных admin-команд scsynth не
        подтверждает вовсе), а не повод помешать вызывающему коду.
        """
        try:
            sock.settimeout(self.OSC_REPLY_TIMEOUT_SECONDS)
            data, _addr = sock.recvfrom(4096)
        except Exception:  # noqa: BLE001 — таймаут = scsynth принял молча (норма)
            return
        try:
            self._log_osc_reply(data)
        except Exception:  # noqa: BLE001
            pass

    def _log_osc_reply(self, data: bytes) -> None:
        """Разобрать ответ scsynth; залогировать, если это ``/fail``.

        Полный OSC-парсер не нужен — только различить ``/fail`` (реальный
        отказ, ту самую строку из логов supercollider, которую раньше
        никто не видел) от остального (``/done``, ``/synced`` и т.п. —
        штатные подтверждения, шум для лога ошибок).
        """
        address, rest = self._split_osc_address(data)
        if address != "/fail":
            return
        args = self._decode_osc_args(rest)
        detail = " ".join(str(a) for a in args) if args else rest.decode("utf-8", "replace")
        self._log_warning(f"🔴 [scsynth] FAILURE IN SERVER: {detail}")

    @staticmethod
    def _split_osc_address(data: bytes) -> Tuple[Optional[str], bytes]:
        """Извлечь OSC-адрес из пакета; вернуть (адрес, остаток-с-выравниванием)."""
        if not data or data[0:1] != b"/":
            return None, b""
        end = data.find(b"\x00")
        if end == -1:
            return None, b""
        address = data[:end].decode("ascii", "replace")
        consumed = end + 1
        while consumed % 4:
            consumed += 1
        return address, data[consumed:]

    @staticmethod
    def _decode_osc_args(rest: bytes) -> List[Any]:
        """Разобрать OSC type-tag строку (``,ssif``...) и аргументы за ней."""
        if not rest or rest[0:1] != b",":
            return []
        end = rest.find(b"\x00")
        if end == -1:
            return []
        tags = rest[1:end].decode("ascii", "replace")
        offset = end + 1
        while offset % 4:
            offset += 1
        args: List[Any] = []
        for tag in tags:
            if tag == "i":
                if offset + 4 > len(rest):
                    break
                args.append(struct.unpack(">i", rest[offset:offset + 4])[0])
                offset += 4
            elif tag == "f":
                if offset + 4 > len(rest):
                    break
                args.append(struct.unpack(">f", rest[offset:offset + 4])[0])
                offset += 4
            elif tag == "s":
                str_end = rest.find(b"\x00", offset)
                if str_end == -1:
                    break
                args.append(rest[offset:str_end].decode("utf-8", "replace"))
                offset = str_end + 1
                while offset % 4:
                    offset += 1
            else:
                # blob (b) и прочие типы не разбираем — для лога достаточно
                # того, что уже накопили; останавливаемся, а не падаем.
                break
        return args

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
        big-endian args. Поддерживает int (``i``), float (``f``) и
        string (``s``) аргументы. Строки нужны для ``/n_set <node>
        <control-name> <value>`` (мастер-фейдер лимитера): имя контрола
        обязано ехать как ``s``, иначе scsynth читает первые 4 байта имени
        как int32 и молча игнорирует установку — ровно та же ловушка, что
        описана в ``_verify_and_retry_synthdefs`` про ``"amp"``.

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

        🔴 FIX (issue #1808): раньше сокет закрывался сразу после
        ``sendto`` (``with`` выходил из блока) — если scsynth отвечал
        ``/fail`` (например «Group 1 not found»), ответ прилетал уже на
        закрытый сокет и терялся молча. Теперь перед закрытием кратко
        слушаем этот же сокет (``_log_scsynth_reply_if_any``) — см.
        обоснование таймаута у ``OSC_REPLY_TIMEOUT_SECONDS``.
        """
        msg = bytearray()
        addr_bytes = address.encode() + b"\x00"
        while len(addr_bytes) % 4:
            addr_bytes += b"\x00"

        def _tag(value: Any) -> bytes:
            if isinstance(value, str):
                return b"s"
            # bool is a subclass of int — проверяем int после str, как раньше.
            return b"i" if isinstance(value, int) else b"f"

        types = b"," + b"".join(_tag(a) for a in args) + b"\x00"
        while len(types) % 4:
            types += b"\x00"
        msg.extend(addr_bytes)
        msg.extend(types)
        for a in args:
            if isinstance(a, str):
                blob = a.encode() + b"\x00"
                while len(blob) % 4:
                    blob += b"\x00"
                msg.extend(blob)
            elif isinstance(a, int):
                msg.extend(struct.pack(">i", a))
            elif isinstance(a, float):
                msg.extend(struct.pack(">f", a))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(bytes(msg), (self.SC_HOST, self.SC_PORT))
            # Issue #1808 — см. docstring выше и обоснование у
            # OSC_REPLY_TIMEOUT_SECONDS. Best-effort, никогда не бросает.
            self._log_scsynth_reply_if_any(sock)

    # ------------------------------------------------------------------
    # Master limiter fader
    # ------------------------------------------------------------------

    def set_master_gain(self, gain: float) -> float:
        """Задать уровень мастер-фейдера ``masterlimiter`` в scsynth.

        Это единственная ручка громкости музыки относительно речи
        (issue #986). Внутренняя динамика микса при этом сохраняется —
        в отличие от старого способа «зарезать amp каждого слоя до 0.42»,
        который выравнивал слои и делал микс плоским.

        Синт ``masterlimiter`` сглаживает изменение через ``Lag.kr``, так
        что смена уровня на лету не даёт щелчка. Если синта нет (сборка без
        обновлённого ``foxdot_init.sc``), scsynth просто залогирует
        ``/n_set Node not found`` — музыка продолжит играть без фейдера.

        Returns:
            Применённое (клэмпнутое) значение.
        """
        self._master_gain = max(0.0, min(1.0, float(gain)))
        try:
            self._send_osc_raw(
                "/n_set", self.MASTER_LIMITER_NODE, "gain", self._master_gain
            )
        except OSError as exc:  # UDP-сокет недоступен — не роняем музыку
            self._log_warning(f"master gain not applied: {exc}")
        return self._master_gain

    # ------------------------------------------------------------------
    # Code safety filter
    # ------------------------------------------------------------------

    def _filter_code(self, code: str) -> Tuple[bool, str]:
        """Проверить код на наличие опасных конструкций.

        Двухслойная проверка:

        1. Текстовый blocklist (``_BLOCKED_TOKENS``) — быстрый отсев
           очевидных ``import`` / ``os`` / ``eval``.
        2. AST-проход (:meth:`_filter_code_ast`) — закрывает классические
           обходы текстового фильтра: доступ к dunder-атрибутам
           (``__class__`` / ``__globals__`` / ``__subclasses__``) и
           ``getattr``/``setattr`` с вычисляемым (собранным из строк)
           именем атрибута. Литеральные ``setattr(Clock, "bpm", 170)``
           остаются разрешены — они нужны для ``Clock.future()``.

        Args:
            code: Строка кода для проверки.

        Returns:
            (is_safe, error_message) — (True, "") если код безопасен.
        """
        match = _BLOCKED_TOKENS.search(code)
        if match:
            return False, f"Запрещённый токен в коде: '{match.group()}'"
        return self._filter_code_ast(code)

    @staticmethod
    def _filter_code_ast(code: str) -> Tuple[bool, str]:
        """AST-слой фильтра безопасности (см. :meth:`_filter_code`).

        Текстовый blocklist сравнивает слова с исходником, поэтому его
        обходит любое имя, собранное во время выполнения
        (``getattr(x, "__cla" + "ss__")``) или добытое через dunder-цепочку
        (``().__class__.__subclasses__()``). Здесь мы смотрим на реальную
        структуру кода, а не на текст.

        Args:
            code: Строка кода для проверки.

        Returns:
            (is_safe, error_message) — (True, "") если код безопасен.
        """
        try:
            tree = ast.parse(code)
        except SyntaxError as exc:
            return False, f"Синтаксическая ошибка в коде: {exc.msg}"

        for node in ast.walk(tree):
            # ``().__class__`` / ``p1.__globals__`` — цепочка побега из
            # песочницы всегда проходит через dunder-атрибут.
            if isinstance(node, ast.Attribute) and _is_dunder(node.attr):
                return False, f"Запрещённый доступ к dunder-атрибуту: '{node.attr}'"
            if isinstance(node, ast.Name) and _is_dunder(node.id):
                return False, f"Запрещённое dunder-имя: '{node.id}'"
            # ``getattr(x, name)`` с невычислимым именем — это обход
            # текстового фильтра. Литеральное имя разрешаем (но не dunder).
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
                if node.func.id in _LITERAL_ATTR_BUILTINS and len(node.args) >= 2:
                    attr_arg = node.args[1]
                    if not isinstance(attr_arg, ast.Constant) or not isinstance(
                        attr_arg.value, str
                    ):
                        return False, (
                            f"'{node.func.id}' допустим только со строковым "
                            "литералом в качестве имени атрибута"
                        )
                    if _is_dunder(attr_arg.value):
                        return False, (
                            f"Запрещённый доступ к dunder-атрибуту: "
                            f"'{attr_arg.value}'"
                        )
        return True, ""

    # ------------------------------------------------------------------
    # Issue #1016 — music-quality guardrail (dramaturgy validator)
    # ------------------------------------------------------------------

    def _validate_music_code(self, code: str) -> Tuple[List[str], List[str]]:
        """Проверить музыкальное качество кода перед отправкой в Renardo.

        Отличается от :meth:`_filter_code` (безопасность): этот валидатор
        ловит *музыкальные* ошибки LLM, из-за которых трек звучит как
        статичный луп (issue #1016):

        1. Абсолютные частоты (``freq=440`` / ``hz=220`` / ``midinote=69``)
           — Renardo ожидает ступени (``p1 >> pluck([0,4,7])``), а не Hz.
           → HARD error, выполнение блокируется.
        2. ``dur=`` у каждого не-play плеера — без него паттерн играет
           staccato-щелчками (дефолтный dur=1 с sus=0).
           → WARNING (play() имеет собственный dur из паттерна).
        3. (soft) Многоголосный трек без развивающих паттернов
           (``.every`` / ``Pvar`` / ``linvar`` / ``Clock.future``)
           — статичный повтор 4-8 нот.
           → WARNING, чтобы LLM исправила до того, как юзер услышит.

        Returns:
            (errors, warnings) — списки строк. errors блокируют выполнение,
            warnings добавляются в result message (LLM их увидит).
        """
        errors: List[str] = []
        warnings: List[str] = []

        # 1. Абсолютные частоты — жёсткий запрет (только ступени).
        freq_match = _ABSOLUTE_FREQ_RE.search(code)
        if freq_match:
            errors.append(
                "Абсолютные частоты запрещены (Renardo ожидает ступени): "
                f"'{freq_match.group(0)}'. Используй степени, например "
                "p1 >> pluck([0,4,7]) или p1 >> pluck([0,4,7], oct=3)."
            )

        # 1b. chop= (не ноль) — щелчки на 16 kHz DAC вместо sidechain.
        if _CHOP_RE.search(code):
            errors.append(
                "chop= запрещён — на 16 kHz DAC даёт щелчки, а не sidechain. "
                "Для дакинга используй amplify=var([1,0.3],[0.5,0.5])."
            )

        # 1c. spack= с ненулевым паком — сырые глитчевые сэмплы.
        if _SPACK_NONZERO_RE.search(code):
            errors.append(
                "spack=1 (пак 1_pitchglitch_samples) запрещён — сырые "
                "глитчевые сэмплы звучат как «звук из базы». Используй "
                "дефолтный пак (без spack=) или sample=P[0,1,2,3]."
            )

        # 2. dur= у каждого не-play плеера — soft warning.
        players = list(_PLAYER_LINE_RE.finditer(code))
        for m in players:
            player_name, synth, args = m.group(1), m.group(2), m.group(3)
            if synth == "play":
                continue  # play() имеет dur из строки паттерна
            if "dur" not in args:
                warnings.append(
                    f"У '{player_name} >> {synth}(...)' нет dur= — паттерн "
                    "будет звучать как staccato-щелчки. Добавь dur (например "
                    "dur=0.5 или dur=[0.5,0.25])."
                )

        # 3. Многоголосный трек без развития — soft warning.
        if len(players) >= 2 and not _DEV_PATTERN_RE.search(code):
            warnings.append(
                "В коде нет развивающих паттернов (.every/Pvar/linvar/"
                "Clock.future) — трек будет звучать как статичный луп из "
                "4-8 нот. Добавь хотя бы один: .every(4, 'stutter'), "
                "lpf=linvar([500,4000], 16) или Pvar-гармонию."
            )

        return errors, warnings

    # ------------------------------------------------------------------
    # Issue #1804 — d4+/p4+ не звучат на роботе, кода-стражи не было
    # ------------------------------------------------------------------

    def _remap_illegal_slots(self, code: str) -> Tuple[str, Optional[str]]:
        """Переставить d4+/p4+/s*/l* в свободный d1-d3/p1-p3 (issue #1804).

        🔴 FIX (live 31.08, «в траве сидел кузнечик»): модель написала

            p1 >> blip([0,2,4,7,9,7,4,2], dur=0.25, amp=0.4)
            p2 >> dub([0,0,0,-2], dur=0.5, oct=3, amp=0.35)
            p3 >> play("X..X..X.", amp=0.25)
            p4 >> play("..o...o.", amp=0.2)     ← не звучит

        Малый барабан пропал без единой ошибки в логах — на роботе physически
        подключены только d1-d3/p1-p3, а p4/d4+ существуют в самом Renardo
        и потому `execute_code` их молча принимал. В промпте это записано
        прямым текстом ("Stay within d1-d3 and p1-p3"), но маленькие модели
        такие правила регулярно нарушают — играть в угадайку с промптом
        больше нельзя, слой должен либо спастись, либо честно провалиться.

        Правило переназначения: play(...) — это обычно барабаны/перкуссия
        → предпочитаем d-слот; любой другой синт (мелодия/бас/пэд) →
        предпочитаем p-слот. Так совпадает с разводкой ролей в
        ``core/arranger.ROLE_PROFILE``. Если предпочитаемая категория уже
        занята — пробуем вторую перед тем, как сдаться. Один и тот же
        недопустимый токен (например, второе упоминание ``p4``) всегда
        переезжает в один и тот же новый слот, чтобы не расщепить один
        логический слой на два разных плеера.

        Returns:
            ``(код, None)`` если всё поместилось в 6 слотов, либо
            ``(исходный_код, сообщение_об_ошибке)`` если слотов не хватило
            — исходный код НЕ должен уходить в Renardo в этом случае.
        """
        occupied: set = {
            m.group("name")
            for m in _PLAYER_ASSIGN_RE.finditer(code)
            if m.group("name") in _ALLOWED_PLAYER_SLOTS_SET
        }
        remapped: Dict[str, str] = {}
        errors: List[str] = []

        def _remap(m: re.Match) -> str:
            name = m.group("name")
            synth = m.group("synth")
            if name in _ALLOWED_PLAYER_SLOTS_SET:
                return m.group(0)
            if name in remapped:
                new_name = remapped[name]
            else:
                preferred = (
                    _ALLOWED_PLAYER_SLOTS
                    if synth == "play"
                    else _ALLOWED_PLAYER_SLOTS[3:] + _ALLOWED_PLAYER_SLOTS[:3]
                )
                free = next((slot for slot in preferred if slot not in occupied), None)
                if free is None:
                    errors.append(
                        f"'{name} >> {synth}(...)' вне d1-d3/p1-p3, а все "
                        "6 слотов уже заняты — слой некуда переставить. "
                        "Убери один из существующих слоёв или объедини "
                        "паттерны."
                    )
                    return m.group(0)
                occupied.add(free)
                remapped[name] = free
                new_name = free
            return f"{m.group('indent')}{new_name}{m.group('arrow')}{synth}("

        fixed_code = _PLAYER_ASSIGN_RE.sub(_remap, code)
        if errors:
            return code, "⛔ Недопустимые слоты плееров: " + " ".join(errors)
        return fixed_code, None

    # ------------------------------------------------------------------
    # Issue #1803 — рисунок play(...), который не делит такт, плывёт
    # ------------------------------------------------------------------

    def _fix_pattern_length(self, code: str) -> str:
        """Достроить рисунок play("...") до степени двойки (issue #1803).

        🔴 FIX (живые прогоны 30-31.08, четыре трека подряд): модель писала
        рисунки, чья длина не делит такт —

            d1 >> play("X..X.o...")   9 шагов
            d2 >> play("=..=...=")    8 шагов

        9 не кратно 8: уже со второго повтора d1 и d2 расходятся по фазе
        друг с другом, и грув «плывёт» — это особенно слышно в жанрах,
        где сетка обязана стоять намертво (диско, метал). Модель символы
        не считает и считать не научится — длина приводится к ближайшей
        СВЕРХУ степени двойки. Округление вверх, а не вниз: степень
        двойки всегда кратна всем меньшим степеням двойки, поэтому
        дополненный рисунок остаётся в фазе с любым другим рисунком той
        же природы, а округление вниз обрезало бы последний удар модели.

        🔴 FIX (ревью после первого прохода): добивка ставилась символом
        ``-``. Это НЕ пауза в FoxDot/Renardo — ``-`` маппится на реальный
        сэмпл (``"hyphen"``, ``renardo_gatherer/collections.py``) и лежит
        в каждом сэмпл-паке (``samples/0_foxdot_default/_/hyphen``), т.е.
        это звучащий хэт. Семь ``-`` на конце девятишагового рисунка
        добавляли модели семь ударов, которых она не писала — грув менялся
        сильнее, чем исходное уползание по фазе, которое чинил этот метод.
        Настоящая пауза — ``.`` (для неё сэмпл-каталога нет ни в одном
        паке); ею и добиваем.
        """

        def _pad(m: re.Match) -> str:
            ws, quote, pattern = m.group(1), m.group(2), m.group(3)
            n = len(pattern)
            if n <= 1:
                return m.group(0)
            target = 1
            while target < n:
                target *= 2
            if target == n:
                return m.group(0)
            padded = pattern + "." * (target - n)
            return f"play({ws}{quote}{padded}{quote}"

        return _PLAY_PATTERN_LEN_RE.sub(_pad, code)

    def _cap_amp(self, code: str) -> str:
        """Ограничить громкость/октаву в коде до безопасных пределов.

        Issue #1000 — phase-3.2 anti-click caps:
        - ``amp=0.9``               → ``amp=0.7`` (если max_amp=0.7)
        - ``amp=P[0.5, 1.0]``       → ``amp=P[0.5, 0.7]``
        - ``amp=1``                 → ``amp=0.7``
        - ``amplify=var([1,0.3])``  → ``amplify=var([0.7,0.3])``
        - ``amplify=0.8``           → ``amplify=0.7``
        - ``oct=9``                 → ``oct=6`` (санитарный потолок)

        Октавный потолок (RC2 в docs/analysis/2026-08-30-music-quality-audit.md):
        раньше здесь стояло ``max_oct = 4`` с обоснованием «oct=5 очень
        резкое/громкое» (issue #1000). Резкость oct=5 — это алиасинг на
        16 kHz, а не громкость. Кап до 4 при этом схлопывал бас (oct=3) и
        лид в соседние октавы: аранжировка без регистрового разделения на
        слух и есть «одна мелодия, которая повторяется».

        🔴 FIX (live 31.08): потолок был поднят до 6 в расчёте на то, что
        алиасинг срежет LPF внутри ``masterlimiter``. Лимитер снят (он
        выдавал NaN и глушил весь выход), и расчёт вместе с ним рухнул.
        Живой прогон: модель написала ``bell(..., oct=7)``, кап опустил до
        6 — и робот засвистел. Обертоны колокола на шестой октаве лежат
        выше Найквиста (8 kHz) и зеркалятся обратно негармоничным визгом;
        ``fuzz(drive=0.6)`` и ``play(rate=1.2)`` в том же коде добавляли
        своих.

        Потолок 5 покрывает регистры аранжировщика целиком (бас 3, пэд 4,
        мелодия 5) — режется только то, что модель пишет от руки выше них.
        Вернуть 6 можно, когда на мастер-шине снова будет анти-алиасинговый
        фильтр — но уже с защитой от NaN.
        """
        max_amp = self._max_amp
        max_oct = 5

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
        timeout_s = max(
            segments * bar_duration_s * self.SEGMENTS_DEADLINE_SAFETY_FACTOR,
            self.MIN_SEGMENTS_DEADLINE_SECONDS,
        )
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

        # Issue #1016 — music-quality guardrail: блокируем абсолютные
        # частоты (только ступени), предупреждаем про dur= и статичные
        # лупы. errors → код НЕ уходит в Renardo; warnings → LLM увидит
        # их в result message и исправит следующим вызовом.
        quality_errors, quality_warnings = self._validate_music_code(code)
        if quality_errors:
            return {
                "success": False,
                "error": "⛔ Код отклонён музыкальным валидатором: "
                + " ".join(quality_errors),
                "code": code,
            }

        # Issue #1804 — d4+/p4+ физически не звучат на роботе. Переставляем
        # слой в свободный d1-d3/p1-p3; если свободных слотов не осталось —
        # честная ошибка вместо тихо потерянного слоя (см. #1804 и
        # ``_remap_illegal_slots`` выше).
        code, slot_error = self._remap_illegal_slots(code)
        if slot_error:
            return {"success": False, "error": slot_error, "code": code}

        # 🔴 FIX (live 11:41 «цоканье»): автозамена pianovel/piano → rhpiano
        # (обе используют MdaPiano физмодель — цокает/щёлкает; rhpiano —
        # компилируемый и чистый). LLM продолжает писать pianovel несмотря
        # на запрет в промпте → защита на уровне кода. Взято из ветки
        # phase-3.2-music-testing (проверено в live-экспериментах юзера).
        if "pianovel" in code:
            code = code.replace("pianovel", "rhpiano")
        # piano заменяем только если это отдельное слово (не rhpiano, pianovel и т.д.)
        code = re.sub(r"(?<![a-zA-Z])piano(?![a-zA-Z])", "rhpiano", code)

        # Issue #1803 — рисунок play(...), чья длина не делит такт, плывёт
        # относительно соседних слоёв на каждом повторе (см.
        # ``_fix_pattern_length`` выше).
        code = self._fix_pattern_length(code)

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

        # 🔴 FIX (live 13.08): предзагружаем сэмпл-буферы для play("...")
        # ДО exec — иначе первая запланированная нота бьёт в PlayBuf, пока
        # scsynth ещё читает файл в буфер (Buffer UGen: no buffer data),
        # и на старте музыки слышен резкий свист/хруст (xrun-бурст).
        self._prewarm_sample_buffers(code)

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

        # Мастер-фейдер применяем лениво, на первом успешном выполнении:
        # ``foxdot_init.sc`` ставит синт ``masterlimiter`` через ~5 с после
        # старта sclang, а MusicManager конструируется раньше — отправка из
        # __init__ пришла бы в несуществующую ноду.
        if not self._master_gain_applied:
            self._master_gain_applied = True
            self.set_master_gain(self._master_gain)

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

        # Issue #1016 — quality warnings surfaced to the LLM so it can fix
        # them on the next call (e.g. add dur=, add a developing pattern).
        if quality_warnings:
            return {
                "success": True,
                "message": "Код выполнен успешно. ⚠️ " + " ".join(quality_warnings),
                "code": code,
            }

        return {"success": True, "message": "Код выполнен успешно", "code": code}

    def _prewarm_sample_buffers(self, code: str) -> None:
        """Pre-allocate sample buffers for ``play("...")`` symbols.

        Live 13.08: ``play("x-o-")`` стартовал в тот же тик, что и
        ``/b_allocRead`` — scsynth логировал ``Buffer UGen: no buffer
        data`` и на старте музыки слышался резкий свист/xrun-бурст.
        Renardo кэширует буферы в ``Samples`` (BufferManager), поэтому
        предзагрузка до ``exec`` — это cache-hit и для самого renardo.

        Args:
            code: FoxDot-код, который сейчас выполнится.
        """
        try:
            samples = self._renardo_context.get("Samples")
            if samples is None:
                return
            for match in _PLAY_SYMBOLS_RE.finditer(code):
                for symbol in match.group(1):
                    # 🔴 FIX (issue #1815): раньше тут пропускался и "-", с
                    # комментарием "пауза (rest)" — НЕВЕРНО. "-" звучащий
                    # сэмпл (renardo_gatherer/collections.py:27 маппит его
                    # на каталог "hyphen", он есть в 0_foxdot_default/_/ и
                    # в 1_pitchglitch_samples/_/ на роботе). Настоящая пауза
                    # — "." (для неё каталога нет ни в одном сэмпл-паке).
                    # "-" — САМЫЙ ходовой символ хэтов (`play("--.-")` почти
                    # в каждом треке), то есть функция не прогревала буфер
                    # именно там, где щелчок/xrun наиболее вероятен — ровно
                    # тот риск, ради которого её и писали. Пробел — просто
                    # разделитель форматирования паттерна, сэмплов не несёт.
                    if symbol.isspace() or symbol == ".":
                        continue
                    try:
                        samples.getBufferFromSymbol(symbol, 0)
                    except Exception:  # noqa: BLE001 — символ может не иметь сэмпла
                        continue
        except Exception:  # noqa: BLE001 — предзагрузка не должна ломать exec
            return

    def _resolve_pattern_name(self, pattern_name: str) -> Tuple[bool, str]:
        """Проверить имя паттерна по whitelist перед остановкой.

        Разрешены только: (а) встроенные плееры Renardo (d1-d9, p1-p9,
        s1-s9, l1-l9) и (б) имена, которые мы сами зарегистрировали через
        :meth:`execute_code`. Всё остальное — включая попытки протащить
        код (``p1.stop(); __import__('os')...``) — отклоняется.

        Args:
            pattern_name: Имя из tool-call-а LLM.

        Returns:
            (is_valid, error_message) — (True, "") если имя допустимо.
        """
        if not isinstance(pattern_name, str) or not _PATTERN_NAME_RE.match(
            pattern_name
        ):
            return False, (
                "Недопустимое имя паттерна — ожидается идентификатор "
                "вида 'p1' или 'bass'."
            )
        known = (
            _RENARDO_PLAYER_NAMES
            | set(self._active_patterns)
            | set(self._pattern_history)
        )
        if pattern_name not in known:
            if self._active_patterns:
                available = ", ".join(sorted(self._active_patterns))
                return False, (
                    f"Неизвестный паттерн '{pattern_name}'. "
                    f"Активны: {available}."
                )
            return False, (
                f"Неизвестный паттерн '{pattern_name}' — "
                "активных паттернов нет."
            )
        return True, ""

    def _call_player_stop(self, pattern_name: str) -> None:
        """Вызвать ``.stop()`` у плеера Renardo без ``exec()``.

        Имя уже прошло :meth:`_resolve_pattern_name`, но мы всё равно
        достаём объект через ``dict.get`` и вызываем метод напрямую —
        так строка от LLM никогда не становится кодом.

        Args:
            pattern_name: Проверенное имя плеера.
        """
        player = self._renardo_context.get(pattern_name)
        if player is None:
            return
        stop = getattr(player, "stop", None)
        if callable(stop):
            stop()

    def stop_pattern(self, pattern_name: str) -> Dict[str, Any]:
        """Остановить именованный паттерн.

        Не требует наличия паттерна в истории — LLM может вызвать stop для
        любого player (d1, p1, ...) даже если execute_code не сохранял по имени.

        Issue G-MUSIC: even when the sclang startup is degraded we still drop
        ``pattern_name`` from ``_active_patterns`` (no live SC nodes to worry
        about), but we tell the caller that music is unavailable so the LLM
        can short-circuit further tool calls.

        Security: ``pattern_name`` приходит от LLM (и, через отравленный
        результат ``search_web``, потенциально от третьей стороны). Раньше
        оно подставлялось в ``f"{pattern_name}.stop()"`` и уходило в
        ``exec()`` — то есть было прямым RCE. Теперь имя проверяется по
        whitelist (:meth:`_resolve_pattern_name`), а сам плеер достаётся
        поиском по namespace-у Renardo, без сборки и выполнения кода.

        Args:
            pattern_name: Имя паттерна/плеера (d1, p1, bass и т.д.).

        Returns:
            dict с ключами ``success`` и ``message`` (или ``error``).
        """
        name_ok, name_error = self._resolve_pattern_name(pattern_name)
        if not name_ok:
            return {"success": False, "error": name_error}

        stop_error: Optional[str] = None
        degraded = self._require_healthy and not self.is_music_stack_healthy()

        if not degraded and self._renardo_available and self._check_supercollider():
            try:
                self._call_player_stop(pattern_name)
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
            segments_for_log = self._music_deadline_segments
            stop_result = self.stop_all()
            result["stopped"] = True
            result["stop_reason"] = "segments_deadline"
            result["deadline_segments"] = segments_for_log
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
        result["stop_reason"] = "idle_ttl"
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


class ComposeMusicTool(MCPTool):
    """Сыграть трек С ФОРМОЙ: модель даёт материал, аранжировщик — развитие.

    Отличие от ``execute_music_code``: тот выполняет готовый код и играет
    его неизменно до остановки (отсюда жалоба «однотипная мелодия, которая
    повторяется»). Здесь модель описывает только материал, а секции,
    вступление и уход слоёв, брейк и кульминацию строит
    :mod:`rob_box_mcp_tools.core.arranger`.

    См. docs/analysis/2026-08-30-music-quality-audit.md (RC4).
    """

    def __init__(self, node, manager: MusicManager) -> None:
        super().__init__(node)
        self._manager = manager

    @property
    def name(self) -> str:
        return "compose_music"

    @property
    def description(self) -> str:
        return (
            "Сыграть музыкальную композицию С РАЗВИТИЕМ (вступление, "
            "нарастание, кульминация, брейк, финал). Ты описываешь только "
            "МАТЕРИАЛ — темп, тональность, лад и по несколько нот для баса, "
            "мелодии и подклада; форму и то, когда какой слой вступает и "
            "уходит, система строит сама. Используй ЭТОТ инструмент для "
            "любой просьбы сыграть музыку, трек, бит или сет. "
            "execute_music_code нужен только для точного воспроизведения "
            "известной мелодии по нотам."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="bpm",
                type="number",
                description="Темп, 60-180. Медленное и лиричное 70-95, "
                "грув 100-120, танцевальное 124-140.",
                required=True,
            ),
            MCPToolParameter(
                name="root",
                type="string",
                description="Тоника: C, D, E, F, G, A, B (можно с #).",
                required=True,
                enum=list(VALID_ROOTS),
                enum_strict=False,
            ),
            MCPToolParameter(
                name="scale",
                type="string",
                description="Лад: minor, major, dorian, mixolydian, lydian, "
                "phrygian, majorPentatonic, harmonicMinor.",
                required=True,
            ),
            MCPToolParameter(
                name="form",
                type="string",
                description=(
                    "Форма композиции. "
                    "arc — универсальная дуга; "
                    "verse_chorus — куплет-припев; "
                    "buildup — клубная с дропом; "
                    "ambient — без ударных, для спокойного и лиричного."
                ),
                required=False,
                enum=sorted(FORMS),
                enum_strict=False,
            ),
            MCPToolParameter(
                name="drums",
                type="string",
                description='Паттерн бочки/малого одной строкой, например '
                '"X..o.X.o" или "X.X.X.X.". Пропусти для музыки без ударных.',
                required=False,
            ),
            MCPToolParameter(
                name="drums_sample",
                type="integer",
                description="Индекс набора ударных 0-4. Меняй его между "
                "треками, иначе все треки звучат одинаково.",
                required=False,
            ),
            MCPToolParameter(
                name="hats",
                type="string",
                description='Паттерн хэтов, например "--.-" или "-.--".',
                required=False,
            ),
            MCPToolParameter(
                name="hats_sample",
                type="integer",
                description="Индекс сэмпла хэтов 0-4. Раньше был прибит к 3, "
                "поэтому хэты во всех треках звучали одинаково. Меняй.",
                required=False,
            ),
            MCPToolParameter(
                name="perc",
                type="string",
                description='Паттерн перкуссии — третий ударный слой поверх '
                'бочки и хэтов, например "..n." или "n..n.n". Форма отводит '
                'ему место в кульминации; без него плотные секции пустее.',
                required=False,
            ),
            MCPToolParameter(
                name="perc_sample",
                type="integer",
                description="Индекс сэмпла перкуссии 0-4.",
                required=False,
            ),
            MCPToolParameter(
                name="bass_synth",
                type="string",
                description="Синт баса: dub, wobblebass, fuzz, bass, jbass, "
                "retrobass, tb303, moogbass.",
                required=False,
            ),
            MCPToolParameter(
                name="bass_notes",
                type="string",
                description='Ступени лада для баса через запятую, например '
                '"0, 0, 3, -2". Держи 2-5 нот.',
                required=False,
            ),
            MCPToolParameter(
                name="lead_synth",
                type="string",
                description="Синт мелодии: blip, arpy, supersawlead, karp, "
                "sitar, marimba, bell, cs80lead, pluck, keys.",
                required=False,
            ),
            MCPToolParameter(
                name="lead_notes",
                type="string",
                description='Ступени лада для мелодии, например '
                '"0, 2, 4, 7, 4, 2". Держи 4-8 нот — это мотив, а не гамма.',
                required=False,
            ),
            MCPToolParameter(
                name="pad_synth",
                type="string",
                description="Синт подклада: warmpad, pads, strings, ambi, "
                "space, sinepad, viola.",
                required=False,
            ),
            MCPToolParameter(
                name="pad_notes",
                type="string",
                description='Аккорд подклада, например "0, 4, 7".',
                required=False,
            ),
            MCPToolParameter(
                name="progression",
                type="string",
                description='Движение тоники по ступеням, например '
                '"0, 0, 5, 3". Даёт гармоническое развитие — с ним трек '
                "заметно живее. Пропусти для статичной гармонии.",
                required=False,
            ),
            MCPToolParameter(
                name="repeat",
                type="boolean",
                description="true — форма зацикливается (диджей-сет, фон под "
                "речь). false — трек заканчивается сам после одной формы.",
                required=False,
            ),
            MCPToolParameter(
                name="swing",
                type="number",
                description="Свинг восьмых, 0-0.3. 0 (по умолчанию) — ровная "
                "сетка, подходит большинству жанров. Ставь 0.1-0.2 для "
                "джаза, блюза, свинга, шафла, фанка — на ровных восьмых "
                "они не звучат как жанр независимо от инструментов.",
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
        bpm: float,
        root: str,
        scale: str,
        form: Optional[str] = None,
        drums: Optional[str] = None,
        drums_sample: int = 0,
        hats_sample: int = 3,
        perc: Optional[str] = None,
        perc_sample: int = 0,
        hats: Optional[str] = None,
        bass_synth: Optional[str] = None,
        bass_notes: Optional[str] = None,
        lead_synth: Optional[str] = None,
        lead_notes: Optional[str] = None,
        pad_synth: Optional[str] = None,
        pad_notes: Optional[str] = None,
        progression: Optional[str] = None,
        repeat: bool = True,
        swing: float = 0.0,
    ) -> MCPToolResult:
        try:
            spec = spec_from_flat(
                bpm=bpm,
                root=root,
                scale=scale,
                form=form or "arc",
                drums=drums,
                drums_sample=drums_sample,
                hats_sample=hats_sample,
                perc=perc,
                perc_sample=perc_sample,
                hats=hats,
                bass_synth=bass_synth,
                bass_notes=bass_notes,
                lead_synth=lead_synth,
                lead_notes=lead_notes,
                pad_synth=pad_synth,
                pad_notes=pad_notes,
                progression=progression,
                repeat=repeat,
                swing=swing,
            )
            code = render(spec)
        except ArrangementError as exc:
            # Сообщение аранжировщика написано так, чтобы модель могла
            # исправиться следующим вызовом, а не гадать.
            return MCPToolResult(success=False, error=str(exc))

        self.log_info(f"Композиция: {form_summary(spec.form)}")
        result = self._manager.execute_code(code, pattern_name="composition")
        if not result["success"]:
            return MCPToolResult(success=False, error=result["error"])

        self._notify_music_state()
        result["form"] = form_summary(spec.form)
        # Явный стоп-сигнал в сообщении, а не только в промпте: live 30.08
        # модель вызвала compose_music и следом execute_music_code со своим
        # кодом. Любой музыкальный вызов начинается с Clock.clear(), поэтому
        # второй вызов стирает только что построенную аранжировку — трёх-
        # минутная композиция превращается в четырёхтактовый луп.
        return MCPToolResult(
            success=True,
            data=result,
            message=(
                f"Играю композицию: {form_summary(spec.form)}. "
                "Музыка уже звучит — НЕ вызывай execute_music_code после "
                "этого, иначе аранжировка будет стёрта."
            ),
        )

    def _notify_music_state(self) -> None:
        """Опубликовать /voice/music/state (issue 989 Fix C)."""
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
        # Issue #1392 follow-up: остановка сгенерированного mp3-трека в
        # sound_node. Graceful-degrade если publisher недоступен (тесты).
        self._sound_stop_pub = None
        self._generated_music_state_pub = None
        try:
            from std_msgs.msg import String

            if hasattr(node, "create_publisher"):
                self._sound_stop_pub = node.create_publisher(
                    String, "/voice/sound/stop", 10
                )
                self._generated_music_state_pub = node.create_publisher(
                    String, "/voice/generated_music/state", 10
                )
        except Exception:  # noqa: BLE001 — unit tests / minimal install
            self._sound_stop_pub = None
            self._generated_music_state_pub = None

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
            # Issue #1392 follow-up: останавливаем и mp3-трек в sound_node.
            self._notify_sound_stop()
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

    def _notify_sound_stop(self) -> None:
        """Остановить mp3-трек в sound_node + сбросить состояние (issue #1392).

        Одна точка правды — ``McpServerNode.stop_generated_track_playback``:
        те же два топика нужны ещё и ``music_cleanup``, и watchdog'у, а
        раньше их публиковал только этот тул, из-за чего mp3 переживал и
        конец диалога, и авто-стоп (live 30.08).
        """
        delegate = getattr(self.node, "stop_generated_track_playback", None)
        if callable(delegate):
            try:
                delegate()
                return
            except Exception as exc:  # noqa: BLE001
                self.log_warning(f"stop_generated_track_playback упал: {exc}")
        try:
            from std_msgs.msg import String

            if self._sound_stop_pub is not None:
                msg = String()
                msg.data = "STOP"
                self._sound_stop_pub.publish(msg)
            if self._generated_music_state_pub is not None:
                state = String()
                state.data = json.dumps({"status": "idle"})
                self._generated_music_state_pub.publish(state)
        except Exception as exc:  # noqa: BLE001
            self.log_warning(f"Не удалось опубликовать sound_stop: {exc}")


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

    #: Кириллица → латиница для ``_slug``.
    #:
    #: 🔴 FIX (live 30.08, vision-pi): старый ``_slug`` заменял КАЖДЫЙ
    #: не-ASCII символ на «_», поэтому любое русское имя превращалось в
    #: строку подчёркиваний той же длины. В живой медиатеке лежит
    #: ``('________________', 'комната_мудрости')``, а «тисбит» и «мурка»
    #: столкнулись бы в один slug, будь они одной длины. Юзер просил
    #: «сохрани как трек тисбит» — LLM, зная про это, каждый раз сама
    #: придумывала латинский slug и придумывала РАЗНЫЙ: в базе лежат
    #: ``tisbeat``, ``tisbit``, ``thisbit``, ``tinbit`` — четыре записи
    #: одного трека, и ни одну из них не находит «удали трек тисбит».
    _TRANSLIT: dict = {
        "а": "a", "б": "b", "в": "v", "г": "g", "д": "d", "е": "e",
        "ё": "e", "ж": "zh", "з": "z", "и": "i", "й": "y", "к": "k",
        "л": "l", "м": "m", "н": "n", "о": "o", "п": "p", "р": "r",
        "с": "s", "т": "t", "у": "u", "ф": "f", "х": "h", "ц": "ts",
        "ч": "ch", "ш": "sh", "щ": "sch", "ъ": "", "ы": "y", "ь": "",
        "э": "e", "ю": "yu", "я": "ya",
    }

    @classmethod
    def _slug(cls, name: str) -> str:
        """Имя трека → стабильный ASCII-slug.

        Кириллица транслитерируется, всё остальное не-ASCII схлопывается
        в «_», повторные «_» склеиваются. «тисбит» → ``tisbit`` при любом
        регистре, поэтому «сохрани как тисбит» и «удали трек тисбит»
        попадают в одну запись.
        """
        lowered = (name or "").lower().strip()
        translit = "".join(cls._TRANSLIT.get(ch, ch) for ch in lowered)
        slug = re.sub(r"[^a-z0-9_]", "_", translit)
        # Схлопываем подряд идущие «_», чтобы «комната мудрости» не
        # превращалась в частокол и чтобы slug оставался читаемым.
        slug = re.sub(r"_+", "_", slug).strip("_")
        return slug

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
                description=(
                    "Имя трека — передавай РОВНО ТО СЛОВО, которым его назвал "
                    "пользователь, включая русское («тисбит», «мурка»). "
                    "Библиотека сама транслитерирует и нормализует его в slug, "
                    "так что «Тисбит», «ТисБит» и «тисбит» попадут в ОДНУ "
                    "запись, и «удали трек тисбит» её найдёт. "
                    "❌ НЕ придумывай свою латинскую транскрипцию: живой лог "
                    "30.08 — один и тот же «тисбит» лёг в базу четырьмя "
                    "записями (tisbeat, tisbit, thisbit, tinbit), и удалить "
                    "его стало нечем."
                ),
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
        #: Счётчик поворота окна результатов. Раньше поиск всегда отдавал
        #: алфавитно первые совпадения, поэтому ``search_samples("kick")``
        #: возвращал один и тот же ответ при каждом вызове и LLM выбирала
        #: одни и те же сэмплы во всех генерациях — не потому что «хотела»,
        #: а потому что остальной коллекции для неё не существовало.
        self._rotation: int = 0

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

        result = search_renardo_samples(
            self._samples_path, query, pack, case, rotate=self._rotation
        )
        self._rotation += 1

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
        # Показываем реальный размер коллекции, а не размер окна: «найдено
        # 30» при двухстах доступных заставляет LLM считать набор бедным и
        # переиспользовать первое попавшееся.
        total = result.get("total_found", found)
        play_codes = [r["play_code"] for r in results_list[:5]]
        suffix = f" ... и ещё {total - len(play_codes)}" if total > len(play_codes) else ""
        return MCPToolResult(
            success=True,
            data=result,
            message=(
                f"Найдено {total} сэмплов по запросу '{query}': "
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
