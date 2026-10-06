#!/usr/bin/env python3
"""
music.py - Инструменты для управления музыкой в реальном времени через Renardo

Модуль предоставляет:
- MusicManager: Базовый класс управления Renardo (история паттернов, SC-проверка, фильтрация кода)
- TrackLibrary: Персистентная медиатека треков (JSON на диске)
- ExecuteMusicCodeTool: Выполнить Renardo-код в безопасном контексте
- StopMusicTool: Остановить паттерны или всю музыку
- GetMusicStateTool: Получить текущее состояние музыки и историю паттернов
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
from typing import Any, Dict, List, Optional, Tuple

from rob_box_voice.core.music_stack_validation import (
    MusicStackStatus,
    load_confirmed_synths,
    load_sclang_health,
)
from rob_box_voice.core.sc_only_custom_synthdefs import (
    CUSTOM_SC_ONLY_SYNTH_NAMES,
    register_sc_only_custom_synthdefs,
)

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType, shared_publisher
from ..core import renardo_sanitizer, sample_loops
from ..core.music_stack_health import MusicStackHealth  # ADR-0134 §5 Phase 2
from ..engine import renardo_adapter
from ..engine.search import find
from ..core.rtttl_library import RtttlLibrary, display_title, match_info

# Live 13.08 — символы сэмплов в play("x-o-") для предзагрузки буферов.
_PLAY_SYMBOLS_RE = re.compile(r'play\(\s*"([^"]*)"')


def _search_alternatives(
    library: Optional["RtttlLibrary"],
    query: Optional[str],
    chosen_title: Optional[str],
    limit: int = 4,
) -> List[Dict[str, Optional[str]]]:
    """До ``limit`` соседних результатов ``search(query)``, кроме выбранной записи."""
    if not query or library is None:
        return []
    alternatives: List[Dict[str, Optional[str]]] = []
    try:
        hits = library.search(query, limit=max(limit * 2, 6))
        for hit in hits:
            title = hit.get("title") or hit.get("name")
            if title == chosen_title:
                continue
            alternatives.append({"name": hit.get("name"), "title": title})
            if len(alternatives) >= limit:
                break
    except Exception:  # noqa: BLE001 — альтернативы необязательны, не мокнутый
        # search() в тестах (RtttlLibrary заменён на Mock() без .search)
        # не должен ронять весь вызов — альтернативы просто пустые.
        return []
    return alternatives


# ---------------------------------------------------------------------------
# issue #2964 — прозрачность вместо молчаливой подмены песни
# ---------------------------------------------------------------------------
# ``RtttlLibrary.get()`` (см. её докстринг) сознательно всегда возвращает
# лучшего ПО ТЕКСТУ кандидата — даже при слабом совпадении (issue #2896).
# Раньше единственной защитой было предупреждение в message: «сверь title
# с тем, что просил юзер». Живой прогон 24.09 показал, что LLM (minimax)
# это предупреждение игнорирует и играет что дали.
#
# Товарищ Шифу (правка к #2964): жёсткий порог/гейт по этому поводу НЕ
# нужен — история отката #2882→#2896 показала, что единая эвристика
# отказа под одно слабое совпадение («stranger things» → None) ломает
# сильные («super mario»). Вместо гейта — ``core.rtttl_library.match_info``/
# ``display_title`` (IDF-вес по корпусу, БЕЗ хардкод-списка стоп-слов) дают
# тулу и промпту скилла composer честную структурированную сверку; решение
# «это та же песня или нет» остаётся у модели по общему правилу в промпте.


def _resolve_melody_with_candidate(
    library: Optional["RtttlLibrary"],
    name: str,
    variants: Optional[List[str]],
) -> Tuple[Optional[str], Optional[Dict[str, Any]]]:
    """Пройти ``name`` → ``variants`` по порядку, вернуть первое найденное.

    Тот же порядок кандидатов, что и раньше (``name`` первым, затем
    ``variants``) — это НЕ вердикт «эта ли песня», просто первая запись,
    которую ``library.get()`` вообще нашла хоть как-то (issue #2896: он
    всегда возвращает лучшего по тексту кандидата). Возвращает
    ``(candidate, rec)`` — ``candidate`` нужен вызывающей стороне, чтобы
    строить ``alternatives``/``match_info`` против ТОГО ЖЕ запроса, что
    реально нашёл запись. ``(None, None)``, если ничего не нашлось вовсе.
    """
    if library is None:
        return None, None
    # #3399: слова запроса целиком нашлись в записи после разбора (падежи, транслит, служебные слова) —
    # «терминатора» → ``terminat``; запрос, который ``get`` и так покрывает, даёт ту же запись.
    hit = find(library, name)
    if hit.found and hit.confidence >= 1.0:
        return hit.query, hit.record
    for candidate in [name] + [v for v in (variants or []) if v]:
        rec = library.get(candidate)
        if rec is not None:
            return candidate, rec
    return None, None


def _mismatch_note(match: Optional[Dict[str, Any]], requested: str) -> str:
    """Текст-предупреждение, если запрос покрыт записью не полностью.

    Не вердикт (см. модульный докстринг выше про issue #2964) — явно
    называет, какие значимые слова запроса не нашлись в найденной записи,
    чтобы модель сама решила, объявлять ли найденное под именем из
    запроса (общее правило — в промпте скилла composer, не здесь).
    """
    if not match or not match.get("unmatched"):
        return ""
    unmatched = ", ".join(match["unmatched"])
    return (
        f" ⚠️ Из запроса «{requested}» не нашлись слова: {unmatched} — "
        "возможно, это другая песня. Не объявляй найденное под именем из "
        "запроса, если по смыслу это не она."
    )


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

# Код-санация Renardo вынесена в core/renardo_sanitizer (единый seam).


# ---------------------------------------------------------------------------
# MusicManager
# ---------------------------------------------------------------------------


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
    # палитра движка v2 (ADR-0152 PR-6): лиды и бас семей тембров, в прелоаде робота
    "kalimba", "hoover", "keys", "jbass",
)



def _ensure_health_for(mgr: "MusicManager") -> MusicStackHealth:
    """Module-level helper that lazily builds ``MusicManager._health``.

    Lives outside the class on purpose: it lets the host class drop one
    named method while still tolerating ``MusicManager.__new__``-bypassed
    instances (the test factory ``_make_manager`` in
    ``test_tools/test_music.py``). ADR-0134 §5 Phase 2 + ADR-0145.
    """
    health = getattr(mgr, "_health", None)
    if health is None:
        health = MusicStackHealth(mgr)
        object.__setattr__(mgr, "_health", health)
    return health



class MusicManager:
    """Управляет интеграцией с Renardo для LLM-контроля музыки в реальном времени.

    Возможности:
    - Безопасное выполнение Renardo-кода (execute_code)
    - История паттернов с возможностью мутации и остановки по имени
    - Проверка доступности SuperCollider перед воспроизведением
    - Фильтрация опасных системных команд в пользовательском коде
    """

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
    MASTER_LIMITER_NODE: int = renardo_adapter.MASTER_NODE  # одно число на оба пути (PR-7)
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
    #: Issue #3133 — сериализует переходы жизненного цикла сессии: конец
    #: формы (таймер/watchdog) против нового кода и стопа (потоки тулов).
    #: На классе, а не в ``__init__``: часть тестов собирает менеджер через
    #: ``__new__``; менеджер в процессе один (mcp_server). RLock — stop_all
    #: зовут и изнутри других переходов.
    _state_lock = threading.RLock()
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
    #: ``execute_music_code`` instead of letting the LLM
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
        #: Issue #2838 — SynthDef-ы, приход которых в scsynth подтвердил
        #: sclang (строки прелоада "SynthDef in scsynth: X" после
        #: Server.sync). ``None`` — подтверждения нет (лог недоступен или
        #: прелоад не завершён). Пишет ``_evaluate_music_stack_health``.
        self._server_confirmed_synths: Optional[frozenset] = None
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
        #: ``is_healthy is False``, ``execute_music_code``
        #: short-circuit with a clear "music unavailable" error so the LLM
        #: doesn't keep retrying against a broken Renardo/FoxDot upstream.
        self._music_stack_status: MusicStackStatus = MusicStackStatus(
            is_healthy=True,
            oscdef_registered=True,
            missing_synths=(),
            fatal_errors=(),
        )
        #: When True, ``execute_code`` reject calls when
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
        # Issue #1812 — 300s was too short for "listening to a track in
        # silence", which is the normal use case, not an abandoned session.
        # default 30 min — overridable via MUSIC_AUTO_STOP_TTL_SECONDS env
        # (mcp_server.py also exposes this as ``_music_watchdog_idle_ttl_s``
        # and passes it explicitly to ``auto_stop_idle_music``; this default
        # only matters when nobody overrides it).
        default_ttl = 1800
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
        # stats — surfaced via get_state() for the AgentCore safety-net
        self._auto_stop_count: int = 0
        # ------------------------------------------------------------------
        # Music stack health (issue G-MUSIC, architect review v3, ADR-0134 §5)
        # ------------------------------------------------------------------
        # Phase 2 decomposition: the 5 health-related methods now live on
        # ``MusicStackHealth`` (core/music_stack_health.py). We hold a
        # reference here so the rest of ``MusicManager`` can keep using
        # ``self._health`` (delegations on the host stay as shims, see
        # ``# SHIM-remove-after-#3014-phase-6``).
        self._health: MusicStackHealth = MusicStackHealth(self)
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
            self._log_synth_truth_discrepancy()
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

    def attach_logger(self, logger: Any) -> None:
        """Логгер узла для сообщений менеджера (иначе — stderr, см. ниже).

        Issue #3166: строка ``[#3112] … started …`` пишется из колбэка
        ``_rbx_track_started`` в потоке клока Renardo — без логгера узла она
        ушла бы только в stderr.
        """
        self._logger = logger

    def _log_info(self, message: str) -> None:
        """INFO через логгер узла, если он есть (иначе stderr, как warning)."""
        logger = getattr(self, "_logger", None)
        if logger is not None:
            try:
                logger.info(message)
                return
            except Exception:  # noqa: BLE001
                pass
        import sys as _sys
        _sys.stderr.write(f"{message}\n")
        _sys.stderr.flush()

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
        штатные подтверждения, шум для лога ошибок). Разбор — общий с
        владельцем плеера v2 (``engine.renardo_adapter``, ADR-0149 PR-4);
        он же получает отказ через ``osc_fail_listener`` (при v1 не задан).
        """
        detail = renardo_adapter.osc_fail_detail(data)
        if detail is None:
            return
        self._log_warning(f"🔴 [scsynth] FAILURE IN SERVER: {detail}")
        listener = getattr(self, "osc_fail_listener", None)
        if callable(listener):
            listener(detail)

    _split_osc_address = staticmethod(renardo_adapter.split_osc_address)
    _decode_osc_args = staticmethod(renardo_adapter.decode_osc_args)

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
    # Live-инцидент 21.09.2026 — реально загруженные SynthDef-ы (для
    # валидации имён синтов в renardo_sanitizer._validate_synth_names)
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # SHIM-remove-after-#3014-phase-6: thin delegations to
    # ``MusicStackHealth`` (ADR-0134 §5 Phase 2). Bodies live in
    # ``core/music_stack_health.py``; these shims keep the public API of
    # ``MusicManager`` byte-identical for callers (and tests) while the
    # host class is being slimmed down across phases 3-6.
    #
    # The five shims are kept as *named* methods (rather than a single
    # ``__getattr__`` dispatcher) so ``unittest.mock.patch.object`` can
    # rebind them on the class — ``patch.object`` checks
    # ``hasattr(MusicManager, name)`` which is not affected by
    # ``__getattr__`` (it is only consulted for instance attribute
    # lookups). The ``_ensure_health`` helper that lived here in the
    # initial Phase-2 cut was moved to a module-level
    # ``_ensure_health_for`` so the host class shrinks by one method
    # (ADR-0145 class-size ratchet).
    # ------------------------------------------------------------------

    def known_synth_names(self) -> Optional[frozenset]:
        # SHIM-remove-after-#3014-phase-6
        return _ensure_health_for(self).known_synth_names()

    def _log_synth_truth_discrepancy(self) -> None:
        # SHIM-remove-after-#3014-phase-6
        _ensure_health_for(self)._log_synth_truth_discrepancy()

    def _evaluate_music_stack_health(
        self,
        sclang_log_path: Optional[str] = None,
    ) -> MusicStackStatus:
        # SHIM-remove-after-#3014-phase-6
        return _ensure_health_for(self)._evaluate_music_stack_health(
            sclang_log_path=sclang_log_path,
        )

    def is_music_stack_healthy(self) -> bool:
        # SHIM-remove-after-#3014-phase-6
        return _ensure_health_for(self).is_music_stack_healthy()

    def music_stack_unavailable_error(self) -> Dict[str, str]:
        # SHIM-remove-after-#3014-phase-6
        return _ensure_health_for(self).music_stack_unavailable_error()

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

    #: Пауза (сек) между ``gate=0`` и ``/g_freeAll`` в :meth:`_ramp_down_group`
    #: — issue #3137, столько же, сколько #1000 уже использовал в ``stop_all``.
    RAMP_DOWN_RELEASE_SECONDS = 0.05

    #: Пауза (сек) между ``/g_freeAll`` и ``/g_new`` в :meth:`_transition_cleanup`
    #: — issue #778 (``FAILURE IN SERVER /g_new negative node IDs are
    #: reserved``): UDP fire-and-forget, scsynth не успевает освободить ID
    #: группы мгновенно, без паузы пересоздание группы гонится с ещё не
    #: обработанным ``/g_freeAll``. Вынесено в именованную константу (было
    #: инлайновым ``time.sleep(0.05)``), потому что issue #3137 R2 (живой
    #: замер 28.09.2026) считает по ней :data:`TRANSITION_CLEANUP_SAFETY_MARGIN_SECONDS`
    #: — см. её docstring.
    TRANSITION_CLEANUP_G_NEW_PAUSE_SECONDS = 0.05

    #: Запас (сек) перед вычисленным дедлайном (``next_bar + Clock.latency``,
    #: см. :meth:`_transition_cleanup_delay_seconds`) — issue #3137 R2
    #: (живой замер 28.09.2026, второй раунд ревью координатора). Отложенный
    #: ramp/freeAll обязан ЗАВЕРШИТЬСЯ в scsynth строго ДО момента, когда там
    #: материализуется первая нота нового трека — иначе возможны два разных
    #: отказа: (1) freeAll убьёт свежесозданную ноду нового трека вместо
    #: старой (новый трек тоже щёлкнет/оборвётся); (2) хуже — если
    #: материализация нового трека наступит РАНЬШЕ, чем наш ``/g_new``
    #: пересоздаст группу 1 (см. :meth:`_transition_cleanup`), у scsynth
    #: нет группы-цели, когда бандл нового трека пробует создать в ней свою
    #: под-группу (``ServerManager.get_bundle``: первое сообщение бандла —
    #: ``/g_new [group_id, 1, 1]``, ``target=1``) → тот же класс отказа, что
    #: и issue #778 («target node not found»), только для НОВОГО трека, не
    #: для пересоздания группы.
    #:
    #: Поэтому запас обязан целиком покрывать локальный конвейер
    #: :meth:`_transition_cleanup` ОТ момента срабатывания таймера ДО
    #: завершения ``/g_new`` — :data:`RAMP_DOWN_RELEASE_SECONDS` (пауза
    #: перед freeAll) + :data:`TRANSITION_CLEANUP_G_NEW_PAUSE_SECONDS`
    #: (пауза перед g_new, issue #778) = 0.10s, плюс буфер на джиттер треда
    #: таймера и сам OSC round-trip отправки трёх сообщений. Старое значение
    #: (0.08s, R1/#3148) было МЕНЬШЕ этих 0.10s — не баг для R1 (дедлайн
    #: стоял ДО ``next_bar``, а реальная материализация — на ``next_bar +
    #: latency``, ≈0.25s запаса набегало случайно), но стало бы гонкой для
    #: R2, если бы margin не подняли вместе со сдвигом дедлайна на
    #: ``+ latency``.
    TRANSITION_CLEANUP_SAFETY_MARGIN_SECONDS = (
        RAMP_DOWN_RELEASE_SECONDS + TRANSITION_CLEANUP_G_NEW_PAUSE_SECONDS + 0.05
    )  # = 0.15

    #: Потолок для :meth:`_transition_cleanup_delay_seconds` — реалистичный
    #: разрыв (``ALIGN_LEAD_BEATS`` долей на разумном BPM) укладывается в
    #: секунды, но ``bpm`` в ``Clock`` — внешнее, не наше состояние: если
    #: оно когда-нибудь окажется вырожденным (около нуля, битый рантайм),
    #: ``beats_until * 60 / bpm`` даёт секунды порядка ``1e11`` —
    #: ``threading.Timer`` с таким интервалом падает в фоновом потоке
    #: (``OverflowError: timestamp too large to convert to C _PyTime_t``,
    #: живой баг этого ревью: ловится ``test_music_clock_phase.py`` с
    #: ``FakeClock(bpm=1e-9)``). Потолок — и защита от зависшего таймера
    #: (freeAll не должен откладываться дольше, чем не свалить node-table
    #: scsynth, см. комментарий про «too many nodes» выше по файлу), и
    #: защита от невалидного OSC-интервала.
    TRANSITION_CLEANUP_MAX_DELAY_SECONDS = 5.0

    #: Дефолт ``Clock.latency`` в Renardo (``renardo_lib/TempoClock.py``
    #: 0.9.13, строка 121: ``self.latency = 0.25 # Time between starting
    #: processing osc messages and sending to server``) — используется в
    #: :meth:`_transition_cleanup_delay_seconds` ТОЛЬКО как fallback, если
    #: у живого ``Clock`` почему-то нет атрибута ``latency`` (см. её
    #: docstring — issue #3137, живой замер 28.09.2026 после #3148).
    #: Обычное значение читается с самого ``clock.latency``, не хардкодится.
    RENARDO_DEFAULT_CLOCK_LATENCY_SECONDS = 0.25

    def _ramp_down_group(self, group: int = 1) -> None:
        """Плавно погасить живые SC-ноды группы перед ``/g_freeAll`` (anti-click).

        Issue #3137 (живой стык двух треков дал окно −180 dBFS). Первая
        версия этого фикса слала ``/n_set -1 "gate" 0.0``, по аналогии с
        прецедентом — commit ``d922ee836``. Ревью координатора (issue
        #3137, R2) поправило это: nodeID ``-1`` у ``/n_set`` НЕ означает
        «все живые ноды» (в Server Command Reference это не документировано
        как спецзначение для ``/n_set``; спецзначения ``-1``/``-2`` есть у
        ``target`` в ``/g_new``/``/s_new``, не у самого ``/n_set``).
        Правильный адресат — ГРУППА: ``/n_set <group> "gate" 0.0`` ставит
        контрол ``gate`` всем нодам ВНУТРИ группы (Server Command
        Reference, ``/n_set``: «Groups will substitute one message for each
        node in the group»).

        Второе уточнение (то же ревью): из ~370 synthdef'ов Renardo на
        роботе (``SynthDefManagement/sclang_code/scsynth/*.scd``) только
        ~60 имеют контрол ``gate`` — play/сэмплы и большинство мелодических
        синтов (``pluck``, ``blip``, ``saw``, …) используют ``sus``-огибающую
        с ``doneAction``, у них ``gate`` нет вовсе, и ``/n_set`` для них —
        no-op (scsynth либо тихо игнорирует неизвестный контрол, либо не
        находит его на синте). Поэтому для БОЛЬШИНСТВА живых нод этот шаг
        ничего не смягчает — щелчок/обрыв решает не сам ``gate=0``, а то,
        КОГДА вызывается ``freeAll`` относительно последней реальной ноты
        (см. :meth:`_schedule_transition_cleanup` — главный фикс #3137).
        ``gate=0`` остаётся полезным для той минорной доли синтов (включая
        ``MdaPiano``/``rhpiano`` из issue #1000), у которых контрол есть.

        Последовательность:

        1. ``/n_set <group> "gate" 0.0`` — на нодах группы, у которых есть
           ``gate``, запускает release-фазу ADSR; на остальных — no-op.
        2. Пауза :data:`RAMP_DOWN_RELEASE_SECONDS` — дать release начаться.
        3. ``/g_freeAll <group>`` — убить оставшиеся живые ноды группы.

        Не блокирует надолго: суммарная пауза — десятки миллисекунд.

        Args:
            group: номер группы и адресат ``gate=0``/``freeAll`` (по
                умолчанию Group 1 — куда Renardo шлёт все ноты).
        """
        renardo_adapter.ramp_down_group(self._send_osc_raw, group, self.RAMP_DOWN_RELEASE_SECONDS)

    def _transition_cleanup_delay_seconds(self) -> float:
        """Секунд до дедлайна: ``next_bar + Clock.latency`` минус запас.

        Корневая причина живого симптома (issue #3137, найдено ревью
        координатора, R1): дело не в жёсткости ``/g_freeAll`` как такового,
        а в МОМЕНТЕ его вызова. ``execute_code`` раньше слал ``freeAll``
        сразу после ``exec`` — в этот момент старые SC-ноды ещё звучат, а
        новые плееры (зарегистрированные тем же ``exec``) встают на
        ``Clock.next_bar()`` (``Players.py:892``) и реально зазвучат не
        раньше, чем клок дойдёт до этой доли. При типичном арранжировщике
        (``core/arranger.py:clock_align_prelude``, ``ALIGN_LEAD_BEATS=2``)
        это ``next_bar() - now() == 2`` доли, то есть ``2·60/BPM`` секунд —
        ≈0.97с на 124 BPM. Freeall в момент exec убивает старый трек СРАЗУ,
        а новый начинает звучать почти секунду спустя → окно цифровой
        тишины (−180 dBFS), а не щелчок.

        R1 (сдвиг teardown на ``next_bar - запас``, #3148) убрал окно
        цифровой тишины на стыке, но живой замер 28.09.2026 (issue #3137,
        деплой ``82c973e62``) нашёл остаточную просадку ≈0.5с до −75 dB.
        Причина (R2, тот же живой комментарий): ``Clock.next_bar()`` — это
        МОМЕНТ, когда Renardo-клок (``TempoClock.py:583``, фоновый тред
        ``__run_block``) отправляет OSC-бандл с новыми нотами в scsynth —
        не момент, когда они реально зазвучат. Сам бандл несёт таймстемп
        ``osc_message_time() == time.time() + Clock.latency``
        (``TempoClock.py:485-487``; дефолт ``Clock.latency = 0.25``,
        ``TempoClock.py:121``) — это НАСТОЯЩИЙ NTP-таймстемп OSC bundle
        (``ServerManager/__init__.py:get_bundle``: ``OSCBundle(time=
        timestamp)``), а scsynth планирует его исполнение (создание своих
        ``/g_new``+``/s_new`` нод — каждая нота у Renardo живёт в
        собственной подгруппе группы 1, см. ``get_bundle``) НА этот момент
        в будущем, не раньше. До этого момента у scsynth просто нет нод
        нового трека — их физически нечем задеть немедленным
        ``/g_freeAll``, поэтому teardown можно (и нужно) держать старый
        трек живым ещё ``Clock.latency`` секунд ПОСЛЕ ``next_bar()``, а не
        только до него.

        Фикс: не звать ``freeAll`` синхронно, а посчитать здесь, сколько
        секунд реально осталось до старта нового трека
        (``next_bar + Clock.latency``), и запланировать ramp/freeAll на
        этот момент минус запас (см. :meth:`_schedule_transition_cleanup`).
        До дедлайна старый трек доигрывает сам — короткие ноты с
        ``sus``/``doneAction`` успевают освободиться естественно, длинные
        попадают под ramp/freeAll ровно на границе (перед тем, как в
        scsynth материализуются ноды нового трека), а не секундой раньше и
        не 0.25с раньше.

        ``Clock.latency`` читается с живого ``clock`` (``getattr``), не
        хардкодится — конкретное значение внешнее (Renardo/оператор могут
        его менять, ``Clock.set_latency(...)``); дефолт Renardo
        (:data:`RENARDO_DEFAULT_CLOCK_LATENCY_SECONDS`) — только fallback,
        если атрибута нет вовсе.

        Returns:
            Секунды до дедлайна, зажатые снизу нулём (``0.0`` — сигнал
            вызвать teardown немедленно: Clock недоступен, BPM невалиден,
            или диагностика упала) и сверху
            :data:`TRANSITION_CLEANUP_MAX_DELAY_SECONDS`. Никогда не бросает.
        """
        try:
            clock = self._renardo_context.get("Clock")
            if clock is None:
                return 0.0
            now = float(clock.now())
            next_bar = float(clock.next_bar())
            bpm = float(self._renardo_bpm())
            if bpm <= 0:
                return 0.0
            latency = float(
                getattr(clock, "latency", self.RENARDO_DEFAULT_CLOCK_LATENCY_SECONDS)
            )
            if latency < 0.0:
                latency = 0.0
            beats_until = max(0.0, next_bar - now)
            seconds_until_bar = beats_until * 60.0 / bpm
            seconds_until_first_note = seconds_until_bar + latency
            delay = max(
                0.0, seconds_until_first_note - self.TRANSITION_CLEANUP_SAFETY_MARGIN_SECONDS
            )
            return min(delay, self.TRANSITION_CLEANUP_MAX_DELAY_SECONDS)
        except Exception:  # noqa: BLE001 — диагностика не должна ронять переход
            return 0.0

    def _transition_cleanup(self, group: int = 1) -> None:
        """Снести СТАРЫЕ ноды группы и подготовить её для НОВОГО трека.

        Тело исполняется либо сразу (``delay<=0`` в
        :meth:`_schedule_transition_cleanup`), либо в потоке
        ``threading.Timer`` — в обоих случаях после того, как
        :meth:`_ramp_down_group` отработает, обязана остаться пауза
        (:data:`TRANSITION_CLEANUP_G_NEW_PAUSE_SECONDS`) перед ``/g_new``
        (issue #778): UDP fire-and-forget, scsynth не успевает освободить
        ID группы мгновенно, без паузы ``/g_new`` (или первая нота нового
        трека, целящаяся в ту же группу) придёт раньше, чем scsynth
        обработает ``/g_freeAll`` → «FAILURE IN SERVER /g_new negative node
        IDs are reserved».

        Issue #3137 R2 (живой замер 28.09.2026): у ЭТОЙ паузы теперь есть
        второй потребитель — :data:`TRANSITION_CLEANUP_SAFETY_MARGIN_SECONDS`
        считает по ней (вместе с :data:`RAMP_DOWN_RELEASE_SECONDS`), сколько
        всего времени занимает весь конвейер этого метода, чтобы дедлайн в
        :meth:`_transition_cleanup_delay_seconds` гарантированно оставлял
        время на его завершение ДО того, как в scsynth материализуется
        первая нота нового трека (её собственный ``/g_new`` целится в ЭТУ
        группу — см. margin'а docstring).
        """
        self._ramp_down_group(group)
        try:
            time.sleep(self.TRANSITION_CLEANUP_G_NEW_PAUSE_SECONDS)
        except Exception:
            pass
        try:
            self._send_osc_raw("/g_new", group, 0, 0)
        except Exception:
            pass

    def _schedule_transition_cleanup(self, group: int = 1) -> None:
        """Отложить ramp/freeAll старого трека до старта нового (issue #3137).

        Раньше ``execute_code`` звало ``/g_freeAll`` СРАЗУ после ``exec`` —
        старые ноды умирали мгновенно, а новые начинали звучать секундой
        позже (:meth:`_transition_cleanup_delay_seconds`), отсюда живой
        симптом −180 dBFS. Теперь teardown откладывается на вычисленный
        момент — старый трек доигрывает почти до самой границы, новый
        стартует туда же, где старый замолк, дыры не остаётся.

        Дедлайн выбран строго ДО момента, когда в scsynth реально
        МАТЕРИАЛИЗУЕТСЯ первая нота нового трека (с запасом
        :data:`TRANSITION_CLEANUP_SAFETY_MARGIN_SECONDS`) — issue #3137 R2
        (живой замер 28.09.2026): это НЕ ``Clock.next_bar()``, а
        ``Clock.next_bar() + Clock.latency``. ``next_bar()`` — момент,
        когда Renardo-клок ОТПРАВЛЯЕТ в scsynth OSC-бандл новой ноты; сам
        бандл несёт NTP-таймстемп ``time.time() + Clock.latency``
        (``renardo_lib/TempoClock.py:485-487``, дефолт ``latency=0.25s``,
        строка 121) — и scsynth ставит его в свой внутренний планировщик,
        создавая ноды бандла (``ServerManager.get_bundle``) РОВНО на этот
        будущий момент, не раньше. До него у scsynth физически нет ни
        одной ноды нового трека — немедленному ``/g_freeAll`` нечего
        задеть, кроме старых, поэтому дедлайн можно (и нужно) держать на
        ``next_bar + latency``, а не на самом ``next_bar`` (см.
        :meth:`_transition_cleanup_delay_seconds` — там же обоснование
        полного расчёта запаса). Это сознательный выбор между двумя
        вариантами, предложенными ревью: (а) freeAll строго до первой
        новой ноты — можно звать по номеру группы, не отслеживая
        конкретные node ID; (б) free только СТАРЫХ node ID — потребовало
        бы вести реестр ID нод на Python-стороне (scsynth сам назначает ID
        при ``/s_new``, Renardo их не публикует) — отдельная инвазивная
        правка ради временного окна в десятки миллисекунд. (а) даёт тот же
        результат проще и без нового состояния, поэтому выбран он: пока
        наш таймер (и весь его конвейер до завершения ``/g_new``, см.
        margin) укладывается СТРОГО РАНЬШЕ, чем в scsynth материализуется
        нода нового трека, ``/g_freeAll`` не может задеть ничего, кроме
        старых нод, а группа-цель для новой ноды (``target=1`` в её
        собственном ``/g_new``, см. :data:`TRANSITION_CLEANUP_SAFETY_MARGIN_SECONDS`)
        успевает быть пересоздана заранее.

        Никогда не блокирует вызывающий поток — либо выполняет teardown
        сразу (``delay<=0``, включая любой сбой диагностики Clock —
        поведение не хуже старого синхронного пути), либо планирует его
        фоновым ``threading.Timer`` (не поток Renardo-клока и не поток
        MCP-инструмента).

        Args:
            group: группа, которую разово освобождаем и пересоздаём.
        """
        delay = self._transition_cleanup_delay_seconds()
        if delay <= 0.0:
            self._transition_cleanup(group)
            return
        timer = threading.Timer(delay, self._transition_cleanup, args=(group,))
        timer.daemon = True
        timer.start()

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

    @property
    def master_gain(self) -> float:
        """Issue #3125 — текущий уровень мастер-фейдера (0.0-1.0)."""
        return self._master_gain

    # ------------------------------------------------------------------
    # Code safety filter — логика вынесена в core/renardo_sanitizer
    # (единый seam). Обёртки ниже оставлены для обратной совместимости
    # тестов, которые зовут приватные методы напрямую.
    # ------------------------------------------------------------------

    def _filter_code(self, code: str) -> Tuple[bool, str]:
        return renardo_sanitizer._filter_code(code)

    @staticmethod
    def _filter_code_ast(code: str) -> Tuple[bool, str]:
        return renardo_sanitizer._filter_code_ast(code)

    # ------------------------------------------------------------------
    # Issue #1016 — music-quality guardrail (dramaturgy validator)
    # ------------------------------------------------------------------

    def _validate_music_code(self, code: str) -> Tuple[List[str], List[str]]:
        return renardo_sanitizer._validate_music_code(code)

    # ------------------------------------------------------------------
    # Issue #1804 — d4+/p4+ не звучат на роботе, кода-стражи не было
    # ------------------------------------------------------------------

    def _remap_illegal_slots(self, code: str) -> Tuple[str, Optional[str]]:
        return renardo_sanitizer._remap_illegal_slots(code)

    # ------------------------------------------------------------------
    # Issue #1803 — рисунок play(...), который не делит такт, плывёт
    # ------------------------------------------------------------------

    def _fix_pattern_length(self, code: str) -> str:
        return renardo_sanitizer._fix_pattern_length(code)

    def _cap_amp(self, code: str) -> str:
        return renardo_sanitizer._cap_amp(code, self._max_amp)

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
        # Единый seam очистки (core/renardo_sanitizer): безопасность →
        # музыкальный валидатор (+ существование синтов, live 21.09.2026,
        # известное множество приходит из known_synth_names()) →
        # перестановка слотов → pianovel→rhpiano → длина рисунка → кап amp.
        # Порядок и сообщения сохранены байт-в-байт.
        sanitized = renardo_sanitizer.sanitize_renando(
            code,
            self._max_amp,
            known_synths=self.known_synth_names(),
            # Issue #2841: лупы пака 1 — только за флагом окружения.
            pack1_loops_enabled=sample_loops.pack1_loops_enabled(),
        )
        if sanitized.security_error:
            return {"success": False, "error": sanitized.security_error}
        if sanitized.quality_errors:
            return {
                "success": False,
                "error": "⛔ Код отклонён музыкальным валидатором: "
                + " ".join(sanitized.quality_errors),
                "code": sanitized.code,
            }
        if sanitized.slot_error:
            return {"success": False, "error": sanitized.slot_error, "code": sanitized.code}

        code = sanitized.code
        quality_warnings = list(sanitized.warnings)

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

        # Если код содержит Clock.clear() — СНАЧАЛА выполняем код (регистрируем
        # новые паттерны), ПОТОМ ПЛАНИРУЕМ (не зовём синхронно!) ramp/freeAll
        # старых SC-нод на момент, когда реально стартует новый трек.
        #
        # Issue #3137 (корень, найден ревью координатора): раньше freeAll
        # звался СРАЗУ после exec — старые ноды умирали мгновенно, а новые
        # плееры встают на Clock.next_bar() и реально начинают звучать
        # секундой(-ями) позже (см. ALIGN_LEAD_BEATS в engine/renardo_adapter.py) —
        # отсюда окно цифровой тишины (−180 dBFS), а не просто щелчок.
        # _schedule_transition_cleanup вычисляет этот разрыв и откладывает
        # teardown группы почти до самой границы — см. её докстринг и
        # _transition_cleanup_delay_seconds для точной математики и выбора
        # между вариантами фикса.
        #
        # Почему freeAll вообще нужен (не только доиграть и забыть): Clock.
        # clear() останавливает планировщик Renardo, но НЕ посылает freeAll
        # в scsynth сам по себе. После многих переходов 1024-нодовая таблица
        # SC забивается → "too many nodes" / "negative node IDs" → тишина.
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
            # Issue #3137: НЕ убиваем старые SC-ноды синхронно здесь —
            # планируем ramp/freeAll на момент, когда реально стартует новый
            # трек (_schedule_transition_cleanup), чтобы старый трек доигрывал
            # почти до самой границы вместо мгновенного обрыва в цифровую
            # тишину. Внутри — тот же anti-click ramp (gate=0 → пауза →
            # freeAll → пауза #778 → /g_new), что раньше шёл здесь синхронно
            # и что ``stop_all`` использует немедленно (там дыра не важна —
            # явная остановка, а не переход между треками).
            self._schedule_transition_cleanup(1)

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
        self._stamp_new_track()

        # Issue #1016 — quality warnings surfaced to the LLM so it can fix
        # them on the next call (e.g. add dur=, add a developing pattern).
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
        # Issue #1815: "-" — звучащий хэт ("hyphen"), а не пауза; настоящая
        # пауза — "." (разбор символов — renardo_adapter.load_sample_buffers,
        # общий с проверкой ресурсов владельца плеера v2).
        try:
            samples = self._renardo_context.get("Samples")
            if samples is None:
                return
            for match in _PLAY_SYMBOLS_RE.finditer(code):
                renardo_adapter.load_sample_buffers(samples, match.group(1))
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

    def _stamp_new_track(self) -> None:
        """Отметить успешно исполненный код: сессия жива (#935)."""
        with self._state_lock:
            now = time.monotonic()
            self._last_music_activity_at = now
            if self._music_session_active_since is None:
                self._music_session_active_since = now

    def _end_music_session(self, now_m: float) -> None:
        """Сбросить состояние сессии: музыки больше нет. Renardo/SuperCollider не трогает."""
        with self._state_lock:
            self._active_patterns.clear()
            self._last_stop_at = now_m
            # Issue #990 — a stop cancels the segments safety-net deadline:
            # music is no longer playing, so there is nothing to backstop.
            self._music_deadline_at = None
            self._music_deadline_segments = None
            # Reset session only when the *whole* session is over so a partial
            # ``stop_pattern``-then-restart sequence doesn't lose the timer
            # (issue #935 — keeps audit trail of when music was active).
            self._music_session_active_since = None
            self._last_music_activity_at = None

    def stop_all(self) -> Dict[str, Any]:
        """Остановить всю музыку: плавный gate=0 ramp-down → freeAll.

        Issue #1000 (phase-3.2 anti-click):
            Hard ``/g_freeAll`` без ramp-down даёт щелчки на MdaPiano/rhpiano
            (физ-модели — release-фаза ADSR не успевает затухнуть).

        Этапы:
        1. ``.stop()`` на всех живых плеерах (d1-d9, p1-p9, s1-s9, l1-l9) —
           снимает их с планировщика Renardo (внутреннее состояние).
        2. ``Clock.clear()`` — убрать все запланированные события.
        3-4. :meth:`_ramp_down_group` (issue #3137) — ``gate=0`` на ноды
           группы 1 → ~50ms на release ADSR → ``/g_freeAll``. Вызывается
           СИНХРОННО (в отличие от ``execute_code``'а — там тот же teardown
           теперь откладывается до старта нового трека, потому что там
           важна секунда тишины между треками; здесь явная остановка,
           отложенность не нужна и не делается).

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

            # Шаг 3-4: gate=0 ramp-down → freeAll (issue #1000 anti-click,
            # issue #3137 — общий хелпер, тот же путь, что execute_code).
            self._ramp_down_group(1)

        # Явный стоп — не «доиграл сам» (issue #3133): finished_track_id=None.
        self._end_music_session(time.monotonic())
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

    def get_state(self) -> Dict[str, Any]:
        """Вернуть текущее состояние музыкального менеджера.

        Returns:
            dict с полями renardo_available, supercollider_running,
            pattern_history, active_patterns.
        """
        supercollider_running = self._check_supercollider()
        if not self._renardo_available and supercollider_running:
            self._ensure_renardo_available()

        return {
            "renardo_available": self._renardo_available,
            "supercollider_running": supercollider_running,
            "pattern_history": dict(self._pattern_history),
            "active_patterns": list(self._active_patterns),
            "renardo_last_error": getattr(self, "_renardo_last_error", None),
            # Music-stack health snapshot — surfaced so the LLM can see
            # ``music_stack_healthy: false`` and avoid retrying calls that
            # will be rejected by ``execute_music_code``.
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

        The AgentCore / watchdog should call this periodically (e.g. once
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
        deadline = self._music_deadline_at
        if deadline is not None and now_m >= deadline:
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

        Convenience hook for AgentCore / dialogue_node to call on
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
    def llm_visible(self) -> bool:
        """ADR-0149 §8.2 (PR-13b): LLM код не пишет; тул — только для харнессов (``/mcp/execute``, подпись ``harness``).

        Им пользуются ``scripts/music/live_check_mcp_call.py`` и ``live_check_3115.sh`` (живая проверка кода v2
        без пересборки образа, ``scripts/music/live_dj/README.md``).
        """
        return False

    @property
    def name(self) -> str:
        return "execute_music_code"

    @property
    def description(self) -> str:
        return (
            "Выполнить готовый Renardo-код (харнесс живой проверки, LLM не виден). "
            "Код выполняется в контексте Renardo "
            "(FoxDot-совместимый синтаксис). "
            "Пример: 'p1 >> pluck([0, 2, 4], dur=0.5, amp=0.8)'. "
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

    @property
    def starts_music(self) -> bool:
        return True

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
            return MCPToolResult(success=True, data=result, message=result["message"])
        return MCPToolResult(success=False, error=result["error"])


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
                # Issue #3108: общие с mcp_server publisher'ы на этой ноде.
                self._sound_stop_pub = shared_publisher(
                    node, String, "/voice/sound/stop", 10
                )
                self._generated_music_state_pub = shared_publisher(
                    node, String, "/voice/generated_music/state", 10
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
            # Issue #1392 follow-up: останавливаем и mp3-трек в sound_node.
            self._notify_sound_stop()
            return MCPToolResult(success=True, data=result, message=result["message"])
        return MCPToolResult(success=False, error=result["error"])

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
            "историю кода паттернов."
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
            f"Активные паттерны: {', '.join(state['active_patterns']) or 'нет'}",
            f"История паттернов: {', '.join(state['pattern_history'].keys()) or 'нет'}",
        ]
        return MCPToolResult(
            success=True,
            data=state,
            message="\n".join(parts),
        )


class SetMusicVolumeTool(MCPTool):
    """Issue #3125 — громкость МУЗЫКИ (мастер-фейдер scsynth), не голоса.

    Живой сет 28.09.2026: «играй громче» во время DJ-сета — у LLM был только
    ``set_volume``, а он крутит ``/tts_node volume_db``, то есть ГОЛОС. Уровень
    музыки задавал лишь ROS-параметр ``music_master_gain`` при старте, до LLM
    он не доходил, и модель честно выполнить просьбу не могла — фантазировала
    «подкручиваю трек на максимум» при неизменном уровне.

    Тул двигает тот же фейдер, что и параметр: ``MusicManager.set_master_gain``
    → ``/n_set 999 gain <v>`` (синт ``masterlimiter``, сглаживание ``Lag.kr``,
    без щелчка). Шаг ``louder``/``quieter`` — ±3 dB (×√2 по амплитуде), как у
    ``set_volume`` для голоса. ``normal`` — значение ``music_master_gain``, с
    которым стартовал сервер. Уровень клэмпится в [0, 1]: выше 1.0 фейдер
    не поднимается (``set_master_gain``).

    Issue #3154: фейдер стоит ПОСЛЕ радио-динамики ``masterfilter`` (лимитер
    держит пик −1 dBFS до фейдера), поэтому шаг ±3 dB — ровно ±3 dB на
    выходе, а не упор в лимитер; на ``max`` (1.0) пик выхода −1 dBFS.
    """

    #: ±3 dB по амплитуде.
    STEP_FACTOR: float = 10 ** (3.0 / 20.0)
    #: Нижний предел ШАГОВОГО «тише»: шаги не должны молча заглушить музыку
    #: в ноль (для тишины есть ``stop_music``); явный ``level=0`` разрешён.
    MIN_STEP_GAIN: float = 0.05
    MAX_GAIN: float = 1.0

    def __init__(self, node, manager: MusicManager) -> None:
        super().__init__(node)
        self._manager = manager
        #: Уровень «как было при старте» — значение ROS-параметра
        #: ``music_master_gain``, которым сконструирован менеджер.
        self._normal_gain: float = float(manager.master_gain)

    @property
    def name(self) -> str:
        return "set_music_volume"

    @property
    def description(self) -> str:
        return (
            "Громкость МУЗЫКИ (трек/DJ-сет: request_music, dj_set), "
            "а НЕ голоса робота. Юзер просит "
            "«громче/тише/погромче/потише» и сейчас играет музыка, или прямо "
            "говорит «музыку/трек/бит громче» — вызывай ЭТОТ тул, а не "
            "set_volume (set_volume меняет только голос). Музыку не "
            "перезапускает: играющий трек продолжает играть, меняется только "
            "уровень. action: louder/quieter — шаг ±3 dB, max — максимум, "
            "normal — стартовый уровень, set — абсолютный уровень level "
            "0..100 (% от максимума). mp3 из MiniMax-библиотеки "
            "(gen_play_from_library) этим тулом не регулируется."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="action",
                type="string",
                description=(
                    "louder — громче на шаг, quieter — тише на шаг, max — на "
                    "максимум, normal — стартовый уровень, set — выставить level"
                ),
                required=True,
                enum=["louder", "quieter", "max", "normal", "set"],
            ),
            MCPToolParameter(
                name="level",
                type="integer",
                description=(
                    "Только для action=set: уровень музыки в процентах от "
                    "максимума, 0..100 (значения вне диапазона обрезаются)."
                ),
                required=False,
            ),
        ]

    @property
    def slice(self) -> str:
        return "personality"

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def _target_gain(self, action: str, level: Optional[float], current: float):
        """Целевой уровень для ``action`` или ``(None, error)``."""
        if action == "louder":
            return min(self.MAX_GAIN, current * self.STEP_FACTOR), None
        if action == "quieter":
            return max(self.MIN_STEP_GAIN, current / self.STEP_FACTOR), None
        if action == "max":
            return self.MAX_GAIN, None
        if action == "normal":
            return self._normal_gain, None
        if action == "set":
            if level is None:
                return None, "action=set требует level 0..100"
            try:
                pct = float(level)
            except (TypeError, ValueError):
                return None, f"level должен быть числом 0..100, получено {level!r}"
            return max(0.0, min(100.0, pct)) / 100.0 * self.MAX_GAIN, None
        return None, f"Неизвестное действие: {action}"

    def execute(self, action: str, level: Optional[float] = None) -> MCPToolResult:
        """Изменить уровень мастер-фейдера музыки."""
        current = float(self._manager.master_gain)
        target, error = self._target_gain(action, level, current)
        if error is not None:
            return MCPToolResult(success=False, error=error)
        applied = self._manager.set_master_gain(target)
        self.log_info(
            f"[set_music_volume] action={action} level={level} "
            f"master_gain {current:.2f} → {applied:.2f}"
        )
        pct = round(applied / self.MAX_GAIN * 100)
        if abs(applied - current) < 1e-3:
            edge = "максимальная" if applied >= self.MAX_GAIN else "уже такая"
            message = f"Громкость музыки не изменилась ({edge}, {pct}%)"
        else:
            message = f"Громкость музыки: {round(current / self.MAX_GAIN * 100)}% → {pct}%"
        return MCPToolResult(
            success=True,
            data={"old_gain": round(current, 3), "new_gain": round(applied, 3), "percent": pct},
            message=message,
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

    НЕ на ``/data/harness_voice.db`` (issue #2000 / ADR-0055): её таблицы
    (``music_tracks``, ``generated_tracks``) сами по себе не конфликтуют со
    схемой ``SQLiteVoiceMemory``, но ``WaypointStore``/``FAQStore`` — да
    (см. их докстринги), а все три стора делят один и тот же
    ``VOICE_MEMORY_DB_PATH``. Переносить музыку в одиночку значило бы
    расщепить единый файл ещё сильнее, а не объединить. См.
    ``docs/adr/0055-voice-memory-db-unify-with-harness.md`` (anti-goal §5.6).

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

    def find_melody(self, query: str) -> Optional[Dict[str, Any]]:
        """Найти известную мелодию (``type='melody'``) по имени/заголовку/тегу.

        Ищет регистро-независимо по ``name`` (slug), ``title`` и ``tags``
        (алиасы). Точное совпадение slug (через транслитерацию :meth:`_slug`)
        имеет приоритет над подстрочным совпадением.

        Args:
            query: Что юзер назвал («кузнечик», «имперский марш»).

        Returns:
            Запись мелодии с ``code``, либо ``None`` если не нашлось или
            колонка ``type`` ещё не добавлена (миграция 006 не применена).
        """
        q = (query or "").strip().lower()
        if not q:
            return None
        slug = self._slug(query)
        with self._lock:
            try:
                rows = self._conn.execute(
                    "SELECT * FROM music_tracks WHERE type = 'melody'"
                ).fetchall()
            except sqlite3.OperationalError:
                return None
        entries = [self._row_to_dict(r, include_code=True) for r in rows]
        # Точное совпадение slug — приоритет.
        for entry in entries:
            if slug and slug == entry.get("name"):
                return entry
        # Подстрочное совпадение по name/title/tags.
        for entry in entries:
            names = [
                entry.get("name") or "",
                entry.get("title") or "",
                *(entry.get("tags") or []),
            ]
            hay = " ".join(str(n).lower() for n in names)
            if q in hay:
                return entry
        return None

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


class LookupMelodyTool(MCPTool):
    """Найти известную мелодию по имени и вернуть её ТОЧНЫЕ ноты.

    Ищет в RTTTL-библиотеке (архив ``data/rtttl_melodies.jsonl.gz``,
    10461 готовых мелодий) и возвращает СЫРУЮ RTTTL-строку в
    ``data['rtttl']`` — БЕЗ воспроизведения и БЕЗ конвертации. Ноты
    играет ``request_music`` (движок v2, ADR-0149 §5.1), а не модель.
    Фолбэк — курируемые мелодии ``music_tracks`` (012). Не допускает
    ошибку #1810 — сыграть гамму и назвать её «кузнечиком».
    """

    def __init__(
        self,
        node,
        library: TrackLibrary,
        manager: MusicManager,
        rtttl_library: Optional[RtttlLibrary] = None,
    ) -> None:
        super().__init__(node)
        self._library = library
        self._manager = manager
        self._rtttl_library = rtttl_library

    @property
    def name(self) -> str:
        return "lookup_melody"

    @property
    def description(self) -> str:
        return (
            "Найти известную мелодию по имени и вернуть её ТОЧНЫЕ ноты сырой "
            "RTTTL-строкой в data['rtttl'], НИЧЕГО не играя: проверить, есть ли "
            "такая мелодия и как она точно называется. Играет мелодию "
            "request_music(intent=melody, text=слова человека) — ноты и "
            "аранжировку решает код. Имя ищи на "
            "АНГЛИЙСКОМ или транслитом («имперский марш» → \"imperial march\"). "
            "Если не нашлось — честно скажи, что не знаешь точных нот."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="name",
                type="string",
                description="Название мелодии (английским или транслитом): "
                "«имперский марш» → \"imperial march\", «кузнечик» → "
                "\"grasshopper\", «happy birthday», «jingle bells»…",
                required=True,
            ),
            MCPToolParameter(
                name="variants",
                type="array",
                description=(
                    "Дополнительные варианты названия (английским/транслитом), "
                    "которые пробовать по порядку, если name не найдётся. "
                    "Например name=\"imperial march\", variants=[\"darth vader\", "
                    "\"star wars theme\"]."
                ),
                required=False,
                items=MCPToolParameter(
                    name="variant",
                    type="string",
                    description="Альтернативное написание/название мелодии.",
                ),
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
        name: str,
        variants: Optional[List[str]] = None,
    ) -> MCPToolResult:
        """Найти ноты и вернуть сырую RTTTL-строку (без воспроизведения)."""
        # 1. RTTTL-библиотека — приоритет.
        if self._rtttl_library is not None:
            # issue #2964 (правка товарища Шифу — НЕ жёсткий гейт, см.
            # модульный докстринг у _resolve_melody_with_candidate): как и
            # раньше (issue #2896), берётся лучший ПО ТЕКСТУ кандидат —
            # ``get()`` его всё равно всегда находит. Честность — в
            # прозрачности результата: реальное название (``display_title``
            # — с исполнителем, если title сам по себе неинформативен по
            # корпусу, напр. «Theme») и явная сверка значимых слов запроса
            # (``match`` — :func:`match_info`, IDF по корпусу, без
            # хардкод-списка стоп-слов). Решение «это та же песня» —
            # у модели, по общему правилу в промпте скилла composer.
            candidate, rec = _resolve_melody_with_candidate(
                self._rtttl_library, name, variants
            )
            if rec is not None:
                title = rec.get("title")
                shown_title = display_title(self._rtttl_library, rec)
                alternatives = _search_alternatives(self._rtttl_library, candidate, title)
                match = match_info(self._rtttl_library, rec, candidate or name)
                return MCPToolResult(
                    success=True,
                    data={
                        "name": rec.get("name"),
                        "title": title,
                        "display_title": shown_title,
                        "rtttl": rec.get("rtttl"),
                        "alternatives": alternatives,
                        "match": match,
                    },
                    message=(
                        f"Нашёл «{shown_title}». Точные ноты в data['rtttl']. "
                        + _mismatch_note(match, candidate or name)
                    ),
                )
        # 2. Фолбэк — курируемые мелодии в SQLite (type='melody', миграция 012).
        entry = self._library.find_melody(name)
        if entry is None:
            return MCPToolResult(
                success=False,
                error=(
                    f"Мелодия {name!r} не найдена в библиотеке. Скажи юзеру "
                    "честно, что не знаешь точных нот, и предложи сыграть "
                    "что-то в похожем духе — НЕ выдавай импровизацию за оригинал."
                ),
            )
        return MCPToolResult(
            success=True,
            data={
                "name": entry.get("name"),
                "title": entry.get("title"),
                "code": entry.get("code"),
            },
            message=f"Нашёл «{entry.get('title')}» (готовый Renardo-код в data['code']).",
        )


class SearchMelodyTool(MCPTool):
    """Поиск по RTTTL-библиотеке (10460 готовых мелодий) по имени/жанру/тегу.

    Возвращает кандидатов (метаданные, без нот). Ноты конкретной мелодии
    берутся через lookup_melody. Нужен, когда юзер хочет не одну мелодию, а
    выбор: «найди новогодние», «что есть из игр?».
    """

    def __init__(self, node, library: RtttlLibrary) -> None:
        super().__init__(node)
        self._library = library

    @property
    def name(self) -> str:
        return "search_melody"

    @property
    def description(self) -> str:
        return (
            "Найти мелодии в RTTTL-библиотеке по названию/жанру/тегу "
            "(английским или транслитом: «новогодние» → \"christmas\", "
            "«игры» → \"game\"). Возвращает до limit кандидатов с "
            "названием, артистом и тегами. Поиск идёт по названию, "
            "исполнителю, тегам и имени внутри формата мелодии. "
            "Чтобы СЫГРАТЬ конкретную — вызови lookup_melody(name=...)."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="query",
                type="string",
                description="Строка поиска: имя, артист, жанр или тег.",
                required=True,
            ),
            MCPToolParameter(
                name="limit",
                type="integer",
                description="Сколько кандидатов вернуть (по умолчанию 20).",
                required=False,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        return False

    def execute(self, query: str, limit: int = 20) -> MCPToolResult:
        try:
            limit = max(1, min(50, int(limit or 20)))
        except (TypeError, ValueError):
            limit = 20
        hits = self._library.search(query, limit=limit)
        if not hits:
            return MCPToolResult(
                success=False,
                error=(
                    f"По запросу {query!r} ничего не найдено в RTTTL-библиотеке. "
                    "Скажи честно и предложи поискать по-другому."
                ),
            )
        return MCPToolResult(
            success=True,
            data={"melodies": hits, "total": len(hits)},
            message=f"Найдено {len(hits)} мелодий по запросу {query!r}.",
        )
