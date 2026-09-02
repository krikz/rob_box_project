#!/usr/bin/env python3
"""dialogue_node.py — Voice dialogue ROS2 shell (Phase 6 v2 / W5).

Thin ROS2 shell that composes DialogCore over the harness ports
(LLMProvider, ToolProvider, MemoryStore, DSM). Owns only ROS2 pub/sub,
the asyncio loop driver, DJ-mode hook, barge-in/cancel, TTS/sound
awaiter release, and lifecycle. Wake-word / silence classification
lives in core.dialogue_text; DJ-mode state machine in core.dj_mode;
TTS chunking + SSML framing + awaiter registry in core.speak_helpers.

Replaces the 2181-line predecessor by extracting the agent loop, every
function_tool body (OpenAI Agents SDK decorators), the provider registry,
history trimming, and all inline tool implementations into the harness
ports delivered in W1-W4.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import logging
import math
import os
import re
import threading
import time
import traceback
import uuid
from typing import Any, List, Optional

import yaml

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry

from rob_box_core.tool_catalog import CORE_SKILL, skill_names, tools_for_skill
from rob_box_harness.config import LLMConfig
from rob_box_harness.core.dialog_core import DialogCore, DialogResult
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateKind,
    DialogueStateMachine,
)
from rob_box_harness.core.tool_registry import ToolRegistry
from rob_box_harness.executors import ROSMCPToolProvider, adapt_tool_provider
from rob_box_harness.health import (
    DEFAULT_HEALTH_TTL_S,
    HealthAwareFallbackLLM,
    HealthCache,
    check_deepseek_balance,
)
from rob_box_harness.memory import (
    Fact,
    InMemoryStore,
    MemoryStore,
    SQLiteVoiceMemory,
    get_speaker_profile,
    speaker_scope,
    touch_speaker,
)
from rob_box_harness.providers import (
    DEFAULT_BASE_URL as MINIMAX_DEFAULT_BASE_URL,
    DEFAULT_MODEL as MINIMAX_DEFAULT_MODEL,
    DEEPSEEK_DEFAULT_BASE_URL,
    DEEPSEEK_DEFAULT_MODEL,
    LLM_PROVIDER_REGISTRY,
    build_deepseek_provider,
    build_minimax_provider,
)
from rob_box_harness.tools import FakeToolProvider, ToolProvider
from rob_box_llm.errors import ProviderError
from rob_box_llm.provider import LLMMessage, LLMSettings

from rob_box_voice.core.command_parser import CommandParser, IntentType
from rob_box_voice.core.skill_router import SkillRouter
from rob_box_voice.core.dialogue_text import (
    DEFAULT_WAKE_WORDS, has_wake_word, is_silence_command, is_unsilence_command, strip_wake_word,
)
from rob_box_voice.core.llm_skip_reasons import (
    LLMSkipReason,
    new_llm_skip_counter,
)
from rob_box_voice.scheduler.quick_decide import QuickVerdict, quick_decide
from rob_box_voice.core.dialogue_guards import (
    BABBLE_BANNED_OPENERS as BABBLE_BANNED_OPENERS,
    BABBLE_PERFORMANCE_KEYWORDS as BABBLE_PERFORMANCE_KEYWORDS,
    GENERATED_MUSIC_TOOLS,
    MUSIC_GUARD_KEYWORDS,
    MUSIC_GUARD_VOCAL_KEYWORDS,
    MUSIC_MODE_TOOLS,
    MUSIC_STOP_TOOLS,
    RENARDO_MUSIC_TOOLS,
    MUSIC_RETRY_PROMPT_PREFIX,
    MUSIC_STOP_OVERRIDES,
    build_babble_retry_prompt,
    build_music_retry_prompt,
    build_renardo_code_retry_prompt,
    build_unbacked_action_retry_prompt,
    build_tool_retry_prompt as build_tool_retry_prompt,
    detect_required_tool as detect_required_tool,
    detect_unbacked_action_claim,
    extract_renardo_code_lines,
    is_metalanguage_babble,
    is_planning_narration,
    is_music_stop_command,
    user_wants_music,
    user_wants_performance,
)
from rob_box_voice.core.dialogue_helpers import (
    EMOTION_TO_ANIMATION as EMOTION_TO_ANIMATION,
    detect_pitch_intent,
    detect_volume_intent,
    format_llm_skipped_summary,
    generate_fallback_response,
    map_emotion_to_animation,
    sanitize_speaker_name,
)
from rob_box_voice.core.music_guard import (
    MusicGuard,
    MusicGuardVerdictKind,
)
from rob_box_voice.core.speech_accumulator import SpeechAccumulator
from rob_box_voice.core.dj_mode import DJHook, DJModeController
from rob_box_voice.core.speak_helpers import (
    EffectAwaiterRegistry, build_ssml_payload, split_into_chunks,
    strip_done_marker, strip_history_marker, strip_markdown,
    strip_speaker_tag, strip_thinking_blocks,
)
from rob_box_voice.startup_greeting import (
    THINKING_SOUND,
    pick_finish_sound,
    pick_greeting,
)
from rob_box_voice.speaker_profiles import (
    SpeakerTracker,
    extract_speaker_name,
    format_speaker_context,
)
from rob_box_voice.tts_voice_registry import format_tts_context
# Issue #1787 — сборка промпта и валидация клички, придуманной LLM.
from rob_box_voice.core import epithets

# Issue #1160 — Prometheus metrics (этап 1 observability).
# ``prometheus_client`` — optional dep; если её нет, всё превращается в
# no-op и старт сервера тихо возвращает ``False``.
from rob_box_voice.observability import (
    init_tracing,
    is_metrics_enabled,
    record_barge_in,
    record_fallback,
    record_pending_queue_latency,
    record_quick_decide_verdict,
    record_session_duration,
    record_task_updated,
    record_llm_prompt_tokens,
    record_skill_activation,
    record_voice_llm_request,
    start_metrics_server,
    start_span,
)


def _xml_attr(value: str) -> str:
    """Escape ``value`` for safe inclusion in an XML attribute (issue #1544).

    Replaces the four characters that would break the ``<music_state …/>``
    attribute syntax: ``"``, ``&``, ``<``, ``>``. Used by
    :meth:`DialogueNode._build_music_state_snapshot` to safely embed user-
    controlled theme names (``DJ-тема может содержать кавычки и амперсанды
    из free-form ввода юзера)``) into the LLM-bound ``<system_context>``.

    Cheap enough to call per-turn (~4 regex subs). Not a full XML escape —
    we only render to LLM, not to a browser.
    """
    # ``str.replace`` chain is fine here — values are short (<200 chars).
    return (
        value
        .replace("&", "&amp;")
        .replace('"', "&quot;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )


ASYNCIO_LOOP_DRIVER_MAX_WORKERS: int = 1
ASYNCIO_LOOP_DRIVER_NAME_PREFIX: str = "dialogue-async-loop"
ASYNCIO_LOOP_DRIVER_SHUTDOWN_TIMEOUT_S: float = 2.0

# S7 (scheduler-segments-merge, issue #968) — upper bound on
# ``_pending_user_messages`` so a run of barge-ins during one very long
# LLM turn cannot grow the queue unbounded. Appending past this cap
# drops the OLDEST queued phrase (keep the most recent user intent) and
# logs a warning — see _on_stt.
_PENDING_USER_MESSAGES_MAX: int = 5

# Issue #1389 compatibility alias. ``LLMSkipReason`` is now the canonical
# source; this tuple remains for callers that imported the merged #1395 symbol.
_LLM_SKIP_REASONS: tuple[str, ...] = tuple(reason.value for reason in LLMSkipReason)

# Issue #992 (live 13.08): BACKING-детектор «2+ speak_text» ложно
# срабатывал на разговорчивую LLM (приветствие + комментарий) и убивал
# только что запущенный TRACK сразу после tts_batch_complete
# («наполни комнату музыкой» → музыка умолкала через ~8с).
# Теперь BACKING требует ПЕВЧЕСКИЙ интент в тексте юзера.
_SINGING_INTENT_RE = re.compile(
    r"\b(?:спо[йюё]\w*|по[йюё]\w*|рэп\w*|реп\w*|песенк\w*|куплет\w*|частушк\w*|напев\w*)\b",
    re.IGNORECASE,
)


def _has_singing_intent(text: "str | None") -> bool:
    """True если юзер явно просил петь/рэповать (BACKING), а не просто музыку."""
    return bool(text) and bool(_SINGING_INTENT_RE.search(text or ""))

# Issue #992 Bug D — banned metalanguage openers + performance keywords
# live in :mod:`rob_box_voice.core.dialogue_guards` (TD-1 decomposition);
# they are imported at the top of this module so
# ``dialogue_node.BABBLE_BANNED_OPENERS`` / ``BABBLE_PERFORMANCE_KEYWORDS``
# keep working for external importers and tests.


class _FallbackLLM:
    """LLM-обёртка с fallback-цепочкой (live 06.08).

    Primary (MiniMax) → при ЛЮБОЙ ошибке (429 Token Plan limit, таймаут,
    5xx) переключается на fallback (DeepSeek), чтобы робот не немел, пока
    primary недоступен. ``LLMConfig.fallback`` декларативно существует,
    но ``_build_llm`` его не использовал → 429 оставлял робота немым.

    Оба метода (``complete`` / ``stream``) пробуют primary, при исключении
    логируют и отдают fallback.

    .. deprecated::
        Заменён на :class:`rob_box_harness.health.HealthAwareFallbackLLM`
        (issue #1082) — реактивная цепочка «пробуем → падаем →
        переключаемся» тратила 15-19с на retry мёртвого MiniMax.
        Класс оставлен для обратной совместимости, ``_build_llm`` его
        больше не использует.
    """

    def __init__(self, primary: Any, fallback: Any, logger: Any) -> None:
        self._primary = primary
        self._fallback = fallback
        self._log = logger
        self._pname = getattr(primary, "name", type(primary).__name__)
        self._fname = getattr(fallback, "name", type(fallback).__name__)

    async def complete(self, messages, tools=None):
        try:
            return await self._primary.complete(messages, tools=tools)
        except Exception as exc:  # noqa: BLE001
            self._log.warning(
                f"🔄 LLM {self._pname} упал: {exc} — fallback {self._fname}"
            )
            return await self._fallback.complete(messages, tools=tools)

    async def stream(self, messages, tools=None):
        try:
            async for chunk in self._primary.stream(messages, tools=tools):
                yield chunk
            return
        except Exception as exc:  # noqa: BLE001
            self._log.warning(
                f"🔄 LLM {self._pname} stream упал: {exc} — fallback {self._fname}"
            )
        async for chunk in self._fallback.stream(messages, tools=tools):
            yield chunk


# 🔴 FIX (13.08, надзор): monkey-patch `_rclpy_logger_safe` УДАЛЁН.
#
# История: 09.08 патч добавлялся против
# ``RcutilsLogger.warning() takes 2 positional arguments but 5 were given``
# (TypeError от %s-вызова в python-logging стиле). Обёртка склеивала args
# в message и звала оригинал.
#
# Проблема патча: обёртка добавляет кадр `wrapper` в стек вызова, и rclpy
# `_find_caller()` (rcutils_logger.py) останавливается на первом кадре ВНЕ
# rclpy — это wrapper. В результате CallerId одинаков для ВСЕХ вызовов
# логгера через патч, а rclpy кэширует контекст (severity/name/filters) по
# CallerId и падает, если тот же CallerId логирует с другим severity:
#   ValueError: Logger severity cannot be changed between calls.
# → крах dialogue_node (DJ tick: warning на 176 + info на 192) → Empty
#   assistant response → робот молчит.
#
# Почему безопасно удалить: реальных %s-вызовов RcutilsLogger в коде НЕТ
# (проверено 13.08: все rclpy-вызовы — f-строки; найденные %s — это
# python logging `logging.getLogger`, он поддерживает %s сам). Патч был
# мёртвым хаком, который только ломал CallerId-механизм rclpy.


class DialogueNode(Node):
    """ROS2 shell that composes DialogCore over the harness ports."""
    def __init__(self) -> None:  # noqa: D401 — ROS2 ctor signature
        super().__init__("dialogue_node")
        # Issue #1234 — OpenTelemetry traces (этап 2). ВАЖНО: вызываем
        # ДО _build_llm(), потому что LLM-провайдеры (openai SDK / httpx)
        # создают свои httpx-клиенты при конструировании — авто-инструментация
        # httpx должна быть включена раньше, чтобы их вызовы попали в трейс.
        # Если opentelemetry-пакетов нет — no-op (см. observability.tracing).
        init_tracing("dialogue_node")
        self._declare_params()
        # Issue #1601 / ADR-0027 §3.4 — supervisor (ADR-0028) переключает
        # ``voice_input_mode`` без рестарта ноды; callback логирует изменение.
        self.add_on_set_parameters_callback(self.parameters_callback)
        # Issue #1409 — SSoT for MCP tool names. Populated from
        # ``ToolRegistry.list_tools()`` at startup (the canonical 32+5
        # manifests the LLM is wired to via ``_build_tool_provider``) and
        # kept in sync via ``_on_mcp_tools_update`` when /mcp/tools refresh
        # messages arrive. ``_load_system_prompt`` uses this set to verify
        # that every tool the LLM can call is mentioned in the
        # ``music_skill_prompt.txt`` (case-insensitive) — silent drift
        # between tool surface and prompt text otherwise makes the LLM
        # confidently say «нет такой функции» (see issue #1403).
        self._mcp_tool_names: set[str] = self._collect_mcp_tool_names()
        self._system_prompt: str = self._load_system_prompt()
        self._skill_prompts: dict[str, str] = self._load_skill_prompts()
        self._validate_skill_fragments(self._skill_prompts)
        #: Детерминированный пред-роутер домена. None — скиллы выключены.
        self._skill_router: Any = None
        #: Последние прочитанные счётчики load_skill из DialogCore — нужны,
        #: чтобы публиковать ПРИРОСТ, а не абсолютное значение.
        self._skill_load_seen: tuple[int, int] = (0, 0)
        self._verbose_llm: bool = bool(self.get_parameter("verbose_llm").value)
        self._wake_words: List[str] = list(self.get_parameter("wake_words").value)
        # Issue #1279 — gate команд движения/статуса: фразы, которые уже
        # распознаны command_node (NAVIGATE/STOP/STATUS/MAP), НЕ должны
        # дублироваться через LLM (LLM интерпретирует «вперёд» как музыку).
        # Используем ТОТ ЖЕ CommandParser, что и command_node, чтобы
        # классификация совпадала 1:1 (один источник правды — core).
        self._command_intent_gate_enabled: bool = bool(
            self.get_parameter("command_intent_gate_enabled").value
        )
        self._command_intent_gate_confidence: float = float(
            self.get_parameter("command_intent_gate_confidence").value
        )
        self._command_parser = CommandParser(
            wake_words=["робот", "робокс", "робобокс"],
            confidence_base=0.8,
        )
        # Пред-роутер домена переиспользует ТОТ ЖЕ CommandParser, что и
        # command_intent_gate — второй классификатор не заводим.
        if self._skill_prompts:
            self._skill_router = SkillRouter(
                self._command_parser,
                known_skills=tuple(sorted(self._skill_prompts)),
                confidence=self._command_intent_gate_confidence,
            )
            self.get_logger().info(
                f"🧭 Пред-роутер скиллов включён: "
                f"{', '.join(sorted(self._skill_prompts))}"
            )
        # Issue #XXXX — «новая сессия» / «сбрось всё» / Telegram «/clear»:
        # сброс всего контекста текущего диалога. Фразы читаем из YAML,
        # дефолт — _DEFAULT_NEW_SESSION_PHRASES.
        self._new_session_enabled: bool = bool(
            self.get_parameter("new_session_enabled").value
        )
        raw_phrases = self.get_parameter("new_session_phrases").value
        self._new_session_phrases: tuple[str, ...] = tuple(
            str(p).strip().lower()
            for p in (raw_phrases or self._DEFAULT_NEW_SESSION_PHRASES)
            if str(p).strip()
        )
        self._barge_in_policy: str = self._resolve_barge_in_policy()

        self._loop = asyncio.new_event_loop()
        self._asyncio_loop_executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=ASYNCIO_LOOP_DRIVER_MAX_WORKERS,
            thread_name_prefix=ASYNCIO_LOOP_DRIVER_NAME_PREFIX,
        )
        self._asyncio_loop_future: Optional[concurrent.futures.Future] = (
            self._asyncio_loop_executor.submit(self._loop.run_forever)
        )
        self._run_task: Optional[asyncio.Task] = None
        self._task_lock = threading.Lock()
        self._run_cancelled: bool = False
        # S7 (scheduler-segments-merge, issue #968) — phrases that arrive
        # while a turn's LLM cycle is still in flight (barge_in_policy=
        # "classify", quick_decide=PENDING_LLM) are queued here instead of
        # starting a second concurrent turn. Drained as ONE follow-up turn
        # from _run_turn's ``finally`` once the turn slot frees up again —
        # see _on_stt / _drain_pending_user_messages. Each entry is
        # (text, enqueued_at) so the drain can log queue latency.
        self._pending_user_messages: "deque[tuple[str, float]]" = deque()
        self._vad_speech_detected: bool = False
        self._effects = EffectAwaiterRegistry(
            release_tts=lambda ev: self._loop.call_soon_threadsafe(ev.set),
            release_sound=lambda ev: self._loop.call_soon_threadsafe(ev.set),
        )

        self._memory: MemoryStore = self._build_memory()
        # Issue #1077 — speaker profiles: подтверждённый speaker_tag →
        # профиль (scope=speaker:<tag>). SpeakerTracker подтверждает tag
        # после 2+ фраз подряд (защита от нестабильных tags Yandex).
        self._speaker_by_text: Dict[str, dict] = {}
        self._speaker_tracker = SpeakerTracker()
        # Issue #1077 — голосовая биометрия (resemblyzer d-vectors, TASK-048):
        # speaker_id_node публикует /voice/speaker/result (JSON: is_known,
        # speaker_id, name, confidence). LLM-контекст получает префикс
        # [Говорит <имя>] / [Говорит: незнакомец] — робот различает
        # собеседников по голосу, а не только по Yandex tag (который
        # присваивается per-session и не стабилен между сессиями).
        self._speaker_id_enabled: bool = bool(
            self.get_parameter("speaker_id_enabled").value)
        self._current_speaker: dict = {"is_known": False}
        self._speaker_lock = threading.Lock()
        # Бэклог-аккумулятор фоновой речи без wake-слова (docs/plans/
        # 2026-08-20-voice-backlog-accumulator-design.md).
        self._speech_accumulator = SpeechAccumulator(
            window_sec=float(self.get_parameter("accumulate_window_sec").value),
        )
        self._accumulate_no_wake_enabled = bool(self.get_parameter("accumulate_no_wake_enabled").value)
        self._pending_backlog_flush = False
        # Issue #1195 — последний chat_id из Telegram (source-маркер
        # [TG:chat_id] в /voice/stt/result). Используется для
        # маршрутизации ответа: dialogue_node кладёт tg_chat_id в
        # payload /voice/dialogue/response, telegram_node читает его и
        # шлёт send_message в нужный чат. None = голосовой ввод.
        self._active_tg_chat_id: Optional[int] = None
        self._dsm: DialogueStateMachine = DialogueStateMachine(
            silence_timeout=float(self.get_parameter("dialogue_timeout").value),
        )
        # Issue #1160 — LLM держим и в атрибуте ноды: метрики
        # (``record_voice_llm_request``) и future OTel spans берут имя
        # провайдера из ``self._llm.name``, а не из ``self._core._llm``
        # (private-атрибут DialogCore). Раньше ``_build_llm()`` вызывался
        # inline и нода теряла ссылку — обращение ``self._llm`` падало
        # AttributeError в ``_run_turn``.
        self._llm = self._build_llm()
        # W7c (issue #968): /harness/task_events publisher — scheduler
        # lifecycle events (task.created/started/completed/...) for
        # monitoring. Created BEFORE _build_tool_provider so the W7b
        # scheduler's on_event callback can publish immediately.
        self._task_events_pub = self.create_publisher(
            String, "/harness/task_events", 10)
        # W7b: the SchedulerToolExecutor wrapping the tool provider
        # (created inside _build_tool_provider). Kept as an attribute so
        # _build_dynamic_system_context can render the [ACTIVE TASKS]
        # block for the LLM. None when scheduler is disabled/failed.
        self._scheduler_executor: Any = None
        self._core: DialogCore = DialogCore(
            llm=self._llm,
            tools=self._build_tool_provider(),
            memory=self._memory,
            dsm=self._dsm,
            history_trim_limit=int(self.get_parameter("history_max_turns").value),
            inactivity_timeout=float(self.get_parameter("dialogue_timeout").value),
            system_prompt=self._system_prompt,
            use_streaming=bool(self.get_parameter("llm_streaming").value),
            on_prompt=self._on_prompt_stats,
            skill_prompts=self._skill_prompts,
        )

        cbg = ReentrantCallbackGroup()
        qos_r = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST, depth=10)
        self._response_pub = self.create_publisher(
            String, "/voice/dialogue/response", 10)
        self._state_pub = self.create_publisher(String, "/voice/dialogue/state", 10)
        self._sound_trigger_pub = self.create_publisher(
            String, "/voice/sound/trigger", 10)
        # Issue #1101 — diagnostics for "why LLM wasn't called".
        # Оператор видит «робот молчит», а в логе — ни одного error/warn.
        # Реальная причина обычно одна из: no_wake_word, silenced,
        # silence_command, empty_after_strip, stt_rejected, music_stop,
        # command_intent.
        # Счётчик + периодическая сводка раз в 5 минут — сразу видно, что
        # фразы теряются на gate'е ещё до LLM.
        # Issue #1389 — strict dict initialized from the enum SSoT. Unknown
        # literal keys still raise KeyError and are rejected by the CI checker.
        self._llm_skipped_counter: dict[str, int] = new_llm_skip_counter()
        self._last_skip_summary_ts: float = time.monotonic()
        self._tts_control_pub = self.create_publisher(
            String, "/voice/tts/control", 10)
        # Issue #1734 — единственный источник истины для barge_in_policy:
        # latched (TRANSIENT_LOCAL) топик вместо ВТОРОГО параметра в
        # stt_node.yaml. Дублирование параметра — ровно тот класс ошибки,
        # который уже случился с wake_words (issue #1252, два YAML,
        # разъехались) и который и породил issue #1734 (stt_node не знал
        # про classify). TRANSIENT_LOCAL закрывает и «порядок старта нод»
        # (поздний subscriber всё равно получает последний семпл), и
        # «потерю отдельного сообщения» (durability держит семпл, пока
        # жив этот publisher — а не полагается на «долетело/не долетело»
        # одного datagram'а).
        self._barge_in_policy_pub = self.create_publisher(
            String,
            "/voice/dialogue/barge_in_policy",
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1,
            ),
        )
        self._publish_barge_in_policy()
        # Music safety-net hook (issue #935): when the dialog ends and the
        # LLM forgot to call stop_music(), we still want playback to stop.
        # We publish a JSON payload on /mcp/music_cleanup so the MCP server
        # runs ``MusicManager.stop_music_on_session_end()`` for us.
        try:
            self._music_cleanup_pub = self.create_publisher(
                String, "/mcp/music_cleanup", 10)
            self.get_logger().info(
                "🎵 [dialogue_node] Publisher на /mcp/music_cleanup готов (issue #935)"
            )
        except Exception as exc:  # noqa: BLE001
            self._music_cleanup_pub = None
            self.get_logger().warning(
                f"⚠️ [dialogue_node] Не удалось создать /mcp/music_cleanup publisher: {exc}"
            )
        # Issue #1016 — empty-response music fallback. When the LLM returns
        # an empty reply to a music request, dialogue_node asks mcp_server
        # to play the top-rated human track from the library instead of
        # leaving the user in silence.
        try:
            self._music_fallback_pub = self.create_publisher(
                String, "/mcp/music_fallback", 10)
            self.get_logger().info(
                "🎵 [dialogue_node] Publisher на /mcp/music_fallback готов (issue #1016)"
            )
        except Exception as exc:  # noqa: BLE001
            self._music_fallback_pub = None
            self.get_logger().warning(
                f"⚠️ [dialogue_node] Не удалось создать /mcp/music_fallback publisher: {exc}"
            )
        self.create_subscription(
            String, "/voice/stt/result", self._on_stt, qos_r, callback_group=cbg)
        # ADR-0027 §3.4 — Quest robot-voice: stt_node публикует распознанную
        # фразу с микрофона Quest в отдельный топик /voice/stt/quest (чтобы
        # не ломать plain-text контракт /voice/stt/result). Маршрутизация —
        # по voice_input_mode (см. _on_quest_stt).
        self.create_subscription(
            String, "/voice/stt/quest", self._on_quest_stt, qos_r, callback_group=cbg)
        # Issue #1279 — command_node публикует feedback («Двигаюсь вперёд»,
        # «Останавливаюсь») на /voice/command/feedback после выполнения
        # команды движения/статуса. dialogue_node озвучивает его через TTS,
        # чтобы пользователь СЛЫШАЛ подтверждение команды (раньше feedback
        # публиковался, но никем не озвучивался — dialogue_node вместо него
        # гнал фразу в LLM и получал музыку вместо движения).
        self.create_subscription(
            String, "/voice/command/feedback", self._on_command_feedback,
            qos_r, callback_group=cbg)
        # Issue #1077 — speaker_tag от Yandex speaker_analysis (отдельный
        # топик, чтобы не ломать plain-text контракт /voice/stt/result).
        # JSON: {"speaker_tag", "text", "duration_s"}. stt_node публикует
        # speaker ПЕРЕД result, поэтому _on_speaker обычно приходит раньше
        # _on_stt; храним по тексту и забираем в _on_stt.
        self.create_subscription(
            String, "/voice/stt/speaker", self._on_speaker, qos_r,
            callback_group=cbg)
        # Issue #1077 — голосовая биометрия: результат speaker_id_node
        # (resemblyzer d-vector, JSON: is_known/speaker_id/name/confidence).
        if self._speaker_id_enabled:
            self.create_subscription(
                String, "/voice/speaker/result", self._on_speaker_result, qos_r,
                callback_group=cbg)
            self._speaker_register_pub = self.create_publisher(
                String, "/voice/speaker/register", 10)
            # Issue #1787 — реплики опознанного спикера уходят в
            # speaker_id_node, который считает по ним темы и выбирает
            # внутреннюю кличку (эпитет). Текст есть только здесь, БД
            # спикеров — только там.
            self._speaker_observe_pub = self.create_publisher(
                String, "/voice/speaker/observe", 10)
            # Issue #1787, слой 2 гибрида — speaker_id_node просит
            # придумать кличку (LLM живёт только здесь), ответ уходит
            # обратно на /voice/speaker/epithet.
            self.create_subscription(
                String, "/voice/speaker/epithet_request",
                self._on_epithet_request, qos_r, callback_group=cbg)
            self._speaker_epithet_pub = self.create_publisher(
                String, "/voice/speaker/epithet", 10)
        self.create_subscription(
            Bool, "/audio/vad", self._on_vad, 10, callback_group=cbg)
        self.create_subscription(
            String, "/voice/tts/finished", self._on_tts_finished, 10,
            callback_group=cbg)
        # Issue #1219 — LLM voice selection: mcp_server (SetVoiceTool)
        # публикует смену голоса JSON {"voice": str, "provider": str};
        # dialogue_node хранит current_voice для контекста [TTS].
        self.create_subscription(
            String, "/voice/tts/current_voice", self._on_tts_current_voice, 10,
            callback_group=cbg)
        # Issue #1229 — фактический провайдер TTS (после фолбека) от tts_node.
        # JSON {"provider": str, "voice": str, ...}. Используется в контексте
        # [TTS], чтобы LLM видела голоса РЕАЛЬНОГО провайдера (а не
        # номинального minimax), и не выбирала голоса, которых нет у
        # фактического провайдера.
        self.create_subscription(
            String, "/voice/tts/provider_state", self._on_tts_provider_state, 10,
            callback_group=cbg)
        # Issue #1392 follow-up — состояние сгенерированной музыки
        # (gen_play_from_library / stop_music / sound_node публикуют JSON).
        self.create_subscription(
            String, "/voice/generated_music/state", self._on_generated_music_state, 10,
            callback_group=cbg)
        # 🔴 FIX (live 31.08): «после нескольких генераций робот начинает
        # тупить и говорит, что растерялся». Музыку останавливает watchdog в
        # mcp_server (reason=idle_ttl, 300 с без диалога), а диалог об этом
        # не узнавал — комментарий в track-mode честно писал «живёт до
        # stop_music/watchdog», но канала для второго не было. Из лога:
        #     1788186658  [watchdog] Авто-стоп 1 паттернов: reason=idle_ttl
        #     1788186797  [track-mode] TRACK играет с прошлого хода
        #     1788186797  [Bug C] LLM skipped ...; publishing spoken nudge
        # Через 139 с после реальной остановки флаг всё ещё говорил «играет»,
        # ретрай-промпт требовал ИЗМЕНИТЬ несуществующий трек, модель
        # отвечала словами — и робот произносил «я растерялся».
        # Теперь флаг следует за сервером, а не за догадкой.
        self.create_subscription(
            String, "/voice/music/state", self._on_music_state, 10,
            callback_group=cbg)
        # Issue #980 — fire music_cleanup only after the *last* TTS chunk of a
        # batch (rap, poetry), not after the first. tts_node publishes this
        # event once ``batch_index == batch_total`` for a given ``batch_id``.
        self.create_subscription(
            String, "/voice/tts/batch_complete",
            self._on_tts_batch_complete, 10,
            callback_group=cbg)
        # Issue #992 — SpeakTextTool publishes this prelude BEFORE the first
        # TTS request of each ``speak_text`` call so we can pre-register the
        # ``batch_id``. Without it we only learn about a batch from the
        # first ``tts_finished`` for that batch — which works for a single
        # ``speak_text`` call but fails when the LLM fires two in a row:
        # batch #1's ``batch_complete`` would fire cleanup while batch #2 is
        # still pending. Payload: ``{"batch_id": str, "chunks_total": int}``.
        self.create_subscription(
            String, "/voice/tts/batch_registered",
            self._on_tts_batch_registered, 10,
            callback_group=cbg)
        self.create_subscription(
            String, "/voice/sound/state", self._on_sound_state, 10,
            callback_group=cbg)
        # Робот-позиция из /odom — лёгкий снимок {x, y, theta} для LLM-контекста
        # <system_context> <position>. Без tf2_ros.Buffer (подписка на /tf ~110 Гц
        # жгла ~45% CPU через wait-set rebuild на rmw_zenoh, mcp-server-cpu-loop).
        self._pose_snapshot = None
        try:
            self.create_subscription(Odometry, "/odom", self._on_odom_snapshot, 10, callback_group=cbg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"⚠️ [dialogue_node] /odom подписка не удалась: {exc}")
        self.create_subscription(
            String, "/voice/dj_mode",
            lambda m: self._dj.handle_message(m.data), 10, callback_group=cbg)
        # Каталог инструментов от mcp_server. Подписка latched
        # (TRANSIENT_LOCAL) — mcp_server публикует каталог один раз при
        # старте, и порядок запуска нод перестаёт иметь значение.
        #
        # ``_on_mcp_tools_update`` существовал с issue #1409, но подписки к
        # нему не было НИ ОДНОЙ: колбэк никогда не вызывался, а
        # ``mcp_tools_available`` навсегда оставался False. Из-за этого в
        # ``_on_vad`` ветка «не рвать agent-loop, пока идут тул-вызовы» была
        # недостижима — barge-in рвал цикл всегда.
        self.create_subscription(
            String, "/mcp/tools", self._on_mcp_tools_update,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=cbg)

        # Deferred music cleanup (issue #935 v2 → #980 → #992): music should
        # keep playing while TTS is still speaking (rap, poetry). Cleanup is
        # published strictly after the *last* TTS batch of a turn finishes
        # — see ``_on_tts_batch_complete``.
        #
        # Issue #992: a single ``_pending_music_cleanup`` boolean is not
        # enough when the LLM fires multiple ``speak_text`` tool calls
        # in one cycle (each call is a separate ``batch_id`` and produces
        # its own ``/voice/tts/batch_complete``). The first batch's
        # completion used to wipe music while the second batch was still
        # playing. ``_active_batches`` tracks every in-flight ``batch_id``
        # so we only fire cleanup when the registry is empty AND the
        # LLM actually asked to stop music.
        self._pending_music_cleanup: bool = False
        self._active_batches: Dict[str, int] = {}

        # 🔴 FIX (live 30.08 15:56): TRACK-музыка не должна умирать от хода,
        # который её не трогал.
        #
        # Лаунж играл 94 секунды, юзер сказал «продолжай лабать мы летим над
        # парижем», LLM ответила словами с tools=[] — и ветка «музыка в этом
        # цикле не запускалась» вооружила cleanup, который остановил трек
        # через 0.1 с после ответа. Робот сказал «Трек летит над Парижем» и
        # замолчал.
        #
        # Контракт TRACK описан парой десятков строк ниже: «композиция живёт
        # до segments или явного stop_music». Ветка ниже его нарушала для
        # ЛЮБОГО следующего хода — включая «который час?» посреди трека.
        # Флаг помнит, что живая музыка — это TRACK, и cleanup для неё не
        # вооружается. Потолок остаётся за watchdog'ом (idle TTL 300 s и
        # segments-дедлайн), явным stop_music и cleanup'ом нового диалога.
        self._track_mode_music_active: bool = False

        # 🔴 FIX (live 30.08, e2e 33251879328): один флаг «в этом ходе гуард
        # уже отправил ретрай» на ВСЕ гуарды сразу.
        #
        # ``_run_turn`` откладывает ``DIALOGUE_END``, когда ретрай в пути:
        # без этого родительский ход роняет DSM в IDLE, и ``process_input``
        # ретрая коротит на закрытом диалоге — возвращает пустоту за 1 мс,
        # без единого HTTP-запроса (issue #1204). Раньше условие
        # перечисляло гуарды поимённо, и оба новых (Bug C′ и Bug E) в него
        # не попали: шаги tc12_delete_track и tc16_delete_waypoint легли с
        # ``llm_error``, юзер услышал «Принял.».
        #
        # Теперь флаг ставит :meth:`_mark_retry_dispatched`, а вызвать её
        # обязан каждый ``_check_*_and_retry`` — это проверяет
        # ``test_dialogue_retry_flag_wiring.py``.
        self._retry_dispatched_in_turn: bool = False
        # Issue #992 Bug B / Bug C — retry budgets and policy now live
        # in :class:`MusicGuard` (TD-2 decomposition, ARCH-review #1405 /
        # ADR-0021). ``_run_turn`` resets the user-budget via
        # ``reset_for_new_user_request``; ``_dispatch_dj_turn`` resets the
        # DJ budget via ``reset_for_new_dj_transition``. No more
        # duplicated counters across the two scopes.
        self._music_guard: MusicGuard = MusicGuard(
            logger=self.get_logger(),
        )

        # Issue #992 Bug D — metalanguage / babble detector.
        # ``True`` after a single metalanguage retry has already been
        # dispatched for the current user turn. We only allow ONE
        # babble retry to avoid an infinite LLM ping-pong; if the LLM
        # babbles again after the retry, we fall through to publish the
        # meta-text verbatim and let the operator debug from logs.
        self._babble_retry_used: bool = False

        # Issue #992 Bug E — «отчитался о действии, но не вызвал тул».
        # Тот же одноразовый контракт, что у babble-флага выше: ретраим
        # РОВНО один раз, иначе LLM и код уходят в пинг-понг.
        self._action_claim_retry_used: bool = False

        # Issue #992 Bug C' — Renardo-код, попавший в реплику вместо
        # execute_music_code. Тот же одноразовый контракт.
        self._code_speech_retry_used: bool = False

        # Issue #1777 / #1762 — non-music tool-skipped retry budget.
        # ``True`` после того как Bug C retry для явного tool-based
        # запроса (get_current_time / search_web / set_voice /
        # memory_search / faq_search) уже был отправлен в текущем turn.
        # Защита от бесконечного LLM ping-pong: один ретрай на turn.
        # Сбрасывается на новый user-initiated turn (см. _run_turn).
        self._tool_retry_used: bool = False

        # Issue #1160 — Prometheus metrics: длительность диалоговой
        # сессии. Фиксируем момент первого wake-word-диалога из IDLE;
        # при DIALOGUE_END / timeout пишем histogram.
        self._session_started_at: Optional[float] = None
        self._session_end_reason: str = "success"

        # Issue #992 Bug A — DJ auto-transitions MUST NOT take the
        # ``new_dialogue`` cleanup path. Wrapping ``_dispatch_turn`` here
        # sets ``is_dj_auto=True`` so the dispatcher skips
        # ``_publish_music_cleanup(reason="new_dialogue")`` and the
        # barge-in cancel that would otherwise wipe an in-flight
        # track and feed the LLM an empty input.
        self._dj = DJModeController(
            hook=DJHook(
                dispatch=self._dispatch_dj_turn,
                is_active=lambda: (self._run_task is not None
                                   and not self._run_task.done()),
                is_dialogue_active=lambda: self._dsm.current_state in (
                    DialogueStateKind.DIALOGUE, DialogueStateKind.SILENCED),
                on_stop=self._on_dj_stop_farewell,
            ),
            logger=self.get_logger(),
        )
        self.create_timer(5.0, self._on_inactivity_check)
        self.create_timer(DJModeController.DJ_TICK_INTERVAL_S, self._dj.tick)
        # 🔴 FIX (live 06.08): startup-приветствие внутри dialogue_node
        # (замена отдельной startup_greeting_node, #1003). Одноразовый
        # таймер: через startup_greeting_sec секунд после старта говорим
        # фразу. tts_node к этому моменту уже прогрет (он стартует раньше
        # и грузится ~3-5с), поэтому гонки нет. 0 = выключено.
        # Фраза — случайная из GREETINGS (startup_greeting.py), если
        # startup_greeting_text пустой (или не задан); иначе — явный текст.
        self._startup_greeting_sec = float(
            self.get_parameter("startup_greeting_sec").value or 0.0
        )
        self._startup_greeting_text = str(
            self.get_parameter("startup_greeting_text").value or ""
        )
        if self._startup_greeting_sec > 0:
            self.get_logger().info(
                f"🗣 Startup greeting через {self._startup_greeting_sec:.0f}s"
            )
            self.create_timer(
                self._startup_greeting_sec, self._on_startup_greeting
            )
        # Issue #1160 — Prometheus metrics server (этап 1).
        # Порт 9100 — стандартный для voice-assistant (см.
        # ``observability/__init__.py``); в проде используется
        # ``10.1.1.11:9100/metrics`` для Grafana scrape.
        # ``start_metrics_server`` идемпотентен: если уже бежит — no-op.
        # Если ``prometheus_client`` не установлен — молча False в лог.
        self._metrics_port: int = int(
            self.get_parameter("metrics_port").value or 0
        )
        if self._metrics_port > 0 and is_metrics_enabled():
            started = start_metrics_server(self._metrics_port)
            if started:
                self.get_logger().info(
                    f"📊 Metrics server listening on :{self._metrics_port}/metrics"
                )
            else:
                self.get_logger().warning(
                    f"📊 Metrics port {self._metrics_port} not bound "
                    "(busy or prometheus_client missing)"
                )
        self.get_logger().info("✅ DialogueNode shell ready (DialogCore wired)")
    def _declare_params(self) -> None:
        # 🔴 FIX (live 18:00): MiniMax Token Plan кончился (429 rate_limit
        # 'Token Plan usage limit reached'). YAML мёртв (#1004) — дефолт
        # в коде единственный живой путь → переключаем на deepseek.
        # DEEPSEEK_API_KEY в env voice-assistant.
        # 🔴 FIX (live 10.08, issue #1089): приоритетная цепочка LLM-провайдеров.
        # Формат: "minimax,deepseek" — порядок = приоритет.
        # Первый в списке = primary, остальные = fallbacks.
        # Каждый провайдер настраивается в своей YAML-секции (<name>.base_url и т.д.).
        self.declare_parameter("llm_providers", "deepseek")
        # Per-provider параметры — каждая YAML-секция провайдера
        # (minimax:, deepseek:, mimo:, qwen:) отдаёт свои настройки
        # через dotted-имена.  Пустая строка → используются
        # well-known defaults из _LLM_PROVIDER_REGISTRY.
        for pname in ("minimax", "deepseek", "mimo", "qwen"):
            self.declare_parameter(f"{pname}.api_key", "")
            self.declare_parameter(f"{pname}.base_url", "")
            self.declare_parameter(f"{pname}.model", "")
            self.declare_parameter(f"{pname}.temperature", 0.0)
            self.declare_parameter(f"{pname}.max_tokens", 0)
            self.declare_parameter(f"{pname}.timeout_s", 0.0)
        self.declare_parameter("api_key", "")
        self.declare_parameter("base_url", "")
        self.declare_parameter("model", "")
        self.declare_parameter("temperature", 0.7)
        self.declare_parameter("max_tokens", 500)
        self.declare_parameter("system_prompt_file", "master_prompt_compact.txt")
        # Move A (change skill-scoped-dialogue-context, фаза 2): доменные
        # фрагменты инструкций, приезжающие вплотную к текущему ходу.
        # ВЫКЛЮЧЕНО по умолчанию — включается решением Шифу по метрикам
        # voice_llm_prompt_tokens, см. Migration Plan change'а.
        self.declare_parameter("skills_enabled", False)
        self.declare_parameter("history_max_turns", 20)
        self.declare_parameter("agent_max_turns", 20)
        self.declare_parameter("dialogue_timeout", 300.0)
        # Scheduler segments/MERGE plan (S1) — "replace" = today's behaviour
        # (barge-in stops TTS unconditionally); "classify" = quick_decide
        # routes the verdict (S4). Garbage value → warn + fall back to
        # "replace" in _resolve_barge_in_policy().
        self.declare_parameter("barge_in_policy", "replace")
        # Один список на весь проект — rob_box_voice.core.dialogue_text.
        # Он же фолбек strip_wake_word, и его порядок неслучаен (длинные
        # варианты первыми, иначе «роб» съедает «роб бокс»). Копий было
        # семь и они разошлись на три разных списка: здесь и в
        # stt_node.py лежало 13 вариантов, в четырёх YAML — 21, в
        # e2e-конфиге — те же 13. Тот самый класс ошибки, из-за которого
        # завели #1252 и заплатили #1734.
        self.declare_parameter("wake_words", list(DEFAULT_WAKE_WORDS))
        self.declare_parameter("enable_mcp_tools", True)
        self.declare_parameter("llm_timeout_sec", 90.0)
        self.declare_parameter("verbose_llm", True)
        # Issue #1279 — gate команд движения/статуса. command_node уже
        # обрабатывает NAVIGATE/STOP/STATUS/MAP (публикует
        # /voice/command/intent и выполняет), dialogue_node НЕ должен
        # дублировать обработку через LLM: иначе LLM интерпретирует
        # «вперёд» как музыку → execute_music_code вместо движения.
        # True = фразы-команды НЕ идут в LLM (обрабатывает command_node).
        self.declare_parameter("command_intent_gate_enabled", True)
        # Порог уверенности CommandParser, при котором фраза считается
        # «уже обработанной командой» (совпадает с command_node.yaml
        # confidence_threshold: 0.7).
        self.declare_parameter("command_intent_gate_confidence", 0.7)
        # Issue #XXXX — «новая сессия» / «сбрось всё» / Telegram «/clear»:
        # детерминированный сброс текущего диалога без вызова LLM.
        self.declare_parameter("new_session_enabled", True)
        self.declare_parameter(
            "new_session_phrases",
            list(self._DEFAULT_NEW_SESSION_PHRASES),
        )
        # 🔴 FIX (live 06.08): стриминг LLM через конфиг (llm_streaming).
        # Замер без стриминга: false → complete() (полный ответ).
        self.declare_parameter("llm_streaming", False)
        # Раньше по умолчанию стоял ``handle_navigation`` — фасад, удалённый
        # вместе с Compositor-скиллами, поэтому фильтр не отсекал ничего.
        self.declare_parameter("history_excluded_tools", ["move_direction"])
        self.declare_parameter("sqlite_db_path", "~/.rob_box/voice.db")
        self.declare_parameter("speaker_id_enabled", True)
        self.declare_parameter("speaker_db_path", "/data/speakers.db")
        # issue #1077: сколько фраз подряд с одним speaker_tag нужно для
        # подтверждения профиля. 2 = защита от нестабильных tags Yandex;
        # 1 = мгновенное подтверждение (если tag стабилен).
        self.declare_parameter("speaker_min_phrases", 2)
        # Бэклог-аккумулятор фоновой речи без wake-слова (docs/plans/
        # 2026-08-20-voice-backlog-accumulator-design.md).
        self.declare_parameter("accumulate_no_wake_enabled", True)
        self.declare_parameter("accumulate_window_sec", 180.0)
        # 🔴 FIX (issue #1082): health-кэш LLM-провайдеров. Файл переживает
        # рестарт робота: если MiniMax мёртв (2056 Token Plan), первый же
        # запрос после ребута идёт на deepseek, а не тратит время на
        # мёртвого. Пусто = кэш только в памяти.
        self.declare_parameter("health_cache_path", "~/.rob_box/llm_health.json")
        self.declare_parameter("health_ttl_s", 300.0)
        # W5a: select the tool-provider backend. ``ros_mcp`` is the
        # production path (LLMToolCallAdapter → ROSMCPToolProvider);
        # ``fake`` swaps in FakeToolProvider for unit tests; ``none``
        # disables tools entirely. Tests override this via launch /
        # YAML so the W5a assertion (provider.list_tools() > 0) can
        # be exercised deterministically without spinning up rclpy
        # publishers.
        self.declare_parameter("tool_provider", "ros_mcp")
        # 🔴 FIX (live 06.08): startup-приветствие БЕЗ отдельной ноды (#1003).
        # Отдельная startup_greeting_node страдала гонкой (tts_node ещё не
        # инициализирован → сообщение терялось). Теперь сам dialogue_node
        # через N секунд после старта говорит фразу напрямую через
        # _publish_response (тот же путь, что и обычная речь робота).
        # 0 = отключено, >0 = задержка в секундах.
        # Пустой startup_greeting_text → случайная фраза из GREETINGS.
        self.declare_parameter("startup_greeting_sec", 12.0)
        self.declare_parameter("startup_greeting_text", "")
        # FAQ/event-mode (test_faq_event_mode.py): _load_event_profile() читает
        # эти параметры, но они НЕ объявлялись — YAML-значения молча
        # игнорировались (класс бага issue #1004). Объявляем, чтобы
        # faq_mode_enabled / faq_event_config_file реально работали.
        self.declare_parameter("faq_mode_enabled", False)
        self.declare_parameter("faq_event_config_file", "")
        self._startup_greeting_fired = False
        # Issue #1219 — LLM voice selection: активный TTS-провайдер для
        # контекста [TTS]. Должен совпадать с tts_node.yaml provider
        # (minimax). Рядом храним current_voice (установленный set_voice),
        # который приходит от mcp_server через /voice/tts/current_voice.
        self.declare_parameter("tts_provider", "minimax")
        self._current_tts_voice: str | None = None
        # Issue #1229 — фактический провайдер TTS (после фолбека) и голос,
        # который РЕАЛЬНО использовал tts_node. Приходят из /voice/tts/
        # provider_state; используются в LLM-контексте [TTS], чтобы LLM
        # видела голоса фактического провайдера, а не номинального.
        self._actual_tts_provider: str | None = None
        self._actual_tts_voice: str | None = None
        # Issue #1392 follow-up — состояние сгенерированной музыки
        # (JSON {"status": "playing"|"idle", "track_id", "title",
        # "duration_ms"}) из /voice/generated_music/state. Показывается LLM
        # в <system_context>, чтобы она сама решала когда вызвать stop_music.
        self._generated_music_state: Optional[dict] = None
        # Issue #1160 — Prometheus metrics endpoint. 9100 = voice (LLM
        # latency / fallback). 0 = отключить старт сервера (полезно для
        # юнит-тестов и CI, где рконфликтует с другими тестами).
        self.declare_parameter("metrics_port", 9100)
        # Issue #1601 / ADR-0027 §3.4 — режим захвата голоса. Используется
        # supervisor'ом (ADR-0028 S5, единственная точка смены) для
        # переключения источника входа (respeaker | quest_passthrough |
        # quest_ttts | quest_stt | quest_llm_formalize | off — W3-1).
        # ``_voice_input_mode`` — кэш последнего значения в поле ноды,
        # который обновляет ``parameters_callback`` и читают
        # ``_on_stt``/``_on_quest_stt``; до прихода первого SetParameters
        # от супервизора дефолт совпадает с YAML/declare_parameter —
        # "respeaker" (обратная совместимость).
        self.declare_parameter("voice_input_mode", "respeaker")
        self._voice_input_mode: str = "respeaker"

    def parameters_callback(self, params):
        """Роутер runtime-изменений параметров (``ros2 param set``).

        ``voice_input_mode`` (Issue #1601 / ADR-0027 §3.4, W3-1):
        сохраняет новое значение в ``self._voice_input_mode`` — это поле
        читают ``_on_stt`` (гейт ReSpeaker-входа при ``off``) и
        ``_on_quest_stt`` (маршрутизация Quest robot-voice). Супервизор
        (ADR-0028 S5) — единственный, кто вызывает SetParameters сюда.

        ⚠️ ``voice_input_mode="off"`` глушит ТОЛЬКО обычных людей у
        ReSpeaker-микрофона. Вход ОПЕРАТОРА (Telegram, Quest robot-voice)
        этим режимом не блокируется — см. docstring ``_on_stt`` и §3.5
        docs/design/dialogue-mode-spec-2026-08-28.md. Не переворачивай
        это правило при доработке.

        ``barge_in_policy`` (issue #1734): обновляет ``self._barge_in_policy``
        и тут же перепубликует его на latched-топик
        ``/voice/dialogue/barge_in_policy`` (``_publish_barge_in_policy``),
        чтобы stt_node узнал новое значение немедленно, без рестарта —
        именно так этот параметр меняли на роботе при воспроизведении
        бага #1734 (``ros2 param set /dialogue_node barge_in_policy
        classify``). Невалидное значение игнорируем и остаёмся на
        текущем — та же логика, что в ``_resolve_barge_in_policy``.
        """
        for param in params:
            if param.name == "voice_input_mode":
                self._voice_input_mode = param.value
                self.get_logger().info(
                    f"🎙 voice_input_mode changed to {param.value!r}"
                )
            elif param.name == "barge_in_policy":
                raw = str(param.value or "replace").strip().lower()
                if raw not in self._BARGE_IN_POLICIES:
                    self.get_logger().warning(
                        f"⚠️ [issue 1734] barge_in_policy={raw!r} — unknown, "
                        f"ignoring runtime change (valid: {self._BARGE_IN_POLICIES})"
                    )
                    continue
                self._barge_in_policy = raw
                self._publish_barge_in_policy()
                self.get_logger().info(
                    f"🔄 [issue 1734] barge_in_policy changed to {raw!r} "
                    f"(republished to stt_node)"
                )
        return SetParametersResult(successful=True)

    def _load_system_prompt(self) -> str:
        prompt_file = self.get_parameter("system_prompt_file").value
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory("rob_box_voice")
            with open(os.path.join(pkg, "prompts", prompt_file),
                      "r", encoding="utf-8") as fh:
                prompt = fh.read()
            self.get_logger().info(
                f"✅ Prompt loaded: {prompt_file} ({len(prompt)} bytes) "
                f"from {os.path.join(pkg, 'prompts', prompt_file)}\n"
                f"   first line: {prompt.split(chr(10))[0][:120]!r}")
            # Regression guard for issue #1219 (voice-change feature): the
            # prompt MUST contain an explicit ``RULE #VOICE`` block
            # instructing the LLM to call ``set_voice(...)`` for voice
            # change requests. Without it, MiniMax-M3 (live provider)
            # silently ignores the request and answers "Ок." without any
            # tool call — the robot stays on the default male voice.
            # If you're refactoring the prompt, keep the rule; if you
            # switch to a different compact prompt, copy the rule over.
            if "RULE #VOICE" not in prompt:
                self.get_logger().warning(
                    "⚠️ [issue 1219] System prompt does not contain "
                    "'RULE #VOICE' — LLM may ignore voice-change "
                    "requests and skip the set_voice tool. See "
                    "master_prompt_compact.txt for the canonical block."
                )
            # Issue #1409 — SSoT tools-vs-prompt validation (music-domain only).
            # ``music_skill_prompt.txt`` is a static contract the LLM reads
            # verbatim at startup, so any MCP tool the LLM can call MUST be
            # mentioned by name — otherwise the LLM degrades to «нет такой
            # функции» fallback (see issue #1403: ``generate_music`` was
            # registered but never mentioned, so the LLM kept using Renardo).
            # Other domain prompts (FAQ, navigation, web_search) stay
            # unchecked for now — TODO when their contracts harden.
            return prompt
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"⚠️ Prompt not found ({exc})")
            return "Ты ROBBOX — умный робот-ассистент. Отвечай кратко и по делу."

    def _collect_mcp_tool_names(self) -> set[str]:
        """Return the canonical set of MCP tool names (SSoT).

        Source of truth is ``ToolRegistry.list_tools()`` — the same
        manifests the LLM is wired to via ``_build_tool_provider``. We
        don't fall back to ``self.available_tools`` here because the
        latter is populated asynchronously by ``/mcp/tools`` messages
        and may be stale/empty at ``_load_system_prompt`` time.
        """
        try:
            return {spec.name for spec in ToolRegistry().list_tools()}
        except Exception as exc:  # noqa: BLE001 — defensive: bad import / init
            self.get_logger().warning(
                f"⚠️ [issue 1409] ToolRegistry probe failed: {exc!r}; "
                "skipping tools-vs-prompt validation"
            )
            return set()

    def _validate_skill_fragments(self, fragments: dict[str, str]) -> None:
        """Предупредить, если инструмент скилла не назван в его тексте.

        Блокер живёт в тесте (``test_skill_prompt_contract.py``) — здесь
        рантайм-страховка на случай, когда на робот приехал промпт из
        другой сборки: контейнер поднимется и заговорит, но оператор
        увидит в логе, какой именно скилл разъехался.

        Раньше эта проверка (``_validate_tools_in_prompt``) смотрела
        ТОЛЬКО музыкальный промпт и только предупреждением — а класс
        расхождения общий: #1403 (``generate_music`` зарегистрирован, в
        тексте не упомянут → LLM отвечает «нет такой функции»).
        """
        if not fragments:
            return
        for skill, text in sorted(fragments.items()):
            lowered = text.lower()
            try:
                tools = tools_for_skill(
                    skill, include_core=(skill == CORE_SKILL)
                )
            except KeyError:
                self.get_logger().warning(
                    f"⚠️ [skills] фрагмент {skill!r} не соответствует ни "
                    "одному скиллу каталога — он не будет активирован"
                )
                continue
            missing = sorted(
                entry.name for entry in tools
                if entry.name.lower() not in lowered
            )
            if missing:
                self.get_logger().warning(
                    f"⚠️ [skills] скилл {skill!r}: {len(missing)} "
                    f"инструмент(ов) не описаны во фрагменте: "
                    f"{', '.join(missing)}. LLM может ответить «нет такой "
                    f"функции» (класс регрессии #1403)."
                )
            else:
                self.get_logger().debug(
                    f"[skills] {skill}: все {len(tools)} инструментов описаны ✓"
                )

    def _load_skill_prompts(self) -> dict[str, str]:
        """Прочитать фрагменты доменных скиллов с диска.

        Файлы читает нода, а не harness: harness обязан оставаться без
        файловой системы и без ROS2 (он получает готовый словарь).

        При ``skills_enabled=false`` возвращаем пустой словарь — тогда
        DialogCore ведёт себя ровно как до скиллов, побайтово.

        Имя файла = имя скилла из каталога. Отсутствие файла для
        объявленного скилла — не ошибка старта: скилл останется без
        текста, а его инструменты никуда не денутся. Ронять ноду из-за
        промпта нельзя — робот должен подняться и говорить.
        """
        try:
            if not bool(self.get_parameter("skills_enabled").value):
                self.get_logger().info(
                    "ℹ️ skills_enabled=false — доменные скиллы выключены "
                    "(Move A не активен, поведение как до change'а)"
                )
                return {}
        except Exception:  # noqa: BLE001 — параметр не объявлен (старый yaml)
            return {}

        try:
            from ament_index_python.packages import get_package_share_directory

            skills_dir = os.path.join(
                get_package_share_directory("rob_box_voice"), "prompts", "skills"
            )
            loaded: dict[str, str] = {}
            absent: list[str] = []
            for skill in skill_names():
                path = os.path.join(skills_dir, f"{skill}.txt")
                try:
                    with open(path, "r", encoding="utf-8") as fh:
                        text = fh.read().strip()
                except OSError:
                    absent.append(skill)
                    continue
                if text:
                    loaded[skill] = text
                else:
                    absent.append(skill)
            self.get_logger().info(
                f"🧩 Загружено фрагментов скиллов: {len(loaded)} "
                f"({', '.join(sorted(loaded)) or '—'})"
            )
            if absent:
                self.get_logger().warning(
                    f"⚠️ Без текста остались скиллы: {', '.join(sorted(absent))} "
                    "— их инструменты работают, но доменных инструкций у LLM нет"
                )
            return loaded
        except Exception as exc:  # noqa: BLE001 — промпт не роняет ноду
            self.get_logger().warning(
                f"⚠️ Не удалось загрузить фрагменты скиллов ({exc}); "
                "Move A остаётся выключенным"
            )
            return {}

    def _activate_skill_for(self, text: str) -> None:
        """Активировать домен ДО обращения к LLM.

        Промах роутера безвреден: при выключенном сужении каталога LLM
        видит все инструменты и при необходимости позовёт ``load_skill``
        сама. Поэтому здесь нет ни ретраев, ни исключений наружу —
        только попытка и метрика.
        """
        # getattr, а не прямой доступ: юнит-тесты этого репо собирают ноду
        # через object.__new__ и не исполняют __init__, поэтому атрибута
        # может не быть. Телеметрия и активация не имеют права ронять ход
        # ни в проде, ни в тестовом двойнике.
        router = getattr(self, "_skill_router", None)
        if router is None:
            return
        try:
            skill = router.route(text)
        except Exception as exc:  # noqa: BLE001 — роутер не роняет ход
            self.get_logger().debug(f"⚠️ [skills] router failed: {exc}")
            return
        if not skill:
            return
        try:
            self._core.set_active_skill(skill)
            record_skill_activation(skill, source="router")
            self.get_logger().debug(f"🧭 [skills] активирован {skill!r}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f"⚠️ [skills] activation failed: {exc}")

    def _publish_skill_load_counters(self) -> None:
        """Опубликовать прирост «домен пришлось грузить вызовом LLM».

        Доля этого источника против ``router`` — метрика промахов
        пред-роутера (задача 3.7).
        """
        core = getattr(self, "_core", None)
        if core is None or getattr(self, "_skill_router", None) is None:
            return
        try:
            loaded, misses = core.skill_load_counters
        except Exception:  # noqa: BLE001
            return
        seen_loaded, seen_misses = getattr(self, "_skill_load_seen", (0, 0))
        for _ in range(max(0, loaded - seen_loaded)):
            record_skill_activation(core.active_skill, source="llm")
        for _ in range(max(0, misses - seen_misses)):
            record_skill_activation("none", source="miss")
        self._skill_load_seen = (loaded, misses)

    def _on_prompt_stats(self, stats: Any) -> None:
        """Опубликовать размер промпта, посчитанный DialogCore.

        Колбэк зовётся из harness на КАЖДОЕ обращение к LLM, включая
        каждую итерацию тул-цикла. Harness намеренно ничего не знает про
        Prometheus — он только считает, публикует нода.

        Любое исключение здесь гасится: телеметрия не имеет права ронять
        живой ход. DialogCore тоже глушит исключения наблюдателя — это
        второй слой на случай прямого вызова из тестов.
        """
        try:
            record_llm_prompt_tokens(
                stats.provider,
                tokens=stats.prompt_tokens,
                skill=stats.skill,
                estimated=stats.estimated,
            )
            if self._verbose_llm:
                self.get_logger().debug(
                    f"[prompt] tokens={stats.prompt_tokens} "
                    f"provider={stats.provider} skill={stats.skill} "
                    f"estimated={stats.estimated}"
                )
        except Exception as exc:  # noqa: BLE001 — метрика не роняет ход
            self.get_logger().debug(f"⚠️ [metrics] prompt stats failed: {exc}")

    def _build_memory(self) -> MemoryStore:
        try:
            store: MemoryStore = SQLiteVoiceMemory(
                db_path=self.get_parameter("sqlite_db_path").value)
            future = asyncio.run_coroutine_threadsafe(store.init(), self._loop)
            future.result(timeout=5.0)
            return store
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ SQLiteVoiceMemory init failed ({exc}); InMemoryStore")
            store = InMemoryStore()
            try:
                future = asyncio.run_coroutine_threadsafe(
                    store.init(), self._loop)
                future.result(timeout=3.0)
            except Exception:
                pass
            return store
    # ── LLM provider registry (well-known defaults) ────────────────────
    # Each entry maps a provider name to its factory and constants.
    # Extend this dict to add new providers (mimo, qwen, etc.).
    # ── LLM provider registry (metadata + well-known defaults) ──────────
    # Each entry maps a provider name to its metadata.  Per-call overrides
    # (base_url, model, api_key) are read from the YAML section
    # ``<provider_name>.base_url`` etc., falling back to these defaults.
    # Extend this dict to add new providers.
    #: Well-known LLM providers. The table itself lives in
    #: ``rob_box_harness.providers.catalog`` — a ROS2-free module — so the
    #: local text-chat entry point (``scripts/dialogue/chat.py``) and this
    #: node cannot drift apart on base URLs, models or env var names. This
    #: attribute stays as the node-local alias the methods below read.
    _LLM_PROVIDER_REGISTRY: dict[str, dict[str, Any]] = LLM_PROVIDER_REGISTRY

    _BARGE_IN_POLICIES = ("replace", "classify")

    def _resolve_barge_in_policy(self) -> str:
        """Resolve ``barge_in_policy`` (S1, scheduler-segments-merge plan).

        ``"replace"`` (default) — today's behaviour: new STT input always
        stops TTS. ``"classify"`` — routes through ``quick_decide`` (S4).
        Any other value is a config typo, not a valid opt-in — warn and
        fall back to ``"replace"`` rather than silently misbehave.
        """
        raw = str(self.get_parameter("barge_in_policy").value or "replace").strip().lower()
        if raw not in self._BARGE_IN_POLICIES:
            self.get_logger().warning(
                f"⚠️ barge_in_policy={raw!r} — unknown, falling back to 'replace' "
                f"(valid: {self._BARGE_IN_POLICIES})"
            )
            return "replace"
        return raw

    def _publish_barge_in_policy(self) -> None:
        """Публикует действующий ``barge_in_policy`` для stt_node (issue #1734).

        stt_node НЕ хранит этот параметр в своём YAML (см. комментарий у
        ``_barge_in_policy_pub`` в ``__init__``) — единственный способ
        узнать актуальное значение это latched-топик
        ``/voice/dialogue/barge_in_policy``. Вызывается один раз при
        старте (сразу после создания паблишера — TRANSIENT_LOCAL
        сохранит семпл для подписчиков, стартовавших позже) и повторно
        из ``parameters_callback`` на каждое runtime-изменение через
        ``ros2 param set /dialogue_node barge_in_policy ...`` — именно
        так баг #1734 воспроизводили на роботе, и теперь это реально
        доходит до stt_node без рестарта.

        ``getattr``-guard: тесты строят ``DialogueNode`` через
        ``object.__new__`` (см. ``test_barge_in_policy.py``) и не всегда
        создают паблишер — тогда просто ничего не публикуем.
        """
        pub = getattr(self, "_barge_in_policy_pub", None)
        if pub is None:
            return
        msg = String()
        msg.data = self._barge_in_policy
        pub.publish(msg)

    def _resolve_provider_chain(self) -> list[str]:
        """Resolve the ordered list of LLM provider names from config.

        Reads ``llm_providers`` (comma-separated).
        Первый в списке = primary, остальные = fallbacks.
        Default: ``["deepseek"]``.
        """
        providers_str = str(
            self.get_parameter("llm_providers").value or "deepseek"
        ).strip()
        chain = [
            p.strip().lower()
            for p in providers_str.split(",")
            if p.strip()
        ]
        self.get_logger().info(
            f"🔗 LLM provider chain: {chain} (primary={chain[0] if chain else '?'})"
        )
        return chain

    def _build_single_provider(self, name: str) -> Any | None:
        """Build one LLM provider from its YAML section + registry defaults.

        Resolution order (per field):
        1. YAML param ``<name>.base_url`` (etc.) — если задан
        2. Registry default (``_LLM_PROVIDER_REGISTRY[name]``)
        3. Module-level constant (``MINIMAX_DEFAULT_BASE_URL`` etc.)

        API key resolution:
        1. YAML ``<name>.api_key`` (явный)
        2. Env var из registry ``env_key_var`` (напр. ``MINIMAX_API_KEY``)
        3. ``None`` — провайдер сам разберётся (или кинет ConfigError)
        """
        name = name.strip().lower()
        entry = self._LLM_PROVIDER_REGISTRY.get(name)
        if entry is None:
            self.get_logger().warning(
                f"⚠️ Unknown LLM provider: {name!r} — skipped. "
                f"Known: {sorted(self._LLM_PROVIDER_REGISTRY.keys())!r}"
            )
            return None

        display = entry["display_name"]

        # Resolve per-field: YAML → registry default → module constant
        def _p(key: str, default: str = "") -> str:
            """Read ``<name>.<key>`` from ROS param, fallback chain."""
            try:
                val = str(self.get_parameter(f"{name}.{key}").value or "").strip()
            except Exception:
                val = ""
            return val or default

        base_url = (
            _p("base_url", entry.get("default_base_url", ""))
            or MINIMAX_DEFAULT_BASE_URL  # fallback для minimax
        )
        model = (
            _p("model", entry.get("default_model", ""))
            or MINIMAX_DEFAULT_MODEL
        )

        # API key: YAML explicit → env var → None
        api_key = _p("api_key") or None
        if not api_key:
            env_var = entry.get("env_key_var", "")
            if env_var:
                api_key = os.environ.get(env_var) or None

        # Timeout / temperature / max_tokens — only if YAML set non-zero
        try:
            timeout_s = float(self.get_parameter(f"{name}.timeout_s").value or 0)
        except Exception:
            timeout_s = 0.0
        try:
            temperature = float(self.get_parameter(f"{name}.temperature").value or 0)
        except Exception:
            temperature = 0.0
        try:
            max_tokens = int(self.get_parameter(f"{name}.max_tokens").value or 0)
        except Exception:
            max_tokens = 0

        # ── Build ──────────────────────────────────────────────────
        try:
            if name == "minimax":
                provider = build_minimax_provider(
                    LLMConfig(
                        provider="minimax",
                        model=model or MINIMAX_DEFAULT_MODEL,
                        api_key=api_key,
                        timeout_s=timeout_s or 90.0,
                    )
                )
            else:
                # OpenAI-совместимые: deepseek, mimo, qwen
                provider = build_deepseek_provider(
                    api_key=api_key,
                    base_url=base_url or DEEPSEEK_DEFAULT_BASE_URL,
                    model=model or DEEPSEEK_DEFAULT_MODEL,
                )

            self.get_logger().info(
                f"✅ LLM provider built: {display} ({name}) "
                f"base_url={base_url or '(default)'} model={model or '(default)'}"
            )
            return provider
        except Exception as exc:  # noqa: BLE001
            # 🔴 FIX (live 10.08): self.get_logger() может крашнуться
            # внутри except-блока из-за _rclpy_logger_safe monkey-patch
            # (ValueError: Logger severity cannot be changed between calls).
            # Защитный fallback: пробуем rclpy-логер, при ошибке → print.
            warn_msg = (
                f"⚠️ LLM provider {name!r} ({display}) "
                f"не построен: {type(exc).__name__}: {exc}"
            )
            try:
                self.get_logger().warning(warn_msg)
            except Exception:
                try:
                    logging.warning(warn_msg)
                except Exception:
                    print(warn_msg, flush=True)
            return None

    def _build_llm(self) -> Any:
        # ── Resolve provider chain ──────────────────────────────────────
        chain_names: list[str] = self._resolve_provider_chain()

        # ── Build each provider (skip failures) ─────────────────────────
        built: list[Any] = []
        chain_display: list[str] = []
        for name in chain_names:
            provider = self._build_single_provider(name)
            if provider is not None:
                built.append(provider)
                chain_display.append(name)

        if not built:
            raise RuntimeError(
                f"No LLM providers could be built from chain {chain_names!r}. "
                "Check API keys and environment variables."
            )

        # ── Start-up config log ─────────────────────────────────────────
        temperature = float(self.get_parameter("temperature").value or 0.7)
        max_tokens = int(self.get_parameter("max_tokens").value or 500)
        self.get_logger().info(
            f"⚙️ LLM CONFIG: chain={chain_display} "
            f"temperature={temperature} max_tokens={max_tokens}"
        )

        # ── Single provider — no fallback needed ────────────────────────
        if len(built) == 1:
            # NOTE: rclpy RcutilsLogger accepts max 2 positional args
            # (fmt, args). Use f-string to avoid "takes 2 positional
            # arguments but N were given" + silent Empty assistant.
            self.get_logger().info(
                f"[health] build_llm: provider_chain={chain_display} active={chain_display[0]} (single)",
            )
            return built[0]

        # ── Multi-provider: HealthAwareFallbackLLM ──────────────────────
        # Health cache (persistent — survives robot restart)
        cache_path = str(
            self.get_parameter("health_cache_path").value or ""
        ).strip()
        try:
            health_ttl = float(
                self.get_parameter("health_ttl_s").value
                or DEFAULT_HEALTH_TTL_S
            )
        except (TypeError, ValueError):
            health_ttl = DEFAULT_HEALTH_TTL_S
        cache = HealthCache(
            ttl_s=health_ttl,
            persist_path=cache_path or None,
        )

        # Balance probes: only for providers that expose a balance API
        balance_checkers: dict[str, Any] = {}
        for i, name in enumerate(chain_display):
            entry = self._LLM_PROVIDER_REGISTRY.get(name, {})
            if entry.get("has_balance_api") and name == "deepseek":
                def _deepseek_balance_probe(
                    _name: str = name,
                ) -> Any:
                    return check_deepseek_balance(
                        DEEPSEEK_DEFAULT_BASE_URL,
                        os.environ.get("DEEPSEEK_API_KEY", ""),
                        timeout_s=5.0,
                    )
                balance_checkers["deepseek"] = _deepseek_balance_probe

        # NOTE: rclpy RcutilsLogger accepts max 2 positional args
        # (fmt, args). Use f-string to avoid "takes 2 positional
        # arguments but N were given" + silent Empty assistant.
        self.get_logger().info(
            f"[health] build_llm: provider_chain={chain_display} active={chain_display[0]} (health-aware, TTL {health_ttl:.0f}s)",
        )
        return HealthAwareFallbackLLM(
            built,
            cache=cache,
            balance_checkers=balance_checkers,
            logger=self.get_logger(),
        )
    def _build_tool_provider(self) -> ToolProvider:
        # W5a: wire the real ROSMCPToolProvider when ``tool_provider``
        # is the default ``"ros_mcp"``. The previous version silently
        # fell through to FakeToolProvider, leaving voice commands
        # like "открой шторы" / "сохрани точку" as no-ops because the
        # LLM saw an empty tool registry. The new path:
        #
        # 1. Probe the ``rob_box_mcp_tools`` package (ament_index) so
        #    we know the bridge is buildable on this image.
        # 2. Lazy-import ``LLMToolCallAdapter`` from
        #    ``rob_box_mcp_tools`` (no static ``exec_depend`` on the
        #    voice package — the import only fires when the operator
        #    asks for MCP).
        # 3. Construct the bridge + ``ROSMCPToolProvider`` and feed
        #    it the 34 manifests from ``ToolRegistry``.
        # 4. Wrap with ``LegacyToolProviderAdapter`` so the legacy
        #    ``discover/execute`` contract that ``DialogCore`` accepts
        #    is satisfied.
        # 5. Assert ``list_tools()`` is non-empty — silent regression
        #    guard against the W5a mismatch re-appearing.
        backend = str(self.get_parameter("tool_provider").value or "ros_mcp")
        # Состояние тул-поверхности. Выставляем ЗДЕСЬ, а не только в
        # ``_on_mcp_tools_update``: провайдер — единственное место, которое
        # достоверно знает, есть ли у LLM инструменты, и знает это ещё до
        # того, как придёт первое сообщение из /mcp/tools.
        self.available_tools: list = []
        self.mcp_tools_available = False
        if backend == "fake" or backend == "none":
            self.get_logger().info(
                f"⚙️ tool_provider={backend}: using FakeToolProvider "
                "(0 MCP tools, LLM chat-only)."
            )
            return FakeToolProvider()
        if backend != "ros_mcp":
            self.get_logger().warning(
                f"⚠️ Unknown tool_provider backend {backend!r}; "
                "expected one of 'ros_mcp', 'fake', 'none'. "
                "Falling back to FakeToolProvider (LLM chat-only)."
            )
            return FakeToolProvider()
        # Probe rob_box_mcp_tools so we surface a clear error if the
        # image was built without the bridge — better than letting
        # the ImportError surface deep inside the LLM tool loop.
        try:
            from ament_index_python.packages import (
                get_package_share_directory as _ament_probe,
            )
            _ament_probe("rob_box_mcp_tools")
        except Exception as exc:  # noqa: BLE001 — startup probe
            raise RuntimeError(
                "tool_provider='ros_mcp' requires the rob_box_mcp_tools "
                f"ROS package to be installed; ament_index probe failed: "
                f"{exc!r}. Either install the package, rebuild the "
                "Docker image with --packages-up-to rob_box_voice, or "
                "set the tool_provider launch arg to 'fake'/'none' for "
                "a chat-only deployment."
            ) from exc
        # Lazy import — rob_box_voice does not statically depend on
        # rob_box_mcp_tools (no <exec_depend>); the import only
        # fires here when the operator asked for MCP tools.
        try:
            from rob_box_mcp_tools.llm_adapter import LLMToolCallAdapter
        except ImportError as exc:
            raise RuntimeError(
                "tool_provider='ros_mcp' requires the Python package "
                "'rob_box_mcp_tools' to be importable. The ament "
                "probe above succeeded, but `import "
                f"rob_box_mcp_tools.llm_adapter` failed: {exc!r}. "
                "Check that the install image includes rob_box_mcp_tools."
            ) from exc
        bridge = LLMToolCallAdapter(self)
        provider = ROSMCPToolProvider(bridge)
        # Feed the 34 manifests from the harness-side catalog. The
        # provider's update_tools() expects the OpenAI-style envelope
        # ({type: function, function: {name, description,
        # parameters}}); build it from ToolRegistry.list_tools()
        # so the LLM-facing surface is the single source of truth.
        registry = ToolRegistry()
        provider.update_tools(
            [
                {
                    "type": "function",
                    "function": {
                        "name": spec.name,
                        "description": spec.description,
                        "parameters": dict(spec.parameters),
                    },
                }
                for spec in registry.list_tools()
            ]
        )
        # Silent regression guard: if the bridge came up but the
        # manifest is empty, the LLM would silently no-op every
        # tool request. Surface a loud error instead so operators
        # notice the wiring mismatch at startup.
        catalogue = provider.list_tools()
        if not catalogue:
            raise RuntimeError(
                "ROSMCPToolProvider came up with an empty tool "
                "catalogue (0 tools). The 34-manifest ToolRegistry "
                "did not register correctly. Refusing to start — "
                "voice commands would silently no-op."
            )
        self.available_tools = [
            {
                "type": "function",
                "function": {
                    "name": descriptor.name,
                    "description": descriptor.description,
                    "parameters": dict(descriptor.parameters),
                },
            }
            for descriptor in catalogue
        ]
        self.mcp_tools_available = True
        self.get_logger().info(
            f"✅ tool_provider='ros_mcp': {len(catalogue)} MCP tools "
            f"wired via LLMToolCallAdapter → ROSMCPToolProvider "
            f"(first: {catalogue[0].name!r})."
        )
        # DialogCore consumes the legacy ``discover/execute`` port
        # contract; adapt the core provider so the harness's
        # orchestration layer stays unchanged.
        provider_adapter = adapt_tool_provider(provider)
        # W7b (issue #968): route channel tools (speak_text / music /
        # anim) through the TaskScheduler. stop_music is deferred until
        # the VOICE channel drains, so it can no longer outrun the TTS
        # chunk (e2e v36). Fail-open: if the scheduler cannot start,
        # the adapter is returned unwrapped and tools execute directly.
        try:
            from rob_box_voice.scheduler.tool_executor import (
                SchedulerToolExecutor,
            )

            scheduler_executor = SchedulerToolExecutor(
                provider_adapter,
                on_event=self._on_task_event,
            )
            self._scheduler_executor = scheduler_executor
            self.get_logger().info(
                "✅ W7b: tool calls routed through TaskScheduler "
                "(voice/music/anim channels; stop_music deferred)."
            )
            return scheduler_executor
        except Exception as exc:  # noqa: BLE001 — fail-open, never break voice
            self.get_logger().warning(
                f"⚠️ W7b SchedulerToolExecutor disabled ({exc!r}); "
                "tools execute directly (pre-W7b path)."
            )
            return provider_adapter

    def _on_task_event(self, event: str, payload: dict) -> None:
        """W7c: publish scheduler lifecycle events to /harness/task_events.

        Payload format follows the issue #968 contract:
        ``{"event": "task.created", "task_id": ..., "tool": ..., ...}``.
        Publishing failures are debug-level only — the event bus must
        never break the scheduler or the dialogue loop.
        """
        try:
            pub = getattr(self, "_task_events_pub", None)
            if pub is not None:
                msg = String(
                    data=json.dumps(
                        {"event": event, **payload}, ensure_ascii=False
                    )
                )
                pub.publish(msg)
        except Exception as exc:  # noqa: BLE001 — observer must not break
            self.get_logger().debug(f"⚠️ task_events publish failed: {exc}")
        # W2-6 (issue #968) — второй наблюдатель того же события:
        # "task.updated" уже эмитится TaskScheduler._Channel.replace_args
        # (S3.2, применённый rewrite/replace MERGE-op на ещё не
        # стартовавшем сегменте). Планировщик не импортирует
        # observability напрямую — метрику считаем здесь, на стороне
        # вызывающего слоя. Отдельный try/except — не должен ронять
        # публикацию на /harness/task_events выше и наоборот.
        if event == "task.updated" and is_metrics_enabled():
            try:
                record_task_updated()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(
                    f"⚠️ [metrics] record_task_updated failed: {exc}"
                )
    def _on_vad(self, msg: Bool) -> None:
        # Use the public attribute name (no underscore) since the pure-method
        # unit tests assert against ``vad_speech_detected``. The legacy
        # ``_vad_speech_detected`` alias is kept for backwards compatibility
        # with any introspection that still looks at the underscored form.
        # Pure-method tests use ``object.__new__(DialogueNode)`` so the
        # attributes are NOT initialised in ``__init__`` — we lazily
        # create them on first VAD edge.
        # ALSO consider the public ``vad_speech_detected`` flag: tests
        # set the public one and expect the falling edge to clear it
        # (test_falling_edge_clears_vad_speech_detected).
        detected = getattr(self, "_vad_speech_detected", False) or getattr(
            self, "vad_speech_detected", False
        )
        is_rising = bool(getattr(msg, "data", False)) and not detected
        is_falling = not bool(getattr(msg, "data", False)) and detected
        if is_rising:
            self._vad_speech_detected = True
            self.vad_speech_detected = True
            self.get_logger().debug("🎤 VAD: speech start")
            # Barge-in: rising edge while the LLM is working triggers an
            # agent-loop interrupt (test_rising_edge_during_llm_sets_interrupt).
            if getattr(self, "llm_processing", False) and not getattr(
                self, "mcp_tools_available", False
            ):
                self.interrupt_agent_loop = True
        elif is_falling:
            self._vad_speech_detected = False
            self.vad_speech_detected = False

    # ═══════════════════════════════════════════════════════════════════════
    #  Event-mode / FAQ helpers (unit-test contracts for test_faq_event_mode.py)
    # ═══════════════════════════════════════════════════════════════════════

    def _load_event_profile(self) -> dict:
        """Load the active event profile from the parameter store.

        Returns a dict with keys like ``name``, ``robot_role``, ``description``
        or an empty dict when FAQ mode is disabled / no profile is configured.
        """
        try:
            faq_mode_param = self.get_parameter("faq_mode_enabled")
            enabled = bool(faq_mode_param.value) if faq_mode_param is not None else False
        except Exception:
            enabled = False
        if not enabled:
            return {}

        try:
            cfg_param = self.get_parameter("faq_event_config_file")
            cfg_value = cfg_param.value if cfg_param is not None else None
        except Exception:
            cfg_value = None
        if not cfg_value:
            return {}
        path = str(cfg_value)
        try:
            with open(path, "r", encoding="utf-8") as fh:
                data = yaml.safe_load(fh)
        except FileNotFoundError:
            self.get_logger().warning(f"Event config file not found: {path}")
            return {}
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to read event profile: {exc}")
            return {}

        if isinstance(data, dict):
            return dict(data.get("event", data) or {})
        return {}

    def _render_event_instructions(self, base_prompt: str) -> str:
        """Render the full system prompt with role + event context applied."""
        profile = getattr(self, "_event_profile", None) or {}
        if not profile:
            return base_prompt

        parts: list = []
        role = profile.get("robot_role")
        if role:
            parts.append(f"Ты — {role}.")
        name = profile.get("name")
        if name:
            parts.append(f"Ты находишься на мероприятии: «{name}».")
        org = profile.get("organization")
        if org:
            parts.append(f"Организатор: {org}.")
        loc = profile.get("location")
        if loc:
            parts.append(f"Место: {loc}.")
        date = profile.get("date")
        if date:
            parts.append(f"Дата: {date}.")
        desc = profile.get("description")
        if desc:
            parts.append(desc)

        if getattr(self, "_faq_store", None) is not None:
            parts.append(
                "ВАЖНО: сначала подними факты из FAQ (faq_search), "
                "потом стилизуй ответ. Для стилизации можешь "
                "использовать рэп или стихи. "
                "Для музыки используй execute_music_code / load_track."
            )
        else:
            parts.append(
                "Для стилизации используй рэп или стихи. "
                "Для музыки используй execute_music_code / load_track."
            )

        parts.append(base_prompt)
        return "\n\n".join(parts)

    def _build_event_faq_prefetch_context(self, query: str) -> str:
        """Return prefetched FAQ context for the supplied query, or ``None``."""
        profile = getattr(self, "_event_profile", None)
        store = getattr(self, "_faq_store", None)
        if profile is None or store is None or not query:
            return None
        event_id = profile.get("event_id") if isinstance(profile, dict) else None
        if not event_id:
            return None
        try:
            matches = store.search(query=query, event_id=event_id, limit=3)
        except Exception:
            return None
        if not matches:
            return None
        lines = ["FAQ для текущего запроса уже проверен (prefetch):"]
        for m in matches:
            q = m.get("question", "")
            a = m.get("answer", "")
            if q:
                lines.append(f"- Q: {q}")
            if a:
                lines.append(f"  A: {a}")
        lines.append(
            "Для музыкального оформления используй execute_music_code / load_track."
        )
        return "\n".join(lines)

    def _on_speaker(self, msg: String) -> None:
        """Issue #1077 — speaker_tag от Yandex speaker_analysis.

        JSON: ``{"speaker_tag": "0", "text": "...", "duration_s": 1.2}``.
        stt_node публикует speaker ПЕРЕД result, поэтому обычно этот
        callback приходит раньше ``_on_stt`` с тем же текстом. Храним по
        тексту; ``_on_stt`` забирает и создаёт/обновляет профиль спикера.
        """
        try:
            payload = json.loads(msg.data or "{}")
        except (json.JSONDecodeError, TypeError):
            return
        tag = payload.get("speaker_tag")
        text = (payload.get("text") or "").strip()
        if not tag or not text:
            return
        # Ограничиваем словарь — старые (невостребованные) записи выкидываем.
        if len(self._speaker_by_text) >= 50:
            self._speaker_by_text.clear()
        self._speaker_by_text[text] = payload
        self.get_logger().debug(
            f"👤 [issue 1077] Speaker event: tag={tag!r} "
            f"duration={payload.get('duration_s')}s text={text[:40]!r}"
        )

    def _on_speaker_result(self, msg: String) -> None:
        """Issue #1077 — результат голосовой биометрии (speaker_id_node).

        JSON: ``{"is_known": true, "speaker_id": "...", "name": "...",
        "confidence": 0.93}`` или ``{"is_known": false}``. Храним
        последний результат; _run_turn использует его для префикса
        [Говорит <имя>] / [Говорит: незнакомец] и session lock.
        """
        try:
            data = json.loads(msg.data or "{}")
        except (json.JSONDecodeError, TypeError):
            return
        # Registration ack — не speaker match.
        if data.get("event") == "registered":
            self.get_logger().info(
                f"✅ [issue 1077] Speaker registered: "
                f"{data.get('name')!r} id={str(data.get('speaker_id', ''))[:8]}"
            )
            return
        with self._speaker_lock:
            self._current_speaker = data

    def _on_command_feedback(self, msg: String) -> None:
        """Issue #1279 — озвучить feedback command_node через TTS.

        command_node публикует «Двигаюсь вперёд», «Останавливаюсь» и т.п.
        на /voice/command/feedback после выполнения команды движения/
        статуса. Раньше этот feedback никто не озвучивал (только
        context_aggregator_node слушал топик для контекста), а dialogue_node
        параллельно гнал ту же фразу в LLM — оттуда и «музыка вместо
        движения». Теперь, когда command-intent gate (issue #1279) убирает
        фразу из LLM-пути, feedback command_node озвучивается здесь, чтобы
        пользователь слышал подтверждение команды.
        """
        text = (msg.data or "").strip()
        if not text:
            return
        self.get_logger().info(f"💬 [command_node feedback] {text[:120]}")
        try:
            self._speak_direct(text)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ [issue 1279] Не удалось озвучить command feedback: {exc}"
            )

    def _on_quest_stt(self, msg: String) -> None:
        """ADR-0027 §3.4 — STT-результат с микрофона Quest (PTT robot-voice).

        ``stt_node`` публикует сюда распознанную фразу (plain text) из
        ``/audio/quest_in``. Маршрутизация — по параметру
        ``voice_input_mode`` (единственная точка переключения; его выставляет
        супервизор — ADR-0028 S5):

        - ``quest_ttts`` → **повторить голосом робота дословно** (STT → TTS,
          без LLM — это не диалог, а «озвучка моих слов»);
        - ``quest_stt`` → LLM-диалог без wake-word (Phase 2, follow-up);
        - ``quest_passthrough`` → не сюда (звук играет sound_node напрямую);
        - ``respeaker`` (default) → игнор: Quest-режим не активен.
        """
        try:
            mode = str(self.get_parameter("voice_input_mode").value or "respeaker")
        except Exception:  # noqa: BLE001 — голый объект в тестах без параметра
            mode = "respeaker"
        if mode == "quest_ttts":
            text = (msg.data or "").strip()
            if text:
                self.get_logger().info(f"🗣️ [quest] robot-voice repeat: {text[:80]!r}")
                self._speak_direct(text)
            return
        if mode == "quest_stt":
            self._on_stt(msg, from_quest=True)
            return
        self.get_logger().info(
            f"🔇 [quest] voice_input_mode={mode!r} — quest STT ignored"
        )

    def _on_stt(self, msg: String, from_quest: bool = False) -> None:
        text = (msg.data or "").strip()
        if not text:
            return
        # Issue #1195 — source marker from telegram_node: ``[TG:chat_id]
        # текст``. Означает, что текст пришёл из Telegram-чата:
        #   * wake-gate не нужен — обращение в чате очевидно;
        #   * запоминаем chat_id для маршрутизации ответа (echo-path);
        #   * голосовая биометрия ([Spkr:...]) к такому тексту НЕ
        #     применима — это не микрофон.
        tg_chat_id: Optional[int] = None
        if text.startswith("[TG:"):
            marker_end = text.find("]")
            if marker_end != -1:
                raw = text[4:marker_end].strip()
                try:
                    tg_chat_id = int(raw)
                except (TypeError, ValueError):
                    tg_chat_id = None
                if tg_chat_id is not None:
                    self._active_tg_chat_id = tg_chat_id
                    text = text[marker_end + 1:].strip()
        # ADR-0027 §3.4 — Quest robot-voice (PTT): результат пришёл через
        # ``/voice/stt/quest`` (см. ``_on_quest_stt``), а не через
        # wake-word-микрофон. Wake-word gate не нужен — оператор явно
        # зажал PTT (как и для Telegram). Источник задаёт ``from_quest``,
        # а не текстовый маркер.
        is_quest: bool = from_quest
        # W3-1 (ADR-0028 S5) — voice_input_mode="off": диалоговая нода
        # глушит ТОЛЬКО обычных людей у ReSpeaker-микрофона (этот гейт
        # смотрит именно на "голый" вход /voice/stt/result — без
        # Telegram-маркера и без Quest-флага). Вход ОПЕРАТОРА этим НЕ
        # блокируется: Telegram (tg_chat_id уже распознан выше) и Quest
        # robot-voice (is_quest=True, приходит из _on_quest_stt при
        # voice_input_mode=quest_stt) продолжают работать как обычно —
        # "off" означает «диалог выключен для окружающих, полное
        # управление у оператора» (§3.5 docs/design/
        # dialogue-mode-spec-2026-08-28.md), а НЕ «робот оглох
        # полностью». НЕ расширяй условие на tg_chat_id/is_quest —
        # это ключевое решение заказчика, разворачивать нельзя.
        if tg_chat_id is None and not is_quest:
            mode = getattr(self, "_voice_input_mode", "respeaker")
            if mode == "off":
                self.get_logger().info(
                    f"🔇 [W3-1] voice_input_mode=off — ReSpeaker вход "
                    f"игнорируется: {text[:60]!r}"
                )
                return
        text_lower = text.lower()
        # Issue #1077 — забираем speaker_tag для ЭТОГО текста (если stt_node
        # успел прислать speaker-событие). pop: один текст — один tag.
        speaker_event = self._speaker_by_text.pop(text, None)
        speaker_tag: Optional[str] = None
        speaker_duration_s: float = 0.0
        if speaker_event:
            speaker_tag = str(speaker_event.get("speaker_tag") or "")
            try:
                speaker_duration_s = float(speaker_event.get("duration_s") or 0.0)
            except (TypeError, ValueError):
                speaker_duration_s = 0.0
            if not speaker_tag:
                speaker_tag = None
        # Issue #1101 — auto-register спикера через regex УДАЛЁН.
        # Теперь LLM сам извлекает имя из user_input и вызывает MCP tool
        # register_speaker(name=X) — см. master_prompt_compact.txt RULE #SYSCTX.
        # Это решает Bug A (regex ловил «зовут» как имя из «а как меня зовут»).
        # Issue 989 Fix A: dialogue_node НЕ должен реагировать на
        # rejected(empty) — это эхо собственной музыки/голоса, а не речь
        # пользователя. Защита на случай, если stt_node начнёт публиковать
        # маркеры отклонения в /voice/stt/result (сейчас он публикует только
        # accepted, но guard дешёвый и страхует от регрессий).
        if text_lower.startswith(("rejected", "«rejected", "empty", "«пусто", "тишина")):
            self.get_logger().info(f"🔇 [issue 989] Игнор rejected/empty маркера: {text[:60]}")
            self._llm_skipped_counter["stt_rejected"] += 1
            return
        state = self._dsm.current_state
        was_idle = state == DialogueStateKind.IDLE  # FIX #992: для music_cleanup new_dialogue
        if state == DialogueStateKind.SILENCED:
            self._llm_skipped_counter["silenced"] += 1
            if is_unsilence_command(text_lower):
                self._dsm.on_event(DialogueEvent.UNSILENCE)
                self._publish_state()
            else:
                self.get_logger().info(
                    f"🔇 [diagnostics] ignored: state=SILENCED text={text[:60]!r}"
                )
            return
        # Universal wake-word gate — only direct address to robot can
        # start or interrupt a dialogue. This prevents false barge-in
        # from background noise, TV, or the robot's own TTS echo.
        # (Regression fix: was incorrectly gated on state==IDLE only.)
        #
        # Issue #1101 (diagnostics) — wake-word-miss раньше логировался
        # на debug(), поэтому в обычном логе его не видно → оператор
        # думает «LLM молчит», а на самом деле фраза не дошла до LLM.
        # Поднимаем до info() с подсчётом причин, плюс раз в окно
        # печатаем сводку ``llm_skipped_total``.
        # Issue #1195 — для текста из Telegram-чата ([TG:...]) wake-gate
        # пропускается: обращение в чате очевидно, нечего фильтровать.
        # ADR-0027 §3.4 — для Quest robot-voice (from_quest) тоже пропускаем.
        if tg_chat_id is None and not is_quest and not has_wake_word(text_lower, self._wake_words):
            accumulator = getattr(self, "_speech_accumulator", None)
            if getattr(self, "_accumulate_no_wake_enabled", False) and accumulator is not None:
                # Бэклог-аккумулятор: не дропаем, а копим фоновую речь
                # (текст + спикер + время) до следующего wake-слова.
                with self._speaker_lock:
                    sp = dict(getattr(self, "_current_speaker", {}) or {})
                sp_name = sanitize_speaker_name(sp.get("name")) if sp.get("is_known") else ""
                accumulator.add(
                    text,
                    speaker_tag=speaker_tag,
                    speaker_name=sp_name or None,
                )
                self.get_logger().info(
                    f"🗒️ [backlog] accumulated (no_wake_word) "
                    f"tag={speaker_tag!r} speaker={sp_name or 'незнакомец'!r} "
                    f"text={text[:60]!r}"
                )
            else:
                self._llm_skipped_counter["no_wake_word"] += 1
                self.get_logger().info(
                    f"🔇 [diagnostics] ignored: no_wake_word text={text[:60]!r} "
                    f"state={state.name}"
                )
                self._maybe_log_skip_summary()
            return
        accumulator = getattr(self, "_speech_accumulator", None)
        backlog_pending = bool(
            getattr(self, "_accumulate_no_wake_enabled", False)
            and accumulator is not None
            and not accumulator.is_empty()
        )
        clean = strip_wake_word(text, self._wake_words)
        if not clean:
            if backlog_pending:
                # Голое wake-слово («робот»): user_input не должен быть
                # пустым — оставляем исходную фразу как сигнал.
                clean = text
            else:
                self._llm_skipped_counter["empty_after_strip"] += 1
                self.get_logger().info(
                    f"🔇 [diagnostics] ignored: empty_after_strip_wake "
                    f"text={text[:60]!r}"
                )
                return
        if is_silence_command(text_lower):
            # 🔴 FIX (live 06.08): «хватит диджеить/музыку/трек» — это НЕ
            # silence, а запрос остановки музыки/DJ. Подстрока «хватит»
            # матчила «хватит диджеить» → робот «молчал», а музыка
            # продолжала играть. Такие команды идут в LLM (stop_music).
            if not is_music_stop_command(text_lower):
                self._llm_skipped_counter["silence_command"] += 1
                self._handle_silence()
                return
            # иначе это music-stop, фоллс на LLM ниже
        # Issue #1279 — command-intent gate: фразы, которые command_node
        # уже распознал как команды движения/статуса (NAVIGATE/STOP/
        # STATUS/MAP/...), НЕ дублируем через LLM. Иначе LLM интерпретирует
        # «вперёд» как музыку → execute_music_code вместо движения.
        # Используем тот же CommandParser с тем же входом (raw STT-текст),
        # что и command_node (один источник правды —
        # rob_box_voice.core.command_parser), поэтому классификация
        # совпадает 1:1.
        # Music-stop фразы («стоп музыку», «хватит диджеить») НЕ гейтим —
        # они должны дойти до LLM, чтобы тот вызвал stop_music.
        if (
            getattr(self, "_command_intent_gate_enabled", False)
            and tg_chat_id is None
            and not is_quest
            and not any(
                kw in text_lower for kw in self._MUSIC_STOP_OVERRIDES
            )
        ):
            command = self._command_parser.parse(text)
            if (
                command.intent != IntentType.UNKNOWN
                and command.confidence >= self._command_intent_gate_confidence
            ):
                self._llm_skipped_counter["command_intent"] += 1
                self._cancel_run("command intent (issue 1279)", stop_tts=True)
                self.get_logger().info(
                    f"🎯 [issue 1279] command intent="
                    f"{command.intent.value} conf={command.confidence:.2f} "
                    f"— LLM dispatch skipped (command_node handles): "
                    f"{text[:60]!r}"
                )
                return
        # Issue #XXXX — «новая сессия» / «сбрось всё» / Telegram «/clear»:
        # сбрасываем весь контекст текущего диалога, не гоняя фразу в LLM.
        if self._is_new_session_command(clean, text_lower, tg_chat_id):
            self._llm_skipped_counter["new_session"] += 1
            self._reset_dialogue_session()
            self.get_logger().info(
                f"🧹 [new-session] session reset: text={text[:60]!r} "
                f"tg={bool(tg_chat_id)}"
            )
            return
        if backlog_pending:
            self._pending_backlog_flush = True
        # S4.2 (scheduler-segments-merge) — barge_in_policy="classify"
        # routes the new input through quick_decide (S4.1, rules only,
        # < 50ms, no second LLM): IGNORE means noise — the turn does not
        # start at all, nothing is cancelled; REPLACE (explicit
        # imperative) reproduces today's unconditional-STOP behaviour;
        # PENDING_LLM cancels the turn WITHOUT stopping TTS (S1.2/S1.3)
        # so an already-playing segment finishes instead of being cut
        # off. "replace" (default) never calls quick_decide at all —
        # exact regression of pre-S4 behaviour.
        if getattr(self, "_barge_in_policy", "replace") == "classify":
            verdict = quick_decide(
                clean, source="tg" if tg_chat_id is not None else "stt",
                active_group=None, clock=time.monotonic,
                previous_text=getattr(self, "_last_stt_text", None),
                previous_ts=getattr(self, "_last_stt_ts", None),
            )
            self._last_stt_text = clean
            self._last_stt_ts = time.monotonic()
            # W2-6 (issue #968) — учёт вердиктов quick_decide (S4.1).
            if is_metrics_enabled():
                try:
                    record_quick_decide_verdict(verdict.value)
                except Exception as _metric_exc:  # noqa: BLE001
                    self.get_logger().debug(
                        f"⚠️ [metrics] record_quick_decide_verdict failed: "
                        f"{_metric_exc!r}"
                    )
            if verdict is QuickVerdict.IGNORE:
                self._llm_skipped_counter["quick_decide_ignore"] += 1
                self.get_logger().info(
                    f"🔇 [quick_decide] IGNORE: {clean[:60]!r}"
                )
                return
            if verdict is QuickVerdict.PENDING_LLM:
                # S7 (scheduler-segments-merge) — a genuinely in-flight
                # turn (LLM cycle not finished yet) must not be raced by
                # a second concurrent turn. Queue the phrase instead of
                # cancelling/dispatching; _run_turn's ``finally`` drains
                # the queue as ONE follow-up turn once the slot frees up
                # (§4.7.3). REPLACE (below) always cuts in regardless.
                with self._task_lock:
                    live_task = self._run_task
                if live_task is not None and not live_task.done():
                    if len(self._pending_user_messages) >= _PENDING_USER_MESSAGES_MAX:
                        dropped, _dropped_ts = self._pending_user_messages.popleft()
                        self.get_logger().warning(
                            f"⚠️ [S7] pending_user_messages overflow "
                            f"(max={_PENDING_USER_MESSAGES_MAX}), dropping "
                            f"oldest: {dropped[:60]!r}"
                        )
                    self._pending_user_messages.append((clean, time.monotonic()))
                    self.get_logger().info(
                        f"📥 [S7] turn in flight — queued: {clean[:60]!r}"
                    )
                    return
            self._cancel_run(
                "new STT input", stop_tts=verdict is QuickVerdict.REPLACE
            )
        else:
            self._cancel_run("new STT input", stop_tts=True)
        sfx = String()
        sfx.data = "thinking"
        self._sound_trigger_pub.publish(sfx)
        # Wake-word gate: when we cross from IDLE the wake-word itself
        # has to drive IDLE → LISTENING, then the speech below drives
        # LISTENING → DIALOGUE. Without the WAKE_WORD event the strip
        # above hides the trigger from DialogCore's on_user_input and
        # the DSM gets stuck in IDLE. (W6 integration tests caught
        # this regression in the W5 shell rewrite.)
        if state == DialogueStateKind.IDLE:
            self._dsm.on_event(DialogueEvent.WAKE_WORD)
            self._publish_state()
        self._dsm.on_event(DialogueEvent.STT_RESULT)
        self._publish_state()
        raw_user_command = clean
        if backlog_pending:
            hint = accumulator.format_user_hint()
            if hint:
                clean = f"{clean}\n{hint}"
        if self._dj.state.enabled:
            clean = self._dj.preamble() + clean
        if self._verbose_llm:
            self.get_logger().info(f"📥 LLM INPUT: {clean[:200]!r}")
        # Issue #1766 — логируем, что в user-turn прошёл бэклог: оператор / e2e
        # видят «backlog_pending=true» в каждом LLM INPUT и могут сматчить с
        # `backlog_handled=true` маркером в _build_dynamic_system_context, чтобы
        # доказать, что бэклог дошёл до LLM в ОБА места (user + system).
        if backlog_pending:
            self.get_logger().info(
                f"📥 LLM INPUT backlog_pending=true backlog_handled=false "
                f"(backlog_hint injected into user-turn; "
                f"backlog_handled=true появится при _build_dynamic_system_context)"
            )
        # 🔴 FIX (live 12:45): Bug C guard должен смотреть ТОЛЬКО оригинальную
        # команду юзера, а не текст с DJ-preamble. Preamble содержит
        # «диджей: ...» — guard видел его и думал «юзер просит музыку»,
        # нудил Bug C и LLM начинала DJ-сессию вместо анекдота.
        self._dispatch_turn(
            clean,
            was_idle=was_idle,
            raw_user_command=raw_user_command,
            speaker_tag=speaker_tag,
            speaker_duration_s=speaker_duration_s,
            from_tg=bool(tg_chat_id is not None),
        )
    def _on_tts_finished(self, msg: String) -> None:
        """Awaiter-release only — cleanup moved to ``_on_tts_batch_complete``.

        Issue #980: firing ``music_cleanup`` from the *first* ``/voice/tts/finished``
        event of a multi-chunk turn cut the playback short (e.g. a 35 s rap
        only played 10 s). We now trigger cleanup exclusively from
        ``/voice/tts/batch_complete`` which fires once after the last chunk.

        Issue #992: register the in-flight ``batch_id`` here (idempotent) so
        ``_on_tts_batch_complete`` can know which batches are still active
        when the LLM fires multiple ``speak_text`` calls in one cycle. The
        first chunk of each batch is enough to register — subsequent chunks
        are no-ops because ``batch_id`` is already in ``_active_batches``.
        """
        self._effects.handle_tts_finished(msg.data or "")
        try:
            payload = json.loads(msg.data or "{}")
        except (json.JSONDecodeError, TypeError):
            return
        batch_id = payload.get("batch_id")
        batch_total = payload.get("batch_total")
        if batch_id is None or batch_total is None:
            return
        try:
            self._register_active_batch(str(batch_id), int(batch_total))
        except (TypeError, ValueError):
            # Defensive: malformed batch metadata must not break the
            # awaiter-release path or the batch tracking.
            self.get_logger().debug(
                f"⚠️ [issue 992] Skipping batch registration: "
                f"batch_id={batch_id!r}, batch_total={batch_total!r}"
            )

    def _on_tts_current_voice(self, msg: String) -> None:
        """Issue #1219 — LLM voice selection: обновить current_voice.

        mcp_server (SetVoiceTool) публикует JSON {"voice": str, "provider": str}
        в /voice/tts/current_voice при вызове set_voice. Храним голос для
        контекста [TTS] (Q8): LLM видит current_voice и может вернуть его
        дефолтным голосом или сменить снова.
        """
        try:
            payload = json.loads(msg.data or "{}")
        except (json.JSONDecodeError, TypeError):
            return
        voice = payload.get("voice")
        if not voice:
            return
        self._current_tts_voice = str(voice)
        try:
            provider = payload.get("provider") or self.get_parameter("tts_provider").value
        except Exception:  # noqa: BLE001 — stub без параметра
            provider = payload.get("provider")
        self.get_logger().info(
            f"🎙️ [issue 1219] TTS current_voice → '{self._current_tts_voice}' "
            f"(provider: {provider})"
        )

    def _on_tts_provider_state(self, msg: String) -> None:
        """Issue #1229 — обновить фактический провайдер TTS (от tts_node).

        tts_node публикует JSON {"provider": str, "voice": str,
        "default_voice": str, "reason": str} после старта, при фолбеке
        провайдера (квота/сеть) и после каждого успешного синтеза.
        Храним фактического провайдера и голос — LLM-контекст [TTS]
        строится по ним (голоса РЕАЛЬНОГО провайдера, а не номинального).
        """
        try:
            payload = json.loads(msg.data or "{}")
        except (json.JSONDecodeError, TypeError):
            return
        provider = payload.get("provider")
        if not provider:
            return
        self._actual_tts_provider = str(provider)
        voice = payload.get("voice")
        if voice:
            self._actual_tts_voice = str(voice)
        # getattr — защита для тестовых стабов без __init__ (атрибут
        # инициализируется в __init__, но handler может вызваться и на
        # голом объекте; лог не должен падать).
        actual_voice = getattr(self, "_actual_tts_voice", None)
        self.get_logger().info(
            f"🎙️ [issue 1229] TTS actual provider → '{self._actual_tts_provider}' "
            f"(voice: {actual_voice}, reason: {payload.get('reason')})"
        )

    def _on_generated_music_state(self, msg: String) -> None:
        """Issue #1392 follow-up — состояние сгенерированной музыки.

        mcp_server (gen_play_from_library / stop_music) и sound_node (конец
        воспроизведения) публикуют JSON {"status": "playing"|"idle", ...}.
        Храним для <generated_music> в system_context, чтобы LLM сама
        решала, когда останавливать трек через stop_music.
        """
        try:
            payload = json.loads(msg.data or "{}")
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(payload, dict):
            return
        self._generated_music_state = payload

    def _on_music_state(self, msg: String) -> None:
        """Renardo-музыка остановилась на сервере — снять флаг «играет».

        ``/voice/music/state`` публикует mcp_server: ``"playing"`` пока у
        MusicManager есть открытая сессия или именованные паттерны, иначе
        ``"idle"``. Раньше этот топик слушал только audio_node (порог VAD,
        issue #989), а диалог вёл собственный ``_track_mode_music_active``
        по своим догадкам — и расходился с реальностью каждый раз, когда
        музыку останавливал не он: watchdog по idle_ttl, стоп из другого
        клиента, падение паттерна.

        Цена расхождения — ``build_music_retry_prompt``: при True он говорит
        модели «музыка ИГРАЕТ, её надо ИЗМЕНИТЬ, а не заводить заново».
        Сказанное про несуществующий трек уводит модель в описание вместо
        вызова тула, ретраи выгорают, и робот произносит «я растерялся».

        Только гасим. Взводит флаг по-прежнему ход диалога: там известно,
        BACKING это или TRACK, а серверу такое различие не видно.
        """
        state = (msg.data or "").strip().lower()
        if state.startswith("idle") and self._track_mode_music_active:
            self._track_mode_music_active = False
            self.get_logger().info(
                "🎵 [track-mode] сервер сообщил idle — снимаю флаг «играет» "
                "(музыку остановил не диалог: watchdog/внешний стоп)"
            )

    def _on_tts_batch_registered(self, msg: String) -> None:
        """Pre-register an in-flight TTS batch (issue #992).

        SpeakTextTool publishes this BEFORE the first ``/voice/tts/request``
        of each ``speak_text`` call so we can lock the batch_id into
        ``_active_batches`` up front. This is what makes multi-speak_text
        cycles work: each ``speak_text`` registers its own batch, the last
        ``batch_complete`` unregisters the last one, and only then do we
        fire ``/mcp/music_cleanup``.

        The registry is also updated by ``_on_tts_finished`` as a
        fallback — this prelude just makes registration *eager* so
        cleanup is correctly held across multiple batches.
        """
        try:
            payload = json.loads(msg.data or "{}")
        except (json.JSONDecodeError, TypeError):
            return
        batch_id = payload.get("batch_id")
        chunks_total = payload.get("chunks_total")
        if batch_id is None or chunks_total is None:
            return
        try:
            self._register_active_batch(str(batch_id), int(chunks_total))
        except (TypeError, ValueError):
            self.get_logger().debug(
                f"⚠️ [issue 992] Skipping batch_registered: "
                f"batch_id={batch_id!r}, chunks_total={chunks_total!r}"
            )

    def _register_active_batch(self, batch_id: str, chunks_total: int) -> None:
        """Track an in-flight TTS batch for music-cleanup gating (issue #992).

        Idempotent: re-registering an already-known ``batch_id`` is a no-op
        so the second/third chunk's ``tts_finished`` doesn't double-count.
        """
        if batch_id in self._active_batches:
            return
        self._active_batches[batch_id] = chunks_total
        self.get_logger().debug(
            f"🎵 [issue 992] Registered active batch {batch_id[:8]}... "
            f"(chunks_total={chunks_total}, "
            f"now active={len(self._active_batches)})"
        )

    def _unregister_active_batch(self, batch_id: str) -> None:
        """Drop a batch from the in-flight registry (issue #992)."""
        removed = self._active_batches.pop(batch_id, None)
        if removed is None:
            return
        self.get_logger().debug(
            f"🎵 [issue 992] Unregistered batch {batch_id[:8]}... "
            f"(remaining={len(self._active_batches)})"
        )

    def _on_tts_batch_complete(self, msg: String) -> None:
        """Fire ``music_cleanup`` once *all* TTS batches have finished (issue #992).

        ``tts_node`` publishes this after the chunk whose ``batch_index ==
        batch_total`` lands on ``/voice/tts/finished`` (success *or* failure).
        Single-chunk turns still produce exactly one ``batch_complete`` so the
        previous behaviour is preserved.

        Issue #992: when the LLM calls ``speak_text`` more than once in a
        single cycle, each call creates a *separate* ``batch_id`` and
        therefore a *separate* ``/voice/tts/batch_complete`` event. The
        previous implementation fired ``/mcp/music_cleanup`` on the first
        event, which cut music off while the second batch was still
        playing. We now unregister the completed batch from
        ``_active_batches`` and only fire cleanup when the registry is
        empty AND ``_pending_music_cleanup`` is set.
        """
        try:
            payload = json.loads(msg.data or "{}")
        except (json.JSONDecodeError, TypeError):
            payload = {}
        batch_id_raw = payload.get("batch_id")
        chunks_total = payload.get("chunks_total")
        batch_id = str(batch_id_raw) if batch_id_raw is not None else None
        if batch_id is not None:
            self._unregister_active_batch(batch_id)
        if self._active_batches:
            # At least one batch is still in flight — hold the cleanup
            # until that one (and any others) finish too.
            self.get_logger().debug(
                f"🎵 [issue 992] batch_complete for "
                f"{batch_id[:8] if batch_id else 'None'}... but "
                f"{len(self._active_batches)} batch(es) still active — "
                f"deferring music_cleanup"
            )
            return
        # 🔴 FIX (issue 992 live 09:09): LLM часто делает speak_text(прелюдия)
        # ПЕРЕД execute_music_code в одном цикле. Прелюдия — короткий batch
        # (2-5s), его batch_complete приходит ПОКА цикл LLM ещё крутится
        # (дальше Мурка/зайчики). Если чистить здесь — музыка убивается
        # через секунды после старта. Откладываем cleanup, пока _run_turn
        # активен: настоящий cleanup придёт от последнего batch_complete
        # (или 10s fallback), когда цикл завершится.
        run_task = self._run_task
        if run_task is not None and not run_task.done():
            self.get_logger().debug(
                f"🎵 [issue 992] batch_complete for {batch_id[:8] if batch_id else 'None'}... "
                "but LLM turn still active — deferring music_cleanup "
                "(prelude/pre-talk batch inside a running cycle)"
            )
            return
        if not self._pending_music_cleanup:
            self.get_logger().debug(
                "tts_batch_complete received but no pending music_cleanup"
            )
            return
        self._pending_music_cleanup = False
        self._publish_music_cleanup(reason="tts_batch_complete")
        self.get_logger().info(
            "🎵 tts_batch_complete fired music_cleanup "
            f"(chunks_total={chunks_total}, last_batch={batch_id[:8] if batch_id else 'None'}...)"
        )
    def _on_sound_state(self, msg: String) -> None:
        self._effects.handle_sound_state(msg.data or "")
    def _on_odom_snapshot(self, msg) -> None:
        """Сохранить снимок позиции {x, y, theta} из nav_msgs/Odometry для LLM-контекста."""
        try:
            pos = msg.pose.pose.position
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._pose_snapshot = {
                "x": float(pos.x),
                "y": float(pos.y),
                "theta": math.atan2(siny_cosp, cosy_cosp),
            }
        except Exception:  # noqa: BLE001
            pass
    def _on_dj_stop_farewell(self, persona: str) -> None:
        """Speak a short goodbye when DJ mode turns off.

        Invoked by DJModeController._reset_state() through the DJHook.
        We do not want the user to hear abrupt silence when the party
        ends, so we publish a one-shot speak_text via the same
        response publisher used by every other turn.
        """
        # Fall back to a friendly default if persona was empty.
        persona_part = (persona or '').strip() or 'Роббокс'
        farewell = (
            'Вечеринка подошла к концу. '
            + persona_part
            + ' выключается, но вернется по первому запросу!'
        )
        try:
            self.get_logger().info(f"DJ farewell: {farewell}")
        except Exception:
            pass
        try:
            self._publish_response(farewell, animation='happy')
        except Exception as exc:  # noqa: BLE001
            try:
                self.get_logger().warning(
                    f"DJ farewell publish failed: {type(exc).__name__}: {exc}"
                )
            except Exception:
                pass

    def _dispatch_dj_turn(self, user_input: str, from_tick: bool = False) -> None:
        """Issue #992 Bug A — DJ auto-transition dispatcher.

        Behaves like :meth:`_dispatch_turn` but:

        * does NOT publish ``music_cleanup`` with ``reason="new_dialogue"`` —
          a DJ tick is an autonomous transition, not a fresh user dialogue.
          Wiping the active track on every transition would either cut the
          music mid-phrase (no execute_music_code in this cycle) or feed
          the LLM an empty input and trip the
          "Что-то я задумался, повтори пожалуйста" fallback.
        * does NOT cancel an in-flight ``_run_task`` — DJ transitions
          compose with the existing DJ cycle instead of barging in. If
          the current task is still running (e.g. the previous DJ turn's
          LLM call hasn't returned yet) we simply queue the next tick;
          :meth:`DJModeController.tick` already defers by 15 s when a
          dialogue is active, so collisions are rare and safe.
        * forwards ``is_dj_auto=True`` so :meth:`_run_turn` applies
          DJ-specific post-turn guards (Bug B + Bug C). The DJ flag is
          threaded through the parameter rather than stored on ``self``
          to avoid a race where :meth:`_apply_music_guard`'s synchronous
          retry is dispatched *before* the next turn's ``_run_turn`` reads
          the flag — without this, a retry launched from inside the
          ``finally`` block would observe the parent's already-cleared
          flag and silently lose the ``was_dj_auto=True`` semantics.

        ``from_tick`` distinguishes a fresh tick transition (resets the
        Bug-B retry budget) from a synchronous retry triggered by the
        music-guard in :meth:`_run_turn` (keeps the budget intact).
        """
        if from_tick:
            self._music_guard.reset_for_new_dj_transition()
        self._dispatch_turn(user_input, is_dj_auto=True)

    def _dispatch_turn(
        self,
        user_input: str,
        is_dj_auto: bool = False,
        was_idle: bool = False,
        is_babble_retry: bool = False,
        is_action_claim_retry: bool = False,
        is_code_retry: bool = False,
        is_synthetic: bool = False,
        raw_user_command: str | None = None,
        speaker_tag: str | None = None,
        speaker_duration_s: float = 0.0,
        from_tg: bool = False,
    ) -> None:
        # Issue #992 Bug A — DJ auto-transitions must NOT publish
        # ``music_cleanup`` with ``reason="new_dialogue"``. Without this
        # guard the LLM cycle is reset mid-track, which in turn trips the
        # empty-assistant fallback ("Что-то я задумался, повтори
        # пожалуйста") and makes the robot sound broken during a DJ
        # session. Bug C guards against the complementary failure mode
        # (LLM skips execute_music_code).
        if is_dj_auto:
            self.get_logger().debug(
                "🎧 [issue 992] DJ auto-transition — skipping "
                "new_dialogue music_cleanup"
            )
        elif self._pending_music_cleanup and was_idle:
            # 🔴 FIX (issue 992 live 08:55): music_cleanup reason=new_dialogue
            # ТОЛЬКО при новом диалоге из IDLE. Продолжение диалога
            # (barge-in: «ещё спой про зайчиков» во время ответа) — это
            # тот же диалог: музыка живёт до tts_batch_complete (#968),
            # LLM сам сменит/оставит трек через execute_music_code.
            # Раньше cleanup слался на ЛЮБОЙ STT → музыка убивалась
            # в момент продолжения (v08:55:11 music_cleanup new_dialogue).
            self._pending_music_cleanup = False
            self._publish_music_cleanup(reason="new_dialogue")

        # Issue #1160 — Prometheus metrics: новый диалог из IDLE (первый
        # wake-word) открывает сессию; DIALOGUE_END / timeout закроет её
        # histogram'ом voice_session_duration_seconds.
        if was_idle and not is_dj_auto and self._session_started_at is None:
            self._session_started_at = time.monotonic()
            self._session_end_reason = "success"

        asyncio.run_coroutine_threadsafe(
            self._run_turn(
                user_input,
                is_dj_auto=is_dj_auto,
                is_babble_retry=is_babble_retry,
                is_action_claim_retry=is_action_claim_retry,
                is_code_retry=is_code_retry,
                is_synthetic=is_synthetic,
                raw_user_command=raw_user_command,
                speaker_tag=speaker_tag,
                speaker_duration_s=speaker_duration_s,
                from_tg=from_tg,
            ),
            self._loop,
        )

    _NAME_STOPWORDS = {
        # Two-system-prompt pattern: имя спикера извлекает LLM через
        # MCP tool register_speaker, не через regex. Старый _maybe_auto_register_speaker
        # удалён — он ловил «зовут» как имя из фразы «а как меня зовут» (Bug A в #1101).
        # Legacy constants оставлены для обратной совместимости с импортами в тестах.
        "как", "что", "это", "так", "всё", "все", "тебя", "меня", "себя",
        "робот", "роббокс", "робакс", "здесь", "там", "тут", "хочу", "буду",
        "сказал", "говорю", "прошу", "попросил",
    }

    async def _apply_speaker_identity(
        self,
        user_input: str,
        speaker_context: Optional[str],
    ) -> str:
        """Issue #1077 — префикс спикера для LLM.

        🔴 FIX (issue #1101): НЕ используем формат «[Говорит <имя>]» —
        DSM-классификатор ``on_user_input`` ищет wake-word («роббокс»,
        «робот», …) в ЛЮБОМ месте текста и матчит его внутри префикса
        «[Говорит робот Антон]» → возвращает ``WAKE_WORD`` вместо
        ``STT_RESULT`` → guard ``event == STT_RESULT`` пропускает LLM →
        «акцепт есть, робот не отвечает».

        Новый формат «[Speaker:<id> name=Антон conf=0.92]» не содержит
        wake-слов. Данные о спикере уже доступны в dynamic_system →
        LLM получает имя из system-context, а не из user-prefix.

        Args:
            user_input: Raw STT-транскрипт.
            speaker_context: Контекст от Yandex tag (если уже загружен).

        Returns:
            user_input с техническим префиксом спикера (без wake-words).
        """
        # Даём speaker_id_node время закончить inference (STT быстрее resemblyzer).
        await asyncio.sleep(0.30)
        with self._speaker_lock:
            sp = dict(self._current_speaker)
        if sp.get("is_known"):
            name = str(sp.get("name") or "")
            conf = float(sp.get("confidence") or 0.0)
            sid = str(sp.get("speaker_id") or "")[:8]
            # Issue #1787 — отдаём реплику speaker_id_node: он копит по ней
            # темы и подбирает внутреннюю кличку. Публикуем СЫРОЙ текст,
            # до служебных префиксов ([Spkr:…] исказил бы подсчёт тем).
            # Делается до проверки имени: эпитет нужен как раз тем, у кого
            # имя мусорное или отсутствует.
            self._publish_speaker_observation(
                str(sp.get("speaker_id") or ""), user_input
            )
            if name:
                # 🔴 FIX (live 12.08): защита от мусорных имён в БД
                # (resemblyzer может хранить "Null", "null", "None").
                # Такие имена — не имена, игнорируем.
                if not sanitize_speaker_name(name):
                    self.get_logger().warning(
                        f"👤 [issue 1077] Ignoring invalid speaker name: {name!r} "
                        f"(treating as unknown)"
                    )
                    if speaker_context is None:
                        user_input = f"[Speaker:unknown] {user_input}"
                    return user_input
                # 🔴 FIX (live 12.08): всегда добавляем имя спикера в
                # user_input — даже если speaker_context уже загружен.
                # LLM получает имя через dynamic_system (<system_context>),
                # но дублирующая подсказка в user_input страхует от
                # «потери» имени (баг: спикер распознан, LLM молчит).
                # Формат без wake-слов: [Spkr:Антон] — DSM не матчит
                # «робот»/«роббокс» внутри скобок.
                tag = f"[Spkr:{name}]"
                if tag not in user_input:
                    user_input = f"{tag} {user_input}"
                self.get_logger().info(
                    f"👤 [issue 1077] Speaker: {name!r} conf={conf:.2f}"
                )
        else:
            if speaker_context is None:
                user_input = f"[Speaker:unknown] {user_input}"
        return user_input

    # Issue #1787 — сколько ждём LLM на выдумывание клички. Это фоновая
    # задача, никто её не слушает в реальном времени: словарная кличка уже
    # записана, и просрочка просто оставляет её в силе. Держим таймаут
    # коротким, чтобы висящий запрос не занимал слот у провайдера.
    EPITHET_LLM_TIMEOUT_S: float = 12.0

    def _on_epithet_request(self, msg: String) -> None:
        """Issue #1787 — speaker_id_node просит LLM придумать кличку.

        JSON: ``{"speaker_id", "fallback", "cluster", "hints", "messages"}``.
        Сам запрос уходит в фон: колбэк ROS не должен ждать сеть.
        """
        try:
            data = json.loads(msg.data or "{}")
        except (json.JSONDecodeError, TypeError):
            return
        speaker_id = str(data.get("speaker_id", "")).strip()
        if not speaker_id:
            return
        try:
            asyncio.run_coroutine_threadsafe(
                self._generate_epithet(
                    speaker_id,
                    fallback=str(data.get("fallback") or ""),
                    cluster=str(data.get("cluster") or "default"),
                    hints=list(data.get("hints") or ()),
                    messages=list(data.get("messages") or ()),
                ),
                self._loop,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ [issue 1787] не удалось запустить генерацию эпитета: "
                f"{type(exc).__name__}: {exc}"
            )

    async def _generate_epithet(
        self,
        speaker_id: str,
        *,
        fallback: str,
        cluster: str,
        hints: list,
        messages: list,
    ) -> None:
        """Спросить у LLM одно слово-кличку и вернуть его speaker_id_node.

        Одноразовый запрос мимо диалогового цикла: история разговора сюда
        не идёт (кличка не должна зависеть от текущего контекста робота),
        инструменты не подключаются, ``max_tokens`` мал — нужно одно
        слово. Любая ошибка провайдера тихо оставляет словарную кличку:
        она уже записана в БД до этого вызова.

        Ответ модели НЕ применяется здесь — он публикуется как
        предложение, а решение принимает speaker_id_node, который один
        владеет БД и знает, какие клички уже заняты.
        """
        llm = getattr(self, "_llm", None)
        pub = getattr(self, "_speaker_epithet_pub", None)
        if llm is None or pub is None:
            return
        prompt = epithets.build_llm_prompt(
            messages, fallback=fallback, cluster=cluster, hints=hints
        )
        try:
            response = await asyncio.wait_for(
                llm.complete(
                    [LLMMessage(role="user", content=prompt)],
                    settings=LLMSettings(max_tokens=16, temperature=1.0),
                ),
                timeout=self.EPITHET_LLM_TIMEOUT_S,
            )
        except (asyncio.TimeoutError, ProviderError) as exc:
            self.get_logger().info(
                f"🔤 [issue 1787] LLM не придумала кличку "
                f"({type(exc).__name__}) — остаётся {fallback!r}"
            )
            return
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ [issue 1787] epithet LLM failed: {type(exc).__name__}: {exc}"
            )
            return

        proposal = epithets.sanitize_llm_epithet(getattr(response, "content", ""))
        if not proposal:
            self.get_logger().info(
                f"🔤 [issue 1787] ответ LLM не похож на кличку "
                f"({str(getattr(response, 'content', ''))[:40]!r}) — "
                f"остаётся {fallback!r}"
            )
            return
        out = String()
        out.data = json.dumps(
            {"speaker_id": speaker_id, "epithet": proposal}, ensure_ascii=False
        )
        pub.publish(out)
        self.get_logger().info(
            f"🔤 [issue 1787] LLM предложила кличку {proposal!r} "
            f"для {speaker_id[:8]} (словарная была {fallback!r})"
        )

    def _publish_speaker_observation(self, speaker_id: str, text: str) -> None:
        """Issue #1787 — отдать реплику спикера в speaker_id_node.

        Односторонний best-effort канал: если публикация не удалась (нода
        не поднята, speaker_id_node выключен параметром), спикер просто
        останется без обновления тем — на диалог это не влияет, поэтому
        любое исключение здесь глушится в лог.
        """
        pub = getattr(self, "_speaker_observe_pub", None)
        if pub is None or not speaker_id or not text.strip():
            return
        try:
            msg = String()
            msg.data = json.dumps(
                {"speaker_id": speaker_id, "text": text}, ensure_ascii=False
            )
            pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(
                f"[issue 1787] observe publish failed: {type(exc).__name__}: {exc}"
            )

    # ── Music state snapshot ───────────────────────────────────────────
    #
    # Issue #1544 — LLM нужен честный обзор того, ЧТО играет прямо сейчас,
    # чтобы решать «стоп музыку» / «давай трек» / «хватит диджеить».
    # До фикса был только <generated_music> для AI-генерации (issue #1392
    # follow-up), и DJ-бит/Renardo оставались невидимыми → LLM говорил
    # verbal «уже выключено» на стоп-команду, пока DJ-бит реально играл.
    #
    # Четыре независимых источника истины:
    #   * DJ-режим (``self._dj.state.enabled``) — autonomous DJ-сессия.
    #   * AI-сгенерированная музыка (``self._generated_music_state`` dict)
    #     — приходит из /voice/generated_music/state топика.
    #   * Активный Renardo-бит — эвристика: либо cleanup-флаг
    #     (``_pending_music_cleanup`` True → LLM запустил execute_music_code
    #     и музыка ещё живёт), либо есть активные TTS-батчи в
    #     ``_active_batches`` (музыка под backing-вокал).
    #   * cleanup_in_progress — отложенный cleanup (cleanup=True означает
    #     «бит был, сейчас будет остановлен после TTS batch_complete»).
    #
    # XML формат — компактный, один блок в <system_context>:
    #
    #   <music_state dj="playing: <theme>" ai="playing: <title>" beat="active" />
    #   <music_state dj="off" ai="idle" beat="silent" />  ← idle
    #
    # Чтобы не путать LLM лишними атрибутами, всегда рендерим ВСЕ три
    # (``dj``, ``ai``, ``beat``) — LLM видит «что есть» и «чего нет» одним
    # взглядом, без чтения нескольких тегов.
    DJ_THEME_UNKNOWN = "unknown theme"

    def _build_music_state_snapshot(self) -> str:
        """Единый <music_state> snapshot для LLM (issue #1544).

        Возвращает строку вида::

            <music_state dj="playing: <theme>" ai="playing: <title>"
                         beat="active" cleanup="pending"/>

        Все четыре атрибута рендерятся всегда (даже если ``off`` /
        ``idle`` / ``silent``) — LLM не должен угадывать по отсутствию
        тега.

        Side effects: только чтение атрибутов; ничего не публикуется и
        не модифицируется. Pure function of ``self.*`` state.
        """
        # ── DJ-режим ──
        dj_state = getattr(self, "_dj", None)
        dj_inner = getattr(dj_state, "state", None) if dj_state else None
        dj_enabled = bool(getattr(dj_inner, "enabled", False))
        if dj_enabled:
            dj_theme = (
                getattr(dj_inner, "theme", None)
                or self.DJ_THEME_UNKNOWN
            )
            dj_attr = f"playing: {dj_theme}"
        else:
            dj_attr = "off"

        # ── AI-сгенерированная музыка (топик /voice/generated_music/state) ──
        gm = getattr(self, "_generated_music_state", None) or {}
        if gm.get("status") == "playing":
            title = gm.get("title") or "без названия"
            ai_attr = f"playing: {title}"
        else:
            ai_attr = "idle"

        # ── Активный Renardo-бит (не-DJ, не-AI) ──
        # Эвристика: cleanup-флаг (LLM только что вызвал execute_music_code,
        # бит ещё жив до tts_batch_complete) ИЛИ есть активные батчи
        # TTS (музыка держится под вокал).
        # Если _music_guard_budget или _pending_music_cleanup=False и
        # _active_batches пуст — значит музыка уже не звучит.
        pending_cleanup = bool(getattr(self, "_pending_music_cleanup", False))
        active_batches = getattr(self, "_active_batches", None) or {}
        beat_attr = "active" if (pending_cleanup or active_batches) else "silent"
        cleanup_attr = "pending" if pending_cleanup else "none"

        return (
            f'  <music_state dj="{_xml_attr(dj_attr)}" '
            f'ai="{_xml_attr(ai_attr)}" '
            f'beat="{beat_attr}" '
            f'cleanup="{cleanup_attr}" />'
        )

    def _build_dynamic_system_context(self) -> str:
        """Two-system-prompt pattern: собрать <system_context> snapshot.

        Возвращает XML-строку для второго system-message в messages[].
        Содержит runtime info: текущий спикер (resemblyzer), TTS provider
        + voice (для gender alignment в ответах LLM), session lock state,
        hardware (battery, motion).

        Каждый turn собирается заново — fresh snapshot. LLM читает данные
        ТОЛЬКО из этого тега, никогда не выводит пользователю (защита от
        prompt injection — пользователь не может подделать tts_voice или
        speaker_name фразами в user_input).
        """
        # user_profile
        with self._speaker_lock:
            sp = dict(self._current_speaker)
        sp_name = sp.get("name") or ""
        sp_conf = float(sp.get("confidence") or 0.0)
        sp_id = sp.get("speaker_id") or ""
        # 🔴 FIX (live 12.08): защита от мусорных имён ("Null", "null", etc.)
        sp_name = sanitize_speaker_name(sp_name)

        # tts provider (читаем из yaml параметра)
        try:
            tts_provider = str(self.get_parameter("tts_provider").value or "minimax")
        except Exception:
            tts_provider = "minimax"
        # Issue #1229 — фактический провайдер (после фолбека) имеет приоритет
        # над номинальным из параметра. tts_node публикует его в
        # /voice/tts/provider_state после старта/фолбека/синтеза.
        actual_provider = getattr(self, "_actual_tts_provider", None)
        if actual_provider:
            tts_provider = actual_provider
        # Issue #1219 — LLM voice selection: строка [TTS] для LLM (Q8).
        # current_voice — установленный set_voice (None → дефолт провайдера).
        # Issue #1229 — если tts_node сообщил фактический голос (например,
        # при фолбеке minimax→yandex реально звучал anton), показываем его:
        # LLM видит, ЧТО реально прозвучало, а не что было запрошено.
        actual_voice = getattr(self, "_actual_tts_voice", None)
        current_voice = actual_voice or getattr(self, "_current_tts_voice", None)
        try:
            tts_context_line = format_tts_context(
                tts_provider,
                current_voice=current_voice,
            )
        except Exception:  # noqa: BLE001 — registry сбойнул, не валим диалог
            tts_context_line = f"[TTS] provider: {tts_provider}"
        # голос по умолчанию — Yandex anton (определяем по TTS config)
        tts_voice = "Yandex_Maxim"  # default — male

        # hardware (если есть доступ к батарее через /robot_status tool,
        # модель сама вызовет — но snapshot даёт baseline)
        battery = "unknown"

        # строим XML
        lines = ["<system_context>"]
        lines.append("  <user_profile>")
        if sp_name:
            lines.append(f"    <name>{sp_name}</name>")
        else:
            lines.append("    <name>unknown</name>")
        if sp_conf:
            lines.append(f"    <voice_confidence>{sp_conf:.2f}</voice_confidence>")
        if sp_id:
            lines.append(f"    <speaker_id>{sp_id[:8]}</speaker_id>")
        # Issue #1787 — внутренняя кличка. Даёт LLM якорь, когда имён
        # два одинаковых или имени нет вовсе. Research §5.3: озвучивать
        # её НЕЛЬЗЯ — юзер этого слова никогда не слышал, «привет,
        # Гроссмейстер» звучало бы как обращение к постороннему.
        sp_epithet = str(sp.get("epithet") or "").strip()
        if sp_epithet:
            lines.append(
                f"    <epithet internal=\"true\">{sp_epithet}</epithet>"
            )
            lines.append(
                "    <epithet_rule>Внутренняя метка робота для различения "
                "тёзок. НИКОГДА не произноси её вслух и не упоминай в "
                "ответе — используй только как признак «это тот же "
                "человек».</epithet_rule>"
            )
        lines.append("  </user_profile>")
        lines.append("  <hardware>")
        lines.append(f"    <battery>{battery}</battery>")
        lines.append(f"    <tts_voice>{tts_voice}</tts_voice>")
        lines.append(f"    <tts_provider>{tts_provider}</tts_provider>")
        lines.append("  </hardware>")
        # Позиция робота из /odom — LLM знает, где робот (ответы «где ты?»).
        pose = getattr(self, "_pose_snapshot", None)
        if pose:
            lines.append(
                f"  <position>x={pose['x']:.2f}, y={pose['y']:.2f}, "
                f"theta={pose['theta']:.3f} (odom frame)</position>"
            )
        else:
            lines.append("  <position>unknown</position>")
        # Issue #1219 — LLM voice selection (Q8): единая строка с голосами.
        lines.append(f"  <tts_context>{tts_context_line}</tts_context>")
        # Issue #1544 — единый <music_state> снимок: объединяет все
        # источники истины (DJ-режим + AI-генерация + активный Renardo бит +
        # cleanup-флаг). Без этого LLM не знал, играет музыка или нет, и
        # на «стоп музыку» отвечал verbal-only (без вызова stop_music tool),
        # когда DJ-бит ещё играл.
        # Issue #1392 follow-up (legacy): раньше был только <generated_music>
        # для AI-генерации; DJ/Renardo бит туда не попадал → баг #1544.
        lines.append(self._build_music_state_snapshot())
        # Issue #1544 — SYSTEM REMINDER: «стоп музыку» ведёт себя по-разному
        # в зависимости от того, ИГРАЕТ ли сейчас что-то. Без этого LLM
        # решает «нечего останавливать» → verbal «уже выключено» вместо
        # вызова stop_music tool. Один блок на весь turn — short enough.
        lines.append(
            "  <reminder>Если юзер говорит «стоп музыку / выключи / хватит "
            "диджеить»: посмотри <music_state> выше — если НЕЧТО играет "
            "(dj_active или ai_active или beat_active), ОБЯЗАТЕЛЬНО вызови "
            "stop_music tool, а потом коротко подтверди; если ВСЁ stopped — "
            "verbal «уже выключено» без tool call.</reminder>"
        )
        # Issue #1777 — SYSTEM REMINDER: русский формат времени. Tool
        # ``get_current_time`` уже возвращает ``formatted_time`` русской
        # прописью; LLM ДОЛЖЕН озвучивать его дословно через speak_text,
        # не склеивать «22:37 вечера» сам. Если LLM игнорирует tool и
        # отвечает разговорным пересказом («тридцать семь минут
        # одиннадцатого») — это регрессия #1777, см. RULE #TIME-FORMAT.
        lines.append(
            "  <reminder>Если юзер спрашивает «который час», «сколько "
            "времени», «время в Москве», «time?», «date today» — "
            "ОБЯЗАТЕЛЬНО вызови get_current_time tool, прочитай поле "
            "formatted_time дословно и озвучь его через speak_text. "
            "НЕ выдумывай время сам.</reminder>"
        )
        # Бэклог-аккумулятор фоновой речи без wake-слова: при сливе добавляем
        # <speech_backlog> внутрь <system_context>. raw_user_command при этом
        # не трогаем — гарды смотрят только на текущую фразу.
        # Issue #1766 — `backlog_handled=true` маркер в логе: оператор / e2e
        # может грепом проверить «был ли в этом turn бэклог» и сравнить с
        # acceptance (LLM должен выполнить явную команду из бэклога).
        if getattr(self, "_pending_backlog_flush", False):
            self._pending_backlog_flush = False
            acc = getattr(self, "_speech_accumulator", None)
            if acc is not None:
                block = acc.format_block()
                if block:
                    # Кол-во записей уже учтено в block (format_block → prune).
                    # Берём ДО clear() — это счётчик «сколько фраз было в
                    # бэклоге при сливе», операторский/e2e-маркер.
                    n_entries = len(acc._entries)  # noqa: SLF001 — diagnostic
                    lines.append(block)
                    self.get_logger().info(
                        f"🗒️ [backlog] flushed to LLM backlog_handled=true "
                        f"entries={n_entries} block={block[:200]!r}"
                    )
                acc.clear()
        lines.append("</system_context>")
        # W7c (issue #968): активные задачи планировщика (voice/music/anim
        # каналы) — LLM видит «что сейчас исполняется» перед каждым ходом
        # и НЕ добавляет пост-амбл «Готово!» поверх играющего трека
        # (INSIGHT #8 из W7_INTEGRATION_PLAN.md).
        executor = getattr(self, "_scheduler_executor", None)
        if executor is not None:
            try:
                block = executor.active_tasks_block()
                if block:
                    lines.append(block)
            except Exception as exc:  # noqa: BLE001 — контекст не должен падать
                self.get_logger().debug(
                    f"⚠️ active_tasks_block failed: {exc}"
                )
            # S5.2 (scheduler-segments-merge) — [SEGMENT PLAN]: LLM видит
            # ACTIVE/PENDING сегменты текущей группы и что можно
            # переписать через task_delta (S6), не начиная песню заново.
            try:
                segment_block = executor.segment_plan_block()
                if segment_block:
                    lines.append(segment_block)
            except Exception as exc:  # noqa: BLE001 — контекст не должен падать
                self.get_logger().debug(
                    f"⚠️ segment_plan_block failed: {exc}"
                )
        return "\n".join(lines)

    async def _handle_speaker_turn(
        self,
        tag: str,
        *,
        user_input: str,
        duration_s: float = 0.0,
    ) -> Optional[str]:
        """Issue #1077 — профиль спикера: подтверждение, touch, контекст.

        Вызывается из ``_run_turn`` перед LLM-вызовом, когда STT прокинул
        speaker_tag (Yandex speaker_analysis).

        Логика:
        1. ``SpeakerTracker.note_phrase`` — подтверждение tag после 2+ фраз
           подряд (>= 0.8с). Короткие (<0.8с) не создают профиль.
        2. Подтверждённый tag → ``touch_speaker``: создаёт/обновляет профиль
           (scope=speaker:<tag>, first_seen/last_seen/dialog_count).
        3. Имя из «меня зовут X» сохраняется в профиль.
        4. Факты спикера (list_facts) форматируются в LLM-контекст.

        Returns:
            Строка-контекст о спикере для system-сообщения, или ``None``
            (tag не подтверждён / профиля ещё нет / ошибка памяти).
        """
        try:
            just_confirmed = self._speaker_tracker.note_phrase(tag, duration_s)
            if not self._speaker_tracker.is_confirmed(tag):
                self.get_logger().debug(
                    f"👤 [issue 1077] tag={tag!r} ещё не подтверждён "
                    f"(streak < {self._speaker_tracker.min_phrases}) — "
                    "профиль не создаём"
                )
                return None

            profile = await touch_speaker(self._memory, tag)
            # Имя из «меня зовут X» — сохраняем в профиль (acceptance #1077).
            name = extract_speaker_name(user_input)
            if name and profile.get("name") != name:
                profile["name"] = name
                await self._memory.save_fact(
                    speaker_scope(tag),
                    Fact(
                        key="profile",
                        value=profile,
                        tags=("speaker", "profile"),
                    ),
                )
                self.get_logger().info(
                    f"👤 [issue 1077] Спикер {tag!r} представился: {name!r}"
                )

            # Факты спикера → контекст LLM (list_facts: все факты scope).
            facts = await self._memory.list_facts(speaker_scope(tag), limit=20)
            context = format_speaker_context(
                profile,
                facts,
                is_new=just_confirmed,
            )
            if context:
                self.get_logger().info(
                    f"👤 [issue 1077] Спикер {tag!r}: диалог "
                    f"#{profile.get('dialog_count', 0)} — контекст загружен"
                )
            return context
        except Exception as exc:  # noqa: BLE001 — память не должна валить диалог
            self.get_logger().warning(
                f"⚠️ [issue 1077] Speaker profile error for tag={tag!r}: {exc}"
            )
            return None

    async def _run_turn(
        self,
        user_input: str,
        *,
        is_dj_auto: bool = False,
        is_babble_retry: bool = False,
        is_action_claim_retry: bool = False,
        is_code_retry: bool = False,
        is_synthetic: bool = False,
        raw_user_command: str | None = None,
        speaker_tag: str | None = None,
        speaker_duration_s: float = 0.0,
        from_tg: bool = False,
    ) -> None:
        with self._task_lock:
            self._run_task = asyncio.current_task()
        self._run_cancelled = False
        # Issue #992 Bug B / Bug C — ``is_dj_auto`` is threaded through
        # ``_dispatch_turn`` rather than read from ``self`` so a
        # synchronous DJ retry dispatched from inside this turn's
        # ``finally`` block does not race with the parent's flag reset.
        was_dj_auto = is_dj_auto
        # Issue #992 Bug D — reset the babble-retry budget only at the
        # TOP of a *user-initiated* turn. When ``is_babble_retry=True``
        # we are inside the LLM-triggered follow-up dispatched by
        # :meth:`_check_babble_and_retry`, and the flag MUST stay True
        # so a still-babbling retry response is not escalated to a
        # second retry (which would loop forever).
        if not is_babble_retry:
            self._babble_retry_used = False
        if not is_action_claim_retry:
            self._action_claim_retry_used = False
        if not is_code_retry:
            self._code_speech_retry_used = False
        # Issue #1777 / #1762 — сброс tool-retry budget на новый
        # user-initiated turn. На babble-retry НЕ сбрасываем (как и
        # babble-budget — см. issue #992 Bug D), иначе синтетический
        # ретрай сам себе «обнулит» бюджет и при следующей такой же
        # ошибке запустит второй ретрай → ping-pong.
        if not is_babble_retry:
            self._tool_retry_used = False
        # Bug C (юзер-музыка) — сброс бюджета на НОВЫЙ юзер-запрос,
        # чтобы каждый запрос получал свежий retry (retry-промпт
        # не должен считаться новым запросом и сбрасывать сам себя).
        # DJ budget оставлен как есть — DJ-retry внутри DJ-transition
        # живёт своей жизнью и ресетится в ``_dispatch_dj_turn``
        # (``reset_for_new_dj_transition``) только при свежем тике.
        if not is_babble_retry and not was_dj_auto and not user_input.startswith(
            MUSIC_RETRY_PROMPT_PREFIX
        ):
            self._music_guard.reset_for_new_user_request()
        # Issue #992 Bug D — when the babble detector schedules a retry
        # we MUST NOT end the dialogue at the bottom of this turn. The
        # retry's ``_run_turn`` will run on the same DSM session and
        # needs to find DIALOGUE (or at least LISTENING) state so the
        # wake-word classifier short-circuits to STT_RESULT and the
        # LLM gate fires. Without this guard the first turn's
        # unconditional ``DIALOGUE_END`` drops the DSM back to IDLE,
        # the retry's ``process_input`` classifies the synthetic
        # prompt as STT_RESULT but stays in IDLE (no-op), and the
        # LLM is never called — the user hears nothing.
        # Общий флаг «гуард отправил ретрай» — сбрасывается на КАЖДЫЙ ход,
        # включая сам ретрай (иначе отложенный DIALOGUE_END залипнет).
        self._retry_dispatched_in_turn = False
        guard_retry_pending = False
        # Issue #918 — turn может быть отменён или упасть ДО присваивания
        # result (speaker-профиль, LLM, тул-луп). Инициализируем None
        # заранее, чтобы finally-блок мог безопасно отличить «результата
        # нет» от «код ниже упал» — и ВСЕГДА довести DIALOGUE_END +
        # _publish_state до конца.
        result = None
        try:
            # Issue #1077 — перед LLM-вызовом обновляем профиль спикера и
            # собираем контекст о нём (имя, факты, число диалогов). Только
            # для подтверждённых tags (2+ фразы подряд, >= 0.8с) — защита
            # от нестабильных tags Yandex. Vosk fallback (tag=None) —
            # профиль не трогаем (edge case #4).
            speaker_context: Optional[str] = None
            if speaker_tag and not is_dj_auto:
                speaker_context = await self._handle_speaker_turn(
                    speaker_tag,
                    user_input=user_input,
                    duration_s=speaker_duration_s,
                )
            # Issue #992 — DJ auto-turns must bypass the wake-word
            # classifier so the LLM is actually called. The DJ prompt
            # intentionally mentions "роббокс" / "диджей" which would
            # otherwise short-circuit into a no-op transition.
            # Issue #1077 — голосовая биометрия: префикс [Говорит <имя>] /
            # [Говорит: незнакомец] из speaker_id_node (resemblyzer d-vector).
            # Yandex speaker_tag присваивается per-session и не стабилен между
            # сессиями, поэтому для ответа «как меня зовут?» полагаемся на
            # биометрию. Session lock: первый известный спикер сессии
            # фиксируется; чужие известные/незнакомцы в той же сессии
            # игнорируются (TASK-048).
            # Issue #1195 — текст из Telegram-чата ([TG:...]): голосовая
            # биометрия НЕ применима (это не микрофон) и не должна
            # «прилипать» от последнего распознанного голосом спикера.
            # Помечаем источник для LLM префиксом [TG] (без wake-слов —
            # DSM-классификатор не матчит). Роли описаны в system prompt
            # (RULE #SRC): оператор/режиссёр vs гости в чате.
            if from_tg:
                user_input = f"[TG] {user_input}"
            elif self._speaker_id_enabled and not was_dj_auto:
                user_input = await self._apply_speaker_identity(
                    user_input, speaker_context
                )
            # Two-system-prompt pattern (live 10.08): собрать dynamic
            # <system_context> snapshot — текущий спикер (resemblyzer),
            # TTS provider/voice (для gender alignment в ответах),
            # session lock state, hardware status. Прокидывается вторым
            # system-message в messages[].
            dynamic_system = self._build_dynamic_system_context()
            # 🔴 FIX (issue #1101): _on_stt уже сделал DSM-переход
            # IDLE→LISTENING→DIALOGUE через WAKE_WORD+STT_RESULT. Передаём
            # preclassified_event=STT_RESULT чтобы DialogCore НЕ
            # переклассифицировал user-text (где может быть 'робот' внутри)
            # и не сломал guard.
            self.get_logger().info(
                f"🚀 [turn] calling process_input: user_input={user_input[:100]!r} "
                f"speaker_tag={speaker_tag!r} was_dj_auto={was_dj_auto}"
            )
            # Issue #1160 — Prometheus metrics: замер LLM-запроса.
            # ``time.monotonic`` (а не time.time) — чтобы NTP-resync
            # не сломал latency histogram. Провайдер берём из
            # текущего self._llm: для одиночного провайдера это
            # ``provider.name`` (HarnessDeepSeekProvider.name =
            # "deepseek", MiniMaxProvider.name = "minimax"); для
            # ``HealthAwareFallbackLLM`` это ``type(provider).__name__``
            # (= "HealthAwareFallbackLLM"), что норм — counter
            # ``result=fallback`` покажет сколько реально ушло на
            # fallback, а histogram latency останется на уровне цепочки.
            _llm_metric_start = time.monotonic()
            _llm_provider_name = getattr(
                self._llm, "name", type(self._llm).__name__
            )
            _llm_metric_recorded = False
            # Issue #1234 — OpenTelemetry span ``dialogue.llm_call`` (этап 2).
            # Обёртка process_input → LLM: атрибуты provider/model/fallback/
            # duration. ``start_span`` — no-op без OTel; с OTel httpx-вызовы
            # LLM-провайдера (openai SDK) станут child-spans под этим span'ом.
            try:
                with start_span(
                    "dialogue.llm_call",
                    {
                        "provider": _llm_provider_name,
                        # Модель LLM: не все провайдеры хранят её публично —
                        # getattr-защита, атрибут опционален (может быть пустым).
                        "model": getattr(self._llm, "model", "")
                        or getattr(self._llm, "_model", ""),
                    },
                ) as _llm_span:
                    # Детерминированная активация домена ДО обращения к
                    # LLM: фрагмент попадает уже в ПЕРВЫЙ запрос хода,
                    # лишнего round-trip нет.
                    self._activate_skill_for(user_input)
                    result: DialogResult = await self._core.process_input(
                        user_input,
                        is_dj_auto=was_dj_auto,
                        is_synthetic=is_synthetic,
                        speaker_tag=speaker_tag,
                        speaker_context=speaker_context,
                        dynamic_system=dynamic_system,
                        preclassified_event=DialogueEvent.STT_RESULT,
                    )
                    # Прирост «домен пришлось грузить вызовом LLM» —
                    # это и есть метрика промахов пред-роутера.
                    self._publish_skill_load_counters()
                    _llm_span.set_attribute(
                        "fallback",
                        _llm_provider_name == "HealthAwareFallbackLLM",
                    )
                    _llm_span.set_attribute(
                        "duration_s",
                        time.monotonic() - _llm_metric_start,
                    )
            finally:
                if not _llm_metric_recorded and is_metrics_enabled():
                    _llm_metric_recorded = True
                    _duration = time.monotonic() - _llm_metric_start
                    # result может быть не определён, если process_input
                    # упал до return — тогда success=False.
                    _result_obj = locals().get("result")
                    _success = _result_obj is not None and not _result_obj.error
                    # Fallback-флажок: HealthAwareFallbackLLM.complete/stream
                    # логирует fallback в свой [health] → можно отследить
                    # через ``_provider_name == "HealthAwareFallbackLLM"``.
                    # Точнее определяется через ``_last_used_provider``,
                    # который мы не видим без патча upstream. Для этапа 1
                    # довольствуемся ``result=fallback`` через отдельный
                    # record_fallback() в health.py (TODO #1160, шаг 2B).
                    try:
                        record_voice_llm_request(
                            _llm_provider_name,
                            success=_success,
                            fallback=(
                                _llm_provider_name == "HealthAwareFallbackLLM"
                            ),
                            duration_s=_duration,
                        )
                    except Exception as _metric_exc:  # noqa: BLE001
                        # Метрики НЕ должны ломать диалог: если запись
                        # упала (например, label-конфликт в тесте) —
                        # только логируем и продолжаем.
                        self.get_logger().warning(
                            f"⚠️ [metrics] record_voice_llm_request failed: "
                            f"{_metric_exc!r}"
                        )
            self.get_logger().info(
                f"✅ [turn] process_input returned: spoken={result.spoken_text!r}[:60] "
                f"tools={list(result.tools_called or ())!r} error={result.error!r}"
            )
            self._handle_result(
                result,
                user_input=user_input,
                is_dj_auto=was_dj_auto,
                raw_user_command=raw_user_command,
            )
            guard_retry_pending = bool(self._retry_dispatched_in_turn)
        except asyncio.CancelledError:
            self.get_logger().info("🛑 Turn cancelled (barge-in)")
            # Issue #1160 — Prometheus metrics: barge-in (пользователь
            # перебил робота wake-word'ом во время TTS/LLM-ответа).
            if is_metrics_enabled():
                record_barge_in()
            result = None
        except Exception as exc:  # noqa: BLE001
            # 🔴 FIX (live 12.08): говорим ДО логгирования — если логгер
            # упадёт (RcutilsLogger bug), пользователь ВСЁ РАВНО услышит
            # ответ. Раньше было наоборот: логгер падал → _speak_direct
            # не выполнялся → робот молчал (баг «принял но не ответил»).
            _tb_str = traceback.format_exc()
            try:
                if self._is_llm_unavailable_error(exc) and not was_dj_auto:
                    # 🔴 FIX (issue #1278): все LLM-провайдеры недоступны —
                    # честная degraded-фраза вместо «Что-то я задумался».
                    # Проверяем ДО generic fallback, чтобы ProviderError
                    # (health-aware-fallback) не маскировался под обычную
                    # ошибку. DJ-auto: не озвучиваем (юзер ничего не
                    # говорил, каждые ~45с это шум — см. 13.08).
                    self._speak_direct(
                        self._generate_fallback_response(
                            raw_user_command or user_input or ""
                        )
                    )
                else:
                    self._speak_direct("Что-то я задумался, повтори пожалуйста")
            except Exception:
                pass
            try:
                self.get_logger().error(
                    f"❌ DialogCore error: {exc}\n{_tb_str[-500:]}"
                )
            except Exception:
                pass
            result = None
        finally:
            # Issue #992 Bug B: ``_apply_music_guard`` may synchronously
            # dispatch a follow-up DJ turn, which sets ``self._run_task``
            # to a fresh coroutine. We must not wipe that reference here
            # or the test driver (and any future hook that watches
            # ``self._run_task``) would lose the retry. Only clear the
            # slot when no replacement was scheduled.
            with self._task_lock:
                if self._run_task is asyncio.current_task():
                    self._run_task = None
            # S7 (scheduler-segments-merge, issue #968) — drain any user
            # phrases that arrived while THIS turn's LLM cycle was in
            # flight (barge_in_policy=classify, quick_decide=PENDING_LLM,
            # see _on_stt). Must run AFTER the slot is cleared above so
            # the drained turn's own _run_turn re-entry sees a free
            # _run_task. Multiple queued phrases are glued into ONE
            # follow-up turn, never N.
            pending_queue_dispatched = self._drain_pending_user_messages()
            # Issue #935 v3: if LLM called stop_music(), defer cleanup until
            # TTS finishes.  Otherwise keep music playing until next dialogue.
            # Issue #992: a second stop_music() call from a follow-up LLM
            # turn (while a previous cleanup is still pending) must be
            # ignored — the flag is already set and the next batch_complete
            # for any active batch will fire cleanup.
            if result and "stop_music" in (result.tools_called or ()):
                if self._pending_music_cleanup:
                    self.get_logger().debug(
                        "🎵 [issue 992] stop_music deferred — already pending, "
                        "ignoring duplicate"
                    )
                else:
                    self._pending_music_cleanup = True
                    self.get_logger().info(
                        "🎵 stop_music deferred — will cleanup after TTS finishes"
                    )
            else:
                # 🔴 FIX (live 09:35): если LLM в этом цикле САМА запустила
                # музыку (execute_music_code) — НЕ убивать её по
                # tts_batch_complete короткой прелюдии («Слушай Баха!»).
                # Музыка, запущенная как композиция, живёт до segments
                # или явного stop_music. Cleanup — только если музыка
                # НЕ запускалась в этом цикле (осталась от прошлого).
                # 🔴 FIX (issue #918): turn может быть отменён (barge-in,
                # VAD-interrupt, silence) или упасть — тогда result=None
                # (except-ветки выше). Без guard'а здесь AttributeError
                # убивает finally ДО DIALOGUE_END/_publish_state → DSM
                # навсегда остаётся в DIALOGUE, /voice/dialogue/state
                # зависает на 'dialogue', scenario_runner.wait_for_idle
                # таймаутит. Guard обязателен: state finalization ниже —
                # критический контракт, music-cleanup — best-effort.
                tools_now = set(result.tools_called or ()) if result else set()
                # 🔴 FIX (live 12.08): load_track, set_dj_mode, set_vibe_preset
                # тоже запускают музыку (не только execute_music_code).
                # Без этого эмбиент/трек умолкал через ~5с после tts_batch_complete.
                # 🔴 FIX (live 30.08): та же авария повторилась с compose_music —
                # список имён теперь один на всю систему (dialogue_guards),
                # чтобы следующий музыкальный инструмент не пришлось помнить
                # добавить в двух местах.
                # 🔴 FIX (live 02.09): та же авария в третий раз — теперь с
                # ``gen_play_from_library`` (mp3 из AI-библиотеки). Live-кейс:
                # «включи трек про весну» реально запустил mp3, но
                # ``GENERATED_MUSIC_TOOLS`` тут не учитывался — флаг
                # ``_track_mode_music_active`` не взводился, следующая
                # реплика («ну вот ты включил уже») пришла из idle,
                # ``_publish_music_cleanup(reason="new_dialogue")`` считал
                # мёртвый воздух и профилактически глушил mp3 через ~14с
                # после старта. Юзер слышал, как робот 3 хода подряд врал
                # «уже играет» на самом деле остановленному треку.
                _music_starters = RENARDO_MUSIC_TOOLS | MUSIC_MODE_TOOLS | GENERATED_MUSIC_TOOLS
                # 🔴 FIX (live 31.08): stop_music не гасил флаг «играет», а
                # снимался он только в _publish_music_cleanup. После «выключи
                # музыку» флаг врал, и следующий Bug-C ретрай уходил в
                # формулировке «музыка ИГРАЕТ, измени её» — на пустоту.
                # Модель послушно описывала изменения без вызова тула, оба
                # ретрая выгорали, робот говорил «я растерялся».
                if tools_now & MUSIC_STOP_TOOLS:
                    self._track_mode_music_active = False
                if tools_now & _music_starters:
                    # Issue #992 TWO MUSIC MODES: BACKING (спой/рэп/песенку) —
                    # музыка это подложка под куплеты, систему ПРОСЯТ
                    # остановить её после финального tts_batch_complete
                    # (master_prompt_compact: "Music stops automatically after
                    # tts_batch_complete"; LLM НЕ зовёт stop_music). TRACK
                    # (сыграй баха/классику) — композиция живёт до команды
                    # юзера. Дискриминатор: BACKING = 2+ speak_text В ЭТОМ
                    # цикле И певческий интент в тексте юзера. Без интента
                    # (live 13.08: «наполни комнату музыкой» + приветствие +
                    # комментарий) — это TRACK, cleanup не планируется.
                    backing_singing = bool(result) and (
                        getattr(result, "speak_text_count", 0) >= 2
                    ) and _has_singing_intent(raw_user_command or user_input)
                    # Живая музыка теперь помнит свой режим: BACKING гасится
                    # после последнего tts_batch_complete, TRACK — живёт
                    # (live 30.08, см. ``_track_mode_music_active``).
                    self._track_mode_music_active = not backing_singing
                    if backing_singing:
                        if not self._pending_music_cleanup:
                            self._pending_music_cleanup = True
                            self.get_logger().info(
                                "🎵 [issue 992] backing mode (2+ speak_text) — "
                                "music_cleanup scheduled at tts_batch_complete"
                            )
                        else:
                            self.get_logger().debug(
                                "🎵 [issue 992] backing mode — cleanup "
                                "already pending"
                            )
                    elif self._pending_music_cleanup:
                        self._pending_music_cleanup = False
                        self.get_logger().info(
                            "🎵 [issue 992] LLM restarted music via "
                            "execute_music_code — cancelled pending cleanup"
                        )
                    else:
                        self.get_logger().debug(
                            "🎵 [issue 992] LLM started music — no cleanup "
                            "scheduled for this turn"
                        )
                elif getattr(self, "_track_mode_music_active", False):
                    # 🔴 FIX (live 30.08 15:56): ход не трогал музыку, но
                    # играет TRACK с прошлого хода — он переживает этот ход.
                    # Иначе «продолжай лабать» (или любой вопрос посреди
                    # трека) глушил композицию через 0.1 с после ответа.
                    # ASCII-тег [track-mode] — чтобы e2e мог грепнуть его
                    # без кириллицы (grep -E в check_patterns бежит по ssh,
                    # где локаль не гарантирована).
                    self.get_logger().info(
                        "🎵 [track-mode] TRACK играет с прошлого хода — "
                        "cleanup НЕ вооружаем (живёт до stop_music/watchdog)"
                    )
                elif not was_dj_auto and not self._pending_music_cleanup:
                    self._pending_music_cleanup = True
                    self.get_logger().info(
                        "🎵 music_cleanup deferred — waiting for TTS or 10s fallback"
                    )
                else:
                    self.get_logger().debug(
                        "🎵 [issue 992] music_cleanup already pending — "
                        "ignoring redundant re-arm"
                    )
            if not was_dj_auto and self._pending_music_cleanup and not self._active_batches:
                self._pending_music_cleanup = False
                self._publish_music_cleanup(reason="tts_batch_complete")
                self.get_logger().info(
                    "🎵 turn finished, no active batches — fired music_cleanup "
                    "(issue 992 prelude-deferral catch-up)"
                )
            # Issue #992 Bug B / Bug C — DJ-mode post-turn guard.
            # ``is_dj_auto`` was threaded through the dispatch path so no
            # shared flag needs to be cleared here. The guard may
            # synchronously dispatch a follow-up DJ turn while we are
            # still inside this turn's ``finally``; that is intentional —
            # ``drive_one_turn`` (and any production loop) drains the
            # retry as part of this very turn cycle.
            #
            # 🔴 FIX (issue #1204, 13.08 DJ incident): guard вызывается
            # ДО DIALOGUE_END. Сам guard переоткрывает DIALOGUE перед
            # ретраем, поэтому закрывать диалог здесь нужно только если
            # ретрай НЕ был задиспатчен — иначе ретрай-тур придёт в IDLE
            # и его process_input короткозамкнётся без вызова LLM.
            music_retry_dispatched = self._apply_music_guard(
                was_dj_auto=was_dj_auto,
                user_input=raw_user_command or user_input,
                tools_called=result.tools_called if result else (),
            )
            # Issue #1777 / #1762 — Bug C retry для non-music tool-based
            # запросов. Раньше ретрай работал ТОЛЬКО для music (issue
            # #992 Bug C). Теперь если юзер явно просит
            # ``get_current_time`` / ``search_web`` / ``set_voice`` /
            # ``memory_search`` / ``faq_search`` и LLM не вызвал tool
            # (tools пустой), отправляем ОДИН CRITICAL retry с явным
            # указанием нужного tool. Защита от ping-pong: один
            # ретрай на turn (см. ``_tool_retry_used``).
            #
            # Вызывается ПОСЛЕ music guard и babble guard — чтобы не
            # гонять их по очереди и не дублировать retry для
            # пересекающихся случаев (например «который час» не должен
            # матчиться music guard'ом).
            tool_retry_dispatched = self._apply_tool_skipped_guard(
                user_input=raw_user_command or user_input,
                tools_called=result.tools_called if result else (),
                other_retry_dispatched=music_retry_dispatched,
            )
            # Issue #992 Bug D — defer the DIALOGUE_END transition
            # when the babble detector scheduled a retry. The retry's
            # ``_run_turn`` needs the DSM to stay in DIALOGUE so the
            # LLM gate fires; otherwise the synthetic prompt is
            # classified as STT_RESULT but the state stays in IDLE
            # (no-op), and the user never hears the retry answer.
            # S7 — same reasoning for a drained pending-queue follow-up
            # turn: it is a continuation of the same session, not the
            # end of it.
            if (
                self._dsm.current_state == DialogueStateKind.DIALOGUE
                and not guard_retry_pending
                and not music_retry_dispatched
                and not pending_queue_dispatched
                and not tool_retry_dispatched
            ):
                self._dsm.on_event(DialogueEvent.DIALOGUE_END)
                # Issue #1160 — Prometheus metrics: сессия закрылась
                # штатно (DIALOGUE_END) — пишем duration histogram.
                self._maybe_record_session_end(result="success")
            # DialogCore completes the DIALOGUE → IDLE transition itself.
            # Publish the resulting state even when no transition is needed
            # here; otherwise the ROS state topic remains stuck at the
            # earlier DIALOGUE notification and scenario runners wait forever.
            self._publish_state()

    # ── Issue #992 Bug B / Bug C — DJ-mode music guard ────────────────

    # Issue #1016 Bug C — narrow keyword heuristic. ``трек`` and ``бит``
    # are deliberately excluded because they fire on chit-chat like
    # "роббокс какой трек посоветуешь?" (issue 992 test_user_normal_chat
    # regression). Keep the list focused on unambiguous "play something
    # NOW" commands so the spoken nudge only fires when the user clearly
    # asked for generated music.
    # Issue #992 Bug C/D — keyword sets live in
    # :mod:`rob_box_voice.core.dialogue_guards` (TD-1 decomposition);
    # class-level aliases keep ``node._MUSIC_GUARD_KEYWORDS`` etc.
    # working for existing call sites and tests.
    _MUSIC_GUARD_KEYWORDS = MUSIC_GUARD_KEYWORDS

    # Issue #1016 — empty-response music fallback. Более широкая эвристика
    # чем _MUSIC_GUARD_KEYWORDS: используется ТОЛЬКО в ветке «LLM вернула
    # пустоту и не вызвала ни одного тула» — там цена ложного
    # срабатывания ниже (вместо «Принял.» + тишины юзер услышит топ-трек
    # из библиотеки). «Поставь что-нибудь» — канонический TRACK-триггер
    # из issue #1016, поэтому «поставь» и «включи» входят сюда.
    # Жанровые слова (джаз/рок/блюз) и «трек»/«бит» сознательно НЕ
    # включены — они встречаются в вопросах-рекомендациях («какой трек
    # посоветуешь?»), где музыка ни к чему.
    _MUSIC_FALLBACK_KEYWORDS = (
        "спой",
        "пой ",
        "рэп",
        "рап",
        "диджей",
        "dj ",
        "dj-",
        "песня",
        "песню",
        "песенк",
        "зачитай",
        "зачита",
        "зачитывай",
        "сыграй",
        "играй",
        "поставь",
        "включи",
        "музык",
        "мелоди",
        "классик",
        "танцевальн",
    )



    # 🔴 FIX (live 06.08): «хватит диджеить/выключи музыку» — юзер просит
    # остановить музыку/DJ, а НЕ замолчать робота. Подстрока «хватит»
    # в silence_commands перехватывала такие команды до LLM. Эти фразы
    # пробивают silence-гейт и идут в LLM (который вызовет stop_music +
    # set_dj_mode(enabled=false)).
    _MUSIC_STOP_OVERRIDES = MUSIC_STOP_OVERRIDES

    # 🔴 FIX (live 10:00): для ГОЛОСОВЫХ запросов («спой/пой/песня»)
    # speak_text достаточно — бит не обязателен (юзер мог попросить
    # спеть ПОД уже играющую музыку, как «спой про мурку в этот
    # момент» — Григ играл, LLM правильно не перезапустила трек).
    # Bug C нудит только если LLM вообще НИЧЕГО не сделала (tools
    # пуст). Для БИТО-обязательных («рэп/зачитай/диджей») — как было:
    # нуднуть если нет execute_music_code.
    _MUSIC_GUARD_VOCAL_KEYWORDS = MUSIC_GUARD_VOCAL_KEYWORDS

    # Issue #XXXX — «новая сессия» / «сбрось всё» / Telegram «/clear»:
    # голосовые фразы, после которых робот сбрасывает весь контекст
    # текущего диалога (историю, DSM, speaker-состояние, бэклог) и
    # начинает с чистого листа. Матч — по подстроке в lowercased-тексте
    # ПОСЛЕ снятия wake-слова. Список можно переопределить через YAML
    # параметр ``new_session_phrases``.
    _DEFAULT_NEW_SESSION_PHRASES: tuple[str, ...] = (
        "новая сессия",
        "новую сессию",
        "новый диалог",
        "новый разговор",
        "начать заново",
        "начни заново",
        "начнём заново",
        "начнем заново",
        "начать сначала",
        "начни сначала",
        "сбрось всё",
        "сбрось все",
        "сбросить всё",
        "сброс сессии",
        "сброс диалога",
        "сбросить диалог",
        "забудь всё",
        "забудь все",
        "забудь что было",
        "очисти историю",
        "очистить историю",
        "сотри историю",
        "стереть историю",
    )

    def _is_new_session_command(
        self,
        clean: str,
        text_lower: str,
        tg_chat_id: Optional[int],
    ) -> bool:
        """Issue #XXXX — пользователь просит начать новую сессию?

        Матчит голосовые фразы (см. ``_DEFAULT_NEW_SESSION_PHRASES`` /
        параметр ``new_session_phrases``) и Telegram-команду ``/clear``.
        Чистая функция от входных строк — без I/O, тестируется без ROS2.
        """
        if not getattr(self, "_new_session_enabled", True):
            return False
        if tg_chat_id is not None and (clean or "").strip().lower() == "/clear":
            return True
        target = (clean or text_lower or "").lower()
        phrases = getattr(self, "_new_session_phrases", None) or (
            self._DEFAULT_NEW_SESSION_PHRASES
        )
        return any(phrase in target for phrase in phrases)

    def _user_wants_music(self, user_input: str) -> bool:
        """Heuristic: does the user request music / a track?

        Used by :meth:`_apply_music_guard` to decide whether Bug C's
        code-side fallback should fire. The check is intentionally
        narrow so we don't retry on ordinary chit-chat that happens
        to mention "track" in passing.

        Delegates to :func:`rob_box_voice.core.dialogue_guards.user_wants_music`
        (TD-1 decomposition).
        """
        return user_wants_music(user_input, logger=self.get_logger())

    def _user_wants_music_fallback(self, user_input: str) -> bool:
        """Issue #1016 — broader heuristic for the empty-response fallback.

        Unlike :meth:`_user_wants_music` (which drives the spoken
        "бит не запустился" nudge), this one is used ONLY in the
        empty-response branch where the LLM returned nothing and no
        tool was called. There the cost of a false positive is low —
        the robot would otherwise say «Принял.» and stay silent, so
        playing the top library track is strictly better.
        """
        if not user_input:
            return False
        low = user_input.lower()
        matched = [kw for kw in self._MUSIC_FALLBACK_KEYWORDS if kw in low]
        if matched:
            self.get_logger().debug(
                f"🎵 [music_fallback] user_input={user_input!r} matched "
                f"keywords={matched!r} → fallback music"
            )
            return True
        return False

    # ── Issue #992 Bug D — metalanguage / babble detection ───────────

    def _is_metalanguage_babble(self, spoken_text: str) -> bool:
        """Issue #992 Bug D — does this LLM output read as meta-talk?

        Returns ``True`` when the LLM final response text starts with a
        known metalanguage opener («зачита», «могу», «хочешь»,
        «сейчас», «устроим», «погнали», «давай», «слушай», «окей»,
        «так», «переключ», «ну что ж»). The check operates on the
        first 80 chars after :func:`strip_markdown` so a lone "**"
        that survived cleaning cannot mask the opener.

        The detector is intentionally *conservative*: a normal answer
        that happens to contain «слушай» somewhere in the middle is
        safe — only the first 80 chars are inspected. When in doubt,
        return ``False``; :meth:`_check_babble_and_retry` will fall
        through to the standard TTS publish path.

        Delegates to
        :func:`rob_box_voice.core.dialogue_guards.is_metalanguage_babble`
        (TD-1 decomposition).
        """
        return is_metalanguage_babble(spoken_text)

    def _user_wants_performance(self, user_input: str) -> bool:
        """Issue #992 Bug D — does the user request a *performance*?

        Used to decide whether a metalanguage reply is a hard bug
        (user asked for a rap, robot returned "Зачитаю рэп про X!") or
        just a stylistic miss (user asked "что нового?", robot replied
        "Слушай, у меня тут..." — still answer-shaped, just informal).

        Delegates to
        :func:`rob_box_voice.core.dialogue_guards.user_wants_performance`
        (TD-1 decomposition).
        """
        return user_wants_performance(user_input)

    def _check_babble_and_retry(
        self,
        *,
        spoken: str,
        user_input: Optional[str],
        tools_called: tuple,
        speak_text_real: int = 0,
    ) -> bool:
        """Issue #992 Bug D — single-shot babble retry dispatcher.

        Inspects ``spoken`` (the LLM final text after strip_markdown)
        and decides whether to force ONE retry with a CRITICAL
        reminder. Returns ``True`` when a retry was scheduled (so the
        caller should skip the regular TTS publish), ``False`` when the
        detector passed and the caller should proceed normally.

        Retry rules — all must hold for a retry to fire:
        1. ``speak_text`` was NOT really called this cycle — i.e.
           ``speak_text_real`` is 0. A real call is already handled by
           the issue-988 anti-duplicate path; a *phantom* call
           (``speak_text({})`` with empty text, issue #1343) did NOT
           voice anything and must not suppress the babble retry.
        2. ``spoken`` is non-empty and starts with a banned opener
           (see :data:`BABBLE_BANNED_OPENERS`).
        3. The user request looks like a performance command OR the
           babble phrase is unambiguously a "promise to do" — i.e. the
           opener is in the *promise* subset («зачит», «погнали»,
           «устроим», «переключ», «давай-ка»). Other openers
           («слушай,», «окей,», «так,») only retry when the user
           explicitly asked for a performance.
        4. We have NOT already used our one-shot babble retry for
           this turn (avoids an infinite LLM ping-pong).
        """
        if speak_text_real > 0:
            return False
        if not spoken:
            return False
        if getattr(self, "_babble_retry_used", False):
            return False
        if not self._is_metalanguage_babble(spoken):
            return False
        # 🔴 FIX (live 02.09): планирование модели вслух («Юзер просит...,
        # запускаю через compose_music») — НИКОГДА не валидный ответ, что бы
        # ни просил юзер. Гейт по user_wants_performance тут не нужен: в
        # живом логе он и не сработал (юзер сказал «ебани ланудж», ни одного
        # ключевого слова), и робот зачитал план вслух, не вызвав тулов.
        user_wants_perf = self._user_wants_performance(user_input or "")
        if not user_wants_perf and not is_planning_narration(spoken):
            # The promise-only subset always retries regardless of
            # user_input — these phrases are NEVER valid answers.
            promise_only = (
                "зачит", "погнали", "устроим",
                "переключ", "давай-ка",
            )
            head = spoken[:60].lower()
            if not any(p in head for p in promise_only):
                return False
        # Issue #992 Bug D — the retry turn needs the same DSM state
        # transitions as a real STT input (IDLE → LISTENING → DIALOGUE)
        # — otherwise DialogCore's process_input sees IDLE and returns
        # an empty result, which trips the
        # "Что-то я задумался, повтори пожалуйста" fallback. This is
        # exactly the wake-word gate logic from ``_on_stt``.
        try:
            if self._dsm.current_state == DialogueStateKind.IDLE:
                self._dsm.on_event(DialogueEvent.WAKE_WORD)
                self._publish_state()
            self._dsm.on_event(DialogueEvent.STT_RESULT)
            self._publish_state()
        except ImportError:
            # dialog_state_machine is part of rob_box_harness; if it
            # ever disappears the safe default is to skip the
            # transition and let process_input return an empty result.
            pass
        # Mark the retry as used BEFORE dispatching so a re-entrant
        # call from the retry itself can never escalate to a second
        # retry.
        self._babble_retry_used = True
        self._mark_retry_dispatched()
        retry_prompt = self._build_babble_retry_prompt(user_input or "")
        self.get_logger().warning(
            "🗣️ [issue 992 Bug D] LLM babble detected — retrying once with "
            f"CRITICAL reminder (head={spoken[:60]!r})"
        )
        # Issue #1204: ``user_input`` здесь — уже оригинальная команда
        # юзера (caller передал raw_user_command or user_input). Форвардим
        # её как raw_user_command, иначе music-гуард на ретрай-туре
        # просканирует синтетический babble-промпт (в нём есть «песня»)
        # и запустит ложный music-ретрай.
        self._dispatch_turn(
            retry_prompt,
            is_babble_retry=True,
            # Synthetic prompt — never persisted as something the user said.
            is_synthetic=True,
            raw_user_command=user_input,
        )
        return True

    def _build_babble_retry_prompt(self, user_input: str) -> str:
        """Issue #992 Bug D — synthetic follow-up prompt for babble retry.

        Echoes the original ``user_input`` so the LLM has the request
        in context, then appends a CRITICAL instruction that names the
        babble pattern and demands a tool-call reply (no plain text
        promises).

        Delegates to
        :func:`rob_box_voice.core.dialogue_guards.build_babble_retry_prompt`
        (TD-1 decomposition).
        """
        return build_babble_retry_prompt(user_input)

    def _check_embedded_renardo_code_and_retry(
        self,
        *,
        spoken: str,
        user_input: Optional[str],
        tools_called: tuple,
    ) -> bool:
        """Issue #992 Bug C' — одноразовый ретрай, когда LLM зачитывает код.

        Live 30.08: модель сочинила мелодию и написала Renardo-код в текст
        ответа (``p1 >> keys(...)``, ``Clock.bpm = ...``) вместо вызова
        ``execute_music_code(code=...)`` — TTS зачитал код вслух. Находим
        строки кода и требуем ОДИН ретрай с вызовом тула и тем же кодом.

        Returns:
            ``True`` — ретрай отправлен, вызывающий НЕ должен публиковать
            текст в TTS.
        """
        if getattr(self, "_code_speech_retry_used", False):
            return False
        if tools_called:
            # LLM уже вызвала тул в этом цикле — не вмешиваемся.
            return False
        if not spoken:
            return False
        code = extract_renardo_code_lines(spoken)
        if not code:
            return False

        # Тот же перевод DSM, что и в babble-ретрае: без него process_input
        # увидит IDLE и вернёт пустой результат.
        try:
            if self._dsm.current_state == DialogueStateKind.IDLE:
                self._dsm.on_event(DialogueEvent.WAKE_WORD)
                self._publish_state()
            self._dsm.on_event(DialogueEvent.STT_RESULT)
            self._publish_state()
        except ImportError:
            pass

        # Помечаем ДО отправки — реентрантный вызов из самого ретрая не
        # должен уметь запустить второй.
        self._code_speech_retry_used = True
        self._mark_retry_dispatched()
        self.get_logger().warning(
            "🎹 [issue 992 Bug C'] Renardo-код в тексте реплики — "
            "требую execute_music_code(code=...) "
            f"(code head={code[:80]!r})"
        )
        self._dispatch_turn(
            build_renardo_code_retry_prompt(code),
            is_code_retry=True,
            is_synthetic=True,
            raw_user_command=user_input,
        )
        return True

    def _check_unbacked_action_claim_and_retry(
        self,
        *,
        spoken: str,
        user_input: Optional[str],
        tools_called: tuple,
    ) -> bool:
        """Issue #992 Bug E — одноразовый ретрай «сказал, но не сделал».

        Живой прогон 30.08 (vision-pi 12:31–12:38): восемь ходов из
        восемнадцати заканчивались утверждением о выполненном действии при
        пустом ``tools_called``. «Точка сохранена.» — а двумя ходами позже
        «Точек пока нет». Детектор узкий (см.
        :data:`~rob_box_voice.core.dialogue_guards.ACTION_CLAIM_RULES`):
        должны совпасть И запрос юзера, И формулировка отчёта, И отсутствие
        нужного тула.

        Returns:
            ``True`` — ретрай отправлен, вызывающий НЕ должен публиковать
            текст в TTS (иначе юзер услышит неправду, а потом ответ ретрая).
        """
        if getattr(self, "_action_claim_retry_used", False):
            return False
        rule = detect_unbacked_action_claim(
            user_input=user_input,
            spoken=spoken,
            tools_called=tuple(tools_called or ()),
        )
        if rule is None:
            return False

        # Тот же перевод DSM, что и в babble-ретрае: без него
        # process_input увидит IDLE и вернёт пустой результат.
        try:
            if self._dsm.current_state == DialogueStateKind.IDLE:
                self._dsm.on_event(DialogueEvent.WAKE_WORD)
                self._publish_state()
            self._dsm.on_event(DialogueEvent.STT_RESULT)
            self._publish_state()
        except ImportError:
            pass

        # Помечаем ДО отправки — реентрантный вызов из самого ретрая
        # не должен уметь запустить второй.
        self._action_claim_retry_used = True
        self._mark_retry_dispatched()
        self.get_logger().warning(
            f"🧾 [issue 992 Bug E] заявлено действие без тула "
            f"(category={rule.category}, tools={list(tools_called)!r}, "
            f"spoken={spoken[:60]!r}) — один ретрай"
        )
        self._dispatch_turn(
            build_unbacked_action_retry_prompt(
                user_input=user_input or "", spoken=spoken, rule=rule
            ),
            is_action_claim_retry=True,
            is_synthetic=True,
            raw_user_command=user_input,
        )
        return True

    def _reopen_dialogue_for_retry(self) -> None:
        """Re-drive the DSM to DIALOGUE before a synchronous retry dispatch.

        ``dialog_core.process_input`` gates the LLM on
        ``current_state == DIALOGUE``. The parent turn's ``process_input``
        already fired ``DIALOGUE_END``, so by the time the post-turn guard
        runs the state is IDLE — a retry dispatched from here would
        short-circuit and never reach the LLM (13.08 DJ incident: retry
        returned in ~1 ms with zero ``[health]`` logs).

        Same pattern as ``_check_babble_and_retry`` (issue #992 Bug D).
        """
        if self._dsm.current_state == DialogueStateKind.IDLE:
            self._dsm.on_event(DialogueEvent.WAKE_WORD)
            self._publish_state()
        self._dsm.on_event(DialogueEvent.STT_RESULT)
        self._publish_state()

    def _mark_retry_dispatched(self) -> None:
        """Пометить, что гуард отправил синхронный ретрай в этом ходе.

        Переоткрыть DSM (``_reopen_dialogue_for_retry``) — половина дела:
        родительский ход в своём ``finally`` всё равно пошлёт
        ``DIALOGUE_END`` и уронит DSM обратно в IDLE ДО того, как ретрай
        доберётся до ``process_input``. Флаг говорит ``_run_turn``
        отложить закрытие диалога (issue #1204).

        Каждый ``_check_*_and_retry`` обязан вызвать этот метод рядом со
        своим одноразовым флагом — иначе ретрай уйдёт в закрытый диалог и
        вернётся пустым за миллисекунду. Именно так легли шаги
        tc12_delete_track и tc16_delete_waypoint в e2e 33251879328.
        """
        self._retry_dispatched_in_turn = True

    def _discard_last_music_reply(self) -> None:
        """Fire-and-forget: retract the last persisted assistant turn.

        Issue #992 — called from :meth:`_apply_music_guard` the moment a
        guard has CONFIRMED a music request got no tool call (its own
        retry included). ``DialogCore.discard_last_reply`` is a coroutine
        and this method runs on the ROS2 callback thread, so it is
        scheduled on the asyncio loop the same way ``_dispatch_turn``
        schedules ``_run_turn`` — fire-and-forget, with a done-callback
        only to log failures (losing the retraction is not fatal, the
        turn stays in history same as before this fix).
        """
        future = asyncio.run_coroutine_threadsafe(
            self._core.discard_last_reply(), self._loop
        )

        def _log_if_failed(fut: "asyncio.Future[bool]") -> None:
            try:
                removed = fut.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(
                    f"🎵 [issue 992] discard_last_reply failed: {exc}"
                )
                return
            if not removed:
                self.get_logger().debug(
                    "🎵 [issue 992] discard_last_reply: no assistant turn "
                    "found to retract"
                )

        future.add_done_callback(_log_if_failed)

    def _apply_music_guard(
        self,
        *,
        was_dj_auto: bool,
        user_input: str,
        tools_called: tuple,
    ) -> bool:
        """Adapter around :meth:`MusicGuard.evaluate` — keeps the ROS2
        side effects (dispatch, speak_direct, dialogue-reopen) out of
        the policy module so :class:`MusicGuard` is unit-testable.

        Issue #992 Bug B — DJ auto-transitions: the LLM is asked to
        play track #N through the music tools, but it frequently
        replies with just a spoken phrase and no ``execute_music_code``
        tool call. Without this guard the DJ cycle silently produces
        zero audio for that transition. We re-arm
        ``next_transition_at`` to ``now + POSTPONE_INTERVAL_S`` so the
        next tick fires shortly, AND publish a one-shot synthetic
        prompt that injects a CRITICAL reminder to call the tool.

        Issue #992 Bug C — user rap/song commands: even outside DJ
        mode, when the user asks for a rap/poem/song and the LLM
        forgets ``execute_music_code``, we publish a short spoken
        acknowledgment so the user hears *something* and can repeat
        the request. We deliberately do NOT auto-pick a beat without
        user consent: that would surprise the operator.

        Returns:
            ``True`` when a synchronous retry turn was dispatched — the
            caller (:meth:`_run_turn` finally block) must then defer its
            own ``DIALOGUE_END`` so the retry's LLM gate fires
            (issue #1204). ``False`` otherwise.
        """
        # 🔴 FIX (live 30.08, e2e renardo_evolve rn02): на «продолжай
        # развивать эту мелодию и добавь баса» СРАЗУ сработали Bug D
        # (ответ начинался с «Окей,») и Bug C (музыкального тула нет) —
        # ушло ДВА синтетических ретрая, вернулось два ответа, и юзер
        # услышал подряд «Тема рассвета с басом — поехали» и «Бас добавлен,
        # мелодия мягко плывёт». Один промах модели = один ретрай: если
        # гуард уже отправил ретрай в этом ходе, музыкальный молчит —
        # ретрай-тур всё равно будет оценён заново.
        if self._retry_dispatched_in_turn:
            self.get_logger().info(
                "🎵 [music_guard] в этом ходе ретрай уже отправлен — "
                "music-гуард пропускает (без двойного дубля ответа)"
            )
            return False

        verdict = self._music_guard.evaluate(
            was_dj_auto=was_dj_auto,
            user_input=user_input,
            tools_called=tuple(tools_called or ()),
            dj_enabled=self._dj.state.enabled,
            build_music_retry_prompt=self._build_music_retry_prompt,
            build_dj_retry_prompt=self._build_dj_retry_prompt,
        )

        if verdict.kind is MusicGuardVerdictKind.SKIP:
            return False

        if verdict.kind is MusicGuardVerdictKind.DJ_RETRY:
            assert verdict.prompt is not None  # build_dj_retry_prompt is wired
            self._dj.state.next_transition_at = (
                time.time() + DJModeController.POSTPONE_INTERVAL_S
            )
            # Issue #1204: ретрай должен реально дойти до LLM —
            # переоткрываем DIALOGUE (process_input уже закрыл его).
            self._reopen_dialogue_for_retry()
            self._dispatch_dj_turn(verdict.prompt)
            return True

        if verdict.kind is MusicGuardVerdictKind.USER_RETRY:
            assert verdict.prompt is not None
            # Issue #992 — the attempt we just evaluated (tools_called
            # empty on a music request) already had its assistant reply
            # persisted by DialogCore as an ordinary successful turn (see
            # ``discard_last_reply`` docstring). Retract it BEFORE
            # dispatching the retry so the next LLM call — and every
            # later turn — doesn't read its own unlabeled false
            # confirmation back as a good example to imitate.
            self._discard_last_music_reply()
            # Issue #1204: ретрай должен реально дойти до LLM —
            # переоткрываем DIALOGUE (process_input уже закрыл его).
            self._reopen_dialogue_for_retry()
            self._dispatch_turn(
                verdict.prompt,
                was_idle=False,
                raw_user_command=user_input,
                # ``verdict.prompt`` is our [CRITICAL] reminder, not the
                # user's words — the real request is ``raw_user_command``
                # and it was already written to history by the turn that
                # triggered this retry.
                is_synthetic=True,
            )
            return True

        if verdict.kind is MusicGuardVerdictKind.FORCE_STOP:
            # 🔴 FIX (live 30.08): юзер сказал «останови музыку», LLM
            # ответила «Музыка выключена.» и не вызвала stop_music —
            # трек продолжал играть. Останавливаем сами: stop идемпотентен,
            # ретрай тут дороже и ненадёжнее. mcp_server по music_cleanup
            # гасит и Renardo-паттерны, и mp3 в sound_node.
            self.get_logger().warning(
                "🎵 [issue 992 Bug F] стоп-команда без stop-тула — "
                "публикую music_cleanup сам"
            )
            try:
                self._publish_music_cleanup(reason="stop_command_guard")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(
                    f"🎵 force-stop failed: {exc}"
                )
            return False

        if verdict.kind is MusicGuardVerdictKind.NUDGE:
            # Issue #992 — the exhausted retry's assistant reply is the
            # same kind of unlabeled fake confirmation as above; retract
            # it too, otherwise it sits in history right next to the
            # honest "растерялся" (which is spoken via ``_speak_direct``
            # and never persisted) as if it were the real answer.
            self._discard_last_music_reply()
            self._speak_direct(
                "Я тут растерялся — бит не запустился, попробуй ещё раз."
            )
            return False

        # SKIP_NOT_APPLICABLE — guard deliberately skipped (stop-command,
        # user did not request music, DJ off, etc.). Policy module already
        # logged the diagnostic.
        return False

    def _build_music_retry_prompt(self, user_input: str) -> str:
        """Synthetic prompt for Bug C retry (user asked for music, LLM skipped
        execute_music_code).

        The LLM frequently concludes «музыка уже играет» from the dialogue
        history (previous runs/songs) and returns ``done`` without calling
        ``execute_music_code``. This prompt explicitly resets that assumption
        and demands the tool call.

        Delegates to
        :func:`rob_box_voice.core.dialogue_guards.build_music_retry_prompt`
        (TD-1 decomposition), прокидывая ЖИВОЕ состояние плеера: с тех пор
        как TRACK-музыка переживает чужой ход, «музыка не играет» в промпте
        стало ложью, и на просьбу ИЗМЕНИТЬ играющее модель отвечала «окей,
        играет X» без вызова тула (e2e renardo_evolve rn03).
        """
        return build_music_retry_prompt(
            user_input, music_playing=getattr(self, "_track_mode_music_active", False)
        )

    def _build_dj_retry_prompt(self) -> str:
        """Synthetic auto-prompt for the Bug-B synchronous retry.

        Re-uses the persona/theme context from
        :class:`DJModeController` but appends a CRITICAL reminder —
        the LLM has ignored the standard auto-prompt at least once,
        so this retry escalates the instruction with an explicit tool
        name and a no-tools rejection clause.
        """
        n = self._dj.state.transition_count
        base = self._dj.build_auto_prompt(n)
        return (
            base
            + "\n\n[CRITICAL] В прошлом цикле ты НЕ вызвал "
            "compose_music — DJ-режим остался без музыки. "
            "В этом цикле ОБЯЗАТЕЛЬНО вызови compose_music. "
            "НЕ вызывай speak_text и другие тулы — "
            "только музыку. Если ты снова не вызовешь "
            "compose_music, цикл будет считаться пустым и "
            "робот озвучит 'задумался'."
        )

    # ── Issue #1777 / #1762 — non-music tool-skipped guard ─────────────

    def _apply_tool_skipped_guard(
        self,
        *,
        user_input: str,
        tools_called: tuple,
        other_retry_dispatched: bool = False,
    ) -> bool:
        """Issue #1777 / #1762 — Bug C retry для non-music tool-based запросов.

        Если юзер явно попросил конкретный tool (``get_current_time`` /
        ``search_web`` / ``set_voice`` / ``memory_search`` /
        ``faq_search``), а LLM вернул ``tools=[]`` (ответил текстом-
        обещанием или просто не вызвал инструмент), отправляем ОДИН
        CRITICAL retry с явным указанием нужного tool.

        Не путать с :meth:`_apply_music_guard` (только music, см. issue
        #992 Bug C) и :meth:`_check_babble_and_retry` (мета-обещания).
        Здесь — конкретный tool-based пропуск.

        Retry rules (все должны выполниться):
        1. ``tools_called`` пустой (LLM не вызвал tool).
        2. ``user_input`` матчит keyword-set в
           :data:`TOOL_REQUEST_PATTERNS` (см.
           :func:`detect_required_tool`).
        3. ``_tool_retry_used`` ещё не взведён (защита от ping-pong).
        4. Уже не было music/babble/action-claim/code retry для этого
           turn (чтобы не конкурировать с другими guards и не отправить
           ДВА синтетических ретрая за один ход — см. Bug B/C
           double-dispatch incident, live 30.08 e2e renardo_evolve rn02,
           разобранный в :meth:`_apply_music_guard`).

        Args:
            other_retry_dispatched: ``True`` когда :meth:`_apply_music_guard`
                (вызывается непосредственно перед этим guard'ом в
                ``_run_turn.finally``) уже задиспатчил свой ретрай в этом
                ходе. Музыкальный гуард не выставляет
                ``_retry_dispatched_in_turn`` сам (историческая причина:
                он проверяется по return value, а не по общему флагу),
                поэтому caller обязан передать это явно — раньше здесь
                стояла эвристика по DJ-таймеру (``next_transition_at``),
                которая не покрывала Bug E (``_action_claim_retry_used``)
                и могла молча разойтись с реальным состоянием.

        Returns:
            ``True`` когда retry диспатчен (caller должен отложить
            ``DIALOGUE_END``). ``False`` иначе.
        """
        if tools_called:
            return False
        if not user_input:
            return False
        if self._tool_retry_used:
            return False
        if other_retry_dispatched or self._retry_dispatched_in_turn:
            return False
        tool_name = detect_required_tool(user_input)
        if not tool_name:
            return False
        retry_prompt = build_tool_retry_prompt(user_input, tool_name)
        if not retry_prompt:
            # Defence-in-depth: build_tool_retry_prompt вернул "" —
            # tool_name не из allow-list (промпт-инъекция?). Не ретраим.
            return False
        # DSM reopen — нужен DIALOGUE state для retry-тура (см. issue #1204).
        self._reopen_dialogue_for_retry()
        # Mark budget BEFORE dispatch — защита от re-entrant эскалации.
        self._tool_retry_used = True
        self.get_logger().warning(
            f"🛠 [issue 1777 / 1762] LLM skip non-music tool {tool_name!r} — "
            f"retrying once with CRITICAL reminder (user={user_input[:60]!r})"
        )
        self._dispatch_turn(
            retry_prompt,
            was_idle=False,
            raw_user_command=user_input,
        )
        return True

    # ═══════════════════════════════════════════════════════════════════════
    #  Agent loop methods (unit-test contracts — test_agent_loop.py)
    # ═══════════════════════════════════════════════════════════════════════

    #: Maximum tool-call iterations before forced stop (agent loop guard).
    MAX_ITERATIONS: int = 30

    def _continue_after_tool_calls(
        self,
        messages: list,
        tool_calls: list,
        tool_results: list,
    ) -> None:
        """Continue the agent loop after tool results are available.

        Implements the recursive LLM-call loop: executes the requested
        tools, feeds their results back to the LLM, and repeats until a
        final text response is produced or ``MAX_ITERATIONS`` is reached.
        Honours ``interrupt_agent_loop`` for early exit and ``listen_for_response``
        for waiting on the user.
        """
        # 1. Honour explicit interrupt.
        if getattr(self, "interrupt_agent_loop", False):
            self.interrupt_agent_loop = False
            self.llm_processing = False
            self.dialogue_in_progress = False
            return

        # 2. Listen-for-response short-circuit: leave the loop, wait for the user.
        for tr in (tool_results or []):
            if tr.get("tool_name") == "listen_for_response":
                # The test contract (test_listen_for_response_stops_loop) asserts
                # that ``_listen_response_waiting`` is False immediately after
                # this method returns. We therefore do NOT set the flag here —
                # it's only used as a transient guard inside the agent loop.
                self.llm_processing = False
                # dialogue_in_progress stays True — wait for user reply.
                return

        max_iter = int(getattr(self.__class__, "MAX_ITERATIONS", 30))
        current_tool_calls = list(tool_calls or [])
        current_tool_results = list(tool_results or [])
        current_messages = list(messages or [])
        iteration = 0
        last_response_text = ""

        while iteration < max_iter:
            iteration += 1

            # Execute any pending tool calls.
            if current_tool_calls:
                # Pass both ``tool_calls`` and ``messages`` positionally.
                # Why positional: ``test_max_iterations_exceeded`` injects
                # ``node._execute_tool_calls = lambda tc, msgs: [...]``
                # (a 2-positional mock) and calls into this code path.
                # The real :meth:`_execute_tool_calls` keeps
                # ``messages=None`` as a kwarg so direct test calls
                # like ``node._execute_tool_calls(tc, messages=[])``
                # (TestExecuteToolCalls) still work.
                new_results = self._execute_tool_calls(
                    current_tool_calls, current_messages
                )
                current_tool_results.extend(new_results)

            # Stream the next LLM turn with the tool results in context.
            result_dict: dict = {
                "full_response": "",
                "chunk_count": 0,
                "tool_calls": None,
                "error": None,
            }

            timeout_s = float(getattr(self, "_llm_timeout_sec", 90.0))
            # Retry-once for transient LLM errors (timeout / 5xx). Test
            # contract (``test_timeout_then_success_on_retry``): the first
            # attempt may fail with ``error='timeout'``; the second attempt
            # mutates ``result_dict`` to a clean success payload. We
            # therefore retry as long as the producer left an error AND
            # we still have attempts left, and stop on the first success.
            #
            # The synthesised ``_streaming_wrapper`` closure binds
            # ``result_dict`` / ``current_messages`` / ``current_tool_results``
            # as **kwargs defaults** so the test mock
            # (``test_agent_loop.py``'s FakeExecutor) can read
            # ``fn.__defaults__[0]`` to inject deterministic chunks into
            # ``result_dict`` without ever running the real
            # ``_do_recursive_streaming`` body. This is the only way the
            # structural contract of
            # ``test_plain_text_saved_to_history`` works.
            #
            # We also stash the result dict on self so that the test
            # fixtures which build a bare ``_Stub`` (no real
            # ``_do_recursive_streaming``) can still find it without
            # chasing the closure.
            self._current_streaming_result = result_dict
            self._current_streaming_messages = current_messages
            self._current_streaming_tool_results = current_tool_results
            def _streaming_wrapper(
                _result=result_dict,
                _messages=current_messages,
                _tool_results=current_tool_results,
            ):
                self._do_recursive_streaming(_result, _messages, _tool_results)
            max_attempts = 2
            for _attempt in range(max_attempts):
                # NOTE: we deliberately do NOT use ``with ThreadPoolExecutor(...)``
                # here. The unit-test harness (``test_agent_loop.py``) replaces
                # ``ThreadPoolExecutor`` with a ``FakeExecutor`` that implements
                # ``submit``/``shutdown`` but NOT the context-manager protocol
                # (``__enter__``/``__exit__``). Using ``with`` would raise
                # ``AttributeError: __enter__`` and the retry/result hand-off
                # would never run. We manage the executor lifecycle manually.
                executor = ThreadPoolExecutor(max_workers=1)
                try:
                    future = executor.submit(_streaming_wrapper)
                    future.result(timeout=timeout_s)
                except concurrent.futures.TimeoutError:
                    result_dict["error"] = "timeout"
                except Exception as exc:  # noqa: BLE001
                    result_dict["error"] = str(exc)
                finally:
                    executor.shutdown(wait=False)
                # Stop retrying on first success.
                if not result_dict.get("error"):
                    break

            if result_dict.get("error"):
                self._speak_simple(
                    "Извините, произошла ошибка при обращении к сервису. Попробуйте ещё раз."
                )
                break

            # LLM produced more tool_calls → execute them and loop again.
            next_calls = result_dict.get("tool_calls")
            if next_calls:
                current_tool_calls = next_calls
                current_tool_results = []
                continue

            last_response_text = result_dict.get("full_response", "")
            chunk_count = result_dict.get("chunk_count", 0)

            # Persist the assistant turn to conversation history.
            if last_response_text:
                # ``ConversationHistory`` exposes ``add_assistant_message``
                # (not a generic ``add_message``); use the typed helper
                # so the test ``test_plain_text_saved_to_history`` can find
                # the entry via ``get_messages()``.
                add_assistant = getattr(
                    self.conversation_history, "add_assistant_message", None
                )
                if add_assistant is not None:
                    add_assistant(last_response_text)
                else:
                    # Fallback to a generic attribute (older harnesses).
                    self.conversation_history.add_message("assistant", last_response_text)

            # Streamed chunks already published via SSML; if zero chunks fell
            # through we publish a simple one-shot response.
            if chunk_count == 0 and last_response_text:
                self._speak_simple(last_response_text)
            break

        if iteration >= max_iter:
            self._speak_simple(
                "Извините, возникла проблема с обработкой запроса. Попробуйте ещё раз."
            )

        # Cleanup flags in a finally-style block.
        self.llm_processing = False
        if not getattr(self, "_listen_response_waiting", False):
            self.dialogue_in_progress = False
        self._listen_response_waiting = False

    def _do_recursive_streaming(
        self,
        _result: dict = None,
        _messages: list = None,
        _tool_results: list = None,
    ) -> None:
        """Streaming call to the LLM, mutating ``_result`` in-place.

        Called inside a ``ThreadPoolExecutor`` by ``_continue_after_tool_calls``
        so the test mock can inject deterministic chunks via
        ``fn.__defaults__[0]`` to inspect the ``_result`` dict.

        Default values are required (not just ``None``) because the
        test contract (``test_agent_loop.py``'s FakeExecutor) reads
        ``fn.__defaults__[0]`` to mutate the result dict in-place. If
        the param has no default, ``__defaults__`` is an empty tuple
        and the mock gets no chance to inject text into ``_result``.
        We therefore declare ``_result=None`` as a default so the mock
        route is reachable; production code always passes ``_result``
        as a kwarg explicitly.
        """
        if _result is None:
            _result = {}
        client = getattr(self, "client", None)
        if client is None:
            _result["error"] = "no LLM client configured"
            return

        msgs = list(_messages or [])
        for tr in (_tool_results or []):
            msgs.append({
                "role": "tool",
                "tool_call_id": tr.get("tool_call_id", ""),
                "content": json.dumps(tr, ensure_ascii=False),
            })

        try:
            stream = client.chat.completions.create(
                model=getattr(self, "model", "default"),
                messages=msgs,
                temperature=getattr(self, "temperature", 0.7),
                max_tokens=getattr(self, "max_tokens", 500),
                stream=True,
            )
        except Exception as exc:  # noqa: BLE001
            _result["error"] = str(exc)
            return

        full_text = ""
        chunk_count = 0
        tool_calls_acc: dict = {}

        try:
            for chunk in stream:
                choices = getattr(chunk, "choices", None) or []
                if not choices:
                    continue
                delta = choices[0].delta
                if delta is None:
                    continue
                content = getattr(delta, "content", None)
                if content:
                    full_text += content
                    chunk_count += 1
                tcs = getattr(delta, "tool_calls", None)
                if tcs:
                    for tc in tcs:
                        idx = getattr(tc, "index", 0)
                        slot = tool_calls_acc.setdefault(idx, {
                            "id": getattr(tc, "id", "") or "",
                            "type": "function",
                            "function": {"name": "", "arguments": ""},
                        })
                        if getattr(tc, "id", None):
                            slot["id"] = tc.id
                        fn = getattr(tc, "function", None)
                        if fn is not None:
                            name = getattr(fn, "name", "")
                            args = getattr(fn, "arguments", "")
                            if name:
                                slot["function"]["name"] += name
                            if args:
                                slot["function"]["arguments"] += args
                if choices[0].finish_reason == "tool_calls":
                    _result["tool_calls"] = list(tool_calls_acc.values())
        except Exception as exc:  # noqa: BLE001
            _result["error"] = str(exc)
            return

        _result["full_response"] = full_text
        _result["chunk_count"] = chunk_count
        if not _result.get("tool_calls"):
            _result["tool_calls"] = None

    def _execute_tool_calls(self, tool_calls: list, messages: list = None) -> list:
        """Execute a batch of MCP tool calls.

        Returns a list of result dicts with keys ``tool_call_id``,
        ``tool_name``, ``success``, ``message`` (and ``error`` on failure).

        Test contract: ``messages`` is **optional** — tests pass it as
        a kwarg (``_execute_tool_calls(tool_calls, messages=[])``) when
        exercising the full loop, and a positional ``messages`` arg
        (``lambda tc, msgs: [...]``) when substituting a mock. The
        parameter is currently captured for forward-compatibility (no
        current logic depends on it).

        Hard caps: only the first ``MAX_TOOL_CALLS`` calls are dispatched
        and the loop aborts after ``MAX_CONSECUTIVE_ERRORS`` failures.
        """
        # Silence the "unused argument" lint while keeping the test-
        # compatible signature.
        del messages
        MAX_TOOL_CALLS = 5
        MAX_CONSECUTIVE_ERRORS = 3

        results: list = []
        consecutive_errors = 0
        truncated = list(tool_calls or [])[:MAX_TOOL_CALLS]

        for tc in truncated:
            tc_id = tc.get("id", "") if isinstance(tc, dict) else ""
            func_info = tc.get("function", {}) if isinstance(tc, dict) else {}
            func_name = func_info.get("name", "unknown") if isinstance(func_info, dict) else "unknown"
            func_args_str = func_info.get("arguments", "{}") if isinstance(func_info, dict) else "{}"

            try:
                args = json.loads(func_args_str)
            except (json.JSONDecodeError, TypeError):
                results.append({
                    "tool_call_id": tc_id,
                    "tool_name": func_name,
                    "success": False,
                    "error": "Ошибка формата аргументов",
                })
                consecutive_errors += 1
                if consecutive_errors >= MAX_CONSECUTIVE_ERRORS:
                    break
                continue

            adapter = getattr(self, "mcp_adapter", None)
            if adapter is None or not hasattr(adapter, "execute_tool_call_sync"):
                results.append({
                    "tool_call_id": tc_id,
                    "tool_name": func_name,
                    "success": False,
                    "error": "no MCP adapter available",
                })
                consecutive_errors += 1
                if consecutive_errors >= MAX_CONSECUTIVE_ERRORS:
                    break
                continue

            try:
                result = adapter.execute_tool_call_sync(func_name, args)
                success = bool(result.get("success", False)) if isinstance(result, dict) else False
                results.append({
                    "tool_call_id": tc_id,
                    "tool_name": func_name,
                    "success": success,
                    "message": result.get("message", "") if isinstance(result, dict) else "",
                    "data": result.get("data") if isinstance(result, dict) else None,
                })
                if not success:
                    consecutive_errors += 1
                else:
                    consecutive_errors = 0
            except Exception as exc:  # noqa: BLE001
                results.append({
                    "tool_call_id": tc_id,
                    "tool_name": func_name,
                    "success": False,
                    "error": str(exc),
                })
                consecutive_errors += 1

            if consecutive_errors >= MAX_CONSECUTIVE_ERRORS:
                break

        return results

    def _handle_result(
        self,
        result: DialogResult,
        user_input: Optional[str] = None,
        is_dj_auto: bool = False,
        raw_user_command: Optional[str] = None,
    ) -> None:
        """Publish (or swallow) the LLM response for one turn.

        ``user_input`` is threaded through from :meth:`_run_turn` so
        :meth:`_check_babble_and_retry` can correlate the LLM reply
        with the user request when deciding whether the response
        reads as metalanguage / babble (issue #992 Bug D). The
        parameter is optional for backwards compatibility with any
        external caller that bypasses :meth:`_run_turn`; when
        ``None`` the babble-retry path falls back to "promise-only"
        detection which is still safe (no false positives on
        ordinary answers).

        ``raw_user_command`` (issue #1204) — оригинальная команда юзера.
        На синтетических ретрай-турах ``user_input`` содержит CRITICAL-
        промпт (со словом «диджея» внутри), который НЕЛЬЗЯ сканировать
        эвристиками стоп-слов / music-intent — иначе ложный
        «stop-command» убивает музыку и выключает DJ.

        ``is_dj_auto`` marks DJ auto-transition turns (was_dj_auto=True,
        generated by the DJ ticker every ~45s, not by the user). For such
        turns an empty LLM response must NOT be answered with a spoken
        «Принял.» — the user did not say anything, so a confirmation
        phrase is noise (13.08: robot said «Принял.» every DJ transition
        in silence).
        """
        if result.error is not None:
            # 🔴 FIX (live 12.08): безопасный лог ошибки — если логгер
            # упадёт, мы НЕ теряем fallback-ответ пользователю.
            try:
                self.get_logger().warning(f"⚠️ DialogCore error: {result.error}")
            except Exception:
                pass
            # 🔴 FIX (issue #1278): когда ВСЕ LLM-провайдеры недоступны
            # (health-aware-fallback: все провайдеры unavailable) — робот
            # должен сказать честную degraded-фразу («интернет недоступен,
            # базовые команды работают»), а НЕ «Принял.»/«задумался».
            # Раньше ProviderError тонул в empty-response fallback ниже и
            # юзер слышал «Принял.» вместо объяснения.
            # 🔴 FIX (13.08, DJ): на DJ auto-transition degraded-фразу НЕ
            # озвучиваем — юзер ничего не говорил, каждые ~45с это шум
            # в тишине (тот же принцип, что подавление «Принял.» ниже).
            if self._is_llm_unavailable_error(result.error) and not is_dj_auto:
                try:
                    fallback_text = self._generate_fallback_response(
                        raw_user_command or user_input or ""
                    )
                    self.get_logger().warning(
                        f"🔧 [issue 1278] все LLM-провайдеры недоступны — "
                        f"degraded-ответ: {fallback_text!r}"
                    )
                    self._publish_response(fallback_text, animation="neutral")
                except Exception as exc:  # noqa: BLE001
                    try:
                        self.get_logger().warning(
                            f"⚠️ degraded-fallback publish failed: {exc}"
                        )
                    except Exception:
                        pass
                return
        spoken = strip_history_marker(result.spoken_text or "")
        # 🔴 FIX (live 20.08 DJ): LLM скопировала входной маркер
        # ``[Spkr:<имя>]`` в свой ответ → служебный текст ``[CRITICAL]
        # ...`` озвучивался целиком (маркер спикера сбивал
        # startswith-проверку служебного текста ниже). Спикер-тег —
        # внутренний маршрутный маркер, в TTS он попадать не должен.
        spoken = strip_speaker_tag(spoken)
        # 🔴 FIX (live 16:02 «английская мысль на Бахе»): MiniMax M3
        # возвращает ``<think>...</think>`` в content перед реальным
        # ответом. Done-чекер ниже ловит «done», но если перед ним лежит
        # английский мысленный комментарий («Music started successfully.
        # Now return "done" — no speak_text...»), он весь уходит в TTS
        # как «озвучка ответа». Strip-блоков ДО done-чекера → в TTS идёт
        # либо пусто (маркер done → тишина), либо реальный финал.
        spoken = strip_thinking_blocks(spoken)
        # Issue #988 (code part): strip Markdown BEFORE chunking. Chunking
        # splits on punctuation, which can cut a paired "*...*" in half;
        # strip_markdown in tts_node only removes *paired* delimiters, so a
        # lone "*" survives and TTS reads it as «звёздочка». Sanitising here
        # (before split_into_chunks) fixes that. Applies to the auto-voice
        # path AND the speak_text path (which also passes through chunking).
        spoken = strip_markdown(spoken)
        # 🔴 FIX (live 09:35): LLM-маркер завершения «done» (и вариации)
        # НЕ должен озвучиваться буквально («дан»). Это происходит когда
        # LLM завершила цикл без speak_text (например повтор «сыграй баха»
        # после уже запущенной музыки → ответил просто done).
        # 🔴 FIX (issue #1564): LLM обучена завершать turn текстом
        # ``done`` (RULE «after LAST speak_text → plain text "done"» в
        # master_prompt_compact.txt), но иногда льёт его В ``spoken``
        # через ``\n\n`` после реального ответа:
        #   spoken="Говорю голосом надёжного мужчины.\n\ndone"
        # → TTS озвучивает «...дан» после каждой фразы. ``_done_marker``
        # ниже ловит только точное равенство, не убирает хвост. Делаем
        # strip хвостового done-marker'а ПЕРВЫМ — после этого equality
        # чек ниже остаётся единственным местом, где решается «пропустить
        # auto-TTS».
        spoken = strip_done_marker(spoken)
        _done_marker = spoken.strip().lower()
        if _done_marker in ("done", "task complete", "task_complete", "готово", "всё", "выполнено"):
            self.get_logger().info(
                f"🔇 LLM completion marker — skip auto-TTS: {spoken[:60]!r}"
            )
            spoken = ""
        tools_called = tuple(result.tools_called or ())
        # Issue #988 — anti-duplicate: when the LLM already called
        # ``speak_text`` during this cycle, the answer (song / poem /
        # phrase) was voiced directly by the MCP tool via
        # ``/voice/tts/request``. Auto-voicing ``result.spoken_text``
        # here would play the same content a second time (E2E v44:
        # песня читается дважды). Per the master-prompt contract the
        # final text after the LAST speak_text is expected to be "done"
        # (or empty), so we skip the TTS publish entirely and only log
        # the LLM output for debugging.
        #
        # Issue #1343 — the guard must test the REAL call, not the
        # name in ``tools_called``. deepseek sometimes emits
        # ``speak_text({})`` / ``speak_text({"text": ""})``: the name
        # lands in ``tools_called`` but the call is rejected by
        # validation and NOTHING is voiced (no ``/voice/tts/request``).
        # Skipping auto-TTS in that case leaves the user with the
        # accept sound and then silence. DialogCore now tracks
        # ``speak_text_real_count`` (calls with non-empty ``text``) and
        # we skip only when speech REALLY happened.
        speak_text_real = int(getattr(result, "speak_text_real_count", 0) or 0)
        # Issue #1708 — hallucinated-lyrics guard diagnostic. When the
        # LLM called BOTH a music tool AND ``speak_text`` in the same
        # cycle, DialogCore's heuristic may have suppressed the
        # ``speak_text`` call (replaced with a sentinel error). If it
        # did, ``speak_text_real_count`` was decremented to zero, but
        # ``tools_called`` still lists both names so operators can see
        # the suppression happened. Log a one-line diagnostic so the
        # live log makes the pattern obvious without grepping.
        # Mirrors dialog_core._MUSIC_LAUNCH_TOOLS — keep the two lists
        # in sync if a new music tool is added to the manifest.
        _music_tool_names = {
            "execute_music_code", "generate_music",
            "gen_play_from_library", "set_vibe_preset", "load_track",
        }
        _has_music_tool = any(
            name in _music_tool_names for name in tools_called
        )
        _has_speak_text = "speak_text" in tools_called
        if _has_music_tool and _has_speak_text:
            # If speak_text_real==0 BUT tools_called still contains
            # speak_text, the dialog_core guard dropped the call.
            # Otherwise (speak_text_real>0), BACKING mode ran normally
            # — only log at debug to avoid noise.
            if speak_text_real == 0:
                self.get_logger().warning(
                    "🎤 [issue 1708] execute_music_code + speak_text в "
                    "одном turn — speak_text ПОДАВЛЕН (hallucinated "
                    f"lyrics guard). tools={list(tools_called)!r} "
                    f"user_input={user_input!r}"
                )
            elif self._verbose_llm:
                self.get_logger().debug(
                    "🎤 [issue 1708] execute_music_code + speak_text "
                    "(backing mode, lyrics allowed) — both ran. "
                    f"tools={list(tools_called)!r}"
                )
        if speak_text_real > 0:
            if self._verbose_llm:
                self.get_logger().info(
                    f"🔇 [issue 988] speak_text called in cycle — "
                    f"skipping auto-TTS of final text: {spoken[:200]!r}"
                )
            else:
                self.get_logger().info(
                    f"🔇 [issue 988] speak_text called — final text skipped "
                    f"(anti-duplicate): {spoken[:80]!r}"
                )
            return
        # 🔴 FIX (live 02.09): «во время сочинения музыки LLM много говорит».
        # Промпт среднего DJ-перехода запрещает speak_text, но НЕ запрещал
        # обычный текст ответа — а он тоже уходит в TTS. Живой лог: каждые
        # 45 секунд поверх бита звучало «Переход номер два отыгран —
        # нарастание с дропом в ре миноре фригийском, сто сорок ударов!».
        # Юзер про такие переходы ничего не спрашивал: это тик таймера.
        # Представление диджея (#1) и прощание (финальный трек) — говорят,
        # см. DJModeController.is_music_only_transition.
        if (
            spoken
            and is_dj_auto
            and self._dj.is_music_only_transition(self._dj.state.transition_count)
        ):
            self.get_logger().info(
                "🔇 [DJ] музыкальный переход #"
                f"{self._dj.state.transition_count} — свободный текст НЕ "
                f"озвучиваю (юзер ничего не спрашивал): {spoken[:120]!r}"
            )
            return
        # Issue #992 Bug D — metalanguage / babble detector. Fires ONE
        # synchronous retry with a CRITICAL prompt reminder when the
        # LLM replied with meta-talk instead of performing the request.
        # Returns ``True`` when a retry was scheduled; in that case we
        # MUST NOT publish the meta-text to TTS — otherwise the user
        # would hear the babble AND then the retry answer.
        if spoken and self._check_babble_and_retry(
            spoken=spoken,
            # Issue #1204: на синтетических ретрай-турах юзер-интент
            # смотрим по оригинальной команде, а не по CRITICAL-промпту.
            user_input=raw_user_command or user_input,
            tools_called=tools_called,
            speak_text_real=speak_text_real,
        ):
            return
        # Issue #992 Bug C' — LLM написала сочинённый Renardo-код в реплику
        # вместо execute_music_code(code=...). Код НЕ читаем вслух —
        # требуем вызов тула.
        if spoken and self._check_embedded_renardo_code_and_retry(
            spoken=spoken,
            user_input=raw_user_command or user_input,
            tools_called=tools_called,
        ):
            return
        # Issue #992 Bug E — «отчитался о действии, но не вызвал тул».
        # Live 30.08: «Точка сохранена.» / «Точка удалена.» / ««Тисбит»
        # удалён из медиатеки.» — всё с tools=[]. Один ретрай, тем же
        # контрактом, что и Bug D выше.
        if spoken and self._check_unbacked_action_claim_and_retry(
            spoken=spoken,
            user_input=raw_user_command or user_input,
            tools_called=tools_called,
        ):
            return
        # 💡 Diagnostic: log the actual state before deciding what to do.
        # Helps answer "why did the robot stay silent?" without guesswork.
        self.get_logger().info(
            f"🔍 [handle_result] spoken={spoken!r} (len={len(spoken)}) "
            f"tools={list(tools_called)!r} user_input={user_input!r}"
        )
        if not spoken:
            # 🔴 FIX (live 10:49): пустой spoken НЕ всегда ошибка — LLM
            # могла выполнить TRACK-запрос («сыграй баха») через
            # execute_music_code и вернуть 'done' без speak_text (новый
            # TRACK-контракт: музыка без болтовни). Мой done-strip
            # обнулил spoken → «задумался» был ЛОЖНЫМ. Fallback — только
            # если LLM вообще ничего не сделала (tools пуст).
            #
            # 🔴 FIX (live 16:58 «всегда задумался»): НЕ падаем в fallback
            # «задумался» — логируем структурированную диагностику
            # (finish_reason + raw) и пишем reminder в память, чтобы
            # СЛЕДУЮЩИЙ turn LLM увидела, что прислала пустоту и так
            # делать не надо. Юзер не получает ложного «задумался».
            if not tools_called:
                # Issue #1204: на синтетических ретрай-турах ``user_input`` —
                # это CRITICAL-промпт («...музыку/диджея...»), а не слова
                # юзера. Все эвристики ниже (music_fallback, стоп-команды)
                # должны смотреть на оригинальную команду юзера.
                _source = raw_user_command or user_input
                # Issue #1016 — empty-response music fallback: LLM вернула
                # пустоту на музыкальный запрос («поставь что-нибудь»,
                # «сыграй классику», «включи музыку») и НЕ вызвала ни одного
                # тула. Просим mcp_server сыграть топ-трек из библиотеки
                # (rating DESC), чтобы юзер услышал музыку, а не тишину.
                # Эвристика шире _MUSIC_GUARD_KEYWORDS — это единственная
                # ветка, где цена ложного срабатывания низкая (робот и так
                # молчал бы).
                try:
                    if self._user_wants_music_fallback(_source or ""):
                        self._publish_music_fallback(reason="empty_response")
                except Exception as exc:  # noqa: BLE001
                    try:
                        self.get_logger().warning(
                            f"⚠️ music_fallback trigger failed: {exc}"
                        )
                    except Exception:
                        pass
                # 🔴 FIX (live 12.08): весь empty-response fallback
                # обёрнут в try/except — если любой внутренний вызов
                # (включая логгер!) упадёт, пользователь ВСЁ РАВНО
                # получит «Принял.». Без этого баг «LLM молчит» остаётся
                # незаметным для пользователя (тишина вместо ответа).
                try:
                    fr = getattr(result, "finish_reason", None)
                    raw = getattr(result, "raw_response", None)
                    # 🔴 FIX (live 06.08): стоп-команда («хватит диджеить»,
                    # «выключи музыку») + пустой ответ LLM → ВСЁ РАВНО
                    # останавливаем музыку/DJ. LLM иногда возвращает пустоту
                    # (DeepSeek empty), и без этого fallback музыка играет
                    # бесконечно (юзер: «сказал хорошо молчу, музло ебашит»).
                    # 🔴 FIX (issue #1204): проверяем _source (оригинальная
                    # команда юзера), а не user_input — на ретрай-турах это
                    # синтетический CRITICAL-промпт с «диджея» внутри.
                    if _source and is_music_stop_command(_source):
                        self.get_logger().warning(
                            "🎵 [issue 992 Bug C] stop-command + empty LLM "
                            "response — forcing music_cleanup + DJ off"
                        )
                        # Публикуем cleanup — mcp_server остановит музыку
                        # (MusicManager.stop_music_on_session_end).
                        try:
                            self._publish_music_cleanup(reason="user_stop_command")
                        except Exception as exc:  # noqa: BLE001
                            self.get_logger().warning(
                                f"🎵 stop fallback failed: {exc}"
                            )
                        # 🔴 FIX (live 06.08 #2): cleanup гасит ТОЛЬКО музыку,
                        # но DJ-тикер (core/dj_mode, tick каждые 5с) живёт по
                        # флагу state.enabled — его сбрасывает только
                        # публикация /voice/dj_mode с enabled=false. Без этого
                        # DJ продолжал генерить переходы (#5, #6...) и после
                        # «говори» снова включал музыку («продолжил диджейский
                        # сет»). Публикуем set_dj_mode(enabled=false) сами.
                        try:
                            dj_msg = String()
                            dj_msg.data = json.dumps({"enabled": False})
                            if getattr(self, "_dj_mode_pub", None) is None:
                                self._dj_mode_pub = self.create_publisher(
                                    String, "/voice/dj_mode", 10
                                )
                            self._dj_mode_pub.publish(dj_msg)
                            self.get_logger().info(
                                "🎵 DJ off published (stop-command fallback)"
                            )
                        except Exception as exc:  # noqa: BLE001
                            self.get_logger().warning(
                                f"🎵 DJ off publish failed: {exc}"
                            )
                    # Короткая диагностика: что именно вернул провайдер.
                    raw_hint = ""
                    if raw is not None:
                        try:
                            raw_hint = str(raw)[:300]
                        except Exception:  # noqa: BLE001
                            raw_hint = "<unprintable raw>"
                    self.get_logger().warning(
                        "⚠️ Empty assistant response (LLM вернул пустоту): "
                        f"finish_reason={fr!r} tools={list(tools_called)!r} "
                        f"raw={raw_hint!r}"
                    )
                    # 🔴 FIX (issue #1217): НЕ пишем [SYSTEM REMINDER] /
                    # [silent_accept] в долгую память как assistant-реплики.
                    # Раньше каждый пустой ответ добавлял в history
                    # синтетические assistant-turn'ы («done» + REMINDER +
                    # silent_accept), и через несколько циклов LLM видела
                    # историю из десятков «done» — и МИМИКИРОВАЛА этот
                    # паттерн, возвращая «done» без tool-вызовов (memory DB
                    # показывала 15+ подряд assistant 'done'). Корректирующий
                    # retry теперь живёт ВНУТРИ DialogCore._run_with_tools
                    # (issue #1217) и учит модель в том же turn; служебные
                    # записи в диалоговую память не нужны (правило control
                    # traffic — как для DJ-auto в dialog_core.process_input).
                except Exception as _empty_fallback_exc:
                    # 🔴 КРИТИЧНО: даже если ВСЁ внутри блока упало
                    # (логгер, память, etc.), пользователь НЕ должен
                    # остаться в тишине. Публикуем fallback-ответ
                    # напрямую, минуя всю сложную логику.
                    try:
                        self.get_logger().error(
                            f"🔥 Empty-response fallback CRASHED: {_empty_fallback_exc}"
                        )
                    except Exception:
                        pass
                # Tell the user that we heard them so they are not left
                # with an «accept + silence» experience.
                # 🔴 FIX (live 12.08): этот вызов ВНЕ внутреннего try/except
                # и выполняется ВСЕГДА — даже если логгер или память упали.
                # 🔴 FIX (13.08): для DJ auto-transition пустой ответ НЕ
                # подтверждаем голосом — юзер ничего не говорил, «Принял.»
                # каждые ~45с (каждый переход DJ-тикера) — шум в тишине.
                if not is_dj_auto:
                    try:
                        self._publish_response("Принял.", animation="neutral")
                    except Exception as exc:  # noqa: BLE001
                        try:
                            self.get_logger().warning(
                                f"⚠️ empty-fallback publish failed: {exc}"
                            )
                        except Exception:
                            pass
                else:
                    try:
                        self.get_logger().info(
                            "🔇 DJ auto-transition: пустой ответ LLM — "
                            "«Принял.» подавлен (юзер ничего не говорил)"
                        )
                    except Exception:
                        pass
                return
            else:
                # Issue #1101 (live 11.08) — when LLM called tools
                # (e.g. stop_music) but returned no spoken text,
                # the user hears silence with no explanation. Log
                # exactly what happened so operators can diagnose.
                tc_list = list(tools_called)
                user_hint = (user_input or "")[:80]
                self.get_logger().info(
                    "🔇 TRACK-запрос выполнен тулами, spoken пуст — "
                    f"тихо завершаю (tools={tc_list!r} "
                    f"user={user_hint!r})"
                )
                # When the only action was stop_music (no speak_text,
                # no music_code), the user gets pure silence after a
                # request — surface a louder diagnostic.
                if tc_list == ["stop_music"]:
                    self.get_logger().warning(
                        "🎵 [diagnostics] LLM called ONLY stop_music "
                        "with no spoken text — user heard silence "
                        f"(user_input={user_hint!r}). "
                        "LLM must follow up with speak_text or "
                        "execute_music_code when stop_music is used."
                    )
                return
        # Issue #980 — split into chunks and publish as a single TTS batch so
        # that /voice/tts/batch_complete fires only after the last chunk.
        # 🔴 FIX (live 06.08): LLM копирует [SYSTEM REMINDER] из памяти как
        # свой ответ (Bug C guard пишет reminder как assistant-turn, LLM его
        # повторяет) → озвучивался служебный текст. Служебные маркеры НЕ
        # озвучиваем — тихо завершаем цикл (LLM уже получил контекст).
        _spoken_stripped = spoken.strip()
        # 🔴 FIX (live 15:55 06.08): LLM копирует формат служебных вставок
        # ([SYSTEM REMINDER], [CRITICAL: ...]) и генерит СВОИ «[CRITICAL:
        # ROOTSYSTEMPOLICY violation...» — озвучивалось как «я тут растерялся».
        # Любой маркер в стиле [ИМЯ: ...] (заглавные, двоеточие, скобки) —
        # служебный, НЕ озвучиваем.
        if _spoken_stripped.startswith(("[SYSTEM", "[СИСТЕМ", "[REMINDER", "[CRITICAL", "[ПОЛИТИКА", "[ROOT")):
            self.get_logger().warning(
                f"🔇 Служебный текст LLM не озвучиваем: {_spoken_stripped[:100]!r}"
            )
            return
        chunks = split_into_chunks(spoken)
        if not chunks:
            self._publish_response(spoken)
        elif len(chunks) == 1:
            self._publish_response(chunks[0])
        else:
            total = self._publish_response_batch(chunks)
            self.get_logger().info(
                f"📦 [dialogue_node] TTS batch: {total} chunks (issue #980)"
            )
        self.get_logger().info(
            f"📤 LLM OUTPUT: {spoken[:200]!r}" if self._verbose_llm
            else f"✅ Turn done. Response: {spoken[:80]!r}")
    def _publish_state(self) -> None:
        msg = String()
        msg.data = self._dsm.current_state.name
        self._state_pub.publish(msg)
    def _on_startup_greeting(self) -> None:
        """Одноразовое приветствие при старте (issue #1003, редизайн 06.08).

        Раньше была отдельная startup_greeting_node — она публиковала
        приветствие до того, как tts_node завершал инициализацию, и
        фраза терялась. Теперь dialogue_node сам ждёт N секунд и шлёт
        текст тем же путём, что обычная речь (build_ssml_payload →
        /voice/dialogue/response → tts_node).

        Последовательность: [thinking] → пауза → [cute/very_cute] →
        случайная фраза из GREETINGS (или явный startup_greeting_text).
        Если юзер уже говорит (состояние != IDLE) — приветствие
        пропускаем, чтобы не перебивать диалог (acceptance #1003).
        """
        if self._startup_greeting_fired:
            return
        self._startup_greeting_fired = True

        # Не перебиваем активный диалог / DJ-режим: приветствие —
        # только когда система в IDLE (юзер ещё не начал говорить).
        if self._dsm.current_state != DialogueStateKind.IDLE:
            self.get_logger().info(
                "Startup greeting: диалог активен — пропускаю приветствие"
            )
            return

        # Thinking-звук, как при обычном диалоге.
        sfx = String()
        sfx.data = THINKING_SOUND
        self._sound_trigger_pub.publish(sfx)

        # Через 2с — радостный звук, ещё через 1.5с — фраза (как в
        # оригинальной startup_greeting_node до рефакторинга).
        self._greeting_timer = self.create_timer(
            2.0, self._on_startup_greeting_finish
        )

    def _on_startup_greeting_finish(self) -> None:
        """Вторая фаза приветствия: радостный звук cute/very_cute."""
        self._cancel_greeting_timer()
        sfx = String()
        sfx.data = pick_finish_sound()
        self._sound_trigger_pub.publish(sfx)
        self._greeting_timer = self.create_timer(
            1.5, self._on_startup_greeting_speak
        )

    def _on_startup_greeting_speak(self) -> None:
        """Третья фаза: публикуем случайную фразу приветствия."""
        self._cancel_greeting_timer()
        phrase = pick_greeting(self._startup_greeting_text)
        self.get_logger().info(f"🗣 Startup greeting: {phrase!r}")
        self._publish_response(phrase)

    def _cancel_greeting_timer(self) -> None:
        """Отменить одноразовый таймер приветствия, если он создан."""
        timer = getattr(self, "_greeting_timer", None)
        if timer is not None:
            try:
                timer.cancel()
            except Exception:  # noqa: BLE001 — таймер может быть уже сработавшим
                pass
            self._greeting_timer = None

    def _publish_response(self, text: str, animation: str = "neutral") -> None:
        """Single-chunk publish — kept for backwards compatibility.

        For multi-chunk turns prefer :meth:`_publish_response_batch` which
        attaches a shared ``batch_id`` so ``tts_node`` can fire
        ``/voice/tts/batch_complete`` once the last chunk lands (issue #980).

        Issue #1195 — если последний вход пришёл из Telegram-чата,
        кладём ``tg_chat_id`` в payload: telegram_node использует его для
        маршрутизации эхо-ответа в правильный чат.
        """
        msg = String()
        msg.data = build_ssml_payload(
            text,
            animation,
            tg_chat_id=self._active_tg_chat_id,
            voice=getattr(self, "_current_tts_voice", None),
        )
        self._response_pub.publish(msg)

    def _publish_response_batch(self, chunks: List[str], animation: str = "neutral") -> int:
        """Publish ``chunks`` as a single TTS batch (issue #980).

        Each chunk carries the same ``batch_id`` plus 1-based ``batch_index``
        and ``batch_total`` counters. ``tts_node`` echoes those fields on
        ``/voice/tts/finished`` and publishes ``/voice/tts/batch_complete``
        after the final chunk. Returns the number of chunks published.

        Issue #992: the batch is registered eagerly on the dialogue_node
        side so the music-cleanup gate knows about it even if the first
        ``/voice/tts/finished`` event is lost (e.g. dropped subscription
        edge case). The registry is idempotent so the matching
        ``_on_tts_finished`` registration is a safe no-op.
        """
        if not chunks:
            return 0
        batch_id = str(uuid.uuid4())
        total = len(chunks)
        self._register_active_batch(batch_id, total)
        for idx, chunk in enumerate(chunks, start=1):
            msg = String()
            msg.data = build_ssml_payload(
                chunk,
                animation,
                batch_id=batch_id,
                batch_index=idx,
                batch_total=total,
                tg_chat_id=self._active_tg_chat_id,
                voice=getattr(self, "_current_tts_voice", None),
            )
            self._response_pub.publish(msg)
        return total

    def _publish_music_cleanup(self, reason: str = "dialogue_end") -> None:
        """Issue #935 — signal mcp_server to auto-stop any active music.

        Best-effort: if the publisher was never created (e.g. mcp_server
        not running in this container), this is a silent no-op. mcp_server
        decides what to do — currently it logs and calls
        ``MusicManager.stop_music_on_session_end()``.
        """
        # Что бы ни было причиной — стоп, новый диалог, конец BACKING-хода —
        # после cleanup живой TRACK-музыки больше нет (live 30.08).
        self._track_mode_music_active = False
        if getattr(self, "_music_cleanup_pub", None) is None:
            self.get_logger().debug("music_cleanup publisher not available")
            return
        try:
            payload = json.dumps({"reason": reason})
            msg = String()
            msg.data = payload
            self._music_cleanup_pub.publish(msg)
            self.get_logger().info(f"music_cleanup sent: reason={reason}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ Не удалось опубликовать /mcp/music_cleanup: {exc}"
            )

    def _publish_music_fallback(self, reason: str = "empty_response") -> None:
        """Issue #1016 — ask mcp_server to play the top-rated library track.

        Used in the empty-response fallback: when the LLM returns no text
        AND no tool calls for a music request («поставь что-нибудь»,
        «сыграй классику»), the user should hear *something* — the best
        human track from the library — instead of silence.

        Best-effort: if the publisher was never created (mcp_server not
        running in this container), this is a silent no-op. mcp_server
        decides what to do — currently it plays the top-rated track from
        ``music_tracks`` (rating DESC).
        """
        if getattr(self, "_music_fallback_pub", None) is None:
            self.get_logger().debug("music_fallback publisher not available")
            return
        try:
            payload = json.dumps({"reason": reason})
            msg = String()
            msg.data = payload
            self._music_fallback_pub.publish(msg)
            self.get_logger().info(f"music_fallback sent: reason={reason}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ Не удалось опубликовать /mcp/music_fallback: {exc}"
            )
    def _speak_direct(self, text: str) -> None:
        for chunk in split_into_chunks(text):
            self._publish_response(chunk)

    # ═══════════════════════════════════════════════════════════════════════
    #  Pure helper methods (unit-test contracts for issue #968 / #980)
    #  ═══════════════════════════════════════════════════════════════════════
    #  TD-1 decomposition: implementations live in
    #  :mod:`rob_box_voice.core.dialogue_helpers`; class-level aliases keep
    #  ``node._EMOTION_TO_ANIMATION`` etc. working for existing callers/tests.

    _EMOTION_TO_ANIMATION: dict = EMOTION_TO_ANIMATION

    def _map_emotion_to_animation(self, emotion: str) -> str:
        """Map an emotion label to its corresponding LED animation key.

        Lookup is case-insensitive; unknown emotions default to ``"idle"``.
        Delegates to
        :func:`rob_box_voice.core.dialogue_helpers.map_emotion_to_animation`
        (TD-1 decomposition).
        """
        return map_emotion_to_animation(emotion)

    def _is_llm_unavailable_error(self, error: Any) -> bool:
        """Issue #1278 — is ``error`` an all-LLM-providers-unavailable failure?

        ``HealthAwareFallbackLLM`` raises ``ProviderError`` with the
        ``health-aware-fallback: все провайдеры unavailable`` marker when
        every provider in the chain is down (TTL not yet expired). The
        shell must distinguish this from ordinary errors so the robot
        says an honest degraded phrase instead of the generic
        «Что-то я задумался» / «Принял.».
        """
        if error is None:
            return False
        if isinstance(error, ProviderError):
            return True
        # ``DialogCore`` used to wrap LLM exceptions into a plain
        # ``Exception`` carrying the traceback text (4ba16f23), which threw
        # the ``ProviderError`` type away and left only this substring
        # match. It now keeps the exception itself (traceback goes to
        # ``DialogResult.error_traceback``), so the ``isinstance`` above is
        # the real check; the marker stays as a safety net for any provider
        # that reports the same condition without the type.
        try:
            msg = str(error)
        except Exception:  # noqa: BLE001 — never crash on a broken __str__
            return False
        return "health-aware-fallback: все провайдеры unavailable" in msg

    def _generate_fallback_response(self, text: str) -> str:
        """Generate a static fallback reply when the LLM is unavailable.

        Delegates to
        :func:`rob_box_voice.core.dialogue_helpers.generate_fallback_response`
        (TD-1 decomposition).
        """
        return generate_fallback_response(text)

    def _detect_volume_intent(self, text: str):
        """Detect volume adjustment intent from user text.

        Delegates to
        :func:`rob_box_voice.core.dialogue_helpers.detect_volume_intent`
        (TD-1 decomposition).
        """
        return detect_volume_intent(text)

    def _detect_pitch_intent(self, text: str):
        """Detect pitch adjustment intent from user text.

        Delegates to
        :func:`rob_box_voice.core.dialogue_helpers.detect_pitch_intent`
        (TD-1 decomposition).
        """
        return detect_pitch_intent(text)

    def _speak_simple(self, text: str, show_error_animation: bool = False) -> None:
        """Publish a one-off TTS payload via SSML JSON with a unique ``dialogue_id``."""
        dialogue_id = str(uuid.uuid4())
        self.current_dialogue_id = dialogue_id
        payload = json.dumps({
            "ssml": f"<speak>{text}</speak>",
            "dialogue_id": dialogue_id,
        }, ensure_ascii=False)
        msg = String()
        msg.data = payload
        self.response_pub.publish(msg)
        if show_error_animation:
            anim_msg = String()
            anim_msg.data = "error"
            pub = getattr(self, "animation_pub", None)
            if pub is not None:
                pub.publish(anim_msg)

    def _trigger_sound(self, name: str) -> None:
        """Publish a one-shot sound trigger; swallows publish exceptions."""
        try:
            msg = String()
            msg.data = name
            pub = getattr(self, "sound_trigger_pub", None)
            if pub is not None:
                pub.publish(msg)
        except Exception:
            pass

    def _on_mcp_tools_update(self, msg) -> None:
        """Parse MCP tools JSON and update ``available_tools``.

        Issue #1409 — also keep ``self._mcp_tool_names`` (SSoT set used
        by ``_load_system_prompt`` validation) in sync. If /mcp/tools
        delivers a fresher catalogue than ``ToolRegistry.list_tools()``
        (e.g. an external MCP server registered new tools after
        startup), we want the next prompt reload to see them too.
        """
        try:
            data = getattr(msg, "data", "") or "[]"
            tools = json.loads(data)
            if isinstance(tools, list):
                self.available_tools = tools
                self.mcp_tools_available = True
                self._mcp_tool_names = {
                    str(t.get("function", {}).get("name", ""))
                    for t in tools
                    if isinstance(t, dict) and t.get("function", {}).get("name")
                }
            else:
                self.available_tools = []
                self.mcp_tools_available = False
        except (json.JSONDecodeError, TypeError) as exc:
            self.mcp_tools_available = False
            self.available_tools = []
            self.get_logger().error(f"MCP tools update parse error: {exc}")

    def _on_perception_update(self, msg) -> None:
        """Update ``internet_available`` and ``current_time_info`` from a perception msg."""
        if hasattr(msg, "internet_available"):
            self.internet_available = bool(msg.internet_available)
        if hasattr(msg, "time_context_json"):
            try:
                self.current_time_info = json.loads(msg.time_context_json or "{}")
            except (json.JSONDecodeError, TypeError):
                self.current_time_info = {}
                self.get_logger().warning("Invalid time_context_json in perception update")

    def vad_callback(self, msg) -> None:
        """VAD callback: track speech edges and signal agent interrupt.

        Tests use ``vad_callback`` (public) instead of the internal
        ``_on_vad``; delegate to the legacy method when present so behavior
        stays identical. The legacy handler updates both ``_vad_speech_detected``
        (private) and ``vad_speech_detected`` (public) so tests that read
        either attribute stay consistent.
        """
        legacy = getattr(self, "_on_vad", None)
        if legacy is not None:
            legacy(msg)
            return
        active = bool(getattr(msg, "data", False))
        if active and not getattr(self, "vad_speech_detected", False):
            self.vad_speech_detected = True
            self._vad_speech_detected = True
            if getattr(self, "llm_processing", False) and not getattr(self, "mcp_tools_available", False):
                self.interrupt_agent_loop = True
        elif not active and getattr(self, "vad_speech_detected", False):
            self.vad_speech_detected = False
            self._vad_speech_detected = False

    def _handle_silence(self) -> None:
        self._cancel_run("silence command", stop_tts=True)
        self._dsm.on_event(DialogueEvent.SILENCE_COMMAND)
        self._publish_state()
        self._speak_direct("Хорошо, молчу.")

    def _drain_pending_user_messages(self) -> bool:
        """S7 (scheduler-segments-merge, issue #968) — dispatch queued
        phrases as ONE follow-up turn.

        Phrases accumulate in ``self._pending_user_messages`` while a
        turn's LLM cycle was in flight (barge_in_policy=classify,
        quick_decide=PENDING_LLM verdict — see ``_on_stt``). Called from
        ``_run_turn``'s ``finally`` once ``self._run_task`` has been
        cleared, so the drained turn's own ``_run_turn`` re-entry finds
        a free slot. Multiple accumulated phrases are glued into ONE
        turn (newline-joined), never N — dispatching N follow-up turns
        would just recreate the concurrency problem S7 exists to avoid.

        Returns True when a follow-up turn was dispatched (the caller
        must then suppress this turn's own DIALOGUE_END — the drained
        turn is a continuation, not the end of the session).

        getattr-guarded like ``_speech_accumulator`` elsewhere in this
        class: this is called unconditionally from every ``_run_turn``,
        and plenty of existing unit tests build a ``DialogueNode`` via
        ``object.__new__`` (bypassing ``__init__``) without setting
        every optional attribute — a missing queue must mean "nothing
        to drain", not an ``AttributeError`` that breaks the turn.
        """
        queue = getattr(self, "_pending_user_messages", None)
        if not queue:
            return False
        queued = list(queue)
        queue.clear()
        texts = [text for text, _enqueued_at in queued]
        now = time.monotonic()
        oldest_ts = min(ts for _text, ts in queued)
        queue_latency_ms = (now - oldest_ts) * 1000
        combined = "\n".join(texts)
        self.get_logger().info(
            f"📤 [S7] draining {len(texts)} pending message(s), "
            f"queue_latency={queue_latency_ms:.0f}ms: {combined[:120]!r}"
        )
        # W2-6 (issue #968) — гистограмма queue-latency: по одному
        # наблюдению на КАЖДУЮ отложенную фразу (не только самую
        # старую) — так распределение отражает реальный разброс, а не
        # только worst-case первой фразы в пачке. Цель — ≤ 200мс
        # (см. docstring record_pending_queue_latency).
        if is_metrics_enabled():
            try:
                for _text, enqueued_at in queued:
                    record_pending_queue_latency(now - enqueued_at)
            except Exception as _metric_exc:  # noqa: BLE001
                self.get_logger().debug(
                    f"⚠️ [metrics] record_pending_queue_latency failed: "
                    f"{_metric_exc!r}"
                )
        self._dispatch_turn(combined, raw_user_command=combined)
        return True

    def _reset_dialogue_session(self) -> None:
        """Issue #XXXX — сброс текущей диалоговой сессии.

        Полный сброс состояния текущего диалога: in-flight turn, DSM → IDLE,
        бэклог-аккумулятор, speaker-состояние, таймер сессии и история
        (асинхронно через ``memory.clear_turns``). LLM не вызывается —
        вместо этого говорим детерминированное подтверждение.

        Issue #1563 — после ``_publish_response`` TTS должен успеть синтези-
        ровать и доиграть «Начинаю новую сессию…», даже если barge-in от
        wake-word в той же фразе успел прислать STOP в ``/voice/tts/control``
        ДО того, как этот код увидел STT-результат. Поэтому ДО отправки
        подтверждения публикуем ``IGNORE_STOP_MS:700`` — TTS игнорирует
        STOP-команды в этом окне и спокойно синтезирует/воспроизводит.
        """
        # 1. Отменяем in-flight turn (barge-in + stop TTS + release effects).
        self._cancel_run("new session reset", stop_tts=True)
        # 1a. Issue #1563 — открыть IMMUNE-окно для TTS, чтобы barge-in
        # STOP (пришедший в той же STT-фразе) не отменил подтверждение
        # «Начинаю новую сессию…». 700 мс — с запасом на синтез Yandex
        # (~300-500 мс) + ALSA-буфер + grace перед AEC-эхо.
        try:
            ignore_msg = String()
            ignore_msg.data = "IGNORE_STOP_MS:700"
            self._tts_control_pub.publish(ignore_msg)
        except Exception as exc:  # noqa: BLE001 — best-effort, не роняем reset
            self.get_logger().warn(
                f"⚠️ [issue 1563] IGNORE_STOP_MS publish failed: {exc}"
            )
        # 2. Бэклог-аккумулятор фоновой речи (если реализован).
        acc = getattr(self, "_speech_accumulator", None)
        if acc is not None:
            try:
                acc.clear()
            except Exception:  # noqa: BLE001
                pass
        self._pending_backlog_flush = False
        # 2a. S7 (scheduler-segments-merge) — очередь фраз, накопленных
        # пока предыдущий турн был в полёте, тоже принадлежит старой
        # сессии — сбрасываем вместе с бэклогом. getattr-guard — как и
        # для _speech_accumulator выше: тестовые фикстуры на
        # object.__new__(DialogueNode) не всегда проходят __init__.
        pending_queue = getattr(self, "_pending_user_messages", None)
        if pending_queue is not None:
            pending_queue.clear()
        # 3. DSM → IDLE из любого состояния (rescue path).
        try:
            self._dsm.reset(DialogueStateKind.IDLE)
        except Exception:  # noqa: BLE001
            pass
        # 4. Speaker-состояние сессии.
        with self._speaker_lock:
            self._current_speaker = {"is_known": False}
        self._speaker_by_text.clear()
        try:
            self._speaker_tracker.reset()
        except Exception:  # noqa: BLE001
            pass
        # 5. Метрика длительности сессии (и сброс таймера).
        try:
            self._maybe_record_session_end(result="reset")
        except Exception:  # noqa: BLE001
            pass
        # 6. История диалога (scope = DialogCore user_id "default") —
        #    асинхронно, потому что SQLiteVoiceMemory работает через loop.
        loop = getattr(self, "_loop", None)
        if loop is not None:
            try:
                asyncio.run_coroutine_threadsafe(
                    self._clear_session_turns(), loop
                )
            except Exception:  # noqa: BLE001
                pass
        # 7. Публикуем состояние и подтверждение.
        self._publish_state()
        self._publish_response(
            "Начинаю новую сессию. Всё, что было до этого, забыто.",
            animation="neutral",
        )

    async def _clear_session_turns(self) -> None:
        """Асинхронно очистить историю диалога текущей сессии."""
        try:
            removed = await self._memory.clear_turns("default")
            self.get_logger().info(
                f"🧹 [new-session] conversation history cleared ({removed} turns)"
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ [new-session] clear_turns failed: {exc}"
            )

    def _maybe_log_skip_summary(self, window_s: float = 300.0) -> None:
        """Issue #1101 — периодическая сводка по пропускам LLM.

        Раз в ``window_s`` секунд (по умолчанию 5 минут) печатает в лог
        одну строку ``📊 [diagnostics] llm_skipped ...``, чтобы оператор
        видел причины «робот молчит». Сводка включается только если есть
        хотя бы один пропуск — пустые окна не спамят.
        """
        now = time.monotonic()
        if now - self._last_skip_summary_ts < window_s:
            return
        summary = format_llm_skipped_summary(
            self._llm_skipped_counter, window_s=window_s
        )
        if summary is None:
            return
        self.get_logger().info(summary)
        self._last_skip_summary_ts = now
    def _cancel_run(self, reason: str, *, stop_tts: bool = True) -> None:
        """Cancel the in-flight LLM turn, optionally muting TTS.

        S1.2 (scheduler-segments-merge, R1) — cancelling the turn and
        muting TTS used to be one inseparable action. ``barge_in_policy=
        "classify"`` (S1.3) needs to cancel the turn WITHOUT stopping
        TTS, so a new user phrase doesn't cut off a segment mid-sentence.
        ``stop_tts=False`` must still release the TTS/sound awaiters —
        skipping that hangs ``speak_helpers._tts_events`` forever and the
        robot never speaks again.
        """
        self._run_cancelled = True
        with self._task_lock:
            task = self._run_task
        if task is not None and not task.done():
            self.get_logger().info(f"🛑 Cancel: {reason}")
            self._loop.call_soon_threadsafe(task.cancel)
        if stop_tts:
            stop_msg = String()
            stop_msg.data = "STOP"
            self._tts_control_pub.publish(stop_msg)
        self._effects.release_all_tts()
        self._effects.clear_sound_event()

    def _maybe_record_session_end(self, result: str = "success") -> None:
        """Issue #1160 — Prometheus metrics: закрыть открытую сессию.

        Пишет histogram ``voice_session_duration_seconds{result=...}``
        один раз на сессию (и сбрасывает таймер). Вызывается из
        DIALOGUE_END (result=success) и из timeout'а (result=fail).
        """
        if self._session_started_at is None:
            return
        duration_s = time.monotonic() - self._session_started_at
        if is_metrics_enabled():
            record_session_duration(duration_s, result=result)
        self._session_started_at = None
        self._session_end_reason = result
    def _on_inactivity_check(self) -> None:
        if self._core.check_timeout():
            self.get_logger().info("⏰ Dialogue timeout → IDLE")
            # Issue #1160 — Prometheus metrics: таймаут диалога = сессия
            # закрылась с result=fail (не штатный DIALOGUE_END).
            self._maybe_record_session_end(result="fail")
            self._publish_state()
    def shutdown_asyncio_loop(self, wait: bool = True) -> None:
        future = getattr(self, "_asyncio_loop_future", None)
        executor = getattr(self, "_asyncio_loop_executor", None)
        loop = getattr(self, "_loop", None)
        if future is None or executor is None or loop is None:
            return
        if future.done():
            executor.shutdown(wait=False)
            return
        try:
            loop.call_soon_threadsafe(loop.stop)
        except RuntimeError as exc:
            self.get_logger().warn(f"asyncio loop stop dispatch failed: {exc}")
        try:
            future.result(timeout=ASYNCIO_LOOP_DRIVER_SHUTDOWN_TIMEOUT_S)
        except concurrent.futures.TimeoutError:
            self.get_logger().warn(
                f"asyncio loop driver did not stop within "
                f"{ASYNCIO_LOOP_DRIVER_SHUTDOWN_TIMEOUT_S:.1f}s; cancelling")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"asyncio loop driver join raised: {exc}")
        finally:
            executor.shutdown(wait=False)
    def destroy_node(self) -> None:
        try:
            self.shutdown_asyncio_loop(wait=False)
        finally:
            super().destroy_node()

def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DialogueNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    try:
        node.shutdown_asyncio_loop(wait=False)
    except Exception:  # noqa: BLE001
        logging.getLogger(__name__).exception("dialogue_node: shutdown failed")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
