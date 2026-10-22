#!/usr/bin/env python3
"""dialogue_node.py — Voice dialogue ROS2 shell (Phase 6 v2 / W5).

Thin ROS2 shell that composes AgentCore over the harness ports
(LLMProvider, ToolProvider, MemoryStore, DSM). Owns only ROS2 pub/sub,
the asyncio loop driver, barge-in/cancel, TTS/sound
awaiter release, and lifecycle. Wake-word / silence classification
lives in core.dialogue_text; music is owned by the v2 engine in
mcp_server (ADR-0149, PR-13a removed the old DJ path);
TTS chunking + SSML framing + awaiter registry in core.speak_helpers.

Replaces the 2181-line predecessor by extracting the agent loop, every
function_tool body (OpenAI Agents SDK decorators), the provider registry,
history trimming, and all inline tool implementations into the harness
ports delivered in W1-W4.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
from concurrent.futures import ThreadPoolExecutor
from collections import deque
import functools
import json
import logging
import math
import os
import sys
import threading
import time
import traceback
import uuid
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import yaml

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, Int32, String
from nav_msgs.msg import Odometry
from rob_box_dialog_msgs.msg import Intent as IntentMsg
from rob_box_dialog_msgs.msg import InterruptRequest
from rob_box_dialog_msgs.msg import ProviderStateArray
from rob_box_dialog_msgs.msg import SessionState, SpeakerResult, SpeechEvent
from rob_box_dialog_msgs.msg import Utterance as UtteranceMsg

from rob_box_core.avatar_command import (
    AVATAR_COMMAND_TOPIC,
)
from rob_box_core.music_tools import MUSIC_AUDIBLE_LYRICS_TOOLS
from rob_box_core.speaker_wire import speaker_result_to_payload
from rob_box_core import expression_client, speech_client
from rob_box_core.utterance import Sink, Utterance
from rob_box_harness.config import LLMConfig
from rob_box_harness.core.agent_core import AgentCore, DialogResult
from rob_box_harness.core.assembly import (
    AgentSpec,
    build_agent,
    build_llm_chain,
    load_system_prompt,
)
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateKind,
    DialogueStateMachine,
)
from rob_box_harness.core.tool_loop.text_classify import is_done_marker
from rob_box_harness.core.tool_registry import ToolRegistry
from rob_box_harness.executors import ROSMCPToolProvider, adapt_tool_provider
from rob_box_harness.health import (
    DEFAULT_HEALTH_TTL_S,
    HealthAwareFallbackLLM,
    HealthCache,
    check_deepseek_balance,
)
from rob_box_harness.identity import Acquaintance, MemoryIdentitySeam
from rob_box_harness.identity.base import FaceSignal  # ADR-0139 §2.1
from rob_box_harness.memory import (
    Fact,
    InMemoryStore,
    MemoryStore,
    SQLiteVoiceMemory,
    speaker_scope,
)
from rob_box_harness.providers import (
    DEFAULT_BASE_URL as MINIMAX_DEFAULT_BASE_URL,
    DEFAULT_MODEL as MINIMAX_DEFAULT_MODEL,
    DEEPSEEK_DEFAULT_BASE_URL,
    DEEPSEEK_DEFAULT_MODEL,
    build_deepseek_provider,
    build_minimax_provider,
)
from rob_box_harness.tools import FakeToolProvider, ToolProvider
from rob_box_llm.errors import ProviderError
from rob_box_llm.provider import LLMMessage, LLMSettings, ToolCall

from rob_box_voice.core.command_parser import CommandParser
from rob_box_voice.core import command_bridge
from rob_box_core.provider_publisher import make_publisher, publish_provider_states
from rob_box_voice.core.provider_reporter import ProviderStateReporter, on_provider_states
from rob_box_voice.core.music_player_state import (
    MUSIC_EVENT_TOPIC,
    MUSIC_STATE_TOPIC,
    MusicEventLog,
    parse_music_state,
)
from rob_box_voice.core.interrupt_request import InterruptGate
from rob_box_voice.core.dialogue_stage_mixin import DialogueStageMixin
from rob_box_voice.core.turn_stages import TurnStages, scene_for_animation
from rob_box_voice.core.media_plan_run import command_turn_context, run_media_plan
from rob_box_dialog.context import build_context
from rob_box_dialog.music_state import MusicStateMemory
from rob_box_voice.core.turn_context import context_sections
from rob_box_voice.core.identity_dedup import identity_dedup_from_env
from rob_box_voice.core.stop_chooser import stop_chooser_from_env
from rob_box_voice.core.tools_view_host import apply_tools_view
from rob_box_voice.core.plan_host import begin_turn as plan_begin_turn
from rob_box_voice.core.plan_host import make_tool_executor
from rob_box_voice.core.plan_host import on_music as plan_on_music
from rob_box_voice.core.plan_host import on_music_event as plan_on_music_event
from rob_box_voice.core.plan_host import perform_busy
from rob_box_voice.core.stt_admission import (
    DefaultSttAdmission,
    MediaCommandStep,
    SttAdmission,
    SttAdmissionHost,
    SttContext,
    SttOutcome,
    SttOutcomeKind,
    parse_speaker_event,
)
from rob_box_voice.core.dialogue_text import (
    DEFAULT_OPERATOR_WAKE_WORDS,
    DEFAULT_WAKE_WORDS,
    has_wake_word,
    is_silence_command,
    resolve_wake_word_namespaces,
    strip_wake_word,
)
from rob_box_voice.core.llm_skip_reasons import (
    LLMSkipReason,
    new_llm_skip_counter,
)
from rob_box_voice.core.tier1_host import runner_of as tier1_runner_of
from rob_box_voice.core.dialogue_guards import (
    ACTION_CLAIM_RULES,
    build_action_claim_failure_fallback,  # Issue #2949
    build_fact_memory_save_fallback,  # Issue #2780 п.3
    build_phantom_action_retry_prompt,  # Issue #2559 phantom-action
    build_system_regurgitate_retry_prompt,
    build_tool_call_markup_retry_prompt,  # Issue #2760
    build_unbacked_action_retry_prompt,
    build_universal_action_claim_retry_prompt,
    detect_phantom_action_claim,  # Issue #2559 phantom-action
    detect_unbacked_action_claim,
    detect_universal_action_claim,
    is_music_stop_command,
    is_system_template_regurgitated,
    is_tool_call_markup,  # Issue #2760
    spoken_matches_claim_category,  # Issue #2780 п.3
    UniversalActionClaimHit,  # Issue #3174
)
from rob_box_voice.core.dialogue_helpers import (
    EMOTION_TO_ANIMATION as EMOTION_TO_ANIMATION,
    detect_pitch_intent,
    format_llm_skipped_summary,
    generate_fallback_response,
    map_emotion_to_animation,
    sanitize_speaker_name,
)
from rob_box_dialog.address import heard_from, log_label, person_ref_from
from rob_box_dialog.backlog import Backlog, phrase_count
from rob_box_dialog import phrases
from rob_box_dialog.knowledge import COMMAND_PROVIDERS_DEFAULT, IDENTITY_CODED_KINDS
from rob_box_dialog.speaker_adapter import AskCandidate, FaceSightings, merge_plan
from rob_box_dialog.doa import DoaTrack
from rob_box_voice.core.stt_admission_host import (  # noqa: F401
    # ADR-0145 §4 (P1 step 1) -- moved out of this module (pure move, no
    # behaviour change); re-exported so ``from rob_box_voice.dialogue_node
    # import <name>`` keeps working for existing callers/tests.
    _DialogueSttHost as _DialogueSttHost,
)
from rob_box_voice.core.tars_status_feed import start_status_feeds
from rob_box_voice.core.media_router import (
    MediaPlan,
    MediaRouter,
    MediaState,
    MediaToolCall,
    dj_theme_switch_plan,
    media_state_from_snapshot,
    media_tool_succeeded,
)
from rob_box_voice.core.media_phrases import honest_launch_reply
from rob_box_voice.core.music_turn import executor_turn_reply, history_recorder, next_turn_id, turn_heard_text
from rob_box_voice.core.named_play import (
    NamedPlayStatus,
    outcome_text,
    run_named_play,
    user_phrase_title,
)
from rob_box_voice.core.session_epoch import TURN_EPOCH, SessionEpoch
from rob_box_voice.core.turn_speech import (
    TurnSpeechHold, decide_turn_speech, final_text_for_speech,
    final_text_skipped_log, wants_lyrics,
)
from rob_box_voice.core.turn_identity import user_profile_lines
from rob_box_voice.core.self_intro import same_person_name
from rob_box_voice.core.set_length_words import heard_set_length
from rob_box_voice.core.speak_helpers import (
    EffectAwaiterRegistry, build_ssml_payload,
    split_into_chunks,
    strip_done_marker, strip_history_marker, strip_markdown,
    EMPTY_REPLY_PHRASE,
    strip_meta_markers,  # Issue #2547 — strip internal section headers
    strip_thinking_blocks,
)
from rob_box_voice.startup_greeting import pick_greeting
# ADR-0101 §3.1 — единый шов «можно ли заговорить» (issue #2536, PR-B).
from rob_box_voice.core.occasion import Occasion, OccasionGate, VerdictKind
from rob_box_voice.core.meeting import (
    MEETING_SOURCE_COOLDOWN_SEC,
    MeetingGreeter,
    greet_node_on_meeting,
    parse_meeting_marker,
)

from rob_box_voice.speaker_profiles import (
    SpeakerTracker,
    extract_speaker_name,
    format_speaker_context,
)
# Issue #1787 — сборка промпта и валидация клички, придуманной LLM.
from rob_box_voice.core import epithets
from rob_box_voice.core.utterance_speaker import UtteranceSpeakerRegistry
from rob_box_voice.core.identity_question import (
    SESSION_TOPIC,
    deliver_turn_result,
    identity_reply,
    introduced_name,
    open_merge_question,
    settled_by_voice,
    settle_identity,
    start_after_turn_question,
)
from rob_box_voice.core.speaker_turn import (
    ask_planned,
    note_backlog_voice,
    note_face,
    note_interlocutor,
    plan_identity_question,
    speaker_section,
    tentative_states,
)
# ADR-0101 §3.1 — ``Occasion`` импортирован выше (PR-B, #2536); старая
# однострочная запись из PR-A удалена как дубликат (использовалась только в
# type-аннотации под ``from __future__ import annotations``, runtime не нужна).

# Issue #1160 — Prometheus metrics (этап 1 observability).
# ``prometheus_client`` — optional dep; если её нет, всё превращается в
# no-op и старт сервера тихо возвращает ``False``.
from rob_box_voice.observability import (
    init_tracing,
    is_metrics_enabled,
    record_barge_in,
    record_session_duration,
    record_llm_prompt_tokens,
    record_voice_llm_request,
    start_metrics_server,
    start_span,
)


# ADR-0145 §4 (P1 step 1) -- this constant must stay defined HERE (not
# re-exported): ``test_no_daemon_threads.py`` and ``test_bounded_fanout.py``
# regex-scan this file's own source text for a literal
# ``ASYNCIO_LOOP_DRIVER_MAX_WORKERS: int = <N>`` assignment line, so
# moving the definition behind an import would silently break that
# structural guard even though the runtime behaviour is unchanged.
ASYNCIO_LOOP_DRIVER_MAX_WORKERS: int = 1
ASYNCIO_LOOP_DRIVER_NAME_PREFIX: str = "dialogue-async-loop"
ASYNCIO_LOOP_DRIVER_SHUTDOWN_TIMEOUT_S: float = 2.0

# Issue #1389 compatibility alias. ``LLMSkipReason`` is now the canonical
# source; this tuple remains for callers that imported the merged #1395 symbol.
_LLM_SKIP_REASONS: tuple[str, ...] = tuple(reason.value for reason in LLMSkipReason)


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


def _clean_name(value) -> Optional[str]:
    """Имя из ack — очищенное, или ``None``.

    Отдельная функция, потому что вызывается четырежды и каждый раз
    одинаково: привести к строке, прогнать через ``sanitize_speaker_name``
    (он режет служебные префиксы и мусор), пустое считать отсутствующим.
    """
    return sanitize_speaker_name(str(value or "")) or None


def identity_question(ack: dict) -> Optional[str]:
    """Вопрос про личность по ack регистрации — или ``None``, если молчим.

    Чистая функция: на входе распарсенный ack, на выходе строка. Вынесена
    из :meth:`DialogueNode._ask_identity_if_ambiguous` по двум причинам —
    её можно проверять без сборки ноды, и она держит цикломатическую
    сложность метода в бюджете ADR-0021 (гард `cc_budget` поймал CC=16
    при лимите 15, когда логика жила внутри метода).

    Два зеркальных случая, оба описаны в ADR-0127 и issue #2747:

    * ``name_twin`` — человек назвался именем, которое в базе уже есть, а
      голос до порога слияния не дотянул. Тёзка это или тот же человек,
      по биометрии не понять;
    * ``voice_conflict`` — наоборот, голос похож на известного, но имя
      другое. Живой пример: Саша и Борис звучат для resemblyzer на
      cos=0.846.

    Молчим (``None``), когда спрашивать не о чем или не о ком: обычная
    регистрация без неоднозначности, либо повод пришёл, но имени в нём
    нет — вопрос «ты тот самый или другой?» про неизвестно кого хуже
    молчания.
    """
    twin = ack.get("name_twin") or {}
    if twin:
        who = _clean_name(twin.get("name")) or _clean_name(ack.get("name"))
        if not who:
            return None
        return (
            f"Слушай, у меня уже записан {who}, но голос звучит иначе. "
            f"Ты тот самый {who} или другой человек?"
        )
    conflict = ack.get("voice_conflict") or {}
    if conflict:
        other = _clean_name(conflict.get("name"))
        if not (other and _clean_name(ack.get("name"))):
            return None
        return (
            f"Твой голос очень похож на голос, который я запомнил как "
            f"{other}. Вы разные люди или это ты под другим именем?"
        )
    return None


# Issue #2809 (продолжение) -- ответ на переспрос про tentative-личность
# ("<Имя>, это ты?" / "Как тебя зовут?"). Простая, консервативная эвристика
# по словам-границам (не по подстрокам -- "давай" не должно матчить "да"),
# как и просил координатор: не NLU, а короткий словарь. Неоднозначный или
# нераспознанный ответ -- ``None``, вызывающий код трактует его как отказ
# (issue #2809: "неоднозначный ответ = не подтверждено").
_IDENTITY_YES_WORDS = frozenset({
    "да", "ага", "угу", "точно", "верно", "конечно", "именно",
})
_IDENTITY_YES_PHRASES = ("это я", "я и есть", "он самый", "она самая")
_IDENTITY_NO_WORDS = frozenset({"нет", "неа", "не"})
_IDENTITY_NO_PHRASES = ("не я", "обознал", "другой человек", "не тот")


def classify_identity_confirmation(text: str) -> Optional[bool]:
    """Да/нет/непонятно на переспрос личности -- ``True``/``False``/``None``.

    Слова сравниваются ЦЕЛИКОМ (по границам, через ``str.split()``), а не
    подстрокой -- иначе «давай, пожалуй» матчил бы «да» и «пожалуй» тоже
    что-то не то. Фразы (``_IDENTITY_YES_PHRASES``/``_NO_PHRASES``) —
    подстрокой, это устойчивые составные обороты, ложных срабатываний на
    обычных словах не даёт.

    И да-, и нет-сигналы одновременно (или ни одного) — ``None``:
    неоднозначно, вызывающий код (``DialogueNode._resolve_pending_
    tentative_answer``) трактует это как отказ, не как повод спросить
    ещё раз (issue #2809: «не чаще одного переспроса на кандидата за
    сессию»).
    """
    norm = text.strip().lower()
    words = set(norm.replace(",", " ").replace(".", " ").split())
    has_yes = bool(words & _IDENTITY_YES_WORDS) or any(
        p in norm for p in _IDENTITY_YES_PHRASES
    )
    has_no = bool(words & _IDENTITY_NO_WORDS) or any(
        p in norm for p in _IDENTITY_NO_PHRASES
    )
    if has_yes and not has_no:
        return True
    if has_no and not has_yes:
        return False
    return None


class DialogueNode(DialogueStageMixin, Node):
    # Issue #2829 (ADR-0131 PR-2) -- явный список ``register_error``
    # причин, которые озвучиваем ("не расслышал, повтори"): намеренно
    # НЕ "любой event==register_error", чтобы будущее расширение ack
    # новым (нам неизвестным) кодом ошибки не начало внезапно что-то
    # говорить без синхронной правки здесь (тот же контракт, что
    # защищал одиночный "too_short" до этого PR, см. #2769).
    _SPOKEN_REGISTER_ERRORS = frozenset(
        {"too_short", "no_utterance_context", "utterance_not_found"}
    )
    """ROS2 shell that composes AgentCore over the harness ports."""
    def __init__(self) -> None:  # noqa: D401 — ROS2 ctor signature
        super().__init__("dialogue_node")
        # Issue #1234 — OpenTelemetry traces (этап 2). ВАЖНО: вызываем
        # ДО _build_llm(), потому что LLM-провайдеры (openai SDK / httpx)
        # создают свои httpx-клиенты при конструировании — авто-инструментация
        # httpx должна быть включена раньше, чтобы их вызовы попали в трейс.
        # Если opentelemetry-пакетов нет — no-op (см. observability.tracing).
        init_tracing("dialogue_node")
        self._declare_params()
        # ADR-0066 §6.3 — ``voice_input_mode`` УДАЛЁН. Runtime-параметры
        # (voice_preset, voice_output_language) логируются
        # в parameters_callback без рестарта ноды.
        self.add_on_set_parameters_callback(self.parameters_callback)
        # ADR-0083 §2.3 — wiring ``dialogue_node`` через ``build_agent(spec)``: системный промпт читает
        # ``load_system_prompt`` (assembly.py) ниже. Срез тулов хода — ``core/tools_view_host`` (ADR-0150 PR-6),
        # скиллов и их фрагментов больше нет.
        self._system_prompt: str = ""
        self._verbose_llm: bool = bool(self.get_parameter("verbose_llm").value)
        # #1990 — SSoT wake-слов (config/wake_words.yaml, docker). Файл
        # (wake_words_file) — источник personality в проде; параметр wake_words
        # остаётся фолбеком для dev-env/юнит-тестов без файла (список тот же,
        # байт-в-байт — см. test_wake_word_sync / #1252).
        self._wake_words: List[str] = list(
            resolve_wake_word_namespaces(
                str(self.get_parameter("wake_words_file").value or ""),
                personality_fallback=list(self.get_parameter("wake_words").value),
                operator_fallback=DEFAULT_OPERATOR_WAKE_WORDS,
            )[0]
        )
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
        # Вейк-слова парсера — по умолчанию из одной таблицы rob_box_dialog.knowledge.
        self._command_parser = CommandParser(confidence_base=0.8)
        # «новая сессия» / «сбрось всё» / Telegram «/clear» — Tier-1 (ADR-0150 PR-6): фразы —
        # ``rob_box_dialog.knowledge.NEW_SESSION_PHRASES``, разбор — ``grammar.match``.
        self._new_session_enabled: bool = bool(
            self.get_parameter("new_session_enabled").value
        )
        # Issue #2628 / ADR-0021 R1 — STT admission pipeline (refactor
        # of the inline 12-branch barrier chain inside ``_on_stt``).
        # Built once at init, re-used on every ``/voice/stt/utterance``.
        self._stt_admission: SttAdmission = DefaultSttAdmission(
            logger=self.get_logger(),
        )

        self._loop = asyncio.new_event_loop()
        self._asyncio_loop_executor = ThreadPoolExecutor(
            max_workers=ASYNCIO_LOOP_DRIVER_MAX_WORKERS,
            thread_name_prefix=ASYNCIO_LOOP_DRIVER_NAME_PREFIX,
        )
        self._asyncio_loop_future: Optional[concurrent.futures.Future] = (
            self._asyncio_loop_executor.submit(self._loop.run_forever)
        )
        self._run_task: Optional[asyncio.Task] = None
        self._task_lock = threading.Lock()
        self._run_cancelled: bool = False
        # Issue #2939 — ход, отменённый новой фразой: сессию у него уже
        # принял следующий ход, закрывать её (DIALOGUE_END) ему нельзя.
        self._handed_over_task: Optional[asyncio.Task] = None
        self._vad_speech_detected: bool = False
        self._effects = EffectAwaiterRegistry(
            release_tts=lambda ev: self._loop.call_soon_threadsafe(ev.set),
            release_sound=lambda ev: self._loop.call_soon_threadsafe(ev.set),
        )

        self._memory: MemoryStore = self._build_memory()
        # Issue #2440 — шов идентичности «Знакомый»: профиль спикера пишется
        # под СТАБИЛЬНЫМ биометрическим id (из /voice/speaker/result), а не
        # под per-session Yandex tag. tag остаётся только сигналом
        # подтверждения реплики внутри SpeakerTracker.
        self._identity = MemoryIdentitySeam(self._memory)
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
        # Issue #2829 (ADR-0131) -- единственный источник правды «кто
        # сказал ЭТУ фразу»: join STT+биометрии по utterance_id вместо
        # чтения последнего значения _current_speaker (гонка STT против
        # resemblyzer, см. ADR §"Проблема"). _current_speaker остаётся
        # как «последнее РАЗРЕШЁННОЕ по utterance_id значение» -- теперь
        # его пишет только _apply_speaker_identity, после resolve().
        self._utterance_speaker = UtteranceSpeakerRegistry()
        # Issue #2829 — какой utterance_id уже резолвнут в _current_speaker
        # (см. _resolve_speaker_for_utterance) -- не резолвим дважды.
        self._last_resolved_utterance_id: Optional[str] = None
        # Issue #2829 (ADR-0131 PR-2) — utterance_id ТЕКУЩЕГО хода. До
        # RegisterSpeakerTool (процесс mcp_server) доезжает скрытым
        # аргументом /mcp/execute через _mcp_turn_context (issue #2842).
        self._current_turn_utterance_id: Optional[str] = None
        self._speaker_resolve_timeout_sec: float = float(
            self.get_parameter("speaker_resolve_timeout_sec").value
        )
        # ADR-0139 §2.6 — face→voice hint: читаем namespace-параметры
        # (см. _declare_params ниже). При выключенном флаге логика
        # полностью отсутствует (note_face_seen не зовётся,
        # _handle_tentative_speaker не ищет hint — ADR-0139 §2.5).
        self._face_voice_hint_enabled: bool = bool(
            self.get_parameter("face_voice_hint.enabled").value
        )
        self._face_voice_hint_window_sec: float = float(
            self.get_parameter("face_voice_hint.window_sec").value
        )
        self._face_voice_hint_high_threshold: float = float(
            self.get_parameter("face_voice_hint.similarity_high_threshold").value
        )
        self._face_voice_hint_low_threshold: float = float(
            self.get_parameter("face_voice_hint.similarity_low_threshold").value
        )
        self._face_voice_hint_buffer_capacity: int = int(
            self.get_parameter("face_voice_hint.buffer_capacity").value
        )
        # Прокидываем параметры в сам шов — ADR-0139 §2.2. Вынесено в
        # helper, чтобы __init__ оставался в CC-budget ADR-0021.
        self._configure_face_voice_hint()
        # Гипотеза имени по собеседнику (#2809) — speaker_turn.tentative_states (ADR-0150 §7.5, #3695).
        # Issue #2828/#2908 -- вопрос по ack регистрации, пришедший посреди хода, ждёт конца хода (#3765).
        self._after_turn_question: Optional[dict] = None
        self._identity_question_session_gap_sec: float = float(
            self.get_parameter("identity_question_session_gap_sec").value
        )
        self._identity_answer_window_sec: float = float(
            self.get_parameter("identity_answer_window_sec").value
        )
        # ADR-0150 §7.3 (PR-5, В12): бэклог фраз без вейка — {text, t, doa_deg,
        # person_ref} на TTL; в LLM — только ≤ 1 свежая фраза того же источника.
        self._backlog = Backlog()
        self._accumulate_no_wake_enabled = bool(self.get_parameter("accumulate_no_wake_enabled").value)
        self._pending_unaddressed: Optional[Dict[str, Any]] = None
        # Направление на говорящего по фразе: DOA ReSpeaker во время речи (VAD).
        self._doa = DoaTrack()
        # ADR-0150 §7.5 (#3695): лица из кадра — для сверки «тот же собеседник» и вопроса об имени (В17);
        # кандидат на вопрос копится репликами, решение — после хода (core/speaker_turn.py).
        self._faces = FaceSightings()
        self._identity_ask_candidate: Optional[AskCandidate] = None
        # Issue #1195 — последний chat_id из Telegram (поле chat_id у
        # Utterance с source=telegram). Используется для
        # маршрутизации ответа: dialogue_node кладёт tg_chat_id в
        # payload /voice/dialogue/response, telegram_node читает его и
        # шлёт send_message в нужный чат. None = голосовой ввод.
        self._active_tg_chat_id: Optional[int] = None
        self._dsm: DialogueStateMachine = DialogueStateMachine(
            silence_timeout=float(self.get_parameter("dialogue_timeout").value),
        )
        # Issue #1986 §5.3 — inactivity timeout ушёл из AgentCore (ядро не
        # владеет таймаутом; это забота оболочки). Нода хранит значение и
        # сама гонит DSM в ``_on_inactivity_check``.
        self._dialogue_timeout_s: float = float(
            self.get_parameter("dialogue_timeout").value
        )
        # Issue #1160 — LLM держим и в атрибуте ноды: метрики
        # (``record_voice_llm_request``) и future OTel spans берут имя
        # провайдера из ``self._llm.name``, а не из ``self._core._llm``
        # (private-атрибут AgentCore). ADR-0083 §2.3 — LLM собираем ОДИН
        # раз через ``build_llm_chain(spec)`` (harness), передаём тот же
        # объект в ``build_agent(..., llm=...)``, чтобы не дублировать
        # build_provider() и не словить расхождение env-ключей.
        personality_spec = self._build_personality_spec()
        self._llm = build_llm_chain(personality_spec)
        # ADR-0150 PR-7b (#3699): исполнитель тулов под планировщиком ресурсов (``core.plan_host``), создаётся в
        # _build_tool_provider; None — тулов MCP нет.
        self._tool_executor: Any = None
        # ADR-0083 §2.3 — системный промпт читает ``load_system_prompt`` (harness) и передаёт в ``build_agent``.
        self._system_prompt = load_system_prompt(personality_spec)
        # ADR-0083 §2.3 — единственная точка сборки ``AgentCore`` в проде.
        # ``tools`` и ``memory`` приносит нода (ROS/asyncio-loop
        # ownership, ADR-0083 §2.3 trade-off); ``llm`` собран выше и
        # передаётся явно, чтобы ``self._llm`` и ``self._core._llm``
        # были ОДНИМ объектом (identity, не только ``==``).
        self._core: AgentCore = build_agent(
            personality_spec,
            tools=self._build_tool_provider(),
            memory=self._memory,
            llm=self._llm,
            system_prompt=self._system_prompt,
        )

        cbg = ReentrantCallbackGroup()
        qos_r = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST, depth=10)
        self._response_pub = self.create_publisher(
            String, "/voice/dialogue/response", 10)
        # ADR-0066 §6.3 — публикация /avatar/command сохранена как legacy
        # fallback. Телеграм-бот публикует в тот же топик из handlers, и
        # супервизор может использовать этот publisher для собственных
        # команд (контракт общий — см. rob_box_core.avatar_command и
        # worker-brief §3.3). RELIABLE+KEEP_LAST depth=10.
        self._avatar_command_pub = self.create_publisher(
            String, AVATAR_COMMAND_TOPIC, 10)
        self._state_pub = self.create_publisher(String, "/voice/dialogue/state", 10)
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
        # ADR-0150 PR-3 — отмена речи личности (SpeechCancel) одним клиентом.
        speech_client.prepare(self, "cancel")
        # ADR-0150 PR-3b (#3690) - стадии хода и рефлексы: на каждой TurnEvent и один ExpressionRequest (лицо, earcon),
        # 0 LLM; кольцо живёт по TurnEvent, исполнители отвечают ExpressionEvent / SoundEvent (молчаливых пропусков нет).
        expression_client.prepare(self)
        self._stages = TurnStages(
            functools.partial(expression_client.express, self), functools.partial(expression_client.turn_event, self),
            epoch=lambda: self._session_epoch_gate().current, normalize=scene_for_animation,
            log=self.get_logger().info)
        self._speaking_watch = speech_client.SpeakingWatch("personality")
        self.create_subscription(SpeechEvent, "/voice/speech/event", self._on_speech_event, 10)
        expression_client.watch_events(self, self._on_expression_event, self._on_sound_event)
        self.create_timer(1.0, self._turn_stages().tick)
        # ADR-0150 PR-4 (#3693) - здоровье LLM-провайдеров: latched /voice/providers/state + одна фраза на смену.
        # Один топик, писатели по видам: свой kind=llm переиздаётся каждый тик, чужие (tts/stt/music_gen) читаются
        # в доску ``ProviderBoard`` (запись без обновлений 10 с = unknown). Фраза об отказе - любого вида.
        self._provider_state_pub = make_publisher(self)
        self._provider_reporter = ProviderStateReporter(
            self._llm, functools.partial(publish_provider_states, self._provider_state_pub), self._speak_direct)
        self.create_subscription(
            ProviderStateArray, "/voice/providers/state",
            functools.partial(on_provider_states, self._provider_reporter.board, time.time),
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(2.0, self._provider_reporter.tick)
        # ADR-0150 §7.1/§7.5 (#3695): снимок сессии (собеседник с азимутом, кадр Identity) — latched, до PR-7 пишет v1.
        self._session_pub = self.create_publisher(
            SessionState, SESSION_TOPIC,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # ADR-0150 §6.2 (PR-5 #3694) — перебивание речи решает диалог: stt_node на сегмент с вейком шлёт
        # /dialog/interrupt_request, ясная стоп-команда рвёт речь личности текущей эпохи, остальное — нет
        # (новая реплика встаёт после текущей). Политика перебивания одна, табличная, без параметра.
        self._interrupt_gate = InterruptGate(
            lambda reason: speech_client.cancel(
                self, agent="personality", epoch=self._session_epoch_gate().current, reason=reason),
            lambda: self._wake_words, self.get_logger().info,
            heard=lambda uid: self._turn_stages().addressed(uid, "respeaker"))
        self.create_subscription(InterruptRequest, "/dialog/interrupt_request", self._interrupt_gate.on_request,
                                 qos_r, callback_group=cbg)
        # ADR-0150 PR-6 (#3696) — Tier-1 грамматика: спорный стоп (В19) решает Chooser; «звучит ли речь» (канал
        # «громче») — SpeakingWatch, его кормит та же подписка /voice/speech/event, что стадии хода (_on_speech_event).
        self._stop_chooser = stop_chooser_from_env(log=self.get_logger)
        self._identity_dedup = identity_dedup_from_env(log=self.get_logger)
        # ADR-0150 PR-2: один типизированный вход реплики (голос и Telegram);
        # id фразы, источник и chat_id - поля, склейки по тексту больше нет.
        self.create_subscription(
            UtteranceMsg, "/voice/stt/utterance",
            # «Клод …» из Telegram (#3518) решает rob_box_dialog.address.decide.
            self._on_stt, qos_r, callback_group=cbg)
        # ADR-0150 PR-5 — команды движения/статуса после допуска: исполнитель command_node.
        self._intent_pub = self.create_publisher(IntentMsg, "/dialog/intent", qos_r)
        self.create_subscription(Bool, "/audio/vad", lambda m: self._doa.on_vad(m.data, time.time()), 10)
        self.create_subscription(Int32, "/audio/direction", lambda m: self._doa.on_direction(m.data, time.time()), 10)
        # ADR-0066 §6.1 — единственная связь с агентом оператора:
        # sub /dialogue/control (String JSON {action: pause|resume}),
        # pub /dialogue/control_ack (String JSON {state, since_ms, ts_s, reason}).
        # Без TTL — личность выходит из паузы только по явному resume.
        # Маршрут речи оператора уехал из dialogue_node: stt_node публикует
        # фразу с левого грипа в /avatar/ptt/result (пайплайн грипа в
        # avatar_supervisor, #1989), wake-поток — в /avatar/stt/result (#1988).
        # Личность больше не знает о Quest.
        self._dialogue_control_pub = self.create_publisher(
            String, "/dialogue/control_ack", qos_r)
        self.create_subscription(
            String, "/dialogue/control", self._on_dialogue_control,
            qos_r, callback_group=cbg)
        self._paused_at_ms: Optional[int] = None
        self._pause_reason: str = ""
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
        # топик). JSON: {"speaker_tag", "text", "duration_s"}. stt_node
        # публикует speaker ПЕРЕД utterance, поэтому _on_speaker обычно
        # приходит раньше _on_stt; храним по тексту и забираем в _on_stt.
        self.create_subscription(
            String, "/voice/stt/speaker", self._on_speaker, qos_r,
            callback_group=cbg)
        # Issue #1077 — голосовая биометрия: результат speaker_id_node
        # (resemblyzer d-vector). ADR-0150 PR-2: типизированный SpeakerResult.
        if self._speaker_id_enabled:
            self.create_subscription(
                SpeakerResult, "/voice/speaker/result", self._on_speaker_result,
                qos_r, callback_group=cbg)
            self._speaker_register_pub = self.create_publisher(
                String, "/voice/speaker/register", 10)
            # Issue #2828 -- ответ «это я» на переспрос про личность
            # склеивает профили тем же путём, что ручная склейка
            # оператора (speaker_id_node._on_merge_request).
            self._speaker_merge_pub = self.create_publisher(
                String, "/voice/speaker/merge", 10)
            # Issue #3801 -- «зови меня Y» переименовывает профиль тем же
            # путём, что тул register_speaker с old_name
            # (speaker_id_node._on_rename_request).
            self._speaker_rename_pub = self.create_publisher(
                String, "/voice/speaker/rename", 10)
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
        # Issue #1229 — фактический провайдер TTS (после фолбека) от tts_node.
        # JSON {"provider": str, "voice": str, ...}. Используется в контексте
        # [TTS], чтобы LLM видела голоса РЕАЛЬНОГО провайдера (а не
        # номинального minimax), и не выбирала голоса, которых нет у
        # фактического провайдера.
        self.create_subscription(
            String, "/voice/tts/provider_state", self._on_tts_provider_state, 10,
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
        #
        # Issue #3133 (ADR-0141): «играет» диалог читает ТОЛЬКО из этого
        # топика (``_music_player_state`` → ``_music_playing_now``). Latched,
        # как у публикатора: после рестарта ноды снимок приходит сразу.
        self._music_player_state = None
        # Issue #3161 — те же снимки → тег <music_state> для LLM (что играет
        # и что замолчало недавно; имя трека — ``dj.title`` снимка v2).
        self._music_state_memory = MusicStateMemory()
        self.create_subscription(
            String, MUSIC_STATE_TOPIC, self._on_music_state,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=cbg)
        # ADR-0149 PR-6: события владельца плеера v2 (started/rejected) — фраза
        # роутера об успехе запуска только по ``started`` (core/media_plan_run.py);
        # ADR-0150 PR-7c: nearly_finished/finished подложки — выступлению (plan_host).
        self._music_events = MusicEventLog()
        self.create_subscription(
            String, MUSIC_EVENT_TOPIC,
            lambda msg: (self._music_events.observe_msg(msg), plan_on_music_event(self, msg)),
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            ),
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
        # Issue #2599 PR-C — лицевой канал как повод заговорить.
        # Отдельным методом, а не инлайном: try/except здесь упирал
        # DialogueNode.__init__ в потолок цикломатической сложности
        # (ADR-0021, cc_budget: CC=21 при лимите 20).
        self._subscribe_vision_events(cbg)
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
        # Issue #2835 — поколение диалоговой сессии: ход/ретрай, родившийся
        # до «новой сессии», после неё не запускается и не включает DJ.
        self._session_epoch = SessionEpoch()

        # Issue #992 Bug E — «отчитался о действии, но не вызвал тул».
        # Тот же одноразовый контракт, что у babble-флага выше: ретраим
        # РОВНО один раз, иначе LLM и код уходят в пинг-понг.
        self._action_claim_retry_used: bool = False

        # Issue #2175 — MiniMax-M3 regurgitates ``<system>...</system>``
        # template после ``set_voice`` + multi-voice user_input. Один
        # одноразовый ретрай с явным требованием отвечать обычным
        # языком, иначе LLM и код уходят в пинг-понг.
        self._system_regurgitate_retry_used: bool = False

        # Issue #2760 — LLM печатает вызов тула текстом
        # (``<function_calls><invoke name=...>``) вместо настоящего
        # tool-call, и разметка уходит в TTS. Один одноразовый ретрай.
        self._tool_call_markup_retry_used: bool = False

        # Issue #2549 — универсальный anti-hallucination guard (см.
        # :func:`detect_universal_action_claim`). Срабатывает когда в
        # ``spoken`` есть action-verb (сделал/запустил/включил/проверю/
        # обновлю/перезапущу/…), но ``tools_called`` пустой. Существующий
        # Bug E guard слишком узкий для DJ-кейсов («сделала два pass»,
        # «вплела тему Грига» — user_input не содержит «сделай»).
        # Тот же одноразовый контракт: один ретрай на turn.
        self._universal_action_claim_retry_used: bool = False

        # Issue #2559 — phantom-action claim (общий, НЕ music-only):
        # «сейчас перезапущу / сделал / подложу» при ``tools_called=[]``.
        # Round 3 live (Vision Pi 10.1.1.21, 15.09.2026) — 6 случаев за
        # час. Bug E (#992 / #2548) ловит только узкие music-claim'ы
        # при ``dj_active`` или music-kw; phantom-action — расширение
        # на ВСЕ action-verb'ы вне зависимости от контекста.
        self._phantom_action_retry_used: bool = False

        # Issue #1881 — общий бюджет СИНТЕТИЧЕСКИХ ретраев на user-turn.
        # Раньше у каждого guard'а был свой одноразовый флаг, и каждый
        # СВОЙ сбрасывал ЧУЖИЕ на следующем turn — ping-pong был неизбежен.
        # Единый бюджет декрементируется любым guard'ом; на свежем
        # user-initiated turn — ресетится в
        # ``_dispatch_turn`` / ``_run_turn``. Чтобы существующие
        # поимённые флаги не разъехались с новым бюджетом (тесты читают
        # их напрямую), guard'ы синхронно выставляют свой флаг и декрементят
        # ``_synthetic_retries_left`` через ``_consume_synthetic_retry``.
        self._synthetic_retries_left: int = self.DEFAULT_SYNTHETIC_RETRIES

        # Issue #1160 — Prometheus metrics: длительность диалоговой
        # сессии. Фиксируем момент первого wake-word-диалога из IDLE;
        # при DIALOGUE_END / timeout пишем histogram.
        self._session_started_at: Optional[float] = None
        self._session_end_reason: str = "success"

        self.create_timer(5.0, self._on_inactivity_check)
        # 🔴 FIX (live 06.08): startup-приветствие внутри dialogue_node
        # (замена отдельной ноды приветствия, #1003). Одноразовый
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
        # #3253 Ш3б: настроенная LLM и wake-слова → latched-топики для экрана
        # ТАРС 1 (quest_node → tars_status). Значения статичны до рестарта.
        self._tars_status_feeds = start_status_feeds(
            self, String, self._llm, self._wake_words
        )
        self.get_logger().info("✅ DialogueNode shell ready (AgentCore wired)")
    def _declare_params(self) -> None:
        # 🔴 FIX (live 18:00): тариф MiniMax кончился (429 rate_limit
        # 'usage limit reached'). YAML мёртв (#1004) — дефолт
        # в коде единственный живой путь → переключаем на deepseek.
        # DEEPSEEK_API_KEY в env voice-assistant.
        # 🔴 FIX (live 10.08, issue #1089): приоритетная цепочка LLM-провайдеров.
        # Формат: "deepseek,minimax" — порядок = приоритет.
        # Первый в списке = primary, остальные = fallbacks.
        # Каждый провайдер настраивается в своей YAML-секции (<name>.base_url и т.д.).
        self.declare_parameter("llm_providers", "deepseek")
        # ADR-0150 §4.5 (#3696 ч.5): провайдеры, которые отвечают ОДНОЙ командой (поле speech, решение О4);
        # форму запроса (tool_choice | json_in_text | …) выбирает код по capabilities адаптера —
        # core/command_bridge.py. По умолчанию — knowledge.COMMAND_PROVIDERS_DEFAULT (решение Шифу 22.10);
        # пустая строка — все на speak_text, как до моста.
        self.declare_parameter("llm_command_providers", COMMAND_PROVIDERS_DEFAULT)
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
        self.declare_parameter("history_max_turns", 20)
        self.declare_parameter("dialogue_timeout", 300.0)
        # Один список на весь проект — rob_box_voice.core.dialogue_text.
        # Он же фолбек strip_wake_word, и его порядок неслучаен (длинные
        # варианты первыми, иначе «роб» съедает «роб бокс»). Копий было
        # семь и они разошлись на три разных списка: здесь и в
        # stt_node.py лежало 13 вариантов, в четырёх YAML — 21, в
        # e2e-конфиге — те же 13. Тот самый класс ошибки, из-за которого
        # завели #1252 и заплатили #1734.
        self.declare_parameter("wake_words", list(DEFAULT_WAKE_WORDS))
        # #1990 (оператор-agent 05) — SSoT wake-слов: config/wake_words.yaml
        # (docker, монтируется в /config). Пусто в dev-env → кодовый фолбек.
        self.declare_parameter("wake_words_file", "")
        self.declare_parameter("enable_mcp_tools", True)
        self.declare_parameter("verbose_llm", True)
        # Issue #1279 — gate команд движения/статуса. command_node уже
        # обрабатывает NAVIGATE/STOP/STATUS/MAP (публикует
        # /voice/command/intent и выполняет), dialogue_node НЕ должен
        # дублировать обработку через LLM: иначе LLM интерпретирует
        # «вперёд» как музыку → запуск музыки вместо движения.
        # True = фразы-команды НЕ идут в LLM (обрабатывает command_node).
        self.declare_parameter("command_intent_gate_enabled", True)
        # Порог уверенности CommandParser, при котором фраза считается
        # «уже обработанной командой» (совпадает с command_node.yaml
        # confidence_threshold: 0.7).
        self.declare_parameter("command_intent_gate_confidence", 0.7)
        # Issue #XXXX — «новая сессия» / «сбрось всё» / Telegram «/clear»:
        # детерминированный сброс текущего диалога без вызова LLM.
        self.declare_parameter("new_session_enabled", True)
        # Issue #2890 — E2E-харнесс сбрасывает сессию диалога в начале
        # каждого акта: ``ros2 param set /dialogue_node
        # e2e_session_reset_token <уникальный токен>`` (тот же приём
        # ``ros2 param set``, что e2e_mode у speaker_id_node/mcp_server).
        # Новое значение = тихий сброс «новая сессия» без подтверждения
        # голосом; при отказе сброса колбэк отклоняет значение, и харнесс
        # видит это, прочитав параметр обратно. См. parameters_callback.
        self.declare_parameter("e2e_session_reset_token", "")
        # 🔴 FIX (live 06.08): стриминг LLM через конфиг (llm_streaming).
        # Замер без стриминга: false → complete() (полный ответ).
        self.declare_parameter("llm_streaming", False)
        # Раньше по умолчанию стоял ``handle_navigation`` — фасад, удалённый
        # вместе с Compositor-скиллами, поэтому фильтр не отсекал ничего.
        self.declare_parameter("history_excluded_tools", ["move_direction"])
        self.declare_parameter("sqlite_db_path", "~/.rob_box/voice.db")
        self.declare_parameter("speaker_id_enabled", True)
        self.declare_parameter("speaker_db_path", "/data/speakers.db")
        # Issue #2809 (продолжение) -- сколько секунд молчания/паузы ещё
        # считается той же "сессией" переспроса про tentative-личность
        # (одна попытка на кандидата за сессию). Тот же дефолт 30s, что
        # gallery_growth_session_gap_sec у speaker_id_node (PR #2757) --
        # намеренно зеркалим значение, а не импортируем его: узлы это
        # разные процессы, общего источника правды для рантайм-параметра
        # между ними в этом репо нет (см. e2e_mode -- тот же паттерн:
        # параметр каждого узла независим).
        #
        # Issue #2809 (живой прогон 35857257981): значение поднято с 30s до
        # 120s. 30s зеркалили gallery_growth_session_gap_sec, но паузы между
        # репликами живого диалога (и шагами харнесса, ~60s) длиннее --
        # принятое решение ("да"/"нет") сбрасывалось посреди разговора, и
        # робот переспрашивал заново (n704/n707). Этот параметр теперь
        # держит только РЕШЁННОЕ состояние; ожидание ответа -- отдельное
        # окно identity_answer_window_sec ниже.
        self.declare_parameter("identity_question_session_gap_sec", 120.0)
        # Issue #2809 -- сколько секунд заданный вопрос ("Саша, это ты?")
        # ждёт ответа. В прогоне 35857257981 "да" пришло через 84s (шаг
        # харнесса ~60s + переспрос STT "не расслышал") -- при общем окне
        # 30s состояние пересоздавалось с asked=False и ответ не читался.
        self.declare_parameter("identity_answer_window_sec", 180.0)
        # Issue #2829 (ADR-0131) -- сколько ждать результат голосовой
        # биометрии ИМЕННО этой фразы, прежде чем честно объявить её
        # unknown. По логам issue #2829 инференс resemblyzer занимает
        # 0.6-1.9с (изредка до ~50с сразу после старта ноды -- для этого
        # выброса таймаут НЕ увеличиваем: лучше unknown, чем 50с тишины
        # в диалоге или, что было раньше, имя предыдущего собеседника).
        # 2.5с = запас x1.3 над верхней границей нормального диапазона.
        self.declare_parameter("speaker_resolve_timeout_sec", 2.5)
        # ADR-0139 §2.6 — face→voice hint: свежее наблюдение лица в шов
        # «Знакомый» отменяет голосовой переспрос #2809, если имя лица и
        # имя голосового кандидата совпадают и hint в окне. Дефолты из
        # ADR-0123 §6 (high=0.78) и ADR-0089 §2.2 (low=0.65 стаб-зеркало).
        # Деградация к текущему поведению — ADR-0139 §2.5: false здесь
        # = подписка на /vision/hailo/events не зовёт note_face_seen,
        # _handle_tentative_speaker не ищет hint. Ключи dotted — стандарт
        # ROS2 для namespace-секций (см. test_yaml_param_consistency).
        self.declare_parameter("face_voice_hint.enabled", True)
        self.declare_parameter("face_voice_hint.window_sec", 30.0)
        self.declare_parameter("face_voice_hint.similarity_high_threshold", 0.78)
        self.declare_parameter("face_voice_hint.similarity_low_threshold", 0.65)
        self.declare_parameter("face_voice_hint.buffer_capacity", 8)
        # issue #1077: сколько фраз подряд с одним speaker_tag нужно для
        # подтверждения профиля. 2 = защита от нестабильных tags Yandex;
        # 1 = мгновенное подтверждение (если tag стабилен).
        self.declare_parameter("speaker_min_phrases", 2)
        # Бэклог фраз без вейка (ADR-0150 §7.3); TTL/окна — knowledge.BACKLOG_*.
        self.declare_parameter("accumulate_no_wake_enabled", True)
        # 🔴 FIX (issue #1082): health-кэш LLM-провайдеров. Файл переживает
        # рестарт робота: если MiniMax мёртв (квота), первый же
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
        # Отдельная нода приветствия страдала гонкой (tts_node ещё не
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
        # ADR-0101 §3.1 / PR-B: единый шов «можно ли заговорить» (issue #2536).
        # Стартовый gate: глобальный дебаунс 2с, startup — one-shot,
        # dj_tick / unclear / inactivity — резерв для PR-D/E.
        # Встреча лицом — повод поздороваться с ЗНАКОМЫМ после долгой разлуки: исключение Шифу 21.10 из В12′.
        self._occasion: OccasionGate = OccasionGate(source_cooldowns={"meeting": MEETING_SOURCE_COOLDOWN_SEC})
        self._meeting_greeter: MeetingGreeter = MeetingGreeter()
        # Issue #1219 — LLM voice selection: активный TTS-провайдер для
        # контекста [TTS]. Должен совпадать с tts_node.yaml provider
        # (minimax). Установленный голос хранит tts_node (VoiceOwner, PR-4 ч.3) и сообщает его в provider_state.
        self.declare_parameter("tts_provider", "minimax")
        # Issue #1229 — фактический провайдер TTS (после фолбека) и голос,
        # который РЕАЛЬНО использовал tts_node. Приходят из /voice/tts/
        # provider_state; используются в LLM-контексте [TTS], чтобы LLM
        # видела голоса фактического провайдера, а не номинального.
        self._actual_tts_provider: str | None = None
        self._actual_tts_voice: str | None = None
        # Issue #1160 — Prometheus metrics endpoint. 9100 = voice (LLM
        # latency / fallback). 0 = отключить старт сервера (полезно для
        # юнит-тестов и CI, где рконфликтует с другими тестами).
        self.declare_parameter("metrics_port", 9100)
        # ADR-0066 §6.3 — `voice_input_mode` УДАЛЁН. Единственная связь
        # оператора с личностью — топик /dialogue/control (sub выше, в
        # __init__).
        #
        # voice-vr 21 / ADR-0080 §2.7 — ``voice_preset`` /
        # ``voice_output_language`` тоже удалены: супервизор больше НЕ
        # пишет в эти параметры (SetParameters-контракт на dialogue_node
        # был мёртвым с ADR-0066 §6.3 — формализатор уехал в
        # ``grip_pipeline`` и читает ``voice_presets.yaml`` напрямую).
        # Грядущий явный канал — расширение ``/dialogue/control`` под
        # set_preset/set_language, отдельная карточка.

    def parameters_callback(self, params):
        """Роутер runtime-изменений параметров (``ros2 param set``).

        ADR-0066 §6.3 — ``voice_input_mode`` УДАЛЁН. Единственный канал
        оператора — ``/dialogue/control``.

        voice-vr 21 / ADR-0080 §2.7 — ``voice_preset`` /
        ``voice_output_language`` УДАЛЕНЫ из dialogue_node; политика
        перебивания — одна (ADR-0150 §6.2, PR-5), параметра у неё нет.
        """
        for param in params:
            if param.name == "e2e_session_reset_token":
                if not self._reset_session_for_e2e_act(str(param.value or "")):
                    return SetParametersResult(
                        successful=False,
                        reason="issue 2890: e2e session reset failed",
                    )
            else:
                # voice-vr 21: ``voice_preset`` / ``voice_output_language``
                # удалены из declare_parameters; их SetParameters не должен
                # доходить сюда. Если пришёл — это либо старая нода
                # supervisor_node (PR #2243), либо ручной ``ros2 param
                # set`` в проде. Не роняем ноду, но логируем warning,
                # чтобы ловилось в метриках.
                self.get_logger().warning(
                    f"⚠️ [voice-vr 21] unknown runtime param ignored: "
                    f"name={param.name!r} value={param.value!r} "
                    f"(voice_preset / voice_output_language были УДАЛЕНЫ "
                    f"из dialogue_node, ADR-0080 §2.7)"
                )
        return SetParametersResult(successful=True)

    def _parse_provider_chain(self) -> tuple[str, ...]:
        """Разобрать CSV ``llm_providers`` в нормализованный tuple.

        ADR-0083 §2.3 — раньше жило inline в ``_resolve_provider_chain``
        dialogue_node; теперь единый inline-парсер для personality-ноды
        (у supervisor'а такой же, ADR-0043 §3.2 / issue #2111).
        ``deepseek`` — дефолт, если параметр пуст.
        """
        providers_str = str(
            self.get_parameter("llm_providers").value or "deepseek"
        ).strip()
        return tuple(
            p.strip().lower()
            for p in providers_str.split(",")
            if p.strip()
        )

    def _resolve_global_llm_settings(self) -> LLMSettings | None:
        """Глобальные ``temperature`` / ``max_tokens`` из ROS-параметров.

        ADR-0083 §2.3 — было частью ``_build_llm_settings_for``.
        YAML 0 → ``None`` (LLMSettings конвенция: «оставь провайдеру»).
        """
        try:
            global_temperature = float(
                self.get_parameter("temperature").value or 0.0
            )
        except Exception:
            global_temperature = 0.0
        try:
            global_max_tokens = int(
                self.get_parameter("max_tokens").value or 0
            )
        except Exception:
            global_max_tokens = 0
        if global_temperature <= 0 and global_max_tokens <= 0:
            return None
        return LLMSettings(
            temperature=(
                global_temperature if global_temperature > 0 else None
            ),
            max_tokens=(global_max_tokens if global_max_tokens > 0 else None),
        )

    def _resolve_per_provider_llm_settings(
        self, chain: tuple[str, ...]
    ) -> dict[str, LLMSettings]:
        """Per-provider override (issue #1883): ``<name>.temperature/max_tokens``.

        Если для провайдера ничего не задано — он пропускается (не
        попадает в ``per_provider_settings``, чтобы ``build_agent``
        увидел «пусто» и не лез в ``None``).
        """
        per_provider: dict[str, LLMSettings] = {}
        for name in chain:
            try:
                per_temperature = float(
                    self.get_parameter(f"{name}.temperature").value or 0.0
                )
            except Exception:
                per_temperature = 0.0
            try:
                per_max_tokens = int(
                    self.get_parameter(f"{name}.max_tokens").value or 0
                )
            except Exception:
                per_max_tokens = 0
            if per_temperature <= 0 and per_max_tokens <= 0:
                continue
            per_provider[name] = LLMSettings(
                temperature=(
                    per_temperature if per_temperature > 0 else None
                ),
                max_tokens=(per_max_tokens if per_max_tokens > 0 else None),
            )
        return per_provider

    def _resolve_personality_prompt_dir(self) -> Path:
        """Где лежат ``prompts/`` для personality.

        ADR-0083 §2.3 — ``prompt_dir`` часть спек. Возвращаем
        ``<share_dir>/prompts`` через ``ament_index_python`` —
        тестовые обёртки (``scripts/dialogue/chat.py``,
        ``ros2 launch test``) могут подменить через mock ``spec``
        напрямую, эта функция только точка правды для production.
        """
        from ament_index_python.packages import (
            get_package_share_directory as _ament_probe,
        )
        return Path(_ament_probe("rob_box_voice")) / "prompts"

    def _build_personality_spec(self) -> AgentSpec:
        """Собрать :class:`AgentSpec` для личности робота (ADR-0083 §2.3).

        Оркестратор: декомпозирован на 4 хелпера (``_parse_provider_chain``,
        ``_resolve_global_llm_settings``, ``_resolve_per_provider_llm_settings``,
        ``_resolve_personality_prompt_dir``) для соблюдения CC-budget
        ADR-0021 (limit 15). Каждый хелпер покрывает один срез параметров;
        ``build_agent`` потом собирает по спеке ``AgentCore`` без знания
        про rclpy.
        """
        chain = self._parse_provider_chain()
        spec_settings = self._resolve_global_llm_settings()
        per_provider = self._resolve_per_provider_llm_settings(chain)

        cache_path_raw = str(
            self.get_parameter("health_cache_path").value or ""
        ).strip()
        try:
            health_ttl = float(
                self.get_parameter("health_ttl_s").value
                or DEFAULT_HEALTH_TTL_S
            )
        except (TypeError, ValueError):
            health_ttl = DEFAULT_HEALTH_TTL_S

        try:
            prompt_dir: Path = self._resolve_personality_prompt_dir()
        except Exception as exc:  # noqa: BLE001 — старт-страховка
            self.get_logger().warning(
                f"⚠️ ament_index probe for rob_box_voice failed: {exc!r}; "
                "spec.prompt_dir будет пустым, load_system_prompt вернёт ''"
            )
            prompt_dir = Path()

        return AgentSpec(
            name="personality",
            prompt_dir=prompt_dir,
            system_prompt_file=str(
                self.get_parameter("system_prompt_file").value
                or "master_prompt_compact.txt"
            ),
            memory_namespace="personality",
            on_prompt=self._on_prompt_stats,
            provider_chain=chain,
            settings=spec_settings,
            per_provider_settings=per_provider,
            use_streaming=bool(
                self.get_parameter("llm_streaming").value or False
            ),
            health_cache_persist_path=(
                Path(cache_path_raw).expanduser() if cache_path_raw else None
            ),
            health_ttl_s=health_ttl,
            history_trim_limit=int(
                self.get_parameter("history_max_turns").value or 20
            ),
            dsm=self._dsm,
            user_id="default",
            wrap_provider=command_bridge.wrapper_for(
                self.get_parameter("llm_command_providers").value, log=self.get_logger().info,
                busy=lambda: perform_busy(self)),  # PR-7c: edit_pending — по ACTIVE-сегменту выступления
        )

    def _on_prompt_stats(self, stats: Any) -> None:
        """Опубликовать размер промпта, посчитанный AgentCore.

        Колбэк зовётся из harness на КАЖДОЕ обращение к LLM, включая
        каждую итерацию тул-цикла. Harness намеренно ничего не знает про
        Prometheus — он только считает, публикует нода.

        Любое исключение здесь гасится: телеметрия не имеет права ронять
        живой ход. AgentCore тоже глушит исключения наблюдателя — это
        второй слой на случай прямого вызова из тестов.
        """
        try:
            record_llm_prompt_tokens(
                stats.provider,
                tokens=stats.prompt_tokens,
                skill=stats.skill,
                estimated=stats.estimated,
            )
            # ADR-0150 A5 (#3696): строка на КАЖДЫЙ вызов LLM — p50 токенов считает scripts/dialog/turn_metrics.py.
            blocks = getattr(stats, "blocks", None)
            if blocks is not None:
                self.get_logger().info(blocks.log_line(
                    prompt_tokens=stats.prompt_tokens, estimated=stats.estimated,
                    provider=stats.provider, view=stats.skill))
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

    # ADR-0066 §6.3 — ``_DEFAULT_VOICE_PRESETS_FILE`` УДАЛЁН вместе с
    # AV-28 (формализация пресета переехала в grip_pipeline супервизора).

    def _mcp_turn_context(self) -> dict:
        """Контекст хода для скрытых аргументов MCP-тулов (issue #2842).

        Читается ``LLMToolCallAdapter`` в момент отправки запроса тула;
        при ``tool_provider=ros_mcp`` тул исполняется в процессе
        ``mcp_server`` и сам до атрибутов этого узла не дотянется.
        """
        uid = getattr(self, "_current_turn_utterance_id", None)
        return {
            "utterance_id": uid,
            # Issue #2925 -- register_speaker_gate: что человек сказал о
            # себе в этой реплике и кем она уверенно узнана по голосу.
            "self_intro_name": getattr(self, "_turn_self_intro", None),
            "intro_registered": bool(
                getattr(self, "_turn_intro_registered", False)
            ),
            "known_speaker_name": self._confident_speaker_name(uid),
            # 06.10: длина сета из слов человека («замути сэт на 30 минут») —
            # решает грамматика, а не LLM: dj_set без tracks брал 10.
            "heard_tracks": getattr(self, "_turn_set_tracks", None),
            # 06.10 set98207: request_music не снимает сет этого же хода.
            "turn_id": getattr(self, "_turn_id", None),
            # 06.10 18:04: тема dj_set сверяется с репликой хода, а не с историей.
            "heard_text": getattr(self, "_turn_heard_text", None),
        }

    def _confident_speaker_name(
        self, utterance_id: Optional[str]
    ) -> Optional[str]:
        """Issue #2925 -- имя, под которым ЭТА реплика уверенно узнана по
        голосу, или ``None``. Только когда снимок ``_current_speaker``
        резолвнут именно по ``utterance_id`` хода (ADR-0131): у хода без
        реплики (Telegram, синтетический) диктора нет, прошлый не в счёт.
        Tentative-снимок (#2809) несёт ``name=None`` -- тоже ``None``."""
        if not utterance_id or utterance_id != getattr(
            self, "_last_resolved_utterance_id", None
        ):
            return None
        with self._speaker_lock:
            sp = dict(self._current_speaker)
        if not sp.get("is_known"):
            return None
        return sanitize_speaker_name(str(sp.get("name") or "")) or None

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
        #    ``discover/execute`` contract that ``AgentCore`` accepts
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
        # Issue #2842 — тулы исполняются в процессе mcp_server, а не
        # здесь: utterance_id хода (register_speaker) едет скрытым
        # аргументом подписанного /mcp/execute, см.
        # llm_adapter.TURN_CONTEXT_ARGS.
        bridge = LLMToolCallAdapter(self, turn_context=self._mcp_turn_context)
        provider = ROSMCPToolProvider(bridge)
        # Feed the 34 manifests from the harness-side catalog. The
        # provider's update_tools() expects the OpenAI-style envelope
        # ({type: function, function: {name, description,
        # parameters}}); build it from ToolRegistry.list_tools()
        # so the LLM-facing surface is the single source of truth.
        # ADR-0149 PR-6: LLM видит dj_set/request_music движка v2, а не compose_music & Co.
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
        # AgentCore consumes the legacy ``discover/execute`` port
        # contract; adapt the core provider so the harness's
        # orchestration layer stays unchanged.
        provider_adapter = adapt_tool_provider(provider)
        # ADR-0150 PR-7b (#3699): каждый тул хода проходит допуск планировщика ресурсов (knowledge.COMPOSITION):
        # совместимое — сразу, «потом» — после события конца занятого блока, исход — событие владельца.
        self._tool_executor = make_tool_executor(self, provider_adapter, self._on_tool_stage)
        self.get_logger().info("✅ PR-7b: тулы хода — через планировщик ресурсов (/dialog/plan_event)")
        return self._tool_executor

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

    # ADR-0066 §6.3 — AV-28: voice_presets (формализация Quest-фраз через
    # LLM) и весь блок ``_resolve_voice_presets_path`` /
    # ``_load_voice_presets`` / ``_resolve_voice_preset`` /
    # ``_resolve_voice_language`` / ``_language_meta`` /
    # ``_language_label`` / ``_language_prompt_section`` /
    # ``_get_formalize_timeout`` / ``_formalize_with_llm`` УДАЛЕНЫ.
    # Формализация пресета переехала в ``grip_pipeline`` супервизора
    # (ADR-0028 §S5, см. ``src/rob_box_supervisor/rob_box_supervisor/
    # grip_pipeline.py`` — там зеркало ``_load_voice_presets`` для
    # LLM-формализации). Личность больше не знает о Quest.

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
                "Для музыки вызывай request_music."
            )
        else:
            parts.append(
                "Для стилизации используй рэп или стихи. "
                "Для музыки вызывай request_music."
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
            "Для музыкального оформления вызывай request_music."
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

    def _ask_identity_if_ambiguous(self, ack: dict) -> None:
        """Переспросить вслух, когда биометрия не решает, кто перед нами.

        Закрывает то, что ADR-0127 отложил: поле ``voice_conflict`` в ack
        нода клала с самого начала, а читать его было некому — «правильное
        поведение продукта и, вероятно, следующий шаг» так и осталось
        следующим шагом. Здесь появляется потребитель, сразу для обоих
        зеркальных случаев.

        **Голос похож, имя другое** (``voice_conflict``, ADR-0127). Живой
        пример из ночного марафона: Саша и Борис звучат для resemblyzer на
        cos=0.846, то есть выше любого рабочего порога слияния. Слить их
        значило бы стереть человека — в базе остался бы один профиль, и
        тот под чужим именем.

        **Имя совпало, голос не дотянул** (``name_twin``, issue #2747).
        Живой пример 22.09.2026: человека перестали узнавать (медиана
        косинуса 0.502 при пороге 0.72), он представился заново — и пара
        профилей «Дэнчик», слитая вручную двумя часами ранее,
        восстановилась за пятнадцать минут разговора.

        В обоих случаях данные уже целы: профиль заведён отдельный, ничего
        не перезаписано (инвариант ADR-0127 — «пока ответа нет, данные
        должны быть целы»). Вопрос нужен, чтобы РЕШЕНИЕ принял человек, а
        не косинус, — склеить два профиля постфактум дёшево
        (``/voice/speaker/merge``), а восстановить стёртую личность нечем.

        Говорим мимо LLM, а не через хинт в следующий ход: ack
        регистрации — событие fire-and-forget, LLM его не видит (тот же
        довод, что у отказа ``register_error`` выше), и к следующей
        реплике повод переспросить уже протухнет.

        Issue #2828: ack приходит, пока ход ещё идёт — LLM после
        ``register_speaker`` дописывает приветствие. Вопрос звучит ПОСЛЕ
        ответа хода, последней репликой (#3765, как вопрос об имени по
        В17), а не вместо него. Ответ человека — кадр ``Identity``
        (``core.identity_question.merge_reply``).
        """
        question = identity_question(ack)
        if not question:
            return
        held = self._queue_identity_question(merge_plan(ack, question))
        self.get_logger().info(
            f"👥 [issue #2747] переспрашиваю про личность: "
            f"twin={bool(ack.get('name_twin'))} "
            f"conflict={bool(ack.get('voice_conflict'))} "
            f"held_until_turn_end={held}"
        )

    def _queue_identity_question(self, plan: dict) -> bool:
        """Посреди хода -- отложить вопрос до конца хода (прозвучит после его
        ответа, ``_run_turn`` finally), вне хода -- сказать сразу. Общая
        точка для #2828 (ack регистрации) и #2908 (отказ регистрации).
        Возвращает ``True``, если отложен."""
        with self._task_lock:
            held = self._run_task is not None
            if held:
                self._after_turn_question = plan
        if not held:
            self._speak_identity_question(plan)
        return held

    def _speak_identity_question(self, plan: Optional[dict]) -> str:
        """Задать вопрос после ответа хода. ``None`` — спрашивать нечего.

        Вопрос склейки (#2828) открывает кадр ``Identity`` (ответ — без
        вейка в окне ``REPLY_WINDOW_S``, #3765); просьба повторить (#2908)
        кадра не открывает. Возвращает ``speech_id`` (``""`` — не озвучен).
        """
        if plan is None:
            return ""
        try:
            speech_id = self._speak_direct(plan["question"])
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"не смог озвучить переспрос ({plan.get('kind')}): {exc!r}"
            )
            return ""
        # Issue #2914 -- в ходе уже есть вопрос о личности: второго не будет
        # (``speaker_turn.plan_identity_question``).
        self._identity_question_asked_in_turn = True
        if speech_id and plan.get("kind") in IDENTITY_CODED_KINDS:
            open_merge_question(self, plan, speech_id)
        return speech_id

    def _on_speaker_result(self, msg: SpeakerResult) -> None:
        """Issue #1077 — результат голосовой биометрии (speaker_id_node).

        ADR-0150 PR-2: типизированный ``SpeakerResult``; словарь
        ``{"is_known": true, "speaker_id": "...", "name": "...",
        "confidence": 0.93}`` (или ``{"is_known": false}``) строит
        ``rob_box_core.speaker_wire`` - единственное место соответствия
        поле <-> ключ. Храним последний результат; _run_turn использует его
        для префикса [Говорит <имя>] / [Говорит: незнакомец] и session lock.
        """
        data = speaker_result_to_payload(msg)
        if data.get("event") in ("registered", "register_error"):
            self._on_register_outcome(data)
            # #3765: исход известен (вопрос, если нужен, уже отложен) — его ждёт settle_identity до вызова LLM.
            self._turn_register_outcome = data
            return
        # Issue #2863 — любой другой служебный ack (ack склейки,
        # переименования) — не результат биометрии и не должен затирать
        # узнанного диктора.
        if data.get("event"):
            return
        self._on_speaker_match(data)

    def _on_register_outcome(self, data: dict) -> None:
        """Ack регистрации от speaker_id_node: ``registered`` (возможно, с
        поводом переспросить) или ``register_error``. Вынесено из
        :meth:`_on_speaker_result` (issue #2913)."""
        # Registration ack — не speaker match.
        if data.get("event") == "registered":
            self.get_logger().info(
                f"✅ [issue 1077] Speaker registered: "
                f"{data.get('name')!r} id={str(data.get('speaker_id', ''))[:8]}"
            )
            self._ask_identity_if_ambiguous(data)
            return
        # Issue #2769 — speaker_id_node отклонил регистрацию: реплика короче
        # MIN_REGISTER_AUDIO_DURATION_SEC, эталон не создан (см.
        # speaker_id_node._do_register / speaker_embeddings.AudioTooShortError).
        # Честный отказ вместо тихого мусора в /data/speakers.db — робот сам
        # просит повторить фразу, а не ждёт, пока LLM додумается переспросить
        # по обрывку ack, который она никогда не видит (register_speaker —
        # fire-and-forget публикация в топик, а не синхронный tool-result).
        if (
            data.get("event") == "register_error"
            and data.get("error") in self._SPOKEN_REGISTER_ERRORS
        ):
            name = data.get("name")
            error = data.get("error")
            if error == "too_short":
                self.get_logger().warning(
                    f"⚠️ [issue #2769] Регистрация '{name}' отклонена — "
                    f"реплика {data.get('duration_s')}с короче требуемых "
                    f"{data.get('min_required_s')}с"
                )
            else:
                # Issue #2829 (ADR-0131 PR-2) — "no_utterance_context"
                # (register_speaker без utterance_id — не должно
                # происходить в проде, только легаси/тесты) или
                # "utterance_not_found" (эмбеддинг для этой фразы не
                # успел появиться за _REGISTER_UTTERANCE_WAIT_SEC).
                # Раньше это молча уходило в _pending_register_name без
                # срока и цепляло "следующую фразу кого угодно" — теперь
                # честный отказ, тот же голосовой ответ, что у #2769.
                self.get_logger().warning(
                    f"⚠️ [issue #2829] Регистрация '{name}' отклонена: "
                    f"{error} (utterance_id={data.get('utterance_id')})"
                )
            # Issue #2908 — отказ приходит, пока ход ещё идёт: LLM дописывает
            # «Рад знакомству, Борис!». Просьба звучит ПОСЛЕ ответа хода
            # (#3765) в согласованной форме — имя услышано, не сохранён голос.
            self.get_logger().info(
                f"🔁 [issue #2908] просьба повторить после ответа хода "
                f"(регистрация '{name}' отклонена: {error})"
            )
            self._queue_identity_question({
                "kind": "register_retry",
                "question": phrases.text("identity.retry_voice"),
            })
        # register_error с неозвучиваемым кодом — молча.

    def _on_speaker_match(self, data: dict) -> None:
        """Результат биометрии одной фразы (не служебный ack)."""
        # Issue #2829 (ADR-0131) — join point: этот результат помечен
        # utterance_id (speaker_id_node считает его от тех же PCM-байт,
        # что видел stt_node). submit() будит _apply_speaker_identity,
        # если она уже ждёт именно эту фразу; если ещё не ждёт — результат
        # ляжет в registry и будет найден при первом resolve().
        utterance_id = data.get("utterance_id")
        if utterance_id:
            self._utterance_speaker.submit(str(utterance_id), data)
            note_backlog_voice(self, str(utterance_id), data)  # #3695: голос фразы бэклога
        # Issue #2863 — фраза, которую биометрия не оценила (мало речи /
        # нет эмбеддинга; частый случай — шум, который STT потом отбросит
        # как пустой), не сбрасывает последнего узнанного. Для СВОЕЙ фразы
        # она остаётся is_known=false (registry выше, #2829 — имя не
        # наследуется), а «кто сейчас» не трогаем.
        if data.get("inconclusive"):
            return
        with self._speaker_lock:
            self._current_speaker = data

    def _current_acquaintance(self) -> Optional[Acquaintance]:
        """Issue #2440 — знакомый из последнего результата биометрии.

        Читает ``_current_speaker`` (результат speaker_id_node) и, если
        спикер опознан, строит ``Acquaintance`` со стабильным
        биометрическим id. Неизвестный спикер (или отсутствие id) → None:
        профиль под per-session tag создавать нельзя.
        """
        with self._speaker_lock:
            sp = dict(self._current_speaker)
        if not sp.get("is_known"):
            return None
        sid = str(sp.get("speaker_id") or "").strip()
        if not sid:
            return None
        name = sanitize_speaker_name(str(sp.get("name") or "")) or None
        epithet = str(sp.get("epithet") or "").strip() or None
        try:
            confidence = float(sp.get("confidence") or 0.0)
        except (TypeError, ValueError):
            confidence = None
        return Acquaintance(
            id=sid,
            name=name,
            epithet=epithet,
            confidence=confidence,
        )

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

    def _on_dialogue_control(self, msg: String) -> None:
        """ADR-0066 — обработчик ``/dialogue/control`` от avatar_supervisor.

        Контракт (ADR-0066 §2.1):

        ::

            {"action": "pause"|"resume", "reason": str, "ts_s": float}

        Любой другой ``action`` или невалидный JSON — WARNING в лог и
        no-op (ack НЕ шлём — решение принято в карточке t_d058dc6f
        «невалидный JSON не ACK-ается»). Успешная команда → ack
        публикуется **после** изменения FSM, через
        ``_publish_control_ack``.
        """
        try:
            payload = json.loads(msg.data or "{}")
            action = str(payload.get("action") or "").strip().lower()
            reason = str(payload.get("reason") or "")
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning(
                f"⚠️ [ADR-0066] /dialogue/control: invalid JSON {msg.data!r}"
            )
            return
        if action == "pause":
            self._apply_operator_pause(reason)
        elif action == "resume":
            self._apply_operator_resume()
        else:
            self.get_logger().warning(
                f"⚠️ [ADR-0066] /dialogue/control: unknown action {action!r}"
            )
            return
        self._publish_control_ack()

    def _apply_operator_pause(self, reason: str) -> None:
        """ADR-0066 — перевести FSM в SILENCED по команде оператора.

        Идемпотентно: повторный pause, когда FSM уже в SILENCED,
        не меняет ``_paused_at_ms`` (чтобы напоминание в супервизоре
        не сбивалось — см. §2.5 «edge-cases»).
        """
        state = self._dsm.current_state
        if state == DialogueStateKind.SILENCED:
            return
        self._dsm.on_event(DialogueEvent.SILENCE_COMMAND)
        self._paused_at_ms = int(time.monotonic() * 1000)
        self._pause_reason = reason
        self._publish_state()
        self.get_logger().info(
            f"⏸️ [ADR-0066] pause reason={reason!r} (was {state.name})"
        )

    def _apply_operator_resume(self) -> None:
        """ADR-0066 — вывести FSM из SILENCED в IDLE по команде оператора.

        Resume из любого другого состояния — no-op + ack с текущим
        состоянием (защита от гонок, §2.5).
        """
        state = self._dsm.current_state
        if state != DialogueStateKind.SILENCED:
            return
        self._dsm.on_event(DialogueEvent.UNSILENCE)
        self._paused_at_ms = None
        self._pause_reason = ""
        self._publish_state()
        self.get_logger().info("▶️ [ADR-0066] resume")

    def _publish_control_ack(self) -> None:
        """ADR-0066 — публикация ack на ``/dialogue/control_ack``.

        Поля: ``state`` (``paused`` для SILENCED, иначе ``state.name.lower()``),
        ``since_ms`` (``_paused_at_ms`` для паузы, иначе monotonic() * 1000),
        ``ts_s``, ``reason`` (только в паузе).
        """
        state = self._dsm.current_state
        is_paused = state == DialogueStateKind.SILENCED
        payload = {
            "state": "paused" if is_paused else state.name.lower(),
            "since_ms": self._paused_at_ms if is_paused
                        else int(time.monotonic() * 1000),
            "ts_s": time.time(),
            "reason": self._pause_reason if is_paused else "",
        }
        out = String()
        out.data = json.dumps(payload, ensure_ascii=False)
        self._dialogue_control_pub.publish(out)

    def _on_stt(self, msg: UtteranceMsg) -> None:
        """STT text admission — Issue #2628 / ADR-0021 R1 refactor.

        Drives the :class:`rob_box_voice.core.stt_admission.SttAdmission`
        pipeline. The 12 inline branches of the predecessor (CC=54) are
        now one ``evaluate()`` call plus a 7-line dispatch tail; CC budget
        dropped to ≤15 (ADR-0021 R1). Pure parsing helper
        (:func:`parse_speaker_event`) lives in ``core/`` and is unit-tested
        without ROS2.

        ADR-0150 PR-2: вход - типизированный ``Utterance``; источник,
        ``chat_id`` и ``utterance_id`` приходят полями, префикс ``[TG:id]``
        в тексте и склейка id по тексту (#2862) больше не нужны.

        Lock order documented in :class:`_DialogueSttHost` docstring.
        """
        raw_text = (msg.text or "").strip()
        if not raw_text:
            return
        # Контракт с координатором: «Клод …» из Telegram он читает из этой строки
        # (до ADR-0150 PR-5 её писал command_node, читавший каждую реплику).
        self.get_logger().info(f"🎤 STT: {log_label(msg)}")
        text = raw_text
        from_tg = msg.source == UtteranceMsg.SOURCE_TELEGRAM and bool(msg.chat_id)
        tg_chat_id = int(msg.chat_id) if from_tg else None
        # Issue #2829/#2862 (ADR-0131) -- id фразы приходит в самом Utterance
        # (у Telegram и оператора его нет -> None).
        utterance_id = msg.utterance_id or None
        if tg_chat_id is not None:
            # Issue #1195 — store chat id for echo routing.
            self._active_tg_chat_id = tg_chat_id
        # Issue #1077 — one STT text ↔ one speaker event (pop on read).
        speaker_event = self._speaker_by_text.pop(raw_text, None)
        speaker_tag, speaker_duration_s = parse_speaker_event(speaker_event)
        state = self._dsm.current_state
        was_idle = state == DialogueStateKind.IDLE
        # Lazy init so legacy test harness (``object.__new__`` without
        # ``__init__``) can drive ``_on_stt`` without owning a pipeline
        # instance. Production code gets the pipeline at __init__ via
        # ``self._stt_admission = DefaultSttAdmission(...)``.
        admission = getattr(self, "_stt_admission", None)
        if admission is None:
            admission = DefaultSttAdmission(logger=self.get_logger())
            self._stt_admission = admission
        ctx = SttContext(
            raw=raw_text,
            text=text,
            text_lower=text.lower(),
            tg_chat_id=tg_chat_id,
            speaker_tag=speaker_tag,
            speaker_duration_s=speaker_duration_s,
            state_name=state.name,
            is_silenced=(state == DialogueStateKind.SILENCED),
            wake_words=tuple(self._wake_words),
            heard=heard_from(msg, now=time.time(), doa=getattr(self, "_doa", None),
                             person_ref=person_ref_from(getattr(self, "_utterance_speaker", None), utterance_id),
                             faces=getattr(self, "_faces", None)),
            skip_counter=self._llm_skipped_counter,
        )
        host = _DialogueSttHost(self, utterance_id)
        dispatch = {
            "was_idle": was_idle,
            "speaker_tag": speaker_tag,
            "speaker_duration_s": speaker_duration_s,
            "from_tg": bool(tg_chat_id is not None),
            "utterance_id": utterance_id,
        }
        # Issue #3176 — заказ по имени, которого нет в базе мелодий, роутер
        # возвращает в приём с того же места (после MediaCommandStep).
        host.media_miss = functools.partial(
            self._resume_stt_after_media, ctx, host, dispatch
        )
        # ADR-0150 PR-6 (В19) — спорный стоп оказался частью команды: приём с шага после Tier-1.
        host.tier1_resume = functools.partial(
            self._resume_stt_after_media, ctx, host, dispatch, step="tier1"
        )
        outcome = self._stt_admission.evaluate(ctx, host)
        self._finish_stt_admission(outcome, text, dispatch)

    def _finish_stt_admission(
        self, outcome: SttOutcome, text: str, dispatch: Dict[str, Any]
    ) -> None:
        """Хвост приёма: не PASS — сводка пропусков, PASS — ход LLM."""
        if outcome.kind is not SttOutcomeKind.PASS:
            self._stage_admission_closed(dispatch.get("utterance_id"), outcome)
            self._maybe_log_skip_summary()
            return
        # PASS — dispatch the cleaned phrase to LLM.
        new_ctx = outcome.new_context
        clean = new_ctx.text if new_ctx is not None else text
        self._dispatch_cleaned(clean=clean, **dispatch)

    def _resume_stt_after_media(
        self,
        ctx: SttContext,
        host: "_DialogueSttHost",
        dispatch: Dict[str, Any],
        clean: str,
        step: str = MediaCommandStep.name,
    ) -> None:
        """Issue #3176 — заказ по имени не нашёлся: реплика идёт в LLM.

        Досчитывает шаги приёма после ``step`` (по умолчанию :class:`MediaCommandStep`:
        backlog, barge-in, FSM + «думаю»-SFX) и отдаёт реплику ходу LLM так же, как
        это сделал бы ``_on_stt`` без роутера. Зовётся из loop'а плана
        роутера — как и синтетические ретраи гуардов (``_dispatch_turn``).
        ADR-0150 PR-6 (В19): ``step="tier1"`` — спорный стоп оказался частью команды.
        """
        self.get_logger().info(
            f"🎛️ [stt-admission] после {step}: реплика продолжает приём: {clean[:60]!r}"
        )
        outcome = self._stt_admission.resume_after(
            step, ctx.with_text(clean), host
        )
        self._finish_stt_admission(outcome, clean, dispatch)

    # -- STT admission helpers -------------------------------------------
    # These three helpers keep ``_on_stt`` at CC≤15 (ADR-0021 R1). The
    # orchestrator (:class:`rob_box_voice.core.stt_admission.SttAdmission`)
    # owns the *order*; the helpers own the *side effects* the orchestrator
    # delegates via the ``SttAdmissionHost`` protocol.

    def _dispatch_cleaned(
        self,
        *,
        clean: str,
        was_idle: bool,
        speaker_tag: Optional[str],
        speaker_duration_s: float,
        from_tg: bool,
        utterance_id: Optional[str] = None,
    ) -> None:
        """Tail of the old ``_on_stt`` after the 12-branch pipeline.

        Issue #2628 — extracted because the orchestrator stops at the
        LAST PASS step (typically :class:`DispatchTriggerStep`, which
        runs the FSM transitions). Everything below builds the final
        user-turn text: verbose log, ``_dispatch_turn``. «Услышал»/«думаю» - рефлексы
        ``addressed`` (допуск) и ``thinking`` (перед LLM), отдельного звука нет.

        ADR-0150 PR-5 — бэклог неадресованных фраз в user-turn больше не
        вставляется: ≤ 1 свежая фраза того же источника идёт в снимок
        (``recent_unaddressed`` в ``_build_dynamic_system_context``).
        """
        raw_user_command = clean
        if self._verbose_llm:
            self.get_logger().info(f"📥 LLM INPUT: {clean[:200]!r}")
        # Issue #live 12:45 — guards see the *raw* command.
        self._dispatch_turn(
            clean,
            was_idle=was_idle,
            raw_user_command=raw_user_command,
            speaker_tag=speaker_tag,
            speaker_duration_s=speaker_duration_s,
            from_tg=from_tg,
            utterance_id=utterance_id,
        )

    def _on_tts_finished(self, msg: String) -> None:
        """Освободить ожидающих конца реплики (эффекты хода).

        Учёт батчей для уборки музыки (#980/#992) удалён в ADR-0150 PR-7c ч.4б (#3700): музыку снимает её
        владелец по явному стопу, а не конец речи.
        """
        self._effects.handle_tts_finished(msg.data or "")

    def _on_tts_provider_state(self, msg: String) -> None:
        """Issue #1229 — обновить фактический провайдер TTS (от tts_node).

        tts_node публикует JSON {"provider": str, "voice": str,
        "default_voice": str, "reason": str} после старта, при фолбеке
        провайдера (квота/сеть) и после каждого успешного синтеза.
        Храним фактического провайдера и голос — LLM-контекст [TTS]
        строится по ним (голоса РЕАЛЬНОГО провайдера, а не номинального).

        Issue #2175 — смена TTS-провайдера (yandex→minimax fallback или
        обратно) тоже инвалидирует MiniMax's кэш system-context, поэтому
        ресетим guard-флаг.
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
        # Issue #2175 — ресетим guard-флаг после смены провайдера
        # (guard сбрасывается на любой смене провайдера/голоса).
        self._system_regurgitate_retry_used = False
        self.get_logger().info(
            f"🎙️ [issue 1229] TTS actual provider → '{self._actual_tts_provider}' "
            f"(voice: {actual_voice}, reason: {payload.get('reason')})"
        )

    def _on_music_state(self, msg: String) -> None:
        """Снимок плеера ``/voice/music/state`` (issue #3133, ADR-0141).

        Единственный источник «играет» для диалога: ``_music_playing_now``,
        гуард #3125 и Bug-C ретрай-промпт читают ``_music_player_state``.
        """
        snapshot = parse_music_state(msg.data)
        if snapshot is None:
            return
        self._music_player_state = snapshot
        self._music_state_mem().observe_state(snapshot)
        plan_on_music(self, snapshot)

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


    def _dispatch_turn(
        self,
        user_input: str,
        was_idle: bool = False,
        is_action_claim_retry: bool = False,
        is_synthetic: bool = False,
        raw_user_command: str | None = None,
        speaker_tag: str | None = None,
        speaker_duration_s: float = 0.0,
        from_tg: bool = False,
        occasion: "Occasion | None" = None,
        utterance_id: str | None = None,
    ) -> None:
        # ADR-0101 §3.1 / #2536 — повод (ещё не подключён в wake-gate,
        # PR-B…F); сигнатура принимает ``occasion``, логирует только при
        # явной передаче. occasion=None → байт-в-байт прежнее поведение.
        if occasion is not None:
            self.get_logger().info(
                f"🎯 [occasion] started turn occasion={occasion.kind}"
            )
        # Issue #1160 — Prometheus metrics: новый диалог из IDLE (первый
        # wake-word) открывает сессию; DIALOGUE_END / timeout закроет её
        # histogram'ом voice_session_duration_seconds.
        if was_idle and self._session_started_at is None:
            self._session_started_at = time.monotonic()
            self._session_end_reason = "success"
        # Issue #3145 — синтетический ретрай значит «гуард отверг ответ
        # этого хода»: ответ уходит из истории ДО ретрая, у всех гуардов.
        if is_synthetic:
            self._retract_rejected_reply()

        asyncio.run_coroutine_threadsafe(
            self._run_turn(
                user_input,
                is_action_claim_retry=is_action_claim_retry,
                is_synthetic=is_synthetic,
                raw_user_command=raw_user_command,
                speaker_tag=speaker_tag,
                speaker_duration_s=speaker_duration_s,
                from_tg=from_tg,
                utterance_id=utterance_id,
                # Issue #2835 — поколение сессии, в котором родился ход
                # (для ретрая изнутри хода — поколение родителя).
                session_epoch=self._session_epoch_gate().epoch_for_dispatch(),
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

    async def _resolve_speaker_for_utterance(
        self, utterance_id: Optional[str]
    ) -> None:
        """Issue #2829 (ADR-0131) — резолвнуть ``_current_speaker`` для
        ОДНОЙ конкретной фразы и запомнить, что она уже резолвнута.

        Идемпотентность (``_last_resolved_utterance_id``) существует
        ради #2828 (``identity_question.merge_reply``): та функция должна
        видеть резолвнутого текущего диктора ДО того, как
        ``_apply_speaker_identity`` вызывается штатно из
        ``_prepare_user_input_context`` -- без кеша второй вызов ждал бы
        ``speaker_resolve_timeout_sec`` ЗАНОВО (utterance уже
        разрешилась или уже протухла, полученный результат не изменится,
        а задержка хода удвоилась бы впустую).

        ``utterance_id=None`` (синтетический/ретрай-ход) -- no-op,
        ``_current_speaker`` остаётся тем, чем было.
        """
        if not utterance_id:
            return
        if utterance_id == getattr(self, "_last_resolved_utterance_id", None):
            return
        resolved = await self._utterance_speaker.resolve(
            utterance_id, self._speaker_resolve_timeout_sec
        )
        if resolved is None:
            self.get_logger().warning(
                f"👤 [issue #2829] speaker biometry timeout for "
                f"utterance={utterance_id} "
                f"({self._speaker_resolve_timeout_sec:.1f}s) — unknown, "
                "NOT reusing previous speaker"
            )
        with self._speaker_lock:
            self._current_speaker = resolved if resolved is not None else {
                "is_known": False
            }
        self._last_resolved_utterance_id = utterance_id

    async def _apply_speaker_identity(
        self,
        user_input: str,
        speaker_context: Optional[str],
        utterance_id: Optional[str] = None,
    ) -> str:
        """Собеседник этой фразы: биометрия по ``utterance_id`` → ``SpeakerRef`` (адаптер) и ветка tentative.

        ADR-0150 §7.3/§7.5 (#3695): текст человека LLM получает как есть — без префиксов собеседника (были
        ``Spkr``/``Speaker`` в квадратных скобках); кто говорит — структурный раздел ``speaker`` снимка хода (имя только при ``confirmed``).
        ``speaker_context`` не используется (сигнатура вызывающих).
        """
        # Issue #2829 (ADR-0131) — резолвим _current_speaker ПО ЭТОЙ фразе
        # (join по utterance_id). Идемпотентно: если ``_prepare_user_input_
        # context`` уже резолвнула этот же utterance_id (нужно #2828 —
        # ответ на переспрос проверяется ДО вызова этого метода), второй
        # resolve() не ждёт ещё раз (см. ``_resolve_speaker_for_utterance``).
        await self._resolve_speaker_for_utterance(utterance_id)
        if not utterance_id:
            # Нет utterance_id (синтетический/ретрай-ход без новой
            # фразы -- babble-retry, DJ auto, action-claim retry, ...):
            # новой аудио-фразы для сверки нет, читаем последний
            # РАЗРЕШЁННЫЙ снимок как есть (issue #2829 гонка тут
            # неприменима -- ждать нечего).
            pass
        with self._speaker_lock:
            sp = dict(self._current_speaker)
        note_interlocutor(self, sp, utterance_id, user_input)
        if sp.get("is_known"):
            name = str(sp.get("name") or "")
            conf = float(sp.get("confidence") or 0.0)
            sid = str(sp.get("speaker_id") or "")[:8]
            # Issue #1787 — отдаём реплику speaker_id_node: он копит по ней
            # темы и подбирает внутреннюю кличку. Публикуем СЫРОЙ текст,
            # как сказал человек.
            # Делается до проверки имени: эпитет нужен как раз тем, у кого
            # имя мусорное или отсутствует.
            self._publish_speaker_observation(
                str(sp.get("speaker_id") or ""), user_input
            )
            if name:
                # live 12.08: мусорные имена в БД ("Null", "None") — не имена; снимок их не пропускает
                # (sanitize_speaker_name в _build_dynamic_system_context).
                if not sanitize_speaker_name(name):
                    self.get_logger().warning(
                        f"👤 [issue 1077] Ignoring invalid speaker name: {name!r} "
                        f"(treating as unknown)"
                    )
                    return user_input
                self.get_logger().info(
                    f"👤 [issue 1077] Speaker: {name!r} conf={conf:.2f}"
                )
            else:
                # Issue #2809 -- speaker_id_node уже подавил имя само
                # (payload {"is_known": True, "name": None,
                # "tentative_name"/"tentative_conf"/"tentative_kind": ...}):
                # is_name_confident() решила "зона сомнения" -- match
                # похож на кого-то из галереи, но недостаточно, чтобы
                # называть человека этим именем как факт. is_known=True
                # тут остаётся полезным сигналом для эпитета/счётчика
                # реплик (уже учтён выше через
                # _publish_speaker_observation). Дальше -- переспрос про
                # tentative-личность (issue #2809, продолжение после
                # ревью координатора): _handle_tentative_speaker решает,
                # разрешён ли ещё один вопрос в этой сессии
                # (identity_question_session_gap_sec), был ли уже ответ
                # ("да"/"нет"), и либо переиспользует confident-путь
                # (после словесного "да"), либо ставит кандидата на вопрос
                # (правило В17 -- после хода, core/speaker_turn); имени-
                # гипотезы в снимке хода нет (#3695).
                user_input = self._handle_tentative_speaker(
                    sp, user_input, utterance_id
                )
        return user_input

    # -- Issue #2809 (продолжение): переспрос про tentative-личность --------
    #
    # Не второй механизм рядом с PR #2798 (_ask_identity_if_ambiguous) --
    # тот реагирует на ack регистрации (event="registered", синхронный
    # fire-and-forget сразу после явного register_speaker), а здесь нет
    # никакого ack: passive identify() просто идёт каждой репликой.
    # Issue #2888: вопрос ОДИН раз за сессию задаёт сам робот, «если
    # уместно» на усмотрение LLM не срабатывало (run 35903232434). С #3695
    # он звучит после ответа хода (правило В17, core/speaker_turn), а имени-
    # гипотезы LLM не видит (раздел speaker снимка).
    # Ответ читается на СЛЕДУЮЩЕЙ реплике простой эвристикой
    # (classify_identity_confirmation) -- без NLU, без второго диалогового
    # состояния поверх обычного turn-цикла.

    def _is_reply_after_question(
        self, state: dict, utterance_id: Optional[str]
    ) -> bool:
        """Issue #2914 -- ход несёт НОВУЮ реплику человека, пришедшую после
        вопроса, а не хвост хода, в котором вопрос задан.

        Сразу после вопроса ``_run_turn`` может запустить ход без новой
        реплики (``utterance_id=None``): дренаж S7 фраз, пришедших ПОКА
        шёл ход, или синтетический ретрай гарда. Их текст ответом не
        является (run 35923951507: ``answer=False`` через 24 мс после
        «Саша, это ты?», настоящее «нет» уже не читалось). Вопрос,
        заданный на ходе без ``utterance_id``, -- прежнее поведение.
        """
        asked_on = state.get("asked_utterance_id")
        if not asked_on:
            return True
        return bool(utterance_id) and utterance_id != asked_on

    def _resolve_pending_tentative_answer(
        self,
        state: dict,
        tentative_name: Optional[str],
        confidence: float,
        user_input: str,
        utterance_id: Optional[str] = None,
    ) -> None:
        """Если в этой сессии уже спрашивали и ответа ещё нет -- прочитать
        ТЕКУЩУЮ (первую после вопроса) реплику как да/нет.

        Issue #2809: решение принимается СРАЗУ и окончательно на первой
        же следующей реплике -- неоднозначный ответ (``None`` от
        ``classify_identity_confirmation``) трактуется как отказ
        (``False``), а не как "жду ещё". Без этого вопрос повис бы до
        конца сессии в ожидании явного "да"/"нет" и мог бы случайно
        сработать на совершенно не связанной более поздней реплике,
        где просто встретилось слово "да".
        """
        if not (state["asked"] and state["confirmed"] is None):
            return
        if not self._is_reply_after_question(state, utterance_id):
            self.get_logger().info(
                "👤 [issue #2914] identity answer NOT read: ход без новой "
                f"реплики человека (utterance_id={utterance_id!r})"
            )
            return
        answer = classify_identity_confirmation(user_input)
        state["confirmed"] = bool(answer)
        self.get_logger().info(
            f"👤 [issue 2809] identity answer read: answer={answer!r} "
            f"-> confirmed={state['confirmed']}"
        )
        if answer and tentative_name:
            state["name"] = tentative_name

    def _confirm_tentative_speaker(
        self,
        speaker_id: str,
        name: str,
        user_input: str,
        utterance_id: Optional[str] = None,
    ) -> str:
        """Словесное "да, это я" -- дальше ведём себя так, как будто
        биометрия сама уверенно опознала: мутируем ``_current_speaker``
        (тот же снимок следующим шагом читает
        ``_build_dynamic_system_context`` этой же реплики): имя попадает в раздел ``speaker`` снимка как у
        уверенной биометрии -- вторую копию её логики не заводим.

        Issue #6 (ADR-0123 §6, #2771): это ЛОКАЛЬНАЯ мутация снимка
        dialogue_node, наружу (``/voice/speaker/result``, откуда читает
        vision_face_node) она не публикуется -- гипотеза не может
        просочиться в слияние лиц раньше настоящего подтверждения
        биометрией.
        """
        with self._speaker_lock:
            self._current_speaker["name"] = name
            self._current_speaker.pop("tentative_name", None)
            self._current_speaker.pop("tentative_conf", None)
            self._current_speaker.pop("tentative_kind", None)
        state = tentative_states(self).get(speaker_id)
        if state is not None and state.get("growth_registered"):
            # Отдельная строка для реплик ПОСЛЕ подтверждения -- по ней
            # E2E (n704) отличает "имя из пережившего паузу состояния" от
            # "LLM повторила имя из истории".
            self.get_logger().info(
                f"👤 [issue 2809] Speaker verbal confirmation reused: "
                f"{name!r} (id={speaker_id[:8]})"
            )
            return user_input
        self.get_logger().info(
            f"👤 [issue 2809] Speaker confirmed verbally: {name!r} "
            f"(id={speaker_id[:8]})"
        )
        if state is not None:
            self._publish_confirmed_identity_growth(
                speaker_id, name, utterance_id
            )
            state["growth_registered"] = True
        return user_input

    def _publish_confirmed_identity_growth(
        self, speaker_id: str, name: str, utterance_id: Optional[str] = None
    ) -> None:
        """Issue #2809/#2757 -- рост галереи подтверждённого профиля.

        Топик тот же, что у LLM-тула ``register_speaker``
        (``/voice/speaker/register``), но с ``purpose: "growth"`` (issue
        #2906): speaker_id_node ведёт такой запрос НЕ через
        ``_do_register``/``register_or_merge``, а через
        ``_do_confirmed_growth`` -- дописывает эмбеддинг в ЭТОТ профиль по
        правилам роста владельца (#2833), новый якорь не заводит и
        ``register_error`` не публикует. Поэтому короткое «да, это я» не
        превращается в «Не расслышал» -- та реплика остаётся только для
        регистрации по просьбе LLM (``_SPOKEN_REGISTER_ERRORS``).

        Issue #2829 (ADR-0131 PR-2) -- ``utterance_id`` — id ЭТОЙ самой
        реплики (словесное "да, это я"), передаётся вызывающим кодом
        (``_confirm_tentative_speaker``, тот же ход). speaker_id_node
        ищет эмбеддинг ИМЕННО этой фразы (см.
        ``_on_register_request``/``_find_embedding_for_utterance``), а
        не "самую свежую за 30с от кого угодно" — старое поведение,
        которое эта строка раньше неявно подразумевала (комментарий
        "эмбеддинг ... из САМОЙ свежей реплики" ниже был честен про
        старый механизм; теперь он не нужен: id есть).
        """
        pub = getattr(self, "_speaker_register_pub", None)
        if pub is None:
            return
        # Issue #2906 -- ``purpose: "growth"``: это НЕ регистрация, а рост
        # галереи уже известного (только что подтверждённого) профиля.
        # speaker_id_node ведёт его отдельным путём (без register_or_merge,
        # без нового якоря, без register_error): раньше короткое «да, это
        # я» (1.65с) отклонялось как too_short, и робот говорил «Не
        # расслышал», хотя всё расслышал.
        payload = {"name": name, "speaker_id": speaker_id, "purpose": "growth"}
        if utterance_id:
            payload["utterance_id"] = utterance_id
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        pub.publish(msg)
        self.get_logger().info(
            f"🌱 [issue #2809/#2757] запрошен рост галереи после "
            f"словесного подтверждения: name={name!r} "
            f"speaker_id={speaker_id[:8]}"
        )

    def _configure_face_voice_hint(self) -> None:
        """Apply configured face-hint parameters to the identity seam."""
        if not self._face_voice_hint_enabled:
            return
        self._identity.configure_face_hint(
            buffer_capacity=self._face_voice_hint_buffer_capacity,
            high_threshold=self._face_voice_hint_high_threshold,
            low_threshold=self._face_voice_hint_low_threshold,
            window_sec=self._face_voice_hint_window_sec,
        )
    def _face_hint_confirmation(
        self,
        *,
        state: dict,
        tentative_name: Optional[str],
        full_sid: str,
        user_input: str,
        utterance_id: Optional[str],
    ) -> bool:
        """ADR-0139 — apply a fresh high-confidence face hint, if present."""
        if not (
            getattr(self, "_face_voice_hint_enabled", False)
            and tentative_name
            and not state.get("asked")
        ):
            return False
        face_obs = self._identity.recent_face_observation_by_name(
            tentative_name,
            window_sec=getattr(self, "_face_voice_hint_window_sec", 30.0),
        )
        if face_obs is None or face_obs.confidence_band != "high":
            return False
        state["asked"] = True
        state["confirmed"] = True
        state["name"] = tentative_name
        self.get_logger().info(
            "👤 [issue #3024 ADR-0139] voice tentative suppressed "
            "by recent face hint (name=%r, age=%.1fs, sim=%.3f); "
            "confirming as %r"
            % (face_obs.name, face_obs.age_sec(), face_obs.similarity, state["name"])
        )
        return True
    def _tentative_identity_parts(
        self, sp: dict
    ) -> tuple[str, str, Optional[str]]:
        """Normalize tentative speaker payload before dispatch."""
        full_sid = str(sp.get("speaker_id") or "")
        tentative_kind = str(sp.get("tentative_kind") or "")
        if not full_sid or tentative_kind not in ("single", "contested"):
            return "", "", None
        tentative_name = None
        if tentative_kind == "single":
            tentative_name = sanitize_speaker_name(
                str(sp.get("tentative_name") or "")
            ) or None
        return full_sid, tentative_kind, tentative_name

    def _finish_tentative_identity(
        self,
        state: dict,
        full_sid: str,
        tentative_kind: str,
        tentative_name: Optional[str],
        sp: dict,
        user_input: str,
        utterance_id: Optional[str],
    ) -> str:
        """Apply the existing confirmation/question tail for tentative speakers."""
        if state.get("confirmed") and state.get("name"):
            return self._confirm_tentative_speaker(
                full_sid, state["name"], user_input, utterance_id
            )
        if state.get("confirmed") is False:
            return user_input
        if not state["asked"]:
            # ADR-0150 В17 (#3695): не спрашивать сейчас — кандидат; вопрос решается после хода
            # (core/speaker_turn.plan_identity_question): не на команду, не раньше N-й фразы, один раз.
            self._identity_ask_candidate = AskCandidate(
                full_sid, tentative_kind, tentative_name, utterance_id, user_input)
            self.get_logger().info(
                f"👤 [issue 2809] identity question candidate: "
                f"kind={tentative_kind} (id={full_sid[:8]})"
            )
        return user_input

    def _handle_tentative_speaker(
        self, sp: dict, user_input: str, utterance_id: Optional[str] = None
    ) -> str:
        """Диспетчер переспроса для tentative-случая (см. блок выше)."""
        full_sid, tentative_kind, tentative_name = self._tentative_identity_parts(sp)
        if not full_sid:
            return user_input

        state = tentative_states(self).fresh(full_sid, time.monotonic())
        intro = getattr(self, "_turn_self_intro", None)
        if intro:
            return self._tentative_with_self_intro(
                state, full_sid, tentative_name, intro, user_input,
                utterance_id,
            )
        self._resolve_pending_tentative_answer(
            state,
            tentative_name,
            float(sp.get("tentative_conf") or sp.get("confidence") or 0.0),
            user_input,
            utterance_id,
        )
        self._face_hint_confirmation(
            state=state,
            tentative_name=tentative_name,
            full_sid=full_sid,
            user_input=user_input,
            utterance_id=utterance_id,
        )
        return self._finish_tentative_identity(
            state,
            full_sid,
            tentative_kind,
            tentative_name,
            sp,
            user_input,
            utterance_id,
        )
    def _tentative_with_self_intro(
        self,
        state: dict,
        full_sid: str,
        tentative_name: Optional[str],
        intro: str,
        user_input: str,
        utterance_id: Optional[str],
    ) -> str:
        """Issue #2925 -- человек назвал себя в этой же реплике.

        Спрашивать «<Имя>, это ты?» (#2888) или «Как тебя зовут?» тут
        не о чем -- ответ уже прозвучал. То же имя, что у гипотезы, --
        это подтверждение, как словесное «да, это я» (#2809: снимок и
        рост галереи). ДРУГОЕ имя -- вопрос не задаётся, реплика идёт в
        регистрацию (:meth:`_register_self_intro`), а похожий голос
        разбирает ADR-0127/#2828 по ack («вы разные люди?»).
        """
        if tentative_name and same_person_name(intro, tentative_name):
            state["asked"] = True
            state["confirmed"] = True
            state["name"] = tentative_name
            return self._confirm_tentative_speaker(
                full_sid, tentative_name, user_input, utterance_id
            )
        self.get_logger().info(
            f"👤 [issue #2925] вопрос о личности НЕ задан: человек назвал "
            f"себя {intro!r} (гипотеза биометрии: {tentative_name!r}, "
            f"id={full_sid[:8]}) -- путь регистрации"
        )
        return user_input

    def _note_self_intro(
        self, user_input: str, utterance_id: Optional[str]
    ) -> None:
        """Issue #2925 -- имя, которым человек назвал себя в реплике хода
        (:func:`extract_self_intro_name`). Только для живой реплики с
        ``utterance_id``: регистрировать без неё нечего (ADR-0131)."""
        # #3695: голое имя в ответ на «Как тебя зовут?» (кадр Identity) — тоже представление
        intro = introduced_name(self, user_input) if utterance_id else None
        self._turn_self_intro = intro
        if intro:
            self.get_logger().info(
                f"👤 [issue #2925] самопредставление в реплике: {intro!r} "
                f"(utterance_id={utterance_id})"
            )

    def _register_self_intro(self, utterance_id: Optional[str]) -> None:
        """Issue #2925 -- явное представление регистрирует голос само, не
        дожидаясь, вызовет ли LLM ``register_speaker`` (класс #2406: на
        роботе «привет я борис …» -- приветствие по имени без тула).

        Тот же запрос, что шлёт тул (``/voice/speaker/register`` с
        ``utterance_id`` этой реплики), тот же ack и те же пути после
        него: ADR-0127 (голос похож, имя другое -- отдельный профиль и
        вопрос #2828), #2863 («уже знаю»), #2908 (отказ). Тул в этом ходе
        получает ``intro_registered`` и второй запрос не шлёт
        (``register_speaker_gate``). Не шлём, если реплика уверенно узнана
        под этим же именем (или только что подтверждена, #2809): нечего
        регистрировать. Исход (ack) ход ждёт до вызова LLM (#3765,
        ``identity_question.settle_identity``).
        """
        intro = getattr(self, "_turn_self_intro", None)
        pub = getattr(self, "_speaker_register_pub", None)
        if not intro or not utterance_id or pub is None:
            return
        known = self._confident_speaker_name(utterance_id)
        # Уверенно узнан под этим же именем -- регистрировать нечего; под
        # другим -- второй профиль не заводится (#3801).
        if settled_by_voice(self, intro, known):
            return
        msg = String()
        msg.data = json.dumps(
            {"name": intro, "utterance_id": utterance_id}, ensure_ascii=False
        )
        pub.publish(msg)
        self._turn_intro_registered = True
        self.get_logger().info(
            f"📝 [issue #2925] регистрация по самопредставлению: "
            f"name={intro!r} utterance_id={utterance_id} "
            f"(узнан по голосу как: {known!r})"
        )

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
        """Спросить у LLM кличку (2–4 слова, #2887) и вернуть её speaker_id_node.

        Одноразовый запрос мимо диалогового цикла: история разговора сюда
        не идёт (кличка не должна зависеть от текущего контекста робота),
        инструменты не подключаются, ``max_tokens`` мал — нужна короткая
        кличка. 48 токенов (было 16 под одно слово): русский текст —
        2–4 символа на токен, кличка до 48 символов плюс запас на кавычки
        и подпись, иначе ответ обрезается посреди слова. Любая ошибка провайдера тихо оставляет словарную кличку:
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
                    settings=LLMSettings(max_tokens=48, temperature=1.0),
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

        proposal, why = epithets.check_llm_epithet(getattr(response, "content", ""))
        if not proposal:
            self.get_logger().info(
                f"🔤 [issue 1787] ответ LLM не похож на кличку "
                f"({str(getattr(response, 'content', ''))[:40]!r}, причина={why}) — "
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
    # Issue #1544/#3161 (ADR-0141) — раздел ``music_state`` снимка хода строится ТОЛЬКО из того, что публикует
    # плеер (``/voice/music/state`` + имя из ``/voice/music/form``): ``rob_box_dialog.music_state`` (#3696).

    def _music_state_mem(self) -> MusicStateMemory:
        """Память о снимках плеера (лениво — для стабов из ``__new__``)."""
        memory = getattr(self, "_music_state_memory", None)
        if memory is None:
            memory = self._music_state_memory = MusicStateMemory()
        return memory

    def _build_dynamic_system_context(
        self, tg_chat_id: Optional[int] = None
    ) -> str:
        """Снимок хода ``<system_context>`` (two-system-prompt pattern), собирается заново на каждый ход.

        ADR-0150 §7.3 (#3696): разделы — данные (:func:`core.turn_context.context_sections`), текст строит
        :func:`rob_box_dialog.context.build_context`; правил в снимке нет. ``tg_chat_id`` (issue #3598) — ход из
        Telegram: личность в ``<user_profile>`` — по chat_id (:mod:`core.turn_identity`), голос комнаты не
        применяется. LLM читает данные только отсюда; фразами в реплике их не подделать (prompt injection).
        Ниже снимка — блоки планировщика (активные задачи, план сегментов).
        """
        with self._speaker_lock:
            sp = dict(self._current_speaker)
        sp_name = sanitize_speaker_name(sp.get("name") or "")  # live 12.08: мусорные имена ("Null") не пускаем
        profile = list(user_profile_lines(sp_name, sp, tg_chat_id))
        # ADR-0150 §7.3 (В12, #3767): подхваченные голым «робот» фразы или ≤ 1 свежая фраза того же собеседника —
        # структурой, с меткой «кто сказал»; остальной бэклог (ТВ, чужие разговоры) — только счётчик фоновой речи.
        recent, self._pending_unaddressed = getattr(self, "_pending_unaddressed", None), None
        backlog = getattr(self, "_backlog", None)
        self.get_logger().info(
            f"🗒️ [backlog] recent_unaddressed entries={phrase_count(recent)} "
            f"background_phrases={backlog.background_phrases(time.time()) if backlog else 0}"
        )
        # #3695: кто сказал реплику — структурой (имя только при confirmed), а не префиксом в её тексте.
        speaker = speaker_section(self) if tg_chat_id is None else None
        lines = [build_context(context_sections(self, "\n".join(profile), recent, speaker))]
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
        2. Подтверждённый tag → ``note_seen`` шва идентичности (issue #2440):
           создаёт/обновляет профиль под ``speaker_scope(биометрический id)``
           (first_seen/last_seen/dialog_count). tag дальше не используется.
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

            # Issue #2440 — профиль ключуем стабильным биометрическим id
            # (результат speaker_id_node), а не per-session tag. tag здесь
            # уже сыграл свою роль: подтвердил реплику в SpeakerTracker.
            person = self._current_acquaintance()
            if person is None:
                self.get_logger().debug(
                    f"👤 [issue 2440] tag={tag!r} подтверждён, но биометрия "
                    "ещё не разрешила спикера — профиль не создаём "
                    "(tag нестабилен между сессиями)"
                )
                return None

            scope = speaker_scope(person.id)
            profile = await self._identity.note_seen(person)
            # Имя из «меня зовут X» — сохраняем в профиль (acceptance #1077).
            name = extract_speaker_name(user_input)
            if name and profile.get("name") != name:
                profile["name"] = name
                await self._memory.save_fact(
                    scope,
                    Fact(
                        key="profile",
                        value=profile,
                        tags=("speaker", "profile"),
                    ),
                )
                self.get_logger().info(
                    f"👤 [issue 1077] Спикер {person.id!r} представился: {name!r}"
                )

            # Факты спикера → контекст LLM (list_facts: все факты scope).
            facts = await self._memory.list_facts(scope, limit=20)
            context = format_speaker_context(
                profile,
                facts,
                is_new=just_confirmed,
            )
            if context:
                self.get_logger().info(
                    f"👤 [issue 1077] Спикер {person.id!r}: диалог "
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
        is_action_claim_retry: bool = False,
        is_synthetic: bool = False,
        raw_user_command: str | None = None,
        speaker_tag: str | None = None,
        speaker_duration_s: float = 0.0,
        from_tg: bool = False,
        utterance_id: str | None = None,
        session_epoch: int | None = None,
    ) -> None:
        # Issue #2835 — ход/ретрай, поставленный в loop до «новой сессии»,
        # а стартовавший после неё, не запускается вовсе.
        if not self._admit_turn_epoch(session_epoch):
            return
        epoch_token = TURN_EPOCH.set(session_epoch)
        with self._task_lock:
            self._run_task = asyncio.current_task()
        # Issue #2829 (ADR-0131 PR-2) — the CURRENT turn's utterance_id,
        # so speaker_id_node registers the phrase the person actually
        # introduced themselves in, not "whoever speaks next". Issue
        # #2842: RegisterSpeakerTool runs in the mcp_server process, so
        # it gets this via _mcp_turn_context → hidden /mcp/execute arg.
        self._current_turn_utterance_id = utterance_id
        # 06.10 set98207: ход для скрытого turn_id (request_music не снимает
        # сет своего хода); синтетический ретрай продолжает ход.
        self._turn_id = self._turn_stages().turn_id_for(utterance_id) or next_turn_id(
            getattr(self, "_turn_id", None),
            is_synthetic, is_action_claim_retry,
        )
        self._turn_heard_text = turn_heard_text(
            getattr(self, "_turn_heard_text", None), user_input, is_synthetic)
        # Issue #2925 -- самопредставление этой реплики; выставляет
        # _prepare_user_input_context (_note_self_intro).
        self._turn_self_intro = None
        self._turn_intro_registered = False
        self._turn_register_outcome = None
        self._turn_hidden_tools = ()
        self._turn_llm_said = []
        self._turn_set_tracks = None
        self._run_cancelled = False
        # Issue #1881 — бюджет синтетических ретраев сбрасывается ТОЛЬКО на
        # ход человека (is_synthetic=False). Issue #2631 — политика в helper.
        self._reset_turn_retry_budgets(is_synthetic=is_synthetic)
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
        self._retry_budget_exhausted_in_turn = False
        # Issue #2914 -- «в этом ходе прозвучал вопрос о личности».
        self._identity_question_asked_in_turn = False
        guard_retry_pending = False
        # Issue #918 — turn может быть отменён или упасть ДО присваивания
        # result (speaker-профиль, LLM, тул-луп). Инициализируем None
        # заранее, чтобы finally-блок мог безопасно отличить «результата
        # нет» от «код ниже упал» — и ВСЕГДА довести DIALOGUE_END +
        # _publish_state до конца.
        result = None
        speech_hold: Optional[TurnSpeechHold] = None
        turn_cancelled = False
        turn_failure = ""
        try:
            # Issue #1077 — перед LLM-вызовом обновляем профиль спикера и
            # собираем контекст о нём (имя, факты, число диалогов). Только
            # для подтверждённых tags (2+ фразы подряд, >= 0.8с) — защита
            # от нестабильных tags Yandex. Vosk fallback (tag=None) —
            # профиль не трогаем (edge case #4).
            speaker_context: Optional[str] = None
            if speaker_tag:
                speaker_context = await self._handle_speaker_turn(
                    speaker_tag,
                    user_input=user_input,
                    duration_s=speaker_duration_s,
                )
            # Issue #2631 (ADR-0021 R1) — блок TG/speaker-identity/
            # dynamic_system (CC=5) вынесен в helper.
            user_input, dynamic_system = await self._prepare_user_input_context(
                user_input=user_input,
                from_tg=from_tg,
                speaker_context=speaker_context,
                utterance_id=utterance_id,
            )
            # 🔴 FIX (issue #1101): _on_stt уже сделал DSM-переход
            # IDLE→LISTENING→DIALOGUE через WAKE_WORD+STT_RESULT. Передаём
            # preclassified_event=STT_RESULT чтобы AgentCore НЕ
            # переклассифицировал user-text (где может быть 'робот' внутри)
            # и не сломал guard.
            self.get_logger().info(
                f"🚀 [turn] calling process_input: user_input={user_input[:100]!r} "
                f"speaker_tag={speaker_tag!r}"
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
            # ADR-0150 PR-3b: рефлекс thinking (лицо, кольцо) - код, до вызова LLM.
            self._turn_stages().thinking(_llm_provider_name)
            # Issue #2631 (ADR-0021 R1) — OTel-span + LLM-метрика (CC=10
            # ветка) вынесены в helper. #3765: ответ на вопрос склейки после
            # регистрации решает код, фраза — из phrases, LLM не зовётся.
            coded_reply = identity_reply(self)
            result = DialogResult(spoken_text=coded_reply) if coded_reply else await self._invoke_llm_with_telemetry(
                user_input=user_input,
                is_synthetic=is_synthetic,
                speaker_tag=speaker_tag,
                speaker_context=speaker_context,
                dynamic_system=dynamic_system,
                metric_start=_llm_metric_start,
                provider_name=_llm_provider_name,
            )
            self.get_logger().info(
                # Issue #1899: include ``finish_reason`` on EVERY completed
                # stream (was previously logged only when the response was
                # empty). Together with the new ``truncated_tool_args`` flag
                # this lets operators see WHY the agent loop burned ~6 s on
                # a redundant tool-call retry — i.e. ``finish_reason='length'``
                # + ``truncated_tool_args=True`` ⇒ the model hit max_tokens
                # while still emitting arguments JSON.
                f"✅ [turn] process_input returned: spoken={result.spoken_text!r}[:60] "
                f"tools={list(result.tools_called or ())!r} "
                f"finish_reason={getattr(result, 'finish_reason', None)!r} "
                f"truncated_tool_args={getattr(result, 'truncated_tool_args', None)!r} "
                f"error={result.error!r}"
            )
            # Issue #2874 -- свободный текст хода придерживается до решения
            # post-turn гуардов в ``finally`` (_release_turn_speech).
            speech_hold = self._turn_speech_hold = TurnSpeechHold()
            deliver_turn_result(self, result, coded_reply, user_input=user_input, raw_user_command=raw_user_command)
            guard_retry_pending = bool(self._retry_dispatched_in_turn)
        except asyncio.CancelledError:
            self.get_logger().info("🛑 Turn cancelled (barge-in)")
            turn_cancelled = True
            # Issue #1160 — Prometheus metrics: barge-in (пользователь
            # перебил робота wake-word'ом во время TTS/LLM-ответа).
            if is_metrics_enabled():
                record_barge_in()
            result = None
        except Exception as exc:  # noqa: BLE001
            # Issue #2631 (ADR-0021 R1) — LLM-error handling (CC=7 ветка)
            # вынесен в helper. Семантика 1-в-1: сначала speak_direct
            # (живой 12.08 FIX — НЕ падать, если логгер уронит RcutilsLogger),
            # затем логгер (в try/except, чтобы и тут не падать).
            self._handle_llm_error(exc, user_input, raw_user_command)
            result = None
            turn_failure = f"llm_error:{type(exc).__name__}"
        finally:
            # A post-turn guard may synchronously dispatch a retry turn,
            # which sets ``self._run_task`` to a fresh coroutine. We must
            # not wipe that reference here or the test driver (and any
            # hook that watches ``self._run_task``) would lose the retry.
            # Only clear the slot when no replacement was scheduled.
            # Issue #2828/#3765 -- вопрос по ack регистрации, отложенный до
            # конца хода, забираем под тем же локом, что освобождает слот:
            # иначе он проскочил бы между «ход идёт» и «хода нет» и потерялся.
            leftover_identity_question = None
            with self._task_lock:
                if self._run_task is asyncio.current_task():
                    self._run_task = None
                    leftover_identity_question = getattr(self, "_after_turn_question", None)
                    self._after_turn_question = None
            # ADR-0150 §7.5 В17 (#3695): вопрос об имени решается после хода (не на команду, не раньше N-й
            # фразы собеседника) и звучит последней репликой.
            identity_ask = plan_identity_question(
                self, result, other_question=leftover_identity_question is not None)
            # Issue #2874 — ход с ретраем гуарда (решено в ``_handle_result``)
            # свой текст не придерживает; отозванный гуардом ответ молчит,
            # остальное звучит. Переспрос #2828, пришедший после ответа, —
            # после него, как и раньше. ADR-0150 PR-7 ч.7d-1: post-turn
            # tool-skipped гуард (#1777) удалён — ретраев из ``finally`` нет.
            self._release_turn_speech(speech_hold, retry_dispatched=False)
            start_after_turn_question(self, leftover_identity_question, result)
            ask_planned(self, identity_ask)
            # ADR-0150 PR-3b: тело хода закончено - исход done|failed решает код по итогам тулов и LLM.
            self._stage_finish(result, turn_cancelled, turn_failure)
            # Issue #992 Bug D — defer the DIALOGUE_END transition
            # when the babble detector scheduled a retry. The retry's
            # ``_run_turn`` needs the DSM to stay in DIALOGUE so the
            # LLM gate fires; otherwise the synthetic prompt is
            # classified as STT_RESULT but the state stays in IDLE
            # (no-op), and the user never hears the retry answer.
            # Issue #2631 (ADR-0021 R1) — DIALOGUE_END decision
            # (CC=5 ветка) вынесен в helper. Сигнатура и поведение
            # идентичны inline-блоку.
            self._finalize_turn_dsm(
                guard_retry_pending=guard_retry_pending,
                tool_retry_dispatched=False,
                session_handed_over=self._take_session_handover(),
            )
            TURN_EPOCH.reset(epoch_token)

    # ── Issue #2835 — поколение сессии ────────────────────────────────

    def _session_epoch_gate(self) -> SessionEpoch:
        """Ленивый доступ к :class:`SessionEpoch`.

        Тестовые фикстуры на ``object.__new__(DialogueNode)`` не проходят
        ``__init__`` — создаём счётчик по первому требованию.
        """
        gate = getattr(self, "_session_epoch", None)
        if gate is None:
            gate = SessionEpoch()
            self._session_epoch = gate
        return gate

    def _admit_turn_epoch(self, session_epoch: Optional[int]) -> bool:
        """Пускать ли ход поколения ``session_epoch`` (issue #2835).

        Ход, поставленный в loop до «новой сессии» (ретрай гуарда, дренаж
        очереди), после сброса отбрасывается.
        """
        gate = self._session_epoch_gate()
        if gate.is_stale(session_epoch):
            self.get_logger().warning(
                f"🧹 [issue 2835] ход из сессии #{session_epoch} отброшен — "
                f"после сброса идёт сессия #{gate.current}"
            )
            return False
        return True

    def _reset_session_music(self) -> None:
        """Issue #2835 — «новая сессия» гасит музыку прошлой сессии.

        ADR-0150 PR-7c ч.4б (#3700): стоп — у владельца плеера (``dj_set(stop)`` + ``stop_music``, как Tier-1
        «выключи музыку»), а не топиком уборки музыки (удалён).
        """
        self._stop_music_by_owner("new_session")

    def _stop_music_by_owner(self, reason: str) -> bool:
        """Жёсткий стоп музыки (сет и дека) через владельца плеера; ``False`` — исполнителя тулов нет."""
        stopped = tier1_runner_of(self).stop_music()
        self.get_logger().info(f"🔇 [{reason}] стоп музыки владельцем плеера: {stopped}")
        return stopped

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
        нужного тула. Музыкальные правила (``music_state``, ``track_load``,
        ``music_prose_action``) удалены в ADR-0149 PR-13a: об успехе запуска
        музыки говорит код по событию ``started``.

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
        # Issue #1881 — общий budget декрементится рядом с поимённым
        # флагом. Если budget == 0, ретрай НЕ отправляется.
        if not self._consume_synthetic_retry(guard_name="action_claim"):
            return False
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

    def _check_universal_action_claim_and_retry(
        self,
        *,
        spoken: str,
        user_input: Optional[str],
        tools_called: tuple,
        tool_error_occurred: bool = False,
    ) -> bool:
        """Issue #2549 — широкий anti-hallucination guard.

        Live DJ-сет 2026-09-15: LLM выдавала ``spoken`` вида::

            «Сделала два pass подряд: сначала один темп-каркас с
            heartbeat'ом и пульсом Бочкинса, потом второй…»
            «Вплела тему Грига как второй голос над пульсом Бочкинса…»
            «Проверю состояние и перезапущу.»
            «Ок, давай я снова перезапущу. Бочкинс с Григом наверху —
            стартую заново.»

        Все четыре — при ``tools_called=()``. Существующий Bug E
        guard (:func:`detect_unbacked_action_claim`) для таких фраз НЕ
        срабатывает: его таблица узкая, требует совпадения И в
        ``user_input``, И в ``spoken``. Здесь же юзер просил абстрактно
        («докрути музыку») без явного «сделай/запусти» — а LLM всё
        равно отчитывается о действии.

        Защита: если в ``spoken`` есть action-verb в past или future и
        ни один тул из :data:`CLAIM_JUSTIFYING_TOOLS` не был вызван —
        это action hallucination. Требуем ОДИН одноразовый CRITICAL-
        ретрай (тот же контракт, что у Bug E выше).

        Returns:
            ``True`` — ретрай отправлен, вызывающий НЕ должен публиковать
            текст в TTS (иначе юзер услышит ложное «сделала/запустил/»,
            а потом ответ ретрая).
        """
        if not spoken:
            return False
        if getattr(self, "_universal_action_claim_retry_used", False):
            return False
        if getattr(self, "_retry_dispatched_in_turn", False):
            # Другой guard уже отправил ретрай в этом turn (babble /
            # action-claim Bug E / renardo / tool) — параллельный
            # CRITICAL-тур вызовет пинг-понг.
            return False

        hit = self._universal_claim_hit(
            spoken=spoken,
            tools_called=tools_called,
            tool_error_occurred=tool_error_occurred,
        )
        if hit is None:
            return False

        # Тот же перевод DSM, что и в babble/renardo/action-claim
        # ретраях: без него process_input увидит IDLE и вернёт пустой
        # результат.
        try:
            if self._dsm.current_state == DialogueStateKind.IDLE:
                self._dsm.on_event(DialogueEvent.WAKE_WORD)
                self._publish_state()
            self._dsm.on_event(DialogueEvent.STT_RESULT)
            self._publish_state()
        except ImportError:
            pass

        # Issue #1881 — общий budget декрементится рядом с поимённым
        # флагом. Если budget == 0, ретрай НЕ отправляется.
        if not self._consume_synthetic_retry(
            guard_name="universal_action_claim"
        ):
            return False
        self._universal_action_claim_retry_used = True
        self._mark_retry_dispatched()
        self.get_logger().warning(
            "🧾 [issue 2549] anti-hallucination guard: spoken содержит "
            f"action-verb «{hit.verb}» ({hit.tense}), "
            f"tool_error_occurred={tool_error_occurred!r} — "
            f"head={hit.excerpt!r}, user_input={user_input!r}, "
            f"tools={list(tools_called)!r}"
        )
        self._dispatch_turn(
            build_universal_action_claim_retry_prompt(
                user_input=user_input,
                spoken=spoken,
                hit=hit,
                tool_error_occurred=tool_error_occurred,
            ),
            is_action_claim_retry=False,  # свой тип, чтобы не путать с Bug E
            is_synthetic=True,
            raw_user_command=user_input,
        )
        return True

    def _check_phantom_action_and_retry(
        self,
        *,
        spoken: str,
        user_input: Optional[str],
        tools_called: tuple,
    ) -> bool:
        """Issue #2559 — общий (НЕ music-only) guard «обещал, но не сделал».

        Round 3 live-check 15.09.2026 (Vision Pi 10.1.1.21): 6 случаев
        за час, когда LLM говорила «сейчас перезапущу / сделал погуще /
        подложу слой» при ``tools_called=[]``. Существующий Bug E
        (``_check_unbacked_action_claim_and_retry``) узкий — бытовые
        «проверю состояние и перезапущу» / «подкручу / установлю голос»
        проходили мимо.

        Условия срабатывания (см. ``detect_phantom_action_claim``):

          1. ``spoken`` содержит хотя бы один ``PHANTOM_ACTION_VERBS_RE``
             stem (сделал / запустил / перезапущу / проверю / подложу /
             подкручу / обновлю / поменяю / изменю / доработаю и т.п.);
          2. ``tools_called`` пуст;
          3. ``user_input`` НЕ содержит «не буду / не надо» (защита
             от ложного срабатывания на легитимный «не буду перезапускать»).

        Контракт ретрая (как Bug E / Bug F): один CRITICAL-ретрай на
        user-turn, иначе LLM уходит в ping-pong (issue #1881). При
        срабатывании возвращает ``True`` — вызывающий НЕ должен
        публиковать текст в TTS (иначе юзер услышит «сделал» + потом
        ответ ретрая).
        """
        if getattr(self, "_phantom_action_retry_used", False):
            return False
        if not detect_phantom_action_claim(
            user_input=user_input,
            spoken=spoken,
            tools_called=self._claim_backing_tools(tools_called),
        ):
            return False

        # Тот же перевод DSM, что и в bug E / Bug F: без него
        # process_input увидит IDLE и вернёт пустой результат.
        try:
            if self._dsm.current_state == DialogueStateKind.IDLE:
                self._dsm.on_event(DialogueEvent.WAKE_WORD)
                self._publish_state()
            self._dsm.on_event(DialogueEvent.STT_RESULT)
            self._publish_state()
        except ImportError:
            pass

        # Issue #1881 — общий budget декрементится рядом с поимённым
        # флагом. Если budget == 0, ретрай НЕ отправляется.
        if not self._consume_synthetic_retry(guard_name="phantom_action"):
            return False
        self._phantom_action_retry_used = True
        self._mark_retry_dispatched()
        self.get_logger().warning(
            f"🎭 [issue 2559] phantom-action claim без тула "
            f"(tools={list(tools_called)!r}, spoken={spoken[:80]!r}) — "
            f"один ретрай с требованием вызвать tool или убрать claim-verb"
        )
        self._dispatch_turn(
            build_phantom_action_retry_prompt(user_input),
            # Свой тип — НЕ путать с Bug E. ``is_action_claim_retry``
            # обрабатывается в Bug E-ветке (``_run_turn``), и phantom-
            # action намеренно избегает этого флага, чтобы флаги разных
            # guards не сбрасывали друг друга (issue #1881 ping-pong fix).
            is_action_claim_retry=False,
            is_synthetic=True,
            raw_user_command=user_input,
        )
        return True

    def _check_tool_call_markup_and_retry(
        self,
        *,
        spoken: str,
        user_input: Optional[str],
        tools_called: tuple,
        speak_text_real: int = 0,
    ) -> bool:
        """Issue #2760 — одноразовый ретрай на «вызов тула написан текстом».

        Live Vision Pi, прогон 35704637846 (акт 2, ``n204_boris_intro_long``):
        модель вернула в ``spoken`` разметку протокола tool-calls при
        ``tools=[]``, и tts_node прочитал её вслух двумя чанками::

            spoken='<function_calls>\\n<invoke name="register_speaker">…'
            🔊 TTS: batch=9b15b85e 1/2, text='<functioncalls>…'

        Проверка стоит ПЕРВОЙ среди post-turn guard'ов — до #2175 и
        babble: разметка не является ни regurgitates системного шаблона,
        ни мета-обещанием, и любой другой guard либо промолчит, либо
        наклеит на неё свой CRITICAL и отправит в TTS вместе с тегами.

        Условия ретрая — те же три, что у
        :meth:`_check_system_template_regurgitate_and_retry`:
        ``speak_text`` реально не звучала, флаг ещё не взведён, общий
        budget синтетических ретраев не исчерпан.

        Returns:
            ``True`` — ретрай отправлен, вызывающий НЕ публикует текст в
            TTS.
        """
        if speak_text_real > 0:
            return False
        if not spoken:
            return False
        if getattr(self, "_tool_call_markup_retry_used", False):
            return False
        if not is_tool_call_markup(spoken):
            return False

        # Тот же перевод DSM, что и в остальных синтетических ретраях:
        # без него process_input увидит IDLE и вернёт пустой результат.
        try:
            if self._dsm.current_state == DialogueStateKind.IDLE:
                self._dsm.on_event(DialogueEvent.WAKE_WORD)
                self._publish_state()
            self._dsm.on_event(DialogueEvent.STT_RESULT)
            self._publish_state()
        except ImportError:
            pass

        if not self._consume_synthetic_retry(guard_name="tool_call_markup"):
            return False
        self._tool_call_markup_retry_used = True
        self._mark_retry_dispatched()
        self.get_logger().warning(
            "🏷️ [issue 2760] LLM написала вызов тула ТЕКСТОМ вместо "
            f"tool-call (head={spoken[:120]!r}) — один ретрай, "
            f"user_input={user_input!r}, tools={list(tools_called)!r}"
        )
        self._dispatch_turn(
            build_tool_call_markup_retry_prompt(user_input),
            is_action_claim_retry=False,  # свой тип, как и у #2175
            is_synthetic=True,
            raw_user_command=user_input,
        )
        return True

    def _check_system_template_regurgitate_and_retry(
        self,
        *,
        spoken: str,
        user_input: Optional[str],
        tools_called: tuple,
        speak_text_real: int = 0,
    ) -> bool:
        """Issue #2175 — одноразовый ретрай на regurgitated system-template.

        Live 08.09 (Vision Pi, 14:52): три запроса подряд после
        ``set_voice`` + multi-voice user_input + новая DJ-skill context
        давали в ``spoken`` ровно кусок СИСТЕМНОГО промпта вида::

            <system>
            [получатель ответа забыл указать антропоморфные атрибуты]
            </system>

        Это невалидный user-facing ответ — TTS озвучивал метаинструкцию
        через Yandex→MiniMax fallback, юзер слышал «получатель ответа
        забыл указать антропоморфные атрибуты» поверх только что
        сменённого голоса. Корневая причина — MiniMax-M3 regurgitates
        system-template при определённых условиях (см. issue body).

        Защита — двухуровневая (это первый уровень, dialogue_node):
        regex-detector :func:`is_system_template_regurgitated` ловит
        ПОЛНЫЙ ``<system>...</system>``-блок без surrounding текста
        и требует ОДИН одноразовый CRITICAL-ретрай. Второй уровень —
        defense-in-depth в :func:`tts_node.dialogue_callback` (отказ
        синтеза + ``/voice/tts/finished(success=False)``).

        Retry rules — все должны выполниться для ретрая:
        1. ``speak_text`` НЕ была реально вызвана (``speak_text_real==0``).
           Если LLM уже что-то произнесла через тул — НЕ вмешиваемся.
        2. ``spoken`` — regurgitated ``<system>...</system>``-блок
           (см. :func:`is_system_template_regurgitated`).
        3. ``_system_regurgitate_retry_used`` ещё ``False`` (защита от
           ping-pong) и общий ``_synthetic_retries_left`` не исчерпан.

        Returns:
            ``True`` — ретрай отправлен, вызывающий НЕ должен публиковать
            текст в TTS (иначе юзер услышит regurgitates, а потом
            ответ ретрая).
        """
        if speak_text_real > 0:
            return False
        if not spoken:
            return False
        if getattr(self, "_system_regurgitate_retry_used", False):
            return False
        if not is_system_template_regurgitated(spoken):
            return False

        # Тот же перевод DSM, что и в babble/renardo/action-claim ретраях:
        # без него process_input увидит IDLE и вернёт пустой результат.
        try:
            if self._dsm.current_state == DialogueStateKind.IDLE:
                self._dsm.on_event(DialogueEvent.WAKE_WORD)
                self._publish_state()
            self._dsm.on_event(DialogueEvent.STT_RESULT)
            self._publish_state()
        except ImportError:
            pass

        # Issue #1881 — общий budget декрементится рядом с поимённым
        # флагом. Если budget == 0, ретрай НЕ отправляется.
        if not self._consume_synthetic_retry(guard_name="system_regurgitate"):
            return False
        self._system_regurgitate_retry_used = True
        self._mark_retry_dispatched()
        self.get_logger().warning(
            "🧾 [issue 2175] MiniMax regurgitates system-template в spoken "
            f"(head={spoken[:120]!r}) — один ретрай, "
            f"user_input={user_input!r}, tools={list(tools_called)!r}"
        )
        self._dispatch_turn(
            build_system_regurgitate_retry_prompt(user_input),
            is_action_claim_retry=False,  # свой тип, чтобы не путать с Bug E
            is_synthetic=True,
            raw_user_command=user_input,
        )
        return True

    def _reopen_dialogue_for_retry(self) -> None:
        """Re-drive the DSM to DIALOGUE before a synchronous retry dispatch.

        ``agent_core.process_input`` gates the LLM on
        ``current_state == DIALOGUE``. The parent turn's ``process_input``
        already fired ``DIALOGUE_END``, so by the time the post-turn guard
        runs the state is IDLE — a retry dispatched from here would
        short-circuit and never reach the LLM (13.08 DJ incident: retry
        returned in ~1 ms with zero ``[health]`` logs).

        Issue #992 / #1204.
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

    def _consume_synthetic_retry(self, *, guard_name: str) -> bool:
        """Issue #1881 — попытаться списать один синтетический ретрай.

        Гарантирует, что общий бюджет ``_synthetic_retries_left`` и
        поимённые одноразовые флаги гуардов синхронны
        и не разъезжаются. Каждый guard, который собирается
        задиспатчить ретрай, ОБЯЗАН сначала вызвать этот метод; если
        он вернул ``False`` — ретрай отменяется, даже если поимённый
        флаг ещё не взведён.

        Returns:
            ``True`` — ретрай разрешён (budget декрементнут), guard
                может диспатчить.
            ``False`` — budget исчерпан (== 0); guard должен
                вернуть ``False`` из своего ``_check_*_and_retry``.
                На этом ЛОГируется предупреждение, чтобы в логе
                было видно, что цикл оборвали НАМЕРЕННО (не молча).
        """
        if self._synthetic_retries_left <= 0:
            self.get_logger().warning(
                f"🚦 [issue 1881 retry-budget] исчерпан на turn, отдаю как есть "
                f"(guard={guard_name})"
            )
            # Issue #2874 — «как есть» = первая фраза, не сырой монолог.
            self._retry_budget_exhausted_in_turn = True
            return False
        self._synthetic_retries_left -= 1
        return True


    def _retract_rejected_reply(self) -> None:
        """Issue #3145 — убрать из истории ответ, отвергнутый гуардом.

        Раньше отзыв делали только music-гуарды; ретраи Bug D/E,
        tool-skipped и прочие оставляли плохой ответ в окне, и после ответа
        ретрая в истории стояли два assistant подряд (живой лог 28.09 — 28
        мест; «Клубняк в клубе, погнали дальше!» ~10 ходов висел ответом на
        «ты диджей Снупдог»). Теперь это делает :meth:`_dispatch_turn` для
        ЛЮБОГО синтетического ретрая.

        ``AgentCore.discard_last_reply`` снимает только ответ последнего
        хода (повторный вызов и ход без записанного ответа — no-op), поэтому
        music-гуард, который и сам зовёт отзыв, второй ответ не снесёт.
        Планируется на loop ДО ретрая — FIFO гарантирует, что ретрай уже не
        увидит отвергнутый ответ.
        """
        core = getattr(self, "_core", None)
        if core is None:
            return
        future = asyncio.run_coroutine_threadsafe(
            core.discard_last_reply(), self._loop
        )

        def _log_if_failed(fut: "asyncio.Future[bool]") -> None:
            try:
                removed = fut.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(
                    f"🧹 [issue 3145] discard_last_reply failed: {exc}"
                )
                return
            if removed:
                self.get_logger().info(
                    "🧹 [issue 3145] отвергнутый гуардом ответ убран из истории"
                )

        # Тестовые стабы ``run_coroutine_threadsafe`` future не возвращают.
        if future is not None:
            future.add_done_callback(_log_if_failed)

    # ------------------------------------------------------------------
    # ── Issue #2631 / ADR-0021 R1 — _run_turn decomp helpers ────────────
    # Цель: удержать CC ``_run_turn`` ≤15 (текущий CC=59; см.
    # ``cc_budget_baseline.json`` и ADR-0021). Каждый helper извлекает
    # один семантический шаг pipeline'а без изменения поведения. Сигнатура
    # ``_run_turn`` (kwargs + return None) и публичные методы узла
    # остаются без изменений — тесты используют ``node._run_turn(...)``
    # напрямую (см. ``test_dialogue_shell.py``).
    def _reset_turn_retry_budgets(self, *, is_synthetic: bool) -> None:
        """Issue #1881 / #2266 / #2631 — сброс retry-флагов на user-turn.

        Делает **ровно** то, что раньше жил в теле ``_run_turn`` в блоке
        ``if not is_synthetic:`` (issue #1881 ping-pong): сбрасывает
        поимённые флаги гуардов и ``_synthetic_retries_left``.
        """
        if is_synthetic:
            return
        self._action_claim_retry_used = False
        self._system_regurgitate_retry_used = False
        self._tool_call_markup_retry_used = False  # Issue #2760
        self._universal_action_claim_retry_used = False
        self._phantom_action_retry_used = False  # Issue #2559
        self._synthetic_retries_left = self.DEFAULT_SYNTHETIC_RETRIES

    def _finalize_turn_dsm(
        self,
        *,
        guard_retry_pending: bool,
        tool_retry_dispatched: bool,
        session_handed_over: bool = False,
    ) -> None:
        """Issue #992 Bug D / S7 — финальный DSM-переход в ``_run_turn``.

        Условие штатного закрытия сессии (DIALOGUE → IDLE):

        * DSM в ``DIALOGUE`` (нет смысла «закрывать» сессию, которой нет),
        * ни один guard не запланировал синхронный/отложенный retry
          (``guard_retry_pending`` / ``tool_retry_dispatched``),
        * ход не отменён новой фразой (``session_handed_over``, issue
          #2939): её приём уже перевёл DSM в DIALOGUE для СЛЕДУЮЩЕГО хода.

        Иначе ретрай/follow-up придёт в ``IDLE`` и короткозамкнётся без
        LLM (issue #992 Bug D, #1204). После успешного DIALOGUE_END
        публикуем ROS-стейт через :meth:`_publish_state` — иначе топик
        зависает на старом значении (см. issue #1101).
        """
        if (
            self._dsm.current_state == DialogueStateKind.DIALOGUE
            and not guard_retry_pending
            and not tool_retry_dispatched
            and not session_handed_over
        ):
            self._dsm.on_event(DialogueEvent.DIALOGUE_END)
            # Issue #1160 — Prometheus metrics: сессия закрылась
            # штатно (DIALOGUE_END) — пишем duration histogram.
            self._maybe_record_session_end(result="success")
        # AgentCore completes the DIALOGUE → IDLE transition itself.
        # Publish the resulting state even when no transition is needed
        # here; otherwise the ROS state topic remains stuck at the
        # earlier DIALOGUE notification and scenario runners wait forever.
        self._publish_state()

    def _handle_llm_error(
        self,
        exc: BaseException,
        user_input: str,
        raw_user_command: Optional[str],
    ) -> None:
        """Issue #1278 / live 12.08 — best-effort LLM-error recovery.

        1. Если провайдер недоступен (issue #1278) —
           озвучиваем degraded-фразу из :meth:`_generate_fallback_response`.
           Иначе — общее «Что-то я задумался».
        2. Логируем traceback (хвост 500 символов).

        Оба шага обёрнуты в ``try/except``: чтобы робот ВСЁ РАВНО
        ответил юзеру, даже если RcutilsLogger упал. Это критический
        контракт «принял но не ответил» (live 12.08) — fix был
        инвертирован именно из-за этого.
        """
        _tb_str = traceback.format_exc()
        try:
            if self._is_llm_unavailable_error(exc):
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
                f"❌ AgentCore error: {exc}\n{_tb_str[-500:]}"
            )
        except Exception:
            pass

    async def _invoke_llm_with_telemetry(
        self,
        *,
        user_input: str,
        is_synthetic: bool,
        speaker_tag: Optional[str],
        speaker_context: Optional[str],
        dynamic_system: Any,
        metric_start: float,
        provider_name: str,
    ) -> DialogResult:
        """Issue #1234 / #1160 — обёртка ``process_input`` с OTel и метриками.

        Вынесен из ``_run_turn`` (CC=10 ветка; ADR-0021 R1, issue #2631).
        Семантика 1-в-1 с inline-блоком:

        1. Открыть OTel-span ``dialogue.llm_call`` с атрибутами provider/model.
           ``start_span`` — no-op без OTel.
        2. Задать срез тулов хода ДО LLM-вызова (``tools_view_host``, ADR-0150 PR-6).
        3. Вызвать ``self._core.process_input(...)`` с preclassified_event.
        4. Записать в span атрибуты ``fallback`` / ``duration_s``.
        5. В ``finally`` — записать Prometheus-метрику
           ``record_voice_llm_request``. ``success`` определяется по
           факту наличия ``result`` И ``result.error`` (как в
           оригинальном блоке через ``locals().get("result")``).
        """
        result: Optional[DialogResult] = None
        success = False
        try:
            with start_span(
                "dialogue.llm_call",
                {
                    "provider": provider_name,
                    "model": getattr(self._llm, "model", "")
                    or getattr(self._llm, "_model", ""),
                },
            ) as llm_span:
                apply_tools_view(self, user_input)
                plan_begin_turn(self, user_input)
                result = await self._core.process_input(
                    user_input,
                    is_synthetic=is_synthetic,
                    speaker_tag=speaker_tag,
                    speaker_context=speaker_context,
                    dynamic_system=dynamic_system,
                    preclassified_event=DialogueEvent.STT_RESULT,
                )
                success = result is not None and not result.error
                llm_span.set_attribute(
                    "fallback",
                    provider_name == "HealthAwareFallbackLLM",
                )
                llm_span.set_attribute(
                    "duration_s",
                    time.monotonic() - metric_start,
                )
                return result
        finally:
            plan_begin_turn(self, "")  # Tier-1 плана живёт один ход LLM: роутер и следующий ход его не видят
            if is_metrics_enabled():
                _duration = time.monotonic() - metric_start
                try:
                    record_voice_llm_request(
                        provider_name,
                        success=success,
                        fallback=(
                            provider_name == "HealthAwareFallbackLLM"
                        ),
                        duration_s=_duration,
                    )
                except Exception as _metric_exc:  # noqa: BLE001
                    self.get_logger().warning(
                        f"⚠️ [metrics] record_voice_llm_request failed: "
                        f"{_metric_exc!r}"
                    )

    async def _prepare_user_input_context(
        self,
        *,
        user_input: str,
        from_tg: bool,
        speaker_context: Optional[str],
        utterance_id: Optional[str] = None,
    ) -> tuple[str, Any]:
        """Issue #992 / #1077 / #1195 / live 10.08 — user_input prelude.

        Вынесен из ``_run_turn`` (CC=5 ветка; ADR-0021 R1, issue #2631).
        Применяет префиксы и контекст ДО LLM-вызова:

        * Telegram-источник → префикс ``[TG] `` (issue #1195).
        * Голосовая биометрия → :meth:`_apply_speaker_identity` (issue #1077).
        * Two-system-prompt snapshot → :meth:`_build_dynamic_system_context`
          (live 10.08). Используется как второй system-message в
          ``messages[]``.

        Возвращает ``(user_input, dynamic_system)``.
        """
        # 06.10: число треков/минут сета из СЫРОЙ реплики -- скрытый
        # аргумент dj_set (llm_adapter.TURN_CONTEXT_ARGS).
        self._turn_set_tracks = heard_set_length(user_input)
        tg_chat_id = None
        if from_tg:
            user_input = f"[TG] {user_input}"
            # Issue #3598 — личность TG-хода решает chat_id, не голос
            # комнаты. Тот же chat_id, что маршрутизирует ответ в Telegram.
            tg_chat_id = self._active_tg_chat_id
        elif self._speaker_id_enabled:
            # Issue #2829 (ADR-0131) -- резолвим _current_speaker ПО ЭТОЙ
            # фразе: ответ на вопрос склейки #2828 (merge_reply, после этой
            # прелюдии) сверяет диктора реплики с профилями из вопроса.
            await self._resolve_speaker_for_utterance(utterance_id)
            # Issue #2925 -- представление в реплике читаем ДО решения о
            # tentative-вопросе (#2888), регистрируем ПОСЛЕ него.
            self._note_self_intro(user_input, utterance_id)
            user_input = await self._apply_speaker_identity(
                user_input, speaker_context, utterance_id
            )
            self._register_self_intro(utterance_id)
            # #3765: личность под вопросом (голос похож на другого) — выясняет робот, ход LLM её не решает.
            await settle_identity(self)
        return user_input, self._build_dynamic_system_context(tg_chat_id)


    def _music_playing_now(self) -> bool:
        """Issue #3125 / #3133 — играет ли сейчас музыка, по словам плеера.

        Источник — только снимок ``/voice/music/state`` (ADR-0141). До
        #3133 здесь читался флаг TRACK-режима, который ставился
        по ИМЕНАМ тулов (в т.ч. ``set_dj_mode(enabled=false)`` и упавшего
        ``compose_music``) и снимался только по idle, а idle после конечного
        трека приходил через 30 минут: робот говорил «Музыка играет» в
        тишине. Снимка ещё нет — не играет. ``stops_at`` в прошлом —
        не играет, даже если свежий idle не дошёл.
        """
        snapshot = getattr(self, "_music_player_state", None)
        return snapshot is not None and snapshot.is_playing()


    # ── Issue #3134 — роутер медиакоманд до LLM ────────────────────────

    def _media_state(self) -> MediaState:
        """ЕДИНСТВЕННЫЙ аксессор состояния плеера для роутера медиакоманд.

        Источник один — снимок плеера ``/voice/music/state`` (#3133,
        ADR-0141): «играет» — :meth:`_music_playing_now`, название трека и
        DJ-сет — поле ``dj`` снимка движка v2 (``title``/``enabled``,
        ADR-0149 §2.3). Роутер других источников не читает: сменить
        источник — здесь и только здесь.
        """
        return media_state_from_snapshot(
            self._music_playing_now(), getattr(self, "_music_player_state", None)
        )

    def _route_media_command(self, text: str, on_miss: Any = None) -> bool:
        """Медиакоманду исполняет код, до LLM. ``True`` — в LLM не идёт.

        MCP-тулы вызываются тем же исполнителем, что и у LLM-хода
        (``PlanToolExecutor`` → ``ROSMCPToolProvider`` →
        ``LLMToolCallAdapter`` → подписанный ``/mcp/execute``).

        ``on_miss`` — issue #3176: куда вернуть реплику, если заказ по имени
        не нашёлся в базе мелодий (``callable(text)``). Без него заказ по
        имени роутер не берёт — реплика сразу идёт в LLM, как раньше.
        """
        router = getattr(self, "_media_router", None)
        if router is None:
            router = self._media_router = MediaRouter()
        plan = router.route(text, self._media_state())
        if plan is None:
            return False
        executor = getattr(self, "_tool_executor", None)
        if plan.play_name:
            return self._start_play_named(plan, executor, text, on_miss)
        if plan.needs_tools and executor is None:
            self.get_logger().warning(
                f"🎛️ [media-router] {plan.command.intent.value}: MCP-тулов нет "
                "(tool_provider не ros_mcp) — отдаю реплику LLM"
            )
            return False
        self.get_logger().info(
            f"🎛️ [media-router] intent={plan.command.intent.value} "
            f"closed={plan.command.closed} provider={router.provider_name} "
            f"tools={[c.name for c in plan.tool_calls]} text={text[:60]!r}"
        )
        self._apply_media_plan_side_effects(plan)
        asyncio.run_coroutine_threadsafe(
            self._execute_media_plan(plan, executor, text), self._loop
        )
        return True

    def _start_play_named(
        self, plan: MediaPlan, executor: Any, text: str, on_miss: Any
    ) -> bool:
        """Issue #3176 — заказ по имени: забрать реплику и искать мелодию.

        Реплика забирается сразу (``True``), а есть ли мелодия, выясняет
        :meth:`_execute_play_named` в loop'е: не нашлась — ``on_miss``
        возвращает её в приём, и отвечает LLM. Некуда вернуть (нет
        ``on_miss``) или нечем искать (нет MCP-исполнителя) — реплику не
        забираем вовсе.
        """
        if executor is None or not callable(on_miss):
            self.get_logger().warning(
                "🎛️ [media-router] play_named: нет MCP-исполнителя или пути "
                "назад в приём — отдаю реплику LLM"
            )
            return False
        self.get_logger().info(
            f"🎛️ [media-router] intent=play_named name={plan.play_name!r} "
            f"provider={self._media_router.provider_name} text={text[:60]!r}"
        )
        self._turn_stages().hold()  # #3794: промах поиска вернёт фразу в LLM - ход до тех пор не закрывать
        asyncio.run_coroutine_threadsafe(
            self._execute_play_named(plan, executor, text, on_miss), self._loop
        )
        return True

    async def _execute_play_named(
        self, plan: MediaPlan, executor: Any, text: str, on_miss: Any
    ) -> None:
        """Issue #3176 — ``lookup_melody`` → ``request_music`` (движок v2) или LLM.

        * Нашлась целиком и заиграла — отменить идущий ход LLM (заказ его
          заменяет), учесть трек (cleanup) и сказать «Ставлю «X»».
        * Не нашлась / совпала не целиком — ``on_miss(text)``: реплика
          идёт в LLM, роутер НИЧЕГО не говорит (иначе двойной ответ).
        * Нашлась, но не заиграла — честная фраза, без LLM.
        """

        async def _call(name: str, args: Dict[str, Any]) -> Tuple[bool, str]:
            return await self._execute_media_tool_result(
                executor, MediaToolCall(name, args)
            )

        # #3525: заказ по имени — своя реплика: новый turn_id (иначе id прошлой команды роутера).
        vars(self).update(command_turn_context(plan, text))
        begin_turn = getattr(executor, "begin_turn", None)
        if callable(begin_turn):
            # Лимит «один трек за ход» (#2859) снимается только на границе
            # хода LLM: трек прошлого хода иначе отказал бы заказу.
            begin_turn()
        outcome = await run_named_play(_call, plan.play_name, plan.command.more)
        self.get_logger().info(
            f"🎛️ [media-router] play_named {plan.play_name!r}: "
            f"{outcome.status.value} {outcome.reason}".rstrip()
        )
        if outcome.status is NamedPlayStatus.MISS:
            # Issue #3400: мимо базы посреди сета — это тема сета, решает код.
            switch = dj_theme_switch_plan(plan, self._media_state())
            if switch is None:
                self._turn_stages().hold(reopen=True)  # #3794: промах поиска - не провал хода
                on_miss(text)
                return
            self._turn_stages().hold(reopen=False)
            self._apply_media_plan_side_effects(switch)
            await self._execute_media_plan(switch, executor, text)
            return
        self._llm_skipped_counter["media_command"] += 1
        self._cancel_run("media command play_named (issue 3176)", stop_tts=True)
        # Issue #3178: слова юзера («к элизе»), не архивный title базы
        # («Fur Elise») — юзер и так знает, что просил.
        title = user_phrase_title(plan.play_name)
        phrase = outcome_text(outcome, title)
        self._record_media_turn(plan, text, phrase, list(outcome.tools_done))
        self._speak_direct(phrase)
        self._turn_stages().hold(reopen=False)

    def _apply_media_plan_side_effects(self, plan: MediaPlan) -> None:
        """Синхронная часть плана: счётчик и отмена идущего хода."""
        self._llm_skipped_counter["media_command"] += 1
        if plan.cancel_inflight:
            self._cancel_run("media command (issue 3134)", stop_tts=True)

    async def _execute_media_plan(
        self, plan: MediaPlan, executor: Any, text: str = ""
    ) -> None:
        """Вызвать тулы плана по порядку и сказать фиксированную фразу.

        Issue #3165: успешные тулы и сказанная фраза записываются для
        модели и гуардов (:meth:`_record_media_turn`).
        """
        # Команда вместо хода LLM — свой ход для скрытых аргументов тулов (тема/длина сета из ЭТОЙ реплики).
        vars(self).update(command_turn_context(plan, text))
        # ADR-0149 PR-6: фраза об успехе запуска — только по событию ``started``.
        _ok, phrase, done = await run_media_plan(
            plan, functools.partial(self._execute_media_tool_result, executor),
            getattr(self, "_music_events", None), log=self.get_logger().info,
            begin_turn=getattr(executor, "begin_turn", None),
        )
        self._record_media_turn(plan, text, phrase, done)
        if phrase:
            self._speak_direct(phrase)

    def _record_media_turn(
        self, plan: MediaPlan, text: str, phrase: str, done: list
    ) -> None:
        """Issue #3165 — ход роутера виден модели и гуардам.

        * История модели: пара «реплика → фраза» с тулами в metadata
          (``AgentCore.record_external_turn``), то есть строка в блоке
          «выполнено в прошлых ходах».
        * Журнал для гуардов #2549 / #2559 / #2949
          (:meth:`_claim_backing_tools`): «я остановил» после стопа
          роутером — не фантом.
        """
        ledger = getattr(self, "_media_actions", None)
        if ledger is None:
            ledger = self._media_actions = deque(maxlen=16)
        ledger.extend(done)
        # ADR-0148 / #3461: фраза запуска в окне модели — образец, который она копирует в ходе без тула.
        record = history_recorder(getattr(self, "_core", None), plan.command.intent)
        if record is not None:
            record(text, phrase, done)

    def _claim_backing_tools(self, tools_called: Any) -> tuple:
        """Issue #3165 — тулы хода плюс тулы роутера медиакоманд этой сессии.

        Для широких гуардов «заявил действие без тула» (#2549, #2559,
        fallback #2949): модель честно пересказывает стоп/громкость/DJ,
        исполненные роутером, а в её собственном ходе тулов нет. Пересказ
        сделанного кодом в этой сессии — факт, а не фантом: окна по времени
        нет (ADR-0150 PR-6, T10), журнал живёт до новой сессии (≤ 16 тулов).
        """
        return tuple(tools_called or ()) + tuple(getattr(self, "_media_actions", None) or ())

    def _universal_claim_hit(
        self,
        *,
        spoken: str,
        tools_called: Any,
        tool_error_occurred: bool = False,
    ) -> Optional[UniversalActionClaimHit]:
        """Issue #2549 + #3165 + #3174 — не подкреплённое заявление действия.

        Журнал роутера (:meth:`_claim_backing_tools`) подкрепляет только
        ПЕРЕСКАЗ сделанного («я остановил трек», прошедшее время). Обещание
        в будущем времени («Сейчас поставлю к Элизе!») — это действие ЭТОГО
        хода, и команда роутера 30 с назад его не выполняет. Живой прогон 29.09 05:00 (issue #3174): журнал роутера
        заглушил гуард, обещание ушло в TTS с ``tools=[]``.
        """
        own = detect_universal_action_claim(
            spoken=spoken,
            tools_called=tuple(tools_called or ()),
            tool_error_occurred=tool_error_occurred,
        )
        if own is None or own.tense == "future":
            return own
        return detect_universal_action_claim(
            spoken=spoken,
            tools_called=self._claim_backing_tools(tools_called),
            tool_error_occurred=tool_error_occurred,
        )

    async def _execute_media_tool_result(
        self, executor: Any, call: Any
    ) -> Tuple[bool, str]:
        """Вызвать MCP-тул роутера: ``(успех, текст результата)``."""
        tool_call = ToolCall(
            id=f"media-router-{uuid.uuid4().hex[:8]}",
            name=call.name,
            arguments=dict(call.arguments),
        )
        try:
            result = await executor.execute(tool_call)
        except Exception as exc:  # noqa: BLE001 — честная фраза вместо падения
            self.get_logger().warning(
                f"🎛️ [media-router] {call.name}({call.arguments}) упал: {exc}"
            )
            return False, ""
        content = str(getattr(result, "content", "") or "")
        ok = media_tool_succeeded(
            bool(getattr(result, "is_error", False)), content
        )
        self.get_logger().info(
            f"🎛️ [media-router] {call.name}({call.arguments}) ok={ok} "
            f"result={content[:160]!r}"
        )
        return ok, content


    # ═══════════════════════════════════════════════════════════════════════
    #: Issue #1881 — лимит синтетических ретраев на ОДИН user-initiated turn.
    #:
    #: Раньше каждый guard (babble / action-claim / code-speech / tool /
    #: music) имел собственный одноразовый флаг и при ретрае сбрасывал
    #: чужие через ``if not is_<X>_retry: self._<Y>_retry_used = False``
    # в ``_run_turn`` — это и был источник ping-pong'a на 8 LLM-вызовов.
    #:
    #: Теперь общий budget живёт в ``self._synthetic_retries_left`` и
    #: декрементится через :meth:`_consume_synthetic_retry`; любой guard
    #: может выстрелить, пока budget > 0. На свежем user-initiated turn
    #: ресетится в :meth:`_run_turn`. 2 взято из live-логов:
    #: babble-retry (1) → если и ретрай babble'нул → ещё 1 (music/babble
    #: любой) → «растерялся». Больше 2 — уже деградация UX, как раз то,
    #: что увидели 02.09 на 8 вызовах.
    DEFAULT_SYNTHETIC_RETRIES: int = 2


    def _handle_result(
        self,
        result: DialogResult,
        user_input: Optional[str] = None,
        raw_user_command: Optional[str] = None,
    ) -> None:
        """Publish (or swallow) the LLM response for one turn.

        ``user_input`` is threaded through from :meth:`_run_turn` so the
        action-claim guards can echo the user request in their retry
        prompt. Optional for callers that bypass :meth:`_run_turn`.

        ``raw_user_command`` (issue #1204) — оригинальная команда юзера.
        На синтетических ретрай-турах ``user_input`` содержит CRITICAL-
        промпт (со словом «диджея» внутри), который НЕЛЬЗЯ сканировать
        эвристиками стоп-слов / music-intent — иначе ложный
        «stop-command» убивает музыку.
        """
        if result.error is not None:
            # 🔴 FIX (live 12.08): безопасный лог ошибки — если логгер
            # упадёт, мы НЕ теряем fallback-ответ пользователю.
            try:
                self.get_logger().warning(f"⚠️ AgentCore error: {result.error}")
            except Exception:
                pass
            # 🔴 FIX (issue #1278): когда ВСЕ LLM-провайдеры недоступны
            # (health-aware-fallback: все провайдеры unavailable) — робот
            # должен сказать честную degraded-фразу («интернет недоступен,
            # базовые команды работают»), а НЕ «Принял.»/«задумался».
            # Раньше ProviderError тонул в empty-response fallback ниже и
            # юзер слышал «Принял.» вместо объяснения.
            if self._is_llm_unavailable_error(result.error):
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
        # Финальный текст хода для авто-TTS (#3598: проверку «цитирует служебное
        # из контекста» сняли в ADR-0150 PR-7 ч.7d-3, #3698).
        spoken = strip_history_marker(
            final_text_for_speech(result, self.get_logger().warning)
        )
        # 🔴 FIX (live 16:02 «английская мысль на Бахе»): MiniMax M3
        # возвращает ``<think>...</think>`` в content перед реальным
        # ответом. Done-чекер ниже ловит «done», но если перед ним лежит
        # английский мысленный комментарий («Music started successfully.
        # Now return "done" — no speak_text...»), он весь уходит в TTS
        # как «озвучка ответа». Strip-блоков ДО done-чекера → в TTS идёт
        # либо пусто (маркер done → тишина), либо реальный финал.
        spoken = strip_thinking_blocks(spoken)
        # Issue #2547: strip internal section-header prefixes like
        # ``[Мнение ассистента]``, ``[Примечание]``, ``[Note]``,
        # ``[Answer]``, ``**Итог:**``, ``**Answer:**`` BEFORE markdown so
        # the bold form (``**…**``) is captured intact, and BEFORE the
        # done-marker equality check so the stripper sees only the
        # user-facing remainder. Live 15.09: 193 cases/hour on Vision Pi
        # DJ-set when TTS reads them as part of the reply.
        spoken = strip_meta_markers(spoken)
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
        if is_done_marker(spoken):  # одна таблица маркеров с ядром (text_classify.DONE_MARKERS)
            self.get_logger().info(
                f"🔇 LLM completion marker — skip auto-TTS: {spoken[:60]!r}"
            )
            spoken = ""
        tools_called = tuple(result.tools_called or ())
        # Issue #3455 — фраза запуска музыки (шаблон кода) без тула запуска
        # в этом ходе: наружу честная фраза, неправда снимается из истории.
        # TEMP(ADR-0148) — #3461.
        spoken = honest_launch_reply(
            spoken, tools_called, log=self.get_logger().warning,
            on_replace=self._retract_rejected_reply,
        )
        # Issue #2949 — was any tool THIS TURN called-but-errored (refused /
        # threw)? A called tool alone does not "back" a spoken claim of
        # success (``CLAIM_JUSTIFYING_TOOLS`` guards below need this to
        # not treat a refused ``save_arrangement_preset`` as a done deal).
        tool_error_occurred = bool(getattr(result, "tool_error_occurred", False))
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
        # accept sound and then silence. AgentCore now tracks
        # ``speak_text_real_count`` (calls with non-empty ``text``) and
        # we skip only when speech REALLY happened.
        speak_text_real = int(getattr(result, "speak_text_real_count", 0) or 0)
        # Issue #1708 — hallucinated-lyrics guard diagnostic. When the
        # LLM called BOTH a music tool AND ``speak_text`` in the same
        # cycle, AgentCore's heuristic may have suppressed the
        # ``speak_text`` call (replaced with a sentinel error). If it
        # did, ``speak_text_real_count`` was decremented to zero, but
        # ``tools_called`` still lists both names so operators can see
        # the suppression happened. Log a one-line diagnostic so the
        # live log makes the pattern obvious without grepping.
        # Единая таблица музыкальных тулов — rob_box_core.music_tools (ADR-0158 §7).
        _music_tool_names = MUSIC_AUDIBLE_LYRICS_TOOLS
        _has_music_tool = any(
            name in _music_tool_names for name in tools_called
        )
        _has_speak_text = "speak_text" in tools_called
        if _has_music_tool and _has_speak_text:
            # If speak_text_real==0 BUT tools_called still contains
            # speak_text, the agent_core guard dropped the call.
            # Otherwise (speak_text_real>0), BACKING mode ran normally
            # — only log at debug to avoid noise.
            if speak_text_real == 0:
                self.get_logger().warning(
                    "🎤 [issue 1708] music tool + speak_text в "
                    "одном turn — speak_text ПОДАВЛЕН (hallucinated "
                    f"lyrics guard). tools={list(tools_called)!r} "
                    f"user_input={user_input!r}"
                )
            elif self._verbose_llm:
                self.get_logger().debug(
                    "🎤 [issue 1708] music tool + speak_text "
                    "(backing mode, lyrics allowed) — both ran. "
                    f"tools={list(tools_called)!r}"
                )
        # ADR-0148 (06.10 set98207/set97492, 15:30 UTC): о музыке хода
        # говорит код по результату dj_set/request_music (MusicTurn
        # исполнителя тулов) и по словам человека, а не текст модели;
        # ответ модели отзывается из истории.
        music_phrase = executor_turn_reply(
            getattr(self, "_tool_executor", None),
            (raw_user_command, user_input),
            getattr(self, "_turn_set_tracks", None),
            speak_text_real,
            tools_called,
        )
        if music_phrase is not None:
            self.get_logger().warning(
                f"🎛️ [music turn] фраза кода вместо ответа модели: {music_phrase!r}"
            )
            self._retract_rejected_reply()
            self._speak_direct(music_phrase)
            return
        if speak_text_real > 0:
            self.get_logger().info(final_text_skipped_log(spoken, self._verbose_llm))
            return
        # Два guard'а, которые обязаны отработать ДО
        # action-claim: их вход — не речь вообще, и остальные детекторы
        # его не узнают (ищут глаголы или мета-обещания).
        #
        # * #2760 — модель написала вызов тула ТЕКСТОМ
        #   (``<function_calls><invoke name="register_speaker">…``) при
        #   tools=[]. Первым: прогон 35704637846 показал, как разметка
        #   уходит в TTS двумя чанками. Штатно её разбирает цикл тулов
        #   (``markup_recovery``), сюда она доезжает, только если имя
        #   тула не опознано.
        # * #2175 — MiniMax regurgitates ``<system>...</system>`` вместо
        #   ответа; без ретрая Yandex→MiniMax fallback озвучивал шаблон.
        #
        # Список, а не две ветки подряд: порядок виден одной строкой, и
        # третий такой guard не добавляет ветвления в и без того тяжёлый
        # ``_handle_result`` (ADR-0021, cc_budget).
        for _pre_speech_guard in (
            self._check_tool_call_markup_and_retry,
            self._check_system_template_regurgitate_and_retry,
        ):
            if _pre_speech_guard(
                spoken=spoken,
                user_input=raw_user_command or user_input,
                tools_called=tools_called,
                speak_text_real=speak_text_real,
            ):
                return
        # Issue #992 Bug E — «отчитался о действии, но не вызвал тул».
        # Live 30.08: «Точка сохранена.» / «Точка удалена.» / ««Тисбит»
        # удалён из медиатеки.» — всё с tools=[]. Один ретрай.
        #
        # CC-budget: прямые вызовы здесь +3 (две ``if spoken and ...``
        # ветки), baseline скорректирован 65 → 68. Заворачивать в
        # helper нельзя — test_dialogue_retry_flag_wiring
        # ::test_guard_call_sites_live_in_handle_result требует, чтобы
        # ВСЕ guard-методы _check_*_and_retry были видны прямо из тела
        # _handle_result через AST-обход (инцидент 30.08.2026: блок
        # вызова Bug C′ засунули в __init__, нода не поднималась).
        if spoken and self._check_unbacked_action_claim_and_retry(
            spoken=spoken,
            user_input=raw_user_command or user_input,
            tools_called=tools_called,
        ):
            return
        # Issue #2780 п.3 (память) + #2949 (generic) — общая точка входа
        # для fallback'ов «ретрай уже потрачен, а claim повторился».
        # Диспетчер, а не два отдельных ``if`` подряд: ``_handle_result``
        # грандфазерен в cc_budget_baseline.json на CC=85 с открытой
        # картой рефакторинга #2556, и растить его ради второй категории
        # нечестно — ветвление живёт в
        # ``_publish_action_claim_fallback_if_needed``.
        if spoken and self._publish_action_claim_fallback_if_needed(
            spoken=spoken,
            tools_called=tuple(tools_called or ()),
            user_input=user_input,
            raw_user_command=raw_user_command,
            has_error=result.error is not None,
            speak_text_real=speak_text_real,
            tool_error_occurred=tool_error_occurred,
        ):
            return
        # Issue #2549 — широкий anti-hallucination guard для action-claim.
        # Узкий Bug E guard выше ловит только случаи, где И запрос юзера,
        # И утверждение LLM попадают в :data:`ACTION_CLAIM_RULES`. LLM часто
        # отчитывается о действии в свободной форме («проверю состояние и
        # перезапущу») при пустом ``tools_called``. Это guard-fallback по
        # action-глаголам в spoken (см. :func:`detect_universal_action_claim`).
        # На ошибке LLM (``result.error``) не ретраим.
        if (
            spoken
            and result.error is None
            and self._check_universal_action_claim_and_retry(
                spoken=spoken,
                user_input=raw_user_command or user_input,
                tools_called=tools_called,
                tool_error_occurred=tool_error_occurred,
            )
        ):
            return
        # Issue #2559 — phantom-action guard (общий, НЕ music-only).
        # Round 3 live (15.09.2026, Vision Pi 10.1.1.21): 6 случаев за час
        # «сейчас перезапущу / сделал / подложу» при ``tools_called=[]``.
        # Bug E узкий; phantom-action закрывает ОБЩИЙ класс
        # «обещал действие — но не вызвал тул» (issue #2559, расширение
        # Bug E на ВСЕ action-verb'ы). CC-budget: +1
        # прямая ветка в этом catch-site, _handle_result уже в exemptions
        # cc_budget_baseline.json (DialogueNode._handle_result: 75).
        if spoken and self._check_phantom_action_and_retry(
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
            # тул запуска музыки и вернуть 'done' без speak_text (новый
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
                # это CRITICAL-промпт, а не слова юзера. Стоп-команду ниже
                # смотрим по оригинальной команде юзера.
                _source = raw_user_command or user_input
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
                            "response — стоп музыки владельцем плеера"
                        )
                        # ADR-0150 PR-7c ч.4б: dj_set(stop) + stop_music, как Tier-1.
                        try:
                            self._stop_music_by_owner("user_stop_command")
                        except Exception as exc:  # noqa: BLE001
                            self.get_logger().warning(
                                f"🎵 stop fallback failed: {exc}"
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
                    # retry теперь живёт ВНУТРИ AgentCore._run_with_tools
                    # (issue #1217) и учит модель в том же turn; служебные
                    # записи в диалоговую память не нужны (правило control
                    # traffic — как для DJ-auto в agent_core.process_input).
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
                try:
                    self._publish_response(EMPTY_REPLY_PHRASE, animation="neutral")
                except Exception as exc:  # noqa: BLE001
                    try:
                        self.get_logger().warning(
                            f"⚠️ empty-fallback publish failed: {exc}"
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
                        "a music-launch tool when stop_music is used."
                    )
                return
        # Issue #980 — split into chunks and publish as a single TTS batch so
        # that the batch closes only after the last chunk.
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
        # Issue #2874 — внутри хода текст придерживается до решения
        # post-turn гуардов о ретрае (см. ``_voice_turn_text``).
        self._voice_turn_text(spoken, user_input=raw_user_command or user_input)
        self.get_logger().info(
            f"📤 LLM OUTPUT: {spoken[:200]!r}" if self._verbose_llm
            else f"✅ Turn done. Response: {spoken[:80]!r}")

    # ── Issue #2874 — свободный текст хода ждёт решения гуардов ─────────

    def _voice_turn_text(self, spoken: str, *, user_input: Optional[str]) -> None:
        """Озвучить свободный текст хода — или придержать до гуардов.

        Внутри ``_run_turn`` (открыт :class:`TurnSpeechHold`) текст ждёт
        post-turn гуардов: ход, за которым последует синхронный ретрай,
        свой текст не озвучивает. Вне хода (прямой вызов
        ``_handle_result``) — публикуем сразу, как раньше.
        """
        hold = getattr(self, "_turn_speech_hold", None)
        if hold is None:
            self._publish_turn_text(spoken, user_input=user_input)
            return
        hold.hold(spoken, user_input)

    def _release_turn_speech(
        self, hold: Optional[TurnSpeechHold], *, retry_dispatched: bool,
    ) -> None:
        """Выпустить (или выбросить) придержанный текст хода (issue #2874)."""
        if hold is None:
            return
        if getattr(self, "_turn_speech_hold", None) is hold:
            self._turn_speech_hold = None
        if hold.text is None:
            return
        self._publish_turn_text(
            hold.text,
            user_input=hold.user_input,
            retry_dispatched=retry_dispatched,
            retracted=hold.retracted,
        )

    def _music_reply_context(self) -> bool:
        """Issue #2874 — музыкальный контекст по снимку плеера: идёт сет или играет музыка."""
        snapshot = getattr(self, "_music_player_state", None)
        return bool(snapshot is not None and (snapshot.dj or snapshot.is_playing()))

    def _publish_turn_text(
        self, spoken: str, *, user_input: Optional[str],
        retry_dispatched: bool = False, retracted: bool = False,
    ) -> None:
        """Решить через :func:`decide_turn_speech`, что звучит, и опубликовать."""
        chunks = split_into_chunks(spoken)
        text = decide_turn_speech(
            spoken,
            n_chunks=len(chunks),
            retry_dispatched=retry_dispatched,
            retracted=retracted,
            budget_exhausted=getattr(self, "_retry_budget_exhausted_in_turn", False),
            music_context=self._music_reply_context(),
            lyrics_requested=wants_lyrics(user_input),
        )
        if text is None:
            self.get_logger().info(
                "🔇 [issue 2874] ответ хода не озвучиваю (ретрай/отзыв гуардом, "
                f"retry={retry_dispatched} retracted={retracted}): {spoken[:120]!r}"
            )
            return
        if text != spoken:
            self.get_logger().warning(
                f"✂️ [issue 2874] {len(chunks)} чанков → первая фраза "
                f"{text!r} (было {len(spoken)} симв.)"
            )
            chunks = split_into_chunks(text)
        if len(chunks) <= 1:
            self._publish_response(chunks[0] if chunks else text)
            return
        total = self._publish_response_batch(chunks)
        self.get_logger().info(
            f"📦 [dialogue_node] TTS batch: {total} chunks (issue #980)"
        )

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self._dsm.current_state.name
        self._state_pub.publish(msg)
    def _on_startup_greeting(self) -> None:
        """Одноразовое приветствие при старте (issue #1003, редизайн 06.08).

        Раньше была отдельная нода приветствия — она публиковала
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

        # «Услышал»-сигнал, как у обычного диалога (earcon рефлекса, а не отдельный триггер звука).
        self._express_earcon("heard")

        # Через 2с — радостный звук, ещё через 1.5с — фраза (как в
        # оригинальной ноды приветствия до рефакторинга).
        self._greeting_timer = self.create_timer(
            2.0, self._on_startup_greeting_finish
        )

    def _on_startup_greeting_finish(self) -> None:
        """Вторая фаза приветствия: радостный звук cute/very_cute.

        ADR-0101 §3.3.3 / PR-B: gating через OccasionGate.may_speak.
        Старый флаг ``_startup_greeting_fired`` остаётся как fallback
        (двойная защита на случай сбоя OccasionGate).
        """
        # ADR-0101 §3.3.3 / PR-B: шов «можно ли заговорить» через Повод.
        # Повод startup — не user-initiated, payload содержит финальную фразу.
        verdict = self._occasion.may_speak(
            Occasion(
                kind="startup",
                is_user_initiated=False,
                payload={"text": self._startup_greeting_text},
            )
        )
        if verdict.kind != VerdictKind.ALLOW:
            # DEFER (one-shot уже consumed / кулдаун) или REFUSE (стаб).
            self.get_logger().info(
                f"startup greeting deferred: {verdict.reason}"
            )
            return
        # Фиксируем факт «заговорили» — один раз за uptime (one-shot).
        self._occasion.mark_consumed(Occasion(kind="startup"))

        self._cancel_greeting_timer()
        self._express_earcon("done")
        self._greeting_timer = self.create_timer(
            1.5, self._on_startup_greeting_speak
        )

    def _on_startup_greeting_speak(self) -> None:
        """Третья фаза: публикуем случайную фразу приветствия."""
        self._cancel_greeting_timer()
        phrase = pick_greeting(self._startup_greeting_text)
        self.get_logger().info(f"🗣 Startup greeting: {phrase!r}")
        self._publish_response(phrase)

    # ------------------------------------------------------------------
    # Встреча лицом — робот заговаривает первым (issue #2599 PR-C)
    # ------------------------------------------------------------------

    def _subscribe_vision_events(self, cbg: Any) -> None:
        """Подписка на лицевые события — источник повода «Встреча».

        Топик общий с person-детекцией и сыплет ~5 событий в секунду на
        человека, поэтому подписка ЛЁГКАЯ: колбэк отсеивает всё, кроме
        маркера ``{"encounter": "start"}`` в ``attributes_json``, и
        только он доходит до OccasionGate (ADR-0123 §3).

        Импорт сообщения — локальный и защищённый: ``rob_box_voice`` не
        зависит от ``rob_box_perception_msgs`` в package.xml, и на Main
        Pi (где лицевой ноды нет) этого типа может не быть. Без лица
        диалог обязан работать как раньше.
        """
        try:
            from rob_box_perception_msgs.msg import VisionEvent as _VisionEvent

            self.create_subscription(
                _VisionEvent, "/vision/hailo/events",
                self._on_vision_event, 10, callback_group=cbg)
            self.get_logger().info(
                "👁 [issue 2599] подписка на /vision/hailo/events — "
                "робот может заговорить первым при появлении человека"
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ [issue 2599] лицевой повод недоступен ({exc!r}): "
                f"робот продолжит отвечать только на голос"
            )

    def _on_vision_event(self, msg: Any) -> None:
        """Отсеять всё, кроме маркера начала Встречи.

        Колбэк горячий: топик общий с person-детекцией и сыплет ~5
        событий в секунду на человека. Поэтому здесь только дешёвая
        проверка. Приветствие Встречи удалено (В12′: робот сам в разговор не вступает; #3695) — маркер Встречи
        нужен только шву лица (ADR-0139).

        ADR-0139 §2.3: дополнительно кладём наблюдение лица в шов
        «Знакомый» (``self._identity.note_face_seen``), чтобы голосовой
        ``_handle_tentative_speaker`` мог снять переспрос #2809 при
        свежем face-hint с тем же именем (issue #3024). Колбэк дешёвый
        — операция in-memory, без сети/БД.

        #3695: каждое лицо из кадра ложится в ``_faces`` — «тот же собеседник» для бэклога и вопроса об имени.
        """
        note_face(self, msg)
        marker = parse_meeting_marker(
            event_type=getattr(msg, "event_type", "") or "",
            attributes_json=getattr(msg, "attributes_json", "") or "",
            source_camera=getattr(msg, "source_camera", "") or "",
        )
        if marker is None:
            return
        # ADR-0139 §2.3 — note_face_seen через тот же подписочный путь,
        # что и прежняя Встреча. Не плодим новый топик
        # /perception/face/meeting (контракт Vision Pi, ADR-0139 §5.2).
        if self._face_voice_hint_enabled:
            try:
                self._identity.note_face_seen(
                    FaceSignal(
                        person_id=marker.person_id,
                        name=marker.name or None,
                        similarity=float(marker.similarity or 0.0),
                        is_new=bool(marker.is_new),
                        source_camera=marker.source_camera,
                        captured_at=time.time(),
                    )
                )
            except Exception as exc:  # noqa: BLE001
                # ADR-0139 §2.5: hint — деградируемая функциональность,
                # падение не должно ронять колбэк.
                self.get_logger().debug(
                    f"👤 [ADR-0139] note_face_seen failed (ignored): {exc!r}"
                )
        # Знакомый после долгой разлуки → одна фраза (исключение Шифу 21.10 из В12′); решение — core/meeting.
        greet_node_on_meeting(self, marker)

    def _cancel_greeting_timer(self) -> None:
        """Отменить одноразовый таймер приветствия, если он создан."""
        timer = getattr(self, "_greeting_timer", None)
        if timer is not None:
            try:
                timer.cancel()
            except Exception:  # noqa: BLE001 — таймер может быть уже сработавшим
                pass
            self._greeting_timer = None

    def _publish_response(
        self, text: str, animation: str = "neutral", *, language: str | None = None
    ) -> str:
        """Single-chunk publish — kept for backwards compatibility. Возвращает ``speech_id`` реплики (#3771).

        For multi-chunk turns prefer :meth:`_publish_response_batch` which
        attaches a shared ``batch_id`` so ``tts_node`` can close the batch
        once the last chunk lands (issue #980).

        Issue #1195 — если последний вход пришёл из Telegram-чата,
        кладём ``tg_chat_id`` в payload: telegram_node использует его для
        маршрутизации эхо-ответа в правильный чат.
        """
        self._turn_stages().expect_speech()
        speech_id = str(uuid.uuid4())
        msg = String()
        msg.data = build_ssml_payload(
            text,
            animation,
            tg_chat_id=self._active_tg_chat_id,
            voice=None,  # текущий голос ведёт tts_node (VoiceOwner)
            language=language,
            epoch=self._session_epoch_gate().epoch_for_dispatch(),
            speech_id=speech_id,
        )
        self._response_pub.publish(msg)
        return speech_id

    def _publish_response_batch(self, chunks: List[str], animation: str = "neutral") -> int:
        """Publish ``chunks`` as a single TTS batch (issue #980).

        Each chunk carries the same ``batch_id`` plus 1-based ``batch_index``
        and ``batch_total`` counters. ``tts_node`` echoes those fields on
        ``/voice/tts/finished`` and marks the final one ``batch_complete``.
        Returns the number of chunks published.
        """
        if not chunks:
            return 0
        self._turn_stages().expect_speech()
        batch_id = str(uuid.uuid4())
        total = len(chunks)
        for idx, chunk in enumerate(chunks, start=1):
            msg = String()
            msg.data = build_ssml_payload(
                chunk,
                animation,
                batch_id=batch_id,
                batch_index=idx,
                batch_total=total,
                tg_chat_id=self._active_tg_chat_id,
                voice=None,  # текущий голос ведёт tts_node (VoiceOwner)
                epoch=self._session_epoch_gate().epoch_for_dispatch(),
            )
            self._response_pub.publish(msg)
        return total

    def _publish_action_claim_fallback_if_needed(
        self,
        *,
        spoken: str,
        tools_called: Tuple[str, ...],
        user_input: Optional[str],
        raw_user_command: Optional[str],
        has_error: bool,
        speak_text_real: int,
        tool_error_occurred: bool = False,
    ) -> bool:
        """Issue #2780 п.3 + #2949 — диспетчер fallback'ов после ретрая.

        Одноразовый ретрай action-claim'а (``_action_claim_retry_used``)
        тратится на ПЕРВОМ ложном заявлении в ходе. Если модель повторяет
        заявление второй репликой того же хода, guard уже молчит, и без
        подмены ``spoken`` юзер услышит уверенное «всё готово» / «всё на
        месте» про несделанное. Каждая категория
        :data:`ACTION_CLAIM_RULES`, у которой есть честная констатация на
        замену, получает свой ``_publish_*_fallback_if_needed``; здесь —
        единственная точка входа из ``_handle_result``.

        Порядок: память (#2780) → generic anti-hallucination (#2949).
        Ветки не пересекаются — у каждой свой контекстный гейт (память —
        «запомни/запиши/сохрани» не про точку и не про трек, generic —
        любой тул из ``CLAIM_JUSTIFYING_TOOLS`` вызван, но ошибся ИЛИ не
        вызван вовсе), так что порядок тут даёт стабильность, а не
        приоритет. Музыкальный fallback (#2548) удалён в ADR-0149 PR-13a.

        Returns:
            ``True`` — fallback опубликован, вызывающий обязан сделать
            ``return`` из ``_handle_result`` (оригинальный ``spoken``
            в TTS НЕ идёт).
        """
        kwargs = dict(
            spoken=spoken,
            tools_called=tools_called,
            user_input=user_input,
            raw_user_command=raw_user_command,
            has_error=has_error,
            speak_text_real=speak_text_real,
        )
        if self._publish_fact_memory_save_fallback_if_needed(**kwargs):
            return True
        return self._publish_universal_action_claim_fallback_if_needed(
            **kwargs,
            tool_error_occurred=tool_error_occurred,
        )

    def _publish_universal_action_claim_fallback_if_needed(
        self,
        *,
        spoken: str,
        tools_called: Tuple[str, ...],
        user_input: Optional[str],
        raw_user_command: Optional[str],
        has_error: bool,
        speak_text_real: int,
        tool_error_occurred: bool = False,
    ) -> bool:
        """Issue #2949 — fallback после НЕудачного universal-action-claim ретрая.

        Живой лог 24.09: ``save_arrangement_preset`` вернул отказ
        («недоступен»), LLM ответила «Записала пресет…», один ретрай
        (:meth:`_check_universal_action_claim_and_retry`) ушёл с
        CRITICAL-просьбой либо повторить, либо честно сказать о
        неудаче — а модель СНОВА заявила об успехе («Записала твоё
        предпочтение…»). Guard уже потратил свой одноразовый ретрай
        (``_universal_action_claim_retry_used``) и молчит; без подмены
        ``spoken`` юзер слышит вторую по счёту ложь подряд.

        Условия — ретрай уже потрачен, ход НЕ с ошибкой LLM, и :func:`detect_universal_action_claim` (с той же
        ``tool_error_occurred``) СНОВА считает заявление непокрытым.
        Один и тот же детектор для ретрая и для fallback'а — критерий
        «подкреплено» не может разъехаться между двумя местами.
        """
        if has_error or not getattr(self, "_universal_action_claim_retry_used", False):
            return False
        hit = self._universal_claim_hit(
            spoken=spoken,
            tools_called=tools_called,
            tool_error_occurred=tool_error_occurred,
        )
        if hit is None:
            return False
        # Issue #3165 — в ходе ни один тул не исполнялся и не падал:
        # «Не получилось выполнить» было бы неправдой (попытки не было).
        fallback = build_action_claim_failure_fallback(
            hit, nothing_attempted=not tools_called and not tool_error_occurred
        )
        self.get_logger().warning(
            "🛟 [issue 2949 fallback] action-claim повторился после "
            f"ретрая (tool_error_occurred={tool_error_occurred!r}) — "
            f"публикую честную неудачу вместо claim'а: spoken={spoken[:80]!r}"
        )
        try:
            self._publish_response(fallback, animation="neutral")
        except Exception as exc:  # noqa: BLE001
            try:
                self.get_logger().warning(
                    f"⚠️ fallback publish failed: {exc}"
                )
            except Exception:
                pass
        return True

    @staticmethod
    def _fact_memory_save_rule():
        """Issue #2780 п.3 — правило ``fact_memory_save`` из таблицы.

        Единственный источник правды и для гейта по ``user_re``, и для
        списка закрывающих тулов (``memory_save`` — факт,
        ``register_speaker`` — имя) лежит в :data:`ACTION_CLAIM_RULES`.
        Копировать оттуда что-либо сюда нельзя: добавленный в правило
        тул молча перестанет закрывать заявку, и робот начнёт извиняться
        за запись, которая на самом деле прошла.
        """
        return next(
            (r for r in ACTION_CLAIM_RULES
             if r.category == "fact_memory_save"),
            None,
        )

    def _fact_memory_save_tools(self) -> frozenset:
        """Issue #2780 п.3 — тулы, закрывающие заявку ``fact_memory_save``."""
        rule = self._fact_memory_save_rule()
        return rule.tools if rule is not None else frozenset()

    def _fact_memory_fallback_eligible(
        self, user_input: Optional[str],
    ) -> bool:
        """Issue #2780 п.3 — юзер в этом ходе правда просил ЗАПОМНИТЬ?

        Гейт — ``user_re`` того же правила («запомни/запиши/сохрани/не забудь/заметь», с
        negative lookahead на точку/трек — у них свои правила и свои
        тулы). Без этого гейта потраченный на waypoint-claim ретрай
        заставил бы «Не получилось точно сохранить факт» звучать в
        совершенно постороннем ходе.
        """
        rule = self._fact_memory_save_rule()
        if not user_input or rule is None:
            return False
        try:
            return bool(rule.user_re.search(user_input))
        except Exception:  # noqa: BLE001
            return False

    def _publish_fact_memory_save_fallback_if_needed(
        self,
        *,
        spoken: str,
        tools_called: Tuple[str, ...],
        user_input: Optional[str],
        raw_user_command: Optional[str],
        has_error: bool,
        speak_text_real: int,
    ) -> bool:
        """Issue #2780 п.3 — fallback spoken после НЕудачного memory-ретрая.

        Прогон 35734532425, акт «Знакомство», шаг ``n206_boris_memory`` —
        один ход, три реплики подряд:

        1. «Запомнил: Спартак с 98-го, пицца раз в неделю.» ``tools=[]``
           → Bug E guard поймал, потратил одноразовый ретрай;
        2. ретрай честно признался («у меня в памяти сбой, проверь»);
        3. «Всё на месте, Спартак и пицца в памяти, запись подтверждена.»
           ``tools=['memory_context']`` — ЧТЕНИЕ памяти, не запись.

        На третьей реплике ``_action_claim_retry_used`` уже True: guard
        в этом ходе больше не выстрелит, и юзер уходит с верой в запись,
        которой нет. PR #2782 научил ``claim_re`` ловить формулировку
        («всё на месте», «запись подтверждена») — эта ветка подменяет
        такой ``spoken`` честной констатацией ПЕРЕД публикацией в TTS.

        Условия все ОДНОВРЕМЕННО:

        1. ``_action_claim_retry_used == True`` — ретрай уже потрачен
           в этой user-turn;
        2. ни один тул из ``rule.tools`` (``memory_save`` /
           ``register_speaker``) не вызван. ВАЖНО: здесь НЕЛЬЗЯ требовать
           пустой ``tools_called`` — в живом логе он равен ``['memory_context']``
           (чтение), и проверка «список пуст» пропустила бы ровно тот
           случай, ради которого карточка заведена;
        3. ``speak_text_real == 0`` (как у остальных guard'ов цепочки);
        4. юзер в этом ходе просил запомнить — ``user_re`` правила
           (см. :meth:`_fact_memory_fallback_eligible`);
        5. ``spoken`` всё ещё матчит ``claim_re`` категории
           ``fact_memory_save``;
        6. нет ``result.error`` — честное сообщение об ошибке LLM
           не маскируем.

        Returns:
            ``True`` — fallback опубликован, вызывающий должен вернуть
            ``return`` из ``_handle_result``.
        """
        if (
            has_error
            or speak_text_real != 0
            or (set(tools_called or ()) & self._fact_memory_save_tools())
            or not getattr(self, "_action_claim_retry_used", False)
        ):
            return False
        if not self._fact_memory_fallback_eligible(
            raw_user_command or user_input,
        ):
            return False
        if not spoken_matches_claim_category("fact_memory_save", spoken):
            return False
        self._emit_fact_memory_save_fallback(spoken)
        return True

    def _emit_fact_memory_save_fallback(self, spoken: str) -> None:
        """Issue #2780 п.3 — публикуем честный fallback + warning в лог."""
        fallback = build_fact_memory_save_fallback()
        self.get_logger().warning(
            f"🛟 [issue 2780 fallback] подтверждение записи в память "
            f"повторилось после ретрая без memory_save — публикую "
            f"констатацию вместо claim'а: spoken={spoken[:80]!r}"
        )
        try:
            self._publish_response(fallback, animation="neutral")
        except Exception as exc:  # noqa: BLE001
            try:
                self.get_logger().warning(
                    f"⚠️ fallback publish failed: {exc}"
                )
            except Exception:
                pass

    def _speak_direct(self, text: str, language: str | None = None) -> str:
        """Озвучить текст напрямую (без AgentCore).

        ``language`` (AV-28) — язык, на котором НАПИСАН ``text``. Уходит в
        payload и дальше в TTS: без него формализатор переписывал реплику
        на французский, а tts_node синтезировал её со статическим
        ``minimax_language="ru"``. ``None`` — обычная русская реплика
        робота, поведение прежнее.

        Возвращает ``speech_id`` последнего чанка (по нему фразу узнают в ``SpeechEvent``) или ``""``.
        """
        speech_id = ""
        for chunk in split_into_chunks(text):
            speech_id = self._publish_response(chunk, language=language)
        return speech_id

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
        # ``AgentCore`` used to wrap LLM exceptions into a plain
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

    def _detect_pitch_intent(self, text: str):
        """Detect pitch adjustment intent from user text.

        Delegates to
        :func:`rob_box_voice.core.dialogue_helpers.detect_pitch_intent`
        (TD-1 decomposition).
        """
        return detect_pitch_intent(text)

    def _on_mcp_tools_update(self, msg) -> None:
        """Parse MCP tools JSON and update ``available_tools``."""
        try:
            data = getattr(msg, "data", "") or "[]"
            tools = json.loads(data)
            if isinstance(tools, list):
                self.available_tools = tools
                self.mcp_tools_available = True
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

    def _reset_dialogue_session(self, *, announce: bool = True):
        """Issue #XXXX — сброс текущей диалоговой сессии.

        ``announce=False`` (issue #2890, E2E-сброс между актами) — тот же
        сброс, но молча: без фразы «Начинаю новую сессию…», которая иначе
        попала бы в запись первого шага акта.
        Возвращает future очистки окна ходов (``None``, если asyncio-цикла
        нет) — E2E-путь ждёт её, чтобы подтверждать сброс, а не надеяться.

        Полный сброс состояния текущего диалога: in-flight turn, DSM → IDLE,
        бэклог-аккумулятор, speaker-состояние, таймер сессии и история
        (in-memory окно ходов через ``AgentCore.clear_history``). LLM не
        вызывается —
        вместо этого говорим детерминированное подтверждение.

        Issue #1563 / ADR-0150 PR-3 — подтверждение «Начинаю новую сессию…»
        несёт НОВУЮ эпоху, а barge-in от wake-word той же фразы отменяет
        реплики не новее старой (``speech_client.cancel(epoch=...)``):
        подтверждение не задевается, окно игнора STOP (#1563) не нужно.
        """
        # 0. Issue #2835 — новое поколение сессии ДО отмены хода: его
        # ``finally`` (отмена асинхронная) уже увидит, что сессия сброшена,
        # и не пошлёт [CRITICAL]-ретрай; ходы/ретраи, стоящие в loop,
        # отбросятся на старте.
        old_epoch = self._session_epoch_gate().current
        self._session_epoch_gate().advance()
        # 1. Отменяем in-flight turn (barge-in + stop TTS + release effects);
        # речь отменяется только старой эпохи — подтверждение уже новой.
        self._cancel_run("new session reset", stop_tts=True, speech_epoch=old_epoch)
        # 1b. Issue #2835 — музыка прошлой сессии остановлена.
        self._reset_session_music()
        # 2. Бэклог-аккумулятор фоновой речи (если реализован).
        backlog = getattr(self, "_backlog", None)
        if backlog is not None:
            backlog.clear()
        self._pending_unaddressed = None
        # 2b. Журнал действий роутера (#3165): пересказ «я остановил» после сброса — уже не этой сессии.
        self._media_actions = deque(maxlen=16)
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
        # 6. История диалога — in-memory окно ходов в AgentCore.
        #    Обёртка остаётся async для совместимости с loop-диспетчером.
        loop = getattr(self, "_loop", None)
        clear_future = None
        if loop is not None:
            try:
                clear_future = asyncio.run_coroutine_threadsafe(
                    self._clear_session_turns(), loop
                )
            except Exception:  # noqa: BLE001
                pass
        # 7. Публикуем состояние и подтверждение.
        self._publish_state()
        if announce:
            self._publish_response(
                "Начинаю новую сессию. Всё, что было до этого, забыто.",
                animation="neutral",
            )
        return clear_future

    async def _clear_session_turns(self) -> bool:
        """Очистить in-memory окно ходов текущей сессии. ``True`` — очищено."""
        try:
            core = getattr(self, "_core", None)
            if core is not None:
                core.clear_history()
            self.get_logger().info(
                "🧹 [new-session] in-memory turn window cleared"
            )
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ [new-session] clear_history failed: {exc}"
            )
            return False

    #: Issue #2890 — сколько ждать очистки окна ходов в asyncio-цикле.
    _E2E_RESET_TIMEOUT_S = 5.0

    def _reset_session_for_e2e_act(self, token: str) -> bool:
        """Issue #2890 — тихий сброс сессии перед актом E2E. ``True`` — сброшено.

        Вызывается из ``parameters_callback`` (``e2e_session_reset_token``).
        До этой правки акты E2E были изолированы только по БД дикторов
        (e2e_mode стирает ``/data/speakers.e2e.db``), а окно ходов LLM
        переживало переход: в акте 2b (run 35903232434) LLM видела ход
        акта 2 «(Саша) поищи … про чай» и сказала НОВОЙ Саше факт
        старой. Сброс — тот же ``_reset_dialogue_session``, что у фразы
        «новая сессия», но без голосового подтверждения.

        Пустой токен — no-op (дефолт параметра при старте ноды). ``False``
        — окно ходов не подтверждено очищенным: колбэк отклонит значение,
        харнесс прочитает параметр обратно и зафейлит акт.
        """
        token = token.strip()
        if not token:
            return True
        try:
            clear_future = self._reset_dialogue_session(announce=False)
            cleared = bool(
                clear_future is not None
                and clear_future.result(timeout=self._E2E_RESET_TIMEOUT_S)
            )
        except Exception as exc:  # noqa: BLE001 — сброс не должен ронять ноду
            self.get_logger().error(
                f"❌ [issue 2890] e2e-сброс сессии (token={token!r}) "
                f"провалился: {type(exc).__name__}: {exc}"
            )
            return False
        if not cleared:
            self.get_logger().error(
                f"❌ [issue 2890] e2e-сброс сессии (token={token!r}): окно "
                "ходов LLM НЕ подтверждено очищенным — акт увидел бы ходы "
                "прошлого акта"
            )
            return False
        # WARNING, не info: строка обязана быть видна в `docker logs
        # voice-assistant` без фильтров — по ней проверяют изоляцию акта.
        self.get_logger().warning(
            f"🧪 dialogue_node: e2e-сброс сессии перед актом "
            f"(token={token!r}) — окно ходов LLM очищено, speaker-состояние "
            "и DSM сброшены"
        )
        return True

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

    def _take_session_handover(self) -> bool:
        """Передал ли текущий ход сессию следующему (#2939).

        Живой лог 24.09: barge-in отменил висящий ход, приём новой фразы
        перевёл DSM в DIALOGUE, а ``finally`` отменённого хода закрыл
        сессию (DIALOGUE_END → IDLE). Следующий ход пришёл в IDLE,
        ``AgentCore`` не позвал LLM — пустой ответ за 1 мс и «Принял.».
        Отметка ставится и тогда, когда ход успел доиграть сам, а отмена
        опоздала: новая фраза всё равно уже принята — закрывать нечего.
        """
        task = asyncio.current_task()
        with self._task_lock:
            marked = getattr(self, "_handed_over_task", None) is task
            if marked:
                self._handed_over_task = None
        return marked

    def _cancel_run(
        self,
        reason: str,
        *,
        stop_tts: bool = True,
        hand_over: bool = False,
        speech_epoch: Optional[int] = None,
    ) -> None:
        """Cancel the in-flight LLM turn, optionally muting TTS.

        ``hand_over=True`` (issue #2939) — отмена ради новой фразы, которая
        сама продолжит сессию: отменённый ход не должен её закрывать.

        ADR-0150 §6.2 (PR-5) — новая адресованная фраза отменяет ход
        БЕЗ остановки речи (``stop_tts=False``): текущая реплика
        дочитывается, ответ нового хода встаёт после неё.
        ``stop_tts=False`` must still release the TTS/sound awaiters —
        skipping that hangs ``speak_helpers._tts_events`` forever and the
        robot never speaks again.
        """
        self._run_cancelled = True
        with self._task_lock:
            task = self._run_task
            if hand_over and task is not None and not task.done():
                self._handed_over_task = task
        if task is not None and not task.done():
            self.get_logger().info(f"🛑 Cancel: {reason}")
            self._loop.call_soon_threadsafe(task.cancel)
        if stop_tts:
            # ADR-0150 PR-3 — отмена речи личности по эпохе (не STOP «всего»):
            # реплики оператора/Telegram и новой эпохи не трогаются.
            speech_client.cancel(
                self,
                agent="personality",
                epoch=self._session_epoch_gate().current if speech_epoch is None else speech_epoch,
                reason=reason,
            )
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
        # Issue #1986 §5.3: AgentCore больше не владеет таймаутом — нода
        # сама гонит DSM (ровно то, что делал удалённый ``check_timeout``:
        # ``DialogueStateMachine.check_inactivity_timeout``).
        if self._dsm.check_inactivity_timeout(self._dialogue_timeout_s):
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


# ADR-0145 §4 (P1 step 1) -- ``_DialogueSttHost`` (the adapter from the
# ``SttAdmissionHost`` Protocol to DialogueNode, including the lock
# discipline docstring: ``_speaker_lock`` in ``accumulate_without_wake``,
# ``_task_lock`` in ``enqueue_pending``, no inversion) moved to
# :mod:`rob_box_voice.core.stt_admission_host` (pure move, no behaviour
# change) and is re-exported via the import above.


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DialogueNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    exit_code = 0
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:  # noqa: BLE001 — issue #2713: an uncaught spin()
        # exception used to escape main() silently, leaving a zombie
        # process the container healthcheck reports as healthy. Log
        # and exit non-zero so launch/docker observe the death instead.
        logging.getLogger(__name__).exception(
            "dialogue_node: executor.spin() crashed, exiting"
        )
        exit_code = 1
    try:
        node.shutdown_asyncio_loop(wait=False)
    except Exception:  # noqa: BLE001
        logging.getLogger(__name__).exception("dialogue_node: shutdown failed")
    rclpy.shutdown()
    if exit_code:
        sys.exit(exit_code)

if __name__ == "__main__":
    main()
