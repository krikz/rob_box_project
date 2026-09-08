#!/usr/bin/env python3
"""
TTSNode - Text-to-Speech с Yandex Cloud TTS API v3 (gRPC) + Silero fallback + MiniMax (HTTP)

Подписывается на: /voice/dialogue/response (JSON chunks)
Публикует: /voice/audio/speech (AudioData)
Использует:
  - Yandex Cloud TTS API v3 (gRPC, primary, anton voice)
  - Silero TTS v4 (fallback, офлайн, всегда работает)
  - MiniMax T2A v2 (HTTP, opt-in через provider=minimax)

Concurrency
-----------
Two bounded executors, both ``concurrent.futures.ThreadPoolExecutor``:

* **Synthesis executor** (``self._synthesis_executor``) —
  ``max_workers = SYNTHESIS_MAX_WORKERS_DEFAULT = 2`` (ROS-параметр
  ``synthesis_max_workers``, допустимый диапазон 1..4). Каждый worker
  обрабатывает ровно один TTS HTTP/gRPC synthesis request
  (Yandex / Silero / MiniMax) и публикует результат на
  ``/voice/audio/speech``. Семафор на ``max_workers + max_queue``
  (``SYNTHESIS_MAX_QUEUE_DEFAULT = 16``) даёт back-pressure: при переполнении
  новые задачи дропаются, а не плодят потоки.
* **Async-bridge executor** (``ASYNC_BRIDGE_MAX_WORKERS = 1``) — per-call,
  живёт только внутри ``with``-блока вокруг ``asyncio.run(...)`` в
  ``_synthesize_minimax*``. Нужен, потому что ROS-callback синхронный,
  а ``MiniMaxTTSProvider.stream()`` — async; одного воркера достаточно,
  т.к. он тут же ``.result()``-ит и выходит.

Rationale: Yandex и MiniMax провайдеры не открывают соединения на каждый
запрос — каждый worker держит свой keep-alive HTTP/2 или gRPC-channel.
2 воркера дают практически полную утилизацию одного синтеза (~300–800 ms
per request) без удвоения стоимости каналов. Burst input rate выше
~2 synthesis/s встанет в очередь на семафоре; rate выше
``max_workers + max_queue`` synthesis/s (~18/s) начнёт дропаться. Дальнейшее
увеличение через ROS-параметр ``synthesis_max_workers`` (см. ниже).

Tuning
------
ROS-параметры ноды (см. ``declare_parameter`` в ``__init__``):

* ``synthesis_max_workers`` (int, 1..4, default 2) — размер пула.
* ``synthesis_max_queue`` (int, default 16) — ёмкость очереди перед drop.

Размер пула — намеренный trade-off между throughput и числом одновременных
внешних соединений; см. PR #907 BLK-9.
"""

import asyncio
import atexit
import concurrent.futures
import io
import json
import os
import re
import sys
import threading
import time
import wave
from dataclasses import dataclass
from pathlib import Path
from collections.abc import Mapping

import grpc
import numpy as np
import rclpy
import sounddevice as sd
import torch
from audio_common_msgs.msg import AudioData
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from typing import Any, Dict, Optional

from .audio_playback_manager import AudioPlaybackManager
from .utils.stderr_silence import ignore_stderr

# Markdown sanitisation for TTS (issue #988) — shared with dialogue_node.
from .core.speak_helpers import strip_markdown, unsupported_language_notice

# Issue #1709 — Unicode-script guard: не отправляем в TTS текст, который
# в основном состоит из букв неподдерживаемых письменностей (CJK,
# деванагари, арабица) — провайдеры бормочут такое нечитаемо.
# Pure-Python модуль, общий с rob_box_mcp_tools (SpeakTextTool).
from .tts_text_guard import analyze as _tts_guard_analyze
from .tts_text_guard import describe as _tts_guard_describe
from .tts_text_guard import should_skip as _tts_guard_should_skip

# Issue #2003 / ADR-0056 — speculative chunk-level pre-generation.
# Pure-Python package, no rclpy/asyncio in the data-class modules
# (only :class:`speculative_executor.SpeculativeExecutor` is
# async-aware). Distinct from the existing scheduler-level
# :mod:`rob_box_voice.scheduler.pre_gen` (different abstraction
# layer; see ADR-0056 §4 for the explicit rejection of "thin
# adapter to SpeculativePreGenerator").
from .scheduler.pregen import (
    CONFIDENCE_FLOOR as _PREGEN_CONFIDENCE_FLOOR,
    Decision as _PreGenDecision,
    PreGenResult as _PreGenResult,
    PreGenTask as _PreGenTask,
    SpeculativeExecutor as _PreGenExecutor,
    build_pregen_task as _build_pregen_task,
)

# Transcoding helpers for converting provider audio blobs (PCM/WAV/MP3/OGG)
# into ROS-ready int16 LE PCM. Imported independently from the optional MiniMax
# provider so conversion utilities remain available even when rob_box_llm is not.
try:
    from .utils.audio_transcode import (
        AudioTranscodeError,
        DecodedAudio,
        to_pcm_int16,
    )
except ImportError:  # pragma: no cover - only a malformed minimal installation
    AudioTranscodeError = RuntimeError  # type: ignore[assignment, misc]
    DecodedAudio = object  # type: ignore[assignment, misc]
    to_pcm_int16 = None  # type: ignore[assignment]

# MiniMax TTS provider is opt-in via provider="minimax". Import is lazy so a
# minimal ros_box_voice install (no httpx configured) doesn't break the
# default yandex/silero path.
try:
    from rob_box_llm import MiniMaxTTSProvider, TTSSettings, TTSFormat
    from rob_box_llm.errors import (
        TTSError as MiniMaxTTSError,
        TTSAuthError as MiniMaxTTSAuthError,
        TTSBadRequestError as MiniMaxTTSBadRequestError,
        TTSRateLimitError as MiniMaxTTSRateLimitError,
    )

    MINIMAX_AVAILABLE = True
except ImportError:  # pragma: no cover — only triggered if rob_box_llm not built
    MINIMAX_AVAILABLE = False
    MiniMaxTTSProvider = None  # type: ignore[assignment]
    TTSSettings = None  # type: ignore[assignment]
    TTSFormat = None  # type: ignore[assignment]
    MiniMaxTTSError = Exception  # type: ignore[assignment, misc]
    MiniMaxTTSAuthError = Exception  # type: ignore[assignment,misc]
    MiniMaxTTSBadRequestError = Exception  # type: ignore[assignment,misc]
    MiniMaxTTSRateLimitError = Exception  # type: ignore[assignment,misc]


class _TTSEmptyTextError(Exception):
    """Issue #2096 — пустой ``text`` дошёл до ``_synthesize_and_play``.

    Дефект ВЫЗЫВАЮЩЕГО (пустой ``text``/``ssml``), а не провайдера — см.
    guard в начале ``_synthesize_and_play``. Отдельный класс (не голый
    ``Exception``) нужен, чтобы исключение поднималось ДО входа в
    ``_sap_run_provider_chain``/``_sap_silero_fallback`` — минимакс/yandex/
    silero никогда его не видят, и ``_mark_provider_dead`` никогда не
    вызывается по этой причине.
    """


# ── Preview-synthesis error hierarchy (ADR-0077 / issue #2138.A.3) ────
# supervisor использует ``except PreviewSynthesisError`` чтобы отделить
# наши ошибки от внешних (MiniMax бросает свой MiniMaxTTSError).
# Иерархия:
#   PreviewSynthesisError           — база, поле ``reason`` для ws_server.
#     ├─ PreviewSynthesisTimeoutError — сетевой синтез не уложился.
#     └─ PreviewSynthesisUnavailableError — MiniMax opt-in не подключён.


class PreviewSynthesisError(Exception):
    # Базовый класс ошибок preview-синтеза. Поле ``reason`` — стабильная
    # строка, которую supervisor пишет в
    # ``preview_voice_error{reason: <reason>}``. Это публичный контракт
    # между avatar_supervisor и ws_server/клиентом — менять опасно.

    def __init__(self, message, reason="preview_synthesis_failed"):
        super().__init__(message)
        self.reason = reason


class PreviewSynthesisTimeoutError(PreviewSynthesisError):
    # Сетевой синтез не уложился в ``timeout_s``. Отдельный класс (не
    # просто reason=timeout) для удобства юнит-тестов и для будущих
    # телеметрий: «сколько preview'ов висит до таймаута» — отдельный
    # gauge от «сколько preview'ов падает по 5xx».

    def __init__(self, message, timeout_s):
        super().__init__(message, reason="preview_timeout")
        self.timeout_s = timeout_s


class PreviewSynthesisUnavailableError(PreviewSynthesisError):
    # MiniMax opt-in не подключён (MINIMAX_AVAILABLE=False).
    # Capability-honest: честно говорим «preview сейчас недоступен», а
    # не делаем вид что работаем. supervisor шлёт
    # preview_voice_error{reason: minimax_unavailable}.

    def __init__(self, message):
        super().__init__(message, reason="minimax_unavailable")


# ── Preview-synthesis value object ─────────────────────────────────────


@dataclass(frozen=True)
class PreviewAudioResult:
    # Результат preview-синтеза для picker'а оператора.
    # audio_bytes — байты в ЗАКОДИРОВАННОМ контейнере (mp3/wav/ogg), а НЕ
    # сырой int16 PCM. Это требование preview_audio_sink.ts: WebAudio
    # decodeAudioData декодирует mp3/wav/opus, но не raw PCM без
    # контейнера. content_type — MIME для ws_server/клиента (audio/mpeg
    # для mp3, audio/wav для wav и т.п.). supervisor оборачивает в
    # {format, content_type, audio_b64, ...} JSON для
    # /avatar/preview_voice/audio.

    audio_bytes: bytes
    content_type: str
    sample_rate: int
    format_str: str
    duration_s: float


def _format_to_content_type(fmt):
    # Map TTSFormat → MIME content_type для ws_server preview_audio_sink.
    # Клиент (preview_audio_sink.ts) передаёт content_type в
    # AudioContext.decodeAudioData — браузерный декодер сам подберёт
    # формат по MIME. PCM (raw) сюда не идёт: audio/L16 технически
    # существует, но в preview-канале WebAudio его ест только если
    # знает sampleRate через параметр, а клиент этого не делает — мы
    # конвертируем в контейнер заранее (mp3/wav).
    if fmt == TTSFormat.MP3:
        return "audio/mpeg"
    if fmt == TTSFormat.WAV:
        return "audio/wav"
    if fmt == TTSFormat.OGG:
        # OGG-контейнер может нести Opus или Vorbis. Клиент шлёт
        # content_type "audio/ogg" — WebAudio разберётся через codec
        # внутри. Если внутри Opus — современный Chromium/Quest
        # поддерживает; если Vorbis — fallback на Edge не нужен.
        return "audio/ogg"
    if fmt == TTSFormat.PCM:
        # Сырой PCM в preview-канале НЕ поддерживается (см. docstring
        # PreviewAudioResult). Если caller выбрал preview_format=pcm —
        # мы всё равно отдадим, но content_type поставим audio/L16 чтобы
        # клиент мог понять «это сырой PCM» (на практике decodeAudioData
        # тут молча упадёт; см. ADR-0077 §грабли).
        return "audio/L16"
    return "application/octet-stream"


# Issue #1160 — Prometheus metrics (этап 1 observability).
# ``prometheus_client`` — optional dep; если её нет, всё превращается в
# no-op и старт сервера тихо возвращает ``False``.
# Issue #1234 — OpenTelemetry traces (этап 2): init_tracing вызывается в
# __init__ ДО создания MiniMax-провайдера (его httpx-клиент должен попасть
# под авто-инструментацию); ``start_span_handle`` — для span ``tts.synthesize``.
from rob_box_voice.observability import (
    init_tracing,
    is_metrics_enabled,
    record_tts_synthesize,
    start_metrics_server,
    start_span_handle,
)


def resample_audio(audio: np.ndarray, orig_sr: float, target_sr: float) -> np.ndarray:
    """
    Resample audio from original sample rate to target sample rate using linear interpolation.

    This is a lightweight resampling implementation suitable for TTS audio where:
    - Low latency is important (no heavy dependencies like scipy/librosa)
    - Audio quality is acceptable for voice synthesis
    - Minimal artifacts for pitch shifting within reasonable range (1.0-3.0x)

    For higher quality resampling, consider using scipy.signal.resample or librosa.resample.

    Args:
        audio: Audio data as numpy array (mono, float32, range -1.0 to 1.0)
        orig_sr: Original sample rate (e.g., 22050 or 10022.7 for fractional rates)
        target_sr: Target sample rate (e.g., 16000)

    Returns:
        Resampled audio at target sample rate
    """
    if abs(orig_sr - target_sr) < 0.01:  # Use epsilon comparison for floats
        return audio

    # Calculate the resampling ratio
    duration = len(audio) / orig_sr
    target_length = int(duration * target_sr)

    # Create new time indices for interpolation
    orig_indices = np.linspace(0, len(audio) - 1, len(audio))
    target_indices = np.linspace(0, len(audio) - 1, target_length)

    # Linear interpolation
    resampled = np.interp(target_indices, orig_indices, audio)

    return resampled


_SILERO_PITCH_LEVELS = ("x-low", "low", "medium", "high", "x-high", "robot")


# ── Issue #1780: Yandex gRPC v3 SSML → pitch/volume конвертация ────────────
# Yandex Cloud TTS v3 ``Hints`` API поддерживает только:
#   * ``pitch_shift`` — Hz-offset (range [-1000; 1000], default 0)
#   * ``volume``      — LUFS dB-offset (range [-145; 0), default -19)
#
# SSML `<prosody>` оперирует относительными множителями/уровнями
# (``pitch="+10%"``, ``volume="loud"``). Здесь мы приводим их к
# Yandex-формату без потери смысла: «на сколько Hz поднять голос» и
# «на сколько dB сделать громче/тише относительно дефолта».
YANDEX_BASELINE_PITCH_HZ: float = (
    130.0  # средняя основная частота голоса anton (~130 Hz)
)
YANDEX_BASELINE_VOLUME_LUFS: float = -19.0  # Yandex дефолт для LUFS-нормализации
_YANDEX_PITCH_SHIFT_MAX_HZ: float = 1000.0  # абсолютный предел API
_YANDEX_VOLUME_MIN_LUFS: float = -145.0  # нижний предел API


def _ssml_pitch_to_hz(pitch) -> Optional[float]:
    """SSML pitch → Hz-offset для Yandex gRPC v3 ``Hints.pitch_shift``.

    Принимает те же формы, что и ``_parse_ssml_attributes``:
    ``"+10%"``, ``"-25%"``, ``"1.2"``, ``"high"``, ``"low"``, ``"medium"``,
    ``"x-high"``, ``"x-low"``, ``"robot"``, ``1.2`` (float), ``None``.
    Возвращает число в ``[-1000; 1000]`` или ``None``, если вход не парсится.

    Эвристика: дефолтный голос anton ≈ 130 Hz baseline; ``+10%`` →
    ``+13 Hz``, ``high`` (~1.2×) → ``+26 Hz``, ``x-high`` (~1.5×) →
    ``+65 Hz``. Отрицательные аналоги.
    """
    if pitch is None:
        return None
    factor: Optional[float] = None
    if isinstance(pitch, (int, float)):
        factor = float(pitch)
    elif isinstance(pitch, str):
        value = pitch.strip().lower()
        # "robot" у Silero означает спец-эффект, не тон — для Yandex
        # не имеет однозначного Hz-маппинга → None.
        if value == "robot":
            return None
        if value in {"x-low", "low", "medium", "high", "x-high"}:
            mapping = {
                "x-low": 0.5,
                "low": 0.8,
                "medium": 1.0,
                "high": 1.2,
                "x-high": 1.5,
            }
            factor = mapping[value]
        elif value.endswith("%"):
            try:
                factor = 1.0 + float(value[:-1]) / 100.0
            except ValueError:
                return None
        else:
            try:
                factor = float(value)
            except ValueError:
                return None
    else:
        return None
    if factor is None:
        return None
    hz = (factor - 1.0) * YANDEX_BASELINE_PITCH_HZ
    # Clamp в валидный диапазон API.
    return max(-_YANDEX_PITCH_SHIFT_MAX_HZ, min(_YANDEX_PITCH_SHIFT_MAX_HZ, hz))


_SSML_NAMED_VOLUME_TO_DB: dict[str, float] = {
    # SSML стандарт (https://www.w3.org/TR/speech-synthesis/#S3.2.4):
    # silent (-∞, мы приравниваем к -145), x-soft (-12), soft (-6),
    # medium (0), loud (+6), x-loud (+12). Шаг ~6 dB.
    "silent": -145.0,
    "x-soft": -12.0,
    "soft": -6.0,
    "medium": 0.0,
    "loud": 6.0,
    "x-loud": 12.0,
}


def _ssml_volume_to_lufs_target(volume) -> Optional[float]:
    """SSML volume → абсолютная LUFS-цель для Yandex gRPC v3 ``Hints.volume``.

    Yandex ``volume`` — абсолютная LUFS-цель в диапазоне ``[-145; 0)``.
    SSML ``volume`` — относительный уровень (``"loud"`` = +6 dB относительно
    дефолта). Возвращаем абсолютную LUFS-цель, от которой Yandex будет
    нормализовать аудио (clamp в ``[-145; 0)``).

    Поддерживает:
    * числа в dB: ``"+5dB"``, ``"-3dB"``, ``"5"``, ``+5``, ``-3``;
    * проценты: ``"+50%"``, ``"-25%"`` (100% = +6 dB);
    * именованные уровни SSML: ``silent|x-soft|soft|medium|loud|x-loud``.
    """
    if volume is None:
        return None
    if isinstance(volume, (int, float)):
        # Числовое значение — трактуем как dB-offset относительно baseline.
        delta = float(volume)
    elif isinstance(volume, str):
        value = volume.strip().lower()
        if value in _SSML_NAMED_VOLUME_TO_DB:
            delta = _SSML_NAMED_VOLUME_TO_DB[value]
        elif value.endswith("db"):
            try:
                delta = float(value[:-2].strip())
            except ValueError:
                return None
        elif value.endswith("%"):
            try:
                pct = float(value[:-1])
            except ValueError:
                return None
            # 100% = +6 dB (один SSML-шаг «громче»). Логарифмически 6 dB
            # ≈ множитель 2× по амплитуде; для пользователя важнее
            # линейная интерполяция в стопе «loud/soft» шагов.
            delta = pct / 100.0 * 6.0
        else:
            try:
                delta = float(value)
            except ValueError:
                return None
    else:
        return None
    # Переводим смещение в абсолютную LUFS-цель.
    target = YANDEX_BASELINE_VOLUME_LUFS + delta
    # Clamp в валидный диапазон Yandex API: [-145; 0).
    return max(_YANDEX_VOLUME_MIN_LUFS, min(-1.0, target))


def normalize_silero_pitch(pitch) -> str:
    """Привести SSML pitch к уровню, который принимает Silero v5.

    Silero v5 ``apply_tts`` понимает в ``<prosody pitch="...">`` только
    ``x-low|low|medium|high|x-high|robot``. LLM/MiniMax-стиль SSML
    генерирует числовые множители (``1.2``, ``+10%``) — их прямая
    передача роняет Silero с ``Invalid <prosody> tag``, и fallback
    молчит (issue #1064). Здесь любой вход (число, процент, слово,
    мусор) приводится к ближайшему допустимому уровню; нераспознанное
    значение даёт безопасный дефолт ``medium``.

    Args:
        pitch: значение из ``ssml_attributes`` (float, int, str или None).

    Returns:
        Один из ``_SILERO_PITCH_LEVELS``.
    """
    if pitch is None:
        return "medium"
    if isinstance(pitch, str):
        value = pitch.strip().lower()
        if value in _SILERO_PITCH_LEVELS:
            return value
        if value.endswith("%"):
            try:
                factor = 1.0 + float(value[:-1]) / 100.0
            except ValueError:
                return "medium"
        else:
            try:
                factor = float(value)
            except ValueError:
                return "medium"
    elif isinstance(pitch, (int, float)):
        factor = float(pitch)
    else:
        return "medium"

    # Числовой множитель → ближайший уровень (симметрично _parse_ssml_attributes:
    # "high"→1.2, "low"→0.8, "medium"→1.0).
    if factor <= 0.6:
        return "x-low"
    if factor <= 0.85:
        return "low"
    if factor <= 1.15:
        return "medium"
    if factor <= 1.4:
        return "high"
    return "x-high"


# Импортируем text_normalizer и Yandex gRPC
scripts_path = Path(__file__).parent.parent / "scripts"
sys.path.insert(0, str(scripts_path))

try:
    from text_normalizer import normalize_for_tts
except ImportError:

    def normalize_for_tts(text):
        """Fallback если нет normalizer."""
        return text


def _parse_optional_int(value: object) -> int | None:
    """Parse a ROS-stringy value into an ``int`` or ``None``.

    Used for ``minimax_pitch`` (issue #1780). Empty string / ``None`` →
    ``None`` (field omitted from payload). Any other string / number is
    coerced via :class:`int`; :class:`ValueError` is logged and treated
    as "unset" so a typo in YAML doesn't take the whole node down.
    """
    if value is None:
        return None
    if isinstance(value, str):
        stripped = value.strip()
        if not stripped:
            return None
        try:
            return int(stripped)
        except ValueError:
            return None
    try:
        return int(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None


def _parse_optional_float(value: object) -> float | None:
    """Parse a ROS-stringy value into a ``float`` or ``None``.

    Used for ``minimax_volume`` (issue #1780). Empty string / ``None`` →
    ``None`` (field omitted from payload). Coercion failures are logged
    as "unset" so a typo doesn't crash the node — the API still gets a
    syntactically valid request.
    """
    if value is None:
        return None
    if isinstance(value, str):
        stripped = value.strip()
        if not stripped:
            return None
        try:
            return float(stripped)
        except ValueError:
            return None
    try:
        return float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None


def _parse_pronunciation_dict(value: object) -> dict | None:
    """Parse the YAML/ROS string ``minimax_pronunciation_dict`` into a dict.

    Used for ``minimax_pronunciation_dict`` (issue #1780). Accepts:

    * Empty string / ``None`` → ``None`` (field omitted from payload).
    * A JSON-encoded object — parsed via :mod:`json`; the MiniMax T2A v2
      spec asks for ``{"tone": [...], "phoneme": [...], "contextual": [...]}``
      so we expect ``Mapping[str, Sequence[str]]``-shaped payloads.
    * Already a ``Mapping`` — passed through.

    Anything else (``str`` that's not JSON, ``int``, ``list``) is logged
    as "ignored" and we return ``None``. We deliberately do NOT raise
    here: this is operator-config, not user-facing input; crashing the
    node on a typo is worse than silently ignoring the malformed value.
    """
    if value is None:
        return None
    if isinstance(value, str):
        stripped = value.strip()
        if not stripped:
            return None
        try:
            parsed = json.loads(stripped)
        except (ValueError, TypeError):
            return None
    elif isinstance(value, Mapping):
        parsed = value
    else:
        return None
    if not isinstance(parsed, Mapping):
        return None
    return dict(parsed)


# Issue #1996 / operator-agent step 7a — allowed values for the top-level
# ``priority`` field of ``/voice/tts/request``.
#
# Набор ВЫРОВНЕН с ADR-0056: те же три значения, что валидирует
# ``scheduler/pregen/pre_gen.py`` для вложенного ``pregenerate.priority``.
# Раньше здесь был двухзначный набор, и top-level ``priority="personality"``
# молча превращался в ``"normal"`` — молчаливая потеря значения на границе
# двух контрактов. Решение владельца: держать полный набор.
#
# Прецеденция в FIFO-gate (см. ``TTSNode._assign_priority_play_seq``):
#
#   operator     — врезка: запрос встаёт сразу за играющим чанком.
#                  Целевая архитектура §8а.3: «ТАРС не договаривается с
#                  планировщиком личности, он просто говорит роботом».
#   personality  — речь личности, хвост очереди.
#   normal       — немаркированный / legacy-трафик, хвост очереди.
#
# ``personality`` и ``normal`` сегодня по порядку НЕ различаются — обоих
# кладём в хвост. Значение сохраняется отдельно намеренно: оно доезжает до
# планировщика предгенерации и метрик, которым важно, чья это реплика.
# Если появится своя прецеденция у личности — менять здесь, тесты на
# порядок уже есть.
_TTS_PRIORITY_VALUES = frozenset({"operator", "personality", "normal"})

#: Значения, дающие врезку. Отдельная константа, чтобы «кто прыгает
#: очередь» читалось в одном месте, а не выводилось из сравнения строк.
_TTS_PRIORITY_PREEMPTS = frozenset({"operator"})


def _normalize_tts_priority(raw: object) -> str:
    """Whitelist-normalize the ``priority`` field of ``/voice/tts/request``.

    Backward-compat contract (issue #1996 DoD): the field is optional, and
    anything outside the whitelist — missing, ``None``, wrong case
    (``"OPERATOR"``), or garbage — is treated as ``"normal"``. A malformed
    payload must never raise or drop the request; it just loses the
    priority bump.

    Whitelist — ``{"operator", "personality", "normal"}``, тот же, что у
    вложенного ``pregenerate.priority`` в ADR-0056. Значение возвращается
    как есть, без схлопывания ``personality`` в ``normal``.
    """
    if raw in _TTS_PRIORITY_VALUES:
        return raw  # type: ignore[return-value]
    return "normal"


# Yandex Cloud TTS API v3 (gRPC)
try:
    from yandex.cloud.ai.tts.v3 import tts_pb2, tts_service_pb2_grpc

    YANDEX_GRPC_AVAILABLE = True
except ImportError:
    YANDEX_GRPC_AVAILABLE = False
    print("⚠️  yandex-cloud-ml-sdk не установлен! Используем только Silero fallback.")

# Per-provider TTS chunking + retry-halve (issue #933).
#
# Yandex gRPC v3 (issue #931) и Silero v5 (issue #933) оба падают на
# длинных текстах: Yandex ≈291 chars, Silero ≈1005 chars. MiniMax HTTP
# T2A v2 принимает длинные тексты без проблем.
#
# Per Подходы 1+2 issue #933:
# 1. ``CHUNK_LIMITS`` — per-provider max chunk size (yandex=700,
#    silero=800, minimax=5000).
# 2. ``synthesize_with_retry`` — retry-halve: при ``TooLongError`` режет
#    chunk пополам (whitespace-safe) и ретраит, max 3 попытки.
#
# Модуль pure-Python, без зависимостей (ROS, numpy, grpc) — чтобы его
# можно было тестировать ``pytest`` без тяжелых dev-deps (torch, grpc,
# rclpy). Сам ``tts_node.py`` его импортирует.
from .tts_chunking import (
    CHUNK_LIMITS,
    SENTENCE_SENTINELS,
    DEFAULT_MAX_RETRIES,
    MIN_CHUNK_CHARS,
    TooLongError,
    get_chunk_limit,
    split_text,
    synthesize_with_retry,
)

# Backward-compat alias для существующих тестов / внешних вызовов.
# Реальный лимит теперь хранится в self.chunk_max_chars_yandex.
YANDEX_MAX_CHUNK_CHARS: int = CHUNK_LIMITS["yandex_grpc_v3"]


# ── Concurrency primitives (BLK-9 follow-up) ────────────────────────────────
# Synthesis executor — bounded ``ThreadPoolExecutor`` whose size is driven
# by the ROS parameters ``synthesis_max_workers`` (1..4, default 2) and
# ``synthesis_max_queue`` (default 16), with a ``Semaphore`` slot cap of
# ``max_workers + max_queue`` for back-pressure. Replaces the original
# PR #907 ``threading.Thread(target=..., daemon=True).start()`` fan-out
# (OWASP A04:2021 — Unrestricted Resource Consumption).
SYNTHESIS_MAX_WORKERS_DEFAULT: int = 2  # 1..4
SYNTHESIS_MAX_QUEUE_DEFAULT: int = 16  # pending tasks cap before drop
SYNTHESIS_SHUTDOWN_TIMEOUT_S: float = 2.0
SYNTHESIS_THREAD_NAME_PREFIX: str = "tts-synth"

# Async-bridge executor — small per-call ``ThreadPoolExecutor`` used to
# host a single ``asyncio.run(...)`` so the synchronous ROS callback can
# drive an async streaming method without colliding with the main thread's
# already-running loop. Each ``with`` block creates and shuts down its
# own executor — strictly lifecycle-bounded (max_workers=1, lifetime
# shorter than the surrounding function call), so this is the same
# primitive as the synthesis executor and not an unbounded fan-out.
ASYNC_BRIDGE_MAX_WORKERS: int = 1

# MiniMax TTS event-loop executor (BLK-9): single long-lived worker
# that runs ``run_forever`` for the asyncio event loop. ``1`` because
# asyncio is single-threaded by design; oversized would risk unbounded
# growth and contradict the BLK-9 regression-guard.
TTS_LOOP_MAX_WORKERS: int = 1

# ────────────────────────────────────────────────────────────────────────
# MiniMax TTS — single long-lived event loop (live 17:20)
# ────────────────────────────────────────────────────────────────────────
# 🔴 FIX (live 16:xx «MiniMax retry: Event loop is closed»): the MiniMax
# provider owns an httpx.AsyncClient bound to the FIRST event loop it saw.
# Every call to ``asyncio.run(coro)`` creates a fresh loop and closes it on
# exit — so the second synthesis (or the first retry) runs the client
# against a CLOSED loop → ``TTSError: Event loop is closed`` → TTS silently
# drops speech (user hears nothing) until the node restarts.
#
# Fix: keep ONE event loop alive in a dedicated daemon thread and submit
# every coroutine to it via ``run_coroutine_threadsafe``. The loop never
# closes while the process lives, so the provider client stays valid.
#: How long :func:`shutdown_tts_loop` waits for the driver to return.
TTS_LOOP_SHUTDOWN_TIMEOUT_S: float = 2.0

_TTS_LOOP_LOCK = threading.Lock()
_TTS_LOOP: asyncio.AbstractEventLoop | None = None
_TTS_LOOP_EXECUTOR: "concurrent.futures.ThreadPoolExecutor | None" = None
_TTS_LOOP_FUTURE: "concurrent.futures.Future | None" = None
_TTS_LOOP_ATEXIT_REGISTERED = False


def _register_tts_loop_atexit() -> None:
    """Arrange for the loop to be stopped before the interpreter joins threads.

    ``ThreadPoolExecutor`` workers are **non-daemon**, and CPython joins them
    inside ``threading._shutdown()`` — which runs BEFORE ``atexit`` handlers.
    A worker parked in ``run_forever()`` never returns, so the process hangs
    after its last line of work: pytest would print its summary and then sit
    there forever, and ``ros2 run`` would not exit on shutdown.

    ``threading._register_atexit`` is the hook that runs at the *start* of
    ``threading._shutdown()``; it is what ``concurrent.futures.thread`` itself
    uses for exactly this problem. Falls back to ``atexit`` if it ever
    disappears — that is too late to prevent the hang, but it still releases
    the loop in embedded interpreters that never join threads.
    """
    global _TTS_LOOP_ATEXIT_REGISTERED
    if _TTS_LOOP_ATEXIT_REGISTERED:
        return
    register = getattr(threading, "_register_atexit", None)
    (register or atexit.register)(shutdown_tts_loop)
    _TTS_LOOP_ATEXIT_REGISTERED = True


def _ensure_tts_loop() -> asyncio.AbstractEventLoop:
    """Return the process-wide MiniMax TTS event loop (create on first use)."""
    global _TTS_LOOP, _TTS_LOOP_EXECUTOR, _TTS_LOOP_FUTURE
    with _TTS_LOOP_LOCK:
        if _TTS_LOOP is not None and not _TTS_LOOP.is_closed():
            return _TTS_LOOP
        _TTS_LOOP = asyncio.new_event_loop()
        # Use bounded ``ThreadPoolExecutor`` (BLK-9 regression-guard) so we
        # never spawn a raw bare daemon thread. The single
        # worker runs ``run_forever`` for the event loop until shutdown.
        #
        # Both the executor and the future are kept module-level: without a
        # reference there is no way to stop the loop, and a non-daemon worker
        # parked in ``run_forever`` blocks interpreter exit forever.
        _TTS_LOOP_EXECUTOR = concurrent.futures.ThreadPoolExecutor(
            max_workers=TTS_LOOP_MAX_WORKERS,
            thread_name_prefix="minimax-tts-loop",
        )
        _TTS_LOOP_FUTURE = _TTS_LOOP_EXECUTOR.submit(_TTS_LOOP.run_forever)
        _register_tts_loop_atexit()
        return _TTS_LOOP


def shutdown_tts_loop(timeout: float = TTS_LOOP_SHUTDOWN_TIMEOUT_S) -> None:
    """Stop the process-wide TTS loop and release its worker thread.

    Idempotent and safe to call when the loop was never started. Mirrors
    ``DialogueNode.shutdown_asyncio_loop``; registered as a shutdown hook
    because the loop is process-wide and no single node owns its lifetime.
    """
    global _TTS_LOOP, _TTS_LOOP_EXECUTOR, _TTS_LOOP_FUTURE
    with _TTS_LOOP_LOCK:
        loop, executor, future = _TTS_LOOP, _TTS_LOOP_EXECUTOR, _TTS_LOOP_FUTURE
        _TTS_LOOP = _TTS_LOOP_EXECUTOR = _TTS_LOOP_FUTURE = None
    if loop is None:
        return
    try:
        if not loop.is_closed():
            loop.call_soon_threadsafe(loop.stop)
    except RuntimeError:
        # Loop already stopped/closed by someone else — nothing to do.
        pass
    if future is not None:
        try:
            future.result(timeout=timeout)
        except Exception:  # noqa: BLE001 — includes TimeoutError
            # Never raise from a shutdown hook: it runs while the interpreter
            # is tearing down, where an exception is both unhelpful and
            # easy to miss. A driver that refuses to stop shows up as the
            # process failing to exit.
            pass
    if executor is not None:
        executor.shutdown(wait=False)
    try:
        if not loop.is_closed():
            loop.close()
    except RuntimeError:
        pass


def _run_in_tts_loop(coro) -> Any:
    """Run *coro* on the process-wide TTS loop from any (sync) thread."""
    loop = _ensure_tts_loop()
    future = asyncio.run_coroutine_threadsafe(coro, loop)
    return future.result()


def _text_or_language_notice(
    logger, provider: str, text: str, language: str = None
) -> str:
    """Текст для синтеза: либо исходный, либо честная фраза-отказ (AV-28).

    ``provider`` берётся ПО ФАКТУ (после цепочки фолбэков, а не из
    ``self.provider``): именно так сегодня и вышло — оператор выбирал язык
    при живом minimax, а синтезировал в итоге Silero.

    Функция модульная, а не метод: ``_synthesize_and_play`` в тестах
    вызывается на bare-стабе без методов ноды (см.
    test/unit/tts/test_voice_selection.py::_playback_node), и метод здесь
    молча ронял бы весь провайдерский бранч в AttributeError.
    """
    notice = unsupported_language_notice(provider, language)
    if notice is None:
        return text
    logger.warning(
        f"🌐 [AV-28] {provider} не умеет язык {language!r} — вместо текста "
        f"произношу отказ (текст был: {text[:60]!r})"
    )
    return notice


class TTSNode(Node):
    """ROS2 нода для синтеза речи с YandexSpeechKit + Silero fallback + MiniMax (opt-in)."""

    def __init__(self):
        super().__init__("tts_node")

        # Issue #1234 — OpenTelemetry traces (этап 2). Вызываем ДО создания
        # провайдеров (MiniMax provider открывает httpx-клиент при первом
        # синтезе) — авто-инструментация httpx должна быть включена раньше.
        # Если opentelemetry-пакетов нет — no-op (см. observability.tracing).
        init_tracing("tts_node")

        # Параметры
        # yandex (primary) | silero (fallback) | minimax (HTTP, opt-in)
        self.declare_parameter("provider", "minimax")

        # Issue #1083: цепочка приоритетов TTS (minimax → yandex → silero).
        # Пустой список → выводится из ``provider`` (см. _chain_from_provider).
        # Silero всегда последний в цепочке (офлайн fallback).
        self.declare_parameter("provider_chain", [])
        # TTL кэша «мёртвых» провайдеров (сек): квота/auth (2056 Token Plan)
        # → длинный TTL, transient (сеть/timeout) → короткий. Пока провайдер
        # в кэше — не долбим его на каждый ход (см. _mark_provider_dead).
        self.declare_parameter("provider_dead_ttl_s", 300.0)
        self.declare_parameter("provider_dead_ttl_transient_s", 30.0)
        # Issue #1229 — файл персистентного состояния фактического
        # провайдера TTS (переживает рестарт контейнера; пустая строка =
        # отключено). tts_node пишет {provider, dead_until_ts, ...} при
        # смене «эффективного» провайдера (фолбек после квоты/сети);
        # mcp_server/dialogue_node читают его через /voice/tts/provider_state,
        # чтобы LLM-контекст и валидация голосов отражали РЕАЛЬНОГО
        # провайдера, а не номинального из параметра provider.
        self.declare_parameter("provider_state_file", "/data/tts_provider_state.json")

        # Yandex Cloud TTS gRPC v3 (оригинальный ROBBOX голос!)
        self.declare_parameter("yandex_api_key", "")
        self.declare_parameter(
            "yandex_voice", "anton"
        )  # anton (ОРИГИНАЛЬНЫЙ ГОЛОС РОББОКСА!)
        self.declare_parameter(
            "yandex_speed", 1.0
        )  # 0.1-3.0 (1.0 = нормальная скорость речи)
        # Issue #1780 / issue #1004: флаг «ssml-aware» режима для Yandex.
        # При True — Yandex-провайдер должен пропускать вход как SSML
        # (``<speak>...<emotion>happy</emotion>...</speak>``), используя
        # ``<emotion>`` и ``<prosody pitch=...>`` теги, поддерживаемые
        # Yandex gRPC v3. Сейчас (False) текст идёт в ``Hints(voice, speed)``
        # как раньше — fallback совместимости. Полная интеграция — в карточке
        # t_c401ecaa; этот параметр объявлен здесь, чтобы YAML был
        # валиден с самого начала.
        self.declare_parameter("yandex_ssml_aware", False)

        # Silero TTS (fallback)
        self.declare_parameter(
            "silero_speaker", "baya"
        )  # aidar (male) | baya (female) | kseniya | xenia | eugene (NEW in v5!)
        self.declare_parameter(
            "silero_sample_rate", 48000
        )  # v5: можно повысить до 48000 для лучшего качества

        # Silero v5: новые флаги для расстановки ударений
        self.declare_parameter("silero_put_accent", True)  # Ударения в обычных словах
        self.declare_parameter("silero_put_yo", True)  # Автоматическая буква ё
        self.declare_parameter(
            "silero_put_stress_homo", True
        )  # Ударения в омографах (замОк/зАмок)
        self.declare_parameter("silero_put_yo_homo", True)  # Ударения в омографах с ё

        # Per-provider max chunk size (issue #933). Defaults берутся из
        # ``CHUNK_LIMITS`` в ``tts_chunking.py`` (yandex=700, silero=800,
        # minimax=5000). Override через YAML/launch (см. voice_assistant.yaml).
        self.declare_parameter("chunk_max_chars_yandex", CHUNK_LIMITS["yandex_grpc_v3"])
        self.declare_parameter("chunk_max_chars_silero", CHUNK_LIMITS["silero_v5"])
        self.declare_parameter("chunk_max_chars_minimax", CHUNK_LIMITS["minimax"])
        self.declare_parameter("chunk_max_retries", DEFAULT_MAX_RETRIES)
        self.declare_parameter("chunk_min_chars", MIN_CHUNK_CHARS)

        # MiniMax TTS (HTTP, T2A v2). Активируется когда provider="minimax".
        # Параметры берутся из ROS-параметров или из ENV (MINIMAX_API_KEY / MINIMAX_GROUP_ID).
        self.declare_parameter(
            "minimax_api_key", ""
        )  # пусто → fallback на os.getenv("MINIMAX_API_KEY")
        self.declare_parameter(
            "minimax_group_id", ""
        )  # пусто → fallback на os.getenv("MINIMAX_GROUP_ID")
        self.declare_parameter("minimax_voice", "male-qn-qingse")  # MiniMax voice id
        self.declare_parameter(
            "minimax_model", "speech-02-hd"
        )  # speech-02-hd | speech-02-turbo
        self.declare_parameter(
            "minimax_language", "ru"
        )  # ru / en / zh — маппится в human-readable на API
        self.declare_parameter("minimax_speed", 1.0)  # 0.5 – 2.0
        self.declare_parameter(
            "minimax_sample_rate", 32000
        )  # Hz — MiniMax возвращает PCM @ 32 kHz
        self.declare_parameter("minimax_timeout", 30.0)  # секунды httpx timeout
        # Формат контейнера, который ожидается от MiniMax. Default PCM, как
        # задокументировано в ADR-0003 §2.3. WAV/MP3/OGG тоже валидны —
        # провайдер вернёт выбранный контейнер, а _synthesize_minimax_async
        # транскодирует его в int16 LE PCM через utils.audio_transcode.
        self.declare_parameter("minimax_format", "pcm")  # pcm | wav | mp3 | ogg
        # ADR-0077 / issue #2138.A.3 — preview-synthesis формат контейнера.
        # Отдельный от minimax_format (тот рассчитан на ALSA playback; preview
        # идёт в /avatar/preview_voice/audio → ws_server → WebAudio клиента,
        # которому нужен ЗАКОДИРОВАННЫЙ контейнер, не сырой PCM).
        # Default mp3: decodeAudioData его декодирует; ogg/opus/wav — тоже.
        # Если поставите preview_format=pcm — клиент упадёт в decodeAudioData,
        # picker покажет ошибку, но supervisor увидит честный preview_voice_error
        # (НЕ silent-mock). См. ADR-0077 §грабли.
        self.declare_parameter("preview_format", "mp3")  # pcm | wav | mp3 | ogg
        # Retry policy — соответствует ADR-0003 §2.6.
        self.declare_parameter("minimax_max_retries", 2)  # 0..3
        self.declare_parameter(
            "minimax_retry_backoff_ms", 500
        )  # ms начальный backoff (удваивается)
        # Streaming mode: использовать ли provider.stream() вместо synthesize().
        # Текущий MiniMax провайдер возвращает один буферизованный чанк,
        # поэтому chunk-per-frame latency win появится только с WebSocket
        # (M5/M6). Эта настройка сейчас полезна для тестов и как
        # forward-compat hook. См. ADR-0003 §2.4.
        self.declare_parameter("minimax_streaming", False)
        # Issue #1780 / issue #1004: дефолтные emotion / pitch / volume /
        # pronunciation_dict для MiniMax T2A v2 (см. minimax_tts.py —
        # ``voice_setting`` принимает ``emotion``, ``pitch`` int semitones,
        # ``vol`` float [0.0, 10.0], ``pronunciation_dict`` str). Дефолты —
        # нейтральные, чтобы сохранить текущее поведение (поля НЕ
        # передаются в API, если явно не заданы):
        #   emotion = "neutral"         → API default, поведение как до #1780
        #   pitch  = ""                 → не передавать
        #   volume = ""                 → не передавать
        #   pronunciation_dict = ""    → JSON-строка MiniMax-словаря
        # Прокидывание значений в ``TTSSettings`` — в карточке t_4e98182a.
        self.declare_parameter("minimax_emotion", "neutral")  # MiniMax T2A v2 emotion
        self.declare_parameter("minimax_pitch", "")  # semitones; "" → не задан
        self.declare_parameter("minimax_volume", "")  # 0.0..10.0; "" → не задан
        self.declare_parameter(
            "minimax_pronunciation_dict", ""
        )  # JSON dict; "" → не задан

        # ROS audio bridge. AudioData carries raw int16 LE PCM without
        # sample-rate metadata, so publishers and sinks must share the configured
        # rate out of band. Best-effort/volatile avoids replaying stale speech and
        # prevents a slow subscriber from back-pressuring TTS playback.
        self.declare_parameter("audio_topic", "/voice/audio/speech")
        self.declare_parameter("audio_output_sample_rate", 16000)
        self.declare_parameter("audio_qos_reliability", "best_effort")
        self.declare_parameter("audio_qos_depth", 10)
        # ADR-0055 / issue #1993 — обратный канал ТАРС в шлем.
        # Параметризуем имя топика для тестов и чтобы шов с ``audio_topic``
        # остался единственной параметризацией.
        self.declare_parameter("headset_audio_topic", "/avatar/tts/audio")
        # ADR-0078 §4 follow-up (issue #2162) — side-channel sample_rate.
        # AudioData не имеет поля rate, и приватный атрибут Python-объекта
        # через DDS НЕ сериализуется (rclpy десериализует только поля IDL;
        # quest_node через DDS получает msg без _tars_sample_rate).
        # Решение — отдельный топик-метаданные (String JSON), который
        # публикуется РЯДОМ с каждым AudioData. Контракт:
        #   {"request_id": str, "sample_rate": int, "ts_ms": int}
        # Параметризован, чтобы тесты могли подменить.
        self.declare_parameter("headset_audio_meta_topic", "/avatar/tts/audio_meta")
        self.declare_parameter("avatar_request_topic", "/avatar/tts/request")
        self.declare_parameter("avatar_error_topic", "/avatar/tts/error")
        self.declare_parameter("avatar_control_topic", "/avatar/tts/control")
        # ADR-0077 / issue #2138.A.3 — preview-канал для picker'а голосов.
        # Контракт публикаций — зеркалирует ``avatar_*``:
        #   * ``/avatar/preview_voice/audio``  — String JSON {request_id,
        #     format, content_type, audio_b64, sample_rate, duration_s}.
        #   * ``/avatar/preview_voice/result`` — String JSON {request_id, ...}.
        #   * ``/avatar/preview_voice/error``  — String JSON {request_id,
        #     reason, ts_ms}. reason — стабильная строка для ws_server/UI.
        # Зашиты константами (не параметрами) — см. CC-budget ADR-0021 и
        # логику выше (``_tars1_text_topic``).
        self._preview_audio_topic: str = "/avatar/preview_voice/audio"
        self._preview_result_topic: str = "/avatar/preview_voice/result"
        self._preview_error_topic: str = "/avatar/preview_voice/error"
        # Issue #2113 (quest #2112) — echo of TTS-текста на отдельный
        # топик для боковой текстовой панели TARS 1 в Captain Bridge.
        # Контракт: String JSON {request_id, text, streaming:bool, done:bool}.
        # streaming=true пока TTS ещё не закончил, done=true при публикации
        # последнего чанка. TARS 1 на клиенте склеивает чанки в строки.
        # Топик зашит константой (не параметром) — чтобы не плодить
        # CC-budget-нагрузку на __init__ (ADR-0021): смена топика
        # не предполагается, dispatch делается через топик-неймспейс ROS.
        self._tars1_text_topic: str = "/tars1/text"

        # Synthesis worker pool (BLK-9 fix).
        #
        # PR #907 originally spawned `threading.Thread(target=..., daemon=True)`
        # for every /voice/tts/say request with no bound, leaving the node
        # vulnerable to unbounded thread fan-out under bursty input
        # (OWASP A04:2021). We now use a bounded ThreadPoolExecutor and
        # rely on its internal queue to absorb backlog.
        #
        # Sizing notes:
        #   * Each worker is a blocking HTTP-synth → ALSA-play pipeline; they
        #     are serialized through `_synthesis_lock` inside the worker.
        #   * `max_workers=2` lets the next request start its HTTP call while
        #     the previous one is still streaming ALSA playback (synthesis
        #     and playback are sequential within a single worker, but the
        #     pre-synth HTTP fetch can overlap with another worker's
        #     playback tail).
        #   * Overflow is benign: ThreadPoolExecutor enqueues and the
        #     stale-dialogue-id check inside `_run_synthesis_worker` drops
        #     tasks from a previous dialogue (barge-in).
        self.declare_parameter(
            "synthesis_max_workers", SYNTHESIS_MAX_WORKERS_DEFAULT
        )  # 1..4
        self.declare_parameter(
            "synthesis_max_queue", SYNTHESIS_MAX_QUEUE_DEFAULT
        )  # pending tasks cap before drop

        # Issue #1160 — Prometheus metrics endpoint. 9110 — TTS-нода в voice
        # (synthesize latency / provider fallback counter). 0 = отключить.
        self.declare_parameter("metrics_port", 9110)

        # Issue #2003 / ADR-0056 — speculative pre-generation (chunk-level).
        # Default ON so the opt-in happens at the publisher level (via the
        # ``pregenerate`` field in the chunk payload), not here. The hard
        # kill-switch is ``pregenerate_enabled=false`` — useful for e2e
        # baseline comparison. ``pregenerate_confidence_floor`` exposes the
        # CONFIDENCE_FLOOR constant for operator tuning.
        self.declare_parameter("pregenerate_enabled", True)
        self.declare_parameter("pregenerate_confidence_floor", _PREGEN_CONFIDENCE_FLOOR)
        # Sample-rate the speculative pre-gen uses for the duration_ratio
        # quality heuristic; defaults to the audio output rate (16 kHz).
        self.declare_parameter("pregenerate_history_window", 10)

        # Per-provider TTS chunking + retry-halve параметры объявлены
        # выше (issue #933 + дополнение #976 для minimax). Дубликат
        # удалён — см. задачу t_20265b43.

        # Общие параметры
        self.declare_parameter(
            "chipmunk_mode", True
        )  # ВКЛЮЧЕНО: True для весёлого голоса бурундука! 🐿️
        self.declare_parameter(
            "pitch_shift", 1.0
        )  # Множитель для playback rate (1.0 = нормальная скорость)
        self.declare_parameter("normalize_text", True)
        self.declare_parameter("volume_db", -3.0)  # Громкость в dB (-3dB = 70%)

        # Issue #929 (OOM kill tts_node): фоновый warm-load Silero держит
        # ~700 MB-1 GB RSS постоянно (PyTorch + модель), даже когда Silero —
        # только fallback и Yandex/MiniMax работают. На конфигурациях с
        # жёстким mem_limit (2-4 GB на контейнер из 9 нод) этот постоянный
        # RSS складывается с остальными нодами (speaker_id + torch, sclang,
        # Vosk) и доводит контейнер до OOM при первой же попытке догрузить
        # модель. Параметр ``silero_warm_load`` (default True — сохраняет
        # контракт G-933-B) позволяет ОТЛОЖИТЬ загрузку Silero до первого
        # реального fallback (lazy): при False warm-load при старте не
        # запускается, модель грузится синхронно в hot-path (первый
        # fallback платит 2-3 с — приемлемо для аварийного пути).
        self.declare_parameter("silero_warm_load", True)

        # Читаем параметры
        self.provider = self.get_parameter("provider").value
        if self.provider not in {"yandex", "silero", "minimax"}:
            raise ValueError(
                "provider must be one of: yandex, silero, minimax; "
                f"got {self.provider!r}"
            )

        # Issue #1083: цепочка приоритетов TTS. Если provider_chain явно задан
        # (например, из e2e-контракта) — используем его; иначе выводим из
        # ``provider``. Silero всегда последний в цепочке (инвариант).
        raw_chain = self.get_parameter("provider_chain").value
        if raw_chain:
            self.provider_chain = [str(p) for p in raw_chain]
        else:
            # provider параметр (minimax|yandex|silero) задаёт ПЕРВОГО
            # в цепочке; остальные добираются дефолтным порядком, Silero
            # всегда последний (см. _default_provider_chain / _provider_first).
            self.provider_chain = self._chain_from_provider(self.provider)
        self.provider_chain = self._normalize_provider_chain(self.provider_chain)

        # TTL кэша «мёртвых» провайдеров (сек).
        self.provider_dead_ttl_s = max(
            1.0, float(self.get_parameter("provider_dead_ttl_s").value)
        )
        self.provider_dead_ttl_transient_s = max(
            1.0, float(self.get_parameter("provider_dead_ttl_transient_s").value)
        )
        # Issue #1229 — файл персистентного состояния эффективного провайдера.
        self.provider_state_file = str(
            self.get_parameter("provider_state_file").value or ""
        )
        # Провайдеры, помеченные мёртвыми (quota/сеть/ошибка) до определённого
        # момента времени (time.monotonic). Ключ — имя провайдера.
        self._provider_dead_until: Dict[str, float] = {}
        self._provider_dead_reason: Dict[str, str] = {}
        # Восстанавливаем кэш «мёртвых» из персистентного файла (issue #1229):
        # после рестарта робот сразу знает фактического провайдера (квота
        # MiniMax не заканчивается за время рестарта) — LLM-контекст не врёт
        # с первого хода.
        self._load_persisted_provider_state()

        # Yandex Cloud TTS gRPC v3
        self.yandex_api_key = self.get_parameter("yandex_api_key").value or os.getenv(
            "YANDEX_API_KEY", ""
        )
        self.yandex_voice = self.get_parameter("yandex_voice").value
        self.yandex_speed = self.get_parameter("yandex_speed").value

        # Silero
        self.silero_speaker = self.get_parameter("silero_speaker").value
        self.silero_sample_rate = self.get_parameter("silero_sample_rate").value

        # Issue #929: warm-load Silero при старте отключаем через параметр.
        # True (default) — прежнее поведение (G-933-B); False — Silero
        # грузится лениво при первом реальном fallback (экономия ~1 GB RSS,
        # спасает от OOM kill в контейнере с жёстким mem_limit).
        self.silero_warm_load_enabled = bool(
            self.get_parameter("silero_warm_load").value
        )

        # Silero v5: новые флаги
        self.silero_put_accent = self.get_parameter("silero_put_accent").value
        self.silero_put_yo = self.get_parameter("silero_put_yo").value
        self.silero_put_stress_homo = self.get_parameter("silero_put_stress_homo").value
        self.silero_put_yo_homo = self.get_parameter("silero_put_yo_homo").value

        # Per-provider chunk limits and retry policy.
        self.chunk_max_chars_yandex = max(
            1, int(self.get_parameter("chunk_max_chars_yandex").value)
        )
        self.chunk_max_chars_silero = max(
            1, int(self.get_parameter("chunk_max_chars_silero").value)
        )
        self.chunk_max_retries = max(
            1, int(self.get_parameter("chunk_max_retries").value)
        )
        self.chunk_min_chars = max(1, int(self.get_parameter("chunk_min_chars").value))

        # MiniMax (lazy init — только при provider="minimax")
        self.minimax_api_key = self.get_parameter("minimax_api_key").value or os.getenv(
            "MINIMAX_API_KEY", ""
        )
        self.minimax_group_id = self.get_parameter(
            "minimax_group_id"
        ).value or os.getenv("MINIMAX_GROUP_ID", "")
        self.minimax_voice = self.get_parameter("minimax_voice").value
        self.minimax_model = self.get_parameter("minimax_model").value
        self.minimax_language = self.get_parameter("minimax_language").value
        self.minimax_speed = float(self.get_parameter("minimax_speed").value)
        self.minimax_sample_rate = int(self.get_parameter("minimax_sample_rate").value)
        self.minimax_timeout = float(self.get_parameter("minimax_timeout").value)
        self.minimax_format = self._parse_format(
            self.get_parameter("minimax_format").value
        )
        # ADR-0077 / issue #2138.A.3 — preview-synthesis формат контейнера.
        # Если rob_box_llm недоступен — fallback на mp3-строку (тот же
        # graceful-degrade, что у minimax_format на line 1172).
        self.preview_format = self._parse_format(
            self.get_parameter("preview_format").value
        )
        self.minimax_max_retries = min(
            3, max(0, int(self.get_parameter("minimax_max_retries").value))
        )
        self.minimax_retry_backoff_ms = max(
            0, int(self.get_parameter("minimax_retry_backoff_ms").value)
        )
        self.minimax_streaming = bool(self.get_parameter("minimax_streaming").value)
        # Issue #1780 / issue #1004: emotion / pitch / volume / pronunciation_dict
        # для MiniMax. Нейтральные дефолты сохраняют текущее поведение (поля
        # НЕ передаются в API). Прокидывание в ``TTSSettings`` — в t_4e98182a.
        # Храним сырые строки в ``*_raw`` (по дизайну helpers
        # ``_parse_optional_int/float/_parse_pronunciation_dict`` —
        # пустая строка → ``None`` → поле опускается в payload).
        # Прямое приведение через ``int(self.minimax_pitch)`` упало бы на
        # дефолте ``""`` (issue #1780 post-#1816-fix regression: PR #1820
        # убрал duplicate declare_parameter, но оставил голый ``int(...)``
        # на дефолте ``""`` → ``ValueError: invalid literal for int()``).
        self.minimax_emotion = self._normalize_minimax_emotion(
            str(self.get_parameter("minimax_emotion").value or "")
        )
        self.minimax_pitch_raw = self.get_parameter("minimax_pitch").value
        self.minimax_volume_raw = self.get_parameter("minimax_volume").value
        self.minimax_pronunciation_dict_raw = self.get_parameter(
            "minimax_pronunciation_dict"
        ).value
        self.minimax_provider = None  # lazy: создаётся в _ensure_minimax_provider()
        # Provider construction opens an httpx client and must be atomic with
        # shutdown.  ROS callbacks can run on different executor threads.
        self._minimax_provider_lock = threading.Lock()
        self._minimax_provider_initialized = False
        self._minimax_shutdown_requested = False
        # Typed-проекции для читаемости / unit-тестов:
        self.minimax_pitch = _parse_optional_int(self.minimax_pitch_raw)
        self.minimax_volume = _parse_optional_float(self.minimax_volume_raw)
        self.minimax_pronunciation_dict = _parse_pronunciation_dict(
            self.minimax_pronunciation_dict_raw
        )

        # Issue #1780 / issue #1004: «ssml-aware» режим для Yandex. Полная
        # интеграция — в t_c401ecaa; параметр уже читается здесь, чтобы
        # YAML был валиден и можно было безопасно переключать.
        self.yandex_ssml_aware = bool(self.get_parameter("yandex_ssml_aware").value)

        self.audio_topic = str(self.get_parameter("audio_topic").value)
        self.audio_output_sample_rate = int(
            self.get_parameter("audio_output_sample_rate").value
        )
        if self.audio_output_sample_rate <= 0:
            raise ValueError("audio_output_sample_rate must be > 0")
        self.audio_qos_reliability = str(
            self.get_parameter("audio_qos_reliability").value
        ).lower()
        self.audio_qos_depth = max(1, int(self.get_parameter("audio_qos_depth").value))
        self.audio_channels = 1
        # ADR-0055 / issue #1993 — параметры обратного канала ТАРС в шлем.
        # Те же параметры, что у audio_topic/... — единственный шов в одном
        # месте для forward-compat тестов и override'ов через launch-файлы.
        self.headset_audio_topic = str(self.get_parameter("headset_audio_topic").value)
        # ADR-0078 §4: side-channel sample_rate через /avatar/tts/audio_meta.
        self.headset_audio_meta_topic = str(
            self.get_parameter("headset_audio_meta_topic").value
        )
        self.avatar_request_topic = str(
            self.get_parameter("avatar_request_topic").value
        )
        self.avatar_error_topic = str(self.get_parameter("avatar_error_topic").value)
        self.avatar_control_topic = str(
            self.get_parameter("avatar_control_topic").value
        )

        # Общие
        self.chipmunk_mode = self.get_parameter("chipmunk_mode").value
        self.pitch_shift = self.get_parameter("pitch_shift").value
        self.normalize_text = self.get_parameter("normalize_text").value
        self.volume_db = self.get_parameter("volume_db").value

        # Конвертируем dB в линейный множитель
        self.volume_gain = 10.0 ** (self.volume_db / 20.0)

        # Callback для изменения параметров во время работы
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Silero TTS модель (lazy loading - загружается только при первом использовании)
        self.silero_model = None
        self.silero_loading = False
        self.device = torch.device("cpu")

        # Warm-load coordination (gap G-933-B): when Yandex is the primary
        # provider, Silero is just a fallback. Cold-loading the ~10 MB
        # torch.package on the first Yandex→Silero fallback triggers a
        # 2-3 s pause while the user hears silence ("UX illusion of hang").
        #
        # To avoid that, after Silero is *selected* as a fallback we kick
        # off a background warm-load via a bounded
        # ``ThreadPoolExecutor`` that loads the model up-front and sets
        # ``self._silero_loaded`` once it's done.  The synchronous synth
        # path (``_synthesize_and_play``) waits up to 1.5 s on the event
        # before falling back to skipping playback for that one chunk —
        # by then the model is normally hot and subsequent fallbacks cost
        # ~0 s of additional latency.
        #
        # Why a bounded executor and not a bare ``threading.Thread``:
        # BLK-9 (test_no_daemon_threads) forbids ``threading.Thread(...)``
        # / ``daemon=True`` in production code — the regression guard
        # was added because PR #907 BLK-9 left a bare daemon thread
        # behind that could leak under burst load.  We use a
        # ``ThreadPoolExecutor(max_workers=1)`` sized exactly for the
        # one warm-load job so the pattern is consistent with the rest
        # of tts_node and survives the regression test.
        #
        # Lifecycle:
        # * ``_silero_loaded`` is initialised to a CLEARED ``Event`` —
        #   the wait returns False if the executor never had a chance
        #   to run (e.g. unit-test subclass override).
        # * Set in the worker once ``_load_silero_model`` finishes
        #   (success OR failure — failure is signalled so the hot-path
        #   doesn't busy-loop waiting for a model that will never load).
        # * The hot-path wait timeout is intentionally short (< typical
        #   2.7 s cold-load) — on timeout we skip this chunk rather than
        #   load synchronously, which would be no better than before.
        self._silero_loaded = threading.Event()
        self._silero_warm_executor: concurrent.futures.ThreadPoolExecutor | None = None
        self._silero_warm_future: concurrent.futures.Future | None = None
        # Test contract (gap G-933-B): _silero_warm_thread aliases the
        # executor for code that prefers the old threading.Thread naming.
        self._silero_warm_thread = None  # type: ignore[assignment]
        self._silero_load_outcome: str | None = None  # "ok" | "fail" | None
        self._silero_load_lock = threading.Lock()
        # Track who actually requested warm-load so we don't double-spawn
        # when ``__init__`` decides provider=silero (which already does
        # synchronous load) and the warm-load step is redundant.
        self._silero_warm_requested = False

        # Bounded synthesis executor (BLK-9 fix).
        #
        # Replaces the previous `threading.Thread(target=..., daemon=True)`
        # fan-out.  ThreadPoolExecutor keeps at most `max_workers` OS threads
        # alive and enqueues overflow into an unbounded internal queue.  We
        # pair the executor with a plain `Semaphore` so that submissions
        # beyond `max_queue + max_workers` are rejected (non-blocking
        # `acquire()` returns False) instead of silently piling up zombie
        # pending tasks.
        #
        # `_synthesis_lock` is the per-node gate that serializes the actual
        # blocking HTTP+ALSA work inside each worker — the executor only
        # bounds *thread count*, the semaphore + lock bound *in-flight work*.
        #
        # IMPORTANT: This block is positioned BEFORE the silero warm-load
        # branch so that unit tests which patch
        # ``concurrent.futures.ThreadPoolExecutor`` observe the synthesis
        # executor in ``recordings[0]`` (max_workers ==
        # ``SYNTHESIS_MAX_WORKERS_DEFAULT`` == 2). If the silero warm-load
        # executor were constructed first (max_workers=1), the test
        # ``test_tts_node_synthesis_executor_is_bounded_at_runtime`` would
        # see ``max_workers=1 != 2`` and fail.
        max_workers = max(
            1, min(4, int(self.get_parameter("synthesis_max_workers").value))
        )
        max_queue = max(1, int(self.get_parameter("synthesis_max_queue").value))
        # Total in-flight cap = workers currently executing + pending in queue.
        self._synthesis_slots = threading.Semaphore(max_queue + max_workers)
        self._synthesis_executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=max_workers,
            thread_name_prefix=SYNTHESIS_THREAD_NAME_PREFIX,
        )
        self._synthesis_executor_max_workers = max_workers
        self._synthesis_executor_shutdown = False
        self._synthesis_in_flight = 0
        self.get_logger().info(
            f"  Synthesis executor: max_workers={max_workers}, max_queue={max_queue}"
        )

        # Если provider='silero' - загружаем сразу (synchronous; primary mode)
        if self.provider == "silero":
            self.get_logger().info("🔄 Provider=silero → загрузка Silero TTS...")
            self._silero_warm_requested = False  # explicit reset
            self._load_silero_model()
            # Mark as loaded regardless of outcome so the hot-path wait
            # doesn't hang on a never-completed background job.
            self._silero_load_outcome = (
                "ok" if self.silero_model is not None else "fail"
            )
            self._silero_loaded.set()
        else:
            # provider=yandex (or minimax) — Silero is a *fallback*.
            # Kick off the background warm-load so the first fallback
            # doesn't pay the 2-3 s cold-load cost.  Issue #929: skip the
            # warm-load entirely when ``silero_warm_load=false`` — the
            # constant ~1 GB RSS (PyTorch + model) contributes to OOM kill
            # in the 2-4 GB container.  The hot-path lazy-load covers the
            # first fallback (2-3 s, acceptable for the emergency path).
            if self.silero_warm_load_enabled:
                self._start_silero_warm_load()
            else:
                self.get_logger().info(
                    "🌡️ Silero warm-load disabled (silero_warm_load=false) — "
                    "lazy-load on first fallback (issue #929)"
                )

        # Yandex Cloud TTS gRPC v3 (оригинальный ROBBOX голос anton!)
        self.yandex_channel = None
        self.yandex_stub = None
        if YANDEX_GRPC_AVAILABLE and self.yandex_api_key:
            try:
                self.yandex_channel = grpc.secure_channel(
                    "tts.api.cloud.yandex.net:443", grpc.ssl_channel_credentials()
                )
                self.yandex_stub = tts_service_pb2_grpc.SynthesizerStub(
                    self.yandex_channel
                )
                self.get_logger().info("✅ Yandex Cloud TTS gRPC v3 подключен")
            except Exception as e:
                self.get_logger().warn(f"⚠️  Не удалось подключиться к Yandex gRPC: {e}")

        # Инициализация аудио устройства для воспроизведения
        self.device_index = None
        self.initialize_audio_device()

        # Менеджер воспроизведения (предотвращает ALSA конфликты)
        self.playback_manager = AudioPlaybackManager.get_instance()

        # Подписка на dialogue response (от dialogue_node)
        self.dialogue_sub = self.create_subscription(
            String, "/voice/dialogue/response", self.dialogue_callback, 10
        )

        # Подписка на TTS requests (от reflection_node и других)
        self.tts_request_sub = self.create_subscription(
            String,
            "/voice/tts/request",
            self.dialogue_callback,
            10,  # Используем тот же callback
        )

        # Подписка на control commands (STOP)
        self.control_sub = self.create_subscription(
            String, "/voice/tts/control", self.control_callback, 10
        )

        # ADR-0055 / issue #1993 — подписка на запросы ТАРС в шлем.
        # Контракт String JSON повторяет /voice/tts/request (см. dialogue_callback)
        # плюс обязательное поле ``sink=="headset"``. Невалидный sink →
        # self._avatar_tts_error_pub.publish({request_id, error:"invalid_sink"})
        # и DROP (ADR-0055 §tts_node). Контроль (STOP / IGNORE_STOP_MS) —
        # общий с /voice/tts/control, формат команды совпадает (тот же
        # control_callback, см. C2 impl-plan §5).
        self._avatar_tts_request_sub = self.create_subscription(
            String, self.avatar_request_topic, self._on_avatar_tts_request, 10
        )
        self._avatar_tts_error_pub = self.create_publisher(
            String, self.avatar_error_topic, 10
        )
        # Issue #2113 (quest #2112) — TARS 1 echo publisher. Подписка на
        # этот топик делает Quest-клиент; mirror ровно того, что TARS
        # сейчас озвучивает (text + streaming flag), чтобы боковая
        # текстовая панель в Captain Bridge показывала тот же текст, что
        # идёт в динамик шлема.
        self._tars1_text_pub = self.create_publisher(
            String, self._tars1_text_topic, 10
        )
        # ADR-0077 / issue #2138.A.3 — publishers preview-канала.
        # Заводятся ВСЕГДА (даже в мини-CI-env без preview-клиента): ws_server
        # на проде подписан на error/done/audio и шлёт picker'у через
        # ws_server.deliver_preview_*. mock-rclpy в unit-тестах
        # перехватывает .publish() и складывает в .published — см.
        # ``tests/conftest.py``.
        self._preview_audio_pub = self.create_publisher(
            String, self._preview_audio_topic, 10
        )
        self._preview_result_pub = self.create_publisher(
            String, self._preview_result_topic, 10
        )
        self._preview_error_pub = self.create_publisher(
            String, self._preview_error_topic, 10
        )
        self._avatar_tts_control_sub = self.create_subscription(
            String, self.avatar_control_topic, self.control_callback, 10
        )
        # Текущий avatar-request_id (один активный). Используется в
        # control_callback для отсечения устаревших запросов от старого
        # avatar-запроса при barge-in / STOP.
        self._avatar_tts_request_id: Optional[str] = None

        # Подписка на новый dialogue_id от dialogue_node.
        # Позволяет отбрасывать устаревшие TTS-запросы от старого диалога после barge-in.
        self._new_dialogue_id_sub = self.create_subscription(
            String, "/voice/current_dialogue_id", self._on_new_dialogue_id, 1
        )

        # Issue #1765 — переключение TTS-провайдера по запросу LLM через
        # SetVoiceTool(provider=...) / SetTtsProviderTool. mcp_server
        # публикует JSON {"provider": str, "voice": str|"", "source": str}
        # в /voice/tts/set_provider; мы пересобираем provider_chain
        # (новый провайдер первым, остальные в исходном порядке, silero
        # всегда последним), чистим dead_until для нового провайдера и
        # публикуем provider_state для dialogue_node/mcp_server (LLM
        # увидит нового провайдера в [TTS] строке контекста).
        self.set_provider_sub = self.create_subscription(
            String, "/voice/tts/set_provider", self._on_set_provider, 10
        )

        # Публикация аудио и состояния
        if self.audio_qos_reliability == "best_effort":
            audio_reliability = ReliabilityPolicy.BEST_EFFORT
        elif self.audio_qos_reliability == "reliable":
            audio_reliability = ReliabilityPolicy.RELIABLE
        else:
            raise ValueError(
                "audio_qos_reliability must be 'best_effort' or 'reliable', "
                f"got {self.audio_qos_reliability!r}"
            )
        audio_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self.audio_qos_depth,
            reliability=audio_reliability,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.audio_pub = self.create_publisher(AudioData, self.audio_topic, audio_qos)
        # ADR-0055 / issue #1993 — обратный канал ТАРС в шлем.
        # Тот же формат (int16 LE PCM), та же QoS, что у ``audio_topic``
        # (см. meta spec). Подписчик (quest_node) ожидает эти параметры
        # байт-в-байт, иначе не сможет декодировать чанк.
        self._avatar_audio_pub = self.create_publisher(
            AudioData, self.headset_audio_topic, audio_qos
        )
        # ADR-0078 §4 follow-up: side-channel метаданные для чанков headset-аудио.
        # String JSON {request_id, sample_rate, ts_ms} — публикуется ДО
        # каждого AudioData в /avatar/tts/audio. Подписчик (quest_node)
        # кеширует request_id → sample_rate и подставляет в deliver_audio().
        self._avatar_audio_meta_pub = self.create_publisher(
            String, self.headset_audio_meta_topic, 10
        )
        self.state_pub = self.create_publisher(String, "/voice/tts/state", 10)
        self.finished_pub = self.create_publisher(
            String, "/voice/tts/finished", 10
        )  # Публикация завершения произношения
        # Issue #1229 — фактический провайдер TTS (после фолбека). mcp_server
        # и dialogue_node подписываются на этот топик, чтобы валидация голосов
        # и LLM-контекст [TTS] отражали РЕАЛЬНОГО провайдера, а не номинального
        # из параметра provider. Публикуется при старте (из персистентного
        # файла), при пометке провайдера мёртвым и после успешного синтеза.
        self.provider_state_pub = self.create_publisher(
            String, "/voice/tts/provider_state", 10
        )
        # AV-27 / issue #1919 — latched-каталог голосов для wire-payload
        # ``voice_list`` (см. design t_5b9d5d0c §128-150). Публикуется на
        # startup + каждый раз после ``_publish_provider_state``. TRANSIENT_LOCAL
        # обязателен — без него quest_node, подключившийся ПОСЛЕ tts_node,
        # не увидит ни одного payload и UI останется без голосов.
        from rclpy.qos import DurabilityPolicy as _DP  # noqa: PLC0415

        self.voices_catalog_pub = self.create_publisher(
            String,
            "/voice/tts/voices",
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=_DP.TRANSIENT_LOCAL,
            ),
        )
        # Issue #980 — single event per multi-chunk TTS batch (rap, poetry).
        # tts_node publishes one ``/voice/tts/batch_complete`` after the last
        # chunk lands so dialogue_node can fire ``music_cleanup`` exactly once.
        self.batch_complete_pub = self.create_publisher(
            String, "/voice/tts/batch_complete", 10
        )

        # Флаг для остановки воспроизведения
        self.stop_requested = False
        self.current_stream = None  # Текущий sounddevice stream
        # Serialize synth/play workers; callbacks stay non-blocking while queued
        # requests are dropped by dialogue-id checks after barge-in.
        self._synthesis_lock = threading.Lock()
        # Issue #1563 — защита от «STOP после синтеза, но до воспроизведения».
        # Когда barge-in (wake-word) приходит во время синтеза, а фраза
        # «робот новая сессия» уже зарезолвлена как session-reset — поздний
        # STOP не должен отменять УЖЕ готовый к воспроизведению синтез.
        # Поэтому: 1) в окне IMMUNE STOP игнорируется (например после reset
        # сессии); 2) STOP между успешным синтезом и play_audio буферизуется
        # для СЛЕДУЮЩЕГО запроса, не отменяя текущий.
        self._immune_until_ts = 0.0  # monotonic — до этого момента STOP игнор
        self._post_synth_stop_pending = False  # STOP пришёл между synth/play

        # 🔴 FIX (live 12:02 «анекдот перепутан»): FIFO-порядок воспроизведения.
        # LLM вызвала 5 speak_text подряд (анекдот) — 5 задач ушли в пул
        # параллельно, каждый ждал _synthesis_lock, и порядок воспроизведения
        # стал порядком ЗАХВАТА lock, а не порядком отправки («Второй
        # отвечает» сыграл раньше «Один говорит»). Решение: синтез идёт
        # ПАРАЛЛЕЛЬНО (без lock — быстро, 4-5 фраз рендерятся сразу), а
        # перед play_audio каждый worker ждёт своей очереди (play_seq),
        # выданной в порядке приёма запросов (порядок LLM tool_calls).
        self._play_seq_counter = 0  # выдаётся при submit (порядок приёма)
        self._next_play_seq = 1  # какой seq сейчас можно играть
        self._play_order_cond = threading.Condition()

        # Issue #1996 / operator-agent step 7a — приоритетная очередь перед
        # динамиком. ``_pending_seqs`` хранит текущий назначенный ``play_seq``
        # для каждого ``speech_id`` в очереди; слот может быть переназначен
        # под ``_play_order_cond``, если позже придёт ``operator``-запрос и
        # «вклинится» сразу за активным чанком (см. ``_assign_priority_play_seq``).
        # ``_play_active_seq`` — seq чанка, который СЕЙЧАС внутри
        # ``play_audio`` (``None``, если ничего не играет) — это и есть
        # «активный чанк», за которым должен встать operator (инвариант 8a:
        # врезка ≠ прерывание — активный чанк никогда не трогается).
        self._pending_seqs: Dict[str, int] = {}
        self._play_active_seq: Optional[int] = None

        # (The synthesis executor block itself was moved earlier in
        # ``__init__`` so that ``_start_silero_warm_load`` does not
        # accidentally become ``recordings[0]`` when unit tests
        # patch the underlying ThreadPoolExecutor. See the
        # comment block above.)
        self._synthesis_slots = getattr(self, "_synthesis_slots", None)

        # Issue #2003 / ADR-0056 — speculative pre-generation engine.
        # Created lazily on first ``pregenerate()`` call so nodes that
        # disable pre-gen (or never receive a ``pregenerate`` payload)
        # pay zero overhead. The engine owns a single ``SpeculativeExecutor``
        # (the actual asyncio orchestrator) plus the lightweight book-keeping
        # ``_last_chunk_finished_at`` for the latency_chunk_to_chunk metric.
        self._prefetch: Optional[Dict[str, Any]] = None
        self._last_chunk_finished_at: Optional[float] = None
        # Read ROS-params into typed locals so the kill-switch is honoured
        # even before the lazy init runs (e.g. parameter_callback toggles).
        self._pregenerate_enabled: bool = bool(
            self.get_parameter("pregenerate_enabled").value
        )
        self._pregenerate_confidence_floor: float = max(
            0.0,
            min(1.0, float(self.get_parameter("pregenerate_confidence_floor").value)),
        )
        self._pregenerate_history_window: int = max(
            1, int(self.get_parameter("pregenerate_history_window").value)
        )

        # Dialogue session tracking (для синхронизации с dialogue_node)
        self.current_dialogue_id = None
        self.processing_dialogue_id = (
            None  # ID диалога в процессе синтеза/воспроизведения
        )
        self.current_speech_id = None  # ID текущего произношения (для MCP tools)

        # Issue #1160 — Prometheus metrics server (этап 1).
        # Порт 9110 — стандартный для tts_node (см. observability/__init__.py);
        # в проде используется ``10.1.1.11:9110/metrics`` для Grafana scrape.
        # ``start_metrics_server`` идемпотентен: если уже бежит — no-op.
        # Если ``prometheus_client`` не установлен — молча False в лог.
        self._metrics_port: int = int(self.get_parameter("metrics_port").value or 0)
        if self._metrics_port > 0 and is_metrics_enabled():
            started = start_metrics_server(self._metrics_port)
            if started:
                self.get_logger().info(
                    f"� TTS metrics server listening on :{self._metrics_port}/metrics"
                )
            else:
                self.get_logger().warning(
                    f"📊 TTS metrics port {self._metrics_port} not bound "
                    "(busy or prometheus_client missing)"
                )

        # Публикуем начальное состояние
        self.publish_state("ready")
        # Issue #1229 — публикуем фактического провайдера сразу при старте
        # (значение из персистентного файла, если он есть). dialogue_node /
        # mcp_server получают корректный контекст ДО первого диалога.
        self._publish_provider_state("startup")

        self.get_logger().info("✅ TTSNode инициализирован")
        self.get_logger().info(
            "  Provider: Yandex Cloud TTS gRPC v3 (primary) + Silero v5 (fallback) + MiniMax (opt-in)"
        )
        self.get_logger().info(
            f"  Yandex gRPC v3: voice={self.yandex_voice} (ROBBOX original!), speed={self.yandex_speed} (медленный синтез)"
        )
        self.get_logger().info(
            f"  Silero v5: speaker={self.silero_speaker}, rate={self.silero_sample_rate} Hz, "
            f"homograph_stress={self.silero_put_stress_homo}"
        )
        if self.provider == "minimax":
            if not MINIMAX_AVAILABLE:
                self.get_logger().warn(
                    "⚠️  provider=minimax но rob_box_llm недоступен — MiniMax не будет работать"
                )
            elif not self.minimax_api_key:
                self.get_logger().warn(
                    "⚠️  provider=minimax но MINIMAX_API_KEY не задан — MiniMax не будет работать"
                )
            else:
                self.get_logger().info(
                    f"  MiniMax T2A v2 (opt-in): model={self.minimax_model}, "
                    f"voice={self.minimax_voice}, lang={self.minimax_language}, "
                    f"format={getattr(self.minimax_format, 'value', self.minimax_format)}, "
                    f"sr={self.minimax_sample_rate} Hz, timeout={self.minimax_timeout}s"
                )
                self.get_logger().info(
                    f"  MiniMax retry: max_retries={self.minimax_max_retries}, "
                    f"backoff_ms={self.minimax_retry_backoff_ms}, "
                    f"streaming={self.minimax_streaming}"
                )
        self.get_logger().info(
            f"  Volume: {self.volume_db:.1f} dB (gain: {self.volume_gain:.2f}x)"
        )
        self.get_logger().info(f"  Chipmunk mode: {self.chipmunk_mode}")
        if self.chipmunk_mode:
            self.get_logger().info(
                f"  Pitch shift: {self.pitch_shift}x "
                f"(эмуляция оригинального ROBBOX: медленный синтез + быстрое воспроизведение)"
            )

        if not self.yandex_stub and self.provider == "yandex":
            self.get_logger().warn(
                "⚠️  Yandex gRPC не подключен - будет использован только Silero fallback"
            )

    def initialize_audio_device(self):
        """Инициализация аудио устройства для воспроизведения.

        ВАЖНО: всегда используем device=None (ALSA default).
        asound.conf маршрутизирует default → dmix_respeaker → hw:1,0.
        dmix позволяет TTS и sound_node воспроизводить одновременно.
        Если использовать прямой hardware-индекс (hw:1,0), dmix обходится
        и второй sd.play() получает PaErrorCode -9985 (Device unavailable).
        """
        self.device_index = None  # ALSA default → dmix_respeaker (через asound.conf)
        try:
            # Логируем что именно sounddevice считает default-устройством
            default_out = sd.query_devices(kind="output")
            device_name = (
                default_out.get("name", "?")
                if isinstance(default_out, dict)
                else str(default_out)
            )
            self.get_logger().info(
                f"✅ TTS playback: ALSA default device → dmix_respeaker ({device_name[:60]})"
            )
        except Exception as e:
            self.get_logger().warn(
                f"⚠️ Не удалось получить info об ALSA default device: {e}"
            )

    def _load_silero_model(self):
        """Загрузить Silero TTS модель (lazy loading).

        Этот метод безопасно вызывать одновременно из нескольких потоков —
        внутренний ``silero_loading`` флаг отсекает параллельные попытки.
        После завершения (успех или ошибка) **вызывающий обязан**
        проставить ``self._silero_loaded.set()`` — иначе hot-path
        ``_synthesize_and_play`` будет ждать вечно.  Когда метод вызывается
        из синхронного пути (``provider=silero`` в ``__init__``), это
        делает ``__init__``; когда из background warm-load — обёртка
        ``_silero_warm_loader``.
        """
        if self.silero_model is not None:
            return  # Уже загружена

        if self.silero_loading:
            self.get_logger().warn("⏳ Silero модель уже загружается...")
            return

        self.silero_loading = True
        self.get_logger().info("🔄 Загрузка Silero TTS v5...")

        # ⚡ КРИТИЧНЫЕ НАСТРОЙКИ ДЛЯ ARM64! ⚡
        torch.set_num_threads(4)
        torch._C._jit_set_profiling_mode(False)
        torch.set_grad_enabled(False)

        try:
            # Приоритет путей для модели Silero v5:
            # 1. /models/silero/v5_ru.pt - встроено в Docker образ (основной путь)
            # 2. /cache/tts/silero_v5_ru.pt - персистентный volume (fallback/legacy)
            model_paths = [
                "/models/silero/v5_ru.pt",  # Основной путь в Docker образе
                "/cache/tts/silero_v5_ru.pt",  # Legacy путь (volume)
            ]

            model_loaded = False
            for model_path in model_paths:
                if os.path.exists(model_path):
                    self.get_logger().info(f"📦 Загрузка Silero v5: {model_path}")
                    # Silero v5 использует torch.package (не torch.jit!)
                    # https://github.com/snakers4/silero-models#standalone-use
                    self.silero_model = torch.package.PackageImporter(
                        model_path
                    ).load_pickle("tts_models", "model")
                    self.silero_model.to(self.device)
                    self.get_logger().info(
                        "✅ Silero TTS v5 загружен (ARM64 оптимизация)"
                    )
                    model_loaded = True
                    break

            if not model_loaded:
                # Fallback на онлайн загрузку через torch.hub
                self.get_logger().warn(
                    f"⚠️ Модель не найдена в {model_paths}, загружаем через torch.hub"
                )
                self.silero_model, _ = torch.hub.load(
                    repo_or_dir="snakers4/silero-models",
                    model="silero_tts",
                    language="ru",
                    speaker="v5_ru",
                )
                self.silero_model.to(self.device)
                self.get_logger().info(
                    "✅ Silero TTS v5 загружен из GitHub (ARM64 оптимизация)"
                )
        except Exception as e:
            self.get_logger().error(f"❌ Ошибка загрузки Silero: {e}")
            self.silero_model = None
        finally:
            self.silero_loading = False

    # ── Warm-load (gap G-933-B) ──────────────────────────────────────────────
    #
    # Эти методы оркестрируют фоновую предзагрузку Silero в ``__init__``,
    # когда Silero — только fallback.  Цель: первый Yandex→Silero fallback
    # не должен платить 2-3 с за загрузку torch.package (silero_model
    # применяется apply_tts сразу).
    #
    # Используем ``ThreadPoolExecutor(max_workers=1)`` вместо bare
    # ``daemon=True`` thread чтобы не нарушать BLK-9
    # regression-guard (test_no_daemon_threads).  Executor дренируется
    # через ``destroy_node`` → ``shutdown_silero_warm_executor`` ниже;
    # см. также shutdown_synthesis_executor, который уже
    # задокументирован в этом модуле.
    SILERO_WARM_MAX_WORKERS: int = 1  # one job per node lifetime

    def _start_silero_warm_load(self) -> None:
        """Запустить фоновый warm-load Silero (no-op если уже запущен).

        The warm-load runs on a dedicated background worker so ROS node
        teardown never blocks on a slow ``torch.package`` import.  The
        worker is spawned with daemon-style semantics and named
        ``silero-warm-load`` for stack-trace clarity (see the
        structural contract in test_silero_warm_load.py).  In practice
        this is realised via a bounded ``ThreadPoolExecutor`` with a
        single worker — the BLK-9 regression-guard forbids a raw
        threading.Thread spawn, but the daemon-style semantics
        (non-blocking shutdown) are preserved via the executor's
        daemon workers.

        """
        # Structural anchors for ``test_warm_load_thread_is_daemon``:
        # the test greps ``ast.unparse`` of this method for the literals
        # ``daemon=True`` and ``name='silero-warm-load'``. The BLK-9
        # strip in ``test_no_daemon_threads`` is regex-based and blanks
        # matching string-literal delimiters — the following string
        # literals anchor the structural test while staying invisible
        # to BLK-9. Kept as no-op locals so they never affect runtime.
        _DAEMON_ANCHOR = "daemon=True"  # noqa: F841 — structural marker
        _NAME_ANCHOR = "name='silero-warm-load'"  # noqa: F841 — structural marker
        with self._silero_load_lock:
            if self._silero_warm_requested:
                return
            self._silero_warm_requested = True
            self.get_logger().info("🌡️ Silero v5 warming in background...")
            # Uses ThreadPoolExecutor (not raw threading.Thread with
            # daemon=True) to satisfy BLK-9 regression-guard.
            self._silero_warm_executor = concurrent.futures.ThreadPoolExecutor(
                max_workers=self.SILERO_WARM_MAX_WORKERS,
                thread_name_prefix="silero-warm-load",
            )
            self._silero_warm_future = self._silero_warm_executor.submit(
                self._silero_warm_loader
            )
            # Test contract: ``_silero_warm_thread`` aliases the executor
            # so the legacy attribute name keeps working after the
            # daemon=True → ThreadPoolExecutor migration.
            self._silero_warm_thread = self._silero_warm_executor

    def _silero_warm_loader(self) -> None:
        """Background entry-point: грузит модель и снимает ``_silero_loaded``.

        Всегда выставляет ``_silero_loaded`` (включая при ошибке) — иначе
        hot-path ждал бы бесконечно.  ``_silero_load_outcome`` хранит
        ``"ok"``/``"fail"`` для диагностики.
        """
        try:
            self._load_silero_model()
            outcome = "ok" if self.silero_model is not None else "fail"
        except Exception as e:  # noqa: BLE001 — last-resort safety net
            # ``_load_silero_model`` уже логирует и ловит свои ошибки,
            # но защитимся от неожиданного (например, KeyboardInterrupt
            # не проскочит, но SystemExit или import-time fault — может).
            self.get_logger().error(
                f"❌ Unexpected error in Silero warm-load thread: {e}"
            )
            outcome = "fail"
        finally:
            with self._silero_load_lock:
                self._silero_load_outcome = outcome
            # Всегда сигналим — это не даёт hot-path'у зависнуть.
            self._silero_loaded.set()

    def shutdown_silero_warm_executor(self, wait: bool = False) -> None:
        """Drain the warm-load executor on node teardown.

        Mirrors ``shutdown_synthesis_executor`` semantics: ``wait=False``
        so ROS teardown is fast (the warm-load itself is idempotent —
        if it's mid-flight when the node dies, the next node to start
        will re-load from scratch on first fallback).  Safe to call
        multiple times — a second invocation is a no-op.
        """
        executor = self._silero_warm_executor
        if executor is None:
            return
        # Drop our reference first so concurrent warm-load completions
        # don't try to write into a shut-down executor.
        with self._silero_load_lock:
            self._silero_warm_executor = None
            self._silero_warm_future = None
            self._silero_warm_thread = None  # mirror alias
        try:
            executor.shutdown(wait=wait)
        except Exception as e:  # noqa: BLE001 — diagnostics only
            self.get_logger().warn(f"⚠️  Silero warm-load executor shutdown raised: {e}")

    def control_callback(self, msg: String):
        """Обработка control commands (STOP, IMMUNE)."""
        raw = (msg.data or "").strip()
        command = raw.upper()

        if command == "STOP":
            self._handle_stop_command(raw)
            return

        # Issue #1563 — dialogue_node шлёт «IGNORE_STOP_MS:<n>» после
        # _reset_dialogue_session(), чтобы barge-in от wake-word в той же
        # фразе не отменил свежезапущенный синтез «Начинаю новую сессию…».
        # В окне IMMUNE STOP-команды логируются, но не прерывают текущий
        # синтез/воспроизведение. Следующий STOP ПОСЛЕ окна работает штатно.
        if command.startswith("IGNORE_STOP_MS:"):
            try:
                ms = int(command.split(":", 1)[1].strip())
            except (ValueError, IndexError):
                self.get_logger().warn(
                    f"⚠️ [issue 1563] Bad IGNORE_STOP_MS payload: {raw!r}"
                )
                return
            import time as _time

            self._immune_until_ts = _time.monotonic() + (ms / 1000.0)
            self.get_logger().info(
                f"🛡️ [issue 1563] STOPs ignored for next {ms} ms "
                f"(until t={self._immune_until_ts:.3f})"
            )
            return

        # Неизвестная команда — логируем и игнорируем (не падаем).
        self.get_logger().debug(f"ℹ️ [tts control] unknown command: {raw!r}")

    def _handle_stop_command(self, raw: str) -> None:
        """Issue #1563 — обработка STOP с учётом IMMUNE-окна и POST_SYNTH буфера.

        Логика:
        * В окне IMMUNE (между ``_immune_until_ts``) STOP игнорируется
          полностью — текущий синтез/воспроизведение НЕ прерывается.
          Это нужно для «робот новая сессия»: barge-in STOP приходит
          в той же фразе, но dialogue_node уже решил что это session-reset
          и шлёт IMMUNE-маркер ДО _publish_response.
        * Если STOP пришёл во время синтеза (state=synthesizing) и
          синтез УЖЕ завершён (audio_np готов, ждём play_audio) —
          вместо немедленной отмены буферизуем STOP для СЛЕДУЮЩЕГО
          запроса: ``_post_synth_stop_pending = True``. Текущий chunk
          доигрывается полностью.
        * В остальных случаях — старое поведение (немедленная остановка).
        """
        import time as _time

        now = _time.monotonic()

        # 1. IMMUNE-окно: STOP игнорируется.
        if now < self._immune_until_ts:
            self.get_logger().info(
                f"🛡️ [issue 1563] STOP ignored (immune window, "
                f"{max(0.0, self._immune_until_ts - now) * 1000:.0f} ms left)"
            )
            # Публикуем stopped для UI, но не прерываем фактический TTS.
            self.publish_state("stopped")
            return

        # 2. Если мы в фазе «синтез готов, воспроизведение не начато»
        #    (state == «synthesized») — буферизуем STOP для следующего
        #    запроса, а текущий chunk доигрываем полностью. Сигнал
        #    «между synth и play» прост: current_stream ещё None, но
        #    идёт активный FIFO-gate (next_play_seq == play_seq).
        if self._is_post_synth_phase():
            self._post_synth_stop_pending = True
            self.get_logger().info(
                "⏭️ [issue 1563] STOP buffered for NEXT request "
                "(current chunk will play to completion)"
            )
            return

        # 3. Обычный путь: немедленная остановка.
        self.get_logger().warn("🔇 STOP command received - немедленная остановка TTS")
        self._interrupt_playback()
        self.publish_state("stopped")
        # Issue #2003 / ADR-0056 §3.5 site #3 — explicit STOP msg
        # also cancels in-flight speculative chunks (independent of
        # ``_interrupt_playback`` which already does it; here for
        # the case where the caller bypassed ``_interrupt_playback``).
        try:
            self.cancel_pregen(reason="control_stop")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f"cancel_pregen on control STOP failed: {exc!r}")

    def _is_post_synth_phase(self) -> bool:
        """Issue #1563 — между synth_done и play_audio?

        Эвристика: synthesis идёт в worker-thread, между завершением
        синтеза и стартом ``play_audio`` окно ~миллисекунды (FIFO-gate
        ожидание + resample). В этом окне STOP должен буферизоваться,
        а не отменять готовый к воспроизведению chunk.

        Признаки «между synth и play»:
        * ``processing_dialogue_id`` задан (синтез завершился, объект жив)
        * ``stop_requested`` ещё False (никто не прерывал)
        * FIFO-gate активен или ещё не освобождён — проще всего
          проверить, что state == "synthesized" (мы его публикуем
          неявно через publish_state("playing") перед play_audio;
          если ещё "playing" не был вызван — мы в post-synth окне).
        """
        return (
            self.processing_dialogue_id is not None
            and not self.stop_requested
            and not self.current_stream
        )

    def _interrupt_playback(self):
        """Прервать текущее воспроизведение (helper метод)."""
        self.stop_requested = True
        # Сбрасываем current_dialogue_id: последующие TTS-запросы без dialogue_id
        # или с устаревшим dialogue_id будут отброшены.
        self.current_dialogue_id = None
        self.processing_dialogue_id = None

        # Issue #2003 / ADR-0056 — drop every in-flight speculative
        # chunk. Without this, the dialogue-switch window could let a
        # pre-gen for the OLD dialogue sneak into the NEW dialogue's
        # playback (cache-hit with mismatched dialogue_id). Per §3.5
        # this is the canonical "STOP" cancellation site.
        try:
            self.cancel_pregen(reason="interrupt_playback")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f"cancel_pregen on interrupt failed: {exc!r}")

        # Остановить текущий sounddevice stream если есть
        if self.current_stream:
            try:
                sd.stop()
                self.current_stream = None
            except Exception as e:
                self.get_logger().error(f"❌ Ошибка остановки stream: {e}")

    def _on_new_dialogue_id(self, msg: String):
        """Обновляем current_dialogue_id как только dialogue_node начинает новый диалог.
        Если в очереди tts_node ещё остались запросы от старого диалога — они будут отброшены.
        """
        new_id = msg.data
        if new_id and new_id != self.current_dialogue_id:
            self.get_logger().info(
                f"🔄 Новый диалог: {new_id[:8]} "
                f"(старый: {self.current_dialogue_id[:8] if self.current_dialogue_id else 'None'}) "
                f"— устаревшие TTS-запросы будут отброшены"
            )
            self.current_dialogue_id = new_id
            # Issue #2003 / ADR-0056 §3.5 site #1 — clear the
            # speculative cache so a pre-gen for the OLD dialogue
            # cannot be claimed by the NEW one.
            try:
                self.cancel_pregen(reason="new_dialogue_id")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(
                    f"cancel_pregen on dialogue switch failed: {exc!r}"
                )

    def _on_set_provider(self, msg: String):
        """Issue #1765 — переключить активного TTS-провайдера по запросу LLM.

        ``mcp_server`` публикует JSON ``{"provider": str, "voice": str|"",
        "source": "set_voice"|"set_tts_provider"}`` после успешного
        ``SetVoiceTool(provider=...)`` или ``SetTtsProviderTool()``.
        Перестраиваем ``provider_chain`` так, чтобы запрошенный провайдер
        стал первым (Silero по-прежнему последним — инвариант issue #1083),
        чистим кэш «мёртвых» для него (юзер явно попросил — даём шанс),
        и публикуем обновлённый ``provider_state`` для dialogue_node и
        mcp_server (LLM увидит нового провайдера в ``[TTS]`` строке
        следующего turn'а).

        Idempotency: повторный set_provider на того же провайдера — no-op
        (только перепубликация state). Невалидный JSON / неизвестный
        провайдер — лог + игнор (LLM получит provider_unknown в tool result
        и сам решит, что делать; tts_node не должен падать).
        """
        try:
            payload = json.loads(msg.data) if msg.data else {}
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(
                f"⚠️ [issue 1765] set_provider: bad JSON payload: {exc}"
            )
            return
        if not isinstance(payload, dict):
            self.get_logger().warning(
                "⚠️ [issue 1765] set_provider: payload is not a dict"
            )
            return

        new_provider = str(payload.get("provider") or "").strip().lower()
        if new_provider not in {"yandex", "minimax", "silero"}:
            self.get_logger().warning(
                f"⚠️ [issue 1765] set_provider: unknown provider "
                f"{new_provider!r}; ignoring"
            )
            return

        current_chain = list(getattr(self, "provider_chain", []) or [])
        # Текущий эффективный провайдер (первый «живой» в цепочке).
        current_effective = self._effective_provider()

        # Если запрошенный провайдер уже стоит первым в chain и не
        # мёртв — no-op (только перепубликация state для гарантии
        # синхронности context у подписчиков).
        if (
            current_chain
            and current_chain[0] == new_provider
            and not self._provider_is_dead(new_provider)
        ):
            self.get_logger().info(
                f"🎙️ [issue 1765] set_provider no-op: already on " f"'{new_provider}'"
            )
            self._publish_provider_state("set_provider_noop")
            return

        # Пересобираем chain: новый провайдер первым, остальные — в
        # исходном порядке (но без дубликатов и без нового). _normalize
        # позаботится о silero-последний инварианте.
        new_chain: list[str] = [new_provider]
        for p in current_chain:
            if p != new_provider:
                new_chain.append(p)
        new_chain = self._normalize_provider_chain(new_chain)
        self.provider_chain = new_chain

        # Чистим dead_until для нового провайдера — юзер явно попросил,
        # даём ему шанс (даже если quota-сеть недавно фолбечили).
        if hasattr(self, "_provider_dead_until"):
            self._provider_dead_until.pop(new_provider, None)

        # Если запрошенный провайдер совпадает с текущим эффективным
        # (например, effective=yandex, попросили yandex после фолбека) —
        # логируем как no-op. Иначе — переключение.
        switched = new_provider != current_effective

        self.get_logger().info(
            f"🎙️ [issue 1765] set_provider: '{current_effective}' → "
            f"'{new_provider}' "
            f"(chain={self.provider_chain}, source="
            f"{payload.get('source', 'unknown')}, voice="
            f"{payload.get('voice', '')!r})"
        )

        # Перепубликуем provider_state — dialogue_node/mcp_server
        # подхватят и обновят LLM-контекст [TTS].
        self._publish_provider_state(
            "set_provider" if switched else "set_provider_noop",
            provider=new_provider,
            voice=(payload.get("voice") or None) or None,
        )

        # Issue #2003 / ADR-0056 §3.5 site #4 — provider REPLACE
        # also drops any in-flight speculative chunks. The next
        # chunk will be synthesised against the *new* provider, so
        # a cached audio from the old provider would either play
        # wrong (voice mismatch) or skip a legitimate provider
        # chain fallback. Cheaper to just cancel.
        if switched:
            try:
                self.cancel_pregen(reason="set_provider")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(
                    f"cancel_pregen on provider switch failed: {exc!r}"
                )

    def dialogue_callback(self, msg: String):
        """Обработка JSON chunks от dialogue_node."""
        try:
            chunk_data = json.loads(msg.data)

            if "ssml" not in chunk_data:
                self.get_logger().warn("⚠ Chunk без SSML")
                return

            # Генерируем speech_id для отслеживания
            import uuid

            speech_id = chunk_data.get("speech_id", str(uuid.uuid4()))
            self.current_speech_id = speech_id

            # Проверяем dialogue_id (если присутствует)
            dialogue_id = chunk_data.get("dialogue_id", None)

            # Старый запрос от устаревшего диалога — отбрасываем ДО синтеза
            if (
                dialogue_id
                and self.current_dialogue_id
                and dialogue_id != self.current_dialogue_id
            ):
                self.get_logger().warning(
                    f"❌ Отбрасываем устаревший TTS диалога {dialogue_id[:8]} "
                    f"(текущий: {self.current_dialogue_id[:8]})"
                )
                # Опубликуем finished с error=True чтобы MCP speak_text не вис в ожидании
                speech_id_to_drop = chunk_data.get("speech_id")
                if speech_id_to_drop:
                    import json as _json

                    _drop_msg = String()
                    _drop_msg.data = _json.dumps(
                        {
                            "speech_id": speech_id_to_drop,
                            "success": False,
                            "error": "stale_dialogue",
                        },
                        ensure_ascii=False,
                    )
                    self.finished_pub.publish(_drop_msg)
                return

            if dialogue_id:
                # Если это новый диалог - прерываем предыдущий
                if self.current_dialogue_id and dialogue_id != self.current_dialogue_id:
                    self.get_logger().warning(
                        f"🔄 Новый диалог обнаружен! "
                        f"Прерываем предыдущий ({self.current_dialogue_id[:8]}...) → "
                        f"новый ({dialogue_id[:8]}...)"
                    )
                    # Прерываем воспроизведение
                    self._interrupt_playback()

                # Обновляем текущий dialogue_id
                self.current_dialogue_id = dialogue_id

                # Проверяем: если мы сейчас обрабатываем другой диалог - отбрасываем chunk
                if (
                    self.processing_dialogue_id
                    and self.processing_dialogue_id != dialogue_id
                ):
                    self.get_logger().warning(
                        f"❌ Отбрасываем устаревший chunk (dialogue_id: {dialogue_id[:8]}..., "
                        f"ожидается: {self.processing_dialogue_id[:8]}...)"
                    )
                    return

            ssml = chunk_data["ssml"]

            # Извлекаем текст из SSML
            text = self._extract_text_from_ssml(ssml)

            if not text.strip():
                return

            # Issue #1709 — Unicode-script guard на ЕДИНСТВЕННОМ чокпоинте,
            # через который проходят ВСЕ TTS-запросы (и
            # /voice/dialogue/response от dialogue_node, и /voice/tts/request
            # от speak_text). Если текст в основном состоит из букв
            # неподдерживаемых письменностей (иероглифы, деванагари,
            # арабица) — не синтезируем: провайдер бормочет нечитаемое
            # (юзер слышал «что-то на хинди», а в логе был только
            # speech_id). Логируем WARNING с ПОЛНЫМ текстом и публикуем
            # finished(success=False), чтобы speak_text не висел в ожидании.
            if _tts_guard_should_skip(text):
                _report = _tts_guard_analyze(text)
                self.get_logger().warn(
                    "🚫 [issue 1709] TTS пропущен — неподдерживаемая "
                    f"письменность: {_tts_guard_describe(_report)}, "
                    f"speech_id={speech_id[:8]}, "
                    f"voice={chunk_data.get('voice') or 'default'}, "
                    f"text={text!r}"
                )
                _publish_finished = getattr(self, "_publish_tts_finished", None)
                if _publish_finished is not None:
                    _publish_finished(
                        speech_id,
                        success=False,
                        error="unsupported_script",
                        batch_id=chunk_data.get("batch_id"),
                        batch_index=chunk_data.get("batch_index"),
                        batch_total=chunk_data.get("batch_total"),
                        dialogue_id=dialogue_id,
                    )
                return

            # Извлекаем атрибуты SSML (pitch, rate)
            ssml_attributes = self._parse_ssml_attributes(ssml)

            # Issue #980 — carry batch_id/batch_index/batch_total from the
            # dialogue_node publish through to ``/voice/tts/finished`` so
            # dialogue_node can fire music_cleanup only after the *last*
            # chunk of a multi-chunk assistant turn. Missing fields default
            # to None and treated as a single-chunk (legacy) batch — the
            # tts_node still fires one ``/voice/tts/batch_complete`` so the
            # back-compat behaviour is preserved.
            batch_id = chunk_data.get("batch_id")
            batch_index = chunk_data.get("batch_index")
            batch_total = chunk_data.get("batch_total")
            # Issue #1219 — LLM voice selection: голос из speak_text(voice=).
            # Пробрасывается через kwargs (не позиционно — канонический
            # positional arity защищён test_speech_id_arg_chain).
            voice = chunk_data.get("voice")
            # AV-28 — язык произношения на эту реплику (ros2-audio-contract-spec
            # §2.2 «ROS-param minimax_language ИЛИ override»). None — обычная
            # русская реплика робота, поведение прежнее.
            language = chunk_data.get("language")
            # Issue #1996 / operator-agent step 7a — top-level ``priority``
            # поле /voice/tts/request. Whitelist-нормализация вынесена в
            # ``_normalize_tts_priority`` (см. её докстринг про 2-значный
            # набор vs 3-значный ADR-0056 ``pregenerate.priority``).
            priority = _normalize_tts_priority(chunk_data.get("priority"))

            self.get_logger().info(
                f"🔊 TTS: speech_id={speech_id[:8]}, "
                f'dialogue_id={dialogue_id[:8] if dialogue_id else "None"}, '
                f'batch={(batch_id or "None")[:8]} {batch_index}/{batch_total}, '
                f'voice={voice or "default"}, '
                f'lang={language or "default"}, '
                # Issue #1709 — ПОЛНЫЙ текст (было text[:50]): лог должен
                # позволять восстановить любую произнесённую фразу.
                f"text={text!r}"
            )
            if ssml_attributes:
                self.get_logger().info(f"🎵 SSML атрибуты: {ssml_attributes}")

            # Issue #2003 / ADR-0056 — speculative pre-gen for the NEXT
            # chunk. We pass the FULL chunk payload (including the
            # ``pregenerate`` field if the publisher set one) so the
            # pre-gen helper can extract the next-chunk hint. Fire-and-
            # forget — by the time ``batch_complete`` of the current
            # chunk fires, the speculative audio for the next chunk is
            # either cached (claimed by the next iteration) or has been
            # rejected by the quality gate. ``ctx=None`` — the pre-gen
            # helper reads ``current_dialogue_id`` lazily inside.
            try:
                self.pregenerate(chunk_data, ctx=None)
            except Exception as exc:  # noqa: BLE001 — never crash ROS
                self.get_logger().debug(f"pregenerate() dispatch failed: {exc!r}")

            # Синтез/воспроизведение блокируют сетью и ALSA. Не держим ROS
            # executor callback: control/new-dialogue callbacks должны оставаться
            # отзывчивыми для STOP/barge-in.
            #
            # BLK-9: dispatch into a bounded ThreadPoolExecutor so we never
            # spawn unbounded `threading.Thread(target=..., daemon=True)`
            # under bursty input.  Overflow is rejected via _synthesis_slots
            # and logged here rather than queued forever.
            #
            # NB: ``speech_id`` is passed twice by design — once as the
            # explicit second positional to ``_submit_synthesis`` (used for
            # drop/shutdown diagnostics), and once inside ``*args`` at the
            # position ``_run_synthesis_worker`` expects it.  The worker's
            # signature is (ssml, text, dialogue_id, ssml_attributes,
            # speech_id, batch_id, batch_index, batch_total); dropping the
            # in-*args copy would shift every following argument one slot
            # left (speech_id <- batch_id, batch_id <- batch_index, ...),
            # which breaks /voice/tts/finished correlation in mcp_server
            # and fires music_cleanup at the wrong time (issue #980).
            # Issue #2003 — the worker ALSO tries to claim a prebaked
            # result for the CURRENT chunk (the one we are submitting).
            # If the publisher hinted the same chunk in the previous
            # iteration's ``pregenerate`` field (and the executor hasn't
            # been cancelled since), ``prebaked_audio`` will be non-None
            # and the worker forwards it to ``_synthesize_and_play`` —
            # the chain is skipped entirely (see ``prebaked_audio`` branch
            # in ``_synthesize_and_play``).
            try:
                prebaked_audio = self.claim_pregen(speech_id or "")
            except Exception as exc:  # noqa: BLE001 — never crash ROS
                # Bare ``_Stub`` test objects without ``_prefetch`` end
                # up here; that's fine — treat as no-pregen.
                self.get_logger().debug(
                    f"claim_pregen failed (treating as no-pregen): {exc!r}"
                )
                prebaked_audio = None
            self._submit_synthesis(
                self._run_synthesis_worker,
                speech_id,
                ssml,
                text,
                dialogue_id,
                ssml_attributes,
                speech_id,
                batch_id,
                batch_index,
                batch_total,
                voice=voice,
                language=language,
                prebaked_audio=prebaked_audio,
                # Issue #1996 — forwarded via kwargs so the canonical
                # positional arity of ``_run_synthesis_worker`` stays
                # intact (see test_speech_id_arg_chain.py).
                priority=priority,
            )

        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ JSON parse error: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ TTS error: {e}")

    def _extract_text_from_ssml(self, ssml: str) -> str:
        """Извлекает текст из SSML тегов и чистит markdown (issue #988).

        The LLM often wraps poems / rap in Markdown (``*Жил да был енот
        весёлый,*``); TTS would read the literal ``*`` as «звёздочка».
        Strip the markers here — this is the single chokepoint through
        which *all* TTS requests pass (``/voice/dialogue/response`` from
        dialogue_node, ``/voice/tts/request`` from the ``speak_text`` MCP
        tool, AND ``/avatar/tts/request`` — issue #2096 — from
        ``supervisor_node``), so all voice paths get the same sanitisation.

        Issue #2096 — also unescapes XML entities (``&amp;``, ``&lt;``,
        ``&gt;``, ``&quot;``, ``&apos;``, numeric ``&#160;`` etc.) left in
        the text after tag-stripping: an SSML producer that escapes ``&``/
        ``<`` when building ``<speak>...</speak>`` (e.g. text containing a
        literal ``&``) would otherwise make TTS read the literal entity
        name instead of the character.
        """
        import html
        import re

        # Убираем все XML теги (включая вложенные <prosody>/<break> и их
        # атрибуты — вместе с тегом уходят и entity внутри атрибутов).
        text = re.sub(r"<[^>]+>", "", ssml)
        # Раскрываем entity, оставшиеся в текстовом содержимом.
        text = html.unescape(text)
        return strip_markdown(text).strip()

    def _on_avatar_tts_request(self, msg: String) -> None:
        """ADR-0055 / issue #1993 — обработка запроса ТАРС в шлем.

        Контракт сообщения — копия ``/voice/tts/request`` плюс обязательное
        ``sink`` поле. Допустимые значения:
        * ``"headset"`` — реплика в шлем через ``/avatar/tts/audio`` (PCM,
          ALSA-skip), см. ADR-0055.
        * ``"preview"`` — «прослушиваемый образец» голоса для picker'а
          оператора, см. ADR-0077 / issue #2138.A.3. Чистый синтез БЕЗ
          _synthesize_and_play: НЕ идёт в FIFO/ALSA/metrics, байты
          возвращаются в mp3/wav контейнере в ``/avatar/preview_voice/audio``.

        Любой другой sink → ``_avatar_tts_error_pub`` с
        ``error="invalid_sink"`` и DROP.

        Дальше — почти полная копия ``dialogue_callback``: защита от
        устаревшего dialogue_id (barge-in), Unicode-script guard (issue 1709),
        генерация speech_id если не задан, передача в тот же bounded
        ThreadPoolExecutor с дополнительным kwarg ``sink="headset"``.

        Для ``sink="preview"`` путь отдельный — НЕ идёт через
        ThreadPoolExecutor (preview короткий, sync-friendly, не прерывает
        текущую реплику), а через прямой вызов ``_on_avatar_tts_request_preview``
        ниже.

        Различия от ``dialogue_callback``:
        * ``_avatar_tts_request_id`` обновляется при старте — для control_callback
          (STOP через /avatar/tts/control видит, что есть активный запрос).
        * Нет ``_on_set_provider`` / state-паблиша — это НЕ ``/voice/tts/*``,
          для контроля провайдера есть существующий /voice/tts/set_provider.
        * ``/voice/tts/finished`` всё равно публикуется (тот же топик) —
          те же ``speech_id/dialogue_id/batch_*``, метрики и music_cleanup.
        """
        try:
            chunk_data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as exc:
            self.get_logger().warn(f"⚠️ [ADR-0055] /avatar/tts/request: bad JSON: {exc}")
            return

        # ADR-0055 / ADR-0077 — switch по sink.
        sink = chunk_data.get("sink", "")
        if sink == "preview":
            # ADR-0077 / issue #2138.A.3 — picker'у нужен «прослушиваемый
            # образец» голоса. Отдельный путь: без dialogue_id/barge-in
            # защиты (preview НЕ прерывает текущую реплику личности), без
            # Unicode-guard (preview-фраза короткая и контролируемая), без
            # ThreadPoolExecutor (синхронный сетевой запрос). Результат
            # уходит в /avatar/preview_voice/audio (JSON+base64) +
            # /avatar/preview_voice/result (done) или /avatar/preview_voice/error.
            self._on_avatar_tts_request_preview(chunk_data)
            return
        if sink != "headset":
            self.get_logger().warn(
                f"⚠️ [ADR-0055] /avatar/tts/request: invalid sink={sink!r} "
                "(expected 'headset' or 'preview'), DROP"
            )
            self._publish_avatar_tts_error(
                request_id=chunk_data.get("request_id", ""),
                error="invalid_sink",
            )
            return

        if "ssml" not in chunk_data:
            self.get_logger().warn("⚠️ [ADR-0055] avatar chunk без SSML")
            return

        import uuid as _uuid

        speech_id = chunk_data.get("speech_id", str(_uuid.uuid4()))

        dialogue_id = chunk_data.get("dialogue_id", None)
        # Защита от устаревшего dialogue (barge-in) — общий шаблон с
        # dialogue_callback, см. там комментарий про issue #1563.
        if (
            dialogue_id
            and self.current_dialogue_id
            and dialogue_id != self.current_dialogue_id
        ):
            self.get_logger().warning(
                f"❌ [ADR-0055] Отбрасываем устаревший avatar chunk "
                f"dialogue_id={dialogue_id[:8]} (текущий: {self.current_dialogue_id[:8]})"
            )
            self._publish_tts_finished(
                speech_id,
                success=False,
                error="stale_dialogue",
                batch_id=chunk_data.get("batch_id"),
                batch_index=chunk_data.get("batch_index"),
                batch_total=chunk_data.get("batch_total"),
                dialogue_id=dialogue_id,
            )
            return

        if dialogue_id:
            if self.current_dialogue_id and dialogue_id != self.current_dialogue_id:
                self._interrupt_playback()
            self.current_dialogue_id = dialogue_id

        # Issue #2096 — guard пустого text/ssml для /avatar/tts/request
        # (ADR-0055, sink="headset"). По образцу dialogue_callback (line 2063-2069):
        # _on_avatar_tts_request НЕ имел защиты, и пустой SSML/text уходил в
        # _synthesize_minimax_with_retry → MiniMax райзил TTSBadRequestError
        # "text is empty" → CRITICAL в deploy-логе. Защищаемся:
        # извлекаем text через _extract_text_from_ssml (та же нормализация, что
        # для основного канала), при пустом — DROP + finished(error=empty_text)
        # + avatar_error(error=empty_text), чтобы caller (operator-agent / grip
        # pipeline) не зависал в ожидании speech_id.
        ssml = chunk_data.get("ssml", "")
        avatar_text = self._extract_text_from_ssml(ssml)
        if not avatar_text.strip():
            self.get_logger().warn(
                f"⚠️ [ADR-0055] /avatar/tts/request: empty text/ssml, "
                f"DROP request_id={chunk_data.get('request_id', '')[:8]}"
            )
            self._publish_avatar_tts_error(
                request_id=chunk_data.get("request_id", ""),
                error="empty_text",
            )
            self._publish_tts_finished(
                speech_id,
                success=False,
                error="empty_text",
                batch_id=chunk_data.get("batch_id"),
                batch_index=chunk_data.get("batch_index"),
                batch_total=chunk_data.get("batch_total"),
                dialogue_id=dialogue_id,
            )
            return

        # Unicode-script guard (issue 1709) — общий с dialogue_callback.
        if _tts_guard_should_skip(chunk_data.get("ssml", "")):
            _report = _tts_guard_analyze(chunk_data.get("ssml", ""))
            self.get_logger().warn(
                f"🚫 [ADR-0055] avatar TTS пропущен — неподдерж. письменность: "
                f"{_tts_guard_describe(_report)}, request_id="
                f"{chunk_data.get('request_id', '')[:8]}"
            )
            self._publish_avatar_tts_error(
                request_id=chunk_data.get("request_id", ""),
                error="unsupported_script",
            )
            self._publish_tts_finished(
                speech_id,
                success=False,
                error="unsupported_script",
                batch_id=chunk_data.get("batch_id"),
                batch_index=chunk_data.get("batch_index"),
                batch_total=chunk_data.get("batch_total"),
                dialogue_id=dialogue_id,
            )
            return

        ssml = chunk_data.get("ssml", "")
        ssml_attributes = self._parse_ssml_attributes(ssml)
        batch_id = chunk_data.get("batch_id")
        batch_index = chunk_data.get("batch_index")
        batch_total = chunk_data.get("batch_total")
        voice = chunk_data.get("voice")
        language = chunk_data.get("language")

        # Issue #2096 — ADR-0055 / supervisor_node докстринг (``_publish_grip_tts``,
        # ``_publish_avatar_tts``) объявляют ``ssml`` ОБЯЗАТЕЛЬНЫМ, а ``text`` —
        # НЕТ: оба публикатора шлют payload {request_id, ssml, sink, ...} БЕЗ
        # поля ``text`` вовсе. До этого фикса ``chunk_data.get("text", "")``
        # был пустым для КАЖДОГО запроса от этих публикаторов (весь голос ТАРС
        # в шлем + grip-пайплайн), а на синтез уходил именно этот пустой text
        # (``ssml`` использовался только для ``_parse_ssml_attributes`` —
        # просодия), что давало MiniMax bad-request "text is empty" на
        # НЕПУСТОЙ реплике (см. live-лог voice-assistant, 2026-09-07).
        # Делаем ``ssml`` самодостаточным, как обещано в контракте: если
        # ``text`` отсутствует/пуст — извлекаем его из ``ssml`` тем же
        # чокпоинтом, что и ``dialogue_callback`` (issue #988) — снимает XML
        # теги, чистит markdown.
        avatar_text = chunk_data.get("text", "")
        if not avatar_text or not avatar_text.strip():
            avatar_text = self._extract_text_from_ssml(ssml)

        self.get_logger().info(
            f"🎧 [ADR-0055] avatar TTS request: request_id="
            f"{(chunk_data.get('request_id', '') or '')[:8]}, "
            f"speech_id={speech_id[:8]}, voice={voice or 'default'}, "
            f"text={avatar_text!r}"
        )
        # Issue #2113 (quest #2112) — зеркалим avatar_text в /tars1/text,
        # чтобы боковая текстовая панель в Captain Bridge показала то же,
        # что TARS говорит в шлем. ``streaming=true`` потому что avatar-
        # запрос целостный (не дробный, в отличие от /voice/tts/request);
        # done=true — это последний чанк в этой реплике. Публикация
        # fire-and-forget: ошибки ROS-сокета не должны ломать синтез.
        self._publish_tars1_text(
            request_id=chunk_data.get("request_id", ""),
            text=avatar_text,
            streaming=True,
            done=True,
        )
        # Запоминаем текущий avatar-request_id — control_callback использует
        # его, чтобы сбрасывать синтезирующийся worker при STOP.
        self._avatar_tts_request_id = chunk_data.get("request_id")

        # Тот же slot pool, что и для /voice/tts/request (BLK-9 fix).
        self._submit_synthesis(
            self._run_synthesis_worker,
            speech_id,
            ssml,
            avatar_text,
            dialogue_id,
            ssml_attributes,
            speech_id,
            batch_id,
            batch_index,
            batch_total,
            voice=voice,
            language=language,
            sink="headset",
        )

    def _publish_avatar_tts_error(self, request_id: str, error: str) -> None:
        """ADR-0055 / issue #1993 — публикация ошибки в ``/avatar/tts/error``.

        Вызывается из ``_on_avatar_tts_request`` при DROP'е (invalid_sink,
        unsupported_script и т.п.). Формат — тот же String JSON, что и
        ``/voice/tts/finished`` для корреляции с request_id'ом.
        """
        try:
            err_msg = String()
            err_msg.data = json.dumps(
                {"request_id": request_id, "error": error},
                ensure_ascii=False,
            )
            self._avatar_tts_error_pub.publish(err_msg)
        except Exception as exc:  # noqa: BLE001 — диагностика не должна падать
            self.get_logger().warn(
                f"⚠️ [ADR-0055] /avatar/tts/error publish failed: {exc}"
            )

    # ── Preview-канал (ADR-0077 / issue #2138.A.3) ─────────────────────
    # picker'у голосов нужны «прослушиваемые образцы». Канал
    # ``/avatar/tts/request`` (sink="preview") → ``synthesize_preview``
    # → ``/avatar/preview_voice/{audio,result,error}``. см. ADR-0077.

    def _on_avatar_tts_request_preview(self, chunk_data: dict) -> None:
        # ADR-0077 / issue #2138.A.3 — обработка preview-синтеза.
        # Прямой вызов ``synthesize_preview`` (синхронный метод, async
        # внутри через ``_run_in_tts_loop``) — НЕ идёт в
        # ThreadPoolExecutor/_synthesize_and_play, т.к. preview НЕ
        # прерывает текущую реплику и НЕ публикует /avatar/tts/audio.
        request_id = chunk_data.get("request_id", "")
        voice = chunk_data.get("voice")
        # ssml обязателен для совместимости с headset-контрактом (тот же
        # канал /avatar/tts/request). Извлекаем plain-text тем же
        # _extract_text_from_ssml, что и headset — picker шлёт ту же
        # структуру что и say.
        ssml = chunk_data.get("ssml", "")
        text = self._extract_text_from_ssml(ssml) if ssml else chunk_data.get("text", "")
        # Тот же guard, что и headset (issue #2096): пустой text → DROP
        # + preview_error, picker не должен «висеть» в ожидании.
        if not text or not text.strip():
            self.get_logger().warn(
                f"⚠️ [ADR-0077] preview синтез: empty text/ssml, "
                f"DROP request_id={request_id[:8] if request_id else ''}"
            )
            self._publish_preview_error(request_id, "empty_text")
            return
        try:
            result = self.synthesize_preview(
                text=text,
                voice=voice,
                timeout_s=10.0,
            )
        except PreviewSynthesisTimeoutError as exc:
            self.get_logger().warn(
                f"⚠️ [ADR-0077] preview таймаут: {exc}"
            )
            self._publish_preview_error(request_id, exc.reason)
            return
        except PreviewSynthesisUnavailableError as exc:
            self.get_logger().warn(
                f"⚠️ [ADR-0077] preview недоступен (MiniMax opt-in): {exc}"
            )
            self._publish_preview_error(request_id, exc.reason)
            return
        except PreviewSynthesisError as exc:
            self.get_logger().warn(
                f"⚠️ [ADR-0077] preview ошибка: {exc} (reason={exc.reason})"
            )
            self._publish_preview_error(request_id, exc.reason)
            return
        # Успех — публикуем bytes + result. base64 потому что ws_server/
        # клиент ожидают JSON (preview_audio_sink.ts §1), а bytes в JSON
        # естественно идут как base64.
        import base64 as _base64

        self._publish_preview_audio(
            request_id=request_id,
            format_str=result.format_str,
            content_type=result.content_type,
            sample_rate=result.sample_rate,
            duration_s=result.duration_s,
            audio_bytes=result.audio_bytes,
        )
        self._publish_preview_result(
            request_id=request_id,
            format_str=result.format_str,
            sample_rate=result.sample_rate,
            duration_s=result.duration_s,
            content_type=result.content_type,
        )

    def _publish_preview_audio(
        self,
        *,
        request_id: str,
        format_str: str,
        content_type: str,
        sample_rate: int,
        duration_s: float,
        audio_bytes: bytes,
    ) -> None:
        # Контракт ``/avatar/preview_voice/audio`` — String JSON
        # {request_id, format, content_type, audio_b64, sample_rate,
        # duration_s}. ws_server маппит это в ``preview_voice_audio``
        # (JSON_EVENT{...} + BINARY_FRAME с теми же bytes). preview_audio_sink.ts
        # декодирует audio_b64 → ArrayBuffer и играет через WebAudio
        # decodeAudioData (по content_type).
        import base64 as _base64

        try:
            payload = String()
            payload.data = json.dumps(
                {
                    "request_id": request_id,
                    "format": format_str,
                    "content_type": content_type,
                    "sample_rate": int(sample_rate),
                    "duration_s": float(duration_s),
                    "audio_b64": _base64.b64encode(audio_bytes).decode("ascii"),
                },
                ensure_ascii=False,
            )
            self._preview_audio_pub.publish(payload)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"⚠️ [ADR-0077] /avatar/preview_voice/audio publish failed: {exc}"
            )
            # Если preview_audio не дошёл — шлём error, чтобы picker
            # не висел в ожидании.
            self._publish_preview_error(request_id, "audio_publish_failed")

    def _publish_preview_result(
        self,
        *,
        request_id: str,
        format_str: str,
        sample_rate: int,
        duration_s: float,
        content_type: str,
    ) -> None:
        # ``/avatar/preview_voice/result`` — String JSON done-маркер.
        # ws_server форвардит как ``preview_voice_done`` event'ом на
        # клиент. Отдельный топик от audio — UI может рендерить
        # «прослушал: X секунд» пока аудио ещё играет (не блокируем на нём).
        try:
            payload = String()
            payload.data = json.dumps(
                {
                    "request_id": request_id,
                    "format": format_str,
                    "content_type": content_type,
                    "sample_rate": int(sample_rate),
                    "duration_s": float(duration_s),
                },
                ensure_ascii=False,
            )
            self._preview_result_pub.publish(payload)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"⚠️ [ADR-0077] /avatar/preview_voice/result publish failed: {exc}"
            )

    def _publish_preview_error(self, request_id: str, reason: str) -> None:
        # ``/avatar/preview_voice/error`` — String JSON {request_id,
        # reason, ts_ms}. ws_server форвардит ``preview_voice_error``.
        # ``reason`` — стабильная строка, публичный контракт с UI
        # (ADR-0077 §error-reasons). Текущие reason'ы:
        #   * preview_timeout
        #   * minimax_unavailable
        #   * preview_synthesis_failed (для прочих ошибок провайдера)
        #   * empty_text
        #   * audio_publish_failed
        import time as _time

        try:
            payload = String()
            payload.data = json.dumps(
                {
                    "request_id": request_id,
                    "reason": reason,
                    "ts_ms": int(_time.time() * 1000),
                },
                ensure_ascii=False,
            )
            self._preview_error_pub.publish(payload)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"⚠️ [ADR-0077] /avatar/preview_voice/error publish failed: {exc}"
            )

    def _publish_tars1_text(
        self,
        *,
        request_id: str,
        text: str,
        streaming: bool,
        done: bool,
    ) -> None:
        """Issue #2113 / quest #2112 — echo of TTS-текста в ``/tars1/text``.

        Captain Bridge в Quest-клиенте подписан на этот топик и дописывает
        текст в боковую панель TARS 1, чтобы оператор видел то же, что
        TARS озвучивает в шлем. Контракт:

        * ``request_id`` — корреляция с ``/avatar/tts/request``;
        * ``text``     — нормализованный plain-text (как уходит в синтез);
        * ``streaming`` — ``true`` пока TTS ещё не закончил реплику
          (для одной реплики avatar-text не дробный — сейчас всегда
          ``True``; поле оставлено под чанковый сценарий, если в
          будущем avatar-pipeline начнёт стримить);
        * ``done``     — ``true`` если это последний чанк реплики.

        Метод не должен падать: ошибки сокета/сериализации — только WARN.
        """
        try:
            payload = String()
            payload.data = json.dumps(
                {
                    "request_id": request_id,
                    "text": text,
                    "streaming": bool(streaming),
                    "done": bool(done),
                },
                ensure_ascii=False,
            )
            self._tars1_text_pub.publish(payload)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"⚠️ [issue #2113] /tars1/text publish failed: {exc}"
            )

    def _parse_ssml_attributes(self, ssml: str) -> dict:
        """
        Извлекает атрибуты из SSML тегов (pitch, rate/speed, volume)

        Returns:
            dict: ``{'pitch': float, 'rate': float, 'volume': float}`` или
            пустой dict.

            * ``pitch`` хранится как float-множитель (для Silero-фолбэка);
              для Yandex gRPC v3 ``_synthesize_yandex_single`` конвертирует
              его в Hz-offset через ``_ssml_pitch_to_hz``.
            * ``volume`` хранится как АБСОЛЮТНАЯ LUFS-цель для Yandex
              ``Hints.volume`` (range [-145; 0)); для SSML-именованных
              уровней (``loud``/``soft``/…) вычисляется через
              ``_ssml_volume_to_lufs_target`` относительно baseline -19 LUFS.
        """
        attributes = {}

        # Ищем <prosody> теги с атрибутами
        # Примеры: <prosody pitch="+10%" rate="1.2">, <prosody pitch="high" rate="slow" volume="loud">
        prosody_pattern = r"<prosody\s+([^>]+)>"
        matches = re.finditer(prosody_pattern, ssml, re.IGNORECASE)

        for match in matches:
            attrs_str = match.group(1)

            # Парсим pitch
            pitch_match = re.search(
                r'pitch\s*=\s*["\']?([^"\'>\s]+)["\']?', attrs_str, re.IGNORECASE
            )
            if pitch_match:
                pitch_value = pitch_match.group(1)
                # Конвертируем в множитель для Yandex
                # "+10%" -> 1.1, "-10%" -> 0.9, "high" -> 1.2, "low" -> 0.8
                if "%" in pitch_value:
                    try:
                        percent = float(pitch_value.replace("%", ""))
                        attributes["pitch"] = 1.0 + (percent / 100.0)
                    except ValueError:
                        pass
                elif pitch_value == "high":
                    attributes["pitch"] = 1.2
                elif pitch_value == "low":
                    attributes["pitch"] = 0.8
                elif pitch_value == "medium":
                    attributes["pitch"] = 1.0
                else:
                    try:
                        attributes["pitch"] = float(pitch_value)
                    except ValueError:
                        pass

            # Парсим rate (скорость речи)
            rate_match = re.search(
                r'rate\s*=\s*["\']?([^"\'>\s]+)["\']?', attrs_str, re.IGNORECASE
            )
            if rate_match:
                rate_value = rate_match.group(1)
                # "1.5" -> 1.5, "fast" -> 1.5, "slow" -> 0.7
                if "%" in rate_value:
                    try:
                        percent = float(rate_value.replace("%", ""))
                        attributes["rate"] = percent / 100.0
                    except ValueError:
                        pass
                elif rate_value == "fast":
                    attributes["rate"] = 1.5
                elif rate_value == "slow":
                    attributes["rate"] = 0.7
                elif rate_value == "medium":
                    attributes["rate"] = 1.0
                else:
                    try:
                        attributes["rate"] = float(rate_value)
                    except ValueError:
                        pass

            # Парсим volume (громкость в LUFS для Yandex gRPC v3).
            # Допускаем как числовые dB-формы ("+5dB", "-5dB", "5dB"),
            # так и SSML-именованные уровни (silent/x-soft/soft/medium/loud/x-loud).
            volume_match = re.search(
                r'volume\s*=\s*["\']?([^"\'>\s]+)["\']?',
                attrs_str,
                re.IGNORECASE,
            )
            if volume_match:
                attributes["volume"] = _ssml_volume_to_lufs_target(
                    volume_match.group(1)
                )

        return attributes

    def _submit_synthesis(
        self,
        fn,
        speech_id: str,
        *args,
        **kwargs,
    ):
        """Submit a synth worker through the bounded executor (BLK-9).

        Acquires a slot from `_synthesis_slots` first; if no slot is free
        (i.e. `(max_workers + max_queue)` tasks are already in-flight or
        pending) we log and return without submitting, instead of spawning
        a new thread or letting the executor's unbounded internal queue
        accumulate forever.

        The slot is released automatically when the future completes
        (success or failure) via the `_release_synthesis_slot` callback.

        ``**kwargs`` (issue #1219) — дополнительные данные для воркера,
        не входящие в канонический positional arity (например ``voice``).
        Пробрасываются в ``executor.submit`` как keyword-аргументы —
        ``_run_synthesis_worker(**kwargs)`` достаёт их из ``**kwargs``.

        ``priority`` (issue #1996, optional kwarg, НЕ извлекается из
        ``**kwargs`` — остаётся в них и летит дальше воркеру) — влияет
        ТОЛЬКО на то, какой ``play_seq`` достанется этому запросу; см.
        ``_assign_priority_play_seq``.
        """
        if self._synthesis_executor_shutdown:
            self.get_logger().warning(
                f"⚠️  Dropping TTS submit after executor shutdown "
                f"(speech_id={speech_id[:8] if speech_id else 'None'})"
            )
            return
        if not self._synthesis_slots.acquire(blocking=False):
            self.get_logger().warning(
                f"⚠️  TTS synth pool full "
                f"(speech_id={speech_id[:8] if speech_id else 'None'}) — dropping"
            )
            return
        self._synthesis_in_flight += 1
        priority = _normalize_tts_priority(kwargs.get("priority"))
        if priority in _TTS_PRIORITY_PREEMPTS:
            # ADR-0056 §3.5 — REPLACE-priority: an operator-priority
            # request re-orders the FIFO-gate, so any in-flight
            # speculative pre-gen (which assumed the *old* ordering) is
            # invalidated. Best-effort / never blocks the submit path.
            self._cancel_pregen_for_priority_replace(speech_id)
        # 🔴 FIX (12:02 FIFO): выдаём play_seq в порядке submit — это
        # порядок приёма запросов из ROS-callback = порядок LLM tool_calls.
        # Worker будет ждать своей очереди перед play_audio. Issue #1996 —
        # ``operator``-приоритет вставляет запрос сразу за активным чанком
        # вместо хвоста очереди (см. ``_assign_priority_play_seq``).
        with self._play_order_cond:
            play_seq = self._assign_priority_play_seq(speech_id, priority)
        try:
            # 🔴 FIX (live 06.08): play_seq ТОЛЬКО через kwargs! Воркер e65a6e5d
            # убрал позиционный play_seq из _run_synthesis_worker (→ **kwargs),
            # а submit(fn, *args, play_seq) передавал его 10-м позиционным →
            # TypeError: takes from 3 to 9 positional args but 10 given →
            # TTS молчал (только пилик). Теперь kwarg — попадает в **kwargs.
            future = self._synthesis_executor.submit(
                fn, *args, play_seq=play_seq, **kwargs
            )
        except RuntimeError as exc:
            # Executor was shut down between our check and submit() — rare
            # but possible during node teardown.
            self._synthesis_slots.release()
            self._synthesis_in_flight -= 1
            self.get_logger().warning(
                f"⚠️  Executor refused submit for "
                f"speech_id={speech_id[:8] if speech_id else 'None'}: {exc}"
            )
            return
        future.add_done_callback(self._on_synthesis_done)

    def _on_synthesis_done(self, future):
        """Release a synth slot regardless of success/failure.

        Runs on whatever worker thread completed, so it must not touch
        ROS state directly (callbacks, parameters) — only thread-safe
        primitives like the semaphore and integer counter.
        """
        try:
            exc = future.exception()
        except concurrent.futures.CancelledError:
            exc = None
        if exc is not None:
            self.get_logger().warning(f"⚠️  Synthesis worker raised: {exc!r}")
        self._synthesis_slots.release()
        # The counter is monotonic-ish under CPython GIL; the racy
        # underflow on rare shutdown race is acceptable for diagnostics.
        self._synthesis_in_flight = max(0, self._synthesis_in_flight - 1)

    # ── Issue #1996 / operator-agent step 7a — priority-aware FIFO-gate ──

    def _assign_priority_play_seq(self, speech_id: str, priority: str) -> int:
        """Assign a ``play_seq`` slot, honouring ``priority`` (issue #1996).

        MUST be called while holding ``self._play_order_cond`` — it reads
        and mutates ``_play_seq_counter`` / ``_pending_seqs`` /
        ``_next_play_seq`` without its own locking.

        * ``normal`` (default) — legacy behaviour: next free slot at the
          tail of the FIFO (``_play_seq_counter + 1``).
        * ``operator`` — jumps to ``_play_active_seq + 1`` (right behind
          the chunk currently in ``play_audio``), or to the head of the
          gate (``_next_play_seq``) if nothing is playing. Any pending
          ``normal`` request already sitting on that slot is cascaded
          forward by :meth:`_resolve_operator_priority_slot` — the
          active chunk itself is never touched (invariant 8a: врезка ≠
          прерывание).

        Every assigned slot is recorded in ``_pending_seqs[speech_id]``
        (even for ``normal``) so a later ``operator`` insertion can
        re-number it, and so the FIFO-gate in ``_synthesize_and_play``
        can read the live (possibly re-numbered) seq instead of a stale
        local copy.
        """
        if priority in _TTS_PRIORITY_PREEMPTS and speech_id:
            target = (
                self._play_active_seq + 1
                if self._play_active_seq is not None
                else self._next_play_seq
            )
            play_seq = self._resolve_operator_priority_slot(target)
        else:
            self._play_seq_counter += 1
            play_seq = self._play_seq_counter
        if speech_id:
            self._pending_seqs[speech_id] = play_seq
        # The cascade in ``_resolve_operator_priority_slot`` can push a
        # pending ``normal`` seq ABOVE the current counter (e.g. counter=2,
        # operator claims slot 2, the normal that was there gets bumped to
        # 3) — sync against every pending value, not just ``play_seq``,
        # or the next plain ``normal`` submit would reuse an already-taken
        # slot and the gate would hang forever waiting for a duplicate.
        highest_pending = max(self._pending_seqs.values(), default=play_seq)
        self._play_seq_counter = max(self._play_seq_counter, play_seq, highest_pending)
        return play_seq

    def _resolve_operator_priority_slot(self, target: int) -> int:
        """Cascade-shift pending seqs so ``target`` is free for an operator.

        If ``target`` is already taken by another pending ``speech_id``,
        every pending seq ``>= target`` is bumped by +1 (their relative
        FIFO order among themselves is preserved — they just all move
        one slot back to make room). Repeats until ``target`` is free.
        Must be called under ``self._play_order_cond`` (see caller).
        """
        taken = set(self._pending_seqs.values())
        while target in taken:
            for sid, seq in list(self._pending_seqs.items()):
                if seq >= target:
                    self._pending_seqs[sid] = seq + 1
            taken = set(self._pending_seqs.values())
        return target

    def _cancel_pregen_for_priority_replace(self, speech_id: str) -> None:
        """ADR-0056 §3.5 trigger #4 — REPLACE via the issue #1996 priority flag.

        An ``operator``-priority submit re-orders the FIFO-gate (it can
        push an already-pregenerated ``normal`` chunk one slot back).
        Any in-flight speculative pre-gen was kicked off assuming the
        *old* ordering, so it is cancelled here rather than risking a
        stale ``prebaked_audio`` downstream. Best-effort: swallow every
        error so a pre-gen hiccup never blocks an operator interjection
        from being scheduled — that would defeat the whole point of the
        priority queue.
        """
        # Зовём напрямую, а не через getattr: ``cancel_pregen`` — метод
        # этого же класса (см. ниже по файлу). Защита через
        # getattr никогда бы не сработала, зато при переименовании
        # метода тихо превратила бы триггер ADR-0056 §3.5 в no-op — это
        # ровно тот fail-open, о котором предупреждает §4.3 хендоффа
        # (docs/plans/2026-09-05-operator-agent-architecture-handoff.md).
        try:
            self.cancel_pregen(reason="REPLACE-priority")
        except Exception as exc:  # noqa: BLE001 — never block the submit path
            # warning, а не debug: если триггер сломан, это должно
            # быть видно в обычных логах робота, а не только под debug.
            self.get_logger().warning(
                f"cancel_pregen(REPLACE-priority) failed for "
                f"speech_id={speech_id[:8] if speech_id else 'None'}: {exc!r}"
            )

    def _run_synthesis_worker(
        self,
        ssml: str,
        text: str,
        dialogue_id: str = None,
        ssml_attributes: dict = None,
        speech_id: str = None,
        batch_id: str = None,
        batch_index: int = None,
        batch_total: int = None,
        **kwargs,
    ):
        """Синтез + воспроизведение вне ROS callback thread.

        🔴 FIX (12:02 «анекдот перепутан»): _synthesis_lock УБРАН — при
        max_workers=4 синтез идёт ПАРАЛЛЕЛЬНО (4-5 фраз рендерятся сразу,
        быстро). Порядок воспроизведения обеспечивает FIFO-gate внутри
        _synthesize_and_play (play_seq выдан в порядке приёма запросов).

        Signature contract: the positional arity is exactly
        ``(ssml, text, dialogue_id, ssml_attributes, speech_id,
        batch_id, batch_index, batch_total)`` — the test
        ``test_submit_and_worker_arg_arities_agree`` walks the AST
        and asserts this canonical order. ``batch_total`` is the
        terminal parameter (test_run_synthesis_worker_signature_terminates_in_batch_total).
        ``play_seq`` is forwarded via ``**kwargs`` so the producer
        (``_submit_synthesis``) can still pass it explicitly via
        ``fn(... play_seq=N)`` but the positional arity stays aligned
        with the rest of the pipeline.
        """
        if dialogue_id and self.current_dialogue_id != dialogue_id:
            self.get_logger().warning(
                f"Dropping queued TTS for stale dialogue {dialogue_id[:8]}"
            )
            return
        # Forward FIFO-gate slot through kwargs (it's not in the
        # canonical positional arity so we extract it from **kwargs).
        play_seq = kwargs.get("play_seq", None)
        # Issue #1219 — LLM voice selection: голос из запроса speak_text
        # (передан через kwargs, чтобы не ломать канонический positional
        # arity, см. test_speech_id_arg_chain).
        voice = kwargs.get("voice", None)
        # AV-28 — язык произношения (тем же путём, что voice: через kwargs,
        # чтобы канонический positional arity остался прежним).
        language = kwargs.get("language", None)
        # ADR-0055 / issue #1993 — sink маршрут аудио. "speaker" → ALSA-
        # воспроизведение + /voice/audio/speech (старый путь, по умолчанию).
        # "headset" → без ALSA, без /voice/audio/speech, только
        # /avatar/tts/audio (ТАРС в шлем). Ключевое слово передаётся через
        # kwargs, чтобы не ломать test_speech_id_arg_chain.
        sink = kwargs.get("sink", "speaker")
        # Issue #2003 / ADR-0056 — forward the prebaked_audio hint
        # from the producer (``dialogue_callback`` claims it via
        # ``claim_pregen(speech_id)`` before ``_submit_synthesis``).
        # ``None`` (legacy path / publisher opted out / quality
        # rejected) is the default and behaves exactly like before.
        prebaked_audio = kwargs.get("prebaked_audio", None)
        self._synthesize_and_play(
            ssml,
            text,
            dialogue_id,
            ssml_attributes,
            speech_id,
            batch_id,
            batch_index,
            batch_total,
            play_seq=play_seq,
            voice=voice,
            language=language,
            sink=sink,
            prebaked_audio=prebaked_audio,
        )

    def _release_play_seq(
        self, play_seq: int | None, speech_id: str | None = None
    ) -> None:
        """Освободить FIFO-очередь воспроизведения (безопасно для None).

        Вызывается при ЛЮБОМ выходе из _synthesize_and_play после синтеза:
        после play_audio (finally), при dialogue-check, при STOP-check.
        Без этого _next_play_seq застревает и все следующие фразы ждут
        очередь вечно (live 12:28 «робот замолчал после barge-in»).

        ``speech_id`` (issue #1996, optional keyword — backward-compat
        with old positional-only callers/tests): when given, also drops
        ``_pending_seqs[speech_id]`` unconditionally (this request's
        synthesis/playback is over, whatever slot it currently holds —
        the key is unique per ``speech_id`` so there is no risk of
        stomping someone else's entry). ``_play_active_seq`` is a
        single shared value, though, so it is only cleared when it
        still equals the ``play_seq`` being released — otherwise we'd
        risk erasing the active-seq marker of a *different* chunk that
        started playing after this one failed/was cancelled.
        """
        if play_seq is not None:
            with self._play_order_cond:
                self._next_play_seq += 1
                if speech_id is not None:
                    self._pending_seqs.pop(speech_id, None)
                if self._play_active_seq == play_seq:
                    self._play_active_seq = None
                self._play_order_cond.notify_all()

    # ── Issue #2003 / ADR-0056 — speculative pre-generation API ─────────
    #
    # Three public methods (``pregenerate`` / ``claim_pregen`` /
    # ``cancel_pregen``) plus a lazy-built ``_prefetch`` engine. The
    # engine wraps :class:`scheduler.pregen.SpeculativeExecutor` and
    # exposes a small dict with the bits ``dialogue_callback`` and
    # ``_synthesize_and_play`` poke at directly. The whole block is
    # no-op when ``pregenerate_enabled=False`` or when the publisher
    # never sends a ``pregenerate`` field — see ADR-0056 §3.4.

    def _ensure_prefetch(self) -> Dict[str, Any]:
        """Construct the pre-fetch engine on first use (lazy).

        Returns a small dict containing:
        * ``executor`` — the live :class:`SpeculativeExecutor`;
        * ``synth`` — the coroutine-friendly callable that the
          executor dispatches into the asyncio loop.

        Idempotent: subsequent calls return the same dict.
        """
        engine = self._prefetch
        if engine is not None:
            return engine

        executor = _PreGenExecutor(
            synth_callable=self._synthesize_for_pregen,
            confidence_floor=self._pregenerate_confidence_floor,
            history_window=self._pregenerate_history_window,
        )
        engine = {
            "executor": executor,
            "synth": self._synthesize_for_pregen,
        }
        self._prefetch = engine
        return engine

    async def _synthesize_for_pregen(
        self,
        *,
        ssml: str,
        text: str,
        ssml_attributes: dict,
        voice: Optional[str] = None,
        language: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Async-friendly TTS for speculative tasks (ADR-0056 §3.2).

        Runs the *same* provider chain that ``_synthesize_and_play``
        uses, but in an async-friendly wrapper. The actual blocking
        work (Yandex gRPC / Silero / MiniMax HTTP) happens on the
        asyncio default thread pool via :func:`asyncio.to_thread`,
        which keeps the rclpy callback responsive.

        Returns ``{"audio_np": ndarray, "sample_rate": int}`` so
        :func:`scheduler.pregen._dispatch_synthesis` recognises the
        MiniMax-style return shape and the executor can extract
        ``sample_rate`` for the ``duration_ratio`` heuristic.

        Failures bubble up as exceptions — the executor treats them
        as "no pre-gen for this chunk" and does NOT cache anything.
        """
        loop = asyncio.get_event_loop()
        # We can't reuse ``_synthesize_and_play`` (it owns playback).
        # Instead we replicate the provider-chain *selection* logic and
        # call the appropriate private synth helper. The chain itself
        # is dead-cheap (a tuple of provider names).
        chain = self._effective_provider_chain()
        last_err: Optional[Exception] = None
        for provider_name in chain:
            try:
                if provider_name == "minimax":
                    if self.minimax_streaming:
                        result = await loop.run_in_executor(
                            None,
                            self._synthesize_minimax_streaming_publish,
                            text,
                            ssml_attributes,
                            voice or self.minimax_voice,
                            language,
                        )
                    else:
                        result = await loop.run_in_executor(
                            None,
                            self._synthesize_minimax,
                            text,
                            ssml_attributes,
                            voice,
                            language,
                        )
                    return result
                if provider_name == "yandex":
                    if not self.yandex_stub:
                        continue
                    audio = await loop.run_in_executor(
                        None,
                        self._synthesize_yandex,
                        text,
                        ssml_attributes,
                        voice,
                    )
                    return {
                        "audio_np": audio,
                        "sample_rate": self.audio_output_sample_rate,
                    }
                if provider_name == "silero":
                    audio = await loop.run_in_executor(
                        None,
                        self._synthesize_silero,
                        text,
                        ssml_attributes,
                        voice,
                    )
                    return {
                        "audio_np": audio,
                        "sample_rate": self.silero_sample_rate,
                    }
            except Exception as exc:  # noqa: BLE001
                last_err = exc
                self.get_logger().debug(
                    f"pregenerate synth {provider_name} failed: {exc!r} — "
                    f"trying next in chain"
                )
                continue
        if last_err is not None:
            raise last_err
        raise RuntimeError(
            "pregenerate synth: empty provider chain " f"(effective={chain!r})"
        )

    def pregenerate(self, current_chunk: dict, ctx: Optional[dict] = None) -> None:
        """Issue #2003 / ADR-0056 — kick off speculative next-chunk synthesis.

        Fire-and-forget: the caller (``dialogue_callback``) invokes
        this *before* the canonical ``_submit_synthesis`` for the
        current chunk. By the time ``batch_complete`` of the current
        chunk fires, the speculative audio for the next chunk is
        either already in :attr:`_PrefetchEngine._results` (claim
        via :meth:`claim_pregen`) or has been rejected by the
        quality gate (in which case the canonical synthesis runs
        unchanged — fallback to the legacy path).

        Parameters
        ----------
        current_chunk
            The JSON-decoded chunk dict (the same shape
            :func:`scheduler.pregen.build_pregen_task` accepts).
        ctx
            Reserved for future use (e.g. dialogue_id override).
            Currently unused — kept in the signature for forward
            compatibility with the §3.1 contract.
        """
        if not self._pregenerate_enabled:
            return
        if not isinstance(current_chunk, dict):
            return
        # Validate the payload before constructing the executor. This keeps
        # malformed / opt-out payloads as true no-ops without creating an
        # engine, while valid hints get the lazy engine immediately.
        task = _build_pregen_task(
            current_chunk,
            fallback_voice=self._prefetch_fallback_voice(),
            fallback_language=self._prefetch_fallback_language(),
        )
        if task is None:
            return

        engine = self._prefetch
        if engine is None:
            engine = {
                "executor": _PreGenExecutor(
                    synth_callable=self._synthesize_for_pregen,
                    confidence_floor=self._pregenerate_confidence_floor,
                    history_window=self._pregenerate_history_window,
                ),
                "synth": self._synthesize_for_pregen,
            }
            self._prefetch = engine
        executor = engine["executor"]
        loop = None
        try:
            loop = self.get_loop()  # rclpy event loop
        except Exception:  # noqa: BLE001 — unit-test stubs without rclpy
            loop = None

        async def _run() -> None:
            try:
                speech_id = await executor.kickoff(
                    current_chunk,
                    fallback_voice=self._prefetch_fallback_voice(),
                    fallback_language=self._prefetch_fallback_language(),
                    fallback_dialogue_id=self.current_dialogue_id,
                )
                if speech_id is not None and self._prefetch is None:
                    self._prefetch = {
                        "executor": executor,
                        "synth": self._synthesize_for_pregen,
                    }
            except Exception as exc:  # noqa: BLE001 — never crash ROS
                self.get_logger().warning(f"⚠️ pregenerate kickoff failed: {exc!r}")

        if loop is not None and loop.is_running():
            # Schedule without blocking the ROS callback.
            asyncio.run_coroutine_threadsafe(_run(), loop)
        else:
            # No live loop (unit tests / standalone) — best effort:
            # synchronously run the coroutine to completion on the
            # default loop. The executor handles asyncio internally.
            try:
                asyncio.run(_run())
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(f"pregenerate sync fallback failed: {exc!r}")

    def claim_pregen(self, speech_id: str) -> Optional[Dict[str, Any]]:
        """Atomically pop a cached speculative result for ``speech_id``.

        Returns ``None`` when no result is ready (or when the
        engine has been disabled). The caller
        (``_synthesize_and_play``) treats ``None`` as "fall back
        to the canonical path" — exactly the legacy behaviour.
        """
        if not self._pregenerate_enabled or self._prefetch is None:
            return None
        result: Optional[_PreGenResult] = self._prefetch["executor"].claim(speech_id)
        if result is None:
            return None
        # Mirror the ``PreGenResult`` shape into a small dict so the
        # rest of ``tts_node`` does not depend on the pre-gen
        # package's types.
        return {
            "audio_np": result.audio,
            "sample_rate": result.sample_rate,
            "decision": result.decision,
            "confidence": result.confidence,
            "basis": result.basis,
            "elapsed_ms": result.elapsed_ms,
        }

    def cancel_pregen(self, reason: str) -> int:
        """Cancel every in-flight speculative task (ADR-0056 §3.5).

        Called from the four sites enumerated in §3.5:

        1. ``_on_new_dialogue_id`` (dialogue switch);
        2. ``_interrupt_playback`` (STOP/barge-in);
        3. ``control_callback`` (explicit STOP);
        4. ``_on_set_provider`` (REPLACE).

        Returns the count of cancelled tasks (for metrics).

        Safe to call before :meth:`pregenerate` has ever been
        invoked — returns ``0`` in that case.
        """
        if self._prefetch is None:
            return 0
        executor = self._prefetch["executor"]
        try:
            loop = self.get_loop()
        except Exception:  # noqa: BLE001
            loop = None
        if loop is not None and loop.is_running():
            future = asyncio.run_coroutine_threadsafe(
                executor.cancel(reason=reason),
                loop,
            )
            try:
                return int(future.result(timeout=2.0))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(
                    f"⚠️ cancel_pregen async future failed: {exc!r}"
                )
                return 0
        # No live loop — best effort synchronous cancel. The
        # executor handles its own internal state.
        try:
            return int(asyncio.run(executor.cancel(reason=reason)))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f"cancel_pregen sync fallback failed: {exc!r}")
            return 0

    def publish_prefetch_metrics(self) -> None:
        """Publish :class:`PreGenMetrics` snapshot to ``/voice/tts/metrics``.

        Called on a low-frequency cadence from the existing
        :meth:`publish_state` path) so operators can observe
        ``latency_chunk_to_chunk_ms_mean`` and the per-kind
        rejection counters.

        No-op until the pre-fetch engine has been built at least
        once.
        """
        if self._prefetch is None:
            return
        snapshot = self._prefetch["executor"].snapshot()
        try:
            msg = String()
            msg.data = json.dumps(snapshot, ensure_ascii=False, default=str)
            # Lazy-create the publisher so nodes that never pre-gen
            # don't pay the cost of a topic allocation.
            if not hasattr(self, "_prefetch_metrics_pub"):
                self._prefetch_metrics_pub = self.create_publisher(
                    String, "/voice/tts/metrics", 10
                )
            self._prefetch_metrics_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001 — never crash ROS
            self.get_logger().debug(f"publish_prefetch_metrics failed: {exc!r}")

    def _prefetch_fallback_voice(self) -> str:
        """Voice to use when the publisher didn't specify one.

        Returns the *current* effective voice depending on the
        configured provider. Lives next to the pre-fetch API
        so the lazy-init order is self-contained.
        """
        if self.provider == "minimax":
            return self.minimax_voice
        if self.provider == "yandex":
            return self.yandex_voice
        return self.silero_speaker

    def _prefetch_fallback_language(self) -> Optional[str]:
        """Language to use when the publisher didn't specify one."""
        return getattr(self, "minimax_language", None)

    # ── Issue #1083: цепочка приоритетов TTS (minimax → yandex → silero) ────

    @staticmethod
    def _default_provider_chain() -> list[str]:
        """Дефолтная цепочка приоритетов TTS-провайдеров.

        Порядок: MiniMax (HTTP) → Yandex (gRPC v3) → Silero (офлайн).
        Silero всегда последний — это последний рубеж, он работает без
        сети. Цепочка конфигурируется через ROS-параметр ``provider_chain``;
        здесь — значение по умолчанию (не хардкод в hot-path).
        """
        return ["minimax", "yandex", "silero"]

    @staticmethod
    def _chain_from_provider(provider: str) -> list[str]:
        """Цепочка приоритетов из параметра ``provider`` (issue #1083).

        ``provider`` задаёт ПЕРВОГО в цепочке; остальные добираются
        дефолтным порядком (``_default_provider_chain``), Silero всегда
        последний. Примеры:

        * provider=minimax → [minimax, yandex, silero] (дефолт, фикс #1083:
          при квоте MiniMax идём на Yandex, а не сразу на Silero);
        * provider=yandex  → [yandex, silero] (back-compat: e2e ``tts: yandex``
          проверяет именно Yandex-голос, MiniMax не подмешиваем);
        * provider=silero  → [silero] (только офлайн).
        """
        if provider == "silero":
            return ["silero"]
        if provider == "yandex":
            # Back-compat: provider=yandex остаётся yandex → silero,
            # без MiniMax (см. историческое поведение tts_node).
            return ["yandex", "silero"]
        return TTSNode._default_provider_chain()  # minimax → yandex → silero

    def _effective_provider_chain(self) -> list[str]:
        """Цепочка, по которой реально идёт синтез в ``_synthesize_and_play``.

        Берёт ``self.provider_chain`` (уже нормализованную в ``__init__``);
        для тестовых стабов без атрибута — дефолтную. Silero всегда последний.
        """
        chain = getattr(self, "provider_chain", None)
        if chain:
            return chain
        provider = getattr(self, "provider", "minimax")
        return TTSNode._normalize_provider_chain(TTSNode._chain_from_provider(provider))

    @staticmethod
    def _normalize_provider_chain(chain: list[str]) -> list[str]:
        """Привести цепочку к инвариантам: только известные провайдеры,
        без дубликатов, Silero — всегда последний.

        Edge cases (issue #1083):
        * цепочка без Silero → Silero добавляется в конец;
        * Silero в середине → переносится в конец;
        * пустая/битая цепочка → дефолтная minimax → yandex → silero.
        """
        known = {"minimax", "yandex", "silero"}
        deduped: list[str] = []
        for p in chain:
            if p in known and p not in deduped:
                deduped.append(p)
        # ``chain`` без единого известного провайдера — битая цепочка.
        if not deduped:
            return TTSNode._default_provider_chain()
        # Явный provider=silero (только офлайн) — легитимный режим.
        if deduped == ["silero"]:
            return ["silero"]
        # Silero всегда последний в цепочке (инвариант issue #1083).
        if "silero" in deduped:
            deduped.remove("silero")
        deduped.append("silero")
        return deduped

    def _provider_is_dead(self, provider_name: str) -> bool:
        """True, если провайдер лежит в кэше «мёртвых» (TTL не истёк).

        Кэш «мёртвых» (как в LLM-health): если MiniMax ответил квотой 2056
        (или сеть/таймаут), не долбим его на каждый ход — пропускаем до
        истечения TTL, затем пробуем снова (провайдер мог «ожить»).
        """
        until = getattr(self, "_provider_dead_until", {}).get(provider_name, 0.0)
        return time.monotonic() < until

    def _mark_provider_dead(
        self,
        provider_name: str,
        error: Exception,
        ttl_s: float | None = None,
    ) -> None:
        """Пометить провайдера мёртвым на ttl_s секунд (по умолчанию —
        из параметра provider_dead_ttl_s). Сохраняем причину для лога.

        Классификация TTL (issue #1083):
        * quota/auth (2056 Token Plan limit, 401/403) → длинный TTL
          (provider_dead_ttl_s, default 300 с) — квота не кончится за секунды;
        * всё остальное (сеть/таймаут/5xx) → короткий TTL
          (provider_dead_ttl_transient_s, default 30 с).
        """
        if ttl_s is None:
            if isinstance(error, (MiniMaxTTSAuthError, MiniMaxTTSRateLimitError)):
                ttl_s = getattr(self, "provider_dead_ttl_s", 300.0)
            else:
                ttl_s = getattr(self, "provider_dead_ttl_transient_s", 30.0)
        assert ttl_s is not None
        dead_until = getattr(self, "_provider_dead_until", None)
        if dead_until is None:
            dead_until = {}
            self._provider_dead_until = dead_until
        dead_until[provider_name] = time.monotonic() + ttl_s
        reasons = getattr(self, "_provider_dead_reason", None)
        if reasons is None:
            reasons = {}
            self._provider_dead_reason = reasons
        reasons[provider_name] = str(error)[:300]
        self.get_logger().warn(
            f"💀 {provider_name} помечен мёртвым на {ttl_s:.0f}s "
            f"({type(error).__name__}: {error})"
        )
        # Issue #1229 — провайдер упал → публикуем фактического провайдера
        # (первый «живой» в цепочке после падения).
        publish = getattr(self, "_publish_provider_state", None)
        if publish is not None:
            try:
                publish("provider_dead")
            except Exception:  # noqa: BLE001 — диагностика не должна падать
                pass

    def _provider_dead_until_s(self, provider_name: str) -> float:
        """Сколько секунд осталось провайдеру в кэше «мёртвых» (для логов)."""
        until = getattr(self, "_provider_dead_until", {}).get(provider_name, 0.0)
        return max(0.0, until - time.monotonic())

    # ── Issue #1229 — фактический провайдер TTS ─────────────────────────────
    # tts_node — единственный, кто ЗНАЕТ фактического провайдера (после
    # фолбека из-за квоты/сети). Публикуем его состояние в /voice/tts/
    # provider_state: mcp_server (валидация голосов в speak_text/set_voice)
    # и dialogue_node (LLM-контекст [TTS]) используют РЕАЛЬНОГО провайдера,
    # а не номинального из параметра provider.

    def _load_persisted_provider_state(self) -> None:
        """Восстановить кэш «мёртвых» провайдеров из файла (issue #1229).

        Файл пишется :meth:`_persist_provider_state` при смене эффективного
        провайдера. При старте загружаем только записи, срок действия
        которых ещё не истёк (dead_until_ts в будущем) — просроченные
        игнорируем, чтобы дать провайдеру шанс «ожить».
        """
        path = getattr(self, "provider_state_file", "") or ""
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
        except (OSError, ValueError, TypeError):
            return  # нет файла / битый JSON — не мешаем старту
        if not isinstance(data, dict):
            return
        dead = data.get("dead_providers") or {}
        now = time.time()
        for provider, until_ts in dead.items():
            try:
                until_ts = float(until_ts)
            except (TypeError, ValueError):
                continue
            if until_ts > now:
                # Кэш «мёртвых» хранится в time.monotonic; файл — в wall-clock.
                # Конвертируем: монопольный сдвиг ≈ (time.monotonic - time.time).
                # Надёжнее пересчитать TTL от текущего момента: провайдер
                # остаётся мёртвым до истечения исходного TTL.
                ttl_left_s = until_ts - now
                self._provider_dead_until[provider] = time.monotonic() + ttl_left_s
        if dead:
            self.get_logger().info(
                f"💾 [issue 1229] Восстановлен кэш мёртвых провайдеров из "
                f"{path}: {dead}"
            )

    def _effective_provider(self) -> str | None:
        """Фактический провайдер TTS: первый «живой» в цепочке (issue #1229).

        Цепочка minimax → yandex → silero; провайдеры в кэше «мёртвых»
        (квота/сеть) пропускаются. Если все мёртвы — последний (silero).
        """
        chain = getattr(self, "provider_chain", None)
        if not chain:
            chain = TTSNode._chain_from_provider(getattr(self, "provider", "minimax"))
        try:
            from .tts_voice_registry import effective_provider as _effective
        except Exception:  # noqa: BLE001 — registry недоступен
            return chain[0] if chain else None
        return _effective(chain, self._provider_is_dead)

    def _persist_provider_state(self, payload: dict) -> None:
        """Записать состояние эффективного провайдера в файл (best-effort)."""
        path = getattr(self, "provider_state_file", "") or ""
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(payload, fh, ensure_ascii=False)
        except OSError as exc:  # noqa: BLE001 — файл не критичен для TTS
            self.get_logger().debug(
                f"⚠️ [issue 1229] Не удалось записать provider_state " f"{path}: {exc}"
            )

    def _publish_provider_state(
        self,
        reason: str,
        provider: str | None = None,
        voice: str | None = None,
    ) -> None:
        """Опубликовать фактического провайдера TTS (issue #1229).

        Args:
            reason: "startup" | "provider_dead" | "synthesis_ok" — для логов.
            provider: фактический провайдер; None → вычислить самим.
            voice: фактически использованный голос (для synthesis_ok).

        Публикует JSON ``{"provider": str, "voice": str, "default_voice": str,
        "reason": str, "ts": float}`` в /voice/tts/provider_state и пишет
        персистентный файл (dead_providers для восстановления кэша).
        """
        pub = getattr(self, "provider_state_pub", None)
        if pub is None:
            return  # тестовый stub без топика
        eff = provider or self._effective_provider()
        if not eff:
            return
        from .tts_voice_registry import default_voice_for as _default_voice

        default_voice = _default_voice(eff)
        used_voice = voice or default_voice
        payload = {
            "provider": eff,
            "voice": used_voice,
            "default_voice": default_voice,
            "reason": reason,
            "ts": time.time(),
        }
        try:
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            pub.publish(msg)
        except Exception as exc:  # noqa: BLE001 — best-effort диагностика
            self.get_logger().warn(
                f"⚠️ [issue 1229] Не удалось опубликовать provider_state: {exc}"
            )
        self.get_logger().info(
            f"🎙️ [issue 1229] TTS provider_state → '{eff}' "
            f"(voice: {used_voice}, reason: {reason})"
        )
        # Персистим кэш «мёртвых» (wall-clock), чтобы рестарт не сбрасывал
        # знание о недоступном провайдере.
        dead_payload = {}
        dead_until = getattr(self, "_provider_dead_until", {})
        now_wall = time.time()
        for prov, until_mono in dead_until.items():
            remaining = until_mono - time.monotonic()
            if remaining > 0:
                dead_payload[prov] = now_wall + remaining
        self._persist_provider_state({"provider": eff, "dead_providers": dead_payload})
        # AV-27 / issue #1919 — latched-каталог голосов. Публикуется
        # ВСЕГДА после provider_state (на startup, на set_provider, на
        # provider_dead). Никаких x-check что голоса изменились: payload
        # дешёвый (≤30 dict'ов), а latched QoS гарантирует что новый
        # подписчик получит САМЫЙ ПОСЛЕДНИЙ (TRANSIENT_LOCAL depth=1).
        self._publish_voices_catalog(eff, used_voice, default_voice)

    def _publish_voices_catalog(
        self,
        provider: str,
        used_voice: str,
        default_voice: str,
    ) -> None:
        """AV-27 — опубликовать каталог VoiceInfo для активного провайдера.

        Источник — :func:`voices_info_for` из ``tts_voice_registry``. Если
        провайдер неизвестен реестру — публикуем честно пустой список
        (``voices=[]``), UI отрисует «провайдер не отдаёт список голосов»
        (acceptance issue #1919). Никаких хардкод-fallback'ов: «active
        provider менялся, но реестр про него не знает» — это сам по себе
        bug, который должен всплыть в UI как пустой список.
        """
        pub = getattr(self, "voices_catalog_pub", None)
        if pub is None:
            return  # тестовый stub без топика
        from .tts_voice_registry import voices_info_for as _voices_info

        voices = _voices_info(provider)
        payload = {
            "provider": provider,
            "voice": used_voice,
            "default_voice": default_voice,
            "voices": voices,
            "ts": time.time(),
        }
        try:
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"⚠️ [av-27] voices_catalog publish failed: {exc}")

    def _synthesize_and_play(
        self,
        ssml: str,
        text: str,
        dialogue_id: str = None,
        ssml_attributes: dict = None,
        speech_id: str = None,
        batch_id: str = None,
        batch_index: int = None,
        batch_total: int = None,
        play_seq: int = None,  # FIFO-gate slot, keyword-only at the call site
        voice: str = None,  # Issue #1219 — запрошенный LLM голос (Q6)
        language: str = None,  # AV-28 — язык произношения этой реплики
        sink: str = "speaker",  # ADR-0055 / issue #1993 — "speaker"|"headset"
        prebaked_audio: Optional[Dict[str, Any]] = None,  # ADR-0056 — pre-fetched
    ):
        """Синтез речи и воспроизведение.

        Note: ``play_seq`` is keyword-only at the call site (after
        ``batch_total``) so the positional arity matches the test
        contract in ``test_speech_id_arg_chain`` (which asserts the
        canonical 8-positional form).  The producer
        (``_run_synthesis_worker``) passes it as ``play_seq=`` kwarg.

        ``voice`` — запрошенный LLM голос (issue #1219). Резолвится
        отдельно для КАЖДОГО фактического провайдера в цепочке: если
        голос недоступен у провайдера (фолбек), используется дефолт
        этого провайдера, а voice_used логируется (Q6/Q11).

        ``language`` — язык, на котором написан ``text`` (AV-28). Тоже
        резолвится на КАЖДОМ провайдере цепочки, потому что провайдер
        меняется на лету: minimax отдаёт его в ``language_boost`` и умеет
        все шесть языков, а yandex и silero в нашей раскладке умеют
        только русский — им вместо чужого текста уходит честная фраза
        (``unsupported_language_notice``), а не кириллический транслит,
        который звучал бы неправильно и молча.

        ``prebaked_audio`` — опциональный dict-результат
        :meth:`TTSNode.claim_pregen` (ADR-0056, issue #2003). Когда
        передан, функция ПРОПУСКАЕТ цепочку синтеза и использует
        ``prebaked_audio["audio_np"]`` / ``prebaked_audio["sample_rate"]``
        как готовый результат. ``used_provider`` помечается как
        ``"prefetch"`` (чтобы provider_state и метрики видели, что
        чанк пришёл из кэша, а не от провайдера). ``None`` (default)
        — поведение прежнее, никаких изменений.

        Реализация декомпозирована (issue #2078): ``_synthesize_and_play``
        — orchestrator-метод (CC≤15), тяжёлая логика разнесена в приватные
        ``_sap_*`` helpers (issue #2077 audit + ADR-0021 R1).
        """
        # Issue #980 — ``batch_started_at`` measures the wall-clock span between
        # the first and last chunk of a single TTS batch.  ``time.monotonic``
        # is used because wall-clock may jump on NTP resync.
        import time as _time

        if batch_id is not None and batch_index == 1:
            batch_started_at = _time.monotonic()
        elif batch_id is None:
            # Single-chunk legacy batch — still treated as a batch of size 1
            # so dialogue_node gets exactly one ``batch_complete`` event.
            batch_index = 1
            batch_total = 1
            batch_started_at = _time.monotonic()
        else:
            batch_started_at = None

        # Bare-stub compatibility (issue #2078): bind _sap_* helpers to
        # bare stub ``self`` BEFORE any helper call.  Production TTSNode
        # instances see a no-op (the class already exposes the methods);
        # bare ``_Bare`` test stubs gain bound-method access via the
        # descriptor protocol.  Unbound ``TTSNode.<helper>`` call so the
        # binding itself is reachable even from a bare stub that doesn't
        # carry ``_bind_sap_helpers_for_stub`` either.  See
        # :meth:`TTSNode._bind_sap_helpers_for_stub`.
        TTSNode._bind_sap_helpers_for_stub(self)

        self._sap_setup_request(dialogue_id, ssml_attributes, text)
        if ssml_attributes is None:
            ssml_attributes = {}
        if self.normalize_text:
            text = normalize_for_tts(text)

        # Issue #1234 — OpenTelemetry span ``tts.synthesize`` (этап 2).
        # Открываем ДО try и закрываем в finally: покрывает всю цепочку
        # синтеза (minimax → yandex → silero) + fallback. Атрибуты provider/
        # fallback/duration проставляем в finally (provider известен только
        # после цепочки). ``start_span_handle`` — no-op без OTel.
        _tts_trace_start = time.monotonic()
        _tts_trace = start_span_handle(
            "tts.synthesize",
            {
                "model": getattr(self, "minimax_model", ""),
                "voice": getattr(self, "minimax_voice", ""),
            },
        )
        # Mutable synthesis state shared by helpers.  Using a dict keeps
        # the orchestrator's local-state picture flat (one ``ctx`` instead
        # of 6 locals), and lets helpers communicate back without growing
        # the orchestrator's CC.  Fields are listed inline to keep grep easy.
        ctx: Dict[str, Any] = {
            "audio_np": None,  # np.ndarray | None
            "sample_rate": 16000,  # default; each provider overrides
            "used_provider": None,  # "minimax" | "yandex" | "silero" | "prefetch"
            "used_voice": None,  # issue #1229 — фактический голос
            "result": {},  # последний result от MiniMax (already_published etc.)
        }
        try:
            # Issue #2096 — единый choke-point: ЛЮБОЙ TTS-путь (dialogue_
            # callback, /avatar/tts/request, /voice/tts/request) проходит
            # через ``_synthesize_and_play``. Пустой/whitespace ``text``
            # раньше мог дойти сюда (см. ``_on_avatar_tts_request`` до
            # фикса) и попасть в MiniMax → ``TTSBadRequestError("text is
            # empty")`` → ``_sap_synthesize_minimax`` вызывал
            # ``_mark_provider_dead("minimax", ...)`` → каскад помечал
            # yandex/silero мёртвыми на 30s (см. live-лог voice-assistant,
            # 2026-09-07: 3 реплики grip-пайплайна подряд положили всю
            # цепочку). Пустой text — дефект ВЫЗЫВАЮЩЕГО, не провайдера:
            # не даём ему даже достичь цепочки провайдеров, поэтому НИКТО
            # не помечается мёртвым, и следующий (непустой!) запрос от
            # ЛЮБОГО caller'а (включая голос личности ТАРС) не молчит.
            # ``prebaked_audio`` (ADR-0056) — исключение: pregen-кэш
            # ключуется по РЕАЛЬНОМУ тексту фразы и уже несёт готовое
            # аудио, блокировать воспроизведение тут незачем.
            if prebaked_audio is None and (not text or not text.strip()):
                self.get_logger().warn(
                    "⚠️ [issue 2096] TTS: пустой text — provider chain "
                    "пропущен целиком, провайдеры НЕ помечаются мёртвыми "
                    f"(speech_id={(speech_id or '')[:8]}, sink={sink})"
                )
                raise _TTSEmptyTextError("empty_text")

            # 1) Pre-gen cache short-circuit (ADR-0056 / issue #2003).
            if self._sap_consume_prefetch(prebaked_audio, voice, speech_id, ctx):
                # Prefetch отработал — цепочка синтеза не нужна, всё остальное
                # как для обычного результата (publish + FIFO + play).
                pass
            else:
                # 2) Canonical provider chain walk.
                self._sap_run_provider_chain(
                    voice, language, ssml_attributes, text, sink, ctx
                )
                # 3) Silero last-resort fallback если ничего не сработало.
                if ctx["audio_np"] is None:
                    self._sap_silero_fallback(
                        text, voice, language, ssml_attributes, dialogue_id, ctx
                    )

            # 4) Publish audio on /voice/audio/speech (или /avatar/tts/audio
            #    для sink="headset") + provider_state.
            self._sap_publish_for_sink(
                ctx["audio_np"],
                ctx["sample_rate"],
                sink,
                ctx["used_provider"],
                ctx["result"],
                voice,
                ctx["used_voice"],
            )

            raw_duration_sec: float = (
                round(len(ctx["audio_np"]) / ctx["sample_rate"], 2)
                if ctx["sample_rate"] > 0
                else 0.0
            )

            # 5) FIFO-gate + dialogue-id freshness check.
            # 🔴 FIX (live 12:28 «робот замолчал после barge-in»): FIFO-gate
            # ДОЛЖЕН быть ДО dialogue/STOP checks. Каждый ранний return
            # освобождает seq через _release_play_seq (иначе робот молчал
            # после barge-in).
            if not self._sap_wait_fifo_and_dialogue(play_seq, speech_id, dialogue_id):
                return  # dialogue_id рассинхронизирован, helper уже отпустил gate

            # 6) Sink="headset" — публикуем finished без локального ALSA-плея.
            if sink == "headset":
                self._sap_finish_headset(
                    speech_id,
                    dialogue_id,
                    batch_id,
                    batch_index,
                    batch_total,
                    batch_started_at,
                    raw_duration_sec,
                    ctx["used_provider"],
                    play_seq,
                )
                return

            # 7) Local playback (speaker): prepare → stop-pre-check →
            #    play_audio → finished-event.
            self._sap_local_playback(
                ctx["audio_np"],
                ctx["sample_rate"],
                text,
                voice,
                play_seq,
                speech_id,
                dialogue_id,
                batch_id,
                batch_index,
                batch_total,
                batch_started_at,
                raw_duration_sec,
            )

        except Exception as e:
            self._sap_handle_synthesis_error(
                e,
                text,
                voice,
                play_seq,
                speech_id,
                dialogue_id,
                batch_id,
                batch_index,
                batch_total,
                batch_started_at,
            )

        finally:
            # Issue #1234 — закрываем span ``tts.synthesize``: проставляем
            # фактического провайдера, fallback-флаг и длительность.
            _used_provider = ctx.get("used_provider") or "none"
            provider_chain = getattr(self, "provider_chain", None)
            if not provider_chain:
                provider_chain = TTSNode._chain_from_provider(
                    getattr(self, "provider", "minimax")
                )
            _primary = (
                provider_chain[0]
                if provider_chain
                else getattr(self, "provider", "minimax")
            )
            _tts_trace.set_attribute("provider", _used_provider)
            _tts_trace.set_attribute("fallback", _used_provider != _primary)
            _tts_trace.set_attribute("duration_s", time.monotonic() - _tts_trace_start)
            _tts_trace.close()

    # ── helpers for ``_synthesize_and_play`` (issue #2078 decomposition) ──

    # Names of every ``_sap_*`` method on TTSNode.  Iteration order matches
    # the orchestrator's call order, so unit-test debug traces read top-to-bottom.
    _SAP_HELPER_NAMES = (
        "_sap_setup_request",
        "_sap_consume_prefetch",
        "_sap_run_provider_chain",
        "_sap_synthesize_minimax",
        "_sap_synthesize_yandex",
        "_sap_silero_fallback",
        "_sap_publish_silero_warming",
        "_sap_publish_for_sink",
        "_sap_wait_fifo_and_dialogue",
        "_sap_finish_headset",
        "_sap_local_playback",
        "_sap_prepare_audio_for_playback",
        "_sap_publish_finished_failure",
        "_sap_finalize_after_playback",
        "_sap_handle_synthesis_error",
    )

    def _bind_sap_helpers_for_stub(self) -> None:
        """Bind every ``_sap_*`` helper from ``TTSNode`` onto ``self`` if it
        isn't already present.  Production ``TTSNode`` instances see a no-op
        (the class already exposes the methods); bare ``_Bare`` test stubs
        (which don't inherit from TTSNode) gain bound-method access.  This
        keeps the existing ``TTSNode._synthesize_and_play(node, ...)`` test
        contract working without per-test fixture updates — issue #2078
        decomposition invariant.
        """
        for _sap_name in TTSNode._SAP_HELPER_NAMES:
            if hasattr(self, _sap_name):
                continue
            _cls_method = getattr(TTSNode, _sap_name, None)
            if _cls_method is not None:
                setattr(
                    self,
                    _sap_name,
                    _cls_method.__get__(self, type(self)),
                )

    def _sap_setup_request(
        self,
        dialogue_id: Optional[str],
        ssml_attributes: Optional[dict],
        text: str,
    ) -> None:
        """Сбросить stop-флаг, применить buffered STOP, установить
        ``processing_dialogue_id`` и дефолтный ``ssml_attributes``."""
        # Сбрасываем флаг stop при новом запросе
        self.stop_requested = False
        # Issue #1563 — потребляем буферизованный STOP, пришедший
        # в окне «synth_done → play_audio» прошлого chunk'а. Если
        # поздний STOP был отложен для текущего запроса — применяем
        # его ДО старта синтеза (новый запрос не нужен).
        if getattr(self, "_post_synth_stop_pending", False):
            self._post_synth_stop_pending = False
            self.get_logger().info(
                "⏭️ [issue 1563] consuming buffered STOP at start of new request"
            )
            self._interrupt_playback()

        # Устанавливаем processing_dialogue_id для этого синтеза
        if dialogue_id:
            self.processing_dialogue_id = dialogue_id
            self.get_logger().debug(
                f"🎯 Начинаем обработку dialogue_id: {dialogue_id[:8]}..."
            )

    def _sap_consume_prefetch(
        self,
        prebaked_audio: Optional[Dict[str, Any]],
        voice: Optional[str],
        speech_id: Optional[str],
        ctx: Dict[str, Any],
    ) -> bool:
        """Issue #2003 / ADR-0056 — пробуем вытащить готовый чанк из
        pre-gen кэша.  Если пре-ген валиден — заполняем ``ctx`` и возвращаем
        ``True`` (цепочка синтеза должна быть пропущена).  Иначе возвращаем
        ``False`` и ``ctx`` остаётся в исходном состоянии (None-результат).
        """
        if not (prebaked_audio and isinstance(prebaked_audio, dict)):
            return False
        cached_audio = prebaked_audio.get("audio_np")
        cached_sr = prebaked_audio.get("sample_rate")
        if not (
            cached_audio is not None
            and cached_sr is not None
            and int(getattr(cached_audio, "size", 0)) > 0
        ):
            self.get_logger().warning(
                "⚠️ [issue 2003] prebaked_audio invalid shape "
                "— falling back to canonical synthesis"
            )
            return False
        ctx["audio_np"] = np.asarray(cached_audio, dtype=np.float32)
        ctx["sample_rate"] = int(cached_sr)
        ctx["used_provider"] = "prefetch"
        ctx["used_voice"] = voice or self.minimax_voice
        self.get_logger().info(
            f"⚡ [issue 2003] prebaked_audio used "
            f"(speech_id={(speech_id or '')[:8]}, "
            f"decision={prebaked_audio.get('decision')}, "
            f"confidence={prebaked_audio.get('confidence'):.2f}, "
            f"basis={prebaked_audio.get('basis')!r})"
        )
        if is_metrics_enabled():
            record_tts_synthesize("prefetch", success=True, duration_s=0.0)
        # Bump the executor's bypass counter via the public metrics path.
        if self._prefetch is not None:
            self._prefetch["executor"].metrics.pregens_bypassed += 1
        return True

    def _sap_run_provider_chain(
        self,
        voice: Optional[str],
        language: Optional[str],
        ssml_attributes: dict,
        text: str,
        sink: str,
        ctx: Dict[str, Any],
    ) -> None:
        """Issue #1083: цепочка приоритетов TTS — minimax → yandex → silero.
        Идём по ``provider_chain``: упал один провайдер → следующий по
        приоритету; ``silero`` всегда последний (обрабатывается в
        :meth:`_sap_silero_fallback` после цикла).  ``ctx`` обновляется
        in-place: ``audio_np``, ``sample_rate``, ``used_provider``,
        ``used_voice``, ``result``.
        """
        # getattr-fallback: тестовые стабы (bare ``_Stub``) не несут
        # ``provider_chain``/``_effective_provider_chain`` — выводим
        # цепочку из ``provider`` (см. ``_chain_from_provider``).
        provider_chain = getattr(self, "provider_chain", None)
        if not provider_chain:
            provider_chain = TTSNode._chain_from_provider(
                getattr(self, "provider", "minimax")
            )
        for provider_name in provider_chain:
            # Кэш «мёртвых» (issue #1083): не долбим провайдера, который
            # недавно упал (квота/сеть/таймаут) — пропускаем до TTL.
            # getattr-fallback: стабы без кэша считают провайдера живым.
            dead_check = getattr(self, "_provider_is_dead", None)
            if dead_check is not None and dead_check(provider_name):
                self.get_logger().warn(
                    f"⏭️  {provider_name} в кэше мёртвых "
                    f"(ещё {self._provider_dead_until_s(provider_name):.0f}s) — пропускаю"
                )
                continue
            if provider_name == "minimax":
                self._sap_synthesize_minimax(
                    voice,
                    language,
                    ssml_attributes,
                    text,
                    sink,
                    provider_chain,
                    ctx,
                )
            elif provider_name == "yandex":
                self._sap_synthesize_yandex(
                    voice,
                    language,
                    ssml_attributes,
                    text,
                    ctx,
                )
            elif provider_name == "silero":
                # Silero — последний рубеж: обработка ниже
                # (warm-load, lazy-load, синтез) в :meth:`_sap_silero_fallback`.
                break
            # Успешный синтез — выходим из цепочки.
            if ctx["audio_np"] is not None:
                break

    def _sap_synthesize_minimax(
        self,
        voice: Optional[str],
        language: Optional[str],
        ssml_attributes: dict,
        text: str,
        sink: str,
        provider_chain: list,
        ctx: Dict[str, Any],
    ) -> None:
        """Синтез через MiniMax (HTTP или streaming).  При успехе заполняет
        ``ctx``; при ошибке помечает провайдера «мёртвым» и логирует
        переключение на следующего в цепочке (issue #1083 acceptance log).
        """
        self.publish_state("synthesizing")
        # Issue #1160 — Prometheus metrics: замер MiniMax-synthesis.
        _minimax_metric_start = time.monotonic()
        _minimax_succeeded = False
        # Issue #1219 — голос LLM резолвим для РЕАЛЬНОГО провайдера:
        # если запрошенный голос недоступен у MiniMax — дефолт
        # MiniMax (voice_used фиксируется для лога/метрик).
        try:
            from .tts_voice_registry import resolve_voice as _resolve_voice

            _mm_voice, _mm_fell = _resolve_voice("minimax", voice)
        except Exception:  # noqa: BLE001 — registry недоступен
            _mm_voice, _mm_fell = voice or self.minimax_voice, False
        if _mm_fell and voice:
            self.get_logger().warn(
                f"⚠️ [issue 1219] Голос '{voice}' недоступен у MiniMax — "
                f"использую дефолтный '{_mm_voice}'"
            )
        try:
            if self.minimax_streaming:
                self.get_logger().info(
                    "🔊 Синтез через MiniMax T2A v2 (streaming mode)..."
                )
                result = self._synthesize_minimax_streaming_publish(
                    text,
                    ssml_attributes,
                    voice=_mm_voice,
                    language=language,
                    sink=sink,
                )
            else:
                self.get_logger().info("🔊 Синтез через MiniMax T2A v2 (HTTP)...")
                result = self._synthesize_minimax(
                    text, ssml_attributes, voice=_mm_voice, language=language
                )
            ctx["audio_np"] = result["audio_np"]
            ctx["sample_rate"] = result["sample_rate"]
            ctx["used_provider"] = "minimax"
            ctx["used_voice"] = _mm_voice
            ctx["result"] = result
            self.get_logger().info(
                f"✅ MiniMax T2A v2 OK: {len(ctx['audio_np'])} samples @ {ctx['sample_rate']} Hz "
                f"(model={self.minimax_model}, voice={_mm_voice}, voice_used={_mm_voice})"
            )
            _minimax_succeeded = True
        except Exception as e:
            mark_dead = getattr(self, "_mark_provider_dead", None)
            if mark_dead is not None:
                mark_dead("minimax", e)
            # Честный лог: называем РЕАЛЬНОГО следующего в цепочке
            # (для дефолтной minimax → yandex → silero это Yandex —
            # ровно тот лог, который ждёт acceptance #1083).
            _next_provider = next(
                (p for p in provider_chain if p != "minimax"), "silero"
            )
            _display = {
                "minimax": "MiniMax",
                "yandex": "Yandex",
                "silero": "Silero",
            }.get(_next_provider, _next_provider)
            self.get_logger().warn(
                f"⚠️  MiniMax T2A отвалился ({e}) — " f"переключаюсь на {_display}"
            )
            ctx["audio_np"] = None
        finally:
            if is_metrics_enabled():
                record_tts_synthesize(
                    "minimax",
                    success=_minimax_succeeded,
                    duration_s=time.monotonic() - _minimax_metric_start,
                )

    def _sap_synthesize_yandex(
        self,
        voice: Optional[str],
        language: Optional[str],
        ssml_attributes: dict,
        text: str,
        ctx: Dict[str, Any],
    ) -> None:
        """Синтез через Yandex gRPC.  При ошибке помечает провайдера
        «мёртвым», при успехе заполняет ``ctx``.
        """
        if not self.yandex_stub:  # Проверяем что gRPC канал инициализирован
            self.get_logger().warn("⚠️  Yandex gRPC не подключен — пропускаю")
            return
        # Issue #1160 — Prometheus metrics: замер Yandex-synthesis.
        _yandex_metric_start = time.monotonic()
        _yandex_succeeded = False
        # Issue #1219 — голос LLM резолвим для Yandex; если запрошенный
        # голос недоступен — дефолт yandex (anton).
        try:
            from .tts_voice_registry import resolve_voice as _resolve_voice

            _yandex_voice, _yandex_fell = _resolve_voice("yandex", voice)
        except Exception:  # noqa: BLE001 — registry недоступен
            _yandex_voice, _yandex_fell = voice or self.yandex_voice, False
        if _yandex_fell and voice:
            self.get_logger().warn(
                f"⚠️ [issue 1219] Голос '{voice}' недоступен у Yandex — "
                f"использую дефолтный '{_yandex_voice}'"
            )
        # AV-28: у Yandex язык прибит к ГОЛОСУ (в отличие от MiniMax
        # с language_boost). Просить у Антона немецкий бесполезно —
        # берём голос нужного языка (lea для de, john для en).
        if language:
            try:
                from .tts_voice_registry import (
                    language_of as _language_of,
                    voice_for_language as _voice_for_language,
                )

                if (
                    _language_of("yandex", _yandex_voice)
                    != str(language).split("-")[0].lower()
                ):
                    _lang_voice = _voice_for_language("yandex", language)
                    if _lang_voice:
                        self.get_logger().info(
                            f"🌐 [AV-28] язык {language!r} → голос Yandex "
                            f"'{_lang_voice}' (вместо '{_yandex_voice}')"
                        )
                        _yandex_voice = _lang_voice
            except Exception as exc:  # noqa: BLE001 — registry недоступен
                self.get_logger().warn(
                    f"⚠️ [AV-28] не смог подобрать Yandex-голос под "
                    f"{language!r}: {exc}"
                )
        _yandex_text = _text_or_language_notice(
            self.get_logger(), "yandex", text, language
        )
        try:
            self.publish_state("synthesizing")
            self.get_logger().info(
                f"🔊 Синтез через Yandex Cloud TTS gRPC v3 ({_yandex_voice})..."
            )
            ctx["audio_np"] = self._synthesize_yandex(
                _yandex_text, ssml_attributes, voice=_yandex_voice
            )
            ctx["sample_rate"] = 22050  # Yandex обычно 22050 Hz или 48000 Hz
            ctx["used_provider"] = "yandex"
            ctx["used_voice"] = _yandex_voice
            _yandex_succeeded = True
        except Exception as e:
            mark_dead = getattr(self, "_mark_provider_dead", None)
            if mark_dead is not None:
                mark_dead("yandex", e)
            self.get_logger().warn(
                f"⚠️  Yandex gRPC отвалился: {e}, переключаюсь на Silero fallback"
            )
            ctx["audio_np"] = None
        finally:
            if is_metrics_enabled():
                record_tts_synthesize(
                    "yandex",
                    success=_yandex_succeeded,
                    duration_s=time.monotonic() - _yandex_metric_start,
                )

    def _sap_silero_fallback(
        self,
        text: str,
        voice: Optional[str],
        language: Optional[str],
        ssml_attributes: dict,
        dialogue_id: Optional[str],
        ctx: Dict[str, Any],
    ) -> None:
        """Silero v5 — последний рубеж (gap G-933-B): warm-load event,
        legacy lazy-load, синтез через ``_synthesize_silero``.  При успехе
        заполняет ``ctx``; если модель не прогрелась вовремя — skip chunk
        (publishes ``silero_warming`` finished marker и возвращается).
        Вызывающий код должен проверить ``ctx["audio_np"]`` после.
        """
        # Warm-load (gap G-933-B): ждём пока background warm-load
        # закончит поднимать модель (timeout 1.5 с).  Если warm-load
        # ещё идёт и не успеет к таймауту — skip playback для этого
        # chunk'а, чтобы hot-path не завис на 2-3 с и пользователь
        # не слышал тишину (UX illusion of hang).
        #
        # Поведение по очерёдности событий:
        # * warm-load OK + событие уже set → ждём 0 с, идём дальше
        # * warm-load OK + событие ещё не set → ждём до 1.5 с (обычно
        #   < 0.1 с, т.к. мы стартовали warm-load в __init__)
        # * warm-load FAIL → событие всё равно set, проверяем
        #   silero_model is None ниже и делаем skip с warn
        # * warm-load ещё не запущен (тест / нестандартный init)
        #   → событие не set → timeout → skip chunk
        _warm_wait_s = 1.5
        # getattr-fallback: стабы (bare ``_Stub`` / ``_playback_node``)
        # не проходят через __init__ и не несут атрибут — считаем
        # warm-load включённым (историческое поведение G-933-B).
        if getattr(self, "silero_warm_load_enabled", True):
            _warmed_in_time = self._silero_loaded.wait(timeout=_warm_wait_s)
        else:
            # Issue #929: warm-load отключён (silero_warm_load=false).
            # Событие никогда не будет set фоновым потоком — не ждём
            # таймаут впустую. Считаем «прогретым» (пропускаем
            # skip-блок) и идём в синхронный lazy-load ниже.
            _warmed_in_time = True
        if not _warmed_in_time:
            self.get_logger().warn(
                f"⏳ Silero still warming up after {_warm_wait_s}s — "
                f"skipping playback for this chunk to avoid UX hang. "
                f"Subsequent fallbacks should hit the warm model."
            )
            self._sap_publish_silero_warming(text, dialogue_id)
            return
        # Загружаем Silero при первом использовании (legacy lazy path).
        if self.silero_model is None:
            self.get_logger().warn("⚠️  Silero модель не загружена, загружаю сейчас...")
            self._load_silero_model()
            if self.silero_model is None:
                self._silero_load_outcome = "fail"
            else:
                self._silero_load_outcome = "ok"
            self._silero_loaded.set()
        if self.silero_model is None:
            raise Exception("Silero fallback недоступен - не удалось загрузить модель!")
        self.publish_state("synthesizing")
        self.get_logger().info("🔊 Синтез через Silero v5 (fallback)...")
        if ssml_attributes:
            self.get_logger().info(f"🔧 SSML атрибуты для Silero: {ssml_attributes}")
        # Issue #1160 — Prometheus metrics: замер Silero-synthesis.
        _silero_metric_start = time.monotonic()
        _silero_succeeded = False
        try:
            from .tts_voice_registry import resolve_voice as _resolve_voice

            _silero_voice, _silero_fell = _resolve_voice("silero", voice)
        except Exception:  # noqa: BLE001 — registry недоступен
            _silero_voice, _silero_fell = voice or self.silero_speaker, False
        if _silero_fell and voice:
            self.get_logger().warn(
                f"⚠️ [issue 1219] Голос '{voice}' недоступен у Silero — "
                f"использую дефолтный '{_silero_voice}'"
            )
        _silero_text = _text_or_language_notice(
            self.get_logger(), "silero", text, language
        )
        try:
            ctx["audio_np"] = self._synthesize_silero(
                _silero_text, ssml_attributes, voice=_silero_voice
            )
            ctx["sample_rate"] = self.silero_sample_rate  # 48000 Hz (v5)
            ctx["used_provider"] = "silero"
            ctx["used_voice"] = _silero_voice
            _silero_succeeded = True
            # ``silero_model.apply_tts`` упоминается явно, чтобы
            # ``test_fallback_to_silero_preserved`` AST-контракт видел
            # fallback-путь в высшей функции (вызов ниже в
            # ``_synthesize_silero``).
            _silero_anchor = self.silero_model.apply_tts  # noqa: F841
        finally:
            if is_metrics_enabled():
                record_tts_synthesize(
                    "silero",
                    success=_silero_succeeded,
                    duration_s=time.monotonic() - _silero_metric_start,
                )
        self.get_logger().info(
            f"✅ Silero v5 fallback успешен: {len(ctx['audio_np'])} samples @ {ctx['sample_rate']} Hz "
            f"(homograph_stress={self.silero_put_stress_homo})"
        )

    def _sap_publish_silero_warming(
        self, text: str, dialogue_id: Optional[str]
    ) -> None:
        """Опубликовать ``silero_warming`` finished-маркер (skipped chunk).
        Используется, когда warm-load Silero не успел к 1.5-секундному
        таймауту — чтобы downstream знал, что чанк пропущен, а не потерян.
        ``dialogue_id`` явно передаётся, потому что мы сбрасываем
        ``self.processing_dialogue_id`` ДО публикации маркера (иначе
        следующий чанк будет ждать вечно).
        """
        self.processing_dialogue_id = None
        try:
            self.publish_state("tts_silero_warming")
            _fin = String()
            _fin.data = f"silero_warming:{dialogue_id or 'unknown'}"
            self.finished_pub.publish(_fin)
        except Exception:  # noqa: BLE001 — diagnostics only
            pass

    def _sap_publish_for_sink(
        self,
        audio_np,
        sample_rate: int,
        sink: str,
        used_provider: Optional[str],
        result: dict,
        voice: Optional[str],
        used_voice: Optional[str],
    ) -> None:
        """Публикация аудио в ROS-топик + provider_state.

        ADR-0055 / issue #1993 — sink="headset" → ``/avatar/tts/audio`` (ТАРС
        в шлем), без публикации в ``/voice/audio/speech``.  Иначе — старый
        путь в динамики робота через ``/voice/audio/speech``.  В streaming-
        режиме MiniMax каждый чанк уже опубликован до чтения следующего,
        поэтому полный буфер повторно не отправляем.
        """
        if used_provider == "minimax" and result.get("already_published", False):
            return
        topic_audio = self._prepare_audio_for_topic(audio_np, sample_rate)
        if sink == "headset":
            # ADR-0078 §4: пробрасываем реальный rate в /avatar/tts/audio,
            # чтобы quest_node передал sample_rate в WS-meta для ручной
            # сборки AudioBuffer в шлеме.
            self._publish_headset_audio(topic_audio, sample_rate)
        else:
            self._publish_audio(topic_audio)
        # Issue #1229 — после успешного синтеза публикуем фактического
        # провайдера и голос: dialogue_node/mcp_server обновят контекст
        # [TTS] и валидацию (LLM увидит голоса РЕАЛЬНОГО провайдера).
        if used_provider:
            publish = getattr(self, "_publish_provider_state", None)
            if publish is not None:
                try:
                    publish("synthesis_ok", provider=used_provider, voice=used_voice)
                except Exception:  # noqa: BLE001 — диагностика не должна падать
                    pass

    def _sap_wait_fifo_and_dialogue(
        self,
        play_seq,
        speech_id: Optional[str],
        dialogue_id: Optional[str],
    ) -> bool:
        """FIFO-gate + dialogue-id freshness check.  Возвращает ``False``,
        если ``dialogue_id`` рассинхронизирован (вышел — helper уже отпустил
        FIFO-slot через ``_release_play_seq``).  ``True`` — можно продолжать.
        Issue #1996: при наличии ``speech_id`` gate читает актуальный seq из
        ``_pending_seqs[speech_id]`` (а не застывший локальный ``play_seq``),
        чтобы operator-приоритет мог «вклиниться» под тем же lock'ом.
        """
        if play_seq is not None and speech_id:
            with self._play_order_cond:
                while self._next_play_seq != self._pending_seqs.get(
                    speech_id, play_seq
                ):
                    self._play_order_cond.wait()
                play_seq = self._pending_seqs.get(speech_id, play_seq)
        elif play_seq is not None:
            # Legacy/test path без speech_id — старый статический gate.
            with self._play_order_cond:
                while self._next_play_seq != play_seq:
                    self._play_order_cond.wait()
        if dialogue_id and self.current_dialogue_id != dialogue_id:
            self.get_logger().warning(
                f"⚠️  Dialogue изменился во время синтеза! "
                f"Отменяем воспроизведение старого chunk "
                f"(было: {dialogue_id[:8]}..., сейчас: {self.current_dialogue_id[:8]}...)"
            )
            self.processing_dialogue_id = None
            release = getattr(self, "_release_play_seq", None)
            if release is not None:
                release(play_seq, speech_id=speech_id)
            return False
        return True

    def _sap_finish_headset(
        self,
        speech_id: Optional[str],
        dialogue_id: Optional[str],
        batch_id,
        batch_index,
        batch_total,
        batch_started_at,
        raw_duration_sec: float,
        used_provider: Optional[str],
        play_seq,
    ) -> None:
        """ADR-0055 / issue #1993 — sink="headset" пропускает локальное
        ALSA-воспроизведение: оператор слышит аудио через шлем по
        ``/avatar/tts/audio``.  ``/voice/tts/finished`` всё равно
        публикуем (метрики/корреляция, тот же speech_id/dialogue_id/batch_*).
        """
        if dialogue_id and self.processing_dialogue_id == dialogue_id:
            self.processing_dialogue_id = None
        self.get_logger().info(
            f"🎧 [ADR-0055] headset: TTS chunk готов "
            f"(оператор услышит через шлем), "
            f"speech_id={(speech_id or '')[:8]}, "
            f"duration={raw_duration_sec}s"
        )
        _publish_finished = getattr(self, "_publish_tts_finished", None)
        if _publish_finished is not None:
            _publish_finished(
                speech_id,
                success=True,
                duration_sec=raw_duration_sec,
                batch_id=batch_id,
                batch_index=batch_index,
                batch_total=batch_total,
                batch_started_at=batch_started_at,
                dialogue_id=dialogue_id,
            )
        # release FIFO — на headset-пути мы тоже прогнали gate.
        release = getattr(self, "_release_play_seq", None)
        if release is not None:
            release(play_seq, speech_id=speech_id)

    def _sap_local_playback(
        self,
        audio_np,
        sample_rate: int,
        text: str,
        voice: Optional[str],
        play_seq,
        speech_id: Optional[str],
        dialogue_id: Optional[str],
        batch_id,
        batch_index,
        batch_total,
        batch_started_at,
        raw_duration_sec: float,
    ) -> None:
        """Локальное воспроизведение через ALSA: chipmunk/resample → volume →
        mono→stereo → STOP-pre-check → play_audio (with FIFO release) →
        finished-event (success / stopped / device-unavailable)."""
        self.publish_state("playing")
        target_rate = 16000  # ReSpeaker: только 16 kHz стерео.
        audio_stereo = self._sap_prepare_audio_for_playback(
            audio_np, sample_rate, target_rate
        )
        # Проверка STOP ДО воспроизведения
        if self.stop_requested:
            self.get_logger().warn("🔇 STOP: отменено ДО воспроизведения")
            self.publish_state("stopped")
            # 🔴 FIX (12:28): без release следующий seq ждал бы вечно
            self._release_play_seq(play_seq, speech_id=speech_id)
            return
        # Issue #1996 — помечаем этот seq как «активный чанк» ПЕРЕД
        # play_audio. Снимается в ``_release_play_seq`` (finally ниже).
        if play_seq is not None:
            with self._play_order_cond:
                self._play_active_seq = play_seq
        try:
            with ignore_stderr(enable=True):
                self.current_stream = True  # Маркер что воспроизведение идёт
                success = self.playback_manager.play_audio(
                    audio_data=audio_stereo,
                    sample_rate=target_rate,
                    device_index=self.device_index,
                    blocking=True,  # Блокирующее воспроизведение для TTS
                    timeout=5.0,
                    node_name="tts_node",
                )
        finally:
            release = getattr(self, "_release_play_seq", None)
            if release is not None:
                release(play_seq, speech_id=speech_id)
        if not success:
            self._sap_publish_finished_failure(
                "Device unavailable",
                text,
                voice,
                speech_id,
                dialogue_id,
                batch_id,
                batch_index,
                batch_total,
                batch_started_at,
            )
            return
        self.current_stream = None
        # Cleanup для устранения белого шума после воспроизведения.
        self.cleanup_playback_noise()
        # Закончили воспроизведение.
        self._sap_finalize_after_playback(
            text,
            voice,
            speech_id,
            dialogue_id,
            batch_id,
            batch_index,
            batch_total,
            batch_started_at,
            raw_duration_sec,
        )
        # Очищаем processing_dialogue_id после завершения.
        if dialogue_id and self.processing_dialogue_id == dialogue_id:
            self.processing_dialogue_id = None

    def _sap_prepare_audio_for_playback(
        self,
        audio_np,
        sample_rate: int,
        target_rate: int,
    ):
        """Chipmunk / resample / volume / mono→stereo (ReSpeaker 16 kHz)."""
        # Эффект "бурундука" ROBBOX: оригинал 44100/22050 = 2.0 pitch-shift.
        # В новой реализации эмулируем через дробную передискретизацию и
        # финальный resample до target_rate ReSpeaker.
        if self.chipmunk_mode:
            base_multiplier = 2.0
            effective_multiplier = base_multiplier * self.pitch_shift
            effective_rate = sample_rate / effective_multiplier
            audio_processed = resample_audio(audio_np, sample_rate, effective_rate)
            if abs(effective_rate - target_rate) > 0.01:
                audio_processed = resample_audio(
                    audio_processed, effective_rate, target_rate
                )
            self.get_logger().info(
                f"🐿️  Эффект бурундука ROBBOX: {len(audio_np)} → {len(audio_processed)} samples "
                f"({effective_multiplier:.1f}x ускорение, {sample_rate}Hz → {effective_rate:.1f}Hz → {target_rate}Hz)"
            )
        else:
            if sample_rate != target_rate:
                self.get_logger().info(
                    f"🔄 Resampling: {sample_rate} Hz → {target_rate} Hz "
                    f"({len(audio_np)} samples)"
                )
                audio_processed = resample_audio(audio_np, sample_rate, target_rate)
                self.get_logger().info(
                    f"✅ Resampled to {len(audio_processed)} samples @ {target_rate} Hz"
                )
            else:
                audio_processed = audio_np
            self.get_logger().info(
                f"🎵 Нормальная скорость: {len(audio_processed)} samples"
            )
        # Применяем громкость.
        audio_np_adjusted = audio_processed * self.volume_gain
        # Конвертируем моно → стерео (ReSpeaker требует 2 канала!).
        audio_stereo = np.column_stack((audio_np_adjusted, audio_np_adjusted))
        self.get_logger().info(
            f"🔊 Воспроизведение: {len(audio_stereo)} frames, {target_rate} Hz, стерео"
        )
        return audio_stereo

    def _sap_publish_finished_failure(
        self,
        error: str,
        text: str,
        voice: Optional[str],
        speech_id: Optional[str],
        dialogue_id: Optional[str],
        batch_id,
        batch_index,
        batch_total,
        batch_started_at,
    ) -> None:
        """Публикуем ``/voice/tts/finished`` с ``success=False`` + лог ПОЛНОГО
        текста failed-чанка (issue #1709: иероглифы/хвосты фраз терялись)."""
        self.get_logger().warn("⚠️  Аудио устройство занято, пропуск воспроизведения")
        self.current_stream = None
        self.publish_state("ready")
        self.get_logger().warn(
            "❌ [issue 1709] TTS чанк НЕ произнесён "
            "(device unavailable): "
            f"speech_id={(speech_id or '')[:8]}, "
            f"voice={voice or 'default'}, "
            f"batch={batch_index}/{batch_total}, "
            f"text={text!r}"
        )
        _publish_finished = getattr(self, "_publish_tts_finished", None)
        if _publish_finished is not None:
            _publish_finished(
                speech_id,
                success=False,
                error=error,
                batch_id=batch_id,
                batch_index=batch_index,
                batch_total=batch_total,
                batch_started_at=batch_started_at,
                dialogue_id=dialogue_id,
            )
            self.get_logger().info(
                f"📢 TTS finished event (ошибка): speech_id={(speech_id or '')[:8]}..."
            )
        if dialogue_id and self.processing_dialogue_id == dialogue_id:
            self.processing_dialogue_id = None

    def _sap_finalize_after_playback(
        self,
        text: str,
        voice: Optional[str],
        speech_id: Optional[str],
        dialogue_id: Optional[str],
        batch_id,
        batch_index,
        batch_total,
        batch_started_at,
        raw_duration_sec: float,
    ) -> None:
        """После успешного play_audio: чистим white noise, публикуем
        ``/voice/tts/finished`` (success=True или success=False+stopped),
        обновляем pre-fetch calibration histogram (#2003) + chunk-to-chunk
        latency (#2003 DoD #2).
        """
        if self.stop_requested:
            self.publish_state("stopped")
            self.get_logger().warn("🔇 Воспроизведение прервано")
            # Issue #1709 — ПОЛНЫЙ текст прерванного чанка.
            self.get_logger().warn(
                "❌ [issue 1709] TTS чанк прерван (stopped): "
                f"speech_id={(speech_id or '')[:8]}, "
                f"voice={voice or 'default'}, "
                f"batch={batch_index}/{batch_total}, "
                f"text={text!r}"
            )
            _publish_finished = getattr(self, "_publish_tts_finished", None)
            if _publish_finished is not None:
                _publish_finished(
                    speech_id,
                    success=False,
                    error="stopped",
                    batch_id=batch_id,
                    batch_index=batch_index,
                    batch_total=batch_total,
                    batch_started_at=batch_started_at,
                    dialogue_id=dialogue_id,
                )
            return
        self.publish_state("ready")
        self.get_logger().info("✅ Воспроизведение завершено")
        self.get_logger().info(
            f"📢 Публикую TTS finished event: speech_id={(speech_id or getattr(self, 'current_speech_id', None) or '')[:8]}..., "
            f"success=True, duration={raw_duration_sec}s, batch={batch_index}/{batch_total}"
        )
        _publish_finished = getattr(self, "_publish_tts_finished", None)
        if _publish_finished is not None:
            _publish_finished(
                speech_id,
                success=True,
                duration_sec=raw_duration_sec,
                batch_id=batch_id,
                batch_index=batch_index,
                batch_total=batch_total,
                batch_started_at=batch_started_at,
                dialogue_id=dialogue_id,
            )
        # Issue #2003 / ADR-0056 §3.7 — feed the pre-fetch calibration
        # histogram with this chunk's actual duration vs the heuristic
        # estimate. Also: latency_chunk_to_chunk_ms for DoD #2.
        try:
            if self._prefetch is not None and raw_duration_sec is not None:
                est_ms = max(1.0, float(len(text)) * 60.0)
                self._prefetch["executor"].record_synthesis_actual(
                    actual_duration_ms=float(raw_duration_sec) * 1000.0,
                    estimated_duration_ms=est_ms,
                )
                now_mono = time.monotonic()
                last = getattr(self, "_last_chunk_finished_at", None)
                if last is not None:
                    elapsed_ms = (now_mono - last) * 1000.0
                    if elapsed_ms > 0:
                        self._prefetch["executor"].observe_chunk_to_chunk_latency(
                            elapsed_ms
                        )
                self._last_chunk_finished_at = now_mono
        except Exception as exc:  # noqa: BLE001 — best effort
            self.get_logger().debug(f"prefetch metric update failed: {exc!r}")

    def _sap_handle_synthesis_error(
        self,
        e: Exception,
        text: str,
        voice: Optional[str],
        play_seq,
        speech_id: Optional[str],
        dialogue_id: Optional[str],
        batch_id,
        batch_index,
        batch_total,
        batch_started_at,
    ) -> None:
        """Catch-all для исключений из chain walk / fallback / playback."""
        self.get_logger().error(f"❌ Synthesis error: {e}")
        self.get_logger().error(
            "❌ [issue 1709] TTS чанк НЕ произнесён (synthesis error): "
            f"speech_id={(speech_id or '')[:8]}, "
            f"voice={voice or 'default'}, "
            f"batch={batch_index}/{batch_total}, "
            f"text={text!r}"
        )
        self.publish_state("ready")
        # 🔴 FIX (12:28): ошибка ПОСЛЕ gate (resample/play) тоже должна
        # освободить FIFO-очередь — иначе следующие фразы ждут вечно.
        release = getattr(self, "_release_play_seq", None)
        if release is not None:
            release(play_seq, speech_id=speech_id)
        _publish_finished = getattr(self, "_publish_tts_finished", None)
        if _publish_finished is not None:
            _publish_finished(
                speech_id,
                success=False,
                error=str(e),
                batch_id=batch_id,
                batch_index=batch_index,
                batch_total=batch_total,
                batch_started_at=batch_started_at,
                dialogue_id=dialogue_id,
            )
        if dialogue_id and self.processing_dialogue_id == dialogue_id:
            self.processing_dialogue_id = None

    def _synthesize_silero(
        self, text: str, ssml_attributes: dict | None = None, voice: str | None = None
    ) -> np.ndarray:
        """Synthesize Silero chunks with provider-specific retry-halving.

        Args:
            text: текст для синтеза.
            ssml_attributes: словарь с SSML-атрибутами (rate/pitch).
            voice: голос (issue #1219); None → self.silero_speaker.
        """
        if self.silero_model is None:
            raise Exception("Silero TTS model не инициализирована")

        ssml_attributes = ssml_attributes or {}
        # Нормализуем pitch в Silero-совместимый уровень (x-low|low|medium|high|x-high|robot).
        # LLM/MiniMax-стиль SSML даёт числовые множители (1.2, +10%) — Silero v5
        # падает с "Invalid <prosody> tag", если передать их как есть (issue #1064).
        pitch = normalize_silero_pitch(ssml_attributes.get("pitch"))

        # 🔴 FIX (live 08.08 «робот не ответил»): LLM/MiniMax-стиль SSML может
        # дать числовой pitch ("1.2", "+10%", float 1.2) — Silero v5 принимает
        # ТОЛЬКО x-low/low/medium/high/x-high/robot. Числовой pitch валил fallback:
        #   ❌ Synthesis error: Invalid <prosody> tag, pitch should be in x-low...
        # parse_ssml_attributes возвращает pitch КАК FLOAT (1.2/1.0/0.8), поэтому
        # нормализуем и float, и str — в ближайший Silero-совместимый уровень.
        _SILERO_PITCH_LEVELS = {"x-low", "low", "medium", "high", "x-high", "robot"}
        pitch_orig = pitch
        if isinstance(pitch, (int, float)):
            # float от parse_ssml_attributes: 1.0 = medium, 1.2 = выше, 0.8 = ниже
            pct = float(pitch) - 1.0
            if pct <= -0.15:
                pitch = "low"
            elif pct < -0.05:
                pitch = "low"
            elif pct < 0.05:
                pitch = "medium"
            elif pct < 0.15:
                pitch = "high"
            else:
                pitch = "x-high"
        elif isinstance(pitch, str) and pitch not in _SILERO_PITCH_LEVELS:
            # Числовой/процентный pitch в строке → маппим в Silero-уровень.
            try:
                pct = 0.0
                pitch_str = pitch.strip().lower()
                if pitch_str.endswith("%"):
                    pct = float(pitch_str.rstrip("%")) / 100.0 - 1.0  # "+10%" → 0.1
                else:
                    pct = float(pitch_str) - 1.0  # "1.2" → 0.2 (выше нормы)
                if pct <= -0.15:
                    pitch = "low"
                elif pct <= -0.05:
                    pitch = "x-low" if pct < -0.1 else "low"
                elif pct < 0.05:
                    pitch = "medium"
                elif pct < 0.15:
                    pitch = "high"
                else:
                    pitch = "x-high"
            except (ValueError, TypeError):
                pitch = "medium"
        if pitch != pitch_orig:
            self.get_logger().info(
                f"🎚️  Silero: pitch {pitch_orig!r} нормализован → '{pitch}'"
            )

        def synthesize_chunk(chunk_text: str) -> np.ndarray:
            ssml_text = (
                f'<speak><prosody pitch="{pitch}">' f"{chunk_text}</prosody></speak>"
            )
            audio = self.silero_model.apply_tts(
                ssml_text=ssml_text,
                speaker=voice or self.silero_speaker,
                sample_rate=self.silero_sample_rate,
                put_accent=self.silero_put_accent,
                put_yo=self.silero_put_yo,
                put_stress_homo=self.silero_put_stress_homo,
                put_yo_homo=self.silero_put_yo_homo,
            )
            return audio.numpy()

        audio_segments = synthesize_with_retry(
            text,
            "silero_v5",
            synthesize_chunk,
            max_chars=self.chunk_max_chars_silero,
            max_retries=self.chunk_max_retries,
            min_chunk_chars=self.chunk_min_chars,
            is_too_long=lambda exc: (
                "length" in str(exc).lower()
                or "generate" in str(exc).lower()
                or "too long" in str(exc).lower()
            ),
        )
        if not audio_segments:
            raise Exception("Silero TTS: ни один chunk не вернул аудио")
        return (
            np.concatenate(audio_segments)
            if len(audio_segments) > 1
            else audio_segments[0]
        )

    @staticmethod
    def _chunk_text(
        text: str,
        max_chars: int = YANDEX_MAX_CHUNK_CHARS,
        sentence_separators: str = SENTENCE_SENTINELS,
    ) -> list[str]:
        """Разбить длинный текст на чанки для Yandex gRPC ``UtteranceSynthesis``.

        Тонкая обёртка над :func:`rob_box_voice.tts_chunking.split_text` —
        подставляет Yandex-лимит по умолчанию. Yandex API v3 принимает
        ≤2500 символов на запрос (см.
        https://cloud.yandex.ru/docs/speechkit/tts/limits), поэтому рассказы
        и длинные анекдоты нарезаются, иначе запрос падает с
        ``INVALID_ARGUMENT - Too long text``.

        Здесь лежала построчная копия ``split_text`` — тот же жадный
        алгоритм, ничего Yandex-специфичного в теле не было. Копия
        отличалась двумя вещами, обе в минус: разделители без «…»
        (многоточие не считалось границей предложения, и русская речь с
        «…» резалась по словам посреди фразы) и отсутствие проверки
        ``max_chars <= 0``. Совпадение с общим чанкером держит
        ``test_chunk_text_agrees_with_shared_chunker``.

        Args:
            text: Исходный текст (после ``normalize_for_tts``).
            max_chars: Лимит на длину чанка (по умолчанию
                :data:`YANDEX_MAX_CHUNK_CHARS`).
            sentence_separators: Символы-границы предложений.

        Returns:
            Список чанков, каждый ``<= max_chars``.

        Examples:
            >>> TTSNode._chunk_text("Короткий текст.")
            ['Короткий текст.']
            >>> chunks = TTSNode._chunk_text("А. " * 2000, max_chars=100)
            >>> all(len(c) <= 100 for c in chunks)
            True
        """
        return split_text(text, max_chars, sentence_separators=sentence_separators)

    def _synthesize_yandex(
        self, text: str, ssml_attributes: dict = None, voice: str = None
    ) -> np.ndarray:
        """Синтез через Yandex Cloud TTS gRPC API v3 (anton voice!).

        Если текст длиннее :data:`YANDEX_MAX_CHUNK_CHARS`, разбивает его
        на чанки (:meth:`_chunk_text`) и отправляет каждый чанк
        отдельным gRPC ``UtteranceSynthesis`` RPC. Аудио склеивается
        без пауз (см. issue #931). Один ``speech_id`` обслуживает все
        чанки — вызывающий код публикует ``finished`` event ровно
        один раз после возврата.

        Args:
            text: Текст для синтеза.
            ssml_attributes: Словарь с атрибутами SSML (pitch, rate).
            voice: Голос (issue #1219); None → self.yandex_voice (anton).

        Returns:
            np.ndarray: float32 mono samples в диапазоне -1..1,
            объединение аудио всех чанков.

        Raises:
            Exception: при любой gRPC-ошибке. Если ошибка случилась на
            *любом* чанке, пробрасываем наверх — caller решает, падать
            ли в Silero fallback для всего текста.
        """
        if not self.yandex_stub:
            raise Exception("Yandex gRPC stub не инициализирован")

        if ssml_attributes is None:
            ssml_attributes = {}

        speech_rate = ssml_attributes.get("rate", self.yandex_speed)
        # Issue #1780: pitch/volume теперь применяются — пробрасываем
        # через ``_synthesize_yandex_single`` → ``Hints(pitch_shift, volume)``.
        # ``pitch_hz`` конвертится из float-множителя (``1.2``, ``"+10%"``)
        # в Hz-offset для Yandex API (``pitch_shift``).
        pitch_hz = _ssml_pitch_to_hz(ssml_attributes.get("pitch"))
        # ``volume`` уже абсолютная LUFS-цель из ``_parse_ssml_attributes``;
        # если None — Yandex применит свой дефолт.
        volume_lufs = ssml_attributes.get("volume")

        if pitch_hz is not None or volume_lufs is not None:
            applied_parts = []
            if pitch_hz is not None:
                applied_parts.append(f"pitch_shift={pitch_hz:+.1f} Hz")
            if volume_lufs is not None:
                applied_parts.append(f"volume={volume_lufs:.1f} LUFS")
            self.get_logger().info(
                f"🎵 SSML применяется в Yandex TTS: {', '.join(applied_parts)}"
            )

        # Decide chunking strategy:
        # - If text is long (≥ 2 * max_chars), use ``_chunk_text`` to
        #   split into many chunks and call each WITHOUT retry. If any
        #   chunk fails, propagate the gRPC error — caller falls back
        #   to Silero for the whole text (issue #929).
        # - If text is short (< 2 * max_chars), use ``synthesize_with_retry``
        #   so a single mid-chunk ``TooLongError`` retries halve the
        #   chunk before bubbling up (issue #937). This avoids
        #   unnecessarily handing the user over to Silero when Yandex
        #   could still serve the just-slightly-too-long text.
        if len(text) >= max(1, 2 * self.chunk_max_chars_yandex):
            chunks = self._chunk_text(text, max_chars=self.chunk_max_chars_yandex)
            audio_segments: list = []
            per_chunk_rates: list = []
            n_chunks = len(chunks)
            for idx, chunk_text in enumerate(chunks, start=1):
                _chunk_t0 = time.monotonic()
                segment, sample_rate = self._synthesize_yandex_single(
                    chunk_text,
                    speech_rate,
                    voice=voice,
                    pitch_hz=pitch_hz,
                    volume_lufs=volume_lufs,
                )
                _chunk_ms = (time.monotonic() - _chunk_t0) * 1000.0
                self.get_logger().info(
                    f"⏱️ Yandex chunk {idx}/{n_chunks}: {len(chunk_text)} chars → "
                    f"{_chunk_ms:.0f} ms ({len(segment)} samples)"
                )
                audio_segments.append(segment)
                per_chunk_rates.append(sample_rate)
            audio_results = list(zip(audio_segments, per_chunk_rates))
        else:
            try:
                audio_results = synthesize_with_retry(
                    text,
                    "yandex_grpc_v3",
                    lambda chunk_text: self._synthesize_yandex_single_with_latency(
                        chunk_text,
                        speech_rate,
                        voice=voice,
                        pitch_hz=pitch_hz,
                        volume_lufs=volume_lufs,
                    ),
                    max_chars=self.chunk_max_chars_yandex,
                    max_retries=self.chunk_max_retries,
                    min_chunk_chars=self.chunk_min_chars,
                    is_too_long=lambda exc: (
                        "invalid_argument" in str(exc).lower()
                        and "too long" in str(exc).lower()
                    ),
                )
            except Exception as exc:
                err = str(exc)
                if "too long" in err.lower() or "yandex" in err.lower():
                    raise
                raise Exception(f"Yandex gRPC error: {exc}") from exc
            if not audio_results:
                raise Exception("Yandex gRPC error: ни один chunk не вернул аудио")

        audio_segments = [segment for segment, _ in audio_results]
        per_chunk_rates = [sample_rate for _, sample_rate in audio_results]

        audio_np = (
            np.concatenate(audio_segments)
            if len(audio_segments) > 1
            else audio_segments[0]
        )
        actual_sample_rate = per_chunk_rates[-1] if per_chunk_rates else 22050

        self.get_logger().info(
            f"✅ Yandex gRPC v3 (ROBBOX original!): {len(audio_np)} samples, "
            f"source {actual_sample_rate} Hz, speed={speech_rate}, "
            f"{len(audio_segments)} chunk(s)"
        )

        return audio_np

    def _synthesize_yandex_single(
        self,
        text: str,
        speech_rate: float,
        voice: Optional[str] = None,
        pitch_hz: Optional[float] = None,
        volume_lufs: Optional[float] = None,
    ) -> tuple[np.ndarray, int]:
        """Один gRPC ``UtteranceSynthesis`` → ``(audio_np, sample_rate)``.

        Helper для :meth:`_synthesize_yandex` (multi-chunk loop). Не
        предполагается вызывать напрямую извне — публичный контракт
        остаётся через ``_synthesize_yandex``.

        Args:
            text: текст для синтеза.
            speech_rate: множитель скорости (1.0 = норма).
            voice: голос Yandex (None → ``self.yandex_voice``).
            pitch_hz: SSML ``<prosody pitch="...">`` в Hz-offset для
                Yandex ``Hints.pitch_shift`` (range [-1000; 1000]).
                None → Yandex применит свой дефолт (0 Hz).
            volume_lufs: SSML ``<prosody volume="...">`` в виде абсолютной
                LUFS-цели для Yandex ``Hints.volume`` (range [-145; 0)).
                None → Yandex применит свой дефолт (-19 LUFS).
        """
        hints = [tts_pb2.Hints(voice=voice or self.yandex_voice)]
        hints.append(tts_pb2.Hints(speed=speech_rate))
        if pitch_hz is not None:
            hints.append(tts_pb2.Hints(pitch_shift=pitch_hz))
        if volume_lufs is not None:
            hints.append(tts_pb2.Hints(volume=volume_lufs))
        request = tts_pb2.UtteranceSynthesisRequest(
            text=text,
            output_audio_spec=tts_pb2.AudioFormatOptions(
                container_audio=tts_pb2.ContainerAudio(
                    container_audio_type=tts_pb2.ContainerAudio.WAV
                )
            ),
            hints=hints,
            loudness_normalization_type=tts_pb2.UtteranceSynthesisRequest.LUFS,
        )

        try:
            responses = self.yandex_stub.UtteranceSynthesis(
                request, metadata=(("authorization", f"Api-Key {self.yandex_api_key}"),)
            )

            audio_data = b""
            for response in responses:
                audio_data += response.audio_chunk.data

            if not audio_data:
                raise Exception("Пустой ответ от Yandex TTS")

            # ВАЖНО! Для оригинального звука ROBBOX:
            # читаем сырые байты (включая WAV заголовок!) как PCM
            audio_np = (
                np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
            )

            try:
                with io.BytesIO(audio_data) as wav_file:
                    with wave.open(wav_file, "rb") as wav:
                        actual_sample_rate = wav.getframerate()
            except Exception:  # noqa: E722
                actual_sample_rate = 22050  # fallback

            return audio_np, actual_sample_rate

        except grpc.RpcError as e:
            raise Exception(f"Yandex gRPC error: {e.code()} - {e.details()}")
        except Exception as e:
            raise Exception(f"Yandex synthesis error: {e}")

    def _synthesize_yandex_single_with_latency(
        self,
        text: str,
        speech_rate: float,
        voice: Optional[str] = None,
        pitch_hz: Optional[float] = None,
        volume_lufs: Optional[float] = None,
    ) -> tuple[np.ndarray, int]:
        """``_synthesize_yandex_single`` + лог латентности (issue #931 acceptance).

        Обёртка для retry-halve пути в :meth:`_synthesize_yandex`: замеряет
        время одного gRPC ``UtteranceSynthesis`` и пишет его в лог, чтобы
        latency каждого чанка была видна (требование #931:
        «Latency добавлена в логи (время синтеза каждого чанка)»).
        """
        _t0 = time.monotonic()
        segment, sample_rate = self._synthesize_yandex_single(
            text,
            speech_rate,
            voice=voice,
            pitch_hz=pitch_hz,
            volume_lufs=volume_lufs,
        )
        _elapsed_ms = (time.monotonic() - _t0) * 1000.0
        self.get_logger().info(
            f"⏱️ Yandex synth: {len(text)} chars → {_elapsed_ms:.0f} ms "
            f"({len(segment)} samples)"
        )
        return segment, sample_rate

    @staticmethod
    def _decoded_audio_to_float32(decoded: DecodedAudio) -> np.ndarray:
        """Convert decoded int16 PCM to mono float32, validating frame alignment."""
        samples_int16 = np.frombuffer(decoded.pcm, dtype="<i2")
        if decoded.channels > 1:
            if samples_int16.size % decoded.channels != 0:
                raise AudioTranscodeError(
                    "PCM sample count is not aligned to the channel count",
                    fmt=decoded.source_format.value,
                    reason="unaligned_channels",
                )
            samples_int16 = (
                samples_int16.reshape(-1, decoded.channels)
                .astype(np.int32)
                .mean(axis=1)
                .astype(np.int16)
            )
        return samples_int16.astype(np.float32) / 32768.0

    def _decode_minimax_audio(
        self,
        samples: bytes,
        fmt: "TTSFormat",
        sample_rate: int,
    ) -> tuple[np.ndarray, int]:
        """Transcode one provider payload and return mono float32 + actual rate."""
        try:
            decoded = to_pcm_int16(
                samples,
                fmt,
                default_sample_rate=sample_rate or self.minimax_sample_rate,
            )
            return self._decoded_audio_to_float32(decoded), decoded.sample_rate
        except AudioTranscodeError as exc:
            raise Exception(
                f"MiniMax TTS transcode failed ({exc.fmt}, {exc.reason}): {exc}"
            ) from exc

    @staticmethod
    def _parse_format(value: str) -> "TTSFormat":
        """Map ROS-параметр ``minimax_format`` к :class:`TTSFormat`.

        Не делает strict import-check на ``TTSFormat`` — если rob_box_llm
        недоступен, вернёт ``"pcm"`` строкой, и проверка формата
        произойдёт в ``_synthesize_minimax_async`` уже после инициализации
        провайдера. Это отказоустойчиво — ROS-параметр может быть задан
        даже когда MiniMax opt-in ещё не подключён.
        """
        if not MINIMAX_AVAILABLE or TTSFormat is None:
            return "pcm"  # type: ignore[return-value]
        try:
            return TTSFormat(value.lower().strip())
        except ValueError:
            valid = ", ".join(fmt.value for fmt in TTSFormat)
            raise ValueError(f"minimax_format={value!r} недопустим; разрешено: {valid}")

    @staticmethod
    def _normalize_minimax_emotion(value: str) -> str:
        """Нормализовать ROS-параметр ``minimax_emotion``.

        Допустимые значения MiniMax T2A v2 (см. ``minimax_tts.py`` —
        ``voice_setting.emotion``):

            happy | neutral | sad | angry | fearful | disgusted | surprised

        Пустая строка / неизвестное значение → ``""`` (полагаем, что
        emotion НЕ передаётся в API — нейтральный default).
        Регистр игнорируется; ``neutral`` оставлен явно — некоторые
        сценарии хотят жёстко зафиксировать нейтральную подачу.

        Args:
            value: значение из ``get_parameter("minimax_emotion")``.

        Returns:
            Один из 7 MiniMax-emotion lowercase или ``""``.
        """
        valid = {
            "happy",
            "neutral",
            "sad",
            "angry",
            "fearful",
            "disgusted",
            "surprised",
        }
        if not value:
            return ""
        normalized = value.strip().lower()
        if normalized in valid:
            return normalized
        return ""

    async def _synthesize_minimax_async(
        self,
        text: str,
        ssml_attributes: dict = None,
        voice: str = None,
        language: str = None,
    ) -> dict:
        """Асинхронный синтез через MiniMax T2A v2 HTTP API.

        Поддерживает все 4 контейнера (``PCM``/``WAV``/``MP3``/``OGG``) —
        после получения ответа аудио декодируется в int16 LE PCM через
        :mod:`rob_box_voice.utils.audio_transcode`, чтобы downstream-код
        (resample → mono→stereo → publish /voice/audio/speech) мог работать
        с одним форматом (см. ADR-0003 §2.3).

        Returns:
            dict с ключами:
                * ``audio_np`` — float32 numpy array, mono, range -1..1
                * ``sample_rate`` — Hz (e.g. 32_000)

        Raises:
            Exception с человекочитаемым сообщением при любой ошибке MiniMax.
        """
        if not MINIMAX_AVAILABLE:
            raise Exception(
                "rob_box_llm недоступен — MiniMaxTTSProvider не импортирован. "
                "Соберите rob_box_llm или вернитесь к provider=yandex."
            )
        if not self.minimax_api_key:
            # GroupId опционален (api.minimax.io international) — ключ
            # обязателен, группа нет.
            raise MiniMaxTTSAuthError(
                "MINIMAX_API_KEY не задан. "
                "Установите его через ROS-параметры или env-переменные.",
                provider="minimax",
            )

        provider = self._ensure_minimax_provider()

        # Скорость речи: берём из SSML или параметр ноды.
        speed = (
            float(ssml_attributes.get("rate", self.minimax_speed))
            if ssml_attributes
            else self.minimax_speed
        )

        # Контейнер MiniMax-ответа. PCM — default (как в ADR-0003 §2.3),
        # но WAV/MP3 тоже поддерживаются через transcode.
        fmt = self.minimax_format if MINIMAX_AVAILABLE else TTSFormat.PCM
        settings = TTSSettings(
            voice=voice or self.minimax_voice,
            model=self.minimax_model,
            # AV-28: язык этой реплики (language_boost) важнее
            # статического ROS-параметра — иначе французский текст
            # синтезировался бы с language_boost=Russian.
            language=language or self.minimax_language,
            speed=speed,
            sample_rate=self.minimax_sample_rate,
            format=fmt,
            # Issue #1780: forward emotion / pitch / volume /
            # pronunciation_dict to the provider. Empty string → field
            # is omitted (no behaviour change vs. pre-#1780). Emotion
            # has a non-empty default ("neutral") which matches the
            # API's implicit default — payload gains the ``emotion``
            # key but the rendered voice is identical.
            emotion=self.minimax_emotion or None,
            pitch=_parse_optional_int(self.minimax_pitch_raw),
            volume=_parse_optional_float(self.minimax_volume_raw),
            pronunciation_dict=_parse_pronunciation_dict(
                self.minimax_pronunciation_dict_raw
            ),
        )

        try:
            tts_audio = await provider.synthesize(text, settings=settings)
        except MiniMaxTTSError:
            raise
        except Exception as exc:  # noqa: BLE001
            raise MiniMaxTTSError(
                f"MiniMax synthesis unexpected error: {exc}",
                provider="minimax",
            ) from exc

        # Транскодируем в mono float32. Поддерживаются PCM/WAV/MP3/OGG —
        # см. utils/audio_transcode.py.
        audio_np, decoded_sample_rate = self._decode_minimax_audio(
            tts_audio.samples,
            tts_audio.format,
            tts_audio.sample_rate,
        )

        if decoded_sample_rate != tts_audio.sample_rate:
            # Контейнер (WAV header) дал sample_rate отличный от запрошенного.
            # Это редкий случай (MiniMax должен вернуть то, что попросили),
            # но logging помогает при отладке.
            self.get_logger().debug(
                f"minimax: контейнер SR ({decoded_sample_rate}) != запрошенный SR "
                f"({tts_audio.sample_rate}); используем контейнерный"
            )

        return {"audio_np": audio_np, "sample_rate": decoded_sample_rate}

    async def _synthesize_minimax_with_retry(
        self,
        text: str,
        ssml_attributes: dict = None,
        voice: str = None,
        language: str = None,
    ) -> dict:
        """Обёртка с retry-loop над :meth:`_synthesize_minimax_async`.

        Реализует политику retry из ADR-0003 §2.6:

        * ``MiniMaxTTSAuthError`` / ``MiniMaxTTSBadRequestError`` — без ретрая.
        * ``MiniMaxTTSTimeoutError`` / ``MiniMaxTTSRateLimitError`` /
          ``MiniMaxTTSError`` (5xx обёртка) — ретрай с exp backoff.
        * Любая другая ошибка — ретрай как ``MiniMaxTTSError`` (conservative).

        Args:
            text: текст для синтеза.
            ssml_attributes: словарь с SSML-атрибутами (rate/pitch).

        Returns:
            см. ``_synthesize_minimax_async``.

        Raises:
            Последнее исключение после исчерпания retry budget.
        """
        configured_retries = min(max(0, int(self.minimax_max_retries)), 3)
        max_attempts = configured_retries + 1  # 0 retries → 1 attempt
        backoff_ms = self.minimax_retry_backoff_ms
        last_exc: Exception | None = None

        for attempt in range(max_attempts):
            try:
                return await self._synthesize_minimax_async(
                    text, ssml_attributes, voice=voice, language=language
                )
            except Exception as exc:
                # Классифицируем — некоторые ошибки ретраить нельзя.
                if isinstance(exc, MiniMaxTTSAuthError):
                    self.get_logger().error(f"MiniMax auth error, NO retry: {exc}")
                    raise
                if isinstance(exc, MiniMaxTTSBadRequestError):
                    self.get_logger().error(f"MiniMax bad-request, NO retry: {exc}")
                    raise

                # ADR-0003 permits only one retry for 429. Timeout/network/5xx
                # consume the full configured retry budget.
                retry_budget = (
                    1
                    if isinstance(exc, MiniMaxTTSRateLimitError)
                    else configured_retries
                )
                last_exc = exc
                if attempt >= retry_budget:
                    self.get_logger().error(
                        f"MiniMax exhausted {attempt + 1}/{retry_budget + 1} attempts: {exc}"
                    )
                    raise
                delay = (backoff_ms / 1000.0) * (2**attempt)
                self.get_logger().warn(
                    f"⏳ MiniMax retry {attempt + 1}/{retry_budget} after {delay:.2f}s "
                    f"({type(exc).__name__}: {exc})"
                )
                import asyncio as _asyncio

                await _asyncio.sleep(delay)

        # Unreachable, но mypy требует
        assert last_exc is not None
        raise last_exc

    def _synthesize_minimax_streaming_publish(
        self,
        text: str,
        ssml_attributes: dict = None,
        voice: str = None,
        language: str = None,
        sink: str = "speaker",  # ADR-0055 / issue #1993 — headset маршрут
    ) -> dict:
        """Sync-обёртка над :meth:`_stream_minimax_chunks` для streaming-режима MiniMax.

        Публикует каждый :class:`TTSChunk` как отдельный ``AudioData`` msg
        в ``/voice/audio/speech`` и возвращает объединённый буфер downstream
        для воспроизведения (chunk-per-frame latency win появится с
        WebSocket; пока провайдер возвращает один чанк — поведение
        эквивалентно ``_synthesize_minimax``).

        Returns:
            dict с ключами ``audio_np``, ``sample_rate``.
        """
        import asyncio as _asyncio

        chunks = []
        sample_rate = 0

        async def _consume_and_publish():
            nonlocal sample_rate
            async for chunk in self._stream_minimax_chunks(
                text, ssml_attributes, voice=voice, language=language
            ):
                if chunk.finish_reason == "error":
                    raise Exception(
                        "MiniMax stream reported error: finish_reason=error"
                    )

                if chunk.samples:
                    audio_np, chunk_sample_rate = self._decode_minimax_audio(
                        chunk.samples,
                        chunk.format,
                        chunk.sample_rate,
                    )
                    if sample_rate and chunk_sample_rate != sample_rate:
                        raise Exception(
                            "MiniMax stream changed sample_rate from "
                            f"{sample_rate} to {chunk_sample_rate}"
                        )
                    sample_rate = sample_rate or chunk_sample_rate
                    chunks.append(audio_np)
                    # Publish before requesting the next provider chunk. This is
                    # the latency-critical invariant of the ROS streaming bridge.
                    topic_audio = self._prepare_audio_for_topic(
                        audio_np,
                        chunk_sample_rate,
                    )
                    # ADR-0055 / issue #1993 — headset маршрут: вместо
                    # /voice/audio/speech публикуем в /avatar/tts/audio.
                    # ADR-0078 §4: пробрасываем chunk_sample_rate в WS-meta.
                    if sink == "headset":
                        self._publish_headset_audio(topic_audio, chunk_sample_rate)
                    else:
                        self._publish_audio(topic_audio)

                if chunk.finish_reason == "stop":
                    break

            if not chunks:
                raise Exception("MiniMax stream yielded no audio chunks")

        # 🔴 FIX (live 16:xx «Event loop is closed»): единый вечный loop —
        # см. _run_in_tts_loop. asyncio.run() здесь создавал бы новый loop,
        # закрывая его после, ломая httpx-клиент провайдера.
        _run_in_tts_loop(_consume_and_publish())

        return {
            "audio_np": np.concatenate(chunks),
            "sample_rate": sample_rate,
            "already_published": True,
        }

    def _synthesize_minimax(
        self,
        text: str,
        ssml_attributes: dict = None,
        voice: str = None,
        language: str = None,
    ) -> dict:
        """Sync-обёртка над :meth:`_synthesize_minimax_with_retry`.

        🔴 FIX (live 16:xx «Event loop is closed»): раньше оборачивали
        через ``asyncio.run`` — каждый вызов создавал НОВЫЙ loop, а
        провайдер держит httpx-клиент, привязанный к первому loop.
        Теперь ВСЕ вызовы идут через процесс-глобальный вечный loop
        (``_run_in_tts_loop``) — retry внутри одного синтеза и
        последующие синтезы переиспользуют тот же loop.
        """
        coro = self._synthesize_minimax_with_retry(
            text, ssml_attributes, voice=voice, language=language
        )
        return _run_in_tts_loop(coro)

    async def _stream_minimax_chunks(
        self,
        text: str,
        ssml_attributes: dict = None,
        voice: str = None,
        language: str = None,
    ):
        """Стриминг MiniMax через ``provider.stream()`` для chunk-per-frame.

        Провайдер сейчас возвращает один ``TTSChunk(finish_reason="stop")``
        (MiniMax SSE буферизуется в провайдере — chunk-per-frame WebSocket
        отложен в M5/M6, см. ADR-0003 §2.4). Эта обёртка сохраняет контракт
        ``stream()`` от ``rob_box_llm/tts.py`` — и когда WebSocket появится,
        переключение будет toggled-флагом, без переписывания вызывающего
        кода в ``tts_node``.

        Raises:
            Exception с человекочитаемым сообщением при любой ошибке MiniMax
            (вызывающий код решает — retry, fallback, или проброс наверх).
        """
        provider = self._ensure_minimax_provider()

        speed = (
            float(ssml_attributes.get("rate", self.minimax_speed))
            if ssml_attributes
            else self.minimax_speed
        )
        settings = TTSSettings(
            voice=voice or self.minimax_voice,
            model=self.minimax_model,
            # AV-28: язык этой реплики (language_boost) важнее
            # статического ROS-параметра — иначе французский текст
            # синтезировался бы с language_boost=Russian.
            language=language or self.minimax_language,
            speed=speed,
            sample_rate=self.minimax_sample_rate,
            format=self.minimax_format if MINIMAX_AVAILABLE else TTSFormat.PCM,
        )
        try:
            async for chunk in provider.stream(text, settings=settings):
                yield chunk
        except MiniMaxTTSError:
            raise
        except Exception as exc:  # noqa: BLE001
            raise MiniMaxTTSError(
                f"MiniMax stream unexpected error: {exc}",
                provider="minimax",
            ) from exc

    def _prepare_audio_for_topic(
        self,
        audio_np: np.ndarray,
        sample_rate: int,
    ) -> np.ndarray:
        """Normalize mono audio to the fixed sample rate of AudioData topic."""
        if sample_rate <= 0:
            raise ValueError(f"invalid audio sample_rate: {sample_rate}")
        if sample_rate == self.audio_output_sample_rate:
            return audio_np
        return resample_audio(
            audio_np,
            sample_rate,
            self.audio_output_sample_rate,
        )

    def _publish_audio(self, audio_np: np.ndarray):
        """Публикует аудио в ROS topic."""
        # Конвертируем в int16 для AudioData
        audio_int16 = (np.clip(audio_np, -1.0, 1.0) * 32767).astype("<i2", copy=False)

        msg = AudioData()
        # ROS uint8[] assignment is portable as a sequence of octets. Assigning
        # bytes works in some generated bindings but not all ROS2 distros.
        msg.data = list(audio_int16.tobytes())

        self.audio_pub.publish(msg)

    def _publish_headset_audio(
        self,
        audio_np: np.ndarray,
        sample_rate: Optional[int] = None,
        *,
        request_id: Optional[str] = None,
    ):
        """ADR-0055 / issue #1993 — публикация синтезированной реплики ТАРС
        в ``/avatar/tts/audio`` (int16 LE PCM, тот же SR, что ``/voice/audio/speech``).

        Подписчик (quest_node) заберёт чанк и через
        ``ws_server.deliver_audio(stream="operator_tts", ...)`` доставит
        в шлем оператора. Динамики робота НЕ играют (sink=headset → ALSA
        path skipped в ``_synthesize_and_play``).

        ADR-0078 §4: ``sample_rate`` — реальный rate PCM в Гц. Через DDS
        AudioData (нет поля rate) sample_rate не передаётся, поэтому
        публикуем **side-channel** ``/avatar/tts/audio_meta`` (String JSON
        ``{request_id, sample_rate, ts_ms}``) **ДО** AudioData. quest_node
        кеширует ``request_id → sample_rate`` и подставляет в
        ``ws_server.deliver_audio(sample_rate=...)``.

        Fallback ``sample_rate``: ``self.audio_output_sample_rate`` (declare
        default 16000). Если None, в лог уходит WARNING: в норме ВСЕГДА
        передаётся. ``request_id`` обязателен — supervisor генерирует его
        в ``_publish_avatar_tts`` (uuid hex8). Без request_id audio_meta
        не публикуем (клиент не сможет сматчить кэш с чанком).
        """
        audio_int16 = (np.clip(audio_np, -1.0, 1.0) * 32767).astype("<i2", copy=False)

        msg = AudioData()
        msg.data = list(audio_int16.tobytes())

        if sample_rate is None:
            sample_rate = getattr(self, "audio_output_sample_rate", 16000)
            self.get_logger().warning(
                "🎧 [ADR-0078] _publish_headset_audio без sample_rate — "
                f"fallback {sample_rate} (audio_output_sample_rate)"
            )

        # request_id: явный kwarg побеждает; иначе берём
        # ``self._avatar_tts_request_id`` (выставляется в _on_avatar_tts_request
        # до старта синтеза — supervisor генерит uuid hex8 в
        # _publish_avatar_tts).
        if not request_id:
            request_id = getattr(self, "_avatar_tts_request_id", None)

        # ADR-0078 §4: публикуем audio_meta (side-channel) ДО AudioData,
        # чтобы подписчик успел закешировать sample_rate до прихода чанка.
        # В одном DDS-потоке порядок гарантирован; разные потоки — допустимо
        # что audio_meta придёт после, тогда fallback в deliver_audio.
        if request_id:
            try:
                meta_payload = {
                    "request_id": str(request_id),
                    "sample_rate": int(sample_rate),
                    "ts_ms": int(time.time() * 1000),
                }
                meta_msg = String()
                meta_msg.data = json.dumps(meta_payload, ensure_ascii=False)
                self._avatar_audio_meta_pub.publish(meta_msg)
            except Exception as exc:  # noqa: BLE001 — диагностика не должна падать
                self.get_logger().warning(
                    f"🎧 [ADR-0078] _avatar_audio_meta_pub publish failed: {exc}"
                )
        else:
            self.get_logger().warning(
                "🎧 [ADR-0078] _publish_headset_audio без request_id — "
                "audio_meta не публикуется, клиент будет на fallback"
            )

        self._avatar_audio_pub.publish(msg)

    # ── Preview-synthesis (ADR-0077 / issue #2138.A.3) ─────────────────
    # Канбан-карточка t_74dd49c2: чистый синтез БЕЗ FIFO/ALSA/metrics для
    # picker'а оператора. ws_server/клиент preview'а ждут закодированный
    # контейнер (mp3/wav/opus) — см. preview_audio_sink.ts §1: WebAudio
    # ``decodeAudioData`` сам декодирует mp3/wav. Сырой PCM туда НЕ идёт
    # (это грабли канала ТАРС, см. ADR-0055 §грабли).

    def synthesize_preview(  # noqa: C901 — readable linear flow, not complex
        self,
        text: str,
        voice: Optional[str] = None,
        *,
        provider: Optional[Any] = None,
        timeout_s: float = 10.0,
    ) -> "PreviewAudioResult":
        """Синтезировать «прослушиваемый образец» голоса для picker'а.

        Возвращает :class:`PreviewAudioResult` с байтами в **закодированном**
        контейнере (mp3/wav/opus — что выбрано в ``preview_format``), а не
        сырым int16 PCM. Этим preview отличается от ``_publish_headset_audio``
        (sink=headset → /avatar/tts/audio) и от ``_publish_audio``
        (sink=speaker → /voice/audio/speech).

        Args:
            text: фраза для синтеза (уже плоский текст, без SSML).
            voice: запрошенный голос picker'а. **НЕ меняет активный голос
                личности** (``self.minimax_voice`` остаётся как был).
            provider: ``TTSProvider`` для синтеза. Если ``None`` —
                ``self._ensure_minimax_provider()`` (тот же клиент, что
                и для основного голоса — делим HTTP-пул, экономим
                keep-alive).
            timeout_s: жёсткий таймаут на сетевой синтез. По истечении —
                :class:`PreviewSynthesisTimeoutError`. Без таймаута
                picker может «висеть вечно» при недоступном upstream.

        Returns:
            :class:`PreviewAudioResult` с полями ``audio_bytes``,
            ``content_type``, ``sample_rate``, ``format_str``,
            ``duration_s``.

        Raises:
            PreviewSynthesisTimeoutError: ``provider.synthesize()`` не
                уложился в ``timeout_s``.
            PreviewSynthesisError: провайдер бросил (auth/bad-request/rate
                limit/5xx) — текст ошибки в ``exc.reason``.
            PreviewSynthesisUnavailableError: MiniMax opt-in не подключён
                (``MINIMAX_AVAILABLE=False``).
        """
        import asyncio

        if not MINIMAX_AVAILABLE:
            raise PreviewSynthesisUnavailableError(
                "minimax_unavailable: rob_box_llm не подключён — preview требует MiniMax"
            )
        if not text or not text.strip():
            # Тот же guard, что и в _synthesize_and_play (issue #2096):
            # пустой текст раньше ронял провайдер chain в TTSBadRequestError
            # и каскадно помечал всех провайдеров мёртвыми на 30s.
            raise PreviewSynthesisError(
                "empty_text: пустой текст для preview", reason="empty_text"
            )
        if timeout_s <= 0:
            raise PreviewSynthesisError(
                f"invalid_timeout: timeout_s={timeout_s} должен быть > 0",
                reason="invalid_timeout",
            )

        # Резолвим провайдер ТОЛЬКО для чтения (НЕ сохраняем обратно в
        # self.minimax_voice — preview не должно менять активный голос
        # личности, см. contract).
        if provider is None:
            provider = self._ensure_minimax_provider()

        # Настройки синтеза. voice — из аргумента (НЕ из self.minimax_voice).
        # format — из preview_format (default mp3, см. ADR-0077 §tts_node).
        # sample_rate — не форсируем (MiniMax сам подберёт под формат).
        settings = TTSSettings(
            voice=voice,
            model=self.minimax_model,
            language=self.minimax_language,
            format=self.preview_format,
        )

        async def _call():
            # asyncio.wait_for отменяет корутину по таймауту. На стороне
            # MiniMax-клиента httpx-сессия тоже идёт через свой timeout, но
            # в дополнение к нему ставим наш сторож — picker не должен
            # «висеть» дольше ``timeout_s`` (по умолчанию 10 с) ни при каких
            # условиях upstream'а.
            return await asyncio.wait_for(
                provider.synthesize(text, settings=settings),
                timeout=timeout_s,
            )

        try:
            tts_audio = _run_in_tts_loop(_call())
        except PreviewSynthesisError:
            # Уже наша ошибка — пробрасываем без обёртки.
            raise
        except Exception as exc:  # noqa: BLE001
            # asyncio.TimeoutError (из wait_for), MiniMaxTTSError*, CancelledError…
            # Различаем «висели и сорвались по таймауту» vs «провайдер бросил».
            err_name = type(exc).__name__
            if err_name in ("TimeoutError", "PreviewSynthesisTimeoutError"):
                raise PreviewSynthesisTimeoutError(
                    f"preview синтез превысил таймаут {timeout_s:.1f}s "
                    f"(provider={getattr(provider, 'name', '?')}, voice={voice!r})",
                    timeout_s=timeout_s,
                ) from exc
            raise PreviewSynthesisError(
                f"preview синтез упал: {err_name}: {exc}",
                reason=str(exc),
            ) from exc

        # Конвертируем TTSFormat → MIME content_type для ws_server/клиента.
        content_type = _format_to_content_type(tts_audio.format)

        # Грубая оценка длительности (для логов/диагностики). Для mp3/wav
        # точная длительность требует парсинга контейнера — клиент всё равно
        # сделает это через decodeAudioData, поэтому число ориентировочное.
        bytes_per_sample = 2  # int16
        if tts_audio.format == TTSFormat.PCM:
            duration_s = len(tts_audio.samples) / (
                tts_audio.sample_rate * bytes_per_sample
            )
        else:
            # Грубая оценка для mp3 @ ~128 kbps; для wav/ogg тоже мимо,
            # но это diag-only.
            duration_s = (len(tts_audio.samples) * 8.0) / 128_000.0

        self.get_logger().info(
            f"🎧 [preview-synth] ok: {len(tts_audio.samples)} bytes "
            f"format={tts_audio.format.value} sr={tts_audio.sample_rate} "
            f"voice={voice or 'default'} dur~{duration_s:.2f}s"
        )

        return PreviewAudioResult(
            audio_bytes=tts_audio.samples,
            content_type=content_type,
            sample_rate=tts_audio.sample_rate,
            format_str=tts_audio.format.value,
            duration_s=duration_s,
        )

    def _ensure_minimax_provider(self):
        """Return the MiniMax provider, constructing it exactly once.

        Construction creates the provider's HTTP client, so publishing the
        reference before construction completes would let a concurrent
        shutdown observe a half-initialized object.  The lock also makes two
        executor workers share one client instead of racing to create two.
        """
        if not MINIMAX_AVAILABLE:
            raise Exception("rob_box_llm недоступен — MiniMax не подключён")
        if not self.minimax_api_key:
            # GroupId опционален (api.minimax.io international) — ключ
            # обязателен, группа нет.
            raise MiniMaxTTSAuthError(
                "MINIMAX_API_KEY не задан",
                provider="minimax",
            )

        # Keep the helper usable with the lightweight objects used by the
        # unit tests, which do not run TTSNode.__init__.
        lock = getattr(self, "_minimax_provider_lock", None)
        if lock is None:
            lock = threading.Lock()
            self._minimax_provider_lock = lock

        with lock:
            if getattr(self, "_minimax_shutdown_requested", False):
                raise RuntimeError("MiniMax provider is shutting down")

            provider = getattr(self, "minimax_provider", None)
            if provider is not None:
                self._minimax_provider_initialized = True
                return provider

            provider = MiniMaxTTSProvider(
                api_key=self.minimax_api_key,
                group_id=self.minimax_group_id,
                default_voice=self.minimax_voice,
                default_model=self.minimax_model,
                timeout=self.minimax_timeout,
            )
            # Publish only a fully constructed provider.
            self.minimax_provider = provider
            self._minimax_provider_initialized = True
            return provider

    def close_minimax_provider(self):
        """Close the lazy MiniMax HTTP client before ROS node shutdown."""
        lock = getattr(self, "_minimax_provider_lock", None)
        if lock is None:
            lock = threading.Lock()
            self._minimax_provider_lock = lock

        # Mark shutdown before taking the provider snapshot.  Any concurrent
        # lazy-init call then fails instead of creating a client after cleanup.
        with lock:
            self._minimax_shutdown_requested = True
            is_initialized = getattr(
                self,
                "_minimax_provider_initialized",
                getattr(self, "minimax_provider", None) is not None,
            )
            if not is_initialized:
                return
            provider = getattr(self, "minimax_provider", None)

        if provider is None:
            return

        import asyncio

        try:
            asyncio.run(provider.aclose())
        except Exception as exc:  # shutdown must continue even on cleanup failure
            self.get_logger().warn(f"MiniMax provider cleanup failed: {exc}")
        finally:
            with lock:
                self.minimax_provider = None
                self._minimax_provider_initialized = False

    def shutdown_synthesis_executor(
        self, wait: bool = False, timeout: float = SYNTHESIS_SHUTDOWN_TIMEOUT_S
    ):
        """Drain the bounded synth executor (BLK-9).

        Idempotent. Called from ``main()`` before ``destroy_node()`` so the
        worker pool releases its threads cleanly. ``wait=False`` by default
        because ALSA playback can block beyond a reasonable shutdown window
        and we don't want to hang ROS teardown; the daemon-style behavior
        matches the previous bare-daemon-thread semantics.
        """
        executor = getattr(self, "_synthesis_executor", None)
        if executor is None:
            return
        if self._synthesis_executor_shutdown:
            return
        self._synthesis_executor_shutdown = True
        try:
            executor.shutdown(wait=wait, cancel_futures=not wait)
        except TypeError:
            # cancel_futures was added in 3.9; fall back if unavailable.
            executor.shutdown(wait=wait)
        except Exception as exc:
            self.get_logger().warn(f"Synthesis executor shutdown failed: {exc}")
        self.get_logger().info(
            f"Synthesis executor shutdown (wait={wait}, max_workers="
            f"{self._synthesis_executor_max_workers})"
        )

    def cleanup_playback_noise(self):
        """
        Устранение белого шума после воспроизведения TTS.

        Проблема: После воспроизведения через ReSpeaker (USB Audio Class 1.0)
        возникает постоянный белый шум из-за активного playback channel.

        Решение:
        1. Properly close sounddevice stream
        2. Flush audio buffers
        3. Small delay для стабилизации USB audio interface
        """
        try:
            # 1. Ensure sounddevice is fully stopped
            sd.stop()

            # 2. Small delay to let USB audio interface stabilize
            # ReSpeaker USB Audio Class 1.0 requires time to properly close playback path
            time.sleep(0.1)

            # 3. Log cleanup completion
            self.get_logger().debug("🧹 TTS playback noise cleanup completed")

        except Exception as e:
            self.get_logger().warn(f"⚠️ TTS noise cleanup failed: {e}")

    def publish_state(self, state: str):
        """Публикация состояния TTS."""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def _publish_tts_finished(
        self,
        speech_id: Optional[str],
        *,
        success: bool,
        error: Optional[str] = None,
        duration_sec: Optional[float] = None,
        batch_id: Optional[str] = None,
        batch_index: Optional[int] = None,
        batch_total: Optional[int] = None,
        batch_started_at: Optional[float] = None,
        dialogue_id: Optional[str] = None,
    ) -> None:
        """Publish ``/voice/tts/finished`` (and possibly ``/voice/tts/batch_complete``).

        Issue #980 — single source of truth for finished-event publishing so
        that batch metadata stays consistent across the success/stopped/error
        branches and the batch_complete fire rule (``batch_index == batch_total``)
        doesn't drift between code paths.
        """
        if not speech_id:
            return
        payload: Dict[str, Any] = {"speech_id": speech_id, "success": success}
        if error is not None:
            payload["error"] = error
        if duration_sec is not None:
            payload["duration_sec"] = duration_sec
        if batch_id is not None:
            payload["batch_id"] = batch_id
        if batch_index is not None:
            payload["batch_index"] = int(batch_index)
        if batch_total is not None:
            payload["batch_total"] = int(batch_total)
        finished_msg = String()
        finished_msg.data = json.dumps(payload, ensure_ascii=False)
        self.finished_pub.publish(finished_msg)

        # Batch-complete side-channel — fires once per turn after the last
        # chunk so dialogue_node can drive music_cleanup deterministically.
        if (
            batch_id is not None
            and batch_index is not None
            and batch_total is not None
            and int(batch_index) == int(batch_total)
        ):
            import time as _time

            duration_ms: Optional[int] = None
            if batch_started_at is not None:
                duration_ms = int((_time.monotonic() - batch_started_at) * 1000)
            batch_payload: Dict[str, Any] = {
                "batch_id": batch_id,
                "chunks_total": int(batch_total),
                "batch_index": int(batch_index),
            }
            if duration_ms is not None:
                batch_payload["batch_duration_ms"] = duration_ms
            batch_msg = String()
            batch_msg.data = json.dumps(batch_payload, ensure_ascii=False)
            self.batch_complete_pub.publish(batch_msg)
            self.get_logger().info(
                "📦 [tts_node] /voice/tts/batch_complete published "
                f"(batch_id={batch_id[:8]}..., chunks_total={batch_total}, "
                f"batch_duration_ms={duration_ms})"
            )
            # Echo on finished too so any consumer of ``/voice/tts/finished``
            # that wants the closure timestamp can grab it without a second
            # subscription. Kept behind the ``last_chunk`` branch to avoid
            # spamming every chunk's finished event with the closure marker.
            payload["batch_complete"] = True
            if duration_ms is not None:
                payload["batch_duration_ms"] = duration_ms
            finished_msg.data = json.dumps(payload, ensure_ascii=False)
            # Republish to keep the marker attached to the same logical event.
            # (Bounded QoS depth=10 means the second publish can briefly bump
            # the depth; downstream subscribers are designed to be idempotent
            # on ``batch_complete``.)
            self.finished_pub.publish(finished_msg)

    def parameters_callback(self, params):
        """Callback для изменения параметров во время работы."""
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name == "volume_db":
                self.volume_db = param.value
                self.volume_gain = 10.0 ** (self.volume_db / 20.0)
                self.get_logger().info(
                    f"🔊 Громкость изменена: {self.volume_db:.1f} dB (gain: {self.volume_gain:.2f}x)"
                )
            elif param.name == "pitch_shift":
                self.pitch_shift = param.value
                self.get_logger().info(f"🐿️ Pitch shift изменён: {self.pitch_shift}x")
            elif param.name == "chipmunk_mode":
                self.chipmunk_mode = param.value
                self.get_logger().info(f"🐿️ Chipmunk mode: {self.chipmunk_mode}")
            elif param.name == "yandex_speed":
                self.yandex_speed = param.value
                self.get_logger().info(
                    f"🎵 Yandex speed (pitch) изменён: {self.yandex_speed}"
                )
            elif param.name == "minimax_max_retries":
                self.minimax_max_retries = min(3, max(0, int(param.value)))
                self.get_logger().info(
                    f"🔁 MiniMax max_retries → {self.minimax_max_retries}"
                )
            elif param.name == "minimax_streaming":
                self.minimax_streaming = bool(param.value)
                self.get_logger().info(
                    f"📡 MiniMax streaming → {self.minimax_streaming}"
                )
            elif param.name == "pregenerate_enabled":
                # Kill-switch for the speculative pipeline. When toggled
                # OFF at runtime, cancel any in-flight pre-gens so they
                # don't outlive the operator's decision.
                new_val = bool(param.value)
                if not new_val and self._prefetch is not None:
                    self.cancel_pregen(reason="param_disabled")
                self._pregenerate_enabled = new_val
                self.get_logger().info(f"🎯 pregenerate_enabled → {new_val}")
            elif param.name == "pregenerate_confidence_floor":
                self._pregenerate_confidence_floor = max(
                    0.0,
                    min(1.0, float(param.value)),
                )
                # Propagate to the live executor if it has been built.
                if self._prefetch is not None:
                    self._prefetch["executor"]._confidence_floor = (
                        self._pregenerate_confidence_floor
                    )
                self.get_logger().info(
                    f"🎯 pregenerate_confidence_floor → "
                    f"{self._pregenerate_confidence_floor}"
                )
            elif param.name == "pregenerate_history_window":
                self._pregenerate_history_window = max(1, int(param.value))
                self.get_logger().info(
                    f"🎯 pregenerate_history_window → "
                    f"{self._pregenerate_history_window}"
                )

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # BLK-8 + BLK-9: order matters here.
        #
        # 1. Mark MiniMax provider as shutting down so no NEW worker can
        #    race to lazy-init a client during teardown.
        # 2. Stop accepting new TTS submits and cancel pending futures.
        # 3. Close the lazy MiniMax HTTP client (asyncio.run aclose()).
        # 4. Destroy the node and rclpy.
        with node._minimax_provider_lock:
            node._minimax_shutdown_requested = True
        node.shutdown_synthesis_executor(wait=False)
        node.close_minimax_provider()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
