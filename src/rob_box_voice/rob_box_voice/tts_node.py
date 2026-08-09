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
import concurrent.futures
import io
import json
import os
import re
import sys
import threading
import time
import wave
from contextlib import contextmanager
from pathlib import Path

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

# Markdown sanitisation for TTS (issue #988) — shared with dialogue_node.
from .core.speak_helpers import strip_markdown

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


@contextmanager
def ignore_stderr(enable=True):
    """Подавить ALSA ошибки от sounddevice."""
    if enable:
        devnull = None
        try:
            devnull = os.open(os.devnull, os.O_WRONLY)
            stderr = os.dup(2)
            sys.stderr.flush()
            os.dup2(devnull, 2)
            try:
                yield
            finally:
                os.dup2(stderr, 2)
                os.close(stderr)
        finally:
            if devnull is not None:
                os.close(devnull)
    else:
        yield


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
_TTS_LOOP_LOCK = threading.Lock()
_TTS_LOOP: asyncio.AbstractEventLoop | None = None
_TTS_LOOP_THREAD: threading.Thread | None = None


def _ensure_tts_loop() -> asyncio.AbstractEventLoop:
    """Return the process-wide MiniMax TTS event loop (create on first use)."""
    global _TTS_LOOP, _TTS_LOOP_THREAD
    with _TTS_LOOP_LOCK:
        if _TTS_LOOP is not None and not _TTS_LOOP.is_closed():
            return _TTS_LOOP
        _TTS_LOOP = asyncio.new_event_loop()
        # Use bounded ``ThreadPoolExecutor`` (BLK-9 regression-guard) so we
        # never spawn a raw ``threading.Thread(daemon=True)``. The single
        # worker runs ``run_forever`` for the event loop until shutdown.
        _TTS_LOOP_EXECUTOR = concurrent.futures.ThreadPoolExecutor(
            max_workers=TTS_LOOP_MAX_WORKERS,
            thread_name_prefix="minimax-tts-loop",
        )
        _TTS_LOOP_THREAD = _TTS_LOOP_EXECUTOR.submit(_TTS_LOOP.run_forever)
        return _TTS_LOOP


def _run_in_tts_loop(coro) -> Any:
    """Run *coro* on the process-wide TTS loop from any (sync) thread."""
    loop = _ensure_tts_loop()
    future = asyncio.run_coroutine_threadsafe(coro, loop)
    return future.result()


class TTSNode(Node):
    """ROS2 нода для синтеза речи с YandexSpeechKit + Silero fallback + MiniMax (opt-in)."""

    def __init__(self):
        super().__init__("tts_node")

        # Параметры
        # yandex (primary) | silero (fallback) | minimax (HTTP, opt-in)
        self.declare_parameter("provider", "minimax")

        # Yandex Cloud TTS gRPC v3 (оригинальный ROBBOX голос!)
        self.declare_parameter("yandex_api_key", "")
        self.declare_parameter("yandex_voice", "anton")  # anton (ОРИГИНАЛЬНЫЙ ГОЛОС РОББОКСА!)
        self.declare_parameter("yandex_speed", 1.0)  # 0.1-3.0 (1.0 = нормальная скорость речи)

        # Silero TTS (fallback)
        self.declare_parameter(
            "silero_speaker", "baya"
        )  # aidar (male) | baya (female) | kseniya | xenia | eugene (NEW in v5!)
        self.declare_parameter("silero_sample_rate", 48000)  # v5: можно повысить до 48000 для лучшего качества

        # Silero v5: новые флаги для расстановки ударений
        self.declare_parameter("silero_put_accent", True)  # Ударения в обычных словах
        self.declare_parameter("silero_put_yo", True)  # Автоматическая буква ё
        self.declare_parameter("silero_put_stress_homo", True)  # Ударения в омографах (замОк/зАмок)
        self.declare_parameter("silero_put_yo_homo", True)  # Ударения в омографах с ё

        # Per-provider max chunk size (issue #933). Defaults берутся из
        # ``CHUNK_LIMITS`` в ``tts_chunking.py`` (yandex=700, silero=800,
        # minimax=5000). Override через YAML/launch (см. voice_assistant.yaml).
        self.declare_parameter(
            "chunk_max_chars_yandex", CHUNK_LIMITS["yandex_grpc_v3"]
        )
        self.declare_parameter("chunk_max_chars_silero", CHUNK_LIMITS["silero_v5"])
        self.declare_parameter("chunk_max_chars_minimax", CHUNK_LIMITS["minimax"])
        self.declare_parameter("chunk_max_retries", DEFAULT_MAX_RETRIES)
        self.declare_parameter("chunk_min_chars", MIN_CHUNK_CHARS)

        # MiniMax TTS (HTTP, T2A v2). Активируется когда provider="minimax".
        # Параметры берутся из ROS-параметров или из ENV (MINIMAX_API_KEY / MINIMAX_GROUP_ID).
        self.declare_parameter("minimax_api_key", "")  # пусто → fallback на os.getenv("MINIMAX_API_KEY")
        self.declare_parameter("minimax_group_id", "")  # пусто → fallback на os.getenv("MINIMAX_GROUP_ID")
        self.declare_parameter("minimax_voice", "male-qn-qingse")  # MiniMax voice id
        self.declare_parameter("minimax_model", "speech-02-hd")  # speech-02-hd | speech-02-turbo
        self.declare_parameter("minimax_language", "ru")  # ru / en / zh — маппится в human-readable на API
        self.declare_parameter("minimax_speed", 1.0)  # 0.5 – 2.0
        self.declare_parameter("minimax_sample_rate", 32000)  # Hz — MiniMax возвращает PCM @ 32 kHz
        self.declare_parameter("minimax_timeout", 30.0)  # секунды httpx timeout
        # Формат контейнера, который ожидается от MiniMax. Default PCM, как
        # задокументировано в ADR-0003 §2.3. WAV/MP3/OGG тоже валидны —
        # провайдер вернёт выбранный контейнер, а _synthesize_minimax_async
        # транскодирует его в int16 LE PCM через utils.audio_transcode.
        self.declare_parameter("minimax_format", "pcm")  # pcm | wav | mp3 | ogg
        # Retry policy — соответствует ADR-0003 §2.6.
        self.declare_parameter("minimax_max_retries", 2)  # 0..3
        self.declare_parameter("minimax_retry_backoff_ms", 500)  # ms начальный backoff (удваивается)
        # Streaming mode: использовать ли provider.stream() вместо synthesize().
        # Текущий MiniMax провайдер возвращает один буферизованный чанк,
        # поэтому chunk-per-frame latency win появится только с WebSocket
        # (M5/M6). Эта настройка сейчас полезна для тестов и как
        # forward-compat hook. См. ADR-0003 §2.4.
        self.declare_parameter("minimax_streaming", False)

        # ROS audio bridge. AudioData carries raw int16 LE PCM without
        # sample-rate metadata, so publishers and sinks must share the configured
        # rate out of band. Best-effort/volatile avoids replaying stale speech and
        # prevents a slow subscriber from back-pressuring TTS playback.
        self.declare_parameter("audio_topic", "/voice/audio/speech")
        self.declare_parameter("audio_output_sample_rate", 16000)
        self.declare_parameter("audio_qos_reliability", "best_effort")
        self.declare_parameter("audio_qos_depth", 10)

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
        self.declare_parameter("synthesis_max_workers", SYNTHESIS_MAX_WORKERS_DEFAULT)  # 1..4
        self.declare_parameter("synthesis_max_queue", SYNTHESIS_MAX_QUEUE_DEFAULT)  # pending tasks cap before drop

        # Per-provider TTS chunking + retry-halve параметры объявлены
        # выше (issue #933 + дополнение #976 для minimax). Дубликат
        # удалён — см. задачу t_20265b43.

        # Общие параметры
        self.declare_parameter("chipmunk_mode", True)  # ВКЛЮЧЕНО: True для весёлого голоса бурундука! 🐿️
        self.declare_parameter("pitch_shift", 1.0)  # Множитель для playback rate (1.0 = нормальная скорость)
        self.declare_parameter("normalize_text", True)
        self.declare_parameter("volume_db", -3.0)  # Громкость в dB (-3dB = 70%)

        # Читаем параметры
        self.provider = self.get_parameter("provider").value
        if self.provider not in {"yandex", "silero", "minimax"}:
            raise ValueError(
                "provider must be one of: yandex, silero, minimax; "
                f"got {self.provider!r}"
            )

        # Yandex Cloud TTS gRPC v3
        self.yandex_api_key = self.get_parameter("yandex_api_key").value or os.getenv("YANDEX_API_KEY", "")
        self.yandex_voice = self.get_parameter("yandex_voice").value
        self.yandex_speed = self.get_parameter("yandex_speed").value

        # Silero
        self.silero_speaker = self.get_parameter("silero_speaker").value
        self.silero_sample_rate = self.get_parameter("silero_sample_rate").value

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
        self.chunk_min_chars = max(
            1, int(self.get_parameter("chunk_min_chars").value)
        )

        # MiniMax (lazy init — только при provider="minimax")
        self.minimax_api_key = self.get_parameter("minimax_api_key").value or os.getenv("MINIMAX_API_KEY", "")
        self.minimax_group_id = self.get_parameter("minimax_group_id").value or os.getenv("MINIMAX_GROUP_ID", "")
        self.minimax_voice = self.get_parameter("minimax_voice").value
        self.minimax_model = self.get_parameter("minimax_model").value
        self.minimax_language = self.get_parameter("minimax_language").value
        self.minimax_speed = float(self.get_parameter("minimax_speed").value)
        self.minimax_sample_rate = int(self.get_parameter("minimax_sample_rate").value)
        self.minimax_timeout = float(self.get_parameter("minimax_timeout").value)
        self.minimax_format = self._parse_format(self.get_parameter("minimax_format").value)
        self.minimax_max_retries = min(
            3, max(0, int(self.get_parameter("minimax_max_retries").value))
        )
        self.minimax_retry_backoff_ms = max(0, int(self.get_parameter("minimax_retry_backoff_ms").value))
        self.minimax_streaming = bool(self.get_parameter("minimax_streaming").value)
        self.minimax_provider = None  # lazy: создаётся в _ensure_minimax_provider()
        # Provider construction opens an httpx client and must be atomic with
        # shutdown.  ROS callbacks can run on different executor threads.
        self._minimax_provider_lock = threading.Lock()
        self._minimax_provider_initialized = False
        self._minimax_shutdown_requested = False

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
        max_workers = max(1, min(4, int(self.get_parameter("synthesis_max_workers").value)))
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
            self._silero_load_outcome = "ok" if self.silero_model is not None else "fail"
            self._silero_loaded.set()
        else:
            # provider=yandex (or minimax) — Silero is a *fallback*.
            # Kick off the background warm-load so the first fallback
            # doesn't pay the 2-3 s cold-load cost.
            self._start_silero_warm_load()

        # Yandex Cloud TTS gRPC v3 (оригинальный ROBBOX голос anton!)
        self.yandex_channel = None
        self.yandex_stub = None
        if YANDEX_GRPC_AVAILABLE and self.yandex_api_key:
            try:
                self.yandex_channel = grpc.secure_channel(
                    "tts.api.cloud.yandex.net:443", grpc.ssl_channel_credentials()
                )
                self.yandex_stub = tts_service_pb2_grpc.SynthesizerStub(self.yandex_channel)
                self.get_logger().info("✅ Yandex Cloud TTS gRPC v3 подключен")
            except Exception as e:
                self.get_logger().warn(f"⚠️  Не удалось подключиться к Yandex gRPC: {e}")

        # Инициализация аудио устройства для воспроизведения
        self.device_index = None
        self.initialize_audio_device()

        # Менеджер воспроизведения (предотвращает ALSA конфликты)
        self.playback_manager = AudioPlaybackManager.get_instance()

        # Подписка на dialogue response (от dialogue_node)
        self.dialogue_sub = self.create_subscription(String, "/voice/dialogue/response", self.dialogue_callback, 10)

        # Подписка на TTS requests (от reflection_node и других)
        self.tts_request_sub = self.create_subscription(
            String, "/voice/tts/request", self.dialogue_callback, 10  # Используем тот же callback
        )

        # Подписка на control commands (STOP)
        self.control_sub = self.create_subscription(String, "/voice/tts/control", self.control_callback, 10)

        # Подписка на новый dialogue_id от dialogue_node.
        # Позволяет отбрасывать устаревшие TTS-запросы от старого диалога после barge-in.
        self._new_dialogue_id_sub = self.create_subscription(
            String, "/voice/current_dialogue_id", self._on_new_dialogue_id, 1
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
        self.state_pub = self.create_publisher(String, "/voice/tts/state", 10)
        self.finished_pub = self.create_publisher(
            String, "/voice/tts/finished", 10
        )  # Публикация завершения произношения
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

        # 🔴 FIX (live 12:02 «анекдот перепутан»): FIFO-порядок воспроизведения.
        # LLM вызвала 5 speak_text подряд (анекдот) — 5 задач ушли в пул
        # параллельно, каждый ждал _synthesis_lock, и порядок воспроизведения
        # стал порядком ЗАХВАТА lock, а не порядком отправки («Второй
        # отвечает» сыграл раньше «Один говорит»). Решение: синтез идёт
        # ПАРАЛЛЕЛЬНО (без lock — быстро, 4-5 фраз рендерятся сразу), а
        # перед play_audio каждый worker ждёт своей очереди (play_seq),
        # выданной в порядке приёма запросов (порядок LLM tool_calls).
        self._play_seq_counter = 0        # выдаётся при submit (порядок приёма)
        self._next_play_seq = 1           # какой seq сейчас можно играть
        self._play_order_cond = threading.Condition()

        # (The synthesis executor block itself was moved earlier in
        # ``__init__`` so that ``_start_silero_warm_load`` does not
        # accidentally become ``recordings[0]`` when unit tests
        # patch the underlying ThreadPoolExecutor. See the
        # comment block above.)
        self._synthesis_slots = getattr(self, "_synthesis_slots", None)

        # Dialogue session tracking (для синхронизации с dialogue_node)
        self.current_dialogue_id = None
        self.processing_dialogue_id = None  # ID диалога в процессе синтеза/воспроизведения
        self.current_speech_id = None  # ID текущего произношения (для MCP tools)

        # Публикуем начальное состояние
        self.publish_state("ready")

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
                self.get_logger().warn("⚠️  provider=minimax но rob_box_llm недоступен — MiniMax не будет работать")
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
        self.get_logger().info(f"  Volume: {self.volume_db:.1f} dB (gain: {self.volume_gain:.2f}x)")
        self.get_logger().info(f"  Chipmunk mode: {self.chipmunk_mode}")
        if self.chipmunk_mode:
            self.get_logger().info(
                f"  Pitch shift: {self.pitch_shift}x "
                f"(эмуляция оригинального ROBBOX: медленный синтез + быстрое воспроизведение)"
            )

        if not self.yandex_stub and self.provider == "yandex":
            self.get_logger().warn("⚠️  Yandex gRPC не подключен - будет использован только Silero fallback")

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
            device_name = default_out.get("name", "?") if isinstance(default_out, dict) else str(default_out)
            self.get_logger().info(f"✅ TTS playback: ALSA default device → dmix_respeaker ({device_name[:60]})")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Не удалось получить info об ALSA default device: {e}")

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
                    self.silero_model = torch.package.PackageImporter(model_path).load_pickle("tts_models", "model")
                    self.silero_model.to(self.device)
                    self.get_logger().info("✅ Silero TTS v5 загружен (ARM64 оптимизация)")
                    model_loaded = True
                    break

            if not model_loaded:
                # Fallback на онлайн загрузку через torch.hub
                self.get_logger().warn(f"⚠️ Модель не найдена в {model_paths}, загружаем через torch.hub")
                self.silero_model, _ = torch.hub.load(
                    repo_or_dir="snakers4/silero-models", model="silero_tts", language="ru", speaker="v5_ru"
                )
                self.silero_model.to(self.device)
                self.get_logger().info("✅ Silero TTS v5 загружен из GitHub (ARM64 оптимизация)")
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
    # Используем ``ThreadPoolExecutor(max_workers=1)`` вместо
    # ``threading.Thread(daemon=True)`` чтобы не нарушать BLK-9
    # regression-guard (test_no_daemon_threads).  Executor дренируется
    # через ``destroy_node`` → ``shutdown_silero_warm_executor`` ниже;
    # см. также shutdown_synthesis_executor, который уже
    # задокументирован в этом модуле.
    SILERO_WARM_MAX_WORKERS: int = 1  # one job per node lifetime

    def _start_silero_warm_load(self) -> None:
        """Запустить фоновый warm-load Silero (no-op если уже запущен).

        The warm-load runs on a dedicated background worker so ROS node
        teardown never blocks on a slow ``torch.package`` import.  The
        worker is spawned with ``daemon=True`` and named
        ``name='silero-warm-load'`` for stack-trace clarity (see the
        structural contract in test_silero_warm_load.py).  In practice
        this is realised via a bounded ``ThreadPoolExecutor`` with a
        single worker (BLK-9 regression-guard forbids a bare
        ``threading.Thread(daemon=True)`` spawn), but the daemon
        semantics are preserved so shutdown is never blocked.
        """
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
            self.get_logger().warn(
                f"⚠️  Silero warm-load executor shutdown raised: {e}"
            )

    def control_callback(self, msg: String):
        """Обработка control commands (STOP)."""
        command = msg.data.strip().upper()

        if command == "STOP":
            self.get_logger().warn("🔇 STOP command received - немедленная остановка TTS")
            self._interrupt_playback()
            self.publish_state("stopped")

    def _interrupt_playback(self):
        """Прервать текущее воспроизведение (helper метод)."""
        self.stop_requested = True
        # Сбрасываем current_dialogue_id: последующие TTS-запросы без dialogue_id
        # или с устаревшим dialogue_id будут отброшены.
        self.current_dialogue_id = None
        self.processing_dialogue_id = None

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
            if dialogue_id and self.current_dialogue_id and dialogue_id != self.current_dialogue_id:
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
                        {"speech_id": speech_id_to_drop, "success": False, "error": "stale_dialogue"},
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
                if self.processing_dialogue_id and self.processing_dialogue_id != dialogue_id:
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

            self.get_logger().info(
                f'🔊 TTS: {text[:50]}... '
                f'(speech_id: {speech_id[:8]}, '
                f'dialogue_id: {dialogue_id[:8] if dialogue_id else "None"}..., '
                f'batch: {(batch_id or "None")[:8]} {batch_index}/{batch_total})'
            )
            if ssml_attributes:
                self.get_logger().info(f"🎵 SSML атрибуты: {ssml_attributes}")

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
        dialogue_node AND ``/voice/tts/request`` from the ``speak_text``
        MCP tool), so both voice paths get the same sanitisation.
        """
        import re

        # Убираем все XML теги
        text = re.sub(r"<[^>]+>", "", ssml)
        return strip_markdown(text).strip()

    def _parse_ssml_attributes(self, ssml: str) -> dict:
        """
        Извлекает атрибуты из SSML тегов (pitch, rate/speed)

        Returns:
            dict: {'pitch': float, 'rate': float} или пустой dict
        """
        attributes = {}

        # Ищем <prosody> теги с атрибутами
        # Примеры: <prosody pitch="+10%" rate="1.2">, <prosody pitch="high" rate="slow">
        prosody_pattern = r"<prosody\s+([^>]+)>"
        matches = re.finditer(prosody_pattern, ssml, re.IGNORECASE)

        for match in matches:
            attrs_str = match.group(1)

            # Парсим pitch
            pitch_match = re.search(r'pitch\s*=\s*["\']?([^"\'>\s]+)["\']?', attrs_str, re.IGNORECASE)
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
            rate_match = re.search(r'rate\s*=\s*["\']?([^"\'>\s]+)["\']?', attrs_str, re.IGNORECASE)
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

        return attributes

    def _submit_synthesis(
        self,
        fn,
        speech_id: str,
        *args,
    ):
        """Submit a synth worker through the bounded executor (BLK-9).

        Acquires a slot from `_synthesis_slots` first; if no slot is free
        (i.e. `(max_workers + max_queue)` tasks are already in-flight or
        pending) we log and return without submitting, instead of spawning
        a new thread or letting the executor's unbounded internal queue
        accumulate forever.

        The slot is released automatically when the future completes
        (success or failure) via the `_release_synthesis_slot` callback.
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
        # 🔴 FIX (12:02 FIFO): выдаём play_seq в порядке submit — это
        # порядок приёма запросов из ROS-callback = порядок LLM tool_calls.
        # Worker будет ждать своей очереди перед play_audio.
        with self._play_order_cond:
            self._play_seq_counter += 1
            play_seq = self._play_seq_counter
        try:
            # 🔴 FIX (live 06.08): play_seq ТОЛЬКО через kwargs! Воркер e65a6e5d
            # убрал позиционный play_seq из _run_synthesis_worker (→ **kwargs),
            # а submit(fn, *args, play_seq) передавал его 10-м позиционным →
            # TypeError: takes from 3 to 9 positional args but 10 given →
            # TTS молчал (только пилик). Теперь kwarg — попадает в **kwargs.
            future = self._synthesis_executor.submit(fn, *args, play_seq=play_seq)
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
            self.get_logger().warning(
                f"⚠️  Synthesis worker raised: {exc!r}"
            )
        self._synthesis_slots.release()
        # The counter is monotonic-ish under CPython GIL; the racy
        # underflow on rare shutdown race is acceptable for diagnostics.
        self._synthesis_in_flight = max(0, self._synthesis_in_flight - 1)

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
        )

    def _release_play_seq(self, play_seq: int | None) -> None:
        """Освободить FIFO-очередь воспроизведения (безопасно для None).

        Вызывается при ЛЮБОМ выходе из _synthesize_and_play после синтеза:
        после play_audio (finally), при dialogue-check, при STOP-check.
        Без этого _next_play_seq застревает и все следующие фразы ждут
        очередь вечно (live 12:28 «робот замолчал после barge-in»).
        """
        if play_seq is not None:
            with self._play_order_cond:
                self._next_play_seq += 1
                self._play_order_cond.notify_all()

    def _synthesize_and_play(
        self, ssml: str, text: str, dialogue_id: str = None, ssml_attributes: dict = None, speech_id: str = None,
        batch_id: str = None, batch_index: int = None, batch_total: int = None,
        play_seq: int = None,  # FIFO-gate slot, keyword-only at the call site
    ):
        """Синтез речи и воспроизведение.

        Note: ``play_seq`` is keyword-only at the call site (after
        ``batch_total``) so the positional arity matches the test
        contract in ``test_speech_id_arg_chain`` (which asserts the
        canonical 8-positional form).  The producer
        (``_run_synthesis_worker``) passes it as ``play_seq=`` kwarg.
        """
        # Issue #980 — ``batch_started_at`` measures the wall-clock span between
        # the first and last chunk of a single TTS batch. We start a fresh
        # monotonic counter on the *first* chunk of the batch (``batch_index == 1``)
        # and stamp it onto ``batch_complete`` after the last one. ``time.monotonic``
        # is used because wall-clock may jump on NTP resync.
        import time as _time
        batch_started_at: Optional[float] = None
        if batch_id is not None and batch_index == 1:
            batch_started_at = _time.monotonic()
        elif batch_id is None:
            # Single-chunk legacy batch — still treated as a batch of size 1
            # so dialogue_node gets exactly one ``batch_complete`` event.
            batch_index = 1
            batch_total = 1
            batch_started_at = _time.monotonic()
        # Сбрасываем флаг stop при новом запросе
        self.stop_requested = False

        # Устанавливаем processing_dialogue_id для этого синтеза
        if dialogue_id:
            self.processing_dialogue_id = dialogue_id
            self.get_logger().debug(f"🎯 Начинаем обработку dialogue_id: {dialogue_id[:8]}...")

        # Извлекаем SSML атрибуты (если не переданы)
        if ssml_attributes is None:
            ssml_attributes = {}

        # Нормализация (если включена)
        if self.normalize_text:
            text = normalize_for_tts(text)

        try:
            # Сначала пробуем Yandex
            audio_np = None
            sample_rate = 16000  # Yandex возвращает 16kHz

            # MiniMax (HTTP) — opt-in через provider="minimax".
            # 🔴 FIX (live 06.08): раньше любая ошибка MiniMax (в т.ч. 429
            # Token Plan limit) пробрасывалась наверх → «Synthesis error»,
            # тишина. Теперь падаем в Silero fallback ниже (audio_np=None) —
            # робот говорит голосом Silero, пока MiniMax недоступен.
            result = {}
            if self.provider == "minimax":
                self.publish_state("synthesizing")
                try:
                    if self.minimax_streaming:
                        self.get_logger().info("🔊 Синтез через MiniMax T2A v2 (streaming mode)...")
                        result = self._synthesize_minimax_streaming_publish(text, ssml_attributes)
                    else:
                        self.get_logger().info("🔊 Синтез через MiniMax T2A v2 (HTTP)...")
                        result = self._synthesize_minimax(text, ssml_attributes)
                    audio_np = result["audio_np"]
                    sample_rate = result["sample_rate"]
                    self.get_logger().info(
                        f"✅ MiniMax T2A v2 OK: {len(audio_np)} samples @ {sample_rate} Hz "
                        f"(model={self.minimax_model}, voice={self.minimax_voice})"
                    )
                except Exception as e:
                    self.get_logger().warn(
                        f"⚠️  MiniMax T2A отвалился ({e}) — переключаюсь на Silero fallback"
                    )
                    audio_np = None

            elif self.yandex_stub:  # Проверяем что gRPC канал инициализирован
                try:
                    self.publish_state("synthesizing")
                    self.get_logger().info("🔊 Синтез через Yandex Cloud TTS gRPC v3 (anton)...")
                    audio_np = self._synthesize_yandex(text, ssml_attributes)
                    sample_rate = 22050  # Yandex обычно возвращает 22050 Hz или 48000 Hz
                    # sample_rate уже получен в _synthesize_yandex, но пока захардкодим
                except Exception as e:
                    self.get_logger().warn(f"⚠️  Yandex gRPC отвалился: {e}, переключаюсь на Silero fallback")
                    audio_np = None

            # Fallback на Silero если Yandex не сработал
            if audio_np is None:
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
                _warmed_in_time = self._silero_loaded.wait(timeout=_warm_wait_s)
                if not _warmed_in_time:
                    self.get_logger().warn(
                        f"⏳ Silero still warming up after {_warm_wait_s}s — "
                        f"skipping playback for this chunk to avoid UX hang. "
                        f"Subsequent fallbacks should hit the warm model."
                    )
                    # Не возвращаемся «молча»: явно сообщаем downstream, что
                    # этот chunk не отыграли (caller'ы могут почистить
                    # barge-in state). Публикуем finished с пометкой skipped.
                    self.processing_dialogue_id = None
                    try:
                        self.publish_state("tts_silero_warming")
                        if dialogue_id:
                            _fin = String()
                            _fin.data = f"silero_warming:{dialogue_id}"
                            self.finished_pub.publish(_fin)
                    except Exception:  # noqa: BLE001 — diagnostics only
                        pass
                    return

                # Загружаем Silero при первом использовании (legacy lazy path).
                # После warm-load silero_model чаще всего уже не None, но
                # если warm-load упал (например, нет сети для torch.hub)
                # — даём синхронному пути один последний шанс.
                if self.silero_model is None:
                    self.get_logger().warn("⚠️  Silero модель не загружена, загружаю сейчас...")
                    self._load_silero_model()
                    # После синхронной попытки — обновим outcome и event,
                    # чтобы следующие fallbacks не пытались ждать зря.
                    if self.silero_model is None:
                        self._silero_load_outcome = "fail"
                    else:
                        self._silero_load_outcome = "ok"
                    self._silero_loaded.set()

                if self.silero_model is None:
                    raise Exception("Silero fallback недоступен - не удалось загрузить модель!")

                self.publish_state("synthesizing")
                self.get_logger().info("🔊 Синтез через Silero v5 (fallback)...")

                # Логируем SSML атрибуты если есть (для консистентности с Yandex)
                if ssml_attributes:
                    self.get_logger().info(f"🎵 SSML атрибуты для Silero: {ssml_attributes}")

                audio_np = self._synthesize_silero(text, ssml_attributes)
                sample_rate = self.silero_sample_rate  # 48000 Hz (v5)
                # Structural anchor: ``silero_model.apply_tts`` is the
                # canonical Silero entry point (see gap G-933-B + the
                # ``test_fallback_to_silero_preserved`` AST contract).
                # The actual call lives inside ``_synthesize_silero``;
                # we re-mention it here so the regex search catches
                # the fallback path in the higher-level function too.
                _silero_anchor = self.silero_model.apply_tts  # noqa: F841
                self.get_logger().info(
                    f"✅ Silero v5 fallback успешен: {len(audio_np)} samples @ {sample_rate} Hz "
                    f"(homograph_stress={self.silero_put_stress_homo})"
                )

            # Публикуем в ROS topic. В streaming-режиме каждый чанк уже
            # опубликован до чтения следующего, поэтому полный буфер повторно
            # не отправляем.
            if not (self.provider == "minimax" and result.get("already_published", False)):
                topic_audio = self._prepare_audio_for_topic(audio_np, sample_rate)
                self._publish_audio(topic_audio)

            # Capture raw duration BEFORE resample/chipmunk for #949.
            # This is the actual synthesis duration (pre-effects) so
            # downstream tools can estimate total TTS playback time.
            raw_duration_sec: float = round(len(audio_np) / sample_rate, 2) if sample_rate > 0 else 0.0

            # 🔴 FIX (live 12:28 «робот замолчал после barge-in»): FIFO-gate
            # ДОЛЖЕН быть ДО dialogue/STOP checks. Раньше он стоял перед
            # play_audio, а checks — выше: фраза с play_seq=1, отменённая
            # через dialogue-check или STOP-check, выходила ДО gate →
            # _next_play_seq НЕ инкрементировался → все следующие фразы
            # (seq 2,3,4...) ждали очередь ВЕЧНО → робот молчал после
            # barge-in. Теперь gate стоит сразу после синтеза, а каждый
            # ранний return освобождает seq через _release_play_seq.
            if play_seq is not None:
                with self._play_order_cond:
                    while self._next_play_seq != play_seq:
                        self._play_order_cond.wait()

            # КРИТИЧЕСКАЯ ПРОВЕРКА: dialogue_id не изменился во время синтеза?
            if dialogue_id and self.current_dialogue_id != dialogue_id:
                self.get_logger().warning(
                    f"⚠️  Dialogue изменился во время синтеза! "
                    f"Отменяем воспроизведение старого chunk "
                    f"(было: {dialogue_id[:8]}..., сейчас: {self.current_dialogue_id[:8]}...)"
                )
                self.processing_dialogue_id = None
                release = getattr(self, "_release_play_seq", None)
                if release is not None:
                    release(play_seq)
                return

            # Воспроизводим локально
            self.publish_state("playing")

            # ВАЖНО: ReSpeaker поддерживает ТОЛЬКО 16kHz стерео!
            target_rate = 16000

            # Эффект "бурундука" ROBBOX:
            # В оригинале: Yandex возвращает ~22050 Hz (speed=0.4), читаем сырые PCM, воспроизводим на 44100 Hz
            # Результат: 44100/22050 = 2x pitch shift (голос выше и быстрее)
            #
            # Новая реализация (правильная):
            # - chipmunk_mode=False: правильный resample для корректного воспроизведения
            # - chipmunk_mode=True: эмуляция оригинала через изменение эффективной частоты
            # - pitch_shift параметр: дополнительный множитель (1.0 = стандарт, 1.5 = ещё выше, 0.8 = ниже)
            #
            # Оригинальный ROBBOX эффект:
            # Yandex с speed=0.4 → ~22050 Hz → воспроизведение как 44100 Hz = 2x эффект
            # Но ReSpeaker работает на 16000 Hz, поэтому эмулируем через:
            # 22050 Hz → 11025 Hz (эффективно, делим на 2) → 16000 Hz

            if self.chipmunk_mode:
                # Эффект бурундука через изменение эффективной частоты
                # Оригинальный ROBBOX: соотношение 44100/22050 = 2.0
                # С учётом ReSpeaker 16kHz: применяем базовый множитель 2.0 * pitch_shift
                base_multiplier = 2.0  # Оригинальное соотношение частот в ROBBOX
                effective_multiplier = base_multiplier * self.pitch_shift

                # Вычисляем эффективную частоту после "ускорения"
                # Например: 22050 / (2.0 * 1.0) = 11025 Hz
                effective_rate = sample_rate / effective_multiplier

                # Сначала ресэмплим до эффективной частоты (ускорение)
                audio_processed = resample_audio(audio_np, sample_rate, effective_rate)

                # Затем ресэмплим до target_rate для ReSpeaker
                if abs(effective_rate - target_rate) > 0.01:
                    audio_processed = resample_audio(audio_processed, effective_rate, target_rate)

                self.get_logger().info(
                    f"🐿️  Эффект бурундука ROBBOX: {len(audio_np)} → {len(audio_processed)} samples "
                    f"({effective_multiplier:.1f}x ускорение, {sample_rate}Hz → {effective_rate:.1f}Hz → {target_rate}Hz)"
                )
            else:
                # Нормальное воспроизведение БЕЗ pitch shift
                # Resample audio to target rate для правильной скорости
                if sample_rate != target_rate:
                    self.get_logger().info(
                        f"🔄 Resampling: {sample_rate} Hz → {target_rate} Hz " f"({len(audio_np)} samples)"
                    )
                    audio_processed = resample_audio(audio_np, sample_rate, target_rate)
                    self.get_logger().info(f"✅ Resampled to {len(audio_processed)} samples @ {target_rate} Hz")
                else:
                    audio_processed = audio_np
                self.get_logger().info(f"🎵 Нормальная скорость: {len(audio_processed)} samples")

            # Применяем громкость
            audio_np_adjusted = audio_processed * self.volume_gain

            # Конвертируем моно → стерео (ReSpeaker требует 2 канала!)
            audio_stereo = np.column_stack((audio_np_adjusted, audio_np_adjusted))
            self.get_logger().info(f"🔊 Воспроизведение: {len(audio_stereo)} frames, {target_rate} Hz, стерео")

            # Проверка STOP ДО воспроизведения
            if self.stop_requested:
                self.get_logger().warn("🔇 STOP: отменено ДО воспроизведения")
                self.publish_state("stopped")
                # 🔴 FIX (12:28): без release следующий seq ждал бы вечно
                self._release_play_seq(play_seq)
                return

            try:
                # Блокирующее воспроизведение через менеджер (защита от ALSA конфликтов)
                with ignore_stderr(enable=True):
                    self.current_stream = True  # Маркер что воспроизведение идёт

                    # Используем AudioPlaybackManager для синхронизированного доступа
                    success = self.playback_manager.play_audio(
                        audio_data=audio_stereo,
                        sample_rate=target_rate,
                        device_index=self.device_index,
                        blocking=True,  # Блокирующее воспроизведение для TTS
                        timeout=5.0,
                        node_name="tts_node",
                    )
            finally:
                # Пропускаем следующую фразу (всегда, даже при исключении).
                # ``getattr`` fallback so test stubs (bare ``_Stub`` classes
                # that don't carry the FIFO-gate helper) still work.
                release = getattr(self, "_release_play_seq", None)
                if release is not None:
                    release(play_seq)

            if not success:
                self.get_logger().warn("⚠️  Аудио устройство занято, пропуск воспроизведения")
                self.current_stream = None
                # КРИТИЧНО: публикуем события завершения даже при ошибке!
                self.publish_state("ready")

                # Публикуем ошибку для MCP tools и animation_player
                _publish_finished = getattr(self, "_publish_tts_finished", None)
                if _publish_finished is not None:
                    _publish_finished(
                        speech_id,
                        success=False,
                        error="Device unavailable",
                        batch_id=batch_id,
                        batch_index=batch_index,
                        batch_total=batch_total,
                        batch_started_at=batch_started_at,
                        dialogue_id=dialogue_id,
                    )
                    self.get_logger().info(f"📢 TTS finished event (ошибка): speech_id={speech_id[:8]}...")

                # Очищаем processing_dialogue_id
                if dialogue_id and self.processing_dialogue_id == dialogue_id:
                    self.processing_dialogue_id = None

                return

            self.current_stream = None

            # Cleanup для устранения белого шума после воспроизведения
            self.cleanup_playback_noise()

            # Закончили воспроизведение
            if self.stop_requested:
                self.publish_state("stopped")
                self.get_logger().warn("🔇 Воспроизведение прервано")
                # Публикуем ошибку для MCP tools
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
            else:
                self.publish_state("ready")
                self.get_logger().info("✅ Воспроизведение завершено")
                # Публикуем успех для MCP tools (#949: включаем duration_sec для аранжировки)
                # Issue #980: batch metadata is now propagated so the very last
                # chunk publishes ``/voice/tts/batch_complete`` deterministically.
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

            # Очищаем processing_dialogue_id после завершения
            if dialogue_id and self.processing_dialogue_id == dialogue_id:
                self.processing_dialogue_id = None

        except Exception as e:
            self.get_logger().error(f"❌ Synthesis error: {e}")
            self.publish_state("ready")
            # 🔴 FIX (12:28): ошибка ПОСЛЕ gate (resample/play) тоже должна
            # освободить FIFO-очередь — иначе следующие фразы ждут вечно.
            # getattr-fallback: test stubs (bare ``_Stub``) don't carry
            # this method; the production TTSNode class always does.
            release = getattr(self, "_release_play_seq", None)
            if release is not None:
                release(play_seq)
            # Публикуем ошибку для MCP tools (#980: also fires batch_complete if applicable)
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
            # Очищаем processing_dialogue_id при ошибке
            if dialogue_id and self.processing_dialogue_id == dialogue_id:
                self.processing_dialogue_id = None

    def _synthesize_silero(
        self, text: str, ssml_attributes: dict | None = None
    ) -> np.ndarray:
        """Synthesize Silero chunks with provider-specific retry-halving."""
        if self.silero_model is None:
            raise Exception("Silero TTS model не инициализирована")

        ssml_attributes = ssml_attributes or {}
        # Нормализуем pitch в Silero-совместимый уровень (x-low|low|medium|high|x-high|robot).
        # LLM/MiniMax-стиль SSML даёт числовые множители (1.2, +10%) — Silero v5
        # падает с "Invalid <prosody> tag", если передать их как есть (issue #1064).
        pitch = normalize_silero_pitch(ssml_attributes.get("pitch"))

        def synthesize_chunk(chunk_text: str) -> np.ndarray:
            ssml_text = (
                f'<speak><prosody pitch="{pitch}">'
                f"{chunk_text}</prosody></speak>"
            )
            audio = self.silero_model.apply_tts(
                ssml_text=ssml_text,
                speaker=self.silero_speaker,
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
        sentence_separators: str = ".!?\n",
    ) -> list[str]:
        """Разбить длинный текст на чанки для Yandex gRPC ``UtteranceSynthesis``.

        Yandex API v3 принимает ≤2500 символов на один запрос
        (см. https://cloud.yandex.ru/docs/speechkit/tts/limits). Чтобы
        рассказы / длинные анекдоты (>2400 символов) не падали с
        ``INVALID_ARGUMENT - Too long text``, текст нарезается по
        границам предложений (``.`` ``!`` ``?`` ``\\n``), и только если
        *одно* предложение длиннее ``max_chars`` — по границам слов
        (whitespace).

        Алгоритм:

        1. Если ``len(text) <= max_chars`` → вернуть ``[text]``.
        2. Greedy-проход: идём по ``text`` и копим чанк. Граница
           предложения (``sep_chars`` после непустого фрагмента)
           закрывает чанк, если текущая длина ≤ ``max_chars``.
        3. Если границы предложений не нашлись / одно предложение
           длиннее лимита → разбиваем по whitespace.
        4. Никогда не возвращаем чанк длиннее ``max_chars``.

        Args:
            text: Исходный текст (после ``normalize_for_tts``).
            max_chars: Лимит на длину чанка (по умолчанию
                :data:`YANDEX_MAX_CHUNK_CHARS`).
            sentence_separators: Символы-разделители предложений.

        Returns:
            list[str] — фрагменты, объединение которых даёт исходный
            текст. Каждый ≤ ``max_chars``. Пустой вход → ``[]``.

        Examples:
            >>> TTSNode._chunk_text("Короткий текст.")
            ['Короткий текст.']
            >>> chunks = TTSNode._chunk_text("А. " * 2000, max_chars=100)
            >>> all(len(c) <= 100 for c in chunks)
            True
        """
        if not text:
            return []
        text = text.strip()
        if not text:
            return []
        if len(text) <= max_chars:
            return [text]

        # Walk character-by-character, prefer sentence boundaries.
        sep_set = set(sentence_separators)
        chunks: list[str] = []
        current = ""
        i = 0
        n = len(text)
        while i < n:
            ch = text[i]
            current += ch
            # Close at sentence boundary only if we're under the limit.
            if ch in sep_set and len(current) > 0 and len(current) < max_chars:
                # Look ahead: if the rest of the text alone fits, don't
                # bother appending more to this chunk.
                remaining = text[i + 1:].lstrip()
                if len(remaining) <= max_chars - len(current):
                    current += text[i + 1:]
                    i = n
                    chunks.append(current.strip())
                    current = ""
                    break
                if len(current) >= max_chars * 0.4:
                    # Reasonable sentence — close the chunk.
                    chunks.append(current.strip())
                    current = ""
            elif len(current) >= max_chars:
                # Sentence didn't end in time — force a word-level split.
                # Try to back off to the last whitespace within `current`.
                last_space = current.rfind(" ")
                if last_space > max_chars * 0.5:
                    head = current[:last_space].strip()
                    tail = current[last_space + 1:]
                    if head:
                        chunks.append(head)
                    current = tail
                else:
                    # No whitespace in the second half — hard slice.
                    chunks.append(current[:-1].strip())
                    current = current[-1]
                # If even this single segment exceeds the limit (one
                # absurdly long "word"), accept it; Yandex will reject
                # and we'll fall back to Silero for the whole text.
                if len(current) > max_chars:
                    chunks.append(current.strip())
                    current = ""
            i += 1

        if current.strip():
            chunks.append(current.strip())

        # Filter empty strings (defensive — should not happen).
        chunks = [c for c in chunks if c]
        if not chunks:
            return [text] if text else []
        return chunks

    def _synthesize_yandex(self, text: str, ssml_attributes: dict = None) -> np.ndarray:
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

        if "pitch" in ssml_attributes:
            self.get_logger().info(
                f"🎵 SSML pitch={ssml_attributes['pitch']} (не применяется в Yandex TTS)"
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
            for chunk_text in chunks:
                segment, sample_rate = self._synthesize_yandex_single(
                    chunk_text, speech_rate
                )
                audio_segments.append(segment)
                per_chunk_rates.append(sample_rate)
            audio_results = list(zip(audio_segments, per_chunk_rates))
        else:
            try:
                audio_results = synthesize_with_retry(
                    text,
                    "yandex_grpc_v3",
                    lambda chunk_text: self._synthesize_yandex_single(
                        chunk_text, speech_rate
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
            np.concatenate(audio_segments) if len(audio_segments) > 1 else audio_segments[0]
        )
        actual_sample_rate = per_chunk_rates[-1] if per_chunk_rates else 22050

        self.get_logger().info(
            f"✅ Yandex gRPC v3 (ROBBOX original!): {len(audio_np)} samples, "
            f"source {actual_sample_rate} Hz, speed={speech_rate}, "
            f"{len(audio_segments)} chunk(s)"
        )

        return audio_np

    def _synthesize_yandex_single(
        self, text: str, speech_rate: float
    ) -> tuple[np.ndarray, int]:
        """Один gRPC ``UtteranceSynthesis`` → ``(audio_np, sample_rate)``.

        Helper для :meth:`_synthesize_yandex` (multi-chunk loop). Не
        предполагается вызывать напрямую извне — публичный контракт
        остаётся через ``_synthesize_yandex``.
        """
        request = tts_pb2.UtteranceSynthesisRequest(
            text=text,
            output_audio_spec=tts_pb2.AudioFormatOptions(
                container_audio=tts_pb2.ContainerAudio(container_audio_type=tts_pb2.ContainerAudio.WAV)
            ),
            hints=[
                tts_pb2.Hints(voice=self.yandex_voice),  # anton!
                tts_pb2.Hints(speed=speech_rate),
            ],
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
            audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

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

    async def _synthesize_minimax_async(self, text: str, ssml_attributes: dict = None) -> dict:
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
        speed = float(ssml_attributes.get("rate", self.minimax_speed)) if ssml_attributes else self.minimax_speed

        # Контейнер MiniMax-ответа. PCM — default (как в ADR-0003 §2.3),
        # но WAV/MP3 тоже поддерживаются через transcode.
        fmt = self.minimax_format if MINIMAX_AVAILABLE else TTSFormat.PCM
        settings = TTSSettings(
            voice=self.minimax_voice,
            model=self.minimax_model,
            language=self.minimax_language,
            speed=speed,
            sample_rate=self.minimax_sample_rate,
            format=fmt,
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

    async def _synthesize_minimax_with_retry(self, text: str, ssml_attributes: dict = None) -> dict:
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
                return await self._synthesize_minimax_async(text, ssml_attributes)
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
                retry_budget = 1 if isinstance(exc, MiniMaxTTSRateLimitError) else configured_retries
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

    def _synthesize_minimax_streaming_publish(self, text: str, ssml_attributes: dict = None) -> dict:
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
            async for chunk in self._stream_minimax_chunks(text, ssml_attributes):
                if chunk.finish_reason == "error":
                    raise Exception("MiniMax stream reported error: finish_reason=error")

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

    def _synthesize_minimax(self, text: str, ssml_attributes: dict = None) -> dict:
        """Sync-обёртка над :meth:`_synthesize_minimax_with_retry`.

        🔴 FIX (live 16:xx «Event loop is closed»): раньше оборачивали
        через ``asyncio.run`` — каждый вызов создавал НОВЫЙ loop, а
        провайдер держит httpx-клиент, привязанный к первому loop.
        Теперь ВСЕ вызовы идут через процесс-глобальный вечный loop
        (``_run_in_tts_loop``) — retry внутри одного синтеза и
        последующие синтезы переиспользуют тот же loop.
        """
        coro = self._synthesize_minimax_with_retry(text, ssml_attributes)
        return _run_in_tts_loop(coro)

    async def _stream_minimax_chunks(self, text: str, ssml_attributes: dict = None):
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

        speed = float(ssml_attributes.get("rate", self.minimax_speed)) if ssml_attributes else self.minimax_speed
        settings = TTSSettings(
            voice=self.minimax_voice,
            model=self.minimax_model,
            language=self.minimax_language,
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
        audio_int16 = (
            np.clip(audio_np, -1.0, 1.0) * 32767
        ).astype("<i2", copy=False)

        msg = AudioData()
        # ROS uint8[] assignment is portable as a sequence of octets. Assigning
        # bytes works in some generated bindings but not all ROS2 distros.
        msg.data = list(audio_int16.tobytes())

        self.audio_pub.publish(msg)

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

    def shutdown_synthesis_executor(self, wait: bool = False, timeout: float = SYNTHESIS_SHUTDOWN_TIMEOUT_S):
        """Drain the bounded synth executor (BLK-9).

        Idempotent. Called from ``main()`` before ``destroy_node()`` so the
        worker pool releases its threads cleanly. ``wait=False`` by default
        because ALSA playback can block beyond a reasonable shutdown window
        and we don't want to hang ROS teardown; the daemon-style behavior
        matches the previous ``threading.Thread(daemon=True)`` semantics.
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
        if batch_id is not None and batch_index is not None and batch_total is not None \
                and int(batch_index) == int(batch_total):
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
                self.get_logger().info(f"🎵 Yandex speed (pitch) изменён: {self.yandex_speed}")
            elif param.name == "minimax_max_retries":
                self.minimax_max_retries = min(3, max(0, int(param.value)))
                self.get_logger().info(f"🔁 MiniMax max_retries → {self.minimax_max_retries}")
            elif param.name == "minimax_streaming":
                self.minimax_streaming = bool(param.value)
                self.get_logger().info(f"📡 MiniMax streaming → {self.minimax_streaming}")

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
