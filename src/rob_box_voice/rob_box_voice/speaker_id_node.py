#!/usr/bin/env python3
"""
speaker_id_node.py — Real-time speaker identification using resemblyzer d-vectors.

Subscribes:
    /audio/speech_audio  (AudioData)  — full speech utterance from audio_node
    /voice/speaker/register (String)  — JSON {"name":"Иван"} — register current speaker
    /voice/speaker/rename   (String)  — JSON {"speaker_id"|"old_name", "new_name"}
    /voice/speaker/merge    (String)  — JSON {"src_speaker_id","dst_speaker_id"} —
                                         issue W5-4, склейка дублей одного голоса
    /voice/speaker/observe  (String)  — JSON {"speaker_id","text"} — реплика
                                         известного спикера для подсчёта тем и
                                         выбора эпитета (issue #1787)
    /voice/speaker/epithet  (String)  — JSON {"speaker_id","epithet"} — кличка,
                                         придуманная LLM (слой 2 гибрида);
                                         принимается после валидации

Publishes:
    /voice/speaker/result (String) — JSON SpeakerMatch or {"is_known":false};
                                     у известного спикера есть поле "epithet"
                                     (внутренняя кличка, issue #1787). Тем же
                                     топиком уходит ack на merge.
    /voice/speaker/epithet_request (String) — JSON {"speaker_id","fallback",
                                     "cluster","hints","messages"} — просьба к
                                     dialogue_node придумать кличку через LLM

Runtime-параметр (``ros2 param set``, НЕ топик — issue #2750, см. §"Seam
guard" в ADR-0128):
    e2e_mode (bool) — переключить активную БД дикторов db_path ↔
        e2e_db_path. Владелец — только E2E-харнесс
        (``ros2 param set /speaker_id_node e2e_mode true|false``, по
        образцу ``ros2 param set /dialogue_node barge_in_policy classify``
        из ``scripts/e2e/run_night_marathon.sh``). См. ``parameters_callback``
        / ``_apply_e2e_mode``.

Parameters:
    db_path                    (str)   — path to SQLite DB       [/data/speakers.db]
    e2e_db_path                (str)   — изолированная БД для E2E-режима (issue
                                        #2750), переключается параметром
                                        e2e_mode выше [/data/speakers.e2e.db]
    identify_threshold         (float) — cosine similarity gate, НЕ зависит от
                                        размера галереи (issue #2747 —
                                        адаптивный порог по gallery_size
                                        опробован и отклонён на реальных
                                        данных, см. speaker_embeddings.
                                        GALLERY_WARMUP_SIZE) [0.72]
    register_match_threshold   (float) — порог слияния при регистрации (issue
                                        W5-4 + #2348; строже identify_threshold —
                                        см. speaker_embeddings.REGISTER_MATCH_THRESHOLD) [0.75]
    gallery_warmup_size        (int)   — issue #2747: потолок числа
                                        эмбеддингов, которые growth-сессия
                                        (см. _apply_growth_session) может
                                        дописать в галерею за один
                                        непрерывный разговор после
                                        register_speaker [5]
    gallery_growth_session_gap_sec (float) — issue #2747: максимальный
                                        разрыв между репликами внутри
                                        growth-сессии — больше этого считаем,
                                        что человек мог уйти и сессия
                                        прервана [30.0]
    memory_db_path              (str)   — harness_voice.db, факты через шов
                                        идентичности (issue #2440) [/data/harness_voice.db]
    voice_facts_db_path         (str)   — voice_memory.db, второй писатель
                                        фактов (issue #2751) [/data/voice_memory.db]
    sample_rate                (int)   — PCM sample rate         [16000]
    enabled                    (bool)  — enable/disable node     [true]
"""

import collections
import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Deque, Dict, Optional, Tuple

import numpy as np
import rclpy
from audio_common_msgs.msg import AudioData
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .core import epithets
from .utils import speaker_embeddings as _se_mod
from .utils.speaker_embeddings import SpeakerDatabase, SpeakerMatch

# Issue #1160 — Prometheus metrics (этап 1 observability).
# ``prometheus_client`` — optional dep; если её нет, всё превращается в
# no-op и старт сервера тихо возвращает ``False``.
from rob_box_voice.observability import (
    is_metrics_enabled,
    record_speaker_recognize,
    start_metrics_server,
)


class SpeakerIdNode(Node):
    """Voice-based speaker identification node."""

    def __init__(self) -> None:
        super().__init__("speaker_id_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("db_path", "/data/speakers.db")
        self.declare_parameter("identify_threshold", 0.72)
        # Issue W5-4 + #2348 — отдельный, более строгий порог для решения
        # «слить с существующим профилем при регистрации vs завести новый»
        # внутри register_or_merge(). Калибровка по
        # .hermes/research/cosine_distributions/REPORT.md (n=280 same / n=666 cross,
        # 0.75 даёт TPR 53 % / FPR 3.9 % — компромисс между «поймать дубль» и
        # «не склеить разных людей»). См. speaker_embeddings.REGISTER_MATCH_THRESHOLD.
        self.declare_parameter("register_match_threshold", 0.75)
        # Issue #2747 — растущая галерея БЕЗ адаптивного порога (порог по
        # cosine пробовали и откатили — см. speaker_embeddings.
        # GALLERY_WARMUP_SIZE: same-voice/cross-voice распределения на
        # реальных данных робота пересекаются целиком, порог не разделяет).
        # Рост галереи теперь держится на непрерывности сессии диалога, а
        # не на похожести голоса — см. _apply_growth_session.
        self.declare_parameter("gallery_warmup_size", 5)
        # Issue #2747 — сколько секунд тишины между репликами ещё считается
        # «тот же непрерывный разговор» для growth-сессии. Значение — то же
        # 30 c, что уже используется этой нодой в другом месте для похожего
        # суждения «эмбеддинг ещё свежий/относится к текущему
        # взаимодействию» (``_MAX_EMBED_AGE_SEC``, ``_on_register_request``)
        # — переиспользуем существующую калибровку, а не придумываем новое
        # число.
        self.declare_parameter("gallery_growth_session_gap_sec", 30.0)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("enabled", True)
        # Issue #1160 — Prometheus metrics endpoint. 9112 — speaker_id_node.
        self.declare_parameter("metrics_port", 9112)
        # Issue #2440 — путь к harness-БД фактов (harness_voice.db). Нужен
        # шву идентичности для переноса ФАКТОВ профиля при склейке (дефект C:
        # раньше merge переносил только эмбеддинги в speakers.db, а факты
        # оставались висеть под старым id). Тот же файл, что у dialogue_node
        # (sqlite_db_path) — WAL допускает второе подключение на запись.
        self.declare_parameter("memory_db_path", "/data/harness_voice.db")
        # Issue #2751 — второй писатель фактов, вне шва идентичности:
        # MCP-инструмент memory_save пишет в voice_facts этого файла (не в
        # harness_voice.db выше). Пока обе БД живы (миграция #2000 не
        # доведена до конца, см. ADR-0128), merge() без этого пути молча
        # терял бы живые факты при склейке профилей. Тот же дефолт, что у
        # VOICE_MEMORY_DB_PATH в mcp_server.py.
        self.declare_parameter("voice_facts_db_path", "/data/voice_memory.db")
        # Issue #2750 — изолированная БД дикторов для E2E-актов знакомства.
        # По умолчанию не используется: узел открывает db_path (боевую) при
        # старте и остаётся на ней, пока ``ros2 param set ... e2e_mode true``
        # не переключит его (см. parameters_callback / _apply_e2e_mode).
        # Раньше «чистую» БД для акта получали ssh-бэкапом+обнулением боевой
        # speakers.db (issue #2750) — здесь то же самое достигается без
        # единого прикосновения к боевому файлу.
        self.declare_parameter("e2e_db_path", "/data/speakers.e2e.db")
        # Runtime-переключатель (см. докстринг модуля). Bool, а не топик —
        # семейный шаблон уже есть в репозитории (``barge_in_policy`` у
        # dialogue_node, ``ros2 param set`` в докстринге
        # run_night_marathon.sh): синхронный ответ вызывающему (успех/провал
        # виден тут же в exit-коде ``ros2 param set``), не требует своего
        # ROS-типа сообщения и не всплывает в seam_without_consumer как
        # «топик без потребителя», потому что consumer — сам параметр, а не
        # что-то, что сканер обязан находить отдельно.
        self.declare_parameter("e2e_mode", False)
        # Issue #2609 — defer resemblyzer warm-load to first real inference
        # unless explicitly opted in. Default False: at boot the robot
        # almost never needs speaker ID on the first few utterances
        # (silence / VAD-only noise → embed_audio returns None anyway),
        # and the warm-load costs ~600 MB RSS (torch + GE2E model). With
        # the CPU-only torch wheel (pinned in
        # docker/vision/voice_*/requirements.txt), the cold-load on first
        # ``embed_audio`` is ~2-3 s — acceptable for a biometric
        # emergency path (зарегистрироваться / опознать нового
        # собеседника), but unacceptable for an always-on warm-load that
        # pays the cost on EVERY container restart even when nobody
        # speaks. Set True to restore legacy behaviour (warm at startup).
        self.declare_parameter("resemblyzer_warmup_on_start", False)

        self._enabled: bool = self.get_parameter("enabled").value
        self._sample_rate: int = self.get_parameter("sample_rate").value
        db_path: str = self.get_parameter("db_path").value
        threshold: float = self.get_parameter("identify_threshold").value
        register_threshold: float = self.get_parameter("register_match_threshold").value
        gallery_warmup_size: int = int(self.get_parameter("gallery_warmup_size").value)
        self._growth_session_gap_sec: float = float(
            self.get_parameter("gallery_growth_session_gap_sec").value
        )

        if not self._enabled:
            self.get_logger().info("⚠️ speaker_id_node disabled via parameter")
            return

        # ── Speaker DB (thread-safe via lock) ─────────────────────────────────
        # Issue #2750 — db_path боевой (запоминаем отдельно от e2e_db_path,
        # чтобы _apply_e2e_mode мог вернуться на неё). self._db_lock
        # охраняет только САМУ замену self._db на новый объект — не каждое
        # обращение к нему (как и раньше: _process_utterance читает self._db
        # из executor-потока без блокировки, склейка/rename — из ROS-потока;
        # это тот же непокрытый гонками участок, что был до этой правки,
        # честно не расширяем его здесь).
        self._prod_db_path: str = db_path
        self._e2e_db_path: str = str(
            self.get_parameter("e2e_db_path").value or "/data/speakers.e2e.db"
        )
        self._e2e_mode_active: bool = False
        self._db_lock = threading.Lock()
        self._db = SpeakerDatabase(db_path)
        # Patch thresholds from parameters (модуль импортирован один раз на
        # уровне файла — как ``_se_mod``, так и ``SpeakerDatabase`` /
        # ``SpeakerMatch`` из того же объекта модуля, см. импорты вверху).
        _se_mod.IDENTIFY_THRESHOLD = threshold
        _se_mod.REGISTER_MATCH_THRESHOLD = register_threshold
        # Issue #2747 — см. declare_parameter выше.
        _se_mod.GALLERY_WARMUP_SIZE = gallery_warmup_size
        self.get_logger().info(
            f"✅ SpeakerDatabase opened: {db_path} "
            f"identify_threshold={threshold} register_match_threshold={register_threshold} "
            f"gallery_warmup_size={gallery_warmup_size} "
            f"gallery_growth_session_gap_sec={self._growth_session_gap_sec}"
        )
        # Issue #2750 — тот же шаблон, что dialogue_node (barge_in_policy)
        # и tts_node (volume_db и др.): валидирующий Humble-колбэк, тело
        # которого делает синхронный побочный эффект (там — перечитать
        # громкость/скорость, здесь — переоткрыть БД). Не топик: топик с
        # «стирает БД по одному сообщению без подтверждения» — bad seam
        # (seam_without_consumer справедливо считает Python-паблишера/
        # подписчика без второго конца новым швом; e2e_voice_test.sh — это
        # bash, сканер его не видит). ``ros2 param set`` синхронный,
        # возвращает успех/провал вызывающему в exit-коде — сильнее топика
        # даже без учёта сторожа.
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Issue #2747 — growth-сессия: непрерывность диалога как якорь для
        # роста галереи ВМЕСТО акустического порога (см. большой
        # комментарий у speaker_embeddings.GALLERY_WARMUP_SIZE — порог по
        # cosine пробовали и откатили). Устанавливается в _do_register(),
        # продлевается/закрывается в _apply_growth_session(). None — нет
        # активной сессии (ничего не растим).
        self._growth_session: Optional[dict] = None

        # ── Pending registration ───────────────────────────────────────────────
        # Set when user says "запомни мой голос как [name]" via /voice/speaker/register.
        # The NEXT speech utterance will be registered under this name.
        self._pending_register_name: Optional[str] = None
        self._pending_register_lock = threading.Lock()

        # Recent embeddings ring-buffer: (timestamp, embedding, duration_sec) —
        # keep last 20 utterances. LLM may take 2-5s to call register_speaker,
        # so a single _last_embedding can be overwritten by ambient noise.
        # Keep a window instead. Issue #2769 — duration_sec хранится вместе с
        # эмбеддингом, чтобы register_speaker (приходит позже, отдельным
        # топиком) мог передать ЕЁ в register_or_merge(duration_sec=...) —
        # без этого поля гейт MIN_REGISTER_AUDIO_DURATION_SEC нечем было бы
        # проверить на этом пути (в отличие от pending_name-ветки, где
        # длительность известна сразу в _process_utterance).
        self._recent_embeddings: Deque[Tuple[float, np.ndarray, float]] = collections.deque(
            maxlen=20
        )
        self._MAX_EMBED_AGE_SEC: float = 30.0

        # Issue #1787 — окно последних реплик КАЖДОГО спикера: на нём
        # считаются темы (epithets.extract_tags) и валентность. 50 — из
        # research §4.1 («новая доминирующая тема > 40% последних 50
        # реплик»). Живёт в памяти ноды, а не в БД: это скользящее окно
        # для решения «пора менять кличку», а не история диалога — её
        # хранит слой памяти harness'а.
        self._speech_log: Dict[str, Deque[str]] = {}
        self._speech_log_lock = threading.Lock()

        # ── Thread pool for inference (non-blocking ROS callbacks) ────────────
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="speaker_id")
        # Issue #2609 — gate the eager warm-load on a parameter (default
        # False). The warmup call below triggers resemblyzer's
        # ``VoiceEncoder(device="cpu")`` which loads torch + the GE2E
        # model (~600 MB RSS with the old `+cu130` wheel; ~70 MB now after
        # the CPU-only pin in docker/vision/voice_*/requirements.txt).
        # On a robot container that runs 9 nodes sharing 4 GB mem_limit,
        # paying that cost on EVERY boot even when nobody speaks is
        # wasteful — the lazy path (first real embed_audio) is fine for
        # biometric, which is a cold path (user explicitly asks
        # "запомни мой голос" or LLM calls register_speaker). Operators
        # that want the legacy "warm at startup" behaviour set
        # ``resemblyzer_warmup_on_start: true`` in speaker_id_node.yaml.
        if bool(self.get_parameter("resemblyzer_warmup_on_start").value):
            # Warm up resemblyzer model immediately so first real inference is fast
            self._executor.submit(self._warmup)
        else:
            self.get_logger().info(
                "🪶 resemblyzer warmup deferred to first embed_audio "
                "(issue #2609: saves ~70 MB RSS on every container restart)"
            )

        # ── QoS ───────────────────────────────────────────────────────────────
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._result_pub = self.create_publisher(String, "/voice/speaker/result", reliable_qos)
        # Issue #1787, слой 2 — просьба к dialogue_node придумать кличку
        # через LLM. Сам узел LLM не знает (и не должен: биометрия обязана
        # работать офлайн), поэтому запрос уходит топиком, а ответ
        # приходит на /voice/speaker/epithet.
        self._epithet_request_pub = self.create_publisher(
            String, "/voice/speaker/epithet_request", reliable_qos
        )

        # Issue #1160 — Prometheus metrics endpoint. 9112 — speaker_id_node.
        # Запускаем сервер ТОЛЬКО если есть резёмблизер (иначе нода не даёт
        # идентификации, и метрики бесполезны). Порт читаем из параметра.
        metrics_port: int = int(self.get_parameter("metrics_port").value or 0)
        if metrics_port > 0 and is_metrics_enabled():
            if start_metrics_server(metrics_port):
                self.get_logger().info(
                    f"📊 Speaker-ID metrics server listening on :{metrics_port}/metrics"
                )
            else:
                self.get_logger().warning(
                    f"📊 Speaker-ID metrics port {metrics_port} not bound "
                    "(busy or prometheus_client missing)"
                )

        # ── Subscribers ────────────────────────────────────────────────────────
        self.create_subscription(
            AudioData,
            "/audio/speech_audio",
            self._on_speech_audio,
            best_effort_qos,
        )
        self.create_subscription(
            String,
            "/voice/speaker/register",
            self._on_register_request,
            reliable_qos,
        )
        self.create_subscription(
            String,
            "/voice/speaker/rename",
            self._on_rename_request,
            reliable_qos,
        )
        # Issue W5-4 — ручная склейка уже расползшихся дублей одного голоса
        # (например, найденных оператором через list_speakers): JSON
        # {"src_speaker_id": "...", "dst_speaker_id": "..."}.
        self.create_subscription(
            String,
            "/voice/speaker/merge",
            self._on_merge_request,
            reliable_qos,
        )
        # Issue #1787 — реплики известного спикера для выбора эпитета.
        # Текст живёт в dialogue_node (STT), голос — здесь; связывает их
        # speaker_id. Отдельный топик, а не расширение /voice/stt/result:
        # эпитет нужен ТОЛЬКО когда биометрия уже опознала говорящего,
        # иначе теми чужой речи испортили бы чужой профиль.
        self.create_subscription(
            String,
            "/voice/speaker/observe",
            self._on_observe_request,
            reliable_qos,
        )
        # Issue #1787, слой 2 — кличка, придуманная LLM в dialogue_node.
        self.create_subscription(
            String,
            "/voice/speaker/epithet",
            self._on_epithet_result,
            reliable_qos,
        )
        self.get_logger().info("🎙️ speaker_id_node ready")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _warmup(self) -> None:
        """Pre-load the resemblyzer GE2E model so first real inference is fast."""
        import time as _time
        t0 = _time.monotonic()
        # Issue #1101 — was feeding 1 second of silence (``bytes(16000 * 2)``
        # = int16 zeros). ``resemblyzer.audio.preprocess_wav`` divides RMS
        # into int16_max inside ``log10`` → ``RuntimeWarning: divide by
        # zero encountered in log10`` + ``invalid value encountered in
        # multiply`` + empty output array (the embedding call silently
        # returned). Use a noise-shaped warmup so RMS > 0 and the
        # model loads cleanly.
        import numpy as np
        rng = np.random.default_rng(42)
        warmup = (
            rng.normal(0, 0.05, 16000).clip(-1, 1).astype(np.float32)
        )
        pcm16 = (warmup * 32767).astype(np.int16).tobytes()
        self._db.embed_audio(pcm16, sample_rate=16000)
        elapsed_ms = int((_time.monotonic() - t0) * 1000)
        self.get_logger().info(f"🔥 Resemblyzer warmup done ({elapsed_ms} ms)")

    def _on_speech_audio(self, msg: AudioData) -> None:
        """Received a complete speech utterance — run inference asynchronously."""
        pcm_bytes = bytes(msg.data)
        self.get_logger().info(
            f"🎤 Received speech audio: {len(pcm_bytes)} bytes ({len(pcm_bytes)/self._sample_rate/2:.1f}s)"
        )
        self._executor.submit(self._process_utterance, pcm_bytes)

    def _on_register_request(self, msg: String) -> None:
        """Register the current (or next) speaker under the given name.

        Expected JSON: {"name": "Иван"} or {"name": "Иван", "speaker_id": "<uuid>"}
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            # Accept plain text name as well
            data = {"name": msg.data.strip()}

        name = data.get("name", "").strip()
        if not name:
            self.get_logger().warning("⚠️ register_request: empty name ignored")
            return

        speaker_id_hint: Optional[str] = data.get("speaker_id")

        # If we have a fresh embedding from the latest utterance, register immediately
        now = time.time()
        with self._pending_register_lock:
            # Find most recent embedding within MAX_EMBED_AGE_SEC
            best_embedding = None
            best_duration: Optional[float] = None
            best_ts = 0.0
            for ts, emb, dur in reversed(self._recent_embeddings):
                if now - ts <= self._MAX_EMBED_AGE_SEC and ts > best_ts:
                    best_embedding = emb
                    best_duration = dur
                    best_ts = ts
            if best_embedding is not None:
                self._executor.submit(
                    self._do_register,
                    name,
                    best_embedding,
                    speaker_id_hint,
                    best_duration,
                )
                self.get_logger().info(
                    f"📝 Registering '{name}' from embedding {now - best_ts:.1f}s ago"
                )
            else:
                # No fresh utterance yet — pend for the next one
                self._pending_register_name = name
                self.get_logger().info(
                    f"📝 Will register next utterance as '{name}'"
                )

    # ── Processing ────────────────────────────────────────────────────────────

    def _process_utterance(self, pcm_bytes: bytes) -> None:
        """Compute embedding and identify (or register) speaker.  Runs in thread."""
        t0 = time.monotonic()
        # Issue #2769 — длительность нужна ОТДЕЛЬНО от identify()-гейта
        # embed_audio(): register_or_merge() проверяет её против более
        # строгого MIN_REGISTER_AUDIO_DURATION_SEC. Формула — как в логе
        # "🎤 Received speech audio" (_on_speech_audio) — int16 моно, 2 байта
        # на сэмпл.
        duration_sec = len(pcm_bytes) / self._sample_rate / 2

        embedding = self._db.embed_audio(pcm_bytes, self._sample_rate)
        if embedding is None:
            # resemblyzer unavailable or audio too short — publish unknown
            self.get_logger().warning(
                f"⚠️ embed_audio returned None for {len(pcm_bytes)} bytes "
                f"({len(pcm_bytes)/self._sample_rate/2:.1f}s) — publishing unknown"
            )
            # Issue #1160 — Prometheus metrics: не удалось извлечь эмбеддинг —
            # считаем это unknown.
            record_speaker_recognize(known=False, confidence=None)
            self._publish_result(None)
            return

        elapsed = (time.monotonic() - t0) * 1000

        # Store as latest embedding for possible registration
        with self._pending_register_lock:
            self._recent_embeddings.append((time.time(), embedding, duration_sec))
            pending_name = self._pending_register_name
            self._pending_register_name = None

        if pending_name:
            # Issue #2748 — _do_register() теперь САМ публикует полноценный
            # SpeakerMatch (is_known=true, source="register") на
            # /voice/speaker/result, поэтому второй, отдельный
            # identify()+publish() здесь больше не нужен — раньше он был
            # ЕДИНСТВЕННЫМ источником такого сигнала для этой ветки, но для
            # ветки _on_register_request (без pending, с сразу доступным
            # эмбеддингом — обычный путь LLM-тула register_speaker) его не
            # было вовсе, что и было причиной бага #2748 («имя не доезжает
            # до лица»). Дублировать здесь identify() с адаптивным порогом
            # (issue #2747) избыточно и может дать РАСХОДЯЩИЙСЯ результат
            # (например, is_known=false из-за шумной первой фразы сразу
            # после регистрации), перезаписав только что опубликованный
            # источник истины «человек сам назвал своё имя».
            registered = self._do_register(
                pending_name, embedding, speaker_id=None, duration_sec=duration_sec
            )
            # Issue #2769 — _do_register() возвращает False, если реплика
            # оказалась короче MIN_REGISTER_AUDIO_DURATION_SEC: профиль НЕ
            # создан, ack event="register_error" уже отправлен изнутри.
            # Метрика и лог "✅ Registered" здесь были бы ложью — заявляли
            # бы известного спикера с confidence=1.0 для эталона, которого
            # не существует.
            if registered:
                self.get_logger().info(
                    f"✅ Registered '{pending_name}' (inference {elapsed:.0f} ms)"
                )
                # Issue #1160 — Prometheus metrics: только что
                # зарегистрированный спикер считается known. confidence=1.0
                # — эмбеддинг только что записан как ЭТАЛОН для profile,
                # self-similarity максимальна (см. _do_register).
                record_speaker_recognize(known=True, confidence=1.0)
            else:
                self.get_logger().warning(
                    f"⚠️ [issue #2769] Registration of '{pending_name}' "
                    f"rejected — audio too short ({duration_sec:.2f}s, "
                    f"inference {elapsed:.0f} ms)"
                )
                record_speaker_recognize(known=False, confidence=None)
            return

        match = self._db.identify(embedding)
        self._log_identify_candidates(embedding)
        # Issue #2747 — рост галереи НЕ зависит от исхода identify() выше
        # (см. большой комментарий у speaker_embeddings.GALLERY_WARMUP_SIZE:
        # акустический гейт для этого решения отклонён — same/cross-voice
        # распределения пересекаются целиком на реальных данных робота).
        # Якорь — непрерывность growth-сессии, открытой в _do_register().
        self._apply_growth_session(embedding, match)
        if match:
            self.get_logger().info(
                f"👤 Speaker: '{match.name}' confidence={match.confidence:.3f} "
                f"({elapsed:.0f} ms)"
            )
        else:
            self.get_logger().info(f"👤 Speaker: unknown ({elapsed:.0f} ms)")

        # Issue #1160 — Prometheus metrics: known/unknown.
        record_speaker_recognize(
            known=bool(match),
            confidence=match.confidence if match else None,
        )
        self._publish_result(match)

    def _apply_growth_session(
        self, embedding: np.ndarray, match: Optional[SpeakerMatch]
    ) -> None:
        """Issue #2747 — дописать эмбеддинг в галерею по якорю сессии.

        НЕ акустический гейт (см. speaker_embeddings.GALLERY_WARMUP_SIZE —
        порог по cosine пробовали и отклонили на реальных данных робота:
        same-voice/cross-voice распределения пересекаются целиком, порог их
        не разделяет). Якорь — непрерывность: ``_do_register()`` открывает
        growth-сессию сразу после явной регистрации («человек только что
        представился»), и, пока реплики идут подряд без большого разрыва,
        они считаются принадлежащими тому же человеку — тот же принцип,
        которым уже пользуется ``mcp_server._on_speaker_result`` для
        события ``event="registered"``.

        Условия закрытия сессии (ADR-0127-стиль: ошибаться дёшево — здесь
        просто НЕ дописываем эмбеддинг, профиль не портится):
          * разрыв с прошлой репликой сессии > ``_growth_session_gap_sec``
            — человек мог уйти, сессия прервана;
          * галерея уже доросла до ``GALLERY_WARMUP_SIZE`` — потолок;
          * ЭТА реплика уверенно (обычный ``identify()``, калиброванный
            порог) опознана как ДРУГОЙ, уже известный спикер — сильное
            прямое свидетельство, что якорь больше не в кадре/у микрофона
            (высокий косинус используется здесь как ДОПОЛНИТЕЛЬНОЕ условие
            ПОВЕРХ якоря — вето, а не самостоятельный порог доверия).
        """
        session = self._growth_session
        if session is None:
            return
        now = time.time()

        if now - session["last_utterance_at"] > self._growth_session_gap_sec:
            self.get_logger().info(
                f"🌙 [issue #2747] growth-сессия '{session['name']}' закрыта "
                f"по таймауту ({now - session['last_utterance_at']:.1f}s > "
                f"{self._growth_session_gap_sec}s без реплик)"
            )
            self._growth_session = None
            return

        if match is not None and match.speaker_id != session["speaker_id"]:
            # Реплика уверенно опознана как ДРУГОЙ человек — якорь больше не
            # актуален (кто-то другой заговорил / подошёл). Не дописываем и
            # закрываем сессию: продолжать доверять якорю после прямого
            # акустического опровержения нельзя.
            self.get_logger().info(
                f"🌙 [issue #2747] growth-сессия '{session['name']}' закрыта: "
                f"реплика уверенно опознана как '{match.name}' "
                f"(score={match.confidence:.3f}) — другой человек у микрофона"
            )
            self._growth_session = None
            return

        if not self._db.append_reference_embedding(
            session["speaker_id"], session["name"], embedding
        ):
            # Потолок GALLERY_WARMUP_SIZE достигнут — сессии больше нечего
            # делать, закрываем её (не ошибка, штатное завершение роста).
            self.get_logger().info(
                f"🌱 [issue #2747] growth-сессия '{session['name']}' "
                f"завершена: галерея достигла {_se_mod.GALLERY_WARMUP_SIZE} "
                f"эмбеддингов"
            )
            self._growth_session = None
            return

        session["last_utterance_at"] = now
        session["count"] += 1
        self.get_logger().info(
            f"🌱 [issue #2747] Галерея '{session['name']}' "
            f"({session['speaker_id'][:8]}) пополнена по якорю сессии: "
            f"+1 эмбеддинг (всего добавлено в сессии: {session['count']}, "
            f"gallery_size={self._db.gallery_size(session['speaker_id'])}"
            f"/{_se_mod.GALLERY_WARMUP_SIZE})"
        )

    def _log_identify_candidates(self, embedding: np.ndarray) -> None:
        """Issue W5-4 п.4 — диагностика: best_score И второй кандидат.

        Без этого лога в проде виден только булев результат identify()
        («известен / неизвестен»), и дрейф голоса между двумя дублирующими
        профилями невозможно отследить постфактум — неясно, насколько
        близко было решение и с кем именно конкурировал победитель. Лог
        уровня INFO — намеренно (не debug): это ровно то, что нужно
        вытащить из логов робота при разборе жалобы «опознал не того».
        """
        candidates = self._db.identify_candidates(embedding, top_n=2)
        if not candidates:
            return
        best = candidates[0]
        if len(candidates) > 1:
            second = candidates[1]
            gap = best.confidence - second.confidence
            self.get_logger().info(
                f"🔍 identify candidates: best='{best.name}'({best.speaker_id[:8]}) "
                f"score={best.confidence:.3f} | second='{second.name}'"
                f"({second.speaker_id[:8]}) score={second.confidence:.3f} | gap={gap:.3f}"
            )
        else:
            self.get_logger().info(
                f"🔍 identify candidates: best='{best.name}'({best.speaker_id[:8]}) "
                f"score={best.confidence:.3f} | (единственный известный спикер в БД)"
            )

    def _on_rename_request(self, msg: String) -> None:
        """Rename an existing speaker entry.

        Expected JSON: {"speaker_id": "<uuid>", "new_name": "<name>"}
        or: {"old_name": "<name>", "new_name": "<name>"}
        (old_name → name-based lookup, for LLM-driven corrections).
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("⚠️ rename_request: invalid JSON ignored")
            return

        speaker_id = data.get("speaker_id", "").strip()
        old_name = data.get("old_name", "").strip()
        new_name = data.get("new_name", "").strip()
        if not speaker_id and not old_name:
            self.get_logger().warning("⚠️ rename_request: missing speaker_id or old_name")
            return
        if not new_name:
            self.get_logger().warning("⚠️ rename_request: missing new_name")
            return

        ok = False
        if speaker_id:
            ok = self._db.rename(speaker_id, new_name)
        else:
            # Issue #1101 — name-based rename for LLM corrections.
            # When user says "I'm not X, I'm Y", LLM calls
            # register_speaker(name=Y, old_name=X) → published as
            # /voice/speaker/rename {"old_name": X, "new_name": Y}.
            sid = self._db.rename_by_name(old_name, new_name)
            ok = sid is not None
            speaker_id = sid or ""

        if ok:
            self.get_logger().info(f"✏️ Renamed → '{new_name}' (id={speaker_id[:8]})")
        else:
            self.get_logger().warning(
                f"⚠️ rename failed: "
                f"{'old_name=' + old_name if old_name else 'speaker_id=' + speaker_id[:8]} "
                f"not found in DB"
            )

        ack = String()
        ack.data = json.dumps(
            {"event": "renamed", "ok": ok, "speaker_id": speaker_id, "new_name": new_name},
            ensure_ascii=False,
        )
        self._result_pub.publish(ack)

    def _on_merge_request(self, msg: String) -> None:
        """Issue W5-4 + #2440 — склеить два профиля одного голоса.

        Expected JSON: {"src_speaker_id": "<uuid>", "dst_speaker_id": "<uuid>"}
        Все эмбеддинги ``src`` переносятся под ``dst``, профиль ``src``
        удаляется, и — через шов идентичности — факты памятного слоя
        (``speaker_scope(src)`` → ``speaker_scope(dst)``) переносятся тоже
        (дефект C из issue #2440). Имя ``dst`` остаётся как есть —
        вызывающий код сам решает, какой из двух id — "основной".
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("⚠️ merge_request: invalid JSON ignored")
            return

        src_id = str(data.get("src_speaker_id", "")).strip()
        dst_id = str(data.get("dst_speaker_id", "")).strip()
        if not src_id or not dst_id:
            self.get_logger().warning(
                "⚠️ merge_request: missing src_speaker_id or dst_speaker_id"
            )
            return

        try:
            embeddings_moved, facts_moved = self._merge_identity(src_id, dst_id)
        except Exception as exc:  # noqa: BLE001 — merge не должен ронять ноду
            self.get_logger().warning(
                f"⚠️ merge via identity seam failed: {type(exc).__name__}: {exc}"
            )
            embeddings_moved, facts_moved = 0, 0
        ok = embeddings_moved > 0
        if ok:
            self.get_logger().info(
                f"🔗 Merged speaker {src_id[:8]} → {dst_id[:8]} "
                f"({embeddings_moved} embeddings, {facts_moved} facts moved)"
            )
        else:
            self.get_logger().warning(
                f"⚠️ merge failed: src={src_id[:8]} dst={dst_id[:8]} "
                "(src==dst, src not found, or dst not found in DB)"
            )

        ack = String()
        ack.data = json.dumps(
            {
                "event": "merged",
                "ok": ok,
                "src_speaker_id": src_id,
                "dst_speaker_id": dst_id,
                "embeddings_moved": embeddings_moved,
                "facts_moved": facts_moved,
            },
            ensure_ascii=False,
        )
        self._result_pub.publish(ack)

    def _wipe_e2e_db_file(self) -> None:
        """Issue #2750 — стереть файл ``e2e_db_path`` перед каждым включением.

        НИКОГДА не трогает ``self._prod_db_path`` — вызывается только из
        ветки ``enabled=True`` в ``_apply_e2e_mode``, и путь для удаления
        берётся из ``self._e2e_db_path`` напрямую (константа узла, не
        значение параметра из запроса — через ``ros2 param set`` нельзя
        передать произвольный путь и стереть что-то ещё).
        """
        import os as _os

        for suffix in ("", "-wal", "-shm", "-journal"):
            path = f"{self._e2e_db_path}{suffix}"
            try:
                if _os.path.exists(path):
                    _os.remove(path)
            except OSError as exc:
                self.get_logger().warning(
                    f"⚠️ не удалось удалить {path!r} перед E2E-прогоном "
                    f"({type(exc).__name__}: {exc}) — база может унаследовать "
                    "профили с прошлого марафона"
                )

    def parameters_callback(self, params):
        """Issue #2750 — ``ros2 param set`` роутер (единственный на узле).

        Humble даёт только ``add_on_set_parameters_callback`` — валидирующий
        колбэк, вызываемый ДО применения значения, обязан вернуть
        ``SetParametersResult``. Побочный эффект (переключение активной БД)
        прямо внутри валидатора — тот же компромисс, что уже живёт в
        ``dialogue_node.parameters_callback`` (``barge_in_policy`` тут же
        переоткрывает состояние ноды) и ``tts_node.parameters_callback``
        (``volume_db``/``pitch_shift``/voice-параметры применяются
        синхронно) — один и тот же ROS2-паттерн этого репозитория, не
        отдельное изобретение под эту задачу.

        Почему параметр, а не топик (как было раньше — см. git history):
        ``seam_without_consumer.py`` (ADR-0021, issue #2118) сканирует
        ``create_publisher``/``create_subscription`` под ``src/`` — топик,
        который стирает БД по одному fire-and-forget сообщению без ответа
        и без авторизации, паблишер которого живёт в
        ``.github/workflows/scripts/e2e_voice_test.sh`` (bash, сторож его
        не видит), заслуженно падал как «шов без потребителя». Параметр
        сильнее и по существу: ``ros2 param set`` синхронный и возвращает
        успех/провал вызывающему в exit-коде, а ``e2e_mode`` объявлен
        (``declare_parameter``) — тот же приём, что ``barge_in_policy`` у
        dialogue_node (см. докстринг ``run_night_marathon.sh``:
        ``ssh <robot> 'ros2 param set /dialogue_node barge_in_policy
        classify'``).

        Только ``e2e_mode`` имеет побочный эффект; остальные параметры
        узла (``identify_threshold`` и т. п.) читаются один раз в
        ``__init__`` и здесь не перехватываются — ``ros2 param set`` на
        них молча проходит валидацию (значение в реестре параметров
        меняется), но узел его не подхватит без рестарта, как и раньше.
        """
        result_ok = True
        for param in params:
            if param.name == "e2e_mode":
                if not self._apply_e2e_mode(bool(param.value)):
                    result_ok = False
        return SetParametersResult(successful=result_ok)

    def _apply_e2e_mode(self, enabled: bool) -> bool:
        """Переключить активную БД дикторов боевая ↔ E2E. ``True`` — успех.

        Вызывается ТОЛЬКО из ``parameters_callback``. Зачем это вообще
        нужно (issue #2750): акт 2 ночного марафона («Знакомство») по
        сценарию регистрирует РЕАЛЬНЫЕ профили голосов — значит ему нужна
        ГАРАНТИРОВАННО пустая БД, иначе результат акта нечитаем (шаг
        ``n211_who_do_you_know`` перечисляет «всех, кого запомнил
        сегодня» и не проходит детерминированно на грязной базе). До этой
        правки чистую БД получали ssh-командой снаружи кода перед КАЖДЫМ
        прогоном (подтверждено владельцем в issue #2750): ``docker exec
        voice-assistant sh -c "cp /data/speakers.db
        /data/speakers.db.bak-<UTC>Z"`` + ``DELETE FROM embeddings; DELETE
        FROM speakers`` — ручная операция поверх БОЕВОГО файла, не в
        коде, и один раз стёрла профиль живого человека через 19 минут
        после регистрации.

        Если этот метод давал бы оператору лишь «ещё один ручной шаг
        вместо старого», человек рано или поздно вернулся бы к
        ``DELETE FROM`` по боевой — поэтому чистота E2E-БД гарантируется
        УЗЛОМ САМ: переход False→True удаляет прежний файл
        ``e2e_db_path`` (и его ``-wal``/``-shm``/``-journal``) и
        открывает его заново с нуля. Переход True→True (повторный
        ``ros2 param set ... true`` тем же значением посреди прогона)
        НЕ чистит базу второй раз — иначе повторный set обнулил бы уже
        накопленные за акт регистрации. Боевой ``db_path`` не трогается
        вообще никогда — это единственный код, которому разрешено уводить
        узел с боевой БД.
        """
        if enabled == self._e2e_mode_active:
            self.get_logger().info(
                f"🧪 e2e_mode: уже {'включён' if enabled else 'выключен'} — no-op"
            )
            return True

        target_path = self._e2e_db_path if enabled else self._prod_db_path
        try:
            with self._db_lock:
                old_db = self._db
                if enabled:
                    # Гарантия чистой базы — только на переходе в E2E-режим,
                    # см. docstring выше. Боевой self._prod_db_path в этой
                    # ветке не участвует вообще.
                    self._wipe_e2e_db_file()
                self._db = SpeakerDatabase(target_path)
                self._e2e_mode_active = enabled
                old_db.close()
        except Exception as exc:  # noqa: BLE001 — переключение БД не должно ронять ноду
            self.get_logger().error(
                f"❌ e2e_mode={'ON' if enabled else 'OFF'} переключение провалилось: "
                f"{type(exc).__name__}: {exc} — активная БД дикторов НЕ изменена "
                f"(осталась {'E2E' if self._e2e_mode_active else 'боевая'})"
            )
            return False

        # WARNING, не info: смена активной БД дикторов — событие, которое
        # обязано быть видно в `docker logs voice-assistant` без фильтров —
        # ровно то, чего не хватало в issue #2750, когда обнуление
        # происходило вне логов ноды вообще.
        self.get_logger().warning(
            f"🧪 speaker_id_node: e2e_mode={'ON' if enabled else 'OFF'} — "
            f"активная БД дикторов теперь {target_path!r} "
            f"({'боевая speakers.db НЕ используется, пока режим включён' if enabled else 'вернулись на боевую'})"
        )
        return True

    def _merge_identity(self, src_id: str, dst_id: str) -> Tuple[int, int]:
        """Issue #2440 — склейка через шов идентичности (эмбеддинги + факты).

        Запускает асинхронную операцию шва в отдельном event loop: колбэк
        ROS синхронный, а ``MemoryStore`` — async. Merge — редкая операторская
        команда, поэтому блокировка колбэка на несколько мс допустима.
        Возвращает ``(embeddings_moved, facts_moved)``.
        """
        import asyncio

        return asyncio.run(self._merge_identity_async(src_id, dst_id))

    async def _merge_identity_async(self, src_id: str, dst_id: str) -> Tuple[int, int]:
        """Асинхронное тело склейки: шов переносит и эмбеддинги, и факты."""
        from rob_box_harness.memory import SQLiteVoiceMemory
        from rob_box_voice.utils.identity_seam import VoiceIdentitySeam

        db_path = str(
            self.get_parameter("memory_db_path").value or "/data/harness_voice.db"
        )
        # Issue #2751 — второй писатель фактов (voice_facts в
        # voice_memory.db), см. docstring VoiceIdentitySeam.__init__.
        legacy_facts_db_path = str(
            self.get_parameter("voice_facts_db_path").value or "/data/voice_memory.db"
        )
        store = SQLiteVoiceMemory(db_path=db_path)
        try:
            await store.init()
            seam = VoiceIdentitySeam(
                self._db, store, legacy_facts_db_path=legacy_facts_db_path
            )
            return await seam.merge(src_id, dst_id)
        finally:
            try:
                await store.teardown()
            except Exception:  # noqa: BLE001
                pass

    def _do_register(
        self,
        name: str,
        embedding: np.ndarray,
        speaker_id: Optional[str],
        duration_sec: Optional[float] = None,
    ) -> bool:
        """Persist speaker embedding to DB and acknowledge.

        Issue W5-4 — использует ``register_or_merge()`` вместо голого
        ``register()``: если ``speaker_id`` не передан явно (обычный путь
        от LLM-тула register_speaker), сначала проверяется, не похож ли
        голос на уже известный профиль (порог REGISTER_MATCH_THRESHOLD,
        строже обычной идентификации) — и, при совпадении ИМЕНИ, эмбеддинг
        дописывается в существующий профиль вместо создания дубля. Именно
        отсутствие этой проверки было причиной бага «один голос — два
        профиля» (денчик/эйджик): раньше КАЖДЫЙ вызов register_speaker
        создавал новый speaker_id безусловно.

        ADR-0127 — если голос похож, а имя ДРУГОЕ, слияния не будет:
        заводится отдельный профиль, а сюда приезжают поля ``conflict_*``.
        Предупреждение печатает именно нода: ``logging``-строки из
        ``utils.speaker_embeddings`` в ``docker logs voice-assistant`` не
        попадают (run 35667281570 — в логе видно только то, что пишет
        ``self.get_logger()``), а оператору нужно увидеть, что робот
        принял двух людей за одного.

        Issue #2769 — ``duration_sec`` уходит в
        ``register_or_merge(duration_sec=...)``: если реплика короче
        ``MIN_REGISTER_AUDIO_DURATION_SEC``, тот бросает
        :class:`speaker_embeddings.AudioTooShortError` ДО записи в БД.
        Здесь это исключение — единственная точка перехвата на пути от
        LLM-тула register_speaker до диска: профиль НЕ создаётся, вместо
        обычного ack публикуется ``{"event": "register_error", "error":
        "too_short", ...}`` на тот же ``/voice/speaker/result`` (тем же
        каналом, которым уходит ``event="registered"`` — dialogue_node
        слушает оба и просит пользователя повторить фразу, см.
        ``_on_speaker_result``). Возвращает ``False`` в этом случае,
        ``True`` — если профиль реально создан/дополнен, чтобы вызывающий
        код (``_process_utterance`` / ``_on_register_request``) не считал
        отказ успешной регистрацией в логах и метриках.
        """
        try:
            outcome = self._db.register_or_merge(
                name, embedding, speaker_id=speaker_id, duration_sec=duration_sec
            )
        except _se_mod.AudioTooShortError as exc:
            self.get_logger().warning(
                f"⚠️ [issue #2769] Registration of '{name}' rejected — "
                f"audio too short for a reliable anchor: "
                f"{exc.duration_sec:.2f}s < {exc.min_required_sec:.1f}s "
                "required. Профиль НЕ создан — прошу повторить фразу."
            )
            ack = String()
            ack.data = json.dumps(
                {
                    "event": "register_error",
                    "error": "too_short",
                    "name": name,
                    "duration_s": round(exc.duration_sec, 2),
                    "min_required_s": exc.min_required_sec,
                },
                ensure_ascii=False,
            )
            self._result_pub.publish(ack)
            return False
        sid, reused = outcome
        if reused:
            self.get_logger().info(
                f"🔗 Speaker '{name}' merged into existing profile (id={sid[:8]}) "
                "— voice matched an already-known speaker, no duplicate created"
            )
        elif outcome.name_conflict:
            self.get_logger().warning(
                f"⚠️ Speaker '{name}' (id={sid[:8]}) — голос похож на уже "
                f"известного '{outcome.conflict_name}' "
                f"(id={outcome.conflict_speaker_id[:8]}, "
                f"score={outcome.conflict_score:.3f} >= порога слияния), но имя "
                f"другое: завожу ОТДЕЛЬНЫЙ профиль и НЕ переименовываю чужой "
                f"(ADR-0127). Если это один человек — склеить вручную через "
                f"/voice/speaker/merge {{\"src_speaker_id\": \"{sid}\", "
                f"\"dst_speaker_id\": \"{outcome.conflict_speaker_id}\"}}"
            )
        else:
            self.get_logger().info(f"✅ Speaker '{name}' registered (id={sid[:8]})")
        # Issue #1787 — новый профиль сразу получает внутреннюю кличку.
        self._ensure_epithet(sid)
        # Publish a registration-ack so dialogue_node can confirm verbally
        ack = String()
        ack_payload = {
            "event": "registered",
            "name": name,
            "speaker_id": sid,
            "reused_profile": reused,
        }
        if outcome.name_conflict:
            # ADR-0127 — dialogue_node получает повод переспросить («я уже
            # знаю голос, похожий на твой — вы разные люди?»), а разбор
            # прогона получает машиночитаемый след конфликта.
            ack_payload["voice_conflict"] = {
                "name": outcome.conflict_name,
                "speaker_id": outcome.conflict_speaker_id,
                "score": round(float(outcome.conflict_score), 4),
            }
        ack.data = json.dumps(ack_payload, ensure_ascii=False)
        self._result_pub.publish(ack)

        # Issue #2748 — до этой правки нода НИЧЕГО не публиковала в момент
        # регистрации, КРОМЕ служебного ack выше (``event: "registered"``),
        # который dialogue_node и
        # rob_box_harness.encounter.voice_adapter.VoiceEncounterAdapter
        # намеренно трактуют как «не сигнал присутствия» (см. их код) — то
        # есть vision_face_node._on_speaker_result (гейт на
        # ``payload.get('is_known')``) никогда не видел момент регистрации,
        # и имя не долетало до лицевой записи, даже когда в кадре было
        # ровно одно лицо. Публикуем ВТОРЫМ, отдельным сообщением полноценный
        # SpeakerMatch с ``source="register"`` — по форме неотличимый от
        # обычного identify()-результата (совместим с dialogue_node /
        # mcp_server / vision_face_node без правок их парсинга), но с
        # пометкой источника: имя названо самим человеком, доверия к нему
        # больше, чем к косинусу (issue #2747 — тот же самый голос сразу
        # после регистрации сам по себе НЕ всегда набирает калиброванный
        # identify_threshold).
        #
        # threshold=0.0 — принудительно берём self-similarity: embedding
        # только что записан В ГАЛЕРЕЮ sid (внутри register_or_merge →
        # register() чуть выше), поэтому лучший кандидат — ГАРАНТИРОВАННО
        # sid с cosine≈1.0 (сравнение вектора с самим собой).
        self_match = self._db.identify(embedding, threshold=0.0)
        if self_match is not None:
            self._publish_result(self_match, source="register")
            # Issue #2747 — открываем growth-сессию: следующие реплики,
            # идущие подряд без большого разрыва (см.
            # _apply_growth_session), будут считаться принадлежащими
            # ЭТОМУ спикеру и дописываться в его галерею — не по похожести
            # голоса, а по факту «только что явно представился». Имя берём
            # из ``self_match.name`` (каноническое написание из БД), а не
            # из аргумента ``name`` — при reuse-слиянии (ADR-0127) они
            # могут отличаться регистром/пробелами, и register() внутри
            # append_reference_embedding() иначе залогировал бы это как
            # неожиданное переименование.
            self._growth_session = {
                "speaker_id": sid,
                "name": self_match.name,
                "last_utterance_at": time.time(),
                "count": 0,
            }
        else:  # pragma: no cover — не должно происходить: sid только что создан
            self.get_logger().warning(
                f"⚠️ [issue #2748] Не удалось получить self-match для только "
                f"что зарегистрированного '{name}' (id={sid[:8]}) — "
                f"is_known=true не опубликован, слияние с лицом пропущено, "
                f"growth-сессия не открыта"
            )
        return True

    # ── Эпитеты (issue #1787) ─────────────────────────────────────────────────

    def _on_observe_request(self, msg: String) -> None:
        """Принять реплику известного спикера — вход для выбора эпитета.

        Expected JSON: ``{"speaker_id": "<uuid>", "text": "..."}``.

        Сама обработка уходит в тот же однопоточный executor, что и
        инференс: SQLite-соединение открыто с ``check_same_thread=False``,
        и параллельные записи из ROS-колбэка и из ``_do_register``
        конкурировали бы за один коннект. Один воркер = сериализация без
        отдельного мьютекса на БД.
        """
        try:
            data = json.loads(msg.data or "{}")
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning("⚠️ observe_request: invalid JSON ignored")
            return
        speaker_id = str(data.get("speaker_id", "")).strip()
        text = str(data.get("text", "")).strip()
        if not speaker_id or not text:
            return
        self._executor.submit(self._process_observation, speaker_id, text)

    def _process_observation(self, speaker_id: str, text: str) -> None:
        """Обновить темы спикера и, если есть повод, пересмотреть кличку."""
        try:
            with self._speech_log_lock:
                window = self._speech_log.setdefault(
                    speaker_id, collections.deque(maxlen=50)
                )
                window.append(text)
                messages = list(window)

            profile = self._db.get_speaker_profile(speaker_id)
            if profile is None:
                # Спикера удалили/слили между публикацией и обработкой.
                return

            tags = epithets.extract_tags(messages)
            sentiment = epithets.score_sentiment(messages)
            if tags:
                self._db.update_speaker_stats(
                    speaker_id,
                    tags=[t.cluster for t in tags],
                    sentiment_score=sentiment,
                )
            else:
                self._db.update_speaker_stats(speaker_id, sentiment_score=sentiment)

            if not profile["epithet"]:
                # Кличка ещё не назначена (или профиль старше миграции) —
                # ставим сразу, не дожидаясь накопления тем (research §5.1,
                # вариант 2: юзер получает кличку немедленно, она может
                # уточниться позже).
                self._assign_epithet(
                    speaker_id,
                    tags,
                    sentiment,
                    epithets.REASON_FIRST_SEEN,
                    messages=messages,
                )
                return

            # Пересмотр — только при новой доминирующей теме И не чаще
            # раза в MIN_REVIEW_INTERVAL_DAYS (research §4.1: стабильность
            # клички важнее реактивности).
            if not epithets.should_review(profile["last_epithet_review"], time.time()):
                return
            new_topic = epithets.find_distinctive_topic(profile["tags"], tags)
            if not new_topic:
                return
            # Кличку берём из НОВОЙ темы, а не из общего топа: старая тема
            # часто ещё лидирует по количеству упоминаний в окне (человек
            # не перестаёт говорить о прежнем разом), и без этой
            # перестановки пересмотр выдавал кандидата из того же
            # кластера — то есть ту же самую кличку.
            ordered = [t for t in tags if t.cluster == new_topic]
            ordered += [t for t in tags if t.cluster != new_topic]
            self._assign_epithet(
                speaker_id,
                ordered,
                sentiment,
                f"{epithets.REASON_NEW_TOPIC}:{new_topic}",
                messages=messages,
            )
        except Exception as exc:  # noqa: BLE001
            # Эпитет — вспомогательная метка. Любой сбой здесь не должен
            # ронять воркер, который в следующий момент считает эмбеддинг.
            self.get_logger().warning(
                f"⚠️ [issue 1787] observe failed for {speaker_id[:8]}: "
                f"{type(exc).__name__}: {exc}"
            )

    def _assign_epithet(
        self,
        speaker_id: str,
        tags,
        sentiment: float,
        reason: str,
        messages: Optional[list] = None,
    ) -> Optional[str]:
        """Слой 1 гибрида: подобрать свободную кличку из словаря.

        ``taken_epithets(exclude_speaker_id=…)`` — то самое место, где
        закрывается коллизия тёзок: кандидат не может совпасть ни с одной
        уже выданной кличкой.

        Записав словарного кандидата, узел просит LLM придумать своё
        слово (слой 2). Порядок именно такой — сначала пишем, потом
        спрашиваем: словарь отвечает мгновенно и офлайн, поэтому робот
        никогда не остаётся без клички, даже если LLM недоступна или
        вернёт мусор.
        """
        candidate = epithets.choose_epithet(
            tags,
            speaker_id=speaker_id,
            taken=self._db.taken_epithets(exclude_speaker_id=speaker_id),
            sentiment=sentiment,
        )
        if not self._db.set_epithet(speaker_id, candidate.label, reason):
            return None
        self.get_logger().info(
            f"🔤 [issue 1787] Эпитет {speaker_id[:8]} → {candidate.label!r} "
            f"(кластер={candidate.source_cluster}, {reason})"
        )
        self._request_llm_epithet(speaker_id, candidate, messages or [])
        return candidate.label

    def _request_llm_epithet(self, speaker_id: str, candidate, messages: list) -> None:
        """Попросить dialogue_node придумать кличку через LLM (слой 2)."""
        pub = getattr(self, "_epithet_request_pub", None)
        if pub is None:
            return
        try:
            msg = String()
            msg.data = json.dumps(
                {
                    "speaker_id": speaker_id,
                    "fallback": candidate.label,
                    "cluster": candidate.source_cluster,
                    "hints": list(
                        epithets.EPITHET_LEXICON.get(
                            candidate.source_cluster, epithets.DEFAULT_POOL_NEUTRAL
                        )[:3]
                    ),
                    "messages": [m for m in messages[-5:] if m],
                },
                ensure_ascii=False,
            )
            pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ [issue 1787] epithet_request publish failed: "
                f"{type(exc).__name__}: {exc}"
            )

    def _on_epithet_result(self, msg: String) -> None:
        """Принять кличку, придуманную LLM, и применить её после проверки.

        Expected JSON: ``{"speaker_id": "<uuid>", "epithet": "Кулибин"}``.

        Всё, что не прошло ``sanitize_llm_epithet`` (фраза вместо слова,
        цифры, уже занятая кличка), молча отбрасывается — в профиле
        остаётся словарный кандидат. Это единственное разумное поведение:
        текст пришёл из модели, которую попросили «придумать слово», и
        доверять ему как команде нельзя.
        """
        try:
            data = json.loads(msg.data or "{}")
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning("⚠️ epithet result: invalid JSON ignored")
            return
        speaker_id = str(data.get("speaker_id", "")).strip()
        raw = data.get("epithet")
        if not speaker_id:
            return

        label = epithets.sanitize_llm_epithet(
            raw, taken=self._db.taken_epithets(exclude_speaker_id=speaker_id)
        )
        if not label:
            self.get_logger().info(
                f"🔤 [issue 1787] LLM-кличка {raw!r} отклонена — "
                f"остаётся словарная у {speaker_id[:8]}"
            )
            return
        if self._db.set_epithet(speaker_id, label, epithets.REASON_LLM):
            self.get_logger().info(
                f"🔤 [issue 1787] LLM переименовала {speaker_id[:8]} → {label!r}"
            )

    def _ensure_epithet(self, speaker_id: str) -> None:
        """Выдать кличку сразу при регистрации, если её ещё нет.

        Без этого новый профиль жил бы без эпитета до первой реплики,
        прилетевшей в ``/voice/speaker/observe`` — а регистрация как раз
        и есть момент, когда робот впервые «знакомится» с голосом.
        """
        try:
            if self._db.get_epithet(speaker_id):
                return
            with self._speech_log_lock:
                messages = list(self._speech_log.get(speaker_id, ()))
            self._assign_epithet(
                speaker_id,
                epithets.extract_tags(messages),
                epithets.score_sentiment(messages),
                epithets.REASON_FIRST_SEEN,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"⚠️ [issue 1787] Не удалось назначить эпитет "
                f"{speaker_id[:8]}: {type(exc).__name__}: {exc}"
            )

    def _publish_result(
        self, match: Optional[SpeakerMatch], source: Optional[str] = None
    ) -> None:
        """Serialise and publish the speaker identification result.

        Issue #2748 — ``source`` — необязательная метка происхождения
        сигнала. Обычная идентификация по фразе её не ставит (совместимость
        со старым форматом payload, который уже читают dialogue_node /
        mcp_server / vision_face_node). ``source="register"`` ставит
        ``_do_register()`` — сигнал «имя названо человеком при регистрации»,
        а не «косинус посчитал похожим»; vision_face_node принимает его
        наравне с обычным узнаванием (см. ``_on_speaker_result`` там —
        гейт только на ``is_known``, поле ``source`` не проверяется).
        """
        if match:
            payload = {
                "is_known": True,
                "speaker_id": match.speaker_id,
                "name": match.name,
                "confidence": round(match.confidence, 4),
                # Issue #1787 — внутренняя кличка. None до первой реплики
                # (профиль из старой БД) — потребитель обязан это терпеть.
                "epithet": match.epithet,
            }
            if source:
                payload["source"] = source
            self.get_logger().info(
                f"📢 Publishing: is_known=true name={match.name!r} "
                f"epithet={match.epithet!r} conf={match.confidence:.3f}"
                + (f" source={source!r}" if source else "")
            )
        else:
            payload = {"is_known": False}
            self.get_logger().info("📢 Publishing: is_known=false")

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._result_pub.publish(msg)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        if hasattr(self, "_executor"):
            self._executor.shutdown(wait=False)
        if hasattr(self, "_db"):
            self._db.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeakerIdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
