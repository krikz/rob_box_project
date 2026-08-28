#!/usr/bin/env python3
"""
speaker_id_node.py — Real-time speaker identification using resemblyzer d-vectors.

Subscribes:
    /audio/speech_audio  (AudioData)  — full speech utterance from audio_node
    /voice/speaker/register (String)  — JSON {"name":"Иван"} — register current speaker

Publishes:
    /voice/speaker/result (String) — JSON SpeakerMatch or {"is_known":false}

Parameters:
    db_path              (str)   — path to SQLite DB       [/data/speakers.db]
    identify_threshold   (float) — cosine similarity gate  [0.75]
    sample_rate          (int)   — PCM sample rate         [16000]
    enabled              (bool)  — enable/disable node     [true]
"""

import collections
import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Deque, Optional, Tuple

import numpy as np
import rclpy
from audio_common_msgs.msg import AudioData
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

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
        self.declare_parameter("identify_threshold", 0.75)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("enabled", True)
        # Issue #1160 — Prometheus metrics endpoint. 9112 — speaker_id_node.
        self.declare_parameter("metrics_port", 9112)

        self._enabled: bool = self.get_parameter("enabled").value
        self._sample_rate: int = self.get_parameter("sample_rate").value
        db_path: str = self.get_parameter("db_path").value
        threshold: float = self.get_parameter("identify_threshold").value

        if not self._enabled:
            self.get_logger().info("⚠️ speaker_id_node disabled via parameter")
            return

        # ── Speaker DB (thread-safe via lock) ─────────────────────────────────
        self._db = SpeakerDatabase(db_path)
        # Patch threshold from parameter
        import rob_box_voice.utils.speaker_embeddings as _se_mod

        _se_mod.IDENTIFY_THRESHOLD = threshold
        self.get_logger().info(f"✅ SpeakerDatabase opened: {db_path} threshold={threshold}")

        # ── Pending registration ───────────────────────────────────────────────
        # Set when user says "запомни мой голос как [name]" via /voice/speaker/register.
        # The NEXT speech utterance will be registered under this name.
        self._pending_register_name: Optional[str] = None
        self._pending_register_lock = threading.Lock()

        # Recent embeddings ring-buffer: (timestamp, embedding) — keep last 20 utterances
        # LLM may take 2-5s to call register_speaker, so a single _last_embedding
        # can be overwritten by ambient noise. Keep a window instead.
        self._recent_embeddings: Deque[Tuple[float, np.ndarray]] = collections.deque(maxlen=20)
        self._MAX_EMBED_AGE_SEC: float = 30.0

        # ── Thread pool for inference (non-blocking ROS callbacks) ────────────
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="speaker_id")
        # Warm up resemblyzer model immediately so first real inference is fast
        self._executor.submit(self._warmup)

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
            best_ts = 0.0
            for ts, emb in reversed(self._recent_embeddings):
                if now - ts <= self._MAX_EMBED_AGE_SEC and ts > best_ts:
                    best_embedding = emb
                    best_ts = ts
            if best_embedding is not None:
                self._executor.submit(
                    self._do_register, name, best_embedding, speaker_id_hint
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
            self._recent_embeddings.append((time.time(), embedding))
            pending_name = self._pending_register_name
            self._pending_register_name = None

        if pending_name:
            self._do_register(pending_name, embedding, speaker_id=None)
            # After registration, also publish as a known speaker result
            match = self._db.identify(embedding)
            self._publish_result(match)
            self.get_logger().info(
                f"✅ Registered & identified '{pending_name}' "
                f"(inference {elapsed:.0f} ms)"
            )
            # Issue #1160 — Prometheus metrics: только что зарегистрированный
            # спикер считается known.
            record_speaker_recognize(
                known=True,
                confidence=match.confidence if match else None,
            )
            return

        match = self._db.identify(embedding)
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

    def _do_register(
        self,
        name: str,
        embedding: np.ndarray,
        speaker_id: Optional[str],
    ) -> None:
        """Persist speaker embedding to DB and acknowledge."""
        sid = self._db.register(name, embedding, speaker_id=speaker_id)
        self.get_logger().info(f"✅ Speaker '{name}' registered (id={sid[:8]})")
        # Publish a registration-ack so dialogue_node can confirm verbally
        ack = String()
        ack.data = json.dumps(
            {"event": "registered", "name": name, "speaker_id": sid},
            ensure_ascii=False,
        )
        self._result_pub.publish(ack)

    def _publish_result(self, match: Optional[SpeakerMatch]) -> None:
        """Serialise and publish the speaker identification result."""
        if match:
            payload = {
                "is_known": True,
                "speaker_id": match.speaker_id,
                "name": match.name,
                "confidence": round(match.confidence, 4),
            }
            self.get_logger().info(
                f"📢 Publishing: is_known=true name={match.name!r} conf={match.confidence:.3f}"
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
