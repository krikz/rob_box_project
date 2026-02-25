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

import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Optional

import numpy as np
import rclpy
from audio_common_msgs.msg import AudioData
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .utils.speaker_embeddings import SpeakerDatabase, SpeakerMatch


class SpeakerIdNode(Node):
    """Voice-based speaker identification node."""

    def __init__(self) -> None:
        super().__init__("speaker_id_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("db_path", "/data/speakers.db")
        self.declare_parameter("identify_threshold", 0.75)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("enabled", True)

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

        # Latest embedding from last processed utterance (for registration)
        self._last_embedding: Optional[np.ndarray] = None

        # ── Thread pool for inference (non-blocking ROS callbacks) ────────────
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="speaker_id")

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

        # ── Subscribers ───────────────────────────────────────────────────────
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

        self.get_logger().info("🎙️ speaker_id_node ready")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_speech_audio(self, msg: AudioData) -> None:
        """Received a complete speech utterance — run inference asynchronously."""
        pcm_bytes = bytes(msg.data)
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
        with self._pending_register_lock:
            if self._last_embedding is not None:
                embedding = self._last_embedding
                self._last_embedding = None
                self._executor.submit(
                    self._do_register, name, embedding, speaker_id_hint
                )
                self.get_logger().info(f"📝 Registering '{name}' from last utterance")
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
            self._publish_result(None)
            return

        elapsed = (time.monotonic() - t0) * 1000

        # Store as latest embedding for possible registration
        with self._pending_register_lock:
            self._last_embedding = embedding
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
            return

        match = self._db.identify(embedding)
        if match:
            self.get_logger().info(
                f"👤 Speaker: '{match.name}' confidence={match.confidence:.3f} "
                f"({elapsed:.0f} ms)"
            )
        else:
            self.get_logger().debug(f"👤 Speaker: unknown ({elapsed:.0f} ms)")

        self._publish_result(match)

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
        else:
            payload = {"is_known": False}

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
