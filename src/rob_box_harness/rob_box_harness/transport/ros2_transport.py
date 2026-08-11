"""ROS2Transport — real Transport for ROS2 topics.

Bridges sync ROS2 subscriptions to async harness events via
``asyncio.run_coroutine_threadsafe``. Follows the Transport ABC
contract from ``rob_box_harness.transport``.

Per ADR-0001 §2.4.5: transport owns the ROS2 subscription lifecycle;
the harness stays ROS-agnostic.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Any

# ── rclpy import guard ──────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

    ROS2_AVAILABLE = True
except ImportError:  # pragma: no cover
    rclpy = None  # type: ignore[assignment]
    Node = None  # type: ignore[assignment]
    qos_profile_sensor_data = 10  # type: ignore[assignment]
    qos_profile_system_default = 10  # type: ignore[assignment]
    ROS2_AVAILABLE = False

from rob_box_harness.transport import (
    BaseTransport,
    KeyEvent,
    TelegramUpdate,
    VadEvent,
)

_logger = logging.getLogger(__name__)


class ROS2Transport(BaseTransport):
    """Real Transport implementation bridging ROS2 to harness events.

    Creates subscriptions to STT and VAD topics. Each ROS2 callback
    uses ``asyncio.run_coroutine_threadsafe`` to safely hand the event
    over to the harness's event loop.

    Parameters
    ----------
    node:
        An active rclpy ``Node`` for creating subscriptions.
    """

    name = "ros2"

    def __init__(self, node: Node) -> None:
        super().__init__(name="ros2")
        self._node: Node = node
        self._event_loop: asyncio.AbstractEventLoop | None = None
        self._subscriptions: list[Any] = []
        self._initialized: bool = False

    # ── lifecycle ───────────────────────────────────────────────

    async def init(self) -> None:
        """Create ROS2 subscriptions.

        Safe to call once. Subsequent calls are no-ops.
        """
        if self._initialized:
            return

        self._event_loop = asyncio.get_running_loop()

        # STT result: String msg on /voice/stt/result
        self._subscriptions.append(
            self._node.create_subscription(
                str,  # rclpy builtin String msg
                "/voice/stt/result",
                self._on_stt_result,
                qos_profile=qos_profile_sensor_data,
            )
        )

        # VAD: String msg on /audio/vad
        self._subscriptions.append(
            self._node.create_subscription(
                str,
                "/audio/vad",
                self._on_vad,
                qos_profile=qos_profile_system_default,
            )
        )

        self._initialized = True
        _logger.info("ROS2Transport initialized — subscribed to /voice/stt/result, /audio/vad")

    async def teardown(self) -> None:
        """Destroy all ROS2 subscriptions. Idempotent."""
        if not self._initialized:
            return

        for sub in self._subscriptions:
            self._node.destroy_subscription(sub)
        self._subscriptions.clear()
        self._event_loop = None
        self._initialized = False
        _logger.info("ROS2Transport torndown — all subscriptions destroyed")

    # ── ROS2 callbacks (sync → async bridge) ─────────────────────

    def _on_stt_result(self, msg: Any) -> None:
        """ROS2 subscription callback for /voice/stt/result.

        Bridges from sync ROS2 callback thread to the harness's
        async event loop via ``run_coroutine_threadsafe``.
        """
        text = msg.data if hasattr(msg, "data") else str(msg)
        if self._event_loop is not None:
            asyncio.run_coroutine_threadsafe(
                self._handle_stt_result(text),
                self._event_loop,
            )

    def _on_vad(self, msg: Any) -> None:
        """ROS2 subscription callback for /audio/vad.

        Bridges from sync ROS2 callback thread to the harness's
        async event loop via ``run_coroutine_threadsafe``.
        """
        raw = msg.data if hasattr(msg, "data") else str(msg)
        # Parse VAD message: expect "speech" or "silence"
        is_speech = "speech" in raw.lower()
        if self._event_loop is not None:
            asyncio.run_coroutine_threadsafe(
                self._handle_vad(is_speech),
                self._event_loop,
            )

    # ── async handlers (call into Transport ABC) ─────────────────

    async def _handle_stt_result(self, text: str) -> None:
        """Forward STT result to bound harness handler."""
        await self._dispatch("stt_result", {"text": text, "confidence": 1.0})

    async def _handle_vad(self, is_speech: bool) -> None:
        """Forward VAD event to bound harness handler."""
        event = VadEvent(is_speech=is_speech)
        await self._dispatch("vad", event)

    # ── Transport ABC abstract methods ───────────────────────────

    async def on_stt_result(self, text: str, *, confidence: float) -> None:
        """Not used by ROS2 transport — STT arrives via subscription callback."""
        await self._dispatch("stt_result", {"text": text, "confidence": confidence})

    async def on_vad(self, event: VadEvent) -> None:
        """Not used by ROS2 transport — VAD arrives via subscription callback."""
        await self._dispatch("vad", event)

    async def on_telegram_update(self, update: TelegramUpdate) -> None:
        """Not used by voice-side ROS2 transport. No-op."""
        await self._dispatch("telegram_update", update)

    async def on_key_event(self, event: KeyEvent) -> None:
        """Not used by voice-side ROS2 transport. No-op."""
        await self._dispatch("key_event", event)
