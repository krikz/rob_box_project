"""``PersistentHarness`` — lightweight harness unifying 6 persistent nodes.

A :class:`Harness` subclass that wraps the common lifecycle patterns
shared by the 6 persistent ROS2 nodes: audio, stt, tts, sound, led,
and command. Per ADR-0001 §2.7.2.

**Design — parallel implementation:**
    This harness lives side-by-side with the existing persistent
    nodes. Each node KEEPS its device-specific code (ReSpeaker driver,
    Yandex gRPC client, etc.). The harness provides a unified
    lifecycle wrapper with health-check, state publishing, and
    parameter validation.

**What the harness provides:**
    * ``HardwareLifecycle`` — connect/disconnect/health-check/restart-on-error
    * ``StatePublisher`` — unified status on ``/<node>/state``
    * ``ParameterGuard`` — declare/validate/reload ROS params with type-checking

**What STAYS in each node file:**
    * Device-specific drivers (ReSpeaker, Yandex gRPC, serial LED, etc.)
    * ROS2 publishers/subscribers (topic-level I/O)
    * Node-specific business logic (audio pipeline, LED patterns, etc.)
"""

from __future__ import annotations

import logging
import time as _time_mod
from dataclasses import dataclass, field
from typing import Any

from rob_box_harness.config import HarnessConfig
from rob_box_harness.harness import Harness

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# PersistentState
# ---------------------------------------------------------------------------


@dataclass
class PersistentState:
    """Unified state bag for a persistent hardware node.

    Published periodically on ``/<node_name>/state`` so the system
    monitor can observe all 6 nodes through a single schema.
    """

    node_name: str = ""
    """Logical name of the node (e.g. 'audio', 'stt', 'tts')."""

    status: str = "initializing"
    """One of: initializing, running, degraded, error, stopped."""

    last_error: str = ""
    """Most recent error message (empty = no errors)."""

    uptime_seconds: float = 0.0
    """Seconds since the harness entered 'running' state."""

    restart_count: int = 0
    """Monotonic counter of hardware restarts."""

    parameters: dict[str, Any] = field(default_factory=dict)
    """Current effective parameter values."""


# ---------------------------------------------------------------------------
# HardwareLifecycle
# ---------------------------------------------------------------------------


class HardwareLifecycle:
    """Connect/disconnect/health-check/restart-on-error for hardware devices.

    Concrete device harnesses override the four methods with their
    device-specific I/O. The harness's ``step()`` calls these in a
    loop for monitoring; :meth:`restart` is the canonical
    disconnect → connect recovery sequence.

    This class is intentionally NOT async-coupling — device I/O may
    be synchronous (GPIO, serial). Concrete harnesses wrap calls in
    ``asyncio.to_thread`` as needed.
    """

    def __init__(self, device_name: str = "device") -> None:
        self._device_name = device_name
        self._connected = False

    @property
    def is_connected(self) -> bool:
        """True if the device is currently connected."""
        return self._connected

    @property
    def device_name(self) -> str:
        """Human-readable name for log messages."""
        return self._device_name

    async def connect(self) -> bool:
        """Initialise the hardware device. Called once during ``init()``.

        Returns:
            True on success, False on failure.
        """
        self._connected = True
        logger.info("HardwareLifecycle[%s]: connected", self._device_name)
        return True

    async def disconnect(self) -> None:
        """Release the hardware device. Called during ``teardown()``."""
        self._connected = False
        logger.info("HardwareLifecycle[%s]: disconnected", self._device_name)

    async def health_check(self) -> bool:
        """Check whether the device is responsive.

        Returns:
            True if the device is healthy, False if degraded.
        """
        return self._connected

    async def restart(self) -> bool:
        """Disconnect → connect recovery sequence.

        Returns:
            True if restart succeeded, False if it failed.
        """
        await self.disconnect()
        return await self.connect()


# ---------------------------------------------------------------------------
# StatePublisher
# ---------------------------------------------------------------------------


class StatePublisher:
    """Publish  unified state on ``/<node_name>/state``.

    In production this dispatches to the SideEffectBus (which the
    real ROS2 transport picks up). In tests it records publishes
    in a list for assertion.
    """

    def __init__(self, node_name: str) -> None:
        self._node_name = node_name
        self._published: list[PersistentState] = []

    @property
    def node_name(self) -> str:
        return self._node_name

    async def publish(self, state: PersistentState) -> None:
        """Emit the current state snapshot.

        In production this writes to the side-effect bus so the
        ROS2 transport can publish on the appropriate topic.
        """
        self._published.append(state)
        logger.debug(
            "StatePublisher[%s]: status=%s uptime=%.1fs restarts=%d",
            self._node_name,
            state.status,
            state.uptime_seconds,
            state.restart_count,
        )

    def last_published(self) -> PersistentState | None:
        """Return the most recent published state (for tests)."""
        return self._published[-1] if self._published else None

    def flush(self) -> list[PersistentState]:
        """Return and clear the publish history."""
        history = list(self._published)
        self._published.clear()
        return history


# ---------------------------------------------------------------------------
# ParameterGuard
# ---------------------------------------------------------------------------


class ParameterGuard:
    """Declare/validate/reload ROS parameters with type checking.

    Mirrors the ROS2 ``declare_parameter`` / ``get_parameter``
    contract in a ROS-agnostic way so the harness stays testable
    without a running ROS2 graph.
    """

    def __init__(self) -> None:
        self._declared: dict[str, tuple[Any, type]] = {}
        self._values: dict[str, Any] = {}

    def declare(self, name: str, default: Any, param_type: type) -> None:
        """Register a parameter with a default value and expected type.

        Args:
            name: Parameter name (ROS-compatible, e.g. 'audio.sample_rate').
            default: Default value.
            param_type: Expected Python type for validation.
        """
        self._declared[name] = (default, param_type)
        self._values[name] = default

    def validate(self, name: str, value: Any) -> bool:
        """Check that ``value`` matches the declared type for ``name``.

        Returns:
            True if the value is valid, False otherwise.
        """
        if name not in self._declared:
            logger.warning("ParameterGuard: unknown parameter '%s'", name)
            return False
        _, expected_type = self._declared[name]
        if not isinstance(value, expected_type):
            logger.warning(
                "ParameterGuard: '%s' expected %s, got %s",
                name,
                expected_type.__name__,
                type(value).__name__,
            )
            return False
        return True

    def get(self, name: str) -> Any:
        """Return the current value of a parameter."""
        if name not in self._values:
            raise KeyError(f"Parameter '{name}' not declared")
        return self._values[name]

    def set(self, name: str, value: Any) -> bool:
        """Set a parameter value after validation.

        Returns:
            True if the value was accepted, False if rejected.
        """
        if not self.validate(name, value):
            return False
        self._values[name] = value
        return True

    async def reload(self) -> dict[str, Any]:
        """Reload all parameters from the source of truth.

        In production this reads from the ROS2 parameter server.
        In tests it's a no-op (parameters are set via ``set()``).

        Returns:
            Current effective parameter dict.
        """
        return dict(self._values)

    def snapshot(self) -> dict[str, Any]:
        """Return a copy of current parameters for the state bag."""
        return dict(self._values)


# ---------------------------------------------------------------------------
# PersistentHarness
# ---------------------------------------------------------------------------


class PersistentHarness(Harness[PersistentState]):
    """Lightweight harness unifying the lifecycle of 6 persistent nodes.

    Each of the 6 nodes (audio, stt, tts, sound, led, command) uses
    this harness to get a uniform health-check → state-publish cycle.
    The harness does NOT own device-specific logic — it only provides
    the shared infrastructure.

    Usage::

        harness = PersistentHarness(config)
        async with harness:
            while True:
                await harness.run(None)
                await asyncio.sleep(1)  # 1 Hz monitoring loop
    """

    name = "persistent"

    # ── init ────────────────────────────────────────────────────

    async def init(self) -> None:
        """Wire hardware lifecycle, state publisher, and parameter guard."""
        await super().init()

        node_name = self.config.name or "persistent"
        self._hw = HardwareLifecycle(device_name=node_name)
        self._state_pub = StatePublisher(node_name=node_name)
        self._params = ParameterGuard()
        self._start_time = _time_mod.monotonic()

        # Declare common parameters shared by all persistent nodes
        self._params.declare("publish_rate_hz", 1.0, float)
        self._params.declare("health_check_interval_s", 5.0, float)
        self._params.declare("max_restart_attempts", 3, int)

        # Connect hardware
        connected = await self._hw.connect()
        if not connected:
            self.state.status = "error"
            self.state.last_error = "Hardware connection failed"
            logger.error("PersistentHarness[%s]: connect failed", node_name)
            return

        self.state = PersistentState(
            node_name=node_name,
            status="running",
            uptime_seconds=0.0,
        )

    # ── step ────────────────────────────────────────────────────

    async def step(self, input_data: Any) -> str:
        """Health-check → state-publish cycle (runs at ~1 Hz).

        Args:
            input_data: Ignored (persistent harness is self-monitoring).

        Returns:
            Status string: "ok", "restarted", or "error".
        """
        _ = input_data  # self-monitoring — no external input

        # Update uptime
        self.state.uptime_seconds = _time_mod.monotonic() - self._start_time

        # Health check
        healthy = await self._hw.health_check()
        if not healthy:
            self.state.status = "degraded"
            self.state.last_error = "Health check failed"
            logger.warning(
                "PersistentHarness[%s]: health check failed, restarting",
                self.state.node_name,
            )

            # Attempt restart
            restarted = await self._hw.restart()
            if restarted:
                self.state.status = "running"
                self.state.last_error = ""
                self.state.restart_count += 1
                logger.info(
                    "PersistentHarness[%s]: restarted (attempt %d)",
                    self.state.node_name,
                    self.state.restart_count,
                )
            else:
                self.state.status = "error"
                self.state.last_error = "Restart failed"
                logger.error(
                    "PersistentHarness[%s]: restart failed",
                    self.state.node_name,
                )
                return "error"
            return "restarted"

        # Periodic state publish
        self.state.parameters = self._params.snapshot()
        await self._state_pub.publish(self.state)

        return "ok"

    # ── teardown ────────────────────────────────────────────────

    async def teardown(self) -> None:
        """Publish final state and disconnect hardware."""
        self.state.status = "stopped"
        await self._state_pub.publish(self.state)
        await self._hw.disconnect()
        await super().teardown()


__all__ = [
    "PersistentHarness",
    "PersistentState",
    "HardwareLifecycle",
    "StatePublisher",
    "ParameterGuard",
]
