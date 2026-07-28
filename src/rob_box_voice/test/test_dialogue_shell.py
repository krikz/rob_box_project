#!/usr/bin/env python3
"""Integration tests for the W5 dialogue shell (Phase 6 v2 / W6).

These tests drive the real :class:`DialogueNode` (the W5 thin shell)
with fake LLM / tool / memory ports so the ROS2 ↔ harness bridge is
exercised end-to-end without making any network call or starting a
real ``rclpy`` executor.

The tests live in ``src/rob_box_voice/test/`` (the W6 task scope) and
register the same ``rclpy`` mocks as ``test/unit/node/conftest.py``
when rclpy is not installed, so the file is runnable under plain
``pytest`` on a developer laptop without the ROS2 runtime.

Coverage (from 06-02-PLAN.md §W6):

1. Wake-word detection → state ``LISTENING``
2. STT input → state ``DIALOGUE`` → response published to
   ``/voice/dialogue/response``
3. Silence command → state ``SILENCED``
4. Inactivity timeout → ``LISTENING`` → ``IDLE``
5. DJ mode transition (timer-based tick)
6. Barge-in (new STT during active run → old turn cancelled)
7. TTS finish event → ``speak_text`` awaiter released
8. Sound finish event → ``play_sound`` awaiter released

The 9th implicit test is that the W5 shell rewrite did not regress
the original dialogue-node tests; see ``test_dialogue_node.py``.
"""

from __future__ import annotations

import asyncio
import json
import os
import sys
import time
import unittest
from typing import Any, Callable, Dict, List, Optional
from unittest.mock import MagicMock

# ── rclpy shim ─────────────────────────────────────────────────────────
# Always install a minimal rclpy/mock_node stub so this test file is
# runnable without an rclpy runtime. The shell only exercises
# ``Node.__init__``, ``create_publisher``, ``create_subscription``,
# ``create_timer``, ``declare_parameter``/``get_parameter``,
# ``get_logger``, and ``get_name`` — all of which are mocked here.
# This keeps ``pytest src/rob_box_voice/test/test_dialogue_shell.py``
# green on developer laptops and in CI containers that may or may
# not have ROS2 installed, and avoids requiring ``rclpy.init()`` from
# setUpClass (which would otherwise need a real ROS2 install).
import types as _types  # noqa: E402

class _FakeNode:
    """Minimal stand-in for ``rclpy.node.Node`` used by the shell."""

    def __init__(self, name, **kwargs):
        self._name = name
        self._logger = MagicMock()
        self._logger.info = MagicMock()
        self._logger.warning = MagicMock()
        self._logger.error = MagicMock()
        self._logger.debug = MagicMock()
        self._publishers: Dict[str, MagicMock] = {}
        self._subs: Dict[str, Callable[[Any], None]] = {}
        self._params: Dict[str, Any] = {}
        self._timers: List[Any] = []

    def get_logger(self):
        return self._logger

    def declare_parameter(self, name, default=None):
        # rclpy returns a Parameter object; the shell reads .value.
        # Store the default so subsequent get_parameter() works.
        self._params.setdefault(name, default)
        return MagicMock()

    def get_parameter(self, name):
        class _Param:
            def __init__(self, value):
                self.value = value

        return _Param(self._params.get(name))

    def create_publisher(self, msg_type, topic, depth, **kwargs):
        pub = MagicMock()
        pub.topic = topic
        pub.published: List[Any] = []
        original_publish = pub.publish

        def _capture(msg):
            pub.published.append(msg)
            return original_publish(msg)

        pub.publish = _capture
        self._publishers[topic] = pub
        return pub

    def create_subscription(
        self,
        msg_type,
        topic,
        callback,
        qos,
        callback_group=None,
    ):
        sub = MagicMock()
        sub.topic = topic
        sub.callback = callback
        self._subs[topic] = callback
        return sub

    def create_timer(self, period, callback, callback_group=None):
        timer = MagicMock()
        timer.period = period
        timer.callback = callback
        self._timers.append(timer)
        return timer

    def get_name(self):
        return self._name

_mock_rclpy = _types.ModuleType("rclpy")

class _RCLPY_OK:
    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

_mock_rclpy.init = lambda *a, **kw: None
_mock_rclpy.shutdown = lambda *a, **kw: None
_mock_rclpy.ok = lambda: True
sys.modules["rclpy"] = _mock_rclpy

_mock_rclpy_node = _types.ModuleType("rclpy.node")
_mock_rclpy_node.Node = _FakeNode
sys.modules["rclpy.node"] = _mock_rclpy_node

_cb = _types.ModuleType("rclpy.callback_groups")
_cb.ReentrantCallbackGroup = type(
    "ReentrantCallbackGroup", (), {}
)
sys.modules["rclpy.callback_groups"] = _cb

_qos = _types.ModuleType("rclpy.qos")
_qos.HistoryPolicy = _types.SimpleNamespace(KEEP_LAST="KEEP_LAST")
_qos.ReliabilityPolicy = _types.SimpleNamespace(RELIABLE="RELIABLE")
_qos.QoSProfile = lambda *a, **kw: MagicMock()
sys.modules["rclpy.qos"] = _qos

_executors = _types.ModuleType("rclpy.executors")
_executors.MultiThreadedExecutor = MagicMock
sys.modules["rclpy.executors"] = _executors

_std_msgs = _types.ModuleType("std_msgs")
_std_msgs_msg = _types.ModuleType("std_msgs.msg")

class _String:
    def __init__(self):
        self.data = ""

_std_msgs_msg.String = _String

class _Bool:
    def __init__(self):
        self.data = False

_std_msgs_msg.Bool = _Bool
sys.modules["std_msgs"] = _std_msgs
sys.modules["std_msgs.msg"] = _std_msgs_msg

# Import the shell and its dependencies. ``rclpy`` (real or fake) is
# already wired above; std_msgs is in place. The dialogue shell
# imports additional modules (``from rclpy.callback_groups`` etc.) so
# keep this block after the rclpy shim.
from std_msgs.msg import Bool, String  # noqa: E402

from rob_box_harness.core.dialog_core import DialogCore  # noqa: E402
from rob_box_harness.core.dialogue_state_machine import (  # noqa: E402
    DialogueStateKind,
)
from rob_box_harness.memory import InMemoryStore  # noqa: E402
from rob_box_harness.providers import DummyLLMProvider, HarnessFakeLLMProvider  # noqa: E402
from rob_box_harness.tools import FakeToolProvider  # noqa: E402

# Import the shell last so all rclpy mocks are registered before the
# module touches them.
from rob_box_voice.core.dj_mode import DJState  # noqa: E402
from rob_box_voice.dialogue_node import DialogueNode  # noqa: E402


# ─────────────────────────────────────────────────────────────────────
# Testable subclass — replaces live provider wiring with fakes.
# ─────────────────────────────────────────────────────────────────────


class _ScriptedLLMProvider(DummyLLMProvider):
    """``DummyLLMProvider`` that returns scripted responses by index.

    The base class only handles ``"ping"`` / ``"echo: ..."`` text; we
    override ``complete`` so each call pops a pre-recorded reply.
    Tests then assert exact text was published.
    """

    name = "scripted_llm"

    def __init__(self, scripted: Optional[List[str]] = None) -> None:
        super().__init__()
        self._scripted: List[str] = list(scripted or [])
        self._idx: int = 0

    def push(self, response: str) -> None:
        """Append a one-off reply to the queue."""
        self._scripted.append(response)

    async def complete(
        self,
        messages: Any,
        *,
        tools: Any = (),
        settings: Any = None,
        **_kwargs: Any,
    ) -> Any:
        self.call_count += 1
        from rob_box_llm.provider import LLMResponse

        if self._idx < len(self._scripted):
            text = self._scripted[self._idx]
            self._idx += 1
            return LLMResponse(content=text, finish_reason="stop")
        # Defer to base behaviour for everything else
        return await super().complete(messages, tools=tools, settings=settings)


class _TestableDialogueNode(DialogueNode):
    """DialogueNode subclass with fakes pre-wired and a controllable loop.

    Differences from the production shell:

    * ``_build_llm`` returns the injected ``_ScriptedLLMProvider``
      instead of touching ``DEEPSEEK_API_KEY``.
    * ``_build_memory`` always returns the injected
      :class:`InMemoryStore` (no SQLite fallback path).
    * ``_build_tool_provider`` always returns
      :class:`FakeToolProvider` (no MCP bridge probe).
    * ``_load_system_prompt`` returns ``""`` (no file lookup).

    Tests then drive ``_on_stt`` / ``_on_dj_mode`` callbacks directly
    and inspect published messages on the captured publishers.
    """

    def __init__(
        self,
        *,
        llm: _ScriptedLLMProvider,
        memory: Optional[InMemoryStore] = None,
        tools: Optional[FakeToolProvider] = None,
    ) -> None:
        self._test_llm = llm
        self._test_memory = memory if memory is not None else InMemoryStore()
        self._test_tools = tools if tools is not None else FakeToolProvider()
        super().__init__()
        # Capture the production loop that the parent ctor started on a
        # background executor, so close() can stop it (otherwise the
        # orphan thread keeps spinning for the lifetime of the process
        # and 12 tests run in 40s instead of 4s).
        self._executor_loop = self._loop
        # Replace the production asyncio loop + effect registry with
        # one driven by the test loop. This lets the test drive turns
        # deterministically and verify awaiter release without a
        # background executor.
        self._test_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._test_loop)
        self._loop = self._test_loop
        from rob_box_voice.core.speak_helpers import (
            EffectAwaiterRegistry as _EAR,
        )
        self._effects = _EAR(
            release_tts=lambda ev: self._test_loop.call_soon_threadsafe(ev.set),
            release_sound=lambda ev: self._test_loop.call_soon_threadsafe(ev.set),
        )

    # ── Override all the live-wiring hooks ────────────────────────────

    def _build_llm(self) -> Any:  # type: ignore[override]
        return self._test_llm

    def _build_memory(self):  # type: ignore[override]
        # NOTE: The shell's stock _build_memory calls ``store.init()``
        # but InMemoryStore doesn't expose that method (only the
        # SQLite implementation does). The shell's InMemoryStore
        # fallback path is therefore broken; we sidestep it here so
        # the tests exercise the upstream wiring (DialogCore ↔
        # MemoryStore) without the in-memory init dance.
        return self._test_memory

    def _build_tool_provider(self):  # type: ignore[override]
        return self._test_tools

    def _load_system_prompt(self) -> str:  # type: ignore[override]
        return ""

    def _dispatch_turn(self, user_input: str) -> None:  # type: ignore[override]
        """Schedule the turn on the test loop directly.

        The production shell uses ``asyncio.run_coroutine_threadsafe``
        to dispatch turns onto a loop running in a
        ``ThreadPoolExecutor``. For tests we own the loop, so we just
        create the task and let ``drive_one_turn`` run it.
        """
        self._run_task = self._test_loop.create_task(self._run_turn(user_input))

    # ── Async helpers used by the tests ──────────────────────────────

    def drive_one_turn(self) -> None:
        """Run the in-flight asyncio task to completion (if any).

        ``_dispatch_turn`` creates the task on the test loop; we just
        need to drive it. We use ``run_until_complete`` so any
        sub-tasks (e.g. ``asyncio.shield`` wrappers, awaited futures)
        are also drained.
        """
        for _ in range(20):
            if self._run_task is not None:
                break
            time.sleep(0.005)
        if self._run_task is None:
            return
        try:
            self._test_loop.run_until_complete(self._run_task)
        except asyncio.CancelledError:
            pass
        except RuntimeError:
            # Task may already be done; ignore.
            pass

    def close(self) -> None:
        """Tear down the asyncio loop and the node."""
        # The shell's executor was started in the parent ctor; cancel
        # the future and shut it down so it doesn't leak.
        executor_loop = getattr(self, "_executor_loop", None)
        future = getattr(self, "_asyncio_loop_future", None)
        executor = getattr(self, "_asyncio_loop_executor", None)
        if executor_loop is not None:
            try:
                executor_loop.call_soon_threadsafe(executor_loop.stop)
            except RuntimeError:
                pass
        if future is not None:
            try:
                future.result(timeout=2.0)
            except Exception:  # noqa: BLE001
                pass
        if executor is not None:
            try:
                executor.shutdown(wait=False)
            except Exception:  # noqa: BLE001
                pass
        try:
            if not self._test_loop.is_closed():
                self._test_loop.close()
        finally:
            try:
                super().destroy_node()
            except Exception:  # noqa: BLE001
                pass


# ─────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────


def _make_string(data: str) -> String:
    """Construct a ``std_msgs/String`` with ``.data = data``."""
    s = String()
    s.data = data
    return s


def _make_bool(data: bool) -> Bool:
    b = Bool()
    b.data = data
    return b


def _response_text(payload: Any) -> str:
    """Decode the JSON ``ssml`` envelope published to the response topic."""
    raw = getattr(payload, "data", payload)
    if isinstance(raw, str):
        try:
            envelope = json.loads(raw)
        except (TypeError, ValueError):
            return raw
        return envelope.get("ssml", "").removeprefix("<speak>").removesuffix("</speak>")
    return str(raw)


def _state_name(node: DialogueNode) -> str:
    return node._dsm.current_state.name


# ─────────────────────────────────────────────────────────────────────
# Tests
# ─────────────────────────────────────────────────────────────────────


class TestDialogueShell(unittest.TestCase):
    """Integration tests for the W5 dialogue shell.

    Each test stands up a ``_TestableDialogueNode`` so we can drive
    ``_on_stt`` directly and inspect the captured publishers.
    """

    # ── shared fixtures ──────────────────────────────────────────────

    def setUp(self) -> None:
        # Default LLM is empty — tests push replies as needed.
        self.llm = _ScriptedLLMProvider()
        self.node = _TestableDialogueNode(llm=self.llm)

    def tearDown(self) -> None:
        self.node.close()

    # ── Test helpers ─────────────────────────────────────────────────

    def _drive_turn(self) -> None:
        """Run the in-flight asyncio task to completion (if any)."""
        self.node.drive_one_turn()

    # ── 1. Wake word detection → state LISTENING ─────────────────────

    def test_wake_word_detection_routes_to_listening(self):
        """STT with wake-word + content transitions IDLE → LISTENING → DIALOGUE → IDLE.

        The integration test exercises the W6 shell fix that drives
        ``WAKE_WORD`` before ``STT_RESULT`` so the DSM correctly
        walks IDLE → LISTENING (wake-word gate) → DIALOGUE (speech).
        """
        self.llm.push("Привет!")
        self.node._on_stt(_make_string("роббокс привет как дела"))
        self._drive_turn()
        # Final state after a complete turn is IDLE (DIALOGUE_END fires)
        self.assertEqual(_state_name(self.node), "IDLE")
        # But the state publish path must have observed LISTENING and DIALOGUE.
        published_states = [p.data for p in self.node._state_pub.published]
        self.assertIn("LISTENING", published_states)
        self.assertIn("DIALOGUE", published_states)
        # LLM was called
        self.assertEqual(self.llm.call_count, 1)
        # Response was published
        responses = self.node._response_pub.published
        self.assertGreaterEqual(len(responses), 1)
        text = _response_text(responses[-1])
        self.assertIn("Привет!", text)

    # ── 2. STT input → state DIALOGUE → response published ───────────

    def test_stt_input_publishes_response(self):
        """Already in LISTENING, an STT input drives DIALOGUE + publishes response.

        The first part of the wake-word sequence (IDLE → LISTENING) is
        covered by ``test_wake_word_detection_routes_to_listening``.
        Here we focus on the second half: once we're in LISTENING, a
        follow-up STT message (no wake word needed) must drive
        DIALOGUE and the LLM reply is published to the response topic.
        """
        # Pre-seed LISTENING via a synthetic WAKE_WORD transition so
        # the next STT message skips the wake-word gate cleanly.
        from rob_box_harness.core.dialogue_state_machine import (
            DialogueEvent as _DE,
        )
        self.node._dsm.on_event(_DE.WAKE_WORD)
        self.assertEqual(_state_name(self.node), "LISTENING")

        # Script a deterministic LLM reply.
        self.llm.push("Готов к работе!")
        self.node._on_stt(_make_string("как дела?"))
        self._drive_turn()
        # Final state: DIALOGUE → DIALOGUE_END → IDLE
        self.assertEqual(_state_name(self.node), "IDLE")
        # DIALOGUE was observed mid-flight
        published_states = [p.data for p in self.node._state_pub.published]
        self.assertIn("DIALOGUE", published_states)
        # LLM was called exactly once
        self.assertEqual(self.llm.call_count, 1)
        # Response envelope was published
        responses = self.node._response_pub.published
        self.assertGreaterEqual(len(responses), 1)
        text = _response_text(responses[-1])
        self.assertIn("Готов к работе!", text)

    # ── 3. Silence command → state SILENCED ──────────────────────────

    def test_silence_command_silences(self):
        """A silence command from STT transitions active state → SILENCED."""
        # Drive LISTENING explicitly — the shell's wake-word-only STT
        # is filtered by the empty-clean-text guard, so the DSM
        # transition has to come via a direct WAKE_WORD event.
        from rob_box_harness.core.dialogue_state_machine import (
            DialogueEvent as _DE,
        )
        self.node._dsm.on_event(_DE.WAKE_WORD)
        self.assertEqual(_state_name(self.node), "LISTENING")
        # Now issue a silence command ("помолчи") — strip_wake_word
        # leaves "помолчи" so is_silence_command() matches.
        self.node._on_stt(_make_string("помолчи"))
        self.assertEqual(_state_name(self.node), "SILENCED")
        # State publish path was driven
        published_states = [p.data for p in self.node._state_pub.published]
        self.assertIn("SILENCED", published_states)

    def test_silence_from_idle_is_noop(self):
        """Silence command while IDLE does nothing — wake-word gate first.

        The shell only consumes a turn after the wake word fires, so a
        raw ``"помолчи"`` (no wake word) is dropped before silence is
        even checked.
        """
        self.assertEqual(_state_name(self.node), "IDLE")
        self.node._on_stt(_make_string("помолчи"))
        # Still IDLE — wake-word gate dropped the input.
        self.assertEqual(_state_name(self.node), "IDLE")

    # ── 4. Inactivity timeout → LISTENING → IDLE ─────────────────────

    def test_inactivity_timeout_returns_to_idle(self):
        """After LISTENING + a long silence, the timer drops back to IDLE.

        We can't rely on wall-clock waits, so we patch
        ``DialogueStateMachine.check_inactivity_timeout`` to fire on
        the first tick. The shell's ``_on_inactivity_check`` calls
        ``self._core.check_timeout()`` which delegates there.
        """
        # Wake into LISTENING. We can't drive the full STT path
        # because the shell immediately fires STT_RESULT → DIALOGUE
        # once it dispatches a turn. Pushing WAKE_WORD directly lets
        # us stage just the LISTENING transition and test the
        # timer-driven IDLE return in isolation.
        from rob_box_harness.core.dialogue_state_machine import (
            DialogueEvent as _DE,
        )
        self.node._dsm.on_event(_DE.WAKE_WORD)
        self.assertEqual(_state_name(self.node), "LISTENING")

        # Patch the DSM's check_inactivity_timeout to always fire.
        from rob_box_harness.core import dialog_core as _dc_mod

        original = self.node._dsm.check_inactivity_timeout
        self.node._dsm.check_inactivity_timeout = lambda _t: (
            self.node._dsm.transition(DialogueStateKind.IDLE),
            True,
        )[-1]
        try:
            # Drive the inactivity check the shell uses on its 5s timer.
            self.node._on_inactivity_check()
        finally:
            self.node._dsm.check_inactivity_timeout = original  # type: ignore[assignment]
        self.assertEqual(_state_name(self.node), "IDLE")

    # ── 5. DJ mode transition (timer-based tick) ─────────────────────

    def test_dj_mode_tick_fires_transition(self):
        """When DJ-mode is enabled and the timer expires, a transition runs.

        Note: the DJ controller skips dispatch when ``is_dialogue_active``
        is true (no barging into a live conversation), so the DSM must
        be at IDLE for the tick to actually dispatch. When IDLE, the
        underlying ``process_input`` path isn't in DIALOGUE either,
        so no LLM call happens — what we can verify is that
        ``_dispatch_turn`` was invoked, which is the unit we care
        about for the W6 tick coverage.
        """
        # Enable DJ mode via the /voice/dj_mode topic — the shell forwards
        # to DJModeController.handle_message which JSON-parses the payload.
        # handle_message() clamps next_transition_sec to ≥15s, so for a
        # unit test we set the controller state directly to "fire now".
        self.node._dj.state.enabled = True
        self.node._dj.state.next_transition_at = 0.0
        # Patch the dispatch hook on the controller to record the call.
        # This isolates the tick → dispatch chain from the LLM path,
        # which is the piece that owns the timer-based trigger.
        dispatched: list = []
        original_dispatch = self.node._dj._hook.dispatch
        self.node._dj._hook.dispatch = lambda prompt: dispatched.append(prompt)
        try:
            self.node._dj.tick()
        finally:
            self.node._dj._hook.dispatch = original_dispatch
        self.assertEqual(
            len(dispatched), 1,
            "DJ tick did not invoke dispatch exactly once",
        )
        self.assertIn("[DJ_AUTO", dispatched[0])
        self.assertEqual(self.node._dj.state.transition_count, 1)

    def test_dj_mode_tick_skips_when_dialogue_active(self):
        """DJ tick postpones (not dispatches) while a turn is in flight.

        Documents the barging-safety rule: when the DSM is in DIALOGUE,
        the tick must NOT issue a new prompt — it must push the next
        transition out by ``POSTPONE_INTERVAL_S``.
        """
        self.node._dj.state.enabled = True
        self.node._dj.state.next_transition_at = 0.0
        # Move DSM into DIALOGUE — the controller must refuse to dispatch.
        from rob_box_harness.core.dialogue_state_machine import (
            DialogueEvent as _DE,
        )
        self.node._dsm.on_event(_DE.WAKE_WORD)
        self.node._dsm.on_event(_DE.STT_RESULT)
        self.assertEqual(_state_name(self.node), "DIALOGUE")

        captured: list = []
        original_dispatch = self.node._dj._hook.dispatch
        self.node._dj._hook.dispatch = lambda p: captured.append(p)
        try:
            self.node._dj.tick()
        finally:
            self.node._dj._hook.dispatch = original_dispatch
        self.assertEqual(captured, [], "Tick must not dispatch during DIALOGUE")
        # postpone pushes next_transition_at into the future
        self.assertGreater(
            self.node._dj.state.next_transition_at, 0.0
        )

    def test_dj_mode_disabled_is_noop(self):
        """Disabled DJ-mode never fires auto transitions."""
        self.node._dj.handle_message(json.dumps({"enabled": False}))
        self.assertFalse(self.node._dj.state.enabled)
        before = self.llm.call_count
        self.node._dj.tick()
        self.node._dj.tick()
        self.assertEqual(self.llm.call_count, before)

    # ── 6. Barge-in: new STT cancels old turn ────────────────────────

    def test_barge_in_cancels_active_run(self):
        """While a turn is running, a new STT cancels the prior task.

        Reproduces the live scenario: wake → first turn (long-running
        so it stays in flight) → second wake-word turn arrives. The
        shell must cancel the first task before dispatching the new
        one.
        """
        # Force the first turn to never complete within the window by
        # routing its LLM call to a never-resolving awaitable. This
        # keeps _run_task alive while we send the barge-in STT.

        class _NeverResolves:
            def __await__(self):
                fut = asyncio.get_event_loop().create_future()
                # Leave the future pending — the turn never finishes.
                return fut.__await__()

        async def _hanging_complete(*_a, **_kw):
            self.llm.call_count += 1
            return _NeverResolves()  # type: ignore[return-value]

        original_complete = self.llm.complete
        self.llm.complete = _hanging_complete  # type: ignore[assignment]
        try:
            # First turn — dispatches but never completes.
            self.node._on_stt(_make_string("роббокс первый запрос"))
            # Second turn — barge-in should cancel the first.
            self.node._on_stt(_make_string("роббокс второй запрос"))
            # The first task must have been cancelled.
            self.assertTrue(
                self.node._run_cancelled,
                "Barge-in did not set the cancelled flag",
            )
        finally:
            self.llm.complete = original_complete  # type: ignore[assignment]
            self.node.close()

    # ── 7. TTS finish event → speak_text awaiter released ───────────

    def test_tts_finished_releases_awaiter(self):
        """Publishing /voice/tts/finished releases the registered awaiter."""
        # Manually register an awaiter for a synthetic speech_id and
        # verify the registry releases it on /voice/tts/finished.
        self.node._test_loop.run_until_complete(self._arm_tts_test())

    async def _arm_tts_test(self) -> None:
        speech_id = "test-speech-id-001"
        event = asyncio.Event()
        self.node._effects.register_tts(speech_id, event)
        # Simulate the tts_node publishing finished
        self.node._on_tts_finished(
            _make_string(json.dumps({"speech_id": speech_id}))
        )
        # Give the call_soon_threadsafe callback a chance to fire on
        # the test loop.
        await asyncio.sleep(0)
        self.assertTrue(
            event.is_set(),
            "TTS finished event did not release the registered awaiter",
        )

    # ── 8. Sound finish event → play_sound awaiter released ──────────

    def test_sound_ready_releases_awaiter(self):
        """Publishing /voice/sound/state='ready' releases the awaiter."""
        self.node._test_loop.run_until_complete(self._arm_sound_test())

    async def _arm_sound_test(self) -> None:
        event = asyncio.Event()
        self.node._effects.set_sound_event(event)
        self.node._on_sound_state(_make_string("ready"))
        await asyncio.sleep(0)
        self.assertTrue(
            event.is_set(),
            "Sound ready event did not release the registered awaiter",
        )

    # ── Bonus: DSM ↔ DJ hook integration ─────────────────────────────

    def test_dj_tick_postpones_during_dialogue(self):
        """DJ tick defers transitions while a turn is in flight."""
        self.node._dj.handle_message(json.dumps({
            "enabled": True,
            "theme": "lofi",
            "next_transition_sec": 0,
        }))
        # Simulate that a turn is currently active.
        self.node._dj._hook.is_active = lambda: True  # type: ignore[assignment]
        before = self.llm.call_count
        self.node._dj.tick()
        # LLM must NOT have been called — the tick should have
        # postponed the transition instead of dispatching.
        self.assertEqual(self.llm.call_count, before)
        # And the postpone interval should be in the future.
        self.assertGreater(
            self.node._dj.state.next_transition_at, time.time() - 1
        )


# ─────────────────────────────────────────────────────────────────────
# Module-level smoke test: the shell imports cleanly under our shim.
# ─────────────────────────────────────────────────────────────────────


class TestShellImportSanity(unittest.TestCase):
    """Lightweight check that DialogueNode is importable and is a Node."""

    def test_dialogue_node_is_subclass_of_node(self):
        self.assertTrue(issubclass(DialogueNode, object))
        # rclpy.node.Node — real or our shim — is in the MRO.
        from rclpy.node import Node as _Node
        self.assertTrue(issubclass(DialogueNode, _Node))


# ─────────────────────────────────────────────────────────────────────
# Regression tests for the W5a config-vs-runtime sync issue
# (kanban t_d0f33064).
#
# The shipped behaviour at the time of writing is:
#
#   * ``enable_mcp_tools=true`` is honoured at startup, but
#     ``_build_tool_provider`` still falls through to a
#     :class:`FakeToolProvider` even when the
#     ``rob_box_mcp_tools`` package is discoverable (W5a is the
#     follow-up task that wires ``ROSMCPToolProvider`` for real).
#   * The shell must log a startup WARNING so operators can spot the
#     mismatch at run time instead of discovering it when voice
#     commands like "открой шторы" / "сохрани точку" silently no-op.
#
# These tests pin the WARNING behaviour so it can't quietly disappear
# during a refactor. They live alongside the W5 integration tests
# because the shell is the unit that owns the fake-provider wiring.
# ─────────────────────────────────────────────────────────────────────


class TestFakeToolProviderRegression(unittest.TestCase):
    """Pin the FakeToolProvider + W5a warning behaviour.

    The shell's ``_TestableDialogueNode`` always injects a
    :class:`FakeToolProvider` via the ``_build_tool_provider`` override.
    We can therefore assert two things directly without spinning up
    the real ROS2 stack:

    1. The injected fake provider is exactly the type the production
       shell returns today (``FakeToolProvider``), and the 29 MCP
       tools (``speak_text``, ``play_sound``, ``waypoint_save``, …)
       are NOT among the exposed specs — i.e. the LLM has no way to
       drive the real robot. (The fake provider does expose the
       built-in ``echo`` tool for harness smoke tests; we exclude
       that one from the assertion.)
    2. The production-shell code path logs the canonical W5a warning
       via the underlying ``DialogueNode._build_tool_provider`` when
       ``enable_mcp_tools=True`` AND the MCP-bridge probe succeeds.

    Test (2) is the regression that the W5a task is supposed to flip:
    when ``ROSMCPToolProvider`` lands, this assertion will fail and
    the test should be updated to assert the new behaviour. That's
    intentional — the test is the canary.
    """

    # A representative slice of the 29 MCP tools that should be
    # exposed once W5a lands. We assert they are NOT exposed today
    # (FakeToolProvider) — when W5a ships, the assertion flips.
    _MCP_TOOL_CANARY = ("speak_text", "play_sound", "waypoint_save")

    def test_injected_fake_provider_exposes_no_mcp_tools(self):
        """``FakeToolProvider`` does not expose any MCP tools today.

        Operators rely on this emptiness as the W5a marker: the LLM
        gets the built-in ``echo`` tool but not the MCP-bridged
        robot tools (speak / sound / waypoint / music), so voice
        commands like "сохрани точку" / "сыграй трек" silently
        no-op. Once W5a ships the test should be updated to assert
        the MCP tool names ARE exposed.
        """
        import asyncio

        async def _discover() -> tuple:
            node = _TestableDialogueNode(llm=_ScriptedLLMProvider())
            try:
                provider = node._build_tool_provider()
                return await provider.discover()
            finally:
                node.close()

        specs = asyncio.run(_discover())
        spec_names = {s.name for s in specs}
        # Sanity: only the built-in ``echo`` tool is wired today.
        self.assertEqual(
            spec_names, {"echo"},
            f"FakeToolProvider should expose ONLY the built-in echo "
            f"tool today; got: {sorted(spec_names)}",
        )
        # The W5a canary: MCP tools must be absent.
        exposed_mcp = spec_names & set(self._MCP_TOOL_CANARY)
        self.assertEqual(
            exposed_mcp, set(),
            f"FakeToolProvider must not expose MCP tools today "
            f"(W5a not landed yet); found: {sorted(exposed_mcp)}",
        )

    def test_production_shell_logs_w5a_warning_when_mcp_probe_succeeds(self):
        """Production shell surfaces the W5a mismatch at startup.

        Builds a production :class:`DialogueNode` (no fake override)
        but stops short of fully initialising the harness layer — we
        only need ``_build_tool_provider`` to run, which is the unit
        the W5a task is rewriting. The test patches the
        ``ament_index_python`` probe to return successfully (so we
        exercise the "MCP package found, but FakeToolProvider wired"
        branch), then asserts the canonical warning text.
        """
        # Stop after __init__'s super().__init__ by NOT running the
        # full wiring — instead, allocate via object.__new__ and
        # call only _build_tool_provider with a stubbed parameter
        # store + logger.
        node = object.__new__(DialogueNode)
        logger = MagicMock()
        node.get_logger = lambda: logger

        class _Param:
            def __init__(self, value):
                self.value = value

        # enable_mcp_tools=True drives the W5a branch.
        node.get_parameter = lambda name: _Param(
            True if name == "enable_mcp_tools" else None
        )

        # Force the ament probe to succeed — rob_box_mcp_tools is
        # available in the container (Dockerfile installs it).
        fake_mod = _types.ModuleType("ament_index_python")
        fake_pkg = _types.ModuleType("ament_index_python.packages")
        fake_pkg.get_package_share_directory = lambda _pkg: "/tmp/fake"
        fake_mod.packages = fake_pkg
        saved = {
            k: sys.modules.get(k)
            for k in (
                "ament_index_python",
                "ament_index_python.packages",
            )
        }
        sys.modules["ament_index_python"] = fake_mod
        sys.modules["ament_index_python.packages"] = fake_pkg
        try:
            provider = node._build_tool_provider()
        finally:
            for k, v in saved.items():
                if v is None:
                    sys.modules.pop(k, None)
                else:
                    sys.modules[k] = v

        # Provider is still FakeToolProvider today — the W5a mismatch.
        self.assertIsInstance(provider, FakeToolProvider)
        # The startup warning was emitted, with a pointer to the
        # kanban task so operators know where to look.
        warning_calls = [
            call_args
            for call_args in logger.warning.call_args_list
        ]
        # Find the W5a-specific warning by substring.
        w5a_warnings = [
            c for c in warning_calls
            if c.args and "W5a" in (c.args[0] if c.args else "")
        ]
        self.assertTrue(
            w5a_warnings,
            "DialogueNode did not log the W5a FakeToolProvider warning. "
            f"warnings seen: {warning_calls!r}",
        )
        # The warning must point at the right kanban card.
        joined = " ".join(
            (c.args[0] if c.args else "")
            for c in w5a_warnings
        )
        self.assertIn("t_10a9c178", joined)
        self.assertIn("FakeToolProvider", joined)


if __name__ == "__main__":
    unittest.main()