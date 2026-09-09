"""Unit tests for :mod:`rob_box_harness.core.acceptance`.

Covers the §8 / §11.2 acceptance criteria that depend on the
*runtime* side of the layer — state-machine transitions, timeout
handling, the feedback block, and the «stop_navigation executes
immediately, even mid-AWAITING» rule.

The asyncio-free ``threading.Timer`` is used for timeout tests so
the suite stays fast and deterministic without needing
``pytest-asyncio``.
"""

from __future__ import annotations

import threading
import time
from typing import Any

import pytest

from rob_box_harness.config import ConfirmationPolicyConfig
from rob_box_harness.core.acceptance import (
    AcceptanceGate,
    PendingSegment,
    SegmentKind,
    SegmentStatus,
    _IllegalTransition,
    render_awaiting_block,
    route_kind,
)
from rob_box_harness.core.confirmation_policy import (
    ConfirmationKind,
    ToolConfirmationPolicy,
    load_default_policy,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def _make_gate(
    *,
    policy: ToolConfirmationPolicy | None = None,
    timeout_ms: int = 20_000,
    enabled: bool = True,
    use_timer: bool = True,
) -> tuple[AcceptanceGate, list[tuple[Any, float, Any]]]:
    """Build a gate with controllable timeouts.

    Returns ``(gate, scheduled)`` where *scheduled* collects every
    ``(handle, delay_s, callback)`` tuple handed to the scheduler.
    Tests can call the callbacks directly or wait for the
    ``threading.Timer`` to fire.

    When ``use_timer=True`` (default) we wire up real
    ``threading.Timer`` so the gate's timeout path is exercised
    end-to-end. Tests that want full control can pass
    ``use_timer=False`` to use a fake scheduler instead.
    """
    cfg = ConfirmationPolicyConfig(
        enabled=enabled,
        confirmation_timeout_ms=timeout_ms,
        timeout_announcement="Отменяю, жду указаний",
        safe_boundary_policy="soft",
    )
    scheduled: list[tuple[Any, float, Any]] = []
    timers: list[threading.Timer] = []

    def schedule(delay_s: float, callback: Any) -> Any:
        if use_timer:
            timer = threading.Timer(delay_s, callback)
            timer.daemon = True
            timer.start()
            timers.append(timer)
            handle = id(timer)
        else:
            handle = object()
        scheduled.append((handle, delay_s, callback))
        return handle

    def cancel(handle: Any) -> None:
        if use_timer and isinstance(handle, int):
            for t in timers:
                if id(t) == handle:
                    t.cancel()
                    return

    policy = policy or load_default_policy()
    gate = AcceptanceGate(
        policy=policy,
        config=cfg,
        timeout_scheduler=schedule,
        timeout_canceller=cancel,
    )
    # Stash timers on the gate so tests can wait on them.
    gate._timers = timers  # type: ignore[attr-defined]
    return gate, scheduled


@pytest.fixture
def synthetic_policy() -> ToolConfirmationPolicy:
    """A tiny policy that exercises every class without YAML loading.

    Tool names mirror the production catalog so :func:`route_kind`
    routes them to the right channel — that way the tests double
    as proof that the routing table stays consistent with the
    policy catalog.
    """
    return ToolConfirmationPolicy.from_mapping(
        {
            "tools": {
                "navigate_to_waypoint": {"class": "require", "plan_template": "Go to {tool_name}"},
                "stop_navigation": {"class": "pass_through"},
                "speak_text": {"class": "pass_through"},
                "delete_waypoint": {"class": "require", "plan_template": "Delete {tool_name}"},
                "set_speed": {"class": "notify"},
            },
        }
    )


# ---------------------------------------------------------------------------
# Classification → status mapping (submits)
# ---------------------------------------------------------------------------


def test_require_submit_creates_awaiting_segment(synthetic_policy: ToolConfirmationPolicy) -> None:
    """§11.2: navigate_to_waypoint creates an AWAITING segment, not ACTIVE."""
    gate, _ = _make_gate(policy=synthetic_policy)
    seg, decision = gate.submit(tool="navigate_to_waypoint", args={"name": "кухня"})
    assert decision.kind is ConfirmationKind.REQUIRE
    assert seg.status is SegmentStatus.AWAITING_CONFIRMATION
    assert seg.decision.plan_text == "Go to navigate_to_waypoint"
    assert seg.kind is SegmentKind.NAV  # routed to the nav channel
    assert gate.awaiting() == [seg]


def test_stop_navigation_submit_creates_active_segment_immediately(
    synthetic_policy: ToolConfirmationPolicy,
) -> None:
    """§11.2: stop_navigation goes to ACTIVE immediately, never AWAITING."""
    gate, _ = _make_gate(policy=synthetic_policy)
    seg, decision = gate.submit(tool="stop_navigation", args={})
    assert decision.kind is ConfirmationKind.PASS_THROUGH
    assert seg.status is SegmentStatus.ACTIVE
    assert seg.confirmed_at is not None
    assert gate.awaiting() == []


def test_speak_text_submit_creates_active_segment_immediately(
    synthetic_policy: ToolConfirmationPolicy,
) -> None:
    """§11.2: speak_text is PENDING → ACTIVE without confirmation."""
    gate, _ = _make_gate(policy=synthetic_policy)
    seg, decision = gate.submit(tool="speak_text", args={"text": "hi"})
    assert decision.kind is ConfirmationKind.PASS_THROUGH
    assert seg.status is SegmentStatus.ACTIVE


def test_notify_submit_creates_active_segment_without_awaiting(
    synthetic_policy: ToolConfirmationPolicy,
) -> None:
    """§8.2: notify tools run, just with an announcement."""
    gate, _ = _make_gate(policy=synthetic_policy)
    seg, decision = gate.submit(tool="set_speed", args={})
    assert decision.kind is ConfirmationKind.NOTIFY
    assert seg.status is SegmentStatus.ACTIVE
    assert seg.confirmed_at is not None


def test_unknown_tool_uses_default_pass_through() -> None:
    gate, _ = _make_gate(policy=ToolConfirmationPolicy.from_mapping({}))
    seg, decision = gate.submit(tool="definitely_not_in_catalog", args={})
    assert decision.kind is ConfirmationKind.PASS_THROUGH
    assert seg.status is SegmentStatus.ACTIVE


# ---------------------------------------------------------------------------
# Confirm / reject / cancel
# ---------------------------------------------------------------------------


def test_confirm_moves_awaiting_to_active(synthetic_policy: ToolConfirmationPolicy) -> None:
    """§11.2: «да» → AWAITING → ACTIVE, timer cancelled."""
    gate, scheduled = _make_gate(policy=synthetic_policy, use_timer=False)
    seg, _ = gate.submit(tool="navigate_to_waypoint", args={})
    assert seg.status is SegmentStatus.AWAITING_CONFIRMATION
    assert len(scheduled) == 1
    handle = seg.timeout_handle
    assert handle is not None

    confirmed = gate.confirm(seg.segment_id)
    assert confirmed.status is SegmentStatus.ACTIVE
    assert confirmed.confirmed_at is not None
    # The gate dropped the timer handle on confirm.
    assert seg.timeout_handle is None
    # Segment no longer appears in awaiting().
    assert gate.awaiting() == []


def test_reject_moves_awaiting_to_rejected(synthetic_policy: ToolConfirmationPolicy) -> None:
    """§11.2: «нет» → AWAITING → REJECTED, feedback event fires."""
    gate, scheduled = _make_gate(policy=synthetic_policy, use_timer=False)

    # Subscribe BEFORE submit so we observe both transitions.
    events: list[tuple[str, SegmentStatus, str]] = []
    gate.subscribe(lambda sid, s, st: events.append((sid, st, s.decision.reason)))

    seg, _ = gate.submit(tool="navigate_to_waypoint", args={})

    rejected = gate.reject(seg.segment_id, reason="user_said_no")
    assert rejected.status is SegmentStatus.REJECTED
    assert rejected.args["_rejection_reason"] == "user_said_no"
    assert events == [
        (seg.segment_id, SegmentStatus.AWAITING_CONFIRMATION, rejected.decision.reason),
        (seg.segment_id, SegmentStatus.REJECTED, rejected.decision.reason),
    ]


def test_confirm_unknown_segment_raises_keyerror(synthetic_policy: ToolConfirmationPolicy) -> None:
    gate, _ = _make_gate(policy=synthetic_policy)
    with pytest.raises(KeyError):
        gate.confirm("nonexistent")


def test_reject_unknown_segment_raises_keyerror(synthetic_policy: ToolConfirmationPolicy) -> None:
    gate, _ = _make_gate(policy=synthetic_policy)
    with pytest.raises(KeyError):
        gate.reject("nonexistent")


# ---------------------------------------------------------------------------
# Timeout — AWAITING → REJECTED after confirmation_timeout_ms
# ---------------------------------------------------------------------------


def test_timeout_transitions_segment_to_rejected(synthetic_policy: ToolConfirmationPolicy) -> None:
    """§11.2: timeout (20s default) → REJECTED + auto-announcement queued."""
    gate, _ = _make_gate(policy=synthetic_policy, timeout_ms=50, use_timer=True)
    seg, _ = gate.submit(tool="navigate_to_waypoint", args={})
    assert seg.status is SegmentStatus.AWAITING_CONFIRMATION

    # Wait for the real threading.Timer to fire.
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        if seg.status is SegmentStatus.REJECTED:
            break
        time.sleep(0.01)
    assert seg.status is SegmentStatus.REJECTED, "timeout should have fired"
    assert seg.args["_rejection_reason"] == "timeout"


def test_confirm_before_timeout_cancels_timer(synthetic_policy: ToolConfirmationPolicy) -> None:
    """§11.2: «да» before timeout cancels the timer; no late REJECTED."""
    gate, _ = _make_gate(policy=synthetic_policy, timeout_ms=200)
    seg, _ = gate.submit(tool="navigate_to_waypoint", args={})
    gate.confirm(seg.segment_id)
    # Wait long enough that the original timer would have fired.
    time.sleep(0.4)
    assert seg.status is SegmentStatus.ACTIVE, "no late transition after cancel"


# ---------------------------------------------------------------------------
# stop_navigation mid-AWAITING — gold rule enforcement
# ---------------------------------------------------------------------------


def test_stop_navigation_during_awaiting_executes_immediately(
    synthetic_policy: ToolConfirmationPolicy,
) -> None:
    """§11.2 e2e сценарий 4: «Едь на кухню» (awaiting) + «стой!» → stop_navigation мгновенно.

    This is the race-protection gold rule: the emergency stop must
    not wait for the existing AWAITING segment to resolve. The
    stop_navigation segment goes straight to ACTIVE; the original
    AWAITING segment is **not** auto-cancelled (the user might
    still confirm it after the emergency is over). The reflex
    layer (Фаза 2.5) is what would call cancel_all() — here we
    just verify the gate's submit path is non-blocking.
    """
    gate, _ = _make_gate(policy=synthetic_policy)
    nav_seg, _ = gate.submit(tool="navigate_to_waypoint", args={})
    assert nav_seg.status is SegmentStatus.AWAITING_CONFIRMATION

    # The user shouts «стой!» — the dialogue_node calls submit on
    # stop_navigation. It must NOT be blocked by the awaiting segment.
    stop_seg, stop_dec = gate.submit(tool="stop_navigation", args={})
    assert stop_dec.kind is ConfirmationKind.PASS_THROUGH
    assert stop_seg.status is SegmentStatus.ACTIVE
    assert nav_seg.status is SegmentStatus.AWAITING_CONFIRMATION  # untouched


def test_cancel_all_clears_every_awaiting_segment(synthetic_policy: ToolConfirmationPolicy) -> None:
    """§8.10: reflex-«стой» during AWAITING cancels all waiting segments."""
    gate, _ = _make_gate(policy=synthetic_policy, use_timer=False)
    a, _ = gate.submit(tool="navigate_to_waypoint", args={})
    b, _ = gate.submit(tool="delete_waypoint", args={})
    assert gate.awaiting() == [a, b]

    cancelled = gate.cancel_all()
    assert {s.segment_id for s in cancelled} == {a.segment_id, b.segment_id}
    assert all(s.status is SegmentStatus.CANCELLED for s in cancelled)
    assert gate.awaiting() == []


# ---------------------------------------------------------------------------
# Race-protection — concurrent input during AWAITING
# ---------------------------------------------------------------------------


def test_concurrent_input_during_awaiting_is_race_safe(
    synthetic_policy: ToolConfirmationPolicy,
) -> None:
    """§11.2: «confirm while a second submit lands» is serialised, not corrupted.

    Submit + confirm from two threads must produce a coherent state
    (either AWAITING→ACTIVE, or AWAITING→REJECTED, but never a
    half-state). This is the unit-level proof of the «quick_decide
    interprets concurrent input as the confirm answer» invariant
    from §8.5 / §11.2 #9.
    """
    gate, _ = _make_gate(policy=synthetic_policy)
    seg, _ = gate.submit(tool="navigate_to_waypoint", args={})

    errors: list[BaseException] = []

    def _confirm() -> None:
        try:
            gate.confirm(seg.segment_id)
        except _IllegalTransition:
            pass  # Acceptable if a reject raced ahead.
        except BaseException as exc:  # pragma: no cover — defensive
            errors.append(exc)

    def _submit_more() -> None:
        try:
            gate.submit(tool="speak_text", args={"i": 1})
        except BaseException as exc:  # pragma: no cover — defensive
            errors.append(exc)

    threads = [threading.Thread(target=_confirm) for _ in range(5)] + [
        threading.Thread(target=_submit_more) for _ in range(5)
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=2.0)

    assert not errors, f"unexpected errors during concurrent input: {errors}"
    # The original segment must end up in a terminal state.
    assert seg.status in (SegmentStatus.ACTIVE, SegmentStatus.REJECTED)
    # No zombie awaiting segments left over.
    for s in gate.all_segments():
        if s.status is SegmentStatus.AWAITING_CONFIRMATION:
            # Pass-throughs auto-advance, so any remaining AWAITING
            # must be a require-classified one we explicitly opened.
            assert s.tool in {"navigate_to_waypoint", "delete_waypoint"}


# ---------------------------------------------------------------------------
# Subscriber hooks
# ---------------------------------------------------------------------------


def test_subscribe_receives_every_transition(synthetic_policy: ToolConfirmationPolicy) -> None:
    gate, _ = _make_gate(policy=synthetic_policy, use_timer=False)
    events: list[SegmentStatus] = []
    gate.subscribe(lambda _sid, _s, status: events.append(status))

    seg, _ = gate.submit(tool="navigate_to_waypoint", args={})
    gate.confirm(seg.segment_id)
    assert events == [SegmentStatus.AWAITING_CONFIRMATION, SegmentStatus.ACTIVE]


def test_subscribe_returns_unsubscribe(synthetic_policy: ToolConfirmationPolicy) -> None:
    gate, _ = _make_gate(policy=synthetic_policy)
    events: list[SegmentStatus] = []

    unsub = gate.subscribe(lambda _sid, _s, st: events.append(st))
    seg, _ = gate.submit(tool="navigate_to_waypoint", args={})
    assert events == [SegmentStatus.AWAITING_CONFIRMATION]

    unsub()
    gate.confirm(seg.segment_id)
    # No further events after unsubscribe.
    assert events == [SegmentStatus.AWAITING_CONFIRMATION]


# ---------------------------------------------------------------------------
# Feedback formatter — [AWAITING] block (§7)
# ---------------------------------------------------------------------------


def test_render_awaiting_block_with_one_segment(
    synthetic_policy: ToolConfirmationPolicy,
) -> None:
    """§11.2: [AWAITING] block contains the segment with elapsed_ms."""
    gate, _ = _make_gate(policy=synthetic_policy)
    seg, _ = gate.submit(tool="navigate_to_waypoint", args={})

    block = render_awaiting_block(gate.awaiting(), now=seg.created_at + 1.5)
    assert "[AWAITING]" in block
    assert "[/AWAITING]" in block
    assert "tool=navigate_to_waypoint" in block
    assert "elapsed_ms=1500" in block
    assert seg.segment_id in block
    assert "Go to navigate_to_waypoint" in block


def test_render_awaiting_block_empty_when_nothing_pending(
    synthetic_policy: ToolConfirmationPolicy,
) -> None:
    """§11.2: empty [AWAITING] block is still emitted so the LLM can detect it."""
    gate, _ = _make_gate(policy=synthetic_policy)
    block = render_awaiting_block(gate.awaiting())
    assert "[AWAITING]" in block
    assert "[/AWAITING]" in block


def test_render_awaiting_block_ignores_non_awaiting_segments(
    synthetic_policy: ToolConfirmationPolicy,
) -> None:
    gate, _ = _make_gate(policy=synthetic_policy)
    # Pass-through immediately ACTIVE — must NOT appear in the block.
    gate.submit(tool="speak_text", args={})
    # Require → AWAITING — must appear.
    seg, _ = gate.submit(tool="navigate_to_waypoint", args={})

    block = render_awaiting_block(gate.all_segments(), now=seg.created_at)
    assert "tool=speak_text" not in block
    assert "tool=navigate_to_waypoint" in block


def test_render_awaiting_block_includes_timeout_announcement(
    synthetic_policy: ToolConfirmationPolicy,
) -> None:
    gate, _ = _make_gate(policy=synthetic_policy)
    seg, _ = gate.submit(tool="navigate_to_waypoint", args={})
    block = render_awaiting_block(
        gate.awaiting(), timeout_announcement="Отменяю, жду указаний", now=seg.created_at
    )
    assert "timeout_announcement=Отменяю, жду указаний" in block


# ---------------------------------------------------------------------------
# Routing — route_kind() and SegmentKind
# ---------------------------------------------------------------------------


def test_route_kind_for_navigation_tools() -> None:
    assert route_kind("navigate_to_waypoint") is SegmentKind.NAV
    assert route_kind("navigate_to_coordinates") is SegmentKind.NAV
    assert route_kind("move_direction") is SegmentKind.NAV
    assert route_kind("stop_navigation") is SegmentKind.NAV


def test_route_kind_for_music_tools() -> None:
    assert route_kind("execute_music_code") is SegmentKind.MUSIC
    assert route_kind("stop_music") is SegmentKind.MUSIC


def test_route_kind_for_voice_tools() -> None:
    assert route_kind("speak_text") is SegmentKind.VOICE


def test_route_kind_for_mutation_tools() -> None:
    assert route_kind("delete_waypoint") is SegmentKind.MUTATE
    assert route_kind("save_waypoint") is SegmentKind.MUTATE
    assert route_kind("memory_save") is SegmentKind.MUTATE


def test_route_kind_unknown_returns_unknown() -> None:
    assert route_kind("nonsense_tool_xyz") is SegmentKind.UNKNOWN


# ---------------------------------------------------------------------------
# State machine — illegal transitions
# ---------------------------------------------------------------------------


def test_pending_segment_illegal_transition() -> None:
    seg = PendingSegment(
        segment_id="x",
        tool="safe",
        args={},
        decision=type("D", (), {"kind": ConfirmationKind.PASS_THROUGH, "reason": "", "plan_text": None})(),
        kind=SegmentKind.VOICE,
        status=SegmentStatus.ACTIVE,
    )
    # ACTIVE → AWAITING is illegal.
    with pytest.raises(_IllegalTransition):
        seg.mark_awaiting()


def test_awaiting_requires_require_decision() -> None:
    """mark_awaiting refuses to move a non-require segment."""
    from rob_box_harness.core.confirmation_policy import AcceptanceDecision

    seg = PendingSegment(
        segment_id="x",
        tool="safe",
        args={},
        decision=AcceptanceDecision(
            kind=ConfirmationKind.PASS_THROUGH,
            tool="safe",
            reason="safe",
            plan_text=None,
        ),
        kind=SegmentKind.VOICE,
        status=SegmentStatus.PENDING,
    )
    with pytest.raises(_IllegalTransition):
        seg.mark_awaiting()


# ---------------------------------------------------------------------------
# Configuration plumbing
# ---------------------------------------------------------------------------


def test_disabled_config_forces_pass_through() -> None:
    """When the layer is off, every tool (including require) passes through."""
    gate, _ = _make_gate(enabled=False)
    seg, dec = gate.submit(tool="nav", args={})
    assert dec.kind is ConfirmationKind.PASS_THROUGH
    assert seg.status is SegmentStatus.ACTIVE
    assert "disabled" in dec.reason.lower()


def test_invalid_config_rejected() -> None:
    """The gate enforces its config type at construction."""
    from rob_box_harness.core.confirmation_policy import ToolConfirmationPolicy

    with pytest.raises(TypeError):
        AcceptanceGate(
            policy=load_default_policy(),
            config="not a config",  # type: ignore[arg-type]
        )
    with pytest.raises(TypeError):
        AcceptanceGate(
            policy="not a policy",  # type: ignore[arg-type]
            config=ConfirmationPolicyConfig(),
        )