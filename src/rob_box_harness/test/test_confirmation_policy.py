"""Unit tests for ``rob_box_harness.core.confirmation_policy``.

Covers the §8 / §11.2 acceptance criteria that depend only on the
classifier and the bundled YAML catalog — no asyncio, no scheduler,
no integration with :class:`DialogCore`. Companion to
``test_acceptance_gate.py`` which exercises the runtime side.
"""

from __future__ import annotations

import pytest

from rob_box_harness.config import ConfirmationPolicyConfig
from rob_box_harness.core.confirmation_policy import (
    EMERGENCY_TOOLS,
    AcceptanceDecision,
    ConfirmationKind,
    ToolConfirmationPolicy,
    load_default_policy,
)
from rob_box_harness.errors import ConfigError


# ---------------------------------------------------------------------------
# YAML catalog (load_default_policy)
# ---------------------------------------------------------------------------


def test_load_default_policy_returns_valid_classifier() -> None:
    """Bundled YAML loads and produces a non-empty catalog."""
    policy = load_default_policy()
    assert isinstance(policy, ToolConfirmationPolicy)
    # We catalogued 30+ tools in step 2 — assert the floor.
    assert len(policy.known_tools) >= 30


def test_navigate_to_waypoint_requires_confirmation() -> None:
    """§11.2 acceptance: navigate_to_waypoint is REQUIRE."""
    policy = load_default_policy()
    decision = policy.classify("navigate_to_waypoint")
    assert decision.kind is ConfirmationKind.REQUIRE
    assert decision.plan_text  # non-empty — used for the awaiting prompt


def test_navigate_to_coordinates_requires_confirmation() -> None:
    policy = load_default_policy()
    assert policy.classify("navigate_to_coordinates").kind is ConfirmationKind.REQUIRE


def test_move_direction_requires_confirmation() -> None:
    """§8.2 explicitly lists move_direction as 🔴 require."""
    policy = load_default_policy()
    assert policy.classify("move_direction").kind is ConfirmationKind.REQUIRE


def test_start_mapping_requires_confirmation() -> None:
    policy = load_default_policy()
    assert policy.classify("start_mapping").kind is ConfirmationKind.REQUIRE


def test_delete_waypoint_requires_confirmation() -> None:
    policy = load_default_policy()
    assert policy.classify("delete_waypoint").kind is ConfirmationKind.REQUIRE


def test_clear_waypoints_requires_confirmation() -> None:
    policy = load_default_policy()
    assert policy.classify("clear_waypoints").kind is ConfirmationKind.REQUIRE


def test_save_waypoint_requires_confirmation() -> None:
    """§8.2: even save_waypoint triggers confirmation when it would overwrite."""
    policy = load_default_policy()
    assert policy.classify("save_waypoint").kind is ConfirmationKind.REQUIRE


def test_set_speed_is_notify() -> None:
    """§8.2: set_speed is 🟡 notify (reversible, but worth announcing)."""
    policy = load_default_policy()
    assert policy.classify("set_speed").kind is ConfirmationKind.NOTIFY


def test_set_volume_is_notify() -> None:
    policy = load_default_policy()
    assert policy.classify("set_volume").kind is ConfirmationKind.NOTIFY


def test_set_dj_mode_is_notify() -> None:
    policy = load_default_policy()
    assert policy.classify("set_dj_mode").kind is ConfirmationKind.NOTIFY


def test_speak_text_is_pass_through() -> None:
    """§11.2 acceptance: speak_text is 🟢 pass_through (no confirm)."""
    policy = load_default_policy()
    decision = policy.classify("speak_text")
    assert decision.kind is ConfirmationKind.PASS_THROUGH
    assert decision.plan_text is None  # no awaiting prompt


def test_play_sound_is_pass_through() -> None:
    policy = load_default_policy()
    assert policy.classify("play_sound").kind is ConfirmationKind.PASS_THROUGH


def test_play_animation_is_pass_through() -> None:
    policy = load_default_policy()
    assert policy.classify("play_animation").kind is ConfirmationKind.PASS_THROUGH


def test_get_tools_are_pass_through() -> None:
    """All get_*/list_* tools are 🟢 read-only."""
    policy = load_default_policy()
    for name in (
        "get_current_time",
        "get_robot_status",
        "get_battery_level",
        "get_current_pose",
        "get_music_state",
        "get_sound_info",
        "get_perception_context",
        "list_waypoints",
        "list_tracks",
    ):
        assert policy.classify(name).kind is ConfirmationKind.PASS_THROUGH, name


def test_memory_tools_are_pass_through() -> None:
    policy = load_default_policy()
    for name in ("memory_save", "memory_search", "memory_context"):
        assert policy.classify(name).kind is ConfirmationKind.PASS_THROUGH, name


def test_estimate_tts_duration_is_pass_through() -> None:
    """§6.2: estimate_tts_duration is internal — not requiring confirm."""
    policy = load_default_policy()
    assert policy.classify("estimate_tts_duration").kind is ConfirmationKind.PASS_THROUGH


def test_unknown_tool_uses_default_class() -> None:
    """Unknown tools fall back to the safe default (pass_through)."""
    policy = load_default_policy()
    decision = policy.classify("totally_unknown_tool_xyz")
    assert decision.kind is ConfirmationKind.PASS_THROUGH
    assert "unknown" in decision.reason.lower()


# ---------------------------------------------------------------------------
# Gold rule — stop_navigation NEVER requires confirmation
# ---------------------------------------------------------------------------


def test_stop_navigation_never_requires_confirm() -> None:
    """§11.2 acceptance (property-style): stop_navigation is NEVER require.

    This is the single most important invariant in §8.2: «аварийные
    команды — мгновенно и без вопросов». We check it explicitly so
    that an accidental edit to the YAML doesn't silently reclassify
    the emergency stop into a blocking call.
    """
    policy = load_default_policy()
    decision = policy.classify("stop_navigation")
    assert decision.kind is ConfirmationKind.PASS_THROUGH
    assert decision.kind is not ConfirmationKind.REQUIRE


@pytest.mark.parametrize("forbidden_tool", sorted(EMERGENCY_TOOLS))
def test_emergency_tools_never_require_confirm_parametrized(forbidden_tool: str) -> None:
    """Property-based: every tool in EMERGENCY_TOOLS must classify as pass_through."""
    policy = load_default_policy()
    assert policy.classify(forbidden_tool).kind is not ConfirmationKind.REQUIRE


def test_from_mapping_rejects_require_for_stop_navigation() -> None:
    """The loader itself rejects reclassifying stop_navigation to require."""
    with pytest.raises(ConfigError) as exc_info:
        ToolConfirmationPolicy.from_mapping(
            {
                "tools": {
                    "stop_navigation": {"class": "require", "reason": "bad idea"},
                },
            }
        )
    assert "stop_navigation" in str(exc_info.value)


def test_extended_with_rejects_require_for_emergency_tool() -> None:
    """Even an override layer cannot make stop_navigation require."""
    base = load_default_policy()
    with pytest.raises(ConfigError) as exc_info:
        base.extended_with({"stop_navigation": {"class": "require", "reason": "no"}})
    assert "stop_navigation" in str(exc_info.value)


def test_extended_with_respects_forbidden_overrides() -> None:
    """Tools listed in forbidden_overrides cannot be reclassified either."""
    base = ToolConfirmationPolicy.from_mapping(
        {
            "tools": {"foo": {"class": "pass_through"}},
            "forbidden_overrides": ["foo"],
        }
    )
    with pytest.raises(ConfigError) as exc_info:
        base.extended_with({"foo": {"class": "require", "reason": "nope"}})
    assert "foo" in str(exc_info.value)


# ---------------------------------------------------------------------------
# Loader schema validation
# ---------------------------------------------------------------------------


def test_from_mapping_rejects_unknown_class() -> None:
    with pytest.raises(ConfigError) as exc_info:
        ToolConfirmationPolicy.from_mapping(
            {"tools": {"x": {"class": "obviously_wrong"}}}
        )
    assert "obviously_wrong" in str(exc_info.value)


def test_from_mapping_rejects_non_mapping_top_level() -> None:
    with pytest.raises(ConfigError):
        ToolConfirmationPolicy.from_mapping("not a mapping")  # type: ignore[arg-type]


def test_from_mapping_rejects_non_mapping_entry() -> None:
    with pytest.raises(ConfigError):
        ToolConfirmationPolicy.from_mapping({"tools": {"x": "not a mapping"}})


def test_from_mapping_rejects_non_string_tool_name() -> None:
    with pytest.raises(ConfigError):
        ToolConfirmationPolicy.from_mapping({"tools": {42: {"class": "pass_through"}}})


def test_from_mapping_rejects_non_string_reason() -> None:
    with pytest.raises(ConfigError):
        ToolConfirmationPolicy.from_mapping(
            {"tools": {"x": {"class": "pass_through", "reason": 123}}}
        )


def test_from_mapping_rejects_non_string_plan_template() -> None:
    with pytest.raises(ConfigError):
        ToolConfirmationPolicy.from_mapping(
            {"tools": {"x": {"class": "require", "plan_template": 42}}}
        )


def test_from_mapping_uses_default_class_when_missing() -> None:
    """Without an explicit default_class we fall back to pass_through."""
    policy = ToolConfirmationPolicy.from_mapping({})
    assert policy.default_kind is ConfirmationKind.PASS_THROUGH


def test_extended_with_rejects_non_mapping_overrides() -> None:
    base = ToolConfirmationPolicy.from_mapping({})
    with pytest.raises(ConfigError):
        base.extended_with("not a mapping")  # type: ignore[arg-type]


def test_extended_with_inherits_unknown_tools() -> None:
    """Override layer should keep the base catalog intact for non-overridden tools."""
    base = ToolConfirmationPolicy.from_mapping(
        {"tools": {"a": {"class": "require"}, "b": {"class": "pass_through"}}}
    )
    override = base.extended_with({"a": {"class": "pass_through"}})
    assert override.classify("a").kind is ConfirmationKind.PASS_THROUGH
    assert override.classify("b").kind is ConfirmationKind.PASS_THROUGH


# ---------------------------------------------------------------------------
# Decision dataclass
# ---------------------------------------------------------------------------


def test_acceptance_decision_is_frozen() -> None:
    decision = AcceptanceDecision(
        kind=ConfirmationKind.REQUIRE,
        tool="navigate_to_waypoint",
        reason="test",
        plan_text="План",
    )
    with pytest.raises(Exception):
        decision.kind = ConfirmationKind.PASS_THROUGH  # type: ignore[misc]


def test_plan_text_rendered_with_tool_name() -> None:
    """Plan templates may use {tool_name} as a placeholder."""
    policy = ToolConfirmationPolicy.from_mapping(
        {"tools": {"x": {"class": "require", "plan_template": "Go to {tool_name}"}}}
    )
    decision = policy.classify("x")
    assert decision.plan_text == "Go to x"


def test_decision_echoes_tool_name() -> None:
    """Decision.tool mirrors the input so feedback events can use it."""
    policy = load_default_policy()
    decision = policy.classify("navigate_to_waypoint")
    assert decision.tool == "navigate_to_waypoint"