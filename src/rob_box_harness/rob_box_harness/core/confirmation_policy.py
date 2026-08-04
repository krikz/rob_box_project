"""``ToolConfirmationPolicy`` — pure-data acceptance classifier (issue #968 §8).

This module is the **catalog** of every tool that needs user
confirmation before it is allowed to touch the hardware. It is
intentionally **pure Python** — no ``rclpy``, no LLM, no ROS2
transport. The :class:`ToolConfirmationPolicy` is consulted by
:meth:`rob_box_harness.core.dialog_core.DialogCore` (or any other
tool-call orchestrator) before a tool result is delivered to the
executor.

Three policy classes, matching the §8.2 table of
``docs/design/SCHEDULER_DESIGN.md``:

* ``require`` — physically destructive / hard-to-reverse calls. The
  orchestrator must put the segment into ``AWAITING_CONFIRMATION``
  and only release it to the executor after the user says "да".
  Examples: ``navigate_to_waypoint``, ``delete_waypoint``,
  ``start_mapping``.
* ``notify`` — reversible calls that warrant a spoken announcement
  but do **not** block execution. Examples: ``set_speed``,
  ``set_dj_mode``, ``set_volume``.
* ``pass_through`` — read-only, safe, or emergency calls. The segment
  goes straight to ``PENDING → ACTIVE``. Examples: ``speak_text``,
  ``stop_navigation`` (🟢 аварийный, см. §8.2 golden rule),
  ``get_*``, ``list_*``.

The "gold rule" of §8.2 — *emergency commands never block* — is
enforced by an explicit unit test
(``tests/test_confirmation_policy.py::test_stop_navigation_never_requires_confirm``)
and by the loader's **forbidden-override** check (an attempt to
reclassify an emergency tool to ``require`` is rejected at load
time, not at runtime).

The classifier is data-only — no tool names are hard-coded in Python.
The mapping lives in :file:`confirmation_policy.yaml` (this
package's ``data/`` directory) so that the policy can be edited
without redeploying the binary, and so that property-based tests can
fuzz arbitrary tool-name combinations.

Usage::

    from rob_box_harness.core.confirmation_policy import (
        load_default_policy, ToolConfirmationPolicy,
    )

    policy = load_default_policy()        # reads data/confirmation_policy.yaml
    decision = policy.classify("navigate_to_waypoint")
    assert decision.kind == "require"

    # Override / layer a per-deployment policy on top:
    from rob_box_harness.core.confirmation_policy import ToolConfirmationPolicy
    custom = ToolConfirmationPolicy.from_mapping({
        "tools": {"stop_navigation": {"class": "pass_through"}},
        "forbidden_overrides": ["stop_navigation"],
    })
    decision2 = custom.classify("stop_navigation")
    assert decision2.kind == "pass_through"

The :class:`AcceptanceDecision` returned by :meth:`classify` is a
frozen dataclass — safe to compare, hash, log, and pass through
asyncio queues without defensive copies.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from importlib import resources
from typing import Any, Mapping

from rob_box_harness.errors import ConfigError


# ---------------------------------------------------------------------------
# Public types
# ---------------------------------------------------------------------------


class ConfirmationKind(str, Enum):
    """The three §8.2 policy classes.

    Stored as a ``str``-valued enum so values round-trip through YAML
    and JSON without bespoke (de)serialisers. The string value is
    the canonical on-the-wire spelling used in the YAML catalog and in
    the ``AcceptanceDecision`` formatter output (§7 feedback events).
    """

    REQUIRE = "require"
    NOTIFY = "notify"
    PASS_THROUGH = "pass_through"


#: Tools that are emergency / safety / read-only by contract and
#: **must never** be downgraded to ``require``. Listed here in code
#: (not YAML) because the gold rule is a hard invariant of §8.2 — an
#: operator editing the YAML by mistake must not be able to silently
#: turn ``stop_navigation`` into a blocking call. The YAML loader
#: rejects any attempt to reclassify these names to ``require`` with a
#: :class:`ConfigError` pointing at the offending line.
EMERGENCY_TOOLS: frozenset[str] = frozenset({"stop_navigation"})


@dataclass(frozen=True)
class AcceptanceDecision:
    """Outcome of :meth:`ToolConfirmationPolicy.classify`.

    Attributes:
        kind: The policy class. One of ``require`` / ``notify`` /
            ``pass_through`` (see :class:`ConfirmationKind`).
        tool: The tool name that was classified. Echoed back so
            log lines and feedback events can include it without a
            second lookup.
        reason: Short human-readable string explaining *why* this
            classification was chosen. Used in the ``[AWAITING]``
            block of §7 feedback events and in the rejection message
            returned to the LLM when the segment is rejected.
        plan_text: Russian phrase describing the plan — only set when
            ``kind == require`` (e.g. ``"План: еду на кухню через
            гостиную, 15 секунд. Подтверждаешь?"``). ``None`` for
            ``notify`` / ``pass_through`` — those do not need an
            awaiting prompt, only an optional announcement.
    """

    kind: ConfirmationKind
    tool: str
    reason: str
    plan_text: str | None = None


# ---------------------------------------------------------------------------
# The classifier
# ---------------------------------------------------------------------------


class ToolConfirmationPolicy:
    """Pure-data classifier that maps tool names to confirmation classes.

    Constructed from a mapping that mirrors the schema of
    ``confirmation_policy.yaml``::

        {
            "tools": {
                "<tool_name>": {"class": "require" | "notify" | "pass_through",
                                "reason": "<short string>",
                                "plan_template": "<ru phrase with {args}>"},
                ...
            },
            "default_class": "pass_through",
            "forbidden_overrides": ["stop_navigation"],
        }

    The mapping is immutable after construction — the policy is a
    value, not a stateful object. To build a layered policy
    (``base.yaml`` + ``pi-main.yaml``), use
    :meth:`extended_with` which returns a *new* classifier with the
    overrides applied on top.
    """

    __slots__ = (
        "_tools",
        "_default_kind",
        "_forbidden_overrides",
        "_plans",
    )

    def __init__(
        self,
        *,
        tools: Mapping[str, tuple[ConfirmationKind, str, str | None]],
        default_kind: ConfirmationKind,
        forbidden_overrides: frozenset[str],
    ) -> None:
        # Defensive copies so the caller cannot mutate the classifier
        # through the dicts they handed us.
        self._tools: dict[str, tuple[ConfirmationKind, str, str | None]] = dict(tools)
        self._default_kind = default_kind
        self._forbidden_overrides = frozenset(forbidden_overrides)
        # Sanity: every emergency tool must resolve to pass_through.
        # This is enforced both here and in ``from_mapping`` so that
        # there is no path that constructs an unsafe classifier.
        for tool in EMERGENCY_TOOLS:
            entry = self._tools.get(tool)
            if entry is not None and entry[0] is ConfirmationKind.REQUIRE:
                raise ConfigError(
                    f"emergency tool '{tool}' must not be classified as 'require' "
                    f"(see SCHEDULER_DESIGN.md §8.2 gold rule)",
                    section=f"confirmation.tools.{tool}.class",
                )

    # ----- factory --------------------------------------------------------

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any]) -> "ToolConfirmationPolicy":
        """Build a classifier from a parsed YAML dict.

        Validation rules (kept strict — fail-fast at startup):

        * ``tools`` must be a mapping of name → entry mapping.
        * ``entry.class`` must be one of ``require`` / ``notify`` /
          ``pass_through`` (case-sensitive).
        * An emergency tool (currently just ``stop_navigation``)
          **must not** be classified as ``require``.
        * ``default_class`` is optional; falls back to ``pass_through``
          (the safest default — unknown tools do not block).
        * ``forbidden_overrides`` is an optional list of tool names
          that must not appear as ``require`` even when extended via
          :meth:`extended_with`.
        """
        if not isinstance(raw, Mapping):
            raise ConfigError(
                "confirmation policy must be a mapping; "
                f"got {type(raw).__name__}",
                section="confirmation",
            )
        tools_raw = raw.get("tools", {})
        if not isinstance(tools_raw, Mapping):
            raise ConfigError(
                "'confirmation.tools' must be a mapping",
                section="confirmation.tools",
            )
        default_raw = raw.get("default_class", "pass_through")
        try:
            default_kind = ConfirmationKind(default_raw)
        except ValueError as exc:
            raise ConfigError(
                f"'confirmation.default_class' must be one of "
                f"require / notify / pass_through; got {default_raw!r}",
                section="confirmation.default_class",
            ) from exc
        forbidden_raw = raw.get("forbidden_overrides", ())
        if forbidden_raw is None:
            forbidden_raw = ()
        if not isinstance(forbidden_raw, (list, tuple)):
            raise ConfigError(
                "'confirmation.forbidden_overrides' must be a list",
                section="confirmation.forbidden_overrides",
            )
        forbidden = frozenset(forbidden_raw)
        if any(not isinstance(name, str) for name in forbidden):
            raise ConfigError(
                "'confirmation.forbidden_overrides' entries must be strings",
                section="confirmation.forbidden_overrides",
            )

        tools: dict[str, tuple[ConfirmationKind, str, str | None]] = {}
        for tool_name, entry in tools_raw.items():
            if not isinstance(tool_name, str) or not tool_name:
                raise ConfigError(
                    "tool names must be non-empty strings",
                    section="confirmation.tools",
                )
            if not isinstance(entry, Mapping):
                raise ConfigError(
                    f"confirmation entry for '{tool_name}' must be a mapping",
                    section=f"confirmation.tools.{tool_name}",
                )
            class_raw = entry.get("class")
            try:
                kind = ConfirmationKind(class_raw)
            except ValueError as exc:
                raise ConfigError(
                    f"confirmation class for '{tool_name}' must be one of "
                    f"require / notify / pass_through; got {class_raw!r}",
                    section=f"confirmation.tools.{tool_name}.class",
                ) from exc
            reason = entry.get("reason", "")
            if not isinstance(reason, str):
                raise ConfigError(
                    f"confirmation.reason for '{tool_name}' must be a string",
                    section=f"confirmation.tools.{tool_name}.reason",
                )
            plan_template = entry.get("plan_template")
            if plan_template is not None and not isinstance(plan_template, str):
                raise ConfigError(
                    f"confirmation.plan_template for '{tool_name}' must be a string",
                    section=f"confirmation.tools.{tool_name}.plan_template",
                )
            tools[tool_name] = (kind, reason, plan_template)

        return cls(
            tools=tools,
            default_kind=default_kind,
            forbidden_overrides=forbidden,
        )

    # ----- public API -----------------------------------------------------

    @property
    def default_kind(self) -> ConfirmationKind:
        """The fallback class for tools not present in the catalog."""
        return self._default_kind

    @property
    def known_tools(self) -> frozenset[str]:
        """The set of tool names this policy knows about (read-only view)."""
        return frozenset(self._tools)

    def classify(self, tool_name: str) -> AcceptanceDecision:
        """Return the :class:`AcceptanceDecision` for *tool_name*.

        Unknown tools resolve to the policy's ``default_kind`` with a
        generic reason — this keeps the orchestrator forward-compatible
        with new tool registrations without a policy reload.
        """
        entry = self._tools.get(tool_name)
        if entry is None:
            return AcceptanceDecision(
                kind=self._default_kind,
                tool=tool_name,
                reason=f"unknown tool — default class '{self._default_kind.value}'",
                plan_text=None,
            )
        kind, reason, plan_template = entry
        return AcceptanceDecision(
            kind=kind,
            tool=tool_name,
            reason=reason or f"policy: {kind.value}",
            plan_text=_render_plan(plan_template, tool_name=tool_name),
        )

    def extended_with(
        self,
        overrides: Mapping[str, Any],
    ) -> "ToolConfirmationPolicy":
        """Return a new classifier with the given overrides applied.

        The result is a layered policy: tools present in *overrides*
        take precedence; everything else is inherited from ``self``.
        An attempt to set ``class: require`` for a tool listed in
        ``forbidden_overrides`` raises :class:`ConfigError` (so that
        a per-deployment YAML cannot accidentally weaken the gold rule).

        This is the building block for the ``base.yaml + pi-main.yaml``
        idiom already used by :mod:`rob_box_harness.config` — the
        acceptance layer uses the same composition pattern.
        """
        if not isinstance(overrides, Mapping):
            raise ConfigError(
                "policy overrides must be a mapping",
                section="confirmation.overrides",
            )
        merged: dict[str, tuple[ConfirmationKind, str, str | None]] = dict(self._tools)
        for tool_name, entry in overrides.items():
            if tool_name in self._forbidden_overrides:
                raise ConfigError(
                    f"tool '{tool_name}' is in 'forbidden_overrides' and "
                    f"cannot be reclassified by an override layer",
                    section=f"confirmation.overrides.{tool_name}",
                )
            if tool_name in EMERGENCY_TOOLS:
                # Defensive: even outside the forbidden_overrides
                # mechanism, never allow an emergency tool to be
                # reclassified to require via a layer file.
                if isinstance(entry, Mapping) and entry.get("class") == "require":
                    raise ConfigError(
                        f"emergency tool '{tool_name}' cannot be reclassified "
                        f"to 'require' (§8.2 gold rule)",
                        section=f"confirmation.overrides.{tool_name}.class",
                    )
            if not isinstance(entry, Mapping):
                raise ConfigError(
                    f"override entry for '{tool_name}' must be a mapping",
                    section=f"confirmation.overrides.{tool_name}",
                )
            class_raw = entry.get("class")
            try:
                kind = ConfirmationKind(class_raw)
            except ValueError as exc:
                raise ConfigError(
                    f"override class for '{tool_name}' must be require / "
                    f"notify / pass_through; got {class_raw!r}",
                    section=f"confirmation.overrides.{tool_name}.class",
                ) from exc
            reason = entry.get("reason", "")
            plan_template = entry.get("plan_template")
            merged[tool_name] = (kind, str(reason), plan_template)
        return ToolConfirmationPolicy(
            tools=merged,
            default_kind=self._default_kind,
            forbidden_overrides=self._forbidden_overrides,
        )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _render_plan(template: str | None, *, tool_name: str) -> str | None:
    """Format a plan phrase for a ``require`` decision.

    Plan templates may use ``{tool_name}`` (the LLM-friendly tool
    name) but **not** arbitrary tool arguments — we do not have them
    here, and they should not bleed into the awaiting prompt anyway.
    Templates that mention ``{arg}`` are returned verbatim so that a
    future formatter (which *does* know the args) can re-render them.
    For now the orchestrator is expected to override ``plan_text``
    when it has the actual arguments at hand (see §8.3 step 2).
    """
    if template is None:
        return None
    try:
        return template.format(tool_name=tool_name)
    except (KeyError, IndexError):
        # Unknown placeholder — return raw, the caller may re-render.
        return template


# ---------------------------------------------------------------------------
# Default loader
# ---------------------------------------------------------------------------


def _default_policy_text() -> str:
    """Read the bundled YAML catalog as text.

    Returns:
        The full YAML text of :file:`data/confirmation_policy.yaml`.

    Raises:
        ConfigError: if the bundled file cannot be read (a packaging
            mistake, not a user error — fail loudly).
    """
    try:
        return (
            resources.files("rob_box_harness.core.data")
            .joinpath("confirmation_policy.yaml")
            .read_text(encoding="utf-8")
        )
    except (FileNotFoundError, OSError) as exc:
        raise ConfigError(
            "bundled confirmation_policy.yaml is missing from "
            "rob_box_harness.core.data — packaging error",
            section="confirmation",
        ) from exc


def load_default_policy() -> ToolConfirmationPolicy:
    """Load the default :class:`ToolConfirmationPolicy` from bundled YAML.

    The bundled file mirrors the §8.2 classification table and is
    the single source of truth for which tools require confirmation.
    Tests use :meth:`ToolConfirmationPolicy.from_mapping` to build
    synthetic policies instead of mutating this one.
    """
    import yaml  # local import — see config.py for the rationale

    text = _default_policy_text()
    parsed = yaml.safe_load(text)
    if parsed is None:
        parsed = {}
    if not isinstance(parsed, Mapping):
        raise ConfigError(
            "confirmation_policy.yaml must be a mapping at the top level",
            section="confirmation",
        )
    return ToolConfirmationPolicy.from_mapping(parsed)


__all__ = [
    "AcceptanceDecision",
    "ConfirmationKind",
    "EMERGENCY_TOOLS",
    "ToolConfirmationPolicy",
    "load_default_policy",
]