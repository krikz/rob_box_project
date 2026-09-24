"""CI guard: every ``SKILL_TOOLS`` entry must be reachable by ``dialogue_node``.

Issue #2947 — live 24.09.2026: ``save_arrangement_preset`` / ``preview_arrangement``
(ADR-0132 PR-5/PR-7, issue #2942) were registered in the tool catalog
(``tools/gen_tool_catalog.py::SKILL_TOOLS``) and wired up on ``mcp_server``, but
never added to ``slice_policy.yaml``. Unit tests for both features run against a
synthetic ``ToolSliceAuthority`` (or skip the slice guard entirely), so nothing
caught the gap — it only showed up on the robot, at night, as a live refusal:

    "Инструмент 'save_arrangement_preset' недоступен: tool
    'save_arrangement_preset' не принадлежит ни одному срезу sender'а
    'dialogue_node' (доступные срезы: core, personality)"

This is exactly the ``lookup_melody``/``search_melody`` regression from 10.09
(see ``test_dialogue_node_can_call_melody_tools`` in
``test_mcp_server_slice_guard.py``) happening again for a different pair of
tools. The fix there was a one-line YAML addition with no test that would have
caught the *next* tool doing the same thing.

This module is that missing guard: it walks the single source of truth for
"what tools the LLM personality is offered" (``SKILL_TOOLS`` in
``tools/gen_tool_catalog.py`` — the same dict ``test_tool_catalog_sync.py``
already treats as authoritative for skill membership) and asserts every tool
name in it is allowed for sender ``dialogue_node`` against the *bundled*
``slice_policy.yaml`` (``load_default_authority()``, not a synthetic mapping).
A new tool added to a skill without a matching ``slice_policy.yaml`` entry now
fails in CI instead of on the robot.
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys

import pytest

from rob_box_mcp_tools.slice_authority import load_default_authority

# test/unit/test_x.py -> parents[2] == src/rob_box_mcp_tools, its parent's
# parent's parent is the repo root (mirrors test_tool_catalog_sync.py, which
# lives one directory higher and uses parents[3] for the same repo root).
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_GENERATOR = _REPO_ROOT / "tools" / "gen_tool_catalog.py"

pytestmark = pytest.mark.skipif(
    not _GENERATOR.exists(),
    reason="running outside a source checkout (installed package): tools/gen_tool_catalog.py unavailable",
)

#: Tools intentionally exempt from dialogue_node slice coverage, with the
#: reason recorded inline. Empty today — every SKILL_TOOLS entry is expected
#: to be reachable by the personality that is offered the skill it lives in.
#: If a future skill is TARS-only or operator-only, add its tools here with a
#: comment explaining why, rather than silently skipping the check.
_INTENTIONAL_EXCEPTIONS: frozenset[str] = frozenset()


@pytest.fixture(scope="module")
def skill_tools() -> dict[str, tuple[str, ...]]:
    """Import ``SKILL_TOOLS`` straight from the generator module.

    Loaded by file path (like ``test_tool_catalog_sync.py``'s ``generator``
    fixture) rather than imported as a package — ``tools/`` is a standalone
    script directory, not an installed package.
    """
    spec = importlib.util.spec_from_file_location("gen_tool_catalog_for_slice_test", _GENERATOR)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.SKILL_TOOLS


def _all_skill_tool_names(skill_tools: dict[str, tuple[str, ...]]) -> set[str]:
    names: set[str] = set()
    for tool_names in skill_tools.values():
        names.update(tool_names)
    return names


def test_every_skill_tool_is_allowed_for_dialogue_node(skill_tools):
    """No ``SKILL_TOOLS`` entry may be invisible to the sender it is offered to.

    ``dialogue_node`` is the sender that actually drives the LLM personality
    at runtime (see ``mcp_server.on_execute_request``'s slice guard); every
    tool a skill advertises to it must resolve against the *bundled*
    ``slice_policy.yaml``, not a test-only synthetic policy.
    """
    authority = load_default_authority()
    all_tools = _all_skill_tool_names(skill_tools) - _INTENTIONAL_EXCEPTIONS

    offenders = []
    for tool in sorted(all_tools):
        decision = authority.is_allowed("dialogue_node", tool)
        if not decision.allowed:
            offenders.append(f"{tool}: {decision.reason}")

    assert not offenders, (
        "SKILL_TOOLS advertises tools that slice_policy.yaml does not grant "
        "sender 'dialogue_node' — add them to the same slice as their "
        "sibling tools in src/rob_box_mcp_tools/rob_box_mcp_tools/data/"
        "slice_policy.yaml, or add an explicit, documented entry to "
        "_INTENTIONAL_EXCEPTIONS in this test:\n  " + "\n  ".join(offenders)
    )


def test_intentional_exceptions_are_still_real_skill_tools(skill_tools):
    """Guard the guard: an exception for a tool that no longer exists is stale."""
    all_tools = _all_skill_tool_names(skill_tools)
    stale = sorted(_INTENTIONAL_EXCEPTIONS - all_tools)
    assert not stale, (
        f"_INTENTIONAL_EXCEPTIONS lists tools no longer in SKILL_TOOLS: {stale} "
        "— remove them, they are not exempting anything anymore"
    )


def test_save_arrangement_preset_and_preview_arrangement_allowed_for_dialogue_node():
    """Issue #2947 regression pin: the two tools the live log actually named.

    Narrower than the generic sweep above, and worded around the exact log
    line from the ADR-0132 PR-5/PR-7 night check, so a future revert of just
    this pair shows up with an unambiguous failure message even if the
    generic sweep is ever weakened.
    """
    authority = load_default_authority()
    for tool in ("save_arrangement_preset", "preview_arrangement"):
        decision = authority.is_allowed("dialogue_node", tool)
        assert decision.allowed is True, (
            f"dialogue_node обязан мочь вызвать {tool} (issue #2947); "
            f"получили reason={decision.reason!r}"
        )
