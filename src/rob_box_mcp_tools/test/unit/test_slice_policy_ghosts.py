"""ADR-0150 D17 (PR-13 #3708): в ``slice_policy.yaml`` нет призраков — имён без тула в каталоге.

Срез ``operator.control`` перечислял пять имён (``dialogue_pause``, ``dialogue_resume``, ``set_voice_preset``,
``set_voice_language``, ``preview_voice``), которых не было ни в каталоге, ни в ``mcp_server``: право на вызов
несуществующего ничего не давало, а читатель политики думал, что ТАРС так умеет.
"""

from __future__ import annotations

from importlib import resources

import yaml

from rob_box_core.tool_catalog import tool_names


def _policy() -> dict:
    text = resources.files("rob_box_mcp_tools.data").joinpath("slice_policy.yaml").read_text(encoding="utf-8")
    return yaml.safe_load(text)


def test_slice_policy_has_no_ghosts():
    known = set(tool_names())
    ghosts = sorted((name, tool) for name, tools in _policy()["slices"].items() for tool in tools or ()
                    if tool not in known)
    assert ghosts == []


def test_operator_control_slice_is_gone():
    policy = _policy()
    assert "operator.control" not in policy["slices"]
    assert all("operator.control" not in slices for slices in policy["senders"].values())


def test_operator_view_is_the_operator_slices_of_the_policy():
    """PR-13b: реестр ТАРС = видимые Личности ∪ срезы ``operator.*`` сендера ``avatar_supervisor`` — одно знание
    (срез), а не флаг ``operator_visible`` рядом с ним; в 13a ``show_metrics`` был в политике, но не в срезе хода."""
    from rob_box_core.tool_catalog import llm_visible_tools, operator_visible_tools

    policy = _policy()
    granted = {t for s in policy["senders"]["avatar_supervisor"] if s.startswith("operator.")
               for t in policy["slices"][s]}
    llm = {e.name for e in llm_visible_tools()}
    assert {e.name for e in operator_visible_tools()} == llm | granted
