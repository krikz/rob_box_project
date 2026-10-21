"""Срез тулов хода в старом пути v1 (ADR-0150 PR-6 #3696): до вызова LLM код решает, какие ≤ 12 схем она увидит.

Вместо пред-роутера скиллов (``skill_router``, 26 регексов) и подгрузки скилла моделью: снимок ноды (тот же, что у
Tier-1) + слова реплики → :func:`rob_box_dialog.tools_view.available_tools` → ``AgentCore.set_visible_tools``. Одна
строка в лог на ход: ``🧰 [tools-view] groups=… tools(N)=…``.
"""

from __future__ import annotations

from typing import Any

from rob_box_dialog.tools_view import available_tools, groups_for
from rob_box_voice.core.tier1_host import Tier1Runner


def apply_tools_view(node: Any, text: str) -> None:
    """Задать срез тулов следующего вызова LLM; без ``AgentCore`` (тестовые ноды) — ничего."""
    core = getattr(node, "_core", None)
    if core is None:
        return
    runner = getattr(node, "_tier1", None) or Tier1Runner(node)
    snapshot = runner.snapshot()
    caps = getattr(getattr(node, "_llm", None), "capabilities", None)
    carry = getattr(node, "_tools_view_groups", ())  # группы прошлого хода: «я же просил про стрижей» — продолжение
    hidden = getattr(node, "_turn_hidden_tools", ()) or ()  # #3765: личность выясняет робот — без register_speaker
    names = [t.name for t in available_tools(snapshot, caps, "personality", text, carry) if t.name not in hidden]
    groups = groups_for(snapshot, text, carry)
    node._tools_view_groups = groups
    label = "+".join(groups)
    core.set_visible_tools(names, label)
    node.get_logger().info(f"🧰 [tools-view] groups={label} tools({len(names)})={','.join(names)}")


__all__ = ["apply_tools_view"]
