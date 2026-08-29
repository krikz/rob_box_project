#!/usr/bin/env python3
"""
scheduler.py - MCP-facing schema for the S6 ``task_delta`` tool (issue #968).

**Architecture note (capability-honest, see
docs/architecture/tts-extension-points.md):** ``mcp_server`` runs as
its own ROS2 node/process, entirely separate from ``dialogue_node``.
It has NO access to dialogue_node's in-process ``TaskScheduler`` — that
object lives inside the ``rob_box_voice`` process and is only reachable
same-process, via ``SchedulerToolExecutor`` (S6.2,
``rob_box_voice/scheduler/tool_executor.py``).

``TaskDeltaTool`` exists purely so ``discover()``/the OpenAI tool
manifest advertises ``task_delta``'s name and schema to the LLM. The
REAL execution path is ``SchedulerToolExecutor`` intercepting the call
before it ever reaches this process. If ``execute()`` here is ever
actually invoked, that interception is missing or failed — it MUST
return an honest ``success=False`` (``capability-honest``: no silent
degradation), never a fabricated success.
"""

from __future__ import annotations

from typing import Any, List

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType

# rob_box_voice.scheduler.delta is pure Python, no ROS. Still wrapped
# in try/except (mirrors the tts_voice_registry/tts_text_guard pattern
# elsewhere in this package) so the package stays importable without
# rob_box_voice in minimal environments (CI linter) — but this
# validation is load-bearing in production: without it task_delta
# can't reject malformed ops before ever reaching the scheduler.
try:
    from rob_box_voice.scheduler.delta import DeltaOp, DeltaOpKind, TaskDelta
except ImportError:  # pragma: no cover — minimal environments (CI linter)
    DeltaOp = None  # type: ignore[assignment,misc]
    DeltaOpKind = None  # type: ignore[assignment,misc]
    TaskDelta = None  # type: ignore[assignment,misc]


def _parse_op(raw: Any) -> "DeltaOp":
    """Parse one JSON-ish ``ops[]`` entry into a validated :class:`DeltaOp`.

    Raises ``ValueError``/``TypeError`` on anything malformed — the
    caller turns that into an honest ``MCPToolResult(success=False)``.
    """
    if not isinstance(raw, dict):
        raise TypeError(f"each op must be an object, got {type(raw).__name__}")
    kind_raw = raw.get("kind")
    try:
        kind = DeltaOpKind(kind_raw)
    except ValueError:
        valid = [k.value for k in DeltaOpKind]
        raise ValueError(f"unknown op kind {kind_raw!r}; valid: {valid}") from None
    return DeltaOp(kind=kind, seg_idx=raw.get("seg_idx"), args=raw.get("args"))


class TaskDeltaTool(MCPTool):
    """MERGE delta tool (S6, scheduler-segments-merge plan, issue #968).

    Lets the LLM rewrite/replace/append/drop PENDING segments of an
    active multi-part performance (e.g. fold "и ещё про енота" into a
    song already playing) instead of restarting from scratch. See
    ``RULE #SEGMENT-PLAN`` in ``master_prompt_compact.txt`` (S5.3).
    """

    @property
    def name(self) -> str:
        return "task_delta"

    @property
    def description(self) -> str:
        return (
            "Изменить PENDING-сегменты активного многочастного выступления "
            "(песня/сказка/стих) без перезапуска с начала. Используй когда "
            "в контексте есть [SEGMENT PLAN] с REWRITEABLE_SEGMENTS и "
            "пользователь попросил что-то добавить/изменить/убрать по ходу "
            "исполнения. Операции: rewrite(seg_idx, args) — переписать "
            "сегмент; replace(seg_idx, args) — то же; append(args) — "
            "добавить новый сегмент в конец; drop(seg_idx) — убрать "
            "сегмент. ACTIVE-сегменты (уже играют) трогать нельзя — только "
            "те, что перечислены в REWRITEABLE_SEGMENTS."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="group_id",
                type="string",
                description=(
                    "Значение GROUP_ID из блока [SEGMENT PLAN] — "
                    "скопируй его как есть (32-символьная hex-строка). "
                    "Это НЕ метка сегмента (seg_0, seg_1) и не task_id."
                ),
                required=True,
            ),
            MCPToolParameter(
                name="ops",
                type="array",
                description=(
                    'Список операций. Каждая — объект {"kind": "rewrite|'
                    'replace|append|drop", "seg_idx": int, "args": {...}}. '
                    '"append" не указывает seg_idx; "drop" не указывает args.'
                ),
                required=True,
                items=MCPToolParameter(
                    name="op",
                    type="object",
                    description="Одна операция дельты.",
                    required=True,
                ),
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.FAST

    @property
    def destructive(self) -> bool:
        # append/rewrite/replace/drop only touch PENDING segments of a
        # group the LLM itself is steering — not destructive in the
        # MCPTool sense (irreversible/dangerous side effect on the
        # robot), same reasoning as SetVoiceTool/EstimateTtsDurationTool.
        return False

    def execute(self, group_id: str = "", ops: list | None = None) -> MCPToolResult:
        group_id = (group_id or "").strip()
        if not group_id:
            return MCPToolResult(
                success=False,
                error="group_id_empty",
                message="group_id не может быть пустым.",
            )
        ops = ops or []
        if not ops:
            return MCPToolResult(
                success=False,
                error="ops_empty",
                message="ops не может быть пустым списком.",
            )
        if DeltaOp is None:  # pragma: no cover — minimal environments
            return MCPToolResult(
                success=False,
                error="delta_module_unavailable",
                message="rob_box_voice.scheduler.delta не импортируется в этом окружении.",
            )
        try:
            parsed_ops = tuple(_parse_op(op) for op in ops)
            TaskDelta(group_id=group_id, ops=parsed_ops)
        except (ValueError, TypeError) as exc:
            return MCPToolResult(
                success=False,
                error="invalid_delta",
                message=f"task_delta ops невалидны: {exc}",
            )

        # Capability-honest: see module docstring. This tool's execute()
        # should never actually run in production — SchedulerToolExecutor
        # intercepts "task_delta" in-process before dispatch reaches
        # mcp_server. Reaching here means that interception is missing.
        self.log_warning(
            f"[task_delta] reached mcp_server directly for group={group_id!r} "
            "— SchedulerToolExecutor should have intercepted this call "
            "in-process (S6.2); mcp_server has no TaskScheduler of its own."
        )
        return MCPToolResult(
            success=False,
            error="scheduler_unavailable",
            message=(
                "task_delta не может быть исполнен на стороне mcp_server: "
                "TaskScheduler живёт в процессе dialogue_node, а не здесь. "
                "Честная ошибка вместо молчаливой деградации."
            ),
        )
