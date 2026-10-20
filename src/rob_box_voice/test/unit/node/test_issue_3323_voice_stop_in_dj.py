"""Issue #3323 — отказ исполнителя в теле ответа — не успех роутера.

Живой прогон 01.10.2026 16:24Z: media-router исполнил ``stop_music`` и
получил отказ ``{"success": false, ...}`` при ``is_error=False``; роутер писал
``ok=True`` и говорил «остановил» поверх отказа. Успех тула роутера теперь
решает и транспорт, и тело (:func:`media_tool_succeeded`).

ADR-0149 PR-13a: часть про DJ_AUTO-ход (запрет ``stop_music`` в автопереходе
#3246, ``TURN_IS_DJ_AUTO``) удалена вместе с DJ-контроллером старого пути.
"""

from __future__ import annotations

import json

from rob_box_llm.provider import ToolResult

from rob_box_voice.core.media_router import (
    STOP_FAIL_TEXT,
    media_tool_succeeded,
)

from .test_issue_3134_media_router_node import _make_node, _stt, run_plans  # noqa: F401


def test_media_tool_succeeded_sees_success_false_body() -> None:
    refusal = json.dumps({"success": False, "error": "tool_forbidden"})
    assert media_tool_succeeded(False, refusal) is False
    assert media_tool_succeeded(True, "{}") is False
    assert media_tool_succeeded(False, json.dumps({"success": True})) is True
    assert media_tool_succeeded(False, "Музыка остановлена") is True
    assert media_tool_succeeded(False, json.dumps({"status": "queued"})) is True


def test_router_logs_ok_false_and_says_fail_on_success_false_result(run_plans):  # noqa: F811
    class _Refusing:
        async def execute(self, call):
            return ToolResult(
                tool_call_id=call.id,
                content=json.dumps({"success": False, "error": "tool_forbidden"}),
                is_error=False,
            )

    n = _make_node(playing=True, track="Still Dre", state_name="DIALOGUE")
    n._scheduler_executor = _Refusing()
    _stt(n, "Робот стоп музыка")
    run_plans()
    logged = [c.args[0] for c in n.get_logger().info.call_args_list]
    assert any("stop_music" in m and "ok=False" in m for m in logged), logged
    n._speak_direct.assert_called_once_with(STOP_FAIL_TEXT)
