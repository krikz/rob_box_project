"""Причины предохранителя ``topic_publish`` и фразы диалога не расходятся (issue #4516, ADR-0148).

Лежит здесь, а не в ``rob_box_dialog/test``: пакет dialog не импортирует ``rob_box_mcp_tools``
(в core-venv его нет).
"""

from __future__ import annotations

from rob_box_dialog import knowledge as kn
from rob_box_dialog.respond import ActionResult, phrase_from_result


def _res(status, data=None, reason=None):
    return ActionResult(
        "t/0", 0, "telegram", "topic_publish", kn.TOOL_CLASSES["topic_publish"], status, reason, data or {}
    )


def test_every_reason_the_module_can_return_has_a_phrase():
    """Причины предохранителя (``topic_publish``) и ключи фраз (``ru.yaml``) не расходятся."""
    from rob_box_mcp_tools import topic_publish as tp

    reasons = {
        tp.TOPIC_FORBIDDEN,
        tp.TYPE_FORBIDDEN,
        tp.NO_SUCH_TOPIC,
        tp.NO_ENDPOINTS,
        tp.AMBIGUOUS_TYPE,
        tp.BAD_FIELDS,
        tp.BAD_COUNT,
        tp.BAD_TOPIC,
        tp.PUBLISH_FAILED,
        tp.ROS_UNAVAILABLE,
        tp.TYPE_UNAVAILABLE,
    }
    for reason in reasons:
        result = _res("rejected", {"summary": f"S-{reason}"}, reason)
        assert phrase_from_result(result).text == f"S-{reason}", reason
    assert tp.NEEDS_CONFIRMATION == "needs_publish_confirmation"
    assert ("topic_publish", tp.NEEDS_CONFIRMATION) in kn.TOOL_QUESTIONS
