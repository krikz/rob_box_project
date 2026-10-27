"""Issue #3323 — отказ исполнителя в теле ответа — не успех роутера.

Живой прогон 01.10.2026 16:24Z: media-router исполнил ``stop_music`` и
получил отказ ``{"success": false, ...}`` при ``is_error=False``; роутер писал
``ok=True`` и говорил «остановил» поверх отказа. Успех тула роутера теперь
решает и транспорт, и тело (:func:`media_tool_succeeded`).

ADR-0149 PR-13a: часть про DJ_AUTO-ход (запрет ``stop_music`` в автопереходе
#3246, ``TURN_IS_DJ_AUTO``) удалена вместе с DJ-контроллером старого пути.
PR-15 (#3711, часть 15p): сквозная проверка роутера в ноде диалога v1 удалена вместе с ней.
"""

from __future__ import annotations

import json

from rob_box_voice.core.media_router import media_tool_succeeded


def test_media_tool_succeeded_sees_success_false_body() -> None:
    refusal = json.dumps({"success": False, "error": "tool_forbidden"})
    assert media_tool_succeeded(False, refusal) is False
    assert media_tool_succeeded(True, "{}") is False
    assert media_tool_succeeded(False, json.dumps({"success": True})) is True
    assert media_tool_succeeded(False, "Музыка остановлена") is True
