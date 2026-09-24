"""Public outcome types returned by the tool loop.

Issue #2630 — ``_ToolLoopOutcome`` was previously defined inside
``agent_core.py`` and imported back from :mod:`rob_box_harness.core.tool_loop.output_format`,
which produced a circular import when ``agent_core`` itself started
importing from ``tool_loop``. Promoting the dataclass to its own
module breaks the cycle and keeps the output-format helpers free of
any AgentCore dependency.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class _ToolLoopOutcome:
    """Что вернул тул-цикл :meth:`AgentCore._run_with_tools`.

    Раньше это был безымянный кортеж, который вызывающая сторона
    распаковывала одной строкой в 118 символов. Аннотация при этом
    обещала шесть элементов, а ``return`` отдавал семь — разъехались
    молча, потому что распаковка длину не проверяет.

    Fields
    ------
    spoken_text:
        Финальный текст модели. Пустая строка, когда ход подавлен
        (babble-фильтр issue #1253).
    tools_called:
        Уникальные имена вызванных тулов, в порядке первого вызова.
    finish_reason:
        ``finish_reason`` последнего ответа — нужен ноде, чтобы отличить
        пустой ответ от обрыва по ``length``.
    raw_response:
        Сырой ответ провайдера, для логов.
    speak_text_count:
        Сколько раз модель ЗВАЛА ``speak_text`` (issue #992: отличает
        BACKING-ход от TRACK-хода).
    speak_text_real_count:
        Сколько из них несли непустой ``text`` и реально бы прозвучали
        (issue #1343 — deepseek шлёт ``speak_text({})``).
    spoken_via_tool:
        Что реально произнесено через ``speak_text``, склеенное через
        перевод строки. Пишется в историю вместо маркера «done», иначе
        модель начинает отвечать «done» сама.
    truncated_tool_args:
        Issue #1899 — propagated from the last ``LLMResponse.truncated_tool_args``.
        ``True`` when the last stream assembled tool-call arguments that
        were cut off mid-JSON (most often ``finish_reason='length'``).
    track_name:
        Issue #2857 — the ``name`` argument of the LAST ``compose_music``
        call this turn (``None`` if compose_music wasn't called, or was
        called without a usable ``name``). Cheap, already-available data
        the DJ fallback uses to announce the track it just started
        instead of the generic «Готово, играю.».
    music_call_args:
        Issue #2967 — full argument dict of the LAST ``compose_music``
        call this turn (``None`` if compose_music wasn't called). Same
        "last wins" capture as ``track_name`` but keeps every argument,
        not just ``name`` — dialogue_node compares it against the
        previous turn's stored args to catch a spoken action-claim
        backed by a no-op replay of the same ``compose_music`` call.
    tool_error_occurred:
        Issue #2949 — ``True`` when at least one tool call THIS TURN
        returned ``is_error=True`` (refusal / exception surfaced as a
        tool-role message). A tool being CALLED is not the same as it
        SUCCEEDING: a refused ``save_arrangement_preset`` still lands in
        ``tools_called`` (issue #2942's ``CLAIM_JUSTIFYING_TOOLS``
        whitelist matches on NAME only), letting the LLM claim success
        for an action that never happened. Previously computed locally
        as ``tool_error_occurred`` in ``AgentCore._run_with_tools`` and
        discarded after feeding the babble filter (issue #1253) — now
        propagated so dialogue_node's action-claim guards can tell
        "called and worked" from "called and failed".
    """
    spoken_text: str
    tools_called: list[str]
    finish_reason: str | None
    raw_response: object
    speak_text_count: int
    speak_text_real_count: int
    spoken_via_tool: str
    truncated_tool_args: bool = False
    track_name: str | None = None
    music_call_args: dict[str, Any] | None = None
    tool_error_occurred: bool = False


__all__ = ["_ToolLoopOutcome"]
