"""Issue #3220 — ``extra={"thinking": None}`` значит «поле thinking не слать».

Включённый thinking у MiniMax-M3 — дефолт модели, когда поля нет: так было
вживую с b5879b79 (05.08, ``thinking=None``) до 6901a14e (06.08, ответы с
``<think>``, +10-20 с). ``{"type": "enabled"}`` с API не сверялся.
Проверяем, что пер-вызовное ``None`` пересиливает дефолт инстанса
(``{"type": "disabled"}``) и не уходит в запрос как ``"thinking": null``.
"""

from __future__ import annotations

import asyncio
from types import SimpleNamespace
from typing import Any

from rob_box_llm.provider import LLMMessage, LLMSettings
from rob_box_llm.providers.minimax import DEFAULT_THINKING_POLICY, MiniMaxProvider


class _Completions:
    def __init__(self) -> None:
        self.calls: list[dict[str, Any]] = []

    async def create(self, **kwargs: Any) -> Any:
        self.calls.append(kwargs)
        message = SimpleNamespace(content="ок", tool_calls=None)
        return SimpleNamespace(
            choices=[SimpleNamespace(message=message, finish_reason="stop")],
            usage=None,
            base_resp={},
        )


def _sent(thinking_instance: Any, extra: dict[str, Any]) -> dict[str, Any]:
    completions = _Completions()
    client = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    provider = MiniMaxProvider(api_key="sk-test", client=client, thinking=thinking_instance)  # type: ignore[arg-type]
    asyncio.run(
        provider.complete(
            [LLMMessage(role="user", content="сочини трек")],
            settings=LLMSettings(extra=extra),
        )
    )
    return completions.calls[0]


def test_default_instance_still_sends_disabled() -> None:
    assert _sent(DEFAULT_THINKING_POLICY, {})["extra_body"]["thinking"] == {"type": "disabled"}


def test_per_call_none_omits_the_field_over_disabled_default() -> None:
    sent = _sent(DEFAULT_THINKING_POLICY, {"thinking": None})
    assert "thinking" not in sent.get("extra_body", {}), sent.get("extra_body")


def test_per_call_none_omits_the_field_without_instance_policy() -> None:
    sent = _sent(None, {"thinking": None})
    assert "thinking" not in sent.get("extra_body", {}), sent.get("extra_body")


def test_other_extra_fields_survive_the_omission() -> None:
    sent = _sent(DEFAULT_THINKING_POLICY, {"thinking": None, "top_k": 5})
    assert sent["extra_body"] == {"top_k": 5}
