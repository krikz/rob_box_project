"""Контрактный тест адаптера (ADR-0150 §4.5, A14; #3686).

FakeLLMProvider в каждом из 4 режимов отвечает на эталонный набор так, как отвечал бы честный провайдер этого режима;
канал речи FieldChannel (решение О4; ToolChannel удалён в PR-7) строит запрос, разбирает ответ, и парсер даёт валидную
команду в 100 %.
Тест проверяет водопровод (запрос -> ответ -> один парсер), а не качество живой модели: живые адаптеры меряет
``scripts/dialog/adapter_benchmark.py``.

Парсер — настоящий ``rob_box_dialog.command.parse`` (PR-1): разобранная команда сверяется с эталонной поле в поле.
"""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import Any, Mapping

import pytest
import yaml

from rob_box_dialog.degrade import MODE_ORDER, select_mode
from rob_box_dialog.speech_channel import (
    COMMAND_TOOL,
    ModeReply,
    FieldChannel,
)
from rob_box_llm.provider import LLMMessage, LLMResponse, LLMSettings, ProviderCapabilities, ToolCall
from rob_box_llm.providers.fake import FakeLLMProvider
from rob_box_dialog.command import Command, Invalid
from rob_box_dialog.command import parse as PARSE

REFERENCE_SET = Path(__file__).resolve().parents[2] / "scripts" / "dialog" / "reference_set.yaml"
SCHEMA = {"type": "object", "properties": {"speech": {"type": "string", "maxLength": 280}}}
#: В эталонном наборе речь хода с действиями помечена тулом v1 ``speak_text``; в команде это поле ``speech``.
SPEAK_TOOL = "speak_text"


def _as_dict(cmd: Command) -> dict[str, Any]:
    """Команда в форме эталона ``_canonical``: только непустые speech / ask / actions / passed."""
    out: dict[str, Any] = {}
    if cmd.speech:
        out["speech"] = cmd.speech
    if cmd.ask:
        out["ask"] = {"expects": cmd.ask.expects, "text": cmd.ask.text}
    if cmd.actions:
        out["actions"] = [{"tool": a.tool, "args": dict(a.args)} for a in cmd.actions]
    if cmd.passed:
        out["passed"] = cmd.passed
    return out


def _nonempty(canonical: dict[str, Any]) -> dict[str, Any]:
    return {k: v for k, v in canonical.items() if v}


# ---- эталонный набор -------------------------------------------------------------------------------------------


def _canonical(entry: dict[str, Any]) -> dict[str, Any] | None:
    """Команда, которую должна выдать модель на фразу; None — LLM не вызывается (tier1)."""
    exp = entry["expected"]
    if "tier1" in exp:
        return None
    if "silence" in exp:
        return {"passed": "not_addressed"}
    if "ask" in exp:
        return {"ask": {"expects": "free", "text": str(exp["ask"])}}
    if "speech_only" in exp:
        return {"speech": "Хорошо."}
    tools = [t for t in exp["actions"] if t != SPEAK_TOOL]
    cmd: dict[str, Any] = {"actions": [{"tool": t, "args": {}} for t in tools]}
    if SPEAK_TOOL in exp["actions"]:
        cmd["speech"] = "Готово."
    return cmd


def load_cases() -> list[tuple[str, dict[str, Any]]]:
    data = yaml.safe_load(REFERENCE_SET.read_text(encoding="utf-8"))
    cases = [(p["id"], c) for p in data["phrases"] if (c := _canonical(p)) is not None]
    assert len(cases) >= 20, "эталонный набор слишком мал для контракта"
    return cases


CASES = load_cases()


# ---- честный fake-провайдер -------------------------------------------------------------------------------------


def caps_for_mode(mode: str) -> ProviderCapabilities:
    return ProviderCapabilities(
        text=True,
        tools=mode in ("structured_output", "tool_choice"),
        structured_output=mode == "structured_output",
        tool_choice=mode in ("structured_output", "tool_choice"),
        json_mode=mode in ("structured_output", "json_mode"),
    )


def _honest_fake(canonical: dict[str, Any], mode: str) -> FakeLLMProvider:
    """Отвечает по тому, что РЕАЛЬНО пришло в запросе (тулы/response_format), а не по заранее известному режиму."""

    async def on_complete(msgs, tools, settings: LLMSettings | None) -> LLMResponse:
        names = [t["function"]["name"] for t in tools]
        if COMMAND_TOOL in names:
            return LLMResponse(tool_calls=(ToolCall("c1", COMMAND_TOOL, canonical),), finish_reason="tool_calls")
        body = json.dumps(canonical, ensure_ascii=False)
        if settings is not None and settings.extra.get("response_format"):
            return LLMResponse(content=body, finish_reason="stop")
        return LLMResponse(content=f"Конечно! {body} Надеюсь, помогло.", finish_reason="stop")

    return FakeLLMProvider(on_complete=on_complete, capabilities=caps_for_mode(mode))


def _ask(provider: FakeLLMProvider, channel, mode: str) -> ModeReply:
    req = channel.build_request(mode)
    extra: Mapping[str, Any] = {"response_format": req.response_format} if req.response_format else {}
    settings = LLMSettings(tool_choice=req.tool_choice, extra=extra)
    msgs = [LLMMessage(role="system", content=req.instruction or "-"), LLMMessage(role="user", content="Робот, ...")]
    resp = asyncio.run(provider.complete(msgs, tools=req.tools, settings=settings))
    return ModeReply(resp.content, tuple((tc.name, tc.arguments) for tc in resp.tool_calls))


# ---- тесты -------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("mode", MODE_ORDER)
def test_fake_caps_select_their_own_mode(mode):
    assert select_mode(caps_for_mode(mode)) == mode


@pytest.mark.parametrize("mode", MODE_ORDER)
def test_field_channel_parses_every_reference_phrase(mode):
    channel = FieldChannel(SCHEMA)
    failures = []
    for pid, canonical in CASES:
        provider = _honest_fake(canonical, mode)
        parsed = channel.parse(_ask(provider, channel, mode), mode, PARSE)
        if not isinstance(parsed, Command):
            failures.append(f"{pid}: {parsed}")
        elif _as_dict(parsed) != _nonempty(canonical):
            failures.append(f"{pid}: {_as_dict(parsed)} != {canonical}")
    assert failures == [], f"[command.parse, mode={mode}] {len(failures)}/{len(CASES)} failed: {failures[:3]}"


@pytest.mark.parametrize("mode", MODE_ORDER)
def test_request_shape_matches_mode(mode):
    req = FieldChannel(SCHEMA).build_request(mode)
    assert req.mode == mode
    assert (req.response_format is not None) == (mode in ("structured_output", "json_mode"))
    assert bool(req.tools) == (mode == "tool_choice")
    assert req.tool_choice == ("required" if mode == "tool_choice" else None)


def test_field_channel_tool_mode_without_command_call_is_unparseable_not_silently_empty():
    """Модель не позвала тул `command`: текст пуст -> парсер обязан отказать (потеря видна, а не молчаливая)."""
    parsed = FieldChannel(SCHEMA).parse(ModeReply(content="просто текст"), "tool_choice", PARSE)
    assert isinstance(parsed, Invalid) and parsed.reason == "no_command"
    assert parsed.ask.key == "understand.invalid"
