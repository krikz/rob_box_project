"""#4152 (живой 28.10 15:12, dev-00f61b1): «Робот, расскажи длинную сказку про енота-механика…» → модель прислала
``perform kind=tale backing=calm``, и код заказал подложку (``[backing] track_id=…``). Решение Шифу: подложка по
умолчанию только у песни и рэпа; сказка, анекдот (``tale``) и стих (``poem``) — без неё, пока человек сам не назвал
музыку.

Здесь — мост режима команд под формой хода (как в бою: форму ставит ``turn.understand`` из текста человека):
``backing`` сказки без просьбы о музыке приходит исполнителю как ``none``, а лог ответа несёт ``fields_dropped``."""

from __future__ import annotations

import asyncio
from typing import Any, AsyncIterator, Iterable, List, Mapping, Optional

from rob_box_dialog import command as cm
from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE
from rob_box_dialog.session import Session
from rob_box_dialog.turn import _Turn, command_form
from rob_box_llm.provider import LLMChunk, LLMMessage, LLMResponse, LLMSettings, ProviderCapabilities, ToolCall
from rob_box_llm.providers.deepseek import DeepSeekProvider
from rob_box_voice.core import command_bridge as cb

DEEPSEEK_CAPS = DeepSeekProvider(api_key="test").capabilities
SEGMENTS = ["Жил-был енот-механик в старом гараже.", "Чинил он всё на свете, даже дождь."]
CAST = [{"name": "Енот", "gender": "male"}]
ROLES = ["", "Енот"]


class _Inner:
    """Провайдер-двойник: заготовленные ответы по очереди."""

    def __init__(self, replies: List[LLMResponse]) -> None:
        self.name, self.model, self._replies, self.requests = "deepseek", "deepseek-model", list(replies), []

    @property
    def capabilities(self) -> ProviderCapabilities:
        return DEEPSEEK_CAPS

    def capabilities_for(self, model: Optional[str]) -> ProviderCapabilities:
        return DEEPSEEK_CAPS

    async def complete(self, messages: Iterable[LLMMessage], *, tools: Iterable[Mapping[str, Any]] = (),
                       settings: Optional[LLMSettings] = None) -> LLMResponse:
        self.requests.append({"messages": list(messages), "tools": list(tools)})
        return self._replies.pop(0)

    async def stream(self, messages: Iterable[LLMMessage], *, tools: Iterable[Mapping[str, Any]] = (),
                     settings: Optional[LLMSettings] = None) -> AsyncIterator[LLMChunk]:  # pragma: no cover
        raise NotImplementedError

    async def aclose(self) -> None:
        return None


def _perform(kind: str, backing: str) -> LLMResponse:
    perform = {"kind": kind, "backing": backing, "segments": SEGMENTS}
    if kind in kn.PERFORM_ROLE_KINDS:
        perform.update(cast=CAST, role_per_segment=ROLES)
    return LLMResponse(tool_calls=(ToolCall(id="c1", name=kn.COMMAND_TOOL, arguments={"perform": perform}),))


def _answer(text: str, kind: str, backing: str):
    """Ответ моста на реплику ``text``: форма хода — из её текста (``command_form``, как в ``turn.understand``)."""
    form = command_form(_Turn("t1", 0, text, VOICE, deadline=None), Session("personality").snapshot)
    lines: List[str] = []
    inner = _Inner([_perform(kind, backing)])

    async def run() -> LLMResponse:
        with cm.turn_form(form):
            return await cb.CommandModeLLM(inner, log=lines.append).complete(
                [LLMMessage(role="user", content=text)], tools=[])

    out = asyncio.run(run())
    assert len(inner.requests) == 1, "лишнее поле - не нарушение: повтора моста нет"
    return out.tool_calls[0].arguments["perform"], lines


def test_the_live_case_of_28_10_tale_calm_without_a_music_request_reaches_the_executor_as_none() -> None:
    perform, lines = _answer("Робот, расскажи длинную сказку про енота-механика", "tale", "calm")
    assert perform["backing"] == "none" and perform["kind"] == "tale" and perform["segments"] == SEGMENTS
    assert any("fields_dropped=['perform.backing']" in line for line in lines)


def test_tale_with_a_music_request_in_the_same_reply_keeps_the_backing() -> None:
    perform, lines = _answer("Робот, расскажи сказку под спокойную музыку", "tale", "calm")
    assert perform["backing"] == "calm" and not any("fields_dropped" in line for line in lines)


def test_rap_keeps_the_beat_whatever_the_wording() -> None:
    perform, lines = _answer("Робот, зачитай что-нибудь про енота в стиле хип-хоп", "rap", "beat")
    assert perform["backing"] == "beat" and not any("fields_dropped" in line for line in lines)


def test_a_joke_and_a_counted_poem_are_without_backing_too() -> None:
    joke, _ = _answer("Робот, расскажи анекдот про енота", "tale", "beat")
    count, _ = _answer("Робот, посчитай вслух до десяти", "poem", "calm")
    assert joke["backing"] == "none" and count["backing"] == "none"
