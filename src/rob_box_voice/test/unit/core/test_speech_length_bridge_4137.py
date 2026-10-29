"""#4137 (решение Шифу 28.10, n907) на мосту режима команд: речь проверяем по ``SPEECH_ACCEPT_MAX_CHARS`` (350), а
просим о ``SPEECH_MAX_CHARS`` (280). Длиннее проверки - ход компактнее своим счётом (``SPEECH_COMPACT_TURNS``), даже
когда повтор ушёл на другое нарушение; и после хода длинно - речь ужимает код (``command.fit_speech``), не заглушка."""

from __future__ import annotations

import asyncio
from typing import Any, AsyncIterator, Iterable, List, Mapping, Optional

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_llm.provider import LLMChunk, LLMMessage, LLMResponse, LLMSettings, ProviderCapabilities, ToolCall
from rob_box_llm.providers.deepseek import DeepSeekProvider
from rob_box_voice.core import command_bridge as cb

DEEPSEEK_CAPS = DeepSeekProvider(api_key="test").capabilities
N907 = ("Свежих материалов за неделю поиск дал мало. Важное: Роскосмос по просьбе NASA законсервирует "
        "переходную камеру модуля «Звезда» из-за утечек воздуха — их ищут с две тысячи девятнадцатого года. "
        "Ещё на этой неделе к станции ушёл грузовой корабль, а китайский зонд прислал новые снимки Луны.")
OVER = N907 + (" Учёные снова обсуждали пилотируемый полёт к Марсу: старт перенесли ещё на год. А новый телескоп нашёл "
               "далёкую галактику.")
SHORT = "За неделю: «Звезду» законсервируют из-за утечек, к станции ушёл грузовик, зонд прислал снимки Луны."
QUERY_DATA = '{"query_results": [{"tool": "search_web", "status": "done", "reason": null, "data": {"results": []}}]}'


def _tool(name: str) -> Mapping[str, Any]:
    return {"type": "function", "function": {"name": name, "description": name,
                                             "parameters": {"type": "object", "properties": {}}}}


TOOLS = [_tool(n) for n in ("memory_save", "get_current_time", "play_sound")]


class _Inner:
    """Провайдер-двойник: заготовленные ответы по очереди, запросы — в ``requests``."""

    def __init__(self, replies: List[LLMResponse]) -> None:
        self.name, self.model, self._replies, self.requests = "deepseek", "deepseek-model", list(replies), []

    @property
    def capabilities(self) -> ProviderCapabilities:
        return DEEPSEEK_CAPS

    def capabilities_for(self, model: Optional[str]) -> ProviderCapabilities:
        return DEEPSEEK_CAPS

    async def complete(self, messages: Iterable[LLMMessage], *, tools: Iterable[Mapping[str, Any]] = (),
                       settings: Optional[LLMSettings] = None) -> LLMResponse:
        self.requests.append(list(messages))
        return self._replies.pop(0)

    async def stream(self, messages: Iterable[LLMMessage], *, tools: Iterable[Mapping[str, Any]] = (),
                     settings: Optional[LLMSettings] = None) -> AsyncIterator[LLMChunk]:  # pragma: no cover
        raise NotImplementedError

    async def aclose(self) -> None:
        return None


def _reply(**fields: Any) -> LLMResponse:
    return LLMResponse(tool_calls=(ToolCall(id="c1", name=kn.COMMAND_TOOL, arguments=fields),))


def _answer(*replies: LLMResponse) -> tuple:
    """Ответ моста на запрос после выполненного ``query`` (как во второй итерации хода n907), запросы и лог."""
    inner, log = _Inner(list(replies)), []
    msgs = [LLMMessage(role="system", content="ты РОББОКС"), LLMMessage(role="user", content="что нового в космосе?"),
            LLMMessage(role="user", content=QUERY_DATA)]
    out = asyncio.run(cb.CommandModeLLM(inner, log.append).complete(msgs, tools=TOOLS))
    return out, inner.requests, log


def test_n907_answer_of_287_is_said_as_is_without_a_second_call():
    assert kn.SPEECH_MAX_CHARS < len(N907) <= kn.SPEECH_ACCEPT_MAX_CHARS
    out, requests, log = _answer(_reply(speech=N907))
    assert out.content == N907 and len(requests) == 1
    assert not any(line.startswith(cb.COMPACT_LOG_MARK) for line in log)


def test_over_check_gets_a_compact_turn_with_the_asked_length_and_its_answer_is_said():
    out, requests, log = _answer(_reply(speech=OVER), _reply(speech=SHORT))
    assert out.content == SHORT and len(requests) == 2
    assert f"{cb.COMPACT_LOG_MARK} speech {len(OVER)} > {kn.SPEECH_ACCEPT_MAX_CHARS} — прошу компактнее" in log
    hint = requests[1][-1].content
    assert kn.VIOLATION_SPEECH_TOO_LONG in hint and f"не длиннее {kn.SPEECH_MAX_CHARS}" in hint and "получены" in hint
    assert requests[1][:3] == requests[0][:3]  # те же данные запроса в окне хода компактнее


def test_still_over_after_compact_turn_is_trimmed_by_code_to_whole_sentences():
    out, requests, log = _answer(_reply(speech=OVER), _reply(speech=OVER))
    assert out.content == N907 and len(requests) == 2  # три целых предложения, четвёртое не влезло
    assert out.tool_calls == ()
    assert any(line.startswith(cb.COMPACT_LOG_MARK) and f"ужата кодом до {len(N907)} символов" in line for line in log)


def test_voice_list_that_stays_long_is_cut_by_code_with_the_cut_mark_not_a_dangling_name():
    """Хвост ревью #4143: перечень голосов (#3883) длиннее проверки и после хода компактнее обрезался по слову
    («…Голос38») и звучал как полный. Обрыв посреди предложения теперь кончает пометка каталога."""
    voices = "У меня есть голоса " + ", ".join(f"Голос{n}" for n in range(60)) + "."
    mark = phrases.text(kn.SPEECH_CUT_PHRASE)
    out, requests, log = _answer(_reply(speech=voices), _reply(speech=voices))
    assert len(requests) == 2 and out.tool_calls == ()
    assert out.content.endswith(f" {mark}") and len(out.content) <= kn.SPEECH_ACCEPT_MAX_CHARS
    head = out.content[:-len(mark)].rstrip()
    assert voices.startswith(head) and not head.endswith(",")
    assert any(line.startswith(cb.COMPACT_LOG_MARK) and f"ужата кодом до {len(out.content)} символов" in line
               for line in log)


def test_compact_turn_has_its_own_count_when_the_retry_went_to_another_violation():
    """n907: повтор ушёл на ``tool_not_available`` - ход компактнее всё равно есть, а не заглушка."""
    wrong = _reply(actions=[{"tool": "no_such_tool", "args": {}}])
    out, requests, log = _answer(wrong, _reply(speech=OVER), _reply(speech=SHORT))
    assert out.content == SHORT and len(requests) == 3
    assert any("retry=1 violations=['tool_not_available'] → verdict=invalid:speech_too_long" in line for line in log)
    assert any(f"retry=2 violations=['{kn.VIOLATION_SPEECH_TOO_LONG}'] → verdict=speech" in line for line in log)


def test_overlong_answer_does_not_spend_the_general_retry():
    """Ответ длиннее проверки, а ход компактнее дал другое нарушение - общий повтор цел и исправляет его."""
    wrong = _reply(actions=[{"tool": "no_such_tool", "args": {}}])
    out, requests, _ = _answer(_reply(speech=OVER), wrong, _reply(speech=SHORT))
    assert out.content == SHORT and len(requests) == 3


def test_overlong_next_to_another_violation_is_a_plain_retry_not_a_compact_turn():
    mixed = _reply(speech=OVER, voice="z" * (kn.VOICE_MAX_CHARS + 1))
    out, requests, log = _answer(mixed, mixed)
    assert len(requests) == 2
    assert out.tool_calls[0].arguments == {kn.COMMAND_INVALID_FIELD: kn.VIOLATION_SPEECH_TOO_LONG}
    assert not any(line.startswith(cb.COMPACT_LOG_MARK) for line in log)


def test_ask_text_over_check_follows_the_same_path():
    ask = {"expects": "yes_no", "text": OVER}
    out, requests, _ = _answer(_reply(ask=ask), _reply(ask=ask))
    assert out.content == N907 and len(requests) == 2


def test_no_speech_at_all_stays_an_invalid_command_for_the_turn_stub():
    out, requests, log = _answer(_reply(speech=""), _reply(speech=""))
    assert len(requests) == 2 and out.tool_calls[0].arguments == {kn.COMMAND_INVALID_FIELD: kn.VIOLATION_EMPTY_COMMAND}
    assert not any(line.startswith(cb.COMPACT_LOG_MARK) for line in log)
