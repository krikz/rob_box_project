"""Мост режима команд в v1: LLM отвечает ОДНОЙ командой, v1-цикл тулов исполняет её как раньше (ADR-0150 §4.1, §4.5).

Временный модуль (#3696 ч.5, удаляется в PR-8 вместе с v1-циклом ``AgentCore``). Оборачивает ОДИН провайдер цепочки:

* **форма запроса — по возможностям адаптера**, а не по промпту: ``degrade.select_mode(capabilities)`` →
  ``tool_choice`` (DeepSeek: тул ``command`` + ``tool_choice="required"``) | ``tool_offered`` (MiniMax: тул ``command``
  без ``tool_choice`` — принуждение сервер не исполняет, ADR-0143, замер #3698 ч.7d) | ``json_mode`` /
  ``structured_output`` | ``json_in_text``; запрос строит ``speech_channel.FieldChannel`` (решение О4: речь — поле
  ``speech``), тула ``speak_text`` модель в этом режиме не видит — слот речи один;
* **ответ разбирает один парсер** ``rob_box_dialog.command.parse``: голая реплика (без действий, выступления и голоса)
  → текст речи (терминальный ход, авто-TTS ``dialogue_node``); остальное → один вызов ``command``, который v1-цикл
  отдаёт исполнителю команды ``rob_box_harness.core.tool_loop.command_turn`` (PR-7 ч.7c: действия по событию
  владельца, фраза кода, ``speech`` при действиях не звучит, ход терминален); ``passed`` → пустой ответ;
* **невалидная команда** (разбор + ``command.validate`` по срезу тулов запроса) возвращается модели в том же запросе:
  её вызов ``command`` + tool-результат с нарушениями (``command.explain``; в JSON-режимах — реплика пользователя),
  не больше ``knowledge.COMMAND_RETRIES_MAX`` раз на ответ (решение Шифу 21.10 по #3698, как deepseek-harness:
  ``invalid arguments`` с перечнем допустимого). Снова невалидна — исполнителю хода уходит вызов ``command`` с полем
  ``knowledge.COMMAND_INVALID_FIELD``: ход закрывается как провал без речи (рефлекс ``failed``). Пока идёт повтор,
  робот молчит: ответ ноде уходит один, после повтора.

Какие провайдеры отвечают командой — строка конфига ``llm_command_providers`` (:func:`wrapper_for`); остальные
остаются на ``speak_text``. ``"passed": false`` / ``args: ""`` MiniMax нормализует ``command.parse`` (PR-7 #3698).
Каждый ответ — строка лога :data:`COMMAND_LOG_MARK` (A14 по провайдерам).
"""

from __future__ import annotations

import json
import time
import uuid
from dataclasses import replace
from types import SimpleNamespace
from typing import Any, AsyncIterator, Awaitable, Callable, Iterable, List, Mapping, Optional, Sequence, Tuple, Union

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import NOT_RUN, Busy, Command, Invalid, Result, explain, is_empty, parse, to_args, validate
from rob_box_dialog.command import ToolCall as CommandCall
from rob_box_dialog.degrade import CommandMode, select_mode
from rob_box_dialog.speech_channel import FieldChannel, ModeRequest
from rob_box_harness.providers.reasoning import without_reasoning
from rob_box_llm.provider import (
    LLMChunk,
    LLMMessage,
    LLMProvider,
    LLMResponse,
    LLMSettings,
    ProviderCapabilities,
    ToolCall,
    ToolResult,
)

#: Маркер строки лога о каждом ответе в режиме команд (читает ``scripts/dialog`` для A14).
COMMAND_LOG_MARK = "🧾 [command]"
#: Префикс id вызова ``command``, который собрал мост (видно в истории и логах исполнителя).
CALL_ID_PREFIX = "cmd-"
#: Режимы, где схема команды идёт текстом в промпте (в остальных её несёт тул или ``response_format``).
_SCHEMA_IN_PROMPT: Tuple[CommandMode, ...] = ("json_mode", "json_in_text")
#: Поправка к правилам v1-промпта о транспорте речи: мастер-промпт учит ``speak_text`` и ``done`` (до PR-8).
V1_TRANSPORT_NOTE = (
    "Форма ответа в этом режиме важнее правил о speak_text и done выше: тула speak_text нет — всё, что человек должен "
    "услышать, клади в поле speech; сказку, стих, рэп, анекдот — в perform.segments (куплет или абзац на сегмент), "
    "голос персонажа — в perform.voice_per_segment; о результате действий не говори — его скажет код; done не пиши."
)

Log = Callable[[str], None]
Ask = Callable[[List[LLMMessage]], Awaitable[LLMResponse]]


def _no_log(_: str) -> None:
    return None


def mode_note(mode: CommandMode, request: ModeRequest, schema: Mapping[str, Any] | None = None) -> str:
    """Добавка к системному промпту: форма команды из ``knowledge`` + поправка v1 + схема для JSON-режимов."""
    parts = [kn.prompt_fragment(), V1_TRANSPORT_NOTE]
    if request.instruction:
        parts.append(request.instruction)
    if mode in _SCHEMA_IN_PROMPT:
        parts.append("JSON-схема команды: " + json.dumps(schema or kn.command_schema("json"), ensure_ascii=False))
    return "\n".join(parts)


def _with_note(messages: Sequence[LLMMessage], note: str) -> List[LLMMessage]:
    """Записать ``note`` в конец первого системного сообщения (блок ``system`` контекста), иначе — новым первым."""
    out = list(messages)
    if out and out[0].role == "system" and isinstance(out[0].content, str):
        out[0] = replace(out[0], content=f"{out[0].content}\n\n{note}")
        return out
    return [LLMMessage(role="system", content=note), *out]


def _tool_name(tool: Mapping[str, Any]) -> str:
    return str((tool.get("function") or {}).get("name", ""))


def reply_command(content: str, calls: Iterable[ToolCall]) -> Result:
    """Команда из ответа провайдера: вызов ``command`` (+ свои тулы как действия), иначе JSON в тексте (+ свои тулы),
    иначе одни свои тулы — команда из действий. Ничего из этого — :class:`Invalid` от парсера."""
    native = [{"name": c.name, "arguments": dict(c.arguments)} for c in calls]
    if any(c["name"] == kn.COMMAND_TOOL for c in native):
        return parse(native)
    extras = tuple(CommandCall(c["name"], dict(c["arguments"])) for c in native)
    from_text = parse(content or "")
    if isinstance(from_text, Command):
        return replace(from_text, actions=from_text.actions + extras)
    if extras and from_text.reason == "no_command":
        return Command(actions=extras)
    return from_text


def judge(response: LLMResponse, available: Iterable[str], busy: Busy = Busy(), agent: str = kn.AGENTS[0]) -> Result:
    """Команда ответа провайдера, проверенная по срезу тулов запроса и занятому сейчас (``busy`` — ACTIVE-сегмент
    выступления для ``edit_pending``, PR-7c). Оборванный JSON аргументов — ``truncated``; вызов ``command`` с пустыми
    аргументами — ``bad_json`` (адаптер отдаёт ``{}``, когда JSON не разобрался); команда, которой нечего исполнить и
    нечего сказать (не ``passed``), — ``empty_command``: тот же повтор с объяснением, а не «тихий ответ» для v1-ретрая
    #1217 мимо команды (ночной замер #3698 22.10, ход 4). ``agent`` — чья команда (телеграм: речь при действиях
    законна, ``say_aloud`` доступен — ``command.validate``)."""
    if response.truncated_tool_args:
        return Invalid("truncated", "аргументы обрезаны по max_tokens")
    if any(c.name == kn.COMMAND_TOOL and not c.arguments for c in response.tool_calls):
        return Invalid("bad_json", "аргументы command пусты или не JSON")
    result = reply_command(response.content, response.tool_calls)
    if not isinstance(result, Command):
        return result
    checked = validate(_without_speech_tool(result), available, SimpleNamespace(agent=agent), busy=busy)
    if isinstance(checked, Command) and is_empty(checked):
        return Invalid(kn.VIOLATION_EMPTY_COMMAND, "нет speech, ask, actions, perform и passed")
    return checked


def retry_messages(response: LLMResponse, invalid: Invalid) -> List[LLMMessage]:
    """Продолжение запроса после невалидной команды: ответ модели как был и нарушения словами — tool-результатом на
    её вызовы (``command`` — нарушения, прочие — «не исполнено»), без вызовов — репликой пользователя."""
    note = explain(invalid)
    said = LLMMessage(role="assistant", content=response.content or "", tool_calls=tuple(response.tool_calls))
    if not response.tool_calls:
        return [said, LLMMessage(role="user", content=note)]
    out = [said]
    for call in response.tool_calls:
        text = note if call.name == kn.COMMAND_TOOL else NOT_RUN
        out.append(LLMMessage(role="tool", content=text, tool_call_id=call.id,
                              tool_result=ToolResult(tool_call_id=call.id, content=text, is_error=True)))
    return out


def _spoken(cmd: Command) -> str:
    """Что прозвучит у команды без действий: речь, вопрос или сегменты выступления (в этом порядке)."""
    if cmd.speech:
        return cmd.speech
    if cmd.ask is not None and cmd.ask.text:
        return cmd.ask.text
    if cmd.perform is not None:
        return "\n".join(s for s in cmd.perform.segments if s)
    return ""


def _without_speech_tool(cmd: Command) -> Command:
    """Тул речи среди действий (модель по привычке v1 положила ``speak_text`` в ``actions``) — не действие: его текст
    и голос становятся речью, если поля ``speech`` нет."""
    spoken = [a for a in cmd.actions if a.tool in kn.SPEECH_CHANNEL_TOOLS]
    if not spoken:
        return cmd
    args = spoken[0].args
    actions = tuple(a for a in cmd.actions if a.tool not in kn.SPEECH_CHANNEL_TOOLS)
    if cmd.speech:
        return replace(cmd, actions=actions)
    return replace(cmd, actions=actions, speech=str(args.get("text") or "").strip() or None,
                   voice=str(args.get("voice") or "").strip() or cmd.voice)


def to_legacy(result: Result) -> Tuple[str, Tuple[ToolCall, ...], str]:
    """Команда → ``(текст речи, вызовы, вердикт для лога)`` в форме v1-цикла: голая реплика — текстом (авто-TTS ноды),
    команда с действиями, выступлением или голосом реплики — одним вызовом ``command`` исполнителю команды.
    Невалидная — тоже вызов ``command``, но с причиной в ``COMMAND_INVALID_FIELD``: речи нет, ход закроет провал.
    «Промолчать» (``passed``) — вызов ``command`` с одним полем ``passed``: пустой ответ без вызова потребитель не отличил бы
    от пустого (``Invalid(empty)``, #3859), а так исход доходит до хода как есть."""
    if isinstance(result, Invalid):
        call = ToolCall(id=f"{CALL_ID_PREFIX}{uuid.uuid4().hex[:12]}", name=kn.COMMAND_TOOL,
                        arguments={kn.COMMAND_INVALID_FIELD: result.reason})
        return "", (call,), f"invalid:{result.reason}"
    if result.passed:
        call = ToolCall(id=f"{CALL_ID_PREFIX}{uuid.uuid4().hex[:12]}", name=kn.COMMAND_TOOL,
                        arguments={"passed": result.passed})
        return "", (call,), f"passed:{result.passed}"
    cmd = _without_speech_tool(result)
    if cmd.actions or cmd.perform is not None or (cmd.voice and _spoken(cmd)):
        call = ToolCall(id=f"{CALL_ID_PREFIX}{uuid.uuid4().hex[:12]}", name=kn.COMMAND_TOOL, arguments=to_args(cmd))
        return "", (call,), "command"
    text = _spoken(cmd)
    return text, (), "speech" if text else "empty"


def log_line(provider: str, mode: CommandMode, result: Result, verdict: str) -> str:
    """Строка лога ответа: провайдер, режим, исход, действия команды, поправки разбора (слоты лица переложены в
    ``expression.face``, лишние поля отброшены — #3698) и речь, которая не прозвучит (при действиях ``speech`` не
    озвучивается, §4.1: она остаётся только здесь — текстового канала мимо TTS у v1 нет)."""
    cmd = result if isinstance(result, Command) else None
    actions = [a.tool for a in cmd.actions if a.tool not in kn.SPEECH_CHANNEL_TOOLS] if cmd is not None else []
    dropped = cmd.speech if actions and cmd is not None and cmd.speech else ""
    detail = getattr(result, "detail", "") if isinstance(result, Invalid) else ""
    fixes = (("fields_moved", cmd.moved), ("fields_dropped", cmd.dropped)) if cmd is not None else ()
    fixed = "".join(f" {name}={list(fields)}" for name, fields in fixes if fields)
    return (f"{COMMAND_LOG_MARK} provider={provider} mode={mode} verdict={verdict} "
            f"actions={actions} speech_dropped={int(bool(dropped))}" + fixed
            + (f" speech={dropped[:120]!r}" if dropped else "") + (f" detail={detail[:120]!r}" if detail else ""))


async def _collect(stream: AsyncIterator[LLMChunk]) -> LLMResponse:
    """Собрать стрим провайдера в один ответ (команду можно разобрать только целиком)."""
    parts: List[str] = []
    calls: List[ToolCall] = []
    finish: Optional[str] = None
    usage: Optional[Mapping[str, int]] = None
    truncated = False
    try:
        async for chunk in stream:
            parts.append(chunk.content_delta or "")
            if chunk.tool_call_delta is not None:
                calls.append(chunk.tool_call_delta)
            finish = chunk.finish_reason or finish
            usage = chunk.usage or usage
            truncated = truncated or chunk.truncated_tool_args
    finally:
        aclose = getattr(stream, "aclose", None)
        if aclose is not None:
            await aclose()
    return LLMResponse(content="".join(parts), tool_calls=tuple(calls), finish_reason=finish, usage=usage or {},
                       truncated_tool_args=truncated)


class CommandModeLLM(LLMProvider):
    """Провайдер цепочки, отвечающий командой; имя, возможности и здоровье — у обёрнутого (имя кэша то же)."""

    def __init__(self, inner: Any, log: Log = _no_log, busy: Callable[[], Busy] = Busy,
                 agent: str = kn.AGENTS[0]) -> None:
        self._inner = inner
        self._busy = busy
        self._agent = agent
        self.name = getattr(inner, "name", type(inner).__name__)
        self.mode: CommandMode = select_mode(inner.capabilities)
        self._log = log

    def __getattr__(self, item: str) -> Any:
        # Только для атрибутов, которых у обёртки нет (``model`` и т.п. для метрик ноды).
        inner = self.__dict__.get("_inner")
        if inner is None:
            raise AttributeError(item)
        return getattr(inner, item)

    @property
    def capabilities(self) -> ProviderCapabilities:
        return self._inner.capabilities

    def capabilities_for(self, model: str | None) -> ProviderCapabilities:
        return self._inner.capabilities_for(model)

    def shape(self, messages: Iterable[LLMMessage], tools: Iterable[Mapping[str, Any]],
              settings: LLMSettings | None) -> Tuple[List[LLMMessage], List[Mapping[str, Any]], LLMSettings]:
        """Запрос в режиме адаптера: тул/формат от ``FieldChannel``, свои тулы без тула речи, добавка к промпту."""
        tools = list(tools)
        schema = kn.with_tool_enum(kn.command_schema("json"), (_tool_name(t) for t in tools))
        request = FieldChannel(schema).build_request(self.mode)
        native = [t for t in tools if _tool_name(t) not in kn.SPEECH_CHANNEL_TOOLS]
        base = settings or LLMSettings()
        extra = dict(base.extra)
        if request.response_format:
            extra["response_format"] = dict(request.response_format)
        shaped = replace(base, tool_choice=request.tool_choice or base.tool_choice, extra=extra)
        return _with_note(list(messages), mode_note(self.mode, request, schema)), [*request.tools, *native], shaped

    async def answer(self, ask: Ask, msgs: List[LLMMessage], offered: Sequence[Mapping[str, Any]]) -> LLMResponse:
        """Ответ провайдера → ответ в форме v1 (речь текстом или вызовы тулов). Невалидная команда — повтор в том же
        запросе с нарушениями, не больше ``COMMAND_RETRIES_MAX`` раз; строка лога на каждый ответ и на повтор."""
        available = frozenset(_tool_name(t) for t in offered)
        response = without_reasoning(await ask(msgs))
        result = self._judged(response, available)
        for attempt in range(1, kn.COMMAND_RETRIES_MAX + 1):
            if not isinstance(result, Invalid):
                break
            started = time.monotonic()
            retry = without_reasoning(await ask([*msgs, *retry_messages(response, result)]))
            fixed = self._judged(retry, available)
            self._log(f"{COMMAND_LOG_MARK} retry={attempt} violations={[v.reason for v in result.all]} "
                      f"→ verdict={to_legacy(fixed)[2]} ({(time.monotonic() - started) * 1000:.0f} мс)")
            response, result = retry, fixed
        text, calls, _ = to_legacy(result)
        return LLMResponse(content=text, tool_calls=calls, finish_reason="tool_calls" if calls else "stop",
                           usage=response.usage, raw=response.raw)

    def _judged(self, response: LLMResponse, available: Iterable[str]) -> Result:
        result = judge(response, available, self._busy(), self._agent)
        self._log(log_line(self.name, self.mode, result, to_legacy(result)[2]))
        return result

    async def complete(self, messages: Iterable[LLMMessage], *, tools: Iterable[Mapping[str, Any]] = (),
                       settings: LLMSettings | None = None) -> LLMResponse:
        msgs, offered, shaped = self.shape(messages, tools, settings)
        return await self.answer(lambda m: self._inner.complete(m, tools=offered, settings=shaped), msgs, offered)

    async def stream(self, messages: Iterable[LLMMessage], *, tools: Iterable[Mapping[str, Any]] = (),
                     settings: LLMSettings | None = None) -> AsyncIterator[LLMChunk]:
        msgs, offered, shaped = self.shape(messages, tools, settings)
        out = await self.answer(lambda m: _collect(self._inner.stream(m, tools=offered, settings=shaped)),
                                msgs, offered)
        if out.content:
            yield LLMChunk(content_delta=out.content)
        for call in out.tool_calls:
            yield LLMChunk(tool_call_delta=call)
        yield LLMChunk(finish_reason=out.finish_reason, usage=out.usage or None)

    async def aclose(self) -> None:
        await self._inner.aclose()


def command_providers(value: Union[str, Iterable[str], None]) -> frozenset:
    """Имена провайдеров из конфига (строка через запятую или список), в нижнем регистре."""
    items = value.split(",") if isinstance(value, str) else (value or ())
    return frozenset(str(n).strip().lower() for n in items if str(n).strip())


def wrapper_for(value: Union[str, Iterable[str], None], log: Log = _no_log,
                busy: Callable[[], Busy] = Busy, agent: str = kn.AGENTS[0]) -> Callable[[Any], Any]:
    """Обёртка провайдера для ``AgentSpec.wrap_provider``: провайдеры из списка — в режим команд, прочие как есть;
    ``busy`` — что занято сейчас (выступление) для валидатора команды, ``agent`` — чьи команды проверяются."""
    names = command_providers(value)

    def wrap(provider: Any) -> Any:
        if str(getattr(provider, "name", "")).lower() not in names:
            return provider
        wrapped = CommandModeLLM(provider, log, busy, agent)
        log(f"{COMMAND_LOG_MARK} provider={wrapped.name} mode={wrapped.mode} (по capabilities адаптера)")
        return wrapped

    return wrap


__all__ = [
    "CALL_ID_PREFIX", "COMMAND_LOG_MARK", "CommandModeLLM", "V1_TRANSPORT_NOTE", "command_providers", "judge",
    "log_line", "mode_note", "reply_command", "retry_messages", "to_legacy", "wrapper_for",
]
