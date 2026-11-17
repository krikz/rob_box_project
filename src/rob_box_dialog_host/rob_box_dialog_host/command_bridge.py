"""Режим команд провайдера: LLM отвечает ОДНОЙ командой, ход v2 получает её ``command.Result`` (ADR-0150 §4.1, §4.5).

Оборачивает ОДИН провайдер цепочки (``AgentSpec.wrap_provider``, :func:`wrapper_for`) — каждый: v2 отвечает только
командой. Ответ обёртки — :class:`CommandResponse` с полем ``command``; формы v1 (текст речи или вызов ``command``
для цикла ``AgentCore``) больше нет (#4354 B2):

* **форма запроса — по возможностям адаптера**, а не по промпту: ``degrade.select_mode(capabilities)`` →
  ``tool_choice`` (DeepSeek: тул ``command`` + ``tool_choice="required"``) | ``tool_offered`` (MiniMax: тул ``command``
  без ``tool_choice`` — принуждение сервер не исполняет, ADR-0143, замер #3698 ч.7d) | ``json_mode`` /
  ``structured_output`` | ``json_in_text``; запрос строит ``speech_channel.FieldChannel`` (решение О4: речь — поле
  ``speech``), тула ``speak_text`` модель в этом режиме не видит — слот речи один;
* **ответ разбирает один парсер** ``rob_box_dialog.command.parse``; что из команды получает ход — :func:`for_turn`
  (голая реплика — одной речью, ``passed`` — одним исходом);
* **невалидная команда** (разбор + ``command.validate`` по срезу тулов запроса) возвращается модели в том же запросе:
  её вызов ``command`` + tool-результат с нарушениями (``command.explain``; в JSON-режимах — реплика пользователя),
  не больше ``knowledge.COMMAND_RETRIES_MAX`` раз на ответ (решение Шифу 21.10 по #3698, как deepseek-harness:
  ``invalid arguments`` с перечнем допустимого). Снова невалидна — ход получает ``Invalid`` и закрывает его кодом
  (допустимые действия ``Invalid.salvage`` исполняются, итог говорит код). Пока идёт повтор, робот молчит: ответ
  ходу уходит один, после повтора;
* **речь длиннее, чем звучит** (#4137, решение Шифу 28.10) — единственное нарушение ``VIOLATION_SPEECH_TOO_LONG``
  (``command.overlong_only``): тот же повтор, но своим счётом ``knowledge.SPEECH_COMPACT_TURNS`` — ход компактнее
  есть, даже когда повтор потрачен на другое; и после него длинно — речь ужимает код (``command.fit_speech``), строка
  лога :data:`COMPACT_LOG_MARK`.

``"passed": false`` / ``args: ""`` MiniMax нормализует ``command.parse`` (PR-7 #3698). Каждый ответ — строка лога
:data:`COMMAND_LOG_MARK` (A14 по провайдерам).
"""

from __future__ import annotations

import json
import time
import uuid
from dataclasses import dataclass, replace
from types import SimpleNamespace
from typing import Any, AsyncIterator, Awaitable, Callable, Iterable, List, Mapping, Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import (NOT_RUN, Busy, Command, Expression, Invalid, Result, explain, fit_speech,
                                    form_schema, is_empty, is_query_data, loud_actions, overlong_only, parse, to_args,
                                    validate, words_only)
from rob_box_dialog.command import ToolCall as CommandCall
from rob_box_dialog.degrade import CommandMode, select_mode
from rob_box_dialog.speech_channel import FieldChannel, ModeRequest
from rob_box_dialog.tools_view import widen_for
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
#: Маркер строк лога о речи длиннее, чем звучит (#4137): ход компактнее и ужатие кодом.
COMPACT_LOG_MARK = "✂️ [command]"
#: Префикс id вызова ``command`` модели в истории запроса (:func:`model_call`, продолжение данными ``query``).
CALL_ID_PREFIX = "cmd-"
#: Ответ модели пуст (ни речи, ни команды) — невалиден (A14: пустой ответ по провайдеру).
EMPTY = "empty"
#: Режимы, где схема команды идёт текстом в промпте (в остальных её несёт тул или ``response_format``).
_SCHEMA_IN_PROMPT: Tuple[CommandMode, ...] = ("json_mode", "json_in_text")

Log = Callable[[str], None]
Ask = Callable[[List[LLMMessage]], Awaitable[LLMResponse]]


def _no_log(_: str) -> None:
    return None


@dataclass(frozen=True)
class CommandResponse(LLMResponse):
    """Ответ провайдера в режиме команд: ``command`` — команда хода (:func:`for_turn`) после разбора, проверки и
    повторов. ``content``/``tool_calls`` пусты: текста и вызовов для цикла тулов нет, ход читает ``command``;
    ``usage``/``raw``/``finish_reason`` — последнего ответа модели."""

    command: Optional[Result] = None


class _Widen(Exception):
    """Модель позвала тул, который есть у агента, но вне среза хода: запрос повторяется с группой этого тула (#3960)."""

    def __init__(self, names: Tuple[str, ...]) -> None:
        super().__init__(",".join(names))
        self.names = names


def _schemas(names: Iterable[str]) -> List[Mapping[str, Any]]:
    """OpenAI-схемы тулов каталога (как их строит порт хода)."""
    from rob_box_core.tool_catalog import get_tool  # noqa: PLC0415

    return [{"type": "function", "function": {"name": t.name, "description": t.description,
                                              "parameters": dict(t.parameters)}} for t in map(get_tool, names)]


def mode_note(mode: CommandMode, request: ModeRequest, schema: Mapping[str, Any] | None = None) -> str:
    """Добавка к системному промпту: форма команды из ``knowledge`` (строки только о полях схемы хода, #4021) + схема
    для JSON-режимов. Промпт Личности о транспорте речи не говорит — форму ответа несёт только эта добавка (#4354 B1)."""
    parts = [kn.prompt_fragment(schema)]
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


def judge(response: LLMResponse, available: Iterable[str], busy: Busy = Busy(), agent: str = kn.AGENTS[0],
          fit: bool = False) -> Result:
    """Команда ответа провайдера, проверенная по срезу тулов запроса и занятому сейчас (``busy`` — ACTIVE-сегмент
    выступления для ``edit_pending``, PR-7c). Оборванный JSON аргументов — ``truncated``; вызов ``command`` с пустыми
    аргументами — ``bad_json`` (адаптер отдаёт ``{}``, когда JSON не разобрался); команда, которой нечего исполнить и
    нечего сказать (не ``passed``), — ``empty_command``: тот же повтор с объяснением, а не «тихий ответ» для v1-ретрая
    #1217 мимо команды (ночной замер #3698 22.10, ход 4). ``agent`` — чья команда (телеграм: речь при действиях
    законна, ``say_aloud`` доступен — ``command.validate``). ``fit`` — речь ужимает код до проверки
    (``command.fit_speech``, последний запас #4137)."""
    if response.truncated_tool_args:
        return Invalid("truncated", "аргументы обрезаны по max_tokens")
    if any(c.name == kn.COMMAND_TOOL and not c.arguments for c in response.tool_calls):
        return Invalid("bad_json", "аргументы command пусты или не JSON")
    result = reply_command(response.content, response.tool_calls)
    if not isinstance(result, Command):
        return result
    cmd = _without_speech_tool(result)
    checked = validate(fit_speech(cmd) if fit else cmd, available, SimpleNamespace(agent=agent), busy=busy)
    if isinstance(checked, Command) and is_empty(checked):
        return Invalid(kn.VIOLATION_EMPTY_COMMAND, "нет speech, ask, actions, perform и passed")
    return checked


def query_done(messages: Iterable[LLMMessage]) -> bool:
    """В запросе есть данные ``query`` этого хода (их вернул ``turn.py`` после выполненного запроса, #3883)."""
    return any(m.role in ("tool", "user") and is_query_data(m.content or "") for m in messages)


def retry_messages(response: LLMResponse, invalid: Invalid, after_query: bool = False) -> List[LLMMessage]:
    """Продолжение запроса после невалидной команды: ответ модели как был и нарушения словами — tool-результатом на
    её вызовы (``command`` — нарушения, прочие — «не исполнено»), без вызовов — репликой пользователя. ``after_query`` —
    запрос хода уже выполнен: подсказка честная, «данные получены» (#3883)."""
    note = explain(invalid, after_query)
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


def _acted(cmd: Command, text: str) -> bool:
    """У команды есть что исполнить: действия, выступление или голос реплики."""
    return bool(cmd.actions) or cmd.perform is not None or bool(cmd.voice and text)


def _overlong(invalid: Invalid) -> str:
    """Что длиннее, чем звучит, — подробности нарушений ``VIOLATION_SPEECH_TOO_LONG`` («speech 400 > 350»)."""
    return "; ".join(v.detail for v in invalid.all)


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


def _keep_salvage(fixed: Result, invalid: Invalid) -> Result:
    """Повтор снова невалиден и исполнить в нём нечего (n903 марафона-9 04.11: на ``speech_with_actions`` модель
    прислала те же действия без речи, но JSON с лишней ``}`` — ``bad_json``), а у отвергнутой команды допустимые
    действия были — ход исполняет их (``Invalid.salvage``), речь отвергнутой отброшена, итог скажет код, а не «Не
    понял». Повтор со своими допустимыми действиями или валидный — как есть."""
    if isinstance(fixed, Invalid) and not fixed.salvage and invalid.salvage:
        return replace(fixed, salvage=invalid.salvage)
    return fixed


def for_turn(result: Result) -> Result:
    """Что из ответа модели получает ход. Невалидная — как есть (причина и ``salvage``: ход закроет её кодом).
    «Промолчать» — один исход ``passed``. Команда с действиями, выступлением, голосом реплики или лицом
    (``expression``: #3974 лицо без слов, #4225 лицо при речи) — целиком. Иначе голая реплика — одной речью
    (``speech``, текст ``ask`` или сегменты выступления: :func:`_spoken`); ни слова — ``Invalid(EMPTY)``.

    Так ход получал команду и до #4354 B2 (через текст речи и вызов ``command`` формы v1): вопрос ``ask`` без
    действий звучит репликой и ``pending_ask`` хода не ставит. Включить его — смена поведения, отдельной карточкой."""
    if isinstance(result, Invalid):
        return result
    if result.passed:
        return Command(passed=result.passed)
    cmd = _without_speech_tool(result)
    text = _spoken(cmd)
    if _acted(cmd, text) or cmd.expression != Expression():
        return cmd
    return Command(speech=text) if text else Invalid(EMPTY, "ни речи, ни команды")


def verdict(result: Result) -> str:
    """Вердикт ответа для строки лога :data:`COMMAND_LOG_MARK`: ``invalid:<причина>``, ``passed:<почему>``,
    ``speech`` (одни слова, и с лицом), ``command`` (есть что исполнить) или ``empty``."""
    if isinstance(result, Invalid):
        return f"invalid:{result.reason}"
    if result.passed:
        return f"passed:{result.passed}"
    cmd = _without_speech_tool(result)
    text = _spoken(cmd)
    acted = _acted(cmd, text)
    if acted or cmd.expression != Expression():
        return "speech" if text and not acted else "command"
    return "speech" if text else "empty"


def model_call(result: Result) -> Optional[ToolCall]:
    """Вызов ``command`` модели, как его видит история запроса (продолжение данными ``query``, ``dialog_v2_ports``):
    аргументы команды хода (``command.to_args``); у невалидной — только её допустимые действия (``salvage``, #4050);
    у ``passed`` — одно это поле. Голой реплике и пустому ответу вызова нет — данные уходят репликой пользователя."""
    if isinstance(result, Invalid):
        if result.reason == EMPTY:
            return None
        args: Mapping[str, Any] = ({"actions": [{"tool": a.tool, "args": dict(a.args)} for a in result.salvage]}
                                   if result.salvage else {})
    elif result.passed:
        args = {"passed": result.passed}
    elif _acted(result, _spoken(result)) or result.expression != Expression():
        args = to_args(result)
    else:
        return None
    return ToolCall(id=f"{CALL_ID_PREFIX}{uuid.uuid4().hex[:12]}", name=kn.COMMAND_TOOL, arguments=dict(args))


def log_line(provider: str, mode: CommandMode, result: Result, verdict: str) -> str:
    """Строка лога ответа: провайдер, режим, исход, действия команды, поправки разбора (слоты лица переложены в
    ``expression.face``, лишние поля отброшены — #3698) и речь, которая не прозвучит (при действиях ``speech`` не
    озвучивается, §4.1: она остаётся только здесь — текстового канала мимо TTS у v1 нет)."""
    cmd = result if isinstance(result, Command) else None
    actions = [a.tool for a in cmd.actions if a.tool not in kn.SPEECH_CHANNEL_TOOLS] if cmd is not None else []
    # отброшена речь только при громком действии: рядом с тихим (``QUIET_WITH_SPEECH_TOOLS``) она звучит (#4320)
    dropped = cmd.speech if cmd is not None and cmd.speech and loud_actions(cmd) and actions else ""
    detail = getattr(result, "detail", "") if isinstance(result, Invalid) else ""
    fixes = (("fields_moved", cmd.moved), ("fields_dropped", cmd.dropped)) if cmd is not None else ()
    fixed = "".join(f" {name}={list(fields)}" for name, fields in fixes if fields)
    return (f"{COMMAND_LOG_MARK} provider={provider} mode={mode} verdict={verdict} "
            f"actions={actions} speech_dropped={int(bool(dropped))}" + fixed
            + (f" speech={dropped[:120]!r}" if dropped else "") + (f" detail={detail[:120]!r}" if detail else ""))


class CommandModeLLM(LLMProvider):
    """Провайдер цепочки, отвечающий командой; имя, возможности и здоровье — у обёрнутого (имя кэша то же)."""

    def __init__(self, inner: Any, log: Log = _no_log, busy: Callable[[], Busy] = Busy,
                 agent: str = kn.AGENTS[0], owners: Optional[Sequence[str]] = None) -> None:
        self._inner = inner
        self._busy = busy
        self._agent = agent
        self._owners = None if owners is None else tuple(owners)
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
        """Запрос в режиме адаптера: тул/формат от ``FieldChannel``, свои тулы без тула речи, добавка к промпту. Схема —
        в форме команды хода (``command.form_schema``, #4021: на адресованную реплику нет ``passed``, без просьбы о
        голосе нет ``voice``)."""
        tools = list(tools)
        offered = kn.with_tool_enum(kn.command_schema("json"), (_tool_name(t) for t in tools))
        schema = form_schema(kn.with_voices(offered, self._busy().voices))
        request = FieldChannel(schema).build_request(self.mode)
        native = [t for t in tools if _tool_name(t) not in kn.SPEECH_CHANNEL_TOOLS]
        base = settings or LLMSettings()
        extra = dict(base.extra)
        if request.response_format:
            extra["response_format"] = dict(request.response_format)
        shaped = replace(base, tool_choice=request.tool_choice or base.tool_choice, extra=extra)
        return _with_note(list(messages), mode_note(self.mode, request, schema)), [*request.tools, *native], shaped

    async def answer(self, ask: Ask, msgs: List[LLMMessage], offered: Sequence[Mapping[str, Any]],
                     widen: bool = False) -> LLMResponse:
        """Ответ провайдера → :class:`CommandResponse` с командой хода. Невалидная команда — повтор в том же
        запросе с нарушениями, не больше ``COMMAND_RETRIES_MAX`` раз; речь длиннее, чем звучит (одно это нарушение,
        ``command.overlong_only``) — ход компактнее своим счётом ``SPEECH_COMPACT_TURNS``, а и после него длинно —
        речь ужимает код (:meth:`_fitted`, #4137). Ответ без слов на последней итерации (``command.words_only``, #4157)
        уходит ходу сразу: повторы за словами даёт ход, он знает свой потолок. Строка лога на каждый ответ и на
        повтор. ``widen`` - первый ответ с тулом вне среза (``tool_not_available``) не повторяется как есть:
        :class:`_Widen` просит запрос с группой этого тула (#3960)."""
        available = frozenset(_tool_name(t) for t in offered)
        response = without_reasoning(await ask(msgs))
        result = self._judged(response, available)
        if widen and isinstance(result, Invalid):
            names = widen_for([v.detail for v in result.all if v.reason == "tool_not_available"], self._agent,
                              sorted(available), self._owners)
            if names:
                raise _Widen(names)
        left = {False: kn.COMMAND_RETRIES_MAX, True: kn.SPEECH_COMPACT_TURNS}  # счёт повторов: общий и на длину
        attempt = 0
        while isinstance(result, Invalid) and not words_only(result) and left[overlong_only(result)] > 0:
            left[overlong_only(result)] -= 1
            attempt += 1
            response, result = await self._retried(ask, msgs, available, response, result, attempt)
        if isinstance(result, Invalid) and overlong_only(result):
            result = self._fitted(response, available, result)
        return CommandResponse(finish_reason=response.finish_reason, usage=response.usage, raw=response.raw,
                               command=for_turn(result))

    async def _retried(self, ask: Ask, msgs: List[LLMMessage], available: Iterable[str], response: LLMResponse,
                       invalid: Invalid, attempt: int) -> Tuple[LLMResponse, Result]:
        """Повтор в том же запросе: ответ модели и нарушения словами (:func:`retry_messages`). Речь длиннее, чем
        звучит, — это ход компактнее: строка :data:`COMPACT_LOG_MARK` до него."""
        if overlong_only(invalid):
            self._log(f"{COMPACT_LOG_MARK} {_overlong(invalid)} — прошу компактнее")
        started = time.monotonic()
        retry = without_reasoning(await ask([*msgs, *retry_messages(response, invalid, query_done(msgs))]))
        fixed = _keep_salvage(self._judged(retry, available), invalid)
        self._log(f"{COMMAND_LOG_MARK} retry={attempt} violations={[v.reason for v in invalid.all]} "
                  f"→ verdict={verdict(fixed)} ({(time.monotonic() - started) * 1000:.0f} мс)"
                  + (f" salvage={[a.tool for a in fixed.salvage]}" if isinstance(fixed, Invalid) and fixed.salvage
                     else ""))
        return retry, fixed

    def _fitted(self, response: LLMResponse, available: Iterable[str], invalid: Invalid) -> Result:
        """Последний запас (#4137): и после хода компактнее речь длиннее, чем звучит, — та же команда с речью, ужатой
        кодом (``command.fit_speech``: целые предложения с начала), а не заглушка. Строку 🧾 этот ответ уже дал."""
        fitted = judge(response, available, self._busy(), self._agent, fit=True)
        said = len(_spoken(fitted)) if isinstance(fitted, Command) else 0
        self._log(f"{COMPACT_LOG_MARK} {_overlong(invalid)} и после хода компактнее → ужата кодом до {said} символов, "
                  f"verdict={verdict(fitted)}")
        return fitted

    def _judged(self, response: LLMResponse, available: Iterable[str]) -> Result:
        result = judge(response, available, self._busy(), self._agent)
        self._log(log_line(self.name, self.mode, result, verdict(result)))
        return result

    async def complete(self, messages: Iterable[LLMMessage], *, tools: Iterable[Mapping[str, Any]] = (),
                       settings: LLMSettings | None = None) -> LLMResponse:
        messages, tools, widen = list(messages), list(tools), True
        while True:
            msgs, offered, shaped = self.shape(messages, tools, settings)
            try:
                return await self.answer(lambda m: self._inner.complete(m, tools=offered, settings=shaped), msgs,
                                         offered, widen)
            except _Widen as wide:
                tools, widen = [*tools, *_schemas(wide.names)], False
                self._log(f"{COMMAND_LOG_MARK} tool_not_available → в запрос добавлены тулы {list(wide.names)}")

    async def stream(self, messages: Iterable[LLMMessage], *, tools: Iterable[Mapping[str, Any]] = (),
                     settings: LLMSettings | None = None) -> AsyncIterator[LLMChunk]:
        """Потокового ответа в режиме команд нет: команду разбирают только целиком, ход зовёт :meth:`complete`."""
        raise NotImplementedError("режим команд отвечает целиком: зовите complete()")
        yield LLMChunk()  # pragma: no cover - делает метод асинхронным генератором, как у LLMProvider

    async def aclose(self) -> None:
        await self._inner.aclose()


def wrapper_for(log: Log = _no_log, busy: Callable[[], Busy] = Busy,
                agent: str = kn.AGENTS[0], owners: Optional[Sequence[str]] = None) -> Callable[[Any], Any]:
    """Обёртка провайдера для ``AgentSpec.wrap_provider``: каждый провайдер цепочки — в режим команд (v2 отвечает
    только командой); ``busy`` — что занято сейчас (выступление) для валидатора команды, ``agent`` — чьи команды
    проверяются, ``owners`` — владельцы тулов продукта (расширение среза не открывает тул чужого владельца)."""

    def wrap(provider: Any) -> Any:
        wrapped = CommandModeLLM(provider, log, busy, agent, owners)
        log(f"{COMMAND_LOG_MARK} provider={wrapped.name} mode={wrapped.mode} (по capabilities адаптера)")
        return wrapped

    return wrap


__all__ = [
    "CALL_ID_PREFIX", "COMMAND_LOG_MARK", "COMPACT_LOG_MARK", "CommandModeLLM", "CommandResponse", "EMPTY",
    "for_turn", "judge", "log_line", "mode_note", "model_call", "query_done", "reply_command", "retry_messages",
    "verdict", "wrapper_for",
]
