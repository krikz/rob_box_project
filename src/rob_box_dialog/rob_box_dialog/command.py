"""Команда LLM — узкий слот ответа хода (ADR-0150 §4.1; ADR-0148 «код решает, LLM говорит»).

LLM на каждом ходе отдаёт ОДНУ команду: речь, вопрос, до трёх действий, выступление, экспрессия. :func:`parse`
собирает её из ``tool_calls``, JSON или свободного текста с JSON-объектом; :func:`validate` проверяет по таблицам
:mod:`.knowledge` и JSON-схемам каталога тулов. Невалидно — :class:`Invalid` со списком нарушений (:class:`Violation`)
без вопроса человеку: ход закрывает провал (рефлекс ``failed``). :func:`explain` — нарушения словами для модели: мост режима команд возвращает их
tool-результатом, один раз на ответ (мост, решение Шифу 21.10 по #3698; ADR-0148 п.2).

:class:`Form` — форма команды ХОДА (#4021): какие поля модели можно на эту реплику. Ход ставит её на время вызова LLM
(:func:`turn_form`), мост строит по ней схему (:func:`form_schema`), :func:`validate` по ней же судит ответ — одно
решение кода в одном месте, не правило в промпте.
"""

from __future__ import annotations

import contextvars
import json
import unicodedata
from contextlib import contextmanager
from dataclasses import dataclass, field, replace
from types import MappingProxyType
from typing import Any, Callable, Collection, Dict, FrozenSet, Iterator, List, Mapping, Optional, Sequence, Tuple, Union

from rob_box_core.tool_catalog import get_tool
from rob_box_dialog import knowledge as kn
from rob_box_dialog.face_scene import FIELDS as FACE_FIELDS, FaceScene, problem as face_problem

#: Поля команды и экспрессии — из той же схемы, что видит LLM (:func:`knowledge.command_schema`).
_COMMAND_FIELDS: Tuple[str, ...] = tuple(kn.command_schema("json")["properties"])
_EXPRESSION_FIELDS: Tuple[str, ...] = tuple(kn.command_schema("json")["properties"]["expression"]["properties"])


@dataclass(frozen=True)
class Expression:
    """Экспрессия хода: эмоция, жест, earcon, сцена лица — enum из :mod:`.knowledge`; ``duration_s`` — сколько держать
    лицо команды, секунды (#4083; к пределам показа его приводит ``expression.show_seconds``), ``None`` — не задано."""

    emotion: str = "neutral"
    gesture: Optional[str] = None
    earcon: Optional[str] = None
    #: TODO(PR-8, ADR-0150 §5.5.9): исполнитель v2 шлёт ``face.to_dict()`` в ``ExpressionRequest.face_json``; проверка
    #: общая с v1 (``play_animation(face=...)``) - :func:`rob_box_dialog.face_scene.validate`.
    face: Optional[FaceScene] = None
    duration_s: Optional[float] = None


@dataclass(frozen=True)
class ToolCall:
    """Вызов тула из ``available_tools``; аргументы проверяются по JSON-схеме каталога."""

    tool: str
    args: Mapping[str, Any] = field(default_factory=lambda: MappingProxyType({}))


@dataclass(frozen=True)
class Ask:
    """Вопрос с ожиданием ответа: ``text`` — слова LLM, ``key`` — шаблон ``phrases/ru.yaml`` (вопрос кода)."""

    expects: str = "free"
    text: Optional[str] = None
    key: Optional[str] = None


@dataclass(frozen=True)
class Performance:
    """Выступление (§5.6.3): сегменты — содержимое, а не отчёт. ``from_segment`` — с какого править (edit_pending);
    его выбирает код (хост выступления: первый ещё не отданный на звучание, #3700), LLM его не задаёт — разбор
    отбрасывает это поле из ответа модели.

    ``voice_per_segment`` — голос каждого сегмента (сказка разными голосами, #3269); ``""`` — текущий голос робота.
    Голос робота он не меняет: это ``set_voice`` (только по явной просьбе, #3744)."""

    kind: str
    segments: Tuple[str, ...]
    backing: str = "none"
    expression_per_segment: Optional[Tuple[Expression, ...]] = None
    from_segment: int = 0
    voice_per_segment: Optional[Tuple[str, ...]] = None


@dataclass(frozen=True)
class Command:
    """Одна команда хода. ``dropped`` — поля, которые разбор или валидатор отбросили, не отвергая команду (сцена
    лица, лишние поля экспрессии); ``moved`` — поля, которые разбор переложил на их место: слоты лица из
    ``expression`` в ``expression.face`` (§5.5.9, #3698: DeepSeek кладёт их на уровень выше) и ``edit_pending`` из
    ``perform`` в команду (#3700: DeepSeek кладёт флаг правки внутрь выступления).

    ``voice`` — голос реплики ``speech``/``ask`` (не голос робота: его меняет ``set_voice``)."""

    speech: Optional[str] = None
    voice: Optional[str] = None
    ask: Optional[Ask] = None
    actions: Tuple[ToolCall, ...] = ()
    perform: Optional[Performance] = None
    when: str = "after_current"
    edit_pending: bool = False
    expression: Expression = Expression()
    passed: Optional[str] = None
    dropped: Tuple[str, ...] = ()
    moved: Tuple[str, ...] = ()


@dataclass(frozen=True)
class Violation:
    """Одно нарушение: машинная причина, где именно, и что допустимо (перечень из :mod:`.knowledge` или схемы)."""

    reason: str
    detail: str = ""
    allowed: Tuple[str, ...] = ()


@dataclass(frozen=True)
class Invalid:
    """Невалидный ответ LLM: машинная причина (первого нарушения), подробность для лога и все найденные нарушения
    (пусто — одно, ``reason``/``detail``). Человеку ничего не говорится: ход закрывается как ``failed``."""

    reason: str
    detail: str = ""
    violations: Tuple[Violation, ...] = ()
    #: Действия отвергнутой команды, допустимые каждое само по себе (не ``query``): после повтора моста ход
    #: исполняет их и говорит итог кодом, а не молчит (#3953, решение Шифу 23.10).
    salvage: Tuple[ToolCall, ...] = ()

    @property
    def all(self) -> Tuple[Violation, ...]:
        return self.violations or (Violation(self.reason, self.detail),)


@dataclass(frozen=True)
class Busy:
    """Что занято сейчас — классы действий и ACTIVE-сегмент выступления (заполняет планировщик, PR-7b) - и голоса
    звучащего провайдера TTS (#3959; пусто - каталог неизвестен, голос сегмента тогда не сверяется)."""

    classes: FrozenSet[str] = frozenset()
    active_segment: Optional[int] = None
    voices: Tuple[str, ...] = ()


Result = Union[Command, Invalid]


@dataclass(frozen=True)
class Form:
    """Форма команды хода (#4021, ADR-0148 п.2: выбор сужает структура, а не правило в промпте).

    ``passed`` — модели можно «промолчать»: только при подхвате бэклога. Адресованность реплики решил код
    (``address.py``), и на адресованную реплику молчанием не ответить (решение Шифу 22.10; n813 23.10: «Робот, тишина?»
    → ``passed`` → «Хм, я задумался…»). ``voice`` — модели можно задать голос реплики (``Command.voice``): человек в
    этой реплике просил о голосе (``tools_view.voice_asked``; n109 23.10: на вопрос про звуки DeepSeek сам включил
    female-shaonv). По умолчанию всё открыто — так зовут валидатор вне хода (режим команд v1, тесты)."""

    passed: bool = True
    voice: bool = True

    def closed(self) -> Tuple[str, ...]:
        """Поля команды, которых в этом ходе нет (порядок ``knowledge.FORM_FIELDS``)."""
        return tuple(name for name in kn.FORM_FIELDS if not getattr(self, name))


#: Форма хода, в котором идёт вызов LLM: ставит ход (:func:`turn_form`), читают мост (схема) и :func:`validate`. Вызов
#: модели идёт в задаче хода (``Ports.ask`` → цепочка → мост), контекст задачи у хода Личности и чата Telegram свой.
_FORM: "contextvars.ContextVar[Form]" = contextvars.ContextVar("rob_box_dialog_command_form", default=Form())


def current_form() -> Form:
    """Форма команды хода в работе; вне хода — всё открыто (:class:`Form` по умолчанию)."""
    return _FORM.get()


@contextmanager
def turn_form(form: Form) -> Iterator[Form]:
    """Форма команды на время вызовов LLM хода (``TurnEngine._understand``)."""
    token = _FORM.set(form)
    try:
        yield form
    finally:
        _FORM.reset(token)


def form_schema(schema: Mapping[str, Any], form: Optional[Form] = None) -> Dict[str, Any]:
    """Схема команды без полей, закрытых формой хода (``None`` — текущей): модель их не видит
    (``knowledge.without_fields``)."""
    return kn.without_fields(schema, (form or current_form()).closed())


@dataclass
class _Repairs:
    """Что разбор поправил, не отвергая команду: переложенные слоты лица и отброшенные поля экспрессии."""

    moved: List[str] = field(default_factory=list)
    dropped: List[str] = field(default_factory=list)


class _Bad(Exception):
    """Внутренний сигнал разбора/проверки: наружу не выходит — превращается в :class:`Invalid`."""

    def __init__(self, reason: str, detail: str = "", allowed: Collection[str] = ()) -> None:
        super().__init__(reason)
        self.reason = reason
        self.detail = detail
        self.allowed = tuple(allowed)

    def violation(self) -> Violation:
        return Violation(self.reason, self.detail, self.allowed)

    def invalid(self) -> Invalid:
        return Invalid(self.reason, self.detail, violations=(self.violation(),))


# ---------------------------------------------------------------------------------------------------------------
# parse
# ---------------------------------------------------------------------------------------------------------------


def parse(raw: Any) -> Result:
    """Команда из ответа LLM: список ``tool_calls`` (тул ``command`` + узкие тулы), объект JSON (dict или строка)
    или свободный текст — из текста берётся первый JSON-объект, остальное отбрасывается и не озвучивается."""
    try:
        if isinstance(raw, Mapping):
            return _command(raw)
        if isinstance(raw, str):
            return _command(_first_object(raw))
        if isinstance(raw, Sequence):
            return _from_tool_calls(raw)
        raise _Bad("bad_type", type(raw).__name__)
    except _Bad as bad:
        return bad.invalid()


def _first_object(text: str) -> Mapping[str, Any]:
    decoder = json.JSONDecoder()
    start = text.find("{")
    while start != -1:
        try:
            value, _ = decoder.raw_decode(text, start)
        except ValueError:
            value = None
        if isinstance(value, dict):
            return value
        start = text.find("{", start + 1)
    raise _Bad("no_command", "в тексте нет JSON-объекта")


def _call_parts(call: Any) -> Tuple[str, Any]:
    if not isinstance(call, Mapping):
        raise _Bad("bad_tool_call", "вызов — не объект")
    body = call.get("function") if isinstance(call.get("function"), Mapping) else call
    name, args = body.get("name"), body.get("arguments", body.get("args", {}))
    if isinstance(args, str):
        try:
            args = json.loads(args) if args.strip() else {}
        except ValueError as exc:
            raise _Bad("bad_json", f"{name}: {exc}") from None
    if not isinstance(name, str) or not isinstance(args, Mapping):
        raise _Bad("bad_tool_call", repr(name))
    return name, args


def _from_tool_calls(calls: Sequence[Any]) -> Command:
    base: Optional[Mapping[str, Any]] = None
    extra: List[ToolCall] = []
    for name, args in (_call_parts(c) for c in calls):
        if name != kn.COMMAND_TOOL:
            extra.append(ToolCall(name, MappingProxyType(dict(args))))
        elif base is not None:
            raise _Bad("several_commands", "больше одного вызова command")
        else:
            base = args
    if base is None and not extra:
        raise _Bad("no_command", "пустой список tool_calls")
    cmd = _command(base or {})
    return replace(cmd, actions=cmd.actions + tuple(extra))


def _get(data: Mapping[str, Any], key: str, kind: Union[type, Tuple[type, ...]], default: Any = None) -> Any:
    value = data.get(key, default)
    if value is None or value is default:
        return value
    if isinstance(value, bool) and kind in (int, float, (int, float)):
        raise _Bad("bad_type", key)
    if not isinstance(value, kind):
        raise _Bad("bad_type", key)
    return value


def _only(data: Mapping[str, Any], allowed: Collection[str], where: str) -> None:
    unknown = sorted(set(data) - set(allowed))
    if unknown:
        raise _Bad("unknown_field", f"{where}: {', '.join(map(str, unknown))}", allowed)


def _clean(text: str, where: str, fix: Optional[_Repairs]) -> str:
    """Текст речи без символов, которых в речи не бывает (``knowledge.SPEECH_DROPPED_CHARS``: U+FFFD — битый UTF-8 в
    ответе провайдера, n105 #4021); вырезанное — в ``dropped`` (лог ``fields_dropped``), команда не отвергается."""
    kept = "".join(c for c in text if c not in kn.SPEECH_DROPPED_CHARS)
    if fix is not None and len(kept) != len(text):
        fix.dropped.append(f"{where}.U+FFFD")
    return kept.strip()


def _text(data: Mapping[str, Any], key: str, fix: Optional[_Repairs] = None, where: str = "") -> Optional[str]:
    value = _get(data, key, str)
    return _clean(value, where or key, fix) or None if value is not None else None


def _expression(data: Any, where: str, fix: _Repairs) -> Expression:
    if data is None:
        return Expression()
    if not isinstance(data, Mapping):
        raise _Bad("bad_type", where)
    data = _lift_face(data, where, fix)
    face = data.get("face")
    return Expression(emotion=_get(data, "emotion", str, "neutral") or "neutral", gesture=_get(data, "gesture", str),
                      earcon=_get(data, "earcon", str), face=None if face is None else _face(face),
                      duration_s=_seconds(data.get("duration_s"), where, fix))


def _seconds(value: Any, where: str, fix: _Repairs) -> Optional[float]:
    """Время лица команды (#4083): число — как есть (пределы показа ставит ``expression.show_seconds``); не число —
    поле в ``dropped`` (лог ``fields_dropped``), команда не отвергается (§5.5.9: экспрессия команду не валит)."""
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        fix.dropped.append(f"{where}.duration_s")
        return None
    return float(value)


def _lift_face(data: Mapping[str, Any], where: str, fix: _Repairs) -> Mapping[str, Any]:
    """Лишние поля экспрессии не валят команду (§5.5.9, решение Шифу 21.10 по #3698): слоты лица на уровне
    ``expression`` переезжают в ``expression.face``, если ``face`` не задан и среди них есть хоть один слот; иначе они,
    как и прочие незнакомые поля, уходят в ``dropped``. ``duration_s`` на уровне ``expression`` - своё поле экспрессии
    (время лица команды, #4083), а не лишнее: не переезжает и не выбрасывается."""
    stray = [k for k in data if k not in _EXPRESSION_FIELDS]
    if not stray:
        return data
    kept = {k: v for k, v in data.items() if k in _EXPRESSION_FIELDS}
    lifted = [k for k in stray if k in FACE_FIELDS]
    if data.get("face") is None and any(k in kn.FACE_SCENE_SLOTS for k in lifted):
        kept["face"] = {k: data[k] for k in lifted}
        fix.moved.extend(f"{where}.{k}" for k in lifted)
    else:
        lifted = []
    fix.dropped.extend(f"{where}.{k}" for k in stray if k not in lifted)
    return kept


def _face(data: Any) -> Any:
    """Сцена лица как есть, без проверки enum (её делает :func:`validate`); не объект — вернётся как маркер ошибки."""
    if not isinstance(data, Mapping) or set(data) - set(FACE_FIELDS):
        return data
    slots = {k: data[k] for k in kn.FACE_SCENE_SLOTS if k in data}
    duration = data.get("duration_s", kn.FACE_SCENE_DEFAULT_S)
    return FaceScene(**slots, duration_s=duration)


def _ask(data: Any, fix: _Repairs) -> Optional[Ask]:
    if data is None:
        return None
    if not isinstance(data, Mapping):
        raise _Bad("bad_type", "ask")
    _only(data, ("expects", "text"), "ask")
    return Ask(expects=_get(data, "expects", str, "free") or "free", text=_text(data, "text", fix, "ask.text"))


def _actions(data: Any) -> Tuple[ToolCall, ...]:
    if data is None:
        return ()
    if not isinstance(data, list):
        raise _Bad("bad_type", "actions")
    out = []
    for item in data:
        if not isinstance(item, Mapping):
            raise _Bad("bad_type", "actions[]")
        _only(item, ("tool", "args"), "actions[]")
        tool, args = _get(item, "tool", str), _args(item.get("args"))
        if not tool:
            raise _Bad("bad_type", "actions[].tool")
        out.append(ToolCall(tool, MappingProxyType(dict(args))))
    return tuple(out)


def _args(value: Any) -> Mapping[str, Any]:
    """Аргументы действия. Пустое значение (``""``, ``null``) — пустой объект, JSON-строка объекта — объект: так
    MiniMax пишет ``"args": ""`` в режиме ``tool_choice`` (бейк-офф 21.10: 17 из 23 invalid, ADR §18.2 О4)."""
    if value is None or (isinstance(value, str) and not value.strip()):
        return {}
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except ValueError:
            raise _Bad("bad_type", "actions[].args") from None
    if not isinstance(value, Mapping):
        raise _Bad("bad_type", "actions[].args")
    return value


def _passed(value: Any) -> Optional[str]:
    """Исход «молчать». ``false`` и пустая строка — «не молчу», как отсутствие поля: MiniMax в ``json_in_text`` пишет
    ``"passed": false`` рядом с речью (бейк-офф 21.10: 49 из 52 invalid). ``true`` — не исход из перечисления."""
    if value is None or value is False or value == "":
        return None
    if not isinstance(value, str):
        raise _Bad("bad_type", "passed")
    return value


def _perform(data: Any, fix: _Repairs) -> Optional[Performance]:
    if data is None:
        return None
    if not isinstance(data, Mapping):
        raise _Bad("bad_type", "perform")
    _only(data, ("kind", "segments", "backing", "expression_per_segment", "voice_per_segment"), "perform")
    segments = _get(data, "segments", list, []) or []
    if not all(isinstance(s, str) for s in segments):
        raise _Bad("bad_type", "perform.segments")
    per = _get(data, "expression_per_segment", list)
    return Performance(
        kind=_get(data, "kind", str, "") or "",
        segments=tuple(_clean(s, f"perform.segments[{i}]", fix) for i, s in enumerate(segments)),
        backing=_get(data, "backing", str, "none") or "none",
        expression_per_segment=None if per is None else tuple(
            _expression(e, f"perform.expression_per_segment[{i}]", fix) for i, e in enumerate(per)),
        voice_per_segment=_voices(data),
    )


def _voices(data: Mapping[str, Any]) -> Optional[Tuple[str, ...]]:
    """Голоса сегментов; ``null`` в списке — текущий голос робота (как ``""``)."""
    voices = _get(data, "voice_per_segment", list)
    if voices is None:
        return None
    if not all(v is None or isinstance(v, str) for v in voices):
        raise _Bad("bad_type", "perform.voice_per_segment")
    return tuple((v or "").strip() for v in voices)


def _lift_edit(data: Mapping[str, Any], fix: _Repairs) -> Mapping[str, Any]:
    """Поля правки выступления на их место (§5.6.4 X3, #3700). Живой прогон A20 22.10, DeepSeek 3 из 3 мимо схемы:
    ``edit_pending`` внутри ``perform`` (→ переезжает в команду, ``moved``; флаг команды, если есть, главный) и
    ``from_segment`` в ``perform`` или в команде с номером звучащего сегмента. С какого сегмента править, выбирает код
    (первый не отданный на звучание — ``PerformHost.edit``), поэтому ``from_segment`` модели уходит в ``dropped``."""
    out = dict(data)
    if "from_segment" in out:
        del out["from_segment"]
        fix.dropped.append("command.from_segment")
    perf = out.get("perform")
    if isinstance(perf, Mapping) and ("edit_pending" in perf or "from_segment" in perf):
        inner = dict(perf)
        if "edit_pending" in inner:
            out.setdefault("edit_pending", inner.pop("edit_pending"))
            fix.moved.append("perform.edit_pending")
        if "from_segment" in inner:
            del inner["from_segment"]
            fix.dropped.append("perform.from_segment")
        out["perform"] = inner
    return out


def _command(data: Mapping[str, Any]) -> Command:
    fix = _Repairs()
    data = _lift_edit(data, fix)
    _only(data, _COMMAND_FIELDS, "command")
    cmd = Command(
        speech=_text(data, "speech", fix), voice=_text(data, "voice", fix), ask=_ask(data.get("ask"), fix),
        actions=_actions(data.get("actions")),
        perform=_perform(data.get("perform"), fix), when=_get(data, "when", str, "after_current") or "after_current",
        edit_pending=bool(_get(data, "edit_pending", bool, False)),
        expression=_expression(data.get("expression"), "expression", fix), passed=_passed(data.get("passed")),
    )
    return replace(cmd, moved=tuple(fix.moved), dropped=tuple(fix.dropped))


# ---------------------------------------------------------------------------------------------------------------
# validate
# ---------------------------------------------------------------------------------------------------------------


def validate(cmd: Result, available_tools: Collection[str], session: Any = None, busy: Busy = Busy(),
             form: Optional[Form] = None) -> Result:
    """Проверить команду до исполнения: тулы видны и есть в таблице классов, аргументы — по схеме каталога,
    enum экспрессии, пределы речи/действий/сегментов, правила ``perform``/``when``/``edit_pending``.

    ``session`` — снимок сессии агента (берётся ``agent``: ``say_aloud`` — только оператору и телеграму);
    ``busy`` — что занято сейчас; ``form`` — форма команды хода (``None`` — текущая, :func:`current_form`):
    голый ``passed`` на адресованную реплику — нарушение ``VIOLATION_PASSED_ADDRESSED``, ``passed`` рядом с ответом там
    же и голос реплики без просьбы о голосе отбрасываются (``dropped``). Невалидная сцена лица отбрасывается (``dropped``), а не валит команду (§5.5.9);
    так же лишние поля ВЛОЖЕННЫХ объектов аргументов тула (``play_animation.face.emotion``, #3974): без отказа
    команды, с записью в ``dropped``. Обязательные поля, типы и enum по-прежнему проверяет схема.
    """
    if isinstance(cmd, Invalid):
        return cmd
    form = form or current_form()
    cmd = _drop_closed_passed(_drop_unasked_voice(_drop_nested_extras(cmd), form), form)
    agent = getattr(session, "agent", None) or kn.AGENTS[0]
    checks: Tuple[Callable[[], Any], ...] = (
        lambda: _check_passed(cmd, form),
        lambda: _check_speech(cmd.speech, kn.SPEECH_MAX_CHARS, "speech"),
        lambda: _check_voice(cmd.voice, busy),
        lambda: _check_ask(cmd.ask),
        lambda: _check_actions(cmd.actions, set(available_tools), agent),
        lambda: _check_speech_with_actions(cmd, agent),
        lambda: _check_perform(cmd, busy),
        lambda: _check_when(cmd.when, busy),
        lambda: _drop_bad_faces(cmd),
    )
    outcomes = [_outcome(check) for check in checks]
    found = tuple(v for v in outcomes if isinstance(v, Violation))
    if found:
        return Invalid(found[0].reason, found[0].detail, violations=found,
                       salvage=_salvage(cmd, set(available_tools), agent))
    return outcomes[-1]


def _drop_extra(schema: Mapping[str, Any], value: Any, path: str, nested: bool) -> Tuple[Any, List[str]]:
    """Значение без лишних полей вложенных закрытых объектов (``additionalProperties: false``) и список отброшенных
    путей. Верхний уровень аргументов не трогаем: лишний аргумент тула — по-прежнему ``bad_args``."""
    if isinstance(value, Mapping):
        props = schema.get("properties", {})
        closed = nested and schema.get("additionalProperties") is False
        dropped = [f"{path}.{k}" for k in value if closed and k not in props]
        kept: Dict[str, Any] = {}
        for key, item in value.items():
            if key in props:
                kept[key], sub = _drop_extra(props[key], item, f"{path}.{key}", True)
                dropped += sub
            elif not closed:
                kept[key] = item
        return kept, dropped
    items = schema.get("items")
    if isinstance(value, (list, tuple)) and isinstance(items, Mapping):
        pairs = [_drop_extra(items, item, f"{path}[{i}]", True) for i, item in enumerate(value)]
        return [v for v, _ in pairs], [d for _, ds in pairs for d in ds]
    return value, []


def _drop_nested_extras(cmd: Command) -> Command:
    """Аргументы действий без лишних вложенных полей (#3974): «n905: play_animation.face.emotion: лишнее поле» не
    повод отказывать команде. Отброшенное - в ``dropped`` (лог ``fields_dropped``)."""
    actions, notes = [], []
    for i, call in enumerate(cmd.actions):
        try:
            schema = get_tool(call.tool).parameters
        except KeyError:  # тула нет в каталоге - скажет _check_actions
            actions.append(call)
            continue
        args, dropped = _drop_extra(schema, call.args, f"actions[{i}].{call.tool}", False)
        actions.append(replace(call, args=MappingProxyType(args)) if dropped else call)
        notes += dropped
    if not notes:
        return cmd
    return replace(cmd, actions=tuple(actions), dropped=cmd.dropped + tuple(notes))


def loud_actions(cmd: Command) -> Tuple[ToolCall, ...]:
    """Действия, рядом с которыми ``speech`` не звучит: все, кроме тихих (``knowledge.QUIET_WITH_SPEECH_TOOLS``)."""
    return tuple(a for a in cmd.actions if a.tool not in kn.QUIET_WITH_SPEECH_TOOLS)


def _salvage(cmd: Command, available: Collection[str], agent: str) -> Tuple[ToolCall, ...]:
    """Действия отвергнутой команды, валидные по одному: тул виден, аргументы по схеме, не ``query`` (данные некому
    вернуть)."""

    def fine(call: ToolCall) -> bool:
        if kn.TOOL_CLASSES.get(call.tool) == "query":
            return False
        try:
            _check_actions((call,), available, agent)
        except _Bad:
            return False
        return True

    return tuple(call for call in cmd.actions if fine(call))


def _outcome(check: Callable[[], Any]) -> Any:
    """Итог одной проверки: её значение или :class:`Violation` — проверки независимы, модель видит все нарушения."""
    try:
        return check()
    except _Bad as bad:
        return bad.violation()


def _check_passed(cmd: Command, form: Form) -> None:
    if cmd.passed is None:
        return
    if not form.passed:
        raise _Bad(kn.VIOLATION_PASSED_ADDRESSED, f"passed={cmd.passed}")
    if cmd.passed not in kn.PASSED:
        raise _Bad("bad_enum", f"passed={cmd.passed}", kn.PASSED)
    if cmd.speech or cmd.ask or cmd.actions or cmd.perform:
        raise _Bad("passed_with_content", "молчание вместе с речью или действиями")


def _drop_closed_passed(cmd: Command, form: Form) -> Command:
    """``passed`` вместе с ответом (речь, вопрос, действия, выступление) там, где форма хода ``passed`` закрыла
    (адресованная реплика, #4021) — лишнее поле, а не нарушение: ответ исполняется, поле — в ``dropped``. Иначе
    ответ терялся: нарушение ``passed_addressed``, повтор, и после повтора ход мычал фразой ``passed`` (ревью #4053,
    ``turn.silence_insisted``). Голый ``passed`` — по-прежнему нарушение; с открытой формой (подхват бэклога) правило
    ``passed_with_content`` прежнее."""
    content = cmd.speech or cmd.ask or cmd.actions or cmd.perform
    if cmd.passed is None or form.passed or not content:
        return cmd
    return replace(cmd, passed=None, dropped=cmd.dropped + ("passed",))


def _drop_unasked_voice(cmd: Command, form: Form) -> Command:
    """Голос реплики без просьбы о голосе (форма хода закрыла ``voice``, #4021 n109) — не нарушение, а лишнее поле:
    реплика звучит текущим голосом робота, поле — в ``dropped`` (лог ``fields_dropped``)."""
    if cmd.voice is None or form.voice:
        return cmd
    return replace(cmd, voice=None, dropped=cmd.dropped + ("voice",))


def _check_voice(voice: Optional[str], busy: Busy) -> None:
    """Голос реплики — из каталога звучащего провайдера (``Busy.voices``; пусто — не сверяется), как и голоса
    сегментов (:func:`_check_voices`): чужое имя TTS подменил бы молча (#4021)."""
    _check_speech(voice, kn.VOICE_MAX_CHARS, "voice")
    if voice and busy.voices and voice not in busy.voices:
        raise _Bad("bad_enum", f"voice={voice}", busy.voices)


def is_empty(cmd: Command) -> bool:
    """Команде нечего исполнить и нечего сказать: ни речи, ни вопроса, ни действий, ни выступления, ни экспрессии, и
    не ``passed``. Экспрессия сверх нейтрали (эмоция, жест, earcon, сцена лица) - полноценная команда «покажи на лице»
    без слов (#3974: «покажи, как тебе грустно»); пустой ответ МОДЕЛИ мост режима команд считает нарушением
    :data:`knowledge.VIOLATION_EMPTY_COMMAND` (#3698). Время без лица (``expression.duration_s`` при нейтрали, #4083)
    показать нечего — команда пустая."""
    return cmd.passed is None and replace(cmd.expression, duration_s=None) == Expression() and not (
        cmd.speech or cmd.ask or cmd.actions or cmd.perform)


def _check_speech(text: Optional[str], limit: int, where: str) -> None:
    if text is None:
        return
    if len(text) > limit:
        raise _Bad("too_long", f"{where}: {len(text)} > {limit}")
    if any(unicodedata.category(c) == "Cc" and c != "\n" for c in text):
        raise _Bad("control_chars", where)


def _check_speech_with_actions(cmd: Command, agent: str) -> None:
    """``speech`` рядом с действиями не звучит (§4.1): нарушение, а не молчаливая потеря ответа. Вместе с ``perform``
    его уже ловит ``perform_with_speech``; у телеграм-агента ``speech`` - текст в чат, он законен."""
    if cmd.speech and loud_actions(cmd) and cmd.perform is None and agent != kn.TEXT_CHANNEL_AGENT:
        raise _Bad(kn.VIOLATION_SPEECH_WITH_ACTIONS)


def _check_ask(ask: Optional[Ask]) -> None:
    if ask is None:
        return
    if ask.expects not in kn.ASK_EXPECTS:
        raise _Bad("bad_enum", f"ask.expects={ask.expects}", kn.ASK_EXPECTS)
    if (ask.text is None) == (ask.key is None):
        raise _Bad("bad_ask", "у вопроса ровно одно из text/key")
    _check_speech(ask.text, kn.SPEECH_MAX_CHARS, "ask.text")


def _check_actions(actions: Tuple[ToolCall, ...], available: Collection[str], agent: str) -> None:
    if len(actions) > kn.ACTIONS_MAX:
        raise _Bad("too_many_actions", f"{len(actions)} > {kn.ACTIONS_MAX}")
    per_class: dict = {}
    for call in actions:
        cls = _action_class(call.tool, available, agent)
        per_class[cls] = per_class.get(cls, 0) + 1
        error = _schema_error(get_tool(call.tool).parameters, call.args, call.tool)
        if error:
            raise _Bad("bad_args", error)
    crowded = [c for c in kn.SINGLE_ACTION_CLASSES if per_class.get(c, 0) > 1]
    if crowded:
        raise _Bad("single_action_class", ", ".join(crowded))


def _same_class_first(tool: str, actionable: Sequence[str]) -> Tuple[str, ...]:
    """Тулы среза хода, из которых модель выбирает замену тулу вне среза: того же класса действия (``TOOL_CLASSES``,
    ADR-0148 — выбор сужает код); если таких нет или класс тула неизвестен — весь срез действий."""
    cls = kn.TOOL_CLASSES.get(tool)
    same = [t for t in actionable if cls is not None and kn.TOOL_CLASSES.get(t) == cls]
    return tuple(same or actionable)


def _action_class(tool: str, available: Collection[str], agent: str) -> str:
    actionable = sorted(t for t in available if t in kn.TOOL_CLASSES and t not in kn.SPEECH_CHANNEL_TOOLS)
    if tool in kn.SPEECH_CHANNEL_TOOLS:
        raise _Bad("speech_channel_tool", tool)
    if tool not in available:
        raise _Bad("tool_not_available", tool, _same_class_first(tool, actionable))
    cls = kn.TOOL_CLASSES.get(tool)
    if cls is None:
        raise _Bad("unknown_tool", tool, actionable)
    if agent not in kn.ACTION_CLASSES[cls].agents:
        raise _Bad("class_not_for_agent", f"{tool}: {cls} — не для {agent}")
    return cls


def _check_perform(cmd: Command, busy: Busy) -> None:
    perf = cmd.perform
    if perf is None:
        if cmd.edit_pending:
            raise _Bad("edit_without_perform", "edit_pending без perform")
        return
    if cmd.speech:
        raise _Bad("perform_with_speech", "perform взаимоисключает speech")
    if perf.kind not in kn.PERFORM_KINDS:
        raise _Bad("bad_enum", f"perform.kind={perf.kind}", kn.PERFORM_KINDS)
    if perf.backing not in kn.PERFORM_BACKINGS:
        raise _Bad("bad_enum", f"perform.backing={perf.backing}", kn.PERFORM_BACKINGS)
    _check_segments(perf, busy)
    for i, segment in enumerate(perf.segments):
        _check_speech(segment, kn.SEGMENT_MAX_CHARS, f"perform.segments[{i}]")
    for name, per in (("expression_per_segment", perf.expression_per_segment),
                      ("voice_per_segment", perf.voice_per_segment)):
        if per is not None and len(per) != len(perf.segments):
            raise _Bad("bad_segments", f"{name} не по числу сегментов")
    _check_voices(perf, busy)
    _check_edit(cmd.edit_pending, busy)


def _check_voices(perf: Performance, busy: Busy) -> None:
    """Голоса сегментов: из каталога звучащего провайдера (``Busy.voices``, пусто - не сверяется); у сказки обязательны
    (#3959: без них она звучит одним голосом, а валидатор молчал)."""
    for i, voice in enumerate(perf.voice_per_segment or ()):
        _check_speech(voice, kn.VOICE_MAX_CHARS, f"perform.voice_per_segment[{i}]")
        if voice and busy.voices and voice not in busy.voices:
            raise _Bad("bad_enum", f"perform.voice_per_segment[{i}]={voice}", busy.voices)
    if perf.kind == "tale" and perf.segments and perf.voice_per_segment is None:
        raise _Bad("tale_without_voices", f"{len(perf.segments)} сегментов без voice_per_segment", busy.voices)


def _check_segments(perf: Performance, busy: Busy) -> None:
    """Число сегментов. Во время выступления ``perform`` — правка (MERGE): хватает одного нового куплета, а без
    куплетов, но с подложкой — «добавь бит» (#993), бит ставит хост выступления. A22 22.10 (#3700): DeepSeek на
    «Робот, добавь бит» шлёт ``perform{backing: beat, segments: []}`` — раньше ``bad_segments`` и ход провален."""
    running = busy.active_segment is not None
    low = (0 if perf.backing in kn.PERFORM_BACKING_MOODS else 1) if running else kn.SEGMENTS_MIN
    if not low <= len(perf.segments) <= kn.SEGMENTS_MAX or not all(perf.segments):
        raise _Bad("bad_segments", f"{len(perf.segments)} сегментов")


def _check_edit(edit_pending: bool, busy: Busy) -> None:
    """Правке нужно идущее выступление; с какого сегмента — решает его хост (#3700), не модель."""
    if edit_pending and busy.active_segment is None:
        raise _Bad("edit_without_active_perform", "править нечего: выступление не идёт")


def _check_when(when: str, busy: Busy) -> None:
    if when not in kn.WHEN:
        raise _Bad("bad_enum", f"when={when}", kn.WHEN)
    if when == "now":
        hard = sorted(c for c in busy.classes if c in kn.ACTION_CLASSES and not kn.ACTION_CLASSES[c].interruptible)
        if hard:
            raise _Bad("now_vs_uninterruptible", ", ".join(hard))


def _expression_ok(expr: Expression) -> Tuple[Expression, bool]:
    """Проверенная экспрессия и флаг «сцена лица отброшена». Чужая эмоция/жест/earcon — команда невалидна."""
    if expr.emotion not in kn.EMOTIONS:
        raise _Bad("bad_enum", f"emotion={expr.emotion}", tuple(kn.EMOTIONS))
    if expr.gesture is not None and expr.gesture not in kn.GESTURES:
        raise _Bad("bad_enum", f"gesture={expr.gesture}", kn.GESTURES)
    if expr.earcon is not None and expr.earcon not in kn.EARCONS:
        raise _Bad("bad_enum", f"earcon={expr.earcon}", kn.EARCONS)
    if expr.face is None or _face_ok(expr.face):
        return expr, False
    return replace(expr, face=None), True


def _face_ok(face: Any) -> bool:
    """Сцена лица годится, если ``face_scene.problem`` ничего не нашёл."""
    return face_problem(face) == ""


def _drop_bad_faces(cmd: Command) -> Command:
    expression, dropped = _expression_ok(cmd.expression)
    notes = ["expression.face"] if dropped else []
    perf = cmd.perform
    if perf is not None and perf.expression_per_segment is not None:
        checked = [_expression_ok(e) for e in perf.expression_per_segment]
        notes += [f"perform.expression_per_segment[{i}].face" for i, (_, d) in enumerate(checked) if d]
        perf = replace(perf, expression_per_segment=tuple(e for e, _ in checked))
    return replace(cmd, expression=expression, perform=perf, dropped=cmd.dropped + tuple(notes))


# ---------------------------------------------------------------------------------------------------------------
# explain — нарушения словами для модели (повтор команды, #3698; по образцу deepseek-harness: invalid arguments)
# ---------------------------------------------------------------------------------------------------------------

#: Первая строка tool-результата невалидной команде.
INVALID_HEAD = "invalid arguments: команда не исполнена, ничего не сделано. Исправь нарушения и пришли команду заново."
#: То же, когда запрос хода (``query``) уже выполнен и его данные у модели (#3883): честно, без «ничего не сделано» —
#: иначе модель кладёт запрос в ``actions`` заново и ход упирается в ``speech_with_actions``.
INVALID_AFTER_QUERY_HEAD = (f"invalid arguments: данные запроса уже получены, запрос выполнен — повторять его не нужно. "
                            f"Пришли команду заново: ответ по этим данным в speech, не длиннее {kn.SPEECH_MAX_CHARS} "
                            f"символов, без actions.")
#: Tool-результат прочим вызовам того же ответа: команда отклонена целиком.
NOT_RUN = "не исполнено: команда этого ответа невалидна, пришли всё заново одной командой."
#: Правило словами по причине нарушения; подробность и перечень допустимого — из :class:`Violation`.
HINTS: Mapping[str, str] = MappingProxyType({
    "unknown_field": "такого поля нет в схеме",
    "bad_enum": "значение не из перечня",
    "bad_type": "не тот тип значения",
    "bad_json": "аргументы command не разобраны как JSON-объект",
    "truncated": "JSON аргументов оборван — ответь короче",
    "no_command": "нет команды: ответ — один JSON-объект команды",
    "several_commands": "команда в ответе одна",
    "perform_with_speech": "при perform поле speech не пиши — весь текст клади в perform.segments",
    kn.VIOLATION_SPEECH_WITH_ACTIONS: "speech при громких actions не звучит: если это ответ по сути (шутка, стих, ответ на "
                                      "вопрос) — положи его в perform.segments; если это слова про само действие "
                                      "(«вот звук») — убери speech, о результате скажет код",
    kn.VIOLATION_EMPTY_COMMAND: "команда пустая: ответь речью (speech), вопросом (ask), действием (actions) или "
                                "выступлением (perform); молчать — passed",
    "single_action_class": "в этом классе действий — одно действие за ход",
    "too_many_actions": f"действий не больше {kn.ACTIONS_MAX}",
    "bad_args": "аргументы действия не по схеме тула",
    "tool_not_available": "этого тула в этом ходе нет: возьми из перечня (тулы хода того же класса действия, "
                          "а если класс неизвестен — все тулы хода)",
    "unknown_tool": "такого тула нет",
    "speech_channel_tool": "речь — поле speech, а не тул в actions",
    "bad_segments": f"в perform от {kn.SEGMENTS_MIN} до {kn.SEGMENTS_MAX} непустых сегментов, "
                    "списки *_per_segment — по числу сегментов",
    "tale_without_voices": "сказка: voice_per_segment обязателен, на каждый сегмент — голос персонажа из списка "
                           "голосов (\"\" — рассказчик, текущий голос); сегмент = реплика одного говорящего, "
                           "повествование и каждая реплика героя — отдельные сегменты",
    "too_long": "слишком длинно",
    "control_chars": "управляющие символы в тексте",
    "passed_with_content": "passed — только без речи и действий",
    kn.VIOLATION_PASSED_ADDRESSED: "реплика адресована тебе — молчать нельзя, поля passed в этом ходе нет: ответь "
                                   "словами в speech (не понял — переспроси в ask)",
    "bad_ask": "у ask нужен text",
    "bad_tool_call": "вызов — объект с name и arguments",
    "class_not_for_agent": "этот класс действий тебе недоступен",
    "edit_without_perform": "edit_pending — только вместе с perform",
    "edit_without_active_perform": "править нечего: выступление не идёт, убери edit_pending",
    "now_vs_uninterruptible": "when=now нельзя: занятое действие не прерывается, нужен after_current",
})


def is_query_data(content: str) -> bool:
    """Сообщение хода с данными ``query`` (``turn._query_data``): запрос в этом ходе уже выполнен."""
    return content.lstrip().startswith('{"' + kn.QUERY_RESULTS_FIELD + '"')


def explain(invalid: Invalid, after_query: bool = False) -> str:
    """Нарушения невалидной команды словами для модели: причина, правило, где, что допустимо. ``after_query`` — запрос
    хода уже выполнен: подсказка говорит, что данные получены, а не что «ничего не сделано» (#3883)."""
    lines = [INVALID_AFTER_QUERY_HEAD if after_query else INVALID_HEAD]
    for v in invalid.all:
        line = f"- {v.reason}: {HINTS.get(v.reason, 'нарушено правило команды')}"
        line += f" ({v.detail})" if v.detail else ""
        lines.append(line + (f". Допустимо: {', '.join(v.allowed)}" if v.allowed else ""))
    return "\n".join(lines)


# ---------------------------------------------------------------------------------------------------------------
# to_args —команда обратно в поля схемы (мост v1 передаёт её циклу тулов одним вызовом ``command``)
# ---------------------------------------------------------------------------------------------------------------


def to_args(cmd: Command) -> Dict[str, Any]:
    """Поля схемы команды; ``parse(to_args(c)) == c`` для разобранной команды (``dropped``/``moved`` не переносятся:
    переложенные слоты уже в ``expression.face``, отброшенных полей в аргументах нет)."""
    out: Dict[str, Any] = {k: v for k, v in (("speech", cmd.speech), ("voice", cmd.voice), ("passed", cmd.passed))
                           if v is not None}
    if cmd.ask is not None:
        out["ask"] = {"expects": cmd.ask.expects, "text": cmd.ask.text}
    if cmd.actions:
        out["actions"] = [{"tool": a.tool, "args": dict(a.args)} for a in cmd.actions]
    if cmd.perform is not None:
        out["perform"] = _perform_args(cmd.perform)
    if cmd.when != "after_current":
        out["when"] = cmd.when
    if cmd.edit_pending:
        out["edit_pending"] = True
    out["expression"] = _expression_args(cmd.expression)
    return out


def _expression_args(expr: Expression) -> Dict[str, Any]:
    out: Dict[str, Any] = {"emotion": expr.emotion}
    out.update({k: v for k, v in (("gesture", expr.gesture), ("earcon", expr.earcon)) if v is not None})
    if expr.face is not None:
        out["face"] = expr.face.to_dict() if isinstance(expr.face, FaceScene) else expr.face
    if expr.duration_s is not None:
        out["duration_s"] = expr.duration_s
    return out


def _perform_args(perf: Performance) -> Dict[str, Any]:
    out: Dict[str, Any] = {"kind": perf.kind, "segments": list(perf.segments), "backing": perf.backing}
    if perf.expression_per_segment is not None:
        out["expression_per_segment"] = [_expression_args(e) for e in perf.expression_per_segment]
    if perf.voice_per_segment is not None:
        out["voice_per_segment"] = list(perf.voice_per_segment)
    return out


# ---------------------------------------------------------------------------------------------------------------
# JSON-схема каталога (подмножество ключевых слов, которое в нём встречается)
# ---------------------------------------------------------------------------------------------------------------

_TYPES = {"object": Mapping, "array": (list, tuple), "string": str, "boolean": bool,
          "integer": int, "number": (int, float)}


def _schema_error(schema: Mapping[str, Any], value: Any, path: str) -> Optional[str]:
    """Первое нарушение схемы или ``None``: type, enum, границы, required/properties/additionalProperties, items."""
    for error in _schema_errors(schema, value, path):
        return error
    return None


def _schema_errors(schema: Mapping[str, Any], value: Any, path: str) -> Iterator[str]:
    kind = schema.get("type")
    expected = _TYPES.get(kind) if isinstance(kind, str) else None
    if expected is not None and (not isinstance(value, expected)
                                 or (isinstance(value, bool) and kind in ("integer", "number"))):
        yield f"{path}: ожидался {kind}"
        return
    if "enum" in schema and value not in schema["enum"]:
        yield f"{path}: {value!r} не из {list(schema['enum'])}"
    yield from _bounds(schema, value, path)
    if isinstance(value, Mapping):
        yield from _object_errors(schema, value, path)
    if isinstance(value, (list, tuple)) and isinstance(schema.get("items"), Mapping):
        for i, item in enumerate(value):
            yield from _schema_errors(schema["items"], item, f"{path}[{i}]")


def _bounds(schema: Mapping[str, Any], value: Any, path: str) -> Iterator[str]:
    number = isinstance(value, (int, float)) and not isinstance(value, bool)
    if number and "minimum" in schema and value < schema["minimum"]:
        yield f"{path}: меньше {schema['minimum']}"
    if number and "maximum" in schema and value > schema["maximum"]:
        yield f"{path}: больше {schema['maximum']}"
    sized = isinstance(value, (str, list, tuple))
    low, high = ("minLength", "maxLength") if isinstance(value, str) else ("minItems", "maxItems")
    if sized and low in schema and len(value) < schema[low]:
        yield f"{path}: короче {schema[low]}"
    if sized and high in schema and len(value) > schema[high]:
        yield f"{path}: длиннее {schema[high]}"


def _object_errors(schema: Mapping[str, Any], value: Mapping[str, Any], path: str) -> Iterator[str]:
    props = schema.get("properties", {})
    for key in schema.get("required", ()):
        if key not in value:
            yield f"{path}.{key}: обязателен"
    if schema.get("additionalProperties") is False:
        for key in value:
            if key not in props:
                yield f"{path}.{key}: лишнее поле"
    for key, sub in props.items():
        if key in value:
            yield from _schema_errors(sub, value[key], f"{path}.{key}")
