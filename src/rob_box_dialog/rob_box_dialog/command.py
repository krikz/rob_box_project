"""Команда LLM — узкий слот ответа хода (ADR-0150 §4.1; ADR-0148 «код решает, LLM говорит»).

LLM на каждом ходе отдаёт ОДНУ команду: речь, вопрос, до трёх действий, выступление, экспрессия. :func:`parse`
собирает её из ``tool_calls``, JSON или свободного текста с JSON-объектом; :func:`validate` проверяет по таблицам
:mod:`.knowledge` и JSON-схемам каталога тулов. Невалидно — :class:`Invalid` с вопросом ``understand.invalid``:
повторного вызова LLM нет, ретраев нет.
"""

from __future__ import annotations

import json
import unicodedata
from dataclasses import dataclass, field, replace
from types import MappingProxyType
from typing import Any, Collection, Dict, FrozenSet, Iterator, List, Mapping, Optional, Sequence, Tuple, Union

from rob_box_core.tool_catalog import get_tool
from rob_box_dialog import knowledge as kn
from rob_box_dialog.face_scene import FaceScene, problem as face_problem

INVALID_ASK_KEY = "understand.invalid"
#: Поля команды — из той же схемы, что видит LLM (:func:`knowledge.command_schema`).
_COMMAND_FIELDS: Tuple[str, ...] = tuple(kn.command_schema("json")["properties"])


@dataclass(frozen=True)
class Expression:
    """Экспрессия хода: эмоция, жест, earcon, сцена лица — enum из :mod:`.knowledge`."""

    emotion: str = "neutral"
    gesture: Optional[str] = None
    earcon: Optional[str] = None
    #: TODO(PR-8, ADR-0150 §5.5.9): исполнитель v2 шлёт ``face.to_dict()`` в ``ExpressionRequest.face_json``; проверка
    #: общая с v1 (``play_animation(face=...)``) - :func:`rob_box_dialog.face_scene.validate`.
    face: Optional[FaceScene] = None


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
    """Выступление (§5.6.3): сегменты — содержимое, а не отчёт; ``from_segment`` — с какого править (edit_pending).

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
    """Одна команда хода. ``dropped`` — поля, которые валидатор отбросил, не отвергая команду (сцена лица).

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


@dataclass(frozen=True)
class Invalid:
    """Невалидный ответ LLM: машинная причина, подробность для лога и вопрос-шаблон человеку."""

    reason: str
    detail: str = ""
    ask: Ask = Ask(expects="free", key=INVALID_ASK_KEY)


@dataclass(frozen=True)
class Busy:
    """Что занято сейчас — классы действий и ACTIVE-сегмент выступления (заполняет планировщик, PR-7b)."""

    classes: FrozenSet[str] = frozenset()
    active_segment: Optional[int] = None


Result = Union[Command, Invalid]


class _Bad(Exception):
    """Внутренний сигнал разбора/проверки: наружу не выходит — превращается в :class:`Invalid`."""

    def __init__(self, reason: str, detail: str = "") -> None:
        super().__init__(reason)
        self.reason = reason
        self.detail = detail


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
        return Invalid(bad.reason, bad.detail)


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
        raise _Bad("unknown_field", f"{where}: {', '.join(map(str, unknown))}")


def _text(data: Mapping[str, Any], key: str) -> Optional[str]:
    value = _get(data, key, str)
    return value.strip() or None if value is not None else None


def _expression(data: Any) -> Expression:
    if data is None:
        return Expression()
    if not isinstance(data, Mapping):
        raise _Bad("bad_type", "expression")
    _only(data, ("emotion", "gesture", "earcon", "face"), "expression")
    face = data.get("face")
    return Expression(emotion=_get(data, "emotion", str, "neutral") or "neutral", gesture=_get(data, "gesture", str),
                      earcon=_get(data, "earcon", str), face=None if face is None else _face(face))


def _face(data: Any) -> Any:
    """Сцена лица как есть, без проверки enum (её делает :func:`validate`); не объект — вернётся как маркер ошибки."""
    if not isinstance(data, Mapping) or set(data) - {*kn.FACE_SCENE_SLOTS, "duration_s"}:
        return data
    slots = {k: data[k] for k in kn.FACE_SCENE_SLOTS if k in data}
    duration = data.get("duration_s", kn.FACE_SCENE_DEFAULT_S)
    return FaceScene(**slots, duration_s=duration)


def _ask(data: Any) -> Optional[Ask]:
    if data is None:
        return None
    if not isinstance(data, Mapping):
        raise _Bad("bad_type", "ask")
    _only(data, ("expects", "text"), "ask")
    return Ask(expects=_get(data, "expects", str, "free") or "free", text=_text(data, "text"))


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


def _perform(data: Any) -> Optional[Performance]:
    if data is None:
        return None
    if not isinstance(data, Mapping):
        raise _Bad("bad_type", "perform")
    _only(data, ("kind", "segments", "backing", "expression_per_segment", "from_segment", "voice_per_segment"),
          "perform")
    segments = _get(data, "segments", list, []) or []
    if not all(isinstance(s, str) for s in segments):
        raise _Bad("bad_type", "perform.segments")
    per = _get(data, "expression_per_segment", list)
    return Performance(
        kind=_get(data, "kind", str, "") or "", segments=tuple(s.strip() for s in segments),
        backing=_get(data, "backing", str, "none") or "none",
        expression_per_segment=None if per is None else tuple(_expression(e) for e in per),
        from_segment=_get(data, "from_segment", int, 0) or 0,
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


def _command(data: Mapping[str, Any]) -> Command:
    _only(data, _COMMAND_FIELDS, "command")
    return Command(
        speech=_text(data, "speech"), voice=_text(data, "voice"), ask=_ask(data.get("ask")),
        actions=_actions(data.get("actions")),
        perform=_perform(data.get("perform")), when=_get(data, "when", str, "after_current") or "after_current",
        edit_pending=bool(_get(data, "edit_pending", bool, False)), expression=_expression(data.get("expression")),
        passed=_passed(data.get("passed")),
    )


# ---------------------------------------------------------------------------------------------------------------
# validate
# ---------------------------------------------------------------------------------------------------------------


def validate(cmd: Result, available_tools: Collection[str], session: Any = None, busy: Busy = Busy()) -> Result:
    """Проверить команду до исполнения: тулы видны и есть в таблице классов, аргументы — по схеме каталога,
    enum экспрессии, пределы речи/действий/сегментов, правила ``perform``/``when``/``edit_pending``.

    ``session`` — снимок сессии агента (берётся ``agent``: ``say_aloud`` — только оператору и телеграму);
    ``busy`` — что занято сейчас. Невалидная сцена лица отбрасывается (``dropped``), а не валит команду (§5.5.9).
    """
    if isinstance(cmd, Invalid):
        return cmd
    try:
        agent = getattr(session, "agent", None) or kn.AGENTS[0]
        _check_passed(cmd)
        _check_speech(cmd.speech, kn.SPEECH_MAX_CHARS, "speech")
        _check_speech(cmd.voice, kn.VOICE_MAX_CHARS, "voice")
        _check_ask(cmd.ask)
        _check_actions(cmd.actions, set(available_tools), agent)
        _check_perform(cmd, busy)
        _check_when(cmd.when, busy)
        return _drop_bad_faces(cmd)
    except _Bad as bad:
        return Invalid(bad.reason, bad.detail)


def _check_passed(cmd: Command) -> None:
    if cmd.passed is None:
        return
    if cmd.passed not in kn.PASSED:
        raise _Bad("bad_enum", f"passed={cmd.passed}")
    if cmd.speech or cmd.ask or cmd.actions or cmd.perform:
        raise _Bad("passed_with_content", "молчание вместе с речью или действиями")


def _check_speech(text: Optional[str], limit: int, where: str) -> None:
    if text is None:
        return
    if len(text) > limit:
        raise _Bad("too_long", f"{where}: {len(text)} > {limit}")
    if any(unicodedata.category(c) == "Cc" and c != "\n" for c in text):
        raise _Bad("control_chars", where)


def _check_ask(ask: Optional[Ask]) -> None:
    if ask is None:
        return
    if ask.expects not in kn.ASK_EXPECTS:
        raise _Bad("bad_enum", f"ask.expects={ask.expects}")
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


def _action_class(tool: str, available: Collection[str], agent: str) -> str:
    if tool in kn.SPEECH_CHANNEL_TOOLS:
        raise _Bad("speech_channel_tool", tool)
    if tool not in available:
        raise _Bad("tool_not_available", tool)
    cls = kn.TOOL_CLASSES.get(tool)
    if cls is None:
        raise _Bad("unknown_tool", tool)
    if agent not in kn.ACTION_CLASSES[cls].agents:
        raise _Bad("class_not_for_agent", f"{tool}: {cls} — не для {agent}")
    return cls


def _check_perform(cmd: Command, busy: Busy) -> None:
    perf = cmd.perform
    if perf is None:
        if cmd.edit_pending:
            raise _Bad("edit_without_perform", "edit_pending без perform")
        return
    if cmd.speech and not cmd.actions:  # при действиях speech не звучит (§4.1) — двух голосов хода нет
        raise _Bad("perform_with_speech", "perform взаимоисключает speech")
    if perf.kind not in kn.PERFORM_KINDS or perf.backing not in kn.PERFORM_BACKINGS:
        raise _Bad("bad_enum", f"perform.kind={perf.kind} backing={perf.backing}")
    if not kn.SEGMENTS_MIN <= len(perf.segments) <= kn.SEGMENTS_MAX or not all(perf.segments):
        raise _Bad("bad_segments", f"{len(perf.segments)} сегментов")
    for i, segment in enumerate(perf.segments):
        _check_speech(segment, kn.SEGMENT_MAX_CHARS, f"perform.segments[{i}]")
    for name, per in (("expression_per_segment", perf.expression_per_segment),
                      ("voice_per_segment", perf.voice_per_segment)):
        if per is not None and len(per) != len(perf.segments):
            raise _Bad("bad_segments", f"{name} не по числу сегментов")
    for i, voice in enumerate(perf.voice_per_segment or ()):
        _check_speech(voice, kn.VOICE_MAX_CHARS, f"perform.voice_per_segment[{i}]")
    _check_edit(cmd.edit_pending, perf.from_segment, busy)


def _check_edit(edit_pending: bool, from_segment: int, busy: Busy) -> None:
    if from_segment < 0:
        raise _Bad("bad_segments", f"from_segment={from_segment}")
    if not edit_pending:
        if from_segment:
            raise _Bad("from_segment_without_edit", str(from_segment))
        return
    if busy.active_segment is None:
        raise _Bad("edit_without_active_perform", "править нечего: выступление не идёт")
    if from_segment < busy.active_segment + 1:
        raise _Bad("edit_started_segment", f"from_segment={from_segment}, ACTIVE={busy.active_segment}")


def _check_when(when: str, busy: Busy) -> None:
    if when not in kn.WHEN:
        raise _Bad("bad_enum", f"when={when}")
    if when == "now":
        hard = sorted(c for c in busy.classes if c in kn.ACTION_CLASSES and not kn.ACTION_CLASSES[c].interruptible)
        if hard:
            raise _Bad("now_vs_uninterruptible", ", ".join(hard))


def _expression_ok(expr: Expression) -> Tuple[Expression, bool]:
    """Проверенная экспрессия и флаг «сцена лица отброшена». Чужая эмоция/жест/earcon — команда невалидна."""
    if expr.emotion not in kn.EMOTIONS:
        raise _Bad("bad_enum", f"emotion={expr.emotion}")
    if expr.gesture is not None and expr.gesture not in kn.GESTURES:
        raise _Bad("bad_enum", f"gesture={expr.gesture}")
    if expr.earcon is not None and expr.earcon not in kn.EARCONS:
        raise _Bad("bad_enum", f"earcon={expr.earcon}")
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
# to_args — команда обратно в поля схемы (мост v1 передаёт её циклу тулов одним вызовом ``command``)
# ---------------------------------------------------------------------------------------------------------------


def to_args(cmd: Command) -> Dict[str, Any]:
    """Поля схемы команды; ``parse(to_args(c)) == c`` для разобранной команды (``dropped`` не переносится)."""
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
    return out


def _perform_args(perf: Performance) -> Dict[str, Any]:
    out: Dict[str, Any] = {"kind": perf.kind, "segments": list(perf.segments), "backing": perf.backing}
    if perf.expression_per_segment is not None:
        out["expression_per_segment"] = [_expression_args(e) for e in perf.expression_per_segment]
    if perf.voice_per_segment is not None:
        out["voice_per_segment"] = list(perf.voice_per_segment)
    if perf.from_segment:
        out["from_segment"] = perf.from_segment
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
