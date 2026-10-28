"""Команда LLM — узкий слот ответа хода (ADR-0150 §4.1; ADR-0148 «код решает, LLM говорит»).

LLM на каждом ходе отдаёт ОДНУ команду: речь, вопрос, до трёх действий, выступление, экспрессия. :func:`parse`
собирает её из ``tool_calls``, JSON или свободного текста с JSON-объектом; :func:`validate` проверяет по таблицам
:mod:`.knowledge` и JSON-схемам каталога тулов. Невалидно — :class:`Invalid` со списком нарушений (:class:`Violation`)
без вопроса человеку: ход закрывает провал (рефлекс ``failed``). :func:`explain` — нарушения словами для модели: мост режима команд возвращает их
tool-результатом, один раз на ответ (мост, решение Шифу 21.10 по #3698; ADR-0148 п.2).

Речь человеку просим не длиннее ``SPEECH_MAX_CHARS``, а проверяем по ``SPEECH_ACCEPT_MAX_CHARS`` (#4137): длиннее —
нарушение ``VIOLATION_SPEECH_TOO_LONG``; одно оно у команды (:func:`overlong_only`) — мост даёт модели ход компактнее, и
после него длинно — речь ужимает код (:func:`fit_speech`).

Косметику код выбрасывает, а не отвергает команду целиком (#4138): ``knowledge.DROPPABLE`` - единственная таблица того,
что можно выбросить (earcon не из перечня, лишний аргумент ``play_animation``, #4174: необязательный аргумент-подсказка
тула с значением не из перечня - ``auto`` или выброс, ``DROPPABLE.soft_args``); выброшенное - в ``Command.dropped``
(лог ``fields_dropped``). Нет обязательного аргумента или тула - команда невалидна, как раньше.

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
from rob_box_dialog import cast as perform_cast
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.cast import Role
from rob_box_dialog.expression import shows
from rob_box_dialog.face_scene import FIELDS as FACE_FIELDS, FaceScene, problem as face_problem
from rob_box_dialog.words import music_promises

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

    Роли (#4084): ``cast`` — герои с репликами (имя и пол), ``role_per_segment`` — чья реплика в сегменте (``""`` —
    рассказчик); их называет модель. ``voice_per_segment`` — голос каждого сегмента, его решает КОД
    (:func:`validate` → :mod:`.cast`): у роли один голос на всё выступление, песня и рэп — голосом робота; ``""`` —
    текущий голос робота. Голос робота он не меняет: это ``set_voice`` (только по явной просьбе, #3744)."""

    kind: str
    segments: Tuple[str, ...]
    backing: str = "none"
    expression_per_segment: Optional[Tuple[Expression, ...]] = None
    from_segment: int = 0
    voice_per_segment: Optional[Tuple[str, ...]] = None
    cast: Optional[Tuple[Role, ...]] = None
    role_per_segment: Optional[Tuple[str, ...]] = None


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
    """Что занято сейчас — классы действий и ACTIVE-сегмент выступления (заполняет планировщик, PR-7b) - и каталог
    голосов ролей звучащего провайдера TTS (#3959; пусто - каталог неизвестен: голоса сегментов тогда не решаются
    заново и не сверяются - так перепроверяется уже решённая команда).

    ``voice_genders`` — пол голосов каталога ``(голос, пол)``: по нему код даёт голос роли (#4084); ``cast`` — роли
    идущего выступления и их голоса ``(ключ роли, голос)``: правка выступления их не перекрашивает."""

    classes: FrozenSet[str] = frozenset()
    active_segment: Optional[int] = None
    voices: Tuple[str, ...] = ()
    voice_genders: Tuple[Tuple[str, str], ...] = ()
    cast: Tuple[Tuple[str, str], ...] = ()

    def role_pool(self) -> Tuple[Tuple[str, str], ...]:
        """Каталог ролей для :func:`.cast.assign`: голоса ``voices`` в их порядке и их пол (неизвестен — ``""``)."""
        genders = dict(self.voice_genders)
        return tuple((voice, genders.get(voice, "")) for voice in self.voices)


Result = Union[Command, Invalid]


@dataclass(frozen=True)
class Form:
    """Форма команды хода (#4021, ADR-0148 п.2: выбор сужает структура, а не правило в промпте).

    ``passed`` — модели можно «промолчать»: только при подхвате бэклога. Адресованность реплики решил код
    (``address.py``), и на адресованную реплику молчанием не ответить (решение Шифу 22.10; n813 23.10: «Робот, тишина?»
    → ``passed`` → «Хм, я задумался…»). ``voice`` — модели можно задать голос реплики (``Command.voice``): человек в
    этой реплике просил о голосе (``tools_view.voice_asked``; n109 23.10: на вопрос про звуки DeepSeek сам включил
    female-shaonv). ``sing`` — путь пения, решённый кодом до модели (#4079: id кандидата ``live:song``/``generate:…``
    от хоста, ``knowledge.SING_FORMS``): живое — только ``perform`` под синт-подложку, генерация и трек закрыты;
    генерация — ``generate_music``, петь выступлением нельзя. ``sing_over`` — музыка уже играет при живом пении («под
    этот бит зачитай рэп», n810): играющее и есть подложка, выступление поверх без второго бита и без вопроса о смене
    (``knowledge.SING_FORMS["live_over"]``). ``backing`` — модели можно заказать подложку сказке, анекдоту и стиху
    (#4152): только когда человек сам назвал музыку в этой реплике (``tools_view.music_asked``); иначе подложка у этих
    видов по умолчанию закрыта (``knowledge.PERFORM_BACKING_BY_KIND``), у песни и рэпа - как была.
    ``speech_required`` — последняя итерация хода после выполненного запроса (#4157, живой 28.10 13:40: «скажи обо мне
    пять фактов» — девять результатов памяти, а ответ последнего вызова — одно лицо без слов): данные у модели, читать
    больше нечем; ответ без слов — нарушение валидатора ``VIOLATION_SPEECH_REQUIRED`` (:func:`_check_answered`:
    до ``SPEECH_PUSHES_MAX`` повторов хода за словами, затем страховка - код говорит по данным,
    ``turn.query_answer``). Схему поле не меняет: ``required: [speech]`` спорил бы с
    ``VIOLATION_SPEECH_WITH_ACTIONS`` - слышимое действие (звук, музыка) после запроса законно без слов, а со словами -
    нарушение. Ставит его ход (``turn.answer_form``). По умолчанию всё открыто — так зовут валидатор вне хода (режим
    команд v1, тесты)."""

    passed: bool = True
    voice: bool = True
    sing: str = ""
    sing_over: bool = False
    backing: bool = True
    speech_required: bool = False

    def closed(self) -> Tuple[str, ...]:
        """Поля команды, которых в этом ходе нет (порядок ``knowledge.FORM_FIELDS``)."""
        return tuple(name for name in kn.FORM_FIELDS if not getattr(self, name))

    @property
    def sing_form(self) -> Optional[kn.SingForm]:
        """Что форма хода закрывает и требует по пути пения; не пение — ``None``."""
        return kn.SING_FORMS.get(kn.sing_family(self.sing, self.sing_over))

    def closed_tools(self) -> Tuple[str, ...]:
        """Тулы, которых в этом ходе нет по пути пения (вне среза и вне команды)."""
        form = self.sing_form
        return form.closed_tools if form is not None else ()


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
    (``knowledge.without_fields``); путь пения сужает перечни выступления (``knowledge.with_sing``)."""
    form = form or current_form()
    return kn.with_sing(kn.without_fields(schema, form.closed()), form.sing, form.sing_over)


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
    """Время лица команды (#4083): число больше нуля — как есть (пределы показа ставит ``expression.show_seconds``);
    ноль, отрицательное и ``NaN`` — время не задано (``None``: обычный ответ держит лицо временем рефлекса, а не
    показом по просьбе); не число — поле в ``dropped`` (лог ``fields_dropped``), команда не отвергается (§5.5.9:
    экспрессия команду не валит)."""
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        fix.dropped.append(f"{where}.duration_s")
        return None
    return float(value) if value > 0 else None


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
    _only(data, ("kind", "segments", "backing", "expression_per_segment", "voice_per_segment", *kn.ROLE_FIELDS),
          "perform")
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
        voice_per_segment=_names(data, "voice_per_segment"),
        cast=_cast(data, fix),
        role_per_segment=_names(data, "role_per_segment"),
    )


def _names(data: Mapping[str, Any], key: str) -> Optional[Tuple[str, ...]]:
    """Список имён по сегментам (голоса, роли); ``null`` в списке — голос робота, рассказчик (как ``""``).
    ``voice_per_segment`` модели в схеме нет: его решает код (:func:`_voice_roles`), разбор держит поле для уже решённой
    команды (мост → исполнитель)."""
    names = _get(data, key, list)
    if names is None:
        return None
    if not all(v is None or isinstance(v, str) for v in names):
        raise _Bad("bad_type", f"perform.{key}")
    return tuple((v or "").strip() for v in names)


def _cast(data: Mapping[str, Any], fix: _Repairs) -> Optional[Tuple[Role, ...]]:
    """Герои выступления (#4084): ``{name, gender}``; строка — герой без пола. Пол вне ``ROLE_GENDERS`` и лишние поля
    (``voice`` героя модель не выбирает) — в ``dropped``: голос подойдёт любой, команда не отвергается."""
    items = _get(data, "cast", list)
    if items is None:
        return None
    out: List[Role] = []
    for i, item in enumerate(items):
        item = {"name": item} if isinstance(item, str) else item
        if not isinstance(item, Mapping) or not isinstance(item.get("name"), str):
            raise _Bad("bad_type", f"perform.cast[{i}]")
        fix.dropped.extend(f"perform.cast[{i}].{key}" for key in sorted(set(item) - {"name", "gender"}))
        gender = item.get("gender") or ""
        if gender not in kn.ROLE_GENDERS:
            fix.dropped.extend([f"perform.cast[{i}].gender"] if gender else [])
            gender = ""
        out.append(Role(item["name"].strip(), gender))
    return tuple(out)


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
    же и голос реплики без просьбы о голосе отбрасываются (``dropped``); форма последней итерации после запроса
    (``Form.speech_required``): ответ без слов (одно лицо, анимация) - нарушение ``VIOLATION_SPEECH_REQUIRED``
    (:func:`_check_answered`, #4157). Невалидная сцена лица и earcon не из перечня
    отбрасываются (``dropped``), а не валят команду (§5.5.9, #4138); так же лишние поля ВЛОЖЕННЫХ объектов аргументов
    тула (``play_animation.face.emotion``, #3974) и лишние аргументы верхнего уровня тулов таблицы
    ``knowledge.DROPPABLE`` (``play_animation.animated``, #4138): без отказа команды, с записью в ``dropped``.
    Необязательный аргумент-подсказка не из перечня (``request_music.genre``, #4174) - ``auto`` или выброс, тоже с записью.
    Обязательные поля, типы и enum по-прежнему проверяет схема.
    """
    if isinstance(cmd, Invalid):
        return cmd
    form = form or current_form()
    cmd = _drop_closed_passed(_drop_unasked_voice(_fit_soft_args(_drop_nested_extras(cmd)), form), form)
    # подложка: сначала вид без просьбы о музыке теряет её (#4152), потом путь пения подгоняет остальное (#4079);
    # число лиц по сегментам (#4174) правит другое поле выступления и идёт последним, порядок с подложкой не важен
    cmd = _fit_sing_backing(_drop_default_backing(cmd, form), form)
    cmd = _fit_expression_count(cmd)
    cmd = _voice_roles(cmd, busy)
    agent = getattr(session, "agent", None) or kn.AGENTS[0]
    # тулы, закрытые путём пения, судит _check_sing своим нарушением: не «тула нет в срезе» — мост его не расширяет
    available = set(available_tools) - set(form.closed_tools())
    open_actions = tuple(a for a in cmd.actions if a.tool not in form.closed_tools())
    checks: Tuple[Callable[[], Any], ...] = (
        lambda: _check_passed(cmd, form),
        lambda: _check_answered(cmd, form),
        lambda: _check_spoken(cmd.speech, "speech"),
        lambda: _check_voice(cmd.voice, busy),
        lambda: _check_ask(cmd.ask),
        lambda: _check_actions(open_actions, available, agent),
        lambda: _check_speech_with_actions(cmd, agent),
        lambda: _check_music_promise(cmd),
        lambda: _check_perform(cmd, busy),
        lambda: _check_sing(cmd, form),
        lambda: _check_when(cmd.when, busy),
        lambda: _drop_bad_expression(cmd),
    )
    outcomes = [_outcome(check) for check in checks]
    found = tuple(v for v in outcomes if isinstance(v, Violation))
    if found:
        return Invalid(found[0].reason, found[0].detail, violations=found,
                       salvage=_salvage(replace(cmd, actions=open_actions), available, agent))
    return outcomes[-1]


def _drop_extra(schema: Mapping[str, Any], value: Any, path: str, droppable: bool) -> Tuple[Any, List[str]]:
    """Значение без лишних полей вложенных закрытых объектов (``additionalProperties: false``) и список отброшенных
    путей. ``droppable`` - выбрасываются ли лишние поля самого этого объекта: вложенные - да, верхний уровень аргументов
    тула - только по таблице ``knowledge.DROPPABLE.tool_args`` (:func:`_extra_args_droppable`), иначе лишний аргумент
    тула - по-прежнему ``bad_args``."""
    if isinstance(value, Mapping):
        props = schema.get("properties", {})
        closed = droppable and schema.get("additionalProperties") is False
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


def _extra_args_droppable(call: ToolCall) -> bool:
    """Лишние аргументы верхнего уровня этого вызова выбрасываются (#4138): тул в ``knowledge.DROPPABLE.tool_args`` и в
    вызове есть хоть один аргумент, несущий смысл (``play_animation``: ``animation`` или ``face``). Только лишнее
    поле (``{"animated": true}``) - вызов без смысла, его отвергает схема, как раньше."""
    meaning = kn.DROPPABLE.tool_args.get(call.tool, ())
    return any(name in call.args for name in meaning)


def _drop_nested_extras(cmd: Command) -> Command:
    """Аргументы действий без лишних полей, которые не меняют действие: вложенные у любого тула (#3974: «n905:
    play_animation.face.emotion: лишнее поле») и верхнего уровня у тулов таблицы (#4138: «n904:
    play_animation.animated: лишнее поле» при ``animation: sad``) - не повод отказывать команде. Отброшенное - в
    ``dropped`` (лог ``fields_dropped``)."""
    actions, notes = [], []
    for i, call in enumerate(cmd.actions):
        try:
            schema = get_tool(call.tool).parameters
        except KeyError:  # тула нет в каталоге - скажет _check_actions
            actions.append(call)
            continue
        args, dropped = _drop_extra(schema, call.args, f"actions[{i}].{call.tool}", _extra_args_droppable(call))
        actions.append(replace(call, args=MappingProxyType(args)) if dropped else call)
        notes += dropped
    if not notes:
        return cmd
    return replace(cmd, actions=tuple(actions), dropped=cmd.dropped + tuple(notes))


def _soften_args(call: ToolCall, index: int) -> Tuple[ToolCall, List[str]]:
    """Вызов, у которого мягкие аргументы (``knowledge.DROPPABLE.soft_args``) не из перечня схемы заменены на ``auto``
    (если оно в перечне) или выброшены, и записи об этом (``actions[0].request_music.genre['acoustic'->auto]``,
    ``...mood['x'->]``). Обязательный аргумент схемы и аргумент без перечня не трогаются."""
    names = kn.DROPPABLE.soft_args.get(call.tool, ())
    try:
        props = get_tool(call.tool).parameters
    except KeyError:  # тула нет в каталоге - скажет _check_actions
        return call, []
    args, notes = dict(call.args), []
    for name in names:
        allowed = props.get("properties", {}).get(name, {}).get("enum")
        if not allowed or name not in args or args[name] in allowed or name in props.get("required", ()):
            continue
        fallback = kn.SOFT_ARG_NEUTRAL if kn.SOFT_ARG_NEUTRAL in allowed else None
        notes.append(f"actions[{index}].{call.tool}.{name}[{args[name]!r}->{fallback or ''}]")
        if fallback is None:
            del args[name]
        else:
            args[name] = fallback
    return (replace(call, args=MappingProxyType(args)) if notes else call), notes


def _fit_soft_args(cmd: Command) -> Command:
    """Необязательные аргументы-подсказки тулов с неверным значением перечня (``request_music.genre='acoustic'``, n310
    28.10 #4174: «Не понял, как это сделать» на просьбу о спокойной музыке) не валят команду: ``auto`` или выброс, запись
    в ``dropped`` (лог ``fields_dropped``) - :func:`_soften_args`. Смысл действия (обязательные поля, ``action``,
    ``track_id``, ``edit_music.change`` и пр.) по-прежнему проверяет схема."""
    pairs = [_soften_args(call, i) for i, call in enumerate(cmd.actions)]
    notes = [note for _, ns in pairs for note in ns]
    if not notes:
        return cmd
    return replace(cmd, actions=tuple(call for call, _ in pairs), dropped=cmd.dropped + tuple(notes))


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


def _drop_default_backing(cmd: Command, form: Form) -> Command:
    """Подложка сказке, анекдоту и стиху без просьбы о музыке (#4152, решение Шифу 28.10: музыку включают, когда нужна
    музыка, - рэп, песня) - не выбор модели: у этих видов бита по умолчанию нет (``knowledge.PERFORM_BACKING_BY_KIND``),
    поле ``backing`` становится ``none``, оно - в ``dropped`` (лог ``fields_dropped``). Человек назвал музыку в этой
    реплике (форма хода ``Form.backing``) - подложка модели остаётся. ``none`` подложку не заказывает, поэтому играющая
    музыка выступлению не мешает и им не трогается. Правка идущего выступления судится так же, вид - из её
    ``perform.kind``. Значение вне перечня (``bad_enum``) и вид вне таблицы сюда не относятся - их судит
    :func:`_check_perform`."""
    perf = cmd.perform
    if perf is None or form.backing or perf.backing not in kn.PERFORM_BACKING_MOODS:
        return cmd
    if kn.PERFORM_BACKING_BY_KIND.get(perf.kind, True):
        return cmd
    return replace(cmd, perform=replace(perf, backing="none"), dropped=cmd.dropped + ("perform.backing",))


def _fit_sing_backing(cmd: Command, form: Form) -> Command:
    """Живое пение (#4079) — под синт-подложку: ``perform`` с подложкой вне перечня пути (``none``, не названа) получает
    первую из перечня (``knowledge.SING_FORMS``), поле — в ``dropped``; музыка уже играет (``live_over``) — ``none``:
    поверх играющего. Подложку решает код по пути, модель — только настроение."""
    sing = form.sing_form
    perf = cmd.perform
    if sing is None or not sing.backings or perf is None or cmd.edit_pending or perf.backing in sing.backings:
        return cmd
    return replace(cmd, perform=replace(perf, backing=sing.backings[0]), dropped=cmd.dropped + ("perform.backing",))


def _voice_roles(cmd: Command, busy: Busy) -> Command:
    """Голоса сегментов выступления решает код (#4084, :mod:`.cast`), не модель. Песня и рэп — голосом робота: роли и
    голоса модели — в ``dropped``. Выступление с ролями — голос роли из каталога ``busy`` (у роли один на всё
    выступление, правка идущего не перекрашивает его роли); ``voice_per_segment`` модели — в ``dropped``. Каталог
    неизвестен (``busy.voices`` пуст: перепроверка уже решённой команды, вне хода) — голоса как есть."""
    perf = cmd.perform
    if perf is None:
        return cmd
    if not perform_cast.has_roles(perf.kind):
        gone = tuple(f"perform.{name}" for name in ("voice_per_segment", *kn.ROLE_FIELDS) if getattr(perf, name))
        clean = replace(perf, voice_per_segment=None, cast=None, role_per_segment=None)
        return cmd if clean == perf else replace(cmd, perform=clean, dropped=cmd.dropped + gone)
    if not busy.voices:
        return cmd
    voices = perform_cast.segment_voices(perf.kind, perf.cast, perf.role_per_segment, busy.role_pool(), dict(busy.cast))
    if voices == perf.voice_per_segment:
        return cmd
    gone = ("perform.voice_per_segment",) if perf.voice_per_segment is not None else ()
    return replace(cmd, perform=replace(perf, voice_per_segment=voices), dropped=cmd.dropped + gone)


def _check_sing(cmd: Command, form: Form) -> None:
    """Путь пения, решённый кодом до модели (#4079, ``knowledge.SING_FORMS``): закрытые тулы не зовутся, живое пение —
    ``perform`` вида пути, генерация — ``generate_music`` без выступления-песни. Правка идущего выступления
    (``edit_pending``) — не новая просьба спеть: форма её не судит."""
    sing = form.sing_form
    if sing is None or cmd.edit_pending:
        return
    kinds = kn.sing_kinds(form.sing)
    closed = [a.tool for a in cmd.actions if a.tool in sing.closed_tools]
    if closed:
        raise _Bad(kn.VIOLATION_SING_ROUTE, f"путь {form.sing}: {', '.join(closed)} закрыт", ("perform",))
    if sing.needs_perform and cmd.perform is None:
        raise _Bad(kn.VIOLATION_SING_ROUTE, f"путь {form.sing}: нужен perform", kinds)
    if cmd.perform is not None and cmd.perform.kind not in kinds:
        raise _Bad(kn.VIOLATION_SING_ROUTE, f"путь {form.sing}: perform.kind={cmd.perform.kind}", kinds)
    if sing.needs_tool and sing.needs_tool not in {a.tool for a in cmd.actions}:
        raise _Bad(kn.VIOLATION_SING_ROUTE, f"путь {form.sing}: нужен {sing.needs_tool}", (sing.needs_tool,))


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


def bare_answer(cmd: Command) -> bool:
    """Ответ команды не слышен (#3995, #3997): нет ни речи, ни вопроса, ни выступления, ни ``passed``; все действия -
    неслышимые (``knowledge.INAUDIBLE_TOOLS``), а ответ есть - эти действия или лицо (эмоция, жест, сцена, один earcon).
    Единственное место знания, что такое «одно лицо или анимация без слов»: им судят ход (``tools_view.needs_words``:
    человек ждёт слов) и форма последней итерации (:func:`_check_answered`, #4157)."""
    if cmd.speech or cmd.ask is not None or cmd.perform is not None or cmd.passed:
        return False
    if any(a.tool not in kn.INAUDIBLE_TOOLS for a in cmd.actions):
        return False
    face = cmd.expression
    return bool(cmd.actions) or shows(face.emotion, face.gesture, face.face) or face.earcon is not None


def _check_answered(cmd: Command, form: Form) -> None:
    """Последняя итерация после выполненного запроса (``Form.speech_required``, #4157): ответ без слов - одно лицо или
    анимация - нарушение ``VIOLATION_SPEECH_REQUIRED``. Данные у модели, читать больше нечем; ход даёт ей повторы с
    подсказкой (:func:`words_only`, ``turn._push_for_words``), и без слов после них закрывает код фразой по данным
    (``turn.query_answer``). Пустая команда - нарушение
    ``empty_command`` моста; слышимые действия (звук, музыка) и действия, исход которых говорит код, ответом остаются -
    слова рядом с ними были бы ``VIOLATION_SPEECH_WITH_ACTIONS``."""
    if form.speech_required and bare_answer(cmd):
        raise _Bad(kn.VIOLATION_SPEECH_REQUIRED, "на лице или анимации без слов")


def _check_speech(text: Optional[str], limit: int, where: str) -> None:
    if text is None:
        return
    if len(text) > limit:
        raise _Bad("too_long", f"{where}: {len(text)} > {limit}")
    _check_chars(text, where)


def _check_chars(text: Optional[str], where: str) -> None:
    """В тексте нет управляющих символов (перевод строки — можно)."""
    if text is not None and any(unicodedata.category(c) == "Cc" and c != "\n" for c in text):
        raise _Bad("control_chars", where)


def _check_spoken(text: Optional[str], where: str) -> None:
    """Речь человеку (``speech``, ``ask.text``, #4137): звучит до ``SPEECH_ACCEPT_MAX_CHARS`` включительно, хоть модель
    и просим о ``SPEECH_MAX_CHARS``; длиннее — своё нарушение ``VIOLATION_SPEECH_TOO_LONG``, не ``too_long``."""
    if text is not None and len(text) > kn.SPEECH_ACCEPT_MAX_CHARS:
        raise _Bad(kn.VIOLATION_SPEECH_TOO_LONG, f"{where} {len(text)} > {kn.SPEECH_ACCEPT_MAX_CHARS}")
    _check_chars(text, where)


def overlong_only(result: Result) -> bool:
    """Команда отвергнута только за длину речи (все нарушения — ``VIOLATION_SPEECH_TOO_LONG``): ей ход компактнее
    (``knowledge.SPEECH_COMPACT_TURNS``), затем :func:`fit_speech`, а не общий повтор и заглушка (#4137)."""
    return isinstance(result, Invalid) and all(v.reason == kn.VIOLATION_SPEECH_TOO_LONG for v in result.all)


def words_only(result: Result) -> bool:
    """Команда отвергнута только за ответ без слов на последней итерации (все нарушения —
    ``VIOLATION_SPEECH_REQUIRED``, #4157): повтор за словами даёт ход (``turn._push_for_words``: он знает потолок
    хода), мост такую команду отдаёт ходу сразу, без своего повтора."""
    return isinstance(result, Invalid) and all(v.reason == kn.VIOLATION_SPEECH_REQUIRED for v in result.all)


def fit_speech(cmd: Command) -> Command:
    """Последний запас (#4137, решение Шифу 28.10): речь человеку (``speech``, ``ask.text``) длиннее
    ``SPEECH_ACCEPT_MAX_CHARS`` и после хода компактнее — первые целые предложения, пока влезают; первое само длиннее —
    по слову. Режет тот же код, что сегменты выступления (``plan.segments.cut``: строки и предложения, слово неделимо),
    второго знания о границах предложения нет; хвостовая запятая или тире обрыва по слову не звучат. Оборванное
    посреди предложения (перечень голосов, #3883: «…Голос38» звучал как полный) кончает пометка каталога
    ``knowledge.SPEECH_CUT_PHRASE`` («и так далее.»), она входит в предел. Целые предложения - без пометки. Не длиннее —
    как есть."""
    from rob_box_dialog.plan.segments import cut, ends_sentence  # noqa: PLC0415 — plan.segments сам импортирует command

    limit = kn.SPEECH_ACCEPT_MAX_CHARS

    def fit(text: Optional[str]) -> Optional[str]:
        if text is None or len(text) <= limit:
            return text
        head = cut(text, limit)[0]
        if ends_sentence(head):
            return head
        mark = phrases.text(kn.SPEECH_CUT_PHRASE)
        return f"{cut(text, limit - len(mark) - 1)[0].rstrip(' ,;:—–-')} {mark}"

    ask = None if cmd.ask is None else replace(cmd.ask, text=fit(cmd.ask.text))
    return replace(cmd, speech=fit(cmd.speech), ask=ask)


def _check_speech_with_actions(cmd: Command, agent: str) -> None:
    """``speech`` рядом с действиями не звучит (§4.1): нарушение, а не молчаливая потеря ответа. Вместе с ``perform``
    его уже ловит ``perform_with_speech``; у телеграм-агента ``speech`` - текст в чат, он законен."""
    if cmd.speech and loud_actions(cmd) and cmd.perform is None and agent != kn.TEXT_CHANNEL_AGENT:
        raise _Bad(kn.VIOLATION_SPEECH_WITH_ACTIONS)


def unkept_promises(cmd: Command) -> Tuple[str, ...]:
    """Виды обещаний действия с музыкой в ``speech`` (``words.music_promises``), которых команда не исполняет: нет тула
    ``knowledge.MUSIC_PROMISES[вид].tools`` среди ``actions`` и (для вида с ``perform``) выступления. Обещание с тулом
    держит команда - о результате скажет код."""
    called = {a.tool for a in cmd.actions}
    return tuple(kind for kind in music_promises(cmd.speech or "")
                 if not called & set(kn.MUSIC_PROMISES[kind].tools)
                 and not (kn.MUSIC_PROMISES[kind].perform and cmd.perform is not None))


def _check_music_promise(cmd: Command) -> None:
    """Речь обещает действие с музыкой или громкостью («сделаю потише», «включу музыку»), а тула, что его исполняет, в
    команде нет (#4020 п.6, ADR-0148): слова без действия - ложь о состоянии (n709: музыку уже остановил код, а модель
    «сделаю потише»). Нарушение ``VIOLATION_MUSIC_PROMISE`` - повтор с подсказкой; прошедшее («музыку выключил») не
    обещание."""
    broken = unkept_promises(cmd)
    if broken:
        raise _Bad(kn.VIOLATION_MUSIC_PROMISE, f"speech обещает {', '.join(broken)} без тула")


def _check_ask(ask: Optional[Ask]) -> None:
    if ask is None:
        return
    if ask.expects not in kn.ASK_EXPECTS:
        raise _Bad("bad_enum", f"ask.expects={ask.expects}", kn.ASK_EXPECTS)
    if (ask.text is None) == (ask.key is None):
        raise _Bad("bad_ask", "у вопроса ровно одно из text/key")
    _check_spoken(ask.text, "ask.text")


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
    # поля, которые называет модель, - первыми: голоса сегментов решает код из role_per_segment (_voice_roles), и при
    # неверной длине ролей его voice_per_segment тоже не той длины - но модель такого поля не видит (#4112)
    for name in ("role_per_segment", "voice_per_segment"):
        per = getattr(perf, name)
        if per is not None and len(per) != len(perf.segments):
            raise _Bad("bad_segments", f"{name} не по числу сегментов")
    _check_roles(perf, busy)
    _check_edit(cmd.edit_pending, busy)


def _fit_expression_count(cmd: Command) -> Command:
    """Лиц по сегментам - ровно столько, сколько сегментов (#4174, n810 28.10: рэп с 4 куплетами и другим числом лиц
    отвергнут целиком дважды). Лицо - косметика (ADR-0148, как #4138): лишние лица отрезаются, недостающие
    дополняются последним лицом, пустой список - лиц нет (``None``); поправка - в ``dropped`` (лог ``fields_dropped``:
    ``perform.expression_per_segment[3->4]``), выступление исполняется. Роли и голоса сегментов смысл несут (чья
    реплика) - их длина по-прежнему нарушение ``bad_segments``. Число самих сегментов судит :func:`_check_segments`."""
    perf = cmd.perform
    faces = None if perf is None else perf.expression_per_segment
    if faces is None or len(faces) == len(perf.segments):
        return cmd
    count = len(perf.segments)
    fitted = faces[:count] + faces[-1:] * (count - len(faces)) if faces else ()
    note = f"perform.expression_per_segment[{len(faces)}->{len(fitted)}]"
    perf = replace(perf, expression_per_segment=fitted or None)
    return replace(cmd, perform=perf, dropped=cmd.dropped + (note,))


def _check_roles(perf: Performance, busy: Busy) -> None:
    """Роли выступления (#4084, :func:`.cast.problem`): у сказки ``cast`` обязателен (#3959: без ролей она звучала одним
    голосом, а валидатор молчал), реплики — только героев из ``cast`` или рассказчика, героев с репликами без голоса не
    больше, чем свободных голосов ролей (голоса героев идущего выступления, ``busy.cast``, заняты). Уже решённая
    команда, перепроверяемая без каталога (голоса есть, ``busy.voices`` пуст), — не сказка без ролей."""
    for i, name in enumerate((*(r.name for r in perf.cast or ()), *(perf.role_per_segment or ()))):
        _check_speech(name, kn.ROLE_NAME_MAX_CHARS, f"perform.role[{i}]")
    for i, voice in enumerate(perf.voice_per_segment or ()):
        _check_speech(voice, kn.VOICE_MAX_CHARS, f"perform.voice_per_segment[{i}]")
    if not perf.segments or (perf.voice_per_segment is not None and not busy.voices):
        return
    found = perform_cast.problem(perf.kind, len(perf.segments), perf.cast, perf.role_per_segment, busy.role_pool(),
                                 dict(busy.cast))
    if found is not None:
        raise _Bad(*found)


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


#: Перечни полей экспрессии с enum (имя поля -> допустимое): значение не из перечня - нарушение ``bad_enum``, кроме
#: полей ``knowledge.DROPPABLE.expression`` (#4138): те выбрасываются.
_EXPRESSION_ENUMS: Mapping[str, Collection[str]] = MappingProxyType(
    {"emotion": kn.EMOTIONS, "gesture": kn.GESTURES, "earcon": kn.EARCONS})


def _expression_ok(expr: Expression) -> Tuple[Expression, Tuple[str, ...]]:
    """Проверенная экспрессия и имена отброшенных полей. Чужая эмоция/жест - команда невалидна; чужой earcon (``none``,
    n904 #4138: «покажи, как тебе грустно» отвергнута целиком) и невалидная сцена лица (§5.5.9) - поле выбрасывается."""
    dropped: List[str] = []
    for name, allowed in _EXPRESSION_ENUMS.items():
        value = getattr(expr, name)
        if value is None or value in allowed:
            continue
        if name not in kn.DROPPABLE.expression:
            raise _Bad("bad_enum", f"{name}={value}", tuple(allowed))
        expr, dropped = replace(expr, **{name: None}), [*dropped, name]
    if expr.face is not None and not _face_ok(expr.face):
        expr, dropped = replace(expr, face=None), [*dropped, "face"]
    return expr, tuple(dropped)


def _face_ok(face: Any) -> bool:
    """Сцена лица годится, если ``face_scene.problem`` ничего не нашёл."""
    return face_problem(face) == ""


def _drop_bad_expression(cmd: Command) -> Command:
    """Экспрессия команды и экспрессии сегментов выступления без отброшенных полей (:func:`_expression_ok`); отброшенное
    - в ``dropped`` (лог ``fields_dropped``): ``expression.face``, ``expression.earcon``,
    ``perform.expression_per_segment[i].face``."""
    expression, gone = _expression_ok(cmd.expression)
    notes = [f"expression.{name}" for name in gone]
    perf = cmd.perform
    if perf is not None and perf.expression_per_segment is not None:
        checked = [_expression_ok(e) for e in perf.expression_per_segment]
        notes += [f"perform.expression_per_segment[{i}].{name}"
                  for i, (_, names) in enumerate(checked) for name in names]
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
                    "список role_per_segment — по числу сегментов",
    perform_cast.TALE_WITHOUT_CAST: "сказка: perform.cast обязателен — герои с репликами (name и gender; героев нет — "
                                    "[]) и role_per_segment — чья реплика в каждом сегменте (\"\" — рассказчик); "
                                    "сегмент = реплика одного говорящего, повествование и каждая реплика героя — "
                                    "отдельные сегменты; голоса героям даст код",
    perform_cast.BAD_CAST: "роли выступления: в role_per_segment — только name героя из cast или \"\" (рассказчик), по "
                           "одной роли на сегмент, по числу сегментов; героев с репликами не больше, чем свободных "
                           "голосов (голоса героев идущего выступления заняты)",
    "too_long": "слишком длинно",
    kn.VIOLATION_SPEECH_TOO_LONG: f"слишком длинно для речи: пришли ту же команду, а ответ сделай компактнее — не "
                                  f"длиннее {kn.SPEECH_MAX_CHARS} символов, тот же смысл по тем же данным",
    "control_chars": "управляющие символы в тексте",
    "passed_with_content": "passed — только без речи и действий",
    kn.VIOLATION_PASSED_ADDRESSED: "реплика адресована тебе — молчать нельзя, поля passed в этом ходе нет: ответь "
                                   "словами в speech (не понял — переспроси в ask)",
    kn.VIOLATION_SPEECH_REQUIRED: f"молчать нельзя: это последний ответ хода, данные запроса уже выше в окне, читать "
                                  f"больше нечем; лицо и анимация слов не заменяют — ответь словами по этим данным в "
                                  f"speech, коротко, не длиннее {kn.SPEECH_MAX_CHARS} символов",
    kn.VIOLATION_MUSIC_PROMISE: kn.MUSIC_PROMISE_HINT,
    "bad_ask": "у ask нужен text",
    "bad_tool_call": "вызов — объект с name и arguments",
    "class_not_for_agent": "этот класс действий тебе недоступен",
    "edit_without_perform": "edit_pending — только вместе с perform",
    "edit_without_active_perform": "править нечего: выступление не идёт, убери edit_pending",
    "now_vs_uninterruptible": "when=now нельзя: занятое действие не прерывается, нужен after_current",
    kn.VIOLATION_SING_ROUTE: "путь пения уже решён кодом: live — спой сам в perform (вид из перечня, бит ставит код), "
                             "генерацию и трек не зови; generate — позови generate_music, песню выступлением не пой; "
                             "edit — музыка играет и её правят: позови edit_music, новый трек, сет и генерацию не "
                             "заказывай",
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
    if perf.cast is not None:
        out["cast"] = [{"name": r.name, **({"gender": r.gender} if r.gender else {})} for r in perf.cast]
    if perf.role_per_segment is not None:
        out["role_per_segment"] = list(perf.role_per_segment)
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
