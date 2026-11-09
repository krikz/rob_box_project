"""Вереница действий (ADR-0165, #4403): команда модели → шаги, оценка времени и тип задачи, фразы плана и итога.

Чистый модуль без портов и часов. Знание (классы в веренице, лимиты типов задач, глаголы фраз) - в ``knowledge``
(``CHAIN_*``), тексты - ``phrases/ru.yaml`` (``chain.*``). Этап 2 (§13): ход вереницу ещё не исполняет; из этого
модуля читает только валидатор команды (:func:`mixed` → нарушение ``chain_mixed``). Раннер шагов, фраза плана в
ходе и итог по исходам - этап 3.

Имя не ``chain``: «цепочка» в коде - цепочка провайдеров LLM (``status_feed.describe_chain``).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Dict, List, Optional, Sequence, Tuple

from rob_box_core.tool_catalog import get_tool
from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.words import done_claims

if TYPE_CHECKING:  # command импортирует этот модуль (валидатор): типы - только для проверки
    from rob_box_dialog.command import Command, ToolCall

#: Исход шага «сделано»; остальные исходы исполнителя (``rejected``, ``timeout``, ``cancelled``) - «не сделано».
_DONE = "done"


@dataclass(frozen=True)
class Step:
    """Шаг вереницы: вызов модели, его класс (``knowledge.call_class``) и ожидаемая длительность, с."""

    call: "ToolCall"
    cls: str
    expected_s: float


@dataclass(frozen=True)
class Chain:
    """Шаги одной команды в порядке модели и тип задачи (``knowledge.CHAIN_TASKS``)."""

    steps: Tuple[Step, ...]
    task: str

    @property
    def estimate_s(self) -> float:
        """Оценка времени - сумма ожидаемых длительностей шагов (ADR-0165 §4)."""
        return sum(step.expected_s for step in self.steps)

    @property
    def long(self) -> bool:
        """Вереница со фразой плана: дольше ``CHAIN_PLAN_AFTER_S`` (решение Шифу В1). Короче - короткая команда."""
        return self.estimate_s > kn.CHAIN_PLAN_AFTER_S

    @property
    def trips(self) -> int:
        return _trips(self.steps)


@dataclass(frozen=True)
class StepOutcome:
    """Исход шага для итога: статус исполнителя или ``CHAIN_STEP_SKIPPED`` (шаг не начинался); ``note`` - фраза
    владельца о причине провала (``respond``), её итог говорит после «не сделал»."""

    step: Step
    status: str
    note: str = ""


def _trips(steps: Sequence[Step]) -> int:
    """Сколько шагов везут робота к цели (``CHAIN_TRIP_TOOLS``): по ним маршрут отличается от поручения."""
    return sum(1 for step in steps if step.call.tool in kn.CHAIN_TRIP_TOOLS)


def step_label(call: "ToolCall") -> str:
    """Чем шаг назвать человеку: первое непустое строковое значение аргументов не из перечня схемы тула (имя точки,
    слова запроса музыки; ``intent=track`` и ``action=louder`` - коды, не слова) и не длиннее
    ``CHAIN_LABEL_MAX_CHARS`` (текст озвучки целиком - не имя). Нет такого - ``""``: шаг назовёт слово класса."""
    props = get_tool(call.tool).parameters.get("properties", {})
    for name, value in call.args.items():
        text = value.strip() if isinstance(value, str) else ""
        if text and "enum" not in props.get(name, {}) and len(text) <= kn.CHAIN_LABEL_MAX_CHARS:
            return text
    return ""


def expected_s(cls: str, call: "ToolCall") -> float:
    """Ожидаемая длительность шага: замер ``CHAIN_EXPECTED_S``, до замера - срок класса (верхняя граница). У класса без
    срока (озвучка: срок задаёт длина речи) - текст вызова на скорость речи ``SPEECH_CHARS_PER_S_GUESS``."""
    if cls in kn.CHAIN_EXPECTED_S:
        return kn.CHAIN_EXPECTED_S[cls]
    timeout = kn.ACTION_CLASSES[cls].timeout_s
    if timeout is not None:
        return float(timeout)
    chars = sum(len(v) for v in call.args.values() if isinstance(v, str))
    return chars / kn.SPEECH_CHARS_PER_S_GUESS


def task_of(trips: int) -> str:
    """Тип задачи по числу поездок: в лимите поручения - поручение, больше - маршрут (ADR-0165 §5)."""
    errand = kn.CHAIN_TASKS[kn.CHAIN_ERRAND]
    return kn.CHAIN_ERRAND if errand.trips_max is None or trips <= errand.trips_max else kn.CHAIN_ROUTE


def _known(actions: Sequence["ToolCall"]) -> Tuple["ToolCall", ...]:
    """Вызовы тулов из таблицы классов; тул вне неё судит валидатор своим нарушением (``unknown_tool``)."""
    return tuple(call for call in actions if call.tool in kn.TOOL_CLASSES)


def chain_of(actions: Sequence["ToolCall"]) -> Chain:
    """Шаги команды: действия классов-шагов (``ChainClass.step``) в порядке модели; ``query``, знакомство и
    выступление шагами не бывают."""
    steps = []
    for call in _known(actions):
        cls = kn.call_class(call.tool, call.args)
        if kn.CHAIN_CLASSES[cls].step:
            steps.append(Step(call, cls, expected_s(cls, call)))
    return Chain(tuple(steps), task_of(_trips(steps)))


def over_limit(chain: Chain) -> bool:
    """Вереница за лимитом своего типа - не начинается (код говорит ``CHAIN_TOO_LONG_PHRASE``, ADR-0165 §5)."""
    task = kn.CHAIN_TASKS[chain.task]
    return (
        len(chain.steps) > task.steps_max
        or (task.trips_max is not None and chain.trips > task.trips_max)
        or (task.term_s is not None and chain.estimate_s > task.term_s)
    )


def mixed(cmd: "Command") -> Tuple[str, ...]:
    """Классы команды, которым рядом с длинной вереницей нельзя (``ChainClass.alongside``: знакомство, выступление).
    Вереница короткая или из одного шага (``CHAIN_MIXED_MIN_STEPS``) - ``()``: команда идёт как сейчас (ADR-0165 §4.1,
    нарушение ``chain_mixed``). Один шаг по оценке сроком класса бывает «долгим» (звук - 10 с), а «звук и шутка»
    (``play_sound`` + ``perform``) - законная команда живого прогона."""
    classes = [kn.call_class(call.tool, call.args) for call in _known(cmd.actions)]
    if cmd.perform is not None:
        classes.append("action.perform")
    banned = tuple(dict.fromkeys(cls for cls in classes if not kn.CHAIN_CLASSES[cls].alongside))
    chain = chain_of(cmd.actions)
    if not banned or len(chain.steps) < kn.CHAIN_MIXED_MIN_STEPS or not chain.long:
        return ()
    return banned


def plan_claims(speech: Optional[str]) -> Tuple[str, ...]:
    """Что речь плана утверждает сделанным (``words.done_claims``). До первого шага сделанного нет: любое утверждение -
    неправда, даже если тул в команде есть (ADR-0165 §3, §9.1)."""
    return done_claims(speech or "")


def _item(step: Step) -> str:
    return step_label(step.call) or kn.CHAIN_CLASS_NOUNS[step.cls]


def _grouped(steps: Sequence[Step], key: str) -> List[str]:
    """Фразы шагов по глаголам (``CHAIN_VERBS``) в порядке первого появления: «Удаляю: хил-один, хил-два.»."""
    groups: Dict[str, List[str]] = {}
    for step in steps:
        verb = kn.CHAIN_VERBS.get(step.call.tool, kn.CHAIN_VERB_DEFAULT)
        groups.setdefault(getattr(verb, key), []).append(_item(step))
    return [phrases.text(phrase, items=", ".join(dict.fromkeys(items))) for phrase, items in groups.items()]


def code_plan(chain: Chain) -> str:
    """Фраза плана кода из действий - запасная при плане модели (ADR-0165 §3): о будущем, без слов о сделанном."""
    return " ".join(_grouped(chain.steps, "plan"))


def plan(chain: Chain, speech: Optional[str]) -> Tuple[str, str]:
    """Кто говорит план и что: ``("model", речь)`` - речь команды, если план модели (``CHAIN_PLAN_BY``), она не пуста и
    ничего не утверждает сделанным (:func:`plan_claims`); иначе ``("code", фраза кода)``."""
    text = (speech or "").strip()
    if kn.CHAIN_PLAN_BY == "model" and text and not plan_claims(text):
        return "model", text
    return "code", code_plan(chain)


def _head(outcomes: Sequence[StepOutcome], stopped: bool, timed_out: bool) -> Optional[str]:
    if stopped:
        return kn.CHAIN_STOPPED_PHRASE
    if timed_out:
        return kn.CHAIN_TIMEOUT_PHRASE
    return kn.CHAIN_DONE_PHRASE if all(o.status == _DONE for o in outcomes) else None


def _skipped_phrase(outcomes: Sequence[StepOutcome]) -> str:
    """Остаток после несостоявшейся поездки - «Дальше не еду», иначе (стоп, срок) - «Не стал»."""
    motion_failed = any(
        o.status not in (_DONE, kn.CHAIN_STEP_SKIPPED) and kn.CHAIN_CLASSES[o.step.cls].stops_rest for o in outcomes
    )
    return kn.CHAIN_MOTION_FAILED_PHRASE if motion_failed else kn.CHAIN_SKIPPED_PHRASE


def summary(outcomes: Sequence[StepOutcome], stopped: bool = False, timed_out: bool = False) -> str:
    """Итог вереницы кодом по исходам шагов (ADR-0165 §7, ADR-0148): начало по исходу, сделанные шаги, не сделанные и
    причины их владельцев, не начатые. Одна реплика."""
    head = _head(outcomes, stopped, timed_out)
    done = [o.step for o in outcomes if o.status == _DONE]
    skipped = [o.step for o in outcomes if o.status == kn.CHAIN_STEP_SKIPPED]
    missed = [o for o in outcomes if o.status not in (_DONE, kn.CHAIN_STEP_SKIPPED)]
    parts = [phrases.text(head)] if head else []
    parts += _grouped(done, "did")
    parts += _grouped([o.step for o in missed], "missed")
    parts += list(dict.fromkeys(o.note for o in missed if o.note))
    if skipped:
        parts.append(phrases.text(_skipped_phrase(outcomes), items=", ".join(dict.fromkeys(map(_item, skipped)))))
    return " ".join(parts)


__all__ = [
    "Chain",
    "Step",
    "StepOutcome",
    "chain_of",
    "code_plan",
    "expected_s",
    "mixed",
    "over_limit",
    "plan",
    "plan_claims",
    "step_label",
    "summary",
    "task_of",
]
