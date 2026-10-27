"""Вопрос тула как окно ответа (#4062; ADR-0148: решает код; ADR-0150 §7.2: кадр сессии с TTL).

Тул может ответить не провалом, а вопросом человеку (``knowledge.TOOL_QUESTIONS``): ``music_library delete`` без
``confirm`` → ``rejected{needs_confirmation}`` «Удалить «X»? Скажи «да, удаляй».». Ход, получивший такой исход,
кладёт в сессию кадр ``TOOL_ASK_SCENE`` — окно ответа: какой тул спросил, по какой причине, с какими аргументами,
срок ``TOOL_ASK_TTL_S`` (:func:`note`). Следующая адресованная реплика:

* ответ целиком по закрытой грамматике (:func:`answer`): «да», «да, удаляй», «удаляй эту песню» — подтверждённый вызов
  (аргументы вопроса + ``ToolQuestion.confirm``, :meth:`Asked.confirmed`) исполняет КОД, без LLM, а фраза об исходе —
  из результата тула; «нет», «не надо», «отмена» — окно снято, фраза кода ``ToolQuestion.declined``. Ложь об успехе на
  этом пути невозможна: слов модели в нём нет (живой робот 27.10 14:47: «Удаляю все четыре…» без вызова тула);
* другие слова («да, удали оба») — ход к LLM, тул вопроса открыт в срезе (:func:`asked_tools` →
  ``tools_view.available_tools``) без круга ``tool_not_available → widen_for``.

Окно снимают: ответ, новый вопрос (заменяет прежний), исход того же вызова, не вопрос (модель подтвердила сама), срок,
новая сессия (``Session.reset``). Чистый модуль: часы — у сессии.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, FrozenSet, List, Mapping, Optional, Sequence, Tuple

from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import ToolCall
from rob_box_dialog.respond import ActionResult, asks
from rob_box_dialog.session import Session
from rob_box_dialog.words import strip_wake_word

_TOKEN = re.compile(r"\w+")
#: Поля данных кадра окна (``SceneFrame.payload``; ``/dialog/session`` отдаёт их JSON-ом).
_TOOL, _REASON, _ARGS = "tool", "reason", "args"


def _norm(word: str) -> str:
    return word.lower().replace("ё", "е")


def _forms(words: Sequence[str]) -> FrozenSet[str]:
    return frozenset(_norm(w) for w in words)


_YES, _NO, _FILLER = _forms(kn.TOOL_ANSWER_YES), _forms(kn.TOOL_ANSWER_NO), _forms(kn.TOOL_ANSWER_FILLER)


@dataclass(frozen=True)
class Asked:
    """Открытый вопрос тула: тул, причина отказа (ключ ``knowledge.TOOL_QUESTIONS``) и аргументы спросившего вызова
    (без полей ``ToolQuestion.confirm``)."""

    tool: str
    reason: str
    args: Mapping[str, Any]

    @property
    def question(self) -> kn.ToolQuestion:
        return kn.TOOL_QUESTIONS[(self.tool, self.reason)]

    def confirmed(self) -> ToolCall:
        """Подтверждённый вызов: аргументы вопроса и ``ToolQuestion.confirm`` поверх (``confirm=true``)."""
        return ToolCall(self.tool, MappingProxyType({**self.args, **self.question.confirm}))

    def same_call(self, call: ToolCall) -> bool:
        """Вызов о том же: тот же тул и все аргументы вопроса (``confirm`` сверху не мешает)."""
        return call.tool == self.tool and all(call.args.get(k) == v for k, v in self.args.items())


@dataclass(frozen=True)
class Reply:
    """Ответ человека на открытый вопрос тула: ``yes`` — согласие, иначе отказ."""

    asked: Asked
    yes: bool


def pending(session: Session) -> Optional[Asked]:
    """Открытый вопрос тула или ``None``: кадра нет, срок вышел (по часам сессии — кадр снимает ``Session.expire``
    хоста раз в секунду, а срок здесь проверяется сам) или причины нет в ``TOOL_QUESTIONS``."""
    frame = session.snapshot.top(kn.TOOL_ASK_SCENE)
    if frame is None or (frame.ttl_until is not None and frame.ttl_until <= session.now()):
        return None
    key = (str(frame.payload.get(_TOOL)), str(frame.payload.get(_REASON)))
    if key not in kn.TOOL_QUESTIONS:
        return None
    return Asked(key[0], key[1], MappingProxyType(dict(frame.payload.get(_ARGS) or {})))


def asked_tools(session: Session) -> Tuple[str, ...]:
    """Тулы открытого вопроса — для среза хода (``tools_view.available_tools(asked=...)``); вопроса нет — ``()``."""
    asked = pending(session)
    return (asked.tool,) if asked is not None else ()


def note(session: Session, actions: Sequence[ToolCall], results: Sequence[ActionResult]) -> Optional[Asked]:
    """После исполнения действий хода (пары вызов-исход по порядку: исполнитель отдаёт результаты в порядке команды):
    вопрос тула (``respond.asks``) открывает окно, прежнее заменяя; исход того же вызова, не вопрос (тул удалил или
    отказал по другой причине), окно снимает. Возвращает открытый этим ходом вопрос или ``None``."""
    opened: Optional[Asked] = None
    for call, result in zip(actions, results):
        if asks(result):
            opened = _open(session, call, result)
            continue
        current = pending(session)
        if current is not None and current.same_call(call):
            session.pop_scene(kn.TOOL_ASK_SCENE)
    return opened


def _open(session: Session, call: ToolCall, result: ActionResult) -> Asked:
    question = kn.TOOL_QUESTIONS[(result.tool, str(result.reason))]
    args = {k: v for k, v in call.args.items() if k not in question.confirm}
    session.pop_scene(kn.TOOL_ASK_SCENE)
    session.push_scene(kn.TOOL_ASK_SCENE, "code", ttl_s=kn.TOOL_ASK_TTL_S,
                       payload={_TOOL: result.tool, _REASON: str(result.reason), _ARGS: args})
    return Asked(result.tool, str(result.reason), MappingProxyType(args))


def close(session: Session) -> None:
    """Снять окно (ответ получен)."""
    session.pop_scene(kn.TOOL_ASK_SCENE)


def answer(text: str, question: kn.ToolQuestion) -> Optional[bool]:
    """Ответ на вопрос тула по закрытой грамматике (``knowledge.TOOL_ANSWER_*``): реплика без вейка целиком из слов
    «да»/«нет», глагола и предмета вопроса и служебных. Есть «нет» — ``False``; иначе «да» или глагол — ``True``;
    лишнее слово или ни «да», ни глагола — ``None`` (не ответ: решает LLM)."""
    kinds = [_kind(_norm(w), question) for w in _TOKEN.findall(strip_wake_word(text or ""))]
    if not kinds or None in kinds:
        return None
    if "no" in kinds:
        return False
    return True if "yes" in kinds else None


def _kind(word: str, question: kn.ToolQuestion) -> Optional[str]:
    """Роль слова в ответе: ``no``, ``yes`` (согласие или глагол действия), ``filler`` (предмет, служебное) или
    ``None`` (слово не из ответа)."""
    if word in _NO:
        return "no"
    if word in _YES or _starts(word, question.verbs):
        return "yes"
    if word in _FILLER or _starts(word, question.objects):
        return "filler"
    return None


def _starts(word: str, stems: Sequence[str]) -> bool:
    return any(word.startswith(_norm(stem)) for stem in stems)


def reply(session: Session, text: str) -> Optional[Reply]:
    """Ответ человека на открытый вопрос тула (:func:`pending`, :func:`answer`) или ``None``."""
    asked = pending(session)
    if asked is None:
        return None
    verdict = answer(text, asked.question)
    return Reply(asked, verdict) if verdict is not None else None


def describe(asked: Asked) -> str:
    """Строка лога окна: тул, причина, аргументы."""
    args: List[str] = [f"{k}={v}" for k, v in asked.args.items()]
    return f"{asked.tool} {asked.reason} {' '.join(args)}".strip()


__all__ = ["Asked", "Reply", "answer", "asked_tools", "close", "describe", "note", "pending", "reply"]
