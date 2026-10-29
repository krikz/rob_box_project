"""Обращение к неизвестному собеседнику чужим именем (ADR-0148: решает код, не промпт).

Биометрия сказала «собеседник неизвестен» (``name_state`` не ``confirmed``), а модель ответила «Понял, Саша.» - имя
из истории разговора, где до этого говорили Саша и Борис. Имя человеку даёт только подтверждённый профиль голоса
(``speaker_adapter.speaker_block``), поэтому обращение к нему по ЛЮБОМУ имени, кроме названного им самим в этой
реплике, - выдумка: ``VIOLATION_FOREIGN_NAME``.

Знание о том, что считается обращением, живёт здесь одно (:func:`addressed`, :func:`without_address`); имена сверяются
по словам и падежам (``words.name_key``), а не подстрокой. Имя в речи О третьем лице («Саша просил передать…», «Это
сказал Борис.») - не обращение и не трогается: обращение - имя в начале фразы перед запятой или «!» («Саша, понял.»),
после запятой перед знаком конца («Понял, Саша.», «Привет, Борис, как дела?») или после слова-приветствия
(``ADDRESS_LEAD_WORDS``: «Привет Борис!»).
"""

from __future__ import annotations

import re
from dataclasses import replace
from typing import Any, Collection, FrozenSet, Iterable, List, Optional, Tuple

from rob_box_core.speaker_wire import NAME_CONFIRMED
from rob_box_dialog import knowledge as kn
from rob_box_dialog.command import Ask, Command
from rob_box_dialog.words import name_key, name_tokens

_WORD = re.compile(r"[\w-]+")
#: Конец обращения: запятая или знак конца фразы (тире и двоеточие делают имя подлежащим: «Борис - молодец»).
_END = ",.!?…;"
#: Конец фразы: после него следующее слово - начало предложения.
_SENTENCE_END = ".!?…"
#: Предел шагов вычистки: фраза не бесконечна, а цикл «найди - вырежи» обязан кончиться.
_STRIP_STEPS = 8


def name_keys(names: Iterable[Any]) -> FrozenSet[str]:
    """Ключи имён для сверки (``words.name_key``: регистр, ё, падеж). Метка из нескольких слов («Иван Петров») - ключ
    каждого слова; метка «незнакомый голос» и пустые - не имена."""
    keys = set()
    for name in names:
        label = str(name or "").strip()
        if not label or label.lower() == kn.UNKNOWN_SPEAKER_LABEL:
            continue
        keys.update(key for key in map(name_key, name_tokens(label)) if key)
    return frozenset(keys)


def unconfirmed(speaker: Any) -> bool:
    """Собеседник есть в снимке, а его имя не подтверждено: ему имени не дают (``None`` - снимка собеседника нет, как у
    канала без биометрии: о нём код ничего не утверждает)."""
    return speaker is not None and getattr(speaker, "name_state", None) != NAME_CONFIRMED


def _bounds(text: str, match: "re.Match[str]") -> Optional[Tuple[int, int]]:
    """Границы вырезки обращения, чьё имя - ``match``, или ``None``: это рассказ о человеке, не обращение к нему."""
    before, after = text[:match.start()].rstrip(), text[match.end():].lstrip()
    ends = not after or after[0] in _END
    if not ends:
        return None
    if before.endswith(","):  # «Понял, Саша.»: режем запятую вместе с именем
        return len(before) - 1, match.end()
    lead = name_tokens(before)[-1:] if before and before[-1] not in _SENTENCE_END else ()
    if lead and lead[0] in kn.ADDRESS_LEAD_WORDS:  # «Привет Борис!»
        return len(before), match.end()
    sentence_start = not before or before[-1] in _SENTENCE_END
    if sentence_start and after[:1] in (",", "!"):  # «Саша, понял.» - имя и знак за ним
        return match.start(), len(text) - len(after) + 1
    return None


def _spans(text: str, keys: Collection[str]) -> List[Tuple[int, int, str]]:
    """Обращения в тексте: ``(от, до, имя как написано)`` в порядке следования."""
    found = []
    for match in _WORD.finditer(text or ""):
        if name_key(match.group()) in keys:
            bounds = _bounds(text, match)
            if bounds is not None:
                found.append((*bounds, match.group()))
    return found


def addressed(text: Optional[str], keys: Collection[str]) -> Tuple[str, ...]:
    """Имена из ``keys`` (:func:`name_keys`), которыми ``text`` обращается к собеседнику, как они написаны."""
    return tuple(name for _from, _to, name in _spans(text or "", keys))


def without_address(text: str, keys: Collection[str]) -> str:
    """``text`` без обращений по именам из ``keys``: «Понял, Саша.» → «Понял.», «Привет, Борис, как дела?» → «Привет,
    как дела?», «Саша, понял.» → «Понял.». Фраза остаётся грамотной: пробелы и запятые подтянуты, первая буква
    предложения заглавная. Остались одни знаки - пусто."""
    for _step in range(_STRIP_STEPS):
        spans = _spans(text, keys)
        if not spans:
            break
        start, end, _name = spans[0]
        text = _joined(text[:start], text[end:])
    return text if any(c.isalnum() for c in text) else ""


def _joined(head: str, tail: str) -> str:
    """Две половины фразы после вырезки: знак конца встаёт к слову, начало предложения - с заглавной."""
    head, tail = head.rstrip(), tail.lstrip()
    if not tail:
        return head
    if tail[0] in _END:
        return head + tail
    if not head or head[-1] in _SENTENCE_END:
        tail = tail[:1].upper() + tail[1:]
    return f"{head} {tail}" if head else tail


def _clean(text: Optional[str], keys: Collection[str]) -> Optional[str]:
    return text if text is None else (without_address(text, keys) or None)


def spoken(cmd: Command) -> Tuple[str, ...]:
    """Слова модели, которые звучат человеку обращением: ``speech`` и вопрос ``ask`` (сегменты выступления - роли
    сказки, у них свои имена, не обращение)."""
    return tuple(t for t in (cmd.speech, cmd.ask.text if cmd.ask is not None else None) if t)


def offending(cmd: Command, keys: Collection[str]) -> Tuple[str, ...]:
    """Имена, которыми слова команды обращаются к собеседнику (пусто - нарушения нет)."""
    return tuple(name for text in spoken(cmd) for name in addressed(text, keys))


def unnamed(cmd: Command, keys: Collection[str]) -> Command:
    """Команда без обращений по именам в ``speech`` и ``ask.text``; остальное не тронуто. Речь, от которой ничего не
    осталось, - ``None`` (дальше решает ``words_owed``: ответ словами ждали); так же вопрос."""
    ask = cmd.ask
    if ask is not None and ask.text:
        text = _clean(ask.text, keys)
        ask = Ask(ask.expects, text, ask.key) if text else None
    return replace(cmd, speech=_clean(cmd.speech, keys), ask=ask)


__all__ = ["addressed", "name_keys", "offending", "spoken", "unconfirmed", "unnamed", "without_address"]
