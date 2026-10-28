"""Инструмент в заказе музыки (ADR-0157 §2.4, §3.1): слово человека → ключ ``knowledge.INSTRUMENTS`` → синт лида.

«Сыграй Баха на балалайке» (прогон 09.10, §1.1): слово «балалайке» было вне поиска названия, заказ уходил к LLM, и
она уводила его в сет. Здесь слот инструмента выделяет код по закрытой таблице знания (``knowledge.INSTRUMENTS``),
без регексов (мораторий #3132): формы слова = основа + падежное окончание, опечатка STT — одна правка
(:func:`within_one`). Чем играть, когда синта нет («балалайка»), решает тоже код (:func:`lead_choice`): первый
сосед той же семьи из таблицы; фразу об этом строит голосовая сторона из этих же данных, не LLM.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

from . import knowledge as kn

#: Короче — опечаток не прощаем: «арфе» ↔ «арене», одна правка — уже другое слово.
_TYPO_MIN_LEN = 5
#: Предлог слота: «на балалайке».
_SLOT_PREP = "на"


def within_one(word: str, form: str) -> bool:
    """Слова различаются не больше чем одной правкой (замена, вставка, удаление буквы)."""
    if word == form:
        return True
    if abs(len(word) - len(form)) > 1:
        return False
    short, long_ = sorted((word, form), key=len)
    i = next((k for k, (a, b) in enumerate(zip(short, long_)) if a != b), len(short))
    if len(short) == len(long_):
        return short[i + 1:] == long_[i + 1:]
    return short[i:] == long_[i + 1:]


def near(word: str, vocab: Iterable[str]) -> Optional[str]:
    """Слово закрытого словаря, от которого ``word`` отличается одной правкой (опечатка STT); точное — первым."""
    forms = tuple(vocab)
    if word in forms:
        return word
    if len(word) < _TYPO_MIN_LEN:
        return None
    return next((f for f in forms if len(f) >= _TYPO_MIN_LEN and within_one(word, f)), None)


def _forms() -> Tuple[Tuple[str, str], ...]:
    """``(форма слова, ключ инструмента)`` — все основы со всеми окончаниями."""
    return tuple((stem + end, key) for key, ins in kn.INSTRUMENTS.items()
                 for stem in ins.stems for end in kn.INSTRUMENT_ENDINGS)


_FORMS = _forms()
_FORM_KEY = dict(_FORMS)


_UNSUPPORTED = frozenset(stem + end for stem in kn.UNSUPPORTED_INSTRUMENT_STEMS for end in kn.INSTRUMENT_ENDINGS)


def instrument_of(word: str) -> Optional[str]:
    """Ключ инструмента по слову («балалайке», «трамбоне», «кселофоне») или ``None``.

    Опечатка прощается, только если она однозначна (#3592): точная форма таблицы побеждает; слово известного
    инструмента без тембра («банджо») не правится в соседа; слово на одной правке от форм ДВУХ разных инструментов —
    ``None`` (честнее «не знаю», чем наугад: «ситар» не должен стать «гитарой»).
    """
    w = word.lower()
    if w in _FORM_KEY:
        return _FORM_KEY[w]
    if w in _UNSUPPORTED or len(w) < _TYPO_MIN_LEN:
        return None
    keys = {key for form, key in _FORMS if len(form) >= _TYPO_MIN_LEN and within_one(w, form)}
    return next(iter(keys)) if len(keys) == 1 else None


def instrument_word(word: str) -> bool:
    """Слово называет инструмент — из таблицы (:func:`instrument_of`) или известный без тембра («гуслях», «банджо»):
    в заказе это слот, а не слово названия (#4016: «спой … на балалайке и гуслях»)."""
    return instrument_of(word) is not None or word.lower() in _UNSUPPORTED


def split_instrument(words: Sequence[str]) -> Tuple[str, List[str]]:
    """``(ключ инструмента, слова без слота)`` по «на <инструменте>» / «на инструменте <X>»; нет — ``("", слова)``."""
    for i, word in enumerate(words):
        if word != _SLOT_PREP:
            continue
        j = i + 1 + (i + 1 < len(words) and words[i + 1] in kn.INSTRUMENT_NOUNS)
        key = instrument_of(words[j]) if j < len(words) else None
        if key:
            return key, [*words[:i], *words[j + 1:]]
    return "", list(words)


def neighbors(key: str) -> Tuple[str, ...]:
    """Инструменты той же семьи с синтом, в порядке таблицы (без самого ``key``)."""
    family = kn.INSTRUMENTS[key].family
    return tuple(k for k, ins in kn.INSTRUMENTS.items() if k != key and ins.family == family and ins.synth)


@dataclass(frozen=True)
class LeadChoice:
    """Чем играть лид: ``asked`` — что просили, ``played`` — чей синт звучит (``""`` — свой тембр трека)."""

    asked: str
    played: str
    synth: Optional[str]
    octave_shift: int

    @property
    def substituted(self) -> bool:
        return self.played != self.asked


def lead_choice(key: str) -> Optional[LeadChoice]:
    """Решение кода о лиде для инструмента ``key``; ``None`` — инструмент не просили или его нет в таблице."""
    ins = kn.INSTRUMENTS.get(key or "")
    if ins is None:
        return None
    if ins.synth:
        return LeadChoice(key, key, ins.synth, ins.octave_shift)
    sub = next(iter(neighbors(key)), "")
    if not sub:
        return LeadChoice(key, "", None, 0)
    return LeadChoice(key, sub, kn.INSTRUMENTS[sub].synth, kn.INSTRUMENTS[sub].octave_shift)


def shifted(pitches: Sequence[int], shift: int, corridor: Tuple[int, int]) -> int:
    """Сдвиг лида, который можно применить: ``shift``, если все ноты остаются в коридоре, иначе 0."""
    if not shift or not pitches:
        return 0
    return shift if corridor[0] <= min(pitches) + shift and max(pitches) + shift <= corridor[1] else 0


__all__ = ["LeadChoice", "instrument_of", "instrument_word", "lead_choice", "near", "neighbors", "shifted",
           "split_instrument", "within_one"]
