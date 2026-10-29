"""Последний сочинённый трек - единственное место знания «какая песня - эта» (#4216, ADR-0148).

Заказ генерации даёт два варианта в кэш, играет один (песня, инструментал). Вчерашние версии рядом (с тем же или чуть
другим названием, про то же) и модель, выбирая по названию из списка, брала их (n707/n708 марафона-5 29.10; короткий
прогон 29.10 05:16: ``search «енота»`` → ``search «Енот»`` → ``info`` старой версии, итерации кончились). Генерация
отмечает здесь тот вариант, что сыграла; ``music_library`` читает отсюда, а не угадывает:

* :meth:`fresh` - id свежего трека;
* :meth:`about` - свежий трек, если тема запроса (слова ``search``) совпала с ним: тул ставит его первым и запоминает тему;
* :meth:`twin` - другой трек «из той же песни»: то же название либо тема, что совпала со свежим.
"""

from __future__ import annotations

import re
import time
from typing import Any, Callable, Mapping, Optional, Tuple

#: Сколько секунд только что сочинённый трек остаётся «этой песней»: «расскажи про неё», «включи с начала» через минуты
#: после ``[song] готова`` - про него, а не про старую версию.
GENERATED_FRESH_S = 900.0
#: Сколько секунд помнится тема поиска, совпавшая со свежим треком: ``info`` по старой версии той же темы идёт следом.
TOPIC_TTL_S = 300.0
#: Слова короче не тема. Основа слова - без двух последних букв, но не короче ``STEM_MIN``: «енота» и «енот-механик» -
#: «енот».
WORD_MIN = 4
STEM_MIN = 4
#: Слова о самой песне - не тема («расскажи про песню про енота»: тема - «енот», а не «песня»).
GENERIC_STEMS: Tuple[str, ...] = ("песн", "трек", "композиц", "музык", "мелоди", "инструментал", "произведен", "саундтрек")
_WORD = re.compile(r"[^\W\d_]+")
#: Поля трека, в которых ищется тема.
_TEXT_FIELDS = ("title", "lyrics", "prompt", "notes", "genre", "mood")


def topic_stems(text: str) -> Tuple[str, ...]:
    """Основы слов темы запроса (строчные, ё → е; слова о самой песне отброшены)."""
    words = (w.casefold().replace("ё", "е") for w in _WORD.findall(text or ""))
    stems = (w[:max(STEM_MIN, len(w) - 2)] for w in words if len(w) >= WORD_MIN)
    return tuple(dict.fromkeys(s for s in stems if not s.startswith(GENERIC_STEMS)))


def _text(info: Mapping[str, Any]) -> str:
    return " ".join(str(info.get(f) or "") for f in _TEXT_FIELDS).casefold().replace("ё", "е")


class LastGenerated:
    """Id последнего сочинённого трека и когда; ``info`` - метаданные трека из кэша (``success`` ложь - трека нет:
    стёртый «этой песней» не бывает)."""

    def __init__(self, info: Callable[[str], Mapping[str, Any]], clock: Callable[[], float] = time.monotonic) -> None:
        self._info, self._clock = info, clock
        self._noted: Optional[Tuple[str, float]] = None
        self._topic: Optional[Tuple[Tuple[str, ...], float]] = None

    def note(self, track_id: str) -> None:
        """Запомнить трек, который генерация только что сыграла (прежняя тема поиска к нему не относится)."""
        if track_id:
            self._noted, self._topic = (str(track_id), self._clock()), None

    def fresh(self, max_age_s: float = GENERATED_FRESH_S) -> Optional[str]:
        """Id последнего сочинённого трека, если он не старше ``max_age_s`` и ещё в кэше, иначе ``None``."""
        noted = self._noted
        if noted is None or self._clock() - noted[1] > max_age_s:
            return None
        return noted[0] if self._info(noted[0]).get("success") else None

    def about(self, stems: Tuple[str, ...]) -> Optional[str]:
        """Свежий трек, если ВСЕ основы темы запроса есть в его названии/тексте; тема запоминается (:meth:`twin`)."""
        fresh = self.fresh()
        if not fresh or not stems or not self._covers(fresh, stems):
            return None
        self._topic = (stems, self._clock())
        return fresh

    def twin(self, track_id: str) -> bool:
        """``track_id`` - другая версия свежего трека: то же название либо недавняя тема (:meth:`about`), что есть и в нём."""
        fresh = self.fresh()
        if not fresh or track_id == fresh:
            return False
        title = str(self._info(track_id).get("title") or "").strip().casefold()
        if title and title == str(self._info(fresh).get("title") or "").strip().casefold():
            return True
        topic = self._topic
        return bool(topic and self._clock() - topic[1] <= TOPIC_TTL_S and self._covers(track_id, topic[0]))

    def _covers(self, track_id: str, stems: Tuple[str, ...]) -> bool:
        info = self._info(track_id)
        text = _text(info) if info.get("success") else ""
        return bool(text) and all(s in text for s in stems)


__all__ = ["GENERATED_FRESH_S", "TOPIC_TTL_S", "LastGenerated", "topic_stems"]
