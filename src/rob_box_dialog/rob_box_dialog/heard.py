"""Починка распознанной речи и переспрос стопа по гипотезам STT — до Tier-1 и до модели (ADR-0148: решает код).

STT искажает ключевые слова команд: «Робот, всё, глуши» на марафоне 23.10 пришло как «робот всох в глуши» (n712) —
модель приняла фразу за реплику, стопа не было, музыка играла дальше. :func:`repair` заменяет НАЧАЛО команды (первые
слова после вейка), если оно ровно совпадает с известным искажением из таблицы
:data:`rob_box_dialog.knowledge.HEARD_FIXES`: формы только из живых логов, слова целиком (нижний регистр, ё → е,
знаки между словами не в счёт). Угадывания нет: слова не из таблицы и искажение не в начале команды остаются как
есть, похожее слово не подбирается.

Шаг C2 (#4017): когда реплика — ясный стоп Tier-1, а другие гипотезы STT (``alternatives``) расходятся со
стоп-смыслом распознанного текста (один вариант — ясный стоп, другой — нет), код **не** исполняет стоп и не
говорит «выполнил», а возвращает структурный переспрос (:func:`stop_reask_phrase`). Детектор стопа — тот же
:func:`rob_box_dialog.words.stop_verdict`, что у Tier-1 и перебивания. Порога confidence нет: без альтернатив
поведение как раньше. Платное/необратимое — не этот шаг.

Единственная точка починки — адресация (``address.decide``): починенный текст видят и Tier-1, и модель; Telegram и
оператор печатают текст сами, их реплики не чинятся. Переспрос стопа по гипотезам — ход (``turn.turn_work``).
Другие закрытые словари искажений живут у своих решений: искажённый вейк — ``words.slipped_wake_rest``
(``WAKE_SLIP_*``), имена движков музыки — ``rob_box_music.knowledge.MUSIC_ENGINES``.

Чистый модуль: :mod:`.knowledge`, :mod:`.words`.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from rob_box_dialog.knowledge import HEARD_FIXES
from rob_box_dialog.words import STOP_CLEAR, stop_verdict

#: Метка строки лога хода о починке (что было → что стало): корпус искажений и живая проверка.
HEARD_MARK = "🩹 [heard]"
#: Причина хода: стоп не исполнен — гипотезы STT расходятся со стоп-смыслом (#4017 C2).
STOP_REASK_REASON = "stt_stop_reask"
#: Виды Tier-1, для которых C2 смотрит расхождение гипотез (тот же набор, что ``interrupt._STOP_KINDS``).
STOP_REASK_KINDS: Tuple[str, ...] = ("stop_all", "stop_speech", "silence")

_TOKEN = re.compile(r"[\w-]+")
#: Длинные искажения раньше коротких: общее начало не съедает более полное совпадение.
_FIXES = tuple(sorted(HEARD_FIXES.items(), key=lambda item: -len(item[0])))


@dataclass(frozen=True)
class Repaired:
    """Текст команды после починки и что починено (``«всох в глуши» → «всё, глуши»``); не чинилось — ``fixes`` пуст."""

    text: str
    fixes: Tuple[str, ...] = ()


def repair(text: str) -> Repaired:
    """Начало команды — известное искажение STT (:data:`~rob_box_dialog.knowledge.HEARD_FIXES`) → что сказано;
    остаток фразы не меняется. Не совпало — текст как есть."""
    source = text or ""
    spans = tuple(_TOKEN.finditer(source))
    words = tuple(span.group(0).lower().replace("ё", "е") for span in spans)
    for heard, said in _FIXES:
        if words[: len(heard)] == heard:
            start, end = spans[0].start(), spans[len(heard) - 1].end()
            return Repaired(source[:start] + said + source[end:], (f"«{source[start:end]}» → «{said}»",))
    return Repaired(source)


def is_clear_stop(text: str, wake_words: Optional[Sequence[str]] = None) -> bool:
    """Ясная стоп-команда по :func:`~rob_box_dialog.words.stop_verdict` (тот же детектор, что Tier-1)."""
    return stop_verdict(text or "", wake_words) == STOP_CLEAR


def stop_hypotheses_diverge(
    text: str,
    alternatives: Sequence[str] = (),
    wake_words: Optional[Sequence[str]] = None,
) -> bool:
    """Распознанный текст и хотя бы одна альтернатива STT расходятся: один — ясный стоп, другой — нет.

    Без альтернатив — ``False`` (поведения не меняем; confidence порогом не сравниваем).
    """
    if not alternatives:
        return False
    primary = is_clear_stop(text, wake_words)
    return any(is_clear_stop(alt, wake_words) != primary for alt in alternatives)


def should_reask_stop(
    kind: str,
    text: str,
    alternatives: Sequence[str] = (),
    wake_words: Optional[Sequence[str]] = None,
) -> bool:
    """Ясный стоп Tier-1 (``kind`` из :data:`STOP_REASK_KINDS`) и гипотезы расходятся → переспрос вместо стопа."""
    return kind in STOP_REASK_KINDS and stop_hypotheses_diverge(text, alternatives, wake_words)


def stop_reask_phrase(text: str) -> str:
    """Фраза переспроса стопа кодом из распознанного текста (не модель): «Я услышал «X» — ты про стоп?»."""
    heard = (text or "").strip() or "…"
    return f"Я услышал «{heard}» — ты про стоп?"


__all__ = [
    "HEARD_MARK",
    "STOP_REASK_KINDS",
    "STOP_REASK_REASON",
    "Repaired",
    "is_clear_stop",
    "repair",
    "should_reask_stop",
    "stop_hypotheses_diverge",
    "stop_reask_phrase",
]
