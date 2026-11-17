"""Починка распознанной речи по закрытой таблице — до Tier-1 и до модели (ADR-0148: решает код).

STT искажает ключевые слова команд: «Робот, всё, глуши» на марафоне 23.10 пришло как «робот всох в глуши» (n712) —
модель приняла фразу за реплику, стопа не было, музыка играла дальше. :func:`repair` заменяет НАЧАЛО команды (первые
слова после вейка), если оно ровно совпадает с известным искажением из таблицы
:data:`rob_box_dialog.knowledge.HEARD_FIXES`: формы только из живых логов, слова целиком (нижний регистр, ё → е,
знаки между словами не в счёт). Угадывания нет: слова не из таблицы и искажение не в начале команды остаются как
есть, похожее слово не подбирается. Остальной шум распознавания — дело модели (строка ``knowledge.STT_NOISE_NOTE``) и
шага C (уверенность и альтернативы STT).

Единственная точка вызова — адресация (``address.decide``): починенный текст видят и Tier-1, и модель; Telegram и
оператор печатают текст сами, их реплики не чинятся. Другие закрытые словари искажений живут у своих решений:
искажённый вейк — ``words.slipped_wake_rest`` (``WAKE_SLIP_*``), имена движков музыки —
``rob_box_music.knowledge.MUSIC_ENGINES``.

Чистый модуль: только таблица :mod:`.knowledge`.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Tuple

from rob_box_dialog.knowledge import HEARD_FIXES

#: Метка строки лога хода о починке (что было → что стало): корпус искажений и живая проверка.
HEARD_MARK = "🩹 [heard]"

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


__all__ = ["HEARD_MARK", "Repaired", "repair"]
