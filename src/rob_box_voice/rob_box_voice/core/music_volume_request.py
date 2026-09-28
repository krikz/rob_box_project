"""music_volume_request.py — «громче/тише» при играющей музыке (issue #3125).

Живой сет 28.09.2026 (develop после #3121): во время DJ-сета юзер просит
«играй громче». Bug C (issue #992) видит слово «играй», считает это заказом
НОВОЙ музыки и требует ``execute_music_code``/``compose_music``, хотя трек
играет (``[track-mode] TRACK играет с прошлого хода``). Ретраи кончаются
nudge «Я тут растерялся — бит не запустился», и так 3 раза из 4::

    [TG] играй громче
    process_input returned: spoken='Громче, громче! Танцпол не слышит!' tools=['set_volume']
    🎵 [issue 992 Bug C] user asked for music but LLM skipped execute_music_code
        (tools=['set_volume']); synchronous retry 1/3

Просьба о ГРОМКОСТИ играющего трека — не просьба завести музыку. Её честно
закрывает ``set_music_volume`` (мастер-фейдер scsynth), а ретрай «вызови
execute_music_code» только уводит модель в перезапуск трека или фантазию.

Отдельный модуль (а не ещё один блок в ``dialogue_guards``), чтобы правка
не пересекалась с параллельными правками эвристик в том файле.
"""

from __future__ import annotations

import re
from typing import Iterable

#: Тулы, которые меняют громкость МУЗЫКИ. ``set_volume`` сюда НЕ входит: он
#: крутит голос (``/tts_node volume_db``), музыку не трогает.
MUSIC_VOLUME_TOOLS: frozenset = frozenset({"set_music_volume"})

#: Ведущие теги источника/контекста: ``[TG]``, ``[Speaker:unknown]``,
#: ``[🎧 Музыкальный режим активен — фоновая музыка играет, …]``. Слова внутри
#: тегов — не слова юзера («фоновая музыка играет» есть в каждом DJ-ходе).
_LEADING_TAGS_RE = re.compile(r"^\s*(?:\[[^\]\n]*\]\s*)+")

#: Слово-«ядро» просьбы о громкости. ``[гк]ромч`` — STT/опечатка «кромче»
#: из живого лога («играй трей кромче а не говори громче»).
_VOLUME_CORE_RE = re.compile(
    r"^(?:по)?[гк]ромч\w*$|^(?:по)?тиш\w*$|^громкост\w*$|^громк\w*$|"
    r"^убав\w*$|^прибав\w*$|^приглуш\w*$|^подкрут\w*$|^выкрут\w*$"
)

#: Служебные слова, которые не меняют смысла «сделай музыку громче». Если в
#: реплике есть что-то сверх ядра и этих слов («сыграй В ПЕЩЕРЕ ГОРНОГО
#: КОРОЛЯ погромче») — это заказ трека, и Bug C должен работать как раньше.
_VOLUME_FILLER_WORDS: frozenset = frozenset({
    "играй", "сыграй", "играйте", "включи", "сделай", "сделайте", "давай",
    "поставь", "ну", "а", "не", "и", "но", "же", "ка", "эй", "йо",
    "музыку", "музыка", "музыки", "музон", "трек", "трека", "трей", "бит",
    "бита", "звук", "звука", "сет", "ещё", "еще", "чуть", "чуточку",
    "немного", "пожалуйста", "плиз", "можно", "на", "полную", "максимум",
    "говори", "голос", "голоса", "это", "так", "очень", "слишком", "её",
    "ее", "его", "мне", "нам", "там", "тут", "сейчас", "уже", "вот", "по",
    "в", "два", "раза", "раз", "чтобы", "чтоб", "сильнее", "больше",
    "меньше", "побольше", "поменьше", "громко", "тихо",
})

_WORD_RE = re.compile(r"[a-zа-яё]+", re.IGNORECASE)


def extract_user_utterance(user_input: str) -> str:
    """Слова юзера без ведущих тегов и без хвостовых служебных блоков.

    ``user_input`` гуарда — это ``[TG] [🎧 …] играй громче\\n[URGENT_BACKLOG] …``
    или ``[Speaker:unknown] играй громче\\n\\n[CRITICAL] …`` (ретрай). Реплика
    юзера — первая строка после тегов; backlog и CRITICAL-блоки идут со
    следующей строки.
    """
    if not user_input:
        return ""
    text = _LEADING_TAGS_RE.sub("", user_input)
    return text.split("\n", 1)[0].strip()


def is_music_volume_request(user_input: str) -> bool:
    """Реплика — ТОЛЬКО просьба сделать громче/тише (без заказа трека)?

    ``True`` для «играй громче», «играй трей кромче а не говори громче»,
    «сделай музыку потише»; ``False`` для «сыграй в пещере горного короля
    погромче» (там назван трек) и для реплик без слова громкости.
    """
    words = [w.lower() for w in _WORD_RE.findall(extract_user_utterance(user_input))]
    if not words:
        return False
    has_core = False
    for word in words:
        if _VOLUME_CORE_RE.match(word):
            has_core = True
        elif word not in _VOLUME_FILLER_WORDS:
            return False
    return has_core


def music_volume_tool_called(tools_called: Iterable[str]) -> bool:
    """В ходе вызван тул громкости МУЗЫКИ (``set_music_volume``)."""
    return bool(set(tools_called or ()) & MUSIC_VOLUME_TOOLS)


__all__ = [
    "MUSIC_VOLUME_TOOLS",
    "extract_user_utterance",
    "is_music_volume_request",
    "music_volume_tool_called",
]
