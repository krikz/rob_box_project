#!/usr/bin/env python3
"""speaker_profiles — Pure-Python логика профилей спикеров (issue #1077).

Этот модуль НЕ импортирует rclpy/vosk/grpc — только stdlib. Позволяет
тестировать подтверждение спикера, извлечение имени и форматирование
LLM-контекста без ROS2-окружения (см. ``test/test_speaker_profiles.py``).

Контекст issue #1077
---------------------
Yandex SpeechKit v3 возвращает ``speaker_analysis`` (speaker_tag + границы +
статистику). Раньше эта информация выбрасывалась — робот не знал, с кем
говорит. Теперь stt_node прокидывает speaker_tag наружу, а dialogue_node
создаёт профиль спикера (scope=speaker:<tag>) после подтверждения.

Edge cases (из карточки #1077):
1. **Нестабильный tag**: Yandex может разбить один голос на tag='0' и tag='1'
   в одной фразе. Правило: профиль подтверждается 2+ фразами подряд с ОДНИМ
   tag; короткие (<0.8с) фразы не считаются и не сбрасывают streak.
2. **Смена спикера**: каждый ход несёт свой tag; streak другого tag
   сбрасывается — профили не смешиваются.
3. **Vosk fallback**: tag=None → трекер не трогаем, профиль не создаётся.
"""

from __future__ import annotations

import re
from typing import Dict, Optional, Sequence

# Порог подтверждения: 2+ фразы подряд с одним tag (защита от мусорных
# tags, которые Yandex иногда генерит при расщеплении одного голоса).
DEFAULT_MIN_PHRASES = 2

# Короткие фразы (<0.8с) не создают профиль — это эхо/шум/обрывки,
# а не устойчивый голос собеседника.
DEFAULT_MIN_DURATION_S = 0.8

# Паттерны «меня зовут X» / «зовут меня X» / «моё имя X» / «имя мне X».
# Имя — первое слово после паттерна (кириллица/латиница/дефис).
_NAME_PATTERNS = (
    r"меня\s+зовут",
    r"зовут\s+меня",
    r"мо[её]\s+имя",
    r"имя\s+мне",
)
_NAME_RE = re.compile(
    r"(?:"
    + "|".join(_NAME_PATTERNS)
    + r")\s+([А-ЯЁа-яёA-Za-z][А-ЯЁа-яёA-Za-z\-]*)",
    re.IGNORECASE,
)

# Ключ факта-профиля внутри scope=speaker:<tag>.
SPEAKER_PROFILE_KEY = "profile"


class SpeakerTracker:
    """Считает подряд идущие фразы одного speaker_tag.

    Профиль подтверждается после ``min_phrases`` фраз подряд с одним tag,
    каждая длительностью >= ``min_duration_s``. Короткие фразы игнорируются
    (не считаются и не сбрасывают streak других tag). Смена tag сбрасывает
    streak предыдущего — защита от нестабильных tags (issue #1077 edge 1/2).
    """

    def __init__(
        self,
        min_phrases: int = DEFAULT_MIN_PHRASES,
        min_duration_s: float = DEFAULT_MIN_DURATION_S,
    ) -> None:
        self.min_phrases = min_phrases
        self.min_duration_s = min_duration_s
        self._streak: Dict[str, int] = {}
        self._confirmed: set = set()

    def note_phrase(self, tag: Optional[str], duration_s: float) -> bool:
        """Зафиксировать фразу с ``tag``.

        Returns:
            ``True`` если именно эта фраза ПОДТВЕРДИЛА tag (переход
            streak == min_phrases). ``False`` во всех остальных случаях
            (tag=None, короткая фраза, streak ещё не достиг порога,
            tag уже был подтверждён ранее).
        """
        if not tag:
            return False
        if duration_s < self.min_duration_s:
            # Короткая фраза — не считаем и не сбрасываем streak.
            return False
        # Сбрасываем streak всех ДРУГИХ tags (требование «подряд»).
        for other in list(self._streak):
            if other != tag:
                self._streak[other] = 0
        self._streak[tag] = self._streak.get(tag, 0) + 1
        if self._streak[tag] >= self.min_phrases and tag not in self._confirmed:
            self._confirmed.add(tag)
            return True
        return False

    def is_confirmed(self, tag: Optional[str]) -> bool:
        """Подтверждён ли ``tag`` (уже накопил min_phrases подряд)."""
        return bool(tag) and tag in self._confirmed


def extract_speaker_name(text: Optional[str]) -> Optional[str]:
    """Извлечь имя из «меня зовут X» / «моё имя X» / «зовут меня X».

    Returns:
        Имя с заглавной буквы или ``None``, если паттерн не найден.
    """
    if not text:
        return None
    match = _NAME_RE.search(text)
    if not match:
        return None
    name = match.group(1).strip(".,!?;: ")
    if not name:
        return None
    return name[0].upper() + name[1:]


def format_speaker_context(
    profile: Dict,
    facts: Sequence,
    *,
    is_new: bool = False,
) -> Optional[str]:
    """Собрать LLM-контекст о спикере из профиля и фактов.

    Args:
        profile: dict-профиль (first_seen/last_seen/dialog_count/name).
        facts: факты из scope=speaker:<tag> (без самого profile).
        is_new: True — это первый подтверждённый диалог со спикером
            (робот должен поприветствовать нового собеседника).

    Returns:
        Строка-контекст для system-сообщения, или ``None`` если контекст
        пуст (нечего подгружать).
    """
    lines: list[str] = []
    profile = profile or {}
    name = profile.get("name")
    if name:
        lines.append(f"Собеседника зовут {name}.")
    # Строка о диалоге добавляется только если профиль реально создан
    # (touch_speaker проставил dialog_count). Пустой profile ({}) — ещё
    # нет подтверждённого профиля → контекст не собираем (None).
    if profile.get("dialog_count") is not None:
        dialog_count = int(profile.get("dialog_count", 0))
        lines.append(f"Это {dialog_count}-й диалог с этим собеседником.")
    if is_new:
        lines.append(
            "Это НОВЫЙ собеседник (первый подтверждённый диалог) — "
            "поприветствуй его."
        )
    for fact in facts or ():
        key = getattr(fact, "key", "")
        if key == SPEAKER_PROFILE_KEY:
            continue
        value = getattr(fact, "value", "")
        if key:
            lines.append(f"Факт о собеседнике ({key}): {value}")
        else:
            lines.append(f"Факт о собеседнике: {value}")
    if not lines:
        return None
    return "Контекст о собеседнике:\n" + "\n".join(lines)


__all__ = [
    "DEFAULT_MIN_PHRASES",
    "DEFAULT_MIN_DURATION_S",
    "SPEAKER_PROFILE_KEY",
    "SpeakerTracker",
    "extract_speaker_name",
    "format_speaker_context",
]
