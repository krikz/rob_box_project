"""skill_router.py — детерминированная активация доменного скилла.

Change ``skill-scoped-dialogue-context``, фаза 3, задача 3.1.

Зачем отдельный модуль
----------------------

Скилл активируется двумя путями, и оба нужны:

1. **Детерминированный** (этот модуль) — реплику классифицируем ДО
   обращения к LLM, и фрагмент оказывается уже в ПЕРВОМ запросе хода.
   Лишнего round-trip нет, латентность голосового цикла не страдает.
2. **Модельный** (``load_skill`` в ``AgentCore``) — когда роутер
   промахнулся. Текст приходит tool-результатом в самый хвост
   ``messages``, то есть в позицию максимальной свежести.

Второй классификатор здесь НЕ заводится. Роутер — тонкая проекция уже
существующих и обкатанных на живом роботе решателей на имена скиллов:

* :class:`~rob_box_voice.core.command_parser.CommandParser` — тот же
  экземпляр, что обслуживает ``command_intent_gate`` (issue #1279);
* музыкальные детекторы из
  :mod:`~rob_box_voice.core.dialogue_guards` — ``user_wants_music``,
  ``is_music_stop_command``, ``user_wants_performance``.

Отсюда граница ответственности: роутер НЕ обязан быть точным. Он обязан
быть **дешёвым и безопасным**. Промах ничего не ломает — при выключенном
сужении каталога (Move B по умолчанию off) LLM видит все инструменты и
может позвать ``load_skill`` сама. Поэтому правила консервативные:
сомневаешься — не активируй, верни ``None``.
"""

from __future__ import annotations

import re
from typing import Optional

from rob_box_voice.core.command_parser import CommandParser, IntentType
from rob_box_voice.core.dialogue_guards import (
    is_music_stop_command,
    user_wants_music,
    user_wants_performance,
)

__all__ = ["SkillRouter"]


#: Императивы, которые перебивают всё остальное. Проверяются ПЕРВЫМИ, до
#: музыкальных детекторов: «запомни, что я люблю джаз» — это память, а не
#: музыка, хотя ``user_wants_music`` и срабатывает на слово «джаз».
#: Глагол в повелительном наклонении задаёт домен надёжнее, чем
#: упоминание жанра в дополнении.
_IMPERATIVES: dict[str, tuple[re.Pattern[str], ...]] = {
    "memory": (
        re.compile(r"\bзапомни\b", re.I),
        re.compile(r"\bкак меня зовут\b", re.I),
        re.compile(r"\b(помнишь|вспомни)\b", re.I),
        re.compile(r"\bменя зовут\b", re.I),
    ),
    "knowledge": (
        re.compile(r"\b(поищи|загугли|погугли)\b", re.I),
        re.compile(r"\b(погод\w+|новост\w+)\b", re.I),
    ),
    "voice-tts": (
        re.compile(r"\bголос\w*\b", re.I),
        # «погромче» / «потише» — приставка не должна ломать границу слова.
        re.compile(r"\b(по)?(громче|тише)\b", re.I),
        re.compile(r"\bгромкость\w*\b", re.I),
    ),
}

#: Домен → компактный признак в реплике. Только формулировки, которые
#: нельзя спутать с соседним доменом. Музыку сознательно НЕ разбираем
#: регулярками — для неё есть готовые детекторы (см. docstring модуля),
#: они уже учитывают контекст.
#:
#: Навигация и карта продублированы здесь НЕ как второй классификатор:
#: ``CommandParser`` — рефлекс-путь, он по замыслу узкий («поезжай К
#: кухне», но не «поезжай НА кухню») и отвечает за ИСПОЛНЕНИЕ команды.
#: Роутеру нужно только имя домена, промах по которому ничего не ломает.
#: Поэтому здесь широкие подсказки, а там — точные команды.
_PATTERNS: dict[str, tuple[re.Pattern[str], ...]] = {
    "navigation": (
        re.compile(r"\b(поезжай|езжай|едь|двигайся|катись|подъедь)\b", re.I),
        re.compile(r"\b(вперёд|вперед|назад|налево|направо)\b", re.I),
        re.compile(r"\b(точк\w+|вейпоинт|waypoint)\b", re.I),
        re.compile(r"\bэто\s+(кухня|гостиная|спальня|коридор|комната)\b", re.I),
        re.compile(r"\bгде\s+ты\b", re.I),
    ),
    "mapping": (
        re.compile(r"\bкартограф\w*", re.I),
        re.compile(r"\bкартир\w*", re.I),
        re.compile(r"\b(построй|создай|сделай|загрузи)\s+карт", re.I),
    ),
    "renardo-library": (
        re.compile(r"\bмедиатек\w*", re.I),
    ),
    "player": (
        re.compile(r"\bбиблиотек\w*", re.I),
        re.compile(r"\b(что|какие)\b.{0,20}\bтрек", re.I),
        re.compile(r"\b(тот|этот|прошл\w+|раньше)\b.{0,20}\bтрек", re.I),
        re.compile(r"\bпослуша(ть|ем)\b.{0,20}\bещ[её]\b", re.I),
    ),
    "dj": (
        re.compile(r"\bдиджей\w*", re.I),
        re.compile(r"\bdj\b", re.I),
        re.compile(r"\bвечеринк\w*", re.I),
    ),
    "expression": (
        re.compile(r"\b(анимаци\w+|помигай|подмигни)\b", re.I),
        re.compile(r"\bзвук\w*\b", re.I),
    ),
}

#: Интенты CommandParser, у которых есть однозначный скилл.
_INTENT_SKILL: dict[IntentType, str] = {
    IntentType.NAVIGATE: "navigation",
    IntentType.MAP: "mapping",
    IntentType.STATUS: "core",
}


class SkillRouter:
    """Классифицирует реплику в имя доменного скилла (или ``None``)."""

    def __init__(
        self,
        parser: CommandParser,
        *,
        known_skills: tuple[str, ...] = (),
        confidence: float = 0.7,
    ) -> None:
        """
        :param parser: ТОТ ЖЕ ``CommandParser``, что у
            ``command_intent_gate`` — один источник классификации.
        :param known_skills: имена скиллов, для которых есть фрагменты.
            Роутер никогда не вернёт скилл вне этого набора: активировать
            домен, у которого нет текста, бессмысленно.
        :param confidence: порог для интентов ``CommandParser``, совпадает
            с ``command_intent_gate_confidence``.
        """
        self._parser = parser
        self._known = set(known_skills)
        self._confidence = confidence

    def route(self, text: str) -> Optional[str]:
        """Вернуть имя скилла для реплики или ``None``.

        ``None`` означает «не уверен»: LLM получит полный каталог и при
        необходимости позовёт ``load_skill`` сама.
        """
        if not text or not text.strip():
            return None

        # 1. Императивы — раньше всего: «запомни, что я люблю джаз» это
        #    память, а не музыка.
        hit = self._first_match(_IMPERATIVES, text)
        if hit is not None:
            return hit

        # 2. Музыка — готовыми детекторами, до общих регулярок: «стоп
        #    музыку» не должно уехать в navigation по слову «стоп».
        music = self._route_music(text)
        if music is not None:
            return self._known_or_none(music)

        # 3. Интенты CommandParser (навигация, карта, статус).
        try:
            command = self._parser.parse(text)
        except Exception:  # noqa: BLE001 — классификация не роняет ход
            command = None
        if command is not None:
            skill = _INTENT_SKILL.get(command.intent)
            if skill and command.confidence >= self._confidence:
                known = self._known_or_none(skill)
                if known is not None:
                    return known

        # 4. Компактные признаки остальных доменов.
        return self._first_match(_PATTERNS, text)

    # ---- internals ------------------------------------------------------

    def _first_match(
        self,
        table: dict[str, tuple[re.Pattern[str], ...]],
        text: str,
    ) -> Optional[str]:
        """Первый домен таблицы, чей признак нашёлся в тексте.

        Домен без загруженного фрагмента пропускается, а не обрывает
        поиск: иначе один невключённый скилл прятал бы все следующие.
        """
        for skill, patterns in table.items():
            if skill not in self._known:
                continue
            if any(pattern.search(text) for pattern in patterns):
                return skill
        return None

    def _route_music(self, text: str) -> Optional[str]:
        """Разложить музыкальный запрос на composer / dj / player."""
        if is_music_stop_command(text):
            # «выключи музыку» — composer несёт stop_music и правило
            # «сверься с <music_state>, прежде чем отвечать словами».
            return "composer"
        if any(pattern.search(text) for pattern in _PATTERNS["dj"]):
            return "dj"
        if any(pattern.search(text) for pattern in _PATTERNS["player"]):
            return "player"
        if any(pattern.search(text) for pattern in _PATTERNS["renardo-library"]):
            return "renardo-library"
        if user_wants_music(text) or user_wants_performance(text):
            return "composer"
        return None

    def _known_or_none(self, skill: str) -> Optional[str]:
        return skill if skill in self._known else None
