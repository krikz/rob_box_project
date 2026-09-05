"""Секции мастер-промпта, привязанные к доменным скиллам (фаза 5 change'а).

Зачем это существует
--------------------

§5 MUSIC (2 181 tok) и §6 WAYPOINTS (184 tok) — доменные правила: они нужны
на музыкальном и навигационном ходу и мертвы на всех остальных. Сейчас они
лежат на позиции 0 системного промпта, перед двадцатью ходами истории, и
проигрывают свежему few-shot из этой истории (``agent_core.py:681``).

Раскол этих секций нельзя делать простым удалением текста из файла: флаг
``skills_enabled`` по умолчанию ``false`` (Migration Plan), и при выключенном
флаге фрагменты скиллов не грузятся вообще. Вырезанный текст в дефолтной
конфигурации не приехал бы ниоткуда — робот потерял бы правила музыки и
навигации целиком.

Поэтому текст остаётся в ``master_prompt_compact.txt`` ровно в одном
экземпляре, а его принадлежность скиллу объявляется разметкой:

* ``<<<SKILL-MOVE composer,dj>>> … <<<SKILL-MOVE-END>>>`` — блок доменных
  правил. При ``skills_enabled=false`` остаётся в системном промпте на своём
  месте; при ``true`` уезжает из промпта во фрагменты названных скиллов и
  доставляется вплотную к текущему ходу.
* ``<<<SKILL-OFF>>> … <<<SKILL-OFF-END>>>`` — текст только для выключенных
  скиллов (маршрутные строки §0, которые после переезда секций начали бы
  врать).
* ``<<<SKILL-ON>>> … <<<SKILL-ON-END>>>`` — текст только для включённых.

Что это даёт:

* дефолтная конфигурация побайтово совпадает с сегодняшней — маркеры
  вырезаются в обоих режимах, содержимое при ``false`` не двигается;
* источник правды один, дублирования «мастер-промпт ↔ фрагмент» нет, поэтому
  нет и нового класса дрейфа;
* откат — снятие флага, как и у остальных фаз change'а.

Модуль намеренно без зависимостей и без ROS2: файлы читает нода, сюда
приходит уже прочитанный текст.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Mapping

__all__ = [
    "MARKER_PREFIX",
    "MovedSection",
    "PromptMarkupError",
    "RenderedPrompt",
    "merge_skill_prompts",
    "render_prompt",
]

#: Любая строка, начинающаяся с этого префикса, обязана быть известным
#: маркером — иначе опечатка в разметке молча уехала бы в промпт.
MARKER_PREFIX = "<<<"

_MOVE_OPEN = re.compile(r"^<<<SKILL-MOVE\s+(?P<skills>[^<>]+)>>>$")
_MOVE_CLOSE = "<<<SKILL-MOVE-END>>>"
_OFF_OPEN = "<<<SKILL-OFF>>>"
_OFF_CLOSE = "<<<SKILL-OFF-END>>>"
_ON_OPEN = "<<<SKILL-ON>>>"
_ON_CLOSE = "<<<SKILL-ON-END>>>"

_CLOSERS = {_MOVE_CLOSE, _OFF_CLOSE, _ON_CLOSE}
_CLOSER_FOR_MODE = {"move": _MOVE_CLOSE, "off": _OFF_CLOSE, "on": _ON_CLOSE}


class PromptMarkupError(ValueError):
    """Разметка секций сломана: незакрытый блок, вложенность, опечатка.

    Поднимается только парсером. Нода ловит её и остаётся в режиме
    выключенных скиллов: сломанный промпт не имеет права ронять робота,
    но и тихо ехать в LLM в неизвестном виде не должен.
    """


@dataclass(frozen=True)
class MovedSection:
    """Блок доменных правил, уехавший из промпта во фрагменты скиллов."""

    #: Имена скиллов из каталога, которым принадлежит блок.
    skills: tuple[str, ...]
    #: Текст блока без маркеров, с обрезанными краевыми пустыми строками.
    text: str
    #: Номер строки открывающего маркера (1-based) — для сообщений об ошибках.
    line: int


@dataclass(frozen=True)
class RenderedPrompt:
    """Результат рендера: системный промпт плюс уехавшие блоки."""

    system_prompt: str
    sections: tuple[MovedSection, ...]

    def by_skill(self) -> dict[str, str]:
        """Собрать блоки по скиллам в порядке появления в промпте."""
        grouped: dict[str, list[str]] = {}
        for section in self.sections:
            for skill in section.skills:
                grouped.setdefault(skill, []).append(section.text)
        return {skill: "\n\n".join(blocks) for skill, blocks in grouped.items()}


def _parse_skills(raw: str, line: int) -> tuple[str, ...]:
    skills = tuple(part.strip() for part in raw.split(",") if part.strip())
    if not skills:
        raise PromptMarkupError(
            f"строка {line}: <<<SKILL-MOVE …>>> без имён скиллов — "
            f"непонятно, куда переезжает блок"
        )
    return skills


def _tidy(lines: list[str]) -> str:
    """Склеить строки, схлопнув пустые пачки, оставшиеся после выреза."""
    out: list[str] = []
    blanks = 0
    for line in lines:
        if line.strip():
            blanks = 0
            out.append(line)
            continue
        blanks += 1
        if blanks <= 1:
            out.append(line)
    text = "\n".join(out)
    return text.rstrip("\n") + "\n" if text.strip() else ""


def render_prompt(text: str, *, skills_enabled: bool) -> RenderedPrompt:
    """Развернуть разметку секций под конкретный режим.

    Args:
        text: содержимое мастер-промпта как оно лежит на диске.
        skills_enabled: значение параметра ``skills_enabled`` ноды.

    Returns:
        Системный промпт без маркеров и, при ``skills_enabled=True``, блоки,
        уехавшие во фрагменты скиллов.

    Raises:
        PromptMarkupError: разметка сломана (незакрытый блок, вложенность,
            неизвестный маркер, ``SKILL-MOVE`` без имён).
    """
    kept: list[str] = []
    sections: list[MovedSection] = []

    mode: str | None = None          # "move" | "off" | "on"
    buffer: list[str] = []
    skills: tuple[str, ...] = ()
    opened_at = 0

    for number, line in enumerate(text.split("\n"), start=1):
        stripped = line.strip()

        if not stripped.startswith(MARKER_PREFIX):
            (buffer if mode is not None else kept).append(line)
            continue

        if stripped in _CLOSERS:
            if mode is None:
                raise PromptMarkupError(
                    f"строка {number}: {stripped} без открывающего маркера"
                )
            if _CLOSER_FOR_MODE[mode] != stripped:
                raise PromptMarkupError(
                    f"строка {number}: {stripped} закрывает не тот блок "
                    f"(открыт на строке {opened_at})"
                )
            if mode == "move":
                block = _tidy(buffer).strip()
                if skills_enabled:
                    if block:
                        sections.append(
                            MovedSection(
                                skills=skills, text=block, line=opened_at
                            )
                        )
                else:
                    kept.extend(buffer)
            elif (mode == "on") == skills_enabled:
                kept.extend(buffer)
            mode, buffer, skills, opened_at = None, [], (), 0
            continue

        if mode is not None:
            raise PromptMarkupError(
                f"строка {number}: {stripped} внутри блока, открытого на "
                f"строке {opened_at} — вложенность не поддерживается"
            )
        move = _MOVE_OPEN.match(stripped)
        if move:
            mode = "move"
            skills = _parse_skills(move.group("skills"), number)
        elif stripped == _OFF_OPEN:
            mode = "off"
        elif stripped == _ON_OPEN:
            mode = "on"
        else:
            raise PromptMarkupError(
                f"строка {number}: неизвестный маркер {stripped!r}"
            )
        opened_at = number

    if mode is not None:
        raise PromptMarkupError(
            f"блок, открытый на строке {opened_at}, не закрыт до конца файла"
        )

    return RenderedPrompt(system_prompt=_tidy(kept), sections=tuple(sections))


def merge_skill_prompts(
    fragments: Mapping[str, str], rendered: RenderedPrompt
) -> dict[str, str]:
    """Приклеить уехавшие блоки к фрагментам соответствующих скиллов.

    Скилл, у которого своего файла нет, получает запись из одних только
    переехавших блоков: терять доменные правила из-за отсутствующего файла
    нельзя — ровно ради этого секции и размечены.
    """
    merged = dict(fragments)
    for skill, block in rendered.by_skill().items():
        own = merged.get(skill, "").strip()
        merged[skill] = f"{own}\n\n{block}" if own else block
    return merged
