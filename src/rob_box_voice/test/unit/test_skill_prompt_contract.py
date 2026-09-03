"""Фаза 4 change'а skill-scoped-dialogue-context — защита от расхождения.

Спека, требование «Защита от расхождения скилла и каталога»: сборка ДОЛЖНА
падать, если инструмент, входящий в скилл, не упомянут в тексте этого
скилла.

Почему блокер, а не warning. Сегодняшняя проверка в ``dialogue_node``
(``_validate_tools_in_prompt``) покрывает ТОЛЬКО музыкальный промпт и
только предупреждением. Ровно этот класс расхождения дал #1403:
``generate_music`` был зарегистрирован, но в тексте не упомянут — и LLM
уверенно отвечала «нет такой функции», уходя на Renardo. Предупреждение
в логе ноды такое не ловит: его никто не читает до инцидента.

Второе требование здесь же — «Инварианты не уходят в скиллы»: §1 CORE
INVARIANTS и §2 ANTI-DUP остаются единственным каноническим контрактом в
мастер-промпте. Копия правила во фрагменте скилла создала бы второй
источник правды, а мастер-промпт прямо запрещает переопределять §2 из
других секций.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from rob_box_core.tool_catalog import (
    CORE_SKILL,
    skill_names,
    tools_for_skill,
)

_PROMPTS = Path(__file__).resolve().parents[2] / "prompts"
_SKILLS_DIR = _PROMPTS / "skills"

#: Правила, которые обязаны жить ТОЛЬКО в мастер-промпте. Фрагмент скилла,
#: переобъявляющий любое из них, роняет сборку: два источника правды по
#: анти-дубликату речи — это молчащий или заикающийся робот.
_INVARIANT_MARKERS: tuple[str, ...] = (
    "RULE #LANG",
    "RULE #UNICODE-SPEECH",
    "RULE #SYSCTX",
    "ANTI-DUP",
    "ANTI-DUPLICATE",
    "CYCLE TERMINATION",
)


def _fragment_path(skill: str) -> Path:
    return _SKILLS_DIR / f"{skill}.txt"


def _declared_skills() -> list[str]:
    return list(skill_names())


def _fragment_text(skill: str) -> str:
    return _fragment_path(skill).read_text(encoding="utf-8")


# ── Требование: защита от расхождения скилла и каталога ──────────────────


@pytest.mark.parametrize("skill", _declared_skills())
def test_every_declared_skill_has_a_fragment(skill: str) -> None:
    """У объявленного в каталоге скилла должен быть текст.

    Скилл без текста — это Move A, который для этого домена молча не
    работает: инструменты предъявлены, инструкций нет.
    """
    assert _fragment_path(skill).exists(), (
        f"нет файла prompts/skills/{skill}.txt для скилла, объявленного "
        f"в tools/gen_tool_catalog.py::SKILL_TOOLS"
    )


@pytest.mark.parametrize("skill", _declared_skills())
def test_every_tool_of_a_skill_is_named_in_its_fragment(skill: str) -> None:
    """Сценарий «инструмент скилла не упомянут в его тексте» → падение.

    Регрессия класса #1403: инструмент есть, в тексте его нет, LLM
    отвечает «нет такой функции» и уходит на другой путь.
    """
    text = _fragment_text(skill).lower()
    # core добавляется к каждому скиллу на предъявлении, но описывать его
    # инструменты обязан только сам core — иначе каждый фрагмент пришлось
    # бы дублировать speak_text и статусные тулы.
    tools = tools_for_skill(skill, include_core=(skill == CORE_SKILL))
    missing = sorted(e.name for e in tools if e.name.lower() not in text)
    assert not missing, (
        f"скилл {skill!r} включает инструменты, не упомянутые в "
        f"prompts/skills/{skill}.txt: {', '.join(missing)}. "
        f"LLM ответит «нет такой функции» (класс регрессии #1403)."
    )


@pytest.mark.parametrize("skill", _declared_skills())
def test_fragment_does_not_name_tools_it_does_not_own(skill: str) -> None:
    """Фрагмент не должен рекламировать чужие инструменты.

    Иначе скилл обещает то, чего при включённом сужении каталога (Move B)
    в его наборе не будет. Упоминание ИМЕНИ ДРУГОГО СКИЛЛА разрешено —
    это маршрутизация («за готовыми mp3 иди в player»), а не обещание.
    """
    text = _fragment_text(skill).lower()
    own = {e.name for e in tools_for_skill(skill, include_core=True)}
    foreign = sorted(
        e.name
        for other in skill_names()
        for e in tools_for_skill(other, include_core=False)
        if e.name not in own and e.name.lower() in text
    )
    assert not foreign, (
        f"prompts/skills/{skill}.txt называет чужие инструменты: "
        f"{', '.join(sorted(set(foreign)))}"
    )


# ── Требование: инварианты не уходят в скиллы ────────────────────────────


@pytest.mark.parametrize("skill", _declared_skills())
def test_fragment_does_not_redeclare_invariants(skill: str) -> None:
    """Сценарий «попытка объявить инвариант частью скилла» → падение."""
    text = _fragment_text(skill)
    duplicated = [marker for marker in _INVARIANT_MARKERS if marker in text]
    assert not duplicated, (
        f"prompts/skills/{skill}.txt дублирует инвариант(ы) "
        f"{', '.join(duplicated)}. Инварианты живут только в "
        f"мастер-промпте — иначе появляется второй источник правды."
    )


def test_master_prompt_still_owns_the_invariants() -> None:
    """Обратная сторона: инварианты не должны исчезнуть из мастер-промпта.

    Раскол промпта по скиллам (фаза 5) не имеет права унести §1/§2.
    """
    master = (_PROMPTS / "master_prompt_compact.txt").read_text(encoding="utf-8")
    for marker in ("RULE #LANG", "RULE #UNICODE-SPEECH", "RULE #SYSCTX"):
        assert marker in master, (
            f"{marker} пропал из master_prompt_compact.txt — это инвариант, "
            f"он обязан оставаться в системном промпте при любом раскладе"
        )


# ── Санитарные проверки самих фрагментов ─────────────────────────────────


@pytest.mark.parametrize("skill", _declared_skills())
def test_fragment_is_not_empty(skill: str) -> None:
    assert _fragment_text(skill).strip(), f"prompts/skills/{skill}.txt пуст"


def test_no_orphan_fragments() -> None:
    """Файл-фрагмент без скилла в каталоге — мусор, который никто не грузит."""
    declared = set(skill_names())
    # Файлы прежней (мёртвой) схемы именования разбираются в задаче 6.2;
    # здесь проверяем только новые, названные по имени скилла.
    orphans = sorted(
        path.stem
        for path in _SKILLS_DIR.glob("*.txt")
        if not path.stem.endswith("_skill_prompt") and path.stem not in declared
    )
    assert not orphans, f"фрагменты без скилла в каталоге: {', '.join(orphans)}"
