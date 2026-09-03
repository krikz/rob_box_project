"""Фаза 1 change'а skill-scoped-dialogue-context — объявление скиллов.

Спека (``specs/dialogue-skill-context/spec.md``), требование «Скилл
объявляется поверх каталога инструментов», три сценария:

* скилл ссылается на существующий инструмент → доступен с ТЕМ ЖЕ
  описанием и схемой, что в каталоге (никакой второй копии контракта);
* скилл ссылается на несуществующий → падение с именем скилла и имени;
* инструмент в нескольких скиллах → допустимо, описание одно.

Класс ошибок, ради которого это существует: Compositor (`e96b912d`)
держал собственные обёртки инструментов, они разошлись с ``execute()``,
и каждый LLM-вызов навигации падал на валидации.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

from rob_box_core.tool_catalog import (
    CORE_SKILL,
    TOOL_CATALOG,
    get_tool,
    llm_visible_tools,
    skill_names,
    tools_for_skill,
)
from rob_box_harness.core.tool_registry import ToolRegistry

_REPO_ROOT = Path(__file__).resolve().parents[3]
_GENERATOR = _REPO_ROOT / "tools" / "gen_tool_catalog.py"


# ── Требование: скилл объявляется поверх каталога ────────────────────────


def test_skill_tools_reuse_the_catalog_contract() -> None:
    """Сценарий «скилл ссылается на существующий инструмент».

    Описание и схема параметров ДОЛЖНЫ быть ровно теми же объектами
    значений, что и в каталоге. Если бы скилл нёс свою копию, она бы
    разошлась — ровно это и случилось с Compositor.
    """
    for entry in tools_for_skill("composer"):
        canonical = get_tool(entry.name)
        assert entry.description == canonical.description
        assert dict(entry.parameters) == dict(canonical.parameters)


def test_every_llm_visible_tool_belongs_to_a_skill() -> None:
    """«Сирот нет» — проверка тестом, а не глазами (задача 1.3).

    Инструмент без скилла при включённом сужении каталога стал бы
    невидимым для LLM молча.
    """
    orphans = sorted(e.name for e in llm_visible_tools() if not e.skill)
    assert orphans == [], f"инструменты без скилла: {orphans}"


def test_hidden_tools_are_exempt_from_skill_assignment() -> None:
    """Скрытый от LLM инструмент в скилл не обязан входить.

    ``generate_music`` скрыт (MiniMax Music API отдаёт 410) — предъявлять
    его незачем, значит и относить к домену незачем.
    """
    hidden = [e for e in TOOL_CATALOG if not e.llm_visible]
    assert hidden, "в каталоге не осталось скрытых инструментов — обнови тест"
    for entry in hidden:
        assert entry.skill == ()


def test_tool_may_belong_to_several_skills() -> None:
    """Сценарий «инструмент принадлежит нескольким скиллам».

    ``stop_music`` нужен композитору, диджею и плееру. Описание при этом
    одно на всех — оно берётся из каталога.
    """
    stop_music = get_tool("stop_music")
    assert len(stop_music.skill) > 1
    for skill in stop_music.skill:
        names = {e.name for e in tools_for_skill(skill)}
        assert "stop_music" in names


def test_unknown_skill_raises_instead_of_returning_empty() -> None:
    """Молчаливый пустой набор доезжает до юзера как «нет такой функции»."""
    with pytest.raises(KeyError) as excinfo:
        tools_for_skill("нет-такого-скилла")
    message = str(excinfo.value)
    assert "нет-такого-скилла" in message
    assert "composer" in message, "сообщение должно перечислять известные скиллы"


def test_core_skill_is_always_included() -> None:
    """``core`` предъявляется всегда — речь нужна в любом ходу."""
    assert CORE_SKILL in skill_names()
    for skill in skill_names():
        names = {e.name for e in tools_for_skill(skill)}
        assert "speak_text" in names, f"{skill} остался без speak_text"


def test_no_active_skill_yields_core_only() -> None:
    """«Скилл не активирован» — это ровно core, а не пустота."""
    selected = tools_for_skill()
    assert {e.name for e in selected} == {
        e.name for e in llm_visible_tools() if CORE_SKILL in e.skill
    }
    assert selected, "core не может быть пустым"


def test_narrowing_is_smaller_than_the_full_catalog() -> None:
    """Ради чего Move B вообще существует."""
    full = len(llm_visible_tools())
    for skill in ("composer", "player", "navigation"):
        narrowed = len(tools_for_skill(skill))
        assert 0 < narrowed < full


# ── Реестр harness'а ─────────────────────────────────────────────────────


def test_registry_default_returns_the_full_catalog() -> None:
    """Без ``skills`` поведение прежнее — это дефолт с выключенным Move B."""
    registry = ToolRegistry()
    assert len(registry.list_tools()) == len(llm_visible_tools())


def test_registry_narrows_by_skill() -> None:
    registry = ToolRegistry()
    narrowed = registry.list_tools(skills=("composer",))
    assert {s.name for s in narrowed} == {
        e.name for e in tools_for_skill("composer")
    }


def test_registry_unknown_skill_raises() -> None:
    registry = ToolRegistry()
    with pytest.raises(KeyError):
        registry.list_tools(skills=("нет-такого",))


# ── Генератор: обе стороны связи проверяются на сборке ───────────────────


def _load_generator():
    """Импортировать генератор как модуль.

    Запускать его копию из tmp_path нельзя: он вычисляет ``REPO_ROOT`` и
    ``TOOLS_DIR`` от собственного ``__file__``, поэтому в песочнице не
    находит классы инструментов и падает раньше проверяемой защиты.
    """
    import importlib.util

    spec = importlib.util.spec_from_file_location("_gen_tool_catalog", _GENERATOR)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _catalog_entries() -> list[dict]:
    """Записи каталога в том виде, в каком их видит ``_assign_skills``."""
    return [
        {"name": e.name, "llm_visible": e.llm_visible}
        for e in TOOL_CATALOG
    ]


@pytest.mark.skipif(not _GENERATOR.exists(), reason="генератор не найден")
def test_generator_rejects_a_skill_naming_an_unknown_tool() -> None:
    """Задача 1.5 / сценарий «скилл ссылается на несуществующий инструмент».

    Опечатка или переименование инструмента ДОЛЖНЫ ронять сборку, а не
    доезжать до робота скиллом, обещающим несуществующую функцию.
    """
    gen = _load_generator()
    gen.SKILL_TOOLS = dict(gen.SKILL_TOOLS)
    gen.SKILL_TOOLS["scheduler"] = ("task_delta", "нет_такого_инструмента")

    with pytest.raises(SystemExit) as excinfo:
        gen._assign_skills(_catalog_entries())

    message = str(excinfo.value)
    assert "нет_такого_инструмента" in message
    assert "scheduler" in message, "сообщение должно называть скилл"


@pytest.mark.skipif(not _GENERATOR.exists(), reason="генератор не найден")
def test_generator_rejects_an_orphan_tool() -> None:
    """Инструмент, выпавший из всех скиллов, ДОЛЖЕН ронять сборку.

    При включённом сужении каталога такой инструмент стал бы невидимым
    для LLM молча — это регрессия класса #1403 («нет такой функции»).
    """
    gen = _load_generator()
    gen.SKILL_TOOLS = dict(gen.SKILL_TOOLS)
    gen.SKILL_TOOLS["scheduler"] = ()

    with pytest.raises(SystemExit) as excinfo:
        gen._assign_skills(_catalog_entries())

    assert "task_delta" in str(excinfo.value)


@pytest.mark.skipif(not _GENERATOR.exists(), reason="генератор не найден")
def test_generator_accepts_the_current_assignment() -> None:
    """Обратная сторона: сегодняшний расклад проходит обе проверки."""
    gen = _load_generator()
    entries = _catalog_entries()
    gen._assign_skills(entries)
    assert all(e["skill"] or not e["llm_visible"] for e in entries)


@pytest.mark.skipif(not _GENERATOR.exists(), reason="генератор не найден")
def test_generated_catalog_is_current() -> None:
    """Задача 1.6 — поле аддитивно, дрейф каталога не появился."""
    result = subprocess.run(
        [sys.executable, str(_GENERATOR), "--check"],
        capture_output=True,
        text=True,
        cwd=str(_REPO_ROOT),
        encoding="utf-8",
        errors="replace",
    )
    assert result.returncode == 0, result.stdout + result.stderr
