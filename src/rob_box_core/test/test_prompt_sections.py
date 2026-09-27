"""Разметка доменных секций мастер-промпта (фаза 5 change'а).

Спека, требование «Инструкции скилла доставляются рядом с текущим ходом»:
доменные правила не имеют права стоять на позиции 0, когда скиллы включены.
Фаза 5 переносит §5 MUSIC и §6 WAYPOINTS — но перенос обязан быть
обратимым флагом, иначе дефолтная конфигурация (``skills_enabled=false``,
фрагменты не грузятся) потеряет эти правила совсем.

Отсюда контракт парсера, который здесь и проверяется:

* при ``skills_enabled=false`` из текста уходят ТОЛЬКО строки-маркеры;
* при ``true`` размеченные блоки уходят из промпта и появляются у своих
  скиллов — ни один блок не теряется по дороге;
* сломанная разметка — исключение, а не тихо уехавший в LLM мусор.
"""

from __future__ import annotations

import pytest

from rob_box_core.prompt_sections import (
    PromptMarkupError,
    merge_skill_prompts,
    render_prompt,
)

_PROMPT = "\n".join(
    [
        "шапка",
        "<<<SKILL-OFF>>>",
        "маршрут на §5",
        "<<<SKILL-OFF-END>>>",
        "<<<SKILL-ON>>>",
        "правила приезжают скиллом",
        "<<<SKILL-ON-END>>>",
        "",
        "<<<SKILL-MOVE composer,dj>>>",
        "# 5. MUSIC",
        "играй кодом",
        "<<<SKILL-MOVE-END>>>",
        "",
        "<<<SKILL-MOVE navigation>>>",
        "# 6. WAYPOINTS",
        "<<<SKILL-MOVE-END>>>",
        "",
        "подвал",
        "",
    ]
)


# ── skills_enabled=false: ничего не двигается ────────────────────────────


def test_disabled_keeps_every_line_but_the_markers() -> None:
    """Дефолтная конфигурация обязана совпадать с доскилловой."""
    rendered = render_prompt(_PROMPT, skills_enabled=False)
    assert rendered.system_prompt == (
        "шапка\n"
        "маршрут на §5\n"
        "\n"
        "# 5. MUSIC\n"
        "играй кодом\n"
        "\n"
        "# 6. WAYPOINTS\n"
        "\n"
        "подвал\n"
    )


def test_disabled_moves_nothing_into_skills() -> None:
    assert render_prompt(_PROMPT, skills_enabled=False).sections == ()


def test_disabled_drops_the_skills_on_text() -> None:
    """Текст «правила приезжают скиллом» при выключенных скиллах — враньё."""
    prompt = render_prompt(_PROMPT, skills_enabled=False).system_prompt
    assert "правила приезжают скиллом" not in prompt


# ── skills_enabled=true: блоки уезжают ровно к своим скиллам ─────────────


def test_enabled_removes_moved_blocks_from_the_prompt() -> None:
    prompt = render_prompt(_PROMPT, skills_enabled=True).system_prompt
    assert "# 5. MUSIC" not in prompt
    assert "# 6. WAYPOINTS" not in prompt
    assert "шапка" in prompt and "подвал" in prompt


def test_enabled_swaps_the_routing_text() -> None:
    """Задача 5.4: §0 не должен маршрутизировать на удалённые секции."""
    prompt = render_prompt(_PROMPT, skills_enabled=True).system_prompt
    assert "маршрут на §5" not in prompt
    assert "правила приезжают скиллом" in prompt


def test_one_block_can_belong_to_several_skills() -> None:
    by_skill = render_prompt(_PROMPT, skills_enabled=True).by_skill()
    assert "играй кодом" in by_skill["composer"]
    assert "играй кодом" in by_skill["dj"]
    assert "# 6. WAYPOINTS" in by_skill["navigation"]
    assert set(by_skill) == {"composer", "dj", "navigation"}


def test_nothing_is_lost_between_the_two_modes() -> None:
    """Каждая содержательная строка либо в промпте, либо во фрагменте.

    Исключение — блоки ``SKILL-OFF``/``SKILL-ON``: это две редакции одного
    и того же куска (маршрутизация §0), и ровно одна из них уместна в
    каждом режиме. Всё остальное обязано доехать до LLM в любом режиме.
    """
    swapped = {"маршрут на §5", "правила приезжают скиллом"}
    enabled = render_prompt(_PROMPT, skills_enabled=True)
    surface = enabled.system_prompt + "\n" + "\n".join(
        enabled.by_skill().values()
    )
    for line in render_prompt(_PROMPT, skills_enabled=False).system_prompt.split(
        "\n"
    ):
        if line.strip() and line not in swapped:
            assert line in surface, f"строка потерялась при расколе: {line!r}"


# ── слияние с файлами фрагментов ─────────────────────────────────────────


def test_merge_appends_to_the_existing_fragment() -> None:
    rendered = render_prompt(_PROMPT, skills_enabled=True)
    merged = merge_skill_prompts({"composer": "[СКИЛЛ composer]"}, rendered)
    assert merged["composer"].startswith("[СКИЛЛ composer]")
    assert "играй кодом" in merged["composer"]


def test_merge_creates_a_fragment_when_the_file_is_missing() -> None:
    """Скилл без файла всё равно получает переехавшие правила.

    Иначе отсутствие ``prompts/skills/navigation.txt`` молча съело бы §6.
    """
    merged = merge_skill_prompts({}, render_prompt(_PROMPT, skills_enabled=True))
    assert "# 6. WAYPOINTS" in merged["navigation"]


def test_merge_does_not_touch_untouched_skills() -> None:
    merged = merge_skill_prompts(
        {"memory": "текст memory"}, render_prompt(_PROMPT, skills_enabled=True)
    )
    assert merged["memory"] == "текст memory"


# ── сломанная разметка ───────────────────────────────────────────────────


@pytest.mark.parametrize(
    "broken",
    [
        pytest.param("<<<SKILL-MOVE composer>>>\nтекст", id="не закрыт"),
        pytest.param("текст\n<<<SKILL-MOVE-END>>>", id="закрыт без открытия"),
        pytest.param(
            "<<<SKILL-MOVE composer>>>\n<<<SKILL-ON>>>\nx\n"
            "<<<SKILL-ON-END>>>\n<<<SKILL-MOVE-END>>>",
            id="вложенность",
        ),
        pytest.param(
            "<<<SKILL-MOVE composer>>>\nx\n<<<SKILL-ON-END>>>", id="чужой закрыватель"
        ),
        pytest.param("<<<SKILL-MOVE >>>\nx\n<<<SKILL-MOVE-END>>>", id="без скиллов"),
        pytest.param("<<<SKILL-MOOVE composer>>>", id="опечатка в маркере"),
    ],
)
def test_broken_markup_raises(broken: str) -> None:
    with pytest.raises(PromptMarkupError):
        render_prompt(broken, skills_enabled=True)


def test_broken_markup_raises_in_both_modes() -> None:
    """Опечатку обязан ловить и выключенный режим — иначе она доедет молча."""
    with pytest.raises(PromptMarkupError):
        render_prompt("<<<SKILL-WAT>>>", skills_enabled=False)


def test_prompt_without_markers_survives_untouched() -> None:
    """Старый промпт без разметки — не ошибка: он просто едет как есть."""
    plain = "строка один\nстрока два\n"
    for enabled in (False, True):
        rendered = render_prompt(plain, skills_enabled=enabled)
        assert rendered.system_prompt == plain
        assert rendered.sections == ()
