"""Фаза 5 change'а skill-scoped-dialogue-context — раскол мастер-промпта.

Задачи 5.1-5.4. §5 MUSIC (2 181 tok) и §6 WAYPOINTS (184 tok) размечены в
``master_prompt_compact.txt`` как секции доменных скиллов. Разметка, а не
удаление текста, потому что ``skills_enabled`` по умолчанию ``false``: при
выключенном флаге фрагменты скиллов не грузятся, и вырезанные секции не
приехали бы ниоткуда — робот в дефолтной конфигурации потерял бы правила
музыки и навигации целиком.

Отсюда два требования, которые здесь пришпилены:

* ``skills_enabled=false`` — промпт совпадает с сегодняшним, до строки;
* ``skills_enabled=true``  — §5/§6 из системного промпта ушли, целиком
  доехали до своих скиллов, а §1/§2 (инварианты) остались на месте.
"""

from __future__ import annotations

import re

from pathlib import Path

import pytest

from rob_box_core.prompt_sections import (
    MARKER_PREFIX,
    merge_skill_prompts,
    render_prompt,
)
from rob_box_core.tool_catalog import skill_names, tools_for_skill

_PROMPTS = Path(__file__).resolve().parents[2] / "prompts"
_MASTER = _PROMPTS / "master_prompt_compact.txt"
_SKILLS_DIR = _PROMPTS / "skills"

#: Секции, которые уезжают в скиллы (задачи 5.1 и 5.2).
_MOVED_MARKERS = ("# 5. MUSIC", "### DJ MODE", "# 6. WAYPOINTS")

#: Инварианты — остаются в системном промпте при любом флаге (задача 5.3).
_INVARIANTS = (
    "# 1. CORE INVARIANTS",
    "# 2. CYCLE TERMINATION",
    "RULE #LANG",
    "RULE #UNICODE-SPEECH",
    "RULE #SYSCTX",
    "RULE #MUSIC",
)


@pytest.fixture(scope="module")
def raw() -> str:
    return _MASTER.read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def disabled(raw: str):
    return render_prompt(raw, skills_enabled=False)


@pytest.fixture(scope="module")
def enabled(raw: str):
    return render_prompt(raw, skills_enabled=True)


def _fragment(skill: str) -> str:
    path = _SKILLS_DIR / f"{skill}.txt"
    return path.read_text(encoding="utf-8") if path.exists() else ""


# ── skills_enabled=false: дефолтная конфигурация не изменилась ───────────


def test_disabled_prompt_is_the_file_minus_markup(raw: str, disabled) -> None:
    """Независимый пересчёт того же результата: маркеры и SKILL-ON долой.

    Проверка намеренно НЕ переиспользует парсер — иначе она доказывала бы
    сама себя. Если эти две реализации разойдутся, значит парсер трогает
    текст промпта, а он не имеет права.
    """
    expected: list[str] = []
    skipping = False
    for line in raw.split("\n"):
        stripped = line.strip()
        if stripped == "<<<SKILL-ON>>>":
            skipping = True
            continue
        if stripped == "<<<SKILL-ON-END>>>":
            skipping = False
            continue
        if stripped.startswith(MARKER_PREFIX) or skipping:
            continue
        expected.append(line)
    assert disabled.system_prompt == "\n".join(expected).rstrip("\n") + "\n"


@pytest.mark.parametrize("marker", _MOVED_MARKERS)
def test_disabled_prompt_still_carries_the_domain_sections(
    disabled, marker: str
) -> None:
    """Скиллы выключены → доменные правила обязаны остаться в промпте."""
    assert marker in disabled.system_prompt


def test_disabled_moves_nothing(disabled) -> None:
    assert disabled.sections == ()


def test_no_marker_leaks_into_the_llm(disabled, enabled) -> None:
    for text in (disabled.system_prompt, enabled.system_prompt,
                 *enabled.by_skill().values()):
        assert MARKER_PREFIX not in text


# ── skills_enabled=true: секции уехали к своим скиллам ───────────────────


@pytest.mark.parametrize("marker", _MOVED_MARKERS)
def test_enabled_prompt_no_longer_carries_the_domain_sections(
    enabled, marker: str
) -> None:
    assert marker not in enabled.system_prompt


def test_moved_sections_land_in_declared_skills(enabled) -> None:
    """Задача 5.1/5.2: §5 → composer/dj/player, §6 → navigation."""
    by_skill = enabled.by_skill()
    assert set(by_skill) == {"composer", "dj", "player", "navigation"}
    assert "execute_music_code" in by_skill["composer"]
    assert "set_vibe_preset" in by_skill["composer"]
    assert "gen_play_from_library" in by_skill["player"]
    assert "set_dj_mode" in by_skill["dj"]
    assert "navigate_to_waypoint" in by_skill["navigation"]


def test_every_target_is_a_declared_skill(enabled) -> None:
    """Опечатка в имени скилла = блок, который никому не доедет."""
    declared = set(skill_names())
    for section in enabled.sections:
        unknown = sorted(set(section.skills) - declared)
        assert not unknown, (
            f"строка {section.line}: секция уезжает в несуществующие скиллы "
            f"{', '.join(unknown)} — в каталоге их нет"
        )


def test_renardo_rules_reach_the_dj_too(enabled) -> None:
    """DJ генерирует свежий трек кодом — без палитры синтов он слеп."""
    dj = enabled.by_skill()["dj"]
    assert "Synth palette" in dj
    assert "FIRE-AND-FORGET" in dj


def test_nothing_is_lost_when_the_sections_move(disabled, enabled) -> None:
    """Каждая строка §5/§6 доехала до LLM — просто другим маршрутом.

    Исключение — блоки ``SKILL-OFF``/``SKILL-ON``: это две редакции
    маршрутных строк §0 и §4, и ровно одна уместна в каждом режиме.
    """
    swapped = {
        line.strip()
        for line in _swapped_lines(_MASTER.read_text(encoding="utf-8"))
    }
    surface = enabled.system_prompt + "\n" + "\n".join(enabled.by_skill().values())
    missing = [
        line for line in disabled.system_prompt.split("\n")
        if line.strip() and line.strip() not in swapped and line not in surface
    ]
    assert not missing, f"строки потерялись при расколе: {missing[:5]}"


def _swapped_lines(raw: str) -> list[str]:
    """Строки внутри блоков ``SKILL-OFF``/``SKILL-ON`` (две редакции одного)."""
    out: list[str] = []
    inside = False
    for line in raw.split("\n"):
        stripped = line.strip()
        if stripped in ("<<<SKILL-OFF>>>", "<<<SKILL-ON>>>"):
            inside = True
            continue
        if stripped in ("<<<SKILL-OFF-END>>>", "<<<SKILL-ON-END>>>"):
            inside = False
            continue
        if inside:
            out.append(line)
    return out


# ── задача 5.3: инварианты не уезжают ────────────────────────────────────


@pytest.mark.parametrize("marker", _INVARIANTS)
def test_invariants_stay_in_the_system_prompt(disabled, enabled, marker: str) -> None:
    assert marker in disabled.system_prompt
    assert marker in enabled.system_prompt, (
        f"{marker} уехал из системного промпта — §1/§2 обязаны остаться "
        f"единственным каноническим контрактом при любом флаге"
    )


@pytest.mark.parametrize("marker", _INVARIANTS)
def test_no_invariant_travels_inside_a_moved_section(enabled, marker: str) -> None:
    """Второй источник правды по анти-дубликату — молчащий робот."""
    for section in enabled.sections:
        assert marker not in section.text, (
            f"секция со строки {section.line} уносит инвариант {marker} "
            f"в скиллы {', '.join(section.skills)}"
        )


def test_moved_sections_may_still_reference_the_invariants(enabled) -> None:
    """Ссылка на §2 — это маршрутизация, а не переобъявление.

    Мастер-промпт прямо разрешает ссылаться и запрещает переопределять;
    §5 ссылается на §2 в блоке про музыку с текстом, и эта ссылка обязана
    пережить переезд — иначе модель не узнает, где канонический контракт.
    """
    assert "see §2" in enabled.by_skill()["composer"]


# ── задача 5.4: §0 и §4 перестают маршрутизировать на удалённые секции ───


def test_orientation_does_not_route_to_removed_sections(enabled) -> None:
    """Ни одной ссылки на §5/§6, когда этих секций в промпте нет.

    Проверка нарочно грубая — по номеру секции, а не по списку известных
    строк: любая новая ссылка на уехавшую секцию обязана падать здесь, а
    не тихо отправлять модель читать пустоту.
    """
    lies = [
        line for line in enabled.system_prompt.split("\n")
        if "§5" in line or "§6" in line
    ]
    assert not lies, (
        f"маршрутизация на удалённые секции осталась (задача 5.4): {lies}"
    )


def test_orientation_explains_where_the_domain_rules_went(enabled) -> None:
    prompt = enabled.system_prompt
    assert "load_skill" in prompt
    assert "блоком СКИЛЛА" in prompt


def test_disabled_orientation_keeps_the_old_routing(disabled) -> None:
    """При выключенных скиллах §0 маршрутизирует как раньше."""
    prompt = disabled.system_prompt
    assert "§5 MUSIC (Renardo + library + DJ)" in prompt
    assert "§6 WAYPOINTS" in prompt
    assert "load_skill" not in prompt


# ── эффективный фрагмент, который реально увидит LLM ─────────────────────


def test_effective_fragment_keeps_the_tool_contract(enabled) -> None:
    """Слияние «файл фрагмента + переехавшие секции» ничего не теряет."""
    merged = merge_skill_prompts(
        {skill: _fragment(skill) for skill in skill_names()}, enabled
    )
    composer = merged["composer"]
    assert composer.startswith("[СКИЛЛ composer")
    assert "Synth palette" in composer
    assert "compose_music" in composer


#: Инструменты, которые переехавшие секции называют, НЕ владея ими.
#: Пока Move B выключен (``skill_tool_narrowing=false``), LLM видит весь
#: каталог и упоминание безобидно. При включённом сужении каждая строка
#: отсюда — потенциальный #1403 («нет такой функции»), поэтому список
#: явный: новое упоминание чужого инструмента обязано быть осознанным.
def _mentions_tool(text: str, name: str) -> bool:
    """Упомянут ли инструмент ``name`` в ``text`` как отдельное слово.

    Раньше здесь была голая подстрока ``name.lower() in text``. Она
    работала, пока все имена были длинными и «своими» (``play_sound``,
    ``execute_music_code``). AV-21 (#1956) добавил в каталог инструмент
    ``say`` — три буквы, совпадающие с обычным английским глаголом. После
    этого секция про Renardo, где написано «say ONE short accept via
    speak_text», стала считаться рекламой чужого инструмента, и develop
    покраснел на ровном месте.

    Границы слова оставляют смысл проверки прежним (секция не должна
    звать чужой инструмент), но перестают ловить текст, который просто
    содержит эти буквы.
    """
    return re.search(rf"(?<![a-z0-9_]){re.escape(name.lower())}(?![a-z0-9_])", text) is not None


_KNOWN_FOREIGN_MENTIONS: dict[str, dict[str, str]] = {
    "### Renardo (execute_music_code) — local synth, ~1s start": {
        "play_sound": "контраст: «play_sound только для эффектов <5s»",
        "estimate_tts_duration": "voice-tts; подсказка про длину куплета",
    },
    "### Готовые AI-треки из библиотеки (gen_*)": {
        "execute_music_code": "маршрут в composer, когда генерация недоступна",
    },
    "### DJ MODE": {
        "execute_music_code": "ТРЕБУЕТСЯ диджею: свежий трек пишется кодом",
        "search_samples": "ТРЕБУЕТСЯ диджею: сэмплы под стиль",
        "load_track": "запрет: «НЕ вызывай load_track в DJ-режиме»",
        "list_tracks": "запрет: там же",
    },
    "# 6. WAYPOINTS": {
        "memory_save": "контраст: «PLACE → save_waypoint, НЕ memory_save»",
    },
}


def test_foreign_tool_mentions_in_moved_sections_are_declared(enabled) -> None:
    """Переехавшая секция не должна тайком рекламировать чужой инструмент.

    Две записи выше помечены ТРЕБУЕТСЯ: инструкции DJ прямо велят писать
    трек через ``execute_music_code`` и брать сэмплы ``search_samples``,
    а каталог даёт скиллу ``dj`` только ``set_dj_mode``/``get_music_state``/
    ``stop_music``. Пока Move B выключен, это ничего не ломает; при
    включении сужения диджей получит инструкцию, которую нечем исполнить.
    Это условие входа в задачу 7.5, а не дефект фазы 5.
    """
    catalog = {
        entry.name
        for skill in skill_names()
        for entry in tools_for_skill(skill, include_core=True)
    }
    for section in enabled.sections:
        own = {
            entry.name
            for skill in section.skills
            for entry in tools_for_skill(skill, include_core=True)
        }
        lowered = section.text.lower()
        foreign = {name for name in catalog - own if _mentions_tool(lowered, name)}
        anchor = section.text.splitlines()[0].strip()
        declared = set(_KNOWN_FOREIGN_MENTIONS.get(anchor, {}))
        assert foreign == declared, (
            f"секция {anchor!r} ({', '.join(section.skills)}): "
            f"незадекларированные чужие инструменты "
            f"{sorted(foreign - declared)}, исчезнувшие "
            f"{sorted(declared - foreign)}. При Move B это класс #1403 — "
            f"обнови _KNOWN_FOREIGN_MENTIONS осознанно."
        )


def test_system_prompt_actually_gets_smaller(disabled, enabled) -> None:
    """Ради этого всё и затевалось — цифра, а не ощущение."""
    before = len(disabled.system_prompt)
    after = len(enabled.system_prompt)
    assert after < before * 0.85, (
        f"системный промпт {before} → {after} симв: раскол не дал выигрыша"
    )
