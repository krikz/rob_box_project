"""Реплика диджея на переходе (ADR-0149 §12 В2, решение Шифу 06.10): факты кода, шаблоны, схема и валидатор."""

import pytest

from rob_box_music import dj_line as dl

NAMES = ("Super Mario Bros", "Тетрис", "Аладдин", "Марио")
TETRIS = dl.LineFacts(2, 4, "Марио, Тетрис, Аладдин", "Тетрис", 4, 3, NAMES)


@pytest.mark.parametrize("facts, phase", [
    (dl.LineFacts(1, 4, "т", None, 4), "opening"),
    (dl.LineFacts(4, 4, "т", None, 2, 5), "final"),
    (dl.LineFacts(3, 4, "т", None, 5, 4), "peak"),
    (dl.LineFacts(2, 4, "т", None, 4, 3), "rise"),
    (dl.LineFacts(3, 10, "т", None, 3, 4), "down"),
    (dl.LineFacts(3, 10, "т", None, 3, 3), "steady"),
])
def test_phase_comes_from_plan_energy_and_position(facts, phase):
    assert facts.phase == phase


def test_facts_for_takes_neighbour_energy_from_plan():
    facts = dl.facts_for(3, 4, "т", "Тетрис", [4, 5, 2, 2])
    assert (facts.energy, facts.prev_energy, facts.phase) == (2, 5, "down")
    assert dl.facts_for(4, 4, "т", None, [4, 5, 2, 2]).phase == "final"


def test_template_names_the_hook_number_and_total():
    line = dl.template_line(TETRIS, seed=0)
    assert "Тетрис" in line and "2 из 4" in line
    assert "Марио" not in line  # в шаблоне — только факт трека


def test_template_without_hook_says_no_melody_name():
    facts = dl.LineFacts(2, 4, "космос", None, 4, 3)
    for seed in range(6):
        line = dl.template_line(facts, seed)
        assert line and "{" not in line


def test_final_track_template_says_final():
    facts = dl.LineFacts(4, 4, "т", "Аладдин", 2, 5)
    assert all(("Финал" in dl.template_line(facts, s) or "Последний" in dl.template_line(facts, s))
               for s in range(6))


def test_template_does_not_repeat_previous_line():
    first = dl.template_line(TETRIS, seed=7)
    assert dl.template_line(TETRIS, seed=7, last=first) != first


def test_long_theme_is_not_substituted():
    facts = dl.LineFacts(1, 4, "мегасет для игроков из RTTTL-мелодий разных игр", None, 4)
    assert "theme" not in facts.values()
    assert all("мегасет" not in dl.template_line(facts, s) for s in range(4))


def test_llm_line_gets_facts_from_code():
    assert dl.validate_line("Трек {no} из {total}: качаем под {hook}!", TETRIS) == "Трек 2 из 4: качаем под Тетрис!"


@pytest.mark.parametrize("text, reason", [
    ("Сейчас играет Марио, держитесь!", "name"),  # другая мелодия буквами
    ("Качаем под Тетрис!", "name"),  # даже верное название — только подстановкой
    ("Аладдина ждали? Он здесь!", "name"),  # падеж не спасает
    ("Трек 3 из 4!", "digits"),
    ("Это {song}!", "field"),
    ("{hook} и снова {hook}!", "field"),
    ("а" * (dl.LINE_MAX + 1), "length"),
    ("", "empty"),
    (None, "empty"),
    ("Скобка { не закрыта", "braces"),
])
def test_validator_rejects_claims_outside_facts(text, reason):
    with pytest.raises(dl.LineInvalid) as err:
        dl.validate_line(text, TETRIS)
    assert err.value.reason == reason


def test_hook_placeholder_is_invalid_when_track_has_no_melody():
    with pytest.raises(dl.LineInvalid):
        dl.validate_line("Под {hook}!", dl.LineFacts(2, 4, "т", None, 4, 3))


def test_generic_title_words_stay_allowed():
    assert dl.validate_line("Супер, поехали дальше!", TETRIS) == "Супер, поехали дальше!"


def test_schema_is_one_short_string():
    schema = dl.schema()
    assert schema["required"] == ["line"] and not schema["additionalProperties"]
    assert schema["properties"]["line"]["maxLength"] == dl.LINE_MAX
    assert dl.tool()["function"]["name"] == dl.SUBMIT_TOOL
    with pytest.raises(dl.LineInvalid):
        dl.payload_line({"line": "x", "extra": 1})


def test_prompt_shows_facts_and_forbids_writing_them():
    system, user = dl.prompt(TETRIS, persona="Робби")
    assert "диджей Робби" in system and "{hook}" in system
    assert "«Тетрис»" in user and "подъём" in user and "{total} = 4" in user


def test_mentions_uses_validator_name_key():
    assert dl.mentions("Марио", "когда будет марио?") and dl.mentions("Аладдин", "где аладдина треки")
    assert not dl.mentions("Super Mario Bros", "супер, где тетрис?")
