"""Схема и валидатор поправки плана от LLM (ADR-0149 §4.5, §4.7; PR-10). Без сети."""

import json
from dataclasses import replace

import pytest

from rob_box_music import knowledge as kn
from rob_box_music import reasoner as rz
from rob_box_music.set_plan import seeded_plan, track_plan
from rob_box_music.theme import seeded_profile

PLAN = seeded_plan(seeded_profile("ночной город"), 42, set_id="s42")
VALID = {"theme_row": "cyber", "mode": "phrygian", "hooks": ["axelf_3", "robot"], "energy": [3, 4, 5, 4]}


def test_valid_answer_becomes_a_refinement():
    ref = rz.validate(json.loads(json.dumps(VALID)))
    assert ref == rz.Refinement("cyber", "phrygian", ("axelf_3", "robot"), (3, 4, 5, 4), None)


def test_none_row_means_theme_outside_the_table():
    assert rz.validate({**VALID, "theme_row": "none"}).row is None


@pytest.mark.parametrize("payload,path", [
    ("не объект", "$"),
    ([VALID], "$"),
    ({**VALID, "bpm": 140}, "bpm"),  # темпа в схеме нет: один темп на сет
    ({**VALID, "theme_row": "jazz"}, "theme_row"),
    ({k: v for k, v in VALID.items() if k != "mode"}, "mode"),
    ({**VALID, "mode": "lydian"}, "mode"),
    ({**VALID, "hooks": []}, "hooks"),
    ({**VALID, "hooks": ["axelf_3", "robot", "tetris", "popcorn"]}, "hooks"),
    ({**VALID, "hooks": ["never_gonna_give"]}, "hooks"),  # только кандидаты, которые показал код
    ({**VALID, "hooks": "axelf_3"}, "hooks"),
    ({**VALID, "energy": [0, 3]}, "energy"),
    ({**VALID, "energy": [3, 6]}, "energy"),
    ({**VALID, "energy": [3.5]}, "energy"),
    ({**VALID, "energy": [True]}, "energy"),
    ({**VALID, "energy": [3] * 11}, "energy"),
    ({**VALID, "hype_line": "Погнали!"}, "hype_line"),  # выкрик выключен (В2)
])
def test_invalid_answers_name_the_field(payload, path):
    with pytest.raises(rz.PlanInvalid) as exc:
        rz.validate(payload)
    assert exc.value.path == path


def test_hype_line_only_when_enabled_and_short():
    assert rz.validate({**VALID, "hype_line": " Погнали! "}, hype=True).hype_line == "Погнали!"
    with pytest.raises(rz.PlanInvalid):
        rz.validate({**VALID, "hype_line": "а" * (rz.HYPE_MAX + 1)}, hype=True)
    assert "hype_line" not in rz.schema()["properties"]
    assert rz.schema(hype=True)["properties"]["hype_line"]["maxLength"] == rz.HYPE_MAX


def test_schema_is_built_from_knowledge_and_has_no_tempo():
    s = rz.schema()
    assert s["additionalProperties"] is False and "bpm" not in s["properties"]
    assert s["properties"]["theme_row"]["enum"] == [*kn.THEMES, "none"]
    assert s["properties"]["mode"]["enum"] == list(kn.STYLES["club"].modes)
    hooks = set(s["properties"]["hooks"]["items"]["enum"])
    assert hooks == {h for row in kn.THEMES.values() for h in row.hooks} | set(kn.DEFAULT_HOOKS)
    assert rz.tool()["function"]["name"] == rz.SUBMIT_TOOL


def test_prompt_shows_theme_seeded_choice_and_candidates():
    system, user = rz.prompt("ночной город", PLAN.profile)
    assert rz.SUBMIT_TOOL in system and "не меняется" in system
    assert "«ночной город»" in user and "строка=none" in user and "axelf_3" in user


def test_apply_keeps_tempo_seed_tonic_and_swing():
    ref = rz.validate(VALID)
    new = rz.apply(PLAN, ref)
    assert (new.bpm, new.seed, new.swing, new.set_id) == (PLAN.bpm, PLAN.seed, PLAN.swing, PLAN.set_id)
    assert new.profile.bpm == PLAN.profile.bpm and new.profile.root == PLAN.profile.root
    assert (new.profile.row, new.profile.mode, new.profile.hook_ids) == ("cyber", "phrygian", ("axelf_3", "robot"))
    assert new.profile.source == PLAN.profile.source == "pool"  # строка от LLM — не слова темы (A11, #3399)
    assert [new.track(n).energy for n in range(1, 5)] == [3, 4, 5, 4]
    assert [new.root(n) for n in range(1, 13)] == [PLAN.root(n) for n in range(1, 13)]  # ход по квинтам тот же
    assert new.track(5) == PLAN.track(5)  # бочка плана остаётся
    assert replace(new.track(5), kick="", template="") == track_plan(5)  # дальше — волна seeded
    assert new.track(50) == track_plan(50)  # сет открытый


def test_hooks_found_by_theme_words_are_llm_candidates_and_nothing_else_is_added():
    """#3399: хуки seeded-профиля (найдены по словам темы во всей библиотеке) — в перечислении схемы и валидатора."""
    prof = seeded_profile("терминатор", found=("terminat", "theme_177"))
    assert rz.hook_candidates(prof)[rz.SEEDED] == ("terminat", "theme_177")
    assert {"terminat", "theme_177"} <= set(rz.schema(profile=prof)["properties"]["hooks"]["items"]["enum"])
    assert rz.validate({**VALID, "hooks": ["terminat"]}, profile=prof).hook_ids == ("terminat",)
    with pytest.raises(rz.PlanInvalid) as err:
        rz.validate({**VALID, "hooks": ["terminat"]})  # без профиля — не кандидат
    assert err.value.path == "hooks"


def test_found_theme_hooks_are_the_only_candidates_and_table_rows_do_not_displace_them():
    """ADR-0152 §5 (#3418): код нашёл хуки темы — схема и валидатор допускают только их; строка таблицы не подменяет."""
    prof = seeded_profile("терминатор", found=("terminat", "theme_178", "theme_177"))
    assert prof.theme_hooks[:3] == ("terminat", "theme_178", "theme_177")
    assert rz.hook_candidates(prof) == {rz.SEEDED: prof.theme_hooks}
    assert set(rz.schema(profile=prof)["properties"]["hooks"]["items"]["enum"]) == set(prof.theme_hooks)
    for hooks in (["terminat", "robotroc", "robot"], ["robot"], ["axelf_3"]):
        with pytest.raises(rz.PlanInvalid) as err:
            rz.validate({**VALID, "hooks": hooks}, profile=prof)
        assert err.value.path == "hooks"
    ref = rz.validate({**VALID, "hooks": ["theme_177", "terminat"]}, profile=prof)  # порядок и подмножество - за LLM
    assert ref.hook_ids == ("theme_177", "terminat")
    assert rz.apply(seeded_plan(prof, 42, set_id="s"), ref).profile.hook_ids == ("theme_177", "terminat")


def test_without_found_theme_hooks_candidates_are_table_rows_and_pool():
    prof = seeded_profile("ночной город")
    assert not prof.theme_hooks
    assert set(rz.hook_candidates(prof)) >= {*kn.THEMES, "pool"}
    assert rz.validate(VALID, profile=prof).hook_ids == ("axelf_3", "robot")
