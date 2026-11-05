#!/usr/bin/env python3
"""test_persona.py — описание персонажа и кличка из него (ADR-0164 §4.1, этап 5). Без сети: ответ модели — строка."""

from __future__ import annotations

import json

import pytest

from rob_box_core import epithets as ep
from rob_box_core import persona as ps


def _proposal(appearance="", speech="", epithet="Бородач в синем свитере"):
    return json.dumps({"appearance": appearance, "speech": speech, "epithet": epithet}, ensure_ascii=False)


@pytest.fixture
def face_material():
    return ps.collect_material("face-1", "s1", has_snapshot=True, encounter_count=1)


# ── Материал ─────────────────────────────────────────────────────────────────


def test_collect_material_trims_and_caps_items():
    many = [f"реплика {i}" for i in range(30)] + ["", "   ", None]
    mat = ps.collect_material("v1", "s1", utterances=many, facts=["x" * 500], names=["Шифу", ""])
    assert len(mat.utterances) == ps.MAX_UTTERANCES
    assert mat.utterances[-1] == "реплика 29"
    assert len(mat.facts[0]) == ps.ITEM_MAX_LEN
    assert mat.names == ("Шифу",)


def test_sources_follow_material():
    assert ps.collect_material("a", "s").sources() == ()
    mat = ps.collect_material("a", "s", has_snapshot=True, facts=["любит шахматы"], encounter_count=2)
    assert mat.sources() == (ps.SECTION_APPEARANCE, ps.SECTION_SPEECH, ps.SECTION_BEHAVIOR)


def test_behavior_is_written_by_code():
    assert ps.describe_behavior(ps.collect_material("a", "s")) == ""
    assert ps.describe_behavior(ps.collect_material("a", "s", encounter_count=1)) == "Встречен впервые."
    assert ps.describe_behavior(ps.collect_material("a", "s", encounter_count=7)) == "Встреч: 7."


def test_prompt_forbids_appearance_without_snapshot():
    prompt = ps.build_proposal_prompt(ps.collect_material("a", "s", facts=["паяет моторы"]))
    assert "Снимка нет" in prompt
    assert "паяет моторы" in prompt
    assert "Бородач" in ps.build_proposal_prompt(ps.collect_material("a", "s", has_snapshot=True))


# ── Схема ────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize("raw", [
    "не json",
    json.dumps({"appearance": "в очках", "epithet": "Очкарик в свитере"}),  # нет ключа
    json.dumps({"appearance": "a", "speech": "", "epithet": "Б в", "extra": 1}),  # лишний ключ
    json.dumps({"appearance": 1, "speech": "", "epithet": "Бородач в синем"}),  # не строка
    json.dumps(["appearance"]),
])
def test_schema_violations_are_rejected(raw):
    assert ps.parse_proposal(raw) == (None, ps.REJECT_SCHEMA)


def test_parse_accepts_fenced_json_and_dict():
    fenced = "```json\n" + _proposal("в очках") + "\n```"
    fields, problem = ps.parse_proposal(fenced)
    assert problem is None and fields["appearance"] == "в очках"
    assert ps.parse_proposal({"appearance": "a  b", "speech": "", "epithet": "x"})[0]["appearance"] == "a b"


# ── Проверка описания и клички ───────────────────────────────────────────────


def test_valid_proposal_gives_description_and_epithet(face_material):
    verdict = ps.check_proposal(_proposal("Борода, синий свитер, очки"), face_material, now=100.0)
    assert verdict.reject_reason is None
    assert verdict.epithet == "Бородач в синем свитере"
    assert verdict.description.appearance == "Борода, синий свитер, очки"
    assert verdict.description.behavior == "Встречен впервые."
    assert verdict.description.session_id == "s1"
    assert verdict.description.sources == (ps.SECTION_APPEARANCE, ps.SECTION_BEHAVIOR)


def test_appearance_without_snapshot_is_made_up():
    mat = ps.collect_material("v", "s", facts=["любит шахматы"])
    verdict = ps.check_proposal(_proposal("Борода", "любит шахматы", "Шахматист с бородой"), mat)
    assert verdict.reject_reason == ps.REJECT_UNSUPPORTED
    assert verdict.description is None and verdict.epithet is None


def test_speech_without_material_is_made_up(face_material):
    verdict = ps.check_proposal(_proposal("Борода", "любит футбол"), face_material)
    assert verdict.reject_reason == ps.REJECT_UNSUPPORTED


def test_too_long_section_is_rejected_not_cut(face_material):
    verdict = ps.check_proposal(_proposal("а" * (ps.SECTION_MAX_LEN + 1)), face_material)
    assert verdict.reject_reason == ps.REJECT_SECTION_LENGTH


def test_empty_description_is_rejected(face_material):
    assert ps.check_proposal(_proposal(""), face_material).reject_reason == ps.REJECT_DESCRIPTION_EMPTY


@pytest.mark.parametrize("label, reason", [
    ("Незнакомец в куртке", ep.REJECT_STRANGER),
    ("Бородач", ep.REJECT_WORD_COUNT),
    ("-", ep.REJECT_WORD_COUNT),
    ("Борис в свитере", ep.REJECT_NAME),
    ("Тихий наблюдатель", ep.REJECT_TAKEN),
])
def test_bad_epithet_keeps_description(face_material, label, reason):
    verdict = ps.check_proposal(
        _proposal("Синий свитер", epithet=label), face_material, taken=["тихий  Наблюдатель"], names=["Борис"],
    )
    assert verdict.epithet is None
    assert verdict.reject_reason == reason
    assert verdict.description is not None and verdict.description.appearance == "Синий свитер"


def test_material_names_also_block_epithet():
    mat = ps.collect_material("f", "s", has_snapshot=True, names=["Михаил"])
    assert ps.check_proposal(_proposal("очки", epithet="Михаил в очках"), mat).reject_reason == ep.REJECT_NAME


# ── Уникальность по обоим хранилищам ─────────────────────────────────────────


def test_taken_union_merges_face_and_voice_stores():
    faces = ["Бородач в синем", None, ""]
    voices = ["бородач  в синем", "Странник-4", "Электрик"]
    assert ps.taken_union(faces, voices) == ["Бородач в синем", "Странник-4", "Электрик"]


def test_taken_union_excludes_own_epithet(face_material):
    taken = ps.taken_union(["Бородач в синем свитере", "Электрик"], exclude="бородач в синем свитере")
    assert taken == ["Электрик"]
    verdict = ps.check_proposal(_proposal("борода"), face_material, taken=taken)
    assert verdict.epithet == "Бородач в синем свитере"


def test_voice_epithet_blocks_face_candidate(face_material):
    taken = ps.taken_union([], ["Бородач в синем свитере"])
    assert ps.check_proposal(_proposal("борода"), face_material, taken=taken).reject_reason == ep.REJECT_TAKEN


# ── Решение ──────────────────────────────────────────────────────────────────


def test_decide_accepts_checked_epithet(face_material):
    verdict = ps.check_proposal(_proposal("борода"), face_material)
    decision = ps.decide(face_material, verdict, current="Странник")
    assert decision.epithet == "Бородач в синем свитере"
    assert decision.reason == ps.REASON_PERSONA and decision.changed
    assert decision.previous == "Странник"
    assert "«Странник» → «Бородач в синем свитере»" in ps.decision_log_line("лица ab12", decision)


def test_decide_same_epithet_is_not_a_change(face_material):
    verdict = ps.check_proposal(_proposal("борода"), face_material)
    decision = ps.decide(face_material, verdict, current="бородач в синем свитере")
    assert not decision.changed


def test_decide_keeps_previous_on_rejection(face_material):
    verdict = ps.check_proposal(_proposal("борода", epithet="Незнакомец"), face_material)
    decision = ps.decide(face_material, verdict, current="Странник")
    assert decision.epithet == "Странник" and not decision.changed
    assert "stranger" in decision.note


def test_decide_without_cloud_gives_dictionary_epithet_with_honest_note(face_material):
    decision = ps.decide(face_material, None, taken=["Странник"], unavailable="vision-LLM не настроен")
    assert decision.reason == ep.REASON_FIRST_SEEN
    assert len(decision.epithet.split()) == 1  # словарная — одно слово
    assert decision.epithet != "Странник"
    assert "vision-LLM не настроен" in decision.note
    assert decision.description is None


def test_decide_replaces_stranger_placeholder(face_material):
    decision = ps.decide(face_material, None, current="Незнакомец", unavailable="потолок")
    assert decision.changed and not ep.is_stranger_epithet(decision.epithet)
    assert decision.previous == "Незнакомец"


def test_fallback_uses_speech_topics():
    mat = ps.collect_material("v-chess", "s", utterances=["люблю шахматы, играю в шахматы каждый день с другом дома"])
    label = ps.fallback_epithet(mat)
    assert label in ep.EPITHET_LEXICON["шахматы"]


def test_fallback_is_deterministic_and_unique():
    mat = ps.collect_material("same-id", "s")
    first = ps.fallback_epithet(mat)
    assert ps.fallback_epithet(mat) == first
    assert ps.fallback_epithet(mat, taken=[first]) != first


# ── Хранение описания ────────────────────────────────────────────────────────


def test_description_roundtrip_and_old_records(face_material):
    desc = ps.check_proposal(_proposal("очки"), face_material, now=5.0).description
    assert ps.PersonaDescription.from_meta(desc.to_meta()) == desc
    assert ps.PersonaDescription.from_meta(None) is None
    assert ps.PersonaDescription.from_meta({"ts": "битое"}) is None
    assert ps.PersonaDescription.from_meta({}).is_empty()


def test_history_entry_matches_voice_format():
    assert ps.history_entry("Странник", "Бородач в синем", ps.REASON_PERSONA, 7) == {
        "ts": 7.0, "old": "Странник", "new": "Бородач в синем", "reason": "persona",
    }


# ── Потолок вызовов ──────────────────────────────────────────────────────────


def test_call_budget_caps_calls_per_window():
    budget = ps.CallBudget(per_window=2, window_sec=3600.0)
    assert budget.try_acquire(0.0) and budget.try_acquire(10.0)
    assert not budget.try_acquire(20.0)
    assert budget.remaining(20.0) == 0
    assert budget.try_acquire(3600.0)  # первый вызов вышел из окна
    assert not budget.try_acquire(3605.0)


def test_call_budget_default_lives_in_one_place():
    assert ps.CallBudget().per_window == ps.VISION_CALLS_PER_HOUR
    assert ps.CallBudget().window_sec == ps.HOUR_SEC


def test_zero_budget_never_calls():
    assert not ps.CallBudget(per_window=0).try_acquire(0.0)


def test_schema_is_narrow():
    assert ps.PROPOSAL_SCHEMA["additionalProperties"] is False
    assert set(ps.PROPOSAL_SCHEMA["required"]) == {"appearance", "speech", "epithet"}
    assert "behavior" not in ps.PROPOSAL_SCHEMA["properties"]
