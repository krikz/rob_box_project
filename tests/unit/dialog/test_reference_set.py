"""Структура scripts/dialog/reference_set.yaml (ADR-0150 PR-0, #3684, решение О1). Разметку утверждает Шифу —
тест проверяет только форму и обязательный состав, а не правильность ожиданий."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]
DATA = yaml.safe_load((REPO / "scripts" / "dialog" / "reference_set.yaml").read_text(encoding="utf-8"))
PHRASES = DATA["phrases"]
EXPECTED_KEYS = {"tier1", "actions", "speech_only", "ask", "silence"}
SOURCES = {"issue", "log", "marathon"}

# лог 20.10: строки, из которых набор обязан иметь фразу (§0.2.2)
MANDATORY_TEXT = [
    "покажи анимацию сон",
    "ещё раз покажи анимацию ускорения",
    "включи голос надежного мужика",
    "спой песенку про стрижей и ежей",
    "запомни, что Саша любит зелёный чай",  # #2755
    "Зачитай рэпчик про голодного робота",  # #3221
    "добавь бит",  # #993
    "смени голос",  # mv03
    "хватит",  # A16
    "Нормально всё работает",  # #1292
    "Робота-диджея",  # #2971
]


def test_at_least_sixty_phrases():
    assert len(PHRASES) >= 60


def test_ids_unique():
    ids = [p["id"] for p in PHRASES]
    assert len(ids) == len(set(ids))


@pytest.mark.parametrize("phrase", PHRASES, ids=[p["id"] for p in PHRASES])
def test_phrase_shape(phrase):
    assert set(phrase) >= {"id", "text", "source", "expected", "must_not", "notes"}
    assert phrase["text"].strip()
    assert phrase["source"] in SOURCES
    assert isinstance(phrase["must_not"], list)
    assert len(phrase["expected"]) == 1 and set(phrase["expected"]) <= EXPECTED_KEYS
    if "actions" in phrase["expected"]:
        assert phrase["expected"]["actions"], "actions не должен быть пустым"


@pytest.mark.parametrize("needle", MANDATORY_TEXT)
def test_mandatory_phrase_present(needle):
    assert any(needle.lower() in p["text"].lower() for p in PHRASES), needle


def test_bare_robot_with_backlog_and_without():
    bare = [p for p in PHRASES if p["text"] == "Робот"]
    assert {next(iter(p["expected"])) for p in bare} >= {"actions", "tier1"}


def test_marked_as_draft_pending_approval():
    assert DATA["status"] == "draft-awaiting-approval"
