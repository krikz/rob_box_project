"""test_issue_2829_identity_ack_speaker_gate.py

Issue #2829 (ADR-0131): «да» на вопрос склейки #2828 засчитывается, только если ответил один из двух профилей, о
которых был вопрос (``new_id``/``known_id``). Неизвестный или посторонний голос чужую склейку не подтверждает.

#3765 (ADR-0150 PR-5c): решение — чистая функция ``speaker_adapter.merge_answer`` по результату биометрии ЭТОЙ фразы
(``_prepare_user_input_context`` резолвит его до ``identity_question.merge_reply``); нода только публикует склейку
по вердикту. Здесь — поведение ноды целиком: ход с ответом от разных голосов.
"""

from __future__ import annotations

import pytest

from rob_box_dialog import phrases
from rob_box_dialog.speaker_adapter import merge_answer

from . import test_issue_2828_identity_question_collision as base

YES = "да, это я"
THIRD_ID = "aaaa0000-0000-0000-0000-000000000000"
PLAN = {"kind": "conflict", "new_id": base.BORIS_ID, "known_id": base.SASHA_ID}


@pytest.mark.parametrize("speaker", [
    {"is_known": False},
    {"is_known": True, "speaker_id": THIRD_ID, "name": "Глеб"},
], ids=["unknown", "third_party"])
def test_yes_from_other_voice_does_not_merge(speaker):
    n = base._asked()

    base._answer(n, YES, speaker=speaker)

    assert base._merged(n) == []
    assert n.tts[-1] == phrases.text("identity.merge_unclear")
    assert n.llm_calls == [base.INTRO]


@pytest.mark.parametrize("speaker_id", [base.BORIS_ID, base.SASHA_ID], ids=["new_id", "known_id"])
def test_yes_from_one_of_two_profiles_merges(speaker_id):
    n = base._asked()

    base._answer(n, YES, speaker={"is_known": True, "speaker_id": speaker_id, "name": "x"})

    assert base._merged(n) == [{"src_speaker_id": base.BORIS_ID, "dst_speaker_id": base.SASHA_ID}]


@pytest.mark.parametrize("speaker, expected", [
    ({"is_known": False}, (None, False)),
    ({"is_known": True, "speaker_id": ""}, (None, False)),
    ({"is_known": True, "speaker_id": THIRD_ID}, (None, False)),
    ({"is_known": True, "speaker_id": base.BORIS_ID}, (True, True)),
    ({"is_known": True, "speaker_id": base.SASHA_ID}, (True, True)),
])
def test_merge_answer_speaker_gate(speaker, expected):
    assert merge_answer(PLAN, YES, speaker) == expected


def test_different_from_any_voice_is_kept_apart():
    """«Мы разные» склейки не требует — посторонний голос его сказать может, данные и так раздельны."""
    assert merge_answer(PLAN, "мы разные люди", {"is_known": False}) == (False, False)
