"""test_issue_2829_identity_ack_speaker_gate.py

Issue #2829 (ADR-0131): «да» на вопрос склейки #2828 засчитывается, только если ответил один из двух профилей, о
которых был вопрос (``new_id``/``known_id``). Неизвестный или посторонний голос чужую склейку не подтверждает.

#3765 (ADR-0150 PR-5c): решение — чистая функция ``speaker_adapter.merge_answer`` по результату биометрии ЭТОЙ фразы.
PR-15 (#3711, часть 15p): сквозные проверки ноды диалога v1 удалены вместе с ней; здесь — сама функция v2.
"""

from __future__ import annotations

import pytest

from rob_box_dialog.speaker_adapter import merge_answer

YES = "да, это я"
BORIS_ID = "f65d3050-0000-0000-0000-000000000000"
SASHA_ID = "5d99f474-0000-0000-0000-000000000000"
THIRD_ID = "aaaa0000-0000-0000-0000-000000000000"
PLAN = {"kind": "conflict", "new_id": BORIS_ID, "known_id": SASHA_ID}


@pytest.mark.parametrize("speaker, expected", [
    ({"is_known": False}, (None, False)),
    ({"is_known": True, "speaker_id": ""}, (None, False)),
    ({"is_known": True, "speaker_id": THIRD_ID}, (None, False)),
    ({"is_known": True, "speaker_id": BORIS_ID}, (True, True)),
    ({"is_known": True, "speaker_id": SASHA_ID}, (True, True)),
])
def test_merge_answer_speaker_gate(speaker, expected):
    assert merge_answer(PLAN, YES, speaker) == expected


def test_different_from_any_voice_is_kept_apart():
    """«Мы разные» склейки не требует — посторонний голос его сказать может, данные и так раздельны."""
    assert merge_answer(PLAN, "мы разные люди", {"is_known": False}) == (False, False)
