"""test_praise_gate.py — pure praise/save-request detector (ADR-0132 PR-7).

Gates ``save_arrangement_preset``: the model may only persist an
arrangement as a mucis-library default for a melody after the user's own
words say so — never on its own judgement.
"""

from __future__ import annotations

import pytest

from rob_box_core.praise_gate import contains_praise


@pytest.mark.parametrize(
    "text",
    [
        "кайф!",
        "вот это огонь, сыграй ещё так",
        "круто получилось",
        "классно",
        "супер, играй так всегда",
        "отлично звучит",
        "шикарно вышло",
        "бомба, врубай",
        "мне нравится этот вариант",
        "понравилось очень",
        "сохрани",
        "сохраните пожалуйста этот вариант",
        "запомни этот вариант",
        "Запомните, пожалуйста",
        "awesome!",
        "nice one, keep it",
    ],
)
def test_detects_praise_or_save_request(text: str) -> None:
    assert contains_praise(text) is True


@pytest.mark.parametrize(
    "text",
    [
        "",
        "   ",
        "сыграй имперский марш",
        "громче",
        "выключи музыку",
        "мне не нравится, сделай по-другому",
        "не сохраняй это",
        "давай другой вариант",
        "а теперь потише",
    ],
)
def test_no_false_positive_on_neutral_or_negative_text(text: str) -> None:
    assert contains_praise(text) is False


def test_case_insensitive() -> None:
    assert contains_praise("КАЙФ, играй так и дальше") is True
    assert contains_praise("СОХРАНИ") is True


def test_negative_feedback_with_similar_prefix_is_not_a_false_positive() -> None:
    """"не сохраняй" содержит "сохраня", не форму "сохрани"/"сохраните" —
    гейт не обязан парсить отрицание, но и не должен путать разные слова."""
    assert contains_praise("не сохраняй это, удали") is False


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
