"""Issue #2925 — границы парсера самопредставления
(:func:`rob_box_voice.core.self_intro.extract_self_intro_name`).

Что считается представлением, а что нет, — в docstring модуля. Строки
«привет я борис…» и «Робот, запомни про меня: Дарья…» — дословно из
issue #2925 (run 35931295672 / 35933157262).
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.self_intro import (
    extract_self_intro_name,
    same_person_name,
)

N708_TEXT = "привет я борис заглянул проверить проводку"
DARYA_TEXT = (
    "Робот, запомни про меня: Дарья болит за Спартак и всегда "
    "приносит пиццу."
)


@pytest.mark.parametrize(
    "text, expected",
    [
        (N708_TEXT, "Борис"),
        ("давай знакомиться, меня зовут Саша", "Саша"),
        ("робот меня зовут Денис говорю", "Денис"),
        ("Меня зовут Эйджик", "Эйджик"),
        ("запомни меня как Дэнчик", "Дэнчик"),
        ("это я, Саша", "Саша"),
        ("я не знаю, меня зовут Борис", "Борис"),
        (DARYA_TEXT, None),
        ("а как меня зовут?", None),
        ("как меня зовут робот", None),
        ("робот нет я не саша ты обознался", None),
        ("да это я", None),
        ("знакомься, это Борис", None),
        ("я заглянул проверить проводку", None),
        ("я Борису сказал", None),
        ("Борис", None),
    ],
)
def test_extract_self_intro_name(text, expected):
    assert extract_self_intro_name(text) == expected


def test_same_person_name():
    assert same_person_name("Алёна", " алена ")
    assert not same_person_name("Саша", "Борис")
    assert not same_person_name("", "")
    assert not same_person_name(None, "Саша")
