"""Промпт Личности ядра: основа без имени продукта + персона профиля (ADR-0167 §2.4, шаг Ш1б-3).

Основа и нейтральная персона - данные пакета (``importlib.resources``); персона продукта - из его профиля. Что промпт
робота с его персоной совпадает с прежним файлом байт в байт, сверяет
``tests/unit/dialog/test_robot_persona_prompt_0167.py``.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from rob_box_dialog import prompts as pr
from rob_box_dialog.profile import load_profile

FIXTURE = Path(__file__).resolve().parent / "fixtures" / "robot_profile"


def test_base_is_package_data_with_each_slot_once_and_no_product_name():
    text = pr.base()
    assert all(text.count(slot) == 1 for slot in pr.SLOTS)
    assert text.startswith(pr.SLOTS[0]) and text.endswith("# 3. CHARACTER\n" + pr.SLOTS[1])
    assert "РОББОКС" not in text and "ROBBOX" not in text.upper()


def test_neutral_prompt_has_no_name_and_no_slots():
    prompt = pr.personality_prompt(pr.neutral_persona())
    assert not any(slot in prompt for slot in pr.SLOTS)
    assert "РОББОКС" not in prompt and prompt.startswith(pr.neutral_persona().identity + " Every answer")


def test_persona_goes_to_its_places():
    prompt = pr.personality_prompt(pr.Persona("You are Тестик.", "- Call yourself «Тестик»."))
    assert prompt.startswith("You are Тестик. Every answer is ONE command")
    assert prompt.endswith("# 3. CHARACTER\n- Call yourself «Тестик».")


def test_robot_fixture_persona_names_the_robot():
    prompt = pr.personality_prompt(load_profile(FIXTURE).persona)
    assert prompt.startswith("You are РОББОКС — ") and prompt.endswith("«ты снова РОББОКС».")


@pytest.mark.parametrize("text", ["", "одна строка", "\n\nбез identity", "identity\n\n   "])
def test_persona_needs_two_blocks(text):
    with pytest.raises(ValueError, match="два блока"):
        pr.parse_persona(text)


def test_persona_crlf_is_the_same_persona():
    assert pr.parse_persona("Кто.\r\n\r\n- Какой.\r\n") == pr.Persona("Кто.", "- Какой.")
