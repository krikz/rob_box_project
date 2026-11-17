"""Промпты терминалов ядра - чат Telegram и агент-оператор: основа без имени продукта + профиль (ADR-0167 Ш1б-4).

Основы - данные пакета (``importlib.resources``): ``telegram.txt`` с местом имени продукта, ``operator.txt`` с местами
персоны оператора. Что промпты робота совпадают с прежними файлами байт в байт, сверяет
``tests/unit/dialog/test_robot_terminal_prompts_0167.py``.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from rob_box_dialog import prompts as pr
from rob_box_dialog.profile import NEUTRAL, load_profile

FIXTURE = Path(__file__).resolve().parent / "fixtures" / "robot_profile"


def test_telegram_base_is_package_data_without_the_product_name():
    text = pr.telegram_base()
    assert text.count(pr.NAME_SLOT) == 2
    assert "РОББОКС" not in text and "ROBBOX" not in text.upper()


def test_telegram_prompt_puts_the_name_in_every_place():
    prompt = pr.telegram_prompt("Тестик")
    assert pr.NAME_SLOT not in prompt
    assert prompt.startswith("You are Тестик's assistant in a Telegram chat")
    assert "call yourself «Тестик»" in prompt


@pytest.mark.parametrize("name", ["", "   "])
def test_telegram_prompt_without_a_name_is_an_error(name):
    """Нейтральный профиль имени не несёт: «You are 's assistant» хуже честной ошибки."""
    with pytest.raises(ValueError, match="имя продукта"):
        pr.telegram_prompt(name)
    with pytest.raises(ValueError):
        pr.telegram_prompt(NEUTRAL.name)


def test_operator_base_is_package_data_with_each_slot_once_and_no_names():
    text = pr.operator_base()
    assert all(text.count(slot) == 1 for slot in pr.SLOTS)
    assert text.startswith(pr.SLOTS[0] + "\n\n") and text.endswith("\n\n" + pr.SLOTS[1])
    assert "ТАРС" not in text and "РОББОКС" not in text


def test_operator_persona_goes_to_its_places():
    rules = pr.operator_rules(pr.Persona("Ты — Пульт.", "Будь краток."))
    assert rules.startswith("Ты — Пульт.\n\n") and rules.endswith("\n\nБудь краток.")
    assert not any(slot in rules for slot in pr.SLOTS)


def test_neutral_operator_has_no_names():
    rules = pr.operator_rules(pr.neutral_operator_persona())
    assert "ТАРС" not in rules and "РОББОКС" not in rules
    assert NEUTRAL.operator_persona == pr.neutral_operator_persona()


def test_robot_fixture_names_tars_and_the_robot():
    profile = load_profile(FIXTURE)
    assert pr.operator_rules(profile.operator_persona).startswith("Ты — ТАРС, агент-оператор робота.")
    assert pr.telegram_prompt(profile.name).startswith("You are РОББОКС's assistant")
