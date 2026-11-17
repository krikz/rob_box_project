"""#4242 (ADR-0160 PR-5): числа и знаки словами для голоса, где сервис цифр не читает.

Проверяется поведение: что слышит Silero, что получают Yandex и MiniMax (их сервис читает цифры сам - текст как
есть), что делает флаг ``reads_digits``. Модуль ``tts_normalizer`` - единственное место знания о чтении чисел.
"""
from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider  # noqa: E402
from rob_box_voice.tts_normalizer import cardinal, ordinal, spell_out  # noqa: E402
from rob_box_voice.tts_providers import SileroTTSProvider, YandexTTSProvider  # noqa: E402

PHRASE = "144 разделить на 12 будет 12, сейчас 12:30, на улице -5 °C, влажность 45%."
SPOKEN = (
    "сто сорок четыре разделить на двенадцать будет двенадцать, сейчас двенадцать тридцать, "
    "на улице минус пять градусов Цельсия, влажность сорок пять процентов."
)


def _adapters():
    return {
        "silero": SileroTTSProvider(logger=MagicMock()),
        "yandex": YandexTTSProvider("fake-key", object(), logger=MagicMock()),
        "minimax": MiniMaxTTSProvider(api_key="k", group_id="g", client=MagicMock()),
    }


# ── адаптеры: по флагу reads_digits ───────────────────────────────────────────


def test_adapters_that_do_not_read_digits_get_words_the_rest_get_text_as_is():
    adapters = _adapters()
    assert [n for n, a in adapters.items() if not a.capabilities().reads_digits] == ["silero"]
    for name, adapter in adapters.items():
        expected = PHRASE if adapter.capabilities().reads_digits else SPOKEN
        assert adapter.prepare(PHRASE) == expected, name


def test_silero_gets_no_digit_at_all():
    prepared = _adapters()["silero"].prepare("Ту-154, H2O, версия 1.2.3, счёт 3:2, № 7, 2024 г.")
    assert not any(ch.isdigit() for ch in prepared), prepared


def test_text_without_digits_or_signs_is_untouched():
    text = "Привет, Робби! Как дела?\nВторая строка — без чисел."
    assert spell_out(text) == text
    assert spell_out("") == ""


# ── числа ─────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    "number, gender, case, words",
    [
        (0, "m", "nom", "ноль"),
        (21, "f", "nom", "двадцать одна"),
        (22, "f", "nom", "двадцать две"),
        (21, "f", "gen", "двадцати одной"),
        (115, "m", "nom", "сто пятнадцать"),
        (2000, "m", "nom", "две тысячи"),
        (1001, "m", "nom", "одна тысяча один"),
        (5000, "m", "gen", "пяти тысяч"),
        (1_000_000, "m", "nom", "один миллион"),
        (2_500_000, "m", "nom", "два миллиона пятьсот тысяч"),
        (-3, "m", "nom", "минус три"),
    ],
)
def test_cardinal(number, gender, case, words):
    assert cardinal(number, gender, case) == words


@pytest.mark.parametrize(
    "number, case, gender, words",
    [
        (1, "nom", "n", "первое"),
        (3, "gen", "n", "третьего"),
        (21, "nom", "n", "двадцать первое"),
        (40, "nom", "m", "сороковой"),
        (2024, "gen", "m", "две тысячи двадцать четвёртого"),
        (2000, "gen", "m", "двухтысячного"),
        (1999, "prep", "m", "тысяча девятьсот девяносто девятом"),
    ],
)
def test_ordinal(number, case, gender, words):
    assert ordinal(number, case, gender) == words


@pytest.mark.parametrize(
    "text, spoken",
    [
        ("3,5 кг", "три целых пять десятых килограмма"),
        ("0.3 м/с", "ноль целых три десятых метра в секунду"),
        ("1 км, 3 км, 5 км, 21 км", "один километр, три километра, пять километров, двадцать один километр"),
        ("25 мс", "двадцать пять миллисекунд"),
        ("21 мин", "двадцать одна минута"),
        ("120 км/ч", "сто двадцать километров в час"),
        ("20%", "двадцать процентов"),
        ("21,5%", "двадцать одна целая пять десятых процента"),
        ("-5 °C", "минус пять градусов Цельсия"),
        ("+7", "плюс семь"),
        ("12:30", "двенадцать тридцать"),
        ("14:05", "четырнадцать ноль пять"),
        ("18:00", "восемнадцать ноль ноль"),
        ("5-7 раз", "от пяти до семи раз"),
        ("5–7 км", "от пяти до семи километров"),
        ("до 5 км", "до пяти километров"),
        ("№5", "номер пять"),
        ("2+2=4", "два плюс два равно четыре"),
        ("15.03.2024", "пятнадцатого марта две тысячи двадцать четвёртого года"),
        ("до 15.03.2024", "до пятнадцатого марта две тысячи двадцать четвёртого года"),
        ("21 января", "двадцать первое января"),
        ("с 1 мая", "с первого мая"),
        ("1-го мая", "первого мая"),
        ("5-й", "пятый"),
        ("3-х человек", "трёх человек"),
        ("в 2024 году", "в две тысячи двадцать четвёртом году"),
        ("к 2024 году", "к две тысячи двадцать четвёртому году"),
        ("с 2024 года", "с две тысячи двадцать четвёртого года"),
        ("3,14159", "три запятая один четыре один пять девять"),
    ],
)
def test_spell_out(text, spoken):
    assert spell_out(text) == spoken


def test_decimal_dot_is_not_a_date():
    assert spell_out("3.5 кг") == "три целых пять десятых килограмма"


def test_not_a_unit_is_not_swallowed():
    assert spell_out("12 арбузов") == "двенадцать арбузов"
    assert spell_out("3 с половиной") == "три с половиной"
