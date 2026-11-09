"""Профиль продукта ``assistant.yaml`` и его загрузчик (ADR-0167 §2.4, шаг Ш1б).

Три случая: путь не задан - нейтральные умолчания ядра и строка лога; путь задан, а файл битый - ``ProfileError``;
файл цел - профиль. Профиль робота (``docker/sound/config/assistant.yaml``) даёт ровно ту таблицу вейков, что хосты
робота читали раньше из ``wake_words.yaml`` напрямую: поведение робота не меняется.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import profile as pf
from rob_box_dialog.prompts import (
    Persona,
    neutral_operator_persona,
    neutral_persona,
    operator_rules,
    personality_prompt,
)
from rob_box_dialog.words import has_wake_word

FIXTURE = Path(__file__).resolve().parent / "fixtures" / "robot_profile"


PERSONA = "You are Ассистент.\n\n- Friendly and brief.\n"
OPERATOR = "Ты — Пульт, оператор Ассистента.\n\nБудь краток.\n"


def _write(
    tmp_path, text, wake="personality:\n  - эй ассистент\noperator:\n  - пульт\n", persona=PERSONA, operator=OPERATOR
):
    (tmp_path / "wake_words.yaml").write_text(wake, encoding="utf-8")
    (tmp_path / "persona.txt").write_text(persona, encoding="utf-8")
    (tmp_path / "operator_persona.txt").write_text(operator, encoding="utf-8")
    (tmp_path / "assistant.yaml").write_text(text, encoding="utf-8")
    return tmp_path / "assistant.yaml"


KEYS = "wake_words: wake_words.yaml\npersona: persona.txt\noperator_persona: operator_persona.txt\n"
GOOD = "format: 3\nname: Ассистент\n" + KEYS


def test_load_good_profile_by_file_and_by_dir(tmp_path):
    file = _write(tmp_path, GOOD)
    for path in (file, tmp_path):
        got = pf.load_profile(path)
        persona = Persona("You are Ассистент.", "- Friendly and brief.")
        operator = Persona("Ты — Пульт, оператор Ассистента.", "Будь краток.")
        assert got == pf.ProductProfile("Ассистент", ("эй ассистент",), ("пульт",), persona, operator, root=tmp_path)
        assert not got.neutral


def test_no_path_gives_neutral_defaults_and_says_so():
    lines = []
    got = pf.resolve_profile("", lines.append)
    assert got is pf.NEUTRAL and got.neutral
    assert got.name == "" and got.wake_personality == () and got.wake_operator == ()
    assert got.persona == neutral_persona()
    assert got.operator_persona == neutral_operator_persona()
    assert lines == [pf.NO_PROFILE_LOG]
    assert has_wake_word("который час", got.wake_personality), "без вейков ядро в режиме «адресовано всё»"


def test_neutral_defaults_name_no_product():
    words = " ".join(pf.NEUTRAL.wake_personality + pf.NEUTRAL.wake_operator) + pf.NEUTRAL.name
    assert "робот" not in words.lower() and "роббокс" not in words.lower()
    persona = pf.NEUTRAL.persona.identity + pf.NEUTRAL.persona.character
    assert "роббокс" not in persona.lower() and "rover" not in persona.lower()
    assert "роббокс" not in personality_prompt(pf.NEUTRAL.persona).lower()
    operator = operator_rules(pf.NEUTRAL.operator_persona).lower()
    assert "тарс" not in operator and "роббокс" not in operator  # «робот», «Quest/Telegram» в основе остаются (Ш1б-4)


@pytest.mark.parametrize(
    "text, why",
    [
        ("format: 3\nname: Ассистент\npersona: persona.txt\noperator_persona: o.txt\n", "нет ключей ['wake_words']"),
        (
            "format: 3\nname: Ассистент\nwake_words: wake_words.yaml\noperator_persona: o.txt\n",
            "нет ключей ['persona']",
        ),
        ("format: 3\nname: Ассистент\nwake_words: wake_words.yaml\npersona: persona.txt\n", "['operator_persona']"),
        ("format: 1\nname: Ассистент\nwake_words: wake_words.yaml\n", "format 1 устарел"),
        ("format: 2\nname: А\nwake_words: wake_words.yaml\npersona: persona.txt\n", "format 2 устарел"),
        ("format: 4\nname: А\n" + KEYS, "format 4"),
        ("format: 3\nname: ''\n" + KEYS, "name"),
        ("format: 3\nname: А\n" + KEYS + "wake: x\n", "лишние ключи ['wake']"),
        ("format: 3\nname: А\n" + KEYS.replace("wake_words.yaml", "nope.yaml"), "таблицы вейков"),
        ("format: 3\nname: А\n" + KEYS.replace("persona: persona.txt", "persona: ''"), "persona - имя файла"),
        ("format: 3\nname: А\n" + KEYS.replace("persona: persona.txt", "persona: nope.txt"), "файла персоны"),
        ("format: 3\nname: А\n" + KEYS.replace("operator_persona.txt", "nope.txt"), "(operator_persona) нет"),
        ("- 1\n- 2\n", "ждали словарь"),
        ("format: [1\n", "не читается как YAML"),
    ],
)
def test_broken_profile_is_an_error_not_a_fallback(tmp_path, text, why):
    with pytest.raises(pf.ProfileError) as err:
        pf.resolve_profile(_write(tmp_path, text))
    assert why in str(err.value)


def test_missing_file_and_empty_wake_table_are_errors(tmp_path):
    with pytest.raises(pf.ProfileError, match="файла нет"):
        pf.resolve_profile(tmp_path / "nope.yaml")
    with pytest.raises(pf.ProfileError, match="нет ни одного вейка personality"):
        pf.load_profile(_write(tmp_path, GOOD, wake="operator:\n  - пульт\n"))


def test_broken_persona_is_an_error(tmp_path):
    for persona in ("", "You are Ассистент.\n", "\n\n- Friendly.\n"):
        with pytest.raises(pf.ProfileError, match="два блока"):
            pf.load_profile(_write(tmp_path, GOOD, persona=persona))
        with pytest.raises(pf.ProfileError, match="два блока"):
            pf.load_profile(_write(tmp_path, GOOD, operator=persona))


def test_test_robot_profile_loads():
    """Тестовый профиль робота (``fixtures/robot_profile``) - корректный профиль формата 3."""
    got = pf.load_profile(FIXTURE)
    personality, operator = kn.resolve_wake_word_namespaces(str(FIXTURE / "wake_words.yaml"))
    assert got.name == "РОББОКС" and not got.neutral
    assert got.persona.identity.startswith("You are РОББОКС")
    assert got.operator_persona.identity.startswith("Ты — ТАРС")
    assert list(got.wake_personality) == personality and list(got.wake_operator) == operator


def test_core_knows_no_product_wake_words(neutral_core):
    """Вейки продукта - в его профиле: в таблице знания ядра нет ни вейков робота, ни их умолчания."""
    assert kn.WAKE_WORDS_WITHOUT_PROFILE == ()
    for gone in ("DEFAULT_WAKE_WORDS", "RISKY_WAKE_WORDS", "DEFAULT_OPERATOR_WAKE_WORDS"):
        assert not hasattr(kn, gone), gone
    assert kn.resolve_wake_word_namespaces(None) == ([], [])


def test_rules_without_a_table_are_neutral(neutral_core):
    """Хост не передал таблицу - ядро нейтрально: вейков нет, адресовано всё, снимать с реплики нечего."""
    from rob_box_dialog import grammar, words
    from rob_box_dialog.session import Session

    snap = Session("personality", clock=lambda: 0.0).snapshot
    assert words.has_wake_word("который час") and words.strip_wake_word("Робот, стоп") == "Робот, стоп"
    assert words.slipped_wake_rest("роботу дали") is None
    assert grammar.match("Робот, стоп", snap) is not None  # «стоп» и без снятого вейка - стоп
