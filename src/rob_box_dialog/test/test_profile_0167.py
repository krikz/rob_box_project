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
from rob_box_dialog.words import has_wake_word

FIXTURE = Path(__file__).resolve().parent / "fixtures" / "robot_profile"


def _write(tmp_path, text, wake="personality:\n  - эй ассистент\noperator:\n  - пульт\n"):
    (tmp_path / "wake_words.yaml").write_text(wake, encoding="utf-8")
    (tmp_path / "assistant.yaml").write_text(text, encoding="utf-8")
    return tmp_path / "assistant.yaml"


GOOD = "format: 1\nname: Ассистент\nwake_words: wake_words.yaml\n"


def test_load_good_profile_by_file_and_by_dir(tmp_path):
    file = _write(tmp_path, GOOD)
    for path in (file, tmp_path):
        got = pf.load_profile(path)
        assert got == pf.ProductProfile("Ассистент", ("эй ассистент",), ("пульт",), root=tmp_path)
        assert not got.neutral


def test_no_path_gives_neutral_defaults_and_says_so():
    lines = []
    got = pf.resolve_profile("", lines.append)
    assert got is pf.NEUTRAL and got.neutral
    assert got.name == "" and got.wake_personality == () and got.wake_operator == ()
    assert lines == [pf.NO_PROFILE_LOG]
    assert has_wake_word("который час", got.wake_personality), "без вейков ядро в режиме «адресовано всё»"


def test_neutral_defaults_name_no_product():
    words = " ".join(pf.NEUTRAL.wake_personality + pf.NEUTRAL.wake_operator) + pf.NEUTRAL.name
    assert "робот" not in words.lower() and "роббокс" not in words.lower()


@pytest.mark.parametrize(
    "text, why",
    [
        ("format: 1\nname: Ассистент\n", "нет ключей ['wake_words']"),
        ("format: 2\nname: Ассистент\nwake_words: wake_words.yaml\n", "format 2"),
        ("format: 1\nname: ''\nwake_words: wake_words.yaml\n", "name"),
        ("format: 1\nname: А\nwake_words: wake_words.yaml\nwake: x\n", "лишние ключи ['wake']"),
        ("format: 1\nname: А\nwake_words: nope.yaml\n", "таблицы вейков"),
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


def test_test_robot_profile_loads():
    """Тестовый профиль робота (``fixtures/robot_profile``) - корректный профиль формата 1."""
    got = pf.load_profile(FIXTURE)
    personality, operator = kn.resolve_wake_word_namespaces(str(FIXTURE / "wake_words.yaml"))
    assert got.name == "РОББОКС" and not got.neutral
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
