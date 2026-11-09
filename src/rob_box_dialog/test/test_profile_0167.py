"""Профиль продукта ``assistant.yaml`` и его загрузчик (ADR-0167 §2.4, шаг Ш1б).

Три случая: путь не задан - нейтральные умолчания ядра и строка лога; путь задан, а файл битый - ``ProfileError``;
файл цел - профиль. Профиль робота (``docker/sound/config/assistant.yaml``) даёт ровно ту таблицу вейков, что хосты
робота читали раньше из ``wake_words.yaml`` напрямую: поведение робота не меняется.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import profile as pf
from rob_box_dialog.words import has_wake_word

REPO = Path(__file__).resolve().parents[3]
ROBOT_PROFILE = REPO / "docker" / "sound" / "config" / "assistant.yaml"
DIALOG_NODE = REPO / "src" / "rob_box_voice" / "rob_box_voice" / "dialog_node.py"
DIALOG_NODE_YAMLS = (
    REPO / "docker" / "sound" / "config" / "voice_assistant" / "dialog_node.yaml",
    REPO / "src" / "rob_box_voice" / "config" / "dialog_node.yaml",
)

needs_robot = pytest.mark.skipif(not ROBOT_PROFILE.is_file(), reason="нет docker/ (окружение без профиля робота)")


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


@needs_robot
def test_robot_profile_gives_the_same_wake_table_hosts_read_before():
    got = pf.load_profile(ROBOT_PROFILE)
    personality, operator = kn.resolve_wake_word_namespaces(str(ROBOT_PROFILE.parent / "wake_words.yaml"))
    assert got.name == "РОББОКС"
    assert list(got.wake_personality) == personality == list(kn.DEFAULT_WAKE_WORDS)
    assert list(got.wake_operator) == operator


@needs_robot
def test_dialog_node_requires_the_robot_profile():
    """Хост робота грузит профиль строго (``load_profile``, не ``resolve_profile``): без профиля нода падает на
    старте, а не просыпается от каждой фразы мастерской на нейтральных умолчаниях."""
    tree = ast.parse(DIALOG_NODE.read_text(encoding="utf-8-sig"))
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)]
    declared = {
        n.args[0].value: n.args[1].value
        for n in calls
        if getattr(n.func, "attr", "") == "declare_parameter" and len(n.args) == 2
        if isinstance(n.args[0], ast.Constant) and isinstance(n.args[1], ast.Constant)
    }
    assert declared.get("profile_path") == "/config/assistant.yaml"
    assert "wake_words_path" not in declared
    names = {getattr(n.func, "id", getattr(n.func, "attr", "")) for n in calls}
    assert "load_profile" in names and "resolve_profile" not in names
    for yaml_file in DIALOG_NODE_YAMLS:
        assert 'profile_path: "/config/assistant.yaml"' in yaml_file.read_text(encoding="utf-8"), yaml_file
