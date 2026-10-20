"""Статическая проверка пакета ``rob_box_dialog_msgs`` (ADR-0150 PR-2).

rosidl на Windows/в unit-CI не запускается, поэтому ловим то, что ломает
``colcon build`` заранее: имена по правилам rosidl, список в CMakeLists равен
файлам на диске, типы полей известны, у каждого поля есть комментарий.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[2] / "rob_box_dialog_msgs"
MSG_DIR = PKG / "msg"
SRV_DIR = PKG / "srv"

#: Набор из карточки #3687 (ADR-0150 §10.1).
EXPECTED_MSGS = {
    "SpeechRequest", "SpeechCancel", "SpeechEvent", "SpeechEnvelope", "ExpressionRequest",
    "ExpressionEvent", "SoundEvent", "PlanEvent", "SessionState", "SceneFrame", "TurnEvent",
    "ActionRequest", "ActionResult", "ProviderState", "ProviderStateArray", "DialogOutput",
    "Intent", "Utterance", "SpeakerResult",
}
BUILTIN = {"bool", "string", "float32", "float64", "uint8", "uint16", "uint32", "uint64",
           "int8", "int16", "int32", "int64"}
FIELD = re.compile(r"^(?P<type>[\w/]+(?:\[\])?)\s+(?P<name>\w+)(?:=(?P<value>.+))?$")
SNAKE = re.compile(r"^[a-z][a-z0-9]*(_[a-z0-9]+)*$")
UPPER = re.compile(r"^[A-Z][A-Z0-9]*(_[A-Z0-9]+)*$")
CAMEL = re.compile(r"^[A-Z][A-Za-z0-9]*$")


def _lines(path: Path):
    """(номер строки, значимая строка, комментарий строкой выше) для каждого поля/константы."""
    previous_comment = ""
    for number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        stripped = raw.strip()
        if stripped.startswith("#"):
            previous_comment = stripped
            continue
        if not stripped or stripped == "---":
            previous_comment = ""
            continue
        yield number, stripped, previous_comment
        previous_comment = ""


def _files():
    return sorted(MSG_DIR.glob("*.msg")) + sorted(SRV_DIR.glob("*.srv"))


def test_the_package_has_exactly_the_cards_messages_and_service():
    assert {p.stem for p in MSG_DIR.glob("*.msg")} == EXPECTED_MSGS
    assert [p.name for p in SRV_DIR.glob("*.srv")] == ["DialogControl.srv"]


def test_cmake_lists_every_interface_file_and_nothing_else():
    cmake = (PKG / "CMakeLists.txt").read_text(encoding="utf-8")
    listed = set(re.findall(r'"((?:msg|srv)/[A-Za-z]+\.(?:msg|srv))"', cmake))
    on_disk = {f"{p.parent.name}/{p.name}" for p in _files()}
    assert listed == on_disk
    assert "builtin_interfaces" in cmake and "rosidl_generate_interfaces" in cmake


def test_package_xml_is_a_rosidl_interface_package():
    xml = (PKG / "package.xml").read_text(encoding="utf-8")
    for needle in ("<name>rob_box_dialog_msgs</name>", "rosidl_default_generators",
                   "rosidl_default_runtime", "rosidl_interface_packages",
                   "<build_type>ament_cmake</build_type>", "builtin_interfaces"):
        assert needle in xml, needle


@pytest.mark.parametrize("path", _files(), ids=lambda p: p.name)
def test_names_and_types_follow_rosidl_rules(path: Path):
    assert CAMEL.match(path.stem), path.name
    local = {p.stem for p in MSG_DIR.glob("*.msg")}
    seen = set()
    for number, line, _ in _lines(path):
        match = FIELD.match(line)
        assert match, f"{path.name}:{number}: не разобрана строка {line!r}"
        name, value = match["name"], match["value"]
        type_name = match["type"].replace("[]", "")
        assert (UPPER if value is not None else SNAKE).match(name), f"{path.name}:{number}: {name}"
        assert name not in seen, f"{path.name}:{number}: дубль {name}"
        seen.add(name)
        known = type_name in BUILTIN or type_name == "builtin_interfaces/Time" or type_name in local
        assert known, f"{path.name}:{number}: неизвестный тип {type_name}"


@pytest.mark.parametrize("path", _files(), ids=lambda p: p.name)
def test_every_field_has_a_comment_and_open_dicts_are_json_only(path: Path):
    """Комментарий у каждого поля (карточка); ``*_json`` - только строки открытых словарей."""
    for number, line, comment in _lines(path):
        match = FIELD.match(line)
        if match["value"] is not None:
            continue  # константы объясняет комментарий поля, к которому они относятся
        assert comment, f"{path.name}:{number}: у поля {match['name']} нет комментария строкой выше"
        if match["name"].endswith("_json"):
            assert match["type"] == "string", f"{path.name}:{number}: *_json должно быть string"


def test_json_fields_exist_only_where_the_adr_allows_open_dictionaries():
    """ADR-0150 §10.1: ``*_json`` - только открытые словари аргументов."""
    found = {}
    for path in _files():
        for _, line, _c in _lines(path):
            match = FIELD.match(line)
            if match["value"] is None and match["name"].endswith("_json"):
                found.setdefault(path.stem, []).append(match["name"])
    assert found == {
        "ExpressionRequest": ["face_json"],
        "SceneFrame": ["payload_json"],
        "ActionRequest": ["args_json"],
        "ActionResult": ["data_json"],
        "Intent": ["args_json"],
    }
