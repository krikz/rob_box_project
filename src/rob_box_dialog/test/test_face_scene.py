"""FaceScene: проверка кодом, одна схема на тул и команду (ADR-0150 §5.5.9, PR-3c #3691)."""

import json
import pathlib
import random

import pytest

from rob_box_dialog import command, face_scene as fs
from rob_box_dialog import knowledge as kn

CASES = json.loads((pathlib.Path(__file__).parent / "face_scene_cases.json").read_text(encoding="utf-8"))


def test_twenty_cases_validate_and_keep_their_slots():
    assert len(CASES) == 20 and len({c["phrase"] for c in CASES}) == 20
    for case in CASES:
        scene, why = fs.validate(case["face"])
        assert why == "" and scene is not None, case
        assert scene.slots() == {k: v for k, v in case["face"].items() if k != "duration_s"}, case
        assert scene.duration_s == case["face"].get("duration_s"), "не задано - None, умолчание ставит show_seconds"


def test_cases_reach_every_slot():
    used = {slot for case in CASES for slot in case["face"]}
    assert used >= set(kn.FACE_SCENE_SLOTS)


@pytest.mark.parametrize("bad, reason", [
    ({"eyes": "neon"}, "eyes=neon"),
    ({"eyes": "wide", "color": "#ff0000"}, "unknown_field=color"),
    ({"mouth": "grin", "duration_s": 31}, "duration_s=31"),
    ({"mouth": "grin", "duration_s": 0}, "duration_s=0"),
    ({"mouth": "grin", "duration_s": True}, "duration_s=True"),
    ({"mouth": "grin", "duration_s": float("nan")}, "duration_s=nan"),
    ({}, "empty"),
    ("not json", "not_json"),
    ("[1]", "not_an_object"),
    (None, "not_an_object"),
])
def test_invalid_scene_is_dropped_with_a_reason(bad, reason):
    assert fs.validate(bad) == (None, reason)


def test_json_string_from_the_model_is_accepted():
    scene, why = fs.validate('{"eyes": "wide", "tint": "red"}')
    assert why == "" and scene.slots() == {"eyes": "wide", "tint": "red"}


def test_random_dicts_never_raise():
    rng = random.Random(3691)
    junk = [None, True, 0, -1, 3.5, 1e9, float("inf"), "", "wide", [], {}, ["wide"], {"a": 1}, b"x"]
    keys = [*fs.FIELDS, "tint2", "", 7]
    for _ in range(1000):
        data = {rng.choice(keys): rng.choice([*junk, *kn.FACE_SCENE_SLOTS["eyes"], *kn.FACE_SCENE_SLOTS["rear"]])
                for _ in range(rng.randint(0, 5))}
        scene, why = fs.validate(data)
        assert (scene is None) == bool(why)
        if scene is not None:
            assert fs.problem(scene) == "" and scene.slots()


def test_to_dict_round_trips_through_validate():
    for case in CASES:
        scene, _ = fs.validate(case["face"])
        assert fs.validate(json.loads(json.dumps(scene.to_dict()))) == (scene, "")


def test_scene_without_time_has_none_and_one_default_comes_from_the_show_rule():
    """#4089: у сцены нет своего умолчания - «не задано» и «задано 5» различаются; время ставит ``show_seconds``."""
    from rob_box_dialog.expression import show_seconds

    scene, _ = fs.validate({"eyes": "wide"})
    assert scene.duration_s is None and "duration_s" not in scene.to_dict()
    assert fs.validate(scene.to_dict()) == (scene, "")
    assert show_seconds(scene.duration_s) == kn.FACE_SHOW_DEFAULT_S
    five, _ = fs.validate({"eyes": "wide", "duration_s": 5})
    assert five.duration_s == 5.0 != scene.duration_s


def test_one_schema_for_the_tool_and_the_command():
    schema = fs.schema()
    expression = kn.command_schema("json")["properties"]["expression"]["properties"]["face"]
    assert schema == expression and schema["additionalProperties"] is False
    assert {k: tuple(v["enum"]) for k, v in schema["properties"].items() if "enum" in v} == dict(kn.FACE_SCENE_SLOTS)
    assert schema["properties"]["duration_s"]["maximum"] == kn.FACE_SCENE_MAX_S
    assert "не задано" in schema["description"]


def test_command_uses_the_same_scene_type_and_check():
    assert command.FaceScene is fs.FaceScene
    cmd = command.parse({"speech": "ок", "expression": {"face": {"eyes": "wide", "rear": "heart"}}})
    assert cmd.expression.face == fs.FaceScene(eyes="wide", rear="heart")


def test_failed_and_silenced_scenes_come_from_the_reflex_table():
    assert set(kn.FACE_SCENE_CANCEL_SCENES) == {"error", "taped_mouth"}
