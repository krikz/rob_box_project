"""play_animation(face=...): одна схема сцены лица в каталоге и в команде, тул проверяет её кодом (ADR-0150 §5.5.9, PR-3c)."""

from rob_box_core.tool_catalog import get_tool
from rob_box_dialog import face_scene, knowledge as kn
from ._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.tools.animation import PlayAnimationTool
_ros_stubs = _ros.fixture()


def _tool():
    return PlayAnimationTool(None)


def test_face_is_one_optional_argument_not_a_new_tool():
    entry = get_tool("play_animation")
    schema = entry.parameters
    assert set(schema["properties"]) == {"animation", "duration", "face"}
    assert schema["required"] == []                          # хватает animation или face
    assert entry.signature["params"] == ["animation", "duration", "face"]


def test_catalog_face_schema_is_the_command_face_schema():
    face = get_tool("play_animation").parameters["properties"]["face"]
    shared = face_scene.schema()
    assert face["type"] == "object" and face["additionalProperties"] is False
    assert face["description"] == shared["description"]
    assert {k: v["enum"] for k, v in face["properties"].items() if "enum" in v} == \
        {k: list(v) for k, v in kn.FACE_SCENE_SLOTS.items()}
    assert (face["properties"]["duration_s"]["minimum"], face["properties"]["duration_s"]["maximum"]) == \
        (shared["properties"]["duration_s"]["minimum"], shared["properties"]["duration_s"]["maximum"])
    assert face["required"] == []


def test_runtime_schema_matches_the_catalog():
    runtime = _tool().to_openai_tool_format()["function"]["parameters"]
    assert runtime == get_tool("play_animation").parameters


def test_execute_accepts_face_with_or_without_animation():
    out = _tool().execute(face={"eyes": "wide", "mouth": "teeth"})
    assert out.success and out.data == {"face": {"eyes": "wide", "mouth": "teeth", "duration_s": 5.0}}
    both = _tool().execute(animation="happy", duration=6, face='{"blink": "wink_right"}')
    assert both.success and both.data["animation"] == "happy" and both.data["face"]["blink"] == "wink_right"


def test_execute_tells_the_model_why_a_face_was_not_accepted():
    out = _tool().execute(face={"eyes": "neon"})
    assert not out.success and "eyes=neon" in out.error
    out = _tool().execute()
    assert not out.success
    dropped = _tool().execute(animation="happy", face={"color": "red"})
    assert dropped.success and "unknown_field=color" in dropped.message and "face" not in dropped.data
