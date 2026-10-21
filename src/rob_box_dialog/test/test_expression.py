"""ADR-0150 PR-3b (#3690): рефлексы стадий хода и наложение эмоции команды."""

import pytest

from rob_box_dialog import expression as ex
from rob_box_dialog import knowledge as kn


def test_every_turn_stage_has_a_reflex_and_the_rest_do_not():
    for stage in kn.TURN_STAGES:
        assert ex.reflex(stage) is not None, stage
    for stage in ("idle", "silenced", "music.playing", "not_addressed", "battery.low", "charging", "нет такой"):
        assert ex.reflex(stage) is None, stage


def test_addressed_is_listening_with_the_heard_earcon():
    plan = ex.reflex("addressed")
    assert (plan.emotion, plan.earcon, plan.ttl_ms) == ("listening", "heard", kn.REFLEXES["addressed"].ttl_ms)


def test_plan_comes_from_the_table_not_from_code():
    for stage in kn.TURN_STAGES:
        row = kn.REFLEXES[stage]
        plan = ex.reflex(stage)
        assert plan.earcon == (row.earcon or "") and plan.ttl_ms == (row.ttl_ms or 0)
        assert plan.emotion in row.face


def test_done_alternates_faces_by_turn_number():
    faces = {ex.reflex("done", snapshot={"turn_no": n}).emotion for n in range(4)}
    assert faces == set(kn.REFLEXES["done"].face)


def test_executing_face_by_class():
    assert ex.reflex("executing", "query").emotion == "thinking_teeth"
    assert ex.reflex("executing", "action.local").emotion == "thinking_teeth"
    assert ex.reflex("executing", "action.motion").emotion == "accelerating"
    assert ex.reflex("executing", "action.motion", {"direction": "left"}).emotion == "turn_left"
    assert ex.reflex("executing", "action.motion", {"direction": "right"}).emotion == "turn_right"
    assert ex.reflex("executing", "action.media").emotion == "thinking_teeth"


def test_executing_media_shows_music_scene_by_tempo_only_after_started():
    started = {"started": True}
    assert ex.reflex("executing", "action.media", {**started, "bpm": 90}).emotion == "music_vibe"
    assert ex.reflex("executing", "action.media", {**started, "bpm": 120}).emotion == "music_groove"
    assert ex.reflex("executing", "action.media", {**started, "bpm": 140}).emotion == "music_shades"
    assert ex.reflex("executing", "action.media", started).emotion == kn.EXECUTING_FACE["action.media"][0]


def test_executing_scenes_exist_in_the_tables():
    assert set(ex.MOTION_FACE.values()) | {ex.MOTION_DEFAULT_FACE} <= set(kn.EXECUTING_FACE["action.motion"])


@pytest.mark.parametrize("emotion", [None, "", "neutral", "idle", "talking", "NEUTRAL"])
def test_neutral_command_emotion_changes_nothing(emotion):
    plan = ex.reflex("speaking")
    assert ex.merge(plan, "speaking", emotion) == plan


def test_command_emotion_colours_speaking_and_done():
    assert ex.merge(ex.reflex("speaking"), "speaking", "happy").emotion == "happy"
    assert ex.merge(ex.reflex("done"), "done", "sad").emotion == "sad"
    assert ex.merge(ex.reflex("done"), "done", "sleepy").emotion == "sleep"
    assert ex.merge(ex.reflex("done"), "done", "police_lights").emotion == "police_lights"


def test_failed_and_timeout_reflex_beats_command_emotion():
    for stage in ("failed", "timeout"):
        plan = ex.reflex(stage)
        assert ex.merge(plan, stage, "happy", "victory") == plan


def test_emotion_does_not_leak_into_other_stages():
    for stage in ("addressed", "thinking", "executing"):
        plan = ex.reflex(stage)
        assert ex.merge(plan, stage, "happy") == plan


def test_known_gesture_is_added_unknown_is_dropped():
    assert ex.merge(ex.reflex("done"), "done", gesture="wink").gesture == "wink"
    assert ex.merge(ex.reflex("done"), "done", gesture="moonwalk").gesture == ""


def test_merge_of_no_plan_is_no_plan():
    assert ex.merge(None, "done", "happy") is None
