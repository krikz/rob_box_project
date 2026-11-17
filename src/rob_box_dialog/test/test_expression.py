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
    assert ex.reflex("executing", "action.media", started).emotion == ex.music_scene(None) == "music_groove"


@pytest.mark.parametrize("bpm,scene", [(None, "music_groove"), (0, "music_groove"), (-5, "music_groove"), (60, "music_vibe"),
                                       (99.9, "music_vibe"), (100, "music_groove"), (124.9, "music_groove"),
                                       (125, "music_shades"), (130, "music_shades"), (200, "music_shades")])
def test_4010_music_scene_of_a_new_track_is_chosen_by_tempo_as_before(bpm, scene):
    assert ex.music_scene(bpm) == scene


def test_4010_music_scene_walks_the_circle_whatever_the_tempo():
    """Один темп весь сет: следующий трек и каждые MUSIC_SCENE_HOLD_S секунд - следующая сцена круга."""
    circle = kn.MUSIC_SCENES
    for bpm in (None, 90, 118, 130, 170):
        first = circle.index(ex.music_scene(bpm))
        assert [ex.music_scene(bpm, track_no=n) for n in range(4)] == [circle[(first + n) % 3] for n in range(4)]
        step = kn.MUSIC_SCENE_HOLD_S
        assert [ex.music_scene(bpm, 0, t) for t in (0, step - 0.1, step, 2 * step, 3 * step)] == [
            circle[first], circle[first], circle[(first + 1) % 3], circle[(first + 2) % 3], circle[first]]
        assert ex.music_scene(bpm, 2, 2 * step) == circle[(first + 4) % 3]    # трек и время складываются


def test_4010_set_of_one_tempo_uses_every_music_scene():
    seen = {ex.music_scene(130.0, track_no=n, elapsed_s=t) for n in range(3) for t in (0, 31, 62)}
    assert seen == set(kn.MUSIC_SCENES) and len(seen) == 3


def test_4010_music_scene_ignores_negative_input():
    assert ex.music_scene(130, -3, -50) == ex.music_scene(130, 0, 0)


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


# -- экспрессия команды v2 на итоге хода (#3997) ----------------------------------------------------------------------
FROWN = {"eyes": "half", "mouth": "frown", "duration_s": 6}


def test_command_face_scene_colours_done_with_the_emotion():
    """Команда ``{emotion: sad, face}`` красит рефлекс ``done``: сцена ``sad``, сцена лица LLM — в том же плане; earcon
    и время жизни остаются от рефлекса."""
    plan = ex.merge(ex.reflex("done"), "done", "sad", "wink", FROWN)
    assert (plan.emotion, plan.gesture, plan.face) == ("sad", "wink", FROWN)
    assert (plan.earcon, plan.ttl_ms) == (ex.reflex("done").earcon, ex.reflex("done").ttl_ms)


def test_command_face_scene_never_beats_failure_or_other_stages():
    for stage in ("failed", "timeout", "addressed", "thinking"):
        assert ex.merge(ex.reflex(stage), stage, "happy", face=FROWN) == ex.reflex(stage), stage


@pytest.mark.parametrize("emotion, gesture, face, shown", [
    ("neutral", None, None, False), ("idle", None, None, False), ("", None, None, False),
    ("sad", None, None, True), ("victory", None, None, True), ("neutral", "wink", None, True),
    ("neutral", "moonwalk", None, False), ("neutral", None, FROWN, True),
])
def test_shows_says_whether_the_command_has_anything_for_the_face(emotion, gesture, face, shown):
    assert ex.shows(emotion, gesture, face) is shown


# -- тишина (#3984, решение Шифу 23.10) -------------------------------------------------------------------------
def test_silenced_reflex_row_is_the_one_place_of_the_taped_face_and_the_ring_animation():
    row = kn.REFLEXES["silenced"]
    assert ex.silent_face() == row.face[0] == "taped_mouth" and row.ring == "silence"
    assert "taped_mouth" in kn.LAB_SCENES and "silence" in kn.RING_MODES
    assert ex.silent_face() not in {s for st in kn.TURN_STAGES for s in kn.REFLEXES[st].face}


def test_in_silence_heard_becomes_the_muffled_sound_done_is_dropped_unknown_earcon_never_passes():
    assert ex.silent_earcon("heard") == "muffled" and ex.silent_earcon("fail") == "muffled"
    assert ex.silent_earcon("done") == "" and ex.silent_earcon("") == "" and ex.silent_earcon(None) == ""
    assert ex.silent_earcon("button_click") == ""
    assert ex.silent_earcon(ex.reflex("addressed").earcon) != ex.reflex("addressed").earcon


def test_session_silence_reads_the_silence_scene_of_the_personality_only():
    from types import SimpleNamespace as ns

    assert ex.session_silence(ns(agent="personality", scenes=[ns(kind="Persona"), ns(kind="Silence")])) is True
    assert ex.session_silence(ns(agent="personality", scenes=[ns(kind="Persona")])) is False
    assert ex.session_silence(ns(agent="personality", scenes=[])) is False
    assert ex.session_silence(ns(agent="telegram", scenes=[ns(kind="Silence")])) is None
    assert ex.session_silence(ns(speaker_azimuth_deg=1.0)) is None
