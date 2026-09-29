"""test_scene_set_spec_metrics.py — разметка сцены и метрики (ADR-0144 §4.2, §6).

Чистая логика ``scripts/maintenance/scene_set/{scene_spec,metrics,extract}.py``:
разбор ``scene.yaml``, визиты и проверенно пустой кадр, разрешённые имена во
времени, атрибуция событий лица/голоса, определения метрик, пересчёт часов
прогона. Без ROS и без робота.

Run:
  python -m pytest tests/unit/maintenance/test_scene_set_spec_metrics.py -v -p no:cacheprovider --no-cov -o addopts=""
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCENE_SET = REPO_ROOT / "scripts" / "maintenance" / "scene_set"
sys.path.insert(0, str(SCENE_SET))

import extract  # noqa: E402
import metrics  # noqa: E402
import scene_spec as ss  # noqa: E402

OWNER = {"label": "owner", "truth": "Дэнчик", "kind": "person", "known_before": True}
MASK = {"label": "mask", "truth": "Литвин", "kind": "mask", "known_before": False, "anchor_cx": [0.1, 0.4]}
PHOTO = {"label": "photo", "truth": "Дэнчик", "kind": "photo", "anchor_cx": [0.1, 0.4]}


def scene(events, participants=(OWNER,), **extra):
    raw = {"schema": 1, "scene": "sX", "participants": list(participants), "events": events}
    raw.update(extra)
    return ss.parse_scene(raw)


def face(t, name="", pid="p1", cx=0.7, kind="detection"):
    return {"t": t, "channel": "face", "kind": kind, "id": pid, "name": name, "tentative_name": "", "bbox_cx": cx}


def voice(t, name="", sid="v1", tentative=""):
    return {"t": t, "channel": "voice", "kind": "speaker", "id": sid, "name": name,
            "tentative_name": tentative, "bbox_cx": None}


# ── scene_spec: разбор ───────────────────────────────────────────────────────


def test_parse_minimal_scene():
    s = scene([{"t": 1, "type": "enter", "who": "owner"}, {"t": 5, "type": "leave", "who": "owner"}])
    assert s.scene == "sX"
    assert s.duration == 5
    assert s.participants["owner"].known_before is True


@pytest.mark.parametrize("raw, msg", [
    ({"scene": "x", "participants": []}, "participants: пусто"),
    ({"scene": "x", "participants": [dict(OWNER, kind="robot")]}, "kind"),
    ({"scene": "x", "participants": [dict(MASK, anchor_cx=[0.5, 0.2])]}, "anchor_cx"),
    ({"scene": "x", "participants": [OWNER, OWNER]}, "повторяется"),
    ({"scene": "x", "participants": [OWNER], "events": [{"t": 1, "type": "enter", "who": "ghost"}]}, "ghost"),
    ({"scene": "x", "participants": [OWNER], "events": [{"t": 1, "type": "say", "who": "owner"}]}, "until"),
    ({"scene": "x", "participants": [OWNER], "events": [{"t": 1, "type": "introduce", "who": "owner"}]}, "name"),
    ({"scene": "x", "participants": [OWNER], "events": [{"t": -1, "type": "empty"}]}, ">= 0"),
    ({"scene": "x", "participants": [OWNER], "events": [{"t": 1, "type": "dance"}]}, "неизвестен"),
    ({"scene": "x", "participants": [OWNER], "expected": {"max_records": {"ghost": 1}}}, "ghost"),
    ({"scene": "x", "participants": [OWNER], "duration": 1, "events": [{"t": 5, "type": "empty"}]}, "duration"),
    ({"scene": "x", "schema": 2, "participants": [OWNER]}, "schema"),
])
def test_parse_rejects_ambiguous_markup(raw, msg):
    with pytest.raises(ss.SceneSpecError, match=msg):
        ss.parse_scene(raw)


# ── scene_spec: визиты и пустой кадр ─────────────────────────────────────────


def test_visits_pairs_and_open_visit_closed_by_duration():
    s = scene([
        {"t": 1, "type": "empty"},
        {"t": 2, "type": "enter", "who": "owner"},
        {"t": 10, "type": "leave", "who": "owner"},
        {"t": 20, "type": "enter", "who": "owner"},
    ], duration=30)
    v = ss.visits(s)
    assert [(x.start, x.end, x.verified_empty) for x in v] == [(2, 10, True), (20, 30, False)]


def test_empty_mark_is_voided_by_a_live_enter_in_between():
    s = scene([
        {"t": 1, "type": "empty"},
        {"t": 2, "type": "enter", "who": "owner"},
        {"t": 3, "type": "enter", "who": "guest"},
    ], participants=(OWNER, dict(OWNER, label="guest", truth="Гость", known_before=False)))
    v = {x.who: x for x in ss.visits(s)}
    assert v["owner"].verified_empty is True
    assert v["guest"].verified_empty is False


def test_static_enter_does_not_void_empty_mark():
    s = scene([
        {"t": 1, "type": "enter", "who": "mask"},
        {"t": 2, "type": "empty"},
        {"t": 3, "type": "enter", "who": "owner"},
    ], participants=(OWNER, MASK))
    assert {x.who: x.verified_empty for x in ss.visits(s)}["owner"] is True


def test_present_at_start_opens_visit_at_zero():
    s = scene([{"t": 4, "type": "leave", "who": "mask"}], participants=(OWNER, dict(MASK, present_at_start=True)))
    assert [(v.who, v.start, v.end) for v in ss.visits(s)] == [("mask", 0.0, 4.0)]


@pytest.mark.parametrize("events", [
    [{"t": 1, "type": "enter", "who": "owner"}, {"t": 2, "type": "enter", "who": "owner"}],
    [{"t": 1, "type": "leave", "who": "owner"}],
])
def test_inconsistent_presence_is_an_error(events):
    with pytest.raises(ss.SceneSpecError):
        ss.visits(scene(events))


# ── scene_spec: разрешённые имена ────────────────────────────────────────────


def test_known_before_allows_truth_from_zero():
    s = scene([])
    assert ss.allowed_names(ss.name_windows(s, "owner"), 0.0) == ("Дэнчик",)


def test_introduce_allows_name_only_from_event():
    s = scene([{"t": 10, "type": "introduce", "who": "mask", "name": "Литвин"}], participants=(OWNER, MASK))
    w = ss.name_windows(s, "mask")
    assert ss.allowed_names(w, 9.9) == ()
    assert ss.allowed_names(w, 10.0) == ("Литвин",)
    assert ss.first_allowed_at(w, 0, 30) == 10.0
    assert ss.first_allowed_at(w, 0, 5) is None


def test_explicit_expected_replaces_rules():
    s = scene([{"t": 10, "type": "introduce", "who": "mask", "name": "Литвин"}], participants=(OWNER, MASK),
              expected={"allowed_names": {"mask": [{"from": 0, "names": []}]}})
    assert ss.allowed_names(ss.name_windows(s, "mask"), 50.0) == ()


def test_photo_gets_no_names_by_default():
    s = scene([], participants=(OWNER, PHOTO))
    assert ss.allowed_names(ss.name_windows(s, "photo"), 1.0) == ()


# ── metrics: атрибуция ───────────────────────────────────────────────────────


def _two_in_frame():
    return scene([
        {"t": 0, "type": "enter", "who": "mask"},
        {"t": 5, "type": "empty"},
        {"t": 10, "type": "enter", "who": "owner"},
        {"t": 40, "type": "leave", "who": "owner"},
    ], participants=(OWNER, MASK), duration=60)


def test_attribute_face_by_anchor_and_by_elimination():
    s = _two_in_frame()
    v = ss.visits(s)
    assert metrics.attribute_face(s, v, 20, 0.2, 1.5) == "mask"
    assert metrics.attribute_face(s, v, 20, 0.7, 1.5) == "owner"


def test_single_static_participant_outside_anchor_is_unattributed():
    s = _two_in_frame()
    v = ss.visits(s)
    # 50 с: владелец ушёл, в кадре по разметке только маска; лицо справа — не маска.
    assert metrics.attribute_face(s, v, 50, 0.8, 1.5) is None
    assert metrics.attribute_face(s, v, 50, 0.3, 1.5) == "mask"


def test_two_live_people_without_anchor_are_ambiguous():
    s = scene([{"t": 0, "type": "enter", "who": "owner"}, {"t": 0, "type": "enter", "who": "guest"}],
              participants=(OWNER, dict(OWNER, label="guest", truth="Гость")), duration=10)
    assert metrics.attribute_face(s, ss.visits(s), 5, 0.5, 1.5) is None


def test_attribute_face_respects_grace():
    s = _two_in_frame()
    v = ss.visits(s)
    assert metrics.attribute_face(s, v, 41.0, 0.7, 1.5) == "owner"
    assert metrics.attribute_face(s, v, 42.0, 0.7, 1.5) is None


def test_attribute_voice_uses_lag_and_latest_window():
    s = scene([
        {"t": 10, "type": "say", "who": "owner", "until": 14},
        {"t": 16, "type": "say", "who": "mask", "until": 20},
    ], participants=(OWNER, MASK))
    w = ss.say_windows(s)
    assert metrics.attribute_voice(w, 12, 8) == "owner"
    assert metrics.attribute_voice(w, 17, 8) == "mask"  # оба окна подходят — последнее
    assert metrics.attribute_voice(w, 29, 8) is None
    assert metrics.attribute_voice(w, 5, 8) is None


# ── metrics: определения ─────────────────────────────────────────────────────


def test_false_names_face_and_voice_counted_with_pairs():
    s = _two_in_frame()
    s = scene([e.__dict__ for e in s.events] + [{"t": 20, "type": "say", "who": "owner", "until": 22}],
              participants=(OWNER, MASK), duration=60)
    j = [face(20, "Дэнчик", "m1", 0.2), face(21, "Дэнчик", "o1", 0.7), voice(24, "Литвин", "v9")]
    m = metrics.compute(s, j)
    assert (m.false_face, m.false_voice) == (1, 1)
    assert m.false_pairs == {"mask<-Дэнчик (face)": 1, "owner<-Литвин (voice)": 1}


def test_tentative_voice_name_is_not_a_false_name():
    s = scene([{"t": 1, "type": "say", "who": "owner", "until": 3}])
    m = metrics.compute(s, [voice(4, "", "v1", tentative="Литвин")])
    assert m.false_voice == 0


def test_spoof_counted_separately_from_false_names():
    s = scene([{"t": 0, "type": "enter", "who": "photo"}], participants=(OWNER, PHOTO), duration=20)
    m = metrics.compute(s, [face(5, "Дэнчик", "o1", 0.2)])
    assert m.spoof_accepted == 1
    assert m.false_face == 0
    assert m.nameable_visits == 0  # фото не называемо — не входит в долю опознанных


def test_fragmentation_counts_distinct_ids_per_participant():
    s = scene([{"t": 0, "type": "enter", "who": "owner"}], duration=30)
    m = metrics.compute(s, [face(1, pid="a"), face(2, pid="b"), face(3, pid="a"), face(4, pid="")])
    assert m.fragmented_face == ["owner"]
    assert m.ids_face == {"owner": ["a", "b"]}


def test_fragmentation_limit_from_expected():
    s = scene([{"t": 0, "type": "enter", "who": "owner"}], duration=30, expected={"max_records": {"owner": 2}})
    assert metrics.compute(s, [face(1, pid="a"), face(2, pid="b")]).fragmented_face == []


def test_recognized_visit_and_latency_from_verified_empty():
    s = scene([
        {"t": 1, "type": "empty"},
        {"t": 5, "type": "enter", "who": "owner"},
        {"t": 30, "type": "leave", "who": "owner"},
    ], duration=40)
    m = metrics.compute(s, [face(6), face(8.5, "Дэнчик"), face(9, "Дэнчик")])
    assert (m.recognized_visits, m.nameable_visits) == (1, 1)
    assert m.latencies == [pytest.approx(3.5)]
    assert m.latency_unverified == 0


def test_latency_excluded_without_verified_empty_frame():
    # Пилот 29.09: владелец уже стоял в кадре — «задержка» бессмысленна.
    s = scene([{"t": 0, "type": "enter", "who": "owner"}], duration=20)
    m = metrics.compute(s, [face(2, "Дэнчик")])
    assert m.recognized_visits == 1
    assert m.latencies == []
    assert m.latency_unverified == 1


def test_latency_counts_from_introduction_when_later_than_enter():
    s = scene([
        {"t": 0, "type": "empty"},
        {"t": 1, "type": "enter", "who": "mask"},
        {"t": 10, "type": "introduce", "who": "mask", "name": "Литвин"},
    ], participants=(OWNER, MASK), duration=30)
    m = metrics.compute(s, [face(5, "", "m1", 0.2), face(14, "Литвин", "m1", 0.2)])
    assert m.latencies == [pytest.approx(4.0)]


def test_unrecognized_visit_counts_in_denominator():
    s = scene([{"t": 0, "type": "enter", "who": "owner"}], duration=20)
    m = metrics.compute(s, [face(2)])
    assert (m.recognized_visits, m.nameable_visits) == (0, 1)


def test_double_greetings_per_visit():
    s = scene([
        {"t": 0, "type": "enter", "who": "owner"}, {"t": 10, "type": "leave", "who": "owner"},
        {"t": 20, "type": "enter", "who": "owner"},
    ], duration=40)
    j = [face(1, kind="encounter"), face(5, kind="encounter"), face(21, kind="encounter")]
    assert metrics.compute(s, j).double_greetings == 1


def test_unattributed_events_are_reported_not_dropped():
    s = scene([{"t": 10, "type": "enter", "who": "owner"}], duration=20)
    m = metrics.compute(s, [face(2, "Дэнчик"), face(3), voice(4, "Дэнчик")])
    assert (m.unattributed, m.unattributed_named) == (3, 2)
    assert m.false_face == 0


def test_report_has_total_and_not_covered_lines():
    s = scene([{"t": 0, "type": "enter", "who": "owner"}], duration=20)
    text = metrics.render_report([metrics.compute(s, [face(1, "Дэнчик")], "v1.1-replay")], "v1.1-replay")
    assert "TOTAL" in text
    assert "not covered" in text
    assert text.count("  - ") >= len(ss.NOT_COVERED)
    assert "1/1" in text


@pytest.mark.parametrize("line", ['{"t": "x", "channel": "face", "kind": "detection"}',
                                  '{"t": 1, "channel": "lidar", "kind": "detection"}'])
def test_journal_rejects_bad_lines(line):
    with pytest.raises(metrics.JournalError):
        metrics.parse_journal([line])


def test_split_case_handles_windows_paths():
    assert metrics._split_case("C:/s/scene.yaml:C:/r/journal.jsonl") == ("C:/s/scene.yaml", "C:/r/journal.jsonl")


def test_cli_end_to_end(tmp_path, capsys):
    import yaml

    sy = tmp_path / "scene.yaml"
    sy.write_text(yaml.safe_dump({"scene": "s01", "participants": [OWNER],
                                  "events": [{"t": 1, "type": "empty"}, {"t": 2, "type": "enter", "who": "owner"}],
                                  "duration": 20}, allow_unicode=True), encoding="utf-8")
    jl = tmp_path / "journal.jsonl"
    jl.write_text("\n".join(json.dumps(r, ensure_ascii=False) for r in
                            [face(3, "Дэнчик"), face(4, "Литвин", kind="encounter")]), encoding="utf-8")
    assert metrics.main(["--case", f"{sy}:{jl}", "--label", "t"]) == 0
    assert metrics.main(["--case", f"{sy}:{jl}", "--strict"]) == 1
    assert "FALSE NAME owner<-Литвин (face) x1" in capsys.readouterr().out


# ── extract ──────────────────────────────────────────────────────────────────


def test_vision_event_detection_and_encounter():
    det = extract.vision_event_record(
        {"event_type": "face", "embedding_id": "p1", "display_name": "Дэнчик", "attributes_json": "", "bbox_cx": 0.4},
        3.0)
    assert det["kind"] == "detection" and det["name"] == "Дэнчик" and det["bbox_cx"] == 0.4
    marker = json.dumps({"encounter": "start", "person_id": "p2", "name": "Литвин"}, ensure_ascii=False)
    enc = extract.vision_event_record(
        {"event_type": "face", "embedding_id": "", "display_name": "", "attributes_json": marker, "bbox_cx": 0.2}, 4.0)
    assert (enc["kind"], enc["id"], enc["name"]) == ("encounter", "p2", "Литвин")


def test_vision_event_non_face_and_broken_marker():
    assert extract.vision_event_record({"event_type": "person"}, 1.0) is None
    r = extract.vision_event_record({"event_type": "face", "attributes_json": "{broken", "bbox_cx": 0.1}, 1.0)
    assert r["kind"] == "detection"


def test_speaker_result_records():
    known = extract.speaker_result_record(json.dumps({"is_known": True, "speaker_id": "s1", "name": "Дэнчик"}), 2.0)
    assert (known["id"], known["name"]) == ("s1", "Дэнчик")
    tent = extract.speaker_result_record(
        json.dumps({"is_known": True, "speaker_id": "s1", "name": None, "tentative_name": "Дэнчик"}), 2.0)
    assert (tent["name"], tent["tentative_name"]) == ("", "Дэнчик")
    assert extract.speaker_result_record(json.dumps({"event": "registered", "name": "X"}), 2.0) is None
    assert extract.speaker_result_record("not json", 2.0) is None


def test_bag_start_and_offsets():
    meta = {"rosbag2_bagfile_information": {"starting_time": {"nanoseconds_since_epoch": 1_700_000_000_500_000_000}}}
    assert extract.bag_start_s(meta) == pytest.approx(1_700_000_000.5)
    assert extract.estimate_offset([]) is None
    # Прогон: доставка 0.05 с + сдвиг часов 1000 с; сцена: доставка 0.05 с.
    replay = [(1000.0 + s + 0.05, s) for s in (1.0, 2.0, 3.0)]
    scene_pairs = [(s + 0.05, s) for s in (1.0, 2.0, 3.0)]
    assert extract.replay_clock_offset(replay, scene_pairs) == pytest.approx(1000.0)
    assert extract.replay_clock_offset(replay, []) is None


def test_build_journal_converts_to_scene_clock():
    msgs = [
        (extract.SPEAKER_TOPIC, 1105.0, json.dumps({"is_known": False})),
        (extract.VISION_TOPIC, 1102.0, {"event_type": "face", "bbox_cx": 0.5}),
        ("/other", 1101.0, None),
    ]
    j = extract.build_journal(msgs, scene_start=100.0, offset=1000.0)
    assert [(r["t"], r["channel"]) for r in j] == [(2.0, "face"), (5.0, "voice")]
