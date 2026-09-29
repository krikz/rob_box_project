"""test_scene_set_replay_conductor.py — изоляция прогона и дирижёр (ADR-0144 §4.1, §5).

``replay.py``: план контейнеров проходит проверку изоляции, а КАЖДЫЙ из
барьеров §5.1, будучи сломан поодиночке, ловится ``check_isolation``.
``conductor.py``: сценарии, SSML-заявка к TTS, пустой кадр, сборка
``scene.yaml``; все 13 сценариев набора проигрываются настоящим
``Conductor`` на поддельном рантайме (без ROS, без сна) и дают валидную
разметку.

Run:
  python -m pytest tests/unit/maintenance/test_scene_set_replay_conductor.py -v \
      -p no:cacheprovider --no-cov -o addopts=""
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
SCENE_SET = REPO_ROOT / "scripts" / "maintenance" / "scene_set"
sys.path.insert(0, str(SCENE_SET))

import conductor  # noqa: E402
import decoder_node  # noqa: E402
import replay  # noqa: E402
import scene_spec as ss  # noqa: E402

SCENE_SCRIPTS = sorted((SCENE_SET / "scenes").glob("s*.yaml"))


# ── replay: топики ───────────────────────────────────────────────────────────


def test_control_topics_are_never_played():
    assert not set(replay.INPUT_TOPICS) & set(replay.CONTROL_TOPICS)


def test_record_covers_inputs_controls_and_raw_audio():
    assert set(replay.INPUT_TOPICS) | set(replay.CONTROL_TOPICS) | {"/audio/audio"} == set(replay.RECORD_TOPICS)


def test_speech_audio_is_an_input():
    # stt_node/speaker_id_node слушают /audio/speech_audio, не /audio/audio (ADR-0144 §2.2).
    assert "/audio/speech_audio" in replay.INPUT_TOPICS
    assert "/voice/speaker/register" in replay.INPUT_TOPICS


def test_outputs_include_camera_info_for_clock_offset():
    assert "/camera/camera/color/camera_info" in replay.OUTPUT_TOPICS
    assert "/camera/camera/color/camera_info" in replay.INPUT_TOPICS


# ── replay: изоляция ─────────────────────────────────────────────────────────

LIVE_ENV = [
    "ROS_DOMAIN_ID=0",
    "ZENOH_SESSION_CONFIG_URI=/tmp/zenoh_session_config.json5",
    "FACE_IDENTIFY_THRESHOLD=0.6",
    "FACE_STORE_ROOT=",
    "ARCFACE_ENABLED=",
    "PATH=/usr/bin",
]


def plan():
    return replay.build_plan(
        "R1", "/home/ros2/scenes/_replay/R1/s01", "/home/ros2/scenes/s01/bag", "/home/ros2/rob_box_project",
        "/home/ros2/rob_box_project/scripts/maintenance/scene_set", "vh:1", "va:1", LIVE_ENV,
    )


def role(p, name):
    return next(c for c in p.all if c.role == name)


def test_default_plan_is_isolated():
    assert replay.check_isolation(plan()) == []


def test_face_env_carries_thresholds_but_not_live_graph():
    env = role(plan(), "face").env
    assert env["FACE_IDENTIFY_THRESHOLD"] == "0.6"
    assert env["FACE_STORE_ROOT"] == "/replay/faces"
    assert env["ROS_DOMAIN_ID"] == "77"
    assert env["ZENOH_SESSION_CONFIG_URI"] == "/replay/zenoh_session.json5"
    assert "PATH" not in env


def test_argv_shape():
    argv = role(plan(), "voice").argv("R1")
    assert argv[:3] == ["docker", "run", "-d"]
    assert "scene-replay-r1-voice" in argv
    assert "ROS_DOMAIN_ID=77" in argv
    assert "va:1" in argv
    player = role(plan(), "player").argv("R1")
    assert "--rm" in player and "-d" not in player


def _broken(mutate):
    p = plan()
    mutate(p)
    return replay.check_isolation(p)


@pytest.mark.parametrize("mutate, needle", [
    (lambda p: p.router_cfg["connect"].update(endpoints=["tcp/10.1.1.10:7447"]), "router: connect"),
    (lambda p: p.router_cfg["listen"].update(endpoints=["tcp/0.0.0.0:7457"]), "router: listen"),
    (lambda p: p.session["connect"].update(endpoints=["tcp/10.1.1.11:7447"]), "session: connect"),
    (lambda p: p.session["scouting"]["multicast"].update(enabled=True), "multicast"),
    (lambda p: p.session["transport"]["shared_memory"].update(enabled=True), "shared_memory"),
    (lambda p: role(p, "face").env.update(ROS_DOMAIN_ID="0"), "face: ROS_DOMAIN_ID"),
    (lambda p: role(p, "voice").env.update(ZENOH_SESSION_CONFIG_URI="/tmp/zenoh_session_config.json5"),
     "voice: чужой"),
    (lambda p: role(p, "face").env.update(FACE_STORE_ROOT="/data/faces"), "FACE_STORE_ROOT"),
    (lambda p: role(p, "face").volumes.append(("/home/ros2/rob_box_project/docker/vision/data/faces",
                                              "/data/faces", "rw")), "задевает data"),
    (lambda p: role(p, "voice").command.__setitem__(-1, role(p, "voice").command[-1].replace(
        "/replay/voice/speakers.db", "/data/speakers.db")), "voice: БД"),
    (lambda p: role(p, "voice").command.__setitem__(-1, role(p, "voice").command[-1].replace(
        "19112", "9112")), "порт"),
    (lambda p: role(p, "player").command.__setitem__(-1, role(p, "player").command[-1] + " /vision/hailo/events"),
     "контрольный"),
])
def test_each_isolation_barrier_is_checked(mutate, needle):
    bad = _broken(mutate)
    assert any(needle in b for b in bad), bad


def test_run_dir_under_data_is_refused():
    p = plan()
    p.run_dir = "/data/replay/R1"
    assert any("run_dir" in b for b in replay.check_isolation(p))


def test_prepare_run_dir_copies_seed_and_writes_configs(tmp_path):
    seed = tmp_path / "seed"
    (seed / "faces" / "p1").mkdir(parents=True)
    (seed / "faces" / "p1" / "meta.json").write_text("{}", encoding="utf-8")
    (seed / "voice").mkdir()
    (seed / "voice" / "speakers.db").write_bytes(b"x")
    run_dir = tmp_path / "run"
    replay.prepare_run_dir(str(run_dir), str(seed), plan())
    assert (run_dir / "faces" / "p1" / "meta.json").exists()
    assert (run_dir / "voice" / "speakers.db").read_bytes() == b"x"
    sess = json.loads((run_dir / "zenoh_session.json5").read_text(encoding="utf-8"))
    assert sess["connect"]["endpoints"] == [replay.ROUTER_ENDPOINT]
    # Сид не тронут: прогон пишет в копию.
    assert (seed / "voice" / "speakers.db").read_bytes() == b"x"


def test_prepare_run_dir_empty_seed(tmp_path):
    replay.prepare_run_dir(str(tmp_path / "run"), None, plan())
    assert list((tmp_path / "run" / "faces").iterdir()) == []


def test_dry_run_cli_reports_isolation(monkeypatch, capsys, tmp_path):
    monkeypatch.setattr(replay, "_inspect", lambda name, fmt: "[]" if "json" in fmt else "img:1")
    rc = replay.main(["--scene-dir", str(tmp_path / "s01"), "--run-id", "R9", "--empty-seed", "--dry-run"])
    assert rc == 0
    assert "isolation: OK" in capsys.readouterr().out


# ── decoder ──────────────────────────────────────────────────────────────────


def test_png_payload_strips_compressed_depth_header():
    assert decoder_node.png_payload(b"\x00" * 12 + b"\x89PNG....") == b"\x89PNG...."
    with pytest.raises(ValueError):
        decoder_node.png_payload(b"\x00" * 12 + b"JFIF")


# ── conductor: сценарий и TTS ────────────────────────────────────────────────

OWNER = {"label": "owner", "truth": "Дэнчик", "known_before": True}


@pytest.mark.parametrize("steps, msg", [
    ([], "пусто"),
    ([{"enter": "owner", "leave": "owner"}], "больше одного"),
    ([{"enter": "ghost"}], "ghost"),
    ([{"speech": "owner", "say": "x"}], "window_s"),
    ([{"mark": {"type": "enter", "who": "owner"}}], "mark.type"),
    ([{"wait_s": 3}], "пустой шаг"),
])
def test_parse_script_rejects(steps, msg):
    with pytest.raises(conductor.ScriptError, match=msg):
        conductor.parse_script({"scene": "x", "participants": [OWNER], "steps": steps})


def test_tts_request_is_ssml_operator_json():
    payload = json.loads(conductor.tts_request('Войдите в кадр & встаньте на метку <1.5>'))
    assert payload == {"ssml": "<speak>Войдите в кадр &amp; встаньте на метку &lt;1.5&gt;</speak>",
                       "source": "operator"}
    assert "text" not in payload  # tts_node молча выкидывает text без ssml


# ── conductor: пустой кадр ───────────────────────────────────────────────────


@pytest.mark.parametrize("cls, cx, anchors, region, expected", [
    ("person", 0.7, [], None, True),
    ("chair", 0.7, [], None, False),
    ("face", 0.2, [(0.1, 0.4)], None, False),   # маска на подставке законно в кадре
    ("face", 0.7, [(0.1, 0.4)], None, True),
    ("face", None, [(0.1, 0.4)], None, True),
    ("face", 0.7, [], (0.1, 0.4), False),        # уход маски: человек рядом не мешает
    ("face", 0.2, [], (0.1, 0.4), True),
])
def test_is_blocking(cls, cx, anchors, region, expected):
    assert conductor.is_blocking(cls, cx, anchors, region) is expected


def test_empty_watcher_needs_live_camera_and_hold():
    w = conductor.EmptyWatcher(start=100.0, hold_s=3.0)
    assert w.empty_since(104.0) is None  # кадров не было — «не видно» ≠ «пусто»
    for t in (100.2, 100.8, 101.4, 102.0, 102.6, 103.2):
        w.frame(t)
    assert w.empty_since(103.2) == 100.0


def test_empty_watcher_blocking_restarts_hold():
    w = conductor.EmptyWatcher(start=100.0, hold_s=3.0)
    for t in (100.5, 101.0, 101.5, 102.0, 102.5, 103.0, 103.5, 104.0, 104.5):
        w.frame(t)
        if t == 101.5:
            w.blocking(t)
    assert w.empty_since(104.0) is None
    assert w.empty_since(104.5) == 101.5


def test_empty_watcher_stale_camera_restarts_hold():
    w = conductor.EmptyWatcher(start=100.0, hold_s=3.0)
    w.frame(100.5)
    w.frame(103.0)  # 2.5 с без кадров — пустоту в этот промежуток не видели
    assert w.empty_since(103.5) is None
    for t in (103.5, 104.0, 104.5, 105.0, 105.5, 106.0):
        w.frame(t)
    assert w.empty_since(105.9) is None
    assert w.empty_since(106.0) == 103.0


def test_cx_summary():
    assert conductor.cx_summary([]) == "лиц не видно"
    assert "anchor_cx: [0.15, 0.35]" in conductor.cx_summary([0.2, 0.3])


# ── conductor: полный прогон сценариев на поддельном рантайме ────────────────


class FakeClock:
    def __init__(self):
        self.now = 1000.0

    def time(self):
        return self.now

    def sleep(self, s):
        self.now += s


class FakeRuntime:
    def __init__(self, clock):
        self.clock = clock
        self.said = []
        self.stt = []
        self.regions = []

    def say(self, text, timeout_s=25.0):
        self.said.append(text)
        self.clock.now += 4.0  # синтез + проигрывание
        return True

    def wait_empty(self, hold_s, timeout_s, region=None):
        self.regions.append(region)
        self.clock.now += hold_s
        return self.clock.now - hold_s


def run_script(path, monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr(conductor.time, "time", clock.time)
    monkeypatch.setattr(conductor.time, "sleep", clock.sleep)
    script = conductor.parse_script(yaml.safe_load(path.read_text(encoding="utf-8")))
    rt = FakeRuntime(clock)
    args = argparse.Namespace(empty_hold_s=3.0, empty_timeout_s=20.0, empty_retries=5)
    cond = conductor.Conductor(script, rt, args, log=lambda m: None)
    bag_start = clock.now - 3.0
    cond.run()
    doc = conductor.assemble_scene(script, cond.marks, bag_start, clock.now - bag_start, {"bag": "bag"})
    return script, rt, doc


def test_all_thirteen_scene_scripts_ship():
    assert [p.stem.split("_")[0] for p in SCENE_SCRIPTS] == [f"s{i:02d}" for i in range(13)]


@pytest.mark.parametrize("path", SCENE_SCRIPTS, ids=lambda p: p.stem)
def test_scene_script_runs_and_yields_valid_markup(path, monkeypatch):
    script, rt, doc = run_script(path, monkeypatch)
    scene = ss.parse_scene(doc)
    vis = ss.visits(scene)
    live_enters = [e for e in scene.events if e.type == "enter" and not scene.participants[e.who].is_static]
    # Каждый вход живого участника — после проверенно пустого кадра.
    assert all(v.verified_empty for v in vis if not scene.participants[v.who].is_static)
    assert len([e for e in scene.events if e.type == "empty"]) == len(live_enters)
    # Повелительные команды: никаких «можете».
    assert not any("можете" in s.lower() for s in rt.said)


def test_static_leave_checks_only_its_corridor(monkeypatch):
    _script, rt, doc = run_script(SCENE_SET / "scenes" / "s10_mask_away_and_back.yaml", monkeypatch)
    assert [0.05, 0.40] in [list(r) for r in rt.regions if r]
    mask_visits = [v for v in ss.visits(ss.parse_scene(doc)) if v.who == "mask"]
    assert len(mask_visits) == 2


def test_s07_introduction_marks_name_window(monkeypatch):
    _script, _rt, doc = run_script(SCENE_SET / "scenes" / "s07_mask_introduces.yaml", monkeypatch)
    scene = ss.parse_scene(doc)
    intro = next(e for e in scene.events if e.type == "introduce")
    w = ss.name_windows(scene, "mask")
    assert ss.allowed_names(w, intro.t - 0.1) == ()
    assert ss.allowed_names(w, intro.t) == ("Литвин",)


def test_s12_mask_never_nameable(monkeypatch):
    _script, _rt, doc = run_script(SCENE_SET / "scenes" / "s12_owner_voice_as_litvin.yaml", monkeypatch)
    scene = ss.parse_scene(doc)
    assert ss.first_allowed_at(ss.name_windows(scene, "mask"), 0, scene.duration) is None


def test_assemble_scene_relative_times_and_validation():
    script = conductor.parse_script({"scene": "x", "participants": [OWNER], "steps": [{"say": "hi"}]})
    marks = [{"type": "empty", "t_abs": 101.0}, {"type": "enter", "who": "owner", "t_abs": 102.5},
             {"type": "say", "who": "owner", "t_abs": 103.0, "until_abs": 110.0}]
    doc = conductor.assemble_scene(script, marks, 100.0, 5.0, {"bag": "bag"})
    assert [(e["type"], e["t"]) for e in doc["events"]] == [("empty", 1.0), ("enter", 2.5), ("say", 3.0)]
    assert doc["events"][2]["until"] == 10.0
    assert doc["duration"] == 10.0
    bad = copy.deepcopy(marks) + [{"type": "leave", "who": "ghost", "t_abs": 104.0}]
    with pytest.raises(ss.SceneSpecError):
        conductor.assemble_scene(script, bad, 100.0, 5.0, {})
