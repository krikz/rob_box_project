"""ADR-0175 §2.1: один инструмент Химеры. Схема узкая, исходы различимы, «спел» только после конца.

Мелодию ищет действующая библиотека на синтетическом индексе (не /opt и не /data). Профиль — заглушка шагов
0.1/0.2: клипы помечены ``clip:``, настоящих файлов Химеры тест не выдумывает. Живой робот здесь не проверяется.
"""

from __future__ import annotations

import sqlite3

import pytest

from rob_box_dialog import phrases
from rob_box_mcp_tools.engine.score_library import INDEX_FILE, ScoreLibrary
from rob_box_music import material as mt
from rob_box_music import works
from rob_box_music.model import Key, PitchEvent

from ._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.tools import chimera_decide as decide
    from rob_box_mcp_tools.tools.chimera import ACTIONS, TIMBRES, ChimeraPerformTool
_ros_stubs = _ros.fixture()

pytestmark = pytest.mark.unit


class _Node:
    def get_logger(self):
        return self

    def info(self, *_a, **_k):
        return None

    def debug(self, *_a, **_k):
        return None

    def warning(self, *_a, **_k):
        return None

    def error(self, *_a, **_k):
        return None


class FakePlayer:
    """Плеер-заглушка: помнит, играет ли, и не стартует после стопа."""

    def __init__(self) -> None:
        self.playing = False
        self.started = 0
        self.stopped = 0
        self.late = 0

    def start(self) -> bool:
        if self.stopped:
            self.late += 1
            return False
        self.playing = True
        self.started += 1
        return True

    def stop(self) -> bool:
        was = self.playing
        self.playing = False
        self.stopped += 1
        return was


class FakeHost:
    """Хост-заглушка: стадии вместо устройства. Считает ответы, чтобы поймать двойной."""

    def __init__(self, stage, player: FakePlayer | None = None) -> None:
        self.stage = stage
        self.player = player or FakePlayer()
        self.asks = 0
        self.plans: list = []

    def perform(self, plan):
        self.asks += 1
        self.plans.append(plan)
        if self.stage and self.stage.get("stage") == "cancelled" and self.stage.get("when") == "render":
            return "req-1", dict(self.stage)  # отмена на счёте: плеер не стартует
        if not self.player.start():
            return "req-1", {"stage": "cancelled", "when": "playback"}
        return "req-1", dict(self.stage) if self.stage else None

    def stop(self, when: str = "playback") -> bool:
        return self.player.stop()


def _note() -> PitchEvent:
    return PitchEvent(60, 0.0, 1.0, 3)


def _material(material_id: str, title: str, composer: str) -> mt.ScoreMaterial:
    return mt.ScoreMaterial(
        material_id=material_id,
        title=title,
        composer=composer,
        source="synthetic",
        license="PD",
        meter=(4, 4),
        bpm=None,
        key=Key(0, "major"),
        melody=(_note(),),
        chords=(mt.ChordSpan(0.0, 4.0, 0, "maj", 0),),
        phrases=(mt.Phrase(0, 1, "new", 1),),
    )


def library(tmp_path, materials) -> ScoreLibrary:
    """Синтетический ``score_index.db``, как пишет импортёр. Не боевая библиотека и не /data."""
    tmp_path.mkdir(parents=True, exist_ok=True)
    rows = []
    for m in materials:
        name = m.material_id.replace(":", "_") + ".json"
        (tmp_path / name).write_text(mt.to_json(m), encoding="utf-8")
        rows.append(works.score_index_row(m, bars=1, file=name))
    conn = sqlite3.connect(str(tmp_path / INDEX_FILE))
    works.write_score_index(conn, rows)
    conn.close()
    return ScoreLibrary(str(tmp_path))


def tool(host, lib=None, profile=None) -> ChimeraPerformTool:
    return ChimeraPerformTool(_Node(), host=host, library=lib, profile=profile or decide.stub_profile())


def test_schema_is_the_four_actions_and_three_timbres():
    params = {p.name: p for p in ChimeraPerformTool(_Node(), host=FakeHost(None)).parameters}
    assert params["action"].enum == list(ACTIONS) == ["call", "speak", "sing", "play"]
    assert params["action"].required is True
    assert params["timbre"].enum == list(TIMBRES) == ["male", "female", "child"]
    assert params["title"].required is False and params["title"].enum is None  # название — не список, а поиск


def test_profile_stub_is_marked_and_has_no_real_files():
    profile = decide.stub_profile()
    assert profile.stub is True and "0.1/0.2" in profile.note
    assert all(clip.startswith("clip:") for clip in profile.calls.values())


def test_call_speak_sing_play_are_different_plans(tmp_path):
    lib = library(tmp_path, [_material("local:hall", "В пещере горного короля", "Григ")])
    profile = decide.stub_profile()
    plans = [
        decide.decide("call", profile, creature="сова", call_name="уханье"),
        decide.decide("speak", profile, phrase_id="privet", timbre="female"),
        decide.decide("sing", profile, title="пещере короля", timbre="male", syllables="ла", library=lib),
        decide.decide("play", profile, title="пещере короля", library=lib),
    ]
    assert [p.action for p in plans] == ["call", "speak", "sing", "play"]
    assert all(p.plays for p in plans)
    assert plans[0].route == decide.ROUTE_CLIP and plans[2].route == decide.ROUTE_LONG
    assert plans[2].material_id == "local:hall" and plans[3].syllables == ""
    assert plans[2].timbre == "male" and plans[3].timbre == ""


def test_refusals_have_different_reasons(tmp_path):
    lib = library(
        tmp_path,
        [
            _material("local:a", "Лунная соната", "Бетховен"),
            _material("local:b", "Лунная река", "Манчини"),
        ],
    )
    profile = decide.stub_profile()
    reasons = [
        decide.decide("dance", profile).reason,
        decide.decide("call", profile, creature="сова", call_name="лай").reason,
        decide.decide("speak", profile, phrase_id="нет-такой", timbre="male").reason,
        decide.decide("sing", profile, title="соната", timbre="robot").reason,
        decide.decide("play", profile, title="нет такой пьесы", library=lib).reason,
        decide.decide("play", profile, title="лунная", library=lib).reason,
    ]
    assert reasons == [
        "unsupported_action",
        "unsupported_pairing",
        "unsupported_pronunciation",
        "unsupported_timbre",
        "unknown_title",
        "ambiguous_title",
    ]
    assert decide.distinct(reasons)


def test_host_failures_differ_from_each_other_and_from_refusals():
    plan = decide.decide("call", decide.stub_profile(), creature="сова", call_name="уханье")
    stages = [
        {"stage": "cancelled", "when": "render"},
        {"stage": "cancelled", "when": "playback"},
        {"stage": "missing_file"},
        {"stage": "offline"},
        {"stage": "skipped", "reason": "busy"},
    ]
    reasons = [decide.outcome_of(plan, s, "r").reason for s in stages]
    assert reasons == ["cancelled_render", "cancelled_playback", "missing_file", "offline", "busy"]
    assert decide.distinct(reasons)
    # missing_file и offline бывают и до хоста (пустой клип, нет сети у профиля), и как стадия хоста.
    # Остальные стадии с причинами плана не пересекаются.
    shared = {"missing_file", "offline"}
    assert set(reasons).isdisjoint(set(decide.DECIDE_REASONS) - shared)
    assert shared <= set(reasons)


def test_sang_only_after_finished_and_one_phrase():
    plan = decide.ChimeraPlan("play", "sing", route=decide.ROUTE_LONG, material_id="local:x", title="Этюд")
    started = decide.outcome_of(plan, {"stage": "started"}, "r1")
    finished = decide.outcome_of(plan, {"stage": "finished"}, "r1")
    assert started.sang is False and started.status == "pending"
    assert finished.sang is True and finished.status == "done"
    assert decide.sang_phrase(started, phrases.text) is None
    said = decide.sang_phrase(finished, phrases.text)
    assert said is not None and "Спел" in said and "Этюд" in said
    assert decide.sang_phrase(decide.outcome_of(plan, {"stage": "finished"}, "r1"), phrases.text) == said


def test_tool_plays_on_fake_host_and_does_not_say_sang_before_finish(tmp_path):
    lib = library(tmp_path, [_material("local:hall", "В пещере горного короля", "Григ")])
    playing = FakeHost({"stage": "started"})
    result = tool(playing, lib).execute(action="play", title="пещере короля")
    assert playing.asks == 1 and playing.player.started == 1 and playing.player.playing is True
    assert result.data["status"] == "pending" and result.data["sang"] is False
    assert result.message is None and result.success is False
    assert result.data["material_id"] == "local:hall"

    done = FakeHost({"stage": "finished"})
    finished = tool(done, lib).execute(action="sing", title="пещере короля", timbre="female", syllables="ла")
    assert finished.success is True and finished.data["sang"] is True
    assert finished.message is not None and "Спел" in finished.message
    assert finished.data["request_id"] == "req-1"


def test_stop_stops_the_fake_player_and_blocks_a_late_start():
    host = FakeHost({"stage": "started"})
    tool(host).execute(action="call", creature="тюлень", call="писк")
    assert host.player.playing is True
    assert host.stop("playback") is True
    assert host.player.playing is False and host.player.stopped == 1
    late = host.perform(decide.ChimeraPlan("play", "play", route=decide.ROUTE_LONG, material_id="local:z"))
    assert late[1]["stage"] == "cancelled" and host.player.late == 1 and host.player.playing is False


def test_cancel_during_render_never_starts_playback():
    host = FakeHost({"stage": "cancelled", "when": "render"})
    result = tool(host).execute(action="speak", phrase_id="privet", timbre="child")
    assert host.player.started == 0 and host.player.playing is False
    assert result.data["reason"] == "cancelled_render" and result.data["sang"] is False
    assert result.message is not None and "счита" in result.message


def test_unknown_title_does_not_ask_the_host(tmp_path):
    lib = library(tmp_path, [_material("local:hall", "В пещере горного короля", "Григ")])
    host = FakeHost({"stage": "finished"})
    result = tool(host, lib).execute(action="play", title="нет такой пьесы совсем")
    assert host.asks == 0 and host.player.started == 0
    assert result.data["reason"] == "unknown_title" and result.data["sang"] is False
    assert "не нашёл" in result.message.lower()


def test_one_request_one_phrase_no_second_reply():
    host = FakeHost({"stage": "finished"})
    result = tool(host).execute(action="call", creature="сова", call="уханье")
    assert host.asks == 1
    assert result.data["sang"] is False  # вызов звучал, но «спел» — только пение и игра
    assert "Спел" not in (result.message or "")
    assert result.message == phrases.text("chimera.done", title=" «clip:owl-hoot»", action="call")
    assert result.error is None  # успех — одна фраза в message; error пуст, второй реплики нет
