"""DJ-сет v2 на симуляторе клока Renardo (ADR-0149 PR-5, эпик #3312).

Настоящие ``SetSession`` + ``PlayerOwner`` + ``RenardoAdapter`` + ``compose``/``render``; вместо
Renardo — клок, который исполняет запланированное по долям (как ``TempoClock``: ``set_time``
чистит очередь), и плееры, встающие на ``next_bar``. Без ROS и без звука.
"""

import heapq
from dataclasses import replace
import itertools
import json
import os
import pathlib
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from rob_box_mcp_tools.engine.player_owner import PlayerOwner
from rob_box_mcp_tools.engine.renardo_adapter import HANDOFF_STOP_BEATS, RenardoAdapter
from rob_box_mcp_tools.engine.session import NEARLY_LEAD_BEATS, SetSession, compose_source
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, library_melodies
from rob_box_music import knowledge as kn
from rob_box_music.set_plan import seeded_plan
from rob_box_music.theme import ThemeProfile
from rob_box_voice.core.music_player_state import parse_music_state

pytestmark = pytest.mark.unit

BPM = 132
PROFILE = ThemeProfile("тест", "club", BPM, 9, "minor", (), None)
PLAN = seeded_plan(PROFILE, 7, set_id="s7")  # один план на сет — один темп


class SimClock:
    """Очередь по долям; ``run_until`` исполняет запланированное по порядку."""

    meter = (4, 4)

    def __init__(self, beat=37.3):
        self.beat, self.bpm, self.latency, self.now_flag = beat, 120.0, 0.25, False
        self.calls, self._queue, self._seq = [], [], itertools.count()

    def now(self):
        return self.beat

    def next_bar(self):
        return self.beat + (4 - self.beat % 4)

    def update_tempo_now(self, bpm):
        self.calls.append(("tempo", bpm))
        self.bpm = float(bpm)

    def set_time(self, beat):
        self.calls.append(("set_time", beat))
        self._queue.clear()  # как TempoClock.set_time
        self.beat = beat

    def get_bpm(self):
        return self.bpm

    def schedule(self, fn, beat):
        heapq.heappush(self._queue, (float(beat), next(self._seq), fn))

    def run_until(self, target):
        while self._queue and self._queue[0][0] <= target:
            beat, _, fn = heapq.heappop(self._queue)
            self.beat = beat
            fn()
        self.beat = target


class Player:
    def __init__(self, clock, name):
        self.clock, self.name, self.event_index, self.playing, self.stops = clock, name, None, False, 0
        self.stopped_at = []

    def __rshift__(self, _other):
        self.event_index, self.playing = self.clock.next_bar(), True
        return self

    def stop(self):
        self.playing, self.stops = False, self.stops + 1
        self.stopped_at.append(self.clock.beat)


#: Все синты тембров v2 (``knowledge.TIMBRES``): сервер-заглушка знает их все.
V2_SYNTHS = frozenset(x for fam in (*kn.STYLES["club"].timbres.values(), kn.SONG_TIMBRES)
                      for synths in fam.values() for x in synths)


class Log:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(("info", msg))

    def warning(self, msg):
        self.lines.append(("warning", msg))

    def warnings(self):
        return [m for level, m in self.lines if level == "warning"]


def _rig(source=None, submit=None, *, exec_fails=False):
    clock = SimClock()
    slots = [s for deck in kn.DECK_SLOTS.values() for s in deck]
    samples = SimpleNamespace(getBufferFromSymbol=lambda *a: SimpleNamespace(bufnum=7), loadBuffer=lambda *a: 7)
    ns = {"Clock": clock, "Samples": samples,
          "play": lambda *a, **k: None, "var": lambda *a, **k: None, "Scale": SimpleNamespace(chromatic=None),
          "loop": lambda *a, **k: None}  # слои сэмплов DJ_Dave (PR-3d)
    ns.update({synth: (lambda *a, **k: None) for synth in V2_SYNTHS})  # тембры темы (PR-3c) — вся таблица
    ns.update({s: Player(clock, s) for s in slots})
    sent = []
    adapter = RenardoAdapter(lambda: ns, lambda: V2_SYNTHS, lambda *a: sent.append(a))
    states, events, log = [], [], Log()
    owner = PlayerOwner(adapter, states.append, lambda e: events.append(json.loads(e)), logger=log,
                        clock=lambda: 1000.0)
    source = source or compose_source(PLAN)
    session = SetSession(owner, source, set_id="s7", bpm=BPM, dj={"theme": "тест"},
                         submit=submit or (lambda fn: fn()), logger=log)
    rig = SimpleNamespace(clock=clock, ns=ns, sent=sent, owner=owner, states=states, events=events, log=log,
                          session=session, adapter=adapter)
    return rig


def _started(rig):
    return [e for e in rig.events if e["event"] == "started"]


def _form(rig):
    return _started(rig)[0]["form_beats"]


#: Блэнд соседних треков сета (PR-8): 8 тактов, своп баса на 4-м (``compose.transition``).
BLEND = 8 * 4


def test_three_tracks_blend_on_two_decks_with_phase_zero_and_the_leaving_deck_is_freed():
    rig = _rig()
    assert rig.session.start()["ok"] is True
    rig.clock.run_until(rig.clock.beat + 2)  # трек 1 встал
    first = _started(rig)[0]
    form, s0 = first["form_beats"], first["start_beat"]
    rig.clock.run_until(s0 + form - BLEND / 2)  # середина блэнда 1→2
    assert rig.ns["d1"].playing and rig.ns["a1"].playing, "оба трека звучат в блэнде"
    rig.clock.run_until(s0 + 3 * form - 3 * BLEND - 3)  # до исполнения трека 4
    started = _started(rig)
    # N+1 встаёт за 8 тактов до границы формы N, с фазой 0 на такте
    assert [e["start_beat"] for e in started] == [s0, s0 + form - BLEND, s0 + 2 * form - 2 * BLEND]
    assert [e["deck"] for e in started] == ["A", "B", "A"]  # блэнд на другой деке
    assert all(e["phase_in_form"] == 0.0 and e["players_aligned"] and e["late_beats"] == 0.0 for e in started)
    assert [e["track_id"].split(":")[1] for e in started] == ["01", "02", "03"]
    kinds = [e["event"] for e in rig.events]
    # N+1 в очереди сразу после started(N), nearly_finished — до входа (A3: переход без LLM, по событию)
    assert kinds[:4] == ["started", "queued", "nearly_finished", "started"]
    nearly = [e for e in rig.events if e["event"] == "nearly_finished"]
    assert nearly[0]["form_end_beat"] == s0 + form and nearly[0]["lead_beats"] == NEARLY_LEAD_BEATS
    queued = [e for e in rig.events if e["event"] == "queued"]
    assert queued[0]["expected_previous"] == started[0]["track_id"]
    # уходящая дека снята за 1/32 до границы СВОЕЙ формы (не до входа N+1) и освобождена
    assert s0 + form - HANDOFF_STOP_BEATS in rig.ns["d1"].stopped_at
    assert rig.ns["a1"].stopped_at[-1] == started[1]["start_beat"] + form - HANDOFF_STOP_BEATS
    assert rig.ns["d1"].playing and not rig.ns["a1"].playing
    infos = [m for level, m in rig.log.lines if level == "info"]
    assert sum("deck A free" in m for m in infos) == 1 and sum("deck B free" in m for m in infos) == 1
    assert sum("блэнд трек" in m for m in infos) == 3  # 1→2, 2→3 и уже поставленный 3→4
    snap = json.loads(rig.states[-1])
    assert snap["state"] == "playing" and snap["dj"]["track_no"] == 3 and snap["dj"]["set_id"] == "s7"
    assert not rig.log.warnings()


def _master(rig):
    """``/n_set 999`` движка: [(доля клока, {ручка: значение})]."""
    return [(beat, dict(zip(a[2::2], a[3::2]))) for beat, a in rig.master]


def test_track_energy_moves_the_master_trim_over_the_blend_and_stop_resets_it():
    """PR-7, ADR-0149 §3.10/§4.6: энергия трека плана → ``trim`` после динамики на доле его старта; в блэнде
    громкость переезжает за длину блэнда; профиль выравнивателя сета; стоп — дефолты (правило сброса)."""
    rig = _rig()
    rig.master = []
    send = rig.adapter._send_osc
    rig.adapter._send_osc = lambda *a: (rig.master.append((rig.clock.beat, a)) if a[1] == 999 else None, send(*a))
    rig.session.start()
    first = rig.clock.beat + 2
    rig.clock.run_until(first)
    s0, form = _started(rig)[0]["start_beat"], _form(rig)
    rig.clock.run_until(s0 + 2 * form - BLEND + 1)  # трек 3 встал
    starts = [e["start_beat"] for e in _started(rig)]
    sets = [(b, m) for b, m in _master(rig) if b in starts]  # между стартами — дуга секций (тест ниже)
    assert [b for b, _m in sets] == starts, "мастер трека — на доле его старта"
    energies = [PLAN.track(no).energy for no in (1, 2, 3)]
    intro = kn.SECTION_TRIM_DB["intro"][0]
    assert [m["trim"] for _b, m in sets] == [kn.ENERGY_TRIM_DB[e] + intro for e in energies]
    assert all({k: m[k] for k in kn.SET_LEVELER} == kn.SET_LEVELER for _b, m in sets)
    assert sets[0][1]["trimLag"] == kn.TRIM_LAG_S
    assert all(m["trimLag"] == pytest.approx(BLEND * 60.0 / BPM) for _b, m in sets[1:]), "переезд за блэнд"
    with patch("rob_box_mcp_tools.engine.renardo_adapter.time.sleep"):
        rig.session.stop()
    assert _master(rig)[-1][1] == {"trimLag": kn.TRIM_LAG_S, **kn.MASTER_DEFAULTS}


def _tap_master(rig):
    rig.master = []
    send = rig.adapter._send_osc
    rig.adapter._send_osc = lambda *a: (rig.master.append((rig.clock.beat, a)) if a[1] == 999 else None, send(*a))


def test_section_arc_moves_the_trim_inside_the_track_and_the_newest_track_owns_it():
    """Дуга громкости (отзыв эксперта 05.10, A6/A9): build поднимается к дропу всю секцию, брейк проваливается,
    второй дроп — пик трека; после старта следующего трека дугу ведёт он — хвост уходящего мастер не трогает."""
    SECTIONS = kn.STYLES["club"].form

    rig = _rig()
    _tap_master(rig)
    rig.session.start()
    rig.clock.run_until(rig.clock.beat + 2)
    s0, form = _started(rig)[0]["start_beat"], _form(rig)
    rig.clock.run_until(s0 + form - BLEND + 1)  # трек 2 встал
    start2 = _started(rig)[1]["start_beat"]
    base = kn.ENERGY_TRIM_DB[PLAN.track(1).energy]
    expected, beat = [], 0.0
    for name, bars, _energy, _roles in SECTIONS:
        offset, rise = kn.SECTION_TRIM_DB[name]
        lag = bars * 4 * 60.0 / BPM if rise else kn.TRIM_LAG_S
        if 0 < beat and s0 + beat < start2:
            expected.append((s0 + beat, base + offset, pytest.approx(lag, abs=1e-3)))
        beat += bars * 4
    inside = [(b, m["trim"], m["trimLag"]) for b, m in _master(rig) if s0 < b < start2]
    assert inside == expected and len(expected) >= 4
    trims = {name: base + kn.SECTION_TRIM_DB[name][0] for name in kn.SECTION_TRIM_DB}
    assert trims["build"] < trims["drop"] < trims["drop2"] == base, "восхождение к пику"
    assert trims["break"] <= trims["drop"] - 4, "брейк проваливается"
    assert all(m["trim"] <= kn.ENERGY_TRIM_DB[5] for _b, m in _master(rig)), "громче trim трека не бывает"
    # хвост трека 1 (outro, outro_tail) приходится на блэнд: мастер уже ведёт трек 2
    rig.clock.run_until(start2 + 4)
    tail = [m["trim"] for b, m in _master(rig) if b > start2]
    assert tail == []


def test_stop_ends_the_arc():
    rig = _rig()
    _tap_master(rig)
    rig.session.start()
    rig.clock.run_until(rig.clock.beat + 2)
    s0 = _started(rig)[0]["start_beat"]
    with patch("rob_box_mcp_tools.engine.renardo_adapter.time.sleep"):
        rig.session.stop()
    seen = len(_master(rig))
    assert _master(rig)[-1][1] == {"trimLag": kn.TRIM_LAG_S, **kn.MASTER_DEFAULTS}
    rig.clock.run_until(s0 + _form(rig))
    assert len(_master(rig)) == seen, "после стопа дуга молчит: trim 0 остаётся"


def test_master_defaults_are_the_synthdef_defaults_and_trim_sits_after_the_dynamics():
    """Дефолты ручек движка = дефолты ``masterfilter.scd`` (v1 без движка звучит как раньше: trim 0)."""
    root = next(p for p in pathlib.Path(__file__).resolve().parents if (p / "docker").is_dir())
    scd = (root / "docker/vision/voice_assistant/custom_synthdefs/masterfilter.scd").read_text(encoding="utf-8")
    code = "\n".join(line.split("//", 1)[0] for line in scd.splitlines())
    head = code.index("|", code.index("SynthDef.new(\\masterfilter"))
    args = dict(item.split("=") for item in "".join(code[head + 1:code.index("|", head + 1)].split()).split(","))
    assert {k: float(args[k]) for k in kn.MASTER_DEFAULTS} == dict(kn.MASTER_DEFAULTS)
    assert float(args["trim"]) == 0.0 and "trimLag" in args
    assert code.index("out = out * Lag.kr(gain, lag);") < code.index("Lag.kr(trimDb.dbamp, trimLag)") < code.index(
        "ReplaceOut.ar(")
    assert ".clip(-40, 0)" in code, "trim только вниз: пик не выше потолка лимитера (A7)"


def test_stop_during_a_blend_silences_both_decks():
    rig = _rig()
    rig.session.start()
    rig.clock.run_until(rig.clock.beat + 2)
    s0, form = _started(rig)[0]["start_beat"], _form(rig)
    rig.clock.run_until(s0 + form - BLEND / 2)
    with patch("rob_box_mcp_tools.engine.renardo_adapter.time.sleep"):
        rig.session.stop()
    rig.clock.run_until(s0 + 3 * form)
    assert not any(p.playing for p in rig.ns.values() if isinstance(p, Player))
    assert len(_started(rig)) == 2 and not any("free" in m for _level, m in rig.log.lines)


def test_forms_that_do_not_mix_fall_back_to_a_splice_on_the_form_boundary():
    """``blend_bars == 0`` (другая длина блэнда) — стык встык на границе формы (PR-5), громко в логе."""
    good = compose_source(PLAN)

    def source(no, deck):
        track = good(no, deck)
        return replace(track, transition_in=replace(track.transition_in, phrase_bars=16)) if no == 2 else track

    rig = _rig(source=source)
    rig.session.start()
    rig.clock.run_until(rig.clock.beat + 2)
    s0, form = _started(rig)[0]["start_beat"], _form(rig)
    rig.clock.run_until(s0 + form + 1)
    started = _started(rig)
    assert [e["start_beat"] for e in started] == [s0, s0 + form] and started[1]["phase_in_form"] == 0.0
    assert any("блэнда нет" in w for w in rig.log.warnings())
    assert rig.ns["d1"].stopped_at[-1] == s0 + form - HANDOFF_STOP_BEATS


def test_one_tempo_per_set_is_set_once_and_never_by_set_time_on_transitions():
    rig = _rig()
    rig.session.start()
    rig.clock.run_until(rig.clock.beat + 2)
    s0, form = _started(rig)[0]["start_beat"], _form(rig)
    rig.clock.run_until(s0 + 3 * form - 1)
    assert [c for c in rig.clock.calls if c[0] == "tempo"] == [("tempo", BPM)]
    assert len([c for c in rig.clock.calls if c[0] == "set_time"]) == 1  # только старт сета
    assert {e["bpm"] for e in _started(rig)} == {float(BPM)}


def test_next_not_ready_extends_the_playing_track_instead_of_silence():
    deferred = []
    rig = _rig(submit=deferred.append)
    rig.session.start()
    rig.clock.run_until(rig.clock.beat + 2)
    s0, form = _started(rig)[0]["start_beat"], _form(rig)
    rig.clock.run_until(s0 + form + 8)  # граница формы прошла, N+1 так и не скомпонован
    assert len(_started(rig)) == 1 and rig.ns["d1"].playing  # трек 1 играет второй проход
    assert any("продлеваю" in w for w in rig.log.warnings())
    assert len(deferred) == 1  # повторная компоновка не плодится, пока первая в работе
    deferred.pop()()  # N+1 готов к середине второго прохода
    rig.clock.run_until(s0 + 2 * form + 1)
    started = _started(rig)  # блэнд в конец второго прохода
    assert [e["start_beat"] for e in started] == [s0, s0 + 2 * form - BLEND] and started[1]["phase_in_form"] == 0.0
    nearly = [e["form_end_beat"] for e in rig.events if e["event"] == "nearly_finished"]
    assert nearly == [s0 + form, s0 + 2 * form]


def _source_with(bad_no, make_bad):
    good = compose_source(PLAN)

    def source(no, deck):
        return make_bad(no, deck) if no == bad_no else good(no, deck)

    return source


def _other_tempo(no, deck):
    from rob_box_music.arrange.compose import compose

    return compose(replace(PLAN, bpm=BPM + 4), no, deck=deck)  # чужой темп


def _broken(no, deck):
    raise ValueError("пэд не лёг")


@pytest.mark.parametrize("make_bad,reason", [(_other_tempo, "tempo_mismatch"), (_broken, "compose_error")])
def test_rejected_next_track_is_loud_and_the_set_does_not_go_silent(make_bad, reason):
    rig = _rig(source=_source_with(2, make_bad))
    rig.session.start()
    rig.clock.run_until(rig.clock.beat + 2)
    s0, form = _started(rig)[0]["start_beat"], _form(rig)
    rig.clock.run_until(s0 + 2 * form - 1)
    rejected = [e for e in rig.events if e["event"] == "rejected"]
    assert len(rejected) >= 1 and {e["reason"] for e in rejected} == {reason}
    assert len(_started(rig)) == 1 and rig.ns["d1"].playing  # трек 1 продлён, тишины нет
    assert [c for c in rig.clock.calls if c[0] == "tempo"] == [("tempo", BPM)]  # чужой темп не применён


def test_stop_closes_the_set_and_nothing_scheduled_revives_it():
    rig = _rig()
    rig.session.start()
    rig.clock.run_until(rig.clock.beat + 2)
    s0, form = _started(rig)[0]["start_beat"], _form(rig)
    rig.clock.run_until(s0 + form - NEARLY_LEAD_BEATS + 1)  # N+1 в очереди, стык запланирован
    with patch("rob_box_mcp_tools.engine.renardo_adapter.time.sleep"):
        result = rig.session.stop()
    assert result["ok"] is True and result["set_id"] == "s7"
    rig.clock.run_until(s0 + 4 * form)
    assert len(_started(rig)) == 1
    assert not any(p.playing for name, p in rig.ns.items() if isinstance(p, Player))
    assert [a[:2] for a in rig.sent if a[1] != 999] == [("/n_set", 1), ("/g_freeAll", 1)]
    assert dict(zip(rig.sent[-1][2::2], rig.sent[-1][3::2]))["trim"] == 0.0, "после стопа trim 0 (PR-7)"
    snap = parse_music_state(rig.states[-1])
    assert snap.state == "idle" and json.loads(rig.states[-1])["dj"] == {"enabled": False}
    assert rig.session.active is False and rig.owner.on_started is None


def test_exec_failure_at_handoff_is_rejected_and_the_old_track_keeps_playing():
    rig = _rig()
    rig.session.start()
    rig.clock.run_until(rig.clock.beat + 2)
    s0, form = _started(rig)[0]["start_beat"], _form(rig)
    rig.ns.pop("sinepad")  # синт исчез из контекста между проверкой и стыком
    with patch.object(rig.adapter, "check", return_value=None):
        rig.clock.run_until(s0 + form + 4)
    rejected = [e for e in rig.events if e["event"] == "rejected"]
    assert [e["reason"] for e in rejected] == ["exec_error"]
    assert len(_started(rig)) == 1 and rig.ns["d1"].playing


def test_queued_artifact_for_another_track_is_dropped_as_stale():
    rig = _rig(submit=lambda fn: None)
    rig.session.start()
    rig.clock.run_until(rig.clock.beat + 2)
    program = SimpleNamespace(track_id="s7:02:B:x", deck="B", bpm=BPM, form_beats=192.0, slots={}, synths=frozenset(),
                              samples=frozenset(), code="")
    rig.owner.enqueue(program, "s7:01:A:чужой")
    assert rig.owner.advance(9999.0) is False
    assert any("artifact_stale" in w for w in rig.log.warnings())
    assert rig.owner.advance(9999.0) is False  # очередь пуста


def test_dj_set_tool_starts_and_stops_a_set():
    rig = _rig()
    tool = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=lambda theme: (), seed=lambda: 123456)
    started = tool.execute(action="start", theme="космос")
    assert started.success and started.data["set_id"] == "set23456" and started.data["ok"] is True
    assert started.data["theme_source"] in ("theme", "pool")
    rig.clock.run_until(rig.clock.beat + 2)
    assert len(_started(rig)) == 1
    with patch("rob_box_mcp_tools.engine.renardo_adapter.time.sleep"):
        stopped = tool.execute(action="stop")
        again = tool.execute(action="stop")
    assert stopped.success and stopped.data["was_playing"] is True
    assert again.success and again.data["was_playing"] is False
    assert tool.execute(action="hotter").success is False
    assert tool.slice == "personality"


def test_library_melodies_take_only_exact_names():
    class Lib:
        def get(self, name):
            records = {"tetris": {"name": "tetris", "rtttl": "t:d=4:c"},
                       "robot": {"name": "robocop", "rtttl": "r:d=4:c"}}  # «лучший по словам» — чужая мелодия
            return records.get(name)

    opened = []
    lookup = library_melodies(lambda: opened.append(1) or Lib())
    assert lookup(["tetris", "robot", "nope"]) == {"tetris": "t:d=4:c"}
    lookup(["tetris"])
    assert opened == [1]  # библиотека открыта один раз


def test_replan_recomposes_next_track_with_the_same_set_history():
    """PR-3d × PR-10: история сета — треки с меньшим номером; повторная компоновка N+1 (``replan``) не видит
    саму себя, а следующий трек видит уже новую версию N+1."""
    import rob_box_mcp_tools.engine.session as session_mod
    from rob_box_mcp_tools.engine.session import plan_source

    seen = []
    real = session_mod.compose

    def spy(plan, no, **kw):
        seen.append((no, [row["kit"] for row in kw["history"]]))
        return real(plan, no, **kw)

    plans = [PLAN]
    source = plan_source(lambda: (plans[0], None))
    with patch.object(session_mod, "compose", spy):
        t1 = source(1, "A")
        t2 = source(2, "B")
        plans[0] = replace(PLAN, seed=PLAN.seed + 1)  # новый план (LLM) — N+1 компонуется заново
        t2b = source(2, "B")
        source(3, "A")
    k1, k2b = t1.history_key.kit, t2b.history_key.kit
    assert seen[:3] == [(1, []), (2, [k1]), (2, [k1])]
    assert seen[3] == (3, [k2b, k1])
    assert t2.history_key.kit != k1 and k2b != k1


def test_progression_cap_holds_across_sets_with_set_memory():
    """A13 (приёмка 02.10): 5 сетов × 6 треков подряд — в любом окне из 10 треков одна прогрессия не чаще 3 раз.

    Без памяти между сетами история обнулялась на каждом сете, и на стыке выходило 5 из 10.
    """
    from rob_box_music.arrange.harmony import PROGRESSION_CAP, PROGRESSION_WINDOW
    from rob_box_mcp_tools.engine.session import SetMemory, plan_source

    themes = ("космос", "ночной город", "лес", "океан", "завод")
    for run in range(3):  # разные сиды — не один удачный расклад
        memory, played = SetMemory(), []
        for i, theme in enumerate(themes):
            seed = 1000 * run + 17 * i + 3
            plan = seeded_plan(ThemeProfile(theme, "club", BPM, (i * 5) % 12, "minor", (), None), seed,
                               set_id=f"set{seed}")
            source = plan_source(lambda plan=plan: (plan, None), memory)
            played += [source(no, "AB"[no % 2]).history_key.progression for no in range(1, 7)]
        assert len(played) == 30
        for start in range(len(played) - PROGRESSION_WINDOW + 1):
            window = played[start:start + PROGRESSION_WINDOW]
            worst = max(window.count(name) for name in set(window))
            assert worst <= PROGRESSION_CAP, (run, start, window)


def test_set_memory_keeps_past_sets_newest_first_and_bounded():
    from rob_box_mcp_tools.engine.session import SetMemory

    memory = SetMemory(depth=3)
    assert memory.begin() == ()
    memory.remember(1, {"n": "a1"})
    memory.remember(2, {"n": "a2"})
    memory.remember(2, {"n": "a2b"})  # повторная компоновка заменяет строку
    assert [r["n"] for r in memory.begin()] == ["a2b", "a1"]
    memory.remember(1, {"n": "b1"})
    memory.remember(2, {"n": "b2"})
    assert [r["n"] for r in memory.begin()] == ["b2", "b1", "a2b"]


def test_set_memory_with_store_survives_restart():
    """I17 (#3399): треки сета уходят в ``music_history`` на старте следующего; новый процесс видит их."""
    from rob_box_mcp_tools.engine.session import SetMemory
    from rob_box_music.diversity import MusicHistory

    store = MusicHistory(":memory:")
    memory = SetMemory(depth=3, store=store)
    memory.begin()
    memory.remember(1, {"melody_name": "terminat", "kit": "k1", "n": "не поле истории"})
    memory.remember(2, {"melody_name": "theme_177", "kit": "k2"})
    memory.begin()
    assert [r["melody_name"] for r in SetMemory(depth=3, store=store).begin()] == ["theme_177", "terminat"]


def _composition_of(line):
    return json.loads(line.split("composition=", 1)[1])


def test_started_line_carries_one_line_composition_json_of_every_axis(tmp_path):
    rig = _rig()
    rig.session._tracks_dir = str(tmp_path)
    assert rig.session.start()["ok"] is True
    rig.clock.run_until(rig.clock.beat + 2)
    lines = [m for _l, m in rig.log.lines if "[set v2]" in m and " started track_id=" in m]
    assert len(lines) == 1 and "\n" not in lines[0]
    comp = _composition_of(lines[0])
    for axis in ("pad", "lead", "bass", "kick", "kit", "form", "bpm", "mode", "root", "hook", "prog", "sample",
                 "perc", "fx", "energy"):
        assert comp[axis] not in (None, ""), (axis, comp)
    assert comp["bpm"] == BPM and comp["kick"] in kn.KICK_SOUNDS
    track_id = lines[0].split("track_id=")[1].split()[0]
    model = json.loads((tmp_path / (track_id.replace(":", "_") + ".json")).read_text(encoding="utf-8"))
    assert model["track_id"] == track_id and model["parts"]["kick"]["sample"] == kn.KICK_SOUNDS[comp["kick"]].sample


def test_track_files_rotate_and_no_dir_means_no_files(tmp_path):
    from rob_box_mcp_tools.engine.session import write_track_file

    track = compose_source(PLAN)(1, "A")
    for n in range(5):
        write_track_file(str(tmp_path), replace(track, track_id=f"s:{n}"), keep=3)
        os.utime(tmp_path / f"s_{n}.json", (n, n))
    assert len(list(tmp_path.glob("*.json"))) == 3
    rig = _rig()  # tracks_dir не задан: строка с composition есть, файлов нет
    rig.session.start()
    rig.clock.run_until(rig.clock.beat + 2)
    assert any("composition={" in m for _l, m in rig.log.lines)
