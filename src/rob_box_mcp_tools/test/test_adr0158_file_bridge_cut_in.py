"""Хвосты ADR-0158 после #3631: мост-файл (трек кэша) — результат врезается на ближайшем такте, стык петли без провала.

Живьём 14.10: при мосте-файле 60 с песня ждала 52 с (вход только на конце прохода файла: нота ``loop`` звучит всю
``sus`` и после снятия плеера), а стык петли файла с собой проседал до −35.6 dB на ~2 с (спад к границе, потом подъём).

Настоящие ``BridgeHold`` + ``SetSession`` + ``PlayerOwner`` + ``RenardoAdapter`` на симуляторе клока (``_rig``); поверх —
модель scsynth для нот ``loop``: какие ноты сыграл каждый плеер (``dur``/``amp=var``/стоп), их огибающая (края — четверть
синуса, патч ``loop.scd``), огибающая деки по ``/n_set 1 fadeT … fadeA 0``. Громкость — сумма мощностей нот (материал
разных проходов не коррелирован), окна по 0.5 с.
"""

from __future__ import annotations

import math
import time

import pytest

from rob_box_mcp_tools.engine.gen import gen_session as gs
from rob_box_mcp_tools.engine.session import NEARLY_LEAD_BEATS
from rob_box_mcp_tools.engine.source_route import cache_tracks
from rob_box_music import knowledge as kn
from rob_box_music.model import AudioTrack
from rob_box_music.render.audio import DECK_FADE, render_audio
from rob_box_voice.core.renardo_synthdef_patches import LOOP_SYNTHDEF

from .test_adr0158_pr8_gen_set import _lib
from .test_engine_session import Player, _rig

pytestmark = pytest.mark.unit

BAR = 4.0
XFADE = kn.GEN_XFADE_BARS * BAR
BPM = kn.AUDIO_DEFAULT_BPM  # трек кэша без номинала: доля — секунда (живой мост 14.10)
WINDOW_S = 0.5


# ── модель scsynth для нот ``loop`` ──

class Var:
    def __init__(self, values, durs, start=0.0):
        self.values, self.durs, self.start = list(values), list(durs), float(start)

    def at(self, beat):
        pos = (beat - self.start) % sum(self.durs)
        for value, dur in zip(self.values, self.durs):
            if pos < dur:
                return value
            pos -= dur
        return self.values[-1]


class RecPlayer(Player):
    """Плеер-симулятор, который помнит, что ему дали: ``(доля старта, kwargs loop)``, и где его сняли."""

    def __init__(self, clock, name):
        super().__init__(clock, name)
        self.programs = []

    def __rshift__(self, other):
        super().__rshift__(other)
        self.programs.append((self.event_index, other))
        return self

    def notes(self):
        """Ноты, которые плеер отправил бы в scsynth: доля, kwargs. Нота с amp 0 не уходит (Renardo)."""
        out = []
        for i, (start, kw) in enumerate(self.programs):
            ends = [s for s in self.stopped_at if s >= start] + [p for p, _ in self.programs[i + 1:]]
            end = min(ends) if ends else math.inf
            beat = start
            while beat < end and beat < start + 4000:
                if kw["amp"].at(beat):
                    out.append((beat, kw))
                beat += kw["dur"]
        return out


def _scsynth_rig(tracks=2):
    rig = _rig(tracks=tracks)
    rig.ns["loop"] = lambda path, **kw: dict(kw, path=path)
    rig.ns["var"] = Var
    for deck in kn.DECK_SLOTS.values():
        for slot in deck:
            rig.ns[slot] = RecPlayer(rig.clock, slot)
    sent = rig.adapter._send_osc  # noqa: SLF001
    rig.fades = []
    rig.adapter._send_osc = lambda *a: (rig.fades.append((rig.clock.beat, a)) if "fadeT" in a else None,  # noqa: SLF001
                                        sent(*a))
    return rig


def _welch_up(x):
    return math.sin(math.pi / 2 * min(1.0, max(0.0, x)))


def _note_gain(beat, start, kw, fades, lat):
    """Амплитуда ноты ``loop`` на слышимой доле ``beat``: края ``atk``/``rel`` (с) внутри ``sus`` (доли) — четверть
    синуса; огибающая деки — с первой ``/n_set fade<дека> 0`` после слышимого старта ноты."""
    t0 = start + lat  # нота звучит на ``Clock.latency`` позже своей доли
    u = beat - t0
    sus = kw["sus"]
    if u < 0 or u >= sus:
        return 0.0
    atk, rel = kw["atk"] * BPM / 60.0, kw["rel"] * BPM / 60.0
    g = _welch_up(u / atk) if u < atk else 1.0
    if u > sus - rel:
        g *= _welch_up((sus - u) / rel)
    names = {v: k for k, v in DECK_FADE.items()}
    for sent_at, args in fades:  # /n_set без метки времени — сразу на доле отправки
        controls = dict(zip(args[2::2], args[3::2]))
        if sent_at >= t0 and controls.get(f"fade{names.get(kw.get('fdeck'), '?')}") == 0.0:
            g *= _welch_up(1.0 - (beat - sent_at) / (controls["fadeT"] * BPM / 60.0)) if beat >= sent_at else 1.0
            break
    return g


def _power_db(rig, beat0, beat1):
    """Окна по ``WINDOW_S``: средняя сумма мощностей нот всех дек, дБ (1.0 — 0 dB)."""
    lat = rig.clock.latency * BPM / 60.0
    notes = [(b, kw) for p in rig.ns.values() if isinstance(p, RecPlayer) for b, kw in p.notes()]
    step = WINDOW_S * BPM / 60.0
    out = []
    w = beat0
    while w + step <= beat1:
        samples = [w + step * k / 20 for k in range(20)]
        power = sum(sum(_note_gain(t, b, kw, rig.fades, lat) ** 2 for b, kw in notes) for t in samples) / 20
        out.append((w, 10 * math.log10(max(power, 1e-12))))
        w += step
    return out


def _wait(cond, timeout=5.0):
    end = time.monotonic() + timeout
    while not cond() and time.monotonic() < end:
        time.sleep(0.01)
    assert cond(), "фон моста не дошёл до шага"


def _started(rig):
    return [e for e in rig.events if e["event"] == "started"]


# ── синт: края — четверть синуса, огибающая деки ──

def test_loop_synthdef_has_equal_power_edges_and_a_deck_fade():
    assert "Env([0, 1, 1, 0], [edgeA, sus - edgeA - edgeR, edgeR], \\wel)" in LOOP_SYNTHDEF
    assert "Select.kr(fdeck, [1, fadeA, fadeB])" in LOOP_SYNTHDEF and "timeScale: fadeT" in LOOP_SYNTHDEF
    for deck in DECK_FADE:
        assert f"fade{deck}=1" in LOOP_SYNTHDEF  # дефолт: нота звучит, пока её деку не погасили
    assert render_audio(AudioTrack("x", "/abs/x.wav", 150.0), "B").code.rstrip().endswith("fdeck=2)")


# ── стык петли моста-файла с собой ──

def test_file_loop_seam_has_no_dip():
    """Мост-файл 150 с петлёй 3 прохода: следующий проход нарастает, пока гаснет этот; окна 0.5 с — без провала
    (было −35.6 dB на ~2 с) и без двойной громкости."""
    rig = _scsynth_rig(tracks=1)
    program = render_audio(AudioTrack("bridge", "/abs/b.wav", 150.0, bpm_nominal=BPM), "A",
                           xfade_bars=kn.GEN_XFADE_BARS, once=False)
    assert program.tail_beats == XFADE and program.form_beats == 148.0 - XFADE  # проход короче файла на кроссфейд
    assert rig.owner.play(program)["ok"]
    rig.clock.run_until(rig.clock.beat + 3.5 * program.form_beats)
    start = _started(rig)[0]["start_beat"]
    seams = [start + k * program.form_beats for k in (1, 2, 3)]
    windows = _power_db(rig, start + XFADE + 1, seams[-1] + XFADE + 4)
    worst = min(db for _, db in windows)
    loudest = max(db for _, db in windows)
    assert worst > -1.0, f"провал на стыке: {worst:.1f} dB"
    assert loudest < 1.0, f"двойная громкость на стыке: {loudest:.1f} dB"
    notes = [b for b, _ in rig.ns[program.slots["audio"]].notes()]
    assert notes[:4] == [start] + seams  # нота на каждый проход, кратно такту


def test_set_track_after_a_looping_file_starts_on_the_pass_boundary():
    """Следующий трек сета после файла петлёй: встык на границе прохода — хвост уходящего и есть кроссфейд
    (вход за кроссфейд до границы дал бы полный уходящий поверх входящего)."""
    rig = _scsynth_rig(tracks=1)
    bridge = AudioTrack("bridge1", "/abs/b.wav", 150.0, bpm_nominal=BPM)
    program = render_audio(bridge, "A", xfade_bars=kn.GEN_XFADE_BARS, once=False)
    hold = gs.BridgeHold(rig.owner, bridge, program, lambda: None, wait_s=0.01)  # результата нет — финал моста
    assert hold.start()["ok"]
    rig.clock.run_until(rig.clock.beat + 2)
    _wait(lambda: rig.owner._queued is not None)  # noqa: SLF001 — финал собран в фоне
    rig.clock.run_until(rig.clock.beat + 2 * program.form_beats + 8)
    started = _started(rig)
    assert [e["deck"] for e in started] == ["A", "B"]
    assert started[1]["start_beat"] == started[0]["start_beat"] + program.form_beats  # граница прохода
    windows = _power_db(rig, started[0]["start_beat"] + XFADE + 1, started[1]["start_beat"] + XFADE + 4)
    assert min(db for _, db in windows) > -1.0 and max(db for _, db in windows) < 1.0


# ── вход результата посреди прохода моста-файла ──

def _file_hold(tmp_path, monkeypatch, bridge_s=150.0, result_s=150.0):
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track",
                        lambda info, track_id, run=None: AudioTrack(track_id, str(tmp_path / "song.wav"), result_s))
    lib = _lib(tmp_path, cached=1, seconds=result_s)
    rig = _scsynth_rig()
    bridge = AudioTrack("bridge1", str(tmp_path / "b.wav"), bridge_s, bpm_nominal=BPM)
    program = render_audio(bridge, "A", xfade_bars=kn.GEN_XFADE_BARS, once=False)
    hold = gs.BridgeHold(rig.owner, bridge, program, lambda: lib, facts={"title": "мост"}, logger=rig.log)
    assert hold.start()["ok"]
    rig.clock.run_until(rig.clock.beat + 2)
    return rig, hold, lib, _started(rig)[0]


@pytest.mark.parametrize("into", [0.5, 1.9])  # середина первого прохода; второй проход, nearly_finished уже был
def test_result_over_a_file_bridge_cuts_in_on_the_next_bar_with_a_crossfade(tmp_path, monkeypatch, into):
    rig, hold, lib, bridge = _file_hold(tmp_path, monkeypatch)
    ready = bridge["start_beat"] + into * bridge["form_beats"]
    rig.clock.run_until(ready)
    assert hold.deliver(cache_tracks(lib)[0].id)
    _wait(lambda: rig.owner._pending is not None)  # noqa: SLF001 — cue поставлен из фона
    rig.clock.run_until(ready + 3 * XFADE)
    live = _started(rig)[-1]
    assert live["track_id"] == hold.live_id and live["deck"] == "B"
    at = live["start_beat"]
    assert at % BAR == 0 and at - ready <= BAR + gs.CUT_IN_CUE_BEATS  # ≤ 1 такт + cue, а не конец прохода файла
    # мост-файл гаснет огибающей своей деки с доли входа (как она слышна: + latency), за кроссфейд
    lat = rig.clock.latency * BPM / 60.0
    assert rig.fades == [(at + lat, ("/n_set", 1, "fadeT", XFADE * 60.0 / BPM, "fadeA", 0.0))]
    slot = kn.DECK_SLOTS["A"][8]
    assert rig.ns[slot].stopped_at and min(s for s in rig.ns[slot].stopped_at if s > ready) <= at + XFADE
    assert all(b < at for b, _ in rig.ns[slot].notes())  # новых проходов моста после входа нет
    windows = _power_db(rig, ready - 8, at + XFADE + 8)
    assert min(db for _, db in windows) > -1.0 and max(db for _, db in windows) < 1.0  # кроссфейд без провала
    after = [db for w, db in _power_db(rig, at + XFADE + lat + 1, at + XFADE + 20)]
    assert after and max(after) < 0.1  # дальше звучит только результат: моста нет
    assert len([e for e in rig.events if e["event"] == "queued"]) == 1  # nearly_finished не готовил второй раз
    assert not [m for m in rig.log.warnings() if "не готов" in m] or into > 1  # мост не продлён после поставки


def test_result_ready_inside_the_last_lead_still_cuts_in_on_the_next_bar(tmp_path, monkeypatch):
    """Готово за ≤ ``lead`` долей до границы прохода: вход тоже на ближайшем такте; ``nearly_finished`` моста между
    постановкой и входом видит поставленный результат и мост не продлевает."""
    rig, hold, lib, bridge = _file_hold(tmp_path, monkeypatch)
    end = bridge["start_beat"] + bridge["form_beats"]
    ready = end - NEARLY_LEAD_BEATS - 3.0  # nearly_finished — между постановкой и входом
    rig.clock.run_until(ready)
    assert hold.deliver(cache_tracks(lib)[0].id)
    _wait(lambda: rig.owner._pending is not None)  # noqa: SLF001
    rig.clock.run_until(end + 3 * XFADE)
    live = _started(rig)[-1]
    assert live["track_id"] == hold.live_id and live["start_beat"] - ready <= BAR + gs.CUT_IN_CUE_BEATS
    assert live["start_beat"] > end - NEARLY_LEAD_BEATS  # nearly_finished моста пришёлся на ожидание входа
    assert not [m for m in rig.log.warnings() if "не готов" in m]
    assert len([e for e in rig.events if e["event"] == "queued"]) == 1


def test_cut_in_on_a_pass_boundary_needs_no_deck_fade():
    form, start = 132.0, 100.0
    assert gs.cut_in_leave((150.0, start, form), 160.0, file=True) == (160.0 + XFADE, XFADE)
    assert gs.cut_in_leave((228.0, start, form), 232.0, file=True) == (232.0, 0.0)  # граница: гаснет хвостом ноты
    assert gs.cut_in_leave((220.0, start, form), 224.0, file=True) == (232.0, XFADE)  # плеер — до нового прохода
    assert gs.cut_in_leave((150.0, start, form), 160.0, file=False) == (160.0 + XFADE, 0.0)  # синтез: без огибающей
