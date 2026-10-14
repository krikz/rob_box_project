"""ADR-0158 PR-8 (G5) — сет из генерации: кэш сначала, недостающее генерацией, варианты не соседями, мост, кроссфейд.

Без сети и денег: фейковый KIE (каждая задача — свой taskId, 2 варианта), настоящий кэш ``GeneratedMusicLibrary``,
фейковый ffmpeg (wav нужной длины), настоящие ``SetSession`` + ``PlayerOwner`` + ``RenardoAdapter`` на симуляторе клока.
"""

from __future__ import annotations

import itertools
import math
import wave
from types import SimpleNamespace

import pytest

from rob_box_mcp_tools.engine.gen import gen_session as gs
from rob_box_mcp_tools.engine.gen.budget import BudgetPolicy
from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary
from rob_box_mcp_tools.engine.gen.provider import GenStatus, GenTicket, GenTrack, GenUnavailable
from rob_box_mcp_tools.engine.session import NEARLY_LEAD_BEATS, deck_program
from rob_box_mcp_tools.engine.source_route import cache_tracks
from rob_box_mcp_tools.engine.tools_v2 import SOURCE_DECLINED, DjSetTool
from rob_box_music import knowledge as kn
from rob_box_music.choice import DeterministicChooser
from rob_box_music.model import AudioTrack
from rob_box_music.render.audio import render_audio
from rob_box_music.source import SourceAsk, SourceContext, set_cache, source_ask
from rob_box_voice.core.media_plan_run import _started_text

from .test_engine_session import BPM, _rig

pytestmark = pytest.mark.unit

ROCK = SourceAsk(genre="rock")


class Kie:
    """Фейковый KIE: каждая задача — свой taskId, 2 варианта ``seconds`` с; ``fail`` — отказ на заказе."""

    name, configured = "kie_suno", True

    def __init__(self, seconds=150.0, fail=None):
        self.seconds, self.fail, self.jobs = seconds, fail, []
        self._ids = itertools.count(1)

    def cost(self, job):
        return 12.0

    def balance(self):
        return 900.0

    def submit(self, job):
        if self.fail:
            raise self.fail
        self.jobs.append(job)
        return GenTicket(f"T{next(self._ids)}", self.name, job, 0.0, 12.0)

    def poll(self, ticket):
        tracks = tuple(GenTrack(v, f"{ticket.task_id}v{v}", f"https://cdn/{ticket.task_id}/{v}.mp3", "",
                                self.seconds, f"Rock {ticket.task_id}.{v}") for v in range(2))
        return GenStatus("complete", tracks, credits=12.0)

    def download(self, url):
        return f"MP3:{self.seconds}".encode()


def fake_ffmpeg(cmd, **_kw):
    """``ffmpeg -i <mp3> … <tmp>``: wav длиной из «mp3» (``MP3:<секунды>``), 100 Гц — файл маленький."""
    src, out = cmd[cmd.index("-i") + 1], cmd[-1]
    seconds = float(open(src, "rb").read().split(b":")[1])
    with wave.open(out, "wb") as wav:
        wav.setnchannels(2)
        wav.setsampwidth(2)
        wav.setframerate(100)
        wav.writeframes(b"\0\0\0\0" * int(seconds * 100))


def _lib(tmp_path, cached=0, seconds=150.0, style="grunge"):
    """Кэш с ``cached`` вариантами одной задачи (гранж — семья рока)."""
    lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
    for v in range(cached):
        lib.save_track(f"MP3:{seconds}".encode(), prompt="grunge", title=f"Grunge {v}", is_instrumental=True,
                       kind="instrumental", style_key=style, genre=style, task_id="OLD", variant=v,
                       duration_ms=int(seconds * 1000))
    return lib


def _set(lib, kie, tracks, bridge=None, gen_reason="", speak=None, bpm=kn.AUDIO_DEFAULT_BPM):
    cache = set_cache(ROCK, SourceContext(cache=cache_tracks(lib)))
    return gs.GenSet(provider=kie, library=lib, ask=ROCK, genre="rock", theme="", set_id="gen1", bpm=bpm,
                     tracks=tracks, cache=cache, bridge=bridge, gen_reason=gen_reason, speak=speak,
                     spawn=lambda fn: fn(), run=fake_ffmpeg)


def test_pick_item_avoids_variant_of_the_previous_task():
    a0, a1, b0 = gs.SetItem("a0", "A", "a"), gs.SetItem("a1", "A", "a"), gs.SetItem("b0", "B", "b")
    assert gs.pick_item([a0, a1, b0], {"a0"}, "A") == (b0, "ok")
    assert gs.pick_item([a0, a1], {"a0"}, "A") == (a1, "sibling")
    assert gs.pick_item([a0], {"a0"}, "A") == (None, "")


def test_tasks_needed_two_variants_per_task_and_ten_at_most():
    assert gs.tasks_needed(5, 2, 0) == 2  # 3 недостающих → 2 задачи (4 варианта)
    assert gs.tasks_needed(5, 5, 0) == 0
    assert gs.tasks_needed(60, 0, 0) == kn.GEN_SET_MAX_TASKS
    assert gs.tasks_needed(60, 0, 9) == 1
    assert gs.task_model(0, True) == "V6_MINI" and gs.task_model(0, False) == gs.task_model(1, True) == "V6"


def test_render_audio_crossfade_shape():
    track = AudioTrack("gen1:02:B", "/abs/t.wav", 149.6, bpm_nominal=60)
    one = render_audio(track, "B")  # одиночный трек — как PR-2: форма вверх до такта, глушитель повтора
    assert one.form_beats == 152.0 and "var([1, 0]" in one.code
    mid = render_audio(track, "B", xfade_bars=4, once=False)
    assert "sus=148" in mid.code and "atk=16" in mid.code and "rel=16" in mid.code
    # проход короче файла на кроссфейд: следующий проход нарастает, пока этот гаснет (стык петли без провала, 14.10)
    assert mid.form_beats == 148.0 - 16 and mid.tail_beats == 16.0
    # плеер встаёт на долю, кратную сумме dur (Renardo ``Player.count``): dur — такт, иначе стык на форму позже
    # (живой рок-сет 13.10: 296-я доля вместо 224-й, 56 с тишины); звучит первый такт формы, повтор — проход файла
    assert "dur=4," in mid.code and "amp=var([1, 0], [4, 128]" in mid.code
    last = render_audio(track, "B", xfade_bars=4, once=True)
    assert "dur=4," in last.code and "amp=var([1, 0], [4, 292]" in last.code  # последний: один проход
    assert last.form_beats == 148.0 and last.tail_beats == 0.0
    assert "var([1, 0]" in deck_program(track, "A", last=True).code


def test_cache_first_then_generation_variants_never_neighbours_and_tasks_le_missing(tmp_path):
    """5 треков, в кэше 2 варианта одной задачи: 3 недостающих → 2 задачи V6; соседей-вариантов нет."""
    lib, kie = _lib(tmp_path, cached=2), Kie()
    gen = _set(lib, kie, tracks=5)
    played = [gen.next_track(no, "AB"[(no - 1) % 2]) for no in range(1, 6)]
    groups = [gen._items[t.track_id][0].group for t in played]
    assert groups[0] == "OLD" and all(a != b for a, b in zip(groups, groups[1:]))
    assert [j.model for j in kie.jobs] == ["V6", "V6"] and gen.task_ids == ["T1", "T2"]
    assert all(j.style_tags == kn.GEN_STYLES["rock"].tags and j.duration_s == kn.GEN_SET_DURATION_S for j in kie.jobs)
    assert lib.ledger.spent_since("2000-01-01") == 24.0 and gen.spent == 24.0
    assert all(isinstance(t, AudioTrack) and t.bpm_nominal == kn.AUDIO_DEFAULT_BPM for t in played)


def test_empty_cache_bridge_first_mini_then_v6(tmp_path):
    lib, kie = _lib(tmp_path), Kie()
    bridge = SimpleNamespace(track_id="gen1:01:A")
    gen = _set(lib, kie, tracks=4, bridge=lambda deck: bridge)
    assert gen.next_track(1, "A") is bridge
    assert [j.model for j in kie.jobs] == ["V6_MINI"]  # заказан на старте моста
    tracks = [gen.next_track(no, "AB"[no % 2]) for no in (2, 3, 4, 5)]
    assert [j.model for j in kie.jobs] == ["V6_MINI", "V6"]
    groups = [gen._items[t.track_id][0].group for t in tracks]
    assert all(a != b for a, b in zip(groups, groups[1:]))


def test_generation_failure_speaks_reason_and_set_plays_on(tmp_path):
    lib, said = _lib(tmp_path, cached=2), []
    gen = _set(lib, Kie(fail=GenUnavailable("no_credits", "402")), tracks=4, speak=said.append)
    tracks = [gen.next_track(no, "AB"[(no - 1) % 2]) for no in range(1, 5)]
    assert said == ["Дописать рок не вышло: на счёте генерации нет кредитов — доиграю из того, что есть."]
    assert gen.gen_reason == "no_credits" and len(tracks) == 4  # повтор сыгранного, но не тишина
    assert gen._items[tracks[-1].track_id][1] in ("repeat", "sibling", "ok")


def test_g5_simulation_no_gap_on_ten_tracks(tmp_path):
    """G5 по симуляции: генерация с худшей измеренной задержкой (MINI 45 с, V6 53 с, ADR §1.3) готова раньше
    ``nearly_finished`` играющего трека — окон тишины нет; варианты не соседи; задач ≤ недостающих треков."""
    latency = {"V6_MINI": 45.0, "V6": 53.0}
    for cached in (0, 2):
        lib, kie = _lib(tmp_path / str(cached), cached=cached), Kie(seconds=150.0)
        bridge_s = 64 * 4 * 60.0 / BPM  # мост — форма 64 такта синтеза
        gen = _set(lib, kie, tracks=10, bridge=(lambda d: SimpleNamespace(track_id="b")) if not cached else None,
                   bpm=BPM)
        now, ordered_at, groups = 0.0, {}, []
        lead_s = NEARLY_LEAD_BEATS * 60.0 / BPM
        xfade_s = kn.GEN_XFADE_BARS * 4 * 60.0 / BPM
        first = gen.next_track(1, "A")
        end = now + (bridge_s if not cached else first.duration_s)
        ordered_at.update({j: now for j in range(len(kie.jobs))})
        for no in range(2, 11):
            jobs_before = len(kie.jobs)
            track = gen.next_track(no, "AB"[no % 2])  # подготовка N+1 — на started(N)
            ordered_at.update({j: now for j in range(jobs_before, len(kie.jobs))})
            item = gen._items[track.track_id][0]
            groups.append(item.group)
            if item.origin == "generated":
                job = int(item.group[1:]) - 1
                ready = ordered_at[job] + latency[kie.jobs[job].model]
                passes = 0 if no > 2 or cached else max(0, math.ceil((ready - (end - lead_s)) / bridge_s))
                end += passes * bridge_s  # мост продлевается проходом формы, пока первый трек пишется
                assert ready <= end - lead_s, f"трек {no} не готов к nearly_finished: {ready:.0f} > {end - lead_s:.0f}"
            now = end - xfade_s  # трек N+1 встал за кроссфейд до конца формы N
            end = now + track.duration_s
        assert all(a != b for a, b in zip(groups, groups[1:]))
        assert len(kie.jobs) <= 10 - min(cached, 10) and len(kie.jobs) == math.ceil((10 - cached) / 2)


def test_set_session_crossfades_files_on_two_decks(tmp_path):
    """Настоящий ``SetSession`` на симуляторе клока: файлы кэша → кроссфейд ``GEN_XFADE_BARS`` тактов, деки A/B."""
    lib = _lib(tmp_path, cached=2, seconds=60.0)
    for v in range(2):
        lib.save_track(b"MP3:60.0", prompt="rock", title=f"Rock {v}", is_instrumental=True, kind="instrumental",
                       style_key="rock", genre="rock", task_id="NEW", variant=v, duration_ms=60000)
    gen = _set(lib, Kie(), tracks=3, gen_reason="no_credits", bpm=BPM)
    rig = _rig(source=gen.next_track, tracks=3)
    assert rig.session.start()["ok"]
    rig.clock.run_until(rig.clock.beat + 4000)
    started = [e for e in rig.events if e["event"] == "started"]
    assert [e["deck"] for e in started] == ["A", "B", "A"] and not [e for e in rig.events if e["event"] == "rejected"]
    form = started[0]["form_beats"]
    # проход файла короче файла на кроссфейд: следующий трек встаёт на границу прохода, хвост уходящего — кроссфейд
    assert started[1]["start_beat"] == started[0]["start_beat"] + form
    assert any(e["event"] == "finished" for e in rig.events)
    # Renardo ставит плеер на долю, кратную сумме dur: каждая доля стыка кратна dur своей программы
    assert all(e["start_beat"] % 4 == 0 for e in started[1:])
    assert "dur=4," in deck_program(AudioTrack("x", "/abs/x.wav", 60.0, bpm_nominal=BPM), "B", last=False).code


def test_song_bridge_takes_theme_without_llm(tmp_path):
    """Мост ожидания генерации берётся кодом: поиск темы без theme-links LLM (13.10: мост песни ждал LLM 8 с,
    ``request_music`` 10.6 с при таймауте хода 10 с)."""
    from rob_box_mcp_tools.engine.tools_v2 import bridge_find

    dj = _dj(_lib(tmp_path), Kie())
    assert bridge_find(dj, "pending") is dj.quick_find and bridge_find(dj, "") is None


def test_bridge_line_llm_colours_with_placeholders_or_code_line():
    facts = gs.bridge_facts("grunge", "V6_MINI")
    assert facts == {"next": "гранж", "bridge": "брейкбит", "eta": "30"}
    assert gs.bridge_line(facts, None) == "Пока я дописываю гранж — вот вам брейкбит, секунд 30."

    def ask(line, outcome="ok"):
        call = SimpleNamespace(name=gs.SUBMIT_BRIDGE, arguments={"line": line})
        return lambda *a: (outcome, SimpleNamespace(tool_calls=[call]), "")

    good = "Дописываю {next}, а пока качаем {bridge} — ещё {eta} секунд!"
    assert gs.bridge_line(facts, ask(good)) == "Дописываю гранж, а пока качаем брейкбит — ещё 30 секунд!"
    for bad in ("Дописываю {next} за 5 секунд, {bridge} {eta}", "Дописываю {next} и {bridge}", "{next} {bridge} {eta} {x}"):
        assert gs.bridge_line(facts, ask(bad)) == gs.bridge_template(facts)  # соврала числом/пропустила факт
    assert gs.bridge_line(facts, ask(good, "late")) == gs.bridge_template(facts)


def test_launch_texts_name_cache_and_reason():
    assert gs.launch_text("rock", 5, 5, "") == "Рок-сет из сочинённого раньше: 5 треков."
    assert gs.launch_text("rock", 5, 2, "") == "Рок-сет на 5 треков: 2 из сочинённого раньше, остальные допишу по ходу."
    assert gs.launch_text("grunge", 2, 2, "budget_exhausted").startswith("Гранж-сет из сочинённого раньше: 2 трека")
    assert gs.launch_text("rock", 3, 1, "no_credits").endswith("дописать не могу: на счёте генерации нет кредитов.")


class Owner:
    """Владелец плеера без клока: ``play`` принимает, ``started`` не шлёт (сет заиграл — по ``ok``)."""

    def __init__(self):
        self.played, self.on_started, self.on_finished = [], None, None

    def play(self, program, dj=None, once=False):
        self.played.append(program)
        return {"ok": True, "track_id": program.track_id}

    def stop(self, reason="user_stop"):
        return {"ok": True, "track_id": None}

    def reject(self, track_id, reason, detail):
        return {"ok": False, "track_id": track_id, "reason": reason, "detail": detail}


def _dj(lib, kie):
    dj = DjSetTool(None, Owner(), melodies=lambda ids: {}, lines=False, scores=SimpleNamespace(
        warm=lambda: None, titles=lambda ids: {}, state="test", search=lambda q: ()))
    dj.gen_sets = gs.GenSetMaker(kie, lambda: lib, DeterministicChooser())
    return dj


def test_dj_set_rock_goes_through_source_path_and_plays_cache(tmp_path, monkeypatch):
    """«включи рок-сет на 3 трека»: synth_fit рока 0 → путь сета (кэш/генерация), трек 1 — файл кэша, фраза кода."""
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track", _fake_audio_track)
    lib, kie = _lib(tmp_path, cached=2), Kie()
    dj = _dj(lib, kie)
    out = dj.execute(action="start", style="rock", tracks=3, heard_text="включи рок-сет на 3 трека")
    assert out.success, out.error
    assert out.data["source"] == "generated" and out.data["route"].startswith(("cache:", "generate:"))
    assert out.data["source_message"] == "Рок-сет на 3 трека: 2 из сочинённого раньше, остальные допишу по ходу."
    assert "loop(" in dj._owner.played[0].code and dj.running
    plan = SimpleNamespace(tool_calls=(SimpleNamespace(name="dj_set", arguments={"action": "start"}),),
                           say_ok="Включаю диджей-сет.")
    assert _started_text(plan, {**out.data, "theme": "рок"}) == out.data["source_message"]  # фраза кода, не шаблон


def test_dj_set_club_stays_synth_and_rock_without_key_or_cache_declines(tmp_path):
    lib = _lib(tmp_path)
    dj = _dj(lib, Kie())
    assert dj.gen_sets.route("", "club", "включи клубный сет") is None  # «у нас норм играет» — синтез, как раньше
    kie = Kie()
    kie.configured = False
    dj = _dj(lib, kie)
    out = dj.execute(action="start", style="rock", heard_text="включи рок-сет")
    assert not out.success and out.data["reason"] == SOURCE_DECLINED == gs.DECLINED
    assert out.data["message"].startswith("Рок синтезом у меня не выходит") and not dj.running


def test_set_left_comes_from_budget_policy(tmp_path):
    maker = gs.GenSetMaker(Kie(), lambda: _lib(tmp_path), DeterministicChooser())
    ctx = maker.router.context(ROCK, for_set=True)
    assert ctx.set_left == BudgetPolicy().set_credits and ctx.gen_ready
    assert maker.router.context(ROCK).set_left is None  # вне сета лимита сета нет


def _fake_audio_track(info, track_id, run=None):
    return AudioTrack(track_id=track_id, path=str(info["path"]).replace("track.mp3", "t.wav"), duration_s=150.0)


def test_source_ask_genre_for_grunge_set():
    assert source_ask("включи гранж").genre == "grunge" and source_ask("Робот, включи рок-сет").genre == "rock"


def _hold(tmp_path, monkeypatch, wait_s=600.0):
    """Мост ожидания на симуляторе клока: мост — файл кэша 30 с на деке A (форма повторяется), результат — 60 с."""
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track",
                        lambda info, track_id, run=None: AudioTrack(track_id, str(tmp_path / "t.wav"), 60.0))
    lib = _lib(tmp_path, cached=1, seconds=60.0)
    rig = _rig(tracks=1)
    model = AudioTrack("bridge1", str(tmp_path / "b.wav"), 30.0, bpm_nominal=BPM)
    program = render_audio(model, "A", xfade_bars=kn.GEN_XFADE_BARS, once=False)
    hold = gs.BridgeHold(rig.owner, model, program, lambda: lib, facts={"title": "мост"}, wait_s=wait_s)
    assert hold.start()["ok"]
    rig.clock.run_until(rig.clock.beat + 2)
    return rig, hold, lib


def _queued_program(rig, timeout=5.0):
    import time

    end = time.monotonic() + timeout
    while rig.owner._queued is None and time.monotonic() < end:  # noqa: SLF001
        time.sleep(0.01)
    return rig.owner._queued[0]  # noqa: SLF001


def test_bridge_hold_keeps_sound_until_result_then_crossfades(tmp_path, monkeypatch):
    """Правило координатора 13.10 (кавер встал через 64 с после 22 с моста — 44 с тишины): мост повторяет форму,
    пока генерация не вернулась; результат входит кроссфейдом ``GEN_XFADE_BARS`` на другой деке."""
    from rob_box_mcp_tools.engine.song_order import held_play

    rig, hold, lib = _hold(tmp_path, monkeypatch)
    said = []
    hold.on_live = lambda: said.append("live")
    bridge = [e for e in rig.events if e["event"] == "started"][0]
    rig.clock.run_until(bridge["start_beat"] + 4 * bridge["form_beats"])  # ждём генерацию 4 прохода моста
    assert rig.owner.now_playing()["track_id"] == "bridge1" and not [e for e in rig.events if e["event"] == "finished"]
    track_id = cache_tracks(lib)[0].id
    assert held_play(hold)(track_id).success  # песня/кавер готовы — тот же шов у SongOrder и PieceCover
    ready = rig.clock.beat
    _wait_pending(rig)
    rig.clock.run_until(rig.clock.beat + 2 * bridge["form_beats"])
    started = [e for e in rig.events if e["event"] == "started"]
    assert started[-1]["track_id"] == hold.live_id and started[-1]["deck"] == "B" and said == ["live"]
    # вход на ближайшем такте (мост-файл гаснет огибающей деки), а не на конце прохода моста (14.10: 52 с ожидания)
    assert started[-1]["start_beat"] - ready <= 4 + gs.CUT_IN_CUE_BEATS


def _wait_pending(rig, timeout=5.0):
    import time

    end = time.monotonic() + timeout
    while rig.owner._pending is None and time.monotonic() < end:  # noqa: SLF001
        time.sleep(0.01)
    assert rig.owner._pending is not None  # noqa: SLF001


def test_bridge_hold_failure_and_timeout_end_with_a_bridge_finale(tmp_path, monkeypatch):
    rig, hold, _ = _hold(tmp_path, monkeypatch)
    assert hold.fail()
    finale = _queued_program(rig)
    assert finale.deck == "B" and finale.track_id.endswith(":02:B") and "var([1, 0]" in finale.code
    rig, hold, _ = _hold(tmp_path / "t", monkeypatch, wait_s=0.01)  # результата нет за срок — тоже финал
    assert _queued_program(rig).track_id.endswith(":02:B")


def test_bridge_hold_result_after_another_order_stays_in_cache(tmp_path, monkeypatch):
    rig, hold, lib = _hold(tmp_path, monkeypatch)
    rig.owner.stop("user_stop")  # человек остановил или заказал другое
    assert not hold.deliver(cache_tracks(lib)[0].id) and not hold.session.active
