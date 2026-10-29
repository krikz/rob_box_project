"""#4006 (+ дубль #4008): в сете из генерации диджей говорит на каждом переходе, как в сете синтезом.

Регрессия ADR-0158 PR-8 (14.10): ``GenSetMaker`` строил ``SetSession`` без ``TransitionLines`` — после фразы запуска
диджей молчал до конца сета (марафон 23.10, акт 6 n605: ложный PASS по фразе запуска). Здесь — поведение: сет из трёх
треков даёт две реплики перехода (трек 1 озвучивает фраза запуска, мост — своя фраза), в реплике факты трека (номер,
длина, произведение кавера), название-метка инструментала (одно на все треки темы) связкой не повторяется,
``music_v2_dj_lines: false`` глушит реплики, но не факты. Без сети: фейковый KIE, настоящий кэш, симулятор клока.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from rob_box_llm.provider import LLMResponse, ToolCall
from rob_box_mcp_tools.engine.dj_lines import TransitionLines, latin_fold
from rob_box_mcp_tools.engine.gen import gen_session as gs
from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary
from rob_box_mcp_tools.engine.source_route import cache_tracks
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool
from rob_box_music import dj_line as dl
from rob_box_music import knowledge as kn
from rob_box_music.choice import DeterministicChooser
from rob_box_music.source import SourceAsk, cover_slot, theme_title

from .test_adr0158_pr8_gen_set import Kie, _fake_audio_track, fake_ffmpeg
from .test_adr0158_v14_set_cover import A, Maker
from .test_engine_reasoner import _wait
from .test_engine_session import Log, _rig, _started

pytestmark = pytest.mark.unit

THEME = "неоновая ночь"
LABEL = theme_title(THEME)  # «Неоновая ночь» — название каждого инструментала, заказанного этой темой (``GenSet._job``)
SYNTH = SourceAsk(genre="synthwave")
REQUEST = "Робот, поставь диджей-сет из трёх треков в стиле синтвейв"
SCORES = SimpleNamespace(warm=lambda: None, titles=lambda ids: {}, state="test", search=lambda q: ())


def _lib(tmp_path, cover=False):
    """Кэш: три инструментала синтвейва из разных задач, все названы темой заказа; ``cover`` — ещё кавер STAR WARS."""
    lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
    for task in ("A", "B", "C"):
        lib.save_track(b"MP3:150.0", prompt="synthwave", title=LABEL, is_instrumental=True, kind="instrumental",
                       style_key="synthwave", genre="synthwave", task_id=task, variant=0, duration_ms=150000)
    if cover:
        lib.save_track(b"MP3:150.0", prompt="cover", title="STAR WARS (RoBBox cover)", is_instrumental=True,
                       kind="cover", style_key="synthwave", task_id="CV", variant=0, duration_ms=150000,
                       work_id="STAR WARS")
    return lib


def _lines(said, log, *, speak=True, ask=None, published=None):
    """``TransitionLines`` как у ``DjSetTool._transition_lines``, но синхронно: фон реплики — сразу."""
    publish = (lambda t, f: published.append((t, dict(f)))) if published is not None else None
    return TransitionLines(said.append if speak else None, titles=lambda ids: {}, ask=ask, fold=latin_fold,
                           logger=log, spawn=lambda fn: fn(), grace_s=0.01, publish=publish)


def _gen(lib, lines, *, keys=(), bridge=None, kie=None, published=None, gen_reason="no_credits"):
    publish = (lambda t, f: published.append((t, dict(f)))) if published is not None else None
    return gs.GenSet(provider=kie or Kie(), library=lib, ask=SYNTH, genre="synthwave", theme=THEME, set_id="gen40",
                     bpm=kn.AUDIO_DEFAULT_BPM, tracks=3, cache=cache_tracks(lib), bridge=bridge,
                     gen_reason=gen_reason, spawn=lambda fn: fn(), run=fake_ffmpeg, keys=keys, lines=lines,
                     publish=publish)


def _play(gen, first=1, last=3):
    """Треки ``first..last`` сета по очереди, как ``SetSession``: подготовка трека → его ``started``; пометки строк."""
    notes = []
    for no in range(first, last + 1):
        track = gen.next_track(no, "AB"[(no - 1) % 2])
        notes.append(gen.on_started(track.track_id))
    return notes


def _spoken(log):
    return [m for _l, m in log.lines if "[dj line]" in m and "text=" in m]


def _templates(no, total, hook=None):
    """Все варианты шаблона кода для фактов трека сета из генерации (фаза — по месту трека, энергий нет)."""
    facts = dl.facts_for(no, total, THEME, hook, ())
    return {dl.template_line(facts, seed) for seed in range(12)}


def test_gen_set_of_three_tracks_says_one_line_on_each_transition_and_none_on_track_one(tmp_path):
    said, log = [], Log()
    gen = _gen(_lib(tmp_path), _lines(said, log))
    notes = _play(gen)
    # трек 1 озвучивает фраза запуска; переходы 1→2 и 2→3 — ровно по одной реплике (I23)
    assert len(said) == 2
    assert notes[0].endswith("dj_line=start_phrase") and all(n.endswith("dj_line=on") for n in notes[1:])
    assert said[0] in _templates(2, 3) and said[1] in _templates(3, 3)  # номер и длина сета — факты кода
    spoken = _spoken(log)
    assert "трек 2/3 phase=steady source=template hook=None" in spoken[0]
    assert "трек 3/3 phase=final source=template hook=None" in spoken[1]


def test_same_theme_title_on_every_track_is_not_repeated_as_the_link(tmp_path):
    """Все инструменталы темы названы одинаково (``theme_title``): название — метка заказа, не мелодия; в реплике
    его нет, и одна и та же связка не звучит на каждом переходе."""
    said, log = [], Log()
    _play(_gen(_lib(tmp_path), _lines(said, log)))
    assert said and not [s for s in said if LABEL.lower() in s.lower()]
    assert said[0] != said[1]


def test_cover_track_line_names_its_piece_instrumentals_do_not(tmp_path):
    """Трек 2 — кавер из кэша: в реплике произведение (``{hook}``); трек 3 — инструментал темы: без названия."""
    said, log = [], Log()
    lines = _lines(said, log)
    gen = _gen(_lib(tmp_path, cover=True), lines, keys=["synthwave", cover_slot("synthwave", "STAR WARS"), "synthwave"])
    _play(gen)
    assert len(said) == 2 and said[0] in _templates(2, 3, "STAR WARS") and "STAR WARS" in said[0]
    assert "STAR WARS" not in said[1] and said[1] in _templates(3, 3)
    assert "hook='STAR WARS'" in _spoken(log)[0]
    # факты играющего трека — и в ``status`` (``now``): номер без моста, длина сета, сыгранный кавер
    assert lines.now()["track_no"] == 3 and lines.now()["tracks"] == 3
    assert lines.now()["played"] == [[2, "STAR WARS"]]


def test_llm_colours_the_line_only_with_placeholders_of_the_track_facts(tmp_path):
    asked = []

    def ask(system, user, tool, deadline_s):
        asked.append(user)
        call = ToolCall("c1", dl.SUBMIT_TOOL, {"line": "Погнали, {hook}! Трек {no} из {total}."})
        return "ok", LLMResponse(tool_calls=(call,), finish_reason="tool_calls"), ""

    said, log = [], Log()
    gen = _gen(_lib(tmp_path, cover=True), _lines(said, log, ask=ask),
               keys=["synthwave", cover_slot("synthwave", "STAR WARS"), "synthwave"])
    _play(gen)
    assert said[0] == "Погнали, STAR WARS! Трек 2 из 3."
    assert "{hook} = «STAR WARS»" in asked[0] and "{total} = 3" in asked[0]
    # трек 3 без мелодии: {hook} в ответе LLM — подстановка без факта → шаблон кода с теми же фактами
    assert said[1] in _templates(3, 3) and "source=template" in _spoken(log)[1]


def test_bridge_is_not_a_set_track_and_gets_no_line(tmp_path):
    """Мост (кэш не закрыл трек 1): фразу ожидания говорит запуск; первый файл — «трек 1», реплики на нём нет."""
    said, log, published = [], Log(), []
    lib = GeneratedMusicLibrary(str(tmp_path / "empty"))
    bridge = SimpleNamespace(track_id="gen40:01:A")
    gen = _gen(lib, _lines(said, log), bridge=lambda deck: bridge, kie=Kie(), gen_reason="", published=published)
    assert gen.next_track(1, "A") is bridge and gen.on_started("gen40:01:A") == "gen=bridge" and not said
    notes = _play(gen, 2, 4)
    assert len(said) == 2 and notes[0].endswith("dj_line=start_phrase")
    assert ["трек 2/3" in s for s in _spoken(log)] == [True, False] and "трек 3/3" in _spoken(log)[1]
    # снимок: номер и длина без моста — та же нумерация, что в реплике
    facts = [f for t, f in published if t.startswith("gen40:03:")]
    assert facts and all(f["track_no"] == 2 and f["tracks"] == 3 for f in facts)
    assert any(f.get("title") == f"{THEME} · трек 2 из 3" for f in facts)


def test_dj_lines_off_by_parameter_keeps_facts(tmp_path):
    """``music_v2_dj_lines: false`` — переходы молчат и в сете из генерации; факты трека всё равно в снимке."""
    said, log, published = [], Log(), []
    gen = _gen(_lib(tmp_path), _lines(said, log, speak=False, published=published))
    notes = _play(gen)
    assert said == [] and not [n for n in notes if "dj_line" in n]
    assert {(f["track_no"], f["tracks"]) for _t, f in published if "melody" in f} == {(1, 3), (2, 3), (3, 3)}


def test_cached_cover_and_new_cover_items_carry_the_piece():
    cached = SimpleNamespace(id="c1", group="CV", title="STAR WARS (RoBBox cover)", kind="cover",
                             work_id="STAR WARS")
    plain = SimpleNamespace(id="i1", group="A", title=LABEL, kind="instrumental", work_id="")
    assert gs.cache_item(cached, "x").hook == "STAR WARS" and gs.cache_item(plain, "synthwave").hook == ""
    maker, key = Maker(), cover_slot("club", A.title)
    covers = gs.SetCovers({key: A}, lambda work, ask, spent: maker.make(None, work, ask, spent))
    items, _task, _credits = covers.task(key, SourceAsk(genre="club"), 0.0)
    assert [(i.title, i.hook) for i in items] == [("Звёздные войны (кавер)", "Звёздные войны")] * 2


def test_dj_set_from_cache_speaks_on_every_transition_through_robot_voice(tmp_path, monkeypatch):
    """Целиком через ``dj_set``: «поставь диджей-сет из трёх треков в стиле синтвейв» → сет из кэша (путь ADR-0158),
    настоящий ``SetSession`` на симуляторе клока; реплики — тем же ``speak`` (``speak_text`` робота), что у сета
    синтезом; ``started`` треков 2 и 3 — с ``dj_line=on``; ``status`` отвечает по фактам играющего трека."""
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track", _fake_audio_track)
    lib, rig, said = _lib(tmp_path), _rig(tracks=3), []
    node = SimpleNamespace(get_logger=lambda: rig.log)
    dj = DjSetTool(node, rig.owner, melodies=lambda ids: {}, seed=lambda: 4006, speak=said.append, scores=SCORES)
    dj.gen_sets = gs.GenSetMaker(Kie(), lambda: lib, DeterministicChooser(), log=node.get_logger)  # как mcp_server
    out = dj.execute(action="start", tracks=3, heard_text=REQUEST)
    assert out.success, out.error
    assert out.data["source"] == "generated" and out.data["tracks"] == 3 and out.data["cached"] == 3
    rig.clock.run_until(rig.clock.beat + 2)
    first = _started(rig)[0]
    for k in (1, 2):
        _wait(lambda: len([e for e in rig.events if e["event"] == "queued"]) == k)
        rig.clock.run_until(first["start_beat"] + k * first["form_beats"] + 1)
    _wait(lambda: len(said) == 2)
    started = [m for _l, m in rig.log.lines if "[set v2]" in m and " started " in m]
    assert len(started) == 3 and "dj_line=start_phrase" in started[0]
    assert all("dj_line=on" in m for m in started[1:])
    assert len(said) == 2 and not [s for s in said if LABEL.lower() in s.lower()]
    assert rig.owner.is_playing()  # реплика поверх музыки: не стоп и не пауза
    _wait(lambda: dj.status().get("track_no") == 3)
    status = dj.status()
    assert status["active"] and status["tracks"] == 3
    # #4060: ответ «что играет» - название и стиль играющего трека, а не его номер в сете
    assert status["message"].startswith("Идёт диджей-сет. Сейчас играет «") and "синтвейв" in status["message"]
    assert "3 из 3" not in status["message"] and "gen40" not in status["message"]
