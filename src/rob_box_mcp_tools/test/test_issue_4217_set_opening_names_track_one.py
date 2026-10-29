"""#4217 (n605 марафона-5): «поставь сет из трёх треков, объявляй каждый» - первый трек звучал общей фразой «Включаю
музыку.», а не объявлением. Объявляет код (ADR-0148): название трека 1 берётся из данных сета (файл кэша), ``dj_set``
кладёт его в исход, фраза старта - из каталога диалога. Без объявлений (реплики диджея выключены) - как было.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from rob_box_mcp_tools.engine.gen import gen_session as gs
from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, set_opening
from rob_box_music.choice import DeterministicChooser

from .test_adr0158_pr8_gen_set import Kie, _fake_audio_track
from .test_engine_session import Log, _rig
from .test_issue_4006_gen_set_dj_lines import SCORES, _gen, _lines, _play

pytestmark = pytest.mark.unit

N605 = "Робот, поставь сет из трёх треков, объявляй каждый."
TITLES = ("Рок дискотека у тёти Гриши", "Гитарный шторм", "Ночной драйв")


def _titled_lib(tmp_path):
    lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
    for task, title in zip("ABC", TITLES):
        lib.save_track(b"MP3:150.0", prompt="club", title=title, is_instrumental=True, kind="instrumental",
                       style_key="club", genre="club", task_id=task, variant=0, duration_ms=150000)
    return lib


def test_opening_is_the_title_of_track_one_once_it_started(tmp_path):
    said, log = [], Log()
    lines = _lines(said, log)
    gen = _gen(_titled_lib(tmp_path), lines)
    assert lines.opening.title(0.01) == ""  # трек 1 ещё не встал - названия нет, выдумки тоже
    _play(gen, last=1)
    assert lines.opening.title() in TITLES
    assert said == []  # реплики трека 1 нет: его объявляет фраза старта сета


def test_opening_is_empty_when_dj_lines_are_off(tmp_path):
    said, log = [], Log()
    lines = _lines(said, log, speak=False)
    _play(_gen(_titled_lib(tmp_path), lines), last=1)
    assert not lines.opening.announces and lines.opening.title(0.01) == ""


def test_set_opening_adds_title_and_length_words_only_when_started_and_named():
    class Lines:
        def __init__(self, title):
            self.opening = SimpleNamespace(title=lambda: title)

    started = {"ok": True, "started": True, "tracks": 3}
    assert set_opening(started, Lines("Гитарный шторм")) == {
        **started, "opening": "Гитарный шторм", "tracks_text": "3 трека"}
    assert set_opening(started, Lines("")) == started  # названия нет - исход прежний
    assert set_opening(started, None) == started  # реплик диджея нет
    assert set_opening({"ok": True, "tracks": 3}, Lines("X")) == {"ok": True, "tracks": 3}  # started не пришло
    assert set_opening({**started, "tracks": 1}, Lines("X"))["tracks_text"] == "1 трек"


def _dj(tmp_path, monkeypatch, **kwargs):
    monkeypatch.setattr("rob_box_mcp_tools.engine.gen.gen_session.audio_track", _fake_audio_track)
    lib, rig = _titled_lib(tmp_path), _rig(tracks=3)
    node = SimpleNamespace(get_logger=lambda: rig.log)

    def confirm(_track_id):  # настоящий ``started`` приходит от клока: гоним его и отдаём событие
        rig.clock.run_until(rig.clock.beat + 2)
        return SimpleNamespace(event="started", fields={})

    dj = DjSetTool(node, rig.owner, melodies=lambda ids: {}, seed=lambda: 4217, scores=SCORES, confirm=confirm,
                   **kwargs)
    dj.gen_sets = gs.GenSetMaker(Kie(), lambda: lib, DeterministicChooser(), log=node.get_logger)
    return dj


def test_dj_set_n605_result_names_first_track_of_the_set(tmp_path, monkeypatch):
    """Целиком через ``dj_set``: просьба n605 -> сет из кэша; после ``started`` трека 1 в исходе его название."""
    said = []
    dj = _dj(tmp_path, monkeypatch, speak=said.append)
    out = dj.execute(action="start", tracks=3, heard_text=N605)
    assert out.success, out.error
    assert out.data["started"] and out.data["tracks"] == 3
    assert out.data["opening"] in TITLES and out.data["tracks_text"] == "3 трека"
    assert said == []  # трек 1 реплики диджея не получает: название звучит во фразе старта (диалог)


def test_dj_set_with_dj_lines_off_gives_no_opening(tmp_path, monkeypatch):
    dj = _dj(tmp_path, monkeypatch, speak=None, lines=False)
    out = dj.execute(action="start", tracks=3, heard_text=N605)
    assert out.success, out.error
    assert "opening" not in out.data
