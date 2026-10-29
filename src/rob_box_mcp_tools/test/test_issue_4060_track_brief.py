"""#4060: «что сейчас играет» - короткий ответ из названия, стиля и темпа трека, а не номер и id (Шифу 27.10).

Раньше ``<music_state>`` показывал ``track: клубный трек · 136 BPM``, диджей отвечал «Сейчас трек 3 из 10», а в данных тула
лежал ключ ``gen48211:00:A``. Здесь поведение цепочки: факты трека -> снимок плеера (``brief``) -> раздел ``music_state``
хода; факты трека сета из генерации (стиль слота, оригинал кавера) доходят до снимка. Без ROS, Renardo и сети.
"""

from __future__ import annotations

import json
import re
from types import SimpleNamespace

import pytest

from rob_box_dialog.music_state import MusicStateMemory
from rob_box_mcp_tools.engine.gen import gen_session as gs
from rob_box_mcp_tools.engine.gen.library_play import description_facts
from rob_box_mcp_tools.engine.player_owner import PlayerOwner
from rob_box_mcp_tools.engine.search import ThemeHits
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool
from rob_box_music.source import cover_slot
from rob_box_music.track_brief import track_brief
from rob_box_voice.core.music_player_state import parse_music_state

from .test_engine_reasoner import _wait
from .test_engine_session import Log, _rig
from .test_issue_4006_gen_set_dj_lines import _gen, _lib, _lines, _play

pytestmark = pytest.mark.unit


class _Deck:
    """Дека без Renardo: ``start`` сразу зовёт ``started``."""

    def check(self, program):
        return None

    def start(self, program, on_started):
        on_started({"track_id": program.track_id, "deck": "A", "form_beats": 64.0, "start_beat": 0.0,
                    "phase_in_form": 0, "players_aligned": True, "bpm": program.bpm})

    def at(self, beat, fn):
        pass

    def stop(self):
        pass


def _playing(track_id, dj, *, bpm=130.0):
    """Владелец плеера играет трек с фактами ``dj``; возвращает (владелец, последний опубликованный снимок)."""
    states = []
    owner = PlayerOwner(_Deck(), states.append, lambda _e: None, clock=lambda: 1000.0)
    owner.play(SimpleNamespace(track_id=track_id, bpm=bpm, deck="A"), dj=dj)
    return owner, states[-1]


def _track_line(raw):
    """Строка ``track`` раздела ``music_state`` хода по снимку плеера, как её видит модель."""
    memory = MusicStateMemory()
    memory.observe_state(parse_music_state(raw), now=1001.0)
    return memory.section(now=1001.0)["track"]


def test_music_state_track_of_a_generated_set_track_is_title_and_style_not_id_or_number():
    """Трек сета из генерации: ключ ``gen48211:00:A`` и метка «Очень странные дела · трек 3 из 10» не доходят до модели."""
    _owner, raw = _playing("gen48211:00:A", {
        "enabled": True, "set_id": "gen48211", "track_no": 3, "tracks": 10, "name": "Ночной город",
        "source": "generated", "style": "synthwave", "library_id": "a1b2c3",
        "title": "Очень странные дела · трек 3 из 10"})
    assert json.loads(raw)["dj"]["brief"] == "«Ночной город», синтвейв"
    line = _track_line(raw)
    assert line == "«Ночной город», синтвейв"
    for key_of_code in ("gen48211", "00:A", "a1b2c3", "трек 3", "из 10"):
        assert key_of_code not in line


def test_music_state_track_of_a_club_track_has_no_bpm_label_but_style_and_tempo_words():
    """Раньше ``track: клубный трек · 136 BPM`` (стиля не было вовсе, темп англ. меткой)."""
    _owner, raw = _playing("req99682:03:A", {
        "enabled": False, "title": "клубный трек · 136 BPM", "source": "synth", "style": "club", "bpm": 136}, bpm=136.0)
    line = _track_line(raw)
    assert line == "трек в стиле клуб, синтез Ренардо, 136 ударов в минуту"
    assert "BPM" not in line and "req99682" not in line


def test_4060_order_words_never_become_the_title_of_a_synth_track_in_the_music_state():
    """Живой 28.10: снимок старого сервера кладёт слова заказа в ``name`` - раздел ``track`` их названием не берёт."""
    _owner, raw = _playing("req99682:03:A", {
        "enabled": False, "title": "клубный трек · 120 BPM", "name": "какую нибудь спокойную музыку",
        "source": "synth", "style": "club", "bpm": 120}, bpm=120.0)
    assert _track_line(raw) == "трек в стиле клуб, синтез Ренардо, 120 ударов в минуту"


def test_synth_set_track_without_melody_is_told_by_style_and_tempo():
    """Сет синтезом: названия у трека нет (мотив диджея) - «трек в стиле …», а не «диджей-сет · трек 2 из 5»."""
    _owner, raw = _playing("s7:02:B", {"enabled": True, "theme": "космос", "style": "synthwave", "set_id": "s7",
                                       "track_no": 2, "tracks": 5, "title": "космос · трек 2 из 5", "bpm": 118},
                           bpm=118.0)
    assert _track_line(raw) == "трек в стиле синтвейв, 118 ударов в минуту"


def test_snapshot_without_the_description_keeps_the_old_title():
    """Сервер старее подписчика: поля ``brief`` в снимке нет - раздел читает ``title``, как раньше."""
    raw = json.dumps({"state": "playing", "track_id": "x", "dj": {"enabled": False, "title": "космос · 120 BPM"},
                      "ts": 1000.0})
    assert _track_line(raw) == "космос · 120 BPM"


def test_track_with_nothing_to_describe_has_no_brief_and_stays_the_title():
    owner, raw = _playing("v2:1:A", {"enabled": False, "title": "космос"}, bpm=120.0)   # темп деки есть, описания нет
    assert "brief" not in json.loads(raw)["dj"] and "brief" not in owner.now_playing()
    assert _track_line(raw) == "космос"


def test_get_music_state_data_carries_the_same_description():
    owner, _raw = _playing("gen48211:00:A", {"enabled": False, "name": "Ночной город", "source": "generated",
                                             "style": "synthwave"})
    assert owner.now_playing()["brief"] == "«Ночной город», синтвейв"


def test_gen_set_tracks_publish_the_style_of_their_slot_and_the_original_of_a_cover(tmp_path):
    """Факты описания трека сета из генерации: стиль слота и (у кавера) оригинал - в снимок, до ``started`` и после."""
    published, log = [], Log()
    lines = _lines([], log, published=published)
    gen = _gen(_lib(tmp_path, cover=True), lines, keys=["synthwave", cover_slot("synthwave", "STAR WARS"), "synthwave"],
               published=published)
    _play(gen)
    by_track = {}
    for track_id, fields in published:
        by_track.setdefault(track_id, {}).update(fields)
    first, cover, last = (by_track[t] for t in sorted(by_track))
    assert first["style"] == "synthwave" and "cover" not in first
    assert cover["style"] == "synthwave" and cover["cover"] == "STAR WARS" and cover["source"] == "generated"
    # описание играющего трека - из тех же фактов, что получит снимок: название кэша, стиль слота, без темпа файла
    assert track_brief(cover) == "«STAR WARS (RoBBox cover)», кавер, синтвейв"
    assert track_brief(last) == "«Неоновая ночь», синтвейв"
    assert "gen40" not in track_brief(last)


def test_item_facts_drop_what_a_track_does_not_have():
    item = gs.SetItem("t1", "g1", "Название", genre="", hook="")
    assert gs.item_facts(item) == {}
    assert gs.item_facts(gs.SetItem("t2", "g2", "Кавер", genre=cover_slot("rock", "Группа крови"), hook="Группа крови")) == {
        "style": "rock", "cover": "Группа крови"}


def test_library_track_facts_carry_title_style_and_the_original_of_a_cover():
    assert description_facts({"title": "Космос", "style_key": "synthwave", "kind": "instrumental", "work_id": ""}) == {
        "name": "Космос", "style": "synthwave"}
    assert description_facts({"title": "Звёздные войны (кавер)", "style_key": "rock", "kind": "cover",
                              "work_id": "Звёздные войны"}) == {
        "name": "Звёздные войны (кавер)", "style": "rock", "cover": "Звёздные войны"}
    assert description_facts({}) == {}


def test_synth_set_status_and_snapshot_are_told_by_style_engine_and_tempo_not_by_the_track_number():
    """Сет синтезом без мелодий темы: ``get_music_state`` отвечает «трек в стиле …», а не «Сейчас трек 1 из N»."""
    rig = _rig()
    node = SimpleNamespace(get_logger=lambda: rig.log)
    tool = DjSetTool(node, rig.owner, melodies=lambda ids: {}, finder=lambda theme: ThemeHits(), seed=lambda: 1,
                     speak=None)
    assert tool.execute(action="start", theme="космос").success
    rig.clock.run_until(rig.clock.beat + 2)
    _wait(lambda: "melody" in tool.status(), timeout=30.0)   # факты трека готовит фоновый поток: под нагрузкой дольше 5 с
    message = tool.status()["message"]
    assert re.match(r"Идёт диджей-сет\. Сейчас играет трек в стиле [а-яё-]+, синтез Ренардо, \d+ уда\w+ в минуту "
                    r"— свой мотив диджея", message), message
    brief = json.loads(rig.states[-1])["dj"]["brief"]
    assert brief.startswith("трек в стиле ") and "синтез Ренардо" in brief and "из " not in brief
