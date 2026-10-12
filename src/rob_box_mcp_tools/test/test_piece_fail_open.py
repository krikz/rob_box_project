"""ADR-0157 PR-2 (#3559) — одиночная пьеса fail-open: кандидаты произведения, история, фраза от кода.

Прогон 09.10: «Токката ре минор» — 8 партитур отвергнуты (годность 0.55–0.73 < 0.8), запасной путь искал сырую фразу и
давал ``not_found``, хотя хуки ``toccata``/``fugueind`` код уже нашёл; «сыграй Баха» трижды играл ``unknown_75``.
Здесь — поведение ``request_music`` на настоящем владельце плеера (``_rig``) с подставными каталогами.
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

from rob_box_mcp_tools.engine.classic import ClassicPick, Candidate, fresh_first, key_slot
from rob_box_mcp_tools.engine.search import PartQuery, Term, ThemeHits, ThemeQuery
from rob_box_mcp_tools.engine.tools_v2 import NOT_FOUND, DjSetTool, RequestMusicTool
from rob_box_music import knowledge as kn
from rob_box_music import material as mt
from rob_box_music.diversity import MusicHistory
from rob_box_music.model import Key, PitchEvent
from rob_box_voice.core.media_phrases import launch_text, piece_text
from rob_box_voice.core.music_turn import Launch, ok_text

from .test_engine_session import _rig, _started

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "rob_box_music" / "test"))
from melodies import CHROMATIC, LONG, SHORT, SLOW  # noqa: E402


class Scores:
    """Подставной индекс партитур: ``search`` — материалы по рангу, ``load`` — сами материалы."""

    state = "подставной индекс"

    def __init__(self, materials=None):
        self.materials = dict(materials or {})

    def search(self, query, limit=8):
        return tuple(self.materials)

    def load(self, ids):
        return {i: self.materials[i] for i in ids if i in self.materials}, []

    def titles(self, ids):
        return {}


def _material(mid: str) -> mt.ScoreMaterial:
    """Материал, который складывается в песню, но без мотива хроматического эталона (``HookError``)."""
    melody = tuple(PitchEvent(60 + (i % 4) * 2, float(i), 1.0, 2) for i in range(32))
    chords = tuple(mt.ChordSpan(float(b * 4), 4.0, 0, "maj", 0) for b in range(8))
    return mt.ScoreMaterial(mid, f"Toccata score {mid}", "t", "s", "PD", (4, 4), 120, Key(2, "minor"), melody, chords)


def _records(table):
    return lambda ids: {i: ({"name": i, "title": table[i][0], "rtttl": table[i][1],
                             "artist": table[i][2] if len(table[i]) > 2 else ""}, table[i][0])
                        for i in ids if i in table}


def _hits(names, word="", *alts):
    """Находки поиска с написаниями слова заказа в архиве (как у ``theme_search``)."""
    query = ThemeQuery(word, (PartQuery(word, (Term(word, alts),)),)) if word else None
    return ThemeHits(tuple(names), query=query)


def _tools(rig, hooks=(), scores=None, table=None, words=lambda word: ThemeHits(), history=None, classic=None,
           asked=None, confirm=None, hits=None):
    def finder(theme):
        if asked is not None:
            asked.append(theme)
        return hits if hits is not None else ThemeHits(tuple(hooks))

    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=finder, seed=lambda: 4242,
                   scores=scores or Scores())
    miss = classic or (lambda query, *, seed, deck="A", lead=None: ClassicPick(query, False, "lookup: не найдена"))
    return RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, seed=lambda: 5, classic=miss,
                            history=history, records=_records(table or {}), words=words, confirm=confirm)


TOCCATA = {"toccata": ("Toccata", CHROMATIC, "Bach"), "fugueind": ("Fugue in D minor", LONG, "Bach")}


def test_toccata_with_eight_refused_scores_plays_the_rtttl_of_the_work():
    """R3/R5: 8 партитур отвергнуты эталоном → RTTTL ``toccata`` того же произведения; ``not_found`` нет; тональность
    («ре минор») — не слово поиска."""
    rig = _rig()
    asked = []
    scores = Scores({f"pdmx:{n}": _material(f"pdmx:{n}") for n in range(8)})
    req = _tools(rig, hooks=("toccata", "fugueind"), scores=scores, table=TOCCATA, asked=asked)
    result = req.execute(intent="melody", text="сыграй токкату ре минор")
    assert result.success, result.error
    data = result.data
    assert data["melody_id"] == "toccata" and data["played"]["source"] == "rtttl"
    assert data["played"]["scores_failed"] == 8 and data["played"]["replacement_for"] is None
    assert asked == ["токкату"]
    assert data["message"] == "Ставлю «Toccata», Bach — мелодией, нот для полной аранжировки нет."
    rig.clock.run_until(rig.clock.beat + 2)
    assert [e["track_id"] for e in _started(rig)] == [data["track_id"]]


def test_a_score_that_makes_a_song_plays_first():
    rig = _rig()
    good = replace(_material("pdmx:good"), title="Toccata and Fugue")
    names = ("c", "c#", "d", "d#", "e", "f", "f#", "g", "g#", "a", "a#", "b")
    motif = "ref:d=4,o=5,b=120:" + ",".join(f"{names[e.midi % 12]}{e.midi // 12 - 1}" for e in good.melody[:12])
    table = {"toccata": ("Toccata", motif)}  # эталон — мотив этой партитуры: годность 1.0
    req = _tools(rig, hooks=("toccata",), scores=Scores({"pdmx:good": good}), table=table)
    data = req.execute(intent="melody", text="сыграй токкату").data
    assert data["ok"] and data["melody_id"] == "pdmx:good" and data["played"]["source"] == "score"
    assert data["message"] == "Ставлю «Toccata and Fugue»."


BACH = {"brandenb": ("Brandenburg Concerto", LONG, "Bach"), "jesujoyo": ("Jesu Joy", SHORT, "Bach"),
        "minuet": ("Minuet", CHROMATIC, "J.S.Bach")}  # у SLOW тот же контур начала, что у LONG


def test_three_bach_requests_play_three_different_pieces_and_history_survives_a_restart():
    """R4: три «сыграй Баха» подряд — три разные записи (``music_history`` читается в classic-пути); четвёртый —
    самая давняя; новый процесс с той же БД продолжает очередь, а не начинает с первой."""
    history = MusicHistory(":memory:")
    played = []
    for _ in range(4):
        req = _tools(_rig(), hooks=tuple(BACH), table=BACH, history=history)
        played.append(req.execute(intent="melody", text="сыграй баха").data["melody_id"])
    assert played[:3] == ["brandenb", "jesujoyo", "minuet"] and played[3] == "brandenb"
    rows = history.recent(10, kit=kn.CLASSIC_HISTORY_KIT)
    assert [r["melody_name"] for r in rows] == ["brandenb", "minuet", "jesujoyo", "brandenb"]
    restarted = _tools(_rig(), hooks=tuple(BACH), table=BACH, history=history)
    assert restarted.execute(intent="melody", text="сыграй баха").data["melody_id"] == "jesujoyo"


def test_played_piece_model_goes_to_the_tracks_dir(tmp_path):
    """I24: модель сыгранной пьесы — в каталог моделей, как у трека сета (``<track_id>.json``)."""
    req = _tools(_rig(), hooks=("toccata",), table=TOCCATA)
    req._tracks_dir = str(tmp_path)
    data = req.execute(intent="melody", text="сыграй токкату").data
    files = [p.name for p in tmp_path.iterdir()]
    assert files == [data["track_id"].replace(":", "_") + ".json"]


def test_history_is_written_only_after_the_piece_started():
    history = MusicHistory(":memory:")
    rig = _rig()
    req = _tools(rig, hooks=tuple(BACH), table=BACH, history=history, confirm=lambda track_id: None)
    assert not req.execute(intent="melody", text="сыграй баха").success
    assert history.recent(10, kit=kn.CLASSIC_HISTORY_KIT) == []


def test_no_candidates_at_all_is_not_found_and_the_deck_is_untouched():
    rig = _rig()
    result = _tools(rig).execute(intent="melody", text="сыграй абвгдейку")
    assert result.success is False and result.data["reason"] == NOT_FOUND and result.data["found"] is False
    assert rig.events == []


def test_candidates_that_all_fail_are_a_loud_compose_error_not_not_found():
    rig = _rig()
    broken = {"toccata": ("Toccata", "toccata:d=4,o=5,b=100:")}  # пустая мелодия: песня не складывается
    result = _tools(rig, hooks=("toccata",), table=broken).execute(intent="melody", text="сыграй токкату")
    assert result.success is False and result.data["reason"] == "compose_error"


def test_nothing_of_the_work_plays_the_nearest_by_words_and_says_so():
    """Произведения нет ни в нотах, ни в RTTTL → ближайшее по словам заказа («Баха»), фраза называет обе вещи."""
    rig = _rig()
    words = {"баха": _hits(["brandenb"])}
    req = _tools(rig, table=BACH, words=lambda word: words.get(word, ThemeHits()))
    data = req.execute(intent="melody", text="сыграй токкату баха").data
    assert data["ok"] and data["melody_id"] == "brandenb"
    assert data["played"]["replacement_for"] == "токкату баха"
    assert data["message"] == "«Токкату баха» не нашёл. Ставлю ближайшее: «Brandenburg Concerto», Bach."


def test_old_whole_name_search_still_plays_before_the_nearest():
    rig = _rig()
    asked = []

    def classic(query, *, seed, deck="A", lead=None):
        asked.append(query)
        return ClassicPick(query, False, "lookup: не найдена")

    _tools(rig, classic=classic).execute(intent="melody", text="поставь калинку")
    assert asked == ["калинку"]


def test_the_voice_says_the_phrase_built_from_the_tool_result():
    """R6: голос (ход LLM, ``music_turn``) говорит фразу из ``played`` результата, а не текст модели."""
    rig = _rig()
    data = _tools(rig, hooks=("toccata",), table=TOCCATA).execute(intent="melody", text="сыграй токкату").data
    launch = Launch("request_music", {"intent": "melody", "text": "сыграй токкату"}, True, repr(data))
    assert ok_text(launch) == data["message"] == "Ставлю «Toccata», Bach — мелодией, нот для полной аранжировки нет."


def test_piece_phrase_names_what_plays_and_what_it_replaced():
    assert piece_text({"title": "Fugue", "composer": "Bach", "source": "rtttl", "replacement_for": "токкату"}) == (
        "«Токкату» не нашёл. Ставлю ближайшее: «Fugue», Bach.")
    assert piece_text({"title": "Toccata", "composer": "J. S. Bach", "source": "score"}) == (
        "Ставлю «Toccata», J. S. Bach.")
    assert piece_text({"title": "J.S.Bach", "composer": "J.S.Bach", "source": "score"}) == "Ставлю «J.S.Bach»."
    played = {"title": "Toccata", "source": "rtttl", "replacement_for": None}
    assert launch_text("Включаю музыку.", {"played": played, "instrument": "xylophone",
                                           "instrument_played": "xylophone"}) == (
        "Ставлю «Toccata» — мелодией, нот для полной аранжировки нет. Играю на ксилофоне.")
    assert launch_text("Включаю музыку.", {}) == "Включаю музыку."


def test_key_words_are_a_slot_not_search_words():
    assert key_slot("токкату ре минор") == ("токкату", "D minor")
    assert key_slot("сонату си-бемоль мажор") == ("сонату", "A# major")
    assert key_slot("фугу до диез минор баха") == ("фугу баха", "C# minor")
    assert key_slot("до свидания") == ("до свидания", "")


def test_fresh_first_puts_the_least_recent_of_the_played_first():
    c = [Candidate("rtttl", n) for n in "abcd"]
    assert [x.ref for x in fresh_first(c, ["b", "a"])] == ["c", "d", "a", "b"]
    assert [x.ref for x in fresh_first(c, ["c", "a", "b", "d"])] == ["d", "b", "a", "c"]


#: Находки «баха» на роботе 12.10 (theme_search): паразиты «Mundian To Bach Ke» (Panjabi MC) — первыми.
BACH_HITS = {"mundiant_2": ("Mundian To Bach Ke", LONG, "Panjabi MC"), "brandenb": ("Brandenburg Concert No. 3", SHORT,
                                                                                    "Bach"),
             "mundiant": ("Mundian To Bach Ke V2", LONG, "Panjabi MC"), "unknown_75": ("J.S.Bach", SLOW, "J.S.Bach"),
             "jesujoyo": ("Jesu Joy Of Man's Desiring", CHROMATIC, "Bach")}


def test_bach_three_times_never_plays_the_name_parasite():
    """R3 (прогон 12.10 16:21: «сыграй Баха» → «Mundian To Bach Ke»): слово «Bach» — имя автора (оно в ``artist`` у
    «Brandenburg Concert»), а у трека Panjabi MC — только в названии: такие записи не кандидаты. Три заказа — три
    разных произведения Баха."""
    history = MusicHistory(":memory:")
    played = []
    for _ in range(3):
        req = _tools(_rig(), hits=_hits(list(BACH_HITS), "баха", "bach"), table=BACH_HITS, history=history)
        played.append(req.execute(intent="melody", text="сыграй баха").data["melody_id"])
    assert played == ["brandenb", "unknown_75", "jesujoyo"]


def test_versions_of_a_played_tune_count_as_played():
    """Версии одной мелодии (общий контур начала) — одна мелодия для свежести: после «brandenb» его версия
    «brandenb_2» уходит в конец очереди."""
    table = {**BACH, "brandenb_2": ("Brandenburg Concerto 2", LONG, "Bach")}
    history = MusicHistory(":memory:")
    played = []
    for _ in range(3):
        req = _tools(_rig(), hooks=("brandenb", "brandenb_2", "jesujoyo", "minuet"), table=table, history=history)
        played.append(req.execute(intent="melody", text="сыграй баха").data["melody_id"])
    assert played == ["brandenb", "jesujoyo", "minuet"]


def test_named_work_keeps_its_only_good_score_on_repeat():
    """Решение координатора 12.10 (16:33 «Токката Баха» → партитура QmU8gH…, 16:34 → история вытолкнула её в
    рингтон ``toccataa_2``): произведение названо — партитура раньше RTTTL всегда, свежесть только внутри уровня."""
    good = replace(_material("pdmx:good"), title="Toccata and Fugue in D Minor", composer="J. S. Bach")
    names = ("c", "c#", "d", "d#", "e", "f", "f#", "g", "g#", "a", "a#", "b")
    motif = "ref:d=4,o=5,b=120:" + ",".join(f"{names[e.midi % 12]}{e.midi // 12 - 1}" for e in good.melody[:12])
    table = {"toccata": ("Toccata", motif, "Bach"), "toccataa": ("Toccata And Fugue", motif, "Bach")}
    other = tuple(PitchEvent(60 + (i % 3), float(i), 1.0, 2) for i in range(32))  # мотива эталона нет
    scores = Scores({"pdmx:bad": replace(_material("pdmx:bad"), melody=other), "pdmx:good": good})
    history = MusicHistory(":memory:")
    played = []
    for _ in range(2):
        req = _tools(_rig(), hits=_hits(list(table), "токкату", "toccata"), scores=scores, table=table,
                     history=history)
        played.append(req.execute(intent="melody", text="сыграй токкату").data["melody_id"])
    assert played == ["pdmx:good", "pdmx:good"]
