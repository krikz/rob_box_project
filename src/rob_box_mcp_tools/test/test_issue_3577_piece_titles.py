"""#3577 (эпик #3557, ADR-0157): одиночная пьеса — главное слово названия, узнаваемое произведение, произносимое имя.

Прогон 12–13.10 на роботе: «сыграй Токкату» → «Bach Fugue in C-Major (from Toccata Adagio and Fugue…)» (фуга: слово
только в скобках источника); «сыграй Баха» ×3 по рейтингу PDMX → Little Prelude BWV 924, хоралы, концерт на 760
долей; фразы TTS — каталожные карточки PDMX («… (real audio)», «… By W. A. Mozart», «SuÌßmayr»). Строки индекса —
настоящие названия и числа ``score_index`` робота (title, composer, n_views, bars, meter, rating); нот нет (В1).
"""

from __future__ import annotations

import sqlite3
import sys
import threading
from pathlib import Path

import pytest

from rob_box_music import knowledge as kn
from rob_box_music import works
from rob_box_music.choice import DeterministicChooser, points
from rob_box_music.works import TITLE_EXACT, WorkRef, WorkWord, hook_words
from rob_box_voice.core.media_phrases import piece_text

from rob_box_mcp_tools.core.rtttl_library import fix_mojibake, spoken_title
from rob_box_mcp_tools.engine.piece import levels
from rob_box_mcp_tools.engine.score_library import INDEX_FILE, ScoreIndex

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts" / "research"))
import music_eval  # noqa: E402

BACH, MOZART, BEETHOVEN = WorkWord(("bach",)), WorkWord(("mozart",)), WorkWord(("beethoven",))
TOCCATA = WorkWord(("toccata",), ("toccata", "toccataa"))


def _row(mid, title, composer, views, bars, meter="4/4", rating=4.8):
    return {"material_id": f"pdmx:{mid}", "title": title, "composer": composer, "rating": rating, "n_ratings": 10,
            "n_views": views, "bars": bars, "meter": meter, "genres": "classical"}


#: Строки индекса робота 12.10 (просмотры, такты, размер — как в ``score_index``; рейтинг малоизвестных — выше).
ROWS = [
    _row("bwv924", "J. S. Bach: Little Prelude in C Major BWV 924", "Johann Sebastian Bach", 1200, 18, rating=4.95),
    _row("chorales", "Bach's chorale harmonisations 241-371 (Breitkopf numbering no text)", "J. S. Bach", 900, 48,
         rating=4.97),
    _row("bwv1052", "J. S. Bach - Harpsichord Concerto No. 1 in D-Minor BWV 1052 1. Allegro (real audio)",
         "J. S. Bach", 2100, 190, rating=4.9),
    _row("bwv846", "Prelude I in C major BWV 846 - Well Tempered Clavier [First Book]", "J. S. Bach", 511539, 34,
         rating=4.78),
    _row("bwv565", "Bach Toccata and Fugue in D Minor (Piano solo)", "J. S. Bach", 225986, 143, rating=4.8),
    _row("bwv564", "Bach Fugue in C-Major (from Toccata Adagio and Fugue in C Major BWV 564)", "J. S. Bach", 800, 60,
         rating=4.99),
    _row("seixas", "Toccata in G minor (Seixas)", "Carlos Seixas", 400, 40, rating=4.7),
    _row("jesu", "Jesu, Joy of Man's Desiring", "J.S. Bach", 600, 40, "3/4", rating=4.6),
    _row("rex", "Requiem in D Minor K. 626 III. Sequentia: Rex Tremendae By W. A. Mozart",
         "Wolfgang Amadeus Mozart Franz Xaver SuÌßmayr", 1751, 22, rating=4.95),
    _row("lacrimosa", "Mozart - Lacrimosa (from the Requiem in D minor)", "W. A. Mozart", 181349, 30, "12/8"),
    _row("k545", "Mozart - Piano Sonata No. 16 - Allegro", "Wolfgang Amadeus Mozart", 161948, 73),
    _row("emp5", "Beethoven Piano Concerto No. 5 (1st Movement) Arr. for 2 Pianos", "Ludwig van Beethoven", 3000,
         256, rating=4.95),
    _row("sym5", "Beethoven Symphony No. 5 (1st movement) Piano solo", "Ludwig van Beethoven", 392070, 268, "2/4"),
    _row("minuet", "Ludwig van Beethoven - Minuet in G (Solo Violin)", "Ludwig van Beethoven", 19611, 17, "3/4"),
    # имя в поле «композитор» чаще, чем в названиях (как в индексе: роль слова — по доле полей)
    _row("k525", "Eine kleine Nachtmusik", "W. A. Mozart", 900, 40, rating=4.9),
    _row("k331", "Rondo Alla Turca", "Wolfgang Amadeus Mozart", 950, 60, "2/4", rating=4.9),
    _row("op27", "Sonata quasi una fantasia", "L. v. Beethoven", 700, 69, "2/2", rating=4.9),
    _row("woo59", "Bagatelle in A minor", "Ludwig van Beethoven", 800, 52, "3/8", rating=4.9),
]


# ── 1. главное слово названия ──

def test_title_head_drops_brackets_and_the_source_tail():
    assert works.title_head("Bach Fugue in C-Major (from Toccata Adagio and Fugue in C Major BWV 564)") == \
        "Bach Fugue in C-Major"
    assert works.title_head("Mozart Lacrymosa from Requiem (SATB with piano)") == "Mozart Lacrymosa"
    assert works.title_head("Toccata in G minor (Seixas)") == "Toccata in G minor"
    assert works.title_head("(Toccata)") == "(Toccata)"  # пусто после среза — всё название


def test_word_only_in_the_source_bracket_is_not_a_title_match():
    """Шифу 13.10: «сыграй токкату» — любая токката (Сейшас — ок), но фуга «(from Toccata…)» — не токката."""
    ref = WorkRef(title=(TOCCATA,))
    assert ref.title_match("Bach Fugue in C-Major (from Toccata Adagio and Fugue in C Major BWV 564)", 1) == 0.0
    assert ref.title_match("Toccata and Fugue in D Minor", 1) == TITLE_EXACT
    assert ref.title_match("Toccata in G minor (Seixas)", 1) == TITLE_EXACT
    keyed = WorkRef(title=(TOCCATA,), key="D minor")
    assert keyed.title_match("Toccata and Fugue in D Minor", 1) == TITLE_EXACT


def test_toccata_order_puts_toccatas_first_and_the_fugue_into_the_rest():
    ref = WorkRef(title=(TOCCATA,))
    found = ScoreIndex(ROWS).work_candidates(ref)
    by_id = {c.id: c for c in found}
    assert by_id["pdmx:bwv564"].title_match == 0.0
    first = levels(ref, found)[0]
    assert {c.id for c in first} == {"pdmx:bwv565", "pdmx:seixas"}
    assert DeterministicChooser().choose(_question(first)).id in {"pdmx:bwv565", "pdmx:seixas"}


def _question(cands):
    from rob_box_music.choice import ChoiceQuestion, Option
    return ChoiceQuestion("токкату", tuple(Option(c.id, c.features()) for c in cands))


# ── 2. узнаваемость и длинная форма ──

def test_known_weight_is_above_rating_and_long_form_is_a_penalty():
    assert kn.CHOOSER_WEIGHTS["known"] > kn.CHOOSER_WEIGHTS["rating"] > 0 > kn.CHOOSER_WEIGHTS["long"]
    assert set(points({"known": 1.0, "long": 1.0})) == set(kn.CHOOSER_WEIGHTS)


def test_known_work_by_views_and_by_hook_title():
    assert works.known_work(None) == works.known_work(kn.KNOWN_VIEWS_FLOOR) == 0.0
    assert works.known_work(kn.KNOWN_VIEWS_FULL) == works.known_work(511539) == 1.0
    assert works.known_work(10_000) == 0.5
    hooks = hook_words([{"title": "Jesu Joy Of Man's Desiring", "artist": "Bach"},
                        {"title": "J.S.Bach", "artist": "J.S.Bach"}])  # имя автора — не произведение
    assert len(hooks) == 1
    assert works.known_work(600, "Jesu, Joy of Man's Desiring", hooks) == 1.0
    assert works.known_work(600, "Little Prelude in C Major", hooks) == 0.0


def test_form_beats_and_long_form():
    assert works.form_beats(190, "4/4") == 760.0 and works.form_beats(136, "3/8") == 204.0
    assert works.long_form(works.form_beats(34, "4/4")) == 0.0
    assert works.long_form(760) == pytest.approx(0.979, abs=1e-3) and works.long_form(1024) == 1.0


@pytest.mark.parametrize("composer, expected", [
    (BACH, {"pdmx:bwv846", "pdmx:bwv565", "pdmx:jesu"}),
    (MOZART, {"pdmx:lacrimosa", "pdmx:k545"}),
    (BEETHOVEN, {"pdmx:sym5", "pdmx:minuet"}),
])
def test_composer_only_order_prefers_the_recognizable_work(composer, expected):
    """«Сыграй Баха/Моцарта/Бетховена»: первым — узнаваемое (просмотры PDMX или хук RTTTL «Jesu Joy»), а не лучший
    рейтинг (BWV 924, хоралы, Rex Tremendae) и не концерт на 760/1024 доли."""
    hooks = hook_words([{"title": "Jesu Joy Of Man's Desiring", "artist": "Bach"}])
    found = ScoreIndex(ROWS).work_candidates(WorkRef(composer=(composer,)), hooks=hooks)
    ranked = DeterministicChooser().rank(_question(found))
    top = {o.id for s, o in ranked if s >= ranked[0][0] - kn.CHOOSER_CLEAR_GAP}
    assert top <= expected, ranked
    unknown = {"pdmx:bwv924", "pdmx:chorales", "pdmx:bwv1052", "pdmx:rex", "pdmx:emp5"}
    assert not top & unknown


# ── 3. произносимое название ──

@pytest.mark.parametrize("title, composer, spoken", [
    ("J. S. Bach - Harpsichord Concerto No. 1 in D-Minor BWV 1052 1. Allegro (real audio)", "J. S. Bach",
     ("Harpsichord Concerto No. 1 in D-Minor 1. Allegro", "Бах")),
    ("Requiem in D Minor K. 626 III. Sequentia: Rex Tremendae By W. A. Mozart",
     "Wolfgang Amadeus Mozart Franz Xaver SuÌßmayr",
     ("Requiem in D Minor III. Sequentia: Rex Tremendae", "Моцарт")),
    ("Beethoven Piano Concerto No. 5 (1st Movement) Arr. for 2 Pianos", "Ludwig van Beethoven",
     ("Piano Concerto No. 5 (1st Movement)", "Бетховен")),
    ("{Professional production score.} Moonlight Sonata", "Beethoven", ("Moonlight Sonata", "Бетховен")),
    ("J. S. Bach: Little Prelude in C Major BWV 924", "Johann Sebastian Bach", ("Little Prelude in C Major", "Бах")),
    ("Bach's chorale harmonisations 241-371 (Breitkopf numbering no text)", "J. S. Bach",
     ("chorale harmonisations 241-371", "Бах")),
    ("Toccata in G minor (Seixas)", "Carlos Seixas", ("Toccata in G minor", "Carlos Seixas")),
    ("Mozart: Minuet in F Major (K2) (easy)", "W. A. Mozart", ("Minuet in F Major", "Моцарт")),
    ("Stand By Me", "Ben E. King", ("Stand By Me", "Ben E. King")),  # «By» не автора — не хвост
    ("Калинка", "", ("Калинка", "")),  # русский алиас RTTTL — как есть
])
def test_spoken_title_strips_pdmx_junk(title, composer, spoken):
    got = spoken_title(title, composer)
    assert got == spoken
    assert len(got[0]) <= kn.SPOKEN_TITLE_MAX


def test_spoken_title_is_never_longer_than_the_limit():
    title, _c = spoken_title("Sonata " + " ".join(f"Movement{i}" for i in range(20)) + " (Breitkopf edition)")
    assert 0 < len(title) <= kn.SPOKEN_TITLE_MAX and not title.endswith(" ")


def test_fix_mojibake_decodes_pairs_and_drops_lost_byte_fragments():
    assert fix_mojibake("DvoÅ\u0099Ã¡k") == "Dvořák"  # пара UTF-8→latin-1 — обратно
    assert fix_mojibake("cafÃ©") == "café"
    assert fix_mojibake("Dvořák") == "Dvořák"  # правильный текст не трогается
    # в индексе робота байт продолжения потерян (``'Su\xcc\xdfmayr'``): «ü» не восстановить — обломок прочь
    assert fix_mojibake("SuÌßmayr") == "Sußmayr"


def test_piece_phrase_is_built_from_the_spoken_title():
    title, composer = spoken_title(ROWS[8]["title"], ROWS[8]["composer"])
    phrase = piece_text({"title": title, "composer": composer, "source": "score"})
    assert phrase == "Ставлю «Requiem in D Minor III. Sequentia: Rex Tremendae», Моцарт."
    assert "Ì" not in phrase and "By" not in phrase


# ── набор реплик: «узнаваемое» на индексе партитур ──

def _library(path):
    path.mkdir(parents=True, exist_ok=True)
    rows = [works.ScoreIndexRow(r["material_id"], r["title"], r["composer"], r["meter"], "C major", r["bars"], "PD",
                                1, f"{i}.json", r["genres"], r["rating"], r["n_ratings"], r["n_views"])
            for i, r in enumerate(ROWS)]
    conn = sqlite3.connect(str(path / INDEX_FILE))
    works.write_score_index(conn, rows)
    conn.close()
    return str(path)


def test_dataset_known_rows_choose_recognizable_scores(tmp_path):
    """``known: true`` набора ``music_requests.yaml`` на индексе с настоящими строками робота: «сыграй Баха/Моцарта/
    Бетховена» — узнаваемая партитура, «сыграй Токкату» — токката (не фуга «from Toccata»)."""
    chain = music_eval.offline_chain(scores_dir=_library(tmp_path / "scores"), db_dir=str(tmp_path))
    ids = {"pr3-bach", "pr3-mozart", "i3577-mozart", "i3577-beethoven", "i3577-toccata"}
    items = [i for i in music_eval.load() if i["id"] in ids]
    report = music_eval.evaluate(items, chain, DeterministicChooser())
    rows = {r["id"]: r for r in report["rows"]}
    assert {i: r["bad"] for i, r in rows.items() if r["bad"]} == {}
    assert all(rows[i]["got"]["choice"]["kind"] == "score" for i in rows if i != "i3577-toccata")
    assert "fugue in c" not in rows["i3577-toccata"]["got"]["choice"]["title"].lower()


# ── 4. прогрев ──

def test_request_tool_warms_the_piece_path_in_background(tmp_path, caplog):
    from rob_box_mcp_tools.engine.score_library import ScoreLibrary
    from rob_box_mcp_tools.engine.search import ThemeHits
    from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, RequestMusicTool

    from .test_engine_session import _rig

    caplog.set_level("INFO")
    rig = _rig()
    scores = ScoreLibrary(_library(tmp_path / "scores"))
    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=lambda theme: ThemeHits(), scores=scores)
    tool = RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, records=lambda ids: {},
                            words=lambda word: ThemeHits())
    thread = tool.warm()
    assert isinstance(thread, threading.Thread)
    thread.join(10)
    assert not thread.is_alive() and scores._index is not None  # индекс построен до первого заказа
    line = next(r.getMessage() for r in caplog.records if "прогрев" in r.getMessage())
    assert "индекс партитур" in line and "каталог RTTTL" in line
