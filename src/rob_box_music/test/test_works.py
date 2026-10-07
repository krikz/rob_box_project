"""ADR-0155 K-2: реестр произведений — модель, чистка идентичности RTTTL, версии, алиасы, гейт, SQLite."""

from __future__ import annotations

import sqlite3

import pytest

from rob_box_music import works as w

UP = "t:d=4,o=5,b=120:c,d,e,f,g,a,b,c6,d6"
UP_TRANSPOSED = "t:d=4,o=6,b=100:d,e,f#,g,a,b,c#7,d7,e7"
LONE = "t:d=4,o=5,b=120:c,c,c,c,d,e,f,g,a"


def rec(name, title, artist="", tags=(), rtttl=UP):
    return {"name": name, "title": title, "artist": artist, "tags": list(tags), "rtttl": rtttl}


# ── нормализация и модель ─────────────────────────────────────────────────────────────────────────────────────

def test_norm_drops_case_yo_and_punctuation():
    assert w.norm("  Звёздные Войны (v1.0)! ") == "звездныевойныv10"
    assert w.norm(None) == ""


def test_fact_without_value_is_rejected_not_stored_as_empty_string():
    with pytest.raises(ValueError):
        w.Fact("  ", "rtttl")
    with pytest.raises(ValueError):
        w.Fact("tv", "somewhere")


def test_exact_artist_link_is_a_proposal_until_a_human_confirms_it():
    """ADR-0155 В6: сопоставление по названию не подтверждается автоматически."""
    for level in ("exact_artist", "exact", "fuzzy"):
        with pytest.raises(ValueError):
            w.WorkSource("pdmx", "pdmx:123", level, 0.9, confirmed=True)
    assert not w.WorkSource("pdmx", "pdmx:123", "exact_artist", 0.9).confirmed
    assert w.WorkSource("pdmx", "pdmx:123", "manual", 1.0, confirmed=True).confirmed


def test_work_requires_sha8_title_and_a_source():
    src = w.WorkSource("rtttl", "rtttl:tetris", confirmed=True)
    assert w.Work("0123abcd", "Tetris", (src,)).title == "Tetris"
    for bad in (dict(work_id="XYZ", title="T", sources=(src,)), dict(work_id="0123abcd", title=" ", sources=(src,)),
                dict(work_id="0123abcd", title="T", sources=())):
        with pytest.raises(ValueError):
            w.Work(**bad)
    with pytest.raises(ValueError):
        w.WorkSource("rtttl", "pdmx:1")  # material_id обязан нести свой kind


# ── чистка идентичности ───────────────────────────────────────────────────────────────────────────────────────

def test_category_artist_becomes_work_type_and_artist_disappears():
    i = w.clean_identity(rec("a", "Fire And Ice", "Computer Games"))
    assert (i.title, i.artist, i.work_type) == ("Fire And Ice", "", "game_theme")
    i = w.clean_identity(rec("b", "Cheers", "Films And Tv", tags=["tv"]))  # категория типа не даёт — тег даёт
    assert (i.artist, i.work_type) == ("", "tv_theme")


def test_theme_title_takes_the_name_from_artist():
    i = w.clean_identity(rec("theme", "Theme", "20th Century Fox", tags=["tv"]))
    assert (i.title, i.artist, i.work_type, i.named) == ("20th Century Fox", "", "tv_theme", True)
    assert w.clean_identity(rec("theme_9", "Theme v2.0", "Adams Family")).title == "Adams Family"


def test_artist_that_is_a_theme_name_is_not_a_person():
    i = w.clean_identity(rec("batman", "Batman", "Batman Theme"))
    assert (i.title, i.artist) == ("Batman", "")


def test_real_artist_and_title_are_kept():
    i = w.clean_identity(rec("x", "Lose Yourself", "Eminem"))
    assert (i.title, i.artist, i.work_type) == ("Lose Yourself", "Eminem", "")


def test_record_without_any_name_is_unnamed_and_stays_alone():
    a, b = rec("unk1", "Unknown"), rec("unk2", "Unknown")
    assert not w.clean_identity(a).named and w.work_key(a) != w.work_key(b)
    assert len(w.build_works([a, b])) == 2


# ── версии, алиасы ────────────────────────────────────────────────────────────────────────────────────────────

def test_versions_of_one_piece_make_one_work_with_consensus_canon_first():
    records = [rec("lone", "Terminator", "Theme", rtttl=LONE), rec("v1", "Terminator", "Theme", rtttl=UP),
               rec("v2", "Terminator", "Theme", rtttl=UP_TRANSPOSED), rec("other", "Tetris")]
    built = w.build_works(records)
    assert len(built) == 2
    term = built[0]
    assert [s.material_id for s in term.sources] == ["rtttl:v1", "rtttl:v2", "rtttl:lone"]
    assert all(s.confirmed and s.link_level == "exact" for s in term.sources)


def test_work_id_is_stable_and_depends_on_title_and_artist():
    assert w.work_id_of(w.work_key(rec("a", "Tetris"))) == w.work_id_of(w.work_key(rec("zz", "TETRIS")))
    assert w.work_key(rec("a", "Dreams", "Corrs")) != w.work_key(rec("b", "Dreams", "Cranberries"))


def test_ru_aliases_attach_to_works_whose_words_contain_the_query():
    mario, tetris = w.build_works([rec("m", "Super Mario Bros"), rec("t", "Tetris")])
    assert "марио" in mario.aliases and "супер марио" in mario.aliases and "тетрис" not in mario.aliases
    assert "тетрис" in tetris.aliases and "коробейники" in tetris.aliases
    assert not any(a == "russian" for work in (mario, tetris) for a in work.aliases)  # обходной алиас — не фраза


def test_alias_pairs_longest_first_and_one_ru_phrase_per_query():
    pairs = w.alias_pairs()
    assert len(pairs) == 23 and ("гимн ссср", "soviet anthem") in pairs  # семена из целых слов (#3493)
    assert [len(p) for p, _q in pairs] == sorted((len(p) for p, _q in pairs), reverse=True)
    ru = w.ru_phrase_by_query()
    assert ru["soviet anthem"] == "гимн ссср" and "russia" not in ru and ru["tetris"] == "тетрис"


def test_stop_list_marks_living_rights_holders():
    zimmer, bach = w.build_works([rec("i", "Interstellar", "Hans Zimmer"), rec("b", "Air", "J. S. Bach")])
    assert zimmer.stop_listed and not bach.stop_listed


# ── гейт и дыры ───────────────────────────────────────────────────────────────────────────────────────────────

def _registry():
    records = [rec("a", "Cheers", "Films And Tv", tags=["tv"]), rec("b", "Fire", "Computer Games", tags=["game"]),
               rec("c", "Hit", "Somebody", tags=["tv"]), rec("d", "Plain", "Nobody")]
    return records, w.build_works(records)


def test_holes_lists_works_without_the_field_inside_the_working_set():
    records, built = _registry()
    working = w.working_ids(records)
    assert len(working) == 3  # у «Plain» нет ни рабочего тега, ни хука
    plain = next(x for x in built if x.title == "Plain").work_id
    assert plain in w.holes(built, "genre") and plain not in w.holes(built, "genre", working)
    assert len(w.holes(built, "year", working)) == 3  # год не заполнен нигде


def test_gate_is_closed_without_a_consumer_even_with_a_big_hole():
    records, built = _registry()
    g = w.gate(built, "year", w.working_ids(records))
    assert g.share == 1.0 and g.verdict.startswith("closed: нет потребителя")


def test_gate_with_consumer_needs_20_percent_and_still_asks_for_a_pilot(monkeypatch):
    records, built = _registry()
    working = w.working_ids(records)
    monkeypatch.setitem(w.FIELD_CONSUMERS, "genre", ("search.py:1",))
    assert w.gate(built, "genre", working).verdict.startswith("closed: дыра")  # 0 дыр из 3
    monkeypatch.setitem(w.FIELD_CONSUMERS, "year", ("search.py:2",))
    assert w.gate(built, "year", working).verdict.startswith("candidate: нужен пилот")


def test_field_must_be_known():
    _records, built = _registry()
    with pytest.raises(ValueError):
        w.holes(built, "tempo")


# ── SQLite ────────────────────────────────────────────────────────────────────────────────────────────────────

def test_registry_is_written_to_the_same_sqlite_idempotently():
    records, built = _registry()
    conn = sqlite3.connect(":memory:")
    assert w.write_registry(conn, built) == 4
    assert w.write_registry(conn, built) == 4  # повтор не множит строки
    assert conn.execute("SELECT COUNT(*) FROM works").fetchone()[0] == 4
    assert conn.execute("SELECT COUNT(*) FROM work_sources").fetchone()[0] == 4
    assert conn.execute("SELECT value, source FROM work_facts WHERE field='work_type' ORDER BY value").fetchall() == [
        ("game_theme", "rtttl"), ("tv_theme", "rtttl"), ("tv_theme", "rtttl")]
    assert conn.execute("SELECT COUNT(*) FROM work_facts WHERE field='year'").fetchone()[0] == 0


def test_report_names_every_field_and_says_no_network():
    records, built = _registry()
    text = w.report(records, built)
    assert all(f in text for f in w.FIELDS) and "запросов в сеть: 0" in text


# ── score_index (ADR-0154 §3.2, M6) ───────────────────────────────────────────────────────────────────────────

def _score_row(mid="pdmx:QmA", license="cc-zero", title="Cheers", **over):
    return w.ScoreIndexRow(material_id=mid, title=title, composer="", meter="4/4", key="C major", bars=8,
                           license=license, phrase_count=2, file=mid.replace(":", "_") + ".json", **over)


def test_score_index_takes_only_usable_licenses_and_names_each_rejection():
    """M6: пустая лицензия, unknown и конфликт в индекс не попадают; причина названа."""
    conn = sqlite3.connect(":memory:")
    rows = [_score_row("pdmx:good1"), _score_row("pdmx:empty", license=""), _score_row("pdmx:unk", license="unknown"),
            _score_row("pdmx:nan", license="NA"), _score_row("pdmx:conf", license="license_conflict"),
            _score_row("pdmx:good2", license="publicdomain", rating=4.5)]
    written, rejected = w.write_score_index(conn, rows)
    assert written == 2 and [m for m, _why in rejected] == ["pdmx:empty", "pdmx:unk", "pdmx:nan", "pdmx:conf"]
    assert all("лицензия" in why for _m, why in rejected)
    assert [r[0] for r in conn.execute("SELECT material_id FROM score_index ORDER BY 1")] == ["pdmx:good1", "pdmx:good2"]


def test_score_index_row_is_built_from_material_with_the_most_repeated_phrase_as_hook():
    from rob_box_music import material as mt
    from rob_box_music.model import Key, PitchEvent
    m = mt.ScoreMaterial("pdmx:QmB", "Тема", "Автор", "PDMX QmB", "publicdomain", (3, 4), 96, Key(9, "minor"),
                         (PitchEvent(69, 0.0, 1.0, 3),),
                         phrases=(mt.Phrase(0, 4, "new", 1), mt.Phrase(4, 4, "repeat", 3), mt.Phrase(8, 4, "repeat", 3)),
                         stats=mt.MaterialStats(rating=4.0, n_views=7, complexity=2))
    row = w.score_index_row(m, bars=12, file="pdmx_QmB.json", genres="folk", n_ratings=5, keysig="0 sharps")
    assert (row.meter, row.key, row.bpm, row.phrase_count, row.hook_phrase_bar) == ("3/4", "A minor", 96, 3, 4)
    assert (row.rating, row.n_ratings, row.n_views, row.keysig, row.license) == (4.0, 5, 7, "0 sharps", "publicdomain")


def test_score_index_is_idempotent_and_upserts():
    conn = sqlite3.connect(":memory:")
    w.write_score_index(conn, [_score_row(rating=3.0)])
    w.write_score_index(conn, [_score_row(rating=4.0)])
    assert conn.execute("SELECT COUNT(*), MAX(rating) FROM score_index").fetchone() == (1, 4.0)


def test_scores_are_linked_to_works_as_unconfirmed_proposals_and_survive_registry_rebuild():
    records, built = _registry()
    conn = sqlite3.connect(":memory:")
    w.write_registry(conn, built)
    w.write_score_index(conn, [_score_row("pdmx:QmA", title="Cheers"), _score_row("pdmx:QmZ", title="Нет такой")])
    rows = conn.execute("SELECT material_id, kind, link_level, confirmed FROM work_sources "
                        "WHERE kind='pdmx'").fetchall()
    assert rows == [("pdmx:QmA", "pdmx", "exact", 0)]  # только по названию; «Нет такой» — без произведения
    w.write_registry(conn, built)  # перестройка реестра стирает work_sources и подвязывает партитуры заново
    assert conn.execute("SELECT COUNT(*) FROM work_sources WHERE kind='pdmx'").fetchone()[0] == 1
    assert conn.execute("SELECT COUNT(*) FROM work_sources WHERE kind='rtttl'").fetchone()[0] == 4
