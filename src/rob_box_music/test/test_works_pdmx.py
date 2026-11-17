"""ADR-0155 K-3: произведения реестра ↔ партитуры пака (``works.match_scores``/``link_score_sources``).

Синтетика: индекс партитур — строки как у ``score_index`` пака (PDMX-названия «Название - Автор», композитор,
лицензия, знаки); произведения — ``(work_id, title, artist, composer)``. Правила уровней — замер K-1 (§2.2),
подтверждение — только человеком (В6), непригодная лицензия — отказ (M6 ADR-0154).
"""

from __future__ import annotations

import sqlite3

import pytest

from rob_box_music import works as w

WORKS = [
    ("00000001", "Dreams", "Cranberries", ""),
    ("00000002", "Dreams", "Corrs", ""),
    ("00000003", "Mountain King", "Grieg", ""),
    ("00000004", "Mozart", "Mozart", ""),
    ("00000005", "Super Mario Bros", "", ""),
    ("00000006", "Death Music", "Super Mario Brothers", ""),
    ("00000007", "Imagine", "John Lennon", ""),
]


def row(mid, title, composer="", license="cc-zero", rating=4.0, keysig="G major", key="E minor"):
    return {"material_id": f"pdmx:{mid}", "title": title, "composer": composer, "license": license,
            "rating": rating, "keysig": keysig, "key": key}


def levels(links):
    return sorted((x.work_id, x.source.material_id, x.source.link_level) for x in links)


def test_title_with_author_is_exact_artist_and_namesakes_of_other_artists_get_nothing():
    """«Dreams» Cranberries ↔ Corrs (главный источник ложных по замеру): партитура с совпавшим автором уходит
    своему произведению, одноимённому чужому не предлагается; без автора — ``exact`` обоим."""
    links, rejected = w.match_scores(WORKS, [row("a", "Dreams", "The Cranberries"), row("b", "Dreams")])
    assert rejected == []
    assert levels(links) == [("00000001", "pdmx:a", "exact_artist"), ("00000001", "pdmx:b", "exact"),
                             ("00000002", "pdmx:b", "exact")]


def test_title_before_dash_or_brackets_counts_only_as_described():
    links, _ = w.match_scores(WORKS, [
        row("lennon", "Imagine - John Lennon"),           # « - автор» совпал с исполнителем → exact_artist
        row("other", "Imagine - Some Arranger"),          # « - кто-то» без совпадения: хвост мог быть названием
        row("brackets", "Imagine (piano cover)"),         # до скобок — точное название
        row("grieg", "Mountain King (Peer Gynt)", "Edvard Grieg")])
    assert levels(links) == [("00000003", "pdmx:grieg", "exact_artist"),
                             ("00000007", "pdmx:brackets", "exact"), ("00000007", "pdmx:lennon", "exact_artist")]


def test_record_named_after_its_author_and_author_before_dash_are_not_titles():
    """«Mozart» — Mozart: не пьеса; «Mozart - Concerto K. 191» с автором Mozart — автор, не название."""
    links, _ = w.match_scores(WORKS, [row("k191", "Mozart - Concerto K. 191", "W. A. Mozart"),
                                      row("plain", "Mozart")])
    assert [x for x in links if x.work_id == "00000004"] == []


def test_fuzzy_is_a_capped_low_weight_proposal_and_needs_two_words():
    rows = [row(f"m{i}", f"Super Mario Bros Theme {i}", rating=5.0 - i / 10) for i in range(8)]
    rows.append(row("death", "Death"))  # одно слово — не пьеса без точного совпадения
    links, _ = w.match_scores(WORKS, rows)
    fuzzy = [x for x in links if x.source.link_level == "fuzzy"]
    assert {x.work_id for x in fuzzy} == {"00000005"}
    assert len(fuzzy) == w.FUZZY_PER_WORK and all(x.source.link_score <= 0.5 for x in fuzzy)
    assert [x.source.material_id for x in fuzzy] == [f"pdmx:m{i}" for i in range(w.FUZZY_PER_WORK)]  # по рейтингу


def test_no_link_is_confirmed_automatically_and_fuzzy_cannot_be_confirmed():
    """В6: ни один уровень сопоставления не подтверждается кодом; подтверждение — только ``manual``."""
    links, _ = w.match_scores(WORKS, [row("a", "Dreams", "Cranberries"), row("b", "Dreams"),
                                      row("f", "Super Mario Bros Overworld")])
    assert {x.source.link_level for x in links} == {"exact_artist", "exact", "fuzzy"}
    assert not any(x.source.confirmed for x in links)
    with pytest.raises(ValueError):
        w.WorkSource("pdmx", "pdmx:f", "fuzzy", 0.4, confirmed=True)


def test_license_conflict_and_unknown_are_refused_with_a_reason_not_linked():
    """M6 ADR-0154: партитура без пригодной лицензии произведению не предлагается."""
    links, rejected = w.match_scores(WORKS, [row("conf", "Imagine", license="license_conflict"),
                                             row("unk", "Imagine", license="unknown"), row("ok", "Imagine")])
    assert [x.source.material_id for x in links] == ["pdmx:ok"]
    assert [m for m, _ in rejected] == ["pdmx:conf", "pdmx:unk"] and all("лицензия" in why for _, why in rejected)


def _registry(conn):
    """Реестр в форме до K-3: ``work_sources`` без колонок партитуры (миграция — ALTER TABLE)."""
    conn.executescript(
        "CREATE TABLE works (work_id TEXT PRIMARY KEY, title TEXT NOT NULL, artist TEXT NOT NULL DEFAULT '', "
        "composer TEXT NOT NULL DEFAULT '', aliases TEXT NOT NULL DEFAULT '[]', "
        "stop_listed INTEGER NOT NULL DEFAULT 0);"
        "CREATE TABLE work_sources (work_id TEXT NOT NULL, material_id TEXT NOT NULL, kind TEXT NOT NULL, "
        "link_level TEXT NOT NULL, link_score REAL NOT NULL, confirmed INTEGER NOT NULL, rank INTEGER NOT NULL, "
        "PRIMARY KEY (work_id, material_id));")
    conn.executemany("INSERT INTO works (work_id, title, artist, composer) VALUES (?,?,?,?)", WORKS)
    conn.execute("INSERT INTO work_sources VALUES ('00000007', 'rtttl:imagine', 'rtttl', 'exact', 1.0, 1, 0)")


def test_links_from_an_attached_pack_index_carry_license_rating_key_and_composer(tmp_path):
    pack = sqlite3.connect(tmp_path / "score_index.db")
    w.write_score_index(pack, [w.ScoreIndexRow("pdmx:QmL", "Imagine", "John Lennon", "4/4", "C major", 8, "cc-zero",
                                               2, "pdmx_QmL.json", rating=4.7, keysig="C major")])
    pack.close()
    conn = sqlite3.connect(":memory:")
    _registry(conn)
    conn.execute("ATTACH DATABASE ? AS scores", (str(tmp_path / "score_index.db"),))
    assert w.link_score_sources(conn, "scores") == 1
    assert w.link_score_sources(conn, "scores") == 1  # повтор не множит
    rows = conn.execute("SELECT material_id, link_level, confirmed, rank, license, rating, keysig, key, composer "
                        "FROM work_sources ORDER BY rank").fetchall()
    assert rows == [("rtttl:imagine", "exact", 1, 0, None, None, None, None, None),
                    ("pdmx:QmL", "exact_artist", 0, 1, "cc-zero", 4.7, "C major", "C major", "John Lennon")]
    text = w.score_links_report(conn)
    assert "с партитурой (любой уровень) 1" in text and "exact_artist" in text
    assert "непригодных лицензий (license_conflict, unknown, пусто) среди связанных: 0" in text


def test_registry_without_pack_index_links_nothing():
    conn = sqlite3.connect(":memory:")
    _registry(conn)
    assert w.link_score_sources(conn) == 0
