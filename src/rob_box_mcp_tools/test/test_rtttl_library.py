"""Tests for ``core.rtttl_library`` — archive migration into SQLite + SQL search."""

from __future__ import annotations

import gzip
import json
from pathlib import Path

from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary


def _make_archive(tmp_path: Path) -> Path:
    records = [
        {
            "name": "nokiatun",
            "title": "Nokia Tune",
            "artist": "",
            "source": "mixed3",
            "tags": ["nokia"],
            "rtttl": "NokiaTun:d=4,o=5,b=225:8e6,8d6,f#,g#,8c#6,8b,d,e,8b,8a,c#,e,2a",
        },
        {
            "name": "mario",
            "title": "Super Mario",
            "artist": "Nintendo",
            "source": "mixed3",
            "tags": ["game"],
            "rtttl": "MarioBro:d=4,o=6,b=80:16g5,32c5,16g.5",
        },
        {
            "name": "tetris",
            "title": "Tetris",
            "artist": "",
            "source": "mixed3",
            "tags": ["game"],
            "rtttl": "Tetris:d=4,o=6,b=80:8f7,16c7,16c_7",
        },
    ]
    path = tmp_path / "melodies.jsonl.gz"
    with gzip.open(path, "wt", encoding="utf-8") as fh:
        for rec in records:
            fh.write(json.dumps(rec) + "\n")
    return path


def _make_lib(tmp_path: Path) -> RtttlLibrary:
    archive = _make_archive(tmp_path)
    db = tmp_path / "melodies.db"
    return RtttlLibrary(db_path=str(db), archive_path=str(archive))


def test_migration_and_total(tmp_path):
    lib = _make_lib(tmp_path)
    assert lib.total() == 3


def test_get(tmp_path):
    lib = _make_lib(tmp_path)
    assert lib.get("mario")["title"] == "Super Mario"
    assert lib.get("MARIO")["title"] == "Super Mario"  # регистронезависимо
    assert lib.get("nokia")["rtttl"].startswith("NokiaTun")
    assert lib.get("nope") is None


def test_get_falls_back_to_title_substring(tmp_path):
    lib = _make_lib(tmp_path)
    # точного имени нет — совпадение по title "Super Mario".
    assert lib.get("super mario")["name"] == "mario"


def test_search_ranked_and_metadata_only(tmp_path):
    lib = _make_lib(tmp_path)
    hits = lib.search("game")
    assert [h["name"] for h in hits] == ["mario", "tetris"]
    assert "rtttl" not in hits[0]  # search отдаёт метаданные, не ноты
    assert lib.search("nintendo", limit=1)[0]["name"] == "mario"


def test_empty_query_returns_nothing(tmp_path):
    lib = _make_lib(tmp_path)
    assert lib.search("") == []
    assert lib.get("") is None


def test_migration_is_idempotent(tmp_path):
    """Повторный init с той же БД не задваивает записи."""
    archive = _make_archive(tmp_path)
    db = str(tmp_path / "melodies.db")
    RtttlLibrary(db_path=db, archive_path=str(archive))
    RtttlLibrary(db_path=db, archive_path=str(archive))
    lib = RtttlLibrary(db_path=db, archive_path=str(archive))
    assert lib.total() == 3


def test_multiword_and_russian_alias(tmp_path):
    """«гимн ссср» / «soviet anthem» / «ussr» должны находить Soviet Hymne."""
    records = [
        {
            "name": "soviethy",
            "title": "Soviet Hymne",
            "artist": "",
            "source": "mixed3",
            "tags": ["anthem"],
            "rtttl": "SovietHy:d=4,o=6,b=225:f,2a_",
        },
        {
            "name": "unknown_115",
            "title": "Unknown",
            "artist": "",
            "source": "mixed3",
            "tags": ["ussr"],
            "rtttl": "x:d=4,o=6,b=100:c",
        },
    ]
    archive = tmp_path / "alias.jsonl.gz"
    with gzip.open(archive, "wt", encoding="utf-8") as fh:
        for rec in records:
            fh.write(json.dumps(rec) + "\n")
    lib = RtttlLibrary(db_path=str(tmp_path / "alias.db"), archive_path=str(archive))

    assert lib.get("soviet anthem")["name"] == "soviethy"  # мультислово → токены
    assert lib.get("гимн ссср")["name"] == "soviethy"      # русский alias
    assert lib.get("ussr")["name"] == "soviethy"           # аббревиатура → не мусор
    assert lib.get("imperial march") is None               # честно None, нет такой


def test_migration_repairs_unknown_title_from_artist(tmp_path):
    """title='Unknown' → artist (там реальное имя, потерянное при сборке архива)."""
    records = [
        {
            "name": "unknown_16",
            "title": "Unknown",
            "artist": "Batman V1.0",
            "source": "mixed3",
            "tags": ["movie"],
            "rtttl": "Unknown:d=4,o=5,b=100:16d#6",
        },
    ]
    archive = tmp_path / "repair.jsonl.gz"
    with gzip.open(archive, "wt", encoding="utf-8") as fh:
        for rec in records:
            fh.write(json.dumps(rec) + "\n")
    lib = RtttlLibrary(db_path=str(tmp_path / "repair.db"), archive_path=str(archive))

    rec = lib.get("batman")
    assert rec is not None
    assert rec["title"] == "Batman V1.0"
    assert rec["rtttl_name"] == "Unknown"  # имя внутри формата RTTTL


def test_rtttl_name_is_searchable(tmp_path):
    """Поиск должен матчить и имя внутри формата RTTTL (префикс до ':')."""
    records = [
        {
            "name": "unknown_x",
            "title": "Unknown",
            "artist": "",
            "source": "mixed3",
            "tags": [],
            "rtttl": "NokiaTun:d=4,o=5,b=225:8e6,8d6",
        },
    ]
    archive = tmp_path / "rtname.jsonl.gz"
    with gzip.open(archive, "wt", encoding="utf-8") as fh:
        for rec in records:
            fh.write(json.dumps(rec) + "\n")
    lib = RtttlLibrary(db_path=str(tmp_path / "rtname.db"), archive_path=str(archive))

    assert lib.get("nokiatun")["rtttl_name"] == "NokiaTun"
    assert lib.search("nokiatun")[0]["rtttl_name"] == "NokiaTun"


def test_multiword_query_prefers_full_title_over_name_token(tmp_path):
    """«happy birthday» → «Happy Birthday To You», а не Ashanti «Happy»."""
    records = [
        {
            "name": "happy",
            "title": "Happy",
            "artist": "Ashanti",
            "source": "mixed3",
            "tags": [],
            "rtttl": "Happy:d=4,o=5,b=100:c",
        },
        {
            "name": "happybir_3",
            "title": "Happy Birthday To You",
            "artist": "Mildred J Hill",
            "source": "mixed3",
            "tags": [],
            "rtttl": "HappyBir:d=8,o=5,b=100:16c,16c,d,c,f,e",
        },
    ]
    archive = tmp_path / "hb.jsonl.gz"
    with gzip.open(archive, "wt", encoding="utf-8") as fh:
        for rec in records:
            fh.write(json.dumps(rec) + "\n")
    lib = RtttlLibrary(db_path=str(tmp_path / "hb.db"), archive_path=str(archive))

    rec = lib.get("happy birthday")
    assert rec is not None
    assert rec["name"] == "happybir_3"


def test_get_russian_anthem_prefers_real_theme_over_garbage_loop(tmp_path):
    """issue #2840: «russian anthem» находил ``russiann`` — 10-нотный цикл

    ``2e,d,c,2d,c,d,2e,g,e,1d`` × 2 в o=7 — вместо полноценной темы
    Александрова (``national_2``). Денилист + скоринг качества должны
    предпочесть содержательную запись зацикленному мусору.
    """
    records = [
        {
            "name": "russiann",
            "title": "Russian National Anthem",
            "artist": "",
            "source": "mixed3",
            "tags": ["anthem", "russia"],
            "rtttl": (
                "RussianN:d=4,o=7,b=125:"
                "2e,d,c,2d,c,d,2e,g,e,1d,2e,d,c,2d,c,d,2e,g,e,1d"
            ),
        },
        {
            "name": "national",
            "title": "National Anthem",
            "artist": "",
            "source": "mixed3",
            "tags": ["anthem"],
            "rtttl": (
                "National:d=4,o=7,b=125:"
                "2e,d,c,2d,c,d,2e,g,e,1d,2e,d,c,2d,c,d,2e,g,e,1d"
            ),
        },
        {
            "name": "national_2",
            "title": "National Anthem Of Russia",
            "artist": "Alexandrov",
            "source": "mixed3",
            "tags": ["anthem", "russia", "soviet"],
            "rtttl": (
                "National:d=8,o=5,b=76:"
                "8g,8p,c6,8g.,16a,b,8e,8e,a,8g,8f,g,8g,16f,16e,8d,8c,d,e,f,"
                "g,a,b,c6,d6,8e6,8d6,c6,8b,a,g,8f,8e,d,c,8d,e,f,g,a,8b,8c6"
            ),
        },
        {
            "name": "unknown_111",
            "title": "National Anthem Of Soviet",
            "artist": "",
            "source": "mixed3",
            "tags": ["anthem", "soviet"],
            "rtttl": "x:d=4,o=6,b=100:c",
        },
    ]
    archive = tmp_path / "anthem.jsonl.gz"
    with gzip.open(archive, "wt", encoding="utf-8") as fh:
        for rec in records:
            fh.write(json.dumps(rec) + "\n")
    db = tmp_path / "anthem.db"
    lib = RtttlLibrary(db_path=str(db), archive_path=str(archive))

    rec = lib.get("russian anthem")
    assert rec is not None
    assert rec["name"] == "national_2"

    # Прямая адресация по имени мусорной записи — по-прежнему честно находит
    # её саму (денилист понижает в ранжировании, но не прячет запись).
    assert lib.get("russiann")["name"] == "russiann"


def test_real_archive_russian_anthem_picks_national_2(tmp_path):
    """Регрессия живого прогона 23.09 (issue #2840): первая версия фикса на
    синтетическом архиве была зелёной, но на РЕАЛЬНОМ архиве блендинг
    качества с текстовым скором подсовывал случайную «Irish National
    Anthem» — «russian» вообще не матчил ни одну запись (архив хранит
    русский гимн под «Russia», не «Russian»), и решало голое совпадение
    «anthem» по полусотне чужих гимнов, где качество мелодии — не смысл —
    выбирало победителя. ``RtttlLibrary()`` без ``archive_path`` грузит
    настоящий бандл ``data/rtttl_melodies.jsonl.gz`` (10461 запись)."""
    lib = RtttlLibrary(db_path=str(tmp_path / "real.db"))
    assert lib.total() > 10000

    rec = lib.get("russian anthem")
    assert rec is not None
    assert rec["name"] == "national_2"


def test_real_archive_direct_name_addressing_survives_alias(tmp_path):
    """Регрессия внутри регрессии (живой прогон 23.09): чтобы «russian
    anthem» матчил ``national_2`` по смыслу, добавлен алиас
    «russian»→«russia» (архив хранит гимн под существительным). Первая
    версия этого алиаса подменяла подстроку ГОЛОСОМ (``str.replace``), и
    ``get('russiann')`` превращался в ``get('russian')`` — прямая
    адресация по имени денилист-записи находила ЧУЖУЮ песню («Russians»).
    Замена обязана идти по границе слова (``\\b``)."""
    lib = RtttlLibrary(db_path=str(tmp_path / "real3.db"))
    rec = lib.get("russiann")
    assert rec is not None
    assert rec["name"] == "russiann"


def test_real_archive_weak_match_table(tmp_path):
    """issue #2877: живой прогон 23.09 — диджей объявил «Stranger Things»,
    сыграл «Strangers In The Night» (``get('stranger things')`` находил
    ``stranger_2`` — «stranger» матчил «strangers» подстрокой, «things» не
    встречался в записи вовсе). Таблица строгих совпадений (должны
    остаться НЕТРОНУТЫМИ фиксом) вперемешку со слабыми (должны стать
    ``None``), все — на РЕАЛЬНОМ архиве (``RtttlLibrary()`` без
    ``archive_path``, тот же бандл ``data/rtttl_melodies.jsonl.gz``)."""
    lib = RtttlLibrary(db_path=str(tmp_path / "weak_match.db"))
    assert lib.total() > 10000

    # Строгие совпадения — issue #2840, не должны сдвинуться этим фиксом.
    expected_names = {
        "russian anthem": "national_2",
        "still dre": "stilldre_2",
        "in the hall of the mountain king": "hallofth_2",
        "hall of the mountain king": "hallofth_2",
        "mario": "supermar_4",
        "terminator": "terminat",
        "next episode": "nextepis_3",
        "гимн россии": "unknown_111",
        "soviet anthem": "unknown_111",
    }
    for query, expected_name in expected_names.items():
        rec = lib.get(query)
        assert rec is not None, f"{query!r} обязан резолвиться (было {expected_name!r})"
        assert rec["name"] == expected_name, (query, rec)

    # Слабые совпадения — issue #2877: честное None вместо ближайшего
    # чужого трека под заявленным названием.
    assert lib.get("stranger things") is None, (
        "get('stranger things') не должен подсовывать 'Strangers In The "
        "Night' (stranger_2) — темы Stranger Things в архиве нет"
    )
    assert lib.get("all the things") is None, (
        "get('all the things') не должен подсовывать 'All The Things She "
        "Said' — это другая песня, а не точное совпадение"
    )


def test_real_archive_query_table_stays_on_topic(tmp_path):
    """Таблица запросов из живого прогона 23.09 — каждый должен находить
    тему СВОЕЙ песни (по подстроке в title), а не первую попавшуюся с
    совпадающим общим словом («anthem», «mountain king» и т.п.)."""
    lib = RtttlLibrary(db_path=str(tmp_path / "real2.db"))
    expected_title_substrings = {
        "russian anthem": "russia",
        "гимн россии": "soviet",  # алиас архива — см. _ALIASES
        "soviet anthem": "soviet",
        "in the hall of the mountain king": "mountain king",
        "still dre": "dre",
        "terminator": "terminator",
        "mario": "mario",
    }
    for query, substr in expected_title_substrings.items():
        rec = lib.get(query)
        assert rec is not None, query
        assert substr in (rec["title"] or "").lower(), (query, rec)
