"""Tests for ``core.rtttl_library`` — archive migration into SQLite + SQL search."""

from __future__ import annotations

import gzip
import json
from pathlib import Path

from rob_box_mcp_tools.core.rtttl_library import (
    RtttlLibrary,
    covers_tokens,
    display_title,
    match_info,
)


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
    """issue #2896 (регрессия #2882→#2877): первая версия фикса на слабые
    совпадения (``_is_weak_match``, «Stranger Things» → «Strangers In The
    Night») системно ломала СИЛЬНЫЕ — ``get('super mario')``/``get('star
    wars')`` возвращали ``None``, хотя архив содержит точные записи
    («Super Mario Brothers 1», «Star Wars - Imperial March …»), просто с
    «лишними» словами в title. Единой эвристики, которая отличает
    «лишние слова — другая песня» от «лишние слова — длинное название»,
    не нашлось, поэтому ``get()`` больше не отказывает молча: он всегда
    возвращает лучшего по тексту кандидата (как до #2882), а прозрачность
    для модели (title + alternatives) обеспечивают вызывающие тулы
    (``ComposeMusicTool``/``LookupMelodyTool``, см. ``tools/music.py``) и
    промпт скилла composer — не сама библиотека. Таблица — на РЕАЛЬНОМ
    архиве (``RtttlLibrary()`` без ``archive_path``, тот же бандл
    ``data/rtttl_melodies.jsonl.gz``)."""
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

    # issue #2896: раньше молча отказывались (regressed None) — теперь
    # ``get()`` обязан найти сильное совпадение, несмотря на «лишние
    # слова» в title («Super Mario Brothers 1», «Star Wars - Imperial
    # March …»).
    strong_regressions = {
        "super mario": "supermar_4",
        "super mario bros": "supermar_4",
        "imperial march": "starwars_4",
        "harry potter": "harrypot_3",
        "nokia tune": "grandeva",
        "star wars": "starwars_4",
    }
    for query, expected_name in strong_regressions.items():
        rec = lib.get(query)
        assert rec is not None, f"{query!r} обязан резолвиться (issue #2896)"
        assert rec["name"] == expected_name, (query, rec)

    # issue #2877 остаётся видимым, но по-другому: get() больше НЕ
    # отвечает честным None на слабое совпадение — он возвращает лучшего
    # кандидата и title, а не подменяет его тишиной. Ответственность
    # «это не та песня» лежит на вызывающей стороне (title + alternatives
    # в tools/music.py, промпт composer.txt), не на самой библиотеке.
    for query in ("stranger things", "all the things"):
        rec = lib.get(query)
        assert rec is not None, (
            f"get({query!r}) обязан вернуть лучшего кандидата с title, "
            "а не молчаливое None (issue #2896) — прозрачность даёт "
            "вызывающая сторона, не сама библиотека"
        )
        assert rec.get("title"), (query, rec)


def test_real_archive_search_finds_full_token_match_first(tmp_path):
    """issue #2941: ``search()`` терял запись, совпадающую по ВСЕМ токенам
    запроса, если частые токены («dre», «anthem», «hot») раздували
    SQL-LIKE кандидатов, а ``LIMIT`` в ``_candidates()`` обрезал выборку ДО
    скоринга — сама запись даже не попадала в Python-скоринг, хотя
    ``get()`` (с ``cap=2000``) её честно находил. Полное совпадение
    («stilldre» из «still dre», слитное написание) обязано быть первым в
    ``search()``, как и в ``get()``."""
    lib = RtttlLibrary(db_path=str(tmp_path / "search_full.db"))
    assert lib.total() > 10000

    hits = lib.search("still dre", limit=5)
    names = [h["name"] for h in hits]
    assert names, "search('still dre') не должен быть пустым"
    assert names[0].startswith("stilldre"), names
    # согласованность: то, что находит get(), search() ставит первым.
    assert lib.get("still dre")["name"] == names[0]

    for query in ("russia anthem", "national anthem of russia"):
        hits = lib.search(query, limit=3)
        names = [h["name"] for h in hits]
        assert "national_2" in names, (query, names)
        # в топ-3, как требует issue #2941
        assert names.index("national_2") < 3, (query, names)

    # next episode — уже работал, не должен сломаться этим фиксом.
    hits = lib.search("next episode", limit=3)
    assert [h["name"] for h in hits][0].startswith("nextepis")

    # "drop it like its hot" — найти реальный ключ в архиве, если есть.
    drop_hit = lib.get("drop it like its hot")
    assert drop_hit is not None
    hits = lib.search("drop it like its hot", limit=5)
    names = [h["name"] for h in hits]
    assert names, "search('drop it like its hot') не должен быть пустым"
    assert names[0] == drop_hit["name"], (names, drop_hit["name"])


def test_search_get_consistency_across_query_table(tmp_path):
    """search()/get() согласованы: что находит get(), search() ставит первым
    (реальный архив, набор запросов из issue #2840/#2896/#2941).

    Запросы, где несколько записей архива честно делят и текстовый скор, И
    качество мелодии между собой (истинная ничья без семантического
    победителя, например «harry potter» — три записи с одинаковым
    ``_melody_quality``) сюда намеренно не включены: и ``get()``
    (:meth:`RtttlLibrary._best_in_bucket`, ``max()`` по порядку прихода
    строк), и ``search()`` (устойчивая сортировка по title) детерминированы
    каждый сам по себе, но тай-брейк для ИСТИННЫХ троек-ничьих — отдельный,
    не связанный с багом #2941 вопрос (какая из формально равных записей
    «правильнее» — не решается текстом/качеством, тут ADR-0132 требует
    ручку/пресет, а не свежий хак под конкретную ничью)."""
    lib = RtttlLibrary(db_path=str(tmp_path / "search_consistency.db"))
    queries = [
        "still dre",
        "russian anthem",
        "russia anthem",
        "soviet anthem",
        "гимн россии",
        "super mario",
        "star wars",
        "imperial march",
        "terminator",
        "next episode",
        "in the hall of the mountain king",
        "happy birthday",
    ]
    for query in queries:
        expected = lib.get(query)
        assert expected is not None, query
        hits = lib.search(query, limit=5)
        names = [h["name"] for h in hits]
        assert names, query
        assert names[0] == expected["name"], (query, names, expected["name"])


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


def test_covers_tokens_rejects_gin_and_juice_found_everybodys_changing(tmp_path):
    """issue #2964: живой прогон 24.09 — ``lookup_melody('Gin and Juice')``
    находил «Everybody's Changing» (Keane) — ни один значимый токен запроса
    не встречается в записи, но ``get()`` всё равно честно (по тексту)
    отдаёт лучшего кандидата (issue #2896). ``covers_tokens()`` обязана
    механически отличить это от настоящего совпадения — без объявления
    оригинала."""
    lib = RtttlLibrary(db_path=str(tmp_path / "gin.db"))
    rec = lib.get("Gin and Juice")
    assert rec is not None
    assert rec["name"] == "everybod_8"
    assert rec["title"] == "Everybody's Changing"
    assert covers_tokens(rec, "Gin and Juice") is False


def test_covers_tokens_rejects_nuthin_but_a_g_thang_found_poppin_them_thangs(tmp_path):
    """issue #2964: ``lookup_melody('Nuthin But A G Thang')`` находил «If I
    Can Poppin Them Thangs» (50 Cent) — «nuthin»/«g» не покрыты, «thang» —
    просто общая подстрока «thangs». compose_music играл эту запись под
    видом G Thang молча."""
    lib = RtttlLibrary(db_path=str(tmp_path / "gthang.db"))
    rec = lib.get("Nuthin But A G Thang")
    assert rec is not None
    assert rec["name"] == "ificanpo"
    assert rec["title"] == "If I Can Poppin Them Thangs"
    assert covers_tokens(rec, "Nuthin But A G Thang") is False


def test_covers_tokens_rejects_stranger_things_found_strangers_in_the_night(tmp_path):
    """issue #2964 (комментарий от 24.09, сет «80s analog horror»):
    ``compose_music(name='stranger things')`` сыграл «Strangers In The
    Night» (Sinatra) под видом «Очень странных дел» — «stranger» покрыт
    подстрокой «strangers», но значимый токен «things» в записи не
    встречается вовсе. Совпадение окончания слова не должно засчитываться
    как честное совпадение темы."""
    lib = RtttlLibrary(db_path=str(tmp_path / "stranger.db"))
    rec = lib.get("stranger things")
    assert rec is not None
    assert rec["name"] == "stranger_2"
    assert rec["title"] == "Strangers In The Night"
    assert covers_tokens(rec, "stranger things") is False


def test_covers_tokens_keeps_terminator_theme_honest_via_artist(tmp_path):
    """issue #2964 (внимание из тикета): ``theme_178`` — title «Theme»,
    artist «Terminatorv v2.0» — правильная тема Терминатора. ``covers_
    tokens()`` обязана учитывать artist, а не только title, иначе честный
    критерий сломал бы этот РАБОЧИЙ кейс."""
    lib = RtttlLibrary(db_path=str(tmp_path / "terminator.db"))
    rec = lib.get("terminator theme")
    assert rec is not None
    assert rec["name"] == "theme_178"
    assert rec["title"] == "Theme"
    assert rec["artist"] == "Terminatorv v2.0"
    assert covers_tokens(rec, "terminator theme") is True

    rec2 = lib.get("terminator")
    assert rec2 is not None
    assert rec2["name"] == "terminat"
    assert covers_tokens(rec2, "terminator") is True


def test_covers_tokens_synthetic_field_rules():
    """Юнит-уровень (без архива): какие поля учитываются, а какие — нет."""
    record = {
        "name": "mario",
        "title": "Super Mario Brothers 1",
        "artist": "Nintendo",
        "tags": ["game", "juice"],
        "rtttl_name": "MarioBro",
    }
    # Все токены покрыты (частично title, частично name) — честное совпадение.
    assert covers_tokens(record, "super mario") is True
    # tags намеренно не участвуют — «juice» есть только в tags, не в
    # title/artist/name/rtttl_name, значит НЕ засчитывается.
    assert covers_tokens(record, "mario juice") is False
    # Пустой/бессмысленный запрос — не «покрыт» (нет значимых токенов).
    assert covers_tokens(record, "") is False
    assert covers_tokens(record, "the of") is False  # только стоп-слова


# ---------------------------------------------------------------------------
# match_info / display_title / token_weights — issue #2964 (правка товарища
# Шифу): БЕЗ жёсткого порога found=False и БЕЗ хардкод-списка стоп-слов
# («theme»/«main»/«soundtrack»/…). Вес токена — IDF по самому корпусу
# архива; частый токен сам весит около нуля. Решение «это та же песня» —
# у модели (промпт скилла composer), тул отдаёт честную структурированную
# сверку.
# ---------------------------------------------------------------------------


def test_token_weight_discounts_frequent_theme_without_a_stopword_list(tmp_path):
    """«theme» встречается в сотнях записей архива → вес почти нулевой сам
    по себе (IDF), «terminator» — редкий токен → вес высокий. Ни одно слово
    не в списке — это чистая статистика корпуса."""
    lib = RtttlLibrary(db_path=str(tmp_path / "weights.db"))
    weights = lib.token_weights()
    theme_weight = weights("theme")
    terminator_weight = weights("terminator")
    assert theme_weight < terminator_weight
    # «theme» — не буквально ноль, но кратно меньше редкого токена.
    assert theme_weight < terminator_weight / 2


def test_match_info_stranger_things_theme_flags_phantom_of_the_opera_confusion(tmp_path):
    """Живой случай из комментария к issue #2964: ``compose_music(name=
    'stranger things theme')`` сыграл ``theme_136`` — «Призрак оперы»
    (title «Theme», artist «Phantom Of The Opera»), а не «Очень странных
    дел». «theme» покрыт (и почти не весит), но значимые «stranger» и
    «things» не совпадают вовсе — match_info честно это показывает
    (низкое coverage), без отдельной ветки кода под этот кейс."""
    lib = RtttlLibrary(db_path=str(tmp_path / "stranger_theme.db"))
    rec = lib.get("stranger things theme")
    assert rec is not None
    assert rec["name"] == "theme_136"
    assert rec["artist"] == "Phantom Of The Opera"
    mi = match_info(lib, rec, "stranger things theme")
    assert mi["matched"] == ["theme"]
    assert set(mi["unmatched"]) == {"stranger", "things"}
    assert mi["coverage"] < 0.3  # «theme» весит мало — низкое покрытие
    # display_title подставляет исполнителя, раз title сам по себе
    # неинформативен — юзер должен понять, что это НЕ Очень странные дела.
    assert display_title(lib, rec) == "Theme — Phantom Of The Opera"


def test_match_info_terminator_theme_full_coverage_via_artist(tmp_path):
    """Контроль: ``theme_178`` (title «Theme», artist «Terminatorv v2.0»)
    — ПРАВИЛЬНАЯ тема Терминатора (внимание из тикета) — общее правило не
    должно её ломать. Оба значимых токена покрыты (artist + title),
    coverage=1.0."""
    lib = RtttlLibrary(db_path=str(tmp_path / "terminator_theme.db"))
    rec = lib.get("terminator theme")
    assert rec is not None
    assert rec["name"] == "theme_178"
    mi = match_info(lib, rec, "terminator theme")
    assert mi["unmatched"] == []
    assert mi["coverage"] == 1.0
    assert display_title(lib, rec) == "Theme — Terminatorv v2.0"


def test_match_info_self_query_never_false_negatives_on_sample(tmp_path):
    """Системная проверка (не пара-по-паре): если запрос — это ТОЧНО title
    записи архива, ``match_info`` обязан дать coverage≈1.0 — общее
    правило работает не только на кейсах из тикета. Сэмпл, не весь архив
    (10к записей × SQL document_frequency на тест — дорого)."""
    lib = RtttlLibrary(db_path=str(tmp_path / "self_query.db"))
    conn = lib._conn  # noqa: SLF001 — тестовый прямой доступ, тот же паттерн, что выше
    rows = conn.execute(
        "SELECT title FROM rtttl_melodies WHERE title != '' "
        "ORDER BY RANDOM() LIMIT 40"
    ).fetchall()
    low_coverage = []
    for row in rows:
        title = row["title"]
        rec = lib.get(title)
        if rec is None:
            continue
        mi = match_info(lib, rec, title)
        if mi["coverage"] < 0.99:
            low_coverage.append((title, rec["name"], mi))
    assert not low_coverage, low_coverage
