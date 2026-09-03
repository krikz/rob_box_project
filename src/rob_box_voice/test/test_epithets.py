#!/usr/bin/env python3
"""test_epithets.py — эпитеты спикеров (issue #1787).

Проверяет два слоя:
- ``core/epithets.py`` — чистая логика «текст → теги → кличка»;
- ``utils/speaker_embeddings.py`` — миграция схемы и хранение эпитета.

Оба модуля грузятся по пути файла, минуя ``utils/__init__.py`` (он тянет
pyaudio через audio_utils) — тот же приём, что в ``test_speaker_embeddings``,
чтобы тест шёл в CI без тяжёлых зависимостей.
"""

from __future__ import annotations

import importlib.util
import sqlite3
import sys
from pathlib import Path

import numpy as np
import pytest

_PKG_ROOT = Path(__file__).resolve().parents[1] / "rob_box_voice"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    # Регистрируем ДО exec_module: @dataclass читает sys.modules[__module__]
    # при разборе аннотаций и падает на AttributeError, если модуля там нет.
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


ep = _load("rob_box_voice.core.epithets", _PKG_ROOT / "core" / "epithets.py")
_se = _load(
    "rob_box_voice.utils.speaker_embeddings",
    _PKG_ROOT / "utils" / "speaker_embeddings.py",
)
SpeakerDatabase = _se.SpeakerDatabase


def _random_embedding(seed: int = 0, dim: int = 256) -> np.ndarray:
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(dim).astype(np.float32)
    return v / np.linalg.norm(v)


@pytest.fixture()
def db(tmp_path):
    database = SpeakerDatabase(str(tmp_path / "speakers.db"))
    yield database
    database.close()


# ── Словарь ──────────────────────────────────────────────────────────────────


def test_lexicon_has_no_duplicate_epithets():
    """Кличка обязана быть уникальной ГЛОБАЛЬНО, а не внутри кластера.

    Иначе два спикера из разных кластеров получили бы одинаковую метку и
    эпитет перестал бы решать задачу, ради которой заведён.
    """
    pool = [e for group in ep.EPITHET_LEXICON.values() for e in group]
    pool += list(ep.DEFAULT_POOL_NEUTRAL) + list(ep.DEFAULT_POOL_RESTLESS)
    duplicates = {e for e in pool if pool.count(e) > 1}
    assert not duplicates, f"дубли в словаре: {duplicates}"
    assert len(pool) >= 200, f"словарь мал: {len(pool)} < 200 (research §4.1)"


def test_every_keyword_cluster_has_epithets():
    """Кластер без эпитетов молча деградирует в дефолтный пул."""
    assert set(ep.CLUSTER_KEYWORDS) == set(ep.EPITHET_LEXICON)


# ── Теги и валентность ───────────────────────────────────────────────────────


def test_extract_tags_finds_dominant_topic():
    tags = ep.extract_tags(
        [
            "обожаю шахматы, особенно ферзевый гамбит",
            "вчера разобрал эндшпиль и пару дебютов на доске",
        ]
    )
    assert tags
    assert tags[0].cluster == "шахматы"
    assert tags[0].share == pytest.approx(1.0)


def test_extract_tags_ignores_too_short_input():
    """Одно слово — не тема. Порог MIN_WORDS_FOR_TAGS защищает от случайности."""
    assert ep.extract_tags(["шахматы"]) == []


def test_extract_tags_empty_when_no_keywords():
    assert ep.extract_tags(["ну не знаю даже что тебе на это сказать сегодня"]) == []


def test_score_sentiment_range():
    assert ep.score_sentiment(["всё бесит, устал, надоело"]) < 0
    assert ep.score_sentiment(["спасибо, отлично, очень нравится"]) > 0
    assert ep.score_sentiment(["стол стул окно"]) == 0.0
    assert ep.score_sentiment([]) == 0.0


# ── Выбор клички ─────────────────────────────────────────────────────────────


def test_epithet_is_deterministic_across_calls():
    """Одинаковый speaker_id → одинаковая кличка (crc32, не hash())."""
    tags = ep.extract_tags(["паяльник и микросхема, транзистор, прошивка платы"])
    first = ep.choose_epithet(tags, speaker_id="abc-123")
    second = ep.choose_epithet(tags, speaker_id="abc-123")
    assert first.label == second.label


def test_two_namesakes_same_topic_get_different_epithets():
    """Главный сценарий issue #1787: «два Дениса-шахматиста».

    Именно на нём research (§4.2) ломал чисто словарный подход. Здесь он
    закрыт передачей ``taken`` — коллизия невозможна по построению.
    """
    tags = ep.extract_tags(
        ["шахматы, дебют, эндшпиль", "ферзь, ладья, рокировка и гамбит"]
    )
    first = ep.choose_epithet(tags, speaker_id="denis-1")
    second = ep.choose_epithet(tags, speaker_id="denis-2", taken=[first.label])
    assert first.label != second.label
    assert first.source_cluster == second.source_cluster == "шахматы"


def test_epithet_falls_back_to_default_pool_without_topics():
    """Молчун без тем всё равно получает кличку сразу (research §5.1)."""
    candidate = ep.choose_epithet([], speaker_id="quiet")
    assert candidate.source_cluster == "default"
    assert candidate.label in ep.DEFAULT_POOL_NEUTRAL


def test_restless_sentiment_switches_default_pool():
    candidate = ep.choose_epithet([], speaker_id="quiet", sentiment=-0.9)
    assert candidate.label in ep.DEFAULT_POOL_RESTLESS


def test_exhausted_pool_gets_numeric_suffix():
    """Когда занято всё — уникальность важнее красоты."""
    taken = [e for group in ep.EPITHET_LEXICON.values() for e in group]
    taken += list(ep.DEFAULT_POOL_NEUTRAL) + list(ep.DEFAULT_POOL_RESTLESS)
    candidate = ep.choose_epithet([], speaker_id="last-one", taken=taken)
    assert candidate.label.endswith("-2")
    assert candidate.label not in taken


# ── Пересмотр ────────────────────────────────────────────────────────────────


def test_should_review_respects_interval():
    day = ep.SECONDS_PER_DAY
    assert ep.should_review(None, 1000.0) is True
    assert ep.should_review(0.0, 10 * day) is False
    assert ep.should_review(0.0, 31 * day) is True


def test_find_distinctive_topic_only_for_new_dominant_theme():
    tags = ep.extract_tags(
        ["шахматы, дебют, эндшпиль", "ферзь, ладья, рокировка и гамбит"]
    )
    # Та же тема, что уже в профиле → повода менять кличку нет.
    assert ep.find_distinctive_topic(["шахматы"], tags) is None
    # Новая доминирующая тема → повод есть.
    assert ep.find_distinctive_topic(["техно"], tags) == "шахматы"


def test_find_distinctive_topic_ignores_weak_theme():
    weak = [ep.TagScore(cluster="спорт", hits=1, share=0.1)]
    assert ep.find_distinctive_topic([], weak) is None


# ── Хранение ─────────────────────────────────────────────────────────────────


def test_migration_adds_columns_to_legacy_db(tmp_path):
    """Старая БД (42 профиля на роботе) досоздаёт колонки без backfill."""
    path = tmp_path / "legacy.db"
    conn = sqlite3.connect(str(path))
    conn.executescript(
        "CREATE TABLE speakers (speaker_id TEXT PRIMARY KEY, name TEXT NOT NULL,"
        " created_at REAL NOT NULL);"
        "CREATE TABLE embeddings (id INTEGER PRIMARY KEY AUTOINCREMENT,"
        " speaker_id TEXT NOT NULL, embedding BLOB NOT NULL, created_at REAL"
        " NOT NULL);"
    )
    conn.execute(
        "INSERT INTO speakers VALUES ('old-1', 'Денчик', 1000.0)"
    )
    conn.commit()
    conn.close()

    database = SpeakerDatabase(str(path))
    try:
        columns = {
            row[1]
            for row in database._conn.execute("PRAGMA table_info(speakers)")
        }
        assert {"epithet", "epithet_history", "tags", "sentiment_score"} <= columns
        profile = database.get_speaker_profile("old-1")
        assert profile["name"] == "Денчик"
        assert profile["epithet"] is None  # backfill не нужен (research §5.5)
    finally:
        database.close()


def test_migration_is_idempotent(tmp_path):
    """Второй старт ноды на той же БД не должен падать на ADD COLUMN."""
    path = str(tmp_path / "twice.db")
    first = SpeakerDatabase(path)
    first.close()
    second = SpeakerDatabase(path)
    second.close()


def test_set_epithet_writes_history(db):
    sid = db.register("Денчик", _random_embedding(1))
    assert db.set_epithet(sid, "Гроссмейстер", ep.REASON_FIRST_SEEN) is True
    assert db.set_epithet(sid, "Дровосек", "new_topic:природа") is True

    profile = db.get_speaker_profile(sid)
    assert profile["epithet"] == "Дровосек"
    history = profile["epithet_history"]
    assert [h["new"] for h in history] == ["Гроссмейстер", "Дровосек"]
    assert history[0]["old"] is None
    assert history[1]["old"] == "Гроссмейстер"
    assert history[1]["reason"] == "new_topic:природа"


def test_set_epithet_is_idempotent_for_same_label(db):
    """Повтор той же клички не плодит записи, но двигает таймер пересмотра."""
    sid = db.register("Саша", _random_embedding(2))
    db.set_epithet(sid, "Конфуций", ep.REASON_FIRST_SEEN)
    before = db.get_speaker_profile(sid)["last_epithet_review"]
    assert db.set_epithet(sid, "Конфуций", ep.REASON_FIRST_SEEN) is True
    after = db.get_speaker_profile(sid)
    assert len(after["epithet_history"]) == 1
    assert after["last_epithet_review"] >= before


def test_set_epithet_rejects_unknown_speaker_and_empty_label(db):
    sid = db.register("Иван", _random_embedding(3))
    assert db.set_epithet("no-such-id", "Барон", "x") is False
    assert db.set_epithet(sid, "   ", "x") is False


def test_taken_epithets_excludes_self(db):
    a = db.register("Денис", _random_embedding(4))
    b = db.register("Денис", _random_embedding(5))
    db.set_epithet(a, "Гроссмейстер", ep.REASON_FIRST_SEEN)
    db.set_epithet(b, "Электрик", ep.REASON_FIRST_SEEN)

    assert sorted(db.taken_epithets()) == ["Гроссмейстер", "Электрик"]
    assert db.taken_epithets(exclude_speaker_id=a) == ["Электрик"]


def test_identify_returns_epithet(db):
    emb = _random_embedding(6)
    sid = db.register("Борис", emb)
    db.set_epithet(sid, "Дровосек", ep.REASON_FIRST_SEEN)

    match = db.identify(emb)
    assert match is not None
    assert match.epithet == "Дровосек"


def test_identify_epithet_is_none_before_assignment(db):
    emb = _random_embedding(7)
    db.register("Аноним", emb)
    match = db.identify(emb)
    assert match is not None
    assert match.epithet is None


def test_update_speaker_stats(db):
    sid = db.register("Женя", _random_embedding(8))
    assert db.update_speaker_stats(sid, tags=["техно", "код"], sentiment_score=0.4)
    profile = db.get_speaker_profile(sid)
    assert profile["tags"] == ["техно", "код"]
    assert profile["sentiment_score"] == pytest.approx(0.4)
    # Пустой вызов не ходит в БД.
    assert db.update_speaker_stats(sid) is False


def test_list_speakers_exposes_epithet(db):
    sid = db.register("Денчик", _random_embedding(9))
    db.set_epithet(sid, "Гроссмейстер", ep.REASON_FIRST_SEEN)
    db.update_speaker_stats(sid, tags=["шахматы"])
    row = next(r for r in db.list_speakers() if r["id"] == sid)
    assert row["epithet"] == "Гроссмейстер"
    assert row["tags"] == ["шахматы"]


def test_broken_history_json_does_not_crash(db):
    """История — диагностика; битый JSON не должен ронять профиль."""
    sid = db.register("Кто-то", _random_embedding(10))
    db._conn.execute(
        "UPDATE speakers SET epithet_history='{не json' WHERE speaker_id=?", (sid,)
    )
    db._conn.commit()
    assert db.get_speaker_profile(sid)["epithet_history"] == []
    # И следующая запись эпитета проходит нормально, начиная историю заново.
    assert db.set_epithet(sid, "Странник", ep.REASON_FIRST_SEEN) is True
    assert len(db.get_speaker_profile(sid)["epithet_history"]) == 1


def test_end_to_end_two_namesakes_stay_distinguishable(db):
    """Сквозной сценарий issue #1787 на уровне БД.

    Два «Дениса» с разными голосами и разными темами: имена в БД
    одинаковые, эпитеты — разные, и identify возвращает правильный.
    """
    emb_a, emb_b = _random_embedding(11), _random_embedding(12)
    a = db.register("Денис", emb_a)
    b = db.register("Денис", emb_b)

    chess = ep.extract_tags(["шахматы, дебют, эндшпиль, ферзь, ладья, гамбит"])
    tech = ep.extract_tags(["паяльник, микросхема, транзистор, прошивка, плата"])

    cand_a = ep.choose_epithet(chess, speaker_id=a, taken=db.taken_epithets(a))
    db.set_epithet(a, cand_a.label, ep.REASON_FIRST_SEEN)
    cand_b = ep.choose_epithet(tech, speaker_id=b, taken=db.taken_epithets(b))
    db.set_epithet(b, cand_b.label, ep.REASON_FIRST_SEEN)

    assert cand_a.label != cand_b.label
    assert db.identify(emb_a).epithet == cand_a.label
    assert db.identify(emb_b).epithet == cand_b.label
    # Имена при этом совпадают — то самое, из-за чего заводился эпитет.
    assert db.identify(emb_a).name == db.identify(emb_b).name == "Денис"


# ── Слой 2: кличка от LLM ────────────────────────────────────────────────────


def test_llm_prompt_mentions_topic_and_replies():
    prompt = ep.build_llm_prompt(
        ["разбирал эндшпиль", "люблю дебюты"],
        fallback="Гроссмейстер",
        cluster="шахматы",
        hints=["Гроссмейстер", "Магистр", "Кадет"],
    )
    assert "шахматы" in prompt
    assert "эндшпиль" in prompt
    assert "ОДНО слово" in prompt
    # Кличка не для ушей юзера — это должно быть сказано модели прямо.
    assert "НИКОГДА не произносится вслух" in prompt


def test_llm_prompt_survives_empty_history():
    prompt = ep.build_llm_prompt([], fallback="Странник")
    assert "Странник" in prompt


def test_sanitize_llm_epithet_accepts_single_word():
    assert ep.sanitize_llm_epithet("Кулибин") == "Кулибин"
    assert ep.sanitize_llm_epithet("  «Кулибин»  ") == "Кулибин"
    assert ep.sanitize_llm_epithet("Звездочёт-Второй") == "Звездочёт-Второй"


def test_sanitize_llm_epithet_takes_first_word_of_chatty_answer():
    """Модели любят добавить пояснение — берём только слово."""
    assert ep.sanitize_llm_epithet("Кулибин — он всё паяет") == "Кулибин"


def test_sanitize_llm_epithet_rejects_garbage():
    for bad in (
        None,
        "",
        "   ",
        "к",                    # короче LLM_EPITHET_MIN_LEN
        "Сверхдлинноеслововыходящеезаграницы",
        "кулибин",              # с маленькой буквы — не кличка
        "Агент007",             # цифры
        "🤖",                    # эмодзи
        "Не могу придумать",    # первое слово не проходит регулярку
    ):
        assert ep.sanitize_llm_epithet(bad) is None, bad


def test_sanitize_llm_epithet_rejects_taken_label():
    assert ep.sanitize_llm_epithet("Кулибин", taken=["кулибин"]) is None
