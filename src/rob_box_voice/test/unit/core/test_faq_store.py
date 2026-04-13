from pathlib import Path

import pytest

from rob_box_voice.core.faq_store import FAQStore, _RU_STOPWORDS, _tokenize_fts


# ---------------------------------------------------------------------------
# Unit tests for _tokenize_fts
# ---------------------------------------------------------------------------


def test_tokenize_fts_strips_stopwords() -> None:
    tokens = _tokenize_fts("расскажи про коррупцию")
    # "расскажи" and "про" are stopwords — only "коррупцию" should remain
    joined = " ".join(tokens)
    assert '"расскажи"' not in joined
    assert '"про"' not in joined
    assert '"коррупцию"' in joined or '"коррупци"' in joined


def test_tokenize_fts_adds_morphological_prefix() -> None:
    tokens = _tokenize_fts("коррупцию")
    # Full form preserved
    assert '"коррупцию"*' in tokens
    # Last 2 chars stripped: "коррупцию"[:-2] == "коррупц"
    assert '"коррупц"*' in tokens


def test_tokenize_fts_stem_matches_base_form_in_fts(tmp_path: Path) -> None:
    """The stem prefix emitted by _tokenize_fts must match the base form stored in FAQ."""
    store = FAQStore(db_path=str(tmp_path / "faq.db"))
    store.replace_items(
        event_id="test",
        items=[
            {
                "question": "Есть ли у вас в Академии коррупция?",
                "answer": "Академия придерживается принципов прозрачности.",
                "category": "general",
                "source": "faq.xlsx",
            }
        ],
    )

    # "расскажи про коррупцию" — all words are stopwords except "коррупцию"
    # After stopword filtering + morphological prefix, "коррупци"* should match "коррупция"
    results = store.search("расскажи про коррупцию", event_id="test", limit=3)

    assert len(results) == 1
    assert "коррупция" in results[0]["question"].lower()


def test_tokenize_fts_returns_empty_for_all_stopwords() -> None:
    """_tokenize_fts returns [] when every token is a stopword.
    The caller (_fts_search) is responsible for the raw-split fallback.
    """
    tokens = _tokenize_fts("да нет ну")
    assert tokens == []


def test_tokenize_fts_no_duplicate_tokens() -> None:
    tokens = _tokenize_fts("коррупция коррупция")
    assert len(set(tokens)) == len(tokens)




def test_replace_items_replaces_only_selected_event(tmp_path: Path) -> None:
    db_path = tmp_path / "faq.db"
    store = FAQStore(db_path=str(db_path))

    inserted = store.replace_items(
        event_id="open-day-2026",
        items=[
            {
                "question": "Где проходит день открытых дверей?",
                "answer": "В главном корпусе.",
                "category": "Общие вопросы",
                "source": "faq.xlsx",
            }
        ],
    )
    assert inserted == 1

    store.replace_items(
        event_id="science-fair-2026",
        items=[
            {
                "question": "Где будет выставка проектов?",
                "answer": "В лабораторном корпусе.",
                "category": "Локации",
                "source": "science.xlsx",
            }
        ],
    )
    store.replace_items(
        event_id="open-day-2026",
        items=[
            {
                "question": "Какие направления бакалавриата доступны?",
                "answer": "Программы по управлению и ИТ.",
                "category": "Общие вопросы",
                "source": "faq.xlsx",
            }
        ],
    )

    open_day_results = store.search(
        "направления бакалавриата", event_id="open-day-2026", limit=3
    )
    science_results = store.search(
        "выставка проектов", event_id="science-fair-2026", limit=3
    )

    assert len(open_day_results) == 1
    assert open_day_results[0]["question"] == "Какие направления бакалавриата доступны?"
    assert len(science_results) == 1
    assert science_results[0]["question"] == "Где будет выставка проектов?"


def test_search_returns_category_and_source_metadata(tmp_path: Path) -> None:
    store = FAQStore(db_path=str(tmp_path / "faq.db"))
    store.replace_items(
        event_id="open-day-2026",
        items=[
            {
                "question": "Что такое ядро бакалавриата?",
                "answer": "Это общеакадемическая часть программы.",
                "category": "Общие вопросы",
                "source": "faq.xlsx",
            }
        ],
    )

    results = store.search("ядро бакалавриата", event_id="open-day-2026", limit=3)

    assert results[0]["category"] == "Общие вопросы"
    assert results[0]["source"] == "faq.xlsx"
    assert results[0]["answer"] == "Это общеакадемическая часть программы."
