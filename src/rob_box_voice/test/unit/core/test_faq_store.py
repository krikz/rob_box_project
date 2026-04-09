from pathlib import Path

from rob_box_voice.core.faq_store import FAQStore


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
