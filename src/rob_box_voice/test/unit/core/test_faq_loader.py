import json
import sys
import types
from pathlib import Path

import pytest

from rob_box_voice.core.faq_loader import load_faq_items


def test_load_faq_items_from_csv(tmp_path: Path) -> None:
    csv_path = tmp_path / "faq.csv"
    csv_path.write_text(
        "Question,Answer,Source,Question type\n"
        "Где проходит день открытых дверей?,В главном корпусе.,faq.csv,Общие вопросы\n",
        encoding="utf-8",
    )

    items = load_faq_items(csv_path)

    assert items == [
        {
            "question": "Где проходит день открытых дверей?",
            "answer": "В главном корпусе.",
            "source": "faq.csv",
            "category": "Общие вопросы",
        }
    ]


def test_load_faq_items_from_json(tmp_path: Path) -> None:
    json_path = tmp_path / "faq.json"
    json_path.write_text(
        json.dumps(
            [
                {
                    "Question": "Какие сферы деятельности представлены?",
                    "Answer": "Государство, управление, право и ИТ.",
                    "Question type": "Общие вопросы",
                    "Source": "faq.json",
                }
            ],
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )

    items = load_faq_items(json_path)

    assert items[0]["question"].startswith("Какие сферы")
    assert items[0]["category"] == "Общие вопросы"


def test_load_faq_items_from_xlsx_via_openpyxl(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    xlsx_path = tmp_path / "faq.xlsx"
    xlsx_path.write_bytes(b"placeholder")

    class FakeSheet:
        def iter_rows(self, values_only: bool = True):
            yield ("№", "Question", "Answer", "Source", "Question type")
            yield (
                1,
                "Что такое кластер?",
                "Кластер объединяет программы.",
                "faq.xlsx",
                "Общие вопросы",
            )

    class FakeWorkbook:
        sheetnames = ["Лист 1"]

        def __getitem__(self, name: str) -> FakeSheet:
            assert name == "Лист 1"
            return FakeSheet()

    fake_openpyxl = types.SimpleNamespace(
        load_workbook=lambda *args, **kwargs: FakeWorkbook(),
    )
    monkeypatch.setitem(sys.modules, "openpyxl", fake_openpyxl)

    items = load_faq_items(xlsx_path)

    assert items == [
        {
            "question": "Что такое кластер?",
            "answer": "Кластер объединяет программы.",
            "source": "faq.xlsx",
            "category": "Общие вопросы",
        }
    ]


def test_load_faq_items_raises_for_missing_required_columns(tmp_path: Path) -> None:
    csv_path = tmp_path / "broken.csv"
    csv_path.write_text("Question,Source\nГде?,faq.csv\n", encoding="utf-8")

    with pytest.raises(ValueError, match="Answer"):
        load_faq_items(csv_path)
