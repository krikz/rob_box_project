"""Раскладка картинок канала: рамки текста не пересекаются, подпись не заходит на знак РОББОКС (дефект поста 009)."""

import itertools
import sys
from pathlib import Path

import pytest

Image = pytest.importorskip("PIL.Image", reason="нужен Pillow (pip install pillow)")
from PIL import ImageDraw  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import infographic  # noqa: E402
import make_channel_media as mcm  # noqa: E402

LONG_SRC = "issue #3733, комментарий координатора от 06.11 (замер 07:45 UTC); docs/research/nav2-hil-tuning.md §8"
FIXTURES = {
    "numbers": {"items": [{"value": "0.90 м", "label": "ширина проёма", "source": LONG_SRC}] * 3},
    "table": {"columns": ["a", "b", "c"], "rows": [["1", {"text": "x", "status": "ok"}, "z"]], "source": LONG_SRC},
    "nodes": {"nodes": [{"name": "A", "items": ["x"]}, {"name": "B", "items": ["y"]}], "source": LONG_SRC},
    "before_after": {"pairs": [{"label": "l", "before": "57–64", "after": "25", "source": LONG_SRC}]},
    "timeline": {
        "events": [
            {"date": "05.11", "text": "t", "source": LONG_SRC},
            {"date": "06.11", "text": "u", "source": LONG_SRC},
        ]
    },
}


def overlaps(a, b) -> bool:
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def assert_clean(boxes, size):
    w, h = size
    for text, box in boxes:
        assert box[0] >= 60 and box[2] <= w - 60 and box[3] <= h - 60, f"{text!r} у самого края: {box}"
    for (t1, b1), (t2, b2) in itertools.combinations(boxes, 2):
        assert not overlaps(b1, b2), f"{t1!r} {b1} налезает на {t2!r} {b2}"


@pytest.mark.parametrize("kind", sorted(FIXTURES))
@pytest.mark.parametrize("size_name", ["card", "wide"])
def test_template_text_boxes_do_not_overlap(kind, size_name):
    boxes = []
    infographic.render({"template": kind, "title": "Заголовок", **FIXTURES[kind]}, size_name, boxes=boxes)
    assert any(t == mcm.BADGE_TEXT for t, _ in boxes)
    assert_clean(boxes, infographic.SIZES[size_name])


@pytest.mark.parametrize("path", sorted((REPO / "docs" / "channel" / "infographics").glob("*.json")))
def test_real_infographics_have_clean_layout(path):
    boxes = []
    infographic.render(infographic.load_spec(path), "card", boxes=boxes)
    assert_clean(boxes, infographic.SIZES["card"])


def test_card_long_caption_wraps_or_shrinks_and_never_touches_badge():
    boxes = []
    caption = "ADR-0148 · честный FAIL лучше красивого PASS, и ещё пара слов про это"
    mcm.card(None, ["Код решает,", "LLM говорит"], caption, boxes=boxes)
    assert_clean(boxes, (1000, 560))
    badge = next(b for t, b in boxes if t == mcm.BADGE_TEXT)
    footer = [(t, b) for t, b in boxes if t != mcm.BADGE_TEXT and b[1] > 400]
    assert footer
    for t, b in footer:
        assert b[2] <= badge[0] - mcm.CAPTION_GAP, f"{t!r} заходит в зону знака"


def test_real_cards_have_clean_layout():
    for big, small, acts in (
        (["131 шаг,", "12 актов,", "одна ночь"], "Ночной голосовой марафон", 12),
        (["Код решает,", "LLM говорит"], "ADR-0148 · честный FAIL лучше красивого PASS", 0),
    ):
        boxes = []
        mcm.card(None, big, small, acts=acts, boxes=boxes)
        assert_clean(boxes, (1000, 560))


def test_caption_that_cannot_fit_is_an_honest_error():
    d = ImageDraw.Draw(Image.new("RGB", (1000, 560)))
    with pytest.raises(mcm.LayoutError):
        mcm.fit_caption(d, "слово " * 60, (30, 24), 400)
    with pytest.raises(mcm.LayoutError):
        mcm.card(None, ["a"], "слово " * 60)
