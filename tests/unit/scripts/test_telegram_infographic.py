"""Инфографика канала: шаблоны дают картинку нужного размера, цифра без источника не рисуется."""

import json
import sys
from pathlib import Path

import pytest
from PIL import Image

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import infographic  # noqa: E402

SRC = "docs/research/x.md"
FIXTURES = {
    "numbers": {"items": [{"value": "1", "label": "a", "source": SRC}, {"value": "2", "label": "b", "source": SRC}]},
    "table": {"columns": ["a", "b"], "rows": [["1", {"text": "x", "status": "ok"}]], "source": SRC},
    "nodes": {"nodes": [{"name": "A", "items": ["x"]}, {"name": "B", "items": ["y"]}], "source": SRC},
    "before_after": {"pairs": [{"label": "l", "before": "64", "after": "25", "source": SRC}]},
    "timeline": {
        "events": [{"date": "05.11", "text": "t", "source": SRC}, {"date": "06.11", "text": "u", "source": SRC}]
    },
}


@pytest.mark.parametrize("kind", sorted(FIXTURES))
def test_template_has_telegram_size(kind):
    spec = {"template": kind, "title": "Заголовок", **FIXTURES[kind]}
    assert infographic.render(spec).size == (1000, 560)
    assert infographic.render(spec, "wide").size == (1280, 720)


def test_refuses_number_without_source():
    items = [{"value": "1", "label": "a"}, {"value": "2", "label": "b", "source": SRC}]
    with pytest.raises(infographic.SpecError, match="source"):
        infographic.render({"template": "numbers", "title": "t", "items": items})


def test_refuses_unknown_template_and_wrong_count():
    with pytest.raises(infographic.SpecError):
        infographic.render({"template": "pie", "title": "t"})
    one = {"template": "numbers", "title": "t", "items": FIXTURES["numbers"]["items"][:1]}
    with pytest.raises(infographic.SpecError):
        infographic.render(one)


def test_gif_from_json_file(tmp_path):
    spec_path = tmp_path / "s.json"
    spec_path.write_text(json.dumps({"template": "nodes", "title": "t", **FIXTURES["nodes"]}), encoding="utf-8")
    out = tmp_path / "o.gif"
    infographic.render_gif(infographic.load_spec(spec_path), out)
    assert Image.open(out).size == (1000, 560)
    assert out.stat().st_size < 2 * 1024 * 1024


def test_real_examples_render_and_have_sources():
    for path in sorted((REPO / "docs" / "channel" / "infographics").glob("*.json")):
        spec = infographic.load_spec(path)
        assert infographic.sources_of(spec), path.name
        assert infographic.render(spec).size == (1000, 560)
