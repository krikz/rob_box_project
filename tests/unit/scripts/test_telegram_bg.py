"""Подложки картинок канала: кэш, затемнение, лимит JPEG, честная ошибка; сеть замокана."""

import base64
import io
import sys
from pathlib import Path

import pytest

Image = pytest.importorskip("PIL.Image", reason="нужен Pillow (pip install pillow)")

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import channel_bg as bg  # noqa: E402
import infographic  # noqa: E402

CFG = {"slug": "bg-test", "prompt": "abstract warehouse", "seed": 3}


def png_b64(color=(250, 250, 250), size=(640, 360)) -> str:
    buf = io.BytesIO()
    Image.new("RGB", size, color).save(buf, "PNG")
    return base64.b64encode(buf.getvalue()).decode()


class FakeRemote:
    def __init__(self, **result):
        self.calls = []
        self.result = result or {"ok": True, "image_base64": png_b64(), "description": ""}

    def __call__(self, request):
        self.calls.append(request)
        return self.result


def test_generates_once_then_takes_cache(tmp_path):
    remote = FakeRemote()
    first = bg.load_background(CFG, tmp_path, remote)
    again = bg.load_background(CFG, tmp_path, lambda r: pytest.fail("кэш есть: API звать нельзя"))
    assert len(remote.calls) == 1 and (tmp_path / "bg-test.jpg").is_file()
    assert first.size == again.size == bg.BG_SIZE


def test_request_forbids_text_and_asks_16_9(tmp_path):
    remote = FakeRemote()
    bg.load_background(CFG, tmp_path, remote)
    req = remote.calls[0]
    assert req["aspect_ratio"] == "16:9" and req["seed"] == 3 and req["prompt"].startswith("abstract warehouse")
    assert "no text, no letters" in req["prompt"] and "no logos" in req["prompt"] and len(req["prompt"]) <= 1500


def test_bright_image_is_darkened_for_readable_text(tmp_path):
    img = bg.load_background(CFG, tmp_path, FakeRemote())  # почти белая картинка
    assert bg.mean_luma(img) <= bg.MAX_LUMA + 3


def test_cached_jpeg_is_within_300kb_even_for_noise(tmp_path):
    noise = Image.effect_noise((1280, 720), 80).convert("RGB")
    bg.save_limited(bg.finish(noise), tmp_path / "n.jpg")
    assert (tmp_path / "n.jpg").stat().st_size <= 300 * 1024


def test_api_refusal_is_honest_error_and_nothing_cached(tmp_path):
    remote = FakeRemote(ok=False, image_base64=None, description="1008: insufficient balance")
    with pytest.raises(bg.BackgroundError, match="1008"):
        bg.load_background(CFG, tmp_path, remote)
    assert not list(tmp_path.glob("*.jpg"))


def test_config_needs_prompt_and_slug(tmp_path):
    with pytest.raises(bg.BackgroundError):
        bg.load_background({"slug": "x"}, tmp_path, FakeRemote())
    with pytest.raises(bg.BackgroundError):
        bg.load_background({"prompt": "p", "slug": "Bad Slug"}, tmp_path, FakeRemote())


def test_infographic_with_background_uses_loader_and_keeps_text_clean(tmp_path):
    spec = {
        "template": "numbers",
        "title": "Заголовок",
        "items": [{"value": "1", "label": "a", "source": "x.md"}, {"value": "2", "label": "b", "source": "x.md"}],
        "background": CFG,
    }
    seen = []
    boxes = []

    def loader(cfg):
        seen.append(cfg["slug"])
        return bg.load_background(cfg, tmp_path, FakeRemote())

    img = infographic.render(spec, boxes=boxes, bg_loader=loader)
    assert seen == ["bg-test"] and img.size == infographic.SIZES["card"]
    assert any(t == "Заголовок" for t, _ in boxes)
