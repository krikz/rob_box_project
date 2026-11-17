"""Декоративные подложки картинок канала: MiniMax image-01 -> затемнить и размыть -> кэш JPEG в docs/channel/media/bg/.

Правило честности (docs/process/telegram-channel.md): сгенерированная иллюстрация только фон, абстракция или тема,
никакого «фото нашего робота». Текст и цифры поверх рисует код. Повторный рендер берёт кэш и API не зовёт.
Ключ MINIMAX_API_KEY остаётся в контейнере telegram-bot на Sound Pi (как у channel_exec.py).
"""

from __future__ import annotations

import base64
import io
import json
import re
import subprocess
from pathlib import Path
from typing import Callable, Optional

from PIL import Image, ImageDraw, ImageEnhance, ImageFilter

REPO = Path(__file__).resolve().parents[2]
BG_DIR = REPO / "docs" / "channel" / "media" / "bg"
BG_SIZE = (1280, 720)
MAX_BYTES = 300 * 1024
MAX_LUMA = 52  # средняя яркость подложки (0-255) не выше: белый и серый текст остаётся читаемым
SUFFIX = (
    " Abstract decorative background illustration, dark moody palette, no text, no letters, no numbers, no logos, "
    "no watermark, no people, not a photograph, not a realistic robot."
)
SLUG = re.compile(r"^[a-z0-9][a-z0-9-]*$")

Remote = Callable[[dict], dict]  # запрос {"prompt","aspect_ratio","seed"} -> {"ok","image_base64","description"}


class BackgroundError(RuntimeError):
    pass


def check_config(cfg: dict) -> None:
    if not cfg.get("prompt"):
        raise BackgroundError("background: нужен prompt")
    if not SLUG.match(str(cfg.get("slug", ""))):
        raise BackgroundError("background: нужен slug (a-z, 0-9, дефис): под ним подложка лежит в кэше")
    if len(cfg["prompt"]) + len(SUFFIX) > 1500:
        raise BackgroundError("background: prompt вместе с обязательной припиской длиннее 1500 знаков (лимит API)")


def cache_path(slug: str, bg_dir: Path = BG_DIR) -> Path:
    return bg_dir / f"{slug}.jpg"


def mean_luma(img: Image.Image) -> float:
    return sum(img.convert("L").resize((64, 36)).getdata()) / (64 * 36)


def finish(raw: Image.Image) -> Image.Image:
    """Обрезать до 16:9, размыть и затемнить так, чтобы средняя яркость была не выше MAX_LUMA."""
    img = raw.convert("RGB")
    scale = max(BG_SIZE[0] / img.width, BG_SIZE[1] / img.height)
    img = img.resize((round(img.width * scale), round(img.height * scale)), Image.LANCZOS)
    left, top = (img.width - BG_SIZE[0]) // 2, (img.height - BG_SIZE[1]) // 2
    img = img.crop((left, top, left + BG_SIZE[0], top + BG_SIZE[1])).filter(ImageFilter.GaussianBlur(5))
    img = ImageEnhance.Brightness(img).enhance(0.5)
    luma = mean_luma(img)
    if luma > MAX_LUMA:
        img = ImageEnhance.Brightness(img).enhance(MAX_LUMA / luma)
    return img


def save_limited(img: Image.Image, path: Path, max_bytes: int = MAX_BYTES) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    for quality in (85, 75, 65, 55, 45, 35):
        buf = io.BytesIO()
        img.save(buf, "JPEG", quality=quality, optimize=True)
        if buf.tell() <= max_bytes:
            path.write_bytes(buf.getvalue())
            return
    raise BackgroundError(f"JPEG подложки не влез в {max_bytes} байт даже при quality=35")


def load_background(cfg: dict, bg_dir: Path = BG_DIR, remote: Optional[Remote] = None) -> Image.Image:
    """Подложка из кэша; нет в кэше: сгенерировать, обработать, сохранить."""
    check_config(cfg)
    path = cache_path(cfg["slug"], bg_dir)
    if path.is_file():
        return Image.open(path).convert("RGB")
    remote = remote or RobotImageRemote()
    result = remote({"prompt": cfg["prompt"] + SUFFIX, "aspect_ratio": "16:9", "seed": cfg.get("seed")})
    if not result.get("ok") or not result.get("image_base64"):
        raise BackgroundError(f"MiniMax не отдал картинку: {result.get('description')}")
    img = finish(Image.open(io.BytesIO(base64.b64decode(result["image_base64"]))))
    save_limited(img, path)
    return Image.open(path).convert("RGB")


class RobotImageRemote:
    """Вызов image_exec.py в контейнере telegram-bot на Sound Pi: ключ в окружении контейнера, наружу не выходит."""

    def __init__(self, robot: str = "ros2@10.1.1.22", executor: Optional[Path] = None):
        self.robot = robot
        self.executor = executor or Path(__file__).with_name("image_exec.py")

    def __call__(self, request: dict) -> dict:
        ssh = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", self.robot]
        subprocess.run(
            ["scp", "-q", "-o", "BatchMode=yes", str(self.executor), f"{self.robot}:/tmp/image_exec.py"],
            check=True,
            timeout=60,
        )
        remote = "docker cp /tmp/image_exec.py telegram-bot:/tmp/image_exec.py && "
        remote += "docker exec -i telegram-bot python3 /tmp/image_exec.py"
        done = subprocess.run(ssh + [remote], input=json.dumps(request), text=True, capture_output=True, timeout=180)
        try:
            return json.loads(done.stdout.strip().splitlines()[-1])
        except (IndexError, ValueError):
            return {"ok": False, "description": f"нет ответа от контейнера (код {done.returncode})"}


def apply_background(img: Image.Image, bg: Image.Image, margin: int = 40, radius: int = 28) -> None:
    """Подложка внутрь скруглённой панели (поля margin), поверх неё код рисует текст."""
    w, h = img.size
    box = (margin, margin, w - margin, h - margin)
    size = (box[2] - box[0], box[3] - box[1])
    scale = max(size[0] / bg.width, size[1] / bg.height)
    fitted = bg.resize((round(bg.width * scale), round(bg.height * scale)), Image.LANCZOS)
    left, top = (fitted.width - size[0]) // 2, (fitted.height - size[1]) // 2
    fitted = fitted.crop((left, top, left + size[0], top + size[1]))
    mask = Image.new("L", size, 0)
    ImageDraw.Draw(mask).rounded_rectangle([0, 0, size[0] - 1, size[1] - 1], radius=radius, fill=255)
    img.paste(fitted, (box[0], box[1]), mask)
