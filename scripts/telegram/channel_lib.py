"""Общий код для канала @robbox_project: разбор постов очереди, выбор следующего, отправка через бота робота.

Токен бота остаётся в контейнере telegram-bot на Sound Pi (как в `scripts/robot_dev/tgsend.sh`): сюда он не
попадает, наружу печатается только ответ Telegram без токена. Правила ведения — `docs/process/telegram-channel.md`.
"""

from __future__ import annotations

import html
import json
import re
import shlex
import subprocess
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Callable, Optional

CHANNEL_ID = -1004323185327
MIN_GAP = timedelta(hours=48)
CAPTION_LIMIT = 1024
MESSAGE_LIMIT = 4096
STATUSES = ("draft", "approved", "posted")
# Словарь хэштегов канала (docs/process/telegram-channel.md): тематические и рубрики. Новый тег — сначала сюда.
TOPIC_TAGS = {
    "#роббокс",
    "#навигация",
    "#симуляция",
    "#железо",
    "#светодиоды",
    "#голос",
    "#музыка",
    "#безопасность",
    "#тесты",
    "#opensource",
    "#ros2",
    "#raspberrypi",
    "#люди",
}
RUBRIC_TAGS = {"#дневник", "#какэтоработает", "#цифры", "#честность"}
TAG_DICTIONARY = TOPIC_TAGS | RUBRIC_TAGS
MIN_TAGS, MAX_TAGS = 2, 4
# Telegram HTML (Bot API, «Formatting options»): b, i, u, s, tg-spoiler, a, tg-emoji, code, pre, blockquote
# (в т.ч. `<blockquote expandable>`); blockquote не вкладываются.
ALLOWED_TAGS = {
    "b",
    "strong",
    "i",
    "em",
    "u",
    "ins",
    "s",
    "strike",
    "del",
    "a",
    "code",
    "pre",
    "tg-spoiler",
    "tg-emoji",
    "blockquote",
    "span",
}
PHOTO_MAX_BYTES = 10 * 1024 * 1024
PHOTO_MAX_SUM = 10000
PHOTO_MAX_RATIO = 20
ANIMATION_MAX_BYTES = 50 * 1024 * 1024
GIF_PROJECT_MAX_BYTES = 2 * 1024 * 1024
DRAFT_CHAT_ID = 495039871  # личка Шифу с ботом робота (как в tgsend.sh)
ANIMATION_EXT = {".gif", ".mp4"}
PHOTO_EXT = {".png", ".jpg", ".jpeg"}
# Приватность: внутренние адреса и токены в канал не попадают никогда.
FORBIDDEN = [
    (re.compile(r"\b10\.1\.1\.\d{1,3}\b"), "внутренний IP"),
    (re.compile(r"\b192\.168\.\d{1,3}\.\d{1,3}\b"), "внутренний IP"),
    (re.compile(r"\d{8,}:[A-Za-z0-9_-]{30,}"), "токен бота"),
]

# Тип отправки: (метод Bot API, поле с файлом)
Transport = Callable[[str, dict, Optional[Path]], dict]


@dataclass
class Post:
    path: Path
    status: str
    media: str
    not_before: datetime
    message_id: Optional[int]
    posted_at: Optional[datetime]
    hashtags: list = field(default_factory=list)
    body: str = ""
    extra_media: str = ""  # вторая картинка (инфографика): уходит сразу после основного поста, без подписи
    pending: list = field(
        default_factory=list
    )  # чего ещё не хватает (media_pending, extra_pending): блокирует публикацию

    @property
    def name(self) -> str:
        return self.path.name


def _dt(value: str) -> Optional[datetime]:
    value = value.strip()
    if not value:
        return None
    return datetime.fromisoformat(value.replace("Z", "+00:00")).astimezone(timezone.utc)


def parse_post(path: Path) -> Post:
    text = path.read_text(encoding="utf-8").replace("\r\n", "\n")  # CRLF после checkout на Windows
    if not text.startswith("---\n"):
        raise ValueError(f"{path.name}: нет front matter")
    head, _, body = text[4:].partition("\n---\n")
    meta = {}
    for line in head.splitlines():
        key, _, value = line.partition(":")
        meta[key.strip()] = value.strip()
    if meta.get("status") not in STATUSES:
        raise ValueError(f"{path.name}: status должен быть из {STATUSES}, а не {meta.get('status')!r}")
    not_before = _dt(meta.get("not_before", ""))
    if not_before is None:
        raise ValueError(f"{path.name}: не задан not_before")
    mid = meta.get("message_id", "")
    return Post(
        path=path,
        status=meta["status"],
        media=meta.get("media", ""),
        not_before=not_before,
        message_id=int(mid) if mid else None,
        posted_at=_dt(meta.get("posted_at", "")),
        hashtags=meta.get("hashtags", "").split(),
        body=body.strip(),
        extra_media=meta.get("extra_media", ""),
        pending=[meta[k] for k in ("media_pending", "extra_pending") if meta.get(k)],
    )


def load_queue(queue_dir: Path) -> list:
    return [parse_post(p) for p in sorted(queue_dir.glob("[0-9][0-9][0-9]-*.md"))]


def last_posted_at(posts: list) -> Optional[datetime]:
    times = [p.posted_at for p in posts if p.status == "posted" and p.posted_at]
    return max(times) if times else None


def pick_next(posts: list, now: datetime, min_gap: timedelta = MIN_GAP) -> Optional[Post]:
    """Следующий пост: первый по номеру со статусом approved и наступившим not_before.

    Если с прошлой публикации прошло меньше min_gap — None (не чаще раза в 48 ч).
    """
    last = last_posted_at(posts)
    if last is not None and now - last < min_gap:
        return None
    for post in posts:  # posts уже отсортированы по имени файла (номеру)
        if post.status == "approved" and post.not_before <= now:
            return post
    return None


def visible_length(html_text: str) -> int:
    """Длина текста «после разбора сущностей» (так Telegram считает лимиты): без тегов, с раскрытыми &lt; &amp;."""
    return len(html.unescape(re.sub(r"<[^>]*>", "", html_text)))


def html_problems(body: str) -> list:
    """Проверка Telegram HTML (Bot API, Formatting options): теги, вложенность, экранирование."""
    problems = []
    stack = []
    for m in re.finditer(r"<(/?)([a-zA-Z][a-zA-Z0-9-]*)([^>]*)>", body):
        closing, tag = m.group(1) == "/", m.group(2).lower()
        if tag not in ALLOWED_TAGS:
            problems.append(f"тег <{tag}> не поддерживается Telegram HTML")
        elif closing:
            if not stack or stack[-1] != tag:
                problems.append(f"</{tag}> без пары или в неверном порядке")
            else:
                stack.pop()
        else:
            if tag == "blockquote" and "blockquote" in stack:
                problems.append("blockquote нельзя вкладывать друг в друга")
            stack.append(tag)
    problems += [f"не закрыт тег <{t}>" for t in stack]
    stripped = re.sub(r"<[^>]*>", "", body)
    if re.search(r"[<>]|&(?!(amp|lt|gt|quot);)", stripped):
        problems.append("голые < > & в тексте: экранируй как &lt; &gt; &amp;")
    return problems


def media_problems(media: Path, label: str) -> list:
    if not media.is_file():
        return [f"нет файла медиа {label}"]
    size, ext = media.stat().st_size, media.suffix.lower()
    if ext in PHOTO_EXT:
        out = []
        if size > PHOTO_MAX_BYTES:
            out.append(f"фото {size} байт больше 10 МБ (sendPhoto)")
        try:
            from PIL import Image
        except ImportError:  # без Pillow размеры сторон не проверить: размер файла уже проверен
            return out
        width, height = Image.open(media).size
        if width + height > PHOTO_MAX_SUM:
            out.append(f"ширина+высота {width + height} больше {PHOTO_MAX_SUM} (sendPhoto)")
        if max(width, height) > PHOTO_MAX_RATIO * min(width, height):
            out.append(f"соотношение сторон больше {PHOTO_MAX_RATIO}:1 (sendPhoto)")
        return out
    if ext in ANIMATION_EXT:
        if size > ANIMATION_MAX_BYTES:
            return [f"анимация {size} байт больше 50 МБ (sendAnimation)"]
        if ext == ".gif" and size > GIF_PROJECT_MAX_BYTES:
            return [f"GIF {size} байт больше 2 МБ (правило проекта)"]
        return []
    return [f"тип медиа {ext} не поддержан"]


def validate(post: Post, repo_root: Path) -> list:
    """Список проблем поста; пустой — можно публиковать. Лимиты — Bot API 10.3, проверены 06.11.2026."""
    problems = []
    limit = CAPTION_LIMIT if post.media else MESSAGE_LIMIT
    length = visible_length(post.body)
    if length > limit:
        problems.append(f"текст {length} знаков после разбора разметки, лимит Telegram {limit}")
    if not post.body.strip():
        problems.append("пустой текст")
    problems += html_problems(post.body)
    for pattern, what in FORBIDDEN:
        if pattern.search(post.body):
            problems.append(f"в тексте {what}: публиковать нельзя")
    for tag in post.hashtags:
        if tag not in post.body:
            problems.append(f"хэштега {tag} нет в тексте")
        if tag not in TAG_DICTIONARY:
            problems.append(f"хэштега {tag} нет в словаре канала")
    for tag in sorted(set(re.findall(r"(?<![\w&])#[^\W\d]\w*", post.body)) - set(post.hashtags)):
        problems.append(f"в тексте тег {tag}, которого нет в hashtags поста")
    if not MIN_TAGS <= len(post.hashtags) <= MAX_TAGS:
        problems.append(f"хэштегов {len(post.hashtags)}, нужно {MIN_TAGS}–{MAX_TAGS}")
    if post.media:
        problems += media_problems(repo_root / post.media, post.media)
    if post.extra_media:
        problems += media_problems(repo_root / post.extra_media, post.extra_media)
    problems += [f"не готово: {item}" for item in post.pending]
    return problems


def build_request(post: Post, repo_root: Path, chat_id: int = CHANNEL_ID) -> tuple:
    """(метод Bot API, параметры, файл или None) для поста."""
    params = {"chat_id": chat_id, "parse_mode": "HTML"}
    if not post.media:
        params["text"] = post.body
        params["link_preview_options"] = {"is_disabled": True}
        return "sendMessage", params, None
    media = repo_root / post.media
    params["caption"] = post.body
    if media.suffix.lower() in ANIMATION_EXT:
        return "sendAnimation", params, media
    return "sendPhoto", params, media


def build_extra_request(post: Post, repo_root: Path, chat_id: int = CHANNEL_ID) -> Optional[tuple]:
    """Вторая картинка поста (инфографика): sendPhoto без подписи; None, если её нет."""
    if not post.extra_media:
        return None
    return "sendPhoto", {"chat_id": chat_id}, repo_root / post.extra_media


FILE_FIELD = {"sendAnimation": "animation", "sendPhoto": "photo", "setChatPhoto": "photo"}


class RobotTransport:
    """Вызов Bot API из контейнера telegram-bot на Sound Pi: токен в окружении контейнера."""

    def __init__(self, robot: str = "ros2@10.1.1.22", executor: Optional[Path] = None):
        self.robot = robot
        self.executor = executor or Path(__file__).with_name("channel_exec.py")

    def __call__(self, method: str, params: dict, media: Optional[Path] = None) -> dict:
        ssh = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", self.robot]
        remote = ["docker cp /tmp/channel_exec.py telegram-bot:/tmp/channel_exec.py"]
        scp = ["scp", "-q", "-o", "BatchMode=yes"]
        request = {"method": method, "params": params, "file": None}
        subprocess.run(scp + [str(self.executor), f"{self.robot}:/tmp/channel_exec.py"], check=True, timeout=60)
        if media is not None:
            tmp = f"/tmp/channel_media{media.suffix.lower()}"
            subprocess.run(scp + [str(media), f"{self.robot}:{tmp}"], check=True, timeout=120)
            remote.append(f"docker cp {shlex.quote(tmp)} telegram-bot:{shlex.quote(tmp)}")
            request["file"] = {"field": FILE_FIELD[method], "path": tmp}
        remote.append("docker exec -i telegram-bot python3 /tmp/channel_exec.py")
        done = subprocess.run(
            ssh + [" && ".join(remote)], input=json.dumps(request), text=True, capture_output=True, timeout=120
        )
        try:
            return json.loads(done.stdout.strip().splitlines()[-1])
        except (IndexError, ValueError):
            return {"ok": False, "description": f"нет ответа от контейнера (код {done.returncode})"}


def set_front_matter(path: Path, **updates) -> None:
    """Меняет значения ключей front matter, остальное не трогает."""
    text = path.read_text(encoding="utf-8").replace("\r\n", "\n")
    head, sep, body = text[4:].partition("\n---\n")
    lines = []
    seen = set()
    for line in head.splitlines():
        key = line.partition(":")[0].strip()
        if key in updates:
            lines.append(f"{key}: {updates[key]}")
            seen.add(key)
        else:
            lines.append(line)
    for key, value in updates.items():
        if key not in seen:
            lines.append(f"{key}: {value}")
    path.write_text("---\n" + "\n".join(lines) + sep + body, encoding="utf-8")


def describe(method: str, params: dict, media: Optional[Path]) -> str:
    """Что будет отправлено — для --dry-run (без токена, он сюда не попадает)."""
    out = [f"  метод:  {method}", f"  чат:    {params.get('chat_id')}"]
    if media is not None:
        out.append(f"  файл:   {media} (" + (f"{media.stat().st_size} байт)" if media.exists() else "НЕТ ФАЙЛА)"))
    text = params.get("caption") or params.get("text") or params.get("title") or params.get("description")
    if text is not None:
        out.append("  текст:")
        out.extend("    | " + line for line in str(text).splitlines())
    return "\n".join(out)
