"""Общий код для канала @robbox_project: разбор постов очереди, выбор следующего, отправка через бота робота.

Токен бота остаётся в контейнере telegram-bot на Sound Pi (как в `scripts/robot_dev/tgsend.sh`): сюда он не
попадает, наружу печатается только ответ Telegram без токена. Правила ведения — `docs/process/telegram-channel.md`.
"""

from __future__ import annotations

import html
import json
import os
import re
import shlex
import subprocess
import tempfile
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Callable, Optional, Protocol

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
ANIMATION_EXT = {".gif"}
VIDEO_EXT = {".mp4"}
VIDEO_MAX_BYTES = 50 * 1024 * 1024  # sendVideo / InputMediaVideo при загрузке файлом
ALBUM_MIN, ALBUM_MAX = 2, 10  # sendMediaGroup: "must include 2-10 items"
PHOTO_EXT = {".png", ".jpg", ".jpeg"}
# Время для Шифу — GMT+1 (решение 27.10). Слоты и not_before считаются в UTC; в тексте Шифу —
# только через slot_text, иначе час по UTC уходит с подписью «по GMT+1» (09.11: «10:00» вместо 11:00).
SHIFU_TZ = timezone(timedelta(hours=1), "GMT+1")


def slot_text(moment: datetime) -> str:
    """«10.11 в 11:00 по GMT+1» — момент (aware, любой пояс) словами для Шифу."""
    return moment.astimezone(SHIFU_TZ).strftime("%d.%m в %H:%M") + " по GMT+1"


# Приватность: внутренние адреса и токены в канал не попадают никогда.
FORBIDDEN = [
    (re.compile(r"\b10\.1\.1\.\d{1,3}\b"), "внутренний IP"),
    (re.compile(r"\b192\.168\.\d{1,3}\.\d{1,3}\b"), "внутренний IP"),
    (re.compile(r"\d{8,}:[A-Za-z0-9_-]{30,}"), "токен бота"),
    (re.compile(r"\bsk-[A-Za-z0-9_-]{20,}"), "ключ API"),
    (re.compile(r"\bgh[pousr]_[A-Za-z0-9]{20,}"), "токен GitHub"),
    (re.compile(r"\bgithub_pat_[A-Za-z0-9_]{20,}"), "токен GitHub"),
    (re.compile(r"\bAAG[A-Za-z0-9_-]{30,}"), "токен бота"),
    (re.compile(r"\beyJ[A-Za-z0-9_-]{20,}"), "секрет в JWT-виде"),
]

# Внутренняя кухня проекта в ПОСТАХ не пишется (Шифу, 07.11): канал — про
# поведение робота, а не про PR-ы, ветки и файлы репозитория. Это правило текста
# поста (студия не даёт модели такое написать, публикатор не пропустит), а не
# приватность: ответы студии Шифу в личке и пояснение «почему так» номера PR
# называть могут — там они нужны, чтобы понять, о каком черновике речь.
COMMIT_TYPES = "feat|fix|docs|chore|refactor|test|perf|ci|style|build|revert"
FORBIDDEN_INTERNAL = [
    (re.compile(r"\bPR\s*#?\s*\d{3,5}\b", re.IGNORECASE), "номер PR"),
    (re.compile(r"#\d{3,5}\b"), "номер PR или issue"),
    (re.compile(r"\bissues?\s*#?\s*\d{3,5}\b", re.IGNORECASE), "номер issue"),
    (re.compile(r"github\.com/\S+/(?:pull|issues)/\d+", re.IGNORECASE), "ссылка на PR или issue"),
    (re.compile(r"\b(?:z-[a-z]+|feature|fix|feat|hotfix|revert|release)/[A-Za-z0-9._-]+"), "имя ветки"),
    (re.compile(r"\bветк[аиуеой]\s+(?:develop|main|master)\b", re.IGNORECASE), "имя ветки"),
    (re.compile(r"\bdevelop\b", re.IGNORECASE), "имя ветки"),
    # Разбор 09.11: черновик p20261109-810 вышел с «<code>fix(dialog)</code>» — тип коммита, не слово для читателя.
    (re.compile(rf"\b(?:{COMMIT_TYPES})(?:\([\w./-]+\))!?:|\b(?:{COMMIT_TYPES})\([\w./-]+\)"), "тип коммита"),
    (re.compile(r"\b(?:docs|src|scripts|config|docker)/[\w./-]+"), "путь в репозитории"),
    (re.compile(r"\b[\w./-]*\w+\.(?:py|yaml|yml|ini|json|toml|cfg|sh|cpp|hpp|launch|xml)\b"), "файл репозитория"),
]
# Что не попадает в текст поста: приватность и внутренняя кухня. Студия и публикатор проверяют одно и то же.
POST_FORBIDDEN = FORBIDDEN + FORBIDDEN_INTERNAL


# Отправка: (метод Bot API, параметры, файл | список файлов для альбома | None) -> ответ контейнера
class Transport(Protocol):
    """Куда уходит пост. Реализации: RobotTransport (через робота) и DirectTransport (через токен студии)."""

    def __call__(self, method: str, params: dict, media: object = None) -> dict: ...


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
    extra_media: str = ""  # вторая картинка (инфографика): уходит в одном альбоме с основным медиа, подпись на первом
    extra_message_ids: list = field(default_factory=list)  # message_id остальных сообщений альбома (без первого)
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
        extra_message_ids=[int(x) for x in meta.get("extra_message_ids", "").replace(",", " ").split()],
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
    if ext in VIDEO_EXT:
        if size > VIDEO_MAX_BYTES:
            return [f"видео {size} байт больше 50 МБ (sendVideo)"]
        return []
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
    # Вышедшие до решения 07.11 посты (003 с «#4487») не переписываем: правило — для новых.
    for pattern, what in FORBIDDEN if post.status == "posted" else POST_FORBIDDEN:
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
        problems += media_problems(resolve_media(repo_root, post.media), post.media)
    if post.extra_media:
        problems += media_problems(resolve_media(repo_root, post.extra_media), post.extra_media)
    problems += album_problems(post)
    problems += [f"не готово: {item}" for item in post.pending]
    return problems


def extract_tags(body: str) -> list:
    """Хештеги из текста — по порядку появления, только из словаря канала.

    Теги достаёт код, а не модель: модель часто забывает поле tags, а хештег в тексте
    есть всегда. Так теги не могут разойтись с текстом и не могут быть чужими.
    """
    found = []
    for match in re.findall(r"#[^\W\d]\w*", body):
        if match in TAG_DICTIONARY and match not in found:
            found.append(match)
    return found


def resolve_media(repo_root: Path, media: str) -> Path:
    """Найти файл материала: сначала в клоне, потом в worktree на develop.

    Свежие материалы лежат в develop и в рабочей ветке студии их ещё нет.
    Публикатор берёт файл оттуда, где он есть, — иначе пост ушёл бы без
    картинки, а это молчаливая потеря материала.
    """
    path = repo_root / media
    if path.exists():
        return path
    develop = Path(os.environ.get("STUDIO_DEVELOP_WORKTREE", repo_root.parent / "rob_box_develop"))
    candidate = develop / media
    if candidate.exists():
        return candidate
    return path


def build_request(post: Post, repo_root: Path, chat_id: int = CHANNEL_ID) -> tuple:
    """(метод Bot API, параметры, файл или None) для поста."""
    params = {"chat_id": chat_id, "parse_mode": "HTML"}
    if not post.media:
        params["text"] = post.body
        params["link_preview_options"] = {"is_disabled": True}
        return "sendMessage", params, None
    media = resolve_media(repo_root, post.media)
    params["caption"] = post.body
    ext = media.suffix.lower()
    if ext in VIDEO_EXT:
        params["supports_streaming"] = True
        return "sendVideo", params, media
    if ext in ANIMATION_EXT:
        return "sendAnimation", params, media
    return "sendPhoto", params, media


def gif_to_mp4(gif: Path, workdir: Optional[Path] = None) -> Path:
    """GIF -> MP4 для альбома: sendMediaGroup не принимает animation, только photo/video/document/audio.

    h264, yuv420p, без звука, чётные размеры (иначе libx264 с yuv420p откажет).
    """
    workdir = workdir or Path(tempfile.gettempdir()) / "channel_album"
    workdir.mkdir(parents=True, exist_ok=True)
    out = workdir / (gif.stem + ".mp4")
    cmd = [
        "ffmpeg", "-y", "-loglevel", "error", "-i", str(gif), "-an",
        "-c:v", "libx264", "-pix_fmt", "yuv420p",
        "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
        "-movflags", "+faststart", str(out),
    ]  # fmt: skip
    subprocess.run(cmd, check=True, timeout=120)
    return out


def album_problems(post: Post) -> list:
    """Лимиты sendMediaGroup (Bot API, проверено 06.11.2026): 2-10 элементов, подпись 0-1024."""
    if not post.extra_media:
        return []
    if not post.media:
        return ["extra_media без основного media: альбому нужно 2-10 элементов"]
    return []


def build_album_request(
    post: Post,
    repo_root: Path,
    chat_id: int = CHANNEL_ID,
    convert: Optional[Callable[[Path], Path]] = None,
) -> tuple:
    """sendMediaGroup: основное медиа + extra_media одним альбомом, подпись на первом элементе.

    GIF в альбоме нельзя (animation не входит в InputMedia*), поэтому он конвертируется в MP4 и идёт как video.
    Возвращает (метод, параметры, список файлов); элементы media ссылаются на файлы как attach://f0, attach://f1.
    """
    files, items = [], []
    for i, rel in enumerate([post.media, post.extra_media]):
        path = resolve_media(repo_root, rel)
        if path.suffix.lower() in ANIMATION_EXT:
            path = (convert or gif_to_mp4)(path)
        is_video = path.suffix.lower() in VIDEO_EXT
        item = {"type": "video" if is_video else "photo", "media": f"attach://f{i}"}
        if is_video:
            item["supports_streaming"] = True
        if i == 0:
            item["caption"] = post.body
            item["parse_mode"] = "HTML"
        files.append(path)
        items.append(item)
    if not ALBUM_MIN <= len(items) <= ALBUM_MAX:
        raise ValueError(f"в альбоме {len(items)} элементов, нужно {ALBUM_MIN}-{ALBUM_MAX}")
    return "sendMediaGroup", {"chat_id": chat_id, "media": items}, files


def build_publish_request(
    post: Post, repo_root: Path, chat_id: int = CHANNEL_ID, convert: Optional[Callable[[Path], Path]] = None
) -> tuple:
    """Что уходит в Telegram: альбом, если есть extra_media, иначе одно сообщение. Решает код, не вызывающий."""
    if post.extra_media:
        return build_album_request(post, repo_root, chat_id, convert)
    return build_request(post, repo_root, chat_id)


def sent_ids(result: dict) -> list:
    """message_id всех сообщений ответа (альбом даёт несколько)."""
    ids = result.get("message_ids")
    if ids:
        return [int(x) for x in ids]
    return [int(result["message_id"])] if result.get("message_id") else []


FILE_FIELD = {
    "sendAnimation": "animation",
    "sendVideo": "video",
    "sendPhoto": "photo",
    "setChatPhoto": "photo",
}


def _as_path(item: object) -> Path:
    """Медиа — всегда путь. Тип object в сигнатуре транспорта требует сузить его здесь, а не молча."""
    if isinstance(item, Path):
        return item
    if isinstance(item, str):
        return Path(item)
    raise TypeError(f"медиа должно быть путём, а не {type(item).__name__}")


class DirectTransport:
    """Прямые вызовы Bot API с токеном студии (TG_STUDIO_BOT_TOKEN).

    Нужен там, где робот недоступен: студия контента живёт на отдельной машине, а канал ведёт отдельный бот
    (@robboxcontentbot). Токен приходит из окружения и наружу не печатается: в сообщениях об ошибках URL
    вырезается, как и в channel_exec.py.

    Ответ приводится к тому же виду, что у RobotTransport: {"ok", "message_id", "message_ids", "description"}.
    """

    def __init__(self, token: Optional[str] = None, timeout: int = 180, session=None):
        self.token = token or os.environ.get("TG_STUDIO_BOT_TOKEN", "")
        if not self.token:
            raise RuntimeError("TG_STUDIO_BOT_TOKEN не задан: студии нужен свой бот, токен робота не подходит")
        self.timeout = timeout
        self._session = session

    def _post(self, method: str, params: dict, files: list) -> dict:
        url = f"https://api.telegram.org/bot{self.token}/{method}"
        data = {k: (json.dumps(v) if isinstance(v, (bool, dict, list)) else v) for k, v in params.items()}
        session = self._session
        if session is None:
            import requests  # локальный импорт: requests нужен только для живой отправки

            session = requests.Session()
        try:
            if files:
                handles = [(f["field"], open(f["path"], "rb")) for f in files]
                try:
                    r = session.post(url, data=data, files=dict(handles), timeout=self.timeout)
                finally:
                    for _, fh in handles:
                        fh.close()
            else:
                r = session.post(url, json=params, timeout=self.timeout)
            payload = r.json()
        except Exception as e:  # не печатать URL: в нём токен
            return {"ok": False, "description": f"ERR {type(e).__name__}"}
        result = payload.get("result")
        if isinstance(result, list):  # sendMediaGroup: массив Message
            ids = [m.get("message_id") for m in result if isinstance(m, dict)]
        elif isinstance(result, dict):
            ids = [result.get("message_id")]
        else:
            ids = []
        ids = [i for i in ids if i]
        return {
            "ok": bool(payload.get("ok")),
            "message_id": ids[0] if ids else None,
            "message_ids": ids,
            "description": payload.get("description", ""),
        }

    def __call__(self, method: str, params: dict, media: object = None) -> dict:
        if isinstance(media, list):  # альбом: поля f0, f1, ... совпадают с attach://fN в params["media"]
            files = [{"field": f"f{i}", "path": str(item)} for i, item in enumerate(media)]
        elif media is not None:
            files = [{"field": FILE_FIELD.get(method, "document"), "path": str(media)}]
        else:
            files = []
        return self._post(method, params, files)


class RobotTransport:
    """Вызов Bot API из контейнера telegram-bot на Sound Pi: токен в окружении контейнера."""

    def __init__(self, robot: str = "ros2@10.1.1.22", executor: Optional[Path] = None):
        self.robot = robot
        self.executor = executor or Path(__file__).with_name("channel_exec.py")

    def __call__(self, method: str, params: dict, media: object = None) -> dict:
        ssh = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", self.robot]
        remote = ["docker cp /tmp/channel_exec.py telegram-bot:/tmp/channel_exec.py"]
        scp = ["scp", "-q", "-o", "BatchMode=yes"]
        request = {"method": method, "params": params, "file": None}
        subprocess.run(scp + [str(self.executor), f"{self.robot}:/tmp/channel_exec.py"], check=True, timeout=60)
        if isinstance(media, list):  # альбом: поля f0, f1, ... совпадают с attach://fN в params["media"]
            request["files"] = []
            for i, item in enumerate(media):
                tmp = f"/tmp/channel_media_{i}{_as_path(item).suffix.lower()}"
                subprocess.run(scp + [str(item), f"{self.robot}:{tmp}"], check=True, timeout=180)
                remote.append(f"docker cp {shlex.quote(tmp)} telegram-bot:{shlex.quote(tmp)}")
                request["files"].append({"field": f"f{i}", "path": tmp})
        elif media is not None:
            tmp = f"/tmp/channel_media{_as_path(media).suffix.lower()}"
            subprocess.run(scp + [str(media), f"{self.robot}:{tmp}"], check=True, timeout=180)
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


def make_transport(kind: str = "robot", robot: str = "ros2@10.1.1.22", token: Optional[str] = None):
    """Транспорт по имени: robot — через контейнер робота на Sound Pi, direct — через токен студии.

    robot остаётся умолчанием: студия и робот ведут канал по-разному, но старый путь не должен ломаться.
    """
    if kind == "direct":
        return DirectTransport(token)
    if kind == "robot":
        return RobotTransport(robot)
    raise ValueError(f"неизвестный транспорт {kind!r}: нужен robot или direct")


def number_of(post: Post) -> int:
    return int(post.name[:3])


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


def describe(method: str, params: dict, media) -> str:
    """Что будет отправлено — для --dry-run (без токена, он сюда не попадает)."""
    out = [f"  метод:  {method}", f"  чат:    {params.get('chat_id')}"]
    for item in media if isinstance(media, list) else ([media] if media is not None else []):
        out.append(f"  файл:   {item} (" + (f"{item.stat().st_size} байт)" if item.exists() else "НЕТ ФАЙЛА)"))
    items = params.get("media")
    if items:
        out.append("  элементы альбома: " + ", ".join(i["type"] for i in items) + " (подпись на первом)")
        text = items[0].get("caption")
    else:
        text = params.get("caption") or params.get("text") or params.get("title") or params.get("description")
    if "message_id" in params:
        out.append(f"  message_id: {params['message_id']}")
    if text is not None:
        out.append("  текст:")
        out.extend("    | " + line for line in str(text).splitlines())
    return "\n".join(out)
