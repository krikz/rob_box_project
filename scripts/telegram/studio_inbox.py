"""Файлы от Шифу: фото, видео, GIF и документы в личке студии (#4513).

Шифу 06.11: «материал он может присылать студии и сам». Правила:
  * принимаем только из чата Шифу (``SHIFU_CHAT_ID``) — чужие файлы студия не трогает;
  * подпись медиа-сообщения лежит в ``message.caption``, а не в ``text`` (Bot API) — раньше студия читала
    только ``text`` и видео с подписью молча пропускала;
  * файл скачивается через ``getFile``: ботам доступны файлы до 20 МБ. Больше — честный ответ, не тишина;
  * файл кладётся в ``run/media/inbox/``; индикатор пишет «Получил файл X, использую в черновике Y»;
  * перед публикацией людей на присланном материале проверяют по правилу приватности канала
    (``docs/process/telegram-channel.md``): запись черновика получает ``privacy_check``.

Сообщения, которые студия принять не может (стикер, голосовое, кружок, контакт…), получают честный
ответ «такое пока не умею» — молча не пропускаем ничего.
"""

from __future__ import annotations

import re
import time
from pathlib import Path
from typing import Optional

BOT_FILE_LIMIT = 20 * 1024 * 1024  # Bot API getFile: «bots can download files of up to 20MB»
MEDIA_FIELDS = ("video", "animation", "document", "photo")
UNSUPPORTED = {
    "sticker": "стикер",
    "voice": "голосовое",
    "audio": "аудио",
    "video_note": "кружок",
    "contact": "контакт",
    "location": "место",
    "venue": "место",
    "poll": "опрос",
    "dice": "кубик",
}
DEFAULT_EXT = {"video": ".mp4", "animation": ".mp4", "photo": ".jpg", "document": ".bin"}
SAFE_RE = re.compile(r"[^A-Za-z0-9А-Яа-яЁё_.-]+")
PRIVACY_NOTE = "материал от Шифу: перед публикацией проверить людей на кадре (лица мастерской — только с согласия)"


def media_of(message: dict) -> Optional[dict]:
    """Файл сообщения: {"kind", "file_id", "size", "name"} или None. Фото — самый большой размер."""
    for field in MEDIA_FIELDS:
        item = message.get(field)
        if not item:
            continue
        if field == "photo":
            item = max(item, key=lambda p: (p.get("file_size") or 0, p.get("width") or 0))
        name = item.get("file_name") or ""
        return {
            "kind": field,
            "file_id": item.get("file_id") or "",
            "size": int(item.get("file_size") or 0),
            "name": name,
            "mime": item.get("mime_type") or "",
        }
    return None


def unsupported_kind(message: dict) -> str:
    """Что прислано, если это не текст и не файл: «стикер», «голосовое»… Пусто — не из таких."""
    for field, label in UNSUPPORTED.items():
        if message.get(field):
            return label
    return ""


def local_name(media: dict, message_id: int, file_path: str = "") -> str:
    """Имя файла в inbox: из имени документа или пути Telegram, с номером сообщения — без коллизий."""
    base = media.get("name") or Path(file_path or "").name
    suffix = Path(base).suffix.lower() or Path(file_path or "").suffix.lower() or DEFAULT_EXT.get(media["kind"], "")
    stem = SAFE_RE.sub("_", Path(base).stem)[:40].strip("_") or media["kind"]
    return f"{message_id}-{stem}{suffix}"


def mb(size: int) -> str:
    return f"{size / 1024 / 1024:.1f} МБ"


def too_big(media: dict) -> str:
    """Причина отказа, если файл больше лимита бота. Пустая строка — влезает."""
    if media.get("size", 0) > BOT_FILE_LIMIT:
        return (
            f"файл {mb(media['size'])}, а бот может скачать только до {mb(BOT_FILE_LIMIT)} (лимит Bot API) — "
            "пришли ссылкой или положи в репо"
        )
    return ""


def download(bot, media: dict, inbox: Path, message_id: int) -> tuple:
    """getFile → файл в inbox. (путь или None, причина)."""
    reason = too_big(media)
    if reason:
        return None, reason
    info = bot.get_file(media["file_id"])
    if not info.get("ok"):
        description = info.get("description") or "Telegram не дал файл"
        if "too big" in description.lower():
            return None, too_big({"size": BOT_FILE_LIMIT + 1}) or description
        return None, f"getFile: {description}"
    file_path = (info.get("result") or {}).get("file_path") or ""
    if not file_path:
        return None, "getFile не вернул путь файла"
    inbox.mkdir(parents=True, exist_ok=True)
    dest = inbox / local_name(media, message_id, file_path)
    reason = bot.download_file(file_path, dest)
    if reason:
        return None, f"скачивание: {reason}"
    return dest, ""


def attach_job(bot, state_path: Path, post_id: str, media: dict, caption: str, message_id: int, out=print) -> bool:
    """Скачать файл Шифу и поставить его медиа черновика (или честно сказать, почему нет)."""
    import channel_studio as cs
    import studio_jobs as jobs
    import studio_media as smedia
    import studio_progress as sp

    label = media.get("name") or {"photo": "фото", "video": "видео", "animation": "GIF"}.get(media["kind"], "файл")
    progress = sp.Progress(bot, cs.SHIFU_CHAT_ID, f"Файл от Шифу: {label}", reply_to=message_id)
    started = time.monotonic()
    size = f", {mb(media['size'])}" if media.get("size") else ""
    progress.step(f"Получил файл {label}{size}: скачиваю (getFile)", "upload_document")
    path, reason = download(bot, media, jobs.run_dir() / "media" / "inbox", message_id)
    if path is None:
        progress.fail(reason)
        return _done(state_path, post_id, "file", progress, started, out)
    progress.note(f"Получил файл {path.name} ({mb(path.stat().st_size)}): лежит в run/media/inbox/")
    if caption:
        progress.note(f"Подпись: «{caption[:120]}»")
    if not post_id:
        progress.finish(f"Получил файл {path.name}. Черновика на обсуждении нет — положил в inbox, возьму в следующий")
        return _done(state_path, post_id, "file", progress, started, out, ok=True)
    if smedia.kind_of(path) is None:
        progress.finish(
            f"Получил файл {path.name}, сохранил. В черновик {post_id} такой тип не ставлю: только фото, GIF, видео"
        )
        return _done(state_path, post_id, "file", progress, started, out, ok=True)
    record = (cs.st.load_state(state_path).get("posts") or {}).get(post_id) or {}
    progress.step(f"Получил файл {path.name}, использую в черновике {post_id}", "upload_video")
    updated = dict(record, id=post_id, media_path=str(path), media_choice=f"inbox:{path.name}")
    ok, reason = jobs.update_draft(bot, updated, cs.SHIFU_CHAT_ID, new_media=path)
    if not ok:
        progress.fail(reason)
        return _done(state_path, post_id, "file", progress, started, out)
    fields = {"media_path": str(path), "media_choice": f"inbox:{path.name}", "message_kind": "media"}
    jobs.save_fields(state_path, post_id, dict(fields, privacy_check=PRIVACY_NOTE))
    progress.finish(
        f"Готово: файл {path.name} — медиа черновика {post_id}. Отметил: людей на кадре проверить до публикации"
    )
    return _done(state_path, post_id, "file", progress, started, out, ok=True)


def _done(state_path, post_id, action, progress, started, out, ok=False) -> bool:
    import studio_jobs as jobs

    jobs.report(state_path, post_id, action, progress, started, out, ok)
    return ok
