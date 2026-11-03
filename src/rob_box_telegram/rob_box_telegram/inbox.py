"""Входящие сообщения чата: одна стабильная строка лога на каждое (канал координатора, ADR-0150 PR-12a #3706).

Координатор читает сообщения Шифу «Клод …» из лога ``telegram-bot``
(``docker logs -f telegram-bot | grep 'tg-in'``, ``docs/process/claude-coordinator-robot-flow.md`` §5): формат строки —
контракт, его сверяет тест. «Клод …» решает та же таблица, что адресацию (``rob_box_dialog.address``).

Строка: ``chat_id=<id> coordinator=<bool> [file=<путь>] [reply_to=<цитата>] text=<текст>``. ``text`` - последним (в нём
бывают пробелы), необязательные поля - перед ним и только когда они есть. ``reply_to`` - текст сообщения, на которое
человек ответил реплаем (до 200 знаков). ``file`` - вложение (фото, документ, голосовое) владельца, сохранённое в
контейнере: координатор забирает его ``docker cp telegram-bot:<путь> .`` (токена бота в пути и в логе нет).
"""

from __future__ import annotations

import logging
import os
import time
from typing import Any, Optional

from rob_box_dialog.address import is_coordinator_call

#: Метка строки — по ней grep координатора.
TG_IN_MARK = "📨 [tg-in]"

_LOG = logging.getLogger(__name__)

#: Чаты, чьи вложения сохраняются в файл (владелец; вложения чужих чатов в диск контейнера не пишем).
SAVE_ATTACHMENTS_FOR = frozenset({495039871})
#: Папка вложений в контейнере и сколько последних файлов в ней держим (диск контейнера не резиновый).
INBOX_DIR = "/tmp/tg_inbox"
KEEP_FILES = 50
#: Предел размера вложения (лимит скачивания Bot API - 20 МБ).
MAX_FILE_BYTES = 20 * 1024 * 1024
#: Длина цитаты реплая в строке.
QUOTE_MAX = 200


def _one_line(text: str) -> str:
    return " ⏎ ".join(part.strip() for part in (text or "").strip().splitlines())


def quote_of(message: Any) -> str:
    """Текст сообщения, на которое ответили реплаем (``reply_to_message``: текст или подпись), до ``QUOTE_MAX`` знаков."""
    target = getattr(message, "reply_to_message", None)
    if target is None:
        return ""
    raw = getattr(target, "text", None) or getattr(target, "caption", None) or ""
    return raw if isinstance(raw, str) else ""


def tg_in_line(chat_id: int, text: str, reply_to: str = "", file: str = "") -> str:
    """``📨 [tg-in] chat_id=<id> coordinator=<true|false> [file=<путь>] [reply_to=<цитата>] text=<текст>``;
    переводы строк - `` ⏎ `` (одна строка). Флаг ``coordinator`` смотрит только на текст самого сообщения."""
    body = _one_line(text)
    flag = "true" if is_coordinator_call(body) else "false"
    fields = f"chat_id={int(chat_id)} coordinator={flag}"
    if file:
        fields += f" file={file}"
    quote = _one_line(reply_to)[:QUOTE_MAX].replace("text=", "text:")  # `text=` внутри цитаты сломал бы разбор строки
    if quote:
        fields += f" reply_to={quote}"
    return f"{TG_IN_MARK} {fields} text={body}"


def log_incoming(node: Any, chat_id: int, text: str, reply_to: str = "", file: str = "") -> None:
    """Строка в лог ноды (ROS-логгер, его видит ``docker logs``); без ноды - в логгер модуля."""
    line = tg_in_line(chat_id, text, reply_to, file)
    get_logger = getattr(node, "get_logger", None)
    if callable(get_logger):
        get_logger().info(line)
    else:
        _LOG.info(line)


def _prune(root: str, keep: int) -> None:
    try:
        names = sorted(os.listdir(root))
    except OSError:
        return
    for name in names[:-keep] if keep > 0 else names:
        try:
            os.remove(os.path.join(root, name))
        except OSError:
            pass


def save_attachment(data: bytes, message_id: int, ext: str, root: str = INBOX_DIR,
                    clock: Any = time.time) -> Optional[str]:
    """Вложение -> ``<root>/<ts>_<message_id>.<ext>``; путь или ``None`` (слишком большое / диск не пишется).
    Имя строит код из метки времени и номера сообщения: имя файла от отправителя в путь не попадает."""
    if not data or len(data) > MAX_FILE_BYTES:
        return None
    safe_ext = "".join(c for c in (ext or "bin").lower() if c.isalnum())[:5] or "bin"
    path = os.path.join(root, f"{int(clock())}_{int(message_id)}.{safe_ext}")
    try:
        os.makedirs(root, exist_ok=True)
        with open(path, "wb") as handle:
            handle.write(data)
    except OSError as exc:
        _LOG.warning("вложение не сохранено (%s): %r", path, exc)
        return None
    _prune(root, KEEP_FILES)
    return path


def attachment_of(message: Any) -> Optional[tuple]:
    """``(объект файла Bot API, расширение)`` вложения сообщения: фото (самый крупный размер), документ, голосовое,
    аудио, кружок; вложения нет - ``None``."""
    photos = getattr(message, "photo", None)
    if photos:
        return photos[-1], "jpg"
    for name, ext in (("voice", "ogg"), ("audio", "mp3"), ("video_note", "mp4"), ("document", "")):
        obj = getattr(message, name, None)
        if obj is not None:
            if not ext:
                ext = os.path.splitext(getattr(obj, "file_name", "") or "")[1].lstrip(".") or "bin"
            return obj, ext
    return None


async def stash_attachment(bot: Any, message: Any, chat_id: int, root: str = INBOX_DIR) -> str:
    """Сохранить вложение владельца; путь файла или ``""`` (не владелец, вложения нет, не скачалось)."""
    if int(chat_id) not in SAVE_ATTACHMENTS_FOR:
        return ""
    found = attachment_of(message)
    if found is None:
        return ""
    obj, ext = found
    try:
        data = bytes(await (await bot.get_file(obj.file_id)).download_as_bytearray())
    except Exception as exc:  # noqa: BLE001 - токен в тексте ошибки не печатаем: тип исключения достаточно
        _LOG.warning("вложение не скачалось: %s", type(exc).__name__)
        return ""
    return save_attachment(data, int(message.message_id), ext, root) or ""
