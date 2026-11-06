"""Студия канала @robbox_project: кнопки, long-poll апдейтов, состояние.

Студия живёт на отдельной машине и ведёт канал своим ботом (@robboxcontentbot, токен TG_STUDIO_BOT_TOKEN).
Наружу она общается с Шифу в его личке (TG_SHIFU_CHAT_ID): присылает идеи и черновики с кнопками
и ждёт решения. Решения — callback_query от Telegram, их разбирает long-poll getUpdates.

Правила ведения канала — docs/process/telegram-channel.md, бриф студии — docs/process/channel-studio-brief.md.

Токен бота не печатается: в сообщениях об ошибках URL вырезается, как в channel_exec.py.

Команды:
  channel_studio.py poll            — разобрать накопленные апдейты (один проход getUpdates)
  channel_studio.py watch           — долгий long-poll в цикле (для контейнера студии)
  channel_studio.py state           — показать состояние (идеи, offset)
  channel_studio.py ideas <файл>    — прислать Шифу список идей с кнопками (файл — JSON, как в state.json)
  channel_studio.py post <файл>     — прислать Шифу черновик поста с кнопками «Опубликовать/Отклонить»
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from pathlib import Path
from typing import Callable, Optional

REPO = Path(__file__).resolve().parents[2]
# Состояние студии — локальное (смещение getUpdates, идеи, статусы), в git не попадает:
# путь по умолчанию внутри docker/studio/, который закрыт .gitignore.
STATE_PATH = Path(os.environ.get("STUDIO_STATE", REPO / "docker" / "studio" / "state.json"))

# Действия кнопок. Ключ попадает в callback_data (максимум 64 байта у Telegram).
ACTION_TAKE = "take"  # «Берём»
ACTION_DISCUSS = "discuss"  # «Обсудить»
ACTION_SKIP = "skip"  # «Мимо»
ACTION_PUBLISH = "publish"  # «Опубликовать»
ACTION_REJECT = "reject"  # «Отклонить»

IDEA_ACTIONS = (ACTION_TAKE, ACTION_DISCUSS, ACTION_SKIP)
POST_ACTIONS = (ACTION_PUBLISH, ACTION_REJECT)

# Личка Шифу: сюда студия шлёт идеи и черновики, отсюда приходят кнопки и ответы.
SHIFU_CHAT_ID = int(os.environ.get("TG_SHIFU_CHAT_ID", "495039871"))

# callback_data: studio:<что>:<id>:<действие>
CALLBACK_RE = re.compile(r"^studio:(idea|post):([A-Za-z0-9_-]{1,32}):([a-z]{2,10})$")


# --------------------------------------------------------------------------- состояние


def load_state(path: Path = STATE_PATH) -> dict:
    """Состояние студии. Битый файл — не повод падать: начинаем с пустого."""
    if not path.exists():
        return {"offset": 0, "ideas": [], "posts": {}}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (ValueError, OSError):
        return {"offset": 0, "ideas": [], "posts": {}}
    data.setdefault("offset", 0)
    data.setdefault("ideas", [])
    data.setdefault("posts", {})
    return data


def save_state(state: dict, path: Path = STATE_PATH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(path)


def find_idea(state: dict, idea_id: str) -> Optional[dict]:
    for idea in state.get("ideas", []):
        if idea.get("id") == idea_id:
            return idea
    return None


# --------------------------------------------------------------------------- Bot API


class StudioBot:
    """Тонкая обёртка над Bot API бота студии. Токен из окружения, наружу не печатается."""

    def __init__(self, token: Optional[str] = None, timeout: int = 60, session=None):
        self.token = token or os.environ.get("TG_STUDIO_BOT_TOKEN", "")
        if not self.token:
            raise RuntimeError("TG_STUDIO_BOT_TOKEN не задан: студии нужен свой бот, токен робота не подходит")
        self.timeout = timeout
        self._session = session

    def _post(self, method: str, params: dict) -> dict:
        import requests  # локальный импорт: сеть нужна только вживую

        url = f"https://api.telegram.org/bot{self.token}/{method}"
        session = self._session or requests.Session()
        try:
            r = session.post(url, json=params, timeout=self.timeout)
            payload = r.json()
        except Exception as e:  # не печатать URL: в нём токен
            return {"ok": False, "description": f"ERR {type(e).__name__}"}
        return {"ok": bool(payload.get("ok")), "result": payload.get("result"), "description": payload.get("description", "")}

    def get_updates(self, offset: int, timeout: int = 30) -> list:
        """Апдейты с offset. Пустой offset=0 Telegram не примет — сначала сдвигаем на -1."""
        result = self._post("getUpdates", {"offset": offset, "timeout": timeout, "allowed_updates": ["callback_query"]})
        if not result.get("ok"):
            return []
        items = result.get("result") or []
        return items if isinstance(items, list) else []

    def answer_callback(self, callback_id: str, text: str = "") -> dict:
        params = {"callback_query_id": callback_id}
        if text:
            params["text"] = text[:200]
        return self._post("answerCallbackQuery", params)

    def send_message(self, chat_id: int, text: str, keyboard: Optional[dict] = None, parse_mode: str = "HTML") -> dict:
        params = {
            "chat_id": chat_id,
            "text": text,
            "parse_mode": parse_mode,
            # Bot API 10.3: link_preview_options вместо устаревшего disable_web_page_preview
            "link_preview_options": {"is_disabled": True},
        }
        if keyboard:
            params["reply_markup"] = keyboard
        return self._post("sendMessage", params)

    def edit_message_text(self, chat_id: int, message_id: int, text: str, keyboard: Optional[dict] = None) -> dict:
        params = {"chat_id": chat_id, "message_id": message_id, "text": text, "parse_mode": "HTML"}
        if keyboard:
            params["reply_markup"] = keyboard
        return self._post("editMessageText", params)


def keyboard(rows: list) -> dict:
    """Клавиатура из рядов кнопок: [[{"text": ..., "callback_data": ...}], ...]."""
    return {"inline_keyboard": rows}


# --------------------------------------------------------------------------- кнопки


def idea_keyboard(idea_id: str) -> dict:
    return keyboard(
        [
            [
                {"text": "Берём", "callback_data": f"studio:idea:{idea_id}:{ACTION_TAKE}"},
                {"text": "Обсудить", "callback_data": f"studio:idea:{idea_id}:{ACTION_DISCUSS}"},
                {"text": "Мимо", "callback_data": f"studio:idea:{idea_id}:{ACTION_SKIP}"},
            ]
        ]
    )


def post_keyboard(post_id: str) -> dict:
    return keyboard(
        [
            [
                {"text": "Опубликовать", "callback_data": f"studio:post:{post_id}:{ACTION_PUBLISH}"},
                {"text": "Отклонить", "callback_data": f"studio:post:{post_id}:{ACTION_REJECT}"},
            ]
        ]
    )


def apply_idea_action(state: dict, idea_id: str, action: str) -> str:
    """Применить решение по идее к состоянию. Возвращает текст ответа Шифу."""
    idea = find_idea(state, idea_id)
    if idea is None:
        return f"Идея {idea_id} не найдена в состоянии студии."
    if action == ACTION_TAKE:
        idea["status"] = "agreed"
        return f"Идея {idea_id} взята в работу: «{idea.get('title', '')}». Собираю материал и пришлю черновик."
    if action == ACTION_DISCUSS:
        idea["status"] = "discuss"
        return (
            f"Обсуждаем идею {idea_id}: «{idea.get('title', '')}». Напишите, что поправить в подаче "
            f"и что показать — учту в черновике."
        )
    if action == ACTION_SKIP:
        idea["status"] = "rejected"
        return f"Идея {idea_id} отложена."
    return f"Неизвестное действие {action}."


def apply_post_action(state: dict, post_id: str, action: str) -> str:
    """Применить решение по посту к состоянию. Возвращает текст ответа Шифу."""
    posts = state.setdefault("posts", {})
    record = posts.setdefault(post_id, {})
    if action == ACTION_PUBLISH:
        record["status"] = "approved"
        return f"Пост {post_id} одобрен — ставлю в очередь канала на ближайший слот."
    if action == ACTION_REJECT:
        record["status"] = "rejected"
        return f"Пост {post_id} отклонён."
    return f"Неизвестное действие {action}."


def handle_callback(state: dict, callback: dict) -> str:
    """Разобрать callback_query: обновить состояние и вернуть текст ответа Шифу."""
    data = (callback.get("data") or "")
    match = CALLBACK_RE.match(data)
    if not match:
        return "Непонятная кнопка."
    kind, item_id, action = match.groups()
    if kind == "idea":
        if action not in IDEA_ACTIONS:
            return "Непонятное действие."
        return apply_idea_action(state, item_id, action)
    if action not in POST_ACTIONS:
        return "Непонятное действие."
    return apply_post_action(state, item_id, action)


# --------------------------------------------------------------------------- опрос апдейтов


def poll_once(bot: StudioBot, state: dict, state_path: Path, timeout: int = 30, out=print) -> int:
    """Один проход getUpdates: разобрать все апдейты, сдвинуть offset, сохранить состояние."""
    updates = bot.get_updates(state.get("offset", 0), timeout=timeout)
    handled = 0
    for update in updates:
        update_id = update.get("update_id", 0)
        state["offset"] = max(state.get("offset", 0), update_id + 1)
        callback = update.get("callback_query")
        if callback:
            answer = handle_callback(state, callback)
            bot.answer_callback(callback.get("id", ""), answer[:200])
            message = callback.get("message") or {}
            chat_id = message.get("chat", {}).get("id")
            if chat_id:
                bot.send_message(chat_id, answer)
            out(f"обработано: {callback.get('data')} → {answer}")
            handled += 1
            continue
        message = update.get("message")
        if message:
            # Текст от Шифу — это обсуждение идеи. Сохраняем, чтобы агент подхватил.
            note = capture_message(state, message)
            if note:
                out(f"сообщение от Шифу сохранено: {note[:60]}")
                handled += 1
    if handled or updates:
        save_state(state, state_path)
    return handled


def capture_message(state: dict, message: dict) -> str:
    """Сохранить текстовое сообщение Шифу в состояние (фаза обсуждения)."""
    text = (message.get("text") or "").strip()
    if not text:
        return ""
    chat = message.get("chat") or {}
    if chat.get("id") != SHIFU_CHAT_ID:
        return ""
    notes = state.setdefault("messages", [])
    notes.append({"at": message.get("date"), "text": text[:2000]})
    del notes[:-20]  # последние 20 сообщений
    return text


def watch(bot: StudioBot, state_path: Path, timeout: int = 30, once: bool = False, out=print) -> int:
    """Цикл long-poll. once=True — один проход (для cron)."""
    total = 0
    while True:
        state = load_state(state_path)
        total += poll_once(bot, state, state_path, timeout=timeout, out=out)
        if once:
            return total
        time.sleep(1)


# --------------------------------------------------------------------------- отправка идей и постов


def send_ideas(bot: StudioBot, ideas: list, chat_id: int, out=print) -> int:
    """Прислать Шифу список идей: по кнопке на каждую. Только идеи, не готовые посты."""
    if not ideas:
        out("идей нет — Шифу ничего не шлю")
        return 0
    sent = 0
    for idea in ideas:
        idea_id = str(idea.get("id") or "")
        if not idea_id:
            continue
        title = idea.get("title", "")
        facts = idea.get("facts") or []
        show = idea.get("show") or ""
        lines = [f"<b>Идея {idea_id}</b>: {title}", ""]
        if facts:
            lines.append("Факты и источники:")
            lines.extend(f"— {f}" for f in facts)
            lines.append("")
        if show:
            lines.append(f"Что показать: {show}")
        lines.append("")
        lines.append("Берём, обсудить или мимо?")
        result = bot.send_message(chat_id, "\n".join(lines), keyboard=idea_keyboard(idea_id))
        if result.get("ok"):
            sent += 1
        else:
            out(f"идея {idea_id} не отправлена: {result.get('description')}")
    out(f"идей отправлено: {sent}")
    return sent


def send_post(bot: StudioBot, post: dict, chat_id: int, out=print) -> int:
    """Прислать Шифу черновик поста с кнопками «Опубликовать/Отклонить»."""
    post_id = str(post.get("id") or "")
    if not post_id:
        out("у поста нет id")
        return 0
    body = post.get("body_html") or post.get("body") or ""
    title = post.get("title", "")
    text = f"<b>Черновик {post_id}</b>" + (f": {title}" if title else "") + "\n\n" + body
    result = bot.send_message(chat_id, text, keyboard=post_keyboard(post_id))
    if not result.get("ok"):
        out(f"черновик не отправлен: {result.get('description')}")
        return 0
    out(f"черновик {post_id} отправлен")
    return 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--state", type=Path, default=STATE_PATH, help="файл состояния студии (state.json)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_poll = sub.add_parser("poll", help="один проход getUpdates (для cron)")
    p_poll.add_argument("--timeout", type=int, default=30, help="таймаут long-poll, с")
    p_poll.add_argument("--token", help="токен бота студии (по умолчанию TG_STUDIO_BOT_TOKEN)")

    p_watch = sub.add_parser("watch", help="долгий long-poll в цикле (для контейнера студии)")
    p_watch.add_argument("--timeout", type=int, default=30, help="таймаут long-poll, с")
    p_watch.add_argument("--once", action="store_true", help="один проход и выход")
    p_watch.add_argument("--token", help="токен бота студии (по умолчанию TG_STUDIO_BOT_TOKEN)")

    sub.add_parser("state", help="показать состояние студии")

    p_ideas = sub.add_parser("ideas", help="прислать Шифу список идей с кнопками")
    p_ideas.add_argument("file", type=Path, help="JSON со списком идей")
    p_ideas.add_argument("--chat-id", type=int, default=int(os.environ.get("TG_SHIFU_CHAT_ID", "495039871")))
    p_ideas.add_argument("--token", help="токен бота студии (по умолчанию TG_STUDIO_BOT_TOKEN)")

    p_post = sub.add_parser("post", help="прислать Шифу черновик поста с кнопками")
    p_post.add_argument("file", type=Path, help="JSON с черновиком")
    p_post.add_argument("--chat-id", type=int, default=int(os.environ.get("TG_SHIFU_CHAT_ID", "495039871")))
    p_post.add_argument("--token", help="токен бота студии (по умолчанию TG_STUDIO_BOT_TOKEN)")

    args = ap.parse_args()
    state_path = args.state

    if args.cmd == "state":
        print(json.dumps(load_state(state_path), ensure_ascii=False, indent=2))
        return 0

    bot = StudioBot(args.token)

    if args.cmd == "poll":
        return 0 if poll_once(bot, load_state(state_path), state_path, timeout=args.timeout) >= 0 else 1
    if args.cmd == "watch":
        watch(bot, state_path, timeout=args.timeout, once=args.once)
        return 0
    if args.cmd == "ideas":
        data = json.loads(args.file.read_text(encoding="utf-8"))
        ideas = data.get("ideas", data) if isinstance(data, dict) else data
        return 0 if send_ideas(bot, ideas, args.chat_id) else 1
    if args.cmd == "post":
        data = json.loads(args.file.read_text(encoding="utf-8"))
        post = data.get("post", data) if isinstance(data, dict) else data
        return 0 if send_post(bot, post, args.chat_id) else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
