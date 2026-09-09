#!/usr/bin/env python3
"""Telegram e2e-тест rob_box-бота через тестовый user-аккаунт (issue #1196, L3).

Полный e2e по каналу  чат → бот → LLM → ответ в чат. Отличие от
голосового e2e (e2e_voice_test.sh): здесь сообщение уходит через реальный
Telegram Bot API от лица ТЕСТОВОГО USER-аккаунта (MTProto/Telethon), а не
инъекцией в /voice/stt/result. Ловит разрывы на всём пути:
Telegram → telegram-bot → dialogue_node → LLM → send_message → чат.

Ограничения (проверено 13.08, issue #1196):
  * бот не может сам себе слать сообщения (User_bot_to_bot_disabled);
  * боты в группах не видят сообщения других ботов;
  * поэтому нужен user-аккаунт (НЕ личный аккаунт Шифу) и отдельная
    тестовая группа, где состоят rob_box-бот и тестовый user.

Секреты — ТОЛЬКО env (не в репо):
  TG_TEST_API_ID       — api_id с my.telegram.org
  TG_TEST_API_HASH     — api_hash с my.telegram.org
  TG_TEST_SESSION      — строка сессии Telethon (или путь к .session файлу)
  TG_TEST_CHAT_ID      — chat_id тестовой группы (например -1001234567890)
  TG_BOT_USERNAME      — username бота (например @rob_box_test_bot)
  TG_TEST_WAIT_ANSWER  — сек ожидания ответа (default 60)
  TG_TEST_VERBOSE      — 1 = печатать сырой диалог

Сценарии:
  --wake      текст с wake word («Робот, ...») → ждём ответ бота в чате
  --no-wake   текст без wake word → LLM отвечает (после фикса потоков #1195)
  --voice     голосовое сообщение → распознавание → ответ (требует файл .ogg)

Usage:
  TG_TEST_API_ID=123 TG_TEST_API_HASH=abc ... python3 telegram_userbot_e2e.py --wake
  TG_TEST_API_ID=123 ... python3 telegram_userbot_e2e.py --no-wake --text "продолжай"
  TG_TEST_API_ID=123 ... python3 telegram_userbot_e2e.py --voice --file /tmp/cmd.ogg

Exit: 0 = PASS, 1 = FAIL, 2 = SKIP (нет секретов), 3 = таймаут/нет ответа.
"""

from __future__ import annotations

import argparse
import asyncio
import os
import sys
from datetime import datetime, timedelta
from typing import Any

try:
    from telethon import TelegramClient
except ImportError:  # pragma: no cover — скрипт запускается на 249/CI с pip install telethon
    TelegramClient = None  # type: ignore[assignment,misc]


def _env(name: str) -> str:
    return os.getenv(name, "").strip()


def _missing_secrets() -> list[str]:
    return [
        name
        for name in ("TG_TEST_API_ID", "TG_TEST_API_HASH", "TG_TEST_SESSION")
        if not _env(name)
    ]


def _session_arg(session: str) -> str:
    """Telethon session: путь к файлу (если заканчивается на .session) или строка."""
    if session.endswith(".session"):
        return session
    return session


async def _wait_for_bot_reply(
    client: "Any",
    chat_id: int,
    sent_ts: datetime,
    wait_seconds: int,
    bot_username: str,
    verbose: bool = False,
) -> tuple[bool, str]:
    """Ждём сообщение бота в чате ПОСЛЕ sent_ts.

    Возвращает (найдено, текст_ответа_или_причина).
    """
    deadline = asyncio.get_event_loop().time() + wait_seconds
    while asyncio.get_event_loop().time() < deadline:
        try:
            async for msg in client.iter_messages(chat_id, limit=10):
                # Сообщение бота, пришедшее после нашего send_message
                if msg.date and msg.date >= sent_ts:
                    sender = msg.sender
                    if sender is None:
                        continue
                    username = getattr(sender, "username", None) or ""
                    is_bot = bool(getattr(sender, "bot", False))
                    if verbose:
                        print(f"  [msg] from={username or sender.id} bot={is_bot} date={msg.date} text={msg.text!r}")
                    if is_bot or (bot_username and username and username.lower() in bot_username.lower()):
                        text = (msg.text or "").strip()
                        if text:
                            return True, text
        except Exception as exc:  # noqa: BLE001
            print(f"  [warn] iter_messages error: {exc!r}")
        await asyncio.sleep(2.0)
    return False, f"нет ответа бота за {wait_seconds}с"


async def run_wake(client: "Any", chat_id: int, args: argparse.Namespace) -> int:
    text = args.text or "Робот, привет! Как дела?"
    sent_ts = datetime.now(tz=__import__("datetime").timezone.utc)
    print(f"[wake] send: {text!r} → chat {chat_id}")
    await client.send_message(chat_id, text)
    ok, reply = await _wait_for_bot_reply(
        client, chat_id, sent_ts, args.wait, args.bot_username, args.verbose
    )
    if ok:
        print(f"[wake] ✅ ответ бота: {reply!r}")
        return 0
    print(f"[wake] ❌ {reply}")
    return 3


async def run_no_wake(client: "Any", chat_id: int, args: argparse.Namespace) -> int:
    text = args.text or "продолжай"
    sent_ts = datetime.now(tz=__import__("datetime").timezone.utc)
    print(f"[no-wake] send: {text!r} → chat {chat_id}")
    await client.send_message(chat_id, text)
    ok, reply = await _wait_for_bot_reply(
        client, chat_id, sent_ts, args.wait, args.bot_username, args.verbose
    )
    # После фикса потоков (#1195) текст без wake word из чата попадает в
    # LLM (в отличие от микрофонного gate). Тут мы это проверяем.
    if ok:
        print(f"[no-wake] ✅ LLM ответил: {reply!r}")
        return 0
    print(f"[no-wake] ❌ {reply}")
    return 3


async def run_voice(client: "Any", chat_id: int, args: argparse.Namespace) -> int:
    if not args.file or not os.path.isfile(args.file):
        print("[voice] ❌ нужен --file путь к .ogg (16k opus)")
        return 1
    sent_ts = datetime.now(tz=__import__("datetime").timezone.utc)
    print(f"[voice] send voice: {args.file} → chat {chat_id}")
    await client.send_file(chat_id, args.file, voice_note=True)
    ok, reply = await _wait_for_bot_reply(
        client, chat_id, sent_ts, args.wait, args.bot_username, args.verbose
    )
    if ok:
        print(f"[voice] ✅ распознавание → ответ: {reply!r}")
        return 0
    print(f"[voice] ❌ {reply}")
    return 3


async def _main(args: argparse.Namespace) -> int:
    missing = _missing_secrets()
    if missing:
        print(f"❌ Нет секретов: {', '.join(missing)} (env) — SKIP")
        return 2
    if TelegramClient is None:
        print("❌ telethon не установлен — pip install telethon — SKIP")
        return 2

    api_id = int(_env("TG_TEST_API_ID"))
    api_hash = _env("TG_TEST_API_HASH")
    session = _session_arg(_env("TG_TEST_SESSION"))
    chat_id = int(_env("TG_TEST_CHAT_ID"))
    bot_username = _env("TG_BOT_USERNAME")

    client = TelegramClient(session, api_id, api_hash)
    try:
        await client.start()
        me = await client.get_me()
        print(f"[auth] user={me.username or me.id}")
        if args.scenario == "wake":
            return await run_wake(client, chat_id, args)
        if args.scenario == "no-wake":
            return await run_no_wake(client, chat_id, args)
        if args.scenario == "voice":
            return await run_voice(client, chat_id, args)
        print(f"❌ неизвестный сценарий: {args.scenario}")
        return 1
    finally:
        await client.disconnect()


def main() -> int:
    parser = argparse.ArgumentParser(description="Telegram e2e через тестовый userbot (issue #1196 L3)")
    parser.add_argument("--scenario", choices=("wake", "no-wake", "voice"), default="wake")
    parser.add_argument("--text", default="", help="текст сообщения (для wake/no-wake)")
    parser.add_argument("--file", default="", help="путь к .ogg (для voice)")
    parser.add_argument("--wait", type=int, default=int(_env("TG_TEST_WAIT_ANSWER") or 60))
    parser.add_argument("--bot-username", default=_env("TG_BOT_USERNAME"))
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    args.verbose = args.verbose or _env("TG_TEST_VERBOSE") == "1"
    try:
        return asyncio.run(_main(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
