"""Отправка текста из stdin в Telegram. Запускать внутри контейнера telegram-bot.

Токен берётся из окружения контейнера (TELEGRAM_BOT_TOKEN) и не печатается.
chat_id — из TG_CHAT_ID, по умолчанию 495039871. Печатается только код ответа.
"""
import os
import sys

import requests

text = sys.stdin.read()
try:
    r = requests.post(
        "https://api.telegram.org/bot%s/sendMessage" % os.environ["TELEGRAM_BOT_TOKEN"],
        json={"chat_id": int(os.environ.get("TG_CHAT_ID", "495039871")), "text": text},
        timeout=20,
    )
    print(r.status_code)
except Exception as e:  # не печатать URL: в нём токен
    print("ERR", type(e).__name__)
