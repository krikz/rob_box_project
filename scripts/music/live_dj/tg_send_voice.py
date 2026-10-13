"""Отправка OGG в Telegram без утечки токена. Запускается внутри контейнера telegram-bot.

Токен сидит в URL (bot<TOKEN>), а traceback requests печатает URL целиком, поэтому
исключения наружу идут только через mask(): тип + текст с вырезанным токеном.
Env: TELEGRAM_BOT_TOKEN, CHAT, B (имя файла без расширения), C (подпись),
TG_RETRIES (по умолчанию 4), TG_RETRY_PAUSE (сек, по умолчанию 3).
Exit 0 — отправлено (HTTP 200), 1 — нет.
"""
import os
import sys
import time

import requests


def mask(text, token):
    """Вырезать токен из любого текста."""
    text = str(text)
    if token:
        text = text.replace(token, "<tok>")
    return text


def send_voice(token, chat, caption, path, retries=4, pause=3.0, post=None, sleep=time.sleep):
    """Вернуть (ok, сообщение). Сообщение гарантированно без токена."""
    post = post or requests.post
    last = "no attempts"
    for attempt in range(1, retries + 1):
        try:
            with open(path, "rb") as f:
                r = post(f"https://api.telegram.org/bot{token}/sendVoice",
                         data={"chat_id": chat, "caption": caption},
                         files={"voice": f}, timeout=120)
            last = mask(f"{r.status_code} {r.text[:120]}", token)
            if r.status_code == 200:
                return True, last
        except Exception as e:  # noqa: BLE001 — любой сбой сети/файла маскируем
            last = mask(f"{type(e).__name__}: {e}", token)
        print(f"attempt {attempt}/{retries} failed: {last}", file=sys.stderr)
        if attempt < retries:
            sleep(pause)
    return False, last


def main():
    token = os.environ["TELEGRAM_BOT_TOKEN"]
    ok, msg = send_voice(
        token, os.environ["CHAT"], os.environ["C"], f"/tmp/{os.environ['B']}.ogg",
        retries=int(os.environ.get("TG_RETRIES", "4")),
        pause=float(os.environ.get("TG_RETRY_PAUSE", "3")))
    print(msg)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
