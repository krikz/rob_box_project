"""Вызов Bot API для канала. Запускать внутри контейнера telegram-bot (его копирует channel_lib.RobotTransport).

Вход (stdin): JSON {"method": ..., "params": {...}, "file": {"field": ..., "path": ...} | null}.
Токен берётся из TELEGRAM_BOT_TOKEN контейнера и не печатается. Печатается одна строка JSON:
{"ok": bool, "message_id": int|null, "description": str}. Разрешены только методы канала и только его chat_id.
"""

import json
import os
import sys

import requests

CHANNEL_ID = int(os.environ.get("CHANNEL_ID", "-1004323185327"))
DRAFT_CHAT_ID = int(os.environ.get("DRAFT_CHAT_ID", "495039871"))  # заготовки Шифу: только send*
METHODS = {
    "sendMessage",
    "sendPhoto",
    "sendAnimation",
    "setChatPhoto",
    "setChatTitle",
    "setChatDescription",
    "pinChatMessage",
}


def main() -> None:
    req = json.load(sys.stdin)
    method, params = req["method"], req["params"]
    chat = int(params.get("chat_id", 0))
    draft_ok = chat == DRAFT_CHAT_ID and method in {"sendMessage", "sendPhoto", "sendAnimation"}
    if method not in METHODS or (chat != CHANNEL_ID and not draft_ok):
        print(json.dumps({"ok": False, "description": "метод или chat_id не разрешены"}))
        return
    url = "https://api.telegram.org/bot%s/%s" % (os.environ["TELEGRAM_BOT_TOKEN"], method)
    try:
        if req.get("file"):
            form = {k: (json.dumps(v) if isinstance(v, (bool, dict, list)) else v) for k, v in params.items()}
            with open(req["file"]["path"], "rb") as fh:
                r = requests.post(url, data=form, files={req["file"]["field"]: fh}, timeout=60)
        else:
            r = requests.post(url, json=params, timeout=30)
        data = r.json()
        result = data.get("result")
        mid = result.get("message_id") if isinstance(result, dict) else None
        print(json.dumps({"ok": bool(data.get("ok")), "message_id": mid, "description": data.get("description", "")}))
    except Exception as e:  # не печатать URL: в нём токен
        print(json.dumps({"ok": False, "description": "ERR " + type(e).__name__}))


if __name__ == "__main__":
    main()
