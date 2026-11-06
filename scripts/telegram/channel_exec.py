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
    "sendVideo",
    "sendMediaGroup",
    "deleteMessage",
    "setChatPhoto",
    "setChatTitle",
    "setChatDescription",
    "pinChatMessage",
}


def main() -> None:
    req = json.load(sys.stdin)
    method, params = req["method"], req["params"]
    chat = int(params.get("chat_id", 0))
    draft_methods = {"sendMessage", "sendPhoto", "sendAnimation", "sendVideo", "sendMediaGroup"}
    draft_ok = chat == DRAFT_CHAT_ID and method in draft_methods
    if method not in METHODS or (chat != CHANNEL_ID and not draft_ok):
        print(json.dumps({"ok": False, "description": "метод или chat_id не разрешены"}))
        return
    url = "https://api.telegram.org/bot%s/%s" % (os.environ["TELEGRAM_BOT_TOKEN"], method)
    try:
        files = req.get("files") or ([req["file"]] if req.get("file") else [])
        if files:
            form = {k: (json.dumps(v) if isinstance(v, (bool, dict, list)) else v) for k, v in params.items()}
            handles = [(f["field"], open(f["path"], "rb")) for f in files]
            try:
                r = requests.post(url, data=form, files=dict(handles), timeout=180)
            finally:
                for _, fh in handles:
                    fh.close()
        else:
            r = requests.post(url, json=params, timeout=30)
        data = r.json()
        result = data.get("result")
        if isinstance(result, list):  # sendMediaGroup: массив Message
            ids = [m.get("message_id") for m in result if isinstance(m, dict)]
        else:
            ids = [result.get("message_id")] if isinstance(result, dict) else []
        ids = [i for i in ids if i]
        print(
            json.dumps(
                {
                    "ok": bool(data.get("ok")),
                    "message_id": ids[0] if ids else None,
                    "message_ids": ids,
                    "description": data.get("description", ""),
                }
            )
        )
    except Exception as e:  # не печатать URL: в нём токен
        print(json.dumps({"ok": False, "description": "ERR " + type(e).__name__}))


if __name__ == "__main__":
    main()
