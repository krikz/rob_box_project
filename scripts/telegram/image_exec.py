"""Вызов MiniMax image-01 для подложек канала. Запускать внутри контейнера telegram-bot (его копирует channel_bg).

Вход (stdin): JSON {"prompt": str, "aspect_ratio": "16:9", "seed": int|null}. Ключ берётся из MINIMAX_API_KEY
контейнера и не печатается. Печатается одна строка JSON: {"ok": bool, "image_base64": str|null, "description": str}.
Документация: https://platform.minimax.io/docs/api-reference/image-generation-t2i (POST /v1/image_generation).
"""

import json
import os
import sys

import requests

URL = "https://api.minimax.io/v1/image_generation"


def main() -> None:
    req = json.load(sys.stdin)
    body = {
        "model": "image-01",
        "prompt": str(req["prompt"])[:1500],
        "aspect_ratio": req.get("aspect_ratio", "16:9"),
        "n": 1,
        "response_format": "base64",
    }
    if req.get("seed") is not None:
        body["seed"] = int(req["seed"])
    try:
        r = requests.post(
            URL,
            json=body,
            headers={"Authorization": "Bearer " + os.environ["MINIMAX_API_KEY"]},
            timeout=120,
        )
        data = r.json()
        base = data.get("base_resp") or {}
        images = (data.get("data") or {}).get("image_base64") or []
        ok = base.get("status_code") == 0 and bool(images)
        desc = "" if ok else f"{base.get('status_code')}: {base.get('status_msg')}"
        print(json.dumps({"ok": ok, "image_base64": images[0] if ok else None, "description": desc}))
    except Exception as e:  # не печатать заголовки: в них ключ
        print(json.dumps({"ok": False, "image_base64": None, "description": "ERR " + type(e).__name__}))


if __name__ == "__main__":
    main()
