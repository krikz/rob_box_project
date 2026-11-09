"""REST-плагин zenoh-роутера (:8000, ADR-0017): put, get и подписка SSE — без библиотеки zenoh, на stdlib.

Агент публикует состояние в роутер своей Pi (`http://127.0.0.1:8000`), наблюдатель workflow читает с роутера
Main Pi. Хранилища (storage_manager) в роутерах нет намеренно (#3669), поэтому `get` вернёт сохранённое,
только если когда-нибудь заведут узкое хранилище на `robots/*/ops/chuangong/**`; без него наблюдатель ждёт
следующей публикации через `subscribe`.
"""

from __future__ import annotations

import json
import socket
import time
import urllib.error
import urllib.request
from typing import Any, Iterator


class ZenohError(RuntimeError):
    """REST-плагин недоступен или ответил ошибкой."""


def _value(sample: dict[str, Any]) -> Any:
    val = sample.get("value")
    if isinstance(val, str):
        try:
            return json.loads(val)
        except ValueError:
            return val
    return val


class ZenohRest:
    def __init__(self, base_url: str, timeout: float = 10.0) -> None:
        self.base = base_url.rstrip("/")
        self.timeout = timeout

    def put(self, key: str, value: Any) -> None:
        data = json.dumps(value, ensure_ascii=False, sort_keys=True).encode()
        req = urllib.request.Request(
            f"{self.base}/{key}", data=data, method="PUT", headers={"Content-Type": "application/json"}
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:  # noqa: S310
                if resp.status >= 300:
                    raise ZenohError(f"PUT {key}: HTTP {resp.status}")
        except (urllib.error.URLError, OSError) as exc:
            raise ZenohError(f"PUT {key}: {exc}") from exc

    def get(self, selector: str) -> list[tuple[str, Any]]:
        req = urllib.request.Request(f"{self.base}/{selector}", headers={"Accept": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:  # noqa: S310
                samples = json.loads(resp.read() or b"[]")
        except (urllib.error.URLError, OSError, ValueError) as exc:
            raise ZenohError(f"GET {selector}: {exc}") from exc
        return [(s.get("key", ""), _value(s)) for s in samples if isinstance(s, dict)]

    def subscribe(self, selector: str, timeout_s: float) -> Iterator[tuple[str, Any]]:
        """Сэмплы PUT по селектору, пока не истечёт `timeout_s` (Server-Sent Events REST-плагина)."""
        deadline = time.monotonic() + timeout_s
        req = urllib.request.Request(f"{self.base}/{selector}", headers={"Accept": "text/event-stream"})
        try:
            resp = urllib.request.urlopen(req, timeout=max(1.0, timeout_s))  # noqa: S310
        except (urllib.error.URLError, OSError) as exc:
            raise ZenohError(f"SSE {selector}: {exc}") from exc
        with resp:
            yield from _sse_samples(resp, deadline)


def _sse_samples(resp: Any, deadline: float) -> Iterator[tuple[str, Any]]:
    while time.monotonic() < deadline:
        try:
            line = resp.readline()
        except (socket.timeout, TimeoutError):
            return  # после таймаута сокет в urllib дальше не читается
        if not line:
            return
        text = line.decode("utf-8", "replace").strip()
        if not text.startswith("data:"):
            continue
        try:
            sample = json.loads(text[5:].strip())
        except ValueError:
            continue
        if isinstance(sample, dict):
            yield sample.get("key", ""), _value(sample)
