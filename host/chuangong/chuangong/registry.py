"""Минимальный клиент OCI Distribution API (registry:2 на katana, потом реестр площадки) на stdlib.

Транспорт — шов `Transport.open(method, url, headers, data) -> Response`: в проде urllib, в тестах фейк.
Чтение (агент): манифест по тегу/digest, блоб целиком с проверкой digest, блоб в файл с докачкой (Range).
Запись (CI, `scripts/build/release.py publish`): загрузка блоба, манифест под тегом, digest образа по тегу.
"""

from __future__ import annotations

import hashlib
import http.client
import io
import json
import os
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol

MANIFEST_ACCEPT = ", ".join(
    [
        "application/vnd.oci.image.manifest.v1+json",
        "application/vnd.oci.image.index.v1+json",
        "application/vnd.docker.distribution.manifest.v2+json",
        "application/vnd.docker.distribution.manifest.list.v2+json",
    ]
)
_CHUNK = 1 << 20


class RegistryError(RuntimeError):
    """Реестр не ответил так, как ожидалось (сеть, 4xx/5xx, неверный digest)."""


@dataclass
class Response:
    status: int
    headers: dict[str, str] = field(default_factory=dict)  # ключи в нижнем регистре
    body: Any = None  # объект с .read(n) или bytes

    def __post_init__(self) -> None:
        if isinstance(self.body, (bytes, bytearray)) or self.body is None:
            self.body = io.BytesIO(bytes(self.body or b""))

    def read(self, n: int = -1) -> bytes:
        return self.body.read(n)


class Transport(Protocol):
    def open(self, method: str, url: str, headers: dict[str, str], data: bytes | None) -> Response: ...


class UrllibTransport:
    def __init__(self, timeout: float = 30.0) -> None:
        self.timeout = timeout

    def open(self, method: str, url: str, headers: dict[str, str], data: bytes | None) -> Response:
        req = urllib.request.Request(url, data=data, method=method, headers=headers)
        try:
            resp = urllib.request.urlopen(req, timeout=self.timeout)  # noqa: S310 — адрес из конфига агента
        except urllib.error.HTTPError as exc:
            return Response(exc.code, {k.lower(): v for k, v in exc.headers.items()}, exc)
        except (urllib.error.URLError, OSError) as exc:
            raise RegistryError(f"{method} {url}: {exc}") from exc
        return Response(resp.status, {k.lower(): v for k, v in resp.headers.items()}, resp)


class Registry:
    def __init__(self, base_url: str, transport: Transport | None = None) -> None:
        self.base = base_url.rstrip("/")
        self.t = transport or UrllibTransport()

    def _url(self, repo: str, kind: str, ref: str) -> str:
        return f"{self.base}/v2/{repo}/{kind}/{ref}"

    def _open(self, method: str, url: str, headers: dict[str, str] | None = None, data: bytes | None = None):
        return self.t.open(method, url, headers or {}, data)

    # --- чтение -------------------------------------------------------------------------------------------

    def get_manifest(self, repo: str, ref: str) -> tuple[str, dict[str, Any]]:
        """(digest, манифест). digest считается по байтам ответа, а не по заголовку."""
        resp = self._open("GET", self._url(repo, "manifests", ref), {"Accept": MANIFEST_ACCEPT})
        if resp.status != 200:
            raise RegistryError(f"манифест {repo}:{ref}: HTTP {resp.status}")
        raw = resp.read()
        digest = "sha256:" + hashlib.sha256(raw).hexdigest()
        if ref.startswith("sha256:") and ref != digest:
            raise RegistryError(f"манифест {repo}@{ref}: реестр отдал другие байты ({digest})")
        try:
            return digest, json.loads(raw)
        except ValueError as exc:
            raise RegistryError(f"манифест {repo}:{ref}: не JSON") from exc

    def platform_manifest(self, repo: str, ref: str, arch: str = "arm64") -> dict[str, Any]:
        """Манифест образа под архитектуру (раскрывает index/manifest list)."""
        _, m = self.get_manifest(repo, ref)
        if "manifests" in m:
            picks = [x for x in m["manifests"] if (x.get("platform") or {}).get("architecture") == arch]
            if not picks:
                raise RegistryError(f"{repo}@{ref}: нет платформы {arch}")
            _, m = self.get_manifest(repo, picks[0]["digest"])
        return m

    def get_blob(self, repo: str, digest: str, max_size: int) -> bytes:
        resp = self._open("GET", self._url(repo, "blobs", digest))
        if resp.status != 200:
            raise RegistryError(f"блоб {repo}@{digest}: HTTP {resp.status}")
        data = resp.read(max_size + 1)
        if len(data) > max_size:
            raise RegistryError(f"блоб {repo}@{digest}: больше {max_size} байт")
        if "sha256:" + hashlib.sha256(data).hexdigest() != digest:
            raise RegistryError(f"блоб {repo}@{digest}: digest не совпал")
        return data

    def download_blob(self, repo: str, digest: str, part: Path) -> Path:
        """Блоб в файл `part` с докачкой: обрыв оставляет `part`, следующий вызов продолжает с его длины.

        Возвращает `part` с полным содержимым, digest проверен. Несовпадение — `part` удаляется, RegistryError.
        """
        have = part.stat().st_size if part.exists() else 0
        headers = {"Range": f"bytes={have}-"} if have else {}
        resp = self._open("GET", self._url(repo, "blobs", digest), headers)
        if resp.status == 416:  # Range за концом: файл уже полный, остаётся проверить digest
            resp = None
        elif resp.status == 200:
            have = 0
        elif resp.status != 206:
            raise RegistryError(f"блоб {repo}@{digest}: HTTP {resp.status}")
        if resp is not None:
            part.parent.mkdir(parents=True, exist_ok=True)
            with part.open("ab" if have else "wb") as fh:
                try:
                    for chunk in iter(lambda: resp.read(_CHUNK), b""):
                        fh.write(chunk)
                except (OSError, http.client.HTTPException) as exc:
                    raise RegistryError(
                        f"блоб {repo}@{digest}: обрыв на {fh.tell()} байт, докачка в следующий раз"
                    ) from exc
                finally:
                    fh.flush()
                    os.fsync(fh.fileno())
        if _file_digest(part) != digest:
            part.unlink(missing_ok=True)
            raise RegistryError(f"блоб {repo}@{digest}: digest скачанного файла не совпал — удалён")
        return part

    # --- запись (CI) --------------------------------------------------------------------------------------

    def blob_exists(self, repo: str, digest: str) -> bool:
        return self._open("HEAD", self._url(repo, "blobs", digest)).status == 200

    def push_blob(self, repo: str, data: bytes) -> str:
        digest = "sha256:" + hashlib.sha256(data).hexdigest()
        if self.blob_exists(repo, digest):
            return digest
        resp = self._open("POST", f"{self.base}/v2/{repo}/blobs/uploads/")
        loc = resp.headers.get("location")
        if resp.status != 202 or not loc:
            raise RegistryError(f"загрузка блоба в {repo}: HTTP {resp.status}")
        if loc.startswith("/"):
            loc = self.base + loc
        sep = "&" if "?" in loc else "?"
        put = self._open("PUT", f"{loc}{sep}digest={digest}", {"Content-Type": "application/octet-stream"}, data)
        if put.status != 201:
            raise RegistryError(f"загрузка блоба {digest} в {repo}: HTTP {put.status}")
        return digest

    def put_manifest(self, repo: str, ref: str, data: bytes, media_type: str) -> None:
        resp = self._open("PUT", self._url(repo, "manifests", ref), {"Content-Type": media_type}, data)
        if resp.status != 201:
            raise RegistryError(f"манифест {repo}:{ref}: HTTP {resp.status}")

    def manifest_digest(self, repo: str, tag: str) -> str:
        """digest манифеста по тегу (тот, по которому `docker pull repo@digest` вытянет тот же образ)."""
        digest, _ = self.get_manifest(repo, tag)
        return digest


def _file_digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(_CHUNK), b""):
            h.update(chunk)
    return "sha256:" + h.hexdigest()
