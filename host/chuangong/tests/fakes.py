"""Фейки chuangong без сети и Docker: реестр в памяти (Range, обрыв, загрузка), docker, zenoh."""

from __future__ import annotations

import gzip
import hashlib
import io
import json
import sys
import tarfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from chuangong.registry import Response  # noqa: E402
from chuangong.stage import StageError  # noqa: E402

REG_HOST = "10.1.1.249:5000"


def digest_of(data: bytes) -> str:
    return "sha256:" + hashlib.sha256(data).hexdigest()


class _Cut(io.BytesIO):
    """Тело ответа, которое рвётся после `limit` байт (как упавший Wi‑Fi посреди скачивания)."""

    def __init__(self, data: bytes, limit: int) -> None:
        super().__init__(data)
        self.limit = limit

    def read(self, n: int = -1) -> bytes:
        if self.tell() >= self.limit:
            raise ConnectionResetError("обрыв (фейк)")
        n = self.limit - self.tell() if n < 0 else min(n, self.limit - self.tell())
        return super().read(n)


class FakeTransport:
    def __init__(self) -> None:
        self.manifests: dict[tuple[str, str], bytes] = {}
        self.blobs: dict[str, bytes] = {}
        self.calls: list[tuple[str, str, dict]] = []
        self.cut: dict[str, int] = {}  # digest -> сколько байт отдать до обрыва (один раз)

    def _upload(self, method, path, data):
        if method == "POST":  # .../blobs/uploads/ → сессия загрузки
            return Response(202, {"location": f"/v2/{path}session-1"}, b"")
        digest = path.split("?digest=", 1)[1]
        if digest_of(data) != digest:
            return Response(400, {}, b"")
        self.blobs[digest] = data
        return Response(201, {}, b"")

    def open(self, method, url, headers, data):
        self.calls.append((method, url, dict(headers)))
        path = url.split("/v2/", 1)[1]
        if "/blobs/uploads/" in path:
            return self._upload(method, path, data)
        repo, kind, ref = path.rsplit("/", 2)
        if kind == "manifests" and method == "PUT":
            self.manifests[(repo, ref)] = data
            self.manifests[(repo, digest_of(data))] = data
            return Response(201, {}, b"")
        if kind == "manifests":
            body = self.manifests.get((repo, ref))
            return Response(200, {}, body) if body is not None else Response(404, {}, b"")
        body = self.blobs.get(ref)
        if body is None:
            return Response(404, {}, b"")
        if method == "HEAD":
            return Response(200, {}, b"")
        start = int(headers["Range"].split("=")[1].rstrip("-")) if "Range" in headers else 0
        status = 206 if start else 200
        if start >= len(body) and start:
            return Response(416, {}, b"")
        chunk = body[start:]
        if ref in self.cut:
            return Response(status, {}, _Cut(chunk, self.cut.pop(ref)))
        return Response(status, {}, chunk)

    def blob_gets(self, digest: str) -> list[dict]:
        return [h for m, u, h in self.calls if m == "GET" and u.endswith(f"/blobs/{digest}")]


class FakeDocker:
    def __init__(self, present=(), fail=(), running=(), hotpatched=()) -> None:
        self.present = set(present)
        self.fail = set(fail)
        self.pulled: list[str] = []
        self.running = list(running)
        self.hotpatched = list(hotpatched)
        self.image_ids = {}
        self.running_ids = []

    def image_present(self, ref):
        return ref in self.present

    def image_id(self, ref):
        return self.image_ids.get(ref) if ref in self.present else None

    def running_image_ids(self):
        return sorted(self.running_ids)

    def pull(self, ref):
        if ref in self.fail:
            raise StageError("pull_failed", f"{ref}: фейковая ошибка")
        self.pulled.append(ref)
        self.present.add(ref)

    def running_images(self):
        return self.running

    def hotpatched_containers(self):
        return self.hotpatched


class FakeZenoh:
    def __init__(self, stored=(), stream=()) -> None:
        self.puts: list[tuple[str, dict]] = []
        self.stored = list(stored)
        self.stream = list(stream)

    def put(self, key, value):
        self.puts.append((key, json.loads(json.dumps(value))))

    def get(self, selector):
        return list(self.stored)

    def subscribe(self, selector, timeout_s):
        yield from self.stream


def make_bundle(files: dict[str, bytes]) -> bytes:
    buf = io.BytesIO()
    with gzip.GzipFile(fileobj=buf, mode="wb", mtime=0) as gz:
        with tarfile.open(fileobj=gz, mode="w", format=tarfile.PAX_FORMAT) as tf:
            for name, data in sorted(files.items()):
                info = tarfile.TarInfo(name)
                info.size = len(data)
                info.mode = 0o755 if name.endswith(".sh") else 0o644
                tf.addfile(info, io.BytesIO(data))
    return buf.getvalue()
