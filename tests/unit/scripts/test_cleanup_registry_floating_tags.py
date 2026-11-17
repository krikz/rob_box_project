"""#3540: cleanup_registry.sh --keep не должен удалять манифест, на который
указывает плавающий тег (<svc>-humble-dev, *-local, score-library-*, ...).

Фейковый registry v2 (http.server в потоке) хранит теги -> digest и честно
исполняет DELETE по digest: снимает ВСЕ теги с этим digest, как настоящий.
"""

from __future__ import annotations

import json
import os
import shutil
import stat
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[3] / "docker/build/scripts/cleanup_registry.sh"
BASH = shutil.which("bash")

pytestmark = pytest.mark.skipif(
    BASH is None or shutil.which("curl") is None,
    reason="bash/curl not available",
)

JQ_SHIM = """import json, sys
# минимальный jq: -r '.repositories[]' / '.tags[]'
expr = sys.argv[-1]
data = json.load(sys.stdin)
for v in data[expr.strip(".[]")]:
    print(v)
"""


class FakeRegistry:
    def __init__(self, repos):
        # repos: {repo: {tag: (digest, created)}}
        self.repos = repos
        self.deleted = []
        outer = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, body=b"", headers=None):
                self.send_response(code)
                for k, v in (headers or {}).items():
                    self.send_header(k, v)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                if self.command != "HEAD":
                    self.wfile.write(body)

            def _route(self):
                parts = self.path.split("/")  # '', v2, ...
                if self.path == "/v2/":
                    return self._send(200, b"{}")
                if self.path == "/v2/_catalog":
                    return self._send(200, json.dumps({"repositories": list(outer.repos)}).encode())
                repo = "/".join(parts[2:-2])
                kind, ref = parts[-2], parts[-1]
                tags = outer.repos.get(repo, {})
                if kind == "tags":
                    return self._send(200, json.dumps({"tags": list(tags)}).encode())
                if kind == "manifests":
                    if self.command == "DELETE":
                        victims = [t for t, (d, _) in tags.items() if d == ref]
                        for t in victims:
                            del tags[t]
                        outer.deleted.append((repo, ref, victims))
                        return self._send(202)
                    if ref not in tags:
                        return self._send(404)
                    digest, _ = tags[ref]
                    body = json.dumps({"config": {"digest": "cfg-" + digest}}).encode()
                    return self._send(200, body, {"Docker-Content-Digest": digest})
                if kind == "blobs":
                    created = next((c for d, c in tags.values() if "cfg-" + d == ref), "")
                    return self._send(200, json.dumps({"created": created}).encode())
                return self._send(404)

            do_GET = do_HEAD = do_DELETE = _route

        self.srv = HTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.srv.server_port}"
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    def close(self):
        self.srv.shutdown()


def _run(reg, tmp_path, *args):
    shimdir = tmp_path / "bin"
    shimdir.mkdir()
    py = Path(sys.executable).as_posix()
    if shutil.which("jq") is None:
        jq = shimdir / "jq"
        jq_py = shimdir / "jq.py"
        jq_py.write_text(JQ_SHIM, encoding="utf-8", newline="\n")
        jq.write_text(
            f"#!/bin/sh\nexec '{py}' '{jq_py.as_posix()}' \"$@\"\n",
            encoding="utf-8", newline="\n",
        )
        jq.chmod(jq.stat().st_mode | stat.S_IEXEC)
    if os.name == "nt" or shutil.which("python3") is None:  # на Windows python3 — заглушка Store
        py3 = shimdir / "python3"
        py3.write_text(f"#!/bin/sh\nexec '{py}' \"$@\"\n", encoding="utf-8", newline="\n")
        py3.chmod(py3.stat().st_mode | stat.S_IEXEC)
    env = dict(os.environ)
    env["PATH"] = shimdir.as_posix() + os.pathsep + env["PATH"]
    env["REGISTRY_URL"] = reg.url
    env["REGISTRY_CONTAINER"] = "no-such-container"
    env["PYTHONIOENCODING"] = "utf-8"
    r = subprocess.run(
        [BASH, SCRIPT.as_posix(), *args],
        env=env, capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=120,
    )
    # защита от вакуумного PASS: скрипт обязан увидеть репозитории
    assert "No repositories found" not in r.stdout, r.stdout + r.stderr
    return r


def test_sha_tag_sharing_digest_with_floating_tag_is_skipped(tmp_path):
    # keep=2: aaaaaaa старый и делит digest с плавающным тегом (образ не менялся)
    # -> кандидат на удаление, но защищён; ddddddd реально старый -> удалён.
    reg = FakeRegistry(
        {
            "ceiling-camera": {
                "ceiling-camera-humble-dev": ("sha256:same", "2026-10-01T00:00:00Z"),
                "ceiling-camera-humble-dev-aaaaaaa": ("sha256:same", "2026-10-01T00:00:00Z"),
                "ceiling-camera-humble-dev-bbbbbbb": ("sha256:new", "2026-10-05T00:00:00Z"),
                "ceiling-camera-humble-dev-ccccccc": ("sha256:newer", "2026-10-06T00:00:00Z"),
                "ceiling-camera-humble-dev-ddddddd": ("sha256:old", "2026-09-01T00:00:00Z"),
            }
        }
    )
    try:
        r = _run(reg, tmp_path, "--keep", "2")
        out = r.stdout + r.stderr
        assert r.returncode == 0, out
        tags = reg.repos["ceiling-camera"]
        assert "ceiling-camera-humble-dev" in tags, out  # плавающий цел
        assert "ceiling-camera-humble-dev-aaaaaaa" in tags, out
        assert "ceiling-camera-humble-dev-ddddddd" not in tags, out
        assert "держит тег ceiling-camera-humble-dev" in out, out
        assert all(d != "sha256:same" for _, d, _ in reg.deleted)
    finally:
        reg.close()


def test_candidate_sharing_digest_with_kept_sha_tag_is_skipped(tmp_path):
    reg = FakeRegistry(
        {
            "svc": {
                "svc-humble-dev-1111111": ("sha256:x", "2026-10-01T00:00:00Z"),
                "svc-humble-dev-2222222": ("sha256:x", "2026-10-01T00:00:00Z"),
                "svc-humble-dev-3333333": ("sha256:y", "2026-10-03T00:00:00Z"),
            }
        }
    )
    try:
        r = _run(reg, tmp_path, "--keep", "2")
        assert r.returncode == 0, r.stdout + r.stderr
        assert len(reg.repos["svc"]) == 3, r.stdout + r.stderr
    finally:
        reg.close()


def test_dry_run_deletes_nothing(tmp_path):
    reg = FakeRegistry(
        {
            "s": {
                "s-dev-aaaaaaa": ("sha256:1", "2026-01-01T00:00:00Z"),
                "s-dev-bbbbbbb": ("sha256:2", "2026-02-01T00:00:00Z"),
            }
        }
    )
    try:
        r = _run(reg, tmp_path, "--keep", "1", "--dry-run")
        assert r.returncode == 0, r.stdout + r.stderr
        assert reg.deleted == []
    finally:
        reg.close()
