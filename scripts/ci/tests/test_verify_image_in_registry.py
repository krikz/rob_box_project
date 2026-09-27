"""Тесты scripts/ci/verify-image-in-registry.sh.

Скрипт вынесен из ТРЁХ копий функции `verify_in_registry`
(L-Build Vision Pi Services.yml, L-Build Main Pi Services.yml,
L-Build Single Service.yml) по docs/plans/2026-09-15-image-versions-seam.md §7.4.
Копии уже разъехались на одну строку сообщения — значит их синхронность
держалась ни на чём.

Registry поднимается настоящим http.server на 127.0.0.1: скрипт ходит
curl'ом, и подменять curl шимом значило бы тестировать шим, а не скрипт.
"""

from __future__ import annotations

import shutil
import socket
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "scripts" / "ci" / "verify-image-in-registry.sh"
BASH = shutil.which("bash") or "/bin/bash"

MANIFEST_PATH = "/v2/krikz/rob_box/manifests/"
PRESENT_TAG = "led-matrix-humble-dev-a1b2c3d"


class _RegistryHandler(BaseHTTPRequestHandler):
    """Отвечает 200 только на один известный тег, на остальные — 404."""

    def do_GET(self) -> None:  # noqa: N802 (имя задано BaseHTTPRequestHandler)
        if self.path == MANIFEST_PATH + PRESENT_TAG:
            body = b'{"schemaVersion":2}'
            self.send_response(200)
            self.send_header("Content-Type", "application/vnd.docker.distribution.manifest.v2+json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404, "manifest unknown")

    def log_message(self, *_args) -> None:  # тишина в выводе pytest
        return


@pytest.fixture(scope="module")
def registry():
    server = HTTPServer(("127.0.0.1", 0), _RegistryHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"127.0.0.1:{server.server_port}"
    finally:
        server.shutdown()
        server.server_close()


def _free_port() -> int:
    """Порт, который заведомо никто не слушает — для проверки недоступности."""
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def run_script(*args: str):
    return subprocess.run(
        [BASH, str(SCRIPT), *args],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=60,
        cwd=str(REPO_ROOT),
    )


def test_script_is_syntactically_valid() -> None:
    assert SCRIPT.exists(), SCRIPT
    result = subprocess.run([BASH, "-n", str(SCRIPT)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_present_tag_returns_zero(registry: str) -> None:
    result = run_script(registry, PRESENT_TAG)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Verify OK" in result.stdout, result.stdout


def test_missing_tag_fails_with_http_code(registry: str) -> None:
    """404 обязан валить job: иначе .image-versions уедет на несуществующий тег.

    Ровно это и было issue #1482 — deploy тянул stale-тег и робот молча
    оставался на старом образе.
    """
    result = run_script(registry, "нет-такого-тега")
    assert result.returncode == 1, result.stdout + result.stderr
    assert "Verify FAIL" in result.stderr, result.stderr
    assert "404" in result.stderr, result.stderr


def test_unreachable_registry_reports_000_not_crash() -> None:
    """Лежащий реестр — понятное сообщение с кодом 000, а не голый выход curl."""
    result = run_script(f"127.0.0.1:{_free_port()}", PRESENT_TAG)
    assert result.returncode == 1, result.stdout + result.stderr
    assert "000" in result.stderr, result.stderr


def test_usage_error_on_wrong_argument_count() -> None:
    assert run_script("127.0.0.1:5000").returncode == 1
    assert run_script().returncode == 1


def test_custom_repo_is_honoured(registry: str) -> None:
    """Третий аргумент меняет путь репозитория, а не игнорируется молча."""
    result = run_script(registry, PRESENT_TAG, "krikz/rob_box_base")
    assert result.returncode == 1, result.stdout + result.stderr
    assert "404" in result.stderr, result.stderr


def test_workflows_do_not_reimplement_the_check() -> None:
    """Guard: ни один workflow не должен заново писать тело проверки.

    Именно так и появились три копии, разъехавшиеся на строку сообщения.
    """
    workflows = [
        REPO_ROOT / ".github" / "workflows" / "L-Build Vision Pi Services.yml",
        REPO_ROOT / ".github" / "workflows" / "L-Build Main Pi Services.yml",
        REPO_ROOT / ".github" / "workflows" / "L-Build Single Service.yml",
    ]
    for path in workflows:
        text = path.read_text(encoding="utf-8")
        assert "verify-image-in-registry.sh" in text, f"{path.name} не зовёт общий скрипт"
        assert "/v2/" not in text, (
            f"{path.name} снова содержит собственный запрос к Registry API — "
            f"тело проверки живёт в scripts/ci/verify-image-in-registry.sh"
        )
