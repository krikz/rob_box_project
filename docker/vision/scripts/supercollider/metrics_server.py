#!/usr/bin/env python3
"""Metrics endpoint для supercollider-контейнера (issue #1160, порт 9102).

Контейнер содержит только scsynth (SuperCollider server, без Python-пакетов),
поэтому не тащим prometheus_client — отдаём Prometheus text format напрямую
через stdlib ``http.server``. Prometheus scrape target:
``10.1.1.11:9102/metrics`` (см. docker/monitoring/config/prometheus.yml).

Метрики:
* ``supercollider_up`` — 1 если scsynth жив (pgrep), иначе 0
* ``supercollider_uptime_seconds`` — время работы контейнера с момента старта
* ``supercollider_synthdefs_count`` — число .scsyndef файлов в общей volume
  (сколько SynthDef'ов скомпилировал Renardo/FoxDot — issue 986)
"""

import http.server
import os
import subprocess
import time

PORT = int(os.environ.get("SUPERCOLLIDER_METRICS_PORT", "9102"))
SYNTHDEFS_DIR = "/root/.local/share/SuperCollider/synthdefs"


def _scsynth_alive() -> int:
    try:
        result = subprocess.run(
            ["pgrep", "-x", "scsynth"],
            capture_output=True,
            timeout=2,
        )
        return 1 if result.returncode == 0 else 0
    except Exception:
        return 0


def _synthdefs_count() -> int:
    try:
        if not os.path.isdir(SYNTHDEFS_DIR):
            return 0
        return sum(
            1 for f in os.listdir(SYNTHDEFS_DIR) if f.endswith(".scsyndef")
        )
    except Exception:
        return 0


class MetricsHandler(http.server.BaseHTTPRequestHandler):
    _start = time.monotonic()

    def do_GET(self) -> None:  # noqa: N802 — http.server API
        if self.path not in ("/metrics", "/"):
            self.send_response(404)
            self.end_headers()
            return
        uptime_s = time.monotonic() - self._start
        body = "\n".join(
            [
                "# HELP supercollider_up 1 if scsynth process is alive.",
                "# TYPE supercollider_up gauge",
                f"supercollider_up {_scsynth_alive()}",
                "# HELP supercollider_uptime_seconds Container uptime.",
                "# TYPE supercollider_uptime_seconds gauge",
                f"supercollider_uptime_seconds {uptime_s:.0f}",
                "# HELP supercollider_synthdefs_count Compiled SynthDef files.",
                "# TYPE supercollider_synthdefs_count gauge",
                f"supercollider_synthdefs_count {_synthdefs_count()}",
                "",
            ]
        ).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/plain; version=0.0.4; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args: object) -> None:  # noqa: A002
        # Тихий access log — метрики скрейпятся каждые 15s.
        pass


if __name__ == "__main__":
    server = http.server.ThreadingHTTPServer(("0.0.0.0", PORT), MetricsHandler)
    print(f"[supercollider-metrics] listening on :{PORT}/metrics")
    server.serve_forever()
