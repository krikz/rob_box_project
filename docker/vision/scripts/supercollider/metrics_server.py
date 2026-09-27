#!/usr/bin/env python3
"""Metrics endpoint для supercollider-контейнера (issue #1160, порт 9102).

Контейнер содержит только scsynth (SuperCollider server, без Python-пакетов),
поэтому не тащим prometheus_client — отдаём Prometheus text format напрямую
через stdlib ``http.server``. Prometheus scrape target:
``10.1.1.11:9102/metrics`` (см. docker/monitoring/config/prometheus.yml).

Метрики:
* ``supercollider_up`` — 1 если scsynth жив (pgrep), иначе 0
* ``supercollider_uptime_seconds`` — время работы контейнера с момента старта
* ``supercollider_synthdefs_count`` — число SynthDef, реально загруженных в
  scsynth прямо сейчас (issue #1809, было issue 986)

🔴 FIX (issue #1809): раньше synthdefs_count считал .scsyndef-файлы в общей
volume. Эта volume почти всегда пуста или содержит единицы файлов — sclang
шлёт определения в scsynth через ``/d_recv`` по OSC (память), на диск он их
не пишет. В реальности scsynth держит в памяти ~300 SynthDef (стартовый
прелоад foxdot_init.sc + всё, что Python renardo досылает через /foxdot по
ходу работы), а метрика честно рапортовала ~5 — по числу файлов, случайно
оказавшихся на диске. Единственный источник правды — сам scsynth: запрос
``/status`` по OSC на 127.0.0.1:57110 возвращает ``/status.reply`` с полем
numSynthDefs (5-й int из ``iiiii``: unused, numUGens, numSynths, numGroups,
numSynthDefs — см. Server Command Reference). Спрашиваем его напрямую.
"""

import http.server
import os
import socket
import struct
import subprocess
import time

PORT = int(os.environ.get("SUPERCOLLIDER_METRICS_PORT", "9102"))
SCSYNTH_HOST = "127.0.0.1"
SCSYNTH_PORT = 57110
SCSYNTH_STATUS_TIMEOUT_S = 1.5


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


def osc_string(s: str) -> bytes:
    """Кодирует строку в OSC-формат: null-терминатор + паддинг до кратности 4.

    ВАЖНО: строка обязана иметь МИНИМУМ ОДИН нулевой терминатор и суммарную
    длину, кратную 4. Наивное ``(4 - len(b) % 4) % 4`` даёт НОЛЬ добивки для
    строк длиной 4, 8, 12 и т.д. — терминатор в этом случае не пишется,
    сообщение уходит без него, и разбор на другом конце съезжает со сдвигом
    (уже ловили на этом баг в group-1 фиксе, commit 8c4079a7). Поэтому
    терминатор добавляется БЕЗУСЛОВНО, а паддинг — циклом.
    """
    b = s.encode() + b"\x00"
    while len(b) % 4:
        b += b"\x00"
    return b


def _read_osc_string(data: bytes, offset: int) -> tuple:
    """Читает одну OSC-строку начиная с ``offset``, возвращает (строка, следующий_offset)."""
    end = data.index(b"\x00", offset)
    value = data[offset:end].decode()
    next_offset = end + 1
    while next_offset % 4:
        next_offset += 1
    return value, next_offset


def _parse_status_reply(data: bytes) -> list:
    """Разбирает ``/status.reply``: адрес, типтэги ``,iiiiiffdd``, значения."""
    address, offset = _read_osc_string(data, 0)
    if address != "/status.reply":
        raise ValueError(f"unexpected OSC reply address: {address!r}")
    typetags, offset = _read_osc_string(data, offset)
    values = []
    for tag in typetags[1:]:  # [0] всегда ','
        if tag == "i":
            values.append(struct.unpack_from(">i", data, offset)[0])
            offset += 4
        elif tag == "f":
            values.append(struct.unpack_from(">f", data, offset)[0])
            offset += 4
        elif tag == "d":
            values.append(struct.unpack_from(">d", data, offset)[0])
            offset += 8
        elif tag == "s":
            value, offset = _read_osc_string(data, offset)
            values.append(value)
        else:
            raise ValueError(f"unsupported OSC type tag: {tag!r}")
    return values


def _synthdefs_count() -> int:
    """Спрашивает scsynth напрямую через ``/status`` (issue #1809).

    ``/status`` не принимает аргументов, но type tag string всё равно
    обязателен по спецификации OSC — шлём ``","``. Любая ошибка (scsynth не
    ответил, порт закрыт, ответ не распарсился) трактуется как «сейчас
    определений 0» — честнее, чем отдавать протухшее число из файловой
    системы.
    """
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(SCSYNTH_STATUS_TIMEOUT_S)
        request = osc_string("/status") + osc_string(",")
        sock.sendto(request, (SCSYNTH_HOST, SCSYNTH_PORT))
        data, _ = sock.recvfrom(1024)
        # iiiii: unused, numUGens, numSynths, numGroups, numSynthDefs
        return int(_parse_status_reply(data)[4])
    except Exception:
        return 0
    finally:
        if sock is not None:
            sock.close()


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
                "# HELP supercollider_synthdefs_count SynthDefs currently loaded in scsynth (via OSC /status, issue #1809).",
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
