#!/usr/bin/env python3
"""Фиксированная нагрузка на scsynth для замера block size (issue #3114).

Запускается ВНУТРИ контейнера ``supercollider`` (там есть python3, но нет
Renardo), обычно так::

    docker exec -i supercollider python3 - --duration 60 < scsynth_bench_stimulus.py

Только stdlib. Что делает:

1. ``/d_load`` N SynthDef'ов из общего каталога synthdefs (scsynth запущен с
   ``-D 0`` и сам их не грузит; палитру обычно заливает sclang из
   voice-assistant, который на время бенча остановлен).
2. Заводит свою группу (``BENCH_GROUP``) и N «плееров» по схеме Renardo:
   на ноту ``/c_set <bus> <freq>`` (Renardo-синты читают частоту через
   ``In.kr(bus, 1)``), затем ``/s_new <synth> -1 addToTail BENCH_GROUP``
   с ``bus``/``sus``/``amp``. Аудио пишется в ПРИВАТНУЮ шину, на выход
   не идёт — бенч молчит, CPU при этом тратится тот же на синтез.
3. Раз в секунду шлёт ``/status`` и копит avgCPU/peakCPU/numSynths.
4. В конце ``/n_free BENCH_GROUP`` (вместе со всеми нодами) и печатает
   ОДНУ строку ``BENCH_RESULT {json}`` — её разбирает scsynth_block_bench.sh.

Ограничение (честно): это не полный путь Renardo — нет эффект-синтов
(lpf/room/…) и ``makeSound``. Нагрузка одинакова для всех block size, поэтому
для СРАВНЕНИЯ размеров годится; абсолютное число CPU на реальном треке
будет выше.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import struct
import sys
import time

BENCH_GROUP = 31140  # 3114 * 10 — не пересекается с группами Renardo (клиент 0 → 1000+)
DEFAULT_SYNTHS = "tb303,organ,fuzz,pluck,bass,warmpad"
DEFAULT_SYNTHDEF_DIR = "/root/.local/share/SuperCollider/synthdefs"
# Длительности нот плееров в долях такта (beats) — фиксированная смесь
# быстрых и длинных нот, как в типичном 6-плеерном треке.
PLAYER_DURS = (0.25, 0.5, 0.25, 1.0, 0.5, 2.0)
SCALE_HZ = (110.0, 130.81, 146.83, 164.81, 196.0, 220.0, 261.63, 293.66)
FIRST_BUS = 200  # приватные шины: 200, 204, … (< -a 1024, мимо выходов 0..7)


def osc_string(s: str) -> bytes:
    b = s.encode() + b"\x00"
    while len(b) % 4:
        b += b"\x00"
    return b


def osc_message(address: str, *args: object) -> bytes:
    tags = ","
    payload = b""
    for a in args:
        if isinstance(a, bool):
            raise TypeError("bool is not an OSC arg here")
        if isinstance(a, int):
            tags += "i"
            payload += struct.pack(">i", a)
        elif isinstance(a, float):
            tags += "f"
            payload += struct.pack(">f", a)
        elif isinstance(a, str):
            tags += "s"
            payload += osc_string(a)
        else:
            raise TypeError(f"unsupported OSC arg {a!r}")
    return osc_string(address) + osc_string(tags) + payload


def _read_osc_string(data: bytes, offset: int) -> tuple[str, int]:
    end = data.index(b"\x00", offset)
    s = data[offset:end].decode(errors="replace")
    offset = end + 1
    while offset % 4:
        offset += 1
    return s, offset


def parse_osc(data: bytes) -> tuple[str, list]:
    address, off = _read_osc_string(data, 0)
    if off >= len(data):
        return address, []
    tags, off = _read_osc_string(data, off)
    values: list = []
    for t in tags[1:]:
        if t == "i":
            values.append(struct.unpack(">i", data[off:off + 4])[0])
            off += 4
        elif t == "f":
            values.append(struct.unpack(">f", data[off:off + 4])[0])
            off += 4
        elif t == "d":
            values.append(struct.unpack(">d", data[off:off + 8])[0])
            off += 8
        elif t == "s":
            s, off = _read_osc_string(data, off)
            values.append(s)
        else:
            break
    return address, values


def schedule(players: list[str], bpm: float, duration: float) -> list[tuple[float, int, str, float, float]]:
    """Детерминированное расписание нот: (t, player_idx, synth, freq, sus)."""
    beat = 60.0 / bpm
    events = []
    for i, synth in enumerate(players):
        dur = PLAYER_DURS[i % len(PLAYER_DURS)] * beat
        t, n = 0.0, 0
        while t < duration:
            freq = SCALE_HZ[(n * (i + 3)) % len(SCALE_HZ)] * (1 + (i % 3))
            events.append((round(t, 6), i, synth, freq, dur))
            t += dur
            n += 1
    events.sort(key=lambda e: (e[0], e[1]))
    return events


class Bench:
    def __init__(self, host: str, port: int) -> None:
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.setblocking(False)
        self.status: list[list] = []
        self.failures: list[str] = []
        self.done: list[str] = []

    def send(self, msg: bytes) -> None:
        self.sock.sendto(msg, self.addr)

    def drain(self) -> None:
        while True:
            try:
                data, _ = self.sock.recvfrom(65536)
            except BlockingIOError:
                return
            try:
                address, values = parse_osc(data)
            except (ValueError, struct.error):
                continue
            if address == "/status.reply":
                self.status.append(values)
            elif address == "/fail":
                self.failures.append(" ".join(str(v) for v in values))
            elif address == "/done":
                self.done.append(" ".join(str(v) for v in values))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--bpm", type=float, default=120.0)
    ap.add_argument("--synths", default=DEFAULT_SYNTHS)
    ap.add_argument("--synthdef-dir", default=DEFAULT_SYNTHDEF_DIR)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=57110)
    ap.add_argument("--amp", type=float, default=0.3)
    ap.add_argument("--dry-run", action="store_true", help="только напечатать расписание")
    args = ap.parse_args(argv)

    wanted = [s.strip() for s in args.synths.split(",") if s.strip()]
    if args.dry_run:
        ev = schedule(wanted, args.bpm, args.duration)
        print(json.dumps({"players": wanted, "events": len(ev)}))
        return 0

    loaded, missing = [], []
    for name in wanted:
        if os.path.exists(os.path.join(args.synthdef_dir, name + ".scsyndef")):
            loaded.append(name)
        else:
            missing.append(name)

    b = Bench(args.host, args.port)
    for name in loaded:
        b.send(osc_message("/d_load", os.path.join(args.synthdef_dir, name + ".scsyndef")))
    time.sleep(1.0)
    b.drain()
    b.send(osc_message("/g_new", BENCH_GROUP, 1, 0))  # addToTail of RootNode

    events = schedule(loaded, args.bpm, args.duration)
    t0 = time.monotonic()
    next_status = t0
    notes = 0
    for t, idx, synth, freq, sus in events:
        while True:
            now = time.monotonic()
            if now >= next_status:
                b.send(osc_message("/status"))
                next_status += 1.0
            b.drain()
            wait = t0 + t - now
            if wait <= 0:
                break
            time.sleep(min(wait, 0.005))
        bus = FIRST_BUS + idx * 4
        b.send(osc_message("/c_set", bus, float(freq)))
        b.send(osc_message(
            "/s_new", synth, -1, 1, BENCH_GROUP,
            "bus", bus, "freq", float(freq), "sus", float(sus),
            "amp", float(args.amp), "blur", 1.0,
        ))
        notes += 1
    # дожидаемся хвостов и последнего /status
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        b.drain()
        time.sleep(0.05)
    b.send(osc_message("/n_free", BENCH_GROUP))
    time.sleep(0.3)
    b.drain()

    # /status.reply: [1, ugens, synths, groups, synthdefs, avgCPU, peakCPU, nomSR, actSR]
    st = [v for v in b.status if len(v) >= 7]
    result = {
        "players": loaded,
        "missing": missing,
        "notes_sent": notes,
        "status_samples": len(st),
        "avg_cpu_mean": round(sum(v[5] for v in st) / len(st), 2) if st else None,
        "peak_cpu_max": round(max(v[6] for v in st), 2) if st else None,
        "max_synths": max(v[2] for v in st) if st else None,
        "synthdefs_loaded": max(v[4] for v in st) if st else None,
        "fail_msgs": b.failures[:5],
        "fail_count": len(b.failures),
    }
    print("BENCH_RESULT " + json.dumps(result, ensure_ascii=False))
    return 0 if loaded and st else 2


if __name__ == "__main__":
    sys.exit(main())
