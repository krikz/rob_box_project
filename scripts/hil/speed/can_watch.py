#!/usr/bin/env python3
"""Счёт кадров CAN по типу команды VESC — доказательство, что в HIL на моторы ничего не ушло.

На Main Pi, только чтение шины (``candump`` слушает, ничего не шлёт)::

    nohup sh -c 'timeout 14400 candump -t a -L can0 | python3 /tmp/hil/can_watch.py /tmp/hil/can_speed.json' &

Расширенный id VESC = (команда << 8) | id контроллера. Команды 0..8 (duty, ток, тормоз, rpm, позиция …) —
управление, 9, 14, 15, 16, 27 — статусы. Каждые 30 с пишет json ``{"frames", "by_cmd", "commands", "erpm_nonzero"}``;
строки с командами 0..8 дописывает в ``<json>.cmd.log``.
"""

import json
import os
import sys
import time

STATUS_1 = 9  # CAN_PACKET_STATUS: erpm (4 байта), ток, duty


def main(out: str) -> int:
    counts: dict = {}
    frames = erpm_nonzero = 0
    commands = 0
    last = time.monotonic()
    with open(out + ".cmd.log", "a") as cmd_log:
        for line in sys.stdin:
            parts = line.split()
            if len(parts) < 3 or "#" not in parts[2]:
                continue
            ident, _, data = parts[2].partition("#")
            try:
                cmd = (int(ident, 16) >> 8) & 0xFF
            except ValueError:
                continue
            frames += 1
            counts[cmd] = counts.get(cmd, 0) + 1
            if cmd < STATUS_1:
                commands += 1
                cmd_log.write(line)
            elif cmd == STATUS_1 and len(data) >= 8 and int(data[:8], 16) != 0:
                erpm_nonzero += 1
            if time.monotonic() - last > 30:
                last = time.monotonic()
                _dump(out, frames, counts, commands, erpm_nonzero)
    _dump(out, frames, counts, commands, erpm_nonzero)
    return 0


def _dump(out: str, frames: int, counts: dict, commands: int, erpm_nonzero: int) -> None:
    with open(out + ".part", "w") as f:
        json.dump(
            {
                "t": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "frames": frames,
                "by_cmd": {str(k): v for k, v in sorted(counts.items())},
                "commands": commands,
                "erpm_nonzero": erpm_nonzero,
            },
            f,
        )
    os.replace(out + ".part", out)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1]))
