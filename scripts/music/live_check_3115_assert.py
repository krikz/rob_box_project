#!/usr/bin/env python3
"""Механические проверки для live_check_3115.sh (#3115). Только stdlib.

Каждая подкоманда печатает одну строку ``PASS: ...`` / ``FAIL: ...`` и
выходит с 0 / 1. На слух здесь не проверяется НИЧЕГО: «не тишина» в wav —
это только признак, что звук вообще был; вердикт «ок / не ок» — у Шифу.

Подкоманды::

    code   <mcp_result.json> <out.foxdot>  — вынуть result.data.code
    drums  <code.foxdot>                   — у d1 и d2 dur=0.25 (или 1/4)
    melody <code.foxdot>                   — у p-плеера amplify=[...] списком
    wav    <file.wav> [min_peak_dbfs]      — длительность/пик/RMS, не тишина
    logerr <log> [<log> ...]               — нет «not found» / «FAILURE IN SERVER»
    grep   <pattern> <log>                 — фиксированная строка есть в логе
    absent <pattern> <log>                 — фиксированной строки в логе НЕТ
"""

from __future__ import annotations

import array
import json
import math
import re
import sys
import wave
from pathlib import Path
from typing import List, Tuple

#: Ошибки scsynth/sclang, из-за которых трек «играет» в тишину (#3115 п.4).
LOG_ERROR_PATTERNS: Tuple[str, ...] = (
    "not found",
    "FAILURE IN SERVER",
    "ERROR: syntax error",
    "Parse error",
)

_PLAYER_LINE = re.compile(r"^\s*(?P<slot>[dp]\d)\s*>>\s*(?P<body>.*)$")


def _player_lines(code: str) -> dict:
    """slot -> текст объявления плеера (с продолжениями до следующего плеера)."""
    out: dict = {}
    current = None
    for line in code.splitlines():
        m = _PLAYER_LINE.match(line)
        if m:
            current = m.group("slot")
            out[current] = m.group("body")
        elif current is not None and line.startswith((" ", "\t")):
            out[current] += " " + line.strip()
        else:
            current = None
    return out


def cmd_code(result_path: str, out_path: str) -> Tuple[bool, str]:
    try:
        payload = json.loads(Path(result_path).read_text(encoding="utf-8").strip().splitlines()[-1])
    except (OSError, ValueError, IndexError) as exc:
        return False, f"не прочитан ответ MCP {result_path}: {exc}"
    result = payload.get("result") or {}
    code = (result.get("data") or {}).get("code")
    if not result.get("success"):
        return False, f"тул вернул ошибку: {result.get('error') or payload.get('error')}"
    if not code:
        return False, "в result.data нет code"
    Path(out_path).write_text(code, encoding="utf-8")
    return True, f"код {len(code)} байт -> {out_path}"


def cmd_drums(code_path: str) -> Tuple[bool, str]:
    players = _player_lines(Path(code_path).read_text(encoding="utf-8"))
    bad: List[str] = []
    for slot in ("d1", "d2"):
        body = players.get(slot)
        if body is None:
            bad.append(f"{slot}: нет плеера")
        elif not re.search(r"\bdur\s*=\s*(0\.25|1\s*/\s*4)\b", body):
            bad.append(f"{slot}: нет dur=0.25")
    if bad:
        return False, "; ".join(bad)
    return True, "d1 и d2 играют с dur=0.25"


def cmd_melody(code_path: str) -> Tuple[bool, str]:
    players = _player_lines(Path(code_path).read_text(encoding="utf-8"))
    hits = sorted(s for s, b in players.items() if s.startswith("p") and re.search(r"\bamplify\s*=\s*\[", b))
    if not hits:
        return False, "ни у одного p-плеера нет amplify=[...] списком"
    return True, "amplify=[...] списком у " + ", ".join(hits)


def cmd_wav(wav_path: str, min_peak_dbfs: float = -50.0) -> Tuple[bool, str]:
    try:
        with wave.open(wav_path, "rb") as w:
            rate, ch, width, n = w.getframerate(), w.getnchannels(), w.getsampwidth(), w.getnframes()
            raw = w.readframes(n)
    except (OSError, wave.Error, EOFError) as exc:
        return False, f"wav не читается: {exc}"
    if width != 2:
        # jack_rec пишет 16 бит по умолчанию (-b 16 передаётся явно).
        return False, f"неожиданная разрядность {width * 8} бит"
    samples = array.array("h", raw)
    dur = n / float(rate) if rate else 0.0
    if not samples:
        return False, f"пустой wav ({dur:.1f}s)"
    peak = max(abs(s) for s in samples) / 32768.0
    rms = math.sqrt(sum(s * s for s in samples) / len(samples)) / 32768.0
    to_db = lambda x: 20 * math.log10(x) if x > 0 else float("-inf")  # noqa: E731
    info = f"{dur:.1f}s {rate}Hz {ch}ch peak={to_db(peak):.1f}dBFS rms={to_db(rms):.1f}dBFS"
    if to_db(peak) < min_peak_dbfs:
        return False, f"тишина ({info})"
    return True, info


def cmd_logerr(*log_paths: str) -> Tuple[bool, str]:
    found: List[str] = []
    for p in log_paths:
        try:
            text = Path(p).read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            return False, f"лог не прочитан {p}: {exc}"
        for line in text.splitlines():
            if any(pat in line for pat in LOG_ERROR_PATTERNS):
                found.append(f"{Path(p).name}: {line.strip()[:160]}")
    if found:
        return False, f"{len(found)} строк с ошибками, первая: {found[0]}"
    return True, "нет " + " / ".join(LOG_ERROR_PATTERNS)


def cmd_absent(pattern: str, log_path: str) -> Tuple[bool, str]:
    ok, msg = cmd_grep(pattern, log_path)
    if msg.startswith("лог не прочитан"):
        return False, msg
    return (not ok), (f"нет «{pattern}»" if not ok else f"найдено: {msg}")


def cmd_grep(pattern: str, log_path: str) -> Tuple[bool, str]:
    try:
        text = Path(log_path).read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        return False, f"лог не прочитан {log_path}: {exc}"
    lines = [ln.strip() for ln in text.splitlines() if pattern in ln]
    if not lines:
        return False, f"нет «{pattern}» в {Path(log_path).name}"
    return True, f"{len(lines)}× «{pattern}», последняя: {lines[-1][:200]}"


COMMANDS = {
    "code": cmd_code,
    "drums": cmd_drums,
    "melody": cmd_melody,
    "wav": lambda path, thr="-50": cmd_wav(path, float(thr)),
    "logerr": cmd_logerr,
    "grep": cmd_grep,
    "absent": cmd_absent,
}


def main(argv: List[str]) -> int:
    if len(argv) < 2 or argv[1] not in COMMANDS:
        print(f"usage: {Path(argv[0]).name} {{{'|'.join(COMMANDS)}}} ...", file=sys.stderr)
        return 64
    try:
        ok, msg = COMMANDS[argv[1]](*argv[2:])
    except TypeError as exc:
        print(f"FAIL: неверные аргументы: {exc}")
        return 1
    print(("PASS: " if ok else "FAIL: ") + msg)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
