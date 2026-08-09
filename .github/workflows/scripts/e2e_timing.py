#!/usr/bin/env python3
"""E2E response timing metrics (from robot voice-assistant logs).

🎯 Телеметрия скорости ответа (юзер: «цель — как можно быстрее ответить»):
  T_accept: Получена фраза → ПРИНЯТО   (STT latency)
  T_llm:    LLM INPUT → первый «Синтез через» (LLM вызов)
  T_tts:    Синтез через → TTS finished success (синтез)
  T_total:  ПРИНЯТО → первый TTS finished (акцепт → ответ)

Usage: python3 e2e_timing.py <voice_e2e_<run_id>.log>
"""
import re
import sys


def ts_of_line(line: str):
    m = re.search(r"\[(\d+\.\d+)\]", line)
    return float(m.group(1)) if m else None


def first_ts(log: str, pattern: str, after_ts: float = None):
    """Timestamp of the FIRST line matching pattern (optionally after after_ts)."""
    for line in log.splitlines():
        if re.search(pattern, line):
            t = ts_of_line(line)
            if t is not None and (after_ts is None or t > after_ts):
                return t
    return None


def last_ts(log: str, pattern: str):
    """Timestamp of the LAST line matching pattern."""
    t = None
    for line in log.splitlines():
        if re.search(pattern, line):
            c = ts_of_line(line)
            if c is not None:
                t = c
    return t


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/voice_e2e.log"
    try:
        log = open(path, encoding="utf-8", errors="replace").read()
    except OSError as exc:
        print(f"e2e_timing: cannot read {path}: {exc}")
        return 0

    # Последний диалог (текущая команда), не приветствие
    accept_t = last_ts(log, r"✅ ПРИНЯТО")
    phrase_t = last_ts(log, r"Получена фраза")
    llm_in_t = first_ts(log, r"📥 LLM INPUT", after_ts=accept_t)
    synth_t = first_ts(log, r"🔊 Синтез через", after_ts=llm_in_t)
    tts_ok_t = first_ts(log, r"TTS finished.*success=True", after_ts=synth_t)

    print("--- e2e response timing (from robot logs) ---")
    if phrase_t and accept_t and accept_t > phrase_t:
        print(f"T_accept: {accept_t - phrase_t:.1f}s  (STT: фраза → ПРИНЯТО)")
    if llm_in_t and synth_t and synth_t > llm_in_t:
        print(f"T_llm:    {synth_t - llm_in_t:.1f}s  (LLM INPUT → Синтез)")
    if synth_t and tts_ok_t and tts_ok_t > synth_t:
        print(f"T_tts:    {tts_ok_t - synth_t:.1f}s  (Синтез → TTS finished)")
    if accept_t and tts_ok_t and tts_ok_t > accept_t:
        total = tts_ok_t - accept_t
        print(f"T_total:  {total:.1f}s  (акцепт → ответ робота)")
        print(f"SUMMARY_MARKER T_total={total:.1f}s")
    print("--- end timing ---")
    return 0


if __name__ == "__main__":
    sys.exit(main())
