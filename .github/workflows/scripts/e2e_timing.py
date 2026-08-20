#!/usr/bin/env python3
"""e2e_timing.py — телеметрия скорости ответа (text + JSON).

🎯 Главная метрика — «конец речи пользователя → начало ответа робота»
(см. docs/design/E2E_TESTING_DESIGN_v2.md §B.2).

Режимы вывода:
  - default: человекочитаемый текст (для лога workflow + step summary).
  - --json: machine-readable JSON (issue #1396 артефакт
    e2e-voice-timing-<run_id>.json) с полной разбивкой:
      wake_to_stt_end_ms, stt_to_llm_ms, llm_to_tts_start_ms,
      tts_to_playback_start_ms, playback_duration_ms, total_latency_ms.

Источник данных — docker-лог voice-assistant робота, скачанный в файл.
Распарсиваются ROS-timestamp'ы вида ``[12345.678]``.

Все метрики — best-effort; если какого-то этапа нет в логе, поле будет
null и в отчёт пойдёт note ``partial_data``.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from typing import Optional


def ts_of_line(line: str) -> Optional[float]:
    m = re.search(r"\[(\d+\.\d+)\]", line)
    return float(m.group(1)) if m else None


def first_ts(log: str, pattern: str, after_ts: Optional[float] = None) -> Optional[float]:
    for line in log.splitlines():
        if re.search(pattern, line):
            t = ts_of_line(line)
            if t is not None and (after_ts is None or t > after_ts):
                return t
    return None


def last_ts(log: str, pattern: str) -> Optional[float]:
    t = None
    for line in log.splitlines():
        if re.search(pattern, line):
            c = ts_of_line(line)
            if c is not None:
                t = c
    return t


def collect_metrics(log: str) -> dict:
    """Возвращает dict с abs метриками (ROS sec timestamps)."""
    accept_t = last_ts(log, r"✅ ПРИНЯТО")
    phrase_t = last_ts(log, r"Получена фраза")
    llm_in_t = first_ts(log, r"📥 LLM INPUT", after_ts=accept_t)
    synth_t = first_ts(log, r"🔊 Синтез через", after_ts=llm_in_t)
    tts_ok_t = first_ts(log, r"TTS finished.*success=True", after_ts=synth_t)
    play_end_t = first_ts(log, r"Воспроизведение завершено", after_ts=tts_ok_t or synth_t)

    recognized = ""
    for line in log.splitlines():
        m = re.search(r"✅ ПРИНЯТО:\s*(.+)", line)
        if m:
            recognized = m.group(1).strip()

    # Длительность TTS-синтеза (для вычисления начала воспроизведения)
    tts_duration = 0.0
    if tts_ok_t is not None:
        for line in log.splitlines():
            if re.search(r"TTS finished.*success=True", line) and ts_of_line(line) == tts_ok_t:
                dm = re.search(r"duration=([\d.]+)s", line)
                if dm:
                    tts_duration = float(dm.group(1))
                break

    # Семантика (для воспроизведения голоса Шифу — issue #1396):
    #   wake_to_stt_end_ms      = юзер замолчал (Получена фраза) → STT ПРИНЯТО
    #   stt_to_llm_ms           = STT ПРИНЯТО → LLM запрос ушёл (LLM INPUT)
    #   llm_to_tts_start_ms     = LLM INPUT → Синтез через (LLM думал)
    #   tts_to_playback_end_ms  = TTS finished → Воспроизведение завершено
    #                              (tts + play_тело в одном — пользователю
    #                              не важно где граница; важно что с момента
    #                              синтеза до конца звука сколько прошло)
    #   playback_only_ms        = Воспроизведение завершено − (TTS finished − duration)
    #                              (если хотим отделить длительность звучания)
    #   total_latency_ms        = wake → TTS finished (момент когда ответ ГОТОВ)
    def ms(a, b):
        if a is None or b is None or a < b:
            return None
        return round((a - b) * 1000.0, 1)

    playback_start = (tts_ok_t - tts_duration) if tts_ok_t is not None else None
    return {
        "recognized": recognized,
        "abs_timestamps": {
            "phrase_t": phrase_t,
            "accept_t": accept_t,
            "llm_in_t": llm_in_t,
            "synth_t": synth_t,
            "tts_ok_t": tts_ok_t,
            "playback_start_t": playback_start,
            "play_end_t": play_end_t,
            "tts_duration_s": round(tts_duration, 3),
        },
        "metrics_ms": {
            "wake_to_stt_end_ms":       ms(accept_t, phrase_t),        # юзер замолчал → STT ОК
            "stt_to_llm_ms":            ms(llm_in_t, accept_t),        # акцепт → LLM вызван
            "llm_to_tts_start_ms":      ms(synth_t, llm_in_t),         # LLM думал → синтез начат
            "tts_to_playback_end_ms":   ms(play_end_t, tts_ok_t),      # TTS ready → звук отыграл
            "playback_only_ms":         ms(play_end_t, playback_start),# длительность звука
            "total_latency_ms":         ms(tts_ok_t, phrase_t),        # wake → ответ готов
        },
    }


def render_text(d: dict, voice_text: str) -> str:
    m = d["metrics_ms"]
    notes = []
    out = []
    out.append("--- e2e response timing (from robot logs) ---")
    out.append(f"COMMAND:    {voice_text or '(не задан)'}")
    out.append(f"RECOGNIZED: {d['recognized'] or '(нет в логе)'}")
    if m["wake_to_stt_end_ms"] is not None:
        out.append(f"T_wake→stt_end: {m['wake_to_stt_end_ms']:.0f}ms  (юзер замолчал → STT ПРИНЯТО)")
    else:
        notes.append("no wake→stt_end")
    if m["stt_to_llm_ms"] is not None:
        out.append(f"T_stt→llm:      {m['stt_to_llm_ms']:.0f}ms  (ПРИНЯТО → LLM INPUT)")
    else:
        notes.append("no stt→llm")
    if m["llm_to_tts_start_ms"] is not None:
        out.append(f"T_llm→tts:        {m['llm_to_tts_start_ms']:.0f}ms  (LLM INPUT → Синтез через)")
    else:
        notes.append("no llm→tts")
    if m["tts_to_playback_end_ms"] is not None:
        out.append(f"T_tts→play_end:   {m['tts_to_playback_end_ms']:.0f}ms  (TTS finished → Воспроизведение завершено)")
    if m["playback_only_ms"] is not None:
        out.append(f"T_playback:       {m['playback_only_ms']:.0f}ms  (только звучание)")
    if m["total_latency_ms"] is not None:
        out.append(f"⭐ T_total:      {m['total_latency_ms']:.0f}ms  (wake → конец синтеза)")
        out.append(f"SUMMARY_MARKER T_total_ms={m['total_latency_ms']:.0f}")
    else:
        notes.append("no total_latency")
    if notes:
        out.append(f"NOTES: {', '.join(notes)} (partial_data)")
    out.append("--- end timing ---")
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("log_path", nargs="?",
                    default="/tmp/voice_e2e.log",
                    help="Путь к docker-логу voice-assistant (по умолчанию /tmp/voice_e2e.log)")
    ap.add_argument("voice_text", nargs="?", default="",
                    help="Текст команды (для контекста в логе)")
    ap.add_argument("--json", action="store_true",
                    help="Вывести JSON вместо текста (issue #1396 artifact)")
    args = ap.parse_args()

    try:
        log = open(args.log_path, encoding="utf-8", errors="replace").read()
    except OSError as exc:
        if args.json:
            print(json.dumps({"error": f"cannot read log: {exc}", "path": args.log_path,
                              "partial_data": True}, ensure_ascii=False))
        else:
            print(f"e2e_timing: cannot read {args.log_path}: {exc}")
        return 0

    d = collect_metrics(log)
    has_any = any(v is not None for v in d["metrics_ms"].values())
    if not has_any:
        # В логе ничего не нашли — маркер partial_data
        d["partial_data"] = True

    if args.json:
        # Дополним контекстом — voice_text, log_path, ts of computation
        import datetime as _dt
        d["voice_text"] = args.voice_text
        d["log_path"] = args.log_path
        d["computed_at"] = _dt.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")
        print(json.dumps(d, ensure_ascii=False, indent=2))
    else:
        print(render_text(d, args.voice_text))
    return 0


if __name__ == "__main__":
    sys.exit(main())
