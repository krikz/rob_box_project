"""Смотритель логов вхолостую (#4033): счётчики вердиктов на синтетических строках, без Loki и без текстов."""

from __future__ import annotations

import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
for _pkg in ("rob_box_core", "rob_box_dialog"):
    sys.path.insert(0, str(REPO / "src" / _pkg))
sys.path.insert(0, str(REPO / "scripts" / "observability"))

import log_watch as lw  # noqa: E402

B = 1793743000.0
SECRET = "СЕКРЕТНАЯ_РЕПЛИКА_ЧЕЛОВЕКА"


def _stt(ts: float, body: str) -> str:
    return f"[stt_node-5] [WARN] [{ts:.9f}] [stt_node]: [stt_attempt] {body}"


def _dialog(ts: float, body: str) -> str:
    return f"[dialog_node-3] [INFO] [{ts:.9f}] [dialog_node]: {body}"


def _turn_end(ts: float, tid: str) -> str:
    return _dialog(ts, f"📊 [turn] turn_id={tid} epoch=1 agent=personality stage=done status=finished latency_ms=1")


DEAF_LINES = [
    _stt(B + 0.0, f"yandex:empty(1ms)->vosk:empty(1ms) -> rejected {SECRET}"),
    _stt(B + 10.0, "minimax:dead(0ms)->yandex:timeout(9ms) -> rejected"),
    _stt(B + 20.0, "yandex:timeout(9ms)->vosk:error(1ms) -> rejected"),
]
SILENT_TURN = [
    _dialog(B + 1000.0, "🧭 [turn-v2] форма команды: закрыты=[] пение=- turn=0123456789ab"),
    _turn_end(B + 1001.0, "0123456789ab"),
]


def test_counts_deaf_and_no_speech():
    s = lw.summarize(DEAF_LINES + SILENT_TURN, B - 10, B + 3000)
    assert s["status"] == "ok"
    assert s["verdicts"]["deaf"]["count"] == 1
    assert s["verdicts"]["deaf"]["by_outcome"] == {"stt_error": 2, "stt_rejected": 3}
    assert s["verdicts"]["deaf"]["with_stt_error_ge_2"] == 1
    assert s["verdicts"]["no_speech"] == {"count": 1, "by_reason": {"empty": 1}}
    assert s["events"]["turns"] == 1 and s["events"]["stt_rejected"] == 3


def test_no_utterance_text_in_output():
    s = lw.summarize(DEAF_LINES + SILENT_TURN, B - 10, B + 3000)
    assert SECRET not in json.dumps(s, ensure_ascii=False)
    assert SECRET not in lw.render_text(s)


def test_empty_window_is_honest_not_zero_failures():
    s = lw.summarize([], B - 10, B + 3000)
    assert s["status"] == "no_events"
    text = lw.render_text(s)
    assert "событий" in text.lower() and "нет" in text
    assert "Робот глух: 0" not in text


def test_window_with_events_and_no_failures_says_zero():
    lines = [
        _dialog(B + 10.0, "🧭 [turn-v2] форма команды: закрыты=[] пение=- turn=aaaaaaaaaaaa"),
        _dialog(B + 10.5, "🧭 [turn-v2] tier1 kind=stop_speech args={} turn=aaaaaaaaaaaa"),
        _turn_end(B + 11.0, "aaaaaaaaaaaa"),
    ]
    s = lw.summarize(lines, B - 10, B + 100)
    assert s["status"] == "ok" and s["verdicts"]["total"] == 0
    assert "Робот глух: 0" in lw.render_text(s)


def test_cli_writes_json_without_sending(tmp_path, capsys):
    log = tmp_path / "x.log"
    log.write_text("\n".join(DEAF_LINES), encoding="utf-8")
    out = tmp_path / "s.json"
    assert lw.main(["--lines-file", str(log), "--json", str(out)]) == 0
    assert json.loads(out.read_text(encoding="utf-8"))["mode"] == "dry_run"
    assert "Робот глух: 1" in capsys.readouterr().out
