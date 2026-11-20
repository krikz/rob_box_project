"""Кейсы смотрителя (#4033 шаг 2а): агрегация по сигнатуре, без текстов фраз, журнал вне репо."""

from __future__ import annotations

import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "rob_box_dialog"))
sys.path.insert(0, str(REPO / "scripts" / "observability"))

import log_cases as lc  # noqa: E402
import log_watch as lw  # noqa: E402

B = 1793743000.0
SECRET = "СЕКРЕТНАЯ_РЕПЛИКА_ЧЕЛОВЕКА"
PHRASE = "всох в глуши музыку"
CLEAN = "включи музыку громче"


def _stt(ts: float, body: str) -> str:
    return f"[stt_node-5] [WARN] [{ts:.9f}] [stt_node]: [stt_attempt] {body}"


def _dialog(ts: float, body: str) -> str:
    return f"[dialog_node-3] [INFO] [{ts:.9f}] [dialog_node]: {body}"


def _accepted(ts: float, phrase: str) -> str:
    return _stt(ts, f"yandex:ok(80ms) -> accepted '{phrase}'")


def _widened(ts: float, tools: str) -> str:
    return _dialog(ts, f"🧾 [command] tool_not_available → в запрос добавлены тулы [{tools}]")


DEAF = [
    _stt(B + 0.0, f"yandex:empty(1ms) -> rejected {SECRET}"),
    _stt(B + 10.0, "minimax:dead(0ms)->yandex:timeout(9ms) -> rejected"),
    _stt(B + 20.0, "yandex:timeout(9ms)->vosk:error(1ms) -> rejected"),
]
ODD = [
    _accepted(B + 100.0, PHRASE),
    _accepted(B + 130.0, PHRASE),
    _accepted(B + 400.0, "хватит глушу"),
]


def test_series_case_carries_reason_and_duration():
    cases, _ = lc.build_cases(DEAF)
    assert len(cases) == 1
    case = cases[0]
    assert case.kind == "series" and case.code.startswith("deaf:")
    assert "stt_rejected=3" in case.code and "duration_s" not in case.code
    assert case.duration_s == 20 and case.signature


def test_same_signature_aggregates_not_duplicates():
    cases, _ = lc.build_cases(ODD[:2])
    assert len(cases) == 1, [c.code for c in cases]
    assert cases[0].kind == "odd_stt" and cases[0].count == 2
    assert cases[0].first_ts == B + 100.0 and cases[0].last_ts == B + 130.0


def test_different_distortion_is_another_case():
    cases, _ = lc.build_cases(ODD)
    codes = {c.code for c in cases}
    assert len(cases) == 2, codes
    assert all(c.kind == "odd_stt" for c in cases)
    assert all("глуш" in c.code for c in cases)
    assert all("всох" not in c.code and "глушу" not in c.code for c in cases)


def test_default_output_has_no_phrase_text():
    summary = lc.cases_summary(DEAF + ODD, B - 10, B + 3000)
    dumped = json.dumps(summary, ensure_ascii=False)
    assert SECRET not in dumped and PHRASE not in dumped
    for word in ("всох", "глушу"):
        assert word not in dumped
    assert PHRASE not in lc.render_cases(summary)
    assert all("samples" not in item for item in summary["cases"]["items"])


def test_uncovered_tool_is_a_case_without_request_text():
    lines = [_widened(B + 5.0, "'make_tea', 'make_tea'"), _widened(B + 50.0, "'make_tea'")]
    cases, _ = lc.build_cases(lines)
    assert len(cases) == 1 and cases[0].count == 2
    assert cases[0].kind == "uncovered" and cases[0].code == "tool_not_available:make_tea"
    assert SECRET not in cases[0].code


def test_clean_command_and_chatter_are_not_cases():
    lines = [
        _accepted(B + 1.0, "сегодня хорошая музыка на улице и вообще"),
        _accepted(B + 2.0, CLEAN),
    ]
    cases, _ = lc.build_cases(lines)
    assert cases == []


def test_empty_window_says_no_events():
    summary = lc.cases_summary([], B - 10, B + 100)
    assert summary["status"] == "no_events"
    text = lc.render_cases(summary)
    assert "нет" in text and "Кейсов 0" not in text


def test_min_count_hides_rare_cases():
    summary = lc.cases_summary(ODD, B - 10, B + 3000, min_count=2)
    assert summary["cases"]["total"] == 1
    assert summary["cases"]["items"][0]["count"] == 2


def test_journal_written_outside_repo_without_phrases(tmp_path):
    journal = tmp_path / "journal"
    private = tmp_path / "private"
    lines = DEAF + ODD
    cases, harness = lc.build_cases(lines)
    assert harness is False
    path = lc.write_journal(cases, journal, harness, private)
    assert path.parent == journal and path.suffix == ".json"
    body = path.read_text(encoding="utf-8")
    assert SECRET not in body and PHRASE not in body
    md = list(journal.glob("*.md"))
    assert md and PHRASE not in md[0].read_text(encoding="utf-8")
    assert not private.exists()
    assert not str(path).startswith(str(REPO))


def test_private_samples_only_for_harness_window(tmp_path):
    journal, private = tmp_path / "j", tmp_path / "p"
    live = [_accepted(B + 1.0, PHRASE), _accepted(B + 2.0, PHRASE)]
    cases, harness = lc.build_cases(live)
    lc.write_journal(cases, journal, harness, private)
    assert harness is False and not private.exists()

    marked = [f"[dialog_node-3] [INFO] [{B:.9f}] [dialog_node]: 🧪 [e2e] e2e-БД: /tmp/x.db", *live]
    cases, harness = lc.build_cases(marked)
    assert harness is True
    lc.write_journal(cases, journal, harness, private)
    samples = list(private.glob("*.json"))
    assert samples and PHRASE in samples[0].read_text(encoding="utf-8")
    assert all(PHRASE not in p.read_text(encoding="utf-8") for p in journal.iterdir())


def test_old_cli_unchanged(tmp_path, capsys):
    log = tmp_path / "x.log"
    log.write_text("\n".join(DEAF), encoding="utf-8")
    out = tmp_path / "s.json"
    assert lw.main(["--lines-file", str(log), "--json", str(out)]) == 0
    saved = json.loads(out.read_text(encoding="utf-8"))
    assert saved["mode"] == "dry_run" and "cases" not in saved
    assert "Робот глух: 1" in capsys.readouterr().out


def test_cases_cli_writes_journal_and_json(tmp_path, capsys):
    log = tmp_path / "x.log"
    log.write_text("\n".join(ODD[:2]), encoding="utf-8")
    journal = tmp_path / "journal"
    out = tmp_path / "c.json"
    rc = lw.main(["--lines-file", str(log), "--cases", "--journal-dir", str(journal), "--json", str(out)])
    assert rc == 0
    saved = json.loads(out.read_text(encoding="utf-8"))
    assert saved["mode"] == "cases" and saved["cases"]["total"] == 1
    assert PHRASE not in capsys.readouterr().out
    written = list(journal.glob("*.json"))
    assert written and PHRASE not in written[0].read_text(encoding="utf-8")
