# -*- coding: utf-8 -*-
"""#4478 (резюме марафона START_FROM_ACT) и #4481 (n603 приёмка, n101 повтор вейка).

Run:
    python -m pytest tests/unit/e2e_scripts/test_issue_4478_4481_resume_and_acceptance.py -v -o addopts=""
"""
from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
for _dir in (ROOT / "scripts" / "e2e", ROOT / "scripts" / "ci" / "e2e"):
    if str(_dir) not in sys.path:
        sys.path.insert(0, str(_dir))

import e2e_events as eev  # noqa: E402

EVENTS = ROOT / "scripts" / "ci" / "e2e" / "e2e_events.py"
NIGHT = ROOT / ".github" / "e2e" / "scenarios" / "night"
MARATHON = ROOT / "scripts" / "e2e" / "run_night_marathon.sh"


def test_resumed_source_skips_dependent_with_reason():
    got = eev.requires_verdict(("n203_sasha_memory_tea",), {}, {"n203_sasha_memory_tea": "RESUMED"})
    assert got["skip"] == "resume:n203_sasha_memory_tea"
    assert got["evidence"] == "резюме: факты пропущенных актов недоступны"


def test_current_act_wins_and_fail_stays_requires():
    # итог этого акта важнее прошлых: шаг, сыгранный в акте, не считается пропущенным резюме
    assert eev.requires_verdict(("a",), {"a": "OK"}, {"a": "RESUMED"})["skip"] == "-"
    assert eev.requires_verdict(("a",), {}, {"a": "FAIL"})["skip"] == "requires:a"
    assert eev.requires_verdict(("a",), {}, {"a": "OK"})["skip"] == "-"


def test_prior_statuses_parses_resumed():
    assert eev.prior_statuses("n101_x=RESUMED,n203=OK") == {"n101_x": "RESUMED", "n203": "OK"}


def test_cli_require_resumed(tmp_path):
    out = subprocess.run(
        [sys.executable, str(EVENTS), "require", "--label", "n1001", "--labels", "n203_sasha_memory_tea",
         "--steps", str(tmp_path / "none.jsonl"), "--prior", "n203_sasha_memory_tea=RESUMED"],
        capture_output=True, text=True, encoding="utf-8")
    assert "skip=resume:n203_sasha_memory_tea" in out.stdout, out


def test_runner_default_assumed_status_is_resumed_not_ok():
    text = MARATHON.read_text(encoding="utf-8")
    assert 'PRIOR_STEPS_ASSUMED="${PRIOR_STEPS_ASSUMED:-RESUMED}"' in text


def _steps():
    found = {}
    for path in NIGHT.glob("night_marathon_act*_v1.json"):
        if "acceptance" in path.name:
            continue
        for step in json.loads(path.read_text(encoding="utf-8")).get("steps", []):
            found[step["label"]] = step
    return found


def test_n603_accepts_dj_set_phrasing():
    kw = _steps()["n603_confirm_dj_playing"]["acceptance"]["expected_keywords"][0]
    assert re.search(kw, "Идёт сет ночная дискотека, первый трек из десяти: Staying Alive", re.IGNORECASE)
    assert re.search(kw, "Идёт сет ночная дискотека", re.IGNORECASE)
    assert re.search(kw, "первый трек из десяти", re.IGNORECASE)
    # узость сохранена: голое «идёт» не засчитывается
    assert not re.search(kw, "Идёт речь о погоде", re.IGNORECASE)


def test_n101_allows_one_wake_replay():
    n101 = _steps()["n101_wake_cold_start"]
    assert eev.WAKE_REPLAY in n101["recover"]
    assert "no_wake" in n101["recover"]
    eev.recover_kinds({}, n101)  # схема поля валидна
