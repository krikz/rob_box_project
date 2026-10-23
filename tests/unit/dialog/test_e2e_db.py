"""turn_metrics/honesty_audit --db читают e2e-БД диалога v2 (ADR-0150 §14, PR-8 #3702).

БД пишет тот же писатель, что и рантайм (``rob_box_dialog.e2e_events``): тест сверяет контракт чтения с записью.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
for _pkg in ("rob_box_core", "rob_box_dialog"):
    sys.path.insert(0, str(REPO / "src" / _pkg))
sys.path.insert(0, str(REPO / "scripts" / "dialog"))

import e2e_db  # noqa: E402
import honesty_audit  # noqa: E402
import turn_metrics  # noqa: E402
from rob_box_dialog.e2e_events import open_events  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture()
def db(tmp_path):
    clock = Clock()
    ev = open_events(str(tmp_path / "e2e.db"), "s1", clock=clock)
    agent = "personality"
    # ход 1: LLM, один ответ, звук сыграл, речь через 1.8 с
    ev.write("user", turn_id="t1", epoch=0, agent=agent, text="покажи анимацию сон")
    ev.write("command", turn_id="t1", epoch=0, agent=agent, data={"provider": "deepseek", "valid": True})
    ev.write("action", turn_id="t1", epoch=0, agent=agent, data={"tool": "play_animation", "status": "done"})
    clock.t += 1.8
    ev.write("speech_started", turn_id="t1", epoch=0, agent=agent, text="Показываю анимацию «сон».")
    ev.write("speech_spoken", turn_id="t1", epoch=0, agent=agent, text="Показываю анимацию «сон».")
    ev.write("turn", turn_id="t1", epoch=0, agent=agent,
             data={"stage": "done", "status": "finished", "llm_calls": 1, "llm_provider": "deepseek"})
    # ход 2: Tier-1, 0 LLM
    clock.t += 10
    ev.write("user", turn_id="t2", epoch=0, agent=agent, text="тише")
    clock.t += 0.4
    ev.write("speech_started", turn_id="t2", epoch=0, agent=agent, text="Тише.")
    ev.write("turn", turn_id="t2", epoch=0, agent=agent, data={"stage": "done", "status": "finished", "llm_calls": 0})
    # ход 3: два ответа minimax, первый невалиден; речь с маркером и выдуманным «записал»
    clock.t += 10
    ev.write("user", turn_id="t3", epoch=0, agent=agent, text="запомни что я Саша")
    ev.write("command", turn_id="t3", epoch=0, agent=agent,
             data={"provider": "minimax", "valid": False, "reason": "bad_json"})
    ev.write("command", turn_id="t3", epoch=0, agent=agent, data={"provider": "minimax", "valid": True})
    clock.t += 5.0
    ev.write("speech_started", turn_id="t3", epoch=0, agent=agent, text="[Spkr:Саша] Записал, Саша.")
    ev.write("speech_spoken", turn_id="t3", epoch=0, agent=agent, text="[Spkr:Саша] Записал, Саша.")
    ev.write("turn", turn_id="t3", epoch=0, agent=agent,
             data={"stage": "done", "status": "finished", "llm_calls": 2, "llm_provider": "minimax"})
    ev.close()
    return tmp_path / "e2e.db"


def test_load_groups_by_turn(db):
    turns = e2e_db.load(db)
    assert [t.turn_id for t in turns] == ["t1", "t2", "t3"]
    assert turns[0].first_sound_ms == pytest.approx(1800.0)
    assert turns[0].tools_done == ("play_animation",)
    assert turns[1].aggregate["llm_calls"] == 0


def test_markers_and_providers(db):
    turns = e2e_db.load(db)
    assert e2e_db.marker_hits(turns) == [("t3", "[Spkr:Саша] Записал, Саша.")]
    assert e2e_db.per_provider(turns) == {"deepseek": (1, 0), "minimax": (2, 1)}


def test_turn_metrics_db_report(db, capsys):
    assert turn_metrics.main(["--db", str(db)]) == 0
    out = capsys.readouterr().out
    assert "Ходов в e2e-БД: 3; с агрегатом: 3; без LLM (Tier-1 и др.): 1." in out
    assert "| A1 `llm_calls` на ход (p50 = 1, p100 ≤ 3) | 3 | 1 | 2 | 2 |" in out
    assert "- A13 служебный текст в начатой речи: 1" in out
    assert "- A14 `minimax`: ответов 2, невалидных 1 (50 %; порог ≤ 10 %)." in out


def test_honesty_audit_db_flags_claim_without_action(db, capsys):
    assert honesty_audit.main(["--db", str(db)]) == 1
    findings = honesty_audit.audit_db(e2e_db.load(db))
    assert [(f.turn, f.rule, f.verdict) for f in findings] == [(3, "memory_saved", "claim_without_tool")]


def test_log_mode_still_requires_log():
    with pytest.raises(SystemExit):
        turn_metrics.main([])
