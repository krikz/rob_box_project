"""Static regression guard for issue #968 (S5.3) — RULE #SEGMENT-PLAN.

Scheduler-segments-merge plan, S5.3: without an explicit prompt rule,
the LLM sees ``[SEGMENT PLAN]`` (S5.1/S5.2) but has no instruction to
actually use it — it could just start the song over from scratch or
call ``stop_music`` on top of a still-playing segment. This file pins
the wording so a future edit can't silently drop the rule (same
guard style as ``test_issue_1709_prompt_unicode_speech.py`` and
``test_prompt_no_legacy_limit.py``).

Run with::

    cd src/rob_box_voice && python3 -m pytest \\
        test/unit/test_issue_968_segment_plan_prompt.py -v
"""

from __future__ import annotations

from pathlib import Path


MASTER_PROMPT = (
    Path(__file__).resolve().parents[2]
    / "prompts"
    / "master_prompt_compact.txt"
)


def _read() -> str:
    return MASTER_PROMPT.read_text(encoding="utf-8")


def test_master_prompt_has_segment_plan_rule() -> None:
    content = _read()
    assert "RULE #SEGMENT-PLAN" in content, (
        "master_prompt_compact.txt missing RULE #SEGMENT-PLAN (issue #968, "
        "S5.3) — without it the LLM has no instruction to use "
        "[SEGMENT PLAN] and will restart the performance from scratch "
        "on every barge-in."
    )


def test_rule_mentions_segment_plan_and_rewriteable_segments() -> None:
    content = _read()
    idx = content.index("RULE #SEGMENT-PLAN")
    rule = content[idx:idx + 800]
    assert "[SEGMENT PLAN]" in rule
    assert "REWRITEABLE_SEGMENTS" in rule


def test_rule_points_at_task_delta_tool() -> None:
    content = _read()
    idx = content.index("RULE #SEGMENT-PLAN")
    rule = content[idx:idx + 800]
    assert "task_delta" in rule, (
        "RULE #SEGMENT-PLAN must tell the LLM to use task_delta to rewrite "
        "PENDING segments instead of restarting the performance."
    )


def test_rule_forbids_restart_and_stop_music() -> None:
    content = _read()
    idx = content.index("RULE #SEGMENT-PLAN")
    rule = content[idx:idx + 800].lower()
    assert "stop_music" in rule, (
        "RULE #SEGMENT-PLAN must explicitly say NOT to call stop_music on "
        "an active [SEGMENT PLAN] — that's the whole point of MERGE over "
        "REPLACE (§0.2 of the plan: MERGE only covers the voice channel, "
        "but the beat must keep playing)."
    )