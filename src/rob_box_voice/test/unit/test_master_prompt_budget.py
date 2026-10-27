"""Мастер-промпт — персона и протокол хода, а не свод правил (ADR-0150 §7.3, A5; ADR-0148; #3696).

Знание о тулах живёт в их схемах (видны только тулы среза хода), данные — в снимке хода; промпт обязан влезать в
бюджет системного блока ``knowledge.CONTEXT_BUDGET_TOKENS["system"]`` и держать ≤ 10 ``RULE #`` (до PR-15 — 0).
"""

from pathlib import Path

from rob_box_core.token_estimate import estimate_tokens
from rob_box_dialog import knowledge as kn
from rob_box_dialog.context import SNAPSHOT_TAG

PROMPT = (Path(__file__).resolve().parents[2] / "prompts" / "master_prompt_compact.txt").read_text(encoding="utf-8")


def test_master_prompt_fits_the_system_block_budget():
    assert estimate_tokens(PROMPT) <= kn.CONTEXT_BUDGET_TOKENS["system"]


def test_at_most_ten_hand_written_rules():
    assert sum("RULE #" in line for line in PROMPT.splitlines()) <= 10


def test_prompt_names_only_snapshot_sections_that_exist():
    """Промпт ссылается на разделы снимка по именам — каждое имя есть в таблице ``CONTEXT_SECTIONS``."""
    assert f"<{SNAPSHOT_TAG}>" in PROMPT
    named = {s for s in ("user_profile", "tts_context", "speech_gender", "providers", "music_state",
                         "recent_unaddressed", "shift_log", "hardware", "tts_voice", "reminder") if f"`{s}`" in PROMPT}
    assert named <= set(kn.CONTEXT_SECTIONS)
