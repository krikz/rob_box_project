"""Static regression guard for issue #1709 — RULE #UNICODE-SPEECH.

Issue #1709: юзер услышал «бормотание на хинди». Причина — LLM
периодически вставляет в ``speak_text`` символы неподдерживаемых
письменностей (CJK / деванагари / арабица), а TTS читает их нечитаемо.

Acceptance issue #1709 требует: «Prompt должен ЯВНО запрещать LLM
использовать иероглифы в speech». Runtime-половина фикса живёт в
``rob_box_voice/tts_text_guard.py`` (см.
``test/unit/core/test_tts_text_guard.py``); этот файл пиннит формулировки
в промпте, чтобы будущий мёрдж не откатил правило молча — как это уже
случалось с «150 characters» (issue #1377, см.
``test_prompt_no_legacy_limit.py``).

Run with::

    cd src/rob_box_voice && python3 -m pytest \\
        test/unit/test_issue_1709_prompt_unicode_speech.py -v
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


def test_master_prompt_has_unicode_speech_rule() -> None:
    """§1 CORE INVARIANTS must carry the named rule."""
    content = _read()
    assert "RULE #UNICODE-SPEECH" in content, (
        "master_prompt_compact.txt missing RULE #UNICODE-SPEECH "
        "(issue #1709). Без него LLM снова начнёт вставлять иероглифы "
        "в speak_text и робот будет бормотать."
    )


def test_rule_lives_in_core_invariants_section() -> None:
    """The rule must sit in §1 (always applied), not in a niche section.

    §1 is the only section the prompt tells the model to read before
    anything else; a rule about EVERY spoken chunk belongs there.
    """
    content = _read()
    core_start = content.index("# 1. CORE INVARIANTS")
    core_end = content.index("# 2. CYCLE TERMINATION")
    core_section = content[core_start:core_end]
    assert "RULE #UNICODE-SPEECH" in core_section, (
        "RULE #UNICODE-SPEECH must live inside §1 CORE INVARIANTS "
        "(apply ALWAYS), not in a later section the model may skip."
    )


def test_rule_names_the_banned_scripts() -> None:
    """The rule must name the scripts explicitly, not hand-wave 'Unicode'."""
    content = _read()
    for script in ("CJK", "Devanagari", "Arabic", "Hangul"):
        assert script in content, (
            f"master_prompt_compact.txt must explicitly name {script} as a "
            "banned script in speak_text (issue #1709 acceptance)."
        )


def test_rule_prescribes_transliteration_alternative() -> None:
    """A ban with no alternative gets ignored — the prompt must offer translit.

    Acceptance issue #1709: «Если нужна идиома — пиши транслитерацию».
    """
    content = _read()
    assert "транслитерацию" in content or "транслит" in content, (
        "RULE #UNICODE-SPEECH must tell the LLM what to do INSTEAD "
        "(write the foreign phrase transliterated in Russian letters)."
    )


def test_rule_documents_runtime_enforcement_contract() -> None:
    """The prompt must mention the runtime rejection code.

    ``SpeakTextTool`` returns ``unsupported_script`` when a chunk trips
    the guard. The LLM has to recognise that error and rewrite the
    phrase instead of retrying the same text forever.
    """
    content = _read()
    assert "unsupported_script" in content, (
        "master_prompt_compact.txt must document the runtime "
        "``unsupported_script`` rejection so the LLM rewrites the phrase "
        "instead of re-sending the same hieroglyphs."
    )


def test_routing_table_mentions_the_rule() -> None:
    """§0 STRUCTURAL ORIENTATION routing table must list the rule.

    The table is the model's index into the prompt; a rule missing from
    it is a rule the model may never route to.
    """
    content = _read()
    header_end = content.index("# 1. CORE INVARIANTS")
    header = content[:header_end]
    assert "#UNICODE-SPEECH" in header, (
        "§0 routing table must list #UNICODE-SPEECH among the §1 core "
        "invariants (issue #1709)."
    )


def test_speak_text_tool_card_repeats_the_constraint() -> None:
    """§4 speak_text card must repeat the letters-only constraint.

    The model picks the tool from §4; the constraint has to be visible
    at the point of use, mirroring how the animation enum and the
    200-character cap are pinned there (issue #1377).
    """
    content = _read()
    card_start = content.index("### speak_text(text, animation)")
    card_end = content.index("### set_voice(voice)")
    card = content[card_start:card_end]
    assert "UNICODE-SPEECH" in card, (
        "The speak_text tool card (§4) must cross-reference "
        "RULE #UNICODE-SPEECH so the constraint is visible where the "
        "model actually composes the call."
    )


def test_text_rules_section_covers_foreign_scripts() -> None:
    """§7 TEXT RULES must cover foreign scripts alongside math Unicode."""
    content = _read()
    rules_start = content.index("# 7. TEXT RULES")
    rules_end = content.index("# 8. CHARACTER")
    rules = content[rules_start:rules_end]
    assert "foreign scripts" in rules.lower(), (
        "§7 TEXT RULES already bans math Unicode (α, ∫); it must also ban "
        "foreign scripts and point at RULE #UNICODE-SPEECH (issue #1709)."
    )
