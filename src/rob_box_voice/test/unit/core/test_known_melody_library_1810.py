"""test_known_melody_library_1810.py — issue #1810 regression guards.

31.08 live incident: "сыграй в траве сидел кузнечик" produced
``blip([0,2,4,7,9,7,4,2], ...)`` — a plain up-and-down scale arc, not the
requested tune. The SAME arc (``[0,2,4,7,...]`` and rotations of it) turned
up as the melody for jazz, disco, a DJ-set finale, and a Django-style track
on the same evening — proof the model was stamping out one scale shape
instead of composing or recalling anything.

The fix has three parts, each pinned down here:

1. KNOWN MELODY LIBRARY gained real, hand-verified children's/folk tunes
   written in scale degrees (not the banned scale-run shape).
2. A HONESTY RULE: a melody requested by name that isn't in the library
   must be searched for (``search_web``) or admitted unknown — never
   silently replaced by an improvisation passed off as the real thing.
3. A motif-construction rule bans the exact arc shape from the incident
   and requires leap+fill / repetition / a tonic anchor instead.

Pure text checks against the prompt file — no ROS2, no SuperCollider.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest


def _repo_root(start: Path) -> Path:
    for parent in [start, *start.parents]:
        if (parent / "docker").is_dir() and (parent / "src").is_dir():
            return parent
    return start.parents[5]


REPO_ROOT = _repo_root(Path(__file__).resolve())
MUSIC_SKILL_PROMPT = (
    REPO_ROOT / "src" / "rob_box_voice" / "prompts" / "skills" / "music_skill_prompt.txt"
)


@pytest.fixture(scope="module")
def prompt_text() -> str:
    return MUSIC_SKILL_PROMPT.read_text(encoding="utf-8")


#: Required real-world tunes (issue #1810 explicitly names these five).
REQUIRED_SONGS = (
    "КУЗНЕЧИК",
    "ЁЛОЧКА",
    "ЧИЖИК-ПЫЖИК",
    "СОБАЧИЙ ВАЛЬС",
    "HAPPY BIRTHDAY",
)


def _extract_degree_calls(text: str) -> list[tuple[str, list, list | None]]:
    """Find ``synth([degrees...], dur=[...], ...)`` calls with a literal list.

    Returns ``(synth_name, degrees, durations_or_None)`` for every match —
    durations are ``None`` when ``dur=`` is a scalar instead of a list
    (still valid Renardo, just nothing to length-check).
    """
    calls: list[tuple[str, list, list | None]] = []
    # p1 >> synth([...], dur=[...] or scalar, ...)
    #
    # ``[^)]*?`` rather than ``[^\n]*?``: real entries in the prompt wrap
    # their argument list over several lines, so a same-line-only pattern
    # matched just the one single-line entry (Собачий вальс) and the
    # >= 4 assertion below failed on a library that was in fact complete.
    # Stopping at ``)`` still keeps the match inside one call — a degree
    # list contains ``]`` but never ``)``.
    pattern = re.compile(
        r"[a-z][a-z0-9]*\(\s*(\[[^\]]*\])\s*,[^)]*?dur\s*=\s*(\[[^\]]*\]|[0-9.]+)",
        re.S,
    )
    for m in pattern.finditer(text):
        degrees_src, dur_src = m.group(1), m.group(2)
        try:
            degrees = eval(degrees_src, {"__builtins__": {}})  # noqa: S307 — literal list only
        except Exception:
            continue
        durations = None
        if dur_src.startswith("["):
            try:
                durations = eval(dur_src, {"__builtins__": {}})  # noqa: S307
            except Exception:
                durations = None
        name = text[m.start():m.start() + 30].split("(")[0].split(">>")[-1].strip()
        calls.append((name, degrees, durations))
    return calls


def test_required_songs_are_named_in_the_library(prompt_text: str) -> None:
    """All five songs from issue #1810 must be present by name."""
    upper = prompt_text.upper()
    missing = [song for song in REQUIRED_SONGS if song not in upper]
    assert not missing, f"missing from KNOWN MELODY LIBRARY: {missing}"


def test_russian_songs_section_has_parseable_degree_lists(prompt_text: str) -> None:
    """Each new song's note list must be a valid Python list, and — when a
    duration list is given — must be the same length as the note list.

    A mismatched length is the single most common way a hand-written melody
    silently breaks (Renardo either truncates or errors at runtime), and
    it's exactly the kind of mistake "no test caught it" lets through.
    """
    start = prompt_text.index("RUSSIAN & CHILDREN'S / FOLK SONGS")
    end = prompt_text.index("HAPPY BIRTHDAY:")
    section = prompt_text[start:end]

    calls = _extract_degree_calls(section)
    assert len(calls) >= 4, (
        f"expected at least 4 parseable note/dur calls in the new songs "
        f"section, found {len(calls)}"
    )
    for name, degrees, durations in calls:
        assert degrees, f"{name}: empty degree list"
        assert all(isinstance(d, int) for d in degrees), (
            f"{name}: degree list must be plain integers, got {degrees}"
        )
        if durations is not None:
            assert len(durations) == len(degrees), (
                f"{name}: {len(degrees)} notes but {len(durations)} durations "
                f"— {degrees} vs {durations}"
            )


def test_kuznechik_opens_on_the_confirmed_hook(prompt_text: str) -> None:
    """The one fact issue #1810 gives us directly: "соль-соль-ми,
    соль-соль-ми" — scale degrees [4,4,2,4,4,2] in do-based numbering.
    Getting this part right is the actual fix; the rest of the verse is
    best-effort. Regression guard so nobody "simplifies" it back into a
    scale run later.
    """
    start = prompt_text.index("КУЗНЕЧИК")
    snippet = prompt_text[start:start + 700]
    assert "[4,4,2, 4,4,2" in snippet or "[4, 4, 2, 4, 4, 2" in snippet, (
        "the confirmed соль-соль-ми,соль-соль-ми hook (degrees 4,4,2 x2) "
        "must open the Кузнечик entry verbatim"
    )
    # The banned scale-run shape from the live incident must not reappear
    # as "the fix" for the very issue that flagged it.
    for banned in ("[0,2,4,7,9,7,4,2]", "[0, 2, 4, 7, 9, 7, 4, 2]"):
        assert banned not in snippet


@pytest.mark.parametrize(
    "banned_arc",
    ["[0,2,4,7,9,7,4,2]", "[0,2,4,7,6,4,2]", "[0,2,4,7,4,2,0]"],
)
def test_motif_rule_bans_the_live_incident_arcs(prompt_text: str, banned_arc: str) -> None:
    """The exact arcs from the 31.08 incident (jazz/disco/DJ-finale/pluck)
    must be named as the ❌ example, not left as an unremarked pattern the
    model could still copy from elsewhere in the prompt as "an example"."""
    assert banned_arc in prompt_text, (
        f"{banned_arc!r} should appear as a named ❌ counter-example so the "
        "model recognises and avoids it, not just be silently absent"
    )
    idx = prompt_text.index(banned_arc)
    # It must be flagged, not presented as usable material.
    nearby = prompt_text[max(0, idx - 5):idx]
    assert "❌" in nearby, f"{banned_arc!r} appears without a ❌ ban marker nearby"


def test_motif_rule_requires_leap_repetition_or_anchor(prompt_text: str) -> None:
    section_start = prompt_text.index("A MOTIF IS NOT A SCALE RUN")
    section = prompt_text[section_start:section_start + 2200]
    for required in ("LEAP", "repeated", "ANCHOR", "tonic"):
        assert required.lower() in section.lower(), (
            f"motif rule missing required concept: {required!r}"
        )
    # At least two concrete ✅ examples, not just prose.
    assert section.count("✅") >= 2


def test_honesty_rule_present_and_forbids_silent_substitution(prompt_text: str) -> None:
    assert "HONESTY RULE" in prompt_text
    start = prompt_text.index("HONESTY RULE")
    section = prompt_text[start:start + 2400]

    # Must route through search_web as the fallback for an unknown named
    # melody before giving up.
    assert "search_web" in section
    # Must explicitly forbid claiming a substitute is the real thing.
    assert "лжет" in section.lower() or "лож" in section.lower() or "лгать" in section.lower() or "lie" in section.lower()
    # Must reference the actual live bug (playing a scale and calling it
    # "Кузнечик") so the rule is anchored to a concrete failure, not vague.
    assert "кузнечик" in section.lower()


def test_search_web_is_documented_in_the_music_skill_prompt(prompt_text: str) -> None:
    """Companion fix: search_web is a registered MCP tool (issue #1101) but
    was never mentioned in music_skill_prompt.txt, so guard #1409 would
    (correctly) flag it as an undocumented tool, and the model never
    considered it as a melody-lookup fallback. Both need it present here."""
    assert "search_web(" in prompt_text
    assert "untrusted" in prompt_text.lower() or "недоверен" in prompt_text.lower(), (
        "search_web results must be flagged as untrusted third-party text"
    )


def test_dj_section_uses_compose_music_not_handwritten_code(prompt_text: str) -> None:
    """Issue #1811: the static DJ-mode instructions at the top of the
    prompt (separate from the dynamic per-transition prompt in dj_mode.py)
    used to tell the model to write raw execute_music_code every transition
    with hand-managed pattern counts and amp sums. That's now the
    arranger's job."""
    dj_start = prompt_text.index("DJ MODE — RULE #1")
    dj_end = prompt_text.index("RENARDO CODE GOTCHAS") if "RENARDO CODE GOTCHAS" in prompt_text else dj_start + 3000
    dj_section = prompt_text[dj_start:dj_end]

    assert "compose_music" in dj_section
    assert "STEP 1. compose_music()" in dj_section

    # Old hand-written-code-only instructions must not remain as the
    # primary DJ instruction.
    assert "STEP 1. execute_music_code()" not in dj_section


def test_dj_section_does_not_require_manual_pattern_amp_bookkeeping(prompt_text: str) -> None:
    """Removed per issue #1811: the arranger + master filter own pattern
    counts and amp sums now, the model shouldn't compute them per transition."""
    dj_start = prompt_text.index("DJ TRANSITIONS — RULE #2")
    dj_end = prompt_text.index("RANDOMIZATION MANDATE")
    dj_section = prompt_text[dj_start:dj_end]

    for stale in (
        "MAX 6 PATTERNS total: no more than 2-3 drums",
        "SUM of all amp values",
        "ANTI-ESCALATION RULE",
    ):
        assert stale not in dj_section, f"{stale!r} is a pre-#1811 hand-coded limit"


def test_dj_section_mentions_new_compose_music_params(prompt_text: str) -> None:
    """hats_sample/perc/perc_sample/swing are new compose_music params
    (#1805/#1806) especially useful for DJ variety — the prompt should
    point the model at them."""
    dj_start = prompt_text.index("DJ TRANSITIONS — RULE #2")
    dj_end = prompt_text.index("RUNTIME-VALIDATED synths")
    dj_section = prompt_text[dj_start:dj_end]

    for param in ("hats_sample", "perc", "perc_sample", "swing"):
        assert param in dj_section, f"{param!r} missing from DJ transition guidance"


#: The three scale arcs issue #1810 names as the symptom. Written without
#: spaces so the check is insensitive to how a given example formats its list.
_BANNED_ARCS = ("0,2,4,7,9,7,4,2", "0,2,4,7,6,4,2", "0,2,4,7,4,2,0")


def test_prompt_examples_do_not_themselves_use_the_banned_arc(prompt_text: str) -> None:
    """The prompt must not teach the bug it bans.

    The canonical ``compose_music`` example carried
    ``lead_notes="0, 2, 4, 7, 4, 2"`` — a prefix of the banned
    ``[0,2,4,7,4,2,0]`` arc — for as long as the example has existed. A model
    that copies the one worked example in front of it will reproduce exactly
    the contour issue #1810 was filed about, no matter what the rule below it
    says. Rules lose to examples; the examples have to agree with the rule.
    """
    for match in re.finditer(r"(lead_notes|bass_notes|pad_notes)\s*=\s*\"([^\"]*)\"", prompt_text):
        field, value = match.group(1), match.group(2)
        compact = value.replace(" ", "")
        assert compact not in _BANNED_ARCS, (
            f"{field}=\"{value}\" is one of the banned #1810 scale arcs; "
            f"the prompt is demonstrating the exact mistake it forbids"
        )


def test_banned_arcs_are_named_as_counter_examples(prompt_text: str) -> None:
    """Each banned arc must still appear once — flagged with ❌, as a lesson."""
    section_start = prompt_text.index("A MOTIF IS NOT A SCALE RUN")
    section = prompt_text[section_start:section_start + 2200]
    for arc in _BANNED_ARCS:
        spaced = "[" + arc + "]"
        assert spaced in section, (
            f"banned arc {spaced} is no longer named as a ❌ counter-example — "
            f"the model needs to recognise the shape, not just be told to avoid "
            f"'scale runs' in the abstract"
        )
