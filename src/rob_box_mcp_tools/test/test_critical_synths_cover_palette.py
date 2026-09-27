"""Every synth the prompt offers must be one the robot can re-send.

Live 30.08: the robot played a beat with no melody. The model had picked
``supersawlead`` for the lead, scsynth did not hold that SynthDef, ``/s_new``
was rejected, and the layer was silently absent — drums, bass and pad played
on, so the track sounded merely "wrong" rather than broken.

A live probe of scsynth found 15 advertised synths missing. Nine of them —
arpy, pianovel, cs80lead, supersawlead, dirt, moogbass, strangerpulsepad,
rave, donk — were in neither the startup preload nor ``CRITICAL_SYNTHS``,
the list ``_verify_and_resend_synthdefs`` uses to repair the server at play
time. Synths that WERE on that list got re-sent and worked, so the repair
mechanism was fine; its coverage was the hole.

The palette therefore has to stay a subset of what we can repair. This is
the third copy of a synth-name list in the tree, and the second time drift
between copies has silenced the robot, so it is pinned here.
"""
from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


def _repo_root(start: Path) -> Path:
    for parent in [start, *start.parents]:
        if (parent / "src").is_dir() and (parent / "docker").is_dir():
            return parent
    raise RuntimeError("repo root not found")


REPO_ROOT = _repo_root(Path(__file__).resolve())
MASTER_PROMPT = (
    REPO_ROOT / "src" / "rob_box_voice" / "prompts" / "master_prompt_compact.txt"
)
MUSIC_PY = (
    REPO_ROOT / "src" / "rob_box_mcp_tools" / "rob_box_mcp_tools" / "tools" / "music.py"
)


def _critical_synths() -> tuple:
    """Read CRITICAL_SYNTHS out of music.py without importing the package.

    ``rob_box_mcp_tools.tools`` pulls in rclpy through its ``__init__``, which
    is not available off-robot; the constant itself is a plain literal, so the
    AST is enough and this test stays runnable anywhere.
    """
    tree = ast.parse(MUSIC_PY.read_text(encoding="utf-8"))
    for node in tree.body:
        targets = (
            [node.target] if isinstance(node, ast.AnnAssign) else
            getattr(node, "targets", [])
        )
        for target in targets:
            if isinstance(target, ast.Name) and target.id == "CRITICAL_SYNTHS":
                return tuple(ast.literal_eval(node.value))
    raise AssertionError("CRITICAL_SYNTHS not found at module level in music.py")


CRITICAL_SYNTHS = _critical_synths()


def _advertised_palette() -> set[str]:
    """Synth names the master prompt offers the model, from its palette block."""
    text = MASTER_PROMPT.read_text(encoding="utf-8")
    match = re.search(
        r"- Synth palette —(.*?)(?=\n- [A-ZА-Я])", text, re.DOTALL
    )
    assert match, "palette block not found in master_prompt_compact.txt"
    block = match.group(1)
    # Names are lowercase words in comma-separated runs after each category
    # label; the labels themselves ("melody", "Bass") are filtered by the
    # explicit skip set below.
    names = set(re.findall(r"\b([a-z][a-z0-9]{2,20})\b", block))
    return names - {"melody", "bass", "pads", "brass", "glitch"} | {"bass", "pads", "brass"}


def test_the_palette_block_is_still_parseable() -> None:
    """Guard the regex itself — a silently empty palette would pass everything."""
    palette = _advertised_palette()
    assert len(palette) >= 30, f"palette looks truncated: {sorted(palette)}"
    assert "supersawlead" in palette


@pytest.mark.parametrize("synth", sorted(_advertised_palette()))
def test_every_advertised_synth_can_be_resent(synth: str) -> None:
    assert synth in CRITICAL_SYNTHS, (
        f"{synth!r} рекламируется палитрой, но его нет в CRITICAL_SYNTHS — "
        "если scsynth его потеряет, досылка не сработает и слой замолчит"
    )


@pytest.mark.parametrize(
    "synth",
    ["arpy", "pianovel", "cs80lead", "supersawlead", "dirt", "moogbass",
     "strangerpulsepad", "rave", "donk"],
)
def test_synths_missing_from_the_live_server_are_covered(synth: str) -> None:
    """The nine confirmed absent on the robot on 30.08."""
    assert synth in CRITICAL_SYNTHS


def test_no_duplicates() -> None:
    dupes = {n for n in CRITICAL_SYNTHS if list(CRITICAL_SYNTHS).count(n) > 1}
    assert not dupes, f"дубли в CRITICAL_SYNTHS: {sorted(dupes)}"
