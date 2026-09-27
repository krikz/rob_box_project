"""Regression tests for issue #1403 — «спой песенку про котика» must route
to ``generate_music`` (MiniMax Music API), not ``execute_music_code`` (Renardo).

E2E run #32158261168 surfaced this bug: the LLM was given the user phrase
«робот, спой песенку про котика», picked ``execute_music_code`` (Renardo
synthesizer) instead of ``generate_music`` (AI mp3 generator), and
answered the user with «У меня нет такой функции. Я умею играть мелодии
только через синтезатор Renardo». Root cause: the prompt's DISPATCH
section listed only «спой мне» → ``generate_music``; the variant
«спой песенку про X» was not in DISPATCH, so the LLM fell through to
Renardo via the generic «сыграй» / «играй» rule.

These tests are PURE PROMPT INSPECTION — they do not import the
``agents`` SDK or instantiate ``MusicSkill`` and so they run in every
CI lane (including the harness one, where the openai-agents SDK is not
installed). They are intentionally a separate file from
``test_music_skill_generated_music.py`` so the module-level
``pytestmark = skipif(not HAS_AGENTS_SDK)`` does not apply here.
"""

from __future__ import annotations

from pathlib import Path

import pytest


# Path resolution — works from the worktree root as well as the main
# repo checkout. The prompt file lives at:
#   <repo>/src/rob_box_voice/prompts/skills/music_skill_prompt.txt
# `__file__` is .../src/rob_box_voice/test/unit/skills/<this>.py,
# so parents[3] = .../src/rob_box_voice.
PROMPT_PATH = Path(__file__).resolve().parents[3] / "prompts" / "skills" / "music_skill_prompt.txt"


def _prompt_text() -> str:
    assert PROMPT_PATH.exists(), f"prompt not found: {PROMPT_PATH}"
    return PROMPT_PATH.read_text(encoding="utf-8")


def _dispatch_section() -> str:
    """Return the DISPATCH block from music_skill_prompt.txt (issue #1371).

    DISPATCH is the routing table that tells the LLM which Renardo / AI
    tool to call for each user phrase. It is bounded by the
    ``⚡ DISPATCH (issue #1371)`` marker and the closing namespace-default
    rule («NEVER guess from context…»).
    """
    text = _prompt_text()
    start = text.find("⚡ DISPATCH")
    assert start != -1, "DISPATCH marker missing in music_skill_prompt.txt"
    end = text.find("NEVER guess from context", start)
    assert end != -1, "DISPATCH end marker missing"
    return text[start:end]


class TestSingSongRoutesToGenerateMusic:
    """Regression guard for issue #1403.

    Every test in this class is a PURE PROMPT INSPECTION test — it does
    not instantiate any skill or call any LLM. The point is to assert
    that the LLM-visible prompt text contains the routing hint that
    would have prevented the e2e failure.
    """

    def test_dispatch_routes_sing_song_to_generate_music(self) -> None:
        """«спой песенку/частушку/куплет/строчку про <X>» → generate_music.

        The exact failing e2e phrase was «спой песенку про котика».
        The prompt must contain an explicit dispatch rule that maps
        the full pattern (verb «спой» + object «песенку/частушку/
        куплет/строчку» + «про <X>») to ``generate_music``.

        Without this rule the LLM defaults to Renardo's
        ``execute_music_code`` (which is a synthesizer, not a singer)
        and answers «У меня нет такой функции».
        """
        dispatch = _dispatch_section()
        dispatch_l = dispatch.lower()
        for marker in ("спой песенку", "частушку", "куплет", "строчку"):
            pos = dispatch_l.find(marker)
            assert pos != -1, (
                f"DISPATCH is missing «{marker}» — the LLM will fall "
                f"through to Renardo and answer «нет такой функции»"
            )
            window = dispatch_l[pos:pos + 320]
            assert "generate_music" in window, (
                f"«{marker}» found in DISPATCH but the expected "
                f"``generate_music`` tool is not in the surrounding "
                f"window ({window!r})"
            )

    def test_dispatch_does_not_route_sing_song_to_execute_music(self) -> None:
        """The «спой песенку» rule's primary action must be
        ``generate_music`` — not ``execute_music_code``.

        The DISPATCH block may MENTION ``execute_music_code`` in the
        new rule (e.g. the warning «Renardo has no vocals — never pick
        execute_music_code…») but the tool that the arrow `→` points
        to must be ``generate_music``. We verify this by checking that
        ``generate_music`` appears BEFORE ``execute_music_code`` in
        the same window around the marker.
        """
        dispatch = _dispatch_section()
        dispatch_l = dispatch.lower()
        for marker in ("спой песенку", "частушку"):
            pos = dispatch_l.find(marker)
            assert pos != -1, (
                f"«{marker}» is not in DISPATCH at all — see "
                f"test_dispatch_routes_sing_song_to_generate_music"
            )
            window = dispatch_l[pos:pos + 320]
            gen_pos = window.find("generate_music")
            assert gen_pos != -1, (
                f"«{marker}» rule has no ``generate_music`` target "
                f"(window={window!r})"
            )
            # If execute_music_code is also in the window (allowed —
            # the warning text mentions it), it must come AFTER the
            # `→ generate_music()` arrow. We don't enforce strict
            # ordering because the warning can be reworded freely;
            # we just require the rule to clearly point at
            # ``generate_music`` first.
            exec_pos = window.find("execute_music_code")
            if exec_pos != -1:
                assert gen_pos < exec_pos or "never" in window, (
                    f"«{marker}» rule has ``generate_music`` AFTER "
                    f"``execute_music_code`` — the LLM will be "
                    f"confused. Window: {window!r}"
                )

    def test_dispatch_lists_sing_about_cat_phrase(self) -> None:
        """The exact failing phrase «спой песенку про котика» must
        appear in DISPATCH as a concrete example.

        The e2e run surfaced this exact user utterance. Putting it in
        the prompt as a worked example gives the LLM an unambiguous
        routing anchor — it sees the resolved answer before having to
        infer one.
        """
        dispatch = _dispatch_section().lower()
        assert "спой песенку про котика" in dispatch, (
            "the failed e2e phrase «спой песенку про котика» must "
            "appear in DISPATCH as a concrete routing example"
        )

    def test_issue_1403_warning_block_present(self) -> None:
        """Fingerprint: the prompt must contain the «🚨 Issue #1403»
        warning block. Future cleanups that drop the block will fail
        this test and force a review of the regression.
        """
        text = _prompt_text()
        assert "🚨 Issue #1403" in text, (
            "explicit «Issue #1403» warning block must be present in "
            "the prompt — this is a regression fingerprint"
        )
        block_idx = text.find("🚨 Issue #1403")
        block = text[block_idx:block_idx + 1200]
        assert "generate_music" in block, (
            "«Issue #1403» warning must explicitly call out "
            "``generate_music`` as the correct tool"
        )
        assert "execute_music_code" in block, (
            "«Issue #1403» warning must explicitly call out "
            "``execute_music_code`` as the WRONG tool the LLM chose"
        )

    def test_prompt_explains_renardo_has_no_vocals(self) -> None:
        """The prompt must explicitly tell the LLM that Renardo is a
        SYNTHESIZER without vocals.

        This is the conceptual reason «спой» can never mean
        ``execute_music_code``. Without this warning the LLM does not
        understand WHY it must pick ``generate_music`` for singing
        requests and falls back to Renardo for "music-shaped" prompts.
        """
        text = _prompt_text()
        text_l = text.lower()
        # Either the explicit «Issue #1403» block OR the existing
        # «Vocals / voice — Renardo has no vocals» bullet in the
        # «WHEN TO USE generate_music vs Renardo» section must be
        # present.
        has_1403 = "🚨 issue #1403" in text_l
        has_no_vocals = (
            "renardo has no vocals" in text_l
            or ("вокал" in text_l and "renardo" in text_l)
        )
        assert has_1403 or has_no_vocals, (
            "prompt must explain that Renardo has no vocals, either "
            "via the explicit «Issue #1403» block or the existing "
            "«Vocals / voice — Renardo has no vocals» guidance"
        )

    @pytest.mark.parametrize(
        "phrase",
        [
            "спой песенку про котика",
            "спой частушку про лето",
            "спой строчку про дождь",
        ],
    )
    def test_dispatch_lists_concrete_sing_examples(self, phrase: str) -> None:
        """Several «спой X про Y» variants must be reachable in DISPATCH
        as concrete routing examples.

        One failed phrase (cat) is not enough — the LLM will see
        other «спой X про Y» requests in production. Parametrize the
        most common variants so a future prompt edit can't accidentally
        leave any of them ambiguous.

        Only the worked-example phrases are checked here. The catch-all
        pattern «спой песенку/песню/частушку/куплет/строчку про X» is
        covered by ``test_dispatch_routes_sing_song_to_generate_music``.
        """
        dispatch = _dispatch_section().lower()
        pos = dispatch.find(phrase)
        assert pos != -1, (
            f"«{phrase}» not in DISPATCH — LLM will likely fall "
            f"through to Renardo (issue #1403 regression)"
        )
        window = dispatch[pos:pos + 320]
        assert "generate_music" in window, (
            f"«{phrase}» found in DISPATCH but ``generate_music`` "
            f"is not in the surrounding window ({window!r})"
        )
