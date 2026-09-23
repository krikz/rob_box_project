"""test_ensure_dj_music_response.py — Unit tests for the DJ fallback helper (issue #2557).

Regression for the live 15.09.2026 incident: the LLM returned ``spoken='done'``
AFTER calling music tools (``compose_music``, ``execute_music_code``,
``set_dj_mode`` …) WITHOUT ``speak_text`` — the user heard music start/stop
and then pure silence (no audible acknowledgement). 22 occurrences/hour on the
DJ live test (Vision Pi 10.1.1.21, round 3 — ×4.4 vs round 2's 5/h).

The cycle-end equality check in ``dialogue_node._handle_result`` correctly
suppressed ``done`` from auto-TTS, but the user-facing info never reached
the speaker. The fix introduces ``ensure_dj_music_response`` (in
``rob_box_voice.core.speak_helpers``) which recognises the «tools called but
no real speech» shape and returns a short DJ-appropriate phrase
(«Готово, играю.») instead. This helper is the SINGLE source of truth so
``_handle_result`` stays under the CC budget.

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/core/test_ensure_dj_music_response.py
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.speak_helpers import (
    _DJ_DEGENERATE_MARKERS,
    _DJ_FALLBACK_PHRASE,
    _DJ_MUSIC_TOOLS,
    ensure_dj_music_response,
)


# Baseline: the constants we depend on must match the spec in issue #2557.
class TestConstants:
    """Pin the helper's contract so a rename of tools / markers is caught."""

    def test_fallback_phrase_is_dj_style(self) -> None:
        # «Готово, играю.» — matches the existing DJ hooks («Принял.», «Понял.»).
        # If this changes, the live test will drift from the docs in
        # ``speak_helpers`` and the e2e voice command may regress.
        assert _DJ_FALLBACK_PHRASE == "Готово, играю."

    def test_music_tools_cover_all_dj_live_cases(self) -> None:
        # The 10 distinct tool combinations logged live 15.09.2026:
        #   compose_music + set_dj_mode (without speak_text) ← target
        #   execute_music_code + speak_text
        #   gen_search_library + set_dj_mode + execute_music_code + speak_text
        #   load_skill + lookup_melody + compose_music + speak_text
        # Every music tool from those shapes must be in the set.
        for name in (
            "compose_music", "execute_music_code", "set_dj_mode",
            "lookup_melody", "load_skill", "set_vibe_preset",
            "stop_music", "load_track", "generate_music",
            "gen_play_from_library",
        ):
            assert name in _DJ_MUSIC_TOOLS, (
                f"{name!r} missing from _DJ_MUSIC_TOOLS — "
                "live #2557 combos won't trigger the fallback"
            )

    def test_degenerate_markers_match_handle_result_check(self) -> None:
        # Keep in sync with dialogue_node._handle_result equality check —
        # otherwise the helper would recognise a marker that the dialogue
        # node doesn't, or vice versa, and the user would hear silence
        # after the helper passes through a marker the dialogue node
        # would have stripped.
        for marker in ("done", "task complete", "task_complete",
                       "готово", "всё", "выполнено"):
            assert marker in _DJ_DEGENERATE_MARKERS


# Empty / marker-only spoken + music tools → fallback fires.
class TestFallbackFires:
    """The bug case: tools_called contains a music tool AND spoken is empty
    or one of the cycle-end markers. The helper MUST return the fallback."""

    def test_empty_spoken_with_compose_music(self) -> None:
        # Live 15.09 log line #1 (verbatim):
        #   spoken='done'[:60] tools=['compose_music', 'set_dj_mode']
        # After strip_done_marker + equality check, spoken is empty.
        # ensure_dj_music_response sees (empty, [compose_music, set_dj_mode]).
        result = ensure_dj_music_response("", ["compose_music", "set_dj_mode"])
        assert result == _DJ_FALLBACK_PHRASE

    def test_done_marker_with_execute_music_code(self) -> None:
        # Live 15.09 log line #2: spoken='done', tools=['execute_music_code'].
        # Note: speak_text_real>0 (speak_text was also in tools_called), so
        # this case is normally short-circuited upstream, but the helper
        # must still return a fallback if it ever sees it (defensive).
        result = ensure_dj_music_response(
            "done", ["execute_music_code"]
        )
        assert result == _DJ_FALLBACK_PHRASE

    def test_double_newline_done_with_set_dj_mode(self) -> None:
        # Spoken that survived strip_done_marker (post-strip would be
        # empty in real life, but the helper accepts the raw form for
        # robustness). Tools: set_dj_mode + compose_music.
        result = ensure_dj_music_response(
            "\n\ndone", ["set_dj_mode", "compose_music"]
        )
        assert result == _DJ_FALLBACK_PHRASE

    def test_whitespace_only_spoken(self) -> None:
        # Edge case: strip chain left whitespace; tools include compose_music.
        result = ensure_dj_music_response(
            "   \n  ", ["compose_music"]
        )
        assert result == _DJ_FALLBACK_PHRASE

    @pytest.mark.parametrize("marker", [
        "done", "Done", "DONE",
        "готово", "Готово", "ГОТОВО",
        "всё", "выполнено", "task complete", "task_complete",
    ])
    def test_degenerate_marker_variants_with_load_track(
        self, marker: str,
    ) -> None:
        # ALL marker variants must trigger the fallback — LLM capitalises
        # inconsistently and the master-prompt contract allows both
        # English and Russian forms.
        result = ensure_dj_music_response(
            marker, ["load_track"]
        )
        assert result == _DJ_FALLBACK_PHRASE

    def test_lookup_melody_alone_triggers_fallback(self) -> None:
        # Edge case: lookup_melody is in the music set; alone it should
        # still trigger the fallback (it's a music state lookup the
        # user would want to know about).
        result = ensure_dj_music_response("", ["lookup_melody"])
        assert result == _DJ_FALLBACK_PHRASE

    def test_load_skill_alone_triggers_fallback(self) -> None:
        # Edge case: load_skill loads a music-related skill — same
        # reasoning as lookup_melody.
        result = ensure_dj_music_response("", ["load_skill"])
        assert result == _DJ_FALLBACK_PHRASE


# Real user-facing reply + music tools → fallback DOES NOT fire.
class TestFallbackDoesNotFire:
    """When the model followed the contract (called music tools AND spoke),
    the helper MUST pass the original spoken through unchanged."""

    def test_real_reply_passes_through(self) -> None:
        # Live expected case: model called execute_music_code AND wrote
        # «Запускаю Баха!» → that's a real reply, leave it.
        spoken = "Запускаю Баха."
        result = ensure_dj_music_response(
            spoken, ["execute_music_code"]
        )
        assert result == spoken

    def test_real_reply_with_multiple_music_tools(self) -> None:
        spoken = "Переключаюсь на новый трек."
        result = ensure_dj_music_response(
            spoken, ["set_dj_mode", "execute_music_code"]
        )
        assert result == spoken

    def test_real_reply_with_set_vibe_preset(self) -> None:
        spoken = "Ставлю спокойный вайб."
        result = ensure_dj_music_response(
            spoken, ["set_vibe_preset"]
        )
        assert result == spoken


# No music tools → fallback never fires (regardless of spoken).
class TestNoMusicTools:
    """When the LLM didn't touch any music tool, the helper MUST be a
    pass-through — non-music flows (conversation, web search, etc.) must
    not be polluted with a DJ announcement."""

    def test_empty_tools_no_fallback(self) -> None:
        # Common case: spoken is the model's reply, no tools at all.
        spoken = "Привет, как дела?"
        result = ensure_dj_music_response(spoken, [])
        assert result == spoken

    def test_non_music_tools_no_fallback(self) -> None:
        # speak_text only — user hears the speak_text reply upstream;
        # no DJ announcement needed here.
        spoken = ""
        result = ensure_dj_music_response(spoken, ["speak_text"])
        assert result == spoken

    def test_web_search_no_fallback(self) -> None:
        # Non-music tools must not trigger the DJ phrase.
        spoken = ""
        result = ensure_dj_music_response(
            spoken, ["web_search", "fetch_url"]
        )
        assert result == spoken

    def test_none_tools_no_fallback(self) -> None:
        # Defensive: tools_called is None (e.g. some malformed
        # DialogResult). Must not crash and must not fire fallback.
        result = ensure_dj_music_response("Hello world", None)
        assert result == "Hello world"


# Existing acceptance criterion #1: every music-tools turn contains speak_text.
# This is the upstream contract — the helper is the LAST line of defence,
# but the master-prompt + bug-C guard usually suppress ``done`` upstream.
class TestAcceptanceMirrorsLiveLog:
    """Pin the live-log cases from issue #2557 so a future refactor of
    the helper is forced to re-verify the bug doesn't regress."""

    @pytest.mark.parametrize("tools,spoken", [
        # 3 of 10 live combinations were without speak_text — these
        # are exactly the cases that triggered the bug.
        (["compose_music", "set_dj_mode"], ""),
        (["compose_music", "set_dj_mode"], "done"),
        (["compose_music", "set_dj_mode"], "\n\ndone"),
        (["execute_music_code"], ""),
        (["set_dj_mode", "lookup_melody"], "готово"),
    ])
    def test_live_log_shape_returns_fallback(
        self, tools, spoken,
    ) -> None:
        assert ensure_dj_music_response(spoken, tools) == _DJ_FALLBACK_PHRASE

    @pytest.mark.parametrize("tools,spoken", [
        # The 7 of 10 live combinations that HAD speak_text — the helper
        # would never see these (speak_text_real>0 returns upstream),
        # but if it does, it must pass through.
        (["execute_music_code", "speak_text"], "Готово."),
        (["speak_text", "compose_music", "set_dj_mode"], "Запускаю."),
        (["load_skill", "lookup_melody", "compose_music", "speak_text"],
         "Делаю."),
        (["load_skill", "get_music_state", "execute_music_code", "speak_text"],
         "Ок."),
        (["gen_search_library", "set_dj_mode", "execute_music_code",
          "speak_text"], "Сейчас."),
    ])
    def test_live_log_shape_with_real_speech_passes_through(
        self, tools, spoken,
    ) -> None:
        assert ensure_dj_music_response(spoken, tools) == spoken


# Issue #2857 — speak_text already voiced a real line this turn; the
# fallback must never stomp it, even though the post-strip ``spoken``
# field looks empty/degenerate (the LLM's cycle-end contract).
class TestSpeakTextAlreadySpoke:
    """Live 23.09.2026 incident: speak_text voiced 'Йоу, народ,
    гангста-драйв качает!', but the helper still overwrote it with
    'Готово, играю.' because it only looked at ``spoken``, not whether
    speak_text ran this turn."""

    def test_speak_text_with_empty_spoken_and_music_tools(self) -> None:
        result = ensure_dj_music_response(
            "", ["speak_text", "compose_music", "set_dj_mode"],
        )
        assert result == ""

    def test_speak_text_with_done_marker_and_music_tools(self) -> None:
        result = ensure_dj_music_response(
            "done", ["speak_text", "execute_music_code"],
        )
        assert result == "done"

    def test_speak_text_with_degenerate_marker_and_dj_auto(self) -> None:
        # Even on a DJ auto-transition (where the fallback would
        # otherwise try to build a track announcement), speak_text
        # having run wins — nothing to add.
        result = ensure_dj_music_response(
            "готово", ["speak_text", "compose_music"],
            is_dj_auto=True, track_name="Дюна",
        )
        assert result == "готово"


# Issue #2857 — on a DJ auto-transition, replace the dull generic
# phrase with a short track-specific line, or stay silent.
class TestDjAutoTrackAnnouncement:
    """Acceptance criteria from issue #2857: DJ-переход без реплики →
    короткая фраза про трек, а не «Готово, играю.»; без данных —
    тишина; юзер-реквест сохраняет старую фразу."""

    def test_dj_auto_with_track_name_mentions_track(self) -> None:
        result = ensure_dj_music_response(
            "", ["compose_music", "set_dj_mode"],
            is_dj_auto=True, track_name="Гангста-драйв",
        )
        assert "Гангста-драйв" in result
        assert result != _DJ_FALLBACK_PHRASE

    def test_dj_auto_with_track_name_and_persona(self) -> None:
        result = ensure_dj_music_response(
            "done", ["compose_music"],
            is_dj_auto=True, track_name="Дюна", persona="ДиДжей Роббокс",
        )
        assert result == "ДиДжей Роббокс: дальше — Дюна!"

    def test_dj_auto_falls_back_to_theme_without_track_name(self) -> None:
        result = ensure_dj_music_response(
            "", ["compose_music"], is_dj_auto=True, theme="ретро-вечеринка",
        )
        assert "ретро-вечеринка" in result
        assert result != _DJ_FALLBACK_PHRASE

    def test_dj_auto_without_any_info_stays_silent(self) -> None:
        result = ensure_dj_music_response(
            "", ["compose_music"], is_dj_auto=True,
        )
        assert result == ""
        assert result != _DJ_FALLBACK_PHRASE

    def test_dj_auto_without_info_and_done_marker_stays_silent(self) -> None:
        result = ensure_dj_music_response(
            "done", ["set_dj_mode", "lookup_melody"], is_dj_auto=True,
        )
        assert result == ""

    def test_direct_user_request_keeps_generic_fallback_phrase(self) -> None:
        # is_dj_auto=False (default) — the user asked directly and got
        # no reply text; the generic confirmation is still correct.
        result = ensure_dj_music_response(
            "", ["compose_music", "set_dj_mode"], is_dj_auto=False,
        )
        assert result == _DJ_FALLBACK_PHRASE

    def test_direct_user_request_with_track_name_still_uses_phrase(
        self,
    ) -> None:
        # Track info being available doesn't matter off the DJ-auto
        # path — a direct request always gets the generic phrase.
        result = ensure_dj_music_response(
            "", ["compose_music"], is_dj_auto=False, track_name="Дюна",
        )
        assert result == _DJ_FALLBACK_PHRASE


# Defensive: types the helper must accept without crashing.
class TestDefensiveInputs:
    """Defensive contract — non-string spoken must pass through, similar
    to strip_done_marker / strip_markdown in this module."""

    def test_none_spoken_passes_through(self) -> None:
        # Pathological: a DialogResult with spoken_text=None. The
        # caller (dialogue_node) handles ``or ""`` upstream, but the
        # helper itself must not crash on None.
        assert ensure_dj_music_response(None, ["compose_music"]) is None  # type: ignore[arg-type]

    def test_int_spoken_passes_through(self) -> None:
        assert ensure_dj_music_response(42, ["compose_music"]) == 42  # type: ignore[arg-type]

    def test_list_spoken_passes_through(self) -> None:
        spoken = ["done"]
        assert ensure_dj_music_response(spoken, ["compose_music"]) == spoken  # type: ignore[arg-type]