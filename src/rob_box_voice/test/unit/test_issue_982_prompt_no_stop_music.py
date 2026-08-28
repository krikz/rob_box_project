from pathlib import Path

"""Static regression guard for issue #982.

Issue #982: the LLM prompt regressed to the pre-v37 wording (commit
4faa74cb) that told the model to call ``stop_music()`` / ``Clock.future``
after a rap/poem/singing performance, cutting the beat off mid-phrase
(real e2e 2026-08-04 17:38: beat stopped at T+14 while 2 more TTS chunks
were still playing). The prompt was re-aligned with the scheduler
architecture (#968) in commit 3b0d7443 — LLM fire-and-forget, the system
owns the music lifecycle and stops music after ``tts_batch_complete``.

These tests pin the *wording* of the prompts so a future merge can never
silently roll the prompt back to the old stop_music/Clock.future model
again (root cause of #982: nobody re-checked the prompt after merges).

Run with::

    python3 -m pytest src/rob_box_voice/test/unit/test_issue_982_prompt_no_stop_music.py
"""


MASTER_PROMPT = (
    Path(__file__).resolve().parents[2]
    / "prompts"
    / "master_prompt_compact.txt"
)

COMPOSITOR_PROMPT = (
    Path(__file__).resolve().parents[2]
    / "prompts"
    / "compositor_prompt.txt"
)


def _read(prompt_path: Path) -> str:
    return prompt_path.read_text(encoding="utf-8")


# ── master_prompt_compact.txt ─────────────────────────────────────────


def test_master_prompt_has_no_old_stop_music_after_rap() -> None:
    """The pre-v37 instructions (stop_music at the end) must be gone."""
    content = _read(MASTER_PROMPT)
    assert "stop_music()` at the end" not in content
    assert "call `stop_music()`!" not in content
    assert "MANDATORY sequence: execute_music_code" not in content
    # Old v37 wording promised "the system will fade out music
    # automatically" — now the system stops music after tts_batch_complete.
    assert "the system will fade out music automatically" not in content


def test_master_prompt_forbids_llm_owned_stop_music() -> None:
    """The LLM must NEVER stop music itself after rap/poem/singing."""
    content = _read(MASTER_PROMPT)
    assert "NEVER call `stop_music()` yourself after rap/poem/singing" in content
    assert "tts_batch_complete" in content
    assert "the system manages music lifecycle" in content


def test_master_prompt_forbids_clock_future_music_stop() -> None:
    """The sneaky bypass — Clock.future inside execute_music_code — is banned."""
    content = _read(MASTER_PROMPT)
    assert (
        "NEVER use `Clock.future` or any timer to schedule music stop"
        in content
    )
    assert "no `stop_music()` after rap/poem/singing, no `Clock.future`" in content


def test_master_prompt_describes_fire_and_forget_model() -> None:
    """Prompt must describe the scheduler model: LLM conducts, system owns channels."""
    content = _read(MASTER_PROMPT)
    assert "FIRE-AND-FORGET MODEL" in content
    assert "The system owns the channels" in content
    # New music contract: segments safety net, duration_sec deprecated.
    assert "segments" in content
    assert "NEVER pass `duration_sec` (deprecated, ignored)" in content


def test_master_prompt_no_old_crutch_mechanisms() -> None:
    """Old crutch wording (3s-timer, debounce) must not reappear."""
    content = _read(MASTER_PROMPT)
    assert "3с-таймер" not in content
    assert "3s-timer" not in content
    assert "debounce" not in content


# ── compositor_prompt.txt ─────────────────────────────────────────────


def test_compositor_prompt_never_stops_music_after_rap() -> None:
    """Compositor must not call handle_music(stop all music) after a performance."""
    content = _read(COMPOSITOR_PROMPT)
    assert (
        "NEVER call `handle_music(\"stop all music\")` after rap/poem"
        in content
    )
    assert "tts_batch_complete" in content
    # The only allowed stop_music case is the explicit DJ-mode exit.
    assert "Stop DJ mode" in content
    assert "This is the ONLY case where stop_music is allowed" in content


def test_compositor_prompt_stop_music_only_on_explicit_request() -> None:
    """Compositor stops music only on an explicit user request (DJ off / stop)."""
    content = _read(COMPOSITOR_PROMPT)
    assert "ONLY on explicit user request" in content
    assert "after rap/poem/singing NEVER stop music yourself" in content
