"""Unit tests for ``rob_box_voice.core.sample_search``."""

from __future__ import annotations

import tempfile
from pathlib import Path

import pytest

from rob_box_voice.core.sample_search import (
    MAX_RESULTS,
    search_renardo_samples,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_samples_tree(base: Path, pack: str = "0_foxdot_default") -> Path:
    """Create a minimal fake sample-pack tree under *base*.

    Returns the pack directory so the caller can write realistic files.
    """
    pack_dir = base / pack
    for letter in ("a", "b", "_", "_loop_"):
        for case_dir in ("lower", "upper"):
            (pack_dir / letter / case_dir).mkdir(parents=True)
    return pack_dir


def _touch(path: Path) -> None:
    """Create an empty file (and parent dirs)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("")


# ---------------------------------------------------------------------------
# Error paths
# ---------------------------------------------------------------------------


def test_missing_root_returns_error() -> None:
    result = search_renardo_samples(Path("/nonexistent/dir"), "kick")
    assert "error" in result
    assert "not found" in result["error"].lower()
    assert "hint" in result


def test_missing_pack_returns_error_with_available_list() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        (root / "0_foxdot_default").mkdir()
        result = search_renardo_samples(root, "kick", pack="nonexistent_pack")
        assert "error" in result
        assert "nonexistent_pack" in result["error"]
        assert "available_packs" in result
        assert "0_foxdot_default" in result["available_packs"]


# ---------------------------------------------------------------------------
# Overview mode (query="*")
# ---------------------------------------------------------------------------


def test_overview_returns_letters_and_counts() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        pack_dir = _make_samples_tree(root)
        _touch(pack_dir / "a" / "lower" / "kick1.wav")
        _touch(pack_dir / "a" / "lower" / "kick2.wav")
        _touch(pack_dir / "b" / "lower" / "snare1.wav")

        result = search_renardo_samples(root, "*")

        assert "letters" in result
        assert "error" not in result
        letters = result["letters"]
        assert letters["a"] == 2
        assert letters["b"] == 1
        assert result["total_samples"] == 3


def test_overview_respects_case() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        pack_dir = _make_samples_tree(root)
        _touch(pack_dir / "a" / "upper" / "KICK1.WAV")
        _touch(pack_dir / "b" / "lower" / "snare1.wav")

        result_upper = search_renardo_samples(root, "*", case="upper")
        assert result_upper["letters"]["a"] == 1
        assert "b" not in result_upper["letters"]

        result_lower = search_renardo_samples(root, "*", case="lower")
        assert result_lower["letters"]["b"] == 1
        assert "a" not in result_lower["letters"]


def test_overview_skips_dot_dirs_and_non_dirs() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        pack_dir = _make_samples_tree(root)
        (pack_dir / ".hidden").mkdir()
        _touch(pack_dir / ".hidden" / "lower" / "secret.wav")
        _touch(pack_dir / "README.txt")

        result = search_renardo_samples(root, "*")
        assert ".hidden" not in result["letters"]


# ---------------------------------------------------------------------------
# Keyword search
# ---------------------------------------------------------------------------


def test_keyword_search_finds_matching_files() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        pack_dir = _make_samples_tree(root)
        _touch(pack_dir / "a" / "lower" / "Kick1.wav")
        _touch(pack_dir / "a" / "lower" / "Kick_808.wav")
        _touch(pack_dir / "b" / "lower" / "Snare_rim.wav")

        result = search_renardo_samples(root, "kick")

        assert result["found"] == 2
        assert result["query"] == "kick"
        results = result["results"]
        assert all(r["letter"] == "a" for r in results)
        assert results[0]["sample_index"] == 0
        assert results[1]["sample_index"] == 1
        assert all("play_code" in r for r in results)
        assert 'd1 >> play("a", sample=0)' == results[0]["play_code"]


def test_keyword_search_respects_case_upper() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        pack_dir = _make_samples_tree(root)
        _touch(pack_dir / "x" / "upper" / "CRASH1.WAV")

        result = search_renardo_samples(root, "crash", case="upper")
        assert result["found"] == 1
        assert result["results"][0]["letter"] == "X"
        assert 'd1 >> play("X", sample=0)' == result["results"][0]["play_code"]


def test_keyword_search_spack_suffix_for_nonzero_pack() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        (root / "0_foxdot_default").mkdir()
        pack_dir = _make_samples_tree(root, pack="1_pitchglitch_samples")
        _touch(pack_dir / "c" / "lower" / "vocal1.wav")

        result = search_renardo_samples(root, "vocal", pack="1_pitchglitch_samples")
        assert result["found"] == 1
        play_code = result["results"][0]["play_code"]
        assert "spack=1" in play_code


def test_keyword_search_no_results_returns_empty() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        _make_samples_tree(root)
        result = search_renardo_samples(root, "nonexistent_keyword_xyz")
        assert result["found"] == 0
        assert "results" not in result


def test_keyword_search_caps_at_max_results() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        pack_dir = _make_samples_tree(root)
        # Create MAX_RESULTS + 5 files in the same letter folder
        for i in range(MAX_RESULTS + 5):
            _touch(pack_dir / "a" / "lower" / f"kick_{i:03d}.wav")

        result = search_renardo_samples(root, "kick")
        assert result["found"] == MAX_RESULTS
        assert len(result["results"]) == MAX_RESULTS


def test_keyword_search_matches_substring_in_filename() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        pack_dir = _make_samples_tree(root)
        _touch(pack_dir / "a" / "lower" / "drum_kick.wav")
        _touch(pack_dir / "b" / "lower" / "snare.wav")

        result = search_renardo_samples(root, "drum")
        assert result["found"] == 1
        assert result["results"][0]["filename"] == "drum_kick.wav"


def test_keyword_search_case_insensitive() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        pack_dir = _make_samples_tree(root)
        _touch(pack_dir / "a" / "lower" / "KICK_HARD.WAV")

        result = search_renardo_samples(root, "kick")
        assert result["found"] == 1


def test_keyword_search_only_audio_extensions() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        pack_dir = _make_samples_tree(root)
        _touch(pack_dir / "a" / "lower" / "kick.wav")
        _touch(pack_dir / "a" / "lower" / "kick.txt")
        _touch(pack_dir / "a" / "lower" / "kick.aiff")
        _touch(pack_dir / "a" / "lower" / "kick.mp3")

        result = search_renardo_samples(root, "kick")
        assert result["found"] == 3  # .wav + .aiff + .mp3, NOT .txt
