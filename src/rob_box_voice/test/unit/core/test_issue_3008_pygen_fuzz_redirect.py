"""Issue #3008 regression (27.09): the fuzz patch never reached scsynth.

``fuzz`` in upstream renardo_lib is a Python-generated synth
(``DefaultPygenSynthDef`` in
``runtime/synthdefs_initialisation/python_defined_synthdefs.py``), not a file
one. Its ``add()`` rewrites ``sclang_code/tmp_code/scsynth/fuzz.scd`` with the
original ``LFSaw.ar(LFSaw.kr(...))`` code on every music start and sends that
path to sclang via ``/foxdot`` — so the upstream def replaced the build-time
patch of ``sclang_code/scsynth/fuzz.scd`` in scsynth. foxdot_init.sc must
redirect such ``/foxdot`` paths to the patched file.
"""

from __future__ import annotations

import os
import re
from pathlib import Path

from rob_box_voice.core.renardo_synthdef_patches import (
    FUZZ_SYNTHDEF,
    PYGEN_PATCHED_SYNTHS,
)


def _repo_root(start: Path) -> Path:
    override = os.environ.get("ROB_BOX_REPO_ROOT")
    if override and (Path(override) / "src").is_dir():
        return Path(override).resolve()
    for parent in [start, *start.parents]:
        if (parent / "src").is_dir() and (parent / "docker").is_dir():
            return parent
    raise RuntimeError(f"repo root not found for {start!s}; set ROB_BOX_REPO_ROOT")


FOXDOT_INIT = (
    _repo_root(Path(__file__).resolve())
    / "docker" / "vision" / "voice_assistant" / "foxdot_init.sc"
)


def _code() -> str:
    """foxdot_init.sc without full-line comments."""
    text = FOXDOT_INIT.read_text(encoding="utf-8")
    return "\n".join(
        line for line in text.splitlines() if not line.lstrip().startswith("//")
    )


def test_fuzz_is_registered_as_pygen_patched():
    assert "fuzz" in PYGEN_PATCHED_SYNTHS


def test_foxdot_init_list_matches_python_list():
    match = re.search(r"var pygenPatchedSynths = \[([^\]]*)\];", _code())
    assert match, "foxdot_init.sc must declare pygenPatchedSynths"
    names = tuple(re.findall(r'"([^"]+)"', match.group(1)))
    assert names == PYGEN_PATCHED_SYNTHS


def test_foxdot_oscdef_loads_redirected_path():
    code = _code()
    oscdef = code[code.index("\\foxdot,"):code.index("'/foxdot'")]
    assert "redirectPatchedPygenPath.value(msg[1].asString)" in oscdef
    # The redirected path — not the raw tmp_code one — is what #1807 restore
    # remembers and what gets loaded.
    assert "loadedSynthPaths.add(path)" in oscdef
    assert "path.load" in oscdef


def test_redirect_targets_patched_dir_only_for_tmp_code_paths():
    code = _code()
    start = code.index("var redirectPatchedPygenPath")
    body = code[start:code.index("var reloadKnownPath", start)]
    assert '{ path.contains("tmp_code") }' in body
    assert "pygenPatchedSynths.includesEqual(name)" in body
    assert 'renardoSynthDir ++ "/" ++ name ++ ".scd"' in body
    # Never redirect while RENARDO_SCLANG_DIR is still the placeholder
    # (same guard as the startup preload; `.exists` is banned in this file by
    # test_music_runtime_assets.py).
    assert "renardoSynthDir != renardoSynthDirPlaceholder" in body


def test_fuzz_modulator_is_audio_rate():
    # scsynth runs with -z 1024 → control rate 16000/1024 ≈ 15.6 Hz, below the
    # modulator frequency (freq/2 = 20..40 Hz for bass notes). A .kr modulator
    # makes the carrier pitch jump every 64 ms.
    assert "LFSaw.kr" not in FUZZ_SYNTHDEF
    assert "Saw.ar(LFSaw.ar(freq, 0, freq, (freq * 2)))" in FUZZ_SYNTHDEF
