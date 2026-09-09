"""Unit tests for FoxDot / SuperCollider music stack validation helpers."""

import os
from pathlib import Path

import pytest

from rob_box_voice.core.music_stack_validation import (
    classify_sclang_log,
    contains_merge_conflict_markers,
    format_music_stack_report,
    is_plugin_dependent_synthdef,
    load_sclang_health,
)
from rob_box_voice.core.renardo_synthdef_patches import (
    patch_organ_scd_content,
    patch_brass_scd_content,
    patch_tb303_scd_content,
    apply_renardo_synthdef_patches,
    resolve_conflicted_scd_content,
)


def test_contains_merge_conflict_markers_detects_conflict_blocks():
    broken_scd = """<<<<<<< HEAD.
SynthDef.new(\\organ, {
=======
SynthDef.new(\\organ, {
>>>>>>> branch
"""

    assert contains_merge_conflict_markers(broken_scd) is True


def test_contains_merge_conflict_markers_ignores_clean_content():
    clean_scd = "SynthDef.new(\\strings, { ReplaceOut.ar(bus, osc) }).add;\n"

    assert contains_merge_conflict_markers(clean_scd) is False


def test_contains_merge_conflict_markers_ignores_plain_separator_text():
    content = "// ======= frequency divider =======\nSynthDef.new(\\pad, { ReplaceOut.ar(bus, osc) }).add;\n"

    assert contains_merge_conflict_markers(content) is False


@pytest.mark.parametrize(
    ("synthdef_source", "expected"),
    [
        (
            "osc = MoogVCF.ar(in: Pulse.ar(freq), fco: 1200, res: 0.4, mul: 1);",
            True,
        ),
        (
            "osc = SyncSaw.ar(syncFreq: freq, sawFreq: freq * 1.5, mul: 0.4);",
            False,
        ),
    ],
)
def test_is_plugin_dependent_synthdef_detects_extension_ugens(synthdef_source: str, expected: bool):
    assert is_plugin_dependent_synthdef(synthdef_source) is expected


def test_classify_sclang_log_reports_healthy_runtime():
    log_text = """
FoxDot OSCdef registered. Ready to compile SynthDefs.
Server running: true
SynthDef preload ok: strings
SynthDef preload ok: wobblebass
"""

    result = classify_sclang_log(log_text, critical_synths=["strings", "wobblebass"])

    assert result.is_healthy is True
    assert result.missing_synths == ()
    assert result.fatal_errors == ()


def test_classify_sclang_log_requires_positive_confirmation_for_each_critical_synth():
    log_text = """
FoxDot OSCdef registered. Ready to compile SynthDefs.
Server running: true
SynthDef preload ok: strings
"""

    result = classify_sclang_log(log_text, critical_synths=["strings", "wobblebass"])

    assert result.is_healthy is False
    assert result.missing_synths == ("wobblebass",)


def test_classify_sclang_log_accepts_variant_readiness_phrase():
    log_text = """
foxdot oscdef ready
Server running: true
"""

    result = classify_sclang_log(log_text, critical_synths=[])

    assert result.is_healthy is True
    assert result.oscdef_registered is True


def test_classify_sclang_log_reports_degraded_runtime():
    log_text = """
FoxDot OSCdef registered. Ready to compile SynthDefs.
ERROR: syntax error, unexpected BINOP, expecting $end
*** ERROR: SynthDef strings not found
"""

    result = classify_sclang_log(log_text, critical_synths=["strings", "wobblebass"])

    assert result.is_healthy is False
    assert "strings" in result.missing_synths
    assert any("syntax error" in error for error in result.fatal_errors)


def test_format_music_stack_report_for_healthy_runtime():
    status = classify_sclang_log(
        "Booting\nFoxDot OSCdef ready\nSynthDef preload ok: strings\n",
        critical_synths=["strings"],
    )

    report = format_music_stack_report(status)

    assert "Music stack healthy" in report
    assert "OSCdef ready: yes" in report
    assert "Missing critical SynthDefs: none" in report


def test_format_music_stack_report_for_degraded_runtime():
    status = classify_sclang_log(
        "FoxDot OSCdef ready\nERROR: SynthDef strings not found\nERROR: syntax error, unexpected BINOP\n",
        critical_synths=["strings", "wobblebass"],
    )

    report = format_music_stack_report(status)

    assert "Music stack degraded" in report
    assert "Missing critical SynthDefs: strings" in report
    assert "Fatal errors:" in report


def test_resolve_conflicted_scd_content_keeps_bottom_version_for_organ_style_conflicts():
    conflicted = """<<<<<<< HEAD:renardo_lib/renardo_lib/osc/scsyndef/organ.scd.
SynthDef(\\organ,
    {|f=440|\nold body\n}).add;
=======
SynthDef.new(\\organ, {
    |f=440|\nnew body\n},
metadata: (category: \\organ)
).add;
>>>>>>> badc8940:FoxDot/osc/scsyndef/organ.scd
"""

    resolved = resolve_conflicted_scd_content(conflicted)

    assert "<<<<<<<" not in resolved
    assert "=======" not in resolved
    assert ">>>>>>>" not in resolved
    assert "SynthDef.new(\\organ, {" in resolved
    assert "metadata: (category: \\organ)" in resolved
    assert "old body" not in resolved


def test_patch_brass_scd_content_replaces_broken_conflicted_source_with_known_good_version():
    conflicted = """<<<<<<< HEAD.
SynthDef(\\brass, { old body }).add;
=======
SynthDef.new(\\brass, { broken new body }).add;
>>>>>>> branch
"""

    patched = patch_brass_scd_content(conflicted)

    assert "<<<<<<<" not in patched
    assert "Resonz.ar" in patched
    assert "Env.perc(atk, sus, amp, 0)" in patched
    assert "ReplaceOut.ar(bus, osc)" in patched


def test_patch_organ_scd_content_replaces_upstream_source_with_stable_organ_version():
    source = """SynthDef.new(\\organ, {.
    |f=440|
    old body
},
metadata: (category: \\organ)
).add;
"""

    patched = patch_organ_scd_content(source)

    assert "LeakDC.ar" in patched
    assert "Env.asr" in patched
    assert "Lag.kr" in patched
    assert "HPF.ar" in patched
    assert "ReplaceOut.ar(bus, osc)" in patched


def test_patch_tb303_scd_content_replaces_upstream_source_with_stable_anti_click_version():
    source = """SynthDef.new(\\tb303, {.
    |atk=0.1, sus=0, dec=1|
    volEnv = EnvGen.ar(Env.new([10e-10, 1, 1, 10e-10], [0.01, sus, dec], 'exp'));
    filEnv = EnvGen.ar(Env.new([10e-10, 1, 10e-10], [0.01, dec], 'exp'));
}).add;
"""

    patched = patch_tb303_scd_content(source)

    assert "LeakDC.ar" in patched
    assert "HPF.ar" in patched
    assert "atk.max(0.02)" in patched
    assert "Lag.kr" in patched
    assert "RLPF.ar" in patched
    assert "0.01, sus, dec" not in patched

def test_apply_renardo_synthdef_patches_patches_tb303_file_in_place(tmp_path):
    tb303_file = tmp_path / "tb303.scd"
    tb303_file.write_text(
        """SynthDef.new(\\tb303, {.
    |atk=0.1, sus=0, dec=1|
    volEnv = EnvGen.ar(Env.new([10e-10, 1, 1, 10e-10], [0.01, sus, dec], 'exp'));
    filEnv = EnvGen.ar(Env.new([10e-10, 1, 10e-10], [0.01, dec], 'exp'));
}).add;
""",
        encoding="utf-8",
    )

    patched_files = apply_renardo_synthdef_patches(tmp_path)
    patched = tb303_file.read_text(encoding="utf-8")

    assert patched_files == ["tb303.scd"]
    assert "LeakDC.ar" in patched
    assert "atk.max(0.02)" in patched


# ---------------------------------------------------------------------------
# load_sclang_health — filesystem-backed helper (issue G-MUSIC)
# ---------------------------------------------------------------------------


def test_load_sclang_health_returns_unhealthy_when_log_missing(tmp_path, monkeypatch):
    """Missing log → unhealthy with the path in fatal_errors (not silent)."""

    log_path = tmp_path / "absent.log"
    monkeypatch.setenv("SCLANG_LOG_PATH", str(log_path))

    status = load_sclang_health()

    assert status.is_healthy is False
    assert status.oscdef_registered is False
    assert status.fatal_errors
    assert any(str(log_path) in err for err in status.fatal_errors)


def test_load_sclang_health_returns_unhealthy_for_degraded_log(tmp_path, monkeypatch):
    """Real-world failure mode: log file exists but contains syntax errors."""

    log_path = tmp_path / "sclang.log"
    log_path.write_text(
        "\n".join([
            "Booting sclang...",
            "FoxDot OSCdef registered. Ready to compile SynthDefs.",
            "ERROR: syntax error, unexpected '.', expecting '}'",
            "ERROR: Command line parse failed",
            "",
        ]),
        encoding="utf-8",
    )
    monkeypatch.setenv("SCLANG_LOG_PATH", str(log_path))

    status = load_sclang_health(critical_synths=["strings"])

    assert status.is_healthy is False
    assert status.oscdef_registered is True
    assert any("syntax error" in err for err in status.fatal_errors)
    assert "strings" in status.missing_synths


def test_load_sclang_health_returns_healthy_when_log_clean(tmp_path, monkeypatch):
    """All critical synths preloaded, no fatal errors → healthy."""

    log_path = tmp_path / "sclang.log"
    log_path.write_text(
        "\n".join([
            "Booting sclang...",
            "FoxDot OSCdef registered. Ready to compile SynthDefs.",
            "SynthDef preload ok: strings",
            "SynthDef preload ok: wobblebass",
            "SynthDef preload ok: warmpad",
            "",
        ]),
        encoding="utf-8",
    )
    monkeypatch.setenv("SCLANG_LOG_PATH", str(log_path))

    status = load_sclang_health(
        critical_synths=["strings", "wobblebass", "warmpad"],
    )

    assert status.is_healthy is True
    assert status.oscdef_registered is True
    assert status.missing_synths == ()
    assert status.fatal_errors == ()


def test_load_sclang_health_explicit_log_path_overrides_env(tmp_path, monkeypatch):
    """``log_path`` arg wins over SCLANG_LOG_PATH env."""

    env_log = tmp_path / "env.log"
    explicit_log = tmp_path / "explicit.log"
    env_log.write_text("ERROR: syntax error, unexpected BINOP", encoding="utf-8")
    explicit_log.write_text(
        "FoxDot OSCdef registered. Ready to compile SynthDefs.",
        encoding="utf-8",
    )
    monkeypatch.setenv("SCLANG_LOG_PATH", str(env_log))

    status = load_sclang_health(log_path=explicit_log)

    assert status.is_healthy is True
