"""Unit tests for FoxDot / SuperCollider music stack validation helpers."""

import os
from pathlib import Path

import pytest

from rob_box_voice.core.music_stack_validation import (
    classify_sclang_log,
    confirmed_synths_from_log,
    contains_merge_conflict_markers,
    load_confirmed_synths,
    format_music_stack_report,
    is_plugin_dependent_synthdef,
    load_sclang_health,
    missing_log_hint,
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
    # RAW-формат живого лога (foxdot_init.sc:179, с live-фикса 30.08) —
    # НЕ "SynthDef preload ok: X" (см. test_classify_sclang_log_accepts_legacy_preload_ok_format
    # для старого формата отдельно).
    log_text = """
FoxDot OSCdef registered. Ready to compile SynthDefs.
Server running: true
SynthDef in scsynth: strings
SynthDef in scsynth: wobblebass
SynthDef preload finished: 63 defs
"""

    result = classify_sclang_log(log_text, critical_synths=["strings", "wobblebass"])

    assert result.is_healthy is True
    assert result.missing_synths == ()
    assert result.fatal_errors == ()


def test_classify_sclang_log_requires_positive_confirmation_for_each_critical_synth():
    log_text = """
FoxDot OSCdef registered. Ready to compile SynthDefs.
Server running: true
SynthDef in scsynth: strings
"""

    result = classify_sclang_log(log_text, critical_synths=["strings", "wobblebass"])

    assert result.is_healthy is False
    assert result.missing_synths == ("wobblebass",)


def test_classify_sclang_log_accepts_legacy_preload_ok_format():
    """Обратная совместимость со старым (до live-фикса 30.08) форматом лога.

    Живых логов в этом формате на роботе уже нет, но regex дёшево держит
    оба варианта — держим тест, чтобы обратную совместимость никто не
    сломал следующей правкой не глядя.
    """
    log_text = """
FoxDot OSCdef registered. Ready to compile SynthDefs.
Server running: true
SynthDef preload ok: strings
SynthDef preload ok: wobblebass
"""

    result = classify_sclang_log(log_text, critical_synths=["strings", "wobblebass"])

    assert result.is_healthy is True
    assert result.missing_synths == ()


def test_classify_sclang_log_matches_actual_foxdot_init_log_format():
    """Регресс на дрейф между foxdot_init.sc и _LOADED_SYNTH_RE (issue 21.09.2026).

    RAW-инцидент на Vision Pi: .sc-файл сменил печатаемую строку с
    "SynthDef preload ok: X" на "SynthDef in scsynth: X" (live-фикс 30.08),
    а regex в music_stack_validation.py остался на старом тексте. Юнит-тесты
    были зелёными (гоняли фикстуры со старой строкой), а на роботе валидатор
    рапортовал ВСЕ критичные SynthDef-ы как missing, хотя все 63 были
    загружены и /tmp/sclang.log содержал ровно новую строку — grep -x
    "SynthDef in scsynth: <name>" nashёл все 11 "отсутствующих" на месте.

    Этот тест читает РЕАЛЬНЫЙ .sc-файл и падает, если кто-то поменяет
    формат postln-строки, не тронув эту фикстуру и regex одновременно.
    """
    sc_path = (
        Path(__file__).resolve().parents[5]
        / "docker"
        / "vision"
        / "voice_assistant"
        / "foxdot_init.sc"
    )
    sc_source = sc_path.read_text(encoding="utf-8")
    assert '"SynthDef in scsynth: " ++ name).postln' in sc_source, (
        "foxdot_init.sc сменил формат лога преload-подтверждения — обнови "
        "_LOADED_SYNTH_RE в music_stack_validation.py, а затем эту фикстуру."
    )

    log_text = "\n".join(
        [
            "FoxDot OSCdef registered. Ready to compile SynthDefs.",
            "SynthDef in scsynth: strings",
            "SynthDef in scsynth: wobblebass",
            "SynthDef preload finished: 63 defs",
        ]
    )

    result = classify_sclang_log(log_text, critical_synths=["strings", "wobblebass"])

    assert result.is_healthy is True
    assert result.missing_synths == ()


# Issue #2838 — RAW-фрагмент /tmp/sclang.log с Vision Pi (23.09.2026).
_ROBOT_SCLANG_TAIL = """\
FoxDot OSCdef registered. Ready to compile SynthDefs.
Server running: true
SynthDef in scsynth: ambi
SynthDef in scsynth: sinepad
SynthDef in scsynth: masterfilter
SynthDef preload finished: 63 defs
Master filter armed at tail of RootNode (node 999)
WARNING: SynthDef bassguitar too big for sending. Retrying via synthdef file
"""


def test_confirmed_synths_from_log_lists_only_server_confirmed_names():
    confirmed = confirmed_synths_from_log(_ROBOT_SCLANG_TAIL)
    assert confirmed == frozenset({"ambi", "sinepad", "masterfilter"})
    # 'sine' ни разу не подтверждён — на роботе scsynth отбил его 235 раз.
    assert "sine" not in confirmed
    # "too big ... Retrying" — не подтверждение прихода.
    assert "bassguitar" not in confirmed


def test_confirmed_synths_from_log_is_none_until_preload_finished():
    """Прелоад не дописан → список неполон → None, а не урезанная «истина»."""
    partial = "SynthDef in scsynth: ambi\nSynthDef in scsynth: arpy\n"
    assert confirmed_synths_from_log(partial) is None


def test_confirmed_synths_from_log_drops_names_reported_not_found():
    log_text = _ROBOT_SCLANG_TAIL + "*** ERROR: SynthDef sinepad not found\n"
    assert "sinepad" not in confirmed_synths_from_log(log_text)


def test_load_confirmed_synths_reads_file_and_none_when_absent(tmp_path):
    log = tmp_path / "sclang.log"
    log.write_text(_ROBOT_SCLANG_TAIL, encoding="utf-8")
    expected = frozenset({"ambi", "sinepad", "masterfilter"})
    assert load_confirmed_synths(log) == expected
    assert load_confirmed_synths(tmp_path / "absent.log") is None


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
        "Booting\nFoxDot OSCdef ready\nSynthDef in scsynth: strings\n",
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
            "SynthDef in scsynth: strings",
            "SynthDef in scsynth: wobblebass",
            "SynthDef in scsynth: warmpad",
            "SynthDef preload finished: 63 defs",
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


# ---------------------------------------------------------------------------
# missing_log_hint — issue #2716 regression
# ---------------------------------------------------------------------------
#
# RAW from the Vision Pi (22.09.2026): validate_music_stack.py reported
# "Missing critical SynthDefs: strings, wobblebass, ..." (11 names) AND
# "Log file not found: /tmp/sclang.log" in the SAME run, while the report
# line right above both of them said "OSCdef ready: yes" — which can only
# be true if the log file existed and was read. The old condition in
# validate_music_stack.py::main() was `not status.is_healthy and not
# status.fatal_errors`, which is true for ANY degraded reason that isn't a
# fatal sclang syntax error — not just a genuinely missing file. These
# tests pin the corrected, file-existence-based condition.


def test_missing_log_hint_is_none_when_healthy():
    status = classify_sclang_log(
        "FoxDot OSCdef registered. Ready to compile SynthDefs.\n"
        "SynthDef in scsynth: strings\n"
        "SynthDef preload finished: 1 defs\n",
        critical_synths=["strings"],
    )

    assert missing_log_hint(status, "/tmp/sclang.log") is None


def test_missing_log_hint_is_none_when_log_file_exists_but_synths_missing(tmp_path):
    """The exact #2716 regression: file present + readable, just degraded.

    Must NOT claim the log is missing — that sent operators looking in the
    wrong place for three weeks (see the identical live incident fixed for
    the regex itself in afdabdd80 / issue history for #2716).
    """

    log_path = tmp_path / "sclang.log"
    log_path.write_text(
        "FoxDot OSCdef registered. Ready to compile SynthDefs.\n"
        "SynthDef preload finished: 0 defs\n",
        encoding="utf-8",
    )
    status = classify_sclang_log(log_path.read_text(encoding="utf-8"), critical_synths=["strings"])

    assert status.is_healthy is False
    assert status.fatal_errors == ()
    assert missing_log_hint(status, log_path) is None


def test_missing_log_hint_reports_path_when_log_file_genuinely_absent(tmp_path):
    log_path = tmp_path / "absent.log"
    status = load_sclang_health(log_path, critical_synths=["strings"])

    assert status.is_healthy is False
    hint = missing_log_hint(status, log_path)

    assert hint == f"Log file not found: {log_path}"
