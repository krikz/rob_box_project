"""Regression tests for the YAML-loader in
``docker/vision/scripts/vision-hailo/start_vision_hailo.sh``.

Issue #2496 (F-2 from t_beba0869 component review):

    YAML-loader unconditionally overrides ENV values — silent degradation
    on prod when ``HAILO_ENABLED=true`` is set in ``.env`` but
    ``vision_hailo_node.hailo_enabled: false`` is left in
    ``docker/vision/config/hailo_models.yaml`` (dev-PoC stub default).
    Robot starts in stub-mode with no warning.

Fix (ADR-0018 capability-honest): ENV > YAML > defaults.
If a key is present non-empty in the environment, the YAML value for
that key is skipped and an ``INFO`` log line is emitted. YAML becomes
**defaults**, not **override**.

These tests invoke the Python heredoc that lives inside the bash
``eval`` block of the start-script. We extract the Python source from
the script with textwrap, write it to a temp file (the script itself
is not executable on dev/CI because it sources ROS workspace), and
assert on the emitted ``export KEY="VALUE"`` lines and the
``INFO: ENV already set`` markers.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = (
    REPO_ROOT
    / "docker"
    / "vision"
    / "scripts"
    / "vision-hailo"
    / "start_vision_hailo.sh"
)


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #


_PY_HEREDOC_RE = re.compile(
    r"python3 -\s*\"\$\{HAILO_MODELS_YAML\}\" <<'PY'\n(?P<body>.*?)\nPY",
    re.DOTALL,
)


def _extract_yaml_loader_python() -> str:
    """Extract the Python heredoc body from start_vision_hailo.sh.

    The script is the SSoT; tests mirror the source so any drift
    (e.g. someone changing the ENV precedence semantics) breaks
    this test with a diff.
    """
    text = SCRIPT.read_text(encoding="utf-8")
    match = _PY_HEREDOC_RE.search(text)
    assert match is not None, (
        f"could not find Python heredoc in {SCRIPT}; "
        "the YAML-loader block was renamed or removed"
    )
    return match.group("body")


def _run_loader(yaml_text: str, env: dict[str, str]) -> subprocess.CompletedProcess[str]:
    """Write a fixture YAML, invoke the loader with `env`, return result.

    The Python block is extracted from the script and written to a
    temp ``.py`` file with ``sys.argv = [loader_path, yaml_path]``.
    We do NOT need ROS / colcon / vision_hailo runtime — only
    PyYAML (already a dev-dep) and the stdlib.
    """
    loader_py = textwrap.dedent(_extract_yaml_loader_python())
    full_env = os.environ.copy()
    full_env.update(env)

    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        loader_path = Path(tmp) / "loader.py"
        yaml_path = Path(tmp) / "hailo_models.yaml"
        loader_path.write_text(loader_py, encoding="utf-8")
        yaml_path.write_text(yaml_text, encoding="utf-8")

        proc = subprocess.run(
            [sys.executable, str(loader_path), str(yaml_path)],
            capture_output=True,
            text=True,
            env=full_env,
            timeout=15,
        )
    return proc


def _exports(proc: subprocess.CompletedProcess[str]) -> dict[str, str]:
    """Parse ``export KEY="VALUE"`` lines from stdout → dict."""
    out: dict[str, str] = {}
    for line in proc.stdout.splitlines():
        m = re.match(r'^export\s+([A-Z0-9_]+)="(.*)"$', line)
        if m:
            out[m.group(1)] = m.group(2)
    return out


def _env_skips(proc: subprocess.CompletedProcess[str]) -> set[str]:
    """Parse the ENV-skip INFO markers from stdout.

    The Python block emits ``echo "[start_vision_hailo] INFO: ENV
    XXX already set, YAML key '...' ignored" >&2`` to stdout, so bash
    ``eval`` runs the echo with stderr redirection. We invoke the
    Python loader directly (no bash wrapper), so the echo command
    itself sits as a literal line in stdout. We parse those literal
    lines here.
    """
    out: set[str] = set()
    pattern = re.compile(r"ENV (\w+) already set, YAML key '[^']+' ignored")
    for line in proc.stdout.splitlines():
        m = pattern.search(line)
        if m:
            out.add(m.group(1))
    return out


# --------------------------------------------------------------------------- #
# Fixtures
# --------------------------------------------------------------------------- #


YAML_PROD_FALSE = textwrap.dedent("""\
    vision_hailo_node:
      hailo_enabled: false
      hef_path: ""
      stub_period_sec: 2.0
      confidence_threshold: 0.5
      input_topic: /oak/rgb/image_raw/compressed
      output_topic: /vision/hailo/events
""")


YAML_PROD_TRUE = textwrap.dedent("""\
    vision_hailo_node:
      hailo_enabled: true
      hef_path: /opt/rob_box/models/yolov8n.hef
      stub_period_sec: 5.0
      confidence_threshold: 0.7
      input_topic: /prod/cam0/image
      output_topic: /prod/vision/events
""")


# --------------------------------------------------------------------------- #
# Tests — ENV wins over YAML (the core fix)
# --------------------------------------------------------------------------- #


def test_env_set_overrides_yaml_false_to_true(tmp_path: Path) -> None:
    """Regression test for issue #2496 / F-2.

    ``.env`` has ``HAILO_ENABLED=true`` (production intent), YAML has
    ``hailo_enabled: false`` (dev-PoC leftover). After fix, ENV wins
    and YAML is skipped for that key (with INFO log line).
    """
    proc = _run_loader(YAML_PROD_FALSE, env={"HAILO_ENABLED": "true"})
    assert proc.returncode == 0, proc.stderr

    exports = _exports(proc)
    assert "HAILO_ENABLED" not in exports, (
        "HAILO_ENABLED should NOT be exported by YAML-loader when ENV is set; "
        f"got exports={exports!r}, stderr={proc.stderr!r}"
    )
    assert "HAILO_ENABLED" in _env_skips(proc), (
        "expected ENV-skip INFO log line for HAILO_ENABLED; "
        f"stderr={proc.stderr!r}"
    )


def test_env_unset_yaml_default_applied() -> None:
    """No ENV → YAML default flows through normally (legacy behaviour)."""
    proc = _run_loader(YAML_PROD_FALSE, env={})
    assert proc.returncode == 0, proc.stderr

    exports = _exports(proc)
    assert exports["HAILO_ENABLED"] == "false"
    assert exports["INPUT_TOPIC"] == "/oak/rgb/image_raw/compressed"
    assert exports["OUTPUT_TOPIC"] == "/vision/hailo/events"
    assert _env_skips(proc) == set()


def test_env_empty_string_falls_through_to_yaml() -> None:
    """``HAILO_ENABLED=`` (empty) is not an explicit override.

    Per ADR-0018 capability-honest, empty ENV is treated as unset:
    YAML default applies. This prevents a botched unset
    (e.g. commented-out line in .env) from being interpreted as a
    deliberate decision.
    """
    proc = _run_loader(YAML_PROD_FALSE, env={"HAILO_ENABLED": ""})
    assert proc.returncode == 0, proc.stderr

    exports = _exports(proc)
    assert exports["HAILO_ENABLED"] == "false", (
        f"empty ENV should fall through to YAML; got HAILO_ENABLED={exports.get('HAILO_ENABLED')!r}"
    )


def test_per_key_independence_env_overrides_only_some() -> None:
    """Per-key independence: ENV for one key, YAML for the rest.

    Operator might override only ``INPUT_TOPIC`` in .env while
    accepting YAML defaults for everything else.
    """
    env = {"INPUT_TOPIC": "/custom/cam5/image"}
    proc = _run_loader(YAML_PROD_FALSE, env=env)
    assert proc.returncode == 0, proc.stderr

    exports = _exports(proc)
    assert "INPUT_TOPIC" not in exports, (
        f"INPUT_TOPIC must be skipped (ENV wins); exports={exports!r}"
    )
    # Other YAML keys should still flow through.
    assert exports["HAILO_ENABLED"] == "false"
    assert exports["CONFIDENCE_THRESHOLD"] == "0.5"
    assert _env_skips(proc) == {"INPUT_TOPIC"}


def test_env_false_value_wins_over_yaml_true() -> None:
    """Symmetric case: ENV ``HAILO_ENABLED=false`` should beat YAML ``true``.

    Important because disabling real inference on prod (e.g. for
    hot-maintenance) must work without editing YAML.
    """
    proc = _run_loader(YAML_PROD_TRUE, env={"HAILO_ENABLED": "false"})
    assert proc.returncode == 0, proc.stderr

    assert "HAILO_ENABLED" not in _exports(proc)
    assert "HAILO_ENABLED" in _env_skips(proc)


def test_yaml_key_with_underscore_mapping() -> None:
    """YAML ``stub_period_sec`` maps to ENV ``STUB_PERIOD_SEC``.

    The loader upper-cases keys; verify the mapping is consistent.
    """
    env = {"STUB_PERIOD_SEC": "10.0"}
    proc = _run_loader(YAML_PROD_FALSE, env=env)
    assert proc.returncode == 0, proc.stderr

    assert "STUB_PERIOD_SEC" not in _exports(proc)
    exports = _exports(proc)
    assert exports["CONFIDENCE_THRESHOLD"] == "0.5"
    assert _env_skips(proc) == {"STUB_PERIOD_SEC"}


# --------------------------------------------------------------------------- #
# Tests — resilience (existing behaviour, regression)
# --------------------------------------------------------------------------- #


def test_missing_yaml_key_skipped() -> None:
    """YAML missing a key → no export, no error, no ENV-skip log."""
    yaml_text = textwrap.dedent("""\
        vision_hailo_node:
          hailo_enabled: true
          # no hef_path, no input_topic, etc.
    """)
    proc = _run_loader(yaml_text, env={})
    assert proc.returncode == 0, proc.stderr

    exports = _exports(proc)
    assert exports == {"HAILO_ENABLED": "true"}


def test_malformed_yaml_does_not_crash() -> None:
    """YAML parse error → warn line emitted to stdout, exit 0, no exports.

    The script handles this defensively (warns + sys.exit(0) inside
    the heredoc) so a typo in the SSoT config doesn't kill the
    container on boot. The WARN is emitted via ``echo "..." >&2``
    to stdout (literal bash command) — see ``_env_skips`` docstring.
    """
    yaml_text = "vision_hailo_node: [unclosed bracket"
    proc = _run_loader(yaml_text, env={"HAILO_ENABLED": "true"})
    assert proc.returncode == 0, proc.stderr

    assert "WARN: yaml parse failed" in proc.stdout, (
        f"expected YAML parse warn in stdout; got stdout={proc.stdout!r}, "
        f"stderr={proc.stderr!r}"
    )
    assert _exports(proc) == {}


def test_yaml_with_bool_and_int_types() -> None:
    """bool → 'true'/'false'; int/float pass through as their str().

    ROS 2 parameter passing requires string 'true'/'false' (not
    Python's True/False), and numeric thresholds need explicit
    string conversion for the shell export.
    """
    yaml_text = textwrap.dedent("""\
        vision_hailo_node:
          hailo_enabled: true
          stub_period_sec: 3.5
          confidence_threshold: 0.42
    """)
    proc = _run_loader(yaml_text, env={})
    assert proc.returncode == 0, proc.stderr

    exports = _exports(proc)
    assert exports["HAILO_ENABLED"] == "true"
    assert exports["STUB_PERIOD_SEC"] == "3.5"
    assert exports["CONFIDENCE_THRESHOLD"] == "0.42"


# --------------------------------------------------------------------------- #
# Test — bash-eval integration (full pipeline emulation)
# --------------------------------------------------------------------------- #


def test_bash_eval_pipeline_yaml_wins_over_yaml_when_env_unset() -> None:
    """Run the loader output through bash ``eval`` and assert final env.

    This is the integration check: ensures the Python output format
    (each line either ``export KEY="VAL"`` or ``echo "..." >&2``) is
    actually consumable by bash. Guards against typos like missing
    quotes around the value.
    """
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        yaml_path = Path(tmp) / "hailo_models.yaml"
        yaml_path.write_text(YAML_PROD_FALSE, encoding="utf-8")

        loader_py = textwrap.dedent(_extract_yaml_loader_python())
        loader_path = Path(tmp) / "loader.py"
        loader_path.write_text(loader_py, encoding="utf-8")

        # Run Python loader to get the bash code, then source it.
        py_proc = subprocess.run(
            [sys.executable, str(loader_path), str(yaml_path)],
            capture_output=True, text=True, timeout=15,
        )
        assert py_proc.returncode == 0, py_proc.stderr

        # Write the bash code to a file, then `source` it (avoids
        # multi-line quoting headaches with `bash -c`).
        bash_path = Path(tmp) / "emit.sh"
        bash_path.write_text(py_proc.stdout + "\nenv\n", encoding="utf-8")

        bash_proc = subprocess.run(
            ["bash", str(bash_path)],
            capture_output=True, text=True,
            env={},  # start clean — bash sees no HAILO_ENABLED in inherited env
            timeout=15,
        )
        assert bash_proc.returncode == 0, bash_proc.stderr

        env_lines = dict(
            line.split("=", 1) for line in bash_proc.stdout.splitlines() if "=" in line
        )
        assert env_lines.get("HAILO_ENABLED") == "false", env_lines
        assert env_lines.get("INPUT_TOPIC") == "/oak/rgb/image_raw/compressed"


def test_bash_eval_pipeline_env_wins_over_yaml() -> None:
    """Integration: bash sees HAILO_ENABLED=true in ENV → YAML skip wins.

    This is the actual production scenario from issue #2496:
    the .env export is preserved through the eval, the YAML value
    is skipped, and the final ENV reflects the operator's intent.
    """
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        yaml_path = Path(tmp) / "hailo_models.yaml"
        yaml_path.write_text(YAML_PROD_FALSE, encoding="utf-8")

        loader_py = textwrap.dedent(_extract_yaml_loader_python())
        loader_path = Path(tmp) / "loader.py"
        loader_path.write_text(loader_py, encoding="utf-8")

        py_proc = subprocess.run(
            [sys.executable, str(loader_path), str(yaml_path)],
            capture_output=True, text=True,
            env={**os.environ, "HAILO_ENABLED": "true"},
            timeout=15,
        )
        assert py_proc.returncode == 0, py_proc.stderr

        bash_path = Path(tmp) / "emit.sh"
        bash_path.write_text(py_proc.stdout + "\nenv\n", encoding="utf-8")

        bash_proc = subprocess.run(
            ["bash", str(bash_path)],
            capture_output=True, text=True,
            env={**os.environ, "HAILO_ENABLED": "true"},
            timeout=15,
        )
        assert bash_proc.returncode == 0, bash_proc.stderr

        env_lines = dict(
            line.split("=", 1) for line in bash_proc.stdout.splitlines() if "=" in line
        )
        # ENV wins — HAILO_ENABLED should be "true", not "false".
        assert env_lines.get("HAILO_ENABLED") == "true", env_lines
        # And the INFO log line was emitted (via echo >&2).
        assert "[start_vision_hailo] INFO: ENV HAILO_ENABLED" in bash_proc.stderr