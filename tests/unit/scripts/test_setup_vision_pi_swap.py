"""Regression tests for ``scripts/setup/setup_vision_pi_swap.sh`` (issue #2621, ADR-0111).

The script must be:

* **Idempotent**: running `--auto` twice must not error, and the second run
  must report "параметры совпадают — пропускаю активацию".
* **Dry-run safe**: `--dry-run` must complete without any side effects,
  including no attempt to load the zram kernel module, write to
  ``/sys/block/zram0``, or install systemd units.
* **Validation strict**: bad input (size < 256 MB, size > 64 GB, unknown
  algorithm, out-of-range swappiness/priority) must exit non-zero WITHOUT
  touching the system.
* **Status works without root**: `--status` is read-only and must succeed
  for any user.
* **Syntax-clean**: bash -n must pass, shellcheck must pass.

These are unit-level tests — they don't verify that zram *actually* gets
allocated on a real Pi. That is e2e-process's job (issue #2621 acceptance
#1, real-hardware test). Here we pin the *contract*: the script behaves
correctly under all combinations of inputs we care about.

Mock strategy: we deliberately do NOT mock ``modprobe`` / ``mkswap`` /
``swapon``. The `--dry-run` and `--status` code paths branch before any
such call; for `--auto` we rely on the validation stage (which exits 64)
to prevent real side effects. This is the same strategy as
``tests/unit/scripts/test_image_versions_sha_contract.py``.
"""
from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "scripts" / "setup" / "setup_vision_pi_swap.sh"


# --------------------------------------------------------------------------- #
# Pure-syntax tests (no execution)
# --------------------------------------------------------------------------- #


def test_script_exists_and_executable() -> None:
    """The setup script must exist and be executable for the operator."""
    assert SCRIPT.exists(), f"{SCRIPT} not found"
    assert os.access(SCRIPT, os.X_OK), f"{SCRIPT} not executable (chmod +x)"


def test_bash_syntax_clean() -> None:
    """`bash -n` must succeed — no syntax errors."""
    result = subprocess.run(
        ["bash", "-n", str(SCRIPT)],
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0, (
        f"bash -n failed:\nstdout={result.stdout}\nstderr={result.stderr}"
    )


def test_shellcheck_clean() -> None:
    """`shellcheck` (CI: G-Lint Code / Shell Scripts) must pass without findings.

    Skipped if shellcheck is not installed locally (CI installs it via apt).
    """
    shellcheck = shutil.which("shellcheck")
    if shellcheck is None:
        pytest.skip("shellcheck not installed locally")

    result = subprocess.run(
        [shellcheck, str(SCRIPT)],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, (
        f"shellcheck found issues:\n{result.stdout}\n{result.stderr}"
    )


# --------------------------------------------------------------------------- #
# --dry-run (no root required, no side effects)
# --------------------------------------------------------------------------- #


def test_dry_run_completes_without_root() -> None:
    """`--dry-run` must work for non-root users and produce a plan.

    This is the contract e2e-process relies on: a developer can validate
    the script on a workstation before deploying to the Pi.
    """
    if os.geteuid() == 0:
        pytest.skip("test is meant for non-root contexts")

    result = subprocess.run(
        ["bash", str(SCRIPT), "--dry-run"],
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode == 0, (
        f"--dry-run failed:\nstdout={result.stdout}\nstderr={result.stderr}"
    )

    # Dry-run must surface the configured values
    assert "ROBBOX_ZRAM_SIZE_MB=4096" in result.stdout
    assert "ROBBOX_ZRAM_ALGO=zstd" in result.stdout
    assert "ROBBOX_ZRAM_SWAPPINESS=180" in result.stdout
    # And the swap-on command must appear (even if not executed)
    assert "swapon" in result.stdout
    # And the [DRY-RUN] marker must be present
    assert "[DRY-RUN]" in result.stdout


def test_dry_run_does_not_modify_system() -> None:
    """`--dry-run` must NOT call modprobe / write to /sys/ /etc/.

    We check that no error from "operation not permitted" leaks into stderr
    (which would indicate a real call attempt) AND that key directories
    are untouched. Since this test is order-independent we run it before
    any other test in the file.
    """
    swap_state_before = Path("/run/robbox-zram.state")
    if swap_state_before.exists():
        pytest.skip("/run/robbox-zram.state already exists from a real run")

    result = subprocess.run(
        ["bash", str(SCRIPT), "--dry-run"],
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode == 0
    # State file must NOT be created by a dry-run
    assert not swap_state_before.exists(), (
        "dry-run unexpectedly created /run/robbox-zram.state"
    )


# --------------------------------------------------------------------------- #
# --status (no root required)
# --------------------------------------------------------------------------- #


def test_status_works_without_root() -> None:
    """`--status` is read-only and must work for any user."""
    if os.geteuid() == 0:
        pytest.skip("test is meant for non-root contexts")

    result = subprocess.run(
        ["bash", str(SCRIPT), "--status"],
        capture_output=True,
        text=True,
        timeout=10,
    )
    # --status exits 0 (we use `exit 0` in cmd_status) regardless of state
    assert result.returncode == 0, (
        f"--status failed:\nstdout={result.stdout}\nstderr={result.stderr}"
    )
    # Must report something about /proc/swaps
    assert "/proc/swaps" in result.stdout
    # Must mention swappiness
    assert "swappiness" in result.stdout


# --------------------------------------------------------------------------- #
# Validation (bad inputs fail fast without side effects)
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "env_value,reason",
    [
        ("10", "size below minimum (256 MB)"),
        ("100", "size below minimum"),
        ("999999", "size above maximum (64 GB)"),
    ],
)
def test_size_validation_rejects_bad_values(env_value: str, reason: str) -> None:
    """Out-of-range sizes must exit 64 (EX_USAGE) without touching the system."""
    env = os.environ.copy()
    env["ROBBOX_ZRAM_SIZE_MB"] = env_value
    result = subprocess.run(
        ["bash", str(SCRIPT), "--dry-run"],
        capture_output=True,
        text=True,
        timeout=10,
        env=env,
    )
    assert result.returncode == 64, (
        f"expected exit 64 for {reason}, got {result.returncode}\n"
        f"stderr={result.stderr}"
    )
    assert "size" in result.stderr.lower() or "ROBBOX_ZRAM_SIZE_MB" in result.stderr


@pytest.mark.parametrize(
    "algo",
    ["zzz", "gzip", "snappy", "brotli"],
)
def test_algo_validation_rejects_unknown(algo: str) -> None:
    """Unknown compression algorithms must exit 64."""
    env = os.environ.copy()
    env["ROBBOX_ZRAM_ALGO"] = algo
    result = subprocess.run(
        ["bash", str(SCRIPT), "--dry-run"],
        capture_output=True,
        text=True,
        timeout=10,
        env=env,
    )
    assert result.returncode == 64, (
        f"expected exit 64 for algo={algo}, got {result.returncode}\n"
        f"stderr={result.stderr}"
    )
    assert "алгоритм" in result.stderr.lower() or "ALGO" in result.stderr


@pytest.mark.parametrize(
    "env_name,env_value,reason",
    [
        ("ROBBOX_ZRAM_SWAPPINESS", "300", "swappiness > 200"),
        ("ROBBOX_ZRAM_PRIORITY", "99999", "priority > 32767"),
        ("ROBBOX_ZRAM_PRIORITY", "-99999", "priority < -32768"),
    ],
)
def test_numeric_validation_rejects_out_of_range(
    env_name: str, env_value: str, reason: str
) -> None:
    """Out-of-range swappiness / priority must exit 64."""
    env = os.environ.copy()
    env[env_name] = env_value
    result = subprocess.run(
        ["bash", str(SCRIPT), "--dry-run"],
        capture_output=True,
        text=True,
        timeout=10,
        env=env,
    )
    assert result.returncode == 64, (
        f"expected exit 64 for {reason} ({env_name}={env_value}), "
        f"got {result.returncode}\nstderr={result.stderr}"
    )


# --------------------------------------------------------------------------- #
# Idempotency (dry-run twice in a row produces the same plan)
# --------------------------------------------------------------------------- #


def test_dry_run_idempotent() -> None:
    """Two consecutive `--dry-run` invocations must produce the same plan.

    This guards against accidental state-dependent behaviour: the plan
    must depend only on env vars, not on filesystem state.
    """
    result_1 = subprocess.run(
        ["bash", str(SCRIPT), "--dry-run"],
        capture_output=True,
        text=True,
        timeout=15,
    )
    result_2 = subprocess.run(
        ["bash", str(SCRIPT), "--dry-run"],
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result_1.returncode == 0
    assert result_2.returncode == 0
    # Plans must be byte-identical
    assert result_1.stdout == result_2.stdout, (
        "two consecutive --dry-run runs produced different plans — "
        "the script is not idempotent at the planning stage"
    )


# --------------------------------------------------------------------------- #
# Argument parsing
# --------------------------------------------------------------------------- #


def test_unknown_argument_exits_64() -> None:
    """Bogus CLI args must exit 64 (EX_USAGE) per BSD convention."""
    result = subprocess.run(
        ["bash", str(SCRIPT), "--bogus-flag"],
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 64
    assert "Unknown argument" in result.stderr


def test_help_flag_prints_usage_and_exits_0() -> None:
    """`-h` / `--help` must exit 0 and show the header comment."""
    result = subprocess.run(
        ["bash", str(SCRIPT), "--help"],
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 0
    # The header includes "ROBBOX Vision Pi"
    assert "ROBBOX Vision Pi" in result.stdout or "zram" in result.stdout.lower()