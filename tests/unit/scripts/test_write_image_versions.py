"""Tests for ``scripts/ci/write-image-versions.sh`` (ADR-0159 PR-3).

One copy of the «write a SHA tag into .image-versions» logic for
``L: Build Pi Services`` and ``L: Build Single Service``.  The contract:

* a field is rewritten in EVERY ``docker/<pi>/.image-versions.<tag>`` that
  declares it — ``PERCEPTION_TAG`` is built by the Main Pi build but deployed
  by Main and Sound Pi (sensor-bridge, ADR-0159 §2.5);
* stdout lists exactly the touched files (the caller ``git add``s them);
* other tags' files and other fields stay untouched;
* no file for the docker tag (``local``) or an undeclared field is a warning,
  not a failure — the same as the old ``sed`` that matched nothing.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "scripts" / "ci" / "write-image-versions.sh"

BASH = shutil.which("bash")
pytestmark = pytest.mark.skipif(BASH is None, reason="bash is not available")


def _tree(tmp_path: Path) -> Path:
    root = tmp_path / "docker"
    files = {
        "main/.image-versions.dev": "# main\n\nNAV2_TAG=dev-old\nPERCEPTION_TAG=dev-old\n",
        "sound/.image-versions.dev": (
            "# sound\n\nVOICE_ASSISTANT_TAG=dev-old\n# separator\nPERCEPTION_TAG=dev-old\n"
        ),
        "vision/.image-versions.dev": "# vision\n\nOAK_D_TAG=dev-old\n",
        "main/.image-versions.test": "PERCEPTION_TAG=test-old\n",
    }
    for rel, text in files.items():
        path = root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(text.encode("utf-8"))
    return root


def _run(root: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [BASH, str(SCRIPT), *args],
        cwd=root.parent,
        capture_output=True,
        text=True,
        encoding="utf-8",
        env={**os.environ, "IMAGE_VERSIONS_DIR": "docker"},
    )


def _read(root: Path, rel: str) -> str:
    return (root / rel).read_bytes().decode("utf-8")


def test_shared_field_is_written_into_every_pi_that_declares_it(tmp_path: Path) -> None:
    root = _tree(tmp_path)
    proc = _run(root, "dev", "dev-abc1234", "PERCEPTION_TAG", "NAV2_TAG")

    assert proc.returncode == 0, proc.stderr
    assert sorted(proc.stdout.split()) == [
        "docker/main/.image-versions.dev",
        "docker/sound/.image-versions.dev",
    ]
    assert _read(root, "main/.image-versions.dev") == (
        "# main\n\nNAV2_TAG=dev-abc1234\nPERCEPTION_TAG=dev-abc1234\n"
    )
    # Sound's own field (written by the Sound build) is untouched.
    assert _read(root, "sound/.image-versions.dev") == (
        "# sound\n\nVOICE_ASSISTANT_TAG=dev-old\n# separator\nPERCEPTION_TAG=dev-abc1234\n"
    )
    assert (
        _read(root, "vision/.image-versions.dev") == "# vision\n\nOAK_D_TAG=dev-old\n"
    )
    # Another docker tag's file is never touched.
    assert _read(root, "main/.image-versions.test") == "PERCEPTION_TAG=test-old\n"


def test_no_file_for_docker_tag_is_empty_output_and_success(tmp_path: Path) -> None:
    root = _tree(tmp_path)
    proc = _run(root, "local", "local-abc1234", "PERCEPTION_TAG")

    assert proc.returncode == 0, proc.stderr
    assert proc.stdout == ""
    assert ".image-versions.local" in proc.stderr


def test_undeclared_field_warns_and_changes_nothing(tmp_path: Path) -> None:
    root = _tree(tmp_path)
    proc = _run(root, "dev", "dev-abc1234", "APRILTAG_TAG")

    assert proc.returncode == 0, proc.stderr
    assert proc.stdout == ""
    assert "APRILTAG_TAG" in proc.stderr
    assert (
        _read(root, "vision/.image-versions.dev") == "# vision\n\nOAK_D_TAG=dev-old\n"
    )


def test_non_tag_argument_is_rejected(tmp_path: Path) -> None:
    root = _tree(tmp_path)
    proc = _run(root, "dev", "dev-abc1234", "rm -rf")

    assert proc.returncode == 2
    assert proc.stdout == ""
