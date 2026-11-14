"""DNS-патч стенда lite-kit применяется к копии исходника и даёт эталонный sha256.

Issue #4553: патч жил только в локальной копии C:\\lite-kit. Здесь он
воспроизводимо применяется к фикстуре scripts/robot_dev/lite-kit-dns/fixtures/
и сверяется с зафиксированным sha256 результата. Сырой вывод bash -n/sha256sum
самопроверки виден в stdout pytest (-v показывает captured output).
"""

import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SELFTEST = ROOT / "scripts" / "robot_dev" / "lite-kit-dns" / "selftest-dns-patch.sh"
APPLY = ROOT / "scripts" / "robot_dev" / "lite-kit-dns" / "apply-dns-patch.sh"
PATCH = ROOT / "scripts" / "robot_dev" / "lite-kit-dns" / "equasar_server.py.dns.patch"
FIXTURE = (
    ROOT
    / "scripts"
    / "robot_dev"
    / "lite-kit-dns"
    / "fixtures"
    / "equasar_server.py.bak-20261108"
)

SOURCE_SHA256 = "d08942a1627cbaa576404a303d548b623a39135500d08978893ef4833a24e6b8"
PATCHED_SHA256 = "87bd82bc1c3235be4be6b2838f17410573276078d41d9551ee47bfd5e16c8d31"


def test_files_exist():
    for p in (SELFTEST, APPLY, PATCH, FIXTURE):
        assert p.is_file(), f"нет файла {p}"


def test_fixture_is_untouched_source():
    import hashlib

    digest = hashlib.sha256(FIXTURE.read_bytes()).hexdigest()
    assert digest == SOURCE_SHA256, (
        "фикстура отличается от исходника lite-kit @ 8a82ff3 — "
        "патч перестал быть применимым к чистому стенду"
    )


def test_patch_keeps_crlf_bytes():
    # -text в .gitattributes обязан уберечь дифф от нормализации в LF:
    # контекстные строки несут CRLF, без них patch --binary испортит концы строк.
    content_lines = [
        line
        for line in PATCH.read_bytes().split(b"\n")
        if line.startswith((b" ", b"+", b"-")) and not line.startswith((b"---", b"+++"))
    ]
    assert content_lines, "в диффе нет ни одной строки контекста"
    crlf = sum(1 for line in content_lines if line.endswith(b"\r"))
    assert (
        crlf > len(content_lines) * 0.9
    ), f"контекстные строки без CRLF ({crlf}/{len(content_lines)}) — gitattributes не сработал"


def test_selftest_passes():
    # Относительный путь + cwd: на Windows subprocess может поднять WSL-backed bash,
    # которому абсолютный Windows-путь (D:/...) недоступен, а cwd он переварит.
    proc = subprocess.run(
        ["bash", "selftest-dns-patch.sh"],
        cwd=SELFTEST.parent,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=120,
    )
    assert proc.returncode == 0, (
        f"selftest-dns-patch.sh упал (rc={proc.returncode})\n"
        f"--- stdout ---\n{proc.stdout}\n--- stderr ---\n{proc.stderr}"
    )
    assert "bash -n: OK" in proc.stdout
    assert SOURCE_SHA256 in proc.stdout
    assert PATCHED_SHA256 in proc.stdout
    assert proc.stdout.rstrip().endswith("SELTEST PASS")
