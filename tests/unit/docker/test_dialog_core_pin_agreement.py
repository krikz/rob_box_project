"""Пины ядра в voice-assistant, Telegram и Teleop совпадают (#4932)."""

from __future__ import annotations

import re
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
_CI = REPO / "tests" / "ci"
if str(_CI) not in sys.path:
    sys.path.insert(0, str(_CI))

from dialog_core_pin.guard import pin_from_dockerfile, pin_in  # noqa: E402

_VOICE = REPO / "docker" / "sound" / "voice_assistant" / "Dockerfile"
_TELEGRAM = REPO / "docker" / "sound" / "telegram_bot" / "Dockerfile"
_TELEOP = REPO / "docker" / "main" / "teleop" / "Dockerfile"


def test_images_pin_the_same_core_tag() -> None:
    pin = pin_from_dockerfile(REPO)
    for dockerfile in (_VOICE, _TELEGRAM, _TELEOP):
        assert pin_in(dockerfile) == pin
    assert re.fullmatch(r"v\d+\.\d+\.\d+", pin), pin


@pytest.mark.parametrize("mismatched", [_TELEGRAM, _TELEOP], ids=["telegram", "teleop"])
def test_guard_refuses_diverged_pins(tmp_path: Path, mismatched: Path) -> None:
    for image in (_VOICE, _TELEGRAM, _TELEOP):
        dockerfile = tmp_path / image.relative_to(REPO)
        dockerfile.parent.mkdir(parents=True)
        pin = "v0.0.1" if image == mismatched else "v9.9.9"
        dockerfile.write_text(
            f"rob-box-dialog-core @ git+https://github.com/krikz/rob_box_dialog_core@{pin}\n",
            encoding="utf-8",
        )
    with pytest.raises(SystemExit, match=f"пины ядра разошлись: .*{mismatched.parent.name}"):
        pin_from_dockerfile(tmp_path)
