"""Пины ядра в образах voice и telegram — один тег (#4932).

Источник истины — docker/sound/voice_assistant/Dockerfile. telegram_bot
сверяется с ним. Расхождение или пропавший пин роняют тест. Сети нет:
читаются только два Dockerfile. Логика сверки — та же функция, что у CI-гарда.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
_CI = REPO / "tests" / "ci"
if str(_CI) not in sys.path:
    sys.path.insert(0, str(_CI))

from dialog_core_pin.guard import pin_from_dockerfile  # noqa: E402

_PIN_RE = re.compile(r"rob_box_dialog_core@([\w.]+)")
_VOICE = REPO / "docker" / "sound" / "voice_assistant" / "Dockerfile"
_TELEGRAM = REPO / "docker" / "sound" / "telegram_bot" / "Dockerfile"


def test_voice_and_telegram_pin_the_same_core_tag() -> None:
    """Оба образа несут один тег. Эталон читается из voice, второго хардкода нет."""
    pin = pin_from_dockerfile(REPO)
    voice = _PIN_RE.search(_VOICE.read_text(encoding="utf-8"))
    telegram = _PIN_RE.search(_TELEGRAM.read_text(encoding="utf-8"))
    assert voice and telegram, "пин rob_box_dialog_core@ пропал из Dockerfile"
    assert voice.group(1) == pin
    assert telegram.group(1) == pin
    assert re.fullmatch(r"v\d+\.\d+\.\d+", pin), pin


def test_guard_refuses_diverged_pins(tmp_path: Path) -> None:
    """Разошедшиеся пины — SystemExit, а не тихий пропуск."""
    voice_dir = tmp_path / "docker" / "sound" / "voice_assistant"
    telegram_dir = tmp_path / "docker" / "sound" / "telegram_bot"
    voice_dir.mkdir(parents=True)
    telegram_dir.mkdir(parents=True)
    (voice_dir / "Dockerfile").write_text(
        "rob-box-dialog-core @ git+https://github.com/krikz/rob_box_dialog_core@v9.9.9\n",
        encoding="utf-8",
    )
    (telegram_dir / "Dockerfile").write_text(
        "rob-box-dialog-core @ git+https://github.com/krikz/rob_box_dialog_core@v0.0.1\n",
        encoding="utf-8",
    )
    with pytest.raises(SystemExit, match="пины ядра разошлись"):
        pin_from_dockerfile(tmp_path)
