"""Связка хода живёт в ``rob_box_dialog_host`` (ADR-0167 Ш1г, решение Шифу 09.11: В3 - Б).

Ш1г-1 перенёс модули, Ш1г-2 перевёл потребителей и удалил псевдонимы старых путей ``rob_box_voice``. Гард держит итог:
модулей связки в ``rob_box_voice`` нет, и никто в коде, тестах и скриптах не импортирует их по старому пути (вторая
точка входа в тот же модуль не возвращается).
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
HOST = REPO / "src" / "rob_box_dialog_host" / "rob_box_dialog_host"
VOICE = REPO / "src" / "rob_box_voice" / "rob_box_voice"
MOVED = ("command_bridge", "command_intent", "command_parser", "dialog_v2_ports", "faq_store", "media_phrases",
         "music_player_state", "speaker_memory", "voice_memory")
OLD = re.compile(r"rob_box_voice\.core\.(%s)(?![\w])|rob_box_voice\.tts_voice_registry(?![\w])" % "|".join(MOVED))
#: Где ищем старые пути: код, тесты, скрипты и CI (история - ADR, планы, отчёты, evidence - как написана).
ROOTS = ("src", "scripts", "tests", ".github", "docker", "packaging")


def test_moved_modules_live_only_in_the_host_package():
    for name in (*MOVED, "tts_voice_registry"):
        assert (HOST / f"{name}.py").is_file(), name
    if VOICE.is_dir():
        assert not [m for m in MOVED if (VOICE / "core" / f"{m}.py").exists()]
        assert not (VOICE / "tts_voice_registry.py").exists()


def test_host_package_does_not_import_rob_box_voice():
    for path in sorted(HOST.glob("*.py")):
        text = path.read_text(encoding="utf-8")
        assert not re.search(r"^\s*(from|import) rob_box_voice\b", text, re.MULTILINE), path.name


@pytest.mark.skipif(not (REPO / "src" / "rob_box_voice").is_dir(), reason="нет чекаута робота")
def test_nobody_imports_the_old_paths():
    me = Path(__file__).resolve()
    hits = []
    for root in ROOTS:
        for path in (REPO / root).rglob("*"):
            if path.suffix not in {".py", ".sh", ".yml", ".yaml", ".toml"} or path.resolve() == me:
                continue
            if "__pycache__" in path.parts:
                continue
            text = path.read_text(encoding="utf-8", errors="replace")
            hits += [f"{path.relative_to(REPO).as_posix()}: {m.group(0)}" for m in OLD.finditer(text)]
    assert hits == []
