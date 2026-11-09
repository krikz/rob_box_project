"""Связка хода переехала из ``rob_box_voice`` в ``rob_box_dialog_host`` (ADR-0167 Ш1г-1, решение Шифу 09.11: В3 - Б).

Старые пути - временные псевдонимы: тот же объект модуля, а не копия кода. Импорт по старому пути и
``mock.patch("rob_box_voice.core.X.attr")`` попадают в настоящий модуль. Ш1г-2 переводит импорты потребителей и
удаляет псевдонимы; :data:`ALIASES` только убывает.
"""

from __future__ import annotations

import ast
import importlib
from pathlib import Path
from unittest import mock

import pytest

HOST = Path(__file__).resolve().parents[1] / "rob_box_dialog_host"
VOICE = Path(__file__).resolve().parents[2] / "rob_box_voice" / "rob_box_voice"

#: Старый путь → модуль пакета связки. Ш1г-2 сводит к пустому.
ALIASES = {
    "rob_box_voice.core.command_bridge": "rob_box_dialog_host.command_bridge",
    "rob_box_voice.core.command_intent": "rob_box_dialog_host.command_intent",
    "rob_box_voice.core.command_parser": "rob_box_dialog_host.command_parser",
    "rob_box_voice.core.dialog_v2_ports": "rob_box_dialog_host.dialog_v2_ports",
    "rob_box_voice.core.faq_store": "rob_box_dialog_host.faq_store",
    "rob_box_voice.core.media_phrases": "rob_box_dialog_host.media_phrases",
    "rob_box_voice.core.music_player_state": "rob_box_dialog_host.music_player_state",
    "rob_box_voice.core.speaker_memory": "rob_box_dialog_host.speaker_memory",
    "rob_box_voice.core.voice_memory": "rob_box_dialog_host.voice_memory",
    "rob_box_voice.tts_voice_registry": "rob_box_dialog_host.tts_voice_registry",
}


def test_every_host_module_has_no_rob_box_voice_import():
    """Пакет связки не тянет ``rob_box_voice`` (ROS-пакет робота): ни на верхнем уровне, ни лениво."""
    for path in sorted(HOST.glob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8-sig"))
        roots = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
        roots |= {n.module.split(".")[0] for n in ast.walk(tree)
                  if isinstance(n, ast.ImportFrom) and n.level == 0 and n.module}
        assert "rob_box_voice" not in roots, path.name


@pytest.mark.skipif(not VOICE.is_dir(), reason="нет src/rob_box_voice (окружение без хостов робота)")
def test_aliases_are_the_same_module_objects():
    for old, new in ALIASES.items():
        assert importlib.import_module(old) is importlib.import_module(new), old


@pytest.mark.skipif(not VOICE.is_dir(), reason="нет src/rob_box_voice (окружение без хостов робота)")
def test_patch_by_the_old_path_reaches_the_real_module():
    import rob_box_dialog_host.media_phrases as real

    with mock.patch("rob_box_voice.core.media_phrases.NOTHING_PLAYING_TEXT", "подмена"):
        assert real.NOTHING_PLAYING_TEXT == "подмена"
    assert real.NOTHING_PLAYING_TEXT != "подмена"


@pytest.mark.skipif(not VOICE.is_dir(), reason="нет src/rob_box_voice (окружение без хостов робота)")
def test_alias_files_hold_no_code_of_their_own():
    """Псевдоним - три строки, вторая реализация не держится («не держать две реализации»)."""
    files = [VOICE / "core" / f"{o.rsplit('.', 1)[1]}.py" for o in ALIASES if ".core." in o]
    files.append(VOICE / "tts_voice_registry.py")
    for f in files:
        body = [n for n in ast.parse(f.read_text(encoding="utf-8")).body if not isinstance(n, ast.Expr)]
        assert [type(n).__name__ for n in body] == ["Import", "Import", "Assign"], f.name
