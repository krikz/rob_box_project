"""#4370: раздел ``tts_context`` кладёт в окно модели код - из звучащего состояния и реестра голосов.

М10 n402 «какой у тебя голос?»: подсказка велела называть голос по ``current_voice_name`` из ``tts_context``, а раздел
в окно никто не клал - робот ответил «MiniMax, мужской, стандартный». Проверяют поведение сборки раздела и снимка
(ADR-0148), а не текст промпта. Путь хоста (``dialog_node._sections``) - в ``test/unit/node/test_dialog_node.py``.
"""

from __future__ import annotations

import json
import types

from rob_box_core.msg_stub import install_dialog_msgs_stub

install_dialog_msgs_stub()

from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog.context import build_context  # noqa: E402

from rob_box_voice.core import plan_host  # noqa: E402
from rob_box_dialog_host.tts_voice_registry import format_tts_context, voice_info_for  # noqa: E402


def _node(provider: str = "minimax", applied: str | None = None) -> types.SimpleNamespace:
    voice = types.SimpleNamespace(applied=applied, provider=provider if applied else None)
    return types.SimpleNamespace(_actual_tts_provider=provider, _actual_tts_voice=None,
                                 _session=types.SimpleNamespace(snapshot=types.SimpleNamespace(voice=voice)))


def test_applied_voice_is_named_by_registry_in_the_window() -> None:
    """Звучит голос, применённый по ``set_voice``: в окне его id и имя для людей из каталога, не из памяти модели."""
    voice_id = "Russian_ReliableMan"
    info = voice_info_for("minimax", voice_id)
    assert info is not None
    section = plan_host.tts_context(_node(applied=voice_id))
    assert section["provider"] == "minimax"
    assert section["current_voice"] == voice_id
    assert section["current_voice_name"] == info["display_name"]
    window = build_context({"tts_context": section})
    assert "<tts_context>" in window and info["display_name"] in window


def test_default_voice_is_named_when_nothing_was_set() -> None:
    """``set_voice`` не звали: звучит голос по умолчанию, и его имя тоже в окне (n402)."""
    section = plan_host.tts_context(_node())
    assert section["current_voice"] == section["default_voice"]
    assert section["current_voice_name"]
    assert json.dumps(section, ensure_ascii=False, separators=(",", ":")) in build_context({"tts_context": section})


def test_section_is_known_to_snapshot_table_and_matches_the_line() -> None:
    """Раздел есть в таблице снимка, а строка ``[TTS]`` и раздел берут имя из одного места."""
    assert "tts_context" in kn.CONTEXT_SECTIONS
    section = plan_host.tts_context(_node(applied="Russian_ReliableMan"))
    line = format_tts_context("minimax", current_voice="Russian_ReliableMan")
    assert f"current_voice_name: {section['current_voice_name']}" in line
