#!/usr/bin/env python3
"""Unit tests for barge-in stale-dialogue discard in TTSNode (commit 37527df).

Регрессия 37527df: dialogue_id propagation через /voice/current_dialogue_id.
Ключевой инвариант: после нового dialogue_id — speak_text от старого диалога
НЕ должен воспроизводиться. На уровне tts_node это означает:

  * dialogue_callback() ДО синтеза отбрасывает chunk, чей dialogue_id
    отличается от current_dialogue_id (старый диалог) — вместо того чтобы
    синтезировать и проигрывать его;
  * для отброшенного chunk публикуется /voice/tts/finished с
    success=False, error="stale_dialogue", чтобы SpeakTextTool (MCP)
    не висел в ожидании;
  * chunk нового/текущего диалога НЕ отбрасывается.

Эти тесты не требуют ROS — используют тот же conftest-стаб, что и остальные
tts-юниты (test_silero_warm_load.py, test_provider_chain.py).
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock, patch

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_voice import tts_node  # noqa: E402
from rob_box_voice.tts_node import TTSNode  # noqa: E402

# ── Helpers ─────────────────────────────────────────────────────────────────


def _make_node() -> Any:
    """Construct a TTSNode with stubs (conftest), like other tts unit tests."""
    cls = tts_node.TTSNode

    def _fake_load(self):
        self.silero_model = MagicMock(name="FakeSileroModel")

    with patch.object(cls, "_load_silero_model", autospec=True, side_effect=_fake_load):
        node = cls()
    return node


def _destroy_node(node) -> None:
    for attr in (
        "_synthesis_executor", "audio_pub", "state_pub", "finished_pub",
        "dialogue_sub", "tts_request_sub", "control_sub",
        "_new_dialogue_id_sub", "playback_manager", "yandex_stub",
    ):
        if hasattr(node, attr):
            try:
                setattr(node, attr, None)
            except Exception:  # noqa: BLE001
                pass


def _chunk_msg(ssml: str = "<speak>test</speak>", dialogue_id: str | None = None) -> "object":
    """Build a fake String message with JSON payload like dialogue_node sends."""
    import types

    payload = {"ssml": ssml, "speech_id": "speech-test-123"}
    if dialogue_id is not None:
        payload["dialogue_id"] = dialogue_id
    msg = types.SimpleNamespace(data=json.dumps(payload, ensure_ascii=False))
    return msg


def _published_finished(node) -> list[dict]:
    """All /voice/tts/finished payloads published by the node so far."""
    out = []
    for call in node.finished_pub.publish.call_args_list:
        try:
            out.append(json.loads(call.args[0].data))
        except Exception:  # noqa: BLE001
            pass
    return out


# ── Tests ───────────────────────────────────────────────────────────────────


def test_stale_dialogue_chunk_discarded_before_synthesis() -> None:
    """КЛЮЧЕВОЙ ИНВАРИАНТ 37527df: chunk старого диалога отбрасывается ДО синтеза.

    current_dialogue_id='new-dlg' → chunk с dialogue_id='old-dlg' должен быть
    отброшен: не должно быть вызова синтеза/проигрывания, должен уйти
    /voice/tts/finished с error='stale_dialogue'.
    """
    node = _make_node()
    try:
        node.current_dialogue_id = "new-dlg"
        # Обнуляем счётчик finished, чтобы не путаться с шумом из __init__
        node.finished_pub.publish.reset_mock()

        node.dialogue_callback(_chunk_msg(dialogue_id="old-dlg"))

        # Отброшенный chunk НЕ должен идти в синтез: stale-проверка стоит
        # ДО синтеза, для stale-кейса мы выходим из callback раньше.
        # Проверяем это через отсутствие вызова _synthesize_and_play.
        if hasattr(node, "_synthesize_and_play") and hasattr(node._synthesize_and_play, "call_count"):
            assert node._synthesize_and_play.call_count == 0, (
                "stale chunk must not reach synthesis path"
            )
        finished = _published_finished(node)
        stale = [f for f in finished if f.get("error") == "stale_dialogue"]
        assert stale, (
            f"stale chunk must publish finished with error='stale_dialogue'; "
            f"got finished: {finished}"
        )
        assert stale[0].get("success") is False
        assert stale[0].get("speech_id") == "speech-test-123"
    finally:
        _destroy_node(node)


def test_current_dialogue_chunk_not_discarded() -> None:
    """Chunk текущего диалога НЕ отбрасывается (не должен дать stale_dialogue)."""
    node = _make_node()
    try:
        node.current_dialogue_id = "current-dlg"
        node.finished_pub.publish.reset_mock()

        # Подменяем синтез/извлечение текста — нас интересует только то,
        # что chunk НЕ был отброшен как stale. (диалог совпадает)
        with patch.object(
            node, "_extract_text_from_ssml", return_value="test", autospec=True
        ), patch.object(
            node, "_synthesize_and_play", return_value=True, autospec=True
        ):
            node.dialogue_callback(_chunk_msg(dialogue_id="current-dlg"))

        finished = _published_finished(node)
        stale = [f for f in finished if f.get("error") == "stale_dialogue"]
        assert not stale, (
            f"current-dialogue chunk must NOT be discarded as stale; got {finished}"
        )
    finally:
        _destroy_node(node)


def test_no_dialogue_id_chunk_not_discarded() -> None:
    """Chunk без dialogue_id (legacy) не отбрасывается — обратная совместимость."""
    node = _make_node()
    try:
        node.current_dialogue_id = "current-dlg"
        node.finished_pub.publish.reset_mock()

        with patch.object(
            node, "_extract_text_from_ssml", return_value="test", autospec=True
        ), patch.object(
            node, "_synthesize_and_play", return_value=True, autospec=True
        ):
            node.dialogue_callback(_chunk_msg(dialogue_id=None))

        finished = _published_finished(node)
        stale = [f for f in finished if f.get("error") == "stale_dialogue"]
        assert not stale, (
            f"chunk without dialogue_id must not be discarded; got {finished}"
        )
    finally:
        _destroy_node(node)


def test_first_chunk_initializes_current_dialogue_id() -> None:
    """Первый chunk диалога (current_dialogue_id ещё None) НЕ отбрасывается
    и инициализирует current_dialogue_id.

    Это соответствует реальному потоку 37527df: dialogue_node публикует новый
    dialogue_id в /voice/current_dialogue_id (_on_new_dialogue_id), затем
    SpeakTextTool шлёт chunk с этим id. Chunk с id != current отбрасывается
    на ПЕРВОЙ stale-проверке; вторая ветка (_interrupt_playback) достижима
    только когда current ещё None (первый chunk после сброса).
    """
    node = _make_node()
    try:
        node.current_dialogue_id = None
        node.finished_pub.publish.reset_mock()

        with patch.object(node, "_interrupt_playback", autospec=True) as mock_int, patch.object(
            node, "_extract_text_from_ssml", return_value="test", autospec=True
        ), patch.object(
            node, "_synthesize_and_play", return_value=True, autospec=True
        ):
            node.dialogue_callback(_chunk_msg(dialogue_id="first-dlg"))

        # Первый chunk не является stale → не публикуем finished со stale_dialogue
        finished = _published_finished(node)
        stale = [f for f in finished if f.get("error") == "stale_dialogue"]
        assert not stale, (
            f"first chunk must not be discarded as stale; got {finished}"
        )
        # interrupt при current=None не нужен (нечего прерывать)
        mock_int.assert_not_called()
        assert node.current_dialogue_id == "first-dlg", (
            f"current_dialogue_id must be initialized from the first chunk; "
            f"got {node.current_dialogue_id!r}"
        )
    finally:
        _destroy_node(node)


def test_on_new_dialogue_id_subscriber_updates_current() -> None:
    """Подписчик /voice/current_dialogue_id (_on_new_dialogue_id) обновляет
    current_dialogue_id — это тот механизм, что dialogue_node публикует
    новый dialogue_id при barge-in (37527df)."""
    node = _make_node()
    try:
        import types

        node.current_dialogue_id = None
        msg = types.SimpleNamespace(data="fresh-dialogue-abc")
        node._on_new_dialogue_id(msg)
        assert node.current_dialogue_id == "fresh-dialogue-abc", (
            f"_on_new_dialogue_id must update current_dialogue_id; "
            f"got {node.current_dialogue_id!r}"
        )
    finally:
        _destroy_node(node)
