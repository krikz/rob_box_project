"""test_vision_face_e2e_voice_3876.py

#3876: голос из E2E-БД дикторов (``SpeakerResult.e2e_db``) не пишет имя в боевую галерею лиц.
Метод ``VisionFaceNode._on_speaker_result`` вызывается на лёгком дублёре без конструктора ноды
(приём ``test_vision_face_log_stats.py``).
"""

from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path
from typing import Any, List
from unittest.mock import MagicMock


# ---------- import target under test (тот же приём, что test_face_recognition.py /
# test_face_tracker.py — пакет не установлен на dev-машине, добавляем его
# корень в sys.path вручную вместо надежды на PYTHONPATH) --------------------

_HERE = Path(__file__).resolve()
_PKG_ROOT = _HERE.parents[2]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

# ── rclpy shim (мирует test_perception_bridge.py / test_context_aggregator.py) ──
if 'rclpy' not in sys.modules:
    _rclpy = types.ModuleType('rclpy')
    _rclpy.init = lambda *a, **k: None
    _rclpy.spin = lambda *a, **k: None
    _rclpy.try_shutdown = lambda *a, **k: None
    _rclpy_node = types.ModuleType('rclpy.node')

    class _StubNode:  # pragma: no cover — только чтобы `from rclpy.node import Node` не упал
        def __init__(self, *a: Any, **k: Any) -> None:
            pass

    _rclpy_node.Node = _StubNode
    _rclpy.node = _rclpy_node
    sys.modules['rclpy'] = _rclpy
    sys.modules['rclpy.node'] = _rclpy_node

m = importlib.import_module('rob_box_perception.vision_face_node')


class _Logger:
    def __init__(self) -> None:
        self.info_lines: List[str] = []

    def info(self, msg: str) -> None:
        self.info_lines.append(msg)

    def warning(self, msg: str) -> None:
        self.info_lines.append(msg)


class _FakeNode:
    def __init__(self) -> None:
        self._recognizer = MagicMock()
        self._speaker_unknown_total = 0
        self._e2e_voice_skip_total = 0
        self._logger = _Logger()

    def get_logger(self) -> _Logger:
        return self._logger


def _msg(**fields: Any) -> types.SimpleNamespace:
    base = dict(event=0, is_known=True, speaker_id='c8fd47c4aaaa', name='Саша', e2e_db=False)
    base.update(fields)
    return types.SimpleNamespace(**base)


def test_prod_voice_result_is_merged_into_face():
    node = _FakeNode()
    m.VisionFaceNode._on_speaker_result(node, _msg(name='Денис'))
    node._recognizer.note_voice_identification.assert_called_once_with(
        speaker_id='c8fd47c4aaaa', name='Денис'
    )


def test_e2e_voice_result_is_not_copied_into_gallery():
    node = _FakeNode()
    m.VisionFaceNode._on_speaker_result(node, _msg(e2e_db=True))
    node._recognizer.note_voice_identification.assert_not_called()
    assert node._e2e_voice_skip_total == 1
    assert any('имя из e2e-голоса не копирую' in line for line in node._logger.info_lines)


def test_message_without_e2e_field_behaves_as_prod():
    """Сообщение старой сборки без поля - боевой результат (поведение не меняется)."""
    node = _FakeNode()
    msg = _msg()
    del msg.e2e_db
    m.VisionFaceNode._on_speaker_result(node, msg)
    node._recognizer.note_voice_identification.assert_called_once()
