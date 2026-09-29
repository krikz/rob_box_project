"""``dialogue_callback`` решает «синтезировать / отбросить» по общему контракту.

Приёмник ``/voice/tts/request`` и продюсеры (``say``, ``speak_text``,
stt_node, telegram, грип) проверяют payload одной функцией —
``rob_box_core.utterance.missing_tts_request_fields``. Здесь фиксируем
сторону приёмника: callback отбрасывает ровно то, что отвергает функция,
и пропускает дальше то, что она принимает. Сторона продюсера ``say`` —
``rob_box_mcp_tools/test/test_say_tool_tts_contract.py``.

Запуск:
    PYTHONPATH=src/rob_box_voice:src/rob_box_core:src/rob_box_harness:src/rob_box_llm \\
        pytest src/rob_box_voice/test/unit/tts/test_tts_request_contract.py -v
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_core.utterance import Utterance, missing_tts_request_fields  # noqa: E402
from rob_box_voice import tts_node as tts_node_mod  # noqa: E402
from rob_box_voice.tts_node import TTSNode  # noqa: E402


def _make_node() -> TTSNode:
    """Bare-stub: callback доходит до проверки SSML и дальше до dialogue_id."""
    logger = MagicMock()
    n = object.__new__(TTSNode)
    n.get_logger = lambda: logger
    n._test_logger = logger
    n.current_speech_id = None
    # Всё после проверки SSML нам не важно: helper dialogue_id — точка,
    # по которой видно «запрос принят».
    n._handle_dialogue_id_change = MagicMock(return_value=True)
    return n


def _msg(payload: dict):
    m = MagicMock()
    m.data = json.dumps(payload, ensure_ascii=False)
    return m


def _dropped_as_no_ssml(node) -> bool:
    return any(
        "без SSML" in str(c.args[0]) for c in node._test_logger.warn.call_args_list
    )


@pytest.mark.parametrize(
    "payload",
    [
        {"text": "привет", "source": "operator"},  # старый формат say (29.09)
        {"text": "привет"},
        {},
    ],
)
def test_callback_drops_payload_rejected_by_contract(payload):
    assert missing_tts_request_fields(payload)
    node = _make_node()

    node.dialogue_callback(_msg(payload))

    assert _dropped_as_no_ssml(node)
    node._handle_dialogue_id_change.assert_not_called()


@pytest.mark.parametrize(
    "payload",
    [
        Utterance(text="x<y & z", extra={"source": "operator"}).to_request(),
        {"ssml": "<speak>привет</speak>"},
    ],
)
def test_callback_accepts_payload_accepted_by_contract(payload):
    assert missing_tts_request_fields(payload) == []
    node = _make_node()

    node.dialogue_callback(_msg(payload))

    assert not _dropped_as_no_ssml(node)
    node._handle_dialogue_id_change.assert_called_once()


def test_callback_uses_shared_contract_function(monkeypatch):
    """Callback зовёт именно общую функцию, а не свою копию проверки."""
    calls = []

    def _spy(payload):
        calls.append(payload)
        return ["ssml"]

    monkeypatch.setattr(tts_node_mod, "missing_tts_request_fields", _spy)
    node = _make_node()

    node.dialogue_callback(_msg({"ssml": "<speak>a</speak>"}))

    assert calls == [{"ssml": "<speak>a</speak>"}]
    assert _dropped_as_no_ssml(node)
