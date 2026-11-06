"""#4252: ``SpeechRequest(sink=headset)`` проходит владельца речи и настоящий вход ``tts_node`` до синтеза в шлем.

Связка, которую не видят тесты по отдельности: владелец кладёт в чанк ``sink=headset`` и ``request_id=speech_id``,
``TTSNode.dialogue_callback`` отдаёт его ветке шлема (``_on_headset_chunk``), та запускает синтез с
``sink="headset"`` и помнит ``request_id`` для ``/avatar/tts/audio_meta``. Финал этой реплики даёт только ``SpeechEvent``
(String-топика ``/voice/tts/finished`` нет с #4252 ч.2).
"""

from __future__ import annotations

import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_core.msg_stub import install_dialog_msgs_stub  # noqa: E402
from rob_box_voice.core.speech_owner import SpeechLedger, SpeechOwner  # noqa: E402
from rob_box_voice.tts_node import TTSNode  # noqa: E402

M = install_dialog_msgs_stub()


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


def _node():
    n = object.__new__(TTSNode)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n._avatar_tts_error_pub = _Pub()
    n._avatar_tts_request_id = None
    n.current_speech_id = None
    n._submit_synthesis = MagicMock()
    n._interrupt_playback = MagicMock()
    n._publish_tars1_text = MagicMock()
    stamp = type(M.SpeechEvent().stamp)()
    n.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=0, to_msg=lambda: stamp))
    owner = SpeechOwner(n, ledger=SpeechLedger(clock=lambda: 100.0))
    owner._msgs, owner._event_pub = M, _Pub()
    n._speech = owner
    return n, owner


def _request(**fields):
    req = M.SpeechRequest()
    req.speech_id, req.text, req.agent, req.sink, req.commit = "sid-h", "Готово.", "operator", "headset", True
    for key, value in fields.items():
        setattr(req, key, value)
    return req


def _kinds(owner):
    names = {getattr(M.SpeechEvent, k): k[5:].lower() for k in dir(M.SpeechEvent) if k.startswith("KIND_")}
    return [names[e.kind] for e in owner._event_pub.sent]


def test_headset_request_reaches_headset_synthesis_with_speech_id_as_request_id():
    node, owner = _node()
    owner.on_request(_request(voice="male-qn-qingse"))
    assert node._submit_synthesis.call_count == 1
    kwargs = node._submit_synthesis.call_args.kwargs
    assert (kwargs["sink"], kwargs["voice"]) == ("headset", "male-qn-qingse")
    assert node._submit_synthesis.call_args.args[1] == "sid-h"  # speech_id для журнала и финала
    assert node._avatar_tts_request_id == "sid-h"  # ключ /avatar/tts/audio_meta = speech_id SpeechEvent
    assert _kinds(owner) == ["queued"]


def test_headset_finish_from_the_pipeline_closes_the_speech_with_an_event():
    node, owner = _node()
    owner.on_request(_request())
    node._publish_tts_finished("sid-h", success=True, duration_sec=0.7)
    assert _kinds(owner) == ["queued", "finished"]
    assert not hasattr(node, "finished_pub")


def test_headset_path_does_not_log_the_old_deprecated_topic_warning():
    """Ложный «/avatar/tts/request DEPRECATED» печатался и на новом пути SpeechRequest(sink=headset)."""
    node, owner = _node()
    owner.on_request(_request())
    logged = " ".join(str(c.args[0]) for c in node.get_logger().warn.call_args_list)
    assert "DEPRECATED" not in logged


def test_headset_empty_text_is_rejected_by_the_pipeline_with_an_event():
    node, owner = _node()
    owner.on_request(_request(text="   "))
    assert node._submit_synthesis.call_count == 0
    assert _kinds(owner) == ["queued", "rejected"]
    assert owner._event_pub.sent[-1].reason == "empty_text"
