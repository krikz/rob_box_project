"""test_issue_3876_e2e_result_stamp.py

#3876: результат speaker_id_node в e2e_mode несёт метку ``e2e_db`` на КАЖДОМ сообщении (одна точка выхода),
чтобы vision_face_node не писал синтетические имена («Саша», «Борис») в боевую галерею лиц.
"""

from __future__ import annotations

import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

_audio_common = types.ModuleType("audio_common_msgs")
_audio_common_msg = types.ModuleType("audio_common_msgs.msg")
_audio_common_msg.AudioData = MagicMock
sys.modules.setdefault("audio_common_msgs", _audio_common)
sys.modules.setdefault("audio_common_msgs.msg", _audio_common_msg)
for _hw in ("pyaudio", "usb", "usb.core", "usb.util", "sounddevice"):
    sys.modules.setdefault(_hw, MagicMock())
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from rob_box_core.speaker_wire import speaker_result_to_payload  # noqa: E402
from rob_box_voice import speaker_id_node as sid_node  # noqa: E402


class _Raw:
    def __init__(self) -> None:
        self.sent: list = []

    def publish(self, msg) -> None:
        self.sent.append(msg)


def _match(name: str) -> dict:
    return {"is_known": True, "speaker_id": "c8fd47c4", "name": name, "confidence": 0.99}


def test_every_result_is_stamped_with_current_mode():
    raw, mode = _Raw(), {"e2e": False}
    pub = sid_node._E2EStampedPublisher(raw, lambda: mode["e2e"])
    sid_node._publish_speaker_result(pub, _match("Денис"))
    mode["e2e"] = True
    sid_node._publish_speaker_result(pub, _match("Саша"))
    sid_node._publish_speaker_result(pub, {"event": "registered", "name": "Борис", "speaker_id": "b"})
    mode["e2e"] = False
    sid_node._publish_speaker_result(pub, _match("Денис"))
    assert [speaker_result_to_payload(m).get("e2e_db", False) for m in raw.sent] == [False, True, True, False]


def test_stamped_publisher_delegates_other_attributes():
    raw = _Raw()
    raw.get_subscription_count = lambda: 3
    assert sid_node._E2EStampedPublisher(raw, lambda: False).get_subscription_count() == 3
