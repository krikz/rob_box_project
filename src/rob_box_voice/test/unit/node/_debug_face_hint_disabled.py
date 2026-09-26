"""Debug-тест для диагностики _handle_tentative_speaker."""

from __future__ import annotations

import threading
from unittest.mock import MagicMock

import pytest

from rob_box_harness.identity import MemoryIdentitySeam
from rob_box_harness.memory import InMemoryStore
from rob_box_voice.dialogue_node import DialogueNode


@pytest.mark.asyncio
async def test_debug_disabled_flag():
    """Распечатать, что произошло в _handle_tentative_speaker."""
    store = InMemoryStore()
    await store.init()
    seam = MemoryIdentitySeam(store)

    n = object.__new__(DialogueNode)
    n.get_logger = MagicMock(return_value=MagicMock())
    n._speaker_lock = threading.Lock()
    n._identity = seam
    n._pending_identity_hint = None
    n._tentative_states = {}
    n._tentative_session_state = MagicMock(
        side_effect=lambda full_sid: n._tentative_states.setdefault(
            full_sid, {"asked": False, "confirmed": False, "name": None}
        )
    )
    n._resolve_pending_tentative_answer = MagicMock()
    n._confirm_tentative_speaker = MagicMock(return_value="[Spkr:confirmed]")
    n._ask_tentative_identity = MagicMock()
    n._tag_tentative = MagicMock(return_value="[Speaker:tentative]")
    n._face_voice_hint_enabled = False
    n._face_voice_hint_window_sec = 30.0
    n._face_voice_hint_high_threshold = 0.78
    n._face_voice_hint_low_threshold = 0.65
    n._turn_self_intro = None

    sp = {
        "speaker_id": "c9e981cb",
        "tentative_kind": "single",
        "tentative_name": "Дэнчик",
        "tentative_conf": 0.791,
        "confidence": 0.791,
    }
    result = n._handle_tentative_speaker(sp, "привет", "utt-2")
    print("\nRESULT:", repr(result))
    print("tentative_states:", n._tentative_states)
    print("ask_called:", n._ask_tentative_identity.called)
    print("confirm_called:", n._confirm_tentative_speaker.called)
    print("tag_called:", n._tag_tentative.called)
    print("resolve_pending called:", n._resolve_pending_tentative_answer.called)
    print("session_state called:", n._tentative_session_state.called)
    print("session_state call_count:", n._tentative_session_state.call_count)
    assert True
