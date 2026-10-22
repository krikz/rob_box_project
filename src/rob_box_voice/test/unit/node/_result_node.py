"""Минимальная ``DialogueNode`` для вызова ``_handle_result`` без ROS (через ``object.__new__``).

Общий помощник тестов ответа хода: что реально ушло в ``/voice/dialogue/response``.
"""

from __future__ import annotations

from unittest.mock import MagicMock

from rob_box_voice.dialogue_node import DialogueNode


def make_result_node() -> DialogueNode:
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n._logger = logger
    n.get_logger = lambda: logger
    n._response_pub = MagicMock()
    n._state_pub = MagicMock()
    n._sound_trigger_pub = MagicMock()
    n._speech_client_pubs = {"cancel": MagicMock()}
    n._dj_mode_pub = None
    n._dsm = MagicMock()
    n._dsm.current_state = MagicMock()
    n._active_tg_chat_id = None
    n._effects = MagicMock()
    n._verbose_llm = False
    # Флаги оставшихся гуардов «заявил без тула» (#992 Bug E и др.).
    n._action_claim_retry_used = False
    n._retry_dispatched_in_turn = False
    n._run_task = None
    n._task_lock = MagicMock()
    n._startup_greeting_fired = False
    return n


def published_texts(node: DialogueNode) -> list[str]:
    """Тексты, реально опубликованные в ``/voice/dialogue/response``."""
    return [c.args[0].data for c in node._response_pub.publish.call_args_list]
