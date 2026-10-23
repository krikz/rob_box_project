"""Реплика робота из сообщений диалога для памяти контекста восприятия (issue #3911).

dialog_engine=v2: у v2 нет топика «ответ», есть запрос речи - ``SpeechRequest`` агента ``personality`` в
``/voice/speech/request``. Удалит PR-15 #3711: dialogue_node v1 пишет JSON с SSML в
``/voice/dialogue/response`` (``std_msgs/String``) - тот же разбор, чтобы узел держал один обработчик.

Образ perception (Main Pi) не содержит ``rob_box_core``, поэтому тема и агент - литералы; дрейф держит
``rob_box_core/test/test_voice_state.py``.
"""

import json
from typing import Any

#: ``rob_box_core.speech_client.SPEECH_REQUEST_TOPIC``.
SPEECH_REQUEST_TOPIC = '/voice/speech/request'
#: ``rob_box_core.voice_state.VOICE_AGENT``: голос, с которым говорят люди (ТАРС, Telegram, служебная речь - не он).
ROBOT_VOICE_AGENT = 'personality'


def _plain(ssml: str) -> str:
    return ssml.replace('<speak>', '').replace('</speak>', '').strip()


def robot_said(msg: Any) -> str:
    """Текст реплики робота; ``''`` - сообщение не реплика робота.

    ``SpeechRequest`` (есть ``agent``): только ``personality`` и только ``commit`` (``commit=False`` - прогрев кэша
    синтеза, не речь). ``String`` v1: JSON с ``ssml`` или голый текст.
    """
    agent = getattr(msg, 'agent', None)
    if agent is not None:
        return _plain(msg.text) if agent == ROBOT_VOICE_AGENT and msg.commit else ''
    try:
        return _plain(json.loads(msg.data).get('ssml', ''))
    except Exception:
        return _plain(msg.data)
