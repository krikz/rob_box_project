"""Реплика робота из сообщений диалога для памяти контекста восприятия (issue #3911).

У v2 нет топика «ответ», есть запрос речи - ``SpeechRequest`` агента ``personality`` в ``/voice/speech/request``.
Разбор JSON ``/voice/dialogue/response`` (dialogue_node v1) удалён вместе с подпиской (#4354 A2).

Образ perception (Main Pi) не содержит ``rob_box_core``, поэтому тема и агент - литералы; дрейф держит
``rob_box_core/test/test_voice_state.py``.
"""

from typing import Any

#: ``rob_box_core.speech_client.SPEECH_REQUEST_TOPIC``.
SPEECH_REQUEST_TOPIC = '/voice/speech/request'
#: ``rob_box_core.voice_state.VOICE_AGENT``: голос, с которым говорят люди (ТАРС, Telegram, служебная речь - не он).
ROBOT_VOICE_AGENT = 'personality'


def robot_said(msg: Any) -> str:
    """Текст реплики робота; ``''`` - сообщение не реплика робота.

    ``SpeechRequest`` (есть ``agent``): только ``personality`` и только ``commit`` (``commit=False`` - прогрев кэша
    синтеза, не речь).
    """
    # ADR-0160 §2.3: SpeechRequest.text - голый текст реплики, SSML на проводе нет.
    if getattr(msg, 'agent', None) != ROBOT_VOICE_AGENT or not msg.commit:
        return ''
    return (msg.text or '').strip()
