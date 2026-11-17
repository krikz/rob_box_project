"""Речь супервизора для оператора: реплика ТАРС в шлем и образец голоса (ADR-0055, ADR-0079, #4252).

* :func:`say_in_headset` — собственная реплика ТАРС в наушники шлема: ``SpeechRequest(agent=operator,
  sink=headset)`` владельцу речи (``tts_node``). Он синтезирует её в ``/avatar/tts/audio`` и отвечает
  ``SpeechEvent`` с тем же ``speech_id``; ``quest_node`` по этому ``speech_id`` доставляет звук в шлем и
  закрывает стадию ТАРС. До #4252 реплика шла String JSON в ``/avatar/tts/request`` мимо владельца речи.
* :func:`publish_preview` — «прослушиваемый образец» голоса для picker'а: ``/avatar/tts/request`` с
  ``sink="preview"``; tts_node делает чистый синтез без FIFO/ALSA/шлема и отдаёт байты в
  ``/avatar/preview_voice/audio`` (см. preview_audio_sink.ts §1).

Пустой текст — DROP с предупреждением (#2096): синтезировать нечего, провайдер упал бы на пустом запросе.
Модуль, а не методы ``AvatarSupervisor``: класс-бюджет ADR-0145.
"""

from __future__ import annotations

import json
from typing import Any, Optional

from rob_box_core import speech_client
from rob_box_core.utterance import Sink, Utterance

#: Автор реплик ТАРС в ``SpeechRequest`` (``speech_client.SPEECH_AGENTS``).
OPERATOR_AGENT = "operator"


def say_in_headset(node: Any, text: str, log: Any) -> str:
    """Реплика ТАРС в шлем; вернуть ``speech_id`` (``""`` — реплика не ушла)."""
    if not text or not text.strip():
        log.warning("avatar_supervisor: реплика в шлем пропущена — пустой текст")
        return ""
    try:
        return speech_client.say(node, Utterance(text=text, sink=Sink.HEADSET), agent=OPERATOR_AGENT)
    except Exception as exc:  # noqa: BLE001 — ROS-сбой публикации не роняет ход оператора
        log.warning(f"avatar_supervisor: SpeechRequest в шлем не ушёл: {exc}")
        return ""


def publish_preview(pub: Any, msg_type: Any, text: str, voice: Optional[str], request_id: str, log: Any) -> str:
    """Образец голоса ``voice`` в ``/avatar/tts/request``; вернуть ``request_id`` (``""`` — пустой текст).

    ``request_id`` пришёл от quest_node через ``/avatar/preview_voice`` — по нему ws_server сводит ответ
    tts_node с picker'ом.
    """
    if not text or not text.strip():
        log.warning(f"avatar_supervisor: preview_tts_request skipped — empty text (voice={voice!r})")
        return ""
    # voice-vr 12 (issue #2197): единый сборщик реплики — ``Utterance`` (голый текст, ADR-0160 §2.3).
    payload = {"request_id": request_id, **Utterance(text=text, sink=Sink.PREVIEW, voice=voice).to_request()}
    try:
        msg = msg_type()
        msg.data = json.dumps(payload, ensure_ascii=False)
        pub.publish(msg)
    except Exception as exc:  # noqa: BLE001
        log.warning(f"avatar_supervisor: preview_tts_request publish failed: {exc}")
    return request_id
