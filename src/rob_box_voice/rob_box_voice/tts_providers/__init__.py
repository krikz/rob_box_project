"""Адаптеры голосовых сервисов на ``BaseTTSProvider`` (ADR-0160 §2.5, #4240).

Каждый адаптер сам готовит голый текст реплики под свой сервис (:meth:`prepare`) и синтезирует его. ROS-нода
``tts_node`` только выбирает провайдера по цепочке, публикует состояние и играет звук. Сырые вызовы сервисов
(запрос gRPC Yandex, ``apply_tts`` Silero) — в ``tts_engines``, их же зовёт вокал песни ``sing_synth``.

MiniMax — адаптер ``rob_box_llm.providers.minimax_tts.MiniMaxTTSProvider``; паузы такта рэпа для него размечает
:func:`.beat.minimax_text` (пакеты робота, которых ``rob_box_llm`` не видит), остальным — :func:`.beat.without_beat`.
"""

from .silero import SileroTTSProvider
from .yandex import YandexTTSProvider

__all__ = ["SileroTTSProvider", "YandexTTSProvider"]
