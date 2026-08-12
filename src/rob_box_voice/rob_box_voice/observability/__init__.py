"""Observability — Prometheus metrics for the voice assistant stack.

Issue #1160 — этап 1: минимальный набор метрик для диалоговой цепочки
(STT → dialogue/LLM → TTS) + telegram. Этап 2 (OpenTelemetry traces)
будет добавлен позже отдельной задачей.

Архитектурные принципы
-----------------------
* **Опциональная зависимость.** ``prometheus_client`` — optional. Если
  пакета нет в окружении (минимальный CI, юнит-тесты без метрик), все
  метрик-функции превращаются в no-op и ``start_metrics_server()``
  возвращает ``False``. Это позволяет тестам и продакшн-коду делить
  один и тот же импорт без ``try/except`` в каждом вызове.
* **Idempotent registration.** Все метрики живут в singleton-реестре
  модуля :mod:`rob_box_voice.observability.metrics`. Повторный импорт
  (например, из юнит-тестов с перезагрузкой) не падает с
  ``ValueError: Duplicated timeseries`` — мы возвращаем ранее
  созданный объект. Это критично, т.к. ROS2-нода живёт в отдельном
  процессе, но юнит-тесты перезапускают её в одном и том же процессе.
* **Lazy HTTP server.** ``start_metrics_server(port)`` запускает
  ``prometheus_client.start_http_server`` ровно один раз — повторные
  вызовы возвращают ``True`` (идемпотентно) и не плодят лишние
  WSGI-серверы, которые роняют порт-бинды в ``OSError``.
* **Имена метрик согласованы с issue #1160 §"Что измерять".**

Endpoints (per issue #1160)
---------------------------
Каждый ROS2-нода — отдельный процесс, поэтому биндит СВОЙ порт:

* ``9100`` — dialogue_node (LLM latency / fallback counter)
* ``9110`` — tts_node (TTS latency / provider fallback)
* ``9111`` — stt_node (recognize counter)
* ``9112`` — speaker_id_node (recognize counter)
* ``9113`` — audio_node (barge-in, session_duration)
* ``9101`` — telegram-bot (message counter)

Музыкальная нода (supercollider) — ``9102`` — будет добавлена
отдельной задачей, когда ``rob_box_music`` появится в репо.
Prometheus scrape config (см. ``docker/monitoring/config/prometheus.yml``)
должен включать каждый из этих endpoint'ов.
"""

from .metrics import (
    DEFAULT_BUCKETS,
    HistogramValue,
    MetricsDisabled,
    get_metric,
    is_metrics_enabled,
    record_barge_in,
    record_fallback,
    record_session_duration,
    record_speaker_recognize,
    record_stt_recognize,
    record_telegram_message,
    record_tts_synthesize,
    record_voice_llm_request,
    start_metrics_server,
)

__all__ = [
    "DEFAULT_BUCKETS",
    "HistogramValue",
    "MetricsDisabled",
    "get_metric",
    "is_metrics_enabled",
    "record_barge_in",
    "record_fallback",
    "record_session_duration",
    "record_speaker_recognize",
    "record_stt_recognize",
    "record_telegram_message",
    "record_tts_synthesize",
    "record_voice_llm_request",
    "start_metrics_server",
]
