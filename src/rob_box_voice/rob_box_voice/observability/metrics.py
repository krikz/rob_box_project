#!/usr/bin/env python3
"""Prometheus-метрики для диалоговой цепочки (issue #1160, этап 1).

Назначение модуля
-----------------
Дать видимость в реальную работу voice-assistant: STT → dialogue/LLM →
TTS. Сейчас observability — только логи в Loki, и ответ на вопрос
"почему робот замолчал на 3 минуты" сводится к ``grep``. После этого
этапа видно:

* сколько раз отработал каждый этап в час;
* p50/p95/p99 latency LLM-вызовов (deepseek vs minimax);
* сколько раз был fallback на Silero / Yandex (health провайдера);
* сколько раз speaker-recognition определил спикера;
* сколько раз был barge-in (прерывание TTS голосом).

Дизайн
------
* ``prometheus_client`` — optional dep. Если пакета нет (CI без него,
  юнит-тесты без метрик), все функции ниже — no-op.
* Метрики создаются лениво при первом обращении через
  :func:`get_metric`. Повторные вызовы возвращают тот же объект
  (через singleton-реестр модуля).
* HTTP-сервер запускается ровно один раз через
  :func:`start_metrics_server` (повторный вызов = no-op).

Имена метрик согласованы с issue #1160 §"Что измерять":

* ``voice_stt_recognize_total{provider, result}`` — counter
* ``voice_llm_request_total{provider, result}`` — counter (result ∈ success|fallback)
* ``voice_llm_request_duration_seconds{provider}`` — histogram
* ``voice_tts_synthesize_total{provider, result}`` — counter (result ∈ success|fail)
* ``voice_tts_synthesize_duration_seconds{provider}`` — histogram
* ``voice_speaker_recognize_total{result}`` — counter (result ∈ known|unknown)
* ``voice_barge_in_total`` — counter
* ``voice_session_duration_seconds{result}`` — histogram
  (result ∈ success|fail — что произошло в конце диалога)
* ``telegram_message_total{direction, type}`` — counter

Метрики W2-6 (issue #968, волна scheduler-segments-merge, фаза S12):

* ``voice_scheduler_quick_decide_total{verdict}`` — counter
  (verdict ∈ IGNORE|REPLACE|PENDING_LLM, см. quick_decide.py)
* ``voice_scheduler_task_updated_total`` — counter (применённая
  MERGE-правка — TaskScheduler.update()/replace_args())
* ``voice_scheduler_pending_queue_latency_seconds`` — histogram
  (задержка отложенной пользовательской фразы в очереди
  ``_pending_user_messages``, цель — ≤ 200 мс)

Все метрики живут в ``prometheus_client.REGISTRY`` (process-global).
Каждая ROS2-нода запускается в отдельном процессе, поэтому
пересечения имён между нодами нет, но ВНУТРИ процесса (unit-тесты
перезапускают ноду несколько раз) idempotent registration обязателен.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Dict, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from prometheus_client import Counter as _Counter  # noqa: F401
    from prometheus_client import Histogram as _Histogram  # noqa: F401

# prometheus_client — optional. Если его нет, no-op behaviour.
try:
    from prometheus_client import (
        CONTENT_TYPE_LATEST,
        REGISTRY,
        CollectorRegistry,
        Counter,
        Histogram,
        start_http_server,
    )
except ImportError:  # pragma: no cover — exercised via unit tests
    CONTENT_TYPE_LATEST = "text/plain; version=0.0.4; charset=utf-8"
    REGISTRY = None
    CollectorRegistry = None  # type: ignore[assignment,misc]
    Counter = None  # type: ignore[assignment,misc]
    Histogram = None  # type: ignore[assignment,misc]
    start_http_server = None  # type: ignore[assignment]


_log = logging.getLogger(__name__)


# ── Бакеты для гистограмм latency (LLM и TTS) ──────────────────────────
# Подобраны под типичные диапазоны:
# * LLM-вызовы: обычно 0.5–4s, p99 до 30s на cold-start и quota-exhausted
# * TTS: 0.1–2s на чанк, p99 до 8s на 16kHz WAV upload
DEFAULT_BUCKETS: tuple[float, ...] = (
    0.05, 0.1, 0.25, 0.5, 1.0, 2.5, 5.0, 10.0, 30.0, 60.0,
)


# ── Тип для no-op результата Histogram.observe() ────────────────────────
class HistogramValue:
    """No-op контекст-менеджер для гистограмм, когда ``prometheus_client`` недоступен.

    Используется так::

        with HistogramValue("voice_llm_request_duration_seconds", {"provider": "deepseek"}):
            response = await client.chat(...)

    С ``prometheus_client`` это нативный ``Histogram.time()``, без него —
    этот класс (измеряет время через ``time.monotonic``, но не
    публикует — оставлено на случай ручной публикации через
    :func:`record_*`).
    """

    __slots__ = ("_start",)

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        # Метрика недоступна — но мы всё равно засекаем время, чтобы
        # тесты могли проверить, что «вызов состоялся» без реального
        # счётчика.
        self._start: float = time.monotonic()

    def __enter__(self) -> "HistogramValue":
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        return None

    @property
    def elapsed(self) -> float:
        return time.monotonic() - self._start


class MetricsDisabled:
    """No-op singleton для Counter/Histogram, когда prometheus_client недоступен.

    Любой вызов ``inc()``, ``observe()``, ``labels(...)`` возвращает
    тот же объект (или self), не падает и не считает. Это позволяет
    прод-коду вызывать ``counter.labels(...).inc()`` без
    ``if prometheus_client is not None`` в каждом месте.
    """

    __slots__ = ()

    def labels(self, *args: Any, **kwargs: Any) -> "MetricsDisabled":
        return self

    def inc(self, amount: float = 1.0) -> None:
        return None

    def observe(self, amount: float) -> None:
        return None

    def time(self) -> HistogramValue:
        return HistogramValue()

    def __getattr__(self, name: str) -> Any:
        # Любой неожиданный атрибут = no-op.
        return self


# Если prometheus_client нет — тип "метрики" = MetricsDisabled.
_METRIC_TYPE: Any = Any  # prometheus_client.Counter / Histogram / MetricsDisabled


# ── Singleton-реестр метрик ─────────────────────────────────────────────
# Ключ — (имя метрики, тип). Значение — Counter/Histogram объект.
_METRIC_REGISTRY: Dict[str, _METRIC_TYPE] = {}
_METRIC_REGISTRY_LOCK = threading.Lock()
_HTTP_SERVER_STARTED: set[int] = set()
_HTTP_SERVER_LOCK = threading.Lock()


def is_metrics_enabled() -> bool:
    """Возвращает ``True`` если ``prometheus_client`` доступен."""
    return Counter is not None and Histogram is not None and start_http_server is not None


def get_metric(
    metric_type: str,
    name: str,
    documentation: str,
    labelnames: tuple[str, ...] = (),
    buckets: Optional[tuple[float, ...]] = None,
) -> _METRIC_TYPE:
    """Создаёт (или возвращает существующую) метрику.

    :param metric_type: ``"counter"`` или ``"histogram"``.
    :param name: имя метрики (snake_case, должно заканчиваться на
        ``_total`` для counter и ``_seconds``/``_bytes`` для histogram
        — стандартное соглашение Prometheus).
    :param documentation: описание для отладочной выдачи.
    :param labelnames: список допустимых label'ов.
    :param buckets: только для histogram; если ``None`` —
        :data:`DEFAULT_BUCKETS`.

    Поведение при отсутствии ``prometheus_client``: возвращает
    singleton-:class:`MetricsDisabled` — прод-код продолжает работать.
    """
    if not is_metrics_enabled():
        return MetricsDisabled()

    key = f"{metric_type}:{name}"
    with _METRIC_REGISTRY_LOCK:
        existing = _METRIC_REGISTRY.get(key)
        if existing is not None:
            return existing

        # Counter/Histogram тут гарантированно не None
        # (см. ``is_metrics_enabled`` выше). type: ignore нужен только
        # потому, что pyright/mypy не понимает optional-импорт через try/except.
        if metric_type == "counter":
            metric: _METRIC_TYPE = Counter(  # type: ignore[misc]
                name,
                documentation,
                labelnames=labelnames,
            )
        elif metric_type == "histogram":
            metric = Histogram(  # type: ignore[misc]
                name,
                documentation,
                labelnames=labelnames,
                buckets=buckets or DEFAULT_BUCKETS,
            )
        else:
            raise ValueError(
                f"Unknown metric_type={metric_type!r}; expected 'counter' or 'histogram'"
            )

        _METRIC_REGISTRY[key] = metric
        return metric


def start_metrics_server(port: int) -> bool:
    """Запускает ``prometheus_client.start_http_server(port)`` для метрик.

    Идемпотентно: повторный вызов с тем же портом — no-op (``True``).
    Каждый ROS2-нода вызывает ``start_metrics_server`` в ``__init__``;
    ROS2 обычно стартует одну ноду на процесс, поэтому port-bind
    случается ровно один раз. Для unit-тестов с несколькими
    нодами-инстансами — флаг ``_HTTP_SERVER_STARTED`` не даёт
    словить ``OSError: [Errno 98] Address already in use``.

    ВНИМАНИЕ: каждый ROS2-нода — отдельный процесс, но в проде мы
    стартуем по ОДНОМУ http-server'у на НОДУ. Поэтому если два
    разных node-экземпляра стартуют в одном процессе (юнит-тест)
    с одним портом, второй получит ``OSError`` и мы залогируем
    warning. Каждый ROS2-нода в проде использует СВОЙ порт
    (dialogue=9100, tts=9110, stt=9111, speaker_id=9112, audio=9113,
    telegram=9101). Промежутки в нумерации оставлены для будущих
    нод.

    :param port: TCP-порт.
    :return: ``True`` если сервер стартанул (или уже бежит),
        ``False`` если ``prometheus_client`` недоступен ИЛИ порт
        занят другим процессом/нодой.
    """
    if not is_metrics_enabled():
        return False

    with _HTTP_SERVER_LOCK:
        if port in _HTTP_SERVER_STARTED:
            return True
        try:
            # type: ignore[call-arg]  # start_http_server гарантированно
            # callable здесь (см. ``is_metrics_enabled``)
            start_http_server(port)  # type: ignore[misc]
        except OSError as exc:
            # Порт уже занят (другая нода в этом процессе или коллизия).
            # Не считаем это фатальной ошибкой: метрики всё равно
            # доступны по /metrics на той ноде, которая заняла порт.
            _log.warning(
                "prometheus_client.start_http_server(%d) failed: %s",
                port, exc,
            )
            # Не помечаем как started: возможно следующий вызов
            # попадёт в нормальную ситуацию. Но для типичного
            # сценария "порт занят коллегой" — просто лог.
            return False
        _HTTP_SERVER_STARTED.add(port)
        _log.info("Prometheus metrics server started on :%d/metrics", port)
        return True


# ── Помощники для прод-кода ─────────────────────────────────────────────
# Эти функции — основная точка контакта с прод-кодом. Каждая ROS2-нода
# зовёт соответствующую ``record_*`` один раз в нужный момент
# (вместо прямой работы с Counter/Histogram), чтобы:

# 1) централизовать имена метрик (легко ревьюить/менять);
# 2) не повторять boilerplate ``labels(...).inc()`` в каждом файле;
# 3) автоматически no-op'ить при отсутствии prometheus_client.

def record_stt_recognize(
    provider: str,
    *,
    success: bool,
    duration_s: Optional[float] = None,
) -> None:
    """Учёт одного STT-распознавания.

    :param provider: ``"yandex"`` или ``"vosk"`` (см. stt_node).
    :param success: ``True`` если результат непустой и валидный;
        ``False`` если STT вернул ``None``/empty/ошибку.
    :param duration_s: длительность распознавания (если хочется
        записывать histogram latency). По умолчанию ``None`` — не
        публикуем (этап 1 не требует STT-latency, см. issue #1160).
    """
    result_label = "success" if success else "empty"
    counter = get_metric(
        "counter",
        "voice_stt_recognize_total",
        "STT recognize calls (success or empty), labelled by provider.",
        labelnames=("provider", "result"),
    )
    counter.labels(provider=provider, result=result_label).inc()
    if duration_s is not None:
        hist = get_metric(
            "histogram",
            "voice_stt_recognize_duration_seconds",
            "STT recognize latency in seconds.",
            labelnames=("provider",),
        )
        hist.labels(provider=provider).observe(duration_s)


def record_voice_llm_request(
    provider: str,
    *,
    success: bool,
    fallback: bool,
    duration_s: float,
) -> None:
    """Учёт одного LLM-запроса от dialogue_node.

    :param provider: ``"minimax"`` / ``"deepseek"`` / ``"mimo"`` /
        ``"qwen"`` (любое имя, что вернёт ``_resolve_provider_chain``).
    :param success: ``True`` если LLM вернул ответ (любой, включая
        "пустой"). ``False`` — все попытки провалились (включая
        fallback-chain).
    :param fallback: ``True`` если primary упал и мы переключились на
        следующего в цепочке (для метки ``result="fallback"``).
        Идёт В ДОПОЛНЕНИЕ к success: ``result="fallback"`` означает
        «получили ответ, но от fallback» — отдельная метрика от
        обычных success/fail.
    :param duration_s: общее время (включая fallback-retry) от старта
        первого запроса до получения ответа.
    """
    counter = get_metric(
        "counter",
        "voice_llm_request_total",
        "Voice-assistant LLM requests, labelled by provider and result.",
        labelnames=("provider", "result"),
    )
    if fallback:
        result_label = "fallback"
    elif success:
        result_label = "success"
    else:
        result_label = "fail"
    counter.labels(provider=provider, result=result_label).inc()

    hist = get_metric(
        "histogram",
        "voice_llm_request_duration_seconds",
        "Voice-assistant LLM request latency (seconds), including fallback retries.",
        labelnames=("provider",),
    )
    hist.labels(provider=provider).observe(duration_s)


#: Бакеты для ``voice_llm_prompt_tokens``. Подобраны вокруг сегодняшнего
#: фиксированного префикса (~27 100 токенов на 02.09.2026: 16 240 схемы
#: инструментов + 10 838 мастер-промпт), чтобы сдвиг вниз от доменных
#: скиллов был виден по перцентилям, а не тонул в одном бакете.
PROMPT_TOKENS_BUCKETS: tuple[float, ...] = (
    2000, 4000, 8000, 12000, 16000, 20000, 24000, 28000, 32000, 40000,
    48000, 64000, float("inf"),
)


def record_llm_prompt_tokens(
    provider: str,
    *,
    tokens: int,
    skill: str = "none",
    estimated: bool = True,
) -> None:
    """Учёт размера ОДНОГО запроса к LLM в токенах.

    Зовётся на каждое обращение к провайдеру, включая каждую итерацию
    тул-цикла: ход с восемью итерациями стоит восьми промптов.

    :param provider: ``"minimax"`` / ``"deepseek"`` / ...
    :param tokens: размер промпта. Неположительные значения игнорируются —
        это признак того, что провайдер прислал мусор, и записывать его в
        гистограмму значит испортить перцентили.
    :param skill: имя активного доменного скилла (``"none"`` пока скиллы
        не включены). Именно эта метка отвечает на вопрос «сколько стоит
        ход композитора против хода плеера».
    :param estimated: ``True`` — число получено клиентской эвристикой,
        потому что провайдер не прислал usage. Метка позволяет не смешивать
        точные и оценочные измерения в одной выборке.
    """
    if tokens <= 0:
        return
    hist = get_metric(
        "histogram",
        "voice_llm_prompt_tokens",
        "Prompt size in tokens per LLM request, by provider, skill and source.",
        labelnames=("provider", "skill", "estimated"),
        buckets=PROMPT_TOKENS_BUCKETS,
    )
    hist.labels(
        provider=provider,
        skill=skill or "none",
        estimated="true" if estimated else "false",
    ).observe(tokens)


def record_skill_activation(
    skill: str,
    *,
    source: str,
) -> None:
    """Учёт одной активации доменного скилла.

    :param skill: имя скилла (``"none"`` — активации не было).
    :param source: ``"router"`` — сработал детерминированный
        пред-роутер; ``"llm"`` — домен пришлось грузить вызовом
        ``load_skill``, то есть роутер промахнулся; ``"miss"`` — LLM
        запросила несуществующий домен.

    Доля ``source="llm"`` и есть метрика промахов роутера (задача 3.7):
    если она растёт, роутер не покрывает реальные формулировки, и
    включать сужение каталога (Move B) рано.
    """
    counter = get_metric(
        "counter",
        "voice_skill_activation_total",
        "Domain-skill activations, labelled by skill and how it was chosen.",
        labelnames=("skill", "source"),
    )
    counter.labels(skill=skill or "none", source=source).inc()


def record_fallback(
    primary: str,
    fallback: str,
    *,
    reason: Optional[str] = None,
) -> None:
    """Учёт fallback'а с primary на fallback-провайдера.

    Отдельный counter от ``voice_llm_request_total{result=fallback}`` —
    этот счётчик инкрементируется ОДИН раз на сам факт переключения,
    а ``voice_llm_request_total`` — на КАЖДЫЙ успешный ответ (включая
    от fallback). Это позволяет отличить «сколько раз fallback
    понадобился» (этот) от «сколько раз ответ дал fallback» (тот).
    """
    counter = get_metric(
        "counter",
        "voice_llm_fallback_total",
        "Voice-assistant LLM fallback transitions, labelled by primary→fallback.",
        labelnames=("primary", "fallback", "reason"),
    )
    counter.labels(
        primary=primary,
        fallback=fallback,
        reason=reason or "unknown",
    ).inc()


def record_tts_synthesize(
    provider: str,
    *,
    success: bool,
    duration_s: Optional[float] = None,
) -> None:
    """Учёт одной попытки TTS-синтеза.

    :param provider: ``"minimax"`` / ``"yandex"`` / ``"silero"``.
    :param success: ``True`` если синтез вернул валидный аудио-блоб;
        ``False`` если провайдер упал (и мы переключились на следующего
        по цепочке).
    :param duration_s: длительность синтеза. По умолчанию ``None`` —
        не публикуем histogram (если caller не хочет; обычно хочет).
    """
    result_label = "success" if success else "fail"
    counter = get_metric(
        "counter",
        "voice_tts_synthesize_total",
        "TTS synthesize attempts, labelled by provider and result.",
        labelnames=("provider", "result"),
    )
    counter.labels(provider=provider, result=result_label).inc()
    if duration_s is not None:
        hist = get_metric(
            "histogram",
            "voice_tts_synthesize_duration_seconds",
            "TTS synthesize latency (seconds), labelled by provider.",
            labelnames=("provider",),
        )
        hist.labels(provider=provider).observe(duration_s)


def record_speaker_recognize(
    *,
    known: bool,
    confidence: Optional[float] = None,
) -> None:
    """Учёт одного speaker-recognition вызова.

    :param known: ``True`` если resemblyzer/Yandex определил спикера
        из профиля; ``False`` если вернул ``unknown``.
    :param confidence: опционально — наблюдаем как histogram для тюнинга
        порогов (см. issue #1160 §speaker-recognition).
    """
    result_label = "known" if known else "unknown"
    counter = get_metric(
        "counter",
        "voice_speaker_recognize_total",
        "Speaker recognition results (known or unknown).",
        labelnames=("result",),
    )
    counter.labels(result=result_label).inc()
    if confidence is not None:
        hist = get_metric(
            "histogram",
            "voice_speaker_recognize_confidence",
            "Speaker recognition confidence score (0..1).",
            labelnames=("result",),
        )
        hist.labels(result=result_label).observe(confidence)


def record_barge_in() -> None:
    """Учёт barge-in — пользователь прервал TTS голосом.

    Вызывается из audio_node/dialogue_node, когда wake-word детектируется
    в момент, когда TTS ещё говорит (issue #993).
    """
    counter = get_metric(
        "counter",
        "voice_barge_in_total",
        "Barge-in events — user interrupted TTS with wake word.",
    )
    counter.inc()


def record_session_duration(
    duration_s: float,
    *,
    result: str = "success",
) -> None:
    """Учёт длительности одной диалоговой сессии (от wake-word до goodbye).

    :param duration_s: длина сессии в секундах.
    :param result: ``"success"`` если сессия завершилась нормально
        (пользователь попрощался); ``"fail"`` если прервалась
        timeout'ом / ошибкой.
    """
    hist = get_metric(
        "histogram",
        "voice_session_duration_seconds",
        "Voice dialogue session duration (seconds), labelled by terminal result.",
        labelnames=("result",),
    )
    hist.labels(result=result).observe(duration_s)


def record_telegram_message(
    direction: str,
    *,
    message_type: str = "text",
) -> None:
    """Учёт telegram-сообщения.

    :param direction: ``"in"`` (входящее от юзера) или ``"out"``
        (исходящее от бота).
    :param message_type: ``"text"`` / ``"voice"`` / ``"command"`` /
        ``"callback"``. Для прод-классификации.
    """
    counter = get_metric(
        "counter",
        "telegram_message_total",
        "Telegram bot messages, labelled by direction and type.",
        labelnames=("direction", "type"),
    )
    counter.labels(direction=direction, type=message_type).inc()


# ── W2-6 (issue #968, scheduler-segments-merge, фаза S12) ──────────────
# Метрики MERGE-цепочки: quick_decide-вердикты (S4.1), применённые
# TaskScheduler.update()-правки (S3.2) и задержка очереди отложенных
# фраз (S7).


def record_quick_decide_verdict(verdict: str) -> None:
    """Учёт одного вердикта ``quick_decide`` (S4.1, barge_in_policy="classify").

    :param verdict: строковое значение :class:`QuickVerdict`
        (``"IGNORE"`` / ``"REPLACE"`` / ``"PENDING_LLM"``) — см.
        ``rob_box_voice.scheduler.quick_decide.QuickVerdict``. Дан как
        ``str``, а не enum, чтобы этот модуль не тянул зависимость на
        ``scheduler`` (наблюдатель не должен знать детали типа —
        только его строковое значение для Prometheus-лейбла).
    """
    counter = get_metric(
        "counter",
        "voice_scheduler_quick_decide_total",
        "quick_decide Level-1 verdicts, labelled by verdict "
        "(IGNORE|REPLACE|PENDING_LLM).",
        labelnames=("verdict",),
    )
    counter.labels(verdict=verdict).inc()


def record_task_updated() -> None:
    """Учёт одной применённой MERGE-правки (S3.2, ``TaskScheduler.update()``).

    Инкрементируется на стороне вызывающего слоя (``dialogue_node``),
    когда приходит lifecycle-событие ``"task.updated"`` от планировщика
    (``_Channel.replace_args`` — rewrite/replace op применился к ещё не
    стартовавшему сегменту). Планировщик (``task_scheduler.py``)
    намеренно НЕ импортирует ``observability`` напрямую — событие уже
    публикуется через существующий ``on_event``-колбэк
    (``TaskScheduler(on_event=...)``), тот же механизм, что используется
    для ``/harness/task_events`` (W7c), эта метрика — второй наблюдатель
    того же события.
    """
    counter = get_metric(
        "counter",
        "voice_scheduler_task_updated_total",
        "Applied TaskScheduler.update() edits (MERGE rewrite/replace "
        "ops on a not-yet-started segment).",
    )
    counter.inc()


#: Бакеты для ``voice_scheduler_pending_queue_latency_seconds`` —
#: подобраны вокруг целевого порога 200мс (S7, §4.7.3), а не общих
#: :data:`DEFAULT_BUCKETS` (те начинаются слишком грубо для суб-
#: секундной цели).
_PENDING_QUEUE_LATENCY_BUCKETS: tuple[float, ...] = (
    0.05, 0.1, 0.15, 0.2, 0.3, 0.5, 1.0, 2.0, 5.0, 10.0,
)


def record_pending_queue_latency(latency_s: float) -> None:
    """Учёт задержки одной фразы в очереди ``_pending_user_messages`` (S7).

    Задержка — от постановки в очередь (когда ``quick_decide`` вернул
    ``PENDING_LLM`` во время in-flight турна) до дренажа
    (``_drain_pending_user_messages``, когда предыдущий турн
    освобождает слот). Целевое значение — **≤ 200 мс** (S7, §4.7.3):
    если дренаж систематически превышает порог, значит турны копятся
    быстрее, чем LLM-цикл успевает их обрабатывать.

    :param latency_s: задержка в секундах (не мс).
    """
    hist = get_metric(
        "histogram",
        "voice_scheduler_pending_queue_latency_seconds",
        "Queue latency of deferred user phrases in _pending_user_messages "
        "(enqueue → drain), target <= 200ms.",
        buckets=_PENDING_QUEUE_LATENCY_BUCKETS,
    )
    hist.observe(latency_s)
