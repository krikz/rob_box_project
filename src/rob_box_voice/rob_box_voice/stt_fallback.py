#!/usr/bin/env python3
"""
stt_fallback — Pure-Python логика STT-fallback (issue #979).

Этот модуль НЕ импортирует rclpy/vosk/grpc — только stdlib. Это позволяет
тестировать retry/timeout/short-phrase/metric-логику без ROS2 окружения
(см. ``test/test_stt_fallback.py``).

Контекст issue #979
-------------------
Раньше после TTS-фразы STT падал на Vosk fallback через 1.4 секунды,
потому что Yandex STT не успевал ответить. Vosk 0.42 (small-ru-0.22)
возвращал мусор ("а а а") вместо русских фраз, и робот молчал на
"расскажи ещё раз сразу".

Что фиксим
-----------
1. ``select_recognition`` — обёрнутый Yandex-вызов с
   ``timeout_s`` (по умолчанию 5.0s, было ~1.3s) и ровно одним retry
   перед тем как упасть на Vosk.
2. ``is_short_phrase`` — единое правило отклонения коротких фраз
   (>= min_chars символов; по умолчанию 3). VOSK-мусор обычно 1-2 chars.
3. ``STTAttempt``/``FallbackReason`` — типизированный результат для
   метрики в логе. Метрика: ``stt_attempt{provider,reason,latency_ms}``.
4. ``summarize_attempts`` — превращает список попыток в строку
   ``"yandex:timeout(4200ms)->vosk:low_confidence(180ms)"`` для логов.

Параметры настраиваются через ROS2 params в ``stt_node.py`` или
напрямую через kwargs в тестах.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Callable, List, Literal, Optional, Protocol, Sequence

# Тонкая типизация для метрик/логирования. Эти значения попадают
# в логи оператора и должны быть стабильными (Prometheus/dashboard парсинг).
FallbackReason = Literal[
    "ok",  # распознали успешно
    "timeout",  # превышен timeout_s у Yandex
    "empty",  # провайдер вернул None/пустую строку
    "error",  # исключение/grpc.RpcError
    "low_confidence",  # слишком короткий результат (< min_chars)
    "rejected_short",  # итоговое отклонение — слишком короткая итоговая фраза
    "no_provider",  # нет ни одного доступного провайдера
]

# Минимальная разумная длина итоговой фразы. Vosk 0.42 возвращает
# "а", "а а" для эха — это мусор, а не команда. 3 символа достаточно
# чтобы отсечь "а"/"да"/"ага"/"э" и при этом принять "да"/"нет"/"стоп".
DEFAULT_MIN_TEXT_CHARS = 3

# Таймаут одного вызова Yandex STT (был 1.3s в старом коде, спрятанный
# внутри grpc-запроса). 5 секунд — эмпирика для фраз 3-4 слов при
# audio_processing_type=REAL_TIME.
DEFAULT_YANDEX_TIMEOUT_S = 5.0

# Количество retry-повторов на Yandex перед падением на Vosk.
# 1 — спецификация issue #979: "один retry перед падением на Vosk".
DEFAULT_YANDEX_MAX_RETRIES = 1

# Backoff между retry (множитель экспоненциальный, секунды).
# Issue #979: «попробовать ещё раз через 1с (single retry)».
DEFAULT_RETRY_BACKOFF_S = 1.0


class STTProvider(Protocol):
    """Минимальный интерфейс провайдера для select_recognition().

    Конкретные реализации (Yandex STT gRPC, Vosk) живут в stt_node.py и
    просто передаются как объекты с методом ``recognize(audio_bytes) -> str | None``.
    Провайдер ОБЯЗАН вернуть ``None`` или пустую строку, если не смог
    распознать (timeout/error маппятся нами в ``FallbackReason``).
    """

    name: str

    def recognize(self, audio_bytes: bytes) -> Optional[str]:  # pragma: no cover
        ...


class STTTimeoutError(TimeoutError):
    """Провайдер превысил свой внутренний deadline.

    Поднимается адаптером (например, ``stt_node._recognize_yandex`` при
    gRPC ``DEADLINE_EXCEEDED``), чтобы ``select_recognition`` классифицировал
    попытку как ``timeout``, а не как ``error``.
    """


@dataclass
class STTAttempt:
    """Одна попытка распознавания — для метрик/логирования."""

    provider: str
    reason: FallbackReason
    latency_ms: int
    text: Optional[str] = None
    error: Optional[str] = None
    attempt_index: int = 0  # 0..max_retries

    @property
    def ok(self) -> bool:
        return self.reason == "ok" and bool(self.text)

    def to_log_dict(self) -> dict:
        """Словарь для structured-logging (ключи стабильны)."""
        out = {
            "provider": self.provider,
            "reason": self.reason,
            "latency_ms": self.latency_ms,
            "attempt": self.attempt_index,
        }
        if self.text is not None:
            out["text"] = self.text
        if self.error is not None:
            out["error"] = self.error
        return out


def is_short_phrase(text: Optional[str], min_chars: int = DEFAULT_MIN_TEXT_CHARS) -> bool:
    """Слишком короткая фраза — отклонить.

    ``min_chars`` — порог после ``strip()``. Принимаем None как короткую фразу.
    """
    if text is None:
        return True
    return len(text.strip()) < min_chars


def _measure(call: Callable[[], Optional[str]], timeout_s: float) -> tuple[Optional[str], int, Optional[str]]:
    """Запустить ``call`` с жёстким soft-timeout.

    Возвращает ``(text, latency_ms, error_message)``. Timeout НЕ прерывает
    сам call (gRPC stub не прерываем), но даёт сигнал retry/timeout-reason.

    Для настоящей отмены нужен был бы отдельный поток с cancel-token —
    Yandex gRPC v3 сам по себе не поддерживает cancellation в середине
    streaming-recognition. Поэтому 5s timeout — это допустимое окно:
    даже если запрос "висит", новый он не блокирует (gRPC channel
    async).
    """
    started = time.monotonic()
    try:
        result = call()
    except STTTimeoutError:
        # Провайдер сам сообщил о превышении deadline (gRPC DEADLINE_EXCEEDED).
        elapsed_ms = int((time.monotonic() - started) * 1000)
        return None, elapsed_ms, f"timeout>{timeout_s}s"
    except Exception as exc:  # noqa: BLE001 — мы хотим все ошибки здесь
        elapsed_ms = int((time.monotonic() - started) * 1000)
        return None, elapsed_ms, repr(exc)[:200]
    elapsed_ms = int((time.monotonic() - started) * 1000)
    if elapsed_ms > timeout_s * 1000:
        # Возможно вернулся какой-то текст, но поздно — фиксируем timeout.
        return None, elapsed_ms, f"timeout>{timeout_s}s"
    return result, elapsed_ms, None


def select_recognition(
    providers: Sequence[STTProvider],
    audio_bytes: bytes,
    *,
    timeout_s: float = DEFAULT_YANDEX_TIMEOUT_S,
    max_retries: int = DEFAULT_YANDEX_MAX_RETRIES,
    retry_backoff_s: float = DEFAULT_RETRY_BACKOFF_S,
    min_text_chars: int = DEFAULT_MIN_TEXT_CHARS,
) -> tuple[Optional[str], List[STTAttempt]]:
    """Запустить цепочку провайдеров с retry на первом.

    Логика:
        * Берём первый провайдер (обычно Yandex).
        * Делаем max_retries+1 попыток с soft-timeout ``timeout_s`` каждая.
        * Если все упали/вернули пусто/timeout — переходим к следующему.
        * Возвращаем ``(text, attempts)`` — text это первый не-пустой
          не-короткий результат, attempts — ВСЕ попытки для метрик.

    Args:
        providers: Непустая последовательность. [0] — primary, [1..] — fallback.
        audio_bytes: PCM int16 LE mono 16kHz.
        timeout_s: Per-call timeout Yandex (sec).
        max_retries: Количество retry на primary (1 = один retry).
        retry_backoff_s: Линейный backoff (в текущей версии константа).
        min_text_chars: Порог ``is_short_phrase``.

    Returns:
        ``(text, attempts)``. ``text`` может быть None — итоговое отклонение.
    """
    if not providers:
        raise ValueError("providers sequence must be non-empty")

    attempts: List[STTAttempt] = []

    for provider_idx, provider in enumerate(providers):
        is_fallback = provider_idx > 0
        # Провайдер после primary (Vosk) — без retry: если мусор, retry
        # только усугубит. Вместо этого сразу идём к следующему.
        attempts_left = max_retries if not is_fallback else 0
        total_attempts = attempts_left + 1

        for attempt_idx in range(total_attempts):
            if attempt_idx > 0:
                # Линейный backoff (для 1 retry — один sleep).
                time.sleep(retry_backoff_s)

            text, latency_ms, error = _measure(
                lambda p=provider: p.recognize(audio_bytes),
                timeout_s=timeout_s,
            )

            # Классификация причины
            if error is not None and error.startswith("timeout>"):
                reason: FallbackReason = "timeout"
                text = None
            elif error is not None:
                reason: FallbackReason = "error"
                text = None
            elif text is None or not text.strip():
                # Провайдер вернул пустоту — это отдельная причина (не
                # low_confidence, который означает «распознали, но слишком
                # коротко»). Vosk 0.42 для эха возвращает "а" — это
                # low_confidence, а не empty.
                reason: FallbackReason = "empty"
            elif is_short_phrase(text, min_chars=min_text_chars):
                reason = "low_confidence"
                # text оставляем как есть для лога, но он слишком короткий
            else:
                reason = "ok"

            attempts.append(
                STTAttempt(
                    provider=provider.name,
                    reason=reason,
                    latency_ms=latency_ms,
                    text=text if text is not None else None,
                    error=error,
                    attempt_index=attempt_idx,
                )
            )

            if reason == "ok":
                # Успех — дальше провайдеры не нужны.
                return text, attempts

            # Иначе: retry/next provider. retry даём только primary.
            if is_fallback or attempt_idx == attempts_left:
                # Если последний retry для primary или мы на fallback —
                # переходим к следующему провайдеру.
                break

    # Сюда дошли, если ни один провайдер не дал "ok".
    # Последний "low_confidence" — это мягкое отклонение Vosk-мусора.
    return None, attempts


def summarize_attempts(attempts: Sequence[STTAttempt]) -> str:
    """Краткое текстовое представление для логов оператора.

    Пример:
        ``yandex:timeout(4200ms)->vosk:low_confidence(180ms)``
        ``yandex:ok(820ms)``
        ``yandex:error(1300ms)``
    """
    parts: list[str] = []
    for a in attempts:
        text_suffix = ""
        if a.text:
            text_suffix = f" '{a.text[:20]}'"
        parts.append(f"{a.provider}:{a.reason}({a.latency_ms}ms{text_suffix})")
    return "->".join(parts)


def log_attempts(
    logger: logging.Logger,
    attempts: Sequence[STTAttempt],
    *,
    final_text: Optional[str],
) -> None:
    """Один структурный лог + читаемая строка.

    Метрика ``stt_attempt`` соответствует Prometheus convention:
    ``provider="yandex" reason="timeout" latency_ms="4200"``.
    Имя logger'а — стабильное, можно grep-ать / парсить.
    """
    summary = summarize_attempts(attempts)
    # Финальный лог — одна строка с итогом.
    if final_text:
        logger.info(f"[stt_attempt] {summary} -> accepted '{final_text}'")
    else:
        logger.warning(f"[stt_attempt] {summary} -> rejected")
    # Каждую попытку — отдельной строкой для парсинга/dashboard.
    for a in attempts:
        logger.info(
            "[stt_attempt_metric] provider=%s reason=%s latency_ms=%d attempt=%d text=%s",
            a.provider,
            a.reason,
            a.latency_ms,
            a.attempt_index,
            f"'{a.text[:30]}'" if a.text else "-",
        )
