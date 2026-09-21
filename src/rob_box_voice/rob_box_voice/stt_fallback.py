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
import threading
import time
from dataclasses import dataclass
from typing import (
    Callable,
    List,
    Literal,
    Mapping,
    Optional,
    Protocol,
    Sequence,
)

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
    "dead",  # провайдер в кэше «мёртвых» (квота/ключ/сеть), пропущен
]

# Минимальная разумная длина итоговой фразы. Vosk 0.42 возвращает
# "а", "а а" для эха — это мусор, а не команда. 3 символа достаточно
# чтобы отсечь "а"/"да"/"ага"/"э" и при этом принять "да"/"нет"/"стоп".
DEFAULT_MIN_TEXT_CHARS = 3

# Таймаут одного вызова Yandex STT (был 1.3s в старом коде, спрятанный
# внутри grpc-запроса). 5.0s → 12.0s (issue #1477) — фразы 4-6 секунд с
# pre-roll и активным TTS/музыкой (issue 989) могут выходить за 5с gRPC
# deadline в Yandex v3 REAL_TIME + speech_analysis. 12с — буфер для
# длинных фраз и медленного EOU (patient=2000ms). Лимит выбран
# эмпирически: probe на 10.1.1.21 показывает 800-1300ms на нормальных
# фразах и ~480ms на FULL_DATA.
DEFAULT_YANDEX_TIMEOUT_S = 12.0

# Количество retry-повторов на Yandex перед падением на Vosk.
# 1 — спецификация issue #979: "один retry перед падением на Vosk".
DEFAULT_YANDEX_MAX_RETRIES = 1

# Backoff между retry (множитель экспоненциальный, секунды).
# Issue #979: «попробовать ещё раз через 1с (single retry)».
DEFAULT_RETRY_BACKOFF_S = 1.0

# TTL кэша «мёртвых» провайдеров для НЕвосстановимых отказов — кончилась
# квота (402/429) или протух ключ (401/403). Такое не «рассосётся» за
# секунды, поэтому провайдер выпадает из цепочки на 5 минут. Значение
# совпадает с TTS (``tts_node.provider_dead_ttl_s``) и LLM
# (``rob_box_harness.health.DEFAULT_HEALTH_TTL_S``) — один класс проблемы,
# один порядок величины.
DEFAULT_DEAD_TTL_S = 300.0

# TTL для транзиентных отказов (сеть, 5xx, таймаут). Короткий: сеть могла
# моргнуть, наказывать провайдера на 5 минут за один DEADLINE_EXCEEDED —
# перебор. Совпадает с ``tts_node.provider_dead_ttl_transient_s``.
DEFAULT_DEAD_TTL_TRANSIENT_S = 30.0


class STTProvider(Protocol):
    """Минимальный интерфейс провайдера для select_recognition().

    Конкретные реализации (Yandex STT gRPC, Vosk) живут в stt_node.py и
    просто передаются как объекты с методом ``recognize(audio_bytes) -> str | None``.
    Провайдер ОБЯЗАН вернуть ``None`` или пустую строку, если не смог
    распознать (timeout/error маппятся нами в ``FallbackReason``).

    Необязательный метод ``prepare()`` вызывается один раз, когда очередь
    дошла до провайдера, и НЕ входит в ``timeout_s`` — там лениво грузится
    модель (Vosk, issue #2609), иначе первая фраза после отказа облака
    отбрасывалась по таймауту.
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


class STTAuthError(Exception):
    """Провайдер отказал по ключу: HTTP 401/403, gRPC UNAUTHENTICATED.

    Не транзиентная ошибка: пока ключ не заменят, каждый следующий вызов
    вернёт то же самое. ``select_recognition`` не ретраит такую попытку и
    помечает провайдера мёртвым на длинный TTL
    (:data:`DEFAULT_DEAD_TTL_S`).
    """


class STTQuotaError(Exception):
    """У провайдера кончились деньги/квота: HTTP 402/429, «limit reached».

    Ровно тот случай, ради которого заведён кэш «мёртвых»: 21.09 у нас
    одновременно лежали MiniMax и Yandex (не пополнен баланс), и без
    кэша нода на КАЖДОЙ фразе честно ждала оба облака по таймауту,
    прежде чем дойти до Vosk. Длинный TTL (:data:`DEFAULT_DEAD_TTL_S`) —
    баланс не появляется за 30 секунд.
    """


def is_permanent_failure(exc: Optional[BaseException]) -> bool:
    """True, если отказ не «рассосётся» сам (квота/ключ) — длинный TTL."""
    return isinstance(exc, (STTAuthError, STTQuotaError))


@dataclass(frozen=True)
class ProviderPolicy:
    """Per-provider retry/timeout-бюджет для :func:`select_recognition`.

    До issue #2365 Phase 2 бюджет был ОДИН на всю цепочку: ``timeout_s``
    и ``max_retries`` в сигнатуре ``select_recognition``, причём retry
    доставались только ``providers[0]``. С тремя провайдерами это
    неверно: MiniMax укладывается в 5с, Yandex — в 12с (issue #1477),
    а Vosk офлайновый и таймаут ему не нужен вовсе.

    Attributes:
        timeout_s: soft-timeout одного вызова ``recognize()``.
        max_retries: сколько ПОВТОРОВ (0 = одна попытка).
        retry_backoff_s: пауза между повторами.
    """

    timeout_s: float = DEFAULT_YANDEX_TIMEOUT_S
    max_retries: int = 0
    retry_backoff_s: float = DEFAULT_RETRY_BACKOFF_S


class ProviderDeadCache:
    """Кэш «мёртвых» STT-провайдеров с TTL.

    Прямой аналог того, что уже есть в двух других доменах:

    * TTS — ``tts_node._provider_dead_until`` / ``_mark_provider_dead``
      (issue #1083): квота MiniMax → провайдер пропускается 5 минут;
    * LLM — :class:`rob_box_harness.health.HealthCache` (issue #1082):
      ``unavailable`` провайдер не попадает в цепочку до истечения TTL.

    Зачем здесь. Без кэша цепочка ``minimax → yandex → vosk`` на мёртвых
    облаках стоит 5с (MiniMax timeout) + 12с×2 (Yandex timeout + retry)
    на КАЖДОЙ фразе, и только потом доходит до Vosk. С кэшем это
    случается один раз, дальше — сразу Vosk, пока TTL не истечёт и мы
    не проверим облака снова.

    Класс намеренно stdlib-only (как весь модуль) — тестируется без ROS.
    Потокобезопасность обеспечивается ``threading.Lock``: ``stt_node``
    зовёт распознавание из callback-потока, а persist — из таймера.
    """

    def __init__(
        self,
        *,
        ttl_s: float = DEFAULT_DEAD_TTL_S,
        transient_ttl_s: float = DEFAULT_DEAD_TTL_TRANSIENT_S,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._ttl_s = float(ttl_s)
        self._transient_ttl_s = float(transient_ttl_s)
        self._clock = clock
        self._lock = threading.Lock()
        self._dead_until: dict[str, float] = {}
        self._reason: dict[str, str] = {}

    # -- запросы ---------------------------------------------------------

    def is_dead(self, provider: str) -> bool:
        """True, пока TTL провайдера не истёк."""
        with self._lock:
            return self._clock() < self._dead_until.get(provider, 0.0)

    def remaining_s(self, provider: str) -> float:
        """Сколько секунд провайдеру осталось «лежать» (для логов)."""
        with self._lock:
            return max(0.0, self._dead_until.get(provider, 0.0) - self._clock())

    def reason(self, provider: str) -> Optional[str]:
        """Почему провайдер помечен мёртвым (для логов/диагностики)."""
        with self._lock:
            return self._reason.get(provider)

    # -- мутации ---------------------------------------------------------

    def mark_alive(self, provider: str) -> None:
        """Провайдер ответил — снимаем отметку (квоту пополнили)."""
        with self._lock:
            self._dead_until.pop(provider, None)
            self._reason.pop(provider, None)

    def mark_dead(
        self,
        provider: str,
        reason: str,
        *,
        transient: bool = True,
        ttl_s: Optional[float] = None,
    ) -> float:
        """Пометить провайдера мёртвым. Возвращает применённый TTL.

        ``transient=True`` (сеть/5xx/таймаут) → короткий TTL;
        ``transient=False`` (квота/ключ) → длинный.
        """
        if ttl_s is None:
            ttl_s = self._transient_ttl_s if transient else self._ttl_s
        with self._lock:
            self._dead_until[provider] = self._clock() + float(ttl_s)
            self._reason[provider] = str(reason)[:300]
        return float(ttl_s)

    # -- персистентность (wall-clock, как в tts_node) ---------------------

    def snapshot_wall(self, now_wall: Optional[float] = None) -> dict:
        """Живые записи в wall-clock — для записи в JSON-файл.

        Формат совпадает с ``tts_node._persist_provider_state``
        (``dead_providers: {name: epoch_ts}``), чтобы объединение кэшей
        STT/TTS/LLM (issue #2702) свелось к смене пути, а не формата.
        """
        now_wall = time.time() if now_wall is None else now_wall
        out: dict = {}
        with self._lock:
            now_mono = self._clock()
            for provider, until_mono in self._dead_until.items():
                remaining = until_mono - now_mono
                if remaining > 0:
                    out[provider] = now_wall + remaining
        return out

    def restore_wall(
        self,
        dead: Mapping[str, object],
        *,
        now_wall: Optional[float] = None,
    ) -> list:
        """Восстановить кэш из wall-clock снимка. Возвращает имена живых записей.

        Просроченные записи игнорируются — провайдер получает шанс «ожить»
        (та же логика, что в ``tts_node._load_persisted_provider_state``).
        """
        now_wall = time.time() if now_wall is None else now_wall
        restored: list = []
        for provider, until_ts in dead.items():
            try:
                until = float(until_ts)  # type: ignore[arg-type]
            except (TypeError, ValueError):
                continue
            if until <= now_wall:
                continue
            with self._lock:
                self._dead_until[str(provider)] = self._clock() + (until - now_wall)
            restored.append(str(provider))
        return restored


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


def _measure(
    call: Callable[[], Optional[str]], timeout_s: float
) -> tuple[Optional[str], int, Optional[str], Optional[BaseException]]:
    """Запустить ``call`` с жёстким soft-timeout.

    Возвращает ``(text, latency_ms, error_message, exc)``. ``exc`` нужен
    вызывающему, чтобы отличить «кончилась квота» (:class:`STTQuotaError`,
    длинный TTL в :class:`ProviderDeadCache`) от «моргнула сеть» (короткий
    TTL) — по одной только строке ошибки это решать нельзя.
    Timeout НЕ прерывает сам call (gRPC stub не прерываем), но даёт сигнал
    retry/timeout-reason.

    Для настоящей отмены нужен был бы отдельный поток с cancel-token —
    Yandex gRPC v3 сам по себе не поддерживает cancellation в середине
    streaming-recognition. Поэтому 5s timeout — это допустимое окно:
    даже если запрос "висит", новый он не блокирует (gRPC channel
    async).
    """
    started = time.monotonic()
    try:
        result = call()
    except STTTimeoutError as exc:
        # Провайдер сам сообщил о превышении deadline (gRPC DEADLINE_EXCEEDED).
        elapsed_ms = int((time.monotonic() - started) * 1000)
        return None, elapsed_ms, f"timeout>{timeout_s}s", exc
    except Exception as exc:  # noqa: BLE001 — мы хотим все ошибки здесь
        elapsed_ms = int((time.monotonic() - started) * 1000)
        return None, elapsed_ms, repr(exc)[:200], exc
    elapsed_ms = int((time.monotonic() - started) * 1000)
    if elapsed_ms > timeout_s * 1000:
        # Возможно вернулся какой-то текст, но поздно — фиксируем timeout.
        return None, elapsed_ms, f"timeout>{timeout_s}s", None
    return result, elapsed_ms, None, None


def _prepare_provider(provider: STTProvider) -> None:
    """Вызвать необязательный ``provider.prepare()`` вне soft-timeout."""
    prepare = getattr(provider, "prepare", None)
    if not callable(prepare):
        return
    try:
        prepare()
    except Exception:  # recognize() сам решит, что делать
        logging.getLogger(__name__).warning("STT provider %s: prepare() failed", provider.name, exc_info=True)


def _classify_attempt(
    text: Optional[str],
    error: Optional[str],
    min_text_chars: int,
) -> tuple[FallbackReason, Optional[str]]:
    """Причина попытки + нормализованный текст.

    Вынесено из ``select_recognition`` (issue #2365 Phase 2): с тремя
    провайдерами и кэшем «мёртвых» функция вылезала за CC-бюджет
    ADR-0021 (limit 15).
    """
    if error is not None and error.startswith("timeout>"):
        return "timeout", None
    if error is not None:
        return "error", None
    if text is None or not text.strip():
        # Провайдер вернул пустоту — это отдельная причина (не
        # low_confidence, который означает «распознали, но слишком
        # коротко»). Vosk 0.42 для эха возвращает "а" — это
        # low_confidence, а не empty.
        return "empty", text
    if is_short_phrase(text, min_chars=min_text_chars):
        # text оставляем как есть для лога, но он слишком короткий.
        return "low_confidence", text
    return "ok", text


def _policy_for(
    provider: STTProvider,
    index: int,
    policies: Optional[Mapping[str, ProviderPolicy]],
    timeout_s: float,
    max_retries: int,
    retry_backoff_s: float,
) -> ProviderPolicy:
    """Бюджет для провайдера: из ``policies`` или legacy-правило.

    Legacy-правило (до issue #2365 Phase 2, сохраняется когда ``policies``
    не передан): общий ``timeout_s`` на всех, retry — только первому
    в цепочке. Явная политика перебивает его целиком.
    """
    if policies is not None:
        policy = policies.get(provider.name)
        if policy is not None:
            return policy
    return ProviderPolicy(
        timeout_s=timeout_s,
        max_retries=max_retries if index == 0 else 0,
        retry_backoff_s=retry_backoff_s,
    )


def _live_providers(
    providers: Sequence[STTProvider],
    dead_cache: Optional[ProviderDeadCache],
    attempts: List[STTAttempt],
) -> Sequence[STTProvider]:
    """Отбросить провайдеров из кэша «мёртвых».

    Пропуск логируется отдельной попыткой ``reason="dead"`` — оператор
    в ``stt_attempt`` видит, ПОЧЕМУ фраза ушла сразу в Vosk, а не
    гадает, куда делся MiniMax.

    Если мёртвыми оказались ВСЕ — кэш игнорируется: глухой робот хуже
    медленного, пусть лучше честно сходит в облако.
    """
    if dead_cache is None:
        return providers
    live = [p for p in providers if not dead_cache.is_dead(p.name)]
    if not live:
        return providers
    for provider in providers:
        if provider not in live:
            attempts.append(
                STTAttempt(
                    provider=provider.name,
                    reason="dead",
                    latency_ms=0,
                    error=(
                        f"dead {dead_cache.remaining_s(provider.name):.0f}s more: "
                        f"{dead_cache.reason(provider.name)}"
                    ),
                )
            )
    return live


def _run_provider(
    provider: STTProvider,
    audio_bytes: bytes,
    policy: ProviderPolicy,
    min_text_chars: int,
    attempts: List[STTAttempt],
    dead_cache: Optional[ProviderDeadCache],
) -> Optional[str]:
    """Отработать одного провайдера с его retry-бюджетом.

    Возвращает текст при ``reason == "ok"``, иначе ``None`` (цепочка
    идёт дальше). Побочно наполняет ``attempts`` и обновляет
    ``dead_cache``.
    """
    for attempt_idx in range(max(0, policy.max_retries) + 1):
        if attempt_idx > 0:
            # Линейный backoff (для 1 retry — один sleep).
            time.sleep(policy.retry_backoff_s)

        text, latency_ms, error, exc = _measure(
            lambda p=provider: p.recognize(audio_bytes),
            timeout_s=policy.timeout_s,
        )
        reason, text = _classify_attempt(text, error, min_text_chars)
        attempts.append(
            STTAttempt(
                provider=provider.name,
                reason=reason,
                latency_ms=latency_ms,
                text=text,
                error=error,
                attempt_index=attempt_idx,
            )
        )

        if reason == "ok":
            if dead_cache is not None:
                dead_cache.mark_alive(provider.name)
            return text

        if reason in ("timeout", "error") and dead_cache is not None:
            permanent = is_permanent_failure(exc)
            dead_cache.mark_dead(
                provider.name,
                error or reason,
                transient=not permanent,
            )
            if permanent:
                # Квота/ключ — повтор вернёт ровно то же самое.
                break
    return None


def select_recognition(
    providers: Sequence[STTProvider],
    audio_bytes: bytes,
    *,
    timeout_s: float = DEFAULT_YANDEX_TIMEOUT_S,
    max_retries: int = DEFAULT_YANDEX_MAX_RETRIES,
    retry_backoff_s: float = DEFAULT_RETRY_BACKOFF_S,
    min_text_chars: int = DEFAULT_MIN_TEXT_CHARS,
    policies: Optional[Mapping[str, ProviderPolicy]] = None,
    dead_cache: Optional[ProviderDeadCache] = None,
) -> tuple[Optional[str], List[STTAttempt]]:
    """Прогнать цепочку провайдеров по приоритету, с retry и фолбеком.

    Логика:
        * Провайдеры из ``dead_cache`` пропускаются (см. :func:`_live_providers`).
        * Каждый получает свой бюджет (:class:`ProviderPolicy`): таймаут,
          число повторов, backoff.
        * Первый ``reason == "ok"`` завершает цепочку.
        * Отказ (timeout/error) помечает провайдера мёртвым в кэше.

    Args:
        providers: Непустая последовательность В ПОРЯДКЕ ПРИОРИТЕТА.
            [0] — primary, [1..] — фолбеки.
        audio_bytes: PCM int16 LE mono 16kHz.
        timeout_s: Legacy per-call timeout (когда ``policies`` не задан).
        max_retries: Legacy число retry для ``providers[0]``.
        retry_backoff_s: Линейный backoff между повторами.
        min_text_chars: Порог ``is_short_phrase``.
        policies: Per-provider бюджет по имени (issue #2365 Phase 2).
            Отсутствующие имена падают на legacy-правило.
        dead_cache: Кэш «мёртвых» провайдеров (issue #2365 Phase 2).
            ``None`` — старое поведение, каждый раз ходим во все облака.

    Returns:
        ``(text, attempts)``. ``text`` — первый непустой НЕ-короткий результат
        (``reason == "ok"``). Если ни один провайдер не дал ``ok``, но хоть
        один вернул непустой (пусть короткий/мусорный) текст — возвращаем
        последний такой текст, чтобы caller отличил rejected(short) от
        rejected(empty). ``text`` равен ``None`` только если ВСЕ попытки
        вернули пусто/ошибки (rejected(empty) — эхо/музыка, молчим).
    """
    if not providers:
        raise ValueError("providers sequence must be non-empty")

    attempts: List[STTAttempt] = []

    for index, provider in enumerate(_live_providers(providers, dead_cache, attempts)):
        policy = _policy_for(
            provider, index, policies, timeout_s, max_retries, retry_backoff_s
        )
        _prepare_provider(provider)
        text = _run_provider(
            provider, audio_bytes, policy, min_text_chars, attempts, dead_cache
        )
        if text is not None:
            return text, attempts

    # Сюда дошли, если ни один провайдер не дал "ok".
    # Возвращаем последний непустой текст (даже если он слишком короткий —
    # rejected_short), чтобы caller (speech_audio_callback) мог отличить
    # «была речь, но слишком короткая/мусор» от «пусто/эхо». Это важно для
    # issue #979 acceptance: Vosk вернул «не» (мусор от слабой модели) —
    # робот должен сказать «не расслышал, скажи ещё раз», а НЕ молчать.
    # None — только если ВСЕ попытки вернули пусто/ошибки (rejected_empty,
    # почти наверняка эхо собственного TTS/музыки — issue 989: молчим).
    last_text: Optional[str] = None
    for _a in attempts:
        if _a.text:
            last_text = _a.text
    return last_text, attempts


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
    # NB: RcutilsLogger не принимает позиционные аргументы (%s-стиль),
    # только одну строку — поэтому f-string.
    for a in attempts:
        a_text = f"'{a.text[:30]}'" if a.text else "-"
        # ``error`` печатаем обязательно: 21.09.2026 на роботе в логе стояло
        # голое ``reason=error``, и чтобы узнать, ЧТО именно ответили облака,
        # пришлось лезть на робота двумя пробниками. Строка уже лежала в
        # STTAttempt.error — её просто выбрасывали.
        a_error = f" error='{a.error[:120]}'" if a.error else ""
        logger.info(
            f"[stt_attempt_metric] provider={a.provider} reason={a.reason} "
            f"latency_ms={a.latency_ms} attempt={a.attempt_index} "
            f"text={a_text}{a_error}"
        )
