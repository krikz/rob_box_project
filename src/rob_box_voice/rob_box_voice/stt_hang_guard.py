#!/usr/bin/env python3
"""stt_hang_guard — поток фразы STT не ждёт вызов, который завис (#4135).

28.10 07:08 UTC (акт 9, run 40648027547): стрим Yandex STT упал по ``DEADLINE_EXCEEDED``, и stt_node повис в
``close()`` старого gRPC-канала прямо в обработчике ошибки. Попытка провайдера не вернулась, а цепочка
yandex → minimax → vosk решает только по ВЕРНУВШЕЙСЯ попытке: ни minimax, ни vosk фразу не получили, робот был
глух 5 минут до ``docker restart``.

Две операции, обе об одном — вызов, который может зависнуть навсегда, не держит поток фразы:

* :meth:`HangGuard.run` — вызов в отдельном потоке, ждём не дольше предела. Не вернулся — брошен
  (``Watched.hung``); что это значит для фразы (провал попытки, следующий провайдер), решает цепочка
  ``stt_fallback.select_recognition``, здесь только факт. Брошенный вызов помнится, пока не вернётся
  (:meth:`HangGuard.hung_for_s`): цепочка не зовёт того же провайдера параллельно с ним — у Vosk один
  ``KaldiRecognizer`` на ноду, второй поток в нём ломает распознавание.
* :meth:`HangGuard.release` — отпустить ресурс: закрытие уходит в фон под тем же сторожем, вызывающий не ждёт
  ни секунды. Пока прошлое закрытие под тем же именем висит, новое не запускается — ресурс просто отпускается,
  висящих потоков не больше одного на имя.

Поток Python не убить: брошенный вызов досиживает в daemon-потоке. Если зависание держит GIL (C-код без
отпускания GIL), не поможет ничто на Python — это честный предел сторожа. 28.10 все потоки stt_node стояли в
``futex_wait`` при нулевом CPU — похоже на ожидание условия с отпущенным GIL, но это не доказано.

Только stdlib — тестируется без ROS (``test/unit/stt/test_issue_4135_stt_hang_failover.py``).
"""

from __future__ import annotations

import contextvars
import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

# Сколько ждать фоновое закрытие, прежде чем считать его зависшим (строка в лог; пока висит, второе закрытие
# под тем же именем не запускается). На поток фразы не влияет — он закрытия не ждёт вовсе.
RELEASE_LIMIT_S = 5.0


@dataclass(frozen=True)
class Watched:
    """Исход вызова под сторожем: вернулся (``value`` или ``exc``) или брошен (``hung``)."""

    value: Any = None
    exc: Optional[BaseException] = None
    hung: bool = False


class HangGuard:
    """Сторож зависаний: ни один вызов не держит вызывающего дольше предела (#4135).

    Одна на ноду: помнит брошенные вызовы между фразами. ``log`` — строка оператору (у stt_node —
    ``get_logger().warning``); ``clock`` подменяется в тестах.
    """

    def __init__(
        self,
        *,
        log: Optional[Callable[[str], None]] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._log = log or logging.getLogger(__name__).warning
        self._clock = clock
        self._lock = threading.Lock()
        # имя -> (метка вызова, когда начат): брошенные и ещё не вернувшиеся вызовы.
        self._stuck: dict[str, tuple[object, float]] = {}

    def warn(self, message: str) -> None:
        """Строка оператору тем же каналом, что строки самого сторожа."""
        self._log(message)

    def hung_for_s(self, name: str) -> Optional[float]:
        """Сколько секунд висит брошенный вызов ``name``; ``None`` — не висит."""
        with self._lock:
            entry = self._stuck.get(name)
        return None if entry is None else self._clock() - entry[1]

    def run(self, name: str, call: Callable[[], Any], limit_s: float) -> Watched:
        """Выполнить ``call`` в отдельном потоке и ждать не дольше ``limit_s`` секунд.

        Исключение ``call`` возвращается в ``Watched.exc`` (кроме ``KeyboardInterrupt``/``SystemExit`` —
        они поднимаются). Контекст (``contextvars``, span трассировки) переезжает в поток вызова.
        """
        token = object()
        started = self._clock()
        box: dict = {}
        finished = threading.Event()
        context = contextvars.copy_context()
        threading.Thread(
            target=self._work,
            args=(name, token, context, call, box, finished),
            name=f"hang-guard-{name}",
            daemon=True,
        ).start()
        if finished.wait(max(0.0, limit_s)) or not self._abandon(
            name, token, started, finished
        ):
            return self._outcome(box)
        self._log(
            f"⏱️ [#4135] '{name}' не вернулся за {limit_s:.1f} с — вызов брошен, досиживает в фоне"
        )
        return Watched(hung=True)

    def release(
        self, name: str, close: Callable[[], Any], limit_s: float = RELEASE_LIMIT_S
    ) -> bool:
        """Отпустить ресурс: ``close`` — в фоне под сторожем, вызывающий не ждёт.

        ``False`` — прошлое закрытие под этим именем ещё висит: новое не запущено, ресурс отпущен без него.
        """
        hung_s = self.hung_for_s(name)
        if hung_s is not None:
            self._log(
                f"⏳ [#4135] '{name}': прошлый вызов висит {hung_s:.0f} с — новый не запускаю, ресурс отпущен без него"
            )
            return False
        threading.Thread(
            target=self._release,
            args=(name, close, limit_s),
            name=f"hang-guard-release-{name}",
            daemon=True,
        ).start()
        return True

    # -- внутреннее ---------------------------------------------------------

    def _release(self, name: str, close: Callable[[], Any], limit_s: float) -> None:
        outcome = self.run(name, close, limit_s)
        if outcome.exc is not None:
            self._log(f"⚠️ [#4135] '{name}': {outcome.exc!r}")

    def _work(self, name, token, context, call, box, finished) -> None:
        try:
            box["value"] = context.run(call)
        except BaseException as exc:  # noqa: BLE001 — исход отдаём вызывающему
            box["exc"] = exc
        finally:
            self._finish(name, token, finished)

    def _abandon(
        self, name: str, token: object, started: float, finished: threading.Event
    ) -> bool:
        """Запомнить вызов брошенным; ``False`` — он успел вернуться между ожиданием и этой проверкой."""
        with self._lock:
            if finished.is_set():
                return False
            self._stuck[name] = (token, started)
            return True

    def _finish(self, name: str, token: object, finished: threading.Event) -> None:
        with self._lock:
            finished.set()
            entry = self._stuck.get(name)
            late = entry is not None and entry[0] is token
            if late:
                del self._stuck[name]
        if late:
            self._log(
                f"♻️ [#4135] '{name}' вернулся через {self._clock() - entry[1]:.1f} с — уже брошен, исход не нужен"
            )

    @staticmethod
    def _outcome(box: dict) -> Watched:
        exc = box.get("exc")
        if exc is not None and not isinstance(exc, Exception):
            raise exc
        return Watched(value=box.get("value"), exc=exc)
