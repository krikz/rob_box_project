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
  ``KaldiRecognizer`` на ноду, второй поток в нём ломает распознавание. Брошенный вызов, вернувшийся позже,
  отдаёт свой исход тому, кто его запустил (``on_late``): отказ по квоте/ключу после предела сторожа
  не теряется (ревью PR #4140).
* :meth:`HangGuard.release` — отпустить ресурс: закрытие уходит в фон под тем же сторожем, вызывающий не ждёт
  ни секунды. Пока прошлое закрытие под тем же именем идёт или висит, новое не запускается — ресурс просто
  отпускается: имя резервируется под замком, висящих потоков не больше одного на имя.

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
        # имена закрытий, которые ``release`` уже запустил и которые ещё не отдали управление сторожу обратно:
        # проверка занятости и резерв имени — один шаг под замком, иначе два закрытия подряд оба видят «свободно».
        self._releasing: set[str] = set()

    def warn(self, message: str) -> None:
        """Строка оператору тем же каналом, что строки самого сторожа."""
        self._log(message)

    def hung_for_s(self, name: str) -> Optional[float]:
        """Сколько секунд висит брошенный вызов ``name``; ``None`` — не висит."""
        with self._lock:
            entry = self._stuck.get(name)
        return None if entry is None else self._clock() - entry[1]

    def run(
        self,
        name: str,
        call: Callable[[], Any],
        limit_s: float,
        on_late: Optional[Callable[[Watched], None]] = None,
    ) -> Watched:
        """Выполнить ``call`` в отдельном потоке и ждать не дольше ``limit_s`` секунд.

        Исключение ``call`` возвращается в ``Watched.exc`` (кроме ``KeyboardInterrupt``/``SystemExit`` —
        они поднимаются). Контекст (``contextvars``, span трассировки) переезжает в поток вызова.

        Поток вызова не запустился (нет ресурса на поток) — это исход вызова, а не падение потока фразы:
        ``Watched.exc`` с причиной, ``call`` не выполнялся.

        ``on_late(Watched)`` зовётся из потока вызова, если вызов брошен и потом всё-таки вернулся (значение или
        исключение): исход нужен тому, кто решает про провайдера, — квота/ключ после предела сторожа не должны
        теряться. Вернулся в срок — не зовётся.
        """
        token = object()
        started = self._clock()
        box: dict = {}
        finished = threading.Event()
        context = contextvars.copy_context()
        try:
            threading.Thread(
                target=self._work,
                args=(name, token, context, call, box, finished, on_late),
                name=f"hang-guard-{name}",
                daemon=True,
            ).start()
        except Exception as exc:  # noqa: BLE001 — «can't start new thread» и подобное: исход попытки, не падение
            self._log(f"⚠️ [#4135] '{name}': поток вызова не запустился ({exc!r}) — вызов не выполнялся")
            return Watched(exc=exc)
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

        ``False`` — закрытие под этим именем уже идёт или висит (или поток не запустился): новое не запущено,
        ресурс отпущен без него. Занятость проверяется и имя резервируется одним шагом под замком: два
        DEADLINE за несколько секунд не дают двух висящих закрытий (ревью PR #4140).
        """
        with self._lock:
            entry = self._stuck.get(name)
            busy = entry is not None or name in self._releasing
            if not busy:
                self._releasing.add(name)
            hung_s = None if entry is None else self._clock() - entry[1]
        if busy:
            what = "ещё идёт" if hung_s is None else f"висит {hung_s:.0f} с"
            self._log(
                f"⏳ [#4135] '{name}': прошлый вызов {what} — новый не запускаю, ресурс отпущен без него"
            )
            return False
        try:
            threading.Thread(
                target=self._release,
                args=(name, close, limit_s),
                name=f"hang-guard-release-{name}",
                daemon=True,
            ).start()
        except Exception as exc:  # noqa: BLE001 — резерв имени не должен остаться навсегда
            self._unreserve(name)
            self._log(f"⚠️ [#4135] '{name}': поток закрытия не запустился ({exc!r}) — ресурс отпущен без закрытия")
            return False
        return True

    # -- внутреннее ---------------------------------------------------------

    def _unreserve(self, name: str) -> None:
        with self._lock:
            self._releasing.discard(name)

    def _release(self, name: str, close: Callable[[], Any], limit_s: float) -> None:
        try:
            outcome = self.run(name, close, limit_s)
        finally:
            self._unreserve(name)
        if outcome.exc is not None:
            self._log(f"⚠️ [#4135] '{name}': {outcome.exc!r}")

    def _work(self, name, token, context, call, box, finished, on_late) -> None:
        try:
            box["value"] = context.run(call)
        except BaseException as exc:  # noqa: BLE001 — исход отдаём вызывающему
            box["exc"] = exc
        finally:
            self._finish(name, token, finished, box, on_late)

    def _abandon(
        self, name: str, token: object, started: float, finished: threading.Event
    ) -> bool:
        """Запомнить вызов брошенным; ``False`` — он успел вернуться между ожиданием и этой проверкой."""
        with self._lock:
            if finished.is_set():
                return False
            self._stuck[name] = (token, started)
            return True

    def _finish(
        self,
        name: str,
        token: object,
        finished: threading.Event,
        box: dict,
        on_late: Optional[Callable[[Watched], None]],
    ) -> None:
        with self._lock:
            finished.set()
            entry = self._stuck.get(name)
            late = entry is not None and entry[0] is token
            if late:
                del self._stuck[name]
        if not late:
            return
        outcome = Watched(value=box.get("value"), exc=box.get("exc"))
        what = "текст не нужен" if outcome.exc is None else f"исход {outcome.exc!r}"
        self._log(
            f"♻️ [#4135] '{name}' вернулся через {self._clock() - entry[1]:.1f} с — уже брошен, {what}"
        )
        if on_late is not None:
            try:
                on_late(outcome)
            except Exception as err:  # noqa: BLE001 — сторож не должен ронять поток вызова
                self._log(f"⚠️ [#4135] '{name}': поздний исход не разобран: {err!r}")

    @staticmethod
    def _outcome(box: dict) -> Watched:
        exc = box.get("exc")
        if exc is not None and not isinstance(exc, Exception):
            raise exc
        return Watched(value=box.get("value"), exc=exc)
