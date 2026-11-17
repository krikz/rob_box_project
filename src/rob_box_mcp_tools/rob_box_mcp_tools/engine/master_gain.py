"""Подтверждение уровня мастер-фейдера музыки сервером (#3790, ADR-0148, ADR-0150 §5.2).

``/n_set 999 gain <v>`` scsynth не подтверждает: узла нет - отвечает ``/fail``, сервера нет - UDP молчит или даёт
``ECONNREFUSED``, а тул раньше в любом случае говорил «Громкость музыки: 50% → 70%». Теперь после установки код просит
у сервера то, что тот сейчас держит (``/s_get 999 gain`` -> ``/n_set 999 gain <v>``), и разбирает ответ в явный исход:

* ``applied`` - сервер вернул уровень, совпавший с заказанным (допуск :data:`GAIN_TOLERANCE`, float32);
* ``rejected`` - сервер или сокет отказали: ``no_master_node`` (``/fail`` «Node 999 not found»), ``osc_error``
  (``OSError`` сокета), ``server_fail`` (иной ``/fail``), ``mismatch`` (уровень отвечен, но не тот);
* ``timeout`` - за :data:`CONFIRM_TIMEOUT_S` не пришло ни ответа, ни отказа - применено ли, неизвестно.

Здесь только чистые функции и данные: сокет приходит снаружи (``RenardoRuntime``), поэтому каждый исход проверяется
фейковым транспортом без SuperCollider. Фраза об успехе строится кодом из исхода (:func:`GainOutcome.applied`), а не
из «мы отправили».
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable, List, Optional, Sequence, Tuple

from . import renardo_adapter

APPLIED = "applied"
REJECTED = "rejected"
TIMEOUT = "timeout"

NO_MASTER_NODE = "no_master_node"
OSC_ERROR = "osc_error"
SERVER_FAIL = "server_fail"
MISMATCH = "mismatch"

#: Сколько ждём ответ ``/s_get``. На loopback ответ приходит за микросекунды; тул не должен заметно тормозить.
CONFIRM_TIMEOUT_S = 0.25
#: Допуск сравнения «заказанное = отвеченное»: scsynth хранит контролы во float32 (ошибка ~1e-8).
GAIN_TOLERANCE = 1e-3

#: Сообщение OSC: ``(адрес, *аргументы)``.
OscCall = Tuple
#: Классификатор пакета: ``True`` - пакет окончательный, ждать дальше не нужно.
IsFinal = Callable[[bytes], bool]
#: Транспорт: отправить сообщения подряд с одного сокета и вернуть пакеты-ответы, пока не придёт окончательный или
#: не выйдет таймаут. ``OSError`` сокета транспорт не глотает.
OscTransport = Callable[[Sequence[OscCall], float, IsFinal], List[bytes]]


@dataclass(frozen=True)
class GainOutcome:
    """Исход установки мастер-фейдера: что заказали, что подтвердил сервер и почему отказ."""

    status: str
    requested: float
    confirmed: Optional[float] = None
    reason: str = ""
    detail: str = ""

    @property
    def applied(self) -> bool:
        return self.status == APPLIED


def decode_reply(data: bytes, node: int) -> Optional[Tuple[str, object]]:
    """Ответ scsynth по нашему узлу: ``("gain", значение)`` на ``/n_set <node> gain <v>``, ``("fail", текст)`` на
    ``/fail``; всё прочее (``/done``, чужие узлы, мусор) - ``None``."""
    address, rest = renardo_adapter.split_osc_address(data)
    args = renardo_adapter.decode_osc_args(rest) if address else []
    if address == "/fail":
        return "fail", " ".join(str(a) for a in args) or rest.decode("utf-8", "replace")
    if address == "/n_set" and len(args) >= 3 and args[0] == node and args[1] == "gain":
        try:
            return "gain", float(args[2])
        except (TypeError, ValueError):
            return None
    return None


def fail_reason(text: str) -> str:
    """Причина отказа по тексту ``/fail``: узла нет («Node 999 not found») - ``no_master_node``, иное - ``server_fail``."""
    lowered = text.lower()
    return NO_MASTER_NODE if "node" in lowered and "not found" in lowered else SERVER_FAIL


def outcome_from_replies(replies: Iterable[bytes], node: int, requested: float) -> GainOutcome:
    """Исход по пакетам-ответам: первый окончательный решает; пусто - ``timeout``."""
    for data in replies:
        decoded = decode_reply(data, node)
        if decoded is None:
            continue
        kind, value = decoded
        if kind == "fail":
            return GainOutcome(REJECTED, requested, reason=fail_reason(str(value)), detail=str(value))
        confirmed = float(value)  # type: ignore[arg-type]
        if abs(confirmed - requested) <= GAIN_TOLERANCE:
            return GainOutcome(APPLIED, requested, confirmed=confirmed)
        return GainOutcome(REJECTED, requested, confirmed=confirmed, reason=MISMATCH,
                           detail=f"сервер держит {confirmed:.3f}, заказано {requested:.3f}")
    return GainOutcome(TIMEOUT, requested, reason=TIMEOUT)


def confirm_gain(transport: OscTransport, node: int, gain: float, timeout: float = CONFIRM_TIMEOUT_S) -> GainOutcome:
    """``/n_set <node> gain <gain>`` и ``/s_get <node> gain`` через ``transport``; исход - по ответу сервера.

    ``OSError`` транспорта (сокет, ``ECONNREFUSED`` при мёртвом scsynth) - ``rejected(osc_error)``, а не «применено».
    """
    calls = (("/n_set", node, "gain", gain), ("/s_get", node, "gain"))
    try:
        replies = transport(calls, timeout, lambda data: decode_reply(data, node) is not None)
    except OSError as exc:
        return GainOutcome(REJECTED, gain, reason=OSC_ERROR, detail=f"{type(exc).__name__}: {exc}")
    return outcome_from_replies(replies, node, gain)


__all__ = [
    "APPLIED", "CONFIRM_TIMEOUT_S", "GAIN_TOLERANCE", "GainOutcome", "MISMATCH", "NO_MASTER_NODE", "OSC_ERROR",
    "REJECTED", "SERVER_FAIL", "TIMEOUT", "confirm_gain", "decode_reply", "fail_reason", "outcome_from_replies",
]
