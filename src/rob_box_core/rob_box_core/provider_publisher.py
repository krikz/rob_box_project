"""provider_publisher - писатель ``/voice/providers/state`` своего вида (ADR-0150 §8, PR-4 ч.2, #3693).

Топик один (latched), писателей несколько (tts_node, stt_node, dialogue_node, mcp_server), каждый публикует только
свои ``ProviderState`` и ПЕРЕИЗДАЁТ их раз в ``period_s``: поздний подписчик по transient_local получает не всех
писателей (проверено на rmw_zenoh, PR #3729), поэтому корректность держится на переиздании, а подписчик считает
запись без обновлений устаревшей (``rob_box_dialog.degrade.ProviderBoard``). Состояние - duck-тип с полями
``name, kind, health, since, last_error_code``; пакет сообщений импортируется лениво.
"""

from __future__ import annotations

from typing import Any, Callable, List, Sequence

TOPIC = "/voice/providers/state"
REPUBLISH_PERIOD_S = 2.0


def publish_provider_states(pub: Any, states: Sequence[Any]) -> None:
    """Снимок в ``ProviderStateArray`` (``since`` - wall-clock секунды)."""
    from rob_box_dialog_msgs.msg import ProviderState as Msg  # noqa: PLC0415
    from rob_box_dialog_msgs.msg import ProviderStateArray  # noqa: PLC0415

    arr = ProviderStateArray()
    for st in states:
        m = Msg()
        m.name, m.kind, m.health = st.name, st.kind, st.health
        m.since.sec = int(st.since)
        m.since.nanosec = int((st.since % 1) * 1e9)
        m.last_error_code = st.last_error_code or ""
        arr.providers.append(m)
    pub.publish(arr)


def make_publisher(node: Any) -> Any:
    """Latched-публикатор топика (RELIABLE, KEEP_LAST 1, TRANSIENT_LOCAL)."""
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: PLC0415
    from rob_box_dialog_msgs.msg import ProviderStateArray  # noqa: PLC0415

    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    return node.create_publisher(ProviderStateArray, TOPIC, qos)


class ProviderStatePublisher:
    """Таймер ноды: ``states_fn()`` -> свои состояния -> переиздание. Пустой список не публикуется."""

    def __init__(self, node: Any, states_fn: Callable[[], List[Any]], period_s: float = REPUBLISH_PERIOD_S) -> None:
        self._states_fn = states_fn
        self._pub = make_publisher(node)
        self._timer = node.create_timer(period_s, self.tick)

    def tick(self) -> None:
        states = self._states_fn()
        if states:
            publish_provider_states(self._pub, states)
