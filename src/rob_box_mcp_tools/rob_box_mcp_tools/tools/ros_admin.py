#!/usr/bin/env python3
"""
ros_admin.py - админские тулы ЧТЕНИЯ ROS-графа (issue #4516, шаг 1; решение Шифу 06.11: «Телега - считай админка»).

Срез ``operator.admin``: тулы видны ТАРС (оператору) и пульту Telegram (``telegram_visible``), голосу Личности не даны.
Публикации в топики здесь НЕТ (шаг 2, с предохранителем).

* :class:`RosListTool` - ``ros_list(kind, filter)``: топики или сервисы с типами.
* :class:`TopicEchoTool` - ``topic_echo(topic, count, timeout_s)``: до 5 сообщений, текст обрезан и пропущен через
  редактор секретов; картинки/облака точек - только сводка (тип, размер, частота).
* :class:`TopicStatsTool` - ``topic_stats(topic, window_s)``: ``hz`` и ``bw`` за окно ≤ 10 с.
* :class:`VescTelemetryTool` - ``vesc_telemetry()``: напряжение/ток/обороты/температуры колёс из ROS
  (``/dynamic_joint_states``), НЕ с CAN; чего в ROS нет, о том тул прямо говорит.

Числа и фразы строит :mod:`rob_box_mcp_tools.ros_introspect` (ADR-0148): модель получает готовый текст и данные.
Свойства тулов (``llm_visible``, ``slice`` ...) литералами в каждом классе: каталог читает их через AST.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from .. import ros_introspect as ri
from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType
from .operator_admin import _sanitize_text


def _rejected(reason: str, text: str, **extra: Any) -> MCPToolResult:
    """Отказ: машинная причина в данных, человеческий текст - в ``error`` (его строит код, не модель)."""
    return MCPToolResult(
        success=False, error=text, message=text, data={"status": "rejected", "reason": reason, **extra}
    )


def _probe_or_none(node: Any) -> Optional[ri.RosProbe]:
    probe = ri.RosProbe(node)
    return probe if probe.available() else None


_NO_ROS = "ROS на этом узле недоступен: ноды с rclpy нет, читать граф нечем."


def _resolve_topic(probe: ri.RosProbe, topic: Any) -> Any:
    """``(тип, None)`` или ``(None, отказ)``: имя безопасно, топик есть в графе, у него есть издатель."""
    if not ri.valid_topic_name(topic):
        return None, _rejected(ri.BAD_TOPIC, f"Имя топика {topic!r} не годится: нужно вида /имя/подимя.")
    type_name = probe.topic_type(topic)
    if type_name is None:
        return None, _rejected(ri.NO_SUCH_TOPIC, f"Топика {topic} в ROS-графе нет.")
    if probe.publishers_count(topic) == 0:
        return None, _rejected(ri.NO_PUBLISHERS, f"У топика {topic} ({type_name}) нет издателей: слушать нечего.")
    return type_name, None


class RosListTool(MCPTool):
    """Топики или сервисы ROS с типами."""

    @property
    def name(self) -> str:
        return "ros_list"

    @property
    def llm_visible(self) -> bool:
        return False

    @property
    def slice(self) -> str:
        return "operator.admin"

    @property
    def telegram_visible(self) -> bool:
        return True

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.MEDIUM

    @property
    def read_only(self) -> bool:
        return True

    @property
    def description(self) -> str:
        return (
            "Список топиков или сервисов ROS 2 с типами сообщений. Звать на «какие есть топики», «покажи сервисы», "
            "«какие топики про лидар». Пересказывай данные результата, ничего не добавляй."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="kind",
                type="string",
                required=False,
                enum=["topics", "services"],
                description="topics - топики (по умолчанию); services - сервисы.",
            ),
            MCPToolParameter(
                name="filter",
                type="string",
                required=False,
                description="Подстрока имени или типа (например lidar, Twist); пусто - всё.",
            ),
        ]

    def execute(self, kind: str = "topics", filter: str = "") -> MCPToolResult:
        probe = _probe_or_none(self.node)
        if probe is None:
            return _rejected(ri.ROS_UNAVAILABLE, _NO_ROS)
        kind = "services" if kind == "services" else "topics"
        pairs = probe.services() if kind == "services" else probe.topics()
        data = ri.summarize_graph(pairs, filter or "")
        return MCPToolResult(
            success=True, data={"status": "done", "kind": kind, **data}, message=ri.graph_message(kind, data)
        )


class TopicEchoTool(MCPTool):
    """Прочитать одно или несколько сообщений топика."""

    @property
    def name(self) -> str:
        return "topic_echo"

    @property
    def llm_visible(self) -> bool:
        return False

    @property
    def slice(self) -> str:
        return "operator.admin"

    @property
    def telegram_visible(self) -> bool:
        return True

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.MEDIUM

    @property
    def read_only(self) -> bool:
        return True

    @property
    def description(self) -> str:
        return (
            "Прочитать из топика ROS 2 одно сообщение или до 5 (аналог ros2 topic echo). Звать на «что в топике "
            "/battery_state», «покажи 3 сообщения из /odom». Картинки и облака точек не отдаются содержимым, только "
            "сводкой. Текст обрезается. Пересказывай данные результата, числа не выдумывай."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="topic",
                type="string",
                required=True,
                description="Полное имя топика с ведущим слэшем, например /battery_state.",
            ),
            MCPToolParameter(
                name="count",
                type="integer",
                required=False,
                description="Сколько сообщений прочитать (1-5, по умолчанию 1).",
            ),
            MCPToolParameter(
                name="timeout_s",
                type="number",
                required=False,
                description="Сколько секунд ждать (до 10, по умолчанию 5).",
            ),
        ]

    def execute(self, topic: str, count: Optional[int] = None, timeout_s: Optional[float] = None) -> MCPToolResult:
        probe = _probe_or_none(self.node)
        if probe is None:
            return _rejected(ri.ROS_UNAVAILABLE, _NO_ROS)
        type_name, refusal = _resolve_topic(probe, topic)
        if refusal is not None:
            return refusal
        n = ri.clamp_int(count, 1, ri.ECHO_COUNT_MAX)
        wait = ri.clamp_seconds(timeout_s, 5.0, ri.ECHO_TIMEOUT_MAX_S)
        samples = probe.collect(topic, type_name, n, wait)
        if not samples:
            return _rejected(
                ri.NO_MESSAGES,
                f"Из {topic} ({type_name}) за {wait:g} с не пришло ни одного сообщения.",
                topic=topic,
                type=type_name,
            )
        return self._answer(probe, topic, type_name, samples, wait)

    @staticmethod
    def _answer(probe: ri.RosProbe, topic: str, type_name: str, samples: List[ri.Sample], wait: float) -> MCPToolResult:
        sizes = [s.size for s in samples]
        if ri.is_bulk_type(type_name) or max(sizes) > ri.BULK_MESSAGE_BYTES:
            meter = ri.RateMeter([(s.at, s.size) for s in samples])
            stats = meter.summary(wait)
            text = (
                f"{topic} ({type_name}): тяжёлый тип, содержимое не отдаётся; получено {len(samples)} сообщ., "
                f"размер до {max(sizes)} Б."
            )
            return MCPToolResult(
                success=True,
                message=text,
                data={
                    "status": "done",
                    "topic": topic,
                    "type": type_name,
                    "summary_only": True,
                    "sizes_bytes": sizes,
                    "hz": stats["hz"],
                },
            )
        texts: List[str] = []
        truncated = False
        for sample in samples:
            body, cut = ri.fit_text(
                _sanitize_text(probe.render(type_name, sample.raw)), ri.TEXT_CHARS_MAX // len(samples)
            )
            truncated = truncated or cut
            texts.append(body)
        message = (
            f"{topic} ({type_name}), сообщений {len(texts)}"
            + (", текст обрезан" if truncated else "")
            + ":\n"
            + "\n---\n".join(texts)
        )
        return MCPToolResult(
            success=True,
            message=message,
            data={
                "status": "done",
                "topic": topic,
                "type": type_name,
                "count": len(texts),
                "truncated": truncated,
                "messages": texts,
            },
        )


class TopicStatsTool(MCPTool):
    """Замер частоты и полосы топика за окно."""

    @property
    def name(self) -> str:
        return "topic_stats"

    @property
    def llm_visible(self) -> bool:
        return False

    @property
    def slice(self) -> str:
        return "operator.admin"

    @property
    def telegram_visible(self) -> bool:
        return True

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.MEDIUM

    @property
    def read_only(self) -> bool:
        return True

    @property
    def description(self) -> str:
        return (
            "Замерить частоту (hz) и полосу (bw) топика ROS 2 за окно до 10 секунд. Звать на «какая частота у "
            "/scan», «померяй hz камеры», «сколько байт в секунду идёт по /odom». Числа в ответе посчитал код: "
            "называй их как есть, не округляй и не придумывай."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="topic",
                type="string",
                required=True,
                description="Полное имя топика с ведущим слэшем, например /scan.",
            ),
            MCPToolParameter(
                name="window_s",
                type="number",
                required=False,
                description="Окно замера в секундах (до 10, по умолчанию 5).",
            ),
        ]

    def execute(self, topic: str, window_s: Optional[float] = None) -> MCPToolResult:
        probe = _probe_or_none(self.node)
        if probe is None:
            return _rejected(ri.ROS_UNAVAILABLE, _NO_ROS)
        type_name, refusal = _resolve_topic(probe, topic)
        if refusal is not None:
            return refusal
        window = ri.clamp_seconds(window_s, ri.RATE_WINDOW_DEFAULT_S, ri.RATE_WINDOW_MAX_S)
        samples = probe.collect(topic, type_name, 1_000_000, window, keep_raw=False)
        stats = ri.RateMeter([(s.at, s.size) for s in samples]).summary(window)
        return MCPToolResult(
            success=True,
            message=ri.rate_message(topic, type_name, stats),
            data={"status": "done", "topic": topic, "type": type_name, **stats},
        )


class VescTelemetryTool(MCPTool):
    """Телеметрия VESC по колёсам из ROS (CAN напрямую не читается)."""

    @property
    def name(self) -> str:
        return "vesc_telemetry"

    @property
    def llm_visible(self) -> bool:
        return False

    @property
    def slice(self) -> str:
        return "operator.admin"

    @property
    def telegram_visible(self) -> bool:
        return True

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.MEDIUM

    @property
    def read_only(self) -> bool:
        return True

    @property
    def description(self) -> str:
        return (
            "Телеметрия моторов-колёс (VESC): напряжение, ток, обороты, температура FET и мотора по каждому колесу, "
            "из ROS. Звать на «что с vesc», «напряжение на колёсах», «температура моторов». Чего в ROS нет, "
            "результат называет прямо: не додумывай такие числа."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return []

    def execute(self) -> MCPToolResult:
        probe = _probe_or_none(self.node)
        if probe is None:
            return _rejected(ri.ROS_UNAVAILABLE, _NO_ROS)
        got = probe.read_joint_states()
        if got is None:
            return _rejected(
                ri.NO_MESSAGES,
                f"Из {ri.JOINT_STATES_TOPIC} за 2 с ничего не пришло: ros2_control не "
                "публикует состояние (контроллеры не запущены или топика нет).",
            )
        report = ri.vesc_report(ri.vesc_from_joint_states(*got))
        data: Dict[str, Any] = {"status": "done", "source": ri.JOINT_STATES_TOPIC, **report}
        return MCPToolResult(success=True, message=ri.vesc_message(report), data=data)
