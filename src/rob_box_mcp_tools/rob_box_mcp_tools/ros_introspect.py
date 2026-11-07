#!/usr/bin/env python3
"""ros_introspect.py - чтение ROS-графа для админских тулов пульта TG и ТАРС (issue #4516, шаг 1: только чтение).

Один модуль знает: какие типы сообщений отдаются только сводкой (картинки, облака точек), какие потолки у чтения
топика (число сообщений, окно замера, длина текста), как считаются ``hz`` и ``bw`` и как из ``/dynamic_joint_states``
достаются данные по колёсам. Тулы (``tools/ros_admin.py``) только зовут эти функции и отдают модели готовый текст и
данные: числа считает код, модель их не выдумывает (ADR-0148).

Чистая часть (потолки, ``RateMeter``, ``summarize_*``, ``vesc_*``) не импортирует rclpy и гоняется в unit-тестах.
``RosProbe`` - тонкий адаптер к rclpy: подписка с ``raw=True`` (сериализованные байты дают и размер для ``bw``, и сырьё
для десериализации), QoS берётся у первого издателя топика, чтобы ``echo`` ловил и «защёлкнутые» топики.
"""

from __future__ import annotations

import math
import re
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

#: Потолки чтения топика: сообщений за один вызов, окно ожидания и замера (с), длина текста ответа (знаков).
ECHO_COUNT_MAX = 5
ECHO_TIMEOUT_MAX_S = 10.0
RATE_WINDOW_MAX_S = 10.0
RATE_WINDOW_DEFAULT_S = 5.0
TEXT_CHARS_MAX = 1500
LIST_ROWS_MAX = 80
#: Сообщение крупнее этого - только сводка, каким бы ни был тип (картинка в чужом типе, длинный массив).
BULK_MESSAGE_BYTES = 64 * 1024

#: Тип содержит одну из подстрок - топик отдаётся только сводкой (тип, размер, частота), не содержимым.
BULK_TYPE_MARKERS: Tuple[str, ...] = (
    "Image",
    "PointCloud",
    "OccupancyGrid",
    "LaserScan",
    "Costmap",
    "Audio",
    "CameraInfo",
    "NNData",
    "SpatialDetection",
)
#: Причины отказа тулов чтения (машинные ключи; текст для человека строит код).
NO_SUCH_TOPIC, NO_PUBLISHERS, NO_MESSAGES, ROS_UNAVAILABLE, BAD_TOPIC = (
    "no_such_topic",
    "no_publishers",
    "no_messages",
    "ros_unavailable",
    "bad_topic",
)

_TOPIC_RE = re.compile(r"^/[A-Za-z0-9_/]+$")


def is_bulk_type(type_name: str) -> bool:
    """Тип «тяжёлый» (картинки, облака точек, карты, аудио): содержимое в ответ не идёт."""
    return any(marker in (type_name or "") for marker in BULK_TYPE_MARKERS)


def valid_topic_name(name: Any) -> bool:
    """Имя топика целиком из безопасных знаков (в CLI и в логи не попадёт чужой синтаксис)."""
    return isinstance(name, str) and len(name) <= 200 and bool(_TOPIC_RE.match(name))


def clamp_int(value: Any, default: int, high: int) -> int:
    """Целое в ``[1, high]``; мусор - ``default``."""
    try:
        return max(1, min(int(value), high))
    except (TypeError, ValueError):
        return default


def clamp_seconds(value: Any, default: float, high: float) -> float:
    """Секунды в ``(0, high]``; мусор и неположительное - ``default``."""
    try:
        seconds = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(seconds) or seconds <= 0:
        return default
    return min(seconds, high)


def fit_text(text: str, limit: int = TEXT_CHARS_MAX) -> Tuple[str, bool]:
    """Текст не длиннее ``limit``; второй элемент - обрезан ли."""
    if len(text) <= limit:
        return text, False
    return text[: limit - 1] + "…", True


# ----------------------------------------------------------------------------------------------------------------
# hz / bw
# ----------------------------------------------------------------------------------------------------------------


@dataclass
class RateMeter:
    """Приходы сообщений за окно: ``add(t, nbytes)`` и итог ``summary(window_s)``.

    ``hz`` - по интервалам между приходами (как ``ros2 topic hz``), нужно ≥ 2 сообщений; ``bw`` - байт в секунду за всё
    окно. Пока сообщений нет, честно ``None``, а не 0: «0 Гц» утверждало бы, что топик молчит, хотя мы могли не успеть.
    """

    arrivals: List[Tuple[float, int]] = field(default_factory=list)

    def add(self, at: float, nbytes: int) -> None:
        self.arrivals.append((at, int(nbytes)))

    def summary(self, window_s: float) -> Dict[str, Any]:
        count = len(self.arrivals)
        total = sum(size for _, size in self.arrivals)
        hz: Optional[float] = None
        if count >= 2:
            span = self.arrivals[-1][0] - self.arrivals[0][0]
            hz = (count - 1) / span if span > 0 else None
        return {
            "count": count,
            "window_s": round(window_s, 2),
            "hz": None if hz is None else round(hz, 2),
            "bw_bytes_s": round(total / window_s, 1) if count and window_s > 0 else None,
            "avg_size_bytes": round(total / count) if count else None,
            "max_size_bytes": max((size for _, size in self.arrivals), default=None),
        }


def rate_message(topic: str, type_name: str, stats: Dict[str, Any]) -> str:
    """Фраза замера по данным ``RateMeter.summary`` (числа - только из него)."""
    if not stats["count"]:
        return f"{topic} ({type_name}): за {stats['window_s']} с не пришло ни одного сообщения."
    hz = "мало данных для частоты (пришло одно сообщение)" if stats["hz"] is None else f"{stats['hz']} Гц"
    return (
        f"{topic} ({type_name}): {hz}; полоса {format_bandwidth(stats['bw_bytes_s'])}; сообщений {stats['count']} "
        f"за {stats['window_s']} с; средний размер {stats['avg_size_bytes']} Б, максимум {stats['max_size_bytes']} Б."
    )


def format_bandwidth(bytes_per_s: Optional[float]) -> str:
    """Байт/с в «123 Б/с», «4.5 КБ/с», «1.20 МБ/с»."""
    if bytes_per_s is None:
        return "нет данных"
    if bytes_per_s < 1024:
        return f"{bytes_per_s:.0f} Б/с"
    if bytes_per_s < 1024 * 1024:
        return f"{bytes_per_s / 1024:.1f} КБ/с"
    return f"{bytes_per_s / (1024 * 1024):.2f} МБ/с"


# ----------------------------------------------------------------------------------------------------------------
# Список топиков и сервисов
# ----------------------------------------------------------------------------------------------------------------


def summarize_graph(
    pairs: Iterable[Tuple[str, Sequence[str]]], flt: str = "", limit: int = LIST_ROWS_MAX
) -> Dict[str, Any]:
    """Строки ``имя: тип`` по ``(имя, [типы])`` ROS: фильтр - подстрока имени или типа, порядок по имени, потолок."""
    needle = (flt or "").strip().lower()
    rows = [
        {"name": name, "type": ", ".join(types)}
        for name, types in sorted(pairs, key=lambda p: p[0])
        if not needle or needle in name.lower() or any(needle in t.lower() for t in types)
    ]
    return {"total": len(rows), "items": rows[:limit], "truncated": len(rows) > limit}


def graph_message(kind: str, data: Dict[str, Any]) -> str:
    """Готовый текст списка: модель пересказывает его, а не пересчитывает."""
    title = "Топики" if kind == "topics" else "Сервисы"
    if not data["total"]:
        return f"{title}: по этому фильтру ничего нет."
    head = f"{title}: {data['total']}" + (f", показаны первые {len(data['items'])}" if data["truncated"] else "")
    return head + ".\n" + "\n".join(f"{row['name']} [{row['type']}]" for row in data["items"])


# ----------------------------------------------------------------------------------------------------------------
# VESC: данные по колёсам из ROS (не с can0)
# ----------------------------------------------------------------------------------------------------------------

#: Колёса: joint ros2_control -> метка (как ``wheel_labels`` в vesc_config.yaml).
WHEEL_JOINTS: Dict[str, str] = {
    "left_front_wheel_joint": "front_left",
    "right_front_wheel_joint": "front_right",
    "left_rear_wheel_joint": "rear_left",
    "right_rear_wheel_joint": "rear_right",
}
WHEEL_ORDER: Tuple[str, ...] = ("front_left", "front_right", "rear_left", "rear_right")
#: Топик состояния ros2_control и joint батареи (источник напряжения пакета, ADR перцепции battery_state).
JOINT_STATES_TOPIC = "/dynamic_joint_states"
BATTERY_JOINT = "battery"
#: Что просили показать -> какие имена полей считаются этим измерением (имя в ROS не выдумываем: берём то, что есть).
VESC_MEASURES: Dict[str, Tuple[str, ...]] = {
    "voltage_v": ("voltage_input", "input_voltage", "voltage", "vin"),
    "current_a": ("current_motor", "motor_current", "current"),
    "rpm": ("rpm", "erpm"),
    "temp_fet_c": ("temperature_pcb", "temp_fet", "temperature_fet", "fet_temperature"),
    "temp_motor_c": ("temperature_motor", "temp_motor", "motor_temperature"),
}
#: Измерения, которые ROS на этом роботе не отдаёт, пока их не выведет vesc_nexus (причина - в тексте отказа).
VESC_MISSING_HINT = (
    "в ROS их нет: vesc_nexus отдаёт в ros2_control только скорость и позицию колеса; прямо с CAN (can0) тул не читает"
)


def vesc_from_joint_states(joint_names: Sequence[str], interface_values: Sequence[Any]) -> Dict[str, Any]:
    """Колёса и батарея из полей ``DynamicJointState``: ``{"wheels": {метка: {интерфейс: значение}}, "battery": {}}``.

    Значения как в ROS (``velocity`` - скорость колеса в единицах ros2_control, без пересчёта в rpm), интерфейсы -
    какие есть. Joint без числовых значений пропускается.
    """
    wheels: Dict[str, Dict[str, float]] = {}
    battery: Dict[str, float] = {}
    for joint, iv in zip(joint_names, interface_values):
        values = {
            str(n): float(v)
            for n, v in zip(getattr(iv, "interface_names", ()), getattr(iv, "values", ()))
            if isinstance(v, (int, float)) and math.isfinite(v)
        }
        if joint in WHEEL_JOINTS:
            wheels[WHEEL_JOINTS[joint]] = values
        elif joint == BATTERY_JOINT:
            battery = values
    return {"wheels": wheels, "battery": battery}


def _pick(values: Dict[str, float], aliases: Tuple[str, ...]) -> Optional[float]:
    for name in aliases:
        if name in values:
            return round(values[name], 3)
    return None


def vesc_report(source: Dict[str, Any]) -> Dict[str, Any]:
    """Сводка VESC: по каждому колесу измерения из :data:`VESC_MEASURES` (``None`` - в ROS нет), «сырые» интерфейсы и
    список измерений, которых нет ни у одного колеса. Напряжение пакета берётся у колеса, а если у колёс его нет - у
    joint батареи."""
    wheels = source.get("wheels", {})
    battery = source.get("battery", {})
    rows: Dict[str, Dict[str, Any]] = {}
    for label in WHEEL_ORDER:
        if label not in wheels:
            continue
        raw = wheels[label]
        row: Dict[str, Any] = {m: _pick(raw, aliases) for m, aliases in VESC_MEASURES.items()}
        row["raw"] = {k: round(v, 4) for k, v in raw.items()}
        rows[label] = row
    pack = _pick(battery, VESC_MEASURES["voltage_v"])
    missing = [m for m in VESC_MEASURES if rows and all(r[m] is None for r in rows.values())]
    if pack is not None and "voltage_v" in missing:
        missing.remove("voltage_v")
    return {"wheels": rows, "pack_voltage_v": pack, "missing": missing}


_MEASURE_TITLES = {
    "voltage_v": "напряжение, В",
    "current_a": "ток, А",
    "rpm": "обороты",
    "temp_fet_c": "температура FET, °C",
    "temp_motor_c": "температура мотора, °C",
}


def vesc_message(report: Dict[str, Any]) -> str:
    """Готовый текст телеметрии: числа из отчёта, отсутствующее названо отсутствующим."""
    if not report["wheels"]:
        return "В ROS нет данных по колёсам: в /dynamic_joint_states не нашлось joint колёс."
    lines = []
    for label, row in report["wheels"].items():
        raw = ", ".join(f"{k}={v}" for k, v in row["raw"].items()) or "интерфейсов нет"
        got = ", ".join(f"{_MEASURE_TITLES[m]} {row[m]}" for m in VESC_MEASURES if row[m] is not None)
        lines.append(f"{label}: {got + '; ' if got else ''}интерфейсы ROS: {raw}")
    if report["pack_voltage_v"] is not None:
        lines.append(f"напряжение пакета (joint battery): {report['pack_voltage_v']} В")
    if report["missing"]:
        names = ", ".join(_MEASURE_TITLES[m] for m in report["missing"])
        lines.append(f"Нет данных: {names} ({VESC_MISSING_HINT}).")
    return "\n".join(lines)


# ----------------------------------------------------------------------------------------------------------------
# rclpy-адаптер
# ----------------------------------------------------------------------------------------------------------------


@dataclass
class Sample:
    """Один пойманный кадр: время прихода и сериализованные байты."""

    at: float
    size: int
    raw: bytes = b""


class RosProbe:
    """Чтение графа и топиков через ноду процесса (rclpy). Нет rclpy / нода не ROS - ``ROS_UNAVAILABLE`` (честно)."""

    def __init__(self, node: Any) -> None:
        self._node = node

    def available(self) -> bool:
        return self._node is not None and hasattr(self._node, "get_topic_names_and_types")

    def topics(self) -> List[Tuple[str, List[str]]]:
        return list(self._node.get_topic_names_and_types())

    def services(self) -> List[Tuple[str, List[str]]]:
        return list(self._node.get_service_names_and_types())

    def topic_type(self, topic: str) -> Optional[str]:
        for name, types in self.topics():
            if name == topic and types:
                return types[0]
        return None

    def _publisher_qos(self, topic: str) -> Any:
        try:
            infos = self._node.get_publishers_info_by_topic(topic)
        except Exception:  # noqa: BLE001 - rclpy бросает разное; без QoS издателя берём sensor_data
            infos = []
        return infos[0].qos_profile if infos else None

    def publishers_count(self, topic: str) -> int:
        try:
            return len(self._node.get_publishers_info_by_topic(topic))
        except Exception:  # noqa: BLE001
            return 0

    def collect(
        self,
        topic: str,
        type_name: str,
        count: int,
        timeout_s: float,
        keep_raw: bool = True,
        clock: Callable[[], float] = time.monotonic,
    ) -> List[Sample]:
        """До ``count`` сообщений или до ``timeout_s``; подписка всегда снимается. ``keep_raw=False`` - только время и
        размер (замер ``bw`` картинок не копит гигабайты в памяти). Кадр крупнее :data:`BULK_MESSAGE_BYTES` байты не
        хранит ни при каком ``keep_raw``: содержимое такого сообщения в ответ не идёт."""
        from rclpy.qos import qos_profile_sensor_data  # noqa: PLC0415 - rclpy только в рантайме ноды
        from rosidl_runtime_py.utilities import get_message  # noqa: PLC0415

        samples: List[Sample] = []
        done = threading.Event()

        def on_raw(data: bytes) -> None:
            size = len(data)
            keep = keep_raw and size <= BULK_MESSAGE_BYTES
            samples.append(Sample(clock(), size, bytes(data) if keep else b""))
            if len(samples) >= count:
                done.set()

        qos = self._publisher_qos(topic) or qos_profile_sensor_data
        sub = self._node.create_subscription(get_message(type_name), topic, on_raw, qos, raw=True)
        try:
            done.wait(timeout=timeout_s)
        finally:
            self._node.destroy_subscription(sub)
        return samples[:count]

    @staticmethod
    def render(type_name: str, raw: bytes) -> str:
        """Сообщение текстом (YAML); битое - честная пометка вместо молчания."""
        from rclpy.serialization import deserialize_message  # noqa: PLC0415
        from rosidl_runtime_py import message_to_yaml  # noqa: PLC0415
        from rosidl_runtime_py.utilities import get_message  # noqa: PLC0415

        try:
            return message_to_yaml(deserialize_message(raw, get_message(type_name))).strip()
        except Exception as exc:  # noqa: BLE001
            return f"[не удалось разобрать сообщение: {type(exc).__name__}]"

    def read_joint_states(self, timeout_s: float = 2.0) -> Optional[Tuple[Sequence[str], Sequence[Any]]]:
        """Одно сообщение ``/dynamic_joint_states`` -> ``(joint_names, interface_values)`` или ``None``."""
        from rclpy.serialization import deserialize_message  # noqa: PLC0415
        from rosidl_runtime_py.utilities import get_message  # noqa: PLC0415

        type_name = self.topic_type(JOINT_STATES_TOPIC)
        if type_name is None:
            return None
        samples = self.collect(JOINT_STATES_TOPIC, type_name, 1, timeout_s)
        if not samples:
            return None
        msg = deserialize_message(samples[0].raw, get_message(type_name))
        return list(msg.joint_names), list(msg.interface_values)
