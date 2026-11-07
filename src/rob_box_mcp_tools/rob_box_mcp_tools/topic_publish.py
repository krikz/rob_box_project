#!/usr/bin/env python3
"""topic_publish.py - предохранитель записи в топики для пульта TG и ТАРС (issue #4516, шаг 2а: без движения).

Решение Шифу 06.11: «Телега - считай админка: читать и писать топики». Безопасность держит КОД (ADR-0148): модель
называет топик и поля, а что можно публиковать, решает этот модуль. Всё знание - здесь, в одном месте:

* :data:`FORBIDDEN_TOPICS` / :data:`FORBIDDEN_TYPES` - чего публиковать нельзя никогда: движение (``*cmd_vel*``),
  навигация,
  ros2_control, VESC/CAN, ``/tf``, служебные топики, действия и сервисы. Движение - отдельный будущий шаг через
  ``twist_mux`` с защитой по лидару, НЕ здесь.
* топик должен быть в ROS-графе и иметь издателя или подписчика; тип берётся из графа (от модели - никогда);
* поля сообщения проверяются по интерфейсу типа строго: неизвестное поле, неверный тип значения - отказ с причиной;
* ``count`` от 1 до :data:`COUNT_MAX`, пауза между публикациями фиксирована (:data:`PAUSE_S`);
* без ``confirm`` тул только спрашивает (``knowledge.TOOL_QUESTIONS``), публикует только после «да» человека;
* каждый исход (опубликовано, отказ, вопрос, сбой) - строка JSONL журнала (:class:`Journal`) с автором;
* фразу результата строит код по факту публикации (:func:`published_text`, :func:`refusal_text`).

Чистая часть не импортирует rclpy и гоняется в unit-тестах; rclpy - только в :class:`RclpyPublisher`.
"""

from __future__ import annotations

import contextlib
import contextvars
import fnmatch
import json
import logging
import math
import os
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, Iterator, List, Mapping, Optional, Sequence, Tuple

from . import ros_introspect as ri

LOG = logging.getLogger("topic_publish")

#: Потолки: сообщений за вызов и фиксированная пауза между ними (с), ожидание подписчика перед первой публикацией (с).
COUNT_MAX = 5
PAUSE_S = 0.2
DISCOVERY_WAIT_S = 1.0
#: Длина поля ``fields`` в строке журнала (знаков), потолок размера файла журнала (байт).
FIELDS_LOG_CHARS = 300
JOURNAL_MAX_BYTES = 10 * 1024 * 1024
JOURNAL_DIR_ENV = "ROB_BOX_ADMIN_AUDIT_DIR"
JOURNAL_DIR_DEFAULT = "/var/log/rob_box_admin"
JOURNAL_FILE = "topic_publish.jsonl"

# Категории запрета: ключ -> как о нём говорит фраза отказа.
MOTION, DRIVES, NAV, SYSTEM = "motion", "drives", "nav", "system"
CATEGORY_TEXT: Mapping[str, str] = {
    MOTION: "топик движения",
    DRIVES: "топик приводов или безопасности",
    NAV: "топик навигации",
    SYSTEM: "служебный топик",
}

#: Запрет по ИМЕНИ топика: ``fnmatch`` по имени в нижнем регистре (``*`` покрывает и ``/``), категория. Новый шаблон
#: добавляется здесь и больше нигде.
FORBIDDEN_TOPICS: Tuple[Tuple[str, str], ...] = (
    # движение: все ``cmd_vel`` (``/cmd_vel``, ``/cmd_vel_emergency``, ``/cmd_vel_joy|quest|web|voice``, любой ns)
    ("*cmd_vel*", MOTION),
    ("*teleop*", MOTION),
    ("/joy", MOTION),
    ("*/joy", MOTION),
    ("*_lock", MOTION),
    # приводы и безопасность: ros2_control, VESC, CAN, аварийные и тормозные
    ("*vesc*", DRIVES),
    ("*motor*", DRIVES),
    ("*wheel*", DRIVES),
    ("*emergency*", DRIVES),
    ("*estop*", DRIVES),
    ("*e_stop*", DRIVES),
    ("*safety*", DRIVES),
    ("*brake*", DRIVES),
    ("/can", DRIVES),
    ("/can/*", DRIVES),
    ("/can[0-9]*", DRIVES),
    ("*/can", DRIVES),
    ("*/can/*", DRIVES),
    ("*/can[0-9]*", DRIVES),
    ("*socketcan*", DRIVES),
    ("/dynamic_joint_states", DRIVES),
    ("/joint_states", DRIVES),
    ("*/joint_states", DRIVES),
    ("*_controller", DRIVES),
    ("*_controller/*", DRIVES),
    ("/controller_manager", DRIVES),
    ("/controller_manager/*", DRIVES),
    ("*joint_trajectory*", DRIVES),
    # навигация: цели, действия, серверы Nav2
    ("/goal_pose", NAV),
    ("*/goal_pose", NAV),
    ("/initialpose", NAV),
    ("*/initialpose", NAV),
    ("/navigate_*", NAV),
    ("*/navigate_*", NAV),
    ("/follow_*", NAV),
    ("/bt_navigator", NAV),
    ("/bt_navigator/*", NAV),
    ("/controller_server/*", NAV),
    ("/planner_server/*", NAV),
    ("/behavior_server/*", NAV),
    ("/waypoint_follower/*", NAV),
    ("/velocity_smoother/*", NAV),
    ("/collision_monitor/*", NAV),
    ("/lifecycle_manager*", NAV),
    # служебное: действия и сервисы, время, tf, параметры, лог, шина тулов MCP, описание робота
    ("*/_action/*", SYSTEM),
    ("/_action/*", SYSTEM),
    ("*/_service_event", SYSTEM),
    ("*/transition_event", SYSTEM),
    ("/tf", SYSTEM),
    ("/tf_static", SYSTEM),
    ("/clock", SYSTEM),
    ("/parameter_events", SYSTEM),
    ("/rosout", SYSTEM),
    ("/mcp/*", SYSTEM),
    ("*/robot_description", SYSTEM),
)

#: Запрет по ТИПУ сообщения (``fnmatch`` по ``пакет/msg/Имя``): даже под безобидным именем эти типы двигают робота или
#: трогают приводы. Тип берётся из графа, модель его не задаёт.
FORBIDDEN_TYPES: Tuple[Tuple[str, str], ...] = (
    ("geometry_msgs/msg/Twist*", MOTION),
    ("ackermann_msgs/*", MOTION),
    ("sensor_msgs/msg/Joy*", MOTION),
    ("rob_box_supervisor_msgs/msg/TeleopHeartbeat", MOTION),
    ("control_msgs/*", DRIVES),
    ("trajectory_msgs/*", DRIVES),
    ("sensor_msgs/msg/JointState", DRIVES),
    ("can_msgs/*", DRIVES),
    ("vesc*", DRIVES),
    ("robot_sensor_hub_msg/msg/DeviceCommand", DRIVES),
    ("nav2_msgs/*", NAV),
    ("*/action/*", SYSTEM),
    ("*/srv/*", SYSTEM),
    ("action_msgs/*", SYSTEM),
    ("tf2_msgs/*", SYSTEM),
    ("rcl_interfaces/*", SYSTEM),
    ("lifecycle_msgs/*", SYSTEM),
    ("rosgraph_msgs/*", SYSTEM),
    ("rob_box_supervisor_msgs/msg/Command", SYSTEM),
)

#: Отказы (машинные ключи; фразы - ``phrases/ru.yaml``, ``local.rejected.<ключ>``, слот ``summary``).
(
    TOPIC_FORBIDDEN,
    TYPE_FORBIDDEN,
    NO_SUCH_TOPIC,
    NO_ENDPOINTS,
    AMBIGUOUS_TYPE,
    BAD_FIELDS,
    BAD_COUNT,
    BAD_TOPIC,
    PUBLISH_FAILED,
    NEEDS_CONFIRMATION,
) = (
    "topic_forbidden",
    "type_forbidden",
    "no_such_topic",
    "no_endpoints",
    "ambiguous_type",
    "bad_fields",
    "bad_count",
    "bad_topic",
    "publish_failed",
    "needs_publish_confirmation",
)
#: Причины, которые остаются в тексте через ``ri`` (одни и те же ключи у тулов чтения).
ROS_UNAVAILABLE, TYPE_UNAVAILABLE = ri.ROS_UNAVAILABLE, ri.TYPE_UNAVAILABLE

#: Кто вызвал: подписанный ``sender`` запроса MCP -> метка автора в журнале. Нет в таблице - честно ``unknown``.
AUTHORS: Mapping[str, str] = {"telegram_agent": "TG", "avatar_supervisor": "ТАРС"}
UNKNOWN_AUTHOR = "unknown"
_CALLER: "contextvars.ContextVar[Optional[str]]" = contextvars.ContextVar("topic_publish_caller", default=None)


@contextlib.contextmanager
def caller_sender(sender: Optional[str]) -> Iterator[None]:
    """Пока идёт вызов тула, ``sender`` подписанного запроса MCP доступен как автор (контекст держит ``mcp_server``)."""
    token = _CALLER.set(sender)
    try:
        yield
    finally:
        _CALLER.reset(token)


def execute_as(registry: Any, sender: Optional[str], tool_name: str, parameters: Mapping[str, Any]) -> Any:
    """``registry.execute`` под ``sender`` подписанного запроса: тулы, что пишут журнал, берут автора из контекста."""
    with caller_sender(sender):
        return registry.execute(tool_name, **parameters)


def current_author() -> str:
    """Метка автора по ``sender`` текущего вызова; не знаем - ``unknown``."""
    return AUTHORS.get(_CALLER.get() or "", UNKNOWN_AUTHOR)


# ----------------------------------------------------------------------------------------------------------------
# Запреты
# ----------------------------------------------------------------------------------------------------------------


def _match(value: str, rules: Sequence[Tuple[str, str]]) -> Optional[str]:
    low = value.lower()
    for pattern, category in rules:
        if fnmatch.fnmatchcase(low, pattern.lower()):
            return category
    return None


def forbidden_topic_category(topic: str) -> Optional[str]:
    """Категория запрета по имени топика или ``None``. Имя приводится к нижнему регистру, хвостовой ``/`` не спасает."""
    return _match(topic.rstrip("/") or "/", FORBIDDEN_TOPICS)


def forbidden_type_category(type_name: str) -> Optional[str]:
    """Категория запрета по типу сообщения или ``None``."""
    return _match(type_name, FORBIDDEN_TYPES)


# ----------------------------------------------------------------------------------------------------------------
# Проверка полей по интерфейсу типа
# ----------------------------------------------------------------------------------------------------------------

_INT_BITS: Mapping[str, Tuple[int, bool]] = {
    "int8": (8, True),
    "uint8": (8, False),
    "int16": (16, True),
    "uint16": (16, False),
    "int32": (32, True),
    "uint32": (32, False),
    "int64": (64, True),
    "uint64": (64, False),
    "octet": (8, False),
    "byte": (8, False),
    "char": (8, False),
}
_FLOATS = ("float", "double", "long double")
_STRINGS = ("string", "wstring")


def _split_array(type_str: str) -> Tuple[str, bool]:
    """``sequence<double>`` / ``double[3]`` / ``double[<=3]`` -> ``("double", True)``; скаляр -> ``(type, False)``."""
    if type_str.startswith("sequence<") and type_str.endswith(">"):
        return type_str[len("sequence<") : -1], True
    if type_str.endswith("]") and "[" in type_str:
        return type_str[: type_str.index("[")], True
    return type_str, False


def _primitive_error(base: str, value: Any) -> Optional[str]:
    """Ошибка значения для примитивного типа или ``None``. Строго: ``"5"`` не число, ``3.7`` не целое, ``1`` не bool."""
    root = base.split("<")[0]  # string<=10 -> string
    if root == "boolean":
        return None if isinstance(value, bool) else "ждали true/false"
    if root in _INT_BITS:
        bits, signed = _INT_BITS[root]
        if isinstance(value, bool) or not isinstance(value, int):
            return "ждали целое число"
        low, high = (-(2 ** (bits - 1)), 2 ** (bits - 1) - 1) if signed else (0, 2**bits - 1)
        return None if low <= value <= high else f"целое вне диапазона {low}..{high}"
    if root in _FLOATS:
        ok = isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
        return None if ok else "ждали конечное число"
    if root in _STRINGS:
        return None if isinstance(value, str) else "ждали строку"
    return f"неизвестный примитив {base}"


def _is_primitive(base: str) -> bool:
    root = base.split("<")[0]
    return root == "boolean" or root in _INT_BITS or root in _FLOATS or root in _STRINGS


def _nested_class(parent: Any, name: str, base: str, load: Callable[[str], Any]) -> Any:
    """Класс вложенного сообщения: по экземпляру поля, иначе по имени типа (``pkg/Name`` -> ``pkg/msg/Name``)."""
    if "/" in base:
        pkg, _, rest = base.partition("/")
        full = base if "/msg/" in base else f"{pkg}/msg/{rest}"
        return load(full)
    return type(getattr(parent(), name))


def _value_error(cls: Any, name: str, type_str: str, value: Any, load: Callable[[str], Any]) -> Optional[str]:
    base, is_array = _split_array(type_str)
    if is_array:
        if not isinstance(value, (list, tuple)):
            return "ждали список"
        for i, item in enumerate(value):
            err = _scalar_error(cls, name, base, item, load)
            if err:
                return f"[{i}]: {err}"
        return None
    return _scalar_error(cls, name, base, value, load)


def _scalar_error(cls: Any, name: str, base: str, value: Any, load: Callable[[str], Any]) -> Optional[str]:
    if _is_primitive(base):
        return _primitive_error(base, value)
    if not isinstance(value, dict):
        return f"ждали объект {base}"
    return validate_fields(_nested_class(cls, name, base, load), value, load)


def validate_fields(cls: Any, fields: Any, load: Callable[[str], Any]) -> Optional[str]:
    """Строгая проверка ``fields`` по интерфейсу класса сообщения (``get_fields_and_field_types``).

    Возвращает понятную ошибку (``поле 'x': ...``) или ``None``. Неизвестное поле и значение не того типа - отказ, а не
    молчаливое приведение (``set_message_fields`` сам превратил бы ``3.7`` в ``3``). ``load`` - загрузчик класса по
    имени типа для вложенных сообщений в массивах."""
    if not isinstance(fields, dict):
        return "fields должен быть объектом {имя поля: значение}"
    spec: Mapping[str, str] = cls.get_fields_and_field_types()
    for name, value in fields.items():
        if name not in spec:
            known = ", ".join(sorted(spec)) or "полей нет"
            return f"неизвестное поле '{name}' (у типа есть: {known})"
        err = _value_error(cls, name, spec[name], value, load)
        if err:
            return f"поле '{name}': {err}"
    return None


# ----------------------------------------------------------------------------------------------------------------
# Фразы (строит код по факту)
# ----------------------------------------------------------------------------------------------------------------


def times_word(n: int) -> str:
    """«1 раз», «2 раза», «5 раз», «11 раз», «22 раза»."""
    if n % 10 == 1 and n % 100 != 11:
        return f"{n} раз"
    if 2 <= n % 10 <= 4 and not 12 <= n % 100 <= 14:
        return f"{n} раза"
    return f"{n} раз"


def published_text(topic: str, type_name: str, published: int, subscribers: int) -> str:
    """«Опубликовал в /x (std_msgs/msg/String) 1 раз, подписчиков 2.»; без подписчиков - честно, что слушать некому."""
    text = f"Опубликовал в {topic} ({type_name}) {times_word(published)}, подписчиков {subscribers}"
    return text + ("." if subscribers else ": слушателей нет, сообщение никто не принял.")


def refusal_text(reason: str, topic: Any, detail: str = "", category: str = "") -> str:
    """Фраза отказа по причине; запрет называет категорию («топик движения»)."""
    if reason in (TOPIC_FORBIDDEN, TYPE_FORBIDDEN):
        return f"Отказ: {topic} - {CATEGORY_TEXT[category]}, публиковать запрещено."
    return f"Отказ: {detail}" if detail else "Отказ: не опубликовал."


def question_text(topic: str, type_name: str, fields_preview: str, count: int) -> str:
    """Подтверждение одной строкой; ответ - «да, публикуй»."""
    return f"Публикую в {topic} ({type_name}) {fields_preview}, {times_word(count)}? Скажи «да, публикуй»."


# ----------------------------------------------------------------------------------------------------------------
# Журнал
# ----------------------------------------------------------------------------------------------------------------


def fields_preview(fields: Any, sanitize: Callable[[str], str] = lambda s: s, limit: int = FIELDS_LOG_CHARS) -> str:
    """Поля одной строкой для журнала и вопроса: JSON, через редактор секретов, не длиннее ``limit``."""
    try:
        raw = json.dumps(fields, ensure_ascii=False, default=str, sort_keys=True)
    except (TypeError, ValueError):
        raw = repr(fields)
    return ri.fit_text(sanitize(raw), limit)[0]


class Journal:
    """Журнал действий: строка JSONL на исход (время UTC, автор, топик, тип, поля, исход).

    Строка всегда идёт в лог процесса (его собирает promtail в Loki), а в файл ``<каталог>/topic_publish.jsonl`` -
    если каталог доступен. Файл не записался - ``False`` и честная метка в ответе тула, публикация не молчит об этом.
    """

    def __init__(
        self,
        directory: Optional[str] = None,
        clock: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
        logger: logging.Logger = LOG,
    ) -> None:
        self._dir = Path(directory or os.environ.get(JOURNAL_DIR_ENV) or JOURNAL_DIR_DEFAULT)
        self._clock = clock
        self._logger = logger
        self._lock = threading.Lock()

    @property
    def path(self) -> Path:
        return self._dir / JOURNAL_FILE

    def record(self, entry: Mapping[str, Any]) -> bool:
        """Дописать строку; ``True`` - попала и в файл."""
        row = {"ts": self._clock().astimezone(timezone.utc).isoformat(timespec="seconds"), **entry}
        line = json.dumps(row, ensure_ascii=False, default=str, sort_keys=False)
        self._logger.info("TOPIC_PUBLISH_AUDIT %s", line)
        try:
            with self._lock:
                self._dir.mkdir(parents=True, exist_ok=True)
                self._rotate()
                with self.path.open("a", encoding="utf-8") as fh:
                    fh.write(line + "\n")
            return True
        except OSError as exc:
            self._logger.error("topic_publish: журнал в файл не записан (%s): %s", self.path, exc)
            return False

    def _rotate(self) -> None:
        try:
            if self.path.stat().st_size > JOURNAL_MAX_BYTES:
                self.path.replace(self.path.with_suffix(".jsonl.1"))
        except FileNotFoundError:
            pass


# ----------------------------------------------------------------------------------------------------------------
# rclpy-адаптер публикации
# ----------------------------------------------------------------------------------------------------------------


@dataclass
class PublishReport:
    """Факт публикации: сколько сообщений ушло и сколько подписчиков сопоставилось с издателем."""

    published: int
    subscribers: int


class RclpyPublisher:
    """Публикация через ноду процесса: издатель на время вызова, QoS как у подписчика (иначе издателя) топика."""

    def __init__(self, node: Any, sleep: Callable[[float], None] = time.sleep) -> None:
        self._node = node
        self._sleep = sleep

    def message_class(self, type_name: str) -> Any:
        return ri.load_message_type(type_name)

    def _qos(self, topic: str) -> Any:
        from rclpy.qos import QoSProfile  # noqa: PLC0415 - rclpy только в рантайме ноды

        for getter in ("get_subscriptions_info_by_topic", "get_publishers_info_by_topic"):
            try:
                infos = getattr(self._node, getter)(topic)
            except Exception:  # noqa: BLE001 - rclpy бросает разное; без QoS берём умолчание
                infos = []
            if infos:
                return infos[0].qos_profile
        return QoSProfile(depth=10)

    @staticmethod
    def build(cls: Any, fields: Dict[str, Any]) -> Any:
        """Сообщение из полей (поля уже прошли :func:`validate_fields`); ошибка сборки - ``ValueError`` с причиной."""
        from rosidl_runtime_py.set_message import set_message_fields  # noqa: PLC0415

        msg = cls()
        try:
            set_message_fields(msg, fields)
        except Exception as exc:  # noqa: BLE001 - AttributeError/AssertionError/ValueError/TypeError из rosidl
            raise ValueError(f"{type(exc).__name__}: {exc}") from exc
        return msg

    def publish(self, topic: str, cls: Any, fields: Dict[str, Any], count: int, has_listeners: bool) -> PublishReport:
        """``count`` публикаций с паузой :data:`PAUSE_S`; издатель всегда снимается."""
        msg = self.build(cls, fields)
        pub = self._node.create_publisher(cls, topic, self._qos(topic))
        sent = 0
        try:
            if has_listeners:
                self._await_match(pub)
            for i in range(count):
                if i:
                    self._sleep(PAUSE_S)
                pub.publish(msg)
                sent += 1
            return PublishReport(sent, int(pub.get_subscription_count()))
        except Exception as exc:  # noqa: BLE001
            raise PublishInterrupted(sent, f"{type(exc).__name__}: {exc}") from exc
        finally:
            self._node.destroy_publisher(pub)

    def _await_match(self, pub: Any) -> None:
        """Подписчики находят нового издателя не мгновенно: ждём совпадения не дольше :data:`DISCOVERY_WAIT_S`."""
        deadline = time.monotonic() + DISCOVERY_WAIT_S
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            self._sleep(0.05)


class PublishInterrupted(Exception):
    """Публикация оборвалась после ``sent`` успешных сообщений."""

    def __init__(self, sent: int, detail: str) -> None:
        super().__init__(detail)
        self.sent = sent
        self.detail = detail


# ----------------------------------------------------------------------------------------------------------------
# Предохранитель
# ----------------------------------------------------------------------------------------------------------------


@dataclass
class Outcome:
    """Исход вызова: ``done`` или ``rejected`` с причиной, фраза и данные для результата тула."""

    status: str
    reason: str
    text: str
    data: Dict[str, Any] = field(default_factory=dict)

    @property
    def done(self) -> bool:
        return self.status == "done"


def _int_count(value: Any) -> Optional[int]:
    """``count`` или ``None``, если не целое от 1 до :data:`COUNT_MAX`; пусто - 1."""
    if value is None:
        return 1
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= COUNT_MAX:
        return None
    return value


class TopicPublishGuard:
    """Один вызов ``topic_publish``: проверки по порядку, вопрос, публикация, журнал.

    ``probe`` - :class:`rob_box_mcp_tools.ros_introspect.RosProbe`, ``publisher`` - :class:`RclpyPublisher` (в тестах
    подменяются), ``sanitize`` - редактор секретов для строки полей.
    """

    def __init__(
        self,
        probe: Any,
        publisher: Any,
        journal: Journal,
        sanitize: Callable[[str], str] = lambda s: s,
        author: Callable[[], str] = current_author,
    ) -> None:
        self._probe = probe
        self._publisher = publisher
        self._journal = journal
        self._sanitize = sanitize
        self._author = author

    def run(self, topic: Any, fields: Any, count: Any = None, confirm: bool = False) -> Outcome:
        ctx: Dict[str, Any] = {"topic": topic if isinstance(topic, str) else repr(topic)[:200]}
        ctx["fields"] = fields_preview(fields, self._sanitize)
        outcome = self._decide(topic, fields, count, confirm, ctx)
        written = self._journal.record(
            {
                "author": self._author(),
                **ctx,
                "outcome": outcome.data.get("outcome", "published" if outcome.done else "refused"),
                "reason": outcome.reason,
                "text": outcome.text,
            }
        )
        outcome.data["journal"] = "file" if written else "log_only"
        return outcome

    # --- проверки -------------------------------------------------------------------------------------------
    def _refuse(self, reason: str, topic: Any, detail: str = "", category: str = "", **extra: Any) -> Outcome:
        text = refusal_text(reason, topic, detail, category)
        data = {"status": "rejected", "reason": reason, "summary": text, "outcome": "refused", **extra}
        return Outcome("rejected", reason, text, data)

    def _decide(self, topic: Any, fields: Any, count: Any, confirm: Any, ctx: Dict[str, Any]) -> Outcome:
        if not ri.valid_topic_name(topic):
            return self._refuse(BAD_TOPIC, topic, f"имя топика {topic!r} не годится: нужно вида /имя/подимя.")
        category = forbidden_topic_category(topic)
        if category:
            return self._refuse(TOPIC_FORBIDDEN, topic, category=category)
        n = _int_count(count)
        if n is None:
            return self._refuse(BAD_COUNT, topic, f"count должен быть целым от 1 до {COUNT_MAX}.")
        ctx["count"] = n
        if not getattr(self._probe, "available", lambda: False)():
            return self._refuse(ROS_UNAVAILABLE, topic, "ROS на этом узле недоступен, публиковать нечем.")
        return self._graph_checks(topic, fields, n, confirm, ctx)

    def _graph_checks(self, topic: str, fields: Any, n: int, confirm: Any, ctx: Dict[str, Any]) -> Outcome:
        types = self._probe.topic_types(topic)
        pubs, subs = self._probe.publishers_count(topic), self._probe.subscribers_count(topic)
        if not types:
            return self._refuse(NO_SUCH_TOPIC, topic, f"топика {topic} в ROS-графе нет.")
        if len(set(types)) > 1:
            return self._refuse(AMBIGUOUS_TYPE, topic, f"у {topic} несколько типов ({', '.join(types)}), тип не ясен.")
        type_name = types[0]
        ctx["type"] = type_name
        category = forbidden_type_category(type_name)
        if category:
            return self._refuse(TYPE_FORBIDDEN, topic, category=category, type=type_name)
        if pubs + subs == 0:
            return self._refuse(NO_ENDPOINTS, topic, f"у {topic} ({type_name}) нет ни издателей, ни подписчиков.")
        return self._fields_checked(topic, type_name, fields, n, confirm)

    def _fields_checked(self, topic: str, type_name: str, fields: Any, n: int, confirm: Any) -> Outcome:
        try:
            cls = self._publisher.message_class(type_name)
        except ri.TypeUnavailable as exc:
            return self._refuse(TYPE_UNAVAILABLE, topic, exc.message(), type=type_name)
        error = validate_fields(cls, fields, self._publisher.message_class)
        if error:
            return self._refuse(BAD_FIELDS, topic, f"поля не подходят к {type_name}: {error}.", type=type_name)
        preview = fields_preview(fields, self._sanitize)
        if confirm is not True:
            text = question_text(topic, type_name, preview, n)
            data = {"status": "rejected", "reason": NEEDS_CONFIRMATION, "question": text, "outcome": "asked"}
            return Outcome("rejected", NEEDS_CONFIRMATION, text, data)
        return self._publish(topic, type_name, cls, fields, n)

    def _publish(self, topic: str, type_name: str, cls: Any, fields: Dict[str, Any], n: int) -> Outcome:
        has_listeners = self._probe.subscribers_count(topic) > 0
        try:
            report = self._publisher.publish(topic, cls, fields, n, has_listeners)
        except PublishInterrupted as exc:
            tail = f"ушло {exc.sent} из {n}, затем сбой: {exc.detail}"
            return self._refuse(
                PUBLISH_FAILED, topic, f"{topic} ({type_name}): {tail}.", outcome="failed", sent=exc.sent
            )
        except ValueError as exc:  # сборка сообщения (set_message_fields) не приняла поля
            return self._refuse(BAD_FIELDS, topic, f"поля не подходят к {type_name}: {exc}.", type=type_name)
        text = published_text(topic, type_name, report.published, report.subscribers)
        data = {
            "status": "done",
            "summary": text,
            "topic": topic,
            "type": type_name,
            "published": report.published,
            "subscribers": report.subscribers,
        }
        return Outcome("done", "", text, data)


__all__: List[str] = [
    "AUTHORS",
    "COUNT_MAX",
    "FORBIDDEN_TOPICS",
    "FORBIDDEN_TYPES",
    "Journal",
    "NEEDS_CONFIRMATION",
    "Outcome",
    "PAUSE_S",
    "PublishInterrupted",
    "PublishReport",
    "RclpyPublisher",
    "TopicPublishGuard",
    "caller_sender",
    "execute_as",
    "current_author",
    "fields_preview",
    "forbidden_topic_category",
    "forbidden_type_category",
    "published_text",
    "times_word",
    "validate_fields",
]
