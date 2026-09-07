"""Тесты TarsPanelDispatcher (issue #2113, quest #2112).

Цель — pure-логика и wire-контракт:

* ``build_panel_url`` собирает правильный URL для Prometheus и Loki;
* ``_on_panel_request`` парсит вход, публикует ``/avatar/tars/panel_url``;
* невалидный JSON / пустой query / неизвестный datasource → ``status="error"``
  с понятной причиной;
* ``register_tool`` добавляет ``show_metrics`` в registry и handler
  публикует запрос + возвращает dict-статус.

ROS-моки: тот же ``FakeNode``/``FakePublisher`` что и в
``test_supervisor_node.py`` (см. conftest.py: ``_install_ros_mocks``).
Запускается через ``pytest -v``.

Запуск::

    cd src/rob_box_supervisor
    PYTHONPATH=. pytest -v test/unit/test_tars_panel.py
"""

from __future__ import annotations

import json
import sys
import types
from dataclasses import dataclass
from typing import Any, Callable
from unittest.mock import MagicMock

import pytest

# ── ROS-стабы: достаточны для импорта std_msgs.msg.String + rclpy.node ──


def _install_ros_stubs() -> None:
    """Подсовывает std_msgs.msg.String и rclpy.node.Node как MagicMock.

    Делает это ДО импорта rob_box_supervisor.tars_panel — иначе
    ``from std_msgs.msg import String`` падает на CI без ROS.
    """
    # std_msgs.msg.String
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type(  # type: ignore[attr-defined]
        "String",
        (),
        {"__init__": lambda self: setattr(self, "data", "")},
    )
    std_msgs.msg = std_msgs_msg  # type: ignore[attr-defined]
    sys.modules.setdefault("std_msgs", std_msgs)
    sys.modules.setdefault("std_msgs.msg", std_msgs_msg)

    # rclpy.node — моки достаточно для ``class Node`` в type hints.
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")

    class _StubNode:  # pragma: no cover — не используется в этом тесте
        pass

    rclpy_node.Node = _StubNode
    sys.modules.setdefault("rclpy", rclpy)
    sys.modules.setdefault("rclpy.node", rclpy_node)


_install_ros_stubs()

# Подсовываем минимальный модуль ``rob_box_harness.tools`` через sys.modules
# ДО импорта rob_box_supervisor.tars_panel — внутри register_tool()
# делается ``from rob_box_harness.tools import ToolSpec``. Полная цепочка
# ``rob_box_harness → health.py → rob_box_llm`` тянет зависимости, которых
# на CI без ROS нет; минимальный stub покрывает только то, что dispatcher
# реально использует (ToolSpec dataclass + ToolHandler typing).
_rob_box_harness = types.ModuleType("rob_box_harness")
_rob_box_harness_tools = types.ModuleType("rob_box_harness.tools")


@dataclass(frozen=True)
class _StubToolSpec:
    """Упрощённая копия rob_box_harness.tools.ToolSpec."""

    name: str
    description: str
    parameters: dict[str, Any]


_rob_box_harness_tools.ToolSpec = _StubToolSpec  # type: ignore[attr-defined]
_rob_box_harness_tools.ToolHandler = Callable[[Any], Any]  # type: ignore[attr-defined]
_rob_box_harness.tools = _rob_box_harness_tools  # type: ignore[attr-defined]
sys.modules.setdefault("rob_box_harness", _rob_box_harness)
sys.modules.setdefault("rob_box_harness.tools", _rob_box_harness_tools)

# Импорт dispatcher'а — после подмены.
from rob_box_supervisor.tars_panel import TarsPanelDispatcher  # noqa: E402


# ── FakeNode, как в conftest.py rob_box_supervisor ─────────────────


class _FakePublisher:
    def __init__(self, topic: str, msg_type: Any) -> None:
        self.topic = topic
        self.msg_type = msg_type
        self.published: list[Any] = []

    def publish(self, msg: Any) -> None:
        self.published.append(msg)


class _FakeSubscription:
    def __init__(self, topic: str, callback: Any) -> None:
        self.topic = topic
        self.callback = callback


class _FakeNode:
    """Минимум интерфейса, нужного TarsPanelDispatcher (create_*)."""

    def __init__(self) -> None:
        self._logger = MagicMock()
        self._publishers: dict[str, _FakePublisher] = {}
        self._subscriptions: list[_FakeSubscription] = []

    def get_logger(self) -> MagicMock:
        return self._logger

    def create_publisher(self, msg_type: Any, topic: str, qos: int = 10) -> _FakePublisher:
        pub = _FakePublisher(topic, msg_type)
        self._publishers[topic] = pub
        return pub

    def create_subscription(
        self, msg_type: Any, topic: str, callback: Any, qos: int = 10
    ) -> _FakeSubscription:
        sub = _FakeSubscription(topic, callback)
        self._subscriptions.append(sub)
        return sub


# ── helpers ────────────────────────────────────────────────────────


def _last_published(node: _FakeNode, topic: str) -> dict[str, Any]:
    pub = node._publishers.get(topic)
    assert pub is not None, f"publisher for {topic!r} not created"
    assert pub.published, f"no messages published on {topic!r}"
    msg = pub.published[-1]
    return json.loads(msg.data)


def _subscription(node: _FakeNode, topic: str) -> Any:
    subs = [s for s in node._subscriptions if s.topic == topic]
    assert subs, f"no subscription for {topic!r}"
    return subs[0]


def _make_msg(payload: Any) -> Any:
    """Создаёт фейковый std_msgs.msg.String с .data = json."""
    m = MagicMock()
    if isinstance(payload, str):
        m.data = payload
    else:
        m.data = json.dumps(payload)
    return m


# ── тесты ──────────────────────────────────────────────────────────


def test_build_panel_url_prometheus_default_base() -> None:
    """build_panel_url собирает корректный URL для Prometheus."""
    node = _FakeNode()
    d = TarsPanelDispatcher(node)
    url = d.build_panel_url("prometheus", "rate(cpu_usage[5m])")
    assert url.startswith("http://prometheus.lan/grafana/")
    assert "prometheus-overview" in url
    assert "query=rate%28cpu_usage%5B5m%5D%29" in url


def test_build_panel_url_loki_path() -> None:
    """build_panel_url собирает корректный URL для Loki."""
    node = _FakeNode()
    d = TarsPanelDispatcher(node)
    url = d.build_panel_url("loki", '{job="voice"}')
    assert "loki-logs" in url
    assert "query=" in url


def test_build_panel_url_custom_base() -> None:
    """base_url с трейлинг-слэшем нормализуется (без дублирования)."""
    node = _FakeNode()
    d = TarsPanelDispatcher(node, base_url="http://example.com/grafana/")
    url = d.build_panel_url("prometheus", "up")
    # Один слэш между base и path — никаких двойных.
    assert url == "http://example.com/grafana/d/prometheus-overview?query=up"


def test_on_panel_request_publishes_ok_url() -> None:
    """Валидный запрос → публикация ``status=ok`` с URL."""
    node = _FakeNode()
    d = TarsPanelDispatcher(node)
    sub = _subscription(node, "/avatar/tars/panel_request")
    sub.callback(
        _make_msg(
            {
                "request_id": "req-1",
                "query": "rate(cpu[5m])",
                "datasource": "prometheus",
            }
        )
    )
    out = _last_published(node, "/avatar/tars/panel_url")
    assert out["request_id"] == "req-1"
    assert out["status"] == "ok"
    assert "prometheus-overview" in out["url"]
    assert out["error"] == ""


def test_on_panel_request_empty_query_publishes_error() -> None:
    """Пустой query → status=error без URL."""
    node = _FakeNode()
    d = TarsPanelDispatcher(node)
    sub = _subscription(node, "/avatar/tars/panel_request")
    sub.callback(_make_msg({"request_id": "req-2", "query": "", "datasource": "prometheus"}))
    out = _last_published(node, "/avatar/tars/panel_url")
    assert out["status"] == "error"
    assert out["url"] == ""
    assert "empty" in out["error"]


def test_on_panel_request_unknown_datasource_publishes_error() -> None:
    """Неизвестный datasource → status=error без URL."""
    node = _FakeNode()
    d = TarsPanelDispatcher(node)
    sub = _subscription(node, "/avatar/tars/panel_request")
    sub.callback(
        _make_msg(
            {
                "request_id": "req-3",
                "query": "up",
                "datasource": "victoria",  # не в _DATASOURCE_PATH
            }
        )
    )
    out = _last_published(node, "/avatar/tars/panel_url")
    assert out["status"] == "error"
    assert "victoria" in out["error"]


def test_on_panel_request_bad_json_silently_drops() -> None:
    """Битый JSON — WARN, ничего не публикуем (graceful no-op)."""
    node = _FakeNode()
    d = TarsPanelDispatcher(node)
    sub = _subscription(node, "/avatar/tars/panel_request")
    sub.callback(_make_msg("{not valid json"))
    assert "/avatar/tars/panel_url" not in node._publishers or not node._publishers[
        "/avatar/tars/panel_url"
    ].published


def test_on_panel_request_non_object_silently_drops() -> None:
    """Не-dict payload — WARN, ничего не публикуем."""
    node = _FakeNode()
    d = TarsPanelDispatcher(node)
    sub = _subscription(node, "/avatar/tars/panel_request")
    sub.callback(_make_msg([1, 2, 3]))
    pub = node._publishers.get("/avatar/tars/panel_url")
    assert pub is None or not pub.published


def test_register_tool_adds_show_metrics_to_registry() -> None:
    """register_tool добавляет спецификацию ``show_metrics`` в registry."""
    node = _FakeNode()
    d = TarsPanelDispatcher(node)
    registry = MagicMock()
    registry.register = MagicMock()
    d.register_tool(registry)
    assert registry.register.call_count == 1
    spec, handler = registry.register.call_args.args
    assert spec.name == "show_metrics"
    assert "query" in spec.parameters.get("properties", {})


@pytest.mark.asyncio
async def test_show_metrics_handler_publishes_request_and_returns_status() -> None:
    """handler ``show_metrics`` публикует panel_request; subscription
    вызывает ``_on_panel_request`` → panel_url заполняется URL'ом.
    """
    import asyncio

    node = _FakeNode()
    d = TarsPanelDispatcher(node)
    registry = MagicMock()

    # Ловим реальную register'нутую пару (spec, handler).
    captured: dict[str, Any] = {}

    def _capture(spec: Any, handler: Any, **kw: Any) -> None:
        captured["spec"] = spec
        captured["handler"] = handler

    registry.register.side_effect = _capture
    d.register_tool(registry)

    handler = captured["handler"]
    result = await handler({"query": "rate(cpu[5m])", "datasource": "prometheus"})
    assert result["status"] == "published"
    assert "request_id" in result
    assert result["datasource"] == "prometheus"
    assert result["query"] == "rate(cpu[5m])"

    # Handler опубликовал в /avatar/tars/panel_request — это эмулирует
    # цикл: подписка на тот же топик вызывает _on_panel_request, который
    # парсит и публикует ответ в /avatar/tars/panel_url.
    req_pub = node._publishers.get("/avatar/tars/panel_request")
    assert req_pub is not None
    assert req_pub.published
    sub = _subscription(node, "/avatar/tars/panel_request")
    sub.callback(req_pub.published[-1])
    # await нужен только для handler'a (async); здесь синхронно.
    await asyncio.sleep(0)

    out = _last_published(node, "/avatar/tars/panel_url")
    assert out["request_id"] == result["request_id"]
    assert out["status"] == "ok"
    assert "prometheus-overview" in out["url"]


@pytest.mark.asyncio
async def test_show_metrics_handler_empty_query_returns_error() -> None:
    """handler ``show_metrics`` без query → status=error без публикации."""
    node = _FakeNode()
    d = TarsPanelDispatcher(node)
    registry = MagicMock()
    captured: dict[str, Any] = {}

    def _capture(spec: Any, handler: Any, **kw: Any) -> None:
        captured["spec"] = spec
        captured["handler"] = handler

    registry.register.side_effect = _capture
    d.register_tool(registry)

    result = await captured["handler"]({"query": ""})
    assert result["status"] == "error"
    assert "required" in result["error"]


@pytest.mark.asyncio
async def test_show_metrics_handler_unknown_datasource_returns_error() -> None:
    """handler ``show_metrics`` с неизвестным datasource → status=error."""
    node = _FakeNode()
    d = TarsPanelDispatcher(node)
    registry = MagicMock()
    captured: dict[str, Any] = {}

    def _capture(spec: Any, handler: Any, **kw: Any) -> None:
        captured["spec"] = spec
        captured["handler"] = handler

    registry.register.side_effect = _capture
    d.register_tool(registry)

    result = await captured["handler"]({"query": "up", "datasource": "elastic"})
    assert result["status"] == "error"
    assert "elastic" in result["error"]


def test_panel_url_topic_default() -> None:
    """topic'и создаются с дефолтами из сигнатуры."""
    node = _FakeNode()
    TarsPanelDispatcher(node)
    assert "/avatar/tars/panel_request" in [s.topic for s in node._subscriptions]
    assert "/avatar/tars/panel_url" in node._publishers


def test_panel_topics_override() -> None:
    """Топики можно переопределить (например, для тестов с namespace)."""
    node = _FakeNode()
    TarsPanelDispatcher(
        node,
        panel_request_topic="/test/req",
        panel_url_topic="/test/url",
    )
    assert "/test/req" in [s.topic for s in node._subscriptions]
    assert "/test/url" in node._publishers