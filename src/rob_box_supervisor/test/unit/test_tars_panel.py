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
import urllib.parse
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


# ── источник данных: фейк вместо HTTP ──────────────────────────────
#
# issue #2184: dispatcher больше не собирает URL «на бумаге», а реально ходит
# в Prometheus/Loki. В юнит-тестах сеть недопустима — подменяем MetricsSource
# целиком, а сборку URL берём настоящую (explore_url — чистая функция).

from rob_box_supervisor.metrics_source import MetricsSource  # noqa: E402


def _stub_source(result: dict[str, Any] | None = None) -> MetricsSource:
    """MetricsSource с замоканным HTTP: каталог пуст, ответ задаётся тестом."""
    src = MetricsSource(http_get=lambda url: b'{"status":"success","data":[]}')
    payload = result if result is not None else {"status": "ok", "query": "up", "series": []}
    src.query_range = lambda query, **kw: dict(payload)  # type: ignore[assignment]
    src.query_logs = lambda query, **kw: dict(payload)  # type: ignore[assignment]
    return src


def _dispatcher(
    node: _FakeNode,
    *,
    result: dict[str, Any] | None = None,
    **kwargs: Any,
) -> TarsPanelDispatcher:
    """Dispatcher без сети и без потоков (spawn выполняется синхронно)."""
    kwargs.setdefault("metrics", _stub_source(result))
    kwargs.setdefault("spawn", lambda fn: fn())
    return TarsPanelDispatcher(node, **kwargs)


_SERIES_OK = {
    "status": "ok",
    "query": "rate(process_cpu_seconds_total[5m])",
    "note": "",
    "series": [
        {
            "name": "voice-assistant",
            "labels": {"instance": "10.1.1.11:9100"},
            "points": [[1788893900.0, 0.03], [1788893960.0, 0.04]],
        }
    ],
    "range_minutes": 15,
}


# ── тесты ──────────────────────────────────────────────────────────


def test_build_panel_url_points_at_explore_not_a_dashboard() -> None:
    """URL ведёт в Grafana Explore — единственную страницу с произвольным query.

    Регрессия issue #2184: раньше собирался ``/d/prometheus-overview?query=…``
    — дашборда с таким UID в Grafana нет, а ``?query=`` дашборд игнорирует.
    """
    node = _FakeNode()
    d = _dispatcher(node)
    url = d.build_panel_url("prometheus", "rate(process_cpu_seconds_total[5m])")
    assert "/explore?" in url
    assert "prometheus-overview" not in url
    # Запрос уезжает внутри JSON-параметра left, а не отдельным ?query=.
    assert "left=" in url
    assert "process_cpu_seconds_total" in urllib.parse.unquote(url)


def test_build_panel_url_loki_uses_loki_datasource() -> None:
    """Для loki в Explore подставляется datasource Loki, а не Prometheus."""
    node = _FakeNode()
    d = _dispatcher(node)
    url = urllib.parse.unquote(d.build_panel_url("loki", '{job="voice"}'))
    assert '"datasource":"Loki"' in url
    # LogQL едет внутри JSON — кавычки в нём экранированы.
    assert r'{job=\"voice\"}' in url


def test_build_panel_url_custom_base() -> None:
    """base_url с трейлинг-слэшем нормализуется (без дублирования слэшей)."""
    node = _FakeNode()
    d = TarsPanelDispatcher(
        node, base_url="http://example.com:3000/", spawn=lambda fn: fn()
    )
    url = d.build_panel_url("prometheus", "up")
    assert url.startswith("http://example.com:3000/explore?")


def test_on_panel_request_publishes_series_to_panel_data() -> None:
    """Валидный запрос → в ``/avatar/tars/panel_data`` уезжают точки.

    Ядро issue #2184: раньше публиковался только URL, и оператор не видел
    ни одной цифры. Теперь на панель едут ряды из Prometheus.
    """
    node = _FakeNode()
    _dispatcher(node, result=_SERIES_OK)
    sub = _subscription(node, "/avatar/tars/panel_request")
    sub.callback(
        _make_msg(
            {
                "request_id": "req-1",
                "query": "cpu",
                "datasource": "prometheus",
            }
        )
    )
    out = _last_published(node, "/avatar/tars/panel_data")
    assert out["request_id"] == "req-1"
    assert out["status"] == "ok"
    assert out["series"][0]["points"] == [[1788893900.0, 0.03], [1788893960.0, 0.04]]
    # summary — то, что ТАРС произносит; в нём должно быть значение, а не
    # «панель открыта».
    assert "voice-assistant" in out["summary"]
    assert "0.04" in out["summary"]


def test_on_panel_request_publishes_url_before_data() -> None:
    """Legacy panel_url публикуется ПЕРВЫМ, иначе он затирает график.

    Клиент обрабатывает события в порядке прихода, а ``setPanelUrl``
    сбрасывает нарисованные данные (ссылка их не несёт).
    """
    node = _FakeNode()
    order: list[str] = []
    _dispatcher(node, result=_SERIES_OK)
    for topic in ("/avatar/tars/panel_url", "/avatar/tars/panel_data"):
        pub = node._publishers[topic]
        original = pub.publish

        def _track(msg: Any, _t: str = topic, _o: Any = original) -> None:
            order.append(_t)
            _o(msg)

        pub.publish = _track  # type: ignore[method-assign]
    sub = _subscription(node, "/avatar/tars/panel_request")
    sub.callback(_make_msg({"request_id": "r", "query": "up"}))
    assert order == ["/avatar/tars/panel_url", "/avatar/tars/panel_data"]


def test_on_panel_request_empty_result_is_not_an_error() -> None:
    """Пустой результат → ``status=empty`` + список доступных метрик.

    Оператор должен понять, что запрос выполнился, а данных нет — иначе он
    решит, что сломался тракт (ADR-0018).
    """
    node = _FakeNode()
    _dispatcher(
        node,
        result={
            "status": "empty",
            "query": "rate(network_latency_ms[5m])",
            "series": [],
            "available": ["up", "voice_llm_request_total"],
        },
    )
    sub = _subscription(node, "/avatar/tars/panel_request")
    sub.callback(_make_msg({"request_id": "r2", "query": "network_latency_ms"}))
    out = _last_published(node, "/avatar/tars/panel_data")
    assert out["status"] == "empty"
    assert out["available"] == ["up", "voice_llm_request_total"]
    assert "данных нет" in out["summary"].lower()


def test_on_panel_request_prometheus_down_reports_honestly() -> None:
    """MetricsUnavailable → status=error с причиной, без падения потока."""
    from rob_box_supervisor.metrics_source import MetricsUnavailable

    node = _FakeNode()
    src = _stub_source()

    def _boom(query: str, **kw: Any) -> dict[str, Any]:
        raise MetricsUnavailable("http://10.1.1.249:9090/api/v1/query_range: timed out")

    src.query_range = _boom  # type: ignore[assignment]
    TarsPanelDispatcher(node, metrics=src, spawn=lambda fn: fn())
    sub = _subscription(node, "/avatar/tars/panel_request")
    sub.callback(_make_msg({"request_id": "r3", "query": "up"}))
    out = _last_published(node, "/avatar/tars/panel_data")
    assert out["status"] == "error"
    assert "timed out" in out["error"]
    url_out = _last_published(node, "/avatar/tars/panel_url")
    assert url_out["url"] == ""  # ссылки, за которой ничего нет, не даём


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
async def test_show_metrics_handler_returns_values_not_a_promise() -> None:
    """handler ``show_metrics`` отдаёт LLM реальные значения, а не обещание.

    Регрессия issue #2184: раньше handler возвращал «Опубликовал запрос…
    Quest откроет панель», и ТАРС уверенно рапортовал об открытии панели,
    которая оставалась пустой.
    """
    node = _FakeNode()
    d = _dispatcher(node, result=_SERIES_OK)
    registry = MagicMock()

    # Ловим реальную register'нутую пару (spec, handler).
    captured: dict[str, Any] = {}

    def _capture(spec: Any, handler: Any, **kw: Any) -> None:
        captured["spec"] = spec
        captured["handler"] = handler

    registry.register.side_effect = _capture
    d.register_tool(registry)

    handler = captured["handler"]
    result = await handler({"query": "cpu", "datasource": "prometheus"})
    assert result["status"] == "ok"
    assert result["series_count"] == 1
    assert "0.04" in result["message"]
    assert result["query"] == "rate(process_cpu_seconds_total[5m])"

    # Handler сам публикует результат на панель — Quest получает данные без
    # второго round-trip'а через /avatar/tars/panel_request.
    out = _last_published(node, "/avatar/tars/panel_data")
    assert out["request_id"] == result["request_id"]
    assert out["status"] == "ok"
    assert out["series"][0]["name"] == "voice-assistant"


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
    _dispatcher(node)
    assert "/avatar/tars/panel_request" in [s.topic for s in node._subscriptions]
    assert "/avatar/tars/panel_url" in node._publishers
    assert "/avatar/tars/panel_data" in node._publishers


def test_panel_topics_override() -> None:
    """Топики можно переопределить (например, для тестов с namespace)."""
    node = _FakeNode()
    _dispatcher(
        node,
        panel_request_topic="/test/req",
        panel_url_topic="/test/url",
        panel_data_topic="/test/data",
    )
    assert "/test/req" in [s.topic for s in node._subscriptions]
    assert "/test/url" in node._publishers
    assert "/test/data" in node._publishers


def test_default_grafana_base_url_is_reachable_host() -> None:
    """Дефолт больше не указывает на несуществующий ``prometheus.lan``.

    Регрессия issue #2184: этот хост не резолвился ни с робота, ни с katana,
    поэтому ссылка на панель была заведомо мёртвой.
    """
    from rob_box_supervisor.tars_panel import DEFAULT_GRAFANA_BASE_URL

    assert "prometheus.lan" not in DEFAULT_GRAFANA_BASE_URL
    assert DEFAULT_GRAFANA_BASE_URL.startswith("http")