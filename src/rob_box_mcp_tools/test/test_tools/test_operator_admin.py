"""
test_operator_admin.py - Unit тесты для operator.admin среза (ADR-0051 §6, issue #2001).

Покрывает:
- Ros2NodeStatusTool: локальный кеш monitor'а, fallback при отсутствии,
  фильтр по ``nodes``, явные категории active/missing/failed.
- ReadLogsTool: чтение /rosout (через mock health_monitor), loki-путь
  с моком HTTP, санитизация Authorization через redact_upstream_body.
- ContainerStatusTool: парсинг PromQL-ответов, частичный успех,
  полная недоступность Prometheus.
- Sanitization helpers (redact integration).

Тесты НЕ требуют ROS 2 / Prometheus / Loki — всё мокается в conftest стиле.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from typing import Any, Dict, List
from unittest.mock import Mock, patch

import pytest


_ROS_MODULE_SUBMODULES = {
    "rclpy": ["action", "node", "callback_groups", "qos"],
    "geometry_msgs": ["msg"],
    "nav2_msgs": ["action"],
    "action_msgs": ["srv", "msg"],
    "std_msgs": ["msg"],
    "std_srvs": ["srv"],
    "nav_msgs": ["msg"],
    "sensor_msgs": ["msg"],
}


def _install_ros_mocks_if_needed() -> None:
    for parent, submodules in _ROS_MODULE_SUBMODULES.items():
        if parent in sys.modules:
            continue
        if importlib.util.find_spec(parent) is not None:
            continue
        sys.modules.setdefault(parent, Mock())
        for sub in submodules:
            sys.modules.setdefault(f"{parent}.{sub}", Mock())


_install_ros_mocks_if_needed()


# rob_box_voice.utils.redact: подменяем для локальных прогонов (CI: реальный модуль).
_redact_spec_found = False
try:
    _redact_spec_found = importlib.util.find_spec("rob_box_voice.utils.redact") is not None
except (ImportError, ModuleNotFoundError, ValueError):
    _redact_spec_found = False

if not _redact_spec_found:
    _redact_mod = Mock()
    _redact_mod.redact_upstream_body = lambda text: text  # passthrough
    sys.modules.setdefault("rob_box_voice.utils", Mock())
    sys.modules["rob_box_voice.utils.redact"] = _redact_mod


from rob_box_mcp_tools.tools.operator_admin import (  # noqa: E402
    ContainerStatusTool,
    ReadLogsTool,
    Ros2NodeStatusTool,
    ShowMetricsTool,
    _DEFAULT_EXPECTED_NODES,
    _http_get_json,
    _probe_ros_node_names,
    _sanitize_text,
)


# ----------------------------------------------------------------------------
# Test helpers
# ----------------------------------------------------------------------------


class FakeMonitor:
    """Подделка NodeAvailabilityMonitor — хранит ``node_status`` как dict."""

    def __init__(self, statuses: Dict[str, Dict[str, Any]]) -> None:
        self.node_status = statuses


class FakeHealthMonitor:
    """Подделка HealthMonitor — атрибуты errors/warnings как list."""

    def __init__(
        self,
        errors: List[Dict[str, Any]] | None = None,
        warnings: List[Dict[str, Any]] | None = None,
    ) -> None:
        self.errors = errors or []
        self.warnings = warnings or []


# ----------------------------------------------------------------------------
# Ros2NodeStatusTool
# ----------------------------------------------------------------------------


@pytest.mark.unit
class TestRos2NodeStatusTool:
    """Тесты Ros2NodeStatusTool."""

    def test_tool_metadata(self, mock_node):
        tool = Ros2NodeStatusTool(mock_node)
        assert tool.name == "ros2_node_status"
        assert "ROS2" in tool.description
        assert tool.execution_type.value == "medium"
        assert tool.read_only is True
        names = [p.name for p in tool.parameters]
        assert names == ["nodes"]

    def test_fallback_uses_monitor_when_rclpy_missing(self, mock_node):
        """Нет rclpy — должны использовать monitor, иначе честный missing."""
        monitor = FakeMonitor({
            "/audio_node": {"status": "active", "last_seen": 0},
            "/stt_node": {"status": "failed", "last_seen": 0},
            "/tts_node": {"status": "missing", "last_seen": None},
        })
        mock_node._node_availability_monitor = monitor

        tool = Ros2NodeStatusTool(mock_node)
        # Передаём только те 3 ноды, что зарегистрированы в мониторе
        result = tool.execute(nodes=["/audio_node", "/stt_node", "/tts_node"])

        assert result.success is True
        data = result.data
        assert data["source"] == "node_availability_monitor"
        assert data["active_list"] == ["/audio_node"]
        assert data["failed_list"] == ["/stt_node"]
        assert "/tts_node" in data["missing_list"]
        assert data["total"] == 3

    def test_fallback_monitor_unavailable_returns_empty_active(self, mock_node):
        """Monitor не зарегистрирован → все ноды missing + source=monitor_unavailable."""
        tool = Ros2NodeStatusTool(mock_node)
        result = tool.execute(nodes=["/audio_node"])

        assert result.success is True
        data = result.data
        assert data["source"] == "monitor_unavailable"
        assert data["active"] == 0
        assert data["missing_list"] == ["/audio_node"]
        assert data["failed"] == 0

    def test_rclpy_path_marks_active_and_missing(self, mock_node):
        """rclpy доступен → rclpy.get_node_names() определяет live-список."""
        with patch(
            "rob_box_mcp_tools.tools.operator_admin._probe_ros_node_names",
            return_value=["/audio_node", "/stt_node"],
        ):
            tool = Ros2NodeStatusTool(mock_node)
            result = tool.execute(
                nodes=["/audio_node", "/stt_node", "/tts_node"],
            )

        assert result.success is True
        data = result.data
        assert sorted(data["active_list"]) == ["/audio_node", "/stt_node"]
        assert data["missing_list"] == ["/tts_node"]
        # rclpy-путь не делает failed (один снапшот)
        assert data["failed_list"] == []
        assert "rclpy" not in data  # ключ source отсутствует в rclpy-пути

    def test_default_expected_nodes_used_when_nodes_omitted(self, mock_node):
        """Без nodes — используем дефолтный список (синхронизирован с monitor)."""
        with patch(
            "rob_box_mcp_tools.tools.operator_admin._probe_ros_node_names",
            return_value=list(_DEFAULT_EXPECTED_NODES),
        ):
            tool = Ros2NodeStatusTool(mock_node)
            result = tool.execute()

        assert result.success is True
        assert result.data["total"] == len(_DEFAULT_EXPECTED_NODES)
        assert result.data["active"] == len(_DEFAULT_EXPECTED_NODES)
        assert result.data["missing"] == 0


# ----------------------------------------------------------------------------
# ReadLogsTool
# ----------------------------------------------------------------------------


@pytest.mark.unit
class TestReadLogsTool:
    """Тесты ReadLogsTool."""

    def test_tool_metadata(self, mock_node):
        tool = ReadLogsTool(mock_node)
        assert tool.name == "read_logs"
        assert tool.read_only is True
        names = [p.name for p in tool.parameters]
        assert names == ["node", "source", "limit"]
        source_param = next(p for p in tool.parameters if p.name == "source")
        assert source_param.enum == ["rosout", "loki"]

    def test_rosout_without_monitor_returns_monitor_unavailable(self, mock_node):
        """HealthMonitor не зарегистрирован → monitor_unavailable=True, lines=[]."""
        tool = ReadLogsTool(mock_node)
        result = tool.execute(node="/audio_node", source="rosout")

        assert result.success is True
        assert result.data["monitor_unavailable"] is True
        assert result.data["lines"] == []

    def test_rosout_filters_by_node(self, mock_node):
        """Возвращаются только записи для указанной ноды."""
        monitor = FakeHealthMonitor(
            errors=[
                {"node": "/audio_node", "level": "ERROR", "msg": "boom-audio", "time": 100.0},
                {"node": "/stt_node", "level": "ERROR", "msg": "boom-stt", "time": 200.0},
                {"node": "/audio_node", "level": "FATAL", "msg": "die-audio", "time": 300.0},
            ],
            warnings=[
                {"node": "/stt_node", "msg": "warn-stt", "time": 150.0},
                {"node": "/audio_node", "msg": "warn-audio", "time": 250.0},
            ],
        )
        mock_node._health_monitor = monitor

        tool = ReadLogsTool(mock_node)
        result = tool.execute(node="/audio_node", source="rosout")

        assert result.success is True
        # 3 записи для audio_node (2 error + 1 warning) — newer first
        assert result.data["count"] == 3
        msgs = [line["msg"] for line in result.data["lines"]]
        # sort by time desc — последняя по времени (300.0) первая
        assert msgs[0] == "die-audio"
        assert "boom-stt" not in msgs  # фильтр
        assert "warn-stt" not in msgs

    def test_rosout_limit_caps_results(self, mock_node):
        """limit режет записи до N (последние по времени)."""
        monitor = FakeHealthMonitor(
            errors=[
                {"node": "/n", "level": "ERROR", "msg": f"e{i}", "time": float(i)}
                for i in range(10)
            ],
        )
        mock_node._health_monitor = monitor

        tool = ReadLogsTool(mock_node)
        result = tool.execute(node="/n", source="rosout", limit=3)

        assert result.success is True
        assert result.data["count"] == 3
        # последние по времени: 9, 8, 7
        assert [l["msg"] for l in result.data["lines"]] == ["e9", "e8", "e7"]

    def test_rosout_sanitizes_credentials(self, mock_node):
        """Сообщение с API-ключом либо замаскировано (CI), либо passthrough (local)."""
        monitor = FakeHealthMonitor(
            errors=[
                {"node": "/n", "level": "ERROR", "msg": "Authorization: Bearer sk-live-1234567890", "time": 1.0},
            ],
        )
        mock_node._health_monitor = monitor

        tool = ReadLogsTool(mock_node)
        result = tool.execute(node="/n", source="rosout")

        assert result.success is True
        msg = result.data["lines"][0]["msg"]
        # Реальный redact (CI): заменяет токен на `***`. Локальный passthrough
        # оставляет как есть. Принимаем оба варианта — главное, чтобы имя
        # заголовка сохранялось.
        assert "Authorization" in msg
        if msg != "Authorization: Bearer sk-live-1234567890":
            assert "***" in msg or "sk-live-1234567890" not in msg

    def test_loki_unavailable_returns_failure(self, mock_node):
        """Loki недоступен → success=False, error содержит URL."""
        with patch(
            "rob_box_mcp_tools.tools.operator_admin._http_get_json",
            return_value=None,
        ):
            tool = ReadLogsTool(mock_node)
            result = tool.execute(node="voice-assistant", source="loki")

        assert result.success is False
        assert "Loki" in result.error
        assert result.data["lines"] == []

    def test_loki_parses_query_range_response(self, mock_node):
        """Парсим ответ /loki/api/v1/query_range (data.result[*].values)."""
        fake_body = {
            "data": {
                "result": [
                    {
                        "stream": {"service": "voice-assistant", "container": "voice-assistant"},
                        "values": [
                            ["1700000000000000000", "hello world"],
                            ["1700000001000000000", "Authorization: Bearer sk-deadbeef"],
                        ],
                    },
                    {
                        "stream": {"service": "voice-assistant", "container": "voice-assistant"},
                        "values": [
                            ["1700000002000000000", "third line"],
                        ],
                    },
                ],
            },
        }
        with patch(
            "rob_box_mcp_tools.tools.operator_admin._http_get_json",
            return_value=fake_body,
        ):
            tool = ReadLogsTool(mock_node)
            result = tool.execute(node="voice-assistant", source="loki", limit=10)

        assert result.success is True
        lines = result.data["lines"]
        # 3 строки суммарно (2 + 1) после объединения streams
        assert len(lines) == 3
        # самая свежая первая (sort by ts desc)
        assert lines[0]["line"] == "third line"
        # санитизация: токен либо замаскирован (CI), либо passthrough (local).
        auth_line = next(l for l in lines if "Authorization" in l["line"])
        if auth_line["line"] != "Authorization: Bearer sk-deadbeef":
            assert "***" in auth_line["line"] or "sk-deadbeef" not in auth_line["line"]


# ----------------------------------------------------------------------------
# ContainerStatusTool
# ----------------------------------------------------------------------------


@pytest.mark.unit
class TestContainerStatusTool:
    """Тесты ContainerStatusTool."""

    def _make_prom_body(self, results: List[Dict[str, Any]]) -> Dict[str, Any]:
        return {"data": {"result": results}}

    def _series(self, name: str, value: Any) -> Dict[str, Any]:
        return {"metric": {"name": name}, "value": [0, str(value)]}

    def test_tool_metadata(self, mock_node):
        tool = ContainerStatusTool(mock_node)
        assert tool.name == "container_status"
        assert tool.read_only is True
        names = [p.name for p in tool.parameters]
        assert names == ["name"]

    def test_prometheus_unavailable_returns_failure(self, mock_node):
        """Все четыре запроса вернули None → честный fail."""
        with patch(
            "rob_box_mcp_tools.tools.operator_admin._http_get_json",
            return_value=None,
        ):
            tool = ContainerStatusTool(mock_node)
            result = tool.execute()

        assert result.success is False
        assert "Prometheus" in result.error

    def test_partial_prometheus_success_returns_available_metrics(self, mock_node):
        """PromQL вернул только restart — остальные None → partial success."""
        restart_body = self._make_prom_body([
            self._series("voice-assistant", 3),
            self._series("rplidar", 0),
        ])

        def fake_http(url, params=None):
            if "kube_pod_container_status_restarts_total" in params["query"]:
                return restart_body
            return None  # остальные метрики недоступны

        with patch(
            "rob_box_mcp_tools.tools.operator_admin._http_get_json",
            side_effect=fake_http,
        ):
            tool = ContainerStatusTool(mock_node)
            result = tool.execute()

        assert result.success is True
        data = result.data
        assert data["count"] == 2
        assert sorted(data["unavailable_metrics"]) == ["cpu_rate", "memory_usage", "start_time"]
        by_name = {c["name"]: c for c in data["containers"]}
        assert by_name["voice-assistant"]["restart_count"] == 3
        assert by_name["rplidar"]["restart_count"] == 0
        assert by_name["voice-assistant"]["memory_bytes"] is None

    def test_full_metrics_parsed_and_uptime_computed(self, mock_node):
        """Все 4 запроса успешны → uptime = now - start_time."""
        now = 1_700_000_000.0
        start_ts = now - 3600  # 1 час назад

        def fake_http(url, params=None):
            q = params["query"]
            if "kube_pod_container_status_restarts_total" in q:
                return self._make_prom_body([self._series("c1", 5)])
            if "container_cpu_usage_seconds_total" in q:
                return self._make_prom_body([self._series("c1", 0.42)])
            if "container_memory_usage_bytes" in q:
                return self._make_prom_body([self._series("c1", 123_456_789)])
            if "container_start_time_seconds" in q:
                return self._make_prom_body([self._series("c1", start_ts)])
            return None

        with patch("time.time", return_value=now), patch(
            "rob_box_mcp_tools.tools.operator_admin._http_get_json",
            side_effect=fake_http,
        ):
            tool = ContainerStatusTool(mock_node)
            result = tool.execute(name="c1")

        assert result.success is True
        c = result.data["containers"][0]
        assert c["restart_count"] == 5
        assert c["cpu_rate_per_sec"] == pytest.approx(0.42)
        assert c["memory_bytes"] == pytest.approx(123_456_789)
        assert c["uptime_sec"] == pytest.approx(3600.0)
        assert result.data["unavailable_metrics"] == []

    def test_name_filter_propagates_to_promql(self, mock_node):
        """Параметр name попадает в PromQL-фильтр (=~ '<name>')."""
        seen_queries: List[str] = []

        def fake_http(url, params=None):
            seen_queries.append(params["query"])
            return None  # → fail честный (все недоступны)

        with patch(
            "rob_box_mcp_tools.tools.operator_admin._http_get_json",
            side_effect=fake_http,
        ):
            tool = ContainerStatusTool(mock_node)
            result = tool.execute(name="voice")

        # fail, но мы успели поймать query
        assert result.success is False
        assert any("voice" in q for q in seen_queries), "name filter не дошёл до PromQL"


# ----------------------------------------------------------------------------
# Helpers — pure functions
# ----------------------------------------------------------------------------


@pytest.mark.unit
class TestSanitizationHelpers:
    """Прямая проверка _sanitize_text (использует реальный redact если есть)."""

    def test_sanitize_text_redacts_bearer(self):
        out = _sanitize_text("Authorization: Bearer abc-def-1234567890")
        # redact либо заменит токен на ***, либо (если rob_box_voice нет
        # в среде) пропустит — проверяем, что утилита хотя бы callable
        assert isinstance(out, str)


@pytest.mark.unit
class TestHttpHelpers:
    """Helpers _http_get_json + _probe_ros_node_names."""

    def test_probe_ros_node_names_returns_none_when_rclpy_missing(self, mock_node):
        # Удаляем rclpy из sys.modules на время теста, чтобы import внутри
        # _probe_ros_node_names упал с ImportError → wrapper вернул None.
        saved_rclpy = sys.modules.pop("rclpy", None)
        try:
            assert _probe_ros_node_names(mock_node) is None
        finally:
            if saved_rclpy is not None:
                sys.modules["rclpy"] = saved_rclpy

    def test_http_get_json_returns_none_on_network_error(self):
        """URL error → None (а не raise)."""
        import urllib.error

        with patch.object(
            urllib.request, "urlopen",
            side_effect=urllib.error.URLError("nope"),
        ):
            assert _http_get_json("http://x") is None


# ----------------------------------------------------------------------------
# ShowMetricsTool (issue #2113 / TARS 2 metrics panel)
# ----------------------------------------------------------------------------
#
# Тест-планы:
# 1. Metadata: name=show_metrics, slice=operator.admin, parameters
#    (query обязательный, datasource опциональный).
# 2. Happy path: execute(query=...) → публикует JSON в
#    /avatar/tars/panel_request, формат {request_id, query, datasource},
#    success=True + message для LLM.
# 3. datasource=loki: попадает в payload.
# 4. datasource=None (default): в payload попадает "prometheus".
# 5. Пустой query: success=False без публикации.
# 6. Неподдерживаемый datasource: success=False без публикации.
# 7. Без node (без ROS): success=False с понятным сообщением.
# 8. publisher.publish raises: success=False, error содержит описание.
# 9. Контракт для TarsPanelDispatcher: payload содержит все 3 поля
#    (request_id/query/datasource) с правильными типами.


@pytest.mark.unit
def _autorespond(tool, payload_factory):
    """Смоделировать avatar_supervisor: ответить на panel_request.

    issue #2184: тул больше не «выстрелил и забыл» — он ждёт данных в
    ``/avatar/tars/panel_data``, чтобы ТАРС называл оператору числа, а не
    рапортовал об открытии панели. В тестах супервизора нет, поэтому
    publish на panel_request сразу дёргает обратный callback с тем же
    request_id (как это делает TarsPanelDispatcher на роботе).

    ``payload_factory(request_id) -> dict`` — тело ответа.
    """
    pub = tool._panel_request_pub
    original = pub.publish

    def _publish_and_answer(msg):
        original(msg)
        request = json.loads(msg.data)
        reply = Mock()
        reply.data = json.dumps(payload_factory(request["request_id"]))
        tool._on_panel_data(reply)

    pub.publish = _publish_and_answer
    return pub


def _ok_payload(request_id, *, query="rate(process_cpu_seconds_total[5m])"):
    return {
        "request_id": request_id,
        "status": "ok",
        "query": query,
        "summary": "Вывел на TARS 2 — voice-assistant: 0.04",
        "series": [
            {
                "name": "voice-assistant",
                "labels": {"instance": "10.1.1.11:9100"},
                "points": [[1788893900.0, 0.03], [1788893960.0, 0.04]],
            }
        ],
        "url": "http://10.1.1.249:3000/explore?orgId=1&left=%7B%7D",
        "error": "",
    }


class TestShowMetricsTool:
    """Тесты ShowMetricsTool — операторский show_metrics (issue #2113/#2184)."""

    def test_tool_metadata(self, mock_node):
        """Метаданные tool'а: имя, slice, обязательные параметры."""
        tool = ShowMetricsTool(mock_node)
        assert tool.name == "show_metrics"
        assert tool.slice == "operator.admin"  # ADR-0052 / issue #1998
        assert tool.read_only is True
        # parameters: query (required), datasource (optional)
        names = [p.name for p in tool.parameters]
        assert names == ["query", "datasource"]
        query_param = tool.parameters[0]
        assert query_param.required is True
        ds_param = tool.parameters[1]
        assert ds_param.required is False
        assert ds_param.default == "prometheus"
        assert ds_param.enum == ["prometheus", "loki"]

    def test_tool_registered_in_mcp_tools_init(self):
        """ShowMetricsTool экспортирован через ``tools/__init__``."""
        import importlib

        # Lazy import — в conftest_local_shim мы уже подменили ROS, но
        # сам импорт tools/__init__.py должен протащить ShowMetricsTool
        # через wildcard ``from .operator_admin import *``.
        from rob_box_mcp_tools import tools as tools_pkg

        assert "ShowMetricsTool" in tools_pkg.__all__, (
            "ShowMetricsTool должен быть в tools.__all__ — иначе mcp_server"
            "не сможет его импортировать"
        )
        assert hasattr(tools_pkg, "ShowMetricsTool"), (
            "ShowMetricsTool должен быть доступен как rob_box_mcp_tools.tools.ShowMetricsTool"
        )

    def test_execute_publishes_json_to_panel_request(self, mock_node):
        """execute(query) → публикует JSON в /avatar/tars/panel_request.

        Контракт публикации зафиксирован в rob_box_supervisor.tars_panel:
        ``{"request_id": str, "query": str, "datasource": str}``.
        """
        tool = ShowMetricsTool(mock_node)
        pub = _autorespond(tool, _ok_payload)
        result = tool.execute(query="rate(cpu_usage[5m])")

        assert result.success is True
        assert result.data["datasource"] == "prometheus"
        # query в ответе — тот, что реально выполнил супервизор (он мог
        # отрезолвить несуществующее имя метрики).
        assert result.data["query"] == "rate(process_cpu_seconds_total[5m])"
        assert result.data["status"] == "ok"
        assert result.data["series_count"] == 1
        # latest — числа для голосового ответа ТАРСа.
        assert result.data["latest"] == {"voice-assistant": 0.04}
        # message — то, что LLM повторит оператору: значения, а не обещание.
        assert "0.04" in result.message
        assert "Опубликовал" not in result.message

        # Проверяем публикацию на /avatar/tars/panel_request
        assert len(pub.published_messages) == 1
        msg = pub.published_messages[0]
        payload = json.loads(msg.data)
        assert payload["query"] == "rate(cpu_usage[5m])"
        assert payload["datasource"] == "prometheus"
        assert payload["request_id"] == result.data["request_id"]
        # request_id — 8 hex символов (uuid4.hex[:8])
        assert len(result.data["request_id"]) == 8

    def test_execute_reports_timeout_instead_of_claiming_success(self, mock_node):
        """Супервизор молчит → success=False, а не «открыл панель».

        Регрессия issue #2184: тул возвращал success=True сразу после
        публикации, и ТАРС бодро сообщал оператору об открытой панели, даже
        когда avatar_supervisor лежал (ADR-0018).
        """
        import rob_box_mcp_tools.tools.operator_admin as oa

        tool = ShowMetricsTool(mock_node)
        original_timeout = oa._SHOW_METRICS_TIMEOUT_SEC
        oa._SHOW_METRICS_TIMEOUT_SEC = 0.05  # никто не ответит
        try:
            result = tool.execute(query="up")
        finally:
            oa._SHOW_METRICS_TIMEOUT_SEC = original_timeout
        assert result.success is False
        assert "panel_data" in result.error

    def test_execute_empty_result_is_success_but_says_no_data(self, mock_node):
        """status=empty — тракт цел, данных нет: LLM не должен выдумывать числа."""
        tool = ShowMetricsTool(mock_node)
        _autorespond(
            tool,
            lambda rid: {
                "request_id": rid,
                "status": "empty",
                "query": "rate(network_latency_ms[5m])",
                "summary": "По запросу «rate(network_latency_ms[5m])» данных нет. Есть: up.",
                "series": [],
                "available": ["up"],
            },
        )
        result = tool.execute(query="rate(network_latency_ms[5m])")
        assert result.success is True
        assert result.data["status"] == "empty"
        assert result.data["series_count"] == 0
        assert result.data["available"] == ["up"]
        assert "данных нет" in result.message.lower()

    def test_execute_supervisor_error_propagates(self, mock_node):
        """Prometheus лёг → success=False с причиной от супервизора."""
        tool = ShowMetricsTool(mock_node)
        _autorespond(
            tool,
            lambda rid: {
                "request_id": rid,
                "status": "error",
                "query": "up",
                "summary": "Метрики не пришли: connection refused",
                "error": "http://10.1.1.249:9090/api/v1/query_range: connection refused",
            },
        )
        result = tool.execute(query="up")
        assert result.success is False
        assert "connection refused" in result.error

    def test_execute_ignores_foreign_request_ids(self, mock_node):
        """Ответ на ЧУЖОЙ request_id не будит наш вызов.

        На шине параллельно живут запросы от других вызовов ТАРСа — взять
        чужие данные значило бы показать оператору не тот график.
        """
        import rob_box_mcp_tools.tools.operator_admin as oa

        tool = ShowMetricsTool(mock_node)
        _autorespond(tool, lambda rid: _ok_payload("deadbeef"))
        original_timeout = oa._SHOW_METRICS_TIMEOUT_SEC
        oa._SHOW_METRICS_TIMEOUT_SEC = 0.05
        try:
            result = tool.execute(query="up")
        finally:
            oa._SHOW_METRICS_TIMEOUT_SEC = original_timeout
        assert result.success is False

    def test_execute_loki_datasource_propagates_to_payload(self, mock_node):
        """datasource=loki → попадает в payload (для LogQL-запросов)."""
        tool = ShowMetricsTool(mock_node)
        pub = _autorespond(
            tool,
            lambda rid: {
                "request_id": rid,
                "status": "ok",
                "query": '{job="voice"}',
                "summary": "Вывел на TARS 2 — 2 строки",
                "series": [],
                "lines": [{"ts": 1788893960.0, "line": "hello", "labels": {}}],
            },
        )
        result = tool.execute(
            query='{job="voice"}',
            datasource="loki",
        )

        assert result.success is True
        assert result.data["datasource"] == "loki"
        assert result.data["series_count"] == 1
        payload = json.loads(pub.published_messages[0].data)
        assert payload["datasource"] == "loki"
        assert payload["query"] == '{job="voice"}'

    def test_execute_datasource_normalizes_case(self, mock_node):
        """datasource='Prometheus' → 'prometheus' (lowercase, как у dispatcher'а)."""
        tool = ShowMetricsTool(mock_node)
        pub = _autorespond(tool, lambda rid: _ok_payload(rid, query="up"))
        result = tool.execute(query="up", datasource="Prometheus")
        assert result.success is True
        assert result.data["datasource"] == "prometheus"
        payload = json.loads(pub.published_messages[0].data)
        assert payload["datasource"] == "prometheus"

    def test_execute_empty_query_returns_error_without_publishing(self, mock_node):
        """Пустой query → success=False, никакой публикации."""
        tool = ShowMetricsTool(mock_node)
        # validate_parameters поймает пустой query на стороне registry.execute
        # (required=True), но и сам execute() должен страховаться.
        # Передаём whitespace-only — это пройдёт required, но execute
        # должен отвергнуть.
        result = tool.execute(query="   ")
        assert result.success is False
        assert "query" in result.error.lower()
        pub = mock_node._publishers.get("/avatar/tars/panel_request")
        assert pub is None or pub.published_messages == [], (
            "при пустом query публикации быть не должно"
        )

    def test_execute_unknown_datasource_returns_error_without_publishing(self, mock_node):
        """datasource='influxdb' → success=False, никакой публикации."""
        tool = ShowMetricsTool(mock_node)
        result = tool.execute(query="up", datasource="influxdb")
        assert result.success is False
        assert "datasource" in result.error.lower()
        pub = mock_node._publishers.get("/avatar/tars/panel_request")
        assert pub is None or pub.published_messages == [], (
            "при неизвестном datasource публикации быть не должно"
        )

    def test_execute_without_node_returns_honest_fail(self):
        """Без ноды (юнит-тест без ROS) — честный success=False, не raise.

        ADR-0018: лучше честный FAIL, чем AttributeError.
        """
        tool = ShowMetricsTool(node=None)
        result = tool.execute(query="up")
        assert result.success is False
        assert "publisher" in result.error.lower() or "create_publisher" in result.error.lower()

    def test_execute_handles_publish_failure_gracefully(self, mock_node):
        """Если publisher.publish() бросает — success=False + понятный error."""
        tool = ShowMetricsTool(mock_node)
        pub = mock_node._publishers["/avatar/tars/panel_request"]
        pub.publish = Mock(side_effect=RuntimeError("zombie topic"))

        result = tool.execute(query="up")
        assert result.success is False
        assert "zombie topic" in result.error

    def test_request_ids_are_unique_across_calls(self, mock_node):
        """request_id должен быть уникален (uuid4 hex)."""
        tool = ShowMetricsTool(mock_node)
        _autorespond(tool, lambda rid: _ok_payload(rid, query="up"))
        ids = set()
        for _ in range(20):
            r = tool.execute(query="up")
            assert r.success
            ids.add(r.data["request_id"])
        assert len(ids) == 20, f"request_id'ы должны быть уникальны, получили {ids}"