#!/usr/bin/env python3
"""
operator_admin.py - Операторские инструменты супервизора (ТАРС).

Срез ``operator.admin`` в каталоге — диагностика ROS-системы и контейнеров
без ``docker.sock`` (см. ADR-0051 §6 + docs/architecture/target-operator-agent-and-dialogue.md §6),
а также «межпроцессные» тулзы, которые прокидывают запрос оператора
в другие ноды (issue #2113, TARS 2 metrics panel).

Инструменты:

* :class:`Ros2NodeStatusTool` — состояние критичных ROS2-нод
  (``active``/``missing``/``failed``). Обертка над
  :class:`rob_box_perception.utils.node_monitor.NodeAvailabilityMonitor`,
  плюс прямой вызов ``rclpy.get_node_names()`` (быстрее и дешевле
  ``subprocess ros2 node list`` на Raspberry Pi).
* :class:`ReadLogsTool` — логи конкретной ноды из ``/rosout`` (ROS-side)
  плюс HTTP-запрос к Loki ``/loki/api/v1/query_range`` для контейнеров.
  Через :func:`rob_box_voice.utils.redact.redact_upstream_body` —
  обязательная санитизация перед выдачей агенту.
* :class:`ContainerStatusTool` — restart-count/CPU/RAM/uptime контейнеров
  через Prometheus ``/api/v1/query`` (cAdvisor уже отдаёт метрики).
* :class:`ShowMetricsTool` (issue #2113, #2184) — публикует запрос в топик
  ``/avatar/tars/panel_request``, на который подписан
  :class:`rob_box_supervisor.tars_panel.TarsPanelDispatcher`, и ЖДЁТ ответа
  в ``/avatar/tars/panel_data``: dispatcher выполняет запрос в
  Prometheus/Loki и отдаёт реальные ряды точек (их же рисует Quest-клиент
  на экране TARS 2). Тул возвращает LLM последние значения, чтобы ТАРС
  называл числа, а не рапортовал об открытии панели.

Все четыре ``execution_type = ToolExecutionType.MEDIUM`` (2-10s): зависят от
сетевых вызовов и ROS-discovery. ``read_only=True``.
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
from typing import Any, Dict, List, Optional

from ..base import MCPTool, MCPToolParameter, MCPToolResult, ToolExecutionType

# Локальный TYPE_CHECKING-импорт ROS-типов, чтобы unit-тесты на CI без
# rclpy не падали на этапе импорта модуля. Сам импорт для создания
# publisher'а / String() сообщения делается в ``__init__`` и ``execute``
# соответственно.
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from std_msgs.msg import String

# ----------------------------------------------------------------------------
# Module-level logger (не зависит от ROS 2 — грейсфул degrade в юнит-тестах).
# ----------------------------------------------------------------------------

_LOG = logging.getLogger(__name__)


# ----------------------------------------------------------------------------
# Defaults — overridable через env / конфиг.
# ----------------------------------------------------------------------------

# Стек мониторинга (docker/monitoring/docker-compose.yaml) живёт НЕ рядом с
# нодами: Prometheus/Loki/Grafana подняты на build-машине katana (10.1.1.249)
# в host-сети, а voice-assistant / avatar-supervisor крутятся на Vision Pi
# тоже в host-сети. Docker-DNS между ними нет — старые дефолты
# ``http://prometheus:9090`` / ``http://loki:3100`` с робота не резолвились
# вообще (проверено 08.09.2026: env PROMETHEUS_URL/LOKI_URL в контейнерах не
# задан ни одной строкой compose), из-за чего молча не работали ещё
# ``container_status`` и ``read_logs(source=loki)``. Прямой адрес katana
# отвечает из контейнера avatar-supervisor: /-/healthy → 200.
_DEFAULT_LOKI_URL = os.environ.get("LOKI_URL", "http://10.1.1.249:3100")
_DEFAULT_PROM_URL = os.environ.get("PROMETHEUS_URL", "http://10.1.1.249:9090")
# Таймаут HTTP-запросов — на Pi4 локальные сервисы отвечают <200ms,
# ставим 2s (запас на cold-cache PromQL и медленный Loki).
_HTTP_TIMEOUT_SEC = float(os.environ.get("OPERATOR_ADMIN_HTTP_TIMEOUT", "2.0"))

# Дефолтный список нод — синхронизирован с
# rob_box_perception/utils/node_monitor.py::NodeAvailabilityMonitor.__init__.
# Когда обе ноды (ТАРС и monitor) стартуют в одном контейнере — operator.admin
# должен отдавать тот же список, что и monitor.
_DEFAULT_EXPECTED_NODES: List[str] = [
    "/audio_node",
    "/stt_node",
    "/tts_node",
    "/dialogue_node",
    "/context_aggregator",
    "/camera",
    "/lslidar_driver_node",
    "/rtabmap/rtabmap",
]


# ----------------------------------------------------------------------------
# Helpers (HTTP + rclpy probe)
# ----------------------------------------------------------------------------


def _http_get_json(url: str, params: Optional[Dict[str, str]] = None) -> Optional[Dict[str, Any]]:
    """Минимальный HTTP-GET c JSON-ответом, без requests (уменьшаем зависимости).

    Возвращает ``None`` при любой сетевой/HTTP-ошибке: вызывающий код отдаёт
    оператору честный ``unavailable`` вместо hardcoded ответа.
    """
    target = url
    if params:
        # ``urlencode`` с ``doseq=True`` поддерживает list-valued params
        # (Loki принимает несколько ``{label=...}`` в одном query).
        target = f"{url}?{urllib.parse.urlencode(params, doseq=True)}"
    try:
        req = urllib.request.Request(target, headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=_HTTP_TIMEOUT_SEC) as resp:  # noqa: S310 — operator.env only
            raw = resp.read()
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        _LOG.debug("HTTP %s failed: %s", target, exc)
        return None
    try:
        return json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        _LOG.debug("HTTP %s: bad JSON: %s", target, exc)
        return None


def _probe_ros_node_names(node: Any) -> Optional[List[str]]:
    """Получить список имён ROS2-нод через rclpy-API.

    Возвращает ``None`` если rclpy недоступен / init не выполнен — вызывающий
    код должен fallback'нуть на локальный кеш ``NodeAvailabilityMonitor``.
    """
    try:
        import rclpy  # noqa: WPS433 — локальный импорт для unit-тестов без ROS
    except ImportError:
        return None
    try:
        if not rclpy.ok():
            return None
        return list(rclpy.get_node_names(node=node) if node is not None else rclpy.get_node_names())
    except Exception as exc:  # noqa: BLE001 — rclpy бросает разное
        _LOG.debug("rclpy.get_node_names failed: %s", exc)
        return None


# ----------------------------------------------------------------------------
# ros2_node_status
# ----------------------------------------------------------------------------


class Ros2NodeStatusTool(MCPTool):
    """Статус ROS2-нод из operator-среза.

    Возвращает три категории (``active``/``missing``/``failed``) для каждой
    ожидаемой ноды. Список ожидаемых нод — параметр ``nodes`` (опционально),
    иначе берётся дефолт из константы ``_DEFAULT_EXPECTED_NODES``.
    """

    @property
    def name(self) -> str:
        return "ros2_node_status"

    @property
    def llm_visible(self) -> bool:
        # ТАРС-только (ADR-0051 §6, #2001): личность эти тулы не видит.
        return False

    @property
    def description(self) -> str:
        return (
            "Получить статус ROS2-нод робота (active/missing/failed). "
            "Используй когда нужно понять, поднялась ли нода (например, "
            "'нода stt_node не работает' или 'проверь audio_node')."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="nodes",
                type="array",
                description=(
                    "Опциональный список ROS2-нод для проверки. Если пусто — "
                    "используется дефолтный список критичных нод системы."
                ),
                required=False,
                items=MCPToolParameter(
                    name="node",
                    type="string",
                    description="Полное имя ROS2-ноды с ведущим слэшем.",
                ),
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.MEDIUM

    @property
    def read_only(self) -> bool:
        return True

    def execute(self, nodes: Optional[List[str]] = None) -> MCPToolResult:
        self.log_info("Запрос статуса ROS2-нод")
        expected = list(nodes) if nodes else list(_DEFAULT_EXPECTED_NODES)

        # 1. Прямой rclpy-probe — быстрее subprocess и не плодит процессов.
        live = _probe_ros_node_names(self.node)
        if live is None:
            # 2. Fallback — NodeAvailabilityMonitor уже хранит результаты
            # последнего цикла check_nodes(); используем их, чтобы не
            # блокировать вызов на subprocess ros2 node list.
            monitor_state = self._collect_from_monitor(expected)
            return MCPToolResult(
                success=True,
                data=monitor_state,
                message=(
                    f"Статус ROS2-нод из локального кеша monitor'а: "
                    f"active={monitor_state['active']}/{monitor_state['total']}, "
                    f"missing={monitor_state['missing']}, "
                    f"failed={monitor_state['failed']}"
                ),
            )

        # Построить статус по live-списку: всё что есть — active,
        # чего нет — missing (без 'failed', т.к. один снапшот не отличает).
        live_set = set(live)
        active = [n for n in expected if n in live_set]
        missing = [n for n in expected if n not in live_set]
        summary = {
            "total": len(expected),
            "active": len(active),
            "missing": len(missing),
            "failed": 0,  # требует серии снапшотов
            "active_list": active,
            "missing_list": missing,
            "failed_list": [],
        }
        return MCPToolResult(
            success=True,
            data=summary,
            message=(
                f"ROS2-ноды: active={len(active)}/{len(expected)}, "
                f"missing={len(missing)}"
            ),
        )

    # ------------------------------------------------------------------

    def _collect_from_monitor(self, expected: List[str]) -> Dict[str, Any]:
        """Собрать состояние из ``NodeAvailabilityMonitor`` (если он есть).

        Если monitor не зарегистрирован в ноде (например, в юнит-тестах) —
        все ноды считаем ``missing`` с пометкой ``source=monitor_unavailable``,
        чтобы оператор видел причину, а не ложный "active".
        """
        # ``NodeAvailabilityMonitor`` хранит состояние в атрибуте ``node_status``
        # родительской ноды (см. utils/node_monitor.py). Ищем его через
        # известные пути: ``self.node._node_status_monitor`` (если воркер тула
        # повесил его) или через registry.
        monitor = None
        node = self.node
        if node is not None and hasattr(node, "_node_availability_monitor"):
            monitor = getattr(node, "_node_availability_monitor", None)

        if monitor is None:
            return {
                "total": len(expected),
                "active": 0,
                "missing": len(expected),
                "failed": 0,
                "active_list": [],
                "missing_list": list(expected),
                "failed_list": [],
                "source": "monitor_unavailable",
                "checked_at": time.time(),
            }

        status = monitor.node_status
        active = [n for n in expected if status.get(n, {}).get("status") == "active"]
        failed = [n for n in expected if status.get(n, {}).get("status") == "failed"]
        missing = [n for n in expected if n not in status or status[n].get("status") == "missing"]
        return {
            "total": len(expected),
            "active": len(active),
            "missing": len(missing),
            "failed": len(failed),
            "active_list": active,
            "missing_list": missing,
            "failed_list": failed,
            "source": "node_availability_monitor",
            "checked_at": time.time(),
        }


# ----------------------------------------------------------------------------
# read_logs
# ----------------------------------------------------------------------------


class ReadLogsTool(MCPTool):
    """Читать логи конкретной ROS2-ноды (через ``/rosout``) или контейнера.

    Два источника:

    * ``source="rosout"`` (по умолчанию): локальный кеш
      :class:`rob_box_perception.health_monitor.HealthMonitor` — даёт
      точно-по-ноде ERROR/WARN за последние ~20 событий.
    * ``source="loki"``: HTTP-запрос ``/loki/api/v1/query_range`` —
      контейнерные логи (все 22 контейнера). ``node`` интерпретируется
      как label ``service``.

    Перед возвратом — обязательный прогон через ``redact_upstream_body``
    (DEEPSEEK_API_KEY и др. лежат рядом, deepseek.py:190 дампит контекст
    в stderr — Issue #6 архитектуры §6.3).
    """

    @property
    def name(self) -> str:
        return "read_logs"

    @property
    def llm_visible(self) -> bool:
        # ТАРС-только (ADR-0051 §6, #2001): личность эти тулы не видит.
        return False

    @property
    def description(self) -> str:
        return (
            "Получить логи конкретной ROS2-ноды или контейнера. "
            "Используй когда нужно посмотреть почему нода не работает "
            "('покажи логи stt_node', 'что в логах voice-assistant')."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="node",
                type="string",
                description="Имя ROS2-ноды или контейнера для чтения логов.",
                required=True,
            ),
            MCPToolParameter(
                name="source",
                type="string",
                description=(
                    "'rosout' — локальные логи /rosout из health_monitor; "
                    "'loki' — контейнерные логи через Loki HTTP API."
                ),
                required=False,
                enum=["rosout", "loki"],
            ),
            MCPToolParameter(
                name="limit",
                type="integer",
                description="Максимум строк (default: 20 для rosout, 50 для loki).",
                required=False,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.MEDIUM

    @property
    def read_only(self) -> bool:
        return True

    def execute(
        self,
        node: str,
        source: str = "rosout",
        limit: Optional[int] = None,
    ) -> MCPToolResult:
        self.log_info(f"Чтение логов: node={node} source={source}")
        if source == "loki":
            return self._read_loki(node, limit or 50)
        return self._read_rosout(node, limit or 20)

    # ------------------------------------------------------------------

    def _read_rosout(self, node: str, limit: int) -> MCPToolResult:
        """Локальный кеш health_monitor'а — ERROR/WARN для указанной ноды."""
        # Ищем HealthMonitor по известному атрибуту — воркер тула регистрирует
        # его в ``self.node._health_monitor`` (см. operator_node main()).
        monitor = None
        if self.node is not None and hasattr(self.node, "_health_monitor"):
            monitor = getattr(self.node, "_health_monitor", None)

        if monitor is None:
            return MCPToolResult(
                success=True,
                data={
                    "node": node,
                    "source": "rosout",
                    "lines": [],
                    "monitor_unavailable": True,
                },
                message=(
                    f"HealthMonitor не зарегистрирован в ноде — локальные "
                    f"логи {node} недоступны. Попробуй source=loki."
                ),
            )

        all_lines = list(getattr(monitor, "errors", [])) + list(getattr(monitor, "warnings", []))
        # Фильтр по имени (msg.name — полное имя ROS2-ноды)
        filtered = [
            entry for entry in all_lines if entry.get("node") == node
        ]
        filtered.sort(key=lambda e: e.get("time", 0), reverse=True)
        lines = filtered[:limit]
        # Санитизация обязательна (§6.3): даже /rosout может нести сообщения
        # от провайдеров LLM, которые любят дампить headers.
        sanitized = [_sanitize_entry(entry) for entry in lines]
        return MCPToolResult(
            success=True,
            data={
                "node": node,
                "source": "rosout",
                "lines": sanitized,
                "count": len(sanitized),
            },
            message=f"/rosout для {node}: найдено {len(sanitized)} записей",
        )

    def _read_loki(self, service: str, limit: int) -> MCPToolResult:
        """Контейнерные логи из Loki."""
        # Окно последних 5 минут — оператор смотрит «что только что упало»,
        # а не историю за час. Узкое окно держит ответ <100ms.
        end = int(time.time())
        start = end - 300
        params = {
            "query": f'{{service="{service}"}}',
            "start": str(start * 1_000_000_000),  # ns
            "end": str(end * 1_000_000_000),
            "limit": str(limit),
            "direction": "backward",
        }
        url = f"{_DEFAULT_LOKI_URL.rstrip('/')}/loki/api/v1/query_range"
        body = _http_get_json(url, params)
        if body is None:
            return MCPToolResult(
                success=False,
                data={"service": service, "lines": []},
                error=(
                    f"Loki недоступен ({_DEFAULT_LOKI_URL}); "
                    f"невозможно получить логи контейнера {service}"
                ),
            )
        # Loki возвращает data.result[*].values[[ts_ns, line], ...]
        lines: List[Dict[str, Any]] = []
        for stream in body.get("data", {}).get("result", []):
            stream_labels = stream.get("stream", {})
            container = stream_labels.get("container", service)
            for ts_ns, raw in stream.get("values", [])[:limit]:
                lines.append(
                    {
                        "ts": ts_ns,
                        "service": service,
                        "container": container,
                        "line": _sanitize_text(raw),
                    }
                )
        # Сортируем по ts (Loki streams уже отсортированы в обратном порядке,
        # но мульти-stream может перемешать) и обрезаем.
        lines.sort(key=lambda x: x["ts"], reverse=True)
        lines = lines[:limit]
        return MCPToolResult(
            success=True,
            data={"service": service, "source": "loki", "lines": lines, "count": len(lines)},
            message=f"Loki {service}: {len(lines)} строк",
        )


# ----------------------------------------------------------------------------
# container_status
# ----------------------------------------------------------------------------


class ContainerStatusTool(MCPTool):
    """Метрики контейнеров через Prometheus (cAdvisor backend).

    Запрашивает restart-count, CPU, RAM, uptime для каждого контейнера
    через PromQL. Параллелит три запроса — каждый ~50ms на локальном
    Prometheus'е, суммарно укладывается в MEDIUM-бюджет.

    ``name`` — подстрока имени контейнера/сервиса (используем ``=~``).
    Пусто = все контейнеры (summary).
    """

    @property
    def name(self) -> str:
        return "container_status"

    @property
    def llm_visible(self) -> bool:
        # ТАРС-только (ADR-0051 §6, #2001): личность эти тулы не видит.
        return False

    @property
    def description(self) -> str:
        return (
            "Получить статус контейнеров (restart-count, CPU, RAM, uptime) "
            "через Prometheus/cAdvisor. Используй когда нужно проверить "
            "здоровье контейнеров ('покажи состояние voice-assistant')."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="name",
                type="string",
                description=(
                    "Имя (или подстрока имени) контейнера. Пусто — все контейнеры."
                ),
                required=False,
            ),
        ]

    @property
    def execution_type(self) -> ToolExecutionType:
        return ToolExecutionType.MEDIUM

    @property
    def read_only(self) -> bool:
        return True

    def execute(self, name: Optional[str] = None) -> MCPToolResult:
        self.log_info(f"container_status: name={name or '<all>'}")

        # Три независимых PromQL-запроса: restart count, CPU rate, memory.
        # Uptime выводим из time() - container_start_time_seconds.
        container_filter = f'name=~".*{name}.*"' if name else 'name=~".+"'

        restart_q = (
            f'sum by (name) (kube_pod_container_status_restarts_total{{{container_filter}}})'
        )
        cpu_q = (
            f'sum by (name) (rate(container_cpu_usage_seconds_total{{{container_filter}}}[1m]))'
        )
        mem_q = (
            f'sum by (name) (container_memory_usage_bytes{{{container_filter}}})'
        )
        start_q = (
            f'min by (name) (container_start_time_seconds{{{container_filter}}})'
        )

        url = f"{_DEFAULT_PROM_URL.rstrip('/')}/api/v1/query"
        restart_body = _http_get_json(url, {"query": restart_q})
        cpu_body = _http_get_json(url, {"query": cpu_q})
        mem_body = _http_get_json(url, {"query": mem_q})
        start_body = _http_get_json(url, {"query": start_q})

        # Любой None — частичный успех (Prometheus может отвечать, но один
        # из метрик отсутствует). Возвращаем то что есть + список unavailable.
        unavailable: List[str] = []
        if restart_body is None:
            unavailable.append("restart_count")
        if cpu_body is None:
            unavailable.append("cpu_rate")
        if mem_body is None:
            unavailable.append("memory_usage")
        if start_body is None:
            unavailable.append("start_time")

        # Полностью недоступен Prometheus — fail честно.
        if len(unavailable) == 4:
            return MCPToolResult(
                success=False,
                data={"containers": []},
                error=(
                    f"Prometheus недоступен ({_DEFAULT_PROM_URL}); "
                    f"container_status невозможно получить"
                ),
            )

        # Сливаем метрики по имени контейнера.
        # ``cast(value_raw)`` всегда float; имя ключа в bucket живёт отдельно.
        by_name: Dict[str, Dict[str, Any]] = {}
        for metric_key, body in (
            ("restart_count", restart_body),
            ("cpu_rate_per_sec", cpu_body),
            ("memory_bytes", mem_body),
            ("start_time_unix", start_body),
        ):
            if body is None:
                continue
            for series in body.get("data", {}).get("result", []):
                metric_name = series["metric"].get("name", "<unknown>")
                bucket = by_name.setdefault(metric_name, {"name": metric_name})
                value_raw = series.get("value", [None, "0"])[1]
                try:
                    bucket[metric_key] = float(value_raw)
                except (TypeError, ValueError):
                    bucket[metric_key] = None

        now = time.time()
        containers: List[Dict[str, Any]] = []
        for cname, metrics in by_name.items():
            start_ts = metrics.get("start_time_unix")
            uptime_sec = max(0.0, now - start_ts) if start_ts else None
            containers.append({
                "name": cname,
                "restart_count": int(metrics["restart_count"]) if metrics.get("restart_count") is not None else None,
                "cpu_rate_per_sec": metrics.get("cpu_rate_per_sec"),
                "memory_bytes": metrics.get("memory_bytes"),
                "uptime_sec": uptime_sec,
            })
        containers.sort(key=lambda c: c["name"])

        return MCPToolResult(
            success=True,
            data={
                "containers": containers,
                "count": len(containers),
                "unavailable_metrics": unavailable,
            },
            message=(
                f"container_status: {len(containers)} контейнеров, "
                f"недоступно метрик: {len(unavailable)}"
            ),
        )


# ----------------------------------------------------------------------------
# Sanitization — обязательная (§6.3 архитектуры).
# ----------------------------------------------------------------------------


def _sanitize_text(text: str) -> str:
    """Прогнать текст через redact-цепочку, fallback на regex passthrough.

    Issue #1998 §6.3: даже /rosout и Loki-контейнерные логи могут
    нести ``DEEPSEEK_API_KEY=...`` / ``Authorization: Bearer ...`` /
    bare JWT. Поэтому прогоняем **обе** функции:

    1. :func:`redact_upstream_body` — заголовки и JSON-поля (LLM-стек).
    2. :func:`redact_log_text` — env-var ``KEY=VALUE`` / CLI ``--flag=``
       / bare JWT и vendor-prefixed токены (``sk-``, ``ghp_``, ``xoxb-``).

    Идемпотентно: повторный прогон по уже реднутому тексту даёт тот же
    результат (важно — некоторые upstream-и уже маскируют часть полей
    сами, наш слой должен быть устойчив к этому).
    """
    try:
        # Локальный импорт — утилита живёт в rob_box_voice, чтобы не
        # зависеть от него жёстко на уровне тула.
        from rob_box_voice.utils.redact import (
            redact_log_text,
            redact_upstream_body,
        )
        return redact_log_text(redact_upstream_body(text))
    except ImportError:
        return text


def _sanitize_entry(entry: Dict[str, Any]) -> Dict[str, Any]:
    """Санитизировать одну запись из health_monitor'а (не трогаем node/level/ts)."""
    out = dict(entry)
    if "msg" in out and isinstance(out["msg"], str):
        out["msg"] = _sanitize_text(out["msg"])
    return out


# ----------------------------------------------------------------------------
# show_metrics (issue #2113 / TARS 2 metrics panel)
# ----------------------------------------------------------------------------
#
# Контракт входа/выхода зафиксирован в
# ``rob_box_supervisor.tars_panel.TarsPanelDispatcher`` (issue #2113):
#
# * ``/avatar/tars/panel_request`` (sub) — JSON ``{"request_id": str,
#   "query": str, "datasource": "prometheus"|"loki"}``;
# * ``/avatar/tars/panel_url`` (pub) — JSON ``{"request_id": str,
#   "url": str, "status": "ok"|"error", "error": str?}``.
#
# Этот тул — MCP-обёртка первой части конвейера: публикует запрос, и
# сразу возвращает LLM короткий текстовый статус («Опубликовал запрос»).
# Сам URL собирает ``TarsPanelDispatcher`` и публикует в ``/avatar/tars/panel_url``
# отдельно — Quest-клиент уже подписан на этот топик.
#
# Грейсфул degrade: если нода не ROS (юнит-тесты без rclpy) —
# ``execute()`` возвращает ``success=False`` с понятным сообщением,
# вместо AttributeError. Это согласуется с ADR-0018 (честный FAIL).


# Поддерживаемые datasource'ы — те же, что и в TarsPanelDispatcher
# (см. _DATASOURCE_PATH в ``rob_box_supervisor.tars_panel``). Дублируем
# список здесь, чтобы тул умел валидировать ``datasource`` ДО публикации
# (TARS panel dispatcher всё равно их проверит и опубликует status=error
# при неизвестном — но мы экономим round-trip и не шумим в логе
# ``/avatar/tars/panel_request`` с заведомо битыми запросами).
_SHOW_METRICS_DATASOURCES = ("prometheus", "loki")

# Сколько ждём ответ супервизора на ``/avatar/tars/panel_data``.
# TarsPanelDispatcher ходит в Prometheus по HTTP (таймаут 5s) в отдельном
# потоке, так что 8s покрывают худший случай с запасом на ROS-доставку.
# Ожидание — не роскошь: без него LLM получал «опубликовал запрос» и бодро
# сообщал оператору «открыл дрейф CPU», даже когда метрики не существует и
# панель оставалась пустой (issue #2184, ADR-0018).
_SHOW_METRICS_TIMEOUT_SEC = float(
    os.environ.get("SHOW_METRICS_TIMEOUT", "8.0")
)


def _latest_values(series: Any) -> Dict[str, Any]:
    """Последнее значение каждого ряда — то, что ТАРС называет голосом.

    Для Loki рядов нет, есть строки: тогда отдаём пусто, а LLM опирается на
    ``message``. Формат ряда — контракт ``/avatar/tars/panel_data``
    (``{"name", "labels", "points": [[ts, value], …]}``).
    """
    out: Dict[str, Any] = {}
    if not isinstance(series, list):
        return out
    for item in series[:6]:
        if not isinstance(item, dict):
            continue
        points = item.get("points")
        if not isinstance(points, list) or not points:
            continue
        last = points[-1]
        try:
            out[str(item.get("name") or "series")] = float(last[1])
        except (TypeError, ValueError, IndexError):
            continue
    return out


class ShowMetricsTool(MCPTool):
    """Публикация LLM tool-call ``show_metrics`` в ``/avatar/tars/panel_request``.

    Используется оператором (ТАРС, через ``/mcp/execute``) — показать
    операторский мониторинг (Grafana) на боковом экране TARS 2
    (Captain Bridge в Quest-клиенте). Тонкая обёртка над двумя топиками:

    * in:  ``query`` (PromQL/LogQL) + опциональный ``datasource``;
    * out: запрос в ``/avatar/tars/panel_request``, ответ с URL — в
      ``/avatar/tars/panel_url`` (в Quest).

    Сама сборка URL — в :class:`rob_box_supervisor.tars_panel.TarsPanelDispatcher`
    (его публикация в ``/avatar/tars/panel_url`` уже подписана
    Quest-клиентом, см. ADR-0060). Этот тул ничего не знает про
    Grafana/Prometheus — только шлёт запрос и ждёт ответа через ROS-шину.
    """

    @property
    def name(self) -> str:
        return "show_metrics"

    @property
    def llm_visible(self) -> bool:
        # ТАРС-только (#2113/#2184): личность эти тулы не видит.
        return False

    @property
    def description(self) -> str:
        return (
            "Показать телеметрию робота на боковом экране TARS 2 "
            "(Captain Bridge) и получить её значения. Тул выполняет запрос "
            "в Prometheus (метрики) или Loki (логи) и возвращает последние "
            "значения — отвечай оператору ИМИ, а не фразой «открыл панель». "
            "Используй когда просят показать графики, метрики, логи, CPU, "
            "память, latency, ошибки ('покажи дрейф CPU', 'открой ошибки "
            "stt_node'). Рабочие запросы: "
            "'rate(process_cpu_seconds_total[5m])' — CPU, "
            "'process_resident_memory_bytes' — RAM, 'up' — живость "
            "экспортеров, 'rate(voice_llm_request_duration_seconds_sum[5m]) "
            "/ rate(voice_llm_request_duration_seconds_count[5m])' — "
            "задержка LLM. Простые слова ('cpu', 'память', 'latency') тоже "
            "принимаются — они резолвятся по живому каталогу метрик."
        )

    @property
    def parameters(self) -> List[MCPToolParameter]:
        return [
            MCPToolParameter(
                name="query",
                type="string",
                description=(
                    "PromQL-выражение (напр. "
                    "'rate(process_cpu_seconds_total[5m])'), LogQL-запрос "
                    "(напр. '{job=\"voice\"}') или простое слово-метрика "
                    "('cpu', 'память', 'latency'). Обязательный."
                ),
                required=True,
            ),
            MCPToolParameter(
                name="datasource",
                type="string",
                description=(
                    "Источник телеметрии: 'prometheus' (default) или 'loki'."
                ),
                required=False,
                enum=list(_SHOW_METRICS_DATASOURCES),
                default="prometheus",
            ),
        ]

    @property
    def slice(self) -> str:
        # ADR-0052 / issue #1998 §6.2 + issue #2113 (TARS 2 metrics).
        # Диагностика/визуализация для оператора — operator.admin.
        return "operator.admin"

    @property
    def execution_type(self) -> ToolExecutionType:
        # Не FAST: тул ждёт, пока супервизор реально сходит в Prometheus/Loki
        # и вернёт данные в ``/avatar/tars/panel_data`` (до
        # _SHOW_METRICS_TIMEOUT_SEC). Это цена честного ответа оператору.
        return ToolExecutionType.MEDIUM

    @property
    def read_only(self) -> bool:
        return True

    def __init__(self, node: Optional[Any] = None) -> None:
        super().__init__(node)
        # Publisher создаём ТОЛЬКО если есть ROS-нода. В юнит-тестах
        # передают ``node=Mock()`` без ``create_publisher`` — поэтому
        # сначала проверяем, что у node есть соответствующий атрибут.
        # Если нет — ``self._panel_request_pub`` останется ``None``,
        # и ``execute()`` вернёт честный ``success=False`` вместо
        # падения (ADR-0018).
        self._panel_request_pub: Optional[Any] = None
        # request_id → результат из /avatar/tars/panel_data. Ключи чистим
        # сразу после чтения — словарь не растёт (одна запись на вызов).
        self._pending: Dict[str, Dict[str, Any]] = {}
        self._pending_events: Dict[str, threading.Event] = {}
        self._pending_lock = threading.Lock()
        if node is not None and hasattr(node, "create_publisher"):
            from std_msgs.msg import String  # noqa: PLC0415 — локальный импорт

            self._panel_request_pub = node.create_publisher(
                String, "/avatar/tars/panel_request", 10
            )
            # Обратный канал: супервизор публикует сюда данные, которые
            # реально приехали из Prometheus/Loki. Подписка — в СВОЕЙ
            # ReentrantCallbackGroup: execute() блокируется на Event, и с
            # MultiThreadedExecutor (см. mcp_server._make_executor) callback
            # должен уметь отработать в другом потоке, иначе получим дедлок.
            if hasattr(node, "create_subscription"):
                try:
                    group = None
                    try:
                        from rclpy.callback_groups import (  # noqa: PLC0415
                            ReentrantCallbackGroup,
                        )

                        group = ReentrantCallbackGroup()
                    except ImportError:
                        pass  # стенд без rclpy — подписка всё равно нужна
                    kwargs = {"callback_group": group} if group is not None else {}
                    node.create_subscription(
                        String,
                        "/avatar/tars/panel_data",
                        self._on_panel_data,
                        10,
                        **kwargs,
                    )
                except Exception as exc:  # noqa: BLE001
                    # Без обратного канала тул продолжает работать — просто
                    # честно сообщит о таймауте вместо конкретных чисел.
                    _LOG.warning(
                        f"show_metrics: panel_data subscription failed: {exc}"
                    )

    def _on_panel_data(self, msg: Any) -> None:
        """Результат от TarsPanelDispatcher → разбудить ждущий execute()."""
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError, AttributeError):
            return
        if not isinstance(payload, dict):
            return
        request_id = str(payload.get("request_id") or "")
        if not request_id:
            return
        with self._pending_lock:
            event = self._pending_events.get(request_id)
            if event is None:
                return  # чужой/протухший запрос — не наш вызов
            self._pending[request_id] = payload
        event.set()

    def _arm_panel_data(self, request_id: str) -> "threading.Event":
        """Подписаться на свой ``request_id`` ДО публикации запроса.

        Порядок важен: супервизор на быстром кэше отвечает раньше, чем
        execute() успевает дойти до ожидания, — если регистрировать Event
        после publish, ответ уходит в никуда и тул врёт про таймаут.
        """
        event = threading.Event()
        with self._pending_lock:
            self._pending_events[request_id] = event
        return event

    def _await_panel_data(
        self, request_id: str, event: "threading.Event", timeout: float
    ) -> Optional[Dict[str, Any]]:
        """Дождаться ``panel_data`` с нашим ``request_id`` (или None)."""
        try:
            if not event.wait(timeout):
                return None
            with self._pending_lock:
                return self._pending.get(request_id)
        finally:
            with self._pending_lock:
                self._pending_events.pop(request_id, None)
                self._pending.pop(request_id, None)

    def execute(
        self,
        query: str,
        datasource: str = "prometheus",
    ) -> MCPToolResult:
        """Опубликовать запрос в ``/avatar/tars/panel_request``.

        Возвращает короткий статус — LLM проговорит его оператору
        («Готово, открыл панель на TARS 2»). Сам URL собирает
        ``TarsPanelDispatcher`` в ``rob_box_supervisor.tars_panel``
        и публикует в ``/avatar/tars/panel_url`` (на этот топик уже
        подписан Quest-клиент).
        """
        query = (query or "").strip()
        datasource = (datasource or "prometheus").strip().lower() or "prometheus"

        # Валидация — ДО публикации, чтобы не зашумлять шину заведомо
        # битыми запросами. TarsPanelDispatcher их всё равно отвергнет,
        # но мы экономим round-trip и держим поведение консистентным
        # с ``MCPTool.validate_parameters`` (тул уже проверил, что
        # ``query`` непустой).
        if not query:
            return MCPToolResult(
                success=False,
                error="query is required",
            )
        if datasource not in _SHOW_METRICS_DATASOURCES:
            return MCPToolResult(
                success=False,
                error=f"unsupported datasource: {datasource!r}",
            )

        self.log_info(
            f"show_metrics: datasource={datasource}, query={query[:64]!r}"
        )

        # 8-hex request_id — совпадает с TarsPanelDispatcher (uuid4 hex).
        # Корреляция между запросом и публикацией URL в
        # ``/avatar/tars/panel_url`` нужна только для дедупа в
        # Quest-клиенте (issue #2113 §«request_id»).
        request_id = uuid.uuid4().hex[:8]
        if self._panel_request_pub is None:
            # Нода без ROS (юнит-тест без rclpy.create_publisher) — честный
            # отказ вместо AttributeError. ADR-0018: «честный FAIL лучше
            # красивого PASS».
            return MCPToolResult(
                success=False,
                data={"request_id": request_id},
                error=(
                    "ROS publisher недоступен: /avatar/tars/panel_request "
                    "не опубликован (node без create_publisher)"
                ),
            )

        event = self._arm_panel_data(request_id)
        try:
            from std_msgs.msg import String  # noqa: PLC0415 — локальный импорт

            payload = String()
            payload.data = json.dumps(
                {
                    "request_id": request_id,
                    "query": query,
                    "datasource": datasource,
                },
                ensure_ascii=False,
            )
            self._panel_request_pub.publish(payload)
        except Exception as exc:  # noqa: BLE001
            self._await_panel_data(request_id, event, 0.0)  # снять регистрацию
            return MCPToolResult(
                success=False,
                data={"request_id": request_id},
                error=f"publish failed: {exc}",
            )

        result = self._await_panel_data(
            request_id, event, _SHOW_METRICS_TIMEOUT_SEC
        )
        if result is None:
            # Супервизор не ответил: либо avatar-supervisor лежит, либо
            # TarsPanelDispatcher не поднялся. Врать «открыл» нельзя —
            # оператор будет смотреть в пустой экран (ADR-0018).
            return MCPToolResult(
                success=False,
                data={
                    "request_id": request_id,
                    "datasource": datasource,
                    "query": query,
                },
                error=(
                    f"avatar_supervisor не ответил за {_SHOW_METRICS_TIMEOUT_SEC:.0f}s "
                    "(/avatar/tars/panel_data). Панель TARS 2 не обновилась."
                ),
            )

        status = str(result.get("status") or "error")
        summary = str(result.get("summary") or "")
        series = result.get("series") or result.get("lines") or []
        data = {
            "request_id": request_id,
            "datasource": datasource,
            "query": result.get("query") or query,
            "status": status,
            "series_count": len(series) if isinstance(series, list) else 0,
            "url": result.get("url", ""),
            "note": result.get("note", ""),
            "available": result.get("available", []),
            # Последние значения — чтобы LLM мог назвать числа голосом, а не
            # пересказывать факт открытия панели.
            "latest": _latest_values(series),
        }
        if status == "error":
            return MCPToolResult(
                success=False,
                data=data,
                error=str(result.get("error") or summary or "unknown error"),
            )
        # status == "empty" — это не сбой тракта, а честный «данных нет»:
        # success=True, но message прямо говорит об этом, чтобы LLM не
        # придумал цифры.
        return MCPToolResult(success=True, data=data, message=summary)