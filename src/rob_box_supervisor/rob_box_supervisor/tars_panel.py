"""tars_panel.py — TARS 2 panel dispatcher (issue #2113/#2184, quest #2112).

Captain Bridge в Quest-клиенте показывает боковой экран TARS 2 с метриками
робота. Этот модуль — на стороне ``avatar_supervisor`` — превращает запрос
оператора («покажи дрейф CPU») в **реальные ряды точек** из Prometheus/Loki и
публикует их в ``/avatar/tars/panel_data``; клиент рисует их на своём canvas
(``tars2_metrics_panel.ts``).

Топики (все — ``std_msgs/String`` с JSON):

* ``/avatar/tars/panel_request`` (sub) — запрос от LLM tool-call
  ``show_metrics``: ``{"request_id": str, "query": str,
  "datasource": "prometheus"|"loki"}``.
* ``/avatar/tars/panel_data``   (pub) — **данные** для отрисовки:
  ``{"request_id", "status": "ok"|"empty"|"error", "datasource", "query",
  "note", "summary", "series": [{"name","labels","points":[[ts,val],…]}],
  "lines": [{"ts","line","labels"}], "url", "error"}``.
* ``/avatar/tars/panel_url``    (pub) — legacy-совместимость (issue #2113):
  ``{"request_id", "url", "status", "error"}``. Клиент по-прежнему на него
  подписан, URL теперь ведёт в Grafana Explore (см. ниже).

Почему данные, а не URL (issue #2184)
-------------------------------------

Исходная версия публиковала только URL, а Quest рисовал его текстом
(«preview only · рендер Grafana не подключён»). Оператор не видел ни одной
цифры, и починить это URL'ом нельзя: iframe в immersive-WebXR не рендерится
в принципе, image-renderer в Grafana не установлен, анонимный доступ выключен
(``/api/search`` → 401), а сам URL вёл на несуществующий хост
``prometheus.lan`` и несуществующий дашборд ``d/prometheus-overview``.
Подробности и замеры — в шапке :mod:`rob_box_supervisor.metrics_source`.

Prometheus и Loki отдают JSON по HTTP без авторизации и без плагинов —
оттуда и берём. Grafana остаётся ссылкой «доглядеть с ноутбука».

Что НЕ делаем:

* Не парсим PromQL/LogQL — этим занимается сам Prometheus/Loki.
* Не открываем браузер и не рендерим Grafana — клиент рисует ряды сам.
"""

from __future__ import annotations

import json
import logging
import threading
from typing import TYPE_CHECKING, Any, Callable, Mapping

from std_msgs.msg import String

from rob_box_supervisor.metrics_source import (
    DEFAULT_GRAFANA_URL,
    DEFAULT_LOKI_URL,
    DEFAULT_PROMETHEUS_URL,
    MetricsSource,
    MetricsUnavailable,
    summarize_for_speech,
)

if TYPE_CHECKING:
    # Импорт отложен до момента register_tool(): при импорте модуля
    # ``rob_box_harness`` тянет ``rob_box_llm`` и другие зависимости,
    # которые на CI-стенде без ROS могут быть недоступны.
    from rob_box_harness.tools import ToolHandler, ToolSpec

#: Совместимость с issue #2113: имя константы осталось, адрес — рабочий.
#: ``prometheus.lan`` не резолвился ни с робота, ни с katana (проверено
#: 08.09.2026), поэтому дефолт указывает на Grafana напрямую.
DEFAULT_GRAFANA_BASE_URL = DEFAULT_GRAFANA_URL

#: Поддерживаемые datasource'ы. Держится в синхроне с
#: ``_SHOW_METRICS_DATASOURCES`` в ``rob_box_mcp_tools.tools.operator_admin``.
SUPPORTED_DATASOURCES: frozenset[str] = frozenset({"prometheus", "loki"})


class TarsPanelDispatcher:
    """Подписка / запрос данных / публикация для TARS 2.

    Использование в ``avatar_supervisor.AvatarSupervisor``:

    .. code-block:: python

        self._tars_panel = TarsPanelDispatcher(self)
        self._tars_panel.register_tool(registry)
    """

    def __init__(
        self,
        node: Any,
        *,
        base_url: str = DEFAULT_GRAFANA_BASE_URL,
        prometheus_url: str = DEFAULT_PROMETHEUS_URL,
        loki_url: str = DEFAULT_LOKI_URL,
        panel_request_topic: str = "/avatar/tars/panel_request",
        panel_url_topic: str = "/avatar/tars/panel_url",
        panel_data_topic: str = "/avatar/tars/panel_data",
        metrics: MetricsSource | None = None,
        spawn: Callable[[Callable[[], None]], None] | None = None,
        logger: logging.Logger | None = None,
    ) -> None:
        self._node = node
        self._base_url = base_url.rstrip("/")
        self._request_topic = panel_request_topic
        self._url_topic = panel_url_topic
        self._data_topic = panel_data_topic
        self._log = logger or node.get_logger() if hasattr(node, "get_logger") else logging.getLogger(__name__)
        self._metrics = metrics or MetricsSource(
            prometheus_url=prometheus_url,
            loki_url=loki_url,
            grafana_url=self._base_url,
            logger=self._log if isinstance(self._log, logging.Logger) else None,
        )
        # HTTP к Prometheus/Loki занимает до _HTTP_TIMEOUT_S — в callback'е
        # подписки это заблокировало бы поток executor'а супервизора (а на
        # Pi их немного). Поэтому работа уезжает в отдельный поток; тесты
        # подменяют spawn на синхронный вызов.
        self._spawn = spawn or _spawn_thread
        # Отдельные publisher'ы: «запросы от LLM tool call» и «ответы
        # для Quest-клиента». В ROS все — String JSON, но разные топики.
        self._panel_request_pub = node.create_publisher(String, panel_request_topic, 10)
        self._panel_url_pub = node.create_publisher(String, panel_url_topic, 10)
        self._panel_data_pub = node.create_publisher(String, panel_data_topic, 10)
        node.create_subscription(
            String,
            panel_request_topic,
            self._on_panel_request,
            10,
        )

    # ─── Wire-level: обработка /avatar/tars/panel_request ──────────────

    def _on_panel_request(self, msg: String) -> None:
        """Пришёл запрос от LLM tool call. Валидируем и уходим за данными.

        Битый JSON молча дропаем (это не запрос оператора, а мусор на шине).
        Пустой query / неизвестный datasource — публикуем ``status="error"``
        сразу, без похода в Prometheus.
        """
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as exc:
            self._log.warning(
                f"[tars_panel] /avatar/tars/panel_request: bad JSON: {exc}"
            )
            return
        if not isinstance(payload, dict):
            self._log.warning(
                f"[tars_panel] /avatar/tars/panel_request: not a dict, "
                f"got {type(payload).__name__}"
            )
            return

        request_id = str(payload.get("request_id", "") or "")
        query = str(payload.get("query", "") or "").strip()
        datasource = str(payload.get("datasource", "prometheus") or "prometheus").lower()

        if not query:
            self._publish_result(
                request_id=request_id,
                datasource=datasource,
                result={"status": "error", "error": "empty query"},
                url="",
            )
            return
        if datasource not in SUPPORTED_DATASOURCES:
            self._publish_result(
                request_id=request_id,
                datasource=datasource,
                result={
                    "status": "error",
                    "error": f"unknown datasource: {datasource!r}",
                },
                url="",
            )
            return

        self._spawn(lambda: self._fetch_and_publish(request_id, datasource, query))

    def _fetch_and_publish(
        self, request_id: str, datasource: str, query: str
    ) -> None:
        """Сходить в Prometheus/Loki и опубликовать результат.

        Крутится в рабочем потоке — сюда нельзя пускать исключения, иначе
        поток умрёт молча, а оператор так и будет смотреть в пустую панель.
        """
        try:
            result: dict[str, Any] = self._query(datasource, query)
        except MetricsUnavailable as exc:
            result = {"status": "error", "error": str(exc), "query": query}
        except Exception as exc:  # noqa: BLE001 — рабочий поток, падать нельзя
            self._log.warning(f"[tars_panel] fetch failed: {exc}")
            result = {"status": "error", "error": f"{type(exc).__name__}: {exc}"}

        url = self._metrics.explore_url(datasource, result.get("query") or query)
        self._publish_result(
            request_id=request_id, datasource=datasource, result=result, url=url
        )

    def _query(self, datasource: str, query: str) -> dict[str, Any]:
        """Один вход в источник данных — Prometheus или Loki."""
        if datasource == "loki":
            return self._metrics.query_logs(query)
        return self._metrics.query_range(query)

    def _publish_result(
        self,
        *,
        request_id: str,
        datasource: str,
        result: Mapping[str, Any],
        url: str,
    ) -> None:
        """Опубликовать ``panel_data`` (+ legacy ``panel_url``)."""
        status = str(result.get("status") or "error")
        summary = summarize_for_speech(result)
        data = {
            "request_id": request_id,
            "status": status,
            "datasource": datasource,
            "query": result.get("query", ""),
            "note": result.get("note", ""),
            "summary": summary,
            "series": result.get("series", []),
            "lines": result.get("lines", []),
            "available": result.get("available", []),
            "range_minutes": result.get("range_minutes", 0),
            "url": url,
            "error": str(result.get("error") or ""),
        }
        series_count = len(data["series"]) + len(data["lines"])
        self._log.info(
            f"[tars_panel] panel_data published: request_id={request_id[:8]}, "
            f"datasource={datasource}, status={status}, series={series_count}"
        )
        # ПОРЯДОК ВАЖЕН: сначала legacy-URL, потом данные. Клиент обрабатывает
        # события в порядке прихода, а `setPanelUrl` сбрасывает нарисованный
        # график (ссылка сама по себе данных не несёт). Опубликуй мы URL
        # вторым — он затёр бы только что показанные метрики.
        self._publish(
            self._panel_url_pub,
            self._url_topic,
            {
                "request_id": request_id,
                # На error URL пуст — клиент показывает честную ошибку, а не
                # ссылку, за которой ничего нет.
                "url": url if status != "error" else "",
                "status": "ok" if status == "ok" else "error",
                "error": data["error"] or ("" if status == "ok" else summary),
            },
        )
        self._publish(self._panel_data_pub, self._data_topic, data)

    def _publish(self, publisher: Any, topic: str, payload: Mapping[str, Any]) -> None:
        """Публикация String JSON. Не падает на ROS-ошибках."""
        try:
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            publisher.publish(msg)
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"[tars_panel] {topic} publish failed: {exc}")

    # ─── Чистая логика: query → URL ───────────────────────────────────

    def build_panel_url(self, datasource: str, query: str) -> str:
        """Собрать URL Grafana **Explore** для запроса.

        Дашборд (``/d/<uid>``) параметр ``query`` игнорирует — произвольное
        выражение из URL принимает только Explore. Раньше здесь собирался
        ``{base}/d/prometheus-overview?query=…``: и дашборда такого не
        существует, и параметр был бы проигнорирован.
        """
        return self._metrics.explore_url(datasource, query)

    # ─── LLM tool: show_metrics ──────────────────────────────────────

    SHOW_METRICS_TOOL_NAME = "show_metrics"

    def register_tool(self, registry: Any) -> None:
        """Зарегистрировать ``show_metrics`` в :class:`ToolRegistry`.

        Handler ходит за данными сам и возвращает LLM короткий честный
        статус: сколько рядов пришло и какие последние значения — либо
        «данных нет, есть вот такие метрики». Раньше он возвращал
        «опубликовал запрос», и ТАРС уверенно говорил «открыл» даже когда
        панель оставалась пустой (ADR-0018).

        Импорт ``ToolSpec`` отложен внутрь метода: ``rob_box_harness``
        тянет ``rob_box_llm`` через ``health.py``, и при cold-import
        на CI без ROS падает ``ModuleNotFoundError``.
        """
        try:
            from rob_box_harness.tools import ToolSpec  # noqa: PLC0415
        except ImportError as exc:  # noqa: BLE001
            self._log.warning(
                f"[tars_panel] cannot import ToolSpec, show_metrics "
                f"tool not registered: {exc}"
            )
            return

        spec = ToolSpec(
            name=self.SHOW_METRICS_TOOL_NAME,
            description=(
                "Show live robot telemetry on the operator's TARS 2 screen. "
                "Runs the query against Prometheus (metrics) or Loki (logs) "
                "and returns the latest values, so answer the operator with "
                "what came back — do not claim a panel opened. Use for "
                "charts, metrics, logs, CPU, memory, latency, errors. "
                "Known-good queries: 'rate(process_cpu_seconds_total[5m])' "
                "(CPU), 'process_resident_memory_bytes' (RAM), 'up' "
                "(exporter health), "
                "'rate(voice_llm_request_duration_seconds_sum[5m]) / "
                "rate(voice_llm_request_duration_seconds_count[5m])' "
                "(LLM latency). Plain words like 'cpu' or 'memory' also "
                "work — they are resolved against the live metric catalog."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "query": {
                        "type": "string",
                        "description": (
                            "PromQL expression, LogQL query, or a plain "
                            "metric word ('cpu', 'memory', 'latency')."
                        ),
                    },
                    "datasource": {
                        "type": "string",
                        "enum": ["prometheus", "loki"],
                        "description": "Telemetry backend; default 'prometheus'.",
                        "default": "prometheus",
                    },
                },
                "required": ["query"],
                "additionalProperties": False,
            },
        )

        async def _show_metrics(args: Mapping[str, Any]) -> dict[str, Any]:
            query = str(args.get("query", "") or "").strip()
            datasource = str(
                args.get("datasource", "prometheus") or "prometheus"
            ).lower()
            if not query:
                return {"status": "error", "error": "query is required"}
            if datasource not in SUPPORTED_DATASOURCES:
                return {
                    "status": "error",
                    "error": f"unsupported datasource: {datasource!r}",
                }
            import asyncio  # noqa: PLC0415
            import uuid as _uuid  # noqa: PLC0415

            request_id = _uuid.uuid4().hex[:8]
            try:
                # HTTP синхронный — уводим в поток, чтобы не держать
                # event loop агента на время запроса к Prometheus.
                result: dict[str, Any] = await asyncio.to_thread(
                    self._query, datasource, query
                )
            except MetricsUnavailable as exc:
                result = {"status": "error", "error": str(exc), "query": query}

            url = self._metrics.explore_url(datasource, result.get("query") or query)
            self._publish_result(
                request_id=request_id,
                datasource=datasource,
                result=result,
                url=url,
            )
            return {
                "status": result.get("status", "error"),
                "request_id": request_id,
                "datasource": datasource,
                "query": result.get("query", query),
                "series_count": len(result.get("series") or result.get("lines") or []),
                "message": summarize_for_speech(result),
            }

        # ToolHandler ожидаем sync или async — наша ``_show_metrics`` async,
        # register() принимает обе формы (см. ``tools.ToolHandler``).
        registry.register(spec, _show_metrics, override=True)
        self._log.info(
            "[tars_panel] show_metrics tool registered "
            f"(prometheus={self._metrics_endpoint()})"
        )

    def _metrics_endpoint(self) -> str:
        return getattr(self._metrics, "_prom", "?")


def _spawn_thread(fn: Callable[[], None]) -> None:
    threading.Thread(target=fn, name="tars-panel-fetch", daemon=True).start()


__all__ = [
    "DEFAULT_GRAFANA_BASE_URL",
    "SUPPORTED_DATASOURCES",
    "TarsPanelDispatcher",
]
