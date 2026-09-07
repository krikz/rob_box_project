"""tars_panel.py — TARS 2 panel dispatcher (issue #2113, quest #2112).

Captain Bridge в Quest-клиенте показывает боковой экран TARS 2 с iframe
Grafana-панели. Этот модуль — на стороне ``avatar_supervisor`` — превращает
текстовый запрос оператора («покажи дрейф CPU») в конкретный URL панели и
публикует его в топике ``/avatar/tars/panel_url``, на который Quest-клиент
уже подписан.

Топики (String JSON):

* ``/avatar/tars/panel_request`` (sub) — запрос от LLM tool-call:
    ``{"request_id": str, "query": str, "datasource": "prometheus"|"loki"}``
* ``/avatar/tars/panel_url``      (pub) — ответ для клиента:
    ``{"request_id": str, "url": str, "status": "ok"|"error", "error": str?}``

LLM tool ``show_metrics`` регистрируется через :class:`TarsPanelDispatcher.register_tool`
и публикует panel_request из операторского AgentCore. По умолчанию
обращение к инструменту возвращает строку-статус (например, «опубликовал
URL в /avatar/tars/panel_url»), чтобы LLM мог сказать оператору «Готово,
открыл».

Маппинг ``query → URL`` намеренно простой: в production Grafana у нас
открывается через reverse-proxy Caddy (URL ``/grafana/...``, см.
``docker/vision/Caddyfile``), а внутри Grafana панели параметризованы
переменными. Конкретный шаблон панели и переменные зависят от того, что
развёрнуто на проде, поэтому dispatcher не пытается быть умным: он
формирует URL из комбинации ``datasource`` + ``query`` и оставляет Grafana
разбираться дальше (на её стороне templating/list variables).

Что НЕ делаем:

* Не идём в Prometheus / Loki напрямую — это дело Grafana через datasource.
* Не запускаем PromQL/LokiQL — dispatcher не знает язык запросов. Grafana
  принимает ``?query=...`` в URL и сама его разбирает через datasource.
* Не открываем браузер / не показываем уведомления — это дело клиента и
  текстового ответа LLM оператору.
"""

from __future__ import annotations

import json
import logging
from typing import Any, Callable, Mapping

from std_msgs.msg import String

from rob_box_harness.tools import ToolHandler, ToolSpec

# Дефолтный базовый URL Grafana. В production обычно закрыт за reverse-proxy:
# см. ``docker/vision/Caddyfile`` (``/grafana/* → grafana:3000``). Параметр
# ``grafana_base_url`` ROS-ноды позволяет переопределить (например, для
# тестов на stub-сервере).
DEFAULT_GRAFANA_BASE_URL = "http://prometheus.lan/grafana"

# Datasource → namespace в Grafana. Расширяемо: если в Grafana появятся
# другие datasource'ы — добавляем ключ.
_DATASOURCE_PATH: Mapping[str, str] = {
    "prometheus": "d/prometheus-overview",
    "loki": "d/loki-logs",
}


class TarsPanelDispatcher:
    """Подписка / публикация / регистрация LLM tool для TARS 2.

    Использование в ``avatar_supervisor.AvatarSupervisor`` (issue #2113,
    C4 плана):

    .. code-block:: python

        self._tars_panel = TarsPanelDispatcher(self)
        self._tars_panel.register_tool(registry)
    """

    def __init__(
        self,
        node: Any,
        *,
        base_url: str = DEFAULT_GRAFANA_BASE_URL,
        panel_request_topic: str = "/avatar/tars/panel_request",
        panel_url_topic: str = "/avatar/tars/panel_url",
        logger: logging.Logger | None = None,
    ) -> None:
        self._node = node
        self._base_url = base_url.rstrip("/")
        self._request_topic = panel_request_topic
        self._url_topic = panel_url_topic
        self._log = logger or node.get_logger() if hasattr(node, "get_logger") else logging.getLogger(__name__)
        self._panel_url_pub = node.create_publisher(String, panel_url_topic, 10)
        node.create_subscription(
            String,
            panel_request_topic,
            self._on_panel_request,
            10,
        )

    # ─── Wire-level: обработка /avatar/tars/panel_request ──────────────

    def _on_panel_request(self, msg: String) -> None:
        """Пришёл запрос от LLM tool call. Парсим и публикуем URL.

        Формат входа:

        .. code-block:: json

            {"request_id": "abc", "query": "rate(cpu_usage[5m])",
             "datasource": "prometheus"}

        Битый JSON / неизвестный datasource → публикуем status="error"
        с понятной причиной (Quest-клиент покажет её в TARS 2).
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
            self._publish_url(
                request_id=request_id,
                url="",
                status="error",
                error="empty query",
            )
            return
        if datasource not in _DATASOURCE_PATH:
            self._publish_url(
                request_id=request_id,
                url="",
                status="error",
                error=f"unknown datasource: {datasource!r}",
            )
            return

        url = self.build_panel_url(datasource, query)
        self._log.info(
            f"[tars_panel] panel_url published: request_id={request_id[:8]}, "
            f"datasource={datasource}, query={query[:32]!r}"
        )
        self._publish_url(
            request_id=request_id,
            url=url,
            status="ok",
            error="",
        )

    def _publish_url(
        self,
        *,
        request_id: str,
        url: str,
        status: str,
        error: str,
    ) -> None:
        """Публикация ``/avatar/tars/panel_url``. Не падает на ROS-ошибках."""
        try:
            payload = String()
            payload.data = json.dumps(
                {
                    "request_id": request_id,
                    "url": url,
                    "status": status,
                    "error": error,
                },
                ensure_ascii=False,
            )
            self._panel_url_pub.publish(payload)
        except Exception as exc:  # noqa: BLE001
            self._log.warning(
                f"[tars_panel] /avatar/tars/panel_url publish failed: {exc}"
            )

    # ─── Чистая логика: query → URL ───────────────────────────────────

    def build_panel_url(self, datasource: str, query: str) -> str:
        """Собрать URL Grafana-панели.

        Шаблон: ``{base}/{datasource_path}?query={query_encoded}``.
        Grafana сама разбирает ``query`` через datasource — этот dispatcher
        не знает PromQL/LokiQL и не пытается их валидировать.
        """
        from urllib.parse import quote

        path = _DATASOURCE_PATH[datasource]
        return f"{self._base_url}/{path}?query={quote(query, safe='')}"

    # ─── LLM tool: show_metrics ──────────────────────────────────────

    SHOW_METRICS_TOOL_NAME = "show_metrics"

    def register_tool(self, registry: Any) -> None:
        """Зарегистрировать ``show_metrics`` в :class:`ToolRegistry`.

        ``handler`` публикует запрос в ``/avatar/tars/panel_request`` и
        возвращает короткий текстовый статус — LLM подставит его в ответ
        оператору («Готово, открыл дрейф CPU на TARS 2»). ``spec``
        фиксирует JSON Schema для параметра ``query`` (обязательный) и
        ``datasource`` (опциональный, default ``prometheus``).
        """
        spec = ToolSpec(
            name=self.SHOW_METRICS_TOOL_NAME,
            description=(
                "Open a Grafana panel on the operator's TARS 2 screen with a "
                "PromQL/Loki query. Returns a short status string the "
                "assistant can repeat back to the operator. Use when the "
                "operator asks to see charts, metrics, logs, dreyf, CPU, "
                "latency, errors, or any telemetry from Prometheus/Loki."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "query": {
                        "type": "string",
                        "description": (
                            "PromQL expression (e.g. 'rate(cpu_usage[5m])') "
                            "or LokiQL LogQL query (e.g. '{job=\"voice\"}'."
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
                return {
                    "status": "error",
                    "error": "query is required",
                }
            if datasource not in _DATASOURCE_PATH:
                return {
                    "status": "error",
                    "error": f"unsupported datasource: {datasource!r}",
                }
            # request_id — случайный 8-hex; корреляция с /avatar/tars/panel_url
            # в Quest-клиенте нужна только для дедупа (одинаковый request_id
            # для двух кликов подряд → игнор). В однопользовательском сценарии
            # это просто маркер.
            import uuid as _uuid

            request_id = _uuid.uuid4().hex[:8]
            try:
                payload = String()
                payload.data = json.dumps(
                    {
                        "request_id": request_id,
                        "query": query,
                        "datasource": datasource,
                    },
                    ensure_ascii=False,
                )
                self._panel_url_pub.publish(payload)
            except Exception as exc:  # noqa: BLE001
                return {
                    "status": "error",
                    "error": f"publish failed: {exc}",
                }
            return {
                "status": "published",
                "request_id": request_id,
                "datasource": datasource,
                "query": query,
                "message": (
                    f"Опубликовал {datasource}-запрос «{query}» в "
                    "/avatar/tars/panel_request. Quest откроет панель на "
                    "TARS 2."
                ),
            }

        # ToolHandler ожидаем sync или async — наша ``_show_metrics`` async,
        # register() принимает обе формы (см. ``tools.ToolHandler``).
        registry.register(spec, _show_metrics, override=True)


__all__ = [
    "TarsPanelDispatcher",
    "DEFAULT_GRAFANA_BASE_URL",
]