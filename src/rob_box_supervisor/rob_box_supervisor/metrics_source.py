"""metrics_source.py — реальные данные для TARS 2 (issue #2184, PR #2185).

До этого модуля тракт ``show_metrics`` заканчивался URL'ом: dispatcher
собирал строку вида ``http://prometheus.lan/grafana/d/prometheus-overview?query=…``
и публиковал её в ``/avatar/tars/panel_url``, а Quest-клиент рисовал host и
path текстом («preview only · рендер Grafana не подключён»). Метрик оператор
не видел никогда — ни одной цифры.

Причины, по которым URL-путь не мог заработать (проверено на стенде
08.09.2026, katana + Vision Pi):

* ``prometheus.lan`` не резолвится (``getent hosts`` пусто), reverse-proxy
  ``docker/vision/Caddyfile`` из docstring'а не существует. Grafana живёт на
  ``http://10.1.1.249:3000`` напрямую.
* дашбордов ``d/prometheus-overview`` / ``d/loki-logs`` в Grafana нет
  (реальные UID — ``rob_box_demo_1`` и т.п.), а ``?query=`` дашборд всё равно
  игнорирует: этот параметр понимает только Explore.
* анонимный доступ в Grafana выключен (``/api/search`` → 401), image-renderer
  не установлен — значит ни iframe, ни PNG-тракт из коробки не отдадут
  картинку.
* настоящий ``<iframe>`` в immersive-WebXR не рендерится в принципе
  (см. шапку ``tars2_metrics_panel.ts``).

Поэтому данные берём там, где они лежат без авторизации и без плагинов — в
HTTP API самих Prometheus и Loki, — и отдаём клиенту рядами точек, которые он
рисует на своём canvas. Grafana остаётся только источником ссылки «открыть на
десктопе» (Explore-URL, а не выдуманный дашборд).

Модуль намеренно без ROS-зависимостей: чистые функции + HTTP, тестируется
без rclpy. ROS-часть (топики, publisher'ы) — в :mod:`tars_panel`.
"""

from __future__ import annotations

import difflib
import json
import logging
import os
import re
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Callable, Iterable, Mapping, Sequence

_LOG = logging.getLogger(__name__)

# ─────────────────────────── адреса стенда ───────────────────────────
#
# Мониторинг (Prometheus/Loki/Grafana, docker/monitoring/docker-compose.yaml)
# крутится на build-машине katana = 10.1.1.249, все три сервиса в host-сети.
# Проверено из контейнера avatar-supervisor на Vision Pi: /-/healthy → 200,
# /api/health → 200. Переопределяется env'ом (docker-compose vision) или
# ROS-параметром ноды.

DEFAULT_PROMETHEUS_URL = os.getenv("PROMETHEUS_URL", "http://10.1.1.249:9090")
DEFAULT_LOKI_URL = os.getenv("LOKI_URL", "http://10.1.1.249:3100")
DEFAULT_GRAFANA_URL = os.getenv("GRAFANA_URL", "http://10.1.1.249:3000")

# Окно по умолчанию для query_range: 15 минут — достаточно, чтобы увидеть
# «дрейф», и достаточно мало, чтобы 60 точек хватило на панель 1280 px.
DEFAULT_RANGE_MINUTES = 15
DEFAULT_MAX_POINTS = 120

# Сколько рядов и точек максимум уезжает в Quest. Панель 4.8 × 2.7 м читается
# оператором с расстояния ~2 м: больше 6 линий там превращается в кашу, а
# JSON по WebSocket на Pi не хочется раздувать (ADR-0060 — payload'ы TARS
# идут тем же каналом, что и телеметрия).
MAX_SERIES = 6
MAX_LOG_LINES = 40

_HTTP_TIMEOUT_S = 5.0

# ───────────────────────── каталог запросов ──────────────────────────
#
# LLM генерирует PromQL по памяти и промахивается: в логах avatar-supervisor
# (08.09.2026) на «покажи дрейф CPU» приехало ``rate(network_latency_ms[5m])``
# — метрики с таким именем в нашем Prometheus нет и не было. Раньше это
# кончалось пустой панелью без объяснений; теперь неизвестное имя резолвится
# по каталогу, а если резолв не удался — оператор видит честное «нет такой
# метрики» и список того, что есть (ADR-0018).
#
# Ключ — то, что LLM (или оператор) реально произносит; значение — рабочий
# PromQL. Проверено на стенде: все выражения возвращают непустой результат.

QUERY_ALIASES: Mapping[str, str] = {
    # CPU. cAdvisor на обеих Pi поднимается только с профилем "monitoring"
    # (docker/{main,vision}/docker-compose.yaml: profiles: ["monitoring"]) и
    # сейчас не запущен — оба таргета cadvisor-* в Prometheus down. Поэтому
    # «CPU» = process_cpu_seconds_total, который отдают сами voice-ноды
    # через prometheus_client. Это честный CPU процессов ТАРСа, просто не
    # общесистемный.
    "cpu": "rate(process_cpu_seconds_total[5m])",
    "cpu_usage": "rate(process_cpu_seconds_total[5m])",
    "cpu_percent": "rate(process_cpu_seconds_total[5m])",
    "node_cpu_seconds_total": "rate(process_cpu_seconds_total[5m])",
    "container_cpu_usage_seconds_total": "rate(process_cpu_seconds_total[5m])",
    # Память.
    "memory": "process_resident_memory_bytes",
    "mem": "process_resident_memory_bytes",
    "ram": "process_resident_memory_bytes",
    "container_memory_usage_bytes": "process_resident_memory_bytes",
    "node_memory_MemAvailable_bytes": "process_resident_memory_bytes",
    # Задержки голосового тракта — то, ради чего метрики вообще собирают.
    "latency": "rate(voice_llm_request_duration_seconds_sum[5m]) "
    "/ rate(voice_llm_request_duration_seconds_count[5m])",
    "network_latency_ms": "rate(voice_llm_request_duration_seconds_sum[5m]) "
    "/ rate(voice_llm_request_duration_seconds_count[5m])",
    "llm_latency": "rate(voice_llm_request_duration_seconds_sum[5m]) "
    "/ rate(voice_llm_request_duration_seconds_count[5m])",
    "tts_latency": "rate(voice_tts_synthesize_duration_seconds_sum[5m]) "
    "/ rate(voice_tts_synthesize_duration_seconds_count[5m])",
    "stt_latency": "rate(voice_stt_recognize_duration_seconds_sum[5m]) "
    "/ rate(voice_stt_recognize_duration_seconds_count[5m])",
    # Живость экспортеров — «кто отвечает».
    "uptime": "up",
    "health": "up",
    "errors": "rate(voice_llm_request_total[5m])",
    # Оператор говорит по-русски, и LLM иногда прокидывает его слово в query
    # как есть. Дешевле принять их здесь, чем объяснять модели в промпте.
    "цпу": "rate(process_cpu_seconds_total[5m])",
    "процессор": "rate(process_cpu_seconds_total[5m])",
    "загрузка": "rate(process_cpu_seconds_total[5m])",
    "память": "process_resident_memory_bytes",
    "озу": "process_resident_memory_bytes",
    "задержка": "rate(voice_llm_request_duration_seconds_sum[5m]) "
    "/ rate(voice_llm_request_duration_seconds_count[5m])",
    "ошибки": "rate(voice_llm_request_total[5m])",
    "живость": "up",
}

# Слова PromQL, которые НЕ являются именами метрик: их нельзя резолвить по
# каталогу. Список закрытый — всё, что реально встречается в наших запросах.
_PROMQL_KEYWORDS: frozenset[str] = frozenset(
    {
        "by",
        "without",
        "on",
        "ignoring",
        "group_left",
        "group_right",
        "offset",
        "bool",
        "and",
        "or",
        "unless",
        "inf",
        "nan",
        "start",
        "end",
    }
)

# Имя метрики: PromQL допускает [a-zA-Z_:][a-zA-Z0-9_:]*. Функции отсекаем
# по следующей за именем '(' — их резолвить не надо.
_METRIC_TOKEN_RE = re.compile(r"[a-zA-Z_:][a-zA-Z0-9_:]*")

#: Алиас — «просто имя метрики» (можно подставить внутрь выражения) или
#: целое выражение (тогда заменяем весь запрос, см. resolve_query).
_IS_METRIC_NAME_RE = re.compile(r"[a-zA-Z_:][a-zA-Z0-9_:]*")


class MetricsUnavailable(RuntimeError):
    """Prometheus/Loki недоступны или ответили не тем.

    Отдельный тип нужен, чтобы :mod:`tars_panel` отличал «сервис не отвечает»
    (оператору стоит сказать «мониторинг лежит») от «запрос пустой» (оператору
    стоит сказать «такой метрики нет»).
    """


# Инъекция для тестов: подменяем сетевой слой, а не urllib целиком.
HttpGet = Callable[[str], bytes]


def _default_http_get(url: str) -> bytes:
    try:
        with urllib.request.urlopen(url, timeout=_HTTP_TIMEOUT_S) as resp:  # noqa: S310
            return resp.read()
    except urllib.error.HTTPError as exc:
        # Prometheus и Loki отвечают на битый запрос кодом 400 и JSON'ом
        # ``{"status":"error","error":"parse error: …"}``. urllib превращает
        # это в исключение и тело теряется — а именно оно и объясняет
        # оператору, что не так с запросом. Возвращаем тело: разбор ошибки
        # уровня приложения — дело query_range/query_logs, а не транспорта.
        body = exc.read()
        if body:
            return body
        raise


class MetricsSource:
    """Prometheus + Loki: запрос → ряды точек, готовые к отрисовке.

    Один объект на ноду. Кэширует каталог имён метрик (TTL 60 с), чтобы
    резолв неизвестного имени не стоил лишнего round-trip'а на каждый
    tool call.
    """

    #: TTL кэша ``/api/v1/label/__name__/values``.
    CATALOG_TTL_S = 60.0

    def __init__(
        self,
        *,
        prometheus_url: str = DEFAULT_PROMETHEUS_URL,
        loki_url: str = DEFAULT_LOKI_URL,
        grafana_url: str = DEFAULT_GRAFANA_URL,
        http_get: HttpGet | None = None,
        logger: logging.Logger | None = None,
    ) -> None:
        self._prom = prometheus_url.rstrip("/")
        self._loki = loki_url.rstrip("/")
        self._grafana = grafana_url.rstrip("/")
        self._http_get = http_get or _default_http_get
        self._log = logger or _LOG
        self._catalog: tuple[str, ...] = ()
        self._catalog_at = 0.0

    # ─── HTTP ───────────────────────────────────────────────────────

    def _get_json(self, url: str) -> dict[str, Any]:
        try:
            raw = self._http_get(url)
        except (urllib.error.URLError, OSError, TimeoutError) as exc:
            raise MetricsUnavailable(f"{url.split('?', 1)[0]}: {exc}") from exc
        try:
            payload = json.loads(raw)
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            raise MetricsUnavailable(f"{url.split('?', 1)[0]}: bad JSON: {exc}") from exc
        if not isinstance(payload, dict):
            raise MetricsUnavailable(
                f"{url.split('?', 1)[0]}: expected object, got "
                f"{type(payload).__name__}"
            )
        return payload

    # ─── каталог метрик ─────────────────────────────────────────────

    def metric_names(self, *, force: bool = False) -> tuple[str, ...]:
        """Имена метрик из Prometheus (кэш ``CATALOG_TTL_S``).

        Пустой кортеж — если Prometheus недоступен: резолв в этом случае
        просто не сработает, а сам запрос всё равно уйдёт как есть.
        """
        now = time.monotonic()
        if not force and self._catalog and (now - self._catalog_at) < self.CATALOG_TTL_S:
            return self._catalog
        url = f"{self._prom}/api/v1/label/__name__/values"
        try:
            payload = self._get_json(url)
        except MetricsUnavailable as exc:
            self._log.warning(f"[metrics] catalog fetch failed: {exc}")
            return self._catalog
        names = payload.get("data")
        if not isinstance(names, list):
            return self._catalog
        self._catalog = tuple(str(n) for n in names)
        self._catalog_at = now
        return self._catalog

    def suggest_metrics(self, limit: int = 8) -> tuple[str, ...]:
        """Что показать оператору, когда его запрос не нашёлся.

        Служебные ряды экспортеров (``go_*``, ``promhttp_*``, ``scrape_*``)
        оператору не интересны — он спрашивает про робота, а не про
        внутренности Prometheus.
        """
        boring = ("go_", "promhttp_", "prometheus_", "scrape_", "net_conntrack_")
        picked = [n for n in self.metric_names() if not n.startswith(boring)]
        return tuple(picked[:limit])

    # ─── резолв запроса ─────────────────────────────────────────────

    def resolve_query(self, query: str) -> tuple[str, str]:
        """``(resolved_query, note)`` — заменить несуществующие имена метрик.

        Порядок: точное совпадение с каталогом → :data:`QUERY_ALIASES` →
        похожее имя из каталога (difflib, cutoff 0.75) → оставить как есть.
        ``note`` непустой только если что-то реально заменили — оператору
        честно проговаривается, что он смотрит не на то, что просил.
        """
        query = (query or "").strip()
        if not query:
            return "", ""

        # Голое имя метрики без выражения — самый частый случай («cpu»).
        # Если bare уже ключ QUERY_ALIASES — резолвим СРАЗУ, без похода в
        # каталог (issue #2272): and-цепочка ``in QUERY_ALIASES and not in
        # metric_names()`` платит HTTP-запрос на /api/v1/label/__name__/values
        # за каждый «холодный» (>60s) tool call, и поверх него ещё идёт
        # query_range — клиент ждёт два круга, а закладывался на один.
        # Смысл каталога здесь — защититься от теоретической коллизии
        # «алиас имеет то же имя, что и реальная метрика». QUERY_ALIASES
        # хранит именно ОВЕРРАЙДЫ (см. docstring модуля, ADR-0018), а не
        # «предложения»: оператор произносит «cpu» — ТАРС показывает CPU
        # процессов ТАРСа, а не упавший cAdvisor. Дополнительная семантика
        # «не применять алиас, если имя реально существует в Prometheus»
        # при текущем составе QUERY_ALIASES не нужна и стоит дорого.
        bare = query.strip()
        if bare.lower() in QUERY_ALIASES:
            resolved = QUERY_ALIASES[bare.lower()]
            return resolved, f"«{bare}» → {resolved}"

        known = set(self.metric_names())
        if not known:
            # Prometheus не ответил — резолвить не по чему, шлём как есть.
            return query, ""

        replacements: list[tuple[str, str]] = []
        result = query
        for token in self._metric_tokens(query):
            if token in known or token in _PROMQL_KEYWORDS:
                continue
            candidate = QUERY_ALIASES.get(token.lower())
            if candidate is None:
                close = difflib.get_close_matches(token, known, n=1, cutoff=0.75)
                candidate = close[0] if close else None
            if candidate is None:
                continue
            if not _IS_METRIC_NAME_RE.fullmatch(candidate):
                # Алиас — целое ВЫРАЖЕНИЕ, а не имя метрики. Подставить его
                # в позицию имени нельзя: «rate(network_latency_ms[5m])»
                # превратилось бы в «rate(rate(a[5m]) / rate(b[5m])[5m])» —
                # Prometheus такое отвергает с 400 (поймано на живом стенде
                # 08.09.2026). Меняем запрос целиком и честно говорим об этом.
                return candidate, f"«{query}» → {candidate}"
            # Целое слово: без границ «cpu» съел бы «cpu_total».
            result = re.sub(rf"(?<![a-zA-Z0-9_:]){re.escape(token)}(?![a-zA-Z0-9_:])",
                            candidate, result)
            replacements.append((token, candidate))

        if not replacements:
            return query, ""
        note = ", ".join(f"«{was}» → {now}" for was, now in replacements)
        return result, note

    @staticmethod
    def _metric_tokens(query: str) -> list[str]:
        """Имена метрик из PromQL: без функций, лейблов, строк и длительностей."""
        # Выкидываем содержимое {...} (селекторы лейблов), [...] (диапазоны:
        # иначе «[5m]» дал бы «m» как имя метрики) и "..."/'...' (значения) —
        # там имён метрик нет, а мусора много.
        stripped = re.sub(r"\{[^}]*\}", " ", query)
        stripped = re.sub(r"\[[^\]]*\]", " ", stripped)
        stripped = re.sub(r"""(['"]).*?\1""", " ", stripped)
        tokens: list[str] = []
        for match in _METRIC_TOKEN_RE.finditer(stripped):
            token = match.group(0)
            after = stripped[match.end() : match.end() + 1]
            if after == "(":  # функция: rate, sum, histogram_quantile…
                continue
            if token in tokens:
                continue
            tokens.append(token)
        return tokens

    # ─── Prometheus: ряды точек ─────────────────────────────────────

    def query_range(
        self,
        query: str,
        *,
        minutes: int = DEFAULT_RANGE_MINUTES,
        max_points: int = DEFAULT_MAX_POINTS,
        now: float | None = None,
    ) -> dict[str, Any]:
        """PromQL → ряды ``{name, labels, points: [[ts, value], …]}``.

        Возвращает dict, а не бросает: пустой результат — это не ошибка, а
        отдельное состояние (``status="empty"``), про которое оператору
        нужно сказать другими словами, чем про «Prometheus лежит».
        """
        end = time.time() if now is None else now
        start = end - minutes * 60
        step = max(1, int((end - start) // max(1, max_points)))
        resolved, note = self.resolve_query(query)
        if not resolved:
            return {"status": "error", "error": "empty query", "series": []}

        params = urllib.parse.urlencode(
            {
                "query": resolved,
                "start": f"{start:.3f}",
                "end": f"{end:.3f}",
                "step": str(step),
            }
        )
        payload = self._get_json(f"{self._prom}/api/v1/query_range?{params}")
        if payload.get("status") != "success":
            # Prometheus отвечает 400 с человекочитаемым error на битый
            # PromQL — отдаём его как есть, оператору это полезнее, чем
            # «что-то пошло не так».
            return {
                "status": "error",
                "error": str(payload.get("error") or "prometheus rejected the query"),
                "query": resolved,
                "note": note,
                "series": [],
            }

        raw_series = (payload.get("data") or {}).get("result") or []
        series = [
            self._series_from_matrix(item)
            for item in raw_series[:MAX_SERIES]
            if isinstance(item, dict)
        ]
        series = [s for s in series if s["points"]]
        if not series:
            return {
                "status": "empty",
                "query": resolved,
                "note": note,
                "series": [],
                "available": list(self.suggest_metrics()),
            }
        return {
            "status": "ok",
            "query": resolved,
            "note": note,
            "series": series,
            "truncated": len(raw_series) > MAX_SERIES,
            "range_minutes": minutes,
        }

    @staticmethod
    def _series_from_matrix(item: Mapping[str, Any]) -> dict[str, Any]:
        labels = {
            str(k): str(v)
            for k, v in (item.get("metric") or {}).items()
            if k != "__name__"
        }
        points: list[list[float]] = []
        for pair in item.get("values") or []:
            try:
                ts, value = pair[0], float(pair[1])
            except (TypeError, ValueError, IndexError):
                continue  # NaN/Inf/битая точка — Prometheus такое отдаёт
            if value != value:  # NaN не переживает JSON round-trip
                continue
            points.append([float(ts), value])
        return {
            "name": _series_name(item.get("metric") or {}),
            "labels": labels,
            "points": points,
        }

    # ─── Loki: строки логов ─────────────────────────────────────────

    def query_logs(
        self,
        query: str,
        *,
        minutes: int = DEFAULT_RANGE_MINUTES,
        limit: int = MAX_LOG_LINES,
        now: float | None = None,
    ) -> dict[str, Any]:
        """LogQL → последние строки ``{ts, line, labels}`` (новые сверху)."""
        query = (query or "").strip()
        if not query:
            return {"status": "error", "error": "empty query", "lines": []}
        end = time.time() if now is None else now
        start = end - minutes * 60
        params = urllib.parse.urlencode(
            {
                "query": query,
                "start": str(int(start * 1e9)),
                "end": str(int(end * 1e9)),
                "limit": str(limit),
                "direction": "backward",
            }
        )
        payload = self._get_json(f"{self._loki}/loki/api/v1/query_range?{params}")
        if payload.get("status") != "success":
            return {
                "status": "error",
                "error": str(payload.get("error") or "loki rejected the query"),
                "query": query,
                "lines": [],
            }
        lines: list[dict[str, Any]] = []
        for stream in (payload.get("data") or {}).get("result") or []:
            labels = {str(k): str(v) for k, v in (stream.get("stream") or {}).items()}
            for entry in stream.get("values") or []:
                try:
                    ts_ns, text = entry[0], str(entry[1])
                except (TypeError, IndexError):
                    continue
                try:
                    ts = int(ts_ns) / 1e9
                except (TypeError, ValueError):
                    continue
                lines.append({"ts": ts, "line": text, "labels": labels})
        lines.sort(key=lambda item: item["ts"], reverse=True)
        lines = lines[:limit]
        if not lines:
            return {"status": "empty", "query": query, "lines": []}
        return {"status": "ok", "query": query, "lines": lines}

    # ─── Grafana: ссылка «открыть на десктопе» ──────────────────────

    def explore_url(self, datasource: str, query: str) -> str:
        """Explore-URL Grafana для того же запроса.

        Explore — единственная страница Grafana, которая принимает
        произвольный запрос из URL (дашборд ``?query=`` игнорирует). В
        ``left`` кладём ИМЯ datasource, а не UID: UID генерируется при
        provisioning'е и на разных инсталляциях разный (на katana сейчас
        ``PBFA97CFB590B2093``), а имя фиксировано в
        ``docker/monitoring/config/grafana/provisioning/datasources``.

        В Quest панель эту ссылку не открывает (iframe в immersive-WebXR не
        работает) — она едет рядом с данными, чтобы оператор мог доглядеть
        подробности с ноутбука.
        """
        name = "Loki" if datasource == "loki" else "Prometheus"
        left = json.dumps(
            {
                "datasource": name,
                "queries": [{"refId": "A", "expr": query}],
                "range": {"from": "now-15m", "to": "now"},
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        return f"{self._grafana}/explore?orgId=1&left={urllib.parse.quote(left, safe='')}"


def _series_name(metric: Mapping[str, Any]) -> str:
    """Короткая подпись ряда для легенды на панели.

    Ряды одного запроса отличаются не метрикой (она общая), а инстансом.
    ``job``/``service`` в нашем Prometheus общий для пяти voice-нод
    (dialogue/tts/stt/speaker_id/audio — см. prometheus.yml), поэтому одного
    ``service`` мало: на живом стенде легенда получалась
    «voice-assistant, voice-assistant, voice-assistant». Добавляем порт
    инстанса — он и различает ноды.
    """
    label = ""
    for key in ("service", "job", "__name__"):
        value = metric.get(key)
        if value:
            label = str(value)
            break
    instance = str(metric.get("instance") or "")
    port = instance.rsplit(":", 1)[-1] if ":" in instance else ""
    if label and port:
        return f"{label}:{port}"
    return label or instance or "series"


def summarize_for_speech(result: Mapping[str, Any]) -> str:
    """Одна строка для ТАРСа: что он должен сказать оператору вслух.

    LLM получает её как результат tool call'а и повторяет. Без этого он
    придумывает «панель открыта» даже когда данных нет — а оператор потом
    смотрит на пустой экран (ADR-0018).
    """
    status = result.get("status")
    if status == "ok":
        series: Sequence[Mapping[str, Any]] = result.get("series") or ()
        head = ", ".join(
            f"{s.get('name')}: {_fmt(_last_value(s))}" for s in series[:3]
        )
        note = result.get("note")
        prefix = f"({note}) " if note else ""
        return f"{prefix}Вывел на TARS 2 — {head}" if head else "Вывел на TARS 2."
    if status == "empty":
        available = ", ".join((result.get("available") or [])[:5])
        tail = f" Есть: {available}." if available else ""
        return f"По запросу «{result.get('query')}» данных нет.{tail}"
    return f"Метрики не пришли: {result.get('error') or 'unknown error'}"


def _last_value(series: Mapping[str, Any]) -> float | None:
    points: Sequence[Sequence[float]] = series.get("points") or ()
    return float(points[-1][1]) if points else None


def _fmt(value: float | None) -> str:
    if value is None:
        return "—"
    if abs(value) >= 1e9:
        return f"{value / 1e9:.2f}G"
    if abs(value) >= 1e6:
        return f"{value / 1e6:.2f}M"
    if abs(value) >= 1e3:
        return f"{value / 1e3:.2f}k"
    if abs(value) >= 1:
        return f"{value:.2f}"
    return f"{value:.4f}".rstrip("0").rstrip(".") or "0"


__all__ = [
    "DEFAULT_GRAFANA_URL",
    "DEFAULT_LOKI_URL",
    "DEFAULT_PROMETHEUS_URL",
    "MAX_LOG_LINES",
    "MAX_SERIES",
    "MetricsSource",
    "MetricsUnavailable",
    "QUERY_ALIASES",
    "summarize_for_speech",
]
