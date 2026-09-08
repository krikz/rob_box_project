"""Юнит-тесты MetricsSource (issue #2184).

Сеть недопустима: ``http_get`` подменяется фейком, который отдаёт заранее
записанные ответы Prometheus/Loki. Формы ответов сняты с живого стенда
(katana, 08.09.2026) — ``/api/v1/query_range``, ``/api/v1/label/__name__/values``
и ``/loki/api/v1/query_range``.

Запуск::

    PYTHONPATH=. pytest -v test/unit/test_metrics_source.py
"""

from __future__ import annotations

import json
import urllib.error
from typing import Any, Callable

import pytest

from rob_box_supervisor.metrics_source import (
    MetricsSource,
    MetricsUnavailable,
    summarize_for_speech,
)

# ── фейковый HTTP ──────────────────────────────────────────────────

_CATALOG = ["up", "process_cpu_seconds_total", "process_resident_memory_bytes",
            "voice_llm_request_total"]


def _matrix(*series: tuple[dict[str, str], list[list[Any]]]) -> bytes:
    return json.dumps(
        {
            "status": "success",
            "data": {
                "resultType": "matrix",
                "result": [
                    {"metric": metric, "values": values} for metric, values in series
                ],
            },
        }
    ).encode()


def _fake_http(
    *,
    catalog: list[str] | None = None,
    range_body: bytes | None = None,
    loki_body: bytes | None = None,
) -> Callable[[str], bytes]:
    def _get(url: str) -> bytes:
        if "/label/__name__/values" in url:
            return json.dumps(
                {"status": "success", "data": _CATALOG if catalog is None else catalog}
            ).encode()
        if "/loki/api/v1/query_range" in url:
            assert loki_body is not None, f"unexpected loki call: {url}"
            return loki_body
        if "/api/v1/query_range" in url:
            assert range_body is not None, f"unexpected prom call: {url}"
            return range_body
        raise AssertionError(f"unexpected URL: {url}")

    return _get


# ── каталог и резолв запросов ──────────────────────────────────────


def test_metric_names_are_cached() -> None:
    """Каталог тянется один раз на TTL — не по запросу на каждый tool call."""
    calls: list[str] = []

    def _get(url: str) -> bytes:
        calls.append(url)
        return json.dumps({"status": "success", "data": _CATALOG}).encode()

    src = MetricsSource(http_get=_get)
    assert src.metric_names() == tuple(_CATALOG)
    assert src.metric_names() == tuple(_CATALOG)
    assert len(calls) == 1


def test_resolve_query_keeps_known_metric_untouched() -> None:
    """Валидный PromQL не трогаем — резолв только для несуществующих имён."""
    src = MetricsSource(http_get=_fake_http())
    resolved, note = src.resolve_query("rate(process_cpu_seconds_total[5m])")
    assert resolved == "rate(process_cpu_seconds_total[5m])"
    assert note == ""


def test_resolve_query_rewrites_hallucinated_metric() -> None:
    """``network_latency_ms`` из логов #2184 резолвится в живую метрику.

    Именно этот запрос LLM прислал на «покажи дрейф CPU» — метрики с таким
    именем в Prometheus нет и не было.
    """
    src = MetricsSource(http_get=_fake_http())
    resolved, note = src.resolve_query("rate(network_latency_ms[5m])")
    assert "network_latency_ms" not in resolved
    assert "voice_llm_request_duration_seconds" in resolved
    assert "network_latency_ms" in note


def test_resolve_query_never_nests_an_alias_expression() -> None:
    """Алиас-ВЫРАЖЕНИЕ заменяет весь запрос, а не подставляется внутрь.

    Регрессия, пойманная на живом Prometheus (katana, 08.09.2026): подстановка
    ``rate(a[5m]) / rate(b[5m])`` в позицию имени внутри ``rate(X[5m])`` даёт
    ``rate(rate(a[5m]) / rate(b[5m])[5m])`` — Prometheus отвечает 400.
    """
    src = MetricsSource(http_get=_fake_http())
    resolved, _ = src.resolve_query("rate(network_latency_ms[5m])")
    assert resolved.count("rate(") == 2  # два rate() рядом, а не вложенные
    assert "[5m])[5m]" not in resolved
    assert resolved.startswith("rate(voice_llm_request_duration_seconds_sum")


def test_resolve_query_ignores_duration_suffix_as_metric() -> None:
    """``[5m]`` не должен читаться как метрика ``m``."""
    assert "m" not in MetricsSource._metric_tokens("rate(up[5m])")


def test_http_error_body_is_returned_not_swallowed(monkeypatch) -> None:
    """Prometheus объясняет 400 в теле ответа — не теряем его.

    Живой стенд (katana, 08.09.2026): битый PromQL → HTTP 400 с телом
    ``{"status":"error","error":"parse error: …"}``. urllib поднимает
    HTTPError, и без обработки тела оператор получал бы «Prometheus
    недоступен» вместо разбора причины.
    """
    import io
    import urllib.request

    from rob_box_supervisor.metrics_source import _default_http_get

    body = json.dumps({"status": "error", "error": "parse error: bad"}).encode()

    def _raise(url: str, timeout: float | None = None) -> Any:
        raise urllib.error.HTTPError(url, 400, "Bad Request", {}, io.BytesIO(body))  # type: ignore[arg-type]

    monkeypatch.setattr(urllib.request, "urlopen", _raise)
    assert _default_http_get("http://prom/api/v1/query_range?x=1") == body


def test_query_range_reports_prometheus_400_as_query_error() -> None:
    """400 с телом от Prometheus → status=error с текстом, а не «сервис лёг»."""
    src = MetricsSource(
        http_get=_fake_http(
            range_body=json.dumps(
                {"status": "error", "error": "parse error: unexpected ["}
            ).encode()
        )
    )
    out = src.query_range("up[")
    assert out["status"] == "error"
    assert "parse error" in out["error"]


def test_resolve_query_bare_word() -> None:
    """Оператор сказал «cpu» — уходит рабочее выражение, а не голое слово."""
    src = MetricsSource(http_get=_fake_http())
    resolved, note = src.resolve_query("cpu")
    assert resolved == "rate(process_cpu_seconds_total[5m])"
    assert note


def test_resolve_query_russian_word() -> None:
    """«память» тоже принимается — оператор говорит по-русски."""
    src = MetricsSource(http_get=_fake_http())
    resolved, _ = src.resolve_query("память")
    assert resolved == "process_resident_memory_bytes"


def test_resolve_query_fuzzy_match_typo() -> None:
    """Опечатка в имени метрики чинится по каталогу (difflib)."""
    src = MetricsSource(http_get=_fake_http())
    resolved, note = src.resolve_query("process_resident_memory_byte")
    assert resolved == "process_resident_memory_bytes"
    assert note


def test_resolve_query_leaves_unknown_when_catalog_unavailable() -> None:
    """Prometheus не ответил → шлём запрос как есть, а не пустоту."""

    def _boom(url: str) -> bytes:
        raise urllib.error.URLError("down")

    src = MetricsSource(http_get=_boom)
    resolved, note = src.resolve_query("whatever_metric")
    assert resolved == "whatever_metric"
    assert note == ""


def test_metric_tokens_ignores_functions_and_labels() -> None:
    """Имена функций и значения лейблов не путаются с именами метрик."""
    tokens = MetricsSource._metric_tokens(
        'sum(rate(voice_llm_request_total{job="voice-assistant"}[5m])) by (job)'
    )
    assert "voice_llm_request_total" in tokens
    for name in ("sum", "rate", "voice-assistant"):
        assert name not in tokens


# ── query_range ────────────────────────────────────────────────────


def test_query_range_returns_points() -> None:
    src = MetricsSource(
        http_get=_fake_http(
            range_body=_matrix(
                (
                    {"__name__": "up", "service": "voice-assistant"},
                    [[1788893900, "1"], [1788893960, "1"]],
                )
            )
        )
    )
    out = src.query_range("up", minutes=15, now=1788893960.0)
    assert out["status"] == "ok"
    assert out["series"][0]["name"] == "voice-assistant"
    assert out["series"][0]["points"] == [[1788893900.0, 1.0], [1788893960.0, 1.0]]


def test_series_name_disambiguates_by_instance_port() -> None:
    """Пять voice-нод делят один job — легенда обязана их различать.

    На живом стенде (08.09.2026) без порта легенда читалась как
    «voice-assistant, voice-assistant, voice-assistant».
    """
    src = MetricsSource(
        http_get=_fake_http(
            range_body=_matrix(
                (
                    {"service": "voice-assistant", "instance": "10.1.1.11:9100"},
                    [[1788893900, "1"]],
                ),
                (
                    {"service": "voice-assistant", "instance": "10.1.1.11:9110"},
                    [[1788893900, "2"]],
                ),
            )
        )
    )
    out = src.query_range("up")
    assert [s["name"] for s in out["series"]] == [
        "voice-assistant:9100",
        "voice-assistant:9110",
    ]


def test_query_range_strips_name_label() -> None:
    src = MetricsSource(
        http_get=_fake_http(
            range_body=_matrix(
                ({"__name__": "up", "service": "voice-assistant"}, [[1788893900, "1"]])
            )
        )
    )
    out = src.query_range("up")
    # __name__ в лейблах не дублируем — он и так в имени ряда.
    assert "__name__" not in out["series"][0]["labels"]


def test_query_range_empty_lists_available_metrics() -> None:
    """Пустой результат — не ошибка: отдаём каталог как подсказку."""
    src = MetricsSource(
        http_get=_fake_http(
            range_body=json.dumps(
                {"status": "success", "data": {"resultType": "matrix", "result": []}}
            ).encode()
        )
    )
    out = src.query_range("up")
    assert out["status"] == "empty"
    assert "up" in out["available"]


def test_query_range_propagates_prometheus_error_text() -> None:
    """Битый PromQL: Prometheus объясняет причину — отдаём её дословно."""
    src = MetricsSource(
        http_get=_fake_http(
            range_body=json.dumps(
                {"status": "error", "error": "parse error: unexpected ]"}
            ).encode()
        )
    )
    out = src.query_range("up[")
    assert out["status"] == "error"
    assert "parse error" in out["error"]


def test_query_range_caps_series_count() -> None:
    """Широкий селектор не завалит WebSocket Quest'а десятками рядов."""
    many = _matrix(
        *[
            ({"instance": f"10.1.1.11:{9100 + i}"}, [[1788893900, str(i)]])
            for i in range(12)
        ]
    )
    src = MetricsSource(http_get=_fake_http(range_body=many))
    out = src.query_range("up")
    assert len(out["series"]) == 6
    assert out["truncated"] is True


def test_query_range_skips_nan_points() -> None:
    """NaN не переживает JSON round-trip — точку выкидываем, ряд остаётся."""
    src = MetricsSource(
        http_get=_fake_http(
            range_body=_matrix(
                ({"job": "voice"}, [[1788893900, "NaN"], [1788893960, "0.5"]])
            )
        )
    )
    out = src.query_range("up")
    assert out["series"][0]["points"] == [[1788893960.0, 0.5]]


def test_query_range_unavailable_raises() -> None:
    """Сеть легла → MetricsUnavailable, а не тихий пустой результат."""

    def _boom(url: str) -> bytes:
        if "/label/" in url:
            return json.dumps({"status": "success", "data": _CATALOG}).encode()
        raise urllib.error.URLError("connection refused")

    src = MetricsSource(http_get=_boom)
    with pytest.raises(MetricsUnavailable):
        src.query_range("up")


# ── Loki ───────────────────────────────────────────────────────────


def test_query_logs_returns_newest_first() -> None:
    body = json.dumps(
        {
            "status": "success",
            "data": {
                "resultType": "streams",
                "result": [
                    {
                        "stream": {"container": "voice-assistant"},
                        "values": [
                            ["1788893900000000000", "older"],
                            ["1788893960000000000", "newer"],
                        ],
                    }
                ],
            },
        }
    ).encode()
    src = MetricsSource(http_get=_fake_http(loki_body=body))
    out = src.query_logs('{container="voice-assistant"}')
    assert out["status"] == "ok"
    assert [line["line"] for line in out["lines"]] == ["newer", "older"]


def test_query_logs_empty_query_is_error() -> None:
    src = MetricsSource(http_get=_fake_http())
    assert src.query_logs("  ")["status"] == "error"


# ── Explore URL ────────────────────────────────────────────────────


def test_explore_url_uses_datasource_name_not_uid() -> None:
    """UID генерируется при provisioning'е и на разных стендах разный."""
    src = MetricsSource(http_get=_fake_http(), grafana_url="http://g:3000")
    url = src.explore_url("prometheus", "up")
    assert url.startswith("http://g:3000/explore?")
    assert "PBFA97CFB590B2093" not in url


# ── речь ТАРСа ─────────────────────────────────────────────────────


def test_summary_names_values() -> None:
    """ТАРС называет числа, а не факт открытия панели."""
    text = summarize_for_speech(
        {
            "status": "ok",
            "series": [{"name": "voice-assistant", "points": [[1.0, 0.0358]]}],
        }
    )
    assert "voice-assistant" in text
    assert "0.0358" in text


def test_summary_empty_mentions_alternatives() -> None:
    text = summarize_for_speech(
        {"status": "empty", "query": "network_latency_ms", "available": ["up"]}
    )
    assert "network_latency_ms" in text
    assert "up" in text


def test_summary_error_is_not_a_success_phrase() -> None:
    text = summarize_for_speech({"status": "error", "error": "connection refused"})
    assert "connection refused" in text
    assert "TARS 2" not in text
