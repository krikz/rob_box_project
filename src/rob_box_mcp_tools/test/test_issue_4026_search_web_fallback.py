"""#4026 (n907/n908 23.10): сбой поиска не выглядит для LLM как «0 результатов».

Марафон 23.10 23:03-23:04: ``search_web`` трижды ответил ``→ 0 results (DDGSException)`` при «✅ выполнен успешно»,
робот сказал «поиск ничего не выдал», 4 вызова LLM впустую. Теперь: DDGS → при сбое бэкенда DeepSeek
``/v1/tools/search`` → оба не ответили - ``success=False``, ``reason=search_unavailable``. Пустая выдача DDGS
(«No results found») - честное «ничего не нашёл» без второго бэкенда. HTTP подменён транспортом, DDGS - классом.
"""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from types import SimpleNamespace

import pytest

from ._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.tools import web_search
    from rob_box_mcp_tools.tools.web_search import SearchWebTool
_ros_stubs = _ros.fixture()

pytestmark = pytest.mark.unit

KEY = "sk-test-secret-key-4026"


class DDGSException(Exception):
    """Как ``ddgs.exceptions.DDGSException``: тул смотрит на текст, а не на класс."""


class _Log:
    def __init__(self):
        self.lines = []

    def _add(self, level, message):
        self.lines.append((level, message))

    def info(self, m):
        self._add("info", m)

    def warning(self, m):
        self._add("warning", m)

    def error(self, m):
        self._add("error", m)

    def debug(self, m):
        self._add("debug", m)

    def text(self):
        return "\n".join(f"{level}: {m}" for level, m in self.lines)


def _ddgs(rows=None, error=None):
    """Класс-замена DDGS: ``text()`` отдаёт ``rows`` или бросает ``error``."""

    class FakeDDGS:
        def __init__(self, timeout=10):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def text(self, query, max_results=5, region="wt-wt", backend="auto"):
            if error is not None:
                raise error
            return list(rows or [])

    return FakeDDGS


class _Transport:
    """Подменённый HTTP: ``reply`` - ``(status, dict | bytes)`` или исключение; все вызовы пишутся в ``calls``."""

    def __init__(self, reply=None):
        self.reply = reply
        self.calls = []

    def __call__(self, method, url, headers, body, timeout):
        self.calls.append(SimpleNamespace(method=method, url=url, headers=dict(headers),
                                          body=json.loads(body) if body else None, timeout=timeout))
        if isinstance(self.reply, Exception):
            raise self.reply
        status, payload = self.reply
        return status, payload if isinstance(payload, bytes) else json.dumps(payload).encode("utf-8")


def _deepseek_ok(*rows):
    return 200, {"results": list(rows)}


WEATHER = {
    "title": "Погода в Москве завтра — Яндекс Погода",
    "url": "https://yandex.ru/pogoda/moscow",
    "snippet": "Короткий сниппет.",
    "summary": "Завтра в Москве +5, облачно, к вечеру небольшой дождь.",
    "rerank_score": 1.73,
    "date": "2026-10-27",
}


def _tool(ddgs=None, transport=None, environ=None):
    log = _Log()
    node = SimpleNamespace(get_logger=lambda: log)
    transport = transport or _Transport(_deepseek_ok(WEATHER))
    tool = SearchWebTool(node, transport=transport, environ={"DEEPSEEK_API_KEY": KEY} if environ is None else environ)
    if ddgs is not None:
        tool._ddgs_available, tool._ddgs_cls = True, ddgs
    return SimpleNamespace(tool=tool, log=log, transport=transport)


DDG_ROW = {"title": "Погода", "body": "Завтра +6, дождь.", "href": "https://example.org/w"}


def test_ddgs_answers_and_deepseek_is_not_called():
    s = _tool(ddgs=_ddgs([DDG_ROW]))
    result = s.tool.execute("погода завтра в Москве")
    assert result.success and result.data["backend"] == "ddgs"
    assert result.data["results"] == [{"title": "Погода", "body": "Завтра +6, дождь.", "url": "https://example.org/w"}]
    assert s.transport.calls == []
    assert "backend=ddgs" in s.log.text() and "→ 1 results" in s.log.text()


def test_ddgs_backend_failure_goes_to_deepseek_with_the_summary():
    s = _tool(ddgs=_ddgs(error=DDGSException("Ratelimit 202")))
    result = s.tool.execute("погода завтра в Москве", max_results=3)
    assert result.success and result.data["backend"] == "deepseek"
    assert result.data["results"] == [{
        "title": WEATHER["title"], "url": WEATHER["url"], "date": "2026-10-27",
        "body": "Завтра в Москве +5, облачно, к вечеру небольшой дождь.",  # summary, а не snippet
    }]
    call = s.transport.calls[0]
    assert (call.method, call.url) == ("POST", "https://api.deepseek.com/v1/tools/search")
    assert call.headers["Authorization"] == f"Bearer {KEY}" and call.headers["Content-Type"] == "application/json"
    assert call.body == {"query": "погода завтра в Москве", "count": 3}
    assert json.loads(result.message)["backend"] == "deepseek"
    log = s.log.text()
    assert "backend=ddgs недоступен: DDGSException: Ratelimit 202" in log and "backend=deepseek" in log


def test_deepseek_falls_back_to_snippet_when_summary_is_empty_and_clips_long_text():
    long_text = "слово " * 300
    s = _tool(ddgs=_ddgs(error=DDGSException("boom")), transport=_Transport(_deepseek_ok(
        {"title": "A", "url": "u1", "snippet": "только сниппет", "summary": ""},
        {"title": "B", "url": "u2", "summary": long_text},
        {"title": "пусто", "url": "u3", "summary": "", "snippet": ""},  # без текста строки нет
    )))
    rows = s.tool.execute("q").data["results"]
    assert [r["body"] for r in rows][0] == "только сниппет"
    assert len(rows) == 2 and rows[1]["body"].endswith("...") and len(rows[1]["body"]) <= s.tool._SUMMARY_CHARS + 3


def test_both_backends_down_is_an_honest_refusal_not_zero_results():
    s = _tool(ddgs=_ddgs(error=DDGSException("Ratelimit 202")), transport=_Transport((500, {"error": "x"})))
    result = s.tool.execute("рецепт борща")
    assert not result.success
    assert result.data["status"] == "rejected" and result.data["reason"] == "search_unavailable"
    assert result.data["results"] == [] and result.data["tried"] == ["ddgs", "deepseek"]
    assert result.error.startswith("search_unavailable:")
    assert "ddgs: DDGSException: Ratelimit 202" in result.error and "deepseek: HTTP 500" in result.error
    assert "сбой связи, а не «ничего не найдено»" in result.message
    assert "search_unavailable" in s.log.text() and "error: " in s.log.text()


@pytest.mark.parametrize("reply,why", [
    ((402, {"error": "Insufficient Balance"}), "HTTP 402 (на балансе пусто)"),
    ((401, {"error": "bad key"}), "HTTP 401 (ключ отвергнут)"),
    ((429, {}), "HTTP 429 (лимит запросов)"),
    ((200, b"<html>not json</html>"), "ответ не разобрать"),
    ((200, {"data": []}), "в ответе нет results[]"),
    (TimeoutError("timed out"), "TimeoutError: timed out"),
])
def test_deepseek_failures_are_named(reply, why):
    s = _tool(ddgs=_ddgs(error=DDGSException("down")), transport=_Transport(reply))
    result = s.tool.execute("q")
    assert not result.success and result.data["reason"] == "search_unavailable"
    assert why in result.error


def test_empty_ddgs_is_a_real_miss_and_deepseek_is_not_asked():
    """«No results found» - не сбой бэкенда: честное «ничего не нашёл по этой строке», второй бэкенд не зовём."""
    s = _tool(ddgs=_ddgs(error=DDGSException("No results found.")))
    result = s.tool.execute('"танец маленьких утят" ноты')
    assert result.success and result.data["results"] == [] and result.data["backend"] == "ddgs"
    assert "вернул 0 результатов" in result.message and "танец маленьких утят" in result.message
    assert "не про то, что такого нет в сети" in result.message
    assert s.transport.calls == []


def test_ddgs_rows_without_text_count_as_empty():
    s = _tool(ddgs=_ddgs([{"title": "t", "body": "", "href": "h"}]))
    result = s.tool.execute("q")
    assert result.success and result.data["results"] == [] and s.transport.calls == []


def test_deepseek_empty_answer_after_a_ddgs_failure_is_a_real_miss():
    s = _tool(ddgs=_ddgs(error=DDGSException("down")), transport=_Transport(_deepseek_ok()))
    result = s.tool.execute("q")
    assert result.success and result.data["results"] == [] and result.data["backend"] == "deepseek"
    assert "вернул 0 результатов" in result.message


def test_no_key_skips_the_stage_with_a_log_line():
    s = _tool(ddgs=_ddgs(error=DDGSException("down")), environ={})
    result = s.tool.execute("q")
    assert not result.success and result.data["reason"] == "search_unavailable"
    assert s.transport.calls == []
    assert "нет ключа DEEPSEEK_API_KEY — ступень пропущена" in s.log.text()


def test_blank_key_counts_as_no_key():
    s = _tool(ddgs=_ddgs(error=DDGSException("down")), environ={"DEEPSEEK_API_KEY": "   "})
    assert not s.tool.execute("q").success and s.transport.calls == []


def test_ddgs_not_installed_goes_to_deepseek(monkeypatch):
    s = _tool()  # ddgs не подставлен: оба импорта ниже падают
    monkeypatch.setattr(SearchWebTool, "_ddgs_class", lambda self: None)
    result = s.tool.execute("q")
    assert result.success and result.data["backend"] == "deepseek"
    assert "пакет ddgs не установлен" in s.log.text()


def test_query_is_clipped_to_the_api_limit_and_count_is_clamped():
    s = _tool(ddgs=_ddgs(error=DDGSException("down")))
    s.tool.execute("я" * 300, max_results=99)
    body = s.transport.calls[0].body
    assert len(body["query"]) == web_search.DEEPSEEK_QUERY_MAX == 128 and body["count"] == 10


def test_deepseek_gets_a_deadline_inside_the_query_class_budget():
    s = _tool(ddgs=_ddgs(error=DDGSException("down")))
    s.tool.execute("q")
    assert s.transport.calls[0].timeout == web_search.DEEPSEEK_TIMEOUT_S <= 4.0


def test_the_key_never_reaches_results_errors_or_logs():
    s = _tool(ddgs=_ddgs(error=DDGSException("down")), transport=_Transport((401, {"error": f"bad {KEY[:3]}"})))
    result = s.tool.execute("q")
    everything = json.dumps(result.to_dict(), ensure_ascii=False) + s.log.text()
    assert KEY not in everything


def test_empty_query_is_refused_without_any_backend():
    s = _tool(ddgs=_ddgs([DDG_ROW]))
    result = s.tool.execute("   ")
    assert not result.success and result.error == "empty_query" and s.transport.calls == []


def test_refusal_travels_as_a_rejected_status_for_the_dialog():
    """Диалог читает ``data.status`` и ``data.reason`` результата (``command_turn.owner_event``): отказ - не ``done``."""
    s = _tool(ddgs=_ddgs(error=DDGSException("down")), environ={})
    data = s.tool.execute("q").data
    assert data["status"] == "rejected" and isinstance(data["reason"], str)


def test_default_transport_is_the_one_kie_uses():
    """Одна реализация HTTP: боевой транспорт поиска - ``urllib_transport`` клиента KIE, не вторая копия."""
    from rob_box_mcp_tools.engine.gen.kie_suno import urllib_transport

    assert SearchWebTool(None)._http() is urllib_transport


def test_real_http_path_against_a_local_server(monkeypatch):
    """Боевой транспорт без подмены: запрос уходит по HTTP с ключом и телом ``{query, count}``, ответ разбирается."""
    seen = {}

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            seen["path"] = self.path
            seen["auth"] = self.headers.get("Authorization")
            seen["type"] = self.headers.get("Content-Type")
            seen["body"] = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            payload = json.dumps({"results": [WEATHER]}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, *_args):
            pass

    server = HTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        monkeypatch.setattr(web_search, "DEEPSEEK_SEARCH_URL", f"http://127.0.0.1:{server.server_port}/v1/tools/search")
        log = _Log()
        tool = SearchWebTool(SimpleNamespace(get_logger=lambda: log), environ={"DEEPSEEK_API_KEY": KEY})
        tool._ddgs_available, tool._ddgs_cls = True, _ddgs(error=DDGSException("down"))
        result = tool.execute("погода завтра в Москве", max_results=2)
    finally:
        server.shutdown()
        server.server_close()
    assert result.success and result.data["backend"] == "deepseek"
    assert result.data["results"][0]["body"].startswith("Завтра в Москве +5")
    assert seen == {"path": "/v1/tools/search", "auth": f"Bearer {KEY}", "type": "application/json",
                    "body": {"query": "погода завтра в Москве", "count": 2}}


def test_unreachable_server_is_search_unavailable_not_a_crash(monkeypatch):
    monkeypatch.setattr(web_search, "DEEPSEEK_SEARCH_URL", "http://127.0.0.1:9/v1/tools/search")  # порт discard: закрыт
    log = _Log()
    tool = SearchWebTool(SimpleNamespace(get_logger=lambda: log), environ={"DEEPSEEK_API_KEY": KEY})
    tool._ddgs_available, tool._ddgs_cls = True, _ddgs(error=DDGSException("down"))
    result = tool.execute("q")
    assert not result.success and result.data["reason"] == "search_unavailable"
    assert "deepseek:" in result.error and KEY not in result.error + log.text()
