"""
test_web_search.py - Unit тесты для инструмента поиска в интернете

Тестирует:
- SearchWebTool: lazy import ddgs, пустой query, ошибки сети, сниппеты

Покрытие: success + error + invalid params.
DDGS мокается — тесты не ходят в интернет.
"""

import sys
from unittest.mock import MagicMock, Mock, patch

import pytest

# Mock ROS 2 модулей перед импортом tools
for _mod in [
    "rclpy",
    "rclpy.node",
    "rclpy.action",
    "rclpy.qos",
]:
    sys.modules.setdefault(_mod, MagicMock())

from rob_box_mcp_tools.tools.web_search import SearchWebTool  # noqa: E402


class _FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, msg):
        self.messages.append(("info", msg))

    def warning(self, msg):
        self.messages.append(("warning", msg))

    def error(self, msg):
        self.messages.append(("error", msg))

    def debug(self, msg):
        self.messages.append(("debug", msg))


class _FakeNode:
    def __init__(self):
        self._logger = _FakeLogger()

    def get_logger(self):
        return self._logger


class _FakeDDGS:
    """Контекстный менеджер, который возвращает результаты из .text()."""

    def __init__(self, results=None, exc=None):
        self._results = results or []
        self._exc = exc

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def text(self, query, max_results=5, region="wt-wt"):
        if self._exc is not None:
            raise self._exc
        return self._results


@pytest.mark.unit
class TestSearchWebTool:
    def test_tool_metadata(self):
        tool = SearchWebTool(_FakeNode())
        assert tool.name == "search_web"
        assert tool.execution_type.value == "fast"
        assert tool.destructive is False
        assert [p.name for p in tool.parameters] == ["query", "max_results"]

    def test_execute_ddgs_not_installed(self):
        tool = SearchWebTool(_FakeNode())
        tool._ddgs_available = False

        with patch("builtins.__import__", side_effect=ImportError("no ddgs")):
            result = tool.execute(query="погода")

        assert result.success is False
        assert "not installed" in result.error
        assert "pip install duckduckgo-search" in result.message

    def test_execute_empty_query(self):
        tool = SearchWebTool(_FakeNode())
        tool._ddgs_available = True
        tool._ddgs_cls = _FakeDDGS

        result = tool.execute(query="   ")

        assert result.success is False
        assert "empty_query" in result.error
        assert "Пустой поисковый запрос" in result.message

    def test_execute_success_returns_snippets(self):
        tool = SearchWebTool(_FakeNode())
        tool._ddgs_available = True
        tool._ddgs_cls = _FakeDDGS

        results = [
            {"title": "Погода в Батайске", "body": "Сегодня солнечно, +25", "href": "https://example.com/weather"},
        ]

        with patch.object(_FakeDDGS, "text", return_value=results):
            result = tool.execute(query="погода в батайске", max_results=3)

        assert result.success is True
        assert result.data["query"] == "погода в батайске"
        assert len(result.data["results"]) == 1
        assert result.data["results"][0]["title"] == "Погода в Батайске"
        assert result.data["results"][0]["body"] == "Сегодня солнечно, +25"
        assert result.data["results"][0]["url"] == "https://example.com/weather"

    def test_execute_skips_empty_bodies(self):
        tool = SearchWebTool(_FakeNode())
        tool._ddgs_available = True
        tool._ddgs_cls = _FakeDDGS

        results = [
            {"title": "Ok", "body": "", "href": "https://x.com"},
            {"title": "Good", "body": "Real content", "href": "https://y.com"},
        ]

        with patch.object(_FakeDDGS, "text", return_value=results):
            result = tool.execute(query="тест")

        assert result.success is True
        assert len(result.data["results"]) == 1
        assert result.data["results"][0]["body"] == "Real content"

    def test_execute_truncates_long_body(self):
        tool = SearchWebTool(_FakeNode())
        tool._ddgs_available = True
        tool._ddgs_cls = _FakeDDGS

        long_body = "x" * 1000
        results = [{"title": "Long", "body": long_body, "href": "https://z.com"}]

        with patch.object(_FakeDDGS, "text", return_value=results):
            result = tool.execute(query="длинный")

        assert result.success is True
        assert len(result.data["results"][0]["body"]) <= 280 + 3  # + "..."
        assert result.data["results"][0]["body"].endswith("...")

    def test_execute_no_results(self):
        tool = SearchWebTool(_FakeNode())
        tool._ddgs_available = True
        tool._ddgs_cls = _FakeDDGS

        with patch.object(_FakeDDGS, "text", return_value=[]):
            result = tool.execute(query="ничего не найдём")

        assert result.success is True
        assert result.data["results"] == []
        assert "Ничего не найдено" in result.message

    def test_execute_ddgs_error_returns_graceful(self):
        tool = SearchWebTool(_FakeNode())
        tool._ddgs_available = True
        tool._ddgs_cls = _FakeDDGS

        with patch.object(_FakeDDGS, "text", side_effect=RuntimeError("timeout")):
            result = tool.execute(query="погода")

        assert result.success is False
        assert "timeout" in result.error
        assert "DuckDuckGo недоступен" in result.message

    def test_max_results_clamped(self):
        tool = SearchWebTool(_FakeNode())
        tool._ddgs_available = True
        tool._ddgs_cls = _FakeDDGS

        with patch.object(_FakeDDGS, "text", return_value=[]) as mock_text:
            tool.execute(query="тест", max_results=100)
            assert mock_text.call_args.kwargs["max_results"] == 10

    def test_lazy_import_success(self):
        tool = SearchWebTool(_FakeNode())
        assert tool._ddgs_available is False

        fake_mod = MagicMock()
        fake_mod.DDGS = _FakeDDGS

        with patch("builtins.__import__", return_value=fake_mod):
            result = tool.execute(query="тест")

        assert tool._ddgs_available is True
        assert tool._ddgs_cls is _FakeDDGS
        assert result.success is True
