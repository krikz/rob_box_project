"""07.10 ~15:12 UTC: «ищи внимательнее танец утят» → ``search_web('"танец маленьких утят" ноты RTTTL OR melody')`` 0
результатов и ``search_melody('танец утят')`` промах → робот: «Танца утят и в нотах, и в сети нет». В архиве есть
«Chicken Dance». Промах — по одной строке: текст результата строит код и называет проверенную строку, а не «нет»."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from ._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.tools.music import SearchMelodyTool
    from rob_box_mcp_tools.tools.web_search import SearchWebTool
_ros_stubs = _ros.fixture()

pytestmark = pytest.mark.unit


def test_melody_miss_names_the_string_and_asks_for_the_original_title():
    tool = SearchMelodyTool(None, SimpleNamespace(search=lambda q, limit: []))
    result = tool.execute("танец утят")
    assert not result.success
    assert "'танец утят'" in result.error and "проверена только эта строка" in result.error
    assert "английское название" in result.error


def test_empty_web_search_is_about_the_string(monkeypatch):
    """Пустой ответ DeepSeek (после пустого ddgs, #4026) - 0 результатов про эту строку; сеть и ключ подменены."""
    seen = []

    def transport(method, url, headers, body, timeout):
        seen.append(url)
        return 200, b'{"results": []}'

    tool = SearchWebTool(None, transport=transport, environ={"DEEPSEEK_API_KEY": "sk-test"})
    tool._ddgs_available, tool._ddgs_cls = True, object
    monkeypatch.setattr(tool, "_ddgs_text", lambda *_a: [])
    result = tool.execute('"танец маленьких утят" ноты')
    assert result.success and result.data["results"] == [] and len(seen) == 1
    assert "вернул 0 результатов" in result.message and "танец маленьких утят" in result.message
    assert "не про то, что такого нет в сети" in result.message
