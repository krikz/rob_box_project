"""#4297: ТАРС на «как сеть» (живой 03.11, лог 4295) - тулов сети не было в срезе, 3x ``tool_not_available``, 7 вызовов LLM.

Таблица слов - ``knowledge.TOOL_GROUPS["network"]``: сеть/связь/интернет/Wi-Fi открывают тулы, что реально отдают данные
о сети (``get_perception_context`` - ``internet_available``; ``show_metrics`` - ``services_up`` и задержки). Данных о
Wi-Fi нет ни у одного тула - это решает ``state_claim`` (вырезает утверждение), а не срез.
"""

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import available_tools

NETWORK_TOOLS = {"get_perception_context", "show_metrics"}


def _names(text):
    return {t.name for t in available_tools(Session("operator").snapshot, None, "operator", text)}


@pytest.mark.parametrize("text", [
    "как сеть", "как сеть, есть связь", "есть интернет?", "что с интернетом", "как там вайфай", "Wi-Fi ловит?",
    "роутер жив?", "проверь связь", "сетевое соединение в порядке?",
])
def test_network_words_open_network_tools(text):
    assert NETWORK_TOOLS <= _names(text), text


@pytest.mark.parametrize("text", ["статус контейнеров", "статус нод", "покажи логи", "покажи CPU"])
def test_other_phrases_do_not_open_network_or_music(text):
    assert "dj_set" not in _names(text), text


def test_containers_slice_stays_narrow():
    names = _names("статус контейнеров")
    assert "container_status" in names and "show_metrics" not in names


def test_robot_health_opens_status_tools():
    assert {"get_robot_status", "get_perception_context"} <= _names("как здоровье робота")


def test_slice_limit_kept():
    names = _names("как сеть, статус контейнеров, покажи логи и CPU, статус нод, батарея")
    assert len(names) <= kn.TOOLS_VISIBLE_MAX


def test_network_table_is_shared_with_state_claims():
    assert kn.STATE_CLAIMS["network"].subjects == kn.NETWORK_STEMS
    assert kn.STATE_CLAIMS["wifi"].subjects == kn.WIFI_STEMS
    assert set(kn.WIFI_STEMS + kn.INTERNET_STEMS + kn.NETWORK_STEMS) == set(kn.TOOL_GROUPS["network"].stems)


@pytest.mark.parametrize("text", ["как сеть", "есть интернет?", "сетевое соединение в порядке?"])
def test_network_words_are_not_music(text):
    """Корень живого сбоя: музыкальная основа «сет» цепляла «сеть», музыка вытесняла тулы сети из среза (12)."""
    assert "dj_set" not in _names(text), text


@pytest.mark.parametrize("text", ["включи сет", "запусти диджей-сет", "поставь сет на ренардо"])
def test_dj_set_is_still_music(text):
    assert "dj_set" in _names(text), text
