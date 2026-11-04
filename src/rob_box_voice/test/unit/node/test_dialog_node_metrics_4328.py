"""#4328: dialog_node v2 поднимает Prometheus-экспортер и считает LLM (запросы по провайдеру/исходу, задержка,
fallback) и barge-in. Счёт ставит код вокруг вызова цепочки, а не модель."""

from __future__ import annotations

import asyncio
import socket
import sys
import types
import urllib.request

import pytest

pytest.importorskip("prometheus_client")

from rob_box_core.msg_stub import install_dialog_msgs_stub  # noqa: E402

M = install_dialog_msgs_stub()
try:  # настоящий rosidl-пакет (CI) или заглушка сервиса DialogControl (Windows), как в test_dialog_node.py
    import rob_box_dialog_msgs.srv  # noqa: F401
except ImportError:
    _srv = types.ModuleType("rob_box_dialog_msgs.srv")
    _request = type("Request", (), {"ACTION_HOLD": 0, "ACTION_RESUME": 1, "ACTION_RESET": 2})
    _srv.DialogControl = type("DialogControl", (), {"Request": _request})
    sys.modules["rob_box_dialog_msgs.srv"] = _srv
    sys.modules["rob_box_dialog_msgs"].srv = _srv

from prometheus_client import REGISTRY  # noqa: E402
from rob_box_dialog.deadline import ProviderFailed  # noqa: E402
from rob_box_llm.errors import ProviderError  # noqa: E402
from rob_box_llm.provider import LLMResponse  # noqa: E402
from rob_box_voice import dialog_node as dn  # noqa: E402
from rob_box_voice.core.dialog_metrics import llm_metrics, record_interrupt  # noqa: E402
from rob_box_voice.core.dialog_v2_ports import CallCounter, V2Ports  # noqa: E402


def _value(name: str, **labels: str) -> float:
    return REGISTRY.get_sample_value(name, labels) or 0.0


class _Chain:
    """Цепочка с health-обёрткой: primary = deepseek, ответил ``answered_by``."""

    def __init__(self, answered_by, error=None):
        self.answered_by, self.error = answered_by, error

    def snapshot(self):
        return [{"name": "deepseek"}, {"name": "minimax"}]

    async def complete(self, messages, tools=(), settings=None):
        if self.error:
            raise self.error
        return LLMResponse(content="Привет!")


def _ask(chain):
    ports = V2Ports(types.SimpleNamespace(), chain, CallCounter(), types.SimpleNamespace(),
                    on_ask=llm_metrics(chain))
    return asyncio.run(ports.ask([{"role": "user", "content": "x"}], [], 60.0))


def test_llm_request_counts_success_with_latency():
    ok = _value("voice_llm_request_total", provider="deepseek", result="success")
    seen = _value("voice_llm_request_duration_seconds_count", provider="deepseek")
    _ask(_Chain("deepseek"))
    assert _value("voice_llm_request_total", provider="deepseek", result="success") == ok + 1
    assert _value("voice_llm_request_duration_seconds_count", provider="deepseek") == seen + 1


def test_llm_answer_from_non_primary_counts_fallback_twice():
    req = _value("voice_llm_request_total", provider="minimax", result="fallback")
    fb = _value("voice_llm_fallback_total", primary="deepseek", fallback="minimax", reason="primary_failed")
    _ask(_Chain("minimax"))
    assert _value("voice_llm_request_total", provider="minimax", result="fallback") == req + 1
    assert _value("voice_llm_fallback_total", primary="deepseek", fallback="minimax",
                  reason="primary_failed") == fb + 1


def test_llm_chain_failure_counts_fail_and_still_raises():
    fail = _value("voice_llm_request_total", provider="minimax", result="fail")
    with pytest.raises(ProviderFailed):
        _ask(_Chain("minimax", ProviderError("chain exhausted")))
    assert _value("voice_llm_request_total", provider="minimax", result="fail") == fail + 1


def test_interrupt_counts_barge_in():
    before = _value("voice_barge_in_total")
    record_interrupt()
    assert _value("voice_barge_in_total") == before + 1


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class _Node:
    def __init__(self, port):
        self.port, self.lines = port, []

    def get_parameter(self, name):
        assert name == "metrics_port"
        return types.SimpleNamespace(value=self.port)

    def get_logger(self):
        return types.SimpleNamespace(info=self.lines.append, warning=self.lines.append)


def test_exporter_starts_on_the_port_from_the_parameter():
    port = _free_port()
    assert dn.start_metrics(_Node(port)) is True
    with urllib.request.urlopen(f"http://127.0.0.1:{port}/metrics", timeout=5) as resp:
        body = resp.read().decode()
    assert "voice_llm_request_total" in body


def test_exporter_off_when_port_is_zero():
    node = _Node(0)
    assert dn.start_metrics(node) is False and node.lines == []


def test_declared_default_port_is_9100():
    declared = {}
    node = types.SimpleNamespace(declare_parameter=lambda name, value=None: declared.__setitem__(name, value))
    dn.declare_params(node)
    assert declared["metrics_port"] == 9100
