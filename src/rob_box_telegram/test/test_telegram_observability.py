"""test_telegram_observability.py — Unit tests for telegram-bot metrics (issue #1160).

Контракт: лёгкий автономный модуль ``rob_box_telegram.observability`` —
no-op без ``prometheus_client``, idempotent server start, counter
``telegram_message_total``.

Pure Python — no ROS, no rclpy, no python-telegram-bot.
"""

from __future__ import annotations

import pytest

from rob_box_telegram.observability import (
    is_metrics_enabled,
    record_telegram_message,
    start_metrics_server,
)


class TestNoopBehaviour:
    def test_start_server_disabled_returns_false(self):
        if is_metrics_enabled():
            pytest.skip("prometheus_client installed — no-op path not exercised")
        assert start_metrics_server(19998) is False

    def test_record_noop(self):
        if is_metrics_enabled():
            pytest.skip("prometheus_client installed — no-op path not exercised")
        record_telegram_message("in", message_type="text")
        record_telegram_message("out", message_type="voice")


@pytest.mark.skipif(
    not is_metrics_enabled(),
    reason="prometheus_client not installed — metric tests skipped",
)
class TestMetricsWithPrometheusClient:
    def test_record_increments(self):
        from prometheus_client import REGISTRY

        def value(direction: str, mtype: str) -> int:
            for metric in REGISTRY.collect():
                for sample in metric.samples:
                    if (
                        sample.name == "telegram_message_total"
                        and sample.labels.get("direction") == direction
                        and sample.labels.get("type") == mtype
                    ):
                        return int(sample.value)
            return 0

        b_in = value("in", "voice")
        record_telegram_message("in", message_type="voice")
        a_in = value("in", "voice")
        assert a_in == b_in + 1
