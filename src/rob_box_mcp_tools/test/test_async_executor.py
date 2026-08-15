"""
test_async_executor.py - Unit tests for AsyncToolExecutor.

Покрывает:
- Event-based ожидание результата (вместо busy-wait polling с sleep(0.05))
- Пробуждение из ROS callback thread через call_soon_threadsafe
- Timeout и очистку _result_events
- Fire-and-forget для INSTANT
- LONG задачи и прерывания
- Параллельное выполнение через gather()

Никаких внешних зависимостей: ROS 2, LLM API и железо не нужны.
"""

import asyncio
import sys
import threading
import time
import types

import pytest

from rob_box_mcp_tools.async_executor import AsyncToolExecutor
from rob_box_mcp_tools.base import ToolExecutionType


# ============================================================
# ROS2-заглушка для std_msgs.msg.String
#
# execute_tool_async делает `from std_msgs.msg import String` в рантайме.
# В unit-тестах ROS2 не установлен (см. TEST_GUIDE.md: «Unit tests use
# mocks and should NOT import real ROS 2»), поэтому подменяем модуль.
# ============================================================
_std_msgs = types.ModuleType("std_msgs")
_std_msgs_msg = types.ModuleType("std_msgs.msg")


class _String:
    def __init__(self, data=""):
        self.data = data


_std_msgs_msg.String = _String
_std_msgs.msg = _std_msgs_msg
sys.modules.setdefault("std_msgs", _std_msgs)
sys.modules.setdefault("std_msgs.msg", _std_msgs_msg)


class MockLogger:
    """Минимальный мок ROS-логгера."""

    def __init__(self):
        self.messages = []

    def info(self, msg):
        self.messages.append(("info", msg))

    def warning(self, msg):
        self.messages.append(("warning", msg))

    def error(self, msg):
        self.messages.append(("error", msg))


class MockPublisher:
    """Минимальный мок ROS-паблишера."""

    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


def _make_executor(result_callback=None):
    """Создать executor с моками."""
    pub = MockPublisher()
    logger = MockLogger()
    executor = AsyncToolExecutor(
        execute_pub=pub,
        result_callback=result_callback,
        logger=logger,
    )
    return executor, pub, logger


def _result_payload(request_id, **overrides):
    """Сформировать payload результата в формате on_result / results_cache."""
    data = {"success": True, "request_id": request_id}
    data.update(overrides)
    return {"request_id": request_id, "result": data}


# ============================================================
# _wait_for_result — event-based ожидание
# ============================================================


class TestWaitForResultEventBased:
    @pytest.mark.asyncio
    async def test_result_already_in_cache_returns_immediately(self):
        """Результат уже в кэше — возвращаем сразу, без регистрации event."""
        executor, _, _ = _make_executor()
        executor.results_cache["r1"] = _result_payload("r1", value=42)

        result = await executor._wait_for_result("r1", 1.0)

        assert result["success"] is True
        assert result["value"] == 42
        assert "r1" not in executor.results_cache
        assert "r1" not in executor._result_events

    @pytest.mark.asyncio
    async def test_result_received_while_waiting_wakes_waiter(self):
        """Результат приходит во время ожидания — waiter просыпается по event."""
        executor, _, _ = _make_executor()

        waiter = asyncio.create_task(executor._wait_for_result("r1", 5.0))
        # Даём корутине зарегистрировать event
        await asyncio.sleep(0)
        assert "r1" in executor._result_events

        # Симулируем приход результата (как это делает on_result в ROS thread)
        executor.on_result_received("r1", _result_payload("r1", value=7))

        result = await waiter
        assert result["success"] is True
        assert result["value"] == 7
        # Event очищен после завершения ожидания
        assert "r1" not in executor._result_events

    @pytest.mark.asyncio
    async def test_timeout_raises_and_cleans_event(self):
        """Нет результата — TimeoutError, event очищен."""
        executor, _, _ = _make_executor()

        with pytest.raises(asyncio.TimeoutError):
            await executor._wait_for_result("r1", 0.05)

        assert "r1" not in executor._result_events

    @pytest.mark.asyncio
    async def test_wake_from_ros_callback_thread(self):
        """
        Результат приходит из отдельного потока (ROS callback thread).

        on_result_received вызывается НЕ из event loop — будим waiter через
        call_soon_threadsafe. Функционально проверяем, что результат получен
        и event очищен; тайминг — только sanity (не завис).
        """
        executor, _, _ = _make_executor()
        loop = asyncio.get_running_loop()

        waiter = asyncio.create_task(executor._wait_for_result("r1", 5.0))
        await asyncio.sleep(0)
        assert "r1" in executor._result_events

        def _deliver():
            time.sleep(0.02)
            executor.on_result_received("r1", _result_payload("r1", value="from-thread"))

        t = threading.Thread(target=_deliver)
        start = time.monotonic()
        t.start()
        result = await waiter
        t.join()
        elapsed = time.monotonic() - start

        assert result["success"] is True
        assert result["value"] == "from-thread"
        assert "r1" not in executor._result_events
        # Sanity: не должно висеть минутами (старый polling вернул бы ~50ms+)
        assert elapsed < 1.0

    @pytest.mark.asyncio
    async def test_multiple_waiters_isolated(self):
        """Два разных request_id не мешают друг другу."""
        executor, _, _ = _make_executor()

        w1 = asyncio.create_task(executor._wait_for_result("a", 5.0))
        w2 = asyncio.create_task(executor._wait_for_result("b", 5.0))
        await asyncio.sleep(0)

        # Приходит результат только для "b" — w2 завершается, w1 продолжает ждать
        executor.on_result_received("b", _result_payload("b", value="bee"))
        r2 = await w2

        assert r2["value"] == "bee"
        assert "b" not in executor._result_events
        assert "a" in executor._result_events

        # Теперь приходит результат для "a"
        executor.on_result_received("a", _result_payload("a", value="aye"))
        r1 = await w1

        assert r1["value"] == "aye"
        assert "a" not in executor._result_events

    @pytest.mark.asyncio
    async def test_result_arrives_before_event_registration(self):
        """
        Результат попадает в кэш ДО вызова _wait_for_result — покрывает
        первый быстрый путь. Второй быстрый путь (между проверкой и
        регистрацией) эмулируем прямой вставкой в кэш перед ожиданием.
        """
        executor, _, _ = _make_executor()
        # Вставляем результат ДО регистрации waiter'а — как если бы
        # on_result_received отработал раньше _wait_for_result.
        executor.on_result_received("r1", _result_payload("r1", value=1))

        result = await executor._wait_for_result("r1", 1.0)

        assert result["value"] == 1
        assert "r1" not in executor.results_cache
        assert "r1" not in executor._result_events


# ============================================================
# execute_tool_async
# ============================================================


class TestExecuteToolAsync:
    @pytest.mark.asyncio
    async def test_instant_fire_and_forget(self):
        """INSTANT — публикуем и не ждём результата."""
        executor, pub, _ = _make_executor()

        result = await executor.execute_tool_async(
            "play_animation", {"animation": "happy"}, ToolExecutionType.INSTANT
        )

        assert result["success"] is True
        assert "fire-and-forget" in result["message"]
        assert len(pub.published) == 1

    @pytest.mark.asyncio
    async def test_fast_success(self):
        """FAST — результат приходит, возвращаем его."""
        executor, pub, _ = _make_executor()

        async def _deliver():
            await asyncio.sleep(0.01)
            # request_id берём из опубликованного сообщения
            import json
            published = json.loads(pub.published[0].data)
            executor.on_result_received(
                published["request_id"], _result_payload(published["request_id"], ok=True)
            )

        asyncio.create_task(_deliver())
        result = await executor.execute_tool_async(
            "play_sound", {"sound": "ding"}, ToolExecutionType.FAST
        )

        assert result["success"] is True
        assert result["ok"] is True

    @pytest.mark.asyncio
    async def test_fast_timeout_returns_error_dict(self):
        """FAST — timeout: возвращаем error dict, не роняем исключение."""
        executor, _, _ = _make_executor()

        result = await executor.execute_tool_async(
            "play_sound", {"sound": "nope"}, ToolExecutionType.FAST, request_id="t1"
        )

        assert result["success"] is False
        assert "Timeout" in result["error"]
        assert "t1" not in executor._result_events

    @pytest.mark.asyncio
    async def test_long_background_and_interrupt(self):
        """LONG — запуск в фоне, прерывание через interrupt_all_long_tasks."""
        executor, pub, _ = _make_executor()

        result = await executor.execute_tool_async(
            "navigate_to_waypoint", {"waypoint": "kitchen"}, ToolExecutionType.LONG,
            request_id="long1",
        )

        assert result["success"] is True
        assert result["background"] is True
        assert "long1" in executor.long_tasks

        cancelled = await executor.interrupt_all_long_tasks()
        assert cancelled == 1
        assert "long1" not in executor.long_tasks
        assert "long1" not in executor._result_events


# ============================================================
# execute_tools_parallel
# ============================================================


class TestExecuteToolsParallel:
    @pytest.mark.asyncio
    async def test_parallel_mixed_types(self):
        """Смешанный пакет tool_calls: INSTANT + FAST + MEDIUM + LONG."""
        executor, pub, _ = _make_executor()

        async def _deliver_all():
            import json
            # FAST/MEDIUM/INSTANT запросы публикуются последовательно (INSTANT →
            # FAST → MEDIUM → LONG), поэтому deliver-таск должен дожидаться
            # появления каждого сообщения, а не отрабатывать один раз.
            delivered = set()
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                for msg in pub.published:
                    data = json.loads(msg.data)
                    rid = data["request_id"]
                    if rid in delivered:
                        continue
                    if data["tool_name"] in ("play_sound", "get_status", "play_animation"):
                        executor.on_result_received(rid, _result_payload(rid, done=True))
                        delivered.add(rid)
                if delivered == {"c1", "c2", "c3"}:
                    break
                await asyncio.sleep(0.01)

        asyncio.create_task(_deliver_all())

        tool_calls = [
            {"id": "c1", "function": {"name": "play_animation", "arguments": {"animation": "happy"}}},
            {"id": "c2", "function": {"name": "play_sound", "arguments": {"sound": "ding"}}},
            {"id": "c3", "function": {"name": "get_status", "arguments": {}}},
            {"id": "c4", "function": {"name": "navigate_to_waypoint", "arguments": {"waypoint": "kitchen"}}},
        ]

        results = await executor.execute_tools_parallel(tool_calls)

        assert len(results) == 4
        # INSTANT вернул fire-and-forget
        assert results[0]["success"] is True
        # FAST и MEDIUM вернули результат
        assert results[1]["done"] is True
        assert results[2]["done"] is True
        # LONG — background
        assert results[3]["background"] is True

    @pytest.mark.asyncio
    async def test_parallel_empty_and_sequence_validation(self):
        """Пустой список и невалидная sequence."""
        executor, _, _ = _make_executor()

        assert await executor.execute_tools_parallel([]) == []

        await executor.new_sequence()
        results = await executor.execute_tools_parallel(
            [{"id": "x1", "function": {"name": "play_animation", "arguments": {}}}],
            sequence_id=0,  # устаревшая sequence
        )
        assert results[0]["cancelled"] is True
