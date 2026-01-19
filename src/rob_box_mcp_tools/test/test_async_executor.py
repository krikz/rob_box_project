#!/usr/bin/env python3
"""
test_async_executor.py - Тесты для AsyncToolExecutor

Тестирует:
- Асинхронное выполнение инструментов
- Параллельное выполнение через asyncio.gather()
- InterruptibleTask и cancellation
- Timeout handling
- Error handling
"""

import pytest
import asyncio
from unittest.mock import Mock, AsyncMock, patch, call

from rob_box_mcp_tools.async_executor import AsyncToolExecutor, InterruptibleTask, ToolCallAccumulator
from rob_box_mcp_tools.base import ToolExecutionType, MCPToolResult


@pytest.fixture
def mock_registry():
    """Mock MCPToolRegistry"""
    registry = Mock()
    registry.execute_tool = Mock(return_value=MCPToolResult(success=True, message="OK"))
    return registry


@pytest.fixture
def executor(mock_registry):
    """AsyncToolExecutor instance"""
    return AsyncToolExecutor(mock_registry)


@pytest.mark.unit
@pytest.mark.asyncio
async def test_execute_instant_tool_fire_and_forget(executor, mock_registry):
    """INSTANT tools должны выполняться fire-and-forget"""
    result = await executor.execute_tool_async("set_emotion", {"emotion": "радость"}, ToolExecutionType.INSTANT)
    
    # Должен вернуться сразу без ожидания
    assert result.success is True
    
    # Registry execute должен быть вызван
    mock_registry.execute_tool.assert_called_once_with("set_emotion", {"emotion": "радость"})


@pytest.mark.unit
@pytest.mark.asyncio
async def test_execute_fast_tool_with_await(executor, mock_registry):
    """FAST tools должны await completion"""
    result = await executor.execute_tool_async("play_sound", {"sound": "happy"}, ToolExecutionType.FAST)
    
    assert result.success is True
    mock_registry.execute_tool.assert_called_once_with("play_sound", {"sound": "happy"})


@pytest.mark.unit
@pytest.mark.asyncio
async def test_execute_medium_tool_with_await(executor, mock_registry):
    """MEDIUM tools должны await completion"""
    result = await executor.execute_tool_async("get_robot_status", {}, ToolExecutionType.MEDIUM)
    
    assert result.success is True
    mock_registry.execute_tool.assert_called_once()


@pytest.mark.unit
@pytest.mark.asyncio
async def test_execute_long_tool_background_task(executor, mock_registry):
    """LONG tools должны запускаться как background task"""
    # Long tool возвращает InterruptibleTask
    task = await executor.execute_tool_async("navigate_to_waypoint", {"waypoint": "кухня"}, ToolExecutionType.LONG)
    
    assert isinstance(task, InterruptibleTask)
    assert task.tool_name == "navigate_to_waypoint"
    
    # Task должен быть в реестре
    assert "navigate_to_waypoint" in executor.long_tasks
    
    # Cleanup
    await task.cancel()


@pytest.mark.unit
@pytest.mark.asyncio
async def test_parallel_execution_instant_and_fast(executor, mock_registry):
    """Параллельное выполнение INSTANT + FAST tools"""
    tool_calls = [
        {"name": "set_emotion", "params": {"emotion": "радость"}, "execution_type": ToolExecutionType.INSTANT},
        {"name": "play_sound", "params": {"sound": "happy"}, "execution_type": ToolExecutionType.FAST},
    ]
    
    start_time = asyncio.get_event_loop().time()
    results = await executor.execute_tools_parallel(tool_calls)
    duration = asyncio.get_event_loop().time() - start_time
    
    # Оба выполнились
    assert len(results) == 2
    assert all(r.success for r in results)
    
    # Параллельно (не последовательно)
    # INSTANT: 0ms, FAST: ~50ms mock → total < 100ms (не 150ms)
    assert duration < 0.15  # 150ms threshold


@pytest.mark.unit
@pytest.mark.asyncio
async def test_parallel_execution_groups_by_type(executor, mock_registry):
    """Группировка по execution_type перед параллельным выполнением"""
    tool_calls = [
        {"name": "set_emotion", "params": {}, "execution_type": ToolExecutionType.INSTANT},
        {"name": "play_animation", "params": {}, "execution_type": ToolExecutionType.INSTANT},
        {"name": "play_sound", "params": {}, "execution_type": ToolExecutionType.FAST},
        {"name": "set_volume", "params": {}, "execution_type": ToolExecutionType.FAST},
    ]
    
    results = await executor.execute_tools_parallel(tool_calls)
    
    # Все 4 выполнены
    assert len(results) == 4
    
    # Registry вызван 4 раза
    assert mock_registry.execute_tool.call_count == 4


@pytest.mark.unit
@pytest.mark.asyncio
async def test_interrupt_all_long_tasks(executor, mock_registry):
    """Прерывание всех LONG tasks"""
    # Запустить 2 long tasks
    task1 = await executor.execute_tool_async("navigate_to_waypoint", {"waypoint": "кухня"}, ToolExecutionType.LONG)
    task2 = await executor.execute_tool_async("move_direction", {"direction": "вперёд"}, ToolExecutionType.LONG)
    
    assert len(executor.long_tasks) == 2
    
    # Прервать все
    await executor.interrupt_all_long_tasks()
    
    # Реестр очищен
    assert len(executor.long_tasks) == 0
    
    # Tasks отменены
    assert task1.task.cancelled()
    assert task2.task.cancelled()


@pytest.mark.unit
@pytest.mark.asyncio
async def test_interrupt_task_by_name(executor, mock_registry):
    """Прерывание конкретного task по имени"""
    task1 = await executor.execute_tool_async("navigate_to_waypoint", {"waypoint": "кухня"}, ToolExecutionType.LONG)
    task2 = await executor.execute_tool_async("move_direction", {"direction": "вперёд"}, ToolExecutionType.LONG)
    
    # Прервать только navigate_to_waypoint
    await executor.interrupt_task_by_name("navigate_to_waypoint")
    
    # Только task1 отменён
    assert task1.task.cancelled()
    assert not task2.task.cancelled()
    
    # В реестре остался только task2
    assert len(executor.long_tasks) == 1
    assert "move_direction" in executor.long_tasks
    
    # Cleanup
    await task2.cancel()


@pytest.mark.unit
@pytest.mark.asyncio
async def test_timeout_handling_fast_tool(executor, mock_registry):
    """Timeout для FAST tool (2s)"""
    # Simulate slow execution
    async def slow_execute(*args, **kwargs):
        await asyncio.sleep(3.0)  # Больше чем timeout
        return MCPToolResult(success=True)
    
    mock_registry.execute_tool = AsyncMock(side_effect=slow_execute)
    
    with pytest.raises(asyncio.TimeoutError):
        await executor.execute_tool_async("play_sound", {}, ToolExecutionType.FAST)


@pytest.mark.unit
@pytest.mark.asyncio
async def test_timeout_handling_medium_tool(executor, mock_registry):
    """Timeout для MEDIUM tool (10s)"""
    async def slow_execute(*args, **kwargs):
        await asyncio.sleep(12.0)  # Больше чем timeout
        return MCPToolResult(success=True)
    
    mock_registry.execute_tool = AsyncMock(side_effect=slow_execute)
    
    with pytest.raises(asyncio.TimeoutError):
        await executor.execute_tool_async("get_status", {}, ToolExecutionType.MEDIUM)


@pytest.mark.unit
@pytest.mark.asyncio
async def test_error_handling_tool_execution(executor, mock_registry):
    """Обработка ошибок при выполнении tool"""
    mock_registry.execute_tool.side_effect = Exception("Tool execution failed")
    
    result = await executor.execute_tool_async("set_emotion", {}, ToolExecutionType.INSTANT)
    
    # Должен вернуть MCPToolResult с ошибкой
    assert result.success is False
    assert "Tool execution failed" in result.error


@pytest.mark.unit
@pytest.mark.asyncio
async def test_interruptible_task_cancel(executor, mock_registry):
    """InterruptibleTask.cancel() должен установить interrupt_event"""
    task = await executor.execute_tool_async("navigate_to_waypoint", {}, ToolExecutionType.LONG)
    
    # Interrupt event не установлен
    assert not task.interrupt_event.is_set()
    
    # Cancel
    await task.cancel()
    
    # Interrupt event установлен
    assert task.interrupt_event.is_set()
    
    # Task отменён
    assert task.task.cancelled()


@pytest.mark.unit
def test_tool_call_accumulator_add_chunk():
    """ToolCallAccumulator должен накапливать chunks"""
    accumulator = ToolCallAccumulator()
    
    # Первый chunk (начало tool_call)
    delta1 = Mock()
    delta1.tool_calls = [Mock(index=0, id="call_1", function=Mock(name="set_emotion", arguments=""))]
    
    accumulator.add_chunk(delta1)
    
    assert "call_1" in accumulator.tool_calls_buffer
    assert accumulator.tool_calls_buffer["call_1"]["function"]["name"] == "set_emotion"
    
    # Второй chunk (incremental arguments)
    delta2 = Mock()
    delta2.tool_calls = [Mock(index=0, id="call_1", function=Mock(name=None, arguments='{"emotion":'))]
    
    accumulator.add_chunk(delta2)
    
    # Arguments накопились
    assert accumulator.tool_calls_buffer["call_1"]["function"]["arguments"] == '{"emotion":'
    
    # Третий chunk (завершение)
    delta3 = Mock()
    delta3.tool_calls = [Mock(index=0, id="call_1", function=Mock(name=None, arguments='"радость"}'))]
    
    accumulator.add_chunk(delta3)
    
    # Полные arguments
    assert accumulator.tool_calls_buffer["call_1"]["function"]["arguments"] == '{"emotion":"радость"}'


@pytest.mark.unit
def test_tool_call_accumulator_get_complete():
    """ToolCallAccumulator должен возвращать готовые tool_calls"""
    accumulator = ToolCallAccumulator()
    
    # Accumulate complete tool_call
    delta = Mock()
    delta.tool_calls = [
        Mock(index=0, id="call_1", function=Mock(name="set_emotion", arguments='{"emotion":"радость"}')),
        Mock(index=1, id="call_2", function=Mock(name="play_sound", arguments='{"sound":"happy"}')),
    ]
    
    accumulator.add_chunk(delta)
    
    # Get complete
    complete = accumulator.get_complete_tool_calls()
    
    assert len(complete) == 2
    assert complete[0]["id"] == "call_1"
    assert complete[0]["function"]["name"] == "set_emotion"
    assert complete[1]["id"] == "call_2"
    assert complete[1]["function"]["name"] == "play_sound"
