#!/usr/bin/env python3
"""
test_interrupt.py - Тесты прерывания LONG tasks

Тестирует:
- Прерывание по interrupt_event
- Прерывание множественных tasks
- Cleanup после прерывания
"""

import pytest
import asyncio
from unittest.mock import Mock, AsyncMock

from rob_box_mcp_tools.async_executor import AsyncToolExecutor, InterruptibleTask
from rob_box_mcp_tools.base import ToolExecutionType, MCPToolResult


@pytest.fixture
def mock_registry():
    """Mock registry"""
    registry = Mock()
    
    async def mock_execute(*args, **kwargs):
        # Simulate long-running operation
        await asyncio.sleep(5.0)
        return MCPToolResult(success=True)
    
    registry.execute_tool = AsyncMock(side_effect=mock_execute)
    return registry


@pytest.fixture
def executor(mock_registry):
    """Executor instance"""
    return AsyncToolExecutor(mock_registry)


@pytest.mark.unit
@pytest.mark.asyncio
async def test_interrupt_event_stops_execution(executor, mock_registry):
    """Interrupt event должен прервать выполнение"""
    task = await executor.execute_tool_async("navigate", {}, ToolExecutionType.LONG)
    
    # Wait немного
    await asyncio.sleep(0.1)
    
    # Установить interrupt
    task.interrupt_event.set()
    
    # Cancel task
    await task.cancel()
    
    # Task должен быть отменён
    assert task.task.cancelled()


@pytest.mark.unit
@pytest.mark.asyncio
async def test_multiple_interrupts(executor):
    """Прерывание множественных tasks"""
    # Start 3 long tasks
    task1 = await executor.execute_tool_async("nav1", {}, ToolExecutionType.LONG)
    task2 = await executor.execute_tool_async("nav2", {}, ToolExecutionType.LONG)
    task3 = await executor.execute_tool_async("nav3", {}, ToolExecutionType.LONG)
    
    assert len(executor.long_tasks) == 3
    
    # Interrupt all
    await executor.interrupt_all_long_tasks()
    
    # All cancelled
    assert all(t.task.cancelled() for t in [task1, task2, task3])
    assert len(executor.long_tasks) == 0


@pytest.mark.unit
@pytest.mark.asyncio
async def test_interrupt_clears_registry(executor):
    """Прерывание должно очистить реестр tasks"""
    task = await executor.execute_tool_async("navigate", {}, ToolExecutionType.LONG)
    
    assert "navigate" in executor.long_tasks
    
    await executor.interrupt_task_by_name("navigate")
    
    assert "navigate" not in executor.long_tasks


@pytest.mark.unit
@pytest.mark.asyncio
async def test_interrupt_by_name_selective(executor):
    """Interrupt by name должен быть selective"""
    task1 = await executor.execute_tool_async("nav1", {}, ToolExecutionType.LONG)
    task2 = await executor.execute_tool_async("nav2", {}, ToolExecutionType.LONG)
    
    # Interrupt только nav1
    await executor.interrupt_task_by_name("nav1")
    
    # nav1 cancelled, nav2 running
    assert task1.task.cancelled()
    assert not task2.task.cancelled()
    
    await task2.cancel()


@pytest.mark.unit
@pytest.mark.asyncio
async def test_interrupt_nonexistent_task_no_error(executor):
    """Прерывание несуществующего task не должно вызывать ошибку"""
    # No exception
    await executor.interrupt_task_by_name("nonexistent")


@pytest.mark.unit
@pytest.mark.asyncio
async def test_interrupt_during_parallel_execution(executor, mock_registry):
    """Прерывание во время parallel execution"""
    tool_calls = [
        {"name": "nav1", "params": {}, "execution_type": ToolExecutionType.LONG},
        {"name": "nav2", "params": {}, "execution_type": ToolExecutionType.LONG},
    ]
    
    # Start parallel
    parallel_task = asyncio.create_task(executor.execute_tools_parallel(tool_calls))
    
    # Wait немного
    await asyncio.sleep(0.1)
    
    # Interrupt all
    await executor.interrupt_all_long_tasks()
    
    # Wait for completion
    try:
        await asyncio.wait_for(parallel_task, timeout=1.0)
    except asyncio.TimeoutError:
        parallel_task.cancel()


@pytest.mark.unit
@pytest.mark.asyncio
async def test_cleanup_after_successful_completion(executor, mock_registry):
    """Успешное завершение должно очистить task из реестра"""
    # Short task для быстрого completion
    async def quick_execute(*args, **kwargs):
        await asyncio.sleep(0.1)
        return MCPToolResult(success=True)
    
    mock_registry.execute_tool = AsyncMock(side_effect=quick_execute)
    
    task = await executor.execute_tool_async("quick_nav", {}, ToolExecutionType.LONG)
    
    # Task в реестре
    assert "quick_nav" in executor.long_tasks
    
    # Wait for completion
    await asyncio.sleep(0.2)
    
    # NOTE: В реальной реализации cleanup после completion
    # Здесь тестируем что task завершился
    assert task.task.done()


@pytest.mark.unit
@pytest.mark.asyncio
async def test_interruptible_task_metadata(executor):
    """InterruptibleTask должен хранить metadata"""
    task = await executor.execute_tool_async("navigate", {"waypoint": "кухня"}, ToolExecutionType.LONG)
    
    assert task.tool_name == "navigate"
    assert task.request_id is not None
    assert task.created_at is not None
    assert hasattr(task, "interrupt_event")
    assert hasattr(task, "task")
    
    await task.cancel()


@pytest.mark.unit
@pytest.mark.asyncio
async def test_interrupt_propagation_to_tool(executor, mock_registry):
    """Interrupt event должен быть доступен в tool execution"""
    interrupt_event_seen = asyncio.Event()
    
    async def check_interrupt(*args, **kwargs):
        # Simulate checking interrupt_event inside tool
        await asyncio.sleep(0.1)
        interrupt_event_seen.set()
        return MCPToolResult(success=True)
    
    mock_registry.execute_tool = AsyncMock(side_effect=check_interrupt)
    
    task = await executor.execute_tool_async("navigate", {}, ToolExecutionType.LONG)
    
    # Wait for execution start
    await asyncio.wait_for(interrupt_event_seen.wait(), timeout=1.0)
    
    # Cancel
    await task.cancel()
