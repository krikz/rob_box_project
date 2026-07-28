"""
Tests for ToolCallExecutor module
"""

import unittest
import json
from unittest.mock import Mock, MagicMock
from rob_box_voice.llm.tool_call_executor import ToolCallExecutor, ToolCallResult


class TestToolCallResult(unittest.TestCase):
    """Tests for ToolCallResult dataclass."""

    def test_successful_result(self):
        """Test successful tool call result."""
        result = ToolCallResult(
            tool_call_id="call_123",
            tool_name="test_tool",
            success=True,
            content='{"result": "success"}'
        )

        self.assertEqual(result.tool_call_id, "call_123")
        self.assertEqual(result.tool_name, "test_tool")
        self.assertTrue(result.success)
        self.assertEqual(result.content, '{"result": "success"}')
        self.assertIsNone(result.error)

    def test_failed_result(self):
        """Test failed tool call result."""
        result = ToolCallResult(
            tool_call_id="call_456",
            tool_name="bad_tool",
            success=False,
            content='{"error": "failed"}',
            error="Tool execution failed"
        )

        self.assertFalse(result.success)
        self.assertEqual(result.error, "Tool execution failed")


class TestToolCallExecutorInit(unittest.TestCase):
    """Tests for ToolCallExecutor initialization."""

    def test_init_default(self):
        """Test initialization with defaults."""
        executor_func = Mock()
        executor = ToolCallExecutor(tool_executor=executor_func)

        self.assertEqual(executor.tool_executor, executor_func)
        self.assertEqual(executor.max_iterations, 10)
        self.assertFalse(executor.interrupt_flag)

    def test_init_custom_max_iterations(self):
        """Test initialization with custom max iterations."""
        executor_func = Mock()
        executor = ToolCallExecutor(tool_executor=executor_func, max_iterations=5)

        self.assertEqual(executor.max_iterations, 5)

    def test_set_interrupt(self):
        """Test setting interrupt flag."""
        executor_func = Mock()
        executor = ToolCallExecutor(tool_executor=executor_func)

        self.assertFalse(executor.interrupt_flag)

        executor.set_interrupt(True)
        self.assertTrue(executor.interrupt_flag)

        executor.set_interrupt(False)
        self.assertFalse(executor.interrupt_flag)


class TestToolCallParsing(unittest.TestCase):
    """Tests for parsing tool calls from LLM messages."""

    def setUp(self):
        self.executor_func = Mock()
        self.executor = ToolCallExecutor(tool_executor=self.executor_func)

    def test_parse_single_tool_call(self):
        """Test parsing single tool call."""
        # Mock LLM message with tool_calls
        message = Mock()
        tool_call = Mock()
        tool_call.id = "call_123"
        tool_call.function = Mock()
        tool_call.function.name = "get_weather"
        tool_call.function.arguments = '{"location": "Moscow"}'
        message.tool_calls = [tool_call]

        result = self.executor.parse_tool_calls(message)

        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]['id'], 'call_123')
        self.assertEqual(result[0]['type'], 'function')
        self.assertEqual(result[0]['function']['name'], 'get_weather')
        self.assertEqual(result[0]['function']['arguments'], '{"location": "Moscow"}')

    def test_parse_multiple_tool_calls(self):
        """Test parsing multiple tool calls."""
        message = Mock()

        tc1 = Mock()
        tc1.id = "call_1"
        tc1.function = Mock()
        tc1.function.name = "tool1"
        tc1.function.arguments = '{"arg1": "val1"}'

        tc2 = Mock()
        tc2.id = "call_2"
        tc2.function = Mock()
        tc2.function.name = "tool2"
        tc2.function.arguments = '{"arg2": "val2"}'

        message.tool_calls = [tc1, tc2]

        result = self.executor.parse_tool_calls(message)

        self.assertEqual(len(result), 2)
        self.assertEqual(result[0]['function']['name'], 'tool1')
        self.assertEqual(result[1]['function']['name'], 'tool2')

    def test_parse_no_tool_calls(self):
        """Test parsing message with no tool calls."""
        message = Mock()
        message.tool_calls = None

        result = self.executor.parse_tool_calls(message)

        self.assertEqual(result, [])

    def test_parse_empty_tool_calls(self):
        """Test parsing message with empty tool calls."""
        message = Mock()
        message.tool_calls = []

        result = self.executor.parse_tool_calls(message)

        self.assertEqual(result, [])


class TestToolCallExecution(unittest.TestCase):
    """Tests for executing tool calls."""

    def setUp(self):
        self.executor_func = Mock()
        self.executor = ToolCallExecutor(tool_executor=self.executor_func)

    def test_execute_successful_tool_call(self):
        """Test successful tool call execution."""
        self.executor_func.return_value = {
            'success': True,
            'message': 'Tool executed successfully',
            'data': {'result': 'value'}
        }

        result = self.executor.execute_tool_call(
            tool_call_id="call_123",
            tool_name="test_tool",
            tool_args_str='{"param": "value"}',
            timeout=10.0
        )

        self.assertEqual(result.tool_call_id, "call_123")
        self.assertEqual(result.tool_name, "test_tool")
        self.assertTrue(result.success)
        self.assertIn('success', result.content)
        self.assertIsNone(result.error)

        # Verify executor was called correctly
        self.executor_func.assert_called_once_with(
            "test_tool",
            {"param": "value"},
            10.0
        )

    def test_execute_failed_tool_call(self):
        """Test failed tool call execution."""
        self.executor_func.return_value = {
            'success': False,
            'error': 'Tool failed to execute'
        }

        result = self.executor.execute_tool_call(
            tool_call_id="call_456",
            tool_name="bad_tool",
            tool_args_str='{"param": "bad"}',
            timeout=5.0
        )

        self.assertEqual(result.tool_call_id, "call_456")
        self.assertEqual(result.tool_name, "bad_tool")
        self.assertFalse(result.success)
        self.assertEqual(result.error, 'Tool failed to execute')

    def test_execute_invalid_json_arguments(self):
        """Test execution with invalid JSON arguments."""
        result = self.executor.execute_tool_call(
            tool_call_id="call_789",
            tool_name="test_tool",
            tool_args_str='invalid json',
            timeout=10.0
        )

        self.assertFalse(result.success)
        self.assertIsNotNone(result.error)
        self.executor_func.assert_not_called()

    def test_execute_tool_exception(self):
        """Test execution when tool raises exception."""
        self.executor_func.side_effect = RuntimeError("Tool crashed")

        result = self.executor.execute_tool_call(
            tool_call_id="call_999",
            tool_name="crash_tool",
            tool_args_str='{}',
            timeout=10.0
        )

        self.assertFalse(result.success)
        self.assertIn('Tool crashed', result.error)


class TestMessageBuilding(unittest.TestCase):
    """Tests for building message dictionaries."""

    def setUp(self):
        self.executor_func = Mock()
        self.executor = ToolCallExecutor(tool_executor=self.executor_func)

    def test_build_tool_result_message(self):
        """Test building tool result message."""
        result = ToolCallResult(
            tool_call_id="call_123",
            tool_name="test_tool",
            success=True,
            content='{"result": "ok"}'
        )

        message = self.executor.build_tool_result_message(result)

        self.assertEqual(message['role'], 'tool')
        self.assertEqual(message['tool_call_id'], 'call_123')
        self.assertEqual(message['name'], 'test_tool')
        self.assertEqual(message['content'], '{"result": "ok"}')

    def test_build_assistant_message_with_tool_calls(self):
        """Test building assistant message with tool calls."""
        tool_calls = [
            {
                "id": "call_1",
                "type": "function",
                "function": {"name": "tool1", "arguments": "{}"}
            }
        ]

        message = self.executor.build_assistant_message_with_tool_calls(
            content="Let me check that",
            tool_calls=tool_calls
        )

        self.assertEqual(message['role'], 'assistant')
        self.assertEqual(message['content'], 'Let me check that')
        self.assertEqual(message['tool_calls'], tool_calls)


class TestProcessToolCallsIteration(unittest.TestCase):
    """Tests for processing multiple tool calls."""

    def setUp(self):
        self.executor_func = Mock()
        self.executor = ToolCallExecutor(tool_executor=self.executor_func)

    def test_process_single_tool_call(self):
        """Test processing single tool call."""
        self.executor_func.return_value = {'success': True, 'data': 'result'}

        tool_calls = [
            {
                "id": "call_1",
                "function": {
                    "name": "tool1",
                    "arguments": '{"param": "value"}'
                }
            }
        ]

        results = self.executor.process_tool_calls_iteration(tool_calls, timeout=10.0)

        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].tool_call_id, "call_1")
        self.assertTrue(results[0].success)

    def test_process_multiple_tool_calls(self):
        """Test processing multiple tool calls."""
        self.executor_func.return_value = {'success': True}

        tool_calls = [
            {"id": "call_1", "function": {"name": "tool1", "arguments": "{}"}},
            {"id": "call_2", "function": {"name": "tool2", "arguments": "{}"}},
            {"id": "call_3", "function": {"name": "tool3", "arguments": "{}"}}
        ]

        results = self.executor.process_tool_calls_iteration(tool_calls)

        self.assertEqual(len(results), 3)
        self.assertEqual(self.executor_func.call_count, 3)


class TestAgentLoopControl(unittest.TestCase):
    """Tests for agent loop control logic."""

    def setUp(self):
        self.executor_func = Mock()
        self.executor = ToolCallExecutor(tool_executor=self.executor_func, max_iterations=5)

    def test_should_continue_with_tool_calls(self):
        """Test should continue when there are tool calls."""
        result = self.executor.should_continue_agent_loop(iteration=1, has_tool_calls=True)
        self.assertTrue(result)

    def test_should_stop_without_tool_calls(self):
        """Test should stop when no tool calls (final answer)."""
        result = self.executor.should_continue_agent_loop(iteration=1, has_tool_calls=False)
        self.assertFalse(result)

    def test_should_stop_at_max_iterations(self):
        """Test should stop at max iterations."""
        result = self.executor.should_continue_agent_loop(iteration=5, has_tool_calls=True)
        self.assertFalse(result)

    def test_should_stop_when_interrupted(self):
        """Test should stop when interrupt flag is set."""
        self.executor.set_interrupt(True)
        result = self.executor.should_continue_agent_loop(iteration=1, has_tool_calls=True)
        self.assertFalse(result)
        # Interrupt flag should be reset
        self.assertFalse(self.executor.interrupt_flag)

    def test_should_continue_below_max_iterations(self):
        """Test should continue below max iterations."""
        result = self.executor.should_continue_agent_loop(iteration=3, has_tool_calls=True)
        self.assertTrue(result)


if __name__ == '__main__':
    unittest.main()
