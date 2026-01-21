"""Unit tests for ToolCallAccumulator."""

import json
import pytest
from rob_box_mcp_tools.core.tool_call_accumulator import ToolCallAccumulator


class MockToolCallDelta:
    """Mock object for tool_call delta from streaming API."""
    
    def __init__(self, index, id=None, type=None, function_name=None, function_arguments=None):
        self.index = index
        if id is not None:
            self.id = id
        if type is not None:
            self.type = type
        if function_name is not None or function_arguments is not None:
            self.function = MockFunction(function_name, function_arguments)


class MockFunction:
    """Mock object for function in tool_call delta."""
    
    def __init__(self, name=None, arguments=None):
        if name is not None:
            self.name = name
        if arguments is not None:
            self.arguments = arguments


class TestToolCallAccumulatorBasics:
    """Test basic accumulator functionality."""
    
    def test_initialization(self):
        """Test accumulator initializes with empty buffer."""
        accumulator = ToolCallAccumulator()
        assert accumulator.get_count() == 0
        assert not accumulator.has_tool_calls()
        assert accumulator.get_complete_tool_calls() == []
    
    def test_single_complete_call(self):
        """Test adding a single complete tool_call in one chunk."""
        accumulator = ToolCallAccumulator()
        
        # Add complete call
        delta = MockToolCallDelta(
            index=0,
            id="call_123",
            type="function",
            function_name="set_emotion",
            function_arguments='{"emotion": "happy"}'
        )
        accumulator.add_chunk([delta])
        
        assert accumulator.get_count() == 1
        assert accumulator.has_tool_calls()
        
        calls = accumulator.get_complete_tool_calls()
        assert len(calls) == 1
        assert calls[0]["id"] == "call_123"
        assert calls[0]["type"] == "function"
        assert calls[0]["function"]["name"] == "set_emotion"
        assert calls[0]["function"]["arguments"] == {"emotion": "happy"}
    
    def test_incremental_arguments(self):
        """Test accumulating arguments across multiple chunks."""
        accumulator = ToolCallAccumulator()
        
        # Chunk 1: Initialize with id and function name
        delta1 = MockToolCallDelta(
            index=0,
            id="call_456",
            type="function",
            function_name="navigate_to"
        )
        accumulator.add_chunk([delta1])
        
        # Chunk 2: First part of arguments
        delta2 = MockToolCallDelta(index=0, function_arguments='{"waypo')
        accumulator.add_chunk([delta2])
        
        # Chunk 3: Middle part
        delta3 = MockToolCallDelta(index=0, function_arguments='int": "k')
        accumulator.add_chunk([delta3])
        
        # Chunk 4: Final part
        delta4 = MockToolCallDelta(index=0, function_arguments='itchen"}')
        accumulator.add_chunk([delta4])
        
        calls = accumulator.get_complete_tool_calls()
        assert len(calls) == 1
        assert calls[0]["id"] == "call_456"
        assert calls[0]["function"]["name"] == "navigate_to"
        assert calls[0]["function"]["arguments"] == {"waypoint": "kitchen"}
    
    def test_multiple_tool_calls(self):
        """Test accumulating multiple tool_calls."""
        accumulator = ToolCallAccumulator()
        
        # First tool_call
        delta1 = MockToolCallDelta(
            index=0,
            id="call_1",
            function_name="set_emotion",
            function_arguments='{"emotion": "happy"}'
        )
        accumulator.add_chunk([delta1])
        
        # Second tool_call
        delta2 = MockToolCallDelta(
            index=1,
            id="call_2",
            function_name="navigate_to",
            function_arguments='{"waypoint": "home"}'
        )
        accumulator.add_chunk([delta2])
        
        assert accumulator.get_count() == 2
        calls = accumulator.get_complete_tool_calls()
        assert len(calls) == 2
        assert calls[0]["function"]["name"] == "set_emotion"
        assert calls[1]["function"]["name"] == "navigate_to"


class TestToolCallAccumulatorEdgeCases:
    """Test edge cases and error handling."""
    
    def test_empty_arguments(self):
        """Test tool_call with empty arguments."""
        accumulator = ToolCallAccumulator()
        
        delta = MockToolCallDelta(
            index=0,
            id="call_789",
            function_name="stop",
            function_arguments=''
        )
        accumulator.add_chunk([delta])
        
        calls = accumulator.get_complete_tool_calls()
        assert len(calls) == 1
        assert calls[0]["function"]["arguments"] == {}
    
    def test_invalid_json_arguments(self):
        """Test handling invalid JSON in arguments."""
        accumulator = ToolCallAccumulator()
        
        delta = MockToolCallDelta(
            index=0,
            id="call_bad",
            function_name="test",
            function_arguments='{"invalid": json'  # Invalid JSON
        )
        accumulator.add_chunk([delta])
        
        calls = accumulator.get_complete_tool_calls()
        assert len(calls) == 1
        # Should return raw call when JSON parsing fails
        assert "id" in calls[0]
    
    def test_no_function_name(self):
        """Test tool_call without function name."""
        accumulator = ToolCallAccumulator()
        
        delta = MockToolCallDelta(
            index=0,
            id="call_no_name",
            function_arguments='{"param": "value"}'
        )
        accumulator.add_chunk([delta])
        
        calls = accumulator.get_complete_tool_calls()
        assert len(calls) == 1
        assert calls[0]["function"]["name"] is None
    
    def test_clear(self):
        """Test clearing accumulator."""
        accumulator = ToolCallAccumulator()
        
        delta = MockToolCallDelta(
            index=0,
            id="call_xyz",
            function_name="test",
            function_arguments='{"test": "value"}'
        )
        accumulator.add_chunk([delta])
        
        assert accumulator.get_count() == 1
        
        accumulator.clear()
        
        assert accumulator.get_count() == 0
        assert not accumulator.has_tool_calls()
        assert accumulator.get_complete_tool_calls() == []
    
    def test_multiple_chunks_in_one_call(self):
        """Test adding multiple deltas in a single add_chunk call."""
        accumulator = ToolCallAccumulator()
        
        deltas = [
            MockToolCallDelta(index=0, id="call_1", function_name="func1", function_arguments='{"a": 1}'),
            MockToolCallDelta(index=1, id="call_2", function_name="func2", function_arguments='{"b": 2}'),
        ]
        accumulator.add_chunk(deltas)
        
        assert accumulator.get_count() == 2
        calls = accumulator.get_complete_tool_calls()
        assert len(calls) == 2
    
    def test_out_of_order_indices(self):
        """Test that indices are properly sorted in output."""
        accumulator = ToolCallAccumulator()
        
        # Add in reverse order
        delta2 = MockToolCallDelta(index=2, id="call_3", function_name="third", function_arguments='{"c": 3}')
        delta0 = MockToolCallDelta(index=0, id="call_1", function_name="first", function_arguments='{"a": 1}')
        delta1 = MockToolCallDelta(index=1, id="call_2", function_name="second", function_arguments='{"b": 2}')
        
        accumulator.add_chunk([delta2])
        accumulator.add_chunk([delta0])
        accumulator.add_chunk([delta1])
        
        calls = accumulator.get_complete_tool_calls()
        assert len(calls) == 3
        assert calls[0]["function"]["name"] == "first"
        assert calls[1]["function"]["name"] == "second"
        assert calls[2]["function"]["name"] == "third"


class TestToolCallAccumulatorRealWorld:
    """Test realistic streaming scenarios."""
    
    def test_realistic_streaming_sequence(self):
        """Test a realistic sequence of streaming chunks."""
        accumulator = ToolCallAccumulator()
        
        # Chunk 1: Initial call with ID and type
        chunk1 = [MockToolCallDelta(index=0, id="call_abc123", type="function")]
        accumulator.add_chunk(chunk1)
        
        # Chunk 2: Function name
        chunk2 = [MockToolCallDelta(index=0, function_name="set_emotion")]
        accumulator.add_chunk(chunk2)
        
        # Chunk 3-7: Arguments streamed character by character (realistic)
        chunks = [
            [MockToolCallDelta(index=0, function_arguments='{"')],
            [MockToolCallDelta(index=0, function_arguments='emotion')],
            [MockToolCallDelta(index=0, function_arguments='": "')],
            [MockToolCallDelta(index=0, function_arguments='радость')],
            [MockToolCallDelta(index=0, function_arguments='"}')],
        ]
        for chunk in chunks:
            accumulator.add_chunk(chunk)
        
        calls = accumulator.get_complete_tool_calls()
        assert len(calls) == 1
        assert calls[0]["id"] == "call_abc123"
        assert calls[0]["type"] == "function"
        assert calls[0]["function"]["name"] == "set_emotion"
        assert calls[0]["function"]["arguments"] == {"emotion": "радость"}
    
    def test_parallel_tool_calls_streaming(self):
        """Test multiple tool_calls streaming in parallel."""
        accumulator = ToolCallAccumulator()
        
        # Both calls start
        chunk1 = [
            MockToolCallDelta(index=0, id="call_1", type="function"),
            MockToolCallDelta(index=1, id="call_2", type="function"),
        ]
        accumulator.add_chunk(chunk1)
        
        # Names arrive
        chunk2 = [
            MockToolCallDelta(index=0, function_name="navigate_to"),
            MockToolCallDelta(index=1, function_name="set_emotion"),
        ]
        accumulator.add_chunk(chunk2)
        
        # Arguments stream in
        chunk3 = [
            MockToolCallDelta(index=0, function_arguments='{"waypoint"'),
            MockToolCallDelta(index=1, function_arguments='{"emotion"'),
        ]
        accumulator.add_chunk(chunk3)
        
        chunk4 = [
            MockToolCallDelta(index=0, function_arguments=': "kitchen"}'),
            MockToolCallDelta(index=1, function_arguments=': "happy"}'),
        ]
        accumulator.add_chunk(chunk4)
        
        calls = accumulator.get_complete_tool_calls()
        assert len(calls) == 2
        assert calls[0]["function"]["name"] == "navigate_to"
        assert calls[0]["function"]["arguments"] == {"waypoint": "kitchen"}
        assert calls[1]["function"]["name"] == "set_emotion"
        assert calls[1]["function"]["arguments"] == {"emotion": "happy"}
