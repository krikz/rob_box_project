"""
Tool Call Accumulator - accumulates streaming tool_calls from LLM API

Handles incremental tool_calls from OpenAI-compatible streaming responses,
assembling fragments into complete tool call structures.
"""

import json
from typing import Any, Dict, List


class ToolCallAccumulator:
    """
    Accumulates tool_calls from streaming chunks.
    
    LLM API returns tool_calls in fragments during streaming:
    - chunk 1: {tool_calls: [{index: 0, id: "call_123", type: "function", function: {name: "play_animation"}}]}
    - chunk 2: {tool_calls: [{index: 0, function: {arguments: '{"ani'}}]}
    - chunk 3: {tool_calls: [{index: 0, function: {arguments: 'mation": '}}]}
    - chunk 4: {tool_calls: [{index: 0, function: {arguments: '"happy"}'}}]}
    
    This class assembles all fragments into complete tool_call structures.
    
    Example:
        >>> accumulator = ToolCallAccumulator()
        >>> # Add chunks as they arrive
        >>> accumulator.add_chunk([delta_chunk1])
        >>> accumulator.add_chunk([delta_chunk2])
        >>> # Get complete tool calls
        >>> complete_calls = accumulator.get_complete_tool_calls()
        >>> # Returns: [{"id": "call_123", "type": "function", "function": {"name": "play_animation", "arguments": {"animation": "happy"}}}]
    """
    
    def __init__(self):
        """Initialize accumulator with empty buffer."""
        self.tool_calls_buffer: Dict[int, Dict[str, Any]] = {}
    
    def add_chunk(self, delta_tool_calls: List[Any]) -> None:
        """
        Add tool_calls chunk from streaming response.
        
        Args:
            delta_tool_calls: List of tool_call delta objects from OpenAI-compatible API
        """
        for tc in delta_tool_calls:
            index = tc.index
            
            if index not in self.tool_calls_buffer:
                # Initialize new tool_call
                self.tool_calls_buffer[index] = {
                    "id": getattr(tc, "id", None),
                    "type": getattr(tc, "type", "function"),
                    "function": {"name": None, "arguments": ""},
                }
            
            # Update existing tool_call
            if hasattr(tc, "id") and tc.id:
                self.tool_calls_buffer[index]["id"] = tc.id
            
            if hasattr(tc, "function") and tc.function:
                func = tc.function
                if hasattr(func, "name") and func.name:
                    self.tool_calls_buffer[index]["function"]["name"] = func.name
                if hasattr(func, "arguments") and func.arguments:
                    self.tool_calls_buffer[index]["function"]["arguments"] += func.arguments
    
    def get_complete_tool_calls(self) -> List[Dict[str, Any]]:
        """
        Get complete list of accumulated tool_calls.
        
        Returns:
            List of tool_calls in format: [{"id": str, "type": str, "function": {"name": str, "arguments": dict}}]
        """
        # Sort by index
        sorted_calls = [self.tool_calls_buffer[idx] for idx in sorted(self.tool_calls_buffer.keys())]
        
        # Parse arguments from JSON string
        result = []
        for call in sorted_calls:
            try:
                arguments_str = call["function"]["arguments"]
                parsed_args = json.loads(arguments_str) if arguments_str else {}
                
                result.append(
                    {
                        "id": call["id"],
                        "type": call["type"],
                        "function": {"name": call["function"]["name"], "arguments": parsed_args},
                    }
                )
            except json.JSONDecodeError:
                # If cannot parse - leave as is
                result.append(call)
        
        return result
    
    def clear(self) -> None:
        """Clear accumulator buffer."""
        self.tool_calls_buffer.clear()
    
    def get_count(self) -> int:
        """
        Get count of accumulated tool_calls.
        
        Returns:
            Number of tool_calls in buffer
        """
        return len(self.tool_calls_buffer)
    
    def has_tool_calls(self) -> bool:
        """
        Check if accumulator has any tool_calls.
        
        Returns:
            True if at least one tool_call is buffered
        """
        return len(self.tool_calls_buffer) > 0
