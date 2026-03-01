"""
Tool Call Executor Module

Handles execution of LLM tool calls with agent loop support.
Pure Python module for processing tool calls from LLM responses.
"""

import json
import logging
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass


@dataclass
class ToolCallResult:
    """Result of a tool call execution"""
    tool_call_id: str
    tool_name: str
    success: bool
    content: str
    error: Optional[str] = None


class ToolCallExecutor:
    """
    Handles execution of tool calls from LLM responses.
    
    Supports:
    - Parsing tool calls from LLM responses
    - Executing tool calls through adapter
    - Managing agent loop iterations
    - Building message history with tool results
    """
    
    def __init__(
        self,
        tool_executor: Callable[[str, Dict[str, Any], float], Dict[str, Any]],
        max_iterations: int = 10,
        logger: Optional[logging.Logger] = None
    ):
        """
        Initialize tool call executor.
        
        Args:
            tool_executor: Function to execute tool calls
                          Signature: (tool_name, tool_args, timeout) -> result_dict
            max_iterations: Maximum agent loop iterations
            logger: Optional logger instance
        """
        self.tool_executor = tool_executor
        self.max_iterations = max_iterations
        self.logger = logger or logging.getLogger(__name__)
        self.interrupt_flag = False
    
    def set_interrupt(self, value: bool = True):
        """Set interrupt flag to stop agent loop"""
        self.interrupt_flag = value
    
    def parse_tool_calls(self, message: Any) -> List[Dict[str, Any]]:
        """
        Parse tool calls from LLM message.
        
        Args:
            message: LLM response message object
            
        Returns:
            List of tool call dictionaries
        """
        if not hasattr(message, 'tool_calls') or not message.tool_calls:
            return []
        
        tool_calls = []
        for tc in message.tool_calls:
            tool_calls.append({
                "id": tc.id,
                "type": "function",
                "function": {
                    "name": tc.function.name,
                    "arguments": tc.function.arguments
                }
            })
        
        return tool_calls
    
    def execute_tool_call(
        self,
        tool_call_id: str,
        tool_name: str,
        tool_args_str: str,
        timeout: float = 10.0
    ) -> ToolCallResult:
        """
        Execute a single tool call.
        
        Args:
            tool_call_id: Tool call ID from LLM
            tool_name: Name of the tool to execute
            tool_args_str: JSON string with tool arguments
            timeout: Execution timeout in seconds
            
        Returns:
            ToolCallResult with execution results
        """
        self.logger.info(f"🛠️  Executing: {tool_name}({tool_args_str[:100]}...)")
        
        # Parse arguments
        try:
            tool_args = json.loads(tool_args_str)
        except json.JSONDecodeError as e:
            self.logger.error(f"❌ Failed to parse arguments for {tool_name}: {e}")
            return ToolCallResult(
                tool_call_id=tool_call_id,
                tool_name=tool_name,
                success=False,
                content=json.dumps({
                    'success': False,
                    'error': f'Invalid argument format: {e}'
                }, ensure_ascii=False),
                error=str(e)
            )
        
        # Execute tool call
        try:
            result = self.tool_executor(tool_name, tool_args, timeout)
        except Exception as e:
            self.logger.error(f"❌ Tool execution error for {tool_name}: {e}")
            return ToolCallResult(
                tool_call_id=tool_call_id,
                tool_name=tool_name,
                success=False,
                content=json.dumps({
                    'success': False,
                    'error': f'Execution error: {e}'
                }, ensure_ascii=False),
                error=str(e)
            )
        
        # Log result
        if result.get('success'):
            self.logger.info(f"✅ {tool_name} completed: {result.get('message', 'OK')[:50]}")
        else:
            self.logger.warning(f"⚠️  {tool_name} returned error: {result.get('error', 'Unknown')}")
        
        return ToolCallResult(
            tool_call_id=tool_call_id,
            tool_name=tool_name,
            success=result.get('success', False),
            content=json.dumps(result, ensure_ascii=False),
            error=result.get('error') if not result.get('success') else None
        )
    
    def build_tool_result_message(self, result: ToolCallResult) -> Dict[str, Any]:
        """
        Build message dict for tool result to add to conversation history.
        
        Args:
            result: ToolCallResult from execution
            
        Returns:
            Message dictionary for conversation history
        """
        return {
            "role": "tool",
            "tool_call_id": result.tool_call_id,
            "name": result.tool_name,
            "content": result.content
        }
    
    def build_assistant_message_with_tool_calls(
        self,
        content: str,
        tool_calls: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Build assistant message with tool calls.
        
        Args:
            content: Assistant message content
            tool_calls: List of tool call dicts
            
        Returns:
            Message dictionary for conversation history
        """
        return {
            "role": "assistant",
            "content": content,
            "tool_calls": tool_calls
        }
    
    def process_tool_calls_iteration(
        self,
        tool_calls: List[Dict[str, Any]],
        timeout: float = 10.0
    ) -> List[ToolCallResult]:
        """
        Process all tool calls in a single iteration.
        
        Args:
            tool_calls: List of tool call dictionaries
            timeout: Execution timeout per tool
            
        Returns:
            List of ToolCallResult objects
        """
        results = []
        
        for tool_call in tool_calls:
            tool_id = tool_call.get('id')
            function = tool_call.get('function', {})
            tool_name = function.get('name')
            tool_args_str = function.get('arguments', '{}')
            
            result = self.execute_tool_call(
                tool_call_id=tool_id,
                tool_name=tool_name,
                tool_args_str=tool_args_str,
                timeout=timeout
            )
            results.append(result)
        
        return results
    
    def should_continue_agent_loop(self, iteration: int, has_tool_calls: bool) -> bool:
        """
        Determine if agent loop should continue.
        
        Args:
            iteration: Current iteration number
            has_tool_calls: Whether current response has tool calls
            
        Returns:
            True if should continue, False otherwise
        """
        if self.interrupt_flag:
            self.logger.warning("🛑 Agent loop interrupted by new user request")
            self.interrupt_flag = False
            return False
        
        if iteration >= self.max_iterations:
            self.logger.warning(f"⚠️  Reached max iterations ({self.max_iterations}), stopping agent loop")
            return False
        
        if not has_tool_calls:
            # No tool calls means final answer
            return False
        
        return True
