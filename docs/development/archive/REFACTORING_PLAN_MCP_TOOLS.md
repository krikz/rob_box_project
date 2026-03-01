# MCP Tools Package Refactoring Plan

## 🎯 Objective

Refactor `rob_box_mcp_tools` package to follow vibe coding principles:
- Separate async execution logic from tool definitions
- Clarify LLM adapter interfaces
- Make all components testable independently
- Improve extensibility for new tools

## 📊 Current State Analysis

### Files Identified

| File | LOC | Issues | Priority |
|------|-----|--------|----------|
| async_executor.py | 595 | Mixed concerns, complex | 🔴 CRITICAL |
| llm_adapter.py | 424 | Streaming + tool calls | 🟡 HIGH |
| base.py | 291 | Protocol definitions | 🟢 LOW |
| system.py (tool) | 359 | Tool implementation | 🟡 MEDIUM |
| navigation.py (tool) | 271 | Tool implementation | 🟡 MEDIUM |
| dialogue.py (tool) | 244 | Tool implementation | 🟡 MEDIUM |
| mcp_server.py | 237 | Server setup | 🟢 LOW |
| deepseek_adapter.py | 206 | Provider-specific | 🟢 LOW |

### Identified Concerns

#### async_executor.py (595 LOC) breaks down to:
1. **Task Management** (~200 LOC)
   - Interruptible tasks
   - Task lifecycle
   - Cancellation
   
2. **Tool Call Accumulation** (~150 LOC)
   - Streaming chunk assembly
   - JSON parsing
   - Validation
   
3. **Execution Engine** (~200 LOC)
   - Parallel execution
   - Fire-and-forget
   - Result collection
   
4. **Error Handling** (~50 LOC)
   - Timeout handling
   - Exception management

#### llm_adapter.py (424 LOC) breaks down to:
1. **Streaming Handler** (~200 LOC)
   - Chunk processing
   - Tool call extraction
   - Content assembly
   
2. **Tool Execution** (~150 LOC)
   - Tool call parsing
   - Parameter validation
   - Result formatting
   
3. **Registry Integration** (~75 LOC)
   - Tool lookup
   - Registry communication

## 🏗️ Target Architecture

### New Directory Structure

```
rob_box_mcp_tools/
├── core/                           # Pure Python, no external deps
│   ├── __init__.py
│   ├── protocol.py                 # Tool protocols (150 LOC)
│   ├── registry.py                 # Tool registry (moved, 130 LOC)
│   ├── executor.py                 # Execution engine (200 LOC)
│   ├── task_manager.py             # Task lifecycle (150 LOC)
│   └── README.md
│
├── streaming/                      # Streaming response handling
│   ├── __init__.py
│   ├── chunk_processor.py          # Process chunks (150 LOC)
│   ├── tool_accumulator.py         # Accumulate tool calls (150 LOC)
│   ├── content_assembler.py        # Assemble content (100 LOC)
│   └── README.md
│
├── adapters/                       # LLM provider adapters
│   ├── __init__.py
│   ├── base.py                     # Base adapter (150 LOC)
│   ├── openai_compatible.py        # OpenAI-style APIs (200 LOC)
│   ├── qwen.py                     # Qwen specifics (100 LOC)
│   ├── deepseek.py                 # DeepSeek specifics (100 LOC)
│   └── README.md
│
├── tools/                          # Tool implementations (unchanged)
│   ├── __init__.py
│   ├── dialogue.py                 # (244 LOC)
│   ├── mapping.py                  # (194 LOC)
│   ├── animation.py                # (176 LOC)
│   ├── navigation.py               # (271 LOC)
│   ├── system.py                   # (359 LOC) - consider splitting
│   ├── perception.py               # (102 LOC)
│   ├── sound.py                    # (87 LOC)
│   └── README.md
│
├── server/                         # MCP server setup
│   ├── __init__.py
│   ├── mcp_server.py               # Server (moved, 200 LOC)
│   └── README.md
│
├── tests/
│   ├── unit/
│   │   ├── core/
│   │   │   ├── test_executor.py
│   │   │   ├── test_task_manager.py
│   │   │   └── test_registry.py
│   │   ├── streaming/
│   │   │   ├── test_chunk_processor.py
│   │   │   └── test_tool_accumulator.py
│   │   └── adapters/
│   │       └── test_openai_compatible.py
│   ├── integration/
│   │   ├── test_tool_execution.py
│   │   └── test_streaming_flow.py
│   └── fixtures/
│       ├── tool_calls/
│       └── streaming_chunks/
│
└── README.md
```

## 📋 Refactoring Steps

### Phase 1: Extract Core Execution Logic (Week 1)

#### Step 1.1: Create Core Structure
- [ ] Create `core/` directory
- [ ] Add `core/README.md`
- [ ] Move `base.py` → `core/protocol.py` (keep base.py as import wrapper)

#### Step 1.2: Extract Task Manager
**Source**: `async_executor.py` lines 22-50 (InterruptibleTask)

**Target**: `core/task_manager.py`

**Responsibilities**:
- Manage task lifecycle
- Handle cancellation
- Track task state
- Timeout management

**Interface**:
```python
import asyncio
from typing import Optional, Callable, Any
from dataclasses import dataclass
from enum import Enum


class TaskState(Enum):
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    CANCELLED = "cancelled"
    FAILED = "failed"


@dataclass
class Task:
    """Managed async task"""
    
    task_id: str
    task: asyncio.Task
    tool_name: str
    state: TaskState
    created_at: float
    completed_at: Optional[float] = None
    result: Optional[Any] = None
    error: Optional[Exception] = None


class TaskManager:
    """Manages async task execution and lifecycle"""
    
    def __init__(self):
        """Initialize task manager"""
        
    async def create_task(
        self,
        coro: Callable,
        tool_name: str,
        timeout: Optional[float] = None
    ) -> Task:
        """Create and track a new task"""
        
    async def cancel_task(self, task_id: str) -> bool:
        """Cancel a running task"""
        
    def get_task(self, task_id: str) -> Optional[Task]:
        """Get task by ID"""
        
    def get_running_tasks(self) -> List[Task]:
        """Get all running tasks"""
```

**Tests**: `tests/unit/core/test_task_manager.py`

#### Step 1.3: Extract Executor
**Source**: `async_executor.py` lines 200-595 (AsyncExecutor)

**Target**: `core/executor.py`

**Responsibilities**:
- Execute tools in parallel
- Handle different execution types (INSTANT, LONG, ASYNC)
- Collect results
- Error handling

**Interface**:
```python
from typing import List, Dict, Any
from .protocol import Tool, ToolExecutionType


class Executor:
    """Executes tools with different strategies"""
    
    def __init__(self, task_manager: TaskManager):
        """Initialize with task manager"""
        
    async def execute_tools(
        self,
        tool_calls: List[Dict[str, Any]],
        registry: 'ToolRegistry'
    ) -> List[Dict[str, Any]]:
        """
        Execute multiple tool calls
        
        Returns:
            List of results (one per tool call)
        """
        
    async def execute_single(
        self,
        tool_name: str,
        parameters: Dict[str, Any],
        execution_type: ToolExecutionType
    ) -> Any:
        """Execute single tool"""
```

**Tests**: `tests/unit/core/test_executor.py`

#### Step 1.4: Move Registry
**Source**: `registry.py`

**Target**: `core/registry.py`

**Changes**:
- Minor cleanup only
- Add better type hints
- Keep backward compatibility with import from root

**Tests**: Update existing tests location

### Phase 2: Extract Streaming Logic (Week 2)

#### Step 2.1: Create Streaming Structure
- [ ] Create `streaming/` directory
- [ ] Add `streaming/README.md`

#### Step 2.2: Extract Tool Accumulator
**Source**: `async_executor.py` lines 52-150 (ToolCallAccumulator)

**Target**: `streaming/tool_accumulator.py`

**Responsibilities**:
- Accumulate streaming tool call chunks
- Parse JSON arguments
- Validate completeness
- Handle malformed data

**Interface**:
```python
from typing import List, Dict, Any, Optional


class ToolCallAccumulator:
    """Accumulates tool calls from streaming chunks"""
    
    def __init__(self):
        """Initialize accumulator"""
        
    def add_chunk(self, delta_tool_calls: List[Any]) -> None:
        """Add chunk from streaming response"""
        
    def get_complete_tool_calls(self) -> List[Dict[str, Any]]:
        """Get complete tool calls"""
        
    def is_complete(self, index: int) -> bool:
        """Check if tool call at index is complete"""
        
    def clear(self) -> None:
        """Clear accumulator state"""
```

**Tests**: `tests/unit/streaming/test_tool_accumulator.py`

#### Step 2.3: Extract Chunk Processor
**Source**: `llm_adapter.py` streaming logic

**Target**: `streaming/chunk_processor.py`

**Responsibilities**:
- Process individual chunks
- Extract content and tool calls
- Handle different chunk types
- Error recovery

**Interface**:
```python
from typing import Optional, Dict, Any
from dataclasses import dataclass


@dataclass
class ProcessedChunk:
    """Processed chunk data"""
    content: Optional[str] = None
    tool_calls: Optional[List[Any]] = None
    finish_reason: Optional[str] = None
    is_final: bool = False


class ChunkProcessor:
    """Process streaming response chunks"""
    
    def process(self, chunk: Any) -> ProcessedChunk:
        """Process single chunk"""
        
    def validate(self, chunk: Any) -> bool:
        """Validate chunk format"""
```

**Tests**: `tests/unit/streaming/test_chunk_processor.py`

### Phase 3: Refactor Adapters (Week 3)

#### Step 3.1: Create Adapters Structure
- [ ] Create `adapters/` directory
- [ ] Add `adapters/README.md`

#### Step 3.2: Create Base Adapter
**Source**: Common logic from `llm_adapter.py` and `deepseek_adapter.py`

**Target**: `adapters/base.py`

**Responsibilities**:
- Define adapter interface
- Common initialization
- Error handling patterns

**Interface**:
```python
from abc import ABC, abstractmethod
from typing import List, Dict, Any, AsyncIterator


class LLMAdapter(ABC):
    """Base class for LLM adapters"""
    
    def __init__(self, api_key: str, base_url: str):
        """Initialize adapter"""
        
    @abstractmethod
    async def stream_chat(
        self,
        messages: List[Dict[str, str]],
        tools: List[Dict[str, Any]],
        **kwargs
    ) -> AsyncIterator[ProcessedChunk]:
        """Stream chat completion with tools"""
        
    @abstractmethod
    def format_tool_result(
        self,
        tool_call_id: str,
        result: Any
    ) -> Dict[str, Any]:
        """Format tool result for next request"""
```

**Tests**: `tests/unit/adapters/test_base.py`

#### Step 3.3: Create OpenAI Compatible Adapter
**Source**: Most of `llm_adapter.py`

**Target**: `adapters/openai_compatible.py`

**Responsibilities**:
- Implement OpenAI-style streaming
- Handle tool calls
- Support multiple compatible providers

**Tests**: `tests/unit/adapters/test_openai_compatible.py`

#### Step 3.4: Refactor Provider-Specific Adapters
- [ ] Update `deepseek_adapter.py` to extend base
- [ ] Create `qwen.py` if needed for Qwen-specific logic
- [ ] Keep backward compatibility

### Phase 4: Tool Refactoring (Optional, Week 4)

#### Step 4.1: Split Large Tools
**Target**: `tools/system.py` (359 LOC)

**Could split into**:
- `tools/system/process.py` - Process management
- `tools/system/info.py` - System information
- `tools/system/control.py` - System control

**Only if makes sense logically**

#### Step 4.2: Add Tool READMEs
- [ ] Document each tool's purpose
- [ ] Add usage examples
- [ ] Document parameters
- [ ] Add test coverage info

### Phase 5: Testing & Documentation (Week 4)

#### Step 5.1: Comprehensive Tests
- [ ] Unit tests for all core modules (80%+ coverage)
- [ ] Unit tests for streaming (mock chunks)
- [ ] Integration tests for tool execution flow
- [ ] Add test fixtures for common scenarios

Example unit test:
```python
# tests/unit/streaming/test_tool_accumulator.py
import pytest
from rob_box_mcp_tools.streaming.tool_accumulator import ToolCallAccumulator


def test_accumulate_streaming_chunks():
    accumulator = ToolCallAccumulator()
    
    # Chunk 1: Initial tool call
    accumulator.add_chunk([
        type('obj', (), {
            'index': 0,
            'id': 'call_123',
            'type': 'function',
            'function': type('obj', (), {
                'name': 'play_animation',
                'arguments': ''
            })()
        })()
    ])
    
    # Chunk 2: Partial arguments
    accumulator.add_chunk([
        type('obj', (), {
            'index': 0,
            'function': type('obj', (), {
                'arguments': '{"emot'
            })()
        })()
    ])
    
    # Chunk 3: Rest of arguments
    accumulator.add_chunk([
        type('obj', (), {
            'index': 0,
            'function': type('obj', (), {
                'arguments': 'ion": "радость"}'
            })()
        })()
    ])
    
    # Get complete tool calls
    calls = accumulator.get_complete_tool_calls()
    
    assert len(calls) == 1
    assert calls[0]['id'] == 'call_123'
    assert calls[0]['function']['name'] == 'play_animation'
    assert calls[0]['function']['arguments'] == '{"emotion": "радость"}'
```

Example integration test:
```python
# tests/integration/test_tool_execution.py
import pytest
from rob_box_mcp_tools.core.registry import ToolRegistry
from rob_box_mcp_tools.core.executor import Executor
from rob_box_mcp_tools.core.task_manager import TaskManager


@pytest.mark.asyncio
async def test_parallel_tool_execution():
    # Setup
    registry = ToolRegistry()
    task_manager = TaskManager()
    executor = Executor(task_manager)
    
    # Register test tools
    @registry.register_tool(execution_type="INSTANT")
    async def test_tool_1(param: str) -> str:
        return f"Result 1: {param}"
    
    @registry.register_tool(execution_type="INSTANT")
    async def test_tool_2(param: str) -> str:
        return f"Result 2: {param}"
    
    # Execute tools in parallel
    tool_calls = [
        {"name": "test_tool_1", "parameters": {"param": "A"}},
        {"name": "test_tool_2", "parameters": {"param": "B"}},
    ]
    
    results = await executor.execute_tools(tool_calls, registry)
    
    assert len(results) == 2
    assert "Result 1: A" in str(results[0])
    assert "Result 2: B" in str(results[1])
```

#### Step 5.2: Documentation
- [ ] Update package README.md with new structure
- [ ] Add module READMEs for core/, streaming/, adapters/
- [ ] Create architecture diagram
- [ ] Document adding new tools
- [ ] Document adding new LLM providers
- [ ] Add examples for common use cases

## 📝 Migration Checklist

### For Each Core Module

- [ ] Extract to new file
- [ ] Write comprehensive unit tests
- [ ] Add module README.md
- [ ] Document public interface
- [ ] Add usage examples
- [ ] Keep backward compatibility

### For Each Adapter

- [ ] Extend base adapter
- [ ] Implement required methods
- [ ] Add provider-specific handling
- [ ] Write unit tests (mock API)
- [ ] Document differences from base

## 🎯 Success Criteria

- [ ] No file >300 LOC
- [ ] All core logic testable without external services
- [ ] 80%+ test coverage for core/streaming
- [ ] Clear separation between execution engine and LLM adapters
- [ ] Easy to add new tools
- [ ] Easy to add new LLM providers
- [ ] Package README.md updated
- [ ] All tests passing
- [ ] Backward compatible

## 🚀 Benefits After Refactoring

1. **For AI Development**:
   - Clear module boundaries
   - Easy to understand each component
   - Simple to extend with new providers

2. **For Testing**:
   - Fast unit tests (no network calls)
   - Easy to mock dependencies
   - Clear test scenarios

3. **For Maintenance**:
   - Easy to update execution strategy
   - Simple to add new LLM providers
   - Clear separation of concerns
   - Easy to debug streaming issues

4. **For Extension**:
   - Simple tool registration
   - Clear adapter interface
   - Documented patterns

## 📚 References

- [VIBE_CODING_ARCHITECTURE.md](./VIBE_CODING_ARCHITECTURE.md)
- [Current async_executor.py](../../src/rob_box_mcp_tools/rob_box_mcp_tools/async_executor.py)
- [Current llm_adapter.py](../../src/rob_box_mcp_tools/rob_box_mcp_tools/llm_adapter.py)

---

**Status**: Planning  
**Start Date**: TBD  
**Estimated Duration**: 4 weeks  
**Owner**: TBD
