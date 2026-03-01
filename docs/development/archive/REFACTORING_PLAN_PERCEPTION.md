# Perception Package Refactoring Plan

## 🎯 Objective

Refactor `rob_box_perception` package to follow vibe coding principles:
- Separate AI/LLM logic from ROS infrastructure
- Break large nodes into focused modules
- Make all components testable in isolation
- Improve maintainability for AI-assisted development

## 📊 Current State Analysis

### Large Files Identified

| File | LOC | Issues | Priority |
|------|-----|--------|----------|
| reflection_node.py | 877 | LLM + event detection + ROS | 🔴 CRITICAL |
| context_aggregator_node.py | 719 | Context building + ROS | 🔴 CRITICAL |
| long_term_memory.py | 208 | Memory management | 🟢 MEDIUM |
| startup_greeting_node.py | 181 | Mixed concerns | 🟢 MEDIUM |
| health_monitor.py | 147 | Good size, minor cleanup | 🟢 LOW |

### Identified Concerns

#### reflection_node.py (877 LOC) breaks down to:
1. **Event Detection** (~250 LOC)
   - State tracking
   - Edge detection
   - Event filtering
   
2. **LLM Integration** (~200 LOC)
   - API client
   - Prompt management
   - Response parsing
   
3. **Dialogue Management** (~150 LOC)
   - User speech handling
   - Silence mode
   - Response timing
   
4. **ROS Integration** (~200 LOC)
   - Subscribers/Publishers
   - Message handling
   - Parameters
   
5. **Memory Management** (~80 LOC)
   - Recent thoughts
   - Context tracking

#### context_aggregator_node.py (719 LOC) breaks down to:
1. **Context Building** (~300 LOC)
   - Data collection from topics
   - Context formatting
   - Event generation
   
2. **Health Monitoring** (~150 LOC)
   - System health checks
   - Battery monitoring
   - Sensor status
   
3. **ROS Integration** (~200 LOC)
   - Multiple subscribers
   - Publishers
   - Service clients
   
4. **State Management** (~70 LOC)
   - Last message tracking
   - Timestamp handling

## 🏗️ Target Architecture

### New Directory Structure

```
rob_box_perception/
├── core/                           # Pure Python, no ROS
│   ├── __init__.py
│   ├── context_builder.py          # Context aggregation (200 LOC)
│   ├── event_detector.py           # Event detection (200 LOC)
│   ├── memory_manager.py           # Memory management (150 LOC)
│   ├── health_analyzer.py          # Health analysis (150 LOC)
│   └── README.md
│
├── llm/                            # LLM integration
│   ├── __init__.py
│   ├── reflection_client.py        # LLM client for reflection (150 LOC)
│   ├── prompt_manager.py           # Prompt templates (200 LOC)
│   ├── response_parser.py          # Parse LLM responses (100 LOC)
│   └── README.md
│
├── models/                         # Data models
│   ├── __init__.py
│   ├── context_event.py            # Context/Event models (100 LOC)
│   ├── health_status.py            # Health models (80 LOC)
│   └── README.md
│
├── nodes/                          # ROS nodes (thin adapters)
│   ├── __init__.py
│   ├── reflection_node.py          # ROS wrapper (200 LOC)
│   ├── context_aggregator_node.py  # ROS wrapper (200 LOC)
│   ├── health_monitor.py           # ROS wrapper (150 LOC)
│   ├── startup_greeting_node.py    # ROS wrapper (100 LOC)
│   └── README.md
│
├── utils/                          # Shared utilities
│   ├── __init__.py
│   ├── time_provider.py            # Time utilities (100 LOC)
│   ├── internet_monitor.py         # Internet check (100 LOC)
│   ├── node_monitor.py             # Node monitoring (100 LOC)
│   └── long_term_memory.py         # File-based memory (200 LOC)
│
├── tests/
│   ├── unit/
│   │   ├── core/
│   │   │   ├── test_context_builder.py
│   │   │   ├── test_event_detector.py
│   │   │   └── test_memory_manager.py
│   │   ├── llm/
│   │   │   ├── test_reflection_client.py
│   │   │   └── test_prompt_manager.py
│   │   └── models/
│   │       └── test_context_event.py
│   ├── integration/
│   │   ├── test_reflection_flow.py
│   │   └── test_context_aggregation.py
│   └── fixtures/
│       ├── contexts/
│       └── events/
│
└── README.md
```

## 📋 Refactoring Steps

### Phase 1: Extract Core Modules (Week 1)

#### Step 1.1: Create Core Structure
- [ ] Create `core/` directory
- [ ] Add `core/README.md`
- [ ] Add `core/__init__.py`

#### Step 1.2: Extract Context Builder
**Source**: `context_aggregator_node.py` lines 100-400

**Target**: `core/context_builder.py`

**Responsibilities**:
- Aggregate data from multiple sources
- Format context for events
- Filter relevant information
- Generate context summaries

**Interface**:
```python
from typing import Dict, Any, List
from .models import ContextEvent


class ContextBuilder:
    """Builds context from sensor data and system state"""
    
    def add_data(self, source: str, data: Dict[str, Any]) -> None:
        """Add data from a source"""
        
    def build_event(self, event_type: str) -> ContextEvent:
        """Build context event from accumulated data"""
        
    def get_summary(self) -> str:
        """Get human-readable context summary"""
        
    def clear(self) -> None:
        """Clear accumulated data"""
```

**Tests**: `tests/unit/core/test_context_builder.py`

#### Step 1.3: Extract Event Detector
**Source**: `reflection_node.py` lines 84-350

**Target**: `core/event_detector.py`

**Responsibilities**:
- Track event states
- Detect state changes (edges)
- Filter significant events
- Apply cooldowns

**Interface**:
```python
from typing import Optional, Dict, Any
from enum import Enum


class EventType(Enum):
    HEALTH_CHANGE = "health_change"
    VISION_CHANGE = "vision_change"
    PERIODIC_CHECK = "periodic_check"


class EventDetector:
    """Detects significant events from context changes"""
    
    def __init__(self, cooldown: float = 60.0):
        """Initialize with cooldown period"""
        
    def process_context(self, context: Dict[str, Any]) -> Optional[EventType]:
        """
        Process context and detect events
        
        Returns:
            EventType if significant event detected, None otherwise
        """
        
    def should_react(self, event_type: EventType) -> bool:
        """Check if should react to event (cooldown logic)"""
```

**Tests**: `tests/unit/core/test_event_detector.py`

#### Step 1.4: Extract Memory Manager
**Source**: `reflection_node.py` lines 80-82 + helper methods

**Target**: `core/memory_manager.py`

**Responsibilities**:
- Manage recent thoughts
- Store conversation history
- Retrieve relevant memories
- Prune old memories

**Interface**:
```python
from typing import List
from dataclasses import dataclass


@dataclass
class Memory:
    content: str
    timestamp: float
    memory_type: str  # thought, speech, event


class MemoryManager:
    """Manages short-term memory for reflection"""
    
    def __init__(self, max_thoughts: int = 10):
        """Initialize with memory limits"""
        
    def add_thought(self, thought: str) -> None:
        """Add internal thought"""
        
    def add_speech(self, speech: str) -> None:
        """Add robot speech"""
        
    def get_recent(self, limit: int = 5) -> List[Memory]:
        """Get recent memories"""
        
    def clear_old(self, max_age_seconds: float) -> None:
        """Clear memories older than threshold"""
```

**Tests**: `tests/unit/core/test_memory_manager.py`

### Phase 2: Extract LLM Layer (Week 2)

#### Step 2.1: Create LLM Structure
- [ ] Create `llm/` directory
- [ ] Add `llm/README.md`

#### Step 2.2: Extract Reflection Client
**Source**: `reflection_node.py` lines 94-150 (LLM setup)

**Target**: `llm/reflection_client.py`

**Responsibilities**:
- LLM API communication
- Handle multiple providers
- Retry logic
- Error handling

**Interface**:
```python
from typing import Optional, List, Dict, Any


class ReflectionClient:
    """LLM client for reflection system"""
    
    def __init__(self, provider: str = "qwen", api_key: Optional[str] = None):
        """Initialize LLM client"""
        
    def generate_thought(
        self, 
        context: str, 
        recent_memories: List[str]
    ) -> str:
        """Generate internal thought based on context"""
        
    def generate_response(
        self, 
        user_question: str, 
        context: str
    ) -> str:
        """Generate response to user question"""
        
    def is_available(self) -> bool:
        """Check if LLM service is available"""
```

**Tests**: `tests/unit/llm/test_reflection_client.py`

#### Step 2.3: Extract Prompt Manager
**Source**: Prompt loading logic from `reflection_node.py`

**Target**: `llm/prompt_manager.py`

**Responsibilities**:
- Load prompt templates
- Format prompts with data
- Manage different prompt types
- Validation

**Interface**:
```python
from typing import Dict, Any
from pathlib import Path


class PromptManager:
    """Manages LLM prompts for reflection"""
    
    def __init__(self, prompts_dir: Path):
        """Initialize with prompts directory"""
        
    def get_reflection_prompt(
        self, 
        context: Dict[str, Any],
        recent_memories: List[str]
    ) -> str:
        """Format reflection prompt"""
        
    def get_response_prompt(
        self,
        user_question: str,
        context: Dict[str, Any]
    ) -> str:
        """Format user response prompt"""
```

**Tests**: `tests/unit/llm/test_prompt_manager.py`

### Phase 3: Create Data Models (Week 2)

#### Step 3.1: Create Models Structure
- [ ] Create `models/` directory
- [ ] Add `models/README.md`

#### Step 3.2: Create Context Event Model
**Source**: Message definitions scattered across nodes

**Target**: `models/context_event.py`

**Responsibilities**:
- Define context data structure
- Validation
- Serialization

**Interface**:
```python
from dataclasses import dataclass
from typing import Dict, Any, Optional


@dataclass
class ContextEvent:
    """Context event data model"""
    
    timestamp: float
    event_type: str
    health: Optional[Dict[str, Any]] = None
    vision: Optional[Dict[str, Any]] = None
    navigation: Optional[Dict[str, Any]] = None
    battery: Optional[float] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary"""
        
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ContextEvent':
        """Create from dictionary"""
        
    def to_text(self) -> str:
        """Convert to human-readable text"""
```

**Tests**: `tests/unit/models/test_context_event.py`

### Phase 4: Refactor Nodes (Week 3)

#### Step 4.1: Refactor reflection_node.py
**Current**: 877 LOC  
**Target**: ~200 LOC (ROS adapter only)

**New Structure**:
```python
# Thin ROS wrapper
class ReflectionNode(Node):
    def __init__(self):
        super().__init__('reflection_node')
        
        # Initialize core components
        self.event_detector = EventDetector()
        self.memory_manager = MemoryManager()
        self.llm_client = ReflectionClient()
        self.prompt_manager = PromptManager()
        
        # ROS setup
        self._setup_ros()
    
    def _setup_ros(self):
        """Setup ROS subscribers/publishers"""
        
    def _on_context_update(self, msg):
        """Handle context updates"""
        event = self.event_detector.process_context(msg.data)
        if event and self.event_detector.should_react(event):
            self._generate_thought(msg.data)
    
    def _generate_thought(self, context):
        """Generate and publish thought"""
        recent = self.memory_manager.get_recent()
        thought = self.llm_client.generate_thought(context, recent)
        self.memory_manager.add_thought(thought)
        # Publish thought
```

Steps:
- [ ] Import core modules
- [ ] Replace inline logic with module calls
- [ ] Add integration tests
- [ ] Verify backward compatibility

#### Step 4.2: Refactor context_aggregator_node.py
**Current**: 719 LOC  
**Target**: ~200 LOC

**New Structure**:
```python
class ContextAggregatorNode(Node):
    def __init__(self):
        super().__init__('context_aggregator')
        
        # Core component
        self.context_builder = ContextBuilder()
        
        # ROS setup
        self._setup_ros()
    
    def _on_health_update(self, msg):
        """Handle health data"""
        self.context_builder.add_data('health', msg.data)
    
    def _on_vision_update(self, msg):
        """Handle vision data"""
        self.context_builder.add_data('vision', msg.data)
    
    def _publish_context_event(self):
        """Build and publish context event"""
        event = self.context_builder.build_event('periodic')
        # Publish event
```

Steps:
- [ ] Import ContextBuilder
- [ ] Replace inline logic
- [ ] Add tests
- [ ] Verify functionality

### Phase 5: Testing & Documentation (Week 4)

#### Step 5.1: Comprehensive Tests
- [ ] Unit tests for all core modules (80%+ coverage)
- [ ] Unit tests for LLM layer (mock API calls)
- [ ] Integration tests for node flows
- [ ] Add test fixtures (sample contexts, events)

#### Step 5.2: Documentation
- [ ] Update package README.md
- [ ] Add module READMEs
- [ ] Create usage examples
- [ ] Document event types
- [ ] Update architecture diagrams

## 🧪 Testing Strategy

### Unit Tests (No ROS)

```python
# tests/unit/core/test_event_detector.py
import pytest
from rob_box_perception.core.event_detector import EventDetector, EventType


def test_health_change_detection():
    detector = EventDetector()
    
    # Initial state
    context1 = {'health': {'cpu': 50, 'memory': 60}}
    event = detector.process_context(context1)
    assert event is None  # First time, no change
    
    # Significant change
    context2 = {'health': {'cpu': 90, 'memory': 60}}
    event = detector.process_context(context2)
    assert event == EventType.HEALTH_CHANGE


def test_cooldown_logic():
    detector = EventDetector(cooldown=10.0)
    
    # First event
    assert detector.should_react(EventType.HEALTH_CHANGE) is True
    
    # Immediate retry (within cooldown)
    assert detector.should_react(EventType.HEALTH_CHANGE) is False
```

### Integration Tests (With ROS Mocks)

```python
# tests/integration/test_reflection_flow.py
import pytest
from unittest.mock import Mock, patch
from rob_box_perception.nodes.reflection_node import ReflectionNode


@patch('rob_box_perception.llm.reflection_client.OpenAI')
def test_context_to_thought_flow(mock_openai):
    # Setup mock
    mock_response = Mock()
    mock_response.choices[0].message.content = "Интересно, что происходит"
    mock_openai.return_value.chat.completions.create.return_value = mock_response
    
    # Create node
    node = ReflectionNode()
    
    # Simulate context update
    context = {'health': {'cpu': 80}, 'vision': {'objects': []}}
    node._on_context_update(context)
    
    # Verify thought was generated
    assert len(node.memory_manager.get_recent()) == 1
```

## 📝 Migration Checklist

### For Each Core Module

- [ ] Extract to new file
- [ ] Write comprehensive unit tests
- [ ] Add module README.md
- [ ] Document public interface
- [ ] Add usage examples

### For Each Node

- [ ] Refactor to use core modules
- [ ] Reduce to <200 LOC
- [ ] Keep backward compatibility
- [ ] Add integration tests
- [ ] Update launch files if needed

## 🎯 Success Criteria

- [ ] No file >300 LOC
- [ ] All core logic testable without ROS
- [ ] 80%+ test coverage for core/llm
- [ ] All nodes are thin adapters (<200 LOC)
- [ ] Package README.md updated
- [ ] All tests passing
- [ ] No breaking changes to ROS interfaces

## 🚀 Benefits After Refactoring

1. **For AI Development**:
   - Can work on event detection without touching ROS
   - LLM prompts isolated and testable
   - Clear module boundaries

2. **For Testing**:
   - Fast unit tests (no ROS overhead)
   - Easy to mock dependencies
   - Clear test scenarios

3. **For Maintenance**:
   - Easy to update LLM provider
   - Simple to add new event types
   - Clear separation of concerns

## 📚 References

- [VIBE_CODING_ARCHITECTURE.md](./VIBE_CODING_ARCHITECTURE.md)
- [Current reflection_node.py](../../src/rob_box_perception/rob_box_perception/reflection_node.py)
- [Current context_aggregator_node.py](../../src/rob_box_perception/rob_box_perception/context_aggregator_node.py)

---

**Status**: Planning  
**Start Date**: TBD  
**Estimated Duration**: 4 weeks  
**Owner**: TBD
