# Voice Package Refactoring Plan

## 🎯 Objective

Refactor `rob_box_voice` package to follow vibe coding principles:
- Break large files into focused modules (<250 LOC each)
- Separate business logic from ROS infrastructure
- Make all components testable in isolation
- Improve AI-assisted development experience

## 📊 Current State Analysis

### Large Files Identified

| File | LOC | Issues | Priority |
|------|-----|--------|----------|
| dialogue_node.py | 2313 | Massive, mixed concerns | 🔴 CRITICAL |
| tts_node.py | 858 | Audio + ROS + TTS logic | 🟡 HIGH |
| command_node.py | 510 | Command parsing + execution | 🟡 HIGH |
| stt_node.py | 360 | STT + audio handling | 🟢 MEDIUM |
| audio_node.py | 359 | Audio recording logic | 🟢 MEDIUM |
| sound_node.py | 358 | Sound playback | 🟢 MEDIUM |

### Identified Concerns

#### dialogue_node.py (2313 LOC) breaks down to:
1. **LLM Client Management** (~400 LOC)
   - Multiple providers (Qwen, DeepSeek)
   - API key handling
   - Streaming response handling
   
2. **Dialogue State** (~300 LOC)
   - Wake word detection
   - Silence mode
   - Query accumulation
   - Conversation history
   
3. **Tool Call Integration** (~500 LOC)
   - MCP Tools adapter
   - Tool call parsing
   - Execution handling
   
4. **Response Processing** (~400 LOC)
   - Streaming chunks
   - Accent replacement
   - TTS formatting
   
5. **ROS Integration** (~400 LOC)
   - Subscribers/Publishers
   - Service clients
   - Parameter handling
   
6. **Error Handling & Fallback** (~300 LOC)
   - Provider fallback
   - Internet connectivity checks
   - Error recovery

## 🏗️ Target Architecture

### New Directory Structure

```
rob_box_voice/
├── core/                           # Pure Python, no ROS
│   ├── __init__.py
│   ├── dialogue_manager.py         # Dialogue state machine (200 LOC)
│   ├── command_parser.py           # Command parsing logic (150 LOC)
│   ├── speech_formatter.py         # TTS text formatting (150 LOC)
│   ├── wake_word_detector.py       # Wake word logic (100 LOC)
│   ├── query_accumulator.py        # Query accumulation (100 LOC)
│   └── README.md
│
├── llm/                            # LLM integration
│   ├── __init__.py
│   ├── client.py                   # Base LLM client (150 LOC)
│   ├── streaming_handler.py        # Streaming response (200 LOC)
│   ├── tool_call_handler.py        # Tool call processing (200 LOC)
│   ├── provider_manager.py         # Multi-provider support (200 LOC)
│   ├── providers/
│   │   ├── __init__.py
│   │   ├── base.py                 # Provider protocol (100 LOC)
│   │   ├── qwen.py                 # Qwen implementation (150 LOC)
│   │   └── deepseek.py             # DeepSeek implementation (150 LOC)
│   └── README.md
│
├── audio/                          # Audio processing
│   ├── __init__.py
│   ├── playback_manager.py         # Audio playback (150 LOC)
│   ├── recording_manager.py        # Audio recording (150 LOC)
│   ├── audio_utils.py              # Audio utilities (100 LOC)
│   └── README.md
│
├── nodes/                          # ROS nodes (thin adapters)
│   ├── __init__.py
│   ├── dialogue_node.py            # ROS wrapper (200 LOC)
│   ├── tts_node.py                 # ROS wrapper (200 LOC)
│   ├── stt_node.py                 # ROS wrapper (200 LOC)
│   ├── command_node.py             # ROS wrapper (150 LOC)
│   ├── audio_node.py               # ROS wrapper (150 LOC)
│   ├── sound_node.py               # ROS wrapper (150 LOC)
│   └── README.md
│
├── tests/
│   ├── unit/
│   │   ├── core/
│   │   │   ├── test_dialogue_manager.py
│   │   │   ├── test_command_parser.py
│   │   │   ├── test_wake_word_detector.py
│   │   │   └── ...
│   │   ├── llm/
│   │   │   ├── test_streaming_handler.py
│   │   │   ├── test_tool_call_handler.py
│   │   │   └── ...
│   │   └── audio/
│   │       └── test_playback_manager.py
│   ├── integration/
│   │   ├── test_dialogue_flow.py
│   │   └── test_tool_execution.py
│   └── fixtures/
│       ├── responses/
│       └── audio_samples/
│
└── README.md
```

## 📋 Refactoring Steps

### Phase 1: Extract Core Modules (Week 1)

#### Step 1.1: Create Core Structure
- [ ] Create `core/` directory
- [ ] Add `core/README.md`
- [ ] Add `core/__init__.py`

#### Step 1.2: Extract Dialogue Manager
**Source**: `dialogue_node.py` lines 73-200 (state management)

**Target**: `core/dialogue_manager.py`

**Responsibilities**:
- Wake word detection
- Silence mode management
- Query accumulation
- Conversation history

**Tests**: `tests/unit/core/test_dialogue_manager.py`

```python
# Example interface
class DialogueManager:
    def is_wake_word(self, text: str) -> bool:
        """Check if text contains wake word"""
        
    def should_respond(self, text: str) -> bool:
        """Check if should respond based on state"""
        
    def accumulate_query(self, text: str) -> Optional[str]:
        """Accumulate query, return when ready"""
        
    def add_to_history(self, role: str, content: str):
        """Add message to conversation history"""
```

#### Step 1.3: Extract Command Parser
**Source**: `command_node.py` lines 50-300

**Target**: `core/command_parser.py`

**Responsibilities**:
- Parse command strings
- Extract parameters
- Validate commands

**Tests**: `tests/unit/core/test_command_parser.py`

#### Step 1.4: Extract Speech Formatter
**Source**: `dialogue_node.py` lines 400-550 (accent replacement)

**Target**: `core/speech_formatter.py`

**Responsibilities**:
- Format text for TTS
- Apply accent replacement
- Handle special characters

**Tests**: `tests/unit/core/test_speech_formatter.py`

### Phase 2: Extract LLM Layer (Week 2)

#### Step 2.1: Create LLM Structure
- [ ] Create `llm/` directory
- [ ] Add `llm/README.md`
- [ ] Define `llm/providers/base.py` protocol

#### Step 2.2: Extract Provider Manager
**Source**: `dialogue_node.py` lines 56-98 (provider config)

**Target**: `llm/provider_manager.py`

**Responsibilities**:
- Manage multiple LLM providers
- Handle provider fallback
- API key management

**Tests**: `tests/unit/llm/test_provider_manager.py`

#### Step 2.3: Extract Streaming Handler
**Source**: `dialogue_node.py` lines 300-600

**Target**: `llm/streaming_handler.py`

**Responsibilities**:
- Handle streaming responses
- Parse chunks
- Yield content and tool calls

**Tests**: `tests/unit/llm/test_streaming_handler.py`

#### Step 2.4: Extract Tool Call Handler
**Source**: `dialogue_node.py` lines 600-900

**Target**: `llm/tool_call_handler.py`

**Responsibilities**:
- Parse tool calls
- Execute via MCP adapter
- Format results

**Tests**: `tests/unit/llm/test_tool_call_handler.py`

### Phase 3: Refactor Nodes (Week 3)

#### Step 3.1: Refactor dialogue_node.py
- [ ] Import from `core/` and `llm/`
- [ ] Reduce to ROS adapter (~200 LOC)
- [ ] Keep backward compatibility
- [ ] Add integration tests

#### Step 3.2: Refactor tts_node.py
- [ ] Extract audio logic to `audio/playback_manager.py`
- [ ] Reduce node to ROS wrapper (~200 LOC)
- [ ] Add tests

#### Step 3.3: Refactor command_node.py
- [ ] Use `core/command_parser.py`
- [ ] Reduce to ROS wrapper (~150 LOC)
- [ ] Add tests

### Phase 4: Testing & Documentation (Week 4)

#### Step 4.1: Comprehensive Tests
- [ ] Unit tests for all core modules (80%+ coverage)
- [ ] Integration tests for flows
- [ ] Add test fixtures

#### Step 4.2: Documentation
- [ ] Update package README.md
- [ ] Add module READMEs
- [ ] Create usage examples
- [ ] Update launch files documentation

## 🧪 Testing Strategy

### Unit Tests (No ROS)

```python
# tests/unit/core/test_dialogue_manager.py
import pytest
from rob_box_voice.core.dialogue_manager import DialogueManager


def test_wake_word_detection():
    manager = DialogueManager(wake_words=["робок", "робот"])
    
    assert manager.is_wake_word("робок, привет") is True
    assert manager.is_wake_word("робот, как дела") is True
    assert manager.is_wake_word("привет") is False


def test_silence_mode():
    manager = DialogueManager()
    manager.enable_silence(duration=60)  # 60 seconds
    
    assert manager.should_respond("робок, привет") is False
    
    # Simulate time passing
    manager.silence_until = None
    assert manager.should_respond("робок, привет") is True
```

### Integration Tests (With Mocks)

```python
# tests/integration/test_dialogue_flow.py
import pytest
from unittest.mock import Mock, patch
from rob_box_voice.nodes.dialogue_node import DialogueNode


@patch('rob_box_voice.llm.client.OpenAI')
def test_full_dialogue_flow(mock_openai):
    # Setup mock LLM response
    mock_response = Mock()
    mock_response.choices[0].delta.content = "Привет!"
    mock_openai.return_value.chat.completions.create.return_value = [mock_response]
    
    # Create node (in test mode)
    node = DialogueNode(test_mode=True)
    
    # Simulate user input
    result = node.process_user_input("робок, привет")
    
    assert "Привет" in result
```

## 📝 Migration Checklist

### For Each Module

- [ ] Extract to new file
- [ ] Write unit tests (80%+ coverage)
- [ ] Add module README.md
- [ ] Update imports in calling code
- [ ] Add integration test
- [ ] Update documentation

### For Each Node

- [ ] Refactor to use core modules
- [ ] Reduce to <200 LOC
- [ ] Keep backward compatibility
- [ ] Add ROS integration tests
- [ ] Update launch files if needed

## 🎯 Success Criteria

- [ ] No file >300 LOC
- [ ] All core logic testable without ROS
- [ ] 80%+ test coverage for core/llm/audio
- [ ] All nodes are thin adapters (<200 LOC)
- [ ] Package README.md updated
- [ ] All tests passing
- [ ] Backward compatible (no breaking changes)

## 🚀 Benefits After Refactoring

1. **For AI Development**:
   - LLM can understand entire modules
   - Clear boundaries reduce hallucinations
   - Easy to prompt for specific changes

2. **For Human Developers**:
   - Easy to locate functionality
   - Safe refactoring with tests
   - Clear dependency graph

3. **For Maintenance**:
   - Easy to add new LLM providers
   - Simple to test edge cases
   - Clear upgrade paths

## 📚 References

- [VIBE_CODING_ARCHITECTURE.md](./VIBE_CODING_ARCHITECTURE.md)
- [Current dialogue_node.py](../../src/rob_box_voice/rob_box_voice/dialogue_node.py)
- [Current package structure](../../src/rob_box_voice/)

---

**Status**: Planning  
**Start Date**: TBD  
**Estimated Duration**: 4 weeks  
**Owner**: TBD
