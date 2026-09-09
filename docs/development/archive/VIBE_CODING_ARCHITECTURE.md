# Vibe Coding Architecture Guide

## 🎯 Purpose

This document defines the architectural principles and patterns for the Rob Box AI Agent system to enable effective **vibe coding** - a development approach where AI and humans collaborate efficiently through clear, modular code structures.

## 🚨 Core Problem

The current AI agent codebase has grown organically and suffers from:

1. **Large monolithic files** (500-2300+ lines) that exceed LLM context limits
2. **Mixed responsibilities** making it hard to reason about individual components
3. **Insufficient test coverage** making refactoring risky
4. **Lack of clear boundaries** between concerns
5. **Poor discoverability** - hard to find where specific functionality lives

This leads to:
- AI hallucinations due to context overflow
- Difficulty maintaining consistent behavior
- High cognitive load for human developers
- Fragile code that breaks easily

## 📚 Vibe Coding Principles (Applied)

Based on industry best practices, we adopt these principles:

### 1. **Single Responsibility Modules** (SRM)
- **Rule**: Each file should have ONE clear purpose
- **Limit**: Maximum 250-300 lines per file
- **Benefit**: LLM can load entire file in context and understand it completely

### 2. **Clear Boundaries** (CB)
- **Rule**: Define explicit interfaces between modules
- **Pattern**: Use Protocol/ABC for contracts
- **Benefit**: Easy to replace implementations, test in isolation

### 3. **Focused Documentation** (FD)
- **Rule**: Each module has README.md with:
  - Purpose (1-2 sentences)
  - Public interface
  - Usage example
  - Test coverage
- **Benefit**: AI can quickly understand what module does

### 4. **Test-First Thinking** (TFT)
- **Rule**: Every module must be testable without dependencies
- **Pattern**: Dependency injection, mocks
- **Benefit**: Confidence in refactoring, clear specifications

### 5. **Layered Architecture** (LA)
- **Rule**: Separate concerns into layers
- **Layers**:
  - **Core**: Business logic (no ROS, no I/O)
  - **Adapters**: ROS nodes, API clients
  - **Infrastructure**: Configuration, utilities
- **Benefit**: Test core logic without ROS runtime

### 6. **Explicit State Management** (ESM)
- **Rule**: State changes are explicit and traceable
- **Pattern**: State machines, events
- **Benefit**: Easy to debug, understand flow

## 🏗️ Target Architecture

### Directory Structure

```
src/
├── rob_box_mcp_tools/          # MCP Tool Integration
│   ├── core/                   # Core logic (no ROS)
│   │   ├── tool_registry.py    # Tool registration (<200 LOC)
│   │   ├── executor.py         # Execution logic (<250 LOC)
│   │   ├── protocol.py         # Interfaces/protocols (<100 LOC)
│   │   └── README.md           # Core layer docs
│   ├── adapters/               # External integrations
│   │   ├── llm_client.py       # LLM API adapter (<200 LOC)
│   │   ├── ros_bridge.py       # ROS integration (<150 LOC)
│   │   └── README.md
│   ├── tools/                  # Individual tools (unchanged)
│   │   ├── dialogue.py
│   │   ├── navigation.py
│   │   └── ...
│   ├── tests/                  # Comprehensive tests
│   │   ├── core/
│   │   ├── adapters/
│   │   └── integration/
│   └── README.md               # Package overview
│
├── rob_box_perception/         # Perception & Context
│   ├── core/                   # Business logic
│   │   ├── context_builder.py  # Context aggregation (<200 LOC)
│   │   ├── event_detector.py   # Event detection (<200 LOC)
│   │   ├── memory_manager.py   # Memory management (<150 LOC)
│   │   └── README.md
│   ├── nodes/                  # ROS nodes (thin adapters)
│   │   ├── health_monitor.py   # (<150 LOC)
│   │   ├── context_aggregator.py  # (<200 LOC)
│   │   ├── reflection_node.py  # (<250 LOC) - refactored
│   │   └── README.md
│   ├── llm/                    # LLM integration
│   │   ├── client.py           # API client (<150 LOC)
│   │   ├── prompt_manager.py   # Prompt handling (<200 LOC)
│   │   └── README.md
│   ├── tests/
│   └── README.md
│
└── rob_box_voice/              # Voice Assistant
    ├── core/                   # Business logic
    │   ├── dialogue_manager.py  # Dialogue state (<250 LOC)
    │   ├── command_parser.py    # Command parsing (<150 LOC)
    │   ├── speech_formatter.py  # Output formatting (<150 LOC)
    │   └── README.md
    ├── nodes/                  # ROS nodes (thin)
    │   ├── stt_node.py          # (<200 LOC)
    │   ├── tts_node.py          # (<200 LOC)
    │   ├── dialogue_node.py     # (<250 LOC) - refactored
    │   ├── command_node.py      # (<200 LOC)
    │   └── README.md
    ├── llm/                    # LLM integration
    │   ├── streaming_client.py  # Streaming support (<200 LOC)
    │   ├── tool_call_handler.py # Tool call logic (<200 LOC)
    │   └── README.md
    ├── audio/                  # Audio processing
    │   ├── playback_manager.py  # Audio playback (<150 LOC)
    │   ├── audio_utils.py       # Utilities (<100 LOC)
    │   └── README.md
    ├── tests/
    └── README.md
```

### File Size Guidelines

| Type | Max LOC | Rationale |
|------|---------|-----------|
| Protocol/Interface | 100 | Pure definitions |
| Utility/Helper | 150 | Single-purpose functions |
| Core Logic | 250 | Complete business logic unit |
| ROS Node | 200 | Thin adapter to core logic |
| Integration | 300 | Can be complex but focused |

## 🔄 Refactoring Strategy

### Phase 1: Extract Core Logic (No Breaking Changes)
1. Create new `core/` directories
2. Extract business logic from large files
3. Keep original files as wrappers (backward compatible)
4. Add tests for core modules

### Phase 2: Refactor Nodes (Gradual)
1. Update nodes to use core modules
2. Remove duplication
3. Add integration tests
4. Deprecate old code paths

### Phase 3: Documentation & Cleanup
1. Add README.md to all modules
2. Remove deprecated code
3. Update main docs
4. Create AI prompt examples

## 📋 Module Template

Every new module should follow this template:

```python
#!/usr/bin/env python3
"""
<module_name>.py - <One-line purpose>

Purpose:
    <2-3 sentences describing what this module does>

Public Interface:
    - <Class/Function 1>: <Brief description>
    - <Class/Function 2>: <Brief description>

Example:
    >>> from rob_box_x.core import ModuleName
    >>> module = ModuleName()
    >>> result = module.do_something()

Tests:
    See: tests/core/test_<module_name>.py

Dependencies:
    - <Package 1> (why)
    - <Package 2> (why)
"""

# Standard library imports
import ...

# Third-party imports
import ...

# Local imports
from .protocol import ...


class ModuleName:
    """
    <Detailed class description>
    
    Attributes:
        attr1: <Description>
        attr2: <Description>
    
    Methods:
        method1: <Description>
        method2: <Description>
    """
    
    def __init__(self):
        """Initialize module"""
        pass
    
    def public_method(self, param: str) -> str:
        """
        <Method description>
        
        Args:
            param: <Description>
        
        Returns:
            <Description>
        
        Raises:
            ValueError: <When>
        """
        return self._private_method(param)
    
    def _private_method(self, param: str) -> str:
        """Internal implementation"""
        return param.upper()
```

## 🧪 Testing Strategy

### Test Structure
```
tests/
├── unit/                    # Fast, isolated tests
│   ├── core/               # Core logic tests (no mocks needed)
│   ├── adapters/           # Adapter tests (mock external services)
│   └── nodes/              # Node tests (mock ROS)
├── integration/             # Component interaction tests
│   ├── test_dialogue_flow.py
│   └── test_tool_execution.py
└── fixtures/                # Shared test data
    ├── prompts/
    └── responses/
```

### Test Guidelines
- **Unit tests**: Test single module in isolation
- **Integration tests**: Test module interactions
- **Coverage target**: 80%+ for core logic
- **Mocking**: Use `unittest.mock` or `pytest-mock`
- **Naming**: `test_<module>_<scenario>_<expected_result>`

## 📝 Documentation Requirements

### Package README.md
```markdown
# Package Name

## Purpose
<1-2 sentences>

## Architecture
<Link to architecture diagram if needed>

## Modules
- **core/**: <Description>
- **adapters/**: <Description>
- **nodes/**: <Description>

## Quick Start
<Simple example>

## Testing
<How to run tests>

## Contributing
<Link to VIBE_CODING_ARCHITECTURE.md>
```

### Module README.md
```markdown
# Module Name

## Purpose
<1-2 sentences>

## Public Interface
<List main classes/functions>

## Usage Example
<Code snippet>

## Tests
<How to test this module>

## Dependencies
<What it depends on and why>
```

## 🤖 AI Prompting Guide

### Effective Prompts (Do's)
✅ **Specific module reference**:
```
"Refactor dialogue_node.py by extracting dialogue state management 
into core/dialogue_manager.py. The manager should handle:
- Wake word detection
- Silence mode
- Query accumulation
Keep it under 250 LOC. Add tests to tests/core/test_dialogue_manager.py"
```

✅ **Step-by-step**:
```
"Step 1: Create core/dialogue_manager.py with DialogueManager class
Step 2: Move wake word logic from dialogue_node.py lines 150-200
Step 3: Add unit tests for wake word detection
Step 4: Update dialogue_node.py to use DialogueManager"
```

✅ **With context**:
```
"Looking at reflection_node.py (877 LOC), extract event detection logic 
to core/event_detector.py. The detector should:
- Track event states (lines 84-87)
- Detect state changes (edge detection)
- Return events that need reaction

Example from current code (lines 350-400):
[paste relevant code]

Make it testable without ROS."
```

### Ineffective Prompts (Don'ts)
❌ "Refactor the voice assistant"
❌ "Make the code better"
❌ "Fix all the issues in perception"

## 🎯 Success Criteria

### Per Module
- [ ] Single responsibility (clear purpose)
- [ ] Under size limit (check LOC)
- [ ] Has README.md
- [ ] Has tests (80%+ coverage)
- [ ] No direct dependencies on other modules (use interfaces)
- [ ] Can be understood by LLM in single context window

### Per Package
- [ ] Clear layered structure (core/adapters/nodes)
- [ ] Package README.md exists
- [ ] All tests passing
- [ ] Documentation up to date
- [ ] Example usage in README

### Overall System
- [ ] No file >300 LOC
- [ ] All core logic testable without ROS
- [ ] Clear module boundaries
- [ ] AI can navigate codebase easily
- [ ] Fast test suite (<30s for unit tests)

## 📚 References

- [Awesome Vibe Coding Guide](https://github.com/analyticalrohit/awesome-vibe-coding-guide)
- [Vibe Coding Best Practices (Web Search Summary)](context)
- [Clean Architecture](https://blog.cleancoder.com/uncle-bob/2012/08/13/the-clean-architecture.html)
- [SOLID Principles](https://en.wikipedia.org/wiki/SOLID)

## 🔄 Migration Path

### For Existing Code
1. **Don't rewrite everything at once**
2. **Extract and test** new modules
3. **Keep old code working** during transition
4. **Gradual migration** - update one caller at a time
5. **Remove old code** only when all callers updated

### For New Features
1. **Start with tests** (what should it do?)
2. **Design interface** (what's the API?)
3. **Implement core** (no dependencies)
4. **Add adapters** (ROS/LLM/etc)
5. **Document** (README + examples)

---

**Last Updated**: 2025-01-20  
**Status**: Active - use for all new development and refactoring
