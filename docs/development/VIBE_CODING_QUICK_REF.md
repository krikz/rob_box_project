# Quick Reference: Vibe Coding for AI Agents

> **For AI/LLM agents working on Rob Box codebase**  
> This is your TL;DR guide. Read this first before making changes.

## 🎯 Core Rules

1. **Files must be <300 LOC** (preferably <200)
2. **One responsibility per file**
3. **Core logic must be testable without ROS**
4. **Always write tests for new modules**
5. **Use layered architecture**: core → adapters → nodes

## 📏 File Size Limits

| Type | Max LOC | Example |
|------|---------|---------|
| Protocol/Interface | 100 | `core/protocol.py` |
| Utility | 150 | `utils/time_provider.py` |
| Core Logic | 250 | `core/dialogue_manager.py` |
| ROS Node | 200 | `nodes/dialogue_node.py` |
| Integration | 300 | `adapters/llm_client.py` |

**If file exceeds limit → Split it!**

## 🏗️ Directory Structure

```
package_name/
├── core/          # Pure Python business logic (NO ROS, NO I/O)
├── adapters/      # External integrations (LLM API, etc)
├── nodes/         # ROS nodes (thin wrappers to core)
├── models/        # Data classes
├── utils/         # Shared utilities
└── tests/
    ├── unit/      # Fast, isolated tests
    └── integration/  # Component interaction tests
```

## ✅ Good Examples

### Good Module Structure
```python
# core/dialogue_manager.py (~200 LOC)
"""
DialogueManager - Manages dialogue state

Purpose:
    Handles wake words, silence mode, query accumulation
    
Public Interface:
    - is_wake_word(text) -> bool
    - should_respond(text) -> bool
    - accumulate_query(text) -> Optional[str]
"""

class DialogueManager:
    def __init__(self, wake_words: List[str]):
        self.wake_words = wake_words
        self.in_silence = False
    
    def is_wake_word(self, text: str) -> bool:
        """Check if text contains wake word"""
        return any(word in text.lower() for word in self.wake_words)
```

### Good Node Structure
```python
# nodes/dialogue_node.py (~200 LOC)
"""ROS node wrapper for dialogue system"""

from rob_box_voice.core.dialogue_manager import DialogueManager
from rob_box_voice.llm.streaming_client import StreamingClient

class DialogueNode(Node):
    def __init__(self):
        super().__init__('dialogue_node')
        
        # Core components (business logic)
        self.dialogue_mgr = DialogueManager(wake_words=['робок'])
        self.llm_client = StreamingClient()
        
        # ROS infrastructure
        self.sub = self.create_subscription(...)
        self.pub = self.create_publisher(...)
    
    def _on_message(self, msg):
        """Handle ROS message - delegate to core"""
        if self.dialogue_mgr.should_respond(msg.data):
            response = self.llm_client.generate(msg.data)
            self.pub.publish(response)
```

### Good Test
```python
# tests/unit/core/test_dialogue_manager.py
def test_wake_word_detection():
    manager = DialogueManager(wake_words=['робок', 'робот'])
    
    assert manager.is_wake_word('робок привет') is True
    assert manager.is_wake_word('привет') is False
```

## ❌ Bad Examples

### ❌ Bad: Too large, mixed concerns
```python
# dialogue_node.py (2313 LOC!) 
class DialogueNode(Node):
    def __init__(self):
        # LLM client setup (100 LOC)
        # Wake word detection (50 LOC)
        # Streaming handler (200 LOC)
        # Tool call integration (300 LOC)
        # ROS setup (100 LOC)
        # Error handling (100 LOC)
        # ...everything in one file
```

### ❌ Bad: Business logic in node
```python
class DialogueNode(Node):
    def _on_message(self, msg):
        # ❌ Wake word logic directly in node
        if 'робок' in msg.data or 'робот' in msg.data:
            # ❌ LLM logic directly in node
            response = requests.post('https://api...')
            # ❌ Can't test without ROS!
```

### ❌ Bad: Untestable
```python
# ❌ No way to test without ROS runtime
def process_message(ros_msg):
    data = ros_msg.data
    # logic here
```

✅ **Better**:
```python
# ✅ Testable function
def process_text(text: str) -> str:
    # logic here
    return result

# ROS wrapper
def _on_message(self, ros_msg):
    result = process_text(ros_msg.data)
    self.pub.publish(result)
```

## 🔧 Common Tasks

### Task: Extract logic from large file

**Step 1**: Identify responsibility
```python
# In dialogue_node.py, lines 150-200 handle wake word detection
```

**Step 2**: Create new module
```bash
touch src/rob_box_voice/rob_box_voice/core/wake_word_detector.py
```

**Step 3**: Extract logic
```python
# core/wake_word_detector.py
class WakeWordDetector:
    def __init__(self, wake_words: List[str]):
        self.wake_words = wake_words
    
    def is_wake_word(self, text: str) -> bool:
        return any(word in text.lower() for word in self.wake_words)
```

**Step 4**: Write tests
```python
# tests/unit/core/test_wake_word_detector.py
def test_detection():
    detector = WakeWordDetector(['робок'])
    assert detector.is_wake_word('робок привет') is True
```

**Step 5**: Update original file
```python
# dialogue_node.py
from rob_box_voice.core.wake_word_detector import WakeWordDetector

class DialogueNode(Node):
    def __init__(self):
        self.wake_detector = WakeWordDetector(['робок'])
    
    def _on_message(self, msg):
        if self.wake_detector.is_wake_word(msg.data):
            # ...
```

### Task: Add new feature

**DON'T**:
- ❌ Add to existing large file
- ❌ Mix with unrelated code

**DO**:
- ✅ Create new focused module
- ✅ Write tests first
- ✅ Integrate through clean interface

### Task: Fix bug

**DON'T**:
- ❌ Make changes in multiple places
- ❌ Skip tests

**DO**:
- ✅ Add test that reproduces bug
- ✅ Fix in single module
- ✅ Verify test passes

## 📝 Prompting Yourself

### ✅ Effective Self-Prompts

**Before starting**:
- "What is the single responsibility of this module?"
- "Can I test this without ROS/LLM API/external service?"
- "Is this file over 250 LOC? Should I split it?"

**During work**:
- "Does this belong in core/ or adapters/?"
- "Can I write a test for this?"
- "Is this the minimal change needed?"

**After changes**:
- "Did I add tests?"
- "Is the interface clear?"
- "Would another AI understand this module?"

### ❌ Warning Signs

If you think:
- "This file is getting long..." → **Stop, split it now**
- "I need to change 5 files for this..." → **Refactor first**
- "I can't test this easily..." → **Wrong layer, move to core/**
- "This is getting complex..." → **Break into smaller pieces**

## 🧪 Testing Checklist

### For Core Modules
- [ ] No ROS dependencies
- [ ] No API calls (mock them)
- [ ] Fast (<100ms per test)
- [ ] Clear test names
- [ ] 80%+ coverage

### For Adapters
- [ ] Mock external services
- [ ] Test error cases
- [ ] Test edge cases

### For Nodes
- [ ] Integration tests
- [ ] Mock ROS if possible
- [ ] Test main flows

## 📊 Module Template

Use this template for new modules:

```python
#!/usr/bin/env python3
"""
<module_name>.py - <One-line purpose>

Purpose:
    <2-3 sentences describing what this does>

Public Interface:
    - Function1: Description
    - Class1: Description

Example:
    >>> from package.module import Class
    >>> obj = Class()
    >>> result = obj.method()

Tests:
    See: tests/unit/test_<module>.py

Dependencies:
    - dependency1 (why)
"""

# Standard library
import typing
from typing import List, Dict, Any

# Third party
# (only if needed)

# Local
from .protocol import BaseClass


class ModuleName:
    """
    Detailed description
    
    Attributes:
        attr1: Description
    
    Methods:
        method1: Description
    """
    
    def __init__(self, param: str):
        """Initialize module"""
        self.param = param
    
    def public_method(self, arg: str) -> str:
        """
        Do something
        
        Args:
            arg: Description
            
        Returns:
            Description
            
        Raises:
            ValueError: When...
        """
        return self._private_method(arg)
    
    def _private_method(self, arg: str) -> str:
        """Internal implementation detail"""
        return arg.upper()


# Example usage
if __name__ == '__main__':
    obj = ModuleName('test')
    print(obj.public_method('hello'))
```

## 🎯 Current Refactoring Status

| Package | Status | Next Action |
|---------|--------|-------------|
| voice | Planning | Extract DialogueManager |
| perception | Planning | Extract ContextBuilder |
| mcp_tools | Planning | Extract TaskManager |

## 📚 Key Documents

**Must read**:
- [VIBE_CODING_ARCHITECTURE.md](./VIBE_CODING_ARCHITECTURE.md) - Core principles

**Package-specific**:
- [REFACTORING_PLAN_VOICE.md](./REFACTORING_PLAN_VOICE.md)
- [REFACTORING_PLAN_PERCEPTION.md](./REFACTORING_PLAN_PERCEPTION.md)
- [REFACTORING_PLAN_MCP_TOOLS.md](./REFACTORING_PLAN_MCP_TOOLS.md)

**Roadmap**:
- [REFACTORING_ROADMAP.md](./REFACTORING_ROADMAP.md)

## 🚀 Quick Start

**You are working on a task?**

1. **Understand**: Read relevant REFACTORING_PLAN_*.md
2. **Focus**: Work on ONE module at a time
3. **Test**: Write test first, then code
4. **Verify**: Run tests, check LOC count
5. **Document**: Add docstring, update README
6. **Commit**: Small, focused commits

**Working with existing code?**

1. **Check size**: `wc -l <file>` - over 300? Time to split
2. **Check tests**: `pytest tests/unit/<module>` - exists? Good!
3. **Check layer**: In `core/`? Should have no ROS/IO
4. **Make minimal change**: Don't refactor everything at once

## ⚡ Shortcuts

```bash
# Check file size
wc -l src/rob_box_voice/rob_box_voice/dialogue_node.py

# Run tests for module
pytest tests/unit/core/test_dialogue_manager.py -v

# Check test coverage
pytest --cov=rob_box_voice.core tests/unit/core/

# Count modules
find src/rob_box_voice/rob_box_voice/core -name "*.py" | wc -l

# Find large files
find src/rob_box_voice -name "*.py" -exec wc -l {} + | sort -rn | head
```

---

**Remember**: Small, focused, testable modules = Happy AI = Happy humans! 🎉
