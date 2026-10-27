# Core Business Logic

## Purpose

Pure Python modules containing business logic with no ROS dependencies. These modules are easy to test and understand, making them ideal for AI-assisted development.

## Modules

### dialogue_manager.py (321 LOC)

**Purpose**: Manages dialogue state machine, wake word detection, silence mode, and query accumulation.

**Public Interface**:
- `DialogueState` - Enum for dialogue states (IDLE, LISTENING, DIALOGUE, SILENCED)
- `DialogueManager` - Main class for dialogue state management

**Key Methods**:
- `is_wake_word(text)` - Check if text contains wake word
- `remove_wake_word(text)` - Remove wake word from text
- `is_silence_command(text)` - Check if text is silence command
- `should_respond(text)` - Check if should respond based on state
- `transition_state(new_state)` - Transition to new state
- `enable_silence(duration)` - Enable silence mode
- `add_query(query)` - Add query to accumulation queue
- `get_accumulated_queries()` - Get and clear accumulated queries

**Usage Example**:
```python
from rob_box_voice.core.dialogue_manager import DialogueManager

# Create manager
manager = DialogueManager(wake_words=['робок', 'робот'])

# Check wake word
if manager.is_wake_word('робок привет'):
    manager.transition_state(DialogueState.LISTENING)

# Process queries
if manager.should_respond(user_text):
    manager.add_query(user_text)
    
    if manager.should_process_queries():
        queries = manager.get_accumulated_queries()
        # Process queries...
```

**Tests**: `test/unit/core/test_dialogue_manager.py` (10 test classes, 25+ tests)

**Dependencies**: None (pure Python with standard library only)

### command_parser.py (307 LOC)

**Purpose**: Parses voice commands, classifies intents, and extracts entities for robot control.

**Public Interface**:
- `IntentType` - Enum for command intent types (NAVIGATE, STOP, FOLLOW, etc.)
- `Command` - Data class for parsed commands (intent, entities, confidence)
- `CommandParser` - Main class for command parsing

**Key Methods**:
- `parse(text)` - Main entry point for command parsing
- `classify_intent(text)` - Classify intent and extract entities
- `remove_wake_word(text)` - Remove wake word from command
- `add_pattern(intent, pattern, entity_type)` - Add custom pattern
- `get_patterns(intent)` - Get patterns for specific intent

**Usage Example**:
```python
from rob_box_voice.core.command_parser import CommandParser, IntentType

# Create parser
parser = CommandParser()

# Parse command
command = parser.parse('робот иди к кухне')
print(command.intent)     # IntentType.NAVIGATE
print(command.entities)   # {'waypoint': 'кухне'}
print(command.confidence) # 0.95

# Parse simple direction
command = parser.parse('вперед')
print(command.intent)     # IntentType.NAVIGATE
print(command.entities)   # {'direction': 'вперед'}
```

**Tests**: `test/unit/core/test_command_parser.py` (11 test classes, 40+ tests)

**Dependencies**: None (pure Python with standard library only)

## Testing

Run unit tests:
```bash
# From package root
python3 -m pytest test/unit/core/test_dialogue_manager.py -v
python3 -m pytest test/unit/core/test_command_parser.py -v

# Or test all core modules
python3 -m pytest test/unit/core/ -v
```

## Design Principles

1. **No ROS dependencies** - Can be tested without ROS runtime
2. **Single responsibility** - Each module does one thing well
3. **Small and focused** - All files <350 LOC
4. **Well documented** - Clear docstrings and examples
5. **Fully tested** - 80%+ test coverage

## Module Summary

| Module | LOC | Tests | Status |
|--------|-----|-------|--------|
| dialogue_manager.py | 321 | 25+ | ✅ Complete |
| command_parser.py | 307 | 40+ | ✅ Complete |

**Total**: 2 of 15 planned modules (13% complete)  
**LOC**: 628 lines, 65+ tests

## Future Modules

According to the refactoring plan, the following modules will be added:

- LLM layer modules (streaming, tool calls, providers)
- Audio layer modules (playback, recording)
- Additional core utilities as needed

## Contributing

When adding new core modules:

1. Keep files <300 LOC (up to 350 LOC acceptable)
2. No ROS dependencies (use adapters for that)
3. Add comprehensive unit tests
4. Update this README
5. Follow the module template in VIBE_CODING_ARCHITECTURE.md
