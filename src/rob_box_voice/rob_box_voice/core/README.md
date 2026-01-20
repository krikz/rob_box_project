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

## Testing

Run unit tests:
```bash
# From package root
python3 -m pytest test/unit/core/test_dialogue_manager.py -v

# Or test all core modules
python3 -m pytest test/unit/core/ -v
```

## Design Principles

1. **No ROS dependencies** - Can be tested without ROS runtime
2. **Single responsibility** - Each module does one thing well
3. **Small and focused** - All files <300 LOC
4. **Well documented** - Clear docstrings and examples
5. **Fully tested** - 80%+ test coverage

## Future Modules

According to the refactoring plan, the following modules will be added:

- `command_parser.py` (~150 LOC) - Command parsing logic
- `speech_formatter.py` (~150 LOC) - TTS text formatting
- `wake_word_detector.py` (~100 LOC) - Wake word logic (if needed separately)
- `query_accumulator.py` (~100 LOC) - Query accumulation (if needed separately)

## Contributing

When adding new core modules:

1. Keep files <300 LOC
2. No ROS dependencies (use adapters for that)
3. Add comprehensive unit tests
4. Update this README
5. Follow the module template in VIBE_CODING_ARCHITECTURE.md
