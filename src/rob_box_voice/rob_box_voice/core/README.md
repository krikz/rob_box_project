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

### speech_formatter.py (253 LOC)

**Purpose**: Formats text for TTS output with accent placement, SSML processing, and text cleanup.

**Public Interface**:
- `SpeechFormatter` - Main class for text formatting

**Key Methods**:
- `format_for_tts(text, add_ssml)` - Main formatting method
- `add_accents(text)` - Add Russian accents to text
- `clean_for_speech(text)` - Clean text for speech synthesis
- `wrap_in_ssml(text)` - Wrap text in SSML tags
- `extract_from_ssml(ssml_text)` - Extract text from SSML
- `is_ssml(text)` - Check if text is SSML

**Usage Example**:
```python
from rob_box_voice.core.speech_formatter import SpeechFormatter

# Create formatter
formatter = SpeechFormatter()

# Format for TTS
formatted = formatter.format_for_tts('**Важно**: привет!', add_ssml=True)
# Result: '<speak>Важно: привет!</speak>' with accents added

# Clean markdown and URLs
clean = formatter.clean_for_speech('See https://example.com here')
# Result: 'See here'
```

**Tests**: `test/unit/core/test_speech_formatter.py` (7 test classes, 30+ tests)

**Dependencies**: AccentReplacer from scripts (optional, graceful fallback)

## Testing

Run unit tests:
```bash
# From package root
python3 -m pytest test/unit/core/test_dialogue_manager.py -v
python3 -m pytest test/unit/core/test_speech_formatter.py -v

# Or test all core modules
python3 -m pytest test/unit/core/ -v
```

## Design Principles

1. **No ROS dependencies** - Can be tested without ROS runtime
2. **Single responsibility** - Each module does one thing well
3. **Small and focused** - All files <300 LOC
4. **Well documented** - Clear docstrings and examples
5. **Fully tested** - 80%+ test coverage

## Module Summary

| Module | LOC | Tests | Status |
|--------|-----|-------|--------|
| dialogue_manager.py | 321 | 25+ | ✅ Complete |
| speech_formatter.py | 253 | 30+ | ✅ Complete |

**Total**: 2 of 15 planned modules (13% complete)

## Future Modules

According to the refactoring plan, the following modules will be added:

- `command_parser.py` (~150 LOC) - Command parsing logic
- LLM layer modules (streaming, tool calls, providers)
- Audio layer modules (playback, recording)

## Contributing

When adding new core modules:

1. Keep files <300 LOC
2. No ROS dependencies (use adapters for that)
3. Add comprehensive unit tests
4. Update this README
5. Follow the module template in VIBE_CODING_ARCHITECTURE.md
