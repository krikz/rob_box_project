#!/usr/bin/env python3
"""
Unit tests for CommandParser

Tests command parsing, intent classification, and entity extraction
without ROS dependencies.
"""

import pytest
from rob_box_voice.core.command_parser import CommandParser, Command, IntentType


class TestCommandParserInit:
    """Test parser initialization."""

    def test_init_default(self):
        """Test default initialization."""
        parser = CommandParser()

        assert parser.wake_words == ['робот', 'робокс', 'робобокс']
        assert parser.confidence_base == 0.8
        assert len(parser.patterns) > 0

    def test_init_custom_wake_words(self):
        """Test initialization with custom wake words."""
        parser = CommandParser(wake_words=['тест', 'робок'])

        assert parser.wake_words == ['тест', 'робок']

    def test_init_custom_confidence(self):
        """Test initialization with custom confidence."""
        parser = CommandParser(confidence_base=0.9)

        assert parser.confidence_base == 0.9


class TestCommandParserWakeWords:
    """Test wake word removal."""

    def test_remove_wake_word_robot(self):
        """Test removing 'робот' wake word."""
        parser = CommandParser()

        result = parser.remove_wake_word('робот иди вперед')
        assert result == 'иди вперед'

    def test_remove_wake_word_roboks(self):
        """Test removing 'робокс' wake word."""
        parser = CommandParser()

        result = parser.remove_wake_word('робокс стоп')
        assert result == 'стоп'

    def test_remove_wake_word_with_punctuation(self):
        """Test removing wake word with punctuation."""
        parser = CommandParser()

        result = parser.remove_wake_word('робот, иди к кухне')
        assert result == 'иди к кухне'

    def test_remove_wake_word_no_wake_word(self):
        """Test text without wake word."""
        parser = CommandParser()

        result = parser.remove_wake_word('иди вперед')
        assert result == 'иди вперед'


class TestCommandParserNavigate:
    """Test navigation intent classification."""

    def test_navigate_to_waypoint_by_name(self):
        """Test navigation to named waypoint."""
        parser = CommandParser()

        command = parser.parse('иди к кухне')

        assert command.intent == IntentType.NAVIGATE
        assert command.entities.get('waypoint') == 'кухне'
        assert command.confidence > 0.7

    def test_navigate_to_waypoint_by_number(self):
        """Test navigation to numbered waypoint."""
        parser = CommandParser()

        command = parser.parse('поезжай к точке 2')

        assert command.intent == IntentType.NAVIGATE
        assert command.entities.get('waypoint') == 'точка 2'
        assert command.confidence > 0.7

    def test_navigate_direction_with_verb(self):
        """Test movement with direction and verb."""
        parser = CommandParser()

        command = parser.parse('иди вперед')

        assert command.intent == IntentType.NAVIGATE
        assert command.entities.get('direction') == 'вперед'
        assert command.confidence > 0.7

    def test_navigate_direction_only(self):
        """Test movement with direction only."""
        parser = CommandParser()

        command = parser.parse('вперед')

        assert command.intent == IntentType.NAVIGATE
        assert command.entities.get('direction') == 'вперед'
        assert command.confidence > 0.7

    def test_navigate_turn_with_verb(self):
        """Test turn with verb."""
        parser = CommandParser()

        command = parser.parse('поверни направо')

        assert command.intent == IntentType.NAVIGATE
        assert command.entities.get('direction') == 'направо'

    def test_navigate_turn_only(self):
        """Test turn without verb."""
        parser = CommandParser()

        command = parser.parse('налево')

        assert command.intent == IntentType.NAVIGATE
        assert command.entities.get('direction') == 'налево'


class TestCommandParserStop:
    """Test stop intent classification."""

    def test_stop_command(self):
        """Test basic stop command."""
        parser = CommandParser()

        command = parser.parse('стоп')

        assert command.intent == IntentType.STOP
        assert command.confidence > 0.7

    def test_stop_halt(self):
        """Test halt command."""
        parser = CommandParser()

        command = parser.parse('halt')

        assert command.intent == IntentType.STOP

    def test_stop_ostanovis(self):
        """Test 'остановись' command."""
        parser = CommandParser()

        command = parser.parse('остановись')

        assert command.intent == IntentType.STOP

    def test_cancel_navigation(self):
        """Test cancel navigation."""
        parser = CommandParser()

        command = parser.parse('отмени навигацию')

        assert command.intent == IntentType.STOP


class TestCommandParserStatus:
    """Test status intent classification."""

    def test_status_where(self):
        """Test 'где ты' status query."""
        parser = CommandParser()

        command = parser.parse('где ты')

        assert command.intent == IntentType.STATUS
        assert command.confidence > 0.7

    def test_status_show(self):
        """Test 'покажи статус'."""
        parser = CommandParser()

        command = parser.parse('покажи статус')

        assert command.intent == IntentType.STATUS


class TestCommandParserMap:
    """Test map intent classification."""

    def test_map_show(self):
        """Test 'покажи карту'."""
        parser = CommandParser()

        command = parser.parse('покажи карту')

        assert command.intent == IntentType.MAP

    def test_map_create(self):
        """Test 'создай карту'."""
        parser = CommandParser()

        command = parser.parse('создай карту')

        assert command.intent == IntentType.MAP


class TestCommandParserVision:
    """Test vision intent classification."""

    def test_vision_what_see(self):
        """Test 'что видишь'."""
        parser = CommandParser()

        command = parser.parse('что видишь')

        assert command.intent == IntentType.VISION

    def test_vision_find(self):
        """Test 'найди объект'."""
        parser = CommandParser()

        command = parser.parse('найди объект')

        assert command.intent == IntentType.VISION


class TestCommandParserFollow:
    """Test follow intent classification."""

    def test_follow_me(self):
        """Test 'следуй за мной'."""
        parser = CommandParser()

        command = parser.parse('следуй за мной')

        assert command.intent == IntentType.FOLLOW

    def test_follow_activate(self):
        """Test 'включи режим следования'."""
        parser = CommandParser()

        command = parser.parse('включи режим следования')

        assert command.intent == IntentType.FOLLOW


class TestCommandParserUnknown:
    """Test unknown command handling."""

    def test_unknown_command(self):
        """Test command that doesn't match any pattern."""
        parser = CommandParser()

        command = parser.parse('непонятная команда')

        assert command.intent == IntentType.UNKNOWN
        assert command.confidence == 0.0
        assert len(command.entities) == 0

    def test_empty_command(self):
        """Test empty command."""
        parser = CommandParser()

        command = parser.parse('')

        assert command.intent == IntentType.UNKNOWN


class TestCommandParserParse:
    """Test full parse pipeline."""

    def test_parse_with_wake_word(self):
        """Test parsing with wake word."""
        parser = CommandParser()

        command = parser.parse('робот иди к кухне')

        assert command.intent == IntentType.NAVIGATE
        assert command.text == 'иди к кухне'  # Wake word removed
        assert 'кухне' in command.entities.get('waypoint', '')

    def test_parse_without_wake_word(self):
        """Test parsing without wake word."""
        parser = CommandParser()

        command = parser.parse('стоп')

        assert command.intent == IntentType.STOP
        assert command.text == 'стоп'

    def test_parse_case_insensitive(self):
        """Test case insensitive parsing."""
        parser = CommandParser()

        command1 = parser.parse('СТОП')
        command2 = parser.parse('стоп')

        assert command1.intent == command2.intent == IntentType.STOP


class TestCommandParserCustomization:
    """Test parser customization."""

    def test_add_pattern(self):
        """Test adding custom pattern."""
        parser = CommandParser()

        # Add custom pattern for STATUS
        parser.add_pattern(IntentType.STATUS, r'покажи\s+позицию', None)

        command = parser.parse('покажи позицию')

        assert command.intent == IntentType.STATUS

    def test_get_patterns(self):
        """Test getting patterns for intent."""
        parser = CommandParser()

        patterns = parser.get_patterns(IntentType.STOP)

        assert len(patterns) > 0
        assert all(isinstance(p, tuple) for p in patterns)

    def test_get_patterns_nonexistent(self):
        """Test getting patterns for nonexistent intent."""
        parser = CommandParser()

        patterns = parser.get_patterns(IntentType.UNKNOWN)

        assert patterns == []


class TestCommandParserConfidence:
    """Test confidence calculation."""

    def test_confidence_full_match(self):
        """Test confidence for full text match."""
        parser = CommandParser()

        command = parser.parse('стоп')

        # Should have high confidence for exact match
        assert command.confidence >= 0.8

    def test_confidence_partial_match(self):
        """Test confidence for partial match."""
        parser = CommandParser()

        command = parser.parse('стоп и остановись')

        # Should still detect but with varying confidence
        assert command.intent == IntentType.STOP
        assert command.confidence > 0.0


class TestCommandParserEdgeCases:
    """Test edge cases and error handling."""

    def test_whitespace_only(self):
        """Test whitespace only input."""
        parser = CommandParser()

        command = parser.parse('   ')

        assert command.intent == IntentType.UNKNOWN

    def test_special_characters(self):
        """Test input with special characters."""
        parser = CommandParser()

        command = parser.parse('стоп!!!')

        assert command.intent == IntentType.STOP

    def test_mixed_language(self):
        """Test mixed language input."""
        parser = CommandParser()

        command = parser.parse('робот halt')

        assert command.intent == IntentType.STOP

    def test_multiple_wake_words(self):
        """Test text with multiple wake words."""
        parser = CommandParser()

        # Should remove first wake word only
        result = parser.remove_wake_word('робот робот стоп')
        assert result == 'робот стоп'


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
