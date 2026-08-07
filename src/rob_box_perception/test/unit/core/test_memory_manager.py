"""Tests for MemoryManager."""

import time
import pytest
from rob_box_perception.core.memory_manager import MemoryManager


class TestMemoryManager:
    """Test suite for MemoryManager."""

    def test_initialization(self):
        """Test MemoryManager initialization."""
        manager = MemoryManager(memory_window=100.0)

        assert manager.memory_window == 100.0
        assert len(manager.recent_events) == 0
        assert len(manager.speech_events) == 0
        assert len(manager.robot_response_events) == 0
        assert not manager.has_events()

    def test_add_event_general(self):
        """Test adding a general event."""
        manager = MemoryManager()

        manager.add_event('user_speech', 'Hello robot', important=True)

        assert len(manager.recent_events) == 1
        assert len(manager.speech_events) == 1
        assert manager.recent_events[0]['type'] == 'user_speech'
        assert manager.recent_events[0]['content'] == 'Hello robot'
        assert manager.recent_events[0]['important'] is True
        assert manager.has_events()

    def test_add_event_types(self):
        """Test adding different event types."""
        manager = MemoryManager()

        manager.add_event('user_speech', 'Speech 1')
        manager.add_event('robot_response', 'Response 1')
        manager.add_event('robot_thought', 'Thought 1')
        manager.add_event('vision', 'Vision 1')
        manager.add_event('error', 'Error 1')

        assert len(manager.recent_events) == 5
        assert len(manager.speech_events) == 1
        assert len(manager.robot_response_events) == 1
        assert len(manager.robot_thought_events) == 1
        assert len(manager.vision_events) == 1
        assert len(manager.system_events) == 1

    def test_cleanup_old_events(self):
        """Test automatic cleanup of old events."""
        manager = MemoryManager(memory_window=0.5)  # 0.5 seconds window

        manager.add_event('user_speech', 'Event 1')
        time.sleep(0.6)
        manager.add_event('user_speech', 'Event 2')

        # Event 1 should be cleaned up
        assert len(manager.recent_events) == 1
        assert manager.recent_events[0]['content'] == 'Event 2'

    def test_get_summary_empty(self):
        """Test get_summary with no events."""
        manager = MemoryManager()

        summary = manager.get_summary()
        assert summary == "Недавних событий нет"

    def test_get_summary_with_events(self):
        """Test get_summary with events."""
        manager = MemoryManager()

        manager.add_event('user_speech', 'Hello', important=True)
        manager.add_event('robot_response', 'Hi there')

        summary = manager.get_summary(count=2)
        assert '❗' in summary  # Important event marker
        assert 'user_speech' in summary
        assert 'robot_response' in summary

    def test_get_summary_count_limit(self):
        """Test get_summary with count limit."""
        manager = MemoryManager()

        for i in range(10):
            manager.add_event('test', f'Event {i}')

        summary = manager.get_summary(count=3)
        lines = summary.split('\n')
        assert len(lines) == 3

    def test_get_all_events(self):
        """Test get_all_events."""
        manager = MemoryManager()

        manager.add_event('test1', 'Content 1')
        manager.add_event('test2', 'Content 2')

        events = manager.get_all_events()
        assert len(events) == 2
        assert events[0]['type'] == 'test1'
        assert events[1]['type'] == 'test2'

    def test_get_events_by_type(self):
        """Test get_events_by_type."""
        manager = MemoryManager()

        manager.add_event('user_speech', 'Speech 1')
        manager.add_event('user_speech', 'Speech 2')
        manager.add_event('robot_response', 'Response 1')

        speech_events = manager.get_events_by_type('user_speech')
        assert len(speech_events) == 2

        response_events = manager.get_events_by_type('robot_response')
        assert len(response_events) == 1

    def test_get_events_by_type_vision(self):
        """Test get_events_by_type for vision and apriltag."""
        manager = MemoryManager()

        manager.add_event('vision', 'Vision 1')
        manager.add_event('apriltag', 'Tag 1')

        vision_events = manager.get_events_by_type('vision')
        assert len(vision_events) == 2  # Both vision and apriltag

    def test_get_recent_count(self):
        """Test get_recent_count."""
        manager = MemoryManager()

        manager.add_event('user_speech', 'Speech 1')
        manager.add_event('user_speech', 'Speech 2')
        manager.add_event('robot_response', 'Response 1')

        assert manager.get_recent_count() == 3
        assert manager.get_recent_count('user_speech') == 2
        assert manager.get_recent_count('robot_response') == 1

    def test_clear_all(self):
        """Test clear_all."""
        manager = MemoryManager()

        manager.add_event('user_speech', 'Speech 1')
        manager.add_event('robot_response', 'Response 1')

        manager.clear_all()

        assert len(manager.recent_events) == 0
        assert len(manager.speech_events) == 0
        assert len(manager.robot_response_events) == 0
        assert not manager.has_events()

    def test_clear_by_type(self):
        """Test clear_by_type."""
        manager = MemoryManager()

        manager.add_event('user_speech', 'Speech 1')
        manager.add_event('user_speech', 'Speech 2')
        manager.add_event('robot_response', 'Response 1')

        manager.clear_by_type('user_speech')

        assert len(manager.recent_events) == 1
        assert len(manager.speech_events) == 0
        assert len(manager.robot_response_events) == 1

    def test_get_important_events(self):
        """Test get_important_events."""
        manager = MemoryManager()

        manager.add_event('event1', 'Normal event 1')
        manager.add_event('event2', 'Important event 1', important=True)
        manager.add_event('event3', 'Normal event 2')
        manager.add_event('event4', 'Important event 2', important=True)

        important = manager.get_important_events()
        assert len(important) == 2
        assert all(e['important'] for e in important)

    def test_get_important_events_with_count(self):
        """Test get_important_events with count limit."""
        manager = MemoryManager()

        for i in range(5):
            manager.add_event(f'event{i}', f'Content {i}', important=True)

        important = manager.get_important_events(count=2)
        assert len(important) == 2

    def test_has_events(self):
        """Test has_events."""
        manager = MemoryManager()

        assert not manager.has_events()

        manager.add_event('test', 'Test event')
        assert manager.has_events()

        manager.clear_all()
        assert not manager.has_events()

    def test_event_time_tracking(self):
        """Test that events store time correctly."""
        manager = MemoryManager()

        before = time.time()
        manager.add_event('test', 'Event')
        after = time.time()

        event = manager.recent_events[0]
        assert 'time' in event
        assert before <= event['time'] <= after

    def test_custom_memory_window(self):
        """Test custom memory window."""
        manager = MemoryManager(memory_window=1.0)

        manager.add_event('event1', 'Content 1')
        time.sleep(1.1)
        manager.add_event('event2', 'Content 2')

        assert len(manager.recent_events) == 1
        assert manager.recent_events[0]['type'] == 'event2'

    def test_multiple_events_same_type(self):
        """Test adding multiple events of the same type."""
        manager = MemoryManager()

        for i in range(5):
            manager.add_event('user_speech', f'Speech {i}')

        assert len(manager.speech_events) == 5
        assert len(manager.recent_events) == 5

    def test_event_ordering(self):
        """Test that events maintain chronological order."""
        manager = MemoryManager()

        manager.add_event('event1', 'First')
        time.sleep(0.01)
        manager.add_event('event2', 'Second')
        time.sleep(0.01)
        manager.add_event('event3', 'Third')

        events = manager.get_all_events()
        assert events[0]['content'] == 'First'
        assert events[1]['content'] == 'Second'
        assert events[2]['content'] == 'Third'
