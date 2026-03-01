"""Unit tests for EventDetector module."""

import time
import unittest
from unittest.mock import patch

from rob_box_perception.core.event_detector import EventDetector, EventChange


class TestEventChange(unittest.TestCase):
    """Tests for EventChange dataclass."""
    
    def test_event_change_creation(self):
        """Test creating an EventChange."""
        change = EventChange(
            event_name='test_event',
            old_value='old',
            new_value='new',
            is_edge=True,
            timestamp=123.456
        )
        
        self.assertEqual(change.event_name, 'test_event')
        self.assertEqual(change.old_value, 'old')
        self.assertEqual(change.new_value, 'new')
        self.assertTrue(change.is_edge)
        self.assertEqual(change.timestamp, 123.456)


class TestEventDetectorInitialization(unittest.TestCase):
    """Tests for EventDetector initialization."""
    
    def test_default_initialization(self):
        """Test default initialization."""
        detector = EventDetector()
        
        self.assertEqual(detector.cooldown_interval, 60.0)
        self.assertEqual(len(detector.event_states), 0)
        self.assertEqual(len(detector.event_last_reaction), 0)
    
    def test_custom_cooldown(self):
        """Test initialization with custom cooldown."""
        detector = EventDetector(cooldown_interval=120.0)
        
        self.assertEqual(detector.cooldown_interval, 120.0)


class TestEventDetectorStateChange(unittest.TestCase):
    """Tests for state change detection."""
    
    def setUp(self):
        """Set up test detector."""
        self.detector = EventDetector()
    
    def test_first_state_change(self):
        """Test detecting first state change."""
        change = self.detector.check_state_change('test_event', 'value1')
        
        self.assertIsNotNone(change)
        self.assertEqual(change.event_name, 'test_event')
        self.assertIsNone(change.old_value)
        self.assertEqual(change.new_value, 'value1')
        self.assertTrue(change.is_edge)
        self.assertIsInstance(change.timestamp, float)
    
    def test_state_change_detected(self):
        """Test detecting state change."""
        # Set initial state
        self.detector.check_state_change('test_event', 'value1')
        
        # Change state
        change = self.detector.check_state_change('test_event', 'value2')
        
        self.assertIsNotNone(change)
        self.assertEqual(change.old_value, 'value1')
        self.assertEqual(change.new_value, 'value2')
        self.assertTrue(change.is_edge)
    
    def test_no_state_change(self):
        """Test when state doesn't change."""
        # Set initial state
        self.detector.check_state_change('test_event', 'value1')
        
        # Same state
        change = self.detector.check_state_change('test_event', 'value1')
        
        self.assertIsNone(change)
    
    def test_track_state_false(self):
        """Test checking without tracking state."""
        change1 = self.detector.check_state_change(
            'test_event',
            'value1',
            track_state=False
        )
        
        # Should not track
        self.assertFalse(self.detector.is_event_tracked('test_event'))
        
        # Next call should still see None as previous
        change2 = self.detector.check_state_change('test_event', 'value2')
        self.assertIsNone(change2.old_value)


class TestEventDetectorCooldown(unittest.TestCase):
    """Tests for cooldown functionality."""
    
    def setUp(self):
        """Set up test detector."""
        self.detector = EventDetector(cooldown_interval=5.0)
    
    def test_should_react_first_time(self):
        """Test should react on first occurrence."""
        self.assertTrue(self.detector.should_react_to_event('test_event'))
    
    def test_should_not_react_during_cooldown(self):
        """Test should not react during cooldown."""
        # Mark reacted
        self.detector.mark_event_reacted('test_event')
        
        # Should not react immediately
        self.assertFalse(self.detector.should_react_to_event('test_event'))
    
    @patch('time.time')
    def test_should_react_after_cooldown(self, mock_time):
        """Test should react after cooldown expires."""
        # Mark reacted at time 100
        mock_time.return_value = 100.0
        self.detector.mark_event_reacted('test_event')
        
        # Check at time 104 (4 seconds, within cooldown)
        mock_time.return_value = 104.0
        self.assertFalse(self.detector.should_react_to_event('test_event'))
        
        # Check at time 106 (6 seconds, after cooldown)
        mock_time.return_value = 106.0
        self.assertTrue(self.detector.should_react_to_event('test_event'))
    
    def test_should_react_without_cooldown_check(self):
        """Test should react when cooldown check disabled."""
        # Mark reacted
        self.detector.mark_event_reacted('test_event')
        
        # Should react if not checking cooldown
        self.assertTrue(
            self.detector.should_react_to_event('test_event', check_cooldown=False)
        )


class TestEventDetectorTimeSinceReaction(unittest.TestCase):
    """Tests for time since reaction tracking."""
    
    def setUp(self):
        """Set up test detector."""
        self.detector = EventDetector()
    
    def test_time_since_never_reacted(self):
        """Test time since reaction when never reacted."""
        time_since = self.detector.get_time_since_last_reaction('test_event')
        self.assertIsNone(time_since)
    
    @patch('time.time')
    def test_time_since_last_reaction(self, mock_time):
        """Test time since last reaction."""
        # Mark reacted at time 100
        mock_time.return_value = 100.0
        self.detector.mark_event_reacted('test_event')
        
        # Check at time 110
        mock_time.return_value = 110.0
        time_since = self.detector.get_time_since_last_reaction('test_event')
        
        self.assertEqual(time_since, 10.0)


class TestEventDetectorReset(unittest.TestCase):
    """Tests for reset functionality."""
    
    def setUp(self):
        """Set up test detector with some state."""
        self.detector = EventDetector()
        self.detector.check_state_change('event1', 'value1')
        self.detector.check_state_change('event2', 'value2')
        self.detector.mark_event_reacted('event1')
    
    def test_reset_event(self):
        """Test resetting a single event."""
        self.detector.reset_event('event1')
        
        self.assertFalse(self.detector.is_event_tracked('event1'))
        self.assertTrue(self.detector.is_event_tracked('event2'))
        self.assertIsNone(
            self.detector.get_time_since_last_reaction('event1')
        )
    
    def test_reset_all(self):
        """Test resetting all events."""
        self.detector.reset_all()
        
        self.assertEqual(len(self.detector.event_states), 0)
        self.assertEqual(len(self.detector.event_last_reaction), 0)


class TestEventDetectorQueries(unittest.TestCase):
    """Tests for query methods."""
    
    def setUp(self):
        """Set up test detector."""
        self.detector = EventDetector()
        self.detector.check_state_change('test_event', 'value1')
    
    def test_get_current_state(self):
        """Test getting current state."""
        state = self.detector.get_current_state('test_event')
        self.assertEqual(state, 'value1')
    
    def test_get_current_state_not_tracked(self):
        """Test getting state of untracked event."""
        state = self.detector.get_current_state('unknown_event')
        self.assertIsNone(state)
    
    def test_is_event_tracked(self):
        """Test checking if event is tracked."""
        self.assertTrue(self.detector.is_event_tracked('test_event'))
        self.assertFalse(self.detector.is_event_tracked('unknown_event'))


class TestEventDetectorEdgeCases(unittest.TestCase):
    """Tests for edge cases."""
    
    def setUp(self):
        """Set up test detector."""
        self.detector = EventDetector()
    
    def test_none_to_value_change(self):
        """Test change from None to value."""
        change = self.detector.check_state_change('test_event', None)
        self.assertIsNone(change.old_value)
        self.assertIsNone(change.new_value)
        
        change = self.detector.check_state_change('test_event', 'value')
        self.assertIsNone(change.old_value)
        self.assertEqual(change.new_value, 'value')
    
    def test_value_to_none_change(self):
        """Test change from value to None."""
        self.detector.check_state_change('test_event', 'value')
        change = self.detector.check_state_change('test_event', None)
        
        self.assertEqual(change.old_value, 'value')
        self.assertIsNone(change.new_value)
        self.assertTrue(change.is_edge)
    
    def test_different_value_types(self):
        """Test with different value types."""
        # String
        self.detector.check_state_change('event1', 'string_value')
        # Integer
        self.detector.check_state_change('event2', 42)
        # Boolean
        self.detector.check_state_change('event3', True)
        # Dict
        self.detector.check_state_change('event4', {'key': 'value'})
        
        self.assertEqual(self.detector.get_current_state('event1'), 'string_value')
        self.assertEqual(self.detector.get_current_state('event2'), 42)
        self.assertEqual(self.detector.get_current_state('event3'), True)
        self.assertEqual(self.detector.get_current_state('event4'), {'key': 'value'})


if __name__ == '__main__':
    unittest.main()
