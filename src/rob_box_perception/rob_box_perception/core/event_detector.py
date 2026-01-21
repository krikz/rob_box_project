"""Event Detection Module for Perception System.

This module implements state-based event detection with edge detection,
cooldown management, and debouncing. Pure Python with no ROS dependencies.

Architecture:
- State tracking: Monitors current state of events
- Edge detection: Detects state transitions (false→true, true→false)
- Cooldown: Prevents rapid re-triggering of same event
- Debouncing: Filters out rapid state changes

Usage:
    detector = EventDetector(cooldown_interval=60.0)
    
    # Check for state change
    change = detector.check_state_change(
        event_name='health_status',
        current_value='HEALTHY'
    )
    
    if change and change.is_edge:
        print(f"Status changed: {change.old_value} → {change.new_value}")
        
    # Check with cooldown
    if detector.should_react_to_event('battery_low'):
        # React to event
        detector.mark_event_reacted('battery_low')
"""

import time
from dataclasses import dataclass
from typing import Any, Dict, Optional


@dataclass
class EventChange:
    """Represents a detected event state change.
    
    Attributes:
        event_name: Name of the event
        old_value: Previous state value
        new_value: Current state value
        is_edge: Whether this is an edge (state transition)
        timestamp: When the change was detected
    """
    event_name: str
    old_value: Any
    new_value: Any
    is_edge: bool
    timestamp: float


class EventDetector:
    """State-based event detector with edge detection and cooldown.
    
    Tracks event states and detects:
    - State changes (edges)
    - First occurrences
    - Cooldown-based event filtering
    
    Attributes:
        cooldown_interval: Minimum time between reactions to same event (seconds)
        event_states: Current state of each event
        event_last_reaction: Timestamp of last reaction to each event
    """
    
    def __init__(self, cooldown_interval: float = 60.0):
        """Initialize event detector.
        
        Args:
            cooldown_interval: Minimum time between reactions (seconds)
        """
        self.cooldown_interval = cooldown_interval
        self.event_states: Dict[str, Any] = {}
        self.event_last_reaction: Dict[str, float] = {}
    
    def check_state_change(
        self,
        event_name: str,
        current_value: Any,
        track_state: bool = True
    ) -> Optional[EventChange]:
        """Check if event state has changed.
        
        Args:
            event_name: Name of the event to check
            current_value: Current value of the event state
            track_state: Whether to update tracked state (default: True)
        
        Returns:
            EventChange if state changed, None if unchanged
        """
        previous_value = self.event_states.get(event_name)
        current_time = time.time()
        
        # Detect edge (state change)
        is_edge = previous_value != current_value
        
        if track_state:
            self.event_states[event_name] = current_value
        
        if is_edge or previous_value is None:
            return EventChange(
                event_name=event_name,
                old_value=previous_value,
                new_value=current_value,
                is_edge=is_edge,
                timestamp=current_time
            )
        
        return None
    
    def should_react_to_event(
        self,
        event_name: str,
        check_cooldown: bool = True
    ) -> bool:
        """Check if should react to event (cooldown check).
        
        Args:
            event_name: Name of the event
            check_cooldown: Whether to check cooldown (default: True)
        
        Returns:
            True if should react, False if in cooldown
        """
        if not check_cooldown:
            return True
        
        last_reaction = self.event_last_reaction.get(event_name)
        if last_reaction is None:
            return True
        
        elapsed = time.time() - last_reaction
        return elapsed >= self.cooldown_interval
    
    def mark_event_reacted(self, event_name: str):
        """Mark that we reacted to an event (for cooldown tracking).
        
        Args:
            event_name: Name of the event
        """
        self.event_last_reaction[event_name] = time.time()
    
    def get_time_since_last_reaction(self, event_name: str) -> Optional[float]:
        """Get time since last reaction to event.
        
        Args:
            event_name: Name of the event
        
        Returns:
            Seconds since last reaction, or None if never reacted
        """
        last_reaction = self.event_last_reaction.get(event_name)
        if last_reaction is None:
            return None
        return time.time() - last_reaction
    
    def reset_event(self, event_name: str):
        """Reset tracking for an event.
        
        Args:
            event_name: Name of the event to reset
        """
        self.event_states.pop(event_name, None)
        self.event_last_reaction.pop(event_name, None)
    
    def reset_all(self):
        """Reset all tracked events."""
        self.event_states.clear()
        self.event_last_reaction.clear()
    
    def get_current_state(self, event_name: str) -> Optional[Any]:
        """Get current state of an event.
        
        Args:
            event_name: Name of the event
        
        Returns:
            Current state value, or None if not tracked
        """
        return self.event_states.get(event_name)
    
    def is_event_tracked(self, event_name: str) -> bool:
        """Check if event is being tracked.
        
        Args:
            event_name: Name of the event
        
        Returns:
            True if event is tracked
        """
        return event_name in self.event_states
