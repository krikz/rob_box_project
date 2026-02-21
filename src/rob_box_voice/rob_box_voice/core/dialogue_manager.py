#!/usr/bin/env python3
"""
dialogue_manager.py - Dialogue State Management

Purpose:
    Manages dialogue state machine, wake word detection, silence mode,
    and query accumulation for the voice assistant.

Public Interface:
    - DialogueManager: Main class for dialogue state management
    - DialogueState: Enum for dialogue states

Example:
    >>> from rob_box_voice.core.dialogue_manager import DialogueManager
    >>> manager = DialogueManager(wake_words=['робок', 'робот'])
    >>> manager.is_wake_word('робок привет')
    True
    >>> manager.should_respond('привет')
    False  # No wake word in IDLE state

Tests:
    See: test/unit/core/test_dialogue_manager.py

Dependencies:
    - time (standard library)
    - typing (standard library)
"""

import time
from enum import Enum
from typing import List, Optional


class DialogueState(Enum):
    """Dialogue state machine states"""
    IDLE = "IDLE"              # Waiting for wake word
    LISTENING = "LISTENING"    # Wake word detected, listening for command
    DIALOGUE = "DIALOGUE"      # Processing user request
    SILENCED = "SILENCED"      # Robot is silent (помолчи command)


class DialogueManager:
    """
    Manages dialogue state and wake word detection.
    
    This class handles the state machine for voice interactions,
    wake word detection, silence mode, and query accumulation.
    
    Attributes:
        wake_words: List of wake word phrases
        silence_commands: List of silence trigger phrases
        unsilence_commands: List of phrases to exit silence mode
        state: Current dialogue state
        silence_until: Timestamp when silence mode ends (None if not silenced)
        last_interaction_time: Timestamp of last user interaction
        dialogue_timeout: Seconds of inactivity before returning to IDLE
        pending_queries: List of accumulated queries
        query_accumulation_timeout: Seconds to wait for more queries
        last_query_time: Timestamp of last query
    
    Methods:
        is_wake_word: Check if text contains wake word
        has_wake_word: Alias for is_wake_word
        remove_wake_word: Remove wake word from text
        is_silence_command: Check if text is silence command
        is_unsilence_command: Check if text is unsilence command
        should_respond: Check if should respond based on state
        transition_state: Transition to new state
        enable_silence: Enable silence mode for duration
        disable_silence: Disable silence mode
        is_silenced: Check if currently in silence mode
        check_timeout: Check and handle dialogue timeout
        add_query: Add query to accumulation queue
        should_process_queries: Check if should process accumulated queries
        get_accumulated_queries: Get and clear accumulated queries
        reset: Reset to initial state
    """
    
    def __init__(
        self,
        wake_words: Optional[List[str]] = None,
        silence_commands: Optional[List[str]] = None,
        unsilence_commands: Optional[List[str]] = None,
        dialogue_timeout: float = 30.0,
        query_accumulation_timeout: float = 2.5
    ):
        """
        Initialize DialogueManager.
        
        Args:
            wake_words: List of wake word phrases (default: ['робок', 'робот', 'роббокс'])
            silence_commands: List of silence triggers (default: ['помолч', 'замолч', 'хватит'])
            unsilence_commands: List of unsilence triggers (default: ['говори', 'включ', 'работ'])
            dialogue_timeout: Seconds before returning to IDLE (default: 30.0)
            query_accumulation_timeout: Seconds to accumulate queries (default: 2.5)
        """
        # Wake words
        self.wake_words = wake_words if wake_words is not None else ['робок', 'робот', 'роббокс']
        self.silence_commands = silence_commands or ['помолч', 'замолч', 'хватит']
        self.unsilence_commands = unsilence_commands or ['говори', 'включ', 'работ', 'отвеч', 'разговар']
        
        # State machine
        self.state = DialogueState.IDLE
        self.silence_until: Optional[float] = None
        self.last_interaction_time = time.time()
        self.dialogue_timeout = dialogue_timeout
        
        # Query accumulation
        self.pending_queries: List[str] = []
        self.query_accumulation_timeout = query_accumulation_timeout
        self.last_query_time: Optional[float] = None
    
    def is_wake_word(self, text: str) -> bool:
        """
        Check if text contains any wake word.

        If wake_words list is empty, returns True (bypass mode — accept all input).

        Args:
            text: Text to check (should be lowercased)

        Returns:
            True if wake word found or wake_words is empty, False otherwise
        """
        if not self.wake_words:
            return True
        return any(wake_word in text for wake_word in self.wake_words)
    
    def has_wake_word(self, text: str) -> bool:
        """Alias for is_wake_word for better readability."""
        return self.is_wake_word(text)
    
    def remove_wake_word(self, text: str) -> str:
        """
        Remove wake word from text.
        
        Args:
            text: Text containing wake word (should be lowercased)
        
        Returns:
            Text with wake word removed and stripped
        """
        for wake_word in self.wake_words:
            if wake_word in text:
                text = text.replace(wake_word, "").strip()
        
        # Clean up extra spaces and punctuation
        text = " ".join(text.split())
        text = text.lstrip(",.!?;: ")
        
        return text.strip()
    
    def is_silence_command(self, text: str) -> bool:
        """
        Check if text is a silence command.
        
        Args:
            text: Text to check (should be lowercased)
        
        Returns:
            True if silence command detected
        """
        return any(cmd in text for cmd in self.silence_commands)
    
    def is_unsilence_command(self, text: str) -> bool:
        """
        Check if text is an unsilence command.
        
        Args:
            text: Text to check (should be lowercased)
        
        Returns:
            True if unsilence command detected
        """
        return any(cmd in text for cmd in self.unsilence_commands)
    
    def should_respond(self, text: str) -> bool:
        """
        Check if should respond to text based on current state.
        
        Args:
            text: User input text (should be lowercased)
        
        Returns:
            True if should respond, False otherwise
        
        Raises:
            None
        """
        # Check if silenced
        if self.is_silenced():
            # In silence mode, only respond to unsilence + wake word
            return self.has_wake_word(text) and self.is_unsilence_command(text)
        
        # In IDLE, need wake word
        if self.state == DialogueState.IDLE:
            return self.has_wake_word(text)
        
        # In LISTENING or DIALOGUE, always respond
        return self.state in (DialogueState.LISTENING, DialogueState.DIALOGUE)
    
    def transition_state(self, new_state: DialogueState):
        """
        Transition to new state.
        
        Args:
            new_state: Target state
        """
        self.state = new_state
        self.last_interaction_time = time.time()
    
    def enable_silence(self, duration: float = 300.0):
        """
        Enable silence mode for duration.
        
        Args:
            duration: Duration in seconds (default: 300 = 5 minutes)
        """
        self.state = DialogueState.SILENCED
        self.silence_until = time.time() + duration
    
    def disable_silence(self):
        """Disable silence mode and return to IDLE."""
        self.state = DialogueState.IDLE
        self.silence_until = None
    
    def is_silenced(self) -> bool:
        """
        Check if currently in silence mode.
        
        Returns:
            True if silenced and not expired, False otherwise
        """
        if self.state != DialogueState.SILENCED:
            return False
        
        # Check if silence expired
        if self.silence_until and time.time() >= self.silence_until:
            self.disable_silence()
            return False
        
        return True
    
    def check_timeout(self) -> bool:
        """
        Check if dialogue has timed out.
        
        Returns:
            True if timeout occurred and state changed to IDLE
        """
        if self.state in (DialogueState.LISTENING, DialogueState.DIALOGUE):
            elapsed = time.time() - self.last_interaction_time
            if elapsed > self.dialogue_timeout:
                self.state = DialogueState.IDLE
                return True
        return False
    
    def add_query(self, query: str):
        """
        Add query to accumulation queue.
        
        Args:
            query: User query to add
        """
        self.pending_queries.append(query)
        self.last_query_time = time.time()
    
    def should_process_queries(self) -> bool:
        """
        Check if should process accumulated queries.
        
        Returns:
            True if timeout reached or no queries pending
        """
        if not self.pending_queries:
            return False
        
        if not self.last_query_time:
            return True
        
        elapsed = time.time() - self.last_query_time
        return elapsed >= self.query_accumulation_timeout
    
    def get_accumulated_queries(self) -> List[str]:
        """
        Get accumulated queries and clear queue.
        
        Returns:
            List of accumulated queries
        """
        queries = self.pending_queries.copy()
        self.pending_queries.clear()
        self.last_query_time = None
        return queries
    
    def reset(self):
        """Reset manager to initial state."""
        self.state = DialogueState.IDLE
        self.silence_until = None
        self.last_interaction_time = time.time()
        self.pending_queries.clear()
        self.last_query_time = None


# Example usage
if __name__ == '__main__':
    # Create manager
    manager = DialogueManager()
    
    # Test wake word detection
    print("Wake word tests:")
    print(f"  'робок привет' -> {manager.is_wake_word('робок привет')}")  # True
    print(f"  'привет' -> {manager.is_wake_word('привет')}")  # False
    
    # Test state transitions
    print("\nState transitions:")
    print(f"  Initial state: {manager.state}")
    manager.transition_state(DialogueState.LISTENING)
    print(f"  After wake word: {manager.state}")
    
    # Test silence mode
    print("\nSilence mode:")
    manager.enable_silence(duration=5)
    print(f"  Is silenced: {manager.is_silenced()}")
    print(f"  Should respond to 'привет': {manager.should_respond('привет')}")
