#!/usr/bin/env python3
"""
Unit tests for DialogueManager

Tests dialogue state management, wake word detection, silence mode,
and query accumulation without ROS dependencies.
"""

import time
import pytest
from rob_box_voice.core.dialogue_manager import DialogueManager, DialogueState


class TestDialogueManagerWakeWords:
    """Test wake word detection"""
    
    def test_is_wake_word_detects_default_words(self):
        """Test detection of default wake words"""
        manager = DialogueManager()
        
        assert manager.is_wake_word('робок привет') is True
        assert manager.is_wake_word('робот как дела') is True
        assert manager.is_wake_word('роббокс поехали') is True
    
    def test_is_wake_word_ignores_non_wake_words(self):
        """Test that non-wake words are not detected"""
        manager = DialogueManager()
        
        assert manager.is_wake_word('привет') is False
        assert manager.is_wake_word('как дела') is False
        assert manager.is_wake_word('поехали') is False
    
    def test_is_wake_word_custom_words(self):
        """Test custom wake words"""
        manager = DialogueManager(wake_words=['тест', 'проверка'])
        
        assert manager.is_wake_word('тест начался') is True
        assert manager.is_wake_word('проверка связи') is True
        assert manager.is_wake_word('робок') is False
    
    def test_has_wake_word_alias(self):
        """Test has_wake_word alias works"""
        manager = DialogueManager()
        
        assert manager.has_wake_word('робок привет') is True
        assert manager.has_wake_word('привет') is False
    
    def test_remove_wake_word(self):
        """Test wake word removal"""
        manager = DialogueManager()
        
        assert manager.remove_wake_word('робок привет') == 'привет'
        assert manager.remove_wake_word('робот как дела') == 'как дела'
        assert manager.remove_wake_word('роббокс, поехали') == 'поехали'
        assert manager.remove_wake_word('привет') == 'привет'


class TestDialogueManagerSilenceMode:
    """Test silence mode management"""
    
    def test_is_silence_command(self):
        """Test silence command detection"""
        manager = DialogueManager()
        
        assert manager.is_silence_command('помолчи') is True
        assert manager.is_silence_command('замолчи пожалуйста') is True
        assert manager.is_silence_command('хватит говорить') is True
        assert manager.is_silence_command('привет') is False
    
    def test_is_unsilence_command(self):
        """Test unsilence command detection"""
        manager = DialogueManager()
        
        assert manager.is_unsilence_command('говори') is True
        assert manager.is_unsilence_command('включись') is True
        assert manager.is_unsilence_command('работай') is True
        assert manager.is_unsilence_command('отвечай') is True
        assert manager.is_unsilence_command('разговаривай') is True
        assert manager.is_unsilence_command('привет') is False
    
    def test_enable_silence(self):
        """Test enabling silence mode"""
        manager = DialogueManager()
        
        assert manager.state == DialogueState.IDLE
        assert manager.is_silenced() is False
        
        manager.enable_silence(duration=10)
        
        assert manager.state == DialogueState.SILENCED
        assert manager.is_silenced() is True
        assert manager.silence_until is not None
    
    def test_disable_silence(self):
        """Test disabling silence mode"""
        manager = DialogueManager()
        manager.enable_silence(duration=10)
        
        assert manager.is_silenced() is True
        
        manager.disable_silence()
        
        assert manager.state == DialogueState.IDLE
        assert manager.is_silenced() is False
        assert manager.silence_until is None
    
    def test_silence_expires(self):
        """Test that silence expires after duration"""
        manager = DialogueManager()
        manager.enable_silence(duration=0.1)  # 100ms
        
        assert manager.is_silenced() is True
        
        time.sleep(0.15)  # Wait for expiration
        
        assert manager.is_silenced() is False
        assert manager.state == DialogueState.IDLE


class TestDialogueManagerStateTransitions:
    """Test state machine transitions"""
    
    def test_initial_state_is_idle(self):
        """Test initial state is IDLE"""
        manager = DialogueManager()
        assert manager.state == DialogueState.IDLE
    
    def test_transition_state(self):
        """Test state transitions"""
        manager = DialogueManager()
        
        manager.transition_state(DialogueState.LISTENING)
        assert manager.state == DialogueState.LISTENING
        
        manager.transition_state(DialogueState.DIALOGUE)
        assert manager.state == DialogueState.DIALOGUE
        
        manager.transition_state(DialogueState.IDLE)
        assert manager.state == DialogueState.IDLE
    
    def test_transition_updates_interaction_time(self):
        """Test that transitions update last_interaction_time"""
        manager = DialogueManager()
        initial_time = manager.last_interaction_time
        
        time.sleep(0.01)
        manager.transition_state(DialogueState.LISTENING)
        
        assert manager.last_interaction_time > initial_time
    
    def test_check_timeout_returns_to_idle(self):
        """Test dialogue timeout"""
        manager = DialogueManager(dialogue_timeout=0.1)
        manager.transition_state(DialogueState.LISTENING)
        
        assert manager.state == DialogueState.LISTENING
        
        time.sleep(0.15)  # Wait for timeout
        timeout_occurred = manager.check_timeout()
        
        assert timeout_occurred is True
        assert manager.state == DialogueState.IDLE
    
    def test_check_timeout_no_timeout_in_idle(self):
        """Test no timeout when in IDLE"""
        manager = DialogueManager(dialogue_timeout=0.1)
        
        time.sleep(0.15)
        timeout_occurred = manager.check_timeout()
        
        assert timeout_occurred is False
        assert manager.state == DialogueState.IDLE


class TestDialogueManagerShouldRespond:
    """Test should_respond logic"""
    
    def test_should_respond_idle_requires_wake_word(self):
        """Test IDLE state requires wake word"""
        manager = DialogueManager()
        
        assert manager.should_respond('привет') is False
        assert manager.should_respond('робок привет') is True
    
    def test_should_respond_listening_always_true(self):
        """Test LISTENING state always responds"""
        manager = DialogueManager()
        manager.transition_state(DialogueState.LISTENING)
        
        assert manager.should_respond('привет') is True
        assert manager.should_respond('как дела') is True
    
    def test_should_respond_dialogue_always_true(self):
        """Test DIALOGUE state always responds"""
        manager = DialogueManager()
        manager.transition_state(DialogueState.DIALOGUE)
        
        assert manager.should_respond('привет') is True
        assert manager.should_respond('как дела') is True
    
    def test_should_respond_silenced_requires_wake_and_unsilence(self):
        """Test SILENCED state requires wake word + unsilence command"""
        manager = DialogueManager()
        manager.enable_silence()
        
        assert manager.should_respond('привет') is False
        assert manager.should_respond('робок привет') is False
        assert manager.should_respond('говори') is False
        assert manager.should_respond('робок говори') is True


class TestDialogueManagerQueryAccumulation:
    """Test query accumulation"""
    
    def test_add_query(self):
        """Test adding queries"""
        manager = DialogueManager()
        
        assert len(manager.pending_queries) == 0
        
        manager.add_query('первый вопрос')
        assert len(manager.pending_queries) == 1
        assert manager.last_query_time is not None
        
        manager.add_query('второй вопрос')
        assert len(manager.pending_queries) == 2
    
    def test_should_process_queries_empty(self):
        """Test should_process_queries with no queries"""
        manager = DialogueManager()
        
        assert manager.should_process_queries() is False
    
    def test_should_process_queries_timeout(self):
        """Test should_process_queries after timeout"""
        manager = DialogueManager(query_accumulation_timeout=0.1)
        manager.add_query('вопрос')
        
        assert manager.should_process_queries() is False
        
        time.sleep(0.15)
        
        assert manager.should_process_queries() is True
    
    def test_get_accumulated_queries(self):
        """Test getting and clearing queries"""
        manager = DialogueManager()
        manager.add_query('вопрос 1')
        manager.add_query('вопрос 2')
        
        queries = manager.get_accumulated_queries()
        
        assert len(queries) == 2
        assert 'вопрос 1' in queries
        assert 'вопрос 2' in queries
        assert len(manager.pending_queries) == 0
        assert manager.last_query_time is None
    
    def test_reset(self):
        """Test reset clears all state"""
        manager = DialogueManager()
        manager.transition_state(DialogueState.LISTENING)
        manager.add_query('вопрос')
        manager.enable_silence()
        
        manager.reset()
        
        assert manager.state == DialogueState.IDLE
        assert manager.silence_until is None
        assert len(manager.pending_queries) == 0
        assert manager.last_query_time is None


class TestDialogueManagerEdgeCases:
    """Test edge cases and error handling"""
    
    def test_empty_wake_words_list(self):
        """Test with empty wake words list — bypass mode: all input accepted"""
        manager = DialogueManager(wake_words=[])

        # Empty list = bypass mode: every message is treated as having a wake word
        assert manager.is_wake_word('робок') is True
        assert manager.is_wake_word('привет') is True
        assert manager.is_wake_word('') is True
    
    def test_wake_word_case_sensitivity(self):
        """Test wake words are case-sensitive (lowercasing is caller's responsibility)"""
        manager = DialogueManager(wake_words=['робок'])
        
        # Assuming lowercasing is done by caller
        assert manager.is_wake_word('робок') is True
        assert manager.is_wake_word('РОБОК') is False  # Different case
    
    def test_multiple_wake_words_in_text(self):
        """Test text with multiple wake words"""
        manager = DialogueManager()
        
        assert manager.is_wake_word('робок робот привет') is True
        
        # Remove first wake word
        result = manager.remove_wake_word('робок робот привет')
        # Should remove only first occurrence
        assert 'привет' in result


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
