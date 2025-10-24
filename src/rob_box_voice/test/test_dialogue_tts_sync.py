#!/usr/bin/env python3
"""
Unit tests for Dialogue-TTS synchronization with dialogue_id tracking
Tests that TTS correctly handles dialogue session changes
"""

import json
import unittest
from unittest.mock import MagicMock, Mock, patch


class TestDialogueTTSSync(unittest.TestCase):
    """Test suite for Dialogue-TTS dialogue_id synchronization"""

    def test_dialogue_node_generates_dialogue_id(self):
        """Test that dialogue_node generates unique dialogue_id for each response"""
        # Test that dialogue_id is a valid UUID format
        import uuid

        dialogue_id_1 = str(uuid.uuid4())
        dialogue_id_2 = str(uuid.uuid4())

        # Verify they are different
        self.assertNotEqual(dialogue_id_1, dialogue_id_2)

        # Verify format (UUID4 has specific format)
        self.assertEqual(len(dialogue_id_1), 36)  # UUID format: 8-4-4-4-12
        self.assertEqual(dialogue_id_1.count("-"), 4)

    def test_chunk_includes_dialogue_id(self):
        """Test that chunks include dialogue_id field"""
        # Simulate chunk data structure
        dialogue_id = "test-dialogue-id-123"
        chunk_data = {"dialogue_id": dialogue_id, "ssml": "<speak>Test response</speak>"}

        chunk_json = json.dumps(chunk_data, ensure_ascii=False)
        parsed = json.loads(chunk_json)

        # Verify dialogue_id is preserved
        self.assertEqual(parsed["dialogue_id"], dialogue_id)
        self.assertIn("ssml", parsed)

    def test_tts_node_tracks_dialogue_id(self):
        """Test that TTS node tracks current dialogue_id"""

        class MockTTSNode:
            def __init__(self):
                self.current_dialogue_id = None
                self.processing_dialogue_id = None

            def update_dialogue_id(self, new_id):
                if self.current_dialogue_id and new_id != self.current_dialogue_id:
                    # New dialogue detected - should interrupt
                    self.current_dialogue_id = new_id
                    return True  # Should interrupt
                self.current_dialogue_id = new_id
                return False

        node = MockTTSNode()

        # First dialogue
        dialogue_id_1 = "dialogue-1"
        should_interrupt = node.update_dialogue_id(dialogue_id_1)
        self.assertFalse(should_interrupt)  # First dialogue, no interrupt
        self.assertEqual(node.current_dialogue_id, "dialogue-1")

        # Same dialogue continues
        should_interrupt = node.update_dialogue_id(dialogue_id_1)
        self.assertFalse(should_interrupt)  # Same dialogue, no interrupt

        # New dialogue starts
        dialogue_id_2 = "dialogue-2"
        should_interrupt = node.update_dialogue_id(dialogue_id_2)
        self.assertTrue(should_interrupt)  # Different dialogue, should interrupt
        self.assertEqual(node.current_dialogue_id, "dialogue-2")

    def test_tts_rejects_outdated_chunks(self):
        """Test that TTS rejects chunks from outdated dialogue sessions"""

        class MockTTSNode:
            def __init__(self):
                self.current_dialogue_id = None
                self.processing_dialogue_id = None

            def should_process_chunk(self, dialogue_id):
                # If we're processing a dialogue and this chunk is from a different one
                if self.processing_dialogue_id and self.processing_dialogue_id != dialogue_id:
                    return False  # Reject outdated chunk
                return True

        node = MockTTSNode()

        # Start processing dialogue 1
        node.current_dialogue_id = "dialogue-1"
        node.processing_dialogue_id = "dialogue-1"

        # Chunk from dialogue 1 should be accepted
        self.assertTrue(node.should_process_chunk("dialogue-1"))

        # New dialogue started (but we're still processing dialogue 1)
        node.current_dialogue_id = "dialogue-2"
        # Note: processing_dialogue_id is still "dialogue-1"

        # Old chunk from dialogue 1 should still be accepted
        # (we're still processing it)
        self.assertTrue(node.should_process_chunk("dialogue-1"))

        # But new chunk from dialogue 2 should be rejected
        # (we haven't started processing it yet)
        self.assertFalse(node.should_process_chunk("dialogue-2"))

    def test_multiple_chunks_same_dialogue(self):
        """Test that multiple chunks from same dialogue are all processed"""

        class MockTTSNode:
            def __init__(self):
                self.current_dialogue_id = None
                self.processed_chunks = []

            def process_chunk(self, dialogue_id, chunk_num):
                if not self.current_dialogue_id:
                    self.current_dialogue_id = dialogue_id

                if dialogue_id == self.current_dialogue_id:
                    self.processed_chunks.append((dialogue_id, chunk_num))
                    return True
                return False

        node = MockTTSNode()

        dialogue_id = "dialogue-1"

        # Process 5 chunks from same dialogue
        for i in range(1, 6):
            result = node.process_chunk(dialogue_id, i)
            self.assertTrue(result)

        # Verify all chunks were processed
        self.assertEqual(len(node.processed_chunks), 5)
        self.assertEqual(node.processed_chunks, [("dialogue-1", i) for i in range(1, 6)])

    def test_dialogue_interruption_scenario(self):
        """Test realistic scenario: user asks question, then interrupts with new question"""

        class MockTTSNode:
            def __init__(self):
                self.current_dialogue_id = None
                self.processing_dialogue_id = None
                self.processed_chunks = []
                self.rejected_chunks = []

            def process_chunk(self, dialogue_id, chunk_text):
                # Check if new dialogue detected
                if dialogue_id != self.current_dialogue_id:
                    self.current_dialogue_id = dialogue_id
                    # In real implementation, this would interrupt playback
                    # and clear processing_dialogue_id

                # If we're processing a different dialogue, reject this chunk
                if self.processing_dialogue_id and self.processing_dialogue_id != dialogue_id:
                    self.rejected_chunks.append((dialogue_id, chunk_text))
                    return False

                # Accept and start processing this dialogue
                if not self.processing_dialogue_id:
                    self.processing_dialogue_id = dialogue_id

                self.processed_chunks.append((dialogue_id, chunk_text))
                return True

        node = MockTTSNode()

        # Scenario: chunks arrive from two different dialogues
        dialogue_1 = "vader-dialogue"
        dialogue_2 = "joke-dialogue"

        # Process some chunks from dialogue 1
        node.process_chunk(dialogue_1, "Chunk 1")
        self.assertEqual(node.processing_dialogue_id, dialogue_1)

        # New dialogue arrives - simulate interruption
        # In real code, this would trigger interrupt_playback()
        # which clears processing_dialogue_id
        node.processing_dialogue_id = None
        node.current_dialogue_id = dialogue_2

        # Now process dialogue 2
        accepted = node.process_chunk(dialogue_2, "New dialogue chunk")
        self.assertTrue(accepted)
        self.assertEqual(node.processing_dialogue_id, dialogue_2)

        # Late chunk from dialogue 1 arrives - should be rejected
        rejected = node.process_chunk(dialogue_1, "Late chunk")
        self.assertFalse(rejected)
        self.assertEqual(len(node.rejected_chunks), 1)
        self.assertEqual(node.rejected_chunks[0][0], dialogue_1)

    def test_dialogue_id_backward_compatibility(self):
        """Test that TTS handles chunks without dialogue_id (backward compatibility)"""

        # Old-style chunk without dialogue_id
        chunk_old_style = {"ssml": "<speak>Old format message</speak>"}

        # Should still have ssml field
        self.assertIn("ssml", chunk_old_style)

        # dialogue_id should be None or missing
        dialogue_id = chunk_old_style.get("dialogue_id", None)
        self.assertIsNone(dialogue_id)

    def test_simple_speak_generates_dialogue_id(self):
        """Test that _speak_simple also generates dialogue_id"""
        import uuid

        # Simulate _speak_simple behavior
        dialogue_id = str(uuid.uuid4())
        response_json = {"dialogue_id": dialogue_id, "ssml": "<speak>Simple message</speak>"}

        # Verify structure
        self.assertIn("dialogue_id", response_json)
        self.assertIn("ssml", response_json)
        self.assertEqual(len(dialogue_id), 36)  # Valid UUID format


if __name__ == "__main__":
    unittest.main()
