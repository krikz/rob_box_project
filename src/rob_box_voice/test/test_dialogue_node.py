#!/usr/bin/env python3
"""
Unit tests for Dialogue Node
Tests DeepSeek API integration and conversation flow
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import rclpy
from rclpy.node import Node
import os

# Import the node to test
# from rob_box_voice.dialogue_node import DialogueNode


class TestDialogueNode(unittest.TestCase):
    """Test suite for Dialogue Node"""

    @classmethod
    def setUpClass(cls):
        """Set up ROS2 context once for all tests"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 context"""
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures before each test"""
        # Mock API key for testing
        self.original_api_key = os.environ.get('DEEPSEEK_API_KEY')
        os.environ['DEEPSEEK_API_KEY'] = 'test-key-12345'
        
        # TODO: Uncomment when DialogueNode is ready
        # self.node = DialogueNode()

    def tearDown(self):
        """Clean up after each test"""
        # Restore original API key
        if self.original_api_key:
            os.environ['DEEPSEEK_API_KEY'] = self.original_api_key
        else:
            os.environ.pop('DEEPSEEK_API_KEY', None)
            
        # TODO: Uncomment when DialogueNode is ready
        # self.node.destroy_node()

    def test_node_creation(self):
        """Test that node can be created with API key"""
        # TODO: Implement when DialogueNode is ready
        # self.assertIsInstance(self.node, Node)
        # self.assertEqual(self.node.get_name(), 'dialogue_node')
        pass

    def test_missing_api_key_raises_error(self):
        """Test that missing API key raises RuntimeError"""
        # TODO: Implement error case
        # os.environ.pop('DEEPSEEK_API_KEY', None)
        # with self.assertRaises(RuntimeError) as context:
        #     DialogueNode()
        # self.assertIn('DEEPSEEK_API_KEY', str(context.exception))
        pass

    @patch('requests.post')
    def test_deepseek_api_call(self, mock_post):
        """Test DeepSeek API request/response"""
        # Mock API response
        mock_response = Mock()
        mock_response.status_code = 200
        mock_response.json.return_value = {
            'choices': [
                {'message': {'content': 'Привет! Как дела?'}}
            ]
        }
        mock_post.return_value = mock_response

        # TODO: Implement when DialogueNode is ready
        # response = self.node.call_deepseek_api("Привет!")
        # self.assertEqual(response, "Привет! Как дела?")
        # mock_post.assert_called_once()
        pass

    @patch('requests.post')
    def test_api_error_handling(self, mock_post):
        """Test handling of API errors"""
        # Mock API error
        mock_post.side_effect = Exception("Connection timeout")

        # TODO: Implement error case
        # with self.assertLogs(level='ERROR') as log:
        #     response = self.node.call_deepseek_api("Test")
        #     self.assertIsNone(response)
        #     self.assertIn('Connection timeout', log.output[0])
        pass

    def test_conversation_context_management(self):
        """Test that conversation context is maintained"""
        # TODO: Implement context tracking
        # self.node.add_to_context("user", "Привет")
        # self.node.add_to_context("assistant", "Здравствуй!")
        # context = self.node.get_context()
        # self.assertEqual(len(context), 2)
        # self.assertEqual(context[0]['role'], 'user')
        pass

    def test_system_prompt_injection(self):
        """Test that system prompt is properly injected"""
        # TODO: Implement system prompt test
        # messages = self.node.build_messages("Привет")
        # self.assertTrue(any(m['role'] == 'system' for m in messages))
        # system_msg = [m for m in messages if m['role'] == 'system'][0]
        # self.assertIn('робот', system_msg['content'].lower())
        pass

    def test_max_context_length(self):
        """Test that context doesn't exceed max length"""
        # TODO: Implement context limit test
        # for i in range(100):  # Add many messages
        #     self.node.add_to_context("user", f"Message {i}")
        # context = self.node.get_context()
        # self.assertLessEqual(len(context), 20)  # Max 20 messages
        pass


if __name__ == '__main__':
    unittest.main()
