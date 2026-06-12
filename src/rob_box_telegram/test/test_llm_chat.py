#!/usr/bin/env python3
"""Tests for rob_box_telegram.llm_chat module."""

import os
import unittest
from unittest.mock import AsyncMock, patch

from rob_box_telegram.llm_chat import LLMChat


class TestLLMChatSessions(unittest.TestCase):
    """Tests for session management."""

    def setUp(self):
        self.chat = LLMChat(provider="deepseek", max_history=5)

    def test_clear_session(self):
        self.chat.sessions[42] = [{"role": "user", "content": "test"}]
        self.chat.clear_session(42)
        self.assertNotIn(42, self.chat.sessions)

    def test_clear_nonexistent_session(self):
        self.chat.clear_session(999)  # Should not raise

    def test_truncate_history(self):
        self.chat.sessions[1] = [{"role": "user", "content": f"msg{i}"} for i in range(10)]
        self.chat._truncate_history(1)
        self.assertEqual(len(self.chat.sessions[1]), 5)

    def test_update_tools(self):
        tools = [{"type": "function", "function": {"name": "test"}}]
        self.chat.update_tools(tools)
        self.assertEqual(self.chat.tools, tools)


class TestLLMChatProviders(unittest.TestCase):
    """Tests for provider configuration."""

    @patch.dict(os.environ, {"DEEPSEEK_API_KEY": "sk-test123"})
    def test_deepseek_api_key(self):
        chat = LLMChat(provider="deepseek")
        self.assertEqual(chat.api_key, "sk-test123")
        self.assertIn("deepseek", chat.base_url)

    @patch.dict(os.environ, {"MIMO_API_KEY": "sk-mimo456"})
    def test_mimo_api_key(self):
        chat = LLMChat(provider="mimo")
        self.assertEqual(chat.api_key, "sk-mimo456")
        self.assertIn("xiaomimimo", chat.base_url)

    @patch.dict(os.environ, {"LLM_API_KEY": "sk-unified"}, clear=True)
    def test_fallback_to_llm_api_key(self):
        os.environ.pop("DEEPSEEK_API_KEY", None)
        chat = LLMChat(provider="deepseek")
        self.assertEqual(chat.api_key, "sk-unified")


class TestLLMChatAPI(unittest.IsolatedAsyncioTestCase):
    """Tests for chat API calls."""

    @patch.dict(os.environ, {"DEEPSEEK_API_KEY": ""})
    async def test_chat_without_api_key(self):
        chat = LLMChat(provider="deepseek")
        chat.api_key = ""
        result = await chat.chat(1, "hello")
        self.assertIsNotNone(result.get("error"))


if __name__ == "__main__":
    unittest.main()
