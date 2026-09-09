#!/usr/bin/env python3
"""Tests for rob_box_telegram.auth module."""

import os
import unittest
from unittest.mock import AsyncMock, MagicMock, patch

from rob_box_telegram.auth import (
    authorized,
    get_allowed_users,
    is_authorized,
    reload_allowed_users,
)


class TestGetAllowedUsers(unittest.TestCase):
    """Tests for get_allowed_users() parsing."""

    @patch.dict(os.environ, {"TELEGRAM_ALLOWED_USERS": "123,456,789"})
    def test_parse_valid_ids(self):
        result = get_allowed_users()
        self.assertEqual(result, {123, 456, 789})

    @patch.dict(os.environ, {"TELEGRAM_ALLOWED_USERS": "  123 , 456 , 789  "})
    def test_parse_with_whitespace(self):
        result = get_allowed_users()
        self.assertEqual(result, {123, 456, 789})

    @patch.dict(os.environ, {"TELEGRAM_ALLOWED_USERS": ""})
    def test_empty_string(self):
        result = get_allowed_users()
        self.assertEqual(result, set())

    @patch.dict(os.environ, {"TELEGRAM_ALLOWED_USERS": "123,invalid,456"})
    def test_skip_invalid_ids(self):
        result = get_allowed_users()
        self.assertEqual(result, {123, 456})

    @patch.dict(os.environ, {}, clear=True)
    def test_env_not_set(self):
        os.environ.pop("TELEGRAM_ALLOWED_USERS", None)
        result = get_allowed_users()
        self.assertEqual(result, set())

    @patch.dict(os.environ, {"TELEGRAM_ALLOWED_USERS": "111"})
    def test_single_id(self):
        result = get_allowed_users()
        self.assertEqual(result, {111})


class TestIsAuthorized(unittest.TestCase):
    """Tests for is_authorized() check."""

    def setUp(self):
        # Reset cache
        import rob_box_telegram.auth as auth_module

        auth_module._allowed_users = None

    @patch.dict(os.environ, {"TELEGRAM_ALLOWED_USERS": "100,200"})
    def test_authorized_user(self):
        reload_allowed_users()
        self.assertTrue(is_authorized(100))
        self.assertTrue(is_authorized(200))

    @patch.dict(os.environ, {"TELEGRAM_ALLOWED_USERS": "100,200"})
    def test_unauthorized_user(self):
        reload_allowed_users()
        self.assertFalse(is_authorized(999))

    @patch.dict(os.environ, {"TELEGRAM_ALLOWED_USERS": ""})
    def test_empty_whitelist_rejects_all(self):
        reload_allowed_users()
        self.assertFalse(is_authorized(100))


class TestGroupAndDmAuth(unittest.IsolatedAsyncioTestCase):
    """Auth по группе и по личке (issue #1196, L1).

    Ключевая семантика telegram_node: ``update.effective_chat.id`` для
    группы — это chat_id ГРУППЫ (обычно отрицательный), а для лички —
    user_id собеседника. Поэтому:

    * группа авторизована, если chat_id группы есть в
      ``TELEGRAM_ALLOWED_USERS`` (любой участник группы проходит);
    * личка авторизована, если user_id есть в whitelist;
    * группа НЕ авторизована, даже если user_id отправителя в whitelist,
      но chat_id самой группы отсутствует (бот проверяет effective_chat.id).
    """

    def setUp(self):
        import rob_box_telegram.auth as auth_module

        auth_module._allowed_users = None

    @patch.dict(os.environ, {"TELEGRAM_ALLOWED_USERS": "-5269346516"})
    def test_group_chat_id_in_whitelist_authorizes(self):
        """Группа с chat_id в whitelist авторизует ВСЕХ участников."""
        reload_allowed_users()
        # Для группы effective_chat.id == chat_id группы (отрицательный).
        self.assertTrue(is_authorized(-5269346516))

    @patch.dict(os.environ, {"TELEGRAM_ALLOWED_USERS": "42,123"})
    def test_dm_user_id_in_whitelist_authorizes(self):
        """Личка авторизована, если user_id есть в whitelist."""
        reload_allowed_users()
        self.assertTrue(is_authorized(42))
        self.assertTrue(is_authorized(123))

    @patch.dict(os.environ, {"TELEGRAM_ALLOWED_USERS": "42"})
    def test_group_not_in_whitelist_rejected_even_if_sender_whitelisted(self):
        """Группа НЕ авторизована, если её chat_id не в whitelist.

        Отправитель — пользователь 42 (в whitelist), но effective_chat.id
        для группы — chat_id группы (-555), которого нет в whitelist.
        """
        reload_allowed_users()
        self.assertTrue(is_authorized(42))       # личка этого юзера — ок
        self.assertFalse(is_authorized(-555))    # группа — нет

    @patch.dict(os.environ, {"TELEGRAM_ALLOWED_USERS": "-1001234567890"})
    def test_supergroup_chat_id_authorizes(self):
        """Супергруппа (префикс -100...) авторизуется по chat_id."""
        reload_allowed_users()
        self.assertTrue(is_authorized(-1001234567890))


class TestAuthorizedDecorator(unittest.IsolatedAsyncioTestCase):
    """Tests for @authorized decorator."""

    def setUp(self):
        import rob_box_telegram.auth as auth_module

        auth_module._allowed_users = {42}

    async def test_allows_authorized_user(self):
        @authorized
        async def handler(update, context):
            return "ok"

        update = MagicMock()
        update.effective_chat.id = 42
        context = MagicMock()

        result = await handler(update, context)
        self.assertEqual(result, "ok")

    async def test_allows_authorized_group(self):
        """Групповой чат: effective_chat.id == chat_id группы в whitelist."""
        import rob_box_telegram.auth as auth_module

        auth_module._allowed_users = {-5269346516}

        @authorized
        async def handler(update, context):
            return "ok"

        update = MagicMock()
        update.effective_chat.id = -5269346516
        context = MagicMock()

        result = await handler(update, context)
        self.assertEqual(result, "ok")

    async def test_rejects_unauthorized_user(self):
        @authorized
        async def handler(update, context):
            return "ok"

        update = MagicMock()
        update.effective_chat.id = 999
        update.message.reply_text = AsyncMock()
        context = MagicMock()

        result = await handler(update, context)
        self.assertIsNone(result)
        update.message.reply_text.assert_called_once()
        call_text = update.message.reply_text.call_args[0][0]
        self.assertIn("999", call_text)
        self.assertIn("Access denied", call_text)


if __name__ == "__main__":
    unittest.main()
