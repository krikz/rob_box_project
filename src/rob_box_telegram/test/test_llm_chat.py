#!/usr/bin/env python3
"""DEPRECATED — Tests for the removed ``rob_box_telegram.llm_chat`` module.

After Phase 6 v2 / W7 the Telegram node no longer owns any LLM logic.
LLM chat now lives in :mod:`rob_box_harness` (DialogCore +
HarnessDeepSeekProvider / HarnessMiMoProvider). The corresponding
tests moved there. This stub is kept so existing CI discovery does not
fail with a missing module — the suite is empty on purpose.
"""

import unittest


@unittest.skip(
    "rob_box_telegram.llm_chat was removed in Phase 6 v2 / W7 — "
    "LLM logic now lives in rob_box_harness."
)
class TestLLMChatSessions(unittest.TestCase):
    def test_placeholder(self):
        pass


@unittest.skip(
    "rob_box_telegram.llm_chat was removed in Phase 6 v2 / W7 — "
    "LLM logic now lives in rob_box_harness."
)
class TestLLMChatProviders(unittest.TestCase):
    def test_placeholder(self):
        pass


@unittest.skip(
    "rob_box_telegram.llm_chat was removed in Phase 6 v2 / W7 — "
    "LLM logic now lives in rob_box_harness."
)
class TestLLMChatAPI(unittest.IsolatedAsyncioTestCase):
    async def test_placeholder(self):
        pass


if __name__ == "__main__":
    unittest.main()
