# Telegram REPL Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add Telegram commands for direct Renardo REPL execution and explicit music stop.

**Architecture:** Reuse the existing Telegram command handler layer and MCP bridge. No new backend service is needed; the bot will map `/repl` to `execute_music_code` and `/stopmusic` to `stop_music`, while keeping `/music` as compatibility.

**Tech Stack:** python-telegram-bot, ROS 2 MCP bridge, unittest/pytest-style handler tests.

---

### Task 1: Add failing command-handler tests

**Files:**
- Create: `src/rob_box_telegram/test/test_commands.py`

**Step 1: Write the failing tests**
- Cover `/repl` empty usage.
- Cover `/repl` sending `execute_music_code` with full argument text.
- Cover `/stopmusic` sending `stop_music`.

**Step 2: Run tests to verify they fail**
- Run: `python3 -m pytest src/rob_box_telegram/test/test_commands.py -q`

### Task 2: Implement handlers and registration

**Files:**
- Modify: `src/rob_box_telegram/rob_box_telegram/handlers/commands.py`
- Modify: `src/rob_box_telegram/rob_box_telegram/telegram_node.py`

**Step 1: Add `/repl` handler**
- Mirror `/music` execution path, but require code text and call `execute_music_code`.

**Step 2: Add `/stopmusic` handler**
- Call `stop_music` and return result.

**Step 3: Update help/start text and command registration**
- Surface the new primary commands while keeping `/music` for compatibility.

### Task 3: Verify

**Files:**
- Test: `src/rob_box_telegram/test/test_commands.py`

**Step 1: Run targeted tests**
- Run: `python3 -m pytest src/rob_box_telegram/test/test_commands.py -q`

**Step 2: Run adjacent package tests if needed**
- Run: `python3 -m pytest src/rob_box_telegram/test/test_mcp_bridge.py src/rob_box_telegram/test/test_auth.py -q`