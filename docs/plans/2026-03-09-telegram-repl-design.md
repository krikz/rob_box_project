# Telegram Repl Design

Goal: allow Telegram operators to send Renardo/FoxDot repl code directly to the robot and stop music with an explicit command.

Design:
- Add a new `/repl` command in the Telegram bot that forwards the raw argument text to `execute_music_code` through the existing MCP bridge.
- Add a new `/stopmusic` command that calls `stop_music` through the same MCP bridge.
- Keep `/music` working as a backward-compatible alias, but document `/repl` and `/stopmusic` as the primary UX.

Data flow:
- Telegram update -> command handler in `handlers/commands.py`.
- Handler -> `node.mcp_bridge.execute_simple(...)`.
- MCP server -> `execute_music_code` or `stop_music` in the running voice-assistant pipeline.
- Result string -> Telegram chat reply.

Error handling:
- Empty `/repl` input returns usage text.
- MCP errors are passed through unchanged in the existing `execute_simple` response format.

Testing:
- Add unit tests for `/repl` success path, `/repl` empty usage path, and `/stopmusic` dispatch.
- Add assertions that `telegram_node` registers the new command handlers.