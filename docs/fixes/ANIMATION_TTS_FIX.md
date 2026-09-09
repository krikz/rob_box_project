# Animation and TTS Synchronization Fixes

**Date:** 2026-01-28  
**Branch:** `copilot/analyze-animation-issues`

## Problems Identified

### Problem 1: Animation Duration Parameter Not Parsed ✅ Fixed

**Issue:** The `play_animation` tool sends animations with duration in format `animation:duration` (e.g., `fire_truck:3`), but the `animation_player_node` was not parsing this format.

**Log Evidence:**
```
[mcp_server-9] [INFO] play_animation с параметрами {'animation': 'fire_truck', 'duration': 3}
[animation_player_node.py-3] [INFO] 🎨 Получен запрос на анимацию: fire_truck:3
```

**Fix:** 
- Modified `voice_animation_callback()` in `animation_player_node.py` to parse `animation:duration` format
- Use parsed duration for the return-to-idle timer instead of random 5-10 seconds

### Problem 2: MCP Tool Call Timeouts ✅ Fixed

**Issue:** Multiple timeout errors occurred when executing tool calls, despite tools completing instantly:

**Log Evidence:**
```
[mcp_server-9] [INFO] [1769615740.284483860] 📤 Публикую результат для play_animation
[dialogue_node-4] [ERROR] [1769615750.837998756] ⏱️ Timeout ожидания результата для play_animation
```

10+ second delay between result publication and timeout!

**Root Cause:** 
- `llm_adapter.py` used `rclpy.spin_once()` in a loop to wait for results
- This doesn't work when the node is already managed by `MultiThreadedExecutor`
- The executor runs in a separate thread and calls callbacks automatically

**Fix:**
- Replaced spin_once polling loop with simple `event.wait(timeout)`
- Reduced timeout from 10.0s to 2.0s for faster response
- Executor's background thread handles callbacks automatically

### Problem 3: LLM Repeats All Animations in Final Response ✅ Fixed

**Issue:** After showing all animations via tool calls, LLM generated a long final message repeating everything already said:

**Log Evidence:**
```
[dialogue_node-4] [INFO] 📝 LLM финальный ответ: Извините, похоже система не отвечает на вызовы инструментов. Я пытался показать вам все мои анимации...
```

**Root Cause:**
- Timeouts made LLM think tools failed
- LLM tried to summarize everything in final response

**Fix:**
- Added prompt instruction: "После показа анимаций НЕ повторяй уже сказанное! Просто скажи короткое завершение"
- With timeout fixes, tools now return success properly

## Files Changed

1. **src/rob_box_animations/scripts/animation_player_node.py**
   - Parse `animation:duration` format
   - Use duration for return-to-idle timer

2. **src/rob_box_mcp_tools/rob_box_mcp_tools/llm_adapter.py**
   - Replace spin_once with event.wait()
   - Simplified synchronization logic

3. **src/rob_box_voice/rob_box_voice/dialogue_node.py**
   - Reduce timeout from 10.0s to 2.0s (2 occurrences)

4. **src/rob_box_voice/prompts/master_prompt_compact.txt**
   - Add instruction to avoid repeating content

## Testing

### Unit Test
```bash
python3 /tmp/test_animation_parsing.py
# All tests passed ✅
```

### Syntax Check
```bash
python3 -m py_compile src/rob_box_animations/scripts/animation_player_node.py
python3 -m py_compile src/rob_box_mcp_tools/rob_box_mcp_tools/llm_adapter.py
python3 -m py_compile src/rob_box_voice/rob_box_voice/dialogue_node.py
# No syntax errors ✅
```

## Expected Behavior After Fix

1. **Animation duration respected:** Animations will show for specified duration before returning to idle
2. **No timeouts:** Tool calls complete in < 2 seconds without timeout errors
3. **Concise responses:** LLM provides short completion instead of repeating everything

## Deployment

These changes require deployment to both Vision Pi (animation_player_node, dialogue_node) and Main Pi (if MCP tools are used there).

Follow standard deployment workflow: commit → push → GitHub Actions → deploy to robots.
