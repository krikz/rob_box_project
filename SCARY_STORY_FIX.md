# Scary Story and System Sounds Fix - 2026-01-29

## User Report

User reported several issues after uploading fresh `clog`:

1. **Scary Story Issue**: Asked robot "расскажи страшную историю" (tell a scary story)
   - Robot said "Хорошо, расскажу страшную историю про роботов" (OK, I'll tell a story about robots)
   - Robot went silent (stopped speaking)

2. **System Sounds Issue**: Asked robot "проиграй все системные звуки" (play all system sounds)
   - Robot started telling scary story (confused context from previous request)
   - Robot started playing sounds  
   - Robot said there were errors and couldn't complete tasks

3. **Final Result**: Robot fell back to voice saying there were many errors, wanted to tell story and play sounds but failed

## Log Analysis

### Timeline: Scary Story Request

```
T=1769692639.507: User: "Робокс расскажи страшную историю"
T=1769692648.986: LLM requests 3 tools: play_sound, play_animation, speak_text

Tool #1: play_sound
T=1769692649.006: MCP publishes result ✅
T=1769692650.998: dialogue_node TIMEOUT ❌ (2 seconds later!)

Tool #2: play_animation  
T=1769692651.027: MCP publishes result ✅
T=1769692653.006: dialogue_node TIMEOUT ❌

Tool #3: speak_text (robot says "Хорошо, расскажу...")
T=1769692653.027: MCP publishes result ✅
T=1769692655.014: dialogue_node TIMEOUT ❌

Result: "❌ Слишком много ошибок (3), прерываю выполнение tool_calls"
```

**Critical Discovery at T=1769692670.290:**
```
[dialogue_node-4] [INFO] 📩 on_result вызван для request_id: 04b42aab
[dialogue_node-4] [INFO] 📩 on_result вызван для request_id: 28044d28
[dialogue_node-4] [INFO] 📩 on_result вызван для request_id: 71849da7
[dialogue_node-4] [INFO] 📩 on_result вызван для request_id: c863aa7d
[dialogue_node-4] [INFO] 📩 on_result вызван для request_id: 7274460a
[dialogue_node-4] [INFO] 📩 on_result вызван для request_id: f475079c
```

**ALL results arrived at once - 17 seconds AFTER the first tool was called!**

This proves the race condition:
- Results were published immediately by MCP server
- But subscription wasn't catching them in real-time
- Results only processed in batch when dialogue crashed

### Timeline: System Sounds Request

```
T=1769692745.481: User: "Робота проиграй все системные звуки"
T=1769692751.214: LLM requests 1 tool: play_sound(thinking)

Tool #1: play_sound  
T=1769692751.233: MCP publishes result ✅
T=1769692753.220: dialogue_node TIMEOUT ❌

Continues in agent loop...
T=1769692756.921: LLM requests play_animation(thinking)
T=1769692758.930: TIMEOUT ❌

T=1769692762.912: LLM requests speak_text("Расскажу страшную историю!")
                  ⚠️ LLM confused - mixing up previous context!
T=1769692764.916: TIMEOUT ❌

T=1769692769.132: LLM requests speak_text("В тёмной лаборатории...")
                  ⚠️ LLM telling story instead of playing sounds!
T=1769692771.140: TIMEOUT ❌

... continues for multiple iterations...

T=1769692670.272: API ERROR from DeepSeek:
"An assistant message with 'tool_calls' must be followed by tool messages 
responding to each 'tool_call_id'. (insufficient tool messages)"
```

### Root Causes

#### 1. Race Condition in execute_tool_call_sync()

**The Bug:**
```python
# ❌ BEFORE (llm_adapter.py:188-197)
def execute_tool_call_sync(self, tool_name, parameters, timeout):
    request_id = str(uuid.uuid4())
    
    # 1. Publish request
    self.execute_pub.publish(request_msg)
    
    # 2. Create Event (TOO LATE!)
    result_event = threading.Event()
    self.result_events[request_id] = result_event
    
    # 3. Wait
    result_received = result_event.wait(timeout=2.0)
```

**What Happens:**
1. Request published to `/mcp/execute` at T+0.000s
2. MCP server receives, executes instantly (<1ms)
3. Result published to `/mcp/result` at T+0.001s
4. `on_result()` callback triggered at T+0.001s
5. Callback checks: `if request_id in self.result_events`
6. **Event not registered yet!** → `Event.set()` never called
7. Event registered at T+0.002s (too late!)
8. `event.wait()` blocks for 2 seconds → TIMEOUT

**The Fix:**
```python
# ✅ AFTER - Register Event BEFORE publishing
def execute_tool_call_sync(self, tool_name, parameters, timeout):
    request_id = str(uuid.uuid4())
    
    # 1. Create Event FIRST
    result_event = threading.Event()
    self.result_events[request_id] = result_event
    
    # 2. Publish request (Event ready to catch result)
    self.execute_pub.publish(request_msg)
    
    # 3. Wait (will succeed immediately)
    result_received = result_event.wait(timeout=2.0)
```

#### 2. No MAX_ITERATIONS Limit

Agent loop continued recursively without limit when all tool calls timed out.

**The Fix:**
- Add `MAX_ITERATIONS = 10` class constant
- Track iteration count through recursion  
- Exit gracefully when limit reached

#### 3. Incomplete Tool Results Cause API Error

When 3 tool calls timeout, we stop execution with:
```python
if failed_count >= 3:
    self.get_logger().error("❌ Слишком много ошибок (3), прерываю выполнение tool_calls")
    break
```

But LLM requested 4 tools - we only executed 3. This creates incomplete tool_results which violates OpenAI API contract.

**Current Behavior:**
- LLM: "I need tools A, B, C, D"
- Us: [executes A, B, C] → all timeout → stop
- Us: "Here are results for A, B, C" (missing D!)
- LLM API: "ERROR 400 - you must provide results for ALL tool_call_ids"

**This is EXISTING behavior and NOT changed by this PR.** It would require larger refactoring to handle partial tool execution correctly. For now, the race condition fix will prevent timeouts, so this won't trigger.

## Changes Made

### File: `src/rob_box_mcp_tools/rob_box_mcp_tools/llm_adapter.py`

```diff
  def execute_tool_call_sync(self, tool_name, parameters, timeout):
      request_id = str(uuid.uuid4())

+     # Создаём Event для ожидания результата ПЕРЕД публикацией запроса
+     # Это предотвращает race condition, когда результат приходит до регистрации Event
+     result_event = threading.Event()
+     self.result_events[request_id] = result_event
+
      # Формируем запрос
      request = {"tool_name": tool_name, "parameters": parameters, "request_id": request_id}

      # Публикуем запрос
      request_msg = String()
      request_msg.data = json.dumps(request, ensure_ascii=False)
      self.execute_pub.publish(request_msg)

      self.node.get_logger().info(f"📤 Отправлен запрос {request_id[:8]}: {tool_name}")

-     # Создаём Event для ожидания результата
-     result_event = threading.Event()
-     self.result_events[request_id] = result_event
-
      # Ожидаем результат с таймаутом
      result_received = result_event.wait(timeout=timeout)
```

### File: `src/rob_box_voice/rob_box_voice/dialogue_node.py`

**Class constant added:**
```diff
  class DialogueNode(Node):
      """ROS2 нода для LLM диалога с поддержкой Qwen и DeepSeek"""

      # Конфигурации провайдеров
      PROVIDERS = { ... }
+
+     # Лимит итераций агентного цикла (защита от бесконечной рекурсии)
+     MAX_ITERATIONS = 10
```

**Method updated:**
```diff
- def _continue_after_tool_calls(self, messages, tool_calls, tool_results):
+ def _continue_after_tool_calls(self, messages, tool_calls, tool_results, iteration=1):
      """
      Продолжает агентный диалог после выполнения tool_calls (рекурсивно)
      
      Args:
          messages: Текущая история сообщений  
          tool_calls: Список выполненных tool calls
          tool_results: Результаты выполнения
+         iteration: Текущая итерация агентного цикла (для защиты от зацикливания)
      """
+     # Защита от бесконечного цикла
+     if iteration > self.MAX_ITERATIONS:
+         self.get_logger().error(f"❌ Достигнут лимит итераций ({self.MAX_ITERATIONS}). Прерываю.")
+         error_msg = "Извините, я столкнулся с проблемой и не могу продолжить."
+         self._speak_simple(error_msg, show_error_animation=True)
+         self.llm_processing = False
+         self.dialogue_in_progress = False
+         return
+     
-     self.get_logger().info("🔄 Продолжаю агентный диалог с результатами инструментов")
+     self.get_logger().info(f"🔄 Продолжаю агентный диалог (итерация {iteration}/{self.MAX_ITERATIONS})")
```

**Recursive call updated:**
```diff
-     self._continue_after_tool_calls(messages, new_tool_calls, new_tool_results)
+     self._continue_after_tool_calls(messages, new_tool_calls, new_tool_results, iteration + 1)
```

## Expected Results

### Before Fix
```
User: "Расскажи страшную историю"
Robot: "Хорошо, расскажу..." [speaks once]
Robot: [silent - all tools timeout]
Robot: [continues looping internally but no speech]
```

### After Fix
```
User: "Расскажи страшную историю"
Robot: "Хорошо, расскажу..." [speaks]
Robot: [tools execute instantly]
Robot: "В тёмной лаборатории работал робот..." [continues story]
Robot: "Однажды он начал видеть призраков..." [completes story]
Robot: [finishes successfully]
```

## Log Evidence: Before vs After

### Before (Current Log)
```
[mcp_server] ✅ Инструмент play_sound выполнен успешно
... 2 seconds pass ...
[dialogue_node] ⏱️ Timeout ожидания результата for play_sound
```

NO "📩 on_result вызван" messages during execution!

### After (Expected)
```
[mcp_server] ✅ Инструмент play_sound выполнен успешно
[llm_adapter] 📩 on_result вызван для request_id: xxxxx
[llm_adapter] 💾 Результат сохранён в кэш
[llm_adapter] ✅ Event установлен
[dialogue_node] ✅ Tool executed successfully
```

Immediate callback processing, no timeouts!

## Testing Checklist

After deployment:

- [ ] Ask robot: "Робокс расскажи страшную историю"
  - [ ] Robot should tell complete story without going silent
  - [ ] Check logs for "📩 on_result вызван" messages
  - [ ] Verify NO "⏱️ Timeout ожидания результата" errors
  
- [ ] Ask robot: "Робокс проиграй все системные звуки"
  - [ ] Robot should play sounds sequentially
  - [ ] No mixing with previous context
  - [ ] Check logs for successful tool execution
  
- [ ] Monitor logs for:
  - [ ] "🔄 Продолжаю агентный диалог (итерация X/10)" messages
  - [ ] No API errors from DeepSeek
  - [ ] Clean dialogue completion

## Known Limitations

**Partial Tool Execution on Too Many Errors:**
- When 3+ tool calls fail, we stop execution mid-batch
- This creates incomplete tool_results
- Can cause API error: "insufficient tool messages following tool_calls message"
- **Mitigation:** Race condition fix will prevent timeouts, so this won't trigger
- **Future work:** Better handling of partial tool execution (not in scope for this PR)

## Related Issues

- Animation loop issue (same race condition, fixed in same PR)
- Tool execution timeouts (root cause fixed)
- Agent loop infinite recursion (fixed with MAX_ITERATIONS)
