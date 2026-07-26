# QoS Mismatch Fix - Robot Reports False Failures

**Date:** 2026-01-29  
**Branch:** `copilot/debug-robot-error-messages`  
**Issue:** Robot executes commands successfully but reports failure to user

## Problem Description

User reported: "Робот прекрасно произвел пять раз милый звук и показал анимацию пожарной машины но на оба запроса он потом сказал что он не смог этого сделать!!!"

Translation: "The robot perfectly produced a cute sound five times and showed a fire truck animation, but for both requests it then said it couldn't do it!!!"

### Symptoms
- Commands execute successfully (sounds play, animations show)
- MCP server logs show successful execution
- Dialogue node reports timeout errors
- Robot tells user the command failed

### Log Evidence

```
[mcp_server-9] [INFO] ✅ Инструмент play_sound выполнен успешно
[mcp_server-9] [INFO] 📤 Публикую результат для play_sound (request_id: 291c90b0)
[mcp_server-9] [INFO] ✅ Результат опубликован на /mcp/result
[sound_node-7] [INFO] ✅ Завершено: cute

[dialogue_node-4] [ERROR] ⏱️ Timeout ожидания результата для play_sound (request_id: 291c90b0)
[dialogue_node-4] [WARN] ⚠️ play_sound вернул ошибку: Timeout ожидания результата инструмента
```

Timing analysis:
- Result published at: `[1769698313.828010016]`
- Timeout occurs at: `[1769698315.818339024]`
- **2 second timeout** despite result being published immediately

## Root Cause ✅ Identified

**QoS (Quality of Service) Profile Mismatch**

ROS 2 prevents communication between publishers and subscribers with incompatible QoS policies:

| Component | Topic | QoS Profile | Status |
|-----------|-------|-------------|--------|
| `mcp_server.py` | `/mcp/result` (pub) | **BEST_EFFORT** | ✅ Correct |
| `llm_adapter.py` | `/mcp/result` (sub) | **BEST_EFFORT** | ✅ Correct |
| `deepseek_adapter.py` | `/mcp/result` (sub) | **RELIABLE** (default) | ❌ Wrong! |

### Why This Breaks Communication

In ROS 2, a subscriber with RELIABLE QoS **cannot** receive messages from a publisher with BEST_EFFORT QoS. This is by design - RELIABLE guarantees delivery, BEST_EFFORT doesn't.

The mismatch causes:
1. MCP server publishes result with BEST_EFFORT
2. DeepSeek adapter's RELIABLE subscriber rejects the message
3. Callback `on_result()` never fires
4. `execute_tool_call_sync()` times out waiting
5. Returns error to LLM
6. LLM tells user the command failed

## Solution ✅ Implemented

### File: `src/rob_box_mcp_tools/rob_box_mcp_tools/deepseek_adapter.py`

**Before:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DeepSeekToolCallAdapter:
    def __init__(self, node: Node):
        self.node = node
        
        # Default QoS = RELIABLE
        self.execute_pub = node.create_publisher(String, "/mcp/execute", 10)
        self.result_sub = node.create_subscription(String, "/mcp/result", self.on_result, 10)
```

**After:**
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

class DeepSeekToolCallAdapter:
    def __init__(self, node: Node):
        self.node = node
        
        # QoS для минимизации задержек в Zenoh (совпадает с MCP server)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.execute_pub = node.create_publisher(String, "/mcp/execute", qos_profile)
        self.result_sub = node.create_subscription(String, "/mcp/result", self.on_result, qos_profile)
```

### Changes Made
1. Added QoSProfile imports
2. Created qos_profile matching MCP server (BEST_EFFORT)
3. Applied qos_profile to both publisher and subscriber
4. Added comment explaining the purpose

## Impact

**Before Fix:**
- 5 tool calls: 5 timeouts = 100% failure rate
- User receives false error messages
- Poor user experience despite correct execution

**After Fix:**
- Tool calls should complete without timeouts
- User receives correct success confirmations
- Proper feedback matching actual execution

## Why BEST_EFFORT is Correct

The system uses **Zenoh DDS** bridge which works better with BEST_EFFORT:
- Lower latency (no retransmission overhead)
- Better for real-time robot control
- Acceptable for voice commands (idempotent operations)
- Consistent with all other MCP communication

## Testing Recommendations

1. **Manual Test:** Ask robot to play sound multiple times
   - Verify sound plays
   - Verify robot confirms success (not error)

2. **Manual Test:** Ask robot to show animations
   - Verify animation displays
   - Verify robot confirms success

3. **Log Verification:** Check dialogue_node logs
   - Should see: `✅ Результат получен`
   - Should NOT see: `⏱️ Timeout ожидания результата`

4. **Integration Test:** Test various MCP tools
   - Navigation commands
   - Volume/pitch adjustments
   - Status queries
   - All should report correct success/failure

## Related Files

- `src/rob_box_mcp_tools/rob_box_mcp_tools/mcp_server.py` - Uses BEST_EFFORT (correct)
- `src/rob_box_mcp_tools/rob_box_mcp_tools/llm_adapter.py` - Uses BEST_EFFORT (correct)
- `src/rob_box_mcp_tools/rob_box_mcp_tools/deepseek_adapter.py` - **Fixed to use BEST_EFFORT**

## Prevention

To avoid similar issues in the future:

1. **Always check QoS** when creating publishers/subscribers to MCP topics
2. **Use BEST_EFFORT** for all MCP communication (documented in mcp_server.py)
3. **Test with logs** - look for timeout errors after successful execution
4. **Use `ros2 topic info -v <topic>`** to inspect QoS settings

## References

- ROS 2 QoS Documentation: https://docs.ros.org/en/kilted/Concepts/About-Quality-of-Service-Settings.html
- Zenoh DDS Bridge: https://github.com/eclipse-zenoh/zenoh-plugin-dds
- Original issue analysis in clog file (lines showing timeout errors)
