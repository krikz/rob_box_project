# Fix: Robot Sound and Speech Repetition Issues

**Date:** 2025-10-23  
**Branch:** `copilot/fix-robot-sound-issue`  
**Issue:** Robot constantly playing "cute" sound and repeating all speech twice

---

## 🔍 Problem Analysis

### Issue #1: Repetitive "Cute" Sound

**Symptom:** Robot plays "cute" sound every 3-4 seconds continuously.

**Root Cause:**
1. `reflection_node` generates thoughts containing words like "healthy", "норма", "готов" every time it receives a context update
2. `_trigger_sound_for_thought()` matches these words and calls `_play_sound('cute')`
3. `context_aggregator` publishes at 2 Hz (every 0.5 seconds)
4. **Critical bug:** `_play_sound()` method had NO debounce logic, despite `sound_debounce_interval = 10.0` being defined

**Evidence from logs:**
```
[reflection_node-3] [INFO] [1761210960.024369299] [reflection_node]: 🤖 AI: thought="Система в норме, statushealthy..."
[sound_node-6] [INFO] [1761210960.033337279] [sound_node]: 🔔 Триггер: cute
[reflection_node-3] [INFO] [1761210963.763389289] [reflection_node]: 🤖 AI: thought="Система остается в здоровом состоянии..."
[sound_node-6] [INFO] [1761210979.064977577] [sound_node]: 🔔 Триггер: cute
[sound_node-6] [INFO] [1761210982.790641657] [sound_node]: 🔔 Триггер: cute
[sound_node-6] [INFO] [1761210986.990537057] [sound_node]: 🔔 Триггер: cute
```

### Issue #2: Speech Repeated Twice

**Symptom:** Robot says everything twice (both TTS and dialogue responses).

**Root Cause:**
1. `dialogue_node` published to **BOTH** topics:
   - `/voice/dialogue/response` (intended for dialogue system)
   - `/voice/tts/request` (intended for direct TTS requests)
2. `tts_node` subscribed to **BOTH** topics with the **SAME callback**:
   ```python
   self.dialogue_sub = self.create_subscription(String, '/voice/dialogue/response', self.dialogue_callback, 10)
   self.tts_request_sub = self.create_subscription(String, '/voice/tts/request', self.dialogue_callback, 10)
   ```
3. Result: Every message from `dialogue_node` was processed **TWICE** by `tts_node`

**Architecture flaw:**
- `/voice/dialogue/response` should be for dialogue responses (dialogue_node → tts_node)
- `/voice/tts/request` should be for direct TTS requests (reflection_node → tts_node)
- `dialogue_node` incorrectly published to both, causing duplication

---

## ✅ Solutions Implemented

### Fix #1: Sound Debounce in reflection_node.py

**File:** `src/rob_box_perception/rob_box_perception/reflection_node.py`

**Changes:**
```python
def _play_sound(self, sound_name: str):
    """Проиграть звуковой эффект (с debounce защитой от повторов)"""
    # Проверка debounce: не играть один и тот же звук слишком часто
    current_time = time.time()
    if sound_name in self.last_sound_time:
        time_since_last = current_time - self.last_sound_time[sound_name]
        if time_since_last < self.sound_debounce_interval:
            self.get_logger().debug(
                f'🔇 Sound debounce: пропускаю {sound_name} '
                f'(прошло {time_since_last:.1f}s < {self.sound_debounce_interval}s)'
            )
            return  # НЕ публикуем звук
    
    try:
        msg = String()
        msg.data = sound_name
        self.sound_pub.publish(msg)
        self.get_logger().debug(f'🎵 Звук: {sound_name}')
        
        # Обновляем время последнего воспроизведения этого звука
        self.last_sound_time[sound_name] = current_time
    except Exception as e:
        self.get_logger().warn(f'⚠️  Ошибка триггера звука: {e}')
```

**Effect:**
- Each sound tracked independently in `last_sound_time` dictionary
- Same sound cannot play more frequently than `sound_debounce_interval` (10 seconds)
- Different sounds (cute, thinking, angry, etc.) have independent timers
- Logs debug message when sound is blocked by debounce

### Fix #2: Remove Duplicate Publishing in dialogue_node.py

**File:** `src/rob_box_voice/rob_box_voice/dialogue_node.py`

**Changes:**

**Location 1 - `_speak_simple()` method (line 278-279):**
```diff
    response_msg = String()
    response_msg.data = json.dumps(response_json, ensure_ascii=False)
    self.response_pub.publish(response_msg)
-   self.tts_pub.publish(response_msg)
+   # NOTE: НЕ публикуем в tts_pub - tts_node уже подписан на response_pub
```

**Location 2 - Main dialogue processing (line 519-522):**
```diff
    response_msg = String()
    response_msg.data = json.dumps(chunk_data, ensure_ascii=False)
    self.response_pub.publish(response_msg)
-   
-   # Публикуем в TTS для синтеза (Phase 6)
-   self.tts_pub.publish(response_msg)
+   # NOTE: НЕ публикуем в tts_pub - tts_node уже подписан на response_pub
+   
    self.get_logger().info(f'🔊 Отправлено в TTS: chunk {chunk_count}')
```

**Effect:**
- `dialogue_node` now only publishes to `/voice/dialogue/response`
- `tts_node` receives each message **once** via `/voice/dialogue/response`
- `/voice/tts/request` remains available for direct TTS requests from other nodes

---

## 🎯 Expected Behavior After Fix

### Sound System:
✅ "cute" sound plays at most once every 10 seconds  
✅ Other sounds (thinking, surprise, confused, angry) tracked independently  
✅ Emotion-based sound triggering preserved  
✅ No spamming of repetitive sounds  

### Speech System:
✅ All speech plays ONCE, not twice  
✅ Clean topic separation:
  - `/voice/dialogue/response` - Dialogue system → TTS
  - `/voice/tts/request` - Direct TTS requests → TTS  
✅ No duplicate TTS processing  

---

## 🧪 Testing Recommendations

### 1. Test Sound Debounce:
```bash
# SSH to Vision Pi
sshpass -p '<ROBOT_PASSWORD>' ssh ros2@10.1.1.21

# Monitor reflection_node logs
docker logs -f perception | grep "Sound debounce\|Триггер\|Звук:"
```

**Expected:** "cute" sound should only appear once every 10+ seconds even with frequent "healthy" thoughts.

### 2. Test Speech (No Duplication):
```bash
# Monitor TTS logs
docker logs -f voice-assistant | grep "TTS:\|dialogue_callback"
```

**Expected:** Each dialogue response should only appear once in TTS processing.

### 3. Test Different Sounds:
- Trigger different emotions in reflection thoughts
- Verify each sound has independent debounce timer
- "cute" blocked doesn't block "thinking", etc.

---

## 📊 Statistics

**Files Changed:** 2  
**Lines Added:** 17  
**Lines Removed:** 4  
**Net Change:** +13 lines  

**Commits:**
1. `62ddd22` - fix: implement sound debounce in reflection_node to prevent repetitive sounds
2. `cb15c3c` - fix: remove duplicate TTS publishing in dialogue_node causing speech repetition

---

## 🏗️ Architecture Notes

### Topic Flow (After Fix):

```
┌─────────────────┐
│ dialogue_node   │
└────────┬────────┘
         │ publishes to
         ├──> /voice/dialogue/response ──┐
         │                               │
┌────────┴─────────┐                     │
│ reflection_node  │                     ▼
└────────┬─────────┘              ┌──────────┐
         │ publishes to           │ tts_node │
         └──> /voice/tts/request ─┤          │
                                  └──────────┘
                                       │
                                       ▼
                                  [Audio Output]
```

### Sound Debounce (After Fix):

```python
last_sound_time = {
    'cute': 1761210960.0,      # Last played at timestamp
    'thinking': 1761210955.0,
    'angry': 1761210945.0,
    # ... independent tracking per sound
}

# When playing 'cute' again:
if current_time - last_sound_time['cute'] < 10.0:
    # Skip! Too soon
else:
    # Play and update timestamp
```

---

## 📝 Related Issues

- Architecture documentation: `docs/architecture/VOICE_SYSTEM.md`
- ROS 2 topic design: `docs/architecture/TOPICS.md`
- Perception system: `docs/packages/rob_box_perception/README.md`

---

## ✅ Code Review Checklist

- [x] Changes are minimal and surgical
- [x] Pre-existing code style preserved
- [x] No new linting errors introduced
- [x] Debounce logic tested with simulation
- [x] Comments added explaining architectural decisions
- [x] No breaking changes to existing functionality
- [x] Topic separation maintained (/dialogue/response vs /tts/request)

---

## 🎓 Lessons Learned

1. **Always implement debounce/rate limiting** when triggering actions based on frequent events
2. **Avoid publishing to multiple topics** when subscribers overlap - leads to duplication
3. **Clear topic separation** is critical for maintainability (dialogue vs direct TTS)
4. **Monitor logs carefully** - repetition patterns indicate architectural issues
5. **Test with real-world scenarios** - 2 Hz context updates revealed the bug

---

**Status:** ✅ Ready for merge to `develop`
