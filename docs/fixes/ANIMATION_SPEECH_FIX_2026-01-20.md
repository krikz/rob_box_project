# Animation and Speech Interaction Fix

**Date:** 2026-01-20  
**Issue:** Robot animation blocking speech  
**Branch:** copilot/fix-locale-warning-issue

## Problem

When the LLM called `set_emotion()` followed by `speak_text()`, the robot would:
1. Show the emotion animation (e.g., "happy")
2. Wait silently for 5-10 seconds
3. Finally speak with idle/neutral animation

**Expected behavior:** Robot should speak immediately after emotion is set, showing the talking animation during speech.

## Root Cause

In `animation_player_node.py`, the `tts_state_callback()` method had an early return when `manual_animation_active = True`:

```python
if self.manual_animation_active:
    # Early return - TTS state ignored!
    return
```

This prevented the robot from switching to the "talking" animation when speech started, because `set_emotion()` had set `manual_animation_active = True`.

## Solution

**Changed:** `src/rob_box_animations/scripts/animation_player_node.py`

Removed the early return and made TTS state always take priority:

```python
def tts_state_callback(self, msg):
    state = msg.data
    
    if state in ['synthesizing', 'playing']:
        # ALWAYS switch to talking, even if manual animation is active
        if not self.is_robot_speaking:
            self.get_logger().info('🗣️ Робот говорит - переключаюсь на talking анимацию')
            self.is_robot_speaking = True
            # Load and play talking animation
            
    elif state in ['ready', 'idle', 'stopped']:
        # Return to idle after speech
        if self.is_robot_speaking:
            self.is_robot_speaking = False
            # Load and play idle animation
```

## Behavior After Fix

**Sequence:**
1. User: "робок, расскажи анекдот"
2. LLM calls `set_emotion(happy)` → 😊 happy animation starts
3. LLM calls `speak_text("Слушай...")` → **immediately** switches to 🗣️ talking animation
4. Speech plays with talking animation
5. Speech ends → returns to idle animation
6. Emotion timer continues independently (5-10s after set_emotion call)

**Key improvement:** No more waiting! Speech starts immediately.

## Testing

Run the test script to validate:

```bash
# Start animation_player first
ros2 run rob_box_animations animation_player_node.py

# In another terminal, run the test
ros2 run rob_box_animations test_animation_speech_interaction.py
```

**Expected output:**
```
✅ SUCCESS: Animation switched to talking!
✅ SUCCESS: Animation returned to idle!
```

## Files Changed

- `src/rob_box_animations/scripts/animation_player_node.py` - Main fix
- `src/rob_box_animations/scripts/test_animation_speech_interaction.py` - Test script (new)

## Related Issues

This fix resolves the complaint from logs:
```
[dialogue_node-4] [INFO] [1768891325.551629940] [dialogue_node]: 🔧 Выполнение: speak_text
[animation_player_node.py-3] [INFO] [1768891361.775917558] [voice_animation_player]: 🎨 Получен запрос на анимацию: happy
[animation_player_node.py-3] [INFO] [1768891361.782321192] [voice_animation_player]: ⏱️ Таймер возврата к idle: 6.8s
# Robot waits 6.8 seconds before speaking! ❌
```

After the fix, speech starts immediately without waiting for the animation timer.
