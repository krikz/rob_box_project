# Testing Guide: Stage 1 Monitoring Integration

**Status:** Ready for testing on robot hardware  
**Date:** October 23, 2025  
**Related Commits:** 4138b88 (code review fixes), f37ffdc (integration)

---

## 🎯 Overview

Stage 1 monitoring components have been integrated into the system:
- ✅ NodeAvailabilityMonitor - tracks ROS2 node status
- ✅ InternetConnectivityMonitor - monitors internet connectivity
- ✅ TimeAwarenessProvider - provides time-of-day context
- ✅ Integration in context_aggregator_node, reflection_node, dialogue_node

---

## 📋 Prerequisites

### 1. Rebuild Messages

The PerceptionEvent message has new fields. Rebuild the package:

```bash
cd /workspace
colcon build --packages-select rob_box_perception_msgs
source install/setup.bash
```

### 2. Rebuild Perception Package

```bash
colcon build --packages-select rob_box_perception
source install/setup.bash
```

### 3. Rebuild Voice Package (optional)

```bash
colcon build --packages-select rob_box_voice
source install/setup.bash
```

---

## 🧪 Test Scenarios

### Test 1: Time Awareness

**Goal:** Verify robot is aware of current time and period

**Steps:**
```bash
# 1. Launch context_aggregator
ros2 run rob_box_perception context_aggregator

# 2. In another terminal, check the published data
ros2 topic echo /perception/context_update --once

# 3. Verify fields are populated:
# - current_time_human: "2025-10-23 19:30:00"
# - time_period: "evening" (or "morning", "day", "night")
# - time_context_json: {...}
```

**Expected Result:**
- ✅ Time fields are populated with correct values
- ✅ Period matches current time of day (5-12: morning, 12-17: day, 17-22: evening, 22-5: night)
- ✅ Timezone is Europe/Moscow

**Logs to check:**
```
[INFO] [context_aggregator]: ✅ Monitoring components initialized
[INFO] [context_aggregator]: 📡 Node Monitor: отслеживаем X нод
[INFO] [context_aggregator]: 🌐 Internet Monitor: проверка каждые 30.0s
```

---

### Test 2: Node Monitoring

**Goal:** Verify node failure detection

**Steps:**
```bash
# 1. Launch context_aggregator
ros2 run rob_box_perception context_aggregator

# 2. Check initial status
ros2 topic echo /perception/context_update --once
# Verify: active_nodes list is populated

# 3. Kill a node (e.g., stt_node if running)
ros2 lifecycle set /stt_node shutdown

# 4. Wait 5-10 seconds, check status again
ros2 topic echo /perception/context_update --once
# Verify: /stt_node now in failed_nodes list

# 5. Restart the node
ros2 run rob_box_voice stt_node

# 6. Wait 5-10 seconds, check status
ros2 topic echo /perception/context_update --once
# Verify: /stt_node back in active_nodes list
```

**Expected Result:**
- ✅ Node transitions: active → failed → active
- ✅ health_issues includes "Упавшие ноды: /stt_node"
- ✅ system_health_status changes: healthy → degraded → healthy

**Logs to check:**
```
[ERROR] [context_aggregator]: ❌ Нода упала: /stt_node
[INFO] [context_aggregator]: ✅ Нода восстановлена: /stt_node
```

---

### Test 3: Internet Monitoring

**Goal:** Verify internet connectivity detection

**Steps:**
```bash
# 1. Launch context_aggregator
ros2 run rob_box_perception context_aggregator

# 2. Check initial status (internet available)
ros2 topic echo /perception/context_update --once
# Verify: internet_available: True

# 3. Disconnect WiFi (on Raspberry Pi)
sudo ip link set wlan0 down

# 4. Wait 30-35 seconds for check to run
ros2 topic echo /perception/context_update --once
# Verify: internet_available: False

# 5. Reconnect WiFi
sudo ip link set wlan0 up

# 6. Wait 30-35 seconds
ros2 topic echo /perception/context_update --once
# Verify: internet_available: True
```

**Expected Result:**
- ✅ internet_available reflects actual status
- ✅ health_issues includes "Нет интернета" when offline
- ✅ system_health_status: healthy → degraded (when offline)

**Logs to check:**
```
[WARN] [context_aggregator]: ⚠️ Интернет недоступен
[INFO] [context_aggregator]: ✅ Интернет восстановлен
```

---

### Test 4: Reflection Node Time Awareness

**Goal:** Verify reflection uses time context

**Steps:**
```bash
# 1. Launch context_aggregator and reflection_node
ros2 launch rob_box_perception internal_dialogue.launch.py

# 2. Send a test message to perception/user_speech
ros2 topic pub --once /perception/user_speech std_msgs/msg/String "data: 'как дела?'"

# 3. Check reflection's internal thought
ros2 topic echo /reflection/internal_thought

# 4. Check if time period is mentioned in context
# (Reflection should see "Сейчас: вечер/утро/день/ночь" in its context)
```

**Expected Result:**
- ✅ Reflection receives time_period and current_time_human
- ✅ Context includes: "🕐 Время: YYYY-MM-DD HH:MM:SS"
- ✅ Context includes: "📅 Период: morning/day/evening/night"

**Logs to check:**
```
[INFO] [reflection_node]: 🧠 Размышление: [should mention time if relevant]
```

---

### Test 5: Dialogue Node Fallback (Internet Offline)

**Goal:** Verify fallback responses when internet is unavailable

**Steps:**
```bash
# 1. Launch full voice assistant stack
ros2 launch rob_box_voice voice_assistant_headless.launch.py

# 2. Say wake word + greeting (with internet)
# "роббокс, привет"
# Should get normal AI response

# 3. Disconnect internet
sudo ip link set wlan0 down

# 4. Wait ~30 seconds for status update

# 5. Say wake word + greeting again
# "роббокс, привет"
# Should get fallback response

# 6. Reconnect internet
sudo ip link set wlan0 up

# 7. Wait ~30 seconds

# 8. Say wake word + question
# "роббокс, как дела?"
# Should get normal AI response again
```

**Expected Result:**
- ✅ With internet: Normal DeepSeek API responses
- ✅ Without internet: Simple fallback responses
- ✅ Fallback responses are appropriate:
  - "Привет! Извините, сейчас нет подключения к интернету..."
  - "Всё работает, но интернет недоступен..."

**Logs to check:**
```
[WARN] [dialogue_node]: ⚠️ Интернет недоступен - переход на fallback режим
[WARN] [dialogue_node]: ⚠️ Интернет недоступен - используем fallback
[INFO] [dialogue_node]: ✅ Интернет восстановлен - нормальный режим
```

---

## 🐛 Troubleshooting

### Issue: PerceptionEvent fields not found

**Symptom:** 
```
AttributeError: 'PerceptionEvent' object has no attribute 'current_time_human'
```

**Solution:**
```bash
# Rebuild perception_msgs
cd /workspace
colcon build --packages-select rob_box_perception_msgs --cmake-clean-cache
source install/setup.bash
```

---

### Issue: pytz not found

**Symptom:**
```
[WARN] [context_aggregator]: TimeAwarenessProvider using local time (pytz unavailable)
```

**Solution:**
```bash
pip3 install pytz
# or
sudo apt install python3-tz
```

---

### Issue: Node monitor shows all nodes as missing

**Symptom:**
```
missing_nodes: ['/audio_node', '/stt_node', ...]
```

**Solution:**
- This is normal on first startup
- Nodes transition from missing → active once they start
- Check that expected nodes are actually running: `ros2 node list`

---

### Issue: Internet always shows as unavailable

**Symptom:**
```
internet_available: False
```

**Solution:**
1. Check if ping works: `ping -c 1 8.8.8.8`
2. Check firewall settings (ICMP might be blocked)
3. Verify network interface is up: `ip addr show`

---

## ✅ Success Criteria

All tests pass when:

- [x] Time fields are correctly populated with Moscow timezone
- [x] Node monitoring detects failed and restored nodes within 5-10 seconds
- [x] Internet monitoring detects connectivity changes within 30-35 seconds
- [x] Reflection node receives and uses time/system context
- [x] Dialogue node falls back to simple responses when offline
- [x] All transitions are logged appropriately
- [x] system_health_status correctly reflects issues
- [x] health_issues list includes relevant problems

---

## 📊 Performance Metrics

Monitor these during testing:

1. **CPU Usage:**
   - context_aggregator should use <5% CPU on RPi4
   - Periodic checks (node list, ping) are lightweight

2. **Memory:**
   - No memory leaks over extended runtime
   - Monitor with: `top -p $(pgrep -f context_aggregator)`

3. **Response Time:**
   - Node failure detection: 5-10 seconds
   - Internet change detection: 30-35 seconds
   - Fallback response: <1 second (no API call)

---

## 📝 Bug Reporting

If you find issues, report with:

1. **What happened:** Describe the issue
2. **Expected behavior:** What should have happened
3. **Steps to reproduce:** Exact commands used
4. **Logs:** Relevant log excerpts
5. **Environment:** RPi model, ROS2 version, commit hash

Example:
```
Issue: Node monitor doesn't detect failed node
Expected: Should show in failed_nodes within 10 seconds
Steps:
  1. ros2 run rob_box_perception context_aggregator
  2. ros2 lifecycle set /stt_node shutdown
  3. Wait 15 seconds
  4. ros2 topic echo /perception/context_update
  
Result: /stt_node still in active_nodes

Logs:
[INFO] [context_aggregator]: 📡 Node Monitor: отслеживаем 9 нод
(no error logs about node failure)

Environment: RPi4, ROS2 kilted, commit f37ffdc
```

---

## 🚀 Next Steps After Testing

Once Stage 1 testing is complete and successful:

1. Document any issues found and fixes applied
2. Proceed to Stage 2: EquipmentHealthMonitor
3. Proceed to Stage 2: ProactiveReflectionTimer
4. Update documentation with real-world observations

---

**Created:** October 23, 2025  
**Last Updated:** October 23, 2025  
**Status:** Ready for testing
