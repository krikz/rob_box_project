# Audio System Troubleshooting

## Critical Error: Invalid Number of Channels

### Error Symptoms

```
Expression 'parameters->channelCount <= maxChans' failed in 'src/hostapi/alsa/pa_linux_alsa.c', line: 1514
[ERROR] Error opening OutputStream: Invalid number of channels [PaErrorCode -9998]
```

This error appears in:
- `sound_node` - when playing sound effects
- `tts_node` - when synthesizing speech

### Root Cause

The audio nodes are hardcoded to use:
- **Device 1** (assumed to be ReSpeaker Mic Array v2.0)
- **Stereo output** (2 channels)
- **16kHz sample rate**

The error occurs when:
1. Device 1 doesn't exist or is not ReSpeaker
2. The audio device doesn't support stereo output (only mono)
3. The device doesn't support 16kHz sample rate

### Diagnosis Steps

#### 1. Check Available Audio Devices

```bash
# SSH to Main Pi
sshpass -p 'open' ssh ros2@10.1.1.20

# List all audio devices
python3 -c "import sounddevice as sd; print(sd.query_devices())"
```

Expected output for ReSpeaker:
```
  0 HDA Intel PCH: ALC269VC Analog (hw:0,0), ALSA (2 in, 2 out)
> 1 ReSpeaker 4 Mic Array (UAC1.0): USB Audio (hw:1,0), ALSA (6 in, 2 out)  ← Should be device 1
  2 sysdefault, ALSA (128 in, 128 out)
```

#### 2. Check ReSpeaker Capabilities

```bash
# Check ALSA device info
cat /proc/asound/card1/stream0

# Should show:
# Playback:
#   Status: Stop
#   Interface 1
#     Altset 1
#     Format: S16_LE
#     Channels: 2      ← Must support 2 channels
#     Endpoint: 0x01 (1 OUT) (ASYNC)
#     Rates: 16000     ← Must support 16kHz
```

#### 3. Test Audio Device Directly

```bash
# Test mono playback
python3 << 'EOF'
import sounddevice as sd
import numpy as np

# Generate 1 second of 440Hz tone (mono)
duration = 1.0
fs = 16000
t = np.linspace(0, duration, int(fs * duration))
audio_mono = np.sin(2 * np.pi * 440 * t)

# Try device 1, mono
print("Testing device 1, mono (1 channel)...")
try:
    sd.play(audio_mono, samplerate=fs, device=1)
    sd.wait()
    print("✅ Mono playback successful")
except Exception as e:
    print(f"❌ Mono failed: {e}")

# Try device 1, stereo
print("Testing device 1, stereo (2 channels)...")
try:
    audio_stereo = np.column_stack((audio_mono, audio_mono))
    sd.play(audio_stereo, samplerate=fs, device=1)
    sd.wait()
    print("✅ Stereo playback successful")
except Exception as e:
    print(f"❌ Stereo failed: {e}")
EOF
```

### Solutions

#### Solution 1: Fix Device Index (Quick Fix)

If ReSpeaker is on a different device index:

1. Find correct device index from diagnosis step 1
2. Update code in **both** files:

**File: `src/rob_box_voice/rob_box_voice/sound_node.py`** (line ~211):
```python
# Change device=1 to correct index
sd.play(samples, samplerate=16000, device=CORRECT_INDEX)
```

**File: `src/rob_box_voice/rob_box_voice/tts_node.py`** (line ~494):
```python
# Change device=1 to correct index
sd.play(audio_stereo, target_rate, device=CORRECT_INDEX, blocking=False)
```

3. Rebuild Docker image:
```bash
cd docker/main/perception
docker build -t rob_box:perception-test .
```

#### Solution 2: Make Device Configurable (Best Practice)

Add ROS parameters for audio device configuration:

1. Update both nodes to accept parameters:
```python
# In __init__
self.declare_parameter("audio_device_index", 1)
self.declare_parameter("audio_sample_rate", 16000)
self.declare_parameter("audio_channels", 2)

self.audio_device = self.get_parameter("audio_device_index").value
self.audio_rate = self.get_parameter("audio_sample_rate").value
self.audio_channels = self.get_parameter("audio_channels").value
```

2. Use parameters in playback:
```python
sd.play(samples, samplerate=self.audio_rate, device=self.audio_device)
```

3. Configure in `docker/main/config/perception/perception_params.yaml`:
```yaml
sound_node:
  ros__parameters:
    audio_device_index: 1
    audio_sample_rate: 16000
    audio_channels: 2

tts_node:
  ros__parameters:
    audio_device_index: 1
    audio_sample_rate: 16000
    audio_channels: 2
```

#### Solution 3: Add Fallback to No-Audio Mode

Add error handling with graceful degradation:

```python
try:
    sd.play(samples, samplerate=16000, device=1)
    sd.wait()
except Exception as e:
    self.get_logger().warn(f"⚠️  Audio playback failed: {e}")
    self.get_logger().warn("⚠️  Continuing without audio...")
    # Don't raise - just log and continue
```

This allows the robot to function without audio if device is not available.

#### Solution 4: Disable Audio Features (Temporary Workaround)

If audio is not needed for current deployment:

1. Stop perception container:
```bash
sshpass -p 'open' ssh ros2@10.1.1.20 'cd ~/rob_box_project/docker/main && docker compose stop perception'
```

2. Or comment out audio nodes in launch file

### Hardware Checklist

For ReSpeaker Mic Array v2.0:

- [ ] USB cable properly connected to Main Pi
- [ ] `lsusb` shows "2886:0018 Seeed Technology Inc."
- [ ] `/proc/asound/card1` exists
- [ ] `aplay -l` lists "ReSpeaker 4 Mic Array"
- [ ] Device supports 2 channels (stereo) output
- [ ] Device supports 16kHz sample rate

### Testing After Fix

```bash
# SSH to Main Pi
sshpass -p 'open' ssh ros2@10.1.1.20

# Check container logs - should not see "Invalid number of channels"
docker logs perception --tail 100 | grep -i "error opening"

# Test sound effect
docker exec perception ros2 topic pub --once /voice/sound/trigger std_msgs/msg/String "{data: 'thinking'}"

# Test TTS
docker exec perception ros2 topic pub --once /voice/tts/synthesize rob_box_interfaces/msg/TTSRequest \
  "{text: 'Тест аудио системы', voice: 'ermil', speed: 1.0}"
```

### Additional Resources

- [ReSpeaker Mic Array v2.0 Documentation](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/)
- [sounddevice Python Documentation](https://python-sounddevice.readthedocs.io/)
- [ALSA Configuration Guide](https://alsa-project.org/wiki/Main_Page)

---

*Last Updated: October 29, 2025*
*Maintainer: Rob Box Project Team*
