# Known Deployment Warnings and Non-Issues

This document explains warnings and errors that appear in deployment logs but are **not critical issues** and can be safely ignored. The deployment workflow has been updated to filter these out automatically.

## Vision Pi

### 1. micro-ros-agent: Serial Port /dev/ttyUSB0 Not Available

**Error Message:**
```
❌ ERROR: Serial port /dev/ttyUSB0 still not available after 30s
⚠️  WARNING: Serial port /dev/ttyUSB0 not found!
```

**Explanation:**
- The ESP32 sensor hub is an optional component that provides additional sensor data
- The robot can function normally without it
- This error appears when the ESP32 is not physically connected via USB

**Impact:** None - robot operates normally without ESP32 sensors

**Resolution:** Connect ESP32 sensor hub if additional sensors are needed, otherwise ignore

---

### 2. cAdvisor: Machine ID and Vendor Warnings

**Error Messages:**
```
E1029 09:14:18.228529 1 info.go:119] Failed to get system UUID: open /etc/machine-id: no such file or directory
W1029 09:14:18.228606 1 machine.go:65] Cannot read vendor id correctly, set empty.
W1029 09:14:30.348293 1 sysinfo.go:203] Nodes topology is not available, providing CPU topology
```

**Explanation:**
- cAdvisor runs inside a Docker container and cannot access certain host system files
- These files are not critical for container monitoring functionality
- This is expected behavior for containerized cAdvisor

**Impact:** None - cAdvisor still collects all necessary container metrics

**Resolution:** Ignore - this is normal for containerized environments

---

### 3. Promtail: Old Container References

**Error Messages:**
```
level=error ts=2025-10-29T09:14:44.376471455Z caller=target.go:100 msg="could not inspect container info" 
container=ca330cc2b2d052d0b2651efc35bb6e5ff1f64e70dec859e67d5458468511e683 
err="Error response from daemon: No such container: ca330cc2b2d052d0b2651efc35bb6e5ff1f64e70dec859e67d5458468511e683"
```

**Explanation:**
- Promtail maintains references to containers that existed before restart
- When containers are restarted with `docker compose down && up`, old container IDs become invalid
- Promtail automatically cleans up these references after a brief period

**Impact:** Minimal - logs from old containers are already collected; only affects cleanup

**Resolution:** Ignore - Promtail will automatically update to track new containers

---

### 4. Loki: Timestamp Entry Warnings

**Error Messages:**
```
level=error ts=2025-10-29T09:14:09.986113151Z caller=client.go:430 component=client host=10.1.1.249:3100 
msg="final error sending batch" status=400 tenant= 
error="server returned HTTP status 400 Bad Request (400): 
entry with timestamp 2025-10-29 08:06:04.416526133 +0000 UTC ignored, 
reason: 'entry too far behind, oldest acceptable timestamp is: 2025-10-29T08:14:08Z'"
```

**Explanation:**
- After deployment restart, Promtail may try to forward old log entries
- Loki has a time window limit for accepting log entries
- Old entries outside this window are rejected

**Impact:** None - historical logs are preserved; only duplicate forwarding is rejected

**Resolution:** Ignore - normal behavior after system restart

---

## Main Pi

### 1. Perception Sound/TTS Audio Configuration Errors ⚠️ NEEDS FIX

**Error Messages:**
```
Expression 'parameters->channelCount <= maxChans' failed in 'src/hostapi/alsa/pa_linux_alsa.c', line: 1514
[ERROR] sound_node (9s ago): ❌ Ошибка воспроизведения thinking: Error opening OutputStream: Invalid number of channels [PaErrorCode -9998]
[ERROR] tts_node (4s ago): ❌ Synthesis error: Error opening OutputStream: Invalid number of channels [PaErrorCode -9998]
```

**Explanation:**
- **This is a REAL configuration issue that needs to be fixed**
- The code is configured to output stereo audio (2 channels) at 16kHz to device 1 (ReSpeaker)
- The error "Invalid number of channels" indicates the audio device doesn't support the configured number of channels
- This happens when:
  - Audio device is not available at all
  - Audio device exists but doesn't support stereo (only mono)
  - Wrong device index is specified (device 1 may not be ReSpeaker)

**Impact:** Audio features (sound effects and TTS) will not work until this is fixed

**Resolution:**

**Option 1: Configure proper audio device (for production with audio)**
1. Check available audio devices:
   ```bash
   python3 -c "import sounddevice as sd; print(sd.query_devices())"
   ```

2. Find the correct device index for ReSpeaker (or your audio output device)

3. Update the code to use correct device and channel configuration:
   - For ReSpeaker: should be stereo (2 channels), 16kHz
   - For other devices: may need mono (1 channel) or different sample rate

4. Fix in `src/rob_box_voice/rob_box_voice/sound_node.py` line 211:
   ```python
   sd.play(samples, samplerate=16000, device=1, channels=2)  # Verify device index
   ```

5. Fix in `src/rob_box_voice/rob_box_voice/tts_node.py` line 494:
   ```python
   sd.play(audio_stereo, target_rate, device=1, blocking=False)  # Verify device index
   ```

**Option 2: Disable audio features (for headless operation)**
1. Stop perception container that has audio:
   ```bash
   docker compose stop perception
   ```

2. Add environment variable to disable audio:
   ```yaml
   environment:
     - ENABLE_AUDIO=false
   ```

**Option 3: Make audio device configurable (recommended fix)**
1. Add ROS parameters for audio device configuration
2. Allow fallback to no-audio mode if device is not available
3. Detect audio capabilities at runtime and configure accordingly

**This error should NOT be ignored** - it indicates either missing hardware or misconfiguration.

---

### 2. Nav2 Costmap: Robot Out of Bounds Warnings

**Error Messages:**
```
[WARN] [1761729332.822083288] [nav2_costmap_2d]: Robot is out of bounds of the costmap!
[WARN] [1761729332.822201806] [global_costmap.global_costmap]: 
Sensor origin at (0.25, -0.00) is out of map bounds (0.00, 0.00) to (4.98, 4.98). 
The costmap cannot raytrace for it.
```

**Explanation:**
- These warnings appear during robot startup before proper localization
- At initialization, robot's estimated position may be (0, 0) which could be outside the loaded map
- Once RTAB-Map and localization initialize, robot position updates and warnings stop

**Impact:** Temporary - warnings stop once localization is established (typically within 30-60 seconds)

**Resolution:** Wait for localization to initialize - warnings will disappear automatically

---

### 3. Nav2 Costmap: Static Map Not Received

**Error Message:**
```
[WARN] [1761729341.822131703] [global_costmap.global_costmap]: Can't update static costmap layer, no map received
```

**Explanation:**
- At startup, Nav2 waits for RTAB-Map to publish the map
- This warning is expected during the initialization sequence
- Once RTAB-Map starts SLAM, the map becomes available

**Impact:** Temporary - resolves when RTAB-Map publishes the first map update

**Resolution:** Normal startup sequence - ignore

---

### 4. ros2_control: CAN Controller ERROR-ACTIVE State

**Error Message:**
```
CAN controller state: ERROR-ACTIVE
```

**Explanation:**
- CAN bus has three operational states: ERROR-ACTIVE, ERROR-PASSIVE, and BUS-OFF
- **ERROR-ACTIVE is the normal operational state**
- It means the controller is actively monitoring and handling errors on the bus
- This is NOT an error - it's the expected healthy state

**Impact:** None - this is normal CAN operation

**Resolution:** Ignore - this indicates healthy CAN bus operation

---

### 5. KDL Parser: Root Link Inertia Warning

**Error Message:**
```
[WARN] [1761729326.512411427] [kdl_parser]: 
The root link base_link has an inertia specified in the URDF, 
but KDL does not support a root link with an inertia. 
As a workaround, you can add an extra dummy link to your URDF.
```

**Explanation:**
- KDL (Kinematics and Dynamics Library) has a limitation with root link inertia
- The URDF design specifies inertia for `base_link` which is physically accurate
- This warning does not affect robot operation or kinematics

**Impact:** None - robot kinematics and dynamics work correctly

**Resolution:** Known URDF/KDL limitation - can be ignored

---

### 6. Controller Manager: No Realtime Kernel Warning

**Error Message:**
```
[WARN] [1761729327.009573516] [controller_manager]: 
No real-time kernel detected on this system. 
See [https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] 
for details on how to enable realtime scheduling.
```

**Explanation:**
- Realtime kernel provides deterministic timing for motor control
- Raspberry Pi 4 runs standard Ubuntu kernel, not RT-PREEMPT
- For this robot's speed and control requirements, standard kernel is sufficient

**Impact:** Minimal - control loop timing may have slight jitter but within acceptable limits

**Resolution:**
- Current setup: Ignore - robot operates within specifications
- Future improvement: Consider RT-PREEMPT kernel for high-speed applications

---

## Topic Configuration

### Camera System Architecture

Rob Box использует **две камеры**:

#### 1. OAK-D (Стерео камера) - Namespace: `/camera`

**Основная камера** для навигации, SLAM и восприятия окружения.

**Публикуемые топики:**
- `/camera/rgb/image_raw` - RGB изображение (720p @ 5 FPS)
- `/camera/rgb/image_raw/compressed` - Сжатое RGB изображение
- `/camera/rgb/camera_info` - Калибровка RGB камеры
- `/camera/stereo/image_raw` - Стерео глубина (640x400 @ 5 FPS)
- `/camera/stereo/image_raw/compressed` - Сжатая стерео глубина
- `/camera/stereo/camera_info` - Калибровка стерео камеры
- `/camera/imu/data` - IMU данные (акселерометр + гироскоп)
- `/camera/nn/spatial_detections` - 3D детекции объектов (если включен NN)

**Конфигурация:** `docker/vision/config/oak-d/oak_d_config.yaml`

**Почему не `/oak`?**
- Пакет `depthai_ros_driver` использует стандартное именование камер ROS 2
- Namespace `camera` обеспечивает совместимость с generic ROS 2 camera tools
- Конфигурируется через параметр `i_tf_base_frame: "camera_link"` в `oak_d_config.yaml`

#### 2. Ceiling Camera (Потолочная камера) - Namespace: `/ceiling_camera`

**Дополнительная камера** для обзора сверху (например, для мониторинга груза).

**Публикуемые топики:**
- `/ceiling_camera/image_raw` - USB камера изображение (1280x720 @ 10 FPS)
- `/ceiling_camera/image_raw/compressed` - Сжатое изображение
- `/ceiling_camera/camera_info` - Калибровка камеры

**Конфигурация:** `docker/vision/config/ceiling-camera/camera_params.yaml`

**Устройство:** `/dev/video0` (USB camera)

### Deployment Verification

Workflow проверяет наличие следующих критически важных топиков:

**OAK-D (обязательные):**
- `/camera/rgb/image_raw/compressed` - RGB изображение для AprilTag и визуализации
- `/camera/stereo/image_raw` - Глубина для RTAB-Map SLAM

**Ceiling Camera (опциональная):**
- `/ceiling_camera/image_raw` - Потолочная камера (если подключена)

**Примечание:** Отсутствие `/ceiling_camera` топиков не является критической ошибкой, если USB камера не подключена.

---

## Summary

The deployment workflow has been updated to automatically filter out these known non-issues:

**Vision Pi Filters:**
- `machine-id` and `vendor id` warnings (cAdvisor)
- `No such container` errors (Promtail cleanup)
- `Serial port /dev/ttyUSB0` warnings (micro-ros-agent optional hardware)
- `entry too far behind` errors (Loki timestamp windows)
- `topology is not available` warnings (cAdvisor)
- `enable watchConfig` info messages (Promtail)

**Main Pi Filters:**
- `CAN controller state: ERROR-ACTIVE` (normal CAN state)
- `out of bounds of the costmap` warnings (Nav2 startup)
- `root link.*inertia` warnings (KDL limitation)
- `No real-time kernel detected` (informational only)

**NOT Filtered (Real Issues That Need Attention):**
- ⚠️ `Error opening OutputStream: Invalid number of channels` - **Audio device misconfiguration**
- ⚠️ `Expression 'parameters->channelCount <= maxChans' failed` - **Audio channel mismatch**

After filtering, only genuinely critical issues will trigger deployment alerts.

---

*Last Updated: October 29, 2025*
*Maintainer: Rob Box Project Team*
