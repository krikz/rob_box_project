# rob_box_animations

ROS2 package for LED Matrix animation playback on rob_box robot.

## 🎬 Features

- **Multi-panel synchronization** - Coordinate animations across 5 LED panels simultaneously
- **YAML-based manifests** - Easy to create and edit animations
- **Frame caching** - Optimized image loading and rendering
- **ROS2 services** - Control playback via standard ROS2 interfaces
- **Priority system** - Handle animation conflicts intelligently
- **Loop support** - Seamless animation looping with configurable transitions

## 📦 Package Contents

```
rob_box_animations/
├── rob_box_animations/          # Python modules
│   ├── animation_loader.py      # YAML manifest loader
│   ├── animation_player.py      # Animation playback engine
│   └── frame_renderer.py        # Image to ROS message converter
├── scripts/                     # ROS2 nodes
│   └── animation_player_node.py # Main animation player node
├── animations/                  # Animation data
│   ├── manifests/              # YAML animation definitions
│   └── frames/                 # PNG image frames
└── launch/                     # Launch files
    └── animation_player.launch.py
```

## 🚀 Quick Start

### Build the package

```bash
cd /path/to/rob_box_project
colcon build --packages-select rob_box_animations
source install/setup.bash
```

### Launch animation player

```bash
# Start with no animation
ros2 launch rob_box_animations animation_player.launch.py

# Start with idle animation
ros2 launch rob_box_animations animation_player.launch.py autostart_animation:=idle
```

### Control via services

```bash
# List available animations
ros2 service call /animation_player/list_animations std_srvs/srv/Trigger

# Load animation
ros2 service call /animation_player/load_animation std_msgs/srv/String "data: 'police_lights'"

# Start playback
ros2 service call /animation_player/play std_srvs/srv/Trigger

# Pause
ros2 service call /animation_player/pause std_srvs/srv/SetBool "data: true"

# Resume
ros2 service call /animation_player/pause std_srvs/srv/SetBool "data: false"

# Stop
ros2 service call /animation_player/stop std_srvs/srv/Trigger
```

### Monitor status

```bash
ros2 topic echo /animation_player/status
```

## 📋 Available Animations

21 animations, names are the LLM tool vocabulary
(`rob_box_mcp_tools/animations.py`): `idle`, `talking`, `thinking`, `happy`,
`sad`, `angry`, `surprised`, `victory`, `sleep`, `wakeup`, `charging`,
`low_battery`, `error`, `turn_left`, `turn_right`, `braking`, `accelerating`,
`police_lights`, `fire_truck`, `ambulance`, `road_service`.
Gallery with previews: [docs/ANIMATIONS.md](docs/ANIMATIONS.md).

## 🎨 Creating or Changing Animations

Frames and manifests under `animations/` are **generated** by the procedural
face in `tools/led_face_lab/` — do not edit the PNGs or YAML by hand. Change
the scene there and re-bake:

```bash
python tools/led_face_lab/bake_manifests.py src/rob_box_animations/animations
python tools/led_face_lab/render_gifs.py src/rob_box_animations/animations docs/assets/animations
```

The manifest format is unchanged (see below); the generator adds
`brightness_budget`, `bake_scale` and `peak_ma_at_255` to `metadata`.

### Test an animation on the robot

```bash
ros2 topic pub --once /voice/animation/request std_msgs/String "data: 'police_lights:10'"
```

## 🔧 ROS2 API

### Services

| Service | Type | Description |
|---------|------|-------------|
| `~/load_animation` | `std_msgs/String` | Load animation by name |
| `~/play` | `std_srvs/Trigger` | Start playback |
| `~/stop` | `std_srvs/Trigger` | Stop playback |
| `~/pause` | `std_srvs/SetBool` | Pause/resume (true/false) |
| `~/list_animations` | `std_srvs/Trigger` | List available animations |

### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `~/status` | `std_msgs/String` | Out | Playback status (1 Hz) |
| `/panel_image` | `sensor_msgs/Image` | Out | Frame images for LED panels |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `animations_dir` | string | pkg share | Path to animations directory |
| `autostart_animation` | string | "" | Animation to start on launch |
| `loop` | bool | true | Loop animations by default |

## 📐 Panel Logical Groups

| Logical Group | Size | Description |
|---------------|------|-------------|
| `headlight_front_left` | 8×8 | Front left headlight (left eye) |
| `headlight_front_right` | 8×8 | Front right headlight (right eye) |
| `headlight_rear_left` | 8×8 | Rear left taillight |
| `headlight_rear_right` | 8×8 | Rear right taillight |
| `headlights_front` | 16×8 | Both front headlights |
| `headlights_rear` | 16×8 | Both rear taillights |
| `headlights_all` | 16×16 | All headlights (2×2 grid) |
| `main_display` | 25×5 | Main display (mouth) |

## 🎯 Animation Priority Levels

When multiple animations are requested:

1. **CRITICAL** - Emergency stops, critical errors (always wins)
2. **HIGH** - Emergency services, safety warnings
3. **NORMAL** - Robot states, missions, navigation
4. **LOW** - Emotions, decorative (can be interrupted)

## 🔍 Troubleshooting

### Animation not found
```
ERROR: Manifest not found
```
Check that YAML file exists in `animations/manifests/` directory.

### Frame image not found
```
ERROR: Frame image not found
```
Verify image paths in manifest are correct and PNG files exist.

### Wrong image size
```
ERROR: Invalid image shape
```
Ensure images match expected panel dimensions (8×8 or 25×5).

### No publishers
```
WARNING: No publisher for logical_group
```
Check that logical group name in manifest matches LED panel configuration.

## 📚 Related Documentation

- [LED Matrix Driver](https://github.com/krikz/ros2leds)
- [Animation List](animations/ANIMATIONS_LIST.md)
- [ros2leds Package](https://github.com/krikz/ros2leds)

## 📝 License

MIT License - Copyright (c) 2025 rob_box

## 🤝 Contributing

Contributions welcome! Please:
1. Create animations following the manifest format
2. Test thoroughly before submitting
3. Document any new features
4. Add animations to ANIMATIONS_LIST.md
