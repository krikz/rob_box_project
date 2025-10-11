# Talking Animation V2.0 - Advanced Realistic Version

## Overview
Improved talking animation featuring realistic gradient eyes and advanced sine/sawtooth waveform mouth with visible teeth. Uses the full 25×5 mouth display matrix for smooth, natural-looking speech animation.

## Key Features

### 1. Realistic Gradient Eyes (8×8)
- **Circular gradient pattern** based on `Rover_PICS/DISARM_0.png` reference
- **Cyan/blue color theme** with smooth transitions
- **5 gradient levels**:
  - Center (brightest): RGB(150, 230, 255)
  - Inner ring: RGB(100, 200, 255)
  - Mid ring: RGB(50, 150, 200)
  - Outer ring: RGB(0, 100, 150)
  - Edge (darkest): RGB(0, 50, 80)

### 2. Advanced Waveform Mouth (25×5)
- **Full matrix utilization**: All 25×5 pixels (125 LEDs)
- **Sine/sawtooth waveforms** for natural curves
- **Visible teeth**: Vertical white stripes at regular intervals
- **4 animation phases**:
  1. Opening (frames 1-3): Sawtooth wave - sharp opening
  2. Peak (frames 4-6): Sine wave - smooth wide open
  3. Closing (frames 7-9): Sawtooth wave - sharp closing
  4. Rest (frames 10-12): Low sine wave - subtle movement

### 3. Color Scheme
- **Background**: RGB(20, 20, 20) - dark
- **Mouth interior**: RGB(200, 50, 50) - red
- **Teeth**: RGB(240, 240, 240) - white
- **Lips/edges**: RGB(100, 100, 100) - gray

## Technical Details

### Frame Count
- **12 frames** (increased from 8)
- **12 FPS** for smooth animation
- **Total duration**: 1200ms per loop

### Audio-Reactive Ranges
Each frame maps to specific audio volume ranges for natural lip-sync:

| Frame | Phase | Audio Range | Description |
|-------|-------|-------------|-------------|
| 1 | Opening | 0.00-0.08 | Closed |
| 2 | Opening | 0.08-0.17 | Slightly open |
| 3 | Opening | 0.17-0.25 | Opening wider |
| 4 | Peak | 0.25-0.40 | Half open |
| 5 | Peak | 0.40-0.55 | Nearly wide |
| 6 | Peak | 0.55-0.70 | Fully open |
| 7 | Closing | 0.70-0.78 | Starting close |
| 8 | Closing | 0.78-0.87 | Closing more |
| 9 | Closing | 0.87-0.92 | Nearly closed |
| 10 | Rest | 0.92-0.95 | Subtle movement |
| 11 | Rest | 0.95-0.98 | Very subtle |
| 12 | Rest | 0.98-1.00 | Minimal |

## Files Generated

### Eye Frames
- `eye_fl_normal.png` - Front left gradient eye (8×8)
- `eye_fr_normal.png` - Front right gradient eye (8×8)

### Mouth Frames (25×5)
- `mouth_talk_01.png` through `mouth_talk_12.png`

### Rear Wheels
- `wheel_rl_talk_off.png` - Rear left off
- `wheel_rr_talk_off.png` - Rear right off

## Implementation

### Generation Method
Located in `scripts/generate_animation_frames.py`:

```python
def _create_realistic_eye(size, state):
    """Creates circular gradient eye with 5 color levels"""
    # Distance-based gradient calculation
    
def _create_advanced_mouth(size, openness, wave_type):
    """Creates waveform mouth with teeth"""
    # Sine/sawtooth wave generation
    # Teeth overlay at regular intervals
```

### Usage

#### Visualize
```bash
python3 scripts/visualize_animations.py
# Select "talking" animation
```

#### Regenerate Frames
```bash
python3 scripts/generate_animation_frames.py --animation talking
```

#### Use in ROS2
```bash
ros2 topic pub /led_matrix/animation_select std_msgs/String "data: 'talking'"
```

## Comparison: V1 vs V2

| Feature | V1.0 (Bender-style) | V2.0 (Realistic) |
|---------|---------------------|------------------|
| Eye style | Solid cyan | Gradient circular |
| Eye colors | 1 | 5 gradient levels |
| Mouth area | ~15×3 center | Full 25×5 |
| Mouth shape | Rectangular | Sine/sawtooth curves |
| Teeth | Simple horizontal | Vertical stripes |
| Frames | 8 | 12 |
| FPS | 10 | 12 |
| Waveforms | None | Sine + Sawtooth |

## Reference Images
Based on user's original robot designs in `Rover_PICS/`:
- `DISARM_0.png` - Gradient circular eye pattern
- `INIT_0.png` - Cross/plus pattern (alternative eye state)

## Future Enhancements
- [ ] Add blinking eye states
- [ ] Eye movement (look left/right)
- [ ] Different mouth colors for emotions
- [ ] Tongue movement (center stripe)
- [ ] Surprise state (wide eyes + open mouth)

## Credits
- **Design reference**: User's original Rover_PICS/ images
- **Implementation**: rob_box animation system v2.0
- **Audio system**: PyAudio-based reactive controller

---

**Version**: 2.0  
**Date**: 2025-10-11  
**Status**: ✅ Complete and tested
