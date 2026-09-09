# Frame Images Placeholder

This directory will contain PNG frame images for animations.

## Structure

Each animation should have its own subdirectory:

```
frames/
├── police/
│   ├── front_left_blue.png
│   ├── front_left_red.png
│   ├── front_left_off.png
│   ├── front_right_blue.png
│   ├── front_right_red.png
│   ├── front_right_off.png
│   ├── rear_blue.png
│   ├── rear_red.png
│   ├── rear_off.png
│   ├── mouth_police_on.png
│   └── mouth_police_off.png
├── road_service/
│   ├── beacon_01.png ... beacon_20.png
│   └── text_01.png ... text_20.png
├── idle/
│   ├── eyes_01.png ... eyes_05.png
│   ├── rear_glow.png
│   ├── mouth_closed.png
│   └── mouth_01.png ... mouth_03.png
├── charging/
│   ├── level_00.png ... level_15.png
│   ├── rear_off.png
│   └── battery_00.png ... battery_15.png
├── emotions/
│   ├── eye_left_happy_01.png
│   ├── eye_left_happy_blink.png
│   ├── eye_right_happy_01.png
│   ├── eye_right_happy_blink.png
│   ├── rear_off.png
│   ├── mouth_smile_01.png
│   ├── mouth_smile_02.png
│   └── mouth_smile_03.png
└── navigation/
    ├── turn_left_on.png
    ├── turn_left_off.png
    ├── front_dim.png
    ├── rear_glow.png
    ├── arrow_left_01.png
    ├── arrow_left_02.png
    ├── arrow_left_03.png
    └── arrow_left_04.png
```

## Image Requirements

### Headlights (Eyes/Taillights)
- **Size:** 8×8 pixels
- **Format:** PNG, RGB (no alpha)
- **Color depth:** 8 bits per channel

### Main Display (Mouth)
- **Size:** 25×5 pixels (width × height)
- **Format:** PNG, RGB (no alpha)
- **Color depth:** 8 bits per channel

## Creating Frames

Use any image editor (GIMP, Photoshop, etc.) to create frames:

1. Create new image with exact dimensions (8×8 or 25×5)
2. Draw your animation frame
3. Export as PNG (flatten to RGB, no transparency)
4. Save in appropriate subdirectory

## Tools

Consider using these tools to generate frames:
- **GIMP** - Manual pixel art
- **Aseprite** - Pixel art animation
- **Python PIL/Pillow** - Programmatic generation
- **ImageMagick** - Batch processing

## Example: Generate solid color frame

```python
from PIL import Image
import numpy as np

# Create 8×8 red image
img = np.zeros((8, 8, 3), dtype=np.uint8)
img[:, :] = [255, 0, 0]  # Red

Image.fromarray(img).save('frames/test/red_8x8.png')

# Create 25×5 blue image
img = np.zeros((5, 25, 3), dtype=np.uint8)
img[:, :] = [0, 0, 255]  # Blue

Image.fromarray(img).save('frames/test/blue_25x5.png')
```

## TODO

- [ ] Create frames for police_lights animation
- [ ] Create frames for road_service animation
- [ ] Create frames for idle animation
- [ ] Create frames for charging animation
- [ ] Create frames for happy emotion
- [ ] Create frames for turn_left navigation
- [ ] Add frame generation scripts
- [ ] Document color palette guidelines
