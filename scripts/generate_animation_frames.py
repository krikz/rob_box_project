#!/usr/bin/env python3
"""
Frame Generator for rob_box LED Animations

Generates PNG frame images for all animations programmatically.
"""

import os
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import argparse
from pathlib import Path


class FrameGenerator:
    """Generate LED matrix animation frames"""
    
    # Panel sizes
    HEADLIGHT_SIZE = (8, 8)  # Width × Height
    DISPLAY_SIZE = (25, 5)   # Width × Height
    
    # Colors
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    YELLOW = (255, 255, 0)
    ORANGE = (255, 165, 0)
    CYAN = (0, 255, 255)
    MAGENTA = (255, 0, 255)
    
    def __init__(self, output_dir: str):
        """
        Initialize generator
        
        Args:
            output_dir: Root directory for frames
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
    
    def create_solid_frame(self, size: tuple, color: tuple, name: str, subdir: str):
        """Create solid color frame"""
        width, height = size
        img = np.zeros((height, width, 3), dtype=np.uint8)
        img[:, :] = color
        
        self._save_frame(img, name, subdir)
    
    def create_gradient_frame(self, size: tuple, color1: tuple, color2: tuple, 
                            name: str, subdir: str, vertical: bool = True):
        """Create gradient frame"""
        width, height = size
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        if vertical:
            for y in range(height):
                ratio = y / height
                color = tuple(int(c1 * (1 - ratio) + c2 * ratio) 
                            for c1, c2 in zip(color1, color2))
                img[y, :] = color
        else:
            for x in range(width):
                ratio = x / width
                color = tuple(int(c1 * (1 - ratio) + c2 * ratio) 
                            for c1, c2 in zip(color1, color2))
                img[:, x] = color
        
        self._save_frame(img, name, subdir)
    
    def create_pattern_frame(self, size: tuple, pattern: np.ndarray, 
                           name: str, subdir: str):
        """Create frame from pattern array"""
        self._save_frame(pattern, name, subdir)
    
    def _save_frame(self, array: np.ndarray, name: str, subdir: str):
        """Save frame to disk"""
        output_path = self.output_dir / subdir
        output_path.mkdir(parents=True, exist_ok=True)
        
        filepath = output_path / name
        img = Image.fromarray(array.astype(np.uint8))
        img.save(filepath)
        print(f"Created: {filepath}")
    
    # === ANIMATION GENERATORS ===
    
    def generate_police_lights(self):
        """Generate police emergency lights frames"""
        print("\n=== Generating police_lights ===")
        subdir = "police"
        
        # Front left wheel - blue/red alternating
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLUE, 
                              "wheel_fl_blue.png", subdir)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.RED, 
                              "wheel_fl_red.png", subdir)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK, 
                              "wheel_fl_off.png", subdir)
        
        # Front right wheel - blue/red alternating (opposite phase)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLUE, 
                              "wheel_fr_blue.png", subdir)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.RED, 
                              "wheel_fr_red.png", subdir)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK, 
                              "wheel_fr_off.png", subdir)
        
        # Rear left wheel
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLUE, 
                              "wheel_rl_blue.png", subdir)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.RED, 
                              "wheel_rl_red.png", subdir)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK, 
                              "wheel_rl_off.png", subdir)
        
        # Rear right wheel
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLUE, 
                              "wheel_rr_blue.png", subdir)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.RED, 
                              "wheel_rr_red.png", subdir)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK, 
                              "wheel_rr_off.png", subdir)
        
        # Main display - POLICE text (simplified)
        self._create_text_frame("POLICE", self.WHITE, "mouth_police_on.png", subdir)
        self.create_solid_frame(self.DISPLAY_SIZE, self.BLACK, 
                              "mouth_police_off.png", subdir)
    
    def generate_road_service(self):
        """Generate road service beacon frames"""
        print("\n=== Generating road_service ===")
        subdir = "road_service"
        
        # Rotating beacon effect (20 frames)
        for i in range(20):
            angle = i * 18  # 360/20 = 18 degrees per frame
            pattern = self._create_rotating_beacon(angle, self.YELLOW)
            self.create_pattern_frame((16, 8), pattern, 
                                    f"beacon_{i+1:02d}.png", subdir)
        
        # Scrolling text "ROAD SERVICE" (20 frames)
        text = "ROAD SERVICE"
        for i in range(20):
            offset = -i
            pattern = self._create_scrolling_text(text, offset, self.YELLOW)
            self.create_pattern_frame(self.DISPLAY_SIZE, pattern,
                                    f"text_{i+1:02d}.png", subdir)
    
    def generate_idle(self):
        """Generate idle breathing animation"""
        print("\n=== Generating idle ===")
        subdir = "idle"
        
        # Breathing effect - 5 levels
        for i in range(5):
            brightness = int(50 + (i * 40))  # 50 to 210
            color = (brightness, brightness, brightness)
            self.create_solid_frame((16, 8), color, 
                                  f"eyes_{i+1:02d}.png", subdir)
        
        # Rear glow
        self.create_solid_frame((16, 8), (50, 0, 0), 
                              "rear_glow.png", subdir)
        
        # Mouth states
        self.create_solid_frame(self.DISPLAY_SIZE, self.BLACK, 
                              "mouth_closed.png", subdir)
        
        for i in range(3):
            height_open = i + 1
            pattern = self._create_mouth(height_open, self.WHITE)
            self.create_pattern_frame(self.DISPLAY_SIZE, pattern,
                                    f"mouth_{i+1:02d}.png", subdir)
    
    def generate_charging(self):
        """Generate charging animation frames"""
        print("\n=== Generating charging ===")
        subdir = "charging"
        
        # Level indicators (16 levels for 16x8)
        for i in range(16):
            pattern = self._create_level_indicator(i, 16, self.GREEN)
            self.create_pattern_frame((16, 8), pattern,
                                    f"level_{i:02d}.png", subdir)
        
        # Rear off
        self.create_solid_frame((16, 8), self.BLACK, 
                              "rear_off.png", subdir)
        
        # Battery animation (16 frames)
        for i in range(16):
            pattern = self._create_battery_indicator(self.DISPLAY_SIZE, i, 16, critical=False)
            self.create_pattern_frame(self.DISPLAY_SIZE, pattern,
                                    f"battery_{i:02d}.png", subdir)
    
    def generate_happy(self):
        """Generate happy emotion frames"""
        print("\n=== Generating happy ===")
        subdir = "emotions"
        
        # Happy eyes
        left_eye = self._create_happy_eye(False)
        right_eye = self._create_happy_eye(True)
        
        self.create_pattern_frame(self.HEADLIGHT_SIZE, left_eye,
                                "eye_left_happy_01.png", subdir)
        self.create_pattern_frame(self.HEADLIGHT_SIZE, right_eye,
                                "eye_right_happy_01.png", subdir)
        
        # Blink (closed eyes)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK,
                              "eye_left_happy_blink.png", subdir)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK,
                              "eye_right_happy_blink.png", subdir)
        
        # Rear off
        self.create_solid_frame((16, 8), self.BLACK,
                              "rear_off.png", subdir)
        
        # Smile (3 levels)
        for i in range(3):
            pattern = self._create_smile(i + 1)
            self.create_pattern_frame(self.DISPLAY_SIZE, pattern,
                                    f"mouth_smile_{i+1:02d}.png", subdir)
    
    def generate_turn_left(self):
        """Generate turn left navigation frames"""
        print("\n=== Generating turn_left ===")
        subdir = "navigation"
        
        # Turn signal orange
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.ORANGE,
                              "turn_left_on.png", subdir)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK,
                              "turn_left_off.png", subdir)
        
        # Dim white front
        self.create_solid_frame(self.HEADLIGHT_SIZE, (100, 100, 100),
                              "front_dim.png", subdir)
        
        # Rear glow
        self.create_solid_frame(self.HEADLIGHT_SIZE, (100, 0, 0),
                              "rear_glow.png", subdir)
        
        # Left arrow animation (4 frames)
        for i in range(4):
            pattern = self._create_arrow_left(i)
            self.create_pattern_frame(self.DISPLAY_SIZE, pattern,
                                    f"arrow_left_{i+1:02d}.png", subdir)
    
    # === PATTERN HELPERS ===
    
    def _create_text_frame(self, text: str, color: tuple, name: str, subdir: str):
        """Create simple text frame (pixel art)"""
        width, height = self.DISPLAY_SIZE
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Simple 3x5 font for LED matrix
        char_map = {
            'P': [[1,1,1],[1,0,1],[1,1,1],[1,0,0],[1,0,0]],
            'O': [[1,1,1],[1,0,1],[1,0,1],[1,0,1],[1,1,1]],
            'L': [[1,0,0],[1,0,0],[1,0,0],[1,0,0],[1,1,1]],
            'I': [[1,1,1],[0,1,0],[0,1,0],[0,1,0],[1,1,1]],
            'C': [[1,1,1],[1,0,0],[1,0,0],[1,0,0],[1,1,1]],
            'E': [[1,1,1],[1,0,0],[1,1,1],[1,0,0],[1,1,1]],
        }
        
        x_offset = 1
        for char in text:
            if char == ' ':
                x_offset += 2
                continue
            if char in char_map:
                char_pattern = char_map[char]
                for y, row in enumerate(char_pattern):
                    for x, pixel in enumerate(row):
                        if pixel and x_offset + x < width:
                            img[y, x_offset + x] = color
                x_offset += 4
        
        self._save_frame(img, name, subdir)
    
    def _create_rotating_beacon(self, angle: int, color: tuple) -> np.ndarray:
        """Create rotating beacon pattern"""
        width, height = 16, 8
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Simple rotating bar
        center_x, center_y = width // 2, height // 2
        length = 6
        
        angle_rad = np.radians(angle)
        for r in range(length):
            x = int(center_x + r * np.cos(angle_rad))
            y = int(center_y + r * np.sin(angle_rad))
            if 0 <= x < width and 0 <= y < height:
                img[y, x] = color
        
        return img
    
    def _create_scrolling_text(self, text: str, offset: int, color: tuple) -> np.ndarray:
        """Create scrolling text pattern"""
        width, height = self.DISPLAY_SIZE
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Draw text at offset (simplified)
        for i, char in enumerate(text):
            x_pos = offset + (i * 2)
            if 0 <= x_pos < width:
                img[2, x_pos] = color
        
        return img
    
    def _create_mouth(self, height_open: int, color: tuple) -> np.ndarray:
        """Create mouth opening pattern"""
        width, height = self.DISPLAY_SIZE
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Simple horizontal line getting taller
        center_y = height // 2
        for y in range(max(0, center_y - height_open), 
                      min(height, center_y + height_open + 1)):
            img[y, 5:20] = color
        
        return img
    
    def _create_level_indicator(self, level: int, max_level: int, 
                               color: tuple) -> np.ndarray:
        """Create level indicator (filling bar)"""
        width, height = 16, 8
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        filled_width = int((level / max_level) * width)
        img[:, :filled_width] = color
        
        return img
    
    def _create_battery_indicator(self, level: int, max_level: int, 
                                 color: tuple) -> np.ndarray:
        """Create battery indicator"""
        width, height = self.DISPLAY_SIZE
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Battery outline
        img[1:4, 2:23] = (100, 100, 100)  # Gray outline
        img[2, 3:22] = self.BLACK  # Clear inside
        
        # Filling level
        filled_width = int((level / max_level) * 18)
        if filled_width > 0:
            img[2, 3:3+filled_width] = color
        
        return img
    
    def _create_happy_eye(self, mirror: bool) -> np.ndarray:
        """Create happy eye pattern"""
        size = self.HEADLIGHT_SIZE[0]
        img = np.zeros((size, size, 3), dtype=np.uint8)
        
        # Simple bright circle
        center = size // 2
        radius = 3
        
        for y in range(size):
            for x in range(size):
                dist = np.sqrt((x - center)**2 + (y - center)**2)
                if dist <= radius:
                    brightness = int(255 * (1 - dist / radius))
                    img[y, x] = (brightness, brightness, brightness)
        
        return img
    
    def _create_smile(self, intensity: int) -> np.ndarray:
        """Create smile pattern"""
        width, height = self.DISPLAY_SIZE
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Simple curved smile
        y = 3 if intensity == 1 else (4 if intensity == 2 else 4)
        img[y, 5:20] = self.WHITE
        
        if intensity > 1:
            img[y-1, 7:18] = self.WHITE
        if intensity > 2:
            img[y-2, 10:15] = self.WHITE
        
        return img
    
    def _create_arrow_left(self, frame: int) -> np.ndarray:
        """Create left arrow animation"""
        width, height = self.DISPLAY_SIZE
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Arrow pointing left, moving
        offset = frame * 3
        arrow_x = 15 - offset
        
        # Arrow shape
        for i in range(5):
            x = arrow_x + i
            if 0 <= x < width:
                img[2, x] = self.ORANGE
        
        # Arrow head
        if 0 <= arrow_x < width:
            img[1, arrow_x] = self.ORANGE
            img[3, arrow_x] = self.ORANGE
        
        return img
    
    def _create_arrow_right(self, frame: int) -> np.ndarray:
        """Create right arrow animation"""
        width, height = self.DISPLAY_SIZE
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Arrow pointing right, moving
        offset = frame * 3
        arrow_x = 5 + offset
        
        # Arrow shape
        for i in range(5):
            x = arrow_x - i
            if 0 <= x < width:
                img[2, x] = self.ORANGE
        
        # Arrow head (pointing right)
        if arrow_x < width:
            img[1, arrow_x] = self.ORANGE
            img[3, arrow_x] = self.ORANGE
        
        return img
    
    def generate_ambulance(self):
        """Generate ambulance emergency lights frames"""
        print("\n=== Generating ambulance ===")
        subdir = "ambulance"
        
        # All wheels flash red
        for suffix in ['fl', 'fr', 'rl', 'rr']:
            self.create_solid_frame(self.HEADLIGHT_SIZE, self.RED,
                                  f"wheel_{suffix}_red.png", subdir)
            self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK,
                                  f"wheel_{suffix}_off.png", subdir)
        
        # Display: "AMBULANCE" text frames
        for i, text in enumerate(["AMBUL", "ULANCE", "AMBU", "LANCE"]):
            img = self._create_scrolling_text(text, i * 6, self.RED)
            self._save_frame(img, f"text_{i+1:02d}.png", subdir)
    
    def generate_fire_truck(self):
        """Generate fire truck emergency lights frames"""
        print("\n=== Generating fire_truck ===")
        subdir = "fire_truck"
        
        # All wheels flash red/yellow
        for suffix in ['fl', 'fr', 'rl', 'rr']:
            self.create_solid_frame(self.HEADLIGHT_SIZE, self.RED,
                                  f"wheel_{suffix}_red.png", subdir)
            self.create_solid_frame(self.HEADLIGHT_SIZE, self.ORANGE,
                                  f"wheel_{suffix}_orange.png", subdir)
            self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK,
                                  f"wheel_{suffix}_off.png", subdir)
        
        # Display: "FIRE" text
        img = self._create_scrolling_text("FIRE!", 8, self.ORANGE)
        self._save_frame(img, "text_fire_on.png", subdir)
        self.create_solid_frame(self.DISPLAY_SIZE, self.BLACK, "text_fire_off.png", subdir)
    
    def generate_turn_right(self):
        """Generate turn right signal frames"""
        print("\n=== Generating turn_right ===")
        subdir = "navigation"
        
        # Right wheels flash orange
        for suffix in ['fr', 'rr']:
            self.create_solid_frame(self.HEADLIGHT_SIZE, self.ORANGE,
                                  f"wheel_{suffix}_orange.png", subdir)
            self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK,
                                  f"wheel_{suffix}_off.png", subdir)
        
        # Left wheels off
        for suffix in ['fl', 'rl']:
            self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK,
                                  f"wheel_{suffix}_off.png", subdir)
        
        # Right arrow animation (4 frames)
        for i in range(4):
            img = self._create_arrow_right(i)
            self._save_frame(img, f"arrow_right_{i+1:02d}.png", subdir)
    
    def generate_braking(self):
        """Generate braking animation frames"""
        print("\n=== Generating braking ===")
        subdir = "navigation"
        
        # Rear wheels bright red
        for suffix in ['rl', 'rr']:
            self.create_solid_frame(self.HEADLIGHT_SIZE, self.RED,
                                  f"brake_{suffix}_on.png", subdir)
        
        # Front wheels off
        for suffix in ['fl', 'fr']:
            self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK,
                                  f"brake_{suffix}_off.png", subdir)
        
        # Display: "STOP" sign (3 frames - pulsing)
        for i, intensity in enumerate([255, 200, 150]):
            color = (intensity, 0, 0)
            img = self._create_scrolling_text("STOP", 9, color)
            self._save_frame(img, f"stop_sign_{i+1:02d}.png", subdir)
    
    def generate_accelerating(self):
        """Generate acceleration animation frames"""
        print("\n=== Generating accelerating ===")
        subdir = "navigation"
        
        # Front wheels bright white (headlights)
        for suffix in ['fl', 'fr']:
            self.create_solid_frame(self.HEADLIGHT_SIZE, self.WHITE,
                                  f"accel_{suffix}_on.png", subdir)
        
        # Rear wheels dim
        for suffix in ['rl', 'rr']:
            dim_color = (30, 30, 30)
            self.create_solid_frame(self.HEADLIGHT_SIZE, dim_color,
                                  f"accel_{suffix}_dim.png", subdir)
        
        # Speed lines animation (6 frames)
        for i in range(6):
            img = self._create_speed_lines(self.DISPLAY_SIZE, i)
            self._save_frame(img, f"speed_{i+1:02d}.png", subdir)
    
    def generate_error(self):
        """Generate error/warning animation frames"""
        print("\n=== Generating error ===")
        subdir = "system"
        
        # All wheels flash red (error state)
        for suffix in ['fl', 'fr', 'rl', 'rr']:
            self.create_solid_frame(self.HEADLIGHT_SIZE, self.RED,
                                  f"wheel_{suffix}_error.png", subdir)
            self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK,
                                  f"wheel_{suffix}_off.png", subdir)
        
        # Display: "ERROR" and "X" pattern
        img = self._create_scrolling_text("ERROR", 8, self.RED)
        self._save_frame(img, "error_text.png", subdir)
        
        img = self._create_x_pattern(self.DISPLAY_SIZE, self.RED)
        self._save_frame(img, "error_x.png", subdir)
        
        self.create_solid_frame(self.DISPLAY_SIZE, self.BLACK, "error_off.png", subdir)
    
    def generate_sad(self):
        """Generate sad emotion animation frames"""
        print("\n=== Generating sad ===")
        subdir = "emotions"
        
        # Dim blue wheels (sad mood)
        sad_blue = (0, 50, 100)
        for suffix in ['fl', 'fr']:
            self.create_solid_frame(self.HEADLIGHT_SIZE, sad_blue,
                                  f"eye_{suffix}_sad.png", subdir)
        
        for suffix in ['rl', 'rr']:
            self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK,
                                  f"wheel_{suffix}_off.png", subdir)
        
        # Sad face (3 frames - frown animation)
        for i in range(3):
            img = self._create_sad_face(self.DISPLAY_SIZE, i)
            self._save_frame(img, f"mouth_sad_{i+1:02d}.png", subdir)
    
    def generate_angry(self):
        """Generate angry emotion animation frames"""
        print("\n=== Generating angry ===")
        subdir = "emotions"
        
        # Bright red pulsing wheels
        for i, intensity in enumerate([255, 200, 150]):
            color = (intensity, 0, 0)
            for suffix in ['fl', 'fr']:
                img = np.full((self.HEADLIGHT_SIZE[1], self.HEADLIGHT_SIZE[0], 3), 
                            color, dtype=np.uint8)
                self._save_frame(img, f"eye_{suffix}_angry_{i+1:02d}.png", subdir)
        
        # Angry eyebrows and mouth (3 frames)
        for i in range(3):
            img = self._create_angry_face(self.DISPLAY_SIZE, i)
            self._save_frame(img, f"mouth_angry_{i+1:02d}.png", subdir)
    
    def generate_surprised(self):
        """Generate surprised emotion animation frames"""
        print("\n=== Generating surprised ===")
        subdir = "emotions"
        
        # Wide eyes (bright white)
        for suffix in ['fl', 'fr']:
            # Large circle pattern for wide eyes
            img = self._create_eye_pattern(self.HEADLIGHT_SIZE, 'wide')
            self._save_frame(img, f"eye_{suffix}_surprised.png", subdir)
        
        # "O" shaped mouth (3 sizes)
        for i, size in enumerate([1, 2, 3]):
            img = self._create_o_mouth(self.DISPLAY_SIZE, size)
            self._save_frame(img, f"mouth_surprised_{i+1:02d}.png", subdir)
    
    def generate_sleep(self):
        """Generate sleeping animation frames"""
        print("\n=== Generating sleep ===")
        subdir = "system"
        
        # Closed eyes (horizontal lines)
        for suffix in ['fl', 'fr']:
            img = self._create_eye_pattern(self.HEADLIGHT_SIZE, 'closed')
            self._save_frame(img, f"eye_{suffix}_closed.png", subdir)
        
        # Rear wheels off
        for suffix in ['rl', 'rr']:
            self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK,
                                  f"wheel_{suffix}_off.png", subdir)
        
        # "ZZZ" animation (5 frames)
        for i in range(5):
            img = self._create_zzz_pattern(self.DISPLAY_SIZE, i)
            self._save_frame(img, f"zzz_{i+1:02d}.png", subdir)
    
    def generate_wakeup(self):
        """Generate wake up animation frames"""
        print("\n=== Generating wakeup ===")
        subdir = "system"
        
        # Eyes opening sequence (6 frames)
        for i in range(6):
            for suffix in ['fl', 'fr']:
                img = self._create_eye_opening(self.HEADLIGHT_SIZE, i)
                self._save_frame(img, f"eye_{suffix}_open_{i+1:02d}.png", subdir)
        
        # Yawn animation (4 frames)
        for i in range(4):
            img = self._create_yawn(self.DISPLAY_SIZE, i)
            self._save_frame(img, f"yawn_{i+1:02d}.png", subdir)
    
    def generate_low_battery(self):
        """Generate low battery warning frames"""
        print("\n=== Generating low_battery ===")
        subdir = "system"
        
        # Wheels flash red slowly
        for suffix in ['fl', 'fr', 'rl', 'rr']:
            dim_red = (100, 0, 0)
            self.create_solid_frame(self.HEADLIGHT_SIZE, dim_red,
                                  f"wheel_{suffix}_low.png", subdir)
            self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK,
                                  f"wheel_{suffix}_off.png", subdir)
        
        # Battery indicator (empty to critical) - 6 frames
        for i in range(6):
            img = self._create_battery_indicator(self.DISPLAY_SIZE, i, 6, critical=True)
            self._save_frame(img, f"battery_low_{i+1:02d}.png", subdir)
    
    def generate_thinking(self):
        """Generate thinking animation frames"""
        print("\n=== Generating thinking ===")
        subdir = "emotions"
        
        # Eyes look around (8 frames)
        positions = ['center', 'right', 'center', 'left', 'center', 'up', 'center', 'down']
        for i, pos in enumerate(positions):
            for suffix in ['fl', 'fr']:
                img = self._create_eye_looking(self.HEADLIGHT_SIZE, pos)
                self._save_frame(img, f"eye_{suffix}_think_{i+1:02d}.png", subdir)
        
        # Thinking dots animation (4 frames)
        for i in range(4):
            img = self._create_thinking_dots(self.DISPLAY_SIZE, i)
            self._save_frame(img, f"dots_{i+1:02d}.png", subdir)
    
    def generate_victory(self):
        """Generate victory/success animation frames"""
        print("\n=== Generating victory ===")
        subdir = "emotions"
        
        # Rainbow cycling on wheels (8 frames)
        colors = [self.RED, self.ORANGE, self.YELLOW, self.GREEN, 
                 self.CYAN, self.BLUE, self.MAGENTA, (255, 255, 255)]
        for i, color in enumerate(colors):
            for suffix in ['fl', 'fr', 'rl', 'rr']:
                self.create_solid_frame(self.HEADLIGHT_SIZE, color,
                                      f"wheel_{suffix}_rainbow_{i+1:02d}.png", subdir)
        
        # Checkmark animation (3 frames)
        for i in range(3):
            img = self._create_checkmark(self.DISPLAY_SIZE, i)
            self._save_frame(img, f"checkmark_{i+1:02d}.png", subdir)
    
    # Helper methods for new patterns
    
    def _create_speed_lines(self, size: tuple, frame: int):
        """Create speed lines moving effect"""
        width, height = size
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Moving horizontal lines
        offset = frame * 4
        for y in [1, 3]:
            for x in range(width):
                if (x + offset) % 8 < 4:
                    img[y, x] = self.CYAN
        
        return img
    
    def _create_x_pattern(self, size: tuple, color: tuple):
        """Create X pattern"""
        width, height = size
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Diagonal lines
        for i in range(min(width, height)):
            if i < width and i < height:
                img[i, i] = color
            if i < width and height - 1 - i >= 0:
                img[height - 1 - i, i] = color
        
        return img
    
    def _create_sad_face(self, size: tuple, frame: int):
        """Create sad face (frown)"""
        width, height = size
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Sad eyes (small dots)
        if frame % 3 != 2:  # Blink occasionally
            img[1, 8] = self.BLUE
            img[1, 16] = self.BLUE
        
        # Frown (inverted arc)
        y = 3
        for x in range(7, 18):
            if (x - 12) ** 2 / 20 < (3.5 - y) ** 2:
                img[y, x] = self.BLUE
        
        return img
    
    def _create_angry_face(self, size: tuple, frame: int):
        """Create angry face with furrowed brows"""
        width, height = size
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Angry eyebrows
        for x in range(6, 11):
            img[1, x] = self.RED
        for x in range(14, 19):
            img[1, x] = self.RED
        
        # Straight line mouth (angry)
        for x in range(8, 17):
            img[3, x] = self.RED
        
        # Intensity pulsing
        if frame > 0:
            img = (img * (0.7 + 0.3 * frame / 2)).astype(np.uint8)
        
        return img
    
    def _create_o_mouth(self, size: tuple, mouth_size: int):
        """Create O-shaped mouth"""
        width, height = size
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Circle in center
        cx, cy = width // 2, height // 2
        for y in range(height):
            for x in range(width):
                dist = ((x - cx) ** 2 + (y - cy) ** 2) ** 0.5
                if mouth_size <= dist < mouth_size + 1:
                    img[y, x] = self.YELLOW
        
        return img
    
    def _create_eye_pattern(self, size: tuple, pattern_type: str):
        """Create different eye patterns"""
        width, height = size
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        if pattern_type == 'wide':
            # Large circle for surprised eyes
            for y in range(2, 6):
                for x in range(2, 6):
                    img[y, x] = self.WHITE
        elif pattern_type == 'closed':
            # Horizontal line for closed eyes
            for x in range(2, 6):
                img[4, x] = (100, 100, 100)
        
        return img
    
    def _create_zzz_pattern(self, size: tuple, frame: int):
        """Create ZZZ sleep pattern"""
        width, height = size
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Animated Z's moving up
        offset = frame
        positions = [(10, 3 - offset % 4), (15, 2 - offset % 4), (20, 1 - offset % 4)]
        
        for x, y in positions:
            if 0 <= y < height and x < width:
                # Draw Z
                for dx in range(3):
                    if x + dx < width:
                        img[y, x + dx] = (150, 150, 255)
        
        return img
    
    def _create_eye_opening(self, size: tuple, frame: int):
        """Create eye opening animation"""
        width, height = size
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Progressive opening
        open_height = min(frame + 1, height // 2)
        center_y = height // 2
        
        for y in range(center_y - open_height, center_y + open_height):
            if 0 <= y < height:
                for x in range(2, 6):
                    img[y, x] = self.WHITE
        
        return img
    
    def _create_yawn(self, size: tuple, frame: int):
        """Create yawn animation"""
        width, height = size
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Growing oval
        mouth_width = 5 + frame * 2
        mouth_height = 2 + frame
        
        cx, cy = width // 2, height // 2
        for y in range(height):
            for x in range(width):
                if ((x - cx) ** 2) / (mouth_width ** 2) + ((y - cy) ** 2) / (mouth_height ** 2) < 1:
                    img[y, x] = (255, 200, 0)
        
        return img
    
    def _create_battery_indicator(self, size: tuple, level: int, max_level: int, critical: bool = False):
        """Create battery level indicator"""
        width, height = size
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Battery outline
        for x in range(5, 20):
            img[1, x] = (100, 100, 100)
            img[3, x] = (100, 100, 100)
        for y in range(1, 4):
            img[y, 5] = (100, 100, 100)
            img[y, 19] = (100, 100, 100)
        
        # Battery tip
        img[2, 20] = (100, 100, 100)
        
        # Fill level
        fill_width = int((level / max_level) * 13)
        color = self.RED if critical and level < 3 else self.GREEN
        for x in range(6, 6 + fill_width):
            img[2, x] = color
        
        return img
    
    def _create_eye_looking(self, size: tuple, direction: str):
        """Create eye looking in direction"""
        width, height = size
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Pupil position
        offsets = {
            'center': (0, 0),
            'left': (-1, 0),
            'right': (1, 0),
            'up': (0, -1),
            'down': (0, 1)
        }
        
        cx, cy = width // 2, height // 2
        dx, dy = offsets.get(direction, (0, 0))
        
        # Eye white
        for y in range(2, 6):
            for x in range(2, 6):
                img[y, x] = (200, 200, 200)
        
        # Pupil
        px, py = cx + dx, cy + dy
        if 0 <= py < height and 0 <= px < width:
            img[py, px] = self.BLUE
            if py + 1 < height:
                img[py + 1, px] = self.BLUE
        
        return img
    
    def _create_thinking_dots(self, size: tuple, frame: int):
        """Create thinking dots animation"""
        width, height = size
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Three dots appearing sequentially
        positions = [(10, 2), (15, 2), (20, 2)]
        for i, (x, y) in enumerate(positions):
            if frame >= i:
                img[y, x] = self.YELLOW
                img[y, x + 1] = self.YELLOW
        
        return img
    
    def _create_checkmark(self, size: tuple, frame: int):
        """Create checkmark pattern"""
        width, height = size
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Progressive checkmark drawing
        if frame >= 0:
            # Short stroke
            img[3, 10] = self.GREEN
            img[4, 11] = self.GREEN
        if frame >= 1:
            # Longer stroke
            img[2, 12] = self.GREEN
            img[1, 13] = self.GREEN
        if frame >= 2:
            # Complete
            img[1, 14] = self.GREEN
            img[2, 15] = self.GREEN
        
        return img
    
    def generate_all(self):
        """Generate all animation frames"""
        print("Generating all LED matrix animation frames...")
        
        # Original animations
        self.generate_police_lights()
        self.generate_road_service()
        self.generate_idle()
        self.generate_charging()
        self.generate_happy()
        self.generate_turn_left()
        
        # New animations
        self.generate_ambulance()
        self.generate_fire_truck()
        self.generate_turn_right()
        self.generate_braking()
        self.generate_accelerating()
        self.generate_error()
        self.generate_sad()
        self.generate_angry()
        self.generate_surprised()
        self.generate_sleep()
        self.generate_wakeup()
        self.generate_low_battery()
        self.generate_thinking()
        self.generate_victory()
        
        print("\n✅ All frames generated successfully!")
        print(f"Output directory: {self.output_dir}")


def main():
    parser = argparse.ArgumentParser(
        description='Generate LED animation frames'
    )
    parser.add_argument(
        '--output-dir',
        default='src/rob_box_animations/animations/frames',
        help='Output directory for frames'
    )
    parser.add_argument(
        '--animation',
        choices=['police', 'road_service', 'idle', 'charging', 'happy', 
                'turn_left', 'ambulance', 'fire_truck', 'turn_right', 'braking',
                'accelerating', 'error', 'sad', 'angry', 'surprised', 'sleep',
                'wakeup', 'low_battery', 'thinking', 'victory', 'all'],
        default='all',
        help='Which animation to generate'
    )
    
    args = parser.parse_args()
    
    generator = FrameGenerator(args.output_dir)
    
    animation_map = {
        'police': generator.generate_police_lights,
        'road_service': generator.generate_road_service,
        'idle': generator.generate_idle,
        'charging': generator.generate_charging,
        'happy': generator.generate_happy,
        'turn_left': generator.generate_turn_left,
        'ambulance': generator.generate_ambulance,
        'fire_truck': generator.generate_fire_truck,
        'turn_right': generator.generate_turn_right,
        'braking': generator.generate_braking,
        'accelerating': generator.generate_accelerating,
        'error': generator.generate_error,
        'sad': generator.generate_sad,
        'angry': generator.generate_angry,
        'surprised': generator.generate_surprised,
        'sleep': generator.generate_sleep,
        'wakeup': generator.generate_wakeup,
        'low_battery': generator.generate_low_battery,
        'thinking': generator.generate_thinking,
        'victory': generator.generate_victory,
    }
    
    if args.animation == 'all':
        generator.generate_all()
    else:
        animation_map[args.animation]()


if __name__ == '__main__':
    main()
