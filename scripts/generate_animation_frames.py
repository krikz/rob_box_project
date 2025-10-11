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
        
        # Front left - blue/red alternating
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLUE, 
                              "front_left_blue.png", subdir)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.RED, 
                              "front_left_red.png", subdir)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK, 
                              "front_left_off.png", subdir)
        
        # Front right - blue/red alternating (opposite phase)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLUE, 
                              "front_right_blue.png", subdir)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.RED, 
                              "front_right_red.png", subdir)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK, 
                              "front_right_off.png", subdir)
        
        # Rear lights
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLUE, 
                              "rear_blue.png", subdir)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.RED, 
                              "rear_red.png", subdir)
        self.create_solid_frame(self.HEADLIGHT_SIZE, self.BLACK, 
                              "rear_off.png", subdir)
        
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
            pattern = self._create_battery_indicator(i, 16, self.GREEN)
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
    
    def generate_all(self):
        """Generate all animation frames"""
        print("Generating all LED matrix animation frames...")
        
        self.generate_police_lights()
        self.generate_road_service()
        self.generate_idle()
        self.generate_charging()
        self.generate_happy()
        self.generate_turn_left()
        
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
                'turn_left', 'all'],
        default='all',
        help='Which animation to generate'
    )
    
    args = parser.parse_args()
    
    generator = FrameGenerator(args.output_dir)
    
    if args.animation == 'all':
        generator.generate_all()
    elif args.animation == 'police':
        generator.generate_police_lights()
    elif args.animation == 'road_service':
        generator.generate_road_service()
    elif args.animation == 'idle':
        generator.generate_idle()
    elif args.animation == 'charging':
        generator.generate_charging()
    elif args.animation == 'happy':
        generator.generate_happy()
    elif args.animation == 'turn_left':
        generator.generate_turn_left()


if __name__ == '__main__':
    main()
