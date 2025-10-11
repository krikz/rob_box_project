"""
LED Animation Manifest Schema

YAML format for defining LED matrix animations for rob_box robot.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict
from enum import Enum


class AnimationPriority(Enum):
    """Animation priority levels"""
    CRITICAL = "critical"  # Emergency stops, errors
    HIGH = "high"          # Emergency services, warnings
    NORMAL = "normal"      # Robot states, missions
    LOW = "low"            # Idle, decorative


class AnimationCategory(Enum):
    """Animation categories"""
    EMERGENCY = "emergency"      # Police, ambulance, road service
    STATE = "state"             # Idle, charging, active
    MISSION = "mission"         # Mission states
    SYSTEM = "system"           # Startup, shutdown
    EMOTION = "emotion"         # Happy, sad, angry, surprised
    NOTIFICATION = "notification"  # Alerts, warnings, info
    NAVIGATION = "navigation"   # Turn signals, braking
    INTERACTION = "interaction"  # Greeting, acknowledgment
    DIAGNOSTIC = "diagnostic"   # Self-test, calibration
    DECORATIVE = "decorative"   # Fun animations


@dataclass
class Frame:
    """Single animation frame"""
    image: str              # Path to PNG file
    duration_ms: int        # Frame duration in milliseconds
    metadata: Dict = field(default_factory=dict)  # Optional metadata


@dataclass
class PanelAnimation:
    """Animation sequence for a single logical group"""
    logical_group: str      # LED panel logical group name
    frames: List[Frame]     # List of frames
    offset_ms: int = 0      # Start offset for sync between panels


@dataclass
class AnimationManifest:
    """Complete animation manifest"""
    # Basic info
    name: str
    description: str
    version: str = "1.0"
    author: str = "rob_box"
    
    # Timing
    duration_ms: int        # Total animation duration
    loop: bool = True       # Loop animation
    fps: int = 10          # Target frames per second
    
    # Panels
    panels: List[PanelAnimation]
    
    # Metadata
    priority: AnimationPriority = AnimationPriority.NORMAL
    category: AnimationCategory = AnimationCategory.STATE
    tags: List[str] = field(default_factory=list)
    
    # Transitions
    fade_in_ms: int = 0     # Fade in duration
    fade_out_ms: int = 0    # Fade out duration
    
    # Conditions
    requires_state: Optional[str] = None  # Required robot state
    min_battery: Optional[int] = None     # Minimum battery level
    
    def get_total_frames(self) -> int:
        """Calculate total number of unique frames"""
        total = 0
        for panel in self.panels:
            total += len(panel.frames)
        return total
    
    def validate(self) -> bool:
        """Validate manifest structure"""
        if not self.name or not self.description:
            return False
        if not self.panels:
            return False
        for panel in self.panels:
            if not panel.logical_group or not panel.frames:
                return False
        return True


# Example usage
if __name__ == "__main__":
    # Create example manifest
    manifest = AnimationManifest(
        name="police_lights",
        description="Police emergency lights",
        duration_ms=1000,
        loop=True,
        fps=10,
        priority=AnimationPriority.HIGH,
        category=AnimationCategory.EMERGENCY,
        tags=["police", "emergency", "lights"],
        panels=[
            PanelAnimation(
                logical_group="headlight_front_left",
                frames=[
                    Frame(image="frames/police/front_left_001.png", duration_ms=100),
                    Frame(image="frames/police/front_left_002.png", duration_ms=100),
                ]
            ),
            PanelAnimation(
                logical_group="headlight_front_right",
                frames=[
                    Frame(image="frames/police/front_right_001.png", duration_ms=100),
                    Frame(image="frames/police/front_right_002.png", duration_ms=100),
                ]
            ),
        ]
    )
    
    print(f"Manifest: {manifest.name}")
    print(f"Total frames: {manifest.get_total_frames()}")
    print(f"Valid: {manifest.validate()}")
