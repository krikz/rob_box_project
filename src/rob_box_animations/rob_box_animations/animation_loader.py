"""
Animation Loader

Loads animation manifests from YAML files and validates them.
"""

import yaml
import os
from pathlib import Path
from typing import Optional, Dict, List
from dataclasses import dataclass, field


@dataclass
class Frame:
    """Single animation frame"""
    image: str
    duration_ms: int
    metadata: Dict = field(default_factory=dict)


@dataclass
class PanelAnimation:
    """Animation for a single LED panel"""
    logical_group: str
    frames: List[Frame]
    offset_ms: int = 0


@dataclass
class AnimationManifest:
    """Complete animation manifest"""
    name: str
    description: str
    version: str
    author: str
    duration_ms: int
    loop: bool
    fps: int
    panels: List[PanelAnimation]
    priority: str
    category: str
    tags: List[str]
    fade_in_ms: int = 0
    fade_out_ms: int = 0
    requires_state: Optional[str] = None
    min_battery: Optional[int] = None


class AnimationLoader:
    """Loads and validates animation manifests"""
    
    def __init__(self, animations_dir: Optional[str] = None):
        """
        Initialize loader
        
        Args:
            animations_dir: Path to animations directory. If None, uses package share directory.
        """
        self.animations_dir = animations_dir
        self.manifests_cache: Dict[str, AnimationManifest] = {}
    
    def load_manifest(self, manifest_path: str) -> AnimationManifest:
        """
        Load animation manifest from YAML file
        
        Args:
            manifest_path: Path to manifest YAML file
            
        Returns:
            AnimationManifest object
            
        Raises:
            FileNotFoundError: If manifest file not found
            ValueError: If manifest is invalid
        """
        # Check cache first
        if manifest_path in self.manifests_cache:
            return self.manifests_cache[manifest_path]
        
        # Resolve full path
        if self.animations_dir and not os.path.isabs(manifest_path):
            full_path = os.path.join(self.animations_dir, 'manifests', manifest_path)
        else:
            full_path = manifest_path
        
        if not os.path.exists(full_path):
            raise FileNotFoundError(f"Manifest not found: {full_path}")
        
        # Load YAML
        with open(full_path, 'r') as f:
            data = yaml.safe_load(f)
        
        # Parse manifest
        manifest = self._parse_manifest(data, full_path)
        
        # Validate
        self._validate_manifest(manifest, full_path)
        
        # Cache and return
        self.manifests_cache[manifest_path] = manifest
        return manifest
    
    def _parse_manifest(self, data: Dict, manifest_path: str) -> AnimationManifest:
        """Parse YAML data into AnimationManifest"""
        
        # Parse panels
        panels = []
        for panel_data in data.get('panels', []):
            frames = []
            for frame_data in panel_data.get('frames', []):
                # Resolve frame image path relative to manifest
                image_path = frame_data['image']
                if not os.path.isabs(image_path):
                    manifest_dir = os.path.dirname(manifest_path)
                    if self.animations_dir:
                        image_path = os.path.join(self.animations_dir, image_path)
                    else:
                        image_path = os.path.join(manifest_dir, image_path)
                
                frame = Frame(
                    image=image_path,
                    duration_ms=frame_data['duration_ms'],
                    metadata=frame_data.get('metadata', {})
                )
                frames.append(frame)
            
            panel = PanelAnimation(
                logical_group=panel_data['logical_group'],
                frames=frames,
                offset_ms=panel_data.get('offset_ms', 0)
            )
            panels.append(panel)
        
        # Parse metadata
        metadata = data.get('metadata', {})
        
        # Create manifest
        manifest = AnimationManifest(
            name=data['name'],
            description=data['description'],
            version=data.get('version', '1.0'),
            author=data.get('author', 'unknown'),
            duration_ms=data['duration_ms'],
            loop=data.get('loop', True),
            fps=data.get('fps', 10),
            panels=panels,
            priority=metadata.get('priority', 'normal'),
            category=metadata.get('category', 'state'),
            tags=metadata.get('tags', []),
            fade_in_ms=data.get('fade_in_ms', 0),
            fade_out_ms=data.get('fade_out_ms', 0),
            requires_state=data.get('requires_state'),
            min_battery=data.get('min_battery')
        )
        
        return manifest
    
    def _validate_manifest(self, manifest: AnimationManifest, manifest_path: str):
        """Validate manifest structure and referenced files"""
        
        if not manifest.name:
            raise ValueError(f"Manifest missing name: {manifest_path}")
        
        if not manifest.panels:
            raise ValueError(f"Manifest has no panels: {manifest_path}")
        
        # Validate each panel
        for panel in manifest.panels:
            if not panel.logical_group:
                raise ValueError(f"Panel missing logical_group: {manifest_path}")
            
            if not panel.frames:
                raise ValueError(f"Panel {panel.logical_group} has no frames: {manifest_path}")
            
            # Validate frame images exist
            for frame in panel.frames:
                if not os.path.exists(frame.image):
                    raise ValueError(f"Frame image not found: {frame.image}")
    
    def list_available_animations(self) -> List[str]:
        """
        List all available animation manifests
        
        Returns:
            List of animation names
        """
        if not self.animations_dir:
            return []
        
        manifests_dir = os.path.join(self.animations_dir, 'manifests')
        if not os.path.exists(manifests_dir):
            return []
        
        animations = []
        for file in os.listdir(manifests_dir):
            if file.endswith('.yaml'):
                try:
                    manifest = self.load_manifest(file)
                    animations.append(manifest.name)
                except Exception:
                    pass  # Skip invalid manifests
        
        return sorted(animations)
    
    def get_animation_info(self, animation_name: str) -> Optional[Dict]:
        """
        Get basic info about an animation without loading all frames
        
        Args:
            animation_name: Name of animation
            
        Returns:
            Dictionary with animation info or None if not found
        """
        manifest_path = f"{animation_name}.yaml"
        try:
            manifest = self.load_manifest(manifest_path)
            return {
                'name': manifest.name,
                'description': manifest.description,
                'category': manifest.category,
                'priority': manifest.priority,
                'duration_ms': manifest.duration_ms,
                'fps': manifest.fps,
                'loop': manifest.loop,
                'tags': manifest.tags
            }
        except Exception:
            return None
