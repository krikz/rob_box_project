"""
Animation Player

Plays animation sequences and publishes frames to LED panels.
"""

import time
from typing import Optional, Dict, List
from threading import Thread, Lock, Event
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from .animation_loader import AnimationLoader, AnimationManifest, PanelAnimation
from .frame_renderer import FrameRenderer


class AnimationPlayer:
    """
    Plays LED matrix animations

    Manages timing, looping, and multi-panel synchronization.
    """

    def __init__(
        self,
        node: Node,
        animations_dir: Optional[str] = None
    ):
        """
        Initialize player

        Args:
            node: ROS2 node for publishing
            animations_dir: Path to animations directory
        """
        self.node = node
        self.loader = AnimationLoader(animations_dir)
        self.renderer = FrameRenderer()

        # Publishers for each logical group
        self.publishers: Dict[str, rclpy.publisher.Publisher] = {}

        # Playback state
        self.current_animation: Optional[AnimationManifest] = None
        self.is_playing = False
        self.is_paused = False
        self.playback_thread: Optional[Thread] = None
        self.stop_event = Event()
        self.pause_event = Event()
        self.state_lock = Lock()

        # Statistics
        self.frames_played = 0
        self.loops_completed = 0

        self.node.get_logger().info('AnimationPlayer initialized')

    def load_animation(self, manifest_path: str) -> bool:
        """
        Load animation from manifest file

        Args:
            manifest_path: Path to manifest YAML

        Returns:
            True if loaded successfully
        """
        try:
            self.current_animation = self.loader.load_manifest(manifest_path)
            self.node.get_logger().info(
                f'Loaded animation: {self.current_animation.name} '
                f'({len(self.current_animation.panels)} panels)'
            )

            # Create publishers for required logical groups
            self._create_publishers()

            return True

        except Exception as e:
            self.node.get_logger().error(f'Failed to load animation: {e}')
            return False

    def _create_publishers(self):
        """Create ROS2 publishers for all panels in animation"""
        if not self.current_animation:
            return

        for panel in self.current_animation.panels:
            logical_group = panel.logical_group

            if logical_group not in self.publishers:
                pub = self.node.create_publisher(
                    Image,
                    '/panel_image',
                    10
                )
                self.publishers[logical_group] = pub
                self.node.get_logger().info(f'Created publisher for {logical_group}')

    def play(self) -> bool:
        """
        Start playing current animation

        Returns:
            True if playback started
        """
        with self.state_lock:
            if not self.current_animation:
                self.node.get_logger().error('No animation loaded')
                return False

            if self.is_playing:
                self.node.get_logger().warn('Animation already playing')
                return False

            self.is_playing = True
            self.is_paused = False
            self.stop_event.clear()
            self.pause_event.clear()
            self.frames_played = 0
            self.loops_completed = 0

            # Start playback thread
            self.playback_thread = Thread(target=self._playback_loop, daemon=True)
            self.playback_thread.start()

            self.node.get_logger().info(
                f'Started playing: {self.current_animation.name}'
            )
            return True

    def stop(self):
        """Stop playback"""
        with self.state_lock:
            if not self.is_playing:
                return

            self.is_playing = False
            self.stop_event.set()
            self.pause_event.set()  # Unpause if paused

        # Wait for thread to finish
        if self.playback_thread:
            self.playback_thread.join(timeout=2.0)

        self.node.get_logger().info('Playback stopped')

    def pause(self):
        """Pause playback"""
        with self.state_lock:
            if not self.is_playing or self.is_paused:
                return

            self.is_paused = True
            self.node.get_logger().info('Playback paused')

    def resume(self):
        """Resume playback"""
        with self.state_lock:
            if not self.is_playing or not self.is_paused:
                return

            self.is_paused = False
            self.pause_event.set()
            self.node.get_logger().info('Playback resumed')

    def _playback_loop(self):
        """Main playback loop (runs in separate thread)"""
        animation = self.current_animation

        if not animation:
            return

        try:
            while not self.stop_event.is_set():
                # Play one cycle
                self._play_cycle()

                self.loops_completed += 1

                # Check if should loop
                if not animation.loop:
                    break

        except Exception as e:
            self.node.get_logger().error(f'Playback error: {e}')

        finally:
            with self.state_lock:
                self.is_playing = False

    def _play_cycle(self):
        """Play one complete animation cycle"""
        animation = self.current_animation

        if not animation:
            return

        # Build timeline of all frame changes
        timeline = self._build_timeline()

        start_time = time.time()

        for timestamp_ms, panel_frames in timeline:
            # Check if should stop
            if self.stop_event.is_set():
                break

            # Check if paused
            if self.is_paused:
                self.pause_event.wait()
                self.pause_event.clear()
                start_time = time.time() - (timestamp_ms / 1000.0)

            # Wait until timestamp
            target_time = start_time + (timestamp_ms / 1000.0)
            sleep_time = target_time - time.time()

            if sleep_time > 0:
                time.sleep(sleep_time)

            # Publish frames for all panels at this timestamp
            for logical_group, frame_path in panel_frames:
                try:
                    msg = self.renderer.render_frame(frame_path, logical_group)
                    msg.header.stamp = self.node.get_clock().now().to_msg()

                    # Use the same publisher for all messages (frame_id differentiates)
                    pub = self.publishers.get(logical_group)
                    if pub:
                        pub.publish(msg)
                        self.frames_played += 1

                except Exception as e:
                    self.node.get_logger().error(
                        f'Failed to render/publish frame: {e}'
                    )

    def _build_timeline(self) -> List[tuple]:
        """
        Build timeline of frame changes across all panels

        Returns:
            List of (timestamp_ms, [(logical_group, frame_path), ...])
        """
        animation = self.current_animation
        timeline = {}

        for panel in animation.panels:
            current_time = panel.offset_ms

            for frame in panel.frames:
                if current_time not in timeline:
                    timeline[current_time] = []

                timeline[current_time].append(
                    (panel.logical_group, frame.image)
                )

                current_time += frame.duration_ms

        # Sort by timestamp
        sorted_timeline = sorted(timeline.items())

        return sorted_timeline

    def get_status(self) -> Dict:
        """
        Get current playback status

        Returns:
            Dictionary with status information
        """
        with self.state_lock:
            return {
                'is_playing': self.is_playing,
                'is_paused': self.is_paused,
                'animation': self.current_animation.name if self.current_animation else None,
                'frames_played': self.frames_played,
                'loops_completed': self.loops_completed,
            }

    def list_animations(self) -> List[str]:
        """List available animations"""
        return self.loader.list_available_animations()
