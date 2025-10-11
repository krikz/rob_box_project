#!/usr/bin/env python3
"""
Audio-Reactive Animation Node

Monitors system audio output and triggers/modulates animations based on audio level.
Synchronizes LED mouth movements with sound output (like Bender from Futurama).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger
import numpy as np
import threading
import time
import yaml
from pathlib import Path

try:
    import pyaudio
    PYAUDIO_AVAILABLE = True
except ImportError:
    PYAUDIO_AVAILABLE = False
    print("Warning: pyaudio not available. Audio reactivity disabled.")


class AudioReactiveAnimationNode(Node):
    """
    ROS2 node for audio-reactive LED animations.
    
    Monitors system audio output and controls animation playback
    based on audio volume levels.
    """
    
    def __init__(self):
        super().__init__('audio_reactive_animation_node')
        
        # Parameters
        self.declare_parameter('animations_dir', 'src/rob_box_animations/animations')
        self.declare_parameter('audio_device_index', -1)  # -1 = default
        self.declare_parameter('sample_rate', 44100)
        self.declare_parameter('chunk_size', 1024)
        
        self.animations_dir = Path(self.get_parameter('animations_dir').value)
        self.audio_device_index = self.get_parameter('audio_device_index').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_size = self.get_parameter('chunk_size').value
        
        # Publishers
        self.audio_level_pub = self.create_publisher(
            Float32, 
            '/audio/level', 
            10
        )
        
        self.animation_trigger_pub = self.create_publisher(
            String,
            '/animation/audio_trigger',
            10
        )
        
        # Subscribers
        self.audio_enable_sub = self.create_subscription(
            String,
            '/audio/enable_reactive',
            self.audio_enable_callback,
            10
        )
        
        # State
        self.audio_reactive_enabled = False
        self.current_animation = None
        self.audio_threshold = 0.3
        self.audio_smoothing = 0.2
        self.smoothed_volume = 0.0
        
        # Audio stream
        self.audio_stream = None
        self.audio_thread = None
        self.running = False
        
        # Initialize PyAudio if available
        if PYAUDIO_AVAILABLE:
            self.pyaudio = pyaudio.PyAudio()
            self.get_logger().info("PyAudio initialized successfully")
        else:
            self.pyaudio = None
            self.get_logger().warning("PyAudio not available - audio reactivity disabled")
        
        self.get_logger().info("Audio-Reactive Animation Node started")
    
    def audio_enable_callback(self, msg: String):
        """Enable/disable audio-reactive animation"""
        animation_name = msg.data
        
        if animation_name == "stop":
            self.stop_audio_monitoring()
            self.audio_reactive_enabled = False
            self.get_logger().info("Audio reactivity disabled")
            return
        
        # Load animation manifest
        manifest_path = self.animations_dir / 'manifests' / f'{animation_name}.yaml'
        
        if not manifest_path.exists():
            self.get_logger().error(f"Animation manifest not found: {manifest_path}")
            return
        
        try:
            with open(manifest_path, 'r') as f:
                manifest = yaml.safe_load(f)
            
            # Check if animation is audio-reactive
            if not manifest.get('audio_reactive', False):
                self.get_logger().warning(
                    f"Animation '{animation_name}' is not audio-reactive"
                )
                return
            
            self.current_animation = manifest
            self.audio_threshold = manifest.get('audio_threshold', 0.3)
            self.audio_smoothing = manifest.get('audio_smoothing', 0.2)
            
            # Start audio monitoring
            self.start_audio_monitoring()
            self.audio_reactive_enabled = True
            
            self.get_logger().info(
                f"Audio reactivity enabled for '{animation_name}' "
                f"(threshold: {self.audio_threshold}, smoothing: {self.audio_smoothing})"
            )
            
        except Exception as e:
            self.get_logger().error(f"Failed to load animation: {e}")
    
    def start_audio_monitoring(self):
        """Start monitoring audio output"""
        if not PYAUDIO_AVAILABLE or self.pyaudio is None:
            self.get_logger().error("Cannot start audio monitoring - PyAudio not available")
            return
        
        if self.running:
            return
        
        try:
            # Open audio stream (monitoring output)
            # Note: This monitors loopback/stereo mix if available
            device_index = self.audio_device_index if self.audio_device_index >= 0 else None
            
            self.audio_stream = self.pyaudio.open(
                format=pyaudio.paInt16,
                channels=2,
                rate=self.sample_rate,
                input=True,
                input_device_index=device_index,
                frames_per_buffer=self.chunk_size,
                stream_callback=self.audio_callback
            )
            
            self.running = True
            self.audio_stream.start_stream()
            
            self.get_logger().info("Audio monitoring started")
            
        except Exception as e:
            self.get_logger().error(f"Failed to start audio stream: {e}")
            self.get_logger().info(
                "Tip: You may need to enable 'Stereo Mix' or 'Loopback' "
                "in your audio settings"
            )
    
    def stop_audio_monitoring(self):
        """Stop monitoring audio output"""
        self.running = False
        
        if self.audio_stream is not None:
            self.audio_stream.stop_stream()
            self.audio_stream.close()
            self.audio_stream = None
            
            self.get_logger().info("Audio monitoring stopped")
    
    def audio_callback(self, in_data, frame_count, time_info, status):
        """Callback for audio stream processing"""
        if not self.audio_reactive_enabled:
            return (in_data, pyaudio.paContinue)
        
        # Convert audio data to numpy array
        audio_data = np.frombuffer(in_data, dtype=np.int16)
        
        # Calculate RMS volume
        rms = np.sqrt(np.mean(audio_data**2))
        
        # Normalize to 0.0-1.0 range (assuming 16-bit audio)
        volume = min(rms / 32768.0, 1.0)
        
        # Apply smoothing (exponential moving average)
        self.smoothed_volume = (
            self.audio_smoothing * volume + 
            (1 - self.audio_smoothing) * self.smoothed_volume
        )
        
        # Publish audio level
        level_msg = Float32()
        level_msg.data = self.smoothed_volume
        self.audio_level_pub.publish(level_msg)
        
        # Trigger animation update if above threshold
        if self.smoothed_volume > self.audio_threshold:
            trigger_msg = String()
            trigger_msg.data = f"{self.smoothed_volume:.2f}"
            self.animation_trigger_pub.publish(trigger_msg)
        
        return (in_data, pyaudio.paContinue)
    
    def destroy_node(self):
        """Cleanup on node shutdown"""
        self.stop_audio_monitoring()
        
        if self.pyaudio is not None:
            self.pyaudio.terminate()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = AudioReactiveAnimationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
