#!/usr/bin/env python3
"""
Animation Player Node

ROS2 node for playing LED matrix animations.
Provides services for loading and controlling animations.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from ament_index_python.packages import get_package_share_directory
import os

from rob_box_animations.animation_player import AnimationPlayer


class AnimationPlayerNode(Node):
    """ROS2 node for animation playback"""

    def __init__(self):
        super().__init__('animation_player')

        # Parameters
        self.declare_parameter('animations_dir', '')
        self.declare_parameter('autostart_animation', '')
        self.declare_parameter('loop', True)

        animations_dir = self.get_parameter('animations_dir').value
        if not animations_dir:
            # Use package share directory
            try:
                pkg_share = get_package_share_directory('rob_box_animations')
                animations_dir = os.path.join(pkg_share, 'animations')
            except Exception:
                animations_dir = None

        # Create player
        self.player = AnimationPlayer(self, animations_dir)

        # Services
        self.srv_load = self.create_service(
            String,
            '~/load_animation',
            self.load_animation_callback
        )

        self.srv_play = self.create_service(
            Trigger,
            '~/play',
            self.play_callback
        )

        self.srv_stop = self.create_service(
            Trigger,
            '~/stop',
            self.stop_callback
        )

        self.srv_pause = self.create_service(
            SetBool,
            '~/pause',
            self.pause_callback
        )

        self.srv_list = self.create_service(
            Trigger,
            '~/list_animations',
            self.list_animations_callback
        )

        # Status publisher
        self.status_pub = self.create_publisher(
            String,
            '~/status',
            10
        )

        # Status timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Animation Player Node started')

        # Autostart animation if specified
        autostart = self.get_parameter('autostart_animation').value
        if autostart:
            self.get_logger().info(f'Autostarting animation: {autostart}')
            if self.player.load_animation(f'{autostart}.yaml'):
                self.player.play()

    def load_animation_callback(self, request, response):
        """Load animation service callback"""
        manifest_path = request.data

        if not manifest_path.endswith('.yaml'):
            manifest_path += '.yaml'

        success = self.player.load_animation(manifest_path)

        response.success = success
        if success:
            response.message = f'Loaded animation: {manifest_path}'
        else:
            response.message = f'Failed to load animation: {manifest_path}'

        return response

    def play_callback(self, request, response):
        """Play animation service callback"""
        success = self.player.play()

        response.success = success
        if success:
            response.message = 'Playback started'
        else:
            response.message = 'Failed to start playback'

        return response

    def stop_callback(self, request, response):
        """Stop animation service callback"""
        self.player.stop()

        response.success = True
        response.message = 'Playback stopped'

        return response

    def pause_callback(self, request, response):
        """Pause/resume animation service callback"""
        if request.data:
            self.player.pause()
            response.success = True
            response.message = 'Playback paused'
        else:
            self.player.resume()
            response.success = True
            response.message = 'Playback resumed'

        return response

    def list_animations_callback(self, request, response):
        """List animations service callback"""
        animations = self.player.list_animations()

        response.success = True
        response.message = '\n'.join(animations) if animations else 'No animations found'

        return response

    def publish_status(self):
        """Publish current status"""
        status = self.player.get_status()

        msg = String()
        msg.data = (
            f"Animation: {status['animation']}, "
            f"Playing: {status['is_playing']}, "
            f"Paused: {status['is_paused']}, "
            f"Frames: {status['frames_played']}, "
            f"Loops: {status['loops_completed']}"
        )

        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = AnimationPlayerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.player.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
