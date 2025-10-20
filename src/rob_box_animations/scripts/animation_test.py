#!/usr/bin/env python3
"""
Animation Test Script

Simple CLI tool to test animations.
"""

import rclpy
from rclpy.node import Node
from std_msgs.srv import String
from std_srvs.srv import Trigger
import sys
import argparse
import time


def main():
    parser = argparse.ArgumentParser(description='Test LED animations')
    parser.add_argument('animation', help='Animation name to play')
    parser.add_argument('--duration', type=float, default=10.0,
                       help='Duration to play in seconds (default: 10)')
    parser.add_argument('--no-loop', action='store_true',
                       help='Play once without looping')

    args = parser.parse_args()

    rclpy.init()
    node = Node('animation_test')

    # Wait for services
    node.get_logger().info('Waiting for animation_player services...')

    load_client = node.create_client(
        String,
        '/animation_player/load_animation'
    )

    play_client = node.create_client(
        Trigger,
        '/animation_player/play'
    )

    stop_client = node.create_client(
        Trigger,
        '/animation_player/stop'
    )

    # Wait for services to be available
    timeout = 5.0
    start_time = time.time()

    while not (load_client.service_is_ready() and
               play_client.service_is_ready() and
               stop_client.service_is_ready()):

        if time.time() - start_time > timeout:
            node.get_logger().error('Timeout waiting for services. Is animation_player running?')
            node.destroy_node()
            rclpy.shutdown()
            return 1

        time.sleep(0.1)

    node.get_logger().info(f'Loading animation: {args.animation}')

    # Load animation
    load_req = String.Request()
    load_req.data = args.animation
    load_future = load_client.call_async(load_req)

    rclpy.spin_until_future_complete(node, load_future, timeout_sec=2.0)

    if not load_future.result().success:
        node.get_logger().error(f'Failed to load animation: {load_future.result().message}')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node.get_logger().info('Animation loaded successfully')

    # Play animation
    play_req = Trigger.Request()
    play_future = play_client.call_async(play_req)

    rclpy.spin_until_future_complete(node, play_future, timeout_sec=2.0)

    if not play_future.result().success:
        node.get_logger().error(f'Failed to start playback: {play_future.result().message}')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node.get_logger().info(f'Playing animation for {args.duration} seconds...')

    # Let it play
    time.sleep(args.duration)

    # Stop
    node.get_logger().info('Stopping animation...')
    stop_req = Trigger.Request()
    stop_future = stop_client.call_async(stop_req)

    rclpy.spin_until_future_complete(node, stop_future, timeout_sec=2.0)

    node.get_logger().info('Test complete')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
