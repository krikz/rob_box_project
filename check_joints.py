#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node('check_joints')

count = 0
def callback(msg):
    global count
    count += 1
    if count <= 5:
        print(f'\n=== Sample {count} ===')
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            print(f'{name:20s}: pos={pos:8.4f} rad, vel={vel:8.4f} rad/s')
    if count >= 5:
        node.destroy_node()
        rclpy.shutdown()

sub = node.create_subscription(JointState, '/joint_states', callback, 10)

try:
    rclpy.spin(node)
except:
    pass
