"""Фейковый std_msgs/String (см. rclpy/__init__.py)."""


class String:
    def __init__(self, data=""):
        self.data = data
