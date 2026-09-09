#!/usr/bin/env python3
"""Tests for rob_box_telegram.camera_cache module."""

import time
import unittest

from rob_box_telegram.camera_cache import CameraCache


class TestCameraCache(unittest.TestCase):
    """Tests for CameraCache thread-safe frame caching."""

    def test_update_and_get(self):
        cache = CameraCache(ttl=5.0)
        jpeg = b"\xff\xd8\xff\xe0fake_jpeg_data"
        cache.update("/camera/rgb", jpeg)
        self.assertEqual(cache.get("/camera/rgb"), jpeg)

    def test_get_missing_topic(self):
        cache = CameraCache(ttl=5.0)
        self.assertIsNone(cache.get("/nonexistent"))

    def test_stale_frame_returns_none(self):
        cache = CameraCache(ttl=0.01)  # 10ms TTL
        cache.update("/camera/rgb", b"data")
        time.sleep(0.02)
        self.assertIsNone(cache.get("/camera/rgb"))

    def test_get_age(self):
        cache = CameraCache(ttl=10.0)
        self.assertIsNone(cache.get_age("/camera/rgb"))
        cache.update("/camera/rgb", b"data")
        age = cache.get_age("/camera/rgb")
        self.assertIsNotNone(age)
        self.assertLess(age, 1.0)

    def test_topics_list(self):
        cache = CameraCache(ttl=5.0)
        cache.update("/cam1", b"a")
        cache.update("/cam2", b"b")
        self.assertIn("/cam1", cache.topics)
        self.assertIn("/cam2", cache.topics)

    def test_overwrite_frame(self):
        cache = CameraCache(ttl=5.0)
        cache.update("/camera/rgb", b"old")
        cache.update("/camera/rgb", b"new")
        self.assertEqual(cache.get("/camera/rgb"), b"new")


if __name__ == "__main__":
    unittest.main()
