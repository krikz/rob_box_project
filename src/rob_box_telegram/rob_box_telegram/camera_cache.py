#!/usr/bin/env python3
"""
camera_cache.py — Subscribes to compressed image topics and caches the latest frame.

Provides instant photo snapshots for Telegram /photo command
without blocking on ROS 2 subscription callbacks.
"""

import logging
import threading
import time
from typing import Dict, Optional, Tuple

logger = logging.getLogger(__name__)


class CameraCache:
    """Thread-safe cache for the latest compressed image frame per topic.

    Attributes:
        _frames: Dict mapping topic name to (jpeg_bytes, timestamp) tuple.
        _lock: Threading lock for concurrent access.
        ttl: Maximum age in seconds before a frame is considered stale.
    """

    def __init__(self, ttl: float = 5.0):
        self._frames: Dict[str, Tuple[bytes, float]] = {}
        self._lock = threading.Lock()
        self.ttl = ttl

    def update(self, topic: str, jpeg_data: bytes) -> None:
        """Store a new frame (called from ROS 2 subscription callback).

        Args:
            topic: ROS topic name (e.g. "/camera/rgb/image_raw/compressed").
            jpeg_data: Raw JPEG bytes from CompressedImage.data.
        """
        with self._lock:
            self._frames[topic] = (jpeg_data, time.monotonic())

    def get(self, topic: str) -> Optional[bytes]:
        """Get the latest frame if it's still fresh.

        Args:
            topic: ROS topic name.

        Returns:
            JPEG bytes or None if no frame or frame is stale.
        """
        with self._lock:
            entry = self._frames.get(topic)
            if entry is None:
                return None
            jpeg_data, ts = entry
            if time.monotonic() - ts > self.ttl:
                logger.debug("Frame for %s is stale (%.1fs old)", topic, time.monotonic() - ts)
                return None
            return jpeg_data

    def get_age(self, topic: str) -> Optional[float]:
        """Get the age of the cached frame in seconds.

        Returns:
            Age in seconds or None if no frame cached.
        """
        with self._lock:
            entry = self._frames.get(topic)
            if entry is None:
                return None
            return time.monotonic() - entry[1]

    @property
    def topics(self) -> list:
        """List of topics with cached frames."""
        with self._lock:
            return list(self._frames.keys())
