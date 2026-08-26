"""Unit-тесты topic registry + payload-кодеков rob_box_quest.

Источник истины: docs/architecture/meta-quest-api.md §4 (BINARY_FRAME topic_id map).
"""

import struct

import msgpack
import pytest

from rob_box_quest.protocol.topics import (
    TOPIC_IDS,
    encode_lidar_2d,
    encode_person_detections,
    encode_robot_status,
    encode_voice_state,
)


class TestTopicRegistry:
    """Топики строго по meta-quest-api.md §4."""

    def test_camera_rear_topic_id(self):
        assert TOPIC_IDS["camera_rear"] == 0x1001

    def test_lidar_2d_topic_id(self):
        assert TOPIC_IDS["lidar_2d"] == 0x1101

    def test_robot_status_topic_id(self):
        assert TOPIC_IDS["robot_status"] == 0x1201

    def test_voice_state_topic_id(self):
        assert TOPIC_IDS["voice_state"] == 0x1202

    def test_voice_audio_preview_topic_id(self):
        assert TOPIC_IDS["voice_audio_preview"] == 0x1401

    def test_person_detections_topic_id(self):
        assert TOPIC_IDS["person_detections"] == 0x1301


class TestLidar2DPayload:
    """lidar_2d: header (8 × float32) + ranges + intensities, все little-endian."""

    def test_payload_size(self):
        n = 360
        payload = encode_lidar_2d(
            angle_min=-3.14159,
            angle_max=3.14159,
            angle_increment=0.01745,
            range_min=0.05,
            range_max=30.0,
            time_increment=0.0,
            scan_time=0.1,
            ranges=[1.0] * n,
            intensities=[0.5] * n,
        )
        # 8 floats header (32 bytes) + n*2 floats (4*2=8 байт на точку)
        assert len(payload) == 8 * 4 + n * 4 * 2

    def test_roundtrip_header(self):
        """Парсинг первых 32 байт: header поля в правильном порядке."""
        payload = encode_lidar_2d(
            angle_min=-1.0,
            angle_max=1.0,
            angle_increment=0.01,
            range_min=0.1,
            range_max=10.0,
            time_increment=0.0,
            scan_time=0.05,
            ranges=[1.5, 2.5],
            intensities=[0.8, 0.9],
        )
        header = struct.unpack_from("<ffffffff", payload, 0)
        assert header == pytest.approx((-1.0, 1.0, 0.01, 0.1, 10.0, 0.0, 0.05, 2), rel=1e-5)


class TestRobotStatusPayload:
    """robot_status: msgpack-dict с фиксированным набором ключей."""

    def test_required_keys(self):
        payload = encode_robot_status(
            battery_pct=85,
            wifi_rssi=-60,
            mode="teleop_active",
            vel_linear=0.5,
            vel_angular=0.1,
            ts_ms=1234567890,
        )
        decoded = msgpack.unpackb(payload, raw=False)
        assert decoded["battery_pct"] == 85
        assert decoded["wifi_rssi"] == -60
        assert decoded["mode"] == "teleop_active"
        assert decoded["vel_linear"] == 0.5
        assert decoded["vel_angular"] == 0.1
        assert decoded["ts_ms"] == 1234567890


class TestPersonDetectionsPayload:
    """person_detections: msgpack, контракт задаём в Phase 1 для Phase 2."""

    def test_payload_has_detections_key(self):
        payload = encode_person_detections(
            ts_ms=1234567890,
            detections=[
                {"id": 1, "cls": "person", "x": 0.5, "y": 0.0, "z": 1.7, "conf": 0.95},
            ],
        )
        decoded = msgpack.unpackb(payload, raw=False)
        assert "detections" in decoded
        assert isinstance(decoded["detections"], list)
        assert len(decoded["detections"]) == 1
        assert decoded["detections"][0]["cls"] == "person"


class TestVoiceStatePayload:
    """Phase 2: voice_state payload расширен полями для TTS picker."""

    def test_required_keys(self):
        payload = encode_voice_state(
            active_voice_id="anton",
            active_preset="standard",
            listening=False,
            last_error=None,
            ts_ms=1234567890,
        )
        decoded = msgpack.unpackb(payload, raw=False)
        assert decoded["active_voice_id"] == "anton"
        assert decoded["active_preset"] == "standard"
        assert decoded["listening"] is False
        assert decoded["last_error"] is None
        assert decoded["ts_ms"] == 1234567890
        # legacy-поле для v1 клиентов сохранено.
        assert decoded["state"] == "speaking"

    def test_with_error_string(self):
        payload = encode_voice_state(
            active_voice_id="alena",
            active_preset="friendly",
            listening=True,
            last_error="voice-pipeline offline",
            ts_ms=1,
        )
        decoded = msgpack.unpackb(payload, raw=False)
        assert decoded["last_error"] == "voice-pipeline offline"
        assert decoded["listening"] is True
