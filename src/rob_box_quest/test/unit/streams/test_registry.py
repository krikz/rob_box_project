"""Unit-тесты streams.registry."""

from rob_box_quest.streams.registry import (
    STREAM_CATALOG,
    StreamKind,
    get_stream,
    list_streams,
    topic_id_for,
)


def test_catalog_has_core_streams():
    for name in ("lidar_2d", "robot_status", "voice_state", "camera_oak_color", "camera_ceiling"):
        assert name in STREAM_CATALOG, f"missing {name}"


def test_kind_assignment():
    assert STREAM_CATALOG["lidar_2d"].kind == StreamKind.ROS_TOPIC
    assert STREAM_CATALOG["camera_oak_color"].kind == StreamKind.CAMERA_DIRECT
    # camera_ceiling — ROS-стрим, а не CAMERA_DIRECT: /dev/video0 держит
    # эксклюзивно контейнер `ceiling-camera` (usb_cam) и в rob-box-quest
    # устройство вообще не прокинуто, так что прямое чтение не давало ни
    # одного кадра. Берём готовый JPEG из image_transport-топика.
    assert STREAM_CATALOG["camera_ceiling"].kind == StreamKind.ROS_TOPIC
    assert STREAM_CATALOG["camera_ceiling"].source == "/ceiling_camera/image_raw/compressed"


def test_map_stream_is_registered():
    spec = STREAM_CATALOG["map_2d"]
    assert spec.kind == StreamKind.ROS_TOPIC
    assert spec.source == "/rtabmap/map"
    assert topic_id_for("map_2d") == 0x1103


def test_topic_ids_match_protocol():
    # Сверяемся с meta-quest-api.md §4 + Phase 1.4 v2 (новые id для камер).
    assert topic_id_for("lidar_2d") == 0x1101
    assert topic_id_for("robot_status") == 0x1201
    assert topic_id_for("voice_state") == 0x1202
    assert topic_id_for("camera_oak_color") == 0x1003
    assert topic_id_for("camera_oak_depth") == 0x1004
    assert topic_id_for("camera_ceiling") == 0x1005


def test_get_stream_unknown_returns_none():
    assert get_stream("totally_bogus_stream") is None


def test_get_stream_known_returns_spec():
    spec = get_stream("lidar_2d")
    assert spec is not None
    assert spec.ui_name == "lidar_2d"
    assert spec.source == "/scan"


def test_list_streams_returns_all():
    streams = list_streams()
    assert len(streams) == len(STREAM_CATALOG)
    names = {s.ui_name for s in streams}
    assert "lidar_2d" in names
    assert "camera_oak_color" in names


def test_default_quality_in_range():
    for spec in STREAM_CATALOG.values():
        assert spec.default_quality in ("low", "med", "high"), f"{spec.ui_name}: {spec.default_quality}"
