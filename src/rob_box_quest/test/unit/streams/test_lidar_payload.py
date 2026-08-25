"""Unit-тесты streams.lidar (чистая функция поверх encode_lidar_2d)."""

import struct

from rob_box_quest.protocol.topics import encode_lidar_2d
from rob_box_quest.streams.lidar import scan_to_payload


def test_scan_to_payload_passes_through():
    n = 360
    kwargs = dict(
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
    direct = encode_lidar_2d(**kwargs)
    through = scan_to_payload(**kwargs)
    assert direct == through
    # Размер: 8 floats header + 2 * n floats data.
    assert len(through) == 8 * 4 + n * 4 * 2


def test_scan_to_payload_matches_laser_scan_fields():
    n = 4
    payload = scan_to_payload(
        angle_min=-1.0,
        angle_max=1.0,
        angle_increment=0.5,
        range_min=0.1,
        range_max=10.0,
        time_increment=0.01,
        scan_time=0.05,
        ranges=[1.0, 2.0, 3.0, 4.0],
        intensities=[0.5, 0.6, 0.7, 0.8],
    )
    header = struct.unpack_from("<ffffffff", payload, 0)
    # n_points = float(4) в последнем поле.
    assert header[-1] == float(n)


def test_scan_to_payload_mismatched_lengths_raises():
    import pytest

    with pytest.raises(ValueError):
        scan_to_payload(
            angle_min=0,
            angle_max=1,
            angle_increment=0.1,
            range_min=0,
            range_max=10,
            time_increment=0,
            scan_time=0,
            ranges=[1.0, 2.0],
            intensities=[0.5],
        )
