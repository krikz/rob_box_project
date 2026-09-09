"""Unit-тесты ``streams.occupancy`` (map_2d payload: PNG решётки + поза)."""

from __future__ import annotations

import pytest

from rob_box_quest.streams.occupancy import (
    FREE_ALPHA,
    OCCUPIED_ALPHA,
    OCCUPIED_RGB,
    OCCUPIED_THRESHOLD,
    encode_map_2d,
    grid_to_png,
)

msgpack = pytest.importorskip("msgpack")
np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")


def decode_png(png: bytes):
    """PNG → BGRA-массив (h, w, 4), как его увидит браузер."""
    arr = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED)
    assert arr is not None, "cv2 не смог прочитать PNG обратно"
    return arr


class TestGridToPng:
    def test_encodes_rgba_of_the_right_size(self):
        png = grid_to_png([-1] * (4 * 3), width=4, height=3)
        arr = decode_png(png)
        assert arr.shape == (3, 4, 4)  # h, w, RGBA

    def test_unknown_cells_are_fully_transparent(self):
        # Неизвестная клетка не должна закрашивать пол мостика — иначе
        # «карты нет» выглядело бы как «вокруг сплошная стена».
        arr = decode_png(grid_to_png([-1] * 4, width=2, height=2))
        assert (arr[:, :, 3] == 0).all()

    def test_free_and_occupied_get_different_alpha(self):
        arr = decode_png(grid_to_png([0, 100], width=2, height=1))
        assert arr[0, 0, 3] == FREE_ALPHA
        assert arr[0, 1, 3] == OCCUPIED_ALPHA

    def test_occupied_cells_use_the_bridge_edge_colour(self):
        arr = decode_png(grid_to_png([100], width=1, height=1))
        b, g, r, _a = arr[0, 0]
        assert (int(r), int(g), int(b)) == OCCUPIED_RGB

    def test_threshold_splits_free_from_occupied(self):
        below, above = OCCUPIED_THRESHOLD - 1, OCCUPIED_THRESHOLD
        arr = decode_png(grid_to_png([below, above], width=2, height=1))
        assert arr[0, 0, 3] == FREE_ALPHA
        assert arr[0, 1, 3] == OCCUPIED_ALPHA

    def test_rows_are_flipped_so_map_y_grows_upwards(self):
        # OccupancyGrid.data[0] — клетка у origin, то есть НИЖНЯЯ строка по
        # оси Y карты. Нулевая строка PNG — верхняя. Без переворота карта
        # на полу оказалась бы зеркальной по ходу движения.
        # Решётка 1×2: нижняя клетка занята, верхняя свободна.
        arr = decode_png(grid_to_png([100, 0], width=1, height=2))
        assert arr[0, 0, 3] == FREE_ALPHA  # верх PNG = верхняя строка карты
        assert arr[1, 0, 3] == OCCUPIED_ALPHA  # низ PNG = data[0]

    @pytest.mark.parametrize(
        "data,w,h",
        [([0] * 3, 2, 2), ([], 1, 1), ([0], 0, 1), ([0], 1, 0)],
    )
    def test_rejects_inconsistent_grid(self, data, w, h):
        with pytest.raises(ValueError):
            grid_to_png(data, width=w, height=h)


class TestEncodeMap2d:
    BASE = dict(
        resolution=0.05,
        width=958,
        height=744,
        origin_x=5.25,
        origin_y=-25.25,
        robot_x=34.5,
        robot_y=4.6,
        robot_yaw=-1.17,
        ts_ms=1_700_000_000_000,
    )

    def test_pose_only_frame_has_no_png_key(self):
        # Лёгкий кадр — тот, у которого поля png просто нет: клиент по его
        # отсутствию понимает, что текстуру пересобирать не надо.
        out = msgpack.unpackb(encode_map_2d(**self.BASE), raw=False)
        assert "png" not in out
        assert len(encode_map_2d(**self.BASE)) < 200

    def test_full_frame_carries_png_bytes(self):
        png = grid_to_png([-1] * 4, width=2, height=2)
        out = msgpack.unpackb(encode_map_2d(**self.BASE, png=png), raw=False)
        assert out["png"] == png

    def test_geometry_round_trips(self):
        out = msgpack.unpackb(encode_map_2d(**self.BASE), raw=False)
        assert out["width"] == 958
        assert out["height"] == 744
        assert out["resolution"] == pytest.approx(0.05)
        assert out["origin_x"] == pytest.approx(5.25)
        assert out["origin_y"] == pytest.approx(-25.25)

    def test_missing_pose_stays_null_not_zero(self):
        # «Позы нет» обязано отличаться от «робот в начале карты»: во втором
        # случае клиент положил бы карту не туда, вместо того чтобы скрыть.
        args = {**self.BASE, "robot_x": None, "robot_y": None, "robot_yaw": None}
        out = msgpack.unpackb(encode_map_2d(**args), raw=False)
        assert out["robot_x"] is None
        assert out["robot_y"] is None
        assert out["robot_yaw"] is None
