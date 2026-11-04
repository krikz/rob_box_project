"""AprilTag на потолочной камере: кормушка, конфиг, compose, калибровка.

Что проверяется (поведение, не тексты):

* кормушка ``ceiling_tag_feed.py`` переводит YUYV (то, что реально шлёт usb_cam —
  замер 04.11.2026: ``encoding: yuv422_yuy2``, 640×480) в mono8 и прореживает
  15 к/с до 5 Гц;
* пути, которые launch-файл и скрипт ждут в контейнере, через тома compose
  указывают на настоящие файлы репо;
* детектор потолка не пишет в ``/detections`` и общий ``/tf`` OAK-D;
* флаг включения по умолчанию выключен;
* camera_info совпадает по размеру с режимом usb_cam, frame_id кадров — оптический
  кадр из URDF;
* ``ceiling_calib.py`` пишет YAML, который читается как camera_info, и (при
  наличии OpenCV) восстанавливает фокус синтетической камеры.

Docker и ROS не нужны.
"""

from __future__ import annotations

import ast
import importlib.util
import re
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
VISION = REPO_ROOT / "docker" / "vision"
COMPOSE = VISION / "docker-compose.yaml"
SCRIPTS = VISION / "scripts" / "ceiling-camera"
LAUNCH = VISION / "config" / "ceiling-camera" / "launch" / "ceiling_apriltag.launch.py"
APRILTAG_CEILING = VISION / "config" / "apriltag" / "ceiling_apriltag.yaml"
APRILTAG_OAK = VISION / "config" / "apriltag" / "apriltag_config.yaml"
CAMERA_PARAMS = VISION / "config" / "ceiling-camera" / "camera_params.yaml"
CAMERA_INFO = VISION / "config" / "ceiling-camera" / "ceiling_camera.yaml"
URDF_GOLDEN = REPO_ROOT / "src" / "rob_box_description" / "test" / "golden" / "rob_box_real.urdf"
ZENOH = REPO_ROOT / "docker" / "common" / "config" / "zenoh_session_config.json5"


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


feed = _load_module("ceiling_tag_feed", SCRIPTS / "ceiling_tag_feed.py")
calib = _load_module("ceiling_calib", SCRIPTS / "ceiling_calib.py")


# ------------------------------------------------------------- кормушка


def _luma(w: int, h: int) -> np.ndarray:
    return (np.arange(w * h, dtype=np.uint32) * 7 % 251).astype(np.uint8).reshape(h, w)


def _yuyv(y: np.ndarray, pad: int = 0) -> bytes:
    h, w = y.shape
    out = np.zeros((h, 2 * w + pad), np.uint8)
    out[:, 0 : 2 * w : 2] = y
    out[:, 1 : 2 * w : 4] = 90  # U
    out[:, 3 : 2 * w : 4] = 200  # V
    return out.tobytes()


def test_yuyv_from_usb_cam_becomes_its_luma():
    y = _luma(640, 480)
    mono = feed.to_mono8("yuv422_yuy2", _yuyv(y), 640, 480, 1280)
    assert mono.shape == (480, 640) and mono.dtype == np.uint8
    assert np.array_equal(mono, y)


def test_yuyv_with_padded_rows():
    y = _luma(6, 4)
    assert np.array_equal(feed.to_mono8("yuv422_yuy2", _yuyv(y, pad=4), 6, 4, 16), y)


def test_uyvy_takes_odd_bytes():
    y = _luma(4, 2)
    raw = np.full((2, 8), 128, np.uint8)
    raw[:, 1::2] = y
    assert np.array_equal(feed.to_mono8("yuv422", raw.tobytes(), 4, 2, 8), y)


def test_mono8_passthrough_and_rgb_to_gray():
    y = _luma(5, 3)
    assert np.array_equal(feed.to_mono8("mono8", y.tobytes(), 5, 3, 5), y)
    rgb = np.zeros((1, 3, 3), np.uint8)
    rgb[0, 0] = (255, 0, 0)
    rgb[0, 1] = (0, 255, 0)
    rgb[0, 2] = (255, 255, 255)
    gray = feed.to_mono8("rgb8", rgb.tobytes(), 3, 1, 9)[0].tolist()
    assert gray[0] == pytest.approx(76, abs=1) and gray[1] == pytest.approx(150, abs=1)
    assert gray[2] == 255
    bgr = rgb[..., ::-1].copy()
    assert feed.to_mono8("bgr8", bgr.tobytes(), 3, 1, 9)[0].tolist() == gray


def test_unknown_encoding_and_short_frame_are_refused():
    with pytest.raises(feed.UnsupportedEncoding):
        feed.to_mono8("16UC1", bytes(8), 2, 2, 4)
    with pytest.raises(ValueError):
        feed.to_mono8("yuv422_yuy2", bytes(10), 4, 2, 8)


def test_rate_gate_thins_15fps_to_5hz():
    gate = feed.RateGate(5.0)
    stamps = [i / 15.0 for i in range(150)]  # 10 с при 15 к/с
    admitted = [t for t in stamps if gate.admit(t)]
    assert len(admitted) == 50
    assert all(b - a == pytest.approx(0.2, abs=1e-6) for a, b in zip(admitted, admitted[1:]))


def test_rate_gate_restarts_when_time_goes_back():
    gate = feed.RateGate(5.0)
    assert gate.admit(100.0)
    assert not gate.admit(100.1)
    assert gate.admit(3.0)  # камеру перезапустили / сим сбросил часы


# ------------------------------------------------------------- compose


def _service() -> dict:
    return yaml.safe_load(COMPOSE.read_text(encoding="utf-8"))["services"]["ceiling-camera"]


def _env(service: dict) -> dict:
    env = service["environment"]
    if isinstance(env, dict):
        return env
    return dict(item.split("=", 1) for item in env)


def _container_to_host(container_path: str) -> Path:
    """Путь в контейнере ceiling-camera → файл репо по томам compose."""
    best = None
    for vol in _service()["volumes"]:
        src, dst = vol.split(":")[:2]
        if container_path == dst or container_path.startswith(dst.rstrip("/") + "/"):
            if best is None or len(dst) > len(best[1]):
                best = (src, dst)
    assert best, f"{container_path} не смонтирован в ceiling-camera"
    src, dst = best
    rel = container_path[len(dst) :].lstrip("/")
    return (VISION / src / rel).resolve()


def test_apriltag_flag_is_off_by_default():
    raw = _env(_service())["CEILING_APRILTAG_ENABLED"]
    m = re.fullmatch(r"\$\{CEILING_APRILTAG_ENABLED:-(\w+)\}", raw)
    assert m, raw
    assert m.group(1) == "false"


def test_start_script_launches_detector_only_under_flag():
    text = (SCRIPTS / "start_ceiling_camera.sh").read_text(encoding="utf-8")
    gate = re.search(r'if \[ "\$\{CEILING_APRILTAG_ENABLED:-false\}" = "true" \]; then(.*?)\nfi', text, re.S)
    assert gate, "запуск детектора не под флагом"
    assert "ceiling_apriltag.launch.py" in gate.group(1)
    assert text.count("ceiling_apriltag.launch.py") == gate.group(0).count("ceiling_apriltag.launch.py")
    launch_path = re.search(r"ros2 launch (\S+)", gate.group(1)).group(1)
    assert _container_to_host(launch_path) == LAUNCH.resolve()


# -------------------------------------------------------------- launch


def _launch_nodes() -> list[dict]:
    """Аргументы Node(...) из launch-файла — без импорта launch_ros."""
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"))
    ns = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant):
            ns[node.targets[0].id] = node.value.value

    def ev(n):
        if isinstance(n, ast.JoinedStr):
            return "".join(ns[v.value.id] if isinstance(v, ast.FormattedValue) else v.value for v in n.values)
        if isinstance(n, ast.Name):
            return ns[n.id]
        if isinstance(n, (ast.List, ast.Tuple)):
            return [ev(e) for e in n.elts]
        if isinstance(n, ast.Dict):
            return {ev(k): ev(v) for k, v in zip(n.keys, n.values)}
        return ast.literal_eval(n)

    return [
        {kw.arg: ev(kw.value) for kw in call.keywords}
        for call in ast.walk(tree)
        if isinstance(call, ast.Call) and getattr(call.func, "id", None) == "Node"
    ]


def _apriltag_node() -> dict:
    (node,) = [n for n in _launch_nodes() if n.get("package") == "apriltag_ros"]
    return node


def test_ceiling_detector_does_not_touch_oak_d_topics():
    remap = dict(_apriltag_node()["remappings"])
    assert remap["detections"] != "/detections"
    assert remap["detections"].startswith("/ceiling_camera/")
    # Кадры тегов tag36h11:<id> совпадают с OAK-D — в общий /tf нельзя.
    assert remap["/tf"].startswith("/ceiling_camera/")


def test_detector_reads_what_the_feed_publishes():
    remap = dict(_apriltag_node()["remappings"])
    image = remap["image_rect"]
    # image_transport::CameraSubscriber ищет camera_info рядом с картинкой.
    assert remap["camera_info"] == image.rsplit("/", 1)[0] + "/camera_info"
    (feed_node,) = [n for n in _launch_nodes() if n.get("name") == "ceiling_tag_feed"]
    out_ns = "/ceiling_camera/apriltag"  # дефолт параметра output_ns в кормушке
    src = (SCRIPTS / "ceiling_tag_feed.py").read_text(encoding="utf-8")
    assert f'"output_ns", "{out_ns}"' in src
    assert image == f"{out_ns}/image"
    assert _container_to_host(feed_node["arguments"][0]) == (SCRIPTS / "ceiling_tag_feed.py").resolve()


def test_feed_is_throttled_to_5hz_or_less():
    (feed_node,) = [n for n in _launch_nodes() if n.get("name") == "ceiling_tag_feed"]
    rate = feed_node["parameters"][0]["rate_hz"]
    assert 0 < rate <= 5.0


def test_new_topics_fall_under_droppable_zenoh_qos():
    """Картинка 5 Гц не должна душить транспорт: префикс rt/ceiling_camera/** → drop."""
    text = ZENOH.read_text(encoding="utf-8")
    assert '"rt/ceiling_camera/**"' in text
    for topic in dict(_apriltag_node()["remappings"]).values():
        assert topic.startswith("/ceiling_camera/")


# ------------------------------------------------------- конфиг apriltag


def test_ceiling_apriltag_config_matches_node_and_keeps_load_low():
    node = _apriltag_node()
    (params_path,) = node["parameters"]
    host = _container_to_host(params_path)
    assert host == APRILTAG_CEILING.resolve()
    cfg = yaml.safe_load(host.read_text(encoding="utf-8"))
    params = cfg[node["name"]]["ros__parameters"]
    oak = yaml.safe_load(APRILTAG_OAK.read_text(encoding="utf-8"))["apriltag"]["ros__parameters"]
    assert params["family"] == oak["family"] == "36h11"
    assert params["size"] == pytest.approx(0.119)  # чёрный квадрат, замер Шифу
    assert params["image_transport"] == "raw"  # mono8 из кормушки, без JPEG
    assert params["detector"]["threads"] == 1
    assert params["detector"]["decimate"] >= 2.0


# ------------------------------------------------- camera_info и frame_id


def _usb_cam_params() -> dict:
    return yaml.safe_load(CAMERA_PARAMS.read_text(encoding="utf-8"))["/**"]["ros__parameters"]


def test_camera_info_matches_usb_cam_mode():
    p = _usb_cam_params()
    info = yaml.safe_load(CAMERA_INFO.read_text(encoding="utf-8"))
    assert (info["image_width"], info["image_height"]) == (p["image_width"], p["image_height"])
    k = info["camera_matrix"]["data"]
    pm = info["projection_matrix"]["data"]
    assert 0 < k[2] < info["image_width"] and 0 < k[5] < info["image_height"]
    # apriltag_node берёт fx, fy, cx, cy из P.
    assert (pm[0], pm[5], pm[2], pm[6]) == (k[0], k[4], k[2], k[5])
    assert p["camera_info_url"].startswith("file://")
    assert _container_to_host(p["camera_info_url"][len("file://") :]) == CAMERA_INFO.resolve()


def test_usb_cam_frame_id_is_the_urdf_optical_frame():
    """usb_cam 0.8 читает ``frame_id`` (camera_frame_id игнорирует → default_cam)."""
    p = _usb_cam_params()
    assert "camera_frame_id" not in p
    assert p["frame_id"] == "ceiling_camera_optical_frame"
    assert f'<link name="{p["frame_id"]}"/>' in URDF_GOLDEN.read_text(encoding="utf-8")


def test_dockerfile_installs_detector():
    text = (VISION / "ceiling-camera" / "Dockerfile").read_text(encoding="utf-8")
    assert "ros-humble-apriltag-ros" in text
    assert "python3-numpy" in text


# ------------------------------------------------------------ калибровка


def test_calibration_yaml_is_a_camera_info():
    k = [[1300.0, 0, 321.5], [0, 1298.0, 239.0], [0, 0, 1]]
    text = calib.camera_info_yaml(640, 480, k, [0.1, -0.2, 0.0, 0.0], rms=0.31, views=22)
    info = yaml.safe_load(text)
    assert (info["image_width"], info["image_height"]) == (640, 480)
    assert info["camera_name"] == "ceiling_camera"
    assert info["camera_matrix"]["data"] == pytest.approx([1300, 0, 321.5, 0, 1298, 239, 0, 0, 1])
    assert info["distortion_coefficients"]["data"] == pytest.approx([0.1, -0.2, 0, 0, 0])
    assert info["projection_matrix"]["data"] == pytest.approx([1300, 0, 321.5, 0, 0, 1298, 239, 0, 0, 0, 1, 0])
    assert info["distortion_model"] == "plumb_bob"


def test_pgm_roundtrip(tmp_path):
    img = _luma(7, 3)
    path = tmp_path / "f.pgm"
    calib.write_pgm(path, img)
    raw = path.read_bytes()
    header = b"P5\n7 3\n255\n"
    assert raw.startswith(header)
    assert np.array_equal(np.frombuffer(raw[len(header) :], np.uint8).reshape(3, 7), img)


def test_calibrate_recovers_synthetic_focal(tmp_path):
    """Синтетическая камера f=1300, 640×480 — калибровка возвращает её K."""
    cv2 = pytest.importorskip("cv2")
    cols, rows, sq = 8, 6, 0.025
    k_true = np.array([[1300.0, 0, 320.0], [0, 1300.0, 240.0], [0, 0, 1]])
    obj = calib.board_points(cols, rows, sq)
    # Растр доски: клетка 40 px, поле 1 клетка.
    px = 40
    board = np.full(((rows + 3) * px, (cols + 3) * px), 255, np.uint8)
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:
                board[(r + 1) * px : (r + 2) * px, (c + 1) * px : (c + 2) * px] = 0
    # Внутренний угол (i, j) доски ↔ пиксель растра ((i+2)·px, (j+2)·px).
    src = np.float32(
        [[2 * px, 2 * px], [(cols + 1) * px, 2 * px], [(cols + 1) * px, (rows + 1) * px], [2 * px, (rows + 1) * px]]
    )
    corners_m = np.float32(
        [[0, 0, 0], [(cols - 1) * sq, 0, 0], [(cols - 1) * sq, (rows - 1) * sq, 0], [0, (rows - 1) * sq, 0]]
    )
    rng = np.random.default_rng(4)
    for n in range(18):
        rvec = np.array([rng.uniform(-0.4, 0.4), rng.uniform(-0.4, 0.4), rng.uniform(-0.3, 0.3)])
        tvec = np.array([rng.uniform(-0.12, 0.0), rng.uniform(-0.09, 0.0), rng.uniform(0.9, 1.2)])
        dst, _ = cv2.projectPoints(corners_m, rvec, tvec, k_true, None)
        h = cv2.getPerspectiveTransform(src, dst.reshape(4, 2).astype(np.float32))
        img = cv2.warpPerspective(board, h, (640, 480), borderValue=255)
        calib.write_pgm(tmp_path / f"v{n:02d}.pgm", img)
    files = sorted(tmp_path.glob("*.pgm"))
    w, hgt, k, d, rms, views = calib.calibrate(files, cols, rows, sq)
    assert (w, hgt) == (640, 480)
    assert views >= calib.MIN_VIEWS
    assert k[0][0] == pytest.approx(1300, rel=0.03)
    assert k[1][1] == pytest.approx(1300, rel=0.03)
    assert rms < 1.0
    assert obj.shape == (cols * rows, 3)
