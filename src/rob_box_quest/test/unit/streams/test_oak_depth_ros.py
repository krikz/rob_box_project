"""Unit-тесты регрессии #2138.B — camera_oak_depth пустая панель.

Кадры глубины OAK-D не доезжали до UI Quest потому, что
``camera_oak_depth`` был объявлен как ``StreamKind.CAMERA_DIRECT`` с
``source="oak:depth"`` — то есть ``QuestBridge`` пытался сам открыть
OAK через depthai SDK. В образе ``rob-box-quest`` depthai нет (он
только в ``camera_oak_color``, и то как stub). В итоге capture-поток
на старте логировал «camera camera_oak_depth unavailable — thread
exits» и панель была пустой.

Тот же путь, что у ``camera_ceiling`` (см. registry.py комментарий):
источник правды для кадров — ``oak-d`` ROS-контейнер, который уже
публикует depth в виде JPEG/PNG. Берём кадры из готового топика,
никакого depthai в ``rob-box-quest``.

Тесты ниже фиксируют НОВЫЙ контракт:

* ``registry.py``: ``camera_oak_depth`` → ``ROS_TOPIC``,
  ``source = /camera/camera/depth/image_rect_raw/compressedDepth``
  (тот же ``compressedDepth``, что использует ``telegram_node`` —
  ``docs/analysis/nodes-current-state.md`` и
  ``src/rob_box_telegram/.../telegram_node.py``).
* ``quest_node.py``: ROS-подписка на этот топик форвардит
  ``bytes(msg.data)`` в ``bridge.publish_frame("camera_oak_depth", ...)``,
  как у ``_on_ceiling_image``. ``OakDepthaiSource`` для depthai
  НЕ стартует (как и для camera_ceiling — см. комментарий в коде).
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from rob_box_quest.streams.registry import (
    STREAM_CATALOG,
    StreamKind,
    get_stream,
)

np = pytest.importorskip("numpy", reason="depth_compressed_to_jpeg требует numpy (только в Docker image)")
cv2 = pytest.importorskip("cv2", reason="depth_compressed_to_jpeg требует cv2 (только в Docker image)")

from rob_box_quest.streams.depth import depth_compressed_to_jpeg  # noqa: E402


def _make_16bit_depth_png(
    values: "np.ndarray",
) -> bytes:
    """np.uint16 (h, w) массив миллиметров → PNG bytes (как публикует OAK-D).

    Тот же путь, что использует ``_strip_compressed_depth_header`` на
    входе: голый PNG БЕЗ 12-байтного ConfigHeader compressedDepth —
    ``depth_compressed_to_jpeg`` его уже не видит, заголовок снимается
    раньше в ``quest_node._on_depth_image``.
    """
    ok, buf = cv2.imencode(".png", values.astype(np.uint16))
    assert ok, "тестовая обвязка не смогла закодировать синтетический PNG"
    return bytes(buf)


# --- Реестр: camera_oak_depth → ROS_TOPIC ----------------------------------


class TestRegistryCameraOakDepth:
    """camera_oak_depth теперь ROS-стрим, не CAMERA_DIRECT.

    Главный регресс #2138.B: был CAMERA_DIRECT → depthai-путь →
    capture-поток не стартовал → панель глубины пустая. Фикс —
    ROS_TOPIC как у camera_ceiling.
    """

    def test_camera_oak_depth_kind_is_ros_topic_not_camera_direct(self):
        """Ключевой инвариант #2138.B: kind == ROS_TOPIC.

        Если кто-то откатит обратно на CAMERA_DIRECT — панель снова
        станет пустой на роботе, потому что depthai в образе нет.
        """
        spec = STREAM_CATALOG["camera_oak_depth"]
        assert spec.kind == StreamKind.ROS_TOPIC, (
            "Регресс #2138.B: camera_oak_depth снова CAMERA_DIRECT → "
            "QuestBridge попытается открыть depthai, capture-поток не "
            "стартанёт, панель глубины останется пустой."
        )

    def test_camera_oak_depth_source_is_oak_d_ros_topic(self):
        """Источник — ROS-топик от oak-d, как и в telegram_node / docs.

        Проект уже использует /camera/camera/depth/image_rect_raw/compressedDepth
        (см. src/rob_box_telegram/.../telegram_node.py:101). Соглашение:
        тот же топик и в camera_oak_depth — единый источник кадров
        глубины в системе.
        """
        spec = STREAM_CATALOG["camera_oak_depth"]
        assert spec.source == (
            "/camera/camera/depth/image_rect_raw/compressedDepth"
        ), (
            "camera_oak_depth должен брать кадры из топика oak-d "
            "(compressedDepth — то же, что в telegram_node). "
            "Если это не так — клиент Quest может ждать формат JPEG, "
            "которого depthai-stream не отдаёт."
        )

    def test_camera_oak_depth_topic_id_unchanged(self):
        """topic_id 0x1004 не меняется — контракт wire-протокола с клиентом.

        Клиент Quest уже подписан на этот id для панели глубины
        (см. webxr_client/src/scene/captain_bridge.ts:66). Смена id
        сломает панель.
        """
        from rob_box_quest.streams.registry import topic_id_for

        assert topic_id_for("camera_oak_depth") == 0x1004


# --- QuestBridge: forwarder + CameraProvider cleanup ------------------------


class TestQuestNodeDepthForwarder:
    """Source-level проверки структуры quest_node.py.

    Тесты парсят AST — это работает на любом окружении, без
    audio_common_msgs / rclpy. Регрессия #2138.B: capture-поток для
    depthai больше не стартует, есть ROS-подписка на compressedDepth.
    """

    def _quest_node_path(self) -> Path:
        repo_root = Path(__file__).resolve().parents[5]
        return (
            repo_root
            / "src"
            / "rob_box_quest"
            / "rob_box_quest"
            / "quest_node.py"
        )

    def test_oak_depth_not_started_via_camera_provider(self):
        """``CameraProvider(cameras=[..., ("camera_oak_depth", "oak:depth", ...)])``
        больше нет — depthai в образе ``rob-box-quest`` отсутствует,
        capture-поток падает на старте, в логе «thread exits».

        Допустим только ``camera_oak_color`` (он и раньше был тут как
        stub — ``OakDepthaiSource.open()`` возвращает False, и поток
        тихо завершается; см. комментарий в provider.py:108).
        """
        source = self._quest_node_path().read_text(encoding="utf-8")
        tree = ast.parse(source)

        # Ищем список ``cameras = [...]``, который инициализирует
        # ``CameraProvider``. Достаём строковые литералы внутри.
        camera_oak_depth_in_provider = False
        for node in ast.walk(tree):
            if not isinstance(node, ast.Assign):
                continue
            # Ищем присваивание ``cameras = [...]``.
            value = node.value
            if not isinstance(value, ast.List):
                continue
            for elt in value.elts:
                if not isinstance(elt, ast.Tuple):
                    continue
                # Кортеж из 3 элементов: (ui_name, source_id, fps).
                # Первый — строка-литерал.
                if len(elt.elts) >= 2:
                    first = elt.elts[0]
                    if (
                        isinstance(first, ast.Constant)
                        and first.value == "camera_oak_depth"
                    ):
                        camera_oak_depth_in_provider = True

        assert not camera_oak_depth_in_provider, (
            "Регресс #2138.B: camera_oak_depth снова в списке камер "
            "CameraProvider → depthai.capture_loop попытается открыть "
            "OAK depth, depthai в образе rob-box-quest нет, поток не "
            "стартанёт, панель глубины останется пустой."
        )

    def test_camera_oak_color_still_in_provider_for_now(self):
        """camera_oak_color пока остаётся в CameraProvider (depthai stub).

        PR #2138.B чинит depth, color — отдельная история. Этот тест
        фиксирует «НЕ удалять в этом PR» — color-фикс требует
        отдельного согласования (правило маленьких PR, ADR-0013).
        """
        source = self._quest_node_path().read_text(encoding="utf-8")
        tree = ast.parse(source)

        found = False
        for node in ast.walk(tree):
            if not isinstance(node, ast.Assign):
                continue
            if not isinstance(node.value, ast.List):
                continue
            for elt in node.value.elts:
                if not isinstance(elt, ast.Tuple):
                    continue
                if len(elt.elts) >= 1:
                    first = elt.elts[0]
                    if (
                        isinstance(first, ast.Constant)
                        and first.value == "camera_oak_color"
                    ):
                        found = True

        assert found, (
            "camera_oak_color удалён из CameraProvider в этом PR — это "
            "нарушение правила маленьких PR (ADR-0013). Для color — "
            "отдельная карточка + согласование. Откатите удаление."
        )

    def test_depth_subscription_uses_compressed_depth_topic(self):
        """В quest_node.py должна быть ROS-подписка на compressedDepth.

        Проверяем на уровне AST, чтобы тест не зависел от
        audio_common_msgs. Ищем ``create_subscription(...,
        "/camera/camera/depth/image_rect_raw/compressedDepth", ...)``.
        """
        source = self._quest_node_path().read_text(encoding="utf-8")
        tree = ast.parse(source)

        # Ищем вызовы create_subscription с целевым топиком.
        target_topic = (
            "/camera/camera/depth/image_rect_raw/compressedDepth"
        )
        # Возможны варианты: голый строковый литерал или параметр ROS.
        # На роботе это параметр depth_topic, но в текущей реализации —
        # жёсткий литерал, как у _camera_ceiling_sub. Ищем литерал.
        found_literal = False
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            func = node.func
            if not (
                isinstance(func, ast.Attribute)
                and func.attr == "create_subscription"
            ):
                continue
            for arg in node.args:
                if (
                    isinstance(arg, ast.Constant)
                    and arg.value == target_topic
                ):
                    found_literal = True

        assert found_literal, (
            "В quest_node.py нет ROS-подписки на "
            f"{target_topic}. Кадры глубины не форвардятся в WS, "
            "панель глубины в шлеме остаётся пустой."
        )


# --- get_stream: smoke для реестра -----------------------------------------


class TestRegistrySmoke:
    def test_get_stream_oak_depth_returns_ros_spec(self):
        spec = get_stream("camera_oak_depth")
        assert spec is not None
        assert spec.kind == StreamKind.ROS_TOPIC
        assert spec.source == (
            "/camera/camera/depth/image_rect_raw/compressedDepth"
        )

    def test_camera_oak_depth_and_ceiling_share_same_kind(self):
        """depth и ceiling — оба ROS_TOPIC, единый паттерн форвардинга."""
        depth_kind = STREAM_CATALOG["camera_oak_depth"].kind
        ceiling_kind = STREAM_CATALOG["camera_ceiling"].kind
        assert depth_kind == ceiling_kind == StreamKind.ROS_TOPIC


# --- QuestNode._on_* handler для depth ------------------------------------


class TestQuestNodeDepthHandler:
    """В QuestNode должен быть обработчик, зеркальный _on_ceiling_image."""

    def test_on_depth_image_handler_exists_and_forwards_to_bridge(self):
        """Источник: AST-парсинг. Ищем метод ``_on_depth_image``, чьё
        тело содержит вызов ``bridge.publish_frame("camera_oak_depth", ...)``.
        """
        repo_root = Path(__file__).resolve().parents[5]
        quest_node_path = (
            repo_root
            / "src"
            / "rob_box_quest"
            / "rob_box_quest"
            / "quest_node.py"
        )
        tree = ast.parse(quest_node_path.read_text(encoding="utf-8"))

        # Найти класс QuestNode → метод _on_depth_image.
        handler_found = False
        forwards_correctly = False
        for node in ast.walk(tree):
            if not isinstance(node, ast.ClassDef):
                continue
            if node.name != "QuestNode":
                continue
            for item in node.body:
                if (
                    not isinstance(item, ast.FunctionDef)
                    or item.name != "_on_depth_image"
                ):
                    continue
                handler_found = True
                # Ищем вызов ``self.bridge.publish_frame(...)`` с
                # первым аргументом строкой "camera_oak_depth".
                for sub in ast.walk(item):
                    if not isinstance(sub, ast.Call):
                        continue
                    func = sub.func
                    if not (
                        isinstance(func, ast.Attribute)
                        and func.attr == "publish_frame"
                    ):
                        continue
                    # Первый позиционный аргумент.
                    if sub.args:
                        first = sub.args[0]
                        if (
                            isinstance(first, ast.Constant)
                            and first.value == "camera_oak_depth"
                        ):
                            forwards_correctly = True

        assert handler_found, (
            "QuestNode._on_depth_image не найден — нужен обработчик ROS "
            "CompressedImage → bridge.publish_frame(\"camera_oak_depth\", ...)."
        )
        assert forwards_correctly, (
            "_on_depth_image найден, но НЕ форвардит в "
            "bridge.publish_frame(\"camera_oak_depth\", bytes(msg.data)) "
            "по образцу _on_ceiling_image."
        )

    def test_on_depth_image_calls_depth_compressed_to_jpeg(self):
        """Регресс продолжения #2138.B (2026-09-08): голый 16-битный PNG

        форвардить клиенту нельзя — ``createImageBitmap()`` схлопывает 16
        бит до 8, взяв старший байт, и комната 500-5000 мм превращается в
        почти чёрный кадр (та же жалоба оператора «панель не работает»,
        просто без ошибки в консоли). ``_on_depth_image`` обязан звать
        ``depth_compressed_to_jpeg`` перед ``bridge.publish_frame``.
        """
        repo_root = Path(__file__).resolve().parents[5]
        quest_node_path = (
            repo_root / "src" / "rob_box_quest" / "rob_box_quest" / "quest_node.py"
        )
        source = quest_node_path.read_text(encoding="utf-8")
        tree = ast.parse(source)

        calls_depth_encoder = False
        for node in ast.walk(tree):
            if not isinstance(node, ast.ClassDef) or node.name != "QuestNode":
                continue
            for item in node.body:
                if not isinstance(item, ast.FunctionDef) or item.name != "_on_depth_image":
                    continue
                for sub in ast.walk(item):
                    if isinstance(sub, ast.Call) and isinstance(sub.func, ast.Name):
                        if sub.func.id == "depth_compressed_to_jpeg":
                            calls_depth_encoder = True

        assert calls_depth_encoder, (
            "_on_depth_image не вызывает depth_compressed_to_jpeg — "
            "16-битный PNG уйдёт клиенту как есть, createImageBitmap() "
            "схлопнет 16 бит до 8 (старший байт), панель глубины станет "
            "почти чёрной вместо пустой — та же жалоба оператора."
        )


# --- streams.depth.depth_compressed_to_jpeg (16-бит PNG → цветной JPEG) -----


class TestDepthCompressedToJpeg:
    """Синтетические 16-битные PNG, без ROS-зависимостей (только cv2/numpy).

    compressedDepth для формата 16UC1 — это 16-битный grayscale PNG в
    миллиметрах (не готовая к показу картинка). Если форвардить его как
    есть, браузерный ``createImageBitmap()`` схлопывает 16 бит → 8,
    беря старший байт: комната 500-5000 мм превращается в почти чёрный
    кадр (0x1388 → 0x13 = 19/255) — визуально та же «пустая панель»,
    просто без ошибки в логах. ``depth_compressed_to_jpeg`` должен это
    предотвращать: нормализовать по перцентилям валидных пикселей и
    наложить псевдо-colormap, а не просто урезать биты.
    """

    def test_returns_valid_jpeg_bytes(self):
        depth_mm = np.full((16, 16), 2000, dtype=np.uint16)
        png = _make_16bit_depth_png(depth_mm)

        jpeg = depth_compressed_to_jpeg(png, max_width=None)

        assert isinstance(jpeg, bytes)
        assert jpeg[:2] == b"\xff\xd8", "JPEG должен начинаться с SOI-маркера 0xFFD8"
        decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        assert decoded is not None, "результат должен сам быть декодируемым JPEG"
        assert decoded.shape[:2] == (16, 16)

    def test_near_pixels_are_bluer_than_far_pixels(self):
        """Синий=близко, красный=далеко (JET colormap, тот же порядок,

        что в rob_box_telegram._depth_compressed_to_jpeg /photo_depth).
        Левая половина кадра — близко (500 мм), правая — далеко (5000 мм).
        """
        depth_mm = np.zeros((8, 16), dtype=np.uint16)
        depth_mm[:, :8] = 500
        depth_mm[:, 8:] = 5000
        png = _make_16bit_depth_png(depth_mm)

        jpeg = depth_compressed_to_jpeg(png, max_width=None)
        decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)  # BGR

        near_px = decoded[4, 2].astype(int)  # (B, G, R)
        far_px = decoded[4, 13].astype(int)

        assert near_px[0] > far_px[0], (
            f"ближний пиксель должен быть синее дальнего: near={tuple(near_px)} "
            f"far={tuple(far_px)}"
        )
        assert far_px[2] > near_px[2], (
            f"дальний пиксель должен быть краснее ближнего: near={tuple(near_px)} "
            f"far={tuple(far_px)}"
        )

    def test_16bit_input_is_not_truncated_to_top_byte(self):
        """Регрессия самого бага: наивное усечение 16→8 бит (>> 8, как

        делает браузерный createImageBitmap) дало бы у всех пикселей в
        диапазоне 256-5000 мм один и тот же старший байт диапазона —
        почти чёрный кадр без контраста. depth_compressed_to_jpeg обязан
        различать 500 мм и 4500 мм по яркости/цвету результата.
        """
        depth_mm = np.zeros((8, 16), dtype=np.uint16)
        depth_mm[:, :8] = 500
        depth_mm[:, 8:] = 4500
        png = _make_16bit_depth_png(depth_mm)

        jpeg = depth_compressed_to_jpeg(png, max_width=None)
        decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)

        near_px = decoded[4, 2].astype(int)
        far_px = decoded[4, 13].astype(int)
        assert tuple(near_px) != tuple(far_px), (
            "500 мм и 4500 мм дали одинаковый цвет — похоже на наивное "
            ">> 8 усечение (тот самый баг), а не перцентильную нормализацию"
        )

    def test_zero_depth_pixels_are_excluded_from_normalization(self):
        """0 в 16UC1 depth = «нет данных», а не «глубина 0 мм». Если 0

        попадёт в перцентильную статистику — валидные пиксели потеряют
        контраст (обширные дыры без данных перетянут перцентили).
        """
        depth_mm = np.zeros((10, 10), dtype=np.uint16)
        depth_mm[:, :] = 0  # почти всё - "нет данных"
        depth_mm[5, 5] = 1000
        depth_mm[5, 6] = 3000
        png = _make_16bit_depth_png(depth_mm)

        jpeg = depth_compressed_to_jpeg(png, max_width=None)
        decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)

        near_px = decoded[5, 5].astype(int)
        far_px = decoded[5, 6].astype(int)
        assert tuple(near_px) != tuple(far_px), (
            "два единственных валидных пикселя (1000мм, 3000мм) получили "
            "одинаковый цвет — статистика нормализации испорчена нулями-дырами"
        )

    def test_all_invalid_pixels_does_not_crash(self):
        """Кадр целиком без данных (робот смотрит в никуда) не должен ронять

        обработчик — деление на диапазон 0 должно быть защищено.
        """
        depth_mm = np.zeros((8, 8), dtype=np.uint16)
        png = _make_16bit_depth_png(depth_mm)

        jpeg = depth_compressed_to_jpeg(png, max_width=None)
        assert jpeg[:2] == b"\xff\xd8"

    def test_downscales_to_max_width(self):
        """max_width обрезает ширину пропорционально — см. обоснование

        даунскейла (WS-трафик, не CPU) в streams/depth.py докстринге.
        """
        depth_mm = np.full((720, 1280), 2000, dtype=np.uint16)
        png = _make_16bit_depth_png(depth_mm)

        jpeg = depth_compressed_to_jpeg(png, max_width=640)
        decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)

        assert decoded.shape[1] == 640
        assert decoded.shape[0] == 360  # пропорционально: 720 * (640/1280)

    def test_max_width_none_keeps_original_size(self):
        depth_mm = np.full((720, 1280), 2000, dtype=np.uint16)
        png = _make_16bit_depth_png(depth_mm)

        jpeg = depth_compressed_to_jpeg(png, max_width=None)
        decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)

        assert decoded.shape[1] == 1280
        assert decoded.shape[0] == 720

    def test_invalid_png_raises_value_error(self):
        with pytest.raises(ValueError):
            depth_compressed_to_jpeg(b"not a png at all")
