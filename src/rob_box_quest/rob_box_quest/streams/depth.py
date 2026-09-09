"""compressedDepth payload: 16-битный PNG (мм) → цветной JPEG.

Источник истины: продолжение диагностики #2138.B (2026-09-08).

Контекст. ``image_transport compressedDepth`` для формата ``16UC1`` кладёт
сырые миллиметры глубины в 16-битный grayscale PNG (см.
``_strip_compressed_depth_header`` в ``quest_node.py`` — там же raw-пруф
структуры ConfigHeader + PNG). Раньше эти PNG-байты форвардились клиенту
как есть (после отрезания ConfigHeader). Это чинит «пустую» панель, но не
чинит саму жалобу оператора: ``createImageBitmap()`` в браузере не умеет
16 бит на канал и молча схлопывает до 8 бит, беря СТАРШИЙ байт каждого
пикселя. Комната 500–5000 мм: 5000 мм = 0x1388 → старший байт 0x13 = 19 из
255 — то есть почти чёрный кадр без единого пикселя ошибки в консоли.
Итог для оператора неотличим от «панель не работает».

Решение — то же, что уже проверено и работает в
``rob_box_telegram/handlers/commands.py:_depth_compressed_to_jpeg``
(команда ``/photo_depth``): нормализовать ВАЛИДНЫЕ (ненулевые — 0 в
16UC1 depth значит «нет данных», не «глубина 0») пиксели по 2/98
перцентилям в 0..255 и наложить псевдо-цвет (синий = близко, красный =
далеко), отдать JPEG.

Почему не общий helper с rob_box_telegram, а отдельная реализация здесь.
``rob_box_quest`` и ``rob_box_telegram`` — разные ROS-пакеты в разных
контейнерах (``rob-box-quest`` / ``rob-box-telegram``), с раздельными
Docker-образами и раздельно объявляемыми рантайм-зависимостями (numpy/cv2
тут не идут через rosdep/setup.py — см. комментарий в ``camera.py`` и
``occupancy.py``, они просто есть в образе). Общий пакет ради одной
функции — лишняя связка двух независимо развёртываемых образов ради ~40
строк, которые почти никогда не меняются (ADR-0013, маленькие
независимые PR). Алгоритм переиспользуется (2/98 перцентиль + линейная
пороговая карта цвета), КОД — нет: ``rob_box_telegram`` идёт через PIL
(``Image.open`` + ``np.array``), а весь остальной ``rob_box_quest``
(``camera.py``, ``occupancy.py``) уже стоит на cv2 (``cv2.imdecode`` /
``cv2.imencode``) и PIL как зависимость тут вообще не используется —
проще и последовательнее с соседними файлами написать те же 20 строк на
cv2, чем тянуть Pillow ради одной функции.

Downscale. 1280×720 @ 5 Гц на Raspberry Pi (Vision Pi) — бюджет 200 мс/
кадр. Реальный замер на роботе (``timeit``, 20 итераций, настоящий кадр
362202 байт PNG) — см. commit message / отчёт задачи: полный размер
укладывается с большим запасом, поэтому даунскейл до 640×360 сделан
НЕ ради вписаться в бюджет CPU, а чтобы урезать WS-трафик (JPEG кадра
глубины оператору не нужен в 720p — контуры препятствий на панели
глубины читаются и в половинном разрешении). Порог настраиваемый через
аргумент ``max_width``.
"""

from __future__ import annotations

from typing import Optional

# JPEG quality — тот же баланс bandwidth/качество, что camera.py (75) для
# цветных камер; глубина хуже сжимается блочно (плавные градиенты), но
# артефакты на псевдо-colormap менее заметны, чем на реальном фото.
DEFAULT_JPEG_QUALITY = 80

# Даунскейл по ширине для WS-трафика (см. докстринг модуля). None — без
# даунскейла (используется в тестах, где важна геометрия исходного PNG).
DEFAULT_MAX_WIDTH = 640


def depth_compressed_to_jpeg(
    png_bytes: bytes,
    *,
    quality: int = DEFAULT_JPEG_QUALITY,
    max_width: Optional[int] = DEFAULT_MAX_WIDTH,
) -> bytes:
    """16-битный grayscale PNG (мм) → цветной JPEG (синий=близко, красный=далеко).

    Args:
        png_bytes: PNG-байты БЕЗ ConfigHeader compressedDepth (см.
            ``quest_node._strip_compressed_depth_header`` — заголовок
            отрезается до вызова этой функции).
        quality: JPEG quality (1-100).
        max_width: если кадр шире — уменьшить пропорционально (см.
            докстринг модуля, "Downscale"). ``None`` — не трогать размер.

    Returns:
        JPEG bytes, готовые к ``bridge.publish_frame``.

    Raises:
        ImportError: нет numpy/cv2 (должны быть в Docker-образе — тот же
            путь, что у camera.py/occupancy.py).
        ValueError: cv2 не смог декодировать PNG.
    """
    import cv2  # type: ignore[import-not-found]
    import numpy as np  # type: ignore[import-not-found]

    # IMREAD_UNCHANGED — иначе cv2 по умолчанию свернёт 16 бит → 8 бит
    # сам, тем же способом, что и браузерный createImageBitmap (тот самый
    # баг, который эта функция чинит).
    arr = cv2.imdecode(np.frombuffer(png_bytes, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if arr is None:
        raise ValueError(f"cv2 не смог декодировать PNG (len={len(png_bytes)})")
    if arr.ndim != 2:
        raise ValueError(f"ожидался grayscale (2D) массив, получено shape={arr.shape}")

    depth = arr.astype(np.float32)
    # 0 в 16UC1 depth = «нет данных» (не «глубина 0 мм») — исключаем из
    # статистики нормализации, иначе дыры без данных перетягивают
    # перцентили и валидные пиксели теряют контраст.
    valid = arr > 0
    if valid.any():
        lo = float(np.percentile(depth[valid], 2))
        hi = float(np.percentile(depth[valid], 98))
    else:
        lo, hi = 0.0, 1.0
    span = max(hi - lo, 1e-6)
    norm = np.clip((depth - lo) / span * 255.0, 0.0, 255.0).astype(np.uint8)
    # Невалидные пиксели красим в 0 (край colormap'а — тёмно-синий), чтобы
    # «нет данных» не выглядело случайным цветом с середины шкалы.
    norm[~valid] = 0

    # JET: низкое значение (близко, после нормализации) → синий; высокое
    # (далеко) → красный. Тот же порядок цвета, что и в
    # rob_box_telegram._depth_compressed_to_jpeg.
    colored = cv2.applyColorMap(norm, cv2.COLORMAP_JET)

    if max_width is not None and colored.shape[1] > max_width:
        scale = max_width / colored.shape[1]
        new_w = max_width
        new_h = max(1, int(round(colored.shape[0] * scale)))
        colored = cv2.resize(colored, (new_w, new_h), interpolation=cv2.INTER_AREA)

    ok, buf = cv2.imencode(".jpg", colored, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError("cv2.imencode('.jpg') failed for depth frame")
    return bytes(buf)


def has_depth_deps() -> bool:
    """Runtime-проверка: numpy + cv2 доступны (тот же паттерн, что camera.py)."""
    try:
        import cv2  # noqa: F401
        import numpy  # noqa: F401

        return True
    except ImportError:
        return False
