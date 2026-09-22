#!/usr/bin/env python3
"""face_embedding.py — ArcFace-эмбеддинги лиц через Hailo (issue #2599 PR-B).

Почему отдельный модуль, а не расширение ``vision_face_loader.py``:
``vision_face_loader`` решает задачу «где на кадре лицо» (RetinaFace —
детекция + якоря + NMS), а этот модуль решает другую задачу — «какой
вектор соответствует уже вырезанному лицу» (ArcFace — классификатор
эмбеддингов без anchors/NMS/postprocess-геометрии). Смешивать их значило
бы завести в одном файле два независимых декодера с разными входами
(736x1280 letterbox кадра vs 112x112 кроп лица) — то же архитектурное
решение, что развело retinaface и yolov8n в PR-A (issue #2599 PR-A).

Спецификация модели (проверено на живом Vision Pi 22.09.2026, HEF на
диске — ``/opt/rob_box/models/arcface_mobilefacenet.hef``,
sha256 ``c75fc632...7ddc7``, 3505142 байт; ``hailo_platform.HEF`` отдаёт
форму входа/выхода напрямую):
  - вход:  ``arcface_mobilefacenet/input_layer1``, форма (112, 112, 3),
    FormatType.UINT8.
  - выход: ``arcface_mobilefacenet/fc1``, форма (512,), FormatType.UINT8.

**512-dim, а не 128.** ``docker/vision/config/hailo_models.yaml`` и
ADR-0089/ADR-0106/ADR-0123 (§4.1: "несколько 128-dim векторов на
человека") называют 128 — это ошибка в документации, унаследованная,
судя по всему, от другой сборки ArcFace (есть публичные варианты с
128-dim head). Реальный HEF, установленный на роботе, отдаёт 512.
Код ниже размер не хардкодит: ``embedding_dim`` читается из фактической
формы выходного тензора после инициализации, а не выводится из имени
модели. ADR-0123 §4.1 и ``hailo_models.yaml`` нужно поправить отдельной
карточкой — это вне рамок #2599 PR-B (там речь про сам эмбеддер, а не
про ревизию чужих ADR).

**Деквантизация выхода — не деталь, а условие работоспособности.**
Выход ``fc1`` квантован в UINT8 (``qp_scale≈0.0167``, ``qp_zp=144``,
замерено на роботе 22.09.2026). Мы просим HailoRT отдавать его сразу
``FormatType.FLOAT32`` — тогда деквантует он сам, и это основной путь.

Что было, когда этого не делали (живой замер там же): сырой uint8 весь
лежит в ПОЛОЖИТЕЛЬНОМ октанте, поэтому косинус между эмбеддингами двух
совершенно разных картинок выходил **0.999** — при любом пороге
узнавания (ADR-0123 §6) робот считал бы всех одним человеком.
L2-нормализация от этого не спасает: косинус инвариантен к
положительному масштабу, но НЕ к сдвигу на ``zero_point``. С FLOAT32
те же две картинки дают 0.15, а одинаковые — 1.0.

Порядок вызовов важен: ``set_format_type`` обязан стоять ДО
``configure()``. После неё сеть уже сконфигурирована и вызов молча
ничего не меняет — именно так первая версия этого модуля осталась на
uint8, пройдя при этом все юнит-тесты (они мокают железо).

Запасные пути, если ``set_format_type`` недоступен (старый HailoRT):
``(uint8 - zero_point) * scale`` по ``quant_infos`` — ВНИМАНИЕ,
атрибут называется во множественном числе и является списком; поиск
``quant_info`` в единственном числе молча не находит ничего. Если нет и
его — вычитаем середину диапазона (128), что грубо, но выводит вектор
из положительного октанта. Если ArcFace начнёт мазать по порогу,
первое место для проверки — здесь.

Инициализация железа — 1:1 паттерн ``RetinaFaceLoader`` из
``vision_face_loader.py`` (``open_device()`` → ``create_infer_model()``
→ ``configure()`` → ``create_bindings()``): ``open_device()`` из
``hailo_device.py`` — единственный шов, который трогает VDevice
(ADR-0112), lazy-init с кешированием ``_init_failed`` — тот же
paттерн, который ADR-0121 предлагает вынести в общий миксин (на момент
написания ADR-0121 имеет статус Proposed и в ``vision_hailo_loader.py``
ещё не приземлился, поэтому этот модуль копирует паттерн явно, как и
сам ``RetinaFaceLoader`` — когда миксин появится, оба потребителя
переедут на него одним PR, см. ADR-0121 «План реализации»).

Failure policy (ADR-0018 capability-honest): сбой инициализации/run()
HailoRT — исключение из ``embed()``, вызывающий код обязан его поймать
и залогировать (как ``vision_face_node`` уже делает для
``RetinaFaceLoader.infer``). Единственное исключение из "raise всегда" —
**один негодный кроп внутри списка** ``crops``: он не должен ронять
эмбеддинг всех остальных лиц в кадре, поэтому даёт ``None`` на своей
позиции, а не прерывает ``embed()``.

Touchpoints:
- ADR-0112 (шов «Ускоритель», hailo_device.open_device).
- ADR-0121 (миксин ленивой инициализации — Proposed, ещё не внедрён).
- ADR-0123 (режимы приватности; этот модуль эмбеддингов не решает,
  что с ними делать дальше — это зона ``FaceStore``).
- Issue #2599 PR-B (ArcFace-эмбеддинги, лицевой трекер, узнавание).
- rob_box_perception.vision_face_loader (RetinaFaceLoader — источник
  паттерна инициализации и bbox_norm формата).
- rob_box_perception.hailo_device (open_device, шов «Ускоритель»).
- rob_box_perception.gaze (лениво импортирует cv2 внутри функций —
  этот модуль следует той же конвенции).
"""

from __future__ import annotations

import logging
import os
import time
from typing import Any, List, Optional, Tuple

from rob_box_perception.hailo_device import open_device
from rob_box_perception.vision_hailo_loader import (
    _is_recoverable_stream_abort,
    _next_reinit_backoff_sec,
)

_LOG = logging.getLogger(__name__)

#: Путь к HEF по умолчанию (staged на Vision Pi, проверено 22.09.2026).
DEFAULT_ARCFACE_HEF = '/opt/rob_box/models/arcface_mobilefacenet.hef'

#: Сторона входного квадрата ArcFace (112x112x3, HxWxC).
ARCFACE_INPUT_SIZE = 112

#: Эмбеддинг ArcFace-моделей семейства mobilefacenet, если фактическая
#: форма выхода недоступна (использован только как fallback для
#: ``embedding_dim`` до первой успешной инициализации — см. свойство).
_FALLBACK_EMBEDDING_DIM = 512


class ArcFaceEmbedder:
    """ArcFace HEF → 512-dim L2-нормализованный эмбеддинг лица.

    Повторяет lazy-init паттерн ``RetinaFaceLoader``
    (``vision_face_loader.py``): устройство и infer_model создаются не
    в ``__init__``, а при первом обращении к ``embed()``, ошибка
    инициализации кешируется в ``_init_failed`` и re-raise'ится на
    повторные вызовы без повторных попыток открыть железо (issue #2599:
    повторный ``open_device()`` после сбоя — источник дополнительных
    гонок за Hailo, тот же урок, что ADR-0112 зафиксировал для
    RetinaFace).

    Attributes:
        _hef_path: путь к arcface_mobilefacenet.hef.
        _vdevice_id: ID VDevice (см. ``hailo_device.open_device``).
        _embedding_dim: форма выходного тензора после инициализации;
            None до первого успешного ``_ensure_initialized()``.
    """

    def __init__(
        self,
        hef_path: str = DEFAULT_ARCFACE_HEF,
        vdevice_id: int = 0,
    ) -> None:
        self._hef_path = hef_path
        self._vdevice_id = vdevice_id
        self._device: Any = None
        self._vdevice: Any = None
        self._infer_model: Any = None
        self._configured: Any = None
        self._bindings: Any = None
        self._input_name: str = ''
        self._output_name: str = ''
        self._embedding_dim: Optional[int] = None
        self._quant_scale: Optional[float] = None
        self._quant_zero_point: Optional[float] = None
        #: True, если HailoRT отдаёт выход уже в FLOAT32 (деквантует сам).
        self._output_is_float: bool = False
        self._init_failed: Optional[BaseException] = None
        # ---- HAILO_STREAM_ABORT(63) recovery state (issue #2625) ----
        # 1:1 паттерн RealHEFLoader/RetinaFaceLoader — см. docstring
        # vision_hailo_loader.RealHEFLoader._ensure_initialized. Общая
        # детекция/backoff переиспользованы через импорт, reset/retry
        # копируется по месту (тот же аргумент, что и у RetinaFaceLoader:
        # ADR-0121 миксин остаётся Proposed).
        self._recovering_from_abort: bool = False
        self._reinit_backoff_sec: float = 0.0
        self._reinit_not_before: float = 0.0

    # ----------------------------------------------------------------
    # Lazy initialization (паттерн RetinaFaceLoader._ensure_initialized)
    # ----------------------------------------------------------------

    def _ensure_initialized(self) -> None:
        if self._configured is not None:
            return
        if self._init_failed is not None:
            raise self._init_failed
        if self._recovering_from_abort:
            self._retry_after_abort()
            return
        try:
            self._init_locked()
        except BaseException as exc:  # noqa: BLE001 (capability-honest)
            self._init_failed = exc
            raise

    def _retry_after_abort(self) -> None:
        """Reinit после `HAILO_STREAM_ABORT(63)` с backoff (issue #2625).

        1:1 паттерн ``RealHEFLoader._retry_after_abort`` (vision_hailo_loader.py).
        """
        now = time.monotonic()
        if now < self._reinit_not_before:
            remaining = self._reinit_not_before - now
            raise RuntimeError(
                f'HailoRT pipeline recovering from HAILO_STREAM_ABORT(63): '
                f'backoff активен ещё {remaining:.1f}s (issue #2625).'
            )
        try:
            self._init_locked()
        except BaseException as exc:  # noqa: BLE001 (capability-honest)
            self._reinit_backoff_sec = _next_reinit_backoff_sec(
                self._reinit_backoff_sec
            )
            self._reinit_not_before = time.monotonic() + self._reinit_backoff_sec
            _LOG.warning(
                'Переинициализация ArcFace после HAILO_STREAM_ABORT(63) '
                f'снова не удалась: {exc!r}. Следующая попытка через '
                f'{self._reinit_backoff_sec:.1f}s (issue #2625).'
            )
            raise
        _LOG.warning(
            'ArcFace HailoRT пайплайн переподнят после '
            'HAILO_STREAM_ABORT(63) (issue #2625) — реинициализация '
            'прошла успешно.'
        )
        self._recovering_from_abort = False
        self._reinit_backoff_sec = 0.0
        self._reinit_not_before = 0.0

    def _reset_after_stream_abort(self, exc: BaseException) -> None:
        """Сбросить lazy-init состояние после abort'а (issue #2625 п.1).

        1:1 паттерн ``RealHEFLoader._reset_after_stream_abort``. Оставляет
        ``_embedding_dim``/``_quant_scale``/``_quant_zero_point``/
        ``_output_is_float`` как есть — это метаданные формы/квантования
        HEF, они не меняются между переинициализациями того же файла и
        безвредны как "последнее известное" значение до следующего
        успешного ``_init_locked()``.
        """
        _LOG.warning(
            'ArcFace HailoRT pipeline abort обнаружен '
            f'(HAILO_STREAM_ABORT(63) pattern): {exc!r}. Сбрасываю '
            'состояние эмбеддера для переинициализации на следующем '
            'вызове embed() (issue #2625, ADR-0112 §5 п.1).'
        )
        self._configured = None
        self._bindings = None
        self._infer_model = None
        self._init_failed = None
        self._vdevice = None
        self._device = None
        self._recovering_from_abort = True
        self._reinit_backoff_sec = 0.0
        self._reinit_not_before = 0.0

    def _init_locked(self) -> None:
        if not os.path.isfile(self._hef_path):
            raise FileNotFoundError(
                f'HEF не найден: {self._hef_path}. Убедитесь, что '
                f'arcface_mobilefacenet.hef staged на Vision Pi '
                f'(issue #2599 PR-B, docker/vision/config/hailo_models.yaml).'
            )

        # Устройство — только через шов «Ускоритель» (ADR-0112). Прямой
        # VDevice здесь означал бы третьего продюсера, который гоняется
        # за Hailo с RetinaFace и yolov8n в обход планировщика сервиса.
        self._device = open_device(self._vdevice_id)
        self._vdevice = self._device.vdevice

        self._infer_model = self._vdevice.create_infer_model(self._hef_path)
        self._infer_model.set_batch_size(1)

        import numpy as np  # type: ignore[import-not-found]

        self._input_name = self._infer_model.input_names[0]
        self._output_name = self._infer_model.output_names[0]

        # Форматы обязаны быть выставлены ДО configure(): после неё сеть
        # уже сконфигурирована, и set_format_type на неё не влияет —
        # именно так первая версия этого кода молча осталась на uint8.
        #
        # Вход arcface HEF: uint8 NHWC 112x112x3 (проверено HEF.get_input
        # на живом железе 22.09.2026 — arcface_mobilefacenet/input_layer1).
        try:
            from hailo_platform import FormatType  # type: ignore[import-not-found]
            try:
                self._infer_model.input().set_format_type(FormatType.UINT8)
            except AttributeError:
                pass
        except ImportError:
            pass

        # Выход fc1 квантован (UINT8, qp_scale≈0.0167, qp_zp=144).
        # Просим HailoRT отдавать его сразу FLOAT32 — деквантует он сам,
        # и это ЕДИНСТВЕННЫЙ надёжный путь.
        #
        # Почему это важно, а не косметика (замерено на живом Vision Pi
        # 22.09.2026): сырой uint8 весь лежит в положительном октанте,
        # поэтому косинус между эмбеддингами ДВУХ РАЗНЫХ картинок выходил
        # 0.999 — при любом пороге узнавания робот считал бы всех одним
        # человеком. L2-нормализация от этого не спасает: косинус
        # инвариантен к положительному масштабу, но НЕ к сдвигу на
        # zero-point. С FLOAT32-выходом те же две картинки дают 0.15, а
        # одинаковые — 1.0.
        self._output_is_float = False
        try:
            from hailo_platform import FormatType  # type: ignore[import-not-found]

            self._infer_model.output(self._output_name).set_format_type(
                FormatType.FLOAT32
            )
            self._output_is_float = True
        except Exception:  # noqa: BLE001
            # Старый hailo_platform без set_format_type — откатываемся на
            # ручную деквантизацию по quant_infos (см. _dequantize).
            self._output_is_float = False

        # configure() — только после того, как форматы выставлены.
        self._configured = self._infer_model.configure()

        input_shape = list(self._infer_model.input(self._input_name).shape)
        input_buffers = {
            self._input_name: np.empty(input_shape, dtype=np.uint8)
        }

        output_shape = list(self._infer_model.output(self._output_name).shape)
        output_buffers = {
            self._output_name: np.empty(
                output_shape,
                dtype=(np.float32 if self._output_is_float else np.uint8),
            )
        }

        # embedding_dim — из фактической формы выхода, а не из константы:
        # это ровно то место, где допущение "128" в конфиге/ADR разошлось
        # бы с реальностью, если бы мы его хардкодили (см. docstring).
        self._embedding_dim = int(np.prod(output_shape))

        self._bindings = self._configured.create_bindings(
            input_buffers=input_buffers,
            output_buffers=output_buffers,
        )
        if self._device.must_activate:
            self._configured.activate()

        self._quant_scale, self._quant_zero_point = self._read_quant_info()

    def _read_quant_info(self) -> Tuple[Optional[float], Optional[float]]:
        """Достать (scale, zero_point) квантования выхода, если HailoRT их отдаёт.

        Разные версии hailo_platform выставляют это по-разному
        (``vstream_info().quant_info``, ``get_output_vstream_infos()``,
        либо не выставляют вовсе). Пробуем самые вероятные пути и молча
        возвращаем ``(None, None)``, если ни один не сработал — тогда
        ``_dequantize`` откатывается на raw-uint8 путь (см. docstring).
        """
        try:
            info = self._infer_model.output(self._output_name)
            # ВНИМАНИЕ: атрибут называется ``quant_infos`` (СПИСОК), а не
            # ``quant_info``. Первая версия искала единственное число,
            # молча не находила и уходила в сырой uint8 — именно так
            # родился косинус 0.999 между разными лицами (см. выше).
            quant = getattr(info, 'quant_infos', None)
            if quant is None:
                quant = getattr(info, 'quant_info', None)
            if isinstance(quant, (list, tuple)):
                quant = quant[0] if quant else None
            if quant is not None:
                scale = getattr(quant, 'qp_scale', None)
                zero_point = getattr(quant, 'qp_zp', None)
                if scale is not None and zero_point is not None:
                    return float(scale), float(zero_point)
        except Exception:  # noqa: BLE001
            pass
        return None, None

    # ----------------------------------------------------------------
    # Public API
    # ----------------------------------------------------------------

    @property
    def embedding_dim(self) -> int:
        """Размерность эмбеддинга. 512 после инициализации реального HEF.

        До первой инициализации возвращает fallback-константу — свойство
        может понадобиться до первого ``embed()`` (например, для
        предвыделения буферов в ``FaceStore``), а лениво инициализировать
        железо только ради числа — лишний повод схватить
        ``HAILO_OUT_OF_PHYSICAL_DEVICES`` раньше времени.
        """
        if self._embedding_dim is not None:
            return self._embedding_dim
        return _FALLBACK_EMBEDDING_DIM

    def embed(self, crops: List[Any]) -> List[Optional[Any]]:
        """Посчитать L2-нормализованные эмбеддинги для списка кропов лиц.

        Args:
            crops: список RGB uint8 ndarray произвольного размера (ресайз
                до 112x112 — внутри). Пустой список — валидный вход.

        Returns:
            Список той же длины, что ``crops``. Каждый элемент — float32
            ndarray формы (embedding_dim,), L2-нормализованный, либо
            ``None`` для кропа, который не удалось обработать (пустой /
            битой формы / cv2 недоступен). Один плохой кроп не должен
            стоить эмбеддингов остальным лицам в кадре — поэтому такой
            кроп даёт ``None``, а не исключение (ADR-0018: raise —
            только для отказа устройства, а не пользовательских данных).

        Raises:
            FileNotFoundError / ImportError / RuntimeError: инициализация
            или run() HailoRT упали — capability-honest, вызывающий код
            обязан поймать и залогировать (как ``vision_face_node`` уже
            делает для ``RetinaFaceLoader.infer``).
        """
        if not crops:
            return []

        self._ensure_initialized()

        import numpy as np  # type: ignore[import-not-found]

        results: List[Optional[Any]] = []
        for crop in crops:
            prepared = prepare_arcface_input(crop)
            if prepared is None:
                results.append(None)
                continue

            assert self._bindings is not None
            assert self._configured is not None

            input_tensor = np.expand_dims(
                np.ascontiguousarray(prepared), axis=0
            )
            try:
                self._bindings.input(self._input_name).set_buffer(input_tensor)
                self._configured.run([self._bindings], timeout=1000)
            except Exception as exc:  # noqa: BLE001 (capability-honest)
                if _is_recoverable_stream_abort(exc):
                    self._reset_after_stream_abort(exc)
                raise RuntimeError(f'HailoRT run() failed: {exc!r}') from exc

            raw = self._bindings.output(self._output_name).get_buffer()
            vec = self._dequantize(np.asarray(raw).reshape(-1))
            results.append(_l2_normalize(vec))

        return results

    def _dequantize(self, raw_uint8: Any) -> Any:
        """uint8 vstream → float32 вектор, честно/приближённо (см. docstring).

        Путь A (честный): ``(raw - zero_point) * scale`` — восстанавливает
        исходный диапазон модели, если HailoRT отдал quant_info.
        Путь B (fallback): raw uint8 как float32 as-is — направление
        вектора после L2-нормализации остаётся устойчивым к
        неизвестному аффинному масштабированию (см. модульный docstring
        про инвариантность cosine similarity), но абсолютная точность
        хуже. Выбранный путь ничем не логируется на каждый вызов
        (это hot path), но зафиксирован здесь текстом для отладки.
        """
        vec = raw_uint8.astype('float32')
        if getattr(self, '_output_is_float', False):
            # HailoRT уже отдал float32 — деквантовать второй раз нельзя.
            return vec
        if self._quant_scale is not None and self._quant_zero_point is not None:
            vec = (vec - self._quant_zero_point) * self._quant_scale
            return vec
        # Ни FLOAT32-выхода, ни quant_infos. Вычесть середину диапазона —
        # грубо, но неизмеримо лучше, чем оставить всё в положительном
        # октанте, где любые два лица похожи на 0.999 (см. _init_locked).
        return vec - 128.0

    def close(self) -> None:
        """Освободить биндинги/устройство. Безопасно вызывать повторно."""
        self._bindings = None
        self._configured = None
        self._infer_model = None
        self._vdevice = None
        self._device = None


# ============================================================================
# Свободные функции — геометрия кропа, препроцессинг, метрики
# ============================================================================

def crop_face(frame_rgb: Any, bbox_norm: Tuple[float, float, float, float], margin: float = 0.4) -> Optional[Any]:
    """Вырезать лицо из кадра по normalized bbox с запасом по краям.

    Args:
        frame_rgb: HxWx3 RGB uint8 ndarray (полный кадр, формат — как
            ``Frame.rgb`` из ``gaze.py`` / вход ``RetinaFaceLoader.infer``).
        bbox_norm: (cx, cy, w, h), normalized [0, 1] — ровно то, что
            отдаёт ``_post_process_faces`` в ``vision_face_loader.py``
            (``bbox_cx``/``bbox_cy``/``bbox_w``/``bbox_h``).
        margin: доля расширения бокса на КАЖДУЮ сторону (0.4 — запас
            под причёску/уши, как в ADR-0123 §4.2 "запас 40% по краям").

    Returns:
        RGB uint8 crop, либо ``None`` если бокс вырожден (нулевая/
        отрицательная сторона) или после клампа к границам кадра не
        осталось ни одного пикселя (бокс целиком снаружи кадра).
    """
    if frame_rgb is None:
        return None

    h, w = frame_rgb.shape[0], frame_rgb.shape[1]
    if h <= 0 or w <= 0:
        return None

    cx, cy, bw, bh = bbox_norm
    if bw <= 0.0 or bh <= 0.0:
        return None

    # Расширяем бокс на margin с каждой стороны -> сторона умножается на
    # (1 + 2*margin) относительно исходной.
    exp_w = bw * (1.0 + 2.0 * margin)
    exp_h = bh * (1.0 + 2.0 * margin)

    x1 = (cx - exp_w / 2.0) * w
    y1 = (cy - exp_h / 2.0) * h
    x2 = (cx + exp_w / 2.0) * w
    y2 = (cy + exp_h / 2.0) * h

    # Клампим к границам кадра.
    x1i = max(0, int(round(x1)))
    y1i = max(0, int(round(y1)))
    x2i = min(w, int(round(x2)))
    y2i = min(h, int(round(y2)))

    if x2i <= x1i or y2i <= y1i:
        return None

    return frame_rgb[y1i:y2i, x1i:x2i]


def prepare_arcface_input(crop_rgb: Any) -> Optional[Any]:
    """Ресайз RGB uint8 кропа до (112, 112, 3) uint8 под вход ArcFace.

    Возвращает ``None``, если cv2 недоступен (см. ``gaze.py`` — та же
    конвенция ленивого импорта, чтобы модуль без cv2 всё ещё импортировался
    и юнит-тесты геометрии/метрик проходили в CI без HailoRT/opencv) или
    кроп пустой/вырожденной формы.
    """
    if crop_rgb is None:
        return None
    if crop_rgb.shape[0] <= 0 or crop_rgb.shape[1] <= 0:
        return None

    try:
        import cv2  # type: ignore[import-not-found]
        import numpy as np  # type: ignore[import-not-found]
    except ImportError:
        _LOG.warning('cv2/numpy недоступны — ArcFace препроцессинг невозможен')
        return None

    resized = cv2.resize(
        crop_rgb, (ARCFACE_INPUT_SIZE, ARCFACE_INPUT_SIZE)
    )
    if resized.dtype != np.uint8:
        resized = resized.astype(np.uint8)
    return resized


def sharpness(crop_rgb: Any) -> float:
    """Variance-of-Laplacian резкость кропа. Больше — резче. 0.0 если cv2 нет.

    Используется отбором "1-3 лучших кадра на встречу" (ADR-0123 §3:
    "крупнее, фронтальнее, резче") — эта функция закрывает критерий
    "резче". Возвращает float, не bool/threshold — порог решает вызывающий
    код (лицевой трекер), не эта функция.
    """
    if crop_rgb is None:
        return 0.0
    if crop_rgb.shape[0] <= 0 or crop_rgb.shape[1] <= 0:
        return 0.0

    try:
        import cv2  # type: ignore[import-not-found]
        import numpy as np  # type: ignore[import-not-found]
    except ImportError:
        _LOG.warning('cv2/numpy недоступны — sharpness недоступен')
        return 0.0

    gray = cv2.cvtColor(crop_rgb, cv2.COLOR_RGB2GRAY)
    laplacian = cv2.Laplacian(gray, cv2.CV_64F)
    return float(np.var(laplacian))


def cosine_similarity(a: Any, b: Any) -> float:
    """Косинусное сходство двух 1-D векторов. 0.0 при несовпадении формы/нулевой норме.

    Используется узнаванием (ADR-0123 §6: "встреча сравнивается с
    галереями всех записей") — по одному сравнению на пару эмбеддингов.
    """
    import numpy as np  # type: ignore[import-not-found]

    a = np.asarray(a, dtype='float64').reshape(-1)
    b = np.asarray(b, dtype='float64').reshape(-1)
    if a.shape != b.shape:
        return 0.0

    norm_a = np.linalg.norm(a)
    norm_b = np.linalg.norm(b)
    if norm_a == 0.0 or norm_b == 0.0:
        return 0.0

    return float(np.dot(a, b) / (norm_a * norm_b))


def encode_jpeg(rgb: Any, quality: int = 85) -> Optional[bytes]:
    """RGB uint8 -> JPEG bytes. ``None`` если cv2 недоступен.

    Нужен снимкам встреч (ADR-0123 §4.2). cv2 работает в BGR, поэтому
    перед ``imencode`` каналы переставляются — тот же порядок конвертации,
    что ``gaze.py`` держит в обратную сторону (BGR -> RGB при decode).
    """
    if rgb is None:
        return None

    try:
        import cv2  # type: ignore[import-not-found]
    except ImportError:
        _LOG.warning('cv2 недоступен — JPEG encode невозможен')
        return None

    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    ok, buf = cv2.imencode(
        '.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    )
    if not ok:
        return None
    return buf.tobytes()


def _l2_normalize(vec: Any) -> Any:
    """L2-нормализация 1-D вектора. Вырожденный (нулевая норма) -> как есть."""
    import numpy as np  # type: ignore[import-not-found]

    vec = np.asarray(vec, dtype='float32').reshape(-1)
    norm = np.linalg.norm(vec)
    if norm == 0.0:
        return vec
    return vec / norm
