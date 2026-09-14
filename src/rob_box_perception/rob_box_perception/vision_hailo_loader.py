"""HEF loader interface для vision_hailo_node (ADR-0089).

Зачем отдельный модуль: класс `HEFLoader` + его реализации (Stub, Real)
НЕ должны зависеть от rclpy — иначе unit-тесты без colcon-build
падают с ModuleNotFoundError. Этот модуль — чистый Python, без ROS,
импортируется и тестируется в любом окружении.

Сам ROS-узел (`vision_hailo_node.py`) импортирует `make_loader`
отсюда. На Pi с rclpy + HailoRT — работает real loader; в CI — stub.
"""

from __future__ import annotations

import os
import time
from typing import Any, Dict, Iterable, List, Optional


# Default HailoRT VDevice id. На Vision Pi с одним AI HAT+ всегда 0.
DEFAULT_VDEVICE_ID = 0


# ============================================================================
# Интерфейс и реализации
# ============================================================================

class HEFLoader:
    """Абстракция над HailoRT VDevice + HEF.

    Узел `vision_hailo_node` общается с HEF через этот интерфейс, чтобы
    тесты могли подменять его `MockHEFLoader` без условных `if CI`.

    NOTE (ADR-0089, decision Q2=A): capability-probing (`is_available`)
    намеренно НЕ часть интерфейса. На Phase 1 маршрутизация делается
    только через фабрику `make_loader(hailo_enabled, hef_path)`, а
    проверка реальной доступности HailoRT отложена в Phase 1.5
    (когда `RealHEFLoader` станет настоящим и будет ясно, где
    физически живёт это знание — в ноде, в launch, или в healthcheck).
    """

    def infer(self, frame_id: str, image: Any) -> List[Dict[str, Any]]:
        """Запуск инференса на одном кадре.

        Args:
            frame_id: ROS header.frame_id источника (например "oak_rgb_frame").
            image: numpy.ndarray с картинкой (формат зависит от модели).

        Returns:
            List[dict]: список детекций в формате VisionEvent-полей
            (без stamp — его проставляет ROS-нода).
        """
        raise NotImplementedError


class StubHEFLoader(HEFLoader):
    """Детерминированный loader для CI / pre-HEF smoke tests.

    Публикует ровно один event каждые `period_sec` секунд:
    "person at 1.0m, confidence 0.92". Это даёт downstream'у
    стабильный сигнал для проверки, что topic жив и формат
    корректный, без зависимости от железа.
    """

    def __init__(self, period_sec: float = 2.0) -> None:
        self._period_sec = period_sec
        self._last_emit = 0.0

    def infer(self, frame_id: str, image: Any) -> List[Dict[str, Any]]:
        now = time.monotonic()
        if now - self._last_emit < self._period_sec:
            return []
        self._last_emit = now
        return [{
            'source_camera': frame_id or 'stub',
            'event_type': 'person',
            'class_name': 'person',
            'class_id': 0,
            'confidence': 0.92,
            'bbox_cx': 0.5,
            'bbox_cy': 0.5,
            'bbox_w': 0.4,
            'bbox_h': 0.6,
            'distance_m': 1.0,
            'embedding_id': '',
            'display_name': '',
            'attributes_json': '',
        }]


class RealHEFLoader(HEFLoader):
    """Реальный HEF loader через `hailort` Python API.

    Импорт `hailo_platform` ленивый — пакет доступен только на Vision Pi
    с установленным HailoRT. Это сохраняет CI-сборку без зависимости от
    ARM64-specific deb-пакета.

    NOTE: Этот класс — контракт для Phase 1 deployment. Сам HailoRT
    вызов (`infer`) **не реализуется в этом stub'е** — задача Phase 1
    acceptance: `hailortcli scan` на железе + pytest тесты на этом
    классе через MockHEFLoader. Реальный `infer` появится в Phase 1.5
    после установки HEF на Vision Pi 5.
    """

    def __init__(
        self,
        hef_path: str,
        vdevice_id: int = DEFAULT_VDEVICE_ID,
    ) -> None:
        self._hef_path = hef_path
        self._vdevice_id = vdevice_id
        self._vdevice = None
        self._infer_model = None

    def _ensure_initialized(self) -> None:
        """Ленивая инициализация HailoRT VDevice + HEF.

        Raises:
            ImportError: hailort не установлен (не на том хосте).
            FileNotFoundError: HEF файл не найден.
            RuntimeError: VDevice не доступен / HEF не компилируется.
        """
        if self._infer_model is not None:
            return
        try:
            from hailo_platform import VDevice, HailoRTException  # noqa: F401
        except ImportError as exc:
            raise ImportError(
                'hailo_platform не установлен. Установите HailoRT на Vision Pi '
                'перед включением hailo_enabled=True (см. docker/vision/vision-hailo/Dockerfile).'
            ) from exc
        if not os.path.isfile(self._hef_path):
            raise FileNotFoundError(
                f'HEF не найден: {self._hef_path}. Скачайте из hailo_model_zoo '
                f'(https://github.com/hailo-ai/hailo_model_zoo).'
            )
        # Реальный код инициализации появится в Phase 1.5:
        #   self._vdevice = VDevice(vdevice_id=self._vdevice_id)
        #   self._infer_model = self._vdevice.create_infer_model(self._hef_path)
        #   self._infer_model.configure()
        raise RuntimeError(
            'RealHEFLoader.infer — Phase 1.5 задача. Phase 1 acceptance: '
            'hailortcli scan + Smoke на Vision Pi. Используйте hailo_enabled=False.'
        )

    def infer(self, frame_id: str, image: Any) -> List[Dict[str, Any]]:
        # Контракт Phase 1: инициализация + post-processing pipeline.
        # Реализация отложена в Phase 1.5 (на железе).
        self._ensure_initialized()
        return []  # pragma: no cover

    # NOTE (ADR-0089, decision Q2=A): capability-probing убран из
    # публичного интерфейса. Маршрутизация stub/real делается в
    # `make_loader(hailo_enabled, hef_path)` — фаза выбирается по
    # launch-параметрам, а не по runtime-пробингу. Если в Phase 1.5
    # понадобится runtime availability check, его следует реализовать
    # внутри `RealHEFLoader` (приватный метод) или вынести в отдельный
    # healthcheck-хелпер, но НЕ возвращать в `HEFLoader` interface.


# ============================================================================
# Factory
# ============================================================================

def make_loader(
    hailo_enabled: bool,
    hef_path: Optional[str],
    stub_period_sec: float,
) -> HEFLoader:
    """Вернуть правильный loader по launch-параметрам.

    Args:
        hailo_enabled: launch-параметр, разрешает использовать HailoRT.
        hef_path: путь к .hef файлу (None или пустая строка = stub).
        stub_period_sec: период публикации stub-событий.
    """
    if hailo_enabled and hef_path:
        return RealHEFLoader(hef_path=hef_path)
    return StubHEFLoader(period_sec=stub_period_sec)


# ============================================================================
# Filter helper (без rclpy, чистый Python — тестируется отдельно)
# ============================================================================

def filter_by_confidence(
    events: Iterable[Dict[str, Any]],
    threshold: float,
) -> List[Dict[str, Any]]:
    """Фильтр VisionEvent-диктов по confidence_threshold.

    Контракт: `confidence >= threshold` -> keep, иначе drop.
    threshold ровно 0.5 — keep (>=).
    """
    return [
        ev for ev in events
        if float(ev.get('confidence', 0.0)) >= threshold
    ]


# ============================================================================
# VisionEvent-dict normalizer (используется и в Node, и в тестах)
# ============================================================================

VISION_EVENT_FIELDS = (
    'source_camera', 'event_type', 'class_name', 'class_id',
    'confidence', 'bbox_cx', 'bbox_cy', 'bbox_w', 'bbox_h',
    'distance_m', 'embedding_id', 'display_name', 'attributes_json',
)


def normalize_event_dict(event_dict: Dict[str, Any]) -> Dict[str, Any]:
    """Вернуть dict с типизированными полями VisionEvent.

    Defaults:
        source_camera='', event_type='scene', class_name='', class_id=-1,
        confidence=0.0, bbox_*=-1.0, distance_m=-1.0,
        embedding_id='', display_name='', attributes_json=''.

    Используется в vision_hailo_node._publish_event (там же
    проставляется stamp), в тестах build_event round-trip, и
    в context_aggregator при маппинге на PerceptionEvent JSON.
    """
    return {
        'source_camera': str(event_dict.get('source_camera', '')),
        'event_type': str(event_dict.get('event_type', 'scene')),
        'class_name': str(event_dict.get('class_name', '')),
        'class_id': int(event_dict.get('class_id', -1)),
        'confidence': float(event_dict.get('confidence', 0.0)),
        'bbox_cx': float(event_dict.get('bbox_cx', -1.0)),
        'bbox_cy': float(event_dict.get('bbox_cy', -1.0)),
        'bbox_w': float(event_dict.get('bbox_w', -1.0)),
        'bbox_h': float(event_dict.get('bbox_h', -1.0)),
        'distance_m': float(event_dict.get('distance_m', -1.0)),
        'embedding_id': str(event_dict.get('embedding_id', '')),
        'display_name': str(event_dict.get('display_name', '')),
        'attributes_json': str(event_dict.get('attributes_json', '')),
    }
