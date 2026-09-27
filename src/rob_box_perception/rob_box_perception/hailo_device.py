#!/usr/bin/env python3
"""Шов «Ускоритель» — единственное место, где код берёт ``/dev/hailo0``.

Правило, подтверждённое на живом роботе 15.09.2026 (issue #2599):
**владелец железа — ``hailort.service``**, а нода владеет только смыслом
модели. Процесс, который создаёт ``VDevice`` напрямую, забирает Hailo
единолично, и следующий продюсер падает с
``HAILO_OUT_OF_PHYSICAL_DEVICES(74)``. Ровно это и происходило, когда
``vision_hailo`` (yolov8n) и ``vision_face`` (retinaface) поднялись рядом:
кто успел первым — тот и работал, второй молча жил в degraded.

Через сервис оба процесса делят устройство (``multi_process_service`` +
общий ``group_id`` + планировщик). Плата за это — одно важное следствие,
которое HailoRT говорит дословно:

    ConfiguredNetworkGroup::activate function is not supported when using
    multi-process service or HailoRT Scheduler.

То есть ручной ``activate()`` (он появился в #2398 как лечение
``HAILO_STREAM_NOT_ACTIVATED(72)``) под сервисом обязан быть выключен —
активацией владеет планировщик. Решение «активировать или нет» живёт
здесь, в шве, а не копиями в каждом лоадере: см. ``must_activate``.

Режимы:

- ``service``   — сокет сервиса на месте: несколько процессов, планировщик
  владеет активацией. Единственный режим, в котором лицо и человек
  работают одновременно.
- ``exclusive`` — сервиса нет: процесс владеет устройством сам, ручной
  ``activate()`` обязателен (поведение до #2599, один продюсер).
- ``legacy``    — HailoRT без ``create_params`` (< 4.18): голый
  ``VDevice()``, активация вручную.

Touchpoints:
- ADR-0089 §2.1 (Phase 2; открытый вопрос «процесс или вторая модель»).
- Issue #2599 (лицевой канал), #2398 (real inference, activate()).
- rob_box_perception.vision_hailo_loader / vision_face_loader.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any, Optional

#: UNIX-сокет ``hailort_service`` на хосте. Контейнер обязан его монтировать
#: (docker/vision/docker-compose.yaml), иначе сервис недостижим и режим
#: молча схлопнется в ``exclusive`` — с гонкой за устройство.
DEFAULT_SERVICE_SOCKET = '/tmp/hailort_uds.sock'

#: Общая группа планировщика. Одинаковая у всех продюсеров — иначе сервис
#: разведёт их по разным группам и делить устройство они не смогут.
DEFAULT_GROUP_ID = 'rob_box'

MODE_SERVICE = 'service'
MODE_EXCLUSIVE = 'exclusive'
MODE_LEGACY = 'legacy'


def service_socket_path() -> str:
    """Путь к сокету сервиса (ENV ``HAILORT_SERVICE_SOCKET`` — override)."""
    return os.environ.get('HAILORT_SERVICE_SOCKET', DEFAULT_SERVICE_SOCKET)


def group_id() -> str:
    """Имя группы планировщика (ENV ``HAILO_GROUP_ID`` — override)."""
    return os.environ.get('HAILO_GROUP_ID', DEFAULT_GROUP_ID)


def service_available(socket_path: Optional[str] = None) -> bool:
    """Достижим ли ``hailort_service`` из этого процесса.

    Проверяем именно сокет, а не systemd: сервис может быть жив на хосте,
    но не смонтирован в контейнер — снаружи это выглядит одинаково
    («сервис есть»), а внутри различие решающее.
    """
    return os.path.exists(socket_path or service_socket_path())


@dataclass(frozen=True)
class HailoDevice:
    """Открытое устройство + режим владения им."""

    vdevice: Any
    mode: str

    @property
    def scheduler_managed(self) -> bool:
        """True — активацией владеет планировщик/сервис, не мы."""
        return self.mode == MODE_SERVICE

    @property
    def must_activate(self) -> bool:
        """Нужен ли ручной ``activate()`` перед первым ``run()``.

        ``exclusive``/``legacy`` — да (иначе ``HAILO_STREAM_NOT_ACTIVATED``,
        #2398). ``service`` — нет (иначе ``HAILO_INVALID_OPERATION``).
        """
        return not self.scheduler_managed


def open_device(device_id: int = 0) -> HailoDevice:
    """Открыть Hailo: через сервис, если он достижим, иначе единолично."""
    try:
        from hailo_platform import VDevice  # type: ignore[import-not-found]
    except ImportError as exc:
        raise ImportError(
            'hailo_platform не установлен. Установите HailoRT на Vision Pi '
            'перед включением real-инференса '
            '(см. docker/vision/vision-hailo/Dockerfile).'
        ) from exc

    try:
        from hailo_platform import (  # type: ignore[import-not-found]
            HailoSchedulingAlgorithm,
        )
        params = VDevice.create_params()
        params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
        if service_available():
            # Железом владеет сервис: только так рядом живут две модели.
            params.multi_process_service = True
            params.group_id = group_id()
            return HailoDevice(
                vdevice=VDevice(params=params), mode=MODE_SERVICE
            )
        params.device_id = str(device_id)
        return HailoDevice(
            vdevice=VDevice(params=params), mode=MODE_EXCLUSIVE
        )
    except (ImportError, TypeError, AttributeError):
        # HailoRT < 4.18 или иная minor-вариация API.
        return HailoDevice(vdevice=VDevice(), mode=MODE_LEGACY)
