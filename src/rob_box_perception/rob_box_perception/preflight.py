"""Shared pre-flight helper для vision_* launch-файлов (ADR-0018).

``make_preflight_check(component_prefix)`` собирает ``OpaqueFunction``
фабрику, которую используют и ``vision_hailo.launch.py``, и
``vision_face.launch.py``. Раньше каждая launch-функция дублировала
~30 строк одинаковой логики (проверка ``/dev/hailo0``, ``hef_path``,
deps ``numpy/cv2/hailo_platform``) — теперь SSoT живёт здесь.

Capability-honest (ADR-0018): если ``hailo_enabled=true``, но одна из
проверок не прошла — логируем ``[WARN] {prefix}.preflight: ...`` и
**не блокируем** запуск. Нода сама деградирует в stub-режим через
``_is_real_mode`` в ``vision_hailo_node.py`` / ``vision_face_node.py``.

Touchpoints:
- ADR-0018 §1 capability-honest.
- ADR-0110 §3 (vision_hailo.launch.py SSoT, был эталонным).
- Issue #2599 PR-A (vision_face.launch.py — был зеркальной копией).
- Issue #2656 (component review 2026-09-15, дефект LOW: дубликат).
"""

from __future__ import annotations

import os
from typing import Any, Callable, List

from launch.substitutions import LaunchConfiguration


_HAILO_DEVICE = '/dev/hailo0'
_PYTHON_DEPS: tuple[str, ...] = ('numpy', 'cv2', 'hailo_platform')


def _build_preflight(component_prefix: str):
    """Собрать ``OpaqueFunction``-совместимую функцию для launch-файла.

    Возвращает замыкание с сигнатурой ``(context, *args, **kwargs) -> list``.
    ``component_prefix`` подставляется в WARN-сообщения, чтобы оператор
    видел, какая именно нода деградирует (vision_hailo vs vision_face).
    """
    def _preflight_check(context, *args, **kwargs) -> List[Any]:
        hailo_enabled = LaunchConfiguration('hailo_enabled').perform(context)
        hef_path = LaunchConfiguration('hef_path').perform(context)

        messages: List[str] = []

        if hailo_enabled.lower() == 'true':
            if not os.path.exists(_HAILO_DEVICE):
                messages.append(
                    f'{component_prefix}.preflight: {_HAILO_DEVICE} '
                    'отсутствует — нода деградирует в stub-режим '
                    '(ADR-0018 capability-honest).'
                )
            if hef_path and not os.path.isfile(hef_path):
                messages.append(
                    f'{component_prefix}.preflight: hef_path={hef_path!r} '
                    'не является файлом — нода деградирует в stub-режим.'
                )
            # Python deps. Не фатально если их нет — узел логирует подробнее.
            for dep in _PYTHON_DEPS:
                try:
                    __import__(dep)
                except ImportError:
                    messages.append(
                        f'{component_prefix}.preflight: {dep} не установлен '
                        '— нода деградирует в stub-режим.'
                    )

        for msg in messages:
            print(f'[WARN] {msg}')
        return []

    return _preflight_check


# Совместимость со старым API: некоторые launch-файлы (если будут)
# могут импортировать ``make_preflight_check`` напрямую.
def make_preflight_check(
    component_prefix: str,
) -> Callable[..., List[Any]]:
    """Public factory — вернуть ``OpaqueFunction``-совместимую функцию.

    Использование::

        from rob_box_perception.preflight import make_preflight_check

        OpaqueFunction(function=make_preflight_check('vision_hailo'))
        OpaqueFunction(function=make_preflight_check('vision_face'))
    """
    return _build_preflight(component_prefix)