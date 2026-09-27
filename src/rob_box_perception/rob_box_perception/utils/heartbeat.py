#!/usr/bin/env python3
"""heartbeat.py — файловый heartbeat живости vision-нод (issue #2703, #2704).

Проблема (issue #2703): healthcheck vision-hailo/vision-face судил о
живости ноды по наличию событий в ``/vision/hailo/events``. В real-режиме
тишина в топике — норма (пустая сцена, ``filter_by_confidence`` отсекает
всё ниже порога), поэтому исправная нода в пустой комнате красилась
``unhealthy`` (подтверждено проходом деплоя 17.09.2026, см. issue #2707).

Решение: общий файловый heartbeat, который нода обновляет **по факту
успешно выполненного ``infer()``**, а не по факту публикации события.
Healthcheck-скрипт в контейнере (``healthcheck_frame.sh`` /
``healthcheck_face_frame.sh``) читает возраст файла — не топик и не
``ros2 topic echo`` (последнее ещё и требует живого демона ros2cli,
который на Vision Pi независимо ломался под ``rmw_zenoh``, issue #2703
п.1 — вне scope этого фикса).

Один хелпер на обе ноды (``vision_hailo_node`` / ``vision_face_node``,
второй — сабкласс первого), чтобы не плодить две копии одной и той же
логики записи файла.

Тестируемость: ``FileHeartbeat`` принимает инжектируемый ``time_fn``
(по умолчанию ``time.time``), поэтому тесты свежести/протухания heartbeat
не должны спать реальные секунды — просто дают детерминированный
callable. (Примечание: ``rob_box_perception.utils.time_provider`` —
``TimeAwarenessProvider`` для человекочитаемого контекста времени суток
("утро"/"день"), это НЕ инжектируемые часы для тестов — не подходит
для этой задачи, поэтому здесь отдельный, более простой механизм.)
"""

from __future__ import annotations

import os
import time
from typing import Callable, Optional


class FileHeartbeat:
    """Пишет метку времени в файл после каждого успешного ``infer()``.

    ``beat()`` вызывается нодой ТОЛЬКО когда ``HEFLoader.infer(...)``
    завершился без исключения — если inference падает (degraded state,
    issue #2538 п.6), heartbeat НЕ обновляется, и файл естественно
    "стареет" до порога healthcheck'а.

    Запись атомарна (``os.replace`` из temp-файла в том же каталоге) —
    healthcheck, читающий файл параллельно, никогда не увидит частично
    записанные данные.
    """

    def __init__(
        self,
        path: str,
        time_fn: Callable[[], float] = time.time,
    ) -> None:
        self._path = path
        self._time_fn = time_fn

    @property
    def path(self) -> str:
        return self._path

    def beat(self) -> None:
        """Обновить heartbeat-файл текущим временем.

        Вызывать строго после успешного ``infer()`` (см. docstring класса).
        I/O ошибка (например, ``/tmp`` недоступен) НЕ должна ронять ноду —
        capability-honest здесь означает: если запись не удалась, файл
        просто не обновится, и healthcheck честно покраснеет по возрасту
        (либо по отсутствию файла), а не молча скроет проблему.
        """
        ts = self._time_fn()
        tmp_path = f'{self._path}.tmp{os.getpid()}'
        try:
            with open(tmp_path, 'w', encoding='utf-8') as f:
                f.write(f'{ts:.6f}\n')
            os.replace(tmp_path, self._path)
        except OSError:
            try:
                if os.path.exists(tmp_path):
                    os.remove(tmp_path)
            except OSError:
                pass

    def age_sec(self) -> Optional[float]:
        """Возраст heartbeat-файла в секундах.

        Читает метку времени ИЗ СОДЕРЖИМОГО файла (то, что записал
        ``beat()``), а не через ``os.path.getmtime()``. Это намеренно:
        в тестах ``time_fn`` — детерминированный fake-clock (см. модуль
        docstring), и сравнение "время из файла" vs "время из того же
        ``time_fn``" даёт корректный возраст без реального ``sleep()``.
        В проде ``time_fn=time.time`` по умолчанию, поэтому значение
        практически совпадает с OS mtime (docker healthcheck-скрипт,
        впрочем, читает именно OS mtime через ``stat`` — независимый
        путь, не завязанный на эту функцию).

        Returns:
            float: секунд с последнего ``beat()``.
            None: файла нет, он пуст или содержимое не парсится — то есть
                heartbeat никогда не было или запись была прервана.
        """
        try:
            with open(self._path, 'r', encoding='utf-8') as f:
                content = f.read().strip()
            ts = float(content)
        except (OSError, ValueError):
            return None
        return self._time_fn() - ts


def default_heartbeat_path(node_name: str) -> str:
    """Дефолтный путь heartbeat-файла по имени ноды.

    ``vision_hailo`` -> ``/tmp/vision_hailo_heartbeat``
    ``vision_face``  -> ``/tmp/vision_face_heartbeat``

    Совпадает с путём, который по умолчанию проверяют
    ``docker/vision/scripts/vision-hailo/healthcheck_frame.sh`` и
    ``healthcheck_face_frame.sh`` — если меняете один, синхронизируйте
    другой (или используйте ``heartbeat_path`` launch-параметр /
    ``HEARTBEAT_PATH`` env, см. ``launch_factory.py``).
    """
    return f'/tmp/{node_name}_heartbeat'
