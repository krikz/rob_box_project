"""Глушилка ALSA-ругани на файловом дескрипторе stderr.

Единственное объявление ``ignore_stderr`` на пакет. До карточки W6-1 их
было три дословные копии — ``audio_node.py``, ``sound_node.py``,
``tts_node.py``; копия в ``sound_node`` при этом была мёртвой (объявлена
и ни разу не вызвана).

Модуль намеренно **без тяжёлых зависимостей** (только stdlib), в отличие
от соседнего ``audio_utils.py``, который жёстко импортирует ``pyaudio``:
хелпер нужен и ``tts_node``, который работает через ``sounddevice`` и
pyaudio не тянет, и тестам, которые грузят модуль по пути в обход
``utils/__init__.py`` (та же схема, что у ``audio_transcode.py``).
"""

import os
import sys
from contextlib import contextmanager
from typing import Iterator


@contextmanager
def ignore_stderr(enable: bool = True) -> Iterator[None]:
    """Подавить ALSA-ошибки от PyAudio/sounddevice (как в jsk-ros-pkg).

    https://github.com/jsk-ros-pkg/jsk_3rdparty/blob/master/respeaker_ros/src/respeaker_ros/__init__.py

    ALSA печатает диагностику из C-кода прямо в дескриптор 2, минуя
    ``sys.stderr``, поэтому подменять надо именно fd: ``dup2`` уводит его
    в ``/dev/null`` на время блока и возвращает обратно в ``finally`` —
    в том числе если тело блока бросило исключение.

    Args:
        enable: ``False`` — сквозной проход, ничего не подменяем.
    """
    if enable:
        devnull = None
        try:
            devnull = os.open(os.devnull, os.O_WRONLY)
            stderr = os.dup(2)
            sys.stderr.flush()
            os.dup2(devnull, 2)
            try:
                yield
            finally:
                os.dup2(stderr, 2)
                os.close(stderr)
        finally:
            if devnull is not None:
                os.close(devnull)
    else:
        yield
