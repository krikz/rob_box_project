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

import ctypes
import ctypes.util
import os
import sys
import threading
from contextlib import contextmanager
from typing import Any, Iterator, Optional

# Ссылка на C-колбэк ALSA. Пока она жива, колбэк не соберёт GC: ALSA хранит
# голый указатель на функцию, и освобождённый ctypes-колбэк роняет процесс
# на первой же ALSA-ошибке.
_alsa_handler: Optional[Any] = None
_alsa_lock = threading.Lock()


def silence_alsa_errors() -> bool:
    """Заглушить ALSA-ругань ОДИН раз на процесс (``snd_lib_error_set_handler``).

    ALSA печатает диагностику из C-кода («ALSA lib pcm.c: Unknown PCM
    cards.pcm.rear», ...) через собственный обработчик ошибок. Подмена этого
    обработчика на пустой глушит ровно ALSA и ничего больше. Дескриптор 2 не
    трогаем: он общий на процесс, и логи ROS из всех потоков идут в него же
    (карточка #3730, см. :func:`ignore_stderr`).

    Идемпотентна и безопасна без ALSA (Windows, CI без ``libasound``):
    возвращает ``False`` и ничего не делает.

    Returns:
        ``True`` — обработчик установлен (сейчас или ранее), ``False`` — ALSA
        в процессе нет.
    """
    global _alsa_handler
    with _alsa_lock:
        if _alsa_handler is not None:
            return True
        name = ctypes.util.find_library("asound") or "libasound.so.2"
        try:
            asound = ctypes.cdll.LoadLibrary(name)
            # (file, line, function, err, fmt, ...) - varargs обработчику не нужны.
            handler_type = ctypes.CFUNCTYPE(
                None,
                ctypes.c_char_p,
                ctypes.c_int,
                ctypes.c_char_p,
                ctypes.c_int,
                ctypes.c_char_p,
            )
            handler = handler_type(lambda _file, _line, _func, _err, _fmt: None)
            asound.snd_lib_error_set_handler(handler)
        except (OSError, AttributeError):
            return False
        _alsa_handler = handler
        return True


@contextmanager
def ignore_stderr(enable: bool = True) -> Iterator[None]:
    """Подавить ALSA-ошибки от PyAudio/sounddevice (как в jsk-ros-pkg).

    https://github.com/jsk-ros-pkg/jsk_3rdparty/blob/master/respeaker_ros/src/respeaker_ros/__init__.py

    ALSA печатает диагностику из C-кода прямо в дескриптор 2, минуя
    ``sys.stderr``, поэтому подменять надо именно fd: ``dup2`` уводит его
    в ``/dev/null`` на время блока и возвращает обратно в ``finally`` —
    в том числе если тело блока бросило исключение.

    **Только на короткий блок одного потока** (открытие устройства). fd 2
    общий на ВЕСЬ процесс: пока блок идёт, строки лога ROS из любого другого
    потока тоже уходят в ``/dev/null``. Обернутый вокруг блокирующего
    ``play_audio`` он съедал ``SpeechEvent queued/synthesized/finished``
    соседних реплик (#3730). Долгие и блокирующие вызовы глушит
    :func:`silence_alsa_errors`.

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
