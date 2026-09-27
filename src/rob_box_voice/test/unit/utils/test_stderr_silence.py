"""``ignore_stderr`` — одно объявление на пакет, а не три копии.

Контекст (карточка W6-1, скан дублей
``docs/plans/2026-08-30-dedup-wave3-handoff.md`` §3.2): контекстный
менеджер, глушащий ALSA-ругань на fd 2, лежал тремя дословными копиями:

* ``audio_node.py:31``  — с единственной ссылкой на первоисточник
  (jsk-ros-pkg/respeaker_ros), где записано, ОТКУДА приём;
* ``sound_node.py:36``  — копия **мёртвая**: объявлена и ни разу не
  вызвана (``grep -n ignore_stderr`` по пакету давал только ``def``);
* ``tts_node.py:147``   — живая, используется на открытии
  ``sounddevice``-потока.

Тот же класс, что уже стоил issue #1252 / #1734 с ``wake_words``: один
приём, размноженный по файлам, и расходящиеся докстринги вместо общего
объяснения.

Модуль грузится **по пути**, минуя ``utils/__init__.py``: тот тянет
``audio_utils`` → ``pyaudio``, которого на dev-контейнерах нет. Ровно так
же поступает ``test_audio_transcode.py`` — см. комментарий там.
"""

from __future__ import annotations

import ast
import importlib.util as _ilu
import os
import pathlib as _pl
import sys as _sys

import pytest

# На Windows дескриптор в текстовом режиме дописывает CR перед каждым LF —
# побайтовое сравнение этого не переживёт. На POSIX флага нет, и там это 0.
_O_BINARY = getattr(os, "O_BINARY", 0)

_VOICE = _pl.Path(__file__).resolve().parents[3] / "rob_box_voice"
_MODULE_PATH = _VOICE / "utils" / "stderr_silence.py"

_spec = _ilu.spec_from_file_location("rob_box_voice_stderr_silence", str(_MODULE_PATH))
assert _spec is not None and _spec.loader is not None, _MODULE_PATH
_stderr_silence = _ilu.module_from_spec(_spec)
_sys.modules[_spec.name] = _stderr_silence
_spec.loader.exec_module(_stderr_silence)

ignore_stderr = _stderr_silence.ignore_stderr


# Ноды, которые обязаны брать хелпер из общего модуля, а не объявлять свой.
_NODES = ("audio_node.py", "sound_node.py", "tts_node.py")


def _write_to_fd2(payload: bytes) -> None:
    """Написать в дескриптор 2 напрямую — так и делает ALSA из C-кода."""
    os.write(2, payload)


def test_suppresses_writes_to_fd2(tmp_path) -> None:
    """Внутри блока запись в fd 2 не доходит до настоящего stderr.

    Проверяем именно дескриптор, а не ``sys.stderr``: ALSA печатает из
    C-кода мимо Python-объекта, поэтому подменять ``sys.stderr`` тут
    бесполезно — ради этого хелпер и дублирует fd через ``os.dup2``.
    """
    sink = tmp_path / "stderr.log"
    saved = os.dup(2)
    try:
        fd = os.open(str(sink), os.O_WRONLY | os.O_CREAT | os.O_TRUNC | _O_BINARY)
        try:
            os.dup2(fd, 2)
        finally:
            os.close(fd)

        with ignore_stderr(enable=True):
            _write_to_fd2(b"ALSA lib pcm.c: cannot open\n")
        _write_to_fd2(b"visible\n")
    finally:
        os.dup2(saved, 2)
        os.close(saved)

    assert sink.read_bytes() == b"visible\n"


def test_enable_false_is_a_passthrough(tmp_path) -> None:
    """``enable=False`` ничего не подменяет — вывод виден."""
    sink = tmp_path / "stderr.log"
    saved = os.dup(2)
    try:
        fd = os.open(str(sink), os.O_WRONLY | os.O_CREAT | os.O_TRUNC | _O_BINARY)
        try:
            os.dup2(fd, 2)
        finally:
            os.close(fd)

        with ignore_stderr(enable=False):
            _write_to_fd2(b"kept\n")
    finally:
        os.dup2(saved, 2)
        os.close(saved)

    assert sink.read_bytes() == b"kept\n"


def test_restores_fd2_when_body_raises() -> None:
    """Исключение внутри блока не оставляет fd 2 подменённым."""
    before = os.fstat(2)
    with pytest.raises(RuntimeError):
        with ignore_stderr(enable=True):
            raise RuntimeError("boom")
    after = os.fstat(2)
    assert (before.st_dev, before.st_ino) == (after.st_dev, after.st_ino)


def _toplevel_functions(path: _pl.Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    return {
        node.name
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }


def test_no_node_declares_its_own_copy() -> None:
    """Предохранитель: четвёртая копия должна ронять тест.

    Ищем именно top-level ``def ignore_stderr`` в исходниках нод — ноды
    тянут ``rclpy``/``grpc``, импортировать их незачем.
    """
    offenders = [name for name in _NODES if "ignore_stderr" in _toplevel_functions(_VOICE / name)]
    assert not offenders, (
        "ignore_stderr объявлен заново — он живёт в "
        "rob_box_voice.utils.stderr_silence и больше нигде: " + ", ".join(offenders)
    )


def test_users_import_the_shared_helper() -> None:
    """Кто вызывает хелпер — импортирует его из общего модуля.

    ``sound_node`` в списке нет намеренно: его копия была мёртвой, и
    вместе с ней ушёл и импорт. Появится вызов — появится и импорт,
    предыдущий тест не даст объявить копию.
    """
    for name in ("audio_node.py", "tts_node.py"):
        source = (_VOICE / name).read_text(encoding="utf-8")
        assert "from .utils.stderr_silence import ignore_stderr" in source, (
            f"{name} вызывает ignore_stderr, но не импортирует общий хелпер"
        )
