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

    ``tts_node`` берёт ``silence_alsa_errors`` (один раз при старте), а не
    ``ignore_stderr`` (#3730: подмена fd 2 вокруг ``play_audio`` съедала
    ``SpeechEvent``-строки соседних потоков).
    """
    expected = {
        "audio_node.py": "from .utils.stderr_silence import ignore_stderr",
        "tts_node.py": "from .utils.stderr_silence import silence_alsa_errors",
    }
    for name, import_line in expected.items():
        source = (_VOICE / name).read_text(encoding="utf-8")
        assert import_line in source, f"{name} вызывает хелпер, но не импортирует его из общего модуля"


# ── silence_alsa_errors (#3730): один раз на процесс, fd 2 не трогаем ─────────


class _FakeAsound:
    """``libasound``: запоминает, какой обработчик ему поставили."""

    def __init__(self) -> None:
        self.handlers: list = []

    def snd_lib_error_set_handler(self, handler) -> None:
        self.handlers.append(handler)


def _install_fake_asound(monkeypatch, asound) -> None:
    monkeypatch.setattr(_stderr_silence, "_alsa_handler", None)
    monkeypatch.setattr(_stderr_silence.ctypes.util, "find_library", lambda _name: "libasound-fake")
    monkeypatch.setattr(_stderr_silence.ctypes.cdll, "LoadLibrary", lambda _name: asound, raising=False)


def test_silence_alsa_errors_installs_a_noop_handler_once(monkeypatch) -> None:
    """Повторные вызовы ничего не переустанавливают; обработчик молчит и не падает."""
    asound = _FakeAsound()
    _install_fake_asound(monkeypatch, asound)

    assert _stderr_silence.silence_alsa_errors() is True
    assert _stderr_silence.silence_alsa_errors() is True

    assert len(asound.handlers) == 1
    # ALSA вызывает обработчик как (file, line, function, err, fmt, ...): он обязан проглотить вызов.
    assert asound.handlers[0](b"pcm.c", 1, b"snd_pcm_open", 2, b"Unknown PCM %s") is None


def test_silence_alsa_errors_without_libasound_is_a_noop(monkeypatch) -> None:
    """Windows / CI без ``libasound``: ``False``, ничего не падает, следующий вызов пробует снова."""
    monkeypatch.setattr(_stderr_silence, "_alsa_handler", None)
    monkeypatch.setattr(_stderr_silence.ctypes.util, "find_library", lambda _name: None)

    def _missing(_name):
        raise OSError("libasound.so.2: cannot open shared object file")

    monkeypatch.setattr(_stderr_silence.ctypes.cdll, "LoadLibrary", _missing, raising=False)

    assert _stderr_silence.silence_alsa_errors() is False
    assert _stderr_silence.silence_alsa_errors() is False
    assert _stderr_silence._alsa_handler is None


def test_silence_alsa_errors_leaves_fd2_alone(monkeypatch, tmp_path) -> None:
    """Глушилка ALSA не подменяет fd 2: строка лога из любого потока доходит до stderr."""
    _install_fake_asound(monkeypatch, _FakeAsound())
    sink = tmp_path / "stderr.log"
    saved = os.dup(2)
    try:
        fd = os.open(str(sink), os.O_WRONLY | os.O_CREAT | os.O_TRUNC | _O_BINARY)
        try:
            os.dup2(fd, 2)
        finally:
            os.close(fd)

        assert _stderr_silence.silence_alsa_errors() is True
        _write_to_fd2(b"SpeechEvent synthesized\n")
    finally:
        os.dup2(saved, 2)
        os.close(saved)

    assert sink.read_bytes() == b"SpeechEvent synthesized\n"
