"""#3730: строки ``SpeechEvent`` не пропадают из лога tts_node, пока играет реплика.

Причина потери: ``play_audio`` был обёрнут в ``ignore_stderr`` — подмену fd 2 на
``/dev/null``. fd 2 общий на ВЕСЬ процесс, а rclpy пишет в него логи из любого потока:
пока блокирующий ``play_audio`` играл реплику, строки ``SpeechEvent
queued/synthesized/finished`` соседних реплик (пред-синтез следующей, итог предыдущей)
уходили в ``/dev/null``. Судья харнесса (``e2e_events``) и разбор логов недосчитывали
события: в марафоне 23.10 у 47 из 180 речей в логе не было ``synthesized``.

Тест проверяет поведение: логгер пишет в дескриптор 2 из C (как rclpy), чужой поток
логирует во время блокирующего ``play_audio`` - строка обязана дойти до настоящего stderr.
"""

from __future__ import annotations

import ast
import os
import sys
import threading
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_voice.tts_node import TTSNode  # noqa: E402

_TTS_NODE_SOURCE = _PACKAGE_ROOT / "rob_box_voice" / "tts_node.py"


class _Bare:
    """Заглушка ноды: только атрибуты, которых касается ``_sap_local_playback``."""


class _FdLogger:
    """Логгер как у rclpy: строка уходит прямо в fd 2, минуя ``sys.stderr``."""

    def info(self, text: str) -> None:
        os.write(2, (text + "\n").encode("utf-8"))

    warn = error = debug = info


def _playback_node(play_audio) -> _Bare:
    node = _Bare()
    node.get_logger = lambda: _FdLogger()
    node.stop_requested = False
    node.chipmunk_mode = False
    node.pitch_shift = 1.0
    node.volume_gain = 1.0
    node.device_index = None
    node.current_stream = None
    node.publish_state = MagicMock()
    node._release_play_seq = MagicMock()
    node._publish_tts_finished = MagicMock()
    node.cleanup_playback_noise = MagicMock()
    node.playback_manager = MagicMock()
    node.playback_manager.play_audio.side_effect = play_audio
    for name in (
        "_sap_prepare_audio_for_playback",
        "_sap_finalize_after_playback",
        "_sap_publish_finished_failure",
    ):
        setattr(node, name, getattr(TTSNode, name).__get__(node, _Bare))
    return node


def _play(node) -> None:
    audio = np.zeros(1600, dtype=np.float32)
    TTSNode._sap_local_playback(
        node,
        audio,
        16000,
        "Яблоко, груша.",
        None,
        None,
        "4e99b21b-0000",
        None,
        None,
        None,
        None,
        None,
        0.1,
    )


def test_speech_event_from_other_thread_reaches_stderr_while_play_audio_blocks(
    capfd,
) -> None:
    """Пред-синтез следующей реплики логирует ``synthesized``, пока играет предыдущая."""

    def play_audio(**_kwargs) -> bool:
        presynth = threading.Thread(
            target=lambda: _FdLogger().info(
                "🗣️ SpeechEvent synthesized speech_id=7d4e462e agent=tars epoch=1 text_len=47"
            ),
            name="presynth-next-chunk",
        )
        presynth.start()
        presynth.join(timeout=5.0)
        return True

    _play(_playback_node(play_audio))

    err = capfd.readouterr().err
    assert "SpeechEvent synthesized speech_id=7d4e462e" in err, err


def test_log_lines_after_play_audio_are_visible(capfd) -> None:
    """Строки самого итога реплики (``Воспроизведение завершено``) тоже доходят до лога."""
    _play(_playback_node(lambda **_kwargs: True))

    err = capfd.readouterr().err
    assert "Воспроизведение завершено" in err, err
    assert "Публикую TTS finished event: speech_id=4e99b21b" in err, err


def test_play_audio_runs_with_untouched_stderr_descriptor() -> None:
    """Во время ``play_audio`` fd 2 тот же файл, что и до него (не ``/dev/null``)."""
    seen: dict = {}

    def play_audio(**_kwargs) -> bool:
        seen["inside"] = os.fstat(2)
        return True

    before = os.fstat(2)
    _play(_playback_node(play_audio))

    assert (seen["inside"].st_dev, seen["inside"].st_ino) == (
        before.st_dev,
        before.st_ino,
    )


def _init_call_names() -> list[str]:
    tree = ast.parse(
        _TTS_NODE_SOURCE.read_text(encoding="utf-8"), filename=str(_TTS_NODE_SOURCE)
    )
    for cls in (
        n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "TTSNode"
    ):
        for fn in (
            n
            for n in cls.body
            if isinstance(n, ast.FunctionDef) and n.name == "__init__"
        ):
            names = []
            for call in (n for n in ast.walk(fn) if isinstance(n, ast.Call)):
                func = call.func
                names.append(
                    func.attr
                    if isinstance(func, ast.Attribute)
                    else getattr(func, "id", "")
                )
            return names
    raise AssertionError("TTSNode.__init__ не найден")


def test_tts_node_silences_alsa_once_in_init_before_opening_devices() -> None:
    """ALSA глушится один раз при старте ноды и до первого обращения к устройствам."""
    calls = _init_call_names()
    assert calls.count("silence_alsa_errors") == 1, calls
    assert calls.index("silence_alsa_errors") < calls.index(
        "initialize_audio_device"
    ), calls


def test_tts_node_never_swaps_fd2() -> None:
    """``ignore_stderr`` в tts_node не вернётся: ни импорта, ни вызова."""
    tree = ast.parse(
        _TTS_NODE_SOURCE.read_text(encoding="utf-8"), filename=str(_TTS_NODE_SOURCE)
    )
    used = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)} | {
        a.name for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) for a in n.names
    }
    assert "ignore_stderr" not in used
