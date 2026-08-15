#!/usr/bin/env python3
"""test_sound_node_priority.py — Unit тесты приоритетов SoundNode (issue #1328).

Проверяет ``SoundNode.trigger_callback``:
- thinking (accept-звук, prio 20) ПРЕРЫВАЕТ ранний «бульк» (boop → button_click, prio 10)
- бульк НЕ прерывает thinking (prio ниже)
- равный приоритет / обычные звуки — прежнее поведение (пропускаем, первый выигрывает)
- токен поколения: прерванный поток НЕ сбрасывает флаги нового

Тест НЕ требует rclpy/sounddevice: все внешние зависимости замоканы через
sys.modules (как в test_stt_node_boop.py), тестируем только чистую логику
приоритетов через SoundNode.__new__ + ручные атрибуты.
"""

from __future__ import annotations

import sys
import threading
from unittest.mock import MagicMock, patch

import pytest

_OPTIONAL_DEPS = {
    "rclpy": True,
    "rclpy.node": True,
    "rclpy.qos": True,
    "std_msgs": False,
    "std_msgs.msg": False,
    "numpy": False,
    "sounddevice": False,
    "pydub": False,
    "pyaudio": False,
    "usb": False,
    "usb.core": False,
    "usb.util": False,
}


def _node_no_op(self, *a, **kw):
    return None


def _ensure_rclpy_mock(monkeypatch):
    """Минимальные mock-модули для импорта rob_box_voice.sound_node."""

    class _NodeBase:
        def __init__(self, *a, **kw):
            pass

        def declare_parameter(self, *a, **kw):
            pass

        def get_parameter(self, name):
            return MagicMock(value="")

        def create_publisher(self, *a, **kw):
            return MagicMock()

        def create_subscription(self, *a, **kw):
            return MagicMock()

        def get_logger(self):
            return MagicMock(
                info=_node_no_op,
                warning=_node_no_op,
                warn=_node_no_op,
                error=_node_no_op,
                debug=_node_no_op,
            )

    class _NodeMod:
        Node = _NodeBase

    class _Rclpy:
        node = _NodeMod()

        @staticmethod
        def init(*a, **kw):
            pass

        @staticmethod
        def shutdown(*a, **kw):
            pass

        @staticmethod
        def spin(*a, **kw):
            pass

    monkeypatch.setitem(sys.modules, "rclpy", _Rclpy())
    monkeypatch.setitem(sys.modules, "rclpy.node", _Rclpy.node)

    class _QoSMod:
        QoSProfile = MagicMock()
        ReliabilityPolicy = MagicMock()
        DurabilityPolicy = MagicMock()
        HistoryPolicy = MagicMock()

    monkeypatch.setitem(sys.modules, "rclpy.qos", _QoSMod())

    class _Msg:
        String = MagicMock()

    monkeypatch.setitem(sys.modules, "std_msgs", _Msg())
    monkeypatch.setitem(sys.modules, "std_msgs.msg", _Msg)

    monkeypatch.setitem(sys.modules, "numpy", MagicMock())

    # sounddevice / pydub / pyaudio / usb — для импорта sound_node и utils
    monkeypatch.setitem(sys.modules, "sounddevice", MagicMock())
    monkeypatch.setitem(sys.modules, "pydub", MagicMock())

    class _PyAudioMod:
        class PyAudio:
            def __init__(self, *a, **kw):
                pass

            def terminate(self):
                pass

    monkeypatch.setitem(sys.modules, "pyaudio", _PyAudioMod())

    class _UsbCore:
        def find(self, *a, **kw):
            return None

        class Device:
            pass

    class _UsbUtil:
        def get_string(self, *a, **kw):
            return ""

    class _Usb:
        core = _UsbCore()
        util = _UsbUtil()

    monkeypatch.setitem(sys.modules, "usb", _Usb())
    monkeypatch.setitem(sys.modules, "usb.core", _UsbCore())
    monkeypatch.setitem(sys.modules, "usb.util", _UsbUtil())


@pytest.fixture(autouse=True)
def _ensure_optional_deps(monkeypatch):
    """autouse: rclpy/sounddevice/pydub/usb замоканы для каждого теста."""
    import rob_box_voice  # noqa: F401

    _popped: list = []
    _leaf_attr: list = []
    for cached in [
        "rob_box_voice.sound_node",
        "rob_box_voice.audio_playback_manager",
        "rob_box_voice.utils",
        "rob_box_voice.utils.audio_utils",
        "rob_box_voice.utils.respeaker_interface",
    ]:
        _prev = sys.modules.pop(cached, None)
        _popped.append((cached, _prev))
        _leaf = cached.split(".")[-1]
        if hasattr(rob_box_voice, _leaf):
            _leaf_attr.append((rob_box_voice, _leaf, getattr(rob_box_voice, _leaf)))
            delattr(rob_box_voice, _leaf)
    _ensure_rclpy_mock(monkeypatch)
    yield
    # Восстанавливаем модули, чтобы не затронуть другие unit-тесты в сессии.
    for _name, _mod in _popped:
        if _mod is not None:
            sys.modules[_name] = _mod
        else:
            # Модуля не было до нас — удаляем мок-версию, чтобы следующий
            # импортёр сделал чистый reimport (иначе чужие тесты получат
            # звуковую подсистему с замоканным sounddevice).
            sys.modules.pop(_name, None)
    # Восстанавливаем атрибуты пакета (delattr в setup убрал их; reimport
    # повесил новые мок-модули — чужие тесты должны видеть прежние).
    for _pkg, _leaf_name, _attr in _leaf_attr:
        setattr(_pkg, _leaf_name, _attr)


def _make_node(**attrs):
    """SoundNode-stub без rclpy: только поля, нужные trigger_callback."""
    from rob_box_voice import sound_node as sound_node_module

    node = sound_node_module.SoundNode.__new__(sound_node_module.SoundNode)
    node.get_logger = lambda: MagicMock(
        info=lambda *a, **kw: None,
        warning=lambda *a, **kw: None,
        warn=lambda *a, **kw: None,
        error=lambda *a, **kw: None,
        debug=lambda *a, **kw: None,
    )
    node.is_playing = False
    node.current_sound = None
    node._play_token = 0
    node.play_thread = None
    node.playback_manager = MagicMock()
    node.state_pub = MagicMock()
    node.trigger_animations = False
    node.sounds = {}
    node.trigger_map = {}
    node.trigger_aliases = {"boop": "button_click"}
    node.sound_groups = {}
    node.trigger_priority = {
        "boop": 10,
        "button_click": 10,
        "thinking": 20,
    }
    for k, v in attrs.items():
        setattr(node, k, v)
    return node


class _SyncThread:
    """threading.Thread-замена: запускает target синхронно (для тестов)."""

    def __init__(self, target=None, args=(), kwargs=None, **kw):
        self._target = target
        self._args = args
        self._kwargs = kwargs or {}

    def start(self):
        assert self._target is not None, "_SyncThread requires a target"
        self._target(*self._args, **self._kwargs)


class TestTriggerPriority:
    """Issue #1328 — приоритеты триггеров в trigger_callback."""

    def test_thinking_interrupts_boop(self):
        """thinking (prio 20) прерывает бульк (button_click, prio 10)."""
        node = _make_node(
            is_playing=True,
            current_sound="button_click",
            sounds={"button_click": MagicMock(), "thinking": MagicMock()},
        )
        msg = MagicMock()
        msg.data = "thinking"

        with patch.object(node, "play_sound_thread") as mock_play, \
             patch.object(node, "select_sound", return_value="thinking") as mock_select, \
             patch("rob_box_voice.sound_node.threading.Thread", _SyncThread):
            node.trigger_callback(msg)

        mock_select.assert_called_once_with("thinking")
        node.playback_manager.stop_playback.assert_called_once_with("sound_node")
        mock_play.assert_called_once_with("thinking", "thinking")

    def test_boop_does_not_interrupt_thinking(self):
        """бульк (prio 10) НЕ прерывает thinking (prio 20)."""
        node = _make_node(
            is_playing=True,
            current_sound="thinking",
            sounds={"button_click": MagicMock()},
        )
        msg = MagicMock()
        msg.data = "boop"

        with patch.object(node, "play_sound_thread") as mock_play, \
             patch("rob_box_voice.sound_node.threading.Thread", _SyncThread):
            node.trigger_callback(msg)

        node.playback_manager.stop_playback.assert_not_called()
        mock_play.assert_not_called()
        assert node.play_thread is None

    def test_equal_priority_skips(self):
        """boop поверх boop (равный prio 10) — пропускаем."""
        node = _make_node(
            is_playing=True,
            current_sound="button_click",
            sounds={"button_click": MagicMock()},
        )
        msg = MagicMock()
        msg.data = "boop"

        with patch.object(node, "play_sound_thread") as mock_play, \
             patch("rob_box_voice.sound_node.threading.Thread", _SyncThread):
            node.trigger_callback(msg)

        mock_play.assert_not_called()
        assert node.play_thread is None

    def test_default_priority_skips(self):
        """Обычный звук (default prio 10) не прерывает текущий."""
        node = _make_node(
            is_playing=True,
            current_sound="button_click",
            sounds={"cute": MagicMock()},
        )
        msg = MagicMock()
        msg.data = "cute"

        with patch.object(node, "play_sound_thread") as mock_play, \
             patch("rob_box_voice.sound_node.threading.Thread", _SyncThread):
            node.trigger_callback(msg)

        mock_play.assert_not_called()
        assert node.play_thread is None

    def test_thinking_interrupts_any_default_sound(self):
        """thinking прерывает и НЕ-бульк звуки (любой default prio 10)."""
        node = _make_node(
            is_playing=True,
            current_sound="cute",
            sounds={"cute": MagicMock(), "thinking": MagicMock()},
        )
        msg = MagicMock()
        msg.data = "thinking"

        with patch.object(node, "play_sound_thread") as mock_play, \
             patch("rob_box_voice.sound_node.threading.Thread", _SyncThread):
            node.trigger_callback(msg)

        node.playback_manager.stop_playback.assert_called_once_with("sound_node")
        mock_play.assert_called_once()

    def test_not_playing_starts_thread(self):
        """Если ничего не играет — обычный старт потока, без stop_playback."""
        node = _make_node(sounds={"talk_1": MagicMock()})
        node.sound_groups = {"talk": ["talk_1"]}
        msg = MagicMock()
        msg.data = "talk"

        with patch.object(node, "play_sound_thread") as mock_play, \
             patch("random.choice", return_value="talk_1"), \
             patch("rob_box_voice.sound_node.threading.Thread", _SyncThread):
            node.trigger_callback(msg)

        node.playback_manager.stop_playback.assert_not_called()
        mock_play.assert_called_once_with("talk_1", "talk")
        assert node._play_token == 1

    def test_unknown_sound_ignored(self):
        """Неизвестный триггер — игнор, никаких потоков."""
        node = _make_node()
        msg = MagicMock()
        msg.data = "nonexistent_sound"

        with patch.object(node, "play_sound_thread") as mock_play:
            node.trigger_callback(msg)

        mock_play.assert_not_called()
        assert node.play_thread is None


class TestPlayTokenGuard:
    """Issue #1328 — токен поколения в play_sound_thread."""

    def test_token_guard_skips_flag_reset_for_old_thread(self):
        """Прерванный поток (token != current) НЕ сбрасывает флаги нового."""
        node = _make_node(
            is_playing=True,
            current_sound="thinking",
            sounds={"button_click": MagicMock()},
        )
        # Поток A захватил token=0 на старте; ПО ХОДУ воспроизведения пришёл
        # thinking → trigger_callback инкрементил _play_token до 1.
        node._play_token = 0

        mock_audio = MagicMock()
        mock_audio.frame_rate = 16000
        mock_audio.channels = 2
        mock_audio.get_array_of_samples.return_value = [0] * 100

        # Имитируем прерывание: пока поток A конвертирует аудио, токен меняется
        def _bump_token(*a, **kw):
            node._play_token += 1
            return [[0, 0]]

        # play_sound_thread использует np.array — замокаем, чтобы не тянуть numpy
        with patch.object(node.playback_manager, "play_audio", return_value=True), \
             patch("rob_box_voice.sound_node.np.array", side_effect=_bump_token), \
             patch("rob_box_voice.sound_node.sd.stop"), \
             patch("rob_box_voice.sound_node.time.sleep"):
            node.play_sound_thread("button_click", "boop")

        # Флаги НЕ сброшены — старый поток не затирает новый
        assert node.is_playing is True
        assert node.current_sound == "button_click"

    def test_token_matches_resets_flags(self):
        """Актуальный поток (token == current) сбрасывает флаги как раньше."""
        node = _make_node(sounds={"talk_1": MagicMock()})
        mock_audio = MagicMock()
        mock_audio.frame_rate = 16000
        mock_audio.channels = 2
        mock_audio.get_array_of_samples.return_value = [0] * 100
        node.sounds["talk_1"] = mock_audio

        with patch.object(node.playback_manager, "play_audio", return_value=True), \
             patch("rob_box_voice.sound_node.np.array", return_value=[[0, 0]]), \
             patch("rob_box_voice.sound_node.sd.stop"), \
             patch("rob_box_voice.sound_node.time.sleep"):
            node.play_sound_thread("talk_1", "talk")

        assert node.is_playing is False
        assert node.current_sound is None
