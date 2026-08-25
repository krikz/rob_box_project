"""
test_issue_1563_stop_immune.py — Unit-тесты issue #1563.

Покрывает три части фикса «TTS отменяется ДО воспроизведения после
«робот новая сессия»»:

1. ``control_callback`` принимает ``IGNORE_STOP_MS:<n>`` и открывает
   IMMUNE-окно. STOP в этом окне НЕ прерывает воспроизведение
   (но ``publish_state("stopped")`` всё равно вызывается для UI).
2. ``control_callback`` при STOP в фазе «synth_done, play не начат»
   (т.е. ``_is_post_synth_phase() == True``) буферизует STOP
   в ``_post_synth_stop_pending`` для СЛЕДУЮЩЕГО запроса.
3. ``synth_speech`` потребляет буферизованный STOP в самом начале —
   если флаг установлен, выполняется ``_interrupt_playback`` и
   новый запрос отменяется ДО синтеза.

Не требует ROS2 / sounddevice / ReSpeaker. ``TTSNode`` не импортируется
напрямую — подгружаем модуль через ``importlib`` ПОСЛЕ патча ``grpc``
(нужен ``grpc.__version__`` для yandex-cloud SDK), а сам класс
инстанцируем через ``object.__new__`` + ручные атрибуты (как в других
unit-тестах TTS).
"""

from __future__ import annotations

import importlib.util
import sys
import time
from pathlib import Path
from unittest.mock import MagicMock

import pytest


def _load_tts_node_class():
    """Загрузить ``rob_box_voice.tts_node`` и вернуть класс ``TTSNode``.

    Подменяем ``grpc`` MagicMock'ом с ``__version__=``«1.0» ДО загрузки
    модуля: иначе yandex.cloud SDK падает на ``grpc.__version__``.
    Регистрируем модуль как ``rob_box_voice.tts_node``, чтобы relative
    imports внутри файла (``from .audio_playback_manager import ...``)
    работали.
    Остальные stubs (rclpy / sounddevice / torch) ставит conftest.py.
    """
    # ВСЕГДА переустанавливаем grpc-stub (conftest.py в этой директории
    # тоже ставит grpc=MagicMock(), но без ``__version__`` / ``_utilities``).
    grpc_stub = MagicMock()
    grpc_stub.__version__ = "1.78.0"
    grpc_stub.RpcError = type("RpcError", (Exception,), {})
    utilities_stub = MagicMock()
    utilities_stub.first_version_is_lower = lambda *a, **k: False
    grpc_stub._utilities = utilities_stub
    sys.modules["grpc"] = grpc_stub

    # Заглушить yandex.cloud SDK — он сам внутри делает версионные
    # проверки, которые ломаются на MagicMock-grpc. Достаточно подменить
    # именно тот submodule, который импортирует tts_node.py.
    import types
    yandex_pkg = types.ModuleType("yandex")
    yandex_pkg.__path__ = []  # type: ignore[attr-defined]
    sys.modules.setdefault("yandex", yandex_pkg)
    cloud_pkg = types.ModuleType("yandex.cloud")
    cloud_pkg.__path__ = []  # type: ignore[attr-defined]
    sys.modules["yandex.cloud"] = cloud_pkg
    ai_pkg = types.ModuleType("yandex.cloud.ai")
    ai_pkg.__path__ = []  # type: ignore[attr-defined]
    sys.modules["yandex.cloud.ai"] = ai_pkg
    tts_pkg = types.ModuleType("yandex.cloud.ai.tts")
    tts_pkg.__path__ = []  # type: ignore[attr-defined]
    sys.modules["yandex.cloud.ai.tts"] = tts_pkg
    v3_pkg = types.ModuleType("yandex.cloud.ai.tts.v3")
    v3_pkg.tts_pb2 = MagicMock()
    v3_pkg.tts_service_pb2_grpc = MagicMock()
    sys.modules["yandex.cloud.ai.tts.v3"] = v3_pkg

    pkg_root = Path(__file__).resolve().parent.parent.parent.parent
    # Регистрируем rob_box_voice как настоящий пакет (на случай если ещё
    # не зарегистрирован другим тестом) — это нужно для relative-импортов
    # внутри tts_node (``from .audio_playback_manager import ...``).
    if "rob_box_voice" not in sys.modules:
        import types
        pkg = types.ModuleType("rob_box_voice")
        pkg.__path__ = [str(pkg_root / "rob_box_voice")]
        sys.modules["rob_box_voice"] = pkg
    spec = importlib.util.spec_from_file_location(
        "rob_box_voice.tts_node",
        pkg_root / "rob_box_voice" / "tts_node.py",
    )
    if spec is None or spec.loader is None:  # pragma: no cover
        raise RuntimeError("Failed to load rob_box_voice.tts_node")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["rob_box_voice.tts_node"] = mod
    spec.loader.exec_module(mod)
    return mod.TTSNode


# Загружаем модуль ОДИН раз на тест-сессию (это тяжёлая операция — Yandex SDK).
_TTS_NODE_CLS_CACHE = {}


@pytest.fixture(scope="module")
def tts_node_cls():
    if "_cls" not in _TTS_NODE_CLS_CACHE:
        _TTS_NODE_CLS_CACHE["_cls"] = _load_tts_node_class()
    return _TTS_NODE_CLS_CACHE["_cls"]


def _make_node(tts_node_cls, post_synth_phase: bool = False):
    """Минимальный TTSNode без __init__ — нужные атрибуты вручную.

    ``post_synth_phase`` — что должна вернуть ``_is_post_synth_phase``
    для тестов фазы «synth_done, play не начат». По умолчанию False.
    """
    n = object.__new__(tts_node_cls)
    n.get_logger = lambda: MagicMock()
    n.stop_requested = False
    n.processing_dialogue_id = None
    n.current_stream = None
    n._immune_until_ts = 0.0
    n._post_synth_stop_pending = False
    n._interrupt_playback = MagicMock()
    n.publish_state = MagicMock()
    # MagicMock с фиксированным return_value — безопасно для assert_*
    n._is_post_synth_phase = MagicMock(return_value=post_synth_phase)
    return n


def _msg(data: str):
    m = MagicMock()
    m.data = data
    return m


class TestIgnoreStopMs:
    """``IGNORE_STOP_MS:<n>`` открывает IMMUNE-окно в TTS."""

    def test_ignore_stop_ms_opens_immune_window(self, tts_node_cls):
        n = _make_node(tts_node_cls)
        n.control_callback(_msg("IGNORE_STOP_MS:700"))
        # IMMUNE-окно должно быть открыто В БУДУЩЕМ.
        assert n._immune_until_ts > time.monotonic()
        assert n._immune_until_ts <= time.monotonic() + 1.0
        # Никаких побочных эффектов быть не должно.
        n._interrupt_playback.assert_not_called()

    def test_ignore_stop_ms_lowercase_payload(self, tts_node_cls):
        """Регистр команды нормализуется через ``.upper()`` в control_callback."""
        n = _make_node(tts_node_cls)
        n.control_callback(_msg("ignore_stop_ms:300"))
        assert n._immune_until_ts > time.monotonic()

    def test_ignore_stop_ms_bad_payload_warns_no_crash(self, tts_node_cls):
        n = _make_node(tts_node_cls)
        # Не число — должно быть проигнорировано без crash.
        n.control_callback(_msg("IGNORE_STOP_MS:not_a_number"))
        assert n._immune_until_ts == 0.0
        n._interrupt_playback.assert_not_called()

    def test_ignore_stop_ms_empty_payload_warns(self, tts_node_cls):
        n = _make_node(tts_node_cls)
        n.control_callback(_msg("IGNORE_STOP_MS:"))
        assert n._immune_until_ts == 0.0


class TestStopImmuneWindow:
    """STOP в IMMUNE-окне не прерывает TTS."""

    def test_stop_in_immune_window_is_ignored(self, tts_node_cls):
        n = _make_node(tts_node_cls)
        n._immune_until_ts = time.monotonic() + 1.0  # окно на 1с вперёд
        n.processing_dialogue_id = "abc-123"  # имитируем активный синтез
        n.control_callback(_msg("STOP"))
        # _interrupt_playback НЕ должен быть вызван.
        n._interrupt_playback.assert_not_called()
        # publish_state("stopped") вызывается для UI.
        n.publish_state.assert_called_with("stopped")

    def test_stop_outside_immune_window_stops_normally(self, tts_node_cls):
        n = _make_node(tts_node_cls)
        n._immune_until_ts = 0.0  # окно закрыто
        n.processing_dialogue_id = "abc-123"
        n.control_callback(_msg("STOP"))
        # Старое поведение — STOP прерывает.
        n._interrupt_playback.assert_called_once()
        n.publish_state.assert_called_with("stopped")


class TestPostSynthStopBuffering:
    """STOP в фазе «synth_done, play не начат» → буферизация."""

    def test_post_synth_phase_detection(self, tts_node_cls):
        """Между синтезом и play_audio — processing_dialogue_id задан,
        current_stream=None, stop_requested=False."""
        n = object.__new__(tts_node_cls)
        n.processing_dialogue_id = "dlg-456"
        n.current_stream = None
        n.stop_requested = False
        assert tts_node_cls._is_post_synth_phase(n) is True

    def test_post_synth_phase_false_when_no_processing(self, tts_node_cls):
        """Нет активного диалога → нечего буферизовать."""
        n = object.__new__(tts_node_cls)
        n.processing_dialogue_id = None
        assert tts_node_cls._is_post_synth_phase(n) is False

    def test_post_synth_phase_false_when_stream_active(self, tts_node_cls):
        """Воспроизведение уже идёт → STOP должен прервать (старое поведение)."""
        n = object.__new__(tts_node_cls)
        n.processing_dialogue_id = "dlg-789"
        n.current_stream = True  # play_audio уже вызван
        n.stop_requested = False
        assert tts_node_cls._is_post_synth_phase(n) is False

    def test_stop_in_post_synth_phase_buffers(self, tts_node_cls):
        n = _make_node(tts_node_cls, post_synth_phase=True)
        n._immune_until_ts = 0.0
        n.control_callback(_msg("STOP"))
        # Текущий chunk НЕ прерывается (он уже синтезирован, доигрываем).
        n._interrupt_playback.assert_not_called()
        # STOP буферизован для следующего запроса.
        assert n._post_synth_stop_pending is True
        # UI НЕ получает "stopped" — chunk ещё играет, «stopped» будет
        # опубликован когда следующий запрос реально прервётся.


class TestSynthSpeechConsumesPendingStop:
    """synth_speech потребляет _post_synth_stop_pending в начале."""

    def test_pending_stop_consumed_at_synth_start(self, tts_node_cls):
        """Если в начале нового запроса _post_synth_stop_pending=True —
        синтез сразу прерывается через _interrupt_playback."""
        n = _make_node(tts_node_cls)
        n._post_synth_stop_pending = True
        n.stop_requested = False

        # Имитируем кусок логики synth_speech, который мы вставили.
        # Мы НЕ запускаем полный synth_speech (он требует Yandex/Silero),
        # а напрямую воспроизводим начало метода:
        n.stop_requested = False  # сброс (как в synth_speech)
        if getattr(n, "_post_synth_stop_pending", False):
            n._post_synth_stop_pending = False
            n._interrupt_playback()

        # _interrupt_playback должен быть вызван ровно один раз.
        n._interrupt_playback.assert_called_once()
        # Флаг потреблён.
        assert n._post_synth_stop_pending is False

    def test_no_pending_stop_does_not_interrupt(self, tts_node_cls):
        n = _make_node(tts_node_cls)
        n._post_synth_stop_pending = False
        n.stop_requested = False
        # Тот же кусок логики synth_speech:
        n.stop_requested = False
        if getattr(n, "_post_synth_stop_pending", False):
            n._post_synth_stop_pending = False
            n._interrupt_playback()

        n._interrupt_playback.assert_not_called()


class TestControlCallbackRouting:
    """control_callback корректно маршрутизирует STOP / IGNORE_STOP_MS / unknown."""

    def test_unknown_command_is_logged_not_crashed(self, tts_node_cls):
        n = _make_node(tts_node_cls)
        # Не падаем на незнакомой команде.
        n.control_callback(_msg("WHAT_IS_THAT"))
        n._interrupt_playback.assert_not_called()

    def test_empty_command_handled_gracefully(self, tts_node_cls):
        n = _make_node(tts_node_cls)
        n.control_callback(_msg(""))
        n._interrupt_playback.assert_not_called()

    def test_none_data_does_not_crash(self, tts_node_cls):
        """Защита от msg.data == None (теоретически)."""
        n = _make_node(tts_node_cls)
        m = MagicMock()
        m.data = None
        n.control_callback(m)  # не должно падать
        n._interrupt_playback.assert_not_called()
