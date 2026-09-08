"""Unit-тесты preview-synthesis канала (issue #2138.A.3).

Контракт: TTSNode должен уметь синтезировать «прослушиваемый образец» голоса
для picker'а оператора и отдавать его supervisor'у в виде base64-кодированных
аудио-байтов + метаданных формата — НЕ через _publish_headset_audio
(сырой int16 PCM, шлёт через AudioData), потому что ws_server/клиент preview'а
ждут закодированный контейнер (mp3/opus/wav), который decodeAudioData на
клиенте умеет декодировать (preview_audio_sink.ts §1).

Что покрывают тесты (RED-стадия перед правкой):

1. ``synthesize_preview()`` возвращает ``PreviewAudioResult`` dataclass с
   полями ``audio_bytes``, ``content_type``, ``sample_rate``, ``format_str``,
   ``duration_s`` — никаких side-effects (не дёргает _publish_audio,
   _publish_headset_audio, _synthesize_and_play, FIFO, ALSA).
2. Provider-chain вызывается в изоляции (моки) — пустой результат или ошибка
   → ``synthesize_preview`` бросает ``PreviewSynthesisError`` с понятной причиной.
3. Таймаут: если провайдер не ответил за N секунд — ``PreviewSynthesisTimeoutError``
   (а НЕ молчаливое зависание — иначе у оператора picker зависает навсегда).
4. Голос берётся из аргумента preview-запроса, активный голос НЕ меняется
   (preview не должно переключать personality voice — это уже валится
   в chain minimax).
5. Параметр preview_format объявлен в __init__ (mp3 по умолчанию, см. ADR-0055
   §tts_node sibling-headset).

Запуск:
    PYTHONPATH=src/rob_box_voice:src/rob_box_core:src/rob_box_harness:src/rob_box_llm \\
        pytest src/rob_box_voice/test/unit/tts/test_tts_node_preview.py -v
"""

from __future__ import annotations

import json
import sys
import threading
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_voice import tts_node as _tts_node_mod  # noqa: E402
from rob_box_voice.tts_node import TTSNode  # noqa: E402
from rob_box_llm import TTSAudio, TTSFormat  # noqa: E402

# _tts_node_mod уже импортирован ради side-effect (FakeNode rebinding).
del _tts_node_mod


# ── helpers ─────────────────────────────────────────────────────────────


class _CapturingPublisher:
    """Ловец любых String/AudioData-паблишей, которые preview НЕ должен трогать."""

    def __init__(self):
        self.messages: list = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


def _bare_node_with(mocks: dict) -> TTSNode:
    """Собрать bare-stub ``TTSNode`` с нужными моками.

    ``mocks`` ожидает ключи:
      audio_pub       — публикатор /voice/audio/speech (sink=speaker).
      avatar_audio_pub — публикатор /avatar/tts/audio (sink=headset).
      preview_audio_pub — публикатор /avatar/preview_voice/audio (новый).
      preview_result_pub — публикатор /avatar/preview_voice/result.
      preview_error_pub  — публикатор /avatar/preview_voice/error.
      finished_pub    — публикатор /voice/tts/finished.
      provider        — объект с методом ``synthesize(text, settings=...)``
                        возвращающим ``TTSAudio``.
    """
    n = object.__new__(TTSNode)
    n.get_logger = lambda: MagicMock()
    # Цепочка провайдеров в обычном _sap_run_provider_chain
    # работает через self._minimax_provider / self._yandex_provider /
    # self._silero_provider. Для preview-synth мы делаем отдельный код-путь
    # с явным provider'ом (см. контракт), поэтому в тестах передаём
    # провайдер явно — тестовая фокусировка на изоляции от основного пути.
    n.audio_pub = mocks.get("audio_pub", _CapturingPublisher())
    n._avatar_audio_pub = mocks.get("avatar_audio_pub", _CapturingPublisher())
    n._preview_audio_pub = mocks.get("preview_audio_pub", _CapturingPublisher())
    n._preview_result_pub = mocks.get("preview_result_pub", _CapturingPublisher())
    n._preview_error_pub = mocks.get("preview_error_pub", _CapturingPublisher())
    n.finished_pub = mocks.get("finished_pub", _CapturingPublisher())
    # preview НЕ должен:
    #   - менять active voice (см. self.minimax_voice)
    #   - идти в FIFO-gate (см. self._play_order_cond)
    #   - играть ALSA (см. self.play_audio)
    n.minimax_voice = "default-active-voice"
    n._play_order_cond = threading.Condition()
    n.play_audio = MagicMock()
    return n


def _fake_provider(audio_bytes: bytes, fmt: TTSFormat = TTSFormat.MP3, sr: int = 24000):
    """Поддельный TTSProvider с настраиваемым результатом."""

    async def _synthesize(text: str, *, settings=None) -> TTSAudio:
        return TTSAudio(samples=audio_bytes, sample_rate=sr, format=fmt)

    p = MagicMock()
    p.synthesize = _synthesize
    p.name = "fake"
    return p


# ── test 1: signature + return-shape ────────────────────────────────────


def test_synthesize_preview_method_exists():
    """``TTSNode.synthesize_preview(...)`` — публичный метод (RED)."""
    assert hasattr(TTSNode, "synthesize_preview"), (
        "synthesize_preview ещё не выделен из _synthesize_and_play"
    )


def test_synthesize_preview_returns_audio_bytes_not_publishes_to_avatar_audio():
    """Preview НЕ идёт в /avatar/tts/audio (int16 PCM) — клиент preview'а ждёт mp3/wav.

    Контракт preview_audio_sink.ts: AudioContext.decodeAudioData ожидает
    ЗАКОДИРОВАННЫЙ контейнер (mp3/opus/wav), не сырой PCM. Если мы пошлём
    int16-байты в /avatar/tts/audio, decodeAudioData вернёт ошибку и оператор
    увидит «preview failed» без звука — то же молчание, что и сейчас с
    "not_implemented", только сложнее диагностировать.
    """
    provider = _fake_provider(b"\\xff\\xfb\\x90\\x00" * 100, fmt=TTSFormat.MP3)
    mocks = {
        "preview_audio_pub": _CapturingPublisher(),
        "avatar_audio_pub": _CapturingPublisher(),
    }
    node = _bare_node_with(mocks)

    result = TTSNode.synthesize_preview(
        node,
        text="привет",
        voice="male-qn-qingse",
        provider=provider,
        timeout_s=5.0,
    )

    # Возврат — структура с байтами и метаданными.
    assert result.audio_bytes == b"\\xff\\xfb\\x90\\x00" * 100
    assert result.content_type == "audio/mpeg"
    assert result.sample_rate == 24000
    # НЕ дёрнули /avatar/tts/audio (PCM-канал).
    assert mocks["avatar_audio_pub"].messages == []
    # НЕ дёрнули /voice/audio/speech (динамики).
    assert mocks["preview_audio_pub"].messages == []  # синтез НЕ публикует, это работа caller'а


def test_synthesize_preview_does_not_change_active_voice():
    """Preview — sample-прослушивание, НЕ меняет активный голос личности.

    До фикса цепочка _synthesize_and_play могла через _sap_run_provider_chain
    переключить ``self.minimax_voice`` (fallback-провайдер подменяет голос на
    дефолтный этого провайдера). Для preview-варианта это недопустимо:
    оператор слушает образец и не должен услышать «голос сменился».
    """
    provider = _fake_provider(b"\\x00" * 16, fmt=TTSFormat.WAV)
    node = _bare_node_with({"preview_audio_pub": _CapturingPublisher()})
    node.minimax_voice = "ACTIVE_VOICE_BEFORE"

    TTSNode.synthesize_preview(
        node,
        text="hi",
        voice="ANOTHER_VOICE_FOR_PREVIEW",
        provider=provider,
        timeout_s=5.0,
    )

    assert node.minimax_voice == "ACTIVE_VOICE_BEFORE", (
        "preview изменил активный голос — регрессия!"
    )


# ── test 2: error-paths ─────────────────────────────────────────────────


def test_synthesize_preview_raises_on_provider_error_with_clear_reason():
    """Если провайдер падает — synthesize_preview бросает с понятной причиной.

    Это требование capability-honest: НЕ silent-fail. ws_server должен
    смочь отдать preview_voice_error{reason:"..."} оператору.
    """
    async def _bad_synthesize(text, *, settings=None):
        raise RuntimeError("minimax 502")

    bad = MagicMock()
    bad.synthesize = _bad_synthesize
    bad.name = "fake"

    node = _bare_node_with({})

    with pytest.raises(Exception) as ei:
        TTSNode.synthesize_preview(
            node,
            text="hi",
            voice="male-qn-qingse",
            provider=bad,
            timeout_s=2.0,
        )
    # Причина должна быть понятной (НЕ «Exception» молча).
    assert "minimax 502" in str(ei.value) or "synthesis" in str(ei.value).lower()


def test_synthesize_preview_times_out_instead_of_hanging():
    """Таймаут — обязательный параметр, иначе preview «висит вечно».

    Без таймаута сценарий «MiniMax не отвечает» делает picker нерабочим до
    рестарта supervisor'а (молчащий сценарий — самый коварный).
    """
    barrier = threading.Event()

    async def _slow_synthesize(text, *, settings=None):
        # Симулируем «висим» пока тест не отпустит barrier.
        barrier.wait(timeout=10.0)
        return TTSAudio(samples=b"\\x00", sample_rate=16000, format=TTSFormat.MP3)

    slow = MagicMock()
    slow.synthesize = _slow_synthesize
    slow.name = "fake"

    node = _bare_node_with({})

    raised = None
    try:
        TTSNode.synthesize_preview(
            node,
            text="hi",
            voice="male-qn-qingse",
            provider=slow,
            timeout_s=0.2,  # 200 мс — тест быстрый
        )
    except Exception as exc:
        raised = exc

    # Отпускаем «висящий» future, чтобы pytest не ругался на warning.
    barrier.set()

    assert raised is not None, "synthesize_preview не бросил по таймауту"
    assert "timeout" in str(raised).lower() or "timed out" in str(raised).lower()


# ── test 3: parameter declared ──────────────────────────────────────────


def test_tts_node_declares_preview_format_parameter():
    """``preview_format`` параметр — отдельный от minimax_format.

    По умолчанию mp3 (как минимум потому, что ws_server preview_audio_sink
    декодирует через WebAudio.decodeAudioData, а сырой PCM он не ест).
    """
    import inspect

    src = inspect.getsource(TTSNode.__init__)
    assert 'declare_parameter("preview_format"' in src, (
        "preview_format параметр не объявлен в tts_node.__init__"
    )
    # Дефолт — mp3 (или wav, но НЕ pcm, иначе decodeAudioData не справится).
    assert "preview_format" in src


# ── test 4: не дёргает _synthesize_and_play / FIFO / ALSA ──────────────


def test_synthesize_preview_does_not_invoke_full_synth_and_play():
    """Preview не идёт через _synthesize_and_play (FIFO+ALSA+metrics)."""
    provider = _fake_provider(b"\\x00" * 8)
    node = _bare_node_with({})
    node._synthesize_and_play = MagicMock(return_value=None)

    TTSNode.synthesize_preview(
        node,
        text="hi",
        voice="male-qn-qingse",
        provider=provider,
        timeout_s=5.0,
    )

    node._synthesize_and_play.assert_not_called()


def test_synthesize_preview_does_not_publish_voice_audio_speech():
    """Preview не публикует в /voice/audio/speech (динамики робота).

    Оператор слушает preview ТОЛЬКО в шлеме через preview_voice_audio.
    Динамики робота — отдельный канал (sink=\"speaker\"), preview
    туда не должен утекать.
    """
    mocks = {"audio_pub": _CapturingPublisher()}
    node = _bare_node_with(mocks)

    provider = _fake_provider(b"\\x00" * 8)
    TTSNode.synthesize_preview(
        node,
        text="hi",
        voice="male-qn-qingse",
        provider=provider,
        timeout_s=5.0,
    )

    assert mocks["audio_pub"].messages == []


# ── test 5: provider_state метрики — НЕ инкрементят preview-путь ──────


def test_synthesize_preview_does_not_publish_provider_state():
    """Preview — diagnostic-tool, не считаем в метриках synthesis_ok.

    Иначе picker-preview будет загрязнять счётчики «synthesis ok/fail»,
    которые используются для отслеживания качества основного голоса.
    """
    mocks = {"audio_pub": _CapturingPublisher()}
    node = _bare_node_with(mocks)
    node._publish_provider_state = MagicMock()

    provider = _fake_provider(b"\\x00" * 8)
    TTSNode.synthesize_preview(
        node,
        text="hi",
        voice="male-qn-qingse",
        provider=provider,
        timeout_s=5.0,
    )

    node._publish_provider_state.assert_not_called()
