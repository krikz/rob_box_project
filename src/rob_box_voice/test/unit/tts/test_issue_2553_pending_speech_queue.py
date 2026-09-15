"""Юнит-тесты для issue #2553 — babble-retry / DJ-overlap pending queue.

Live DJ-сет (Vision Pi, develop-образ, 2026-09-15 round2) за 30 минут
генерировал **14 STOP-команд TTS** (tts_node-5 WARN «STOP command received»).
Юзер слышал обрывки фраз: «Сет запу-» → обрыв → «Новый бит в стиле фанк» →
обрыв. Каждые ~20 секунд babble-retry (dialogue_node `_check_babble_and_retry`
→ `_dispatch_turn` → новый `_publish_response`) синхронно диспатчит НОВЫЙ
speak во время активного TTS-воспроизведения.

Фикс (issue #2553): tts_node теперь буферизует новый chunk в
``_pending_speech_queue``, если при приёме ``dialogue_callback`` уже
играет чанк из ДРУГОГО ``batch_id``. После ``batch_complete`` активного
batch'а (``_publish_tts_finished`` side-channel) очередь дренируется и
отложенные chunk'ы идут в обычный FIFO-gate — без обрыва текущей речи.

Тесты НЕ поднимают ROS-стек: тот же conftest + ``_BareNode`` подход, что и
``test_tts_priority_queue``. ``_BareNode`` обходит тяжёлый ``TTSNode.__init__``
через object.__new__ + ручное выставление FIFO-gate-атрибутов.

Покрытие (verbatim из issue DoD):
1. active batch другой batch_id → новый chunk уходит в очередь,
   ``_submit_synthesis`` НЕ зовётся.
2. active batch тот же batch_id → submit идёт по-старому (FIFO внутри
   batch'а сохраняется).
3. _play_active_seq=None (TTS idle) → submit идёт по-старому.
4. legacy single-chunk без batch_id (batch_id=None с обеих сторон) →
   submit идёт по-старому.
5. operator-приоритет НЕ откладывается (issue #1996 invariant 8a).
6. ``_drain_pending_speech_queue`` достаёт chunk'и и переотправляет их
   в FIFO через ``_submit_synthesis``.
7. ``_publish_tts_finished`` c batch_complete дренит очередь.
8. Очередь переполняется (MAX=8) — лишние chunk'ы дропаются с warning.
9. AST-страховка: новые методы экспортированы на классе (а не только
   в instance __dict__).
"""

from __future__ import annotations

import json
import sys
import threading
from pathlib import Path
from unittest.mock import MagicMock

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_voice.tts_node import TTSNode  # noqa: E402

# ── Вспомогательные мини-классы ────────────────────────────────────────────


class _CapturingLogger:
    """Простой логгер, чтобы ``get_logger().info/warn/...`` не падали."""

    def __init__(self) -> None:
        self.info_msgs: list = []
        self.warn_msgs: list = []
        self.error_msgs: list = []
        self.debug_msgs: list = []

    def info(self, msg):
        self.info_msgs.append(msg)

    def warn(self, msg):
        self.warn_msgs.append(msg)

    def warning(self, msg):
        self.warn_msgs.append(msg)

    def error(self, msg):
        self.error_msgs.append(msg)

    def debug(self, msg):
        self.debug_msgs.append(msg)


class _CapturingPublisher:
    def __init__(self) -> None:
        self.messages: list = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


def _make_node() -> TTSNode:
    """Минимальный TTSNode c FIFO-gate атрибутами + issue #2553 state.

    Использует ``object.__new__`` чтобы обойти ``TTSNode.__init__``
    (который инициализирует rclpy-инфраструктуру и другие зависимости).
    Все нужные атрибуты выставляются руками.
    """
    n = object.__new__(TTSNode)
    logger = _CapturingLogger()
    n._logger = logger
    n.get_logger = lambda: logger
    n.finished_pub = _CapturingPublisher()
    n.batch_complete_pub = _CapturingPublisher()
    n._submit_synthesis = MagicMock()

    # FIFO-gate state (issue #1996) + новая очередь (issue #2553).
    n._play_seq_counter = 0
    n._next_play_seq = 1
    n._play_order_cond = threading.Condition()
    n._pending_seqs = {}
    n._play_active_seq = None
    n._active_batch_id = None
    n._pending_speech_queue = []

    # Pre-gen off (issue #2003) — claim_pregen вернёт None, не упадёт.
    n._prefetch = None
    n._pregenerate_enabled = False

    return n


def _msg(payload: dict):
    """Собрать fake ROS String-сообщение."""
    m = MagicMock()
    m.data = json.dumps(payload, ensure_ascii=False)
    return m


def _finished_payloads(node) -> list:
    return [json.loads(m.data) for m in node.finished_pub.messages]


# ── 1. active batch другой batch_id → chunk в очередь, _submit_synthesis НЕ зовётся


def test_new_chunk_from_different_batch_is_queued():
    """AC #1 — babble-retry поверх DJ-сета: новый chunk отложен в очередь.

    Setup:
      - ``_play_active_seq`` = 1, ``_active_batch_id`` = «dj-set-N»
        (играет turn #N).
      - dialogue_callback получает chunk из batch_id=«babble-retry-M».

    Expect:
      - ``_submit_synthesis`` НЕ зовётся.
      - ``_pending_speech_queue`` имеет 1 элемент с правильными полями.
      - ``finished_pub`` получает finished с success=True и queued=True
        (НЕ batch_complete — реальное завершение придёт после drain).
    """
    node = _make_node()
    node._play_active_seq = 1
    node._active_batch_id = "dj-set-N"
    node.dialogue_callback(
        _msg(
            {
                "ssml": "<speak>Новый бит в стиле фанк</speak>",
                "speech_id": "babble-retry-001",
                "batch_id": "babble-retry-M",
                "batch_index": 1,
                "batch_total": 1,
                "priority": "normal",
            }
        )
    )

    # Chunk не ушёл в submit — лежит в очереди.
    node._submit_synthesis.assert_not_called()
    assert len(node._pending_speech_queue) == 1
    queued = node._pending_speech_queue[0]
    assert queued["speech_id"] == "babble-retry-001"
    assert queued["batch_id"] == "babble-retry-M"
    assert queued["text"] == "Новый бит в стиле фанк"
    assert queued["priority"] == "normal"

    # Upstream (mcp_server / dialogue_node) получил finished,
    # чтобы ``speak_text`` не висел в ожидании.
    # ``queued=True`` гарантирует, что ``batch_complete`` side-channel
    # НЕ сработал (см. описание ``_publish_tts_finished(queued=True)``).
    finished = _finished_payloads(node)
    assert len(finished) == 1
    assert finished[0]["speech_id"] == "babble-retry-001"
    assert finished[0]["success"] is True
    assert finished[0]["queued"] is True
    assert finished[0]["batch_id"] == "babble-retry-M"
    # batch_complete НЕ публиковался — иначе dialogue_node триггерил бы
    # music_cleanup раньше времени.
    assert len(node.batch_complete_pub.messages) == 0


# ── 2. active batch тот же batch_id → submit идёт по-старому (FIFO внутри batch)


def test_new_chunk_same_batch_is_not_queued():
    """AC #2 — chunk ТОГО ЖЕ batch (продолжение turn #N) идёт в FIFO.

    Если babble-retry порождает ТЕКСТ внутри того же turn #N (новый
    speak_text без смены batch_id), chunk НЕ должен попадать в очередь —
    это нормальное продолжение, не overlap.
    """
    node = _make_node()
    node._play_active_seq = 1
    node._active_batch_id = "turn-N"
    node.dialogue_callback(
        _msg(
            {
                "ssml": "<speak>И ещё куплет</speak>",
                "speech_id": "turn-N-chunk-2",
                "batch_id": "turn-N",
                "batch_index": 2,
                "batch_total": 3,
                "priority": "normal",
            }
        )
    )

    # Submit прошёл — chunk ушёл в FIFO-gate.
    node._submit_synthesis.assert_called_once()
    # Очередь пуста.
    assert node._pending_speech_queue == []


# ── 3. TTS idle (_play_active_seq=None) → submit идёт по-старому


def test_no_active_chunk_means_normal_submit():
    """AC #3 — TTS простаивает, новый chunk идёт через submit."""
    node = _make_node()
    assert node._play_active_seq is None
    assert node._active_batch_id is None
    node.dialogue_callback(
        _msg(
            {
                "ssml": "<speak>Сет запущен</speak>",
                "speech_id": "first-001",
                "batch_id": "first-batch",
                "batch_index": 1,
                "batch_total": 1,
                "priority": "normal",
            }
        )
    )

    node._submit_synthesis.assert_called_once()
    assert node._pending_speech_queue == []


# ── 4. legacy single-chunk без batch_id — пускаем по-старому


def test_legacy_no_batch_id_passes_through():
    """AC #4 — legacy single-chunk (batch_id=None) не должен попадать в очередь.

    До issue #2553 такое работало: идёт через FIFO-gate и играется в
    порядке приёма. Мы НЕ должны ломать back-compat даже когда
    ``_play_active_seq`` != None.
    """
    node = _make_node()
    node._play_active_seq = 1
    node._active_batch_id = "first-batch"  # active batch с batch_id есть
    node.dialogue_callback(
        _msg(
            {
                "ssml": "<speak>Хорошо</speak>",
                "speech_id": "legacy-001",
                # batch_id отсутствует — back-compat
            }
        )
    )

    # Submit прошёл (FIFO-gate упорядочит legacy по seq).
    node._submit_synthesis.assert_called_once()
    assert node._pending_speech_queue == []


# ── 5. operator-приоритет НЕ откладывается (issue #1996 invariant 8a)


def test_operator_priority_is_not_queued():
    """AC #5 — operator-приоритет врезается сразу за активным (НЕ очередь).

    Issue #1996 invariant 8a: «врезка ≠ прерывание — оператор ВСЕГДА
    становится сразу за активным chunk'ом». Issue #2553 НЕ должен
    нарушать это — operator идёт в обычный FIFO-gate через
    ``_submit_synthesis``, где ``_assign_priority_play_seq`` сделает
    своё дело.
    """
    node = _make_node()
    node._play_active_seq = 1
    node._active_batch_id = "dj-set-N"
    node.dialogue_callback(
        _msg(
            {
                "ssml": "<speak>Стоп, оператор здесь</speak>",
                "speech_id": "op-001",
                "batch_id": "operator-batch",
                "batch_index": 1,
                "batch_total": 1,
                "priority": "operator",
            }
        )
    )

    node._submit_synthesis.assert_called_once()
    assert node._pending_speech_queue == []


# ── 6. _drain_pending_speech_queue достаёт chunk'и и re-submit'ит их


def test_drain_submits_queued_chunks_in_fifo():
    """AC #6 — drain переотправляет chunk'и в FIFO-gate.

    setup: 2 chunk'а в очереди.
    drain → оба chunk'а уходят через _submit_synthesis в том же порядке.
    """
    node = _make_node()
    node._pending_speech_queue = [
        {
            "speech_id": "babble-1",
            "batch_id": "babble-batch",
            "batch_index": 1,
            "batch_total": 1,
            "ssml": "<speak>Первый</speak>",
            "text": "Первый",
            "dialogue_id": None,
            "ssml_attributes": None,
            "voice": None,
            "language": None,
            "priority": "normal",
        },
        {
            "speech_id": "babble-2",
            "batch_id": "babble-batch",
            "batch_index": 1,
            "batch_total": 1,
            "ssml": "<speak>Второй</speak>",
            "text": "Второй",
            "dialogue_id": None,
            "ssml_attributes": None,
            "voice": None,
            "language": None,
            "priority": "normal",
        },
    ]

    drained = node._drain_pending_speech_queue(reason="test")

    assert drained == 2
    assert node._pending_speech_queue == []
    # Два submit'а в порядке FIFO.
    assert node._submit_synthesis.call_count == 2
    submit_calls = node._submit_synthesis.call_args_list
    assert submit_calls[0].args[1] == "babble-1"  # speech_id
    assert submit_calls[1].args[1] == "babble-2"


def test_drain_empty_queue_is_noop():
    """AC #6.1 — drain пустой очереди: 0 submit'ов, return 0."""
    node = _make_node()
    drained = node._drain_pending_speech_queue(reason="test")
    assert drained == 0
    node._submit_synthesis.assert_not_called()


def test_drain_handles_submit_failure_gracefully():
    """AC #6.2 — submit падает на одном chunk'е: drain не падает целиком.

    pending[0] = "ok" → submit успех → success_count = 1.
    pending[1] = "boom" → submit бросает → warning, НЕ success.
    Возвращаемое значение = число УСПЕШНО отправленных.
    """
    node = _make_node()
    node._pending_speech_queue = [
        {
            "speech_id": "ok",
            "batch_id": "b",
            "batch_index": 1,
            "batch_total": 1,
            "ssml": "<s/>",
            "text": "ok",
            "dialogue_id": None,
            "ssml_attributes": None,
            "voice": None,
            "language": None,
            "priority": "normal",
        },
        {
            "speech_id": "boom",
            "batch_id": "b",
            "batch_index": 1,
            "batch_total": 1,
            "ssml": "<s/>",
            "text": "boom",
            "dialogue_id": None,
            "ssml_attributes": None,
            "voice": None,
            "language": None,
            "priority": "normal",
        },
    ]
    side_effects = [None, RuntimeError("synthesis dead")]
    node._submit_synthesis.side_effect = side_effects

    drained = node._drain_pending_speech_queue(reason="test")

    assert drained == 1  # 1 успешно отправлен, 1 упал → success_count = 1
    assert node._submit_synthesis.call_count == 2
    # В логе было предупреждение про «boom».
    assert any("boom" in m for m in node._logger.warn_msgs)


# ── 7. _publish_tts_finished с batch_complete дренит очередь


def test_publish_tts_finished_drains_on_batch_complete():
    """AC #7 — после batch_complete активного batch'а очередь дренится.

    ``_publish_tts_finished`` с batch_index==batch_total вызывает
    ``_drain_pending_speech_queue``. Очередь непуста → drain → 1 submit.
    """
    node = _make_node()
    node._pending_speech_queue = [
        {
            "speech_id": "babble-q",
            "batch_id": "babble-batch",
            "batch_index": 1,
            "batch_total": 1,
            "ssml": "<s/>",
            "text": "Отложенный",
            "dialogue_id": None,
            "ssml_attributes": None,
            "voice": None,
            "language": None,
            "priority": "normal",
        }
    ]

    node._publish_tts_finished(
        "active-last-chunk",
        success=True,
        batch_id="active-batch",
        batch_index=2,
        batch_total=2,
        batch_started_at=0.0,
        dialogue_id=None,
    )

    # Очередь пуста — дренировали.
    assert node._pending_speech_queue == []
    # Submit был (re-submit pending chunk'а).
    node._submit_synthesis.assert_called_once()


def test_publish_tts_finished_no_drain_when_not_batch_complete():
    """AC #7.1 — finished НЕ последнего chunk'а: drain НЕ зовётся.

    ``batch_index < batch_total`` → обычный finished, очередь не трогаем.
    """
    node = _make_node()
    node._pending_speech_queue = [
        {
            "speech_id": "stay",
            "batch_id": "b",
            "batch_index": 1,
            "batch_total": 1,
            "ssml": "<s/>",
            "text": "x",
            "dialogue_id": None,
            "ssml_attributes": None,
            "voice": None,
            "language": None,
            "priority": "normal",
        }
    ]

    node._publish_tts_finished(
        "active-mid-chunk",
        success=True,
        batch_id="active-batch",
        batch_index=1,
        batch_total=3,
        batch_started_at=0.0,
        dialogue_id=None,
    )

    # Очередь не дренировалась.
    assert len(node._pending_speech_queue) == 1
    node._submit_synthesis.assert_not_called()


# ── 8. Очередь переполняется — лишние chunk'ы дропаются


def test_pending_queue_overflow_drops_with_warning():
    """AC #8 — MAX=8 chunk'ов в очереди: 9-й дропается с warning.

    Защита от бесконечного роста памяти если upstream спамит
    (dialogue_node / DJModeController сломался и шлёт без остановки).
    """
    node = _make_node()
    node._play_active_seq = 1
    node._active_batch_id = "dj-set-N"
    # Заполняем очередь до MAX.
    node._pending_speech_queue = [
        {"speech_id": f"old-{i}", "batch_id": "x"} for i in range(8)
    ]

    # 9-й chunk — должен дропнуться.
    node.dialogue_callback(
        _msg(
            {
                "ssml": "<speak>overflow</speak>",
                "speech_id": "overflow-9",
                "batch_id": "spam",
                "batch_index": 1,
                "batch_total": 1,
                "priority": "normal",
            }
        )
    )

    # Submit НЕ вызван (overflow → return раньше).
    node._submit_synthesis.assert_not_called()
    # Очередь всё ещё дли 8 — overflow chunk в неё НЕ попал.
    assert len(node._pending_speech_queue) == 8
    # В логе — warning про overflow.
    assert any("переполнена" in m for m in node._logger.warn_msgs)
    # upstream получил finished с success=False + error=queue_overflow,
    # queued=True (НЕ триггерит batch_complete / drain).
    finished = _finished_payloads(node)
    assert len(finished) == 1
    assert finished[0]["success"] is False
    assert finished[0]["error"] == "pending_queue_overflow"
    assert finished[0]["queued"] is True
    # batch_complete НЕ публиковался (queued=True подавляет).
    assert len(node.batch_complete_pub.messages) == 0


# ── 9. AST-страховка: новые методы живут в TTSNode (no orphan import)


def test_pending_queue_methods_are_on_tts_node_class():
    """AC #9 — методы доступны на классе (не только инстансе через __dict__).

    Другая нода может делать ``TTSNode._enqueue_pending_speech(node, ...)``
    через getattr-guard; убедимся, что класс их экспортирует явно.
    """
    assert hasattr(TTSNode, "_enqueue_pending_speech")
    assert hasattr(TTSNode, "_drain_pending_speech_queue")
    assert hasattr(TTSNode, "_PENDING_SPEECH_QUEUE_MAX")
    assert TTSNode._PENDING_SPEECH_QUEUE_MAX == 8
