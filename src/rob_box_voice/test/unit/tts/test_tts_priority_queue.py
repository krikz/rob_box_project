"""Юнит-тесты приоритетной очереди tts_node (issue #1996 / operator-agent 7a).

`tts_node` принимает поле ``priority`` (``operator`` | ``normal``) в JSON
``/voice/tts/request`` (и в общем с ним ``/voice/dialogue/response``
callback'е) и вставляет ``operator``-запрос СРАЗУ ЗА текущим активным
чанком (если он играет) или в голову FIFO (если нет). Активный чанк НЕ
прерывается — это явный инвариант 8a / DoD карточки.

Эти тесты НЕ поднимают ROS-стек: используется та же conftest, что и
``test_speech_id_arg_chain`` / ``test_silero_pitch_normalization``, и
обходится тяжёлый конструктор ``TTSNode.__init__`` через подкласс
``_BareNode``, который инициализирует только FIFO-gate-атрибуты (плюс
``_prefetch``/``_pregenerate_enabled`` для ADR-0056-стыковки).

В отличие от первой версии этого файла (WIP-ветка, коммит ``9654b585``),
тесты здесь зовут РЕАЛЬНЫЙ ``TTSNode._submit_synthesis`` (через
``_BareNode``, у которого ``_synthesis_executor`` — ``MagicMock``, так что
``executor.submit(...)`` не блокирует и не запускает воркер) вместо
ручной копии cond-lock-арифметики. Это устраняет риск дрейфа теста от
прод-кода, на который явно указывал докстринг оригинальной версии.

Покрытие (verbatim из DoD issue #1996 + стыковка с ADR-0056):
1. operator во время ACTIVE-чанка встаёт сразу за ним, без рестарта.
2. Без поля ``priority`` — старое поведение (normal FIFO).
3. Очередь пуста + operator → operator становится первым.
4. ``Стой!`` (active chunk in play_audio) не отменяется
   operator-вставкой (врезка ≠ прерывание, инвариант 8a).
5. Неизвестное / отсутствующее / не-строковое значение поля — ``normal``.
6. AST-страховка: priority не позиционный параметр
   ``_run_synthesis_worker`` (канонический arity, test_speech_id_arg_chain).
7. operator-приоритет триггерит ``cancel_pregen(reason="REPLACE-priority")``
   (ADR-0056 §3.5 п.4) — normal-приоритет его НЕ трогает.
"""
from __future__ import annotations

import sys
import threading
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_voice.tts_node import TTSNode, _normalize_tts_priority  # noqa: E402


class _BareNode(TTSNode):
    """``TTSNode`` без тяжёлого ``__init__``.

    Инициализирует ровно те атрибуты, которые трогает
    ``_submit_synthesis`` (FIFO-gate + ``_pending_seqs``/
    ``_play_active_seq`` для issue #1996 + ``_prefetch``/
    ``_pregenerate_enabled`` так, чтобы унаследованный
    ``cancel_pregen`` мог быть вызван без rclpy — см. ADR-0056).

    ``_synthesis_executor`` — ``MagicMock``: ``_submit_synthesis``
    зовёт ``executor.submit(...)`` по-настоящему, но т.к. это мок,
    воркер никогда реально не запускается и не блокирует тест.
    """

    def __init__(self) -> None:  # noqa: D401 — test stub, no super().__init__
        self._play_seq_counter = 0
        self._next_play_seq = 1
        self._play_order_cond = threading.Condition()
        self._pending_seqs = {}
        self._play_active_seq = None
        # Bounded-fanout state (BLK-9).
        self._synthesis_executor_shutdown = False
        self._synthesis_in_flight = 0
        self._synthesis_executor = MagicMock()
        self._synthesis_executor.submit.return_value = MagicMock()
        self._synthesis_slots = MagicMock()
        self._synthesis_slots.acquire.return_value = True
        # Issue #2003 / ADR-0056 — pregen engine off by default; keeps
        # the inherited ``cancel_pregen`` a safe no-op (returns 0)
        # unless a test explicitly wires it up.
        self._prefetch = None
        self._pregenerate_enabled = False
        # Logger — noop.
        self._logger = MagicMock()
        self.get_logger = MagicMock(return_value=self._logger)


@pytest.fixture
def node() -> _BareNode:
    """Чистая ``_BareNode`` для каждого теста."""
    return _BareNode()


def _submit(node: _BareNode, speech_id: str, priority=None) -> int:
    """Submit through the REAL ``_submit_synthesis`` and return the seq.

    A thin wrapper so tests read like the original mirrored-helper
    version, but every call goes through production code.
    ``fn``/``args`` are irrelevant here (the executor is a MagicMock),
    only the ``_play_order_cond`` seq-assignment side effect matters.
    """
    kwargs = {} if priority is None else {"priority": priority}
    node._submit_synthesis(lambda *a, **kw: None, speech_id, **kwargs)
    return node._pending_seqs[speech_id]


# ── 1. operator во время ACTIVE-чанка встаёт сразу за ним ─────────────────


def test_operator_inserted_behind_active_chunk(node):
    """Покрывает DoD #1: operator во время ACTIVE-чанка встаёт за ним.

    Сценарий:
      - normal(N1) submit → seq=1
      - normal(N2) submit → seq=2
      - (N1 «играет» → _play_active_seq = 1)
      - operator(O) submit → должен встать сразу за 1, т.е. seq=2,
        а N2 должен каскадно сдвинуться на 3 (FIFO-порядок normal'ов
        сохраняется — N2 всё ещё после O в очереди).
    """
    seq_n1 = _submit(node, "N1", priority="normal")
    seq_n2 = _submit(node, "N2", priority="normal")
    # Эмулируем «N1 сейчас играет»: ставим active_seq напрямую, как
    # делает ``_synthesize_and_play`` непосредственно перед
    # ``playback_manager.play_audio``.
    node._play_active_seq = seq_n1

    seq_op = _submit(node, "O", priority="operator")

    assert node._pending_seqs["N1"] == seq_n1, (
        f"Активный N1 не должен двигаться: был {seq_n1}, "
        f"стал {node._pending_seqs['N1']}"
    )
    assert seq_op == seq_n1 + 1, (
        f"operator должен встать сразу за активным: "
        f"ожидаем {seq_n1 + 1}, получили {seq_op}"
    )
    assert node._pending_seqs["N2"] == seq_n2 + 1, (
        f"N2 каскадно сдвигается на +1 при вклинивании operator: "
        f"был {seq_n2}, ожидаем {seq_n2 + 1}, "
        f"получили {node._pending_seqs['N2']}"
    )
    assert node._play_seq_counter == max(seq_op, node._pending_seqs["N2"])


# ── 2. regression: запрос без поля priority = normal FIFO ────────────────


def test_normal_request_no_priority_field(node):
    """Покрывает DoD #2: запрос без поля priority — старое FIFO."""
    seq_a = _submit(node, "A")  # priority вообще не передан
    seq_b = _submit(node, "B")
    seq_c = _submit(node, "C")
    assert (seq_a, seq_b, seq_c) == (1, 2, 3), (
        f"normal FIFO должен выдать 1, 2, 3 — получили {(seq_a, seq_b, seq_c)}"
    )
    assert node._play_active_seq is None
    assert node._pending_seqs == {"A": 1, "B": 2, "C": 3}


# ── 3. operator без активного чанка занимает голову FIFO ─────────────────


def test_operator_no_active_chunk_takes_head(node):
    """Покрывает DoD #3 (extension): пустая очередь → operator = head."""
    seq_op = _submit(node, "O", priority="operator")
    assert seq_op == 1, (
        f"operator на пустой очереди должен получить seq=1, получили {seq_op}"
    )
    assert node._pending_seqs == {"O": 1}
    assert node._play_seq_counter == 1


# ── 4. operator не прерывает уже играющий чанк (инвариант 8a) ────────────


def test_active_chunk_not_interrupted_by_operator(node):
    """Покрывает DoD: врезка ≠ прерывание, инвариант 8a.

    ``_submit_synthesis`` обязан ТОЛЬКО переназначить pending-seq'ы,
    не вызывать ``stop_requested=True`` и не сбрасывать
    ``_play_active_seq``.
    """
    seq_n1 = _submit(node, "N1", priority="normal")
    seq_n2 = _submit(node, "N2", priority="normal")
    node._play_active_seq = seq_n1
    node.stop_requested = False
    node.current_stream = True

    seq_op = _submit(node, "O", priority="operator")

    assert node._play_active_seq == seq_n1, (
        f"active_seq не должен меняться при вклинивании: "
        f"был {seq_n1}, стал {node._play_active_seq}"
    )
    assert node.stop_requested is False, (
        "operator не должен выставлять stop_requested=True"
    )
    assert node.current_stream is True, (
        "current_stream не должен сбрасываться — N1 продолжает играть"
    )
    assert seq_op == seq_n1 + 1
    assert node._pending_seqs["N2"] == seq_n2 + 1


# ── 5. fallback: неизвестное / отсутствующее значение = normal ───────────


def test_priority_field_default_normal(node):
    """Покрывает backward-compat: ``priority`` невалидное / отсутствует → normal."""
    seq_no_field = _submit(node, "X")
    seq_normal = _submit(node, "Y", priority="normal")
    seq_garbage = _submit(node, "Z", priority="bogus")
    seq_caps = _submit(node, "W", priority="OPERATOR")
    seq_none = _submit(node, "V", priority=None)

    assert (seq_no_field, seq_normal, seq_garbage, seq_caps, seq_none) == (
        1, 2, 3, 4, 5,
    ), "Все не-«operator» значения должны трактоваться как normal FIFO"
    assert node._pending_seqs == {"X": 1, "Y": 2, "Z": 3, "W": 4, "V": 5}


def test_normalize_tts_priority_whitelist():
    """Прямая проверка whitelist-хелпера (issue #1996).

    2-значный набор (``operator``/``normal``) — НЕ 3-значный
    ``{"normal", "operator", "personality"}`` из ADR-0056
    ``pregenerate.priority`` (другое поле, другое место в payload,
    описывает СЛЕДУЮЩИЙ чанк, а не текущий запрос). См. модульный
    докстринг ``_normalize_tts_priority`` в tts_node.py.
    """
    assert _normalize_tts_priority("operator") == "operator"
    assert _normalize_tts_priority("normal") == "normal"
    assert _normalize_tts_priority(None) == "normal"
    assert _normalize_tts_priority("") == "normal"
    assert _normalize_tts_priority("OPERATOR") == "normal"
    assert _normalize_tts_priority("personality") == "normal"
    assert _normalize_tts_priority(123) == "normal"


# ── 6. AST-regression: priority не добавлен как позиционный параметр ──────


def test_priority_not_positional_in_run_synthesis_worker():
    """Issue #1996 не должен ломать канонический 8-positional arity.

    Это страховка от регрессии test_speech_id_arg_chain:
    ``_run_synthesis_worker`` принимает ``**kwargs``; ``priority``
    вообще не извлекается внутри него (он нужен только
    ``_submit_synthesis`` для расстановки seq) — но страховка всё
    равно держит контракт explicit, на случай если кто-то решит
    прокинуть его дальше и случайно сделает позиционным.
    """
    import ast as _ast

    src_path = _PACKAGE_ROOT / "rob_box_voice" / "tts_node.py"
    tree = _ast.parse(src_path.read_text(encoding="utf-8"))

    def _find_func(name: str) -> _ast.FunctionDef:
        for node in tree.body:
            if isinstance(node, _ast.FunctionDef) and node.name == name:
                return node
            if isinstance(node, _ast.ClassDef):
                for m in node.body:
                    if isinstance(m, _ast.FunctionDef) and m.name == name:
                        return m
        raise LookupError(name)

    fn = _find_func("_run_synthesis_worker")
    positional_args = [a.arg for a in fn.args.args]
    assert "priority" not in positional_args, (
        f"priority не должен быть позиционным параметром "
        f"_run_synthesis_worker, иначе ломается test_speech_id_arg_chain. "
        f"Текущие args: {positional_args}"
    )
    all_kw = {a.arg for a in fn.args.kwonlyargs} | set(
        (fn.args.kwarg.arg if fn.args.kwarg else "",)
    )
    assert "kwargs" in all_kw or any(
        a.arg == "priority" for a in fn.args.kwonlyargs
    ), (
        f"priority должен приходить через **kwargs или быть kwonly. "
        f"kwonly={[a.arg for a in fn.args.kwonlyargs]}, "
        f"vararg={fn.args.kwarg.arg if fn.args.kwarg else None}"
    )


# ── 7. стыковка с ADR-0056: operator триггерит cancel_pregen ─────────────


def test_operator_priority_triggers_pregen_cancellation(node):
    """ADR-0056 §3.5 п.4 — REPLACE-семантика через priority-флаг 7a.

    Operator-приоритет переупорядочивает FIFO-gate (может сдвинуть уже
    запущенный спекулятивный pre-gen на другой слот), поэтому
    ``_submit_synthesis`` должен инвалидировать in-flight pre-gen через
    ``cancel_pregen(reason="REPLACE-priority")``. Проверяем спаем поверх
    унаследованного ``cancel_pregen`` (не трогаем его реальную async-
    машинерию — это уже покрыто ``test/unit/pregen/``).
    """
    node.cancel_pregen = MagicMock(return_value=0)

    _submit(node, "N1", priority="normal")
    node.cancel_pregen.assert_not_called()

    _submit(node, "O", priority="operator")
    node.cancel_pregen.assert_called_once_with(reason="REPLACE-priority")


def test_cancel_pregen_failure_does_not_block_operator_submit(node):
    """Best-effort: если cancel_pregen падает, operator всё равно встаёт в очередь.

    Инвариант из докстринга ``_cancel_pregen_for_priority_replace``:
    ошибка pre-gen-инвалидации не должна блокировать саму цель
    priority-очереди — вставку operator-реплики.
    """
    node.cancel_pregen = MagicMock(side_effect=RuntimeError("boom"))

    seq_op = _submit(node, "O", priority="operator")

    assert seq_op == 1
    assert node._pending_seqs == {"O": 1}
