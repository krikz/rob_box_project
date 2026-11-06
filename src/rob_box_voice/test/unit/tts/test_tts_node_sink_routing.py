"""Issue #2198 / #4252 — единый вход синтезатора ``dialogue_callback`` с полем ``sink``.

Тесты покрывают инвариант: **приёмник реплики (динамики / шлем / preview) задаётся полем ``sink`` чанка**.

* ``dialogue_callback`` (чанк от владельца речи, ``SpeechRequest``) — единственный вход синтеза.
  ``sink`` ∈ {``"speaker"`` (default), ``"headset"`` → ``_on_headset_chunk``, ``"preview"`` →
  ``_on_avatar_tts_request_preview``}.
* ``/avatar/tts/request`` (``_on_avatar_tts_request``) — только образец голоса (``sink="preview"``); реплики в шлем
  туда не ходят с #4252 ч.2, ложного WARNING «DEPRECATED» больше нет.
* Любой не-валидный ``sink`` → DROP + WARN.

Запуск:
    PYTHONPATH=src/rob_box_voice:src/rob_box_core:src/rob_box_harness:src/rob_box_llm \\
        pytest src/rob_box_voice/test/unit/tts/test_tts_node_sink_routing.py -v
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import FinalsSpy, _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_voice import tts_node as _tts_node_mod  # noqa: E402
from rob_box_voice.tts_node import TTSNode  # noqa: E402

del _tts_node_mod


# ── helpers ──────────────────────────────────────────────────────────────


class _CapturingPublisher:
    """Минимальный ловец String/AudioData паблишей."""

    def __init__(self):
        self.messages: list = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


def _make_voice_node() -> TTSNode:
    """Bare-stub для ``dialogue_callback``.

    Достаточно для проверки маршрутизации по ``sink``:
      - ``get_logger`` (MagicMock) — для WARN/INFO;
      - ``_speech`` (владелец-шпион финалов) / ``_avatar_tts_error_pub`` — для _on_avatar_tts_request;
      - ``_submit_synthesis`` (MagicMock) — для проверки «пошёл ли синтез»;
      - ``_on_headset_chunk`` (MagicMock) — для делегации в headset-путь;
      - ``_on_avatar_tts_request_preview`` (MagicMock) — для делегации в preview-путь.

    ``logger`` — ОДИН MagicMock на ноду: иначе каждый вызов ``self.get_logger()``
    возвращал бы свежий mock, и тест на ``logger.warn.called`` не видел бы
    сообщений, залогированных внутри callback'а.
    """
    logger = MagicMock()
    n = object.__new__(TTSNode)
    n.get_logger = lambda: logger
    n._test_logger = logger  # для тестовых assert'ов
    n._speech = FinalsSpy()
    n._avatar_tts_error_pub = _CapturingPublisher()
    n._submit_synthesis = MagicMock()
    n._on_headset_chunk = MagicMock()
    n._on_avatar_tts_request_preview = MagicMock()
    n.current_speech_id = None
    n._interrupt_playback = MagicMock()
    return n


def _msg(payload: dict):
    m = MagicMock()
    m.data = json.dumps(payload, ensure_ascii=False)
    return m


# ── 1. ``/voice/tts/request`` принимает поле ``sink`` (issue #2198) ──────


def test_voice_tts_request_without_sink_defaults_to_speaker_path():
    """Без поля sink — старый путь в динамики (backward-compat).

    Проверяем, что ``_submit_synthesis`` НЕ вызван (bare-stub без
    полной инициализации), и НЕ дёрнуты пути шлема и образца: ``_on_headset_chunk``
    и ``_on_avatar_tts_request_preview`` не вызывались.
    """
    node = _make_voice_node()
    node.dialogue_callback(_msg({"text": "привет", "text": "привет"}))

    node._on_headset_chunk.assert_not_called()
    node._on_avatar_tts_request_preview.assert_not_called()


def test_voice_tts_request_with_speaker_routes_to_speakers():
    """``sink='speaker'`` → старый путь в динамики (НЕ headset/preview)."""
    node = _make_voice_node()
    node.dialogue_callback(
        _msg(
            {
                "text": "привет",
                "text": "привет",
                "sink": "speaker",
            }
        )
    )

    node._on_headset_chunk.assert_not_called()
    node._on_avatar_tts_request_preview.assert_not_called()


def test_voice_tts_request_with_headset_delegates_to_headset_chunk():
    """``sink='headset'`` → ``_on_headset_chunk`` с разобранным чанком (#4252); ``/avatar/tts/request`` не участвует."""
    node = _make_voice_node()
    node._on_avatar_tts_request = MagicMock()  # type: ignore[assignment]
    node.dialogue_callback(
        _msg({"text": "привет", "sink": "headset", "request_id": "req-from-voice-with-sink-headset"})
    )

    node._on_headset_chunk.assert_called_once()
    assert node._on_headset_chunk.call_args.args[0]["request_id"] == "req-from-voice-with-sink-headset"
    node._on_avatar_tts_request.assert_not_called()
    node._submit_synthesis.assert_not_called()


def test_voice_tts_request_with_preview_delegates_to_preview_path():
    """``sink='preview'`` → ``_on_avatar_tts_request_preview`` (синхронный путь ADR-0079), не шлем."""
    node = _make_voice_node()
    node.dialogue_callback(
        _msg({"request_id": "req-from-voice-with-sink-preview", "text": "привет", "sink": "preview",
              "voice": "male-qn-qingse"})
    )

    node._on_avatar_tts_request_preview.assert_called_once()
    assert node._on_avatar_tts_request_preview.call_args.args[0]["voice"] == "male-qn-qingse"
    node._on_headset_chunk.assert_not_called()
    node._submit_synthesis.assert_not_called()


def test_voice_tts_request_unknown_sink_drops_with_warning():
    """Любой другой sink (не speaker/headset/preview) → DROP + WARN."""
    node = _make_voice_node()
    logger = node._test_logger
    node.dialogue_callback(
        _msg(
            {
                "text": "привет",
                "text": "привет",
                "sink": "matrix",  # неизвестный приёмник
            }
        )
    )
    # DROP → _submit_synthesis, _on_headset_chunk не вызваны.
    node._submit_synthesis.assert_not_called()
    node._on_headset_chunk.assert_not_called()
    # WARN залогирован.
    assert logger.warn.called, (
        f"expected WARN для unknown sink, got calls: "
        f"{[str(c) for c, _ in logger.warn.call_args_list]}"
    )


def test_voice_tts_request_bad_json_does_not_crash():
    """Битый JSON → warning + DROP, без падения."""
    node = _make_voice_node()
    bad = MagicMock()
    bad.data = "not-valid-json{"
    # Не должно упасть.
    node.dialogue_callback(bad)
    node._submit_synthesis.assert_not_called()


# ── 2. ``/avatar/tts/request`` — только образец голоса (#4252 ч.2) ────


def test_avatar_tts_request_preview_goes_to_preview_without_deprecated_warning():
    """``/avatar/tts/request`` с ``sink='preview'`` — штатный путь образца: без WARNING «DEPRECATED»."""
    node = _make_voice_node()
    node._on_avatar_tts_request(_msg({"request_id": "req-preview", "text": "Привет", "sink": "preview"}))

    node._on_avatar_tts_request_preview.assert_called_once()
    warn_calls = [str(c) for c in node._test_logger.warn.call_args_list]
    assert not any("deprecated" in c.lower() for c in warn_calls), warn_calls


# ── 3. AST-инвариант: ``sink`` switch живёт ровно в одном месте ────────


def test_dialogue_callback_does_not_read_sink_directly():
    """AST-инвариант issue #2198 (ADR-0021 CC-budget): ``dialogue_callback``
    НЕ читает поле ``sink`` из payload напрямую — это работа helper'а
    ``_resolve_voice_tts_sink`` (primary chokepoint). Сам
    ``dialogue_callback`` лишь делегирует в helper и читает результат.

    Если кто-то снова впишет ``chunk_data["sink"]`` или
    ``chunk_data.get("sink", ...)`` прямо в ``dialogue_callback`` —
    тест упадёт, потому что это раздувает CC (baseline=28).
    """
    import ast
    import inspect
    import textwrap

    src = textwrap.dedent(inspect.getsource(TTSNode.dialogue_callback))
    tree = ast.parse(src)

    sink_field_reads: list[str] = []
    for node in ast.walk(tree):
        # 1. chunk_data["sink"] → ast.Subscript
        if isinstance(node, ast.Subscript):
            sl = node.slice
            if (
                isinstance(sl, ast.Constant)
                and sl.value == "sink"
                and isinstance(node.value, ast.Name)
                and node.value.id in ("chunk_data", "msg")
            ):
                sink_field_reads.append(node.value.id)
                continue
        # 2. chunk_data.get("sink", default) → ast.Call на Attribute
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            if node.func.attr != "get":
                continue
            if not node.args:
                continue
            key = node.args[0]
            if (
                isinstance(key, ast.Constant)
                and key.value == "sink"
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id in ("chunk_data", "msg")
            ):
                sink_field_reads.append(node.func.value.id)

    assert sink_field_reads == [], (
        "dialogue_callback НЕ должен читать chunk_data['sink'] / "
        "chunk_data.get('sink', ...) напрямую — это работа "
        "_resolve_voice_tts_sink (ADR-0021 CC-budget). "
        f"Найденные reads: {sink_field_reads}"
    )


def test_dialogue_callback_delegates_to_resolve_helper():
    """``dialogue_callback`` обязан дёргать ``_resolve_voice_tts_sink``
    для маршрутизации sink'а — это декомпозиция chokepoint'а в helper.
    """
    import ast
    import inspect
    import textwrap

    src = textwrap.dedent(inspect.getsource(TTSNode.dialogue_callback))
    tree = ast.parse(src)
    called_names: set[str] = set()

    for ast_node in ast.walk(tree):
        if isinstance(ast_node, ast.Call) and isinstance(
            ast_node.func, ast.Attribute
        ):
            if (
                isinstance(ast_node.func.value, ast.Name)
                and ast_node.func.value.id == "self"
            ):
                called_names.add(ast_node.func.attr)

    assert "_resolve_voice_tts_sink" in called_names, (
        "dialogue_callback обязан делегировать маршрутизацию sink'а в "
        "_resolve_voice_tts_sink (ADR-0021 CC-budget). "
        f"Найденные self-методы: {sorted(called_names)}"
    )


def test_sink_dispatch_routes_headset_and_preview_through_one_helper():
    """``dialogue_callback`` → ``_dispatch_voice_tts_sink`` → ``_on_headset_chunk`` / ``_on_avatar_tts_request_preview``.

    #4252 ч.2: обёртка ``/avatar/tts/request`` в цепочке шлема не участвует — тот же набор гвардов
    у реплики ТАРС гарантирует единственный путь ``_on_headset_chunk`` (инвариант 6b).
    """
    import ast
    import inspect
    import textwrap

    def _self_calls(fn) -> set:
        tree = ast.parse(textwrap.dedent(inspect.getsource(fn)))
        return {
            n.func.attr for n in ast.walk(tree)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            and isinstance(n.func.value, ast.Name) and n.func.value.id == "self"
        }

    assert "_dispatch_voice_tts_sink" in _self_calls(TTSNode.dialogue_callback)
    dispatch = _self_calls(TTSNode._dispatch_voice_tts_sink)
    assert {"_on_headset_chunk", "_on_avatar_tts_request_preview"} <= dispatch, dispatch
    assert "_on_avatar_tts_request" not in dispatch, dispatch


def test_sink_field_is_dispatched_in_exactly_one_primary_place():
    """AST-инвариант issue #2198 DoD: «ровно один вход синтезатора».

    Решение о маршруте по полю ``sink`` payload'а принимается в одном
    месте — ``_resolve_voice_tts_sink`` (pure-helper маршрутизации для
    нового канала ``/voice/tts/request``). ``dialogue_callback``
    делегирует в этот helper и не принимает решений сам.
    ``_on_avatar_tts_request`` (топик ``/avatar/tts/request``, только образец
    голоса) проверяет свой ``sink == "preview"`` сам.

    Что проверяет AST-тест: в исходнике ``tts_node.py`` (по всему файлу)
    методов, читающих поле ``sink`` из payload (``chunk_data``/``msg``)
    как **первичный** switch — ровно два:
      * ``_resolve_voice_tts_sink`` — новый primary chokepoint;
      * ``_on_avatar_tts_request`` — вход образца голоса (preview-only).
    ``dialogue_callback`` НЕ читает sink напрямую (делегирует в helper).
    """
    import ast
    import inspect

    src = inspect.getsource(TTSNode)
    tree = ast.parse(src)

    primary_switch_methods: list[str] = []

    for node in ast.walk(tree):
        if not isinstance(node, ast.FunctionDef):
            continue
        reads_sink_from_payload = False
        for sub in ast.walk(node):
            if isinstance(sub, ast.Subscript):
                sl = sub.slice
                if (
                    isinstance(sl, ast.Constant)
                    and sl.value == "sink"
                    and isinstance(sub.value, ast.Name)
                    and sub.value.id in ("chunk_data", "msg")
                ):
                    reads_sink_from_payload = True
                    break
            if isinstance(sub, ast.Call) and isinstance(sub.func, ast.Attribute):
                if sub.func.attr != "get" or not sub.args:
                    continue
                key = sub.args[0]
                if (
                    isinstance(key, ast.Constant)
                    and key.value == "sink"
                    and isinstance(sub.func.value, ast.Name)
                    and sub.func.value.id in ("chunk_data", "msg")
                ):
                    reads_sink_from_payload = True
                    break
        if reads_sink_from_payload:
            primary_switch_methods.append(node.name)

    # Chokepoint для нового канала — ``_resolve_voice_tts_sink`` (helper,
    # ADR-0021: не раздувать CC dialogue_callback). ``dialogue_callback``
    # НЕ должен читать sink сам — только звать helper.
    assert "dialogue_callback" not in primary_switch_methods, (
        "dialogue_callback НЕ должен читать sink из payload напрямую — "
        "это работа _resolve_voice_tts_sink (ADR-0021 CC-budget). "
        f"primary_switch_methods={primary_switch_methods}"
    )
    assert "_resolve_voice_tts_sink" in primary_switch_methods, (
        "_resolve_voice_tts_sink (primary chokepoint) обязан читать sink "
        f"из payload, got primary_switch_methods={primary_switch_methods}"
    )
    # Вход образца голоса /avatar/tts/request тоже читает sink (только "preview").
    assert set(primary_switch_methods) <= {
        "_resolve_voice_tts_sink",
        "_on_avatar_tts_request",
    }, (
        f"Найден неизвестный primary-switch (обход chokepoint'а!): "
        f"{primary_switch_methods}. Допустимы только _resolve_voice_tts_sink "
        f"(вход синтеза) и _on_avatar_tts_request (образец голоса)."
    )


# ── 4. backward-compat: dialogue_callback со старым payload (без sink) ──


def test_voice_tts_request_legacy_payload_without_sink_still_works():
    """Legacy payload от dialogue_node / speak_text (БЕЗ поля sink)
    продолжает работать как раньше — старый путь в динамики.

    Это критично: ``dialogue_node`` шлёт payload без sink с момента
    создания (issue #1709, issue #988). Никаких регрессий — sink
    опциональный, default='speaker'.
    """
    node = _make_voice_node()
    # Просто проверяем, что callback не падает и НЕ дёргает
    # avatar/preview цепочку.
    try:
        node.dialogue_callback(
            _msg(
                {
                    "text": "привет, робот",
                    "text": "привет, робот",
                    "speech_id": "legacy-1",
                    # sink отсутствует
                }
            )
        )
    except Exception as exc:  # noqa: BLE001
        pytest.fail(f"legacy payload упал в dialogue_callback: {exc!r}")

    node._on_headset_chunk.assert_not_called()
    node._on_avatar_tts_request_preview.assert_not_called()


# ── 5. issue #2318 — SoT-значение ``Sink.SPEAKERS`` ("speakers") ────────
#
# Регрессия из deploy z-{e2e}/test-round-388: продюсеры перешли на
# ``rob_box_core.utterance.Sink.SPEAKERS`` (== "speakers", voice-vr 12 /
# #2197), а consumer остался на единственном числе "speaker"
# (voice-vr 13 / #2198). Итог на роботе:
#
#   [tts_node] ⚠ /voice/tts/request: unknown sink='speakers'
#              (expected 'speaker'/'headset'/'preview'), DROP
#
# — то есть КАЖДАЯ реплика в динамики молча дропалась.


def test_sot_sink_speakers_value_is_accepted_by_resolver():
    """``Sink.SPEAKERS`` из SoT обязан резолвиться, а не давать ``None``.

    Тест берёт значение из самого SoT-модуля, а не строковый литерал —
    иначе при переименовании enum'а рассинхрон опять пройдёт незаметно.
    """
    from rob_box_core.utterance import Sink

    node = _make_voice_node()
    canonical, raw = node._resolve_voice_tts_sink({"sink": Sink.SPEAKERS.value})

    assert raw == Sink.SPEAKERS.value
    assert canonical == "speaker", (
        f"Sink.SPEAKERS ({Sink.SPEAKERS.value!r}) должен канонизироваться "
        f"в 'speaker' (единый вариант написания ниже по стеку), got {canonical!r}"
    )


def test_every_sot_sink_member_resolves_to_known_route():
    """Ни одно значение из ``Sink`` не должно уходить в DROP.

    Это и есть инвариант контракта: SoT-сборщик и consumer говорят на
    одном языке. Добавили новый Sink в ``rob_box_core`` и забыли про
    ``tts_node`` — тест падает здесь, а не в проде голосом «робот молчит».
    """
    from rob_box_core.utterance import Sink

    node = _make_voice_node()
    unresolved = {
        member.value: node._resolve_voice_tts_sink({"sink": member.value})[0]
        for member in Sink
        if node._resolve_voice_tts_sink({"sink": member.value})[0] is None
    }
    assert not unresolved, (
        f"значения Sink, которые tts_node дропает: {unresolved} — "
        f"рассинхрон rob_box_core.utterance ↔ tts_node (issue #2318)"
    )


def test_voice_tts_request_with_speakers_routes_to_speaker_path():
    """``sink='speakers'`` идёт в динамики, а НЕ в headset/preview и НЕ в DROP."""
    node = _make_voice_node()
    logger = node._test_logger
    node.dialogue_callback(
        _msg(
            {
                "text": "привет",
                "text": "привет",
                "sink": "speakers",
            }
        )
    )

    node._on_headset_chunk.assert_not_called()
    node._on_avatar_tts_request_preview.assert_not_called()
    warn_calls = [str(c) for c, _ in logger.warn.call_args_list]
    assert not any("unknown sink" in c for c in warn_calls), (
        f"sink='speakers' (SoT Sink.SPEAKERS) не должен давать DROP, "
        f"got warns: {warn_calls}"
    )


def test_utterance_to_request_payload_is_not_dropped_end_to_end():
    """Payload, собранный SoT-сборщиком, проходит маршрутизацию.

    Ближайший к проду вариант проверки: строим запрос ровно тем же
    вызовом, что и продюсеры (``Utterance(...).to_request()``), и
    прогоняем через настоящий ``dialogue_callback``.
    """
    from rob_box_core.utterance import Sink, Utterance

    node = _make_voice_node()
    logger = node._test_logger
    payload = Utterance(text="привет, робот", sink=Sink.SPEAKERS).to_request()
    assert payload["sink"] == "speakers"  # фиксируем, что SoT шлёт именно это

    node.dialogue_callback(_msg(payload))

    node._on_headset_chunk.assert_not_called()
    node._on_avatar_tts_request_preview.assert_not_called()
    warn_calls = [str(c) for c, _ in logger.warn.call_args_list]
    assert not any(
        "unknown sink" in c for c in warn_calls
    ), f"Utterance(sink=Sink.SPEAKERS).to_request() дропнут в tts_node: {warn_calls}"


def test_unknown_sink_still_drops_after_2318_fix():
    """Расширение whitelist НЕ должно превращать его в «пропускай всё»."""
    node = _make_voice_node()
    canonical, raw = node._resolve_voice_tts_sink({"sink": "matrix"})
    assert canonical is None and raw == "matrix"
