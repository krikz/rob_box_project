"""Issue #2198 / voice-vr 13 — единый вход ``/voice/tts/request`` с полем ``sink``.

Тесты покрывают инвариант задачи: **приёмник реплики (динамики / шлем /
preview) задаётся полем ``sink`` в payload, а не отдельным ROS-топиком**.

* ``/voice/tts/request`` (subscribed в ``dialogue_callback``) — единственный
  канал для синтеза. Поле ``sink`` ∈ {``"speaker"`` (default), ``"headset"``,
  ``"preview"``} маршрутизирует в соответствующий путь.
* ``/avatar/tts/request`` (subscribed в ``_on_avatar_tts_request``) —
  DEPRECATED алиас на этот релиз (см. задачу, удалить в следующем
  релизе). Логирует WARNING на каждый вызов.
* Любой не-валидный ``sink`` → DROP + WARN (как для ``/avatar/tts/request``
  с ``sink="bogus"``, так и для ``/voice/tts/request``).

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

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

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
      - ``finished_pub`` / ``_avatar_tts_error_pub`` — для _on_avatar_tts_request;
      - ``_submit_synthesis`` (MagicMock) — для проверки «пошёл ли синтез»;
      - ``_dispatch_avatar_tts_sink`` (MagicMock) — для делегации в headset-путь;
      - ``_on_avatar_tts_request_preview`` (MagicMock) — для делегации в preview-путь.

    ``logger`` — ОДИН MagicMock на ноду: иначе каждый вызов ``self.get_logger()``
    возвращал бы свежий mock, и тест на ``logger.warn.called`` не видел бы
    сообщений, залогированных внутри callback'а.
    """
    logger = MagicMock()
    n = object.__new__(TTSNode)
    n.get_logger = lambda: logger
    n._test_logger = logger  # для тестовых assert'ов
    n.finished_pub = _CapturingPublisher()
    n._avatar_tts_error_pub = _CapturingPublisher()
    n._submit_synthesis = MagicMock()
    n._dispatch_avatar_tts_sink = MagicMock(return_value=False)
    n._on_avatar_tts_request_preview = MagicMock()
    n.current_speech_id = None
    n.current_dialogue_id = None
    n.processing_dialogue_id = None
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
    полной инициализации), и НЕ дёрнут ``_on_avatar_tts_request``
    delegation-цепочка. Достаточно assert'нуть, что ``_dispatch_avatar_tts_sink``
    и ``_on_avatar_tts_request_preview`` не вызывались.
    """
    node = _make_voice_node()
    node.dialogue_callback(_msg({"ssml": "<speak>привет</speak>", "text": "привет"}))

    node._dispatch_avatar_tts_sink.assert_not_called()
    node._on_avatar_tts_request_preview.assert_not_called()


def test_voice_tts_request_with_speaker_routes_to_speakers():
    """``sink='speaker'`` → старый путь в динамики (НЕ headset/preview)."""
    node = _make_voice_node()
    node.dialogue_callback(
        _msg(
            {
                "ssml": "<speak>привет</speak>",
                "text": "привет",
                "sink": "speaker",
            }
        )
    )

    node._dispatch_avatar_tts_sink.assert_not_called()
    node._on_avatar_tts_request_preview.assert_not_called()


def test_voice_tts_request_with_headset_delegates_to_avatar_handler():
    """``sink='headset'`` в ``/voice/tts/request`` → делегация в headset-путь.

    Семантика: ``dialogue_callback`` теперь знает про поле sink и для
    ``headset`` маршрутизирует в ``_on_avatar_tts_request`` (тот же
    набор гвардов, что и для deprecated ``/avatar/tts/request`` —
    backward-compat с прямыми публикаторами).
    """
    node = _make_voice_node()
    # Подменяем _on_avatar_tts_request на mock, чтобы убедиться, что
    # dialogue_callback его дёрнул (не проверяя внутренности headset-пути —
    # это уже покрыто test_tts_node_avatar.py).
    node._on_avatar_tts_request = MagicMock()  # type: ignore[assignment]
    node.dialogue_callback(
        _msg(
            {
                "ssml": "<speak>привет</speak>",
                "text": "привет",
                "sink": "headset",
                "request_id": "req-from-voice-with-sink-headset",
            }
        )
    )

    node._on_avatar_tts_request.assert_called_once()
    # Убедимся, что message передан «как есть» — callback распарсит JSON
    # сам (как в существующей реализации для backward-compat).
    call_msg = node._on_avatar_tts_request.call_args.args[0]
    assert json.loads(call_msg.data)["sink"] == "headset"


def test_voice_tts_request_with_preview_delegates_to_preview_path():
    """``sink='preview'`` в ``/voice/tts/request`` → ``_on_avatar_tts_request_preview``.

    Это синхронный preview-путь (НЕ ThreadPoolExecutor, НЕ barge-in,
    НЕ dialogues), см. ADR-0077. Маршрутизация должна обходить
    headset-путь, который публикует в ``/avatar/tts/audio``.
    """
    node = _make_voice_node()
    # Чтобы preview-путь дошёл до _on_avatar_tts_request_preview без
    # падения на guard'ах — делаем заглушки:
    node._extract_text_from_ssml = lambda ssml: "привет"  # type: ignore[assignment]
    node._parse_ssml_attributes = lambda ssml: {}  # type: ignore[assignment]
    node._publish_tars1_text = MagicMock()  # type: ignore[assignment]
    node._avatar_tts_error_pub = _CapturingPublisher()

    # preview-путь идёт напрямую через _on_avatar_tts_request_preview,
    # а не через полный _on_avatar_tts_request (который логирует
    # deprecated-WARNING и делает JSON-парсинг заново). Чтобы избежать
    # side-effect-ов от deprecated-WARNING, отдельный callback:
    original_on_avatar = node._on_avatar_tts_request
    call_count = {"avatar": 0}

    def _counting_on_avatar(msg):
        call_count["avatar"] += 1
        # Делегируем в preview через оригинальный код — но мы bare-stub,
        # поэтому простой no-op.

    node._on_avatar_tts_request = _counting_on_avatar  # type: ignore[assignment]
    node.dialogue_callback(
        _msg(
            {
                "request_id": "req-from-voice-with-sink-preview",
                "ssml": "<speak>привет</speak>",
                "sink": "preview",
                "voice": "male-qn-qingse",
            }
        )
    )

    # _on_avatar_tts_request_preview вызывается ИЗ _on_avatar_tts_request.
    # Подтверждаем, что мы зашли в avatar-цепочку (deprecated callback),
    # но не в headset-путь (``_submit_synthesis`` не вызван).
    assert call_count["avatar"] == 1
    node._submit_synthesis.assert_not_called()
    del original_on_avatar


def test_voice_tts_request_unknown_sink_drops_with_warning():
    """Любой другой sink (не speaker/headset/preview) → DROP + WARN."""
    node = _make_voice_node()
    logger = node._test_logger
    node.dialogue_callback(
        _msg(
            {
                "ssml": "<speak>привет</speak>",
                "text": "привет",
                "sink": "matrix",  # неизвестный приёмник
            }
        )
    )
    # DROP → _submit_synthesis, _dispatch_avatar_tts_sink не вызваны.
    node._submit_synthesis.assert_not_called()
    node._dispatch_avatar_tts_sink.assert_not_called()
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


# ── 2. ``/avatar/tts/request`` deprecated — WARNING на каждый вызов ────


def test_avatar_tts_request_logs_deprecated_warning():
    """Прямой вызов ``_on_avatar_tts_request`` (через ``/avatar/tts/request``)
    логирует WARNING про deprecated и переход на ``/voice/tts/request`` с
    полем ``sink``.
    """
    node = _make_voice_node()
    # Чтобы тест не падал на guard'ах (sink must be valid), прокинем
    # sink='headset' + подменяем _submit_synthesis.
    node._submit_synthesis = MagicMock()  # type: ignore[assignment]
    node._parse_ssml_attributes = lambda ssml: {}  # type: ignore[assignment]
    node._extract_text_from_ssml = lambda ssml: "Готово"  # type: ignore[assignment]
    node._publish_tars1_text = MagicMock()  # type: ignore[assignment]
    node._dispatch_avatar_tts_sink = MagicMock(return_value=True)  # type: ignore[assignment]

    logger = node._test_logger
    node._on_avatar_tts_request(
        _msg(
            {
                "request_id": "req-avatar-deprecated",
                "ssml": "<speak>Готово</speak>",
                "sink": "headset",
            }
        )
    )

    # WARN с упоминанием issue #2198 и /voice/tts/request.
    warn_calls = [
        str(call_args)
        for call_args, _ in logger.warn.call_args_list
    ]
    assert any("2198" in str(c) and "deprecated" in str(c).lower() for c in warn_calls), (
        f"ожидался WARNING про deprecated /avatar/tts/request, "
        f"got: {warn_calls}"
    )


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


def test_sink_dispatch_via_voice_or_avatar_topics_route_through_consistent_helpers():
    """``/voice/tts/request`` с sink ∈ {headset, preview} маршрутизирует в
    ТЕ ЖЕ helper'ы, что и ``/avatar/tts/request`` (backward-compat).

    После рефакторинга step 2/2: dialogue_callback делегирует dispatch в
    ``_dispatch_voice_tts_sink`` (ADR-0021 CC-budget helper), который уже
    вызывает ``_on_avatar_tts_request`` для headset/preview-путей.
    Проверяем через AST, что цепочка «dialogue_callback → dispatch →
    _on_avatar_tts_request» сохранена (либо напрямую, либо через helper).
    Это гарантирует, что инвариант 6b (одинаковый набор гвардов для
    ТАРС-в-шлем, независимо от топика-источника) сохраняется.
    """
    import ast
    import inspect
    import textwrap

    src_dlg = textwrap.dedent(inspect.getsource(TTSNode.dialogue_callback))
    tree_dlg = ast.parse(src_dlg)
    called_in_dialogue: set[str] = set()
    for ast_node in ast.walk(tree_dlg):
        if isinstance(ast_node, ast.Call) and isinstance(
            ast_node.func, ast.Attribute
        ):
            if (
                isinstance(ast_node.func.value, ast.Name)
                and ast_node.func.value.id == "self"
            ):
                called_in_dialogue.add(ast_node.func.attr)

    # Ищем цепочку: dialogue_callback → (напрямую или через helper) →
    # _on_avatar_tts_request. Допустимы обе формы после рефакторинга.
    if "_on_avatar_tts_request" in called_in_dialogue:
        # Старая форма (step 1/2): прямой вызов.
        assert True
        return
    assert "_dispatch_voice_tts_sink" in called_in_dialogue, (
        "dialogue_callback обязан делегировать dispatch (либо напрямую в "
        "_on_avatar_tts_request, либо через _dispatch_voice_tts_sink) для "
        "консистентности гвардов. "
        f"Найденные self-методы: {sorted(called_in_dialogue)}"
    )

    # Проверяем, что _dispatch_voice_tts_sink вызывает _on_avatar_tts_request.
    src_disp = textwrap.dedent(inspect.getsource(TTSNode._dispatch_voice_tts_sink))
    tree_disp = ast.parse(src_disp)
    called_in_dispatch: set[str] = set()
    for ast_node in ast.walk(tree_disp):
        if isinstance(ast_node, ast.Call) and isinstance(
            ast_node.func, ast.Attribute
        ):
            if (
                isinstance(ast_node.func.value, ast.Name)
                and ast_node.func.value.id == "self"
            ):
                called_in_dispatch.add(ast_node.func.attr)

    assert "_on_avatar_tts_request" in called_in_dispatch, (
        "_dispatch_voice_tts_sink обязан делегировать в "
        "_on_avatar_tts_request для headset/preview-путей (invariant 6b). "
        f"Найденные self-методы в helper: {sorted(called_in_dispatch)}"
    )


def test_sink_field_is_dispatched_in_exactly_one_primary_place():
    """AST-инвариант issue #2198 DoD: «ровно один вход синтезатора».

    Решение о маршруте по полю ``sink`` payload'а принимается в одном
    месте — ``_resolve_voice_tts_sink`` (pure-helper маршрутизации для
    нового канала ``/voice/tts/request``). ``dialogue_callback``
    делегирует в этот helper и не принимает решений сам.
    ``_on_avatar_tts_request`` (deprecated backward-compat для прямых
    публикаторов в ``/avatar/tts/request``) имеет свой локальный switch
    через ``_dispatch_avatar_tts_sink`` — он уйдёт вместе с удалением
    deprecated-подписки в следующем релизе.

    Что проверяет AST-тест: в исходнике ``tts_node.py`` (по всему файлу)
    методов, читающих поле ``sink`` из payload (``chunk_data``/``msg``)
    как **первичный** switch — ровно два:
      * ``_resolve_voice_tts_sink`` — новый primary chokepoint;
      * ``_on_avatar_tts_request`` — deprecated wrapper, уйдёт.
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
    # Deprecated wrapper для backward-compat: тоже читает sink из
    # payload (через _dispatch_avatar_tts_sink). Уйдёт вместе с
    # удалением /avatar/tts/request подписки в следующем релизе.
    assert set(primary_switch_methods) <= {
        "_resolve_voice_tts_sink",
        "_on_avatar_tts_request",
    }, (
        f"Найден неизвестный primary-switch (обход chokepoint'а!): "
        f"{primary_switch_methods}. Допустимы только _resolve_voice_tts_sink "
        f"(новый канал) и _on_avatar_tts_request (deprecated backward-compat)."
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
                    "ssml": "<speak>привет, робот</speak>",
                    "text": "привет, робот",
                    "speech_id": "legacy-1",
                    # sink отсутствует
                }
            )
        )
    except Exception as exc:  # noqa: BLE001
        pytest.fail(f"legacy payload упал в dialogue_callback: {exc!r}")

    node._dispatch_avatar_tts_sink.assert_not_called()
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
                "ssml": "<speak>привет</speak>",
                "text": "привет",
                "sink": "speakers",
            }
        )
    )

    node._dispatch_avatar_tts_sink.assert_not_called()
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

    node._dispatch_avatar_tts_sink.assert_not_called()
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
