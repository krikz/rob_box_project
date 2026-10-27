"""Unit-тесты для ``/voice/tts/set_voice`` топик-контракта (ADR-0080 §2.7).

voice-vr 21 / ADR-0080 §2.7 — supervisor перестал писать в
``yandex_voice``/``minimax_voice``/``silero_speaker`` через
SetParameters (знание внутренней схемы имён) и публикует JSON в
``/voice/tts/set_voice``. ``_on_set_voice`` в ``TTSNode`` —
единственный потребитель этого топика; он применяет голос к
атрибуту (живой для следующего синтеза).

Покрывает:

* ``_on_set_voice`` с ``{"voice_id": "alena", "provider": "yandex"}``
  → ``self.yandex_voice == "alena"`` + лог-сообщение.
* ``_on_set_voice`` без ``provider`` (только ``voice_id``) →
  resolve через ``tts_voice_registry`` (в unit-стенде реестр
  замокан — lookup возвращает None → DROP с warning).
* ``_on_set_voice`` с неизвестным ``voice_id`` → DROP (никого
  не находим в реестре).
* ``_on_set_voice`` с битым JSON → DROP с warning.
* Идемпотентность: повторный вызов с тем же voice_id → атрибут
  остаётся прежним (без побочных эффектов).

Запуск:
    PYTHONPATH=src/rob_box_voice:src/rob_box_core:src/rob_box_harness:src/rob_box_llm \\
        pytest src/rob_box_voice/test/unit/tts/test_tts_node_set_voice.py -v
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from unittest.mock import MagicMock

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]  # rob_box_voice/
sys.path.insert(0, str(_PACKAGE_ROOT))

from test.unit.tts.conftest import _install_all_mocks  # noqa: E402

_install_all_mocks()

from rob_box_voice import tts_node as _tts_node_mod  # noqa: E402
from rob_box_voice.tts_node import TTSNode, _LIVE_VOICE_PARAMS  # noqa: E402


def _make_set_voice_node():
    """Минимальный node-stub для ``_on_set_voice``.

    Нужны только logger и атрибуты voice (``yandex_voice``/
    ``minimax_voice``/``silero_speaker``), которые ``_on_set_voice``
    устанавливает. Используем ``object.__new__`` чтобы не поднимать
    реальную TTSNode (конструктор тяжёлый).
    """
    n = object.__new__(TTSNode)
    n.get_logger = MagicMock()
    # Дефолтные значения (как __init__):
    n.yandex_voice = "anton"
    n.minimax_voice = "male-qn-qingse"
    n.silero_speaker = "baya"
    return n


def _msg(payload: dict):
    m = MagicMock()
    m.data = json.dumps(payload, ensure_ascii=False)
    return m


def test_set_voice_with_provider_hint_yandex_updates_attribute():
    """voice-vr 21 / ADR-0080 §2.7: payload ``{"voice_id": "alena",
    "provider": "yandex"}`` → ``self.yandex_voice == "alena"``.

    Знание схемы имён (yandex → yandex_voice) живёт ТОЛЬКО в
    ``_LIVE_VOICE_PARAMS`` tts_node — supervisor этого не знает.
    """
    n = _make_set_voice_node()
    n._on_set_voice(_msg({"voice_id": "alena", "provider": "yandex", "source": "set_voice"}))
    assert n.yandex_voice == "alena"
    # Другие провайдеры не трогаем.
    assert n.minimax_voice == "male-qn-qingse"
    assert n.silero_speaker == "baya"


def test_set_voice_with_provider_hint_minimax_updates_attribute():
    n = _make_set_voice_node()
    n._on_set_voice(_msg({"voice_id": "Russian_BrightHeroine", "provider": "minimax", "source": "set_voice"}))
    assert n.minimax_voice == "Russian_BrightHeroine"
    assert n.yandex_voice == "anton"  # не тронут


# ── #4086: голоса каста марафона в каталоге; топик не принимает голос вне каталога ──────────────────────────────


def _warnings(n) -> str:
    return " ".join(str(call) for call in n.get_logger().warning.call_args_list)


def test_set_voice_cast_voice_by_id_is_applied():
    """Голос каста (``English_Jovialman``) - голос каталога MiniMax: применяется как есть."""
    n = _make_set_voice_node()
    n._on_set_voice(_msg({"voice_id": "English_Jovialman", "provider": "minimax", "source": "set_voice"}))
    assert n.minimax_voice == "English_Jovialman"


def test_set_voice_topic_takes_ids_not_names():
    """Топик ``/voice/tts/set_voice`` читает id (его пишут supervisor и Quest, оба с проверкой по каталогу). Имя
    («Весельчак») в id переводит владелец голоса (``VoiceOwner`` через ``resolve_voice``), а не этот обработчик."""
    n = _make_set_voice_node()
    n._on_set_voice(_msg({"voice_id": "Весельчак", "provider": "minimax", "source": "set_voice"}))
    assert n.minimax_voice == "male-qn-qingse"
    assert "нет в каталоге" in _warnings(n)


def test_set_voice_unknown_voice_with_provider_hint_is_refused_by_code():
    """Раньше при явном ``provider`` в параметр уходил любой id, а синтез молча (WARN) брал дефолт.
    Теперь голос вне каталога топик не принимает: голос не меняется, в логе причина."""
    n = _make_set_voice_node()
    n._on_set_voice(_msg({"voice_id": "Terminator_3000", "provider": "minimax", "source": "set_voice"}))
    assert n.minimax_voice == "male-qn-qingse"
    assert "Terminator_3000" in _warnings(n) and "нет в каталоге" in _warnings(n)


def test_set_voice_voice_of_another_provider_is_refused():
    """``zahar`` - голос Yandex; у MiniMax его нет (раньше топик клал его в ``minimax_voice``)."""
    n = _make_set_voice_node()
    n._on_set_voice(_msg({"voice_id": "zahar", "provider": "minimax", "source": "set_voice"}))
    assert n.minimax_voice == "male-qn-qingse"
    assert n.yandex_voice == "anton"
    assert "нет в каталоге" in _warnings(n) and "'minimax'" in _warnings(n)


def test_set_voice_with_provider_hint_silero_updates_attribute():
    n = _make_set_voice_node()
    n._on_set_voice(_msg({"voice_id": "kseniya", "provider": "silero", "source": "set_voice"}))
    assert n.silero_speaker == "kseniya"
    assert n.yandex_voice == "anton"  # не тронут


def test_set_voice_invalid_json_dropped():
    """Битый JSON → DROP с warning, атрибут не меняется."""
    n = _make_set_voice_node()
    bad_msg = MagicMock()
    bad_msg.data = "{not valid json"
    n._on_set_voice(bad_msg)
    assert n.yandex_voice == "anton"
    # Проверяем что warning был залоган.
    warning_calls = [
        call for call in n.get_logger().warning.call_args_list
        if "bad JSON" in str(call)
    ]
    assert warning_calls, "expected warning log for bad JSON"


def test_set_voice_missing_voice_id_dropped():
    """Payload без ``voice_id`` → DROP, ничего не меняется."""
    n = _make_set_voice_node()
    n._on_set_voice(_msg({"provider": "yandex", "source": "set_voice"}))
    assert n.yandex_voice == "anton"
    warning_calls = [
        call for call in n.get_logger().warning.call_args_list
        if "missing or empty voice_id" in str(call)
    ]
    assert warning_calls


def test_set_voice_empty_voice_id_dropped():
    n = _make_set_voice_node()
    n._on_set_voice(_msg({"voice_id": "", "provider": "yandex"}))
    assert n.yandex_voice == "anton"


def test_set_voice_unknown_provider_dropped():
    """Неизвестный провайдер → DROP, атрибут не меняется.

    Защита от того, чтобы чужой код не мог записать в tts_node через
    произвольного провайдера (что было бы новым швом).
    """
    n = _make_set_voice_node()
    n._on_set_voice(_msg({"voice_id": "x", "provider": "openai"}))
    assert n.yandex_voice == "anton"
    assert n.minimax_voice == "male-qn-qingse"
    assert n.silero_speaker == "baya"
    warning_calls = [
        call for call in n.get_logger().warning.call_args_list
        if "провайдер неизвестен" in str(call)
    ]
    assert warning_calls


def test_set_voice_idempotent_same_voice():
    """Повторный set_voice с тем же voice_id → атрибут прежний."""
    n = _make_set_voice_node()
    n._on_set_voice(_msg({"voice_id": "alena", "provider": "yandex"}))
    first = n.yandex_voice
    n._on_set_voice(_msg({"voice_id": "alena", "provider": "yandex"}))
    assert n.yandex_voice == first == "alena"


def test_set_voice_without_provider_falls_back_to_registry_lookup():
    """Без provider в payload — пытаемся резолвить через
    ``tts_voice_registry``. Если реестр вернул None — DROP (warning).

    На mock-стенде ``voices_for`` через registry mock может возвращать
    что угодно. Мы проверяем что handler не падает и не меняет
    атрибут «не глядя» — это и есть главный DoD.
    """
    n = _make_set_voice_node()
    # Голос «bogus» точно не в реестре (mock может вернуть [],
    # но мы проверяем что handler не пишет «в никуда»).
    n._on_set_voice(_msg({"voice_id": "bogus_voice_xyz"}))
    # Атрибут не должен смениться «на пустое значение» — только на
    # если registry сказал нам provider.
    assert n.yandex_voice == "anton"


def test_set_voice_topic_subscribed():
    """Sanity: ``/voice/tts/set_voice`` подписка создаётся в __init__.

    Это контрактный тест: если кто-то снесёт ``self.set_voice_sub``
    или ``/voice/tts/set_voice``, мы это поймаем.
    """
    from test.unit.tts.conftest import _install_all_mocks
    _install_all_mocks()

    # Создаём ноду через полноценный __init__ (mock-rclpy).
    n = TTSNode()
    try:
        # mock-rclpy хранит ``_subscriptions`` как список кортежей
        # ``((msg_type, topic, callback, *args), kwargs)`` (см.
        # ``src/rob_box_voice/test/ros_stubs.py``). Берём topic (index 1
        # в первом элементе пары).
        subs = [
            entry[0][1] if isinstance(entry, tuple) and len(entry) >= 1
            else entry for entry in n._subscriptions
        ]
        assert "/voice/tts/set_voice" in subs, (
            "voice-vr 21: TTSNode должен подписываться на "
            "/voice/tts/set_voice (ADR-0080 §2.7)"
        )
        # PR-4 ч.3: смену голоса/провайдера просит тул через /voice/tts/voice_request (VoiceOwner);
        # прямой /voice/tts/set_provider без писателей снят.
        assert "/voice/tts/voice_request" in subs
        assert "/voice/tts/set_provider" not in subs
    finally:
        try:
            n.destroy_node()
        except Exception:
            pass
