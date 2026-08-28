"""
test_pure_methods.py — Unit-тесты «лёгких» методов DialogueNode.

Покрывает:
  - _map_emotion_to_animation    (чистый dict lookup)
  - _generate_fallback_response  (чистый string matching)
  - _detect_volume_intent        (regex, без ROS)
  - _detect_pitch_intent         (regex, без ROS)
  - _speak_simple                (publish + json)
  - _trigger_sound               (publish)
  - _on_mcp_tools_update         (JSON parse → attributes)
  - _on_perception_update        (attribute update)
  - vad_callback                 (flag logic)

Не требует ROS2 — rclpy замокан в conftest.py.
"""

import json
from unittest.mock import MagicMock, call

import pytest

from rob_box_voice.dialogue_node import DialogueNode


# ─────────────────────────────────────────────────────────────────────────────
#  Fixture
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def node():
    """Минимальная DialogueNode без __init__."""
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n._logger = logger
    n.get_logger = lambda: logger

    # Publishers
    n.response_pub = MagicMock()
    n.animation_pub = MagicMock()
    n.sound_trigger_pub = MagicMock()
    n.state_pub = MagicMock()

    # State attrs
    n.current_dialogue_id = None
    n.available_tools = []
    n.mcp_tools_available = False
    n.internet_available = True
    n.current_time_info = {}

    # VAD
    n.vad_speech_detected = False
    n.llm_processing = False
    n.interrupt_agent_loop = False

    # dialogue_manager (for _publish_state)
    n.dialogue_manager = MagicMock()
    n.dialogue_manager.state.value = "idle"

    # speaker (for _build_dynamic_system_context)
    import threading
    n._speaker_lock = threading.Lock()
    n._current_speaker = {}

    # tts params (issue #1229 — actual provider / voice)
    n._actual_tts_provider = None
    n._actual_tts_voice = None
    n._current_tts_voice = None
    n._declared_params = {"tts_provider": "minimax"}

    def _get_parameter(name):
        return type("P", (), {"value": n._declared_params.get(name, "minimax")})()

    n.get_parameter = _get_parameter

    # Issue #1544 — music-state snapshot sources.
    # Все четыре поля по умолчанию "ничего не играет"; тесты точечно
    # переопределяют нужные куски для проверки каждой ветки.
    n._dj = None  # type: ignore[assignment]
    n._pending_music_cleanup = False
    n._active_batches = {}
    n._generated_music_state = None
    # SpeechAccumulator/SchedulerExecutor — реальные инстансы, но
    # ``_build_dynamic_system_context`` лезет через getattr с дефолтом None
    # в каждой ветке, поэтому просто притворимся, что их нет.
    setattr(n, "_speech_accumulator", None)
    setattr(n, "_accumulate_no_wake_enabled", False)
    setattr(n, "_pending_backlog_flush", False)
    setattr(n, "_scheduler_executor", None)

    return n


# ─────────────────────────────────────────────────────────────────────────────
#  _map_emotion_to_animation
# ─────────────────────────────────────────────────────────────────────────────

class TestMapEmotionToAnimation:
    def test_known_emotions_mapped_correctly(self, node):
        assert node._map_emotion_to_animation("happy") == "happy"
        assert node._map_emotion_to_animation("sad") == "sad"
        assert node._map_emotion_to_animation("angry") == "angry"
        assert node._map_emotion_to_animation("surprised") == "surprised"
        assert node._map_emotion_to_animation("thinking") == "thinking"
        assert node._map_emotion_to_animation("excited") == "victory"
        assert node._map_emotion_to_animation("confused") == "thinking"
        assert node._map_emotion_to_animation("worried") == "sad"

    def test_neutral_and_calm_map_to_idle(self, node):
        assert node._map_emotion_to_animation("neutral") == "idle"
        assert node._map_emotion_to_animation("calm") == "idle"

    def test_unknown_emotion_returns_idle(self, node):
        assert node._map_emotion_to_animation("furious") == "idle"
        assert node._map_emotion_to_animation("") == "idle"
        assert node._map_emotion_to_animation("XYZ") == "idle"

    def test_case_insensitive(self, node):
        assert node._map_emotion_to_animation("HAPPY") == "happy"
        assert node._map_emotion_to_animation("Sad") == "sad"
        assert node._map_emotion_to_animation("NEUTRAL") == "idle"


# ─────────────────────────────────────────────────────────────────────────────
#  _generate_fallback_response
# ─────────────────────────────────────────────────────────────────────────────

class TestGenerateFallbackResponse:
    def test_greeting_returns_no_internet_message(self, node):
        resp = node._generate_fallback_response("Привет!")
        assert "интернет" in resp.lower()

    def test_hello_english(self, node):
        resp = node._generate_fallback_response("hello robot")
        assert "интернет" in resp.lower()

    def test_how_are_you(self, node):
        resp = node._generate_fallback_response("как дела?")
        assert "интернет" in resp.lower()

    def test_thanks(self, node):
        resp = node._generate_fallback_response("спасибо тебе!")
        assert resp == "Пожалуйста!"

    def test_bye(self, node):
        resp = node._generate_fallback_response("пока!")
        assert resp == "До свидания!"

    def test_bye_english(self, node):
        resp = node._generate_fallback_response("bye")
        assert resp == "До свидания!"

    def test_unknown_returns_default(self, node):
        resp = node._generate_fallback_response("сделай что-нибудь")
        assert "интернет" in resp.lower()
        assert "недоступен" in resp.lower()


# ─────────────────────────────────────────────────────────────────────────────
#  _detect_volume_intent
# ─────────────────────────────────────────────────────────────────────────────

class TestDetectVolumeIntent:
    def test_louder(self, node):
        assert node._detect_volume_intent("громче пожалуйста") == "louder"
        assert node._detect_volume_intent("сделай громко") == "louder"

    def test_quieter(self, node):
        assert node._detect_volume_intent("говори тише") == "quieter"
        assert node._detect_volume_intent("потише пожалуйста") == "quieter"

    def test_volume_max(self, node):
        # "говори громко" → специфичный паттерн max, не louder
        assert node._detect_volume_intent("говори громко") == "max"
        assert node._detect_volume_intent("максимальная громкость") == "max"

    def test_volume_normal(self, node):
        # max/normal проверяются до louder — "громкость" не перехватывается r"громко"
        assert node._detect_volume_intent("нормальная громкость") == "normal"
        assert node._detect_volume_intent("стандартная громкость") == "normal"
        assert node._detect_volume_intent("обычная громкость") == "normal"

    def test_no_match_returns_none(self, node):
        assert node._detect_volume_intent("расскажи анекдот") is None
        assert node._detect_volume_intent("") is None

    def test_case_insensitive_via_lower(self, node):
        assert node._detect_volume_intent("ТИШЕ") == "quieter"


# ─────────────────────────────────────────────────────────────────────────────
#  _detect_pitch_intent
# ─────────────────────────────────────────────────────────────────────────────

class TestDetectPitchIntent:
    def test_higher(self, node):
        assert node._detect_pitch_intent("говори выше") == "higher"
        assert node._detect_pitch_intent("повысь голос") == "higher"

    def test_lower(self, node):
        assert node._detect_pitch_intent("говори ниже") == "lower"
        assert node._detect_pitch_intent("голос ниже") == "lower"

    def test_normal(self, node):
        assert node._detect_pitch_intent("нормальный голос") == "normal"
        assert node._detect_pitch_intent("говори нормально") == "normal"
        assert node._detect_pitch_intent("обычный голос") == "normal"

    def test_no_match_returns_none(self, node):
        assert node._detect_pitch_intent("расскажи сказку") is None
        assert node._detect_pitch_intent("") is None


# ─────────────────────────────────────────────────────────────────────────────
#  _speak_simple
# ─────────────────────────────────────────────────────────────────────────────

class TestSpeakSimple:
    def test_publishes_ssml_json(self, node):
        node._speak_simple("Привет мир")

        node.response_pub.publish.assert_called_once()
        published = node.response_pub.publish.call_args[0][0]
        data = json.loads(published.data)
        assert "<speak>Привет мир</speak>" in data["ssml"]
        assert "dialogue_id" in data

    def test_sets_current_dialogue_id(self, node):
        node._speak_simple("Тест")
        assert node.current_dialogue_id is not None

    def test_without_error_animation_no_anim_publish(self, node):
        node._speak_simple("Тест", show_error_animation=False)
        node.animation_pub.publish.assert_not_called()

    def test_with_error_animation_publishes_error(self, node):
        node._speak_simple("Ошибка", show_error_animation=True)
        node.animation_pub.publish.assert_called_once()
        anim_data = node.animation_pub.publish.call_args[0][0].data
        assert "error" in anim_data

    def test_dialogue_id_unique_per_call(self, node):
        node._speak_simple("раз")
        id1 = node.current_dialogue_id
        node._speak_simple("два")
        id2 = node.current_dialogue_id
        assert id1 != id2


# ─────────────────────────────────────────────────────────────────────────────
#  _trigger_sound
# ─────────────────────────────────────────────────────────────────────────────

class TestTriggerSound:
    def test_publishes_sound_name(self, node):
        node._trigger_sound("startup")

        node.sound_trigger_pub.publish.assert_called_once()
        msg = node.sound_trigger_pub.publish.call_args[0][0]
        assert msg.data == "startup"

    def test_no_crash_on_publish_exception(self, node):
        node.sound_trigger_pub.publish.side_effect = RuntimeError("bus error")
        # Должен поглотить ошибку без краша
        node._trigger_sound("startup")


# ─────────────────────────────────────────────────────────────────────────────
#  _on_mcp_tools_update
# ─────────────────────────────────────────────────────────────────────────────

class TestOnMcpToolsUpdate:
    def _make_msg(self, data: str):
        msg = MagicMock()
        msg.data = data
        return msg

    def test_valid_json_sets_tools(self, node):
        tools = [
            {"function": {"name": "navigate"}, "type": "function"},
            {"function": {"name": "speak"}, "type": "function"},
        ]
        node._on_mcp_tools_update(self._make_msg(json.dumps(tools)))

        assert node.available_tools == tools
        assert node.mcp_tools_available is True

    def test_empty_list_clears_tools(self, node):
        node.available_tools = [{"function": {"name": "old"}}]
        node._on_mcp_tools_update(self._make_msg("[]"))
        assert node.available_tools == []
        assert node.mcp_tools_available is True

    def test_invalid_json_sets_unavailable(self, node):
        node.mcp_tools_available = True
        node._on_mcp_tools_update(self._make_msg("{invalid json"))
        assert node.mcp_tools_available is False

    def test_invalid_json_logs_error(self, node):
        node._on_mcp_tools_update(self._make_msg("not json"))
        node.get_logger().error.assert_called()


# ─────────────────────────────────────────────────────────────────────────────
#  _on_perception_update
# ─────────────────────────────────────────────────────────────────────────────

class TestOnPerceptionUpdate:
    def _make_msg(self, internet=None, time_json=None):
        msg = MagicMock(spec=[])  # spec=[] → нет атрибутов по умолчанию
        if internet is not None:
            msg.internet_available = internet
        if time_json is not None:
            msg.time_context_json = time_json
        return msg

    def test_internet_available_true(self, node):
        node.internet_available = False
        node._on_perception_update(self._make_msg(internet=True))
        assert node.internet_available is True

    def test_internet_becomes_unavailable(self, node):
        node.internet_available = True
        node._on_perception_update(self._make_msg(internet=False))
        assert node.internet_available is False

    def test_time_context_json_parsed(self, node):
        time_data = {"time_only": "12:30", "date": "2026-02-21"}
        node._on_perception_update(self._make_msg(time_json=json.dumps(time_data)))
        assert node.current_time_info["time_only"] == "12:30"

    def test_invalid_time_json_no_crash(self, node):
        node._on_perception_update(self._make_msg(time_json="bad json"))
        # Не падает, логирует warning
        node.get_logger().warning.assert_called()

    def test_msg_without_attrs_no_crash(self, node):
        """Сообщение без internet_available и time_context_json."""
        msg = MagicMock(spec=[])
        node._on_perception_update(msg)  # не должно падать


# ─────────────────────────────────────────────────────────────────────────────
#  vad_callback
# ─────────────────────────────────────────────────────────────────────────────

class TestVadCallback:
    def _make_msg(self, active: bool):
        msg = MagicMock()
        msg.data = active
        return msg

    def test_rising_edge_sets_vad_speech_detected(self, node):
        node.vad_speech_detected = False
        node._vad_callback(self._make_msg(True)) if hasattr(node, '_vad_callback') \
            else node.vad_callback(self._make_msg(True))
        assert node.vad_speech_detected is True

    def test_rising_edge_during_llm_sets_interrupt(self, node):
        node.vad_speech_detected = False
        node.llm_processing = True
        node.mcp_tools_available = False
        node.vad_callback(self._make_msg(True))
        assert node.interrupt_agent_loop is True

    def test_no_interrupt_when_not_processing(self, node):
        node.vad_speech_detected = False
        node.llm_processing = False
        node.vad_callback(self._make_msg(True))
        assert node.interrupt_agent_loop is False

    def test_falling_edge_clears_vad_speech_detected(self, node):
        node.vad_speech_detected = True
        node.vad_callback(self._make_msg(False))
        assert node.vad_speech_detected is False

    def test_no_re_trigger_if_already_detected(self, node):
        """Rising edge уже был: повторный True не должен ставить interrupt."""
        node.vad_speech_detected = True  # уже детектирован
        node.llm_processing = True
        node.interrupt_agent_loop = False
        node.mcp_tools_available = False
        node.vad_callback(self._make_msg(True))
        # Нет rising edge (уже было True), interrupt не трогаем
        assert node.interrupt_agent_loop is False


# ─────────────────────────────────────────────────────────────────────────────
#  _on_tts_provider_state (issue #1229)
# ─────────────────────────────────────────────────────────────────────────────

class TestOnTtsProviderState:
    """tts_node публикует ФАКТИЧЕСКОГО провайдера (после фолбека minimax→yandex).

    DialogueNode запоминает его и голос — LLM-контекст [TTS] строится по
    реальному провайдеру, а не по номинальному из параметра tts_provider.
    """

    def _make_msg(self, data: str):
        msg = MagicMock()
        msg.data = data
        return msg

    def test_valid_payload_sets_actual_provider(self, node):
        node._actual_tts_provider = None
        node._actual_tts_voice = None
        node._on_tts_provider_state(
            self._make_msg(
                '{"provider": "yandex", "voice": "zahar", "reason": "provider_dead"}'
            )
        )
        assert node._actual_tts_provider == "yandex"
        assert node._actual_tts_voice == "zahar"

    def test_payload_without_voice_keeps_provider(self, node):
        node._actual_tts_provider = None
        node._actual_tts_voice = None
        node._on_tts_provider_state(
            self._make_msg('{"provider": "silero", "reason": "startup"}')
        )
        assert node._actual_tts_provider == "silero"
        assert node._actual_tts_voice is None

    def test_missing_provider_ignored(self, node):
        node._actual_tts_provider = None
        node._on_tts_provider_state(self._make_msg('{"voice": "anton"}'))
        assert node._actual_tts_provider is None

    def test_invalid_json_no_crash(self, node):
        node._actual_tts_provider = "minimax"
        node._on_tts_provider_state(self._make_msg("not-json"))
        assert node._actual_tts_provider == "minimax"

    def test_empty_data_no_crash(self, node):
        node._actual_tts_provider = "minimax"
        node._on_tts_provider_state(self._make_msg(""))
        assert node._actual_tts_provider == "minimax"

    def test_repeated_updates_overwrite(self, node):
        node._actual_tts_provider = None
        node._on_tts_provider_state(
            self._make_msg('{"provider": "minimax", "reason": "synthesis_ok"}')
        )
        node._on_tts_provider_state(
            self._make_msg('{"provider": "yandex", "voice": "anton", "reason": "provider_dead"}')
        )
        assert node._actual_tts_provider == "yandex"
        assert node._actual_tts_voice == "anton"


# ─────────────────────────────────────────────────────────────────────────────
#  _build_dynamic_system_context — фактический провайдер в [TTS] (issue #1229)
# ─────────────────────────────────────────────────────────────────────────────

class TestBuildDynamicSystemContextTtsProvider:
    """LLM-контекст [TTS] строится по ФАКТИЧЕСКОМУ провайдеру.

    tts_node публикует реального провайдера после фолбека (minimax→yandex);
    dialogue_node должен показывать LLM голоса РЕАЛЬНОГО провайдера, а не
    номинального из параметра tts_provider (иначе LLM выбирает minimax-голоса,
    которых нет у yandex, и робот говорит тем же голосом).
    """

    def test_nominal_provider_without_state(self, node):
        """Нет provider_state → параметр tts_provider (minimax)."""
        ctx = node._build_dynamic_system_context()
        assert "<tts_provider>minimax</tts_provider>" in ctx
        assert "provider: minimax" in ctx

    def test_actual_provider_overrides_param(self, node):
        """Фолбек minimax→yandex: context показывает yandex + его голоса."""
        node._actual_tts_provider = "yandex"
        ctx = node._build_dynamic_system_context()
        assert "<tts_provider>yandex</tts_provider>" in ctx
        assert "provider: yandex" in ctx
        # голоса РЕАЛЬНОГО провайдера (yandex), а не minimax
        assert "anton" in ctx
        assert "male-qn-qingse" not in ctx

    def test_actual_voice_shown_in_context(self, node):
        """tts_node сообщил фактический голос (anton после фолбека) — показываем."""
        node._actual_tts_provider = "yandex"
        node._actual_tts_voice = "zahar"
        ctx = node._build_dynamic_system_context()
        assert "current_voice: zahar" in ctx

    def test_actual_provider_resets_to_param(self, node):
        """Провайдер «ожил» (minimax снова работает) — context снова minimax."""
        node._actual_tts_provider = "yandex"
        node._build_dynamic_system_context()
        node._actual_tts_provider = None
        ctx = node._build_dynamic_system_context()
        assert "provider: minimax" in ctx


# ─────────────────────────────────────────────────────────────────────────────
#  _build_music_state_snapshot — issue #1544
# ─────────────────────────────────────────────────────────────────────────────

class _FakeDJController:
    """Замена ``DJModeController`` для unit-тестов (issue #1544).

    Реальный класс дёргает ``DJHook.dispatch`` при инициализации, что
    ломает test fixture. Для snapshot'а нужны только ``state.enabled`` и
    ``state.theme`` — подменяем контроллер минимальным стабом.
    """

    def __init__(self, *, enabled: bool = False, theme: str = "") -> None:
        # Тот же dataclass, что в dj_mode.DJModeController.state.
        self.state = type("S", (), {"enabled": enabled, "theme": theme})()


class TestBuildMusicStateSnapshot:
    """Единый <music_state> snapshot для LLM (issue #1544).

    До фикса LLM не знал, играет DJ-бит или AI-генерация, и на команду
    «стоп музыку» отвечал verbal-only (без вызова stop_music tool).
    Снимок обязан показывать ВСЕ четыре источника истины:

    * ``dj``  — DJ-режим (theme или off)
    * ``ai``  — AI-сгенерированная музыка (title или idle)
    * ``beat`` — активный Renardo-бит (active/silent)
    * ``cleanup`` — отложен ли music_cleanup (pending/none)

    Все атрибуты рендерятся всегда — LLM не должен угадывать по
    отсутствию тега.
    """

    def test_all_idle_when_nothing_plays(self, node) -> None:
        """Базовое состояние: всё молчит, snapshot показывает off/idle/silent."""
        # node fixture уже инициализирует _dj=None, _pending_music_cleanup=False,
        # _active_batches={}, _generated_music_state=None.
        snap = node._build_music_state_snapshot()
        assert 'dj="off"' in snap
        assert 'ai="idle"' in snap
        assert 'beat="silent"' in snap
        assert 'cleanup="none"' in snap
        # Корневой тег — один блок, не несколько.
        assert snap.count("<music_state") == 1
        assert snap.count("/>") == 1

    def test_dj_active_with_theme(self, node) -> None:
        """DJ-режим ON с theme → ``dj=playing: <theme>``."""
        node._dj = _FakeDJController(enabled=True, theme="lofi study")
        snap = node._build_music_state_snapshot()
        assert 'dj="playing: lofi study"' in snap

    def test_dj_active_without_theme_falls_back_to_unknown(self, node) -> None:
        """DJ ON, но theme пустой → ``playing: unknown theme`` (не пустота)."""
        node._dj = _FakeDJController(enabled=True, theme="")
        snap = node._build_music_state_snapshot()
        assert 'dj="playing: unknown theme"' in snap

    def test_ai_generated_music_playing(self, node) -> None:
        """Топик /voice/generated_music/state сообщил playing → ``ai=playing: ...``."""
        node._generated_music_state = {
            "status": "playing",
            "title": "Synthwave Sunset",
            "track_id": "abc12345xyz",
        }
        snap = node._build_music_state_snapshot()
        assert 'ai="playing: Synthwave Sunset"' in snap

    def test_ai_generated_music_idle(self, node) -> None:
        """AI-music payload без status=playing → ``ai=idle``."""
        node._generated_music_state = {"status": "idle"}
        snap = node._build_music_state_snapshot()
        assert 'ai="idle"' in snap

    def test_beat_active_via_pending_cleanup(self, node) -> None:
        """LLM вызвал execute_music_code → cleanup pending → beat=active.

        Это САМЫЙ частый кейс для dj02 (issue #1544): LLM запустил Renardo-
        бит, cleanup ещё не fired (ждёт tts_batch_complete), юзер говорит
        «выключи музыку» → snapshot ОБЯЗАН показать beat=active, иначе LLM
        скажет «уже выключено» вместо stop_music.
        """
        node._pending_music_cleanup = True
        snap = node._build_music_state_snapshot()
        assert 'beat="active"' in snap
        assert 'cleanup="pending"' in snap

    def test_beat_active_via_active_batches(self, node) -> None:
        """TTS-батчи ещё живут → бит держится под вокал (backing mode)."""
        node._active_batches = {"batch-1": 2, "batch-2": 3}
        snap = node._build_music_state_snapshot()
        assert 'beat="active"' in snap
        # Cleanup ещё не scheduled, но батчи есть — бит держится.
        assert 'cleanup="none"' in snap

    def test_all_three_playing_simultaneously(self, node) -> None:
        """DJ + AI + beat — все три источника активны (теоретический edge case)."""
        node._dj = _FakeDJController(enabled=True, theme="ambient")
        node._generated_music_state = {
            "status": "playing",
            "title": "AI Beat",
            "track_id": "deadbeef",
        }
        node._pending_music_cleanup = True
        snap = node._build_music_state_snapshot()
        assert 'dj="playing: ambient"' in snap
        assert 'ai="playing: AI Beat"' in snap
        assert 'beat="active"' in snap
        assert 'cleanup="pending"' in snap

    def test_theme_with_quotes_is_xml_escaped(self, node) -> None:
        """Тема с кавычками/амперсандами НЕ ломает XML атрибут."""
        # Если escape не сработает — pytest парсинг XML через поиск
        # сломается (часть атрибута «отвалится»).
        node._dj = _FakeDJController(
            enabled=True,
            theme='Techno "rave" & more',
        )
        snap = node._build_music_state_snapshot()
        # Сырые кавычки/амперсанд внутри dj-значения → экранируются.
        assert "&quot;rave&quot;" in snap
        assert "&amp; more" in snap
        # Сырых символов быть не должно.
        assert 'theme="Techno "rave"' not in snap


# ─────────────────────────────────────────────────────────────────────────────
#  _build_dynamic_system_context — issue #1544 music_state в system_context
# ─────────────────────────────────────────────────────────────────────────────

class TestBuildDynamicSystemContextMusicState:
    """Issue #1544: <music_state> появляется внутри <system_context>."""
    def test_snapshot_included_in_system_context(self, node) -> None:
        """<system_context> содержит <music_state …/> блок."""
        ctx = node._build_dynamic_system_context()
        assert "<music_state" in ctx
        assert "</system_context>" in ctx
        # Всегда есть reminder о том, как реагировать на «стоп музыку».
        assert "Если юзер говорит «стоп музыку" in ctx

    def test_dj_active_reflected_in_system_context(self, node) -> None:
        """DJ-режим виден в system_context (не только в snapshot helper)."""
        node._dj = _FakeDJController(enabled=True, theme="deep house")
        ctx = node._build_dynamic_system_context()
        assert 'dj="playing: deep house"' in ctx

    def test_beat_pending_reflected_in_system_context(self, node) -> None:
        """Pending cleanup отражается в system_context."""
        node._pending_music_cleanup = True
        ctx = node._build_dynamic_system_context()
        assert 'cleanup="pending"' in ctx
        assert 'beat="active"' in ctx

    def test_legacy_generated_music_tag_replaced(self, node) -> None:
        """Issue #1392 follow-up: <generated_music> тег УДАЛЁН (заменён на <music_state>)."""
        # AI-gen музыка играет, но snapshot идёт через <music_state>, не через
        # старый <generated_music>.
        node._generated_music_state = {
            "status": "playing",
            "title": "AI track",
            "track_id": "abc12345",
        }
        ctx = node._build_dynamic_system_context()
        assert "<generated_music>" not in ctx
        assert 'ai="playing: AI track"' in ctx
