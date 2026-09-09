"""Unit-тесты для AvatarSupervisor ROS 2 ноды (голос + супервизор-агент).

После #1987 (ADR-0051 §2.2) AvatarSupervisor отвечает ТОЛЬКО за голос и
агент оператора: арбитраж floor/FSM + /avatar/state вынесены в отдельную
ноду ``avatar_arbiter`` (см. test_arbiter_node.py). Здесь мы тестируем то,
что осталось в этой ноде:

- Нода создаётся с name="avatar_supervisor" и параметром mode="monitor".
- Voice-управление dialogue_node (ADR-0028 S5 / ADR-0066 §6.7): после
  удаления ``voice_input_mode`` супервизор управляет личностью через
  топик ``/dialogue/control``. ``/avatar/set_voice_mode`` остался для
  legacy-контракта (UI Quest, web-admin) — маппится на pause/resume.
- Нода НЕ регистрирует floor-сервисы и НЕ публикует /avatar/state
  (арбитраж — в avatar_arbiter), НЕ правит twist_mux.

Агент оператора (AV-21) тестируется отдельно в test_avatar_agent.py.
"""
from __future__ import annotations

import json
import pathlib
import unittest
from unittest.mock import MagicMock

from rob_box_supervisor.supervisor_node import (
    DIALOGUE_CONTROL_ACTIONS,
    DIALOGUE_CONTROL_PAUSE,
    DIALOGUE_CONTROL_RESUME,
    DIALOGUE_CONTROL_TOPIC,
    GRIP_DEFAULT_LANGUAGE,
    MONITOR_MODE_REASON,
    SET_VOICE_LANGUAGE_TOPIC,
    SET_VOICE_MODE_TOPIC,
    SET_VOICE_PRESET_TOPIC,
    VOICE_LANGUAGES,
    VOICE_PRESET_IDS,
    AvatarSupervisor,
)


def _make_string_msg(data: str) -> MagicMock:
    """Создать фейковый std_msgs/String с .data."""
    m = MagicMock()
    m.data = data
    return m


class TestAvatarSupervisorCreation(unittest.TestCase):
    def test_node_name_is_avatar_supervisor(self) -> None:
        node = AvatarSupervisor()
        try:
            self.assertEqual(node.get_name(), "avatar_supervisor")
        finally:
            node.destroy_node()

    def test_mode_parameter_defaults_to_monitor(self) -> None:
        node = AvatarSupervisor()
        try:
            self.assertEqual(node.get_parameter("mode").value, "monitor")
            self.assertTrue(node.has_parameter("mode"))
        finally:
            node.destroy_node()

    def test_llm_providers_defaults_minimax_deepseek(self) -> None:
        """Issue #2111: дефолт ``llm_providers`` = ``minimax,deepseek``.

        Исторически был ``"deepseek"`` — и supervisor поднимался с одним
        провайдером без API-ключа, agent не отвечал. После фикса дефолт
        повторяет ``dialogue_node`` (там тот же CSV зашит в
        ``declare_parameter`` + runtime yaml). Тест ловит регрессию при
        следующем «sync chain ordering» PR, чтобы не пришлось снова
        ловить через e2e.

        ADR-0043 §3.2: если кто-то меняет default chain — этот тест и
        ``test_dialogue_node.py::test_resolve_provider_chain_parses_csv``
        должны обновляться в одном коммите.
        """
        node = AvatarSupervisor()
        try:
            self.assertTrue(node.has_parameter("llm_providers"))
            self.assertEqual(
                node.get_parameter("llm_providers").value, "minimax,deepseek"
            )
        finally:
            node.destroy_node()


class TestAvatarSupervisorDoesNotMutateExternalState(unittest.TestCase):
    """После #1987 супервизор (голос+агент) НЕ трогает floor/твист-мукс/аватар-состояние."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_no_floor_services_registered(self) -> None:
        """Арбитраж уехал в avatar_arbiter: floor-сервисы здесь НЕ регистрируются."""
        names = [s.name for s in self.node._services]
        self.assertNotIn("/avatar_arbiter/acquire_floor", names)
        self.assertNotIn("/avatar_arbiter/release_floor", names)
        self.assertNotIn("/avatar_arbiter/set_avatar_mode", names)
        # относительные имена (корень) тоже не должны появляться
        self.assertNotIn("acquire_floor", names)
        self.assertNotIn("release_floor", names)
        self.assertNotIn("set_avatar_mode", names)

    def test_no_avatar_state_publisher(self) -> None:
        """/avatar/state публикует avatar_arbiter, а не супервизор."""
        self.assertNotIn("/avatar/state", self.node._publishers)

    def test_no_twist_mux_publisher(self) -> None:
        """Нода НЕ публикует cmd_vel_* напрямую."""
        for t in self.node._publishers:
            self.assertNotIn("cmd_vel", t)
            self.assertNotIn("twist_mux", t)

    def test_no_set_parameter_calls_for_dialogue_via_pubs(self) -> None:
        """Нет publisher-ов на /voice/ (кроме /voice/tts/request, шаг 4б, и
        /voice/tts/set_voice, voice-vr 21 / ADR-0080 §2.7) +
        в /dialogue/ — только ``/dialogue/control`` (ADR-0066 §6.7,
        pause/resume для личности). Голос-параметры — через топик
        /voice/tts/set_voice (НЕ записью в чужие ROS-параметры), голос
        личности — yaml-direct через ``grip_pipeline``.
        """
        voice_pubs = [t for t in self.node._publishers if t.startswith("/voice/")]
        self.assertEqual(
            sorted(voice_pubs),
            sorted(["/voice/tts/request", "/voice/tts/set_voice"]),
        )
        dialogue_pubs = [
            t for t in self.node._publishers if t.startswith("/dialogue/")
        ]
        self.assertEqual(dialogue_pubs, ["/dialogue/control"])

    def test_log_startup_diagnostics_uses_single_msg_arg(self) -> None:
        """Регресс #1644: ``_log.info`` получает ОДИН строковый msg.

        После #1987 строка diagnostics больше НЕ содержит ``typed_services=``
        (это поле уехало в avatar_arbiter).
        """
        self.node._log.reset_mock()
        self.node._log_startup_diagnostics()
        self.assertTrue(self.node._log.info.called)
        call = self.node._log.info.call_args
        self.assertEqual(len(call.args), 1)
        msg = call.args[0]
        self.assertIsInstance(msg, str)
        self.assertIn("avatar_supervisor started", msg)
        self.assertIn(f"mode={self.node._mode}", msg)
        self.assertNotIn("typed_services=", msg)
        self.assertNotIn("msgpack=", msg)
        self.assertEqual(call.kwargs, {})


class TestAvatarSupervisorVoiceMode(unittest.TestCase):
    """ADR-0066 §6.7 — супервизор управляет личностью через ``/dialogue/control``.

    После удаления ``voice_input_mode`` (ADR-0066 §6) параметр
    ``voice_input_mode`` на dialogue_node больше не существует, и супервизор
    вместо ``_set_dialogue_param`` публикует JSON в ``/dialogue/control``.
    Тесты проверяют:
      * Legacy-контракт ``/avatar/set_voice_mode`` маппится на
        ``resume`` (``respeaker``) / ``pause`` (``off``).
      * Все остальные режимы (quest_*, …) — отвергаются с
        ``reason="voice_mode_deprecated: ..."`` (ADR-0018 — честный FAIL).
      * В monitor-режиме ничего не публикуется (S12).
      * Publisher ``/dialogue/control`` объявлен и используется.
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    # ── topology ─────────────────────────────────────────────────────
    def test_set_voice_mode_topic_subscribed(self) -> None:
        topics = [s.topic for s in self.node._subscriptions]
        self.assertIn(SET_VOICE_MODE_TOPIC, topics)

    def test_dialogue_control_publisher_declared(self) -> None:
        """ADR-0066 §6.7 — супервизор публикует в ``/dialogue/control``."""
        self.assertIn(DIALOGUE_CONTROL_TOPIC, self.node._publishers)
        self.assertEqual(DIALOGUE_CONTROL_TOPIC, "/dialogue/control")

    def test_legacy_mode_mapping(self) -> None:
        """``LEGACY_VOICE_MODE_TO_ACTION`` — маппинг ``respeaker→resume``,
        ``off→pause``. Тест-инвариант: словарь полный, ключи только эти два."""
        self.assertEqual(
            set(self.node.LEGACY_VOICE_MODE_TO_ACTION.keys()),
            {"respeaker", "off"},
        )
        self.assertEqual(
            self.node.LEGACY_VOICE_MODE_TO_ACTION["respeaker"],
            DIALOGUE_CONTROL_RESUME,
        )
        self.assertEqual(
            self.node.LEGACY_VOICE_MODE_TO_ACTION["off"],
            DIALOGUE_CONTROL_PAUSE,
        )

    # ── monitor-mode: legacy contract accepted, but NO publish (S12) ──
    def test_monitor_mode_respeaker_publishes_nothing(self) -> None:
        """S12: monitor-режим принимает команду, но НЕ публикует в
        ``/dialogue/control`` — мы не вмешиваемся в чужие параметры/топики."""
        pub = self.node._dialogue_control_pub
        published_before = len(pub.published)
        applied, reason = self.node._apply_voice_mode("respeaker")
        self.assertFalse(applied)
        self.assertEqual(reason, MONITOR_MODE_REASON)
        self.assertEqual(len(pub.published), published_before)

    # ── active-mode: legacy contract → publish pause/resume ─────────
    def _published_pauses(self, node) -> list[dict]:
        """Достать все ``pause``-payload из опубликованного в ``/dialogue/control``."""
        pub = node._dialogue_control_pub
        out = []
        for msg in pub.published:
            payload = json.loads(msg.data)
            if payload.get("action") == DIALOGUE_CONTROL_PAUSE:
                out.append(payload)
        return out

    def _published_resumes(self, node) -> list[dict]:
        pub = node._dialogue_control_pub
        out = []
        for msg in pub.published:
            payload = json.loads(msg.data)
            if payload.get("action") == DIALOGUE_CONTROL_RESUME:
                out.append(payload)
        return out

    def test_active_mode_respeaker_publishes_resume(self) -> None:
        """``respeaker`` в active → publish ``resume`` на ``/dialogue/control``."""
        self.node._mode = "active"
        applied, reason = self.node._apply_voice_mode("respeaker")
        self.assertTrue(applied, f"expected applied=True, got reason={reason!r}")
        self.assertEqual(reason, "applied")
        resumes = self._published_resumes(self.node)
        self.assertEqual(len(resumes), 1)
        self.assertEqual(resumes[0]["action"], DIALOGUE_CONTROL_RESUME)
        self.assertIn("legacy_set_voice_mode:respeaker", resumes[0]["reason"])
        self.assertIn("ts_s", resumes[0])
        # Никаких pause-ов для respeaker.
        self.assertEqual(len(self._published_pauses(self.node)), 0)

    def test_active_mode_off_publishes_pause(self) -> None:
        """``off`` в active → publish ``pause`` (W3-1 эквивалент)."""
        self.node._mode = "active"
        applied, reason = self.node._apply_voice_mode("off")
        self.assertTrue(applied, f"expected applied=True, got reason={reason!r}")
        self.assertEqual(reason, "applied")
        pauses = self._published_pauses(self.node)
        self.assertEqual(len(pauses), 1)
        self.assertEqual(pauses[0]["action"], DIALOGUE_CONTROL_PAUSE)
        self.assertIn("legacy_set_voice_mode:off", pauses[0]["reason"])

    # ── removed legacy values → reject with voice_mode_deprecated ────
    def test_quest_ttts_rejected_as_deprecated(self) -> None:
        """``quest_ttts`` (и прочие quest_*) — больше не поддерживаются.
        Отвергаем с ``reason="voice_mode_deprecated: ..."`` без публикации.
        """
        self.node._mode = "active"
        pub = self.node._dialogue_control_pub
        published_before = len(pub.published)
        applied, reason = self.node._apply_voice_mode("quest_ttts")
        self.assertFalse(applied)
        self.assertIn("voice_mode_deprecated", reason)
        self.assertIn("quest_ttts", reason)
        # Ничего не опубликовано.
        self.assertEqual(len(pub.published), published_before)

    def test_quest_stt_rejected_as_deprecated(self) -> None:
        self.node._mode = "active"
        applied, reason = self.node._apply_voice_mode("quest_stt")
        self.assertFalse(applied)
        self.assertIn("voice_mode_deprecated", reason)

    def test_unknown_mode_rejected_as_deprecated(self) -> None:
        """Произвольный мусор — отвергаем с тем же reason."""
        self.node._mode = "active"
        applied, reason = self.node._apply_voice_mode("totally_made_up")
        self.assertFalse(applied)
        self.assertIn("voice_mode_deprecated", reason)

    # ── /avatar/set_voice_mode topic handler ─────────────────────────
    def test_on_set_voice_mode_feeds_apply(self) -> None:
        """Топик → ``_apply_voice_mode``; в monitor отдаёт monitor_reason."""
        self.node._apply_voice_mode = MagicMock(
            return_value=(False, MONITOR_MODE_REASON)
        )
        self.node._on_set_voice_mode(_make_string_msg("respeaker"))
        self.node._apply_voice_mode.assert_called_once_with("respeaker")


class TestAvatarSupervisorVoicePresetsAndLanguage(unittest.TestCase):
    """AV-28 §P7 — супервизор владеет voice_preset + voice_output_language.

    Маршрут: UI → ws_server.set_voice → Bridge → /avatar/set_voice_preset
    (или _language) → supervisor (валидация whitelist + фиксация в
    grip_pipeline.yaml-direct, voice-vr 21 / ADR-0080 §2.7 — больше
    НЕ пишет в чужие ROS-параметры).
    Симметрично TestAvatarSupervisorVoiceMode (выше), но для параметров
    стиля речи и языка вывода, которые появились в Phase 3 (AV-28).
    """

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_preset_topic_subscribed(self) -> None:
        topics = [s.topic for s in self.node._subscriptions]
        self.assertIn(SET_VOICE_PRESET_TOPIC, topics)
        self.assertIn(SET_VOICE_LANGUAGE_TOPIC, topics)

    def test_monitor_mode_does_not_apply_preset(self) -> None:
        """В monitor супервизор принимает preset, но НЕ применяет (S12)."""
        applied, reason = self.node._apply_voice_preset("lenin")
        self.assertFalse(applied)
        self.assertEqual(reason, MONITOR_MODE_REASON)

    def test_monitor_mode_does_not_apply_language(self) -> None:
        applied, reason = self.node._apply_voice_language("en")
        self.assertFalse(applied)
        self.assertEqual(reason, MONITOR_MODE_REASON)

    def test_invalid_preset_rejected(self) -> None:
        """Не-whitelisted preset отвергается — UI получит NACK на сервере,
        а здесь на supervisor-стороне ловим как ``invalid_voice_preset``."""
        self.node._mode = "active"
        applied, reason = self.node._apply_voice_preset("not_a_preset")
        self.assertFalse(applied)
        self.assertIn("invalid_voice_preset", reason)

    def test_invalid_language_rejected(self) -> None:
        self.node._mode = "active"
        applied, reason = self.node._apply_voice_language("xx")
        self.assertFalse(applied)
        self.assertIn("invalid_voice_language", reason)

    def test_whitelists_match_ws_server_and_yaml(self) -> None:
        """Whitelist'ы = ws_server.VOICE_* = ``rob_box_core.bridge_protocol``
        (здесь SoT для ``voice_presets.yaml``) = yaml-источник.

        voice-vr 21: один источник (``bridge_protocol``), и ws_server /
        supervisor импортируют его же, а не держат локальную копию.
        Разъехавшись, они давали молчаливый отказ: ws_server отвечал
        Quest'у ack, а супервизор ронял запрос в applied=False. Так уехали
        ``translate`` и языки fr/de/zh/hi — оператор жал кнопку, UI
        подсвечивал выбор, робот его не получал.
        """
        import yaml

        from rob_box_quest.server.ws_server import (
            VOICE_LANGUAGES as WS_LANGUAGES,
            VOICE_PRESET_IDS as WS_PRESETS,
        )
        from rob_box_core.bridge_protocol import (
            VOICE_LANGUAGES as CATALOG_LANGUAGES,
            VOICE_PRESET_IDS as CATALOG_PRESETS,
        )

        yaml_path = (
            pathlib.Path(__file__).resolve().parents[3]
            / "rob_box_voice"
            / "config"
            / "voice_presets.yaml"
        )
        data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        yaml_presets = set(data["presets"].keys())
        yaml_languages = {str(code).lower() for code in data["languages"]}
        # 1. YAML — финальный источник истины.
        self.assertEqual(yaml_presets, set(VOICE_PRESET_IDS))
        self.assertEqual(yaml_languages, set(VOICE_LANGUAGES))
        # 2. Каталог (bridge_protocol) — переэкспорт этой же константы,
        #    должен быть биткомпактен с YAML (иначе codegen рассинхронится).
        self.assertEqual(yaml_presets, set(CATALOG_PRESETS))
        self.assertEqual(yaml_languages, set(CATALOG_LANGUAGES))
        # 3. ws_server — это symlink ``bridge_protocol``,
        #    должен быть идентичен ему (адрес регресс — три копии списка).
        self.assertEqual(set(WS_PRESETS), set(CATALOG_PRESETS))
        self.assertEqual(set(WS_LANGUAGES), set(CATALOG_LANGUAGES))
        # 4. Класс валидирует ровно этими списками — второй копии
        #    на классе быть не должно (ранее был ``_AV28_*``, который
        #    и разъезжался с ws_server).
        self.assertEqual(set(self.node._AV28_PRESET_IDS), set(VOICE_PRESET_IDS))
        self.assertEqual(set(self.node._AV28_LANGUAGES), set(VOICE_LANGUAGES))

    def test_grip_default_language_is_re_export_of_catalog(self) -> None:
        """issue #2265 — дефолтный язык пайплайна живёт в одном месте.

        ``GRIP_DEFAULT_LANGUAGE`` должен быть тем же объектом, что
        ``rob_box_core.bridge_protocol.VOICE_PIPELINE_DEFAULT_LANGUAGE``
        (прямой импорт через алиас — тот же приём, что для
        ``VOICE_PRESET_IDS`` выше в этом тест-классе).

        До фикса это были две независимые константы «ru» в двух
        модулях; простой assertEqual пройдёт даже на копии, поэтому
        проверяем ``is`` — гарантирует, что переменная не переприсвоена.
        """
        from rob_box_core.bridge_protocol import (
            VOICE_PIPELINE_DEFAULT_LANGUAGE as CATALOG_DEFAULT_LANG,
        )
        self.assertIs(
            GRIP_DEFAULT_LANGUAGE,
            CATALOG_DEFAULT_LANG,
            "GRIP_DEFAULT_LANGUAGE оторвался от канона — "
            "верни прямой импорт из rob_box_core.bridge_protocol "
            "(ADR-0080 §2.7 / issue #2265).",
        )
        self.assertEqual(GRIP_DEFAULT_LANGUAGE, "ru")
        self.assertIn(GRIP_DEFAULT_LANGUAGE, VOICE_LANGUAGES)

    def test_empty_preset_rejected(self) -> None:
        """Пустой payload — это битый UI; не пытаемся выставить
        пустую строку параметром (dialogue_node упадёт)."""
        self.node._mode = "active"
        applied, reason = self.node._apply_voice_preset("")
        self.assertFalse(applied)
        self.assertEqual(reason, "empty_voice_preset")

    def test_empty_language_rejected(self) -> None:
        self.node._mode = "active"
        applied, reason = self.node._apply_voice_language("")
        self.assertFalse(applied)
        self.assertEqual(reason, "empty_voice_language")

    def test_active_mode_dispatches_preset(self) -> None:
        """voice-vr 21: валидный preset в active → ``applied`` без
        записи в чужие ROS-параметры dialogue_node (ADR-0080 §2.7).
        Живой путь — ``grip_pipeline`` yaml-direct, поэтому проверяем
        только факт валидации whitelist'а + сигнатуру возврата; никаких
        побочных эффектов на mock-rclpy быть не должно.
        """
        self.node._mode = "active"
        applied, reason = self.node._apply_voice_preset("philosopher")
        self.assertTrue(applied)
        self.assertEqual(reason, "applied")

    def test_active_mode_dispatches_language(self) -> None:
        self.node._mode = "active"
        applied, reason = self.node._apply_voice_language("en")
        self.assertTrue(applied)
        self.assertEqual(reason, "applied")

    def test_on_set_voice_preset_feeds_apply(self) -> None:
        """Топик → _apply_voice_preset; в monitor применяется=false."""
        self.node._apply_voice_preset = MagicMock(
            return_value=(False, MONITOR_MODE_REASON)
        )
        self.node._on_set_voice_preset(_make_string_msg("lenin"))
        self.node._apply_voice_preset.assert_called_once_with("lenin")

    def test_on_set_voice_language_feeds_apply(self) -> None:
        self.node._apply_voice_language = MagicMock(
            return_value=(False, MONITOR_MODE_REASON)
        )
        self.node._on_set_voice_language(_make_string_msg("ru"))
        self.node._apply_voice_language.assert_called_once_with("ru")

    def test_active_mode_no_set_parameters_call(self) -> None:
        """voice-vr 21: ``_apply_voice_*`` НЕ пишет в чужие ROS-параметры
        (визуальная регрессия на ADR-0080 §2.7 — единственная живая
        проточка параметров — tts_node picker через /voice/tts/set_voice
        топик, не через клиент записи). Раньше были
        ``_set_dialogue_param`` + ``_set_tts_voice_param`` mocks;
        теперь атрибуты вообще не должны существовать на классе.
        """
        self.node._mode = "active"
        # Атрибут не должен существовать ни на инстансе, ни на классе.
        self.assertFalse(
            hasattr(self.node, "_set_dialogue_param"),
            "voice-vr 21: _set_dialogue_param удалён; "
            "supervisor не пишет в чужие ROS-параметры",
        )
        # voice-vr 21 / ADR-0080 §2.7: то же для tts_node.
        self.assertFalse(
            hasattr(self.node, "_set_tts_voice_param"),
            "voice-vr 21: _set_tts_voice_param удалён; "
            "supervisor пишет в tts_node через /voice/tts/set_voice, "
            "а не клиентом записи параметров",
        )
        # Старое поведение, которое роняло узел через param_set_failed,
        # теперь недостижимо. Гарантируем, что _apply всё равно не падает
        # (нет шинного RPC, поэтому нечего ловить).
        applied, reason = self.node._apply_voice_preset("street")
        self.assertTrue(applied)
        self.assertEqual(reason, "applied")

    def test_apply_set_voice_publishes_to_tts_set_voice_topic(self) -> None:
        """voice-vr 21 / ADR-0080 §2.7 — ``_apply_set_voice`` в active
        публикует JSON в ``/voice/tts/set_voice`` (НЕ клиентом записи).
        Регрессия на регрессию: после удаления ``_set_tts_voice_param``
        живой write-side для голоса tts_node — это топик-контракт.
        """
        from unittest.mock import MagicMock

        from rob_box_supervisor.supervisor_node import SET_TTS_VOICE_TOPIC

        self.node._mode = "active"
        # Подменяем publisher на MagicMock чтобы перехватить publish().
        self.node._set_voice_tts_pub = MagicMock()
        applied, reason = self.node._apply_set_voice("alena", provider_hint="yandex")
        self.assertTrue(applied)
        self.assertEqual(reason, "applied:yandex")
        # Проверяем что publish был вызван с правильным JSON.
        self.node._set_voice_tts_pub.publish.assert_called_once()
        # Извлекаем RosString.data из вызова.
        call_args = self.node._set_voice_tts_pub.publish.call_args
        ros_msg = call_args[0][0]
        import json as _json
        payload = _json.loads(ros_msg.data)
        self.assertEqual(payload["voice_id"], "alena")
        self.assertEqual(payload["provider"], "yandex")
        self.assertEqual(payload["source"], "set_voice")
        # topic объявлен в SET_TTS_VOICE_TOPIC.
        self.assertEqual(SET_TTS_VOICE_TOPIC, "/voice/tts/set_voice")

    def test_no_set_parameters_call_to_dialogue_node_in_source(self) -> None:
        """voice-vr 21 / ADR-0080 §2.7 — DoD-критерий: в исходниках
        ``AvatarSupervisor`` не должно быть НИ ОДНОГО живого
        клиента записи в чужие ROS-параметры на
        ``/dialogue_node/set_parameters`` или
        ``/tts_node/set_parameters``.

        Это регрессия на регрессию: до #2243 (голос-vr 21) были
        ``_set_dialogue_param`` + ``_set_tts_voice_param`` с ленивыми
        клиентами к этим сервисам, которые слали ``voice_preset``/
        ``voice_output_language`` (мёртвая запись — ADR-0066 §6.3,
        dialogue_node только логировал) и ``yandex_voice``/
        ``minimax_voice``/``silero_speaker`` (живая запись, но
        сцеплённая с внутренней схемой имён tts_node).
        «Мёртвый»/«знание-схемы» write-side — кандидат на повторную
        регрессию при будущих карточках, ловим текстовым grep.
        """
        import pathlib

        # Ищем ``src/rob_box_supervisor/rob_box_supervisor/supervisor_node.py``
        # относительно файла теста:
        #   parents[0] = test/unit/
        #   parents[1] = test/
        #   parents[2] = src/rob_box_supervisor/      ← тут лежит «наш» пакет
        #   parents[3] = src/
        # Поэтому идём через ``parents[2]`` + ``rob_box_supervisor/``.
        src_path = (
            pathlib.Path(__file__).resolve().parents[2]
            / "rob_box_supervisor"
            / "supervisor_node.py"
        )
        if not src_path.is_file():
            # Fallback для редкого cwd=src/rob_box_supervisor/test/unit.
            src_path = pathlib.Path(__file__).resolve().parents[1] / "supervisor_node.py"
        # Читаем исходник и выделяем «живые» строки (без комментариев и
        # docstrings) — иначе любой docstring про запись в чужие
        # параметры роняет тест.
        live_lines = []
        for raw in src_path.read_text(encoding="utf-8").splitlines():
            stripped = raw.strip()
            if not stripped or stripped.startswith("#"):
                continue
            # Простое определение docstring (грубое, но достаточное для
            # нашего файла — он не использует nested docstring):
            if stripped.startswith(('"""', "'''")):
                continue
            live_lines.append(stripped)
        live = "\n".join(live_lines)
        # DoD: write-side в чужие ROS-параметры не должен встречаться
        # как живой код. Конкретно:
        # * нет /dialogue_node/set_parameters (был _set_dialogue_param);
        # * нет /tts_node/set_parameters (был _set_tts_voice_param);
        # * нет create_client(.../set_parameters) вообще.
        for forbidden in (
            "/dialogue_node/set_parameters",
            "/tts_node/set_parameters",
            "_set_dialogue_param",
            "_set_tts_voice_param",
            "_voice_param_key_for",
            "_tts_param_client",
            "_dialogue_param_client",
            '"/dialogue_node/set_parameters"',
            '"/tts_node/set_parameters"',
        ):
            self.assertNotIn(
                forbidden,
                live,
                f"voice-vr 21: forbidden live reference to {forbidden!r} "
                f"in supervisor_node.py — supervisor должен перестать "
                f"писать в чужие ROS-параметры (ADR-0080 §2.7)",
            )


if __name__ == "__main__":
    unittest.main()
