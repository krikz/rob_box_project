"""Unit-тесты для AvatarSupervisor ROS 2 ноды (голос + супервизор-агент).

После #1987 (ADR-0051 §2.2) AvatarSupervisor отвечает ТОЛЬКО за голос и
агент оператора: арбитраж floor/FSM + /avatar/state вынесены в отдельную
ноду ``avatar_arbiter`` (см. test_arbiter_node.py). Здесь мы тестируем то,
что осталось в этой ноде:

- Нода создаётся с name="avatar_supervisor" и параметром mode="monitor".
- Voice-управление dialogue_node (ADR-0028 S5): /avatar/set_voice_mode,
  preset/language (AV-28), set_voice/preview (AV-27) — в monitor применяется
  false (S12), в active — SetParameters на dialogue_node.
- Нода НЕ регистрирует floor-сервисы и НЕ публикует /avatar/state
  (арбитраж — в avatar_arbiter), НЕ правит twist_mux.

Агент оператора (AV-21) тестируется отдельно в test_avatar_agent.py.
"""
from __future__ import annotations

import pathlib
import unittest
from unittest.mock import MagicMock

from rob_box_supervisor.supervisor_node import (
    MONITOR_MODE_REASON,
    SET_VOICE_LANGUAGE_TOPIC,
    SET_VOICE_MODE_TOPIC,
    SET_VOICE_PRESET_TOPIC,
    VOICE_INPUT_MODES,
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
        """Нет publisher-ов на /dialogue/ или лишних /voice/ (голос-параметры —
        через параметр-клиенты под mode=active, не топики).

        Единственное исключение — /voice/tts/request (шаг 4б, issue #1989):
        пайплайн грипа публикует туда текст оператора (динамики робота).
        """
        voice_pubs = [t for t in self.node._publishers if t.startswith("/voice/")]
        self.assertEqual(voice_pubs, ["/voice/tts/request"])
        for topic in self.node._publishers:
            self.assertFalse(topic.startswith("/dialogue/"))

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
    """ADR-0028 S5 — супервизор владеет voice_input_mode (Phase 1)."""

    def setUp(self) -> None:
        self.node = AvatarSupervisor()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_set_voice_mode_topic_subscribed(self) -> None:
        topics = [s.topic for s in self.node._subscriptions]
        self.assertIn(SET_VOICE_MODE_TOPIC, topics)

    def test_monitor_mode_does_not_apply(self) -> None:
        """В monitor супервизор принимает режим, но НЕ применяет (S12)."""
        applied, reason = self.node._apply_voice_mode("quest_ttts")
        self.assertFalse(applied)
        self.assertEqual(reason, MONITOR_MODE_REASON)

    def test_invalid_mode_rejected(self) -> None:
        applied, reason = self.node._apply_voice_mode("not_a_mode")
        self.assertFalse(applied)
        self.assertIn("invalid_voice_mode", reason)

    def test_active_mode_dispatches_param_set(self) -> None:
        """В active режиме валидный режим → _set_dialogue_param вызывается."""
        self.node._mode = "active"
        self.node._set_dialogue_param = MagicMock()
        applied, reason = self.node._apply_voice_mode("quest_ttts")
        self.assertTrue(applied)
        self.assertEqual(reason, "applied")
        self.node._set_dialogue_param.assert_called_once_with("voice_input_mode", "quest_ttts")

    def test_on_set_voice_mode_feeds_apply(self) -> None:
        """Топик → _apply_voice_mode; в monitor применяется=false."""
        self.node._apply_voice_mode = MagicMock(return_value=(False, MONITOR_MODE_REASON))
        self.node._on_set_voice_mode(_make_string_msg("quest_ttts"))
        self.node._apply_voice_mode.assert_called_once_with("quest_ttts")

    def test_off_mode_is_valid(self) -> None:
        """W3-1 — "off" ("диалог off", §3.5 dialogue-mode-spec) в списке
        допустимых режимов voice_input_mode (ADR-0027 §3.4)."""
        self.assertIn("off", VOICE_INPUT_MODES)

    def test_active_mode_dispatches_off(self) -> None:
        """В active режиме "off" применяется так же, как остальные режимы —
        супервизор не отличает "off" от прочих значений на своей стороне,
        вся логика блокировки ReSpeaker — в dialogue_node."""
        self.node._mode = "active"
        self.node._set_dialogue_param = MagicMock()
        applied, reason = self.node._apply_voice_mode("off")
        self.assertTrue(applied)
        self.assertEqual(reason, "applied")
        self.node._set_dialogue_param.assert_called_once_with("voice_input_mode", "off")


class TestAvatarSupervisorVoicePresetsAndLanguage(unittest.TestCase):
    """AV-28 §P7 — супервизор владеет voice_preset + voice_output_language.

    Маршрут: UI → ws_server.set_voice → Bridge → /avatar/set_voice_preset
    (или _language) → supervisor → SetParameters на dialogue_node.
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
        """Whitelist'ы супервизора = ws_server.VOICE_* = voice_presets.yaml.

        Разъехавшись, они дают молчаливый отказ: ws_server отвечает
        Quest'у ack, а супервизор роняет запрос в applied=False. Так уехали
        `translate` и языки fr/de/zh/hi — оператор жал кнопку, UI
        подсвечивал выбор, робот его не получал.
        """
        import yaml

        yaml_path = (
            pathlib.Path(__file__).resolve().parents[3]
            / "rob_box_voice"
            / "config"
            / "voice_presets.yaml"
        )
        data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        self.assertEqual(set(data["presets"].keys()), set(VOICE_PRESET_IDS))
        self.assertEqual(
            {str(code).lower() for code in data["languages"]},
            set(VOICE_LANGUAGES),
        )
        # Класс валидирует ровно этими списками (второй копии больше нет).
        self.assertEqual(set(self.node._AV28_PRESET_IDS), set(VOICE_PRESET_IDS))
        self.assertEqual(set(self.node._AV28_LANGUAGES), set(VOICE_LANGUAGES))

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
        """В active режиме валидный preset → SetParameters(voice_preset=...)."""
        self.node._mode = "active"
        self.node._set_dialogue_param = MagicMock()
        applied, reason = self.node._apply_voice_preset("philosopher")
        self.assertTrue(applied)
        self.assertEqual(reason, "applied")
        self.node._set_dialogue_param.assert_called_once_with(
            "voice_preset", "philosopher"
        )

    def test_active_mode_dispatches_language(self) -> None:
        self.node._mode = "active"
        self.node._set_dialogue_param = MagicMock()
        applied, reason = self.node._apply_voice_language("en")
        self.assertTrue(applied)
        self.assertEqual(reason, "applied")
        self.node._set_dialogue_param.assert_called_once_with(
            "voice_output_language", "en"
        )

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

    def test_param_set_failure_reported(self) -> None:
        """Ошибка RPC SetParameters должна отдаваться как param_set_failed,
        а не валить ноду (BLE001-семейство ошибок)."""
        self.node._mode = "active"
        self.node._set_dialogue_param = MagicMock(
            side_effect=RuntimeError("service unavailable")
        )
        applied, reason = self.node._apply_voice_preset("street")
        self.assertFalse(applied)
        self.assertIn("param_set_failed", reason)
        self.assertIn("service unavailable", reason)


if __name__ == "__main__":
    unittest.main()
