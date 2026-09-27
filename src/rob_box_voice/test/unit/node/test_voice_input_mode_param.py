"""test_voice_input_mode_param.py — regression для Issue #1601 / ADR-0027 §3.4.

Закрывает расхождение: ADR-0027 описывает 5 режимов захвата голоса
(``respeaker | quest_passthrough | quest_ttts | quest_stt |
quest_llm_formalize``, default ``respeaker``), а ``dialogue_node._declare_params()``
параметр не объявлял → supervisor (ADR-0028) не мог переключать режимы.

Тест страхует три инварианта:

  1. ``src/rob_box_voice/config/dialogue_node.yaml`` содержит ключ
     ``voice_input_mode`` со значением ``"respeaker"``.
  2. ``dialogue_node._declare_params()`` вызывает
     ``self.declare_parameter("voice_input_mode", "respeaker")``.
  3. ``DialogueNode.parameters_callback`` корректно логирует изменение
     и возвращает ``SetParametersResult(successful=True)``.

Не требует ROS2: rclpy замокан в ``conftest.py``, а ``parameters_callback``
импортирует ``rcl_interfaces.msg.SetParametersResult`` лениво (как
tts_node.py), так что в unit-окружении достаточно подсунуть поддельный
класс в ``sys.modules``.
"""

from __future__ import annotations

import re
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest
import yaml

from rob_box_voice.dialogue_node import DialogueNode


# ─────────────────────────────────────────────────────────────────────────────
#  Пути к источникам истины
# ─────────────────────────────────────────────────────────────────────────────

# Тест лежит в ``src/rob_box_voice/test/unit/node/...``; корень моно-репо —
# это ``parents[5]`` (тогда ``parents[5]/src/rob_box_voice/...`` валиден).
# Раньше ошибочно использовался ``parents[4]`` (= ``.../src``), что давало
# дублирование ``src/src/...`` при резолве относительных путей.
REPO_ROOT = Path(__file__).resolve().parents[5]
DIALOGUE_NODE_PY = (
    REPO_ROOT / "src" / "rob_box_voice" / "rob_box_voice" / "dialogue_node.py"
)
DIALOGUE_NODE_YAML = (
    REPO_ROOT / "src" / "rob_box_voice" / "config" / "dialogue_node.yaml"
)


# ─────────────────────────────────────────────────────────────────────────────
#  Helpers
# ─────────────────────────────────────────────────────────────────────────────


def _make_node() -> DialogueNode:
    """DialogueNode через ``object.__new__`` + ручные атрибуты."""
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n._logger = logger
    n.get_logger = lambda: logger
    return n


def _install_fake_rcl_interfaces(monkeypatch: pytest.MonkeyPatch) -> None:
    """Подсовываем фейковый ``rcl_interfaces.msg.SetParametersResult``.

    Тест НЕ требует установленного ROS2: ``parameters_callback`` импортирует
    ``SetParametersResult`` лениво внутри метода (как tts_node.py), так что
    достаточно зарегистрировать класс в ``sys.modules`` ДО вызова колбэка.
    """

    rcl_interfaces = types.ModuleType("rcl_interfaces")
    msg = types.ModuleType("rcl_interfaces.msg")
    msg.SetParametersResult = type(  # type: ignore[attr-defined]
        "SetParametersResult",
        (),
        {
            "__init__": lambda self, successful=True: setattr(
                self, "successful", successful
            )
        },
    )
    rcl_interfaces.msg = msg
    monkeypatch.setitem(sys.modules, "rcl_interfaces", rcl_interfaces)
    monkeypatch.setitem(sys.modules, "rcl_interfaces.msg", msg)


def _make_param(name: str, value):
    """Имитация rcl_interfaces.msg.Parameter для parameters_callback."""
    p = MagicMock()
    p.name = name
    p.value = value
    return p


# ─────────────────────────────────────────────────────────────────────────────
#  YAML — ключ voice_input_mode: respeaker
# ─────────────────────────────────────────────────────────────────────────────


class TestDialogueNodeYaml:
    """Регрессионный контракт на config-файл (issue #1601 / ADR-0027 §3.4)."""

    def test_yaml_has_voice_input_mode_key(self):
        """Ключ voice_input_mode присутствует в dialogue_node.yaml."""
        with DIALOGUE_NODE_YAML.open(encoding="utf-8") as fh:
            config = yaml.safe_load(fh)
        params = config["dialogue_node"]["ros__parameters"]
        assert (
            "voice_input_mode" in params
        ), "dialogue_node.yaml: ключ voice_input_mode обязателен (ADR-0027 §3.4)"

    def test_yaml_default_is_respeaker(self):
        """Default в YAML — ``respeaker`` (ADR-0027 §3.4, обратная совместимость)."""
        with DIALOGUE_NODE_YAML.open(encoding="utf-8") as fh:
            config = yaml.safe_load(fh)
        params = config["dialogue_node"]["ros__parameters"]
        assert params["voice_input_mode"] == "respeaker", (
            "ADR-0027 §3.4: default voice_input_mode = 'respeaker' "
            "(обратная совместимость для уже работающих respeaker-only "
            "инсталляций, supervisor ADR-0028 ещё не на всех роботах)."
        )


# ─────────────────────────────────────────────────────────────────────────────
#  dialogue_node.py — declare_parameter("voice_input_mode", ...)
# ─────────────────────────────────────────────────────────────────────────────


class TestDeclareParams:
    """Регрессионный контракт на _declare_params в dialogue_node.py."""

    def test_declare_params_contains_voice_input_mode(self):
        """Метод _declare_params() содержит declare_parameter("voice_input_mode", ...)."""
        src = DIALOGUE_NODE_PY.read_text(encoding="utf-8")

        # Берём только тело метода (всё, что между ``def _declare_params`` и
        # следующим ``def`` на том же уровне отступа) — иначе регэксп
        # может зацепить комментарий/строку-док в неожиданном месте.
        m = re.search(
            r"def _declare_params\(self\).*?(?=\n    def |\Z)",
            src,
            flags=re.DOTALL,
        )
        assert m, "dialogue_node.py: метод _declare_params не найден"
        body = m.group(0)

        assert re.search(
            r'self\.declare_parameter\(\s*"voice_input_mode"\s*,\s*"respeaker"\s*\)',
            body,
        ), (
            "dialogue_node.py: в _declare_params() ожидается\n"
            '    self.declare_parameter("voice_input_mode", "respeaker")\n'
            "Без этого supervisor (ADR-0028) не сможет переключать режимы "
            "(rclpy отвергает set_parameters с unknown name)."
        )


# ─────────────────────────────────────────────────────────────────────────────
#  parameters_callback — stub-обработчик для runtime-переключения
# ─────────────────────────────────────────────────────────────────────────────


class TestParametersCallback:
    """Stub-обработчик Issue #1601: логирует изменение режима и возвращает OK."""

    def test_logs_voice_input_mode_change(self, monkeypatch: pytest.MonkeyPatch):
        """При изменении voice_input_mode пишется info-лог с новым значением."""
        _install_fake_rcl_interfaces(monkeypatch)
        n = _make_node()
        n.parameters_callback([_make_param("voice_input_mode", "quest_passthrough")])

        n.get_logger().info.assert_called_once()
        msg = n.get_logger().info.call_args[0][0]
        assert "voice_input_mode" in msg
        assert "quest_passthrough" in msg

    def test_returns_successful_result(self, monkeypatch: pytest.MonkeyPatch):
        """Возвращает SetParametersResult(successful=True) — supervisor
        переключение не отклоняется."""
        _install_fake_rcl_interfaces(monkeypatch)
        n = _make_node()
        result = n.parameters_callback([_make_param("voice_input_mode", "quest_ttts")])
        assert result.successful is True

    def test_ignores_other_parameters(self, monkeypatch: pytest.MonkeyPatch):
        """Callback не падает на параметрах, к которым не относится."""
        _install_fake_rcl_interfaces(monkeypatch)
        n = _make_node()
        # Меняем wake_words (другая фича) — наш stub-логгер не должен
        # срабатывать, callback не должен упасть.
        result = n.parameters_callback([_make_param("wake_words", ["робот"])])
        n.get_logger().info.assert_not_called()
        assert result.successful is True

    def test_multiple_changes_logged_in_order(self, monkeypatch: pytest.MonkeyPatch):
        """Несколько последовательных изменений — каждый логируется (один вызов
        на param в списке)."""
        _install_fake_rcl_interfaces(monkeypatch)
        n = _make_node()
        n.parameters_callback(
            [
                _make_param("voice_input_mode", "quest_stt"),
                _make_param("voice_input_mode", "quest_llm_formalize"),
            ]
        )
        # Оба изменения — в один info-вызов (один param на каждый),
        # но цикл прошёл по обоим.
        assert n.get_logger().info.call_count == 2
        values = [c.args[0] for c in n.get_logger().info.call_args_list]
        assert any("quest_stt" in v for v in values)
        assert any("quest_llm_formalize" in v for v in values)
