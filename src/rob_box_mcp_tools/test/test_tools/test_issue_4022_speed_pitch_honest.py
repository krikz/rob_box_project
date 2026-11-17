"""#4022 — ``set_speed`` / ``set_pitch`` не говорят «готово», пока голос не изменился.

Живой прогон 23.10 (акт 4, n409–n411): при звучащем MiniMax ``set_speed`` писал ``yandex_speed``, ``set_pitch`` писал
``pitch_shift`` при выключенном ``chipmunk_mode`` — оба отвечали успехом, голос не менялся. Теперь тул выставляет
параметр АКТИВНОГО провайдера, читает его назад и строит ответ из прочитанного; провайдер без регулировки — отказ.

Сервер параметров ``tts_node`` здесь — сценарный: хранит значения, принимает/отклоняет запись, может «забыть» применить.
"""

from __future__ import annotations

import sys
from types import ModuleType, SimpleNamespace

import pytest

from .._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.tools.system import SetPitchTool, SetSpeedTool
_ros_stubs = _ros.fixture()

NOT_SET, BOOL, DOUBLE = 0, 1, 3


class _Done:
    """Завершённый future: ``wait_future`` ждёт его ``add_done_callback``."""

    def __init__(self, result):
        self._result = result

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        callback(self)


class _Reply:
    def __init__(self, successful=True, reason=""):
        self.successful, self.reason = successful, reason


class TtsParamServer:
    """Параметры ``tts_node`` как их видят ROS-сервисы get/set_parameters."""

    def __init__(self, **params):
        self.params = dict(params)
        self.writes: list = []
        self.rejects: dict = {}  # параметр -> причина отказа ноды
        self.forgets: set = set()  # принято, но не применено (значение не меняется)
        self.silent = False  # нода не отвечает

    def get(self, request):
        if self.silent:
            return _Done(None)
        values = []
        for name in request.names:
            raw = self.params.get(name)
            kind = NOT_SET if raw is None else BOOL if isinstance(raw, bool) else DOUBLE
            values.append(SimpleNamespace(type=kind, double_value=float(raw or 0.0) if kind == DOUBLE else 0.0,
                                          bool_value=bool(raw) if kind == BOOL else False))
        return _Done(SimpleNamespace(values=values))

    def set(self, request):
        replies = []
        for param in request.parameters:
            self.writes.append((param.name, param.value.double_value))
            if param.name in self.rejects:
                replies.append(_Reply(False, self.rejects[param.name]))
                continue
            if param.name not in self.forgets:
                self.params[param.name] = param.value.double_value
            replies.append(_Reply())
        return _Done(SimpleNamespace(results=replies))


class _Client:
    def __init__(self, call, available=True):
        self._call, self._available = call, available

    def wait_for_service(self, timeout_sec=0.0):
        return self._available

    def call_async(self, request):
        return self._call(request)


def _install_rcl_interfaces(monkeypatch):
    """Тонкая подмена ``rcl_interfaces`` (в unit-окружении без ROS): только то, что трогают тулы."""
    msg, srv = ModuleType("rcl_interfaces.msg"), ModuleType("rcl_interfaces.srv")
    msg.ParameterType = SimpleNamespace(PARAMETER_NOT_SET=NOT_SET, PARAMETER_BOOL=BOOL, PARAMETER_DOUBLE=DOUBLE)
    msg.Parameter = lambda: SimpleNamespace(name="", value=None)
    msg.ParameterValue = lambda: SimpleNamespace(type=NOT_SET, double_value=0.0, bool_value=False)
    srv.GetParameters = SimpleNamespace(Request=lambda: SimpleNamespace(names=[]))
    srv.SetParameters = SimpleNamespace(Request=lambda: SimpleNamespace(parameters=[]))
    pkg = ModuleType("rcl_interfaces")
    pkg.msg, pkg.srv = msg, srv
    for name, module in (("rcl_interfaces", pkg), ("rcl_interfaces.msg", msg), ("rcl_interfaces.srv", srv)):
        monkeypatch.setitem(sys.modules, name, module)


def _make(tool_cls, monkeypatch, server, provider="minimax", available=True):
    _install_rcl_interfaces(monkeypatch)
    clients = {"/tts_node/get_parameters": _Client(server.get, available),
               "/tts_node/set_parameters": _Client(server.set, available)}
    logger = SimpleNamespace(info=lambda *_: None, warning=lambda *_: None, error=lambda *_: None,
                             debug=lambda *_: None)
    node = SimpleNamespace(create_client=lambda _srv, name: clients[name], get_logger=lambda: logger,
                           actual_tts_provider=provider)
    return tool_cls(node)


# ── set_speed ────────────────────────────────────────────────────────────────


def test_set_speed_under_minimax_changes_minimax_speed_and_says_the_value_it_read_back(monkeypatch):
    server = TtsParamServer(minimax_speed=1.0, yandex_speed=1.0)
    result = _make(SetSpeedTool, monkeypatch, server, "minimax").execute("slower")
    assert result.success
    assert server.params == {"minimax_speed": 0.8, "yandex_speed": 1.0}  # звучащий голос — minimax, не yandex
    assert server.writes == [("minimax_speed", 0.8)]
    assert result.data["status"] == "done" and result.data["provider"] == "minimax"
    assert result.data["param"] == "minimax_speed" and result.data["new_speed"] == 0.8
    assert "0.8" in result.message and "медленнее" in result.message


def test_set_speed_under_yandex_changes_yandex_speed(monkeypatch):
    server = TtsParamServer(minimax_speed=1.0, yandex_speed=1.0)
    result = _make(SetSpeedTool, monkeypatch, server, "yandex").execute("faster")
    assert result.success and server.params == {"minimax_speed": 1.0, "yandex_speed": 1.2}
    assert result.data["param"] == "yandex_speed" and result.data["new_speed"] == 1.2


@pytest.mark.parametrize("provider", ["silero", "someone_new"])
def test_set_speed_for_a_provider_without_speed_control_is_rejected_not_done(monkeypatch, provider):
    server = TtsParamServer(minimax_speed=1.0, yandex_speed=1.0)
    result = _make(SetSpeedTool, monkeypatch, server, provider).execute("slower")
    assert not result.success
    assert result.data["status"] == "rejected" and result.data["reason"] == "speed_unsupported"
    assert "скорост" in result.error and "не изменился" in result.error
    assert server.writes == []  # голос не трогали


def test_silero_is_named_in_the_refusal_the_way_the_robot_says_it(monkeypatch):
    result = _make(SetSpeedTool, monkeypatch, TtsParamServer(), "silero").execute("faster")
    assert "Silero" in result.error


def test_speed_the_node_took_but_did_not_apply_is_not_success(monkeypatch):
    server = TtsParamServer(minimax_speed=1.0)
    server.forgets.add("minimax_speed")
    result = _make(SetSpeedTool, monkeypatch, server, "minimax").execute("slower")
    assert not result.success
    assert result.data["status"] == "rejected" and result.data["reason"] == "not_applied"


def test_node_refusal_reaches_the_answer(monkeypatch):
    server = TtsParamServer(minimax_speed=1.0)
    server.rejects["minimax_speed"] = "minimax_speed вне диапазона"
    result = _make(SetSpeedTool, monkeypatch, server, "minimax").execute("faster")
    assert not result.success and result.data["reason"] == "set_failed"
    assert "вне диапазона" in result.error


def test_missing_speed_param_is_not_read_as_zero(monkeypatch):
    server = TtsParamServer(yandex_speed=1.0)  # minimax_speed у ноды нет (старая сборка)
    result = _make(SetSpeedTool, monkeypatch, server, "minimax").execute("faster")
    assert not result.success and result.data["reason"] == "read_failed"
    assert server.writes == []


def test_silent_node_is_not_success(monkeypatch):
    server = TtsParamServer(minimax_speed=1.0)
    server.silent = True
    result = _make(SetSpeedTool, monkeypatch, server, "minimax").execute("faster")
    assert not result.success and result.data["reason"] == "read_failed"


def test_params_service_down_is_a_refusal_with_reason(monkeypatch):
    result = _make(SetSpeedTool, monkeypatch, TtsParamServer(minimax_speed=1.0), "minimax", available=False).execute(
        "faster")
    assert not result.success and result.data["reason"] == "params_unavailable"


@pytest.mark.parametrize("action", ["wobble", "higher", ""])
def test_unknown_speed_action_is_rejected_before_touching_the_node(monkeypatch, action):
    server = TtsParamServer(minimax_speed=1.0)
    result = _make(SetSpeedTool, monkeypatch, server, "minimax").execute(action)
    assert not result.success and result.data["reason"] == "bad_action"
    assert server.writes == []


def test_speed_steps_stop_at_the_ends_and_say_so(monkeypatch):
    fast = TtsParamServer(minimax_speed=2.0)
    result = _make(SetSpeedTool, monkeypatch, fast, "minimax").execute("faster")
    assert result.success and fast.writes == [] and result.message == "Скорость уже максимальная"
    slow = TtsParamServer(minimax_speed=0.5)
    result = _make(SetSpeedTool, monkeypatch, slow, "minimax").execute("slower")
    assert result.success and slow.writes == [] and result.message == "Скорость уже минимальная"


def test_normal_speed_returns_to_one(monkeypatch):
    server = TtsParamServer(minimax_speed=1.6)
    result = _make(SetSpeedTool, monkeypatch, server, "minimax").execute("normal")
    assert result.success and server.params["minimax_speed"] == 1.0 and result.data["old_speed"] == 1.6


def test_repeated_steps_walk_the_scale_without_float_tails(monkeypatch):
    server = TtsParamServer(minimax_speed=1.0)
    tool = _make(SetSpeedTool, monkeypatch, server, "minimax")
    for _ in range(4):
        tool.execute("slower")
    assert [w[1] for w in server.writes] == [0.8, 0.6, 0.5]  # дальше — граница, записи нет
    assert server.params["minimax_speed"] == 0.5


# ── set_pitch ────────────────────────────────────────────────────────────────


def test_set_pitch_under_minimax_without_chipmunk_changes_the_voice_param(monkeypatch):
    """Прод: ``chipmunk_mode: false``, звучит MiniMax — тон реально идёт в ``voice_setting.pitch`` (полутона)."""
    server = TtsParamServer(pitch_shift=1.0, chipmunk_mode=False)
    result = _make(SetPitchTool, monkeypatch, server, "minimax").execute("higher")
    assert result.success and server.params["pitch_shift"] == 1.2
    assert result.data["provider"] == "minimax" and result.data["new_pitch"] == 1.2
    assert "+3 полутона" in result.message


def test_set_pitch_lower_names_the_semitones(monkeypatch):
    server = TtsParamServer(pitch_shift=1.0, chipmunk_mode=False)
    result = _make(SetPitchTool, monkeypatch, server, "minimax").execute("lower")
    assert result.success and server.params["pitch_shift"] == 0.8 and "-4 полутона" in result.message


@pytest.mark.parametrize("provider", ["yandex", "silero"])
def test_set_pitch_without_chipmunk_where_no_provider_pitch_exists_is_an_honest_refusal(monkeypatch, provider):
    server = TtsParamServer(pitch_shift=1.0, chipmunk_mode=False)
    result = _make(SetPitchTool, monkeypatch, server, provider).execute("higher")
    assert not result.success
    assert result.data["status"] == "rejected" and result.data["reason"] == "pitch_unsupported"
    assert "тона" in result.error and "не изменился" in result.error
    assert server.writes == [] and server.params["pitch_shift"] == 1.0


@pytest.mark.parametrize("provider", ["yandex", "silero", "minimax"])
def test_set_pitch_with_chipmunk_works_for_every_provider_by_resampling(monkeypatch, provider):
    server = TtsParamServer(pitch_shift=1.0, chipmunk_mode=True)
    result = _make(SetPitchTool, monkeypatch, server, provider).execute("higher")
    assert result.success and server.params["pitch_shift"] == 1.2
    assert "×1.2" in result.message


def test_pitch_the_node_did_not_apply_is_not_success(monkeypatch):
    server = TtsParamServer(pitch_shift=1.0, chipmunk_mode=False)
    server.forgets.add("pitch_shift")
    result = _make(SetPitchTool, monkeypatch, server, "minimax").execute("higher")
    assert not result.success and result.data["reason"] == "not_applied"


def test_unreadable_chipmunk_mode_is_a_refusal_not_a_guess(monkeypatch):
    server = TtsParamServer(pitch_shift=1.0)  # chipmunk_mode не прочитать
    result = _make(SetPitchTool, monkeypatch, server, "minimax").execute("higher")
    assert not result.success and result.data["reason"] == "read_failed"
    assert server.writes == []


def test_pitch_steps_stop_at_the_ends_and_say_so(monkeypatch):
    high = TtsParamServer(pitch_shift=2.0, chipmunk_mode=False)
    result = _make(SetPitchTool, monkeypatch, high, "minimax").execute("higher")
    assert result.success and high.writes == [] and result.message == "Голос уже максимально высокий"
    low = TtsParamServer(pitch_shift=0.5, chipmunk_mode=False)
    result = _make(SetPitchTool, monkeypatch, low, "minimax").execute("lower")
    assert result.success and low.writes == [] and result.message == "Голос уже минимально низкий"


def test_normal_pitch_returns_to_one(monkeypatch):
    server = TtsParamServer(pitch_shift=1.4, chipmunk_mode=False)
    result = _make(SetPitchTool, monkeypatch, server, "minimax").execute("normal")
    assert result.success and server.params["pitch_shift"] == 1.0


def test_unknown_pitch_action_is_rejected_before_touching_the_node(monkeypatch):
    server = TtsParamServer(pitch_shift=1.0, chipmunk_mode=False)
    result = _make(SetPitchTool, monkeypatch, server, "minimax").execute("faster")
    assert not result.success and result.data["reason"] == "bad_action" and server.writes == []


@pytest.mark.parametrize("tool_cls", [SetSpeedTool, SetPitchTool])
def test_every_action_the_schema_offers_is_handled(monkeypatch, tool_cls):
    """Схема, которую видит LLM (enum), и исполнитель не расходятся: ни одно предложенное действие не «неизвестно»."""
    server = TtsParamServer(minimax_speed=1.0, pitch_shift=1.0, chipmunk_mode=False)
    tool = _make(tool_cls, monkeypatch, server, "minimax")
    (action,) = tool.parameters
    assert action.enum and set(action.enum) <= set(tool._STEPS)
    for value in action.enum:
        assert (tool.execute(value).data or {}).get("reason") != "bad_action"


def test_both_tools_declare_the_parameter_server_as_their_owner_event():
    assert SetSpeedTool.event_source == SetPitchTool.event_source == "srv:/tts_node/set_parameters"


# ── #4074: «басом» - режим тона того же голоса; normal снимает его у любого провайдера ──────────────────────────


@pytest.mark.parametrize("provider", ["yandex", "silero"])
def test_normal_pitch_lifts_the_stored_mode_even_where_the_voice_has_no_pitch(monkeypatch, provider):
    """Бас поставлен на MiniMax (0.8), потом голос Алёны (Yandex, тона нет): «обычным голосом» снимает шаг с оси, а не
    отказывает - иначе бас вернулся бы вместе с голосом MiniMax."""
    server = TtsParamServer(pitch_shift=0.8, chipmunk_mode=False)
    result = _make(SetPitchTool, monkeypatch, server, provider).execute("normal")
    assert result.success and result.data["status"] == "done"
    assert server.params["pitch_shift"] == 1.0 and server.writes == [("pitch_shift", 1.0)]


def test_pitch_already_normal_answers_with_the_at_limit_reason_the_dialog_reads(monkeypatch):
    from rob_box_dialog.knowledge import VOICE_STEP_AT_LIMIT

    server = TtsParamServer(pitch_shift=1.0, chipmunk_mode=False)
    result = _make(SetPitchTool, monkeypatch, server, "yandex").execute("normal")
    assert result.success and server.writes == [] and result.data["reason"] == VOICE_STEP_AT_LIMIT
    low = TtsParamServer(pitch_shift=0.5, chipmunk_mode=False)
    result = _make(SetPitchTool, monkeypatch, low, "minimax").execute("lower")
    assert result.data["reason"] == VOICE_STEP_AT_LIMIT and low.writes == []
