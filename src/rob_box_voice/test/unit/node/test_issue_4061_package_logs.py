"""#4061: INFO python-логгеров пакета ``rob_box_dialog`` виден в логе ноды.

Живой робот 27.10 (образ с #4042): ``docker logs voice-assistant | grep -c '📨 \\[execute\\]'`` = 0, хотя тулы
вызывались. ``Executor`` пишет в ``logging.getLogger(__name__)``; у python logging без обработчика уровня INFO нет
(только WARNING и выше через ``lastResort``). Одно место решает, куда пишут логгеры пакета:
``dialog_node.RosLogHandler``.
"""

from __future__ import annotations

import logging
import sys
import types

import pytest
from rob_box_core.msg_stub import install_dialog_msgs_stub

M = install_dialog_msgs_stub()
try:  # настоящий rosidl-пакет (CI) или заглушка сервиса DialogControl (Windows) — как в test_dialog_node.py
    import rob_box_dialog_msgs.srv  # noqa: F401
except ImportError:
    class _Request:
        ACTION_HOLD, ACTION_RESUME, ACTION_RESET = 0, 1, 2

    _srv = types.ModuleType("rob_box_dialog_msgs.srv")
    _srv.DialogControl = type("DialogControl", (), {"Request": _Request,
                                                    "Response": lambda: types.SimpleNamespace()})
    sys.modules["rob_box_dialog_msgs.srv"] = _srv
    sys.modules["rob_box_dialog_msgs"].srv = _srv

from rob_box_llm.provider import LLMResponse, ToolCall, ToolResult  # noqa: E402
from rob_box_voice import dialog_node as dn  # noqa: E402

EXECUTE_LINE = "📨 [execute] событие"
PACKAGE = logging.getLogger(dn.DIALOG_LOGGER)


class _RosLogger:
    """Логгер ROS по поведению: запись по уровням (rclpy ``info``/``warning``/``error``)."""

    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(("info", text))

    def warning(self, text):
        self.lines.append(("warning", text))

    def error(self, text):
        self.lines.append(("error", text))


@pytest.fixture()
def ros():
    logger = _RosLogger()
    handler = dn.RosLogHandler.attach(logger)
    yield logger
    handler.close()


def test_info_of_any_package_module_reaches_the_ros_log(ros):
    """Не только ``execute``: ``turn.py``, ``plan.*``, ``chats`` пишут так же - обработчик на логгер пакета берёт
    все."""
    logging.getLogger("rob_box_dialog.execute").info(f"{EXECUTE_LINE} play_animation t-1/0 done - ход_ждёт=True")
    logging.getLogger("rob_box_dialog.turn").info("речь при действиях не звучит")
    logging.getLogger("rob_box_dialog.plan.rap_pauses").info("разметка пауз снята")
    assert ros.lines == [("info", f"{EXECUTE_LINE} play_animation t-1/0 done - ход_ждёт=True"),
                         ("info", "речь при действиях не звучит"), ("info", "разметка пауз снята")]


def test_each_level_goes_to_its_own_ros_method(ros):
    log = logging.getLogger("rob_box_dialog.execute")
    log.debug("отладка")
    log.info("инфо")
    log.warning("⚠️ [execute] stale_event ref=t-1/0")
    log.error("упало")
    log.critical("совсем")
    assert ros.lines == [("info", "инфо"), ("warning", "⚠️ [execute] stale_event ref=t-1/0"), ("error", "упало"),
                         ("error", "совсем")]


def test_message_arguments_and_exception_text_are_formatted(ros):
    log = logging.getLogger("rob_box_dialog.chats")
    log.info("чат %s ход %d", "telegram:42", 3)
    try:
        raise ValueError("нет голоса")
    except ValueError:
        log.error("ход упал", exc_info=True)
    assert ros.lines[0] == ("info", "чат telegram:42 ход 3")
    level, text = ros.lines[1]
    assert level == "error" and text.startswith("ход упал") and "ValueError: нет голоса" in text


def test_loggers_outside_the_package_are_not_routed(ros):
    logging.getLogger("rob_box_voice.core.other").info("чужой пакет")
    logging.getLogger("rob_box_dialogs").info("похожее имя - не подпакет")
    assert ros.lines == []


def test_attach_again_replaces_the_previous_handler_and_close_restores_the_logger():
    PACKAGE.setLevel(logging.NOTSET)
    first, second = _RosLogger(), _RosLogger()
    one = dn.RosLogHandler.attach(first)
    two = dn.RosLogHandler.attach(second)  # перезапуск ноды в том же процессе: прежний обработчик снят
    try:
        logging.getLogger("rob_box_dialog.execute").info("строка")
        assert first.lines == [] and second.lines == [("info", "строка")]
        assert [h for h in PACKAGE.handlers if isinstance(h, dn.RosLogHandler)] == [two]
        assert PACKAGE.level == logging.INFO
    finally:
        two.close()
        one.close()
    assert not [h for h in PACKAGE.handlers if isinstance(h, dn.RosLogHandler)]
    assert PACKAGE.level == logging.NOTSET


def test_a_failing_ros_logger_does_not_break_the_logging_call(capsys):
    class _Broken(_RosLogger):
        def info(self, text):
            raise RuntimeError("rclpy: Logger severity cannot be changed between calls.")

    handler = dn.RosLogHandler.attach(_Broken())
    try:
        logging.getLogger("rob_box_dialog.execute").info("строка")  # не бросает: контракт logging (handleError)
    finally:
        handler.close()
    assert "Logging error" in capsys.readouterr().err  # сбой виден в stderr, а не проглочен


# -- на настоящем пути ноды ----------------------------------------------------------------------------------------


class _Tools:
    async def execute(self, call):
        return ToolResult(tool_call_id=call.id, content="ok", data={"status": "done"})


@pytest.fixture()
def node(monkeypatch):
    order = ToolCall(id="c1", name="command", arguments={
        "actions": [{"tool": "request_music", "args": {"intent": "track", "text": "сыграй Running Up That Hill"}}]})

    class _LLM:
        answered_by = "deepseek"

        def __init__(self):
            self.responses = [LLMResponse(content="", tool_calls=(order,))]

        async def complete(self, messages, tools=(), settings=None):
            return self.responses.pop(0)

    llm = _LLM()
    tools = _Tools()
    monkeypatch.setattr(dn, "build_llm", lambda node, counter, agent="personality": (llm, "Ты робот."))
    monkeypatch.setattr(dn, "build_tools", lambda node: tools)
    monkeypatch.setattr(dn, "build_chat_tools", lambda node, personality_tools: tools)
    made = dn.DialogNode()
    stamp = type(M.TurnEvent().stamp)(sec=1, nanosec=0)
    made.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: stamp))
    yield made
    made.shutdown()


def _logged(node):
    return [c.args[0] for c in node.get_logger().info.call_args_list]


def test_executor_info_of_a_real_turn_is_in_the_node_log(node):
    """Ход с тулом: итог владельца дошёл до исполнителя - строка ``📨 [execute]`` в логе ноды (раньше её не было)."""
    msg = M.Utterance()
    msg.text, msg.source, msg.utterance_id = "Робот, сыграй Running Up That Hill", "respeaker", "u4061"
    node._on_utterance(msg)
    node.last_turn.result(timeout=10)
    lines = [line for line in _logged(node) if line.startswith(EXECUTE_LINE)]
    assert len(lines) == 1 and "request_music" in lines[0] and "done" in lines[0], lines


def test_shutdown_detaches_the_node_from_the_package_logger(node):
    assert [h for h in PACKAGE.handlers if isinstance(h, dn.RosLogHandler)]
    node.shutdown()
    assert not [h for h in PACKAGE.handlers if isinstance(h, dn.RosLogHandler)]
