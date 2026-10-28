"""register_speaker: исход - событие владельца голоса собеседника, а не публикация (#3793, ADR-0148, ADR-0150 §5.2).

Живой прогон 21.10: «Робот, я Глеб» (1.29 с) - регистрацию по представлению speaker_id_node отклонил ``too_short``, а
тул ответил LLM «уже отправлена», и LLM сказала «Записал вас: голос теперь узнаю.». Теперь тул отдаёт исход
``SpeakerResult`` с тем же ``utterance_id``: registered / rejected{reason} / timeout, фраза - из ``phrases``.
"""

import json
import threading

from rob_box_dialog import knowledge as kn
from rob_box_dialog_msgs.msg import SpeakerResult

from ._ros_stubs import RosStubs

_ros = RosStubs()
with _ros:
    from rob_box_mcp_tools.tools import dialogue as dialogue_module
    from rob_box_mcp_tools.tools.dialogue import RegisterSpeakerTool
_ros_stubs = _ros.fixture()

UTT = "07b1d1cf0196"  # из лога #3793


class _Node:
    """Нода mcp_server: ``reply(body)`` - что ответит speaker_id_node на запрос регистрации (``None`` - молчит)."""

    def __init__(self, reply=lambda _b: None):
        self._reply, self.sent, self.subs = reply, [], {}

    def create_publisher(self, _typ, topic, _depth):
        node = self

        class _Pub:
            def publish(self, msg):
                body = json.loads(msg.data)
                node.sent.append((topic, body))
                for event in node._reply(body) or ():
                    threading.Thread(target=node.owner_says, args=(event,)).start()
        return _Pub()

    def create_subscription(self, _typ, topic, cb, _depth):
        self.subs[topic] = cb

    def owner_says(self, event):
        self.subs[dialogue_module.SPEAKER_RESULT_TOPIC](event)

    def get_logger(self):
        class _Log:
            def info(self, *_a, **_k):
                pass
            warning = error = debug = info
        return _Log()


def _result(event, utterance_id=UTT, name="Глеб", **fields):
    msg = SpeakerResult()
    msg.event, msg.utterance_id, msg.name = event, utterance_id, name
    for key, value in fields.items():
        setattr(msg, key, value)
    return msg


def _registered(body, **fields):
    return [_result(SpeakerResult.EVENT_REGISTERED, body["utterance_id"], body["name"], **fields)]


def _too_short(body):
    return [_result(SpeakerResult.EVENT_REGISTER_ERROR, body["utterance_id"], body["name"], error="too_short",
                    duration_s=1.29, min_required_s=3.0)]


def _tool(monkeypatch, reply=lambda _b: None, deadline=2.0):
    monkeypatch.setattr(dialogue_module, "register_deadline_s", lambda: deadline)
    node = _Node(reply)
    return RegisterSpeakerTool(node), node


def _registers(node):
    return [body for topic, body in node.sent if topic == "/voice/speaker/register"]


def test_registered_event_is_success_with_the_phrase_from_the_catalog(monkeypatch):
    tool, node = _tool(monkeypatch, _registered)
    out = tool.execute(name="глеб", utterance_id=UTT)
    assert _registers(node) == [{"name": "Глеб", "utterance_id": UTT}]
    assert out.success and out.data["status"] == "done" and out.data["name"] == "Глеб"
    assert out.message == "Запомнил тебя, Глеб." and out.error is None


def test_too_short_is_an_honest_refusal_not_ok(monkeypatch):
    tool, _ = _tool(monkeypatch, _too_short)
    out = tool.execute(name="Глеб", utterance_id=UTT)
    assert not out.success and out.data["status"] == "rejected" and out.data["reason"] == "too_short"
    # модель при отказе видит error: там фраза кода, а не «отправлено»
    assert out.message == out.error == "Голос не запомнил: фраза слишком короткая."


def test_unknown_error_code_falls_back_to_the_generic_refusal(monkeypatch):
    tool, _ = _tool(monkeypatch, lambda b: [_result(SpeakerResult.EVENT_REGISTER_ERROR, b["utterance_id"],
                                                    error="utterance_not_found")])
    out = tool.execute(name="Глеб", utterance_id=UTT)
    assert out.data["reason"] == "utterance_not_found" and out.message == "Не смог запомнить голос."


def test_silent_owner_is_timeout_not_success(monkeypatch):
    tool, _ = _tool(monkeypatch, deadline=0.1)
    out = tool.execute(name="Глеб", utterance_id=UTT)
    assert not out.success and out.data["status"] == "timeout"
    assert out.message == out.error == "Не дождался, пока голос запишется."


def test_result_of_another_phrase_or_a_biometry_match_is_not_ours(monkeypatch):
    tool, _ = _tool(monkeypatch, lambda b: [_result(SpeakerResult.EVENT_REGISTERED, "other-utterance"),
                                            _result(SpeakerResult.EVENT_MATCH, b["utterance_id"])], deadline=0.2)
    assert tool.execute(name="Глеб", utterance_id=UTT).data["status"] == "timeout"


def test_already_known_voice_says_so(monkeypatch):
    tool, _ = _tool(monkeypatch, lambda b: _registered(b, already_known=True))
    out = tool.execute(name="Глеб", utterance_id=UTT)
    assert out.success and out.data["reason"] == "already_known" and out.message == "Я тебя и так знаю, Глеб."


def test_confident_voice_renamed_says_we_know_each_other(monkeypatch):
    """#4054: голос фразы уверенно узнан под другим именем — speaker_id_node переименовал профиль (ack ``registered``
    с ``new_name``): исход ``done{renamed}``, фраза кода «мы знакомы», а не «Запомнил тебя»."""
    tool, _ = _tool(monkeypatch, lambda b: _registered(b, reused_profile=True, new_name=b["name"]))
    out = tool.execute(name="тамара", utterance_id=UTT)
    assert out.success and out.data["status"] == "done" and out.data["reason"] == "renamed"
    assert out.message == "Мы знакомы — я тебя узнал по голосу. Буду звать тебя Тамара." and out.error is None


def test_intro_sent_by_robot_returns_its_rejection_without_a_second_request(monkeypatch):
    """Живой случай #3793: робот сам отправил регистрацию по «я Глеб», too_short пришёл ДО вызова тула."""
    tool, node = _tool(monkeypatch, deadline=5.0)
    node.owner_says(_too_short({"utterance_id": UTT, "name": "Глеб"})[0])
    out = tool.execute(name="Глеб", utterance_id=UTT, self_intro_name="Глеб", intro_registered=True)
    assert _registers(node) == []
    assert not out.success and out.data["reason"] == "too_short" and out.data["sent_by_robot"] is True
    assert "Записал" not in (out.message or "") and out.error == "Голос не запомнил: фраза слишком короткая."


def test_intro_sent_by_robot_waits_for_an_outcome_still_on_its_way(monkeypatch):
    tool, node = _tool(monkeypatch)
    threading.Timer(0.05, node.owner_says, args=(_result(SpeakerResult.EVENT_REGISTERED),)).start()
    out = tool.execute(name="Глеб", utterance_id=UTT, self_intro_name="Глеб", intro_registered=True)
    assert out.success and out.message == "Запомнил тебя, Глеб." and _registers(node) == []


def test_intro_sent_by_robot_without_outcome_is_timeout(monkeypatch):
    tool, node = _tool(monkeypatch, deadline=0.1)
    out = tool.execute(name="Глеб", utterance_id=UTT, self_intro_name="Глеб", intro_registered=True)
    assert out.data["status"] == "timeout" and _registers(node) == []


def test_without_a_phrase_nothing_is_sent_and_the_refusal_is_known(monkeypatch):
    """speaker_id_node без utterance_id откажет всегда (no_utterance_context, #2829) - запрос не нужен."""
    tool, node = _tool(monkeypatch)
    out = tool.execute(name="Глеб")
    assert _registers(node) == [] and out.data["reason"] == "no_utterance_context" and not out.success


def test_deadline_is_the_class_deadline_from_knowledge():
    assert dialogue_module.register_deadline_s() == kn.ACTION_CLASSES[kn.TOOL_CLASSES["register_speaker"]].timeout_s


def test_catalog_names_the_owner_event_source():
    from rob_box_core.tool_catalog import get_tool

    assert get_tool("register_speaker").event_source == dialogue_module.SPEAKER_RESULT_TOPIC == "/voice/speaker/result"


# --- падеж имени, которое модель пишет в профиль (ревью PR #4118) ---------------------------------------------------


def _renames(node):
    return [body for topic, body in node.sent if topic == "/voice/speaker/rename"]


def test_name_written_by_the_model_in_instrumental_goes_to_the_profile_in_nominative(monkeypatch):
    """«Зови меня Тамарой»: модель передала name='Тамарой' как услышала - владельцу голосов уходит «Тамара», фраза
    о профиле тоже «Тамара», а не «Тамарой» (``words.profile_name``)."""
    tool, node = _tool(monkeypatch, _registered)
    out = tool.execute(name="Тамарой", utterance_id=UTT)
    assert _registers(node) == [{"name": "Тамара", "utterance_id": UTT}]
    assert out.data["name"] == "Тамара" and "Тамарой" not in (out.message or "")


def test_masculine_accusative_is_nominative_in_the_profile_too(monkeypatch):
    tool, node = _tool(monkeypatch, _registered)
    tool.execute(name="бориса", utterance_id=UTT)
    assert _registers(node) == [{"name": "Борис", "utterance_id": UTT}]


def test_rename_to_a_name_in_instrumental_publishes_the_nominative(monkeypatch):
    tool, node = _tool(monkeypatch, lambda b: _registered(b) if "utterance_id" in b else None)  # rename-пакет без фразы
    tool.execute(name="Тамарой", old_name="Эйджик", utterance_id=UTT)
    assert _renames(node) == [{"old_name": "Эйджик", "new_name": "Тамара"}]
    assert _registers(node) == [{"name": "Тамара", "utterance_id": UTT}]


def test_female_name_that_looks_like_a_masculine_genitive_is_registered_as_it_is(monkeypatch):
    """«Меня зовут Валерия»: женское имя, не «Валерий» (оно стоит в словаре имён)."""
    tool, node = _tool(monkeypatch, _registered)
    tool.execute(name="Валерия", utterance_id=UTT)
    assert _registers(node) == [{"name": "Валерия", "utterance_id": UTT}]
