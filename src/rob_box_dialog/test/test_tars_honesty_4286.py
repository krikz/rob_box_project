"""#4286: ТАРС (оператор, dialog v2) - находки живого прогона 20 команд 03.11 (#4253, лог avatar-supervisor).

1. №13 «как сеть, есть связь» - «Wi-Fi и роутер живы» без единого тула и поля о сети в ходе (класс К1 #4020: фраза о
   состоянии не из результата). Решает код (``state_claim``, таблица ``knowledge.STATE_CLAIMS``): повтор с подсказкой,
   затем утверждение вырезано, говорит код, о чём данных нет.
2. №11 «статус контейнеров» - срез открывал всю телеметрию, модель крутила ``show_metrics`` до потолка запросов
   (``reason=query_fallback:ceiling``). Узкий срез: «контейнеры» - ``container_status``, «ноды» - ``ros2_node_status``.
3. №1 «какой у тебя сейчас статус» - ``get_robot_status`` дважды подряд. Тот же ``query`` в ходе второй раз не
   исполняется: подсказка модели, снова повтор - ход закрывает код.

Ответы модели и данные тулов - из ``4253_sup_full.log`` (строки 1081-1097, 33-64, 484-694).
"""

import asyncio

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog import state_claim
from rob_box_dialog.address import Addressed
from rob_box_dialog.command import Command, ToolCall
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import available_tools
from rob_box_dialog.turn import Answer, TurnEngine

#: Ответ модели №13 (строка 1097 лога): интернет - из ``internet_available`` в ``recent_actions``, сервисы - из
#: ``show_metrics up``, Wi-Fi и роутер - ниоткуда.
WIFI_SPEECH = ("Связь с интернетом есть, Wi-Fi и роутер живы. По сервисам: живы cadvisor-main и cadvisor-vision, лежат "
               "otel-collector, voice-assistant, telegram-bot и supercollider. Время 20:47.")
HONEST = ("Связь с интернетом есть. По сервисам: живы cadvisor-main и cadvisor-vision, лежат otel-collector, "
          "voice-assistant, telegram-bot и supercollider. Время 20:47.")
PERCEPTION = {"internet_available": True, "battery_percent": 90.0, "failed_nodes": [], "missing_nodes": [],
              "system_health_status": "HEALTHY"}
METRICS_UP = {"query": "up", "datasource": "prometheus", "status": "ok", "series_count": 6}
ROBOT_STATUS = {"position": {"x": 0.002, "y": 0.0007, "theta": 0.007}, "battery_level": 90.04, "systems": {}}
STATUS_SPEECH = "Батарея 90%, стою почти в нулевой точке. Данные по системам пустые — если нужно, гляну контейнеры."


class Ports:
    """Транспорт хода: ответы LLM по очереди; исход тула - ``done`` с данными таблицы ``outcomes``."""

    def __init__(self, answers, outcomes=None):
        self.answers, self.outcomes = list(answers), outcomes or {}
        self.asked, self.executed, self.said, self.stages = [], [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([a.tool for a in actions])
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.call_class(a.tool, a.args),
                             "done", None, dict(self.outcomes.get(a.tool, {}))) for n, a in enumerate(actions)]

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, turn_id, epoch, stage, status, reason="", cls=None, llm_calls=0, provider="", source="",
              expression=None):
        self.stages.append((stage, status, reason))

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _query(tool, **args):
    return Answer(Command(actions=(ToolCall(tool, args),)), "deepseek",
                  follow=lambda data: ({"role": "tool", "content": data},))


def _speech(text):
    return Answer(Command(speech=text), "deepseek", follow=lambda data: ({"role": "tool", "content": data},))


def _run(ports, text, agent="operator"):
    engine = TurnEngine(Session(agent), ports, system_prompt="Ты ТАРС.")
    return asyncio.run(engine.run(Addressed(text=text, source="telegram")))


def _net_turn(*answers):
    return Ports([_query("get_perception_context"), _query("show_metrics", query="up"), *answers],
                 {"get_perception_context": PERCEPTION, "show_metrics": METRICS_UP})


# -- 1. утверждение о состоянии без данных ---------------------------------------------------------------------------


@pytest.mark.parametrize("text, kinds", [
    (WIFI_SPEECH, ("wifi", "internet", "network", "containers")),
    ("ROS2-ноды: отказавших и отсутствующих нет, система здорова.", ("nodes",)),
    ("Батарея 90%, стою почти в нулевой точке.", ("battery",)),
    ("Роутер жив, Wi-Fi ловит.", ("wifi",)),
    ("Все 11 контейнеров подняты: lslidar, nav2, ros2-control.", ("containers",)),  # «ros2-control» - не нода
    ("Моторы в норме, колёса крутятся.", ("motors",)),
])
def test_claims_are_found_by_words(text, kinds):
    assert state_claim.claims(text) == kinds


@pytest.mark.parametrize("text", [
    "Про Wi-Fi и роутер данных у меня нет — утверждать не буду.",  # фраза кода сама не утверждение
    "Роутер жив?",                                                  # вопрос
    STATUS_SPEECH.split(". ")[1],                                   # «гляну контейнеры» - предложение проверить
    "Сейчас 20:47, вторник.",
    "Температуру процессоров показать не могу.",
    "",
])
def test_not_a_claim(text):
    assert state_claim.claims(text) == ()


def test_claim_is_backed_by_tool_or_data_key():
    seen = [{"tool": "get_perception_context", "status": "done", "data": PERCEPTION}]
    assert state_claim.unbacked(WIFI_SPEECH, seen + [{"tool": "show_metrics", "status": "done", "data": METRICS_UP}]) \
        == ("wifi",), "интернет и сеть - из internet_available, сервисы - show_metrics, Wi-Fi - ниоткуда"
    assert state_claim.unbacked("Батарея 90%.", [{"tool": "get_robot_status", "status": "done", "data": {}}]) == ()
    assert state_claim.unbacked("Батарея 90%.", [{"tool": "get_robot_status", "status": "rejected", "data": {}}]) \
        == ("battery",), "провал тула - не данные"
    # живой 03.11 (#4286, прогон заливки): «отвечают по сети» - по данным опроса Prometheus (``show_metrics``)
    assert state_claim.unbacked("Косвенно: Prometheus и cadvisor отвечают по сети, значит канал до робота работает.",
                                [{"tool": "show_metrics", "status": "done", "data": METRICS_UP}]) == ()


def test_cut_keeps_the_backed_part_of_the_sentence():
    assert state_claim.without_claims(WIFI_SPEECH, ("wifi",)) == HONEST
    assert state_claim.without_claims("Роутер жив, интернет есть. Время 20:47.", ("wifi",)) == "Интернет есть. Время 20:47."


def test_live_13_wifi_claim_gets_a_retry_with_hint():
    """№13: модель сказала «Wi-Fi и роутер живы» - не звучит, модель получает нарушение и отвечает честно."""
    ports = _net_turn(_speech(WIFI_SPEECH), _speech(HONEST))
    _run(ports, "как сеть, есть связь")
    assert ports.said == [HONEST]
    assert kn.STATE_CLAIM_HINT in ports.asked[-1][-1]["content"]


def test_live_13_claim_after_retry_is_cut_and_code_speaks():
    """Модель и после повтора утверждает про Wi-Fi: ложное вырезано, звучат правда по данным и фраза кода."""
    ports = _net_turn(_speech(WIFI_SPEECH), _speech(WIFI_SPEECH))
    _run(ports, "как сеть, есть связь")
    assert ports.said == [f"{HONEST} {phrases.text(kn.STATE_CLAIM_PHRASE, subjects='Wi-Fi и роутер')}"]
    assert not any("роутер живы" in text for text in ports.said)


def test_backed_claim_goes_as_is():
    ports = Ports([_query("get_robot_status"), _speech(STATUS_SPEECH)], {"get_robot_status": ROBOT_STATUS})
    _run(ports, "какой у тебя сейчас статус")
    assert ports.said == [STATUS_SPEECH] and len(ports.asked) == 2


def test_personality_is_not_judged_by_the_table():
    """Таблица агентов одна (``STATE_CLAIM_AGENTS``): речь Личности этим правилом не судится."""
    assert "personality" not in kn.STATE_CLAIM_AGENTS
    ports = Ports([_speech("Батарея в норме.")])
    _run(ports, "как ты", agent="personality")
    assert ports.said == ["Батарея в норме."]


def test_code_phrase_exists_in_catalog():
    assert kn.STATE_CLAIM_PHRASE in kn.PHRASE_KEYS
    assert "Wi-Fi" in state_claim.phrase(("wifi",))


# -- 2. «контейнеры/ноды» - свой тул, без всей телеметрии ------------------------------------------------------------


def _names(text):
    return {t.name for t in available_tools(Session("operator").snapshot, None, "operator", text)}


def test_live_11_containers_slice_is_container_status_not_metrics():
    names = _names("статус контейнеров")
    assert "container_status" in names
    assert "show_metrics" not in names, "show_metrics up→cpu→memory крутился до потолка (№11)"


def test_nodes_slice_is_node_status():
    names = _names("статус нод")
    assert "ros2_node_status" in names and "show_metrics" not in names


def test_metrics_words_still_open_show_metrics():
    assert "show_metrics" in _names("покажи CPU")
    assert {"read_logs", "container_status"} <= _names("покажи логи контейнера voice-assistant")


# -- 3. повтор того же query в ходе ----------------------------------------------------------------------------------


def test_live_1_repeated_status_read_is_not_executed():
    """№1: ``get_robot_status``, затем снова он же - второй не исполняется, модели - подсказка, ответ по данным."""
    ports = Ports([_query("get_robot_status"), _query("get_robot_status"), _speech(STATUS_SPEECH)],
                  {"get_robot_status": ROBOT_STATUS})
    _run(ports, "какой у тебя сейчас статус")
    assert ports.executed == [["get_robot_status"]]
    assert kn.QUERY_REPEAT_HINT in ports.asked[-1][-1]["content"]
    assert ports.said == [STATUS_SPEECH]


def test_repeat_after_hint_is_closed_by_code():
    """Повтор и после подсказки - модель больше не зовётся, говорит код по данным хода."""
    ports = Ports([_query("get_robot_status")] * 3 + [_speech("лишний вызов")], {"get_robot_status": ROBOT_STATUS})
    _run(ports, "какой у тебя сейчас статус")
    assert ports.executed == [["get_robot_status"]]
    assert len(ports.asked) == 3 and ports.said and "лишний вызов" not in ports.said


def test_same_tool_other_args_is_a_new_read():
    ports = Ports([_query("show_metrics", query="up"), _query("show_metrics", query="cpu"), _speech("CPU 0.3.")],
                  {"show_metrics": METRICS_UP})
    _run(ports, "покажи CPU")
    assert ports.executed == [["show_metrics"], ["show_metrics"]]
