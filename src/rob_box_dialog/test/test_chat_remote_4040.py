"""Чат Telegram — пульт робота (#4040, решение Шифу 28.10, ``knowledge.CHAT_OUTLETS``).

Просьбу действия на роботе исполняет РОБОТ от своего имени (выступление — хосту выступления, лицо — лицу робота), в
чат — фраза кода об исполнении из результата, а не текст песни; справку агент отвечает в чат, робот молчит; служебный
текст агента на робот не идёт. Случай 26.10: «спой песенку про кабана…» → ``perform`` ушёл в чат пятью сообщениями.
"""

import asyncio
from types import MappingProxyType

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import Addressed
from rob_box_dialog.chats import ChatPorts, ChatTurns, channel_ref, face_words, perform_phrase, quote, robot_face
from rob_box_dialog.command import Command, Expression, Performance, ToolCall, _COMMAND_FIELDS
from rob_box_dialog.respond import ActionResult, quiet_in_performance
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, Shown, TurnEngine, outcome_phrase

BOAR = ("Вот идёт кабан по грязи, катится с холма как камень с горы", "Роллинг ап зе Хилл — кабан летит вниз")
SONG = Performance(kind="song", segments=BOAR, backing="beat")
SERVICE = "Выполняю: отправил роботу, сейчас всё будет."


class Robot:
    """Пульт робота (как ``dialog_node.RobotRemote``): что ему отдал чат — выступления и лица."""

    def __init__(self, outcome=None):
        self.outcome = {"status": "pending", "block_id": "perform-x"} if outcome is None else outcome
        self.performed, self.shown = [], []

    def perform(self, turn_id, perf, edit):
        self.performed.append((perf, edit))
        return self.outcome

    def show(self, turn_id, expression):
        self.shown.append(expression)


class Base:
    """Общие порты хоста: ответы LLM по очереди, исходы действий — ``done`` с аргументами вызова в данных."""

    def __init__(self, answers):
        self.answers, self.executed = list(answers), []

    async def ask(self, messages, tools, timeout_s):
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([a.tool for a in actions])
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.call_class(a.tool, a.args),
                             "done", data=MappingProxyType(dict(a.args)))
                for n, a in enumerate(actions)]

    def intent(self, turn_id, epoch, kind):
        return None


class Host:
    """Хост чата, как ``dialog_node.build_chats``; ``sent`` — тексты в чат, ``robot`` — что ушло роботу."""

    def __init__(self, answers, robot=None):
        self.base = Base(answers)
        self.robot = Robot() if robot is None else robot
        self.sent = []
        self.chats = ChatTurns(self._make, ("робот",), lambda line: None)

    def _make(self, chat_id):
        session = Session("telegram", session_id=channel_ref(chat_id))
        ports = ChatPorts(self.base, chat_id, lambda ref, text, sid: self.sent.append(text), lambda line: None,
                          robot=self.robot)
        return TurnEngine(session, ports, system_prompt="Ты пульт робота в чате.")

    def say(self, text, chat_id=7):
        decision = self.chats.decide(chat_id, text, 0.0)
        assert isinstance(decision, Addressed)
        return asyncio.run(self.chats.run(chat_id, decision))


def _answer(command):
    return Answer(command, "deepseek", follow=lambda data: [{"role": "user", "content": data}])


def test_sing_from_chat_is_performed_by_the_robot_and_the_chat_gets_the_code_phrase_not_the_lyrics():
    """26.10: ``perform`` из чата уходил в чат пятью сообщениями, вслух — ничего. Теперь сегменты поёт робот."""
    host = Host([_answer(Command(perform=SONG))])

    host.say("спой песенку про кабана в стиле Роллинг ап зе Хилл")

    assert host.robot.performed == [(SONG, False)]
    assert host.sent == [phrases.text("remote.perform.song", line=quote(BOAR[0]))]
    assert host.sent[0].startswith("Пою на роботе: «Вот идёт кабан")
    assert not any(line in text for line in BOAR[1:] for text in host.sent), "текст песни в чат не идёт — только цитата"


def test_time_question_from_chat_is_answered_in_chat_and_the_robot_stays_silent():
    """Справка — ответ в чат; лицо команды рядом с текстом — настроение текста, лицо робота не меняется."""
    time_query = Command(actions=(ToolCall("get_current_time", {}),))
    reply = Command(speech="Сейчас 15:30.", expression=Expression(emotion="happy"))
    host = Host([_answer(time_query), _answer(reply)])

    host.say("который час")

    assert host.base.executed == [["get_current_time"]]
    assert host.sent == ["Сейчас 15:30."]
    assert host.robot.performed == [] and host.robot.shown == []


def test_show_rainbow_from_chat_plays_on_the_robot_face_and_the_chat_gets_the_code_phrase():
    host = Host([_answer(Command(actions=(ToolCall("play_animation", {"animation": "rainbow"}),)))])

    host.say("покажи радугу")

    assert host.base.executed == [["play_animation"]]
    assert len(host.robot.shown) == 1 and isinstance(host.robot.shown[0], Shown)
    assert host.robot.shown[0].emotion == "rainbow"
    assert host.sent == ["Показываю на роботе: rainbow."]


def test_service_text_of_the_chat_agent_goes_to_the_chat_and_never_to_the_robot():
    """Служебный текст агента («выполняю, отправил роботу») — текстом в чат; роботу — только действие."""
    cmd = Command(speech=SERVICE, actions=(ToolCall("play_animation", {"animation": "happy"}),))
    host = Host([_answer(cmd)])

    host.say("покекай роботом")

    assert host.robot.performed == [], "речь агента выступлением робота не звучит"
    assert [s.emotion for s in host.robot.shown] == ["happy"]
    assert host.sent == [SERVICE, "Показываю на роботе: happy."]


def test_bare_face_answer_from_chat_shows_on_the_robot_instead_of_silence():
    """26.10 20:53: команда из одной ``expression`` — в чат ничего, на роботе ничего. Теперь — лицо робота и фраза."""
    host = Host([_answer(Command(expression=Expression(emotion="happy", gesture="wink")))])

    host.say("просто чтоб ты покекал")

    assert [(s.emotion, s.gesture) for s in host.robot.shown] == [("happy", "wink")]
    assert host.sent == [phrases.text(kn.REMOTE_SHOW_PHRASE, face=face_words(host.robot.shown[0]))]


def test_refusal_of_the_robot_performer_is_said_by_code_without_lyrics():
    host = Host([_answer(Command(perform=SONG))], robot=Robot({"status": "rejected", "reason": "no_backing"}))

    host.say("спой")

    assert host.sent == [phrases.text("local.rejected")]


def test_without_a_robot_remote_the_lyrics_still_do_not_go_to_the_chat():
    """Пульта у хоста нет — честный отказ кодом (``no_robot``), а не старый путь «сегменты текстом в чат»."""
    sent = []
    ports = ChatPorts(Base([]), 1, lambda ref, text, sid: sent.append(text), lambda line: None)

    outcome = ports.perform("t1", 0, SONG, False)

    assert outcome == {"status": "rejected", "reason": kn.REMOTE_NO_ROBOT} and sent == []


def test_performance_from_chat_does_not_mute_a_later_music_outcome_in_the_chat():
    """#4075 молчит «Включаю музыку.» поверх рэпа в динамиках; в чат о запуске музыки на роботе код говорит всегда."""
    music = Command(actions=(ToolCall("request_music", {"genre": "lofi"}),))
    host = Host([_answer(Command(perform=SONG)), _answer(music)])
    host.say("спой")

    host.say("включи лоу-фай")

    started = ActionResult("x/0", 0, "telegram", "request_music", kn.TOOL_CLASSES["request_music"], "done")
    assert quiet_in_performance(started), "в динамиках поверх выступления этот успех молчит (#4075)"
    assert host.chats.engine(7).log.performing(), "выступление робота записано в окне чата"
    assert host.sent[-1] == outcome_phrase([started])


def test_perform_phrase_follows_the_hosts_outcome():
    assert perform_phrase(SONG, {"status": "pending"}).startswith("Пою на роботе: «")
    assert perform_phrase(Performance("rap", BOAR), {"status": "pending"}).startswith("Читаю рэп на роботе: «")
    assert perform_phrase(SONG, {"status": "done"}).startswith("Добавил к выступлению робота: «")
    assert perform_phrase(Performance("rap", (), backing="beat"), {"status": "done", "backing": True}) == \
        "Добавил бит к выступлению робота."
    assert perform_phrase(SONG, {"status": "rejected", "reason": "edit_without_active_perform"}) is None


def test_quote_is_a_short_report_not_the_song():
    long_line = "ла " * 60
    assert len(quote(long_line)) == kn.REMOTE_QUOTE_MAX_CHARS and quote(long_line).endswith("…")
    assert quote("  Вот   идёт кабан ") == "Вот идёт кабан"


def test_robot_face_is_an_action_on_the_robot_not_the_mood_of_chat_text():
    happy = Expression(emotion="happy")
    shown = Shown(emotion="rainbow")
    assert robot_face(shown, said=True) is shown, "сцена play_animation — действие на роботе"
    assert robot_face(happy, said=False) is happy, "ответ лицом без текста — на лицо робота"
    assert robot_face(happy, said=True) is None, "рядом с текстом в чат — настроение текста"
    assert robot_face(None, said=False) is None


def test_face_the_chat_port_did_not_show_is_not_written_as_shown_4073():
    from rob_box_dialog.deadline import TurnDeadline
    from rob_box_dialog.turn import _Turn, finish_stage

    snap = Session("telegram").snapshot
    turn = _Turn("t1", 0, "покекай", "telegram", TurnDeadline(30.0), stage="done", expression=Expression("happy"))
    robot = Robot()
    remote = ChatPorts(base=None, chat_id=7, send=lambda *a: None, log=lambda line: None, robot=robot)
    assert finish_stage(remote, turn, snap) == Expression("happy") and robot.shown == [Expression("happy")]
    remote.say("t1", 0, "Кек!")  # текст в чат — лицо его настроение, роботу не уходит
    assert finish_stage(remote, turn, snap) is None and len(robot.shown) == 1


def test_chat_outlets_table_names_command_fields_and_every_perform_kind_has_a_phrase():
    assert set(kn.CHAT_OUTLETS) <= set(_COMMAND_FIELDS)
    assert set(kn.CHAT_OUTLETS.values()) <= {kn.REMOTE_ROBOT, kn.REMOTE_CHAT}
    assert kn.CHAT_OUTLETS["speech"] == kn.REMOTE_CHAT and kn.CHAT_OUTLETS["perform"] == kn.REMOTE_ROBOT
    assert set(kn.REMOTE_PERFORM_PHRASES) == set(kn.PERFORM_KINDS)
    assert all(key in phrases.catalog() for key in kn.REMOTE_PERFORM_PHRASES.values())
