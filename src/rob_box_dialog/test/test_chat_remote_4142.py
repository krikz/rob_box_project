"""Хвосты пульта робота из чата Telegram (#4142, после PR #4139 / #4040; решение Шифу 28.10 по «спой» из чата).

1. «Спой» из чата — тот же путь пения, что у голоса (хост ``TurnEngine(sing=…)``), музыка — музыка робота; явные
   указания: голос («спой голосом X» — сегменты этим голосом), подложка-пьеса («под Калинку» — поверх неё).
2. Лицо в тишине Личности: «Показываю на роботе» — только по исходу показа (ADR-0148).
3. Конец выступления закрывает его запись «идёт» и в окне чата.
4. Выступление + лицо команды — одно сообщение в чат.
5. Слова сцен в чат — русские, из одного словаря (``knowledge.SCENE_WORDS``).
6. Просьба сделать роботом, а команда — только текст в чат: нарушение и повтор, затем фраза кода (классы 1 и 3 #4040).
"""

import asyncio
from types import MappingProxyType

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog import phrases
from rob_box_dialog.address import Addressed
from rob_box_dialog.chats import ChatPorts, ChatTurns, channel_ref, face_words, quote, show_phrase
from rob_box_dialog.command import Command, Expression, Performance, ToolCall, current_form
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import MusicStateRef, Session
from rob_box_dialog.turn import Answer, Shown, TurnEngine, asked_voice, over_ordered_music, robot_owed
from rob_box_dialog.turn_log import PERFORM_STARTED

BOAR = ("Вот идёт кабан по грязи, катится с холма как камень с горы", "Роллинг ап зе Хилл — кабан летит вниз")
SONG = Performance(kind="song", segments=BOAR, backing="beat")
CLAIM = "Покекал немного: глазами поводил, повертелся."


class Robot:
    """Пульт робота (как ``dialog_node.RobotRemote``): ``refuse`` — исход показа лица («» — показано)."""

    def __init__(self, refuse=""):
        self.refuse = refuse
        self.performed, self.shown, self.turns = [], [], []

    def perform(self, turn_id, perf, edit):
        self.performed.append((perf, edit))
        self.turns.append(turn_id)
        return {"status": "pending", "block_id": f"perform-{turn_id}"}

    def show(self, turn_id, expression):
        if self.refuse:
            return self.refuse
        self.shown.append(expression)
        return ""


class Base:
    """Общие порты хоста: ответы LLM по очереди; что видела модель (форма хода, тулы, сообщения); исходы действий —
    ``done`` (``status`` — исход по имени тула)."""

    def __init__(self, answers, status=None):
        self.answers, self.executed, self.asked = list(answers), [], []
        self.status = dict(status or {})

    async def ask(self, messages, tools, timeout_s):
        self.asked.append((current_form(), tuple(t.name for t in tools), list(messages)))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([a.tool for a in actions])
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.call_class(a.tool, a.args),
                             self.status.get(a.tool, "done"), data=MappingProxyType(dict(a.args)))
                for n, a in enumerate(actions)]

    def intent(self, turn_id, epoch, kind):
        return None


class Host:
    """Хост чата, как ``dialog_node.build_chats``: ``sing`` — путь пения хоста (тот же, что у Личности)."""

    def __init__(self, answers, robot=None, sing=None, status=None):
        self.base = Base(answers, status)
        self.robot = Robot() if robot is None else robot
        self.sent = []
        self.routed = []
        self.sing = sing
        self.chats = ChatTurns(self._make, ("робот",), lambda line: None)

    def _route(self, text, recent, playing):
        self.routed.append((text, playing))
        return self.sing(text) if callable(self.sing) else (self.sing or "")

    def _make(self, chat_id):
        session = Session("telegram", session_id=channel_ref(chat_id))
        ports = ChatPorts(self.base, chat_id, lambda ref, text, sid: self.sent.append(text), lambda line: None,
                          robot=self.robot)
        return TurnEngine(session, ports, system_prompt="Ты пульт робота в чате.",
                          sing=self._route if self.sing is not None else None)

    def say(self, text, chat_id=7):
        decision = self.chats.decide(chat_id, text, 0.0)
        assert isinstance(decision, Addressed)
        return asyncio.run(self.chats.run(chat_id, decision))


def _answer(command):
    return Answer(command, "deepseek", follow=lambda data: [{"role": "user", "content": data}])


# --- 1. «спой» из чата — тот же путь пения, что у голоса ----------------------------------------------------------


@pytest.mark.parametrize("route, closed, needed", [
    ("live:song", ("generate_music", "request_music"), ""),
    ("generate:song:V6", (), "generate_music"),
])
def test_sing_from_chat_takes_the_route_the_voice_path_decides(route, closed, needed):
    """Решение Шифу 28.10: у чата то же правило, что у голоса — путь пения решает код хоста до модели (живое под синт
    или Suno), а не «всегда выступление» из правила промпта. Форма хода чата — форма этого пути."""
    host = Host([_answer(Command(perform=SONG))], sing=route)

    host.say("спой песенку про кабана")

    form, tools, _ = host.base.asked[0]
    assert host.routed == [("спой песенку про кабана", False)], "чат спрашивает тот же путь пения"
    assert form.sing == route and form.sing_form is kn.SING_FORMS[kn.sing_family(route)]
    assert not set(closed) & set(tools), "закрытые путём тулы чату не предложены"
    assert not needed or needed in tools, "тул, который путь требует, у чата есть (Suno — generate_music)"


def test_chat_sing_route_sees_the_robots_music_live_over():
    """Чат — пульт робота: «под этот бит зачитай» судится по музыке РОБОТА (снимок хоста), как у голоса (n810)."""
    host = Host([_answer(Command(perform=Performance("rap", BOAR)))], sing="live:rap")
    host.chats.set_music(MusicStateRef("playing", track_id="t1"))

    host.say("под этот бит зачитай рэп")

    form, _, _ = host.base.asked[0]
    assert host.routed == [("под этот бит зачитай рэп", True)]
    assert form.sing_over and form.sing_form is kn.SING_FORMS["live_over"]


def test_robot_music_reaches_existing_and_new_chats():
    host = Host([])
    old = host.chats.engine(1)
    host.chats.set_music(MusicStateRef("playing", track_id="t1"))

    assert old.session.snapshot.music_playing
    assert host.chats.engine(2).session.snapshot.music_playing, "новый чат начинает со снимка музыки робота"
    host.chats.set_music(MusicStateRef("idle"))
    assert not old.session.snapshot.music_playing


def test_asked_voice_sings_every_segment_of_this_performance_in_that_voice():
    """«Спой голосом Алисы»: голос команды (форма открывает его только по просьбе о голосе) — голос всех сегментов
    выступления один раз; голос робота (``set_voice``) не меняется."""
    host = Host([_answer(Command(perform=SONG, voice="alice"))])

    host.say("спой голосом Алисы про кабана")

    perf = host.robot.performed[0][0]
    assert perf.voice_per_segment == ("alice", "alice")
    assert asked_voice(Command(perform=SONG)).perform.voice_per_segment is None, "без просьбы — голос робота"
    roles = Performance("tale", BOAR, voice_per_segment=("", "zhenya"))
    assert asked_voice(Command(perform=roles, voice="alice")).perform is roles, "голоса ролей решил код (#4084)"


def test_performance_next_to_the_piece_it_ordered_goes_over_that_piece():
    """«Агитируй толпу под Калинку-Малинку» (Шифу 28.10): синт «Калинки» — подложка, выступление поверх неё
    (``backing=none``): без второго бита и без вопроса «выключить и зачитать?»."""
    piece = ToolCall("request_music", {"intent": "melody", "text": "калинку-малинку"})
    agitation = Performance("poem", ("Народ, не стой, иди сюда!", "Робот зажигает под Калинку!"), backing="beat")
    host = Host([_answer(Command(actions=(piece,), perform=agitation))])

    host.say("агитируй толпу под Калинку-Малинку")

    assert host.base.executed == [["request_music"]]
    assert [p.backing for p, _ in host.robot.performed] == ["none"]


def test_performance_keeps_its_backing_when_the_ordered_music_did_not_start():
    piece = ToolCall("request_music", {"intent": "melody", "text": "калинку"})
    cmd = Command(actions=(piece,), perform=Performance("rap", BOAR, backing="beat"))
    rejected = ActionResult("t/0", 0, "telegram", "request_music", kn.call_class("request_music", piece.args),
                            "rejected", "busy")
    done = ActionResult("t/0", 0, "telegram", "request_music", kn.call_class("request_music", piece.args), "done")

    assert over_ordered_music(cmd, [rejected]).perform.backing == "beat"
    assert over_ordered_music(cmd, [done]).perform.backing == "none"
    assert over_ordered_music(Command(perform=SONG), []).perform.backing == "beat", "без заказа музыки — как было"


# --- 2. лицо в тишине Личности: фраза по исходу показа ------------------------------------------------------------


@pytest.mark.parametrize("cmd", [
    Command(actions=(ToolCall("play_animation", {"animation": "victory"}),)),
    Command(expression=Expression(emotion="happy")),
])
def test_face_from_chat_in_personality_silence_is_not_claimed_as_shown(cmd):
    """face_node в тишине отвечает ``skipped{silenced}`` (рот заклеен): «Показываю на роботе: …» в чат не уходит —
    фраза кода о причине; ответ лицом не записан в окне показанным."""
    host = Host([_answer(cmd)], robot=Robot(refuse=kn.REMOTE_SILENCED))

    host.say("покажи победу")

    assert host.robot.shown == []
    assert host.sent == [phrases.text(f"{kn.REMOTE_SHOW_REJECTED_PHRASE}.{kn.REMOTE_SILENCED}")]
    window = host.chats.engine(7).log.window()
    assert not any("ответ лицом" in str(e.content) for e in window), "лицо не записано показанным"


def test_show_phrase_follows_the_outcome_of_the_show():
    victory = Shown(emotion="victory")
    assert show_phrase("", victory) == "Показываю на роботе: победа."
    assert show_phrase(kn.REMOTE_SILENCED, victory).startswith("Не показал на роботе")
    assert show_phrase("operator_hold", victory) == phrases.text(f"{kn.PERFORM_REJECTED_PHRASE}.operator_hold")
    assert show_phrase("unknown_reason", victory) == phrases.text("local.rejected")


def test_every_show_refusal_has_its_phrase():
    from rob_box_dialog.address import HELD

    assert HELD in kn.REMOTE_SHOW_REFUSALS
    assert all(f"{kn.REMOTE_SHOW_REJECTED_PHRASE}.{r}" in phrases.catalog() for r in kn.REMOTE_SHOW_REFUSALS)


# --- 3. конец выступления закрывает запись и в окне чата ----------------------------------------------------------


def test_end_of_the_robot_performance_closes_its_record_in_the_chat_window():
    """block_end хоста выступления раньше уходил только в журнал Личности: модель чата видела «выступление идёт»,
    пока ход не выпадал из окна (лишний edit_pending, неверный ответ на «что делаешь»)."""
    host = Host([_answer(Command(perform=SONG))])
    host.say("спой")
    log = host.chats.engine(7).log
    assert log.performing()

    assert host.chats.performance_ended(host.robot.turns[0], "done", 2)

    assert not log.performing()
    record = next(e for e in log.window() if e.role == "tool").content
    assert record["tool"] == "perform" and record["status"] != PERFORM_STARTED
    assert record["data"]["segments_done"] == 2


def test_performance_end_without_chats_closes_nothing():
    assert ChatTurns(lambda chat_id: None).performance_ended("t1", "done") is False


# --- 4. выступление + лицо команды — одно сообщение ---------------------------------------------------------------


def test_perform_with_expression_from_chat_is_one_message():
    """«Пою на роботе: «…»» — текст хода в чат; лицо команды рядом с ним — настроение, а не второе сообщение
    «Показываю на роботе: happy.»."""
    host = Host([_answer(Command(perform=SONG, expression=Expression(emotion="happy")))])

    host.say("спой")

    assert host.sent == [phrases.text("remote.perform.song", line=quote(BOAR[0]))]
    assert host.robot.shown == []


# --- 5. русские слова сцен из одного словаря ----------------------------------------------------------------------


def test_scene_words_are_russian_and_cover_every_scene_name():
    names = {*kn.KNOWN_ANIMATIONS, *kn.LAB_SCENES, *kn.EMOTIONS.values(), *kn.GESTURES}
    assert names <= set(kn.SCENE_WORDS), f"нет слова: {sorted(names - set(kn.SCENE_WORDS))}"
    assert all(any("а" <= ch <= "я" or ch == "ё" for ch in word.lower()) for word in kn.SCENE_WORDS.values())


def test_face_words_in_chat_are_russian_not_scene_names():
    assert face_words(Expression(emotion="happy", gesture="wink")) == "радость, подмигивание"
    queue = Shown(emotion="happy", then=(Expression(emotion="sad"), Expression(emotion="victory")))
    assert face_words(queue) == "радость, грусть, победа"


def test_bare_face_answer_from_chat_names_the_face_in_russian():
    host = Host([_answer(Command(expression=Expression(emotion="happy", gesture="wink")))])

    host.say("просто чтоб ты покекал")

    assert host.sent == ["Показываю на роботе: радость, подмигивание."]


# --- 6. просьба сделать роботом, а команда — только текст в чат ---------------------------------------------------


def test_robot_ask_answered_by_chat_text_alone_goes_back_to_the_model():
    """26.10 20:52: «Покекай роботом» → «Покекал немного: глазами поводил, повертелся» без единого тула. Код по словам
    человека и структуре команды: текст о несделанном в чат не идёт, модель получает нарушение и делает роботом."""
    show = Command(actions=(ToolCall("play_animation", {"animation": "happy"}),))
    host = Host([_answer(Command(speech=CLAIM)), _answer(show)])

    host.say("Покекай роботом")

    assert len(host.base.asked) == 2, "повтор с нарушением"
    assert kn.VIOLATION_REMOTE_OWED in host.base.asked[1][2][-1]["content"]
    assert host.base.executed == [["play_animation"]]
    assert CLAIM not in host.sent
    assert host.sent == ["Показываю на роботе: радость."]


def test_through_the_robot_ask_ends_with_say_aloud_not_chat_text():
    """26.10 20:53: «Именно через робота чтобы хороший кек» → текст в чат; теперь нарушение → ``say_aloud``."""
    aloud = Command(speech="Кек-кек!", actions=(ToolCall("say_aloud", {"text": "Кек-кек!"}),))
    host = Host([_answer(Command(speech="Кек. Кек-кек. Кеково!")), _answer(aloud)])

    host.say("Именно через робота чтобы хороший кек")

    assert host.base.executed == [["say_aloud"]]
    assert "Кек. Кек-кек. Кеково!" not in host.sent


def test_robot_ask_still_answered_by_text_after_the_retry_gets_the_code_phrase():
    host = Host([_answer(Command(speech=CLAIM)), _answer(Command(speech=CLAIM))])

    host.say("покекай роботом")

    assert host.base.executed == []
    assert host.sent == [phrases.text(kn.REMOTE_OWED_PHRASE)], "о несделанном говорит код, а не модель"


def test_reference_question_in_chat_is_answered_by_text_without_a_retry():
    host = Host([_answer(Command(speech="Сейчас 15:30."))])

    host.say("который час")

    assert len(host.base.asked) == 1 and host.sent == ["Сейчас 15:30."]


def test_robot_owed_is_only_for_the_chat_agent_and_only_without_robot_outlets():
    from rob_box_dialog.deadline import TurnDeadline
    from rob_box_dialog.turn import _Turn

    chat, voice = Session("telegram").snapshot, Session("personality").snapshot
    turn = _Turn("t1", 0, "покекай роботом", "telegram", TurnDeadline(30.0))
    speech = Command(speech=CLAIM)
    assert robot_owed(chat, turn, speech)
    assert not robot_owed(voice, turn, speech), "у Личности речь и есть голос робота"
    assert not robot_owed(chat, turn, Command(speech="Кек", expression=Expression(emotion="happy")))
    assert not robot_owed(chat, turn, Command(perform=SONG))
    assert not robot_owed(chat, turn, Command(actions=(ToolCall("say_aloud", {"text": "кек"}),)))
    plain = _Turn("t2", 0, "расскажи анекдот", "telegram", TurnDeadline(30.0))
    assert not robot_owed(chat, plain, speech), "без просьбы сделать роботом — текст в чат законен"
    acted = _Turn("t3", 0, "покекай роботом", "telegram", TurnDeadline(30.0),
                  acted=(ToolCall("play_animation", {"animation": "happy"}),))
    assert not robot_owed(chat, acted, speech), "на роботе в этом ходе уже сделано"
