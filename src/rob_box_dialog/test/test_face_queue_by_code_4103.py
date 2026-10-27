"""#4103 (хвост #4083): «покажи по очереди радость, грусть и удивление» — очередь сцен лица собирает код, а не модель.

Живая проверка 27.10 18:55 (образ dev-20ad0ae): очередь в коде есть (#4088: ``Shown.then``, ``expression.queued``), но
DeepSeek на «Робот, покажи по очереди радость, грусть и удивление» делал ОДИН вызов (``play_animation thinking`` 5 с —
ни одной из трёх сцен), на «покажи по очереди нейтральное лицо и грусть» — один ``idle``, а на ``done`` лицо красил
рефлекс ``happy``. Решает код (ADR-0148): это грамматика реплики человека (Tier-1 ``show_queue``), имена сцен — один
словарь анимаций ``knowledge.KNOWN_ANIMATIONS`` + ``ANIMATION_ALIASES``; нейтраль показа — лицо покоя на своё время.
"""

import asyncio
import logging

import pytest

from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.agents import agent_spec, allows_tier1
from rob_box_dialog.command import Command, Expression, ToolCall
from rob_box_dialog.expression import held_face, hold_ms, merge, queued, reflex
from rob_box_dialog.grammar import match, show_queue_scenes
from rob_box_dialog.respond import ActionResult
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, Shown, TurnEngine
from rob_box_dialog.turn_log import ASSISTANT

DEFAULT = kn.FACE_SHOW_DEFAULT_S
REST = kn.REFLEXES["idle"].face[0]
#: Фразы живой проверки 27.10 18:55 (вейк снимает адресация).
N_THREE = "покажи по очереди радость, грусть и удивление"
N_NEUTRAL = "покажи по очереди нейтральное лицо и грусть"


class Ports:
    """Транспорт хода: LLM отвечает по очереди (пустой список — модель не должна быть вызвана), действия — исход
    ``statuses[tool]`` (по умолчанию ``done``), аргументы вызова поверх данных тула (как ``owner_event``)."""

    def __init__(self, answers=(), statuses=None):
        self.answers, self.statuses = list(answers), dict(statuses or {})
        self.asked, self.executed, self.said, self.stages, self.faces = [], [], [], [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        return self.answers.pop(0)

    async def execute(self, actions, snapshot, turn_id):
        self.executed.append([(a.tool, dict(a.args)) for a in actions])
        return [ActionResult(f"{turn_id}/{n}", snapshot.epoch, snapshot.agent, a.tool, kn.TOOL_CLASSES[a.tool],
                             self.statuses.get(a.tool, "done"), None, dict(a.args)) for n, a in enumerate(actions)]

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append((text, voice, emotion))
        return f"sp{len(self.said)}"

    def stage(self, turn_id, epoch, stage, status, reason="", cls=None, llm_calls=0, provider="", source="",
              expression=None):
        self.stages.append((stage, status))
        if expression is not None:
            self.faces.append((stage, expression))

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _run(ports, text, session=None):
    engine = TurnEngine(session or Session("personality"), ports, system_prompt="Ты робот.")
    return engine, asyncio.run(engine.run(Addressed(text, VOICE)))


def _requests(expression):
    """Что уйдёт лицу ``ExpressionRequest``-ами (как ``dialog_v2_ports``: ``stage_plan`` итога + ``show_queue``):
    ``(через сколько секунд после итога, сцена, ttl_ms)`` по порядку."""
    seconds = expression.duration_s
    done = merge(reflex("done"), "done", expression.emotion, expression.gesture, held_face(expression.face, seconds),
                 hold_ms(seconds))
    rest = queued(expression.duration_s, getattr(expression, "then", ()))
    return [(0.0, done.emotion, done.ttl_ms)] + [(at, p.emotion, p.ttl_ms) for at, p in rest]


# -- (1) грамматика реплики: имена по одной таблице алиасов --------------------------------------------------------


@pytest.mark.parametrize("text, scenes", [
    ("Робот, покажи по очереди радость, грусть и удивление.", ("happy", "sad", "surprised")),
    (N_THREE, ("happy", "sad", "surprised")),
    (N_NEUTRAL, ("idle", "sad")),
    ("сначала покажи радость, потом грусть", ("happy", "sad")),
    ("покажи радость, грусть и удивление", ("happy", "sad", "surprised")),  # «X, Y и Z» без слова «по очереди»
    ("покажи поочерёдно спокойное лицо, а затем злость", ("idle", "angry")),
    ("покажи молча по очереди радость и грусть", ("happy", "sad")),
    ("ничего не говори, покажи радость и удивление", ("happy", "surprised")),
    ("тихо покажи сон и победу", ("sleep", "victory")),
    ("покажи-ка по очереди happy и sad", ("happy", "sad")),
    ("изобрази по очереди удивление и задумчивость", ("surprised", "thinking")),
])
def test_show_queue_is_tier1_with_scenes_in_the_order_of_the_request(text, scenes):
    command = match(text, Session("personality").snapshot)
    assert command is not None and command.kind == kn.SHOW_QUEUE_KIND and not command.disputed
    assert command.args["scenes"] == scenes
    assert all(name in kn.KNOWN_ANIMATIONS for name in scenes)


@pytest.mark.parametrize("text, scenes", [
    ("покажи злость и гнев", ("angry",)),                                       # синонимы подряд - одна сцена
    ("покажи радость, грусть, печаль и удивление", ("happy", "sad", "surprised")),  # грусть и печаль - sad, sad
    ("покажи радость и счастье, потом грусть", ("happy", "sad")),
    ("покажи радость, грусть и снова радость", ("happy", "sad", "happy")),     # через другую сцену повтор остаётся
    ("покажи грусть и печаль и тоску", ("sad",)),
])
def test_adjacent_equal_scenes_of_the_queue_are_one(text, scenes):
    """Ревью #4109: синонимы подряд («злость и гнев» → ``angry``, ``angry``) дублировали сцену - лицо держало бы её дважды.
    Соседние одинаковые сцены - одна; повтор через другую сцену остаётся. Сколько имён названо - считается по словам
    реплики (``SHOW_QUEUE_MIN``..``SHOW_QUEUE_MAX``), поэтому «злость и гнев» - очередь кода из одной сцены."""
    command = match(text, Session("personality").snapshot)
    assert command is not None and command.kind == kn.SHOW_QUEUE_KIND
    assert command.args["scenes"] == scenes == show_queue_scenes(text)


def test_queue_of_synonyms_is_one_play_animation_without_llm():
    ports = Ports()
    _, agg = _run(ports, "покажи злость и гнев")
    assert ports.asked == [] and agg.llm_calls == 0
    assert ports.executed == [[("play_animation", {"animation": "angry"})]]
    (_, face), = ports.faces
    assert face == Shown(emotion="angry", duration_s=DEFAULT) and face.then == ()


@pytest.mark.parametrize("text", [
    "покажи радость",                                 # одна эмоция — путь модели, как раньше
    "покажи по очереди радость и кота",               # неизвестное слово
    "покажи кухню и гостиную",
    "покажи не радость, а грусть",                    # отрицание
    "покажи радость и грусть два раза",
    "покажи радость и грусть одновременно",
    "покажи по очереди",
    "покажи анимацию сон",
    "покажи как тебе грустно",
    "Робот, а теперь сирену и мигалку покажи, напугаем Валеру.",  # n903
    "радость и грусть",                               # без глагола показа
    "покажи радость, грусть, удивление, злость, сон и победу",  # длиннее SHOW_QUEUE_MAX
])
def test_not_a_show_queue_goes_to_the_model(text):
    assert show_queue_scenes(text) == ()
    command = match(text, Session("personality").snapshot)
    assert command is None or command.kind != kn.SHOW_QUEUE_KIND


def test_alias_table_is_one_source_with_the_tools():
    """Имена сцен очереди — та же таблица, что у тула (``normalize_animation``): алиасы ведут в словарь, сами в нём не
    стоят; реплика человека и аргумент LLM сводятся одним правилом."""
    for alias, target in kn.ANIMATION_ALIASES.items():
        assert alias not in kn.KNOWN_ANIMATIONS and target in kn.KNOWN_ANIMATIONS, alias
        assert alias == alias.lower() and "ё" not in alias, alias
    assert [kn.animation_name(w) for w in ("Радость", "грусть", "удивлённое", "нейтральное", "HAPPY", "кот", "")] == [
        "happy", "sad", "surprised", "idle", "happy", None, None]


def test_show_queue_is_the_personality_grammar_only_and_not_in_silence():
    """Лицо есть только у Личности: в чате и у оператора «покажи по очереди…» решает их модель. В тишине лицо —
    заклеенный рот (#3984): Tier-1 показа нет, ход молчит, как вся тишина."""
    assert allows_tier1(agent_spec("personality"), kn.SHOW_QUEUE_KIND)
    assert not allows_tier1(agent_spec("telegram"), kn.SHOW_QUEUE_KIND)
    assert not allows_tier1(agent_spec("operator"), kn.SHOW_QUEUE_KIND)
    session = Session("personality")
    session.push_scene("Silence", "user")
    assert match(N_THREE, session.snapshot) is None


# -- (1) ход: очередь кода без LLM, каждая сцена ≥ минимума --------------------------------------------------------


def test_three_emotions_are_three_face_requests_one_after_another_without_llm(caplog):
    """18:55:07 «покажи по очереди радость, грусть и удивление»: 0 вызовов LLM (модель делала один ``thinking``), три
    ``play_animation`` по порядку реплики, речи нет; на лицо — ``happy`` на итоге, ``sad`` и ``surprised`` очередью,
    каждая ``FACE_SHOW_DEFAULT_S`` (≥ ``FACE_SHOW_MIN_S``), следующая — когда кончилась предыдущая."""
    ports = Ports()
    with caplog.at_level(logging.INFO, logger="rob_box_dialog.turn"):
        engine, agg = _run(ports, N_THREE)
    assert ports.asked == [] and agg.llm_calls == 0
    assert (f"🧭 [turn-v2] tier1 kind=show_queue args={{'scenes': ('happy', 'sad', 'surprised')}} "
            f"turn={agg.turn_id}") in caplog.text, "строка лога для живой проверки"
    assert ports.executed == [[("play_animation", {"animation": n}) for n in ("happy", "sad", "surprised")]]
    assert ports.said == [] and (agg.stage, agg.status) == ("done", "finished")
    (stage, face), = ports.faces
    assert stage == "done" and face == Shown(emotion="happy", duration_s=DEFAULT, then=(
        Shown(emotion="sad", duration_s=DEFAULT), Shown(emotion="surprised", duration_s=DEFAULT)))
    ms = int(DEFAULT * 1000)
    assert _requests(face) == [(0.0, "happy", ms), (DEFAULT, "sad", ms), (2 * DEFAULT, "surprised", ms)]
    assert all(ttl >= kn.FACE_SHOW_MIN_S * 1000 for _, _, ttl in _requests(face))
    reply = [e.content for e in engine.log.window() if e.role == ASSISTANT]
    assert len(reply) == 1 and all(n in reply[0] for n in ("happy", "sad", "surprised")), (
        "окно LLM видит ответ действием: просьба не висит неотвеченной (#4066)")


def test_neutral_face_then_sadness_holds_the_rest_face_not_the_happy_reflex():
    """18:55:42 «покажи по очереди нейтральное лицо и грусть»: модель делала один ``idle``, на ``done`` — рефлекс
    ``happy`` 5 с, грусти нет. Теперь: лицо покоя на своё время (не ``happy``), затем ``sad`` очередью."""
    ports = Ports()
    _, agg = _run(ports, N_NEUTRAL)
    assert ports.asked == [] and agg.llm_calls == 0
    assert ports.executed == [[("play_animation", {"animation": "idle"}), ("play_animation", {"animation": "sad"})]]
    (_, face), = ports.faces
    ms = int(DEFAULT * 1000)
    assert _requests(face) == [(0.0, REST, ms), (DEFAULT, "sad", ms)]
    assert _requests(face)[0][1] != reflex("done").emotion, "нейтраль не красится рефлексом happy"


def test_silent_ask_in_the_queue_is_the_same_queue_without_speech():
    ports = Ports()
    _, agg = _run(ports, "покажи молча по очереди радость и грусть")
    assert ports.said == [] and ports.asked == [] and agg.llm_calls == 0
    assert [e for _, e, _ in _requests(ports.faces[0][1])] == ["happy", "sad"]


def test_rejected_scene_says_the_outcome_and_shows_no_queue():
    """Тул не принял сцену: «радостно» о провале не бывает — очереди нет, фраза исхода по результату, не молчание."""
    ports = Ports(statuses={"play_animation": "rejected"})
    _, agg = _run(ports, N_THREE)
    assert ports.asked == [] and ports.faces == []
    assert len(ports.said) == 1 and ports.said[0][0]


# -- (3) одна эмоция или неизвестное слово — путь модели, как сейчас -----------------------------------------------


@pytest.mark.parametrize("text", ["покажи радость", "покажи по очереди радость и кота"])
def test_one_emotion_or_unknown_word_is_the_model_path(text):
    cmd = Command(actions=(ToolCall("play_animation", {"animation": "happy"}),))
    ports = Ports([Answer(cmd, "deepseek")])
    _, agg = _run(ports, text)
    assert len(ports.asked) == 1 and agg.llm_calls == 1
    assert ports.executed == [[("play_animation", {"animation": "happy"})]]
    (_, face), = ports.faces
    assert face == Shown(emotion="happy", duration_s=DEFAULT)


def test_model_single_neutral_call_is_the_rest_face_too():
    """Та же нейтраль вызовом модели («покажи нейтральное лицо» — одна сцена, путь модели): лицо покоя на своё время,
    а не рефлекс ``happy``."""
    cmd = Command(actions=(ToolCall("play_animation", {"animation": "idle", "duration": 4}),))
    ports = Ports([Answer(cmd, "deepseek")])
    _run(ports, "покажи нейтральное лицо")
    (_, face), = ports.faces
    assert _requests(face) == [(0.0, REST, int(kn.FACE_SHOW_MIN_S * 1000))]


def test_ordinary_neutral_command_expression_still_shows_nothing():
    """Нейтраль экспрессии команды (значение по умолчанию схемы) — по-прежнему «нечего показать»: правило нейтрали
    показа касается только сцены, которую попросили показать."""
    ports = Ports([Answer(Command(speech="Привет!", expression=Expression(emotion="neutral")), "deepseek")])
    _run(ports, "привет")
    assert ports.faces == []
    assert queued(5.0, [Expression(emotion="neutral")]) == ()
    assert merge(reflex("done"), "done", "neutral") == reflex("done")
