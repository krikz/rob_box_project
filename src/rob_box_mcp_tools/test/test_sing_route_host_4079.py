"""#4079 — хост пути пения хода Личности (``engine.sing_route.SingRouteHost``): решение ``rob_box_music`` → форма хода.

Шов: путь пения решается ДО основной модели (решение Шифу 27.10: «явные слова сужают кодом, голое «спой» решает
чойсер по контексту с перевесом живому»), ход диалога v2 сужает по нему команду структурой (``command.Form.sing``).
Здесь — связка целиком без LLM: реплика → ``choose_sing`` → id → форма хода → срез тулов и валидатор.
"""

import asyncio
import logging
from types import SimpleNamespace

import pytest

from rob_box_dialog import knowledge as dkn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, Form, current_form, parse, validate
from rob_box_dialog.session import Session
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_mcp_tools.engine.sing_route import GEN_KIND, SingRouteHost, gen_health
from rob_box_music.choice import Choice, DeterministicChooser

pytestmark = pytest.mark.unit


class NoModel:
    def choose(self, question, *, deadline_ms=0):
        raise AssertionError(f"модель чойсера позвана: {question.text}")


class Model:
    def __init__(self, pick):
        self.pick, self.questions = pick, []

    def choose(self, question, *, deadline_ms=0):
        self.questions.append(question)
        return Choice(self.pick, 0.8, "jev")


def _host(states=(), model=None):
    lines = []
    host = SingRouteHost(DeterministicChooser(model or NoModel()), lambda: list(states), lines.append)
    return host, lines


def _state(health):
    return SimpleNamespace(name="kie", kind=GEN_KIND, health=health)


@pytest.mark.parametrize("text,route", [
    ("Робот, спой под синт про енота", "live:song"),
    ("Робот, спой вживую", "live:song"),
    ("Робот, спой на синтезаторе", "live:song"),
    ("Робот, спой на Ренардо", "live:song"),
    ("Робот, сгенерируй песню про енота своим голосом", "generate:robot_song:V6"),
    ("Робот, запиши песню", "generate:song:V6"),
    ("Робот, спой про енота", "live:song"),  # голое «спой»: перевес живому без контекста
    ("Робот, сыграй на синтезаторе бит", ""),  # не пение: путь синтеза #4071, форма не меняется
    ("Робот, поставь синтвейв", ""),
])
def test_host_decides_the_sing_route_and_logs_it_for_the_harness(text, route):
    host, lines = _host()
    assert host(text, (), False) == route
    if route:
        assert len(lines) == 1 and lines[0].startswith("🎚️ [source] пение ") and lines[0].endswith(
            f"→ {route} ({'chooser:deterministic' if text.endswith('енота') and 'спой про' in text else 'code'})")
        assert "кандидаты=[" in lines[0]
    else:
        assert lines == []


def test_host_gives_the_chooser_the_context_of_the_dialog():
    """Голое «спой» после «сохрани для Бориса»: разрыв мал — решает модель чойсера и видит контекст процесса диалога
    (реплики, музыка, здоровье генерации с доски провайдеров)."""
    model = Model("generate:song:V6")
    host, lines = _host([_state("ok")], model)
    assert host("Робот, спой про енота", ("сохрани для Бориса",), True) == "generate:song:V6"
    text = model.questions[0].text
    assert "сохрани для Бориса" in text and "играет музыка: да" in text and "генерация: ok" in text
    assert lines[0].endswith("→ generate:song:V6 (chooser:jev)")


@pytest.mark.parametrize("health,ids", [("quota", ["live:song"]), ("down", ["live:song"]), ("auth", ["live:song"]),
                                        ("ok", ["live:song", "generate:song:V6"]),
                                        ("degraded", ["live:song", "generate:song:V6"])])
def test_dead_generation_on_the_provider_board_leaves_only_live(health, ids):
    host, lines = _host([_state(health)])
    host("Робот, спой про енота", (), False)
    assert f"кандидаты=[{', '.join(ids)}]" in lines[0]
    assert (health in dkn.PROVIDER_USABLE_HEALTH) is (len(ids) == 2)


def test_gen_health_reads_the_music_gen_record_only():
    assert gen_health([SimpleNamespace(kind="llm", health="down"), _state("quota")]) == "quota"
    assert gen_health([SimpleNamespace(kind="tts", health="down")]) == ""


# -- связка целиком: реплика → хост → форма хода → срез и валидатор ----------------------------------------------


class Ports:
    def __init__(self, answer):
        self.answer, self.asked, self.forms = answer, [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append([t.name for t in tools])
        self.forms.append(current_form())
        return self.answer

    async def execute(self, actions, snapshot, turn_id):
        return []

    def say(self, *args, **kwargs):
        return "sp"

    def stage(self, *args, **kwargs):
        pass

    def cancel_speech(self, *args, **kwargs):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, *args):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return {"status": "pending"}


def _form_of(text):
    host, _lines = _host()
    ports = Ports(Answer(Command(speech="…")))
    engine = TurnEngine(Session("personality"), ports, system_prompt="Ты робот.", sing=host)
    asyncio.run(engine.run(Addressed(text, VOICE)))
    return ports.forms[-1], ports.asked[-1]


def test_live_words_close_the_generation_and_require_singing_in_the_turn():
    form, tools = _form_of("Робот, спой под синт про енота")
    assert form.sing == "live:song" and "generate_music" not in tools and "request_music" not in tools
    gen = parse({"actions": [{"tool": "generate_music", "args": {"kind": "song", "theme": "енот"}}]})
    assert validate(gen, tools, form=form).reason == dkn.VIOLATION_SING_ROUTE
    song = validate(parse({"perform": {"kind": "song", "segments": ["Енот", "поёт"]}}), tools, form=form)
    assert isinstance(song, Command) and song.perform.backing == "beat"


def test_generation_words_require_the_generation_tool_in_the_turn():
    form, tools = _form_of("Робот, сгенерируй песню про енота своим голосом")
    assert form.sing == "generate:robot_song:V6" and "generate_music" in tools
    song = parse({"perform": {"kind": "song", "segments": ["Енот", "поёт"]}})
    assert validate(song, tools, form=form).reason == dkn.VIOLATION_SING_ROUTE


def test_play_on_the_synth_without_singing_keeps_the_turn_form_open():
    form, tools = _form_of("Робот, сыграй на синтезаторе бит")
    assert form == Form(passed=False, voice=False) and "request_music" in tools


def test_host_line_is_logged_once_per_sing_turn(caplog):
    host = SingRouteHost(DeterministicChooser(NoModel()), lambda: [], logging.getLogger("t").info)
    with caplog.at_level(logging.INFO):
        host("Робот, спой вживую", (), False)
    assert [r.getMessage() for r in caplog.records] == [
        "🎚️ [source] пение song слова=live кандидаты=[live:song] → live:song (code)"]
