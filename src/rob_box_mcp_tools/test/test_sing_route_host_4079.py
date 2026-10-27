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


class ModelCalled(BaseException):
    """Модель чойсера позвана, где решает код: ``BaseException`` проходит сквозь ``except Exception`` выбирателя
    (``AssertionError`` молча упал бы в детерминированный ответ — ревью #4071 круг 2 NIT-8)."""


class NoModel:
    def choose(self, question, *, deadline_ms=0):
        raise ModelCalled(f"модель чойсера позвана: {question.text}")


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
    # вилка Шифу 27.10: синт назван + пение — живое пение кодом, даже рядом с «сгенерируй/сочини/запиши»
    ("Робот, сгенерируй песню на Ренардо", "live:song"),
    ("Робот, сочини песню на синтезаторе", "live:song"),
    ("Робот, запиши рэп на Ренардо", "live:rap"),
    ("Робот, сгенерируй песню вживую", "live:song"),
    ("Робот, а теперь под этот бит зачитай короткий рэп про енотика.", "live:rap"),  # акт 8 n810
    # ревью #4071 B1/B2: не пение и не генерация — форма хода не меняется
    ("Робот, поставь музыку без вокала", ""), ("Робот, мне нравится твой голос", ""),
    ("Робот, ответь своим голосом", ""), ("Робот, не рассказывай спойлеры", ""),
    ("Робот, сделай песню потише", ""), ("Робот, запиши, что я люблю песни", ""),
    ("Робот, расскажи про песни, которые написала нейросеть", ""),
    # ревью #4071 круг 2 (BLOCK-1..5, NIT-2): не пение и не генерация («… на полную громкость» до модели не доходит —
    # громкость берёт Tier-1, поэтому здесь, на хосте)
    ("Робот, сделай свой голос помягче", ""), ("Робот, запиши название песни", ""),
    ("Робот, сделай песню на полную громкость", ""), ("Робот, сделай эту песню на повтор", ""),
    ("Робот, создай плейлист песен", ""), ("Робот, напиши текст песни", ""),
    ("Робот, мне нравится твой голос в этой песне", ""), ("Робот, прочитай статью про рэп", ""),
    ("Робот, я хочу спеть караоке", ""), ("Робот, не пой больше", ""),
    ("Робот, пой про енота", "live:song"),  # NIT-3: «пой» — глагол пения словом
])
def test_host_decides_the_sing_route_and_logs_it_for_the_harness(text, route):
    host, lines = _host()
    assert host(text, (), False) == route
    if route:
        assert len(lines) == 1 and lines[0].startswith("🎚️ [source] пение ") and lines[0].endswith(
            f"→ {route} ({'chooser:deterministic' if text.endswith('енота') and 'пой про' in text else 'code'})")
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


@pytest.mark.parametrize("text", ["Робот, сгенерируй песню на Ренардо", "Робот, сочини песню на синтезаторе"])
def test_generate_a_song_on_the_synth_is_a_performance_not_a_generation(text):
    """Решение Шифу 27.10: «он должен на «Сгенерируй песню на Ренардо» сделать перформ» — генерация закрыта, выступление
    обязательно (KIE на Ренардо не играет); отказа нет."""
    form, tools = _form_of(text)
    assert form.sing == "live:song" and "generate_music" not in tools
    gen = parse({"actions": [{"tool": "generate_music", "args": {"kind": "song", "theme": "енот"}}]})
    assert validate(gen, tools, form=form).reason == dkn.VIOLATION_SING_ROUTE
    assert isinstance(validate(parse({"perform": {"kind": "song", "segments": ["Енот", "поёт"]}}), tools, form=form),
                      Command)


@pytest.mark.parametrize("text", ["Робот, мне нравится твой голос", "Робот, запиши, что я люблю песни",
                                  "Робот, поставь музыку без вокала"])
def test_false_singing_or_generation_keeps_the_music_tools_in_the_slice(text):
    """Ревью #4071 B1/B2: ложное пение не убирает ``request_music``/``generate_music`` из среза и не требует ``perform``
    («сделай песню потише» до модели не доходит вовсе — громкость берёт Tier-1; путь хоста — тест выше)."""
    form, _tools = _form_of(text)
    assert form.sing == "" and form.sing_form is None


#: Ревью #4071 круг 2 (BLOCK-1..5): реплика → ответ модели, который форма пения запрещала бы. Теперь формы пения нет —
#: ответ проходит валидатор, ``request_music``/``generate_music`` в срезе, ``perform`` не обязателен.
ROUND_2 = [
    ("Робот, сделай свой голос помягче", {"actions": [{"tool": "set_voice", "args": {"voice": "female-shaonv"}}]}),
    ("Робот, сделай твой голос женским", {"actions": [{"tool": "set_voice", "args": {"voice": "female-shaonv"}}]}),
    ("Робот, запиши название этой песни", {"speech": "Записал."}),
    ("Робот, создай плейлист песен",
     {"actions": [{"tool": "request_music", "args": {"intent": "track", "text": "плейлист песен"}}]}),
    ("Робот, придумай название для песни", {"speech": "Назову её Енот."}),
    ("Робот, мне нравится твой голос и твои песни", {"speech": "Спасибо!"}),
    ("Робот, включи рэп и прочитай новости",
     {"actions": [{"tool": "request_music", "args": {"intent": "track", "text": "рэп"}}]}),
    ("Робот, я хочу спеть караоке", {"speech": "Давай!"}),
]


@pytest.mark.parametrize("text,answer", ROUND_2)
def test_false_singing_or_generation_of_the_review_round_2_passes_the_turn_validator(text, answer):
    form, tools = _form_of(text)
    assert form.sing == "" and form.sing_form is None  # форма пения не закрывает тулов и не требует perform
    assert isinstance(validate(parse(answer), tools, form=form), Command)


@pytest.mark.parametrize("text", ["Робот, сгенерируй песню не на синтезаторе", "Робот, сгенерируй песню про Ренардо"])
def test_a_negated_or_theme_engine_keeps_the_generation_the_person_asked_for(text):
    """NIT-1: «не на синтезаторе», «про Ренардо» — генерация, как просили: ``generate_music`` обязателен, выступление
    вместо неё — нарушение формы (код больше не делает обратное просьбе)."""
    form, tools = _form_of(text)
    assert form.sing == "generate:song:V6" and "generate_music" in tools
    song = parse({"perform": {"kind": "song", "segments": ["Енот", "поёт"]}})
    assert validate(song, tools, form=form).reason == dkn.VIOLATION_SING_ROUTE


def test_host_line_is_logged_once_per_sing_turn(caplog):
    host = SingRouteHost(DeterministicChooser(NoModel()), lambda: [], logging.getLogger("t").info)
    with caplog.at_level(logging.INFO):
        host("Робот, спой вживую", (), False)
    assert [r.getMessage() for r in caplog.records] == [
        "🎚️ [source] пение song слова=live кандидаты=[live:song] → live:song (code)"]
