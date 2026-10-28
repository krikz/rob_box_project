"""#4152 (решение Шифу 28.10): подложка выступления — по виду, а не по выбору модели (ADR-0148 п.2).

«На сказку необязательно музыку включать… на анекдот тоже… или стих… музыку надо включать, когда надо музыку включать —
рэпчик, песенка». Живой 28.10 15:12: «Робот, расскажи длинную сказку про енота-механика…» → модель дала ``perform
kind=tale backing=calm``, код заказал подложку (``[backing] track_id=…``). Теперь:

* таблица одна — ``knowledge.PERFORM_BACKING_BY_KIND``: по умолчанию подложка только у песни и рэпа, у сказки (и
  анекдота — он ``tale``) и стиха (и счёта как стиха — ``poem``) её нет;
* человек сам назвал музыку в этой реплике (``tools_view.music_asked`` - слова группы ``music``, текст ЧЕЛОВЕКА, не
  ответ модели) — подложка модели остаётся (``Form.backing``, ``command_form``);
* иначе валидатор делает ``backing`` = ``none`` (``dropped`` → лог ``fields_dropped``); ``none`` подложку не заказывает,
  и играющая музыка выступлению не мешает (ни вопроса «выключить и зачитать?», ни нового бита, ни снятия).
"""

import asyncio

import pytest

from perform_sim import Sim

from rob_box_dialog import command as cm
from rob_box_dialog import knowledge as kn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Busy, Command, Form, Invalid, Performance, parse, validate
from rob_box_dialog.plan.backing import Backing, MusicStop
from rob_box_dialog.plan.perform import AskReplace, edit_asks_backing
from rob_box_dialog.session import Session
from rob_box_dialog.tools_view import music_asked, requested_groups
from rob_box_dialog.turn import Answer, TurnEngine, _Turn, command_form

#: Форма хода, где человек музыку не называл (``command_form`` для «расскажи сказку»); открытая форма (по умолчанию) —
#: музыку просили или это не ход (режим команд v1, тесты).
UNASKED = Form(passed=False, voice=False, backing=False)
ASKED = Form(passed=False, voice=False, backing=True)
SEGS = ["Жил-был енот-механик в старом гараже.", "Чинил он всё на свете, даже дождь."]
NO_TOOLS = ()  # команда без действий: тулы среза валидатору не нужны


def _perform(kind, backing, segments=SEGS, **extra):
    return {"perform": {"kind": kind, "backing": backing, "segments": segments,
                        **({"cast": [{"name": "Енот", "gender": "male"}], "role_per_segment": ["Енот"] * len(segments)}
                           if kind in kn.PERFORM_ROLE_KINDS else {}), **extra}}


def _check(raw, form):
    return validate(parse(raw), NO_TOOLS, form=form)


def _form_of(text):
    return command_form(_Turn("t1", 0, text, VOICE, deadline=None), Session("personality").snapshot)


# -- таблица: одна, полная, песня и рэп = «поют» --------------------------------------------------------------------


def test_the_table_decides_every_performance_kind_and_only_sung_kinds_have_a_default_backing():
    assert set(kn.PERFORM_BACKING_BY_KIND) == set(kn.PERFORM_KINDS), "новый вид выступления получает решение о бите"
    with_default = {kind for kind, default in kn.PERFORM_BACKING_BY_KIND.items() if default}
    assert with_default == set(kn.SING_PERFORM_KINDS) == {"song", "rap"}
    assert {k for k in kn.PERFORM_KINDS if not kn.PERFORM_BACKING_BY_KIND[k]} == {"tale", "poem"}


# -- валидатор: сказка, анекдот, стих без просьбы о музыке → без подложки ---------------------------------------------


@pytest.mark.parametrize("kind, backing", [
    ("tale", "calm"),  # сказка (n412, 15:12 28.10)
    ("tale", "beat"),  # анекдот идёт видом tale
    ("poem", "calm"),  # стих и счёт как стих (n802 «посчитай вслух»)
    ("poem", "beat"),
])
def test_tale_joke_and_poem_lose_the_models_backing_when_music_was_not_asked(kind, backing):
    cmd = _check(_perform(kind, backing), UNASKED)
    assert not isinstance(cmd, Invalid), cmd
    assert cmd.perform.backing == "none"
    assert cmd.dropped == ("perform.backing",), "лог fields_dropped: решение видно, а не молча"
    assert cmd.perform.segments == tuple(SEGS) and cmd.perform.kind == kind


@pytest.mark.parametrize("kind, backing", [("rap", "beat"), ("rap", "calm"), ("song", "calm"), ("song", "beat")])
def test_song_and_rap_keep_the_backing_the_model_chose(kind, backing):
    cmd = _check(_perform(kind, backing), UNASKED)
    assert cmd.perform.backing == backing and "perform.backing" not in cmd.dropped


@pytest.mark.parametrize("kind", ["tale", "poem"])
def test_tale_and_poem_keep_the_backing_when_the_human_asked_for_music(kind):
    cmd = _check(_perform(kind, "calm"), ASKED)
    assert cmd.perform.backing == "calm" and "perform.backing" not in cmd.dropped


def test_open_form_outside_a_turn_leaves_the_backing_as_it_was():
    assert validate(parse(_perform("tale", "calm")), NO_TOOLS).perform.backing == "calm"  # Form() по умолчанию


def test_backing_none_is_not_reported_as_dropped():
    cmd = _check(_perform("tale", "none"), UNASKED)
    assert cmd.perform.backing == "none" and cmd.dropped == ()


def test_a_value_outside_the_list_is_still_a_violation_not_a_silent_drop():
    bad = _check(_perform("tale", "loud"), UNASKED)
    assert isinstance(bad, Invalid) and bad.reason == "bad_enum"


def test_a_tail_edit_of_a_running_tale_does_not_order_a_backing_either():
    """Правка идущей сказки («и про лису») с ``backing: calm`` от модели - без заказа бита (``edit_asks_backing``)."""
    raw = {"edit_pending": True, **_perform("tale", "calm", ["А ещё в гараже жила лиса."])}
    busy = Busy(active_segment=1)
    cmd = validate(parse(raw), NO_TOOLS, busy=busy, form=UNASKED)
    assert not isinstance(cmd, Invalid), cmd
    assert cmd.perform.backing == "none" and edit_asks_backing(cmd.perform, False) is False
    asked = validate(parse(raw), NO_TOOLS, busy=busy, form=ASKED)
    assert asked.perform.backing == "calm" and edit_asks_backing(asked.perform, False) is True


# -- слова человека: одна таблица с группой музыкальных тулов ----------------------------------------------------------


@pytest.mark.parametrize("text", [
    "Робот, расскажи длинную сказку про енота-механика",  # 15:12 28.10
    "расскажи сказку",
    "расскажи анекдот",
    "расскажи стих про осень",
    "посчитай вслух до десяти",  # n802
    "включи сказку на ночь",  # «включи»/«поставь» - общие глаголы, музыку не называют
    "расскажи сказку про красную шапочку",
])
def test_command_form_closes_the_default_backing_when_the_human_did_not_name_music(text):
    form = _form_of(text)
    assert form.backing is False and not music_asked(text)


@pytest.mark.parametrize("text", [
    "расскажи сказку под спокойную музыку",
    "расскажи сказку с музыкой",
    "расскажи анекдот под бит",
    "прочитай стих с подложкой",
    "расскажи сказку под ритм",
    "зачитай рэп про енота",
    "спой песню про енота",
])
def test_command_form_opens_the_backing_when_the_human_named_music(text):
    assert _form_of(text).backing is True and music_asked(text)


def test_music_words_are_the_music_group_of_the_tool_slice_not_a_second_table():
    """Слово открывает и музыкальные тулы среза, и подложку выступления: то, что просит музыку, — одно знание."""
    for text in ("под музыку", "с битом", "с подложкой", "под ритм"):
        assert "music" in requested_groups(text) and music_asked(text)


def test_the_form_in_the_log_and_the_schema_are_unchanged_by_the_new_field():
    """Подложка - не поле верхнего уровня: схема команды остаётся той же, закрытые поля формы - прежние."""
    assert UNASKED.closed() == ("passed", "voice") and ASKED.closed() == ("passed", "voice")
    assert Form().closed() == ()


class _FormPorts:
    """Порт хода: запоминает форму команды на момент вызова LLM (её читает мост)."""

    def __init__(self):
        self.forms = []

    async def ask(self, messages, tools, timeout_s):
        self.forms.append(cm.current_form())
        return Answer(Command(speech="Хорошо."), "deepseek")

    async def execute(self, actions, snapshot, turn_id):
        return []

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        return "sp1"

    def stage(self, turn_id, epoch, stage, status, reason="", cls=None, llm_calls=0, provider="", source=""):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


@pytest.mark.parametrize("text, asked", [
    ("расскажи длинную сказку про енота-механика", False),
    ("расскажи сказку под спокойную музыку", True),
])
def test_the_turn_puts_the_music_request_of_this_reply_into_the_form_the_model_is_judged_by(text, asked):
    ports = _FormPorts()
    asyncio.run(TurnEngine(Session("personality"), ports, system_prompt="Ты робот.").run(Addressed(text, VOICE)))
    assert [form.backing for form in ports.forms] == [asked]
    assert cm.current_form() == Form(), "после хода форма не течёт в чужие вызовы"


# -- выступление: подложки нет — играющая музыка не тронута -----------------------------------------------------------


def _tale(backing):
    return Performance(kind="tale", segments=tuple(SEGS), backing=backing)


def test_tale_without_backing_over_playing_music_asks_nothing_and_stops_nothing():
    """«Расскажи сказку» поверх играющего трека: ни вопроса «выключить и зачитать?», ни нового бита, ни снятия."""
    sim = Sim(_tale("none"), bpm=120)
    sim.start(playing="Калинка-Малинка")
    sim.run(300)
    assert sim.asks == [] and sim.music_log == []
    assert not [e for _, e in sim.effects if isinstance(e, (Backing, MusicStop, AskReplace))]
    assert sim.block.end_reason == "done" and sim.block.backing.state == "none"


def test_the_models_calm_over_playing_music_asks_to_replace_it_unless_the_validator_took_the_backing_away():
    """Контроль: без решения кода та же сказка с ``calm`` спрашивает «выключить и зачитать?» (``AskReplace``); после
    валидатора (музыку не просили) подложка ``none`` - вопроса нет, музыка играет дальше."""
    sim = Sim(_tale("calm"), bpm=120)
    sim.start(playing="Калинка-Малинка")
    assert len(sim.asks) == 1
    perf = _check(_perform("tale", "calm"), UNASKED).perform
    after = Sim(Performance(kind=perf.kind, segments=perf.segments, backing=perf.backing), bpm=120)
    after.start(playing="Калинка-Малинка")
    assert after.asks == [] and after.music_log == []
