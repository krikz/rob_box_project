"""#4029 — тул ``edit_music``: правка ИГРАЮЩЕГО трека на такте, а не заказ нового (ADR-0148: решает код).

Кейс Шифу 27.10 (живой робот, hotpatch PR #4071): рэп про енотика под бит → «и про комарика» (правка выступления) →
«сделай бит потяжелее, более металлический!» → ``request_music(intent=track)`` → платная генерация KIE, мост
``synth:breaks``, «подложка кончилась — дальше без тактов (synced=false)», «Включаю музыку.» посреди рэпа, через 40 с
сгенерированный трек сам. Марафон 27.10: n504/n506 — новый трек 60 BPM вместо правки, n505 — музыка не тронута.

Здесь настоящие ``PlayerOwner`` + ``RenardoAdapter`` + ``compose``/``render`` на симуляторе клока Renardo (как
``test_backing_deck``), хост пути хода и форма хода диалога v2 — без LLM и без звука.
"""

import asyncio
from types import SimpleNamespace

import pytest

from rob_box_core.tool_catalog import get_tool
from rob_box_dialog import knowledge as dkn
from rob_box_dialog.address import VOICE, Addressed
from rob_box_dialog.command import Command, parse, validate
from rob_box_dialog.session import MusicStateRef, Session
from rob_box_dialog.turn import Answer, TurnEngine
from rob_box_mcp_tools.engine.backing_deck import BackingDeck
from rob_box_mcp_tools.engine.renardo_adapter import RETUNE_LEAD_BEATS
from rob_box_mcp_tools.engine.search import ThemeHits
from rob_box_mcp_tools.engine.sing_route import SingRouteHost
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, EditMusicTool, RequestMusicTool, music_busy
from rob_box_mcp_tools.engine.track_edit import TrackEditor
from rob_box_music import knowledge as kn
from rob_box_music.choice import DeterministicChooser
from rob_box_music.model import BEATS_PER_BAR

from ._ros_stubs import RosStubs
from .test_backing_deck import _confirm
from .test_engine_session import V2_SYNTHS, Player, _rig
from .test_sing_route_host_4079 import NoModel, Ports

_ros = RosStubs()
with _ros:  # llm_adapter тянет rclpy — без ROS нужны заглушки
    from rob_box_mcp_tools.llm_adapter import TURN_CONTEXT_ARGS
_ros_stubs = _ros.fixture()

pytestmark = pytest.mark.unit

SHIFU = "Робот, сделай бит потяжелее, более металлический!"


class LivePlayer(Player):
    """Плеер Renardo: играющий на ``>>`` меняет атрибуты и играет дальше с текущей доли (фаза та же); каждый ``>>`` —
    в журнал ``(слот, доля клока, синт)``."""

    def __init__(self, clock, name, journal):
        super().__init__(clock, name)
        self.journal = journal

    def __rshift__(self, synth):
        self.journal.append((self.name, self.clock.beat, synth))
        if self.playing:
            return self
        return super().__rshift__(synth)


def _live_rig():
    """Рига ``test_engine_session`` с живыми плеерами и синтами, которые называют себя (видно, что встало)."""
    rig = _rig()
    rig.journal, rig.starts = [], []
    for slot in [s for deck in kn.DECK_SLOTS.values() for s in deck]:
        rig.ns[slot] = LivePlayer(rig.clock, slot, rig.journal)
    rig.ns.update({synth: (lambda *a, _s=synth, **k: _s) for synth in V2_SYNTHS})
    rig.ns["play"] = lambda *a, **k: "play"
    rig.ns["var"] = lambda *a, **k: rig.starts.append((rig.clock.beat, k.get("start")))
    return rig


def _clock_wait(rig):
    """``TrackEditor`` ждёт такта правки: на симуляторе — крутить клок, пока событие не встанет."""

    def wait(done, _seconds):
        for _ in range(64):
            if done.is_set():
                return True
            rig.clock.run_until(rig.clock.beat + 0.5)
        return done.is_set()

    return wait


def _tools(rig):
    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=lambda theme: ThemeHits(), seed=lambda: 4242)
    req = RequestMusicTool(
        None,
        rig.owner,
        dj,
        melodies=lambda ids: {},
        seed=lambda: 777,
        records=lambda ids: {},
        words=lambda word: ThemeHits(),
    )
    req.editor = TrackEditor(rig.owner, rig.adapter, wait=_clock_wait(rig))
    req.backing = BackingDeck(
        rig.owner,
        rig.adapter,
        lambda mood: req._stage_club("", mood),
        _confirm(rig),
        lambda: music_busy(rig.owner, dj),
        log=rig.log,
    )
    return req, EditMusicTool(None, req)


def _no_new_order(*_args, **_kwargs):
    raise AssertionError("правка заказала новый трек (request_music/generate_music)")


def _events(rig, kind):
    return [e for e in rig.events if e["event"] == kind]


# --- кейс Шифу: бит выступления правится на такте, выступление держит сетку -----------------------------------------


def test_shifu_case_heavier_beat_under_the_rap_is_an_edit_on_the_bar_not_a_new_track():
    rig = _live_rig()
    req, tool = _tools(rig)
    backing = req.execute(intent="backing", text="", mood="groove", min_seconds=60.0).data
    track_id, start, end = backing["track_id"], backing["start_beat"], backing["end_beat"]
    rig.clock.run_until(start + 6.3)  # рэп идёт: середина второго такта
    req.order = _no_new_order  # ни request_music, ни generate_music: заказа нового нет
    said_at = rig.clock.beat

    result = tool.execute(heard_text=SHIFU)

    assert result.success, result
    data = result.data
    assert data["changes"] == ["heavier"] and data["backing"] is True and data["heard"] is True
    assert data["message"] == "Сделал звук тяжелее." and data["bpm"] == backing["bpm"]
    # трек тот же: нового started нет, сетка (bpm, доля старта) — прежняя
    assert [e["track_id"] for e in _events(rig, "started")] == [track_id]
    assert not _events(rig, "rejected")
    assert rig.owner.now_playing()["track_id"] == track_id
    # правка встала на границе такта формы, после просьбы, в те же слоты деки
    edit = [(slot, beat, synth) for slot, beat, synth in rig.journal if beat > start]
    beats = {beat for _slot, beat, _synth in edit}
    assert len(beats) == 1
    at = beats.pop() + RETUNE_LEAD_BEATS
    assert at > said_at and (at - start) % BEATS_PER_BAR == 0
    assert {slot for slot, _b, _s in edit} <= set(kn.DECK_SLOTS[backing.get("deck", "A")])
    # секции считаются от доли старта играющей формы, а не заново с такта правки
    assert {s for beat, s in rig.starts if beat > start} == {start}
    # тяжелее: тональные роли — из семьи hard клуба
    hard = {synth for synths in kn.STYLES["club"].timbres["hard"].values() for synth in synths}
    tonal = {synth for _slot, _b, synth in edit if synth not in ("play", None) and synth in V2_SYNTHS}
    assert tonal and tonal <= hard | {"loop"}
    # конец подложки (проходы формы по длине речи) не сдвинулся: выступление и бит кончаются как договорились
    rig.clock.run_until(end + 0.01)
    assert [e["track_id"] for e in _events(rig, "finished")] == [track_id]


def test_faster_on_our_synth_track_changes_the_tempo_on_the_bar_and_says_it():
    rig = _live_rig()
    req, tool = _tools(rig)
    assert req.execute(intent="track", text="поставь трек").success
    rig.clock.run_until(rig.clock.beat + 9)
    (started,) = _events(rig, "started")
    old = started["bpm"]
    req.order = _no_new_order

    result = tool.execute(heard_text="Робот, ускорь темп")

    assert result.success, result
    new = round(old * kn.EDIT_TEMPO_STEP)
    assert result.data["changes"] == ["faster"] and result.data["bpm"] == new
    assert result.data["message"] == f"Ускорил до {new} BPM." and result.data["heard"] is False
    assert ("tempo", float(new)) in rig.clock.calls
    assert rig.owner.now_playing()["bpm"] == new and len(_events(rig, "started")) == 1


def test_slot_counts_only_when_the_words_name_nothing():
    rig = _live_rig()
    req, tool = _tools(rig)
    assert req.execute(intent="track", text="поставь трек").success
    rig.clock.run_until(rig.clock.beat + 9)
    assert tool.execute(change="darker", heard_text="Робот, как в киберпанке").data["changes"] == ["darker"]
    # слова человека решают: слот LLM («быстрее») при «помрачнее» в реплике не берётся
    words = tool.execute(change="faster", heard_text="Робот, сделай атмосферу помрачнее").data
    assert words["changes"] == [] and words["limited"] == ["darker"]  # уже мрачно — честный предел, не темп


# --- не свой синт-трек: честный отказ кодом, без генерации ----------------------------------------------------------


class Owner:
    """Снимок плеера без своего синт-трека; правка звука или фактов здесь — ошибка теста."""

    def __init__(self, **snap):
        self.snap = snap

    def now_playing(self):
        return dict(self.snap)

    def position(self, *args, **kwargs):
        raise AssertionError("правка полезла в деку, которую так не меняют")

    update_dj = position


class Deck:
    """Клок и деки Renardo: любая правка звука — ошибка теста."""

    def check(self, *args, **kwargs):
        raise AssertionError("правка тронула звук, который так не меняется")

    retune = check


@pytest.mark.parametrize(
    "snap,kind",
    [
        ({"playing": True, "track_id": "gen:42", "source": "generated"}, "file"),  # трек кэша/генерации
        ({"playing": True, "track_id": "classic:x", "source": "piece"}, "piece"),
        ({"playing": True, "track_id": "s7:03:B", "source": "synth", "enabled": True}, "set"),
        ({"playing": False}, "none"),
    ],
)
def test_speed_up_of_a_cached_track_is_an_honest_refusal_with_an_offer(snap, kind):
    editor = TrackEditor(Owner(**snap), Deck())
    tool = EditMusicTool(None, SimpleNamespace(editor=editor))
    result = tool.execute(heard_text="Робот, ускорь")
    assert not result.success
    assert result.data["message"] == kn.EDIT_REFUSALS[kind] and result.data["reason"] == f"edit_{kind}"
    assert result.error == kn.EDIT_REFUSALS[kind]


def test_edit_music_is_one_narrow_tool_of_the_catalog():
    entry = get_tool("edit_music")
    assert entry.parameters["properties"]["change"]["enum"] == list(kn.TRACK_EDITS)
    assert entry.parameters["required"] == [] and not entry.starts_music
    assert TURN_CONTEXT_ARGS["edit_music"] == ("turn_id", "heard_text")
    assert dkn.TOOL_CLASSES["edit_music"] == "action.media"


# --- путь хода: хост решает до модели, форма закрывает заказ нового -------------------------------------------------


def _host():
    lines = []
    return SingRouteHost(DeterministicChooser(NoModel()), lambda: [], lines.append), lines


@pytest.mark.parametrize(
    "text,route",
    [
        (SHIFU, "edit:heavier"),
        ("Робот, сделай атмосферу помрачнее, как в киберпанке", "edit:darker"),  # n505
        ("Робот, продолжай развивать эту тему и добавь баса", "edit:develop+more_bass"),  # n504
        ("Робот, ускорь темп", "edit:faster"),  # n506
        ("Робот, читай быстрее", ""),  # речь робота названа: скорость голоса, не бит
        ("Робот, поставь что-нибудь помрачнее", ""),  # заказ другого трека
    ],
)
def test_host_decides_the_edit_route_under_playing_music(text, route):
    host, lines = _host()
    assert host(text, (), True) == route
    assert host(text, (), False) == ""  # в тишине править нечего
    assert bool(lines) is bool(route)


def test_singing_over_the_beat_stays_singing():
    host, _lines = _host()
    assert host("Робот, зачитай рэп под этот бит", (), True).startswith("live:")


def _form_under_music(text):
    host, _lines = _host()
    ports = Ports(Answer(Command(speech="…")))
    session = Session("personality")
    session.set_music(MusicStateRef("playing", track_id="req00777:03:A:abc", bpm=136.0, title="клубный трек · 136"))
    engine = TurnEngine(session, ports, system_prompt="Ты робот.", sing=host)
    asyncio.run(engine.run(Addressed(text, VOICE)))
    return ports.forms[-1], ports.asked[-1]


def test_shifu_case_turn_form_closes_new_music_and_requires_the_edit():
    form, tools = _form_under_music(SHIFU)
    assert form.sing == "edit:heavier"
    assert "edit_music" in tools and not {"request_music", "generate_music", "dj_set"} & set(tools)
    track = parse(
        {
            "actions": [
                {
                    "tool": "request_music",
                    "args": {"intent": "track", "text": "металл", "genre": "rock", "mood": "dark"},
                }
            ]
        }
    )
    assert validate(track, tools, form=form).reason == dkn.VIOLATION_SING_ROUTE  # живой лог 15:16:56 — закрыт
    gen = parse({"actions": [{"tool": "generate_music", "args": {"kind": "instrumental", "style": "metal"}}]})
    assert validate(gen, tools, form=form).reason == dkn.VIOLATION_SING_ROUTE
    talk = parse({"speech": "Сейчас сделаю потяжелее!"})
    assert validate(talk, tools, form=form).reason == dkn.VIOLATION_SING_ROUTE  # слова без правки — не исход
    edit = validate(parse({"actions": [{"tool": "edit_music", "args": {"change": "heavier"}}]}), tools, form=form)
    assert isinstance(edit, Command) and edit.actions[0].tool == "edit_music"


def test_n505_darker_turn_is_an_edit_and_n502_order_in_silence_is_untouched():
    form, tools = _form_under_music("Робот, сделай атмосферу помрачнее, как в киберпанке")
    assert form.sing == "edit:darker" and "edit_music" in tools
    form, tools = _form_under_music("Робот, поставь что-нибудь помрачнее")
    assert form.sing == "" and "request_music" in tools
