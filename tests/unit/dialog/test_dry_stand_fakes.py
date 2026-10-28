"""Фейки сухого стенда судят шаги марафона как живой робот (карточка #4163, этап 1: сверка A с марафоном 28.10).

Расхождения сверки, которые шли от стенда, а не от модели:

* музыка (n312, n511, n602, n708, n709, n712): фейковый плеер не писал строк владельца ``🎵 [music v2] started`` /
  ``stop track_id=… reason=user_stop``, по которым шаг судит харнесс; поиск в библиотеке включал музыку;
* память акта 10 (n1001, n1003, n1004, n1008, n1009): акты 2b/2c стирали каст и память акта 2 (живой марафон стирает
  e2e-БД один раз, актом ``first``, #3972), и в акте 10 голос Саши был «unknown»; раздела ``speaker_memory`` хоста
  (#3919) у стенда не было вовсе.

Шаги - НАСТОЯЩИЕ шаги марафона (``dry_marathon.load_act``), модель - фейк за настоящим мостом команд.
"""

from __future__ import annotations

import asyncio
import json
import re
import sys
from pathlib import Path

import pytest

pytest.importorskip("openai", reason="сухому стенду нужен настоящий мост команд, а ему - клиент openai")

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "dialog"))

import dry_marathon as dm  # noqa: E402
import dry_stand  # noqa: E402
from rob_box_dialog import phrases  # noqa: E402
from rob_box_llm.provider import LLMResponse, ProviderCapabilities, ToolCall  # noqa: E402
from rob_box_llm.providers.fake import FakeLLMProvider  # noqa: E402
from rob_box_voice.core import command_bridge  # noqa: E402
from rob_box_voice.core.dialog_v2_ports import CallCounter, Counted  # noqa: E402

#: Паттерн шагов стопа марафона (``gen_night_marathon``): стоп с НАСТОЯЩИМ id трека и причиной человека.
STOP_PATTERN = re.compile(r"🎵 \[music v2\] stop track_id=[a-z0-9]\S* reason=user_stop")
STARTED_PATTERN = re.compile(r"🎵 \[music v2\] started track_id=")


def marathon_step(act: str, label: str) -> dict:
    return next(s for s in dm.load_act(act).steps if s["label"] == label)


def user_text(msgs) -> str:
    return next((str(m.content) for m in reversed(msgs) if m.role == "user"), "")


def query_data(msgs) -> dict:
    try:
        return json.loads(msgs[-1].content)["query_results"][0]["data"]
    except (ValueError, KeyError, TypeError):
        return {}


class Env:
    """Стенд с фейковой моделью за мостом команд; ``rule(env, msgs, tools)`` - ответ модели (аргументы ``command``)."""

    def __init__(self, rule, tmp_path: Path) -> None:
        self.counter = CallCounter()
        self.seen = []

        async def on_complete(msgs, tools, settings):
            self.seen.append(list(msgs))
            return LLMResponse(content="", finish_reason="tool_calls", tool_calls=(
                ToolCall("c1", "command", rule(self, list(msgs), [t["function"]["name"] for t in tools])),))

        inner = FakeLLMProvider(on_complete=on_complete, name="deepseek",
                                capabilities=ProviderCapabilities(tools=True, tool_choice=True))
        relay = dm.LogRelay()
        llm = command_bridge.wrapper_for("deepseek", log=relay)(Counted(inner, self.counter))
        self.stand = dry_stand.DryStand(llm, self.counter, "Ты робот.", tmp_path)
        relay.target = self.stand

    def run(self, step: dict):
        return asyncio.run(dm.run_step(self.stand, "t", step, "", 300, 0))[0]

    def act(self, act: dm.Act, phase: str = dm.NO_PHASE):
        return asyncio.run(dm.run_act(self.stand, act, 300, 0, phase=phase))


@pytest.fixture
def make_env(tmp_path):
    envs = []

    def make(rule):
        env = Env(rule, tmp_path / f"env{len(envs)}")
        envs.append(env)
        return env

    yield make
    for env in envs:
        env.stand.close()


def call(tool: str, **args) -> dict:
    return {"actions": [{"tool": tool, "args": args}]}


def say(text: str) -> dict:
    return {"speech": text}


# ── Формат строк: те же, что пишет живой владелец ─────────────────────────────────────────────────────────────────


def test_music_owner_lines_are_the_formats_of_the_real_player_and_set():
    """Строки фейкового плеера сверены с источниками: поменяется формат у владельца - тест красный."""
    pairs = [
        ("rob_box_mcp_tools/rob_box_mcp_tools/engine/player_owner.py", '🎵 [music v2] started track_id={track_id} "'),
        ("rob_box_mcp_tools/rob_box_mcp_tools/engine/player_owner.py",
         "🎵 [music v2] stop track_id={last} reason={reason}"),
        ("rob_box_mcp_tools/rob_box_mcp_tools/engine/session.py", "🎧 [set v2] {self.set_id} stop reason={reason}"),
        ("rob_box_mcp_tools/rob_box_mcp_tools/tools/music.py", 'stop_deck("user_stop")'),
        ("rob_box_mcp_tools/rob_box_mcp_tools/engine/gen/library_play.py", 'f"gen_{track_id[:8]}_{int(clock())}"'),
        ("rob_box_mcp_tools/rob_box_mcp_tools/engine/tools_v2.py", 'set_id=f"req{seed % 100000:05d}"'),
    ]
    for rel, fragment in pairs:
        assert fragment in (REPO / "src" / rel).read_text(encoding="utf-8"), f"{rel}: формат изменился: {fragment}"


def test_player_lines_have_the_live_shape(make_env):
    env = make_env(lambda env, msgs, tools: say("Ок."))
    env.stand.emit("-")
    track = env.stand.player.request("ночной бит")
    env.stand.player.stop("user_stop")
    lines = env.stand._log.lines  # noqa: SLF001
    assert re.fullmatch(r"req\d{5}:01:A", track)
    assert any(line.startswith(f"🎵 [music v2] started track_id={track} ") and "phase_in_form=0" in line
               for line in lines), lines
    assert f"🎵 [music v2] stop track_id={track} reason=user_stop" in lines


# ── Стоп музыки: n312 / n511 / n709 / n712 ───────────────────────────────────────────────────────────────────────


def stop_rule(env, msgs, tools):
    return call("stop_music") if "stop_music" in tools else say("Хорошо.")


def test_n312_stop_of_playing_music_passes_by_the_live_patterns(make_env):
    """Живой n312 (28.10): PASS по строке ``stop track_id=<id> reason=user_stop``. Стенд пишет её так же."""
    env = make_env(stop_rule)
    track = env.stand.player.request("ночной бит")
    report = env.run(marathon_step("3", "n312_stop_music"))
    assert report.verdict == dm.PASS, report.fail
    assert f"🎵 [music v2] stop track_id={track} reason=user_stop" in report.log
    assert report.said == [phrases.text("music.stop_ok")] and not env.stand.player.playing


def test_stop_without_playing_track_writes_no_track_line_and_says_idle(make_env):
    """Стоп в тишине: строки с id трека нет (у живого ``PlayerOwner.stop`` пишет ``track_id=None``), шаг стопа
    красный по паттерну, фраза - «ничего не играет» по ``was_playing=False``."""
    env = make_env(stop_rule)
    report = env.run(marathon_step("3", "n312_stop_music"))
    assert not STOP_PATTERN.search(report.log), report.log
    assert "🎵 [music v2] stop track_id=None reason=user_stop" in report.log
    assert report.verdict == dm.FAIL and any("паттерны шага не найдены" in f for f in report.fail)
    assert report.said == [phrases.text("music.stop_idle")]


def test_tier1_stop_all_writes_the_stop_line_of_the_playing_track(make_env):
    """n511/n709: стоп Tier-1 кодом (``stop_all``) - тот же тул ``stop_music``, та же строка владельца."""
    env = make_env(lambda env, msgs, tools: say("Понял."))
    track = env.stand.player.request("бит")
    report = env.run(marathon_step("5", "n511_stop_music"))
    assert "tier1 kind=stop_all" in report.log
    assert f"🎵 [music v2] stop track_id={track} reason=user_stop" in report.log, report.log
    assert report.verdict == dm.PASS, report.fail


def test_stop_music_closes_the_set_with_its_line_then_the_deck(make_env):
    env = make_env(stop_rule)
    started = env.stand.player.start_set("дискотека")
    env.stand.emit("-")
    ok, data = env.stand.tools._t_stop_music({})  # noqa: SLF001
    lines = env.stand._log.lines  # noqa: SLF001
    assert ok and data == {"was_playing": True}
    assert lines[-2:] == [f"🎧 [set v2] {started['set_id']} stop reason=user_stop",
                          f"🎵 [music v2] stop track_id={started['track_id']} reason=user_stop"]


# ── Старт музыки: n602 / n708 ─────────────────────────────────────────────────────────────────────────────────────


def test_n602_dj_set_start_writes_started_and_dj_is_on_in_the_snapshot(make_env):
    """Живой n602: PASS на первой попытке (``dj_set`` + ``started``). У стенда раньше не было ``started`` - повтор шага
    шёл уже при «сете», и модель отвечала «сет уже включён»."""
    env = make_env(lambda env, msgs, tools: call("dj_set", action="start", theme="ночная дискотека")
                   if "dj_set" in tools else say("Хорошо."))
    report = env.run(marathon_step("6", "n602_dj_on"))
    assert report.verdict == dm.PASS and report.attempts == 1, report.fail
    assert re.search(r"🎵 \[music v2\] started track_id=set\d{5}:01:A ", report.log)
    assert env.stand.music_mem.section()["dj"] == "on"


def test_n608_stop_of_the_set_writes_the_set_line_and_the_stop_line_of_its_track(make_env):
    """«Хватит диджеить, выключай»: стоп Tier-1 закрывает сет (строка сета, затем стоп деки с id трека сета)."""
    env = make_env(lambda env, msgs, tools: say("Ок."))
    started = env.stand.player.start_set("дискотека")
    report = env.run(marathon_step("6", "n608_dj_off"))
    assert report.verdict == dm.PASS, report.fail
    assert f"🎧 [set v2] {started['set_id']} stop reason=user_stop" in report.log
    assert f"🎵 [music v2] stop track_id={started['track_id']} reason=user_stop" in report.log
    assert env.stand.music_mem.section()["dj"] == "off"


def test_dj_set_stop_reports_was_playing_like_the_tool(make_env):
    """``DjSetTool._stop``: сет идёт - ``was_playing=True``; сета нет - стоп деки, ``was_playing`` по её треку (фраза
    ``knowledge.DJ_STOP``: «Выключил» или «ничего не играет»)."""
    env = make_env(lambda env, msgs, tools: say("Ок."))
    tools = env.stand.tools
    env.stand.player.start_set("дискотека")
    assert tools._t_dj_set({"action": "stop"})[1]["was_playing"] is True  # noqa: SLF001
    ok, idle = tools._t_dj_set({"action": "stop"})  # noqa: SLF001
    assert ok and idle["was_playing"] is False and idle["track_id"] is None


def gen_play_rule(env, msgs, tools):
    """Модель n708: поиск по библиотеке, затем ``play`` найденного id (данные ``query`` - из ответа тула)."""
    if msgs[-1].role == "tool":
        found = query_data(msgs).get("tracks") or []
        return call("music_library", action="play", track_id=found[0]["id"]) if found else say("Не нашёл.")
    return call("music_library", action="search", query="енот") if "music_library" in tools else say("Ок.")


def test_n708_library_play_writes_started_after_search(make_env):
    env = make_env(gen_play_rule)
    song = env.stand.library.add("Песня про енота-механика", kind="song")
    report = env.run(marathon_step("7", "n708_gen_play"))
    assert report.verdict == dm.PASS and report.attempts == 1, report.fail
    assert report.tools == ["music_library", "music_library"]
    assert re.search(rf"🎵 \[music v2\] started track_id=gen_{song['id'][:8]}_\d+ ", report.log)


def test_library_list_search_info_do_not_start_music(make_env):
    """Старый фейк включал музыку на ЛЮБОЕ действие библиотеки: «что лежит в библиотеке» (n704) оставляло трек
    играть, и состояние «играет» протекало в следующие шаги. Играет только ``play``."""
    env = make_env(lambda env, msgs, tools: say("Ок."))
    tools = env.stand.tools
    env.stand.emit("-")
    ok_list, listed = tools._t_music_library({"action": "list"})  # noqa: SLF001
    ok_search, found = tools._t_music_library({"action": "search", "query": "дождь"})  # noqa: SLF001
    ok_info, info = tools._t_music_library({"action": "info", "track_id": found["tracks"][0]["id"]})  # noqa: SLF001
    assert ok_list and ok_search and ok_info
    assert listed["total"] == len(dry_stand.LIBRARY_SEED) and found["total"] == 1 and info["title"]
    assert not env.stand.player.playing and not any(STARTED_PATTERN.search(x) for x in env.stand._log.lines)  # noqa: SLF001
    ok_unknown, refused = tools._t_music_library({"action": "info", "track_id": "no-such-id"})  # noqa: SLF001
    assert not ok_unknown and refused["reason"] == "unknown_track"


# ── Предусловие акта: музыка не переживает акт (#3871, #4027) ─────────────────────────────────────────────────────


def test_act_start_stops_music_like_the_harness_and_keeps_the_step_log_clean(make_env):
    env = make_env(lambda env, msgs, tools: say("Тихо."))
    env.stand.player.start_set("дискотека")
    steps = [{"label": "n601", "voice": "anton", "text": "Робот, сейчас тихо?"}]
    act = dm.Act("6", "За пультом", Path("act6.json"), steps, True, False, False)
    assert dm.isolate_act_start(env.stand, act, dm.MIDDLE) == "stopped"
    assert not env.stand.player.busy() and env.stand.music_mem.section()["playing"] == "no"
    report = env.act(act, dm.MIDDLE)[0]
    assert "[music v2] stop" not in report.log and "[set v2]" not in report.log
    assert dm.isolate_act_start(env.stand, act, dm.MIDDLE) == "was_idle"


# ── Каст и память между актами марафона (#3972) ───────────────────────────────────────────────────────────────────


def test_marathon_phases_follow_the_runner():
    acts = [dm.load_act(k) for k in ("1", "2", "2b", "2c", "3", "10")]
    assert dm.marathon_phases(acts) == ["first", "middle", "middle", "middle", "middle", "last"]
    assert dm.marathon_phases(acts[:1]) == [dm.NO_PHASE]
    assert dm.marathon_phases(acts[4:]) == [dm.NO_PHASE, dm.NO_PHASE]  # план с наследующего акта - без фазы


def acquaintance_rule(env, msgs, tools):
    text = user_text(msgs)
    if msgs[-1].role == "tool":
        return say("Готово.")
    if "меня зовут Саша" in text and "register_speaker" in tools:
        return call("register_speaker", name="Саша")
    if "запомни" in text and "memory_save" in tools:
        return call("memory_save", fact="Саша пьёт только зелёный чай", category="preference")
    return say("Слушаю.")


def three_acts():
    def act(key, lines, inherits, registers, memory):
        steps = [{"label": f"n_{key}_{i}", "voice": v, "text": t} for i, (v, t) in enumerate(lines)]
        return dm.Act(key, key, Path(f"act{key}.json"), steps, inherits, registers, memory)

    return [act("2", [("anton", "Робот, привет, меня зовут Саша."), ("anton", "Робот, запомни: я пью зелёный чай.")],
                False, True, True),
            act("2b", [("ermil", "Робот, привет, меня зовут Борис.")], False, True, False),
            act("10", [("anton", "Робот, как меня зовут?")], True, False, False)]


def test_voice_registered_in_an_earlier_act_is_recognized_with_its_facts_in_act_10(make_env):
    """Живой марафон 28.10: e2e-БД стёрты один раз (акт 1, ``first``), каст {'Саша', 'Борис'} дожил до акта 10, и
    голос Саши там узнан. Стенд: акт 2b (``register_speaker`` в сценарии) больше не стирает Сашу акта 2, а раздел
    ``speaker_memory`` хода несёт его факт - как у ``dialog_node`` (#3919)."""
    env = make_env(acquaintance_rule)
    acts = three_acts()
    reports = [env.act(a, p) for a, p in zip(acts, dm.marathon_phases(acts))]
    final = reports[-1][0]
    assert "👤 Speaker: 'Саша' confidence=0.880" in final.log, final.log
    assert re.search(r"🧠 \[speaker-memory\] person=\w+ dialog_count=\d+ facts=1 e2e=True", final.log), final.log
    snapshot = user_text(env.seen[-1])
    assert "Саша" in snapshot and "зелёный чай" in snapshot


def test_regression_wiping_the_cast_per_act_loses_the_voice_by_act_10(make_env):
    """Старое поведение (акт с ``register_speaker`` стирает каст сам, без фазы марафона) - Саша в акте 10 «unknown»:
    ровно расхождение n1001/n1003/n1004/n1008/n1009 сверки A."""
    env = make_env(acquaintance_rule)
    reports = [env.act(a, dm.NO_PHASE) for a in three_acts()]
    final = reports[-1][0]
    assert "👤 Speaker: unknown" in final.log and "Саша" not in user_text(env.seen[-1])
