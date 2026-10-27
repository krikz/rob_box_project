"""#4090 — хвост вилки пения на пути тула музыки: фраза ближайшего стиля из исхода ``request_music`` и «на
синтезаторе» = «на Ренардо» для инструментала (решение товарища Шифу 27.10 17:07 UTC).

Живая проверка 27.10 18:58 UTC (raw — комментарий #4079): «Робот, сыграй метал на Ренардо.» → ``request_music
{'engine': 'synth', 'genre': 'rock', …}`` → ``[source] … genre=metal engine=synth кандидаты=['synth:breaks'] →
synth:breaks (code)``, а вслух — только «Включаю музыку.»: фраза кода о ближайшем стиле лежала в данных исхода
(``source_note``), диалог v2 её не брал. Здесь путь исхода целиком, как на роботе: настоящий ``RequestMusicTool`` →
результат MCP (``data``) → ``command_turn.owner_event`` → ``Executor`` диалога → ``respond.join`` — фразу строит код
из данных исхода, LLM её не пишет.
"""

import asyncio
import logging
from types import SimpleNamespace

import pytest

from rob_box_dialog import phrases
from rob_box_dialog.command import ToolCall
from rob_box_dialog.execute import Executor
from rob_box_dialog.respond import join
from rob_box_harness.core.tool_loop.command_turn import owner_event
from rob_box_llm.provider import ToolResult as LLMToolResult
from rob_box_mcp_tools.engine import tools_v2
from rob_box_mcp_tools.engine.classic import ClassicPick
from rob_box_mcp_tools.engine.search import ThemeHits
from rob_box_mcp_tools.engine.tools_v2 import DjSetTool, GenerateMusicTool, RequestMusicTool
from rob_box_music.source import LIVE_TOOL_TEXT

from .test_engine_session import _rig

pytestmark = pytest.mark.unit

HEARD_1858 = "Робот, сыграй метал на Ренардо."  # дословно реплика живой проверки 27.10 18:58 UTC
NEAREST = "Метал на синте играю ближайшим стилем — брейкбит."


def _tools(rig):
    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=lambda theme: ThemeHits(), seed=lambda: 4242)
    req = RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, seed=lambda: 777,
                           classic=lambda query, seed: ClassicPick(query=query, found=False, reason="нет в базе"),
                           records=lambda ids: {}, words=lambda word: ThemeHits())
    return dj, req


def _asks(req):
    """Просьбы, с которыми заказ пошёл в выбор пути (по ним видно, какой движок решил код)."""
    seen = []
    real = req._source.route  # noqa: SLF001

    def spy(ask, **kwargs):
        seen.append(ask)
        return real(ask, **kwargs)

    req._source.route = spy  # noqa: SLF001
    return seen


def _spoken(tool, name, args, heard):
    """Ход диалога v2 с одним действием: исполнитель шлёт вызов тулу (как ``dialog_v2_ports._send``), исход тула —
    событие владельца (``owner_event``), фраза — код по результату (``respond.join``). ``(фраза, результат хода)``."""
    calls = []

    async def turn():
        executor = Executor(lambda request: deliver(executor, request))
        results = await executor.run([ToolCall(name, args)], SimpleNamespace(epoch=0, agent="personality"), "t4090")
        return results

    def deliver(executor, request):
        result = tool.execute(**dict(request.args), heard_text=heard)
        calls.append(result)
        executor.on_event(owner_event(request, LLMToolResult(tool_call_id="c1", content=result.error or "",
                                                             is_error=not result.success, data=result.data)))

    results = asyncio.run(turn())
    said = join(results)
    return (said.text if said is not None else ""), results, calls


def test_nearest_style_phrase_of_the_live_check_is_built_from_the_outcome_of_request_music():
    """«Сыграй метал на Ренардо» с аргументами LLM живого прогона (``genre=rock``, ``engine=synth``): синт играет
    брейкбит, и вслух — фраза кода о ближайшем стиле перед фразой запуска, а не голое «Включаю музыку.»."""
    rig = _rig()
    _dj, req = _tools(rig)
    text, results, calls = _spoken(req, "request_music",
                                   {"intent": "track", "text": "Сыграй метал на Ренардо", "genre": "rock",
                                    "engine": "synth"}, HEARD_1858)
    assert calls[0].success and calls[0].data["style"] == "breaks" and calls[0].data["engine"] == "synth"
    assert results[0].status == "done" and results[0].data["source_note"] == NEAREST
    assert text == f"{NEAREST} {phrases.text('media.done')}"
    assert rig.owner.now_playing()["engine"] == "synth"


def test_a_genre_the_synth_plays_says_only_the_launch_phrase():
    """Синт играет просимое («клубный трек на Ренардо») — фразы о замене нет: только фраза запуска."""
    rig = _rig()
    _dj, req = _tools(rig)
    text, results, _calls = _spoken(req, "request_music", {"intent": "track", "text": "Сыграй клубный трек на Ренардо",
                                                           "engine": "synth"}, "Робот, сыграй клубный трек на Ренардо")
    assert results[0].status == "done" and not results[0].data.get("source_note")
    assert text == phrases.text("media.done")


@pytest.mark.parametrize("heard", ["Робот, сгенерируй трек на синтезаторе", "Робот, сгенерируй трек на синте"])
def test_generate_music_on_the_synthesizer_plays_the_synth_for_free(heard, caplog):
    """Решение Шифу 27.10 (#4090 D): «сгенерируй трек на синтезаторе» = «на Ренардо» — играет синт, генерация KIE не
    запускается (путь ``synth:*`` кодом), фраза — как у любого заказа синта."""
    rig = _rig()
    _dj, req = _tools(rig)
    gen = GenerateMusicTool(None, req)
    asks = _asks(req)
    with caplog.at_level(logging.INFO):
        text, results, calls = _spoken(gen, "generate_music", {"kind": "instrumental"}, heard)
    assert asks[0].generated and asks[0].engine == "synth"
    assert calls[0].success and calls[0].data["engine"] == "synth" and calls[0].data["source"] == "synth"
    assert results[0].status == "done" and text == phrases.text("media.done")
    lines = [r.getMessage() for r in caplog.records if "[source]" in r.getMessage()]
    assert lines and all("кандидаты=['synth:club'] → synth:club (code)" in line for line in lines), lines


def test_model_slot_engine_synth_is_kept_when_the_person_named_the_synthesizer(caplog):
    """``settle_engine`` не выбрасывает ``engine=synth`` модели, когда человек назвал синтезатор, даже в просьбе
    «сгенерируй» (раньше — «слот engine=synth без опоры в словах человека — отброшен» и платная генерация)."""
    rig = _rig()
    _dj, req = _tools(rig)
    asks = _asks(req)
    heard = "Робот, сгенерируй метал на синтезаторе"
    with caplog.at_level(logging.INFO):
        text, results, _calls = _spoken(req, "request_music", {"intent": "track", "text": heard, "engine": "synth"},
                                        heard)
    assert asks[0].engine == "synth" and results[0].data["engine"] == "synth"
    assert not any("отброшен" in r.getMessage() for r in caplog.records)
    assert text == f"{NEAREST} {phrases.text('media.done')}"


def test_song_being_written_over_a_synth_bridge_says_the_wait_phrase_not_the_bridge_note(tmp_path):
    """Песня пишется, мост — синтез (кэша нет): в данных исхода и фраза моста «…песня ещё пишется, играю своё.», и
    фраза ожидания. Вслух — фраза ожидания целиком, как в v1 (``media_phrases.launch_text``), без противоречия."""
    from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary

    from .test_adr0158_pr7_song import FakeStudio
    from .test_adr0158_pr7_song import _request as song_request

    heard = "Робот, спой песню про батарейку своим голосом"  # без жанра: мост — синтез клуба
    req = song_request(_rig(), FakeStudio(), GeneratedMusicLibrary(str(tmp_path / "empty")))
    text, results, calls = _spoken(req, "request_music", {"intent": "track", "text": heard}, heard)
    data = calls[0].data
    assert calls[0].success and data["pending"] == "song" and data.get("source_note")  # мост сказал бы своё
    assert results[0].status == "done" and text == data["source_message"]
    assert "Пишу песню" in text and data["source_note"] not in text


def test_two_songs_on_the_synthesizer_are_live_singing_not_a_paid_instrumental():
    """Карточка #4090 C: «сгенерируй две песни на синтезаторе» уходило в платный ``generate:instrumental``. Теперь —
    клетка «синт + пение»: поёт робот сам (путь ``live:song``, тул генерацию не запускает)."""
    rig = _rig()
    _dj, req = _tools(rig)
    gen = GenerateMusicTool(None, req)
    result = gen.execute(kind="instrumental", heard_text="Робот, сгенерируй две песни на синтезаторе")
    assert not result.success and result.data["route"] == "live:song"
    assert result.data["reason"] == tools_v2.SOURCE_DECLINED and LIVE_TOOL_TEXT in (result.error or "")
    assert rig.owner.now_playing().get("playing") is not True
