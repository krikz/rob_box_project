"""#4973: инструментал не поёт поданные слова — отказ до заказа, без подмены темы.

Вида заказа KIE, который поёт текст сам, в замерах нет (``kie_suno.SINGS_SUPPLIED_WORDS``:
только ``robot_song``, и то на уже спетом вокале). Если слова уже лежат в аргументах, а путь
всё равно инструментал (LLM позвала ``kind=instrumental``, либо реплика без фразы таблицы),
заказ не уходит: кредиты целы, тема в Suno не подменяется. Путь песни этот отказ не трогает.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.base import MCPToolResult
from rob_box_mcp_tools.engine.engine_tools import SOURCE_DECLINED, GenerateMusicTool, RequestMusicTool
from rob_box_mcp_tools.engine.gen import kie_suno
from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary
from rob_box_mcp_tools.engine.gen.provider import GenJob, GenUnavailable
from rob_box_mcp_tools.engine.gen import song as sg
from rob_box_mcp_tools.engine.instrumental_order import supplied_lyrics_refusal
from rob_box_mcp_tools.engine.search import ThemeHits
from rob_box_mcp_tools.engine.song_order import song_request
from rob_box_music.source import REASON_WORDS

from .test_adr0158_pr7_song import ZH_LYRICS, FakeStudio
from .test_engine_session import _rig
from .test_gen_kie_suno import _provider

pytestmark = pytest.mark.unit

#: Синтетика, не чей-то текст: длины хватает на порог готовых слов.
GIVEN = "ветер тихо дует над рекою\n" * 6
#: Однострочные клаузы той же синтетики: речь после STT переводов строк не имеет (#5222).
SPOKEN = ", ".join(["ветер тихо дует над рекою"] * 6)
SHORT_THEME = "зима, снег, звёзды"


class _Maker:
    """Исполнитель инструментала, который считает заказ, но его не шлёт."""

    def __init__(self):
        self.orders = []

    def ready(self):
        return True

    def provider_ok(self):
        return True

    def balance(self):
        return 900.0

    def busy(self):
        return False

    def ordered_in(self, turn):
        return False

    def launch(self, genre, model, theme, turn, done):
        self.orders.append((genre, model, theme, turn))
        return True


def _instrumental_request(tmp_path):
    """Заказ с живым путём инструментала: генерация подключена, мост — синтез (кэш пуст)."""
    rig = _rig()
    maker = _Maker()
    played = []

    def play(track_id, hold=None):
        played.append(track_id)
        return MCPToolResult(
            success=True, data={"ok": True, "track_id": track_id, "started": True}
        )

    from rob_box_mcp_tools.engine.engine_tools import DjSetTool

    dj_tool = DjSetTool(
        None,
        rig.owner,
        melodies=lambda ids: {},
        finder=lambda theme: ThemeHits(),
        seed=lambda: 1,
    )
    req = RequestMusicTool(
        None,
        rig.owner,
        dj_tool,
        melodies=lambda ids: {},
        seed=lambda: 7,
        records=lambda ids: {},
        words=lambda word: ThemeHits(),
        gen_library=lambda: GeneratedMusicLibrary(str(tmp_path / "empty")),
        play_cached=play,
        songs=FakeStudio(),
        instrumental=maker,
    )
    return req, maker, played


def test_instrumental_with_given_lyrics_declines_before_launch_and_does_not_substitute_theme(
    tmp_path,
):
    """Слова в ``theme``, реплика без фразы пения: путь — инструментал, но заказ не стартует."""
    req, maker, played = _instrumental_request(tmp_path)
    result = GenerateMusicTool(None, req).execute(
        kind="instrumental", theme=GIVEN, heard_text="сделай трек"
    )
    assert not result.success and result.data["reason"] == SOURCE_DECLINED
    assert REASON_WORDS["no_lyrics_path"] in result.data["message"]
    assert maker.orders == [] and not played
    assert "Сочиняю" not in result.data["message"]


def test_spoken_one_line_lyrics_decline_an_instrumental_before_launch(tmp_path):
    """Шесть клауз одной строкой — сказанные вслух слова: отказ до заказа, KIE не вызван (#5222)."""
    assert sg.is_spoken_lyrics(SPOKEN) and not sg.is_given_lyrics(SPOKEN)
    req, maker, played = _instrumental_request(tmp_path)
    result = GenerateMusicTool(None, req).execute(
        kind="instrumental", theme=SPOKEN, heard_text="сгенерируй инструментал")
    assert not result.success and result.data["reason"] == SOURCE_DECLINED
    assert REASON_WORDS["no_lyrics_path"] in result.data["message"]
    assert maker.orders == [] and not played


def test_spoken_lyrics_in_heard_text_alone_decline_too():
    """Слова лежат только в heard_text (args[5]): гард смотрит и его, не только theme."""
    args = ("track", "", None, "rock", "t-heard", SPOKEN)
    assert song_request(args, "rock").lyrics == SPOKEN
    assert REASON_WORDS["no_lyrics_path"] in supplied_lyrics_refusal(args, "rock")


def test_short_theme_still_orders_an_instrumental(tmp_path):
    """Короткая тема — не слова: заказ идёт, как до #5222."""
    assert not sg.is_spoken_lyrics(SHORT_THEME) and not sg.is_given_lyrics(SHORT_THEME)
    req, maker, played = _instrumental_request(tmp_path)
    result = GenerateMusicTool(None, req).execute(
        kind="instrumental", theme=SHORT_THEME, heard_text="сгенерируй инструментал")
    assert result.success and maker.orders and maker.orders[0][2]
    assert REASON_WORDS["no_lyrics_path"] not in str(result.data)
    assert not played


def test_spoken_lyrics_bound_is_chars_and_clauses():
    """Граница порога: 119 знаков — тема, 121 знак при шести клаузах — слова. По одной паре."""
    under = "а" * 119
    over = "ветер дует, снег идёт, город спит, печь трещит, ночь темна, " + "о" * 70
    assert len(under) == 119 and not sg.is_spoken_lyrics(under)
    assert len(over) >= 121 and over.count(",") == 5 and sg.is_spoken_lyrics(over)
    five = "ветер дует, снег идёт, город спит, печь трещит, ночь темна " + "о" * 80
    assert five.count(",") == 4 and len(five) >= 121 and not sg.is_spoken_lyrics(five)


def test_words_after_the_phrase_block_an_instrumental_order_too():
    """Хвост после «этими словами» — те же готовые слова: инструментал их не заказывает."""
    heard = "сделай трек с этими словами " + GIVEN
    args = ("track", "сделай трек", None, "rock", "t1", heard)
    assert song_request(args, "rock").lyrics == GIVEN
    message = supplied_lyrics_refusal(args, "rock")
    assert REASON_WORDS["no_lyrics_path"] in message
    assert (
        supplied_lyrics_refusal(
            ("track", "про осень", None, "rock", "t2", "сделай трек"), "rock"
        )
        == ""
    )


def test_kie_rejects_supplied_lyrics_on_a_kind_that_does_not_sing_them():
    """Клиент не шлёт HTTP, если слова пришли в вид без поля слов. ``robot_song`` вид поёт."""
    assert kie_suno.SINGS_SUPPLIED_WORDS == frozenset({"robot_song"})
    provider, transport = _provider({})
    with pytest.raises(GenUnavailable) as err:
        provider.submit(
            GenJob(kind="instrumental", style_key="rock", title="t", lyrics=GIVEN)
        )
    assert err.value.reason == "rejected" and transport.calls == []
    with pytest.raises(GenUnavailable) as cover:
        provider.submit(
            GenJob(
                kind="cover",
                style_key="rock",
                title="t",
                lyrics=ZH_LYRICS,
                audio_path="x.wav",
            )
        )
    assert cover.value.reason == "rejected" and transport.calls == []


def test_song_path_still_carries_verbatim_lyrics(tmp_path):
    """Отказ инструментала не закрывает путь песни: дословные слова доезжают до студии."""
    from .test_adr0158_pr7_song import _request

    studio = FakeStudio()
    result = _request(
        _rig(), studio, GeneratedMusicLibrary(str(tmp_path / "song")), []
    ).execute(intent="track", text=GIVEN, heard_text="сгенерируй песню своим голосом")
    assert result.success and studio.requests and studio.requests[0].lyrics == GIVEN
