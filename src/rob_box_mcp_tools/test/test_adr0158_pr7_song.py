"""ADR-0158 PR-7 — песня голосом робота: текст от LLM → проверка кода → вокал текущим голосом → Suno → кэш → дека.

Всё на фейках (LLM, TTS-бэкенды, KIE, дека): без сети и денег. G7 (ADR §4): голос песни — тот, что дают
``/voice/tts/provider_state`` и ``VoiceStateStore`` (3 голоса двух провайдеров), пресет пения идёт в TTS и в Suno.
"""

import threading
from dataclasses import replace

import pytest

from rob_box_mcp_tools.base import MCPToolResult
from rob_box_mcp_tools.engine.gen import song as sg
from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary
from rob_box_mcp_tools.engine.gen.provider import GenStatus, GenTicket, GenTrack, GenUnavailable
from rob_box_mcp_tools.engine.search import ThemeHits
from rob_box_mcp_tools.engine.tools_v2 import SOURCE_DECLINED, DjSetTool, RequestMusicTool
from rob_box_mcp_tools.voice_state import VoiceStateStore
from rob_box_music.source import CacheTrack, SourceAsk, SourceContext, source_ask, source_candidates
from rob_box_voice.sing_synth import synth_vocal_to_file
from rob_box_voice.tts_voice_registry import sing_preset_for

from .test_engine_session import _rig

pytestmark = pytest.mark.unit

LYRICS = ("Я робот, у меня есть батарейка,\nМоя дорога — тихая железка.\nЯ еду, где не ходят поезда,\n"
          "И светят мне ночные провода.\n\nПрипев:\nЭй, мир, я рядом, слышишь, это я,\nМоя душа — из стали и огня.\n"
          "Эй, мир, я рядом, слышишь, это я,\nМоя душа — из стали и огня.")
GOOD = {"title": "Я робот", "lyrics": LYRICS}
PHRASE = "Робот, спой песню про батарейку в стиле гранж своим голосом"
VOICES = [("minimax", "Russian_ReliableMan"), ("yandex", "anton"), ("silero", "baya")]


# ── проверка текста ───────────────────────────────────────────────────────────

def test_good_lyrics_pass_and_each_defect_is_named():
    assert sg.check_song("Я робот", LYRICS, "ru") == ""
    assert "40" in sg.check_song("я" * 41, LYRICS, "ru")
    assert "950" in sg.check_song("Я", LYRICS + "\nстрока" * 150, "ru")
    assert "строк" in sg.check_song("Я", "Одна строка про робота и его батарейку", "ru")
    english = "I am a robot with a battery\n" * 8
    assert "«ru»" in sg.check_song("I", english, "ru") and sg.check_song("I", english, "en") == ""


def test_lyrics_are_asked_once_more_with_the_named_defect_then_refused():
    seen = []

    def ask(system, user, schema):
        seen.append((system, user))
        return {"title": "Я", "lyrics": "коротко"}

    with pytest.raises(sg.SongFailed) as err:
        sg.write_lyrics(ask, sg.SongRequest("про батарейку", "grunge"), "ru", sg.rules_text())
    assert err.value.reason == "lyrics" and len(seen) == 2  # ровно один повтор (ADR-0148)
    assert "Прошлый ответ не принят" in seen[1][1] and "Прошлый ответ не принят" not in seen[0][1]
    assert "русский" in seen[0][0] and "{language}" not in seen[0][0]  # язык — из пресета голоса
    assert "гранж" in seen[0][1]


def test_second_try_may_succeed_and_llm_errors_are_not_retried():
    answers = iter([{"title": "x", "lyrics": "мало"}, GOOD])
    assert sg.write_lyrics(lambda *a: next(answers), sg.SongRequest("тема"), "ru", "{language}") == sg.Song(
        "Я робот", LYRICS)
    calls = []

    def boom(*a):
        calls.append(1)
        raise TimeoutError("нет ответа")

    with pytest.raises(sg.SongFailed) as err:
        sg.write_lyrics(boom, sg.SongRequest("тема"), "ru", "{language}")
    assert err.value.reason == "lyrics" and len(calls) == 1


def test_suno_tags_come_from_the_table_not_from_the_llm():
    assert sg.song_tags(sg.SongRequest("т", "grunge"), "deep male vocal") == (
        "grunge", "90s grunge rock, loud-quiet-loud, distorted guitars, deep male vocal")
    assert sg.song_tags(sg.SongRequest("т", "нет-такого"), "")[0] == sg.DEFAULT_GENRE


# ── текущий голос ─────────────────────────────────────────────────────────────

def test_current_voice_is_the_provider_state_plus_voice_store():
    store = VoiceStateStore()
    assert sg.current_voice("yandex", store) == ("yandex", "anton")  # дефолт провайдера
    store.set_voice("alena")
    assert sg.current_voice("yandex", store) == ("yandex", "alena")  # set_voice
    with pytest.raises(sg.SongFailed) as err:
        sg.current_voice(None, store)
    assert err.value.reason == "no_voice"


# ── конвейер на фейках ────────────────────────────────────────────────────────

class FakeSuno:
    """Фейк ``MusicGenProvider``: запоминает заказ, отдаёт два варианта; ``upload`` здесь не нужен (его делает submit)."""

    name = "kie_suno"

    def __init__(self, fail=None):
        self.jobs, self.fail, self.key = [], fail, True

    def configured(self):
        return self.key

    def cost(self, job):
        return 12.0

    def submit(self, job):
        if self.fail:
            raise self.fail
        self.jobs.append(job)
        return GenTicket("task-7", self.name, job, 0.0, 12.0)

    def poll(self, ticket):
        tracks = tuple(GenTrack(i, f"t{i}", f"https://x/{i}.mp3", "", 74.8 + i, "Я робот") for i in (0, 1))
        return GenStatus("complete", tracks, credits=12.0)

    def balance(self):
        return 100.0

    def download(self, url):
        return b"ID3" + url.encode()


class FakeVocal:
    """Бэкенд пения: запоминает пресет, который ему передал ``synth_vocal_to_file`` (G7)."""

    fmt = "wav"

    def __init__(self, provider):
        self.provider, self.presets = provider, []

    def synth(self, text, preset):
        self.presets.append(preset)
        return b"RIFF-fake", 48000, 5.0


@pytest.fixture
def library(tmp_path):
    return GeneratedMusicLibrary(str(tmp_path / "music_library"))


def _studio(library, voice=("minimax", "Russian_ReliableMan"), suno=None, lyrics=None, played=None, said=None):
    suno = suno or FakeSuno()
    backends, sung = {name: FakeVocal(name) for name, _v in VOICES}, []

    def sing(provider, voice_id, text, path):
        sung.append((provider, voice_id, text))
        return synth_vocal_to_file(provider, voice_id, text, path, backends=backends)

    def play(track_id):
        (played if played is not None else []).append(track_id)
        return MCPToolResult(success=True, data={"ok": True})

    studio = sg.SongStudio(suno, lambda: library, lyrics or (lambda *a: GOOD), lambda: voice, play,
                           (said if said is not None else []).append, sing=sing, clock=lambda: 0.0)
    return studio, suno, backends, sung


@pytest.mark.parametrize("provider,voice", VOICES)
def test_g7_the_song_is_sung_by_the_current_voice_with_its_preset(library, provider, voice):
    """G7: 3 голоса двух провайдеров — вокал текущим голосом по пресету; пол/вес/теги в Suno — из того же пресета."""
    studio, suno, backends, sung = _studio(library, (provider, voice))
    result = studio.make(sg.SongRequest("про батарейку", "grunge"))
    preset = sing_preset_for(provider, voice)
    assert [s[:2] for s in sung] == [(provider, voice)] and backends[provider].presets == [preset]
    job = suno.jobs[0]
    assert (job.kind, job.vocal_gender, job.audio_weight) == ("robot_song", preset.vocal_gender, preset.audio_weight)
    assert job.style_tags == f"90s grunge rock, loud-quiet-loud, distorted guitars, {preset.tags_hint}"
    assert job.lyrics == LYRICS and job.audio_path.endswith(".wav")
    row = library.get_track_info(result.track_id)  # кэш: оба варианта, с provider/voice
    assert (row["tts_provider"], row["voice"], row["kind"], row["task_id"]) == (provider, voice, "robot_song", "task-7")
    assert library.count == 2 and (result.title, result.provider, result.voice) == ("Я робот", provider, voice)
    assert result.duration_s == 74.8 and result.credits == 12.0  # из ответа провайдера
    assert library.ledger.spent_since("2000-01-01T00:00:00+00:00") == 12.0


def test_vocal_workdir_is_removed_after_the_order(library, tmp_path):
    studio, suno, _b, _s = _studio(library)
    studio.make(sg.SongRequest("т"))
    import os
    assert not os.path.exists(suno.jobs[0].audio_path)


def test_unknown_voice_and_missing_cache_are_honest_failures(library):
    studio, *_ = _studio(library, ("minimax", "Нет_Такого"))
    with pytest.raises(sg.SongFailed) as err:
        studio.make(sg.SongRequest("т"))
    assert err.value.reason == "no_preset"  # никакого тихого фолбека на другой голос
    nolib = sg.SongStudio(FakeSuno(), lambda: None, lambda *a: GOOD, lambda: VOICES[0], lambda t: None, lambda t: None)
    with pytest.raises(sg.SongFailed) as err:
        nolib.make(sg.SongRequest("т"))
    assert err.value.reason == "no_track"


def test_provider_failure_is_a_typed_reason_and_spends_nothing(library):
    studio, *_ = _studio(library, suno=FakeSuno(GenUnavailable("no_credits", "KIE code=402")))
    with pytest.raises(GenUnavailable) as err:
        studio.make(sg.SongRequest("т"))
    assert err.value.reason == "no_credits" and library.count == 0


# ── фон: итог говорит код ─────────────────────────────────────────────────────

def _run(studio, request=None, failed=lambda why: f"не вышло:{why}"):
    """Запустить фон и дождаться, пока студия скажет итог и освободится."""
    done, inner = threading.Event(), studio._speak
    studio._speak = lambda text: (inner(text), done.set())
    assert studio.launch(request or sg.SongRequest("т", "grunge"), failed)
    assert done.wait(10), "студия не сказала итог"
    for _ in range(500):
        if not studio.busy():
            break
        threading.Event().wait(0.01)


def test_success_plays_the_first_variant_then_code_says_title_duration_voice(library):
    played, said = [], []
    studio, *_ = _studio(library, played=played, said=said)
    _run(studio)
    assert len(played) == 1 and library.get_track_info(played[0])["variant"] == 0
    assert said == ["Песня «Я робот» готова, 1 минута 15 секунд. Пою голосом Надёжный мужчина."]


def test_silero_voice_is_announced_as_the_spare_one(library):
    said = []
    studio, *_ = _studio(library, ("silero", "baya"), said=said)
    _run(studio)
    assert said == ["Песня «Я робот» готова, 1 минута 15 секунд. Пою запасным голосом Бая (женский)."]


def test_failure_is_spoken_by_code_and_dead_provider_is_remembered(library):
    said = []
    clock = [0.0]
    studio, *_ = _studio(library, suno=FakeSuno(GenUnavailable("provider_down", "KIE 455")), said=said)
    studio._clock = lambda: clock[0]
    _run(studio)
    assert said == ["не вышло:provider_down"] and not studio.provider_ok() and not library.count
    clock[0] = sg.DEAD_FOR_S + 1
    assert studio.provider_ok()  # через 10 минут пробуем снова (ADR §3.5)


def test_unplayed_song_is_not_announced_as_playing(library):
    said = []
    studio, *_ = _studio(library, said=said)
    studio._play = lambda track_id: MCPToolResult(success=False, error="нет деки")
    _run(studio)
    assert said == ["не вышло:play_failed"]


def test_second_order_while_busy_is_refused(library):
    gate, started = threading.Event(), threading.Event()

    def slow(*a):
        started.set()
        gate.wait(5)
        return GOOD

    studio, *_ = _studio(library, lyrics=slow)
    assert studio.launch(sg.SongRequest("т"), lambda why: why) and started.wait(5)
    assert studio.busy() and not studio.launch(sg.SongRequest("т"), lambda why: why)
    gate.set()


# ── путь: кандидаты и чойсер ──────────────────────────────────────────────────

def test_generate_candidate_exists_only_for_a_sing_ask_with_a_ready_executor():
    ask = source_ask(PHRASE)
    assert ask.vocal and ask.robot_voice and ask.genre == "grunge"
    ctx = SourceContext(gen_ready=True, day_left=120.0)
    options, reason = source_candidates(ask, ctx)
    assert [o.id for o in options] == ["generate:robot_song:V6"] and reason == ""
    for blocked, why in ((replace(ctx, day_left=0.0), "budget_exhausted"), (replace(ctx, has_key=False), "no_key"),
                         (replace(ctx, provider_ok=False), "provider_down"), (replace(ctx, gen_ready=False), "not_wired")):
        options, reason = source_candidates(ask, blocked)
        assert reason == why and options[0].id.startswith("decline:")


def test_cache_song_in_another_voice_is_not_offered_for_my_voice():
    ask = SourceAsk(vocal=True, robot_voice=True, genre="grunge")
    mine = CacheTrack("a", "Моя", "robot_song", "grunge", False, voice="Russian_ReliableMan", group="1")
    other = CacheTrack("b", "Чужая", "robot_song", "grunge", False, voice="anton", group="2")
    options, _ = source_candidates(ask, SourceContext(cache=(mine, other), voice="Russian_ReliableMan"))
    assert [o.id for o in options] == ["cache:a"]
    options, _ = source_candidates(ask, SourceContext(cache=(mine, other)))  # голос неизвестен — не режем
    assert {o.id for o in options} == {"cache:a", "cache:b"}


# ── заказ целиком: мост, фраза ожидания, провал ───────────────────────────────

class FakeStudio:
    """Студия без потоков: что запросили у ``launch`` и что скажет при провале."""

    def __init__(self, busy=False):
        self.requests, self.failed, self._busy = [], None, busy

    def has_key(self):
        return True

    def provider_ok(self):
        return True

    def voice_id(self):
        return "Russian_ReliableMan"

    def voice(self):
        return "minimax", "Russian_ReliableMan"

    def busy(self):
        return self._busy

    def ordered_in(self, turn):
        return bool(self._busy and turn and turn == "t1")

    def launch(self, request, on_failed, turn=None):
        self.requests.append(request)
        self.failed = on_failed
        return True


@pytest.fixture
def cache(tmp_path):
    lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
    for variant in (0, 1):  # инструментал гранжа — мост на время ожидания
        lib.save_track(b"ID3inst%d" % variant, prompt="grunge", title="Robbox Grunge Test", kind="instrumental",
                       style_key="grunge", genre="grunge", is_instrumental=True, task_id="f7cff936", variant=variant)
    return lib


def _request(rig, studio, library=None, played=None):
    dj = DjSetTool(None, rig.owner, melodies=lambda ids: {}, finder=lambda theme: ThemeHits(), seed=lambda: 4242)

    def play(track_id):
        played.append(track_id)
        return MCPToolResult(success=True, data={"ok": True, "track_id": f"gen_{track_id[:8]}_1", "started": True})

    return RequestMusicTool(None, rig.owner, dj, melodies=lambda ids: {}, seed=lambda: 777, records=lambda ids: {},
                            words=lambda word: ThemeHits(), gen_library=lambda: library,
                            play_cached=play if played is not None else None, songs=studio)


def test_sing_ask_starts_the_song_and_a_cache_bridge_with_a_code_phrase(cache):
    studio, played = FakeStudio(), []
    req = _request(_rig(), studio, cache, played)
    result = req.execute(intent="track", text=PHRASE, heard_text=PHRASE)
    assert result.success and result.data["pending"] == "song" and len(played) == 1  # мост играет
    assert result.data["source_message"].startswith("Пишу песню, пою голосом ")
    assert result.data["source_message"].endswith("минуты полторы. Пока играет своё.")
    assert studio.requests == [sg.SongRequest(PHRASE, "grunge", "")]


@pytest.fixture
def empty(tmp_path):
    return GeneratedMusicLibrary(str(tmp_path / "empty"))


def test_without_a_bridge_the_wait_phrase_does_not_promise_music(empty):
    studio = FakeStudio()
    req = _request(_rig(), studio, empty)
    result = req.execute(intent="track", text=PHRASE, heard_text=PHRASE)
    assert result.success and result.data["pending"] == "song"
    assert "Пока играет своё" not in result.data["source_message"] and len(studio.requests) == 1


def test_failure_phrase_is_built_by_code_from_the_reason(cache, empty):
    studio = FakeStudio()
    bridged = _request(_rig(), studio, cache, [])
    bridged.execute(intent="track", text=PHRASE, heard_text=PHRASE)
    assert studio.failed("no_credits") == ("Песню сделать не вышло: на счёте генерации нет кредитов. "
                                           "Играет пока своё.")
    studio2 = FakeStudio()
    bare = _request(_rig(), studio2, empty)
    bare.execute(intent="track", text=PHRASE, heard_text=PHRASE)
    phrase = studio2.failed("lyrics")  # моста нет: выбор заново без generate → честный отказ с причиной
    assert phrase.startswith("Сгенерировать не вышло: текст не сложился, а синтез не поёт.")


def test_busy_studio_declines_with_a_phrase_and_starts_nothing(cache):
    studio, played = FakeStudio(busy=True), []
    result = _request(_rig(), studio, cache, played).execute(intent="track", text=PHRASE, heard_text=PHRASE)
    assert not result.success and result.data["reason"] == SOURCE_DECLINED and not played and not studio.requests
    assert result.data["message"] == sg.BUSY_TEXT


def test_instrumental_ask_never_reaches_the_song_studio(cache):
    studio, played = FakeStudio(), []
    req = _request(_rig(), studio, cache, played)
    result = req.execute(intent="track", text="сгенерируй гранж", heard_text="Робот, сгенерируй гранж")
    assert studio.requests == [] and result.data.get("pending") is None  # инструментал — PR-8, здесь not_wired


def test_decline_code_is_the_same_on_both_sides():
    from rob_box_mcp_tools.engine.song_order import DECLINED

    assert DECLINED == SOURCE_DECLINED


def test_second_tool_call_of_the_same_turn_is_not_a_new_order(cache):
    """Прогон 13.10: LLM зовёт request_music и generate_music в одном ходе — второй вызов не «занято», а то же ожидание."""
    studio = FakeStudio(busy=True)
    req = _request(_rig(), studio, cache, [])
    same = req.execute(intent="track", text=PHRASE, heard_text=PHRASE, turn_id="t1")
    assert same.success and same.data["pending"] == "song" and not studio.requests
    other = req.execute(intent="track", text=PHRASE, heard_text=PHRASE, turn_id="t2")
    assert not other.success and other.data["message"] == sg.BUSY_TEXT


def test_song_language_minimax_follows_the_robot_not_the_catalog_voice():
    """Прогон 13.10: дефолтный MiniMax-голос числится китайским → песня вышла на китайском. Язык MiniMax — язык робота."""
    assert sing_preset_for("minimax", "male-qn-qingse").language == "zh"
    assert sg.song_language("minimax", "zh") == "ru"
    assert sg.song_language("yandex", "de") == "de" and sg.song_language("silero", "ru") == "ru"


def test_language_proxy_gives_tts_the_song_language():
    backend = FakeVocal("minimax")
    proxy = sg._Language(backend, "ru")
    proxy.synth("текст", sing_preset_for("minimax", "male-qn-qingse"))
    assert backend.presets[0].language == "ru" and (proxy.provider, proxy.fmt) == ("minimax", "wav")


def test_late_started_is_waited_for_not_called_a_failure(library):
    """Прогон 13.10: трек деки встаёт на границу формы (до ~47 с), ``gen_play_from_library`` ждёт 6 с и отвечает
    ``not_started``, а песня потом играет. Ждём ``started`` дольше; пришло — песня «играет», нет — честный провал."""
    from types import SimpleNamespace

    said = []
    studio, *_ = _studio(library, said=said)
    late = MCPToolResult(success=False, data={"reason": "not_started", "track_id": "gen_x_1"}, error="не заиграл")
    studio._play = lambda track_id: late
    studio._started = lambda track_id: SimpleNamespace(event="started") if track_id == "gen_x_1" else None
    _run(studio)
    assert said[0].startswith("Песня «Я робот» готова")
    said.clear()
    studio._started = lambda track_id: None  # started так и не пришло
    _run(studio)
    assert said == ["не вышло:play_failed"]
