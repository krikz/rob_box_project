"""Ранний звук генерации: ``stream_audio_url`` до ``success`` (ADR-0158, ``engine/gen/stream.py`` + ``pipeline``).

Боевой ``KieSunoProvider`` на фейковом HTTP-транспорте (без сети и без денег): стрим полный / обрезанный / 403 /
нет поля. Декод ``ffmpeg`` подменён фейком, который пишет wav той длины, что зашита в «mp3» (``SEC:<секунды>``);
сам декод — отдельным тестом на настоящем ``ffmpeg`` (если он есть).
"""

from __future__ import annotations

import json
import shutil
import subprocess
import wave
from pathlib import Path

import pytest

from rob_box_mcp_tools.engine.gen import kie_suno, stream
from rob_box_mcp_tools.engine.gen.audio import SERVER_WAV
from rob_box_mcp_tools.engine.gen.budget import Budget
from rob_box_mcp_tools.engine.gen.library import GeneratedMusicLibrary
from rob_box_mcp_tools.engine.gen.pipeline import generate_into_cache
from rob_box_mcp_tools.engine.gen.provider import GenJob, GenTrack, GenUnavailable

JOB = GenJob(kind="instrumental", style_key="grunge", title="RoBBox grunge", style_tags="90s grunge",
             duration_s=120, model="V6")
STREAM0 = "https://audiostream.kie.ai/s/0.mp3"
AUDIO0 = "https://tempfile.example/0.mp3"


def _items(stage: str, duration=None):
    out = []
    for i in range(2):
        item = {"id": f"trk{i}", "title": "Grunge", "tags": "grunge",
                "stream_audio_url": f"https://audiostream.kie.ai/s/{i}.mp3"}
        if stage in ("first", "success") and (i == 0 or stage == "success"):
            item["audio_url"] = f"https://tempfile.example/{i}.mp3"
        if stage == "success" or duration is not None:
            item["duration"] = duration if duration is not None else 119.4
        out.append(item)
    return out


def _record(stage: str, *, no_stream: bool = False, duration=None):
    state = "success" if stage == "success" else "generating"
    items = [] if stage == "queued" else _items(stage, duration)
    if no_stream:
        items = [{k: v for k, v in it.items() if k != "stream_audio_url"} for it in items]
        if stage == "text":
            items = []
    data = {"taskId": "T1", "state": state, "resultJson": json.dumps({"data": items}) if items else ""}
    if stage == "success":
        data["creditsConsumed"] = 12
    return 200, {"code": 200, "data": data}


class Transport:
    """recordInfo — по очереди стадий (последняя повторяется); медиа — по URL; запоминает все GET медиа."""

    def __init__(self, stages, media):
        self.stages, self.media, self.calls, self.timeouts = list(stages), dict(media), [], {}

    def __call__(self, method, url, headers, body, timeout):
        self.calls.append(url)
        if "createTask" in url:
            return 200, json.dumps({"code": 200, "data": {"taskId": "T1"}}).encode()
        if "recordInfo" in url:
            stage = self.stages.pop(0) if len(self.stages) > 1 else self.stages[0]
            status, payload = stage if isinstance(stage, tuple) else _record(stage)
            return status, json.dumps(payload).encode()
        self.timeouts[url] = timeout
        assert headers.get("User-Agent") == kie_suno.USER_AGENT  # Python-urllib → 403 у audiostream.kie.ai
        return self.media[url]


def fake_server_wav(mp3_path, run=None):
    """«Декод»: wav длиной из ``SEC:<с>`` в начале файла (частота 1 кГц — файл маленький)."""
    seconds = float(Path(mp3_path).read_bytes().split(b"|")[0].split(b":")[1])
    wav = Path(mp3_path).with_name(SERVER_WAV)
    with wave.open(str(wav), "wb") as out:
        out.setnchannels(1)
        out.setsampwidth(2)
        out.setframerate(1000)
        out.writeframes(b"\0\0" * int(seconds * 1000))
    return str(wav)


class Log:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(("info", msg))

    def warning(self, msg):
        self.lines.append(("warning", msg))

    def text(self):
        return "\n".join(m for _, m in self.lines)


@pytest.fixture(autouse=True)
def _fake_decode(monkeypatch):
    monkeypatch.setattr(stream, "server_wav", fake_server_wav)


def _run(tmp_path, stages, media, job=JOB, run_finisher=True):
    lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
    transport = Transport(stages, media)
    provider = kie_suno.KieSunoProvider("sk-test", transport=transport, sleep=lambda s: None)
    log, pending = Log(), []
    res = generate_into_cache(provider, lib, Budget(lib), job, early=True, log=lambda: log, sleep=lambda s: None,
                              spawn=lambda fn: pending.append(fn))
    polls_at_return = sum("recordInfo" in u for u in transport.calls)
    if run_finisher:
        for fn in pending:
            fn()
    return res, lib, transport, log, polls_at_return


MEDIA = {STREAM0: (200, b"SEC:118.0|stream"), AUDIO0: (200, b"SEC:119.4|final"),
         "https://tempfile.example/1.mp3": (200, b"SEC:119.0|v1")}


class TestEarlyStream:
    def test_full_stream_is_delivered_before_success_and_finisher_completes_the_cache(self, tmp_path):
        res, lib, t, log, polls = _run(tmp_path, ["queued", "text", "success"], MEDIA, run_finisher=False)
        assert res.early and polls == 2  # отдан на стадии text, success ещё не опрошен
        assert len(res.saved) == 1 and res.saved[0]["variant"] == 0
        track_dir = Path(res.saved[0]["path"]).parent
        wav, mp3 = track_dir / SERVER_WAV, track_dir / "track.mp3"
        assert wav.exists() and wav.stat().st_mtime >= mp3.stat().st_mtime  # декод второй раз не нужен
        assert mp3.read_bytes() == b"SEC:118.0|stream" and lib.count == 1
        assert t.timeouts[STREAM0] == stream.STREAM_DOWNLOAD_TIMEOUT_S
        assert lib.ledger.spent_since("2000-01-01T00:00:00+00:00") == 12.0  # трата записана сразу (оценкой)

    def test_finisher_adds_second_variant_and_final_file_without_new_track(self, tmp_path):
        res, lib, t, log, _ = _run(tmp_path, ["queued", "text", "success"], MEDIA)
        assert lib.count == 2
        info = lib.get_track_info(res.saved[0]["id"])
        assert info["duration_ms"] == 119400 and Path(info["path"]).read_bytes() == b"SEC:119.4|final"
        assert "финальный audio_url ≠ стрим, файл заменён" in log.text() and "финал кэша: +1 вар." in log.text()

    def test_truncated_stream_falls_back_to_success(self, tmp_path):
        media = {**MEDIA, STREAM0: (200, b"SEC:40.0|half")}
        res, lib, t, log, _ = _run(tmp_path, ["text", "success"], media)
        assert "ранний звук (text) не принят: short: 40.0 с < 117.0 с" in log.text()
        # на success первый вариант — мосту сразу, второй качает финишёр; audio_url 1-го не качается дважды
        assert res.early and "ранний звук на стадии complete" in log.text() and lib.count == 2
        assert t.calls.count(AUDIO0) == 1
        assert Path(lib.get_track_info(res.saved[0]["id"])["path"]).read_bytes() == b"SEC:119.4|final"

    def test_stream_403_falls_back_to_success(self, tmp_path):
        media = {**MEDIA, STREAM0: (403, b"Forbidden")}
        res, lib, t, log, _ = _run(tmp_path, ["text", "success"], media)
        assert res.early and lib.count == 2
        assert "provider_down: скачивание: HTTP 403" in log.text() and "жду success" in log.text()

    def test_no_stream_field_waits_for_success_without_extra_downloads(self, tmp_path):
        stages = [_record("text", no_stream=True), _record("success", no_stream=True)]
        res, lib, t, log, polls = _run(tmp_path, stages, MEDIA, run_finisher=False)
        assert STREAM0 not in t.calls and "не принят" not in log.text()
        assert res.early and polls == 2 and lib.count == 1  # второй вариант ещё не качался
        assert t.calls.count("https://tempfile.example/1.mp3") == 0

    def test_complete_stage_first_variant_short_downloads_both_as_before(self, tmp_path):
        media = {**MEDIA, AUDIO0: (200, b"SEC:30.0|bad")}
        res, lib, t, log, _ = _run(tmp_path, [_record("success", no_stream=True)], media)
        assert not res.early and lib.count == 2 and len(res.saved) == 2
        assert "ранний звук (complete) не принят: short" in log.text() and "качаю оба варианта" in log.text()

    def test_first_stage_uses_audio_url_of_first_variant(self, tmp_path):
        media = {**MEDIA, STREAM0: (200, b"SEC:30.0|half")}
        res, lib, t, log, polls = _run(tmp_path, ["text", "first", "success"], media, run_finisher=False)
        assert res.early and polls == 2
        assert Path(res.saved[0]["path"]).read_bytes() == b"SEC:119.4|final"

    def test_failed_finisher_keeps_early_track_and_says_so(self, tmp_path):
        stages = ["text", (200, {"code": 200, "data": {"taskId": "T1", "state": "fail", "failMsg": "boom"}})]
        res, lib, t, log, _ = _run(tmp_path, stages, MEDIA)
        assert res.early and lib.count == 1
        assert "финал кэша не дописан: rejected: boom" in log.text()

    def test_fact_credits_settle_the_estimate(self, tmp_path):
        stages = ["text", (200, {"code": 200, "data": {**_record("success")[1]["data"], "creditsConsumed": 10}})]
        res, lib, *_ = _run(tmp_path, stages, MEDIA)
        assert res.credits_estimated and lib.ledger.spent_since("2000-01-01T00:00:00+00:00") == 10.0

    def test_early_off_keeps_old_behaviour(self, tmp_path):
        lib = GeneratedMusicLibrary(str(tmp_path / "lib"))
        t = Transport(["text", "success"], MEDIA)
        provider = kie_suno.KieSunoProvider("sk-test", transport=t, sleep=lambda s: None)
        res = generate_into_cache(provider, lib, Budget(lib), JOB, sleep=lambda s: None)
        assert not res.early and lib.count == 2 and STREAM0 not in t.calls


class TestMinimum:
    TRACK = GenTrack(0, "t", "", STREAM0, 0.0)

    def test_response_duration_wins(self):
        assert stream.minimum_seconds(JOB, GenTrack(0, "t", "", STREAM0, 90.0)) == 87.0

    def test_ordered_duration_for_instrumental_and_cover(self):
        assert stream.minimum_seconds(JOB, self.TRACK) == 117.0

    def test_song_length_comes_from_vocal(self):
        job = GenJob(kind="robot_song", style_key="rock", title="s", audio_path="/v.wav")

        def run(cmd, **kw):
            return subprocess.CompletedProcess(cmd, 0, stdout=b"82.0\n")
        assert stream.minimum_seconds(job, self.TRACK, run) == pytest.approx(82.0 * stream.SONG_MIN_FRACTION)

    def test_unknown_song_length_refuses_early(self):
        job = GenJob(kind="robot_song", style_key="rock", title="s", audio_path="")
        with pytest.raises(stream.StreamIncomplete) as e:
            stream.minimum_seconds(job, self.TRACK)
        assert e.value.reason == "unknown_length"


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="нет ffmpeg")
def test_verify_decodes_with_real_ffmpeg(tmp_path, monkeypatch):
    from rob_box_mcp_tools.engine.gen import audio
    monkeypatch.setattr(stream, "server_wav", audio.server_wav)
    src = tmp_path / "src.wav"
    with wave.open(str(src), "wb") as out:
        out.setnchannels(1)
        out.setsampwidth(2)
        out.setframerate(16000)
        out.writeframes(b"\0\1" * 16000 * 5)
    wav, seconds = stream.verify(src.read_bytes(), 4.0)
    assert seconds == pytest.approx(5.0, abs=0.05) and Path(wav).name == SERVER_WAV
    shutil.rmtree(Path(wav).parent)
    with pytest.raises(stream.StreamIncomplete) as e:
        stream.verify(src.read_bytes(), 6.0)
    assert e.value.reason == "short"


def test_download_sends_own_user_agent_and_timeout():
    seen = {}

    def transport(method, url, headers, body, timeout):
        seen.update(ua=headers.get("User-Agent"), timeout=timeout)
        return 200, b"x"
    kie_suno.KieSunoProvider("k", transport=transport).download(STREAM0, timeout_s=7.0)
    assert seen == {"ua": kie_suno.USER_AGENT, "timeout": 7.0}
    with pytest.raises(GenUnavailable):
        kie_suno.KieSunoProvider("k", transport=lambda *a: (403, b"")).download(STREAM0)
