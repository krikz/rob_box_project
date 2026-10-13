"""Клиент KIE/Suno (ADR-0158 §2.1, §3.5) на фейковом HTTP — без сети и без денег.

Формы ответов взяты из реальных ответов KIE 13.10 (``record.json`` пробы
гранжа): HTTP всегда 200, ошибка — ``code`` в теле, треки — строка
``resultJson``.
"""

from __future__ import annotations

import json

import pytest

from rob_box_mcp_tools.engine.gen import kie_suno
from rob_box_music.knowledge import GEN_MODELS
from rob_box_mcp_tools.engine.gen.provider import (
    REASONS,
    GenJob,
    GenUnavailable,
    poll_interval_s,
    wait_complete,
)

SECRET = "sk-test-SECRET-123"


def _record(state: str, tracks: bool = True, credits: float = 12.0, **extra) -> dict:
    items = [
        {"id": f"trk{i}", "audio_url": f"https://tempfile.example/{i}.mp3",
         "stream_audio_url": f"https://audiostream.kie.ai/stream/{i}.mp3",
         "duration": 60.0, "title": "T", "tags": "grunge"}
        for i in range(2)
    ] if tracks else []
    data = {"taskId": "task1", "state": state, "creditsConsumed": credits, "costTime": 22,
            "resultJson": json.dumps({"code": 200, "data": items}) if tracks else ""}
    data.update(extra)
    return {"code": 200, "msg": "success", "data": data}


class FakeTransport:
    """Очередь ответов по подстроке URL; запоминает все запросы."""

    def __init__(self, routes=None):
        self.routes = routes or {}
        self.calls = []

    def __call__(self, method, url, headers, body, timeout):
        self.calls.append((method, url, dict(headers), body))
        for needle, queue in self.routes.items():
            if needle in url:
                item = queue.pop(0) if isinstance(queue, list) and len(queue) > 1 else (
                    queue[0] if isinstance(queue, list) else queue)
                if isinstance(item, Exception):
                    raise item
                status, payload = item
                raw = payload if isinstance(payload, bytes) else json.dumps(payload).encode()
                return status, raw
        raise AssertionError(f"неожиданный запрос {method} {url}")


def _provider(routes, **kw):
    t = FakeTransport(routes)
    return kie_suno.KieSunoProvider(SECRET, transport=t, sleep=lambda s: t.calls.append(("sleep", s, {}, None)), **kw), t


JOB = GenJob(kind="instrumental", style_key="grunge", title="Robbox Grunge Test",
             style_tags="90s grunge, 116 bpm", duration_s=60, model="V6_MINI")


class TestSubmit:
    def test_instrumental_payload_matches_live_probe(self):
        p, t = _provider({"createTask": (200, {"code": 200, "data": {"taskId": "task1"}})})
        ticket = p.submit(JOB)
        assert ticket.task_id == "task1" and ticket.cost == 12.0 and ticket.provider == "kie_suno"
        method, url, headers, body = t.calls[0]
        assert (method, url) == ("POST", "https://api.kie.ai/api/v1/jobs/createTask")
        assert json.loads(body) == {"model": "ai-music-api/generate", "input": {
            "custom_mode": True, "instrumental": True, "model": "V6_MINI",
            "title": "Robbox Grunge Test", "style": "90s grunge, 116 bpm", "duration": 60}}
        assert headers["Authorization"] == f"Bearer {SECRET}"
        assert headers["User-Agent"] and "urllib" not in headers["User-Agent"].lower()

    def test_no_key_is_typed_error_before_any_request(self, monkeypatch):
        monkeypatch.delenv("KIE_API", raising=False)
        t = FakeTransport()
        p = kie_suno.KieSunoProvider(transport=t)
        with pytest.raises(GenUnavailable) as e:
            p.submit(JOB)
        assert e.value.reason == "no_key" and t.calls == []

    def test_key_from_env(self, monkeypatch):
        monkeypatch.setenv("KIE_API", SECRET)
        t = FakeTransport({"createTask": (200, {"code": 200, "data": {"taskId": "x"}})})
        kie_suno.KieSunoProvider(transport=t).submit(JOB)
        assert t.calls[0][2]["Authorization"] == f"Bearer {SECRET}"

    def test_key_never_in_repr_or_error(self):
        p, _ = _provider({"createTask": (200, {"code": 402, "msg": "Insufficient credits"})})
        assert SECRET not in repr(p)
        with pytest.raises(GenUnavailable) as e:
            p.submit(JOB)
        assert SECRET not in str(e.value)

    @pytest.mark.parametrize("code,reason", [
        (401, "no_key"), (402, "no_credits"), (422, "rejected"), (400, "rejected"),
        (455, "provider_down"), (500, "provider_down"), (503, "provider_down"),
    ])
    def test_error_code_in_http_200_body_maps_to_reason(self, code, reason):
        """У KIE HTTP-статус всегда 200 — смотрим ``code`` в теле."""
        p, _ = _provider({"createTask": (200, {"code": code, "msg": "boom"})})
        with pytest.raises(GenUnavailable) as e:
            p.submit(JOB)
        assert e.value.reason == reason and e.value.reason in REASONS

    def test_non_200_http_status_without_body_code(self):
        p, _ = _provider({"createTask": (502, b"<html>bad gateway</html>")})
        with pytest.raises(GenUnavailable) as e:
            p.submit(JOB)
        assert e.value.reason == "provider_down"

    def test_network_error_is_provider_down(self):
        p, _ = _provider({"createTask": GenUnavailable("provider_down", "сеть: URLError")})
        with pytest.raises(GenUnavailable) as e:
            p.submit(JOB)
        assert e.value.reason == "provider_down"

    def test_429_retries_once_after_10s_then_succeeds(self):
        p, t = _provider({"createTask": [(200, {"code": 429, "msg": "rate"}),
                                         (200, {"code": 200, "data": {"taskId": "t2"}})]})
        assert p.submit(JOB).task_id == "t2"
        assert ("sleep", 10.0, {}, None) in t.calls

    def test_429_twice_gives_rate_limited(self):
        p, _ = _provider({"createTask": [(200, {"code": 429}), (200, {"code": 429})]})
        with pytest.raises(GenUnavailable) as e:
            p.submit(JOB)
        assert e.value.reason == "rate_limited"

    def test_song_kind_not_supported_yet(self):
        p, t = _provider({})
        with pytest.raises(GenUnavailable) as e:
            p.submit(GenJob(kind="song", style_key="x", title="t"))
        assert e.value.reason == "rejected" and t.calls == []

    def test_unknown_model_rejected(self):
        p, _ = _provider({})
        with pytest.raises(GenUnavailable) as e:
            p.submit(GenJob(kind="instrumental", style_key="x", title="t", model="V99"))
        assert e.value.reason == "rejected"

    def test_cover_uploads_then_creates_task(self, tmp_path):
        wav = tmp_path / "ours.wav"
        wav.write_bytes(b"RIFFfake")
        p, t = _provider({
            "file-stream-upload": (200, {"code": 200, "data": {"downloadUrl": "https://tempfile.example/up.wav"}}),
            "createTask": (200, {"code": 200, "data": {"taskId": "cv1"}}),
        })
        job = GenJob(kind="cover", style_key="orchestral", title="Cover", style_tags="orchestra",
                     duration_s=90, audio_path=str(wav), audio_weight=0.65)
        assert p.submit(job).task_id == "cv1"
        up, create = t.calls
        assert up[1] == kie_suno.UPLOAD_URL and b"RIFFfake" in up[3]
        body = json.loads(create[3])
        assert body["model"] == "ai-music-api/upload-and-cover-audio"
        assert body["input"]["upload_url"] == "https://tempfile.example/up.wav"
        assert body["input"]["audio_weight"] == 0.65

    def test_robot_song_add_instrumental_payload(self, tmp_path):
        mp3 = tmp_path / "v.mp3"
        mp3.write_bytes(b"ID3fake")
        p, t = _provider({
            "file-stream-upload": (200, {"code": 200, "data": {"downloadUrl": "https://tempfile.example/v.mp3"}}),
            "createTask": (200, {"code": 200, "data": {"taskId": "s1"}}),
        })
        job = GenJob(kind="robot_song", style_key="grunge", title="Я робот", style_tags="grunge",
                     lyrics="строка", audio_path=str(mp3), model="V6", vocal_gender="m", audio_weight=0.8)
        p.submit(job)
        body = json.loads(t.calls[1][3])
        assert body["model"] == "ai-music-api/add-instrumental"
        assert body["input"] == {"upload_url": "https://tempfile.example/v.mp3", "title": "Я робот",
                                 "model": "V6", "tags": "grunge", "lyrics": "строка",
                                 "vocal_gender": "m", "audio_weight": 0.8}

    def test_cover_without_audio_rejected(self):
        p, _ = _provider({})
        with pytest.raises(GenUnavailable) as e:
            p.submit(GenJob(kind="cover", style_key="x", title="t"))
        assert e.value.reason == "rejected"


class TestPoll:
    def test_success_parses_result_json_string_and_credits(self):
        p, _ = _provider({"recordInfo": (200, _record("success"))})
        ticket = _ticket(p)
        st = p.poll(ticket)
        assert st.stage == "complete" and st.credits == 12.0 and st.cost_time_s == 22.0
        assert [t.variant for t in st.tracks] == [0, 1]
        assert st.tracks[0].audio_url == "https://tempfile.example/0.mp3"
        assert st.tracks[1].duration_s == 60.0

    @pytest.mark.parametrize("state", ["waiting", "queuing", "generating"])
    def test_in_progress_without_tracks_is_queued(self, state):
        p, _ = _provider({"recordInfo": (200, _record(state, tracks=False))})
        assert p.poll(_ticket(p)).stage == "queued"

    def test_fail_state_carries_message(self):
        p, _ = _provider({"recordInfo": (200, _record("fail", tracks=False, failMsg="content policy"))})
        st = p.poll(_ticket(p))
        assert st.stage == "failed" and st.fail_msg == "content policy"

    def test_success_without_tracks_is_provider_down(self):
        p, _ = _provider({"recordInfo": (200, _record("success", tracks=False))})
        with pytest.raises(GenUnavailable) as e:
            p.poll(_ticket(p))
        assert e.value.reason == "provider_down"


def _ticket(p):
    from rob_box_mcp_tools.engine.gen.provider import GenTicket
    return GenTicket("task1", p.name, JOB, 0.0, 12.0)


class TestWaitComplete:
    def test_polls_until_complete_with_2s_then_4s_steps(self):
        states = ["waiting", "generating", "success"]
        p, _ = _provider({"recordInfo": [(200, _record(s, tracks=(s == "success"))) for s in states]})
        clock_t = [0.0]
        slept = []

        def sleep(s):
            slept.append(s)
            clock_t[0] += 40  # 40 с за шаг: второй шаг уже за порогом 30 с
        st = wait_complete(p, _ticket(p), sleep=sleep, clock=lambda: clock_t[0])
        assert st.stage == "complete" and slept == [2.0, 4.0]

    def test_failed_stage_raises_rejected(self):
        p, _ = _provider({"recordInfo": (200, _record("fail", tracks=False, failMsg="nope"))})
        with pytest.raises(GenUnavailable) as e:
            wait_complete(p, _ticket(p), sleep=lambda s: None, clock=lambda: 0.0)
        assert e.value.reason == "rejected" and e.value.detail == "nope"

    def test_timeout_after_180s_keeps_ticket(self):
        p, _ = _provider({"recordInfo": (200, _record("generating", tracks=False))})
        now = [0.0]

        def sleep(s):
            now[0] += s
        with pytest.raises(GenUnavailable) as e:
            wait_complete(p, _ticket(p), sleep=sleep, clock=lambda: now[0])
        assert e.value.reason == "timeout" and e.value.ticket.task_id == "task1"
        assert now[0] <= 180.0 + 1e-6

    def test_poll_interval_steps(self):
        assert poll_interval_s(0) == 2.0 and poll_interval_s(29.9) == 2.0 and poll_interval_s(30) == 4.0


class TestBalanceCostDownload:
    def test_balance_number(self):
        p, t = _provider({"chat/credit": (200, {"code": 200, "msg": "success", "data": 108})})
        assert p.balance() == 108.0
        assert t.calls[0][1] == "https://api.kie.ai/api/v1/chat/credit"

    def test_balance_dict_form(self):
        p, _ = _provider({"chat/credit": (200, {"code": 200, "data": {"credits": 7.5}})})
        assert p.balance() == 7.5

    def test_balance_unknown_is_none_not_zero(self):
        for routes in (
            {"chat/credit": (200, {"code": 500, "msg": "x"})},
            {"chat/credit": GenUnavailable("provider_down")},
            {"chat/credit": (200, {"code": 200, "data": "много"})},
        ):
            p, _ = _provider(routes)
            assert p.balance() is None

    def test_balance_without_key_is_none(self, monkeypatch):
        monkeypatch.delenv("KIE_API", raising=False)
        assert kie_suno.KieSunoProvider(transport=FakeTransport()).balance() is None

    def test_cost_from_table(self):
        p, _ = _provider({})
        assert p.cost(JOB) == GEN_MODELS["V6_MINI"].credits_per_task == 12.0

    def test_download_sends_own_user_agent(self):
        p, t = _provider({"audiostream.kie.ai": (200, b"MP3DATA")})
        assert p.download("https://audiostream.kie.ai/stream/x.mp3") == b"MP3DATA"
        ua = t.calls[0][2]["User-Agent"]
        assert ua and "python-urllib" not in ua.lower()

    def test_download_403_is_provider_down(self):
        p, _ = _provider({"audiostream": (403, b"forbidden")})
        with pytest.raises(GenUnavailable) as e:
            p.download("https://audiostream.kie.ai/stream/x.mp3")
        assert e.value.reason == "provider_down"
