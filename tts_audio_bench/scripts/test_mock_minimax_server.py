"""HTTP integration tests for :mod:`tts_audio_bench.scripts.mock_minimax_server`.

The mock server is the test bench's stand-in for the real
``https://api.minimax.io`` endpoint. We hit it directly over HTTP with
``urllib`` and assert:

* the request shape is validated (missing GroupId → 401 with base_resp 1001);
* the SSE chunked response emits ``data:<json>\n\n`` and a trailing
  ``data:[DONE]\n\n``;
* ``?fail_status=…`` injects base_resp errors;
* ``?chunk_count=2`` slices a fixture across two SSE events.

These tests deliberately use ``urllib.request`` (no ``requests`` dep)
so the bench stays self-contained.
"""
from __future__ import annotations

import json
import socket
import sys
import time
from pathlib import Path
from urllib import request as urlrequest
from urllib.error import HTTPError, URLError

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mock_minimax_server as mock_mod  # noqa: E402
from mock_minimax_server import start_server  # noqa: E402

FIXTURES = HERE.parent / "fixtures"


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _post(port: int, payload: dict, query: str = "") -> dict:
    """Return the parsed JSON envelope (non-streaming path)."""
    url = f"http://127.0.0.1:{port}/v1/t2a_v2?GroupId=test&{query}"
    req = urlrequest.Request(url, method="POST")
    req.add_header("Authorization", "Bearer test")
    req.add_header("Content-Type", "application/json")
    data = json.dumps(payload).encode()
    with urlrequest.urlopen(req, data=data, timeout=10) as resp:
        return json.loads(resp.read().decode())


def _post_stream(port: int, payload: dict, query: str = "") -> bytes:
    url = f"http://127.0.0.1:{port}/v1/t2a_v2?GroupId=test&{query}"
    req = urlrequest.Request(url, method="POST")
    req.add_header("Authorization", "Bearer test")
    req.add_header("Content-Type", "application/json")
    data = json.dumps(payload).encode()
    with urlrequest.urlopen(req, data=data, timeout=10) as resp:
        return resp.read()


@pytest.fixture()
def server():
    port = _free_port()
    s = start_server(port=port, fixtures_root=FIXTURES)
    time.sleep(0.05)
    try:
        yield port
    finally:
        s.shutdown()


_PCM_PAYLOAD = {
    "model": "speech-02-hd",
    "text": "hello",
    "voice_setting": {"voice_id": "male-qn-qingse"},
    "audio_setting": {"format": "pcm", "sample_rate": 32000, "channel": 1},
}


def test_mock_server_returns_pcm_envelope(server):
    env = _post(server, _PCM_PAYLOAD)
    assert env["base_resp"]["status_code"] == 0
    assert env["data"]["audio_sample_rate"] == 32000
    # Fixture: 1.00 s int16 mono @ 32 000 Hz → 64 000 bytes → 128 000 hex chars.
    # (Previous test expected 32 000 bytes / 64 000 hex chars — that was the
    # 16 000 Hz fixture size. The bench default SR is 32 000 Hz.)
    assert len(env["data"]["audio"]) == 128000


def test_mock_server_rejects_missing_group_id(server):
    # Override the request to omit GroupId — we must hit the same path
    # but with no query string at all, so mock returns 401 + base_resp 1001.
    url = f"http://127.0.0.1:{server}/v1/t2a_v2"
    req = urlrequest.Request(url, method="POST")
    req.add_header("Authorization", "Bearer test")
    req.add_header("Content-Type", "application/json")
    data = json.dumps(_PCM_PAYLOAD).encode()
    with pytest.raises(HTTPError) as exc_info:
        urlrequest.urlopen(req, data=data, timeout=10)
    assert exc_info.value.code == 401
    payload = json.loads(exc_info.value.read())
    assert payload["base_resp"]["status_code"] == 1001


def test_mock_server_rejects_missing_bearer(server):
    url = f"http://127.0.0.1:{server}/v1/t2a_v2?GroupId=test"
    req = urlrequest.Request(url, method="POST")
    req.add_header("Content-Type", "application/json")  # no Authorization
    data = json.dumps(_PCM_PAYLOAD).encode()
    with pytest.raises(HTTPError) as exc_info:
        urlrequest.urlopen(req, data=data, timeout=10)
    assert exc_info.value.code == 401


def test_mock_server_injects_failure_via_query():
    port = _free_port()
    s = start_server(port=port, fixtures_root=FIXTURES)
    time.sleep(0.05)
    try:
        env = _post(port, _PCM_PAYLOAD, query="fail_status=1003&fail_msg=blocked")
        assert env["base_resp"]["status_code"] == 1003
        assert env["base_resp"]["status_msg"] == "blocked"
    finally:
        s.shutdown()


def test_mock_server_injects_failure_via_default_args():
    port = _free_port()
    s = start_server(port=port, fixtures_root=FIXTURES, fail_status=1001, fail_msg="bad")
    time.sleep(0.05)
    try:
        env = _post(port, _PCM_PAYLOAD, query="")
        assert env["base_resp"]["status_code"] == 1001
        assert env["base_resp"]["status_msg"] == "bad"
    finally:
        s.shutdown()


def test_mock_server_streaming_emits_sse_envelope():
    port = _free_port()
    s = start_server(port=port, fixtures_root=FIXTURES)
    time.sleep(0.05)
    try:
        payload = dict(_PCM_PAYLOAD, stream=True)
        body = _post_stream(port, payload, query="chunk_count=2")
        # Body is chunked transfer-encoded at the HTTP layer but Python
        # already dechunks it for us — we should see 2 data:<json>\n\n
        # events followed by a [DONE] event.
        text = body.decode()
        events = [line for line in text.split("\n\n") if line.startswith("data:")]
        assert len(events) == 3  # 2 chunks + DONE
        assert events[-1] == "data:[DONE]"
        for line in events[:-1]:
            assert line.startswith("data:")
            payload_json = json.loads(line[len("data:"):])
            assert payload_json["base_resp"]["status_code"] == 0
            assert "chunk_index" in payload_json["extra_info"]
            assert payload_json["extra_info"]["chunk_count"] == 2
    finally:
        s.shutdown()
