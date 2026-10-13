"""tg_send_voice: токен не утекает в вывод, есть повторы, неуспех = False."""
import importlib.util
from pathlib import Path

import pytest

TOKEN = "123456:SECRETtoken_abc"
_P = Path(__file__).resolve().parents[3] / "scripts" / "music" / "live_dj" / "tg_send_voice.py"
_spec = importlib.util.spec_from_file_location("tg_send_voice", _P)
tsv = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(tsv)


class _R:
    def __init__(self, code, text="ok"):
        self.status_code, self.text = code, text


def _nosleep(_s):
    return None


@pytest.fixture
def ogg(tmp_path):
    p = tmp_path / "x.ogg"
    p.write_bytes(b"OggS")
    return str(p)


def test_mask_removes_token():
    assert TOKEN not in tsv.mask(f"url /bot{TOKEN}/sendVoice", TOKEN)


def test_exception_with_token_in_url_is_masked_and_fails(ogg, capsys):
    def post(url, **kw):
        raise ConnectionError(f"ConnectTimeout host api.telegram.org url={url}")
    ok, msg = tsv.send_voice(TOKEN, "1", "c", ogg, retries=3, pause=0, post=post, sleep=_nosleep)
    out = capsys.readouterr()
    assert ok is False
    assert TOKEN not in msg and TOKEN not in out.err and TOKEN not in out.out
    assert "ConnectionError" in msg and "<tok>" in msg


def test_retries_then_success(ogg):
    calls = []

    def post(url, **kw):
        calls.append(1)
        if len(calls) < 3:
            raise TimeoutError("boom " + url)
        return _R(200)
    ok, _ = tsv.send_voice(TOKEN, "1", "c", ogg, retries=4, pause=0, post=post, sleep=_nosleep)
    assert ok and len(calls) == 3


def test_http_error_retried_and_fails(ogg):
    calls = []

    def post(url, **kw):
        calls.append(1)
        return _R(502, "bad gateway")
    ok, msg = tsv.send_voice(TOKEN, "1", "c", ogg, retries=3, pause=0, post=post, sleep=_nosleep)
    assert not ok and len(calls) == 3 and msg.startswith("502")


def test_main_exit_code_nonzero_on_failure(monkeypatch):
    monkeypatch.setenv("TELEGRAM_BOT_TOKEN", TOKEN)
    monkeypatch.setenv("CHAT", "1")
    monkeypatch.setenv("C", "c")
    monkeypatch.setenv("B", "nonexistent_zzz")
    monkeypatch.setenv("TG_RETRIES", "1")
    assert tsv.main() == 1
