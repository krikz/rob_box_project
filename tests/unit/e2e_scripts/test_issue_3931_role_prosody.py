"""#3931: питч и темп роли доезжают до MiniMax и входят в ключ кэша синтеза.

Решение Шифу 23.10: голоса ролей марафона разнесены голосом, питчем и темпом. Таблица
знания -- ``map_tts_voice`` (строка ``voice:pitch:speed``); тест проверяет, что ровно эта
строка (а) разбирается настоящим ``synth_minimax`` в ``voice_setting`` запроса, (б) меняет
ключ кэша, чтобы старые wav не вернулись.
"""

import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from tests.unit.e2e_scripts.test_issue_3875_synth_retry_cache import (
    extract_function,
    needs_bash,
    run_bash,
)

WAV_HEX = (b"RIFF" + b"\x00" * 80).hex()


class _Capture(BaseHTTPRequestHandler):
    bodies = []

    def do_POST(self):  # noqa: N802
        length = int(self.headers.get("Content-Length", "0"))
        _Capture.bodies.append(json.loads(self.rfile.read(length)))
        payload = json.dumps({"base_resp": {"status_code": 0}, "data": {"audio": WAV_HEX}}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, *args):  # тишина в выводе pytest
        pass


@pytest.fixture()
def minimax_server():
    pytest.importorskip("requests")
    _Capture.bodies = []
    server = HTTPServer(("127.0.0.1", 0), _Capture)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield "http://127.0.0.1:%d" % server.server_port, _Capture.bodies
    server.shutdown()


def _synth(tmp_path, base_url, voice, speed="1.0"):
    return run_bash(
        tmp_path,
        "unset -f synth_minimax; source /dev/stdin <<'EOF_FN'\n%s\nEOF_FN\n" % extract_function("synth_minimax")
        + 'YANDEX_SPEED=%s MINIMAX_API_KEY=x MINIMAX_TTS_BASE_URL=%s '
        'synth_command "Робот, привет" %s "$OUT_DIR/cmd.wav" > "$OUT_DIR/s.log" 2>&1; echo "RC=$?"; cat "$OUT_DIR/s.log"'
        % (speed, base_url, voice),
    )


@needs_bash
class TestRoleProsodyReachesMinimax:
    @pytest.mark.parametrize(
        "voice,voice_id,pitch,speed",
        [
            # каст 26.10 (#3992): подбор по эмбеддингам на живом канале; Гриша и Валера - женские голоса
            ("anton", "English_Jovialman", 0, 1.0),
            ("ermil", "English_WiseScholar", 0, 1.0),
            ("zahar", "Chinese (Mandarin)_Gentle_Senior", 0, 1.0),
            ("filipp", "English_Whispering_girl", 0, 1.0),
            ("arina", "Portuguese_StrictBoss", 0, 1.0),
        ],
    )
    def test_role_pitch_and_speed_are_sent(self, tmp_path, minimax_server, voice, voice_id, pitch, speed):
        url, bodies = minimax_server
        out, err = _synth(tmp_path, url, voice)
        assert "RC=0" in out, (out, err)
        assert len(bodies) == 1
        setting = bodies[0]["voice_setting"]
        assert setting == {"voice_id": voice_id, "speed": speed, "pitch": pitch}

    def test_plain_native_voice_has_neutral_prosody(self, tmp_path, minimax_server):
        url, bodies = minimax_server
        out, err = _synth(tmp_path, url, "Russian_CrazyQueen")
        assert "RC=0" in out, (out, err)
        assert bodies[0]["voice_setting"] == {"voice_id": "Russian_CrazyQueen", "speed": 1.0, "pitch": 0}

    def test_global_speed_multiplies_role_speed(self, tmp_path, minimax_server):
        url, bodies = minimax_server
        out, err = _synth(tmp_path, url, "Russian_ReliableMan:2:1.1", speed="0.9")
        assert "RC=0" in out, (out, err)
        assert bodies[0]["voice_setting"] == {"voice_id": "Russian_ReliableMan", "speed": 0.99, "pitch": 2}


@needs_bash
class TestCacheKeyCoversProsody:
    def test_pitch_and_speed_change_the_key(self, tmp_path):
        out, err = run_bash(
            tmp_path,
            'for v in Russian_ReliableMan Russian_ReliableMan:3:1.12 Russian_ReliableMan:4:1.12 '
            'Russian_ReliableMan:3:1.2; do synth_cache_key minimax "$v" "Робот"; echo; done',
        )
        keys = [line for line in out.splitlines() if len(line) == 64]
        assert len(keys) == 4 and len(set(keys)) == 4, (out, err)
