"""05.11 (Шифу, марафон): одиночное вейк-слово -- русским голосом, остальные реплики -- голосом роли.

Голос с акцентом (English_Jovialman) вейк «Робот!» теряет. Решение принимает код харнесса: ``e2e_events.is_bare_wake``
(единственный список ``BARE_WAKE_WORDS``) + ``wake_voice`` харнесса; реплика с командой голос роли сохраняет
(многоголосие актов -- биометрия).
"""

import re
import sys

import pytest

from tests.unit.e2e_scripts.test_issue_3875_synth_retry_cache import SCRIPTS_DIR, needs_bash, run_bash
from tests.unit.e2e_scripts.test_issue_3931_role_prosody import _Capture, minimax_server  # noqa: F401

HARNESS = SCRIPTS_DIR
sys.path.insert(0, str(HARNESS))
import e2e_events as eev  # noqa: E402


@pytest.mark.parametrize("text", ["Робот!", "Робот.", "робот", " Робби, ", "Роббокс…", "РОБОТ?!"])
def test_bare_wake_detected(text):
    assert eev.is_bare_wake(text)


@pytest.mark.parametrize("text", ["Робот, привет", "Робот, говори.", "Привет, робот", "Робота", "", "Робот робот"])
def test_phrases_with_speech_are_not_bare(text):
    assert not eev.is_bare_wake(text)


def test_recovery_phrase_is_bare_wake():
    assert eev.is_bare_wake(eev.RECOVERY["no_wake"].phrase)


def _synth(tmp_path, url, text, voice):
    return run_bash(
        tmp_path,
        "unset -f synth_minimax; source /dev/stdin <<'EOF_FN'\n%s\nEOF_FN\n" % _fn("synth_minimax")
        + 'E2E_EVENTS_PY="%s/e2e_events.py" MINIMAX_API_KEY=x MINIMAX_TTS_BASE_URL=%s '
        'synth_command "%s" %s "$OUT_DIR/cmd.wav" > "$OUT_DIR/s.log" 2>&1; echo "RC=$?"; cat "$OUT_DIR/s.log"'
        % (HARNESS.as_posix(), url, text, voice),
    )


def _fn(name):
    from tests.unit.e2e_scripts.test_issue_3875_synth_retry_cache import extract_function

    return extract_function(name)


@needs_bash
class TestBareWakeVoice:
    @pytest.mark.parametrize("voice", ["anton", "ermil", "zahar", "filipp"])
    def test_bare_wake_uses_russian_voice_for_every_cast_member(self, tmp_path, minimax_server, voice):  # noqa: F811
        url, bodies = minimax_server
        out, err = _synth(tmp_path, url, "Робот.", voice)
        assert "RC=0" in out, (out, err)
        assert bodies[0]["voice_setting"]["voice_id"] == "Russian_ReliableMan"
        assert "synth wake-only" in out

    def test_phrase_with_command_keeps_role_voice(self, tmp_path, minimax_server):  # noqa: F811
        url, bodies = minimax_server
        out, err = _synth(tmp_path, url, "Робот, привет", "anton")
        assert "RC=0" in out, (out, err)
        assert bodies[0]["voice_setting"]["voice_id"] == "English_Jovialman"
        assert not re.search("wake-only", out)

    def test_wake_voice_per_provider(self, tmp_path):
        out, err = run_bash(
            tmp_path,
            'for p in yandex silero minimax; do wake_voice "$p"; echo; done',
        )
        lines = [ln for ln in out.splitlines() if ln and not ln.startswith(">>>") and not ln.startswith("CALLS")]
        assert lines[:3] == ["anton", "aidar", "Russian_ReliableMan"], (out, err)
