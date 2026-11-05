"""М11 (05.11, #4236): ключи acceptance n603/n306 и политика восстановления n806."""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
NIGHT = os.path.join(ROOT, ".github", "e2e", "scenarios", "night")
sys.path.insert(0, os.path.join(ROOT, "scripts", "ci", "e2e"))
import e2e_events as eev  # noqa: E402


def _step(act_file, label):
    with open(os.path.join(NIGHT, act_file), encoding="utf-8") as fh:
        return next(s for s in json.load(fh)["steps"] if s["label"] == label)


def _keywords(act_file, label):
    return "|".join(str(k) for k in _step(act_file, label)["acceptance"]["expected_keywords"]).lower()


def test_n603_accepts_dj_set_answer():
    keys = _keywords("night_marathon_act6_dj_booth_v1.json", "n603_confirm_dj_playing")
    for part in ("идёт диджей", "диджей-сет", "трек один из"):
        assert part in keys


def test_n306_accepts_nameless_voice_answer():
    keys = _keywords("night_marathon_act3_backlog_diarization_v1.json", "n306_who_was_talking")
    assert "имени нет" in keys and "без имени" in keys


def test_n806_may_recover_silenced_but_n805b_may_not():
    with open(os.path.join(NIGHT, "night_marathon_act8_barge_in_v1.json"), encoding="utf-8") as fh:
        policy = dict(eev.recover_policy(json.load(fh)))
    assert eev.SILENCED in policy["n806_long_count_again"]
    assert eev.SILENCED not in policy["n805b_unsilence"]
