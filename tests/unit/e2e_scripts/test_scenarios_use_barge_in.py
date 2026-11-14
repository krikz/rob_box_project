import json
from pathlib import Path

SCENARIO = (
    Path(__file__).resolve().parents[3]
    / ".github"
    / "e2e"
    / "scenarios"
    / "night"
    / "night_marathon_act8_barge_in_v1.json"
)


def test_n811_waits_for_actual_rap_speech():
    steps = json.loads(SCENARIO.read_text(encoding="utf-8"))["steps"]
    label = "n811_stop_speech_keep_music"
    n811 = next(step for step in steps if step["label"] == label)
    assert n811["barge_in"] == {
        "after": "perform_started",
        "n": 1,
        "delay_sec": 3.0,
    }
