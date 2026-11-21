"""ADR-0175 §1.2: режим «отвечай звуками» — решение кода, не второй путь звука.

Гард: число, имя и причина ошибки клипом не подменяются. «Сказал» строится только
из фактического ``finished``. ``started`` и ``skipped`` фразы успеха не дают.
"""

from __future__ import annotations

import json
from pathlib import Path

from rob_box_dialog_msgs.msg import SoundEvent
from rob_box_voice.core.sound_answer import (
    DISABLE_FORMS,
    ENABLE_FORMS,
    SoundAnswerGate,
    SpeechAsClip,
    StopFlag,
    aborted,
    arm,
    choose,
    classify,
    finished_phrase,
    keeps_precise_text,
    load_manifest,
    phrase_from_event,
)

_MANIFEST = Path(__file__).resolve().parents[3] / "config" / "sound_answer_manifest.json"
_SAID = "Сказал."


def test_stub_manifest_has_five_ids_and_no_foreign_files():
    body = json.loads(_MANIFEST.read_text(encoding="utf-8"))
    ids = [row["id"] for row in body["clips"]]
    assert ids == ["bird", "seal", "elephant", "alligator", "growl"]
    assert body["version"] == "stub-0"
    assert all("/" not in row["file"] and ".." not in row["file"] for row in body["clips"])
    loaded = load_manifest(str(_MANIFEST))
    assert loaded.version == "stub-0"
    assert len(loaded.clips) == 5


def test_enable_and_disable_are_closed_forms():
    assert classify("отвечай звуками") == "enable"
    assert classify("ну отвечай мне звуками пожалуйста") == "enable"
    assert classify("обычным голосом") == "disable"
    assert classify("верни обычный голос") == "disable"
    assert classify("стоп") == "stop"
    assert classify("отвечай звуками и спой") == ""
    assert all(classify(" ".join(form)) == "enable" for form in ENABLE_FORMS)
    assert all(classify(" ".join(form)) == "disable" for form in DISABLE_FORMS)


def test_precise_text_is_not_replaced_by_a_clip():
    manifest = load_manifest(str(_MANIFEST))
    for text in ("осталось 3 метра", "это Маша", "ошибка: нет файла", "не удалось открыть дверь"):
        assert keeps_precise_text(text, known_names=("Маша",))
        choice = choose(text, manifest, known_names=("Маша",))
        assert choice.clip_id is None
        assert choice.precise


def test_said_only_from_finished():
    assert finished_phrase(SoundEvent.KIND_FINISHED) == _SAID
    assert finished_phrase(SoundEvent.KIND_STARTED) is None
    assert finished_phrase(SoundEvent.KIND_SKIPPED, "not_found") is None
    assert finished_phrase(SoundEvent.KIND_SKIPPED, "stopped") is None
    assert finished_phrase(SoundEvent.KIND_FINISHED, "stopped") is None
    assert _SAID not in ""


def test_mode_lives_for_the_session_and_new_session_clears_it():
    gate = SoundAnswerGate(str(_MANIFEST), "")
    assert gate.observe("отвечай звуками", "s1", 0) == "enable"
    assert gate.active("s1", 0)
    assert gate.observe("стоп", "s1", 1) == "stop"
    assert gate.active("s1", 1)  # стоп клип рвёт, режим сессии не снимает
    assert gate.observe("обычным голосом", "s1", 1) == "disable"
    assert not gate.active("s1", 1)
    gate.observe("отвечай звуками", "s1", 2)
    gate.observe("привет", "s2", 0)  # новая сессия
    assert not gate.active("s2", 0)


def test_missing_file_does_not_replace_speech_or_claim_success(tmp_path):
    gate = SoundAnswerGate(str(_MANIFEST), str(tmp_path))
    gate.observe("отвечай звуками", "s", 0)
    assert gate.instead_of_speech("хорошо", "s", 0, "r1") is None
    assert gate.on_finished("r1", SoundEvent.KIND_FINISHED) is None


def test_ready_clip_replaces_speech_and_phrase_waits_for_finished(tmp_path):
    (tmp_path / "bird.wav").write_bytes(b"")
    gate = SoundAnswerGate(str(_MANIFEST), str(tmp_path))
    gate.observe("отвечай звуками", "s", 0)
    assert gate.instead_of_speech("привет", "s", 0, "r1") == "bird"
    assert gate.on_finished("r1", SoundEvent.KIND_STARTED) is None
    assert gate.on_finished("r1", SoundEvent.KIND_FINISHED) == _SAID


def test_speech_wrapper_does_not_speak_and_play(tmp_path):
    (tmp_path / "elephant.wav").write_bytes(b"")
    published = []
    spoken = []

    class _Snap:
        session_id, epoch = "s", 0

    class _Session:
        snapshot = _Snap()

    gate = SoundAnswerGate(str(_MANIFEST), str(tmp_path))
    clip = SpeechAsClip(gate, lambda *a, **k: spoken.append(a[2]) or "speech", published.append, _Session())
    gate.observe("отвечай звуками", "s", 0)
    assert clip("t", 0, "хорошо") == ""
    assert spoken == []
    assert published and "elephant" in published[0]
    assert clip("t", 0, "осталось 3") == "speech"
    assert spoken == ["осталось 3"]
    assert len(published) == 1  # точный текст клипом не дублируется


def test_finished_phrase_is_spoken_not_played_again(tmp_path):
    """Хост говорит «Сказал.» исходной речью. Через обёртку эта фраза сама стала бы клипом."""
    (tmp_path / "elephant.wav").write_bytes(b"")
    published = []
    spoken = []

    class _Snap:
        session_id, epoch = "s", 4

    class _Session:
        snapshot = _Snap()

    gate = SoundAnswerGate(str(_MANIFEST), str(tmp_path))
    clip = SpeechAsClip(gate, lambda *a, **k: spoken.append(a[2]) or "speech", published.append, _Session())
    gate.observe("отвечай звуками", "s", 4)
    assert clip("t", 4, "хорошо") == ""
    event = type("E", (), {"request_id": json.loads(published[0])["request_id"], "kind": SoundEvent.KIND_FINISHED,
                           "reason": ""})()
    phrase = phrase_from_event(clip, event)
    assert phrase == _SAID
    assert clip.say_finished("", 4, phrase) == "speech"
    assert spoken == [_SAID]
    assert len(published) == 1
    later = type("E", (), {"request_id": event.request_id, "kind": SoundEvent.KIND_SKIPPED, "reason": "stopped"})()
    assert phrase_from_event(clip, later) is None
    assert spoken == [_SAID]


def test_stop_generation_is_not_finished():
    flag = StopFlag()
    generation = arm(flag)
    assert not aborted(flag, generation)
    flag.set()
    assert aborted(flag, generation)
