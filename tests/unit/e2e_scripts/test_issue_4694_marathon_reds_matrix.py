# -*- coding: utf-8 -*-
"""#4694: красные шаги полного марафона 12.11 против текущего develop.

Источник сырых вердиктов — комментарии эпика #4694 (12–13.11): n309/n310 run 43199239029,
акт 4 run 43201271798, акт 5 run 43203031370, акт 8 run 43206628264. Матрица не пересказывает
комментарий: каждый шаг проверяет, что влитый фикс или честный каскад всё ещё в коде develop.
Ожидание, которое устарело под влитый код, здесь не нашлось — чинить нечего, и тест это держит.

Живой марафон не гоняется. Что ещё нужно на роботе — строки ``live`` матрицы.

Run:
    /home/ros2/rob_box_project/.venv-core/bin/pytest -v tests/unit/e2e_scripts/test_issue_4694_marathon_reds_matrix.py -o addopts=""
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "e2e"))
sys.path.insert(0, str(REPO_ROOT / "scripts" / "ci" / "e2e"))

import e2e_events as eev  # noqa: E402
import e2e_tool_match as etm  # noqa: E402
import gen_night_marathon as gen  # noqa: E402

HARNESS = (REPO_ROOT / "scripts" / "ci" / "e2e" / "e2e_voice_test.sh").read_text(encoding="utf-8")

# шаг -> (корень по сырому разбору 12.11, влитый фикс, что ещё нужно на роботе)
MATRIX = {
    "n309_bg_command_valera": (
        "voice",
        "PR #4705: шёпот Валеры +12 dB до STT, иначе backlog_miss на тихом EQ",
        "акт 3: n309 даёт [backlog] accumulated, n310 вызывает dj_set",
    ),
    "n310_execute_backlog_lru": (
        "cascade",
        "каскад n309: cycle_unconfirmed, потому что подхватывать было нечего",
        "тот же акт 3, после зелёного n309",
    ),
    "n401_list_voices": (
        "harness",
        "first_voice_cycle_position не считает промежуточную фразу содержательным ответом",
        "акт 4: list_tts_voices до содержательной речи — PASS, речь до тула — FAIL",
    ),
    "n402_current_voice": (
        "harness",
        "NAMES_DEFAULT_VOICE принимает Цинсэ/Квингсе, не только male-qn-qingse",
        "акт 4: ответ с именем голоса Qingse зелёный, ответ без имени — красный",
    ),
    "n411_pitch_bass": (
        "stt",
        "PR #4881: stt_replay до 3 попыток, если ПРИНЯТО потеряло «говори»/«бас»",
        "акт 4: живой STT той же фразы; офлайн-replay WAV run 43201271798 так и не снят",
    ),
    "n511_stop_music": (
        "open",
        "PR #4402/#4431 закрывают путь «глуши музыку» кодом; run 43203031370 ему противоречит, trace нет",
        "акт 5 со snapshot.music_playing и Addressed на входе «глуши музыку»",
    ),
    "n805b_unsilence": (
        "stt",
        "PR #4881: «говори»→«гоморели» — повтор фразы, не FAIL turn_silenced с первой попытки",
        "акт 8: n805 включил тишину, n805b снимает её tier1 kind=unsilence",
    ),
    "n806_long_count_again": (
        "cascade",
        "каскад n805b: робот ещё в тишине; отдельного дефекта счёта нет (min_count_end, PR #4872)",
        "акт 8 после зелёного n805b: счёт до сорока, не обрыв на десяти",
    ),
    "n807_merge_addition": (
        "cascade",
        "SKIP barge_in_anchor_missing честный: якорь n806 не наступил; #4342 живьём не подтверждён",
        "акт 8 после зелёного n806: правка счёта не обрывает речь и не повторяет числа",
    ),
    "n808_repeat_last": (
        "closed",
        "PR #4776: повтор берёт прозвучавшую реплику, не обещание; ключей у шага нет by design",
        "акт 8: «повтори» после n803 отдаёт услышанное «56», не обещание",
    ),
    "n810_rap_over_music": (
        "harness",
        "якорь n811 — perform_started после маркера kind=rap, не «Минутку»; рэп поверх бита — PR #4733",
        "акт 8: n810 стартует kind=rap, n811 не открывает окно по waiting-фразе",
    ),
}


def _steps():
    return {s["label"]: s for act in gen.ACTS for s in act["steps"]}


def test_matrix_covers_every_red_of_the_12_11_marathon():
    steps = _steps()
    assert set(MATRIX) <= set(steps)
    assert len(MATRIX) == 11


def test_n309_whisper_gets_the_measured_gain_and_others_do_not():
    """#4705: только filipp/kostya получают volume=12dB. Другой голос тем же фильтром не усиливается."""
    assert "filipp|kostya) filter=" in HARNESS and "volume=12dB" in HARNESS
    anton = HARNESS.split("filipp|kostya)", 1)[0].rsplit(")", 1)[-1]
    assert "volume=12dB" not in anton
    assert _steps()["n309_bg_command_valera"]["voice"] == "filipp"
    assert _steps()["n309_bg_command_valera"]["patterns"] == [gen.BACKLOG_ACC]


def test_n401_waiting_filler_is_not_a_verbal_answer():
    """Run 43201271798: «промежуточная фраза» до list_tts_voices — не ответ. Речь до тула по-прежнему красная."""
    filler = (
        "[dialog_node-3] 🧭 [turn-v2] промежуточная фраза: ждём llm 6.0 с turn=t1\n"
        "[tts_node-2] 🔊 TTS: speech_id=aaaa1111 text='Секунду.'\n"
        "Запрос выполнения: list_tts_voices с параметрами {}\n"
        "[tts_node-2] 🔊 TTS: speech_id=bbbb2222 text='Голоса: Антон, Борис.'\n"
    )
    assert etm.first_voice_cycle_position(filler) > filler.index("list_tts_voices")
    verbal = (
        "🔊 TTS: speech_id=cccc3333 text='У меня только антон'\n"
        "Запрос выполнения: list_tts_voices с параметрами {}\n"
    )
    assert etm.first_voice_cycle_position(verbal) < verbal.index("list_tts_voices")


def test_n402_names_qingse_in_cyrillic_and_rejects_a_nameless_answer():
    step = _steps()["n402_current_voice"]
    keys = step["acceptance"]["expected_keywords"]
    assert any("цинсэ" in k and "квингсе" in k for k in keys)

    def _said(words: str) -> str:
        return ("[tts_node-4] [INFO] [1.0] [tts_node]: 🔊 TTS: speech_id=d1 text='%s'" % words)

    assert etm.step_verdict(step["acceptance"], _said("Голос называется Цинсэ."))["pass"]
    # «мужской» содержит «муж» — ключ голоса; имя не названо, когда сказано чужое имя.
    assert not etm.step_verdict(step["acceptance"], _said("Сейчас я говорю на MiniMax, голос Надёжный мужчина."))["pass"]


def test_n411_and_n805b_replay_a_missed_command_up_to_three_times():
    """#4881: потеря слова шага — не FAIL с первой попытки. Потолок 3, считая первую."""
    assert eev.STT_GARBAGE_MAX_ATTEMPTS == 3
    for label, heard in (("n411_pitch_bass", "робот галибаса"), ("n805b_unsilence", "робот гоморели")):
        step = _steps()[label]
        assert "stt_replay" in step.get("recover", []), label
        first = eev.stt_garbage_retry(step["text"], heard, 1, missing="говори")
        assert first["retry"] == 1 and first["max"] == 3
        exhausted = eev.stt_garbage_retry(step["text"], heard, 3, missing="говори")
        assert exhausted["retry"] == 0 and exhausted["reason"] == "attempts_exhausted"


def test_n806_still_rejects_a_count_cut_to_ten():
    """#4872: каскад n805b не отменяет проверку длины. Обрыв на десяти — красный, когда шаг играется."""
    step = _steps()["n806_long_count_again"]
    assert step["acceptance"]["min_count_end"] == 20
    assert _steps()["n805b_unsilence"]["skip_when"]["absent"] is True


def test_n807_skip_is_the_missing_anchor_not_a_weakened_check():
    """12.11: SKIP barge_in_anchor_missing. Якорь n806 не наступил — шаг не играется, проверка счёта не снята."""
    step = _steps()["n807_merge_addition"]
    assert step["barge_in"]["after"] == "speech_started"
    assert step["acceptance"]["count_continues"] is True
    assert step["acceptance"]["no_repeat_numbers"] is True


def test_n808_repeat_has_no_keywords_because_the_code_repeats_what_was_heard():
    """#4776: фразу повтора строит код из прозвучавшего. Ключей в сценарии нет — их нечем ослабить."""
    step = _steps()["n808_repeat_last"]
    assert "expected_keywords" not in step.get("acceptance", {})
    log = (REPO_ROOT / "src/rob_box_dialog/rob_box_dialog/turn_log.py").read_text(encoding="utf-8")
    assert "def last_spoken" in log


def test_n810_anchor_is_the_rap_start_not_the_waiting_phrase():
    """Run 43206628264: «Минутку, обдумываю.» открыло окно n811. Якорь — старт kind=rap, не waiting-фраза."""
    step = _steps()["n810_rap_over_music"]
    assert gen.perform_started("rap") in step["patterns"]
    assert _steps()["n811_stop_speech_keep_music"]["barge_in"]["after"] == "perform_started"
    waiting = (
        "[stt_node] [INFO] [1.0] [stt_node]: ✅ ПРИНЯТО: робот зачитай рэп\n"
        "[tts_node] [INFO] [2.0] [tts_node]: 🔊 TTS: speech_id=w1 text='Минутку, обдумываю.'\n"
        "[tts_node] [INFO] [3.0] [tts_node]: 🗣️ SpeechEvent started speech_id=w1\n"
    )
    assert eev.anchor(waiting, "perform_started", 1)["found"] == 0
    rap = waiting + (
        "[dialog_node] [INFO] [4.0] [dialog_node]: 🎤 выступление старт kind=rap 'status': 'pending'\n"
        "[tts_node] [INFO] [5.0] [tts_node]: 🗣️ SpeechEvent started speech_id=r1\n"
    )
    assert eev.anchor(rap, "perform_started", 1)["found"] == 1


def test_n511_stop_is_still_required_when_music_plays():
    """Ожидание не ослаблено: музыка играет — stop_music обязателен. Trace run 43203031370 в develop нет."""
    step = _steps()["n511_stop_music"]
    assert "stop_music" in step["_result_markers"]["tools"]
    assert any("stop_music" in p for p in step["patterns"])
    assert step["skip_when"]["reason"] == "NO_MUSIC_N506"
