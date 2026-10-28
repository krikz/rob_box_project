"""Issue #3880 (эпик #3683, PR-14) -- сценарии ночного марафона ждут поведения dialog v2, а не v1.

Прогоны 22-23.10 на v2 краснели по ожиданиям сценария (тул ``speak_text``, ``[backlog] flushed to LLM``,
``speaker='…'`` в строке бэклога, ``LLM dispatch skipped``, ``register_speaker`` у незнакомца), а не по
поведению робота. Тест держит три вещи:

* генератор и JSON синхронны (``gen_night_marathon.py --check``);
* паттерны сценариев существуют в исходниках v2 (строка лога собирается из f-строки самой ноды) и матчат образец;
* стоп-шаги акта 8 не требуют речи (``silent_ok``), но требуют отмены речи.

Run:
    python -m pytest tests/unit/e2e_scripts/test_issue_3880_marathon_v2_expectations.py -v --no-cov
"""

from __future__ import annotations

import importlib.util
import json
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
NIGHT = ROOT / ".github/e2e/scenarios/night"
GEN = ROOT / "scripts/e2e/gen_night_marathon.py"


def _gen():
    sys.path.insert(0, str(GEN.parent))
    spec = importlib.util.spec_from_file_location("gen_night_marathon_3880", GEN)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _steps(act_file: str) -> dict:
    data = json.loads((NIGHT / act_file).read_text(encoding="utf-8"))
    return {s["label"]: s for s in data["steps"]}


ACT1 = "night_marathon_act1_wakeup_v1.json"
ACT2 = "night_marathon_act2_acquaintance_v1.json"
ACT3 = "night_marathon_act3_backlog_diarization_v1.json"
ACT8 = "night_marathon_act8_barge_in_v1.json"

# Строки-образцы -- в формате f-строк нод v2 (источник проверяется ниже).
BACKLOG_LINE = "[dialog_node-4] [INFO] [1790801810.7]: [backlog] accumulated reason=no_wake entries=3"
TURN_LINE = ("[dialog_node-4] [INFO] [1790801811.2]: 📊 [turn] turn_id=ab12 epoch=1 agent=tars stage=done "
             "status=finished latency_ms=812 llm_calls=1 llm_provider=minimax tokens=412")
TURN_BARE_LINE = TURN_LINE.replace("llm_calls=1", "llm_calls=0")
STARTED_LINE = ("[tts_node-5] [INFO] [1790801812.0]: 🗣️ SpeechEvent started speech_id=91b3cb4b agent=tars "
                "epoch=1 text_len=140 ")
CANCELLED_LINE = ("[tts_node-5] [INFO] [1790801815.0]: 🗣️ SpeechEvent cancelled speech_id=91b3cb4b agent=tars "
                  "epoch=1 text_len=140 spoken_chars=44 reason=stopped")
INTERRUPT_LINE = ("[dialog_node-4] [INFO] [1790801815.0]: 🛑 [interrupt] stop via=interrupt_request 'робот стоп' "
                  "epoch=1 boundary=as_requested turns_cancelled=1 за 0.4 мс utterance=u1")
DISPATCH_LINE = "[llm_adapter-7] [INFO] [1790801813.0]: 📤 Отправлен запрос 3f9a01bc: request_music"


def _hit(pattern: str, line: str) -> bool:
    return re.search(pattern, line) is not None


def test_generated_json_is_up_to_date():
    assert _gen().check() == 0, "перегенерируйте: py -3 scripts/e2e/gen_night_marathon.py"


def test_manifest_keeps_manual_acts_2b_2c():
    acts = json.loads((NIGHT / "night_marathon_manifest.json").read_text(encoding="utf-8"))["acts"]
    ids = [a["act"] for a in acts]
    assert ids[:4] == [1, 2, "2b", "2c"], ids
    for a in acts:
        assert (ROOT / a["scenario_file"]).exists() and (ROOT / a["acceptance_file"]).exists(), a


def test_log_formats_the_patterns_rely_on_exist_in_v2_sources():
    node = (ROOT / "src/rob_box_voice/rob_box_voice/dialog_node.py").read_text(encoding="utf-8")
    assert 'f"[backlog] accumulated reason={decision.reason} entries="' in node
    assert '🛑 [interrupt] {stop.policy} via={via}' in node
    owner = (ROOT / "src/rob_box_voice/rob_box_voice/core/speech_owner.py").read_text(encoding="utf-8")
    assert "🗣️ SpeechEvent {kind} speech_id={rec.speech_id[:8]}" in owner
    tlog = (ROOT / "src/rob_box_dialog/rob_box_dialog/turn_log.py").read_text(encoding="utf-8")
    assert 'AGGREGATE_MARK = "📊 [turn]"' in tlog
    assert "stage={self.stage} " in tlog and "status={self.status} " in tlog and "llm_calls={self.llm_calls}" in tlog
    adapter = (ROOT / "src/rob_box_mcp_tools/rob_box_mcp_tools/llm_adapter.py").read_text(encoding="utf-8")
    assert 'f"📤 Отправлен запрос {request_id[:8]}: {tool_name}"' in adapter


def test_pattern_constants_match_sample_lines():
    g = _gen()
    assert _hit(g.BACKLOG_ACC, BACKLOG_LINE)
    assert _hit(g.TURN_DONE, TURN_LINE)
    assert _hit(g.TURN_WITH_LLM, TURN_LINE)
    assert not _hit(g.TURN_WITH_LLM, TURN_BARE_LINE), "голый вейк без бэклога (llm_calls=0) не доказывает подхват"
    assert _hit(g.SPEECH_STARTED, STARTED_LINE)
    assert _hit(g.SPEECH_CANCELLED, CANCELLED_LINE) and _hit(g.SPEECH_CANCELLED, INTERRUPT_LINE)
    assert not _hit(g.SPEECH_CANCELLED, STARTED_LINE)
    assert _hit(g.MUSIC_DISPATCH, DISPATCH_LINE)


def test_no_v1_expectations_left_in_the_fixed_steps():
    fixed = ([_steps(ACT1)["n107_command_intent_gate"]] + list(_steps(ACT3).values()) + list(_steps(ACT8).values()))
    for step in fixed:
        # v1-фразы допустимы только в пояснениях (_comment/_why), не в том, что харнесс проверяет
        checked = json.dumps({"patterns": step.get("patterns", []),
                              "tools": step.get("acceptance", {}).get("expected_tool_calls", [])},
                             ensure_ascii=False)
        for v1 in ("LLM dispatch skipped", "command intent", "flushed to LLM", "no_wake_word", "speaker='",
                   "task_delta", "speak_text", "ясный стоп"):
            assert v1 not in checked, (step["label"], v1)


def test_act8_speech_steps_expect_speech_event_not_speak_text():
    steps = _steps(ACT8)
    for label in ("n802_long_count", "n804_long_story", "n806_long_count_again", "n810_rap_over_music"):
        assert "SpeechEvent started" in " ".join(steps[label]["patterns"]), label
        assert "speak_text" not in steps[label].get("acceptance", {}).get("expected_tool_calls", []), label
    gate = json.loads((NIGHT / "night_marathon_act8_barge_in_acceptance_v1.json").read_text(encoding="utf-8"))
    assert "speak_text" not in gate["expected_tool_calls"]
    assert any("SpeechEvent started" in c for c in gate["expected_tool_calls"])


def test_act8_stop_steps_are_silent_ok_but_require_cancel():
    steps = _steps(ACT8)
    for label in ("n805_hard_stop", "n811_stop_speech_keep_music"):
        assert steps[label]["silent_ok"] is True, label
        assert any("SpeechEvent cancelled" in p and r"\[interrupt\]" in p for p in steps[label]["patterns"]), label
    # n803 стоп + новая тема: речь звучит, но старая рвётся
    assert any("SpeechEvent cancelled" in p for p in steps["n803_barge_in_new_topic"]["patterns"])
    # n807 -- добавка, НЕ стоп: молчать можно, рвать речь нельзя
    n807 = steps["n807_merge_addition"]
    assert n807["silent_ok"] is True
    assert "SpeechEvent cancelled" in n807["acceptance"]["must_not_call"]
    # n809 -- музыка: факт действия держит expected_tool_calls, а не речь
    n809 = steps["n809_music_bed"]
    assert n809["silent_ok"] is True and n809["acceptance"]["expected_tool_calls"] == ["request_music"]


def test_act3_backlog_flow_is_bare_robot_pickup():
    steps = _steps(ACT3)
    for label in ("n302_bg_sasha", "n303_bg_boris", "n304_bg_grisha_unknown", "n305_bg_valera_unknown",
                  "n308_bg_command_boris", "n309_bg_command_valera"):
        assert steps[label]["expect"] == "backlog", label
        assert steps[label]["patterns"] == [r"\[backlog\] accumulated reason=no_wake"], label
    for label in ("n305b_bare_robot_pickup", "n310_execute_backlog_lru"):
        assert steps[label]["text"] == "Робот.", label
        assert "sleep_before_sec" not in steps[label], "пауза съест границу свежего подхвата BACKLOG_FRESH_S=20 с"
        assert any("llm_calls=[1-9]" in p for p in steps[label]["patterns"]), label
    labels = list(steps)
    assert labels.index("n305b_bare_robot_pickup") == labels.index("n305_bg_valera_unknown") + 1
    assert labels.index("n310_execute_backlog_lru") == labels.index("n309_bg_command_valera") + 1
    # диктора проверяет ответ робота, а не строка бэклога (#3971, разбор 26.10: «неопознан» -- тоже честная метка)
    assert steps["n306_who_was_talking"]["acceptance"]["expected_keywords"] == ["Борис", _gen().UNKNOWN_VOICE_ANSWER]
    assert "patterns" in steps["n306_who_was_talking"] and not steps["n306_who_was_talking"]["patterns"]


def test_pickup_window_constants_match_the_scenario_comment():
    kn = (ROOT / "src/rob_box_dialog/rob_box_dialog/knowledge.py").read_text(encoding="utf-8")
    # #3975: дальность подхвата - всё окно бэклога (3 мин), а «по любому» (всё свежее, чужое тоже) осталось 20 с:
    # LRU акта 3 (n308 старше 20 с - не берётся, берётся n309) держится на BACKLOG_FRESH_S
    assert "BACKLOG_FRESH_S = 20.0" in kn, "сценарий акта 3 рассчитан на границу «свежего» подхвата 20 с"
    assert "BACKLOG_PICKUP_S = BACKGROUND_WINDOW_S" in kn, "дальность подхвата - окно бэклога, не вторая цифра"


def test_n210_no_longer_forbids_register_speaker():
    acc = _steps(ACT2)["n210_grisha_no_name"]["acceptance"]
    assert "register_speaker" not in acc.get("must_not_call", [])
    assert acc["must_not_identify_as"] == ["Борис", "Саша"]
    said = acc["must_not_say"]
    assert "Саш" not in said and "Борис" not in said, "голый стем краснил «…не перепутаю с Сашей» (#4077, n210)"
    assert {"Саша,", "ты Саша", "это Саша", "Борис,", "ты Борис"} <= set(said)
    assert {"Спартак", "пицц"} <= set(said), "факты других дикторов — как были"


@pytest.mark.parametrize("label", ["n805_hard_stop", "n807_merge_addition", "n809_music_bed",
                                   "n811_stop_speech_keep_music"])
def test_harness_reads_silent_ok_from_the_scenario(tmp_path, label):
    import shutil
    import subprocess
    bash = shutil.which("bash")
    if bash is None:
        pytest.skip("bash недоступен")
    lib = ROOT / ".github/workflows/scripts/e2e_voice_lib.sh"
    out = tmp_path / "o.tsv"
    script = ('source "%s"; parse_scenario_to_tsv "%s" "%s"; '
              'while IFS=$\'\\x1f\' eval "$E2E_SCENARIO_ROW_READ"; do '
              'echo "$label|$silent_ok"; done < "%s"'
              % (lib.as_posix(), (NIGHT / ACT8).as_posix(), out.as_posix(), out.as_posix()))
    p = subprocess.run([bash, "-c", script], capture_output=True, text=True, encoding="utf-8")
    assert "%s|1" % label in p.stdout.splitlines(), (p.stdout, p.stderr)


ACT4 = "night_marathon_act4_voice_prosody_v1.json"
SYNTH_LINE = ("[tts_node-5] [INFO] [1790801812.0]: 🗣️ SpeechEvent synthesized speech_id=91b3cb4b agent=tars "
              "epoch=1 text_len=40 duration_ms=2100 provider=minimax")
TALE_START_LINE = ("[dialog_node-4] [INFO] [1790801812.0]: 🧭 [turn-v2] выступление старт kind=tale backing=calm "
                   "segments=7 → {'status': 'pending', 'block_id': 'perform-ee2404b9048f'} turn=ee2404b9048f")
TALE_END_LINE = ("[dialog_node-4] [INFO] [1790801912.0]: 🎤 [perform] perform-: конец (done) сегментов 7, символов 917, "
                 "synced=True, подложка stopped")


def test_act4_voice_steps_use_v2_markers():
    """#3880 + #3971 (разбор 26.10, R4/R6): провайдер реплики назван (не «любой»), сказка -- по событиям выступления."""
    g = _gen()
    steps = _steps(ACT4)
    assert steps["n402_current_voice"]["patterns"] == [g.spoken_by("minimax")]
    assert steps["n404_speak_after_set"]["patterns"] == [g.spoken_by("yandex")]
    assert _hit(g.spoken_by("minimax"), SYNTH_LINE)
    assert not _hit(g.spoken_by("yandex"), SYNTH_LINE)
    assert not _hit(g.spoken_by("minimax"), STARTED_LINE)
    tale = steps["n412_skazka_multivoice"]
    assert tale["patterns"] == [g.perform_started("tale"), g.PERFORM_DONE]
    assert "set_voice" not in tale["acceptance"].get("expected_tool_calls", [])
    assert _hit(g.perform_started("tale"), TALE_START_LINE) and _hit(g.PERFORM_DONE, TALE_END_LINE)
    assert not _hit(g.PERFORM_DONE, TALE_END_LINE.replace("(done)", "(stopped)"))
    pitch = steps["n411_pitch_bass"]
    assert pitch["patterns"] == [g.prosody_changed((g.PITCH_PARAM,), g.BELOW_NORMAL)]
    syn = (ROOT / "src/rob_box_voice/rob_box_voice/core/speech_owner.py").read_text(encoding="utf-8")
    assert 'self._emit("synthesized", rec, duration_ms=int(duration_s * 1000), provider=provider)' in syn
    assert 'f"applied={applied!r} provider={provider} reason={reason}"' in syn


def test_no_act_json_expects_speak_tool_anywhere():
    """Акты 6/7/9/10 (#3880): тула ``speak_text`` в v2 нет -- ни в паттернах шагов, ни в GATE-1 акта."""
    for path in sorted(NIGHT.glob("night_marathon_act*_v1.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        if "acceptance" in path.name:
            assert "speak_text" not in json.dumps(data.get("expected_tool_calls"), ensure_ascii=False), path.name
            continue
        for s in data.get("steps", []):
            assert "speak_text" not in json.dumps(s.get("patterns"), ensure_ascii=False), (path.name, s["label"])


@pytest.mark.parametrize("act_file,label", [
    ("night_marathon_act9_world_outside_v1.json", "n906_anekdot_anim"),
    ("night_marathon_act9_world_outside_v1.json", "n909_foreign_language"),
    ("night_marathon_act10_finale_memory_v1.json", "n1011_goodnight"),
    ("night_marathon_act6_dj_booth_v1.json", "n605_dj_set_plan"),
])
def test_speech_steps_of_acts_6_9_10_wait_for_speech_started(act_file, label):
    pats = _steps(act_file)[label]["patterns"]
    assert any("SpeechEvent started" in p for p in pats), pats
