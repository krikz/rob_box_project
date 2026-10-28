# -*- coding: utf-8 -*-
"""Карточка #3971 (сверка марафона 27.10): харнесс судит по ВИДУ результата, который просила реплика.

Что было зелёным ложно (raw -- комментарий координатора в #3971 от 27.10, ``docker logs voice-assistant``, UTC):

* n502 «сыграй на синтезаторе» -- ``engine=renardo`` отброшен, ``started gen_5b4e8031`` (трек библиотеки);
* n504/n506 «развивай» / «ускорь» -- оба ``started gen_e9756071_…`` с ``bpm=60.0``: другой трек, темп не вырос
  (после #4029 / PR #4116 правка играющего -- ``edit_music`` на такте того же трека: ``edit applied … bpm=…``);
* n707 «расскажи про трек» -- 09:46:05 «Нашёл ответ, но не смог сложить его в короткую фразу.», ход ``status=done``;
* n608 «хватит диджеить, выключай» -- после ``dj_set {"action": "stop"}`` робот сказал «Включаю музыку.»;
* n805b «Робот, говори» -- n805 потерял вейк (STT: «замолчи»), тишины не было, «говори» вне тишины -- каскадный FAIL.

Решение в коде (ADR-0148): одна таблица видов результата ``gen_night_marathon.RESULT_KINDS`` (доказательство, подмена,
противоречащая речь, сверка с музыкой до шага), судья сверки -- ``e2e_events.music_compare`` по строкам деки за акт,
фразы-заглушки -- таблица ``ROBOT_PHRASES`` по ключам каталога робота, предусловие n805b -- ``skip_when`` с ``absent``.
Сверка шагов с логом -- те же функции, что у живого харнесса (``e2e_tool_match``, ``e2e_events``).

Run:
    python -m pytest tests/unit/e2e_scripts/test_issue_3971_result_kind.py -v -o addopts=""
"""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / ".github" / "workflows" / "scripts"
NIGHT = REPO_ROOT / ".github" / "e2e" / "scenarios" / "night"
ACT5 = NIGHT / "night_marathon_act5_renardo_live_v1.json"
ACT8 = NIGHT / "night_marathon_act8_barge_in_v1.json"
E2E_SCRIPT = SCRIPTS_DIR / "e2e_voice_test.sh"
LIB = SCRIPTS_DIR / "e2e_voice_lib.sh"
FAKE = Path(__file__).resolve().parent / "fake_robot_ssh.sh"
for _dir in (REPO_ROOT / "scripts" / "e2e", SCRIPTS_DIR):
    if str(_dir) not in sys.path:
        sys.path.insert(0, str(_dir))

import e2e_events as eev  # noqa: E402
import e2e_tool_match as etm  # noqa: E402
import gen_night_marathon as gen  # noqa: E402

DONE = "📊 [turn] turn_id=t1 epoch=1 agent=personality stage=done status=finished latency_ms=900 llm_calls=1 " \
       "llm_provider=deepseek tokens=10"
FALLBACK = DONE.replace("status=finished", "status=done")


def _steps() -> Dict[str, Dict[str, Any]]:
    return {s["label"]: s for a in gen.ACTS for s in a["steps"]}


def _verdict(label: str, log: str) -> Dict[str, Any]:
    """Patterns + acceptance шага по логу -- как у харнесса (оба должны пройти)."""
    step = _steps()[label]
    missing = etm.patterns_missing(log, step["patterns"])
    acc = etm.step_verdict(step.get("acceptance", {}), log, label=label)
    return {"missing": missing, "acc": acc, "pass": not missing and acc["pass"]}


def _tts(text: str) -> str:
    """Что робот ПРОИЗНЁС (``robot_speech``: строка ``🔊 TTS: … text='…'`` tts_node)."""
    return ("[tts_node-4] [INFO] [1792795079.69] [tts_node]: 🔊 TTS: speech_id=764009bb, dialogue_id=None, "
            "batch=None None/None, voice=default, lang=default, text='%s'" % text)


#: Часы робота 23.10 (va_marathon.log): короткая метка ``ts`` в тестах -- секунды от T0 (метку строки лога харнесс
#: узнаёт только как ROS-время, 9+ цифр).
T0 = 1792793000.0


def _at(ts: float) -> float:
    return ts if ts >= 1e9 else T0 + ts


def _mcp(ts: float, msg: str) -> str:
    return "[mcp_server-9] [INFO] [%.3f] [mcp_server]: %s" % (_at(ts), msg)


def started(ts: float, track_id: str, bpm: float) -> str:
    """``player_owner.track_started`` (va_marathon.log:6024, 23.10)."""
    return _mcp(ts, "🎵 [music v2] started track_id=%s clock_beat=9792.069 start_beat=9792.0 phase_in_form=0.0 "
                    "late_beats=0.069 bpm=%s deck=A form_beats=192.0 players_aligned=True latency_s=0.5"
                % (track_id, bpm))


def stopped(ts: float, track_id: str, reason: str = "user_stop") -> str:
    """``player_owner.stop`` (va_marathon.log:6773, 23.10)."""
    return _mcp(ts, "🎵 [music v2] stop track_id=%s reason=%s" % (track_id, reason))


def edited(ts: float, track_id: str, bpm: float) -> str:
    """``engine/track_edit.TrackEditor._retune`` (#4029): правка встала на такте того же трека."""
    return _mcp(ts, "🎚️ [music v2] edit applied track_id=%s at_beat=9856 bpm=%g" % (track_id, bpm))


def _judge(kind: str, lines, since: float, now: float = 1793200000.0) -> Dict[str, object]:
    return eev.music_compare(kind, "%.3f\n%s" % (now, "\n".join(lines)), _at(since))


# ── одна таблица видов результата ──────────────────────────────────────────────────────────────────────────────


def test_result_kinds_name_only_compares_the_judge_knows_and_steps_carry_them():
    """Генератор пишет поле ``music_compare``, судья читает его своей таблицей -- имена из одной таблицы."""
    compares = {k: r.compare for k, r in gen.RESULT_KINDS.items() if r.compare}
    assert set(compares.values()) <= set(eev.COMPARES)
    steps = _steps()
    asked = {
        "n502_start_renardo": "synth", "n711_renardo_not_broken": "synth", "n809_music_bed": "synth",
        "n504_add_bass": "edited", "n505_vibe_dark": "edited", "n506_faster": "faster",
        "n604_dj_rock": "changed", "n607_vibe_chill": "changed", "n602_dj_on": "started", "n708_gen_play": "started",
        "n312_stop_music": "stopped", "n511_stop_music": "stopped", "n608_dj_off": "stopped",
        "n709_stop_generated": "stopped", "n712_stop_all": "stopped", "n812_stop_music": "stopped",
    }
    for label, kind in asked.items():
        step = steps[label]
        assert step["_result_markers"]["result"] == kind, label
        assert step.get(eev.COMPARE_FIELD, "") == gen.RESULT_KINDS[kind].compare, label
        assert eev.compare_kind(step) == gen.RESULT_KINDS[kind].compare, label
        for proof in gen.RESULT_KINDS[kind].proof:
            assert proof in step["patterns"], (label, proof)
    with pytest.raises(KeyError):
        gen.action_step("nX", "Робот, сыграй.", ["request_music"], result="loud")


def test_generated_scenario_carries_the_compare_field_for_the_harness():
    """JSON на диске (его читает харнесс) несёт ``music_compare`` -- не только словарь генератора."""
    act5 = {s["label"]: s for s in json.loads(ACT5.read_text(encoding="utf-8"))["steps"]}
    assert act5["n506_faster"][eev.COMPARE_FIELD] == "faster"
    assert act5["n511_stop_music"][eev.COMPARE_FIELD] == "stopped"
    for label in ("n504_add_bass", "n505_vibe_dark"):
        assert eev.COMPARE_FIELD not in act5[label], "правка тембра и баса доказана строкой шага, сверка не нужна"
        assert gen.EDIT_APPLIED in act5[label]["patterns"], label


def test_edit_of_the_playing_track_is_the_edit_tool_on_the_bar_not_a_new_track():
    """#4029 (PR #4116): «добавь баса» / «помрачнее» -- ``edit_music`` и ``edit applied`` того же трека; фраза нового
    трека «Включаю музыку.» (27.10 на n504-n506) и любой новый трек на деке -- не правка."""
    ok = ["✅ Инструмент edit_music выполнен успешно", edited(5.0, "req93513:01:A:bce191fa", 136.0),
          _tts("Добавил баса и сделал звук тяжелее."), DONE]
    for label in ("n504_add_bass", "n505_vibe_dark"):
        assert _verdict(label, "\n".join(ok))["pass"], label
        assert not _verdict(label, "\n".join(ok + [_tts("Включаю музыку.")]))["pass"], label
        new_track = ok + [_mcp(4.0, "🎵 [music v2] exec track_id=req93574:01:A:64e243fb deck=A bpm=125 — ждём started")]
        assert not _verdict(label, "\n".join(new_track))["pass"], label
        failed = ["✅ Инструмент edit_music выполнен успешно", _tts("Не получилось поменять музыку — играет как было."),
                  DONE]
        assert not _verdict(label, "\n".join(failed))["pass"], label


def test_n502_synth_route_with_a_library_track_on_the_deck_is_not_the_synth():
    """27.10: путь и тул в порядке, а на деке не синт-трек -- красный; синт-трек ``req…`` -- зелёный."""
    route = _mcp(1.0, "🎚️ [source] asked=[] genre=ambient engine=renardo кандидаты=['synth:ambient'] → synth:ambient "
                      "(code) gen=ok")
    done = "✅ Инструмент request_music выполнен успешно"
    synth = "\n".join([route, done, started(2.0, "req93513:01:A:bce191fa", 136.0), DONE])
    assert _verdict("n502_start_renardo", synth)["pass"]
    library = "\n".join([route, done, started(2.0, "gen_5b4e8031_1793093054", 60.0), DONE])
    verdict = _verdict("n502_start_renardo", library)
    assert not verdict["pass"] and verdict["missing"] == [gen.SYNTH_STARTED]
    dj_set = "\n".join([route, done, started(2.0, "set12345:01:A:ab12", 120.0), DONE])
    assert not _verdict("n809_music_bed", dj_set)["pass"], "сет диджея -- не трек синтеза по заказу"


def test_n809_asks_the_synth_by_the_engine_word_stt_keeps():
    """«на рендардо» STT путал («рем дарда», #4070) -- просьба синта словом «на синтезаторе», как n502/n711."""
    for label in ("n502_start_renardo", "n711_renardo_not_broken", "n809_music_bed"):
        text = _steps()[label]["text"]
        assert "синтезатор" in text and "рендардо" not in text.lower(), label


# ── заглушки и status=done ─────────────────────────────────────────────────────────────────────────────────────


def test_robot_phrases_are_the_catalog_phrases():
    """Таблица фраз генератора -- те же варианты, что в каталоге робота (``phrases/ru.yaml``), ключ в ключ."""
    yaml = pytest.importorskip("yaml")
    catalog = yaml.safe_load((REPO_ROOT / "src/rob_box_dialog/rob_box_dialog/phrases/ru.yaml").read_text(
        encoding="utf-8"))
    for key, variants in gen.ROBOT_PHRASES.items():
        value = catalog[key]
        assert variants == ((value,) if isinstance(value, str) else tuple(value)), key
    assert set(gen.STUB_PHRASE_KEYS) <= set(gen.ROBOT_PHRASES)


def test_every_step_is_red_on_a_stub_phrase_or_a_turn_closed_by_a_stub():
    """Заглушка -- не ответ ни для одного шага: все шаги марафона несут обе проверки."""
    for label, step in _steps().items():
        acc = step["acceptance"]
        assert gen.TURN_FALLBACK in acc["must_not_call"], label
        assert gen.STUB_SPEECH in acc["must_not_say"], label
    for text in ("Нашёл ответ, но не смог сложить его в короткую фразу. Спроси ещё раз?", "Не вышло.",
                 "Хм, я задумался и ничего не придумал. Повтори?", "DeepSeek так и не ответил — я перестал ждать. "
                 "Повтори, пожалуйста.", "Не понял, как это сделать. Скажи ещё раз?"):
        assert etm.keyword_hit(_tts(text), gen.STUB_SPEECH), text
    for honest in ("Это «Енот-механик», рок, две минуты сорок.", "Тех, кто не назвался, по именам не знаю.",
                   "Вышло отлично!"):
        assert not etm.keyword_hit(_tts(honest), gen.STUB_SPEECH), honest


def test_n707_stub_answer_and_status_done_are_red_honest_answer_green():
    """27.10 09:46:05: n707 засчитан на заглушке при ``status=done`` -- теперь красный."""
    call = "📥 Запрос выполнения: music_library с параметрами {'action': 'info', 'track_id': 'gen_ab12'}"
    stub = "\n".join([call, _tts("Нашёл ответ, но не смог сложить его в короткую фразу. Спроси ещё раз?"), FALLBACK])
    verdict = _verdict("n707_gen_track_info", stub)
    assert not verdict["pass"]
    assert verdict["acc"]["forbidden_said"] and gen.TURN_FALLBACK in verdict["acc"]["forbidden_calls"]
    only_status = "\n".join([call, _tts("Это «Енот-механик», рок, две минуты сорок."), FALLBACK])
    assert not _verdict("n707_gen_track_info", only_status)["pass"], "ход закрыт заглушкой кода, а не ответом модели"
    honest = "\n".join([call, _tts("Это «Енот-механик», рок, две минуты сорок."), DONE])
    assert _verdict("n707_gen_track_info", honest)["pass"]


# ── стоп: фраза из результата, не холостой ─────────────────────────────────────────────────────────────────────


def test_stop_phrase_must_come_from_the_stop_result():
    """27.10 09:37:37: после ``dj_set stop`` -- «Включаю музыку.»: фраза ЗАПУСКА на стопе -- красный."""
    base = ["📤 Отправлен запрос ab: dj_set", "✅ Инструмент dj_set выполнен успешно",
            stopped(5.0, "gen_343d5428_1793093828"), DONE]
    assert _verdict("n608_dj_off", "\n".join(base + [_tts("Выключил музыку.")]))["pass"]
    for lie in ("Включаю музыку.", "Секунду, музыка запускается.",
                "Сейчас ничего не играет — на всякий случай всё остановил."):
        assert not _verdict("n608_dj_off", "\n".join(base + [_tts(lie)]))["pass"], lie


def test_a_new_set_replacing_the_old_one_is_not_a_stop():
    """``stop … reason=new_set`` -- деку занял новый сет (``dj_set start``), это не «выключи»."""
    log = "\n".join(["✅ Инструмент dj_set выполнен успешно", stopped(5.0, "gen94023:01:A", "new_set"),
                     started(6.0, "gen94122:01:A", 60.0), DONE])
    assert not _verdict("n608_dj_off", log)["pass"]


# ── судья сверки с музыкой ДО шага (e2e_events.music_compare) ────────────────────────────────────────────────


def test_faster_is_the_same_track_edited_to_a_higher_bpm():
    """#4029: «ускорь» -- правка на такте того же трека (``edit applied``, без нового ``started``), bpm выше игравшего;
    до шага трек уже правили (n504/n505 -- тембр и бас, bpm тот же) -- сверка с последним темпом деки."""
    track = "req93513:01:A:bce191fa"
    before = [started(100.0, track, 136.0), edited(150.0, track, 136.0)]
    verdict = _judge("faster", before + [edited(210.0, track, 147.0)], 200.0)
    assert verdict["ok"] == 1 and "до шага %s bpm=136, в конце шага %s bpm=147" % (track, track) in verdict["detail"]
    lower = _judge("faster", before + [edited(210.0, track, 126.0)], 200.0)
    assert lower["ok"] == 0 and "темп не вырос" in lower["detail"]
    unchanged = _judge("faster", before, 200.0)
    assert unchanged["ok"] == 0 and "темп не вырос" in unchanged["detail"]
    foreign = _judge("faster", before + [edited(210.0, "req11111:01:A:x", 160.0)], 200.0)
    assert foreign["ok"] == 0, "правка чужого трека не меняет играющий"


def test_faster_by_a_new_track_is_not_faster_raw_23_10_and_27_10():
    """23.10 n505 -> n506: req93603 bpm=123 -> req93630 bpm=124 (новый трек, случайно на 1 быстрее); 27.10 тот же трек
    библиотеки заново при bpm=60 -- оба не «ускорь»."""
    raw_2310 = [started(1792793604.46, "req93603:03:A:d0ef59cb", 123.0),
                started(1792793631.57, "req93630:03:A:929b381b", 124.0)]
    verdict = _judge("faster", raw_2310, 1792793625.0)
    assert verdict["ok"] == 0 and "трек сменился" in verdict["detail"]
    raw_2710 = [started(1793093116.0, "gen_e9756071_1793093114", 60.0),
                started(1793093184.0, "gen_e9756071_1793093182", 60.0)]
    assert _judge("faster", raw_2710, 1793093170.0)["ok"] == 0


def test_stopped_needs_music_before_a_user_stop_and_silence_after():
    playing = started(100.0, "req93630:03:A:929b381b", 124.0)
    ok = _judge("stopped", [playing, stopped(210.0, "req93630:03:A:929b381b")], 200.0)
    assert ok["ok"] == 1, ok
    idle = _judge("stopped", [stopped(210.0, "None")], 200.0)
    assert idle["ok"] == 0 and "не играла" in idle["detail"], "n812 23.10: стоп вхолостую"
    replaced = _judge("stopped", [playing, stopped(210.0, "req93630:03:A:929b381b", "new_set"),
                                  started(211.0, "set1:01:A:ab", 60.0)], 200.0)
    assert replaced["ok"] == 0 and "играет" in replaced["detail"]
    restarted = _judge("stopped", [playing, stopped(210.0, "req93630:03:A:929b381b"),
                                   started(215.0, "req99999:01:A:cd", 120.0)], 200.0)
    assert restarted["ok"] == 0, "выключил и тут же включил другое -- не «выключи»"


def test_judge_reads_only_deck_events_with_robot_time_and_says_when_it_could_not_read():
    stale = _mcp(150.0, "⚠️ [music v2] started track_id=req2:01:A:x устарел — заменён до старта")
    waiting = _mcp(205.0, "🎚️ [music v2] edit track_id=req1:01:A:x at_beat=96 bpm→150 — ждём такта")
    before = started(100.0, "req1:01:A:x", 120.0)
    verdict = _judge("faster", [before, stale, waiting], 200.0)
    assert verdict["ok"] == 0 and "в конце шага req1:01:A:x bpm=120" in verdict["detail"], \
        "устаревший старт и правка, которая не встала на такт, -- не события деки"
    no_clock = eev.music_compare("faster", started(100.0, "req1:01:A:x", 120.0), _at(50.0))
    assert no_clock == {"fetch": 0, "ok": 0, "detail": "лог робота не прочитан -- сверка не доказана"}
    assert eev.music_event("🎵 [music v2] started track_id=req1:01:A:x bpm=120.0") is None, "без метки времени"


def test_step_window_start_is_the_harness_utc_stamp():
    assert eev.epoch_of("2026-10-27T23:24:30Z") == pytest.approx(1793143470.0)
    assert eev.epoch_of("1793143470.5") == 1793143470.5
    assert eev.epoch_of("вчера") is None


def test_compare_cli_reads_the_field_from_the_scenario(tmp_path):
    """CLI харнесса: ``compare --scenario --label`` без ``--since`` -- только вид сверки; с ним -- вердикт по stdin."""
    def run(*args, stdin=""):
        p = subprocess.run([sys.executable, str(SCRIPTS_DIR / "e2e_events.py"), "compare", *args], input=stdin,
                           capture_output=True, text=True, encoding="utf-8")
        return p.returncode, dict(line.split("=", 1) for line in p.stdout.splitlines() if "=" in line), p.stderr

    assert run("--scenario", str(ACT5), "--label", "n506_faster")[1] == {"compare": "faster"}
    assert run("--scenario", str(ACT5), "--label", "n505_vibe_dark")[1] == {"compare": "-"}
    log = "1792793700.0\n" + "\n".join([started(1792793604.46, "req93603:03:A:d0ef59cb", 123.0),
                                        started(1792793631.57, "req93630:03:A:929b381b", 124.0)])
    rc, out, _ = run("--scenario", str(ACT5), "--label", "n506_faster", "--since", "1792793625", stdin=log)
    assert rc == 0 and out["compare"] == "faster" and out["fetch"] == "1" and out["ok"] == "0"
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"steps": [{"label": "x", eev.COMPARE_FIELD: "louder"}]}), encoding="utf-8")
    rc, _, err = run("--scenario", str(bad), "--label", "x")
    assert rc == 2 and "louder" in err, "кривое поле -- громко, а не шаг без сверки"


# ── n805b: предусловие тишины и сочетание с восстановлением ─────────────────────────────────────────────────────


def _act8() -> Dict[str, Any]:
    return json.loads(ACT8.read_text(encoding="utf-8"))


def test_n805b_is_not_played_without_the_silence_of_n805():
    """27.10 09:56: n805 потерял вейк (STT «замолчи», бэклог), тишины нет -- n805b SKIP NOT_SILENCED, а не FAIL."""
    step = next(s for s in _act8()["steps"] if s["label"] == "n805b_unsilence")
    rule = eev.skip_rule(step)
    assert rule.reason == "NOT_SILENCED" and rule.absent
    lost_wake = "1793100000.0\n[dialog_node-3] [INFO] [1793099761.85] [dialog_node]: 🎤 STT: замолчи\n" \
                "[dialog_node-3] [INFO] [1793099761.86] [dialog_node]: [backlog] accumulated reason=no_wake entries=1"
    verdict = eev.skip_verdict(rule, lost_wake)
    assert verdict["skip"] == "NOT_SILENCED" and "нет строки" in verdict["evidence"]
    silenced = lost_wake + "\n[dialog_node-3] [INFO] [1793099770.0] [dialog_node]: 🧭 [turn-v2] tier1 kind=silence " \
                           "args={} turn=ab12"
    assert eev.skip_verdict(rule, silenced)["skip"] == "-"
    assert eev.skip_verdict(rule, "не часы\n")["fetch"] == 0, "лог не прочитан -- шаг играется (громко в лог)"


def test_n805b_own_lost_wake_is_recovered_by_the_bare_wake_never_by_its_own_phrase():
    """Сочетание с #3989: своя потеря вейка -- «Робот!» (подхват «говори» из бэклога -- Tier-1 unsilence в тишине);
    «Робот, говори.» харнессом -- это и есть шаг, такое восстановление запрещено кодом. n805 -- перебиватель, без
    восстановления."""
    act8 = _act8()
    steps = {s["label"]: s for s in act8["steps"]}
    assert eev.recover_kinds(act8, steps["n805b_unsilence"]) == ("no_wake",)
    assert eev.recover_kinds(act8, steps["n805_hard_stop"]) == ()
    assert eev.RECOVERY["no_wake"].phrase == "Робот!" and not eev.RECOVERY["no_wake"].replay


@pytest.mark.parametrize("rel,fragment", [
    # строки деки, которые читает судья и паттерны видов результата
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/engine/player_owner.py", 'f"🎵 [music v2] started track_id={track_id} "'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/engine/player_owner.py",
     '"clock_beat", "start_beat", "phase_in_form", "late_beats",'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/engine/player_owner.py", '"bpm", "deck", "form_beats"'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/engine/player_owner.py", 'f"🎵 [music v2] finished track_id={track_id}"'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/engine/track_edit.py",
     'f"🎚️ [music v2] edit applied track_id={track_id} at_beat={outcome.get(\'at_beat\'):g} "'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/engine/track_edit.py", 'f"bpm={outcome.get(\'bpm\'):g}"'),
    # стоп человека -- user_stop; деку занял другой заказ -- другая причина
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/tools/music.py", 'stop_deck("user_stop")'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/engine/tools_v2.py", 'self._session.stop("new_set")'),
    # синт-трек заказа -- сет req<сид>
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/engine/tools_v2.py", 'set_id=f"req{seed % 100000:05d}"'),
    # тул правки играющего и его фразы
    ("src/rob_box_dialog/rob_box_dialog/knowledge.py", 'EDIT_TOOL = "edit_music"'),
    ("src/rob_box_mcp_tools/rob_box_mcp_tools/engine/tools_v2.py", 'return "edit_music"'),
    # тишина Tier-1 и снятие только в тишине (n805b)
    ("src/rob_box_dialog/rob_box_dialog/grammar.py", 'if snapshot.top("Silence") is None or not has_word(text, '
                                                     'kn.UNSILENCE_WORDS):'),
    # ход, закрытый заглушкой кода, -- status=done
    ("src/rob_box_dialog/rob_box_dialog/turn.py", 'turn.stage, turn.status, turn.reason = "done", "done", '
                                                  '"query_fallback:ceiling"'),
])
def test_marker_sources_exist_in_robot_code(rel, fragment):
    """Маркеры видов результата собраны из строк кода робота: поменяли строку -- тест краснеет раньше марафона."""
    assert fragment in (REPO_ROOT / rel).read_text(encoding="utf-8"), rel


def test_skip_rule_absent_must_be_a_bool():
    with pytest.raises(ValueError, match="absent"):
        eev.skip_rule({"label": "x", "skip_when": {"reason": "NOT_SILENCED", "log": "a", "absent": "yes"}})


# ── bash: настоящая check_music_compare харнесса на фейковом роботе ────────────────────────────────────────────

#: Полный путь: на Windows ``subprocess`` ищет «bash» сначала в System32 (WSL), а не по PATH (Git Bash).
BASH = shutil.which("bash")
needs_bash = pytest.mark.skipif(BASH is None, reason="нужен bash")


def _extract(name: str) -> str:
    lines = E2E_SCRIPT.read_text(encoding="utf-8").replace("\r\n", "\n").split("\n")
    start = next(i for i, ln in enumerate(lines) if ln.startswith(name + "() {"))
    end = next(i for i in range(start + 1, len(lines)) if lines[i] == "}")
    return "\n".join(lines[start:end + 1])


def _harness(tmp_path: Path, log_lines, label: str, since: str):
    log = tmp_path / "robot.log"
    log.write_text("\n".join(log_lines) + "\n", encoding="utf-8")
    script = "\n".join([
        "set -u",
        # python3 харнесса -- тот же интерпретатор, что у pytest (на Windows python3 в PATH бывает заглушкой Store)
        'python3() { "%s" "$@"; }' % Path(sys.executable).as_posix(),
        'export FAKE_ROBOT_STATE="%s" FAKE_ROBOT_LOG="%s"' % ((tmp_path / "state").as_posix(), log.as_posix()),
        'ROBOT_SSH="bash %s"' % FAKE.as_posix(),
        'SCRIPT_DIR_E2E="%s"' % SCRIPTS_DIR.as_posix(),
        'log() { echo ">>> $*"; }',
        LIB.read_text(encoding="utf-8").replace("\r\n", "\n"),
        _extract("check_music_compare"),
        'E2E_RUN_BEFORE=600s; check_music_compare "%s" "%s" "%s"; echo "RC=$?"' % (label, ACT5.as_posix(), since),
    ])
    runner = tmp_path / "harness.sh"  # файлом: библиотека харнесса длиннее предела командной строки Windows
    with open(runner, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(script + "\n")
    p = subprocess.run([BASH, runner.as_posix()], capture_output=True, text=True, encoding="utf-8", timeout=60,
                       env=dict(__import__("os").environ, PYTHONUTF8="1"))
    return p.stdout, p.stderr


def _fake(ts_ago: float, msg: str) -> str:
    return "[mcp_server-9] [INFO] [{NOW-%s}] [mcp_server]: %s" % (ts_ago, msg)


@needs_bash
def test_harness_turns_a_random_faster_track_red_and_a_real_one_green(tmp_path):
    since = "%.0f" % (time.time() - 30)
    base = _fake(60, "🎵 [music v2] started track_id=req93513:01:A:x clock_beat=1 bpm=120.0 deck=A")
    other = _fake(10, "🎵 [music v2] started track_id=req93630:01:A:y clock_beat=1 bpm=124.0 deck=A")
    out, err = _harness(tmp_path, [base, other], "n506_faster", since)
    assert "PATTERN_MISS: music_compare=faster: трек сменился" in out and "RC=1" in out, (out, err)
    faster = _fake(10, "🎚️ [music v2] edit applied track_id=req93513:01:A:x at_beat=96 bpm=130")
    out, err = _harness(tmp_path, [base, faster], "n506_faster", since)
    assert "PATTERN_OK: music_compare=faster: до шага req93513:01:A:x bpm=120, в конце шага req93513:01:A:x bpm=130" \
        in out and "RC=0" in out, (out, err)


@needs_bash
def test_harness_skips_the_compare_for_a_step_without_the_field(tmp_path):
    out, err = _harness(tmp_path, [], "n505_vibe_dark", "%.0f" % time.time())
    assert out.strip() == "RC=0", (out, err)
    assert not (tmp_path / "state" / "ssh_calls").exists(), "без поля сверки лог робота не читается"
