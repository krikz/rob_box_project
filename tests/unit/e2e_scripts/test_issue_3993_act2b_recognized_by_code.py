"""#3993 (б), ревью PR #4121: на вопрос узнавания акта 2b отвечает код по снимку, и его фраза проходит судью шага.

Блокер ревью: матч «реплика целиком = глагол + дополнение» не срабатывал ни на одной реплике акта 2b, ради которого
заведена карточка, - они уходили модели, которая 23.10 сказала «Голос я пока не запомнил, но имя твоё знаю — Саша».
Здесь реплики берутся из САМОГО файла сценария (не копией), идут через ход v2 (``TurnEngine``) с вейком, снятым так же,
как ``address.decide`` (``strip_wake_word``), а сказанное кодом судит тот же ``e2e_tool_match.step_verdict``, что живой
харнесс и сухой стенд: вердикт биометрии шага + реплика ``tts_node``.

Run:
    python -m pytest tests/unit/e2e_scripts/test_issue_3993_act2b_recognized_by_code.py -v --no-cov
"""

from __future__ import annotations

import asyncio
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
ACT2B = ROOT / ".github" / "e2e" / "scenarios" / "night" / "night_marathon_act2b_identity_question_v1.json"
SCRIPTS = ROOT / ".github" / "workflows" / "scripts"

for _pkg in ("rob_box_core", "rob_box_dialog"):
    _path = str(ROOT / "src" / _pkg)
    if _path not in sys.path:
        sys.path.insert(0, _path)
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import e2e_tool_match as etm  # noqa: E402
from rob_box_dialog.address import VOICE, Addressed  # noqa: E402
from rob_box_dialog.session import Session  # noqa: E402
from rob_box_dialog.speaker_adapter import asks_recognized, speaker_ref  # noqa: E402
from rob_box_dialog.turn import TurnEngine  # noqa: E402
from rob_box_dialog.words import strip_wake_word  # noqa: E402

#: Шаги акта 2b, на которые отвечает код (вопрос узнавания), - остальные идут модели: память (n703, n707),
#: самопредставление (n701, n705, n709), выбор из имён «кто говорит, я или Борис?» (n708 - кодом не покрыт).
BY_CODE = ("n702_sasha_recognized_no_question", "n706_boris_recognized_no_question",
           "n710_grisha_not_named_by_voice")
#: n704 «ты ещё здесь, помнишь меня?» - с 28.10 вопрос к памяти (решение Шифу, вариант Б, #3993): отвечает модель.
BY_MODEL_REMEMBER = "n704_sasha_after_gap_no_question"
#: Снимок каждого шага, каким его ждёт акт: Саша и Борис записаны в акте 2 (имя подтверждено), тётя Гриша - профиль
#: без имени (#3858). Спор 23.10 (0.944, имя скрыто, run 40027008219) - отдельный случай ниже.
CONFIRMED = {"anton": {"is_known": True, "speaker_id": "429eae2c", "name": "Саша", "confidence": 0.934},
             "ermil": {"is_known": True, "speaker_id": "6eb1a5c6", "name": "Борис", "confidence": 0.951},
             "zahar": {"is_known": True, "speaker_id": "5e6f7a8b", "name": None, "epithet": "Соседка"}}
CONTESTED = {"is_known": True, "speaker_id": "429eae2c", "name": None, "confidence": 0.944, "epithet": "Спутник",
             "tentative_conf": 0.944, "tentative_kind": "contested"}


def _steps() -> dict:
    return {s["label"]: s for s in json.loads(ACT2B.read_text(encoding="utf-8"))["steps"]}


def _l(node: str, msg: str) -> str:
    return f"[{node}-8] [INFO] [1792791014.46] [{node}]: {msg}\n"


def _log(result: dict, said: str) -> str:
    """Вердикт биометрии (``speaker_id_node._verdict_line``: только опубликованное имя) + реплика ``tts_node``."""
    head = (_l("speaker_id_node", f"👤 Speaker: '{result['name']}' confidence={result['confidence']:.3f} (433 ms)")
            if result.get("name") else "")
    tts = (f"🔊 TTS: speech_id=78c32f4b, dialogue_id=None, batch=None None/None, voice=default, lang=default, "
           f"text={said!r}")
    return head + _l("tts_node", tts)


class _Ports:
    """Транспорт хода без модели: вызов LLM в этих шагах - провал теста."""

    def __init__(self):
        self.asked, self.said = [], []

    async def ask(self, messages, tools, timeout_s):
        self.asked.append(list(messages))
        raise AssertionError("модель не должна зваться на вопрос узнавания")

    async def execute(self, actions, snapshot, turn_id):
        return []

    def say(self, turn_id, epoch, text, voice=None, emotion=None):
        self.said.append(text)
        return f"sp{len(self.said)}"

    def stage(self, *args, **kwargs):
        pass

    def cancel_speech(self, epoch, reason, boundary="as_requested"):
        pass

    def stop_perform(self, how):
        return False

    def intent(self, turn_id, epoch, kind):
        pass

    def perform(self, turn_id, epoch, perf, edit):
        return None


def _code_turn(text: str, result: dict):
    session = Session("personality")
    session.set_speaker(speaker_ref(result, now=1.0))
    ports = _Ports()
    agg = asyncio.run(TurnEngine(session, ports, system_prompt="Ты робот.").run(
        Addressed(strip_wake_word(text), VOICE)))
    return agg, ports


def test_by_code_steps_are_exactly_the_recognition_questions_of_the_scenario():
    """Сверка с файлом сценария: правка текста шага, после которой код перестал (или начал) отвечать, видна здесь."""
    steps = _steps()
    assert set(BY_CODE) <= set(steps)
    assert {label for label, s in steps.items() if asks_recognized(strip_wake_word(s["text"]))} == set(BY_CODE)


def test_remember_me_step_goes_to_the_model_not_to_the_snapshot_3993():
    """Решение Шифу 28.10 (вариант Б): n704 «ты ещё здесь, помнишь меня?» - ход модели с памятью, код не отвечает."""
    step = _steps()[BY_MODEL_REMEMBER]
    assert "помнишь" in step["text"]
    assert not asks_recognized(strip_wake_word(step["text"]))


@pytest.mark.parametrize("label", BY_CODE)
def test_code_answer_passes_the_step_judge(label):
    """Снимок, которого ждёт акт: 0 вызовов LLM, а сказанное кодом проходит acceptance и patterns шага (имя есть,
    «не запомнил»/«не узнаю», вопроса личности и чужого имени нет)."""
    step = _steps()[label]
    result = CONFIRMED[step["voice"]]
    agg, ports = _code_turn(step["text"], result)
    assert ports.asked == [] and agg.llm_calls == 0 and len(ports.said) == 1
    log = _log(result, ports.said[0])
    verdict = etm.step_verdict(step.get("acceptance") or {}, log, label=label)
    assert verdict["pass"], (ports.said[0], verdict["reason"])
    assert etm.patterns_missing(log, step.get("patterns") or []) == []


@pytest.mark.parametrize("label", BY_CODE[:2])
def test_contested_voice_gets_no_lie_and_no_candidate_name(label):
    """Спор 23.10 (0.944, имя скрыто): код не говорит «не запомнил» и не называет кандидата; шаг всё равно красный -
    по вердикту биометрии (имя не опубликовано), а не по выдумке модели."""
    step = _steps()[label]
    agg, ports = _code_turn(step["text"], CONTESTED)
    said = ports.said[0]
    assert ports.asked == [] and agg.llm_calls == 0
    assert "не запомнил" not in said and "Саш" not in said and "Борис" not in said and "?" not in said
    verdict = etm.step_verdict(step.get("acceptance") or {}, _log(CONTESTED, said), label=label)
    assert verdict["pass"] is False  # expected_keywords: имени нет - честный красный шаг


def test_grisha_enrolled_from_this_very_phrase_is_not_told_her_voice_is_remembered():
    """Круг 1 ревью PR #4121 (блокер): если профиль тёти Гриши заведён из САМОЙ реплики n710 (речь ≥ 3 с, #3858
    ``source=enroll``), голос слышен впервые - код говорит «по голосу пока не узнаю», а не «голос твой помню»; судья
    шага (имена и факты Саши и Бориса не звучат) это пропускает."""
    label = BY_CODE[2]
    step = _steps()[label]
    enrolled = {**CONFIRMED[step["voice"]], "confidence": 0.0, "source": "enroll"}
    agg, ports = _code_turn(step["text"], enrolled)
    assert ports.asked == [] and agg.llm_calls == 0 and ports.said == ["По голосу пока не узнаю."]
    verdict = etm.step_verdict(step.get("acceptance") or {}, _log(enrolled, ports.said[0]), label=label)
    assert verdict["pass"], verdict["reason"]
