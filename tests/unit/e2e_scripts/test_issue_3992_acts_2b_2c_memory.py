"""#3992, #3993 — рукописные акты 2b/2c ночного марафона: память дикторов на весь марафон, без вопроса личности.

Решения Шифу 26.10: переспрос «ты Саша?» не делаем; голоса ролей разведены кастом #4038. С #3980/#3972 e2e-БД
дикторов держится весь марафон (Саша и Борис записаны в акте 2), с #4048 спорный голос не получает ни имени, ни
двойника. Старые 2b/2c были написаны под пустую БД и под вопрос личности: склейку знакомого голоса харнесс красил
(run 40027008219, 40028174075), а принудительный band_high=0.99 сам делал contested.

Шаги судит тот же код, что живой харнесс и сухой стенд: ``e2e_tool_match.step_verdict`` (acceptance) и
``patterns_missing`` (patterns). Строки лога — f-строки ``speaker_id_node.py`` (сверка с исходником ниже) и
строки живого лога 23.10 (``va_marathon.log`` марафона 210648). Кто «представляется» в реплике, решает код диалога
(``rob_box_dialog.words``), а не регэксп теста.

Run:
    python -m pytest tests/unit/e2e_scripts/test_issue_3992_acts_2b_2c_memory.py -v --no-cov
"""

from __future__ import annotations

import ast
import importlib.util
import json
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
NIGHT = ROOT / ".github" / "e2e" / "scenarios" / "night"
SCRIPTS = ROOT / ".github" / "workflows" / "scripts"
GEN = ROOT / "scripts" / "e2e" / "gen_night_marathon.py"
SPEAKER_ID_NODE = ROOT / "src" / "rob_box_voice" / "rob_box_voice" / "speaker_id_node.py"

ACT2 = "night_marathon_act2_acquaintance_v1.json"
ACT2B = "night_marathon_act2b_identity_question_v1.json"
ACT2C = "night_marathon_act2c_voice_conflict_question_v1.json"
LATER_ACTS = ("night_marathon_act3_backlog_diarization_v1.json", "night_marathon_act10_finale_memory_v1.json")

for _pkg in ("rob_box_core", "rob_box_dialog"):
    _path = str(ROOT / "src" / _pkg)
    if _path not in sys.path:
        sys.path.insert(0, _path)
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import e2e_tool_match as etm  # noqa: E402
from rob_box_dialog.words import introduced_name, introduces_self, name_tokens  # noqa: E402

#: Голоса, которые в актах 3 и 10 обязаны остаться незнакомцами (тётя Гриша, Валера): в 2b/2c не представляются.
STRANGER_VOICES = ("zahar", "filipp", "kostya")
#: Голос → имя по акту 2: ровно эти профили 2b узнаёт.
CAST = {"anton": "Саша", "ermil": "Борис"}
#: Формы ВОПРОСА личности (решение Шифу 26.10: не задаём). Утвердительное «да, это ты, Саша» — ответ, не вопрос.
QUESTION_FORMS = ("это ты?", "как тебя зовут")


def _act(name: str) -> dict:
    return json.loads((NIGHT / name).read_text(encoding="utf-8"))


def _steps(name: str) -> dict:
    return {s["label"]: s for s in _act(name)["steps"]}


def _gen():
    sys.path.insert(0, str(GEN.parent))
    spec = importlib.util.spec_from_file_location("gen_night_marathon_3992", GEN)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ── строки лога voice-assistant (формат docker logs) ─────────────────────────


def _l(node: str, msg: str, level: str = "INFO") -> str:
    return f"[{node}-8] [{level}] [1792791014.46] [{node}]: {msg}\n"


def _verdict(name: str, conf: float) -> str:
    # speaker_id_node._verdict_line, уверенное имя (= опубликовано, name_state confirmed)
    return _l("speaker_id_node", f"👤 Speaker: '{name}' confidence={conf:.3f} (433 ms)")


def _tts(text: str) -> str:
    return _l(
        "tts_node",
        "🔊 TTS: speech_id=78c32f4b, dialogue_id=None, batch=None None/None, voice=default, "
        f"lang=default, text={text!r}",
    )


def _register_call(name: str) -> str:
    return (
        _l("dialog_node", "📤 Отправлен запрос 4fd64941: register_speaker")
        + _l("mcp_server", f"📥 Запрос выполнения: register_speaker с параметрами {{'name': '{name}'}}")
        + _l("mcp_server", "✅ Инструмент register_speaker выполнен успешно")
    )


#: Живой лог 23.10 21:30:14 (n701, band_high=0.99), вердикт в формате после #3858/#4048: имя не опубликовано.
CONTESTED_SASHA = (
    _l(
        "speaker_id_node",
        "🔍 identify candidates: best='Саша'(429eae2c) score=0.944 | second='Борис'(6eb1a5c6) "
        "score=0.661 | gap=0.283 | речь=5.97s",
    )
    + _l(
        "speaker_id_node",
        "👤 Speaker: unknown (433 ms) -- имя не подтверждено (contested), ближайший кандидат "
        "'Саша' confidence=0.944",
    )
    + _l(
        "speaker_id_node",
        "📢 Publishing: is_known=true name=None epithet='Спутник' conf=0.944 "
        "(name suppressed: contested, issue #2809)",
    )
)
MERGED_SASHA = _l(
    "speaker_id_node",
    "🔗 Speaker 'Саша' merged into existing profile (id=429eae2c) — voice matched an "
    "already-known speaker, no duplicate created",
)
NINA_NEW_NAMELESS = _l(
    "speaker_id_node", "👤 Speaker: безымянный профиль id=5e6f7a8b (новый, кличка 'Соседка') (412 ms)"
)


def _claimed_unnamed(name: str, sid8: str = "5e6f7a8b") -> str:
    """speaker_id_node._do_register, ветка outcome.claimed_unnamed (#3858): имя присвоено безымянному профилю."""
    return _l(
        "speaker_id_node",
        f"🏷️ [issue #3858] '{name}' представился — имя присвоено профилю без имени "
        f"(id={sid8}), дубля нет, кличка и накопленное остались",
    )


NINA_CLAIMED = _claimed_unnamed("Нина")
NINA_REGISTERED = _l("speaker_id_node", "✅ Speaker 'Нина' registered (id=5e6f7a8b)")


def _judge(step: dict, log: str) -> tuple:
    """(вердикт acceptance, ненайденные паттерны) — те же функции, что у харнесса и сухого стенда."""
    return etm.step_verdict(step.get("acceptance") or {}, log, label=step["label"]), etm.patterns_missing(
        log, step.get("patterns") or []
    )


# ── контракт: число шагов, без машинерии вопроса личности ───────────────────


@pytest.mark.parametrize("act_file, key", [(ACT2B, "2b"), (ACT2C, "2c")])
def test_step_count_matches_manual_acts_of_generator(act_file, key):
    """Генератор и манифест правит другой воркер: число шагов рукописного акта обязано совпасть с MANUAL_ACTS."""
    manual = {m["act"]: m for m in _gen().MANUAL_ACTS}
    assert len(_act(act_file)["steps"]) == manual[key]["steps"]


@pytest.mark.parametrize("act_file", [ACT2B, ACT2C])
def test_no_identity_question_machinery(act_file):
    """Шифу 26.10: вопроса личности нет — ни шагов-ответов на него, ни форсированных порогов биометрии."""
    act = _act(act_file)
    assert "node_params" not in act, "полоса/порог биометрии штатные: band_high=0.99 сам делал contested (#3993)"
    for s in act["steps"]:
        assert not s.get("when_robot_asked") and not s.get("required_question"), s["label"]


@pytest.mark.parametrize("act_file", [ACT2B, ACT2C])
def test_scenario_text_keeps_speaker_db_isolation_anchor(act_file):
    """Отдельный прогон акта обязан идти по e2e-БД дикторов: якорь харнесса — слово register_speaker в сценарии."""
    raw = (NIGHT / act_file).read_text(encoding="utf-8")
    assert "register_speaker" in raw
    assert "memory_save" not in raw


def test_known_voices_are_judged_by_confirmed_verdict_and_never_asked():
    for label, step in _steps(ACT2B).items():
        name = CAST.get(step["voice"])
        if name is None:
            continue
        acc = step["acceptance"]
        assert f"👤 Speaker: '{name}' confidence=(0\\.[89]|1\\.)" in step["patterns"], label
        for form in QUESTION_FORMS:
            assert form in acc["must_not_say"], (label, form)
        assert set(acc["must_not_identify_as"]) == set(CAST.values()) - {name}, label
        # повторная регистрация знакомого голоса (склейка, «уже знаю») — норма, не FAIL
        assert not etm.registration_expected(acc), label
        assert etm.REGISTER_TOOL not in acc["must_not_call"], label


# ── кто представляется: решает код диалога ──────────────────────────────────


def test_strangers_of_acts_3_and_10_never_introduce_themselves():
    """Гриша и Валера — незнакомцы до акта 10 (n306, n1002): в 2b/2c их голоса не называются и не регистрируются."""
    for act_file in (ACT2, ACT2B, ACT2C):
        for s in _act(act_file)["steps"]:
            if s["voice"] in STRANGER_VOICES:
                assert not introduces_self(s["text"]), (act_file, s["label"])
                assert etm.REGISTER_TOOL not in (s.get("acceptance") or {}).get("expected_tool_calls", []), s["label"]


def test_one_name_per_voice_across_acts_2_2b():
    """Голос → имя как в акте 2: Саша — anton, Борис — ermil; других имён эти голоса не называют."""
    names = {}
    for act_file in (ACT2, ACT2B):
        for s in _act(act_file)["steps"]:
            name = introduced_name(name_tokens(s["text"]))
            if name:
                names.setdefault(s["voice"], set()).add(name)
    assert names == {"anton": {"саша"}, "ermil": {"борис"}}


def test_2c_second_name_is_the_only_one_voice_two_names_step():
    named = [(s["label"], introduced_name(name_tokens(s["text"]))) for s in _act(ACT2C)["steps"]]
    assert named == [
        ("n721_nina_own_profile", "нина"),
        ("n722_nina_second_name_no_duplicate", "тамара"),
        ("n723_nina_one_name", None),
    ]
    assert {s["voice"] for s in _act(ACT2C)["steps"]} == {"arina"}, "голос Нины больше нигде не звучит"
    for later in LATER_ACTS:
        assert "arina" not in {s["voice"] for s in _act(later)["steps"]}


def test_female_voices_speak_in_feminine():
    """Каст #4038: Гриша и Нина — женские голоса; «я заскочил» → «я заскочила» (n709)."""
    masculine_past = re.compile(r"\bя\s+(?:\w+\s+)?\w+л(?:ся)?\b", re.IGNORECASE)
    for act_file in (ACT2B, ACT2C):
        for s in _act(act_file)["steps"]:
            if s["voice"] in ("zahar", "filipp", "arina"):
                assert not masculine_past.search(s["text"]), (s["label"], s["text"])
    assert "заскочила" in _steps(ACT2B)["n709_grisha_drop_in"]["text"]


# ── 2b: поведение судьи на логах ────────────────────────────────────────────


def test_n701_merge_into_act2_profile_is_green():
    """23.10 n701: Саша узнан, register_speaker склеил с профилем акта 2 — харнесс красил это. Теперь норма."""
    step = _steps(ACT2B)["n701_sasha_back_recognized"]
    log = _verdict("Саша", 0.944) + _register_call("Саша") + MERGED_SASHA + _tts("Запомнил тебя, Саша.")
    verdict, missing = _judge(step, log)
    assert verdict["pass"], verdict["reason"]
    assert missing == []


def test_n701_contested_name_is_red():
    """Имя скрыто (contested, как 23.10 при band_high=0.99): вердикт unknown — паттерна confirmed нет."""
    step = _steps(ACT2B)["n701_sasha_back_recognized"]
    _, missing = _judge(step, CONTESTED_SASHA + _register_call("Саша") + MERGED_SASHA)
    assert missing == step["patterns"]


def test_confirmed_pattern_needs_confidence_at_least_0_8():
    pattern = _steps(ACT2B)["n702_sasha_recognized_no_question"]["patterns"]
    assert etm.patterns_missing(_verdict("Саша", 0.81), pattern) == []
    assert etm.patterns_missing(_verdict("Саша", 1.0), pattern) == []
    assert etm.patterns_missing(_verdict("Саша", 0.76), pattern) == pattern


def test_n704_phrase_is_long_enough_for_a_confident_verdict():
    """n704 идёт после паузы 150 с с порогом confidence >= 0.8: 34 символа (~2 с речи) — мало для уверенного вердикта.

    Нижняя граница 50 символов — как у соседних реплик Саши (n702: 62); верхняя — бюджет VAD-окна 15 с (<= 96 символов).
    Реплика обращена к роботу, имени не называет и тот же смысл («ты ещё здесь, помнишь меня?»).
    """
    step = _steps(ACT2B)["n704_sasha_after_gap_no_question"]
    text = step["text"]
    assert 50 <= len(text) <= 96, len(text)
    assert text.startswith("Робот,") and "помнишь меня" in text
    assert not introduces_self(text) and introduced_name(name_tokens(text)) is None
    assert step["sleep_before_sec"] == 150


@pytest.mark.parametrize(
    "speech, ok",
    [
        ("Да, Саша, это ты — узнал по голосу, ты чинишь технику по вечерам.", True),
        ("Саша, это ты?", False),
        ("Как тебя зовут?", False),
        ("Голос я пока не запомнил, но имя твоё знаю — Саша.", False),  # 23.10 21:30:40, голос при этом узнан
    ],
)
def test_n702_answer_without_identity_question(speech, ok):
    step = _steps(ACT2B)["n702_sasha_recognized_no_question"]
    verdict, missing = _judge(step, _verdict("Саша", 0.934) + _tts(speech))
    assert verdict["pass"] is ok, verdict["reason"]
    assert missing == []


def test_n703_fact_of_act2_from_memory_and_no_foreign_fact():
    step = _steps(ACT2B)["n703_sasha_memory_from_act2"]
    good = _verdict("Саша", 0.92) + _tts("Помню, Саша: ты пьёшь зелёный чай без сахара и не ешь лук.")
    assert _judge(step, good)[0]["pass"]
    leak = _verdict("Саша", 0.92) + _tts("Зелёный чай без сахара, и ты болеешь за Спартак.")
    assert not _judge(step, leak)[0]["pass"]


def test_n708_boris_verdict_on_sasha_phrase_is_red():
    step = _steps(ACT2B)["n708_sasha_not_boris"]
    assert _judge(step, _verdict("Саша", 0.9) + _tts("Сейчас говоришь ты, Саша, а не Борис."))[0]["pass"]
    verdict, missing = _judge(step, _verdict("Борис", 0.82) + _tts("Это Саша."))
    assert not verdict["pass"] and "Борис" in verdict["reason"]
    assert missing == step["patterns"]


@pytest.mark.parametrize("label", ["n709_grisha_drop_in", "n710_grisha_not_named_by_voice"])
def test_stranger_verdict(label):
    """Безымянный профиль Гриши (#3858) — не чьё-то имя; вердикт 'Саша' — путаница голосов."""
    step = _steps(ACT2B)[label]
    nameless = _l(
        "speaker_id_node", "👤 Speaker: безымянный профиль id=1a2b3c4d (кличка 'Ворчунья') " "confidence=0.912 (380 ms)"
    )
    assert _judge(step, nameless + _tts("Хорошо, заходи, когда будет время."))[0]["pass"]
    assert not _judge(step, _verdict("Саша", 0.837) + _tts("Хорошо, заходи."))[0]["pass"]


# ── 2c: свой профиль, второго нет ────────────────────────────────────────────


@pytest.mark.parametrize(
    "outcome",
    [NINA_NEW_NAMELESS + NINA_CLAIMED, NINA_REGISTERED],
    ids=["claimed_own_nameless_3858", "brand_new_profile"],
)
def test_n721_own_profile_is_green(outcome):
    step = _steps(ACT2C)["n721_nina_own_profile"]
    verdict, missing = _judge(step, _register_call("Нина") + outcome + _tts("Приятно познакомиться, Нина."))
    assert verdict["pass"], verdict["reason"]
    assert missing == []


def test_n721_taking_over_someone_elses_nameless_profile_is_red():
    """Голос Нины опознан старым безымянным профилем (Гриши, >= 0.72) — имя досталось чужому профилю."""
    step = _steps(ACT2C)["n721_nina_own_profile"]
    grisha = _l(
        "speaker_id_node", "👤 Speaker: безымянный профиль id=1a2b3c4d (кличка 'Ворчунья') " "confidence=0.731 (380 ms)"
    )
    claimed = NINA_CLAIMED.replace("5e6f7a8b", "1a2b3c4d")
    _, missing = _judge(step, grisha + _register_call("Нина") + claimed)
    assert missing == [step["patterns"][1]]


def test_n721_conflict_with_known_voice_is_red():
    """ADR-0127: голос похож на Сашу — отдельный профиль с флагом конфликта. Для «другого регистра» это провал."""
    step = _steps(ACT2C)["n721_nina_own_profile"]
    conflict = _l(
        "speaker_id_node",
        "⚠️ Speaker 'Нина' (id=77aa88bb) — голос похож на уже известного 'Саша' "
        "(id=429eae2c, score=0.781 >= порога слияния), но имя другое: завожу ОТДЕЛЬНЫЙ "
        "профиль и НЕ переименовываю чужой (ADR-0127).",
        level="WARN",
    )
    _, missing = _judge(step, _register_call("Нина") + conflict)
    assert step["patterns"][0] in missing


def test_n722_second_profile_on_same_voice_is_red():
    step = _steps(ACT2C)["n722_nina_second_name_no_duplicate"]
    dup = _l(
        "speaker_id_node",
        "⚠️ Speaker 'Тамара' (id=99cc00dd) — голос похож на уже известного 'Нина' "
        "(id=5e6f7a8b, score=0.902 >= порога слияния), но имя другое: завожу ОТДЕЛЬНЫЙ "
        "профиль и НЕ переименовываю чужой (ADR-0127).",
        level="WARN",
    )
    verdict, _ = _judge(step, _verdict("Нина", 0.9) + _register_call("Тамара") + dup + _tts("Запомнил тебя, Тамара."))
    assert not verdict["pass"]
    assert "register_speaker" in verdict["reason"]
    one_voice = _verdict("Нина", 0.9) + _tts(
        "Голос-то у тебя Нинин. Хочешь, чтобы звал Тамарой, скажи: зови меня Тамара."
    )
    assert _judge(step, one_voice)[0]["pass"]


def test_n722_is_known_red_until_3801_is_ported_to_v2():
    """Красный ожидается ПО КОДУ и вживую не проверен (LLM может и не вызвать register_speaker) — так и написано;
    карточка порта правила — #4054, до удаления v1 (#3711)."""
    known_red = _steps(ACT2C)["n722_nina_second_name_no_duplicate"]["_known_red"]
    assert "#3801" in known_red and "#3711" in known_red
    assert "#4054" in known_red
    assert "ожидаемо красный (по коду, не проверено" in known_red
    assert "красный по делу" not in known_red, "'по делу' — утверждение о проверенном; живого прогона нет"
    assert "карточки на порт нет" not in known_red, "карточка есть: #4054"


@pytest.mark.parametrize("label, name", [("n201_sasha_intro_long", "Саша"), ("n204_boris_intro_long", "Борис")])
def test_act2_introduction_claiming_nameless_profile_is_confirmed(label, name):
    """#3858, unnamed_profiles_enabled=true: первая фраза нового голоса >= 3 с завела безымянный профиль, представление
    присвоило имя ЕМУ — строки «✅ … registered» нет. Харнесс называл это «NOT confirmed», хотя робот прав
    (при STOP_ON_FOUNDATION_FAIL=1 раннер оборвал бы марафон на акте 2)."""
    acc = _steps(ACT2)[label]["acceptance"]
    assert etm.registration_expected(acc), "шаг обязан доказывать регистрацию: флага require_registration_accepted нет"
    log = _register_call(name) + _claimed_unnamed(name) + _tts(f"Приятно познакомиться, {name}.")
    verdict = etm.step_verdict(acc, log, label=label)
    assert verdict["pass"], verdict["reason"]
    assert verdict["registration"]["accepted"] == [name]
    # без строки присвоения (и без «registered») регистрация не доказана — красный, как и был
    bare = etm.step_verdict(acc, _register_call(name) + _tts("Приятно познакомиться."), label=label)
    assert not bare["pass"] and "NOT confirmed" in bare["reason"]


def test_n723_one_name_per_voice():
    step = _steps(ACT2C)["n723_nina_one_name"]
    verdict, missing = _judge(step, _verdict("Нина", 0.91) + _tts("Тебя зовут Нина, соседка снизу."))
    assert verdict["pass"] and missing == []
    verdict, missing = _judge(step, _verdict("Тамара", 0.93) + _tts("Тебя зовут Тамара."))
    assert not verdict["pass"] and missing == step["patterns"]


# ── строки лога, на которые опираются паттерны, есть в коде робота ───────────


def _logged_by_branch(branch_test: str, placeholders: dict) -> str:
    """Сообщение, которое speaker_id_node логирует в ветке ``if <branch_test>:`` — собрано из f-строки ИСХОДНИКА.

    Подстановки — по тексту выражения (``name``, ``sid[:8]``); неизвестная подстановка — KeyError, а не пустое место.
    """
    source = SPEAKER_ID_NODE.read_text(encoding="utf-8")
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.If) and ast.get_source_segment(source, node.test) == branch_test:
            message = node.body[0].value.args[0]  # self.get_logger().info(f"…")
            return "".join(
                (
                    placeholders[ast.get_source_segment(source, part.value)]
                    if isinstance(part, ast.FormattedValue)
                    else part.value
                )
                for part in message.values
            )
    raise AssertionError(f"ветка `if {branch_test}:` не найдена в {SPEAKER_ID_NODE}")


def test_claimed_unnamed_fixture_is_the_line_speaker_id_node_logs():
    """Фикстура _claimed_unnamed — не «похожая» строка, а f-строка ветки outcome.claimed_unnamed из исходника,
    и ровно её харнесс обязан принимать как регистрацию (#3858)."""
    real = _logged_by_branch("outcome.claimed_unnamed", {"sid[:8]": "5e6f7a8b", "name": "Нина"})
    assert _claimed_unnamed("Нина") == _l("speaker_id_node", real)
    outcome = etm.registration_outcome(_l("speaker_id_node", real))
    assert outcome == {"accepted": ["Нина"], "merged": [], "rejected": []}


def test_log_formats_exist_in_speaker_id_node():
    src = SPEAKER_ID_NODE.read_text(encoding="utf-8")
    assert "f\"👤 Speaker: '{match.name}' confidence={match.confidence:.3f} \"" in src
    assert 'f"👤 Speaker: безымянный профиль id={enrolled.speaker_id[:8]} (новый, кличка {enrolled.epithet!r})' in src
    assert "' представился — имя присвоено профилю без имени (id={sid[:8]})" in src
    assert "f\"✅ Speaker '{name}' registered (id={sid[:8]})\"" in src
    assert "merged into existing profile (id={sid[:8]}) " in src
    assert "голос похож на уже " in src
