# -*- coding: utf-8 -*-
"""Issue #2764 — ``expected_keywords`` матчатся по РЕЧИ РОБОТА, не по логу.

Зачем этот тест существует
==========================
``check_acceptance`` в ``e2e_voice_test.sh`` искал ключевые слова подстрокой
по всему ``docker logs voice-assistant --since <шаг>``. В этот лог попадает
не только ответ робота, но и реплика говорящего плюс подпись диктора,
которую ставит ``speaker_id_node``::

    dialogue_node: user_input='[Spkr:Саша] привет, давай знакомиться...'
    dialogue_node: 👤 [issue 1077] Speaker: 'Саша' conf=0.81

Из-за этого три из четырёх keyword-проверок акта 2 ночного марафона были
тавтологиями — зелёными независимо от того, что робот ответил:

* ``n207_recall_sasha``    KW=['Саш']                говорит Саша → ключ в логе всегда
* ``n209_recall_boris``    KW=['Борис|Спартак|пицц'] говорит Борис → ключ в логе всегда
* ``n211_who_do_you_know`` KW=['Саш', 'Борис']       половина ключа бесплатная

Корпус строк ниже снят с живого робота (Vision Pi, 22.09.2026) — формат
логов тут контракт, поэтому в тестах именно настоящие строки, а не
синтетика «похожего вида».
"""
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = REPO_ROOT / ".github" / "workflows" / "scripts"
sys.path.insert(0, str(SCRIPTS))

from e2e_tool_match import keyword_hit, robot_speech  # noqa: E402


# Живой фрагмент лога: робот поздоровался по лицу, Саша ответил, робот
# переспросил имя. Имя «Саша» есть В ЛОГЕ, но робот его НЕ ПРОИЗНОСИЛ.
LIVE_LOG = (
    "[tts_node-5] [INFO] [1790073470.711094079] [tts_node]: 🔊 TTS: "
    "speech_id=3f15b0f0, dialogue_id=None, batch=None None/None, "
    "voice=default, lang=default, text='Добрый день, Денис!'\n"
    "[dialogue_node-4] [INFO] [1790073471.0] [dialogue_node]: "
    "user_input='[Spkr:Саша] привет, давай знакомиться как следует'\n"
    "[speaker_id_node-9] [INFO] [1790073471.2] [speaker_id_node]: "
    "👤 [issue 1077] Speaker: 'Саша' conf=0.81\n"
    "[mcp_server-10] [INFO] [1790073471.5] [mcp_server]: 📥 Запрос выполнения: "
    "speak_text с параметрами {'text': 'Лицо знакомое, но я тебя по голосу "
    "пока не узнаю — представься, как тебя зовут?', 'animation': 'happy'}\n"
    "[dialogue_node-4] [INFO] [1790073502.4] [dialogue_node]: ✅ [turn] "
    "process_input returned: spoken='Привет! У меня всё отлично'[:60] tools=[]\n"
)


class TestRobotSpeech:
    def test_extracts_all_three_channels(self):
        speech = robot_speech(LIVE_LOG)
        assert "Добрый день, Денис!" in speech          # 🔊 TTS: ... text='...'
        assert "Лицо знакомое" in speech                # speak_text с параметрами
        assert "Привет! У меня всё отлично" in speech   # spoken='...'[:60]

    def test_excludes_user_input_and_speaker_label(self):
        speech = robot_speech(robot_speech_input := LIVE_LOG)
        assert "[Spkr:" not in speech
        assert "давай знакомиться" not in speech
        assert robot_speech_input != speech

    @pytest.mark.parametrize("logs", ["", None])
    def test_empty_logs_give_empty_speech(self, logs):
        assert robot_speech(logs) == ""


class TestKeywordHit:
    def test_name_only_in_speaker_label_does_not_count(self):
        """Ядро #2764: «Саш» есть в логе, но робот его не произносил."""
        assert keyword_hit(LIVE_LOG, "Саш") is False

    def test_name_actually_spoken_counts(self):
        assert keyword_hit(LIVE_LOG, "Денис") is True

    def test_alternation_any_of(self):
        """#2753: «а|б|в» — это ИЛИ, а не поиск строки вместе с палками."""
        assert keyword_hit(LIVE_LOG, "Борис|Спартак|знакомое") is True
        assert keyword_hit(LIVE_LOG, "Борис|Спартак|пицц") is False

    def test_case_insensitive(self):
        assert keyword_hit(LIVE_LOG, "ДЕНИС") is True

    def test_silent_robot_fails_keywords(self):
        """Молчащий робот не должен проходить keyword-проверку.

        Обратная сторона фикса: если речи в окне нет вообще, ключ не
        найден — это валидный красный, а не сбой парсера.
        """
        silent = (
            "[dialogue_node-4] [INFO] [1.0] [dialogue_node]: "
            "user_input='[Spkr:Саша] как меня зовут?'\n"
            "[dialogue_node-4] [WARN] [1.1] [dialogue_node]: 🤐 [issue 1882] "
            "planning-narration hard-mute: speaking nothing\n"
        )
        assert robot_speech(silent) == ""
        assert keyword_hit(silent, "Саш") is False

    def test_blank_alternatives_ignored(self):
        assert keyword_hit(LIVE_LOG, "|Денис|") is True
        assert keyword_hit(LIVE_LOG, "||") is False
