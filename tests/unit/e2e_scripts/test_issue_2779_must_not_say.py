# -*- coding: utf-8 -*-
"""Issue #2779 — ``must_not_say`` матчится по РЕЧИ робота, не по всему логу.

Зачем этот тест существует
==========================
Первая версия фикса #2779 клала имена/факты утёкшего диктора («Борис»,
«Спартак», «пицц») в ``must_not_call`` шага ``n210_grisha_no_name``
(PR #2789). Ревью на живых данных показало ту же болезнь, что #2764 уже
лечил для ``expected_keywords``, только в обратную сторону:

``must_not_call`` для НЕ-имени-тула (``tool_invoked()`` в
``e2e_tool_match.py``, строка ~122) падает в подстрочный поиск по ВСЕМУ
логу шага::

    if not TOOL_NAME_RE.match(frag_l):
        return frag_l in low            # весь лог, не только речь робота

А в окно шага ``n210`` ГАРАНТИРОВАННО попадают строки голосовой биометрии
(``speaker_id_node.py``), которые печатаются НЕЗАВИСИМО от того, что
ответил робот — это ровно строки из прогона 35734532425, по которому
заведена карточка #2779::

    🔍 identify candidates: best='Борис'(f499c2f7) score=0.688 | second='Саша'(a1b2c3d4) score=0.538 | gap=0.150
    👤 Speaker: unknown (1003 ms)

и, когда биометрия ДЕЙСТВИТЕЛЬНО опознаёт кого-то, ``dialogue_node``
добавляет технический префикс в ``user_input``::

    [Spkr:Борис] Робот, а про меня что помнишь?

Если положить «Борис» в ``must_not_call``, шаг ``n210`` становится
тавтологически КРАСНЫМ на КАЖДОМ прогоне — «Борис» всегда есть в логе
шага независимо от произнесённого. Это симметричная (наоборот) версия
issue #2764, где ``expected_keywords`` были тавтологически ЗЕЛЁНЫМИ.

Правильное поле — ``must_not_say`` (см. ``e2e_voice_test.sh:check_acceptance``,
раздел ``forbidden_said``): оно ищет ТОЛЬКО в ``robot_speech(logs)`` — том
же канале, что и ``expected_keywords`` (issue #2764) — и НЕ видит строки
биометрии/технические префиксы, только TTS text / ``speak_text`` /
``spoken=``.

Корпус логов ниже — синтетика в формате настоящих строк
``speaker_id_node.py``/``dialogue_node.py`` (не «похожего вида», а точная
копия f-строк: см. ``speaker_id_node.py:539/543/646-648`` и
``dialogue_node.py:_apply_speaker_identity`` — ``tag = f"[Spkr:{name}]"``).
"""
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = REPO_ROOT / ".github" / "workflows" / "scripts"
sys.path.insert(0, str(SCRIPTS))

from e2e_tool_match import keyword_hit, robot_speech  # noqa: E402


# Ровно случай из issue #2779: биометрия дважды подряд честно говорит
# unknown (speaker_id_node.py:543), но candidates-диагностика (:646-648)
# всё равно называет «Борис» лучшим (проигравшим порог) кандидатом — и
# ЭТО единственное место, где имя встречается. Робот отвечает по существу,
# не называя никого чужим именем (n210_grisha_no_name — желаемое поведение
# ПОСЛЕ фикса #2779).
CLEAN_LOG = (
    "[speaker_id_node-9] [INFO] [1.0] [speaker_id_node]: "
    "🔍 identify candidates: best='Борис'(f499c2f7) score=0.521 | "
    "second='Саша'(a1b2c3d4) score=0.405 | gap=0.117\n"
    "[speaker_id_node-9] [INFO] [1.1] [speaker_id_node]: "
    "👤 Speaker: unknown (424 ms)\n"
    "[dialogue_node-4] [INFO] [1.2] [dialogue_node]: "
    "user_input='[Speaker:unknown] Робот, я мимо шёл. Имя своё я тебе "
    "называть не буду.'\n"
    "[dialogue_node-4] [INFO] [1.3] [dialogue_node]: ✅ [turn] "
    "process_input returned: spoken='Понял, не буду настаивать. Если "
    "что-то понадобится — зови.'[:60] tools=[]\n"
)

# Тот же лог, но робот ДЕЙСТВИТЕЛЬНО адресовал ответ по чужому имени —
# регрессия, которую #2779 и должен ловить. Добавлена строка успешного
# опознания ПРЕДЫДУЩЕГО собеседника ([Spkr:Борис]) — она тоже не должна
# считаться, важна только строка spoken=.
LEAKED_LOG = (
    "[speaker_id_node-9] [INFO] [1.0] [speaker_id_node]: "
    "🔍 identify candidates: best='Борис'(f499c2f7) score=0.688 | "
    "second='Саша'(a1b2c3d4) score=0.538 | gap=0.150\n"
    "[speaker_id_node-9] [INFO] [1.1] [speaker_id_node]: "
    "👤 Speaker: unknown (1003 ms)\n"
    "[dialogue_node-4] [INFO] [1.2] [dialogue_node]: "
    "user_input='[Spkr:Борис] Робота, а про меня что помнишь?'\n"
    "[dialogue_node-4] [INFO] [1.3] [dialogue_node]: ✅ [turn] "
    "process_input returned: spoken='Борис, я тебя по голосу узнал. "
    "Болеешь за Спартак с девяносто восьмого, раз в неделю приносишь "
    "пиццу.'[:60] tools=[]\n"
)


def forbidden_said(logs: str, must_not_say):
    """Локальная копия ``forbidden_said`` из ``check_acceptance()``
    (``e2e_voice_test.sh``, раздел issue #2779). Синхронна с тем, что
    реально выполняется на роботе: ``[k for k in must_not_say if
    keyword_hit(logs, k)]``.
    """
    return [k for k in must_not_say if keyword_hit(logs, k)]


class TestMustNotSayScopedToRobotSpeech:
    """Ядро #2779: биометрия называет имя кандидата в диагностике — это
    НЕ считается «робот сказал»."""

    def test_biometry_only_mention_does_not_trigger(self):
        """«Борис» есть в identify candidates / Speaker: — но НЕ в речи.

        Именно это должно ПРОЙТИ (робот ничего не слил), а не покраснеть.
        """
        assert "борис" in CLEAN_LOG.lower()  # sanity: имя реально в логе
        assert keyword_hit(CLEAN_LOG, "Борис") is False
        assert forbidden_said(CLEAN_LOG, ["Борис", "Саш", "Спартак", "пицц"]) == []

    def test_actually_spoken_name_triggers(self):
        """Регрессия #2779: имя ПРОИЗНЕСЕНО (spoken=) → must_not_say ловит."""
        assert keyword_hit(LEAKED_LOG, "Борис") is True
        hits = forbidden_said(LEAKED_LOG, ["Борис", "Саш", "Спартак", "пицц"])
        assert "Борис" in hits
        assert "Спартак" in hits
        assert "пицц" in hits

    def test_speaker_prefix_tag_does_not_count_as_speech(self):
        """``[Spkr:Борис]`` — служебный маршрутный префикс user_input,
        не речь робота. Сам по себе (без spoken=) не должен триггерить.
        """
        log = (
            "[dialogue_node-4] [INFO] [1.0] [dialogue_node]: "
            "user_input='[Spkr:Борис] хочу продолжить разговор'\n"
            "[dialogue_node-4] [INFO] [1.1] [dialogue_node]: ✅ [turn] "
            "process_input returned: spoken='Конечно, продолжаем.'[:60] "
            "tools=[]\n"
        )
        assert keyword_hit(log, "Борис") is False

    def test_old_must_not_call_would_have_been_tautologically_red(self):
        """Документирует, ПОЧЕМУ ``must_not_call`` — неправильное поле:

        свободный (не-snake_case) текст в ``must_not_call`` матчит ВЕСЬ
        лог (см. ``tool_invoked`` fallback в e2e_tool_match.py) — «Борис»
        находится там, даже когда робот ничего подобного не говорил.
        """
        low = CLEAN_LOG.lower()
        # Это ``tool_invoked()``-fallback для не-tool-имени: обычный
        # substring по ВСЕМУ логу — ровно то, что ломало n210.
        whole_log_match = "борис" in low
        assert whole_log_match is True, (
            "sanity: если это False, значит корпус лога больше не "
            "воспроизводит инцидент — must_not_call и must_not_say "
            "перестанут отличаться в этом тесте"
        )
        # А в РЕЧИ робота (must_not_say) этого имени нет — контраст.
        assert keyword_hit(CLEAN_LOG, "Борис") is False


class TestRobotSpeechExtractionForBiometryLines:
    """robot_speech() не должен видеть строки identify candidates /
    Speaker: / user_input= — они не входят ни в один из трёх каналов
    ``_SPEECH_PATTERNS`` (TTS text= / speak_text параметры / spoken=)."""

    def test_clean_log_speech_has_no_biometry_leftovers(self):
        speech = robot_speech(CLEAN_LOG)
        assert "identify candidates" not in speech
        assert "Speaker: unknown" not in speech
        assert "[Speaker:unknown]" not in speech
        assert "Понял, не буду настаивать" in speech

    def test_leaked_log_speech_is_exactly_the_spoken_text(self):
        speech = robot_speech(LEAKED_LOG)
        assert "[Spkr:Борис]" not in speech
        assert "identify candidates" not in speech
        assert "Борис, я тебя по голосу узнал" in speech
