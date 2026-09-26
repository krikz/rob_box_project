# -*- coding: utf-8 -*-
"""ADR-0134 / Issue #2754 — ``must_not_identify_as`` ловит ``Speaker: 'NAME'``.

Зачем этот тест существует
==========================
Живой прогон 35734532425: робот опознал незнакомца (zahar/Гриша) как
«Бориса» (score=0.816) и произнёс ``Speaker: 'Борис'`` в логе
speaker_id_node — после чего LLM назвала незнакомца Борисом и
пересказала его факты. must_not_say в issue #2779 ловит ТОЛЬКО речь
робота (``robot_speech()``), а событие атрибуции в speaker_id_node
проходит мимо: нужно новое поле ``must_not_identify_as``, поиск
подстрокой ``Speaker: 'NAME'`` по ВСЕМУ логу шага (как ``grep -F``).

Этот тест гоняет ``forbidden_identified`` — локальную копию логики из
check_acceptance() (e2e_voice_test.sh, раздел ADR-0134 / issue #2754).
Локальная копия нужна по тем же причинам, что у test_issue_2779_must_not_say.py
(см. там ``forbidden_said``): bash-функцию check_acceptance() нельзя
вызвать из unit-теста без поднятия docker/SSH, поэтому мы тестируем
чистую substring-семантику здесь, а синхронизацию с bash-реализацией
обеспечивает её тривиальность: ``for name in must_not_identify_as:
needle = "Speaker: '%s'" % name; if needle in logs: ...``.

Корпус логов ниже — синтетика в формате настоящих строк
``speaker_id_node.py``/``dialogue_node.py`` (то же, что в
test_issue_2779_must_not_say.py).
"""
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = REPO_ROOT / ".github" / "workflows" / "scripts"
sys.path.insert(0, str(SCRIPTS))


def forbidden_identified(logs: str, must_not_identify_as):
    """Локальная копия ``forbidden_identified`` из ``check_acceptance()``.

    Реальный код в ``.github/workflows/scripts/e2e_voice_test.sh``::

        forbidden_identified = []
        for name in must_not_identify_as:
            needle = "Speaker: '%s'" % name
            if needle in logs:
                forbidden_identified.append(name)

    Семантика: «хотя бы одно вхождение подстроки ``Speaker: 'NAME'`` в
    любом месте лога шага» (поиск подстрокой == ``grep -F`` без хвоста).
    Локальная копия намеренно тривиальна, чтобы расхождение с bash было
    видно глазами.
    """
    out = []
    for name in must_not_identify_as:
        needle = "Speaker: '%s'" % name
        if needle in logs:
            out.append(name)
    return out


# Ровно случай из issue #2754: биометрия опознала незнакомца как «Борис»
# (Speaker: 'Борис' в логе), и LLM после этого назвала его по имени.
LEAKED_LOG = (
    "[speaker_id_node-9] [INFO] [1.0] [speaker_id_node]: "
    "🔍 identify candidates: best='Борис'(f499c2f7) score=0.816 | "
    "second='Саша'(a1b2c3d4) score=0.538 | gap=0.150\n"
    "[speaker_id_node-9] [INFO] [1.1] [speaker_id_node]: "
    "👤 Speaker: 'Борис' (424 ms)\n"
    "[dialogue_node-4] [INFO] [1.2] [dialogue_node]: "
    "✅ [turn] process_input returned: "
    "spoken='Да, Борис, ты сегодня с пиццей.'[:60] tools=[]\n"
)

# Тот же сценарий, но биометрия НЕ опознала: Speaker: unknown, и в
# candidates имя «Борис» — просто проигравший порог кандидат. Ровно
# то, что в must_not_say-тесте (issue #2779) уже проверяется для речи.
CLEAN_LOG = (
    "[speaker_id_node-9] [INFO] [1.0] [speaker_id_node]: "
    "🔍 identify candidates: best='Борис'(f499c2f7) score=0.521 | "
    "second='Саша'(a1b2c3d4) score=0.405 | gap=0.117\n"
    "[speaker_id_node-9] [INFO] [1.1] [speaker_id_node]: "
    "👤 Speaker: unknown (424 ms)\n"
    "[dialogue_node-4] [INFO] [1.2] [dialogue_node]: "
    "✅ [turn] process_input returned: "
    "spoken='Понял, не буду настаивать.'[:60] tools=[]\n"
)


class TestMustNotIdentifyAsBasics:
    """Ядро ADR-0134 / #2754: ``Speaker: 'NAME'`` в логе = FAIL."""

    def test_clean_log_unknown_passes(self):
        """Speaker: unknown → PASS. «Борис» в candidates — не событие
        атрибуции, это диагностическая строка (проигравший порог)."""
        assert "Speaker: unknown" in CLEAN_LOG  # sanity
        assert forbidden_identified(CLEAN_LOG, ["Борис", "Саша"]) == []

    def test_leaked_log_speaker_attributed_fails(self):
        """Регрессия #2754: Speaker: 'Борис' → must_not_identify_as ловит."""
        assert forbidden_identified(LEAKED_LOG, ["Борис", "Саша"]) == ["Борис"]

    def test_old_format_speaker_confidence_also_caught(self):
        """Старый формат ``Speaker: 'Борис' confidence=...`` ловится
        подстрокой — grep -F без хвоста берёт всё, что начинается на
        ``Speaker: 'Борис'``. Защита от рефакторинга формата лога."""
        log = (
            "[dialogue_node-9] [INFO] [1.0] [dialogue_node]: "
            "🪪 Speaker: 'Борис' confidence=0.816 gap=0.150 "
            "(старый формат dialogue_node)\n"
        )
        assert forbidden_identified(log, ["Борис"]) == ["Борис"]

    def test_partial_name_match_does_not_trigger(self):
        """``Speaker: 'Бориска'`` НЕ ловится при must_not_identify_as=['Борис'].

        Строгая семантика grep -F: needle = ``Speaker: 'Борис'`` (с
        закрывающим апострофом) ищется как подстрока; в ``Speaker: 'Бориска'``
        после «Борис» идёт «к», апострофа нет — вхождения нет. То же
        поведение даёт ``if needle in logs`` в check_acceptance().
        Проверяем здесь, чтобы случайно не сменить bash-реализацию
        на регэксп с якорями ``\\b`` и не разойтись.
        """
        log = (
            "[speaker_id_node-9] [INFO] [1.0] [speaker_id_node]: "
            "👤 Speaker: 'Бориска' (424 ms)\n"
        )
        assert forbidden_identified(log, ["Борис"]) == []

    def test_other_name_in_log_not_caught_when_not_listed(self):
        """«Саша» в логе не ловится, если в списке только «Борис»."""
        log = (
            "[speaker_id_node-9] [INFO] [1.0] [speaker_id_node]: "
            "👤 Speaker: 'Саша' (424 ms)\n"
        )
        assert forbidden_identified(log, ["Борис"]) == []

    def test_empty_list_is_noop(self):
        """must_not_identify_as=[] — пустой список, ничего не ловим."""
        assert forbidden_identified(LEAKED_LOG, []) == []

    def test_none_is_treated_as_empty(self):
        """acc.get('must_not_identify_as', []) or [] — None → []. Защита
        от regressions в парсере acceptance.json."""
        assert forbidden_identified(LEAKED_LOG, None or []) == []


class TestMustNotIdentifyAsVsMustNotSay:
    """Два поля ловят два разных слоя одной регрессии #2754:

    - must_not_say: «робот ПРОИЗНЁС» (robot_speech-scoped)
    - must_not_identify_as: «speaker_id_node атрибутировал» (substring
      по Speaker: 'NAME')

    В реальной регрессии 35734532425 сработали ОБА: биометрия
    атрибутировала И робот потом назвал по имени. Тест ниже — оба
    случая в одном логе, оба ассерта должны вернуть имя.
    """

    def test_both_layers_trigger_on_same_log(self):
        log = LEAKED_LOG
        # layer 1: must_not_identify_as
        ident = forbidden_identified(log, ["Борис", "Саша"])
        assert "Борис" in ident
        # layer 2: must_not_say (тут только контраст: имя реально в речи)
        from e2e_tool_match import keyword_hit

        assert keyword_hit(log, "Борис") is True
        # Симметрия must_not_say/identify_as: оба ловят «Борис» в
        # LEAKED_LOG, но РАЗНЫЕ строки (Speaker: vs spoken=).
        assert "Speaker: 'Борис'" in log
        assert "spoken='Да, Борис" in log

    def test_clean_log_passes_both_layers(self):
        """CLEAN_LOG: Speaker: unknown + spoken без имени чужого диктора —
        оба слоя молчат."""
        from e2e_tool_match import keyword_hit

        assert forbidden_identified(CLEAN_LOG, ["Борис", "Саша"]) == []
        assert keyword_hit(CLEAN_LOG, "Борис") is False
        assert keyword_hit(CLEAN_LOG, "Саша") is False
