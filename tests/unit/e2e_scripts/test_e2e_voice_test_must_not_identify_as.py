# -*- coding: utf-8 -*-
"""ADR-0134 §2.1 — ``identify_failures`` (парсер «незнакомец НЕ опознан как известный»).

Зачем эти тесты существуют
==========================
Issue #2754 / ADR-0134 §2.1: на шаге ``n210_grisha_no_name`` робот
опознаёт незнакомца как «Борис» (score=0.816 > порога), но шаг
остаётся зелёным — потому что проверка ``must_not_call: register_speaker``
не ловит «identify выдал Бориса». Это симметричная дыра к issue #2779
(``must_not_say``): оба поля проверяли разные инварианты, но оба
молча пропускали случай «голосовая биометрия утверждает, что это Борис».

Парсер ``identify_failures`` (см. ``e2e_tool_match.py:545``) сканирует
лог шага на строки вида::

    👤 Speaker: 'Борис'      # финальный вердикт (identify успешно опознал)
    👤 Speaker: 'unknown'    # отказ (identify не прошёл порог)

Контракт ``acc``::

    {"must_not_identify_as": ["Борис", "Саша"]}

Если хоть одно из имён попало в финальный вердикт — функция возвращает
список строк-причин (по одной на каждое запрещённое имя). Пустой
список = инвариант выполнен. Этот список потом подмешивается в общий
``failures`` шага в ``check_acceptance`` (bash-блок).

Тесты ниже — единственный гарант того, что:
1. Парсер ловит «Борис» в финальном вердикте (не только в candidates).
2. Парсер НЕ ловит, если «Борис» только в candidates-диагностике
   (это симметрия #2779).
3. Парсер работает с многоэлементными списками.
4. Парсер устойчив к старому формату лога (без эмодзи 👤 — некоторые
   старые харнессы не передают эмодзи через stdout-stderr канал).
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

# __file__ = tests/unit/e2e_scripts/test_*.py → parents[3] = repo root
# (parents[0]=e2e_scripts, [1]=unit, [2]=tests, [3]=<repo>).
# Раньше было parents[2] — это указывало на tests/ и sys.path вставлял
# tests/.github/... которого не существует. Шифу в ретро не указывал
# на это явно, но модуль e2e_tool_match.py не находится → ImportError.
REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = REPO_ROOT / ".github" / "workflows" / "scripts"
sys.path.insert(0, str(SCRIPTS))

from e2e_tool_match import identify_failures  # noqa: E402


# ── корпус логов ────────────────────────────────────────────────────────────
#
# Строки — точные копии f-строк из ``speaker_id_node.py:539/543/646-648``,
# обёрнутые в ROS-лог-префикс (как их печатает ``docker logs voice-assistant``).
# Не «похожего вида», а реальный формат — иначе парсер может промахнуться.

def _l(node: str, level: str, msg: str) -> str:
    return f"[{node}-9] [{level}] [1758638000.1] [{node}]: {msg}\n"


# ДИАГНОСТИКА identify (Борис — лучший кандидат), но финальный вердикт —
# unknown (порог не пройден). Это ровно случай из #2779, симметричный
# к #2754. Здесь парсер НЕ должен срабатывать на «Борис» в candidates.
CLEAN_DIAGNOSTIC_LOG = (
    _l("speaker_id_node", "INFO",
       "🔍 identify candidates: best='Борис'(f499c2f7) score=0.521 | "
       "second='Саша'(a1b2c3d4) score=0.405 | gap=0.117")
    + _l("speaker_id_node", "INFO", "👤 Speaker: unknown (424 ms)")
    + _l("dialogue_node", "INFO",
         "✅ [turn] process_input returned: spoken='Понял, не буду настаивать.'[:60] "
         "tools=[]")
)

# Финальный вердикт «Борис» — порог пройден, identify реально опознал.
# Это ровно случай из #2754: незнакомец опознан как Борис, шаг должен
# упасть, потому что ``must_not_identify_as: ['Борис']``.
BORIS_VERDICT_LOG = (
    _l("speaker_id_node", "INFO",
       "🔍 identify candidates: best='Борис'(f499c2f7) score=0.816 | "
       "second='Саша'(a1b2c3d4) score=0.502 | gap=0.314")
    + _l("speaker_id_node", "INFO", "👤 Speaker: 'Борис' (812 ms)")
    + _l("dialogue_node", "INFO",
         "✅ [turn] process_input returned: spoken='Борис, привет!'[:60] "
         "tools=[]")
)

# То же, что BORIS_VERDICT_LOG, плюс второй финальный вердикт «Саша»
# (retried шаг — две попытки, обе упали по identify). Любой из forbidden
# в verdicts → провисает инвариант. Это тест на multi-attempt.
BORIS_AND_SASHA_VERDICTS = (
    _l("speaker_id_node", "INFO",
       "🔍 identify candidates: best='Борис'(f499c2f7) score=0.816 | "
       "second='Саша'(a1b2c3d4) score=0.502 | gap=0.314")
    + _l("speaker_id_node", "INFO", "👤 Speaker: 'Борис' (812 ms)")
    + _l("speaker_id_node", "INFO",
       "🔍 identify candidates: best='Саша'(a1b2c3d4) score=0.781 | "
       "second='Борис'(f499c2f7) score=0.495 | gap=0.286")
    + _l("speaker_id_node", "INFO", "👤 Speaker: 'Саша' (905 ms)")
)

# Старый формат лога (issue #2754 живой прогон был с этим форматом
# до того, как speaker_id_node получил префикс-эмодзи). Без 👤, но с
# явным ``confidence=...``. Парсер должен ловить и этот формат тоже —
# иначе мы регрессируем #2754.
OLD_FORMAT_LOG = (
    _l("speaker_id_node", "INFO",
       "[speaker_id_node] Speaker: 'Борис' confidence=0.95 embedding=f499c2f7")
    + _l("dialogue_node", "INFO",
         "✅ [turn] process_input returned: spoken='Борис, привет!'[:60] "
         "tools=[]")
)


# ── Test 1: must_not_identify_as=['Борис'] + вердикт 'Борис' → FAIL ─────────
class TestMustNotIdentifyAsBlocksKnownMatch:
    def test_must_not_identify_as_blocks_known_match(self):
        """Синтетический лог-файл + must_not_identify_as=['Борис'] +
        лог содержит ``👤 Speaker: 'Борис'`` → функция возвращает
        непустой список (есть причины FAIL)."""
        acc = {"must_not_identify_as": ["Борис"]}
        failures = identify_failures(acc, BORIS_VERDICT_LOG)
        assert len(failures) >= 1, failures
        # Причина должна содержать 'Борис' — чтобы оператор видел, кто
        # именно был опознан. Это требование issue #2754: «не понятно
        # КАКОЕ имя сматчилось → красный, но непонятно почему».
        assert any("Борис" in f for f in failures), failures


# ── Test 2: тот же acc, но лог без 'Борис' (только 'unknown') → PASS ───────
class TestMustNotIdentifyAsPassesForUnknown:
    def test_must_not_identify_as_passes_for_unknown(self):
        """acc = {'must_not_identify_as': ['Борис']} + лог с финальным
        вердиктом ``👤 Speaker: unknown`` → пустой список."""
        acc = {"must_not_identify_as": ["Борис"]}
        failures = identify_failures(acc, CLEAN_DIAGNOSTIC_LOG)
        assert failures == [], failures

    def test_candidates_diagnostik_does_not_trigger(self):
        """Симметрия #2779: «Борис» есть в candidates, но финальный
        вердикт ``unknown``. Парсер НЕ должен срабатывать на
        candidates — это диагностика, ещё не вердикт."""
        acc = {"must_not_identify_as": ["Борис"]}
        failures = identify_failures(acc, CLEAN_DIAGNOSTIC_LOG)
        assert failures == []


# ── Test 3: must_not_identify_as=['Борис','Саша'] + оба в вердиктах → FAIL ─
class TestMultipleNamesBlock:
    def test_multiple_names_block(self):
        """acc = {'must_not_identify_as': ['Борис', 'Саша']} + лог с
        финальными вердиктами и 'Борис', и 'Саша' (retried шаг) →
        в failures обе причины (по одной на каждое запрещённое имя)."""
        acc = {"must_not_identify_as": ["Борис", "Саша"]}
        failures = identify_failures(acc, BORIS_AND_SASHA_VERDICTS)
        # Должны быть обе причины — иначе «часть имён проскочила» = дыра.
        joined = " ".join(failures)
        assert "Борис" in joined, failures
        assert "Саша" in joined, failures


# ── Test 4: якорение парсера устойчиво к старому формату лога ──────────────
#
# До того как speaker_id_node получил префикс-эмодзи 👤 (PR, который
# закрывал #2754 в прошлом), строки лога были без эмодзи. Если
# парсер завязан ТОЛЬКО на 👤 — он регрессирует на старых логах. Это
# требование «устойчивость якоря» (ADR-0134 §2.1: «парсер должен
# покрывать ОБА формата»).
class TestAnchorRobustToOldFormat:
    def test_old_format_without_emoji_also_blocked(self):
        """acc = {'must_not_identify_as': ['Борис']} + лог в старом
        формате (без 👤, но с явным ``confidence=...``) → тоже
        FAIL. Иначе мы регрессируем #2754 на старых харнессах."""
        acc = {"must_not_identify_as": ["Борис"]}
        failures = identify_failures(acc, OLD_FORMAT_LOG)
        assert len(failures) >= 1, failures
        assert any("Борис" in f for f in failures), failures


# ── Test 5 (доп.): acc без must_not_identify_as → zero effect ──────────────
class TestEmptyForbiddenListIsZeroEffect:
    def test_no_field_returns_empty(self):
        """acc={} → пустой список. Поле опционально, обратная
        совместимость с acceptance.json, где его нет."""
        assert identify_failures({}, BORIS_VERDICT_LOG) == []

    def test_empty_list_returns_empty(self):
        """acc={'must_not_identify_as': []} → пустой список."""
        assert identify_failures(
            {"must_not_identify_as": []}, BORIS_VERDICT_LOG
        ) == []


# ── Test 6 (доп.): невалидный тип acc → диагностическая причина FAIL ───────
class TestSchemaErrorsAreReported:
    def test_non_list_field_is_schema_error(self):
        """acc={'must_not_identify_as': 'Борис'} (строка вместо списка)
        → функция возвращает причину с упоминанием типа."""
        failures = identify_failures(
            {"must_not_identify_as": "Борис"}, BORIS_VERDICT_LOG
        )
        assert any("list[str]" in f or "must be list" in f for f in failures), \
            failures

    def test_non_string_element_is_partially_checked(self):
        """Если в списке есть int (опечатка в acceptance.json), функция
        возвращает диагностическую причину, но всё равно проверяет
        остальные str'и (best-effort)."""
        acc = {"must_not_identify_as": ["Борис", 42]}
        failures = identify_failures(acc, BORIS_VERDICT_LOG)
        # Должна быть причина про «не str» (диагностика).
        assert any("is not str" in f or "42" in f for f in failures), failures
        # И должна быть причина про «Борис» (best-effort).
        assert any("Борис" in f for f in failures), failures


# ── Test 7 (доп.): нет вердиктов вообще (шаг не дошёл до биометрии) ─────────
class TestNoVerdictIsNotFailure:
    def test_no_verdict_at_all_is_zero_effect(self):
        """acc={'must_not_identify_as': ['Борис']} + лог без строк
        ``👤 Speaker: ...`` (например, wake-gate SKIP, шаг не дошёл
        до биометрии) → пустой список. Инвариант не нарушен —
        провисать нечего."""
        log_without_verdict = (
            _l("wake_gate", "INFO", "wake-gate SKIP — no cold-start match")
            + _l("dialogue_node", "INFO",
                 "✅ [turn] process_input returned: spoken='...'[:60] tools=[]")
        )
        failures = identify_failures(
            {"must_not_identify_as": ["Борис"]}, log_without_verdict
        )
        assert failures == []
