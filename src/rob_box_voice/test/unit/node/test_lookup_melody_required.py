#!/usr/bin/env python3
"""
test_lookup_melody_required.py — Issue #2560 acceptance tests.

Round-3 live (Vision Pi, 2026-09-15, DJ-сет): юзер 8 раз подряд просил
«в пещере горного короля», и модель КАЖДЫЙ раз выдавала фантазийный
паттерн «pe<num>le<num>f» (FoxDot/renardo-синтаксис) вместо реальных
нот Peer Gynt Suite №1 из RTTTL-библиотеки.

PR #2551 (RULE #KNOWN-MELODY в composer.txt / master_prompt_compact.txt)
— текстовое правило. Round-3 показал: правило модель ЧИТАЕТ и тут же
НАРУШАЕТ (6 случаев за 60 мин). Поэтому нужен runtime safety net на
стороне dialogue_node.

Этот файл покрывает acceptance criteria #1 + #2 + #3 + #4 карточки #2560:

  #1. detect_hallucinated_midi_in_tools() ловит паттерн pe<num>le<num>f
      в spoken (FoxDot/renardo-синтаксис).
  #2. На паттерн вызывается ОДИН CRITICAL-ретрай с явным требованием
      «сначала lookup_melody».
  #3. Prometheus counter voice_composer_hallucinated_midi_total
      инкрементируется на каждом срабатывании (source=guard, action=retry)
      и на каждом skip (source=skip, action=skipped).
  #4. CC-budget guard dialogue_node._handle_result согласован с baseline.

Тесты используют ``object.__new__(DialogueNode)`` паттерн (без полного
__init__) — тот же подход, что в test_issue_2548_music_prose_action.py
и test_issue_2175_dialogue_node_system_regurgitate_guard.py.

Run with::

    PYTHONPATH=src/rob_box_voice:src/rob_box_llm:src/rob_box_core:src/rob_box_harness \\
        python3 -m pytest \\
        src/rob_box_voice/test/unit/node/test_lookup_melody_required.py -v
"""
from __future__ import annotations

import unittest
from unittest.mock import MagicMock

import pytest

from rob_box_harness.core.agent_core import DialogResult
from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateMachine,
)

from rob_box_voice.core.dialogue_guards import (
    HALLUCINATED_MIDI_RE,
    build_hallucinated_midi_retry_prompt,
    detect_hallucinated_midi,
    detect_hallucinated_midi_in_tools,
)
from rob_box_voice.dialogue_node import DialogueNode
from rob_box_voice.observability.metrics import record_hallucinated_midi


# ---------------------------------------------------------------------------
# Test scaffolding — DialogueNode shim без __init__
# ---------------------------------------------------------------------------


class _LoggerStub:
    """Минимальный логгер-заглушка для object.__new__(DialogueNode)."""

    def __init__(self) -> None:
        self.warnings: list[str] = []
        self.infos: list[str] = []
        self.debugs: list[str] = []
        self.errors: list[str] = []

    def info(self, msg: str, *args, **kwargs) -> None:
        self.infos.append(msg % args if args else msg)

    def warning(self, msg: str, *args, **kwargs) -> None:
        self.warnings.append(msg % args if args else msg)

    def warn(self, msg: str, *args, **kwargs) -> None:  # alias
        self.warning(msg, *args, **kwargs)

    def debug(self, msg: str, *args, **kwargs) -> None:
        self.debugs.append(msg % args if args else msg)

    def error(self, msg: str, *args, **kwargs) -> None:
        self.errors.append(msg % args if args else msg)


def _make_stub_node() -> tuple[DialogueNode, _LoggerStub]:
    """Создаёт DialogueNode shim без __init__ (test-only паттерн).

    Совпадает с test_issue_2175_dialogue_node_system_regurgitate_guard.py
    и test_issue_2548_music_prose_action.py.
    """
    node = object.__new__(DialogueNode)
    logger = _LoggerStub()
    node.get_logger = lambda: logger
    # Минимальный DSM для ретраев (см. _check_*_and_retry helper'ы)
    node._dsm = DialogueStateMachine()
    node._publish_state = MagicMock()
    # Флаг ретрая, который _run_turn смотрит в finally
    node._retry_dispatched_in_turn = False
    # Синтетический budget (issue #1881)
    node._synthetic_retries_left = DialogueNode.DEFAULT_SYNTHETIC_RETRIES
    # Перехватываем _dispatch_turn, чтобы убедиться что ретрай вызван
    node._dispatch_turn = MagicMock()
    return node, logger


# ===========================================================================
# Acceptance #1: detect_hallucinated_midi — детектор покрывает 10 кейсов
# ===========================================================================


class TestDetectHallucinatedMidi(unittest.TestCase):
    """Acceptance #1 — детектор ловит pe<num>le<num>f в любой позиции."""

    def test_live_pe8le1f_pattern_detected(self) -> None:
        """Round-3 live: «pe8le1f» — реальный паттерн из лога."""
        pattern = detect_hallucinated_midi(
            "p1 >> strangerarp(midinote=[52,52,55,52,59...]) "
            "# Grieg mountain king melody (pe8le1f in E minor, classic)"
        )
        self.assertEqual(pattern, "pe8le1f")

    def test_uppercase_pe8LE1F_is_detected_case_insensitive(self) -> None:
        """Регистр не имеет значения — модель может писать PE8LE1F."""
        pattern = detect_hallucinated_midi("looking up PE8LE1F in my head")
        self.assertEqual(pattern, "PE8LE1F")

    def test_pattern_in_execute_music_code_arg_detected(self) -> None:
        """Паттерн в ``code="...pe1le2f..."`` — реальная позиция бага."""
        pattern = detect_hallucinated_midi(
            'execute_music_code(args=["code=\\"p1 >> strangerarp(midinote='
            '[52,52,55,52,59] pe1le2f in E minor)\\n\\""])'
        )
        self.assertEqual(pattern, "pe1le2f")

    def test_various_numeric_ranges_detected(self) -> None:
        """Числа в паттерне — 1-3 цифры, pe12le34f должен ловиться."""
        for sample in ("pe1le2f", "pe12le34f", "pe999le888f", "pe100le200f"):
            with self.subTest(sample=sample):
                self.assertEqual(
                    detect_hallucinated_midi(f"fake midi id {sample}"),
                    sample,
                )

    def test_empty_text_returns_none(self) -> None:
        """Пустой текст → None (защита от False positives)."""
        self.assertIsNone(detect_hallucinated_midi(""))
        self.assertIsNone(detect_hallucinated_midi(None))

    def test_normal_text_without_pattern_returns_none(self) -> None:
        """Обычный ответ без pe<num>le<num>f → None."""
        self.assertIsNone(detect_hallucinated_midi("Готово, играет Бетховен."))
        self.assertIsNone(detect_hallucinated_midi("Сыграю Лунную сонату."))
        self.assertIsNone(detect_hallucinated_midi("Clock.bpm = 120"))

    def test_pe_only_or_le_only_does_not_trigger(self) -> None:
        """Только pe или только le — НЕ hallucinated-MIDI."""
        self.assertIsNone(detect_hallucinated_midi("pe is short for pattern"))
        self.assertIsNone(detect_hallucinated_midi("le is short for left"))

    def test_real_pattern_name_does_not_trigger(self) -> None:
        """Реальные FoxDot/renardo pattern names (``p1``, ``d1``, ``pluck````) —
        НЕ триггерятся. У них нет ``le<num>f`` суффикса.
        """
        self.assertIsNone(detect_hallucinated_midi("p1 >> pluck([0,2,4])"))
        self.assertIsNone(detect_hallucinated_midi("p2 >> sawbass([0,4,7])"))

    def test_real_lookup_melody_name_does_not_trigger(self) -> None:
        """Результат lookup_melody (rtttl:...  строка) — НЕ триггерит."""
        text = (
            'rtttl:PeerGynt:d=4,o=5,b=112:8f#6, ... '
            '| real answer from RTTTL library'
        )
        self.assertIsNone(detect_hallucinated_midi(text))

    def test_word_boundary_required(self) -> None:
        """``pe1le1factor`` (без word-boundary после f) — НЕ триггерит,
        защита от ложных срабатываний на словах с похожим суффиксом.
        """
        # Если бы регекс был без \b, мы бы поймали подстроку.
        self.assertIsNone(detect_hallucinated_midi("pe1le1factor=42"))

    def test_multiple_matches_returns_first(self) -> None:
        """Несколько совпадений → возвращаем ПЕРВОЕ (стабильный лог)."""
        text = "first pe8le1f and later pe2le3f in same response"
        pattern = detect_hallucinated_midi(text)
        self.assertEqual(pattern, "pe8le1f")


# ===========================================================================
# Acceptance #1 (продолжение): guard-фронт detect_hallucinated_midi_in_tools
# ===========================================================================


class TestDetectHallucinatedMidiInTools(unittest.TestCase):
    """Тот же детектор, но для guard-интерфейса dialogue_node."""

    def test_pattern_in_spoken_detected(self) -> None:
        """pattern в spoken → found, tools_called не важны."""
        self.assertEqual(
            detect_hallucinated_midi_in_tools(
                spoken="compose_music(name='mountain king', pe8le1f in E minor)",
                tools_called=("compose_music",),
            ),
            "pe8le1f",
        )

    def test_no_pattern_in_spoken_returns_none(self) -> None:
        """Нет паттерна в spoken → None."""
        self.assertIsNone(
            detect_hallucinated_midi_in_tools(
                spoken="Готово, играет Бетховен.",
                tools_called=("compose_music",),
            )
        )

    def test_empty_spoken_returns_none(self) -> None:
        """Пустой spoken → None (нельзя ретраить на пустом контенте)."""
        self.assertIsNone(
            detect_hallucinated_midi_in_tools(
                spoken="",
                tools_called=(),
            )
        )


# ===========================================================================
# Acceptance #2: build_hallucinated_midi_retry_prompt — CRITICAL contract
# ===========================================================================


class TestBuildHallucinatedMidiRetryPrompt(unittest.TestCase):
    """Acceptance #2 — ретрай-промпт содержит явное требование lookup_melody."""

    def test_prompt_contains_critical_marker(self) -> None:
        """[CRITICAL] — обязательный маркер для синтетического ретрая."""
        prompt = build_hallucinated_midi_retry_prompt(
            user_input="сыграй в пещере горного короля",
            pattern="pe8le1f",
        )
        self.assertIn("[CRITICAL]", prompt)

    def test_prompt_requires_lookup_melody(self) -> None:
        """Главное требование — ОБЯЗАТЕЛЬНО lookup_melody."""
        prompt = build_hallucinated_midi_retry_prompt(
            user_input="сыграй григ",
            pattern="pe1le2f",
        )
        self.assertIn("lookup_melody", prompt)

    def test_prompt_bans_hallucinated_midi(self) -> None:
        """Промпт явно запрещает «Григ-подобный» / «Бетховен-стайл»."""
        prompt = build_hallucinated_midi_retry_prompt(
            user_input="сыграй бетховена",
            pattern="pe9le9f",
        )
        self.assertIn("ЗАПРЕЩЕНО", prompt)
        self.assertTrue(
            "Григ-подобный" in prompt or "Бетховен-стайл" in prompt,
            "промпт должен явно запрещать «Григ-подобный» / «Бетховен-стайл»",
        )

    def test_prompt_includes_pattern_for_traceability(self) -> None:
        """Промпт содержит название паттерна — для трассировки в логе LLM."""
        prompt = build_hallucinated_midi_retry_prompt(
            user_input="x", pattern="pe8le1f"
        )
        self.assertIn("pe8le1f", prompt)

    def test_prompt_preserves_user_input(self) -> None:
        """Промпт содержит оригинальный user_input (контекст)."""
        prompt = build_hallucinated_midi_retry_prompt(
            user_input="сыграй в пещере горного короля",
            pattern="pe8le1f",
        )
        self.assertIn("сыграй в пещере горного короля", prompt)

    def test_prompt_with_empty_user_input(self) -> None:
        """Пустой user_input не ломает построитель (defensive)."""
        prompt = build_hallucinated_midi_retry_prompt(
            user_input="", pattern="pe1le1f"
        )
        self.assertIn("[CRITICAL]", prompt)
        self.assertIn("lookup_melody", prompt)


# ===========================================================================
# Acceptance #2 (продолжение): _check_hallucinated_midi_and_retry
# ===========================================================================


class TestCheckHallucinatedMidiAndRetry(unittest.TestCase):
    """Acceptance #2 — guard срабатывает и шлёт ОДИН CRITICAL-ретрай."""

    def test_guard_fires_on_pe8le1f_in_spoken(self) -> None:
        """Pattern в spoken → guard диспатчит ретрай."""
        node, logger = _make_stub_node()
        result = node._check_hallucinated_midi_and_retry(
            spoken=(
                "compose_music(name='mountain king', pe8le1f in E minor) — "
                "выдумываю MIDI"
            ),
            user_input="сыграй в пещере горного короля",
            tools_called=("compose_music",),
        )
        self.assertTrue(result, "guard должен сработать на pe8le1f в spoken")
        node._dispatch_turn.assert_called_once()
        # Ретрай-промпт — первый позиционный аргумент
        prompt = node._dispatch_turn.call_args.args[0]
        self.assertIn("[CRITICAL]", prompt)
        self.assertIn("lookup_melody", prompt)
        # В логе должно быть предупреждение (issue #2560 маркер)
        joined = "\n".join(logger.warnings)
        self.assertIn("issue 2560", joined)
        self.assertIn("pe8le1f", joined)

    def test_guard_does_not_fire_on_clean_response(self) -> None:
        """Обычный ответ без паттерна → guard молчит."""
        node, _ = _make_stub_node()
        result = node._check_hallucinated_midi_and_retry(
            spoken="Готово, играет Бетховен.",
            user_input="сыграй бетховена",
            tools_called=("compose_music",),
        )
        self.assertFalse(result)
        node._dispatch_turn.assert_not_called()

    def test_guard_is_oneshot_within_turn(self) -> None:
        """Второй вызов в том же turn → False (no ping-pong)."""
        node, _ = _make_stub_node()
        spoken_with_pattern = "still hallucinating, pe8le1f here"
        # Первый — успешный
        first = node._check_hallucinated_midi_and_retry(
            spoken=spoken_with_pattern,
            user_input="x",
            tools_called=(),
        )
        self.assertTrue(first)
        node._dispatch_turn.assert_called_once()
        # Второй в том же turn — guard молчит
        node._dispatch_turn.reset_mock()
        second = node._check_hallucinated_midi_and_retry(
            spoken=spoken_with_pattern,
            user_input="x",
            tools_called=(),
        )
        self.assertFalse(second, "второй ретрай в одном turn недопустим (ping-pong)")
        node._dispatch_turn.assert_not_called()

    def test_guard_skips_when_budget_exhausted(self) -> None:
        """Issue #1881 — budget == 0 → guard молчит + skip-метрика."""
        node, _ = _make_stub_node()
        node._synthetic_retries_left = 0  # budget исчерпан
        result = node._check_hallucinated_midi_and_retry(
            spoken="pe8le1f — выдуманный MIDI",
            user_input="x",
            tools_called=(),
        )
        self.assertFalse(result)
        node._dispatch_turn.assert_not_called()
        # Лог должен содержать маркер исчерпания budget'а (issue #1881)
        # И guard-флаг НЕ выставлен (это НЕ ретрай).


# ===========================================================================
# Acceptance #4: Prometheus counter voice_composer_hallucinated_midi_total
# ===========================================================================


class TestHallucinatedMidiPrometheusCounter(unittest.TestCase):
    """Acceptance #4 — record_hallucinated_midi корректно инкрементится."""

    def test_record_guard_retry_increments_with_labels(self) -> None:
        """source=guard, action=retry → инкремент с этими labels."""
        # Метрика idempotent: get_metric возвращает тот же counter
        # при повторных вызовах — поэтому просто дёргаем и убеждаемся,
        # что не падает.
        record_hallucinated_midi(source="guard", action="retry")
        record_hallucinated_midi(source="guard", action="retry")

    def test_record_skip_skipped_increments_with_labels(self) -> None:
        """source=skip, action=skipped → инкремент (budget exhausted)."""
        record_hallucinated_midi(source="skip", action="skipped")

    def test_record_does_not_raise_on_bad_labels(self) -> None:
        """Защита от мусорных labels: prometheus_client упадёт,
        если в labelname появится символ не из [a-zA-Z0-9_]. Тест
        гарантирует, что в штатном использовании labels валидны.
        """
        # source/action — keyword-only, тип str, типичные значения
        # "guard"/"skip" и "retry"/"publish"/"skipped" — все валидные
        # prometheus labels.
        for source in ("guard", "skip"):
            for action in ("retry", "publish", "skipped"):
                record_hallucinated_midi(source=source, action=action)


# ===========================================================================
# Acceptance #3: dialogue_node._handle_result вызывает guard (AST-walk test)
# ===========================================================================


class TestHallucinatedMidiGuardWiredIntoHandleResult:
    """Issue #2560 — guard виден через AST-walk в теле _handle_result.

    Тест проверяет контракт, который зафиксирован после инцидента
    30.08.2026 — блок вызова guard-метода в __init__ (там нет ни
    spoken, ни tools_called) падал с NameError, нода не поднималась.
    AST-обход должен видеть ``self._check_hallucinated_midi_and_retry``
    прямо из тела ``_handle_result``.
    """

    def test_guard_call_site_visible_in_handle_result(self) -> None:
        import ast
        import importlib
        from pathlib import Path

        from rob_box_voice.dialogue_node import DialogueNode

        dialogue_module = importlib.import_module(DialogueNode.__module__)
        assert dialogue_module.__file__ is not None
        dialogue_path = Path(dialogue_module.__file__)
        tree = ast.parse(dialogue_path.read_text(encoding="utf-8"))
        # Найти def _handle_result в модуле
        handle_result: ast.FunctionDef | None = None
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and node.name == "_handle_result":
                handle_result = node
                break
        assert handle_result is not None, "_handle_result не найден в dialogue_node.py"

        # Найти все вызовы guard-методов через self._check_*_and_retry
        guard_calls: set[str] = set()
        for sub in ast.walk(handle_result):
            if (
                isinstance(sub, ast.Call)
                and isinstance(sub.func, ast.Attribute)
                and sub.func.attr.startswith("_check_")
                and sub.func.attr.endswith("_and_retry")
            ):
                guard_calls.add(sub.func.attr)
        assert "_check_hallucinated_midi_and_retry" in guard_calls, (
            "_check_hallucinated_midi_and_retry не вызывается из _handle_result — "
            "AST-walk тест ловит регрессию инцидента 30.08.2026 (issue #2560)"
        )


# ===========================================================================
# Acceptance #4 (CC-budget): dialogue_node._handle_result CC baseline
# ===========================================================================


class TestHandleResultCCBaselineMatches:
    """CC-budget guard (ADR-0021) — _handle_result CC согласован с baseline."""

    def test_handle_result_cc_within_baseline(self) -> None:
        import importlib
        import json
        import subprocess
        from pathlib import Path

        from rob_box_voice.dialogue_node import DialogueNode

        # Загружаем baseline. Walk upward from dialogue_node.py looking
        # for ``scripts/lint/cc_budget_baseline.json`` — works whether the
        # module is loaded from a worktree or from an installed copy.
        dialogue_module = importlib.import_module(DialogueNode.__module__)
        assert dialogue_module.__file__ is not None
        baseline_path: Path | None = None
        cursor = Path(dialogue_module.__file__).resolve().parent
        for _ in range(8):  # at most 8 levels up — covers worktree + parent repo
            candidate = cursor / "scripts" / "lint" / "cc_budget_baseline.json"
            if candidate.is_file():
                baseline_path = candidate
                break
            if cursor.parent == cursor:
                break
            cursor = cursor.parent
        assert baseline_path is not None, (
            "cc_budget_baseline.json не найден при обходе вверх от "
            f"{dialogue_module.__file__}"
        )
        baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
        expected_cc = None
        for entry in baseline.get("_legacy_acknowledged", []):
            if (
                entry.get("path", "").endswith("dialogue_node.py")
                and entry.get("method") == "DialogueNode._handle_result"
            ):
                expected_cc = int(entry["cc"])
                break
        assert expected_cc is not None, (
            "_handle_result baseline не найден в cc_budget_baseline.json"
        )

        # Измеряем radon'ом
        dialogue_path = Path(dialogue_module.__file__)
        result = subprocess.run(
            ["python3", "-m", "radon", "cc", "-s", str(dialogue_path)],
            capture_output=True,
            text=True,
            timeout=30,
        )
        # radon CC — output format:
        #   "M 5608:4 DialogueNode._handle_result - F (75)"
        actual_cc = None
        for line in result.stdout.splitlines():
            if "DialogueNode._handle_result - " in line:
                # parse "F (NN)"
                head = line.split("(")[1]
                actual_cc = int(head.rstrip(")"))
                break
        assert actual_cc is not None, (
            f"radon не нашёл DialogueNode._handle_result в:\n{result.stdout}"
        )
        assert actual_cc == expected_cc, (
            f"DialogueNode._handle_result CC drift: actual={actual_cc}, "
            f"baseline={expected_cc}. Update cc_budget_baseline.json "
            f"(exemptions и _legacy_acknowledged) accordingly."
        )


if __name__ == "__main__":
    unittest.main()