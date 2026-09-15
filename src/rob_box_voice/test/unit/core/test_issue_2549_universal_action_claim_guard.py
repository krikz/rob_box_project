#!/usr/bin/env python3
"""test_issue_2549_universal_action_claim_guard.py — universal anti-hallucination guard.

Issue #2549 — LLM отчитывается о действии в свободной форме при пустом
``tools_called``. Узкий Bug E guard (:func:`detect_unbacked_action_claim`)
ловит только случаи, где И запрос юзера, И утверждение LLM попадают в
таблицу :data:`ACTION_CLAIM_RULES`. DJ-сет 2026-09-15 показал, что LLM
часто отчитывается о действии В СВОБОДНОЙ ФОРМЕ (без явного «сделай/запусти»
в запросе юзера)::

    «Сделала два pass подряд: сначала один темп-каркас с heartbeat'ом и
    пульсом Бочкинса, потом второй…» (tools=[])
    «Вплела тему Грига как второй голос над пульсом Бочкинса…» (tools=[])
    «Понимаю, пока не звучит — дай минуту, проверю состояние и
    перезапущу.» (tools=[])
    «Ок, давай я снова перезапущу. Бочкинс с Григом наверху — стартую
    заново.» (tools=[])

Юзер слышит «готово» — а музыка не менялась. Это action hallucination.

Защита: широкий детектор :func:`detect_universal_action_claim` ищет в
``spoken`` action-verb в past или future + проверяет, что нет вызванного
тула из whitelist :data:`CLAIM_JUSTIFYING_TOOLS`. Один одноразовый
CRITICAL-ретрай (тот же контракт, что у Bug E guard'а).

Тесты — unit-уровень: мини-DialogueNode с подменой зависимостей, чтобы
не поднимать тяжёлый ``__init__`` (rclpy, executor, memory, tools).
Подход идентичен :mod:`test_issue_1777_tool_skipped_guard`.
"""

from __future__ import annotations

import unittest
from typing import Optional

from rob_box_harness.core.dialogue_state_machine import (DialogueStateKind,
                                                         DialogueStateMachine)

# ── Минимальный dialogue_node — повторяем hooks нужные для guard'а ──────


class _DispatchCall:
    """Запись о вызове ``_dispatch_turn`` для assertion."""

    __slots__ = ("user_input", "raw_user_command", "was_idle", "is_synthetic")

    def __init__(
        self,
        user_input: str,
        raw_user_command: Optional[str],
        was_idle: bool,
        is_synthetic: bool,
    ) -> None:
        self.user_input = user_input
        self.raw_user_command = raw_user_command
        self.was_idle = was_idle
        self.is_synthetic = is_synthetic


class _MiniDialogueNode:
    """Зеркало минимального интерфейса ``DialogueNode`` для guard'а."""

    def __init__(self) -> None:
        self._universal_action_claim_retry_used: bool = False
        self._retry_dispatched_in_turn: bool = False
        self._dispatched: list[_DispatchCall] = []
        self._reopen_called: int = 0
        self.warnings: list[str] = []
        self._dsm = DialogueStateMachine(initial=DialogueStateKind.DIALOGUE)

    def get_logger(self):  # noqa: D401 — minimal logger
        class _Logger:
            def __init__(self, outer):
                self._outer = outer

            def warning(self, msg: str) -> None:
                self._outer.warnings.append(msg)

        return _Logger(self)

    def _reopen_dialogue_for_retry(self) -> None:
        self._reopen_called += 1

    def _consume_synthetic_retry(self, *, guard_name: str) -> bool:
        # Упрощённая модель: бюджет всегда есть (1).
        return True

    def _mark_retry_dispatched(self) -> None:
        self._retry_dispatched_in_turn = True

    def _dispatch_turn(
        self,
        user_input: str,
        *,
        was_idle: bool = False,
        is_action_claim_retry: bool = False,
        is_synthetic: bool = False,
        raw_user_command: Optional[str] = None,
        **kwargs,
    ) -> None:
        self._dispatched.append(
            _DispatchCall(
                user_input=user_input,
                raw_user_command=raw_user_command,
                was_idle=was_idle,
                is_synthetic=is_synthetic,
            )
        )

    # ── тестируемый метод — копия из dialogue_node ──

    def _check_universal_action_claim_and_retry(
        self,
        *,
        spoken: str,
        user_input: Optional[str],
        tools_called: tuple,
    ) -> bool:
        if not spoken:
            return False
        if getattr(self, "_universal_action_claim_retry_used", False):
            return False
        if getattr(self, "_retry_dispatched_in_turn", False):
            return False

        from rob_box_voice.core.dialogue_guards import (
            build_universal_action_claim_retry_prompt,
            detect_universal_action_claim)

        hit = detect_universal_action_claim(
            spoken=spoken,
            tools_called=tuple(tools_called or ()),
        )
        if hit is None:
            return False

        # DSM reopen — упрощённо пропускаем (мини-DSM в DIALOGUE).
        self._reopen_dialogue_for_retry()

        if not self._consume_synthetic_retry(guard_name="universal_action_claim"):
            return False
        self._universal_action_claim_retry_used = True
        self._mark_retry_dispatched()
        self.get_logger().warning(
            "🧾 [issue 2549] anti-hallucination guard: spoken содержит "
            f"action-verb «{hit.verb}» ({hit.tense}), tools пуст — "
            f"head={hit.excerpt!r}, user_input={user_input!r}, "
            f"tools={list(tools_called)!r}"
        )
        self._dispatch_turn(
            build_universal_action_claim_retry_prompt(
                user_input=user_input, spoken=spoken, hit=hit
            ),
            is_action_claim_retry=False,
            is_synthetic=True,
            raw_user_command=user_input,
        )
        return True


# ── Pure-детектор unit-тесты (без DialogueNode) ─────────────────────────


class TestUniversalActionClaimDetector(unittest.TestCase):
    """Тесты :func:`detect_universal_action_claim` напрямую."""

    def test_past_verb_triggers(self):
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        hit = detect_universal_action_claim(
            spoken="Сделала два pass подряд: сначала один темп-каркас.",
            tools_called=(),
        )
        assert hit is not None  # noqa: S101 — narrowing for type checker
        self.assertEqual(hit.tense, "past")
        # Verb сохраняется as-is (case-insensitive match возвращает
        # оригинальный регистр: «Сделала», не «сделал»).
        self.assertTrue(hit.verb.lower().startswith("сделал"))

    def test_future_verb_triggers(self):
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        hit = detect_universal_action_claim(
            spoken="Дай минуту, проверю состояние и перезапущу.",
            tools_called=(),
        )
        assert hit is not None  # noqa: S101
        self.assertEqual(hit.tense, "future")

    def test_no_verb_no_trigger(self):
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        hit = detect_universal_action_claim(
            spoken="Привет! Как дела?",
            tools_called=(),
        )
        self.assertIsNone(hit)

    def test_empty_no_trigger(self):
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        hit = detect_universal_action_claim(
            spoken="",
            tools_called=(),
        )
        self.assertIsNone(hit)

    def test_tools_called_justifies_no_retry(self):
        """Если LLM вызвала compose_music — spoken-action оправдан."""
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        hit = detect_universal_action_claim(
            spoken="Запустил новый трек.",
            tools_called=("compose_music",),
        )
        self.assertIsNone(hit)

    def test_get_music_state_justifies(self):
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        hit = detect_universal_action_claim(
            spoken="Проверил состояние — играет тишина.",
            tools_called=("get_music_state",),
        )
        self.assertIsNone(hit)

    def test_set_volume_justifies(self):
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        hit = detect_universal_action_claim(
            spoken="Подкрутил громкость на максимум.",
            tools_called=("set_volume",),
        )
        self.assertIsNone(hit)

    def test_speak_text_alone_does_not_justify(self):
        """``speak_text`` НЕ оправдывает action-claim (он только говорит)."""
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        hit = detect_universal_action_claim(
            spoken="Запустил новый трек.",
            tools_called=("speak_text",),
        )
        self.assertIsNotNone(hit)

    def test_save_waypoint_justifies(self):
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        hit = detect_universal_action_claim(
            spoken="Сохранил точку на кухне.",
            tools_called=("save_waypoint",),
        )
        self.assertIsNone(hit)

    def test_dj_klass_grieg_quote(self):
        """Дословный кейс из карточки #2549."""
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        text = "Вплела тему Грига как второй голос над пульсом Бочкинса…"
        hit = detect_universal_action_claim(
            spoken=text,
            tools_called=(),
        )
        assert hit is not None  # noqa: S101
        self.assertEqual(hit.tense, "past")

    def test_dj_perezeapusk(self):
        """Дословный кейс из карточки #2549."""
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        text = (
            "Ок, давай я снова перезапущу. Бочкинс с Григом наверху — "
            "стартую заново."
        )
        hit = detect_universal_action_claim(
            spoken=text,
            tools_called=(),
        )
        assert hit is not None  # noqa: S101
        # «стартую» и «перезапущу» — оба future. Проверяем что хоть один
        # сработал.
        self.assertEqual(hit.tense, "future")

    def test_past_verb_priority(self):
        """Past tense матч идёт первым (чаще в спонтанных ответах)."""
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        text = "Сделала pass, и потом проверю."
        hit = detect_universal_action_claim(
            spoken=text,
            tools_called=(),
        )
        assert hit is not None  # noqa: S101
        self.assertEqual(hit.tense, "past")

    def test_feminine_endings(self):
        """«Сделала», «запустила», «проверила» — женский род."""
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        for text in [
            "Сделала композицию.",
            "Запустила новый бит.",
            "Проверила состояние.",
            "Обновила тему.",
        ]:
            with self.subTest(text=text):
                hit = detect_universal_action_claim(
                    spoken=text,
                    tools_called=(),
                )
                self.assertIsNotNone(
                    hit,
                    f"expected hit for «{text}»",
                )


# ── End-to-end guard тесты (через _MiniDialogueNode) ────────────────────


class TestGuardRetry(unittest.TestCase):
    """Проверяет что guard отправляет ОДИН ретрай."""

    def test_action_verb_no_tool_dispatches_one_retry(self):
        """Acceptance #1: «Запустил новый трек» + tools=[] → ОДИН retry."""
        node = _MiniDialogueNode()
        result = node._check_universal_action_claim_and_retry(
            spoken="Запустил новый трек.",
            user_input="включи музыку",
            tools_called=(),
        )
        self.assertTrue(result)
        self.assertEqual(len(node._dispatched), 1)
        call = node._dispatched[0]
        self.assertIn("[CRITICAL]", call.user_input)
        self.assertIn("Запустил", call.user_input)  # echoed claim
        self.assertTrue(call.is_synthetic)
        # raw_user_command — оригинал, чтобы Bug C/D guard'ы на retry-туре
        # не сканировали синтетический промпт.
        self.assertEqual(call.raw_user_command, "включи музыку")

    def test_tool_called_no_retry(self):
        """Acceptance #2: «Запустил новый трек» + tools=[compose_music] →
        НЕ retry (action оправдан)."""
        node = _MiniDialogueNode()
        result = node._check_universal_action_claim_and_retry(
            spoken="Запустил новый трек.",
            user_input="включи музыку",
            tools_called=("compose_music",),
        )
        self.assertFalse(result)
        self.assertEqual(node._dispatched, [])

    def test_no_action_verb_no_retry(self):
        """Acceptance #3: «Подумаю ещё» + tools=[] → НЕ retry."""
        node = _MiniDialogueNode()
        result = node._check_universal_action_claim_and_retry(
            spoken="Подумаю ещё над этим.",
            user_input="расскажи анекдот",
            tools_called=(),
        )
        self.assertFalse(result)
        self.assertEqual(node._dispatched, [])

    def test_one_shot_flag_blocks_second_retry(self):
        """Защита от ping-pong: один ретрай на turn."""
        node = _MiniDialogueNode()
        # Первый вызов успешен.
        self.assertTrue(
            node._check_universal_action_claim_and_retry(
                spoken="Сделал pass.",
                user_input="докрути",
                tools_called=(),
            )
        )
        # Второй вызов на том же turn (тул так и не вызвался) — молчим.
        self.assertFalse(
            node._check_universal_action_claim_and_retry(
                spoken="Сделал ещё один pass.",
                user_input="докрути",
                tools_called=(),
            )
        )
        # Один ретрай, не два.
        self.assertEqual(len(node._dispatched), 1)

    def test_other_retry_dispatched_blocks_action_guard(self):
        """Если babble/action-claim Bug E/renardo guard уже задиспатчил
        ретрай в этом turn, anti-hallucination guard молчит."""
        node = _MiniDialogueNode()
        node._retry_dispatched_in_turn = True
        self.assertFalse(
            node._check_universal_action_claim_and_retry(
                spoken="Запустил новый трек.",
                user_input="включи",
                tools_called=(),
            )
        )

    def test_empty_spoken_no_retry(self):
        """Пустой spoken (защита от race-condition)."""
        node = _MiniDialogueNode()
        self.assertFalse(
            node._check_universal_action_claim_and_retry(
                spoken="",
                user_input="включи",
                tools_called=(),
            )
        )


class TestGuardPrompt(unittest.TestCase):
    """Тесты структуры CRITICAL-ретрай-промпта."""

    def test_prompt_contains_user_input_echo(self):
        from rob_box_voice.core.dialogue_guards import (
            build_universal_action_claim_retry_prompt,
            detect_universal_action_claim)

        hit = detect_universal_action_claim(
            spoken="Запустил новый трек.",
            tools_called=(),
        )
        assert hit is not None  # noqa: S101
        prompt = build_universal_action_claim_retry_prompt(
            user_input="включи музыку",
            spoken="Запустил новый трек.",
            hit=hit,
        )
        self.assertIn("включи музыку", prompt)
        self.assertIn("[CRITICAL]", prompt)

    def test_prompt_names_caught_verb(self):
        from rob_box_voice.core.dialogue_guards import (
            build_universal_action_claim_retry_prompt,
            detect_universal_action_claim)

        hit = detect_universal_action_claim(
            spoken="Вплела тему Грига.",
            tools_called=(),
        )
        assert hit is not None  # noqa: S101
        prompt = build_universal_action_claim_retry_prompt(
            user_input="",
            spoken="Вплела тему Грига.",
            hit=hit,
        )
        # Пойманный глагол должен быть назван в промпте явно (case-as-is:
        # regex-матч IGNORECASE сохраняет оригинальный регистр).
        self.assertIn("Вплела", prompt)

    def test_prompt_strips_previous_critical_block(self):
        """Если user_input уже содержит [CRITICAL] — обрезаем перед
        склейкой (тот же контракт, что у babble/regurgitate)."""
        from rob_box_voice.core.dialogue_guards import (
            build_universal_action_claim_retry_prompt,
            detect_universal_action_claim)

        hit = detect_universal_action_claim(
            spoken="Сделал.",
            tools_called=(),
        )
        assert hit is not None  # noqa: S101
        prompt = build_universal_action_claim_retry_prompt(
            user_input="докрути\n\n[CRITICAL] старый блок",
            spoken="Сделал.",
            hit=hit,
        )
        self.assertEqual(prompt.count("[CRITICAL]"), 1)
        self.assertNotIn("старый блок", prompt)


# ── Frame-привязка к issue #2549 ────────────────────────────────────────


class TestGuardIssue2549Scenarios(unittest.TestCase):
    """Дословные сценарии из issue #2549 должны срабатывать."""

    def test_dj_pass_dva(self):
        """Сделала два pass подряд..."""
        node = _MiniDialogueNode()
        self.assertTrue(
            node._check_universal_action_claim_and_retry(
                spoken=(
                    "Сделала два pass подряд: сначала один темп-каркас "
                    "с heartbeat'ом и пульсом Бочкинса, потом второй…"
                ),
                user_input="докрути музыку",
                tools_called=(),
            )
        )

    def test_dj_vplela_griga(self):
        """Вплела тему Грига как второй голос над пульсом Бочкинса..."""
        node = _MiniDialogueNode()
        self.assertTrue(
            node._check_universal_action_claim_and_retry(
                spoken=("Вплела тему Грига как второй голос над пульсом " "Бочкинса…"),
                user_input="добавь Грига",
                tools_called=(),
            )
        )

    def test_dj_proveriu_sostoyanie(self):
        """Проверю состояние и перезапущу."""
        node = _MiniDialogueNode()
        self.assertTrue(
            node._check_universal_action_claim_and_retry(
                spoken=(
                    "Понимаю, пока не звучит — дай минуту, проверю "
                    "состояние и перезапущу."
                ),
                user_input="что-то не то",
                tools_called=(),
            )
        )

    def test_dj_perezapusk(self):
        """Ок, давай я снова перезапущу. Бочкинс с Григом наверху —
        стартую заново."""
        node = _MiniDialogueNode()
        self.assertTrue(
            node._check_universal_action_claim_and_retry(
                spoken=(
                    "Ок, давай я снова перезапущу. Бочкинс с Григом "
                    "наверху — стартую заново."
                ),
                user_input="переделай",
                tools_called=(),
            )
        )

    def test_tools_called_non_empty_no_retry(self):
        """Acceptance #4 (контраст к DJ): если LLM ВСЁ-ТАКИ вызвала tool —
        guard молчит, action оправдан."""
        node = _MiniDialogueNode()
        self.assertFalse(
            node._check_universal_action_claim_and_retry(
                spoken="Запустил новый трек.",
                user_input="включи",
                tools_called=("execute_music_code",),
            )
        )


# -----------------------------------------------------------------------
# Task t_c7d707bc — расширение coverage verbs до полного списка из
# спецификации Issue #2559 body §1.
#
# Issue #2559 live 15.09 18:00-19:00: робот говорил «проверю и
# перезапущу», «установлю заново», «остановлю проигрывание», «подложу
# второй голос», «переключу на другой режим» при tools=[]. Узкий Bug E
# guard не ловил эти prose-фразы. PR #2555 (issue #2549) дал базовый
# набор verbs; эта секция добавляет:
#
#   past: установил[аи]?, остановил[аи]?, подложил[аи]?,
#         переключил[аи]?, починил[аи]?, перезагрузил[аи]?
#   future: установлю, остановлю, подложу, переключу, подкручу,
#           поправлю, починю, перезагружу
# -----------------------------------------------------------------------


class TestExtendedVerbsCoverage(unittest.TestCase):
    """Task t_c7d707bc — verbs добавлены в Issue body §1."""

    # ---------- PAST ----------------------------------------------------

    def test_ustanovil_triggers(self):
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        for v in ("установил", "установила", "установили"):
            hit = detect_universal_action_claim(
                spoken=f"Я {v} пакет заново.",
                tools_called=(),
            )
            self.assertIsNotNone(hit, f"verb {v!r} not detected (past)")
            self.assertEqual(hit.tense, "past")

    def test_ostanovil_triggers(self):
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        for v in ("остановил", "остановила", "остановили"):
            hit = detect_universal_action_claim(
                spoken=f"Я {v} проигрывание.",
                tools_called=(),
            )
            self.assertIsNotNone(hit, f"verb {v!r} not detected (past)")
            self.assertEqual(hit.tense, "past")

    def test_podlozhil_triggers(self):
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        for v in ("подложил", "подложила", "подложили"):
            hit = detect_universal_action_claim(
                spoken=f"Я {v} второй голос сверху.",
                tools_called=(),
            )
            self.assertIsNotNone(hit, f"verb {v!r} not detected (past)")

    def test_pereklyuchil_triggers(self):
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        for v in ("переключил", "переключила", "переключили"):
            hit = detect_universal_action_claim(
                spoken=f"Я {v} режим на джангл.",
                tools_called=(),
            )
            self.assertIsNotNone(hit, f"verb {v!r} not detected (past)")

    def test_pochinil_triggers(self):
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        for v in ("починил", "починила", "починили"):
            hit = detect_universal_action_claim(
                spoken=f"Я {v} канал связи.",
                tools_called=(),
            )
            self.assertIsNotNone(hit, f"verb {v!r} not detected (past)")

    def test_perezagruzil_triggers(self):
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        for v in ("перезагрузил", "перезагрузила", "перезагрузили"):
            hit = detect_universal_action_claim(
                spoken=f"Я {v} ноду.",
                tools_called=(),
            )
            self.assertIsNotNone(hit, f"verb {v!r} not detected (past)")

    # ---------- FUTURE --------------------------------------------------

    def test_ustanovlyu_triggers(self):
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        hit = detect_universal_action_claim(
            spoken="Дай минуту, установлю пакет заново.",
            tools_called=(),
        )
        assert hit is not None  # noqa: S101
        self.assertEqual(hit.tense, "future")
        self.assertEqual(hit.verb.lower(), "установлю")

    def test_ostanovlyu_triggers(self):
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        hit = detect_universal_action_claim(
            spoken="Подожди, остановлю проигрывание.",
            tools_called=(),
        )
        assert hit is not None  # noqa: S101
        self.assertEqual(hit.tense, "future")
        self.assertEqual(hit.verb.lower(), "остановлю")

    def test_podlozhu_triggers(self):
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        hit = detect_universal_action_claim(
            spoken="Сейчас подложу второй голос сверху.",
            tools_called=(),
        )
        assert hit is not None  # noqa: S101
        self.assertEqual(hit.tense, "future")
        self.assertEqual(hit.verb.lower(), "подложу")

    def test_pereklyuchu_triggers(self):
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        hit = detect_universal_action_claim(
            spoken="Сейчас переключу режим на джангл.",
            tools_called=(),
        )
        assert hit is not None  # noqa: S101
        self.assertEqual(hit.tense, "future")
        self.assertEqual(hit.verb.lower(), "переключу")

    def test_podkruchu_triggers(self):
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        hit = detect_universal_action_claim(
            spoken="Сейчас подкручу громкость.",
            tools_called=(),
        )
        assert hit is not None  # noqa: S101
        self.assertEqual(hit.tense, "future")
        self.assertEqual(hit.verb.lower(), "подкручу")

    def test_synonyms_future_trigger(self):
        """Issue body §1 synonyms: поправлю/починю/перезагружу."""
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        for v in ("поправлю", "починю", "перезагружу"):
            hit = detect_universal_action_claim(
                spoken=f"Дай минуту, я {v}.",
                tools_called=(),
            )
            self.assertIsNotNone(hit, f"verb {v!r} not detected (future)")
            self.assertEqual(hit.tense, "future")

    # ---------- Integration: new verbs compose with tool whitelist -----

    def test_new_verb_with_music_tool_does_not_trigger(self):
        """Если LLM вызвала compose_music — guard НЕ ретраит даже на новые
        verbs."""
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        hit = detect_universal_action_claim(
            spoken="Установил пресет джангла.",
            tools_called=("set_vibe_preset",),
        )
        self.assertIsNone(hit)

    def test_new_verb_without_tool_triggers(self):
        """Issue body §1 acceptance: новый verb без tool → ретрай."""
        from rob_box_voice.core.dialogue_guards import \
            detect_universal_action_claim

        hit = detect_universal_action_claim(
            spoken="Переключу на джангл через секунду.",
            tools_called=(),
        )
        assert hit is not None  # noqa: S101
        self.assertEqual(hit.verb.lower(), "переключу")


if __name__ == "__main__":
    unittest.main()
