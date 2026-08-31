"""test_issue_1777_tool_skipped_guard.py — non-music tool-skipped retry guard.

Issue #1777 (главный кейс) — LLM отвечает на «который час?» без вызова
``get_current_time`` tool (tools=[]), говорит время из головы.

Issue #1762 (расширение) — то же для ``search_web``, ``set_voice``,
``memory_search``, ``faq_search``. Раньше ретрай (Bug C) работал только
для music (issue #992 Bug C). Теперь guard покрывает ВСЕ явные
tool-based запросы.

Это unit-тесты, которые НЕ поднимают полный run loop (как babble guard
в test_issue_992_babble_guard.py). Тестируется напрямую метод
``DialogueNode._apply_tool_skipped_guard`` через мини-DialogueNode с
подменой зависимостей (``_music_guard``, ``_dsm``, ``_dispatch_turn``,
``_reopen_dialogue_for_retry``).

Зачем отдельные тесты, а не e2e через ``test_dialogue_shell.py``:
* E2E прогон требует полного setup'а rclpy-ноды (mock subscriptions,
  publishers, executor) — это долго и хрупко.
* Логика guard-а чисто детерминирована (5 правил + детекция по
  keyword-ам), unit-тест покрывает 100% за 0.5с.
* E2E через e2e_voice_test.sh покроет реальный pipeline на Vision Pi
  после merge (см. e2e блок в PR).

Issue: #1777
"""

from __future__ import annotations

import unittest
from typing import Optional, Tuple

from rob_box_harness.core.dialogue_state_machine import (
    DialogueEvent,
    DialogueStateKind,
    DialogueStateMachine,
)


# ── Минимальный dialogue_node — переопределяем только нужные hooks ──────


class _DispatchCall:
    """Запись о вызове ``_dispatch_turn`` для assertion."""

    __slots__ = ("user_input", "raw_user_command", "was_idle", "is_babble_retry")

    def __init__(
        self,
        user_input: str,
        raw_user_command: Optional[str],
        was_idle: bool,
        is_babble_retry: bool,
    ) -> None:
        self.user_input = user_input
        self.raw_user_command = raw_user_command
        self.was_idle = was_idle
        self.is_babble_retry = is_babble_retry

    def __repr__(self) -> str:
        return (
            f"_DispatchCall(user_input={self.user_input[:50]!r}, "
            f"raw_user_command={self.raw_user_command[:30]!r}, "
            f"was_idle={self.was_idle}, is_babble_retry={self.is_babble_retry})"
        )


class _MiniDialogueNode:
    """Минимальный объект, повторяющий интерфейс ``DialogueNode``
    нужный для ``_apply_tool_skipped_guard``.

    Мы НЕ наследуемся от настоящего ``DialogueNode`` — у него слишком
    тяжёлый ``__init__`` (rclpy, executor, memory, tools). Вместо этого
    зеркалим минимальный state + методы.
    """

    def __init__(self) -> None:
        self._tool_retry_used: bool = False
        self._babble_retry_used: bool = False
        self._music_guard_retry_active_flag: bool = False
        self._dispatched: list[_DispatchCall] = []
        self._reopen_called: int = 0
        self.warnings: list[str] = []
        self._dsm = DialogueStateMachine(initial=DialogueStateKind.DIALOGUE)

    # ── hooks, которые тестируемый метод вызывает ──

    def get_logger(self):  # noqa: D401 — minimal logger
        class _Logger:
            def __init__(self, outer):
                self._outer = outer

            def warning(self, msg: str) -> None:
                self._outer.warnings.append(msg)

        return _Logger(self)

    def _reopen_dialogue_for_retry(self) -> None:
        # Реальный код вызывает _dsm.on_event(...). Следим только за вызовом.
        self._reopen_called += 1

    def _dispatch_turn(
        self,
        user_input: str,
        *,
        was_idle: bool = False,
        is_babble_retry: bool = False,
        raw_user_command: Optional[str] = None,
    ) -> None:
        self._dispatched.append(
            _DispatchCall(
                user_input=user_input,
                raw_user_command=raw_user_command,
                was_idle=was_idle,
                is_babble_retry=is_babble_retry,
            )
        )

    def _music_guard_retry_active(self) -> bool:
        return self._music_guard_retry_active_flag

    # ── сам тестируемый метод — копия из dialogue_node._apply_tool_skipped_guard ──

    def _apply_tool_skipped_guard(
        self,
        *,
        user_input: str,
        tools_called: tuple,
    ) -> bool:
        if tools_called:
            return False
        if not user_input:
            return False
        if self._tool_retry_used:
            return False
        if self._babble_retry_used or self._music_guard_retry_active():
            return False
        # Import here чтобы избежать тяжёлой загрузки на верхнем уровне.
        from rob_box_voice.core.dialogue_guards import (
            build_tool_retry_prompt,
            detect_required_tool,
        )
        tool_name = detect_required_tool(user_input)
        if not tool_name:
            return False
        retry_prompt = build_tool_retry_prompt(user_input, tool_name)
        if not retry_prompt:
            return False
        self._reopen_dialogue_for_retry()
        self._tool_retry_used = True
        self.get_logger().warning(
            f"🛠 [issue 1777 / 1762] LLM skip non-music tool {tool_name!r} — "
            f"retrying once with CRITICAL reminder (user={user_input[:60]!r})"
        )
        self._dispatch_turn(
            retry_prompt,
            was_idle=False,
            raw_user_command=user_input,
        )
        return True


# ── Unit-тесты ─────────────────────────────────────────────────────────────


class TestToolSkippedGuardTime(unittest.TestCase):
    """Issue #1777 — главный кейс: «который час» / «сколько времени»."""

    def test_time_request_dispatches_retry(self):
        node = _MiniDialogueNode()
        result = node._apply_tool_skipped_guard(
            user_input="который час",
            tools_called=(),
        )
        self.assertTrue(result)
        self.assertEqual(len(node._dispatched), 1)
        call = node._dispatched[0]
        # CRITICAL retry prompt with get_current_time, plus echoes user input.
        self.assertIn("[CRITICAL]", call.user_input)
        self.assertIn("get_current_time", call.user_input)
        self.assertEqual(call.raw_user_command, "который час")
        self.assertFalse(call.was_idle)
        # Flag взведён — повторный вызов НЕ должен ничего делать.
        self.assertTrue(node._tool_retry_used)

    def test_skolkoy_vremeni(self):
        node = _MiniDialogueNode()
        self.assertTrue(
            node._apply_tool_skipped_guard(
                user_input="сколько времени",
                tools_called=(),
            )
        )
        self.assertEqual(len(node._dispatched), 1)
        self.assertIn("сколько времени", node._dispatched[0].user_input)
        self.assertIn("get_current_time", node._dispatched[0].user_input)

    def test_vremya_v_moskve(self):
        node = _MiniDialogueNode()
        self.assertTrue(
            node._apply_tool_skipped_guard(
                user_input="время в москве",
                tools_called=(),
            )
        )
        self.assertIn("время в москве", node._dispatched[0].user_input)

    def test_kakaya_data(self):
        node = _MiniDialogueNode()
        self.assertTrue(
            node._apply_tool_skipped_guard(
                user_input="какая дата",
                tools_called=(),
            )
        )
        self.assertIn("get_current_time", node._dispatched[0].user_input)


class TestToolSkippedGuardSearch(unittest.TestCase):
    """Issue #1762 — расширение Bug C на search_web."""

    def test_pogoda_dispatches_search_web_retry(self):
        node = _MiniDialogueNode()
        self.assertTrue(
            node._apply_tool_skipped_guard(
                user_input="погода в бишкеке",
                tools_called=(),
            )
        )
        call = node._dispatched[0]
        self.assertIn("search_web", call.user_input)
        # Не подменять tool — search_web, не get_current_time.
        self.assertNotIn("get_current_time", call.user_input)

    def test_novosti(self):
        node = _MiniDialogueNode()
        self.assertTrue(
            node._apply_tool_skipped_guard(
                user_input="новости про биткоин",
                tools_called=(),
            )
        )
        self.assertIn("search_web", node._dispatched[0].user_input)


class TestToolSkippedGuardVoice(unittest.TestCase):
    """Issue #1765 — расширение Bug C на set_voice."""

    def test_perekljuchi_golos(self):
        node = _MiniDialogueNode()
        self.assertTrue(
            node._apply_tool_skipped_guard(
                user_input="переключи голос на арт",
                tools_called=(),
            )
        )
        call = node._dispatched[0]
        self.assertIn("set_voice", call.user_input)
        self.assertIn("list_voices", call.user_input)
        self.assertNotIn("get_current_time", call.user_input)


class TestToolSkippedGuardMemory(unittest.TestCase):
    """Issue #1770 — расширение Bug C на memory_search (с фильтром speaker)."""

    def test_chto_ty_znaesh_obо_mne(self):
        node = _MiniDialogueNode()
        self.assertTrue(
            node._apply_tool_skipped_guard(
                user_input="что ты знаешь обо мне",
                tools_called=(),
            )
        )
        call = node._dispatched[0]
        self.assertIn("memory_search", call.user_input)
        # Не подставлять факты чужих юзеров.
        self.assertIn("speaker_id", call.user_input)


class TestToolSkippedGuardFAQ(unittest.TestCase):
    """Расширение на faq_search."""

    def test_chto_ty_umeesh(self):
        node = _MiniDialogueNode()
        self.assertTrue(
            node._apply_tool_skipped_guard(
                user_input="что ты умеешь",
                tools_called=(),
            )
        )
        self.assertIn("faq_search", node._dispatched[0].user_input)


class TestToolSkippedGuardNegativeCases(unittest.TestCase):
    """Условия, при которых guard НЕ должен срабатывать."""

    def test_tools_called_non_empty_no_retry(self):
        """Если LLM всё-таки вызвал хоть один tool — ретрай не нужен."""
        node = _MiniDialogueNode()
        self.assertFalse(
            node._apply_tool_skipped_guard(
                user_input="который час",
                tools_called=("get_current_time",),
            )
        )
        self.assertEqual(node._dispatched, [])

    def test_empty_user_input_no_retry(self):
        node = _MiniDialogueNode()
        self.assertFalse(
            node._apply_tool_skipped_guard(
                user_input="",
                tools_called=(),
            )
        )
        self.assertEqual(node._dispatched, [])

    def test_non_tool_request_no_retry(self):
        """Чит-чат «как дела» НЕ должен ретраить — это обычный ответ."""
        node = _MiniDialogueNode()
        self.assertFalse(
            node._apply_tool_skipped_guard(
                user_input="как дела",
                tools_called=(),
            )
        )
        self.assertEqual(node._dispatched, [])

    def test_music_request_no_retry_from_tool_guard(self):
        """«Спой песенку» — это music guard (issue #992 Bug C), не этот guard."""
        node = _MiniDialogueNode()
        self.assertFalse(
            node._apply_tool_skipped_guard(
                user_input="спой песенку",
                tools_called=(),
            )
        )
        self.assertEqual(node._dispatched, [])

    def test_second_call_no_retry(self):
        """Защита от ping-pong: один ретрай на turn."""
        node = _MiniDialogueNode()
        # Первый вызов успешен.
        self.assertTrue(
            node._apply_tool_skipped_guard(
                user_input="который час",
                tools_called=(),
            )
        )
        # Второй вызов на том же turn (тул так и не вызвался) — молчим.
        self.assertFalse(
            node._apply_tool_skipped_guard(
                user_input="который час",
                tools_called=(),
            )
        )
        # Один ретрай, не два.
        self.assertEqual(len(node._dispatched), 1)

    def test_babble_retry_used_blocks_tool_retry(self):
        """Если babble guard уже задиспатчил ретрай, tool guard молчит —
        иначе два параллельных ретрай-тура → двойной вызов LLM."""
        node = _MiniDialogueNode()
        node._babble_retry_used = True
        self.assertFalse(
            node._apply_tool_skipped_guard(
                user_input="который час",
                tools_called=(),
            )
        )

    def test_music_guard_retry_active_blocks_tool_retry(self):
        """Аналогично для music guard (защита от двойных retry-туров)."""
        node = _MiniDialogueNode()
        node._music_guard_retry_active_flag = True
        self.assertFalse(
            node._apply_tool_skipped_guard(
                user_input="который час",
                tools_called=(),
            )
        )


class TestToolSkippedGuardHookInvariants(unittest.TestCase):
    """Side-effect invariants: каждый успешный guard-вызов обязан
    reopen + dispatch + log + взвести флаг."""

    def test_retry_reopens_dialogue(self):
        node = _MiniDialogueNode()
        node._apply_tool_skipped_guard(
            user_input="который час",
            tools_called=(),
        )
        self.assertEqual(node._reopen_called, 1)

    def test_retry_logs_warning(self):
        node = _MiniDialogueNode()
        node._apply_tool_skipped_guard(
            user_input="который час",
            tools_called=(),
        )
        self.assertEqual(len(node.warnings), 1)
        self.assertIn("get_current_time", node.warnings[0])

    def test_retry_passes_raw_user_command(self):
        """raw_user_command должен быть ОРИГИНАЛЬНЫМ user_input, не
        синтетическим промптом (как у babble guard — см. issue #1204)."""
        node = _MiniDialogueNode()
        node._apply_tool_skipped_guard(
            user_input="который час",
            tools_called=(),
        )
        call = node._dispatched[0]
        # synthetic prompt — для LLM.
        self.assertIn("get_current_time", call.user_input)
        # raw_user_command — для music guard retry-тура, чтобы он не
        # сканировал синтетический промпт.
        self.assertEqual(call.raw_user_command, "который час")
        self.assertNotEqual(call.raw_user_command, call.user_input)


class TestToolSkippedGuardFrame(unittest.TestCase):
    """Привязка к issue #1777 — все фиксы в этом frame должны
    проходить этот smoke-test."""

    def test_kotoryy_chas_hallucination_from_issue(self):
        """Сценарий из issue #1777 дословно:

            User: «Робот, который час?»
            LLM (БЕЗ нашего фикса): «Сейчас тридцать семь минут
                одиннадцатого вечера...» (tools=[])

        С нашим фиксом: guard видит явный tool-pattern «который час»,
        tools пуст → CRITICAL retry с указанием get_current_time.
        """
        node = _MiniDialogueNode()
        # Симулируем первый turn: LLM ответила из головы.
        result = node._apply_tool_skipped_guard(
            user_input="который час",
            tools_called=(),  # ← LLM не вызвала tool
        )
        self.assertTrue(result)
        # На retry-тур LLM должен увидеть явный CRITICAL reminder.
        retry_prompt = node._dispatched[0].user_input
        self.assertIn("get_current_time", retry_prompt)
        self.assertIn("Не выдумывай", retry_prompt)
        # И в логе остался след для оператора.
        self.assertIn("LLM skip non-music tool", node.warnings[0])


if __name__ == "__main__":
    unittest.main()
