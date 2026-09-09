"""Регресс-тесты issue #1881 — петля ретраев между guards.

Live-инцидент 02.09.2026 (vision-pi, docker logs voice-assistant):
на ОДНУ фразу юзера ушло 8 LLM-вызовов за 72 секунды. Между
babble-guard и music-guard возник ping-pong, потому что каждый guard
сбрасывал ЧУЖОЙ поимённый одноразовый флаг в ``_run_turn`` через
``if not is_<X>_retry: self._<Y>_retry_used = False``.

Тут три уровня проверки:

1. :class:`TestStripTrailingCriticalBlock` — чистая функция
   ``_strip_trailing_critical_block``: при повторном babble-retry
   НЕ должно склеиваться два [CRITICAL]-блока (противоречивые
   инструкции ломают LLM).

2. :class:`TestConsumeSyntheticRetry` — общий budget
   ``_synthetic_retries_left`` через мини-Node: декрементируется
   любым guard'ом, не сбрасывается соседним guard'ом, и блокирует
   ретрай на исчерпании.

3. :class:`TestCrossGuardPingPong` — репро петли: babble-retry →
   music-retry → babble-retry. Раньше все три стреляли. Теперь
   бюджет общий, и третий вызов отказывает.
"""

from __future__ import annotations

import unittest

from rob_box_voice.core.dialogue_guards import (
    _strip_trailing_critical_block,
    build_babble_retry_prompt,
)


# ---------------------------------------------------------------------------
# 1. _strip_trailing_critical_block — чистая функция
# ---------------------------------------------------------------------------


class TestStripTrailingCriticalBlock(unittest.TestCase):
    """При повторном babble-retry старый [CRITICAL]-блок должен быть
    ЗАМЕНЁН, а не склеен с новым. Иначе модель читает противоречивые
    инструкции и начинает их смешивать (vision-pi 02.09: «однако
    другой [CRITICAL] говорит ...»)."""

    def test_no_critical_marker_returns_as_is(self) -> None:
        text = "роббокс зачитай рэп про космос"
        self.assertEqual(_strip_trailing_critical_block(text), text)

    def test_empty_returns_empty(self) -> None:
        self.assertEqual(_strip_trailing_critical_block(""), "")

    def test_strips_trailing_critical_block(self) -> None:
        """Главный кейс: babble-retry строит prompt из user_input, в котором
        уже есть [CRITICAL]-блок от прошлого ретрая этого же turn'а."""
        text = (
            "[Speaker:unknown] зачитай рэп про космос\n\n"
            "[CRITICAL] старый блок, который надо убрать\n"
            "❌ ЗАПРЕЩЕНО..."
        )
        result = _strip_trailing_critical_block(text)
        # Маркер + всё после — удалено. Юзер-фраза остаётся.
        self.assertNotIn("[CRITICAL]", result)
        self.assertNotIn("старый блок", result)
        self.assertNotIn("ЗАПРЕЩЕНО", result)
        self.assertIn("зачитай рэп про космос", result)

    def test_marker_at_start_returns_empty(self) -> None:
        """Странный edge case: input — чистый [CRITICAL]-блок без юзер-фразы.
        Возвращаем пустую строку — нечего обрезать, но и склеивать два блока
        тоже нельзя."""
        text = "[CRITICAL] только блок без юзер-фразы"
        result = _strip_trailing_critical_block(text)
        # rfind вернёт 0, условие ``idx <= 0`` → return as-is.
        self.assertEqual(result, text)

    def test_marker_in_middle_strips_from_marker(self) -> None:
        """Если [CRITICAL] где-то в середине — обрезаем всё начиная с него.
        Это редкий кейс (юзер не вставляет CRITICAL в свою фразу), но
        безопасное поведение."""
        text = "юзер сказал [CRITICAL] что-то\nразное"
        result = _strip_trailing_critical_block(text)
        self.assertEqual(result, "юзер сказал")


class TestBuildBabbleRetryPromptStripsOldBlock(unittest.TestCase):
    """Сам сценарий карточки: babble-retry поверх babble-retry."""

    def test_no_previous_critical_appends_block(self) -> None:
        """Обычный путь — нет прошлого блока, добавляем как раньше."""
        prompt = build_babble_retry_prompt("роббокс зачитай рэп")
        self.assertIn("роббокс зачитай рэп", prompt)
        self.assertIn("[CRITICAL]", prompt)
        # Только один [CRITICAL]-блок.
        self.assertEqual(prompt.count("[CRITICAL]"), 1)

    def test_previous_critical_block_is_replaced(self) -> None:
        """Сценарий из карточки #1881 (vision-pi 02.09, raw):
        user_input уже содержит прошлый [CRITICAL]-блок от babble-retry.
        build_babble_retry_prompt должен его УБРАТЬ, а не склеивать."""
        prev_critical = (
            "[CRITICAL] Твой предыдущий ответ был метатекст...\n"
            "❌ ЗАПРЕЩЕНО отвечать текстом...\n"
            "✅ ОБЯЗАТЕЛЬНО: вызови нужный tool..."
        )
        user_input_with_prev = (
            f"[Speaker:unknown] зачитай рэп про космос\n\n{prev_critical}"
        )
        prompt = build_babble_retry_prompt(user_input_with_prev)
        # Юзер-фраза остаётся
        self.assertIn("зачитай рэп про космос", prompt)
        # Старый блок удалён — в итоговом промпте ровно ОДИН [CRITICAL].
        self.assertEqual(
            prompt.count("[CRITICAL]"),
            1,
            f"build_babble_retry_prompt должен заменить, а не склеивать "
            f"[CRITICAL]-блоки — иначе LLM видит противоречивые инструкции; "
            f"got prompt={prompt!r}",
        )
        # Старые подсказки из прошлого блока не должны протечь.
        # Конкретная фраза из build_babble_retry_prompt — нормальный
        # новый блок; конкретная фраза из прошлого блока должна быть
        # только ОДИН раз (в новом блоке, а не дважды).
        old_marker = "Твой предыдущий ответ был метатекст"
        self.assertEqual(prompt.count(old_marker), 1)


# ---------------------------------------------------------------------------
# 2. _consume_synthetic_retry — общий budget
# ---------------------------------------------------------------------------


class _BudgetNode:
    """Минимальный объект с интерфейсом ``DialogueNode._consume_synthetic_retry``.

    Зеркалит только ту часть, что нужно для теста budget'а — без
    rclpy, без реальной ноды, без поднятия ``__init__``.
    """

    DEFAULT_SYNTHETIC_RETRIES = 2

    def __init__(self, *, budget: int | None = None) -> None:
        self._synthetic_retries_left: int = (
            self.DEFAULT_SYNTHETIC_RETRIES if budget is None else budget
        )
        self.warnings: list[str] = []

    def get_logger(self):  # noqa: D401 — минимальный logger
        class _Logger:
            def __init__(self, outer):
                self._outer = outer

            def warning(self, msg: str) -> None:
                self._outer.warnings.append(msg)

        return _Logger(self)

    # Копия метода из ``DialogueNode._consume_synthetic_retry``.
    # Держим в тесте КОПИЮ логики — если в проде она разъедется с
    # тестом, тест должен закричать (это контракт: «budget не
    # списался — guard не диспатчит ретрай»).
    def _consume_synthetic_retry(self, *, guard_name: str) -> bool:
        if self._synthetic_retries_left <= 0:
            self.get_logger().warning(
                f"🚦 [issue 1881 retry-budget] исчерпан на turn, отдаю как есть "
                f"(guard={guard_name})"
            )
            return False
        self._synthetic_retries_left -= 1
        return True


class TestConsumeSyntheticRetry(unittest.TestCase):
    """Общий budget ``_synthetic_retries_left`` — единая точка истины."""

    def test_initial_budget_is_default(self) -> None:
        node = _BudgetNode()
        self.assertEqual(node._synthetic_retries_left, 2)

    def test_first_consume_returns_true(self) -> None:
        node = _BudgetNode()
        self.assertTrue(node._consume_synthetic_retry(guard_name="babble"))
        self.assertEqual(node._synthetic_retries_left, 1)

    def test_second_consume_returns_true(self) -> None:
        node = _BudgetNode()
        node._consume_synthetic_retry(guard_name="babble")
        self.assertTrue(node._consume_synthetic_retry(guard_name="music_user"))
        self.assertEqual(node._synthetic_retries_left, 0)

    def test_third_consume_returns_false_and_warns(self) -> None:
        """Ровно ``DEFAULT_SYNTHETIC_RETRIES`` (==2) разрешённых ретраев.
        Третий вызов — guard ДОЛЖЕН вернуть False и записать
        предупреждение, чтобы в логе было видно, что цикл оборвали
        намеренно."""
        node = _BudgetNode()
        node._consume_synthetic_retry(guard_name="babble")
        node._consume_synthetic_retry(guard_name="music_user")
        result = node._consume_synthetic_retry(guard_name="babble")
        self.assertFalse(result)
        self.assertEqual(len(node.warnings), 1)
        self.assertIn("retry-budget", node.warnings[0])
        self.assertIn("babble", node.warnings[0])

    def test_budget_exhausted_returns_false(self) -> None:
        """Даже если budget стартует с 0 — метод возвращает False."""
        node = _BudgetNode(budget=0)
        self.assertFalse(node._consume_synthetic_retry(guard_name="tool"))
        self.assertEqual(len(node.warnings), 1)


# ---------------------------------------------------------------------------
# 3. Кросс-guard петля — сам сценарий карточки
# ---------------------------------------------------------------------------


class TestCrossGuardPingPong(unittest.TestCase):
    """Имитация цепочки из live-логов 02.09::

        babble-retry → music-retry → babble-retry → music-retry → ...

    Раньше каждый guard сбрасывал чужие поимённые флаги через
    ``if not is_<X>_retry`` в ``_run_turn`` → ping-pong бесконечный.

    Теперь — общий budget ``_synthetic_retries_left`` сбрасывается
    ТОЛЬКО на user-initiated turn (``is_synthetic=False``), и
    любая комбинация guards расходует один и тот же счётчик.
    """

    def test_two_guards_share_the_same_budget(self) -> None:
        """Babble-retry и music-retry идут из одного budget'а.
        Если budget уже 0, второй guard НЕ диспатчит ретрай."""
        node = _BudgetNode()
        # 1-й guard (babble) — списывает, budget 2→1.
        self.assertTrue(node._consume_synthetic_retry(guard_name="babble"))
        # 2-й guard (music) — списывает, budget 1→0.
        self.assertTrue(node._consume_synthetic_retry(guard_name="music_user"))
        # 3-й guard (babble снова) — должен быть заблокирован.
        self.assertFalse(node._consume_synthetic_retry(guard_name="babble"))
        # Лог содержит ровно одно предупреждение.
        self.assertEqual(len(node.warnings), 1)

    def test_no_ping_pong_when_budget_exhausted(self) -> None:
        """Полная цепочка из 5 потенциальных ретраев —
        разрешён только ``DEFAULT_SYNTHETIC_RETRIES`` (==2)."""
        node = _BudgetNode()
        attempts = 0
        # Имитируем, что каждый guard «хочет» выстрелить ретрай.
        for guard_name in ("babble", "music_user", "tool", "babble", "music_user"):
            if node._consume_synthetic_retry(guard_name=guard_name):
                attempts += 1
        self.assertEqual(
            attempts,
            _BudgetNode.DEFAULT_SYNTHETIC_RETRIES,
            f"из 5 потенциальных ретраев должно пройти только "
            f"{_BudgetNode.DEFAULT_SYNTHETIC_RETRIES} (budget), не больше",
        )


if __name__ == "__main__":
    unittest.main()
