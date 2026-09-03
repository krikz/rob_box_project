"""
test_issue_1895_dj_retry_budget_chain.py — репро цепочки DJ-tick → DJ_RETRY → babble.

Issue #1895 (follow-up к #1881): ``MusicGuardVerdictKind.DJ_RETRY`` —
такой же синтетический ретрай, как и остальные guard'ы. До фикса он
диспатчил ``_dispatch_dj_turn(verdict.prompt)`` без
``_consume_synthetic_retry(guard_name="music_dj")``, поэтому:

  * ``_run_turn`` сбрасывал общий budget ``_synthetic_retries_left`` в
    ``DEFAULT_SYNTHETIC_RETRIES`` (ветка ``if not is_synthetic:``), и
    каждый DJ_RETRY начинал отсчёт заново;
  * в комбинации с babble-retry это давало до 9 LLM-вызовов на ОДИН
    DJ-переход без единого слова юзера (см. live-логи #1881, vision-pi
    02.09, raw).

Здесь проверяем, что DJ_RETRY делит общий budget с babble/tool/code
guard'ами — сколько бы ни «хотел» выстрелить DJ-guard, после
``DEFAULT_SYNTHETIC_RETRIES`` суммарных списаний он обязан вернуть
``False``.
"""

from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


# Import the sibling _BudgetNode harness from #1881 without relying on
# pytest's collection layout (it imports as a top-level module only when
# pytest is run from the directory that contains it). Loading by file
# path keeps this test independent of conftest / pytest rootdir.
_1881_PATH = Path(__file__).with_name("test_issue_1881_synthetic_retry_budget.py")
_spec = importlib.util.spec_from_file_location(
    "test_issue_1881_synthetic_retry_budget", _1881_PATH,
)
_1881 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_1881)  # type: ignore[union-attr]
_BudgetNode = _1881._BudgetNode


# ---------------------------------------------------------------------------
# Цепочка из acceptance #1895
# ---------------------------------------------------------------------------


class TestDjRetrySharesBudgetWithOtherGuards(unittest.TestCase):
    """DJ_RETRY декрементит общий budget — не «свежий» счётчик."""

    def test_dj_retry_decrements_shared_budget(self) -> None:
        """Один DJ_RETRY декрементит ``_synthetic_retries_left`` ровно
        один раз — наравне с babble/tool guard'ами."""
        node = _BudgetNode()
        before = node._synthetic_retries_left
        result = node._consume_synthetic_retry(guard_name="music_dj")
        self.assertTrue(
            result,
            "DJ_RETRY должен списывать бюджет — иначе ping-pong DJ↔babble",
        )
        self.assertEqual(
            node._synthetic_retries_left,
            before - 1,
            "DJ_RETRY списывает ровно один слот из общего бюджета",
        )

    def test_dj_retry_blocked_after_budget_exhausted(self) -> None:
        """Когда budget=0 — DJ_RETRY возвращает False и НЕ диспатчит
        ретрай. Без этого guard продолжал бы пинг-понгом после
        исчерпания budget'а другими guard'ами."""
        node = _BudgetNode(budget=0)
        self.assertFalse(
            node._consume_synthetic_retry(guard_name="music_dj"),
            "DJ_RETRY на пустом бюджете должен возвращать False — "
            "иначе guard-цепочка уходит в бесконечный цикл",
        )
        # Budget остаётся нулевым — никаких побочных декрементов.
        self.assertEqual(node._synthetic_retries_left, 0)
        # Предупреждение для логов.
        self.assertEqual(len(node.warnings), 1)
        self.assertIn("music_dj", node.warnings[0])


class TestDjRetryChainAcceptance(unittest.TestCase):
    """Acceptance #1895: цепочка DJ-tick → DJ_RETRY → babble —
    суммарно НЕ больше ``DEFAULT_SYNTHETIC_RETRIES`` синтетических
    ретраев на один переход.

    Раньше DJ_RETRY НЕ списывал общий budget → ping-pong мог
    крутиться до бесконечности (vision-pi 02.09, до 9 LLM-вызовов).
    Теперь budget — единая точка истины для ВСЕХ guard'ов.
    """

    def test_full_dj_to_babble_chain_caps_at_default(self) -> None:
        """Самая тяжёлая цепочка из acceptance:

            DJ-tick → DJ_RETRY (1) → babble-retry (2) → DJ_RETRY (3) →
            tool-retry (4) → ...

        Должна быть ОБРЕЗАНА после ``DEFAULT_SYNTHETIC_RETRIES`` (==2)
        удачных ретраев. С третьей попытки любой guard возвращает
        False.
        """
        node = _BudgetNode()
        successful: list[str] = []
        # Имитируем «каждый guard хочет ретрай»: DJ-music, babble,
        # DJ-music, tool — это ровно 4 попытки, как в реальной
        # петле из live-логов.
        sequence = ("music_dj", "babble", "music_dj", "tool")
        for guard_name in sequence:
            if node._consume_synthetic_retry(guard_name=guard_name):
                successful.append(guard_name)

        self.assertEqual(
            len(successful),
            _BudgetNode.DEFAULT_SYNTHETIC_RETRIES,
            f"DJ-цепочка должна срезаться на {_BudgetNode.DEFAULT_SYNTHETIC_RETRIES}; "
            f"прошло {len(successful)}: {successful!r}",
        )
        # Первые два — всегда (порядок в sequence). Третий (music_dj) —
        # заблокирован.
        self.assertEqual(successful, ["music_dj", "babble"])
        # Два варнинга — про оба отказа (3-й и 4-й guard).
        self.assertEqual(
            len(node.warnings),
            2,
            f"должно быть ровно 2 предупреждения (3-й и 4-й guard), "
            f"не {len(node.warnings)}: {node.warnings!r}",
        )
        self.assertTrue(
            any("music_dj" in w for w in node.warnings),
            "хотя бы один варнинг должен быть от music_dj guard'а",
        )

    def test_dj_retry_does_not_double_dip_into_budget(self) -> None:
        """DJ_RETRY не должен мутировать budget дважды за один вызов
        guard'а (защита от регрессии: если кто-то «усилит» декремент —
        тест это поймает)."""
        node = _BudgetNode()
        before = node._synthetic_retries_left
        node._consume_synthetic_retry(guard_name="music_dj")
        after = node._synthetic_retries_left
        self.assertEqual(
            before - after,
            1,
            f"DJ_RETRY за один вызов списывает ровно 1 слот, "
            f"не {before - after}",
        )

    def test_dj_retry_resets_only_on_user_turn(self) -> None:
        """После исчерпания budget'а DJ_RETRY не должен «обнулять»
        счётчик обратно в DEFAULT_SYNTHETIC_RETRIES.

        В старом коде сброс жил в ``_run_turn`` под
        ``if not is_synthetic:`` — DJ_RETRY приходил с
        ``is_synthetic=False`` (потому что ``_dispatch_dj_turn`` шёл
        через ``_dispatch_turn(is_dj_auto=True)``), и сброс СРАБАТЫВАЛ
        каждый раз. Поэтому в live-логах budget=2 мог откатиться
        после первого же DJ_RETRY.

        Здесь мы НЕ моделируем reset (это работа ``_run_turn``, не
        ``_consume_synthetic_retry``). Тест проверяет, что
        ``_consume_synthetic_retry`` сам по себе не возвращает budget
        в DEFAULT после исчерпания.
        """
        node = _BudgetNode()
        node._consume_synthetic_retry(guard_name="music_dj")  # 2→1
        node._consume_synthetic_retry(guard_name="music_dj")  # 1→0
        self.assertEqual(node._synthetic_retries_left, 0)
        # Ещё раз — не должен списать «отрицательно» и не должен
        # выставить budget обратно в DEFAULT.
        node._consume_synthetic_retry(guard_name="music_dj")
        self.assertEqual(
            node._synthetic_retries_left,
            0,
            "DJ_RETRY на пустом бюджете НЕ сбрасывает budget обратно",
        )


if __name__ == "__main__":
    unittest.main()
