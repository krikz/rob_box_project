"""
test_system_prompt_tools_check.py — Issue #1409 / ARCH-review #1405,
обобщено change'ом skill-scoped-dialogue-context (фаза 4, задача 4.1).

Рантайм-страховка «инструменты против текста», которая раньше называлась
``DialogueNode._validate_tools_in_prompt`` и смотрела ТОЛЬКО
``music_skill_prompt.txt``. Класс расхождения общий, а не музыкальный:
#1403 — ``generate_music`` был зарегистрирован, но в тексте не упомянут,
и LLM уверенно отвечала «нет такой функции», уходя на другой путь.

Теперь проверка зовётся ``_validate_skill_fragments`` и проходит по ВСЕМ
загруженным доменным фрагментам. Блокером стал отдельный тест
(``test_skill_prompt_contract.py``) — здесь остаётся именно рантайм-
предупреждение: контейнер с рассинхронизированным промптом обязан
подняться и заговорить, а не упасть, но оператор должен увидеть в логе,
какой скилл разъехался.

Не требует ROS2 — rclpy замокан в conftest.py.
"""

from unittest.mock import MagicMock

import pytest

from rob_box_voice.dialogue_node import DialogueNode


# ─────────────────────────────────────────────────────────────────────────────
#  Fixtures
# ─────────────────────────────────────────────────────────────────────────────


@pytest.fixture
def node():
    """Минимальная DialogueNode без __init__ (как в test_pure_methods)."""
    n = object.__new__(DialogueNode)
    logger = MagicMock()
    n._logger = logger
    n.get_logger = lambda: logger
    return n


def _warnings(node) -> list[str]:
    return [str(call.args[0]) for call in node.get_logger().warning.call_args_list]


# ─────────────────────────────────────────────────────────────────────────────
#  Расхождение «инструмент есть — в тексте не назван»
# ─────────────────────────────────────────────────────────────────────────────


class TestValidateSkillFragments:
    def test_missing_tool_is_reported_with_skill_and_name(self, node):
        """Главный сценарий: инструмент скилла не описан во фрагменте.

        Регрессия класса #1403 — предупреждение обязано назвать И скилл,
        И конкретный инструмент, иначе по логу непонятно, что чинить.
        """
        node._validate_skill_fragments(
            {"scheduler": "Скилл планировщика без единого имени инструмента."}
        )

        warnings = _warnings(node)
        assert warnings, "расхождение не было залогировано"
        assert any("scheduler" in w and "task_delta" in w for w in warnings)

    def test_full_coverage_produces_no_warning(self, node):
        """Все инструменты названы → тихо."""
        node._validate_skill_fragments(
            {"scheduler": "Используй `task_delta` чтобы поправить задачу."}
        )

        assert not _warnings(node)

    def test_case_insensitive_match(self, node):
        """Имя в другом регистре считается упомянутым."""
        node._validate_skill_fragments({"scheduler": "Зови TASK_DELTA."})

        assert not _warnings(node)

    def test_every_skill_is_checked_not_just_music(self, node):
        """Ради чего задача 4.1: проверка больше не музыко-специфична."""
        node._validate_skill_fragments(
            {
                "scheduler": "пусто",
                "knowledge": "пусто",
            }
        )

        warnings = _warnings(node)
        assert any("scheduler" in w for w in warnings)
        assert any("knowledge" in w for w in warnings)

    def test_unknown_skill_name_is_reported_not_crashed(self, node):
        """Фрагмент, не соответствующий ни одному скиллу каталога.

        Такое приезжает, когда на робот попал промпт из другой сборки.
        Нода обязана подняться и предупредить, а не упасть.
        """
        node._validate_skill_fragments({"нет-такого-скилла": "текст"})

        warnings = _warnings(node)
        assert any("нет-такого-скилла" in w for w in warnings)

    def test_empty_mapping_skips_silently(self, node):
        """``skills_enabled=false`` → проверять нечего, лог чистый."""
        node._validate_skill_fragments({})

        assert not _warnings(node)

    def test_core_skill_checks_its_own_tools(self, node):
        """core описывает свои инструменты сам — он единственный такой."""
        node._validate_skill_fragments({"core": "Только про speak_text."})

        warnings = _warnings(node)
        assert any("core" in w and "get_battery_level" in w for w in warnings)

    def test_non_core_skill_is_not_required_to_describe_core_tools(self, node):
        """Иначе каждый фрагмент дублировал бы speak_text и статусные тулы."""
        node._validate_skill_fragments(
            {"scheduler": "Зови `task_delta`, и ничего больше."}
        )

        assert not _warnings(node)
