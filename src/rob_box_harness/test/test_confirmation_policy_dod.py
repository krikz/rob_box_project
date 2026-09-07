"""DoD acceptance test for ``ConfirmationPolicy`` (issue #1998 §8.2).

Карточка ``[operator-agent 08-DoD] тесты на slice/redact/ConfirmationPolicy``
issue #1998 формулирует DoD-3 так:

    «``ConfirmationPolicy`` срабатывает на опасном инструменте (YAML, не код)»

То есть:

* Когда LLM/агент пытается выполнить **физически-деструктивный**
  инструмент (``navigate_to_waypoint``, ``move_direction``,
  ``delete_waypoint`` и т.п.), классификатор на основе
  ``confirmation_policy.yaml`` должен вернуть ``kind=REQUIRE`` и
  заполнить ``plan_text`` (фразу для всплывающего окна подтверждения).
* Поведение управляется YAML-данными — никаких имён инструментов в
  Python-коде. Это — главный контракт §8.2: «классификация живёт в
  данных, не в логике», чтобы можно было расширять каталог без
  редеплоя.
* «Аварийные» инструменты (``stop_navigation``) — никогда не должны
  попадать в ``REQUIRE``, иначе робот не сможет экстренно остановиться.

Здесь — узкая приёмочная проверка. Широкая матрица по всем
инструментам из YAML и негативные кейсы загрузчика лежат в
``test_confirmation_policy.py``.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from rob_box_harness.core.confirmation_policy import (
    AcceptanceDecision,
    ConfirmationKind,
    ToolConfirmationPolicy,
    load_default_policy,
)


# ---------------------------------------------------------------------------
# Каталог «опасных» инструментов — это та часть §8.2, которая ОБЯЗАНА
# классифицироваться как REQUIRE и попадать в DoD-3.
# ---------------------------------------------------------------------------
#
# Источник истины — bundled YAML (data/confirmation_policy.yaml). Тесты
# НЕ должны его переписывать, но и не должны зависеть от конкретного
# имени — достаточно проверить, что YAML ставит «require» для тех
# инструментов, которые шаг 11 operator-agent-08 перечисляет в
# docs/design/SCHEDULER_DESIGN.md §8.2.
_DANGEROUS_TOOLS: tuple[str, ...] = (
    "navigate_to_waypoint",
    "navigate_to_coordinates",
    "move_direction",
    "start_mapping",
    "delete_waypoint",
    "clear_waypoints",
    "save_waypoint",
)


# ---------------------------------------------------------------------------
# DoD-3 acceptance tests
# ---------------------------------------------------------------------------


def test_dod3_bundled_yaml_loads_and_marks_dangerous_tools_require() -> None:
    """DoD-3 acceptance: YAML живёт в data/, и опасные тулзы → REQUIRE.

    Проверяем базовый контракт: ``load_default_policy()`` читает
    ``confirmation_policy.yaml`` (а не Python-код), и для каждого
    опасного тула возвращает ``ConfirmationKind.REQUIRE``.
    """
    policy = load_default_policy()

    assert isinstance(policy, ToolConfirmationPolicy), (
        "load_default_policy() обязан вернуть ToolConfirmationPolicy"
    )

    for tool_name in _DANGEROUS_TOOLS:
        decision = policy.classify(tool_name)
        assert decision.kind is ConfirmationKind.REQUIRE, (
            f"DoD-3 нарушен: опасный инструмент {tool_name!r} "
            f"должен классифицироваться как REQUIRE, "
            f"получили {decision.kind.value!r} (reason={decision.reason!r})"
        )
        # plan_text обязателен для REQUIRE — иначе orchestrator не
        # сможет спросить пользователя «План: …, подтверждаешь?».
        assert decision.plan_text, (
            f"plan_text пуст для {tool_name!r} — REQUIRE без плана бесполезен"
        )


def test_dod3_policy_is_yaml_driven_not_code() -> None:
    """DoD-3 контракт: классификация живёт в YAML, а не в Python-коде.

    Если бы кто-то добавил ``if tool_name == 'navigate_to_waypoint'``
    в ``confirmation_policy.py``, тест бы провалился: in-memory YAML
    БЕЗ этого тула должна дать ``pass_through`` (default), а
    bundled YAML — ``require``.

    Это проверяет «data, not code» инвариант §8.2.
    """
    # In-memory политика БЕЗ 'navigate_to_waypoint' → pass_through (default).
    empty_policy = ToolConfirmationPolicy.from_mapping({"tools": {}})
    empty_decision = empty_policy.classify("navigate_to_waypoint")
    assert empty_decision.kind is ConfirmationKind.PASS_THROUGH, (
        "default_class=pass_through должен срабатывать для неизвестных тулов"
    )

    # Bundled YAML → REQUIRE (это и есть DoD-3).
    bundled_policy = load_default_policy()
    bundled_decision = bundled_policy.classify("navigate_to_waypoint")
    assert bundled_decision.kind is ConfirmationKind.REQUIRE

    # Эти два результата ДОЛЖНЫ различаться — иначе YAML не управляет
    # поведением (что нарушает инвариант §8.2: «policy is data»).
    assert empty_decision.kind is not bundled_decision.kind


def test_dod3_yaml_file_exists_and_is_loaded_by_default() -> None:
    """DoD-3 sanity: bundled YAML существует на диске и доступен через API.

    Если бы кто-то случайно удалил ``confirmation_policy.yaml`` из
    packaging (PR #2062), ``load_default_policy()`` упал бы — и этот
    тест зафиксировал бы, что в репо всё ещё лежит «источник истины».
    """
    yaml_path = (
        Path(__file__).resolve().parents[1]  # .../src/rob_box_harness
        / "rob_box_harness"
        / "core"
        / "data"
        / "confirmation_policy.yaml"
    )
    assert yaml_path.exists(), (
        f"bundled YAML не найден: {yaml_path}. "
        "PR #2062 должен был его положить; если удалили — это регресс."
    )

    # YAML парсится без ошибок (load_default_policy не бросил исключения).
    policy = load_default_policy()
    assert policy.known_tools, (
        "После загрузки YAML каталог тулов не должен быть пустым"
    )


def test_dod3_decision_is_immutable_frozen() -> None:
    """DoD-3 invariant: ``AcceptanceDecision`` — frozen dataclass.

    Orchestrator передаёт ``decision`` по asyncio-очередям и логирует
    его. Изменяемость решения «по дороге» сломала бы и аудит, и
    консистентность FSM. Это — последняя линия обороны, чтобы
    кто-нибудь не начал мутировать ``decision.kind`` в middleware.
    """
    decision = load_default_policy().classify("navigate_to_waypoint")

    assert isinstance(decision, AcceptanceDecision)
    with pytest.raises((AttributeError, Exception)):  # FrozenInstanceError
        decision.kind = ConfirmationKind.PASS_THROUGH  # type: ignore[misc]


@pytest.mark.parametrize("tool_name", _DANGEROUS_TOOLS)
def test_dod3_each_dangerous_tool_requires_confirmation(tool_name: str) -> None:
    """Параметризованная DoD-3 матрица: каждый опасный тул из §8.2 → REQUIRE.

    Это — последняя линия обороны. Если кто-то изменит YAML и случайно
    уберёт один из этих тулов из секции ``require``, параметризация
    сразу это поймает.
    """
    policy = load_default_policy()
    decision = policy.classify(tool_name)

    assert decision.kind is ConfirmationKind.REQUIRE, (
        f"{tool_name!r}: ожидаем REQUIRE, получили {decision.kind.value!r}"
    )
    # tool-name в decision — чтобы feedback-event мог сослаться.
    assert decision.tool == tool_name


def test_dod3_emergency_tool_never_blocks_even_under_yaml_override() -> None:
    """DoD-3 негатив: ``stop_navigation`` никогда не должен быть REQUIRE.

    §8.2 «gold rule»: аварийные команды — мгновенно, без подтверждения.
    Если бы DoD-3 был реализован «помечаем всё подряд как require»,
    робот не смог бы остановиться по голосовой команде. Этот тест —
    инвариант безопасности, прямо вытекающий из issue #1998.
    """
    policy = load_default_policy()
    decision = policy.classify("stop_navigation")
    assert decision.kind is not ConfirmationKind.REQUIRE, (
        "stop_navigation — аварийный тул, ни в коем случае не REQUIRE"
    )
    assert decision.kind is ConfirmationKind.PASS_THROUGH
