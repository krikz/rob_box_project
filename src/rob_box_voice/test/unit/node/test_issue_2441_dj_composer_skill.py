"""test_issue_2441_dj_composer_skill.py — DJ_AUTO-ход обязан получать composer-знание.

issue #2441: режим диджея нёс свою копию инструкции по аранжировке
(tech_line/stage_line в build_auto_prompt) и на первом переходе не видел
composer.txt (lookup_melody / name= / RTTTL-темы). После фикса:

* build_auto_prompt не несёт copy-paste аранжировки (см. test_dramaturgy_fix_1016);
* DJ_AUTO-ход (is_dj_auto=True) ДЕТЕРМИНИРОВАННО активирует скилл composer
  через ``_activate_skill_for(force_skill="composer")`` — не полагаясь на
  regex-роутер по тексту синтетического промпта.

Проверка активного скилла — прямая (set_active_skill / содержимое composer.txt),
не косвенная regex-удача.
"""

from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock

from rob_box_voice.dialogue_node import DialogueNode


def _repo_root(start: Path) -> Path:
    for parent in [start, *start.parents]:
        if (parent / "docker").is_dir() and (parent / "src").is_dir():
            return parent
    return start.parents[5]


REPO_ROOT = _repo_root(Path(__file__).resolve())
COMPOSER_PROMPT = (
    REPO_ROOT / "src" / "rob_box_voice" / "prompts" / "skills" / "composer.txt"
)


def _make_node() -> DialogueNode:
    """DialogueNode через object.__new__ (как в остальных unit-тестах)."""
    node = object.__new__(DialogueNode)
    node.get_logger = lambda: MagicMock()
    node._skill_router = MagicMock()
    node._skill_router.route.return_value = "dj"
    node._core = MagicMock()
    return node


def test_dj_auto_forces_composer_skill() -> None:
    """DJ_AUTO-ход форсирует composer, а не regex-роутер.

    Роутер по тексту синтетического промпта может вернуть «dj» (на n=1
    текст содержит литерал «робот-диджей») — но DJ-переход обязан получать
    composer, где живёт lookup_melody/name=/RTTTL. force_skill обходит роутер.
    """
    node = _make_node()

    node._activate_skill_for("Ты робот-диджей...", force_skill="composer")

    node._core.set_active_skill.assert_called_once_with("composer")
    node._skill_router.route.assert_not_called()


def test_regular_turn_still_uses_the_router() -> None:
    """Обычный ход без force_skill по-прежнему полагается на роутер."""
    node = _make_node()

    node._activate_skill_for("включи диджея")

    node._skill_router.route.assert_called_once_with("включи диджея")
    node._core.set_active_skill.assert_called_once_with("dj")


def test_composer_fragment_carries_the_knowledge_dj_was_missing() -> None:
    """composer.txt содержит механику, которой DJ не знал до #2441.

    Прямая проверка источника знания: если DJ_AUTO-ход получает composer
    (см. test_dj_auto_forces_composer_skill), он получает и lookup_melody +
    name= — путь для известных мелодий по имени.
    """
    composer = COMPOSER_PROMPT.read_text(encoding="utf-8")

    for token in ("lookup_melody", "name="):
        assert token in composer, (
            f"{token!r} отсутствует в composer.txt — DJ-переход не увидит "
            "путь для известной мелодии по имени"
        )
