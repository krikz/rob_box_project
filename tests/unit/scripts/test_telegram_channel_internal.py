"""В постах не пишут внутреннюю кухню проекта (Шифу, 07.11).

«В постах не нужно писать про пиары ветки и детали из репозитория и исью
пока мы это не пишем» — канал про поведение робота, а не про PR-ы.
"""

import sys
from datetime import datetime, timezone
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "telegram"))

import channel_lib as lib  # noqa: E402
import studio_text as text  # noqa: E402

BAD = [
    "PR #4503 влит в develop: сброс ICP-одометрии",
    "Оба исправления относятся к #4495",
    "Влили #4502 в develop",
    "Подробности в issue #4492",
    "Ссылка: https://github.com/krikz/rob_box_project/pull/4504",
    "Изменения в ветке develop",
    "Параметры задаёт patch_rtabmap_launch.py",
    "Удалены rtabmap.ini и rtabmap.yaml",
    "Подробности в docs/process/telegram-channel.md",
    "Скрипт scripts/telegram/studio_text.py поправили",
]

GOOD = [
    "Робот перестал говорить «приехал», когда потерял опору",
    "После потери опоры одометрия сбрасывается сама",
    "На карте склада робокс больше не уплывает в сторону",
    "Теперь потеря опоры и обычное прибытие различаются",
]


def internal_problems(body: str) -> list:
    """Что скажет валидатор студии про «внутренности» в тексте."""
    tags = ["#роббокс", "#дневник"]
    return [
        p
        for p in text.validate_post(body + " #роббокс #дневник", tags, [], has_media=False)
        if p.startswith("в тексте")
    ]


@pytest.mark.parametrize("body", BAD)
def test_internal_details_are_rejected(body):
    """PR-ы, ветки и файлы репозитория в пост не проходят."""
    problems = internal_problems(body)
    assert problems, f"прошло незаметно: {body}"


@pytest.mark.parametrize("body", GOOD)
def test_plain_behaviour_text_passes(body):
    """Обычный текст о поведении робота проходит."""
    assert internal_problems(body) == []


def _post(body: str, status: str = "approved") -> "lib.Post":
    return lib.Post(
        path=Path("099-test.md"),
        status=status,
        media="",
        not_before=datetime(2026, 11, 10, 10, tzinfo=timezone.utc),
        message_id=None,
        posted_at=None,
        hashtags=["#роббокс", "#дневник"],
        body=body + " #роббокс #дневник",
    )


def test_publisher_uses_the_same_rule(tmp_path):
    """Публикатор проверяет то же, что студия: правило общее."""
    problems = lib.validate(_post("В develop влили PR #4503"), tmp_path)
    assert any("номер PR" in p for p in problems), problems


def test_already_posted_is_not_rejected_retroactively(tmp_path):
    """Вышедшее до решения 07.11 (пост 003 с «#4487») не переписываем."""
    problems = lib.validate(_post("Лицо шлёт 25 кадров (после правки #4487)", status="posted"), tmp_path)
    assert not any("номер PR" in p for p in problems), problems


def test_forbidden_list_is_shared():
    """Список запрещённого — общий для студии и публикатора."""
    assert lib.FORBIDDEN_INTERNAL
    assert all(item in lib.POST_FORBIDDEN for item in lib.FORBIDDEN_INTERNAL + lib.FORBIDDEN)
