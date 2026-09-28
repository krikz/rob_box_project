"""Guard test for issue #3132: мораторий на новые регексы в музыкальных гуардах.

Контекст (``docs/design/2026-09-28-music-dj-systemic-analysis.md`` §6, шаг Ш0):
повторяется один и тот же цикл — LLM не вызвала тул, вместо диагностики
причины добавляется ещё один ``re.compile`` в один из музыкальных гуардов,
а через 1-3 дня новый паттерн срабатывает ложно на живом роботе (пары
#2897->#2971, #2966->#3004, Bug C->#2834/#2999/#3125, Bug F->#2565,
#1881->#1895). Пока не закрыты шаги Ш1/Ш2 того же плана, новые регексы в
этих трёх модулях допустимы только с явного разрешения товарища Шифу.

Этот тест — механическая часть моратория: он считает вызовы
``re.compile(`` в каждом из трёх файлов и сравнивает с зафиксированным
здесь числом (``_MAX_RE_COMPILE``).

* Число выросло -> FAIL. Кто-то добавил регекс без разрешения Шифу, либо
  разрешение получено и нужно поднять предел в этом файле вместе с PR
  (и сослаться на разрешение Шифу в PR/issue).
* Число не изменилось -> PASS, тихо.
* Число упало (регексы удалили, например в рамках Ш1/Ш2) -> PASS, но с
  предупреждением: зафиксированный предел стоит понизить в этом же PR,
  иначе мораторий станет менее строгим, чем реальное состояние кода.

CONTRIBUTING.md, секция «Мораторий на музыкальные регексы (issue #3132)»,
содержит текстовую версию этого правила.
"""

from __future__ import annotations

import re
import warnings
from pathlib import Path

import pytest

# core/ пакета rob_box_voice: .../src/rob_box_voice/test/unit/core/<this file>
# -> parents[3] == .../src/rob_box_voice, дальше rob_box_voice/core.
_CORE_DIR = Path(__file__).resolve().parents[3] / "rob_box_voice" / "core"

# Зафиксированные пределы на 28.09.2026 (issue #3132). Понижать можно
# в любом PR, убирающем регексы; повышать — только с явного разрешения
# товарища Шифу (см. docs/design/2026-09-28-music-dj-systemic-analysis.md §6).
#
# Issue #3134 (Ш2): грамматика «громче/тише/стоп/ты диджей X» перенесена в
# ``media_command_grammar.py`` (роутер медиакоманд до LLM): 3 регекса из
# удалённого ``music_volume_request.py``, 4 из удалённого ``dj_request.py``
# и ``MUSIC_STOP_COMMAND_RE`` из ``dialogue_guards.py`` (35 -> 34). Новых нет:
# ``_VOLUME_CORE_RE`` заменён одним ``_WORD_CLASS_RE`` (классы слов).
_MAX_RE_COMPILE: dict[str, int] = {
    "dialogue_guards.py": 34,
    "music_guard.py": 0,
    "media_command_grammar.py": 8,
}

_RE_COMPILE_CALL = re.compile(r"re\.compile\(")


def _count_re_compile(path: Path) -> int:
    text = path.read_text(encoding="utf-8")
    return len(_RE_COMPILE_CALL.findall(text))


@pytest.mark.parametrize("filename,limit", sorted(_MAX_RE_COMPILE.items()))
def test_re_compile_count_within_moratorium(filename: str, limit: int) -> None:
    path = _CORE_DIR / filename
    assert path.is_file(), (
        f"issue #3132 guard: ожидаемый модуль не найден: {path}. "
        "Если модуль переименован/перемещён — обнови _CORE_DIR/_MAX_RE_COMPILE "
        "в этом тесте."
    )

    actual = _count_re_compile(path)

    assert actual <= limit, (
        f"{filename}: re.compile(...) стало {actual}, было зафиксировано {limit}. "
        "Мораторий #3132 (см. docs/design/2026-09-28-music-dj-systemic-analysis.md "
        "§6 Ш0): новые регексы в музыкальных гуардах — только с явного "
        "разрешения товарища Шифу, пока не закрыты Ш1 и Ш2. Если разрешение "
        "получено — подними _MAX_RE_COMPILE в этом тесте вместе с PR и сошлись "
        "на разрешение в описании PR/issue."
    )

    if actual < limit:
        warnings.warn(
            f"{filename}: re.compile(...) стало {actual}, зафиксировано "
            f"{limit} в _MAX_RE_COMPILE (issue #3132). Понижи предел в "
            "этом тесте в том же PR, который убрал регексы — иначе "
            "мораторий станет слабее фактического состояния кода.",
            UserWarning,
            stacklevel=1,
        )
