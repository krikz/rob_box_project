#!/usr/bin/env python3
"""Регресс-тест для сопоставления ``expected_keywords`` (issue #2753).

Ключ вида ``"Борис|Спартак|пицц"`` сценарии пишут как АЛЬТЕРНАЦИЮ — так же,
как соседнее поле ``patterns`` (оно идёт через ``grep -E``). До фикса
``check_acceptance`` сравнивал ключ целой подстрокой и искал в логах строку
вместе с палками — не находил никогда:

    >>> STEP n209_recall_boris: ✅ ПОЛНЫЙ ЦИКЛ (акцепт + LLM + TTS)
    >>> ACCEPTANCE[n209_recall_boris]: ❌ expected keywords missing in logs:
        ['Борис|Спартак|пицц']

при том что робот ответил ровно по делу (run 35699257202, акт 2). Вечно
красными были три шага: n209 в акте 2, n1002 и n1009 в финальном акте 10.

Тест гоняет настоящий вердикт шага ``e2e_tool_match.step_verdict`` (с #3906 heredoc ``check_acceptance`` только
зовёт её, своих копий правил в харнессе нет; сухой стенд ``scripts/dialog/dry_marathon.py`` зовёт ту же функцию) и
проверяет, что ``expected_keywords`` с альтернацией проходят через неё. Если кто-то вернёт сравнение целой строкой —
тест покраснеет здесь, а не через сутки в ночном марафоне.

Запуск: python3 scripts/testing/test_e2e_expected_keywords.py
Exit:   0 = PASS, 1 = есть провалы.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
HARNESS = REPO_ROOT / ".github" / "workflows" / "scripts" / "e2e_voice_test.sh"

# Фрагмент лога робота из run 35699257202, шаг n209_recall_boris.
LOGS = (
    "[dialogue_node] 🔍 [handle_result] spoken='Тебя зовут Борис, ты друг Саши "
    "и приходишь примерно раз в неделю с пиццей. Болеешь за Спартак, Саша над "
    "этим посмеивается.' (len=122) tools=[]"
).lower()


def load_keyword_hit(logs_low: str):
    """``expected_keywords`` шага через ``step_verdict``: ключ найден, если он не в ``missing_keywords``."""
    source = HARNESS.read_text(encoding="utf-8")
    if "from e2e_tool_match import step_verdict" not in source:
        raise SystemExit(
            f"FATAL: {HARNESS} не зовёт e2e_tool_match.step_verdict — "
            "проверку expected_keywords вернули в харнесс, тест надо обновить"
        )
    sys.path.insert(0, str(HARNESS.parent))
    from e2e_tool_match import step_verdict

    def keyword_hit(kw: str) -> bool:
        verdict = step_verdict({"expected_keywords": [kw]}, logs_low)
        return kw in verdict["found_keywords"] and kw not in verdict["missing_keywords"]

    return keyword_hit


CASES = [
    # (ключ, ожидание, зачем)
    ("Борис|Спартак|пицц", True, "issue #2753: альтернация, совпал первый вариант"),
    ("Спартак|пицц", True, "альтернация без первого варианта"),
    ("пицц", True, "одиночный ключ (обычный случай) не сломан"),
    ("Саш", True, "подстрока внутри слова — как и было"),
    ("БОРИС", True, "регистр не важен"),
    ("зелён|чай", False, "ни одного варианта нет в логе — честный miss"),
    ("Гриша", False, "одиночный ключ, которого нет"),
    ("Борис|", True, "пустой хвост после палки не ломает разбор"),
    ("|", False, "ключ из одних палок ничему не соответствует"),
    ("", False, "пустой ключ не совпадает (иначе любой шаг зелёный)"),
    ("  Спартак  ", True, "пробелы по краям варианта обрезаются"),
]


def main() -> int:
    keyword_hit = load_keyword_hit(LOGS)
    failures = 0
    for key, expected, why in CASES:
        actual = bool(keyword_hit(key))
        status = "OK  " if actual == expected else "FAIL"
        if actual != expected:
            failures += 1
        print(f"{status} {key!r:38} -> {actual!s:5} (ожидалось {expected}) — {why}")

    print()
    print(f"PASS={len(CASES) - failures} FAIL={failures}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
