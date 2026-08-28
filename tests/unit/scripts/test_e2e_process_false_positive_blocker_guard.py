"""Regression test for retro t_bb56f2a1 (28.08.2026).

`scripts/agent_flow/agent-flow-e2e-process.sh::detect_known_blocker()` искал
OPEN issue с сигнатурой (например "no_wake_word") в title/body. После того как
основной блокер #1668 был CLOSED, фильтр продолжал ловить эпик-фичи с
упоминанием сигнатуры в title (например #1684 Captain Bridge "wake-word"):
они имели метки `feature, epic:*, meta-quest, source:gsd` — никаких bug/process/voice/e2e.

Без фильтра ротация вставала вечно после закрытия основного блокера.
Fail-streak рос бесконечно (17+).

Этот тест пинит contract false-positive guard:

1. `detect_known_blocker()` должен иметь ВТОРОЙ guard после `blocker_issue_for_sig`:
   если hit-issue не имеет bug/process/voice/e2e/regression меток → это false-positive.

2. `blocker_issue_for_sig()` должен расширить exclude-фильтр меток, чтобы
   feature/epic/xr/source:gsd issues сразу отфильтровывались на первом этапе
   (без необходимости делать второй `gh issue view` запрос).

3. `collect_issues_json` фильтр (`e2e-done`/`e2e:rejected` strip) и PR-side
   merge должны иметь try/except защиту от JSON parse errors (defense-in-depth).
   Падение JSON парсинга НЕ должно валить весь тик — fallback на оригинал.

4. Скрипт должен `bash -n` парситься чисто (защита от синтаксических опечаток).
"""
from __future__ import annotations

import pathlib
import re
import subprocess
import unittest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO_ROOT / "scripts" / "agent_flow" / "agent-flow-e2e-process.sh"


def _script_text() -> str:
    return SCRIPT_PATH.read_text()


class FalsePositiveBlockerGuardTest(unittest.TestCase):
    """Тест защиты от false-positive в detect_known_blocker."""

    def test_blocker_issue_for_sig_excludes_feature_epic_xr_labels(self):
        """blocker_issue_for_sig должен exclude'ить feature/epic:* / xr / source:gsd
        чтобы feature-эпики с упоминанием сигнатуры в title не считались блокерами."""
        text = _script_text()
        m = re.search(
            r"^blocker_issue_for_sig\(\)\s*\{(.*?)^}",
            text,
            flags=re.MULTILINE | re.DOTALL,
        )
        self.assertIsNotNone(m, "blocker_issue_for_sig() function not found")
        body = m.group(1)
        # Required exclude labels (regression false-positive vector for #1684 etc.)
        for label in ('"feature"', '"epic:avatar"', '"epic:quest"',
                      '"meta-quest"', '"webxr"', '"source:gsd"'):
            self.assertIn(
                label, body,
                f"blocker_issue_for_sig missing exclude label {label} "
                f"(false-positive guard retro t_bb56f2a1)"
            )

    def test_detect_known_blocker_has_false_positive_guard(self):
        """detect_known_blocker должен проверять метки hit-issue через --jq
        и пропускать feature-эпики (без bug/process/voice/e2e меток)."""
        text = _script_text()
        m = re.search(
            r"^detect_known_blocker\(\)\s*\{(.*?)^detect_fail_signature",
            text,
            flags=re.MULTILINE | re.DOTALL,
        )
        self.assertIsNotNone(m, "detect_known_blocker() function not found")
        body = m.group(1)
        # Required: gh issue view + any() проверка на bug/process/voice/e2e
        self.assertIn("gh issue view", body,
                      "detect_known_blocker missing hit-issue label check")
        self.assertIn(".labels[].name", body,
                      "detect_known_blocker missing labels parsing via --jq")
        # Strict matching: 'any(. == "bug" or . == "process" ...)'
        # (NOT bash-glob `*bug*` which would match 'release-buggy')
        self.assertIn('"bug"', body)
        self.assertIn('"process"', body)
        self.assertIn('"voice"', body)
        self.assertIn('"e2e"', body)
        # Anti-pattern: bash case `*bug*|*voice*` (fragile substring match)
        self.assertNotRegex(
            body, r"\*bug\*\|\*voice\*",
            "detect_known_blocker uses fragile bash-glob match instead of jq "
            "any() — would match 'release-buggy' or 'voice-over' as bug/voice"
        )

    def test_false_positive_guard_logs_skipped_hit(self):
        """False-positive guard должен явно логировать пропуск hit-issue,
        чтобы Шифу видел в cron output, КАКИЕ именно feature-эпики НЕ блокируют."""
        text = _script_text()
        self.assertRegex(
            text,
            r"false-positive blocker guard.*hit.*НЕ блокер.*Продолжаем ротацию",
            "detect_known_blocker missing user-visible log on false-positive skip",
        )


class CollectIssuesJsonHardeningTest(unittest.TestCase):
    """Тест защиты collect_issues_json от JSON parse errors."""

    def test_e2e_done_filter_has_try_except(self):
        """Inline python для фильтра e2e-done/e2e:rejected должен иметь
        try/except защиту (defense-in-depth против rate-limit warnings и т.п.)."""
        text = _script_text()
        # Найти блок filter (после issues_json="$(gh_list_issues_by_label...)")
        m = re.search(
            r"issues_json=\"\$\(gh_list_issues_by_label.*?\n(.*?)\n\s+_filtered=\"\$\(printf '%s'",
            text,
            flags=re.DOTALL,
        )
        # Альтернативный поиск: от "if [ -n \"$issues_json\" ]" до "_filtered="
        if not m:
            m = re.search(
                r'if \[ -n "\$issues_json" \].*?_filtered="\$\(printf',
                text,
                flags=re.DOTALL,
            )
        self.assertIsNotNone(
            m,
            "could not locate filter block in collect_issues_json",
        )
        # Ищем try/except вокруг json.load
        region = text[m.start():m.start() + 500]
        self.assertIn(
            "try:", region,
            "collect_issues_json e2e-done filter missing try/except around json.load"
        )

    def test_pr_side_merge_has_try_except(self):
        """Inline python для PR-side merge (raw_issues.append) должен иметь
        try/except защиту. Также _prs_count / _merged_count должны быть
        defensively parseable."""
        text = _script_text()
        # Найти _prs_count / _merged_count
        self.assertRegex(
            text,
            r"_prs_count=\"\$\(printf.*?2>/dev/null \|\| echo \"-1\"",
            "collect_issues_json _prs_count missing 2>/dev/null + echo -1 fallback",
        )
        self.assertRegex(
            text,
            r"_merged_count=\"\$\(printf.*?2>/dev/null \|\| echo \"-1\"",
            "collect_issues_json _merged_count missing 2>/dev/null + echo -1 fallback",
        )


class ScriptParseTest(unittest.TestCase):
    """Smoke test: bash -n."""

    def test_bash_n_parses_clean(self):
        """Скрипт должен проходить bash -n (защита от синтаксических опечаток)."""
        result = subprocess.run(
            ["bash", "-n", str(SCRIPT_PATH)],
            capture_output=True,
            text=True,
        )
        self.assertEqual(
            result.returncode, 0,
            f"bash -n failed:\nstdout: {result.stdout}\nstderr: {result.stderr}",
        )


if __name__ == "__main__":
    unittest.main()