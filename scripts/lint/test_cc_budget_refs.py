#!/usr/bin/env python3
"""Self-tests for scripts/lint/cc_budget_refs.py (issue #2186, #2626).

The cc_budget_refs guard exists to plug the "silent baseline growth" hole
that allowed ``WSSServer._on_json_cmd`` to balloon to CC=107 undetected
(issue #2186). After #2626, the guard also enforces ADR-0021-r1:

* R-1a — legacy-cc is immutable: bump or shrink above legacy.cc → FAIL.
* R-1d — ``--verify-remote`` requires issue to be **state=open**, not
  merely labeled ``type:tech-debt``.
* R-1f — final message is honest: «долг под контролем, но НЕ покрыт»
  beats the lie «OK — все имеют ref-cards» when ref-cards are empty.

The guard itself is non-trivial. Per the convention set by
``test_seam_without_consumer.py`` (issue #2118): the same "no silent
regression hiding behind coverage elsewhere" rule that produced this
guard must apply to the guard's own code. If a future refactor breaks
detection of one of the historical patterns below, the test should
fail loudly here, not three days after the next real regression.

Coverage matrix:

| Дефект | Тест | ADR-0021-r1 |
|---|---|---|
| #2186 baseline-bump без ref-card | test_fails_when_exempt_missing_ref | — |
| #2186 dangling ref-card | test_fails_on_dangling_ref_card | — |
| #2186 bad ref format | test_fails_on_bad_ref_format | — |
| #2626 #1 — bump legacy-cc | test_fails_on_legacy_cc_growth | R-1a |
| #2626 #1 — снижение legacy-cc | test_fails_on_legacy_cc_shrink | R-1a |
| #2626 #2 — verify-remote не видит closed issue | (network-only, exercise_remote) | R-1d |
| #2626 #3 — phantom baseline entry | test_fails_on_dangling_ref_card (legacy-paths) | R-1c |
| #2626 #4 — recovered без обновления baseline | (covered by ``cc_budget.py`` test, not here) | R-1b |
| #2626 #5 — честное сообщение | test_main_prints_breakdown | R-1f |

Run:
    python -m unittest scripts.lint.test_cc_budget_refs -v
    python scripts/lint/test_cc_budget_refs.py
"""

from __future__ import annotations

import io
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cc_budget_refs as refs  # noqa: E402


def _baseline(
    *,
    exemptions: dict[str, dict[str, int]] | None = None,
    refactor_cards: dict[str, str] | None = None,
    legacy: list[dict] | None = None,
) -> dict:
    """Tiny helper to make test fixtures readable."""
    return {
        "version": 1,
        "exemptions": exemptions or {},
        "_refactor_cards": refactor_cards or {},
        "_legacy_acknowledged": legacy or [],
    }


class ExemptKeysTests(unittest.TestCase):
    def test_exempt_keys_flattens_path_method(self) -> None:
        baseline = _baseline(
            exemptions={
                "src/a/a.py": {"Foo.bar": 17},
                "src/b/b.py": {"Baz.__init__": 22, "Baz.qux": 16},
            }
        )
        self.assertEqual(
            refs._exempt_keys(baseline),
            {
                "src/a/a.py:Foo.bar",
                "src/b/b.py:Baz.__init__",
                "src/b/b.py:Baz.qux",
            },
        )

    def test_exempt_keys_skips_container_entries(self) -> None:
        # Garbage in JSON (e.g. someone hand-wrote a dict); guard must not
        # blow up nor leak the dict into the key set. Real int cc values
        # (even plain ``99``) are legitimate and must be kept.
        baseline = _baseline(
            exemptions={"src/a/a.py": {"Foo.bar": 17, "Foo.baz": {"cc": 99}}},  # type: ignore[dict-item]
        )
        # The dict ``{"cc": 99}`` is skipped by isinstance(int) check.
        # Upstream ``cc_budget.py`` will choke on non-int when comparing,
        # so the guard pre-filters container garbage.
        self.assertEqual(refs._exempt_keys(baseline), {"src/a/a.py:Foo.bar"})


class LegacyKeysTests(unittest.TestCase):
    def test_legacy_keys_skips_empty_paths_or_methods(self) -> None:
        baseline = _baseline(
            legacy=[
                {"path": "src/a/a.py", "method": "X.y", "cc": 18},
                {"path": "", "method": "X.z", "cc": 18},
                {"path": "src/a/a.py", "method": "", "cc": 18},
                {"cc": 99},  # no path/method at all
            ]
        )
        self.assertEqual(refs._legacy_keys(baseline), {"src/a/a.py:X.y"})

    def test_legacy_index_first_wins_on_duplicates(self) -> None:
        # Если кто-то задвоил entries (бывает при ручной правке),
        # ``_legacy_index`` берёт первый — это важно, потому что
        # ``check_local`` использует его для cc-сравнения.
        baseline = _baseline(
            legacy=[
                {"path": "src/a/a.py", "method": "X.y", "cc": 18, "since": "first"},
                {"path": "src/a/a.py", "method": "X.y", "cc": 19, "since": "second"},
            ]
        )
        self.assertEqual(
            refs._legacy_index(baseline),
            {"src/a/a.py:X.y": {"path": "src/a/a.py", "method": "X.y", "cc": 18, "since": "first"}},
        )


class LegacySinceExtractionTests(unittest.TestCase):
    """R-1d: парсинг ``#NNNN`` из поля ``since:`` legacy-entry."""

    def test_extracts_single_issue_ref(self) -> None:
        self.assertEqual(
            refs._extract_legacy_refs(
                {
                    "since": "cc_budget_baseline.json created 2026-09-08; "
                    "legacy grandfather — parent gate #1984 / #2077 covers decomp backlog"
                }
            ),
            {1984, 2077},
        )

    def test_extracts_multiple_issue_refs_from_long_since(self) -> None:
        # Ровно тот текст из baseline.json: «bumped 65 → 68 … 82 → 85»
        # содержит семь ссылок на issues.
        entry = {
            "since": (
                "cc_budget_baseline.json created 2026-09-08; legacy grandfather — "
                "parent gate #1984 / #2077 covers decomp backlog; "
                "bumped 65 -> 68 for issue #2548 prose-action-claim fallback (CC-budget guard); "
                "bumped 68 -> 72 for issue #2547 strip_meta_markers pipeline; "
                "bumped 79 -> 82 for issue #2557 DJ-music-tools fallback"
            )
        }
        self.assertEqual(refs._extract_legacy_refs(entry), {1984, 2077, 2548, 2547, 2557})

    def test_returns_empty_when_no_issue_refs(self) -> None:
        self.assertEqual(refs._extract_legacy_refs({"since": "no refs here"}), set())

    def test_returns_empty_when_since_is_empty(self) -> None:
        self.assertEqual(refs._extract_legacy_refs({}), set())


class RefFormatTests(unittest.TestCase):
    def test_accepts_plain_issue_ref(self) -> None:
        self.assertTrue(refs._ISSUE_REF_RE.match("#1234"))

    def test_accepts_issue_ref_with_alias(self) -> None:
        self.assertEqual(
            refs._ISSUE_REF_RE.match("#2195 (voice-vr 10)").groups(),  # type: ignore[union-attr]
            ("2195", "voice-vr 10"),
        )

    def test_accepts_issue_ref_with_whitespace_around_alias(self) -> None:
        # The regex anchors are strict (\s* around alias contents only).
        # Leading/trailing whitespace on the whole string is rejected —
        # guards should call .strip() before matching (see check_local).
        match = refs._ISSUE_REF_RE.match("#1984  (parent gate)")
        assert match is not None
        self.assertEqual(match.groups(), ("1984", "parent gate"))

    def test_rejects_garbage(self) -> None:
        for bad in (
            "https://github.com/foo/bar/issues/1",
            "issue #1",
            "#abc",
            "#1 #2",
            "",
            "  #1984  ",
        ):
            self.assertIsNone(
                refs._ISSUE_REF_RE.match(bad), msg=f"should reject: {bad!r}"
            )


class LocalCheckTests(unittest.TestCase):
    def test_ok_when_every_exempt_has_ref(self) -> None:
        baseline = _baseline(
            exemptions={"src/a/a.py": {"X.y": 17}},
            refactor_cards={"src/a/a.py:X.y": "#1234 (test)"},
        )
        self.assertEqual(refs.check_local(baseline), [])

    def test_fails_when_exempt_missing_ref(self) -> None:
        # The headline scenario from issue #2186: PR adds a new exempt
        # with no _refactor_cards entry. Must FAIL — this is exactly
        # the silent-baseline-growth hole the guard exists to plug.
        baseline = _baseline(
            exemptions={"src/a/a.py": {"X.y": 17}},
            refactor_cards={},
        )
        violations = refs.check_local(baseline)
        self.assertEqual(len(violations), 1)
        self.assertIn("src/a/a.py:X.y", violations[0])
        self.assertIn("нет записи в _refactor_cards", violations[0])

    def test_legacy_photo_hides_missing_ref(self) -> None:
        # Backward-compat: methods grandfathered before the guard was
        # added must not require a ref-card or the guard would block
        # every existing PR with a baseline change. Photo must mirror
        # the actual exempt (same path, same method, same cc).
        baseline = _baseline(
            exemptions={"src/a/a.py": {"X.y": 17}},
            legacy=[{"path": "src/a/a.py", "method": "X.y", "cc": 17}],
        )
        self.assertEqual(refs.check_local(baseline), [])

    def test_legacy_with_ref_emits_cleanup_info_not_failure(self) -> None:
        # If a legacy-photo'd method also has a ref-card, the guard
        # prints a soft hint to drop it from _legacy_acknowledged —
        # but does NOT fail the build (cleanup is a separate task).
        baseline = _baseline(
            exemptions={"src/a/a.py": {"X.y": 17}},
            refactor_cards={"src/a/a.py:X.y": "#1234"},
            legacy=[{"path": "src/a/a.py", "method": "X.y", "cc": 17}],
        )
        violations = refs.check_local(baseline)
        self.assertEqual(violations, [])

    def test_fails_on_dangling_ref_card(self) -> None:
        # Ref-card points at a method that isn't in exemptions: either
        # the method got fixed and the ref-card needs cleanup, or the
        # ref-card is a typo. Either way: FAIL.
        baseline = _baseline(
            exemptions={"src/a/a.py": {"X.y": 17}},
            refactor_cards={"src/a/a.py:X.y": "#1234", "src/a/a.py:Z.w": "#1235"},
        )
        violations = refs.check_local(baseline)
        self.assertTrue(any("Z.w" in v and "висячая" in v for v in violations))

    def test_fails_on_bad_ref_format(self) -> None:
        baseline = _baseline(
            exemptions={"src/a/a.py": {"X.y": 17}},
            refactor_cards={"src/a/a.py:X.y": "not-an-issue"},
        )
        violations = refs.check_local(baseline)
        self.assertTrue(any("не похоже" in v for v in violations))

    def test_fails_on_legacy_cc_growth(self) -> None:
        # Issue #2626, defect #1 (R-1a, ADR-0021-r1): bump ``cc`` сверх
        # legacy.cc без переноса в _refactor_cards — то, чем #2186 не
        # ловил. Восемь сентябрьских bump'ов именно так и выглядели.
        baseline = _baseline(
            exemptions={"src/a/a.py": {"X.y": 22}},  # было 17, bump'нули до 22
            legacy=[{"path": "src/a/a.py", "method": "X.y", "cc": 17}],
        )
        violations = refs.check_local(baseline)
        self.assertTrue(
            any("рост legacy-записи" in v and "R-1a" in v for v in violations),
            msg=f"expected R-1a growth-FAIL, got: {violations}",
        )

    def test_fails_on_legacy_cc_shrink(self) -> None:
        # Симметрично: снижение cc (например, после частичного refactor)
        # не должно «освобождать» от обязательства либо убрать из
        # legacy, либо завести _refactor_cards. Иначе legacy.cc —
        # ложь, и мы не узнаем, какой реально cc был унаследован.
        baseline = _baseline(
            exemptions={"src/a/a.py": {"X.y": 14}},  # было 17, снизили до 14
            legacy=[{"path": "src/a/a.py", "method": "X.y", "cc": 17}],
        )
        violations = refs.check_local(baseline)
        self.assertTrue(
            any("снижение" in v for v in violations),
            msg=f"expected R-1a shrink-FAIL, got: {violations}",
        )
        self.assertTrue(
            any("R-1a" in v for v in violations),
            msg=f"expected R-1a reference in any violation, got: {violations}",
        )

    def test_fails_on_legacy_photo_pointing_to_nothing(self) -> None:
        # Legacy entry references a method that has been removed from
        # exemptions entirely (refactor done, cleanup pending). The
        # photo is now dangling — block the merge so it doesn't rot.
        baseline = _baseline(
            exemptions={},
            legacy=[{"path": "src/a/a.py", "method": "X.y", "cc": 17}],
        )
        violations = refs.check_local(baseline)
        self.assertTrue(any("висячая запись" in v for v in violations))

    def test_real_baseline_in_repo_passes_local(self) -> None:
        # End-to-end smoke test against the actual on-disk baseline.
        # If this breaks after a real baseline edit, the guard has a
        # bug — not the baseline (the baseline was the prior commit
        # snapshot, validated by human review).
        real = Path(__file__).resolve().parent / "cc_budget_baseline.json"
        if not real.exists():
            self.skipTest(f"baseline not found: {real}")
        baseline = refs._load_baseline(real)
        violations = refs.check_local(baseline)
        self.assertEqual(
            violations,
            [],
            msg=f"real baseline should pass check_local, got: {violations}",
        )


class MainOutputTests(unittest.TestCase):
    """R-1f: финальное сообщение main() — честная разбивка, не «OK»."""

    def _run_main_with(self, baseline: dict, argv: list[str] | None = None) -> tuple[int, str]:
        import tempfile

        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as fh:
            import json as _json

            fh.write(_json.dumps(baseline))
            tmp = Path(fh.name)
        old_argv = sys.argv
        try:
            sys.argv = ["cc_budget_refs.py", "--baseline", str(tmp)] + (argv or [])
            buf = io.StringIO()
            old_stdout = sys.stdout
            sys.stdout = buf
            try:
                rc = refs.main()
            finally:
                sys.stdout = old_stdout
            return rc, buf.getvalue()
        finally:
            sys.argv = old_argv
            tmp.unlink(missing_ok=True)

    def test_breakdown_printed_in_header(self) -> None:
        # При пустых карточках печатаем «0 с ref-card, N legacy» —
        # это видно по grep'у в ночном мониторинге.
        baseline = _baseline(
            exemptions={"src/a/a.py": {"X.y": 17}},
            legacy=[{"path": "src/a/a.py", "method": "X.y", "cc": 17}],
        )
        rc, out = self._run_main_with(baseline)
        self.assertEqual(rc, 0)
        self.assertIn("0 с ref-card", out)
        self.assertIn("1 legacy", out)
        # Честный текст: legacy — это долг, не «ок».
        self.assertIn("долг под контролем", out)
        self.assertNotIn("OK — все exemptions имеют ref-cards", out)

    def test_breakdown_with_zero_legacy_says_no_debt(self) -> None:
        baseline = _baseline(
            exemptions={"src/a/a.py": {"X.y": 17}},
            refactor_cards={"src/a/a.py:X.y": "#1234"},
        )
        rc, out = self._run_main_with(baseline)
        self.assertEqual(rc, 0)
        self.assertIn("OK — все exemptions имеют ref-cards; долга нет", out)

    def test_breakdown_counts_phantoms(self) -> None:
        # Phantom-ref (ref-card на ключ, которого нет в exemptions)
        # должен быть посчитан и помечен в разбивке как «N фантомных».
        baseline = _baseline(
            exemptions={"src/a/a.py": {"X.y": 17}},
            refactor_cards={
                "src/a/a.py:X.y": "#1234",
                "src/a/a.py:Z.w": "#1235",  # phantom
            },
        )
        rc, out = self._run_main_with(baseline)
        self.assertNotEqual(rc, 0)  # phantom → FAIL
        self.assertIn("1 фантомных", out)


if __name__ == "__main__":
    unittest.main()
