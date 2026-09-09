#!/usr/bin/env python3
"""Self-tests for scripts/lint/cc_budget_refs.py (issue #2186, ADR-0021 R1).

The cc_budget_refs guard exists to plug the "silent baseline growth" hole
that allowed ``WSSServer._on_json_cmd`` to balloon to CC=107 undetected
(issue #2186). The guard itself is non-trivial (legacy-photo, ref-card
syntax, dangling-link detection, remote verify). Per the convention set
by ``test_seam_without_consumer.py`` (issue #2118): the same "no silent
regression hiding behind coverage elsewhere" rule that produced this
guard must apply to the guard's own code. If a future refactor breaks
detection of one of the historical patterns below, the test should
fail loudly here, not three days after the next real regression.

Two kinds of coverage:

* Unit tests for the structural checks (``check_local``) against
  synthetic baseline fixtures: missing ref-card, dangling ref-card,
  bad ref format, legacy-with-ref cleanup hint, legacy-photo drift
  (cc mismatch, missing-from-exemptions).
* Smoke test for ``_legacy_keys`` / ``_exempt_keys`` helpers.

Run:
    python -m unittest scripts.lint.test_cc_budget_refs -v
    python scripts/lint/test_cc_budget_refs.py
"""

from __future__ import annotations

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

    def test_fails_on_legacy_photo_drift_cc_mismatch(self) -> None:
        # The photo's cc no longer matches the actual exempt cc. Either
        # somebody hand-edited one side, or the guard's been bypassed
        # with --update-baseline. Either way: FAIL loudly so the next
        # reader doesn't trust a stale photo.
        baseline = _baseline(
            exemptions={"src/a/a.py": {"X.y": 20}},
            legacy=[{"path": "src/a/a.py", "method": "X.y", "cc": 17}],
        )
        violations = refs.check_local(baseline)
        self.assertTrue(any("не совпадает с exemptions cc" in v for v in violations))

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


if __name__ == "__main__":
    unittest.main()
