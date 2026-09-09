#!/usr/bin/env python3
"""e2e check for the hermes-agent skill-validation patch (fix t_1ab37fa8)
AND the architectural-scope hint (ADR-0036 §4.1, child t_37f4fecf).

Run AFTER a hermes-agent update + install.sh (which re-applies
scripts/agent_flow/vendor/hermes-agent-skill-validation.patch):

    bash scripts/agent_flow/install.sh
    python3 scripts/agent_flow/tests/e2e_skill_validation.py

Verifies on a REAL profile that:
  1. _profile_skill_names returns a non-empty frozenset of the skills the
     runtime actually loads (symlink-following, so category symlinks count);
  2. _validate_skills_for_assignee ACCEPTS a skill installed on the profile;
  3. _validate_skills_for_assignee REJECTS a skill not installed on the
     profile with the expected ValueError (t_6c6c98fb regression guard);
  4. _count_skills (profiles.py symlink-following fix) reports a sane count,
     not ~5x fewer than reality;
  5. _validate_scope_for_assignee HINTS when assignee ∈ NON_ARCHITECT_PROFILES
     AND body contains an architectural keyword AND a TDD-shaped skill is
     requested (ADR-0036 §4.1, regression for t_e2ae0c29 5h31m mis-scope loop);
  6. _validate_scope_for_assignee is SILENT when assignee ∉ NON_ARCHITECT
     (e.g. devops / architect — they ARE the owners);
  7. _validate_scope_for_assignee respects the ``force=True`` override
     (CLI flag ``--force-scope``);
  8. _validate_scope_for_assignee is SILENT when no architectural keyword
     is in the body (avoids noisy false positives on plain bug fixes).

Exit 0 = all checks passed. Non-zero = regression (patch lost or broken).
"""
import os
import sys
import tempfile
import warnings

# Default to the production install path so the canonical invocation
#   python3 scripts/agent_flow/tests/e2e_skill_validation.py
# keeps working after `bash scripts/agent_flow/install.sh`. Allow override
# via env so the test can be pointed at an alternate worktree (e.g. CI
# applies the patch to a fresh ``origin/main`` checkout and runs the test
# there with HERMES_AGENT_SRC=/tmp/ha-scope-final).
HERMES_AGENT_SRC = os.environ.get(
    "HERMES_AGENT_SRC", "/home/builder/.hermes/hermes-agent",
)
if HERMES_AGENT_SRC not in sys.path:
    sys.path.insert(0, HERMES_AGENT_SRC)

# Isolate the kanban DB to a tmp file WITHOUT overriding HERMES_HOME —
# overriding HERMES_HOME would hide real profiles from
# ``_profile_skill_names`` (they live under ``~/.hermes/profiles/``).
# ``HERMES_KANBAN_DB`` is the supported per-call DB override and is
# honoured by ``kanban_db_path`` (see hermes_cli/kanban_db.py::kanban_db_path
# step 1). Without isolation, every test run writes real ``case-*`` tasks
# to the active board (issue surfaced in development: case-* pollution).
_KANBAN_DB_DIR = tempfile.mkdtemp(prefix="kanban-scope-validator-test-")
os.environ["HERMES_KANBAN_DB"] = os.path.join(_KANBAN_DB_DIR, "kanban.db")

from hermes_cli import kanban_db as kb
from hermes_cli.kanban_db import (
    _profile_skill_names,
    _validate_scope_for_assignee,
    _validate_skills_for_assignee,
)
from hermes_cli.profiles import _count_skills, get_profile_dir

PROFILE = sys.argv[1] if len(sys.argv) > 1 else "devops"

installed = _profile_skill_names(PROFILE)
assert installed is not None, "profile skill names should not be None for a real profile"
assert isinstance(installed, frozenset) and len(installed) > 0, (
    f"expected non-empty frozenset, got {installed!r}"
)
print(f"OK _profile_skill_names({PROFILE!r}) -> {len(installed)} skills")

# Pick any installed skill for the accept-check
probe = next(iter(sorted(installed)))
_validate_skills_for_assignee(PROFILE, [probe])
print(f"OK _validate_skills_for_assignee accepted installed skill {probe!r}")

# Foreign skill must be rejected with the t_6c6c98fb guard message
try:
    _validate_skills_for_assignee(PROFILE, ["definitely-not-a-real-skill-xyz"])
except ValueError as e:
    msg = str(e)
    assert "not installed in profile" in msg and PROFILE in msg
    print(f"OK _validate_skills_for_assignee rejected foreign skill: {msg[:90]}...")
else:
    raise SystemExit("FAIL: expected ValueError for foreign skill, got none")

count = _count_skills(get_profile_dir(PROFILE))
print(f"OK _count_skills -> {count}")
assert count >= 5, f"expected a sane skill count >= 5, got {count} (symlink-following broken?)"


# ---------------------------------------------------------------------------
# Architectural-scope hint coverage (ADR-0036 §4.1, child t_37f4fecf).
# These checks exercise the heuristic directly. The end-to-end DB path
# (hint -> task_events row) is covered by tests/hermes_cli/test_kanban_db.py
# upstream; here we lock the keyword/skill/assignee matrix.
# ---------------------------------------------------------------------------

def _capture_hint(assignee, body, skills, *, force=False):
    """Call _validate_scope_for_assignee with warnings captured."""
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        msg = _validate_scope_for_assignee(assignee, body, skills, force=force)
    hint_warnings = [w for w in caught if "scope-hint" in str(w.message)]
    return msg, len(hint_warnings)


# Case A — assignee=backend + body has "ADR-" + skill=TDD → warning fires.
hint_msg, hint_warnings = _capture_hint(
    "backend", "design ADR-1234 for X", ["test-driven-development"],
)
assert hint_msg is not None, "FAIL: hint_msg should be set for backend+ADR+TDD"
assert hint_warnings == 1, f"FAIL: expected 1 warning, got {hint_warnings}"
assert "backend" in hint_msg and "ADR-" in hint_msg, (
    f"FAIL: hint_msg should name assignee+keyword, got: {hint_msg[:120]}"
)
print(f"OK test_scope_hint_backend_adr: warning emitted + hint returned ({hint_msg[:80]}...)")

# Case B — assignee=devops + body has "ADR-" + skill=agent-flow-ops → SILENT.
# devops IS an architect/devops-class owner (ADR-0036 §2f), so no hint.
hint_msg, hint_warnings = _capture_hint(
    "devops", "design ADR-1234 for X", ["agent-flow-ops"],
)
assert hint_msg is None and hint_warnings == 0, (
    f"FAIL: devops must not trigger; got msg={hint_msg!r}, warnings={hint_warnings}"
)
print("OK test_scope_hint_devops_adr: silent (devops is owner-class)")

# Case C — assignee=backend + body has "ADR-" + skill=TDD + force=True → SILENT.
hint_msg, hint_warnings = _capture_hint(
    "backend", "design ADR-1234 for X", ["test-driven-development"], force=True,
)
assert hint_msg is None and hint_warnings == 0, (
    f"FAIL: force=True must suppress; got msg={hint_msg!r}, warnings={hint_warnings}"
)
print("OK test_scope_force_override: --force-scope suppresses warning")

# Case D — assignee=backend + body without architectural keywords → SILENT.
hint_msg, hint_warnings = _capture_hint(
    "backend", "plain bug fix in parser", ["test-driven-development"],
)
assert hint_msg is None and hint_warnings == 0, (
    f"FAIL: body without keyword must not trigger; got msg={hint_msg!r}, warnings={hint_warnings}"
)
print("OK test_scope_no_keyword: silent (no architectural keyword in body)")

print("\nALL E2E SKILL-VALIDATION CHECKS PASSED")
