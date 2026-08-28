#!/usr/bin/env python3
"""e2e check for the hermes-agent skill-validation patch (fix t_1ab37fa8).

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
     not ~5x fewer than reality.

Exit 0 = all checks passed. Non-zero = regression (patch lost or broken).
"""
import sys

sys.path.insert(0, "/home/builder/.hermes/hermes-agent")

from hermes_cli.kanban_db import _profile_skill_names, _validate_skills_for_assignee
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

print("\nALL E2E SKILL-VALIDATION CHECKS PASSED")
