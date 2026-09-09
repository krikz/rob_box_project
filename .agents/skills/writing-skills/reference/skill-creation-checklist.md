# Skill Creation Checklist

Load this file when you are finishing or reviewing a skill.

## RED phase
- Create realistic test scenarios.
- Run them without the skill.
- Record baseline failures and rationalizations.

## GREEN phase
- Validate `name` and `description`.
- Keep `description` focused on triggers, not workflow.
- Write only the content needed to fix observed failures.
- Prefer supporting files for heavy reference or templates.
- Re-run the scenarios with the skill loaded.

## REFACTOR phase
- Add counters for new loopholes.
- Tighten wording where agents still rationalize.
- Re-test until behavior is stable.

## Quality checks
- Main skill file stays concise.
- Examples are few and high quality.
- Cross-references are clear and on-demand.
- Anti-patterns and red flags are explicit when relevant.

## Release checks
- Validate all referenced files exist.
- Verify the skill is discoverable by its description.
- Commit only after testing is complete.
