# Anthropic Skill Authoring Notes

Load this file only when you need extra guidance beyond the main writing-skills workflow.

This is a local summary of recurring Anthropic-style skill authoring guidance, not a verbatim upstream document.

## Core ideas

- Keep `SKILL.md` short, action-oriented, and easy to scan.
- Put trigger conditions in `description`; keep workflow detail in the body.
- Prefer supporting files for heavy reference, templates, or phase-specific material.
- Avoid duplicating content that already lives in another skill or reference file.
- Optimize for discovery first, then correctness, then completeness.
- Use examples sparingly; one strong example beats many weak ones.
- Test the skill against realistic scenarios before treating it as complete.

## Practical checklist

- Is the `description` only about when to use the skill?
- Does `SKILL.md` contain only the decisions needed in most invocations?
- Did you move bulky reference content out of the main skill file?
- Are supporting files named clearly and loaded only on demand?
- Can another agent find and apply this skill without repo-specific folklore?
