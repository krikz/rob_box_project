# Testing Skills With Subagents

Load this file when designing or running evaluation scenarios for a skill.

## Goal

Prove that the skill changes agent behavior in realistic situations.

## RED → GREEN → REFACTOR

### RED
- Run a scenario without the skill.
- Capture what the agent does wrong.
- Record exact rationalizations, shortcuts, and missing context.

### GREEN
- Add the minimal skill content needed to address those failures.
- Run the same scenario again with the skill loaded.
- Confirm the agent now behaves correctly.

### REFACTOR
- Add counters for new loopholes discovered in testing.
- Re-run until the behavior is stable.

## Pressure types

Use one or more of these pressures depending on the skill:
- time pressure
- sunk-cost pressure
- authority pressure
- ambiguity
- fatigue / impatience
- tempting shortcut that violates the workflow

## By skill type

### Discipline-enforcing skills
Test whether the agent still follows the rule under pressure.

### Technique skills
Test whether the agent can correctly apply the method to a new case.

### Pattern skills
Test whether the agent can recognize when the pattern applies and when it does not.

### Reference skills
Test whether the agent can retrieve the right part and use it correctly.

## Scenario design

A good scenario is:
- narrow enough to judge clearly
- realistic enough to trigger shortcuts
- specific about success/failure
- repeatable after edits

## Evidence to capture

- what the agent did
- what it ignored
- which words signaled confusion or rationalization
- what change in the skill caused improved behavior
