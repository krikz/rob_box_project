# Graphviz Conventions

Load this file only when editing or validating graphviz diagrams inside a skill.

## When to use diagrams

Use a diagram only when a decision point or loop is genuinely easier to understand visually than as a list.

Good uses:
- non-obvious branching
- retry / review loops
- "when to use A vs B" decisions

Do not use diagrams for:
- reference data
- long linear instructions
- code examples
- labels with no semantic meaning

## Style rules

- Keep diagrams small enough to read without zooming.
- Use semantic labels, not placeholders like `step1` or `helper2`.
- Prefer `diamond` for decisions, `box` for actions, `doublecircle` or `ellipse` for terminal states.
- Keep one diagram focused on one decision space.
- Avoid embedding code in node labels.

## Validation

If Graphviz is available locally, validate with:

```bash
dot -Tsvg input.dot -o output.svg
```

If rendering fails, simplify labels first, then check quoting and bracket balance.
