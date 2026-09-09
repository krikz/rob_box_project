# rob_box_core

Cross-cutting abstractions shared by every rob_box harness. P0 foundation
per `docs/adr/0001-harness-architecture.md` and `docs/refactoring-plan.md`.

This package is deliberately ROS-free at its core — the same code is exercised
in unit tests without bringing up `rclpy`. ROS-specific helpers (logger adapter,
param guard) are intentionally deferred to P1.

## Modules

| Module             | What it gives you                                          |
|--------------------|------------------------------------------------------------|
| `clock.py`         | `Clock` ABC + `SystemClock` / `MockClock`. Time injection. |
| `memory.py`        | `MemoryStore` port + `InMemoryStore`. Persistence.         |
| `dialogue_state.py`| `DialogueState` + `DialogueStateMachine`. P0.3 wrapper.    |

## `dialogue_state` — important note

The `DialogueState` enum here is a *new* type (still string-valued), not a
re-export of `rob_box_voice.core.dialogue_manager.DialogueState`. Both enums
share the same string values (`"IDLE"`, `"LISTENING"`, `"DIALOGUE"`,
`"SILENCED"`) so they're interchangeable on the wire, but they're distinct
types in Python. This is by design: P0 is additive only, the existing
`DialogueManager` stays as-is until P1.

`DialogueStateMachine` is the formal port — explicit transition table, illegal
transitions raise, `on_enter` / `on_exit` hooks available. It is *not yet wired
into* the live `DialogueNode`; that migration happens in P1 per the plan.

## Testing

```bash
cd src/rob_box_core
PYTHONPATH=. python3 -m pytest test/ -v
PYTHONPATH=. python3 -m coverage run --source=rob_box_core -m pytest test/
python3 -m coverage report --include='rob_box_core/*'
```
