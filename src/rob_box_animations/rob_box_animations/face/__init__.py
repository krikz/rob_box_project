"""Procedural LED face (ADR-0150 §5.5): scenes rendered in real time.

``states.SCENES`` — every scene; ``scene.Scene(spec).frame(t, ctx)`` — one frame;
``face`` — models, palette (levels 0..25), panel layout and current budget.
The lab ``tools/led_face_lab`` previews these scenes, it does not own them.
"""
