"""GenHealth: здоровье генерации музыки для /voice/providers/state (PR-4 ч.2, #3693)."""

from __future__ import annotations


def test_gen_health_maps_reasons_and_expires():
    from rob_box_mcp_tools.engine.gen.provider import GenHealth, gen_provider_states

    now = [100.0]
    h = GenHealth(clock=lambda: now[0])
    assert gen_provider_states(h) == []
    h.note_ok()
    h.note_failure("rejected")  # про заказ, не про провайдера
    assert h.snapshot()[0] == "ok"
    h.note_failure("no_credits")
    st = gen_provider_states(h)[0]
    assert (st.kind, st.name, st.health) == ("music_gen", "kie", "quota")
    h.note_failure("no_key")
    assert h.snapshot()[0] == "auth"
    now[0] += 3600.0
    assert h.snapshot() is None
