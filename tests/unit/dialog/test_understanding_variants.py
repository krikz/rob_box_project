"""Варианты понимания фразы на сухом стенде (карточка #4163): A как на роботе, B понимает модель, C понимает Jev.

Что доказывают тесты: на B срез тулов не зависит от слов фразы (контроль: на A зависит, тест это видит), Tier-1 в B и C
отдаёт только безопасность, подмена живёт только на время реплики и не течёт между стендами, боевой код не тронут.
LLM - фейковый провайдер за НАСТОЯЩИМ мостом команд (``understanding_measure.fake_stand``).
"""

from __future__ import annotations

import asyncio
import sys
from pathlib import Path

import pytest

# Мост команд и порты хода тянут клиент LLM-провайдеров (`openai`): в лёгком окружении «Python Code Quality» его нет.
pytest.importorskip("openai", reason="сухому стенду нужен настоящий мост команд, а ему - клиент openai")

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts" / "dialog"))

import dry_stand  # noqa: E402
import understanding_measure as umeasure  # noqa: E402
import understanding_variants as uv  # noqa: E402
from jev_router import Route  # noqa: E402
from rob_box_dialog import knowledge as kn  # noqa: E402
from rob_box_dialog import tools_view  # noqa: E402
from rob_box_dialog import turn as turn_mod  # noqa: E402
from rob_box_dialog.grammar import match as real_match  # noqa: E402
from rob_box_dialog.session import MusicStateRef, Session  # noqa: E402

#: Фразы, каждая из которых открывает свою группу тулов в срезе A (по словам), и фраза без групп.
PHRASES = ["Робот, спой песню", "Робот, смени голос", "Робот, сколько заряда", "Робот, включи сирену",
           "Робот, как дела", ""]
#: Фраза → вид Tier-1, который на ней срабатывает в A (``grammar.match``).
TIER1_PHRASES = {"стоп": "stop_all", "замолчи": "silence", "хватит читать": "stop_speech",
                 "новая сессия": "new_session", "погромче": "volume", "говори басом": "voice_step",
                 "говори нормально": kn.PROSODY_NORMAL_KIND, "повтори, я не расслышал": kn.REPEAT_KIND,
                 "покажи по очереди радость, грусть и удивление": kn.SHOW_QUEUE_KIND}


def names(tools) -> list:
    return [t.name for t in tools]


def snapshot(*, music: bool = False, silence: bool = False):
    session = Session(kn.AGENTS[0])
    if music:
        session.set_music(MusicStateRef("playing", track_id="t1", title="трек"))
    if silence:
        session.push_scene("Silence", "user")
    return session.snapshot


# ── Срез тулов ────────────────────────────────────────────────────────────────────────────────────────────────────


def test_a_slice_depends_on_the_phrase_control():
    """Контроль: настоящий срез A от слов зависит - без этого тест независимости B ничего бы не доказывал."""
    snap = snapshot()
    assert names(tools_view.available_tools(snap, text="Робот, спой песню")) != \
        names(tools_view.available_tools(snap, text="Робот, включи сирену"))


@pytest.mark.parametrize("music", [False, True])
def test_b_slice_does_not_depend_on_the_phrase(music):
    snap = snapshot(music=music)
    full = uv.full_slice()
    slices = {text: names(full(snap, None, kn.AGENTS[0], text)) for text in PHRASES}
    assert len({tuple(s) for s in slices.values()}) == 1, slices


@pytest.mark.parametrize("music", [False, True])
def test_bn_slice_does_not_depend_on_the_phrase_and_is_the_real_one_with_empty_text(music):
    snap = snapshot(music=music)
    narrow = uv.narrow_slice()
    slices = {text: names(narrow(snap, None, kn.AGENTS[0], text)) for text in PHRASES}
    assert len({tuple(s) for s in slices.values()}) == 1, slices
    assert slices[""] == names(tools_view.available_tools(snap, text=""))


def test_b_slice_is_state_only_silence_means_no_tools():
    assert uv.full_slice()(snapshot(silence=True), None, kn.AGENTS[0], "Робот, спой песню") == ()
    assert names(uv.full_slice()(snapshot(), None, kn.AGENTS[0], "")) != []


def test_b_slice_is_wider_than_a_and_has_no_motion():
    snap = snapshot()
    wide = names(uv.full_slice()(snap, None, kn.AGENTS[0], ""))
    assert len(wide) > kn.TOOLS_VISIBLE_MAX
    assert not set(wide) & set(kn.MOTION_TOOLS)
    for text in PHRASES:  # всё, что A показывает в каком-то ходе (кроме движения и голоса, закрытого явной просьбой), видно B
        assert set(names(tools_view.available_tools(snap, text=text))) - set(kn.MOTION_TOOLS) <= set(wide)


def test_b_slice_keeps_the_always_visible_tools_first():
    wide = names(uv.full_slice()(snapshot(), None, kn.AGENTS[0], ""))
    assert wide[:len(kn.TOOLS_ALWAYS)] == list(kn.TOOLS_ALWAYS)


# ── Tier-1 ────────────────────────────────────────────────────────────────────────────────────────────────────────


def test_safety_kinds_are_real_tier1_kinds():
    assert uv.SAFETY_KINDS <= set(kn.TIER1_KINDS)
    assert uv.SAFETY_KINDS == {"stop_all", "stop_speech", "silence", "unsilence", "new_session"}


@pytest.mark.parametrize("text, kind", sorted(TIER1_PHRASES.items()))
def test_safety_match_passes_only_safety_kinds(text, kind):
    snap = snapshot()
    assert real_match(text, snap).kind == kind  # в A команда есть
    kept = uv.safety_match()(text, snap)
    assert (kept.kind if kept else None) == (kind if kind in uv.SAFETY_KINDS else None)


def test_safety_match_keeps_unsilence_when_silenced():
    snap = snapshot(silence=True)
    assert uv.safety_match()("Робот, говори", snap).kind == "unsilence"
    assert uv.safety_match()("стоп", snap).kind == "stop_all"


# ── Подмена на стенде ─────────────────────────────────────────────────────────────────────────────────────────────


def say(text: str):
    return lambda msgs, tools: {"speech": text}


@pytest.fixture
def stand_for(tmp_path):
    stands = []

    def make(variant, rule=None, router=None):
        stand = umeasure.fake_stand(rule or say("Хорошо."), tmp_path / f"s{len(stands)}", variant, router)
        stands.append(stand)
        return stand

    yield make
    for stand in stands:
        stand.close()


def hear(stand, text):
    return asyncio.run(stand.hear(text, "anton"))


def test_volume_is_tier1_in_a_and_goes_to_the_model_in_b(stand_for):
    a, b = hear(stand_for(uv.A), "Робот, погромче"), hear(stand_for(uv.B), "Робот, погромче")
    assert (a.tier1, a.llm_calls) == ("volume", 0)
    assert (b.tier1, b.llm_calls >= 1) == ("", True)


@pytest.mark.parametrize("variant", [uv.A, uv.B, uv.C])
def test_stop_stays_tier1_in_every_variant(stand_for, variant):
    router = umeasure.OracleRouter(umeasure.Oracle({"acts": []}, ["робот"])) if variant == uv.C else None
    result = hear(stand_for(variant, router=router), "Робот, стоп")
    assert (result.tier1, result.llm_calls) == ("stop_all", 0)
    assert result.route is None  # Tier-1 решил первым: классификатор не зовётся


def test_offered_slice_is_reported_per_turn(stand_for):
    a, b = hear(stand_for(uv.A), "Робот, смени голос"), hear(stand_for(uv.B), "Робот, смени голос")
    assert "set_voice" in a.offered and "set_voice" in b.offered
    assert len(b.offered) > len(a.offered)
    assert b.variant == uv.B and a.variant == uv.A


def test_installed_swaps_only_for_the_turn_and_restores(stand_for):
    before = turn_mod.match, turn_mod.available_tools
    hear(stand_for(uv.B), "Робот, привет")
    assert (turn_mod.match, turn_mod.available_tools) == before
    assert getattr(turn_mod, "_understanding_variant", None) is None
    with pytest.raises(RuntimeError):
        with uv.installed(uv.B, []):
            with uv.installed(uv.A, []):
                pass
    assert (turn_mod.match, turn_mod.available_tools) == before


def test_installed_restores_after_an_exception():
    before = turn_mod.match, turn_mod.available_tools
    with pytest.raises(ZeroDivisionError):
        with uv.installed(uv.B, []):
            1 / 0
    assert (turn_mod.match, turn_mod.available_tools) == before


def test_variant_must_be_known(stand_for):
    with pytest.raises(ValueError):
        uv.check("Z")
    with pytest.raises(ValueError):
        dry_stand.DryStand(None, None, "", Path("."), variant="Z")


# ── Вариант C ─────────────────────────────────────────────────────────────────────────────────────────────────────


class FixedRouter:
    def __init__(self, group: str, confidence: float, accepted: bool, error: str = "") -> None:
        self._route = Route(group, confidence, accepted, 5.0, 10, 2, error)

    async def choose(self, text, music_playing=False):
        return self._route


def test_c_without_a_router_is_refused(tmp_path):
    with pytest.raises(ValueError):
        dry_stand.DryStand(None, None, "", tmp_path, variant=uv.C)


def test_c_accepted_choice_narrows_the_slice_to_the_group(stand_for):
    result = hear(stand_for(uv.C, router=FixedRouter("sound", 0.9, True)), "Робот, дзынь")
    assert "play_sound" in result.offered and "request_music" not in result.offered
    assert set(kn.TOOLS_ALWAYS) <= set(result.offered) and len(result.offered) <= kn.TOOLS_VISIBLE_MAX
    assert result.route.group == "sound"


def test_c_below_threshold_or_failed_choice_falls_back_to_b_slice(stand_for):
    wide = len(hear(stand_for(uv.B), "Робот, дзынь").offered)
    for router in (FixedRouter("sound", 0.3, False), FixedRouter("", 0.0, False, "ProviderTimeout")):
        assert len(hear(stand_for(uv.C, router=router), "Робот, дзынь").offered) == wide


def test_c_none_choice_gives_the_state_only_narrow_slice(stand_for):
    narrow = hear(stand_for(uv.C, router=FixedRouter(uv.NO_GROUP, 0.9, True)), "Робот, как дела").offered
    assert narrow == names(tools_view.available_tools(snapshot(), text=""))


def test_c_keeps_the_music_scene_next_to_the_chosen_group(stand_for):
    stand = stand_for(uv.C, router=FixedRouter("sound", 0.9, True))
    stand.player.start("клубный трек")
    offered = hear(stand, "Робот, дзынь").offered
    assert "play_sound" in offered and "stop_music" in offered
