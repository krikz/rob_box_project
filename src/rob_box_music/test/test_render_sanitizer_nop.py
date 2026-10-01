"""Санитайзер v1 ничего не меняет в программе v2 — значит, v2 он не нужен (ADR-0149 §3.2, PR-2).

Санитайзер живёт в старом ``rob_box_mcp_tools``; тест подставляет путь сам (как ``test_knowledge_legacy``)
и удаляется вместе со старым путём (PR-13…15).
"""

import pathlib
import sys

import pytest

from melodies import MELODIES, profile
from rob_box_music.arrange.compose import compose
from rob_box_music.render.renardo import render

SRC = pathlib.Path(__file__).resolve().parents[2]
if not (SRC / "rob_box_mcp_tools").is_dir():  # пакет поставлен отдельно от монорепо
    pytest.skip("нет src/rob_box_mcp_tools рядом с пакетом", allow_module_level=True)
if str(SRC / "rob_box_mcp_tools") not in sys.path:
    sys.path.append(str(SRC / "rob_box_mcp_tools"))

from rob_box_mcp_tools.core.renardo_sanitizer import sanitize_renando  # noqa: E402

#: Потолок слоя санитайзера в проде (``club_arranger.MAX_LAYER_AMP``).
MAX_AMP = 0.85


@pytest.mark.parametrize("deck", ["A", "B"])
@pytest.mark.parametrize("seed", range(20))
def test_sanitizer_is_a_nop_on_v2_program(seed, deck):
    track = compose(profile(root=seed % 12), 1, set_seed=seed, melodies=MELODIES if seed % 2 else None, deck=deck)
    program = render(track, deck)
    known = frozenset(program.synths)
    result = sanitize_renando(program.code, MAX_AMP, known_synths=known)
    assert result.security_error is None
    assert result.quality_errors == () and result.slot_error is None
    assert result.code == program.code
