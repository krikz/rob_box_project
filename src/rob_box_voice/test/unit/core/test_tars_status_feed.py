"""v1 ``core.tars_status_feed.describe_llm`` (issue #3253, Ш3б): имена из ``_providers`` v1.

Payload, вейки и дедупликация — ``src/rob_box_dialog/test/test_status_feed.py`` (#3912).
"""

from __future__ import annotations

from types import SimpleNamespace

from rob_box_voice.core.tars_status_feed import describe_llm


def test_describe_llm_single_provider() -> None:
    d = describe_llm(SimpleNamespace(name="deepseek"))
    assert d is not None
    assert d["provider"] == "deepseek"
    assert d["chain"] == ["deepseek"]
    assert d["source"] == "configured"


def test_describe_llm_chain_takes_primary_first() -> None:
    llm = SimpleNamespace(
        name="health-aware-fallback",
        _providers=[SimpleNamespace(name="MiniMax"), SimpleNamespace(name="deepseek")],
    )
    d = describe_llm(llm)
    assert d["provider"] == "minimax"
    assert d["chain"] == ["minimax", "deepseek"]


def test_describe_llm_unknown_model_means_no_model_key() -> None:
    d = describe_llm(SimpleNamespace(name="no-such-provider"))
    assert d is not None and "model" not in d


def test_describe_llm_without_name_is_none() -> None:
    assert describe_llm(object()) is None
