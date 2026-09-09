"""Tests for :mod:`rob_box_voice.core.llm_config`.

Covers the pure provider-resolution helpers extracted from
``DialogueNode._resolve_*`` methods (P1.3 step 3, ADR-0001 §2.7).
"""

from __future__ import annotations

import pytest

from rob_box_voice.core.llm_config import (
    DEFAULT_FALLBACK_MODEL,
    PROVIDERS,
    ProviderConfig,
    ProviderOverrides,
    known_providers,
    resolve_api_key,
    resolve_base_url,
    resolve_model,
    resolve_provider_config,
)


# ── Registry sanity ────────────────────────────────────────────────────────


def test_known_providers_lists_registry_keys():
    assert set(known_providers()) == {"deepseek", "mimo"}


def test_known_providers_returns_tuple_immutable():
    result = known_providers()
    assert isinstance(result, tuple)
    # Adding to the returned tuple must not mutate the registry.
    with pytest.raises(AttributeError):
        result.append("new")  # type: ignore[attr-defined]


def test_providers_registry_has_required_fields():
    for name, entry in PROVIDERS.items():
        assert "base_url" in entry, f"{name} missing base_url"
        assert "model" in entry, f"{name} missing model"
        assert "fallback_model" in entry, f"{name} missing fallback_model"
        assert "env_vars" in entry, f"{name} missing env_vars"
        assert entry["env_vars"], f"{name} env_vars is empty"


def test_deepseek_points_to_deepseek_host():
    entry = PROVIDERS["deepseek"]
    assert entry["base_url"].startswith("https://api.deepseek.com")
    assert "DEEPSEEK_API_KEY" in entry["env_vars"]


def test_mimo_points_to_mimo_host():
    entry = PROVIDERS["mimo"]
    assert entry["base_url"].startswith("https://api.xiaomimimo.com")
    assert "MIMO_API_KEY" in entry["env_vars"]


# ── resolve_api_key ────────────────────────────────────────────────────────


def test_resolve_api_key_prefers_override(monkeypatch):
    monkeypatch.setenv("DEEPSEEK_API_KEY", "from-env")
    assert (
        resolve_api_key("deepseek", override="from-param", env=monkeypatch)
        == "from-param"
    )


def _env(monkeypatch) -> dict[str, str]:
    """Snapshot the current os.environ (after monkeypatch deltas) as a plain dict.

    The llm_config helpers accept ``Mapping[str, str]`` rather than
    ``os.environ`` directly, so that tests don't have to mutate the
    global env. MonkeyPatch is a Mapping but lacks ``.get`` for
    ``os.environ``-style reads — pass a dict snapshot instead.
    """
    import os
    return dict(os.environ)


def test_resolve_api_key_falls_back_to_first_env(monkeypatch):
    monkeypatch.setenv("DEEPSEEK_API_KEY", "primary")
    monkeypatch.delenv("LLM_API_KEY", raising=False)
    assert resolve_api_key("deepseek", env=_env(monkeypatch)) == "primary"


def test_resolve_api_key_falls_back_to_second_env(monkeypatch):
    monkeypatch.delenv("DEEPSEEK_API_KEY", raising=False)
    monkeypatch.setenv("LLM_API_KEY", "secondary")
    assert resolve_api_key("deepseek", env=_env(monkeypatch)) == "secondary"


def test_resolve_api_key_raises_when_missing(monkeypatch):
    monkeypatch.delenv("DEEPSEEK_API_KEY", raising=False)
    monkeypatch.delenv("LLM_API_KEY", raising=False)
    with pytest.raises(RuntimeError) as exc_info:
        resolve_api_key("deepseek", env=_env(monkeypatch))
    assert "API key not found" in str(exc_info.value)
    assert "DEEPSEEK_API_KEY" in str(exc_info.value)


def test_resolve_api_key_unknown_provider():
    with pytest.raises(KeyError) as exc_info:
        resolve_api_key("nonexistent", env={})
    assert "nonexistent" in str(exc_info.value)


def test_resolve_api_key_uses_provided_env_dict():
    env = {"DEEPSEEK_API_KEY": "from-dict"}
    assert resolve_api_key("deepseek", env=env) == "from-dict"


# ── resolve_base_url ───────────────────────────────────────────────────────


def test_resolve_base_url_prefers_override():
    assert (
        resolve_base_url("deepseek", override="https://override.example/v1")
        == "https://override.example/v1"
    )


def test_resolve_base_url_uses_registry_default():
    assert (
        resolve_base_url("deepseek")
        == PROVIDERS["deepseek"]["base_url"]
    )


def test_resolve_base_url_unknown_provider_returns_empty():
    # Empty override + unknown provider → empty string (no raise, for
    # caller convenience — caller still gets a usable config).
    assert resolve_base_url("unknown", override="") == ""


# ── resolve_model / resolve_fallback_model ────────────────────────────────


def test_resolve_model_prefers_override():
    assert (
        resolve_model("deepseek", override="custom-model-v9")
        == "custom-model-v9"
    )


def test_resolve_model_uses_registry_default():
    assert resolve_model("deepseek") == PROVIDERS["deepseek"]["model"]


def test_resolve_fallback_model_uses_registry_default():
    assert (
        resolve_model("deepseek", field_name="fallback_model")
        == PROVIDERS["deepseek"]["fallback_model"]
    )


def test_resolve_fallback_model_prefers_override():
    assert (
        resolve_model("deepseek", override="custom-fb", field_name="fallback_model")
        == "custom-fb"
    )


def test_resolve_model_unknown_provider_returns_default():
    assert resolve_model("unknown") == DEFAULT_FALLBACK_MODEL


# ── resolve_provider_config (one-shot) ─────────────────────────────────────


def test_resolve_provider_config_returns_complete_config(monkeypatch):
    monkeypatch.setenv("DEEPSEEK_API_KEY", "k")
    cfg = resolve_provider_config("deepseek", env=_env(monkeypatch))
    assert isinstance(cfg, ProviderConfig)
    assert cfg.provider == "deepseek"
    assert cfg.api_key == "k"
    assert cfg.base_url == PROVIDERS["deepseek"]["base_url"]
    assert cfg.model == PROVIDERS["deepseek"]["model"]
    assert cfg.fallback_model == PROVIDERS["deepseek"]["fallback_model"]
    assert "DEEPSEEK_API_KEY" in cfg.env_vars


def test_resolve_provider_config_applies_overrides(monkeypatch):
    monkeypatch.delenv("DEEPSEEK_API_KEY", raising=False)
    monkeypatch.delenv("LLM_API_KEY", raising=False)
    overrides = ProviderOverrides(
        api_key="explicit-key",
        base_url="https://custom.example/v1",
        model="custom-model",
        fallback_model="custom-fallback",
    )
    cfg = resolve_provider_config("deepseek", overrides=overrides, env=_env(monkeypatch))
    assert cfg.api_key == "explicit-key"
    assert cfg.base_url == "https://custom.example/v1"
    assert cfg.model == "custom-model"
    assert cfg.fallback_model == "custom-fallback"


def test_resolve_provider_config_raises_on_missing_key(monkeypatch):
    monkeypatch.delenv("DEEPSEEK_API_KEY", raising=False)
    monkeypatch.delenv("LLM_API_KEY", raising=False)
    with pytest.raises(RuntimeError):
        resolve_provider_config("deepseek", env=_env(monkeypatch))


def test_resolve_provider_config_returns_empty_key_when_allowed(monkeypatch):
    monkeypatch.delenv("DEEPSEEK_API_KEY", raising=False)
    monkeypatch.delenv("LLM_API_KEY", raising=False)
    cfg = resolve_provider_config(
        "deepseek", env=_env(monkeypatch), error_on_missing_key=False
    )
    assert cfg.api_key == ""
    assert cfg.base_url  # still resolved


def test_resolve_provider_config_unknown_provider_raises():
    with pytest.raises(KeyError):
        resolve_provider_config("unknown", env={})


def test_resolve_provider_config_as_dict_round_trip(monkeypatch):
    monkeypatch.setenv("DEEPSEEK_API_KEY", "k")
    cfg = resolve_provider_config("deepseek", env=_env(monkeypatch))
    d = cfg.as_dict()
    assert d["provider"] == "deepseek"
    assert d["api_key"] == "k"
    assert isinstance(d["env_vars"], list)


def test_resolve_provider_config_uses_custom_registry():
    custom_registry = {
        "testprov": {
            "base_url": "https://test.example/v1",
            "model": "test-model",
            "fallback_model": "test-fb",
            "env_vars": ["TEST_API_KEY"],
        },
    }
    overrides = ProviderOverrides(api_key="k-from-override")
    cfg = resolve_provider_config(
        "testprov",
        overrides=overrides,
        env={},
        registry=custom_registry,
    )
    assert cfg.provider == "testprov"
    assert cfg.api_key == "k-from-override"
    assert cfg.base_url == "https://test.example/v1"
    assert cfg.model == "test-model"
    assert cfg.fallback_model == "test-fb"


def test_resolve_provider_config_picks_first_env_var_in_order(monkeypatch):
    monkeypatch.setenv("MIMO_API_KEY", "mimo-primary")
    monkeypatch.setenv("LLM_API_KEY", "generic")
    cfg = resolve_provider_config("mimo", env=_env(monkeypatch))
    # MIMO_API_KEY is first in mimo.env_vars — must take precedence.
    assert cfg.api_key == "mimo-primary"