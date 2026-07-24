"""Harness configuration — dataclasses + YAML + env loader.

Configuration is YAML-first, with two extension points that make the
loader production-grade without dragging in a heavy settings library:

1. **Environment interleave** — every YAML string of the form
   ``"${ENV_VAR}"`` is replaced with ``os.environ["ENV_VAR"]`` before
   validation. This is the only place secrets enter the system:
   ``MINIMAX_API_KEY``, ``DEEPSEEK_API_KEY``, etc. should NEVER
   appear inline in the YAML.
2. **Layered files** — :func:`load_config` accepts a single path OR
   an iterable of paths; later files override earlier ones. This is
   the "base.yaml + env.yaml" idiom (ADR-0001 §2.5.3).

The dataclasses are frozen so :class:`HarnessConfig` instances can be
treated as immutable values (handy for replay/tests). Validation runs
in :meth:`HarnessConfig.from_dict` and raises :class:`ConfigError`
with a precise ``section`` field pointing to the offending area.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping, MutableMapping

from rob_box_harness.errors import ConfigError

# Matches ``${FOO_BAR}`` with optional whitespace inside the braces.
# Greedy on the inner name: ``${A_B}`` is ONE placeholder, not nested.
_ENV_PATTERN = re.compile(r"\$\{\s*([A-Z][A-Z0-9_]*)\s*\}")


# ---------------------------------------------------------------------------
# Dataclasses
# ---------------------------------------------------------------------------


# Allowed values for ``harness.kind`` — mirrors the kinds enumerated in
# ADR-0001 §2.5.1. Concrete harnesses (dialog/persistent/telegram) are
# registered via :class:`rob_box_harness.registry.HarnessRegistry`; this
# enum is just for the schema.
HarnessKind = str  # "dialog" | "persistent" | "telegram" | test-registered names


@dataclass(frozen=True)
class LLMConfig:
    """Configuration for the LLMProvider port."""

    provider: str
    model: str | None = None
    fallback: tuple[str, ...] = ()
    temperature: float | None = None
    max_tokens: int | None = None
    timeout_s: float | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)
    api_key: str | None = None  # may be set via ENV; never serialised


@dataclass(frozen=True)
class ToolsConfig:
    """Configuration for the ToolProvider port."""

    provider: str
    endpoint: str | None = None
    timeout_s: float | None = None
    max_concurrent: int | None = None


@dataclass(frozen=True)
class MemoryConfig:
    """Configuration for the MemoryStore port."""

    backend: str  # "in_memory" | "sqlite" | future: "redis"
    path: str | None = None
    ttl_days: int | None = None


@dataclass(frozen=True)
class EffectsConfig:
    """Configuration for the SideEffectBus."""

    bus: str  # "noop" | "recording" | "composite"
    composite: tuple[str, ...] = ()


@dataclass(frozen=True)
class TransportConfig:
    """Configuration for the Transport port."""

    kind: str  # "fake" | "ros2" (and future transports)
    topics: Mapping[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class LoggingConfig:
    """Configuration for the structured logger."""

    level: str = "INFO"
    redact: tuple[str, ...] = ()
    format: str = "structured"  # "structured" | "plain"


@dataclass(frozen=True)
class HarnessConfig:
    """Root configuration object passed to every harness.

    The schema mirrors ADR-0001 §2.5.1. ``harness`` is the only
    mandatory section; everything else has a sensible default that
    makes the smoke tests below trivial.
    """

    harness: HarnessKind
    name: str = "unnamed_harness"
    state: Mapping[str, Any] = field(default_factory=dict)
    llm: LLMConfig | None = None
    tools: ToolsConfig | None = None
    memory: MemoryConfig | None = None
    effects: EffectsConfig | None = None
    transport: TransportConfig | None = None
    logging: LoggingConfig = field(default_factory=LoggingConfig)

    # ----- factories -----------------------------------------------------

    @classmethod
    def from_dict(
        cls,
        raw: Mapping[str, Any],
        *,
        secrets: Mapping[str, str] | None = None,
    ) -> "HarnessConfig":
        """Build a :class:`HarnessConfig` from a parsed YAML dict.

        ``secrets`` is an explicit env map; if ``None`` we read
        ``os.environ``. The ``${ENV_VAR}`` substitution uses ``secrets``
        first, then ``os.environ`` (so tests can fully control the
        environment without touching the real os.environ).
        """
        if not isinstance(raw, Mapping):
            raise ConfigError(
                "top-level config must be a mapping; "
                f"got {type(raw).__name__}"
            )

        harness_section = raw.get("harness")
        if not isinstance(harness_section, Mapping):
            raise ConfigError(
                "missing or non-mapping 'harness' section",
                section="harness",
            )
        harness_kind = harness_section.get("kind")
        if not isinstance(harness_kind, str) or not harness_kind:
            raise ConfigError(
                "'harness.kind' must be a non-empty string",
                section="harness.kind",
            )
        harness_name = harness_section.get("name", "unnamed_harness")
        if not isinstance(harness_name, str):
            raise ConfigError(
                "'harness.name' must be a string when provided",
                section="harness.name",
            )

        state = harness_section.get("state", {})
        if not isinstance(state, Mapping):
            raise ConfigError(
                "'harness.state' must be a mapping",
                section="harness.state",
            )

        llm_cfg = _parse_llm(raw.get("llm"), secrets=secrets)
        tools_cfg = _parse_tools(raw.get("tools"))
        memory_cfg = _parse_memory(raw.get("memory"))
        effects_cfg = _parse_effects(raw.get("effects"))
        transport_cfg = _parse_transport(raw.get("transport"))
        logging_cfg = _parse_logging(raw.get("logging"))

        return cls(
            harness=harness_kind,
            name=harness_name,
            state=dict(state),
            llm=llm_cfg,
            tools=tools_cfg,
            memory=memory_cfg,
            effects=effects_cfg,
            transport=transport_cfg,
            logging=logging_cfg,
        )


# ---------------------------------------------------------------------------
# Loader
# ---------------------------------------------------------------------------


def load_config(
    source: "str | Path | Iterable[str | Path] | None",
    *,
    secrets: Mapping[str, str] | None = None,
) -> HarnessConfig:
    """Load and validate a :class:`HarnessConfig`.

    ``source`` accepts:

    * ``None`` — fall back to env ``ROB_BOX_CONFIG_PATH`` (or return a
      "dialog" default if even that is unset). This mirrors how
      ``rob_box_llm`` discovers its config in scripts.
    * a single path — read that YAML.
    * an iterable of paths — read all in order, deep-merge dicts,
      last wins. Convenient for ``base.yaml`` + ``pi-main.yaml`` layers.

    Returns a fully validated :class:`HarnessConfig`. Raises
    :class:`ConfigError` on any structural problem; the ``section``
    attribute points at the offending key.
    """
    if source is None:
        env_path = os.environ.get("ROB_BOX_CONFIG_PATH")
        if env_path:
            return load_config(env_path, secrets=secrets)
        return HarnessConfig.from_dict({"harness": {"kind": "dialog"}}, secrets=secrets)

    if isinstance(source, (str, Path)):
        raw = _read_yaml_file(Path(source))
        return HarnessConfig.from_dict(_interpolate_env(raw, secrets=secrets), secrets=secrets)

    merged: dict[str, Any] = {}
    for item in source:
        sub = _read_yaml_file(Path(item))
        sub = _interpolate_env(sub, secrets=secrets)
        _deep_merge(merged, sub)
    return HarnessConfig.from_dict(merged, secrets=secrets)


# ---------------------------------------------------------------------------
# Internals
# ---------------------------------------------------------------------------


def _read_yaml_file(path: Path) -> Mapping[str, Any]:
    """Read a YAML file. Imported lazily so the dev environment that
    hasn't installed PyYAML still gets a clean ImportError.
    """
    if not path.exists():
        raise ConfigError(f"config file not found: {path}")
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover — exercised on minimal envs
        raise ConfigError(
            "PyYAML is required to load YAML config; "
            "install with `pip install pyyaml`"
        ) from exc
    loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    if loaded is None:
        return {}
    if not isinstance(loaded, Mapping):
        raise ConfigError(
            f"top-level YAML in {path} must be a mapping; "
            f"got {type(loaded).__name__}"
        )
    return loaded


def _interpolate_env(
    raw: Mapping[str, Any],
    *,
    secrets: Mapping[str, str] | None,
) -> dict[str, Any]:
    """Recursively replace ``${ENV_VAR}`` placeholders with their values.

    Resolution order: explicit ``secrets`` map, then ``os.environ``.
    Unknown placeholders raise :class:`ConfigError` so typos are
    surfaced at startup, not at first request.
    """
    result: dict[str, Any] = _interpolate_value(raw, secrets=secrets)
    return result


def _interpolate_value(value: Any, *, secrets: Mapping[str, str] | None) -> Any:
    if isinstance(value, str):
        return _resolve_string(value, secrets=secrets)
    if isinstance(value, Mapping):
        return {k: _interpolate_value(v, secrets=secrets) for k, v in value.items()}
    if isinstance(value, list):
        return [_interpolate_value(item, secrets=secrets) for item in value]
    if isinstance(value, tuple):
        return tuple(_interpolate_value(item, secrets=secrets) for item in value)
    return value


def _resolve_string(text: str, *, secrets: Mapping[str, str] | None) -> str:
    def _replace(match: re.Match[str]) -> str:
        name = match.group(1)
        if secrets is not None and name in secrets:
            return secrets[name]
        env_value = os.environ.get(name)
        if env_value is None:
            raise ConfigError(
                f"unresolved environment placeholder ${{{name}}}; "
                f"set the env var or pass it via `secrets=`"
            )
        return env_value

    return _ENV_PATTERN.sub(_replace, text)


def _deep_merge(target: MutableMapping[str, Any], source: Mapping[str, Any]) -> None:
    """Deep-merge ``source`` into ``target`` in place; ``source`` wins."""
    for key, value in source.items():
        if (
            key in target
            and isinstance(target[key], MutableMapping)
            and isinstance(value, Mapping)
        ):
            _deep_merge(target[key], value)
        else:
            target[key] = value


def _parse_llm(
    raw: Any,
    *,
    secrets: Mapping[str, str] | None,
) -> LLMConfig | None:
    if raw is None:
        return None
    if not isinstance(raw, Mapping):
        raise ConfigError("'llm' section must be a mapping", section="llm")
    provider = raw.get("provider")
    if not isinstance(provider, str) or not provider:
        raise ConfigError(
            "'llm.provider' must be a non-empty string",
            section="llm.provider",
        )
    fallback = raw.get("fallback", ())
    if fallback is None:
        fallback = ()
    if not isinstance(fallback, (list, tuple)):
        raise ConfigError(
            "'llm.fallback' must be a list of provider names",
            section="llm.fallback",
        )
    extras = raw.get("extra", {})
    if not isinstance(extras, Mapping):
        raise ConfigError(
            "'llm.extra' must be a mapping",
            section="llm.extra",
        )
    timeout_s = raw.get("timeout_s")
    if timeout_s is not None and not isinstance(timeout_s, (int, float)):
        raise ConfigError(
            "'llm.timeout_s' must be a number",
            section="llm.timeout_s",
        )
    api_key_raw = raw.get("api_key")
    api_key: str | None = None
    if api_key_raw is not None:
        api_key = _resolve_string(str(api_key_raw), secrets=secrets)
    return LLMConfig(
        provider=provider,
        model=raw.get("model"),
        fallback=tuple(fallback),
        temperature=raw.get("temperature"),
        max_tokens=raw.get("max_tokens"),
        timeout_s=float(timeout_s) if timeout_s is not None else None,
        extra=dict(extras),
        api_key=api_key,
    )


def _parse_tools(raw: Any) -> ToolsConfig | None:
    if raw is None:
        return None
    if not isinstance(raw, Mapping):
        raise ConfigError("'tools' section must be a mapping", section="tools")
    provider = raw.get("provider")
    if not isinstance(provider, str) or not provider:
        raise ConfigError(
            "'tools.provider' must be a non-empty string",
            section="tools.provider",
        )
    return ToolsConfig(
        provider=provider,
        endpoint=raw.get("endpoint"),
        timeout_s=raw.get("timeout_s"),
        max_concurrent=raw.get("max_concurrent"),
    )


def _parse_memory(raw: Any) -> MemoryConfig | None:
    if raw is None:
        return None
    if not isinstance(raw, Mapping):
        raise ConfigError("'memory' section must be a mapping", section="memory")
    backend = raw.get("backend")
    if not isinstance(backend, str) or not backend:
        raise ConfigError(
            "'memory.backend' must be a non-empty string",
            section="memory.backend",
        )
    return MemoryConfig(
        backend=backend,
        path=raw.get("path"),
        ttl_days=raw.get("ttl_days"),
    )


def _parse_effects(raw: Any) -> EffectsConfig | None:
    if raw is None:
        return None
    if not isinstance(raw, Mapping):
        raise ConfigError(
            "'effects' section must be a mapping",
            section="effects",
        )
    bus = raw.get("bus")
    if not isinstance(bus, str) or not bus:
        raise ConfigError(
            "'effects.bus' must be a non-empty string",
            section="effects.bus",
        )
    composite_raw = raw.get("composite", ())
    if composite_raw is None:
        composite_raw = ()
    if not isinstance(composite_raw, (list, tuple)):
        raise ConfigError(
            "'effects.composite' must be a list",
            section="effects.composite",
        )
    return EffectsConfig(bus=bus, composite=tuple(composite_raw))


def _parse_transport(raw: Any) -> TransportConfig | None:
    if raw is None:
        return None
    if not isinstance(raw, Mapping):
        raise ConfigError(
            "'transport' section must be a mapping",
            section="transport",
        )
    kind = raw.get("kind")
    if not isinstance(kind, str) or not kind:
        raise ConfigError(
            "'transport.kind' must be a non-empty string",
            section="transport.kind",
        )
    topics = raw.get("topics", {})
    if not isinstance(topics, Mapping):
        raise ConfigError(
            "'transport.topics' must be a mapping",
            section="transport.topics",
        )
    return TransportConfig(kind=kind, topics=dict(topics))


def _parse_logging(raw: Any) -> LoggingConfig:
    if raw is None:
        return LoggingConfig()
    if not isinstance(raw, Mapping):
        raise ConfigError(
            "'logging' section must be a mapping",
            section="logging",
        )
    level = raw.get("level", "INFO")
    if not isinstance(level, str):
        raise ConfigError(
            "'logging.level' must be a string",
            section="logging.level",
        )
    redact = raw.get("redact", ())
    if redact is None:
        redact = ()
    if not isinstance(redact, (list, tuple)):
        raise ConfigError(
            "'logging.redact' must be a list",
            section="logging.redact",
        )
    fmt = raw.get("format", "structured")
    if fmt not in ("structured", "plain"):
        raise ConfigError(
            f"'logging.format' must be 'structured' or 'plain'; got {fmt!r}",
            section="logging.format",
        )
    return LoggingConfig(level=level, redact=tuple(redact), format=fmt)


__all__ = [
    "HarnessKind",
    "LLMConfig",
    "ToolsConfig",
    "MemoryConfig",
    "EffectsConfig",
    "TransportConfig",
    "LoggingConfig",
    "HarnessConfig",
    "load_config",
]
