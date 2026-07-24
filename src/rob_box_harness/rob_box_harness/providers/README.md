# MiniMax LLM-provider (harness-side)

This directory contains the harness-side `MiniMaxProvider` —
:class:`rob_box_harness.providers.minimax.MiniMaxProvider`. It is the
LLM-port adapter for the [MiniMax](https://www.minimax.io) hosted
chat-completions API, wired into the
[Harness Framework](../README.md) (ADR-0001).

The module is a thin wrapper around the upstream
`rob_box_llm.providers.minimax.MiniMaxProvider`, which already
implements the full `LLMProvider` contract along with the M1-M10
requirements from ADR-0001 §2.6 (capabilities, image-size limit,
`base_resp` envelope, API-key redaction, auth hardening, `aclose`).
The harness-side wrapper adds three things on top:

1. **Env-based authentication** — the API key is read from
   `MINIMAX_API_KEY` (or supplied explicitly via `api_key=`). The
   harness refuses to silently fall back to a YAML-embedded literal,
   per ADR-0001 §2.5.2.
2. **A `chat(messages, **kwargs)` convenience method** — wraps
   `LLMProvider.complete` so callers can pass `temperature=`,
   `max_tokens=`, etc. by keyword without first building an
   `LLMSettings` object.
3. **Optional retry with exponential backoff** — when the upstream
   provider raises `RateLimitError` or `TimeoutError` (transient
   only), the call is retried with
   `delay = base * (2 ** attempt) + jitter`. `AuthError`,
   `ContentFilterError` and `CapabilityUnavailableError` bypass the
   retry — they are programming errors, not transient failures.

## TL;DR

```python
import asyncio
from rob_box_harness.providers.minimax import (
    MiniMaxProvider,
    build_minimax_provider,
    RetryPolicy,
)
from rob_box_harness.config import HarnessConfig, LLMConfig

# 1. Build from a HarnessConfig (the production path).
config = HarnessConfig.from_dict(
    {
        "harness": {"kind": "echo"},
        "llm": {
            "provider": "minimax",
            "model": "MiniMax-M3",
            "timeout_s": 30,
            "api_key": "${MINIMAX_API_KEY}",  # resolved via env
        },
    },
    # secrets={"MINIMAX_API_KEY": "sk-test"},  # for tests
)
provider = build_minimax_provider(config.llm)

# 2. Or construct directly (handy for scripts / notebooks).
provider = MiniMaxProvider(
    api_key="sk-test",
    model="MiniMax-M3",
    retry=RetryPolicy(max_attempts=4, backoff_base=0.5),
)

# 3. Use the convenience chat() shortcut.
async def go() -> None:
    response = await provider.chat(
        [
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": "Say hi in one word."},
        ],
        temperature=0.0,
        max_tokens=16,
    )
    print(response.content)

asyncio.run(go())
```

## YAML configuration

Add an `llm` section to your harness config:

```yaml
harness:
  kind: dialog
  name: dialog_main

llm:
  provider: minimax
  model: MiniMax-M3        # default; vision-capable
  timeout_s: 30            # HTTP timeout; default 30
  api_key: ${MINIMAX_API_KEY}   # required; resolved from env
  fallback:
    - deepseek
    - mimo
```

The `api_key` field is **required** and **must** use the `${ENV_VAR}`
placeholder form. Bare strings are rejected at load time by
`HarnessConfig.from_dict` (`ConfigError` with `section="llm.api_key"`).

See [ADR-0001 §2.5.2](../../../../docs/adr/0001-harness-architecture.md)
for the full YAML schema and the secret-resolution rules.

## Environment variables

| Variable               | Required | Purpose                                                       |
|------------------------|----------|---------------------------------------------------------------|
| `MINIMAX_API_KEY`      | yes      | API key for the MiniMax global endpoint (`api.minimax.io`).   |

The provider never reads or writes any other env vars, and the API
key is never logged. Attach `MiniMaxRedactedLogFilter` to your root
logger if you want belt-and-braces redaction of accidental
`Authorization` leaks:

```python
import logging
from rob_box_harness.providers.minimax import MiniMaxRedactedLogFilter

root = logging.getLogger()
root.addFilter(MiniMaxRedactedLogFilter(api_key_env="MINIMAX_API_KEY"))
```

## Capabilities

| Capability        | Model-dependent        | Notes                                                |
|-------------------|------------------------|------------------------------------------------------|
| `text`            | all                    | Always `True`.                                       |
| `streaming_text`  | all                    | `True`.                                              |
| `tools`           | all                    | `True`; use `complete()` (not `stream()`) for tool calls. |
| `streaming_tools` | n/a                    | `False` — streaming tool-call delta aggregation is not implemented. |
| `image_input`     | vision models only     | `True` for `MiniMax-M3`, `MiniMax-M2-vision`, `*vision*`; `False` otherwise. |

Use `provider.capabilities_for(model_name)` to narrow the
capabilities to a specific model (the harness' fallback chain
relies on this).

```python
caps = provider.capabilities_for("MiniMax-M3")
assert caps.image_input is True

caps = provider.capabilities_for("MiniMax-M2.7")
assert caps.image_input is False
```

## Error handling

The provider raises typed errors from `rob_box_llm.errors`:

| Error                       | Retryable? | Meaning                                                |
|-----------------------------|------------|--------------------------------------------------------|
| `RateLimitError`            | yes        | 429 / quota exhausted.                                 |
| `TimeoutError`              | yes        | Network or read timeout.                               |
| `AuthError`                 | no         | 401 / 403 — bad API key or revoked token.              |
| `ContentFilterError`        | no         | Provider-side safety refusal.                          |
| `CapabilityUnavailableError`| no         | E.g. `image_input=True` on a text-only model.          |
| `ProviderError`             | no         | Catch-all for the upstream provider envelope.          |

`AuthError`, `ContentFilterError` and `CapabilityUnavailableError`
**always** propagate immediately — they are programming errors, not
transient failures, and retrying them only hides a real bug.

## Retry policy

`RetryPolicy` is a small frozen dataclass:

```python
@dataclass(frozen=True)
class RetryPolicy:
    max_attempts: int = 3       # including the initial call
    backoff_base: float = 0.5   # seconds; first retry waits this much
    backoff_jitter: float = 0.25  # uniform jitter in [0, backoff_jitter)
```

`max_attempts=1` disables retries. The wait time before retry
attempt `n` (1-based) is `backoff_base * 2 ** (n - 1)` plus a uniform
random jitter in `[0, backoff_jitter)`. The provider sleeps
`asyncio.sleep(delay)` between attempts so the OS event loop stays
responsive.

The retry loop wraps `complete()` only. `stream()` is wrapped too
for the initial request round-trip, but mid-stream errors become
the caller's problem (they surface as `LLMChunk(finish_reason="error")`
once the first chunk has been yielded — see
`rob_box_llm.provider.LLMProvider.stream`).

## Testing

The test suite lives in [`test/test_minimax_provider.py`](../../test/test_minimax_provider.py).
It uses a fake OpenAI SDK client (mirror of the same pattern in
`rob_box_llm/test/test_minimax_provider.py`) so no real network is
touched. Coverage includes:

- Provider identity (`name`, `capabilities`, `capabilities_for(model)`).
- Auth: missing `MINIMAX_API_KEY` raises `ConfigError`; explicit
  `api_key=` wins; env-var threading via the `env=` kwarg of
  `build_minimax_provider`.
- Request shape: model, messages, thinking-policy, tools, image parts.
- Response parsing: text + tool calls, `base_resp` envelope.
- Error mapping: `AuthError` / `RateLimitError` / `TimeoutError` /
  `ContentFilterError` / `CapabilityUnavailableError`.
- Retry loop: retries on transient errors, sleeps `backoff + jitter`,
  re-raises after `max_attempts`; non-transient errors propagate
  immediately.
- `chat(messages, **kwargs)` shortcut: builds `LLMSettings` and
  delegates to `complete()`.
- `aclose()` is idempotent.

Run the suite:

```bash
cd src/rob_box_harness
python -m pytest test/test_minimax_provider.py -v
```
