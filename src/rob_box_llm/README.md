# rob_box_llm

Shared LLM-provider abstraction. P0 foundation per `docs/adr/0001-harness-architecture.md`
and `docs/refactoring-plan.md`.

## What lives here

* `LLMProvider` — async ABC; `complete()` and `stream()`.
* `LLMMessage`, `LLMResponse`, `LLMChunk`, `LLMSettings`, `ToolCall`, `ToolResult` — value objects.
* `DeepSeekProvider`, `MiMoProvider` — concrete providers built on
  `openai.AsyncOpenAI`. Same protocol, different `base_url` / default model.
* `FakeLLMProvider` — deterministic in-memory stand-in for tests; recordable,
  scriptable, supports `on_complete` / `on_stream` callbacks.
* `errors.ProviderError` and friends — typed exception hierarchy. All SDK
  exceptions get mapped onto these by the OpenAI-compatible base class.

## What does NOT live here (yet)

* Fallback / retry logic — P1 (`FallbackProvider` wraps two providers).
* ROS integration. This module is ROS-free on purpose.
* The existing `rob_box_voice.llm.provider_manager.ProviderManager` is
  **untouched** per P0 rules — migration of live consumers happens in P1.

## Usage

```python
from rob_box_llm import DeepSeekProvider, LLMMessage, LLMSettings

provider = DeepSeekProvider(api_key="...", model="deepseek-chat")
resp = await provider.complete(
    [
        LLMMessage(role="system", content="Be terse."),
        LLMMessage(role="user", content="hi"),
    ],
    settings=LLMSettings(temperature=0.0),
)
print(resp.content)
```

## TTS providers

* **`MiniMaxTTSProvider`** — concrete impl over MiniMax T2A v2 HTTP
  (`https://api.minimax.io/v1/t2a_v2`). See
  [`docs/guides/MINIMAX_TTS.md`](../../docs/guides/MINIMAX_TTS.md) for the
  end-to-end user guide (API key, env vars, yaml config, supported voices
  and languages, troubleshooting, code samples), and
  [`docs/architecture/minimax-tts-architecture.md`](../../docs/architecture/minimax-tts-architecture.md)
  for the implementation contract.
* **`FakeTTSProvider`** — deterministic in-memory impl, for tests and
  offline dev (echoes text as a synthetic PCM buffer; `aclose()` is a
  no-op).

Quick start:

```python
import asyncio, os
from rob_box_llm import MiniMaxTTSProvider, TTSSettings

async def main():
    provider = MiniMaxTTSProvider()  # reads MINIMAX_API_KEY / MINIMAX_GROUP_ID from env
    try:
        audio = await provider.synthesize(
            "Привет!", settings=TTSSettings(voice="male-qn-qingse", language="ru")
        )
        print(f"{len(audio.samples)} samples @ {audio.sample_rate} Hz")
    finally:
        await provider.aclose()

asyncio.run(main())
```

ROS2 wiring lives in `rob_box_voice/tts_node.py` (opt-in via
`tts_node.provider = "minimax"`); see the user guide for launch args
and a copy-pasteable `voice_assistant.yaml` snippet.

## Testing

```bash
cd src/rob_box_llm
PYTHONPATH=. python3 -m pytest test/ -v
PYTHONPATH=. python3 -m pytest test/ -k minimax        # just the MiniMax TTS provider
PYTHONPATH=. python3 -m pytest test/ -k 'minimax or tts_conformance' \
    --cov=rob_box_llm.providers.minimax_tts \
    --cov-report=term-missing \
    --cov-fail-under=85        # coverage gate (mirrors CI)
PYTHONPATH=. python3 -m coverage run --source=rob_box_llm -m pytest test/
python3 -m coverage report --include='rob_box_llm/*'
```

Or from the repo root via the bundled `Makefile`:

```bash
make test-tts           # full suite + 85% coverage gate (CI parity)
make test-tts-fast      # skip the coverage gate for faster local feedback
make test-tts-verbose   # -vv + stdout-captured logs for debugging
```

The CI job `.github/workflows/G-TTS-Provider-Tests.yml` runs the same
`pytest -k 'minimax or tts_conformance' … --cov-fail-under=85` invocation
on every push and PR, and uploads the coverage report
(`coverage.xml` + `htmlcov/`) as a build artifact.

All tests are offline — the OpenAI SDK client is replaced with a fake,
and `MiniMaxTTSProvider` is exercised against `httpx.MockTransport`.

The MiniMax-specific suite covers the format round-trip (`pcm`/`wav`/`mp3`/`ogg`),
parameter mapping (voice / language / speed / volume / pitch / emotion /
`extra` allow-list), every typed error (`TTSAuthError` / `TTSRateLimitError` /
`TTSBadRequestError` / `TTSTimeoutError`), SSE streaming, and a guard that
the API key and group id never appear in any log sink.

### Conformance suite

`test/test_tts_conformance.py` is the cross-provider contract check. It
parametrises the `TTSProvider` ABC contract (signature shape, return
types, `finish_reason` semantics, `aclose()` idempotency, pre-flight
empty-text guard) over every concrete provider — currently
`FakeTTSProvider` and `MiniMaxTTSProvider`. Adding a new provider to
`rob_box_llm.providers` is a one-line edit to `CONFORMANCE_PROVIDERS`
in that file; the entire conformance matrix then runs against the new
implementation.
