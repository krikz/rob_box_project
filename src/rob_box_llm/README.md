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

## Testing

```bash
cd src/rob_box_llm
PYTHONPATH=. python3 -m pytest test/ -v
PYTHONPATH=. python3 -m pytest test/ -k minimax        # just the MiniMax TTS provider
PYTHONPATH=. python3 -m coverage run --source=rob_box_llm -m pytest test/
python3 -m coverage report --include='rob_box_llm/*'
```

All tests are offline — the OpenAI SDK client is replaced with a fake,
and `MiniMaxTTSProvider` is exercised against `httpx.MockTransport`.

The MiniMax-specific suite covers the format round-trip (`pcm`/`wav`/`mp3`/`ogg`),
parameter mapping (voice / language / speed / volume / pitch / emotion /
`extra` allow-list), every typed error (`TTSAuthError` / `TTSRateLimitError` /
`TTSBadRequestError` / `TTSTimeoutError`), SSE streaming, and a guard that
the API key and group id never appear in any log sink.
