"""End-to-end smoke tests for the harness + MiniMax provider stack.

ADR-0001 P0 / A-card: prove that ``EchoHarness`` (the canonical
request/response loop) drives the **real** ``HarnessMiniMaxProvider``
over the wire, and that ``MiniMaxTTSProvider`` synthesises a real
``TTSAudio`` payload via the harness-side wrapper. Every scenario:

* runs through production-like code paths (no ``MockTransport``,
  no ``respx`` rewrites, no stub OpenAI client),
* reads credentials from ``os.environ`` (the same surface
  ``docker-compose`` uses),
* uses the production timeouts (60s for LLM, 30s for TTS),
* writes a per-scenario log file under ``tests/integration/_artifacts/``
  so the resulting transcript can be embedded verbatim into
  ``docs/adr/0009-integration-test-report.md``.

A scenario **skips** with a clear ``MINIMAX_API_KEY missing`` (or
``MINIMAX_GROUP_ID missing``) reason when the matching credential
is absent. This is the same behaviour the project ships in
``conftest.py`` for the unit-test runner — a CI without secrets
goes green instead of red.

The one scenario that does NOT require real credentials is the
``auth_probe`` test: it deliberately sends an obviously-invalid
bearer so the upstream returns its structured error envelope.
That gives the test author a way to **prove the wire is open**
end-to-end on any host that can reach ``api.minimax.io``, even
without a production key. With a real key, the same code path
performs a full round-trip — the assertion stays identical
because the probe tests the *failure path* and a successful call
would replace the typed error with an ``LLMResponse``.

Run from the worktree root::

    pytest tests/integration/test_e2e_harness_minimax.py -v -s \\
        --override-ini="addopts="

The ``--override-ini="addopts="`` is required because the project's
root ``pytest.ini`` enforces coverage gates that are not relevant
to integration runs.
"""
from __future__ import annotations

import asyncio
import json
import os
import socket
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any, Mapping

import pytest


# ---------------------------------------------------------------------------
# Path bootstrap — pytest.ini lives in the worktree root, so src/ is a
# sibling of tests/. We add the harness/llm/voice roots explicitly so a
# ``pytest`` invocation from the repo root works without any
# PYTHONPATH surgery.
# ---------------------------------------------------------------------------
_THIS = Path(__file__).resolve()
_WORKTREE = _THIS.parents[2]
for _p in (
    _WORKTREE / "src" / "rob_box_harness",
    _WORKTREE / "src" / "rob_box_llm",
    _WORKTREE / "src" / "rob_box_voice",
):
    if _p.exists():
        sys.path.insert(0, str(_p))

from rob_box_harness import HarnessConfig  # noqa: E402
from rob_box_harness.harnesses.echo import EchoHarness  # noqa: E402
from rob_box_harness.providers.minimax import (  # noqa: E402
    DEFAULT_BASE_URL as LLM_BASE_URL,
    DEFAULT_MODEL as LLM_DEFAULT_MODEL,
    HarnessMiniMaxProvider,
    RetryPolicy,
)
from rob_box_harness.tts.minimax_tts import (  # noqa: E402
    DEFAULT_BASE_URL as TTS_BASE_URL,
    DEFAULT_MODEL as TTS_DEFAULT_MODEL,
)
from rob_box_llm.errors import ProviderError  # noqa: E402
from rob_box_llm.provider import (  # noqa: E402
    LLMMessage,
    LLMResponse,
    LLMSettings,
)
from rob_box_llm.tts import (  # noqa: E402
    TTSAudio,
    TTSFormat,
    TTSSettings,
)


# Production-like timeouts, as required by the task brief.
LLM_TIMEOUT_S: float = 60.0
TTS_TIMEOUT_S: float = 30.0


# ---------------------------------------------------------------------------
# Skip-with-reason helpers
# ---------------------------------------------------------------------------


def _require_env(*vars: str) -> Mapping[str, str]:
    """Return {var: value} for every requested var, or skip the test."""
    out: dict[str, str] = {}
    missing: list[str] = []
    for v in vars:
        val = os.environ.get(v, "")
        if val:
            out[v] = val
        else:
            missing.append(v)
    if missing:
        pytest.skip(
            f"integration scenario requires {', '.join(missing)}; "
            f"set them in the environment to enable real round-trips.",
            allow_module_level=False,
        )
    return out


# ---------------------------------------------------------------------------
# Lightweight connectivity probe — fail fast if the network is unreachable,
# so we don't burn the 60s LLM timeout on a 30s DNS / connect hang.
# ---------------------------------------------------------------------------


def _probe_endpoint(base_url: str, *, timeout: float = 5.0) -> None:
    """Verify ``base_url`` is reachable. Skips with a clear reason on failure.

    ``base_url`` is the API root (e.g. ``https://api.minimax.io``).
    We just need a TCP / TLS round-trip to the host — we don't
    care about the HTTP status, since the upstream endpoints return
    different codes for GET vs POST and we want a real HTTP client
    for that anyway.
    """
    from urllib.parse import urlparse

    parsed = urlparse(base_url)
    host = parsed.hostname
    if host is None:
        pytest.skip(f"invalid base URL {base_url!r}")
    try:
        with socket.create_connection((host, 443), timeout=timeout):
            return
    except OSError as exc:
        pytest.skip(
            f"network unreachable for {host}:443 ({exc}); "
            "integration scenario cannot proceed."
        )


# ---------------------------------------------------------------------------
# Artifact directory — every test writes its log here.
# ---------------------------------------------------------------------------
ARTIFACTS = _THIS.parent / "_artifacts"
ARTIFACTS.mkdir(parents=True, exist_ok=True)


def _write_log(name: str, payload: Mapping[str, Any]) -> Path:
    """Persist a per-scenario transcript JSON under ``_artifacts/``."""
    path = ARTIFACTS / f"{name}.json"
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False, default=str)
    return path


# ---------------------------------------------------------------------------
# Scenario 1 — Harness LLM round-trip via EchoHarness
# ---------------------------------------------------------------------------


@pytest.mark.integration
@pytest.mark.network
@pytest.mark.asyncio
async def test_e2e_harness_minimax_llm_text() -> None:
    """EchoHarness + HarnessMiniMaxProvider → real LLMResponse.

    Probes the wire end-to-end:

      user_text  →  EchoHarness.step()  →
      HarnessMiniMaxProvider.complete()  →
      OpenAI SDK over https://api.minimax.io/v1/chat/completions  →
      LLMResponse(...)
    """
    env = _require_env("MINIMAX_API_KEY")
    api_key = env["MINIMAX_API_KEY"]
    _probe_endpoint(LLM_BASE_URL)

    llm = HarnessMiniMaxProvider(
        api_key=api_key,
        model=LLM_DEFAULT_MODEL,
        timeout=LLM_TIMEOUT_S,
        retry=RetryPolicy(max_attempts=1),
        thinking=None,  # disable thinking policy — OpenAI SDK doesn't accept it
    )
    config = HarnessConfig(
        harness="echo",
        name="e2e_llm_text",
        state={"scenario": "e2e_harness_minimax_llm_text"},
    )
    harness = EchoHarness(config, llm=llm)
    user_text = (
        "Reply with exactly one short line in English starting with the word "
        "INTEGRATION. No trailing punctuation."
    )
    log: dict[str, Any] = {
        "scenario": "e2e_harness_minimax_llm_text",
        "endpoint": f"{LLM_BASE_URL}/chat/completions",
        "model": LLM_DEFAULT_MODEL,
        "timeout_s": LLM_TIMEOUT_S,
        "user_text": user_text,
    }
    started = time.monotonic()
    try:
        await harness.init()
        result = await harness.run(user_text)
        elapsed = time.monotonic() - started
    finally:
        await harness.teardown()

    output = result.output
    log["elapsed_s"] = round(elapsed, 3)
    log["output_type"] = type(output).__name__
    log["output"] = output

    # Contract checks: the LLMResponse typing must hold.
    assert isinstance(output, str), (
        f"harness returned {type(output).__name__}, expected str; "
        f"full result={result!r}"
    )
    assert output.strip(), f"empty response: {output!r}"

    log["status"] = "PASS"
    log_path = _write_log("e2e_harness_minimax_llm_text", log)
    print(f"\n[scenario 1] PASS — {len(output)} chars in {elapsed:.2f}s → {log_path}")


# ---------------------------------------------------------------------------
# Scenario 2 — Upstream LLM provider typed round-trip
# ---------------------------------------------------------------------------


@pytest.mark.integration
@pytest.mark.network
@pytest.mark.asyncio
async def test_e2e_upstream_minimax_llm_typing() -> None:
    """Upstream ``rob_box_llm.providers.minimax.MiniMaxProvider`` typing.

    Exercises the LLMResponse / LLMMessage / LLMSettings shapes end to
    end without the harness on top. Proves the upstream provider —
    the same one ``HarnessMiniMaxProvider`` composes — returns a
    well-typed object with all the contract fields populated.
    """
    env = _require_env("MINIMAX_API_KEY")
    api_key = env["MINIMAX_API_KEY"]
    _probe_endpoint(LLM_BASE_URL)

    from rob_box_llm.providers.minimax import MiniMaxProvider as UpstreamLLM

    provider = UpstreamLLM(
        api_key=api_key,
        model=LLM_DEFAULT_MODEL,
        timeout=LLM_TIMEOUT_S,
        thinking=None,  # disable thinking-policy for the simple typing probe
    )
    messages = [LLMMessage(role="user", content="Reply with the single word: OK")]
    settings = LLMSettings(max_tokens=16, temperature=0.0)

    log: dict[str, Any] = {
        "scenario": "e2e_upstream_minimax_llm_typing",
        "endpoint": f"{LLM_BASE_URL}/chat/completions",
        "model": LLM_DEFAULT_MODEL,
        "messages": [asdict(m) for m in messages],
        "settings": {
            "model": settings.model,
            "max_tokens": settings.max_tokens,
            "temperature": settings.temperature,
        },
    }
    started = time.monotonic()
    try:
        response = await provider.complete(messages, settings=settings)
    finally:
        await provider.aclose()
    elapsed = time.monotonic() - started

    log["elapsed_s"] = round(elapsed, 3)
    log["response_type"] = type(response).__name__
    log["finish_reason"] = response.finish_reason
    log["content_len"] = len(response.content)
    log["content_preview"] = response.content[:200]
    log["usage"] = dict(response.usage)

    # Contract: LLMResponse is a frozen dataclass; every contract
    # attribute must be present and of the right type.
    assert isinstance(response, LLMResponse), type(response).__name__
    assert isinstance(response.content, str)
    assert response.content.strip(), f"empty content: {response.content!r}"
    assert response.finish_reason in {"stop", "length", "tool_calls"}, response.finish_reason

    log["status"] = "PASS"
    log_path = _write_log("e2e_upstream_minimax_llm_typing", log)
    print(
        f"\n[scenario 2] PASS — LLMResponse.finish_reason={response.finish_reason!r} "
        f"in {elapsed:.2f}s → {log_path}"
    )


# ---------------------------------------------------------------------------
# Scenario 3 — Upstream TTS provider real synthesis
# ---------------------------------------------------------------------------


@pytest.mark.integration
@pytest.mark.network
@pytest.mark.asyncio
async def test_e2e_upstream_minimax_tts_synthesize() -> None:
    """Real ``MiniMaxTTSProvider.synthesize`` round-trip.

    Produces a real audio buffer over the wire and validates the
    ``TTSAudio`` shape: non-empty ``samples``, sane ``sample_rate``,
    and a ``format`` that matches what we asked for. We do **not**
    attempt to play the audio — the contract is bytes + sample rate;
    playback is the ROS sink's job.
    """
    env = _require_env("MINIMAX_API_KEY", "MINIMAX_GROUP_ID")
    api_key = env["MINIMAX_API_KEY"]
    group_id = env["MINIMAX_GROUP_ID"]
    _probe_endpoint(TTS_BASE_URL)

    from rob_box_llm.providers.minimax_tts import MiniMaxTTSProvider as UpstreamTTS

    provider = UpstreamTTS(
        api_key=api_key,
        group_id=group_id,
        base_url=TTS_BASE_URL,
        default_voice="male-qn-qingse",
        default_model=TTS_DEFAULT_MODEL,
        timeout=TTS_TIMEOUT_S,
    )
    settings = TTSSettings(
        voice="male-qn-qingse",
        model=TTS_DEFAULT_MODEL,
        language="ru",
        speed=1.0,
        sample_rate=32_000,
        format=TTSFormat.PCM,
    )
    text = "Привет, мир."

    log: dict[str, Any] = {
        "scenario": "e2e_upstream_minimax_tts_synthesize",
        "endpoint": TTS_BASE_URL + "/v1/t2a_v2",
        "model": TTS_DEFAULT_MODEL,
        "voice": settings.voice,
        "language": settings.language,
        "sample_rate_hz": settings.sample_rate,
        "format": settings.format.value,
        "text": text,
    }
    started = time.monotonic()
    try:
        audio = await provider.synthesize(text, settings=settings)
    finally:
        await provider.aclose()
    elapsed = time.monotonic() - started

    log["elapsed_s"] = round(elapsed, 3)
    log["audio_type"] = type(audio).__name__
    log["samples_bytes"] = len(audio.samples)
    log["sample_rate_hz"] = audio.sample_rate
    log["format"] = audio.format.value
    log["duration_s_est"] = round(audio.duration_s, 3)

    # Contract: TTSAudio is a frozen dataclass with non-empty bytes.
    assert isinstance(audio, TTSAudio)
    assert isinstance(audio.samples, bytes)
    assert len(audio.samples) > 0, f"empty audio buffer (got {len(audio.samples)} bytes)"
    assert audio.sample_rate > 0
    assert audio.format in {TTSFormat.PCM, TTSFormat.MP3, TTSFormat.WAV, TTSFormat.OGG}

    log["status"] = "PASS"
    log_path = _write_log("e2e_upstream_minimax_tts_synthesize", log)
    print(
        f"\n[scenario 3] PASS — TTSAudio "
        f"{len(audio.samples)} bytes @ {audio.sample_rate}Hz "
        f"({audio.duration_s:.2f}s) in {elapsed:.2f}s → {log_path}"
    )


# ---------------------------------------------------------------------------
# Scenario 4 — Auth probe: real wire round-trip with an invalid key
# ---------------------------------------------------------------------------


@pytest.mark.integration
@pytest.mark.asyncio
async def test_e2e_harness_minimax_llm_auth_probe() -> None:
    """End-to-end probe: harness + provider reach api.minimax.io and
    receive the upstream's auth-error envelope.

    A real key is NOT required for this scenario — the test
    deliberately sends an obviously-invalid bearer token so the
    upstream returns its structured error (HTTP 401,
    ``base_resp.status_code=1004``). The point is to prove that:

      1. ``EchoHarness`` → ``HarnessMiniMaxProvider.complete`` → the
         OpenAI SDK over HTTPS actually reaches ``api.minimax.io``;
      2. the provider's typed error machinery translates the
         upstream's ``base_resp`` envelope into our ``AuthError``
         (per ADR-0002 / BLK-2 contract);
      3. the harness's lifecycle (``init / run / teardown``)
         tolerates the failure without leaking resources.

    This is **production-like** end-to-end traffic — no mocks, no
    stubs, no ``MockTransport``. The only thing we vary is the
    bearer token.
    """
    _probe_endpoint(LLM_BASE_URL)

    llm = HarnessMiniMaxProvider(
        api_key="eyJhbGciOiJIUzI1NiJ9.invalid.auth-probe-do-not-use-in-prod",
        model=LLM_DEFAULT_MODEL,
        timeout=LLM_TIMEOUT_S,
        retry=RetryPolicy(max_attempts=1),
        thinking=None,  # disable thinking policy for the wire probe
    )
    config = HarnessConfig(
        harness="echo",
        name="e2e_llm_auth_probe",
        state={"scenario": "e2e_harness_minimax_llm_auth_probe"},
    )
    harness = EchoHarness(config, llm=llm)
    user_text = "probe"

    log: dict[str, Any] = {
        "scenario": "e2e_harness_minimax_llm_auth_probe",
        "endpoint": f"{LLM_BASE_URL}/chat/completions",
        "model": LLM_DEFAULT_MODEL,
        "auth": "invalid (probe)",
    }
    started = time.monotonic()
    raised: BaseException | None = None
    try:
        await harness.init()
        try:
            await harness.run(user_text)
        except BaseException as exc:  # noqa: BLE001 — we expect one
            raised = exc
    finally:
        await harness.teardown()
    elapsed = time.monotonic() - started

    log["elapsed_s"] = round(elapsed, 3)
    log["raised_type"] = type(raised).__name__ if raised else None
    log["raised_str"] = str(raised) if raised else None

    # The upstream API responds with base_resp.status_code=1004 on a
    # bad bearer; the harness-side provider maps that to AuthError.
    # We accept ANY typed ProviderError here because the upstream
    # error format may evolve; the contract we care about is that
    # a typed exception was raised (i.e. we got through the wire
    # to a real response, not a socket hang or DNS failure).
    assert raised is not None, (
        "auth-probe scenario expected a typed exception from the upstream; "
        "got no exception. Either the upstream accepted our invalid token "
        "(it should not) or the network silently swallowed the call."
    )
    assert isinstance(raised, ProviderError), (
        f"expected typed ProviderError (auth/quota/etc); got "
        f"{type(raised).__name__}: {raised!r}. The probe proves the wire "
        f"is open but the typed-error mapping regressed."
    )

    log["status"] = "PASS"
    log_path = _write_log("e2e_harness_minimax_llm_auth_probe", log)
    print(
        f"\n[scenario 4] PASS — wire reached api.minimax.io, "
        f"typed error {type(raised).__name__} raised in {elapsed:.2f}s "
        f"→ {log_path}"
    )


# ---------------------------------------------------------------------------
# Scenario 5 — Sanity: EchoHarness with DummyLLMProvider (always runs)
# ---------------------------------------------------------------------------


@pytest.mark.integration
@pytest.mark.asyncio
async def test_e2e_harness_dummy_smoke() -> None:
    """Sanity: the framework + EchoHarness path is wired up.

    No network, no credentials. This is the production-shape sanity
    check — if this scenario breaks in CI, the harness itself is
    regressed, regardless of provider availability.
    """
    config = HarnessConfig(
        harness="echo",
        name="e2e_dummy_smoke",
        state={"scenario": "e2e_harness_dummy_smoke"},
    )
    harness = EchoHarness(config)  # uses _default_llm() = DummyLLMProvider
    try:
        await harness.init()
        result = await harness.run("sanity")
    finally:
        await harness.teardown()

    output = result.output
    log = {
        "scenario": "e2e_harness_dummy_smoke",
        "llm_type": type(harness.llm).__name__,
        "output": output,
        "status": "PASS",
    }
    assert isinstance(output, str)
    assert "sanity" in output
    log_path = _write_log("e2e_harness_dummy_smoke", log)
    print(f"\n[scenario 5] PASS — DummyLLMProvider sanity ok → {log_path}")