#!/usr/bin/env python3
"""Synthesize a short phrase with MiniMax TTS and save it as PCM/WAV."""

from __future__ import annotations

import argparse
import asyncio
import os
import wave
from pathlib import Path
from typing import Protocol, cast

from rob_box_llm.tts_provider_registry import (
    TTSProviderFactory,
    register_builtin_tts_providers,
)


class _MiniMaxBytesProvider(Protocol):
    async def synthesize_bytes(
        self, text: str, voice: str | None = None, **opts: object
    ) -> bytes: ...

    async def aclose(self) -> None: ...


def _env_int(name: str, default: int) -> int:
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        value = int(raw)
    except ValueError as exc:
        raise SystemExit(f"{name} must be an integer") from exc
    if value < 1:
        raise SystemExit(f"{name} must be at least 1")
    return value


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Synthesize speech with the registered MiniMax TTS provider."
    )
    parser.add_argument(
        "--text",
        default="Привет! Это MiniMax TTS.",
        help="short phrase to synthesize",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("minimax_tts.wav"),
        help="destination WAV path (default: minimax_tts.wav)",
    )
    return parser.parse_args()


async def _run(args: argparse.Namespace) -> None:
    config: dict[str, object] = {
        "api_key": os.environ["MINIMAX_API_KEY"],
        "group_id": os.environ["MINIMAX_GROUP_ID"],
        "base_url": os.getenv("MINIMAX_TTS_BASE_URL", "https://api.minimax.io"),
        "default_voice": os.getenv("MINIMAX_TTS_VOICE", "Russian_CalmWoman"),
        "max_concurrency": _env_int("MINIMAX_TTS_MAX_CONCURRENCY", 1),
    }

    registry = register_builtin_tts_providers()
    provider = cast(
        _MiniMaxBytesProvider,
        TTSProviderFactory.create("minimax", config, registry),
    )
    try:
        pcm = await provider.synthesize_bytes(args.text, format="pcm_24000")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with wave.open(str(args.output), "wb") as writer:
            writer.setnchannels(1)
            writer.setsampwidth(2)  # signed 16-bit little-endian PCM
            writer.setframerate(24_000)
            writer.writeframes(pcm)
    finally:
        await provider.aclose()
        TTSProviderFactory.reset_cache()

    print(f"Wrote {args.output} ({len(pcm)} PCM bytes, 24000 Hz, mono, 16-bit)")


def main() -> None:
    args = _parse_args()
    missing = [
        name for name in ("MINIMAX_API_KEY", "MINIMAX_GROUP_ID") if not os.getenv(name)
    ]
    if missing:
        raise SystemExit(
            f"Missing required environment variable(s): {', '.join(missing)}"
        )
    asyncio.run(_run(args))


if __name__ == "__main__":
    main()
