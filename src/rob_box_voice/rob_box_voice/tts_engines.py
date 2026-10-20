#!/usr/bin/env python3
"""tts_engines.py — единственная реализация «сырых» движков Yandex и Silero.

Раньше запрос Yandex SpeechKit v3, канал gRPC и загрузка/вызов Silero v5
жили внутри ``tts_node.py`` (ROS-нода). Синтез вокала для песни
(``sing_synth.py``, ADR-0158 PR-6) должен делать то же самое, поэтому код
вынесен сюда, а обе стороны зовут эти функции — копий нет.

Без ROS. grpc, yandex-pb2 и torch импортируются лениво внутри функций.
"""

from __future__ import annotations

import os
from typing import Any, Optional, Sequence

YANDEX_TTS_ENDPOINT = "tts.api.cloud.yandex.net:443"

#: Silero v5: ресурсный пак (bind-mount с хоста), затем legacy-volume.
SILERO_MODEL_PATHS: tuple[str, ...] = (
    "/models/silero/v5_ru.pt",
    "/cache/tts/silero_v5_ru.pt",
)


# ── Yandex SpeechKit v3 ──────────────────────────────────────────────────────


def make_yandex_stub() -> tuple[Any, Any]:
    """Канал gRPC и ``SynthesizerStub`` -> ``(channel, stub)``."""
    import grpc
    from yandex.cloud.ai.tts.v3 import tts_service_pb2_grpc

    channel = grpc.secure_channel(YANDEX_TTS_ENDPOINT, grpc.ssl_channel_credentials())
    return channel, tts_service_pb2_grpc.SynthesizerStub(channel)


def build_yandex_request(
    text: str,
    voice: str,
    speed: float,
    *,
    pitch_hz: Optional[float] = None,
    volume_lufs: Optional[float] = None,
    role: Optional[str] = None,
    raw_sample_rate: Optional[int] = None,
) -> Any:
    """``UtteranceSynthesisRequest``.

    ``raw_sample_rate=None`` -> контейнер WAV (поведение tts_node);
    число -> сырой LINEAR16 PCM с этой частотой (вокал для песни).
    """
    from yandex.cloud.ai.tts.v3 import tts_pb2

    hints = [tts_pb2.Hints(voice=voice), tts_pb2.Hints(speed=speed)]
    if role:
        hints.append(tts_pb2.Hints(role=role))
    if pitch_hz is not None:
        hints.append(tts_pb2.Hints(pitch_shift=pitch_hz))
    if volume_lufs is not None:
        hints.append(tts_pb2.Hints(volume=volume_lufs))
    if raw_sample_rate is None:
        spec = tts_pb2.AudioFormatOptions(
            container_audio=tts_pb2.ContainerAudio(
                container_audio_type=tts_pb2.ContainerAudio.WAV
            )
        )
    else:
        spec = tts_pb2.AudioFormatOptions(
            raw_audio=tts_pb2.RawAudio(
                audio_encoding=tts_pb2.RawAudio.LINEAR16_PCM,
                sample_rate_hertz=raw_sample_rate,
            )
        )
    return tts_pb2.UtteranceSynthesisRequest(
        text=text,
        output_audio_spec=spec,
        hints=hints,
        loudness_normalization_type=tts_pb2.UtteranceSynthesisRequest.LUFS,
    )


def yandex_collect_audio(stub: Any, api_key: str, request: Any) -> bytes:
    """Выполнить запрос и склеить аудио-чанки; ``grpc.RpcError`` не глотается."""
    responses = stub.UtteranceSynthesis(
        request, metadata=(("authorization", f"Api-Key {api_key}"),)
    )
    audio_data = b""
    for response in responses:
        audio_data += response.audio_chunk.data
    return audio_data


# ── Silero v5 ────────────────────────────────────────────────────────────────


def find_silero_model_path(paths: Sequence[str] = SILERO_MODEL_PATHS) -> Optional[str]:
    """Первый существующий путь к модели Silero или ``None``."""
    for path in paths:
        if os.path.exists(path):
            return path
    return None


def load_silero_package(path: str, device: Any) -> Any:
    """Silero v5 — это ``torch.package`` (не ``torch.jit``)."""
    import torch

    model = torch.package.PackageImporter(path).load_pickle("tts_models", "model")
    model.to(device)
    return model


def silero_apply_tts(
    model: Any,
    speaker: str,
    sample_rate: int,
    *,
    text: Optional[str] = None,
    ssml_text: Optional[str] = None,
    put_accent: bool = True,
    put_yo: bool = True,
    put_stress_homo: bool = True,
    put_yo_homo: bool = True,
) -> Any:
    """Вызов ``model.apply_tts`` (text или ssml_text) -> numpy-массив float32."""
    kwargs: dict[str, Any] = {
        "speaker": speaker,
        "sample_rate": sample_rate,
        "put_accent": put_accent,
        "put_yo": put_yo,
        "put_stress_homo": put_stress_homo,
        "put_yo_homo": put_yo_homo,
    }
    if ssml_text is not None:
        kwargs["ssml_text"] = ssml_text
    else:
        kwargs["text"] = text
    return model.apply_tts(**kwargs).numpy()
