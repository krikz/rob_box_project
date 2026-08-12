#!/usr/bin/env python3
"""Синтез Yandex TTS v3: text + voice -> wav (контракт как в tts_node)."""
import sys, os, grpc
from yandex.cloud.ai.tts.v3 import tts_pb2, tts_service_pb2_grpc

text = sys.argv[1]
voice = sys.argv[2] if len(sys.argv) > 2 else "anton"
out = sys.argv[3] if len(sys.argv) > 3 else "/tmp/cmd.wav"
speed = float(os.environ.get("YANDEX_SPEED", "1.0"))

key = os.environ["YANDEX_API_KEY"]
ch = grpc.secure_channel("tts.api.cloud.yandex.net:443", grpc.ssl_channel_credentials())
stub = tts_service_pb2_grpc.SynthesizerStub(ch)
req = tts_pb2.UtteranceSynthesisRequest(
    text=text,
    output_audio_spec=tts_pb2.AudioFormatOptions(
        container_audio=tts_pb2.ContainerAudio(container_audio_type=tts_pb2.ContainerAudio.WAV)
    ),
    hints=[tts_pb2.Hints(voice=voice), tts_pb2.Hints(speed=speed)],
    loudness_normalization_type=tts_pb2.UtteranceSynthesisRequest.LUFS,
)
resp = stub.UtteranceSynthesis(req, metadata=(("authorization", f"Api-Key {key}"),))
data = b""
for r in resp:
    data += r.audio_chunk.data
if not data:
    sys.exit("YANDEX_EMPTY")
with open(out, "wb") as f:
    f.write(data)
print(f"YANDEX_SYNTH_OK {len(data)} bytes voice={voice} speed={speed}")
