#!/usr/bin/env python3
"""measure_tts_voice_distinctness.py — насколько голоса TTS различимы для robot'а.

Зачем
=====
Таблица ``map_tts_voice`` в ``.github/workflows/scripts/e2e_voice_lib.sh``
утверждает, что четыре яндексовских голоса сценариев (anton / ermil /
zahar / filipp) переводятся в «гарантированно различимых» четырёх
speaker'ов у каждого провайдера. Это утверждение никогда не измерялось.

Night-marathon 22.09.2026 (run 35667281570, акт 2) показал обратное: под
minimax Саша (anton → Russian_ReliableMan) и Борис (ermil →
Russian_HandsomeChildhoodFriend) дали cos=0.846 на боевых эмбеддингах из
``/data/speakers.db`` робота — выше порога слияния профилей (0.75) и выше
порога опознания (0.72). Для resemblyzer это ОДИН человек, поэтому
проверки акта 3 (``speaker='Саш'`` / ``speaker='Борис'``) под minimax не
могут пройти в принципе.

Скрипт синтезирует по две фразы на голос, считает resemblyzer-эмбеддинги и
печатает:

* **intra-voice** — cos между двумя фразами ОДНОГО голоса. Это «потолок»:
  так выглядит один и тот же человек в идеальных условиях.
* **inter-voice** — матрицу max-cos между разными голосами (max по парам
  фраз — та же семантика, что у ``SpeakerDatabase.identify()``, которая
  берёт лучший скор по пулу эмбеддингов спикера).
* вердикт: какие пары НЕ различимы (inter >= порога слияния /
  опознания), с явной отметкой пар, которые реально играет night-marathon.

Где запускать
=============
Внутри контейнера ``voice-assistant`` на роботе — там уже есть
resemblyzer, torch, requests, yandex-grpc и ключи провайдеров::

    docker cp scripts/e2e/measure_tts_voice_distinctness.py \\
        voice-assistant:/tmp/
    docker exec voice-assistant python3 /tmp/measure_tts_voice_distinctness.py \\
        --provider minimax --json /tmp/minimax_voices.json

Можно измерить и готовые wav-ы (например, снятые с билд-машины CI, где
синтезирует Silero)::

    python3 measure_tts_voice_distinctness.py --wav-dir ./wavs

Формат ``--wav-dir``: ``<voice>__<n>.wav`` (две и более фразы на голос).

Контракт по коду синтеза
========================
Запросы к провайдерам ПОВТОРЯЮТ то, что делает e2e-харнесс
(``synth_minimax`` / ``synth_yandex`` в
``.github/workflows/scripts/e2e_voice_test.sh``) — мерить надо ровно тот
звук, который e2e играет роботу. При расхождении источник истины —
харнесс, а не этот скрипт.

Замечание о канале
==================
Скрипт меряет ФАЙЛЫ синтеза, а робот слышит их через колонку, комнату и
микрофон. Общий канал записи обычно ПОДНИМАЕТ косинус между разными
голосами (одинаковая окраска), поэтому измеренное здесь значение —
оптимистичная нижняя оценка: если пара неразличима уже в файлах, на
роботе будет не лучше. Сверка: боевая пара anton/ermil дала 0.846 на
роботе (docker exec voice-assistant, /data/speakers.db, 22.09.2026).
"""

from __future__ import annotations

import argparse
import itertools
import json
import os
import re
import sys
import tempfile
import time
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np

# Пороги — копия констант из
# src/rob_box_voice/rob_box_voice/utils/speaker_embeddings.py. Дублируются
# намеренно: скрипт запускается внутри контейнера по одному файлу, без
# доступа к дереву исходников.
IDENTIFY_THRESHOLD = 0.72
REGISTER_MATCH_THRESHOLD = 0.75

# Две РАЗНЫЕ фразы на голос: intra-voice cos на одном и том же тексте
# был бы завышен (совпадает и фонетика), а нам нужен «тот же человек,
# другая реплика» — ровно то, что происходит в акте 2 (n201 → n202).
PHRASES = (
    "Робот, привет, давай знакомиться как следует. Я твой ночной инженер, "
    "я собирал тебе блок питания и переделывал левый мотор.",
    "Робот, слушай дальше и запоминай, как я звучу. Вчера я всю ночь паял "
    "этот несчастный блок питания, и он всё равно гудит как трансформатор.",
)

# Голоса провайдера. Первые четыре в каждом списке — те, которыми
# night-marathon играет Сашу / Бориса / дядю Гришу / четвёртого
# собеседника (порядок соответствует anton / ermil / zahar / filipp в
# map_tts_voice).
SCENARIO_VOICES = ("anton", "ermil", "zahar", "filipp")

PROVIDER_VOICES: Dict[str, List[str]] = {
    "yandex": ["anton", "ermil", "zahar", "filipp", "alena", "jane", "arina", "omazh"],
    "silero": ["aidar", "eugene", "baya", "xenia", "kseniya"],
    "minimax": [
        "Russian_ReliableMan",             # anton
        "Russian_HandsomeChildhoodFriend",  # ermil / madirus
        "Russian_Bad-temperedBoy",         # zahar
        "Russian_AttractiveGuy",           # filipp / kostya
        "Russian_BrightHeroine",           # alena
        "Russian_AmbitiousWoman",          # jane
        "Russian_PessimisticGirl",         # arina
        "Russian_CrazyQueen",              # omazh / rush
    ],
}

# Обратная карта «голос провайдера → голос сценария» ровно по таблице
# map_tts_voice: нужна, чтобы в отчёте было видно, какая пара реально
# звучит в актах 2-3, а какая — просто соседняя запись каталога.
SCENARIO_OF: Dict[str, Dict[str, str]] = {
    "yandex": {v: v for v in PROVIDER_VOICES["yandex"]},
    "silero": {
        "aidar": "anton",
        "eugene": "ermil",
        "baya": "zahar",
        "xenia": "filipp",
        "kseniya": "alena",
    },
    "minimax": {
        "Russian_ReliableMan": "anton",
        "Russian_HandsomeChildhoodFriend": "ermil",
        "Russian_Bad-temperedBoy": "zahar",
        "Russian_AttractiveGuy": "filipp",
        "Russian_BrightHeroine": "alena",
        "Russian_AmbitiousWoman": "jane",
        "Russian_PessimisticGirl": "arina",
        "Russian_CrazyQueen": "omazh",
    },
}


# ── Синтез ──────────────────────────────────────────────────────────────────


def synth_minimax(
    text: str, voice: str, out_wav: Path, pitch: int = 0, speed: float = 1.0
) -> None:
    """POST /v1/t2a_v2, hex-аудио в ``data.audio`` (как synth_minimax в e2e).

    ``pitch`` (полутона, -12..12) и ``speed`` (0.5..2.0) — штатные поля
    ``voice_setting`` T2A v2. Харнесс сегодня шлёт только speed
    (из ``YANDEX_SPEED``), pitch не трогает.
    """
    import requests

    key = os.environ.get("MINIMAX_API_KEY", "")
    if not key:
        raise RuntimeError("MINIMAX_API_KEY не задан")
    base = os.environ.get("MINIMAX_TTS_BASE_URL", "https://api.minimax.io").rstrip("/")
    payload = {
        "model": os.environ.get("MINIMAX_TTS_MODEL", "speech-02-hd"),
        "text": text,
        "stream": False,
        "voice_setting": {"voice_id": voice, "speed": speed, "pitch": pitch},
        "audio_setting": {
            "sample_rate": 32000,
            "bitrate": 128000,
            "format": "wav",
            "channel": 1,
        },
    }
    resp = requests.post(
        base + "/v1/t2a_v2",
        headers={"Authorization": "Bearer " + key, "Content-Type": "application/json"},
        json=payload,
        timeout=90,
    )
    body = resp.json()
    base_resp = body.get("base_resp") or {}
    status = int(base_resp.get("status_code", 0) or 0)
    if resp.status_code >= 400 or status != 0:
        msg = base_resp.get("status_msg") or resp.text[:200]
        raise RuntimeError(f"minimax HTTP {resp.status_code} status={status}: {msg}")
    audio_hex = (body.get("data") or {}).get("audio") or ""
    if not audio_hex:
        raise RuntimeError("minimax ответил без data.audio")
    out_wav.write_bytes(bytes.fromhex(audio_hex))


def synth_yandex(
    text: str, voice: str, out_wav: Path, pitch: int = 0, speed: float = 1.0
) -> None:
    """SpeechKit v3 gRPC (как synth_yandex в e2e): LINEAR16_PCM 16 kHz → wav.

    У SpeechKit v3 питч задаётся не полем запроса, а SSML/ролями голоса,
    поэтому ``pitch`` здесь не поддержан — передача ненулевого значения
    это явная ошибка, а не тихая потеря настройки.
    """
    if pitch:
        raise RuntimeError("yandex: pitch не поддержан этим скриптом")
    import grpc
    from yandex.cloud.ai.tts.v3 import tts_pb2, tts_service_pb2_grpc

    key = os.environ.get("YANDEX_API_KEY", "")
    if not key:
        raise RuntimeError("YANDEX_API_KEY не задан")
    request = tts_pb2.UtteranceSynthesisRequest(
        text=text,
        output_audio_spec=tts_pb2.AudioFormatOptions(
            container_audio=tts_pb2.ContainerAudio(
                container_audio_type=tts_pb2.ContainerAudio.WAV
            )
        ),
        hints=[tts_pb2.Hints(voice=voice)],
        loudness_normalization_type=tts_pb2.UtteranceSynthesisRequest.LUFS,
    )
    channel = grpc.secure_channel(
        "tts.api.cloud.yandex.net:443", grpc.ssl_channel_credentials()
    )
    try:
        stub = tts_service_pb2_grpc.SynthesizerStub(channel)
        chunks = [
            piece.audio_chunk.data
            for piece in stub.UtteranceSynthesis(
                request, metadata=(("authorization", "Api-Key " + key),), timeout=90
            )
        ]
    finally:
        channel.close()
    if not chunks:
        raise RuntimeError("yandex вернул пустой поток аудио")
    out_wav.write_bytes(b"".join(chunks))


_SILERO_PITCH = {
    -2: "x-low",
    -1: "low",
    0: "medium",
    1: "high",
    2: "x-high",
}


def synth_silero(
    text: str, voice: str, out_wav: Path, pitch: int = 0, speed: float = 1.0
) -> None:
    """Локальный Silero v5 (как synth_silero в e2e): torch.package → wav.

    ``pitch`` у Silero — не полутона, а пять ступеней SSML prosody
    (-2..+2 → x-low..x-high), ``speed`` — ``rate`` в процентах.
    """
    import wave

    import torch

    torch.set_grad_enabled(False)
    torch.set_num_threads(int(os.environ.get("E2E_SILERO_THREADS", "4")))
    home = os.path.expanduser("~")
    candidates = [
        os.environ.get("E2E_SILERO_MODEL") or "",
        "/models/silero/v5_ru.pt",
        "/cache/tts/silero_v5_ru.pt",
        home + "/.cache/rob_box_voice/tts_models/v5_ru.pt",
    ]
    model = None
    for path in [c for c in candidates if c and os.path.exists(c)]:
        model = torch.package.PackageImporter(path).load_pickle("tts_models", "model")
        break
    if model is None:
        raise RuntimeError(f"нет локальной модели Silero: пробовал {candidates}")
    model.to(torch.device("cpu"))
    rate = int(os.environ.get("E2E_SILERO_SAMPLE_RATE", "48000"))
    if pitch not in _SILERO_PITCH:
        raise RuntimeError(f"silero: pitch должен быть в {sorted(_SILERO_PITCH)}")
    prosody = (
        f'<prosody pitch="{_SILERO_PITCH[pitch]}" rate="{int(round(speed * 100))}%">'
    )
    audio = model.apply_tts(
        ssml_text="<speak>" + prosody + text + "</prosody></speak>",
        speaker=voice,
        sample_rate=rate,
        put_accent=True,
        put_yo=True,
    )
    pcm = (np.clip(audio.numpy(), -1.0, 1.0) * 32767).astype(np.int16)
    with wave.open(str(out_wav), "wb") as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(rate)
        handle.writeframes(pcm.tobytes())


SYNTHS = {
    "minimax": synth_minimax,
    "silero": synth_silero,
    "yandex": synth_yandex,
}


# ── Эмбеддинги ──────────────────────────────────────────────────────────────


def embed_files(files: Sequence[Path]) -> List[np.ndarray]:
    """resemblyzer d-vector на каждый wav (тот же энкодер, что у робота)."""
    from resemblyzer import VoiceEncoder, preprocess_wav

    encoder = VoiceEncoder(device="cpu")
    out = []
    for path in files:
        wav = preprocess_wav(str(path))
        emb = encoder.embed_utterance(wav).astype(np.float32)
        out.append(emb / (np.linalg.norm(emb) + 1e-9))
    return out


def cos(a: np.ndarray, b: np.ndarray) -> float:
    return float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9))


# ── Отчёт ───────────────────────────────────────────────────────────────────


def build_report(
    provider: str, embeddings: Dict[str, List[np.ndarray]]
) -> Dict[str, object]:
    voices = list(embeddings)
    scenario_of = SCENARIO_OF.get(provider, {})

    intra = {}
    for voice, embs in embeddings.items():
        pairs = [cos(a, b) for a, b in itertools.combinations(embs, 2)]
        intra[voice] = min(pairs) if pairs else None

    inter = {}
    for left, right in itertools.combinations(voices, 2):
        best = max(cos(a, b) for a in embeddings[left] for b in embeddings[right])
        inter[f"{left}|{right}"] = best

    scenario_pairs = []
    for left, right in itertools.combinations(voices, 2):
        # У варианта голоса ярлык вида voice[p-6,s1] — в таблицу
        # сценария смотрим по базовому голосу.
        s_left = scenario_of.get(left.split('[')[0])
        s_right = scenario_of.get(right.split('[')[0])
        if s_left in SCENARIO_VOICES and s_right in SCENARIO_VOICES:
            scenario_pairs.append(
                {
                    "scenario_voices": [s_left, s_right],
                    "provider_voices": [left, right],
                    "cos": inter[f"{left}|{right}"],
                    "merges_on_register": inter[f"{left}|{right}"]
                    >= REGISTER_MATCH_THRESHOLD,
                    "identified_as_same": inter[f"{left}|{right}"]
                    >= IDENTIFY_THRESHOLD,
                }
            )

    return {
        "provider": provider,
        "identify_threshold": IDENTIFY_THRESHOLD,
        "register_match_threshold": REGISTER_MATCH_THRESHOLD,
        "voices": voices,
        "intra_voice_min_cos": intra,
        "inter_voice_max_cos": inter,
        "scenario_pairs": scenario_pairs,
    }


def print_report(report: Dict[str, object]) -> None:
    voices = list(report["voices"])  # type: ignore[arg-type]
    intra = report["intra_voice_min_cos"]  # type: ignore[assignment]
    inter = report["inter_voice_max_cos"]  # type: ignore[assignment]
    width = max(len(v) for v in voices) + 1

    print(f"\n=== provider={report['provider']} ===")
    print(
        f"пороги: identify={report['identify_threshold']} "
        f"register_match={report['register_match_threshold']}"
    )

    print("\n-- intra-voice (одна и та же «личность», две разные фразы) --")
    for voice in voices:
        value = intra[voice]  # type: ignore[index]
        print(f"  {voice:<{width}} {value:.4f}" if value is not None else f"  {voice}  n/a")

    print("\n-- inter-voice max-cos --")
    print(" " * width + "".join(f"{v[:10]:>11}" for v in voices))
    for left in voices:
        row = [f"{left:<{width}}"]
        for right in voices:
            if left == right:
                row.append(f"{'—':>11}")
                continue
            key = f"{left}|{right}" if f"{left}|{right}" in inter else f"{right}|{left}"
            row.append(f"{inter[key]:>11.4f}")  # type: ignore[index]
        print("".join(row))

    collisions = [
        (key, value)
        for key, value in inter.items()  # type: ignore[union-attr]
        if value >= report["identify_threshold"]
    ]
    print("\n-- пары, которые робот считает одним человеком --")
    if not collisions:
        print("  (нет — все голоса различимы)")
    for key, value in sorted(collisions, key=lambda kv: -kv[1]):
        mark = (
            "СЛИВАЕТ ПРОФИЛИ"
            if value >= report["register_match_threshold"]
            else "путает при опознании"
        )
        print(f"  {value:.4f}  {key.replace('|', '  vs  ')}   <-- {mark}")

    print("\n-- пары, которые реально играет night-marathon --")
    for pair in report["scenario_pairs"]:  # type: ignore[union-attr]
        verdict = "НЕРАЗЛИЧИМЫ" if pair["identified_as_same"] else "ок"
        print(
            f"  {pair['cos']:.4f}  {'/'.join(pair['scenario_voices'])}"
            f"  ({' + '.join(pair['provider_voices'])})  <-- {verdict}"
        )


# ── main ────────────────────────────────────────────────────────────────────


def collect_wav_dir(wav_dir: Path) -> Dict[str, List[Path]]:
    files: Dict[str, List[Path]] = {}
    for path in sorted(wav_dir.glob("*.wav")):
        voice = re.sub(r"__\d+$", "", path.stem)
        files.setdefault(voice, []).append(path)
    missing = [v for v, f in files.items() if len(f) < 2]
    if missing:
        print(
            f"WARN: у голосов {missing} меньше двух фраз — intra-voice не посчитается",
            file=sys.stderr,
        )
    return files


def parse_voice_spec(spec: str) -> Tuple[str, int, float]:
    """``voice[:pitch[:speed]]`` → ``(voice, pitch, speed)``.

    Вариант голоса — это «тот же диктор, но всегда с этим питчем и
    темпом». Настройка ФИКСИРОВАНА за говорящим на весь сценарий,
    поэтому intra-voice от неё не страдает, а inter-voice может упасть:
    ровно то, что нужно проверить, прежде чем чинить таблицу голосов
    подкруткой просодии вместо смены голоса.
    """
    parts = spec.split(":")
    voice = parts[0]
    pitch = int(parts[1]) if len(parts) > 1 and parts[1] != "" else 0
    speed = float(parts[2]) if len(parts) > 2 and parts[2] != "" else 1.0
    return voice, pitch, speed


def label_of(voice: str, pitch: int, speed: float) -> str:
    if pitch == 0 and speed == 1.0:
        return voice
    return f"{voice}[p{pitch:+d},s{speed:g}]"


def synthesize(
    provider: str, specs: Sequence[str], out_dir: Path, phrases: Sequence[str]
) -> Dict[str, List[Path]]:
    synth = SYNTHS[provider]
    out_dir.mkdir(parents=True, exist_ok=True)
    files: Dict[str, List[Path]] = {}
    for spec in specs:
        voice, pitch, speed = parse_voice_spec(spec)
        label = label_of(voice, pitch, speed)
        safe = re.sub(r"[^A-Za-z0-9_.+-]", "_", label)
        for index, phrase in enumerate(phrases, start=1):
            target = out_dir / f"{safe}__{index}.wav"
            if target.exists() and target.stat().st_size > 1024:
                print(f"  {label} #{index}: уже синтезирован, пропускаю")
            else:
                started = time.monotonic()
                synth(phrase, voice, target, pitch=pitch, speed=speed)
                print(
                    f"  {label} #{index}: {target.stat().st_size} байт "
                    f"за {time.monotonic() - started:.1f}s"
                )
            files.setdefault(label, []).append(target)
    return files


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--provider", choices=sorted(SYNTHS), help="кого синтезировать")
    parser.add_argument(
        "--voices",
        help=(
            "список голосов через запятую; каждый — voice[:pitch[:speed]] "
            "(например Russian_ReliableMan:-6:1.0). По умолчанию — каталог провайдера"
        ),
    )
    parser.add_argument("--wav-dir", type=Path, help="мерить готовые wav, не синтезировать")
    parser.add_argument("--out-dir", type=Path, help="куда класть синтез (по умолчанию tmp)")
    parser.add_argument("--json", type=Path, help="записать отчёт в JSON")
    parser.add_argument(
        "--fail-on-collision",
        action="store_true",
        help="ненулевой код возврата, если пара голосов сценария неразличима",
    )
    args = parser.parse_args(argv)

    if args.wav_dir:
        provider = args.provider or "wav-dir"
        files = collect_wav_dir(args.wav_dir)
    else:
        if not args.provider:
            parser.error("нужен --provider или --wav-dir")
        provider = args.provider
        voices = (
            [v.strip() for v in args.voices.split(",") if v.strip()]
            if args.voices
            else PROVIDER_VOICES[provider]
        )
        out_dir = args.out_dir or Path(tempfile.mkdtemp(prefix=f"tts_{provider}_"))
        print(f"Синтезирую {len(voices)} голосов × {len(PHRASES)} фраз в {out_dir}")
        files = synthesize(provider, voices, out_dir, PHRASES)

    if not files:
        print("нечего мерить: ни одного wav", file=sys.stderr)
        return 2

    print("\nСчитаю resemblyzer-эмбеддинги...")
    embeddings = {voice: embed_files(paths) for voice, paths in files.items()}

    report = build_report(provider, embeddings)
    print_report(report)

    if args.json:
        args.json.write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print(f"\nJSON: {args.json}")

    broken = [p for p in report["scenario_pairs"] if p["identified_as_same"]]
    if broken and args.fail_on_collision:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
