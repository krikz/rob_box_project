#!/usr/bin/env python3
"""loudness_nrt_v2.py — громкость и полосы слоя в scsynth NRT 16 кГц (ADR-0152 §3.1, issue #3422).

Харнесс v2 вместо удалённого ``club_loudness_nrt.py`` (#3391): программа Renardo → события
``rob_box_music.render.events.program_events`` (семантика ``Players.py`` — в одном месте, без копии v1) →
OSC-партитура (``/s_new`` с таймстампами, бандл ноты как у ``ServerManager.get_bundle`` renardo 0.9.13:
``startSound`` → синт → эффекты → ``makeSound``) → ``scsynth -N`` 16 кГц с SynthDef-ами renardo_lib,
``custom_synthdefs/`` и ``masterfilter.scd`` (``gain 0.5, dyn 0`` — шкала ``knowledge.LOUDNESS_SOURCE``) → WAV →
dB RMS и доли энергии < 150 / 150–2000 / > 2000 Гц (``knowledge.LAYER_BANDS_HZ``). Один слой — один рендер.

Режимы:

* ``--regress`` — рамка 29.09 (``loudness_frame_0929.json``: программы ``--sweep`` старого харнесса, seed 0,
  ``dj_dave_32``, 124 BPM, 5 тоник) против ``knowledge.LAYER_MEASURED_DB``, допуск ±1.5 дБ.
* ``--sweep РОЛЬ СИНТ…`` — та же рамка роли с подменой синта: dB при уровне замера и при половине (наклон
  ``amp``), полосы. Новые строки ``LAYER_MEASURED_DB``/``LAYER_BANDS`` — в той же шкале, что старые.
* ``--track SEED`` — трек v2 (``arrange.compose.club_track``) → ``render`` → каждый слой отдельно: dB RMS по
  секциям, где роль звучит, против ``Part.level_db`` модели, и полосы. Слои ``loop()`` (DJ_Dave) пропускаются.

Это НЕ замер на роботе: ReSpeaker/ALSA, ``Clock.latency`` и джиттер не моделируются. Где запускать —
``scripts/music/loudness_nrt_v2.md`` (katana, одноразовый контейнер образа ``voice-assistant``).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import struct
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple
from urllib.request import urlopen

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src" / "rob_box_music"))

from rob_box_music import knowledge as kn  # noqa: E402
from rob_box_music.model import SAMPLE_ROLES  # noqa: E402
from rob_box_music.render.events import NoteEvent, program_events  # noqa: E402

SAMPLE_RATE = 16000
BUS_BASE, BUS_POOL = 16, 900
MASTER_NODE = 999
#: Мастер замера моделей (``LOUDNESS_SOURCE``): фейдер 0.5, без радио-динамики — шкала аддитивна.
MASTER = {"gain": 0.5, "dyn": 0.0}
#: Сдвиг нот от начала партитуры: SynthDef-ы и буферы грузятся в бандле 0.
NOTE_OFFSET_S = 0.01
TAIL_S = 2.0
FRAME_FILE = Path(__file__).with_name("loudness_frame_0929.json")
CUSTOM_DEFS = REPO / "docker/vision/voice_assistant/custom_synthdefs"
TOLERANCE_DB = 1.5
#: ``LAYER_MEASURED_DB`` проверяемых вариантов регресса: (роль, вариант).
REGRESS = (("pad", "sinepad"), ("lead", "pluck"), ("bass", "bass"), ("kick", "four_on_floor"))
EFFECT_DEFS = ("startSound", "makeSound", "highPassFilter", "lowPassFilter", "combDelay", "reverb")
SAMPLES_SERVER = "https://collections.renardo.org/samples/0_foxdot_default"
SYMBOL_DIRS = {"-": "hyphen", "*": "asterix", "~": "tilde", "+": "plus", "=": "equals", "&": "ampersand",
               "@": "at", ":": "colon", "#": "hash", "%": "percent", "!": "exclamation", "/": "forwardslash"}

Message = Tuple[Any, ...]
Bundle = Tuple[float, List[Message]]


# ── Партитура ──────────────────────────────────────────────────────────────────────────────────────────────


def midi_hz(midi: float) -> float:
    return 440.0 * 2.0 ** ((midi - 69.0) / 12.0)


def _effects(ev: NoteEvent, beat_dur: float) -> List[Tuple[str, List[Any]]]:
    """Эффекты ноты в порядке renardo (order 2): hpf, lpf, echo, room — только заданные (не 0)."""
    fx = ev.fx
    chain = (("highPassFilter", "hpf", ["hpr", 1.0]), ("lowPassFilter", "lpf", ["lpr", 1.0]),
             ("combDelay", "echo", ["beat_dur", beat_dur, "echotime", 1.0]),
             ("reverb", "room", ["mix", float(fx.get("mix", 0.1))]))
    return [(name, [key, float(fx[key])] + extra) for name, key, extra in chain if fx.get(key)]


def note_messages(ev: NoteEvent, k: int, node: int, beat_dur: float,
                  buffers: Mapping[str, Tuple[int, int]]) -> List[Message]:
    """Бандл одной ноты (``ServerManager.get_bundle``): группа, ``startSound`` (``rate`` = частота синта или
    скорость сэмпла), синт, эффекты, ``makeSound``. ``sus`` синта — секунды, ``startSound`` держит ``sus·8``."""
    bus, group, sus = BUS_BASE + k % BUS_POOL, node, ev.sus_beats * beat_dur
    if ev.sample is not None:
        buf, channels = buffers[ev.sample]
        rate, synth, extra = 1.0, "play2" if channels == 2 else "play1", ["buf", buf]
    else:
        rate = midi_hz(float(ev.midi) + ev.detune)
        synth, extra = ev.synth, ["rate", 0.0, "freq", rate]
    msgs: List[Message] = [("/g_new", group, 1, 1),
                           ("/s_new", "startSound", node + 1, 0, group, "bus", bus, "sus", sus * 8, "rate", rate),
                           ("/s_new", synth, node + 2, 1, group, "bus", bus, "amp", float(ev.amp), "sus", sus,
                            "pan", float(ev.pan), "fmod", 0.0, "blur", 1.0, "beat_dur", beat_dur, *extra)]
    node += 3
    for name, args in _effects(ev, beat_dur):
        msgs.append(("/s_new", name, node, 1, group, "bus", bus, *args))
        node += 1
    msgs.append(("/s_new", "makeSound", node, 1, group, "bus", bus, "sus", sus))
    return msgs


def score(events: Sequence[NoteEvent], bpm: float, def_dir: str, buffers: Mapping[str, Tuple[int, int]],
          paths: Mapping[str, str], duration: float, master: Mapping[str, float] = MASTER) -> List[Bundle]:
    """События → бандлы партитуры NRT. Детерминирована: порядок — (доля, слот, синт, высота), узлы подряд."""
    beat_dur = 60.0 / bpm
    head: List[Message] = [("/d_loadDir", def_dir)]
    head += [("/b_allocRead", buffers[key][0], paths[key]) for key in sorted(buffers)]
    out: List[Bundle] = [(0.0, head), (0.001, [("/g_new", 1, 0, 0)]),
                         (0.002, [("/s_new", "masterfilter", MASTER_NODE, 1, 0,
                                   *[a for k, v in sorted(master.items()) for a in (k, float(v))])])]
    order = sorted(events, key=lambda e: (e.beat, e.slot, e.synth, e.midi or 0.0, e.pan))
    node = 2000
    for k, ev in enumerate(order):
        msgs = note_messages(ev, k, node, beat_dur, buffers)
        node += len(msgs) + 1
        out.append((round(ev.beat * beat_dur + NOTE_OFFSET_S, 6), msgs))
    out.append((duration, [("/c_set", 0, 0.0)]))
    return out


def _osc_str(value: str) -> bytes:
    raw = value.encode("utf-8") + b"\0"
    return raw + b"\0" * (-len(raw) % 4)


def osc_message(address: str, *args: Any) -> bytes:
    tags, body = ",", b""
    for arg in args:
        if isinstance(arg, int) and not isinstance(arg, bool):
            tags, body = tags + "i", body + struct.pack(">i", arg)
        elif isinstance(arg, float):
            tags, body = tags + "f", body + struct.pack(">f", arg)
        else:
            tags, body = tags + "s", body + _osc_str(str(arg))
    return _osc_str(address) + _osc_str(tags) + body


def encode_score(bundles: Iterable[Bundle]) -> bytes:
    """Бинарный Score для ``scsynth -N``: бандл с NTP-временем от нуля, перед каждым — длина."""
    out = b""
    for time, msgs in bundles:
        seconds = int(time)
        body = b"#bundle\0" + struct.pack(">II", seconds, int((time - seconds) * (1 << 32)))
        for msg in msgs:
            raw = osc_message(*msg)
            body += struct.pack(">i", len(raw)) + raw
        out += struct.pack(">i", len(body)) + body
    return out


# ── Замер ──────────────────────────────────────────────────────────────────────────────────────────────────


def rms_db(x) -> float:
    import numpy as np

    value = float(np.sqrt(np.mean(np.square(np.asarray(x, dtype="f8"))))) if len(x) else 0.0
    return round(20.0 * math.log10(value), 2) if value > 0 else -200.0


def band_shares(x, sr: int = SAMPLE_RATE, edges: Tuple[float, float] = kn.LAYER_BANDS_HZ) -> Tuple[float, ...]:
    """Доли энергии сигнала ниже ``edges[0]``, между, выше ``edges[1]`` Гц (спектр мощности всего сигнала;
    границы — как ``live_dj/compare.profile``). Сумма — 1.0; тишина — ``ValueError``."""
    import numpy as np

    power = np.abs(np.fft.rfft(np.asarray(x, dtype="f8"))) ** 2
    freqs = np.fft.rfftfreq(len(x), 1.0 / sr)
    total = float(power.sum())
    if total <= 0:
        raise ValueError("тишина: полос нет")
    low = float(power[freqs < edges[0]].sum()) / total
    mid = float(power[(freqs >= edges[0]) & (freqs < edges[1])].sum()) / total
    return round(low, 3), round(mid, 3), round(1.0 - round(low, 3) - round(mid, 3), 3)


def read_wav(path: Path):
    """Левый канал WAV (float32 или int16) — моно-слой в центре, как ``LOUDNESS_SOURCE``."""
    return _wav_frames(path)[:, 0]


def _wav_frames(path: Path):
    """Кадры WAV (float32 или int16) — массив (кадры × каналы)."""
    import numpy as np

    raw = path.read_bytes()
    chunks: Dict[bytes, bytes] = {}
    pos = 12
    while pos + 8 <= len(raw):
        cid, size = raw[pos:pos + 4], struct.unpack("<I", raw[pos + 4:pos + 8])[0]
        chunks.setdefault(cid, raw[pos + 8:pos + 8 + size])
        pos += 8 + size + (size & 1)
    fmt_tag, channels = struct.unpack("<HH", chunks[b"fmt "][:4])
    bits = struct.unpack("<H", chunks[b"fmt "][14:16])[0]
    dtype = "<f4" if fmt_tag == 3 or bits == 32 else "<i2"
    data = np.frombuffer(chunks[b"data"], dtype=dtype).astype("f8") / (1.0 if dtype == "<f4" else 32768.0)
    return data.reshape(-1, channels)


def read_wav_mono(path: Path):
    """Моно-сумма каналов WAV ((L + R) / 2): так тему слышит слушатель у двух близких колонок и запись в Telegram."""
    return _wav_frames(path).mean(axis=1)


# ── Читаемость темы (#тема-лид, 07.10): атака, хвост, чистота высоты, яркость ────────────────────────────────
#: Огибающая — RMS окнами 5 мс; хвост — до ``peak − CLARITY_TAIL_DB``; атака — до ``peak − 3 дБ``.
CLARITY_FRAME_S = 0.005
CLARITY_TAIL_DB = 30.0
#: Окно чистоты высоты вокруг гармоник k·f0, центы: голос, расстроенный дальше (хорус supersaw ±26 ц, hoover ±60 ц,
#: стохастический ``rave``), уносит энергию из окна.
PURITY_CENTS = 15.0
#: Полоса «яркости» темы на выходе 16 кГц: 1–4 кГц (присутствие; ниже — середина пэда и баса).
BRIGHT_BAND_HZ = (1000.0, 4000.0)


def envelope_db(x, sr: int = SAMPLE_RATE, frame_s: float = CLARITY_FRAME_S):
    """dB RMS по окнам ``frame_s`` (тишина — −200)."""
    import numpy as np

    n = max(1, int(round(frame_s * sr)))
    frames = np.asarray(x, dtype="f8")[: len(x) // n * n].reshape(-1, n)
    rms = np.sqrt(np.mean(frames ** 2, axis=1))
    return np.where(rms > 0, 20.0 * np.log10(np.maximum(rms, 1e-20)), -200.0)


def note_shape(x, onset_s: float, sus_s: float, span_s: float, sr: int = SAMPLE_RATE) -> Dict[str, float]:
    """Атака (мс от начала ноты до ``пик − 3 дБ``) и хвост (мс от конца ``sus`` до ``пик − 30 дБ``) одной ноты,
    звучащей одна в окне ``[onset, onset + span)``."""
    import numpy as np

    env = envelope_db(x[int(onset_s * sr):int((onset_s + span_s) * sr)], sr)
    peak_i = int(np.argmax(env))
    peak = float(env[peak_i])
    attack_i = int(np.argmax(env >= peak - 3.0))
    below = np.nonzero(env[peak_i:] < peak - CLARITY_TAIL_DB)[0]
    end_i = peak_i + int(below[0]) if len(below) else len(env)
    frame_ms = CLARITY_FRAME_S * 1000.0
    return {"attack_ms": round(attack_i * frame_ms, 1),
            "tail_ms": round(max(0.0, end_i * frame_ms - sus_s * 1000.0), 1)}


def pitch_purity(x, f0: float, sr: int = SAMPLE_RATE, cents: float = PURITY_CENTS) -> float:
    """Доля энергии в окнах ±``cents`` вокруг сетки ``k·f0/2`` (до Найквиста) — 1.0 у чистого тона; хорус и
    шумовой синт размазывают её между линиями сетки. Сетка от ``f0/2``: синты, звучащие октавой ниже или с ЧМ на
    ``f0/2`` (``arpy`` — ``Impulse(freq/2)``, ``keys`` — ``LFPar(freq/2)``, суб-голос ``hoover``), остаются чистыми.
    Спектр — Ханн с дополнением нулями до 1 Гц."""
    import numpy as np

    sig = np.asarray(x, dtype="f8") * np.hanning(len(x))
    n = max(len(sig), sr)
    power = np.abs(np.fft.rfft(sig, n)) ** 2
    freqs = np.fft.rfftfreq(n, 1.0 / sr)
    total = float(power[freqs >= 40.0].sum())
    if total <= 0:
        raise ValueError("тишина: чистоты нет")
    ratio, base = 2.0 ** (cents / 1200.0), f0 / 2.0
    mask = np.zeros(len(freqs), dtype=bool)
    k = 1
    while k * base / ratio < sr / 2:
        mask |= (freqs >= k * base / ratio) & (freqs <= k * base * ratio)
        k += 1
    return round(float(power[mask & (freqs >= 40.0)].sum()) / total, 3)


def brightness(x, sr: int = SAMPLE_RATE, band: Tuple[float, float] = BRIGHT_BAND_HZ) -> Dict[str, float]:
    """Спектральный центроид (Гц) и доля энергии ``band`` (1–4 кГц)."""
    import numpy as np

    power = np.abs(np.fft.rfft(np.asarray(x, dtype="f8"))) ** 2
    freqs = np.fft.rfftfreq(len(x), 1.0 / sr)
    total = float(power.sum())
    if total <= 0:
        raise ValueError("тишина: яркости нет")
    share = float(power[(freqs >= band[0]) & (freqs < band[1])].sum()) / total
    return {"centroid_hz": round(float((power * freqs).sum()) / total), "bright_share": round(share, 3)}


# ── scsynth / sclang ───────────────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Rig:
    """Где SuperCollider, SynthDef-ы и сэмплы; ``out`` — каталог рендеров."""

    renardo: Path
    samples: Path
    out: Path
    sclang: str = "sclang"
    scsynth: str = "scsynth"
    timeout: float = 900.0

    @property
    def def_dir(self) -> Path:
        return self.out / "defs"


def def_source(rig: Rig, name: str) -> Path:
    """``.scd`` синта: renardo_lib (как ``SynthDefManagement``), иначе ``custom_synthdefs`` репо."""
    sc = rig.renardo / "SynthDefManagement" / "sclang_code"
    for path in (sc / "scsynth" / f"{name}.scd", sc / "sceffects" / f"{name}.scd", CUSTOM_DEFS / f"{name}.scd"):
        if path.exists():
            return path
    raise SystemExit(f"SynthDef {name!r} не найден ни в renardo_lib, ни в custom_synthdefs")


def compile_defs(rig: Rig, synths: Iterable[str]) -> None:
    """Скомпилировать недостающие ``.scsyndef`` (``.add`` → ``.writeDefFile``) одним прогоном sclang."""
    names = sorted(set(synths) | {"play1", "play2", "masterfilter", *EFFECT_DEFS})
    rig.def_dir.mkdir(parents=True, exist_ok=True)
    todo = [n for n in names if not (rig.def_dir / f"{n}.scsyndef").exists()]
    if not todo:
        return
    target = str(rig.def_dir).replace("\\", "/")
    blocks = []
    for name in todo:
        # ``).add;`` и ``).add`` в конце строки без точки с запятой (mhpad, combs, prophet, vibass)
        source = def_source(rig, name).read_text(encoding="utf-8")
        text = re.sub(r"\.add\s*(;|$)", f'.writeDefFile("{target}");', source, flags=re.M)
        blocks.append("{\n" + text + "\n}.value;")
    script = rig.def_dir / "compile_defs.scd"
    script.write_text("\n".join(blocks + ['"DEFS_DONE".postln;', "0.exit;"]) + "\n", encoding="utf-8")
    # как sclang робота (``start_voice_assistant.sh``): без IDE и без песочницы QtWebEngine под root
    proc = subprocess.run([rig.sclang, "-i", "none", str(script)], capture_output=True, text=True,
                          timeout=rig.timeout, env=dict(os.environ, QT_QPA_PLATFORM="offscreen",
                                                        QTWEBENGINE_CHROMIUM_FLAGS="--no-sandbox"))
    missing = [n for n in todo if not (rig.def_dir / f"{n}.scsyndef").exists()]
    if "DEFS_DONE" not in proc.stdout or missing:
        raise SystemExit(f"sclang не скомпилировал {missing}:\n{proc.stdout[-3000:]}\n{proc.stderr[-2000:]}")


def sample_file(samples: Path, key: str) -> Path:
    """Файл ``play()``: символ + номер (``X12``) → ``x/upper/<12-й по алфавиту>`` (сортировка renardo)."""
    folder = sample_dir(samples, key[0])
    files = sorted(f for f in os.listdir(folder) if f.lower().endswith((".wav", ".aif", ".aiff")))
    return folder / files[int(key[1:] or 0) % len(files)]


def sample_dir(samples: Path, symbol: str) -> Path:
    if symbol.isalpha():
        return samples / symbol.lower() / ("upper" if symbol.isupper() else "lower")
    return samples / "_" / SYMBOL_DIRS[symbol]


def fetch_samples(samples: Path, symbols: Iterable[str]) -> None:
    """Скачать папки символов ``0_foxdot_default`` с сервера renardo (только недостающие)."""
    need = [s for s in sorted(set(symbols)) if not sample_dir(samples, s).is_dir()]
    if not need:
        return
    with urlopen(f"{SAMPLES_SERVER}/collection_index.json", timeout=60) as resp:  # noqa: S310 — фикс. адрес
        index = json.load(resp)
    prefixes = [str(sample_dir(Path("."), s).as_posix()) + "/" for s in need]
    stack = [index]
    while stack:
        node = stack.pop()
        stack += node.get("children", [])
        rel = node.get("path", "").split("0_foxdot_default/", 1)[-1]
        if "url" in node and any(rel.startswith(p) for p in prefixes):
            dest = samples / rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            with urlopen(node["url"], timeout=60) as resp:  # noqa: S310
                dest.write_bytes(resp.read())


def wav_channels(path: Path) -> int:
    raw = path.read_bytes()[:64]
    return struct.unpack("<H", raw[22:24])[0] if raw[:4] == b"RIFF" else 1


def render_wav(rig: Rig, events: Sequence[NoteEvent], bpm: float, tag: str, duration: float) -> Path:
    """События → партитура → ``scsynth -N`` (16 кГц, float, стерео) → WAV."""
    keys = sorted({e.sample for e in events if e.sample is not None})
    files = {key: sample_file(rig.samples, key) for key in keys}
    buffers = {key: (i + 1, wav_channels(files[key])) for i, key in enumerate(keys)}
    paths = {key: str(files[key]).replace("\\", "/") for key in keys}
    def_dir = str(rig.def_dir).replace("\\", "/")
    osc, wav = rig.out / f"{tag}.osc", rig.out / f"{tag}.wav"
    osc.write_bytes(encode_score(score(events, bpm, def_dir, buffers, paths, duration)))
    proc = subprocess.run([rig.scsynth, "-N", str(osc), "_", str(wav), str(SAMPLE_RATE), "WAV", "float",
                           "-o", "2", "-i", "0", "-a", "1024", "-m", "262144", "-n", "65536", "-c", "16384"],
                          capture_output=True, text=True, timeout=rig.timeout)
    if not wav.exists():
        raise SystemExit(f"NRT не дал wav: {proc.stdout[-2000:]}\n{proc.stderr[-2000:]}")
    return wav


def measure(rig: Rig, code: str, tag: str, form_beats: Optional[float] = None,
            windows: Optional[Sequence[Tuple[float, float]]] = None) -> Dict[str, Any]:
    """Программа → рендер → dB RMS и полосы по окнам (доли; по умолчанию вся форма)."""
    import numpy as np

    program, events = program_events(code, form_beats)
    form = float(form_beats or program.form_beats)
    beat_dur = 60.0 / program.bpm
    compile_defs(rig, {e.synth for e in events if e.sample is None})
    signal = read_wav(render_wav(rig, events, program.bpm, tag, form * beat_dur + TAIL_S))
    spans = windows or [(0.0, form)]
    body = np.concatenate([signal[int(a * beat_dur * SAMPLE_RATE):int(b * beat_dur * SAMPLE_RATE)] for a, b in spans])
    db = rms_db(body)
    return {"db": db, "bands": band_shares(body) if db > -200.0 else None, "events": len(events)}


# ── Режимы ─────────────────────────────────────────────────────────────────────────────────────────────────


def load_frame(path: Path = FRAME_FILE) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def frame_code(code: str, synth: Optional[str] = None, amp: Optional[float] = None) -> str:
    """Программа рамки 29.09 с другим синтом слоя и/или постоянным ``amp`` (весь блок звучит)."""
    if synth:
        code = re.sub(r"^(p[1-3]) >> \w+\(", rf"\1 >> {synth}(", code, count=1, flags=re.M)
    if amp is not None:
        code = re.sub(r"\bamp=[0-9.]+", f"amp={amp:g}", code)
    return code


def _mean_bands(rows: Sequence[Sequence[float]]) -> Tuple[float, ...]:
    low = round(sum(r[0] for r in rows) / len(rows), 3)
    mid = round(sum(r[1] for r in rows) / len(rows), 3)
    return low, mid, round(1.0 - low - mid, 3)


def frame_layer(rig: Rig, role: str, synth: Optional[str], roots: Sequence[str], half: bool) -> Dict[str, Any]:
    """Слой рамки 29.09 по тоникам: среднее dB (как таблица 29.09), разброс, полосы; ``half`` — наклон ``amp``
    по первой тонике (уровень и его половина: ``p = Δ / 6.02``)."""
    frame = load_frame()
    level = float(frame["levels"][role])
    rows = []
    for root in roots:
        code = frame_code(frame["programs"][role][root], synth)
        rows.append(measure(rig, code, f"{role}_{synth or 'ref'}_{root.replace('#', 's')}"))
    dbs = [r["db"] for r in rows]
    out = {"role": role, "synth": synth, "level": level, "db": round(sum(dbs) / len(dbs), 2),
           "db_by_root": dict(zip(roots, dbs)), "spread_db": round(max(dbs) - min(dbs), 2),
           "bands": _mean_bands([r["bands"] for r in rows]) if all(r["bands"] for r in rows) else None}
    if half:
        code = frame_code(frame["programs"][role][roots[0]], synth, amp=level / 2)
        db_half = measure(rig, code, f"{role}_{synth}_half")["db"]
        out["exponent"] = round((dbs[0] - db_half) / (20 * math.log10(2)), 2)
    return out


def regress(rig: Rig, roots: Sequence[str]) -> Dict[str, Any]:
    """Рамка 29.09 против ``LAYER_MEASURED_DB``: ±:data:`TOLERANCE_DB` на каждый вариант."""
    rows = []
    for role, option in REGRESS:
        got = frame_layer(rig, role, None, roots, half=False)
        want = kn.LAYER_MEASURED_DB[role][1][option]
        rows.append(dict(got, option=option, want_db=want, delta_db=round(got["db"] - want, 2),
                         ok=abs(got["db"] - want) <= TOLERANCE_DB))
    return {"tolerance_db": TOLERANCE_DB, "ok": all(r["ok"] for r in rows), "rows": rows}


def _role_windows(track: Any, role: str) -> List[Tuple[float, float]]:
    spans, beat = [], 0.0
    for sec in track.form.sections:
        length = sec.bars * 4.0
        if role in sec.roles:
            spans.append((beat, beat + length))
        beat += length
    return spans


def track_layers(rig: Rig, seed: int) -> Dict[str, Any]:
    """Трек v2 сида: каждый слой ``render`` отдельно — dB по секциям роли против ``Part.level_db`` модели."""
    from rob_box_music.arrange.compose import club_track
    from rob_box_music.render.renardo import render

    track = club_track(seed)
    program = render(track, "A")
    lines = program.code.splitlines()
    rows = []
    for role, slot in sorted(program.slots.items()):
        part = track.parts[role]
        if role in SAMPLE_ROLES:
            rows.append({"role": role, "skipped": "loop() DJ_Dave — сэмплы пака не замеряются этим харнессом"})
            continue
        body = [ln for ln in lines if ln.startswith(f"{slot} >> ")]
        code = f"Clock.bpm = {track.bpm}\n" + "\n".join(body) + "\n"
        got = measure(rig, code, f"track{seed}_{role}", program.form_beats, _role_windows(track, role))
        rows.append(dict(got, role=role, synth=part.synth_or_sample, model_db=part.level_db,
                         delta_db=round(got["db"] - part.level_db, 2)))
    return {"seed": seed, "track_id": track.track_id, "bpm": track.bpm, "rows": rows}


#: Рамка читаемости: темп клуба, тема «В пещере горного короля» (Григ, начало, ля минор, восьмые; длинные — четверти)
#: в коридоре лида; одиночные ноты — восьмая (``sus`` = шаг темы), раз в :data:`CLARITY_GAP_BEATS` долей.
CLARITY_BPM = 132.0
CLARITY_THEME: Tuple[Tuple[int, float], ...] = (
    (69, 0.5), (71, 0.5), (72, 0.5), (74, 0.5), (76, 0.5), (72, 0.5), (76, 1.0),
    (75, 0.5), (71, 0.5), (75, 1.0), (74, 0.5), (70, 0.5), (74, 1.0),
    (69, 0.5), (71, 0.5), (72, 0.5), (74, 0.5), (76, 0.5), (72, 0.5), (76, 0.5), (81, 0.5),
    (79, 0.5), (76, 0.5), (72, 0.5), (76, 0.5), (79, 2.0))
CLARITY_NOTES: Tuple[int, ...] = (64, 69, 72, 76, 81)
CLARITY_NOTE_BEATS = 0.5
CLARITY_GAP_BEATS = 4.0
CLARITY_HELD_BEATS = 2.0
#: Пэды для маскировки: тот же замер полосы 1–4 кГц аккордом в коридоре пэда на уровне роли.
CLARITY_PAD_CHORD: Tuple[int, ...] = (57, 60, 64)


def _model_amp(role: str, synth: str) -> Tuple[float, bool]:
    """``amp`` синта на цели роли клуба по модели громкости (как ``arrange.mix.level_amp``); без замера — 0.5."""
    unit = kn.LANE_DB_AT_UNIT.get(role, {}).get(synth)
    if unit is None:
        return 0.5, False
    target = kn.STYLES["club"].role_level_db[role]
    return min(kn.MAX_LAYER_AMP, 10.0 ** ((target - unit) / (20.0 * kn.AMP_EXPONENT.get(synth, 1.0)))), True


def _voices(synth: str, beat: float, midi: float, amp: float, sus: float, stereo: bool) -> List[NoteEvent]:
    """Нота как её играет рендер: синт с шириной (``knowledge.SYNTH_STEREO``) — два голоса ±pan, второй со сдвигом
    ``detune`` и Хаасом (``Mix.stereo`` → ``pan/pshift/delay``), мощность делится на голоса."""
    st = kn.SYNTH_STEREO.get(synth) if stereo else None
    if not st:
        return [NoteEvent(beat, "p1", synth, amp, 1.0, sus, midi=float(midi))]
    voice = amp / math.sqrt(2.0)
    haas = st.get("haas_ms", 0.0) * CLARITY_BPM / 60000.0
    return [NoteEvent(beat, "p1", synth, voice, 1.0, sus, midi=float(midi), pan=-st["pan"]),
            NoteEvent(beat + haas, "p1", synth, voice, 1.0, sus, midi=float(midi), pan=st["pan"],
                      detune=st.get("detune", 0.0))]


def clarity_layer(rig: Rig, synth: str, role: str = "lead", stereo: bool = False) -> Dict[str, Any]:
    """Читаемость синта темой: одиночные ноты (атака, хвост после ``sus``), долгая нота (чистота высоты), тема
    (разделение нот — dB последней четверти шага против первой, яркость, dB RMS); у пэда — аккорд (яркость, dB)."""
    import numpy as np

    beat_s = 60.0 / CLARITY_BPM
    amp, measured = _model_amp(role, synth)
    events: List[NoteEvent] = []
    beat = 0.0
    if role == "pad":
        events += [NoteEvent(0.0, "p1", synth, amp, 1.0, 4.0, midi=float(m)) for m in CLARITY_PAD_CHORD]
        signal = read_wav_mono(render_wav(rig, events, CLARITY_BPM, f"clarity_pad_{synth}", 4 * beat_s + TAIL_S))
        body = signal[int(0.05 * SAMPLE_RATE):int(4 * beat_s * SAMPLE_RATE)]
        return {"synth": synth, "role": role, "amp": round(amp, 3), "amp_measured": measured,
                "db": rms_db(body), **brightness(body)}
    for midi in CLARITY_NOTES:
        events += _voices(synth, beat, midi, amp, CLARITY_NOTE_BEATS, stereo)
        beat += CLARITY_GAP_BEATS
    held_at = beat
    events += _voices(synth, held_at, 69, amp, CLARITY_HELD_BEATS, stereo)
    beat += CLARITY_GAP_BEATS
    theme_at = beat
    for midi, dur in CLARITY_THEME:
        events += _voices(synth, beat, midi, amp, dur, stereo)
        beat += dur
    tag = f"clarity_{synth}{'_st' if stereo else ''}"
    signal = read_wav_mono(render_wav(rig, events, CLARITY_BPM, tag, beat * beat_s + TAIL_S))
    offset = NOTE_OFFSET_S
    shapes = [note_shape(signal, i * CLARITY_GAP_BEATS * beat_s + offset, CLARITY_NOTE_BEATS * beat_s,
                         CLARITY_GAP_BEATS * beat_s) for i in range(len(CLARITY_NOTES))]
    held = signal[int((held_at * beat_s + offset + 0.05) * SAMPLE_RATE):
                  int((held_at + CLARITY_HELD_BEATS) * beat_s * SAMPLE_RATE)]
    sep, t = [], theme_at
    for _, dur in CLARITY_THEME[:-1]:
        a, b = (t * beat_s + offset), ((t + dur) * beat_s + offset)
        q = (b - a) / 4.0
        head = signal[int(a * SAMPLE_RATE):int((a + q) * SAMPLE_RATE)]
        tail = signal[int((b - q) * SAMPLE_RATE):int(b * SAMPLE_RATE)]
        sep.append(rms_db(tail) - rms_db(head))
        t += dur
    theme = signal[int((theme_at * beat_s + offset) * SAMPLE_RATE):int(beat * beat_s * SAMPLE_RATE)]
    return {"synth": synth, "role": role, "stereo": stereo, "amp": round(amp, 3), "amp_measured": measured,
            "attack_ms": round(float(np.median([x["attack_ms"] for x in shapes])), 1),
            "tail_ms": round(float(np.median([x["tail_ms"] for x in shapes])), 1),
            "tail_ms_max": max(x["tail_ms"] for x in shapes),
            "purity": pitch_purity(held, midi_hz(69)),
            "separation_db": round(float(np.median(sep)), 1),
            "db": rms_db(theme), **brightness(theme)}


def clarity(rig: Rig, leads: Sequence[str], pads: Sequence[str]) -> Dict[str, Any]:
    """Строки читаемости лидов (моно и, у синтов с шириной, как играет рендер) и яркости пэдов на уровне роли."""
    compile_defs(rig, set(leads) | set(pads))
    rows = [clarity_layer(rig, s) for s in leads]
    rows += [clarity_layer(rig, s, stereo=True) for s in leads if s in kn.SYNTH_STEREO]
    rows += [clarity_layer(rig, s, role="pad") for s in pads]
    return {"bpm": CLARITY_BPM, "rows": rows}


def _rig(args: argparse.Namespace) -> Rig:
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    return Rig(Path(args.renardo), Path(args.samples), out, args.sclang, args.scsynth, args.timeout)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--regress", action="store_true", help="рамка 29.09 против LAYER_MEASURED_DB (±1.5 дБ)")
    mode.add_argument("--sweep", nargs="+", metavar=("ROLE", "SYNTH"), help="роль и синты: dB, наклон, полосы")
    mode.add_argument("--track", type=int, metavar="SEED", help="трек v2: слои против модели")
    mode.add_argument("--clarity", nargs="+", metavar="SYNTH", help="читаемость темы лидами (pad:СИНТ — яркость пэда)")
    parser.add_argument("--renardo", required=True, help="каталог пакета renardo_lib (установленный или из колеса)")
    parser.add_argument("--samples", required=True, help="каталог 0_foxdot_default (недостающее скачивается)")
    parser.add_argument("--out", default="nrt_out")
    parser.add_argument("--roots", nargs="+", default=["A#", "D", "A", "E", "G"])
    parser.add_argument("--sclang", default="sclang")
    parser.add_argument("--scsynth", default="scsynth")
    parser.add_argument("--timeout", type=float, default=900.0)
    args = parser.parse_args(argv)
    rig = _rig(args)
    fetch_samples(rig.samples, ["X", "-", "*"] if args.track is not None else ["X"])
    if args.regress:
        result = regress(rig, args.roots)
    elif args.sweep:
        role, synths = args.sweep[0], args.sweep[1:]
        result = {"sweep": [frame_layer(rig, role, s, args.roots, half=True) for s in synths]}
    elif args.clarity:
        result = clarity(rig, [s for s in args.clarity if not s.startswith("pad:")],
                         [s[4:] for s in args.clarity if s.startswith("pad:")])
    else:
        result = track_layers(rig, args.track)
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result.get("ok", True) else 1


if __name__ == "__main__":
    raise SystemExit(main())
