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
    return data.reshape(-1, channels)[:, 0]


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
    else:
        result = track_layers(rig, args.track)
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result.get("ok", True) else 1


if __name__ == "__main__":
    raise SystemExit(main())
