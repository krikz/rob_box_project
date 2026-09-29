#!/usr/bin/env python3
"""club_loudness_nrt.py — офлайн-рендер club-трека в scsynth NRT и замер RMS.

Issue #3154. Зачем: статическая модель громкости
(``rob_box_mcp_tools/core/club_loudness.py``) опирается на уровни ролей
по синтам, а в репо их нет — SynthDef'ы лежат в пакете ``renardo_lib``.
Этот скрипт получает их честно: рендерит РЕАЛЬНУЮ программу
``render_club`` через настоящие SynthDef'ы Renardo 0.9.13, сэмплы
``0_foxdot_default`` и мастер-шину ``masterfilter.scd`` из репо в
scsynth NRT на 16 кГц (как на роботе) и меряет RMS по блокам формы.

Это НЕ замер на роботе. Отличия от живого тракта (известные):

* события строит этот скрипт по семантике Renardo (``Players.py``:
  ``amp * amplify``, ``sus = dur`` если не задан, ``rate = 0`` у синтов,
  ``startSound``/синт/эффекты/``makeSound`` в группе ноты), а не сам
  Renardo; ``Clock.latency``, джиттер и ``set_time`` не моделируются;
* эффекты — ``hpf``/``lpf``/``echo``/``room``+``mix`` (то, что пишут club и
  classic), порядок как в renardo (order=2); код перед рендером проходит
  ``sanitize_renando`` — как на роботе;
* сэмпл-пак — локальный ``%APPDATA%/renardo/samples/0_foxdot_default``
  (тот же пак по умолчанию, что и в образе, но файлы не сверены побайтно);
* ReSpeaker/ALSA после scsynth не моделируются (jack_rec тоже снимает
  цифровой выход scsynth, так что сравнение с ним честное по месту).

Нужно: SuperCollider (sclang/scsynth), распакованный wheel renardo_lib
0.9.13 (``--renardo``), сэмплы (``--samples``), numpy.

Примеры::

    # трек сида: RMS по блокам 8 долей (калибровка — как в render_club_kit)
    python scripts/music/club_loudness_nrt.py --seed 7886811 --root D \
        --renardo /tmp/rl/renardo_lib --out /tmp/nrt
    # готовая программа (например classic из compose_music)
    python scripts/music/club_loudness_nrt.py --code elise.py --renardo ... --out ...
    # таблица слоёв для core/club_loudness._MEASURED_DB (+ наклон по ×0.5)
    python scripts/music/club_loudness_nrt.py --sweep --root D --renardo ... --out ...
    # classic (issue #3154): энергия нот синтов палитры и ударов play(),
    # затем генерация core/_classic_loudness_table.py из сохранённых JSON
    python scripts/music/club_loudness_nrt.py --sweep-notes pluck blip ... --renardo ... --out ... > notes.json
    python scripts/music/club_loudness_nrt.py --sweep-drums --renardo ... --out ... > drums.json
    python scripts/music/club_loudness_nrt.py --write-classic-table notes.json drums.json

``renardo_lib`` — распакованный wheel: ``pip download renardo_lib==0.9.13
--no-deps`` и ``python -m zipfile -e <whl> <dir>``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import struct
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src" / "rob_box_mcp_tools"))

SAMPLE_RATE = 16000
BUS_BASE = 16
BUS_POOL = 900
SYMBOL_DIRS = {
    "-": "hyphen", "=": "equals", "*": "asterix", "~": "tilde", "+": "plus", "&": "ampersand", "@": "at",
    "\\": "backslash", "^": "caret", ":": "colon", "$": "dollar", "!": "exclamation", "/": "forwardslash",
    "#": "hash", "%": "percent", "?": "question", ";": "semicolon", "1": "1", "2": "2", "3": "3", "4": "4",
}


# ---------------------------------------------------------------------------
# Заглушки FoxDot: исполнить программу club и собрать плееры
# ---------------------------------------------------------------------------


class TimeVar:
    """``var``/``linvar`` Renardo по доле клока (цикл по сумме длительностей)."""

    def __init__(self, values: Sequence[float], durs: Any, linear: bool) -> None:
        self.values = [float(v) for v in values]
        if isinstance(durs, (int, float)):
            durs = [durs] * len(self.values)
        self.durs = [float(d) for d in durs]
        self.linear = linear
        self.total = sum(self.durs)

    def at(self, beat: float) -> float:
        pos = beat % self.total
        for i, dur in enumerate(self.durs):
            if pos < dur:
                if not self.linear:
                    return self.values[i]
                nxt = self.values[(i + 1) % len(self.values)]
                return self.values[i] + (nxt - self.values[i]) * pos / dur
            pos -= dur
        return self.values[-1]


def value_at(value: Any, beat: float) -> float:
    return value.at(beat) if isinstance(value, TimeVar) else float(value)


@dataclass
class Spec:
    synth: str
    degree: Any
    kwargs: Dict[str, Any] = field(default_factory=dict)


class _Slot:
    def __init__(self, name: str, sink: Dict[str, Spec]) -> None:
        self.name, self.sink = name, sink

    def __rshift__(self, spec: Spec) -> None:
        self.sink[self.name] = spec


class _Clock:
    bpm = 120
    now_flag = False

    def clear(self) -> None:
        pass

    def future(self, beats: float, _fn: Any) -> None:
        self.form_beats = float(beats)

    def set_time(self, *_a: Any) -> None:
        pass

    def now(self) -> float:
        return 0.0


class _Players(dict):
    form_beats: Optional[float] = None


class _Synths(dict):
    def __missing__(self, name: str) -> Any:
        return lambda degree=None, **kw: Spec(name, degree, kw)


def run_program(code: str) -> Tuple[Dict[str, Spec], float]:
    """Исполнить программу club в заглушках. → (плееры, bpm)."""
    players: Dict[str, Spec] = {}
    clock = _Clock()
    ns: Dict[str, Any] = _Synths(
        Clock=clock,
        Scale=type("Scale", (), {"chromatic": "chromatic", "minor": "minor", "default": None}),
        Root=type("Root", (), {"default": None}),
        var=lambda v, d: TimeVar(v, d, linear=False),
        linvar=lambda v, d: TimeVar(v, d, linear=True),
    )
    for slot in ("d1", "d2", "d3", "p1", "p2", "p3"):
        ns[slot] = _Slot(slot, players)
    exec(compile(code, "<club>", "exec"), ns)  # noqa: S102 — свой код аранжировщика
    players_view = _Players(players)
    players_view.form_beats = getattr(clock, "form_beats", None)
    return players_view, float(clock.bpm)


# ---------------------------------------------------------------------------
# События по семантике Renardo
# ---------------------------------------------------------------------------


def play_steps(pattern: str) -> List[str]:
    from rob_box_mcp_tools.core.renardo_sanitizer import _play_steps

    steps = _play_steps(pattern)
    assert steps is not None, pattern
    return steps


@dataclass
class Event:
    time: float
    synth: str
    lane: str
    amp: float
    sus: float
    freq: Optional[float] = None
    sample: Optional[str] = None
    fx: Dict[str, float] = field(default_factory=dict)


def _pick(value: Any, index: int, beat: float) -> Any:
    """Значение атрибута плеера для события ``index``: список цикличен, var — по доле."""
    if isinstance(value, list):
        value = value[index % len(value)]
    return value_at(value, beat) if isinstance(value, (TimeVar, int, float)) else value


def _fx_at(kwargs: Dict[str, Any], index: int, beat: float) -> Dict[str, float]:
    fx = {}
    for key in ("hpf", "lpf", "echo", "room"):
        if key in kwargs:
            value = _pick(kwargs[key], index, beat)
            if value:
                fx[key] = float(value)
    if "room" in fx:
        fx["mix"] = float(_pick(kwargs.get("mix", 0.1), index, beat))
    return fx


def _play_symbol(steps: List[str], index: int) -> str:
    step = steps[index % len(steps)]
    if step.startswith("("):
        inner = step[1:-1]
        step = inner[(index // len(steps)) % len(inner)]
    return step


def events_for(slot: str, spec: Spec, form_beats: float, beat_dur: float) -> List[Event]:
    """События плеера за форму: ``amp * amplify``, ``sus`` = ``dur`` если не задан."""
    kw = spec.kwargs
    default_dur = 0.5 if spec.synth == "play" else 1
    out: List[Event] = []
    steps = play_steps(spec.degree) if spec.synth == "play" else None
    notes = spec.degree if isinstance(spec.degree, list) else [spec.degree]
    index, beat = 0, 0.0
    while beat < form_beats - 1e-9:
        dur = float(_pick(kw.get("dur", default_dur), index, beat))
        sus = float(_pick(kw["sus"], index, beat)) if "sus" in kw else dur
        amp = float(_pick(kw.get("amp", 1), index, beat)) * float(_pick(kw.get("amplify", 1), index, beat))
        if amp > 0:
            base = dict(time=beat * beat_dur, lane=slot, amp=amp, sus=sus * beat_dur,
                        fx=dict(_fx_at(kw, index, beat), beat_dur=beat_dur))
            if steps is not None:
                symbol = _play_symbol(steps, index)
                if symbol not in ". ":
                    sample = int(_pick(kw.get("sample", 0), index, beat))
                    out.append(Event(synth="play", sample=f"{symbol}{sample}", **base))
            else:
                note = notes[index % len(notes)]
                for midi in (note if isinstance(note, tuple) else (note,)):
                    if midi is None:
                        continue
                    freq = 440.0 * 2 ** ((float(midi) - 69) / 12)
                    out.append(Event(synth=spec.synth, freq=freq, **base))
        index += 1
        beat += dur
    return out


# ---------------------------------------------------------------------------
# Score для sclang
# ---------------------------------------------------------------------------


def sample_file(samples: Path, key: str) -> Path:
    symbol, index = key[0], int(key[1:] or 0)
    if symbol.isalpha():
        folder = samples / symbol.lower() / ("upper" if symbol.isupper() else "lower")
    else:
        folder = samples / "_" / SYMBOL_DIRS[symbol]
    files = sorted(f for f in os.listdir(folder) if f.lower().endswith((".wav", ".aif", ".aiff")))
    return folder / files[index % len(files)]


def _riff_chunks(path: Path) -> Dict[bytes, bytes]:
    """Чанки RIFF/WAVE (``wave`` из stdlib не читает float-WAV, формат 3)."""
    raw = path.read_bytes()
    chunks: Dict[bytes, bytes] = {}
    pos = 12
    while pos + 8 <= len(raw):
        cid, size = raw[pos:pos + 4], struct.unpack("<I", raw[pos + 4:pos + 8])[0]
        chunks.setdefault(cid, raw[pos + 8:pos + 8 + size])
        pos += 8 + size + (size & 1)
    return chunks


def wav_channels(path: Path) -> int:
    """Число каналов WAV; не RIFF (aif и т.п.) — 1 (scsynth прочитает файл сам)."""
    fmt = _riff_chunks(path).get(b"fmt ") if path.read_bytes()[:4] == b"RIFF" else None
    return struct.unpack("<H", fmt[2:4])[0] if fmt else 1


def _sc_path(path: Path) -> str:
    return str(path).replace("\\", "/")


def _def_expr(path: Path) -> str:
    text = path.read_text(encoding="utf-8")
    text = re.sub(r"\)\s*\.add\s*;\s*$", ")", text.strip())
    return "(" + text + ")"


def _patched_sources() -> Dict[str, str]:
    """Исходники, которые образ робота пишет поверх renardo_lib (``fix_brass_scd.py``)."""
    sys.path.insert(0, str(REPO / "src" / "rob_box_voice"))
    try:
        from rob_box_voice.core import renardo_synthdef_patches as patches
    except Exception:  # noqa: BLE001 — без патчей рендер идёт по исходникам пакета
        return {}
    names = {"brass": "BRASS_SYNTHDEF", "organ": "ORGAN_SYNTHDEF", "tb303": "TB303_SYNTHDEF", "fuzz": "FUZZ_SYNTHDEF"}
    return {name: getattr(patches, const) for name, const in names.items() if hasattr(patches, const)}


def compile_defs(synths: Sequence[str], renardo: Path, def_dir: Path, sclang: str, timeout: float) -> None:
    """Скомпилировать нужные SynthDef'ы в ``def_dir`` (``writeDefFile``) через sclang."""
    sc = renardo / "SynthDefManagement" / "sclang_code"
    custom = REPO / "docker/vision/voice_assistant/custom_synthdefs"
    files = [sc / "scsynth" / f"{name}.scd" if (sc / "scsynth" / f"{name}.scd").exists()
             else custom / f"{name}.scd" for name in synths]
    files += [sc / "scsynth" / "play1.scd", sc / "scsynth" / "play2.scd"]
    files += [sc / "sceffects" / f"{n}.scd" for n in
              ("startSound", "makeSound", "highPassFilter", "lowPassFilter", "combDelay", "reverb")]
    files.append(REPO / "docker/vision/voice_assistant/custom_synthdefs/masterfilter.scd")
    def_dir.mkdir(parents=True, exist_ok=True)
    patched = _patched_sources()
    lines = ["("]
    for path in files:
        source = patched.get(path.stem)
        expr = "(" + re.sub(r"\)\s*\.add\s*;\s*$", ")", source.strip()) + ")" if source else _def_expr(path)
        lines.append(f'{expr}.writeDefFile("{_sc_path(def_dir)}");')
    lines += ['"DEFS_DONE".postln;', "0.exit;", ")"]
    script = def_dir / "compile_defs.scd"
    script.write_text("\n".join(lines) + "\n", encoding="utf-8")
    proc = subprocess.run([sclang, str(script)], cwd=str(Path(sclang).parent),
                          capture_output=True, text=True, timeout=timeout)
    if "DEFS_DONE" not in proc.stdout:
        raise SystemExit(f"sclang не скомпилировал SynthDef'ы:\n{proc.stdout[-3000:]}\n{proc.stderr[-2000:]}")


def _osc_str(value: str) -> bytes:
    raw = value.encode("utf-8") + b"\0"
    return raw + b"\0" * (-len(raw) % 4)


def osc_message(address: str, *args: Any) -> bytes:
    tags, body = ",", b""
    for arg in args:
        if isinstance(arg, bool) or isinstance(arg, int):
            tags += "i"
            body += struct.pack(">i", int(arg))
        elif isinstance(arg, float):
            tags += "f"
            body += struct.pack(">f", arg)
        else:
            tags += "s"
            body += _osc_str(str(arg))
    return _osc_str(address) + _osc_str(tags) + body


def osc_bundle(time: float, messages: Sequence[bytes]) -> bytes:
    seconds = int(time)
    frac = int((time - seconds) * (1 << 32))
    body = b"#bundle\0" + struct.pack(">II", seconds, frac)
    for msg in messages:
        body += struct.pack(">i", len(msg)) + msg
    return struct.pack(">i", len(body)) + body


def build_score(events: List[Event], samples: Path, def_dir: Path, duration: float,
                master_gain: float) -> bytes:
    """Бинарный OSC-score для ``scsynth -N`` (семантика нот Renardo ``get_bundle``)."""
    buffers: Dict[str, Tuple[int, str]] = {}
    for ev in events:
        if ev.sample and ev.sample not in buffers:
            path = sample_file(samples, ev.sample)
            play = "play2" if wav_channels(path) == 2 else "play1"
            buffers[ev.sample] = (len(buffers) + 1, play)
            buffers[ev.sample + "#path"] = (0, _sc_path(path))
    head = [osc_message("/d_loadDir", _sc_path(def_dir))]
    for sym, (num, play) in list(buffers.items()):
        if not sym.endswith("#path"):
            head.append(osc_message("/b_allocRead", num, buffers[sym + "#path"][1]))
    out = [osc_bundle(0.0, head),
           osc_bundle(0.001, [osc_message("/g_new", 1, 0, 0)]),
           osc_bundle(0.002, [osc_message("/s_new", "masterfilter", 999, 1, 0, "gain", float(master_gain))])]
    node = 2000
    for k, ev in enumerate(sorted(events, key=lambda e: e.time)):
        bus = BUS_BASE + (k % BUS_POOL)
        group, node = node, node + 1
        msgs = [osc_message("/g_new", group, 1, 1)]
        if ev.sample:
            num, play = buffers[ev.sample]
            rate, synth, extra = 1.0, play, ["buf", num]
        else:
            rate, synth, extra = float(ev.freq), ev.synth, ["rate", 0.0, "freq", float(ev.freq)]
        msgs.append(osc_message("/s_new", "startSound", node, 0, group, "bus", bus,
                                "sus", float(ev.sus * 8), "rate", rate))
        node += 1
        beat_dur = float(ev.fx.get("beat_dur", 0.5))
        msgs.append(osc_message("/s_new", synth, node, 1, group, "bus", bus, "amp", float(ev.amp),
                                "sus", float(ev.sus), "pan", 0.0, "fmod", 0.0, "blur", 1.0,
                                "beat_dur", beat_dur, *extra))
        node += 1
        # порядок эффектов order=2 в renardo: hpf, lpf, echo, room
        for name, key, extra_args in (("highPassFilter", "hpf", ("hpr", 1.0)),
                                      ("lowPassFilter", "lpf", ("lpr", 1.0)),
                                      ("combDelay", "echo", ("beat_dur", beat_dur, "echotime", 1.0)),
                                      ("reverb", "room", ("mix", ev.fx.get("mix", 0.1)))):
            if key in ev.fx:
                msgs.append(osc_message("/s_new", name, node, 1, group, "bus", bus,
                                        key, float(ev.fx[key]), *[float(a) if not isinstance(a, str) else a
                                                                  for a in extra_args]))
                node += 1
        msgs.append(osc_message("/s_new", "makeSound", node, 1, group, "bus", bus, "sus", float(ev.sus)))
        node += 1
        out.append(osc_bundle(ev.time + 0.01, msgs))
    out.append(osc_bundle(duration, [osc_message("/c_set", 0, 0.0)]))
    return b"".join(out)


# ---------------------------------------------------------------------------
# Замер
# ---------------------------------------------------------------------------


def read_wav(path: Path):
    import numpy as np

    chunks = _riff_chunks(path)
    fmt_tag, channels = struct.unpack("<HH", chunks[b"fmt "][:4])
    bits = struct.unpack("<H", chunks[b"fmt "][14:16])[0]
    if fmt_tag == 3 or bits == 32:
        data = np.frombuffer(chunks[b"data"], dtype="<f4")
    else:
        data = np.frombuffer(chunks[b"data"], dtype="<i2").astype("f4") / 32768.0
    return data.reshape(-1, channels)[:, 0]


def rms_db(x) -> float:
    import numpy as np

    value = float(np.sqrt(np.mean(np.square(x.astype("f8"))))) if len(x) else 0.0
    return 20 * math.log10(value) if value > 0 else -200.0


def block_levels(signal, beat_dur: float, block_beats: float, n_blocks: int) -> List[float]:
    per = int(round(block_beats * beat_dur * SAMPLE_RATE))
    start = int(0.01 * SAMPLE_RATE)
    return [round(rms_db(signal[start + i * per:start + (i + 1) * per]), 1) for i in range(n_blocks)]


def _club_program(args: argparse.Namespace) -> Tuple[str, Dict[str, Any], str]:
    from rob_box_mcp_tools.core.club_arranger import club_kit, render_club_kit

    kit = club_kit(args.seed)
    kit.update(json.loads(args.kit or "{}"))
    levels = json.loads(getattr(args, "levels", None) or "null")
    code = render_club_kit(kit, bpm=args.bpm, root=args.root, seed=args.seed, levels=levels)
    tag = f"seed{args.seed}_{args.root.replace('#', 's')}_{int(args.bpm)}_" + "-".join(
        kit[k] for k in ("template", "kick", "hats", "lead", "bass", "pad")) + (
        "_lv" + "".join(f"{k[0]}{v:g}" for k, v in sorted(levels.items())) if levels else "")
    return code, {"seed": args.seed, "root": args.root, "kit": kit}, tag


def render_events(events: List[Event], synths: Sequence[str], args: argparse.Namespace, out: Path, tag: str,
                  duration: float) -> Path:
    """События → NRT-рендер scsynth → путь к wav (float, 16 кГц)."""
    wav, osc = out / f"{tag}.wav", out / f"{tag}.osc"
    def_dir = out / "defs"
    compile_defs(synths, Path(args.renardo), def_dir, args.sclang, args.timeout)
    osc.write_bytes(build_score(events, Path(args.samples), def_dir, duration, args.master_gain))
    scsynth = str(Path(args.sclang).with_name("scsynth.exe" if os.name == "nt" else "scsynth"))
    proc = subprocess.run([scsynth, "-N", str(osc), "_", str(wav), str(SAMPLE_RATE), "WAV", "float",
                           "-o", "2", "-i", "0", "-a", "1024", "-m", "262144", "-n", "65536", "-c", "16384"],
                          cwd=str(Path(scsynth).parent), capture_output=True, text=True, timeout=args.timeout)
    if not wav.exists():
        raise SystemExit(f"NRT не дал wav: {proc.stdout[-2000:]}\n{proc.stderr[-2000:]}")
    return wav


def _sanitized(code: str) -> str:
    """Как на роботе: код проходит санитайзер (pianovel→rhpiano, кап amp=N, oct)."""
    from rob_box_mcp_tools.core.renardo_sanitizer import sanitize_renando

    result = sanitize_renando(code, 0.85)
    return result.code if result.code else code


def classic_program(name: str, **kwargs: Any) -> str:
    """Код classic, как его собирает ``compose_music(name=...)`` (без ROS)."""
    from unittest.mock import MagicMock

    for mod in ("rclpy", "rclpy.node", "rclpy.action", "rclpy.qos", "std_msgs", "std_msgs.msg",
                "geometry_msgs", "geometry_msgs.msg", "nav2_msgs", "nav2_msgs.action",
                "action_msgs", "action_msgs.srv", "action_msgs.msg"):
        sys.modules.setdefault(mod, MagicMock())
    from rob_box_mcp_tools.core.rtttl_library import RtttlLibrary
    from rob_box_mcp_tools.tools.music import ComposeMusicTool

    tool = ComposeMusicTool(MagicMock(), MagicMock(), RtttlLibrary())
    err, built = tool._build_arrangement(name=name, **kwargs)
    if err is not None:
        raise SystemExit(f"classic {name!r}: {err.error}")
    return built.code


def render(args: argparse.Namespace) -> Dict[str, Any]:
    """Отрендерить программу (club по сиду или ``--code``) и снять RMS по блокам 8 долей."""
    import numpy as np

    if getattr(args, "code", None):
        code, meta, tag = Path(args.code).read_text(encoding="utf-8"), {"code": str(args.code)}, Path(args.code).stem
    else:
        code, meta, tag = _club_program(args)
    code = _sanitized(code)
    players, bpm = run_program(code)
    clock_form = getattr(players, "form_beats", None)
    form = float(args.form or clock_form or 128)
    beat_dur = 60.0 / bpm
    events: List[Event] = []
    for slot, spec in players.items():
        if args.only and slot not in args.only:
            continue
        events.extend(events_for(slot, spec, form, beat_dur))
    synths = sorted({e.synth for e in events if e.synth != "play"})
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    tag += ("_" + "-".join(args.only)) if args.only else ""
    # Windows MAX_PATH: длинный тег → короткий префикс + хэш (имя файла ≤ ~60 символов).
    tag = tag if len(tag) <= 60 else tag[:40] + "_" + hashlib.sha1(tag.encode()).hexdigest()[:12]
    wav = render_events(events, synths, args, out, tag, form * beat_dur + 2.0)
    signal = read_wav(wav)
    body = signal[: int(form * beat_dur * SAMPLE_RATE)]
    return dict(meta, bpm=bpm, only=args.only, events=len(events), wav=str(wav), form_beats=form,
                form_rms_db=round(rms_db(body), 1),
                peak_dbfs=round(20 * math.log10(float(np.max(np.abs(body))) or 1e-10), 1),
                block_rms_db=block_levels(signal, beat_dur, 8, int(form // 8)))


_SWEEP_SLOTS = {"kick": "d1", "hats": "d2", "clap": "d3", "lead": "p1", "bass": "p2", "pad": "p3"}


def _single_slot_program(code: str, slot: str, amp: float) -> str:
    """Оставить в программе один плеер и поставить ему постоянный ``amp`` (весь блок звучит)."""
    out, keep = [], True
    for line in code.splitlines():
        head = re.match(r"^(d[1-3]|p[1-3]) >> ", line)
        if head:
            keep = head.group(1) == slot
        elif line and not line[0].isspace():
            # верхнеуровневая строка (Clock/Root/Scale/gflt/<slot>_motif) — нужна всем
            out.append(line)
            continue
        if keep or not line:
            out.append(line)
    return re.sub(r"amp=(var\(\[[^\]]*\], \[[^\]]*\]\)|[0-9.]+)", f"amp={amp:g}", "\n".join(out)) + "\n"


def _sweep_options() -> List[Tuple[str, str]]:
    from rob_box_mcp_tools.core.club_arranger import HATS_PATTERNS, KICK_PATTERNS, ROLE_SYNTHS

    pairs = [("kick", k) for k in KICK_PATTERNS] + [("hats", h) for h in HATS_PATTERNS] + [("clap", "clap")]
    return pairs + [(role, synth) for role in ("lead", "bass", "pad") for synth in ROLE_SYNTHS[role]]


def sweep(args: argparse.Namespace) -> Dict[str, Dict[str, Dict[str, float]]]:
    """Таблица для ``core/club_loudness._MEASURED_DB``: слой в одиночку, весь блок звучит.

    Уровень — ``LAYER_LEVELS`` и ×0.5 от него (наклон ``p`` = разница / 6.02).
    Каркас — эталон seed=0 на ``dj_dave_32`` с заменой варианта слоя.
    """
    from rob_box_mcp_tools.core.club_arranger import LAYER_LEVELS, club_kit, render_club_kit

    table: Dict[str, Dict[str, Dict[str, float]]] = {}
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    for lane, option in _sweep_options():
        kit = dict(club_kit(0), template="dj_dave_32")
        if lane != "clap":
            kit[lane] = option
        code = render_club_kit(kit, bpm=args.bpm, root=args.root, seed=0)
        level = LAYER_LEVELS[lane]
        row = {}
        for name, amp in (("db", level), ("db_half", level / 2)):
            path = out / f"sweep_{lane}_{option}_{name}.py"
            path.write_text(_single_slot_program(code, _SWEEP_SLOTS[lane], amp), encoding="utf-8")
            one = argparse.Namespace(**dict(vars(args), code=path, only=None, form=None))
            row[name] = render(one)["form_rms_db"]
        row["level"] = level
        row["exponent"] = round((row["db"] - row["db_half"]) / (20 * math.log10(2)), 2)
        table.setdefault(lane, {})[option] = row
        print(lane, option, row, file=sys.stderr, flush=True)
    return table


# ---------------------------------------------------------------------------
# Issue #3154 (classic): энергия одной ноты / одного удара — таблица для
# core/classic_loudness.py. Каждая нота звучит отдельно (пауза 8·sus + 1.5 с:
# makeSound держит узел sus·8), энергия = Σx² / SR на окне ноты, в dB.
# ---------------------------------------------------------------------------

NOTE_PITCHES: Tuple[int, ...] = tuple(range(24, 97, 6))
NOTE_SUS_S: Tuple[float, ...] = (0.125, 0.5, 2.0)
NOTE_AMP = 0.1
SCAN_AMPS: Tuple[float, ...] = (0.05, 0.1, 0.2, 0.4, 0.85)
SCAN_PITCHES: Tuple[int, ...] = (48, 72)
NOTE_BEAT_S = 0.5
NOTE_FILTERS: Dict[str, Dict[str, float]] = {
    "none": {}, "hpf261": {"hpf": 261.6}, "hpf523": {"hpf": 523.3},
    "lpf523": {"lpf": 523.3}, "lpf2000": {"lpf": 2000.0},
}
DRUM_SAMPLES = (0, 1, 2, 3)


def _energy_db(signal, start_s: float, length_s: float) -> float:
    import numpy as np

    a, b = int(start_s * SAMPLE_RATE), int((start_s + length_s) * SAMPLE_RATE)
    energy = float(np.sum(np.square(signal[a:b].astype("f8")))) / SAMPLE_RATE
    return round(10 * math.log10(energy), 2) if energy > 0 else -200.0


TAIL_PITCHES = (48, 60, 72)
TAIL_SUS_S = (0.5, 2.0)


def _t95(signal, start_s: float, length_s: float) -> float:
    """Секунды от начала ноты до 95 % её энергии (сколько нота реально звучит)."""
    import numpy as np

    a, b = int(start_s * SAMPLE_RATE), int((start_s + length_s) * SAMPLE_RATE)
    energy = np.cumsum(np.square(signal[a:b].astype("f8")))
    if not len(energy) or energy[-1] <= 0:
        return 0.0
    return round(float(np.searchsorted(energy, 0.95 * energy[-1])) / SAMPLE_RATE, 3)


def _isolated(specs: List[Tuple[Any, Dict[str, Any]]], gap) -> Tuple[List[Event], List[Tuple[Any, float, float]]]:
    """Разложить ноты по времени без перекрытия: [(ключ, начало, окно)]."""
    events, windows, t = [], [], 0.2
    for key, kw in specs:
        window = gap(kw)
        events.append(Event(time=t, **kw))
        windows.append((key, t, window))
        t += window
    return events, windows


def _note(synth: str, midi: float, sus: float, amp: float, fx: Mapping[str, float]) -> Dict[str, Any]:
    return dict(synth=synth, lane="p1", amp=amp, sus=sus, freq=440.0 * 2 ** ((midi - 69) / 12),
                fx=dict(fx, beat_dur=NOTE_BEAT_S))


def sweep_notes(args: argparse.Namespace, synths: Sequence[str]) -> Dict[str, Any]:
    """Энергия ноты синта: фильтр × sus × высота при amp=NOTE_AMP + скан по amp.

    Скан (высоты :data:`SCAN_PITCHES`, sus 0.5 с, amp :data:`SCAN_AMPS`) даёт
    наклон по amp и «безопасный потолок»: у части синтов энергия не растёт
    с amp, а ОБРЫВАЕТСЯ (ambi за hpf: amp ≥ 0.15 → −110 dB в этом рендере),
    калибровка не должна поднимать такой слой выше.
    """
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    table: Dict[str, Any] = {}
    for synth in synths:
        specs = []
        for cond, fx in NOTE_FILTERS.items():
            for sus in NOTE_SUS_S:
                for midi in NOTE_PITCHES:
                    specs.append((("grid", cond, sus, midi), _note(synth, midi, sus, NOTE_AMP, fx)))
            for amp in SCAN_AMPS:
                for midi in SCAN_PITCHES:
                    specs.append((("scan", cond, amp, midi), _note(synth, midi, 0.5, amp, fx)))
        events, windows = _isolated(specs, lambda kw: 8 * kw["sus"] + 1.5)
        wav = render_events(events, [synth], args, out, f"notes_{synth}", windows[-1][1] + windows[-1][2] + 1)
        signal = read_wav(wav)
        row: Dict[str, Any] = {cond: {str(sus): {} for sus in NOTE_SUS_S} for cond in NOTE_FILTERS}
        row["scan"] = {cond: {} for cond in NOTE_FILTERS}
        tails: Dict[str, List[float]] = {}
        for (kind, cond, x, midi), start, window in windows:
            db = _energy_db(signal, start, window)
            if kind == "grid":
                row[cond][str(x)][str(midi)] = db
                if cond == "none" and midi in TAIL_PITCHES and x in TAIL_SUS_S:
                    tails.setdefault(str(x), []).append(_t95(signal, start, window))
            else:
                row["scan"][cond].setdefault(str(x), []).append(db)
        row["tail95_s"] = {sus: sorted(v)[len(v) // 2] for sus, v in tails.items()}
        table[synth] = row
        print(synth, {c: {a: v for a, v in row["scan"][c].items()} for c in ("none", "hpf261")},
              file=sys.stderr, flush=True)
    return table


def drum_symbols(samples: Path) -> List[str]:
    symbols = [c for c in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"
               if (samples / c.lower() / ("upper" if c.isupper() else "lower")).is_dir()]
    return symbols + [c for c, d in SYMBOL_DIRS.items() if (samples / "_" / d).is_dir()]


def sweep_drums(args: argparse.Namespace) -> Dict[str, Dict[str, float]]:
    """Энергия одного удара ``play()`` при amp=0.5 (dur 1/4 доли на 120 BPM)."""
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    specs = []
    for symbol in drum_symbols(Path(args.samples)):
        for sample in DRUM_SAMPLES:
            specs.append(((symbol, sample), dict(
                synth="play", lane="d1", amp=NOTE_AMP, sus=0.25 * NOTE_BEAT_S, sample=f"{symbol}{sample}",
                fx=dict(beat_dur=NOTE_BEAT_S))))
    events, windows = _isolated(specs, lambda kw: 8 * kw["sus"] + 1.0)
    wav = render_events(events, [], args, out, "drums", windows[-1][1] + windows[-1][2] + 1)
    signal = read_wav(wav)
    table: Dict[str, Dict[str, float]] = {}
    for (symbol, sample), start, window in windows:
        table.setdefault(symbol, {})[str(sample)] = _energy_db(signal, start, window)
    return table


#: Провал точки ниже обоих соседей больше чем на столько dB — выброс, не тембр.
DESPIKE_DB = 12.0


def _despike(values: List[float]) -> List[float]:
    """Убрать одиночные провалы по высоте (~1 % точек: gong, scatter, space, soprano …).

    ``makeSound`` renardo гасит ноту ``DetectSilence`` (порог 1e-4 за 0,1 с):
    нота с медленной атакой за фильтром, или синт со случайным возбуждением,
    иногда «умирает» на старте — одна точка сетки уходит на −80…−110 dB.
    Это случайность одного прогона, а не свойство высоты: точка заменяется
    средним соседей.
    """
    out = list(values)
    for i, value in enumerate(values):
        neighbours = [values[j] for j in (i - 1, i + 1) if 0 <= j < len(values)]
        if value < min(neighbours) - DESPIKE_DB:
            out[i] = sum(neighbours) / len(neighbours)
    return out


SAFE_FALL_DB = 6.0
SAFE_MARGIN = 0.5


def _scan_db(scan: Dict[str, List[float]], amp: float) -> float:
    values = scan[str(amp)]
    return 10 * math.log10(sum(10 ** (v / 10) for v in values) / len(values))


def _scan_exponent(scan: Dict[str, List[float]]) -> int:
    slope = (_scan_db(scan, 0.2) - _scan_db(scan, 0.05)) / (20 * math.log10(4))
    return 2 if slope >= 1.5 else 1


def _safe_amp(scan: Dict[str, List[float]], exponent: int) -> float:
    """Потолок amp по скану: до первого ОБРЫВА энергии (не насыщения).

    Обрыв — громче amp, а энергия УПАЛА больше чем на 6 dB к предыдущему
    шагу скана (ambi за hpf: −55 → −110 dB). Порог обрыва плавает с
    высотой и ``sus`` (ambi: 0.15 на ноте 60, 0.2+ на 48/72), поэтому за
    потолок берётся половина последнего целого шага. Насыщение tanh
    мастера (dub: +3 dB вместо +12 при 0.4→0.85) обрывом не считается.
    ``exponent`` в сигнатуре — для ясности вызова, в критерии не участвует.
    """
    del exponent
    previous_amp, previous_db = SCAN_AMPS[0], _scan_db(scan, SCAN_AMPS[0])
    for amp in SCAN_AMPS[1:]:
        db = _scan_db(scan, amp)
        if db < previous_db - SAFE_FALL_DB:
            return round(previous_amp * SAFE_MARGIN, 3)
        previous_amp, previous_db = amp, db
    return SCAN_AMPS[-1]


def write_classic_table(notes: Dict[str, Any], drums: Dict[str, Dict[str, float]], path: Path) -> None:
    """Сгенерировать ``core/_classic_loudness_table.py`` из ``--sweep-notes``/``--sweep-drums``.

    Наклон по amp — по скану amp 0.05…0.2 без фильтра, округлён до 1 или 2
    (граница 1.5): в исходниках SynthDef amp входит в сигнал один или два
    раза, а синты со случайным возбуждением дают шумный дробный наклон.
    Безопасный потолок — наибольший amp скана, до которого энергия идёт по
    наклону без обрыва (:func:`_safe_amp`).
    """
    def row(values: Dict[str, float]) -> str:
        return "(" + ", ".join(f"{v:.1f}" for v in _despike([values[str(m)] for m in NOTE_PITCHES])) + ")"

    lines = [
        '"""Сгенерировано ``scripts/music/club_loudness_nrt.py`` (``write_classic_table``), не править руками.',
        "",
        "Энергия одной ноты (dB, 10·log10(Σx²/SR)) в офлайн-рендере scsynth NRT 16 кГц:",
        "renardo_lib 0.9.13 + патчи образа (brass/organ/tb303/fuzz) + masterfilter gain=0.5,",
        f"amp={NOTE_AMP}, 120 BPM. НЕ замер на роботе. Issue #3154.",
        '"""',
        "",
        "# flake8: noqa",
        f"NOTE_PITCHES = {tuple(NOTE_PITCHES)!r}",
        f"NOTE_SUS_S = {tuple(NOTE_SUS_S)!r}",
        f"NOTE_AMP = {NOTE_AMP!r}",
        f"FILTERS = {tuple(NOTE_FILTERS)!r}",
        "",
        "#: синт -> фильтр -> (по NOTE_SUS_S) -> (по NOTE_PITCHES) dB энергии ноты",
        "NOTE_DB = {",
    ]
    for synth in sorted(notes):
        lines.append(f"    {synth!r}: {{")
        for cond in NOTE_FILTERS:
            rows = ", ".join(row(notes[synth][cond][str(sus)]) for sus in NOTE_SUS_S)
            lines.append(f"        {cond!r}: ({rows}),")
        lines.append("    },")
    lines += ["}", "", "#: синт -> наклон громкости по amp (dB = 20·p·log10(amp))", "EXPONENT = {"]
    exponents = {synth: _scan_exponent(notes[synth]["scan"]["none"]) for synth in notes}
    for synth in sorted(notes):
        lines.append(f"    {synth!r}: {exponents[synth]},")
    lines += ["}", "", "#: синт -> фильтр -> наибольший amp, где энергия ещё растёт по наклону", "SAFE_AMP = {"]
    for synth in sorted(notes):
        safe = {cond: _safe_amp(notes[synth]["scan"][cond], exponents[synth]) for cond in NOTE_FILTERS}
        lines.append(f"    {synth!r}: {safe!r},")
    lines += ["}", "", "#: синт -> (t95 при sus 0.5 с, t95 при sus 2 с), с: сколько нота звучит (makeSound держит sus·8)",
              "TAIL_S = {"]
    for synth in sorted(notes):
        tail = notes[synth].get("tail95_s", {})
        lines.append(f"    {synth!r}: ({tail.get('0.5', 0.5):.2f}, {tail.get('2.0', 2.0):.2f}),")
    lines += ["}", "", "#: символ play() -> dB энергии удара по sample=0..3", "DRUM_DB = {"]
    for symbol in sorted(drums):
        values = ", ".join(f"{drums[symbol][str(i)]:.1f}" for i in DRUM_SAMPLES)
        lines.append(f"    {symbol!r}: ({values}),")
    lines += ["}", ""]
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--root", default="A#")
    parser.add_argument("--bpm", type=float, default=124)
    parser.add_argument("--master-gain", type=float, default=0.5)
    parser.add_argument("--kit", default=None, help='правка каркаса сида, JSON: {"lead": "blip"}')
    parser.add_argument("--levels", default=None, help='множители слоёв 0..1, JSON: {"pad": 0.5}')
    parser.add_argument("--code", type=Path, default=None, help="готовая программа (classic) вместо club")
    parser.add_argument("--form", type=float, default=None, help="длина формы в долях (иначе Clock.future)")
    parser.add_argument("--only", nargs="*", default=None, help="только эти слоты (d1 p2 …)")
    parser.add_argument("--renardo", default=None, help="распакованный пакет renardo_lib")
    default_samples = Path(os.environ.get("APPDATA", "~")) / "renardo/samples/0_foxdot_default"
    parser.add_argument("--samples", default=str(default_samples))
    parser.add_argument("--sclang", default=r"C:\Program Files\SuperCollider-3.14.1\sclang.exe")
    parser.add_argument("--out", default="nrt_out")
    parser.add_argument("--timeout", type=float, default=600)
    parser.add_argument("--sweep", action="store_true", help="таблица уровней слоёв для core/club_loudness")
    parser.add_argument("--sweep-notes", nargs="+", default=None, metavar="SYNTH",
                        help="энергия нот синтов для core/classic_loudness")
    parser.add_argument("--sweep-drums", action="store_true", help="энергия ударов play() для classic")
    parser.add_argument("--write-classic-table", nargs=2, default=None, metavar=("NOTES_JSON", "DRUMS_JSON"),
                        help="собрать core/_classic_loudness_table.py из сохранённых --sweep-notes/--sweep-drums")
    args = parser.parse_args(argv)
    if args.write_classic_table:
        notes_json, drums_json = (json.loads(Path(p).read_text(encoding="utf-8")) for p in args.write_classic_table)
        target = REPO / "src/rob_box_mcp_tools/rob_box_mcp_tools/core/_classic_loudness_table.py"
        write_classic_table(notes_json, drums_json, target)
        print(target)
        return 0
    if args.sweep_notes:
        result: Any = sweep_notes(args, args.sweep_notes)
    elif args.sweep_drums:
        result = sweep_drums(args)
    else:
        result = sweep(args) if args.sweep else render(args)
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
