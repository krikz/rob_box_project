"""Scenes in real time: ``Scene.frame(t, ctx) -> Frame`` over the scene generators.

A scene function in ``states``/``talk``/``vehicles`` is a generator: setup for
one pass of ``T`` seconds, then ``for t in ticks: ... yield f``. ``Ticks`` is
the clock the runner sets before asking for the next frame, so the same code
renders the lab preview (t on the 25 fps grid) and the live face (t from the
node's timer, live values in ``ticks.ctx``). Past ``T`` the pass restarts with
a new seed: blinks and saccades do not repeat loop after loop.
"""
from __future__ import annotations

import inspect
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterator, List, Mapping, Optional, Tuple

import numpy as np

from . import effects as E
from . import rear as R
from .face import FPS, Frame


@dataclass(frozen=True)
class FaceContext:
    """Live inputs of a scene. ``None`` — not known, the scene uses its preview."""

    #: speaker direction, degrees off the robot's nose, + = robot's left (DOA)
    gaze_deg: Optional[float] = None
    #: mouth openness 0..1 from the TTS envelope; None = the scene's own syllables
    mouth_level: Optional[float] = None
    #: vowel hint of the envelope window: 'A' | 'O' | 'E'
    vowel: str = 'A'
    #: tempo of the playing track
    bpm: Optional[float] = None
    #: beats since the track start: integer part = beat number, fraction = phase
    beat_phase: Optional[float] = None
    #: tone of a talking face: 'neutral' | 'happy' | 'excited'
    mood: str = 'neutral'
    #: RGB multiplier of eyes and mouth (DSL tint, PR-3c); None = palette as drawn
    tint: Optional[Tuple[float, float, float]] = None
    #: rear lights when the scene does not draw them (rear.THEMES)
    rear_theme: str = 'default'
    #: slots of the LLM face scene (``face_scene.FaceScene.slots()``) for ``dsl.face_scene``
    face: Optional[Mapping[str, str]] = None


IDLE = FaceContext()


class Ticks:
    """The clock a scene generator iterates: yields whatever ``t`` the runner set."""

    def __init__(self) -> None:
        self.t = 0.0
        self.ctx = IDLE

    def __iter__(self) -> Iterator[float]:
        while True:
            yield self.t


@dataclass(frozen=True)
class SceneSpec:
    """A registered scene: generator function, length of one pass, lab metadata."""

    fn: Callable[..., Iterator[Frame]]
    period: float
    group: str
    title: str
    note: str
    kwargs: Dict[str, object] = field(default_factory=dict)

    def start(self, ticks: Ticks, loop: int) -> Iterator[Frame]:
        """Generator for pass ``loop``; pass 0 uses the scene's own seed (deterministic)."""
        kw = dict(self.kwargs, T=self.period)
        seed = inspect.signature(self.fn).parameters.get('seed')
        if loop and seed is not None:
            kw['seed'] = int(kw.get('seed', seed.default)) + 7919 * loop
        return self.fn(ticks, **kw)


def finish(f: Frame, t: float, ctx: FaceContext) -> Frame:
    """Rear lights for scenes that do not draw them, the LLM scene's ``feat``, the tint, then ``bg`` and ``fx``."""
    if not (f.rear_l.any() or f.rear_r.any()) and (not E.has_bg(ctx.face) or ctx.face.get('rear')):
        theme = R.THEMES.get(ctx.rear_theme, R.THEMES['default'])
        f.rear_l, f.rear_r = theme(t)
    E.features(f, t, ctx.face)
    if ctx.tint is not None:
        k = np.asarray(ctx.tint, dtype=float)
        f.eye_l, f.eye_r, f.mouth = f.eye_l * k, f.eye_r * k, f.mouth * k
    return E.layers(f, t, ctx.face)


class Scene:
    """One running scene: ``frame(t)`` for t seconds since the scene started."""

    def __init__(self, spec: SceneSpec) -> None:
        self.spec = spec
        self._ticks = Ticks()
        self._gen: Optional[Iterator[Frame]] = None
        self._loop = -1
        self._last = -1.0

    def raw(self, t: float, ctx: FaceContext = IDLE) -> Frame:
        """Frame as the scene draws it (no rear fill, no tint)."""
        loop, local = divmod(max(0.0, t), self.spec.period)
        loop = int(loop)
        if self._gen is None or loop != self._loop or local < self._last:
            self._ticks = Ticks()
            self._gen = self.spec.start(self._ticks, loop)
            self._loop = loop
        self._last = local
        self._ticks.t, self._ticks.ctx = local, ctx
        return next(self._gen)

    def frame(self, t: float, ctx: FaceContext = IDLE) -> Frame:
        return finish(self.raw(t, ctx), t, ctx)


def clip(spec: SceneSpec, T: Optional[float] = None) -> List[Frame]:
    """Offline render of one pass at FPS (lab previews, goldens): raw frames."""
    scene = Scene(spec)
    n = int(round((T or spec.period) * FPS))
    return [scene.raw(i / FPS) for i in range(n)]
