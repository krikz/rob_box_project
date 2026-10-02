"""``Program`` — результат рендера: текст Renardo + метаданные для проверки и лога (ADR-0149 §3.2)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet, Mapping


@dataclass(frozen=True)
class Program:
    """Что исполнит плеер и что он обязан сверить до exec (I15).

    ``code`` — только присваивания плеерам деки: без ``Clock.clear``/``Clock.bpm``
    (темп ставит владелец плеера один раз на сет), без постобработки.
    """

    code: str
    track_id: str
    deck: str
    bpm: int
    form_beats: float
    slots: Mapping[str, str]  # роль → слот деки
    synths: FrozenSet[str]  # SynthDef-ы тональных ролей (сверка с сервером)
    samples: FrozenSet[str]  # символы play() ударных ролей; ``X:12`` — с номером файла ``sample=``
    #: Файлы ``loop()`` от корня сэмплов (``dj_dave/...``): владелец плеера сверяет их с диском до exec (I15, §3.11).
    sample_files: FrozenSet[str] = frozenset()


__all__ = ["Program"]
