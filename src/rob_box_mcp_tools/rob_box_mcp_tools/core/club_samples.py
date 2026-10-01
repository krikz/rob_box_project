"""club_samples.py — слой сэмплов DJ_Dave в club-треке (issue #3254, ADR-0146).

Приёмка #3219 провалилась на слух (Шифу, 01.10): DJ-сеты играют только
``compose_music style=club``, а club-генератор (:mod:`core.club_arranger`)
собирал трек из синтов и штатных барабанов ``play()`` — сэмплы пака
DJ_Dave (:mod:`core.sample_dave`) доходили до звука лишь через ``loop()`` в
``execute_music_code``, который DJ-ход модели запрещён. ``groove_loop``
(#2841) тоже не помог бы: это ручка classic-аранжировщика
(:func:`core.arranger._add_loop_layer`), в club её нет.

Здесь — ещё одна ось выбора club (как каркас и пулы :mod:`core.club_pools`):
какой сэмпл-слой из белого списка играет в треке. Выбор —
:func:`core.music_diversity.weighted_pick` со штрафом за недавнее (колонка
``sample`` в ``music_history``).

Слот. Санитайзер даёт ровно шесть плееров (d1-d3/p1-p3), и все шесть club
занимает. Сэмпл-слой встаёт в ``d3`` ВМЕСТО клэпа с открытым хэтом (так и
эталон «By Design»: d3 отдан psr). Гейт секций — слой ``perc`` шаблона
матрицы: он есть во всех club-шаблонах, но до сих пор не рендерился (не
было слота). В треке со слоем сэмплов клэпа нет — в историю ``clap`` не
пишется (см. :func:`core.club_history.remember_club`).

Громкость. Модель :mod:`core.club_loudness` сэмплов не знает (снята
NRT-рендером синтов и ``0_foxdot_default``). Уровень слоя — доля
(:attr:`SampleLayer.rel`) от ОСНОВНОГО уровня бочки каркаса после
калибровки, то есть множитель стиля/каркаса club_loudness применяется и к
сэмплу; доли взяты из эталона «By Design» (psr 0.3 при бочке 0.7). На
роботе громкость сэмплов НЕ замерена.

Тональность. Ударные/брейки неголосовые — с любой тоникой. Вокальный чоп
``algorave_spilltab`` в оригинале звучит в A# minor (эталон «By Design»,
``root=10``), чоп по долям (``pos``) высоту не меняет — поэтому он в пуле
только при тонике A#.

Честная деградация: флаг ``ROB_BOX_PACK1_LOOPS`` выключен или файла нет на
диске — слоя нет, причина в :attr:`SamplePick.reason` (лог и ответ тула).
"""

from __future__ import annotations

import os
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence, Tuple

from rob_box_music import knowledge as kn

from . import sample_dave
from .club_stereo import PAN_HATS, pan_alternating, pan_list
from .music_diversity import weighted_pick
from .sample_loops import PACK1_LOOPS_ENV, pack1_loops_enabled

__all__ = [
    "SAMPLE_LAYERS",
    "SAMPLE_SLOT",
    "SampleLayer",
    "SamplePick",
    "pick_club_sample",
    "sample_layer_line",
    "sample_sentence",
]

#: Слот слоя сэмплов (вместо клэпа) и слой матрицы, чей гейт он берёт.
SAMPLE_SLOT = "d3"
SAMPLE_LANE = "perc"

#: Корень сэмплов Renardo внутри контейнеров (пак dj_dave смонтирован туда же).
_SAMPLES_ROOT_ENV = "RENARDO_SAMPLES_PATH"
_SAMPLES_ROOT_DEFAULT = "/root/.config/renardo/samples"


@dataclass(frozen=True)
class SampleLayer:
    """Вариант слоя.

    Attributes:
        sample: имя из белого списка :mod:`core.sample_dave`.
        kind: ``step`` — короткий удар на каждую 16-ю с пампингом бочки;
            ``stretch`` — луп, растянутый ``beat_stretch=1`` на ``beats``
            долей (темп трека); ``chop`` — фраза кусками по доле (``pos``),
            высота не меняется.
        beats: длина одного проигрыша (``stretch``) или число кусков (``chop``).
        rel: уровень пика слоя как доля основного уровня бочки каркаса.
        roots: тоники, при которых вариант в пуле (``()`` — любые).
    """

    sample: str
    kind: str
    beats: int
    rel: float
    roots: Tuple[str, ...] = ()


#: Роль сэмпла в каталоге → вид слоя.
_KIND_OF_ROLE = {"perc": "step", "loop": "stretch", "vox": "chop"}


def _layer(sample: str, rel: float) -> SampleLayer:
    """Слой из ``knowledge.SAMPLE_CATALOG``: вид — по роли, доли — по темпу оригинала, тоники — по тональности
    (ADR-0149 PR-3d: метаданные сэмпла живут в одной таблице)."""
    info = kn.SAMPLE_CATALOG[sample]
    roots = (kn.ROOTS[info.key[0]],) if info.key else ()
    return SampleLayer(sample, _KIND_OF_ROLE[info.role], info.beats or 1, rel, roots)


#: Белый список слоя. Выбор: короткая перкуссия psr (номера из списка
#: оригинала «By Design» ``psr:[2|5|6|...|29]``), шейкер и брейк Array
#: (140 bpm в оригинале), бит whatuneed (8 долей при 128 bpm) и вокальный
#: чоп spilltab (A# minor). Длинные вокальные фразы Array/algorave с неизвестной
#: тональностью сюда НЕ взяты — на чужой тонике они бы спорили с басом.
SAMPLE_LAYERS: Mapping[str, SampleLayer] = {
    "psr_10": _layer("dirt_psr_10", 0.43),
    "psr_06": _layer("dirt_psr_06", 0.43),
    "psr_25": _layer("dirt_psr_25", 0.43),
    "array_shaker": _layer("array_perc_shaker", 0.4),
    "array_break": _layer("array_perc_break", 0.5),
    "wun_beat": _layer("algorave_wun_beat", 0.5),
    "spilltab_chop": _layer("algorave_spilltab", 0.43),
}


@dataclass(frozen=True)
class SamplePick:
    """Итог выбора: ``name=None`` — слоя нет, ``reason`` — почему."""

    name: Optional[str]
    reason: str

    def info(self) -> dict:
        """Для ``data`` ответа тула и лога."""
        layer = SAMPLE_LAYERS.get(self.name or "")
        return {
            "layer": self.name,
            "sample": layer.sample if layer else None,
            "path": sample_dave.find_sample(layer.sample).path if layer else None,
            "slot": SAMPLE_SLOT if layer else None,
            "reason": self.reason,
        }


def _samples_root(samples_root: Optional[str]) -> Path:
    return Path(samples_root or os.environ.get(_SAMPLES_ROOT_ENV, _SAMPLES_ROOT_DEFAULT))


def _file_on_disk(layer: SampleLayer, root: Path) -> Path:
    """Путь до файла сэмпла от корня сэмплов (``../../dj_dave/x`` → ``dj_dave/x``)."""
    return root / sample_dave.find_sample(layer.sample).path.replace("../../", "", 1)


def _pool(root_note: str) -> list:
    return [name for name, layer in SAMPLE_LAYERS.items() if not layer.roots or root_note in layer.roots]


def pick_club_sample(
    seed: int, recent: Sequence[Mapping[str, Any]], root_note: Optional[str] = "A#",
    enabled: Optional[bool] = None, samples_root: Optional[str] = None,
) -> SamplePick:
    """Выбрать слой сэмплов трека (или честно объяснить, почему его нет).

    ``seed=0`` — эталон без сэмплов (снимок seed=0 не меняется). Иначе —
    ``weighted_pick`` по пулу тоники со штрафом за недавнее (``recent`` —
    история, свежие первыми), свой ГСЧ ``club-sample:<seed>``.
    """
    if seed == 0:
        return SamplePick(None, "seed=0 — эталон без сэмплов")
    if not (pack1_loops_enabled() if enabled is None else enabled):
        return SamplePick(None, f"пак DJ_Dave выключен (флаг {PACK1_LOOPS_ENV})")
    pool = _pool(root_note or "A#")
    name = weighted_pick(pool, [r.get("sample") for r in recent], random.Random(f"club-sample:{seed}"))
    path = _file_on_disk(SAMPLE_LAYERS[name], _samples_root(samples_root))
    if not path.is_file():
        return SamplePick(None, f"файла {path} нет на диске (Ресурсный пак dj-dave-samples не разложен)")
    return SamplePick(name, "выбран из пула со штрафом за недавнее")


#: ``pan`` слоя-«step»: удар на каждую 16-ю, стороны чередуются (с левой, зеркально хэтам).
_STEP_PAN = pan_list("x" * 16, PAN_HATS, -1)
#: ``pan`` слоёв «stretch»/«chop»: одно событие = лупа/доля, стороны чередуются (с левой).
_ALT_PAN = pan_alternating(PAN_HATS, mirror=True)


def _step_line(layer: SampleLayer, gate: str, pump: str) -> str:
    return f"{SAMPLE_SLOT} >> loop({layer.sample!r}, dur=1/4, pan={_STEP_PAN}, amp={gate}, amplify={pump})"


def _stretch_line(layer: SampleLayer, gate: str, pump: str) -> str:
    return f"{SAMPLE_SLOT} >> loop({layer.sample!r}, dur={layer.beats}, beat_stretch=1, pan={_ALT_PAN}, amp={gate})"


def _chop_line(layer: SampleLayer, gate: str, pump: str) -> str:
    pos = ", ".join(str(i) for i in range(layer.beats))
    return f"{SAMPLE_SLOT} >> loop({layer.sample!r}, P[{pos}], dur=1, pan={_ALT_PAN}, amp={gate})"


_LINES = {"step": _step_line, "stretch": _stretch_line, "chop": _chop_line}


def sample_layer_line(name: str, gate: str, pump: str) -> str:
    """Строка плеера слоя: ``d3 >> loop('<имя>', ...)`` (путь подставит санитайзер)."""
    layer = SAMPLE_LAYERS[name]
    return _LINES[layer.kind](layer, gate, pump)


def sample_peak(name: str, kick_main: float, pump_high: float, cap: float) -> float:
    """Уровень гейта слоя: ``rel`` × основной уровень бочки (у ``step`` — до пампинга)."""
    layer = SAMPLE_LAYERS[name]
    level = layer.rel * kick_main / (pump_high if layer.kind == "step" else 1.0)
    return min(cap, level)


def sample_sentence(info: Optional[Mapping[str, Any]]) -> str:
    """Фраза для ответа compose_music: какой сэмпл-слой звучит (или почему нет)."""
    if not info:
        return ""
    if info.get("layer"):
        return (
            f" Слой сэмплов DJ_Dave: {info['layer']} ({info['sample']}) в {info['slot']} "
            "вместо клэпа, по секциям перкуссии."
        )
    return f" Слоя сэмплов DJ_Dave нет: {info.get('reason')}."
