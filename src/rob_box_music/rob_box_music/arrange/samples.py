"""Сэмплы DJ_Dave в треке (ADR-0149 §3.11; PR-3d) — приёмами самой DJ_Dave (Strudel «Array», «By Design»).

* ``sample`` — перкуссия psr: на КАЖДУЮ 16-ю случайный файл из пула (``s("psr:[2|5|…]").fast(16)``),
  детерминированно по сиду трека, под той же сайдчейн-огибающей, что пэд и бас (``knowledge.DUCK_ROLES``).
  Пул трека — ``POOL_SIZE`` файлов: ядро — список psr из её кода (``knowledge.DAVE_PSR``), добор — короткие
  удары каталога; штраф за недавнее — по колонке ``perc`` истории. Звучит в build, дропах и break.
* ``loop`` — брейк-луп как часть грува (интервью: «ударный стек — клэп + брейк-луп»): ``loopAt(l).chop(l*8)
  .legato(1)`` — файл режется на восьмые, каждый кусок перезапускается на своей доле (не один ``loop`` с
  ``beat_stretch``: луп не уплывает по фазе). Звучит только во втором дропе: слой, которым второй дроп плотнее первого (пик трека).
* ``fx`` — одиночный крэш/хит на первой доле дропов: не в intro/outro — в блэнде PR-8 они звучат на двух деках.

Синт ``loop`` Renardo — ``PlayBuf(loop: 1)`` на всю ``sus``: у удара ``sus`` ≤ длины файла (звучит один раз,
см. ``render.renardo``). Уровень — ``arrange.mix`` по модели громкости (``Style.role_level_db``).
Тональные банки (build/drop-вокал Array, вокал-чопы) в трек не идут: тональность у них не размечена, на чужой
тонике они спорят с басом (кроме spilltab в A# minor). Моно и mp3 звучат (проба 02.10).
"""

from __future__ import annotations

import random
from typing import List, Mapping, Sequence, Tuple

from .. import knowledge as kn
from ..diversity import recent_values, weighted_pick
from ..model import STEPS_PER_BAR, Key, Part
from . import rhythm

LOOP_ROLES: Tuple[str, ...] = ("loop",)
FX_ROLES: Tuple[str, ...] = ("fx",)
#: Длина удара psr-слоя, с: короче 60 мс огибающая ``loop`` (50 мс атака) не открывается, длиннее 0.3 с — не щелчок.
TICK_SECONDS = (0.06, 0.3)
#: Файлов в пуле трека и длина случайной последовательности (16-х; Renardo зацикливает ``buf`` по событию).
POOL_SIZE = 8
SEQUENCE_STEPS = 2 * STEPS_PER_BAR
#: Кусок нарезки лупа — восьмая (``chop(l*8)`` на такт), в шагах 16-х.
CHOP_STEPS = 2
#: Уровень партии до ``arrange.mix.mix_parts`` (он ставит уровень роли по модели громкости).
_UNLEVELED = 0.0
#: Память оси «сэмплы»: затухание штрафа и глубина истории (у остальных осей ``DEFAULT_DECAY``).
DECAY = 0.9
HISTORY = 50


def usable(info: kn.SampleInfo, key: Key) -> bool:
    """Нетональный (либо в тональности трека); удар psr-слоя — щелчок длиной ``TICK_SECONDS``."""
    tick = info.role != "perc" or TICK_SECONDS[0] <= info.seconds <= TICK_SECONDS[1] or (
        info.name in kn.DAVE_PSR and info.seconds >= TICK_SECONDS[0])  # её psr — любой длины: ``sus`` режет до 16-й
    return tick and (not info.tonal or info.key == (key.root, key.mode))


def pool(roles: Sequence[str], key: Key) -> List[str]:
    return sorted(n for n, i in kn.SAMPLE_CATALOG.items() if i.role in roles and usable(i, key))


def pick(roles: Sequence[str], key: Key, history: Sequence[Mapping], field: str, rng: random.Random) -> str:
    """Сэмпл роли со штрафом за недавнее по колонке ``field`` истории (свежие первыми)."""
    return weighted_pick(pool(roles, key), recent_values(history[:HISTORY], field), rng, decay=DECAY)


def perc_pool(key: Key, history: Sequence[Mapping], rng: random.Random) -> Tuple[str, ...]:
    """``POOL_SIZE`` ударов трека: psr DJ_Dave первыми в очереди, штраф за недавние пулы (колонка ``perc``)."""
    recent = [name for row in history[:HISTORY] for name in (row.get("perc") or "").split(",") if name]
    options = [n for n in pool(("perc",), key)]
    chosen: List[str] = []
    for _ in range(min(POOL_SIZE, len(options))):
        dave = [n for n in options if n in kn.DAVE_PSR and n not in chosen]
        rest = [n for n in options if n not in chosen]
        chosen.append(weighted_pick(dave if len(chosen) < POOL_SIZE // 2 and dave else rest, recent, rng,
                                    decay=DECAY))
    return tuple(sorted(chosen))


def perc_part(style: kn.Style, names: Sequence[str], kit: str, rng: random.Random) -> Part:
    """psr-слой: удар на каждую 16-ю, файл — случайный из пула на каждый шаг (``SEQUENCE_STEPS``, по кругу; каждый
    файл пула звучит хотя бы раз), акцент на шагах рисунка ``perc`` каркаса ``kit`` стиля, остальные тише."""
    accents = style.kits[kit]["perc"]
    grid = rhythm.grid(range(STEPS_PER_BAR), accents={i: 3 if ch == "x" else 1 for i, ch in enumerate(accents)})
    sequence = list(names) + [rng.choice(list(names)) for _ in range(SEQUENCE_STEPS - len(names))]
    rng.shuffle(sequence)
    level = sorted(names, key=lambda n: kn.SAMPLE_CATALOG[n].mean_db)[len(names) // 2]  # уровень — по медиане
    return Part("sample", level, grid, None, _UNLEVELED, (0, 0), pool=tuple(sequence))


def loop_part(name: str) -> Part:
    """Брейк-луп нарезкой: кусок — восьмая, все куски файла подряд (длина — ``SampleInfo.beats``)."""
    span = int(kn.SAMPLE_CATALOG[name].beats or 4) * 4
    length = max(span, STEPS_PER_BAR)
    return Part("loop", name, rhythm.grid(range(0, length, CHOP_STEPS), length, accents={}), None, _UNLEVELED, (0, 0))


def fx_part(name: str, section_bars: int) -> Part:
    """``fx``: один удар на первой доле каждой секции, где роль звучит."""
    return Part("fx", name, rhythm.grid([0], section_bars * STEPS_PER_BAR), None, _UNLEVELED, (0, 0))


__all__ = ["CHOP_STEPS", "DECAY", "FX_ROLES", "HISTORY", "LOOP_ROLES", "POOL_SIZE", "SEQUENCE_STEPS",
           "TICK_SECONDS", "fx_part", "loop_part", "perc_part", "perc_pool", "pick", "pool", "usable"]
