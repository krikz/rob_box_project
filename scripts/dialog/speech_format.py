"""Оформление речи кодом - эксперимент консоли 29.10 (только стенд, робот не меняется).

Модель пишет голый текст (``speech_text.PLAIN``). Экран оформляет :func:`screen` (тире прямой речи у героя - по голосу
роли, его дал код). Голос оформляет КАЖДЫЙ провайдер сам (:class:`MiniMax`, :class:`Yandex`, :class:`Silero`): он
знает, что его сервис умеет без нас, и доделывает только остальное. Общее у всех одно - паузы рэпа по сетке такта
(``rap_pauses``): такт знает только робот; провайдер лишь пишет паузу своей меткой.

Что сервис умеет сам (флаги класса), и откуда это известно:
- Silero: ударения и ё ставит модель (``put_accent``/``put_yo`` в tts_node, включены); цифр не читает - нужен
  нормализатор (по памяти о Silero, не мерено);
- Yandex: ударение понимает знаком ``+`` перед гласной (TTS-разметка v3, документация Yandex); цифры, по памяти,
  читает сам - не мерено;
- MiniMax: пауза ``<#x#>`` (рэп на роботе); нормализация - поле ``text_normalization`` (на роботе не задано);
  ``+`` не документирован - не шлём.
Длительности пауз посчитаны по темпу MiniMax (``SYLLABLES_PER_S``) - для других голосов не мерены.
Пакеты нормализатора и ударений необязательны: нет пакета - шаг пропущен, и это видно в строке шагов.
"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence
from xml.sax.saxutils import escape

from rob_box_dialog.plan import rap_pauses as rp

NORMALIZERS = ("none", "rutextnorm", "ru-normalizr")
DASH = "—"

_PAUSE_TAG_RE = re.compile(r"\s*<#(\d+(?:\.\d+)?)#>\s*")
_BREAK_RE = re.compile(r'(<break time="\d+ms"/>)')
#: Слово с одной гласной и ``+`` перед ней («+я», «н+а», «н+е»): silero-stress ставит ударение и в предлоги, а Yandex
#: по ``+`` выделяет слово голосом - «н+а в+етке» звучит с ударным предлогом. Знак снимаем.
_SINGLE_VOWEL_STRESS_RE = re.compile(r"(?<![\w+])[^\W\dаеёиоуыэюяАЕЁИОУЫЭЮЯ_]*\+[аеёиоуыэюяАЕЁИОУЫЭЮЯ]"
                                     r"[^\W\dаеёиоуыэюяАЕЁИОУЫЭЮЯ_]*(?![\w+])")

Fn = Callable[[str], str]


@dataclass
class Step:
    """Шаг оформления и сколько он занял (мс); ``skipped`` - почему шаг пропущен."""

    name: str
    ms: Optional[float] = None
    skipped: str = ""


@dataclass
class Voiced:
    """Что ушло бы в синтез провайдеру и какими шагами."""

    provider: str
    text: str
    steps: List[Step] = field(default_factory=list)


def screen(text: str, voice: str) -> str:
    """Реплика на экран: у героя (голос роли) - тире прямой речи; модель тире не пишет, его ставит код по голосу."""
    return f"{DASH} {text}" if voice else text


@dataclass
class Tools:
    """Пакеты оформления, загруженные один раз: функция или ``None`` и почему её нет."""

    normalize: Optional[Fn] = None
    normalize_why: str = "выключен"
    normalizer_name: str = "none"
    stress: Optional[Fn] = None
    stress_why: str = "выключены"


class Provider:
    """Провайдер голоса оформляет текст для своего сервиса сам; флаги - что сервис делает без нас."""

    name = ""
    reads_digits = True    # сервис сам читает цифры и знаки
    sets_stress = True     # сервис сам ставит ударения (или они ему не нужны)

    def __init__(self, tools: Tools) -> None:
        self.tools = tools

    def voice(self, text: str, bar_s: float = 0.0) -> Voiced:
        steps: List[Step] = []
        if self.reads_digits and not (bar_s > 0 and "\n" in text):
            steps.append(Step("нормализатор", skipped="сервис читает сам"))
        else:  # такт считает длину строки по гласным: в «2024» их нет - числа словами нужны и тому, кто их читает
            text = _timed(steps, f"нормализатор {self.tools.normalizer_name}", self.tools.normalize,
                          self.tools.normalize_why, text)
        if self.sets_stress:
            steps.append(Step("ударения", skipped="сервис ставит сам или не понимает +"))
        else:
            text = _timed(steps, "ударения", self.tools.stress, self.tools.stress_why, text)
        if bar_s > 0 and "\n" not in text:
            steps.append(Step("паузы такта", skipped="не куплет (нет строк)"))
        elif bar_s > 0:  # на роботе такт - только у рэпа под подложку; в превью - у куплета со строками
            mark = rp.markup(text, bar_s)
            steps.append(Step("паузы такта", skipped="" if mark else "одна строка"))
            if mark:
                text = _PAUSE_TAG_RE.sub(lambda m: self.pause(float(m.group(1))), mark.text).strip()
        return Voiced(self.name, self.wrap(text), steps)

    def pause(self, seconds: float) -> str:
        raise NotImplementedError

    def wrap(self, text: str) -> str:
        return text


class MiniMax(Provider):
    name = "minimax"

    def pause(self, seconds: float) -> str:
        return f" {rp.pause_tag(seconds)} "


class Yandex(Provider):
    name = "yandex"
    sets_stress = False

    def voice(self, text: str, bar_s: float = 0.0) -> Voiced:
        voiced = super().voice(text, bar_s)
        voiced.text = _SINGLE_VOWEL_STRESS_RE.sub(lambda m: m.group(0).replace("+", ""), voiced.text)
        return voiced

    def pause(self, seconds: float) -> str:
        return f" sil<[{int(round(seconds * 1000))}]> "


class Silero(Provider):
    name = "silero"
    reads_digits = False

    def pause(self, seconds: float) -> str:
        return f' <break time="{int(round(seconds * 1000))}ms"/> '

    def wrap(self, text: str) -> str:
        body = "".join(p if _BREAK_RE.fullmatch(p) else escape(p) for p in _BREAK_RE.split(text))
        return f"<speak>{body}</speak>"


PROVIDER_CLASSES = (MiniMax, Yandex, Silero)
PROVIDERS = tuple(cls.name for cls in PROVIDER_CLASSES)


def _timed(steps: List[Step], name: str, fn: Optional[Fn], why: str, text: str) -> str:
    if fn is None:
        steps.append(Step(name, skipped=why))
        return text
    start = time.perf_counter()
    out = fn(text)
    steps.append(Step(name, ms=(time.perf_counter() - start) * 1000))
    return out


def load_tools(normalizer: str = "none", stress: bool = True) -> Tools:
    """Пакеты оформления: нормализатор по имени и ударения silero-stress; нет пакета - причина в ``*_why``."""
    tools = Tools(normalizer_name=normalizer)
    if normalizer not in NORMALIZERS:
        raise ValueError(f"нормализатор {normalizer!r}: есть {', '.join(NORMALIZERS)}")
    if normalizer != "none":
        try:
            tools.normalize, tools.normalize_why = _NORMALIZER_LOADERS[normalizer](), ""
        except ImportError as exc:
            tools.normalize_why = f"нет пакета ({exc.name})"
    if stress:
        try:
            from silero_stress import load_accentor  # noqa: PLC0415 — тяжёлый пакет (torch), необязателен
        except ImportError as exc:
            tools.stress_why = f"нет пакета ({exc.name})"
        else:
            tools.stress, tools.stress_why = load_accentor(), ""
    return tools


def _rutextnorm() -> Fn:
    from rutextnorm import normalize_russian  # noqa: PLC0415 — один файл, правила-регулярки, MIT

    return normalize_russian


def _ru_normalizr() -> Fn:
    from ru_normalizr import Normalizer  # noqa: PLC0415 — pymorphy3 + num2words, MIT

    return Normalizer().normalize


_NORMALIZER_LOADERS: Dict[str, Callable[[], Fn]] = {"rutextnorm": _rutextnorm, "ru-normalizr": _ru_normalizr}


def providers(tools: Tools, names: Sequence[str] = PROVIDERS) -> List[Provider]:
    by_name = {cls.name: cls for cls in PROVIDER_CLASSES}
    unknown = [name for name in names if name not in by_name]
    if unknown:
        raise ValueError(f"провайдер голоса {', '.join(unknown)}: есть {', '.join(PROVIDERS)}")
    return [by_name[name](tools) for name in names]


def steps_line(steps: Sequence[Step]) -> str:
    return ", ".join(f"{s.name}: {s.skipped}" if s.skipped else s.name if s.ms is None else f"{s.name} {s.ms:.0f} мс"
                     for s in steps) or "-"


__all__ = ["DASH", "MiniMax", "NORMALIZERS", "PROVIDERS", "Provider", "Silero", "Step", "Tools", "Voiced", "Yandex",
           "load_tools", "providers", "screen", "steps_line"]
