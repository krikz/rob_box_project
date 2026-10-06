"""Реплика диджея на переходе: факты трека, шаблоны, схема и валидатор фразы LLM (ADR-0149 §12 В2, решение Шифу 06.10).

Чистая часть — без сети и ROS; кто и когда спрашивает LLM и говорит — ``rob_box_mcp_tools.engine.dj_lines``.

Факты решает код (ADR-0148): номер трека и длина сета, тема, человеческое название мелодии-хука трека, фаза дуги
энергии (:class:`LineFacts`). LLM только раскрашивает их в персоне диджея, и названия и числа она НЕ пишет: в её
строке на их месте подстановки ``{hook}``, ``{no}``, ``{total}``, ``{theme}``, которые заполняет код
(:func:`validate_line`). Цифра или известное название (мелодии сета, части темы) буквами в тексте LLM — фраза
отвергнута: так LLM не может назвать мелодию, которая не играет («Марио реально играет» 06.10), или соврать номером.
Отвергнута, опоздала, LLM недоступна — шаблон кода с теми же фактами (:func:`template_line`), никогда не тишина.
"""

from __future__ import annotations

import re
import string
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

#: Длина фразы LLM (схема) — короткий выкрик поверх музыки, не монолог.
LINE_MAX = 80
SUBMIT_TOOL = "submit_dj_line"
#: Подстановки, которыми LLM ссылается на факты; значения вставляет код.
FIELDS = ("hook", "no", "total", "theme")
#: Название короче — не сверяется (слишком много случайных совпадений с обычными словами).
NAME_MIN = 4
#: Тема длиннее — не подставляется во фразу (06.10 тема была в два предложения): шаблон без ``{theme}``.
THEME_MAX = 32
#: Служебные слова архивных названий («Super Mario Bros Theme»): не название, их LLM писать можно («супер»).
GENERIC_WORDS = frozenset({"super", "theme", "song", "music", "main", "title", "intro", "remix", "version", "part",
                           "game", "from", "with", "tema", "pesnia", "muzika"})

#: Фаза трека в дуге сета → как её назвать LLM (факт кода, не выбор LLM).
PHASES: Mapping[str, str] = {
    "opening": "первый трек сета", "rise": "подъём энергии", "peak": "кульминация сета",
    "down": "спад энергии", "steady": "держим волну", "final": "финальный трек сета",
}

#: Шаблоны по фазе: (с мелодией, без мелодии — у трека свой мотив). Выбор — по сиду трека, без повтора подряд.
TEMPLATES: Mapping[str, Tuple[Tuple[str, ...], Tuple[str, ...]]] = {
    "opening": (("Погнали! Трек {no} из {total}, открываем: {hook}.",
                 "Диджей на месте! Первым номером — {hook}.",
                 "Сет «{theme}» начинается: {hook}!"),
                ("Погнали! Сет «{theme}», трек {no} из {total}.",
                 "Диджей на месте! Трек {no} из {total}, качаем.")),
    "rise": (("Поднимаем! Трек {no} из {total} — {hook}.",
              "Громче и выше: дальше {hook}.",
              "Трек {no}: разгоняемся, звучит {hook}."),
             ("Поднимаем! Трек {no} из {total}.",
              "Разгоняемся, трек {no} из {total}.")),
    "peak": (("Кульминация! Трек {no} из {total} — {hook}!",
              "Вот он, пик сета: {hook}!",
              "Руки вверх! На пике — {hook}."),
             ("Кульминация! Трек {no} из {total}!",
              "Пик сета, трек {no} из {total} — руки вверх!")),
    "down": (("Выдыхаем. Трек {no} из {total}: {hook}.",
              "Чуть тише, но не сдаёмся: {hook}.",
              "Плавно вниз, трек {no} — {hook}."),
             ("Выдыхаем. Трек {no} из {total}.",
              "Чуть спокойнее, трек {no} из {total}.")),
    "steady": (("Трек {no} из {total}: {hook}. Держим волну!",
                "Не сбавляем: дальше {hook}.",
                "Едем дальше — {hook}, трек {no}."),
               ("Трек {no} из {total}, держим волну!",
                "Не сбавляем! Трек {no} из {total}.")),
    "final": (("Финальный трек! Провожаем сет: {hook}.",
               "Последний, {no} из {total}: {hook}!",
               "Финал сета — {hook}. Спасибо, что танцуете!"),
              ("Финальный трек! {no} из {total}, провожаем сет.",
               "Последний трек сета — спасибо, что с нами!")),
}


class LineInvalid(ValueError):
    """Фраза LLM не прошла валидатор; ``reason`` — почему (в лог ``line_invalid{reason}``)."""

    def __init__(self, reason: str, message: str) -> None:
        super().__init__(f"{reason}: {message}")
        self.reason = reason


@dataclass(frozen=True)
class LineFacts:
    """Факты перехода — всё, что фраза вправе утверждать. ``tracks=0`` — длина сета не известна; ``hook=None`` —
    у трека нет мелодии из библиотеки (мотив диджея); ``names`` — названия, которые LLM нельзя писать буквами
    (мелодии сета и части темы, включая не найденные)."""

    track_no: int
    tracks: int
    theme: str
    hook: Optional[str]
    energy: int
    prev_energy: Optional[int] = None
    names: Tuple[str, ...] = ()

    @property
    def phase(self) -> str:
        """Фаза дуги: последний трек, первый, пик (5), подъём/спад к прошлому треку, ровно."""
        if self.tracks > 1 and self.track_no >= self.tracks:
            return "final"
        if self.track_no <= 1:
            return "opening"
        if self.energy >= 5:
            return "peak"
        if self.prev_energy is not None and self.energy != self.prev_energy:
            return "rise" if self.energy > self.prev_energy else "down"
        return "steady"

    def values(self) -> Dict[str, str]:
        """Значения подстановок; факта нет — подстановки нет (её использование — ``LineInvalid``)."""
        theme = self.theme.strip()
        out = {"no": str(self.track_no), "theme": theme if len(theme) <= THEME_MAX else ""}
        if self.tracks >= self.track_no:
            out["total"] = str(self.tracks)
        if self.hook:
            out["hook"] = self.hook
        return {k: v for k, v in out.items() if v}


def template_line(facts: LineFacts, seed: int, last: Optional[str] = None) -> str:
    """Шаблон кода по фазе: вариант по ``seed`` трека, не тот же текст, что ``last`` (без повторов подряд). Шаблон
    с подстановкой, которой нет в фактах (мелодия, длина, тема), не берётся."""
    with_hook, without = TEMPLATES[facts.phase]
    values = facts.values()
    variants = [t for t in (with_hook if "hook" in values else without) if _fields(t) <= set(values)]
    variants = variants or [t for t in without if _fields(t) <= set(values)] or ["Трек {no}!"]
    lines = [t.format(**values) for t in variants]
    pick = seed % len(lines)
    if lines[pick] == last and len(lines) > 1:
        pick = (pick + 1) % len(lines)
    return lines[pick]


def _fields(template: str) -> set:
    return {name for _lit, name, _spec, _conv in string.Formatter().parse(template) if name is not None}


def schema() -> Dict[str, Any]:
    """Узкая схема ответа: одна строка ≤ :data:`LINE_MAX`, ничего больше."""
    fields = ", ".join("{" + f + "}" for f in FIELDS)
    line = {"type": "string", "maxLength": LINE_MAX,
            "description": f"фраза диджея по-русски; названия и числа — только подстановками {fields}"}
    return {"type": "object", "properties": {"line": line}, "required": ["line"], "additionalProperties": False}


def tool() -> Dict[str, Any]:
    return {"type": "function", "function": {
        "name": SUBMIT_TOOL, "description": "Отдать одну короткую фразу диджея на переход. Вызвать ровно один раз.",
        "parameters": schema()}}


def prompt(facts: LineFacts, persona: Optional[str] = None) -> Tuple[str, str]:
    """``(system, user)``: факты показаны словами для настроения, писать их — только подстановками."""
    who = f"диджей {persona}" if persona else "диджей робота"
    system = (f"Ты {who}. Скажи одну короткую весёлую фразу на смену трека, по-русски, не длиннее {LINE_MAX} "
              "символов. Названия мелодий, тему и числа НЕ пиши словами и цифрами — только подстановками: {hook} — "
              "мелодия трека, {no} — номер трека, {total} — сколько треков в сете, {theme} — тема сета; код вставит "
              "их сам. Ничего не утверждай о треке, кроме этих фактов; других мелодий не называй. Ответ — один вызов "
              f"{SUBMIT_TOOL}, без текста.")
    values = facts.values()
    hook = f"{{hook}} = «{values['hook']}»" if "hook" in values else "мелодии нет (свой мотив) — {hook} нельзя"
    total = f"{{total}} = {values['total']}" if "total" in values else "длина сета не известна — {total} нельзя"
    user = (f"Фаза: {PHASES[facts.phase]}. {{no}} = {facts.track_no}; {total}; {hook}; "
            f"{{theme}} = «{facts.theme}».")
    return system, user


Fold = Callable[[str], str]
_WORD = re.compile(r"\w+", re.UNICODE)


def plain_fold(text: str) -> str:
    """Свёртка для сверки названий по умолчанию: нижний регистр, ё → е (движок даёт ещё и транслит)."""
    return text.lower().replace("ё", "е")


def _name_keys(names: Iterable[str], fold: Fold) -> Tuple[str, ...]:
    """Ключ названия — начало каждого его значимого слова (окончание срезано: «Аладдина», «Тетрисом» ловятся)."""
    keys = []
    for name in names:
        for word in _words(name):
            if len(word) >= NAME_MIN and not word.isdigit() and word not in GENERIC_WORDS:
                folded = fold(word)
                keys.append(folded[:max(NAME_MIN, len(folded) - 2)])
    return tuple(dict.fromkeys(keys))


def _words(text: str) -> Tuple[str, ...]:
    return tuple(_WORD.findall(plain_fold(text)))


def validate_line(text: Any, facts: LineFacts, fold: Fold = plain_fold) -> str:
    """Строка LLM → готовая фраза с фактами кода, или ``LineInvalid``. Проверки структурные: подстановки только из
    :data:`FIELDS` и только те, чьи факты есть, каждая не больше раза; в собственном тексте LLM нет цифр и нет
    известных названий (:attr:`LineFacts.names` и мелодия трека) — их место только в подстановках."""
    parsed = _parsed(text)
    values = facts.values()
    used = [name for _lit, name, _spec, _conv in parsed if name is not None]
    if any(name not in values for name in used) or len(used) != len(set(used)):
        raise LineInvalid("field", f"подстановки {used}, доступны {sorted(values)}")
    _check_literal(" ".join(lit for lit, _name, _spec, _conv in parsed), facts, fold)
    return text.strip().format(**values)


def _parsed(text: Any) -> List[Tuple[str, Optional[str], Optional[str], Optional[str]]]:
    """Строка LLM → куски ``string.Formatter``; пусто, длинно, сломанные скобки — ``LineInvalid``."""
    if not isinstance(text, str) or not text.strip():
        raise LineInvalid("empty", "нет строки")
    if len(text.strip()) > LINE_MAX:
        raise LineInvalid("length", f"{len(text.strip())} > {LINE_MAX} символов")
    try:
        return list(string.Formatter().parse(text.strip()))
    except ValueError as exc:
        raise LineInvalid("braces", str(exc)) from None


def _check_literal(literal: str, facts: LineFacts, fold: Fold) -> None:
    """Собственный текст LLM: без цифр и без известных названий буквами."""
    if any(ch.isdigit() for ch in literal):
        raise LineInvalid("digits", "числа — только подстановками {no}/{total}")
    words = [fold(w) for w in _words(literal)]
    for key in _name_keys((*facts.names, *([facts.hook] if facts.hook else ())), fold):
        if any(w.startswith(key) for w in words):
            raise LineInvalid("name", f"название буквами ({key}…) — только подстановкой {{hook}}/{{theme}}")


def mentions(name: str, text: str, fold: Fold = plain_fold) -> bool:
    """Называет ли ``text`` мелодию ``name`` (тот же ключ, что у валидатора: начало значимого слова, падеж не
    важен) — «когда будет Марио?» против «Super Mario Bros» и «Марио»."""
    words = [fold(w) for w in _words(text)]
    return any(w.startswith(key) for key in _name_keys([name], fold) for w in words)


def tracks_word(n: int) -> str:
    """«трек/трека/треков» к числу ``n``."""
    if n % 10 == 1 and n % 100 != 11:
        return "трек"
    return "трека" if 2 <= n % 10 <= 4 and not 12 <= n % 100 <= 14 else "треков"


def _quoted(names: Sequence[str]) -> str:
    return ", ".join(f"«{n}»" for n in names)


def now_playing_text(facts: Mapping[str, Any]) -> str:
    """«Что играет» из фактов снимка сета (``dj``: ``track_no``, ``tracks``, ``melody``, ``next_melodies``,
    ``not_found``) — фразу строит код (ADR-0148), не LLM."""
    no, total, melody = facts.get("track_no"), facts.get("tracks"), facts.get("melody")
    head = f"Сейчас трек {no} из {total}" if total else f"Сейчас трек {no}"
    parts = [f"{head}: «{melody}»." if melody else f"{head}: свой мотив диджея, без мелодии из библиотеки."]
    if facts.get("next_melodies"):
        parts.append(f"Дальше по плану: {_quoted(facts['next_melodies'])}.")
    if facts.get("not_found"):
        parts.append(f"Не нашлось в библиотеке: {_quoted(facts['not_found'])}.")
    return " ".join(parts)


def when_text(facts: Mapping[str, Any], asked: str, fold: Fold = plain_fold) -> Optional[str]:
    """«Когда будет / где X» по фактам сета: играет сейчас, через сколько треков по плану, не нашлось в библиотеке.
    X не совпал ни с одним фактом — ``None`` (кода сказать нечего, вопрос уходит LLM)."""
    melody = facts.get("melody")
    if melody and mentions(melody, asked, fold):
        return f"«{melody}» играет прямо сейчас — трек {facts.get('track_no')} из {facts.get('tracks')}."
    for i, name in enumerate(facts.get("next_melodies") or (), start=1):
        if mentions(name, asked, fold):
            return f"«{name}» по плану через {i} {tracks_word(i)}."
    for name in facts.get("not_found") or ():
        if mentions(name, asked, fold):
            return f"«{name}» в библиотеке мелодий не нашлось — в этом сете не будет. {now_playing_text(facts)}"
    return None


def payload_line(payload: Any) -> Any:
    """Строка из аргументов ``submit_dj_line`` (лишние поля — нет)."""
    if not isinstance(payload, Mapping) or set(payload) != {"line"}:
        raise LineInvalid("schema", f"ожидался объект {{line}}, пришло {payload!r}"[:120])
    return payload["line"]


def facts_for(track_no: int, tracks: int, theme: str, hook: Optional[str], energies: Sequence[int],
              names: Iterable[str] = ()) -> LineFacts:
    """Факты трека из плана сета: ``energies`` — энергия треков 1..N по плану (прошлый трек — сосед слева)."""
    energy = energies[track_no - 1] if 0 < track_no <= len(energies) else 0
    prev = energies[track_no - 2] if 1 < track_no <= len(energies) + 1 else None
    return LineFacts(track_no, tracks, theme, hook, energy, prev, tuple(n for n in names if n))


__all__ = ["FIELDS", "LINE_MAX", "LineFacts", "LineInvalid", "PHASES", "SUBMIT_TOOL", "TEMPLATES", "facts_for",
           "mentions", "now_playing_text", "payload_line", "plain_fold", "prompt", "schema", "template_line", "tool",
           "tracks_word", "validate_line", "when_text"]
