#!/usr/bin/env python3
"""
tts_text_guard.py — Unicode-script guard for TTS text (issue #1709).

Проблема (live 28.08, «робот бормотал на хинди»)
-------------------------------------------------
LLM периодически вставляет в ``speak_text`` фрагменты на неподдерживаемых
письменностях (иероглифы CJK, деванагари, арабица). TTS-провайдеры
(Yandex gRPC v3 / Silero / MiniMax) читают такой текст как попало: юзер
слышит невнятное бормотание, а в логах ``mcp_server`` от чанка оставался
только ``speech_id`` — восстановить, ЧТО именно было произнесено, было
невозможно.

Что делает модуль
-----------------
1. ``analyze(text)`` — считает долю букв «чужих» письменностей от общего
   числа букв текста (не-буквы — пробелы, цифры, пунктуация, эмодзи —
   не участвуют в расчёте).
2. ``should_skip(text)`` — True, если доля превышает
   ``DEFAULT_FOREIGN_SCRIPT_THRESHOLD`` (10%, см. acceptance issue #1709).
   Такой чанк НЕ отправляется в TTS: вместо бормотания пишем warning с
   ПОЛНЫМ текстом.
3. ``describe(report)`` — однострочная диагностика для лога
   (доля, найденные письменности, сэмпл символов).

Разрешены **кириллица и латиница** (русский + английский + транслит).
Всё остальное алфавитное — «чужая» письменность. Один-два символа в
длинной русской фразе (например «идиома звучит как 你好») дают долю
ниже порога и НЕ блокируют произношение — блокируется только текст,
который в основном состоит из нечитаемых символов.

Чистый Python без ROS-зависимостей — модуль импортируют и
``rob_box_voice`` (``tts_node``), и ``rob_box_mcp_tools``
(``SpeakTextTool``), как ``tts_voice_registry`` (issue #1219).
"""

from __future__ import annotations

from typing import List, NamedTuple, Tuple

#: Доля букв чужих письменностей, при превышении которой чанк не
#: произносится (acceptance issue #1709: «если чанк содержит >10%
#: non-cyrillic+non-latin — лог warning + пропустить»).
DEFAULT_FOREIGN_SCRIPT_THRESHOLD: float = 0.10

#: Разрешённые диапазоны букв: базовая латиница, латиница с диакритикой
#: (Latin-1 Supplement / Extended-A / Extended-B) и кириллица
#: (основной блок + Cyrillic Supplement).
_ALLOWED_RANGES: Tuple[Tuple[int, int], ...] = (
    (0x0041, 0x005A),  # A-Z
    (0x0061, 0x007A),  # a-z
    (0x00C0, 0x00FF),  # Latin-1 Supplement letters
    (0x0100, 0x017F),  # Latin Extended-A
    (0x0180, 0x024F),  # Latin Extended-B
    (0x0400, 0x04FF),  # Cyrillic
    (0x0500, 0x052F),  # Cyrillic Supplement
)

#: Диапазоны чужих письменностей с человекочитаемыми именами. Список не
#: обязан быть исчерпывающим: символ, не попавший ни в разрешённые, ни в
#: эти диапазоны, помечается как ``other`` (и всё равно считается чужим).
_FOREIGN_RANGES: Tuple[Tuple[int, int, str], ...] = (
    (0x0370, 0x03FF, "greek"),
    (0x1F00, 0x1FFF, "greek"),
    (0x0530, 0x058F, "armenian"),
    (0x0590, 0x05FF, "hebrew"),
    (0x0600, 0x06FF, "arabic"),
    (0x0750, 0x077F, "arabic"),
    (0x0700, 0x074F, "syriac"),
    (0x0900, 0x097F, "devanagari"),
    (0x0980, 0x09FF, "bengali"),
    (0x0A00, 0x0A7F, "gurmukhi"),
    (0x0A80, 0x0AFF, "gujarati"),
    (0x0B00, 0x0B7F, "oriya"),
    (0x0B80, 0x0BFF, "tamil"),
    (0x0C00, 0x0C7F, "telugu"),
    (0x0C80, 0x0CFF, "kannada"),
    (0x0D00, 0x0D7F, "malayalam"),
    (0x0E00, 0x0E7F, "thai"),
    (0x0F00, 0x0FFF, "tibetan"),
    (0x1000, 0x109F, "myanmar"),
    (0x10A0, 0x10FF, "georgian"),
    (0x1200, 0x137F, "ethiopic"),
    (0x13A0, 0x13FF, "cherokee"),
    (0x3040, 0x309F, "hiragana"),
    (0x30A0, 0x30FF, "katakana"),
    (0x3130, 0x318F, "hangul"),
    (0x1100, 0x11FF, "hangul"),
    (0xAC00, 0xD7AF, "hangul"),
    (0x3400, 0x4DBF, "cjk"),
    (0x4E00, 0x9FFF, "cjk"),
    (0xF900, 0xFAFF, "cjk"),
    (0x20000, 0x2FA1F, "cjk"),
)

#: Сколько чужих символов показывать в диагностике (лог не должен пухнуть).
_SAMPLE_LIMIT: int = 12


class ForeignScriptReport(NamedTuple):
    """Результат анализа текста.

    * ``total_letters`` — сколько всего букв в тексте (не-буквы не считаются);
    * ``foreign_letters`` — сколько из них принадлежит чужим письменностям;
    * ``ratio`` — ``foreign_letters / total_letters`` (0.0 если букв нет);
    * ``scripts`` — отсортированный кортеж имён найденных письменностей;
    * ``sample`` — первые несколько чужих символов (для лога).
    """

    total_letters: int
    foreign_letters: int
    ratio: float
    scripts: Tuple[str, ...]
    sample: str


def _in_ranges(code: int, ranges) -> bool:
    for start, end in ranges:
        if start <= code <= end:
            return True
    return False


def is_allowed_letter(ch: str) -> bool:
    """True, если символ — буква разрешённой письменности (латиница/кириллица).

    Не-буквы (пробелы, цифры, пунктуация, эмодзи) возвращают False — они
    в расчёте доли не участвуют вообще, см. :func:`analyze`.
    """
    if not ch.isalpha():
        return False
    return _in_ranges(ord(ch), _ALLOWED_RANGES)


def script_of(ch: str) -> str:
    """Имя чужой письменности для символа, либо ``""``.

    Возвращает ``""`` для не-букв и для букв разрешённых письменностей.
    Буква, не попавшая ни в один известный чужой диапазон, помечается
    как ``other`` — она всё равно нечитаема для TTS.
    """
    if not ch.isalpha():
        return ""
    code = ord(ch)
    if _in_ranges(code, _ALLOWED_RANGES):
        return ""
    for start, end, name in _FOREIGN_RANGES:
        if start <= code <= end:
            return name
    return "other"


def analyze(text: str) -> ForeignScriptReport:
    """Посчитать долю букв чужих письменностей в ``text``.

    Не-строка и пустая строка дают нулевой отчёт (guard никогда не
    блокирует то, что и так не текст).
    """
    if not isinstance(text, str) or not text:
        return ForeignScriptReport(0, 0, 0.0, (), "")

    total = 0
    foreign = 0
    scripts: List[str] = []
    sample: List[str] = []
    for ch in text:
        if not ch.isalpha():
            continue
        total += 1
        name = script_of(ch)
        if not name:
            continue
        foreign += 1
        if name not in scripts:
            scripts.append(name)
        if len(sample) < _SAMPLE_LIMIT:
            sample.append(ch)

    ratio = (foreign / total) if total else 0.0
    return ForeignScriptReport(
        total_letters=total,
        foreign_letters=foreign,
        ratio=ratio,
        scripts=tuple(sorted(scripts)),
        sample="".join(sample),
    )


def should_skip(
    text: str,
    threshold: float = DEFAULT_FOREIGN_SCRIPT_THRESHOLD,
) -> bool:
    """True, если чанк не следует произносить (доля чужих букв > порога).

    Порог сравнивается СТРОГО (``>``): ровно 10% чужих символов ещё
    произносятся, всё что выше — блокируется.
    """
    return analyze(text).ratio > threshold


def describe(report: ForeignScriptReport) -> str:
    """Однострочная диагностика отчёта для лога."""
    return (
        f"foreign_ratio={report.ratio:.0%} "
        f"({report.foreign_letters}/{report.total_letters} букв), "
        f"scripts={','.join(report.scripts) or 'none'}, "
        f"sample={report.sample!r}"
    )


__all__ = [
    "DEFAULT_FOREIGN_SCRIPT_THRESHOLD",
    "ForeignScriptReport",
    "analyze",
    "describe",
    "is_allowed_letter",
    "script_of",
    "should_skip",
]
