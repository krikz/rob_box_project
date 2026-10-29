"""Правка идущего выступления не повторяет уже сказанное (#4207, ADR-0148: решает код).

Марафоны 4-6, акт 8 (n807): «посчитай от одного до двадцати», посреди счёта «и после каждого числа добавляй слово шаг».
Модель присылала счёт С НАЧАЛА («Один шаг, два шага…»), а код ставил его на места ещё не сказанных сегментов - числа
9-19 звучали дважды. Вторая половина дефекта - в коде: правка ``edit_pending`` берёт у модели ВСЕ присланные
сегменты. Эта чистая функция снимает из правки то, что уже прозвучало, до того как оно встанет на места несказанных
сегментов (:meth:`~rob_box_dialog.plan.perform.SpeechSide.edit`); первая половина - данные в окне хода
(:func:`~rob_box_dialog.plan.perform.context_block`: ``last_said``, ``pending``).

Что считается повтором (только ведущая часть правки: модель переписывает с начала, а новое идёт за ней; припев в
середине правки законен и остаётся):

* сегмент слово в слово совпадает с уже сказанным (без знаков и регистра);
* счётный сегмент (не меньше двух числительных, и числительные - не меньше половины слов) целиком из чисел, которые уже
  прозвучали («Девять шагов, десять шагов…» после «Девять, десять…»); в сегменте-счёте, где есть и новые числа,
  ведущие предложения из уже сказанных чисел отрезаются - остаётся непрозвучавшее («Двадцать — шаг. Все дошли.» после
  «двадцать» → «Все дошли.»).

Разбор - по словам и таблицам, без регексов. Модуль чистый: без ROS и I/O.
"""

from __future__ import annotations

from typing import Dict, FrozenSet, List, Sequence, Tuple

#: Числа словами (именительный, рода): значение - канон, по нему сегменты сравниваются; цифры берутся как есть.
_NUMERAL_FORMS: Dict[int, Tuple[str, ...]] = {
    0: ("ноль", "нуль"), 1: ("один", "одна", "одно"), 2: ("два", "две"), 3: ("три",), 4: ("четыре",), 5: ("пять",),
    6: ("шесть",), 7: ("семь",), 8: ("восемь",), 9: ("девять",), 10: ("десять",), 11: ("одиннадцать",),
    12: ("двенадцать",), 13: ("тринадцать",), 14: ("четырнадцать",), 15: ("пятнадцать",), 16: ("шестнадцать",),
    17: ("семнадцать",), 18: ("восемнадцать",), 19: ("девятнадцать",), 20: ("двадцать",), 30: ("тридцать",),
    40: ("сорок",), 50: ("пятьдесят",), 60: ("шестьдесят",), 70: ("семьдесят",), 80: ("восемьдесят",),
    90: ("девяносто",), 100: ("сто",),
}
NUMERAL_WORDS: Dict[str, int] = {form: value for value, forms in _NUMERAL_FORMS.items() for form in forms}
SENTENCE_END = ".!?…"


def words(text: str) -> Tuple[str, ...]:
    """Слова и числа текста: без знаков, в нижнем регистре, «ё» = «е». «Один — шаг, два!» → один, шаг, два."""
    out: List[str] = []
    current: List[str] = []
    for ch in text.lower().replace("ё", "е") + " ":
        if ch.isalnum():
            current.append(ch)
        elif current:
            out.append("".join(current))
            current = []
    return tuple(out)


def numerals(text: str) -> Tuple[int, ...]:
    """Числа текста по порядку: словами («девять», «одна») и цифрами («9»)."""
    return tuple(int(w) if w.isdigit() else NUMERAL_WORDS[w] for w in words(text) if w.isdigit() or w in NUMERAL_WORDS)


def is_counting(text: str) -> bool:
    """Счёт, а не рассказ с числом: не меньше двух чисел, и они - не меньше половины слов («Один шаг, два шага»)."""
    count = len(numerals(text))
    return count >= 2 and count * 2 >= len(words(text))


def said_numbers(said: Sequence[str]) -> FrozenSet[int]:
    """Какие числа уже прозвучали в сказанном."""
    return frozenset(n for text in said for n in numerals(text))


def is_repeat(text: str, said: Sequence[str], said_nums: FrozenSet[int]) -> bool:
    """Правка этим сегментом повторила бы сказанное: те же слова или счёт целиком из уже названных чисел."""
    key = words(text)
    if not key:
        return False
    if any(words(done) == key for done in said):
        return True
    return is_counting(text) and all(n in said_nums for n in numerals(text))


def _sentences(text: str) -> List[str]:
    """Предложения по знакам конца (знак остаётся при предложении); хвост без знака - своё предложение."""
    out: List[str] = []
    start = 0
    for i, ch in enumerate(text):
        if ch in SENTENCE_END and (i + 1 == len(text) or text[i + 1].isspace()):
            out.append(text[start:i + 1].strip())
            start = i + 1
    if text[start:].strip():
        out.append(text[start:].strip())
    return out


def _unsaid_part(text: str, said_nums: FrozenSet[int], counting: bool) -> str:
    """Счёт, где есть и новые числа: ведущие предложения из уже названных чисел отрезаются («Двадцать — шаг. Все
    дошли.» после «двадцать» → «Все дошли.»); концовка счёт не отменяет. Счёт уже шёл в снятых ведущих сегментах
    (``counting``) - хватает одного числа в сегменте («Девятнадцать — шаг.»), иначе нужно два. Не счёт - как есть;
    всё предложения-повторы - ``""``."""
    if len(numerals(text)) < (1 if counting else 2):
        return text
    sentences = _sentences(text)
    skip = 0
    for sentence in sentences:
        nums = numerals(sentence)
        if not (nums and all(n in said_nums for n in nums) and len(nums) * 2 >= len(words(sentence))):
            break
        skip += 1
    return " ".join(sentences[skip:]) if skip else text


def drop_said(rows: Sequence[Tuple[str, str, str]], said: Sequence[str]) -> List[Tuple[str, str, str]]:
    """Правка без ведущих повторов: ``rows`` - ``(текст, голос, эмоция)``, ``said`` - тексты, что уже прозвучали или
    отданы на звучание. Ведущие сегменты-повторы снимаются; первый не повтор остаётся (счёт - без уже названных
    чисел в начале) и всё за ним - как есть. Всё повтор - ``[]``."""
    nums = said_numbers(said)
    counting = False
    for i, (text, voice, emotion) in enumerate(rows):
        if is_repeat(text, said, nums):
            counting = counting or is_counting(text)
            continue
        part = _unsaid_part(text, nums, counting)
        if part:
            return [(part, voice, emotion), *rows[i + 1:]]
    return []


__all__ = ["NUMERAL_WORDS", "drop_said", "is_counting", "is_repeat", "numerals", "said_numbers", "words"]
