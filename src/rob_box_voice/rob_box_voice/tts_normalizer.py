"""tts_normalizer - числа и знаки словами для голоса (ADR-0160 §2.5 шаг 1, PR-5, #4242).

Единственное место знания «как читаются цифры и знаки по-русски»: падежи числительных, склонение единиц, порядковые,
даты, время, проценты, градусы. Звать :func:`spell_out` должен ``prepare`` адаптера, и только если сервис цифры НЕ
читает (``capabilities().reads_digits`` ложь: Silero) либо тексту нужна длина строки по гласным (рэп, PR-4). Экран и
чат цифры оставляют цифрами (решение Шифу В2) - сюда они не ходят.

Только стандартная библиотека: пакетов нормализации (rutextnorm, ru-normalizr) в образе нет, а ради одной функции
тащить ``pymorphy3``/``num2words`` в образ Sound Pi незачем. Склоняется то, что слышно в обычной речи робота:

* числительные - именительный и родительный (после «до/от/из/без/около/после/для/у/с» и в «от X до Y»), род
  единицы («одна минута», «две секунды»), согласование единицы («1 км», «3 км», «5 км», «2,5 км»);
* порядковые (``5-й``, ``1-го``, даты, годы) - все падежи, три рода;
* остальное (дательный и предложный падежи числительных) - именительный: слышно, что число, а не как оно склонено.

Чистая функция: без сети, часов и ROS. Что не распознано, но состоит из цифр, читается числом по цифрам - сервис без
чтения цифр не должен получить ни одной.
"""

from __future__ import annotations

import re
from typing import Callable, Dict, List, Optional, Tuple

# --- числительные ------------------------------------------------------------------------------------------------

_UNITS = {
    "nom": {
        "m": ("", "один", "два", "три", "четыре", "пять", "шесть", "семь", "восемь", "девять"),
        "f": ("", "одна", "две", "три", "четыре", "пять", "шесть", "семь", "восемь", "девять"),
        "n": ("", "одно", "два", "три", "четыре", "пять", "шесть", "семь", "восемь", "девять"),
    },
    "gen": {
        "m": ("", "одного", "двух", "трёх", "четырёх", "пяти", "шести", "семи", "восьми", "девяти"),
        "f": ("", "одной", "двух", "трёх", "четырёх", "пяти", "шести", "семи", "восьми", "девяти"),
        "n": ("", "одного", "двух", "трёх", "четырёх", "пяти", "шести", "семи", "восьми", "девяти"),
    },
}
_TEENS = {
    "nom": ("десять", "одиннадцать", "двенадцать", "тринадцать", "четырнадцать", "пятнадцать", "шестнадцать",
            "семнадцать", "восемнадцать", "девятнадцать"),
    "gen": ("десяти", "одиннадцати", "двенадцати", "тринадцати", "четырнадцати", "пятнадцати", "шестнадцати",
            "семнадцати", "восемнадцати", "девятнадцати"),
}
_TENS = {
    "nom": ("", "", "двадцать", "тридцать", "сорок", "пятьдесят", "шестьдесят", "семьдесят", "восемьдесят",
            "девяносто"),
    "gen": ("", "", "двадцати", "тридцати", "сорока", "пятидесяти", "шестидесяти", "семидесяти", "восьмидесяти",
            "девяноста"),
}
_HUNDREDS = {
    "nom": ("", "сто", "двести", "триста", "четыреста", "пятьсот", "шестьсот", "семьсот", "восемьсот", "девятьсот"),
    "gen": ("", "ста", "двухсот", "трёхсот", "четырёхсот", "пятисот", "шестисот", "семисот", "восьмисот",
            "девятисот"),
}
#: Разряды: (одна, несколько, много) в именительном, род. Родительный - «одной тысячи» / «двух тысяч».
_SCALES: Tuple[Tuple[Tuple[str, str, str], str], ...] = (
    (("", "", ""), "m"),
    (("тысяча", "тысячи", "тысяч"), "f"),
    (("миллион", "миллиона", "миллионов"), "m"),
    (("миллиард", "миллиарда", "миллиардов"), "m"),
    (("триллион", "триллиона", "триллионов"), "m"),
)
_MAX_CARDINAL = 10 ** (3 * len(_SCALES)) - 1


def plural_index(number: int) -> int:
    """0 - «один» (1, 21, 101…), 1 - «несколько» (2–4, 22…), 2 - «много» (0, 5–20, 25…)."""
    last_two = abs(number) % 100
    if 11 <= last_two <= 14:
        return 2
    last = last_two % 10
    return 0 if last == 1 else 1 if 2 <= last <= 4 else 2


def _triple(n: int, gender: str, case: str) -> List[str]:
    words: List[str] = []
    hundreds, rest = divmod(n, 100)
    if hundreds:
        words.append(_HUNDREDS[case][hundreds])
    if 10 <= rest <= 19:
        words.append(_TEENS[case][rest - 10])
    else:
        tens, unit = divmod(rest, 10)
        if tens:
            words.append(_TENS[case][tens])
        if unit:
            words.append(_UNITS[case][gender][unit])
    return words


def _groups(number: int) -> List[int]:
    """Тройки цифр от младшей к старшей."""
    groups: List[int] = []
    while number:
        number, part = divmod(number, 1000)
        groups.append(part)
    return groups


def _scale_word(forms: Tuple[str, str, str], part: int, case: str) -> str:
    """«тысяча / тысячи / тысяч»: именительный - по числу при разряде, родительный - единственное или множественное."""
    index = plural_index(part)
    if case == "nom":
        return forms[index]
    return forms[1] if index == 0 else forms[2]


def cardinal(number: int, gender: str = "m", case: str = "nom") -> str:
    """Целое словами: ``cardinal(21, "f")`` → «двадцать одна»; ``case="gen"`` → «двадцати одной».

    Больше 999 триллионов - цифра за цифрой (так не читают, но и не молчат)."""
    if number < 0:
        return f"минус {cardinal(-number, gender, case)}"
    if number == 0:
        return "ноль" if case == "nom" else "ноля"
    if number > _MAX_CARDINAL:
        return " ".join(cardinal(int(d), gender, case) for d in str(number))
    groups = _groups(number)
    words: List[str] = []
    for scale in range(len(groups) - 1, -1, -1):
        part = groups[scale]
        if not part:
            continue
        forms, scale_gender = _SCALES[scale]
        words.extend(_triple(part, scale_gender if scale else gender, case))
        if scale:
            words.append(_scale_word(forms, part, case))
    return " ".join(words)


# --- порядковые --------------------------------------------------------------------------------------------------

#: Именительный мужского рода; остальные формы - по окончанию (``_inflect_ordinal``).
_ORD_UNITS = ("", "первый", "второй", "третий", "четвёртый", "пятый", "шестой", "седьмой", "восьмой", "девятый")
_ORD_TEENS = ("десятый", "одиннадцатый", "двенадцатый", "тринадцатый", "четырнадцатый", "пятнадцатый",
              "шестнадцатый", "семнадцатый", "восемнадцатый", "девятнадцатый")
_ORD_TENS = ("", "", "двадцатый", "тридцатый", "сороковой", "пятидесятый", "шестидесятый", "семидесятый",
             "восьмидесятый", "девяностый")
_ORD_HUNDREDS = ("", "сотый", "двухсотый", "трёхсотый", "четырёхсотый", "пятисотый", "шестисотый", "семисотый",
                 "восьмисотый", "девятисотый")
_ORD_THOUSAND_PREFIX = ("", "", "двух", "трёх", "четырёх", "пяти", "шести", "семи", "восьми", "девяти")

#: Окончания по (падеж, род): твёрдый «-ый/-ой» и мягкий «третий».
_ORD_ENDINGS_HARD = {
    ("nom", "m"): "ый", ("nom", "n"): "ое", ("nom", "f"): "ая",
    ("gen", "m"): "ого", ("gen", "n"): "ого", ("gen", "f"): "ой",
    ("dat", "m"): "ому", ("dat", "n"): "ому", ("dat", "f"): "ой",
    ("prep", "m"): "ом", ("prep", "n"): "ом", ("prep", "f"): "ой",
    ("instr", "m"): "ым", ("instr", "n"): "ым", ("instr", "f"): "ой",
}
_ORD_ENDINGS_SOFT = {
    ("nom", "m"): "ий", ("nom", "n"): "ье", ("nom", "f"): "ья",
    ("gen", "m"): "ьего", ("gen", "n"): "ьего", ("gen", "f"): "ьей",
    ("dat", "m"): "ьему", ("dat", "n"): "ьему", ("dat", "f"): "ьей",
    ("prep", "m"): "ьем", ("prep", "n"): "ьем", ("prep", "f"): "ьей",
    ("instr", "m"): "ьим", ("instr", "n"): "ьим", ("instr", "f"): "ьей",
}


def _inflect_ordinal(word: str, case: str, gender: str) -> str:
    """«пятый» → по падежу и роду. Ударный «-ой» (второй, шестой, сороковой) склоняется как «-ый»."""
    if (case, gender) == ("nom", "m"):
        return word
    if word.endswith("ий"):
        return word[:-2] + _ORD_ENDINGS_SOFT[(case, gender)]
    return word[:-2] + _ORD_ENDINGS_HARD[(case, gender)]


def _head(number: int) -> str:
    """Начало составного порядкового: «тысяча девятьсот…», а не «одна тысяча девятьсот…»."""
    words = cardinal(number)
    return words[len("одна "):] if words.startswith("одна тысяч") else words


def ordinal(number: int, case: str = "nom", gender: str = "m") -> str:
    """Порядковое: ``ordinal(24, "gen")`` → «двадцать четвёртого»; ``ordinal(2024, "gen")`` → «две тысячи двадцать
    четвёртого». До 9999; больше - количественным (порядковое такой величины не читают)."""
    if number <= 0 or number > 9999:
        return cardinal(number, gender, "nom")
    head, tail = _ordinal_tail(number)
    return " ".join(w for w in (_head(head) if head else "", _inflect_ordinal(tail, case, gender)) if w)


def _ordinal_tail(number: int) -> Tuple[int, str]:
    """Число → (что читается количественным, последнее слово порядковым в именительном мужского рода)."""
    rest = number % 100
    if rest:
        unit = rest % 10
        if rest < 10:
            return number - rest, _ORD_UNITS[rest]
        if rest < 20:
            return number - rest, _ORD_TEENS[rest - 10]
        return (number - rest, _ORD_TENS[rest // 10]) if not unit else (number - unit, _ORD_UNITS[unit])
    hundreds = number % 1000
    if hundreds:
        return number - hundreds, _ORD_HUNDREDS[hundreds // 100]
    thousands = number // 1000
    return 0, "тысячный" if thousands == 1 else f"{_ORD_THOUSAND_PREFIX[thousands]}тысячный"


# --- единицы и знаки (знание живёт здесь) -----------------------------------------------------------------------

#: Единица → (одна, несколько, много, род). «Несколько» = родительный единственного.
_UNIT_WORDS: Dict[str, Tuple[str, str, str, str]] = {
    "км": ("километр", "километра", "километров", "m"),
    "м": ("метр", "метра", "метров", "m"),
    "дм": ("дециметр", "дециметра", "дециметров", "m"),
    "см": ("сантиметр", "сантиметра", "сантиметров", "m"),
    "мм": ("миллиметр", "миллиметра", "миллиметров", "m"),
    "кг": ("килограмм", "килограмма", "килограммов", "m"),
    "г": ("грамм", "грамма", "граммов", "m"),
    "мг": ("миллиграмм", "миллиграмма", "миллиграммов", "m"),
    "т": ("тонна", "тонны", "тонн", "f"),
    "л": ("литр", "литра", "литров", "m"),
    "мл": ("миллилитр", "миллилитра", "миллилитров", "m"),
    "ч": ("час", "часа", "часов", "m"),
    "мин": ("минута", "минуты", "минут", "f"),
    "сек": ("секунда", "секунды", "секунд", "f"),
    "мс": ("миллисекунда", "миллисекунды", "миллисекунд", "f"),
    "мкс": ("микросекунда", "микросекунды", "микросекунд", "f"),
    "гц": ("герц", "герца", "герц", "m"),
    "кгц": ("килогерц", "килогерца", "килогерц", "m"),
    "мгц": ("мегагерц", "мегагерца", "мегагерц", "m"),
    "ггц": ("гигагерц", "гигагерца", "гигагерц", "m"),
    "вт": ("ватт", "ватта", "ватт", "m"),
    "квт": ("киловатт", "киловатта", "киловатт", "m"),
    "мб": ("мегабайт", "мегабайта", "мегабайт", "m"),
    "гб": ("гигабайт", "гигабайта", "гигабайт", "m"),
    "кб": ("килобайт", "килобайта", "килобайт", "m"),
    "тб": ("терабайт", "терабайта", "терабайт", "m"),
    "шт": ("штука", "штуки", "штук", "f"),
    "руб": ("рубль", "рубля", "рублей", "m"),
    "₽": ("рубль", "рубля", "рублей", "m"),
    "$": ("доллар", "доллара", "долларов", "m"),
    "€": ("евро", "евро", "евро", "m"),
    "%": ("процент", "процента", "процентов", "m"),
    "°": ("градус", "градуса", "градусов", "m"),
    "°c": ("градус Цельсия", "градуса Цельсия", "градусов Цельсия", "m"),
    "°f": ("градус Фаренгейта", "градуса Фаренгейта", "градусов Фаренгейта", "m"),
}
#: Составные единицы скорости: первая часть склоняется, остальное - как есть.
_UNIT_RATE_SUFFIX = {"/ч": " в час", "/с": " в секунду", "/мин": " в минуту", "/сек": " в секунду"}
#: Единицы, которые без пробела от числа ("5с", "5с") не берём: слишком похоже на слово или букву.
_UNIT_NEEDS_SPACE = frozenset({"г", "т", "м", "л", "ч"})

_MONTHS_GEN = ("января", "февраля", "марта", "апреля", "мая", "июня", "июля", "августа", "сентября", "октября",
               "ноября", "декабря")
#: Предлоги, после которых число стоит в родительном падеже.
_GEN_PREPOSITIONS = frozenset({"до", "от", "из", "без", "около", "после", "для", "у", "с", "более", "менее",
                               "свыше", "кроме", "среди", "вместо", "против", "возле", "около"})
_DATE_GEN_PREPOSITIONS = frozenset({"до", "от", "с", "после", "около", "из", "без", "для", "у"})
#: Предлоги, после которых год - в предложном («в 2024 году»); «к» - в дательном.
_YEAR_PREP = frozenset({"в", "на", "при", "о"})

#: Знаки между числами и перед числом.
_BINARY_SIGNS = {"+": "плюс", "=": "равно", "×": "умножить на", "*": "умножить на", "÷": "разделить на",
                 "≈": "примерно", "≠": "не равно"}
_PLAIN_SIGNS = {"№": "номер", "≥": "больше либо равно", "≤": "меньше либо равно", "±": "плюс-минус"}

_NUM = r"\d+(?:[.,]\d+)?"
_DASHES = r"\-–—−"
_UNIT_LETTERS = "|".join(sorted((re.escape(u) for u in _UNIT_WORDS if u[0].isalpha()), key=len, reverse=True))
_UNIT_RE = rf"(?:%|°\s?[CcСсFf]?|₽|\$|€|(?:{_UNIT_LETTERS})(?:/(?:ч|с|мин|сек))?)"
_NUMBER_RE = re.compile(
    rf"(?P<sign>(?<![\w.,])[+{_DASHES}](?=\d))?(?<![\w.,])(?P<a>{_NUM})(?![.,]\d)"
    rf"(?:\s?[{_DASHES}]\s?(?P<b>{_NUM}))?"
    rf"(?:(?P<sp>\s?)(?P<unit>{_UNIT_RE})(?![\w]))?",
    re.IGNORECASE,
)
_DATE_RE = re.compile(r"(?<![\w.])(?:(\d{1,2})\.(\d{1,2})\.(\d{4})|(\d{2})\.(\d{2}))(?![\w]|\.\d)")
_DAY_MONTH_RE = re.compile(rf"(?<![\w.,])(\d{{1,2}})\s+({'|'.join(_MONTHS_GEN)})(?![а-яё])", re.IGNORECASE)
_TIME_RE = re.compile(r"(?<![\w.:])([01]?\d|2[0-3]):([0-5]\d)(?![\d:])")
_YEAR_RE = re.compile(r"(?<![\w.,])(\d{4})\s?(г\.|году|года|годом|годе|год|г)(?![а-яё\w])", re.IGNORECASE)
_ORD_SUFFIX_RE = re.compile(r"(?<![\w.,])(\d+)[-–](ый|ой|ий|го|му|ом|им|ым|ая|ей|ую|ое|ее|е|й|я|м|х|ти)(?![\w])",
                            re.IGNORECASE)
_BARE_DIGITS_RE = re.compile(r"\d+")
_WORD_BEFORE_RE = re.compile(r"([A-Za-zА-Яа-яЁё]+)\W*$")


def _prev_word(text: str, position: int) -> str:
    match = _WORD_BEFORE_RE.search(text[:position])
    return match.group(1).lower() if match else ""


def _fraction_words(whole: str, frac: str, gender_unit: str = "m") -> str:
    """«3,5» → «три целых пять десятых». Больше трёх знаков - целая часть, «запятая», дробная по разрядам."""
    whole_n, frac_n = int(whole), int(frac)
    if len(frac) > 3:
        return f"{cardinal(whole_n)} запятая {' '.join(cardinal(int(d)) for d in frac)}"
    denominator = ("десят", "сот", "тысячн")[len(frac) - 1]
    whole_words = cardinal(whole_n, "f")
    whole_unit = "целая" if plural_index(whole_n) == 0 else "целых"
    frac_unit = f"{denominator}ая" if plural_index(frac_n) == 0 else f"{denominator}ых"
    return f"{whole_words} {whole_unit} {cardinal(frac_n, 'f')} {frac_unit}"


def _unit_forms(unit_raw: str) -> Optional[Tuple[Tuple[str, str, str, str], str]]:
    """Единица из текста → (формы, хвост «в час»); ``None``, если это не единица."""
    key = re.sub(r"\s", "", unit_raw).lower()
    rate = ""
    for suffix, words in _UNIT_RATE_SUFFIX.items():
        if key.endswith(suffix):
            key, rate = key[: -len(suffix)], words
            break
    if key.startswith("°"):  # кириллическая «С» тоже градус Цельсия
        key = "°" + {"": "", "c": "c", "с": "c", "f": "f"}[key[1:]]
    return (_UNIT_WORDS[key], rate) if key in _UNIT_WORDS else None


def _number_words(raw: str, unit: Optional[Tuple[str, str, str, str]], case: str) -> Tuple[str, int]:
    """Число (целое или десятичное) + единица в нужном падеже и согласовании → (слова, «род числа» для единицы)."""
    gender = unit[3] if unit else "m"
    if re.search(r"[.,]", raw):
        whole, frac = re.split(r"[.,]", raw)
        return _fraction_words(whole, frac), -1
    number = int(raw)
    return cardinal(number, gender, case), number


def _unit_word(forms: Tuple[str, str, str, str], number: int, case: str) -> str:
    if number < 0:  # дробное: родительный единственного
        return forms[1]
    index = plural_index(number)
    if case == "gen":
        return forms[1] if index == 0 else forms[2]
    return forms[index]


#: Окончание после дефиса («5-й», «1-го») → (падеж, род) порядкового.
_ORD_SUFFIX_CASES = {
    "ый": ("nom", "m"), "ой": ("nom", "m"), "ий": ("nom", "m"), "й": ("nom", "m"), "ое": ("nom", "n"),
    "ее": ("nom", "n"), "е": ("nom", "n"), "ая": ("nom", "f"), "я": ("nom", "f"), "ую": ("nom", "f"),
    "го": ("gen", "m"), "ей": ("gen", "f"), "му": ("dat", "m"), "ом": ("prep", "m"), "м": ("prep", "m"),
    "им": ("instr", "m"), "ым": ("instr", "m"),
}


def _date(match: "re.Match[str]") -> str:
    day, month, year = int(match.group(1) or match.group(4)), int(match.group(2) or match.group(5)), match.group(3)
    if not (1 <= day <= 31 and 1 <= month <= 12):
        return match.group(0)
    gen = bool(year) or _prev_word(match.string, match.start()) in _DATE_GEN_PREPOSITIONS  # «15 марта 2024 года»
    words = f"{ordinal(day, 'gen' if gen else 'nom', 'n')} {_MONTHS_GEN[month - 1]}"
    return f"{words} {ordinal(int(year), 'gen')} года" if year else words


def _day_month(match: "re.Match[str]") -> str:
    day = int(match.group(1))
    if not 1 <= day <= 31:
        return match.group(0)
    case = "gen" if _prev_word(match.string, match.start()) in _DATE_GEN_PREPOSITIONS else "nom"
    return f"{ordinal(day, case, 'n')} {match.group(2).lower()}"


def _time_of_day(match: "re.Match[str]") -> str:
    hours, minutes = int(match.group(1)), int(match.group(2))
    tail = "ноль ноль" if minutes == 0 else f"ноль {cardinal(minutes)}" if minutes < 10 else cardinal(minutes)
    return f"{cardinal(hours)} {tail}"


def _year_case(prev: str, word: str) -> Tuple[str, str]:
    """Падеж года и слово «год» по написанию и предлогу: «в 2024 году», «с 2024 года», «к 2024 году», «2024 г.»."""
    short = word in ("г.", "г")
    if word in ("году", "годе") or (short and prev in _YEAR_PREP):
        return ("dat" if prev == "к" else "prep"), "году"
    if word == "годом":
        return "instr", "годом"
    if word == "года" or (short and prev in _DATE_GEN_PREPOSITIONS):
        return "gen", "года"
    return "nom", "год"


def _year(match: "re.Match[str]") -> str:
    case, noun = _year_case(_prev_word(match.string, match.start()), match.group(2).lower())
    return f"{ordinal(int(match.group(1)), case)} {noun}"


def _ordinal_suffix(match: "re.Match[str]") -> str:
    number, ending = int(match.group(1)), match.group(2).lower()
    if ending in ("х", "ти"):  # «3-х человек», «5-ти минут»
        return cardinal(number, "m", "gen")
    case, gender = _ORD_SUFFIX_CASES[ending]
    return ordinal(number, case, gender)


def _unit_of(match: "re.Match[str]") -> Tuple[bool, Optional[Tuple[str, str, str, str]], str]:
    """Единица после числа → (есть ли слово-не-единица, формы, «в час»). Однобуквенная без пробела - не единица."""
    unit_raw = match.group("unit")
    if not unit_raw:
        return False, None, ""
    found = _unit_forms(unit_raw)
    if found is None or (unit_raw.lower() in _UNIT_NEEDS_SPACE and not match.group("sp")):
        return True, None, ""  # «12 арбузов» - слово, не единица
    return False, found[0], found[1]


def _number_expr(match: "re.Match[str]") -> str:
    not_unit, unit, rate = _unit_of(match)
    if not_unit:
        return match.group(0)
    first, second = match.group("a"), match.group("b")
    if second is not None:  # диапазон «5–7 км» - «от пяти до семи километров»
        high, number = _number_words(second, unit, "gen")
        body, case = f"от {_number_words(first, unit, 'gen')[0]} до {high}", "gen"
    else:
        case = "gen" if _prev_word(match.string, match.start()) in _GEN_PREPOSITIONS else "nom"
        body, number = _number_words(first, unit, case)
        if match.group("sign"):
            body = f"{'плюс' if match.group('sign') == '+' else 'минус'} {body}"
    return f"{body} {_unit_word(unit, number, case)}{rate}" if unit else body


def _plain_signs(text_: str) -> str:
    for sign, word in _PLAIN_SIGNS.items():
        text_ = re.sub(rf"\s*{re.escape(sign)}\s*", f" {word} ", text_)
    for sign, word in _BINARY_SIGNS.items():
        text_ = re.sub(rf"(?<=\d)\s*{re.escape(sign)}\s*(?=[\d{_DASHES}]?\d)", f" {word} ", text_)
    return text_


def spell_out(text: str) -> str:
    """Числа и знаки словами; всё остальное в тексте не меняется.

    >>> spell_out("Сейчас 12:30, на улице -5 °C, влажность 45%.")
    'Сейчас двенадцать тридцать, на улице минус пять градусов Цельсия, влажность сорок пять процентов.'
    """
    if not text or not (re.search(r"\d", text) or any(s in text for s in (*_PLAIN_SIGNS, "%", "°"))):
        return text
    for step in _STEPS:
        text = step(text)
    return re.sub(r"[ 	]{2,}", " ", text).strip()


_STEPS: Tuple[Callable[[str], str], ...] = (
    lambda t: _DATE_RE.sub(_date, t),
    lambda t: _DAY_MONTH_RE.sub(_day_month, t),
    lambda t: _TIME_RE.sub(_time_of_day, t),
    lambda t: _YEAR_RE.sub(_year, t),
    lambda t: _ORD_SUFFIX_RE.sub(_ordinal_suffix, t),
    _plain_signs,
    lambda t: _NUMBER_RE.sub(_number_expr, t),
    lambda t: _BARE_DIGITS_RE.sub(lambda m: cardinal(int(m.group(0))), t),
)

__all__ = ["cardinal", "ordinal", "plural_index", "spell_out"]
