"""Корпус замера понимания, части 2-4: другие тексты для шагов марафона (карточка #4163, этап 2).

* часть 2 - перефразы (``paraphrases.json``, собирает ``build_paraphrases.py``);
* часть 3 - реальные искажения STT (``stt_distortions.json``, ``build_stt.py``);
* часть 4 - английский (``english.json``, ``build_english.py``).

Файл части хранит только тексты: ``alternatives`` = ``{id шага части 1: [{"text": …, …}, …]}``. Ожидания шага
(``acceptance``, ``patterns``, ``expect``, повторы) в файл не копируются: при загрузке шаг берётся из части 1 как есть, и
меняется только ``text`` (:func:`substituted`). ``base`` - имя и хеш части 1, против которой собраны альтернативы:
пересобрали часть 1 - часть 2-4 не грузится, пока её не пересоберут.

Прогон части (``understanding_measure --parts``) - весь марафон по актам, как часть 1: контекст актов (что играет, кто
знаком) честный. У оцениваемого шага с альтернативой текст заменён: прогон ``r`` берёт альтернативу
``((r - 1) mod K) + 1`` (:func:`alternative_for`). Шаг без альтернативы идёт исходным текстом и в метрику части не входит.

Автопроверка альтернатив (:func:`check_paraphrase`, :func:`check_translation`) - то, без чего ожидание шага стало бы
неверным: вейк-слово (в начале - то же; без вейка - без вейка), имена людей и треков, числа, цитаты в «…», и якоря -
слова ожиданий (``expected_keywords`` / ``must_not_say`` этого шага и шагов ПОСЛЕ него), которые есть в исходной фразе:
«поищи, что я говорил про чай» → в памяти ищется чай; «по-китайски» → n1007 проверяет, что про «китайск» робот забыл.
"""

from __future__ import annotations

import copy
import hashlib
import json
import re
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

HERE = Path(__file__).resolve().parent
SCHEMA = 1
#: Часть → (файл, название). Часть 1 - ``marathon.json`` (``build_marathon.py``), здесь её нет: альтернатив у неё нет.
PARTS: Dict[int, Tuple[str, str]] = {2: ("paraphrases.json", "перефразы"), 3: ("stt_distortions.json", "искажения STT"),
                                     4: ("english.json", "английский")}
TITLES = {1: "фразы марафона", **{p: title for p, (_, title) in PARTS.items()}}

#: Основы имён людей, голосов, команды и героя трека в фразах марафона (``scripts/e2e/gen_night_marathon.py``): имя
#: остаётся, падеж может меняться («Саша» → «Саши»). Тест сверяет: каждое слово с большой буквы внутри фразы - здесь.
NAME_STEMS: Dict[str, Tuple[str, ...]] = {
    "саш": ("sasha",), "борис": ("boris",), "гриш": ("grisha",), "валер": ("valera",), "нин": ("nina",),
    "тамар": ("tamara",), "алён": ("alyona", "alena", "alyena"), "джейн": ("jane",), "спартак": ("spartak",),
    "красн": ("red",), "шапочк": ("riding hood",), "енот": ("raccoon",)}
#: Числительные фраз марафона (формы) → значение; порядковые - отдельно (``1st`` …), цифры - как количественные.
_CARDINALS = {1: "один одна одно одного одной одному одним одном", 2: "два две двух двум двумя",
              3: "три трёх трех трём трем тремя троих", 4: "четыре четырёх четырех", 5: "пять пяти пятью",
              6: "шесть шести", 7: "семь семи семью", 8: "восемь восьми", 9: "девять девяти", 10: "десять десяти",
              20: "двадцать двадцати двадцатью", 30: "тридцать тридцати", 40: "сорок сорока",
              50: "пятьдесят пятидесяти", 98: "девяносто"}
_ORDINALS = {"1st": "первый первая первое первого первой первом первую первым",
             "2nd": "второй вторая второе второго втором вторую вторым",
             "3rd": "третий третья третье третьего третьей третьем третью третьим"}
_EN_NUMBERS = {1: "one", 2: "two", 3: "three", 4: "four", 5: "five", 6: "six", 7: "seven", 8: "eight", 9: "nine",
               10: "ten", 20: "twenty", 30: "thirty", 40: "forty", 50: "fifty"}
_EN_ORDINALS = {"1st": "first", "2nd": "second", "3rd": "third"}
NUMBER_WORDS: Dict[str, Any] = {**{w: v for v, forms in _CARDINALS.items() for w in forms.split()},
                                **{w: v for v, forms in _ORDINALS.items() for w in forms.split()}}
#: «a couple of words» - «в двух словах»; «one» в английском чаще местоимение («each one»): единица перевода не
#: считается лишним числом, если в исходнике единицы нет (:func:`check_translation`).
EN_NUMBER_WORDS: Dict[str, Any] = {**{w: v for v, w in _EN_NUMBERS.items()}, **{w: v for v, w in _EN_ORDINALS.items()},
                                   "couple": 2}
#: Вейк английской части: «Robot» в таблице вейков робота нет (``docker/sound/config/wake_words.yaml``), с ним шаг
#: не адресован ни в одном варианте. Имя робота латиницей - ``robbox`` - в таблице есть.
EN_WAKE = "Robbox"
#: Строки ``must_not_say``, которые есть больше чем у половины шагов, - общий список фраз отказа робота, не якорь.
GENERIC_SHARE = 0.5

_WORD = re.compile(r"[\w-]+", re.UNICODE)
_QUOTE = re.compile(r"«([^»]+)»")
_CYRILLIC = re.compile(r"[а-яё]", re.IGNORECASE)


# ── Файл части ────────────────────────────────────────────────────────────────────────────────────────────────────


def digest(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, ensure_ascii=False, sort_keys=True).encode("utf-8")).hexdigest()


def part_path(part: int) -> Path:
    return HERE / PARTS[part][0]


def index_of(base: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    return {s["id"]: s for a in base["acts"] for s in a["steps"]}


def load_part(part: int, base: Dict[str, Any], path: Optional[Path] = None) -> Dict[str, Any]:
    """Часть 2-4. Громкий отказ (``SystemExit``), если файл правили руками, если он собран против другой части 1, если
    альтернатива висит на неизвестном или неоцениваемом шаге или у шага пустой список."""
    path = Path(path or part_path(part))
    data = json.loads(path.read_text(encoding="utf-8"))
    alts = data["alternatives"]
    if digest(alts) != data["corpus_sha256"]:
        raise SystemExit(f"{path}: хеш не совпал с corpus_sha256 - файл правили руками; пересобрать и закоммитить")
    if data["base"]["corpus_sha256"] != base["corpus_sha256"]:
        raise SystemExit(f"{path}: собран против части 1 {data['base']['corpus_sha256'][:12]}, а сейчас "
                         f"{base['corpus_sha256'][:12]} - пересобрать часть {part}")
    index = index_of(base)
    bad = sorted(i for i, rows in alts.items() if i not in index or not index[i]["scored"] or not rows)
    if bad:
        raise SystemExit(f"{path}: альтернативы у неизвестных, неоцениваемых шагов или пустые: {bad[:5]}")
    return data


def write_part(path: Path, payload: Dict[str, Any]) -> str:
    """Записать часть с хешем ``alternatives``; вернуть хеш."""
    payload = {**payload, "corpus_sha256": digest(payload["alternatives"])}
    ordered = {k: payload[k] for k in ("name", "schema", "part", "base", "generator", "method", "counts", "excluded",
                                       "corpus_sha256", "alternatives") if k in payload}
    Path(path).write_text(json.dumps(ordered, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    return payload["corpus_sha256"]


def base_ref(base: Dict[str, Any]) -> Dict[str, str]:
    return {"name": base["name"], "corpus_sha256": base["corpus_sha256"]}


def repoint(data: Dict[str, Any], old_base: Dict[str, Any], new_base: Dict[str, Any]) -> Dict[str, Any]:
    """Перевесить замороженную часть на пересобранную часть 1 БЕЗ новых текстов: остаются альтернативы шагов, чей текст
    в части 1 не изменился (и шаг по-прежнему оцениваемый); у остальных альтернативы убираются с причиной - они про
    другую фразу. Так в репозитории остаётся подмножество ровно того корпуса, что мерился."""
    if data["base"]["corpus_sha256"] != old_base["corpus_sha256"]:
        raise SystemExit(f"часть собрана против {data['base']['corpus_sha256'][:12]}, а старая часть 1 - "
                         f"{old_base['corpus_sha256'][:12]}")
    old, new = index_of(old_base), index_of(new_base)
    keep: Dict[str, List[Dict[str, Any]]] = {}
    dropped: List[Dict[str, Any]] = []
    for sid, rows in data["alternatives"].items():
        before, after = old[sid]["step"]["text"], (new.get(sid) or {}).get("step", {}).get("text")
        if after == before and new[sid]["scored"]:
            keep[sid] = rows
            continue
        dropped.append({"id": sid, "alternatives_dropped": len(rows),
                        "reason": f"часть 1 пересобрана ({old_base['corpus_sha256'][:12]} → "
                                  f"{new_base['corpus_sha256'][:12]}): текст шага сменился «{before}» → «{after}», "
                                  f"альтернативы старого текста к новому шагу не относятся"})
    # Повторный перевес (#4196: часть 1 пересобрана ещё раз, ни один шаг не сменился) хранит цепочку: хеш ИЗМЕРЕННОГО
    # корпуса и первая часть 1 остаются прежними, убранные шаги копятся - файл по-прежнему подмножество измеренного.
    prev = data["method"].get("repointed") or {}
    method = {**data["method"], "repointed": {"measured_sha256": prev.get("measured_sha256", data["corpus_sha256"]),
                                              "from_base": prev.get("from_base", old_base["corpus_sha256"]),
                                              "to_base": new_base["corpus_sha256"],
                                              "dropped_steps": [*prev.get("dropped_steps", []),
                                                                *(d["id"] for d in dropped)]}}
    excluded = [*data.get("excluded", []), *dropped]
    return {**data, "base": base_ref(new_base), "method": method, "excluded": excluded, "alternatives": keep,
            "counts": recount(data["counts"], keep, excluded)}


def recount(counts: Dict[str, Any], alternatives: Dict[str, List[Dict[str, Any]]],
            excluded: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Счётчики части после правки списка альтернатив: те же ключи, новые значения."""
    sizes = [len(r) for r in alternatives.values()]
    fresh = {"steps": len(alternatives), "alternatives": sum(sizes), "excluded": len(excluded),
             "max_per_step": max(sizes or [0]), "steps_below_min": sum(s < 2 for s in sizes),
             "from_issues": sum(1 for rows in alternatives.values() for r in rows
                                if (r.get("source") or {}).get("kind") == "issue"),
             **{f"steps_with_{n}": sizes.count(n) for n in (1, 2, 3)}}
    return {k: fresh.get(k, v) for k, v in counts.items()}


def repoint_file(part: int, path: Path, old_base_path: Path, new_base: Dict[str, Any]) -> str:
    """CLI-обёртка :func:`repoint` для сборщиков (``--repoint-from <старый marathon.json>``); вернуть новый хеш."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    old_base = json.loads(Path(old_base_path).read_text(encoding="utf-8"))
    out = repoint(data, old_base, new_base)
    sha = write_part(Path(path), out)
    print(f"часть {part}: перевешена на часть 1 {new_base['corpus_sha256'][:12]}, {out['counts']}, sha256 {sha[:12]} "
          f"(мерилась {data['corpus_sha256'][:12]}); убраны шаги: {out['method']['repointed']['dropped_steps']}")
    return sha


# ── Подстановка в марафон ─────────────────────────────────────────────────────────────────────────────────────────


def alternative_for(rows: Sequence[Dict[str, Any]], run: int) -> Dict[str, Any]:
    """Альтернатива прогона ``run`` (с 1): по кругу, ``((run - 1) mod K) + 1``."""
    return rows[(run - 1) % len(rows)]


def substituted(step: Dict[str, Any], text: str) -> Dict[str, Any]:
    """Шаг части 1 с другим текстом: ожидания те же, меняется только ``text``."""
    out = copy.deepcopy(step)
    out["text"] = text
    return out


def part_corpus(base: Dict[str, Any], alternatives: Dict[str, List[Dict[str, Any]]], run: int) -> Dict[str, Any]:
    """Корпус для прогона ``run`` части: все акты и шаги части 1 по порядку, у шагов с альтернативой - другой текст;
    ``scored`` - только у шагов части (остальные идут контекстом, как акустические в части 1)."""
    acts = []
    for act in base["acts"]:
        steps = []
        for entry in act["steps"]:
            rows = alternatives.get(entry["id"])
            if rows:
                entry = {**entry, "step": substituted(entry["step"], alternative_for(rows, run)["text"])}
            else:
                entry = {**entry, "scored": False}
            steps.append(entry)
        acts.append({**act, "steps": steps})
    return {**base, "acts": acts}


# ── Слова фразы ───────────────────────────────────────────────────────────────────────────────────────────────────


def norm(text: str) -> str:
    """Нижний регистр, «ё» → «е», только слова через пробел."""
    return " ".join(_WORD.findall(text.lower().replace("ё", "е"))).replace("-", " ")


def _wake_pattern(wakes: Sequence[str]) -> "re.Pattern[str]":
    ordered = sorted({w.lower() for w in wakes if w}, key=len, reverse=True)
    return re.compile(r"(?<![\w-])(" + "|".join(re.escape(w) for w in ordered) + r")(?![\w-])", re.IGNORECASE)


def wakes_in(text: str, wakes: Sequence[str]) -> List[str]:
    """Вейк-слова в фразе по порядку (по границам слов, без регистра) - как их ищет адресация."""
    return [m.group(1).lower() for m in _wake_pattern(wakes).finditer(text)]


def leading_wake(text: str, wakes: Sequence[str]) -> str:
    """Вейк в самом начале фразы (до него только пробелы и знаки) или «»."""
    match = _wake_pattern(wakes).match(text.lstrip(" \t«\"'—-"))
    return match.group(1).lower() if match else ""


def names_in(text: str) -> List[str]:
    """Основы имён (:data:`NAME_STEMS`), которые есть в фразе в любом падеже."""
    words = [w.lower() for w in _WORD.findall(text)]
    return sorted({stem for stem in NAME_STEMS for w in words if w.startswith(stem)})


def capitalized_inside(text: str) -> List[str]:
    """Слова с большой буквы не в начале предложения - кандидаты в имена (тест сверяет их с :data:`NAME_STEMS`)."""
    out, start = [], True
    for token in re.findall(r"[\w-]+|[.!?]", text):
        if token in ".!?":
            start = True
            continue
        if not start and token[:1].isupper():
            out.append(token)
        start = False
    return out


def numbers_in(text: str, table: Optional[Dict[str, Any]] = None) -> List[Any]:
    """Числа фразы: цифры и числительные (:data:`NUMBER_WORDS`), отсортированные значения."""
    table = NUMBER_WORDS if table is None else table
    found: List[Any] = []
    for word in _WORD.findall(text.lower()):
        if word.isdigit():
            found.append(int(word))
        elif word in table:
            found.append(table[word])
    return sorted(found, key=str)


def quotes_in(text: str) -> List[str]:
    return [q.strip() for q in _QUOTE.findall(text)]


def _has_any(text: str, group: str) -> bool:
    """Есть ли в фразе одно из слов группы ``a|b|c`` - с начала слова (``рок`` не ловится в «сорока»)."""
    normal = " " + norm(text)
    return any(alt and (" " + alt) in normal for alt in (norm(a) for a in group.split("|")))


# ── Якоря из ожиданий ─────────────────────────────────────────────────────────────────────────────────────────────


#: Строка ожидания с этим префиксом - регэксп по речи робота (``e2e_tool_match.REGEX_PREFIX``), не слова: не якорь.
REGEX_PREFIX = "re:"


def _groups_of(step: Dict[str, Any], generic: Iterable[str], forbidden: bool) -> List[str]:
    """Группы слов ожиданий шага: ``expected_keywords`` и (``forbidden``) ``must_not_say`` без общих фраз отказа."""
    acc = step.get("acceptance") or {}
    skip = set(generic)
    groups = [*(acc.get("expected_keywords") or []), *((acc.get("must_not_say") or []) if forbidden else [])]
    return [g for g in groups if g not in skip and not g.startswith(REGEX_PREFIX)]


def generic_groups(base: Dict[str, Any]) -> List[str]:
    """``must_not_say``, общие больше чем для половины шагов: фразы отказа робота, а не слова о смысле реплики."""
    steps = [s["step"] for a in base["acts"] for s in a["steps"]]
    counts: Dict[str, int] = {}
    for step in steps:
        for item in set((step.get("acceptance") or {}).get("must_not_say") or []):
            counts[item] = counts.get(item, 0) + 1
    return sorted(item for item, n in counts.items() if n > GENERIC_SHARE * len(steps))


def anchors(base: Dict[str, Any]) -> Dict[str, List[str]]:
    """Якоря шага - группы слов, хотя бы одно слово которых есть в исходной фразе: ``expected_keywords`` самого шага
    (робот отвечает словами вопроса: «сейчас тихо?» - «да, тихо») и ``expected_keywords`` / ``must_not_say`` шагов
    ПОСЛЕ него (память и утечка: n203 «зелёный чай» → n1009 ждёт «зелён»; n907 «космос» → n1007 запрещает «космос»).
    Свой ``must_not_say`` якорем не бывает: это запрет ответу, а не смысл реплики. Альтернатива обязана сохранить хотя
    бы одно слово каждой группы-якоря."""
    flat = [s for a in base["acts"] for s in a["steps"]]
    generic = generic_groups(base)
    out: Dict[str, List[str]] = {}
    later: List[str] = []
    for entry in reversed(flat):
        groups = dict.fromkeys(_groups_of(entry["step"], generic, forbidden=False) + later)
        out[entry["id"]] = [g for g in groups if _has_any(entry["step"]["text"], g)]
        later = _groups_of(entry["step"], generic, forbidden=True) + later
    return out


def leak_groups(base: Dict[str, Any]) -> Dict[str, List[str]]:
    """Группы слов собственных ожиданий шага (``expected_keywords`` и ``must_not_say`` без общих фраз отказа), которых
    в исходной фразе НЕТ: перефраза не должна их приносить - «что я говорил про чай» → «…про зелёный чай» подсказывает
    ответ и делает шаг легче, чем он есть."""
    generic = generic_groups(base)
    return {s["id"]: [g for g in _groups_of(s["step"], generic, forbidden=True) if not _has_any(s["step"]["text"], g)]
            for a in base["acts"] for s in a["steps"]}


# ── Проверки ──────────────────────────────────────────────────────────────────────────────────────────────────────


def check_paraphrase(original: str, alt: str, wakes: Sequence[str], groups: Sequence[str] = (),
                     leaks: Sequence[str] = ()) -> List[str]:
    """Что в перефразе нарушено (пусто - годится): вейк, имена, числа, цитаты, якоря ожиданий, подсказка ответа
    (``leaks`` - :func:`leak_groups`), не копия."""
    problems: List[str] = []
    if leading_wake(original, wakes) != leading_wake(alt, wakes):
        problems.append(f"вейк в начале: было {leading_wake(original, wakes)!r}, стало {leading_wake(alt, wakes)!r}")
    if len(wakes_in(original, wakes)) != len(wakes_in(alt, wakes)):
        problems.append(f"вейк-слов: было {wakes_in(original, wakes)}, стало {wakes_in(alt, wakes)}")
    lost = [n for n in names_in(original) if n not in names_in(alt)]
    if lost:
        problems.append(f"пропали имена: {lost}")
    if numbers_in(original) != numbers_in(alt):
        problems.append(f"числа: было {numbers_in(original)}, стало {numbers_in(alt)}")
    problems += [f"цитата «{q}» не сохранена" for q in quotes_in(original) if norm(q) not in norm(alt)]
    problems += [f"нет ни одного слова якоря «{g}»" for g in groups if not _has_any(alt, g)]
    problems += [f"подсказывает ответ: слово из ожиданий шага «{g}»" for g in leaks if _has_any(alt, g)]
    if norm(alt) == norm(original):
        problems.append("не перефраза: те же слова")
    return problems


def check_translation(original: str, alt: str, wakes: Sequence[str]) -> List[str]:
    """Что в переводе нарушено: вейк (в начале исходника - ``Robbox`` в начале перевода, без вейка - без вейка), имена
    латиницей, числа (цифрами или словами), кириллицы нет."""
    problems: List[str] = []
    want = EN_WAKE.lower() if leading_wake(original, wakes) else ""
    if leading_wake(alt, wakes) != want:
        problems.append(f"вейк в начале: нужен {want!r}, стоит {leading_wake(alt, wakes)!r}")
    if len(wakes_in(original, wakes)) != len(wakes_in(alt, wakes)):
        problems.append(f"вейк-слов: было {len(wakes_in(original, wakes))}, стало {len(wakes_in(alt, wakes))}")
    lower = alt.lower()
    lost = [n for n in names_in(original) if not any(form in lower for form in NAME_STEMS[n])]
    if lost:
        problems.append(f"пропали имена: {[NAME_STEMS[n][0] for n in lost]}")
    ru, en = numbers_in(original), numbers_in(alt, EN_NUMBER_WORDS)
    en = en if 1 in ru else [n for n in en if n != 1]
    if ru != en:
        problems.append(f"числа: было {ru}, стало {en}")
    if _CYRILLIC.search(alt):
        problems.append("в переводе осталась кириллица")
    return problems


__all__ = ["EN_WAKE", "NAME_STEMS", "PARTS", "TITLES", "alternative_for", "anchors", "base_ref", "capitalized_inside",
           "check_paraphrase", "check_translation", "digest", "generic_groups", "index_of", "leading_wake", "leak_groups",
           "load_part", "names_in", "norm", "numbers_in", "part_corpus", "part_path", "quotes_in", "recount",
           "repoint", "repoint_file", "substituted", "wakes_in", "write_part"]
