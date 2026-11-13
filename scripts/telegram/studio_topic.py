"""Выбор темы поста: интерес читателя решает код, а не модель (ADR-0148).

Разбор 13.11 (issue #4702): сессия собрала 44 видимых факта и предложила один
пост — худший из них: мелкую правку голоса. Причина была не в сборе фактов,
а в выборе: тема бралась «первой в списке» (``studio_draft.KIND_PRIORITY`` +
свежесть), поэтому свежий однострочный fix всегда оказывался впереди.

Здесь порядок тем решает код — по правилам, записанными в константы:

  * вид события (KIND_WEIGHT) — прогон робота интереснее правки;
  * масштаб события (SCALE_MAX) — сколько целей прошёл робот, сколько минут
    симуляции, сколько файлов тронул PR: большое событие читателю интереснее
    мелкой правки;
  * интерес читателя (INTEREST_MAX) — видно ли по теме, что делает робот;
  * постабельность (POSTABILITY_BONUS) — есть ли своя картинка к теме;
  * повтор (REPEAT_PENALTY) — тему, которую уже показывали, уводят в конец
    очереди (факт из ленты при этом НЕ исчезает — решение Шифу 06.11);
  * свежесть — только последний признак, когда баллы равны: при равенстве
    свежий факт приоритетнее старого (в баллы свежесть не входит).

Модуль чистый: ни сети, ни модели, ни файлов. Только функции и числа.
"""

from __future__ import annotations

import re
from datetime import datetime, timezone

# --- веса вида события --------------------------------------------------------
# Сколько баллов даёт вид факта. Прогон робота — самый интересный читателю:
# это то, что он видит на складе.
KIND_WEIGHT = {
    "robot_run": 60,
    "merged_pr": 30,
    "closed_issue": 28,
    "media": 22,
    "adr": 14,
    "decisions": 12,
    "run": 10,
}
KIND_WEIGHT_OTHER = 10

# --- масштаб события ----------------------------------------------------------
# Событие тем интереснее, чем оно крупнее: больше целей, длиннее симуляция,
# больше изменённых файлов. Потолок один на все виды событий.
SCALE_MAX = 40
# Прогон робота: балл за каждую цель маршрута и за каждые 60 секунд симуляции.
SCALE_ROBOT_GOAL = 4
SCALE_ROBOT_GOALS_CAP = 20  # баллов максимум за цели
SCALE_ROBOT_MINUTE = 1
SCALE_ROBOT_MINUTES_CAP = 20  # баллов максимум за минуты симуляции
# PR: балл за каждый изменённый файл (потолок тот же).
SCALE_PR_FILE = 2

# --- интерес читателя ---------------------------------------------------------
# Темы, которые читатель канала видит в поведении робота. Тема «про репозиторий»
# читателю неинтересна, даже если правка большая: `e2e`, `refactor`, `ci`,
# `chore` — про стенд и про репозиторий (ADR-0171, раздел 2а).
INTEREST_MAX = 20
INTEREST_PER_WORD = 4
INTEREST_WORDS = (
    "навигац",
    "маршрут",
    "локализац",
    "карт",
    "склад",
    "депо",
    "заезд",
    "проезд",
    "лицо",
    "face",
    "музык",
    "music",
    "голос",
    "voice",
    "микрофон",
    "wake",
    "диалог",
    "dialog",
    "светодиод",
    "led",
    "камер",
    "camera",
    "заряд",
    "аккумулятор",
    "робот",
    "robot",
    "симуляц",
    "sim",
    "gazebo",
)

# --- постабельность ----------------------------------------------------------
# Тема с собственной картинкой лучше темы без картинки: пост выходит с
# графикой, и картинка совпадает с темой.
POSTABILITY_BONUS = 10

# --- повтор -------------------------------------------------------------------
# Тему, которую уже показывали и не взяли, предлагают снова, но в самом конце
# очереди. Штраф больше любой суммы баллов (60 + 40 + 20 + 10 = 130), поэтому
# показанная тема всегда ниже любой непоказанной, а порядок между показанными
# остаётся прежним. Из ленты факт при этом НЕ исчезает (решение Шифу 06.11):
# когда остальные темы закончатся, до неё очередь дойдёт.
MAX_SCORE = 130  # вид события + масштаб + интерес + своя графика
REPEAT_PENALTY = MAX_SCORE + 20

# Минимальная длина утверждения: короткое («новый ADR») — не тема для поста.
MIN_CLAIM_CHARS = 20

PR_NUMBER_RE = re.compile(r"#(\d+)")


def _text(fact: dict) -> str:
    """Всё, чем факт описывает событие: утверждение, источник, доказательства."""
    parts = [fact.get("claim") or "", fact.get("source") or ""]
    parts += list(fact.get("evidence") or [])
    return " ".join(str(part) for part in parts)


# --- незавершённая работа (черновик) -----------------------------------------
# Заголовок, начинающийся с «WIP », — это черновик: работу ещё не доделали,
# и показывать её читателю нельзя (ADR-0171, раздел 2а).
#
# Раньше отсев технических правок смотрел на начало названия, а «WIP
# refactor(e2e): …» начинается с «WIP », а не с «refactor(» — такая правка
# проходила в ленту как видимая и семь изменённых файлов давали ей больше
# очков, чем содержательной правке о музыке в голосе.
DRAFT_PREFIXES = ("wip ", "wip:", "[wip]", "(wip)")
CLAIM_MARKER = "влит в develop:"
# Штраф уводит черновик на самое дно: он остаётся в списке (факт из очереди
# не убирается), но темой поста не становится, пока есть обычные темы.
DRAFT_PENALTY = MAX_SCORE + 20


def is_draft(text: str) -> bool:
    """Черновик ли это: заголовок начинается с «WIP » (в любом регистре).

    Работает и с заголовком PR, и с утверждением факта («PR #N влит в develop:
    WIP …») — ради этого снимается префикс вида факта.
    """
    body = (text or "").strip()
    marker = body.lower().find(CLAIM_MARKER)
    if marker >= 0:
        # Отдельно, а не прямо в срез: black ставит перед «:» пробел, а flake8
        # (E203) за это ругается.
        start = marker + len(CLAIM_MARKER)
        body = body[start:].strip()
    return body.lower().startswith(DRAFT_PREFIXES)


def kind_score(fact: dict) -> int:
    """Баллы за вид события."""
    return KIND_WEIGHT.get(fact.get("kind") or "", KIND_WEIGHT_OTHER)


def scale_score(fact: dict) -> int:
    """Баллы за масштаб события: сколько работы за собой событие оставило.

    Для прогона робота — цели маршрута и минуты симуляции, для PR — число
    изменённых файлов. Всё берётся из чисел самого факта: выдумывать нечего.
    """
    kind = fact.get("kind") or ""
    numbers = fact.get("numbers") or {}
    score = 0
    if kind == "robot_run":
        goals = _as_number(numbers.get("goals"))
        seconds = _as_number(numbers.get("sim_s"))
        if goals is not None:
            score += min(int(goals * SCALE_ROBOT_GOAL), SCALE_ROBOT_GOALS_CAP)
        if seconds is not None:
            score += min(int(seconds // 60 * SCALE_ROBOT_MINUTE), SCALE_ROBOT_MINUTES_CAP)
    elif kind in ("merged_pr", "closed_issue"):
        score += len(fact.get("paths") or []) * SCALE_PR_FILE
    return min(score, SCALE_MAX)


def interest_score(fact: dict) -> int:
    """Баллы за интерес читателя: видно ли по теме поведение робота."""
    low = (fact.get("claim") or "").lower()
    hits = sum(1 for word in INTEREST_WORDS if _mentions(low, word))
    return min(hits * INTEREST_PER_WORD, INTEREST_MAX)


def postability_score(fact: dict, facts: list) -> int:
    """Баллы за наличие своей графики к теме.

    Своё медиа — факт вида ``media`` из того же PR или с тем же ключом.
    """
    if not _has_own_media(fact, facts):
        return 0
    return POSTABILITY_BONUS


def repeat_penalty(fact: dict) -> int:
    """Штраф за тему, которую уже показывали (счётчик показов в факте)."""
    attempts = _as_number(fact.get("attempts")) or 0
    if attempts <= 0:
        return 0
    return REPEAT_PENALTY


def draft_penalty(fact: dict) -> int:
    """Штраф за незавершённую работу: «WIP …» — черновик, не тема для канала."""
    if is_draft(fact.get("claim") or ""):
        return DRAFT_PENALTY
    return 0


def score(fact: dict, facts: list) -> int:
    """Итоговые баллы факта: чем больше, тем интереснее тема.

    Свежесть сюда НЕ входит: при равенстве баллов решает она, но сама по себе
    свежесть тему не выбирает (иначе свежий мелкий fix снова победил бы).
    """
    return (
        kind_score(fact)
        + scale_score(fact)
        + interest_score(fact)
        + postability_score(fact, facts)
        - repeat_penalty(fact)
        - draft_penalty(fact)
    )


def rank(facts: list) -> list:
    """Факты от лучшей темы к худшей.

    Порядок детерминирован: сначала баллы, потом (при равенстве) свежесть,
    потом исходный порядок — список не «прыгает» между сессиями.
    """
    usable = [f for f in facts or [] if _claim_len(f) >= MIN_CLAIM_CHARS]
    if not usable:
        return []
    rows = [(score(fact, usable), _time_key(fact), index, fact) for index, fact in enumerate(usable)]
    rows.sort(key=lambda row: (-row[0], -row[1], row[2]))
    return [row[3] for row in rows]


def _claim_len(fact: dict) -> int:
    return len((fact.get("claim") or "").strip())


def _mentions(low_text: str, word: str) -> bool:
    """Есть ли слово в тексте. Для латиницы — по границам слова.

    Без границ «led» нашлось бы внутри «travelled», а «sim» — внутри «simple».
    """
    if word.isascii():
        return bool(re.search(rf"\b{re.escape(word)}\b", low_text))
    return word in low_text


def _as_number(value) -> float | None:
    """Число из факта или None — если числа нет, балл не начисляем."""
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    try:
        return float(str(value).replace(",", "."))
    except (TypeError, ValueError):
        return None


def _fact_time(fact: dict) -> datetime | None:
    """Момент события из факта: для прогона — конец прогона, для PR — merge."""
    for field in ("ended_at", "merged_at", "closed_at", "created_at"):
        moment = _parse_time(fact.get(field))
        if moment is not None:
            return moment
    return None


def _parse_time(value) -> datetime | None:
    """Разбор метки времени: ``Z`` и ``+00:00`` (Python 3.10 не понимает ``Z``)."""
    if not value:
        return None
    text = str(value).strip()
    if text.endswith("Z"):
        text = text[:-1] + "+00:00"
    try:
        moment = datetime.fromisoformat(text)
    except ValueError:
        return None
    if moment.tzinfo is None:
        moment = moment.replace(tzinfo=timezone.utc)
    return moment


def _time_key(fact: dict) -> float:
    """Числовой ключ свежести (для сортировки): больше — свежее."""
    moment = _fact_time(fact)
    if moment is None:
        return 0.0
    return moment.timestamp()


def _has_own_media(fact: dict, facts: list) -> bool:
    """Есть ли у темы своя картинка: медиа-факт того же PR или с тем же ключом."""
    key = fact.get("key")
    numbers = _pr_numbers(fact)
    for other in facts or []:
        if other.get("kind") != "media":
            continue
        if key and other.get("key") == key:
            return True
        if numbers & _pr_numbers(other):
            return True
    return False


def _pr_numbers(fact: dict) -> set:
    """Номера PR, которые называет факт."""
    found = set(PR_NUMBER_RE.findall(_text(fact)))
    own = str(fact.get("pr") or "").strip().lstrip("#")
    if own:
        found.add(own)
    return found
