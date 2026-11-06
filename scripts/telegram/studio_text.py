"""Генератор текста поста для канала: факты → MiniMax → JSON → проверка кодом.

Правила ролей (ADR-0148 «код решает, LLM говорит»):
  * факты собирает код (studio_facts.py) — у каждого есть источник;
  * текст пишет модель, но только по этим фактам и правилам канала;
  * проверяет код: числа и имена собственные в тексте есть среди фактов, лимиты и разметку
    проверяет channel_lib.validate, лиц/внутренних IP/ключей быть не должно.

Модель — MiniMax-M3.1-Flash (та же, что у профилей Hermes). У неё thinking выключить
нельзя: API отвечает HTTP 400, поэтому глубина размышлений задаётся reasoning_effort
(см. ниже). Для MiniMax-M3 выключение через thinking={"type": "disabled"} работает.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "telegram"))

import channel_lib as lib  # noqa: E402

DEFAULT_MODEL = "MiniMax-M3.1-Flash"
API_URL = "https://api.minimax.io/v1/chat/completions"

# Модели MiniMax и их «мышление» (проверено вживую 06.11.2026, см. также
# src/rob_box_llm/rob_box_llm/providers/minimax.py::apply_model_thinking):
#
#   MiniMax-M3           — thinking выключается: thinking={"type": "disabled"}
#   MiniMax-M3.1-Flash   — thinking выключить НЕЛЬЗЯ (HTTP 400), модель требует
#                          adaptive thinking и умеет брать «глубину» reasoning_effort.
#                          Без reasoning_effort размышления не ограничены: они съедают
#                          весь max_tokens, и content приходит ПУСТЫМ (finish_reason=length).
#                          Поэтому для Flash всегда ставим reasoning_effort.
THINKING_DISABLED_MODELS = ("MiniMax-M3",)
DEFAULT_REASONING_EFFORT = "low"


def thinking_params(model: str) -> dict:
    """Как выразить «думай меньше» для конкретной модели MiniMax."""
    if model in THINKING_DISABLED_MODELS:
        return {"thinking": {"type": "disabled"}}
    return {"reasoning_effort": DEFAULT_REASONING_EFFORT}


# Числа в тексте: «48 часов», «3 попытки», «99.5%». Дробные и проценты — отдельно.
NUMBER_RE = re.compile(r"\d+(?:[.,]\d+)?")
# Имена собственные — только технические: латиница в CamelCase (Nav2, RTABMap) или
# snake_case (done_claim). Русские слова с заглавной буквы (начало предложения) не проверяем:
# иначе валидатор ругается на «Проверили», «Иначе» и отбрасывает нормальные тексты.
PROPER_RE = re.compile(r"\b[A-Za-z][A-Za-z0-9]*[a-z][A-Za-z0-9]*\b|\b[a-z][a-z0-9]*(?:_[a-z0-9]+)+\b")

# Слова латиницей, которые встречаются в любом тексте и фактом не являются.
STOP_PROPER = {
    "PR",
    "PRs",
    "HTTP",
    "GitHub",
    "API",
    "HTML",
    "JSON",
    "ID",
    "OK",
    "MiniMax",
    "Telegram",
    "Robot",
    "Dev",
    "Ops",
    "Release",
    "Nightly",
    "CI",
    "CD",
    "TODO",
    "FIXME",
}

# Кириллические имена: любая заглавная кириллица вне стоп-слов — кандидат в имя.
# Имя может стоять в начале фразы («Саша проверил…») — это самое естественное
# место для имени, поэтому начало предложения не даёт права её пропустить.
# Ложные срабатывания отсекаются стоп-словами (STOP_CYRILLIC) и фактами.
CYRILLIC_NAME_RE = re.compile(r"\b[А-ЯЁ][а-яё]{2,}\b")

# Окончания, по которым видно, что слово — обычное глагольное слово, а не имя.
# Без этого фильтра каждое предложение поста начиналось бы с «ложного имени»
# («Проверяем», «Работает», «Сделали») и пост отклонялся бы впустую.
# Корни глаголов прошедшего времени. Нужны, чтобы слова на -ла/-ли не
# считались именами: «Настроили» и «Настроила» дают один корень (слово без
# последних двух букв), а «Данила», «Мила», «Людмила» именами остаются —
# их корни («дани», «мил», «людмил») в списке отсутствуют.
VERB_STEMS = frozenset(
    {
        "провер",
        "сдела",
        "настро",
        "добав",
        "измен",
        "постав",
        "убра",
        "почин",
        "поня",
        "нача",
        "законч",
        "продолж",
        "повтор",
        "получ",
        "собер",
        "взял",
        "нашл",
        "прочит",
        "откры",
        "закры",
        "разобр",
        "показа",
        "принес",
        "пришл",
        "увид",
        "выясн",
        "остав",
        "переда",
        "запуст",
        "додела",
        "опубликов",
        "подтверд",
        "сохран",
        "обработа",
        "выгруз",
        "скача",
        "загруз",
        "сравн",
        "напис",
        "скопиров",
        "приня",
        "реши",
        "отмен",
        "верну",
        "подари",
        "пойма",
        "учи",
        "мыл",
        "замер",
        "поменя",
        "обнов",
        "синхронизиров",
        "разверн",
        "перенес",
        "проанализиров",
        "вычисл",
        "вылож",
        "вычит",
        "уточн",
        "откат",
        "забыл",
        "забы",
        "съехал",
        "съеха",
        "поехал",
        "поеха",
        "доехал",
        "доеха",
        "наехал",
        "сел",
        "работа",
        "стои",
        "заня",
        "выигр",
        "проигра",
        "помещ",
        "помест",
        "пересел",
        "сел",
        "вынул",
        "выну",
        "надел",
        "над",
        "залез",
        "зале",
        "запер",
        "запер",
        "открыл",
        "закрыл",
        "слыш",
        "крич",
        "показал",
        "показ",
        "покрут",
        "крут",
        "прокрут",
        "зафиксирова",
        "задефиксирова",
        "перекачал",
        "перекач",
        "откатил",
        "откат",
    }
)

VERBISH_ENDINGS = (
    "ем",
    "ём",
    "ет",
    "ит",
    "ат",
    "ят",
    "ут",
    "ют",
    "но",
    "емся",
    "лся",
    "лась",
    "лось",
    "ать",
    "ять",
    "ить",
    "ешь",
    "ите",
    "ают",
    "яют",
    "уем",
    "аем",
    "яем",
    "ло",
    "ны",
    "ем",
)

# Слова канала, которые всегда с заглавной и не являются именами.
STOP_CYRILLIC = {
    "Запуск",
    "Проверка",
    "Сборка",
    "Обновление",
    "Исправление",
    "Замена",
    "Настройка",
    "Тест",
    "Работа",
    "Результат",
    "Результаты",
    "Ошибка",
    "Проблема",
    "Итог",
    "Итоги",
    "Вывод",
    "Выводы",
    "Замечание",
    "Замечания",
    "Причина",
    "Цель",
    "Задача",
    "План",
    "Шаг",
    "Шаги",
    "Новость",
    "Новости",
    "Пример",
    "Вариант",
    "Способ",
    "Метод",
    "Прогон",
    "Замер",
    "Съёмка",
    "Сбор",
    "Ветка",
    "Продукт",
    "Линия",
    "Точка",
    "Команда",
    "Сценарий",
    "Формат",
    "РОББОКС",
    "Робот",
    "Робота",
    "Роботу",
    "Роботом",
    "Роботы",
    "Мы",
    "Нам",
    "Наш",
    "Наша",
    "Наши",
    "Шифу",
    "Привет",
    "Пост",
    "Посты",
    "Канал",
    "Канала",
    "Слот",
    "Ссылка",
    "Так",
    "Тогда",
    "Теперь",
    "Если",
    "Когда",
    "Что",
    "Это",
    "Для",
    "Он",
    "Она",
    "Оно",
    "Они",
    "Его",
    "Её",
    "Их",
    "Вот",
    "Да",
    "Нет",
    "Как",
    "Где",
    "Зачем",
    "Почему",
    "Смотри",
    "Сделал",
    "Сделано",
    "Работает",
    "Работа",
    "Часть",
    "Раз",
    "Пара",
    "Вместе",
    "Есть",
    "Быть",
    "Главное",
    "Суть",
    "Идея",
    "Идеи",
    "Факт",
    "Факты",
    "Источник",
    "Разбор",
    "Замер",
    "Замеры",
    "Результат",
    "Проверка",
    "Проверки",
    "Правило",
    "Правила",
    "Смысл",
    "Вывод",
    "Причина",
    "Причины",
    "Симптом",
    "Ошибка",
    "Ошибки",
    "Ремонт",
    "Прогон",
    "Прогоны",
    "Шаг",
    "Шаги",
    "Слоты",
    "Тема",
    "Темы",
    "Угол",
    "Углы",
    "Съёмка",
    "Сцена",
    "Сценарий",
    "Ролик",
    "Ролики",
    "Кадр",
    "Кадры",
    "Видео",
    "Гифка",
    "Гифки",
    "Инфографика",
    "Файл",
    "Файлы",
    "Строка",
    "Строки",
    "Порядок",
    "Задача",
    "Задачи",
    "Вопрос",
    "Вопросы",
    "Ответ",
    "Ответы",
    "Итак",
    "Короче",
    "Потом",
    "Сейчас",
    "Раньше",
    "Позже",
    "Здесь",
    "Там",
    "Иначе",
    "Поэтому",
    "Проверили",
    "Изменение",
    "Изменения",
    "Оговорка",
    "Оговорки",
    "Пример",
    "Примеры",
    "Мысль",
    "Мысли",
    "Сводка",
    "Итог",
    "Итоги",
    "Также",
    "Кроме",
    "После",
    "Перед",
    "Между",
    "Через",
    "Вместо",
    "Согласно",
    "Благодаря",
    "Наконец",
    "Однако",
    "Хотя",
    "Пока",
    "Потому",
    "Проверяем",
    "Проверим",
    "Работали",
    "Сделали",
    "Сделаем",
    "Поставили",
    "Добавили",
    "Изменили",
    "Настроили",
    "Убрали",
    "Поняли",
    "Починили",
    "Идём",
    "Идем",
    "Пошли",
    "Ждём",
    "Ждем",
    "Начали",
    "Закончили",
    "Продолжили",
    "Повторили",
    "Получилось",
    "Вышло",
    "Выходит",
    "Кажется",
    "Похоже",
    "Значит",
    "Сразу",
    "Правда",
    "Честно",
    "Кстати",
    "Сначала",
    "Затем",
    "Сегодня",
    "Вчера",
    "Завтра",
    "Утром",
    "Вечером",
    "Днём",
    "Ночью",
    "Отдельно",
    "Почти",
    "Кто",
    "Куда",
    "Откуда",
    "Сколько",
    "Какой",
    "Какая",
    "Какие",
    "Чей",
    "Чья",
    "Мой",
    "Твой",
    "Ваш",
    "Им",
    "Ему",
    "Ей",
    "Все",
    "Всё",
    "Каждый",
    "Каждая",
    "Любой",
    "Любая",
    "Самый",
    "Самая",
    "Первое",
    "Первая",
    "Второе",
    "Третье",
    "Новое",
    "Новая",
    "Старый",
    "Старая",
    "Свежий",
    "Свежая",
    "Дальше",
    "Сперва",
    "Два",
    "Три",
    "Десять",
    "Сто",
    "Мало",
    "Много",
    "Больше",
    "Меньше",
    "Хорошо",
    "Плохо",
    "Быстро",
    "Медленно",
    "Легко",
    "Сложно",
    "Просто",
    "Понятно",
    "Странно",
    "Жаль",
    "Зато",
    "Ну",
    "А",
    "И",
    "Но",
    "Или",
    "Чтобы",
    "Без",
    "При",
    "Про",
    "Из",
    "По",
    "На",
    "За",
    "К",
    "О",
    "У",
    "С",
    "В",
    "Во",
    "Со",
    "От",
    "До",
    "Ты",
    "Вы",
    "Этот",
    "Эта",
    "Эти",
    "Такой",
    "Такая",
    "Сам",
    "Всю",
    "Сюда",
    "Туда",
    "Оттуда",
    "Вслух",
    "Против",
    "Около",
    "Вдруг",
    "Вроде",
    "Едва",
    "Чуть",
    "Совсем",
    "Довольно",
    "Слишком",
    "Очень",
    "Крайне",
    "Весь",
    "Вся",
    "Другой",
    "Другая",
    "Другие",
    "Первый",
    "Второй",
    "Третий",
    "Нынешний",
    "Текущий",
    "Новый",
    "Команда",
    "Команды",
    "Кейс",
    "Кейсы",
    "Штука",
    "Человек",
    "Люди",
}


def extract_fact_text(facts: list) -> str:
    """Всё, что модель имеет право утверждать: утверждения, источники, пути."""
    parts = []
    for fact in facts:
        claim = fact.get("claim", "")
        source = fact.get("source", "")
        parts.append(f"- {claim} (источник: {source})")
        for evidence in fact.get("evidence", []) or []:
            parts.append(f"  - доказательство: {evidence}")
    return "\n".join(parts)


def load_rules(repo_root: Path) -> str:
    """Правила канала: тон, теги, лимиты, приватность."""
    path = repo_root / "docs" / "process" / "telegram-channel.md"
    if not path.is_file():
        return "Правила канала не найдены — пиши просто, по-русски, короткими абзацами."
    text = path.read_text(encoding="utf-8")
    return text[:12000]


def build_prompt(facts: list, rules: str, media_ref: str = "") -> str:
    """Промпт: только факты и правила, никаких просьб «придумай».

    По брифу сначала в личку уходит СПИСОК ИДЕЙ, а не готовые посты: текст пишется
    позже, только для идеи, которую Шифу взял. Поэтому здесь body_html не просим.
    """
    return (
        "Ты готовишь СПИСОК ИДЕЙ для Telegram-канала проекта робота. У тебя есть только "
        "факты ниже и правила канала. НИЧЕГО, чего нет в фактах, ты не пишешь: ни новых "
        "чисел, ни новых имён, ни предположений. Если фактов мало — верни пустой массив ideas.\n\n"
        f"ПРАВИЛА КАНАЛА:\n{rules}\n\n"
        f"ФАКТЫ (источник каждого обязателен):\n{extract_fact_text(facts)}\n\n"
        f"МЕДИА: {media_ref or 'готового медиа нет — в поле media_ref оставь пустым'}\n\n"
        "Верни СТРОГО JSON без пояснений:\n"
        '{"ideas": [{"id": "i1", "title": "о чём пост — одна строка", '
        '"facts": ["использованные факты"], "show": "что показать: готовое медиа из '
        'репозитория, инфографика или «нужна съёмка»", "deadline": "примерный срок", '
        '"media_ref": "путь к готовому медиа или пустая строка"}]}\n\n'
        "Требования: title — до 80 знаков, без кавычек; facts — только точные формулировки "
        "фактов выше, иначе идея будет отклонена; show — что именно показать читателю; "
        "deadline — когда лучше опубликовать (например «на следующей неделе»); "
        "media_ref — путь к файлу из репозитория, если он уже готов, иначе пустая строка."
    )


def call_minimax(prompt: str, model: str = DEFAULT_MODEL, api_key: Optional[str] = None, timeout: int = 240) -> dict:
    """MiniMax → JSON. Ошибка — не исключение наружу, а {"error": ...}."""
    key = api_key or os.environ.get("MINIMAX_API_KEY", "")
    if not key:
        return {"error": "MINIMAX_API_KEY не задан"}
    payload = {
        "model": model,
        "messages": [{"role": "user", "content": prompt}],
        "temperature": 0.3,
        "max_tokens": 8000,
        **thinking_params(model),
    }
    try:
        import requests

        response = requests.post(API_URL, json=payload, headers={"Authorization": f"Bearer {key}"}, timeout=timeout)
    except Exception as e:  # noqa: BLE001 — текст ошибки без ключа
        return {"error": f"сеть MiniMax: {type(e).__name__}"}
    if response.status_code != 200:
        return {"error": f"HTTP {response.status_code}: {response.text[:200]}"}
    try:
        data = response.json()
    except ValueError:
        return {"error": "ответ MiniMax не JSON"}
    choice = (data.get("choices") or [{}])[0]
    content = choice.get("message", {}).get("content", "") or ""
    if not content.strip():
        # Размышления съели весь бюджет токенов — у Flash это без reasoning_effort.
        finish = choice.get("finish_reason")
        usage = data.get("usage") or {}
        return {
            "error": f"модель вернула пустой ответ (finish_reason={finish}, "
            f"completion_tokens={usage.get('completion_tokens')}) — "
            "размышления съели max_tokens, нужен reasoning_effort"
        }
    if "<think>" in content:
        content = content.split("</think>")[-1]
    start, end = content.find("{"), content.rfind("}")
    if start < 0 or end <= start:
        return {"error": "в ответе нет JSON", "raw": content[:400]}
    try:
        stop = end + 1
        return json.loads(content[start:stop])
    except ValueError as e:
        return {"error": f"JSON не разобрался: {e}", "raw": content[:400]}


def _fact_numbers(fact: dict) -> set:
    """Числа, которые есть в конкретном факте.

    Из чисел факта убираем номера прогонов (8+ цифр) и даты: они не разрешают
    упоминать «12 м» в тексте. Раньше разрешалось всё, что встречалось где угодно,
    и «12» из run_id legitimировал «12 метров».
    """
    text = " ".join(
        [
            fact.get("claim", "") or "",
            *(fact.get("evidence", []) or []),
        ]
    )
    # Даты и метки времени — не числа утверждения. Метку времени надо убирать
    # целиком: «2026-11-06T12:30Z» иначе оставляет «12» и «30», и они
    # legitimировали бы «12 м» в тексте.
    text = re.sub(r"\b\d{4}-\d{2}-\d{2}T\d{2}:\d{2}(?::\d{2})?Z?\b", " ", text)
    text = re.sub(r"\b\d{4}-\d{2}-\d{2}\b", " ", text)
    text = re.sub(r"\b\d{2}:\d{2}\b", " ", text)
    numbers = set()
    for match in NUMBER_RE.findall(text):
        digits = match.replace(",", "").replace(".", "")
        if len(digits) >= 8:
            # Идентификатор прогона: цифр много, смысла в тексте поста не несёт.
            continue
        numbers.add(match.replace(",", "."))
    return numbers


def _fact_refs(fact: dict) -> set:
    """Номера PR/issue из источника факта: «PR #4502» — их можно упоминать в тексте."""
    return set(re.findall(r"#(\d{1,6})\b", fact.get("source") or ""))


def _claim_matches_fact(claim: str, fact_claim: str) -> bool:
    """Совпадает ли источник, указанный моделью, с фактом.

    Модель цитирует факт — но не может его ДОПИСАТЬ. Раньше сверка шла
    по префиксу в обе стороны, и приписка «12 м, Саша рад» проходила:
    начиналась с факта «12 м», а значит «содержала» его. Теперь приписка
    с именем или числом, которых в факте нет, — уже не совпадение.
    """
    raw_claim = (claim or "").strip()
    raw_fact = (fact_claim or "").strip()
    low_claim = raw_claim.lower()
    low_fact = raw_fact.lower()
    if not low_claim or not low_fact:
        return False
    if low_fact.startswith(low_claim):
        # Модель процитировала часть факта — законно.
        return True
    if not low_claim.startswith(low_fact):
        return False
    cut = len(raw_fact)
    tail = raw_claim[cut:]
    if not tail.strip(" ,;:-—."):
        return True
    # Хвост есть — проверяем, что в нём нет придуманного: чисел и имён.
    if re.search(r"\d", tail):
        return False
    if re.search(r"[А-ЯЁ][а-яё]{1,}", tail):
        return False
    if re.search(r"\b(?:раз|два|три|четыре|пять|шесть|семь|восемь|девять|десять)\b", tail):
        return False
    return True


# Частые русские имена: слово из этого списка — человек, а не термин.
FIRST_NAMES = frozenset(
    {
        "александр",
        "александра",
        "алексей",
        "алексея",
        "анастасия",
        "андрей",
        "анна",
        "антон",
        "артём",
        "артем",
        "борис",
        "вадим",
        "валентин",
        "валерий",
        "василий",
        "василий",
        "вениамин",
        "виктор",
        "виктория",
        "виталий",
        "владимир",
        "владислава",
        "вячеслав",
        "геннадий",
        "георгий",
        "григорий",
        "даниил",
        "данила",
        "данил",
        "денис",
        "дмитрий",
        "евгений",
        "егор",
        "елена",
        "елисей",
        "емельян",
        "ефим",
        "жанна",
        "захар",
        "иван",
        "иванна",
        "игорь",
        "илья",
        "ирина",
        "кирилл",
        "константин",
        "лаврентий",
        "леонид",
        "леонид",
        "любовь",
        "людмила",
        "максим",
        "марк",
        "марина",
        "мартин",
        "матвей",
        "мария",
        "михаил",
        "мила",
        "михаил",
        "надежда",
        "николай",
        "никита",
        "олив",
        "ольга",
        "павел",
        "павел",
        "пётр",
        "петр",
        "платон",
        "полина",
        "прасковья",
        "пётр",
        "рафаил",
        "римма",
        "роберт",
        "roman",
        "россия",
        "руслан",
        "самвел",
        "светлана",
        "сергей",
        "севастьян",
        "симеон",
        "соломония",
        "софия",
        "степан",
        "тамара",
        "тарас",
        "теодосий",
        "тимур",
        "тимофей",
        "трофим",
        "фаина",
        "фёдор",
        "федор",
        "фома",
        "фёдор",
        "харитон",
        "юрий",
        "юлия",
        "яков",
        "ярослав",
        "саша",
        "сабрина",
        "севастьян",
        "эдуард",
        "эльвира",
        "эмма",
        "яна",
        "яндекс",
    }
)

# Окончания отчеств и фамилий: «Ильич», «Ивановна», «Петров».
NAME_ENDINGS = (
    "ич",
    "ична",
    "ичну",
    "ичей",
    "овна",
    "евна",
    "ична",
    "ын",
    "ин",
    "ов",
    "ев",
    "ёв",
    "ук",
    "юк",
    "як",
    "енко",
    "ич",
)


def _looks_like_name(word: str, plain: str) -> bool:
    """Похоже ли это слово на имя человека.

    Три признака: есть в списке имён, оканчивается как отчество или фамилия,
    либо стоит рядом с другим заглавленным словом («Иван Иванов»).
    """
    lowered = word.lower()
    # Ищем и полную форму («мила»), и основу («мил»): в списке имена записаны
    # в разных формах, и «Мила» обязана ловиться.
    stem = lowered.rstrip("аяеию")
    if lowered in FIRST_NAMES or stem in FIRST_NAMES:
        return True
    if word.lower().endswith(NAME_ENDINGS):
        return True
    # Рядом стоит другое заглавленное слово — похоже на «имя фамилия».
    pattern = rf"\b{re.escape(word)}\s+[А-ЯЁ][а-яё]{{2,}}\b"
    if re.search(pattern, plain):
        return True
    return False


def _is_common_word(word: str, plain: str) -> bool:
    """Обычное ли это слово, а не имя.

    1. Если то же слово есть в тексте в нижнем регистре — это обычное слово:
       «Запуск стенда прошёл, запуск занимает минуту».
    2. Если слово выведено от глагола («Запуск» от «запустили») — тоже
       обычное слово: корни глаголов лежат в VERB_STEMS.
    """
    lowered = word.lower()
    if lowered != plain.lower() and re.search(rf"(?<![А-ЯЁ]){re.escape(lowered)}\b", plain):
        return True
    for length in (5, 6, 4):
        prefix = lowered[:length]
        if len(prefix) < 4:
            continue
        for stem in VERB_STEMS:
            if stem.startswith(prefix):
                return True
    return False


def check_against_facts(body: str, facts: list, own_facts: Optional[list] = None) -> list:
    """Каждое число и имя собственное в тексте должно быть среди фактов.

    own_facts — факты конкретной идеи (её post_id). Если он передан, сверка идёт
    только с ними: число из чужого факта того же поста не должно legitimировать
    утверждение. Иначе — со всеми переданными фактами.
    """
    # Пустой список own_facts — это «сверяться не с чем», а не «сверяйся со всем».
    # Различать None (не передали) и [] (передали пустым) обязательно: иначе
    # число из чужого факта проходило проверку.
    scope = facts if own_facts is None else own_facts
    problems = []
    # HTML убираем: атрибуты (<a href="...">) — не имена собственные, а синтаксис разметки.
    plain = re.sub(r"<[^>]+>", " ", body)

    allowed = set()
    refs = set()
    for fact in scope:
        allowed |= _fact_numbers(fact)
        refs |= _fact_refs(fact)
    for match in NUMBER_RE.findall(plain):
        normalized = match.replace(",", ".")
        digits = match.replace(",", "").replace(".", "")
        if len(digits) >= 8:
            problems.append(f"идентификатор прогона «{match}» не место в тексте поста")
            continue
        if normalized not in allowed and match not in refs:
            problems.append(f"число «{match}» не найдено среди фактов")

    # Имена проверяем по ВСЕМ фактам сессии, числа — только по своим.
    # Разница принципиальная: число в тексте — это утверждение, и оно должно
    # опираться на факты именно этого поста. Имя — это слово проекта: «Параметры»
    # может встретиться в другом факте сессии, и это не выдумка. Выдуманное имя
    # («Саша», «Данила») не встречается нигде — и по-прежнему ловится.
    name_scope = facts if facts else scope
    blob = " ".join(
        [
            part
            for fact in name_scope
            for part in [fact.get("claim", "") or "", fact.get("source", "") or "", *(fact.get("evidence", []) or [])]
        ]
    ).lower()
    for match in PROPER_RE.findall(plain):
        if match in STOP_PROPER:
            continue
        if match.lower() not in blob:
            problems.append(f"имя «{match}» не найдено среди фактов")

    for match in CYRILLIC_NAME_RE.findall(plain):
        if match in STOP_CYRILLIC:
            continue
        lowered = match.lower()
        if _is_common_word(match, plain):
            continue
        # Обычное глагольное слово — не имя: «Проверяем», «Сделали», «Работает».
        if lowered.endswith(VERBISH_ENDINGS):
            continue
        # Слова на -ла/-ли: «Настроили» — глагол, «Данила» — имя.
        # Различаем по корню: у обеих форм глагола корень один.
        if lowered.endswith(("ла", "ли")):
            # «Настроили» — глагол, «Данила» — имя. Различаем по корню:
            # у форм на -ли и -ла корень один (слово без двух букв),
            # а у части глаголов отличается на одну букву — берём оба.
            core = lowered[:-2]
            if core in VERB_STEMS or core[:-1] in VERB_STEMS:
                continue
        if not _looks_like_name(match, plain):
            # Обычное слово с заглавной буквы — не имя.
            continue
        if lowered not in blob:
            problems.append(f"имя «{match}» не найдено среди фактов")
    return problems


def match_fact_keys(idea: dict, facts: list) -> list:
    """Ключи фактов, которые идея действительно использует.

    Модель возвращает тексты утверждений, а не ключи. Сопоставляем по началу текста:
    если утверждение модели начинается с утверждения факта — факт использован.
    """
    used = []
    for claim in idea.get("facts") or []:
        claim = (claim or "").strip().lower()
        if not claim:
            continue
        for fact in facts:
            fact_claim = (fact.get("claim") or "").strip().lower()
            if not fact_claim:
                continue
            if claim.startswith(fact_claim[:60]) or fact_claim.startswith(claim[:60]):
                key = fact.get("key")
                if key and key not in used:
                    used.append(key)
                break
    return used


def extract_tags(body: str) -> list:
    """Хештеги из текста — по порядку появления, только из словаря канала.

    Реализация — в channel_lib (общий модуль канала), здесь только реэкспорт:
    так правило хештегов одно на канал, а не по копии в каждом скрипте.
    """
    return lib.extract_tags(body)


def validate_idea(idea: dict, facts: list) -> list:
    """Проверка идеи кодом: тема, источники, что показать, срок.

    Идея — это ещё не пост: текста в ней нет by design (бриф: сначала список идей).
    Поэтому проверяем то, что в идее есть.
    """
    problems = []
    title = (idea.get("title") or "").strip()
    if not title:
        problems.append("пустая тема идеи")
    elif len(title) > 120:
        problems.append(f"тема {len(title)} знаков, много")
    if not (idea.get("facts") or []):
        problems.append("у идеи нет фактов")
    # Поле facts из ответа модели проверяем: Шифу не должен увидеть выдуманные источники.
    for claim in idea.get("facts") or []:
        claim = (claim or "").strip().lower()
        if not claim:
            continue
        known = any(_claim_matches_fact(claim, (fact.get("claim") or "").strip()) for fact in facts)
        if not known:
            problems.append(f"источник «{claim[:40]}» не найден среди фактов")
    title = (idea.get("title") or "").strip()
    if title:
        # Заголовок — тоже текст для читателя: имя или ключ в нём недопустимы.
        problems += check_against_facts(title, facts)
    if not (idea.get("show") or "").strip():
        problems.append("не сказано, что показать")
    if not (idea.get("deadline") or "").strip():
        problems.append("не назван примерный срок")
    return problems


def validate_post(
    body: str,
    tags: list,
    facts: list,
    has_media: bool = False,
    own_facts: Optional[list] = None,
) -> list:
    """Проверка готового поста кодом: факты, лимиты, разметка, теги, приватность.

    own_facts — факты именно этой идеи: сверка идёт только с ними, иначе число
    из чужого факта того же поста legitimировало бы выдумку.
    """
    problems = check_against_facts(body, facts, own_facts=own_facts)
    if not body.strip():
        problems.append("пустой текст")
    if not 2 <= len(tags) <= 5:
        problems.append(f"хештегов {len(tags)}, нужно 2–5")
    for tag in tags:
        if tag not in lib.TAG_DICTIONARY:
            problems.append(f"хештег {tag} не в словаре канала")
        if tag not in body:
            problems.append(f"хештег {tag} не упомянут в тексте")
    limit = lib.CAPTION_LIMIT if has_media else lib.MESSAGE_LIMIT
    length = lib.visible_length(body)
    if length > limit:
        problems.append(f"текст {length} знаков, лимит {limit}")
    problems += lib.html_problems(body)
    for pattern, what in lib.FORBIDDEN:
        if pattern.search(body):
            problems.append(f"в тексте {what}")
    return problems


def build_post_prompt(idea: dict, facts: list, rules: str, instruction: str = "") -> str:
    """Промпт производства: пишем ТЕКСТ поста по уже согласованной идее.

    instruction — не «новый пост», а правка существующего текста: так
    просит Шифу кнопками «короче», «проще», «подача» (решение 06.11).
    """
    head = (
        "Ты пишешь текст поста для Telegram-канала проекта робота. Ничего, "
        "что не сказано в идее, добавлять нельзя — пиши только то, на что она "
        "и факты, на которые она опирается. НИЧЕГО, чего нет в идее или в фактах.\n"
        f"ПРАВИЛА КАНАЛА:\n{rules}\n\n"
        f"ИДЕЯ: {idea.get('title', '')}\n"
        f"ЧТО ПОКАЗАТЬ: {idea.get('show', '')}\n"
        f"СРОК: {idea.get('deadline', '')}\n"
        f"МЕДИА: {idea.get('media_ref') or 'готового медиа нет'}\n\n"
        f"ФАКТЫ:\n{extract_fact_text(facts)}\n\n"
    )
    if instruction:
        head += f"ПРАВКА ТЕКСТА (это правка того же поста, а не новый пост): {instruction}\n\n"
    tail = (
        "Верни СТРОГО JSON:\n"
        '{"body_html": "текст поста", "tags": ["#роббокс"]}\n\n'
        "Требования к body_html: Telegram HTML разрешает только теги "
        "b, strong, i, em, u, ins, s, strike, del, a, code, pre, blockquote, tg-spoiler, tg-emoji. "
        "Других тегов быть не должно: ни <br>, ни <ul>, ни <li>, ни <ol>, ни <p>, ни <h1>. "
        "Абзацы разделяй переводом строки, список — строками с «-». "
        "Тег <a> со ссылкой на источник разрешён. "
        "Цель 400–700 знаков; хештеги только из словаря канала; "
        "никаких внутренних IP, лиц людей и ключей."
    )
    return head + tail


def produce_post(
    idea: dict,
    facts: list,
    model: str = DEFAULT_MODEL,
    repo_root: Path = REPO_ROOT,
    instruction: str = "",
) -> dict:
    """Произвести пост по согласованной идее: модель пишет, код проверяет."""
    prompt = build_post_prompt(idea, facts, load_rules(repo_root), instruction=instruction)
    result = call_minimax(prompt, model=model)
    if "error" in result:
        return {"error": result["error"]}
    body = (result.get("body_html") or "").strip()
    tags = result.get("tags") or []
    if not tags:
        tags = lib.extract_tags(body)
    problems = validate_post(
        body,
        tags,
        facts,
        has_media=bool(idea.get("media_ref")),
        own_facts=facts if idea.get("fact_keys") else None,
    )
    return {
        "body_html": body,
        "tags": tags,
        "problems": problems,
        "ok": not problems,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--facts", type=Path, required=True, help="JSON с фактами (вывод studio_facts.py)")
    ap.add_argument("--out", type=Path, help="куда записать идеи (JSON)")
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    ap.add_argument("--produce", type=Path, help="JSON с одной согласованной идеей: написать текст поста")
    ap.add_argument("--instruction", default="", help="правка текста (кнопки итерации)")
    args = ap.parse_args()

    data = json.loads(args.facts.read_text(encoding="utf-8"))
    facts = data.get("facts", data) if isinstance(data, dict) else data
    visible_facts = [f for f in facts if f.get("visible")]

    # Режим производства: пишем текст поста по согласованной идее.
    if args.produce:
        idea = json.loads(args.produce.read_text(encoding="utf-8"))
        if isinstance(idea, dict) and "idea" in idea:
            idea = idea["idea"]
        # Факты идеи — по её ключам, а не по всем: текст должен опираться на своё.
        keys = set(idea.get("fact_keys") or [])
        own = [f for f in facts if f.get("key") in keys] or visible_facts
        result = produce_post(idea, own, model=args.model, repo_root=args.repo_root)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result.get("ok") else 1

    if not visible_facts:
        payload = {"ideas": [], "reason": "нет видимых фактов — показывать нечего"}
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0

    prompt = build_prompt(visible_facts, load_rules(args.repo_root))
    result = call_minimax(prompt, model=args.model)
    if "error" in result:
        print(json.dumps({"ideas": [], "error": result["error"]}, ensure_ascii=False, indent=2))
        return 1

    ideas = result.get("ideas") or []
    # Идентификатор задаёт код: модель каждый раз начинает с «i1», и идеи разных сессий
    # слипались бы в одну. Префикс с датой сессии делает их разными и узнаваемыми.
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d")
    checked = []
    for index, idea in enumerate(ideas, start=1):
        idea = dict(idea)
        idea["id"] = f"i{stamp}-{index}"
        # Ключи фактов: по ним студия поймёт, что событие уже показано.
        idea["fact_keys"] = match_fact_keys(idea, visible_facts)
        # Полные факты идеи: нужны, чтобы поставить событие в ожидание
        # (иначе оно предлагалось бы снова каждую сессию).
        idea["fact_dicts"] = [f for f in visible_facts if f.get("key") in set(idea["fact_keys"])]
        problems = validate_idea(idea, visible_facts)
        idea["problems"] = problems
        idea["ok"] = not problems
        checked.append(idea)
    payload = {"ideas": checked, "fact_count": len(visible_facts)}
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
