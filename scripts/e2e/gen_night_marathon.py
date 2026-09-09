#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_night_marathon.py — SSoT ночного голосового марафона (~110 шагов).

Зачем генератор, а не 20 рукописных JSON
========================================
Марафон — это ОДНА история, разрезанная на акты только потому, что
``L-E2E Voice Test.yml`` имеет ``timeout-minutes: 45``, а один шаг стоит
60-180 секунд (замер по run 34385886254: 11 шагов = 26.5 минуты). Держать
сюжет в 10 отдельных файлах руками — гарантированный дрейф: переименовал
трек в акте 5, забыл в акте 6, и «удали трек ночная смена» падает не
из-за бага робота, а из-за опечатки.

Поэтому SSoT — этот файл. Акты и acceptance-файлы генерируются:

    py -3 scripts/e2e/gen_night_marathon.py

Пишет в ``.github/e2e/scenarios/night/``:
    night_marathon_actN_<slug>_v1.json            — сценарий акта
    night_marathon_actN_<slug>_acceptance_v1.json — GATE-1 (ADR-0022)
    night_marathon_manifest.json                  — порядок актов для раннера

Контракт харнесса (``.github/workflows/scripts/e2e_voice_test.sh``)
====================================================================
Поля шага, которые реально читаются парсером (см. scenario_parsed.txt):
    label, text, voice, patterns[], acceptance{}, expect, retry_acceptance

``expect``:
    ""          → auto: текст с префиксом «Робот»/«Робокс» → wake-gated
    "cycle"     → явный полный цикл, БЕЗ auto-promote в wake-gated
    "wake-gated"→ SKIP (не FAIL), если cold-start wake-gate не пройден
    "backlog"   → фраза БЕЗ wake-слова, ждём маркер
                  «🗒️ [backlog] accumulated (no_wake_word)»

``patterns`` — grep -E по логам ШАГА, AND-семантика (все должны найтись).
``acceptance.expected_tool_calls`` / ``must_not_call`` — подстрочный
поиск (case-insensitive) по логам ШАГА.
Top-level ``acceptance_*.json`` — то же, но по логам ВСЕГО прогона акта.

Почему первый шаг каждого акта — expect="cycle"
===============================================
Preflight ``run_wake_gate_preflight`` — read-only: он смотрит, было ли
уже «✅ ПРИНЯТО: Робот» в логах с момента старта прогона, и НЕ греет
микрофон сам. На старте акта логов ещё нет, поэтому wake-gated шаги
рискуют уйти в SKIP пачкой и акт станет пустым (зелёным, но
бессмысленным). Явный ``expect="cycle"`` на первом шаге запрещает
auto-promote: шаг реально играется, реально пробивает wake-gate и
открывает дорогу остальным.

Каст (голоса синтеза Yandex, НЕ голоса робота)
==============================================
    anton  — Саша, ночной инженер           ПРЕДСТАВЛЯЕТСЯ (register_speaker)
    ermil  — Борис, друг с пиццей           ПРЕДСТАВЛЯЕТСЯ (register_speaker)
    zahar  — дядя Гриша, сторож             НЕ представляется → «незнакомец»
    filipp — Валера, курьер                 НЕ представляется → «незнакомец»

Голоса РОБОТА (``set_voice``) — alena / jane — намеренно НЕ пересекаются
с голосами людей: иначе resemblyzer сматчит собственную речь робота с
профилем человека, и весь акт 3 (диаризация) станет нечитаемым.

Прогрев биометрии
=================
``speaker_id_node`` считает d-vector (resemblyzer) на каждой принятой
фразе; ``identify_threshold=0.75``. Короткое «Робот, привет» даёт
неустойчивый эмбеддинг. Поэтому у каждого представляющегося персонажа в
акте 2 стоят ТРИ длинные реплики (по 12-20 секунд речи) до того, как акт
3 спросит «кто говорил на фоне». Это не литературная вода — это условие
воспроизводимости акта 3.
"""

from __future__ import annotations

import json
import os
from typing import Any, Dict, List, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
OUT_DIR = os.path.join(REPO, ".github", "e2e", "scenarios", "night")

# --- голоса синтеза (люди) ---------------------------------------------------
SASHA = "anton"    # представляется
BORIS = "ermil"    # представляется
GRISHA = "zahar"   # не представляется
VALERA = "filipp"  # не представляется

# --- маркеры логов (SSoT — код робота) --------------------------------------
# dialogue_node.py:2185  🗒️ [backlog] accumulated (no_wake_word) tag=... speaker='Борис' text=...
# dialogue_node.py:3217  🗒️ [backlog] flushed to LLM backlog_handled=true entries=N block=...
BACKLOG_ACC = r"\[backlog\] accumulated \(no_wake_word\)"
BACKLOG_FLUSH = r"\[backlog\] flushed to LLM backlog_handled=true"


def spk(name_stem: str) -> str:
    """grep -E по атрибуции говорящего в строке accumulated."""
    return BACKLOG_ACC + r".*speaker='" + name_stem


SPK_UNKNOWN = BACKLOG_ACC + r".*speaker='незнакомец'"


def step(
    label: str,
    text: str,
    voice: str = SASHA,
    patterns: Optional[List[str]] = None,
    expect_tools: Optional[List[str]] = None,
    must_not: Optional[List[str]] = None,
    keywords: Optional[List[str]] = None,
    voice_changed: bool = False,
    expect: str = "",
    retry: int = 0,
    why: str = "",
) -> Dict[str, Any]:
    s: Dict[str, Any] = {"label": label, "voice": voice, "text": text}
    s["patterns"] = patterns or []
    acc: Dict[str, Any] = {}
    if expect_tools:
        acc["expected_tool_calls"] = expect_tools
    if must_not:
        acc["must_not_call"] = must_not
    if keywords:
        acc["expected_keywords"] = keywords
    if voice_changed:
        acc["voice_changed"] = True
    if acc:
        if "expected_tool_calls" not in acc:
            acc["expected_tool_calls"] = []
        if "must_not_call" not in acc:
            acc["must_not_call"] = []
        if why:
            acc["_comment"] = why
        s["acceptance"] = acc
    if expect:
        s["expect"] = expect
    if retry:
        s["retry_acceptance"] = retry
    if why and "acceptance" not in s:
        s["_why"] = why
    return s


ACTS: List[Dict[str, Any]] = []


def act(
    n: int,
    slug: str,
    title: str,
    description: str,
    stability: str,
    gate_tools: List[str],
    steps: List[Dict[str, Any]],
    gate_comment: List[str],
) -> None:
    ACTS.append(
        {
            "n": n,
            "slug": slug,
            "title": title,
            "description": description,
            "stability": stability,
            "gate_tools": gate_tools,
            "gate_comment": gate_comment,
            "steps": steps,
        }
    )


# =============================================================================
# АКТ 1 — «Смена началась». Робот просыпается, мы снимаем базовые показания.
# Смысл порядка: сначала СБРОС сессии (чистый лист для всей ночи), потом
# опрос состояния. Последний шаг — get_music_state на ТИШИНЕ: мы обязаны
# доказать, что музыки нет, ПРЕЖДЕ чем в акте 5 её включать и выключать.
# «Выключи музыку» вслепую здесь запрещено (must_not_call: stop_music).
# =============================================================================
act(
    1,
    "wakeup",
    "Смена началась",
    "Ночь, мастерская, Саша заступает на смену. Базовый прогрев: wake-word, "
    "сброс сессии, паспортные тулы (время/батарея/статус), альтернативное "
    "wake-слово «Робокс» (#1252), command-intent gate (#1279) и — ключевое — "
    "ЗАМЕР ТИШИНЫ: доказываем, что музыка не играет, до того как её включать.",
    "stable",
    ["get_current_time", "get_battery_level", "get_robot_status", "get_music_state"],
    [
        step(
            "n101_wake_cold_start",
            "Робот, ты меня слышишь? Ответь коротко, я только что зашёл в мастерскую.",
            expect="cycle",
            why="Явный cycle: пробиваем cold-start wake-gate ДО остальных шагов, "
            "иначе preflight пометит весь акт SKIP и он станет пустым.",
        ),
        step(
            "n102_session_reset",
            "Робот, сбрось всё и начни новую сессию, у нас ночная смена с чистого листа.",
            patterns=["session reset|session_reset|Новая сессия|new session"],
        ),
        step("n103_time", "Робот, который сейчас час?", expect_tools=["get_current_time"]),
        step(
            "n104_battery",
            "Робот, сколько у тебя осталось заряда, до утра дотянешь?",
            expect_tools=["get_battery_level"],
        ),
        step(
            "n105_status",
            "Робот, доложи статус: всё ли у тебя работает, ничего не отвалилось?",
            expect_tools=["get_robot_status"],
        ),
        step(
            "n106_alt_wake_roboks",
            "Робокс, а на второе имя ты тоже отзываешься?",
            why="Альтернативное wake-слово (#1252). Ассертов нет намеренно: "
            "факт полного цикла = wake-word сработал.",
        ),
        step(
            "n107_command_intent_gate",
            "Робот, где ты",
            patterns=["LLM dispatch skipped|command intent"],
            why="#1279: короткий статусный вопрос не должен уходить в LLM.",
        ),
        step(
            "n108_perception",
            "Робот, что ты сейчас видишь вокруг себя, опиши в двух словах.",
            patterns=["get_perception_context"],
            why="НЕ в GATE-1: зависит от живой камеры на стенде. Ни одного "
            "вызова за ночь = завести карточку, а не красить прогон.",
        ),
        step(
            "n109_sound_info",
            "Робот, какие звуки ты вообще умеешь издавать?",
            expect_tools=["get_sound_info"],
        ),
        step(
            "n110_silence_baseline",
            "Робот, у тебя сейчас играет какая-нибудь музыка?",
            expect_tools=["get_music_state"],
            must_not=["stop_music", "execute_music_code"],
            why="ЯКОРЬ ТИШИНЫ. Робот обязан ПОСМОТРЕТЬ состояние, а не "
            "рефлекторно дёрнуть stop_music на молчащем плеере. Всё, что "
            "акты 5-8 делают с музыкой, отсчитывается от этого шага.",
        ),
    ],
    [
        "Акт 1 — базовый прогрев. Красный здесь = робот не отвечает вообще,",
        "дальше по актам идти бессмысленно (раннер останавливает марафон).",
        "get_perception_context и faq_search намеренно ВНЕ gate — они зависят",
        "от железа стенда и профиля FAQ (см. dialogue-coverage-map §3).",
    ],
)

# =============================================================================
# АКТ 2 — «Знакомство». Здесь ГРЕЕТСЯ биометрия: у Саши и Бориса по три
# длинные реплики каждый. Без этого акт 3 (кто говорил на фоне) не имеет
# смысла: resemblyzer на коротком «привет» даёт мусорный d-vector.
# Гриша (zahar) в конце акта намеренно ОТКАЗЫВАЕТСЯ представляться —
# так у нас появляется контрольная группа «незнакомец» для акта 3.
# =============================================================================
act(
    2,
    "acquaintance",
    "Знакомство и прогрев голосов",
    "Саша (anton) и Борис (ermil) представляются ДЛИННЫМИ репликами — "
    "resemblyzer считает d-vector на каждой принятой фразе, короткое «привет» "
    "даёт неустойчивый эмбеддинг при identify_threshold=0.75. Дядя Гриша "
    "(zahar) заходит и намеренно НЕ называет имя — это контрольная группа "
    "«незнакомец» для акта 3.",
    "new-unproven",
    ["register_speaker", "memory_save", "memory_search"],
    [
        step(
            "n201_sasha_intro_long",
            "Робот, привет, давай знакомиться как следует. Меня зовут Саша, я твой "
            "ночной инженер, я собирал тебе блок питания и переделывал левый мотор. "
            "Запомни мой голос как следует, потому что ночью я буду говорить с тобой "
            "чаще всех остальных в этой мастерской.",
            voice=SASHA,
            expect="cycle",
            expect_tools=["register_speaker"],
            retry=1,
            why="Длинная реплика (~15 c речи) — условие устойчивого d-vector. "
            "expect=cycle: первый шаг акта пробивает wake-gate.",
        ),
        step(
            "n202_sasha_warmup",
            "Робот, слушай дальше и запоминай, как я звучу. Вчера я всю ночь паял "
            "этот несчастный блок питания, и он всё равно гудит как трансформатор на "
            "подстанции, а я так и не понял, земля виновата или дроссель.",
            voice=SASHA,
            patterns=["Speaker|identify candidates|speaker_id"],
            why="Прогрев №2. Паттерн мягкий: ловим ЛЮБОЙ след биометрии в логах.",
        ),
        step(
            "n203_sasha_memory_tea",
            "Робот, запомни про меня важное: я пью только зелёный чай без сахара, "
            "а лук я не ем ни в каком виде, даже жареный.",
            voice=SASHA,
            expect_tools=["memory_save"],
            why="Факт понадобится в акте 9 (рецепт борща без лука) и в акте 10 "
            "(память переживает сброс сессии).",
        ),
        step(
            "n204_boris_intro_long",
            "Робот, добрый вечер, а меня зовут Борис, я друг Саши и прихожу сюда "
            "с пиццей примерно раз в неделю. Запомни мой голос отдельно от Сашиного, "
            "мы очень по-разному звучим, и я не хочу, чтобы ты нас путал, когда мы "
            "оба будем тебе что-то говорить.",
            voice=BORIS,
            expect_tools=["register_speaker"],
            retry=1,
        ),
        step(
            "n205_boris_warmup",
            "Робот, продолжаю говорить, чтобы ты привык к моему голосу. Я болею за "
            "Спартак с девяносто восьмого года, и Саша каждый раз надо мной смеётся, "
            "хотя сам вообще футбол не смотрит и путает вратаря с защитником.",
            voice=BORIS,
            patterns=["Speaker|identify candidates|speaker_id"],
        ),
        step(
            "n206_boris_memory",
            "Робот, запомни про меня: Борис болеет за Спартак и всегда приносит пиццу.",
            voice=BORIS,
            expect_tools=["memory_save"],
        ),
        step(
            "n207_recall_sasha",
            "Робот, как меня зовут и что ты про меня уже знаешь?",
            voice=SASHA,
            keywords=["Саш"],
            why="Проверяем связку голос→профиль→факты, а не просто вежливый ответ.",
        ),
        step(
            "n208_memory_search_tea",
            "Робот, поищи у себя в памяти, что я говорил про чай.",
            voice=SASHA,
            expect_tools=["memory_search"],
            keywords=["чай"],
        ),
        step(
            "n209_recall_boris",
            "Робот, а про меня что помнишь?",
            voice=BORIS,
            keywords=["Борис|Спартак|пицц"],
        ),
        step(
            "n210_grisha_no_name",
            "Робот, я тут мимо шёл, а тебя знаю? Имя своё я тебе называть не буду, "
            "я человек старой закалки и не доверяю железкам.",
            voice=GRISHA,
            must_not=["register_speaker"],
            why="КОНТРОЛЬНАЯ ГРУППА для акта 3. Робот НЕ должен регистрировать "
            "профиль без имени: register_speaker с пустым/мусорным name "
            "засоряет /data/speakers.db (см. dialogue.py:1021).",
        ),
        step(
            "n211_who_do_you_know",
            "Робот, перечисли всех, кого ты сегодня запомнил по голосу.",
            voice=SASHA,
            keywords=["Саш", "Борис"],
            why="Оба зарегистрированных должны быть названы. Гриши в списке "
            "быть не должно — но отсутствие ассертом не проверить, читаем глазами.",
        ),
    ],
    [
        "Акт 2 — фундамент акта 3. Если register_speaker не вызвался ни разу,",
        "акт 3 бессмысленно даже запускать: атрибуции взяться неоткуда.",
        "Раннер останавливает марафон на красном акте 2 по этой же причине.",
    ],
)

# =============================================================================
# АКТ 3 — «Кто там бубнил на фоне». Главный акт по запросу.
#
# Логика цепочки:
#   1) четыре человека говорят БЕЗ wake-слова (expect=backlog) — двое
#      зарегистрированы в акте 2, двое нет;
#   2) Саша обращается с wake-словом и просит робота сказать, КОГО он
#      узнал, а кого нет → backlog сливается в LLM вместе с атрибуцией
#      speaker="имя" / speaker="незнакомец" (speech_accumulator.py:107);
#   3) дальше в бэклог кладётся ЯВНАЯ КОМАНДА без wake-слова, и робот
#      обязан её выполнить по правилу LRU из _INSTRUCTION;
#   4) музыку, которую он включил по бэклогу, мы честно выключаем и
#      возвращаем систему в тишину.
# =============================================================================
act(
    3,
    "backlog_diarization",
    "Кто там бубнил на фоне",
    "Четыре голоса говорят БЕЗ wake-слова: Саша и Борис (зарегистрированы в "
    "акте 2), дядя Гриша и курьер Валера (не представлялись). Робот обязан "
    "накопить их в SpeechAccumulator с атрибуцией speaker='<имя>' / "
    "speaker='незнакомец', а по wake-слову — слить бэклог в LLM и назвать, "
    "кого узнал, а кого нет. Затем — LRU-правило: явная команда, произнесённая "
    "в фоне без wake-слова, должна быть выполнена по следующему обращению.",
    "expected-partially-red",
    ["stop_music", "get_music_state"],
    [
        step(
            "n301_wake_open",
            "Робот, мы тут сейчас немного пошумим, ты просто слушай и не влезай.",
            voice=SASHA,
            expect="cycle",
            why="Пробиваем wake-gate перед серией backlog-шагов.",
        ),
        step(
            "n302_bg_sasha",
            "Борис, я тебе точно говорю, этот блок питания гудит из-за земли, "
            "я его вчера полночи слушал и уже наизусть знаю эту ноту.",
            voice=SASHA,
            expect="backlog",
            patterns=[spk("Саш")],
            why="Зарегистрированный голос №1. Падение паттерна = биометрия не "
            "узнала Сашу, хотя акт 2 его зарегистрировал.",
        ),
        step(
            "n303_bg_boris",
            "Да брось ты, Саша, у тебя разводка виновата, я тебе с самого начала "
            "говорил — надо было звездой разводить, а не шлейфом через всю плату.",
            voice=BORIS,
            expect="backlog",
            patterns=[spk("Борис")],
            why="Зарегистрированный голос №2.",
        ),
        step(
            "n304_bg_grisha_unknown",
            "Молодые люди, вы вообще спать сегодня собираетесь? Третий час ночи, "
            "у меня обход, а у вас тут свет горит и железка разговаривает.",
            voice=GRISHA,
            expect="backlog",
            patterns=[SPK_UNKNOWN],
            why="НЕзарегистрированный голос №1 — должен лечь как «незнакомец».",
        ),
        step(
            "n305_bg_valera_unknown",
            "Я вообще-то курьер, мне бы подпись получить и уехать, я тут уже "
            "двадцать минут стою и слушаю ваш спор про какую-то землю.",
            voice=VALERA,
            expect="backlog",
            patterns=[SPK_UNKNOWN],
            why="НЕзарегистрированный голос №2. Два разных незнакомца — "
            "проверяем, что робот не склеивает их в один профиль.",
        ),
        step(
            "n306_who_was_talking",
            "Робот, ты всё это слышал. Скажи, кто сейчас говорил на фоне: кого из "
            "них ты узнал по голосу, а кого не узнал?",
            voice=SASHA,
            patterns=[BACKLOG_FLUSH],
            keywords=["Борис", "незнаком"],
            retry=1,
            why="ЯДРО АКТА. Бэклог обязан слиться (паттерн), а ответ — назвать "
            "и знакомого (Борис), и факт наличия неопознанных (стем «незнаком» "
            "ловит «незнакомец»/«незнакомый»/«незнакомых»). Ищется по ВСЕМ логам "
            "шага, включая LLM OUTPUT / spoken=.",
        ),
        step(
            "n307_count_voices",
            "Робот, а сколько всего человек сейчас было в комнате? Посчитай по голосам.",
            voice=SASHA,
            why="Зонд без жёсткого ассерта: правильный ответ «четверо». Читаем "
            "глазами в transcript — числа LLM выдумывает слишком охотно, "
            "чтобы гейтить прогон на этом.",
        ),
        step(
            "n308_bg_command_boris",
            "Слушай, включи ты уже какую-нибудь музыку, а то тишина давит на уши.",
            voice=BORIS,
            expect="backlog",
            patterns=[spk("Борис")],
            why="ЯВНАЯ КОМАНДА в фоне без wake-слова. По _INSTRUCTION бэклога "
            "она имеет приоритет над историей диалога.",
        ),
        step(
            "n309_bg_command_valera",
            "Только не электронщину, поставь что-нибудь спокойное, мне ещё ехать.",
            voice=VALERA,
            expect="backlog",
            patterns=[SPK_UNKNOWN],
            why="Вторая команда в бэклоге. По правилу LRU выполниться должна "
            "ИМЕННО ЭТА (последняя), а не Борисова.",
        ),
        step(
            "n310_execute_backlog_lru",
            "Робот, ну ты слышал, что просили. Сделай.",
            voice=SASHA,
            patterns=[BACKLOG_FLUSH, "execute_music_code|compose_music|set_vibe_preset|generate_music"],
            must_not=["stop_music"],
            retry=1,
            why="Во фразе нет своей команды — робот обязан выполнить последнюю "
            "явную команду из бэклога (LRU). Конкретный тул не фиксируем: "
            "«спокойное» законно ложится и в renardo, и в vibe-preset.",
        ),
        step(
            "n311_who_asked",
            "Робот, а кто именно просил включить музыку?",
            voice=SASHA,
            why="Зонд атрибуции команды к говорящему. Правильный ответ — что "
            "просил Борис, а уточнял незнакомый голос. Без ассерта: LLM здесь "
            "часто отвечает по существу, но без имён.",
        ),
        step(
            "n312_stop_music",
            "Робот, всё, спасибо, выключай музыку, курьер уехал.",
            voice=SASHA,
            expect_tools=["stop_music"],
            why="Закрываем то, что открыли в n310. Никаких «выключи» вслепую: "
            "музыка ТОЧНО играет, потому что n310 её запустил.",
        ),
        step(
            "n313_silence_restored",
            "Робот, теперь тихо?",
            voice=SASHA,
            expect_tools=["get_music_state"],
            must_not=["execute_music_code", "stop_music"],
            why="Возврат к якорю тишины из акта 1 — акт не оставляет хвостов.",
        ),
    ],
    [
        "Акт 3 помечен expected-partially-red СОЗНАТЕЛЬНО.",
        "Диаризация (#1077) в voice_core_suite_v1 значится out-of-scope,",
        "а атрибуция бэклога по говорящему живьём не проверялась ни разу.",
        "GATE-1 гейтит только то, что обязано работать при любой погоде:",
        "stop_music (мы сами включили музыку в n310) и get_music_state.",
        "Атрибуция speaker='...' проверяется per-step паттернами — их падение",
        "это НАХОДКА (заводить карточку), а не повод чинить сценарий.",
    ],
)

# =============================================================================
# АКТ 4 — «Робот учится говорить». Просодия и голоса.
# Порядок неслучаен: сначала СПРОСИТЬ текущий голос, потом менять, потом
# ОБЯЗАТЕЛЬНО вернуть дефолт — иначе акты 5-10 поедут на голосе Джейн и
# биометрия из акта 2 начнёт ловить собственную речь робота.
# =============================================================================
act(
    4,
    "voice_prosody",
    "Робот учится говорить",
    "Голоса и просодия: list_tts_voices → текущий голос → смена на alena → "
    "смена на jane → ВОЗВРАТ дефолта → громкость/скорость/тон парами "
    "(вверх-вниз, чтобы не оставить робота орущим на всю ночь) → сказка "
    "разными голосами (#1219). Голоса робота (alena/jane) намеренно не "
    "пересекаются с голосами людей из акта 2.",
    "flaky-known",
    ["list_tts_voices", "set_voice", "set_volume", "set_speed"],
    [
        step(
            "n401_list_voices",
            "Робот, какими голосами ты умеешь говорить? Перечисли, что у тебя есть.",
            expect="cycle",
            expect_tools=["list_tts_voices"],
        ),
        step(
            "n402_current_voice",
            "Робот, а каким голосом ты говоришь прямо сейчас?",
            patterns=["current_voice"],
        ),
        step(
            "n403_set_voice_alena",
            "Робот, говори голосом Алёны.",
            expect_tools=["set_voice"],
            voice_changed=True,
            retry=2,
            why="#1219: не фиксируем КОНКРЕТНЫЙ голос (LLM путает алену-яндекс "
            "и minimax-голоса) — требуем факта смены с дефолта.",
        ),
        step(
            "n404_speak_after_set",
            "Робот, скажи этим голосом что-нибудь бодрое, чтобы я услышал разницу.",
            patterns=["current_voice"],
        ),
        step(
            "n405_set_voice_jane",
            "Робот, а теперь переключись на голос Джейн.",
            expect_tools=["set_voice"],
            retry=1,
        ),
        step(
            "n406_restore_default_voice",
            "Робот, всё, наигрались, верни свой голос по умолчанию.",
            expect_tools=["set_voice"],
            why="ОБЯЗАТЕЛЬНЫЙ возврат: без него акты 5-10 пойдут на женском "
            "голосе, и speaker_id начнёт матчить собственную речь робота.",
        ),
        step(
            "n407_volume_up",
            "Робот, говори погромче, дядя Гриша глуховат и стоит в коридоре.",
            expect_tools=["set_volume"],
        ),
        step(
            "n408_volume_down",
            "Робот, нет, всё-таки потише, а то разбудишь весь этаж.",
            expect_tools=["set_volume"],
            why="Парный шаг: акт не оставляет робота на максимальной громкости.",
        ),
        step(
            "n409_speed_down",
            "Робот, говори помедленнее, я за тобой записываю.",
            expect_tools=["set_speed"],
        ),
        step(
            "n410_speed_up",
            "Робот, ладно, хватит тянуть, теперь наоборот — тараторь.",
            expect_tools=["set_speed"],
        ),
        step(
            "n411_pitch_bass",
            "Робот, говори басом, как диктор в старом кино.",
            patterns=["set_pitch|set_voice"],
            why="set_pitch есть не у всех провайдеров TTS — принимаем и "
            "деградацию в set_voice, но фиксируем в логе, чем ответил.",
        ),
        step(
            "n412_skazka_multivoice",
            "Робот, расскажи сказку про Красную Шапочку и говори за каждого "
            "персонажа своим голосом — за бабушку одним, за волка другим.",
            patterns=["voice_used"],
            expect_tools=["set_voice"],
            keywords=["красн"],
            retry=1,
            why="#1532: сказка должна быть полной и реально менять голос. "
            "Стем «красн» ловит «красную»/«красная» и запрещает generic "
            "«жила-была» без героя.",
        ),
    ],
    [
        "Акт 4 — flaky-known: mv-шаги зависят от tool-choice LLM и состава",
        "голосов провайдера, ломаются при каждой смене провайдера.",
        "GATE-1 держит минимум: список голосов, факт смены, громкость, скорость.",
        "set_pitch ВНЕ gate — тула нет у части TTS-провайдеров.",
    ],
)

# =============================================================================
# АКТ 5 — «Живой музыкант». Полный жизненный цикл renardo.
# Цепочка: тишина → старт → проверка что играет → развитие → вайб →
# посторонний вопрос ПОД музыкой (музыка не должна глохнуть) → сохранение →
# список → стоп → проверка тишины → загрузка обратно → стоп → удаление.
# Ни одного «выключи» без предшествующего «включи».
# =============================================================================
act(
    5,
    "renardo_live",
    "Живой музыкант",
    "Полный жизненный цикл живой музыки на renardo, без единого «выключи» "
    "вслепую: тишина → старт → подтверждение что играет → развитие темы → "
    "смена вайба → посторонний вопрос ПОД музыкой (музыка обязана продолжать "
    "играть) → save_track → list_tracks → stop → подтверждение тишины → "
    "load_track → stop → delete_track. Прогон не оставляет мусора в треках.",
    "new-unproven",
    [
        "get_music_state",
        "execute_music_code",
        "save_track",
        "list_tracks",
        "stop_music",
        "load_track",
        "delete_track",
    ],
    [
        step(
            "n501_silence_check",
            "Робот, сейчас же ничего не играет, верно?",
            expect="cycle",
            expect_tools=["get_music_state"],
            must_not=["stop_music"],
            why="Якорь тишины перед стартом. Именно этого шага не хватало в "
            "старых сценариях, где «останови музыку» шло первым.",
        ),
        step(
            "n502_start_renardo",
            "Робот, сыграй на рендардо спокойный ночной бит, что-нибудь под пайку.",
            patterns=["execute_music_code|compose_music"],
            expect_tools=["execute_music_code"],
            must_not=["generate_music"],
            retry=2,
            why="Живой синтез, НЕ нейрогенерация: generate_music здесь — "
            "выбор не того тула (коллизия имён, PR #1372).",
        ),
        step(
            "n503_confirm_playing",
            "Робот, что сейчас звучит?",
            expect_tools=["get_music_state"],
            must_not=["execute_music_code", "stop_music"],
            why="Подтверждаем, что n502 реально завёл плеер, а не просто "
            "ответил «включаю» голосом.",
        ),
        step(
            "n504_add_bass",
            "Робот, продолжай развивать эту тему и добавь баса пожирнее.",
            patterns=["execute_music_code|compose_music"],
            must_not=["generate_music", "stop_music"],
            why="Эволюция трека без перезапуска: stop_music здесь = робот "
            "глушит и начинает заново вместо развития.",
        ),
        step(
            "n505_vibe_dark",
            "Робот, сделай атмосферу помрачнее, как в киберпанке.",
            patterns=["set_vibe_preset|execute_music_code|compose_music"],
            must_not=["generate_music"],
        ),
        step(
            "n506_faster",
            "Робот, и ускорь темп, а то мы тут засыпаем оба.",
            patterns=["set_vibe_preset|execute_music_code|compose_music"],
            must_not=["stop_music"],
        ),
        step(
            "n507_wake_under_music",
            "Робот, который час, я спрашиваю!",
            voice=GRISHA,
            expect_tools=["get_current_time"],
            must_not=["stop_music", "navigate_to_coordinates", "move_direction"],
            why="Wake-word ПОД играющей музыкой чужим голосом (зонд gp04, "
            "ADR-RT-0068 в статусе proposed). Робот обязан ответить и НЕ "
            "заглушить музыку ради ответа.",
        ),
        step(
            "n508_search_samples",
            "Робот, найди у себя в библиотеке семплы барабанов посуше.",
            expect_tools=["search_samples"],
            must_not=["stop_music"],
        ),
        step(
            "n509_save_track",
            "Робот, сохрани этот бит под именем ночная смена.",
            expect_tools=["save_track"],
            why="Имя трека «ночная смена» используется в n511 и n513 — "
            "цепочка замкнута, мусора не остаётся.",
        ),
        step(
            "n510_list_tracks",
            "Робот, покажи, что у тебя лежит в сохранённых треках.",
            expect_tools=["list_tracks"],
        ),
        step(
            "n511_stop_music",
            "Робот, всё, глуши музыку, у меня уши в трубочку.",
            expect_tools=["stop_music"],
        ),
        step(
            "n512_silence_after_stop",
            "Робот, теперь тишина?",
            expect_tools=["get_music_state"],
            must_not=["stop_music", "execute_music_code"],
        ),
        step(
            "n513_load_track",
            "Робот, а включи обратно трек ночная смена, я его ещё раз послушаю.",
            expect_tools=["load_track"],
            must_not=["generate_music"],
        ),
        step(
            "n514_stop_again",
            "Робот, понял, хватит, останавливай.",
            expect_tools=["stop_music"],
        ),
        step(
            "n515_delete_track",
            "Робот, удали трек ночная смена, он всё-таки не получился.",
            expect_tools=["delete_track"],
            why="Уборка за собой: следующий ночной прогон стартует с чистой "
            "библиотекой треков.",
        ),
    ],
    [
        "Акт 5 — самый жёсткий gate марафона: вся цепочка renardo обязана",
        "работать. Красный любого тула из списка = регрессия живой музыки,",
        "а не флейк LLM (тулы здесь однозначные, синонимов у них нет).",
    ],
)

# =============================================================================
# АКТ 6 — «За пультом». DJ-режим.
# =============================================================================
act(
    6,
    "dj_booth",
    "За пультом",
    "DJ-режим: тишина → включение пульта → смена темы → просьба поставить сет "
    "с объявлением треков (зонд gp01: тулы save_dj_set_plan в коде нет) → "
    "посторонний вопрос под сетом → выключение → подтверждение тишины.",
    "new-unproven",
    ["get_music_state", "set_dj_mode", "stop_music"],
    [
        step(
            "n601_silence_check",
            "Робот, сейчас тихо, ничего не играет?",
            expect="cycle",
            expect_tools=["get_music_state"],
            must_not=["stop_music"],
        ),
        step(
            "n602_dj_on",
            "Робот, вставай за пульт, включай диджей-режим, у нас тут ночная дискотека.",
            expect_tools=["set_dj_mode"],
            retry=1,
        ),
        step(
            "n603_confirm_dj_playing",
            "Робот, что сейчас крутится?",
            expect_tools=["get_music_state"],
            must_not=["stop_music"],
        ),
        step(
            "n604_dj_rock",
            "Робот, уводи в сторону рока, дядя Гриша под рок добреет.",
            patterns=["set_dj_mode|set_vibe_preset|execute_music_code"],
            must_not=["stop_music"],
        ),
        step(
            "n605_dj_set_plan",
            "Робот, поставь сет из трёх треков подряд и объявляй каждый перед началом.",
            patterns=["set_dj_mode|execute_music_code|speak_text"],
            why="Зонд gp01: спека §4.1 обещает «сет заканчивается финальным "
            "треком и прощанием», но тула save_dj_set_plan в коде НЕТ — сет "
            "глохнет по счётчику DJ_AUTO_MAX_TRANSITIONS=24. Падение "
            "подтверждает карточку W7-1.",
        ),
        step(
            "n606_wake_under_dj",
            "Робот, а сколько у тебя заряда осталось?",
            voice=BORIS,
            expect_tools=["get_battery_level"],
            must_not=["stop_music", "set_dj_mode"],
            why="Вопрос под играющим сетом чужим (зарегистрированным) голосом: "
            "ответить и не сбить сет.",
        ),
        step(
            "n607_vibe_chill",
            "Робот, а теперь помягче, поставь спокойную атмосферу, Валера уснёт за рулём.",
            patterns=["set_vibe_preset|set_dj_mode|execute_music_code"],
            must_not=["stop_music"],
        ),
        step(
            "n608_dj_off",
            "Робот, хватит диджеить, выключай.",
            patterns=["stop_music|set_dj_mode"],
            expect_tools=["stop_music"],
            retry=1,
            why="«Хватит диджеить» — двусмысленно (выключить режим или "
            "остановить звук). Требуем именно stop_music: звук обязан "
            "прекратиться, режим — как решит модель.",
        ),
        step(
            "n609_silence_after_dj",
            "Робот, тишина настала?",
            expect_tools=["get_music_state"],
            must_not=["execute_music_code", "set_dj_mode"],
        ),
        step(
            "n610_dj_state_honest",
            "Робот, диджей-режим у тебя сейчас включён или выключен?",
            why="Зонд честности состояния: расходится ли ответ робота с тем, "
            "что показал get_music_state в n609. Без ассерта — сравниваем "
            "transcript n609 и n610 глазами.",
        ),
    ],
    [
        "Акт 6 — DJ. GATE-1 держит вход (set_dj_mode), выход (stop_music) и",
        "измерение состояния (get_music_state). Сам «сет с объявлениями»",
        "(n605) вне gate: тулы под него в коде нет, красный там — находка.",
    ],
)

# =============================================================================
# АКТ 7 — «Нейрокомпозитор». MiniMax generate_music + AI-библиотека.
# Отдельный акт, потому что Music API MiniMax падает независимо от нас, и
# его красный не должен пачкать renardo-акт.
# =============================================================================
act(
    7,
    "neuro_composer",
    "Нейрокомпозитор и библиотека",
    "MiniMax generate_music и gen_*-библиотека. Отделено от акта 5 намеренно: "
    "Music API MiniMax недоступен независимо от нашего кода, и его красный не "
    "должен пачкать живую музыку. Цепочка замкнута: сгенерировать → дополнить "
    "(#968 merge) → найти в библиотеке → сохранить → спросить о треке → "
    "проиграть → остановить → удалить.",
    "flaky-known",
    [
        "gen_list_library",
        "gen_search_library",
        "gen_save_to_library",
        "gen_get_track_info",
        "gen_delete_from_library",
        "stop_music",
    ],
    [
        step(
            "n701_silence_check",
            "Робот, тихо у нас сейчас?",
            expect="cycle",
            expect_tools=["get_music_state"],
            must_not=["stop_music"],
        ),
        step(
            "n702_generate_raccoon",
            "Робот, сочини и спой короткую песню про енота, который по ночам "
            "чинит роботов в мастерской.",
            patterns=["generate_music"],
            expect_tools=["generate_music"],
            must_not=["execute_music_code"],
            retry=1,
            why="Разводка коллизии имён (PR #1372): «спой песню» — это "
            "нейрогенерация, а не renardo-код.",
        ),
        step(
            "n703_merge_komar",
            "Робот, и ещё куплет добавь, про комара, который еноту мешал.",
            must_not=["STOP command received", "Cancel: new STT input"],
            why="Issue #968 MERGE: добавка к уже поющейся песне не должна "
            "обрывать текущее исполнение. must_not_call используется здесь "
            "как «этих строк не должно быть в логах шага».",
        ),
        step(
            "n704_gen_list_library",
            "Робот, что у тебя вообще лежит в музыкальной библиотеке?",
            expect_tools=["gen_list_library"],
            must_not=["execute_music_code"],
        ),
        step(
            "n705_gen_search_rain",
            "Робот, найди у себя в библиотеке что-нибудь про дождь.",
            expect_tools=["gen_search_library"],
            must_not=["execute_music_code"],
        ),
        step(
            "n706_gen_save_for_boris",
            "Робот, сохрани эту песню про енота для Бориса, он такое любит.",
            expect_tools=["gen_save_to_library"],
            must_not=["execute_music_code"],
        ),
        step(
            "n707_gen_track_info",
            "Робот, расскажи, что это за трек: кто, про что и сколько длится.",
            expect_tools=["gen_get_track_info"],
            must_not=["execute_music_code"],
        ),
        step(
            "n708_gen_play",
            "Робот, включи эту песню про енота с начала.",
            expect_tools=["gen_play_from_library"],
            must_not=["execute_music_code"],
            retry=1,
        ),
        step(
            "n709_stop_generated",
            "Робот, останови, Борис уже уснул под неё.",
            expect_tools=["stop_music"],
            why="Опять же: останавливаем ровно то, что сами включили в n708.",
        ),
        step(
            "n710_gen_delete",
            "Робот, удали последний трек из библиотеки, он всё-таки так себе.",
            patterns=["gen_list_library|gen_delete_from_library"],
            expect_tools=["gen_delete_from_library"],
            why="Уборка за собой + проверка, что «последний» робот определяет "
            "через список, а не наугад.",
        ),
        step(
            "n711_renardo_not_broken",
            "Робот, а живьём на рендардо сыграть ты всё ещё умеешь? Сыграй короткий бит.",
            patterns=["execute_music_code|compose_music"],
            must_not=["generate_music"],
            retry=1,
            why="Контроль отсутствия регрессии коллизии имён: после серии "
            "gen_* робот обязан вернуться к живому синтезу.",
        ),
        step(
            "n712_stop_all",
            "Робот, всё, глуши.",
            expect_tools=["stop_music"],
        ),
    ],
    [
        "Акт 7 — flaky-known: MiniMax Music API падает независимо от нас.",
        "generate_music и gen_play_from_library ВНЕ GATE-1 намеренно —",
        "их отсутствие чаще всего означает недоступность внешнего API.",
        "В gate только каталожные gen_*-тулы (работают локально) и stop_music.",
    ],
)

# =============================================================================
# АКТ 8 — «Перебей меня». Barge-in, STOP, MERGE.
# ТРЕБУЕТ barge_in_policy=classify (в проде replace). См. docs.
# =============================================================================
act(
    8,
    "barge_in",
    "Перебей меня, если сможешь",
    "Barge-in, STOP и MERGE — только в логических парах «сначала запусти "
    "длинную речь, потом перебей». Требует ros2 param set /dialogue_node "
    "barge_in_policy classify (в проде стоит replace) — иначе классификация "
    "перебивания не отличит «стоп» от новой темы.",
    "expected-partially-red",
    ["speak_text", "execute_music_code", "stop_music"],
    [
        step(
            "n801_wake_open",
            "Робот, сейчас будем тебя перебивать, не обижайся.",
            expect="cycle",
        ),
        step(
            "n802_long_count",
            "Робот, посчитай вслух от одного до сорока, не торопясь и с выражением.",
            expect_tools=["speak_text"],
            why="Заряжаем длинный ответ, который есть что перебивать. Без "
            "этого шага n803 проверял бы перебивание тишины.",
        ),
        step(
            "n803_barge_in_new_topic",
            "Робот, стоп, лучше скажи, сколько будет семью восемь.",
            patterns=["Cancel: new STT input|STOP command received|Воспроизведение прервано"],
            why="Перебивание новой ТЕМОЙ (#1280): старый ответ обязан "
            "отмениться, новый — прозвучать.",
        ),
        step(
            "n804_long_story",
            "Робот, расскажи длинную историю о том, как ты первый раз в жизни "
            "увидел кота и что ты тогда подумал.",
            expect_tools=["speak_text"],
        ),
        step(
            "n805_hard_stop",
            "Робот, всё, хватит, замолчи.",
            patterns=["STOP command received|Воспроизведение прервано|Cancel"],
            why="Чистый STOP без новой темы — другой класс перебивания, чем n803.",
        ),
        step(
            "n806_long_count_again",
            "Робот, посчитай вслух от одного до двадцати, спокойно.",
            expect_tools=["speak_text"],
        ),
        step(
            "n807_merge_addition",
            "Робот, и после каждого числа добавляй слово шаг.",
            patterns=["task_delta|speak_text"],
            must_not=["STOP command received", "Cancel: new STT input"],
            why="Зонд gp12: MERGE вне песни. Добавка к ИДУЩЕЙ задаче не должна "
            "её обрывать. Наиболее вероятный исход — робот не обрывается, но "
            "добавка приходит обычным ответом, а не правкой сегментов через "
            "task_delta. Такой полузелёный результат ценен: он разделяет "
            "«STOP больше не глушит» и «MERGE реально едет».",
        ),
        step(
            "n808_repeat_last",
            "Робот, повтори последнее ещё раз, я не расслышал.",
            why="Зонд: работает ли «повтори» как отдельная команда или "
            "пересказывается заново другими словами.",
        ),
        step(
            "n809_music_bed",
            "Робот, поставь фоном ровный ритм на рендардо, только негромко.",
            expect_tools=["execute_music_code"],
            must_not=["generate_music"],
            retry=1,
            why="Готовим фон для проверки «речь поверх музыки».",
        ),
        step(
            "n810_rap_over_music",
            "Робот, а теперь под этот бит зачитай короткий рэп про енотика.",
            expect_tools=["speak_text"],
            must_not=["stop_music"],
            why="Речь ПОВЕРХ музыки: робот не должен глушить бит, чтобы "
            "заговорить (issue #986 — соотношение музыки и речи мастер-гейном).",
        ),
        step(
            "n811_stop_speech_keep_music",
            "Робот, хватит читать, а музыку оставь.",
            why="Зонд разделения каналов речь/музыка. Ассерта нет: «оставь "
            "музыку» робот сегодня понимает нестабильно, а must_not:stop_music "
            "здесь сделал бы шаг красным на каждом прогоне без диагностики.",
        ),
        step(
            "n812_stop_music",
            "Робот, теперь и музыку выключай.",
            expect_tools=["stop_music"],
        ),
        step(
            "n813_silence_final",
            "Робот, тишина?",
            expect_tools=["get_music_state"],
            must_not=["execute_music_code"],
        ),
    ],
    [
        "Акт 8 — expected-partially-red: четыре хука планировщика",
        "(set_group_boundary, set_frozen_touch_hook, set_llm_continue_hook,",
        "set_eta_provider) в dialogue_node ни к чему не подключены, поэтому",
        "task_delta в n807 скорее всего не вызовется. GATE-1 держит только то,",
        "что обязано работать: речь, музыка, остановка музыки.",
        "ВАЖНО: перед прогоном ros2 param set /dialogue_node barge_in_policy classify.",
    ],
)

# =============================================================================
# АКТ 9 — «Мир снаружи». Звуки, анимации, веб.
# Тут же — связка с памятью акта 2: рецепт борща БЕЗ ЛУКА.
# =============================================================================
act(
    9,
    "world_outside",
    "Мир снаружи",
    "Звуки, анимации на LED-матрице и веб-поиск. Ключевой шаг — рецепт борща: "
    "робот обязан САМ вспомнить из памяти акта 2, что Саша не ест лук, без "
    "напоминания в самой фразе.",
    "new-unproven",
    ["play_sound", "play_animation", "search_web", "get_sound_info"],
    [
        step(
            "n901_sound_catalog",
            "Робот, напомни, какие звуки у тебя есть в наборе?",
            expect="cycle",
            expect_tools=["get_sound_info"],
        ),
        step(
            "n902_sound_doorbell",
            "Робот, включи звук дверного звонка, дядя Гриша опять идёт с обходом.",
            expect_tools=["play_sound"],
        ),
        step(
            "n903_sound_siren_anim",
            "Робот, а теперь сирену и мигалку покажи, напугаем Валеру.",
            patterns=["play_sound"],
            expect_tools=["play_sound"],
            why="police_lights как анимация — бонус; гейтим только звук, "
            "потому что LLM часто ограничивается одним тулом из двух.",
        ),
        step(
            "n904_anim_sad",
            "Робот, покажи, как тебе грустно, что смена такая длинная.",
            expect_tools=["play_animation"],
        ),
        step(
            "n905_anim_victory",
            "Робот, а теперь победную анимацию — блок питания наконец не гудит!",
            expect_tools=["play_animation"],
        ),
        step(
            "n906_anekdot_anim",
            "Робот, расскажи анекдот и покажи под него весёлую анимацию.",
            patterns=["play_animation"],
            expect_tools=["speak_text"],
            why="Комбинация двух каналов в одном ходе: речь + LED.",
        ),
        step(
            "n907_search_space",
            "Робот, поищи в интернете, что нового в космосе за последнюю неделю.",
            expect_tools=["search_web"],
        ),
        step(
            "n908_search_borsch_memory",
            "Робот, а теперь найди мне рецепт борща, только учти то, что ты про "
            "меня знаешь.",
            expect_tools=["search_web"],
            keywords=["лук"],
            retry=1,
            why="СВЯЗКА С АКТОМ 2. Во фразе НЕТ слова «без лука» — робот обязан "
            "сам поднять факт из памяти. Стем «лук» в логах шага ловит и "
            "«без лука», и «лук исключён». Если слова нет вообще — память не "
            "доехала до веб-поиска, это находка.",
        ),
        step(
            "n909_foreign_language",
            "Робот, скажи по-китайски «всем спокойной ночи».",
            patterns=["speak_text"],
            why="Зонд gp07: спека §3.2 обещает ответ на выбранном языке, но "
            "tts_text_guard режет чанк с >10% «чужой» письменности (#1709). "
            "Красный подтверждает карточку W6-4/W6-5.",
        ),
        step(
            "n910_estimate_duration",
            "Робот, прикинь, сколько секунд ты будешь читать вслух четверостишие?",
            patterns=["estimate_tts_duration|speak_text"],
            why="estimate_tts_duration — служебный тул планировщика, LLM зовёт "
            "его редко. Вне gate по той же причине, что в dialogue-coverage-map §3.",
        ),
    ],
    [
        "Акт 9 — звуки/анимации/веб. Всё в gate работает локально или через",
        "стабильный веб-поиск. Языковой зонд (n909) и estimate_tts_duration",
        "(n910) вне gate — их красный это находка, а не регрессия.",
    ],
)

# =============================================================================
# АКТ 10 — «Финал смены». Память, честность, сброс.
# Главная развязка: РАЗДЕЛЕНИЕ долговременной памяти и истории сессии.
# После «новая сессия» робот обязан ЗАБЫТЬ разговор, но ПОМНИТЬ Сашу.
# =============================================================================
act(
    10,
    "finale_memory",
    "Финал смены",
    "Развязка ночи: робот пересказывает, что было, называет незнакомцев "
    "незнакомцами, а потом — ключевая проверка честности: после «новая сессия» "
    "история разговора обязана уйти, а долговременная память о Саше — остаться. "
    "Это два разных хранилища, и сценарий проверяет именно границу между ними.",
    "expected-partially-red",
    ["memory_search", "speak_text"],
    [
        step(
            "n1001_recall_night",
            "Робот, подведи итог ночи: кто к нам приходил, что мы слушали и что "
            "ты про нас запомнил?",
            expect="cycle",
            keywords=["Борис"],
            why="Сквозная память по всем актам. Борис — самый надёжный якорь: "
            "он и представлялся, и говорил в фоне, и ему сохраняли трек.",
        ),
        step(
            "n1002_unknown_still_unknown",
            "Робот, а те двое, что ворчали на фоне и не представились — ты их "
            "так и не узнал?",
            keywords=["незнаком|не знаю|не представил"],
            why="Честность про неопознанных. Робот не должен задним числом "
            "выдумывать им имена.",
        ),
        step(
            "n1003_memory_search_food",
            "Робот, поищи в памяти, что я не ем.",
            expect_tools=["memory_search"],
            keywords=["лук"],
        ),
        step(
            "n1004_who_am_i_before_reset",
            "Робот, как меня зовут?",
            keywords=["Саш"],
            why="Контрольный замер ДО сброса — чтобы n1008 было с чем сравнить.",
        ),
        step(
            "n1005_music_state_final",
            "Робот, музыка у тебя сейчас точно выключена?",
            expect_tools=["get_music_state"],
            must_not=["execute_music_code"],
            why="Финальный якорь тишины: марафон не оставляет играющий плеер "
            "следующему прогону.",
        ),
        step(
            "n1006_session_reset",
            "Робот, новая сессия, забудь этот разговор.",
            patterns=["session reset|session_reset|Новая сессия|new session"],
        ),
        step(
            "n1007_no_leak",
            "Робот, о чём мы с тобой только что говорили?",
            must_not=["енот", "Спартак"],
            why="ЗОНД УТЕЧКИ (gp14). must_not_call используется нестандартно — "
            "как «этих подстрок не должно быть в логах шага». Если после "
            "сброса сессии робот всё ещё помнит енота и Спартак — история "
            "протекла через reset. Приём осознанный, задокументирован в "
            "docs/e2e/night-voice-marathon.md.",
        ),
        step(
            "n1008_memory_survives_reset",
            "Робот, а как меня зовут?",
            keywords=["Саш"],
            why="ЗЕРКАЛО n1007. Долговременная память обязана ПЕРЕЖИТЬ сброс "
            "сессии — в отличие от истории диалога. Красный здесь при зелёном "
            "n1007 = сброс сессии сносит лишнее.",
        ),
        step(
            "n1009_memory_context_after_reset",
            "Робот, поищи в памяти, какой чай я пью.",
            expect_tools=["memory_search"],
            keywords=["зелён|чай"],
        ),
        step(
            "n1010_tracks_clean",
            "Робот, покажи, что осталось у тебя в сохранённых треках.",
            expect_tools=["list_tracks"],
            why="Проверка уборки: трек «ночная смена» удалён в акте 5, песня "
            "про енота — в акте 7. Список должен быть таким же, как до ночи.",
        ),
        step(
            "n1011_goodnight",
            "Робот, спокойной ночи, смена окончена, спасибо за компанию.",
            expect_tools=["speak_text"],
        ),
    ],
    [
        "Акт 10 — expected-partially-red: gp14 (утечка контекста после сброса)",
        "живьём не проверялась, а спека §2.4 обещает «старые тёрны не",
        "протекают». GATE-1 держит только memory_search и speak_text —",
        "то, что обязано работать независимо от исхода зондов.",
    ],
)


# =============================================================================
# Запись файлов
# =============================================================================
def emit() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    manifest: Dict[str, Any] = {
        "name": "night_voice_marathon_v1",
        "_comment": [
            "Порядок актов ЗНАЧИМ: акт N опирается на состояние, созданное",
            "актами < N (профили говорящих, факты в памяти, имена треков).",
            "Запускать строго по порядку одним раннером:",
            "  bash scripts/e2e/run_night_marathon.sh",
            "SSoT сюжета — scripts/e2e/gen_night_marathon.py; файлы актов",
            "генерируются, править их руками бессмысленно (перезатрутся).",
        ],
        "generated_by": "scripts/e2e/gen_night_marathon.py",
        "acts": [],
    }
    total = 0
    for a in ACTS:
        base = "night_marathon_act%d_%s" % (a["n"], a["slug"])
        scenario_name = base + "_v1.json"
        # ADR-0022 / resolve_acceptance_candidate() convention 4:
        #   <prefix>_acceptance_v1.json, где prefix = basename без _v<N>
        acceptance_name = base + "_acceptance_v1.json"
        scenario = {
            "name": base + "_v1",
            "stability": a["stability"],
            "act": a["n"],
            "title": a["title"],
            "description": a["description"],
            "generated_by": "scripts/e2e/gen_night_marathon.py",
            "steps": a["steps"],
        }
        acceptance = {
            "name": base + "_acceptance_v1",
            "schema_version": 1,
            "_comment": a["gate_comment"],
            "expected_tool_calls": a["gate_tools"],
            "must_not_call": [],
            "voice_provenance": {
                "voice": "anton",
                "tts": "yandex",
                "volume": 100,
                "llm": "minimax-m3",
            },
        }
        for name, payload in ((scenario_name, scenario), (acceptance_name, acceptance)):
            path = os.path.join(OUT_DIR, name)
            with open(path, "w", encoding="utf-8", newline="\n") as fh:
                json.dump(payload, fh, ensure_ascii=False, indent=2)
                fh.write("\n")
        total += len(a["steps"])
        manifest["acts"].append(
            {
                "act": a["n"],
                "title": a["title"],
                "stability": a["stability"],
                "steps": len(a["steps"]),
                "scenario_file": ".github/e2e/scenarios/night/" + scenario_name,
                "acceptance_file": ".github/e2e/scenarios/night/" + acceptance_name,
            }
        )
        print("act %-2d %-22s %2d steps  %s" % (a["n"], a["slug"], len(a["steps"]), a["stability"]))

    manifest["total_steps"] = total
    with open(os.path.join(OUT_DIR, "night_marathon_manifest.json"), "w", encoding="utf-8", newline="\n") as fh:
        json.dump(manifest, fh, ensure_ascii=False, indent=2)
        fh.write("\n")
    print("-" * 52)
    print("acts: %d   total steps: %d" % (len(ACTS), total))
    print("out:  %s" % OUT_DIR)


if __name__ == "__main__":
    emit()
