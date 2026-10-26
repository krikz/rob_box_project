#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_night_marathon.py — SSoT ночного голосового марафона (число шагов -- ``total_steps`` манифеста).

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
    "backlog"   → фраза БЕЗ wake-слова, ждём маркер dialog v2
                  «[backlog] accumulated reason=no_wake entries=N»
                  (v1-форма «accumulated (no_wake_word)» харнесс ещё понимает, сценарии её не ждут)

``silent_ok`` (bool): ход без речи допустим -- Tier-1 «стоп/хватит» гасит речь и молчит
(``llm_calls=0``), звук/анимация по прямой просьбе исполняются тихо (решение Шифу 26.10). Сам флаг ничего
не доказывает: смысл шагу даёт паттерн результата (отмена речи, ``✅ Инструмент … выполнен успешно``).

``recover`` (акт или шаг, #3989/#4034): какие восстановления харнесса разрешены -- ``true`` (все), ``false``/``[]``
(никаких), список из ``no_wake``/``silenced``. Читает ``e2e_events.recover_kinds``; восстановленный шаг -- не чистый
OK, а ``OK recovered_<вид>`` (маркер ``E2E_RECOVER`` в логе акта).

``_skip_when`` (шаг, с подчёркиванием: харнесс его НЕ читает): предусловие внешнего мира, без которого шаг теряет
смысл, -- ``{"reason": "KIE_QUOTA", "log": <regex>}``. Пока харнесс не умеет SKIP по строке лога, правило -- данные
для человека и для будущей проверки харнесса (R21 разбора 26.10).

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

Каст (голоса синтеза людей, НЕ голоса робота; #4038, решение Шифу 26.10)
========================================================================
Имя голоса в шаге -- метка роли; голос MiniMax ей подбирает ``map_tts_voice`` харнесса
(``e2e_voice_lib.sh``, замер ``docs/dialog/role_voices_biometrics_2026-10-26.md``). Марафон идёт с
``tts_provider=minimax``: голоса Yandex одного регистра склеиваются (0.84–0.92).
    anton  — Саша, ночной инженер     (English_Jovialman, муж.)       ПРЕДСТАВЛЯЕТСЯ (register_speaker)
    ermil  — Борис, друг с пиццей     (English_WiseScholar, муж.)     ПРЕДСТАВЛЯЕТСЯ (register_speaker)
    zahar  — тётя Гриша, сторож       (Gentle_Senior, ЖЕНСКИЙ)       НЕ представляется → «незнакомец»
    filipp — Валера, курьерша         (Whispering_girl, ЖЕНСКИЙ)     НЕ представляется → «незнакомец»
Реплики Гриши и Валеры от первого лица -- в женском роде («я мимо шла»). Голоса разведены (робот на 28 фразах
завёл ровно 4 профиля), поэтому шаги атрибуции (n210, n304–n311) НЕ помечаются «предел голосов».

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
import re
from typing import Any, Callable, Dict, List, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
OUT_DIR = os.path.join(REPO, ".github", "e2e", "scenarios", "night")
# Корень репозитория — нужно для поиска evidence/tts-voice-distinctness-*/
# относительно REPO, а не HERE (= scripts/e2e/). Используется
# _validate_voice_distinctness, см. ADR-0134 §2.2.
REPO_ROOT = REPO

# --- голоса синтеза (люди) ---------------------------------------------------
SASHA = "anton"    # представляется
BORIS = "ermil"    # представляется
GRISHA = "zahar"   # не представляется
VALERA = "filipp"  # не представляется

# --- маркеры логов dialog v2 (SSoT — код робота) -----------------------------
# PR-14 (#3880, эпик #3683): v2 — диалог по умолчанию, v1 удаляется в PR-15. Ожидания сценариев — только по
# строкам v2; ожидания v1 (``LLM dispatch skipped``, ``[backlog] flushed to LLM``, тул ``speak_text``,
# ``speaker='…'`` в строке бэклога) не сохранены.
#
# dialog_node.py:277   [backlog] accumulated reason=no_wake entries=N     (диктора в строке НЕТ)
# turn_log.py          📊 [turn] turn_id=… stage=done status=finished latency_ms=… llm_calls=N …
# speech_owner.py:411  🗣️ SpeechEvent started|finished|cancelled speech_id=… agent=… epoch=… text_len=…
# dialog_node.py:350   🛑 [interrupt] <policy> via=… '<текст>' epoch=… boundary=… turns_cancelled=…
BACKLOG_ACC = r"\[backlog\] accumulated reason=no_wake"
#: Ход дошёл до конца штатно (``stage=done status=finished``).
TURN_DONE = r"📊 \[turn\] .*stage=done status=finished"
#: Ход с обращением к LLM (голый вейк без бэклога — рефлекс «слушаю», ``llm_calls=0``).
TURN_WITH_LLM = TURN_DONE + r" .*llm_calls=[1-9]"
#: Робот начал говорить: замена v1-ожидания тула ``speak_text`` (в v2 речь — ``command.speech``, тула нет).
SPEECH_STARTED = r"SpeechEvent started speech_id"
#: Речь погашена перебиванием: ``SpeechEvent cancelled`` владельца речи или решение ``🛑 [interrupt]`` ноды.
SPEECH_CANCELLED = r"SpeechEvent cancelled|🛑 \[interrupt\]"
#: Тулы музыки, которыми v2 законно отвечает на «поставь что-нибудь»: ``📤 Отправлен запрос <id>: <tool>``.
MUSIC_DISPATCH = r"📤 Отправлен запрос [0-9a-f]+: (request_music|generate_music|music_library|dj_set)"


def spoken_by(provider: str) -> str:
    """Реплика синтезирована провайдером ``provider`` (``speech_owner``: ``SpeechEvent synthesized … provider=<имя>``).

    Голый ``provider=<любой>`` -- вакуум (R6 разбора 26.10: совпадает с каждым ходом); шаг называет провайдера,
    которым обязан звучать: голос по умолчанию -- MiniMax, Алёна и Джейн -- голоса Yandex (``tts_voice_registry``)."""
    return r"SpeechEvent synthesized speech_id=.* provider=" + provider + r"\b"


# --- выступление (perform) v2: старт и конец по событиям владельца, а не по окнам (#3971 R4) ----------------------
# turn.py:      🧭 [turn-v2] выступление старт kind=<kind> backing=… segments=N → {…}
# perform.py:   🎤 [perform] perform-: конец (done) сегментов N, символов M, …
# plan_events:  🗓️ [plan] PlanEvent block_end block=perform-… reason=done
# В ``actions`` команды perform НЕ попадает (command_bridge логирует actions=[]), v1-ожидание ``actions=[…perform…]``
# давало PATTERN_MISS на верной сказке, ретрай заказывал вторую (n412, 23.10 22:04:48).


#: Начало строки старта выступления (``turn.py``). Шаг с этим паттерном ретрая не имеет: повтор фразы после старта
#: заказал бы второе выступление (n412 23.10: вторая сказка, 2.5 мин) -- ``step`` такой шаг не соберёт.
PERFORM_STARTED_MARK = "выступление старт kind="


def perform_started(kind: str) -> str:
    """Выступление вида ``kind`` (tale/poem/rap/song) взял владелец."""
    return PERFORM_STARTED_MARK + kind + r"\b"


#: Выступление дочитано до конца: строка владельца выступления или его ``PlanEvent block_end … reason=done``.
PERFORM_DONE = r"\[perform\] [^ ]*: конец \(done\)|PlanEvent block_end .*reason=done"
#: Вопрос выступления вместо него (``phrases/ru.yaml`` ``perform.ask_replace_music``: «Сейчас играет …, выключить и
#: зачитать?»). Это не история и не рэп, а ``SpeechEvent started`` он даёт -- ложный PASS n804/n806/n810 23.10
#: (#3971 R22, робот 22:53:09: на «под этот бит зачитай рэп» спросил, выключить ли бит). Ищется в РЕЧИ робота.
PERFORM_ASKS_TO_REPLACE = "выключить и зачитать"

# --- результат действий без речи и лица (решения Шифу 26.10, #4036/#4041) -----------------------------------------
#: Лицо команды на итоге хода (``dialog_v2_ports.stage``): ``🎭 [turn-v2] лицо команды на done: emotion=<сцена> …``.
FACE_DONE = r"🎭 \[turn-v2\] лицо команды на done: emotion="


def face_or_animation(*emotions: str) -> str:
    """Робот ПОКАЗАЛ эмоцию: лицо команды на ``done`` с одной из ``emotions`` (сцена ``knowledge.EMOTIONS``) или
    ``play_animation`` с такой анимацией. Тул анимации не обязателен: v2 показывает эмоцию ``command.expression``."""
    alt = "(" + "|".join(emotions) + ")"
    return FACE_DONE + alt + r"\b|play_animation с параметрами \{.*'animation': '" + alt + "'"


# --- скорость и тон голоса (#4039: тул меняет параметр АКТИВНОГО провайдера, tts_node пишет «изменён») ------------
#: Оси скорости по провайдерам -- ``rob_box_voice.tts_prosody.SPEED_AXES`` (drift держит тест генератора).
SPEED_PARAMS = ("minimax_speed", "yandex_speed")
#: Ось тона -- ``tts_prosody.PITCH_AXIS``.
PITCH_PARAM = "pitch_shift"
#: Значение оси в строке ``tts_node``: ``🎚️ <param> изменён: <значение :g>`` -- «0.8», «1», «1.2», «2».
BELOW_NORMAL = r"0\.[0-9]+"
ABOVE_NORMAL = r"(1\.[0-9]+|2)(\s|$)"
NORMAL = r"1(\s|$)"


def prosody_changed(params: tuple, value: str) -> str:
    """Параметр голоса реально изменён нодой (``tts_node.parameters_callback``), а не «тул вызван»."""
    return r"(" + "|".join(params) + r") изменён: " + value


#: «Уже нормально» -- тул прочитал ось назад и она на норме (``system._Step`` normal.edge): тоже результат, не вызов.
SPEED_ALREADY_NORMAL = r"\[set_speed\] Speed \[[a-z]+, [a-z_]+\]: Скорость уже нормальная"
PITCH_ALREADY_NORMAL = r"\[set_pitch\] Pitch \[[a-z]+, pitch_shift\]: Голос уже нормальный"


def speaker_sure(name: str) -> str:
    """Голос опознан как ``name`` с уверенностью ≥ 0.8 (``speaker_id_node``: ``👤 Speaker: 'Имя' confidence=0.914``).

    Замена вакуумного ``Speaker|identify candidates|speaker_id`` (R6: совпадал с каждым ходом, в т.ч. ``unknown``)."""
    return r"👤 Speaker: '" + name + r"' confidence=(0\.[89]|1\.)"


#: Issue #3165 — ответ «тишина» на вопрос о состоянии музыки (n110, n313):
#: альтернативы через «|», подстрокой по речи робота (см. keyword_hit).
SILENCE_ANSWER = "тишин|тихо|ничего не играет|не играет|не звучит|музыки нет"
#: Issue #3165 — «Не получилось выполнить» на вопрос, где ничего не
#: исполнялось, — ложная фраза-отказ (fallback #2949).
FALSE_FAILURE_PHRASE = "не получилось выполнить"
#: Ложное утверждение «тихо» на вопрос, когда музыка играет (n503, n603): запрещённая фраза в РЕЧИ робота.
SILENCE_CLAIM = "тишин|ничего не играет|не играет|не звучит|музыки нет"
#: Ответ «играет» (n503, n603): глагол состояния или факт трека (стиль, темп, ритм) -- по снимку ``<music_state>``.
PLAYING_ANSWER = "играет|играю|звучит|звучат|крутится|крутим|идёт|идет|bpm|темп|ритм|бит|рок|трек|мелоди"
#: «Музыка выключена» -- ответ на n1005 в формулировке вопроса.
SILENCE_OR_OFF_ANSWER = SILENCE_ANSWER + "|выключен"
#: Зонд утечки истории после «новой сессии» (n1007, R20): стемы фраз ОКНА разговора, которых нет в долгой памяти --
#: вопросы акта 10 до сброса (n1001, n1002) и темы акта 9 (n907, n909, n910). Факты памяти (енот, Спартак, лук, чай)
#: сюда не входят: после сброса они законно приходят из memory_search.
LEAK_PROBE_WINDOW_PHRASES = ["ворчал", "итог ночи", "космос", "китайск", "четверостиш"]

# --- маркеры РЕЗУЛЬТАТА действия (#3971, разбор марафона 23.10 §1.A) -----------------------------------------
# Харнесс судит по тому, что случилось, а не по имени тула и не по наличию речи: «поставлю ритм» без тула и
# «Не понял» поверх трека из кэша раньше давали PASS (n502, n703, n711, n809). Маркеры -- строки робота
# (``mcp_server.py``: ``✅ Инструмент <tool> выполнен успешно``; ``player_owner.py``: ``🎵 [music v2] started|stop``).
# Идут в ``patterns`` шага (grep -E по логам шага, AND) -- второго механизма acceptance нет.
#: Музыкальный движок v2 реально поставил трек на деку.
MUSIC_STARTED = r"🎵 \[music v2\] started track_id="
#: Музыкальный движок v2 реально снял деку С ТРЕКОМ: id трека -- ``gen…``/``req…``/``set…`` (строчные), а
#: ``track_id=None`` -- стоп вхолостую на пустой деке (n812 23.10: «Выключил музыку.» при ``was_playing=False``).
MUSIC_STOPPED = r"🎵 \[music v2\] stop track_id=[a-z0-9]"
#: Тул остановки застал музыку играющей (``music.StopMusicTool``: ``[stop_music] was_playing=True`` или outro бита
#: выступления ``{'was_playing': True, …}``).
MUSIC_WAS_PLAYING = r"\[stop_music\] .*was_playing(=|': )True"
#: Любая остановка деки (подстрока для ``must_not_call``: «а музыку оставь» -- деку не трогать).
MUSIC_STOP_ANY = "[music v2] stop"
#: Фраза потолка действия музыки (``phrases/ru.yaml`` ``media.timeout``): там, где музыка обязана заиграть, это ложь
#: (n809: «Музыка пока не заиграла.», трек стартовал через 1,4 с). Ищется в РЕЧИ робота (``must_not_say``).
MUSIC_NOT_STARTED = "Музыка пока не заиграла"
#: Tier-1 «хватит читать/говорить» (#4035): гасит речь и выступление, плеер музыки не трогает.
TIER1_STOP_SPEECH = r"tier1 kind=stop_speech\b"
#: Ход упал по дедлайну (``stage=timeout``: «долго думаю» поверх живой речи) -- подстрока для ``must_not_call``.
TURN_TIMEOUT = "stage=timeout"

# --- предусловия внешнего мира (R21 разбора 26.10) ------------------------------------------------------------------
#: Баланс KIE кончился (``engine/gen/cover.py``: ``🎼 [cover] баланс KIE: 0.0``). Песни этого прогона нет -- шаги
#: «сохрани/расскажи/включи/удали эту песню» теряют смысл, а delete бьёт по чужому треку (n710, 23.10 22:39:22).
KIE_QUOTA_LOG = r"\[cover\] баланс KIE: 0(\.0+)?(\s|$)"
#: Причина пропуска -> строка лога, при которой шаг не играется. Одна таблица; шаги ссылаются на ключ.
SKIP_WHEN: Dict[str, str] = {"KIE_QUOTA": KIE_QUOTA_LOG}
#: Тулы живой правки/сета музыки v2: любой из них -- честная попытка поменять играющее (n504-n506, n604, n607).
MUSIC_EDIT_TOOLS = ("dj_set", "request_music", "generate_music")
#: Тулы, которыми v2 законно запускает музыку (n310: «поставь что-нибудь»).
MUSIC_START_TOOLS = ("request_music", "generate_music", "music_library", "dj_set")


def tool_done(*tools: str) -> str:
    """Паттерн «тул выполнен успешно» (``✅ Инструмент <tool> выполнен успешно``); несколько тулов -- любой из них."""
    name = tools[0] if len(tools) == 1 else "(" + "|".join(tools) + ")"
    return "✅ Инструмент " + name + " выполнен успешно"


#: Перебить через 3 с после начала речи робота (первая реплика ответа на предыдущую фразу).
BARGE_3S: Dict[str, Any] = {"after": "speech_started", "n": 1, "delay_sec": 3.0}


def step(
    label: str,
    text: str,
    voice: str = SASHA,
    patterns: Optional[List[str]] = None,
    expect_tools: Optional[List[str]] = None,
    must_not: Optional[List[str]] = None,
    must_not_say: Optional[List[str]] = None,
    must_not_identify_as: Optional[List[str]] = None,
    keywords: Optional[List[str]] = None,
    voice_changed: bool = False,
    expect: str = "",
    retry: int = 0,
    why: str = "",
    discovery_tools: Optional[List[str]] = None,
    barge_in: Optional[Dict[str, Any]] = None,
    silent_ok: bool = False,
    skip_when: str = "",
) -> Dict[str, Any]:
    s: Dict[str, Any] = {"label": label, "voice": voice, "text": text}
    # Issue #3857 -- barge_in: НАМЕРЕННОЕ перебивание. Без него харнесс перед шагом ждёт тишины, и
    # «стой/хватит посреди речи» на деле говорилось в тишину. С ним: ждём якорь в речи предыдущей
    # фразы, играем поверх через delay_sec. Якорь не наступил -> SKIP barge_in_anchor_missing.
    if barge_in:
        s["barge_in"] = barge_in
    # Issue #3863/#3880 -- silent_ok: ход без речи допустим (Tier-1 «стоп»/«хватит»: llm_calls=0, речи нет, речь
    # робота гасится). Харнесс (e2e_voice_lib.sh) читает поле ``silent_ok``; конец хода -- ``📊 [turn] stage=done``.
    # Само по себе поле молчание не проверяет -- смысл шагу даёт паттерн отмены речи.
    if silent_ok:
        s["silent_ok"] = True
    s["patterns"] = patterns or []
    # #3971 R4: после старта выступления повтор фразы заказывает второе -- ретрай у такого шага запрещён структурой.
    if retry and any(p.startswith(PERFORM_STARTED_MARK) for p in s["patterns"]):
        raise ValueError(f"{label}: шаг ждёт выступление ({PERFORM_STARTED_MARK}…) -- retry запрещён (#3971 R4)")
    acc = _acceptance(
        expect_tools, discovery_tools, must_not, must_not_say, must_not_identify_as, keywords, voice_changed, why
    )
    if acc:
        s["acceptance"] = acc
    if expect:
        s["expect"] = expect
    if retry:
        s["retry_acceptance"] = retry
    if why and "acceptance" not in s:
        s["_why"] = why
    # R21 разбора 26.10: предусловие внешнего мира (ключ SKIP_WHEN). Поле с «_» -- харнесс его не читает (см. шапку).
    if skip_when:
        s["_skip_when"] = {"reason": skip_when, "log": SKIP_WHEN[skip_when]}
    return s


def _acceptance(
    expect_tools: Optional[List[str]],
    discovery_tools: Optional[List[str]],
    must_not: Optional[List[str]],
    must_not_say: Optional[List[str]],
    must_not_identify_as: Optional[List[str]],
    keywords: Optional[List[str]],
    voice_changed: bool,
    why: str,
) -> Dict[str, Any]:
    """Блок ``acceptance`` шага (пустой -- шаг без него). Порядок ключей -- как в JSON актов (сверяет ``--check``).

    Issue #2406 -- ``discovery_tools``: тул, который ОБЯЗАН быть вызван ДО первого голосового ответа (проверяется
    check_acceptance через first_invocation_position/first_voice_cycle_position).
    Issue #2779 -- ``must_not_say`` НЕ то же самое, что ``must_not``: ``must_not`` (-> ``must_not_call``) свободным
    текстом матчит ВЕСЬ лог шага, включая строки биометрии (``Speaker: 'Борис'``), там имя диктора тавтологически
    КРАСНО. ``must_not_say`` ищет ТОЛЬКО в речи робота (``robot_speech()``, тот же канал, что ``expected_keywords``).
    """
    fields = (
        ("expected_tool_calls", expect_tools),
        ("discovery_tools", discovery_tools),
        ("must_not_call", must_not),
        ("must_not_say", must_not_say),
        ("must_not_identify_as", must_not_identify_as),
        ("expected_keywords", keywords),
        ("voice_changed", True if voice_changed else None),
    )
    acc: Dict[str, Any] = {key: value for key, value in fields if value}
    if not acc:
        return acc
    acc.setdefault("expected_tool_calls", [])
    acc.setdefault("must_not_call", [])
    if why:
        acc["_comment"] = why
    return acc


def action_step(
    label: str,
    text: str,
    result_of: Optional[List[str]] = None,
    *,
    music: str = "",
    known_red: str = "",
    patterns: Optional[List[str]] = None,
    must_not: Optional[List[str]] = None,
    **kwargs: Any,
) -> Dict[str, Any]:
    """Шаг-действие: PASS только при МАРКЕРЕ РЕЗУЛЬТАТА в логе робота (#3971), а не по речи или имени тула.

    ``result_of`` -- тулы, любой из которых обязан закончиться ``✅ Инструмент <tool> выполнен успешно``;
    ``music`` -- ``"started"`` (``🎵 [music v2] started``: трек встал на деку), ``"changed"`` (играющую музыку
    правят: трек может и не смениться, маркера деки не ждём) или ``"stopped"`` (``🎵 [music v2] stop`` с id трека:
    дека снята); при ``started``/``changed`` робот не вправе сказать фразу потолка ``Музыка пока не заиграла``
    (#3971 R22). Всегда: ход дошёл до ``stage=done status=finished`` и в шаге нет ``stage=timeout``.
    ``known_red`` -- причина, по которой шаг сегодня красный по делу (поле ``_known_red`` для людей; харнесс его
    не читает, шаг остаётся красным честно).
    """
    pats = list(patterns or [])
    if result_of:
        pats.append(tool_done(*result_of))
    pats.extend({"started": [MUSIC_STARTED], "stopped": [MUSIC_STOPPED]}.get(music, []))
    pats.append(TURN_DONE)
    if music in ("started", "changed"):
        kwargs["must_not_say"] = list(kwargs.get("must_not_say") or []) + [MUSIC_NOT_STARTED]
    s = step(label, text, patterns=pats, must_not=list(must_not or []) + [TURN_TIMEOUT], **kwargs)
    s["_result_markers"] = {"tools": list(result_of or []), "music": music}
    if known_red:
        s["_known_red"] = known_red
    return s


# =============================================================================
# ADR-0134 §2.2 — provider-aware guard: «выбранные голоса разводятся?»
# =============================================================================
#
# Контракт (см. ADR-0134 §2.2, evidence/tts-voice-distinctness-2026-09-22/):
# 1. Из steps[] собираем уникальные voice= (не None).
# 2. Поднимаем evidence/tts-voice-distinctness-*/<provider>_voices.json.
#    Файл замера лежит в evidence/tts-voice-distinctness-YYYY-MM-DD/
#    (ближайшая по дате директория к REPO). У файла формат::
#        {"voices": ["A","B",...], "inter_voice_max_cos": {"A|B": 0.95, ...}}
#    Ключ «A|B» — пара голосов; значение — максимальный косинус,
#    который БЫЛ зафиксирован между ними на замере. Чем ближе к 1.0, тем
#    менее различимы голоса.
# 3. Для КАЖДОЙ пары уникальных голосов (a, b) смотрим значение в
#    ``inter_voice_max_cos`` — сначала ключ ``a|b``, потом ``b|a``
#    (замерщик мог сохранить только одну сторону). Если значение
#    ``>= threshold`` — это нарушение: такие голоса не разводятся identify().
# 4. Самопара ``voice|voice``: если в шагах ВСЕ voice= одинаковые (один
#    уникальный голос использован N>=2 раз), проверяем запись ``v|v`` —
#    «voice v имеет плохую внутреннюю согласованность (samples расходятся
#    друг с другом, как разные голоса)». Это закрывает дыру «4 шага с
#    voice='A' едут в сценарий и identify() не различает шаги».
# 5. Поведение по умолчанию при нарушении:
#    * нарушение найдено → ``raise SystemExit(2)`` с диагностикой (на CI
#      генерация должна падать, чтобы e2e не уехал в неразличимый сценарий);
#    * файла замера нет (greenfield-провайдер) → warning в stderr, НЕ raise
#      (см. ADR §5 trade-off #1).
#
# Helper сделан отдельно от emit(), чтобы:
# * его можно было прогнать из pytest с подменой evidence (monkeypatch);
# * логика была одна и для greenfield-провайдера, и для прода;
# * ``step()`` остался чистым (kw-only параметры — это scope backend).
def _validate_voice_distinctness(
    steps: List[Dict[str, Any]],
    provider: str,
    threshold: float,
    *,
    loader: Optional[Callable[[str], Optional[Dict[str, Any]]]] = None,
    evidence_root: Optional[str] = None,
    warn: Optional[Callable[[str], None]] = None,
) -> Dict[str, Any]:
    """Поднимает evidence/<provider>_voices.json и валидирует пары.

    Параметры
    ---------
    steps : list[dict]
        Список шагов (то, что вернул ``step()``). Используется только
        ``step["voice"]``; ``None``/отсутствие пропускаются.
    provider : str
        Имя TTS-провайдера (``"minimax"``, ``"yandex"``, ...).
    threshold : float
        Максимально допустимый косинус между любыми двумя голосами шагов.
        Если в evidence есть пара ``>= threshold`` — raise ``SystemExit(2)``.
    loader : callable, optional
        ``loader(provider) -> dict | None``. По умолчанию — поиск файла
        в evidence/tts-voice-distinctness-*/<provider>_voices.json. Точка
        расширения для тестов (monkeypatch).
    evidence_root : str, optional
        Корень поиска evidence-директорий. По умолчанию — REPO_ROOT/evidence.
    warn : callable, optional
        ``warn(msg)``. По умолчанию — ``print(..., file=sys.stderr)``.
        Используется для greenfield-предупреждения (файл замера отсутствует).
    """
    import sys as _sys
    if warn is None:
        def warn(msg: str) -> None:
            print("WARNING: " + msg, file=_sys.stderr)

    # 1. Собрать уникальные voice=
    voices: List[str] = sorted({
        s["voice"] for s in steps
        if isinstance(s, dict) and isinstance(s.get("voice"), str)
    })
    # Проверять есть смысл только если в шагах использовано >=2 раз
    # один и тот же voice= (или разных >=2). Если 0 или 1 шаг с voice=
    # — distinctness-инвариант не имеет смысла (нечего разводить).
    n_voice_steps = sum(
        1 for s in steps
        if isinstance(s, dict) and isinstance(s.get("voice"), str)
    )
    if n_voice_steps < 2:
        return {"ok": True, "voices": voices, "violations": [], "missing": False}

    # 2. Загрузить evidence (или сразу вызвать кастомный loader — для тестов).
    if loader is not None:
        data = loader(provider)
    else:
        root = evidence_root or os.path.join(REPO_ROOT, "evidence")
        data = _load_voice_distinctness_evidence(root, provider)

    if data is None:
        # 5. Greenfield: файла нет — warning, не error (см. ADR §2.2 + §5 #1).
        warn(
            "no voice-distinctness evidence for provider %r "
            "(look in evidence/tts-voice-distinctness-*/). "
            "Run scripts/e2e/measure_tts_voice_distinctness.py --provider=%s "
            "to populate it. Skipping distinctness check." % (provider, provider)
        )
        return {
            "ok": True, "voices": voices, "violations": [],
            "missing": True, "evidence_file": None,
        }

    # 3-4. Проверить пары (a, b) и самопары (v, v) — если в шагах
    # только один уникальный голос, проверять «пары» нечего, но
    # самопара ``v|v`` ловит «samples v расходятся друг с другом».
    inter = data.get("inter_voice_max_cos", {}) if isinstance(data, dict) else {}
    violations: List[Dict[str, Any]] = []

    def _cos_for(a: str, b: str):
        """Значение из evidence для пары (a, b). Возвращает float или
        None, если замерщик эту пару не покрыл."""
        v = inter.get("%s|%s" % (a, b))
        if v is None:
            v = inter.get("%s|%s" % (b, a))
        return v

    if len(voices) == 1:
        # Только один уникальный голос — смотрим самопару.
        v = voices[0]
        cos = _cos_for(v, v)
        if cos is not None and cos >= threshold:
            violations.append({
                "pair": [v, v], "max_cos": cos, "threshold": threshold,
            })
    else:
        for i, a in enumerate(voices):
            for b in voices[i + 1:]:
                cos = _cos_for(a, b)
                if cos is None:
                    # Пара не замерена — пропускаем (не наша забота сейчас;
                    # отдельный PR добавит «нет данных ⇒ fail», когда будут
                    # полные матрицы по всем провайдерам).
                    continue
                if cos >= threshold:
                    violations.append({
                        "pair": [a, b], "max_cos": cos, "threshold": threshold,
                    })

    if violations:
        # Сортируем «самый плохой» сверху, чтобы сообщение вело прямо в корень.
        violations.sort(key=lambda x: -x["max_cos"])
        worst = violations[0]
        worst_pair = "|".join(worst["pair"])
        msg = (
            "voice distinctness guard (ADR-0134 §2.2): provider=%s "
            "has indistinguishable voices %s with max cosine %.3f "
            ">= threshold %.3f. All %d violating pairs: %s. "
            "Either pick more distinct voices for the scenario, raise "
            "--voice-distinctness-threshold, or regenerate the scenario "
            "with a different cast."
            % (
                provider, worst_pair, worst["max_cos"], worst["threshold"],
                len(violations),
                ", ".join(
                    "%s(%.3f)" % ("|".join(v["pair"]), v["max_cos"])
                    for v in violations
                ),
            )
        )
        # Диагностика в stderr ОБЯЗАТЕЛЬНА перед raise — иначе
        # SystemExit(2) на CI проглатывается без звука (exit-code
        # без сообщения = «CI красный, но непонятно почему»). Это
        # та же причина, по которой helper печатает warning при
        # greenfield: оператор должен ВИДЕТЬ проблему.
        _sys.stderr.write(msg + "\n")
        _sys.stderr.flush()
        # 4. Fail-fast: падаем именно на этапе генерации, чтобы e2e не
        # уехал в неразводимый сценарий.
        raise SystemExit(2)

    return {
        "ok": True, "voices": voices, "violations": [],
        "missing": False, "evidence_file": data.get("_source_path"),
    }


#: Имя файла замера для провайдера (ищется внутри каждой evidence-папки).
_VOICE_DISTINCTNESS_FILENAME_TPL = "%s_voices.json"


def _load_voice_distinctness_evidence(
    evidence_root: str, provider: str
) -> Optional[Dict[str, Any]]:
    """Найти и загрузить evidence/tts-voice-distinctness-*/<provider>_voices.json.

    Берём ПОСЛЕДНЮЮ по дате директорию (лексикографически == ISO-дата),
    в которой лежит нужный файл. Если в репо несколько замеров — используем
    самый свежий; остальные остаются для исторической сверки.
    """
    if not os.path.isdir(evidence_root):
        return None
    pattern = re.compile(r"^tts-voice-distinctness-(\d{4}-\d{2}-\d{2})$")
    candidates: List[str] = []
    fname = _VOICE_DISTINCTNESS_FILENAME_TPL % provider
    for entry in sorted(os.listdir(evidence_root)):
        full = os.path.join(evidence_root, entry)
        if not (pattern.match(entry) and os.path.isdir(full)):
            continue
        if os.path.isfile(os.path.join(full, fname)):
            candidates.append(full)
    if not candidates:
        return None
    path = os.path.join(candidates[-1], fname)
    try:
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(data, dict):
        return None
    # Прикрепляем путь — пригодится для диагностики в caller'е.
    try:
        data["_source_path"] = path  # type: ignore[index]
    except (KeyError, TypeError):
        pass
    return data


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
    recover: Any = None,
) -> None:
    """``recover`` -- поле акта для харнесса (#3989/#4034, ``e2e_events.recover_kinds``): ``None`` -- не писать
    (по умолчанию харнесс разрешает все восстановления), иначе ``True``/``False``/список видов."""
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
            "recover": recover,
        }
    )


# =============================================================================
# АКТ 1 — «Смена началась». Робот просыпается, мы снимаем базовые показания.
# Смысл порядка: сначала СБРОС сессии (чистый лист для всей ночи), потом
# опрос состояния. Последний шаг — вопрос о ТИШИНЕ (ответ по снимку): мы обязаны
# доказать, что музыки нет, ПРЕЖДЕ чем в акте 5 её включать и выключать.
# «Выключи музыку» вслепую здесь запрещено (must_not_call: stop_music).
# =============================================================================
act(
    1,
    "wakeup",
    "Смена началась",
    "Ночь, мастерская, Саша заступает на смену. Базовый прогрев: wake-word, "
    "сброс сессии, паспортные тулы (время/батарея/статус), альтернативное "
    "wake-слово «Робокс» (#1252), короткий статусный вопрос (#1279) и — ключевое — "
    "ЗАМЕР ТИШИНЫ: доказываем, что музыка не играет, до того как её включать.",
    "stable",
    ["get_current_time", "get_battery_level", "get_robot_status"],
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
            patterns=["session reset|session_reset|Новая сессия|new session|tier1 kind=new_session"],
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
            patterns=[TURN_DONE],
            why="#1279: короткий статусный вопрос получает ответ, ход не падает. v1-гейт "
            "(`LLM dispatch skipped|command intent`) в v2 нет: путь решает Tier-1 / команда "
            "LLM (#3880, run 39869288905 — ответ был, сценарий ждал строку гейта). Проверяем "
            "исход: ход дошёл до `stage=done status=finished`, а полный цикл (речь) подтверждает "
            "харнесс. Что короткий вопрос НЕ ходит в LLM, больше не утверждаем -- v2 этого не обещает.",
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
            keywords=[SILENCE_ANSWER],
            must_not=["stop_music"],
            must_not_say=[FALSE_FAILURE_PHRASE],
            why="ЯКОРЬ ТИШИНЫ. Робот обязан ответить по состоянию плеера, а "
            "не рефлекторно дёрнуть stop_music на молчащем плеере. Всё, что "
            "акты 5-8 делают с музыкой, отсчитывается от этого шага. "
            "Issue #3165: ответ — из <music_state> (снимок плеера), "
            "get_music_state не обязателен; проверяем сам ответ «тишина».",
        ),
    ],
    [
        "Акт 1 — базовый прогрев. Красный здесь = робот не отвечает вообще,",
        "дальше по актам идти бессмысленно (раннер останавливает марафон).",
        "get_perception_context и faq_search намеренно ВНЕ gate — они зависят",
        "от железа стенда и профиля FAQ (см. dialogue-coverage-map §3).",
        "recover=[silenced]: акт проверяет САМ вейк -- потерянный вейк харнесс не лечит",
        "«Робот!» (no_wake), шаг честно красный; тишину, оставленную прошлым прогоном, снимать можно.",
    ],
    # #4034: акт 1 -- проверка вейка. Восстановление no_wake («Робот!» подхватывает бэклог) спрятало бы ровно то,
    # что акт меряет; silenced (тишина с прошлого прогона) к вейку отношения не имеет и восстанавливается.
    recover=["silenced"],
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
    "даёт неустойчивый эмбеддинг при identify_threshold=0.75. Тётя Гриша "
    "(zahar, женский голос по касту #4038) заходит и намеренно НЕ называет имя — это контрольная группа "
    "«незнакомец» для акта 3.",
    "new-unproven",
    ["register_speaker", "memory_save", "memory_search"],
    [
        step(
            "n201_sasha_intro_long",
            "Робот! Привет. Давай знакомиться как следует. Меня зовут Саша, я твой "
            "ночной инженер и механик.",
            voice=SASHA,
            expect="cycle",
            expect_tools=["register_speaker"],
            discovery_tools=["register_speaker"],
            retry=1,
            why="Разбито на n201/n201b/n201c: audio_node режет речь длиннее "
            "15с VAD-окна (run 35658231116, ❌ Речь отклонена: 17.37с), "
            "исходная реплика ~25с не доходила до STT. Имя и register_speaker "
            "остаются на этом шаге. expect=cycle: первый шаг акта пробивает "
            "wake-gate. issue #2406: discovery_tools — register_speaker "
            "должен быть вызван ДО голосового ответа (regression guard). "
            "Issue #2907: «и механик» удлиняет реплику регистрации (85 → 95 "
            "символов), чтобы чистой речи после VAD было с запасом над "
            "порогом эталона 3.0с (run 35916899022: 51 символ дал 2.97с "
            "→ too_short). Потолок — test_e2e_scenario_playable.sh (<=96 "
            "символов без WARN): длиннее уже упирается в 15с VAD-окно.",
        ),
        step(
            "n201b_sasha_backstory",
            "Я собирал тебе блок питания и переделывал левый мотор. Запомни мой "
            "голос как следует.",
            voice=SASHA,
            expect="backlog",
            why="Продолжение n201_sasha_intro_long — см. why там (run 35658231116, "
            "лимит VAD 15с). Суммарная длина речи сохранена для устойчивого "
            "d-vector.",
        ),
        step(
            "n201c_sasha_voice_note",
            "Потому что ночью я буду говорить с тобой чаще всех остальных в "
            "этой мастерской.",
            voice=SASHA,
            expect="backlog",
            why="Хвост n201_sasha_intro_long — см. n201b.",
        ),
        step(
            "n202a_sasha_warmup",
            "Робот, слушай дальше и запоминай, как я звучу.",
            voice=SASHA,
            patterns=[speaker_sure("Саша")],
            must_not=["memory_save"],
            why="n202 разбит на три шага: 15с VAD-окно (run 35658231116) не "
            "пропускало исходную реплику ~20с. R6 разбора 26.10: прежний паттерн "
            "`Speaker|identify candidates|speaker_id` совпадал с каждым ходом (и с `Speaker: unknown`) и спрятал "
            "незапрошенный memory_save из бэклога. Теперь результат: голос опознан как Саша с уверенностью >= 0.8; "
            "«запоминай, как я звучу» -- про голос, факт в память не пишется (must_not memory_save).",
        ),
        step(
            "n202b_sasha_warmup_hum",
            "Вчера я всю ночь паял этот несчастный блок питания, и он всё равно "
            "гудит как трансформатор.",
            voice=SASHA,
            patterns=[speaker_sure("Саша")],
            expect="backlog",
        ),
        step(
            "n202c_sasha_warmup_guess",
            "А я так и не понял, земля виновата или дроссель.",
            voice=SASHA,
            patterns=[speaker_sure("Саша")],
            expect="backlog",
        ),
        step(
            "n203_sasha_memory_tea",
            "Робот, запомни: я пью только зелёный чай без сахара и совсем не ем "
            "лук, даже жареный.",
            voice=SASHA,
            expect_tools=["memory_save"],
            why="Сокращено под бюджет VAD 15с (run 35658231116), оба факта "
            "(зелёный чай без сахара, не ест лук) сохранены — нужны в акте 9 "
            "(рецепт борща без лука) и в акте 10 (память переживает сброс "
            "сессии).",
        ),
        step(
            "n204_boris_intro_long",
            "Робот, добрый вечер, меня зовут Борис, я друг Саши и прихожу с "
            "пиццей по пятницам, раз в неделю.",
            voice=BORIS,
            expect_tools=["register_speaker"],
            discovery_tools=["register_speaker"],
            retry=1,
            why="Разбито на n204/n204b/n204c: audio_node режет речь длиннее "
            "15с VAD-окна (run 35658231116) — исходная реплика ~24с не "
            "доходила до STT. Имя и register_speaker остаются на этом шаге. "
            "issue #2406: discovery_tools — register_speaker должен быть "
            "вызван ДО голосового ответа (regression guard). Issue #2907: "
            "«по пятницам» удлиняет реплику регистрации (92 → 96 символов, "
            "факт «раз в неделю» сохранён) — ровно до потолка "
            "test_e2e_scenario_playable.sh без WARN; длиннее упирается в 15с "
            "VAD-окно.",
        ),
        step(
            "n204b_boris_voice_note",
            "Запомни мой голос отдельно от Сашиного, мы очень по-разному звучим.",
            voice=BORIS,
            expect="backlog",
            why="Продолжение n204_boris_intro_long — см. why там (run "
            "35658231116, лимит VAD 15с).",
        ),
        step(
            "n204c_boris_voice_reason",
            "Я не хочу, чтобы ты нас путал, когда мы оба будем тебе что-то "
            "говорить.",
            voice=BORIS,
            expect="backlog",
            why="Хвост n204_boris_intro_long — см. n204b.",
        ),
        step(
            "n205a_boris_warmup",
            "Робот, продолжаю говорить, чтобы ты привык к моему голосу.",
            voice=BORIS,
            patterns=[speaker_sure("Борис")],
            why="n205 разбит на три шага: 15с VAD-окно (run 35658231116) не "
            "пропускало исходную реплику ~20с. R6: результат -- голос опознан как Борис (>= 0.8) на каждой части, "
            "а не любой след биометрии.",
        ),
        step(
            "n205b_boris_warmup_spartak",
            "Я болею за Спартак с девяносто восьмого года, и Саша каждый раз "
            "надо мной смеётся.",
            voice=BORIS,
            patterns=[speaker_sure("Борис")],
            expect="backlog",
        ),
        step(
            "n205c_boris_warmup_tease",
            "Хотя сам вообще футбол не смотрит и путает вратаря с защитником.",
            voice=BORIS,
            patterns=[speaker_sure("Борис")],
            expect="backlog",
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
        # Issue #2865: здесь раньше стоял n208_memory_search_tea. Он
        # спрашивал про чай, пока факт n203 ещё лежал в окне чата
        # («Записал, Саша. Зелёный чай без сахара…»): LLM отвечала из
        # истории, memory_search не звала (run 35886659057, tools=[]).
        # Проверка #2793 перенесена в конец акта — ПОСЛЕ сброса сессии
        # (n212/n213), где факт живёт только в voice_facts.
        step(
            "n209_recall_boris",
            "Робот, а про меня что помнишь?",
            voice=BORIS,
            keywords=["Борис|Спартак|пицц"],
        ),
        step(
            "n210_grisha_no_name",
            "Робот, я мимо шла. Имя своё я тебе называть не буду, я старой "
            "закалки и не доверяю железкам.",
            voice=GRISHA,
            must_not_identify_as=["Борис", "Саша"],
            # Issue #2779 AC3 — имена и факты ДРУГИХ дикторов каста не
            # должны ПРОЗВУЧАТЬ незнакомцу. Это must_not_say (проверяется
            # только по robot_speech(), т.е. по РЕЧИ робота), а НЕ
            # must_not_call: свободный текст в must_not_call матчится по
            # ВСЕМУ логу шага (tool_invoked() fallback), а там
            # гарантированно есть строки биометрии `identify candidates:
            # best='Борис'...` / `Speaker: 'Борис'` — они звучат в логе
            # независимо от того, что ответил робот, так что «Борис» в
            # must_not_call был бы тавтологически КРАСНЫМ на КАЖДОМ
            # прогоне (см. обсуждение в PR #2789). must_not_say/
            # Борис/Саш(а)/Спартак/пицц — именно то, что утекло в живом
            # прогоне 35734532425 (робот назвал незнакомца Борисом и
            # пересказал его факты).
            must_not_say=["Борис", "Саш", "Спартак", "пицц"],
            why="Сокращено под бюджет VAD 15с (run 35658231116), отказ "
            "называть имя сохранён. Каст #4038 (решение Шифу 26.10): Гриша -- женский голос, реплика в женском "
            "роде («шла»); голоса разведены, шаг НЕ помечается «предел голосов». КОНТРОЛЬНАЯ ГРУППА для акта 3. "
            "Решение Шифу "
            "22.10 (#3858, «режим мастерской» для голоса): незнакомый голос получает "
            "профиль без имени с кличкой -- поэтому запрет `must_not=[register_speaker]` "
            "снят (run 39830739928: он краснел по замыслу сценария, а не по ошибке "
            "робота; профиль заводит владелец биометрии, не LLM-диалог). Остаётся то, "
            "что #3858 оставил в силе (#2779): незнакомец не получает имя/факты "
            "чужого (по голосу) диктора -- ни из бэклога непрошедшей wake-гейт "
            "реплики, ни из истории/памяти. Ожидание под #3858: безымянный профиль "
            "Гриши с кличкой в speakers.e2e.db -- НОРМА (лишней регистрации тут нет); "
            "путаница с Борисом/Сашей -- нет: `must_not_identify_as` читает вердикт "
            "`👤 Speaker: 'Имя'`, а speaker_id_node теперь печатает его ПОСЛЕ решения "
            "об имени и ровно по опубликованному (в полосе сомнения -- `Speaker: "
            "unknown -- имя не подтверждено`, безымянный профиль -- без кавычек).",
        ),
        step(
            "n211_who_do_you_know",
            "Робот, перечисли всех, кого ты сегодня запомнил по голосу.",
            voice=SASHA,
            keywords=["Саш", "Борис"],
            # Issue #2779 AC3 — Гриша НЕ должен попасть в список «кого
            # запомнил»: он отказался называть имя (n210) и не был
            # зарегистрирован. must_not_say (не must_not_call!) — та же
            # причина, что у n210: "Гриш" в имени шага/логах диалога
            # встречается по другим поводам (лейбл сценария, реплика
            # ведущего), а нас интересует только то, что ПРОИЗНЁС робот.
            # Раньше это можно было проверить только глазами; теперь
            # ассертим явно.
            # Issue #2864 — в БД акта ровно два профиля (Саша, Борис), а
            # робот сказал «запомнил троих», посчитав кличку Саши
            # «Незнакомец» отдельным человеком. Настоящего счёта «названо
            # людей ≤ профилей» харнесс не умеет (нужна новая возможность —
            # см. PR), поэтому ловим живую форму ошибки: число «три» и
            # саму кличку-незнакомца. Не «трое»: это подстрока слова
            # «настроение» (сравнение подстрочное, см. e2e_voice_test.sh).
            must_not_say=[
                "Гриш",
                "троих",
                "трёх человек|трех человек|три человека",
                "Незнакомец",
            ],
            why="Оба зарегистрированных должны быть названы. Гриши в списке "
            "быть не должно — раньше это можно было проверить только "
            "глазами, теперь есть явный must_not_say (issue #2779 AC3).",
        ),
        # Issue #2865 — проверка #2793 (memory_search находит факт из
        # voice_facts). Факт о чае должен лежать ТОЛЬКО в voice_facts:
        # «новая сессия» чистит окно ходов AgentCore
        # (dialogue_node._reset_dialogue_session → clear_history), но не
        # трогает voice_memory.db (e2e_mode: /data/voice_memory.e2e.db).
        # Тот же приём, что n1006→n1009 в акте 10. Сброс стоит В КОНЦЕ
        # акта, а не перед n209: иначе n209/n211 (#2864) тоже потеряли бы
        # историю, и поменялось бы то, что они проверяют.
        step(
            "n212_session_reset_before_search",
            "Робот, новая сессия, забудь этот разговор.",
            voice=SASHA,
            patterns=["session reset|session_reset|Новая сессия|new session|tier1 kind=new_session"],
            why="Issue #2865: без сброса факт n203 лежит в окне чата, и LLM "
            "отвечает на n213 из истории, не зовя memory_search (run "
            "35886659057, n208: tools=[]). Паттерн — строка "
            "«🧹 [new-session] session reset» из dialogue_node.reset_session: "
            "если сброс не случился, n213 ничего не доказывает, и этот шаг "
            "обязан покраснеть сам.",
        ),
        step(
            "n213_memory_search_tea_after_reset",
            "Робот, поищи у себя в памяти, что я говорил про чай.",
            voice=SASHA,
            # Тул реально ВЕРНУЛ хиты из БД, а не просто был вызван:
            # MemorySearchTool.execute логирует
            #   [memory_search] query='…' speaker=… → N results (vec=…)
            # N=0 (поиск пустой, как до #2793) этот паттерн не пропускает.
            patterns=[r"\[memory_search\] query=.* [1-9][0-9]* results"],
            expect_tools=["memory_search"],
            # «зелён», а не «чай»: «чай» есть в самом вопросе, и ответ
            # «про чай ничего не нашёл» проходил бы проверку. Цвет чая
            # после сброса сессии робот может взять только из voice_facts.
            # «зелен» — на случай, если LLM/TTS напишет без «ё».
            keywords=["зелён|зелен"],
            why="Issue #2865 / #2793: вопрос задаётся ПОСЛЕ сброса сессии "
            "(n212) — факт о чае есть только в voice_facts. Зелёный = "
            "memory_search вызван, вернул >=1 хит и робот произнёс факт "
            "(«зелёный»), которого нет ни в вопросе, ни в окне чата.",
        ),
    ],
    [
        "Акт 2 — фундамент акта 3. Если register_speaker не вызвался ни разу,",
        "акт 3 бессмысленно даже запускать: атрибуции взяться неоткуда.",
        "Раннер останавливает марафон на красном акте 2 по этой же причине.",
    ],
)

# =============================================================================
# АКТ 3 — «Кто там бубнил на фоне». Бэклог фоновой речи на dialog v2 (В12′, #3767).
#
# Логика цепочки:
#   1) четыре человека говорят БЕЗ wake-слова (expect=backlog) -- v2 копит их в
#      бэклог: `[backlog] accumulated reason=no_wake entries=N`. Имени диктора в
#      этой строке НЕТ (v1 писал speaker='…'): кто говорил, проверяется только по
#      ответу робота, не по строке бэклога;
#   2) голое «Робот.» отдельной фразой ЗАБИРАЕТ свежий бэклог (BACKLOG_PICKUP_S=20 с
#      и примыкающие куски одного рассказа): текст хода берёт код из бэклога, ход
#      идёт в LLM (`📊 [turn] … llm_calls>=1`). Пустой бэклог дал бы рефлекс «слушаю»
#      с llm_calls=0. Забирается ТОЛЬКО то, что моложе 20 с, поэтому «Робот.» идёт
#      сразу за последней фоновой репликой -- без sleep_before_sec;
#   3) вопрос «кто говорил на фоне» -- зонд атрибуции по ответу робота;
#   4) явные команды в фоне, затем голое «Робот.» -- выполняется ПОСЛЕДНЯЯ (LRU);
#   5) музыку, которую он включил, честно выключаем и возвращаем тишину.
# =============================================================================
act(
    3,
    "backlog_diarization",
    "Кто там бубнил на фоне",
    "Четыре голоса говорят БЕЗ wake-слова: Саша и Борис (зарегистрированы в "
    "акте 2), тётя Гриша и курьерша Валера (не представлялись; женские голоса каста #4038). dialog v2 копит их "
    "в бэклог (`[backlog] accumulated reason=no_wake`), а по голому «Робот.» "
    "(В12′, #3767) забирает свежий бэклог в ход. Затем -- вопрос, кого робот "
    "узнал, и LRU-правило: явная команда, произнесённая в фоне без wake-слова, "
    "выполняется по голому «Робот.».",
    "expected-partially-red",
    ["stop_music"],
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
            "Борис, блок питания гудит из-за земли, я вчера полночи его слушал "
            "и знаю эту ноту наизусть.",
            voice=SASHA,
            expect="backlog",
            patterns=[BACKLOG_ACC],
            why="Сокращено под бюджет VAD 15с (run 35658231116, ❌ Речь "
            "отклонена: 17.37с) -- старая реплика тонула ещё в audio_node, до "
            "STT и бэклога. Зарегистрированный голос №1. v2 (#3880): строка бэклога "
            "не называет диктора, паттерн проверяет только факт накопления; "
            "диктор проверяется по ответу робота в n306.",
        ),
        step(
            "n303_bg_boris",
            "Саша, у тебя разводка виновата, надо было звездой разводить, а не "
            "шлейфом через плату.",
            voice=BORIS,
            expect="backlog",
            patterns=[BACKLOG_ACC],
            why="Сокращено под бюджет VAD 15с -- на run 35658231116 именно этот "
            "шаг звучал 17.37с и терялся до STT (см. vad_reject_reason в "
            "e2e_voice_test.sh). Зарегистрированный голос №2.",
        ),
        step(
            "n304_bg_grisha_unknown",
            "Молодые люди, спать собираетесь? Третий час ночи, у меня обход, "
            "свет горит, железка говорит.",
            voice=GRISHA,
            expect="backlog",
            patterns=[BACKLOG_ACC],
            why="Сокращено под бюджет VAD 15с (run 35658231116). "
            "НЕзарегистрированный голос №1 (в v2 строка бэклога не помечает его "
            "«незнакомцем»).",
        ),
        step(
            "n305_bg_valera_unknown",
            "Я курьер, мне бы подпись получить и уехать, стою уже двадцать "
            "минут, слушаю ваш спор про землю.",
            voice=VALERA,
            expect="backlog",
            patterns=[BACKLOG_ACC],
            why="Сокращено под бюджет VAD 15с (run 35658231116) -- соседний "
            "n305 прошёл на 14.81с при лимите 15.0с, то есть был лотереей с "
            "зазором 0.19с. НЕзарегистрированный голос №2. Это ПОСЛЕДНЯЯ "
            "фоновая реплика перед n305b: голое «Робот.» заберёт её (моложе 20 с).",
        ),
        step(
            "n305b_bare_robot_pickup",
            "Робот.",
            voice=SASHA,
            expect="cycle",
            silent_ok=True,
            patterns=[TURN_WITH_LLM],
            why="В12′ (#3767): голое «Робот» ВСЕГДА забирает свежий бэклог, текст хода "
            "-- реплика курьерши из n305. Доказательство подхвата -- ход с LLM "
            "(`llm_calls>=1`): без бэклога голый вейк даёт рефлекс «слушаю» без LLM. "
            "silent_ok: подхваченная болтовня законно может закончиться `passed` без "
            "речи (turn.py: passed из бэклога -- тихо). Без sleep_before_sec: окно "
            "подхвата 20 с, пауза его съест. Строки `[backlog] pickup n=… speaker=…` "
            "(#3767 п.4) в v2-ноде нет -- паттерна на неё нет намеренно.",
        ),
        step(
            "n306_who_was_talking",
            "Робот, скажи, кто сейчас говорил на фоне: кого ты узнал по "
            "голосу, а кого нет?",
            voice=SASHA,
            keywords=["Борис", "незнаком|неопознан"],
            retry=1,
            why="ЯДРО АКТА, зонд атрибуции по ОТВЕТУ робота (не по строке бэклога: "
            "в v2 она диктора не пишет). Ответ должен назвать и знакомого "
            "(Борис), и факт наличия неопознанных (стемы «незнаком|неопознан» ловят "
            "«незнакомец»/«незнакомый»/«неопознанные» -- одна метка неузнанного у кода не одна, разбор 26.10). "
            "Ищется по речи робота. Каст #4038 развёл голоса (Гриша и Валера -- женские): шаг НЕ помечается "
            "«предел голосов». "
            "ВНИМАНИЕ (#3880): метки `Addressed.picked` (кто сказал) в v2 до LLM "
            "не доходят (turn.py их не читает), поэтому шаг может краснеть как "
            "находка, а не как ошибка сценария -- акт expected-partially-red.",
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
            patterns=[BACKLOG_ACC],
            why="ЯВНАЯ КОМАНДА в фоне без wake-слова. В v2 к моменту голого «Робот.» "
            "она старше 20 с и подхвачена не будет -- это нормально, LRU-проверка "
            "ниже про последнюю.",
        ),
        step(
            "n309_bg_command_valera",
            "Только не электронщину, поставь что-нибудь спокойное, мне ещё ехать.",
            voice=VALERA,
            expect="backlog",
            patterns=[BACKLOG_ACC],
            why="Вторая команда в бэклоге. Выполниться должна ИМЕННО ЭТА "
            "(последняя, моложе 20 с), а не Борисова.",
        ),
        step(
            "n310_execute_backlog_lru",
            "Робот.",
            voice=SASHA,
            expect="cycle",
            silent_ok=True,
            patterns=[TURN_WITH_LLM, MUSIC_DISPATCH, tool_done(*MUSIC_START_TOOLS)],
            must_not=["stop_music", TURN_TIMEOUT],
            must_not_say=[MUSIC_NOT_STARTED],
            why="Во фразе нет своей команды (голое «Робот.», В12′) — робот обязан "
            "выполнить последнюю явную команду из бэклога (LRU): ход с LLM и вызов "
            "тула музыки (`📤 Отправлен запрос <id>: <tool>`). Конкретный тул не "
            "фиксируем: «спокойное» законно ложится и в renardo, и в библиотеку, и в "
            "dj_set. Фраза потолка «Музыка пока не заиграла» здесь -- ложь (музыка обязана заиграть, #3971 R22). "
            "silent_ok: речь после запуска музыки не обязательна. v1-паттерн "
            "`[backlog] flushed to LLM` убран (в v2 его нет). Без retry: подхват бэклога "
            "-- разовое событие, на повторе бэклог уже пуст (голый вейк = рефлекс).",
        ),
        step(
            "n311_who_asked",
            "Робот, а кто именно просил включить музыку?",
            voice=SASHA,
            why="Зонд атрибуции команды к говорящему. Правильный ответ — что "
            "просил Борис, а уточнял незнакомый голос. Без ассерта: LLM здесь "
            "часто отвечает по существу, но без имён.",
        ),
        action_step(
            "n312_stop_music",
            "Робот, всё, спасибо, выключай музыку, курьерша уехала.",
            ["stop_music"],
            music="stopped",
            voice=SASHA,
            expect_tools=["stop_music"],
            why="Закрываем то, что открыли в n310. Никаких «выключи» вслепую: "
            "музыка ТОЧНО играет, потому что n310 её запустил.",
        ),
        step(
            "n313_silence_restored",
            "Робот, теперь тихо?",
            voice=SASHA,
            keywords=[SILENCE_ANSWER],
            must_not=["stop_music"],
            must_not_say=[FALSE_FAILURE_PHRASE],
            why="Возврат к якорю тишины из акта 1 — акт не оставляет хвостов. "
            "Issue #3165: ответ — из <music_state> (снимок плеера, #3161), "
            "get_music_state больше не обязателен (живой прогон 29.09: "
            "обязательный тул наказывал верное «тишина» ретраями и фразой "
            "«Не получилось выполнить»). Проверяем сам ответ.",
        ),
    ],
    [
        "Акт 3 помечен expected-partially-red СОЗНАТЕЛЬНО.",
        "Диаризация (#1077) в voice_core_suite_v1 значится out-of-scope,",
        "а атрибуция фоновых реплик по говорящему живьём не проверялась ни разу:",
        "строка бэклога dialog v2 диктора не пишет, метки picked до LLM не доходят.",
        "Голоса каста разведены (#4038: Гриша и Валера -- женские), поэтому красный",
        "n304-n311 -- не «предел голосов», а находка по роботу.",
        "GATE-1 гейтит только то, что обязано работать при любой погоде:",
        "stop_music (мы сами включили музыку в n310).",
        "Атрибуция проверяется per-step по ответу робота (n306) — её падение",
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
            discovery_tools=["list_tts_voices"],
            why="issue #2406: discovery_tools — list_tts_voices должен быть "
            "вызван ДО голосового ответа (regression guard, run 365 / n401 "
            "из issue #2406).",
        ),
        step(
            "n402_current_voice",
            "Робот, а каким голосом ты говоришь прямо сейчас?",
            patterns=[spoken_by("minimax")],
            keywords=["minimax|минимакс|мини макс|мини-макс|qingse|цинсе"],
            why="R6 разбора 26.10: прежний `SpeechEvent synthesized … provider=<любой>` совпадал с каждым ходом. "
            "Результат теперь: ответ ЗВУЧИТ голосом по умолчанию (MiniMax, `tts_node.yaml` minimax_voice "
            "male-qn-qingse; харнесс возвращает голос и параметры до акта) и НАЗЫВАЕТ его -- провайдер или голос "
            "(23.10: «Сейчас я на MiniMax, и голос у меня male-qn-qingse»). Если робот ушёл в фолбек Yandex, шаг "
            "красный -- это и есть находка (голосовой стек деградирует молча).",
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
            patterns=[spoken_by("yandex")],
            why="R6: результат -- реплика синтезирована ГОЛОСОМ ИЗ n403. Алёна есть только у Yandex "
            "(`tts_voice_registry`: yandex:alena), голос робота по умолчанию -- MiniMax, значит «этим голосом» = "
            "`SpeechEvent synthesized … provider=yandex`. Не сменил голос в n403 -- здесь тоже честно красный.",
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
            "Робот, говори погромче, тётя Гриша глуховата и стоит в коридоре.",
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
            patterns=[prosody_changed(SPEED_PARAMS, BELOW_NORMAL)],
            why="Эффект, а не вызов (разбор 26.10: ложный PASS -- set_speed писал yandex_speed, а звучал MiniMax). "
            "#4039: тул меняет ось АКТИВНОГО провайдера, нода пишет `🎚️ minimax_speed изменён: 0.8` -- скорость "
            "стала ниже нормы (харнесс вернул ось к норме до акта).",
        ),
        step(
            "n410_speed_up",
            "Робот, ладно, хватит тянуть, теперь наоборот — тараторь.",
            patterns=[prosody_changed(SPEED_PARAMS, ABOVE_NORMAL)],
            why="«Тараторь» -- быстрее НОРМЫ: после n409 (0.8) один шаг возвращает лишь 1.0 (обычная речь), это не "
            "эффект просьбы. 23.10 LLM звала set_speed дважды параллельно, оба прочитали 0.8 и записали 1.0 -- "
            "такой ход здесь красный по делу.",
        ),
        step(
            "n411_pitch_bass",
            "Робот, говори басом, как диктор в старом кино.",
            patterns=[prosody_changed((PITCH_PARAM,), BELOW_NORMAL)],
            why="v2 (#3880, run 39878786968): шаг проходил OK при ответе «таких "
            "инструментов нет». #4039: тон слышен у MiniMax (`voice_setting.pitch`) без chipmunk_mode; эффект -- "
            "`🎚️ pitch_shift изменён: 0.8` (ниже нормы). У провайдера без тона тул честно отказывает -- шаг "
            "красный, акт flaky-known.",
        ),
        step(
            "n411b_prosody_normal",
            "Робот, всё, верни обычную скорость речи и обычный тон.",
            patterns=[
                prosody_changed(SPEED_PARAMS, NORMAL) + "|" + SPEED_ALREADY_NORMAL,
                prosody_changed((PITCH_PARAM,), NORMAL) + "|" + PITCH_ALREADY_NORMAL,
            ],
            why="Сброс просодии перед сказкой (акт возвращает робота в исходное). Отдельного маркера сброса пока "
            "нет -- проверяем значения осей: скорость и тон снова 1 (`изменён: 1`) либо тул прочитал, что ось "
            "уже на норме (`Скорость уже нормальная` / `Голос уже нормальный`: честное «уже», не каскад от n410/n411).",
        ),
        step(
            "n412_skazka_multivoice",
            "Робот, расскажи короткую сказку про Красную Шапочку, говори за "
            "каждого персонажа своим голосом.",
            patterns=[perform_started("tale"), PERFORM_DONE],
            keywords=["красн"],
            why="#3971 R4 (23.10 22:04:48): v1-паттерн `actions=[…perform…]` не совпадал с логом v2 (command_bridge "
            "пишет actions=[]), верная сказка давала PATTERN_MISS, ретрай заказывал вторую (2.5 мин). Теперь "
            "результат по событиям владельца: `выступление старт kind=tale` И конец `perform-: конец (done)` / "
            "`PlanEvent block_end … reason=done`. Ретрая НЕТ: после старта выступления повтор заказал бы второе. "
            "Конец ждём по событиям: сегменты ставятся в очередь сразу (`SpeechEvent queued`), харнесс ждёт, пока "
            "все закроются, -- это и есть block_end; но его потолок -- E2E_REACTION_WINDOW×E2E_MAX_ATTEMPTS "
            "(≈2 мин), поэтому просим КОРОТКУЮ сказку (23.10: 917 символов -- 99 с, 1519 -- 160 с не влезли). "
            "#1532: сказка полная, голоса персонажей -- `voice_per_segment` (без него код -- нарушение, #3966). "
            "Стем «красн» ловит «красную»/«красная» и запрещает generic «жила-была» без героя.",
        ),
    ],
    [
        "Акт 4 — flaky-known: mv-шаги зависят от tool-choice LLM и состава",
        "голосов провайдера, ломаются при каждой смене провайдера.",
        "GATE-1 держит минимум: список голосов, факт смены, громкость, скорость.",
        "set_pitch ВНЕ gate — тона нет у части TTS-провайдеров (#4039: честный отказ).",
        "Скорость и тон шаги судят по эффекту (`<ось> изменён: <значение>` в tts_node), а не по вызову тула.",
    ],
)

# =============================================================================
# АКТ 5 — «Живой музыкант». Полный жизненный цикл renardo.
# Цепочка: тишина → старт → проверка что играет → развитие → вайб →
# посторонний вопрос ПОД музыкой (музыка не должна глохнуть) → стоп →
# проверка тишины.
# Ни одного «выключи» без предшествующего «включи».
# =============================================================================
act(
    5,
    "renardo_live",
    "Живой музыкант",
    "Полный жизненный цикл живой музыки на renardo, без единого «выключи» "
    "вслепую: тишина → старт → подтверждение что играет → развитие темы → "
    "смена вайба → посторонний вопрос ПОД музыкой (музыка обязана продолжать "
    "играть) → stop → подтверждение тишины.",
    "new-unproven",
    [
        "request_music",
        "stop_music",
    ],
    [
        step(
            "n501_silence_check",
            "Робот, сейчас же ничего не играет, верно?",
            expect="cycle",
            keywords=[SILENCE_ANSWER],
            must_not=["stop_music"],
            must_not_say=[FALSE_FAILURE_PHRASE],
            why="Якорь тишины перед стартом. Именно этого шага не хватало в "
            "старых сценариях, где «останови музыку» шло первым. #3971: ответ -- из "
            "снимка `<music_state>` (ADR-0148), вызов get_music_state не требуем; "
            "проверяем фразу «тишина» (run 161509: робот пересказал сказку вместо ответа).",
        ),
        action_step(
            "n502_start_renardo",
            "Робот, сыграй на рендардо спокойный ночной бит, что-нибудь под пайку.",
            ["request_music"],
            music="started",
            expect_tools=["request_music"],
            must_not=["generate_music"],
            retry=2,
            why="Живой синтез, НЕ нейрогенерация: generate_music здесь — "
            "выбор не того тула (коллизия имён, PR #1372). #3971: PASS только при "
            "`Инструмент request_music выполнен успешно` + `[music v2] started` (run 132908: "
            "трек из кэша + «не понял» давал ложный PASS).",
        ),
        step(
            "n503_confirm_playing",
            "Робот, что сейчас звучит?",
            keywords=[PLAYING_ANSWER],
            must_not=["stop_music"],
            must_not_say=[SILENCE_CLAIM, FALSE_FAILURE_PHRASE],
            why="Подтверждаем, что n502 реально завёл плеер, а не просто "
            "ответил «включаю» голосом. #3971: ответ из снимка `<music_state>` (ADR-0148), "
            "get_music_state не требуем; «тишина» при играющем плеере -- красный.",
        ),
        action_step(
            "n504_add_bass",
            "Робот, продолжай развивать эту тему и добавь баса пожирнее.",
            list(MUSIC_EDIT_TOOLS),
            music="changed",
            must_not=["stop_music"],
            why="Эволюция трека без перезапуска: stop_music здесь = робот "
            "глушит и начинает заново вместо развития. #3971: в v2 тулов живой правки Renardo "
            "нет -- честная попытка это dj_set|request_music|generate_music (run 132908: робот "
            "звал dj_set, харнесс ждал request_music); успех тула обязателен.",
        ),
        action_step(
            "n505_vibe_dark",
            "Робот, сделай атмосферу помрачнее, как в киберпанке.",
            list(MUSIC_EDIT_TOOLS),
            music="changed",
            must_not=["stop_music"],
            why="#3971: см. n504 -- любой из dj_set|request_music|generate_music, но с успехом тула.",
        ),
        action_step(
            "n506_faster",
            "Робот, и ускорь темп, а то мы тут засыпаем оба.",
            list(MUSIC_EDIT_TOOLS),
            music="changed",
            must_not=["stop_music", "set_speed", "set_pitch"],
            why="#3976: «ускорь темп» под музыкой -- про музыку, а не про голос робота; set_speed/set_pitch "
            "(стем «темп» открывает группу voice) -- реальный баг. Шаг остаётся красным, пока его не починят: "
            "запрет и обязательный музыкальный тул держат это честно.",
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
        action_step(
            "n511_stop_music",
            "Робот, всё, глуши музыку, у меня уши в трубочку.",
            ["stop_music"],
            music="stopped",
            expect_tools=["stop_music"],
            why="#3971: стоп засчитывается по `Инструмент stop_music выполнен успешно` + "
            "`[music v2] stop`, а не по слову «выключаю».",
        ),
        step(
            "n512_silence_after_stop",
            "Робот, теперь тишина?",
            keywords=[SILENCE_ANSWER],
            must_not=["stop_music"],
            must_not_say=[FALSE_FAILURE_PHRASE],
            why="#3971: ответ из снимка `<music_state>` (ADR-0148), get_music_state не требуем.",
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
    ["dj_set"],
    [
        step(
            "n601_silence_check",
            "Робот, сейчас тихо, ничего не играет?",
            expect="cycle",
            keywords=[SILENCE_ANSWER],
            must_not=["stop_music"],
            must_not_say=[FALSE_FAILURE_PHRASE],
            why="#3971: ответ из снимка `<music_state>` (ADR-0148), get_music_state не требуем.",
        ),
        action_step(
            "n602_dj_on",
            "Робот, вставай за пульт, включай диджей-режим, у нас тут ночная дискотека.",
            ["dj_set"],
            music="started",
            expect_tools=["dj_set"],
            retry=1,
            why="#3971: v2-тул -- dj_set (action=start), прежний v1-тул режима диджея удалён. PASS только при "
            "`Инструмент dj_set выполнен успешно` + `[music v2] started`.",
        ),
        step(
            "n603_confirm_dj_playing",
            "Робот, что сейчас крутится?",
            keywords=[PLAYING_ANSWER],
            must_not=["stop_music"],
            must_not_say=[SILENCE_CLAIM, FALSE_FAILURE_PHRASE],
            why="#3971: ответ из снимка `<music_state>` (ADR-0148); «тишина» при играющем сете -- красный.",
        ),
        action_step(
            "n604_dj_rock",
            "Робот, уводи в сторону рока, тётя Гриша под рок добреет.",
            ["dj_set", "request_music"],
            music="changed",
            must_not=["stop_music"],
            why="#3971: смена стиля сета -- `dj_set` (start с новой темой) или request_music; успех тула обязателен.",
        ),
        action_step(
            "n605_dj_set_plan",
            "Робот, поставь сет из трёх треков подряд и объявляй каждый перед началом.",
            ["dj_set", "request_music"],
            patterns=[SPEECH_STARTED],
            why="Зонд gp01: спека §4.1 обещает «сет заканчивается финальным "
            "треком и прощанием», но тула save_dj_set_plan в коде НЕТ — сет "
            "глохнет по счётчику DJ_AUTO_MAX_TRANSITIONS=24. Падение "
            "подтверждает карточку W7-1. #3971: просьба -- и действие, и объявление треков вслух: "
            "нужны оба -- `SpeechEvent started` (#3880) И успех тула; речь без результата не PASS. "
            "Разбор 26.10 (n605: прогресс-фраза засчитана объявлением) -- шаг НАМЕРЕННО не ужесточён, пока не влит "
            "#4006 (диджей молчит между треками): ждать перехода и `dj_line` -- после него.",
        ),
        step(
            "n606_wake_under_dj",
            "Робот, а сколько у тебя заряда осталось?",
            voice=BORIS,
            expect_tools=["get_battery_level"],
            must_not=["stop_music", "dj_set"],
            why="Вопрос под играющим сетом чужим (зарегистрированным) голосом: "
            "ответить и не сбить сет.",
        ),
        action_step(
            "n607_vibe_chill",
            "Робот, а теперь помягче, поставь спокойную атмосферу, Валера уснёт за рулём.",
            ["dj_set", "request_music"],
            music="changed",
            must_not=["stop_music"],
            why="#3971: см. n604.",
        ),
        action_step(
            "n608_dj_off",
            "Робот, хватит диджеить, выключай.",
            ["dj_set", "stop_music"],
            music="stopped",
            retry=1,
            why="«Хватит диджеить» — двусмысленно (`dj_set` action=stop или `stop_music`). #3971: тул не "
            "фиксируем (v2 робот гасит сет через dj_set), но звук обязан прекратиться: успех любого из "
            "двух тулов + `[music v2] stop`.",
        ),
        step(
            "n609_silence_after_dj",
            "Робот, тишина настала?",
            keywords=[SILENCE_ANSWER],
            must_not=["dj_set"],
            must_not_say=[FALSE_FAILURE_PHRASE],
            why="#3971: ответ из снимка `<music_state>` (ADR-0148), get_music_state не требуем.",
        ),
        step(
            "n610_dj_state_honest",
            "Робот, диджей-режим у тебя сейчас включён или выключен?",
            why="Зонд честности состояния: расходится ли ответ робота с тем, "
            "что показал снимок `<music_state>` в n609. Без ассерта — сравниваем "
            "transcript n609 и n610 глазами.",
        ),
    ],
    [
        "Акт 6 — DJ. GATE-1 держит вызов dj_set (вход; робот v2 гасит сет тем же dj_set",
        "action=stop, поэтому stop_music в gate акта не ставим -- выход держит n608 по",
        "маркеру `[music v2] stop`). Состояние музыки берётся из снимка `<music_state>`,",
        "get_music_state не требуется (#3971). Сам «сет с объявлениями»",
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
        "music_library",
        "stop_music",
    ],
    [
        step(
            "n701_silence_check",
            "Робот, тихо у нас сейчас?",
            expect="cycle",
            keywords=[SILENCE_ANSWER],
            must_not=["stop_music"],
            must_not_say=[FALSE_FAILURE_PHRASE],
            why="#3971: ответ из снимка `<music_state>` (ADR-0148), get_music_state не требуем.",
        ),
        action_step(
            "n702_generate_raccoon",
            "Робот, сочини и спой короткую песню про енота, который по ночам "
            "чинит роботов в мастерской.",
            ["generate_music"],
            expect_tools=["generate_music"],
            retry=1,
            known_red="только при квоте KIE (`[cover] баланс KIE: 0`): при `<providers>` music_gen health != ok "
            "(run 132908: health=quota весь марафон) честный ответ робота -- `perform song`, а не generate_music. "
            "Харнесс условий по снимку не умеет (дамп LLM-запроса с `<providers>` вырезается из лога шага), "
            "поэтому под квотой шаг красный по делу. 26.10 баланс есть (1712) -- шаг обязан быть зелёным.",
            why="Разводка коллизии имён (PR #1372): «спой песню» — это "
            "нейрогенерация, а не renardo-код. #3971: PASS только при `Инструмент generate_music "
            "выполнен успешно`. Под квотой KIE шаг ожидаемо красный (см. _known_red).",
        ),
        action_step(
            "n703_merge_komar",
            "Робот, и ещё куплет добавь, про комара, который еноту мешал.",
            must_not=["STOP command received", "Cancel: new STT input"],
            why="Issue #968 MERGE: добавка к уже поющейся песне не должна "
            "обрывать текущее исполнение. must_not_call используется здесь "
            "как «этих строк не должно быть в логах шага». #3971: ход обязан дойти до "
            "`stage=done status=finished` без `stage=timeout` (run 132908: ход убит на 12 с, "
            "робот сказал «долго думаю» -- ложный PASS).",
        ),
        step(
            "n704_music_library",
            "Робот, что у тебя вообще лежит в музыкальной библиотеке?",
            expect_tools=["music_library"],
        ),
        step(
            "n705_gen_search_rain",
            "Робот, найди у себя в библиотеке что-нибудь про дождь.",
            expect_tools=["music_library"],
        ),
        step(
            "n706_gen_save_for_boris",
            "Робот, сохрани эту песню про енота для Бориса, он такое любит.",
            expect_tools=["music_library"],
            skip_when="KIE_QUOTA",
            why="n706-n710 -- жизнь ПЕСНИ ЭТОГО ПРОГОНА (n702). R21 разбора 26.10: при `[cover] баланс KIE: 0` "
            "песни нет, шаги краснеют каскадом, а delete бьёт по чужому треку -- их место SKIP KIE_QUOTA "
            "(`_skip_when`; харнесс поле пока не читает -- см. шапку генератора).",
        ),
        step(
            "n707_gen_track_info",
            "Робот, расскажи, что это за трек: кто, про что и сколько длится.",
            expect_tools=["music_library"],
            skip_when="KIE_QUOTA",
        ),
        action_step(
            "n708_gen_play",
            "Робот, включи эту песню про енота с начала.",
            ["music_library"],
            music="started",
            expect_tools=["music_library"],
            retry=1,
            skip_when="KIE_QUOTA",
            why="#3971: воспроизведение засчитывается по `Инструмент music_library выполнен успешно` + "
            "`[music v2] started`.",
        ),
        action_step(
            "n709_stop_generated",
            "Робот, останови, Борис уже уснул под неё.",
            ["stop_music"],
            music="stopped",
            expect_tools=["stop_music"],
            skip_when="KIE_QUOTA",
            why="Опять же: останавливаем ровно то, что сами включили в n708. #3971: по маркеру "
            "`[music v2] stop` с id трека (не `track_id=None`), а не по слову «остановил».",
        ),
        action_step(
            "n710_gen_delete",
            "Робот, удали из библиотеки эту песню про енота, она всё-таки так себе.",
            ["music_library"],
            patterns=[r"music_library с параметрами \{.*'action': 'delete'"],
            expect_tools=["music_library"],
            skip_when="KIE_QUOTA",
            why="Уборка за собой. R21 (23.10 22:39:22): «удали ПОСЛЕДНИЙ трек» без песни этого прогона удалило "
            "чужой трек и осталось зелёным. Теперь просьба называет песню этого прогона (про енота): id робот "
            "берёт из поиска/списка, без такой песни удалять нечего. Результат -- запрос `delete` и успех "
            "music_library. Под квотой KIE шаг не играется (`_skip_when`), destructive delete вслепую не гоняем.",
        ),
        action_step(
            "n711_renardo_not_broken",
            "Робот, а живьём на рендардо сыграть ты всё ещё умеешь? Сыграй короткий бит.",
            ["request_music"],
            music="started",
            expect_tools=["request_music"],
            must_not=["generate_music"],
            retry=1,
            why="Контроль отсутствия регрессии коллизии имён: после серии "
            "gen_* робот обязан вернуться к живому синтезу. #3971: `Инструмент request_music "
            "выполнен успешно` + `[music v2] started` (run 132908: трек из кэша без Renardo давал ложный "
            "PASS; кэш от Renardo по логу `started` не отличить -- это известный предел шага).",
        ),
        action_step(
            "n712_stop_all",
            "Робот, всё, глуши.",
            ["stop_music"],
            music="stopped",
            expect_tools=["stop_music"],
        ),
    ],
    [
        "Акт 7 — flaky-known: MiniMax Music API падает независимо от нас.",
        "Квота KIE (`[cover] баланс KIE: 0`): n706-n710 -- SKIP KIE_QUOTA (`_skip_when`, R21),",
        "delete без песни этого прогона не гонять. 26.10 баланс 1712 -- правило стоит на будущее.",
        "generate_music и music_library ВНЕ GATE-1 намеренно —",
        "их отсутствие чаще всего означает недоступность внешнего API.",
        "В gate только каталожные gen_*-тулы (работают локально) и stop_music.",
    ],
)

# =============================================================================
# АКТ 8 — «Перебей меня». Barge-in, STOP, MERGE.
# ADR-0150 PR-5 (#3694): политика перебивания одна, параметра нет — речь рвёт
# только ясная стоп-команда (/dialog/interrupt_request → «ясный стоп»), новая
# тема или добавка без стопа речь не обрывает (ответ — после текущей реплики).
#
# Ожидания на dialog v2 (#3880): тула `speak_text` нет -- речь это `command.speech`,
# её факт -- `SpeechEvent started`; «стоп» -- Tier-1 без речи (`llm_calls=0`), речь
# гасится (`SpeechEvent cancelled` / `🛑 [interrupt]`), ход кончается молча
# (silent_ok). Ожидания v1 не сохранены.
# =============================================================================
act(
    8,
    "barge_in",
    "Перебей меня, если сможешь",
    "Barge-in, STOP и MERGE — только в логических парах «сначала запусти "
    "длинную речь, потом перебей». Политика одна (ADR-0150 PR-5): речь рвёт "
    "только ясная стоп-команда, новая тема без стопа встаёт после текущей речи.",
    "expected-partially-red",
    [SPEECH_STARTED, "request_music", "stop_music"],
    [
        step(
            "n801_wake_open",
            "Робот, сейчас будем тебя перебивать, не обижайся.",
            expect="cycle",
        ),
        step(
            "n802_long_count",
            "Робот, посчитай вслух от одного до сорока, не торопясь и с выражением.",
            patterns=[SPEECH_STARTED],
            why="Заряжаем длинный ответ, который есть что перебивать. Без "
            "этого шага n803 проверял бы перебивание тишины. v2 (#3880): вместо "
            "тула `speak_text` -- `SpeechEvent started` (конца речи не ждём: "
            "следующий шаг перебивает, `finished` в окно шага не попадёт).",
        ),
        step(
            "n803_barge_in_new_topic",
            "Робот, стоп, лучше скажи, сколько будет семью восемь.",
            barge_in=BARGE_3S,
            patterns=[SPEECH_CANCELLED],
            why="Стоп + новая тема (#1280, В19 «всё, хватит, спой другую»): «стоп» первым "
            "словом — ясная стоп-команда, старый ответ рвётся, новый — звучит.",
        ),
        step(
            "n804_long_story",
            "Робот, расскажи историю о том, как ты первый раз в жизни увидел "
            "кота и что подумал.",
            patterns=[SPEECH_STARTED],
            must_not_say=[PERFORM_ASKS_TO_REPLACE],
            why="Сокращено, чтобы уйти от края окна VAD (было 100 символов, "
            "запас 1.2с, run 35658231116) — реплика лишь просит длинную "
            "историю, длина сгенерированного ответа не зависит от неё. v2: "
            "`SpeechEvent started` вместо `speak_text` (см. n802). #3971 R22: вопрос выступления «выключить и "
            "зачитать?» (музыка, оставленная прошлым актом) -- тоже `SpeechEvent started`, но не история.",
        ),
        step(
            "n805_hard_stop",
            "Робот, всё, хватит, замолчи.",
            barge_in=BARGE_3S,
            silent_ok=True,
            patterns=[SPEECH_CANCELLED],
            why="Чистый STOP без новой темы — другой класс перебивания, чем n803. "
            "v2 (#3880, run 39876993047): Tier-1 стоп отвечает молча, речь гасится "
            "(`SpeechEvent cancelled reason=stopped` ~170 мс от партиала) -- "
            "харнесс ждал TTS и краснел `cycle_unconfirmed_after_accept`. Теперь: "
            "`silent_ok` (ход без речи допустим) + ОБЯЗАТЕЛЬНЫЙ паттерн отмены "
            "речи: молчание без отмены -- по-прежнему красный.",
        ),
        step(
            "n805b_unsilence",
            "Робот, говори.",
            silent_ok=True,
            patterns=["tier1 kind=unsilence"],
            why="Решение Шифу 23.10: «всё, хватит, замолчи» (n805, `tier1 kind=silence`) включает тишину "
            "(`SILENCE_TTL_S` = 600 с), и снимает её только явное «говори» (`UNSILENCE_WORDS`) -- вейк-слово "
            "тишину НЕ снимает. Без этого шага n806 «посчитай» уходит в молчащего робота (run 132908: робот "
            "считал в тишине без тулов). Снятие -- Tier-1 без речи (`tier1_phrase` для unsilence пуст), "
            "поэтому `silent_ok` + обязательный маркер `tier1 kind=unsilence`; что робот снова отвечает, "
            "доказывает следующий шаг (n806: `SpeechEvent started`).",
        ),
        step(
            "n806_long_count_again",
            "Робот, посчитай вслух от одного до двадцати, спокойно.",
            patterns=[SPEECH_STARTED],
            must_not_say=[PERFORM_ASKS_TO_REPLACE],
            why="v2: `SpeechEvent started` вместо `speak_text` (см. n802); вопрос выступления -- не счёт (см. n804).",
        ),
        step(
            "n807_merge_addition",
            "Робот, и после каждого числа добавляй слово шаг.",
            barge_in=BARGE_3S,
            silent_ok=True,
            patterns=[TURN_DONE],
            must_not=["🛑 [interrupt]", "SpeechEvent cancelled"],
            why="Зонд gp12: MERGE вне песни. Добавка к ИДУЩЕЙ задаче не должна "
            "её обрывать (ADR-0150 PR-5: не стоп — речь не рвётся, interrupt.py "
            "NO_INTERRUPT; «Cancel: new STT input» здесь законен — отменяется ход LLM, "
            "не речь). v2 (#3880): `task_delta|speak_text` убран -- в v2 нет ни "
            "того, ни другого; вместо него ход дошёл до `stage=done status=finished`. "
            "Исход -- добавка отвечает после текущей речи (APPEND) либо молча "
            "(`silent_ok`): в run 39876993047 шаг краснел `cycle_unconfirmed_after_"
            "accept` при искажённом STT («пять робот и после каждого числа…»). "
            "Это НЕ стоп-шаг: паттерн отмены речи здесь не ставим, наоборот.",
        ),
        step(
            "n808_repeat_last",
            "Робот, повтори последнее ещё раз, я не расслышал.",
            why="Зонд: работает ли «повтори» как отдельная команда или "
            "пересказывается заново другими словами.",
        ),
        action_step(
            "n809_music_bed",
            "Робот, поставь фоном ровный ритм на рендардо, только негромко.",
            ["request_music"],
            music="started",
            expect_tools=["request_music"],
            must_not=["generate_music"],
            silent_ok=True,
            retry=1,
            why="Готовим фон для проверки «речь поверх музыки». v2 (#3880, run "
            "39876993047): запрос музыки может закончиться без речи -- харнесс ждал "
            "TTS (reply_after=41s ×3, `cycle_unconfirmed_after_accept`). `silent_ok` "
            "допускает ход без речи, а факт действия держит `expect_tools=request_music` "
            "(`Запрос выполнения: request_music` в mcp_server) -- шаг по-прежнему "
            "краснеет, если музыку не запросили. #3971: «поставлю ритм» без тула больше не PASS -- "
            "нужны `Инструмент request_music выполнен успешно` + `[music v2] started`.",
        ),
        step(
            "n810_rap_over_music",
            "Робот, а теперь под этот бит зачитай короткий рэп про енотика.",
            patterns=[SPEECH_STARTED],
            must_not=["stop_music"],
            must_not_say=[PERFORM_ASKS_TO_REPLACE],
            why="Речь ПОВЕРХ музыки: робот не должен глушить бит, чтобы "
            "заговорить (issue #986 — соотношение музыки и речи мастер-гейном). "
            "v2: `SpeechEvent started` вместо `speak_text` (см. n802). #3971 R22 (робот 23.10 22:53:09): на «под "
            "этот бит зачитай» робот спросил «Сейчас играет …, выключить и зачитать?» -- харнесс засчитал вопрос "
            "речью. Просьба уже назвала бит, вопрос -- не рэп: шаг красный.",
        ),
        step(
            "n811_stop_speech_keep_music",
            "Робот, хватит читать, а музыку оставь.",
            barge_in=BARGE_3S,
            silent_ok=True,
            patterns=[SPEECH_CANCELLED, TIER1_STOP_SPEECH],
            must_not=["stop_music", MUSIC_STOP_ANY],
            why="Разделение каналов речь/музыка. #4035 (охват стопа): «хватит читать, а музыку оставь» -- Tier-1 "
            "`stop_speech`: речь и выступление гаснут (`silent_ok` + паттерн отмены, как n805), плеер играет -- "
            "ни тула stop_music (в т.ч. outro бита), ни `[music v2] stop` в окне шага. До #4035 Tier-1 `stop_all` "
            "гасил и музыку (разбор 26.10, #3994).",
        ),
        action_step(
            "n812_stop_music",
            "Робот, теперь и музыку выключай.",
            ["stop_music"],
            music="stopped",
            patterns=[MUSIC_WAS_PLAYING],
            expect_tools=["stop_music"],
            why="Разбор 26.10 (22:55:01): стоп вхолостую `track_id=None` + `was_playing=False` давал PASS и фразу "
            "«Выключил музыку.». Теперь результат -- тул застал музыку играющей (`was_playing=True`) и дека снята "
            "с настоящим id трека. Музыку погасил n811 -- здесь честно красный (каскад от нарушения «оставь»).",
        ),
        step(
            "n813_silence_final",
            "Робот, тишина?",
            keywords=[SILENCE_ANSWER],
            must_not_say=[FALSE_FAILURE_PHRASE],
            why="#3971: ответ из снимка `<music_state>` (ADR-0148), get_music_state не требуем.",
        ),
    ],
    [
        "Акт 8 — expected-partially-red: правки идущего выступления (MERGE в PENDING)",
        "нет до ADR-0150 PR-7c; планировщик PR-7b ставит новую реплику после текущей.",
        "GATE-1 держит только то, что обязано работать: речь (`SpeechEvent started`,",
        "в dialog v2 тула speak_text нет), музыка, остановка музыки.",
        "Параметра политики перебивания нет (ADR-0150 PR-5) — ничего выставлять не нужно.",
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
        action_step(
            "n902_sound_doorbell",
            "Робот, включи звук дверного звонка, тётя Гриша опять идёт с обходом.",
            ["play_sound"],
            silent_ok=True,
            why="Решение Шифу 26.10: тихое действие по прямой просьбе -- норма (код: SILENT_DONE_TOOLS). Раньше шаг "
            "без silent_ok краснел `nospeech` и до acceptance не доходил (act9 23.10). Результат -- "
            "`✅ Инструмент play_sound выполнен успешно`.",
        ),
        action_step(
            "n903_sound_siren_anim",
            "Робот, а теперь сирену и мигалку покажи, напугаем Валеру.",
            ["play_sound"],
            patterns=[tool_done("play_animation")],
            silent_ok=True,
            why="Просьба -- и сирена (звук), и мигалка (анимация police_lights): результат обоих тулов (23.10 робот "
            "сделал оба тихо). Тихо -- норма (решение Шифу 26.10); «Виу-виу!» (#4031) допустимо, но не обязательно.",
        ),
        step(
            "n904_anim_sad",
            "Робот, покажи, как тебе грустно, что смена такая длинная.",
            patterns=[face_or_animation("sad"), TURN_DONE],
            silent_ok=True,
            why="v2 показывает эмоцию лицом команды (`command.expression`), тул play_animation не обязателен "
            "(23.10: expression sad + речь, сценарий требовал v1-тул). Результат -- `🎭 [turn-v2] лицо команды на "
            "done: emotion=sad` (#4041) либо play_animation sad. Показать молча -- законно (просьба «покажи»).",
        ),
        step(
            "n905_anim_victory",
            "Робот, а теперь победную анимацию — блок питания наконец не гудит!",
            patterns=[face_or_animation("victory", "happy"), TURN_DONE],
            silent_ok=True,
            why="См. n904: победа -- лицо команды emotion=victory|happy или play_animation victory|happy.",
        ),
        step(
            "n906_anekdot_anim",
            "Робот, расскажи анекдот и покажи под него весёлую анимацию.",
            patterns=[face_or_animation("happy", "victory"), SPEECH_STARTED],
            why="Комбинация двух каналов в одном ходе: речь + LED. v2 (#3880): вместо тула "
            "`speak_text` -- `SpeechEvent started` (см. n802). Весёлое -- лицом команды или play_animation "
            "(голое имя тула в паттерне совпадало и со строкой действий без результата).",
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
            patterns=[SPEECH_STARTED],
            why="v2 (#3880): `SpeechEvent started` вместо `speak_text`. "
            "Зонд gp07: спека §3.2 обещает ответ на выбранном языке, но "
            "tts_text_guard режет чанк с >10% «чужой» письменности (#1709). "
            "Красный подтверждает карточку W6-4/W6-5.",
        ),
        step(
            "n910_estimate_duration",
            "Робот, прикинь, сколько секунд ты будешь читать вслух четверостишие?",
            patterns=["estimate_tts_duration|" + SPEECH_STARTED],
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
    ["memory_search", SPEECH_STARTED],
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
            keywords=["незнаком|не назвал|не представил|неопознан|без имён|без имен|не знаю"],
            why="Честность про неопознанных. Робот не должен задним числом "
            "выдумывать им имена. Разбор 26.10 (R20, робот 23:12:16): честный ответ «не назвались / "
            "неопознанными / без имён» краснел на узком regex -- стемы расширены.",
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
            keywords=[SILENCE_OR_OFF_ANSWER],
            must_not_say=[FALSE_FAILURE_PHRASE],
            why="Финальный якорь тишины: марафон не оставляет играющий плеер "
            "следующему прогону. #3971: ответ из снимка `<music_state>` (ADR-0148), "
            "get_music_state не требуем.",
        ),
        step(
            "n1006_session_reset",
            "Робот, новая сессия, забудь этот разговор.",
            patterns=["session reset|session_reset|Новая сессия|new session|tier1 kind=new_session"],
        ),
        step(
            "n1007_no_leak",
            "Робот, о чём мы с тобой только что говорили?",
            must_not_say=LEAK_PROBE_WINDOW_PHRASES,
            why="ЗОНД УТЕЧКИ (gp14). R20 разбора 26.10: «енот» и «Спартак» -- факты ДОЛГОЙ памяти (n206, n706 их "
            "сохраняют), они законно приходят через memory_search/speaker_memory и после сброса, а запрет "
            "подстроки по всему логу краснил честный ответ. Утечку ИСТОРИИ проверяем по фразам из окна, которых в "
            "долгой памяти нет: вопросы акта 10 до сброса (n1001 «итог ночи», n1002 «ворчали») и темы акта 9 "
            "(космос, китайский, четверостишие). Ищется в РЕЧИ робота (must_not_say), не по логу.",
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
            "n1011_goodnight",
            "Робот, спокойной ночи, смена окончена, спасибо за компанию.",
            patterns=[SPEECH_STARTED],
        ),
    ],
    [
        "Акт 10 — expected-partially-red: gp14 (утечка контекста после сброса)",
        "живьём не проверялась, а спека §2.4 обещает «старые тёрны не",
        "протекают». GATE-1 держит только memory_search и речь (`SpeechEvent started`) —",
        "то, что обязано работать независимо от исхода зондов.",
    ],
)


# =============================================================================
# Issue #2890 — какие акты НАМЕРЕННО наследуют окно диалога предыдущего.
# Харнесс (e2e_voice_test.sh:isolate_act_start) сбрасывает сессию dialogue_node
# в начале каждого акта; флаг inherits_previous_act — единственный способ
# сказать «не сбрасывать». Акт 1 — начало ночи (сам начинает с «новой
# сессии»), акт 2 стирает каст дикторов (register_speaker) и наследовать
# разговор о стёртых людях не может — харнесс это запрещает. Акты 3-10 —
# продолжение одной истории: «акт N опирается на состояние актов < N»
# (манифест), акт 10 прямо просит пересказать всю ночь.
# =============================================================================
ACTS_INHERITING_PREVIOUS = frozenset(range(3, 11))
INHERITS_REASON = (
    "issue #2890: ночной марафон — одна история; акт продолжает разговор "
    "предыдущих актов (манифест: акт N опирается на состояние актов < N), "
    "поэтому харнесс НЕ сбрасывает окно диалога перед ним"
)


# =============================================================================
# Акты, которых НЕТ в SSoT (рукописные JSON). Манифест обязан их знать: раннер идёт по манифесту, а
# перегенерация без этих записей молча выкидывала 2b/2c из ночи (#3880, расхождение генератор<->JSON).
# =============================================================================
MANUAL_ACTS: List[Dict[str, Any]] = [
    {
        "after_act": 2,
        "act": "2b",
        "title": "Переспрос личности",
        "stability": "new-unproven",
        "steps": 10,
        "slug": "act2b_identity_question",
        "_manual_not_generated": "issue #2809 -- этот акт НЕ входит в SSoT gen_night_marathon.py. "
        "Перегенерация манифеста скриптом ЗАТРЁТ эту запись -- см. \"generated_by\" в самом файле сценария.",
    },
    {
        "after_act": 2,
        "act": "2c",
        "title": "Вы разные люди?",
        "stability": "new-unproven",
        "steps": 3,
        "slug": "act2c_voice_conflict_question",
        "_manual_not_generated": "issue #2828 -- этот акт НЕ входит в SSoT gen_night_marathon.py. "
        "Перегенерация манифеста скриптом ЗАТРЁТ эту запись -- см. \"generated_by\" в самом файле сценария.",
    },
]


def _manual_entry(m: Dict[str, Any]) -> Dict[str, Any]:
    base = "night_marathon_" + m["slug"]
    return {
        "act": m["act"],
        "title": m["title"],
        "stability": m["stability"],
        "steps": m["steps"],
        "scenario_file": ".github/e2e/scenarios/night/" + base + "_v1.json",
        "acceptance_file": ".github/e2e/scenarios/night/" + base + "_acceptance_v1.json",
        "_manual_not_generated": m["_manual_not_generated"],
    }


# =============================================================================
# Сборка и запись файлов
# =============================================================================
def build() -> Dict[str, Dict[str, Any]]:
    """Все генерируемые файлы: ``{имя файла: payload}``. Без побочных эффектов (``--check`` сверяет с диском)."""
    files: Dict[str, Dict[str, Any]] = {}
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
        }
        if a["n"] in ACTS_INHERITING_PREVIOUS:
            scenario["inherits_previous_act"] = True
            scenario["_inherits_previous_act_reason"] = INHERITS_REASON
        if a["recover"] is not None:
            scenario["recover"] = a["recover"]
        scenario["steps"] = a["steps"]
        acceptance = {
            "name": base + "_acceptance_v1",
            "schema_version": 1,
            "_comment": a["gate_comment"],
            "expected_tool_calls": a["gate_tools"],
            "must_not_call": [],
            # Каст #4038: команды людей синтезирует MiniMax (Yandex-роли склеиваются 0.84-0.92).
            "voice_provenance": {
                "voice": "anton",
                "tts": "minimax",
                "volume": 100,
                "llm": "minimax-m3",
            },
        }
        files[scenario_name] = scenario
        files[acceptance_name] = acceptance
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
        for m in MANUAL_ACTS:
            if m["after_act"] == a["n"]:
                manifest["acts"].append(_manual_entry(m))
                total += m["steps"]
    manifest["total_steps"] = total
    files["night_marathon_manifest.json"] = manifest
    return files


def _dump(payload: Dict[str, Any]) -> str:
    return json.dumps(payload, ensure_ascii=False, indent=2) + "\n"


def check() -> int:
    """``--check``: JSON на диске == то, что соберёт генератор. 0 -- актуальны, 1 -- расходятся (список файлов)."""
    stale: List[str] = []
    for name, payload in build().items():
        path = os.path.join(OUT_DIR, name)
        try:
            with open(path, encoding="utf-8", newline="") as fh:
                on_disk = fh.read().replace("\r\n", "\n")
        except OSError:
            on_disk = None
        if on_disk != _dump(payload):
            stale.append(name)
    if stale:
        print("JSON устарели относительно gen_night_marathon.py "
              "(перегенерируйте: py -3 scripts/e2e/gen_night_marathon.py):")
        for name in stale:
            print("  - " + name)
        return 1
    print("JSON актуальны (%d файлов)" % len(build()))
    return 0


def emit() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    files = build()
    for name, payload in files.items():
        with open(os.path.join(OUT_DIR, name), "w", encoding="utf-8", newline="\n") as fh:
            fh.write(_dump(payload))
    total = 0
    for a in ACTS:
        total += len(a["steps"])
        print("act %-2d %-22s %2d steps  %s" % (a["n"], a["slug"], len(a["steps"]), a["stability"]))
    total += sum(m["steps"] for m in MANUAL_ACTS)
    print("-" * 52)
    print("acts: %d (+%d рукописных)   total steps: %d" % (len(ACTS), len(MANUAL_ACTS), total))
    print("out:  %s" % OUT_DIR)


if __name__ == "__main__":
    import sys as _sys

    if "--check" in _sys.argv[1:]:
        raise SystemExit(check())
    emit()
