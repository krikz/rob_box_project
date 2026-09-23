#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""e2e_tool_match.py — «тул РЕАЛЬНО вызван» vs «тул просто доступен».

Зачем это существует
====================
``check_acceptance`` и ``check_gate1_aggregate`` в ``e2e_voice_test.sh``
искали имя тула ПОДСТРОКОЙ по ``docker logs voice-assistant``. Это не
работает: ``dialogue_node`` на каждом ходе печатает в лог весь запрос к
LLM, включая строку

    tools(56): clear_waypoints, compose_music, ..., stop_music, ...

и системный промпт, где тулы упомянуты по именам в правилах.

Замер на живом роботе (run 34408526453, акт 1 ночного марафона, 1065
строк лога, 11 минут прогона)::

    TOOL                       bare  quoted  zapros
    get_current_time             38       4       1   ← реально вызывался
    get_battery_level            36       4       1   ← реально вызывался
    get_sound_info               31       4       1   ← реально вызывался
    get_music_state              17       0       0   ← НЕ вызывался
    move_direction               17       0       0   ← НЕ вызывался
    clear_waypoints              17       0       0   ← НЕ вызывался
    stop_music                   17       0       0   ← НЕ вызывался

Голое имя даёт 17 совпадений у тула, который не вызывали ни разу — это
пол, который создаёт сам лог запроса. Отсюда два следствия, и оба плохие:

* ``expected_tool_calls`` проходил ВСЕГДА. GATE-1 печатал
  «✅ all checks passed» независимо от поведения робота — то есть
  ADR-0022 гейт был декоративным.
* ``must_not_call`` падал ВСЕГДА, когда его выставляли. В акте 1
  марафона шаг «у тебя сейчас играет музыка?» получил
  «❌ forbidden tool calls invoked: ['stop_music', 'execute_music_code']»,
  хотя робот в этом ходе не вызвал вообще ни одного тула (``tools=[]``).

Как отличить вызов от доступности
=================================
Реальный вызов оставляет в логе имя тула В КАВЫЧКАХ или после явного
маркера исполнения:

    dialogue_node: ✅ [turn] process_input returned: ... tools=['get_current_time', 'speak_text']
    dialogue_node:   [3] assistant: '' tool_calls=(ToolCall(id='...', name='get_current_time', ...),)
    dialogue_node: 📤 Отправлен запрос b5119339: get_current_time
    mcp_server:    📥 Запрос выполнения: get_current_time с параметрами {}
    mcp_server:    ✅ Инструмент get_current_time выполнен успешно

Списки доступных тулов и текст промпта имён в кавычках не содержат —
там они через запятую без кавычек. Поэтому кавычки и маркеры исполнения
дают чистый ноль на невызванных тулах (колонки quoted/zapros выше).

Контракт функции
================
``tool_invoked(logs, frag)``:

* ``frag`` похож на имя тула (``^[a-z][a-z0-9_]*$``) → ищем ТОЛЬКО
  маркеры реального вызова;
* иначе (``"STOP command received"``, ``"Cancel: new STT input"``,
  ``"session reset"``) → обычный подстрочный поиск, как раньше.

Второе правило обязательно: сценарии кладут в ``must_not_call`` не
только имена тулов, но и куски строк лога (например, проверка «добавка
куплета не оборвала песню» ищет отсутствие ``STOP command received``).
Ломать этот способ нельзя.
"""

from __future__ import annotations

import re

__all__ = ["TOOL_NAME_RE", "invocation_markers", "tool_invoked",
           "first_invocation_position", "VOICE_CYCLE_MARKERS",
           "registration_outcome", "registration_expected",
           "registration_failures", "registration_pending"]

# Маркеры голосового ответа (любой из них обозначает «робот начал говорить»).
# Используются для assertion «discovery tool был вызван ДО голосового ответа»
# (issue #2406 — verbal-only LLM answers на discovery/inquiry-шагах).
# Все маркеры — lower-case: вызывающий сравнивает с логом в lower-case.
VOICE_CYCLE_MARKERS = (
    # Финальная строка dialogue_node после LLM/TTS цикла (есть spoken='...').
    "✅ [turn] process_input returned:",
    # tool_calls перечисление (speak_text = голосовой ответ, не молчание).
    "'speak_text'",
    # mcp_server финальный ack по speak_text.
    "инструмент speak_text выполнен",
    # TTS ack от yandex_tts / minimax_tts.
    "tts finished",
)

#: Как выглядит имя тула. Все 56 зарегистрированных тулов —
#: snake_case из ``def name(self) -> str`` в rob_box_mcp_tools/tools/*.py.
TOOL_NAME_RE = re.compile(r"^[a-z][a-z0-9_]*$")


def invocation_markers(tool: str) -> list:
    """Подстроки, каждая из которых доказывает РЕАЛЬНЫЙ вызов ``tool``.

    Все — lower-case: вызывающий сравнивает с логом в lower-case.
    """
    t = tool.lower()
    return [
        # tools=['x', 'y'] в итоговой строке хода + ToolCall(name='x')
        "'%s'" % t,
        '"%s"' % t,
        # dialogue_node → mcp_server и обратно
        "запрос выполнения: %s" % t,
        "инструмент %s выполнен" % t,
        "публикую результат для %s" % t,
    ]


def tool_invoked(logs: str, frag: str) -> bool:
    """True, если ``frag`` найден как РЕАЛЬНЫЙ вызов (или как обычная
    подстрока, если ``frag`` — не имя тула)."""
    if frag is None:
        return False
    low = logs.lower()
    frag_l = frag.strip().lower()
    if not frag_l:
        return False
    if not TOOL_NAME_RE.match(frag_l):
        # свободный текст (кусок строки лога) — поведение как было
        return frag_l in low
    return any(m in low for m in invocation_markers(frag_l))


def first_invocation_position(logs: str, tool: str) -> int | None:
    """Позиция первого РЕАЛЬНОГО вызова ``tool`` в ``logs`` (lower-case
    индекс первого символа матча), или ``None`` если тул не вызывался.

    Используется для assertion «discovery tool был вызван ДО голосового
    ответа» (issue #2406, ретро n313). Возвращаем позицию (int), чтобы
    вызывающий мог сравнить её с позицией первого voice-cycle маркера.

    Не-имя тула (free text) — функция НЕ поддерживает, контракт ``TOOL_NAME_RE``.
    Вызывающий валидирует имя до вызова.
    """
    if not tool or not TOOL_NAME_RE.match(tool.lower()):
        return None
    low = logs.lower()
    return _first_marker_position(low, invocation_markers(tool.lower()))


def _first_marker_position(lowered_logs: str, markers: list[str] | tuple[str]) -> int | None:
    """Helper: индекс первого вхождения любой из подстрок в lower-case логе.

    Возвращает ``None`` если ни один маркер не найден. Раньше аналогичный
    код был inline в check_acceptance, теперь — общий хелпер.
    """
    earliest: int | None = None
    for m in markers:
        idx = lowered_logs.find(m)
        if idx == -1:
            continue
        if earliest is None or idx < earliest:
            earliest = idx
    return earliest


def first_voice_cycle_position(logs: str) -> int | None:
    """Позиция первого маркера голосового цикла в ``logs`` (lower-case индекс),
    или ``None`` если голосового ответа ещё не было.

    Используется для assertion «discovery tool был вызван ДО первого
    голосового ответа» (issue #2406). Маркеры те же, что в soft-hint
    ``check_gate1_aggregate``: 'tts finished' / 'speak_text' в списке tools=
    или финальная строка ``✅ [turn] process_input returned:``.
    """
    low = logs.lower()
    return _first_marker_position(low, list(VOICE_CYCLE_MARKERS))


# =============================================================================
# Issue #2764: expected_keywords искались по ВСЕМУ логу шага
# =============================================================================
# Та же болезнь, что вылечена выше для имён тулов, только в другом канале.
# ``check_acceptance`` матчил ``expected_keywords`` подстрокой по всему
# ``docker logs voice-assistant --since <шаг>``. А в этот лог попадает и
# реплика САМОГО ГОВОРЯЩЕГО, и подпись диктора, которую ставит
# speaker_id_node:
#
#   dialogue_node: user_input='[Spkr:Саша] привет, давай знакомиться...'
#   dialogue_node: 👤 [issue 1077] Speaker: 'Саша' conf=0.81
#
# Из-за этого три из четырёх keyword-проверок акта 2 были тавтологиями —
# зелёными независимо от поведения робота (замер 22.09.2026):
#
#   n207_recall_sasha      KW=['Саш']                 говорит Саша → «Саш» в логе всегда
#   n209_recall_boris      KW=['Борис|Спартак|пицц']  говорит Борис → «Борис» в логе всегда
#   n211_who_do_you_know   KW=['Саш', 'Борис']        половина ключа бесплатная
#
# А шаг n207 при этом объявлен в сценарии как проверка связки
# «голос → профиль → факты, а не просто вежливый ответ». Проверки не было.
#
# Лечение: ключевые слова ищутся ТОЛЬКО в том, что робот произнёс.
# Три канала, все три реально встречаются в логе живого робота
# (замер 22.09.2026, Vision Pi):
#
#   tts_node:    🔊 TTS: speech_id=..., voice=default, lang=default, text='Добрый день, Денис!'
#   mcp_server:  📥 Запрос выполнения: speak_text с параметрами {'text': 'Лицо знакомое...', 'animation': 'happy'}
#   dialogue_node: ✅ [turn] process_input returned: spoken='Привет! У меня всё отлично...'[:60]
#
# Третий канал обрезан до 60 символов — он идёт последним и нужен как
# подстраховка, когда TTS не доехал (например, guard заглушил синтез).
_SPEECH_PATTERNS = (
    # 🔊 TTS: ... text='...'  — text идёт последним полем строки
    re.compile(r"🔊 TTS:.*?\btext='(.*?)'\s*$", re.MULTILINE),
    # Запрос выполнения: speak_text с параметрами {'text': '...', ...}
    re.compile(r"speak_text с параметрами \{'text':\s*'(.*?)'\s*[,}]"),
    # process_input returned: spoken='...'[:60]  /  spoken='...' (len=NN)
    re.compile(r"spoken='(.*?)'(?:\[:\d+\]|\s*\(len=\d+\))"),
)


def robot_speech(logs: str) -> str:
    """Только то, что робот ПРОИЗНЁС, склеенное в одну строку.

    Пустая строка означает «робот в этом окне не сказал ничего» — это
    валидный (красный) исход шага, а не сбой парсера: молчащий робот не
    должен проходить keyword-проверку.
    """
    if not logs:
        return ""
    out: list[str] = []
    for pat in _SPEECH_PATTERNS:
        out.extend(pat.findall(logs))
    return "\n".join(out)


def keyword_hit(logs: str, kw: str) -> bool:
    """``kw`` (алтернативы через ``|``) найден в РЕЧИ робота.

    Контракт совпадает со старым ``_keyword_hit``: регистронезависимо,
    ``|`` — это ИЛИ, пустые альтернативы игнорируются.
    """
    speech = robot_speech(logs).lower()
    if not speech:
        return False
    return any(v.strip() and v.strip() in speech for v in kw.lower().split("|"))


# ── Issue #2846: «register_speaker вызван» != «голос зарегистрирован» ─────────
#
# Живой прогон 35875477264 (акт 2, develop f9b826d): шаги n201…n202c получили
# ``E2E_STEP … OK``, а speaker_id_node отклонял КАЖДУЮ регистрацию:
#
#   [speaker_id_node] ⚠️ [issue #2829] register_request for 'Саша' has no
#       utterance_id -- honest refusal instead of registering whoever speaks next
#   [dialogue_node]   ⚠️ [issue #2829] Регистрация 'Саша' отклонена:
#       no_utterance_context (utterance_id=None)
#
# ``expected_tool_calls``/``discovery_tools`` доказывают только, что LLM
# ДЁРНУЛА тул. register_speaker — fire-and-forget публикация в топик: тул
# «выполнен успешно» и тогда, когда нода биометрии профиль не завела.
# Исход регистрации виден ТОЛЬКО по логу speaker_id_node / dialogue_node.
#
# Регэкспы ниже — копии f-строк из кода робота, а не «похожий вид»
# (src/rob_box_voice/rob_box_voice/speaker_id_node.py):
#   _do_register:            ✅ Speaker '{name}' registered (id=…)
#   _do_register (ADR-0127): ⚠️ Speaker '{name}' (id=…) — голос похож на уже известного …
#   _do_register:            🔗 Speaker '{name}' merged into existing profile (id=…)
#   _on_register_request:    register_request for '{name}' has no utterance_id
#   _register_after_wait:    register_request for '{name}': no embedding for utterance=…
#   _do_register (#2769):    Registration of '{name}' rejected — audio too short
# и dialogue_node.py (_on_speaker_result):
#   Регистрация '{name}' отклонена: {error} (utterance_id=…)
#   Регистрация '{name}' отклонена — реплика …с короче …   (too_short)
# Отказ ищется по ОБЕИМ нодам: dialogue_node параллельно правят (#2842), и
# если его строка поменяется, строка speaker_id_node всё равно поймает отказ.

REGISTER_TOOL = "register_speaker"

_REG_ACCEPTED_RES = (
    # Новый профиль (и тёзка #2747 — у него та же строка).
    re.compile(r"✅ Speaker '([^']*)' registered \(id=[0-9a-f]*\)"),
    # ADR-0127: голос похож на чужой, имя другое — заведён ОТДЕЛЬНЫЙ профиль.
    # Это принятая регистрация (данные целы); переспрос проверяют n722/n723.
    re.compile(r"Speaker '([^']*)' \(id=[0-9a-f]*\) — голос похож на уже известного"),
)
_REG_MERGED_RE = re.compile(
    r"Speaker '([^']*)' merged into existing profile \(id=[0-9a-f]*\)"
)
_REG_REJECTED_RES = (
    (re.compile(r"register_request for '([^']*)' has no utterance_id"),
     "no_utterance_context"),
    (re.compile(r"register_request for '([^']*)': no embedding for utterance="),
     "utterance_not_found"),
    (re.compile(r"Registration of '([^']*)' rejected"), "too_short"),
)
# Причину dialogue_node печатает явно после двоеточия; у too_short вместо
# двоеточия тире — тогда причины в строке нет, и это too_short.
_REG_REJECTED_DIALOGUE_RE = re.compile(
    r"Регистрация '([^']*)' отклонена(?::\s*([A-Za-z_]+))?"
)


def registration_outcome(logs: str) -> dict:
    """Что speaker_id_node СДЕЛАЛ с регистрациями в окне ``logs``.

    ``{"accepted": [имя, …], "merged": [строка, …],
    "rejected": [{"name": …, "reason": …}, …]}`` — без дублей: один и тот
    же отказ печатают обе ноды, в вердикт он должен попасть один раз.
    """
    accepted: list = []
    merged: list = []
    rejected: list = []
    seen_rej: set = set()
    if not logs:
        return {"accepted": accepted, "merged": merged, "rejected": rejected}
    for pat in _REG_ACCEPTED_RES:
        for name in pat.findall(logs):
            if name not in accepted:
                accepted.append(name)
    for m in _REG_MERGED_RE.finditer(logs):
        if m.group(0) not in merged:
            merged.append(m.group(0))

    def _add(name: str, reason: str) -> None:
        if (name, reason) not in seen_rej:
            seen_rej.add((name, reason))
            rejected.append({"name": name, "reason": reason})

    for pat, reason in _REG_REJECTED_RES:
        for name in pat.findall(logs):
            _add(name, reason)
    for name, reason in _REG_REJECTED_DIALOGUE_RE.findall(logs):
        _add(name, reason or "too_short")
    return {"accepted": accepted, "merged": merged, "rejected": rejected}


def _acc_list(acc: dict, key: str) -> list:
    val = acc.get(key) or []
    if not isinstance(val, list):
        return []
    return [v for v in val if isinstance(v, str)]


def registration_expected(acc: dict) -> bool:
    """Обязан ли шаг ДОКАЗАТЬ, что регистрация голоса принята.

    Авто-правило: да, если ``register_speaker`` есть в ``expected_tool_calls``
    или ``discovery_tools`` шага — сценарий, требующий вызова тула, требует
    и его результата (иначе зелёный шаг поверх пустой базы дикторов).
    Явное булево ``require_registration_accepted`` перекрывает авто-правило
    в обе стороны. Не-булево значение — ошибка схемы (её отдаёт
    :func:`registration_failures`), при нём действует авто-правило.
    """
    flag = acc.get("require_registration_accepted")
    if isinstance(flag, bool):
        return flag
    return (REGISTER_TOOL in _acc_list(acc, "expected_tool_calls")
            or REGISTER_TOOL in _acc_list(acc, "discovery_tools"))


def _registration_in_scope(acc: dict) -> bool:
    """Шаг вообще про регистрацию (в т.ч. ``must_not_call: [register_speaker]``)
    — тогда склейка профиля тоже проверяется, как в прежней bash-проверке
    ``*register_speaker*`` scenario-цикла (run 35667281570)."""
    return (registration_expected(acc)
            or REGISTER_TOOL in _acc_list(acc, "must_not_call"))


def registration_pending(acc: dict, logs: str) -> bool:
    """True — исход регистрации ожидается, но в логе его ещё нет.

    speaker_id_node ждёт эмбеддинг фразы до 1.5с (_REGISTER_UTTERANCE_WAIT_SEC)
    после вызова тула. Если LLM дёрнула тул последним действием хода, ack
    может лечь в лог позже, чем харнесс начнёт проверку — check_acceptance
    дочитывает лог, пока эта функция True (не дольше таймаута).
    """
    if not registration_expected(acc):
        return False
    out = registration_outcome(logs)
    return not (out["accepted"] or out["merged"] or out["rejected"])


def registration_failures(acc: dict, logs: str) -> list:
    """Причины FAIL шага по исходу регистрации (пустой список — всё честно).

    * отказ speaker_id_node (``no_utterance_context`` / ``utterance_not_found``
      / ``too_short``) — FAIL, даже если тул «выполнен успешно»;
    * регистрация ожидалась, но ни принятия, ни отказа в логе нет — FAIL:
      не доказано — не принято (ADR-0018);
    * склейка с уже известным профилем — FAIL (перенесено из bash-проверки
      scenario-цикла, чтобы вердикт шага был ОДНОЙ строкой, а не
      «✅ all checks passed» и следом «❌ регистрация СКЛЕИЛАСЬ»).
    """
    failures: list = []
    flag = acc.get("require_registration_accepted")
    if flag is not None and not isinstance(flag, bool):
        failures.append(
            f"require_registration_accepted must be true/false, got {flag!r}"
        )
    if not _registration_in_scope(acc):
        return failures
    out = registration_outcome(logs)
    if out["merged"]:
        failures.append(
            "speaker registration merged into an existing profile: "
            + "; ".join(out["merged"])
            + " (not a separate profile, run 35667281570)"
        )
    if not registration_expected(acc):
        return failures
    if out["rejected"]:
        failures.append(
            "speaker registration REJECTED by speaker_id_node: "
            + ", ".join(f"{r['name']!r} {r['reason']}" for r in out["rejected"])
            + " (issue #2846: register_speaker was called, no profile saved)"
        )
    elif not (out["accepted"] or out["merged"]):
        failures.append(
            "speaker registration NOT confirmed: no \"✅ Speaker '<name>' "
            "registered\" from speaker_id_node in the step log (issue #2846: "
            "a register_speaker call alone does not prove the voice was saved)"
        )
    return failures
