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

__all__ = ["TOOL_NAME_RE", "invocation_markers", "tool_invoked"]

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
