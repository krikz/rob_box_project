"""Какие тулы «музыкальные» — единственная таблица знания (ADR-0158 §7, «одна реализация на знание»).

Раньше эти списки жили в пяти местах и расходились: ``agent_core._MUSIC_LAUNCH_TOOLS`` (теперь удалён),
``dialogue_node._music_tool_names``, ``media_phrases.LAUNCH_TOOLS``, ``dialogue_guards.MUSIC_STOP_TOOLS`` и
``tool_executor._MUSIC_TOOLS``. Смысл у них разный, поэтому здесь не один набор, а признаки одной таблицы
(``_ROLES``); модули импортируют готовые наборы ниже.

Имена тулов сверяются с каталогом (``TOOL_CATALOG``) тестом ``test_music_tools``: опечатка или снятый тул
валят тест, а не молча выпадают из набора.
"""

from __future__ import annotations

#: Роль тула -> тулы. Роли независимы: тул может входить в несколько.
_ROLES: dict[str, frozenset[str]] = {
    # Запускает музыку в ЭТОМ ходе: только они подкрепляют фразу запуска «включаю…» (media_phrases).
    "launch": frozenset({"dj_set", "request_music", "generate_music"}),
    # После них пользователь слышит звук с возможным вокалом; гард выдуманной лирики (#1708) и
    # «музыку мог запустить не v2-тул» (music_turn) смотрят именно на них. request_music/dj_set сюда
    # не входят намеренно: их запуск говорит код (ADR-0148), а не модель.
    "audible_lyrics": frozenset({"generate_music", "music_library"}),
    # Останавливают музыку; идут через планировщик (канал MUSIC) и ждут конца озвучки (#968).
    "stop": frozenset({"stop_music"}),
    # Музыкальные тулы, чей вызов оправдывает фразу «сделал» (CLAIM_JUSTIFYING_TOOLS) помимо launch/stop.
    "claim_extra": frozenset({"lookup_melody", "music_library"}),
}

#: Запускают музыку в этом ходе.
MUSIC_LAUNCH_TOOLS: frozenset[str] = _ROLES["launch"]
#: После них слышен звук с возможным вокалом (гард выдуманной лирики, чужой запуск в music_turn).
MUSIC_AUDIBLE_LYRICS_TOOLS: frozenset[str] = _ROLES["audible_lyrics"]
#: Останавливают музыку.
MUSIC_STOP_TOOLS: frozenset[str] = _ROLES["stop"]
#: Музыкальные тулы, вызов которых оправдывает фразу «сделал».
MUSIC_CLAIM_TOOLS: frozenset[str] = _ROLES["launch"] | _ROLES["stop"] | _ROLES["claim_extra"]
#: Все имена таблицы — для сверки с каталогом.
ALL_MUSIC_TOOL_NAMES: frozenset[str] = frozenset().union(*_ROLES.values())
