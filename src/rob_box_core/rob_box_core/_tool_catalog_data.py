"""Dialogue tool catalog — GENERATED, do not edit by hand.

Source of truth: the ``MCPTool`` subclasses in
``src/rob_box_mcp_tools/rob_box_mcp_tools/tools/*.py``.

Regenerate with::

    python tools/gen_tool_catalog.py

``test_tool_catalog_is_current`` fails the build when this file drifts from
the tool classes, which is what keeps the LLM-facing catalog and the
executable tools from disagreeing (they did, for a long time — see the
generator's module docstring).

Consumers: ``rob_box_core.tool_catalog`` (typed access) and, through it,
``rob_box_harness.core.tool_registry`` — neither may import ROS2, which is
why this is checked-in data rather than an import-time reflection.
"""

from __future__ import annotations

from typing import Any

#: One entry per ``MCPTool`` subclass, sorted by name. ``signature`` mirrors
#: what ``execute()`` accepts so the catalog can be verified against the
#: code that runs it.
TOOL_CATALOG_DATA: tuple[dict[str, Any], ...] = (   {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': False,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'add_music_material',
        'description': 'Принять музыкальный материал, который человек ПРИСЛАЛ в '
                       'сообщении (RTTTL-строка, Strudel/Tidal-код '
                       'note("...")/n("..."), или список нот «e4 g4 b4 c5»), и '
                       'сохранить его в библиотеку мелодий. Возвращает name — им '
                       'играется хук: compose_music(style="club", name=<name>). '
                       'Передай text ДОСЛОВНО как прислал человек. Если не распознал — '
                       'вернёт честную ошибку: скажи об этом и не выдумывай ноты.',
        'parameters': {   'type': 'object',
                          'properties': {   'text': {   'type': 'string',
                                                        'description': 'Текст '
                                                                       'сообщения с '
                                                                       'материалом '
                                                                       'дословно (код, '
                                                                       'RTTTL или '
                                                                       'ноты).'},
                                            'title': {   'type': 'string',
                                                         'description': 'Название '
                                                                        'материала '
                                                                        '(необязательно; '
                                                                        'иначе из '
                                                                        'комментария в '
                                                                        'коде).'}},
                          'required': ['text'],
                          'additionalProperties': False},
        'signature': {   'params': ['text', 'title'],
                         'required': ['text'],
                         'accepts_kwargs': False},
        'skill': ('composer', 'dj')},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'instant',
        'name': 'clear_waypoints',
        'description': 'Удалить ВСЕ сохранённые точки на текущей карте. Используй '
                       "когда пользователь говорит 'очисти все точки', 'удали все "
                       "точки'.",
        'parameters': {   'type': 'object',
                          'properties': {},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': [], 'required': [], 'accepts_kwargs': False},
        'skill': ('navigation',)},
    {   'llm_visible': False,
        'operator_visible': False,
        'read_only': True,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'medium',
        'name': 'container_status',
        'description': 'Получить статус контейнеров (restart-count, CPU, RAM, uptime) '
                       'через Prometheus/cAdvisor. Используй когда нужно проверить '
                       "здоровье контейнеров ('покажи состояние voice-assistant').",
        'parameters': {   'type': 'object',
                          'properties': {   'name': {   'type': 'string',
                                                        'description': 'Имя (или '
                                                                       'подстрока '
                                                                       'имени) '
                                                                       'контейнера. '
                                                                       'Пусто — все '
                                                                       'контейнеры.'}},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': ['name'], 'required': [], 'accepts_kwargs': False},
        'skill': ()},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'medium',
        'name': 'continue_mapping',
        'description': 'Продолжить картографирование территории (режим SLAM). '
                       'Используй для добавления новых областей к карте.',
        'parameters': {   'type': 'object',
                          'properties': {},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': [], 'required': [], 'accepts_kwargs': False},
        'skill': ('mapping',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'instant',
        'name': 'delete_waypoint',
        'description': 'Удалить сохранённую точку по имени. Используй когда '
                       "пользователь говорит 'удали зал', 'забудь кухню'.",
        'parameters': {   'type': 'object',
                          'properties': {   'name': {   'type': 'string',
                                                        'description': 'Название точки '
                                                                       'для удаления'}},
                          'required': ['name'],
                          'additionalProperties': False},
        'signature': {   'params': ['name'],
                         'required': ['name'],
                         'accepts_kwargs': False},
        'skill': ('navigation',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': False,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'dj_set',
        'description': 'Диджей-сет: action=start — начать сет на тему theme (треки, '
                       'переходы и темп решает код), tracks — сколько треков, только '
                       'если человек назвал число (без числа длину решает код); '
                       'action=stop — закончить сет и выключить музыку. Об успехе '
                       'робот скажет сам, когда музыка реально заиграет; ok=false — '
                       'музыка не заиграла.',
        'parameters': {   'type': 'object',
                          'properties': {   'action': {   'type': 'string',
                                                          'description': 'start — '
                                                                         'начать сет, '
                                                                         'stop — '
                                                                         'закончить',
                                                          'enum': ['start', 'stop']},
                                            'theme': {   'type': 'string',
                                                         'description': 'Тема сета '
                                                                        'словами '
                                                                        'человека '
                                                                        '(например '
                                                                        '«космос»)'},
                                            'persona': {   'type': 'string',
                                                           'description': 'Имя диджея '
                                                                          'для реплик'},
                                            'style': {   'type': 'string',
                                                         'description': 'Стиль сета '
                                                                        '(один на весь '
                                                                        'сет); auto — '
                                                                        'по словам '
                                                                        'темы, иначе '
                                                                        'клубный',
                                                         'enum': [   'auto',
                                                                     'club',
                                                                     'rave',
                                                                     'synthwave',
                                                                     'chiptune',
                                                                     'breaks',
                                                                     'dnb',
                                                                     'lofi',
                                                                     'rock',
                                                                     'jazz']},
                                            'tracks': {   'type': 'integer',
                                                          'description': 'Сколько '
                                                                         'треков в '
                                                                         'сете — '
                                                                         'только если '
                                                                         'человек '
                                                                         'назвал число',
                                                          'minimum': 1,
                                                          'maximum': 60}},
                          'required': ['action'],
                          'additionalProperties': False},
        'signature': {   'params': [   'action',
                                       'theme',
                                       'persona',
                                       'style',
                                       'tracks',
                                       'heard_tracks',
                                       'turn_id',
                                       'heard_text'],
                         'required': [],
                         'accepts_kwargs': False},
        'skill': ('dj',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': False,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'estimate_tts_duration',
        'description': 'Оценить длительность TTS-озвучки для заданного текста в '
                       'секундах. Используется для планирования аранжировки музыки под '
                       'длительность рэпа/стиха. Возвращает estimate_sec (float) — '
                       'примерное время звучания с учётом chipmunk-ускорения.',
        'parameters': {   'type': 'object',
                          'properties': {   'text': {   'type': 'string',
                                                        'description': 'Текст для '
                                                                       'оценки '
                                                                       'длительности '
                                                                       'озвучки.'},
                                            'chars_per_second': {   'type': 'number',
                                                                    'description': 'Скорость '
                                                                                   'озвучки '
                                                                                   '(символов/сек). '
                                                                                   'По '
                                                                                   'умолчанию '
                                                                                   '30 '
                                                                                   '— '
                                                                                   'калиброванное '
                                                                                   'значение '
                                                                                   'для '
                                                                                   'русского '
                                                                                   'TTS '
                                                                                   'с '
                                                                                   'chipmunk '
                                                                                   '2x.'}},
                          'required': ['text'],
                          'additionalProperties': False},
        'signature': {   'params': ['text', 'chars_per_second'],
                         'required': ['text'],
                         'accepts_kwargs': False},
        'skill': ('voice-tts',)},
    {   'llm_visible': False,
        'operator_visible': False,
        'read_only': False,
        'destructive': False,
        'idempotent': False,
        'starts_music': True,
        'execution_type': 'fast',
        'name': 'execute_music_code',
        'description': 'Выполнить готовый Renardo-код (харнесс живой проверки, LLM не '
                       'виден). Код выполняется в контексте Renardo '
                       "(FoxDot-совместимый синтаксис). Пример: 'p1 >> pluck([0, 2, "
                       "4], dur=0.5, amp=0.8)'. Опасные системные команды "
                       'автоматически блокируются. Укажи pattern_name чтобы паттерн '
                       'можно было остановить или изменить позже.',
        'parameters': {   'type': 'object',
                          'properties': {   'code': {   'type': 'string',
                                                        'description': 'Строка '
                                                                       'Python/Renardo-кода '
                                                                       'для '
                                                                       'выполнения. '
                                                                       "Например: 'p1 "
                                                                       '>> pluck([0, '
                                                                       "2, 4])'"},
                                            'pattern_name': {   'type': 'string',
                                                                'description': 'Имя '
                                                                               'паттерна '
                                                                               'для '
                                                                               'хранения '
                                                                               'в '
                                                                               'истории '
                                                                               '(например: '
                                                                               "'p1', "
                                                                               "'bass', "
                                                                               "'drums'). "
                                                                               'Используется '
                                                                               'для '
                                                                               'последующей '
                                                                               'мутации '
                                                                               'или '
                                                                               'остановки '
                                                                               'паттерна.'},
                                            'segments': {   'type': 'integer',
                                                            'description': 'Сколько '
                                                                           'тактов '
                                                                           '(баров) '
                                                                           'должна '
                                                                           'играть '
                                                                           'фоновая '
                                                                           'музыка (1 '
                                                                           'бар = 4 '
                                                                           'бита). Это '
                                                                           'ТОЛЬКО '
                                                                           'предохранитель: '
                                                                           'система '
                                                                           'сама '
                                                                           'останавливает '
                                                                           'музыку '
                                                                           'после '
                                                                           'tts_batch_complete, '
                                                                           'segments '
                                                                           'лишь '
                                                                           'ограничивает '
                                                                           'время '
                                                                           'игры, если '
                                                                           'TTS завис. '
                                                                           'Для песни '
                                                                           'обычно '
                                                                           '8-16 '
                                                                           'тактов. '
                                                                           'Если не '
                                                                           'знаешь — '
                                                                           'НЕ '
                                                                           'указывай '
                                                                           '(дефолт: '
                                                                           'музыка '
                                                                           'играет до '
                                                                           'конца '
                                                                           'озвучки). '
                                                                           '#990'},
                                            'duration_sec': {   'type': 'number',
                                                                'description': 'DEPRECATED '
                                                                               '(#990) '
                                                                               '— '
                                                                               'игнорируется '
                                                                               'для '
                                                                               'остановки '
                                                                               'музыки, '
                                                                               'оставлен '
                                                                               'для '
                                                                               'обратной '
                                                                               'совместимости. '
                                                                               'НЕ '
                                                                               'используй. '
                                                                               'Вместо '
                                                                               'него '
                                                                               'передавай '
                                                                               'segments.'}},
                          'required': ['code'],
                          'additionalProperties': False},
        'signature': {   'params': ['code', 'pattern_name', 'segments', 'duration_sec'],
                         'required': ['code'],
                         'accepts_kwargs': False},
        'skill': ()},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': True,
        'destructive': False,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'faq_search',
        'description': 'Поиск по FAQ активного мероприятия. Используй когда '
                       'пользователь спрашивает о программе, локации, поступлении, '
                       'организации или других деталях мероприятия. Возвращает '
                       'вопрос-ответные пары из загруженного FAQ-файла.',
        'parameters': {   'type': 'object',
                          'properties': {   'query': {   'type': 'string',
                                                         'description': 'Поисковый '
                                                                        'запрос — '
                                                                        'вопрос или '
                                                                        'ключевые '
                                                                        'слова о '
                                                                        'мероприятии.'},
                                            'limit': {   'type': 'integer',
                                                         'description': 'Максимальное '
                                                                        'количество '
                                                                        'результатов '
                                                                        '(по умолчанию '
                                                                        '3, максимум '
                                                                        '10).'}},
                          'required': ['query'],
                          'additionalProperties': False},
        'signature': {   'params': ['query', 'limit'],
                         'required': ['query'],
                         'accepts_kwargs': False},
        'skill': ('knowledge',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'medium',
        'name': 'finish_mapping',
        'description': 'Завершить картографирование и перейти в режим навигации по '
                       'готовой карте (локализация). Можно указать имя карты (например '
                       "'квартира', 'офис').",
        'parameters': {   'type': 'object',
                          'properties': {   'map_name': {   'type': 'string',
                                                            'description': 'Название '
                                                                           'карты '
                                                                           '(опционально, '
                                                                           'например '
                                                                           "'квартира')"}},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': ['map_name'], 'required': [], 'accepts_kwargs': False},
        'skill': ('mapping',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'gen_delete_from_library',
        'description': 'Удалить трек из библиотеки сгенерированной музыки: запись в '
                       'SQLite + mp3/meta.json на диске. Необратимо. Используй когда '
                       'юзер говорит «удали тот грустный трек», «удали последний '
                       'трек».',
        'parameters': {   'type': 'object',
                          'properties': {   'track_id': {   'type': 'string',
                                                            'description': 'UUID трека '
                                                                           'в '
                                                                           'библиотеке '
                                                                           '(получи из '
                                                                           'gen_list_library '
                                                                           '/ '
                                                                           'gen_search_library '
                                                                           '/ '
                                                                           'generate_music). '
                                                                           'Пример: '
                                                                           "'b7216742f61f44078e4a17f7acbed388'."}},
                          'required': ['track_id'],
                          'additionalProperties': False},
        'signature': {   'params': ['track_id'],
                         'required': ['track_id'],
                         'accepts_kwargs': True},
        'skill': ('player',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': True,
        'destructive': False,
        'idempotent': True,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'gen_get_track_info',
        'description': 'Получить подробные метаданные одного трека: title, prompt, '
                       'lyrics, tags, mood, genre, duration_ms, sample_rate, bitrate, '
                       'play_count, rating, created_at, path. Используй когда юзер '
                       'спрашивает «что за трек такой-то», «расскажи про этот трек».',
        'parameters': {   'type': 'object',
                          'properties': {   'track_id': {   'type': 'string',
                                                            'description': 'UUID трека '
                                                                           'в '
                                                                           'библиотеке '
                                                                           '(получи из '
                                                                           'gen_list_library '
                                                                           '/ '
                                                                           'gen_search_library '
                                                                           '/ '
                                                                           'generate_music). '
                                                                           'Пример: '
                                                                           "'b7216742f61f44078e4a17f7acbed388'."}},
                          'required': ['track_id'],
                          'additionalProperties': False},
        'signature': {   'params': ['track_id'],
                         'required': ['track_id'],
                         'accepts_kwargs': True},
        'skill': ('player',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': True,
        'destructive': False,
        'idempotent': True,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'gen_list_library',
        'description': 'Показать список треков из библиотеки сгенерированной музыки '
                       '(/data/music_library). Возвращает id, title, prompt, tags, '
                       'rating, play_count, created_at. Используй когда юзер '
                       'спрашивает «что у нас в библиотеке», «какие треки есть», «что '
                       'ты раньше генерировал».',
        'parameters': {   'type': 'object',
                          'properties': {   'limit': {   'type': 'integer',
                                                         'description': 'Максимум '
                                                                        'треков '
                                                                        '(default 20, '
                                                                        'max 100).',
                                                         'default': 20},
                                            'sort_by': {   'type': 'string',
                                                           'description': 'Сортировка: '
                                                                          "'recent' "
                                                                          '(default), '
                                                                          "'popular', "
                                                                          "'rating'.",
                                                           'enum': [   'recent',
                                                                       'popular',
                                                                       'rating'],
                                                           'default': 'recent'},
                                            'tag': {   'type': 'string',
                                                       'description': 'Фильтр по тегу '
                                                                      '(точное '
                                                                      'совпадение).',
                                                       'default': ''},
                                            'mood': {   'type': 'string',
                                                        'description': 'Фильтр по '
                                                                       'настроению '
                                                                       '(точное '
                                                                       'совпадение).',
                                                        'default': ''}},
                          'required': [],
                          'additionalProperties': False},
        'signature': {   'params': ['limit', 'sort_by', 'tag', 'mood'],
                         'required': [],
                         'accepts_kwargs': True},
        'skill': ('player',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': False,
        'idempotent': False,
        'starts_music': True,
        'execution_type': 'fast',
        'name': 'gen_play_from_library',
        'description': 'Воспроизвести mp3-трек из библиотеки сгенерированной музыки '
                       '(публикует путь в sound_node, который играет файл через '
                       'динамик). Возвращает track_id, path, duration_ms, title. '
                       'play_count увеличивается на 1. Используй когда юзер говорит '
                       '«сыграй тот грустный трек», «давай послушаем ещё раз».',
        'parameters': {   'type': 'object',
                          'properties': {   'track_id': {   'type': 'string',
                                                            'description': 'UUID трека '
                                                                           'в '
                                                                           'библиотеке '
                                                                           '(получи из '
                                                                           'gen_list_library '
                                                                           '/ '
                                                                           'gen_search_library '
                                                                           '/ '
                                                                           'generate_music). '
                                                                           'Пример: '
                                                                           "'b7216742f61f44078e4a17f7acbed388'."}},
                          'required': ['track_id'],
                          'additionalProperties': False},
        'signature': {   'params': ['track_id'],
                         'required': ['track_id'],
                         'accepts_kwargs': True},
        'skill': ('player',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'gen_save_to_library',
        'description': 'Обновить метаданные (tags, rating, notes, mood, genre) для уже '
                       'сгенерированного трека. Используй когда юзер говорит «сохрани '
                       'этот трек», «пометь его как любимый», «поставь 5 звёзд».',
        'parameters': {   'type': 'object',
                          'properties': {   'track_id': {   'type': 'string',
                                                            'description': 'UUID трека '
                                                                           'в '
                                                                           'библиотеке '
                                                                           '(получи из '
                                                                           'gen_list_library '
                                                                           '/ '
                                                                           'gen_search_library '
                                                                           '/ '
                                                                           'generate_music). '
                                                                           'Пример: '
                                                                           "'b7216742f61f44078e4a17f7acbed388'."},
                                            'title': {   'type': 'string',
                                                         'description': 'Новое '
                                                                        'название '
                                                                        '(пустая '
                                                                        'строка = не '
                                                                        'менять).',
                                                         'default': ''},
                                            'tags': {   'type': 'string',
                                                        'description': 'Новый список '
                                                                       'тегов через '
                                                                       'запятую '
                                                                       '(заменяет '
                                                                       'существующие).',
                                                        'default': ''},
                                            'mood': {   'type': 'string',
                                                        'description': 'Настроение '
                                                                       '(заменяет '
                                                                       'существующее).',
                                                        'default': ''},
                                            'genre': {   'type': 'string',
                                                         'description': 'Жанр '
                                                                        '(заменяет '
                                                                        'существующий).',
                                                         'default': ''},
                                            'rating': {   'type': 'integer',
                                                          'description': 'Рейтинг 0-5 '
                                                                         '(-1 = не '
                                                                         'менять).',
                                                          'default': -1},
                                            'notes': {   'type': 'string',
                                                         'description': 'Заметки '
                                                                        '(заменяют '
                                                                        'существующие).',
                                                         'default': ''}},
                          'required': ['track_id'],
                          'additionalProperties': False},
        'signature': {   'params': [   'track_id',
                                       'title',
                                       'tags',
                                       'mood',
                                       'genre',
                                       'rating',
                                       'notes'],
                         'required': ['track_id'],
                         'accepts_kwargs': True},
        'skill': ('player',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': True,
        'destructive': True,
        'idempotent': True,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'gen_search_library',
        'description': 'Искать треки в библиотеке сгенерированной музыки по ключевому '
                       'слову. Поиск по title/prompt/lyrics/genre/mood/notes. '
                       'Используй когда юзер говорит «найди трек про дождь», «сыграй '
                       'тот грустный трек», «есть что-то романтичное?».',
        'parameters': {   'type': 'object',
                          'properties': {   'query': {   'type': 'string',
                                                         'description': 'Поисковый '
                                                                        'запрос (1-200 '
                                                                        'chars).'},
                                            'limit': {   'type': 'integer',
                                                         'description': 'Максимум '
                                                                        'результатов '
                                                                        '(default 5, '
                                                                        'max 20).',
                                                         'default': 5}},
                          'required': ['query'],
                          'additionalProperties': False},
        'signature': {   'params': ['query', 'limit'],
                         'required': ['query'],
                         'accepts_kwargs': True},
        'skill': ('player',)},
    {   'llm_visible': False,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': True,
        'execution_type': 'long',
        'name': 'generate_music',
        'description': 'Сгенерировать новый музыкальный трек через MiniMax Music API и '
                       'сохранить в библиотеку (/data/music_library). Возвращает '
                       'track_id и путь к mp3. Генерация занимает 40-160 секунд — '
                       'обязательно предупреди юзера о паузе! Используй когда юзер '
                       'просит «спой/сыграй что-нибудь новое» или называет конкретную '
                       'тему/стиль, которой нет в сохранённой библиотеке (проверь '
                       'через gen_search_library).',
        'parameters': {   'type': 'object',
                          'properties': {   'prompt': {   'type': 'string',
                                                          'description': 'Описание '
                                                                         'стиля/настроения '
                                                                         'трека '
                                                                         '(1-2000 '
                                                                         'chars): '
                                                                         'жанр, темп, '
                                                                         'инструменты, '
                                                                         'вокал, '
                                                                         'референсы '
                                                                         "('Indie "
                                                                         'folk, '
                                                                         'melancholic, '
                                                                         "92 bpm'). "
                                                                         'Для '
                                                                         'инструментальных '
                                                                         'треков '
                                                                         'передавай '
                                                                         'is_instrumental=true.'},
                                            'lyrics': {   'type': 'string',
                                                          'description': 'Текст песни '
                                                                         'с тегами '
                                                                         '[Verse]/[Chorus]/[Bridge] '
                                                                         'и т.п. '
                                                                         'Обязателен, '
                                                                         'если '
                                                                         'is_instrumental=false. '
                                                                         'До ~3000 '
                                                                         'символов.',
                                                          'default': ''},
                                            'is_instrumental': {   'type': 'boolean',
                                                                   'description': 'true '
                                                                                  '— '
                                                                                  'без '
                                                                                  'вокала '
                                                                                  '(только '
                                                                                  'музыка). '
                                                                                  'false '
                                                                                  '(по '
                                                                                  'умолчанию) '
                                                                                  '— '
                                                                                  'трек '
                                                                                  'с '
                                                                                  'пением '
                                                                                  'по '
                                                                                  'lyrics.',
                                                                   'default': False},
                                            'mood': {   'type': 'string',
                                                        'description': 'Настроение '
                                                                       '(например '
                                                                       "'romantic', "
                                                                       "'dark', "
                                                                       "'energetic', "
                                                                       "'melancholic'). "
                                                                       'Сохраняется '
                                                                       'как тег в '
                                                                       'библиотеке.',
                                                        'default': ''},
                                            'genre': {   'type': 'string',
                                                         'description': 'Жанр '
                                                                        '(например '
                                                                        "'indie folk', "
                                                                        "'synthwave', "
                                                                        "'hip-hop'). "
                                                                        'Сохраняется '
                                                                        'как тег в '
                                                                        'библиотеке.',
                                                         'default': ''},
                                            'lang': {   'type': 'string',
                                                        'description': 'Язык вокальной '
                                                                       "части ('ru', "
                                                                       "'en', …). По "
                                                                       'умолчанию '
                                                                       'auto.',
                                                        'default': ''},
                                            'tags': {   'type': 'string',
                                                        'description': 'Доп. теги '
                                                                       'через запятую '
                                                                       'для поиска '
                                                                       "('rainy,love,sad'). "
                                                                       'Будут '
                                                                       'сохранены в '
                                                                       'библиотеке.',
                                                        'default': ''},
                                            'model': {   'type': 'string',
                                                         'description': 'Override '
                                                                        'модели: '
                                                                        "'music-3.0-free' "
                                                                        '(RPM 3, '
                                                                        'бесплатно) '
                                                                        'или '
                                                                        "'music-3.0' "
                                                                        '(RPM 120, '
                                                                        'paid). По '
                                                                        'умолчанию — '
                                                                        'free.',
                                                         'default': ''},
                                            'title': {   'type': 'string',
                                                         'description': 'Читаемое '
                                                                        'название '
                                                                        'трека для '
                                                                        'meta.json.',
                                                         'default': ''},
                                            'auto_save': {   'type': 'boolean',
                                                             'description': 'true (по '
                                                                            'умолчанию) '
                                                                            '— '
                                                                            'автоматически '
                                                                            'сохранить '
                                                                            'в '
                                                                            'библиотеку. '
                                                                            'false — '
                                                                            'только '
                                                                            'сгенерировать '
                                                                            'и вернуть '
                                                                            'audio_bytes, '
                                                                            'без '
                                                                            'записи.',
                                                             'default': True}},
                          'required': ['prompt'],
                          'additionalProperties': False},
        'signature': {   'params': [   'prompt',
                                       'lyrics',
                                       'is_instrumental',
                                       'mood',
                                       'genre',
                                       'lang',
                                       'tags',
                                       'model',
                                       'title',
                                       'auto_save'],
                         'required': ['prompt'],
                         'accepts_kwargs': True},
        'skill': ()},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'get_battery_level',
        'description': 'Получить текущий уровень заряда батареи робота в процентах.',
        'parameters': {   'type': 'object',
                          'properties': {},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': [], 'required': [], 'accepts_kwargs': False},
        'skill': ('core',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'get_current_pose',
        'description': 'Получить текущую позицию робота (x, y, theta) в системе '
                       "координат карты. Используй перед 'миссиями' чтобы запомнить "
                       "точку возврата, или когда пользователь спрашивает 'где ты?'.",
        'parameters': {   'type': 'object',
                          'properties': {},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': [], 'required': [], 'accepts_kwargs': False},
        'skill': ('navigation',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'instant',
        'name': 'get_current_time',
        'description': 'Получить текущее время и дату. Используй когда пользователь '
                       'спрашивает который час, какая дата, какой день недели, какое '
                       'время суток, сколько сейчас времени.',
        'parameters': {   'type': 'object',
                          'properties': {},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': [], 'required': [], 'accepts_kwargs': False},
        'skill': ('core',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': True,
        'destructive': False,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'get_music_state',
        'description': 'Получить текущее состояние музыкального менеджера: доступность '
                       'Renardo и SuperCollider, список активных паттернов, историю '
                       'кода паттернов.',
        'parameters': {   'type': 'object',
                          'properties': {},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': [], 'required': [], 'accepts_kwargs': False},
        'skill': ('composer', 'dj')},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'medium',
        'name': 'get_perception_context',
        'description': 'Что робот сейчас видит камерой и чувствует: лица и объекты в '
                       'кадре (детекции с уверенностью; age_s — сколько секунд назад '
                       'замечено, свежее пары секунд — видно сейчас), а также время '
                       'суток, заряд батареи, температура, движение, здоровье системы, '
                       'режим картографирования, сводка недавних событий. Вызывай на '
                       '«что ты видишь», «кто перед тобой», «видишь меня».',
        'parameters': {   'type': 'object',
                          'properties': {},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': [], 'required': [], 'accepts_kwargs': False},
        'skill': ('core',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'medium',
        'name': 'get_robot_status',
        'description': 'Получить текущий статус робота (позиция, батарея, состояние '
                       'систем).',
        'parameters': {   'type': 'object',
                          'properties': {},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': [], 'required': [], 'accepts_kwargs': False},
        'skill': ('core',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'get_sound_info',
        'description': 'Получить информацию о доступных звуковых эффектах. Используй '
                       'этот инструмент чтобы узнать какие звуки доступны, их '
                       'описание, длительность, категорию и рекомендуемое '
                       'использование. Можно запросить информацию о конкретном звуке '
                       'или получить список всех звуков определенной категории.',
        'parameters': {   'type': 'object',
                          'properties': {   'sound_name': {   'type': 'string',
                                                              'description': 'Название '
                                                                             'конкретного '
                                                                             'звука '
                                                                             'для '
                                                                             'получения '
                                                                             'подробной '
                                                                             'информации '
                                                                             '(опционально). '
                                                                             'Если не '
                                                                             'указано, '
                                                                             'вернется '
                                                                             'список '
                                                                             'всех '
                                                                             'звуков.'},
                                            'category': {   'type': 'string',
                                                            'description': 'Фильтр по '
                                                                           'категории: '
                                                                           "'base', "
                                                                           "'ui', "
                                                                           "'robot' "
                                                                           '(опционально)',
                                                            'enum': [   'base',
                                                                        'ui',
                                                                        'robot']}},
                          'required': [],
                          'additionalProperties': False},
        'signature': {   'params': ['sound_name', 'category'],
                         'required': [],
                         'accepts_kwargs': False},
        'skill': ('expression',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': False,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'list_tts_voices',
        'description': 'Список доступных голосов TTS. Без аргументов — голоса '
                       'АКТИВНОГО провайдера (см. [TTS] provider: в контексте). С '
                       'аргументом provider — голоса конкретного провайдера («какие '
                       "голоса есть на Яндексе?» → provider='yandex'). Используй когда "
                       'юзер спрашивает «какие у тебя голоса?» / «а на yandex какие?».',
        'parameters': {   'type': 'object',
                          'properties': {   'provider': {   'type': 'string',
                                                            'description': 'Опциональный '
                                                                           'TTS-провайдер '
                                                                           '(«yandex» '
                                                                           '| '
                                                                           '«minimax» '
                                                                           '| '
                                                                           '«silero»). '
                                                                           'Без '
                                                                           'аргумента '
                                                                           '— активный '
                                                                           'провайдер.',
                                                            'enum': [   'yandex',
                                                                        'minimax',
                                                                        'silero']}},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': ['provider'], 'required': [], 'accepts_kwargs': False},
        'skill': ('voice-tts',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'instant',
        'name': 'list_waypoints',
        'description': 'Получить список всех сохранённых точек (waypoints) для '
                       'навигации на текущей карте.',
        'parameters': {   'type': 'object',
                          'properties': {},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': [], 'required': [], 'accepts_kwargs': False},
        'skill': ('navigation',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'medium',
        'name': 'listen_for_response',
        'description': 'Ждать ответ пользователя через речь. Робот активирует микрофон '
                       'и будет ждать ответа (таймаут 30 секунд). ИСПОЛЬЗУЙ когда '
                       'задал вопрос пользователю или ждёшь продолжения диалога.',
        'parameters': {   'type': 'object',
                          'properties': {   'timeout_seconds': {   'type': 'integer',
                                                                   'description': 'Таймаут '
                                                                                  'ожидания '
                                                                                  'в '
                                                                                  'секундах '
                                                                                  '(по '
                                                                                  'умолчанию '
                                                                                  '30)'},
                                            'prompt_text': {   'type': 'string',
                                                               'description': 'Текст-подсказка '
                                                                              'что '
                                                                              'ждёшь '
                                                                              'от '
                                                                              'пользователя '
                                                                              '(для '
                                                                              'логирования)'}},
                          'required': [],
                          'additionalProperties': False},
        'signature': {   'params': ['timeout_seconds', 'prompt_text'],
                         'required': [],
                         'accepts_kwargs': False},
        'skill': ('core',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'long',
        'name': 'load_map',
        'description': 'Загрузить сохранённую карту и перейти в режим локализации '
                       '(навигации). Используй когда нужно переключиться на уже '
                       'исследованную локацию.',
        'parameters': {   'type': 'object',
                          'properties': {   'map_name': {   'type': 'string',
                                                            'description': 'Название '
                                                                           'карты для '
                                                                           'загрузки '
                                                                           '(например '
                                                                           "'квартира', "
                                                                           "'офис'). "
                                                                           'Если не '
                                                                           'указано — '
                                                                           'перезагружает '
                                                                           'текущую.'}},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': ['map_name'], 'required': [], 'accepts_kwargs': False},
        'skill': ('mapping',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': True,
        'destructive': False,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'lookup_melody',
        'description': 'Найти известную мелодию по имени и вернуть её ТОЧНЫЕ ноты '
                       "сырой RTTTL-строкой в data['rtttl'], НИЧЕГО не играя: "
                       'проверить, есть ли такая мелодия и как она точно называется. '
                       'Играет мелодию request_music(intent=melody, text=слова '
                       'человека) — ноты и аранжировку решает код. Имя ищи на '
                       'АНГЛИЙСКОМ или транслитом («имперский марш» → "imperial '
                       'march"). Если не нашлось — честно скажи, что не знаешь точных '
                       'нот.',
        'parameters': {   'type': 'object',
                          'properties': {   'name': {   'type': 'string',
                                                        'description': 'Название '
                                                                       'мелодии '
                                                                       '(английским '
                                                                       'или '
                                                                       'транслитом): '
                                                                       '«имперский '
                                                                       'марш» → '
                                                                       '"imperial '
                                                                       'march", '
                                                                       '«кузнечик» → '
                                                                       '"grasshopper", '
                                                                       '«happy '
                                                                       'birthday», '
                                                                       '«jingle '
                                                                       'bells»…'},
                                            'variants': {   'type': 'array',
                                                            'description': 'Дополнительные '
                                                                           'варианты '
                                                                           'названия '
                                                                           '(английским/транслитом), '
                                                                           'которые '
                                                                           'пробовать '
                                                                           'по '
                                                                           'порядку, '
                                                                           'если name '
                                                                           'не '
                                                                           'найдётся. '
                                                                           'Например '
                                                                           'name="imperial '
                                                                           'march", '
                                                                           'variants=["darth '
                                                                           'vader", '
                                                                           '"star wars '
                                                                           'theme"].',
                                                            'items': {   'type': 'string',
                                                                         'description': 'Альтернативное '
                                                                                        'написание/название '
                                                                                        'мелодии.'}}},
                          'required': ['name'],
                          'additionalProperties': False},
        'signature': {   'params': ['name', 'variants'],
                         'required': ['name'],
                         'accepts_kwargs': False},
        'skill': ('composer',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'medium',
        'name': 'memory_context',
        'description': 'Получить контекст памяти из предыдущих сессий: последние '
                       'реплики + известные факты о пользователе. Используй в начале '
                       'разговора для восстановления контекста, или чтобы напомнить '
                       'себе что знаешь о пользователе.',
        'parameters': {   'type': 'object',
                          'properties': {   'limit': {   'type': 'integer',
                                                         'description': 'Количество '
                                                                        'последних '
                                                                        'реплик для '
                                                                        'загрузки (по '
                                                                        'умолчанию '
                                                                        '10).'},
                                            'query': {   'type': 'string',
                                                         'description': 'Опциональный '
                                                                        'поисковый '
                                                                        'запрос — если '
                                                                        'задан, '
                                                                        'возвращает '
                                                                        'релевантные '
                                                                        'реплики '
                                                                        'вместо '
                                                                        'хронологических '
                                                                        'последних.'},
                                            'speaker_id': {   'type': 'string',
                                                              'description': 'Опционально: '
                                                                             'voice-biometric '
                                                                             'id '
                                                                             'текущего '
                                                                             'спикера '
                                                                             '(из '
                                                                             '<system_context>/<speaker_id>). '
                                                                             'Передавай '
                                                                             'ВСЕГДА, '
                                                                             'когда '
                                                                             'вопрос '
                                                                             'про '
                                                                             'конкретного '
                                                                             'человека '
                                                                             '("о чём '
                                                                             'мы '
                                                                             'говорили", '
                                                                             '"что ты '
                                                                             'обо мне '
                                                                             'знаешь") '
                                                                             '— иначе '
                                                                             'вернутся '
                                                                             'факты/реплики '
                                                                             'ДРУГОГО '
                                                                             'зарегистрированного '
                                                                             'пользователя.'}},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': [], 'required': [], 'accepts_kwargs': True},
        'skill': ('memory',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'medium',
        'name': 'memory_save',
        'description': 'Сохранить факт или предпочтение пользователя в долгосрочную '
                       'память. Используй после того как узнал что-то важное: имя, '
                       'предпочтение, привычку. Данные сохраняются между сессиями и '
                       'помогут лучше обслуживать пользователя в будущем.',
        'parameters': {   'type': 'object',
                          'properties': {   'fact': {   'type': 'string',
                                                        'description': 'Текст факта '
                                                                       'для '
                                                                       'сохранения. '
                                                                       'Пример: '
                                                                       "'Пользователя "
                                                                       'зовут '
                                                                       "Алексей', "
                                                                       "'Предпочитает "
                                                                       'краткие '
                                                                       "ответы'."},
                                            'category': {   'type': 'string',
                                                            'description': 'Категория '
                                                                           'факта для '
                                                                           'организации '
                                                                           'памяти.',
                                                            'enum': [   'preference',
                                                                        'habit',
                                                                        'name',
                                                                        'general']},
                                            'speaker_id': {   'type': 'string',
                                                              'description': 'Опционально: '
                                                                             'voice-biometric '
                                                                             'id '
                                                                             'текущего '
                                                                             'спикера '
                                                                             '(из '
                                                                             '<system_context>/<speaker_id>). '
                                                                             'Если '
                                                                             'передан '
                                                                             '— факт '
                                                                             'сохраняется '
                                                                             'ТОЛЬКО '
                                                                             'этому '
                                                                             'пользователю; '
                                                                             'иначе '
                                                                             'факт '
                                                                             'становится '
                                                                             'глобальным. '
                                                                             'ВСЕГДА '
                                                                             'передавай '
                                                                             'speaker_id '
                                                                             'для '
                                                                             'персональных '
                                                                             'фактов '
                                                                             '(имя, '
                                                                             'предпочтения).'}},
                          'required': ['fact'],
                          'additionalProperties': False},
        'signature': {'params': [], 'required': [], 'accepts_kwargs': True},
        'skill': ('memory',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'medium',
        'name': 'memory_search',
        'description': 'Поиск по долгосрочной памяти: сохранённые факты о пользователе '
                       '(memory_save) И история разговоров со всех сессий. Используй '
                       "когда нужно вспомнить что-то про пользователя ('что я говорил "
                       "про...', 'что ты обо мне знаешь') или сверить факт, который "
                       'только что сохранил через memory_save — включая в этом же '
                       'разговоре, сразу после сохранения. Возвращает и факты, и '
                       'фрагменты прошлых разговоров, помечая каждый результат полем '
                       "kind: 'fact' | 'turn'.",
        'parameters': {   'type': 'object',
                          'properties': {   'query': {   'type': 'string',
                                                         'description': 'Поисковый '
                                                                        'запрос на '
                                                                        'русском или '
                                                                        'английском.'},
                                            'limit': {   'type': 'integer',
                                                         'description': 'Максимальное '
                                                                        'количество '
                                                                        'результатов '
                                                                        '(по умолчанию '
                                                                        '5, максимум '
                                                                        '20).'},
                                            'speaker_id': {   'type': 'string',
                                                              'description': 'Опционально: '
                                                                             'voice-biometric '
                                                                             'id '
                                                                             'текущего '
                                                                             'спикера '
                                                                             '(из '
                                                                             '<system_context>/<speaker_id>). '
                                                                             'Передавай '
                                                                             'ВСЕГДА, '
                                                                             'когда '
                                                                             'вопрос '
                                                                             'про '
                                                                             'конкретного '
                                                                             'человека '
                                                                             '("что я '
                                                                             'люблю", '
                                                                             '"моё '
                                                                             'имя") — '
                                                                             'иначе '
                                                                             'вернутся '
                                                                             'факты/реплики '
                                                                             'ДРУГОГО '
                                                                             'зарегистрированного '
                                                                             'пользователя.'}},
                          'required': ['query'],
                          'additionalProperties': False},
        'signature': {'params': [], 'required': [], 'accepts_kwargs': True},
        'skill': ('memory',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'long',
        'name': 'move_direction',
        'description': 'Движение робота в указанном направлении. Используй для команд '
                       "'вперёд', 'назад', 'поверни налево'.",
        'parameters': {   'type': 'object',
                          'properties': {   'direction': {   'type': 'string',
                                                             'description': 'Направление '
                                                                            'движения',
                                                             'enum': [   'вперёд',
                                                                         'назад',
                                                                         'налево',
                                                                         'направо']},
                                            'distance': {   'type': 'number',
                                                            'description': 'Расстояние '
                                                                           'в метрах '
                                                                           '(по '
                                                                           'умолчанию '
                                                                           '1.0). '
                                                                           'Только для '
                                                                           'движения '
                                                                           'вперёд/назад.',
                                                            'default': 1.0}},
                          'required': ['direction'],
                          'additionalProperties': False},
        'signature': {   'params': ['direction', 'distance'],
                         'required': ['direction'],
                         'accepts_kwargs': False},
        'skill': ('navigation',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'long',
        'name': 'navigate_to_coordinates',
        'description': 'Навигация робота к произвольным координатам (x, y, theta) в '
                       'системе координат карты. Используй для возвращения на '
                       'сохранённую позицию (после get_current_pose).',
        'parameters': {   'type': 'object',
                          'properties': {   'x': {   'type': 'number',
                                                     'description': 'Координата X в '
                                                                    'метрах'},
                                            'y': {   'type': 'number',
                                                     'description': 'Координата Y в '
                                                                    'метрах'},
                                            'theta': {   'type': 'number',
                                                         'description': 'Ориентация в '
                                                                        'радианах (по '
                                                                        'умолчанию '
                                                                        '0.0)',
                                                         'default': 0.0}},
                          'required': ['x', 'y'],
                          'additionalProperties': False},
        'signature': {   'params': ['x', 'y', 'theta'],
                         'required': ['x', 'y'],
                         'accepts_kwargs': False},
        'skill': ('navigation',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'long',
        'name': 'navigate_to_waypoint',
        'description': 'Навигация робота к именованной точке (waypoint) из базы. '
                       "Используй для команд типа 'иди к кухне', 'поезжай в зал'. "
                       'Сначала проверь доступные точки через list_waypoints.',
        'parameters': {   'type': 'object',
                          'properties': {   'waypoint': {   'type': 'string',
                                                            'description': 'Название '
                                                                           'точки '
                                                                           'назначения '
                                                                           '(например '
                                                                           "'кухня', "
                                                                           "'зал')"}},
                          'required': ['waypoint'],
                          'additionalProperties': False},
        'signature': {   'params': ['waypoint'],
                         'required': ['waypoint'],
                         'accepts_kwargs': False},
        'skill': ('navigation',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'long',
        'name': 'optimize_map',
        'description': 'Оптимизировать карту после завершения картографирования: поиск '
                       'дополнительных loop closures, bundle adjustment, очистка '
                       'occupancy grid, backup.',
        'parameters': {   'type': 'object',
                          'properties': {},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': [], 'required': [], 'accepts_kwargs': False},
        'skill': ('mapping',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'instant',
        'name': 'play_animation',
        'description': 'Запустить LED анимацию на матрице робота (381 LED) на '
                       'указанное время. ОБЯЗАТЕЛЬНО используй анимации во время '
                       'разговора для визуального выражения эмоций и действий. При '
                       'рассказе историй, объяснениях, описаниях - ВСЕГДА добавляй '
                       'подходящие анимации параллельно с текстом. Доступны анимации с '
                       'эмоциями (happy, sad, angry, surprised, thinking, victory) и '
                       'другие анимации (police_lights, fire_truck, ambulance, '
                       'turn_left, turn_right). Когда показываешь анимации по запросу '
                       '- НЕ описывай что происходит в анимации, просто говори: '
                       "'Показываю анимацию <название>' или 'Есть анимация "
                       "<название>'.",
        'parameters': {   'type': 'object',
                          'properties': {   'animation': {   'type': 'string',
                                                             'description': 'Название '
                                                                            'анимации '
                                                                            'для '
                                                                            'воспроизведения',
                                                             'enum': [   'accelerating',
                                                                         'ambulance',
                                                                         'angry',
                                                                         'braking',
                                                                         'charging',
                                                                         'error',
                                                                         'fire_truck',
                                                                         'happy',
                                                                         'idle',
                                                                         'low_battery',
                                                                         'police_lights',
                                                                         'road_service',
                                                                         'sad',
                                                                         'sleep',
                                                                         'surprised',
                                                                         'talking',
                                                                         'thinking',
                                                                         'turn_left',
                                                                         'turn_right',
                                                                         'victory',
                                                                         'wakeup']},
                                            'duration': {   'type': 'number',
                                                            'description': 'Длительность '
                                                                           'анимации в '
                                                                           'секундах '
                                                                           '(рекомендуется '
                                                                           'от 2 до '
                                                                           '30, '
                                                                           'значения '
                                                                           'вне '
                                                                           'диапазона '
                                                                           'будут '
                                                                           'установлены '
                                                                           'в 2)'}},
                          'required': ['animation'],
                          'additionalProperties': False},
        'signature': {   'params': ['animation', 'duration'],
                         'required': ['animation'],
                         'accepts_kwargs': False},
        'skill': ('expression',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'instant',
        'name': 'play_sound',
        'description': 'Воспроизвести звуковой эффект. ИСПОЛЬЗУЙ АВТОМАТИЧЕСКИ для '
                       'звукового сопровождения эмоций во время разговора.\n'
                       '\n'
                       'Категории звуков:\n'
                       '- BASE (robot_*): Эмоции и реакции робота - affirm, angry, '
                       'concerned, confirm, confused, cute, happy, sigh, surprise, '
                       'thinking, talk_1-4, error, drip_*\n'
                       '- UI (ui_*): Звуки интерфейса - activate, button, bell, chime, '
                       'confirm, notification, roger, menu_click\n'
                       '- ROBOT (robot_*): Спецэффекты - alert, glitch, impact, '
                       'power_up, flyby, whoosh, bubbles, stinger, work_1-3\n'
                       '\n'
                       'Примеры использования:\n'
                       '- Подтверждение команды: robot_confirm, ui_roger, '
                       'robot_affirm\n'
                       '- Ошибка: robot_error, robot_glitch, robot_alert\n'
                       '- Обработка/размышление: robot_thinking, robot_loop, '
                       'robot_work_1\n'
                       '- Успех: robot_happy, ui_confirm, ui_chime\n'
                       '- Удивление: robot_surprise, robot_concerned',
        'parameters': {   'type': 'object',
                          'properties': {   'sound': {   'type': 'string',
                                                         'description': 'Название '
                                                                        'звукового '
                                                                        'эффекта из '
                                                                        'sound_catalog.json',
                                                         'enum': [   'robot_affirm',
                                                                     'robot_alert',
                                                                     'robot_angry',
                                                                     'robot_bubbles',
                                                                     'robot_concerned',
                                                                     'robot_confirm',
                                                                     'robot_confused',
                                                                     'robot_confused_alt',
                                                                     'robot_cute',
                                                                     'robot_drip_a1',
                                                                     'robot_drip_d4',
                                                                     'robot_drip_d5',
                                                                     'robot_drip_e4',
                                                                     'robot_error',
                                                                     'robot_fantasy',
                                                                     'robot_flyby',
                                                                     'robot_glitch',
                                                                     'robot_happy',
                                                                     'robot_impact',
                                                                     'robot_liquid',
                                                                     'robot_loop',
                                                                     'robot_power_up',
                                                                     'robot_sigh',
                                                                     'robot_stinger',
                                                                     'robot_stun',
                                                                     'robot_surprise',
                                                                     'robot_talk_1',
                                                                     'robot_talk_2',
                                                                     'robot_talk_3',
                                                                     'robot_talk_4',
                                                                     'robot_talk_beep_1',
                                                                     'robot_talk_beep_2',
                                                                     'robot_terminal',
                                                                     'robot_thinking',
                                                                     'robot_very_cute',
                                                                     'robot_whoosh',
                                                                     'robot_work_1',
                                                                     'robot_work_2',
                                                                     'robot_work_3',
                                                                     'ui_activate',
                                                                     'ui_bell',
                                                                     'ui_button',
                                                                     'ui_chime',
                                                                     'ui_confirm',
                                                                     'ui_dot',
                                                                     'ui_menu_click',
                                                                     'ui_note_e',
                                                                     'ui_notification',
                                                                     'ui_radio_start',
                                                                     'ui_random',
                                                                     'ui_roger']}},
                          'required': ['sound'],
                          'additionalProperties': False},
        'signature': {   'params': ['sound'],
                         'required': ['sound'],
                         'accepts_kwargs': False},
        'skill': ('expression',)},
    {   'llm_visible': False,
        'operator_visible': False,
        'read_only': True,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'medium',
        'name': 'read_logs',
        'description': 'Получить логи конкретной ROS2-ноды или контейнера. Используй '
                       "когда нужно посмотреть почему нода не работает ('покажи логи "
                       "stt_node', 'что в логах voice-assistant').",
        'parameters': {   'type': 'object',
                          'properties': {   'node': {   'type': 'string',
                                                        'description': 'Имя ROS2-ноды '
                                                                       'или контейнера '
                                                                       'для чтения '
                                                                       'логов.'},
                                            'source': {   'type': 'string',
                                                          'description': "'rosout' — "
                                                                         'локальные '
                                                                         'логи /rosout '
                                                                         'из '
                                                                         'health_monitor; '
                                                                         "'loki' — "
                                                                         'контейнерные '
                                                                         'логи через '
                                                                         'Loki HTTP '
                                                                         'API.',
                                                          'enum': ['rosout', 'loki']},
                                            'limit': {   'type': 'integer',
                                                         'description': 'Максимум '
                                                                        'строк '
                                                                        '(default: 20 '
                                                                        'для rosout, '
                                                                        '50 для '
                                                                        'loki).'}},
                          'required': ['node'],
                          'additionalProperties': False},
        'signature': {   'params': ['node', 'source', 'limit'],
                         'required': ['node'],
                         'accepts_kwargs': False},
        'skill': ()},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': False,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'medium',
        'name': 'register_speaker',
        'description': 'Зарегистрировать голос текущего собеседника в voice biometric '
                       'DB (resemblyzer d-vector). Вызывай когда: (1) пользователь '
                       'представился фразой типа «меня зовут X» / «моё имя X» — '
                       'извлеки имя из user_input и передай name=ИМЯ (Cyrillic, ≥2 '
                       'буквы, с заглавной); (2) хочешь узнать имя незнакомца — '
                       'передай name=null и спроси «Как вас зовут?» через speak_text. '
                       'ВАЖНО: НЕ передавай служебные слова «зовут», «имя», «меня», '
                       '«зовут-это», «зовут меня» — это шумовые токены из фразы, а не '
                       'реальные имена. Извлеки имя из контекста вручную (например, '
                       'для фразы «робот меня зовут Денис говорю» — передай '
                       'name="Денис"). Имя сохранится в БД вместе с эмбеддингом '
                       'голоса, после этого пользователя можно будет узнавать по '
                       'голосу.',
        'parameters': {   'type': 'object',
                          'properties': {   'name': {   'type': 'string',
                                                        'description': 'Имя спикера '
                                                                       'для '
                                                                       'регистрации '
                                                                       '(Cyrillic). '
                                                                       'Передай '
                                                                       'null/None если '
                                                                       'хочешь '
                                                                       'спросить имя '
                                                                       'незнакомца — '
                                                                       'робот спросит '
                                                                       '«Как вас '
                                                                       'зовут?» и ждёт '
                                                                       'ответа. Если '
                                                                       'пользователь '
                                                                       'ИСПРАВЛЯЕТ имя '
                                                                       '(«я не X, я '
                                                                       'Y») — передай '
                                                                       'name=Y и '
                                                                       'old_name=X.'},
                                            'old_name': {   'type': 'string',
                                                            'description': 'Предыдущее '
                                                                           'имя (если '
                                                                           'пользователь '
                                                                           'исправляет: '
                                                                           '«я не '
                                                                           'Эйджик, я '
                                                                           'Денчик» → '
                                                                           "old_name='Эйджик', "
                                                                           "name='Денчик'). "
                                                                           'Опусти, '
                                                                           'если '
                                                                           'пользователь '
                                                                           'представляется '
                                                                           'впервые.'}},
                          'required': [],
                          'additionalProperties': False},
        'signature': {   'params': [   'name',
                                       'old_name',
                                       'utterance_id',
                                       'self_intro_name',
                                       'intro_registered',
                                       'known_speaker_name'],
                         'required': [],
                         'accepts_kwargs': False},
        'skill': ('memory',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': False,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'request_music',
        'description': 'Поставить музыку по просьбе человека: intent=track — клубный '
                       'трек (темп, тональность и аранжировку решает код), '
                       'intent=melody — известная мелодия по названию из text. text — '
                       'слова человека дословно. Об успехе робот скажет сам, когда '
                       'музыка реально заиграет; ok=false — музыка не заиграла.',
        'parameters': {   'type': 'object',
                          'properties': {   'intent': {   'type': 'string',
                                                          'description': 'track — '
                                                                         'трек, melody '
                                                                         '— мелодия по '
                                                                         'названию',
                                                          'enum': ['track', 'melody']},
                                            'text': {   'type': 'string',
                                                        'description': 'Слова человека '
                                                                       'дословно, без '
                                                                       'пересказа'},
                                            'mood': {   'type': 'string',
                                                        'description': 'Настроение '
                                                                       'трека',
                                                        'enum': [   'bright',
                                                                    'calm',
                                                                    'dark',
                                                                    'epic',
                                                                    'groove',
                                                                    'playful']},
                                            'genre': {   'type': 'string',
                                                         'description': 'Жанр; auto — '
                                                                        'решает код',
                                                         'enum': [   'auto',
                                                                     'club',
                                                                     'classical',
                                                                     'folk']}},
                          'required': ['intent', 'text'],
                          'additionalProperties': False},
        'signature': {   'params': ['intent', 'text', 'mood', 'genre', 'turn_id'],
                         'required': [],
                         'accepts_kwargs': False},
        'skill': ('composer', 'dj')},
    {   'llm_visible': False,
        'operator_visible': False,
        'read_only': True,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'medium',
        'name': 'ros2_node_status',
        'description': 'Получить статус ROS2-нод робота (active/missing/failed). '
                       'Используй когда нужно понять, поднялась ли нода (например, '
                       "'нода stt_node не работает' или 'проверь audio_node').",
        'parameters': {   'type': 'object',
                          'properties': {   'nodes': {   'type': 'array',
                                                         'description': 'Опциональный '
                                                                        'список '
                                                                        'ROS2-нод для '
                                                                        'проверки. '
                                                                        'Если пусто — '
                                                                        'используется '
                                                                        'дефолтный '
                                                                        'список '
                                                                        'критичных нод '
                                                                        'системы.',
                                                         'items': {   'type': 'string',
                                                                      'description': 'Полное '
                                                                                     'имя '
                                                                                     'ROS2-ноды '
                                                                                     'с '
                                                                                     'ведущим '
                                                                                     'слэшем.'}}},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': ['nodes'], 'required': [], 'accepts_kwargs': False},
        'skill': ()},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'save_waypoint',
        'description': 'Сохранить текущую позицию робота как именованную точку. '
                       "Используй когда пользователь говорит 'запомни это место как "
                       "кухня', 'это зал', 'сохрани точку спальня'. Если точка с таким "
                       'именем уже есть — координаты обновятся.',
        'parameters': {   'type': 'object',
                          'properties': {   'name': {   'type': 'string',
                                                        'description': 'Название точки '
                                                                       '(например '
                                                                       "'кухня', "
                                                                       "'зал', "
                                                                       "'спальня')"}},
                          'required': ['name'],
                          'additionalProperties': False},
        'signature': {   'params': ['name'],
                         'required': ['name'],
                         'accepts_kwargs': False},
        'skill': ('navigation',)},
    {   'llm_visible': False,
        'operator_visible': True,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'instant',
        'name': 'say',
        'description': 'Произнести текст голосом от имени оператора робота. Используй, '
                       'когда оператор дал голосовую или текстовую команду произнести '
                       'что-то вслух (например, мотивирующая фраза, обращение к людям '
                       'рядом, объявление). НЕ используй для эмоциональной реплики от '
                       'лица робота — это делает speak_text. Пока say выполняется, '
                       'личность робота молчит (ADR-0028 S5).',
        'parameters': {   'type': 'object',
                          'properties': {   'text': {   'type': 'string',
                                                        'description': 'Текст для '
                                                                       'произнесения. '
                                                                       'Без ударений и '
                                                                       'SSML — '
                                                                       'операторский '
                                                                       'voice-floor '
                                                                       'сам выберет '
                                                                       'провайдера и '
                                                                       'голос. Можно '
                                                                       'использовать '
                                                                       'русский '
                                                                       'язык.'}},
                          'required': ['text'],
                          'additionalProperties': False},
        'signature': {   'params': ['text'],
                         'required': ['text'],
                         'accepts_kwargs': False},
        'skill': ()},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': False,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'search_melody',
        'description': 'Найти мелодии в RTTTL-библиотеке по названию/жанру/тегу '
                       '(английским или транслитом: «новогодние» → "christmas", «игры» '
                       '→ "game"). Возвращает до limit кандидатов с названием, '
                       'артистом и тегами. Поиск идёт по названию, исполнителю, тегам '
                       'и имени внутри формата мелодии. Чтобы СЫГРАТЬ конкретную — '
                       'вызови lookup_melody(name=...).',
        'parameters': {   'type': 'object',
                          'properties': {   'query': {   'type': 'string',
                                                         'description': 'Строка '
                                                                        'поиска: имя, '
                                                                        'артист, жанр '
                                                                        'или тег.'},
                                            'limit': {   'type': 'integer',
                                                         'description': 'Сколько '
                                                                        'кандидатов '
                                                                        'вернуть (по '
                                                                        'умолчанию '
                                                                        '20).'}},
                          'required': ['query'],
                          'additionalProperties': False},
        'signature': {   'params': ['query', 'limit'],
                         'required': ['query'],
                         'accepts_kwargs': False},
        'skill': ('composer',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': False,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'search_web',
        'description': 'Поиск в интернете через DuckDuckGo. Возвращает до N сниппетов '
                       '(title, text, url). Используй ОБЯЗАТЕЛЬНО для любых вопросов, '
                       'требующих свежих данных, которых нет в твоей тренировочной '
                       'выборке: погода («погода в Батайске сегодня»), новости («что '
                       'случилось в мире сегодня»), курсы и цены («курс доллара '
                       'сейчас», «сколько стоит iPhone 16»), спорт («счёт матча '
                       'Спартак-Зенит»), локальная информация («работает ли метро в '
                       'Москве»), факты и даты («когда день города в Ростове»). ТАКЖЕ '
                       'для музыки: ноты незнакомой мелодии («<название> ноты», '
                       '«<name> melody notes MIDI») и стиль/темп артиста или группы. '
                       'НЕ используй для: личных фактов о собеседнике '
                       '(memory_search/memory_context).',
        'parameters': {   'type': 'object',
                          'properties': {   'query': {   'type': 'string',
                                                         'description': 'Поисковый '
                                                                        'запрос на '
                                                                        'русском или '
                                                                        'английском. '
                                                                        'Будь '
                                                                        'конкретным: '
                                                                        'укажи город, '
                                                                        'дату, тему.'},
                                            'max_results': {   'type': 'integer',
                                                               'description': 'Сколько '
                                                                              'результатов '
                                                                              'вернуть '
                                                                              '(1-10, '
                                                                              'по '
                                                                              'умолчанию '
                                                                              '5).'}},
                          'required': ['query'],
                          'additionalProperties': False},
        'signature': {   'params': ['query', 'max_results'],
                         'required': ['query'],
                         'accepts_kwargs': False},
        'skill': ('composer', 'knowledge')},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': False,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'set_music_volume',
        'description': 'Громкость МУЗЫКИ (трек/DJ-сет: request_music, dj_set), а НЕ '
                       'голоса робота. Юзер просит «громче/тише/погромче/потише» и '
                       'сейчас играет музыка, или прямо говорит «музыку/трек/бит '
                       'громче» — вызывай ЭТОТ тул, а не set_volume (set_volume меняет '
                       'только голос). Музыку не перезапускает: играющий трек '
                       'продолжает играть, меняется только уровень. action: '
                       'louder/quieter — шаг ±3 dB, max — максимум, normal — стартовый '
                       'уровень, set — абсолютный уровень level 0..100 (% от '
                       'максимума). mp3 из MiniMax-библиотеки (gen_play_from_library) '
                       'этим тулом не регулируется.',
        'parameters': {   'type': 'object',
                          'properties': {   'action': {   'type': 'string',
                                                          'description': 'louder — '
                                                                         'громче на '
                                                                         'шаг, quieter '
                                                                         '— тише на '
                                                                         'шаг, max — '
                                                                         'на максимум, '
                                                                         'normal — '
                                                                         'стартовый '
                                                                         'уровень, set '
                                                                         '— выставить '
                                                                         'level',
                                                          'enum': [   'louder',
                                                                      'quieter',
                                                                      'max',
                                                                      'normal',
                                                                      'set']},
                                            'level': {   'type': 'integer',
                                                         'description': 'Только для '
                                                                        'action=set: '
                                                                        'уровень '
                                                                        'музыки в '
                                                                        'процентах от '
                                                                        'максимума, '
                                                                        '0..100 '
                                                                        '(значения вне '
                                                                        'диапазона '
                                                                        'обрезаются).'}},
                          'required': ['action'],
                          'additionalProperties': False},
        'signature': {   'params': ['action', 'level'],
                         'required': ['action'],
                         'accepts_kwargs': False},
        'skill': ('composer', 'dj', 'voice-tts')},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'set_pitch',
        'description': "Установить высоту голоса робота. Используй для команд 'говори "
                       "выше', 'говори ниже'.",
        'parameters': {   'type': 'object',
                          'properties': {   'action': {   'type': 'string',
                                                          'description': 'Действие с '
                                                                         'высотой '
                                                                         'голоса',
                                                          'enum': [   'higher',
                                                                      'lower',
                                                                      'normal']}},
                          'required': ['action'],
                          'additionalProperties': False},
        'signature': {   'params': ['action'],
                         'required': ['action'],
                         'accepts_kwargs': False},
        'skill': ('voice-tts',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'set_speed',
        'description': "Установить скорость речи робота. Используй для команд 'говори "
                       "быстрее', 'говори медленнее'.",
        'parameters': {   'type': 'object',
                          'properties': {   'action': {   'type': 'string',
                                                          'description': 'Действие со '
                                                                         'скоростью '
                                                                         'речи',
                                                          'enum': [   'faster',
                                                                      'slower',
                                                                      'normal']}},
                          'required': ['action'],
                          'additionalProperties': False},
        'signature': {   'params': ['action'],
                         'required': ['action'],
                         'accepts_kwargs': False},
        'skill': ('voice-tts',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': False,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'set_tts_provider',
        'description': 'Переключить активного TTS-провайдера (yandex ↔ minimax ↔ '
                       'silero). Используй когда юзер просит СМЕНИТЬ ПРОВАЙДЕРА '
                       'целиком: «давай говорить Яндексом», «переключись на MiniMax», '
                       '«через Silero — без интернета». Голос будет дефолтным для '
                       'нового провайдера. Для смены голоса внутри текущего провайдера '
                       '— используй set_voice(voice=...).',
        'parameters': {   'type': 'object',
                          'properties': {   'provider': {   'type': 'string',
                                                            'description': 'Имя '
                                                                           'TTS-провайдера: '
                                                                           '«yandex» | '
                                                                           '«minimax» '
                                                                           '| '
                                                                           '«silero».',
                                                            'enum': [   'yandex',
                                                                        'minimax',
                                                                        'silero']}},
                          'required': ['provider'],
                          'additionalProperties': False},
        'signature': {   'params': ['provider'],
                         'required': ['provider'],
                         'accepts_kwargs': False},
        'skill': ('voice-tts',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': False,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'set_voice',
        'description': 'Установить голос TTS на диалог (персистентно, пока не сменят). '
                       'Используй когда: (1) пользователь просит «говори голосом X» — '
                       'передай voice=X; (2) хочешь рассказывать историю от разных лиц '
                       '(старик → zahar, девушка → alena и т.п.); (3) хочешь вернуться '
                       'к дефолтному голосу — передай voice=<default_voice>; (4) юзер '
                       'просит голос с КОНКРЕТНОГО провайдера («Яндекс Артём», «Yandex '
                       "anton») — передай voice=... И provider='yandex'. Доступные "
                       'голоса перечислены в контексте: [TTS] voices=... После '
                       'set_voice следующий speak_text без voice= говорит '
                       'установленным голосом.',
        'parameters': {   'type': 'object',
                          'properties': {   'voice': {   'type': 'string',
                                                         'description': 'Имя голоса '
                                                                        'для установки '
                                                                        '(из списка '
                                                                        '[TTS] '
                                                                        'voices=...). '
                                                                        'Например: '
                                                                        'alena, zahar, '
                                                                        'jane '
                                                                        '(Yandex); '
                                                                        'Russian_ReliableMan, '
                                                                        'Russian_BrightHeroine '
                                                                        '(MiniMax); '
                                                                        'aidar, baya '
                                                                        '(Silero).'},
                                            'provider': {   'type': 'string',
                                                            'description': 'Опциональный '
                                                                           'TTS-провайдер '
                                                                           '(«yandex» '
                                                                           '| '
                                                                           '«minimax» '
                                                                           '| '
                                                                           '«silero»). '
                                                                           'Если задан '
                                                                           '— бот '
                                                                           'СНАЧАЛА '
                                                                           'переключает '
                                                                           'провайдера, '
                                                                           'потом '
                                                                           'валидирует '
                                                                           'voice '
                                                                           'против '
                                                                           'голосов '
                                                                           'НОВОГО '
                                                                           'провайдера. '
                                                                           'Используй '
                                                                           'когда юзер '
                                                                           'просит '
                                                                           'голос по '
                                                                           'имени, '
                                                                           'привязанному '
                                                                           'к '
                                                                           'конкретному '
                                                                           'провайдеру '
                                                                           '(«Яндекс '
                                                                           'Артём», '
                                                                           '«Yandex '
                                                                           'anton»), а '
                                                                           'в [TTS] '
                                                                           'provider: '
                                                                           'сейчас '
                                                                           'другой '
                                                                           'провайдер.',
                                                            'enum': [   'yandex',
                                                                        'minimax',
                                                                        'silero']}},
                          'required': ['voice'],
                          'additionalProperties': False},
        'signature': {   'params': ['voice', 'provider'],
                         'required': ['voice'],
                         'accepts_kwargs': False},
        'skill': ('voice-tts',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'set_volume',
        'description': 'Громкость ГОЛОСА робота (речь TTS), НЕ музыки. Используй для '
                       "'говори громче', 'тише говори', 'максимальная громкость "
                       "голоса', а также для 'громче/тише', когда музыка НЕ играет. "
                       'Если играет трек/бит/DJ-сет и юзер просит громче/тише музыку — '
                       'вызывай set_music_volume, а не этот тул.',
        'parameters': {   'type': 'object',
                          'properties': {   'action': {   'type': 'string',
                                                          'description': 'Действие с '
                                                                         'громкостью',
                                                          'enum': [   'louder',
                                                                      'quieter',
                                                                      'max',
                                                                      'normal']}},
                          'required': ['action'],
                          'additionalProperties': False},
        'signature': {   'params': ['action'],
                         'required': ['action'],
                         'accepts_kwargs': False},
        'skill': ('voice-tts',)},
    {   'llm_visible': False,
        'operator_visible': False,
        'read_only': True,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'medium',
        'name': 'show_metrics',
        'description': 'Показать телеметрию робота на боковом экране TARS 2 (Captain '
                       'Bridge) и получить её значения. Тул выполняет запрос в '
                       'Prometheus (метрики) или Loki (логи) и возвращает последние '
                       'значения — отвечай оператору ИМИ, а не фразой «открыл панель». '
                       'Используй когда просят показать графики, метрики, логи, CPU, '
                       "память, latency, ошибки ('покажи дрейф CPU', 'открой ошибки "
                       "stt_node'). Рабочие запросы: "
                       "'rate(process_cpu_seconds_total[5m])' — CPU, "
                       "'process_resident_memory_bytes' — RAM, 'up' — живость "
                       "экспортеров, 'rate(voice_llm_request_duration_seconds_sum[5m]) "
                       "/ rate(voice_llm_request_duration_seconds_count[5m])' — "
                       "задержка LLM. Простые слова ('cpu', 'память', 'latency') тоже "
                       'принимаются — они резолвятся по живому каталогу метрик.',
        'parameters': {   'type': 'object',
                          'properties': {   'query': {   'type': 'string',
                                                         'description': 'PromQL-выражение '
                                                                        '(напр. '
                                                                        "'rate(process_cpu_seconds_total[5m])'), "
                                                                        'LogQL-запрос '
                                                                        '(напр. '
                                                                        '\'{job="voice"}\') '
                                                                        'или простое '
                                                                        'слово-метрика '
                                                                        "('cpu', "
                                                                        "'память', "
                                                                        "'latency'). "
                                                                        'Обязательный.'},
                                            'datasource': {   'type': 'string',
                                                              'description': 'Источник '
                                                                             'телеметрии: '
                                                                             "'prometheus' "
                                                                             '(default) '
                                                                             'или '
                                                                             "'loki'.",
                                                              'enum': [   'prometheus',
                                                                          'loki'],
                                                              'default': 'prometheus'}},
                          'required': ['query'],
                          'additionalProperties': False},
        'signature': {   'params': ['query', 'datasource'],
                         'required': ['query'],
                         'accepts_kwargs': False},
        'skill': ()},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'medium',
        'name': 'speak_text',
        'description': 'Произнести текст голосом через TTS. ИСПОЛЬЗУЙ ЭТО вместо '
                       'возврата JSON с SSML. ОБЯЗАТЕЛЬНО указывай animation - это '
                       'покажет соответствующую анимацию на LED матрице робота (happy, '
                       'sad, police_lights, и т.д.). Можешь вызвать несколько раз для '
                       'разных фраз, делать паузы между ними через play_sound или '
                       'play_animation.',
        'parameters': {   'type': 'object',
                          'properties': {   'text': {   'type': 'string',
                                                        'description': 'Текст для '
                                                                       'произнесения. '
                                                                       'Можно '
                                                                       'использовать '
                                                                       'русские '
                                                                       'ударения (+ '
                                                                       'после '
                                                                       'гласной).'},
                                            'voice': {   'type': 'string',
                                                         'description': 'Голос TTS для '
                                                                        'этой реплики '
                                                                        '(опционально, '
                                                                        'issue #1219). '
                                                                        'Имя голоса '
                                                                        'активного '
                                                                        'провайдера, '
                                                                        'напр. '
                                                                        'alena/zahar '
                                                                        '(Yandex), '
                                                                        'Russian_ReliableMan/Russian_BrightHeroine '
                                                                        '(MiniMax), '
                                                                        'aidar/baya '
                                                                        '(Silero). '
                                                                        'Если голос не '
                                                                        'указан — '
                                                                        'используется '
                                                                        'голос, '
                                                                        'установленный '
                                                                        'set_voice, '
                                                                        'иначе '
                                                                        'дефолтный '
                                                                        'голос '
                                                                        'провайдера. '
                                                                        'Неизвестный/недоступный '
                                                                        'голос '
                                                                        'заменяется на '
                                                                        'дефолтный, '
                                                                        'фактический '
                                                                        'голос придёт '
                                                                        'в voice_used '
                                                                        'результата.'},
                                            'animation': {   'type': 'string',
                                                             'description': 'Анимация '
                                                                            'для '
                                                                            'отображения '
                                                                            'на LED '
                                                                            'матрице '
                                                                            'во время '
                                                                            'речи. '
                                                                            'Выбирай '
                                                                            'подходящую '
                                                                            'анимацию '
                                                                            'для '
                                                                            'контекста '
                                                                            '(эмоциональные: '
                                                                            'happy, '
                                                                            'sad, '
                                                                            'angry, '
                                                                            'surprised; '
                                                                            'специальные: '
                                                                            'police_lights, '
                                                                            'fire_truck, '
                                                                            'thinking, '
                                                                            'и т.д.). '
                                                                            'Псевдонимы '
                                                                            'нормализуются: '
                                                                            'neutral→idle, '
                                                                            'excited→happy, '
                                                                            'confused→thinking, '
                                                                            'talk→talking. '
                                                                            'Если '
                                                                            'указано '
                                                                            'неизвестное '
                                                                            'значение '
                                                                            '— будет '
                                                                            'warning в '
                                                                            'лог и '
                                                                            'анимация '
                                                                            'останется '
                                                                            'без '
                                                                            'изменений.',
                                                             'enum': [   'accelerating',
                                                                         'ambulance',
                                                                         'angry',
                                                                         'braking',
                                                                         'charging',
                                                                         'error',
                                                                         'fire_truck',
                                                                         'happy',
                                                                         'idle',
                                                                         'low_battery',
                                                                         'police_lights',
                                                                         'road_service',
                                                                         'sad',
                                                                         'sleep',
                                                                         'surprised',
                                                                         'talking',
                                                                         'thinking',
                                                                         'turn_left',
                                                                         'turn_right',
                                                                         'victory',
                                                                         'wakeup']}},
                          'required': ['text'],
                          'additionalProperties': False},
        'signature': {   'params': ['text', 'animation', 'voice'],
                         'required': ['text'],
                         'accepts_kwargs': False},
        'skill': ('core',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'medium',
        'name': 'start_mapping',
        'description': 'Начать физическое сканирование и построение карты помещения '
                       '(SLAM). Вызывай ТОЛЬКО если пользователь явно просит '
                       "СКАНИРОВАТЬ, КАРТИРОВАТЬ или СТРОИТЬ КАРТУ — например: 'начни "
                       "картографирование', 'построй карту', 'давай сканируем "
                       "квартиру', 'старт маппинга'. НЕ вызывай если пользователь: "
                       'играет в ролевую игру, просит провести экскурсию, упоминает '
                       'место в разговоре, просит рассказать о чём-то, даёт тебе роль '
                       'или персонажа. Если передан map_name — new_location '
                       'автоматически True (новая карта с нуля). Если говорит '
                       "'продолжить картирование' — передай new_location=false.",
        'parameters': {   'type': 'object',
                          'properties': {   'map_name': {   'type': 'string',
                                                            'description': 'Название '
                                                                           'новой '
                                                                           'карты '
                                                                           '(например '
                                                                           "'квартира', "
                                                                           "'офис'). "
                                                                           'Если '
                                                                           'передан — '
                                                                           'автоматически '
                                                                           'включает '
                                                                           'new_location=True.'},
                                            'new_location': {   'type': 'boolean',
                                                                'description': 'True — '
                                                                               'стереть '
                                                                               'базу и '
                                                                               'начать '
                                                                               'с '
                                                                               'нуля. '
                                                                               'False '
                                                                               '— '
                                                                               'продолжить '
                                                                               'текущую '
                                                                               'карту. '
                                                                               'Если '
                                                                               'не '
                                                                               'указан '
                                                                               '— True '
                                                                               'когда '
                                                                               'передан '
                                                                               'map_name, '
                                                                               'иначе '
                                                                               'False.'}},
                          'required': [],
                          'additionalProperties': False},
        'signature': {   'params': ['map_name', 'new_location'],
                         'required': [],
                         'accepts_kwargs': False},
        'skill': ('mapping',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': False,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'stop_music',
        'description': 'Остановить музыкальный паттерн по имени или всю музыку. Если '
                       'указан pattern_name — остановится только этот паттерн. Если '
                       "pattern_name не указан или равен 'all' — остановится вся "
                       'музыка (Clock.clear()).',
        'parameters': {   'type': 'object',
                          'properties': {   'pattern_name': {   'type': 'string',
                                                                'description': 'Имя '
                                                                               'паттерна '
                                                                               'для '
                                                                               'остановки '
                                                                               '(например: '
                                                                               "'p1', "
                                                                               "'bass'). "
                                                                               'Передай '
                                                                               "'all' "
                                                                               'или '
                                                                               'оставь '
                                                                               'пустым '
                                                                               'для '
                                                                               'остановки '
                                                                               'всей '
                                                                               'музыки.'}},
                          'required': [],
                          'additionalProperties': False},
        'signature': {   'params': ['pattern_name'],
                         'required': [],
                         'accepts_kwargs': False},
        'skill': ('composer', 'dj', 'player')},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'stop_navigation',
        'description': 'Остановить текущую навигацию робота. Используй для команд '
                       "'стоп', 'остановись', 'отмени движение'.",
        'parameters': {   'type': 'object',
                          'properties': {},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': [], 'required': [], 'accepts_kwargs': False},
        'skill': ('navigation',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': False,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'name': 'task_delta',
        'description': 'Изменить PENDING-сегменты активного многочастного выступления '
                       '(песня/сказка/стих) без перезапуска с начала. Используй когда '
                       'в контексте есть [SEGMENT PLAN] с REWRITEABLE_SEGMENTS и '
                       'пользователь попросил что-то добавить/изменить/убрать по ходу '
                       'исполнения. Операции: rewrite(seg_idx, args) — переписать '
                       'сегмент; replace(seg_idx, args) — то же; append(args) — '
                       'добавить новый сегмент в конец; drop(seg_idx) — убрать '
                       'сегмент. ACTIVE-сегменты (уже играют) трогать нельзя — только '
                       'те, что перечислены в REWRITEABLE_SEGMENTS.',
        'parameters': {   'type': 'object',
                          'properties': {   'group_id': {   'type': 'string',
                                                            'description': 'Значение '
                                                                           'GROUP_ID '
                                                                           'из блока '
                                                                           '[SEGMENT '
                                                                           'PLAN] — '
                                                                           'скопируй '
                                                                           'его как '
                                                                           'есть '
                                                                           '(32-символьная '
                                                                           'hex-строка). '
                                                                           'Это НЕ '
                                                                           'метка '
                                                                           'сегмента '
                                                                           '(seg_0, '
                                                                           'seg_1) и '
                                                                           'не '
                                                                           'task_id.'},
                                            'ops': {   'type': 'array',
                                                       'description': 'Список '
                                                                      'операций. '
                                                                      'Каждая — объект '
                                                                      '{"kind": '
                                                                      '"rewrite|replace|append|drop", '
                                                                      '"seg_idx": int, '
                                                                      '"args": {...}}. '
                                                                      '"append" не '
                                                                      'указывает '
                                                                      'seg_idx; "drop" '
                                                                      'не указывает '
                                                                      'args.',
                                                       'items': {   'type': 'object',
                                                                    'description': 'Одна '
                                                                                   'операция '
                                                                                   'дельты.'}}},
                          'required': ['group_id', 'ops'],
                          'additionalProperties': False},
        'signature': {   'params': ['group_id', 'ops'],
                         'required': [],
                         'accepts_kwargs': False},
        'skill': ('scheduler',)})
