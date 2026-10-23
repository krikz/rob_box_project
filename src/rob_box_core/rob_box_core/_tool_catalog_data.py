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
        'event_source': 'inproc:melody_library',
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
        'event_source': 'inproc:waypoint_store',
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
        'event_source': '',
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
        'event_source': 'srv:/rtabmap/rtabmap/set_mode_mapping',
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
        'event_source': 'inproc:waypoint_store',
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
        'event_source': '/voice/music/event',
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
                                                          'maximum': 60},
                                            'styles': {   'type': 'array',
                                                          'description': 'Стили по '
                                                                         'трекам, если '
                                                                         'человек '
                                                                         'назвал '
                                                                         'несколько, — '
                                                                         'по порядку '
                                                                         'его слов',
                                                          'items': {   'type': 'string',
                                                                       'description': 'стиль '
                                                                                      'трека',
                                                                       'enum': [   'club',
                                                                                   'rave',
                                                                                   'synthwave',
                                                                                   'chiptune',
                                                                                   'breaks',
                                                                                   'dnb',
                                                                                   'lofi',
                                                                                   'jazz',
                                                                                   'rock',
                                                                                   'grunge',
                                                                                   'metal',
                                                                                   'blues',
                                                                                   'chanson',
                                                                                   'orchestral',
                                                                                   'acoustic']}},
                                            'minutes': {   'type': 'integer',
                                                           'description': 'Сколько '
                                                                          'минут '
                                                                          'музыки — '
                                                                          'только если '
                                                                          'человек '
                                                                          'назвал '
                                                                          'время',
                                                           'minimum': 1,
                                                           'maximum': 180}},
                          'required': ['action'],
                          'additionalProperties': False},
        'signature': {   'params': [   'action',
                                       'theme',
                                       'persona',
                                       'style',
                                       'tracks',
                                       'styles',
                                       'minutes',
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
        'event_source': '',
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
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': True,
        'destructive': False,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'event_source': '',
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
        'event_source': 'srv:/rtabmap/rtabmap/set_mode_localization',
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
        'destructive': False,
        'idempotent': False,
        'starts_music': True,
        'execution_type': 'fast',
        'event_source': '/voice/music/event',
        'name': 'generate_music',
        'description': 'Просьба сгенерировать музыку (Suno): kind=instrumental — без '
                       'слов, song — песня, robot_song — песня голосом робота; style — '
                       'жанр, если назван. Это просьба, а не приказ: что играть (новую '
                       'генерацию, сочинённое раньше или своё) и что сказать — решает '
                       'робот сам по словам человека; ok=false — честный отказ, его '
                       'фразу робот скажет сам. «Спой / сочини песню про X» — '
                       'kind=robot_song, theme=слова человека (не music_library); слов '
                       'песни не сочиняй и не читай через speak_text.',
        'parameters': {   'type': 'object',
                          'properties': {   'kind': {   'type': 'string',
                                                        'description': 'instrumental, '
                                                                       'song или '
                                                                       'robot_song',
                                                        'enum': [   'instrumental',
                                                                    'song',
                                                                    'robot_song']},
                                            'style': {   'type': 'string',
                                                         'description': 'Жанр, если '
                                                                        'человек его '
                                                                        'назвал',
                                                         'enum': [   '',
                                                                     'club',
                                                                     'rave',
                                                                     'synthwave',
                                                                     'chiptune',
                                                                     'breaks',
                                                                     'dnb',
                                                                     'lofi',
                                                                     'jazz',
                                                                     'rock',
                                                                     'grunge',
                                                                     'metal',
                                                                     'blues',
                                                                     'chanson',
                                                                     'orchestral',
                                                                     'acoustic']},
                                            'theme': {   'type': 'string',
                                                         'description': 'Тема словами '
                                                                        'человека'}},
                          'required': ['kind'],
                          'additionalProperties': False},
        'signature': {   'params': ['kind', 'style', 'theme', 'turn_id', 'heard_text'],
                         'required': [],
                         'accepts_kwargs': False},
        'skill': ('composer', 'player')},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'event_source': '',
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
        'event_source': '',
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
        'event_source': '',
        'name': 'get_current_time',
        'description': 'Получить текущее время и дату. Используй когда пользователь '
                       'спрашивает который час, какая дата, какой день недели, какое '
                       'время суток, сколько сейчас времени. Поле formatted_time уже '
                       'написано русской прописью — озвучивай его дословно, время не '
                       'выдумывай.',
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
        'event_source': '',
        'name': 'get_music_state',
        'description': 'Что сейчас играет: диджей-сет (трек N из M, мелодия), трек или '
                       'пьеса (название, источник, темп, инструмент, сколько играет и '
                       'сколько осталось) или «ничего». Отвечай фразой из message.',
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
        'event_source': '',
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
        'event_source': '',
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
        'event_source': '',
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
        'event_source': '',
        'name': 'list_tts_voices',
        'description': 'Голоса TTS. Без аргументов — обзор: провайдеры, сколько '
                       'голосов у каждого и по паре характерных (отвечай кратко, не '
                       'перечисляй всё). С аргументом provider — полный список голосов '
                       'конкретного провайдера («какие голоса есть на Яндексе?» → '
                       "provider='yandex'). Используй когда юзер спрашивает «какие у "
                       'тебя голоса?» / «а на yandex какие?».',
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
        'event_source': '',
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
        'event_source': '',
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
        'event_source': 'srv:/rtabmap/rtabmap/load_database',
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
        'event_source': '',
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
        'event_source': '',
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
        'event_source': 'inproc:voice_memory',
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
        'event_source': '',
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
                                                                        'английском: '
                                                                        '1–2 ключевых '
                                                                        'слова в '
                                                                        'начальной '
                                                                        'форме, без '
                                                                        'перечислений '
                                                                        'и описаний.'},
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
        'event_source': 'action:/navigate_to_pose',
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
        'execution_type': 'fast',
        'event_source': '/voice/music/event',
        'name': 'music_library',
        'description': 'Библиотека сгенерированной музыки робота '
                       '(/data/music_library). action: list — «что у тебя есть '
                       'сгенерированного»; search — найти трек по слову (query); info '
                       '— подробности одного трека; play — включить трек по track_id '
                       '(стоп — stop_music); rate — «этот трек нравится / не нравится» '
                       '(verdict like / dislike; трек, который не нравится, больше не '
                       'включается, но не удаляется); update — переименовать, теги, '
                       'настроение, жанр, заметка; delete — удалить трек насовсем. '
                       'track_id бери из list / search, не выдумывай. Ответ содержит '
                       'фразу, которую надо сказать.',
        'parameters': {   'type': 'object',
                          'properties': {   'action': {   'type': 'string',
                                                          'description': 'Что сделать '
                                                                         'с '
                                                                         'библиотекой.',
                                                          'enum': [   'list',
                                                                      'search',
                                                                      'info',
                                                                      'play',
                                                                      'rate',
                                                                      'update',
                                                                      'delete']},
                                            'track_id': {   'type': 'string',
                                                            'description': 'UUID трека '
                                                                           'из list / '
                                                                           'search '
                                                                           '(info, '
                                                                           'play, '
                                                                           'update, '
                                                                           'delete; у '
                                                                           'rate — по '
                                                                           'умолчанию '
                                                                           'играющий '
                                                                           'сейчас или '
                                                                           'последний '
                                                                           'игравший).',
                                                            'default': ''},
                                            'query': {   'type': 'string',
                                                         'description': 'Слово поиска '
                                                                        'для search '
                                                                        '(1-200 '
                                                                        'символов).',
                                                         'default': ''},
                                            'verdict': {   'type': 'string',
                                                           'description': 'Для rate: '
                                                                          'like — трек '
                                                                          'нравится, '
                                                                          'dislike — '
                                                                          'не '
                                                                          'нравится.',
                                                           'enum': [   '',
                                                                       'like',
                                                                       'dislike'],
                                                           'default': ''},
                                            'limit': {   'type': 'integer',
                                                         'description': 'Для list / '
                                                                        'search: '
                                                                        'максимум '
                                                                        'треков (до '
                                                                        '100).',
                                                         'default': 20},
                                            'sort_by': {   'type': 'string',
                                                           'description': 'Для list: '
                                                                          'recent — '
                                                                          'новые, '
                                                                          'popular — '
                                                                          'самые '
                                                                          'играемые, '
                                                                          'rating — '
                                                                          'лучшие.',
                                                           'enum': [   'recent',
                                                                       'popular',
                                                                       'rating'],
                                                           'default': 'recent'},
                                            'title': {   'type': 'string',
                                                         'description': 'Для update: '
                                                                        'новое '
                                                                        'название '
                                                                        '(пусто — не '
                                                                        'менять).',
                                                         'default': ''},
                                            'tags': {   'type': 'string',
                                                        'description': 'Для update: '
                                                                       'теги через '
                                                                       'запятую '
                                                                       '(заменяют '
                                                                       'прежние).',
                                                        'default': ''},
                                            'mood': {   'type': 'string',
                                                        'description': 'Для update: '
                                                                       'настроение.',
                                                        'default': ''},
                                            'genre': {   'type': 'string',
                                                         'description': 'Для update: '
                                                                        'жанр.',
                                                         'default': ''},
                                            'notes': {   'type': 'string',
                                                         'description': 'Для update: '
                                                                        'заметка.',
                                                         'default': ''}},
                          'required': ['action'],
                          'additionalProperties': False},
        'signature': {   'params': ['action', 'track_id'],
                         'required': [],
                         'accepts_kwargs': True},
        'skill': ('player',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'long',
        'event_source': 'action:/navigate_to_pose',
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
        'event_source': 'action:/navigate_to_pose',
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
        'event_source': 'srv:/rtabmap/rtabmap/detect_more_loop_closures',
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
        'event_source': '/voice/expression/event',
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
                                                                            'воспроизведения '
                                                                            '(можно '
                                                                            'вместе с '
                                                                            'face)',
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
                                                                           'в 2)'},
                                            'face': {   'type': 'object',
                                                        'description': 'Слоты лица; '
                                                                       'всё, что не '
                                                                       'задано, '
                                                                       'берётся из '
                                                                       'текущей сцены. '
                                                                       'left/right - '
                                                                       'со стороны '
                                                                       'человека '
                                                                       'напротив '
                                                                       'робота. Только '
                                                                       'значения из '
                                                                       'списков, '
                                                                       'свободных '
                                                                       'цветов и форм '
                                                                       'нет. bg - фон '
                                                                       'под глазами и '
                                                                       'ртом; fx - '
                                                                       'разовый '
                                                                       'импульс в '
                                                                       'начале сцены '
                                                                       '(slash - удар '
                                                                       'катаной, '
                                                                       'glitch - сбой, '
                                                                       'boot - '
                                                                       'включение, '
                                                                       'power_down - '
                                                                       'выключение); '
                                                                       'feat=loading_eyes '
                                                                       '- спиннеры '
                                                                       'вместо глаз '
                                                                       '(жду, думаю).',
                                                        'properties': {   'eyes': {   'type': 'string',
                                                                                      'description': 'Слот '
                                                                                                     'лица '
                                                                                                     '«eyes»',
                                                                                      'enum': [   'open',
                                                                                                  'half',
                                                                                                  'squint',
                                                                                                  'wide',
                                                                                                  'closed',
                                                                                                  'shades',
                                                                                                  'x']},
                                                                          'gaze': {   'type': 'string',
                                                                                      'description': 'Слот '
                                                                                                     'лица '
                                                                                                     '«gaze»',
                                                                                      'enum': [   'speaker',
                                                                                                  'left',
                                                                                                  'right',
                                                                                                  'up',
                                                                                                  'down',
                                                                                                  'wander']},
                                                                          'blink': {   'type': 'string',
                                                                                       'description': 'Слот '
                                                                                                      'лица '
                                                                                                      '«blink»',
                                                                                       'enum': [   'normal',
                                                                                                   'rapid',
                                                                                                   'none',
                                                                                                   'wink_left',
                                                                                                   'wink_right']},
                                                                          'mouth': {   'type': 'string',
                                                                                       'description': 'Слот '
                                                                                                      'лица '
                                                                                                      '«mouth»',
                                                                                       'enum': [   'grin',
                                                                                                   'frown',
                                                                                                   'flat',
                                                                                                   'o',
                                                                                                   'teeth',
                                                                                                   'eq',
                                                                                                   'talk']},
                                                                          'tint': {   'type': 'string',
                                                                                      'description': 'Слот '
                                                                                                     'лица '
                                                                                                     '«tint»',
                                                                                      'enum': [   'warm',
                                                                                                  'cold',
                                                                                                  'red',
                                                                                                  'green',
                                                                                                  'blue',
                                                                                                  'purple',
                                                                                                  'rainbow']},
                                                                          'tempo': {   'type': 'string',
                                                                                       'description': 'Слот '
                                                                                                      'лица '
                                                                                                      '«tempo»',
                                                                                       'enum': [   'slow',
                                                                                                   'normal',
                                                                                                   'fast']},
                                                                          'rear': {   'type': 'string',
                                                                                      'description': 'Слот '
                                                                                                     'лица '
                                                                                                     '«rear»',
                                                                                      'enum': [   'off',
                                                                                                  'markers',
                                                                                                  'hazard',
                                                                                                  'party',
                                                                                                  'new_year',
                                                                                                  'halloween',
                                                                                                  'heart',
                                                                                                  'rocket']},
                                                                          'bg': {   'type': 'string',
                                                                                    'description': 'Слот '
                                                                                                   'лица '
                                                                                                   '«bg»',
                                                                                    'enum': [   'none',
                                                                                                'rainbow',
                                                                                                'fire',
                                                                                                'matrix']},
                                                                          'fx': {   'type': 'string',
                                                                                    'description': 'Слот '
                                                                                                   'лица '
                                                                                                   '«fx»',
                                                                                    'enum': [   'slash',
                                                                                                'glitch',
                                                                                                'boot',
                                                                                                'power_down']},
                                                                          'feat': {   'type': 'string',
                                                                                      'description': 'Слот '
                                                                                                     'лица '
                                                                                                     '«feat»',
                                                                                      'enum': [   'loading_eyes']},
                                                                          'duration_s': {   'type': 'number',
                                                                                            'description': 'Сколько '
                                                                                                           'держать '
                                                                                                           'сцену, '
                                                                                                           'секунды',
                                                                                            'minimum': 0,
                                                                                            'maximum': 30.0}},
                                                        'required': [],
                                                        'additionalProperties': False}},
                          'required': [],
                          'additionalProperties': False},
        'signature': {   'params': ['animation', 'duration', 'face'],
                         'required': [],
                         'accepts_kwargs': False},
        'skill': ('expression',)},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'event_source': '/voice/sound/event',
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
        'event_source': '',
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
        'event_source': '/voice/speaker/result',
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
                       'name="Денис"). Ответ тула — исход: голос записан, не записан '
                       '(с причиной) или ответа не дождались.',
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
        'event_source': '/voice/music/event',
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
                                                                     'folk']},
                                            'min_seconds': {   'type': 'number',
                                                               'description': 'Служебный '
                                                                              'аргумент '
                                                                              'кода '
                                                                              'выступления, '
                                                                              'не '
                                                                              'заполняй'}},
                          'required': ['intent', 'text'],
                          'additionalProperties': False},
        'signature': {   'params': [   'intent',
                                       'text',
                                       'mood',
                                       'genre',
                                       'turn_id',
                                       'heard_text',
                                       'min_seconds'],
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
        'event_source': '',
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
        'event_source': 'inproc:waypoint_store',
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
        'execution_type': 'medium',
        'event_source': '/voice/speech/event',
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
    {   'llm_visible': False,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'medium',
        'event_source': '/voice/speech/event',
        'name': 'say_aloud',
        'telegram_visible': True,
        'description': 'Озвучить текст динамиками робота, когда человек в чате просит '
                       'рассказать, прочитать или сказать вслух (например, сочинённую '
                       'историю). Текст чата ты пишешь в ответе, а сюда передай ту же '
                       'реплику целиком. Итог «Озвучил» в чат пишет код по событию '
                       'динамиков, не ты.',
        'parameters': {   'type': 'object',
                          'properties': {   'text': {   'type': 'string',
                                                        'description': 'Что '
                                                                       'произнести, '
                                                                       'русским '
                                                                       'текстом без '
                                                                       'SSML.'},
                                            'voice': {   'type': 'string',
                                                         'description': 'Голос разовой '
                                                                        'реплики; '
                                                                        'пусто - '
                                                                        'текущий голос '
                                                                        'робота.'}},
                          'required': ['text'],
                          'additionalProperties': False},
        'signature': {   'params': ['text', 'voice'],
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
        'event_source': '',
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
        'event_source': '',
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
        'event_source': '',
        'name': 'set_music_volume',
        'description': 'Громкость МУЗЫКИ (трек/DJ-сет: request_music, dj_set), а НЕ '
                       'голоса робота. Юзер просит «громче/тише/погромче/потише» и '
                       'сейчас играет музыка, или прямо говорит «музыку/трек/бит '
                       'громче» — вызывай ЭТОТ тул, а не set_volume (set_volume меняет '
                       'только голос). Музыку не перезапускает: играющий трек '
                       'продолжает играть, меняется только уровень. action: '
                       'louder/quieter — шаг ±3 dB, max — максимум, normal — стартовый '
                       'уровень, set — абсолютный уровень level 0..100 (% от '
                       'максимума). Трек из библиотеки генераций (music_library play) '
                       'играет на той же деке — регулируется тоже.',
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
        'event_source': 'srv:/tts_node/set_parameters',
        'name': 'set_pitch',
        'description': 'Высота голоса робота шагом: higher / lower (±0.2, диапазон '
                       "0.5–2.0), normal — 1.0. Для команд 'говори выше', 'говори "
                       "ниже'. Абсолютного значения нет; <prosody pitch> в speak_text "
                       'не работает (MiniMax и Yandex его вырезают, pitch '
                       'игнорируется) — только этот тул.',
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
        'event_source': 'srv:/tts_node/set_parameters',
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
        'event_source': '/voice/speech/event',
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
        'event_source': '/voice/speech/event',
        'name': 'set_voice',
        'description': 'Сменить голос робота НАСОВСЕМ (персистентно, пока не сменят) — '
                       'только когда человек сам просит сменить голос: (1) «говори '
                       'голосом X» — передай voice=X; (2) «верни обычный голос» — '
                       'передай voice=default (провайдера и голос подставит код); (3) '
                       'юзер просит голос с КОНКРЕТНОГО провайдера («Яндекс Артём», '
                       "«Yandex anton») — передай voice=... И provider='yandex'. "
                       'Доступные голоса перечислены в контексте: [TTS] voices=... '
                       'Голоса персонажей сказки — параметр voice у speak_text (только '
                       'эта реплика), не set_voice. Фразу о смене голоса робот говорит '
                       'сам.',
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
        'event_source': 'srv:/tts_node/set_parameters',
        'name': 'set_volume',
        'description': 'Громкость ГОЛОСА робота (речь TTS), НЕ музыки. Используй для '
                       "'говори громче', 'тише говори', 'максимальная громкость "
                       "голоса', а также для 'громче/тише', когда музыка НЕ играет. "
                       'Если играет трек/бит/DJ-сет и юзер просит громче/тише музыку — '
                       'вызывай set_music_volume, а не этот тул. Названный уровень в '
                       "процентах ('громкость 60 %') — action=set и level.",
        'parameters': {   'type': 'object',
                          'properties': {   'action': {   'type': 'string',
                                                          'description': 'louder — '
                                                                         'громче на '
                                                                         'шаг, quieter '
                                                                         '— тише на '
                                                                         'шаг, max — '
                                                                         'на максимум, '
                                                                         'normal — '
                                                                         'обычный '
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
                                                                        'громкость '
                                                                        'голоса в '
                                                                        'процентах, '
                                                                        '0..100 '
                                                                        '(значения вне '
                                                                        'диапазона '
                                                                        'обрезаются).'}},
                          'required': ['action'],
                          'additionalProperties': False},
        'signature': {   'params': ['action', 'level'],
                         'required': ['action'],
                         'accepts_kwargs': False},
        'skill': ('voice-tts',)},
    {   'llm_visible': False,
        'operator_visible': True,
        'read_only': True,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'medium',
        'event_source': '',
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
        'event_source': '',
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
        'event_source': 'srv:/rtabmap/rtabmap/set_mode_mapping',
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
        'event_source': 'inproc:player_owner',
        'name': 'stop_music',
        'description': 'Выключить всю музыку: диджей-сет, трек или пьесу движка и mp3 '
                       'из библиотеки. Повторный вызов в тишине безопасен. Что '
                       'сказать, вернёт тул.',
        'parameters': {   'type': 'object',
                          'properties': {   'boundary': {   'type': 'string',
                                                            'description': 'now — '
                                                                           'сразу (по '
                                                                           'умолчанию); '
                                                                           'bar — бит '
                                                                           'выступления '
                                                                           'на '
                                                                           'ближайшем '
                                                                           'такте',
                                                            'enum': ['now', 'bar']}},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': ['boundary'], 'required': [], 'accepts_kwargs': False},
        'skill': ('composer', 'dj', 'player')},
    {   'llm_visible': True,
        'operator_visible': False,
        'read_only': False,
        'destructive': True,
        'idempotent': False,
        'starts_music': False,
        'execution_type': 'fast',
        'event_source': 'srv:/navigate_to_pose/_action/cancel_goal',
        'name': 'stop_navigation',
        'description': 'Остановить текущую навигацию робота. Используй для команд '
                       "'стоп', 'остановись', 'отмени движение'.",
        'parameters': {   'type': 'object',
                          'properties': {},
                          'required': [],
                          'additionalProperties': False},
        'signature': {'params': [], 'required': [], 'accepts_kwargs': False},
        'skill': ('navigation',)})
