"""Порты владельцев тулов: какие тулы каталога за каким владельцем (ADR-0167 §2.4, шаг Ш1в).

Владелец - тот, кто исполняет тул (речь - ``tts_node``, музыка - плеер ``mcp_server``, движение - Nav2, лица -
``vision_face``…). Порты - знание ядра: таблица :data:`TOOL_OWNERS`, у каждого тула командой
(``knowledge.TOOL_CLASSES``) ровно один владелец. Какие владельцы у продукта ЕСТЬ - решает его профиль (``owners`` в
``assistant.yaml``, формат 4): у робота - все, у колонки нет движения, камеры и лиц. Тул владельца, которого у продукта нет, модель не видит ни в
одном срезе (``agents.tools_for`` → ``tools_view``, раздел инструментов ТАРС) - продукт не обещает того, чего не умеет
(07.11 колонка без этого отвечала «заряд 87 %» и «никого в кадре», ADR-0167 §2.4).

Хост передаёт владельцев профиля сессии (``Session(..., owners=...)`` → ``SessionSnapshot.owners``) и мосту команд.
Не передал (``None``) - умолчание ядра :data:`OWNERS_WITHOUT_PROFILE`: пусто, без продуктовых тулов, как
``knowledge.WAKE_WORDS_WITHOUT_PROFILE`` без вейков. Читается в момент вызова (тесты ставят владельцев робота
фикстурой).

Транспорт речи (``knowledge.SPEECH_CHANNEL_TOOLS``: ``speak_text``) - не тул владельца: канал речи есть у любого
продукта.
"""

from __future__ import annotations

import functools
from types import MappingProxyType
from typing import FrozenSet, Mapping, Optional, Sequence, Tuple

#: Порт владельца → тулы каталога ``rob_box_core.tool_catalog``. Порядок портов - порядок в ошибках и в профиле робота.
TOOL_OWNERS: Mapping[str, Tuple[str, ...]] = MappingProxyType(
    {
        # речь робота и её голос - tts_node (SpeechOwner, VoiceOwner); say / say_aloud - речь динамиками от ТАРС и чата
        "speech": (
            "set_voice",
            "set_tts_provider",
            "list_tts_voices",
            "set_speed",
            "set_pitch",
            "estimate_tts_duration",
            "set_volume",
            "say",
            "say_aloud",
        ),
        # слух - stt_node: ждать ответ человека
        "hearing": ("listen_for_response",),
        # память о людях и фактах (БД памяти голоса)
        "memory": ("memory_save", "memory_search", "memory_context"),
        "clock": ("get_current_time",),
        "web": ("search_web",),
        # музыка - PlayerOwner в mcp_server (Renardo, кэш Suno, библиотека мелодий)
        "music": (
            "request_music",
            "dj_set",
            "stop_music",
            "get_music_state",
            "music_library",
            "generate_music",
            "edit_music",
            "set_music_volume",
            "lookup_melody",
            "search_melody",
            "add_music_material",
        ),
        "sound": ("play_sound", "get_sound_info"),
        # лицо робота на экране - face_node
        "face": ("play_animation",),
        # голос собеседника - speaker_id_node
        "speaker_id": ("register_speaker",),
        # лица из камеры - FaceStore ноды vision_face (сервис /perception/people)
        "people": ("people_list", "person_photo", "name_person", "find_similar", "merge_people"),
        # движение базы и точки маршрута - Nav2
        "navigation": (
            "navigate_to_waypoint",
            "navigate_to_coordinates",
            "move_direction",
            "stop_navigation",
            "get_current_pose",
            "list_waypoints",
            "save_waypoint",
            "delete_waypoint",
            "clear_waypoints",
        ),
        # карты - rtabmap
        "mapping": (
            "start_mapping",
            "continue_mapping",
            "finish_mapping",
            "optimize_map",
            "load_map",
            "list_maps",
            "delete_map",
        ),
        "battery": ("get_robot_status", "get_battery_level"),
        # восприятие: камера, люди в кадре, интернет
        "perception": ("get_perception_context",),
        # FAQ мероприятия
        "faq": ("faq_search",),
        # пульт: Prometheus/cAdvisor, логи, граф ROS, VESC
        "telemetry": ("show_metrics", "container_status", "read_logs"),
        "ros": ("ros2_node_status", "ros_list", "topic_echo", "topic_stats", "topic_publish"),
        "vesc": ("vesc_telemetry",),
    }
)
#: Все порты ядра (порядок таблицы).
OWNERS: Tuple[str, ...] = tuple(TOOL_OWNERS)
#: Владельцы, когда хост не передал профиль (``owners=None``): нет - нейтральное ядро без продуктовых тулов.
OWNERS_WITHOUT_PROFILE: Tuple[str, ...] = ()

_OWNER_OF: Mapping[str, str] = MappingProxyType({t: o for o, tools in TOOL_OWNERS.items() for t in tools})


def owner_of(tool: str) -> Optional[str]:
    """Владелец тула (порт :data:`TOOL_OWNERS`) или ``None`` - тул ничей (транспорт речи, чужое имя)."""
    return _OWNER_OF.get(tool)


def owners_or_default(owners: Optional[Sequence[str]]) -> Tuple[str, ...]:
    """Владельцы, переданные хостом (профиль продукта), иначе умолчание ядра :data:`OWNERS_WITHOUT_PROFILE`."""
    return OWNERS_WITHOUT_PROFILE if owners is None else tuple(owners)


def owned_tools(owners: Optional[Sequence[str]]) -> FrozenSet[str]:
    """Тулы этих владельцев (``None`` - умолчание ядра). Чужое имя владельца - ``ValueError``: опечатка в профиле не
    должна молча прятать тулы."""
    return _owned(owners_or_default(owners))


@functools.lru_cache(maxsize=32)
def _owned(names: Tuple[str, ...]) -> FrozenSet[str]:
    unknown = sorted(set(names) - set(OWNERS))
    if unknown:
        raise ValueError(f"незнакомые владельцы {unknown}; порты ядра: {list(OWNERS)}")
    return frozenset(t for o in names for t in TOOL_OWNERS[o])


__all__ = ["OWNERS", "OWNERS_WITHOUT_PROFILE", "TOOL_OWNERS", "owned_tools", "owner_of", "owners_or_default"]
