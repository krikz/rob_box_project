"""grip_pipeline.py — pure-логика пайплайна грипа (issue #1989, шаг 4б).

Прямоточный путь речи оператора с левого грипа (§7.5
``docs/architecture/target-operator-agent-and-dialogue.md``):

    /avatar/ptt/result + /avatar/voice_pipeline
        → transform(text, preset, language)
        → /voice/tts/request

Владелец — ``avatar_supervisor``, но НЕ его агентский цикл: здесь нет ни
``AgentCore``, ни ``ToolProvider``, ни памяти, ни истории (инвариант 6c).
Трансформация — это ровно **0 или 1** вызов LLM:

* «Без стиля» / preset = none  → 0 вызовов, текст уходит дословно;
* пресет ``translate``          → 1 вызов, перевод на целевой язык;
* стилизующий пресет            → 1 вызов, переписывание в стиле.

``tool_calls`` от модели в любом случае игнорируются: этот путь не имеет
доступа к инструментам, поэтому даже «случайно поехали» невозможен.

Модуль намеренно не зависит от rclpy и от ноды — он читает источник истины
``voice_presets.yaml`` пакета ``rob_box_voice`` (пресеты, языки, дефолты) и
собирает пару ``LLMMessage`` (system = секция промпта пресета, user =
директива). Всё остальное (топики, состояние, публикация в TTS) — в
``AvatarSupervisor`` (``supervisor_node.py``).

Пока шаг 6 (удаление ``voice_input_mode``/формализатора из ``dialogue_node``)
не сделан, загрузка yaml здесь — это зеркало ``dialogue_node._load_voice_presets``;
после удаления дубля останется только этот модуль.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Optional

# Маркер EN-половины двуязычных prompt-файлов (``presets/<key>.txt``).
# RU-часть идёт первой, EN-часть начинается со строки, содержащей этот маркер.
_RU_EN_MARKER = "EN version"

# preset-значения, означающие «без стиля» (0 вызовов LLM), даже когда
# ``llm_enabled=true``. Семантика ``preset=none`` из карточки #1989.
_OFF_PRESETS: frozenset[str] = frozenset({"none", "off"})

# Функция классификации и лейблы — константы-строки, чтобы тесты и метрики
# не матчили магические литералы.
MODE_DIRECT = "direct"
MODE_TRANSLATE = "translate"
MODE_STYLE = "style"


def resolve_voice_presets_path() -> Optional[str]:
    """Абсолютный путь к ``voice_presets.yaml`` пакета ``rob_box_voice``.

    Источник истины пресетов живёт в rob_box_voice (config/voice_presets.yaml),
    а не в rob_box_supervisor. Порядок:
      1. ament share ``rob_box_voice/config/voice_presets.yaml`` (робот);
      2. source-tree ``<repo>/src/rob_box_voice/config/voice_presets.yaml``
         (colcon symlink / unit-тесты).

    ``None`` — файл нигде не найден (пайплайн честно уходит в дословный TTS).
    """
    try:
        from ament_index_python.packages import (  # noqa: PLC0415
            get_package_share_directory,
        )

        share = os.path.join(
            get_package_share_directory("rob_box_voice"), "config", "voice_presets.yaml"
        )
        if os.path.isfile(share):
            return share
    except Exception:  # noqa: BLE001 — нет ament (unit-тесты / не-colcon)
        pass
    # Source-tree fallback: <repo>/src/rob_box_voice/config/voice_presets.yaml.
    source = os.path.join(
        str(Path(__file__).resolve().parents[2]),
        "rob_box_voice",
        "config",
        "voice_presets.yaml",
    )
    return source if os.path.isfile(source) else None


def _load_preset(key: Any, cfg: Any, base_dir: str) -> dict[str, Any]:
    """Собрать описание одного пресета (имя + текст промпта) из yaml-cfg.

    Пустой dict, если ``cfg`` — не словарь (битая запись). prompt-файл
    читается относительно директории yaml (конвенция ``presets/<key>.txt``).
    """
    if not isinstance(cfg, dict):
        return {}
    name = str(cfg.get("name") or key)
    prompt_file = cfg.get("prompt_file")
    prompt_text = ""
    if prompt_file:
        raw_file = str(prompt_file)
        prompt_path = raw_file if os.path.isabs(raw_file) else os.path.join(base_dir, raw_file)
        try:
            prompt_text = Path(prompt_path).read_text(encoding="utf-8")
        except OSError:
            prompt_text = ""
    return {
        "name": name,
        "prompt_file": str(prompt_file or ""),
        "prompt_text": prompt_text,
    }


def _normalize_languages(languages_raw: Any) -> dict[str, Any]:
    """Привести ``languages`` из yaml к map ``код → {name,label,prompt_section}``.

    Поддерживает и dict-форму (каноническая), и устаревшую списочную.
    """
    if isinstance(languages_raw, dict):
        return {
            str(code).lower(): (meta if isinstance(meta, dict) else {})
            for code, meta in languages_raw.items()
        }
    if isinstance(languages_raw, list):
        return {str(x).lower(): {} for x in languages_raw}
    return {}


def load_voice_presets() -> dict[str, Any]:
    """Прочитать ``voice_presets.yaml`` + prompt-файлы пресетов.

    Возвращает dict вида::

        {
            "presets": {key: {"name", "prompt_file", "prompt_text"}},
            "languages": {code: {"name", "label", "prompt_section"}},
            "default_preset": str,
            "default_language": str,
            "_path": str | None,
        }

    При любой ошибке (нет файла, битый yaml) возвращает безопасный минимум
    с пустыми ``presets``/``languages`` — вызывающий код пайплайна увидит
    отсутствие ``prompt_text`` и уйдёт в дословный TTS (ADR-0018: честный
    fallback, а не молчаливая деградация).
    """
    fallback: dict[str, Any] = {
        "presets": {},
        "languages": {},
        "default_preset": "technical",
        "default_language": "ru",
        "_path": None,
    }
    path = resolve_voice_presets_path()
    if path is None:
        return fallback
    try:
        import yaml  # noqa: PLC0415 — ленивый импорт: supervisor importable без yaml

        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
    except Exception:  # noqa: BLE001
        return {**fallback, "_path": path}

    presets: dict[str, Any] = {}
    base_dir = os.path.dirname(path)
    for key, cfg in (data.get("presets") or {}).items():
        loaded = _load_preset(key, cfg, base_dir)
        if loaded:
            presets[str(key)] = loaded
    return {
        "presets": presets,
        "languages": _normalize_languages(data.get("languages")),
        "default_preset": str(data.get("default_preset") or "technical"),
        "default_language": str(data.get("default_language") or "ru").lower(),
        "_path": path,
    }


def language_label(languages: dict[str, Any], language: str) -> str:
    """Как назвать язык в директиве для LLM («на языке «французский»»)."""
    meta = languages.get(str(language).lower()) or {}
    label = meta.get("label")
    if isinstance(label, str) and label.strip():
        return label.strip()
    return str(language).lower() or "русский"


def language_prompt_section(languages: dict[str, Any], language: str) -> str:
    """Какую секцию двуязычного prompt-файла отдать модели: ``"ru"`` | ``"en"``.

    Для всех языков кроме русского берём EN-секцию: в ней нет правила
    «отвечай по-русски», а нужный язык задаёт директива в user-сообщении.
    """
    meta = languages.get(str(language).lower()) or {}
    section = meta.get("prompt_section")
    if isinstance(section, str):
        normalized = section.strip().lower()
        if normalized in ("ru", "en"):
            return normalized
    return "ru" if str(language).lower() == "ru" else "en"


def select_prompt_section(prompt_text: str, section: str) -> str:
    """Вернуть RU- или EN-половину двуязычного preset-промпта.

    RU-часть идёт первой, EN-часть — с маркера ``"EN version"``. Если маркер
    отсутствует (одноязычный файл) — возвращаем текст как есть: модель сама
    разберётся, язык задан директивой в user-сообщении.
    """
    marker_idx = prompt_text.find(_RU_EN_MARKER)
    if marker_idx == -1:
        return prompt_text
    return prompt_text[marker_idx:] if section == "en" else prompt_text[:marker_idx]


def classify_preset(
    preset: str, llm_enabled: bool, known_presets: Any
) -> str:
    """Классифицировать выбор панели в режим трансформации (pure).

    Возвращает одну из констант ``MODE_DIRECT`` / ``MODE_TRANSLATE`` /
    ``MODE_STYLE``. ``known_presets`` — коллекция валидных ключей пресетов
    (без ``translate``-семантики: он проверяется раньше).
    """
    if not llm_enabled:
        return MODE_DIRECT
    normalized = (preset or "").strip().lower()
    if not normalized or normalized in _OFF_PRESETS:
        return MODE_DIRECT
    if normalized == "translate":
        return MODE_TRANSLATE
    if normalized in known_presets:
        return MODE_STYLE
    # Неизвестный пресет при включённом LLM не стилизуем молча: дословный
    # TTS честнее, чем «похожий на выбранный, но не он» стиль.
    return MODE_DIRECT


def build_messages(
    system_prompt: str,
    preset_name: str,
    target_language_label: str,
    user_text: str,
) -> list[Any]:
    """Собрать пару LLMMessage для одношаговой трансформации.

    Ровно два сообщения (system + user), без истории и без инструментов.
    ``LLMMessage`` импортируется лениво — модуль остаётся importable без
    rob_box_llm (CI-минимум).
    """
    from rob_box_llm.provider import LLMMessage  # noqa: PLC0415

    user_msg = (
        "Исходная фраза оператора (дословно, без wake-word):\n\n"
        '"""\n'
        f"{user_text}\n"
        '"""\n\n'
        f"Перепиши её в стиле пресета «{preset_name}» на языке "
        f"«{target_language_label}». Сохрани смысл, длину (±×2) и все факты "
        "дословно. Не добавляй ничего от себя."
    )
    return [
        LLMMessage(role="system", content=system_prompt),
        LLMMessage(role="user", content=user_msg),
    ]


__all__ = [
    "MODE_DIRECT",
    "MODE_STYLE",
    "MODE_TRANSLATE",
    "build_messages",
    "classify_preset",
    "language_label",
    "language_prompt_section",
    "load_voice_presets",
    "resolve_voice_presets_path",
    "select_prompt_section",
]
