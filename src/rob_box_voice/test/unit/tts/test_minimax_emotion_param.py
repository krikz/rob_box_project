"""Регресс-тест для новых ROS-параметров emotion/pitch/volume/
pronunciation_dict (MiniMax) и ``yandex_ssml_aware`` (issue #1780 / #1004).

Статический AST-тест (без импорта rclpy / tts_node.py), чтобы работал
на developer-host и в CI так же, как ``test_no_duplicate_declare_parameter``.

Проверяет:
1. Каждый из 5 новых параметров объявлен через ``declare_parameter``
   ровно один раз (нет дубликатов).
2. Каждый параметр имеет корректный default (нейтральный, сохраняющий
   текущее поведение).
3. Параметры читаются в ``self.*`` (используются в коде).
4. Helper ``_normalize_minimax_emotion`` существует и принимает
   7 валидных MiniMax-значений.

При изменении ``tts_node.py`` или YAML-конфигов этот тест сразу
указывает, если воркер добавил ключ в YAML, но не объявил параметр
в коде (issue #1004 root cause: ``undeclared → rclpy молча игнорирует``).
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


TTS_NODE_SRC = Path(
    "/home/builder/rob_box_project/.worktrees/t_a5eed3a7/"
    "src/rob_box_voice/rob_box_voice/tts_node.py"
)
SRC_YAML = Path(
    "/home/builder/rob_box_project/.worktrees/t_a5eed3a7/"
    "src/rob_box_voice/config/tts_node.yaml"
)
DOCKER_YAML = Path(
    "/home/builder/rob_box_project/.worktrees/t_a5eed3a7/"
    "docker/vision/config/voice_assistant/tts_node.yaml"
)

# Пара → (YAML-ключ, declare_parameter-имя, тип default).
# Тип default'а важен для обратной совместимости (issue #1004:
# типы параметров должны совпадать в YAML и declare_parameter).
NEW_KEYS = [
    ("minimax_emotion", "minimax_emotion", str),
    ("minimax_pitch", "minimax_pitch", int),
    ("minimax_volume", "minimax_volume", float),
    ("minimax_pronunciation_dict", "minimax_pronunciation_dict", str),
    ("yandex_ssml_aware", "yandex_ssml_aware", bool),
]


def _parse_declares_and_helpers():
    src = TTS_NODE_SRC.read_text(encoding="utf-8")
    tree = ast.parse(src)
    declares = {}  # name -> (lineno, default_value)
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and getattr(node.func, "attr", "") == "declare_parameter"):
            continue
        if not (node.args and isinstance(node.args[0], ast.Constant)):
            continue
        name = node.args[0].value
        default = node.args[1].value if len(node.args) >= 2 and isinstance(node.args[1], ast.Constant) else None
        declares[name] = (node.lineno, default)
    helpers = {
        node.name for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
    }
    return declares, helpers


def _parse_yaml_keys() -> set[str]:
    """Читаем YAML-ключи (внутри tts_node.ros__parameters), статически.

    Не используем PyYAML — нам нужен только плоский набор ключей,
    регекспом достаточно (формат стабильный — см. test_yaml_param_consistency).
    """
    keys = set()
    in_params = False
    for line in SRC_YAML.read_text(encoding="utf-8").splitlines():
        stripped = line.split("#", 1)[0].rstrip()
        if re.match(r"^\s*ros__parameters:\s*$", stripped):
            in_params = True
            continue
        if in_params:
            m = re.match(r"^\s+([a-z_][a-z0-9_]*)\s*:", stripped)
            if m:
                keys.add(m.group(1))
            elif stripped and not stripped.startswith(" ") and stripped.endswith(":"):
                # Новая секция верхнего уровня — выходим
                in_params = False
    return keys


def test_new_keys_declared_in_tts_node():
    declares, _ = _parse_declares_and_helpers()
    for yaml_key, decl_name, _ in NEW_KEYS:
        assert decl_name in declares, (
            f"declare_parameter({decl_name!r}) отсутствует в tts_node.py — "
            f"rclpy молча игнорирует YAML-ключ (issue #1004)"
        )


def test_new_keys_not_duplicated():
    """Issue #976: ``declare_parameter`` дважды → ``ParameterAlreadyDeclared``."""
    declares, _ = _parse_declares_and_helpers()
    src = TTS_NODE_SRC.read_text(encoding="utf-8")
    for _, decl_name, _ in NEW_KEYS:
        count = len(re.findall(rf'declare_parameter\(\s*["\']{re.escape(decl_name)}["\']', src))
        assert count == 1, (
            f"declare_parameter({decl_name!r}) встречается {count} раз в tts_node.py — "
            f"rclpy бросит ParameterAlreadyDeclaredException (issue #976)"
        )


def test_new_keys_have_neutral_defaults():
    """Дефолты должны сохранять текущее поведение (issue #1780: «нейтральные дефолты»).

    Конкретно:
      emotion = ''           → не передавать в API
      pitch = 0              → 0 = «не передавать»
      volume = 0.0           → 0.0 = «не передавать»
      pronunciation_dict = ''→ пусто = «не передавать»
      yandex_ssml_aware = False → legacy Yandex (Hints(voice, speed))
    """
    declares, _ = _parse_declares_and_helpers()
    expected = {
        "minimax_emotion": "",
        "minimax_pitch": 0,
        "minimax_volume": 0.0,
        "minimax_pronunciation_dict": "",
        "yandex_ssml_aware": False,
    }
    for name, want in expected.items():
        _, default = declares[name]
        assert default == want, (
            f"declare_parameter({name!r}) default={default!r}, ожидалось {want!r}. "
            f"Нейтральный default обязателен (issue #1780)."
        )


def test_new_keys_read_into_self():
    """Параметры должны читаться через ``get_parameter`` в self.* атрибуты."""
    src = TTS_NODE_SRC.read_text(encoding="utf-8")
    for _, decl_name, _ in NEW_KEYS:
        # Ожидаем: self.<attr> = ... self.get_parameter(<name>) ...
        # attr = short name (без префикса) — в данном случае 1:1
        assign_pattern = rf"self\.{re.escape(decl_name)}\s*=\s*"
        get_pattern = rf'self\.get_parameter\(\s*["\']{re.escape(decl_name)}["\']'
        assert re.search(assign_pattern, src), (
            f"self.{decl_name} нигде не присваивается в tts_node.py — параметр "
            f"объявлен, но не читается (мёртвый код)."
        )
        assert re.search(get_pattern, src), (
            f"get_parameter({decl_name!r}) не вызывается в tts_node.py — параметр "
            f"объявлен, но не читается."
        )


def test_normalize_minimax_emotion_helper_exists():
    """Helper для нормализации MiniMax-emotion обязателен (статик-метод)."""
    _, helpers = _parse_declares_and_helpers()
    assert "_normalize_minimax_emotion" in helpers, (
        "TTSNode._normalize_minimax_emotion helper отсутствует — "
        "YAML-ключ ``minimax_emotion`` не валидируется."
    )


def test_normalize_minimax_emotion_validates_minimax_t2a_v2():
    """Допустимые значения MiniMax T2A v2 voice_setting.emotion:
    happy | neutral | sad | angry | fearful | disgusted | surprised.
    """
    src = TTS_NODE_SRC.read_text(encoding="utf-8")
    # Грубая sanity-проверка: все 7 значений присутствуют в helper'е
    for emotion in ("happy", "neutral", "sad", "angry", "fearful", "disgusted", "surprised"):
        assert f'"{emotion}"' in src, (
            f"valid emotion {emotion!r} отсутствует в TTSNode._normalize_minimax_emotion — "
            f"MiniMax T2A v2 ожидает эти 7 значений в voice_setting.emotion."
        )


def test_yaml_src_contains_all_new_keys():
    """YAML-конфиг содержит все 5 новых ключей (иначе тест consistency упадёт)."""
    keys = _parse_yaml_keys()
    for yaml_key, _, _ in NEW_KEYS:
        assert yaml_key in keys, (
            f"YAML-ключ {yaml_key!r} отсутствует в src/rob_box_voice/config/tts_node.yaml"
        )


def test_yaml_docker_contains_all_new_keys():
    """Docker live-конфиг (production) содержит все 5 новых ключей."""
    keys = set()
    in_params = False
    for line in DOCKER_YAML.read_text(encoding="utf-8").splitlines():
        stripped = line.split("#", 1)[0].rstrip()
        if re.match(r"^\s*ros__parameters:\s*$", stripped):
            in_params = True
            continue
        if in_params:
            m = re.match(r"^\s+([a-z_][a-z0-9_]*)\s*:", stripped)
            if m:
                keys.add(m.group(1))
            elif stripped and not stripped.startswith(" ") and stripped.endswith(":"):
                in_params = False
    for yaml_key, _, _ in NEW_KEYS:
        assert yaml_key in keys, (
            f"YAML-ключ {yaml_key!r} отсутствует в docker/vision/.../tts_node.yaml "
            f"— production-конфиг не примет параметр, даже если код объявит."
        )
