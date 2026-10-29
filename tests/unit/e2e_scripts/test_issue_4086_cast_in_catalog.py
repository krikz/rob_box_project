"""#4086: каждый голос каста марафона (харнесс e2e) есть в каталоге голосов робота.

Каст живёт в ``map_tts_voice`` харнесса (``.github/workflows/scripts/e2e_voice_lib.sh``, ветка ``minimax``): bash не
импортируется из Python, поэтому источник один - эта функция, а каталог (``tts_voice_registry``) сверяется с ней тестом.
Разойдутся (в харнесс добавили голос, а в каталог робота нет) - «Робот, говори голосом <имя каста>» опять честно ответит
«такого голоса нет», как 27.10 (dev-0d94634): тест красный раньше живой проверки.

Источник читается как ТЕКСТ, без bash: на Windows-хосте ``bash`` - это WSL, и сверка зависела бы от его запуска.

Run:
    python -m pytest tests/unit/e2e_scripts/test_issue_4086_cast_in_catalog.py -v --no-cov
"""

from __future__ import annotations

import importlib.util
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
E2E_LIB = ROOT / ".github" / "workflows" / "scripts" / "e2e_voice_lib.sh"
REGISTRY = ROOT / "src" / "rob_box_voice" / "rob_box_voice" / "tts_voice_registry.py"

#: Каст #3992 (docs/dialog/role_voices_biometrics_2026-10-26.md) и запасной женский голос #4038.
CAST = {
    "English_Jovialman",
    "English_WiseScholar",
    "Chinese (Mandarin)_Gentle_Senior",
    "English_Whispering_girl",
    "Portuguese_StrictBoss",
}


def _registry():
    """Реестр голосов по пути файла: чистый Python, пакет ``rob_box_voice`` (ROS-окружение) не нужен."""
    spec = importlib.util.spec_from_file_location("_registry_4086", REGISTRY)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclass в модуле ищет свой модуль в sys.modules
    spec.loader.exec_module(module)
    return module


def _minimax_arm(lib_text: str) -> str:
    """Ветка ``minimax)`` функции ``map_tts_voice`` - до ``;;`` после ``return 0``."""
    start = lib_text.index("map_tts_voice() {")
    arm_start = lib_text.index("        minimax)", start)
    arm_end = lib_text.index("            ;;", arm_start)
    return lib_text[arm_start:arm_end]


def harness_minimax_voices() -> set:
    """id голосов MiniMax, которые ``map_tts_voice minimax`` печатает для имён сценариев: ``printf '<id>[:pitch:speed]'``.

    ``printf '%s' "$voice"`` (проход голосов, уже native для провайдера) не в счёт: его id приходит от вызывающего.
    """
    arm = _minimax_arm(E2E_LIB.read_text(encoding="utf-8"))
    specs = re.findall(r"printf '([^'%][^']*)'", arm)
    return {spec.split(":")[0] for spec in specs}


def test_the_harness_source_is_readable_and_names_the_whole_cast() -> None:
    """Сверка не пустая: из харнесса извлечены все пять голосов каста (иначе формат функции изменился - поправить тест)."""
    voices = harness_minimax_voices()
    assert (
        CAST <= voices
    ), f"в map_tts_voice minimax нет {sorted(CAST - voices)}; нашлось {sorted(voices)}"


def test_every_voice_the_harness_gives_to_minimax_is_in_the_robot_catalogue() -> None:
    catalogue = set(_registry().voices_for("minimax"))
    missing = sorted(harness_minimax_voices() - catalogue)
    assert not missing, (
        f"голоса харнесса ({E2E_LIB.name}, map_tts_voice minimax) есть только у харнесса: {missing}. "
        "Добавь их в PROVIDER_VOICES['minimax'] и VOICE_METADATA (tts_voice_registry.py) с русским именем."
    )


@pytest.mark.parametrize("voice_id", sorted(CAST))
def test_every_cast_voice_has_a_russian_name_the_robot_understands(
    voice_id: str,
) -> None:
    registry = _registry()
    info = registry.voice_info_for("minimax", voice_id)
    assert info is not None and re.search(r"[А-Яа-яЁё]", info["display_name"]), info
    assert registry.resolve_voice("minimax", info["display_name"]) == (voice_id, False)
