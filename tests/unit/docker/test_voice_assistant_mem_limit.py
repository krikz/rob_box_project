"""Regression tests for voice-assistant memory limits in
``docker/vision/docker-compose.yaml``.

Issue #2676 — voice-assistant контейнер жонглировал RSS 3.4-3.9 GB на
лимите 4 GB, пики до 99.6% → каскадные SIGKILL exit code -9 у tts_node /
stt_node / speaker_id_node каждые ~30с.

Фикс (ADR-0118, kanban t_880acd3a):
* ``mem_limit: 5g`` (↑ с 4g) — даёт ~1 GB headroom для cold-start всех 9
  нод + Silero warm-load + всплески диалогового трафика
* ``memswap_limit: 6g`` — разрешает 1 GB swap cushion на короткие пики
  (пока zram-swap по ADR-0111 ещё не задеплоен)

Тест извлекает секцию ``voice-assistant:`` из compose-файла, проверяет что
оба лимита на месте и в правильных значениях. Также проверяет что НЕ
вернулся случайно старый ``mem_limit: 4g`` (issue #929 → бампнули до 4 GB,
теперь 5 GB).

Refs:
    * issue #2676 — root cause
    * ADR-0118 — этот фикс
    * ADR-0111 — zram-swap (ещё не задеплоен, но ожидается)
    * #929 — предыдущий OOM-kill, бампнули 2g → 4g
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
COMPOSE_FILE = REPO_ROOT / "docker" / "vision" / "docker-compose.yaml"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _parse_bytes(value: str | int | float) -> int:
    """Парсит docker-compose memory value (например "5g", "512m", 2147483648).

    Поддерживает суффиксы b/k/m/g (lower-case) и ``None`` → 0.
    Без зависимостей — чтобы тест работал без docker SDK.
    """
    if value is None:
        return 0
    if isinstance(value, (int, float)):
        return int(value)
    s = str(value).strip().lower()
    m = re.match(r"^(\d+(?:\.\d+)?)([kmg]?)$", s)
    if not m:
        raise ValueError(f"cannot parse memory value: {value!r}")
    n = float(m.group(1))
    unit = m.group(2)
    mult = {"": 1, "k": 1024, "m": 1024 * 1024, "g": 1024 * 1024 * 1024}[unit]
    return int(n * mult)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def compose_doc() -> dict:
    """Парсит docker/vision/docker-compose.yaml один раз на модуль."""
    assert COMPOSE_FILE.exists(), f"compose file missing: {COMPOSE_FILE}"
    with COMPOSE_FILE.open() as f:
        return yaml.safe_load(f)


def test_compose_file_loads_and_has_voice_assistant(compose_doc: dict) -> None:
    """Sanity: compose валидный YAML и содержит секцию voice-assistant."""
    assert "services" in compose_doc, "compose file must have 'services' root"
    assert "voice-assistant" in compose_doc["services"], (
        "voice-assistant service must be defined"
    )


def test_voice_assistant_has_mem_limit(compose_doc: dict) -> None:
    """voice-assistant обязан иметь mem_limit — иначе OOM-killer не
    защищён от каскадного падения соседей (ADR-0111)."""
    svc = compose_doc["services"]["voice-assistant"]
    assert "mem_limit" in svc, "voice-assistant must have mem_limit (ADR-0111)"
    assert "memswap_limit" in svc, (
        "voice-assistant must have memswap_limit (ADR-0118, "
        "issue #2676 — без swap cushion SIGKILL каскадирует)"
    )


def test_mem_limit_is_5g_per_adr0118(compose_doc: dict) -> None:
    """mem_limit должен быть 5g (↑ с 4g после issue #2676, ADR-0118).

    Регрессия issue #2676: 4g → 3.9 GB RSS в пике → 99.6% → SIGKILL.
    """
    svc = compose_doc["services"]["voice-assistant"]
    mem_limit = _parse_bytes(svc["mem_limit"])
    expected = _parse_bytes("5g")
    assert mem_limit == expected, (
        f"voice-assistant mem_limit should be 5g per ADR-0118; "
        f"got {svc['mem_limit']!r} = {mem_limit} bytes "
        f"(expected {expected} = 5 GiB)"
    )


def test_mem_limit_not_regressed_to_4g(compose_doc: dict) -> None:
    """Антирегрессия: mem_limit не должен быть откатан к 4g.

    История: #929 → 2g→4g, #2676 → 4g→5g. Если кто-то откатит до 4g,
    этот тест покраснеет.
    """
    svc = compose_doc["services"]["voice-assistant"]
    mem_limit = _parse_bytes(svc["mem_limit"])
    not_allowed_4g = _parse_bytes("4g")
    assert mem_limit != not_allowed_4g, (
        "voice-assistant mem_limit regressed to 4g — issue #2676 "
        "requires 5g (ADR-0118)"
    )


def test_memswap_limit_is_6g_per_adr0118(compose_doc: dict) -> None:
    """memswap_limit должен быть 6g (5g RAM + 1g swap cushion).

    swap cushion защищает от OOM-killer при кратковременных пиках пока
    zram-swap по ADR-0111 ещё не задеплоен.
    """
    svc = compose_doc["services"]["voice-assistant"]
    memswap_limit = _parse_bytes(svc["memswap_limit"])
    expected = _parse_bytes("6g")
    assert memswap_limit == expected, (
        f"voice-assistant memswap_limit should be 6g per ADR-0118; "
        f"got {svc['memswap_limit']!r} = {memswap_limit} bytes "
        f"(expected {expected} = 6 GiB)"
    )


def test_memswap_exceeds_mem_limit_by_about_1g(compose_doc: dict) -> None:
    """memswap_limit ≥ mem_limit + 1 GB.

    Логика (ADR-0118 §2.3):
    - memswap_limit = mem_limit: нет cushion, OOM-killer при первом пике
    - memswap_limit = mem_limit + 1 GB: 1 GB swap cushion
    - memswap_limit >> mem_limit: container может сожрать весь swap
    """
    svc = compose_doc["services"]["voice-assistant"]
    mem = _parse_bytes(svc["mem_limit"])
    memswap = _parse_bytes(svc["memswap_limit"])
    delta = memswap - mem
    one_gb = _parse_bytes("1g")
    # Допуск ±10% на случай если кто-то поставит 5.5g memswap_limit
    assert delta >= int(one_gb * 0.9), (
        f"memswap_limit ({memswap}) - mem_limit ({mem}) = {delta} bytes; "
        f"expected ≥ ~1 GB cushion (ADR-0118 §2.3)"
    )
    # И не должно быть слишком большим (≤ mem_limit + 2 GB)
    two_gb = _parse_bytes("2g")
    assert delta <= int(two_gb * 1.1), (
        f"memswap_limit - mem_limit = {delta} bytes is too large; "
        f"expected ≤ ~2 GB cushion — otherwise container can hog swap "
        f"and starve neighbours"
    )


def test_voice_assistant_comment_mentions_adr0118(compose_doc: dict) -> None:
    """Sanity: в docker-compose.yaml должен быть ADR-0118 / issue #2676
    рядом с mem_limit — чтобы будущий разработчик понимал контекст."""
    raw = COMPOSE_FILE.read_text()
    # Найти блок voice-assistant (грубо — до следующего top-level ключа)
    m = re.search(
        r"^  voice-assistant:\n(?:\s+.*\n)+?(?=^  [a-z]|\Z)", raw, re.MULTILINE
    )
    assert m, "could not extract voice-assistant block from compose"
    block = m.group(0)
    assert "ADR-0118" in block or "issue #2676" in block, (
        "voice-assistant block should reference ADR-0118 or issue #2676 "
        "next to mem_limit (regression guard for #929-style context loss)"
    )
