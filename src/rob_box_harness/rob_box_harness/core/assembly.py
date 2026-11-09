"""Сборка LLM-цепочки агента по :class:`AgentSpec` (ADR-0083; цикл v1 ``AgentCore`` и ``build_agent`` удалены, #4354 A3b).

Диалог v2 (``dialog_node``, ТАРС ``operator_turns``) берёт отсюда :func:`build_llm_chain` (``HealthAwareFallbackLLM``
с общим на машину кэшем здоровья). Системный промпт собирает хост из основы ядра и персоны профиля
(``rob_box_dialog.prompts``, ADR-0167 Ш1б): загрузчик промпта из файла (``load_system_prompt``, поля ``prompt_dir`` /
``system_prompt_file``) удалён в Ш1в - его никто не звал. Расхождения между агентами раскладываются по полям спека, а не
по веткам ``if spec.name == ...``.
"""

from __future__ import annotations

import logging
import os
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from rob_box_harness.health import (
    DEFAULT_HEALTH_TTL_S,
    HealthAwareFallbackLLM,
    HealthCache,
    check_deepseek_balance,
)
from rob_box_harness.providers import (
    DEEPSEEK_DEFAULT_BASE_URL,
    LLM_PROVIDER_REGISTRY,  # noqa: F401  — re-exported for tests
    build_provider,
)
from rob_box_llm.provider import LLMSettings

_logger = logging.getLogger(__name__)


# ── Spec ────────────────────────────────────────────────────────────


def _same_provider(provider: Any) -> Any:
    """Обёртка провайдера по умолчанию: без изменений."""
    return provider


@dataclass(frozen=True)
class AgentSpec:
    """Декларативная конфигурация одного агента.

    Поля разделены на «что у агента своё» (всегда разное у личности и
    ТАРС) и «что обязано быть одинаковым» (инварианты сборки). Это та
    же фактура, что в ADR-0080 §1.6: «выглядят случайными, а не
    намеренными» — здесь каждое расхождение получает имя.

    Frozen dataclass — чтобы нода не модифицировала собранный спек в
    рантайме (это та же дисциплина, что и ADR-0018: «не добавлять
    параметры молча»).
    """

    # ── идентичность (всегда разное у двух агентов) ─────────────
    name: str
    """``"personality"`` или ``"operator"`` — значение колонки ``agent``."""

    # ── LLM (частично общее — см. ADR-0083 §2.3) ────────────────
    provider_chain: tuple[str, ...] = ("deepseek",)
    """Упорядоченный список имён провайдеров (``minimax``, ``deepseek``,
    …). Первый = primary, остальные = fallbacks."""

    settings: LLMSettings | None = None
    """Глобальные ``temperature`` / ``max_tokens`` для primary.
    Провайдеры с явными per-provider overrides в
    ``per_provider_settings`` игнорируют это."""

    per_provider_settings: Mapping[str, LLMSettings] = field(
        default_factory=dict
    )
    """Карта ``{provider_name: LLMSettings}`` — issue #1883. Personality
    заполняет (у MiniMax своя ``temperature``), supervisor держит
    пустым (per-provider не нужны, ADR-0083 §1.2 #B)."""

    health_cache_persist_path: Path | None = None
    """Общий для всех агентов машины путь (по дефолту
    ``~/.rob_box/llm_health.json``). У supervisor раньше стоял
    ``HealthCache()`` без ``persist_path`` — это и был класс инцидента
    «после рестарта все больные снова здоровы» (ADR-0083 §1.3 #1)."""

    health_ttl_s: float = DEFAULT_HEALTH_TTL_S

    health_balance_checkers: Mapping[
        str, Callable[[], Any]
    ] = field(default_factory=dict)
    """``{provider_name: async_callable}``. Personality регистрирует
    deepseek-пробер (есть balance API); supervisor — пусто."""

    wrap_provider: Callable[[Any], Any] = _same_provider
    """Обёртка каждого собранного провайдера цепочки ДО ``HealthAwareFallbackLLM`` (имя кэша здоровья — у
    обёрнутого). Personality так включает режим команд по capabilities провайдера (ADR-0150 §4.5,
    ``rob_box_voice.core.command_bridge``); по умолчанию провайдер как есть."""


def _resolve_provider_settings(
    spec: AgentSpec,
    name: str,
) -> LLMSettings | None:
    """Подобрать ``LLMSettings`` для конкретного провайдера в цепочке.

    Precedence:

    1. ``per_provider_settings[name]`` — явный override (issue #1883).
    2. ``spec.settings`` — глобальные temperature/max_tokens.
    3. ``None`` — отдать выбор провайдеру (его default).

    Зеркалит поведение ``dialogue_node._build_llm_settings_for``, но
    в чистом виде — без YAML/ROS-зависимостей.
    """
    if name in spec.per_provider_settings:
        return spec.per_provider_settings[name]
    return spec.settings


def build_llm_chain(spec: AgentSpec) -> Any:
    """Публичная обёртка над :func:`_build_llm_chain` (ADR-0083 §2.3).

    Цепочка LLM хода v2 (``dialog_v2_ports``, ТАРС ``operator_turns``).

    Контракт повторяет внутренний путь: per-provider ``LLMSettings``
    разрешаются через :func:`_resolve_provider_settings` (precedence
    per-provider → spec.settings → None), deepseek-пробер подставляется
    автоматически, если в ``provider_chain`` есть ``deepseek`` и
    ``spec.health_balance_checkers`` пуст. Это та же логика, что и до
    рефакторинга в ``dialogue_node._build_llm`` (ADR-0083 §1.2 #A).
    """
    checkers = (
        _default_balance_checkers(spec)
        if not spec.health_balance_checkers
        else dict(spec.health_balance_checkers)
    )
    settings_for: dict[str, LLMSettings] = {
        name: _resolve_provider_settings(spec, name)
        for name in spec.provider_chain
    }
    effective_spec = spec
    if checkers is not spec.health_balance_checkers and len(
        spec.provider_chain
    ) > 1:
        effective_spec = _with_checkers(spec, checkers)
    return _build_llm_chain(effective_spec, settings_for=settings_for)


def _build_llm_chain(
    spec: AgentSpec,
    settings_for: dict[str, LLMSettings],
) -> Any:
    """Собрать LLM-цепочку по спеке.

    Поведение совпадает с бывшим ``dialogue_node._build_llm`` /
    ``supervisor_node._build_operator_llm``: первый живой провайдер
    становится primary, остальные — fallbacks. Один живой провайдер
    возвращается as-is; 2+ → оборачиваются в
    :class:`HealthAwareFallbackLLM` с общим ``HealthCache``.

    Per-provider ``LLMSettings`` форвардятся в
    ``HealthAwareFallbackLLM.settings_for`` (issue #1883) — supervisor
    больше не роняет эту фичу (ADR-0083 §1.2 #B).

    ``balance_checkers`` тоже едут в обёртку (deepseek) — supervisor
    раньше их игнорировал (ADR-0083 §1.2 #A).
    """
    chain_names = [n.strip().lower() for n in spec.provider_chain if n.strip()]
    if not chain_names:
        raise RuntimeError(
            f"AgentSpec {spec.name!r}: provider_chain is empty"
        )

    built: list[Any] = []
    for name in chain_names:
        try:
            provider = build_provider(name)
        except (KeyError, Exception) as exc:  # noqa: BLE001
            _logger.warning(
                "[assembly] %s: provider %r unavailable: %s",
                spec.name, name, exc,
            )
            continue
        built.append(spec.wrap_provider(provider))

    if not built:
        raise RuntimeError(
            f"AgentSpec {spec.name!r}: no LLM provider could be built from "
            f"chain {chain_names!r} (check API keys / env vars)"
        )

    if len(built) == 1:
        return built[0]

    cache = HealthCache(
        ttl_s=spec.health_ttl_s,
        persist_path=spec.health_cache_persist_path,
    )
    return HealthAwareFallbackLLM(
        built,
        cache=cache,
        balance_checkers=dict(spec.health_balance_checkers),
        settings_for=settings_for,
    )


# ── Helpers ─────────────────────────────────────────────────────────


def _default_balance_checkers(
    spec: AgentSpec,
) -> dict[str, Callable[[], Any]]:
    """Каркас дефолтных balance-проберов (deepseek).

    Используется, если в ``spec.health_balance_checkers`` ничего не
    задано явно и в ``provider_chain`` есть ``deepseek`` (у него
    единственного в registry есть ``has_balance_api=True`` — см.
    ``rob_box_harness.providers.catalog``). Это сохраняет поведение
    dialogue_node, где deepseek-пробер жил прямо в ``_build_llm``.
    """
    if spec.health_balance_checkers:
        return dict(spec.health_balance_checkers)
    if "deepseek" not in spec.provider_chain:
        return {}
    async def _deepseek_probe() -> Any:  # pragma: no cover — network probe
        return await check_deepseek_balance(
            DEEPSEEK_DEFAULT_BASE_URL,
            os.environ.get("DEEPSEEK_API_KEY", ""),
            timeout_s=5.0,
        )
    return {"deepseek": _deepseek_probe}


# ── Public entry ────────────────────────────────────────────────────


def _with_checkers(
    spec: AgentSpec,
    checkers: Mapping[str, Callable[[], Any]],
) -> AgentSpec:
    """Вернуть копию спека с подменённым ``health_balance_checkers``.

    Frozen dataclass нельзя мутировать, поэтому для авто-детекта
    deepseek-пробера делаем ``dataclasses.replace``. Это нулевой
    оверхед (вызывается один раз на старте ноды).
    """
    from dataclasses import replace

    return replace(spec, health_balance_checkers=dict(checkers))


__all__ = [
    "AgentSpec",
    "build_llm_chain",
]
