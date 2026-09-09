"""``build_agent`` — single assembly bench for ``AgentCore``.

ADR-0083 фиксирует контракт сборки :class:`AgentCore`. До этого ADR
:class:`AgentCore` собирался в двух местах (``dialogue_node._build_llm``
+ ``supervisor_node._build_operator_llm``), и расхождения выглядели
случайными, а не намеренными:

* supervisor ронял ``HealthCache(persist_path=...)`` (эфемерный кеш,
  после рестарта все «больные» провайдеры снова «здоровы»);
* supervisor не использовал per-provider ``LLMSettings``;
* supervisor лез в ``rob_box_voice/prompts/skills`` через относительный
  путь (протечка в чужой пакет).

Этот модуль собирает ``AgentCore`` по одной декларативной спеке —
:class:`AgentSpec` — и является **единственной** точкой, где
``AgentCore(`` встречается в проде (ADR-0083 §2.1). Все «заботы» из
ADR-0080 §1.6 раскладываются по полям спека:

* ``prompt_dir`` / ``system_prompt_file`` — где лежит system_prompt.
* ``skill_slice`` — какие фрагменты скиллов читать
  (:func:`load_skill_prompts`).
* ``tools`` — готовый ``ToolProvider`` от ноды (нужен rclpy для
  ``ROSMCPToolProvider``).
* ``provider_chain`` / ``settings`` / ``per_provider_settings`` — LLM.
* ``health_cache_persist_path`` / ``health_ttl_s`` /
  ``health_balance_checkers`` — общие на машину.
* ``memory_namespace`` — значение колонки ``agent`` в ``facts``
  (ADR-0083 §2.4).
* ``on_prompt`` — observer для метрик.

Нода остаётся владельцем ``tools`` (ей нужен ``self`` для
``ROSMCPToolProvider(LLMToolCallAdapter(self))`` — тащить rclpy в harness
нельзя); память (``MemoryStore``) тоже конструируется нодой, потому что
ей принадлежит asyncio-loop, через который ``SQLiteVoiceMemory.init()``
синхронно вызывает ``conn.executescript``. Всё остальное, что можно
собрать без ROS-зависимостей и без asyncio-loop, делает
:func:`build_agent`.
"""

from __future__ import annotations

import logging
import os
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

from rob_box_harness.core.agent_core import AgentCore
from rob_box_harness.core.dialogue_state_machine import DialogueStateMachine
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

if TYPE_CHECKING:
    from rob_box_harness.memory import MemoryStore
    from rob_box_harness.tools import ToolProvider

_logger = logging.getLogger(__name__)


# ── Spec ────────────────────────────────────────────────────────────


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

    prompt_dir: Path
    """Каталог, где лежат ``<system_prompt_file>`` и ``skills/``."""

    system_prompt_file: str
    """Имя ``.txt`` внутри ``prompt_dir`` (system_prompt)."""

    skill_slice: tuple[str, ...] = ()
    """Имена фрагментов из ``prompt_dir/skills/<name>.txt`` для загрузки.
    Пустой tuple = секция скиллов выключена (Move A, ADR-0080 §3.5).
    """

    use_scheduler: bool = False
    """Пока W7b планировщик (issue #968) живёт только на стороне
    личности. Supervisor собирает агента ``use_scheduler=False`` —
    явное значение через спек, а не хардкод в supervisor'е."""

    memory_namespace: str = "personality"
    """Значение колонки ``agent`` в ``facts`` (ADR-0083 §2.4)."""

    on_prompt: Callable[..., None] | None = None
    """Observer, который AgentCore зовёт на каждом ``complete()``.
    DialogueNode пробрасывает ``_on_prompt_stats``, supervisor —
    ``_record_supervisor_prompt_stats``."""

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

    use_streaming: bool = False

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

    # ── обязательно общее ────────────────────────────────────────
    history_trim_limit: int = 10

    narrow_tools_to_skill: bool = False
    """ТАРС видит все инструменты без сужения — это явное значение,
    а не хардкод supervisor'а (ADR-0083 §2.2 #J)."""

    dsm: DialogueStateMachine = field(default_factory=DialogueStateMachine)

    user_id: str = "default"


# ── Prompt / skill loaders ──────────────────────────────────────────


def load_system_prompt(spec: AgentSpec) -> str:
    """Прочитать ``prompt_dir / system_prompt_file``.

    Пустой/битый файл → возврат ``""``, ``build_agent`` после этого
    отказывается собирать core (явный «нет промпта — нет агента»).
    Делает best-effort: если ``prompt_dir`` не существует, логирует
    и возвращает ``""`` — тот же контракт, что был у
    ``dialogue_node._load_system_prompt`` (нода не падает на отсутствии
    промпта, чтобы не валить старт робота).
    """
    if not spec.prompt_dir or not spec.prompt_dir.is_dir():
        _logger.warning(
            "[assembly] prompt_dir missing for %s: %s",
            spec.name,
            spec.prompt_dir,
        )
        return ""
    path = spec.prompt_dir / spec.system_prompt_file
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError as exc:
        _logger.warning(
            "[assembly] system prompt unreadable for %s: %s (%s)",
            spec.name, path, exc,
        )
        return ""


def load_skill_prompts(spec: AgentSpec) -> dict[str, str]:
    """Прочитать фрагменты скиллов из ``prompt_dir/skills/``.

    Возвращает ``{skill_name: text}`` для каждого имени из
    ``spec.skill_slice``. Отсутствие файла для объявленного скилла
    **не** считается ошибкой — скилл просто остаётся без текста, а
    его инструменты продолжают работать.

    В отличие от supervisor_node, эта функция **никогда** не лезет в
    ``rob_box_voice/prompts/skills`` — ADR-0083 §1.3 #3 фиксирует
    эту протечку как класс регрессии, который этот модуль закрывает.
    Если ТАРСу нужны personality-фрагменты, они должны быть
    опубликованы через ``tool_catalog`` (ADR-0051 §6) или явно
    включены в ``skill_slice`` со ссылкой на нужный пакет — но не
    здесь.
    """
    if not spec.skill_slice:
        return {}
    if not spec.prompt_dir or not spec.prompt_dir.is_dir():
        _logger.info(
            "[assembly] %s: skills disabled (prompt_dir missing or empty "
            "skill_slice)",
            spec.name,
        )
        return {}
    skills_dir = spec.prompt_dir / "skills"
    loaded: dict[str, str] = {}
    absent: list[str] = []
    for skill in spec.skill_slice:
        path = skills_dir / f"{skill}.txt"
        try:
            text = path.read_text(encoding="utf-8").strip()
        except OSError:
            absent.append(skill)
            continue
        if text:
            loaded[skill] = text
        else:
            absent.append(skill)
    if loaded:
        _logger.info(
            "[assembly] %s: loaded %d skill fragment(s) — %s",
            spec.name, len(loaded), ", ".join(sorted(loaded)),
        )
    if absent:
        _logger.warning(
            "[assembly] %s: %d skill fragment(s) had no usable text: %s "
            "(tools remain registered; LLM runs without domain instructions)",
            spec.name, len(absent), ", ".join(sorted(absent)),
        )
    return loaded


# ── LLM chain builder ───────────────────────────────────────────────


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

    Нода может вызвать её, чтобы собрать LLM ДО :func:`build_agent` —
    нужно для метрик ``record_voice_llm_request`` / OTel span
    ``dialogue.llm_call``, которые читают ``self._llm.name`` /
    ``self._llm.model``. Возвращённый объект можно передать в
    :func:`build_agent` параметром ``llm=``, и тогда внутренний
    :func:`_build_llm_chain` повторно НЕ вызывается.

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
        built.append(provider)

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


def build_agent(
    spec: AgentSpec,
    *,
    tools: "ToolProvider",
    memory: "MemoryStore",
    llm: Any = None,
    system_prompt: str | None = None,
    skill_prompts: Mapping[str, str] | None = None,
) -> AgentCore:
    """Собрать :class:`AgentCore` по спеке (ADR-0083 §2.1).

    Единственная публичная сборка ``AgentCore`` в проекте. Все расхождения
    между личностью и ТАРС раскладываются по полям :class:`AgentSpec` —
    здесь нет ``if spec.name == "operator"`` веток.

    Аргументы:

    * ``spec`` — декларативная конфигурация (см. :class:`AgentSpec`).
    * ``tools`` — готовый ``ToolProvider``, собранный нодой. Нода
      обязана принести его собранным, потому что
      ``ROSMCPToolProvider`` держит ссылку на ноду для создания
      service-клиентов (см. ADR-0083 §2.3 trade-off).
    * ``memory`` — готовый ``MemoryStore``. Нода конструирует его
      сама, потому что ей принадлежит asyncio-loop, через который
      ``SQLiteVoiceMemory.init()`` синхронно вызывает
      ``conn.executescript`` (см. ADR-0083 §2.3).
    * ``llm`` — готовый LLM (опционально). Если нода уже собрала LLM
      (например, через :func:`build_llm_chain` для метрик
      ``record_voice_llm_request``), она может передать его сюда, чтобы
      избежать двойной сборки. По умолчанию ``None`` — LLM собирается
      внутри из ``spec.provider_chain`` (обратная совместимость с
      тестами, supervisor'ом и сценариями, где нода не держит ссылку
      на LLM).
    * ``system_prompt`` — опциональная ЗАМЕНА текста system_prompt
      (ADR-0083 §2.3 follow-up). По умолчанию ``None`` — текст читается
      из ``spec.prompt_dir / spec.system_prompt_file``. Если нода
      применила к прочитанному тексту post-processing
      (``_split_skill_sections`` dialogue_node — раскол §5/§6
      мастер-промпта по фрагментам скиллов), она передаёт результат
      сюда, иначе сборка не увидит pre-split текст.
    * ``skill_prompts`` — опциональная ЗАМЕНА фрагментов скиллов
      (ADR-0083 §2.3 follow-up). По умолчанию ``None`` — фрагменты
      читаются из ``spec.prompt_dir/skills/`` по ``spec.skill_slice``.
      Нода может передать сюда post-processed словарь (после
      ``_split_skill_sections`` + ``merge_skill_prompts``).

    Контракт:

    * ``spec.prompt_dir`` + ``spec.system_prompt_file`` — читаются
      здесь; если файл пуст/отсутствует, ``AgentCore`` всё равно
      поднимается, но с пустым system_prompt (это документировано в
      ADR-0083 §1.2 #C: «намеренное расхождение»).
    * ``spec.health_cache_persist_path`` — общий на машину, supervisor
      больше не игнорирует персист (ADR-0083 §1.3 #1).
    """
    if not spec.provider_chain:
        raise RuntimeError(
            f"AgentSpec {spec.name!r}: provider_chain is empty"
        )

    # Per-provider settings map for ``HealthAwareFallbackLLM.settings_for``.
    # We resolve once up-front so the wrapper receives a stable dict and
    # the single-provider path can also use the primary entry.
    settings_for: dict[str, LLMSettings] = {
        name: _resolve_provider_settings(spec, name)
        for name in spec.provider_chain
    }
    primary_settings = settings_for[spec.provider_chain[0]]

    if llm is None:
        # Auto-fill deepseek balance probe when caller didn't override.
        # Only relevant for the internal LLM build path: when the node
        # passes its own ``llm=``, the chain is already wired and balance
        # probes are its problem.
        checkers = (
            _default_balance_checkers(spec)
            if not spec.health_balance_checkers
            else dict(spec.health_balance_checkers)
        )
        # Apply auto-detected checkers to a *derived* spec-shape so the
        # frozen dataclass stays immutable. The chain builder reads
        # ``spec.health_balance_checkers`` directly, so we re-build it
        # once here with the auto-detected probe instead of mutating
        # the spec.
        effective_spec = spec
        if checkers is not spec.health_balance_checkers and len(
            spec.provider_chain
        ) > 1:
            effective_spec = _with_checkers(spec, checkers)
        llm = _build_llm_chain(effective_spec, settings_for=settings_for)

    # ADR-0083 §2.3 follow-up: caller may post-process the prompt /
    # skill fragments (dialogue_node applies ``_split_skill_sections``
    # before core starts). When overrides are None we read from disk.
    if system_prompt is None:
        system_prompt = load_system_prompt(spec)
    if skill_prompts is None:
        skill_prompts = load_skill_prompts(spec)

    core = AgentCore(
        llm=llm,
        tools=tools,
        memory=memory,
        dsm=spec.dsm,
        user_id=spec.user_id,
        history_trim_limit=spec.history_trim_limit,
        system_prompt=system_prompt,
        use_streaming=spec.use_streaming,
        on_prompt=spec.on_prompt,
        skill_prompts=skill_prompts,
        narrow_tools_to_skill=spec.narrow_tools_to_skill,
        llm_settings=primary_settings,
    )
    _logger.info(
        "[assembly] built AgentCore for %s "
        "(providers=%s, prompt=%d chars, skills=%s, memory_namespace=%s, "
        "scheduler=%s)",
        spec.name,
        list(spec.provider_chain),
        len(system_prompt),
        sorted(skill_prompts.keys()),
        spec.memory_namespace,
        spec.use_scheduler,
    )
    return core


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


def normalize_skill_slice(slice_: Iterable[str] | None) -> tuple[str, ...]:
    """Превратить произвольный iterable в frozen-совместимый tuple.

    Используется нодами при чтении ``skill_slice`` из YAML, где
    ожидается list — frozen-контракт требует tuple, иначе
    ``dataclasses.replace`` в тестах выдаёт ``TypeError``.
    """
    if not slice_:
        return ()
    return tuple(slice_)


__all__ = [
    "AgentSpec",
    "build_agent",
    "build_llm_chain",
    "load_system_prompt",
    "load_skill_prompts",
    "normalize_skill_slice",
]
