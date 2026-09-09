"""Два TTS-реестра — это два СЛОЯ, и сводить их нельзя.

Карточка W6-2 («понять, какой канонический») ставила вопрос неверно.
Канонического нет: ``rob_box_llm.tts_provider_registry`` и
``rob_box_harness.tts.registry`` — верхний и нижний уровень одного
намеренного двухуровневого дизайна:

* upstream (``rob_box_llm``) — библиотечный слой. Конфиг — обычный
  ``Mapping``, ``resolve`` кидает ``KeyError``, билдер отдаёт СЫРОЙ
  ``rob_box_llm.providers.minimax_tts.MiniMaxTTSProvider``.
* harness (``rob_box_harness.tts.registry``) — слой фреймворка. Конфиг —
  типизированный ``TTSConfig``, ``resolve`` кидает
  ``ProviderNotFoundError(port="tts")`` из таксономии ошибок харнеса,
  билдер отдаёт harness-обёртку (env-аутентификация, ретраи, кэш).

Слить их в один нельзя технически: ``rob_box_harness`` зависит от
``rob_box_llm``, обратной зависимости быть не должно. Дизайн
зафиксирован в ``.planning/phases/06-harness-p0-finalization/06-ARCHITECT-REVIEW.md``
(A.2.12, «двухуровневый registry ✅ Соответствует») — и там же записан
риск: «Каждое изменение MiniMax API = правка в 2 местах. **Дрейф
гарантирован**». Этот файл — предохранитель против того дрейфа.

Ровно та же развилка, что в §6 хендофа 2026-08-30 с вокальными
словарями (коммит ``fecae517``): совпадение имени и сходство тела —
гипотеза, а не диагноз; намерение написано в комментариях с обеих сторон.

Две группы тестов:

1. **Различия — намеренные.** Падают, если кто-то «поможет» и сведёт
   слои: тип конфига и тип исключения у ``resolve``.
2. **Общий контракт — общий.** Один и тот же набор прогоняется по обоим
   реестрам. Отрастит один поведение, которого нет у другого, — упадёт.

⚠️ Отдельная находка, НЕ чинится здесь (заведена отдельной задачей):
harness-овый ``_stable_config_hash`` перечисляет 9 полей ``TTSConfig``
из 12 и молча роняет ``api_key`` и ``extra``. Два конфига, различающиеся
только ключом, дают ОДИН ключ кэша, и второй вызов
``TTSProviderFactory.create`` вернёт провайдера, построенного на первом
ключе. Сейчас это латентно — ни один реестр не имеет продакшн-потребителя
(``tts_node.py:3299`` конструирует ``MiniMaxTTSProvider`` напрямую).
"""

from __future__ import annotations

from typing import Any, Mapping

import pytest

from rob_box_harness.config import TTSConfig
from rob_box_harness.errors import ProviderNotFoundError
from rob_box_harness.tts.registry import (
    TTSProviderFactory as HarnessFactory,
    TTSProviderRegistry as HarnessRegistry,
)
from rob_box_llm.tts_provider_registry import (
    TTSProviderFactory as UpstreamFactory,
    TTSProviderRegistry as UpstreamRegistry,
)


class _Sentinel:
    """То, что отдаёт фальшивый билдер. Провайдеров тут не строим."""

    def __init__(self, config: Any) -> None:
        self.config = config


def _fake_builder(config: Any) -> _Sentinel:
    return _Sentinel(config)


def _tts_config(**overrides: Any) -> TTSConfig:
    base: dict[str, Any] = {"provider": "minimax", "voice": "anton"}
    base.update(overrides)
    return TTSConfig(**base)


# ---------------------------------------------------------------------------
# 1. Различия между слоями — намеренные
# ---------------------------------------------------------------------------


def test_the_two_registries_are_different_classes() -> None:
    """Если однажды это станет одним классом — значит слои схлопнули."""
    assert HarnessRegistry is not UpstreamRegistry
    assert HarnessFactory is not UpstreamFactory


def test_upstream_resolve_raises_keyerror() -> None:
    """Библиотечный слой не знает про таксономию ошибок харнеса."""
    with pytest.raises(KeyError):
        UpstreamRegistry().resolve("nobody")


def test_harness_resolve_raises_the_harness_error_with_the_port() -> None:
    """Слой фреймворка обязан назвать порт — по нему разводят дашборды."""
    with pytest.raises(ProviderNotFoundError) as caught:
        HarnessRegistry().resolve("nobody")
    assert caught.value.port == "tts"
    assert not isinstance(caught.value, KeyError)


def test_upstream_factory_takes_a_mapping_config() -> None:
    """Ключ кэша строится по ``config.items()`` — значит нужен Mapping."""
    registry = UpstreamRegistry()
    registry.register("fake", _fake_builder)
    config: Mapping[str, Any] = {"voice": "anton"}
    try:
        built = UpstreamFactory.create("fake", config, registry)
        assert built.config is config
        # TTSConfig — не Mapping, у него нет .items()
        with pytest.raises(AttributeError):
            UpstreamFactory.create("fake", _tts_config(), registry)  # type: ignore[arg-type]
    finally:
        UpstreamFactory.reset_cache()


def test_harness_factory_takes_a_typed_tts_config() -> None:
    """Ключ кэша читает поля ``TTSConfig`` — значит dict не подойдёт."""
    registry = HarnessRegistry()
    registry.register("fake", _fake_builder)
    config = _tts_config()
    try:
        built = HarnessFactory.create("fake", config, registry)
        assert built.config is config
        # у dict нет атрибута .provider
        with pytest.raises(AttributeError):
            HarnessFactory.create("fake", {"provider": "minimax"}, registry)  # type: ignore[arg-type]
    finally:
        HarnessFactory.reset_cache()


def test_the_two_factories_do_not_share_a_cache() -> None:
    """``_cache`` — атрибут класса; у разных классов он разный.

    Иначе ключ ``("minimax", …)`` из одного слоя отдавал бы провайдера
    другого слоя.
    """
    upstream_registry = UpstreamRegistry()
    upstream_registry.register("minimax", _fake_builder)
    harness_registry = HarnessRegistry()
    harness_registry.register("minimax", _fake_builder)
    try:
        first = UpstreamFactory.create("minimax", {"voice": "anton"}, upstream_registry)
        second = HarnessFactory.create("minimax", _tts_config(), harness_registry)
        assert first is not second
    finally:
        UpstreamFactory.reset_cache()
        HarnessFactory.reset_cache()


# ---------------------------------------------------------------------------
# 2. Общий контракт — общий для обоих слоёв
# ---------------------------------------------------------------------------


_BOTH = pytest.mark.parametrize(
    "make_registry",
    [pytest.param(UpstreamRegistry, id="upstream"), pytest.param(HarnessRegistry, id="harness")],
)


@_BOTH
def test_register_then_resolve_returns_the_builder(make_registry) -> None:
    registry = make_registry()
    registry.register("fake", _fake_builder)
    assert registry.resolve("fake") is _fake_builder


@_BOTH
def test_duplicate_registration_is_refused(make_registry) -> None:
    """Молчаливая перезапись прятала бы ошибку рефакторинга."""
    registry = make_registry()
    registry.register("fake", _fake_builder)
    with pytest.raises(ValueError, match="already registered"):
        registry.register("fake", _fake_builder)


@_BOTH
def test_names_are_sorted(make_registry) -> None:
    registry = make_registry()
    for name in ("zulu", "alpha", "mike"):
        registry.register(name, _fake_builder)
    assert registry.names() == ["alpha", "mike", "zulu"]


@_BOTH
def test_unregister_is_a_no_op_on_a_missing_name(make_registry) -> None:
    """Тестовый хелпер не должен падать на повторной уборке."""
    registry = make_registry()
    registry.unregister("never-was-here")
    assert registry.names() == []


@_BOTH
def test_unregister_removes_the_builder(make_registry) -> None:
    registry = make_registry()
    registry.register("fake", _fake_builder)
    registry.unregister("fake")
    assert registry.names() == []
    with pytest.raises((KeyError, ProviderNotFoundError)):
        registry.resolve("fake")


@_BOTH
def test_resolve_error_lists_the_available_names(make_registry) -> None:
    """Сообщение об ошибке — то, что читает человек в логе робота."""
    registry = make_registry()
    registry.register("fake", _fake_builder)
    with pytest.raises((KeyError, ProviderNotFoundError)) as caught:
        registry.resolve("missing")
    message = str(caught.value)
    assert "missing" in message
    assert "fake" in message


def test_both_builtin_registrations_use_the_same_canonical_name() -> None:
    """Имя провайдера — общий словарь между слоями, вот оно и сверяется.

    Билдеры при этом РАЗНЫЕ: upstream строит сырой провайдер, harness —
    свою обёртку. Сами билдеры не зовём: им нужны ключи из окружения.
    """
    from rob_box_harness.tts.registry import register_builtin_tts_providers as harness_builtins
    from rob_box_llm.tts_provider_registry import register_builtin_tts_providers as upstream_builtins

    assert upstream_builtins().names() == harness_builtins().names() == ["minimax"]
