"""``RetryPolicy`` — один класс на харнес, а не по копии на провайдера.

Контекст (карточка W6-1, скан дублей
``docs/plans/2026-08-30-dedup-wave3-handoff.md`` §3.2). Копий было две,
нормализованный исходник совпадал символ в символ:

* ``providers/deepseek.py:99``  — её ре-экспортит ``providers/__init__``
  и из неё берёт политику ``providers/mimo.py``;
* ``providers/minimax.py:128``  — её ре-экспортит ``tts/minimax_tts.py``
  (и через него ``tts/__init__``), из неё же импортирует
  ``test_integration_offline.py``.

Расхождение уже было — не в поведении, а в идентичности: до слияния
``rob_box_harness.providers.RetryPolicy`` (версия deepseek) и
``rob_box_harness.tts.RetryPolicy`` (версия minimax) были РАЗНЫМИ
объектами. Спасала только структурная совместимость: любой
``isinstance``-чек или сравнение классов между LLM- и TTS-половиной
молча давал бы False. ``test_minimax_tts.py::…retry_policy_is_shared``
проверял идентичность только внутри minimax-ветки и поэтому был зелёным.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from rob_box_harness.providers import RetryPolicy as PackageRetryPolicy
from rob_box_harness.providers.deepseek import RetryPolicy as DeepSeekRetryPolicy
from rob_box_harness.providers.minimax import RetryPolicy as MiniMaxRetryPolicy
from rob_box_harness.providers.retry import RetryPolicy
from rob_box_harness.tts import RetryPolicy as TTSRetryPolicy

_PROVIDERS = Path(__file__).resolve().parents[1] / "rob_box_harness" / "providers"


def test_every_import_path_yields_the_same_class() -> None:
    """LLM- и TTS-половина должны говорить об ОДНОМ классе.

    До слияния это был единственный настоящий дефект в паре: поведение
    совпадало, а объекты — нет.
    """
    assert DeepSeekRetryPolicy is RetryPolicy
    assert MiniMaxRetryPolicy is RetryPolicy
    assert PackageRetryPolicy is RetryPolicy
    assert TTSRetryPolicy is RetryPolicy


def test_defaults_are_unchanged() -> None:
    """Дефолты — те же, что несли обе копии."""
    policy = RetryPolicy()
    assert (policy.max_attempts, policy.backoff_base, policy.backoff_jitter) == (
        3,
        0.5,
        0.25,
    )


def test_validation_is_unchanged() -> None:
    with pytest.raises(ValueError):
        RetryPolicy(max_attempts=0)
    with pytest.raises(ValueError):
        RetryPolicy(backoff_base=-0.1)
    with pytest.raises(ValueError):
        RetryPolicy(backoff_jitter=-0.1)


def test_backoff_is_unchanged() -> None:
    policy = RetryPolicy(backoff_base=0.5, backoff_jitter=0.0)
    assert policy.delay_for(0) == 0.0
    assert policy.delay_for(1) == 0.5
    assert policy.delay_for(2) == 1.0
    assert policy.delay_for(3) == 2.0


def _toplevel_classes(path: Path) -> set:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    return {node.name for node in tree.body if isinstance(node, ast.ClassDef)}


def test_no_provider_declares_its_own_copy() -> None:
    """Предохранитель: третья копия должна ронять тест."""
    offenders = [
        path.name
        for path in sorted(_PROVIDERS.glob("*.py"))
        if path.name != "retry.py" and "RetryPolicy" in _toplevel_classes(path)
    ]
    assert not offenders, (
        "RetryPolicy объявлен заново — он живёт в "
        "rob_box_harness.providers.retry и больше нигде: " + ", ".join(offenders)
    )
