# -*- coding: utf-8 -*-
"""ADR-0134 §2.2 — provider-aware guard ``_validate_voice_distinctness``.

Зачем эти тесты существуют
==========================
ADR-0134 (PR #3020) фиксирует архитектурное решение: voice distinctness —
это инвариант, а не «best effort». Раньше в ``gen_night_marathon.py`` этот
инвариант не проверялся вовсе: сценарий мог уехать с 4 голосами A/A/A/A
(или парой 0.95/0.95), и identify() потом честно сливал их в один
профиль — акт 3 (диаризация) становился нечитаемым. ``_validate_voice_distinctness``
— это «тонкий» helper, который:

1. собирает уникальные ``voice=`` из шагов;
2. поднимает ``evidence/tts-voice-distinctness-*/<provider>_voices.json``;
3. для каждой пары (и для одиночного голоса с самопарой ``voice|voice``)
   смотрит ``inter_voice_max_cos[voice|other]`` (или самопары) и сверяет
   с ``--voice-distinctness-threshold``;
4. при нарушении → ``SystemExit(2)`` (fail-fast на этапе генерации);
5. при отсутствии evidence-файла → WARNING в stderr, НЕ raise (greenfield).

Это «тестовый контракт» (ADR-0134 §2.2 + комментарий товарища Шифу:
helper может быть на тестовой стороне), а backend потом просто вызовет
его из ``emit()`` после подбора шагов. Тесты ниже — единственный
гарант того, что helper действительно ловит то, что обещает.

Корпус evidence здесь — синтетика (``inter_voice_max_cos={'A|A': 0.95}``):
production-evidence в ``evidence/tts-voice-distinctness-2026-09-22/``
использовать НЕЛЬЗЯ — оно для других провайдеров и формата; для guard'а
достаточно подменять ``loader`` через monkeypatch и не трогать диск.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_E2E = REPO_ROOT / "scripts" / "e2e"
sys.path.insert(0, str(SCRIPTS_E2E))

from gen_night_marathon import (  # noqa: E402
    _validate_voice_distinctness,
    step,
)


def _step(label: str, voice: str, text: str = "...") -> dict:
    """Синтетический шаг в формате ``step()``. Текст не важен — guard
    читает только ``step["voice"]``."""
    return step(label=label, voice=voice, text=text)


def _fake_loader(data):
    """Замыкание-loader'а: при вызове с любым provider возвращает ``data``."""
    def loader(provider: str):
        return data
    return loader


# ── Test 1: 4 идентичных голоса → raise SystemExit(2) ──────────────────────
#
# Сценарий: в шагах выставлен ``voice='A'`` 4 раза подряд. Никаких
# разных голосов нет — значит identify() не сможет различить шаги даже
# теоретически. helper должен поймать это по самопарной записи в
# evidence (``A|A``). Это закрывает дыру «все 4 шага одним голосом».
class TestFourIdenticalVoicesRaise:
    def test_four_identical_voices_raises(self, capsys):
        """4 шага с voice='A' + evidence ``A|A=0.95`` → SystemExit(2).

        Контракт ошибки: в stderr попадает диагностика, содержащая
        *имя голоса* ('A') и *слово про similarity* (sim/cos). Это
        требование ADR-0134 §2.2 «генерация должна падать с понятной
        диагностикой, а не молча». Тест явно НЕ проверяет exit-code
        == 0 ни в каком виде — только что SystemExit(2) сработал.
        """
        steps = [_step(f"s{i}", "A") for i in range(4)]
        data = {"inter_voice_max_cos": {"A|A": 0.95}}
        with pytest.raises(SystemExit) as exc:
            _validate_voice_distinctness(
                steps, provider="minimax", threshold=0.8,
                loader=_fake_loader(data),
            )
        assert exc.value.code == 2
        # Диагностика в stderr.
        captured = capsys.readouterr()
        assert "A" in captured.err
        assert ("sim" in captured.err) or ("cos" in captured.err)


# ── Test 2: yandex без файла → WARNING, не raise ────────────────────────────
#
# Сценарий: greenfield-провайдер, evidence-файла ещё нет. helper
# НЕ должен падать — должен предупредить и пропустить проверку
# (см. ADR-0134 §5 trade-off #1: «не наказывать за отсутствие
# замера — это блокирует новых провайдеров»).
class TestYandexGreenfieldWarnsNoData:
    def test_yandex_warns_when_no_data(self, capsys):
        """provider='yandex' + loader возвращает None → WARNING в stderr,
        НЕ raise."""
        steps = [_step("a", "y_voice1"), _step("b", "y_voice2")]
        with pytest.warns(None) if False else _NullContext():
            try:
                result = _validate_voice_distinctness(
                    steps, provider="yandex", threshold=0.8,
                    loader=lambda p: None,
                )
            except SystemExit as e:  # pragma: no cover — guardrail
                pytest.fail(
                    f"greenfield-провайдер не должен raise: SystemExit({e.code})"
                )
        assert result["missing"] is True
        assert result["ok"] is True
        assert result["violations"] == []
        captured = capsys.readouterr()
        # WARNING должен содержать имя провайдера — чтобы оператор
        # понял, КАКОЙ провайдер надо замерить.
        assert "yandex" in captured.err
        assert "WARNING" in captured.err


class _NullContext:
    """Пустой context-manager, чтобы не тащить ``contextlib.suppress``
    в сигнатуру и не путать читателя."""
    def __enter__(self): return self
    def __exit__(self, *a): return False


# ── Test 3: --voice-distinctness-threshold=0.5 + пара 0.6 → raise ────────────
#
# Сценарий: порог жёстче (0.5), у пары голосов замерено 0.6 — выше
# порога. helper должен поднять. Это проверка, что kwarg ``threshold``
# действительно доходит до сравнения (а не зашит в helper).
class TestThresholdOverride:
    def test_threshold_override_takes_effect(self, capsys):
        """threshold=0.5 + пара (A,B) с max_cos=0.6 → SystemExit(2).

        Дополнительно: при threshold=0.7 эта же пара должна пройти
        (sanity: helper умеет отличать «выше порога» от «ниже»).
        """
        steps = [
            _step("a1", "A"),
            _step("a2", "A"),
            _step("b1", "B"),
            _step("b2", "B"),
        ]
        data = {
            "inter_voice_max_cos": {
                "A|B": 0.6, "B|A": 0.6,  # обе стороны на всякий случай
            }
        }
        with pytest.raises(SystemExit) as exc:
            _validate_voice_distinctness(
                steps, provider="minimax", threshold=0.5,
                loader=_fake_loader(data),
            )
        assert exc.value.code == 2

        # sanity: с порогом 0.7 — должно пройти (0.6 < 0.7).
        result = _validate_voice_distinctness(
            steps, provider="minimax", threshold=0.7,
            loader=_fake_loader(data),
        )
        assert result["ok"] is True
        assert result["violations"] == []


# ── Test 4: все пары ниже порога → успех ────────────────────────────────────
#
# Сценарий: 4 разных голоса, все пары ниже threshold — guard
# пропускает (возвращает ``{"ok": True, ...}``). Это sanity-тест
# против «всегда raise» — bug, который выглядел бы как «мы
# заблокировали вообще всю генерацию».
class TestNoFalsePositive:
    def test_no_falsely_passing_pair(self):
        """provider='minimax', все пары ниже threshold → ok=True,
        violations=[], missing=False."""
        steps = [
            _step("a1", "A"), _step("a2", "A"),
            _step("b1", "B"), _step("b2", "B"),
            _step("c1", "C"), _step("c2", "C"),
            _step("d1", "D"), _step("d2", "D"),
        ]
        # Все возможные пары < 0.5.
        data = {
            "inter_voice_max_cos": {
                "A|B": 0.10, "A|C": 0.20, "A|D": 0.15,
                "B|C": 0.05, "B|D": 0.25, "C|D": 0.30,
            }
        }
        result = _validate_voice_distinctness(
            steps, provider="minimax", threshold=0.5,
            loader=_fake_loader(data),
        )
        assert result["ok"] is True
        assert result["violations"] == []
        assert result["missing"] is False
        assert sorted(result["voices"]) == ["A", "B", "C", "D"]


# ── Test 5 (доп.): ключ ``a|b`` или ``b|a`` — helper пробует оба ───────────
#
# Сценарий: замерщик мог сохранить только одну сторону пары. helper
# должен это учитывать. Это часть контракта ADR-0134 §2.2.
class TestAsymmetricKeysSupported:
    def test_only_one_side_of_pair_key_is_enough(self):
        """В evidence только ключ ``B|A`` (нет ``A|B``). helper должен
        использовать значение и raise'нуть, если оно >= threshold."""
        steps = [_step("a1", "A"), _step("b1", "B")]
        data = {"inter_voice_max_cos": {"B|A": 0.9}}  # только обратная сторона
        with pytest.raises(SystemExit) as exc:
            _validate_voice_distinctness(
                steps, provider="minimax", threshold=0.8,
                loader=_fake_loader(data),
            )
        assert exc.value.code == 2


# ── Test 6 (доп.): пара отсутствует в evidence → пропускаем ────────────────
#
# Сценарий: замер не покрыл пару ``A|B``. helper НЕ должен считать
# это «неопределённо плохим» — иначе генерация падала бы на любом
# непокрытом голосе (см. ADR §5: «нет данных ⇒ fail» будет отдельным
# PR, когда будут полные матрицы). Сейчас — пропускаем.
class TestMissingPairInEvidenceIsSkipped:
    def test_unmeasured_pair_is_not_violation(self):
        steps = [_step("a1", "A"), _step("b1", "B"), _step("c1", "C")]
        data = {"inter_voice_max_cos": {"A|B": 0.1}}  # только одна пара
        result = _validate_voice_distinctness(
            steps, provider="minimax", threshold=0.5,
            loader=_fake_loader(data),
        )
        assert result["ok"] is True
        assert result["violations"] == []
