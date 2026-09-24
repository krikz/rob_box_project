"""Issue #2969 — сид вариативности аранжировки в ``core.harmonize``.

Живой лог 24.09.2026 (два DJ-сета той же темы, 10:55 и 11:05 UTC): у The
Next Episode бас (``p1``) и пэд (``p3``) вышли побайтно идентичными между
сетами, а рисунок ударных не менялся все 15 треков сессии (4 каркаса
бочки/малого, 3 — хэтов, независимо от темы). Причина — ``harmonize()``
полностью детерминирована от нот темы: ``bass_style``/``pad_style``/
``drum_style`` на ``auto`` всегда строят один и тот же вариант.

Контракт этого PR (ADR-0018 «чинить системно», товарищ Шифу, 24.09):

* без ``seed`` (``None``, по умолчанию) — байт-в-байт прежнее поведение
  (``test_arranger_golden`` не имеет права измениться);
* заданный ``seed`` детерминированно выбирает конкретный вариант
  ``auto``-ручек: тот же сид у ЛЮБОЙ мелодии всегда даёт тот же результат
  (воспроизводимость), разные сиды у одной мелодии — разный бас/пэд/
  ударные (issue #2969 acceptance: «сид зависит от сета/номера трека, а
  не только от песни»);
* явно заданный ``bass_style``/``pad_style``/``drum_style`` (не ``auto``)
  сид не трогает — «явная ручка всегда побеждает» (тот же контракт, что у
  пресетов ADR-0132 PR-7);
* механизм общий для ЛЮБОЙ темы — не хардкод под конкретную песню. Один
  regression-тест на The Next Episode (ниже) — только чтобы не отвалился
  ИМЕННО тот живой баг, не место для новой логики.
"""

from __future__ import annotations

import pytest

from rob_box_mcp_tools.core import harmonize as hz
from rob_box_mcp_tools.core.rtttl_compose import melody_to_compose_params, rtttl_to_melody

#: Темы разной плотности/лада — сид обязан работать на ЛЮБОЙ из них, не
#: только на одной конкретной мелодии.
_THEMES = {
    "dense_major": "dense:d=8,o=5,b=120:c,d,e,f,g,a,b,c6,b,a,g,f,e,d,c,d",
    "sparse_minor": "sparse:d=2,o=5,b=90:c,g,e,c",
    "syncopated": "sync:d=4,o=5,b=100:8c,8d,e,8f,8g,a,8b,8c6",
}


def _harmony(rtttl: str, seed: int | None = None, **knobs):
    options = hz.HarmonizeOptions(seed=seed, **knobs)
    return melody_to_compose_params(rtttl_to_melody(rtttl), options=options)["harmony"]


class TestSeedIsOptOutSafe:
    """``seed=None`` — прежнее поведение, ноль риска регресса без него."""

    @pytest.mark.parametrize("rtttl", _THEMES.values())
    def test_no_seed_matches_explicit_none(self, rtttl):
        a = _harmony(rtttl)
        b = _harmony(rtttl, seed=None)
        assert (a.bass, a.pad, a.drums, a.hats) == (b.bass, b.pad, b.drums, b.hats)

    @pytest.mark.parametrize("rtttl", _THEMES.values())
    def test_no_seed_matches_pre_2969_auto_defaults(self, rtttl):
        """auto без сида — ровно то, что строили ``_STYLE_DRUMS``/``_bass_shape`` раньше."""
        default = _harmony(rtttl)
        explicit_auto = _harmony(rtttl, bass_style="auto", pad_style="auto")
        assert (default.bass, default.pad) == (explicit_auto.bass, explicit_auto.pad)


class TestSeedVariesAnyMelody:
    """Механизм общий: работает на любой теме, не только на одной песне."""

    @pytest.mark.parametrize("rtttl", _THEMES.values())
    def test_different_seeds_change_bass_or_pad_or_drums(self, rtttl):
        baseline = _harmony(rtttl, seed=1)
        different = 0
        for seed in range(2, 12):
            other = _harmony(rtttl, seed=seed)
            if (baseline.bass, baseline.pad, baseline.drums, baseline.hats) != (
                other.bass, other.pad, other.drums, other.hats,
            ):
                different += 1
        # Не все 10 сидов обязаны отличаться от seed=1 (варианты циклятся
        # по малому числу вариантов и совпадения возможны), но подавляющее
        # большинство — обязано: иначе сид ничего не варьирует.
        assert different >= 5

    @pytest.mark.parametrize("rtttl", _THEMES.values())
    def test_same_seed_is_reproducible(self, rtttl):
        """Тот же сид — тот же результат КАЖДЫЙ раз (не системный ГСЧ)."""
        first = _harmony(rtttl, seed=42)
        second = _harmony(rtttl, seed=42)
        assert (first.bass, first.pad, first.drums, first.hats) == (
            second.bass, second.pad, second.drums, second.hats,
        )

    @pytest.mark.parametrize("rtttl", _THEMES.values())
    def test_drum_pattern_is_one_of_the_genre_skeletons(self, rtttl):
        """Сид переключает ``drum_style`` на конкретный жанровый каркас,
        а не оставляет старый auto-рисунок по гистограмме темы."""
        seen = set()
        for seed in range(0, 20):
            harmony = _harmony(rtttl, seed=seed)
            seen.add(harmony.decisions["drum_style"])
        assert seen <= set(hz._SEED_DRUM_STYLES)
        assert len(seen) >= 3


class TestSeedRespectsExplicitKnobs:
    """«Явная ручка всегда побеждает» — сид меняет только auto."""

    def test_explicit_bass_style_is_not_overridden(self):
        rtttl = _THEMES["dense_major"]
        for seed in (1, 2, 3):
            harmony = _harmony(rtttl, seed=seed, bass_style="pedal")
            assert harmony.decisions["knob_bass_style"] == "pedal"

    def test_explicit_pad_style_is_not_overridden(self):
        rtttl = _THEMES["dense_major"]
        for seed in (1, 2, 3):
            harmony = _harmony(rtttl, seed=seed, pad_style="off")
            assert harmony.decisions["knob_pad_style"] == "off"

    def test_explicit_drum_style_is_not_overridden(self):
        rtttl = _THEMES["dense_major"]
        for seed in (1, 2, 3):
            params = melody_to_compose_params(
                rtttl_to_melody(rtttl),
                drum_style="march",
                options=hz.HarmonizeOptions(seed=seed),
            )
            assert params["harmony"].decisions["drum_style"] == "march"


class TestSeedValidation:
    def test_non_int_seed_is_rejected(self):
        with pytest.raises(ValueError):
            hz.HarmonizeOptions(seed="7")  # type: ignore[arg-type]

    def test_bool_seed_is_rejected(self):
        with pytest.raises(ValueError):
            hz.HarmonizeOptions(seed=True)  # type: ignore[arg-type]


class TestNextEpisodeRegression:
    """Regression-тест ИМЕННО живого бага (issue #2969) — не общая логика.

    RTTTL — синтетическая тема сопоставимой формы (G-funk синкопа), не
    настоящие ноты трека: важно воспроизвести СИМПТОМ (повтор сета даёт
    тот же p1/p3/d1), а не зашить конкретную песню в тест общего
    механизма.
    """

    _RTTTL = "nextep:d=8,o=4,b=94:c,e,g,8c5,a,f,8d,c,e,g,8c5,a,f,8d,c"

    def test_replaying_the_same_set_gives_a_different_bass_pad_and_drums(self):
        set1_seed = 1 * 100 + 2  # сет 1, трек 2 (как The Next Episode в логе)
        set2_seed = 2 * 100 + 2  # сет 2, тот же номер трека — другой сет

        set1 = _harmony(self._RTTTL, seed=set1_seed)
        set2 = _harmony(self._RTTTL, seed=set2_seed)

        assert (set1.bass, set1.pad, set1.drums, set1.hats) != (
            set2.bass, set2.pad, set2.drums, set2.hats,
        )
