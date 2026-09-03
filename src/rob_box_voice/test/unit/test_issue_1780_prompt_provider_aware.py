"""Snapshot-тест мастер-промпта для issue #1780.

Контекст: issue #1780 расширил TTS-стек, и теперь активный провайдер
может быть как **yandex** (gRPC v3), так и **minimax** (T2A v2). У
них разные каталоги голосов (yandex-only «Алёна/Антон/Захар» vs
minimax-каталог «Russian_BrightHeroine/female-shaonv/...»), разные
поля prosody (yandex → Hints.pitch_shift + Hints.volume в LUFS,
minimax → voice_setting.pitch (int semitones) + voice_setting.vol
в [0.0, 10.0]) и разная поддержка SSML (yandex — да, minimax — нет,
берёт emotion/pitch/volume из TTSSettings).

Мастер-промпт (``master_prompt_compact.txt``) должен явно отражать
это различие, иначе LLM будет галлюцинировать «голос Алены» на
minimax-провайдере (repro: PR #1636 / mv01 fail-streak).

Что покрываем:

* **acceptance criteria пункта 4** task'а — «новый мастер-промпт
  содержит блок про различия провайдеров» (простая проверка
  подстроки).
* Защита от регрессии, при которой правило
  ``RULE #VOICE-CROSS-PROVIDER`` будет случайно удалено при слиянии
  изменений в промпте.

Что НЕ покрываем (уже есть в test_mv01_voice_by_name_cross_provider.py):

* конкретные имена голосов («Алёна», «Антон», «Захар»),
* детальные формулировки «не вызывай set_voice с недоступным»,
* fallback на default_voice.

Refs:

* issue #1780 — prosody forwarding
* PR #1636 — repro set_voice(alena) на minimax
* test_mv01_voice_by_name_cross_provider.py — детальный guardrail
"""

from __future__ import annotations

from pathlib import Path


MASTER_PROMPT = (
    Path(__file__).resolve().parents[2]
    / "prompts"
    / "master_prompt_compact.txt"
)


def _read_prompt() -> str:
    return MASTER_PROMPT.read_text(encoding="utf-8")


class TestMasterPromptProviderAware:
    """Мастер-промпт должен явно отражать различия TTS-провайдеров."""

    def test_prompt_contains_voice_cross_provider_rule(self) -> None:
        """``RULE #VOICE-CROSS-PROVIDER`` блок существует.

        Это структурный маркер — без него LLM не имеет инструкции
        «что делать, когда юзер просит голос, завязанный на
        другой провайдер».
        """
        content = _read_prompt()
        assert "RULE #VOICE-CROSS-PROVIDER" in content, (
            "master_prompt_compact.txt must contain RULE #VOICE-CROSS-PROVIDER "
            "block to teach LLM about provider-specific voice names "
            "(issue #1780 / PR #1636 / mv01 fail-streak). "
            "If you renamed the rule, update both this test and "
            "test_mv01_voice_by_name_cross_provider.py."
        )

    def test_prompt_mentions_both_yandex_and_minimax_providers(self) -> None:
        """Мастер-промпт упоминает оба провайдера: yandex и minimax.

        Без явного упоминания LLM не понимает, что у неё может быть
        выбор провайдера и что голоса привязаны к провайдеру.
        """
        content = _read_prompt()
        content_lower = content.lower()
        assert "yandex" in content_lower, (
            "master_prompt_compact.txt must mention 'yandex' as a TTS provider "
            "(Российские голоса Алёна/Антон/Захар — yandex-only)."
        )
        assert "minimax" in content_lower, (
            "master_prompt_compact.txt must mention 'minimax' as a TTS provider "
            "(emotion/pitch/volume forwarding реализован именно там, "
            "issue #1780)."
        )

    def test_prompt_teaches_checking_active_provider(self) -> None:
        """Мастер-промпт учит LLM проверять активный ``[TTS] provider:``.

        Cross-provider awareness = «сначала посмотри какой провайдер
        сейчас активен, потом решай что делать».
        """
        content = _read_prompt()
        assert "[TTS] provider" in content, (
            "master_prompt_compact.txt must reference '[TTS] provider' tag — "
            "LLM uses it to know which provider is currently active."
        )

    def test_prompt_has_multiple_distinct_voice_catalogue_examples(self) -> None:
        """Мастер-промпт явно даёт примеры из обоих каталогов.

        Защита от регрессии, при которой из RULE #VOICE выпилили все
        примеры (как в pre-#1219 версии — там был только один набор
        имён, и LLM путалась).
        """
        content = _read_prompt()
        # Yandex-only примеры (русские имена каталога yandex).
        # Минимум одно имя должно быть упомянуто как пример голоса.
        yandex_markers = ("Алён", "Антон", "Захар", "Филипп", "ermil", "jane", "madirus", "omazh")
        # MiniMax-каталог (русские/английские имена) — должен быть
        # упомянут в RULE #VOICE mapping hints.
        minimax_markers = (
            "Russian_BrightHeroine",
            "Russian_ReliableMan",
            "Russian_AmbitiousWoman",
            "female-shaonv",
            "male-qn-qingse",
        )
        content_lower = content.lower()

        assert any(m.lower() in content_lower for m in yandex_markers), (
            "master_prompt_compact.txt must mention at least one yandex-only "
            "voice name (Алён/Антон/Захар/Филипп/ermil/jane/madirus/omazh) "
            "as a cross-provider marker."
        )
        assert any(m in content for m in minimax_markers), (
            "master_prompt_compact.txt must mention at least one minimax "
            "voice name (Russian_BrightHeroine / Russian_ReliableMan / "
            "Russian_AmbitiousWoman / female-shaonv / male-qn-qingse) so LLM "
            "knows the catalogue of the active provider."
        )

    def test_prompt_distinguishes_provider_specific_voice_anchors(self) -> None:
        """Мастер-промпт явно говорит, что имена вроде «Алёна» —
        yandex-only prompt-anchors, а НЕ общий каталог.

        Это последняя строчка RULE #VOICE-CROSS-PROVIDER («имена вроде
        «алёна/Алёна» — это yandex-only prompt-anchors, не каталог
        всех возможных голосов»). Без неё LLM считает «алёна» общим
        именем и пытается его вызвать на любом провайдере.
        """
        content = _read_prompt()
        content_lower = content.lower()
        # Хотя бы одна из формулировок.
        anchor_phrases = [
            "yandex-only prompt-anchor",
            "yandex-only",
            "привязаны к конкретному tts-провайдеру",
            "привязаны к конкретному провайдеру",
            "привязаны к конкретному tts",
            "привязаны к конкретному tts провайдер",
        ]
        assert any(p in content_lower for p in anchor_phrases), (
            "master_prompt_compact.txt must explicitly state that voice "
            "names like «Алёна» are provider-specific anchors, not a "
            "universal catalogue. This is the closing line of "
            "RULE #VOICE-CROSS-PROVIDER."
        )