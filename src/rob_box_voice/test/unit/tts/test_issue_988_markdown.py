"""test_issue_988_markdown.py — TTS must strip Markdown before synthesis
(issue #988).

E2E v44: the final text contained ``*Жил да был енот весёлый,*`` and
TTS read the literal ``*`` as «звёздочка» — the recording sounded like
«звёздочка звёздочка лалала». The fix sanitises text in
``tts_text_clean.speech_text`` — the single chokepoint through which ALL
TTS requests pass (speakers, headset, preview, pre-synthesis). Since
ADR-0160 §2.3 (#4239) the request carries plain text, not SSML.
"""

from __future__ import annotations

from rob_box_voice.tts_text_clean import speech_text


class TestSpeechTextStripsMarkdown:
    def test_poem_markdown_stripped(self):
        text = speech_text(
            "А вот и песенка про енотика!\n\n"
            "*Жил да был енот весёлый,*\n"
            "*полосатый, озорной.*"
        )
        assert "*" not in text
        assert "Жил да был енот весёлый," in text
        assert "полосатый, озорной." in text

    def test_bold_heading_and_quote_removed(self):
        text = speech_text("**Повторю слова:**\n> Жил-был енотик\n# Песня")
        assert "**" not in text
        assert text.startswith("Повторю слова:")
        assert "Жил-был енотик" in text
        assert "Песня" in text

    def test_model_tags_removed_but_words_kept(self):
        assert speech_text("<Текст>Привет робот</Текст>") == "Привет робот"

    def test_plain_text_unchanged(self):
        assert speech_text("Привет! Как дела?") == "Привет! Как дела?"

    def test_lone_asterisk_multiplication_preserved(self):
        # Not markdown emphasis — keep the multiplication sign.
        assert speech_text("2 * 3 = 6") == "2 * 3 = 6"
