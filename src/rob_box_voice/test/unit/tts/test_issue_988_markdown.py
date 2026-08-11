"""test_issue_988_markdown.py — TTS must strip Markdown before synthesis
(issue #988).

E2E v44: the final text contained ``*Жил да был енот весёлый,*`` and
TTS read the literal ``*`` as «звёздочка» — the recording sounded like
«звёздочка звёздочка лалала». The fix sanitises text in
``TTSNode._extract_text_from_ssml`` — the single chokepoint through
which ALL TTS requests pass (both ``/voice/dialogue/response`` from
dialogue_node and ``/voice/tts/request`` from the ``speak_text`` MCP
tool).

Uses the same rclpy/grpc/torch mocks as the other tts unit tests.
"""

from __future__ import annotations

from test.unit.tts.conftest import _install_all_mocks

_install_all_mocks()

from rob_box_voice.tts_node import TTSNode  # noqa: E402


def _node() -> TTSNode:
    """TTSNode without __init__ — only the pure method under test."""
    return TTSNode.__new__(TTSNode)


class TestExtractTextFromSsmlStripsMarkdown:
    def test_poem_markdown_stripped(self):
        ssml = (
            "<speak>А вот и песенка про енотика! 🎶🦝\n\n"
            "*Жил да был енот весёлый,*\n"
            "*полосатый, озорной.*</speak>"
        )
        text = _node()._extract_text_from_ssml(ssml)
        assert "звёздочка" not in text.lower()
        assert "*" not in text
        assert "Жил да был енот весёлый," in text
        assert "полосатый, озорной." in text

    def test_bold_heading_and_quote_removed(self):
        ssml = (
            "<speak>**Повторю слова:**\n"
            "> Жил-был енотик\n"
            "# Песня</speak>"
        )
        text = _node()._extract_text_from_ssml(ssml)
        assert "**" not in text
        assert text.startswith("Повторю слова:")
        assert "Жил-был енотик" in text
        assert "Песня" in text

    def test_ssml_tags_removed_but_words_kept(self):
        ssml = "<speak><prosody pitch='high'>Привет робот</prosody></speak>"
        text = _node()._extract_text_from_ssml(ssml)
        assert text == "Привет робот"

    def test_plain_text_unchanged(self):
        ssml = "<speak>Привет! Как дела?</speak>"
        text = _node()._extract_text_from_ssml(ssml)
        assert text == "Привет! Как дела?"

    def test_lone_asterisk_multiplication_preserved(self):
        # Not markdown emphasis — keep the multiplication sign.
        ssml = "<speak>2 * 3 = 6</speak>"
        text = _node()._extract_text_from_ssml(ssml)
        assert text == "2 * 3 = 6"
