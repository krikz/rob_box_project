#!/usr/bin/env python3
"""
speech_formatter.py - Speech Text Formatting for TTS

Purpose:
    Formats text for TTS output, including accent placement,
    SSML processing, and special character handling.

Public Interface:
    - SpeechFormatter: Main class for text formatting

Example:
    >>> from rob_box_voice.core.speech_formatter import SpeechFormatter
    >>> formatter = SpeechFormatter()
    >>> formatted = formatter.format_for_tts('Привет робот')
    >>> print(formatted)  # With accents added

Tests:
    See: test/unit/core/test_speech_formatter.py

Dependencies:
    - AccentReplacer from scripts (for accent placement)
    - Standard library only
"""

import os
import re
from typing import Optional
from pathlib import Path


class SpeechFormatter:
    """
    Formats text for TTS output.
    
    This class handles accent placement, SSML processing,
    and text cleanup for natural speech synthesis.
    
    Attributes:
        accent_replacer: AccentReplacer instance for adding accents
        enable_accents: Whether to add accents to text
    
    Methods:
        format_for_tts: Format text for TTS output
        add_accents: Add Russian accents to text
        clean_for_speech: Clean text for speech synthesis
        wrap_in_ssml: Wrap text in SSML tags
        extract_from_ssml: Extract text from SSML
        is_ssml: Check if text is SSML
    """
    
    def __init__(self, enable_accents: bool = True, accent_config_path: Optional[str] = None):
        """
        Initialize SpeechFormatter.
        
        Args:
            enable_accents: Enable accent placement (default: True)
            accent_config_path: Path to accent configuration file (optional)
        """
        self.enable_accents = enable_accents
        self.accent_replacer = None
        
        if enable_accents:
            self._init_accent_replacer(accent_config_path)
    
    def _init_accent_replacer(self, config_path: Optional[str] = None):
        """Initialize accent replacer with fallback handling."""
        try:
            # Import from scripts directory
            import sys
            scripts_path = Path(__file__).parent.parent / "scripts"
            if str(scripts_path) not in sys.path:
                sys.path.insert(0, str(scripts_path))
            
            from accent_replacer import AccentReplacer
            self.accent_replacer = AccentReplacer(config_path=config_path)
        except ImportError:
            # Fallback: accent_replacer module unavailable on this platform.
            # Keep enable_accents=True so callers see the intent; add_accents
            # is a no-op when accent_replacer is None (checked in format_for_tts).
            self.accent_replacer = None
    
    def format_for_tts(self, text: str, add_ssml: bool = False) -> str:
        """
        Format text for TTS output.
        
        This is the main method that applies all formatting:
        - Cleans text
        - Adds accents (if enabled)
        - Wraps in SSML (if requested)
        
        Args:
            text: Input text to format
            add_ssml: Whether to wrap in SSML tags
        
        Returns:
            Formatted text ready for TTS
        """
        # Clean text first
        cleaned = self.clean_for_speech(text)
        
        # Add accents if enabled
        if self.enable_accents and self.accent_replacer:
            cleaned = self.add_accents(cleaned)
        
        # Wrap in SSML if requested
        if add_ssml and not self.is_ssml(cleaned):
            cleaned = self.wrap_in_ssml(cleaned)
        
        return cleaned
    
    def add_accents(self, text: str) -> str:
        """
        Add Russian accents to text.
        
        Args:
            text: Text without accents
        
        Returns:
            Text with accents added
        """
        if not self.accent_replacer:
            return text
        
        # If text contains SSML, process only the content
        if self.is_ssml(text):
            # Extract text, add accents, rewrap
            extracted = self.extract_from_ssml(text)
            with_accents = self.accent_replacer.add_accents(extracted)
            return self.wrap_in_ssml(with_accents)
        
        return self.accent_replacer.add_accents(text)
    
    def clean_for_speech(self, text: str) -> str:
        """
        Clean text for speech synthesis.
        
        Removes or replaces characters that don't sound good in TTS.
        
        Args:
            text: Raw text
        
        Returns:
            Cleaned text
        """
        # Remove extra whitespace
        text = " ".join(text.split())
        
        # Remove markdown formatting
        text = re.sub(r'\*\*(.+?)\*\*', r'\1', text)  # **bold**
        text = re.sub(r'\*(.+?)\*', r'\1', text)      # *italic*
        text = re.sub(r'__(.+?)__', r'\1', text)      # __bold__
        text = re.sub(r'_(.+?)_', r'\1', text)        # _italic_
        
        # Remove URLs
        text = re.sub(r'https?://[^\s]+', '', text)
        
        # Clean up multiple punctuation
        text = re.sub(r'\.{3,}', '...', text)  # Multiple dots -> ellipsis
        text = re.sub(r'!{2,}', '!', text)     # Multiple exclamations
        text = re.sub(r'\?{2,}', '?', text)    # Multiple questions
        
        # Remove extra spaces again after cleanup
        text = " ".join(text.split())
        
        return text.strip()
    
    def wrap_in_ssml(self, text: str) -> str:
        """
        Wrap text in SSML speak tags.
        
        Args:
            text: Text to wrap
        
        Returns:
            Text wrapped in <speak>...</speak>
        """
        if self.is_ssml(text):
            return text
        
        return f"<speak>{text}</speak>"
    
    def extract_from_ssml(self, ssml_text: str) -> str:
        """
        Extract plain text from SSML.
        
        Args:
            ssml_text: SSML formatted text
        
        Returns:
            Plain text without SSML tags
        """
        # Remove speak tags
        text = re.sub(r'<speak>(.*?)</speak>', r'\1', ssml_text, flags=re.DOTALL)
        
        # Remove other SSML tags but keep content
        text = re.sub(r'<[^>]+>', '', text)
        
        return text.strip()
    
    def is_ssml(self, text: str) -> bool:
        """
        Check if text is SSML formatted.
        
        Args:
            text: Text to check
        
        Returns:
            True if text contains <speak> tags
        """
        return '<speak>' in text and '</speak>' in text
    
    def get_stats(self) -> dict:
        """
        Get formatter statistics.
        
        Returns:
            Dictionary with stats
        """
        stats = {
            'accents_enabled': self.enable_accents,
            'accent_replacer_loaded': self.accent_replacer is not None
        }
        
        if self.accent_replacer:
            stats['accent_dictionary'] = self.accent_replacer.get_stats()
        
        return stats


# Example usage
if __name__ == '__main__':
    # Create formatter
    formatter = SpeechFormatter()
    
    print("📊 Formatter stats:")
    import json
    print(json.dumps(formatter.get_stats(), indent=2, ensure_ascii=False))
    
    print("\n🧪 Formatting tests:")
    
    test_cases = [
        ("Привет! Я робот.", False),
        ("**Важно**: система работает.", False),
        ("Проверка https://example.com ссылки", False),
        ("<speak>Уже в SSML</speak>", False),
        ("Обычный текст", True),
    ]
    
    for text, add_ssml in test_cases:
        result = formatter.format_for_tts(text, add_ssml=add_ssml)
        print(f"\nВход: {text}")
        print(f"Выход: {result}")
        print(f"SSML: {add_ssml}")
